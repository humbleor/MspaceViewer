#include "../include/OutlierStageExperiment.h"

#include "Decoupling.h"
#include "Preparation.h"

#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <unordered_set>
#include <vector>

namespace
{
constexpr std::size_t pointCount = 3000;
constexpr std::size_t trialCount = 50;
constexpr double noiseSigma = 0.005;
constexpr double rotationSuccessThresholdDegrees = 3.0;
constexpr double translationSuccessThreshold = 0.5;
constexpr unsigned int experimentSeed = 20260906U;

constexpr std::array<double, 10> outlierRatios{
    0.50, 0.60, 0.70, 0.80, 0.90,
    0.95, 0.96, 0.97, 0.98, 0.99
};

struct SimulationInstance
{
    PointCloudPtr source;
    PointCloudPtr target;
    pcl::Correspondences correspondences;
    std::size_t inlierCount{ 0 };
    Eigen::Matrix4d groundTruth{ Eigen::Matrix4d::Identity() };
};

struct StageCounts
{
    std::size_t truePositives{ 0 };
    std::size_t falsePositives{ 0 };

    [[nodiscard]] std::size_t retainedCount() const noexcept
    {
        return truePositives + falsePositives;
    }
};

struct StageAccumulator
{
    std::uint64_t truePositiveSum{ 0 };
    std::uint64_t falsePositiveSum{ 0 };
    double timeSecondsSum{ 0.0 };
    std::vector<double> timeMilliseconds;
};

struct AccuracyAccumulator
{
    double rotationErrorDegreesSum{ 0.0 };
    double translationErrorSum{ 0.0 };
    std::size_t successCount{ 0 };
};

double median(std::vector<double> values)
{
    if (values.empty())
        return 0.0;
    std::sort(values.begin(), values.end());
    const std::size_t middle = values.size() / 2;
    return values.size() % 2 == 0
        ? (values[middle - 1] + values[middle]) * 0.5
        : values[middle];
}

void addRadialGaussianNoise(
    pcl::PointXYZ& point,
    std::mt19937& randomEngine,
    std::normal_distribution<double>& noiseDistribution,
    std::uniform_real_distribution<double>& angleDistribution)
{
    const double radialNoise = noiseDistribution(randomEngine);
    const double theta = angleDistribution(randomEngine);
    const double phi = angleDistribution(randomEngine);
    point.x += static_cast<float>(
        radialNoise * std::sin(phi) * std::cos(theta));
    point.y += static_cast<float>(
        radialNoise * std::sin(phi) * std::sin(theta));
    point.z += static_cast<float>(radialNoise * std::cos(phi));
}

SimulationInstance generateInstance(
    std::mt19937& randomEngine,
    double outlierRatio)
{
    std::uniform_real_distribution<double> coordinateDistribution(-1.0, 1.0);
    std::uniform_real_distribution<double> transformationDistribution(-1.0, 1.0);
    std::uniform_real_distribution<double> angleDistribution(-M_PI, M_PI);
    std::normal_distribution<double> noiseDistribution(0.0, noiseSigma);

    SimulationInstance instance;
    instance.source.reset(new pcl::PointCloud<pcl::PointXYZ>);
    instance.target.reset(new pcl::PointCloud<pcl::PointXYZ>);
    instance.source->reserve(pointCount);
    instance.target->reserve(pointCount);
    instance.correspondences.reserve(pointCount);

    const std::size_t outlierCount =
        static_cast<std::size_t>(pointCount * outlierRatio);
    instance.inlierCount = pointCount - outlierCount;

    for (std::size_t index = 0; index < pointCount; ++index)
    {
        pcl::PointXYZ sourcePoint(
            static_cast<float>(coordinateDistribution(randomEngine)),
            static_cast<float>(coordinateDistribution(randomEngine)),
            static_cast<float>(coordinateDistribution(randomEngine)));
        pcl::PointXYZ targetPoint = sourcePoint;

        if (index >= instance.inlierCount)
        {
            targetPoint.x =
                static_cast<float>(coordinateDistribution(randomEngine));
            targetPoint.y =
                static_cast<float>(coordinateDistribution(randomEngine));
            targetPoint.z =
                static_cast<float>(coordinateDistribution(randomEngine));
        }

        addRadialGaussianNoise(
            sourcePoint,
            randomEngine,
            noiseDistribution,
            angleDistribution);
        addRadialGaussianNoise(
            targetPoint,
            randomEngine,
            noiseDistribution,
            angleDistribution);

        instance.source->push_back(sourcePoint);
        instance.target->push_back(targetPoint);
        instance.correspondences.emplace_back(
            static_cast<int>(index),
            static_cast<int>(index),
            0.0f);
    }

    const double yaw = angleDistribution(randomEngine);
    instance.groundTruth(0, 0) = std::cos(yaw);
    instance.groundTruth(0, 1) = -std::sin(yaw);
    instance.groundTruth(1, 0) = std::sin(yaw);
    instance.groundTruth(1, 1) = std::cos(yaw);
    instance.groundTruth(0, 3) = transformationDistribution(randomEngine);
    instance.groundTruth(1, 3) = transformationDistribution(randomEngine);
    instance.groundTruth(2, 3) = transformationDistribution(randomEngine);
    pcl::transformPointCloud(
        *instance.target,
        *instance.target,
        instance.groundTruth.cast<float>());

    instance.source->width = static_cast<std::uint32_t>(pointCount);
    instance.source->height = 1;
    instance.target->width = static_cast<std::uint32_t>(pointCount);
    instance.target->height = 1;
    return instance;
}

std::uint64_t correspondenceKey(
    const pcl::Correspondence& correspondence)
{
    return
        (static_cast<std::uint64_t>(
            static_cast<std::uint32_t>(correspondence.index_query)) << 32) |
        static_cast<std::uint32_t>(correspondence.index_match);
}

StageCounts countStageCorrespondences(
    const pcl::Correspondences& correspondences,
    std::size_t inlierCount)
{
    StageCounts counts;
    std::unordered_set<std::uint64_t> observed;
    observed.reserve(correspondences.size());

    for (const pcl::Correspondence& correspondence : correspondences)
    {
        if (!observed.emplace(correspondenceKey(correspondence)).second)
            continue;

        const bool isTrueInlier =
            correspondence.index_query == correspondence.index_match &&
            correspondence.index_query >= 0 &&
            static_cast<std::size_t>(correspondence.index_query) < inlierCount;
        if (isTrueInlier)
            ++counts.truePositives;
        else
            ++counts.falsePositives;
    }
    return counts;
}

double safeRatio(std::uint64_t numerator, std::uint64_t denominator)
{
    return denominator == 0
        ? 0.0
        : static_cast<double>(numerator) /
            static_cast<double>(denominator);
}
}

int runOutlierStageExperiment(
    double requestedResolution,
    double requestedAngleResolutionDegrees)
{
    if (!std::isfinite(requestedResolution) || requestedResolution <= 0.0)
    {
        std::cerr << "Resolution must be a positive finite number." << std::endl;
        return 2;
    }
    const float resolution = static_cast<float>(requestedResolution);
    if (!std::isfinite(requestedAngleResolutionDegrees) ||
        requestedAngleResolutionDegrees <= 0.0)
    {
        std::cerr
            << "Angle resolution must be a positive finite number."
            << std::endl;
        return 2;
    }
    const float angleResolutionDegrees =
        static_cast<float>(requestedAngleResolutionDegrees);

    std::ofstream detailFile("outlier_stage_detail.csv");
    std::ofstream summaryFile("outlier_stage_summary.csv");
    std::ofstream totalFile("outlier_stage_total_time.csv");
    std::ofstream accuracyDetailFile("outlier_stage_accuracy_detail.csv");
    std::ofstream accuracySummaryFile("outlier_stage_accuracy_summary.csv");
    if (!detailFile || !summaryFile || !totalFile ||
        !accuracyDetailFile || !accuracySummaryFile)
    {
        std::cerr << "Cannot create outlier-stage result files." << std::endl;
        return 3;
    }

    detailFile
        << "trial,outlier_ratio,inlier_count,stage,tp,fp,n,time_ms,"
        << "precision,recall,end_to_end_time_ms\n";
    summaryFile
        << "outlier_ratio,ground_truth_inliers,ground_truth_outliers,"
        << "stage,mean_tp,mean_fp,mean_n,mean_time_ms,median_time_ms,"
        << "precision,recall\n";
    totalFile
        << "outlier_ratio,mean_end_to_end_time_ms,"
        << "median_end_to_end_time_ms\n";
    accuracyDetailFile
        << "trial,outlier_ratio,rotation_error_deg,translation_error,"
        << "success\n";
    accuracySummaryFile
        << "outlier_ratio,mean_rotation_error_deg,mean_translation_error,"
        << "success_rate_percent\n";

    std::cout << std::fixed << std::setprecision(6)
        << "N=" << pointCount
        << ", sigma=" << noiseSigma
        << ", resolution=" << resolution
        << ", angle_resolution_deg=" << angleResolutionDegrees
        << ", trials=" << trialCount << '\n';

    for (std::size_t ratioIndex = 0;
        ratioIndex < outlierRatios.size();
        ++ratioIndex)
    {
        const double outlierRatio = outlierRatios[ratioIndex];
        std::array<StageAccumulator, 4> accumulated{};
        std::mt19937 randomEngine(
            experimentSeed + static_cast<unsigned int>(ratioIndex) * 1009U);
        std::size_t inlierCount = 0;
        std::vector<double> endToEndTimesMilliseconds;
        endToEndTimesMilliseconds.reserve(trialCount);
        AccuracyAccumulator accuracy{};

        for (std::size_t trial = 0; trial < trialCount; ++trial)
        {
            SimulationInstance instance =
                generateInstance(randomEngine, outlierRatio);
            inlierCount = instance.inlierCount;
            const auto solverStart = std::chrono::steady_clock::now();
            Decoupling solver(
                instance.source,
                instance.target,
                instance.correspondences,
                resolution,
                Eigen::Vector3d::UnitZ(),
                false,
                angleResolutionDegrees);
            const auto solverEnd = std::chrono::steady_clock::now();
            const double endToEndTimeMilliseconds =
                std::chrono::duration<double, std::milli>(
                    solverEnd - solverStart).count();
            endToEndTimesMilliseconds.push_back(endToEndTimeMilliseconds);

            const Eigen::Matrix4d errorTransformation =
                instance.groundTruth * solver.getTransformation().inverse();
            const Eigen::Matrix3d rotationError =
                errorTransformation.block<3, 3>(0, 0);
            const double rotationCosine = std::clamp(
                (rotationError.trace() - 1.0) * 0.5,
                -1.0,
                1.0);
            const double rotationErrorDegrees =
                std::acos(rotationCosine) * 180.0 / M_PI;
            const double translationError =
                errorTransformation.block<3, 1>(0, 3).norm();
            const bool success = solver.isValid() &&
                rotationErrorDegrees < rotationSuccessThresholdDegrees &&
                translationError < translationSuccessThreshold;
            accuracy.rotationErrorDegreesSum += rotationErrorDegrees;
            accuracy.translationErrorSum += translationError;
            if (success)
                ++accuracy.successCount;
            accuracyDetailFile << std::setprecision(10)
                << trial << ','
                << outlierRatio << ','
                << rotationErrorDegrees << ','
                << translationError << ','
                << (success ? 1 : 0) << '\n';

            const std::array<double, 4>& stageTimes =
                solver.getStageTimesSeconds();
            for (std::size_t stageIndex = 0; stageIndex < 4; ++stageIndex)
            {
                const StageCounts counts = countStageCorrespondences(
                    solver.getStageCorrespondences(stageIndex),
                    instance.inlierCount);
                StageAccumulator& accumulator = accumulated[stageIndex];
                accumulator.truePositiveSum += counts.truePositives;
                accumulator.falsePositiveSum += counts.falsePositives;
                accumulator.timeSecondsSum += stageTimes[stageIndex];
                accumulator.timeMilliseconds.push_back(
                    stageTimes[stageIndex] * 1000.0);

                detailFile << std::setprecision(10)
                    << trial << ','
                    << outlierRatio << ','
                    << instance.inlierCount << ','
                    << (stageIndex + 1) << ','
                    << counts.truePositives << ','
                    << counts.falsePositives << ','
                    << counts.retainedCount() << ','
                    << stageTimes[stageIndex] * 1000.0 << ','
                    << safeRatio(
                        counts.truePositives,
                        counts.retainedCount()) << ','
                    << safeRatio(
                        counts.truePositives,
                        instance.inlierCount) << ','
                    << endToEndTimeMilliseconds << '\n';
            }
        }

        for (std::size_t stageIndex = 0; stageIndex < 4; ++stageIndex)
        {
            const StageAccumulator& accumulator = accumulated[stageIndex];
            const std::uint64_t retainedSum =
                accumulator.truePositiveSum +
                accumulator.falsePositiveSum;
            const double meanTp =
                static_cast<double>(accumulator.truePositiveSum) /
                static_cast<double>(trialCount);
            const double meanFp =
                static_cast<double>(accumulator.falsePositiveSum) /
                static_cast<double>(trialCount);
            const double meanN = meanTp + meanFp;
            const double meanTimeMilliseconds =
                accumulator.timeSecondsSum * 1000.0 /
                static_cast<double>(trialCount);
            const double medianTimeMilliseconds =
                median(accumulator.timeMilliseconds);
            const double precision = safeRatio(
                accumulator.truePositiveSum,
                retainedSum);
            const double recall = safeRatio(
                accumulator.truePositiveSum,
                static_cast<std::uint64_t>(inlierCount) * trialCount);

            summaryFile << std::setprecision(10)
                << outlierRatio << ','
                << inlierCount << ','
                << (pointCount - inlierCount) << ','
                << (stageIndex + 1) << ','
                << meanTp << ','
                << meanFp << ','
                << meanN << ','
                << meanTimeMilliseconds << ','
                << medianTimeMilliseconds << ','
                << precision << ','
                << recall << '\n';

            std::cout
                << "eta=" << outlierRatio
                << ", Stage " << (stageIndex + 1)
                << ": TP=" << meanTp
                << ", FP=" << meanFp
                << ", N=" << meanN
                << ", T=" << meanTimeMilliseconds << " ms"
                << " (median " << medianTimeMilliseconds << " ms)"
                << ", Precision=" << precision
                << ", Recall=" << recall << '\n';
        }

        const double meanEndToEndTimeMilliseconds =
            std::accumulate(
                endToEndTimesMilliseconds.begin(),
                endToEndTimesMilliseconds.end(),
                0.0) / static_cast<double>(trialCount);
        const double medianEndToEndTimeMilliseconds =
            median(endToEndTimesMilliseconds);
        totalFile << std::setprecision(10)
            << outlierRatio << ','
            << meanEndToEndTimeMilliseconds << ','
            << medianEndToEndTimeMilliseconds << '\n';
        accuracySummaryFile << std::setprecision(10)
            << outlierRatio << ','
            << accuracy.rotationErrorDegreesSum /
                static_cast<double>(trialCount) << ','
            << accuracy.translationErrorSum /
                static_cast<double>(trialCount) << ','
            << 100.0 * static_cast<double>(accuracy.successCount) /
                static_cast<double>(trialCount) << '\n';
    }

    std::cout
        << "Saved outlier_stage_detail.csv and "
        << "outlier_stage_summary.csv, outlier_stage_total_time.csv and "
        << "outlier_stage_accuracy_summary.csv"
        << std::endl;
    return 0;
}

int runAnglePruningAblationExperiment()
{
    constexpr double outlierRatio = 0.95;
    constexpr float resolution = 0.015f;
    constexpr float angleResolutionDegrees = 5.0f;
    constexpr std::size_t ratioIndex = 5;

    struct AblationAccumulator
    {
        double rotationErrorSum{ 0.0 };
        double translationErrorSum{ 0.0 };
        double stage2PrecisionSum{ 0.0 };
        double stage2RecallSum{ 0.0 };
        double finalPrecisionSum{ 0.0 };
        double finalRecallSum{ 0.0 };
        std::size_t successCount{ 0 };
        std::vector<double> stage2TimesMilliseconds;
        std::vector<double> endToEndTimesMilliseconds;
    };

    std::ofstream detailFile("angle_pruning_ablation_detail.csv");
    std::ofstream summaryFile("angle_pruning_ablation_summary.csv");
    if (!detailFile || !summaryFile)
    {
        std::cerr << "Cannot create angle-pruning ablation files." << std::endl;
        return 3;
    }

    detailFile
        << "trial,method,pruning_enabled,N,outlier_ratio,noise_sigma,"
        << "resolution,angle_resolution_deg,stage2_tp,stage2_fp,stage2_n,"
        << "stage2_precision,stage2_recall,final_tp,final_fp,final_n,"
        << "final_precision,final_recall,stage2_time_ms,end_to_end_time_ms,"
        << "rotation_error_deg,translation_error,success\n";
    summaryFile
        << "method,pruning_enabled,N,outlier_ratio,noise_sigma,resolution,"
        << "angle_resolution_deg,trials,mean_stage2_precision,"
        << "mean_stage2_recall,mean_final_precision,mean_final_recall,"
        << "mean_stage2_time_ms,median_stage2_time_ms,"
        << "mean_end_to_end_time_ms,median_end_to_end_time_ms,"
        << "mean_rotation_error_deg,mean_translation_error,"
        << "success_rate_percent\n";

    std::array<AblationAccumulator, 2> accumulated{};
    std::mt19937 randomEngine(
        experimentSeed + static_cast<unsigned int>(ratioIndex) * 1009U);

    for (std::size_t trial = 0; trial < trialCount; ++trial)
    {
        const SimulationInstance instance =
            generateInstance(randomEngine, outlierRatio);
        const std::array<std::size_t, 2> executionOrder =
            trial % 2 == 0
            ? std::array<std::size_t, 2>{ 0, 1 }
            : std::array<std::size_t, 2>{ 1, 0 };

        for (std::size_t methodIndex : executionOrder)
        {
            const bool pruningEnabled = methodIndex == 0;
            const char* methodName = pruningEnabled
                ? "with_pruning"
                : "without_pruning";
            const auto solverStart = std::chrono::steady_clock::now();
            Decoupling solver(
                instance.source,
                instance.target,
                instance.correspondences,
                resolution,
                Eigen::Vector3d::UnitZ(),
                false,
                angleResolutionDegrees,
                pruningEnabled);
            const auto solverEnd = std::chrono::steady_clock::now();
            const double endToEndTimeMilliseconds =
                std::chrono::duration<double, std::milli>(
                    solverEnd - solverStart).count();

            const StageCounts stage2Counts = countStageCorrespondences(
                solver.getStageCorrespondences(1),
                instance.inlierCount);
            const StageCounts finalCounts = countStageCorrespondences(
                solver.getStageCorrespondences(3),
                instance.inlierCount);
            const double stage2Precision = safeRatio(
                stage2Counts.truePositives,
                stage2Counts.retainedCount());
            const double stage2Recall = safeRatio(
                stage2Counts.truePositives,
                instance.inlierCount);
            const double finalPrecision = safeRatio(
                finalCounts.truePositives,
                finalCounts.retainedCount());
            const double finalRecall = safeRatio(
                finalCounts.truePositives,
                instance.inlierCount);

            const Eigen::Matrix4d errorTransformation =
                instance.groundTruth * solver.getTransformation().inverse();
            const double rotationCosine = std::clamp(
                (errorTransformation.block<3, 3>(0, 0).trace() - 1.0) * 0.5,
                -1.0,
                1.0);
            const double rotationErrorDegrees =
                std::acos(rotationCosine) * 180.0 / M_PI;
            const double translationError =
                errorTransformation.block<3, 1>(0, 3).norm();
            const bool success = solver.isValid() &&
                std::isfinite(rotationErrorDegrees) &&
                std::isfinite(translationError) &&
                rotationErrorDegrees < rotationSuccessThresholdDegrees &&
                translationError < translationSuccessThreshold;
            const double stage2TimeMilliseconds =
                solver.getStageTimesSeconds()[1] * 1000.0;

            AblationAccumulator& accumulator = accumulated[methodIndex];
            accumulator.rotationErrorSum += rotationErrorDegrees;
            accumulator.translationErrorSum += translationError;
            accumulator.stage2PrecisionSum += stage2Precision;
            accumulator.stage2RecallSum += stage2Recall;
            accumulator.finalPrecisionSum += finalPrecision;
            accumulator.finalRecallSum += finalRecall;
            accumulator.stage2TimesMilliseconds.push_back(
                stage2TimeMilliseconds);
            accumulator.endToEndTimesMilliseconds.push_back(
                endToEndTimeMilliseconds);
            if (success)
                ++accumulator.successCount;

            detailFile << std::setprecision(10)
                << trial << ',' << methodName << ','
                << (pruningEnabled ? 1 : 0) << ',' << pointCount << ','
                << outlierRatio << ',' << noiseSigma << ',' << resolution
                << ',' << angleResolutionDegrees << ','
                << stage2Counts.truePositives << ','
                << stage2Counts.falsePositives << ','
                << stage2Counts.retainedCount() << ','
                << stage2Precision << ',' << stage2Recall << ','
                << finalCounts.truePositives << ','
                << finalCounts.falsePositives << ','
                << finalCounts.retainedCount() << ','
                << finalPrecision << ',' << finalRecall << ','
                << stage2TimeMilliseconds << ','
                << endToEndTimeMilliseconds << ','
                << rotationErrorDegrees << ',' << translationError << ','
                << (success ? 1 : 0) << '\n';
        }
    }

    std::cout << std::fixed << std::setprecision(6)
        << "N=" << pointCount << ", eta=" << outlierRatio
        << ", sigma=" << noiseSigma << ", resolution=" << resolution
        << ", angle_resolution_deg=" << angleResolutionDegrees
        << ", trials=" << trialCount << '\n';

    for (std::size_t methodIndex = 0; methodIndex < 2; ++methodIndex)
    {
        const bool pruningEnabled = methodIndex == 0;
        const char* methodName = pruningEnabled
            ? "with_pruning"
            : "without_pruning";
        const AblationAccumulator& accumulator = accumulated[methodIndex];
        const double meanStage2Time = std::accumulate(
            accumulator.stage2TimesMilliseconds.begin(),
            accumulator.stage2TimesMilliseconds.end(),
            0.0) / static_cast<double>(trialCount);
        const double meanEndToEndTime = std::accumulate(
            accumulator.endToEndTimesMilliseconds.begin(),
            accumulator.endToEndTimesMilliseconds.end(),
            0.0) / static_cast<double>(trialCount);
        const double meanRotationError =
            accumulator.rotationErrorSum / trialCount;
        const double meanTranslationError =
            accumulator.translationErrorSum / trialCount;
        const double successRate =
            100.0 * accumulator.successCount / trialCount;

        summaryFile << std::setprecision(10)
            << methodName << ',' << (pruningEnabled ? 1 : 0) << ','
            << pointCount << ',' << outlierRatio << ',' << noiseSigma << ','
            << resolution << ',' << angleResolutionDegrees << ','
            << trialCount << ','
            << accumulator.stage2PrecisionSum / trialCount << ','
            << accumulator.stage2RecallSum / trialCount << ','
            << accumulator.finalPrecisionSum / trialCount << ','
            << accumulator.finalRecallSum / trialCount << ','
            << meanStage2Time << ','
            << median(accumulator.stage2TimesMilliseconds) << ','
            << meanEndToEndTime << ','
            << median(accumulator.endToEndTimesMilliseconds) << ','
            << meanRotationError << ',' << meanTranslationError << ','
            << successRate << '\n';

        std::cout << methodName
            << ": Stage-II time=" << meanStage2Time << " ms"
            << ", total time=" << meanEndToEndTime << " ms"
            << ", Stage-II P/R="
            << accumulator.stage2PrecisionSum / trialCount << '/'
            << accumulator.stage2RecallSum / trialCount
            << ", final P/R="
            << accumulator.finalPrecisionSum / trialCount << '/'
            << accumulator.finalRecallSum / trialCount
            << ", RE=" << meanRotationError
            << ", TE=" << meanTranslationError
            << ", success=" << successRate << "%\n";
    }

    std::cout
        << "Saved angle_pruning_ablation_detail.csv and "
        << "angle_pruning_ablation_summary.csv\n";
    return 0;
}

int runStageOrderAblationExperiment()
{
    constexpr double outlierRatio = 0.95;
    constexpr float resolution = 0.015f;
    constexpr float angleResolutionDegrees = 5.0f;
    constexpr std::size_t ratioIndex = 5;

    struct OrderAccumulator
    {
        std::array<double, 2> precisionSum{ 0.0, 0.0 };
        std::array<double, 2> recallSum{ 0.0, 0.0 };
        std::array<double, 2> timeSumMilliseconds{ 0.0, 0.0 };
        double finalPrecisionSum{ 0.0 };
        double finalRecallSum{ 0.0 };
        double rotationErrorSum{ 0.0 };
        double translationErrorSum{ 0.0 };
        std::size_t successCount{ 0 };
        std::vector<double> endToEndTimesMilliseconds;
    };

    std::ofstream detailFile("stage_order_ablation_detail.csv");
    std::ofstream summaryFile("stage_order_ablation_summary.csv");
    if (!detailFile || !summaryFile)
    {
        std::cerr << "Cannot create stage-order ablation files." << std::endl;
        return 3;
    }

    detailFile
        << "trial,order,swapped,N,outlier_ratio,noise_sigma,resolution,"
        << "angle_resolution_deg,stage1_operation,stage1_tp,stage1_fp,"
        << "stage1_n,stage1_precision,stage1_recall,stage1_time_ms,"
        << "stage2_operation,stage2_tp,stage2_fp,stage2_n,stage2_precision,"
        << "stage2_recall,stage2_time_ms,final_tp,final_fp,final_n,"
        << "final_precision,final_recall,end_to_end_time_ms,"
        << "rotation_error_deg,translation_error,success\n";
    summaryFile
        << "order,swapped,N,outlier_ratio,noise_sigma,resolution,"
        << "angle_resolution_deg,trials,stage1_operation,"
        << "mean_stage1_precision,mean_stage1_recall,mean_stage1_time_ms,"
        << "stage2_operation,mean_stage2_precision,mean_stage2_recall,"
        << "mean_stage2_time_ms,mean_final_precision,mean_final_recall,"
        << "mean_end_to_end_time_ms,median_end_to_end_time_ms,"
        << "mean_rotation_error_deg,mean_translation_error,"
        << "success_rate_percent\n";

    std::array<OrderAccumulator, 2> accumulated{};
    std::mt19937 randomEngine(
        experimentSeed + static_cast<unsigned int>(ratioIndex) * 1009U);

    for (std::size_t trial = 0; trial < trialCount; ++trial)
    {
        const SimulationInstance instance =
            generateInstance(randomEngine, outlierRatio);
        const std::array<std::size_t, 2> executionOrder =
            trial % 2 == 0
            ? std::array<std::size_t, 2>{ 0, 1 }
            : std::array<std::size_t, 2>{ 1, 0 };

        for (std::size_t orderIndex : executionOrder)
        {
            const bool swapped = orderIndex == 1;
            const char* orderName = swapped
                ? "rotation_then_vertical"
                : "vertical_then_rotation";
            const char* stage1Operation = swapped ? "rotation" : "vertical";
            const char* stage2Operation = swapped ? "vertical" : "rotation";

            const auto solverStart = std::chrono::steady_clock::now();
            Decoupling solver(
                instance.source,
                instance.target,
                instance.correspondences,
                resolution,
                Eigen::Vector3d::UnitZ(),
                false,
                angleResolutionDegrees,
                true,
                swapped);
            const auto solverEnd = std::chrono::steady_clock::now();
            const double endToEndTimeMilliseconds =
                std::chrono::duration<double, std::milli>(
                    solverEnd - solverStart).count();

            std::array<StageCounts, 2> stageCounts{
                countStageCorrespondences(
                    solver.getStageCorrespondences(0), instance.inlierCount),
                countStageCorrespondences(
                    solver.getStageCorrespondences(1), instance.inlierCount)
            };
            const StageCounts finalCounts = countStageCorrespondences(
                solver.getStageCorrespondences(3), instance.inlierCount);
            std::array<double, 2> precision{};
            std::array<double, 2> recall{};
            for (std::size_t stage = 0; stage < 2; ++stage)
            {
                precision[stage] = safeRatio(
                    stageCounts[stage].truePositives,
                    stageCounts[stage].retainedCount());
                recall[stage] = safeRatio(
                    stageCounts[stage].truePositives,
                    instance.inlierCount);
            }
            const double finalPrecision = safeRatio(
                finalCounts.truePositives, finalCounts.retainedCount());
            const double finalRecall = safeRatio(
                finalCounts.truePositives, instance.inlierCount);

            const Eigen::Matrix4d errorTransformation =
                instance.groundTruth * solver.getTransformation().inverse();
            const double rotationCosine = std::clamp(
                (errorTransformation.block<3, 3>(0, 0).trace() - 1.0) * 0.5,
                -1.0,
                1.0);
            const double rotationErrorDegrees =
                std::acos(rotationCosine) * 180.0 / M_PI;
            const double translationError =
                errorTransformation.block<3, 1>(0, 3).norm();
            const bool success = solver.isValid() &&
                std::isfinite(rotationErrorDegrees) &&
                std::isfinite(translationError) &&
                rotationErrorDegrees < rotationSuccessThresholdDegrees &&
                translationError < translationSuccessThreshold;
            const auto& stageTimes = solver.getStageTimesSeconds();

            OrderAccumulator& accumulator = accumulated[orderIndex];
            for (std::size_t stage = 0; stage < 2; ++stage)
            {
                accumulator.precisionSum[stage] += precision[stage];
                accumulator.recallSum[stage] += recall[stage];
                accumulator.timeSumMilliseconds[stage] +=
                    stageTimes[stage] * 1000.0;
            }
            accumulator.finalPrecisionSum += finalPrecision;
            accumulator.finalRecallSum += finalRecall;
            accumulator.rotationErrorSum += rotationErrorDegrees;
            accumulator.translationErrorSum += translationError;
            accumulator.endToEndTimesMilliseconds.push_back(
                endToEndTimeMilliseconds);
            if (success)
                ++accumulator.successCount;

            detailFile << std::setprecision(10)
                << trial << ',' << orderName << ',' << (swapped ? 1 : 0)
                << ',' << pointCount << ',' << outlierRatio << ','
                << noiseSigma << ',' << resolution << ','
                << angleResolutionDegrees << ',' << stage1Operation << ','
                << stageCounts[0].truePositives << ','
                << stageCounts[0].falsePositives << ','
                << stageCounts[0].retainedCount() << ','
                << precision[0] << ',' << recall[0] << ','
                << stageTimes[0] * 1000.0 << ',' << stage2Operation << ','
                << stageCounts[1].truePositives << ','
                << stageCounts[1].falsePositives << ','
                << stageCounts[1].retainedCount() << ','
                << precision[1] << ',' << recall[1] << ','
                << stageTimes[1] * 1000.0 << ','
                << finalCounts.truePositives << ','
                << finalCounts.falsePositives << ','
                << finalCounts.retainedCount() << ','
                << finalPrecision << ',' << finalRecall << ','
                << endToEndTimeMilliseconds << ','
                << rotationErrorDegrees << ',' << translationError << ','
                << (success ? 1 : 0) << '\n';
        }
    }

    std::cout << std::fixed << std::setprecision(6)
        << "N=" << pointCount << ", eta=" << outlierRatio
        << ", sigma=" << noiseSigma << ", resolution=" << resolution
        << ", angle_resolution_deg=" << angleResolutionDegrees
        << ", trials=" << trialCount << '\n';

    for (std::size_t orderIndex = 0; orderIndex < 2; ++orderIndex)
    {
        const bool swapped = orderIndex == 1;
        const char* orderName = swapped
            ? "rotation_then_vertical"
            : "vertical_then_rotation";
        const char* stage1Operation = swapped ? "rotation" : "vertical";
        const char* stage2Operation = swapped ? "vertical" : "rotation";
        const OrderAccumulator& accumulator = accumulated[orderIndex];
        const double meanEndToEndTime = std::accumulate(
            accumulator.endToEndTimesMilliseconds.begin(),
            accumulator.endToEndTimesMilliseconds.end(),
            0.0) / static_cast<double>(trialCount);
        const double successRate =
            100.0 * accumulator.successCount / trialCount;

        summaryFile << std::setprecision(10)
            << orderName << ',' << (swapped ? 1 : 0) << ',' << pointCount
            << ',' << outlierRatio << ',' << noiseSigma << ',' << resolution
            << ',' << angleResolutionDegrees << ',' << trialCount << ','
            << stage1Operation << ','
            << accumulator.precisionSum[0] / trialCount << ','
            << accumulator.recallSum[0] / trialCount << ','
            << accumulator.timeSumMilliseconds[0] / trialCount << ','
            << stage2Operation << ','
            << accumulator.precisionSum[1] / trialCount << ','
            << accumulator.recallSum[1] / trialCount << ','
            << accumulator.timeSumMilliseconds[1] / trialCount << ','
            << accumulator.finalPrecisionSum / trialCount << ','
            << accumulator.finalRecallSum / trialCount << ','
            << meanEndToEndTime << ','
            << median(accumulator.endToEndTimesMilliseconds) << ','
            << accumulator.rotationErrorSum / trialCount << ','
            << accumulator.translationErrorSum / trialCount << ','
            << successRate << '\n';

        std::cout << orderName
            << ": Stage-I P/R/time="
            << accumulator.precisionSum[0] / trialCount << '/'
            << accumulator.recallSum[0] / trialCount << '/'
            << accumulator.timeSumMilliseconds[0] / trialCount << " ms"
            << ", Stage-II P/R/time="
            << accumulator.precisionSum[1] / trialCount << '/'
            << accumulator.recallSum[1] / trialCount << '/'
            << accumulator.timeSumMilliseconds[1] / trialCount << " ms"
            << ", final P/R="
            << accumulator.finalPrecisionSum / trialCount << '/'
            << accumulator.finalRecallSum / trialCount
            << ", total=" << meanEndToEndTime << " ms"
            << ", RE=" << accumulator.rotationErrorSum / trialCount
            << ", TE=" << accumulator.translationErrorSum / trialCount
            << ", success=" << successRate << "%\n";
    }

    std::cout
        << "Saved stage_order_ablation_detail.csv and "
        << "stage_order_ablation_summary.csv\n";
    return 0;
}
