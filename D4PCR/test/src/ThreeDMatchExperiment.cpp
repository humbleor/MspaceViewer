#include "../include/ThreeDMatchExperiment.h"

#include "Decoupling.h"
#include "Preparation.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
namespace fs = std::filesystem;

constexpr double rotationSuccessThresholdDegrees = 3.0;
constexpr double translationSuccessThreshold = 0.5;

struct GroundTruthPair
{
    int firstFragment{ 0 };
    int secondFragment{ 0 };
    int fragmentCount{ 0 };
    Eigen::Matrix4d transformation{ Eigen::Matrix4d::Identity() };
};

struct PairResult
{
    std::string scene;
    int firstFragment{ 0 };
    int secondFragment{ 0 };
    int fragmentCount{ 0 };
    std::size_t initialCount{ 0 };
    std::size_t groundTruthInlierCount{ 0 };
    std::size_t retainedCount{ 0 };
    std::size_t truePositiveCount{ 0 };
    std::size_t falsePositiveCount{ 0 };
    std::array<std::size_t, 4> stageRetainedCounts{};
    std::array<std::size_t, 4> stageTruePositiveCounts{};
    std::array<std::size_t, 4> stageFalsePositiveCounts{};
    double initialInlierRatio{ 0.0 };
    double precision{ 0.0 };
    double recall{ 0.0 };
    double rotationErrorDegrees{ std::numeric_limits<double>::quiet_NaN() };
    double translationError{ std::numeric_limits<double>::quiet_NaN() };
    double runtimeMilliseconds{ 0.0 };
    bool valid{ false };
    bool successful{ false };
    Eigen::Matrix4d estimatedTransformation{ Eigen::Matrix4d::Identity() };
};

struct SceneSummary
{
    std::string scene;
    std::size_t pairCount{ 0 };
    std::size_t validCount{ 0 };
    std::size_t successCount{ 0 };
    std::uint64_t groundTruthInlierCount{ 0 };
    std::uint64_t truePositiveCount{ 0 };
    std::uint64_t falsePositiveCount{ 0 };
    double rotationErrorSum{ 0.0 };
    double translationErrorSum{ 0.0 };
    double runtimeMillisecondsSum{ 0.0 };
    double precisionSum{ 0.0 };
    double recallSum{ 0.0 };
    std::vector<double> rotationErrors;
    std::vector<double> translationErrors;
};

double median(std::vector<double> values)
{
    if (values.empty())
        return std::numeric_limits<double>::quiet_NaN();
    std::sort(values.begin(), values.end());
    const std::size_t middle = values.size() / 2;
    return values.size() % 2 == 0
        ? (values[middle - 1] + values[middle]) * 0.5
        : values[middle];
}

std::vector<GroundTruthPair> readGroundTruthLog(const fs::path& logPath)
{
    std::ifstream input(logPath);
    if (!input)
        throw std::runtime_error("Cannot open ground-truth log: " + logPath.string());

    std::vector<GroundTruthPair> pairs;
    GroundTruthPair pair;
    while (input >> pair.firstFragment >> pair.secondFragment >> pair.fragmentCount)
    {
        for (int row = 0; row < 4; ++row)
            for (int column = 0; column < 4; ++column)
                if (!(input >> pair.transformation(row, column)))
                    throw std::runtime_error("Malformed ground-truth matrix: " + logPath.string());
        pairs.emplace_back(pair);
    }
    return pairs;
}

Eigen::Matrix3d makeSourceGravityAlignment(const Eigen::Matrix3d& groundTruthRotation)
{
    const double yaw = std::atan2(groundTruthRotation(1, 0), groundTruthRotation(0, 0));
    Eigen::Matrix3d yawRotation = Eigen::Matrix3d::Identity();
    yawRotation(0, 0) = std::cos(yaw);
    yawRotation(0, 1) = -std::sin(yaw);
    yawRotation(1, 0) = std::sin(yaw);
    yawRotation(1, 1) = std::cos(yaw);
    return yawRotation.transpose() * groundTruthRotation;
}

PairResult evaluatePair(
    const std::string& scene,
    const GroundTruthPair& groundTruth,
    const fs::path& correspondencePath,
    double resolution,
    double angleResolutionDegrees,
    double inlierThreshold,
    bool enableFinalResidualFilter)
{
    PairResult result;
    result.scene = scene;
    result.firstFragment = groundTruth.firstFragment;
    result.secondFragment = groundTruth.secondFragment;
    result.fragmentCount = groundTruth.fragmentCount;

    std::ifstream input(correspondencePath);
    if (!input)
        throw std::runtime_error("Cannot open correspondence file: " + correspondencePath.string());

    const Eigen::Matrix3d groundTruthRotation =
        groundTruth.transformation.block<3, 3>(0, 0);
    const Eigen::Vector3d groundTruthTranslation =
        groundTruth.transformation.block<3, 1>(0, 3);
    const Eigen::Matrix3d sourceAlignment =
        makeSourceGravityAlignment(groundTruthRotation);

    PointCloudPtr source(new pcl::PointCloud<pcl::PointXYZ>);
    PointCloudPtr target(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::Correspondences correspondences;
    std::vector<bool> inlierLabels;

    double x1, y1, z1, x2, y2, z2;
    while (input >> x1 >> y1 >> z1 >> x2 >> y2 >> z2)
    {
        const Eigen::Vector3d targetPoint(x1, y1, z1);
        const Eigen::Vector3d originalSourcePoint(x2, y2, z2);
        const Eigen::Vector3d alignedSourcePoint = sourceAlignment * originalSourcePoint;

        target->emplace_back(
            static_cast<float>(targetPoint.x()),
            static_cast<float>(targetPoint.y()),
            static_cast<float>(targetPoint.z()));
        source->emplace_back(
            static_cast<float>(alignedSourcePoint.x()),
            static_cast<float>(alignedSourcePoint.y()),
            static_cast<float>(alignedSourcePoint.z()));

        pcl::Correspondence correspondence;
        correspondence.index_query = static_cast<int>(correspondences.size());
        correspondence.index_match = correspondence.index_query;
        correspondence.distance = 0.0f;
        correspondences.emplace_back(correspondence);

        const bool isInlier =
            (groundTruthRotation * originalSourcePoint + groundTruthTranslation -
                targetPoint).norm() < inlierThreshold;
        inlierLabels.emplace_back(isInlier);
        if (isInlier)
            ++result.groundTruthInlierCount;
    }

    source->width = static_cast<std::uint32_t>(source->size());
    source->height = 1;
    source->is_dense = true;
    target->width = static_cast<std::uint32_t>(target->size());
    target->height = 1;
    target->is_dense = true;
    result.initialCount = correspondences.size();
    result.initialInlierRatio = result.initialCount == 0
        ? 0.0
        : static_cast<double>(result.groundTruthInlierCount) / result.initialCount;
    if (correspondences.empty())
        return result;

    Decoupling solver(
        source,
        target,
        correspondences,
        static_cast<float>(resolution),
        Eigen::Vector3d::UnitZ(),
        false,
        static_cast<float>(angleResolutionDegrees));

    for (std::size_t stage = 0; stage < 4; ++stage)
    {
        const pcl::Correspondences& stageCorrespondences =
            solver.getStageCorrespondences(stage);
        result.stageRetainedCounts[stage] = stageCorrespondences.size();
        for (const pcl::Correspondence& correspondence : stageCorrespondences)
        {
            const std::size_t index = static_cast<std::size_t>(correspondence.index_query);
            if (index < inlierLabels.size() && inlierLabels[index])
                ++result.stageTruePositiveCounts[stage];
            else
                ++result.stageFalsePositiveCounts[stage];
        }
    }
    result.retainedCount = result.stageRetainedCounts[3];
    result.truePositiveCount = result.stageTruePositiveCounts[3];
    result.falsePositiveCount = result.stageFalsePositiveCounts[3];

    result.precision = result.retainedCount == 0
        ? 0.0
        : static_cast<double>(result.truePositiveCount) / result.retainedCount;
    result.recall = result.groundTruthInlierCount == 0
        ? 0.0
        : static_cast<double>(result.truePositiveCount) / result.groundTruthInlierCount;
    const auto& stageTimes = solver.getStageTimesSeconds();
    result.runtimeMilliseconds =
        std::accumulate(stageTimes.begin(), stageTimes.end(), 0.0) * 1000.0;
    if (!solver.isValid())
        return result;

    Eigen::Matrix4d alignment = Eigen::Matrix4d::Identity();
    alignment.block<3, 3>(0, 0) = sourceAlignment;
    result.estimatedTransformation = solver.getTransformation() * alignment;

    if (enableFinalResidualFilter)
    {
        const auto filterByResidual = [&source, &target, &correspondences,
            inlierThreshold](const Eigen::Matrix4d& candidateTransformation)
        {
            pcl::Correspondences filtered;
            filtered.reserve(correspondences.size());
            for (const pcl::Correspondence& correspondence : correspondences)
            {
                const auto& sourcePoint = source->points[correspondence.index_query];
                const auto& targetPoint = target->points[correspondence.index_match];
                const Eigen::Vector3d transformedSource =
                    candidateTransformation.block<3, 3>(0, 0) *
                        Eigen::Vector3d(sourcePoint.x, sourcePoint.y, sourcePoint.z) +
                    candidateTransformation.block<3, 1>(0, 3);
                const double residual =
                    (transformedSource - Eigen::Vector3d(
                        targetPoint.x, targetPoint.y, targetPoint.z)).norm();
                if (residual < inlierThreshold)
                    filtered.emplace_back(correspondence);
            }
            return filtered;
        };

        Eigen::Matrix4d refinedAlignedTransformation = solver.getTransformation();
        pcl::Correspondences residualInliers =
            filterByResidual(refinedAlignedTransformation);
        if (residualInliers.size() >= 3)
        {
            Eigen::Vector3d sourceCentroid = Eigen::Vector3d::Zero();
            Eigen::Vector3d targetCentroid = Eigen::Vector3d::Zero();
            for (const pcl::Correspondence& correspondence : residualInliers)
            {
                const auto& sourcePoint = source->points[correspondence.index_query];
                const auto& targetPoint = target->points[correspondence.index_match];
                sourceCentroid += Eigen::Vector3d(
                    sourcePoint.x, sourcePoint.y, sourcePoint.z);
                targetCentroid += Eigen::Vector3d(
                    targetPoint.x, targetPoint.y, targetPoint.z);
            }
            sourceCentroid /= static_cast<double>(residualInliers.size());
            targetCentroid /= static_cast<double>(residualInliers.size());

            double dotSum = 0.0;
            double crossSum = 0.0;
            for (const pcl::Correspondence& correspondence : residualInliers)
            {
                const auto& sourcePoint = source->points[correspondence.index_query];
                const auto& targetPoint = target->points[correspondence.index_match];
                const double sourceX = sourcePoint.x - sourceCentroid.x();
                const double sourceY = sourcePoint.y - sourceCentroid.y();
                const double targetX = targetPoint.x - targetCentroid.x();
                const double targetY = targetPoint.y - targetCentroid.y();
                dotSum += sourceX * targetX + sourceY * targetY;
                crossSum += sourceX * targetY - sourceY * targetX;
            }

            if (std::hypot(dotSum, crossSum) >
                std::numeric_limits<double>::epsilon())
            {
                const double yaw = std::atan2(crossSum, dotSum);
                Eigen::Matrix3d refinedRotation = Eigen::Matrix3d::Identity();
                refinedRotation(0, 0) = std::cos(yaw);
                refinedRotation(0, 1) = -std::sin(yaw);
                refinedRotation(1, 0) = std::sin(yaw);
                refinedRotation(1, 1) = std::cos(yaw);
                refinedAlignedTransformation.setIdentity();
                refinedAlignedTransformation.block<3, 3>(0, 0) = refinedRotation;
                refinedAlignedTransformation.block<3, 1>(0, 3) =
                    targetCentroid - refinedRotation * sourceCentroid;
                residualInliers = filterByResidual(refinedAlignedTransformation);
            }
        }

        result.estimatedTransformation = refinedAlignedTransformation * alignment;
        result.retainedCount = residualInliers.size();
        result.truePositiveCount = 0;
        result.falsePositiveCount = 0;
        for (const pcl::Correspondence& correspondence : residualInliers)
        {
            const std::size_t index =
                static_cast<std::size_t>(correspondence.index_query);
            if (index < inlierLabels.size() && inlierLabels[index])
                ++result.truePositiveCount;
            else
                ++result.falsePositiveCount;
        }
        result.precision = result.retainedCount == 0
            ? 0.0
            : static_cast<double>(result.truePositiveCount) /
                result.retainedCount;
        result.recall = result.groundTruthInlierCount == 0
            ? 0.0
            : static_cast<double>(result.truePositiveCount) /
                result.groundTruthInlierCount;
    }

    const Eigen::Matrix3d rotationDifference =
        result.estimatedTransformation.block<3, 3>(0, 0) *
        groundTruthRotation.transpose();
    const double cosine = std::clamp(
        (rotationDifference.trace() - 1.0) * 0.5, -1.0, 1.0);
    result.rotationErrorDegrees = std::acos(cosine) * 180.0 / M_PI;
    result.translationError =
        (result.estimatedTransformation.block<3, 1>(0, 3) -
            groundTruthTranslation).norm();
    result.valid =
        std::isfinite(result.rotationErrorDegrees) &&
        std::isfinite(result.translationError);
    result.successful =
        result.valid &&
        result.rotationErrorDegrees < rotationSuccessThresholdDegrees &&
        result.translationError < translationSuccessThreshold;
    return result;
}

void writeTransformationLog(std::ofstream& output, const PairResult& result)
{
    if (!result.valid)
        return;
    output << result.firstFragment << '\t'
        << result.secondFragment << '\t'
        << result.fragmentCount << '\n';
    output << std::scientific << std::setprecision(8);
    for (int row = 0; row < 4; ++row)
    {
        for (int column = 0; column < 4; ++column)
        {
            if (column > 0)
                output << '\t';
            output << result.estimatedTransformation(row, column);
        }
        output << '\n';
    }
}

void addResult(SceneSummary& summary, const PairResult& result)
{
    ++summary.pairCount;
    summary.groundTruthInlierCount += result.groundTruthInlierCount;
    summary.truePositiveCount += result.truePositiveCount;
    summary.falsePositiveCount += result.falsePositiveCount;
    summary.runtimeMillisecondsSum += result.runtimeMilliseconds;
    summary.precisionSum += result.precision;
    summary.recallSum += result.recall;
    if (!result.valid)
        return;
    ++summary.validCount;
    if (result.successful)
        ++summary.successCount;
    summary.rotationErrorSum += result.rotationErrorDegrees;
    summary.translationErrorSum += result.translationError;
    summary.rotationErrors.emplace_back(result.rotationErrorDegrees);
    summary.translationErrors.emplace_back(result.translationError);
}

void writeSummaryRow(std::ofstream& output, const SceneSummary& summary)
{
    const double validCount = static_cast<double>(summary.validCount);
    const double pairCount = static_cast<double>(summary.pairCount);
    const double retainedCount = static_cast<double>(
        summary.truePositiveCount + summary.falsePositiveCount);
    output << summary.scene << ',' << summary.pairCount << ',' << summary.validCount << ','
        << (pairCount > 0.0 ? summary.validCount / pairCount : 0.0) << ','
        << (validCount > 0.0
            ? summary.rotationErrorSum / validCount
            : std::numeric_limits<double>::quiet_NaN()) << ','
        << median(summary.rotationErrors) << ','
        << (validCount > 0.0
            ? summary.translationErrorSum / validCount
            : std::numeric_limits<double>::quiet_NaN()) << ','
        << median(summary.translationErrors) << ','
        << (pairCount > 0.0 ? summary.precisionSum / pairCount : 0.0) << ','
        << (pairCount > 0.0 ? summary.recallSum / pairCount : 0.0) << ','
        << (retainedCount > 0.0
            ? summary.truePositiveCount / retainedCount
            : 0.0) << ','
        << (summary.groundTruthInlierCount > 0
            ? static_cast<double>(summary.truePositiveCount) /
                summary.groundTruthInlierCount
            : 0.0) << ','
        << (pairCount > 0.0 ? summary.successCount / pairCount : 0.0) << ','
        << (pairCount > 0.0 ? summary.runtimeMillisecondsSum / pairCount : 0.0)
        << '\n';
}
}

int runThreeDMatchExperiment(
    const std::string& correspondenceDirectory,
    const std::string& evaluationDirectory,
    double resolution,
    double angleResolutionDegrees,
    double inlierThreshold,
    const std::string& outputDirectory,
    bool enableFinalResidualFilter)
{
    if (resolution <= 0.0 || angleResolutionDegrees <= 0.0 || inlierThreshold <= 0.0)
    {
        std::cerr << "3DMatch thresholds must be positive." << std::endl;
        return 2;
    }

    const fs::path correspondenceRoot(correspondenceDirectory);
    const fs::path evaluationRoot(evaluationDirectory);
    if (!fs::is_directory(correspondenceRoot) || !fs::is_directory(evaluationRoot))
    {
        std::cerr << "3DMatch input directory does not exist." << std::endl;
        return 2;
    }

    const fs::path outputRoot(outputDirectory);
    fs::create_directories(outputRoot);
    std::ofstream pairOutput(outputRoot / "pair_results.csv");
    std::ofstream summaryOutput(outputRoot / "scene_summary.csv");
    if (!pairOutput || !summaryOutput)
    {
        std::cerr << "Cannot create 3DMatch output files." << std::endl;
        return 2;
    }

    pairOutput
        << "scene,fragment_1,fragment_2,initial_count,gt_inliers,initial_inlier_ratio,"
        << "stage1_n,stage1_tp,stage1_fp,stage2_n,stage2_tp,stage2_fp,"
        << "stage3_n,stage3_tp,stage3_fp,stage4_n,stage4_tp,stage4_fp,"
        << "retained_count,tp,fp,precision,recall,rotation_error_deg,translation_error_m,"
        << "runtime_ms,valid,success\n";
    summaryOutput
        << "scene,pairs,valid_pairs,valid_rate,mean_re_deg,median_re_deg,mean_te_m,"
        << "median_te_m,mean_pair_precision,mean_pair_recall,aggregate_precision,"
        << "aggregate_recall,success_rate,mean_runtime_ms\n";

    std::vector<fs::path> groundTruthLogs;
    for (const fs::directory_entry& entry :
        fs::recursive_directory_iterator(evaluationRoot))
    {
        if (entry.is_regular_file() && entry.path().filename() == "gt.log")
            groundTruthLogs.emplace_back(entry.path());
    }
    std::sort(groundTruthLogs.begin(), groundTruthLogs.end());
    if (groundTruthLogs.empty())
    {
        std::cerr << "No extracted gt.log files found under: "
            << evaluationRoot << std::endl;
        return 2;
    }

    SceneSummary overall;
    overall.scene = "ALL";
    std::size_t missingFiles = 0;
    pairOutput << std::setprecision(10);
    summaryOutput << std::setprecision(10);

    for (const fs::path& logPath : groundTruthLogs)
    {
        std::string scene = logPath.parent_path().filename().string();
        const std::string suffix = "-evaluation";
        if (scene.size() >= suffix.size() &&
            scene.compare(scene.size() - suffix.size(), suffix.size(), suffix) == 0)
            scene.erase(scene.size() - suffix.size());

        const std::vector<GroundTruthPair> pairs = readGroundTruthLog(logPath);
        SceneSummary sceneSummary;
        sceneSummary.scene = scene;
        std::ofstream transformationOutput(outputRoot / (scene + ".log"));

        std::cout << "3DMatch scene: " << scene
            << ", pairs=" << pairs.size() << std::endl;
        for (std::size_t pairIndex = 0; pairIndex < pairs.size(); ++pairIndex)
        {
            const GroundTruthPair& groundTruth = pairs[pairIndex];
            const fs::path correspondencePath = correspondenceRoot /
                (scene + "_cloud_bin_" + std::to_string(groundTruth.firstFragment) +
                    "_cloud_bin_" + std::to_string(groundTruth.secondFragment) + ".txt");
            if (!fs::is_regular_file(correspondencePath))
            {
                ++missingFiles;
                continue;
            }

            const PairResult result = evaluatePair(
                scene,
                groundTruth,
                correspondencePath,
                resolution,
                angleResolutionDegrees,
                inlierThreshold,
                enableFinalResidualFilter);
            addResult(sceneSummary, result);
            addResult(overall, result);
            writeTransformationLog(transformationOutput, result);

            pairOutput << result.scene << ',' << result.firstFragment << ','
                << result.secondFragment << ',' << result.initialCount << ','
                << result.groundTruthInlierCount << ',' << result.initialInlierRatio << ',';
            for (std::size_t stage = 0; stage < 4; ++stage)
            {
                pairOutput << result.stageRetainedCounts[stage] << ','
                    << result.stageTruePositiveCounts[stage] << ','
                    << result.stageFalsePositiveCounts[stage] << ',';
            }
            pairOutput << result.retainedCount << ',' << result.truePositiveCount << ','
                << result.falsePositiveCount << ',' << result.precision << ','
                << result.recall << ',' << result.rotationErrorDegrees << ','
                << result.translationError << ',' << result.runtimeMilliseconds << ','
                << (result.valid ? 1 : 0) << ',' << (result.successful ? 1 : 0) << '\n';

            if ((pairIndex + 1) % 25 == 0 || pairIndex + 1 == pairs.size())
                std::cout << "  completed " << pairIndex + 1 << '/'
                    << pairs.size() << std::endl;
        }
        writeSummaryRow(summaryOutput, sceneSummary);
    }

    writeSummaryRow(summaryOutput, overall);
    std::cout << "3DMatch evaluation complete. Pairs=" << overall.pairCount
        << ", valid=" << overall.validCount
        << ", missing correspondence files=" << missingFiles << '\n'
        << "Outputs: " << fs::absolute(outputRoot).string() << std::endl;
    return missingFiles == 0 ? 0 : 3;
}


