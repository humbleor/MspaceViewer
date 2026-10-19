#include "../include/GravityExperiment.h"

#include "Decoupling.h"
#include "Preparation.h"

#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <numeric>
#include <random>
#include <vector>

namespace
{
constexpr std::size_t pointCount = 1000;
constexpr std::size_t trialCount = 100;
constexpr double noiseSigma = 0.005;
constexpr double rotationSuccessThresholdDegrees = 3.0;
constexpr double translationSuccessThreshold = 0.5;
constexpr unsigned int experimentSeed = 20260905U;

constexpr std::array<double, 11> gravityErrorsDegrees{
    0.0, 0.5, 1.0, 1.5, 2.0, 2.5,
    3.0, 3.5, 4.0, 4.5, 5.0
};

struct SimulationInstance
{
    PointCloudPtr source;
    PointCloudPtr target;
    pcl::Correspondences correspondences;
    Eigen::Matrix4d groundTruth{ Eigen::Matrix4d::Identity() };
    double perturbationDirectionRadians{ 0.0 };
};

struct ErrorMetrics
{
    double rotationErrorDegrees{ 0.0 };
    double translationError{ 0.0 };
    bool success{ false };
};

struct MetricAccumulator
{
    double rotationErrorSum{ 0.0 };
    double translationErrorSum{ 0.0 };
    std::size_t successCount{ 0 };
};

SimulationInstance generateInstance(
    std::mt19937& randomEngine,
    double outlierRatio)
{
    std::uniform_real_distribution<double> coordinateDistribution(-1.0, 1.0);
    std::uniform_real_distribution<double> transformationDistribution(-1.0, 1.0);
    std::uniform_real_distribution<double> yawDistribution(-M_PI, M_PI);
    std::uniform_real_distribution<double> phiDistribution(0.0, 2.0 * M_PI);
    std::normal_distribution<double> noiseDistribution(0.0, noiseSigma);

    SimulationInstance instance;
    instance.source.reset(new pcl::PointCloud<pcl::PointXYZ>);
    instance.target.reset(new pcl::PointCloud<pcl::PointXYZ>);
    instance.source->reserve(pointCount);
    instance.target->reserve(pointCount);
    instance.correspondences.reserve(pointCount);

    const double yaw = yawDistribution(randomEngine);
    const double cosYaw = std::cos(yaw);
    const double sinYaw = std::sin(yaw);
    instance.groundTruth(0, 0) = cosYaw;
    instance.groundTruth(0, 1) = -sinYaw;
    instance.groundTruth(1, 0) = sinYaw;
    instance.groundTruth(1, 1) = cosYaw;
    instance.groundTruth(0, 3) = transformationDistribution(randomEngine);
    instance.groundTruth(1, 3) = transformationDistribution(randomEngine);
    instance.groundTruth(2, 3) = transformationDistribution(randomEngine);
    instance.perturbationDirectionRadians = phiDistribution(randomEngine);

    const std::size_t inlierCount = static_cast<std::size_t>(
        std::llround(pointCount * (1.0 - outlierRatio)));
    std::vector<bool> isInlier(pointCount, false);
    for (std::size_t i = 0; i < inlierCount; ++i)
        isInlier[i] = true;
    std::shuffle(isInlier.begin(), isInlier.end(), randomEngine);

    for (std::size_t i = 0; i < pointCount; ++i)
    {
        const Eigen::Vector3d basePoint(
            coordinateDistribution(randomEngine),
            coordinateDistribution(randomEngine),
            coordinateDistribution(randomEngine));

        Eigen::Vector3d sourcePoint = basePoint;
        sourcePoint.x() += noiseDistribution(randomEngine);
        sourcePoint.y() += noiseDistribution(randomEngine);
        sourcePoint.z() += noiseDistribution(randomEngine);

        Eigen::Vector3d targetPoint;
        if (isInlier[i])
        {
            targetPoint = basePoint;
        }
        else
        {
            targetPoint = Eigen::Vector3d(
                coordinateDistribution(randomEngine),
                coordinateDistribution(randomEngine),
                coordinateDistribution(randomEngine));
        }

        targetPoint.x() += noiseDistribution(randomEngine);
        targetPoint.y() += noiseDistribution(randomEngine);
        targetPoint.z() += noiseDistribution(randomEngine);
        targetPoint =
            instance.groundTruth.block<3, 3>(0, 0) * targetPoint +
            instance.groundTruth.block<3, 1>(0, 3);

        instance.source->emplace_back(
            static_cast<float>(sourcePoint.x()),
            static_cast<float>(sourcePoint.y()),
            static_cast<float>(sourcePoint.z()));
        instance.target->emplace_back(
            static_cast<float>(targetPoint.x()),
            static_cast<float>(targetPoint.y()),
            static_cast<float>(targetPoint.z()));

        pcl::Correspondence correspondence;
        correspondence.index_query = static_cast<int>(i);
        correspondence.index_match = static_cast<int>(i);
        correspondence.distance = 0.0f;
        instance.correspondences.emplace_back(correspondence);
    }

    instance.source->width = static_cast<std::uint32_t>(pointCount);
    instance.source->height = 1;
    instance.source->is_dense = true;
    instance.target->width = static_cast<std::uint32_t>(pointCount);
    instance.target->height = 1;
    instance.target->is_dense = true;
    return instance;
}

Eigen::Vector3d perturbGravity(double errorDegrees, double phi)
{
    const double alpha = errorDegrees * M_PI / 180.0;
    return Eigen::Vector3d(
        std::sin(alpha) * std::cos(phi),
        std::sin(alpha) * std::sin(phi),
        std::cos(alpha));
}

ErrorMetrics evaluate(
    const Eigen::Matrix4d& estimated,
    const Eigen::Matrix4d& groundTruth,
    bool solverValid)
{
    const Eigen::Matrix4d errorTransformation =
        groundTruth * estimated.inverse();
    const Eigen::Matrix3d rotationError =
        errorTransformation.block<3, 3>(0, 0);
    const double cosine = std::clamp(
        (rotationError.trace() - 1.0) * 0.5,
        -1.0,
        1.0);

    ErrorMetrics metrics;
    metrics.rotationErrorDegrees = std::acos(cosine) * 180.0 / M_PI;
    metrics.translationError =
        errorTransformation.block<3, 1>(0, 3).norm();
    metrics.success = solverValid &&
        metrics.rotationErrorDegrees < rotationSuccessThresholdDegrees &&
        metrics.translationError < translationSuccessThreshold;
    return metrics;
}
}

int runGravitySensitivityExperiment(
    double outlierRatio,
    double resolution)
{
    if (!std::isfinite(outlierRatio) ||
        outlierRatio < 0.0 ||
        outlierRatio >= 1.0 ||
        !std::isfinite(resolution) ||
        resolution <= 0.0f)
    {
        std::cerr
            << "Invalid experiment parameters: eta must be in [0, 1), "
            << "and resolution must be positive." << std::endl;
        return 2;
    }

    std::mt19937 randomEngine(experimentSeed);
    std::vector<SimulationInstance> instances;
    instances.reserve(trialCount);
    for (std::size_t trial = 0; trial < trialCount; ++trial)
        instances.emplace_back(
            generateInstance(randomEngine, outlierRatio));

    std::ofstream detailFile("gravity_sensitivity_detail.csv");
    std::ofstream summaryFile("gravity_sensitivity_summary.csv");
    if (!detailFile || !summaryFile)
    {
        std::cerr << "Cannot create gravity sensitivity result files." << std::endl;
        return 3;
    }

    detailFile
        << "trial,outlier_ratio,resolution,phi_rad,gravity_error_deg,"
        << "gravity_x,gravity_y,gravity_z,"
        << "rotation_error_deg,translation_error,success\n";
    summaryFile
        << "outlier_ratio,resolution,gravity_error_deg,mean_rotation_error_deg,"
        << "mean_translation_error,success_rate_percent\n";

    std::array<MetricAccumulator, gravityErrorsDegrees.size()> accumulated{};

    for (std::size_t trial = 0; trial < instances.size(); ++trial)
    {
        const SimulationInstance& instance = instances[trial];
        for (std::size_t angleIndex = 0;
            angleIndex < gravityErrorsDegrees.size();
            ++angleIndex)
        {
            const double gravityError = gravityErrorsDegrees[angleIndex];
            const Eigen::Vector3d perturbedGravity =
                perturbGravity(
                    gravityError,
                    instance.perturbationDirectionRadians);

            Decoupling solver(
                instance.source,
                instance.target,
                instance.correspondences,
                static_cast<float>(resolution),
                perturbedGravity,
                false);

            const ErrorMetrics metrics =
                evaluate(
                    solver.getTransformation(),
                    instance.groundTruth,
                    solver.isValid());
            MetricAccumulator& accumulator = accumulated[angleIndex];
            accumulator.rotationErrorSum += metrics.rotationErrorDegrees;
            accumulator.translationErrorSum += metrics.translationError;
            if (metrics.success)
                ++accumulator.successCount;

            detailFile << std::setprecision(10)
                << trial << ','
                << outlierRatio << ','
                << resolution << ','
                << instance.perturbationDirectionRadians << ','
                << gravityError << ','
                << perturbedGravity.x() << ','
                << perturbedGravity.y() << ','
                << perturbedGravity.z() << ','
                << metrics.rotationErrorDegrees << ','
                << metrics.translationError << ','
                << (metrics.success ? 1 : 0) << '\n';
        }
    }

    std::cout << std::fixed << std::setprecision(6)
        << "N=" << pointCount
        << ", eta=" << outlierRatio
        << ", sigma=" << noiseSigma
        << ", resolution=" << resolution << '\n'
        << "Gravity error (deg), Mean RE (deg), Mean TE, Success Rate (%)\n";
    for (std::size_t angleIndex = 0;
        angleIndex < gravityErrorsDegrees.size();
        ++angleIndex)
    {
        const MetricAccumulator& accumulator = accumulated[angleIndex];
        const double meanRotationError =
            accumulator.rotationErrorSum / static_cast<double>(trialCount);
        const double meanTranslationError =
            accumulator.translationErrorSum / static_cast<double>(trialCount);
        const double successRate =
            100.0 * static_cast<double>(accumulator.successCount) /
            static_cast<double>(trialCount);

        summaryFile << std::setprecision(10)
            << outlierRatio << ','
            << resolution << ','
            << gravityErrorsDegrees[angleIndex] << ','
            << meanRotationError << ','
            << meanTranslationError << ','
            << successRate << '\n';

        std::cout
            << gravityErrorsDegrees[angleIndex] << ", "
            << meanRotationError << ", "
            << meanTranslationError << ", "
            << successRate << '\n';
    }

    std::cout
        << "Saved gravity_sensitivity_detail.csv and "
        << "gravity_sensitivity_summary.csv" << std::endl;
    return 0;
}
