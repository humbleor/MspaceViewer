#pragma once

#include <string>
#include <functional>
#include <limits>
#include "dll_export.h"
#include "Preparation.h"

namespace d4pcr::global
{
struct PipelineOptions
{
    double resolution{ 0.05 };
    double angleResolutionDegrees{ 5.0 };
    int mutualNeighborCount{ 5 };
    int minimumInitialCorrespondences{ 20 };
    int minimumD4PcrInliers{ 15 };
    double minimumD4PcrInlierRatio{ 0.001 };
    double d4PcrResidualThreshold{ 0.10 };
    // Greedy angle pruning can discard true matches before the best anchor is visited.
    bool enableAngleCandidatePruning{ false };
    bool refineCorrespondenceConsensus{ true };

    // Large TLS pairs can need more than 50 updates at the unchanged 1e-6 tolerance.
    int icpMaximumIterations{ 50 };
    int minimumIcpCorrespondences{ 30 };
    double icpMaximumCorrespondenceDistance{ 0.15 };
    double minimumIcpOverlap{ 0.05 };
    double icpTransformationTolerance{ 1e-6 };

    double cycleRotationThresholdDegrees{ 5.0 };
    double cycleTranslationThreshold{ 0.20 };
    int minimumCyclesForRejection{ 2 };
    double cycleFailureRatioThreshold{ 0.60 };

    double gravitySigmaRadians{ 1e-3 };
    double yawSigmaRadians{ 2.0 * 3.14159265358979323846 / 180.0 };
    double minimumTranslationSigma{ 0.01 };
    double robustHuberThreshold{ 1.345 };
};

using ProgressCallback = std::function<void(int, const std::string&)>;

struct IcpResult
{
    bool converged{ false };
    Eigen::Matrix4d transformation{ Eigen::Matrix4d::Identity() };
    std::size_t correspondenceCount{ 0 };
    double overlap{ 0.0 };
    double rmse{ std::numeric_limits<double>::infinity() };
    int iterations{ 0 };
};

DLL_EXPORT IcpResult refineFourDofIcp(
    const PointCloudPtr& source,
    const PointCloudPtr& target,
    const Eigen::Matrix4d& initialTransformation,
    const PipelineOptions& options = {});

DLL_EXPORT int runGlobalOptimizationPipeline(
    const std::string& pointCloudDirectory,
    const std::string& outputDirectory,
    const PipelineOptions& options = {},
    const ProgressCallback& progress = {});

DLL_EXPORT int runGlobalOptimizationSelfTest();

DLL_EXPORT int runPairwiseDiagnostics(
    const std::string& sourcePath,
    const std::string& targetPath,
    const std::string& outputDirectory,
    const PipelineOptions& options = {});
}
