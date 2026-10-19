#pragma once

#include <string>

int runThreeDMatchExperiment(
    const std::string& correspondenceDirectory,
    const std::string& evaluationDirectory,
    double resolution = 0.1,
    double angleResolutionDegrees = 5.0,
    double inlierThreshold = 0.1,
    const std::string& outputDirectory = "outputs/3dmatch",
    bool enableFinalResidualFilter = false);


