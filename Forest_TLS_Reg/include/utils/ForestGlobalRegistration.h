#pragma once
#include "utils/ForestPairRegistration.h"
#include <filesystem>

namespace forest {
using GlobalProgress = std::function<void(int, const std::string&)>;
struct GlobalOptions {
    ConfigSetting config = defaultForestConfig();
    double mergeResolution = 0.05; // LAS output voxel; 0 preserves all original points.
    double cycleRotationDegrees = 2.0;
    double cycleTranslationMeters = 0.5;
};
void runGlobalRegistration(const std::filesystem::path& input,
                           const std::filesystem::path& output,
                           const GlobalOptions& options = {},
                           const GlobalProgress& report = {});
int runGlobalGraphSelfTest();
} // namespace forest
