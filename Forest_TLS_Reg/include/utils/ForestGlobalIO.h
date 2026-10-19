#pragma once
#include "utils/ForestGlobalRegistration.h"

namespace forest::io {
using Cloud = pcl::PointCloud<pcl::PointXYZ>;
std::string utf8(const std::filesystem::path& path);
Cloud::Ptr load(const std::filesystem::path& path, const GlobalProgress& report = {});
PreparedCloud prepareCached(const std::filesystem::path& input,
                            const std::filesystem::path& cacheFile,
                            const ConfigSetting& config, const GlobalProgress& report);
uint64_t writeLas(const std::filesystem::path& output,
                 const std::vector<std::filesystem::path>& inputs,
                 const std::vector<uint64_t>& originalCounts,
                 const std::vector<Eigen::Matrix4d>& poses,
                 double resolution, const GlobalProgress& report = {});
void exportLasFromRun(const std::filesystem::path& input,
                      const std::filesystem::path& completedRun,
                      const std::filesystem::path& output,
                      double resolution, const GlobalProgress& report = {});
} // namespace forest::io
