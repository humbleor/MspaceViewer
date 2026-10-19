#pragma once

#include "dst/DST.h"
#include <functional>
#include <limits>
#include <string>
#include <filesystem>

namespace forest {
struct PreparedCloud {
    FrameInfo features;
    pcl::PointCloud<pcl::PointXYZ>::Ptr sampled;
    size_t originalPointCount = 0;
};
ConfigSetting defaultForestConfig();
PreparedCloud prepareCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& raw,
                           const ConfigSetting& config);
struct GeometryQuality {
    size_t tested = 0;
    size_t inliers = 0;
    double overlap = 0;
    double rmse = std::numeric_limits<double>::infinity();
};

struct PairResult {
    bool accepted = false;
    bool converged = false;
    size_t iterations = 0;
    size_t optimizerInliers = 0;
    size_t sourceCenters = 0, targetCenters = 0;
    size_t sourceDescriptors = 0, targetDescriptors = 0, matchedTriangles = 0;
    double coarseScore = 0;
    double coarseThreshold = 0;
    Eigen::Matrix4d coarse = Eigen::Matrix4d::Identity();
    // Complete source -> target transform, including the coarse initialization.
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    GeometryQuality coarseForward, coarseReverse, finalForward, finalReverse;
    std::string reason;
};

// Metrics use deterministic subsets (at most 50k queries per direction) of
// 0.15 m voxel clouds, nearest-neighbour distance <= 0.30 m. No ground truth.
PairResult registerPair(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
                        const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
                        const ConfigSetting& config,
                        std::function<void(int, const std::string&)> report = {});
PairResult registerPreparedPair(const PreparedCloud& source, const PreparedCloud& target,
                               const ConfigSetting& config,
                               std::function<void(int, const std::string&)> report = {});
void savePairReport(const std::filesystem::path& path, const PairResult& result);
} // namespace forest
