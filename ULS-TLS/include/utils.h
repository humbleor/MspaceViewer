#pragma once

#include "PointCloud.hpp"
#include "KDTreePointCloudAdaptor.hpp"
#include "PCAEstimation.hpp"

using Point2f = Point<float, 2>;
using Box2f = Box<float, 2>;
using PointCloud2f = PointCloud<float, 2>;
using PointCloud2fPtr = std::shared_ptr<PointCloud2f>;
using KDTree2f = KDTreePointCloudAdaptor<float, 2>;
using KDTree2fPtr = std::shared_ptr<KDTree2f>;

using Point2d = Point<double, 2>;
using Box2d = Box<double, 2>;
using PointCloud2d = PointCloud<double, 2>;
using PointCloud2dPtr = std::shared_ptr<PointCloud2d>;
using KDTree2d = KDTreePointCloudAdaptor<double, 2>;
using KDTree2dPtr = std::shared_ptr<KDTree2d>;

using Point3f = Point<float, 3>;
using Box3f = Box<float, 3>;
using PointCloud3f = PointCloud<float, 3>;
using PointCloud3fPtr = std::shared_ptr<PointCloud3f>;
using KDTree3f = KDTreePointCloudAdaptor<float, 3>;
using KDTree3fPtr = std::shared_ptr<KDTree3f>;
using PCAEstimator3f = PCAEstimator<float, 3>;

using Point3d = Point<double, 3>;
using Box3d = Box<double, 3>;
using PointCloud3d = PointCloud<double, 3>;
using PointCloud3dPtr = std::shared_ptr<PointCloud3d>;
using KDTree3d = KDTreePointCloudAdaptor<double, 3>;
using KDTree3dPtr = std::shared_ptr<KDTree3d>;
using PCAEstimator3d = PCAEstimator<double, 3>;

// 用于 unordered_map 的 hash 函数
struct pair_hash {
    std::size_t operator()(const std::pair<int, int>& p) const noexcept {
        return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
    }
};
