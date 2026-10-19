#include "../include/GlobalOptimizationPipeline.h"

#include "../include/Decoupling.h"
#include "../include/Preparation.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/AttitudeFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <liblas/liblas.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <optional>
#include <queue>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace d4pcr::global
{
namespace
{
namespace fs = std::filesystem;

constexpr double pi = 3.14159265358979323846;

enum class EdgeState
{
    RejectedByD4Pcr,
    RejectedByInitialCycle,
    RejectedByIcp,
    RejectedByFinalCycle,
    Reliable
};

struct FeatureCloud
{
    int id{ -1 };
    fs::path path;
    PointCloudPtr original{ new pcl::PointCloud<pcl::PointXYZ> };
    PointCloudPtr aligned{ new pcl::PointCloud<pcl::PointXYZ> };
    PointCloudPtr sampled{ new pcl::PointCloud<pcl::PointXYZ> };
    FPFHPtr descriptors{ new pcl::PointCloud<pcl::FPFHSignature33> };
    std::vector<int> descriptorPointIndices;
    Eigen::Matrix4d alignedFromOriginal{ Eigen::Matrix4d::Identity() };
};

struct ResidualStatistics
{
    std::size_t inlierCount{ 0 };
    double inlierRatio{ 0.0 };
    double rmse{ std::numeric_limits<double>::infinity() };
};

struct PairwiseEdge
{
    int source{ -1 };
    int target{ -1 };
    std::size_t initialCorrespondenceCount{ 0 };
    std::size_t d4PcrInlierCount{ 0 };
    double d4PcrInlierRatio{ 0.0 };
    double d4PcrRmse{ std::numeric_limits<double>::infinity() };
    std::size_t unrefinedInlierCount{ 0 };
    std::array<std::size_t, 4> stageCounts{};
    std::string rejectionReason{ "insufficient_initial_correspondences" };
    Eigen::Matrix4d d4PcrTransformation{ Eigen::Matrix4d::Identity() };
    IcpResult icp;
    Eigen::Matrix4d transformation{ Eigen::Matrix4d::Identity() };
    EdgeState state{ EdgeState::RejectedByD4Pcr };
    int initialCycleCount{ 0 };
    int initialFailedCycleCount{ 0 };
    int finalCycleCount{ 0 };
    int finalFailedCycleCount{ 0 };
    bool hasGroundTruth{ false };
    Eigen::Matrix4d groundTruthTransformation{ Eigen::Matrix4d::Identity() };
    double d4PcrRotationErrorDegrees{ std::numeric_limits<double>::infinity() };
    double d4PcrTranslationError{ std::numeric_limits<double>::infinity() };
    double icpRotationErrorDegrees{ std::numeric_limits<double>::infinity() };
    double icpTranslationError{ std::numeric_limits<double>::infinity() };
    double optimizedRotationErrorDegrees{ std::numeric_limits<double>::infinity() };
    double optimizedTranslationError{ std::numeric_limits<double>::infinity() };
};

struct CyclePassResult
{
    std::size_t totalCycles{ 0 };
    std::size_t failedCycles{ 0 };
    std::vector<std::size_t> rejectedEdges;
};

double radians(double degrees)
{
    return degrees * pi / 180.0;
}

double degrees(double radiansValue)
{
    return radiansValue * 180.0 / pi;
}

double normalizeAngle(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

double yawFromRotation(const Eigen::Matrix3d& rotation)
{
    return std::atan2(rotation(1, 0), rotation(0, 0));
}

Eigen::Matrix4d makePose(double x, double y, double z, double yaw)
{
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pose(0, 0) = std::cos(yaw);
    pose(0, 1) = -std::sin(yaw);
    pose(1, 0) = std::sin(yaw);
    pose(1, 1) = std::cos(yaw);
    pose(0, 3) = x;
    pose(1, 3) = y;
    pose(2, 3) = z;
    return pose;
}

Eigen::Matrix4d projectToFourDof(const Eigen::Matrix4d& transformation)
{
    const double yaw = yawFromRotation(transformation.block<3, 3>(0, 0));
    return makePose(
        transformation(0, 3),
        transformation(1, 3),
        transformation(2, 3),
        yaw);
}

double rotationErrorDegrees(const Eigen::Matrix4d& transformation)
{
    const Eigen::Matrix3d rotation = transformation.block<3, 3>(0, 0);
    const double cosine = std::clamp((rotation.trace() - 1.0) * 0.5, -1.0, 1.0);
    return degrees(std::acos(cosine));
}

std::pair<double, double> transformationErrors(
    const Eigen::Matrix4d& estimated,
    const Eigen::Matrix4d& groundTruth)
{
    Eigen::Matrix4d rotationDelta = Eigen::Matrix4d::Identity();
    rotationDelta.block<3, 3>(0, 0) =
        estimated.block<3, 3>(0, 0).transpose() *
        groundTruth.block<3, 3>(0, 0);
    const double rotationError = rotationErrorDegrees(rotationDelta);
    const double translationError =
        (estimated.block<3, 1>(0, 3) -
            groundTruth.block<3, 1>(0, 3)).norm();
    return { rotationError, translationError };
}

std::string stateName(EdgeState state)
{
    switch (state)
    {
    case EdgeState::RejectedByD4Pcr:
        return "rejected_d4pcr";
    case EdgeState::RejectedByInitialCycle:
        return "rejected_initial_cycle";
    case EdgeState::RejectedByIcp:
        return "rejected_icp";
    case EdgeState::RejectedByFinalCycle:
        return "rejected_final_cycle";
    case EdgeState::Reliable:
        return "reliable";
    }
    return "unknown";
}

std::string pointCloudExtension(const fs::path& path)
{
    std::string extension = path.extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(),
        [](unsigned char character) { return static_cast<char>(std::tolower(character)); });
    return extension;
}

PointCloudPtr loadPointCloud(const fs::path& path)
{
    PointCloudPtr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    const std::string extension = pointCloudExtension(path);
    int status = -1;
    if (extension == ".pcd")
        status = pcl::io::loadPCDFile(path.string(), *cloud);
    else if (extension == ".ply")
        status = pcl::io::loadPLYFile(path.string(), *cloud);
    else if (extension == ".las")
    {
        std::ifstream stream(path, std::ios::binary);
        if (!stream)
            throw std::runtime_error("Cannot open LAS point cloud: " + path.string());
        try
        {
            liblas::ReaderFactory factory;
            liblas::Reader reader = factory.CreateWithStream(stream);
            cloud->reserve(reader.GetHeader().GetPointRecordsCount());
            while (reader.ReadNextPoint())
            {
                const auto& point = reader.GetPoint();
                cloud->emplace_back(static_cast<float>(point.GetX()),
                    static_cast<float>(point.GetY()), static_cast<float>(point.GetZ()));
            }
            status = 0;
        }
        catch (const std::exception& error)
        {
            throw std::runtime_error("Cannot read LAS point cloud " + path.string() +
                ": " + error.what());
        }
    }
    if (status < 0)
        throw std::runtime_error("Cannot load point cloud: " + path.string());

    std::vector<int> retainedIndices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, retainedIndices);
    PointCloudPtr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    filtered->reserve(cloud->size());
    for (const auto& point : cloud->points)
    {
        if (std::isfinite(point.x) && std::isfinite(point.y) &&
            std::isfinite(point.z) &&
            std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z) > 1e-7f)
            filtered->emplace_back(point);
    }
    filtered->width = static_cast<std::uint32_t>(filtered->size());
    filtered->height = 1;
    filtered->is_dense = true;
    return filtered;
}

std::unordered_map<std::string, Eigen::Vector3d> readGravityDirections(
    const fs::path& directory)
{
    std::unordered_map<std::string, Eigen::Vector3d> directions;
    const fs::path path = directory / "gravity.csv";
    if (!fs::is_regular_file(path))
        return directions;

    std::ifstream input(path);
    std::string line;
    while (std::getline(input, line))
    {
        if (line.empty())
            continue;
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream stream(line);
        std::string name;
        Eigen::Vector3d gravity;
        if (!(stream >> name >> gravity.x() >> gravity.y() >> gravity.z()))
            continue;
        if (gravity.norm() > 1e-12)
            directions[name] = gravity.normalized();
    }
    return directions;
}

Eigen::Matrix4d makeGravityAlignment(const Eigen::Vector3d& gravity)
{
    if (!gravity.allFinite() || gravity.norm() <= 1e-12)
        throw std::runtime_error("Gravity direction is invalid.");
    const Eigen::Quaterniond rotation = Eigen::Quaterniond::FromTwoVectors(
        gravity.normalized(), Eigen::Vector3d::UnitZ());
    Eigen::Matrix4d alignment = Eigen::Matrix4d::Identity();
    alignment.block<3, 3>(0, 0) = rotation.normalized().toRotationMatrix();
    return alignment;
}

PointCloudPtr voxelDownsample(const PointCloudPtr& input, double resolution)
{
    PointCloudPtr output(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(input);
    const float leaf = static_cast<float>(resolution);
    filter.setLeafSize(leaf, leaf, leaf);
    filter.filter(*output);
    return output;
}

bool descriptorFinite(const pcl::FPFHSignature33& descriptor)
{
    for (float value : descriptor.histogram)
        if (!std::isfinite(value))
            return false;
    return true;
}

void computeFpfhFeatures(FeatureCloud& featureCloud, double resolution)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss;
    iss.setSearchMethod(tree);
    iss.setSalientRadius(6.0 * resolution);
    iss.setNonMaxRadius(2.0 * resolution);
    iss.setThreshold21(0.99);
    iss.setThreshold32(0.2);
    iss.setMinNeighbors(4);
    iss.setNumberOfThreads(1);
    iss.setInputCloud(featureCloud.sampled);
    pcl::PointCloud<pcl::PointXYZ> keypoints;
    iss.compute(keypoints);
    const pcl::PointIndicesConstPtr keypointIndices = iss.getKeypointsIndices();
    if (!keypointIndices || keypointIndices->indices.empty())
        return;

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(featureCloud.sampled);
    normalEstimation.setSearchMethod(tree);
    normalEstimation.setRadiusSearch(3.0 * resolution);
    normalEstimation.compute(*normals);

    pcl::PointIndicesPtr indices(new pcl::PointIndices(*keypointIndices));
    FPFHPtr rawDescriptors(new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> estimation;
    estimation.setInputCloud(featureCloud.sampled);
    estimation.setInputNormals(normals);
    estimation.setSearchMethod(tree);
    estimation.setRadiusSearch(10.0 * resolution);
    estimation.setNumberOfThreads(1);
    estimation.setIndices(indices);
    estimation.compute(*rawDescriptors);

    const std::size_t count = std::min(
        rawDescriptors->size(), keypointIndices->indices.size());
    featureCloud.descriptors->reserve(count);
    featureCloud.descriptorPointIndices.reserve(count);
    for (std::size_t index = 0; index < count; ++index)
    {
        if (!descriptorFinite(rawDescriptors->points[index]))
            continue;
        featureCloud.descriptors->emplace_back(rawDescriptors->points[index]);
        featureCloud.descriptorPointIndices.emplace_back(
            keypointIndices->indices[index]);
    }
    featureCloud.descriptors->width =
        static_cast<std::uint32_t>(featureCloud.descriptors->size());
    featureCloud.descriptors->height = 1;
    featureCloud.descriptors->is_dense = true;
}

pcl::Correspondences buildMutualTopKCorrespondences(
    const FeatureCloud& source,
    const FeatureCloud& target,
    int requestedNeighborCount)
{
    pcl::Correspondences correspondences;
    if (source.descriptors->empty() || target.descriptors->empty())
        return correspondences;

    const int sourceK = std::min<int>(
        requestedNeighborCount, static_cast<int>(source.descriptors->size()));
    const int targetK = std::min<int>(
        requestedNeighborCount, static_cast<int>(target.descriptors->size()));
    pcl::KdTreeFLANN<pcl::FPFHSignature33> sourceTree;
    pcl::KdTreeFLANN<pcl::FPFHSignature33> targetTree;
    sourceTree.setInputCloud(source.descriptors);
    targetTree.setInputCloud(target.descriptors);

    std::vector<int> targetIndices(targetK);
    std::vector<float> targetDistances(targetK);
    for (int sourceDescriptorIndex = 0;
        sourceDescriptorIndex < static_cast<int>(source.descriptors->size());
        ++sourceDescriptorIndex)
    {
        const int targetFound = targetTree.nearestKSearch(
            source.descriptors->points[sourceDescriptorIndex],
            targetK,
            targetIndices,
            targetDistances);
        for (int targetRank = 0; targetRank < targetFound; ++targetRank)
        {
            const int targetDescriptorIndex = targetIndices[targetRank];
            std::vector<int> reverseIndices(sourceK);
            std::vector<float> reverseDistances(sourceK);
            const int reverseFound = sourceTree.nearestKSearch(
                target.descriptors->points[targetDescriptorIndex],
                sourceK,
                reverseIndices,
                reverseDistances);
            const bool reciprocal = std::find(
                reverseIndices.begin(),
                reverseIndices.begin() + reverseFound,
                sourceDescriptorIndex) != reverseIndices.begin() + reverseFound;
            if (!reciprocal)
                continue;

            pcl::Correspondence correspondence;
            correspondence.index_query =
                source.descriptorPointIndices[sourceDescriptorIndex];
            correspondence.index_match =
                target.descriptorPointIndices[targetDescriptorIndex];
            correspondence.distance = targetDistances[targetRank];
            correspondences.emplace_back(correspondence);
        }
    }
    return correspondences;
}

ResidualStatistics evaluateCorrespondenceResiduals(
    const PointCloudPtr& source,
    const PointCloudPtr& target,
    const pcl::Correspondences& correspondences,
    const Eigen::Matrix4d& sourceToTarget,
    double threshold)
{
    ResidualStatistics result;
    double squaredErrorSum = 0.0;
    for (const auto& correspondence : correspondences)
    {
        if (correspondence.index_query < 0 || correspondence.index_match < 0 ||
            correspondence.index_query >= static_cast<int>(source->size()) ||
            correspondence.index_match >= static_cast<int>(target->size()))
            continue;
        const auto& sourcePoint = source->points[correspondence.index_query];
        const auto& targetPoint = target->points[correspondence.index_match];
        const Eigen::Vector3d transformed =
            sourceToTarget.block<3, 3>(0, 0) *
                Eigen::Vector3d(sourcePoint.x, sourcePoint.y, sourcePoint.z) +
            sourceToTarget.block<3, 1>(0, 3);
        const double residual = (transformed - Eigen::Vector3d(
            targetPoint.x, targetPoint.y, targetPoint.z)).norm();
        if (residual <= threshold)
        {
            ++result.inlierCount;
            squaredErrorSum += residual * residual;
        }
    }
    result.inlierRatio = correspondences.empty()
        ? 0.0
        : static_cast<double>(result.inlierCount) / correspondences.size();
    if (result.inlierCount > 0)
        result.rmse = std::sqrt(squaredErrorSum / result.inlierCount);
    return result;
}

Eigen::Matrix4d refineCorrespondenceConsensus(
    const PointCloudPtr& source, const PointCloudPtr& target,
    const pcl::Correspondences& correspondences,
    const Eigen::Matrix4d& initial, double threshold)
{
    Eigen::Matrix4d best = initial;
    ResidualStatistics bestStatistics = evaluateCorrespondenceResiduals(
        source, target, correspondences, best, threshold);
    for (int iteration = 0; iteration < 10; ++iteration)
    {
        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> inliers;
        Eigen::Vector3d sourceMean = Eigen::Vector3d::Zero();
        Eigen::Vector3d targetMean = Eigen::Vector3d::Zero();
        for (const auto& correspondence : correspondences)
        {
            if (correspondence.index_query < 0 || correspondence.index_match < 0 ||
                correspondence.index_query >= static_cast<int>(source->size()) ||
                correspondence.index_match >= static_cast<int>(target->size()))
                continue;
            const Eigen::Vector3d p = source->points[correspondence.index_query].getVector3fMap().cast<double>();
            const Eigen::Vector3d q = target->points[correspondence.index_match].getVector3fMap().cast<double>();
            if ((best.block<3, 3>(0, 0) * p + best.block<3, 1>(0, 3) - q).norm() > threshold)
                continue;
            inliers.emplace_back(p, q);
            sourceMean += p;
            targetMean += q;
        }
        if (inliers.size() < 3)
            break;
        sourceMean /= static_cast<double>(inliers.size());
        targetMean /= static_cast<double>(inliers.size());
        double dot = 0.0, cross = 0.0;
        for (const auto& [p, q] : inliers)
        {
            const Eigen::Vector3d a = p - sourceMean;
            const Eigen::Vector3d b = q - targetMean;
            dot += a.x() * b.x() + a.y() * b.y();
            cross += a.x() * b.y() - a.y() * b.x();
        }
        if (std::hypot(dot, cross) < 1e-12)
            break;
        Eigen::Matrix4d candidate = makePose(0, 0, 0, std::atan2(cross, dot));
        candidate.block<3, 1>(0, 3) = targetMean - candidate.block<3, 3>(0, 0) * sourceMean;
        const auto statistics = evaluateCorrespondenceResiduals(
            source, target, correspondences, candidate, threshold);
        if (statistics.inlierCount < bestStatistics.inlierCount ||
            (statistics.inlierCount == bestStatistics.inlierCount && statistics.rmse >= bestStatistics.rmse))
            break;
        best = candidate;
        bestStatistics = statistics;
    }
    return best;
}

IcpResult runFourDofIcp(
    const PointCloudPtr& source,
    const PointCloudPtr& target,
    const Eigen::Matrix4d& initialTransformation,
    const PipelineOptions& options)
{
    IcpResult result;
    result.transformation = projectToFourDof(initialTransformation);
    if (source->empty() || target->empty())
        return result;

    pcl::KdTreeFLANN<pcl::PointXYZ> targetTree;
    targetTree.setInputCloud(target);
    const double maximumSquaredDistance =
        options.icpMaximumCorrespondenceDistance *
        options.icpMaximumCorrespondenceDistance;
    std::vector<int> indices(1);
    std::vector<float> distances(1);

    for (int iteration = 0; iteration < options.icpMaximumIterations; ++iteration)
    {
        std::vector<Eigen::Vector3d> sourcePoints;
        std::vector<Eigen::Vector3d> targetPoints;
        std::vector<double> squaredDistances;
        sourcePoints.reserve(source->size());
        targetPoints.reserve(source->size());
        squaredDistances.reserve(source->size());

        for (const auto& sourcePoint : source->points)
        {
            const Eigen::Vector3d original(
                sourcePoint.x, sourcePoint.y, sourcePoint.z);
            const Eigen::Vector3d transformed =
                result.transformation.block<3, 3>(0, 0) * original +
                result.transformation.block<3, 1>(0, 3);
            pcl::PointXYZ query(
                static_cast<float>(transformed.x()),
                static_cast<float>(transformed.y()),
                static_cast<float>(transformed.z()));
            if (targetTree.nearestKSearch(query, 1, indices, distances) != 1 ||
                distances[0] > maximumSquaredDistance)
                continue;
            const auto& targetPoint = target->points[indices[0]];
            sourcePoints.emplace_back(original);
            targetPoints.emplace_back(targetPoint.x, targetPoint.y, targetPoint.z);
            squaredDistances.emplace_back(distances[0]);
        }

        if (sourcePoints.size() <
            static_cast<std::size_t>(options.minimumIcpCorrespondences))
            return result;

        Eigen::Vector3d sourceCentroid = Eigen::Vector3d::Zero();
        Eigen::Vector3d targetCentroid = Eigen::Vector3d::Zero();
        for (std::size_t index = 0; index < sourcePoints.size(); ++index)
        {
            sourceCentroid += sourcePoints[index];
            targetCentroid += targetPoints[index];
        }
        sourceCentroid /= static_cast<double>(sourcePoints.size());
        targetCentroid /= static_cast<double>(targetPoints.size());

        double dotSum = 0.0;
        double crossSum = 0.0;
        for (std::size_t index = 0; index < sourcePoints.size(); ++index)
        {
            const Eigen::Vector3d sourceCentered =
                sourcePoints[index] - sourceCentroid;
            const Eigen::Vector3d targetCentered =
                targetPoints[index] - targetCentroid;
            dotSum += sourceCentered.x() * targetCentered.x() +
                sourceCentered.y() * targetCentered.y();
            crossSum += sourceCentered.x() * targetCentered.y() -
                sourceCentered.y() * targetCentered.x();
        }
        if (std::hypot(dotSum, crossSum) <=
            std::numeric_limits<double>::epsilon())
            return result;

        const double yaw = std::atan2(crossSum, dotSum);
        Eigen::Matrix4d updated = makePose(0.0, 0.0, 0.0, yaw);
        updated.block<3, 1>(0, 3) =
            targetCentroid - updated.block<3, 3>(0, 0) * sourceCentroid;

        const Eigen::Matrix4d correction =
            updated * result.transformation.inverse();
        const double translationChange = correction.block<3, 1>(0, 3).norm();
        const double rotationChange = std::abs(normalizeAngle(
            yawFromRotation(correction.block<3, 3>(0, 0))));
        result.transformation = updated;
        result.correspondenceCount = sourcePoints.size();
        result.overlap = static_cast<double>(sourcePoints.size()) / source->size();
        result.rmse = std::sqrt(std::accumulate(
            squaredDistances.begin(), squaredDistances.end(), 0.0) /
            squaredDistances.size());
        result.iterations = iteration + 1;
        if (iteration == 0 || result.iterations % 5 == 0)
            std::cout << "ICP iteration " << result.iterations
                << ": translation_change=" << translationChange
                << ", yaw_change_rad=" << rotationChange
                << ", rmse=" << result.rmse << ", overlap=" << result.overlap << std::endl;

        if (translationChange <= options.icpTransformationTolerance &&
            rotationChange <= options.icpTransformationTolerance)
        {
            result.converged = true;
            break;
        }
    }

    // 用最终变换重新建立最近邻，确保输出的重叠率与 RMSE 对应最终解，
    // 而不是收敛前一次迭代所使用的对应关系。
    std::size_t finalCorrespondenceCount = 0;
    double finalSquaredError = 0.0;
    for (const auto& sourcePoint : source->points)
    {
        const Eigen::Vector3d original(
            sourcePoint.x, sourcePoint.y, sourcePoint.z);
        const Eigen::Vector3d transformed =
            result.transformation.block<3, 3>(0, 0) * original +
            result.transformation.block<3, 1>(0, 3);
        pcl::PointXYZ query(
            static_cast<float>(transformed.x()),
            static_cast<float>(transformed.y()),
            static_cast<float>(transformed.z()));
        if (targetTree.nearestKSearch(query, 1, indices, distances) != 1 ||
            distances[0] > maximumSquaredDistance)
            continue;
        ++finalCorrespondenceCount;
        finalSquaredError += distances[0];
    }
    result.correspondenceCount = finalCorrespondenceCount;
    result.overlap = static_cast<double>(finalCorrespondenceCount) / source->size();
    result.rmse = finalCorrespondenceCount == 0
        ? std::numeric_limits<double>::infinity()
        : std::sqrt(finalSquaredError / finalCorrespondenceCount);
    return result;
}

std::uint64_t edgeKey(int first, int second)
{
    const auto low = static_cast<std::uint32_t>(std::min(first, second));
    const auto high = static_cast<std::uint32_t>(std::max(first, second));
    return (static_cast<std::uint64_t>(low) << 32U) | high;
}

std::optional<Eigen::Matrix4d> directedTransformation(
    int from,
    int to,
    const std::vector<PairwiseEdge>& edges,
    const std::unordered_map<std::uint64_t, std::size_t>& lookup,
    bool useIcp)
{
    const auto iterator = lookup.find(edgeKey(from, to));
    if (iterator == lookup.end())
        return std::nullopt;
    const PairwiseEdge& edge = edges[iterator->second];
    if (edge.state != EdgeState::Reliable)
        return std::nullopt;
    const Eigen::Matrix4d& sourceToTarget = useIcp
        ? edge.icp.transformation
        : edge.d4PcrTransformation;
    if (edge.source == from && edge.target == to)
        return sourceToTarget;
    return sourceToTarget.inverse();
}

CyclePassResult applyCycleConsistency(
    int nodeCount,
    std::vector<PairwiseEdge>& edges,
    const PipelineOptions& options,
    bool useIcp,
    EdgeState rejectionState)
{
    CyclePassResult pass;
    std::unordered_map<std::uint64_t, std::size_t> lookup;
    for (std::size_t index = 0; index < edges.size(); ++index)
        if (edges[index].state == EdgeState::Reliable)
            lookup[edgeKey(edges[index].source, edges[index].target)] = index;

    std::vector<int> cycleCounts(edges.size(), 0);
    std::vector<int> failedCounts(edges.size(), 0);
    for (int i = 0; i < nodeCount; ++i)
    {
        for (int j = i + 1; j < nodeCount; ++j)
        {
            for (int k = j + 1; k < nodeCount; ++k)
            {
                const auto iToJ = directedTransformation(i, j, edges, lookup, useIcp);
                const auto jToK = directedTransformation(j, k, edges, lookup, useIcp);
                const auto kToI = directedTransformation(k, i, edges, lookup, useIcp);
                if (!iToJ || !jToK || !kToI)
                    continue;
                ++pass.totalCycles;
                const Eigen::Matrix4d closure = *kToI * *jToK * *iToJ;
                const bool failed =
                    rotationErrorDegrees(closure) >
                        options.cycleRotationThresholdDegrees ||
                    closure.block<3, 1>(0, 3).norm() >
                        options.cycleTranslationThreshold;
                const std::array<std::size_t, 3> edgeIndices{
                    lookup.at(edgeKey(i, j)),
                    lookup.at(edgeKey(j, k)),
                    lookup.at(edgeKey(i, k))
                };
                for (std::size_t edgeIndex : edgeIndices)
                {
                    ++cycleCounts[edgeIndex];
                    if (failed)
                        ++failedCounts[edgeIndex];
                }
                if (failed)
                    ++pass.failedCycles;
            }
        }
    }

    for (std::size_t index = 0; index < edges.size(); ++index)
    {
        PairwiseEdge& edge = edges[index];
        if (useIcp)
        {
            edge.finalCycleCount = cycleCounts[index];
            edge.finalFailedCycleCount = failedCounts[index];
        }
        else
        {
            edge.initialCycleCount = cycleCounts[index];
            edge.initialFailedCycleCount = failedCounts[index];
        }
        if (edge.state != EdgeState::Reliable ||
            cycleCounts[index] < options.minimumCyclesForRejection)
            continue;
        const double failureRatio = static_cast<double>(failedCounts[index]) /
            cycleCounts[index];
        if (failureRatio >= options.cycleFailureRatioThreshold)
        {
            edge.state = rejectionState;
            edge.rejectionReason = stateName(rejectionState);
            pass.rejectedEdges.emplace_back(index);
        }
    }
    return pass;
}

gtsam::Pose3 toGtsamPose(const Eigen::Matrix4d& transformation)
{
    return gtsam::Pose3(transformation);
}

std::vector<Eigen::Matrix4d> buildInitialGlobalPoses(
    int nodeCount,
    const std::vector<PairwiseEdge>& edges,
    std::vector<bool>& reachable)
{
    std::vector<Eigen::Matrix4d> poses(
        nodeCount, Eigen::Matrix4d::Identity());
    reachable.assign(nodeCount, false);
    if (nodeCount == 0)
        return poses;
    reachable[0] = true;
    std::queue<int> pending;
    pending.push(0);
    while (!pending.empty())
    {
        const int current = pending.front();
        pending.pop();
        for (const PairwiseEdge& edge : edges)
        {
            if (edge.state != EdgeState::Reliable)
                continue;
            if (edge.source == current && !reachable[edge.target])
            {
                poses[edge.target] =
                    poses[current] * edge.transformation.inverse();
                reachable[edge.target] = true;
                pending.push(edge.target);
            }
            else if (edge.target == current && !reachable[edge.source])
            {
                poses[edge.source] = poses[current] * edge.transformation;
                reachable[edge.source] = true;
                pending.push(edge.source);
            }
        }
    }
    return poses;
}

std::vector<Eigen::Matrix4d> optimizePoseGraph(
    int nodeCount,
    const std::vector<PairwiseEdge>& edges,
    const PipelineOptions& options,
    std::vector<bool>& reachable)
{
    std::vector<Eigen::Matrix4d> initial =
        buildInitialGlobalPoses(nodeCount, edges, reachable);
    if (nodeCount == 0 || !reachable[0])
        return initial;

    std::cout << "Building GTSAM pose graph ..." << std::endl;
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values values;
    const auto key = [](int node) { return gtsam::Symbol('x', node); };

    const gtsam::Vector6 priorSigmas = (gtsam::Vector6() <<
        1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished();
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
        key(0), gtsam::Pose3(), gtsam::noiseModel::Diagonal::Sigmas(priorSigmas));

    std::cout << "GTSAM prior ready." << std::endl;
    const auto gravityNoise = gtsam::noiseModel::Isotropic::Sigma(
        2, options.gravitySigmaRadians);
    for (int node = 0; node < nodeCount; ++node)
    {
        if (!reachable[node])
            continue;
        values.insert(key(node), toGtsamPose(initial[node]));
        graph.emplace_shared<gtsam::Pose3AttitudeFactor>(
            key(node),
            gtsam::Unit3(Eigen::Vector3d::UnitZ()),
            gravityNoise,
            gtsam::Unit3(Eigen::Vector3d::UnitZ()));
    }

    for (const PairwiseEdge& edge : edges)
    {
        if (edge.state != EdgeState::Reliable ||
            !reachable[edge.source] || !reachable[edge.target])
            continue;
        const double translationSigma = std::max(
            options.minimumTranslationSigma,
            std::isfinite(edge.icp.rmse)
                ? edge.icp.rmse
                : options.minimumTranslationSigma);
        const gtsam::Vector6 sigmas = (gtsam::Vector6() <<
            options.gravitySigmaRadians,
            options.gravitySigmaRadians,
            options.yawSigmaRadians,
            translationSigma,
            translationSigma,
            translationSigma).finished();
        const auto baseNoise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
        const auto robustNoise = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(
                options.robustHuberThreshold),
            baseNoise);
        const Eigen::Matrix4d sourceFromTarget = edge.transformation.inverse();
        graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            key(edge.source),
            key(edge.target),
            toGtsamPose(sourceFromTarget),
            robustNoise);
    }

    std::cout << "GTSAM factors ready: " << graph.size() << std::endl;
    gtsam::LevenbergMarquardtParams parameters;
    parameters.setVerbosityLM("SILENT");
    parameters.setMaxIterations(100);
    const gtsam::Values optimized =
        gtsam::LevenbergMarquardtOptimizer(graph, values, parameters).optimize();
    std::cout << "GTSAM optimization completed." << std::endl;

    std::vector<Eigen::Matrix4d> result = initial;
    for (int node = 0; node < nodeCount; ++node)
    {
        if (reachable[node])
            result[node] = optimized.at<gtsam::Pose3>(key(node)).matrix();
    }
    return result;
}

std::ofstream openOutput(const fs::path& path)
{
    errno = 0;
    std::ofstream output(path);
    if (!output)
        throw std::system_error(errno ? errno : EIO, std::generic_category(),
            "Cannot create output: " + path.string());
    return output;
}

void writeEdges(
    const fs::path& path,
    const std::vector<PairwiseEdge>& edges,
    const std::vector<FeatureCloud>& clouds)
{
    auto output = openOutput(path);
    output << "source,target,source_file,target_file,state,initial_correspondences,d4pcr_inliers,"
        "d4pcr_inlier_ratio,d4pcr_rmse,icp_converged,icp_correspondences,"
        "icp_overlap,icp_rmse,icp_iterations,initial_cycles,"
        "initial_failed_cycles,final_cycles,final_failed_cycles,ground_truth,"
        "d4pcr_re_deg,d4pcr_te,icp_re_deg,icp_te,optimized_re_deg,optimized_te,"
        "unrefined_inliers,stage1_count,stage2_count,stage3_count,stage4_count,rejection_reason\n";
    output << std::setprecision(10);
    for (const PairwiseEdge& edge : edges)
    {
        output << edge.source << ',' << edge.target
            << ',' << clouds[edge.source].path.filename().string()
            << ',' << clouds[edge.target].path.filename().string()
            << ',' << stateName(edge.state)
            << ',' << edge.initialCorrespondenceCount
            << ',' << edge.d4PcrInlierCount
            << ',' << edge.d4PcrInlierRatio
            << ',' << edge.d4PcrRmse
            << ',' << (edge.icp.converged ? 1 : 0)
            << ',' << edge.icp.correspondenceCount
            << ',' << edge.icp.overlap
            << ',' << edge.icp.rmse
            << ',' << edge.icp.iterations
            << ',' << edge.initialCycleCount
            << ',' << edge.initialFailedCycleCount
            << ',' << edge.finalCycleCount
            << ',' << edge.finalFailedCycleCount
            << ',' << (edge.hasGroundTruth ? 1 : 0)
            << ',' << edge.d4PcrRotationErrorDegrees
            << ',' << edge.d4PcrTranslationError
            << ',' << edge.icpRotationErrorDegrees
            << ',' << edge.icpTranslationError
            << ',' << edge.optimizedRotationErrorDegrees
            << ',' << edge.optimizedTranslationError
            << ',' << edge.unrefinedInlierCount;
        for (const auto count : edge.stageCounts)
            output << ',' << count;
        output << ',' << edge.rejectionReason << '\n';
    }
}

void writeParameters(const fs::path& path, const PipelineOptions& options)
{
    auto output = openOutput(path);
    output << std::setprecision(10)
        << "resolution=" << options.resolution << '\n'
        << "angle_resolution_degrees=" << options.angleResolutionDegrees << '\n'
        << "angle_candidate_pruning=" << options.enableAngleCandidatePruning << '\n'
        << "refine_correspondence_consensus=" << options.refineCorrespondenceConsensus << '\n'
        << "mutual_neighbor_count=" << options.mutualNeighborCount << '\n'
        << "minimum_initial_correspondences=" << options.minimumInitialCorrespondences << '\n'
        << "minimum_d4pcr_inliers=" << options.minimumD4PcrInliers << '\n'
        << "minimum_d4pcr_inlier_ratio=" << options.minimumD4PcrInlierRatio << '\n'
        << "d4pcr_residual_threshold=" << options.d4PcrResidualThreshold << '\n'
        << "icp_maximum_iterations=" << options.icpMaximumIterations << '\n'
        << "minimum_icp_correspondences=" << options.minimumIcpCorrespondences << '\n'
        << "icp_maximum_correspondence_distance=" << options.icpMaximumCorrespondenceDistance << '\n'
        << "minimum_icp_overlap=" << options.minimumIcpOverlap << '\n'
        << "cycle_rotation_threshold_degrees=" << options.cycleRotationThresholdDegrees << '\n'
        << "cycle_translation_threshold=" << options.cycleTranslationThreshold << '\n'
        << "minimum_cycles_for_rejection=" << options.minimumCyclesForRejection << '\n'
        << "cycle_failure_ratio_threshold=" << options.cycleFailureRatioThreshold << '\n';
}

void writePoses(
    const fs::path& path,
    const std::vector<FeatureCloud>& clouds,
    const std::vector<Eigen::Matrix4d>& worldFromAligned,
    const std::vector<bool>& reachable)
{
    std::ofstream output(path);
    if (!output)
        throw std::runtime_error("Cannot create pose output: " + path.string());
    output << "node,file,connected";
    for (int row = 0; row < 4; ++row)
        for (int column = 0; column < 4; ++column)
            output << ",m" << row << column;
    output << '\n' << std::setprecision(12);
    for (std::size_t node = 0; node < clouds.size(); ++node)
    {
        const Eigen::Matrix4d worldFromOriginal =
            worldFromAligned[node] * clouds[node].alignedFromOriginal;
        output << node << ',' << clouds[node].path.filename().string()
            << ',' << (reachable[node] ? 1 : 0);
        for (int row = 0; row < 4; ++row)
            for (int column = 0; column < 4; ++column)
                output << ',' << worldFromOriginal(row, column);
        output << '\n';
    }
}

void writeMergedCloud(
    const fs::path& path,
    const std::vector<FeatureCloud>& clouds,
    const std::vector<Eigen::Matrix4d>& worldFromAligned,
    const std::vector<bool>& reachable,
    double resolution)
{
    PointCloudPtr merged(new pcl::PointCloud<pcl::PointXYZ>);
    for (std::size_t node = 0; node < clouds.size(); ++node)
    {
        if (!reachable[node])
            continue;
        const Eigen::Matrix4d worldFromOriginal =
            worldFromAligned[node] * clouds[node].alignedFromOriginal;
        pcl::PointCloud<pcl::PointXYZ> transformed;
        pcl::transformPointCloud(
            *clouds[node].original,
            transformed,
            worldFromOriginal.cast<float>());
        *merged += transformed;
    }
    PointCloudPtr sampled = voxelDownsample(merged, resolution);
    if (pcl::io::savePCDFileBinary(path.string(), *sampled) < 0)
        throw std::runtime_error("Cannot write merged cloud: " + path.string());
}

std::vector<fs::path> discoverPointClouds(const fs::path& directory)
{
    std::vector<fs::path> paths;
    for (const fs::directory_entry& entry : fs::directory_iterator(directory))
    {
        if (!entry.is_regular_file())
            continue;
        const std::string extension = pointCloudExtension(entry.path());
        if (extension == ".pcd" || extension == ".ply" || extension == ".las")
            paths.emplace_back(entry.path());
    }
    std::sort(paths.begin(), paths.end());
    return paths;
}

std::vector<std::pair<int, int>> readRequestedPairs(
    const fs::path& directory,
    const std::vector<fs::path>& paths)
{
    std::unordered_map<std::string, int> nodeByName;
    for (std::size_t index = 0; index < paths.size(); ++index)
    {
        nodeByName[paths[index].stem().string()] = static_cast<int>(index);
        nodeByName[paths[index].filename().string()] = static_cast<int>(index);
    }

    const fs::path pairPath = directory / "pairs.txt";
    std::vector<std::pair<int, int>> pairs;
    std::set<std::uint64_t> usedPairs;
    if (fs::is_regular_file(pairPath))
    {
        std::ifstream input(pairPath);
        std::string sourceName;
        std::string targetName;
        while (input >> sourceName >> targetName)
        {
            const auto sourceIterator = nodeByName.find(sourceName);
            const auto targetIterator = nodeByName.find(targetName);
            if (sourceIterator == nodeByName.end() ||
                targetIterator == nodeByName.end())
                throw std::runtime_error(
                    "Unknown point cloud in pairs.txt: " +
                    sourceName + " " + targetName);
            if (sourceIterator->second == targetIterator->second)
                throw std::runtime_error("Self edge is not allowed in pairs.txt.");
            const std::uint64_t key = edgeKey(
                sourceIterator->second, targetIterator->second);
            if (usedPairs.insert(key).second)
                pairs.emplace_back(sourceIterator->second, targetIterator->second);
        }
        if (pairs.empty())
            throw std::runtime_error("pairs.txt contains no valid pair.");
        return pairs;
    }

    for (int source = 0; source < static_cast<int>(paths.size()); ++source)
        for (int target = source + 1;
            target < static_cast<int>(paths.size()); ++target)
            pairs.emplace_back(source, target);
    return pairs;
}

std::optional<Eigen::Matrix4d> readFirstTransformation(const fs::path& path)
{
    if (!fs::is_regular_file(path))
        return std::nullopt;
    std::ifstream input(path);
    std::string content(
        (std::istreambuf_iterator<char>(input)),
        std::istreambuf_iterator<char>());
    std::replace(content.begin(), content.end(), ',', ' ');
    std::istringstream stream(content);
    Eigen::Matrix4d transformation;
    for (int row = 0; row < 4; ++row)
        for (int column = 0; column < 4; ++column)
            if (!(stream >> transformation(row, column)))
                throw std::runtime_error(
                    "Ground-truth file does not start with a 4x4 matrix: " +
                    path.string());
    return transformation;
}

void attachGroundTruth(
    PairwiseEdge& edge,
    const FeatureCloud& source,
    const FeatureCloud& target,
    const fs::path& groundTruthDirectory)
{
    const fs::path path = groundTruthDirectory /
        (source.path.stem().string() + "-" +
            target.path.stem().string() + ".tfm");
    const auto rawGroundTruth = readFirstTransformation(path);
    if (!rawGroundTruth)
        return;
    edge.hasGroundTruth = true;
    edge.groundTruthTransformation =
        target.alignedFromOriginal * (*rawGroundTruth) *
        source.alignedFromOriginal.inverse();
    if (edge.d4PcrInlierCount > 0 && std::isfinite(edge.d4PcrRmse))
    {
        const auto errors = transformationErrors(
            edge.d4PcrTransformation, edge.groundTruthTransformation);
        edge.d4PcrRotationErrorDegrees = errors.first;
        edge.d4PcrTranslationError = errors.second;
    }
}

PairwiseEdge estimatePairwiseEdge(
    const FeatureCloud& source,
    const FeatureCloud& target,
    const PipelineOptions& options,
    const pcl::Correspondences* preparedCorrespondences = nullptr)
{
    PairwiseEdge edge;
    edge.source = source.id;
    edge.target = target.id;
    pcl::Correspondences generated;
    if (!preparedCorrespondences)
        generated = buildMutualTopKCorrespondences(source, target, options.mutualNeighborCount);
    const auto& correspondences = preparedCorrespondences ? *preparedCorrespondences : generated;
    edge.initialCorrespondenceCount = correspondences.size();
    if (correspondences.size() <
        static_cast<std::size_t>(options.minimumInitialCorrespondences))
        return edge;

    Decoupling solver(
        source.sampled,
        target.sampled,
        correspondences,
        static_cast<float>(options.resolution),
        Eigen::Vector3d::UnitZ(),
        false,
        static_cast<float>(options.angleResolutionDegrees),
        options.enableAngleCandidatePruning);
    for (std::size_t stage = 0; stage < edge.stageCounts.size(); ++stage)
        edge.stageCounts[stage] = solver.getStageCorrespondences(stage).size();
    if (!solver.isValid())
    {
        edge.rejectionReason = "invalid_d4pcr_estimate";
        return edge;
    }

    edge.d4PcrTransformation = projectToFourDof(solver.getTransformation());
    edge.unrefinedInlierCount = evaluateCorrespondenceResiduals(source.sampled,
        target.sampled, correspondences, edge.d4PcrTransformation,
        options.d4PcrResidualThreshold).inlierCount;
    if (options.refineCorrespondenceConsensus)
        edge.d4PcrTransformation = refineCorrespondenceConsensus(source.sampled,
            target.sampled, correspondences, edge.d4PcrTransformation, options.d4PcrResidualThreshold);
    const ResidualStatistics residuals = evaluateCorrespondenceResiduals(
        source.sampled,
        target.sampled,
        correspondences,
        edge.d4PcrTransformation,
        options.d4PcrResidualThreshold);
    edge.d4PcrInlierCount = residuals.inlierCount;
    edge.d4PcrInlierRatio = residuals.inlierRatio;
    edge.d4PcrRmse = residuals.rmse;
    if (edge.d4PcrInlierCount <
            static_cast<std::size_t>(options.minimumD4PcrInliers) ||
        edge.d4PcrInlierRatio < options.minimumD4PcrInlierRatio)
    {
        edge.rejectionReason = edge.d4PcrInlierCount < static_cast<std::size_t>(options.minimumD4PcrInliers)
            ? "insufficient_d4pcr_inlier_count" : "insufficient_d4pcr_inlier_ratio";
        return edge;
    }

    edge.transformation = edge.d4PcrTransformation;
    edge.state = EdgeState::Reliable;
    edge.rejectionReason.clear();
    return edge;
}

bool runCycleAndGraphSyntheticTest()
{
    constexpr int nodeCount = 4;
    const std::array<Eigen::Matrix4d, nodeCount> truth{
        makePose(0.0, 0.0, 0.0, radians(0.0)),
        makePose(1.0, 0.0, 0.1, radians(8.0)),
        makePose(1.0, 1.0, 0.2, radians(17.0)),
        makePose(0.0, 1.0, 0.1, radians(25.0))
    };
    std::vector<PairwiseEdge> edges;
    for (int source = 0; source < nodeCount; ++source)
    {
        for (int target = source + 1; target < nodeCount; ++target)
        {
            PairwiseEdge edge;
            edge.source = source;
            edge.target = target;
            edge.state = EdgeState::Reliable;
            edge.icp.converged = true;
            edge.icp.rmse = 0.01;
            edge.icp.transformation = truth[target].inverse() * truth[source];
            edge.transformation = edge.icp.transformation;
            if (source == 0 && target == 3)
            {
                edge.icp.transformation =
                    makePose(0.7, -0.4, 0.2, radians(18.0)) *
                    edge.icp.transformation;
                edge.transformation = edge.icp.transformation;
            }
            edges.emplace_back(edge);
        }
    }

    PipelineOptions options;
    options.cycleRotationThresholdDegrees = 3.0;
    options.cycleTranslationThreshold = 0.10;
    options.minimumCyclesForRejection = 2;
    options.cycleFailureRatioThreshold = 0.75;
    const CyclePassResult cycle = applyCycleConsistency(
        nodeCount,
        edges,
        options,
        true,
        EdgeState::RejectedByFinalCycle);
    const auto bad = std::find_if(edges.begin(), edges.end(), [](const PairwiseEdge& edge)
    {
        return edge.source == 0 && edge.target == 3;
    });
    if (bad == edges.end() || bad->state != EdgeState::RejectedByFinalCycle ||
        cycle.failedCycles == 0)
        return false;

    std::vector<bool> reachable;
    const std::vector<Eigen::Matrix4d> optimized = optimizePoseGraph(
        nodeCount, edges, options, reachable);
    for (int node = 0; node < nodeCount; ++node)
    {
        if (!reachable[node])
            return false;
        const Eigen::Matrix4d delta = truth[node].inverse() * optimized[node];
        if (rotationErrorDegrees(delta) > 0.1 ||
            delta.block<3, 1>(0, 3).norm() > 1e-3)
            return false;
    }
    return true;
}

bool runIcpSyntheticTest()
{
    std::mt19937 generator(20261014U);
    std::uniform_real_distribution<double> coordinate(-1.0, 1.0);
    PointCloudPtr source(new pcl::PointCloud<pcl::PointXYZ>);
    for (int index = 0; index < 1000; ++index)
        source->emplace_back(
            static_cast<float>(coordinate(generator)),
            static_cast<float>(coordinate(generator)),
            static_cast<float>(coordinate(generator)));
    source->width = static_cast<std::uint32_t>(source->size());
    source->height = 1;
    source->is_dense = true;
    const Eigen::Matrix4d truth = makePose(0.35, -0.22, 0.12, radians(12.0));
    PointCloudPtr target(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*source, *target, truth.cast<float>());

    PipelineOptions options;
    options.icpMaximumCorrespondenceDistance = 0.30;
    options.minimumIcpCorrespondences = 100;
    options.minimumIcpOverlap = 0.5;
    const Eigen::Matrix4d initial =
        makePose(0.32, -0.20, 0.10, radians(10.0));
    const IcpResult result = runFourDofIcp(source, target, initial, options);
    const Eigen::Matrix4d delta = result.transformation * truth.inverse();
    return result.converged &&
        rotationErrorDegrees(delta) < 0.05 &&
        delta.block<3, 1>(0, 3).norm() < 1e-3;
}
}

IcpResult refineFourDofIcp(
    const PointCloudPtr& source,
    const PointCloudPtr& target,
    const Eigen::Matrix4d& initialTransformation,
    const PipelineOptions& options)
{
    if (!source || !target || !initialTransformation.allFinite() ||
        !std::isfinite(options.icpMaximumCorrespondenceDistance) ||
        options.icpMaximumCorrespondenceDistance <= 0.0 ||
        options.icpMaximumIterations <= 0 || options.minimumIcpCorrespondences < 3)
        throw std::invalid_argument("Invalid 4-DOF ICP input or parameters.");
    return runFourDofIcp(source, target, initialTransformation, options);
}

int runPairwiseDiagnostics(const std::string& sourcePath, const std::string& targetPath,
    const std::string& outputDirectory, const PipelineOptions& options)
{
    try
    {
        if (!std::isfinite(options.resolution) || options.resolution <= 0)
            throw std::invalid_argument("Resolution must be positive.");
        const fs::path outputRoot(outputDirectory);
        fs::create_directories(outputRoot);
        writeParameters(outputRoot / "diagnostic_parameters.txt", options);
        std::vector<FeatureCloud> clouds;
        for (const auto& name : {sourcePath, targetPath})
        {
            FeatureCloud cloud;
            cloud.id = static_cast<int>(clouds.size());
            cloud.path = fs::path(name);
            const auto cachePrefix = outputRoot / ("cloud" + std::to_string(cloud.id));
            const auto sampledPath = fs::path(cachePrefix.string() + "_sampled.pcd");
            const auto descriptorPath = fs::path(cachePrefix.string() + "_descriptors.pcd");
            const auto indexPath = fs::path(cachePrefix.string() + "_indices.txt");
            const auto stampPath = fs::path(cachePrefix.string() + "_stamp.txt");
            std::ostringstream stamp;
            stamp << "fpfh-cache-v1\n" << fs::absolute(cloud.path).string() << '\n'
                << fs::file_size(cloud.path) << '\n' << fs::last_write_time(cloud.path).time_since_epoch().count()
                << '\n' << std::setprecision(17) << options.resolution << '\n';
            std::ifstream oldStamp(stampPath);
            const std::string stored((std::istreambuf_iterator<char>(oldStamp)), std::istreambuf_iterator<char>());
            bool cached = stored == stamp.str() && fs::exists(sampledPath) && fs::exists(descriptorPath) && fs::exists(indexPath);
            if (cached)
            {
                cached = pcl::io::loadPCDFile(sampledPath.string(), *cloud.sampled) == 0 &&
                    pcl::io::loadPCDFile(descriptorPath.string(), *cloud.descriptors) == 0;
                std::ifstream indices(indexPath);
                int index;
                while (indices >> index)
                    cloud.descriptorPointIndices.push_back(index);
                cached = cached && cloud.descriptorPointIndices.size() == cloud.descriptors->size() &&
                    std::all_of(cloud.descriptorPointIndices.begin(), cloud.descriptorPointIndices.end(),
                        [&](int i) { return i >= 0 && i < static_cast<int>(cloud.sampled->size()); });
            }
            if (!cached)
            {
                cloud.descriptors->clear();
                cloud.descriptorPointIndices.clear();
                std::cout << "Reading " << name << std::endl;
                auto raw = loadPointCloud(cloud.path);
                std::cout << "Voxel sampling " << raw->size() << " points ..." << std::endl;
                cloud.sampled = voxelDownsample(raw, options.resolution);
                raw.reset();
                std::cout << "Computing ISS/normals/FPFH on " << cloud.sampled->size() << " points ..." << std::endl;
                computeFpfhFeatures(cloud, options.resolution);
                if (cloud.descriptors->empty())
                    throw std::runtime_error("No finite FPFH descriptors: " + name);
                if (pcl::io::savePCDFileBinary(sampledPath.string(), *cloud.sampled) != 0 ||
                    pcl::io::savePCDFileBinary(descriptorPath.string(), *cloud.descriptors) != 0)
                    throw std::runtime_error("Cannot write diagnostic feature cache.");
                auto indices = openOutput(indexPath);
                for (int index : cloud.descriptorPointIndices)
                    indices << index << '\n';
                indices.close();
                if (!indices)
                    throw std::runtime_error("Cannot write descriptor indices.");
                auto savedStamp = openOutput(stampPath);
                savedStamp << stamp.str();
            }
            std::cout << "Features " << cloud.id << ": sampled=" << cloud.sampled->size()
                << ", FPFH=" << cloud.descriptors->size() << ", cached=" << cached << std::endl;
            clouds.emplace_back(std::move(cloud));
        }
        std::cout << "Matching FPFH descriptors ..." << std::endl;
        const auto correspondences = buildMutualTopKCorrespondences(clouds[0], clouds[1], options.mutualNeighborCount);
        std::cout << "Correspondences: " << correspondences.size() << std::endl;
        {
            auto matches = openOutput(outputRoot / "feature_correspondences.csv");
            matches << "source_index,target_index,descriptor_distance\n";
            for (const auto& match : correspondences)
                matches << match.index_query << ',' << match.index_match << ',' << match.distance << '\n';
        }
        for (const bool baseline : {true, false})
        {
            auto mode = options;
            mode.enableAngleCandidatePruning = baseline;
            mode.refineCorrespondenceConsensus = !baseline;
            const std::string label = baseline ? "baseline" : "unpruned_refined";
            std::cout << "Estimating " << label << " ..." << std::endl;
            auto edge = estimatePairwiseEdge(clouds[0], clouds[1], mode, &correspondences);
            std::cout << label << ": inliers=" << edge.d4PcrInlierCount << ", ratio="
                << edge.d4PcrInlierRatio << ", rmse=" << edge.d4PcrRmse << ", state=" << stateName(edge.state)
                << ", reason=" << edge.rejectionReason << ", stages=";
            for (auto count : edge.stageCounts) std::cout << count << '/';
            std::cout << std::endl;
            // Only an accepted improved candidate reaches the expensive ICP check.
            // The rejected baseline would not run ICP in the production pipeline.
            if (!baseline && edge.state == EdgeState::Reliable)
            {
                std::cout << "Validating accepted candidate with ICP ..." << std::endl;
                edge.icp = runFourDofIcp(clouds[0].sampled, clouds[1].sampled,
                    edge.d4PcrTransformation, options);
                std::cout << label << " ICP: overlap=" << edge.icp.overlap << ", rmse=" << edge.icp.rmse
                    << ", converged=" << edge.icp.converged << std::endl;
                auto refinedMatrix = openOutput(outputRoot / (label + "_icp_matrix.txt"));
                refinedMatrix << std::setprecision(12);
                for (int row = 0; row < 4; ++row)
                    for (int column = 0; column < 4; ++column)
                        refinedMatrix << edge.icp.transformation(row, column) << (column == 3 ? '\n' : '\t');
                if (!edge.icp.converged || edge.icp.correspondenceCount <
                    static_cast<std::size_t>(options.minimumIcpCorrespondences) ||
                    edge.icp.overlap < options.minimumIcpOverlap ||
                    !std::isfinite(edge.icp.rmse) || edge.icp.rmse > options.icpMaximumCorrespondenceDistance)
                {
                    edge.state = EdgeState::RejectedByIcp;
                    edge.rejectionReason = "icp_validation_failed";
                }
                else
                    edge.transformation = edge.icp.transformation;
            }
            writeEdges(outputRoot / (label + "_edges.csv"), {edge}, clouds);
            auto matrix = openOutput(outputRoot / (label + "_candidate_matrix.txt"));
            matrix << std::setprecision(12);
            for (int row = 0; row < 4; ++row)
                for (int column = 0; column < 4; ++column)
                    matrix << edge.d4PcrTransformation(row, column) << (column == 3 ? '\n' : '\t');
        }
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "Pair diagnostics failed: " << error.what() << std::endl;
        return 2;
    }
}

int runGlobalOptimizationPipeline(
    const std::string& pointCloudDirectory,
    const std::string& outputDirectory,
    const PipelineOptions& options,
    const ProgressCallback& progress)
{
    try
    {
        const auto report = [&](int value, const std::string& message) {
            if (progress)
                progress(value, message);
        };
        if (!std::isfinite(options.resolution) || options.resolution <= 0.0 ||
            options.angleResolutionDegrees <= 0.0 ||
            options.mutualNeighborCount <= 0 ||
            options.icpMaximumCorrespondenceDistance <= 0.0)
            throw std::invalid_argument("Global optimization parameters must be positive.");

        const fs::path inputRoot(pointCloudDirectory);
        if (!fs::is_directory(inputRoot))
            throw std::runtime_error(
                "Point-cloud directory does not exist: " + inputRoot.string());
        if (fs::weakly_canonical(inputRoot) == fs::weakly_canonical(fs::path(outputDirectory)))
            throw std::invalid_argument("Use a separate output directory for global optimization.");
        const fs::path outputRoot(outputDirectory);
        fs::create_directories(outputRoot);
        // Fail on unwritable output before expensive feature extraction.
        writeParameters(outputRoot / "test_parameters.txt", options);
        const std::vector<fs::path> paths = discoverPointClouds(inputRoot);
        if (paths.size() < 2)
            throw std::runtime_error(
                "At least two PCD, PLY or LAS point clouds are required.");

        const auto gravityDirections = readGravityDirections(inputRoot);
        std::vector<FeatureCloud> clouds;
        clouds.reserve(paths.size());
        for (std::size_t index = 0; index < paths.size(); ++index)
        {
            report(static_cast<int>(20 * index / paths.size()),
                "Loading point cloud " + std::to_string(index + 1) + "/" +
                std::to_string(paths.size()) + ": " + paths[index].filename().string());
            FeatureCloud cloud;
            cloud.id = static_cast<int>(index);
            cloud.path = paths[index];
            cloud.original = loadPointCloud(cloud.path);
            if (cloud.original->empty())
                throw std::runtime_error("Point cloud has no finite points: " + cloud.path.string());
            const auto gravityIterator = gravityDirections.find(
                cloud.path.filename().string());
            const Eigen::Vector3d gravity = gravityIterator == gravityDirections.end()
                ? Eigen::Vector3d::UnitZ()
                : gravityIterator->second;
            cloud.alignedFromOriginal = makeGravityAlignment(gravity);
            pcl::transformPointCloud(
                *cloud.original,
                *cloud.aligned,
                cloud.alignedFromOriginal.cast<float>());
            cloud.sampled = voxelDownsample(cloud.aligned, options.resolution);
            computeFpfhFeatures(cloud, options.resolution);
            std::cout << "Node " << cloud.id << ": "
                << cloud.path.filename().string()
                << ", sampled=" << cloud.sampled->size()
                << ", FPFH=" << cloud.descriptors->size() << std::endl;
            clouds.emplace_back(std::move(cloud));
        }
        report(20, "Point-cloud features ready.");

        const std::vector<std::pair<int, int>> requestedPairs =
            readRequestedPairs(inputRoot, paths);
        std::cout << "Pair count: " << requestedPairs.size() << std::endl;
        std::vector<PairwiseEdge> edges;
        edges.reserve(requestedPairs.size());
        for (const auto& [source, target] : requestedPairs)
        {
            report(20 + static_cast<int>(35 * edges.size() / requestedPairs.size()),
                "D4PCR: " + clouds[source].path.stem().string() + " -> " +
                clouds[target].path.stem().string());
            PairwiseEdge edge = estimatePairwiseEdge(
                clouds[source], clouds[target], options);
            attachGroundTruth(
                edge,
                clouds[source],
                clouds[target],
                inputRoot / "groundtruth");
            std::cout << "Pair "
                << clouds[source].path.stem().string() << " -> "
                << clouds[target].path.stem().string()
                << ": correspondences=" << edge.initialCorrespondenceCount
                << ", D4PCR inliers=" << edge.d4PcrInlierCount
                << ", ratio=" << edge.d4PcrInlierRatio
                << ", state=" << stateName(edge.state)
                << ", reason=" << edge.rejectionReason << ", stages=";
            for (const auto count : edge.stageCounts) std::cout << count << '/';
            std::cout << std::endl;
            edges.emplace_back(std::move(edge));
        }
        writeEdges(outputRoot / "pairwise_edges.csv", edges, clouds);
        if (std::none_of(edges.begin(), edges.end(), [](const PairwiseEdge& edge) {
            return edge.state == EdgeState::Reliable;
        }))
            throw std::runtime_error("All D4PCR pairs were rejected; see pairwise_edges.csv. ICP and GTSAM were not run.");

        report(55, "Checking initial cycle consistency ...");
        const CyclePassResult initialCycle = applyCycleConsistency(
            static_cast<int>(clouds.size()),
            edges,
            options,
            false,
            EdgeState::RejectedByInitialCycle);
        std::cout << "Initial cycles: " << initialCycle.totalCycles
            << ", failed=" << initialCycle.failedCycles
            << ", rejected edges=" << initialCycle.rejectedEdges.size()
            << std::endl;

        std::size_t refinedCount = 0;
        for (PairwiseEdge& edge : edges)
        {
            if (edge.state != EdgeState::Reliable)
            {
                ++refinedCount;
                continue;
            }
            report(60 + static_cast<int>(20 * refinedCount++ / edges.size()),
                "Refining pairwise registrations with 4-DOF ICP ...");
            edge.icp = runFourDofIcp(
                clouds[edge.source].sampled,
                clouds[edge.target].sampled,
                edge.d4PcrTransformation,
                options);
            if (!edge.icp.converged ||
                edge.icp.correspondenceCount <
                    static_cast<std::size_t>(options.minimumIcpCorrespondences) ||
                edge.icp.overlap < options.minimumIcpOverlap ||
                !std::isfinite(edge.icp.rmse) ||
                edge.icp.rmse > options.icpMaximumCorrespondenceDistance)
            {
                edge.state = EdgeState::RejectedByIcp;
                edge.rejectionReason = "icp_validation_failed";
                continue;
            }
            edge.transformation = edge.icp.transformation;
            if (edge.hasGroundTruth)
            {
                const auto errors = transformationErrors(
                    edge.transformation, edge.groundTruthTransformation);
                edge.icpRotationErrorDegrees = errors.first;
                edge.icpTranslationError = errors.second;
            }
        }

        report(80, "Checking refined cycle consistency ...");
        const CyclePassResult finalCycle = applyCycleConsistency(
            static_cast<int>(clouds.size()),
            edges,
            options,
            true,
            EdgeState::RejectedByFinalCycle);
        std::cout << "Final cycles: " << finalCycle.totalCycles
            << ", failed=" << finalCycle.failedCycles
            << ", rejected edges=" << finalCycle.rejectedEdges.size()
            << std::endl;

        std::vector<bool> reachable;
        buildInitialGlobalPoses(static_cast<int>(clouds.size()), edges, reachable);
        writeEdges(outputRoot / "pairwise_edges.csv", edges, clouds);
        if (static_cast<std::size_t>(std::count(reachable.begin(), reachable.end(), true)) != clouds.size())
            throw std::runtime_error("Reliable-edge graph is disconnected; see pairwise_edges.csv. GTSAM was not run.");
        report(85, "Optimizing global poses with GTSAM ...");
        const std::vector<Eigen::Matrix4d> optimizedPoses = optimizePoseGraph(
            static_cast<int>(clouds.size()), edges, options, reachable);
        for (PairwiseEdge& edge : edges)
        {
            if (!edge.hasGroundTruth || !reachable[edge.source] ||
                !reachable[edge.target])
                continue;
            const Eigen::Matrix4d optimizedSourceToTarget =
                optimizedPoses[edge.target].inverse() *
                optimizedPoses[edge.source];
            const auto errors = transformationErrors(
                optimizedSourceToTarget, edge.groundTruthTransformation);
            edge.optimizedRotationErrorDegrees = errors.first;
            edge.optimizedTranslationError = errors.second;
        }
        const std::size_t connectedCount = static_cast<std::size_t>(std::count(
            reachable.begin(), reachable.end(), true));

        report(90, "Saving global registration results ...");
        writeParameters(outputRoot / "test_parameters.txt", options);
        writeEdges(outputRoot / "pairwise_edges.csv", edges, clouds);
        if (connectedCount != clouds.size())
            throw std::runtime_error(
                "Reliable-edge graph is disconnected: " +
                std::to_string(connectedCount) + "/" +
                std::to_string(clouds.size()) + " nodes are connected to node 0.");
        writePoses(
            outputRoot / "global_poses.csv",
            clouds,
            optimizedPoses,
            reachable);
        writeMergedCloud(
            outputRoot / "merged_global.pcd",
            clouds,
            optimizedPoses,
            reachable,
            options.resolution);
        std::cout << "Global optimization outputs: "
            << fs::absolute(outputRoot).string() << std::endl;
        report(100, "Global optimization completed.");
        return 0;
    }
    catch (const std::exception& exception)
    {
        if (progress)
            progress(100, std::string("Global optimization failed: ") + exception.what());
        std::cerr << "Global optimization failed: "
            << exception.what() << std::endl;
        return 2;
    }
}

int runGlobalOptimizationSelfTest()
{
    const bool cycleAndGraphPassed = runCycleAndGraphSyntheticTest();
    const bool icpPassed = runIcpSyntheticTest();
    std::cout << "4-DOF ICP synthetic test: "
        << (icpPassed ? "PASS" : "FAIL") << std::endl;
    std::cout << "Cycle rejection and GTSAM graph test: "
        << (cycleAndGraphPassed ? "PASS" : "FAIL") << std::endl;
    return cycleAndGraphPassed && icpPassed ? 0 : 2;
}
}
