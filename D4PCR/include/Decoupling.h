#pragma once

#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <chrono>
#include <numeric>
#include <array>
#include <memory>
#include <cmath>
#include <cstddef>
#include <limits>
#include <Eigen/Geometry>

#include "dll_export.h"
#include "../include/Preparation.h"
#include "../include/IntervalStabbing.h"

class DLL_EXPORT Decoupling
{
public:
	// 兼容原有调用方（MainWindow / OutlierRemovalRegCLI）的四参数入口。
	// 默认重力方向为 +Z、启用角度候选剪枝、不交换阶段顺序。
	Decoupling(
		PointCloudPtr sampledCloudS,
		PointCloudPtr sampledCloudT,
		pcl::Correspondences cors,
		float resolution)
		: Decoupling(
			sampledCloudS,
			sampledCloudT,
			std::move(cors),
			resolution,
			Eigen::Vector3d::UnitZ(),
			true,
			1.0f,
			true,
			false)
	{
	}

	Decoupling(
		PointCloudPtr sampledCloudS,
		PointCloudPtr sampledCloudT,
		pcl::Correspondences cors,
		float resolution,
		const Eigen::Vector3d& gravityDirection,
		bool verbose = true,
		float angleResolutionDegrees = 1.0f,
		bool enableAngleCandidatePruning = true,
		bool swapVerticalAndRotationStages = false);

	~Decoupling() = default;

	[[nodiscard]] Eigen::Matrix4d getTransformation() const noexcept { return transformation; }
	[[nodiscard]] pcl::Correspondences getCors1() const noexcept { return cors1; }
	[[nodiscard]] pcl::Correspondences getCors2() const noexcept { return cors2; }
	[[nodiscard]] pcl::Correspondences getCors3() const noexcept { return cors3; }
	[[nodiscard]] pcl::Correspondences getCors() const noexcept { return corsRes; }
	[[nodiscard]] const pcl::Correspondences& getStageCorrespondences(
		std::size_t stageIndex) const noexcept;
	[[nodiscard]] const std::array<double, 4>& getStageTimesSeconds() const noexcept
	{
		return stageTimesSeconds;
	}
	[[nodiscard]] float getHorizontalTranslationLength() const noexcept { return horizontalTranslationLength; }
	[[nodiscard]] bool isValid() const noexcept { return valid; }

private:
	void decoupling(
		PointCloudPtr sampledCloudS,
		PointCloudPtr sampledCloudT,
		const pcl::Correspondences& cors,
		float resolution,
		bool verbose,
		float angleResolutionDegrees,
		bool enableAngleCandidatePruning,
		bool swapVerticalAndRotationStages);

	pcl::Correspondences cors1;
	pcl::Correspondences cors2;
	pcl::Correspondences cors3;
	pcl::Correspondences corsRes;
	std::array<double, 4> stageTimesSeconds{ 0.0, 0.0, 0.0, 0.0 };
	float horizontalTranslationLength{ std::numeric_limits<float>::quiet_NaN() };
	bool valid{ false };
	Eigen::Matrix4d transformation{ Eigen::Matrix4d::Identity() };
};
