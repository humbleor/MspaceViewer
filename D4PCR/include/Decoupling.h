#pragma once

#include <unordered_map>
#include <unordered_set>
#include "dll_export.h"
#include "../include/Preparation.h"
#include "../include/IntervalStabbing.h"

class DLL_EXPORT Decoupling
{
public:
	Decoupling(PointCloudPtr sampledCloudS, PointCloudPtr sampledCloudT, pcl::Correspondences cors, float resolution);
	~Decoupling();
	Eigen::Matrix4d getTransformation();
	pcl::Correspondences getCors() { return corsRes; };

private:
	void decoupling(PointCloudPtr sampledCloudS, PointCloudPtr sampledCloudT, pcl::Correspondences cors, float resolution);
	void decoupling2(PointCloudPtr sampledCloudS, PointCloudPtr sampledCloudT, pcl::Correspondences cors, float resolution);
	void computeTransformationMatrix(PointCloudPtr sampledCloudS, PointCloudPtr sampledCloudT);

	pcl::Correspondences corsRes;
	Eigen::Matrix4d transformation;
};

