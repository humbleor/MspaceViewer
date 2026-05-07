#ifndef _PREPARATION_H_
#define _PREPARATION_H_

//pcl
#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/transformation_estimation_svd.h> 
#include <pcl/registration/transformation_estimation_2D.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/common/centroid.h>

using PointCloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;
using IndicesPtr = pcl::PointIndicesPtr;
using FPFHPtr = pcl::PointCloud<pcl::FPFHSignature33>::Ptr;


#include <vector>
#include <utility>
#include <algorithm>
#include <chrono>

// interval stabbing algorithm

// left point is true, right point is false.

struct Interval
{
	float value;
	bool is_left;
};

// add new property

struct IntervalW
{
	float value;
	bool is_left;
	unsigned int index;
};

using Intervals = std::vector<Interval>;
using IntervalsW = std::vector<IntervalW>;
using IntervalsWSet = std::vector<IntervalsW>;

using Edge2D = std::pair<Eigen::Vector2f, Eigen::Vector2f>;
using Edge2DSet = std::vector<Edge2D>;

#endif // !_PREPARATION_H_

