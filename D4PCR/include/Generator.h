#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <chrono>
#include <random>

class Generator
{
public:
	Generator(unsigned int number, double outliersRatio);
	~Generator();
	pcl::PointCloud<pcl::PointXYZ>::Ptr getSource();
	pcl::PointCloud<pcl::PointXYZ>::Ptr getTarget();
	Eigen::Matrix4d getTransformation();

private:
	void randomPointGenerator(unsigned int number);
	void gaussiamNoiseGenerator();
	void outliersGenerator(double outliersRatio);
	void randomTransformation();

private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr source;
	pcl::PointCloud<pcl::PointXYZ>::Ptr target;
	Eigen::Matrix4d transformation;
};

