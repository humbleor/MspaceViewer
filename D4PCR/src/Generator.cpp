#include "../include/Generator.h"

Generator::Generator(unsigned int number, double outliersRatio)
{
	source = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	target = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	randomPointGenerator(number);
	outliersGenerator(outliersRatio);
	gaussiamNoiseGenerator();
	randomTransformation();
}

Generator::~Generator()
{
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Generator::getSource()
{
	return source;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Generator::getTarget()
{
	return target;
}

Eigen::Matrix4d Generator::getTransformation()
{
	return transformation;
}

void Generator::randomPointGenerator(unsigned int number)
{
	pcl::PointXYZ point;
	// 使用当前时间作为种子
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::default_random_engine generator(seed);
	std::uniform_real_distribution<double> distribution(-1.0, 1.0);
	for (unsigned int i = 0; i < number; i++)
	{
		point.x = distribution(generator);
		point.y = distribution(generator);
		point.z = distribution(generator);
		source->points.emplace_back(point);
		target->points.emplace_back(point);
		//std::cout << point.x << "," << point.y << "," << point.z << std::endl;
	}
}

void Generator::gaussiamNoiseGenerator()
{
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::default_random_engine generator(seed);

	std::normal_distribution<double> distribution(0.0, 0.005);
	std::uniform_real_distribution<double> angle_range(-M_PI, M_PI);  // 随机角度分布

	for (size_t i = 0; i < target->points.size(); i++)
	{
		double gnoise = distribution(generator);
		//std::cout << "gnoise:" << gnoise << std::endl;
		double theta = angle_range(generator);
		double phi = angle_range(generator);

		// 计算噪声的方向向量
		double dx = gnoise * std::sin(phi) * std::cos(theta);
		double dy = gnoise * std::sin(phi) * std::sin(theta);
		double dz = gnoise * std::cos(phi);

		// 将噪声添加到点的坐标上
		target->points[i].x += dx;
		target->points[i].y += dy;
		target->points[i].z += dz;
	}

	for (size_t i = 0; i < source->points.size(); i++)
	{
		double gnoise = distribution(generator);
		//std::cout << "gnoise:" << gnoise << std::endl;
		double theta = angle_range(generator);
		double phi = angle_range(generator);
		// 计算噪声的方向向量
		double dx = gnoise * std::sin(phi) * std::cos(theta);
		double dy = gnoise * std::sin(phi) * std::sin(theta);
		double dz = gnoise * std::cos(phi);
		// 将噪声添加到点的坐标上
		source->points[i].x += dx;
		source->points[i].y += dy;
		source->points[i].z += dz;
	}
}

void Generator::outliersGenerator(double outliersRatio)
{
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::default_random_engine generator(seed);
	std::uniform_real_distribution<double> distribution(-1.0, 1.0);
	unsigned int outliersNum = target->points.size() * outliersRatio;
	for (size_t i = target->points.size() - outliersNum; i < target->points.size(); i++)
	{
		target->points[i].x = distribution(generator);
		target->points[i].y = distribution(generator);
		target->points[i].z = distribution(generator);
		//std::cout << target->points[i].x << "," << target->points[i].y << "," << target->points[i].z << std::endl;
	}
}

void Generator::randomTransformation()
{
	transformation = Eigen::Matrix4d::Zero();
	transformation(3, 3) = 1.0;
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::default_random_engine generator(seed);
	std::uniform_real_distribution<double> distance_range(-1, 1);
	transformation(0, 3) = distance_range(generator);
	transformation(1, 3) = distance_range(generator);
	transformation(2, 3) = distance_range(generator);

	std::uniform_real_distribution<double> angle_range(-M_PI, M_PI);
	double theta = angle_range(generator);
	transformation(0, 0) = cos(theta);
	transformation(0, 1) = -sin(theta);
	transformation(1, 0) = sin(theta);
	transformation(1, 1) = cos(theta);
	transformation(2, 2) = 1.0f;

	std::cout << "Reference Matrix:" << std::endl;
	std::cout << transformation << std::endl;

	pcl::transformPointCloud(*target, *target, transformation);
}
