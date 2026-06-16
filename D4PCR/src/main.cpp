#include "../include/Preparation.h"
#include "../include/RelationshipConstruction.h"
#include "../include/IntervalStabbing.h"
#include "../include/Decoupling.h"
#include "../include/Generator.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <stdio.h>
#include <boost/thread/thread.hpp>

#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <boost/random.hpp>
#include <fstream>  
#include <string>  
#include <vector> 
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/distances.h>

#include <pcl/registration/icp.h>

#include <pcl/common/transforms.h>
using namespace std;

int main()
{
	//PointCloudPtr error = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	//PointCloudPtr truth = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	//pcl::io::loadPCDFile<pcl::PointXYZ>("seq04-70-error.pcd", *error);
	//pcl::io::loadPCDFile<pcl::PointXYZ>("seq04-70-truth.pcd", *truth);
	//std::vector<float> distanceSet;
	//for (size_t i = 0; i < error->points.size(); i++)
	//{
	//	distanceSet.emplace_back(pcl::euclideanDistance(error->points[i], truth->points[i]));
	//}

	//std::ofstream datafile;
	//datafile.open("error.txt", std::ofstream::out);
	//for (size_t i = 0; i < distanceSet.size(); i++)
	//{
	//	datafile << error->points[i].x << " " << error->points[i].y << " " << error->points[i].z << " " << distanceSet[i] << std::endl;
	//}
	//datafile.close();

	float resolution = 0.05f;
	bool center = false;
	PointCloudPtr sampledCloudS(new pcl::PointCloud<pcl::PointXYZ>);
	PointCloudPtr sampledCloudT(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::Correspondences cors;
	//Real World
	std::shared_ptr<RelationshipConstruction> rc =
		std::make_shared<RelationshipConstruction>(
			"H:/genhe/diffcult/112/4.pcd", "H:/genhe/diffcult/112/1.pcd", resolution, center);

	cors = rc->getCorrespondences();
	sampledCloudS = rc->getPointCloudS();
	sampledCloudT = rc->getPointCloudT();

	//std::ofstream datafile;
	//datafile.open("G:/first_paper/code/4DOF-Registration-main/data.txt", std::ofstream::out);

	//for (size_t i = 0; i < cors.size(); i++)
	//{
	//	datafile << sampledCloudS->points[cors[i].index_query].x << " "
	//		<< sampledCloudS->points[cors[i].index_query].y << " "
	//		<< sampledCloudS->points[cors[i].index_query].z << " "
	//		<< sampledCloudT->points[cors[i].index_match].x << " "
	//		<< sampledCloudT->points[cors[i].index_match].y << " "
	//		<< sampledCloudT->points[cors[i].index_match].z << std::endl;
	//}
	//datafile.close();

	////Simulation Data
	//unsigned int number = 50000000;
	//double outliersRatio = 0.95; 
	//

	//std::vector<float> errorRSet, errorTSet, timeSet;
	//for (size_t i = 0; i < 50; i++)
	//{
	//	PointCloudPtr sampledCloudS(new pcl::PointCloud<pcl::PointXYZ>);
	//	PointCloudPtr sampledCloudT(new pcl::PointCloud<pcl::PointXYZ>);
	//	Generator generator(number, outliersRatio);
	//	sampledCloudS = generator.getSource();
	//	sampledCloudT = generator.getTarget();
	//	Eigen::Matrix4d transformation = generator.getTransformation();
	//	pcl::Correspondences cors;
	//	pcl::Correspondence cor;
	//	for (size_t i = 0; i < sampledCloudT->points.size(); i++)
	//	{
	//		cor.index_match = i;
	//		cor.index_query = i;
	//		cors.emplace_back(cor);
	//	}
	//	std::cout << cors.size() << std::endl;
	auto timeS = std::chrono::system_clock::now();
	//float threshold = 0.3;
	Decoupling decoupling(sampledCloudS, sampledCloudT, cors, resolution);
	auto timeE = std::chrono::system_clock::now();
	Eigen::Matrix4d transformation_matrix = decoupling.getTransformation();
	////transformation_matrix(2, 3) = optimalZ;

	if (center)
	{
		Eigen::Matrix3d R;
		Eigen::Vector3d t;
		R = transformation_matrix.block<3, 3>(0, 0);
		t = transformation_matrix.block<3, 1>(0, 3);

		Eigen::Matrix4d transtemp;
		transtemp.fill(0.0);

		transtemp.block<3, 3>(0, 0) = R;
		transtemp.block<3, 1>(0, 3) = - R * rc->getCenterS() + t + rc->getCenterT();
		transtemp(3, 3) = 1.0;

		transformation_matrix = transtemp;
	}

	std::cout << "transformation matrix before ICP: \n" << transformation_matrix << std::endl;

	Eigen::Matrix4d transformation;
	transformation << -0.1677002098, -0.9848748721, 0.0435674868, 0.7267793869,
		0.9857250433, -0.1668480677, 0.0225357773, 8.8133397103,
		-0.0149257698, 0.0467248174, 0.9987962819, -1.6471664799,
		0   ,         0     ,       0   ,         1;

	Eigen::Matrix4d inT = Eigen::Matrix4d::Zero();
	inT = transformation_matrix.inverse();
	Eigen::Matrix4d deltaT = transformation * inT;

	Eigen::Matrix3d rota = deltaT.block<3, 3>(0, 0);

	double trace = rota(0, 0) + rota(1, 1) + rota(2, 2) - 1;
	if (trace > 2)
		trace = 4 - trace;
	//std::cout << "trace:" << trace << std::endl;
	double errorR = acos(trace / 2);
	errorR = errorR * 180.0 / M_PI;
	std::cout << errorR << std::endl;
	float errorT = std::sqrtf(std::powf(deltaT(0, 3), 2) + std::powf(deltaT(1, 3), 2) + std::powf(deltaT(2, 3), 2));
	std::cout << errorT << std::endl;

		PointCloudPtr corS(new pcl::PointCloud<pcl::PointXYZ>);
		PointCloudPtr corT(new pcl::PointCloud<pcl::PointXYZ>);

		for (size_t i = 0; i < cors.size(); i++)
		{
			corS->points.emplace_back(sampledCloudS->points[cors[i].index_query]);
			corT->points.emplace_back(sampledCloudT->points[cors[i].index_match]);
		}
		corS->width = cors.size();
		corS->height = 1;
		corT->width = cors.size();
		corT->height = 1;

		PointCloudPtr resultS(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::transformPointCloud(*corS, *resultS, transformation);

		size_t correctNumber = 0;
		for (size_t i = 0; i < cors.size(); i++)
		{
			if (pcl::euclideanDistance(resultS->points[i], corT->points[i]) <= resolution)
			{
				correctNumber++;
			}
		}
		std::cout << "correctNumber:" << correctNumber << std::endl;
		std::cout << "outlier ratio: " << std::to_string((double)(cors.size() - correctNumber) / (double)cors.size()) << std::endl;

	//	double time = double(std::chrono::duration_cast<std::chrono::microseconds>
	//		(timeE - timeS).count()) / 1000000.0;
	//	std::cout << time << std::endl;
	//	errorRSet.emplace_back(errorR);
	//	errorTSet.emplace_back(errorT);
	//	timeSet.emplace_back(time);
	//}

	//std::ofstream datafile;
	//datafile.open("dataFile.txt", std::ofstream::out);
	//for (size_t i = 0; i < errorRSet.size(); i++)
	//{
	//	datafile << errorRSet[i] << " ";
	//}
	//datafile << std::endl;
	//for (size_t i = 0; i < errorTSet.size(); i++)
	//{
	//	datafile << errorTSet[i] << " ";
	//}
	//datafile << std::endl;
	//for (size_t i = 0; i < timeSet.size(); i++)
	//{
	//	datafile << timeSet[i] << " ";
	//}
	//datafile.close();

	//	// ����PCLVisualizer����
	//pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
	//viewer.setBackgroundColor(1.0, 1.0, 1.0); // RGBֵΪ(1,1,1)��ʾ��ɫ

	//// ���������ӵ����ӻ���
	//viewer.addPointCloud<pcl::PointXYZ>(sampledCloudS, "sample cloudS");

	//// ���õ��Ƶ���ɫ����ѡ��
	//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.67, 0.0, "sample cloudS");
	//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloudS");

	//// ���������ӵ����ӻ���
	//viewer.addPointCloud<pcl::PointXYZ>(sampledCloudT, "sample cloudT");

	//// ���õ��Ƶ���ɫ����ѡ��
	//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.67, 1.0, "sample cloudT");
	//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloudT");

	//Eigen::Matrix4d transformation;
	//transformation << 0.9809000554, - 0.1945041227, 0.0017965474, 1.5854004199,
	//	0.1944882123, 0.9808818581, 0.0067168253, 11.9651981853,
	//	- 0.0030686510, - 0.0062391270, 0.9999758280, - 0.0869550337,
	//	0, 0, 0, 1;

	//auto timeS = std::chrono::system_clock::now();
	//Decoupling decoupling(sampledCloudS, sampledCloudT, cors, resolution);
	//pcl::Correspondences corRes = decoupling.getCors();

	//for (size_t i = 0; i < cors.size(); i += 1)
	//{
	//	viewer.addLine<pcl::PointXYZ>(sampledCloudS->points[cors[i].index_query], sampledCloudT->points[cors[i].index_match], "line" + std::to_string(i));
	//	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "line" + std::to_string(i));
	//	// ����ֱ�ߵ�͸����
	//	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, "line" + std::to_string(i));  // ͸��������Ϊ0.5
	//}

	//for (size_t i = number*outliersRatio; i < cors.size(); i += 1)
	//{
	//	viewer.addLine<pcl::PointXYZ>(sampledCloudS->points[cors[i].index_query], sampledCloudT->points[cors[i].index_match], "line" + std::to_string(i));
	//	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "line" + std::to_string(i));
	//	// ����ֱ�ߵ�͸����
	//	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, "line" + std::to_string(i));  // ͸��������Ϊ0.5
	//}



	//// ��ѭ����ֱ�����ڱ��ر�
	//while (!viewer.wasStopped())
	//{
	//	viewer.spinOnce(100);
	//}





	//Eigen::Matrix4d transformation_matrix = decoupling.getTransformation();
	//auto timeE = std::chrono::system_clock::now();
	//////transformation_matrix(2, 3) = optimalZ;
	//std::cout << "transformation matrix before ICP: \n" << transformation_matrix << std::endl;

	//Eigen::Matrix4d inT = Eigen::Matrix4d::Zero();
	//inT = transformation_matrix.inverse();
	//Eigen::Matrix4d deltaT = transformation * inT;

	//Eigen::Matrix3d rota = deltaT.block<3, 3>(0, 0);

	//double trace = rota(0, 0) + rota(1, 1) + rota(2, 2) - 1;
	//std::cout << "trace:" << trace << std::endl;
	//double errorR = acos(trace / 2);
	//errorR = errorR * 180.0 / M_PI;
	//std::cout << errorR << " ";
	//double errorT = std::sqrtf(std::powf(deltaT(0, 3), 2) + std::powf(deltaT(1, 3), 2) + std::powf(deltaT(2, 3), 2));
	//std::cout << errorT << " ";
	//double time = double(std::chrono::duration_cast<std::chrono::microseconds>
	//	(timeE - timeS).count()) / 1000000.0;
	//std::cout << time << std::endl;

	//Decoupling decoupling(sampledCloudS, sampledCloudT, cors, resolution);
	//Eigen::Matrix4d transformation = decoupling.getTransformation();
	//std::cout << transformation << std::endl;
}