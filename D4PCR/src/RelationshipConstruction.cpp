#include "../include/RelationshipConstruction.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <filesystem>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <liblas/liblas.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

RelationshipConstruction::RelationshipConstruction(std::string pathS, std::string pathT, float resolution, bool getcenter)
{
	loadPointCloudFile(pathS, _inputCloudS);
	loadPointCloudFile(pathT, _inputCloudT);

	if (getcenter)
	{
		std::cout << "===============Center Calculation .....==============" << std::endl;
		auto timeS = std::chrono::system_clock::now();
		getCenter(_inputCloudS, _centerS);
		getCenter(_inputCloudT, _centerT);
		auto timeE = std::chrono::system_clock::now();
		std::cout << "Time consumption of center calculation: " <<
			double(std::chrono::duration_cast<std::chrono::milliseconds>
				(timeE - timeS).count()) / 1000.0 << std::endl;
	}

	std::cout << "Size of PointCloudS = " << std::to_string(_inputCloudS->size() / 1000000.0) <<
		"; Size of PointCloudT = " << std::to_string(_inputCloudT->size() / 1000000.0) << std::endl;
	std::cout << "==================Downsamping .....==================" << std::endl;
	auto timeS = std::chrono::system_clock::now();
	downSamping(_inputCloudS, _inputCloudS, resolution);
	downSamping(_inputCloudT, _inputCloudT, resolution);

	auto timeE = std::chrono::system_clock::now();
	std::cout << "Time consumption of dowmsampling: " <<
		double(std::chrono::duration_cast<std::chrono::milliseconds>
			(timeE - timeS).count()) / 1000.0 << std::endl;

	std::cout << "==============Key Point Extraction .....=============" << std::endl;
	timeS = std::chrono::system_clock::now();
	PointCloudPtr keyPointCloudS(new pcl::PointCloud<pcl::PointXYZ>());
	PointCloudPtr keyPointCloudT(new pcl::PointCloud<pcl::PointXYZ>());
	IndicesPtr indicesS(new pcl::PointIndices);
	IndicesPtr indicesT(new pcl::PointIndices);
	keyPointExtration(_inputCloudS, indicesS, keyPointCloudS, resolution);
	keyPointExtration(_inputCloudT, indicesT, keyPointCloudT, resolution);
	timeE = std::chrono::system_clock::now();
	std::cout << "Time consumption of key point extraction: " <<
		double(std::chrono::duration_cast<std::chrono::milliseconds>
			(timeE - timeS).count()) / 1000.0 << std::endl;
	std::cout << "Size of keyPointCloudS = " << std::to_string(keyPointCloudS->size() / 1000.0) <<
		"; Size of keyPointCloudT = " << std::to_string(keyPointCloudT->size() / 1000.0) << std::endl;

	std::cout << "=============Descriptor Calculation .....============" << std::endl;
	timeS = std::chrono::system_clock::now();
	FPFHPtr fpfhS(new pcl::PointCloud<pcl::FPFHSignature33>());
	FPFHPtr fpfhT(new pcl::PointCloud<pcl::FPFHSignature33>());
	descriptorCalculation(_inputCloudS, indicesS, fpfhS, resolution);
	descriptorCalculation(_inputCloudT, indicesT, fpfhT, resolution);
	timeE = std::chrono::system_clock::now();
	std::cout << "Time consumption of descriptor calculation: " <<
		double(std::chrono::duration_cast<std::chrono::milliseconds>
			(timeE - timeS).count()) / 1000.0 << std::endl;

	std::cout << "==========Correspondence Construction .....==========" << std::endl;
	timeS = std::chrono::system_clock::now();
	correspondenceConstruction(fpfhS, fpfhT, indicesS, indicesT, _cors);
	timeE = std::chrono::system_clock::now();
	std::cout << "Time consumption of correspondence construction: " <<
		double(std::chrono::duration_cast<std::chrono::milliseconds>
			(timeE - timeS).count()) / 1000.0 << std::endl;
	std::cout << "Number of correspondences = " << _cors.size() << std::endl;

	//std::ofstream datafile;
	//datafile.open("input_syn.txt", std::ofstream::out);
	//datafile << _cors.size() << std::endl;
	//datafile << "0,0,-1,0,0,-1" << std::endl;
	//for (size_t i = 0; i < _cors.size(); i++)
	//{
	//	datafile << _inputCloudS->points[_cors[i].index_query].x << " "
	//		<< _inputCloudS->points[_cors[i].index_query].y << " "
	//		<< _inputCloudS->points[_cors[i].index_query].z << " "
	//		<< _inputCloudT->points[_cors[i].index_match].x << " "
	//		<< _inputCloudT->points[_cors[i].index_match].y << " "
	//		<< _inputCloudT->points[_cors[i].index_match].z << std::endl;
	//}
	//datafile.close();
}

RelationshipConstruction::~RelationshipConstruction()
{
}

PointCloudPtr RelationshipConstruction::getPointCloudS()
{
	return _inputCloudS;
}

PointCloudPtr RelationshipConstruction::getPointCloudT()
{
	return _inputCloudT;
}

Eigen::Vector3d RelationshipConstruction::getCenterS()
{
	return _centerS;
}

Eigen::Vector3d RelationshipConstruction::getCenterT()
{
	return _centerT;
}

pcl::Correspondences RelationshipConstruction::getCorrespondences()
{
	return _cors;
}

void RelationshipConstruction::loadPointCloudFile(std::string path, PointCloudPtr& cloud)
{
	cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	std::string ext = std::filesystem::path(path).extension().string();
	std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

	if (ext == ".pcd")
	{
		if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud) == -1)
		{
			PCL_ERROR("Couldn't read file: %s\n", path.c_str());
		}
	}
	else if (ext == ".ply")
	{
		pcl::PointCloud<pcl::PointXYZ> tmp;
		if (pcl::io::loadPLYFile(path, tmp) < 0)
		{
			PCL_ERROR("Couldn't read PLY file: %s\n", path.c_str());
		}
		*cloud = tmp;
	}
	else if (ext == ".las" || ext == ".laz")
	{
		std::ifstream ifs(path.c_str(), std::ios::in | std::ios::binary);
		if (!ifs.is_open())
		{
			PCL_ERROR("Couldn't open LAS file: %s\n", path.c_str());
			return;
		}
		liblas::ReaderFactory f;
		liblas::Reader reader = f.CreateWithStream(ifs);
		unsigned long int nbPoints = reader.GetHeader().GetPointRecordsCount();
		cloud->points.reserve(nbPoints);
		while (reader.ReadNextPoint())
		{
			pcl::PointXYZ p;
			p.x = reader.GetPoint().GetX();
			p.y = reader.GetPoint().GetY();
			p.z = reader.GetPoint().GetZ();
			cloud->points.push_back(p);
		}
		cloud->width = nbPoints;
		cloud->height = 1;
		cloud->is_dense = false;
	}
	else
	{
		PCL_ERROR("Unsupported file format: %s\n", ext.c_str());
	}
}

void RelationshipConstruction::getCenter(PointCloudPtr input, Eigen::Vector3d& center)
{
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*input, centroid);

	for (size_t i = 0; i < input->points.size(); i++)
	{
		input->points[i].x -= centroid[0];
		input->points[i].y -= centroid[1];
		input->points[i].z -= centroid[2];
	}

	center[0] = (double)centroid[0];
	center[1] = (double)centroid[1];
	center[2] = (double)centroid[2];
}

void RelationshipConstruction::downSamping(PointCloudPtr input, PointCloudPtr output, float resolution)
{
	//format for filtering
	pcl::PCLPointCloud2::Ptr cloud2(new pcl::PCLPointCloud2());
	pcl::PCLPointCloud2::Ptr cloudVG2(new pcl::PCLPointCloud2());
	pcl::toPCLPointCloud2(*input, *cloud2);
	//set up filtering parameters
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(cloud2);
	sor.setLeafSize(resolution, resolution, resolution);
	//filtering process
	sor.filter(*cloudVG2);
	pcl::fromPCLPointCloud2(*cloudVG2, *output);
}

void RelationshipConstruction::keyPointExtration(PointCloudPtr inputCloud, IndicesPtr indices, PointCloudPtr outputCloud, float resolution)
{
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>());
	pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss;
	iss.setSearchMethod(kdtree);
	iss.setSalientRadius(4 * resolution);
	iss.setNonMaxRadius(2 * resolution);
	iss.setThreshold21(0.975);
	iss.setThreshold32(0.975);
	iss.setMinNeighbors(4);
	iss.setNumberOfThreads(1);
	iss.setInputCloud(inputCloud);
	iss.compute(*outputCloud);
	indices->indices = iss.getKeypointsIndices()->indices;
	indices->header = iss.getKeypointsIndices()->header;
}

void RelationshipConstruction::descriptorCalculation(PointCloudPtr inputCloud, IndicesPtr indices, FPFHPtr fpfhCloud, float resolution)
{
	//compute normal
	pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>());
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(inputCloud);
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(3 * resolution);
	ne.compute(*normal);

	//compute fpfh using normals
	pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
	fpfh_est.setInputCloud(inputCloud);
	fpfh_est.setInputNormals(normal);
	fpfh_est.setSearchMethod(tree);
	fpfh_est.setRadiusSearch(8 * resolution);
	fpfh_est.setNumberOfThreads(16);
	fpfh_est.setIndices(indices);
	fpfh_est.compute(*fpfhCloud);
}

void RelationshipConstruction::correspondenceConstruction(FPFHPtr fpfhCloudS, FPFHPtr fpfhCloudT, IndicesPtr indicesS, IndicesPtr indicesT, pcl::Correspondences& correspondences)
{
	pcl::Correspondences fpfhCor;
	pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> corr_estimation;
	corr_estimation.setInputSource(fpfhCloudS);
	corr_estimation.setInputTarget(fpfhCloudT);
	corr_estimation.determineReciprocalCorrespondences(fpfhCor);
	for (size_t i = 0; i < fpfhCor.size(); i++)
	{
		pcl::Correspondence cor;
		cor.index_query = indicesS->indices[fpfhCor[i].index_query];
		cor.index_match = indicesT->indices[fpfhCor[i].index_match];
		correspondences.emplace_back(cor);
	}

	// Use a KdTree to search for the nearest matches in feature space
	pcl::search::KdTree<pcl::FPFHSignature33> treeS;
	treeS.setInputCloud(fpfhCloudS);
	pcl::search::KdTree<pcl::FPFHSignature33> treeT;
	treeT.setInputCloud(fpfhCloudT);
	for (size_t i = 0; i < fpfhCloudS->size(); i++) {
		std::vector<int> corrIdxTmp(5);
		std::vector<float> corrDisTmp(5);
		//find the best n matches in target fpfh
		treeT.nearestKSearch(*fpfhCloudS, i, 5, corrIdxTmp, corrDisTmp);
		for (size_t j = 0; j < corrIdxTmp.size(); j++) {
			bool removeFlag = true;
			int searchIdx = corrIdxTmp[j];
			std::vector<int> corrIdxTmpT(5);
			std::vector<float> corrDisTmpT(5);
			treeS.nearestKSearch(*fpfhCloudT, searchIdx, 5, corrIdxTmpT, corrDisTmpT);
			for (size_t k = 0; k < 5; k++) {
				if (corrIdxTmpT.data()[k] == i) {
					removeFlag = false;
					break;
				}
			}
			if (removeFlag == false) {
				pcl::Correspondence corrTabTmp;
				corrTabTmp.index_query = indicesS->indices[i];
				corrTabTmp.index_match = indicesT->indices[corrIdxTmp[j]];
				corrTabTmp.distance = corrDisTmp[j];
				correspondences.push_back(corrTabTmp);
			}
		}
	}
}

// Explicit template instantiations for KdTreeFLANN<FPFHSignature33>
// Required because vcpkg PCL search library doesn't include this specialization


