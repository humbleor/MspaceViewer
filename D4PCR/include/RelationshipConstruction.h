#ifndef _RELATIONSHIPCONSTRUCTION_H_
#define _RELATIONSHIPCONSTRUCTION_H_

#include "Preparation.h"
#include "dll_export.h"

class DLL_EXPORT RelationshipConstruction
{
public:
	RelationshipConstruction(std::string pathS, std::string pathT, float resolution, bool getCenter);
	~RelationshipConstruction();

	PointCloudPtr getPointCloudS();
	PointCloudPtr getPointCloudT();
	Eigen::Vector3d getCenterS();
	Eigen::Vector3d getCenterT();
	pcl::Correspondences getCorrespondences();

private:
	void loadPCDFileS(std::string pathS);
	void loadPCDFileT(std::string pathT);

	void getCenter(PointCloudPtr input, Eigen::Vector3d& center);
	void downSamping(PointCloudPtr input, PointCloudPtr output, float resolution);
	void keyPointExtration(PointCloudPtr inputCloud, IndicesPtr indices, PointCloudPtr outputCloud, float resolution);
	void descriptorCalculation(PointCloudPtr inputCloud, IndicesPtr indices, FPFHPtr fpfhCloud, float resolution);
	void correspondenceConstruction(FPFHPtr fpfhCloudS, FPFHPtr fpfhCloudT, IndicesPtr indicesS, IndicesPtr indicesT, pcl::Correspondences& correspondences);

private:
	PointCloudPtr _inputCloudS;
	PointCloudPtr _inputCloudT;
	float _resolution;

	Eigen::Vector3d _centerS;
	Eigen::Vector3d _centerT;

	pcl::Correspondences _cors;
};

#endif // !_RELATIONSHIPCONSTRUCTION_H_

