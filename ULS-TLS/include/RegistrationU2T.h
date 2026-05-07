#pragma once
#include "utils.h"
#include <corecrt_math_defines.h>
#include "GridMinimum.h"
#include <omp.h>
#include "LoadLasFile.h"

struct correspondence
{
	size_t index_TLS;
	size_t index_UAV;
};

class DLL_EXPORT RegistrationU2T

{
public:
	RegistrationU2T(PointCloud3fPtr UAV, PointCloud3fPtr TLS);
	~RegistrationU2T();

	void setGridFilterRes(float resolution);
	void gridStep(float step);
	void setSearchRadius(float radius);
	void setRadiusStep(float radiusStep);
	void setNumSectors(size_t numSectors);
	void setTLSCenter(Point3f centerTLS);
	void descriptorsThreshold(float angleThe, float a2DThe, float a3DThe);
	void registration();
	std::array<std::array<float, 4>, 4> getTranslationMatrix();

private:
	void gridminimumFilter();
	void rasterization();
	void TLSfilter();
	void initialCorrespondences(std::vector<std::pair<int, int>>& potentialCor);
	void descriptorsMatchting(std::vector<std::pair<int, int>> potentialCor, size_t& rotationTLS, std::pair<int, int>& cor);
	void coarseRegistration(std::vector<std::pair<int, int>> potentialCor, size_t rotationTLS, std::pair<int, int> cor);

	std::vector<float> PCADescriptors(PointCloud3fPtr _cloud, std::vector<size_t> indices);

	std::vector<std::vector<float>> discretizePolarZAverage(
		const PointCloud3f& cloud,
		const std::vector<size_t>& idx,
		const Point3f& search_pt,
		float R,
		float dR,
		int numSectors);

	float calculateStandardDeviation(const std::vector<float>& vec1, float mean);
private:
	PointCloud3fPtr _UAV;
	PointCloud3fPtr _UAVFilter;
	KDTree2fPtr _treeUAV;
	PointCloud3fPtr _TLS;
	PointCloud3fPtr _TLSFilter;
	Point3f _centerTLS;
	KDTree2fPtr _treeTLS;
	float _resolution;
	float _gridStep;
	float _radius;
	Box3f _box;
	std::vector<size_t> _TLSFilterIndeces;
	std::unordered_map<std::pair<int, int>, std::vector<size_t>, pair_hash> _points_in_grid;
	float _radiusStep;
	size_t _numSectors;
	float _angleThreshold;
	float _a2DThreshold;
	float _a3DThreshold;
	std::array<std::array<float, 4>, 4> _transformationMatrix;
};

