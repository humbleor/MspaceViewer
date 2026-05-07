#pragma once
#include "../include/RegistrationU2T.h"

RegistrationU2T::RegistrationU2T(PointCloud3fPtr UAV, PointCloud3fPtr TLS)
{
	_UAV = std::make_shared<PointCloud3f>();
	_TLS = std::make_shared<PointCloud3f>();
	_UAVFilter = std::make_shared<PointCloud3f>();
	_TLSFilter = std::make_shared<PointCloud3f>();
    _UAV = UAV;
	_TLS = TLS;
}

RegistrationU2T::~RegistrationU2T()
{
}

void RegistrationU2T::setGridFilterRes(float resolution)
{
    _resolution = resolution;
}

void RegistrationU2T::gridStep(float step)
{
    _gridStep = step;
}

void RegistrationU2T::setSearchRadius(float radius)
{
	_radius = radius;
}

void RegistrationU2T::setRadiusStep(float radiusStep)
{
    _radiusStep = radiusStep;
}

void RegistrationU2T::setNumSectors(size_t numSectors)
{
    _numSectors = numSectors;
}

void RegistrationU2T::setTLSCenter(Point3f centerTLS)
{
    _centerTLS = centerTLS;
}

void RegistrationU2T::descriptorsThreshold(float angleThe, float a2DThe, float a3DThe)
{
	_angleThreshold = angleThe;
	_a2DThreshold = a2DThe;
	_a3DThreshold = a3DThe;
}

void RegistrationU2T::registration()
{
    gridminimumFilter();
    rasterization();
    TLSfilter();
    std::vector<std::pair<int, int>> potentialCor;
	initialCorrespondences(potentialCor);
	if (potentialCor.empty())
	{
		std::cout << "No potential correspondences found." << std::endl;
		return;
	}
	size_t rotationTLS = 0;
    std::pair<int, int> optimalCor;
	descriptorsMatchting(potentialCor, rotationTLS, optimalCor);
	_UAVFilter->clear();
    _TLSFilter->clear();
    _points_in_grid.clear();
	coarseRegistration(potentialCor, rotationTLS, optimalCor);
}

std::array<std::array<float, 4>, 4> RegistrationU2T::getTranslationMatrix()
{
    return _transformationMatrix;
}

void RegistrationU2T::gridminimumFilter()
{
    std::shared_ptr<GridMinimumFilter> filterUAV = std::make_shared<GridMinimumFilter>(_resolution);
    filterUAV->setInputCloud(_UAV);
    filterUAV->filter(_UAVFilter);
	std::shared_ptr<GridMinimumFilter> filterTLS = std::make_shared<GridMinimumFilter>(_resolution);
	filterTLS->setInputCloud(_TLS);
	filterTLS->filter(_TLSFilter);
}

void RegistrationU2T::rasterization()
{
    PointCloud2fPtr UAV2D = std::make_shared<PointCloud2f>();
    for (size_t i = 0; i < _UAVFilter->points().size(); i++)
    {
        Point2f pt;
        pt.coords()[0] = _UAVFilter->points()[i].coords()[0];
        pt.coords()[1] = _UAVFilter->points()[i].coords()[1];
        UAV2D->addPoint(pt);
    }

    _treeUAV = std::make_shared<KDTree2f>(*UAV2D, 10, 1);

    _box = _UAVFilter->boundingBox();
	int rows = static_cast<int>((_box.max().coords()[0] - _box.min().coords()[0]) / _gridStep) + 1;
	int cols = static_cast<int>((_box.max().coords()[1] - _box.min().coords()[1]) / _gridStep) + 1;

    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {

            Point2f center;
            center.coords()[0] = _box.min().coords()[0] + row * _gridStep + _gridStep / 2;
            center.coords()[1] = _box.min().coords()[1] + col * _gridStep + _gridStep / 2;


            std::vector<nanoflann::ResultItem<size_t, float>> neighbors;
            _treeUAV->index->radiusSearch(&(center.coords()[0]), _radius * _radius, neighbors);

            if (!neighbors.empty()) {
                std::pair<int, int> grid_id = { row, col };
                for (const auto& item : neighbors) {
                    _points_in_grid[grid_id].push_back(item.first);
                }

            }
        }
    }

}

void RegistrationU2T::TLSfilter()
{
	PointCloud2fPtr TLS2D = std::make_shared<PointCloud2f>();
    Point2f point2d;
    for (size_t i = 0; i < _TLSFilter->points().size(); i++)
    {
		point2d.coords()[0] = _TLSFilter->points()[i].coords()[0];
		point2d.coords()[1] = _TLSFilter->points()[i].coords()[1];
		TLS2D->addPoint(point2d);
    }
    _treeTLS = std::make_shared<KDTree2f>(*TLS2D, 10, 1);
    Point2f center;
	center.coords()[0] = _centerTLS.coords()[0];
	center.coords()[1] = _centerTLS.coords()[1];
    std::vector<nanoflann::ResultItem<size_t, float>> neighbors;
    _treeTLS->index->radiusSearch(&(center.coords()[0]), _radius * _radius, neighbors);

    for (size_t i = 0; i < neighbors.size(); i++)
        _TLSFilterIndeces.emplace_back(neighbors[i].first);

	PointCloud3fPtr pointSet = std::make_shared<PointCloud3f>();
    for (size_t i = 0; i < _TLSFilterIndeces.size(); i++)
    {
		pointSet->addPoint(_TLSFilter->points()[_TLSFilterIndeces[i]]);
    }
}

void RegistrationU2T::initialCorrespondences(std::vector<std::pair<int, int>>& potentialCor)
{
    potentialCor.clear();

    std::vector<float> descriptorsTLS = PCADescriptors(_TLSFilter, _TLSFilterIndeces);
    size_t R_num_min = size_t(_numSectors * _radius / _radiusStep / 4);
    for (const auto& [grid_id, indices] : _points_in_grid) {

        if (indices.size() > R_num_min) // Skip grids with insufficient points
        {
            std::vector<float> descriptorsUAV = PCADescriptors(_UAVFilter, indices);
            float angleDiff = fabs(descriptorsTLS[0] - descriptorsUAV[0]);
            float a2DDiff = fabs(descriptorsTLS[1] - descriptorsUAV[1]);
            float a3DDiff = fabs(descriptorsTLS[2] - descriptorsUAV[2]);
            if (angleDiff <= _angleThreshold && a2DDiff <= _a2DThreshold && a3DDiff <= _a3DThreshold)
                potentialCor.emplace_back(grid_id);
            else
                _points_in_grid[grid_id].clear(); // Clear grids with insufficient points
        }
        else
            _points_in_grid[grid_id].clear(); // Clear grids with insufficient points
    }
}

void RegistrationU2T::descriptorsMatchting(std::vector<std::pair<int, int>> potentialCor, size_t& rotationTLS, std::pair<int, int>& cor)
{
	std::vector<std::vector<float>> descriptorsTLS;
    descriptorsTLS = discretizePolarZAverage(*_TLSFilter, _TLSFilterIndeces, _centerTLS, _radius, _radiusStep, _numSectors);

	std::vector<std::vector<float>> descriptorsUAV;
    std::vector<size_t> indices;

    std::vector<float> std_match(potentialCor.size(), 100.0);
    std::vector<size_t> colR_match(potentialCor.size());

    for (size_t i = 0; i < potentialCor.size(); i++)
    {
        Point3f centerUAV;
        centerUAV.coords()[0] = _box.min().coords()[0] + potentialCor[i].first * _gridStep + _gridStep / 2;
        centerUAV.coords()[1] = _box.min().coords()[1] + potentialCor[i].second * _gridStep + _gridStep / 2;
		centerUAV.coords()[2] = 0;
        indices = _points_in_grid[{potentialCor[i].first, potentialCor[i].second}];
		descriptorsUAV = discretizePolarZAverage(*_UAVFilter, indices, centerUAV, _radius, _radiusStep, _numSectors);

        std::vector<float> stdR;
        std::vector<int> colR;

        for (size_t j = 0; j < _numSectors; j++)
        {
			size_t num_col = j + 1;
			if (num_col >= _numSectors)
                num_col = 0;
            for (auto& row : descriptorsTLS) 
                std::rotate(row.begin(), row.begin() + 1, row.end());
			float sum = 0.0;
			float std = 100.0;
			std::vector<float> diff;
            for (size_t k = 0; k < descriptorsTLS.size(); ++k) {
				for (size_t l = 0; l < descriptorsTLS[k].size(); ++l) {
					float dz_diff = descriptorsUAV[k][l] - descriptorsTLS[k][l];
					if (isnormal(dz_diff)) {
						diff.emplace_back(dz_diff);
						sum += dz_diff;
					}
				}
            }
            if (diff.size() > 100) // Ensure enough points for std deviation calculation
            {
				std = calculateStandardDeviation(diff, sum / diff.size());
            }

            stdR.emplace_back(std);
            colR.emplace_back(num_col);
        }
        size_t min_index0 = std::min_element(stdR.begin(), stdR.end()) - stdR.begin();
        std_match[i] = stdR[min_index0];
        colR_match[i] = colR[min_index0];
    }
    size_t min_index = std::min_element(std_match.begin(), std_match.end()) - std_match.begin();
    rotationTLS = colR_match[min_index];
	cor = potentialCor[min_index];
}

void RegistrationU2T::coarseRegistration(std::vector<std::pair<int, int>> potentialCor, size_t rotationTLS, std::pair<int, int> cor)
{
	float xy_stp = 0.05;
    Point2f centerULS;
	centerULS.coords()[0] = _box.min().coords()[0] + cor.first * _gridStep + _gridStep / 2;
	centerULS.coords()[1] = _box.min().coords()[1] + cor.second * _gridStep + _gridStep / 2;
	PointCloud2fPtr UAV2D = std::make_shared<PointCloud2f>();
	Point2f point2d;
    for (size_t i = 0; i < _UAV->points().size(); i++)
    {
		point2d.coords()[0] = _UAV->points()[i].coords()[0];
		point2d.coords()[1] = _UAV->points()[i].coords()[1];
		UAV2D->addPoint(point2d);
    }
	_treeUAV = std::make_shared<KDTree2f>(*UAV2D, 10, 1);
	std::vector<nanoflann::ResultItem<size_t, float>> neighborsUAV;
	_treeUAV->index->radiusSearch(&(centerULS.coords()[0]), 10.0 * 10.0, neighborsUAV);
    std::vector<size_t> indices;
    for (const auto& item : neighborsUAV)
        indices.emplace_back(item.first);


    PointCloud2fPtr TLS2D = std::make_shared<PointCloud2f>();
    for (size_t i = 0; i < _TLS->points().size(); i++)
    {
        point2d.coords()[0] = _TLS->points()[i].coords()[0];
        point2d.coords()[1] = _TLS->points()[i].coords()[1];
        TLS2D->addPoint(point2d);
    }
    _treeTLS = std::make_shared<KDTree2f>(*TLS2D, 10, 1);
    Point2f center;
    center.coords()[0] = _centerTLS.coords()[0];
    center.coords()[1] = _centerTLS.coords()[1];
    std::vector<nanoflann::ResultItem<size_t, float>> neighborsTLS;
    _treeTLS->index->radiusSearch(&(center.coords()[0]), _radius * _radius, neighborsTLS);

    _TLSFilterIndeces.clear();
    for (const auto& item : neighborsTLS)
        _TLSFilterIndeces.emplace_back(item.first);

	std::vector<std::vector<float>> descriptorsTLS;
	descriptorsTLS = discretizePolarZAverage(*_TLS, _TLSFilterIndeces, _centerTLS, _radius, 0.05, 720);

	std::vector<std::vector<float>> descriptorsUAV;
	Point3f centerULS1;
    centerULS1.coords()[3] = 0;

    std::vector<float> stdSet;
    std::vector<float> dzRSet;
    std::vector<int> colSet;
	PointCloud3fPtr centerULSSet = std::make_shared<PointCloud3f>();
    for (int i = -_gridStep / xy_stp; i <= _gridStep / xy_stp; i++)
    {
        centerULS1.coords()[0] = _box.min().coords()[0] + cor.first * _gridStep + _gridStep / 2 + i * xy_stp;
        for (int j = -_gridStep / xy_stp; j <= _gridStep / xy_stp; j++)
        {
            centerULS1.coords()[1] = _box.min().coords()[1] + cor.second * _gridStep + _gridStep / 2 + j * xy_stp;

			descriptorsUAV = discretizePolarZAverage(*_UAV, indices, centerULS1, _radius, 0.05, 720);
			std::vector<float> stdR;
			std::vector<int> colR;
            for (int m = -10; m <= 10; m++)
            {
                float sum = 0.0;
                float std = 100.0;
                int idx_col = 720 * 1.0 / _numSectors * rotationTLS + m;
                std::vector<float> diff;

                for (size_t k = 0; k < descriptorsTLS.size(); ++k) {
                    for (size_t l = 0; l < descriptorsTLS[k].size(); ++l) {

                        int rotation_colIdx = l + idx_col;
                        if (rotation_colIdx >= descriptorsTLS[k].size())
                        {
                            rotation_colIdx -= descriptorsTLS[k].size();
                        }
                        if (rotation_colIdx < 0)
                        {
                            rotation_colIdx += descriptorsTLS[k].size();
                        }
                        float dz_diff = descriptorsUAV[k][l] - descriptorsTLS[k][rotation_colIdx];
                        if (isnormal(dz_diff))
                        {
                            diff.push_back(dz_diff);
                            sum += dz_diff;
                        }
                    }
                }
                if (diff.size() > 100/* num_sectors*(R_neighbors/ R_stp *0.1)*/)
                {

                    std = calculateStandardDeviation(diff, sum / diff.size());

                }
                if (idx_col < 0)
                {
                    idx_col += 720;
                }
                if (idx_col >= 720)
                {
                    idx_col -= 720;
                }
                centerULSSet->addPoint(centerULS1);
                stdSet.push_back(std);
                dzRSet.push_back(sum / diff.size());
                colSet.push_back(idx_col);
            }
        }
    }
    size_t minIndex = std::min_element(stdSet.begin(), stdSet.end()) - stdSet.begin();
    float rotationAngle = 2.0 * M_PI - colSet[minIndex] * 2.0 * M_PI / 720;

    _transformationMatrix = { {
    {{1.0f, 0.0f, 0.0f, 0.0f}},
    {{0.0f, 1.0f, 0.0f, 0.0f}},
    {{0.0f, 0.0f, 1.0f, 0.0f}},
    {{0.0f, 0.0f, 0.0f, 1.0f}}} };

    _transformationMatrix[0][0] = cos(rotationAngle);
    _transformationMatrix[0][1] = -sin(rotationAngle);
    _transformationMatrix[1][0] = sin(rotationAngle);
    _transformationMatrix[1][1] = cos(rotationAngle);
    _transformationMatrix[2][3] = dzRSet[minIndex];
    _transformationMatrix[1][3] = centerULSSet->points()[minIndex].coords()[1] - (_centerTLS.coords()[0] * std::sin(rotationAngle) + _centerTLS.coords()[1] * std::cos(rotationAngle));
    _transformationMatrix[0][3] = centerULSSet->points()[minIndex].coords()[0] - (_centerTLS.coords()[0] * std::cos(rotationAngle) - _centerTLS.coords()[1] * std::sin(rotationAngle));
}

std::vector<float> RegistrationU2T::PCADescriptors(PointCloud3fPtr _cloud, std::vector<size_t> indices)
{
    std::vector<float> descriptors(3, std::numeric_limits<float>::quiet_NaN());
    if (indices.size() < 3) return descriptors;

	PointCloud3f subCloud;
    for (size_t idx : indices) {
        subCloud.addPoint(_cloud->points()[idx]);
    }

	KDTree3f tree(subCloud, 10, 1);

	PCAEstimator3f pca(subCloud);
	if (!pca.compute()) {
		return std::vector<float>(3, std::numeric_limits<float>::quiet_NaN());
	}
	std::array<float, 3> eigenvalues = pca.eigenvalues();
	std::array<std::array<float, 3>, 3> eigenvectors = pca.eigenvectors();

    float angle = acos(fabs(eigenvectors[2][2])) * 180.0 / M_PI;
	float a2D = (sqrt(eigenvalues[1]) - sqrt(eigenvalues[2])) / sqrt(eigenvalues[0]);
	float a3D = sqrt(eigenvalues[2]) / sqrt(eigenvalues[0]);

	descriptors[0] = angle;
	descriptors[1] = a2D; 
	descriptors[2] = a3D;

    return descriptors;
}

std::vector<std::vector<float>> RegistrationU2T::discretizePolarZAverage(const PointCloud3f& cloud, const std::vector<size_t>& idx, const Point3f& search_pt, float R, float dR, int numSectors)
{

    int nP = static_cast<int>(std::ceil(R / dR));
    float dtheta = 2 * M_PI / numSectors;
    std::vector<std::vector<int>> count(nP, std::vector<int>(numSectors, 0));
    std::vector<std::vector<float>> zsum(nP, std::vector<float>(numSectors, 0));

    for (int i : idx) {
        const auto& pt = cloud[i];
        float dx = pt.coords()[0] - search_pt.coords()[0];
        float dy = pt.coords()[1] - search_pt.coords()[1];
        float dz = pt.coords()[2];

        float distance2 = dx * dx + dy * dy;
        if (distance2 > R * R) continue;

        float distance = std::sqrt(distance2);
        float angle = std::atan2(dy, dx);
        if (angle < 0) angle += 2 * M_PI;

        int r_idx = static_cast<int>(distance / dR);
        int a_idx = static_cast<int>(angle / dtheta);

        if (r_idx < nP && a_idx < numSectors) {
            count[r_idx][a_idx]++;
            zsum[r_idx][a_idx] += dz;
        }
    }

    std::vector<std::vector<float>> zavg(nP, std::vector<float>(numSectors, std::numeric_limits<float>::quiet_NaN()));

    for (int i = 0; i < nP; ++i) 
        for (int j = 0; j < numSectors; ++j) 
            if (count[i][j] > 0)
                zavg[i][j] = zsum[i][j] / count[i][j];
    return zavg;
}

float RegistrationU2T::calculateStandardDeviation(const std::vector<float>& vec1, float mean)
{
    float variance = 0.0;

    for (size_t i = 0; i < vec1.size(); i++)
    {
        if (fabs(vec1[i] - mean) <= 0.5)
            variance += pow(vec1[i] - mean, 2);
        else
            variance += 0.25;
    }
    float standardDeviation = sqrt(variance / vec1.size());
    return standardDeviation;
}
