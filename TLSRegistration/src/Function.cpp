#include "Function.h"

void cloud_with_normal(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_normals)
{
	//-----------------拼接点云和法线信息---------------------
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;//OMP加速
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	//建立kdtree来进行近邻点集搜索
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	n.setNumberOfThreads(10);//设置openMP的线程数
	//n.setViewPoint(0,0,0);//设置视点，默认为（0，0，0）
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(10);//点云法线计算时，需要所搜的近邻点大小
	//n.setRadiusSearch(0.03);//半径搜索
	n.compute(*normals);//开始进行法线计算
	//将点云数据与法线信息拼接
	pcl::concatenateFields(*cloud, *normals, *cloud_normals);
}

vector<vector<int>> compute_rhombus_pointclouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float R, int num_sectors)
{
	vector<vector<int>> rhombus_indices(num_sectors);
	size_t npts = cloud->points.size();

	for (size_t i = 0; i < static_cast<size_t>(num_sectors / 3); i++)
	{
		const float end_num1 = 360.0f / num_sectors * i + 120.f;
		const float end_num2 = 360.0f / num_sectors * i + 240.f;

		// Precompute cos/sin for the three rhombus centers (avoid recomputing per point)
		const float cx1 = R * cosf((360.0f / num_sectors * i + 60.f) / 180.0f * M_PI);
		const float sy1 = R * sinf((360.0f / num_sectors * i + 60.f) / 180.0f * M_PI);
		const float cx2 = R * cosf((360.0f / num_sectors * i + 180.f) / 180.0f * M_PI);
		const float sy2 = R * sinf((360.0f / num_sectors * i + 180.f) / 180.0f * M_PI);
		const float cx3 = R * cosf((360.0f / num_sectors * i + 300.f) / 180.0f * M_PI);
		const float sy3 = R * sinf((360.0f / num_sectors * i + 300.f) / 180.0f * M_PI);

		const float dx_o2o1 = -cx1, dy_o2o1 = -sy1;
		const float len_o2o1 = R;
		const float dx_o3o1 = -cx2, dy_o3o1 = -sy2;
		const float len_o3o1 = R;
		const float dx_o4o1 = -cx3, dy_o4o1 = -sy3;
		const float len_o4o1 = R;

		vector<int>& idx1 = rhombus_indices[i];
		vector<int>& idx2 = rhombus_indices[i + num_sectors / 3];
		vector<int>& idx3 = rhombus_indices[i + num_sectors / 3 * 2];

		// Reserve approximate capacity (~1/3 of points each, plus margin)
		idx1.reserve(npts / num_sectors + npts / num_sectors / 2);
		idx2.reserve(npts / num_sectors + npts / num_sectors / 2);
		idx3.reserve(npts / num_sectors + npts / num_sectors / 2);

		for (int pt_idx = 0; pt_idx < static_cast<int>(npts); ++pt_idx)
		{
			const pcl::PointXYZ& point = cloud->points[pt_idx];
			float theta = atan2f(point.y, point.x);
			float r = sqrtf(point.x * point.x + point.y * point.y);
			if (r > R) continue;

			if (theta < 0) theta += 2.f * M_PI;
			theta = theta / M_PI * 180.f;

			if (360.0f / num_sectors * i <= theta && theta <= end_num1)
			{
				const float dx_o2p = point.x - cx1;
				const float dy_o2p = point.y - sy1;
				const float dot_product = dx_o2o1 * dx_o2p + dy_o2o1 * dy_o2p;
				const float angle1 = acosf(dot_product / (len_o2o1 * sqrtf(dx_o2p * dx_o2p + dy_o2p * dy_o2p)));
				if (angle1 <= M_PI / 3.f) idx1.push_back(pt_idx);
			}
			else if ((360.0f / num_sectors * i + 120.f) <= theta && theta <= end_num2)
			{
				const float dx_o3p = point.x - cx2;
				const float dy_o3p = point.y - sy2;
				const float dot_product2 = dx_o3o1 * dx_o3p + dy_o3o1 * dy_o3p;
				const float angle2 = acosf(dot_product2 / (len_o3o1 * sqrtf(dx_o3p * dx_o3p + dy_o3p * dy_o3p)));
				if (angle2 <= M_PI / 3.f) idx2.push_back(pt_idx);
			}
			else
			{
				const float dx_o4p = point.x - cx3;
				const float dy_o4p = point.y - sy3;
				const float dot_product3 = dx_o4o1 * dx_o4p + dy_o4o1 * dy_o4p;
				const float angle3 = acosf(dot_product3 / (len_o4o1 * sqrtf(dx_o4p * dx_o4p + dy_o4p * dy_o4p)));
				if (angle3 <= M_PI / 3.f) idx3.push_back(pt_idx);
			}
		}
	}

	int yushu = num_sectors % 3;
	if (yushu > 0)
	{
		for (size_t i = 0; i < static_cast<size_t>(yushu); i++)
		{
			vector<int>& idx = rhombus_indices[num_sectors - 1 - i];
			idx.reserve(npts / num_sectors + npts / num_sectors / 2);

			const float cxo = R * cosf((360.f - 360.f / num_sectors * (i + 1) + 60.f) / 180.0f * M_PI);
			const float syo = R * sinf((360.f - 360.f / num_sectors * (i + 1) + 60.f) / 180.0f * M_PI);
			const float dx_o4o1 = -cxo, dy_o4o1 = -syo;
			const float len_o4o1 = R;

			const float ang_lo = 360.f - 360.f / num_sectors * (i + 1);
			const float ang_hi = 360.f - 360.f / num_sectors * (i + 1) + 120.f - 360.f;

			for (int pt_idx = 0; pt_idx < static_cast<int>(npts); ++pt_idx)
			{
				const pcl::PointXYZ& point = cloud->points[pt_idx];
				float theta = atan2f(point.y, point.x);
				float r = sqrtf(point.x * point.x + point.y * point.y);
				if (r > R) continue;

				if (theta < 0) theta += 2.f * M_PI;
				theta = theta / M_PI * 180.f;

				if ((ang_lo <= theta && theta < 360.f) || (0.f <= theta && theta <= ang_hi))
				{
					const float dx_o4p = point.x - cxo;
					const float dy_o4p = point.y - syo;
					const float dot_product3 = dx_o4o1 * dx_o4p + dy_o4o1 * dy_o4p;
					const float angle = acosf(dot_product3 / (len_o4o1 * sqrtf(dx_o4p * dx_o4p + dy_o4p * dy_o4p)));
					if (angle <= M_PI / 3.f) idx.push_back(pt_idx);
				}
			}
		}
	}

	return rhombus_indices;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr get_OverlapRhombus_pointclouds(pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud, const vector<int>& indices, float R, float angle_start)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr rhombus1(new pcl::PointCloud<pcl::PointXYZ>);
	float  angle_starting = angle_start;
	float angle_ending = angle_starting + 120.0;

	// Reserve capacity for the output cloud
	rhombus1->points.reserve(indices.size());

	for (int idx : indices)
	{
		const pcl::PointXYZ& point = (*original_cloud)[idx];
		// 计算极坐标角度和距离
		float theta = atan2f(point.y, point.x);
		float r = sqrtf(point.x * point.x + point.y * point.y);
		if (r > R) continue;

		if (theta < 0) theta += 2.f * M_PI;
		theta = theta / M_PI * 180.f;

		const float dx_o2o1 = -R * cosf((angle_starting + 60.f) / 180.0f * M_PI);
		const float dy_o2o1 = -R * sinf((angle_starting + 60.f) / 180.0f * M_PI);
		const float dx_o2p = point.x - R * cosf((angle_starting + 60.f) / 180.0f * M_PI);
		const float dy_o2p = point.y - R * sinf((angle_starting + 60.f) / 180.0f * M_PI);
		const float dot_product = dx_o2o1 * dx_o2p + dy_o2o1 * dy_o2p;
		const float angle1 = acosf(dot_product / (sqrtf(dx_o2o1 * dx_o2o1 + dy_o2o1 * dy_o2o1) * sqrtf(dx_o2p * dx_o2p + dy_o2p * dy_o2p)));

		if (0 <= angle_starting && angle_starting < 240.0)
		{
			if (angle_starting <= theta && theta <= angle_ending)
			{
				if (angle1 <= M_PI / 3)
				{
					rhombus1->points.push_back(point);
				}
			}
		}
		if (240.0 <= angle_starting && angle_starting < 360)
		{
			if ((angle_starting <= theta && theta < 360.0) || (0.0 <= theta && theta <= (angle_ending - 360.0)))
			{
				if (angle1 <= M_PI / 3)
				{
					rhombus1->points.push_back(point);
				}
			}
		}
	}
	rhombus1->width = rhombus1->points.size();
	rhombus1->height = 1;
	return rhombus1;
}

vector<vector<pcl::PointXYZ>> get_rhombus_descriptors(const pcl::PointCloud<pcl::PointXYZ>::Ptr& original_cloud, const vector<int>& indices, float Angle_starting, float r, float step)
{
	int Row = static_cast<int>(r * sinf(60.0f * M_PI / 180.0f) / step) + 1;
	int Col = Row;
	//网格的个数
	vector<vector<pcl::PointCloud<pcl::PointXYZ>>> PointsInGrid(Row, vector<pcl::PointCloud<pcl::PointXYZ>>(Col));
	Eigen::Vector3f v1(cosf(Angle_starting * M_PI / 180.0f), sinf(Angle_starting * M_PI / 180.0f), 0.0f);
	Eigen::Vector3f v2(cosf((Angle_starting + 120.0f) * M_PI / 180.0f), sinf((Angle_starting + 120.0f) * M_PI / 180.0f), 0.0f);
	//创建一个用于保存的行列
	vector<vector<pcl::PointXYZ>> Z_PntInGrid(Row, vector<pcl::PointXYZ>(Col));

	if (indices.empty()) {
		return Z_PntInGrid;
	}

	for (int idx : indices)
	{
		const pcl::PointXYZ& p = (*original_cloud)[idx];
		Eigen::Vector3f pnt(p.x, p.y, 0.0f);
		float dist1 = pnt.cross(v1).norm();
		float dist2 = pnt.cross(v2).norm();
		int rowID = static_cast<int>(dist1 / step);
		int colID = static_cast<int>(dist2 / step);
		if (rowID >= 0 && rowID < Row && colID >= 0 && colID < Col)
		{
			PointsInGrid[rowID][colID].push_back(p);
		}
	}

	for (size_t i = 0; i < static_cast<size_t>(Row); i++) {
		for (size_t j = 0; j < static_cast<size_t>(Col); j++)
		{
			if (PointsInGrid[i][j].size() < 5)
			{
				Z_PntInGrid[i][j].x = std::numeric_limits<float>::quiet_NaN();
				Z_PntInGrid[i][j].y = std::numeric_limits<float>::quiet_NaN();
				Z_PntInGrid[i][j].z = std::numeric_limits<float>::quiet_NaN();
			}
			else
			{
				float Z_value = 0.0f;
				float Y_value = 0.0f;
				float X_value = 0.0f;

				for (size_t k = 0; k < PointsInGrid[i][j].size(); k++)
				{
					Z_value += PointsInGrid[i][j][k].z;
					Eigen::Vector3f pp(PointsInGrid[i][j][k].x, PointsInGrid[i][j][k].y, 0.0f);
					Y_value += pp.cross(v2).norm();
					X_value += pp.cross(v1).norm();
				}
				Z_value /= PointsInGrid[i][j].size();
				Y_value /= PointsInGrid[i][j].size();
				X_value /= PointsInGrid[i][j].size();
				Z_PntInGrid[i][j].x = X_value;
				Z_PntInGrid[i][j].y = Y_value;
				Z_PntInGrid[i][j].z = Z_value;
			}
		}
	}

	return Z_PntInGrid;
}

float calculateStandardDeviation(const std::vector<float>& vec1, float diff) {
	// 计算平均值
	float sum = 0.0;
	for (float num : vec1) {
		sum += num;
	}
	float mean = sum / vec1.size();

	// 计算方差
	float variance = 0.0;

	for (size_t i = 0; i < vec1.size(); i++)
	{
		if (fabs(vec1[i] - mean) <= diff)
		{
			variance += pow(vec1[i] - mean, 2);
		}
		else
		{
			variance += diff * diff;
		}
	}
	// 计算标准差
	float standardDeviation = sqrt(variance / vec1.size());
	return standardDeviation;
}

vector<vector<pcl::PointCloud<pcl::PointXYZ>>> get_PntInrhombus(pcl::PointCloud<pcl::PointXYZ>::Ptr raw_patch, float Angle_starting, float r, float step)
{
	int Row = static_cast<int>(r * sinf(60.0f * M_PI / 180.0f) / step) + 1;
	int Col = Row;
	//网格的个数
	vector<vector<pcl::PointCloud<pcl::PointXYZ>>> PointsInGrid(Row, vector<pcl::PointCloud<pcl::PointXYZ>>(Col));
	Eigen::Vector3f v1(cosf(Angle_starting * M_PI / 180.0f), sinf(Angle_starting * M_PI / 180.0f), 0.0f);
	Eigen::Vector3f v2(cosf((Angle_starting + 120.0f) * M_PI / 180.0f), sinf((Angle_starting + 120.0f) * M_PI / 180.0f), 0.0f);
	//创建一个用于保存的行列
	for (const auto& p : *raw_patch)
	{
		Eigen::Vector3f pnt(p.x, p.y, 0.0f);
		float dist1 = pnt.cross(v1).norm();
		float dist2 = pnt.cross(v2).norm();
		int rowID = static_cast<int>(dist1 / step);
		int colID = static_cast<int>(dist2 / step);
		if (rowID >= 0 && rowID < Row && colID >= 0 && colID < Col)
		{
			PointsInGrid[rowID][colID].push_back(p);
		}
	}

	return PointsInGrid;
}

Eigen::Matrix4f calculateFourParameterTransformation(const pcl::PointXYZ& source_point1, const pcl::PointXYZ& source_point2, const pcl::PointXYZ& target_point1, const pcl::PointXYZ& target_point2) {

	// 计算旋转角度
	Eigen::Vector3f source_vector(source_point2.x - source_point1.x, source_point2.y - source_point1.y, 0.0);
	Eigen::Vector3f target_vector(target_point2.x - target_point1.x, target_point2.y - target_point1.y, 0.0);
	float target_angle = std::atan2(target_vector.y(), target_vector.x());
	float source_angle = std::atan2(source_vector.y(), source_vector.x());
	if (target_angle < 0)
	{
		target_angle += 2 * M_PI;
	}
	if (source_angle < 0)
	{
		source_angle += 2 * M_PI;
	}
	float rotation_angle = target_angle - source_angle;
	if (rotation_angle < 0)
	{
		rotation_angle += 2 * M_PI;
	}

	Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
	transformation(0, 0) = cos(rotation_angle);
	transformation(0, 1) = -sin(rotation_angle);
	transformation(1, 0) = sin(rotation_angle);
	transformation(1, 1) = cos(rotation_angle);
	transformation(2, 3) = (target_point1.z - source_point1.z + target_point2.z - source_point2.z) / 2.0;//Z 平移
	transformation(1, 3) = target_point1.y - (source_point1.x * std::sin(rotation_angle) + source_point1.y * std::cos(rotation_angle));
	transformation(0, 3) = target_point1.x - (source_point1.x * std::cos(rotation_angle) - source_point1.y * std::sin(rotation_angle)); //X 平移
	return transformation;
}

float ComputeRmse_PCR(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudSrc, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudTgt, float dist)
{
	float res = 0.0;
	int n_points = 0;
	float dz = 0.0;
	float rmse;//rmse dz

	std::vector<int> indices(1);
	std::vector<float> sqr_distances(1);
	//step1: 新建kdtree进行搜索
	pcl::search::KdTree<pcl::PointXYZ> tree;
	tree.setInputCloud(cloudSrc);

	//step2: 对于目标中的每个点，找出其最近邻源中的点
	for (size_t i = 0; i < cloudTgt->points.size(); ++i)
	{
		pcl::PointXYZ Pnt_Tgt = cloudTgt->points[i];
		tree.nearestKSearch(Pnt_Tgt, 1, indices, sqr_distances);
		//step3: 统计最小距离和、有效点数
		if (sqr_distances[0] <= dist * dist)
		{
			res += sqr_distances[0];
			n_points += 1;
		}
		else
		{
			res += dist * dist;
		}
	}
	rmse = sqrt(res / cloudTgt->points.size());
	return rmse;
}

pcl::PointCloud<pcl::Normal>::Ptr compute_normal(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::search::KdTree<pcl::PointXYZ>::Ptr tree)
{
	//-------------------------法线计算-----------------------
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
	n.setInputCloud(input_cloud);
	n.setNumberOfThreads(2);//设置openMP的线程数
	n.setSearchMethod(tree);
	n.setKSearch(8);
	n.compute(*normals);

	return normals;
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr compute_fpfh_feature(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::search::KdTree<pcl::PointXYZ>::Ptr tree)
{
	//------------------FPFH描述子计算-------------------------------
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);
	pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> f;
	f.setNumberOfThreads(2); //指定8核计算
	f.setInputCloud(input_cloud);
	f.setInputNormals(normals);
	f.setSearchMethod(tree);
	f.setKSearch(12);
	f.compute(*fpfh);

	return fpfh;
}
