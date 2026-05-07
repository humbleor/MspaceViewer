#include "../include/Function.h"

void cloud_with_normal(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_normals)
{
	//-----------------拼接点云数据与法线信息---------------------
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;//OMP加速
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	//建立kdtree来进行近邻点集搜索
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	n.setNumberOfThreads(10);//设置openMP的线程数
	//n.setViewPoint(0,0,0);//设置视点，默认为（0，0，0）
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(10);//点云法向计算时，需要所搜的近邻点大小
	//n.setRadiusSearch(0.03);//半径搜素
	n.compute(*normals);//开始进行法向计
	//将点云数据与法向信息拼接
	pcl::concatenateFields(*cloud, *normals, *cloud_normals);
}

vector <pcl::PointCloud<pcl::PointXYZ>::Ptr> compute_rhombus_pointclouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float R, int num_sectors)
{

	int Times = 0;
	//std::vector <pcl::PointCloud<pcl::PointXYZ>::Ptr> rhombus_pointclouds(num_sectors);
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> rhombus_pointclouds(num_sectors, nullptr);
	for (size_t i = 0; i < num_sectors / 3; i++)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr rhombus1(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr rhombus2(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr rhombus3(new pcl::PointCloud<pcl::PointXYZ>);
		/*	std::cout << "rhombus1->points.size(): " << rhombus1->points.size() << endl;
			std::cout << "rhombus2->points.size(): " << rhombus2->points.size() << endl;
			std::cout << "rhombus3->points.size(): " << rhombus3->points.size() << endl;*/
			//rhombus1 i 0-119		
		const float end_num1 = 360.0 / num_sectors * i + 120;//120-239
		//rhombus2 i+120		120-239	
		const float end_num2 = 360.0 / num_sectors * i + 240;//240-359
		//rhombus3 i+240	240-359	
		const float end_num3 = 360.0 / num_sectors * i + 360;//360-479
		for (auto it = cloud->begin(); it != cloud->end(); ++it)
		{
			pcl::PointXYZ point = *it;
			// 计算点的极坐标角度和距离
			float theta = std::atan2(point.y, point.x);
			float r = std::sqrt(point.x * point.x + point.y * point.y);
			if (r > R)
			{
				continue;
			}

			if (theta < 0)
			{
				theta += 2 * M_PI;
			}
			theta = theta / M_PI * 180.0;

			if (360.0 / num_sectors * i <= theta && theta <= end_num1)
			{
				const float dx_o2o1 = -R * cos((360.0 / num_sectors * i + 60) / 180.0f * M_PI);
				const float dy_o2o1 = -R * sin((360.0 / num_sectors * i + 60) / 180.0f * M_PI);
				const float dx_o2p = point.x - R * cos((360.0 / num_sectors * i + 60) / 180.0f * M_PI);
				const float dy_o2p = point.y - R * sin((360.0 / num_sectors * i + 60) / 180.0f * M_PI);
				const float dot_product = dx_o2o1 * dx_o2p + dy_o2o1 * dy_o2p;
				const float angle1 = acos(dot_product / (std::sqrt(dx_o2o1 * dx_o2o1 + dy_o2o1 * dy_o2o1) * std::sqrt(dx_o2p * dx_o2p + dy_o2p * dy_o2p)));
				if (angle1 <= M_PI / 3)
				{
					rhombus1->points.push_back(point);
				}

			}
			else if ((360.0 / num_sectors * i + 120) * 1.0 <= theta && theta <= end_num2)
			{
				const float dx_o3o1 = -R * cos((360.0 / num_sectors * i + 180) / 180.0f * M_PI);
				const float dy_o3o1 = -R * sin((360.0 / num_sectors * i + 180) / 180.0f * M_PI);
				const float dx_o3p = point.x - R * cos((360.0 / num_sectors * i + 180) / 180.0f * M_PI);
				const float dy_o3p = point.y - R * sin((360.0 / num_sectors * i + 180) / 180.0f * M_PI);
				const float dot_product2 = dx_o3o1 * dx_o3p + dy_o3o1 * dy_o3p;
				const float angle2 = acos(dot_product2 / (std::sqrt(dx_o3o1 * dx_o3o1 + dy_o3o1 * dy_o3o1) * std::sqrt(dx_o3p * dx_o3p + dy_o3p * dy_o3p)));
				if (angle2 <= M_PI / 3)
				{
					rhombus2->points.push_back(point);
				}
			}
			else /*(((360.0 / num_sectors * i + 240) * 1.0 <= theta && theta < 360.0) || (0.0 <= theta && theta <= (end_num3 - 360.0)))*/
			{
				const float dx_o4o1 = -R * cos((360.0 / num_sectors * i + 300) / 180.0f * M_PI);
				const float dy_o4o1 = -R * sin((360.0 / num_sectors * i + 300) / 180.0f * M_PI);
				const float dx_o4p = point.x - R * cos((360.0 / num_sectors * i + 300) / 180.0f * M_PI);
				const float dy_o4p = point.y - R * sin((360.0 / num_sectors * i + 300) / 180.0f * M_PI);
				const float dot_product3 = dx_o4o1 * dx_o4p + dy_o4o1 * dy_o4p;
				const float angle3 = acos(dot_product3 / (std::sqrt(dx_o4o1 * dx_o4o1 + dy_o4o1 * dy_o4o1) * std::sqrt(dx_o4p * dx_o4p + dy_o4p * dy_o4p)));
				if (angle3 <= M_PI / 3)
				{
					rhombus3->points.push_back(point);
				}
			}
		}

		/*	std::cout << "rhombus1->points.size(): " << rhombus1->points.size() << endl;
			std::cout << "rhombus2->points.size(): " << rhombus2->points.size() << endl;
			std::cout << "rhombus3->points.size(): " << rhombus3->points.size() << endl;*/

		rhombus1->width = rhombus1->points.size();
		rhombus1->height = 1;
		rhombus2->width = rhombus2->points.size();
		rhombus2->height = 1;
		rhombus3->width = rhombus3->points.size();
		rhombus3->height = 1;
		Times += 1;
		rhombus_pointclouds[i] = rhombus1;
		rhombus_pointclouds[i + num_sectors / 3] = rhombus2;
		rhombus_pointclouds[i + num_sectors / 3 * 2] = rhombus3;

	}

	int yushu = num_sectors % 3;
	if (yushu > 0)
	{
		for (size_t i = 0; i < yushu; i++)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr rhombus(new pcl::PointCloud<pcl::PointXYZ>);

			for (auto it = cloud->begin(); it != cloud->end(); ++it)
			{
				pcl::PointXYZ point = *it;
				// 计算点的极坐标角度和距离
				float theta = std::atan2(point.y, point.x);
				float r = std::sqrt(point.x * point.x + point.y * point.y);
				if (r > R)
				{
					continue;
				}

				if (theta < 0)
				{
					theta += 2 * M_PI;
				}
				theta = theta / M_PI * 180.0;

				if ((360 - 360.0 / num_sectors * (i + 1) <= theta && theta < 360.0) || (0.0 <= theta && theta <= (360 - 360.0 / num_sectors * (i + 1) + 120 - 360.0)))
				{
					const float dx_o4o1 = -R * cos((360 - 360.0 / num_sectors * (i + 1) + 60) / 180.0f * M_PI);
					const float dy_o4o1 = -R * sin((360 - 360.0 / num_sectors * (i + 1) + 60) / 180.0f * M_PI);
					const float dx_o4p = point.x - R * cos((360 - 360.0 / num_sectors * (i + 1) + 60) / 180.0f * M_PI);
					const float dy_o4p = point.y - R * sin((360 - 360.0 / num_sectors * (i + 1) + 60) / 180.0f * M_PI);
					const float dot_product3 = dx_o4o1 * dx_o4p + dy_o4o1 * dy_o4p;
					const float angle = acos(dot_product3 / (std::sqrt(dx_o4o1 * dx_o4o1 + dy_o4o1 * dy_o4o1) * std::sqrt(dx_o4p * dx_o4p + dy_o4p * dy_o4p)));
					if (angle <= M_PI / 3)
					{
						rhombus->points.push_back(point);
					}
				}
			}


			rhombus->width = rhombus->points.size();
			rhombus->height = 1;
			rhombus_pointclouds[num_sectors - 1 - i] = rhombus;
			//std::cout << "余数计算了n次: " << i+1 << endl;
		}
	}

	return rhombus_pointclouds;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr get_OverlapRhombus_pointclouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float R, float angle_start)
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr rhombus1(new pcl::PointCloud<pcl::PointXYZ>);
	float  angle_starting = angle_start;
	float angle_ending = angle_starting + 120.0;

	for (auto it = cloud->begin(); it != cloud->end(); ++it)
	{
		pcl::PointXYZ point = *it;
		// 计算点的极坐标角度和距离
		float theta = std::atan2(point.y, point.x);
		float r = std::sqrt(point.x * point.x + point.y * point.y);
		//std::cout << "r:"<< r<<" theta:" << theta <<" angle_starting:"<< angle_starting<<"  " << 111 << endl;
		if (r > R)
		{
			continue;
		}
		if (theta < 0)
		{
			theta += 2 * M_PI;
		}
		theta = theta / M_PI * 180.0;
		//std::cout << "r:" << r << " theta:" << theta << " " << 222 << endl;
		const float dx_o2o1 = -R * cos((angle_starting + 60) / 180.0f * M_PI);
		const float dy_o2o1 = -R * sin((angle_starting + 60) / 180.0f * M_PI);
		const float dx_o2p = point.x - R * cos((angle_starting + 60) / 180.0f * M_PI);
		const float dy_o2p = point.y - R * sin((angle_starting + 60) / 180.0f * M_PI);
		const float dot_product = dx_o2o1 * dx_o2p + dy_o2o1 * dy_o2p;
		const float angle1 = acos(dot_product / (std::sqrt(dx_o2o1 * dx_o2o1 + dy_o2o1 * dy_o2o1) * std::sqrt(dx_o2p * dx_o2p + dy_o2p * dy_o2p)));
		//std::cout << "r:" << r << " theta:" << theta <<" angle1:"<< angle1 << " " << 333 << endl;
		if (0 <= angle_starting < 240.0)
		{

			if (angle_starting <= theta && theta <= angle_ending)
			{
				if (angle1 <= M_PI / 3)
				{
					rhombus1->points.push_back(point);
					//std::cout << "angle_starting:" << angle_starting << "  angle_ending:" << angle_ending <<"push_back(point):"<<111<< endl;

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
					//std::cout << "angle_starting:" << angle_starting << "  angle_ending:" << angle_ending << "push_back(point):" << 111 << endl;

				}
			}
		}
	}
	rhombus1->width = rhombus1->points.size();
	rhombus1->height = 1;
	return rhombus1;
}

vector<vector<pcl::PointXYZ>> get_rhombus_descriptors(pcl::PointCloud<pcl::PointXYZ>::Ptr raw_patch, float Angle_starting, float r, float step)
{
	int Row = static_cast<int>(r * sin(60.0 * M_PI / 180.0) / step) + 1;//格网行数
	int Col = Row;//格网列数
	//包含点的格网
	vector<vector<pcl::PointCloud<pcl::PointXYZ>>> PointsInGrid(Row, vector<pcl::PointCloud<pcl::PointXYZ>>(Col));
	Eigen::Vector3f v1(cos(Angle_starting * M_PI / 180.0), sin(Angle_starting * M_PI / 180.0), 0.0);
	Eigen::Vector3f v2(cos((Angle_starting + 120.0) * M_PI / 180.0), sin((Angle_starting + 120.0) * M_PI / 180.0), 0.0);
	//计算任一点所在的行列号
	vector<vector<pcl::PointXYZ>> Z_PntInGrid(Row, vector<pcl::PointXYZ>(Col));
	if (raw_patch == nullptr) {
		std::cout << "空点云" << endl;
	}
	else
	{
		for (const auto& p : *raw_patch)/////报错。。。。。。。
		{

			Eigen::Vector3f pnt(p.x, p.y, 0.0);
			float dist1 = pnt.cross(v1).norm();
			float dist2 = pnt.cross(v2).norm();
			int rowID = static_cast<int>(dist1 / step);
			int colID = static_cast<int>(dist2 / step);
			//PointsInGrid[rowID][colID].push_back(p);
			if (rowID < Row && colID < Col)
			{
				PointsInGrid[rowID][colID].push_back(p);

			}
		}


		for (size_t i = 0; i < Row; i++) {
			for (size_t j = 0; j < Col; j++)
			{
				if (PointsInGrid[i][j].size() < 5)
				{

					Z_PntInGrid[i][j].x = (std::numeric_limits<float>::quiet_NaN());
					Z_PntInGrid[i][j].y = (std::numeric_limits<float>::quiet_NaN());
					Z_PntInGrid[i][j].z = (std::numeric_limits<float>::quiet_NaN());

					//cout <<"\n\n\n\ Something CRAZY is Happening dude!!! \n\n\n"<< endl;
				}
				else
				{
					float Z_value = 0.0;
					float Y_value = 0.0;
					float X_value = 0.0;
					float std_z = 0.0;

					for (size_t k = 0; k < PointsInGrid[i][j].size(); k++)
					{
						Z_value += PointsInGrid[i][j][k].z;
						Eigen::Vector3f pp(PointsInGrid[i][j][k].x, PointsInGrid[i][j][k].y, 0.0);
						Y_value += pp.cross(v2).norm();
						X_value += pp.cross(v1).norm();

					}
					Z_value /= PointsInGrid[i][j].size();
					Y_value /= PointsInGrid[i][j].size();
					X_value /= PointsInGrid[i][j].size();
					Z_PntInGrid[i][j].x = (X_value);
					Z_PntInGrid[i][j].y = (Y_value);
					Z_PntInGrid[i][j].z = (Z_value);
				}
			}
		}
	}

	return Z_PntInGrid;
	/////////////////////////////////////////////////////////////////////////////////////////
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
	int Row = static_cast<int>(r * sin(60.0 * M_PI / 180.0) / step) + 1;//格网行数
	int Col = Row;//格网列数
	//包含点的格网
	vector<vector<pcl::PointCloud<pcl::PointXYZ>>> PointsInGrid(Row, vector<pcl::PointCloud<pcl::PointXYZ>>(Col));
	Eigen::Vector3f v1(cos(Angle_starting * M_PI / 180.0), sin(Angle_starting * M_PI / 180.0), 0.0);
	Eigen::Vector3f v2(cos((Angle_starting + 120.0) * M_PI / 180.0), sin((Angle_starting + 120.0) * M_PI / 180.0), 0.0);
	//计算任一点所在的行列号
	vector<vector<pcl::PointXYZ>> Z_PntInGrid(Row, vector<pcl::PointXYZ>(Col));
	for (const auto& p : *raw_patch)
	{
		Eigen::Vector3f pnt(p.x, p.y, 0.0);
		float dist1 = pnt.cross(v1).norm();
		float dist2 = pnt.cross(v2).norm();
		int rowID = static_cast<int>(dist1 / step);
		int colID = static_cast<int>(dist2 / step);
		//PointsInGrid[rowID][colID].push_back(p);
		if (rowID < Row && colID < Col)
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
	//step1: 新建kdtree用于搜索
	pcl::search::KdTree<pcl::PointXYZ> tree;
	tree.setInputCloud(cloudSrc);

	//step2: 遍历点云每个点，并找出与它距离最近的点
	for (size_t i = 0; i < cloudTgt->points.size(); ++i)
	{
		pcl::PointXYZ Pnt_Tgt = cloudTgt->points[i];
		tree.nearestKSearch(Pnt_Tgt, 1, indices, sqr_distances);
		//step3: 统计最小距离和、有效点数量
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
	//-------------------------法向量估计-----------------------
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

	//------------------FPFH估计-------------------------------
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