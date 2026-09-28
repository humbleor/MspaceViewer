#include "../include/RegistrationForm_TLS.h"
#include "../include/PointCloudLoader.h"
#include <QCoreApplication>
#include <QFutureWatcher>

static void logToLogger(QPointer<QTextEdit> logger, const QString& text)
{
	QMetaObject::invokeMethod(QCoreApplication::instance(), [logger, text]() {
		if (logger)
			logger->insertPlainText(text);
	}, Qt::QueuedConnection);
}

RegistrationForm_TLS::RegistrationForm_TLS(QWidget* parent)
	:QDialog(parent),
	Ui::TLSRegistration()
{
	this->setupUi(this);
	connect(_selectInputFileOfSource, &QPushButton::clicked, this, &RegistrationForm_TLS::selectInputFileOfSource);
	connect(_selectInputFileOfTarget, &QPushButton::clicked, this, &RegistrationForm_TLS::selectInputFileOfTarget);
	connect(_selectOutputDir, &QPushButton::clicked, this, &RegistrationForm_TLS::selectOutputDir);
	connect(_action_OK, &QPushButton::clicked, this, &RegistrationForm_TLS::apply);
	connect(_action_cancel, &QPushButton::clicked, this, &RegistrationForm_TLS::reject);
	initParam();
}

RegistrationForm_TLS::~RegistrationForm_TLS()
{
}


void RegistrationForm_TLS::selectInputFileOfSource()
{
	QStringList fileTypes;
	fileTypes << "All Point Cloud Files (*.las *.laz *.pcd *.ply)"
			  << "LAS Files (*.las)"
			  << "LAZ Files (*.laz)"
			  << "PCD Files (*.pcd)"
			  << "PLY Files (*.ply)";
	QString file = QFileDialog::getOpenFileName(this, tr("Select Source File"), "", fileTypes.join(";;"));
	if (file.isEmpty())
		return;
	_inputFileOfSource->setText(file);
}

void RegistrationForm_TLS::selectInputFileOfTarget()
{
	QStringList fileTypes;
	fileTypes << "All Point Cloud Files (*.las *.laz *.pcd *.ply)"
			  << "LAS Files (*.las)"
			  << "LAZ Files (*.laz)"
			  << "PCD Files (*.pcd)"
			  << "PLY Files (*.ply)";
	QString file = QFileDialog::getOpenFileName(this, tr("Select Target File"), "", fileTypes.join(";;"));
	if (file.isEmpty())
		return;
	_inputFileOfTarget->setText(file);
}

void RegistrationForm_TLS::selectOutputDir()
{
	QString dir = QFileDialog::getExistingDirectory(this, tr("Select Output Directory"), "");
	if (dir.isEmpty())
		return;
	_outputFileOfDir->setText(dir);
}

bool RegistrationForm_TLS::executeRegistration(QProgressDialog* progress, QTextEdit* logger,
	std::function<void()> onFinished)
{
	if (_sector_num->text().isEmpty() ||
		_resolution_Radius->text().isEmpty() ||
		_maxRadius->text().isEmpty() ||
		_minRadius->text().isEmpty() ||
		_error_dis->text().isEmpty() ||
		_error_ang->text().isEmpty() ||
		_error_z->text().isEmpty() ||
		_pointsConstrain->text().isEmpty() ||
		_zConstrain->text().isEmpty())
		return false;

	TLSRegParams params;
	params.sourceFile = _inputFileOfSource->text();
	params.targetFile = _inputFileOfTarget->text();
	params.outputDir = _outputFileOfDir->text();
	params.sector_num = _sector_num->text().toInt();
	params.resolution_Radius = _resolution_Radius->text().toFloat();
	params.maxRadius = _maxRadius->text().toFloat();
	params.minRadius = _minRadius->text().toFloat();
	params.error_dis = _error_dis->text().toFloat();
	params.error_ang = _error_ang->text().toFloat();
	params.error_z = _error_z->text().toFloat();
	params.pointsConstrain = _pointsConstrain->text().toUInt();
	params.zConstrain = _zConstrain->text().toFloat();

	if (!progress)
		return false;
	progress->setRange(0, 100);
	progress->setValue(0);
	progress->setLabelText(tr("Loading point clouds ..."));
	progress->setCancelButton(nullptr);
	progress->setWindowFlags(progress->windowFlags() & ~Qt::WindowCloseButtonHint);
	progress->setAutoClose(false);
	progress->setWindowModality(Qt::NonModal);
	progress->show();

	QPointer<QProgressDialog> progressGuard(progress);
	QPointer<QTextEdit> loggerGuard(logger);
	auto reportProgress = [progressGuard](int value, const QString& stage) {
		QMetaObject::invokeMethod(QCoreApplication::instance(), [progressGuard, value, stage]() {
			if (progressGuard) {
				progressGuard->setLabelText(stage);
				progressGuard->setValue(value);
			}
		}, Qt::QueuedConnection);
	};
	auto* watcher = new QFutureWatcher<void>(progress);
	connect(watcher, &QFutureWatcher<void>::finished, progress,
		[progressGuard, watcher, onFinished = std::move(onFinished)]() {
			if (progressGuard) {
				progressGuard->setValue(100);
				progressGuard->close();
				progressGuard->deleteLater();
			}
			onFinished();
			watcher->deleteLater();
		});
	watcher->setFuture(QtConcurrent::run([params, loggerGuard, reportProgress]() {
		registration(params, loggerGuard, reportProgress);
	}));
	return true;
}

void RegistrationForm_TLS::apply()
{
	this->done(QDialog::Accepted); // 等同于 this->accept();
}

void RegistrationForm_TLS::reject()
{
	this->done(QDialog::Rejected); // 等同于 this->reject();
}

void RegistrationForm_TLS::initParam()
{
	_sector_num->setText("360");
	_resolution_Radius->setText("0.2");
	_maxRadius->setText("10.0");
	_minRadius->setText("5.0");

	_error_dis->setText("0.2");
	_error_z->setText("0.1");
	_error_ang->setText("30.0");

	_pointsConstrain->setText("20");
	_zConstrain->setText("0.2");
}

void RegistrationForm_TLS::registration(TLSRegParams params, QPointer<QTextEdit> logger,
	std::function<void(int, const QString&)> reportProgress)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (!pcutil::loadPointCloud(params.sourceFile.toStdString(), source_cloud))
	{
		logToLogger(logger, tr("Failed to load source point cloud file!") + "\n");
		return;
	}
	if (!pcutil::loadPointCloud(params.targetFile.toStdString(), target_cloud))
	{
		logToLogger(logger, tr("Failed to load target point cloud file!") + "\n");
		return;
	}
	reportProgress(5, tr("Building rhombus descriptors ..."));

	int num_sectors = params.sector_num;
	float R_neighbor = params.maxRadius;
	float r_neighbor = params.minRadius;
	float dR = params.resolution_Radius;
	float Step = dR;

	float corr_canshu1 = params.error_dis;
	float corr_canshu2 = params.error_z;
	float corr_canshu3 = params.error_ang;

	unsigned int icp_canshu1 = params.pointsConstrain;
	float icp_canshu2 = params.zConstrain;

	//------------------------------------------粗配准------------------------------------------
	int num_combinations = num_sectors * num_sectors;
	vector<vector<int>> target_rhombus_indices;
	target_rhombus_indices = compute_rhombus_pointclouds(target_cloud, R_neighbor, num_sectors);
	vector<vector<int>> source_rhombus_indices;
	source_rhombus_indices = compute_rhombus_pointclouds(source_cloud, R_neighbor, num_sectors);
	int fault_tolerant_iterations = 2;// 描述子匹配时允许相邻网格偏移
	std::vector <vector<vector<pcl::PointXYZ>>> target_rhombus_descriptors(num_sectors);
	std::vector <vector<vector<pcl::PointXYZ>>> source_rhombus_descriptors(num_sectors);
	float dTheta = 360.0 / num_sectors;
	Step *= sqrt(3) / 2;

	for (int i = 0; i < num_sectors; i++)
	{
		target_rhombus_descriptors[i] = get_rhombus_descriptors(target_cloud, target_rhombus_indices[i], i * dTheta, R_neighbor, Step);
		source_rhombus_descriptors[i] = get_rhombus_descriptors(source_cloud, source_rhombus_indices[i], i * dTheta, R_neighbor, Step);
		if ((i + 1) % std::max(1, num_sectors / 15) == 0)
			reportProgress(5 + 15 * (i + 1) / num_sectors,
				tr("Building rhombus descriptors: sector %1 of %2 ...").arg(i + 1).arg(num_sectors));

	}
	reportProgress(20, tr("Matching descriptors ..."));

	int num_dR = static_cast<int>((R_neighbor - r_neighbor) / dR) + 1;
	std::vector<std::vector<float>> corr_dR(num_dR, std::vector<float>(4));//i j l dz
	vector<float> std_dR(num_dR, 10.0);
	std::vector<std::vector<int>> corr(num_combinations, std::vector<int>(2));
	vector<float> score(num_combinations, 10.0);
	vector<float> Diff_Zt_Zs(num_combinations, 0.0);//Zt_Zs
	vector<float> Diff_Lts(num_combinations, 0.0);// XY 平面上的平移长度

	// Pre-allocate neighbor buffers once to avoid repeated allocations in inner loops
	int max_grid_line_num = static_cast<int>((R_neighbor) * sin(60.0 * M_PI / 180.0) / Step) + 1;
	int max_neighbor_size = max_grid_line_num * max_grid_line_num * fault_tolerant_iterations;
	vector<float> neighbor_DZ(max_neighbor_size, 100.0f);
	vector<float> neighbor_Dist(max_neighbor_size, 100.0f);
	vector<float> neighbor_DiffD(max_neighbor_size, 100.0f);
	for (size_t ii = 0; ii < num_dR; ii++)
	{
		int count = 0;
		int grid_line_num = static_cast<int>((R_neighbor - ii * dR) * sin(60.0 * M_PI / 180.0) / Step) + 1;
		vector<float> diff_Z;

		//diff_Z.reserve(grid_line_num * grid_line_num);
			int cur_neighbor_size = grid_line_num * grid_line_num * fault_tolerant_iterations;
			if (cur_neighbor_size > max_neighbor_size) {
				neighbor_DZ.resize(cur_neighbor_size, 100.0f);
				neighbor_Dist.resize(cur_neighbor_size, 100.0f);
				neighbor_DiffD.resize(cur_neighbor_size, 100.0f);
			}
		for (size_t i = 0; i < num_sectors; i++)
		{
			for (size_t j = 0; j < num_sectors; j++)
			{
				diff_Z.clear();
				float sum_dz = 0.0;
				float sum_dl = 0.0;
				for (size_t k = 0; k < grid_line_num; k++)
				{
					for (size_t l = 0; l < grid_line_num; l++)
					{

						size_t index = k * grid_line_num * fault_tolerant_iterations + l * fault_tolerant_iterations;
						for (size_t m = 0; m < fault_tolerant_iterations; m++)
						{

							if (static_cast<int>(grid_line_num - 1 - k - m) < 0 || static_cast<int>(grid_line_num - 1 - l - m) < 0)
							{
								break;
							}

							float dz_diff = target_rhombus_descriptors[i][k][l].z - source_rhombus_descriptors[j][grid_line_num - 1 - k - m][grid_line_num - 1 - l - m].z;

							if (isnormal(dz_diff))
							{

								neighbor_DZ[index + m] = (target_rhombus_descriptors[i][k][l].z - source_rhombus_descriptors[j][grid_line_num - 1 - k - m][grid_line_num - 1 - l - m].z);
								neighbor_Dist[index + m] = ((target_rhombus_descriptors[i][k][l].x + source_rhombus_descriptors[j][grid_line_num - 1 - k - m][grid_line_num - 1 - l - m].x + target_rhombus_descriptors[i][k][l].y + source_rhombus_descriptors[j][grid_line_num - 1 - k - m][grid_line_num - 1 - l - m].y) / sqrt(3));
								neighbor_DiffD[index + m] = (fabs(target_rhombus_descriptors[i][k][l].x + source_rhombus_descriptors[j][grid_line_num - 1 - k - m][grid_line_num - 1 - l - m].x - target_rhombus_descriptors[i][k][l].y - source_rhombus_descriptors[j][grid_line_num - 1 - k - m][grid_line_num - 1 - l - m].y));


							}
							else
							{
								neighbor_DZ[index + m] = 100.0;
								neighbor_Dist[index + m] = 100.0;
								neighbor_DiffD[index + m] = 100.0;
							}

						}
						size_t min_index = std::min_element(neighbor_DiffD.begin() + index, neighbor_DiffD.begin() + index + fault_tolerant_iterations) - neighbor_DiffD.begin();
						// 获取最小值的索引
						if (neighbor_DiffD[min_index] <= 0.05)  // 差值阈值为 0.05
						{
							sum_dz += neighbor_DZ[min_index];
							sum_dl += neighbor_Dist[min_index];
							diff_Z.push_back(neighbor_DZ[min_index]);

						}


					}
				}

				if (diff_Z.size() > 10/*diff_Z.size() > static_cast<int>(grid_line_num * grid_line_num * 0.1)*/)
				{
					Diff_Zt_Zs[count] = sum_dz / diff_Z.size();
					Diff_Lts[count] = sum_dl / diff_Z.size();
					float Z_std = calculateStandardDeviation(diff_Z, 0.2);
					corr[count][0] = i;
					corr[count][1] = j;
					score[count] = Z_std;
				}
				count += 1;
			}
			const int completed = static_cast<int>(ii) * num_sectors + static_cast<int>(i) + 1;
			const int total = num_dR * num_sectors;
			if (completed % std::max(1, total / 55) == 0)
				reportProgress(20 + 55 * completed / total,
					tr("Matching descriptors: radius %1 of %2 ...").arg(ii + 1).arg(num_dR));
		}

		auto smallest = std::min_element(std::begin(score), std::end(score));                   // 获取最小值迭代器
		int nMinIndex = std::distance(std::begin(score), smallest);                             // 获取最小值索引
		corr_dR[ii][0] = corr[nMinIndex][0] * dTheta;//i j l dz
		corr_dR[ii][1] = corr[nMinIndex][1] * dTheta;//i j l dz
		corr_dR[ii][2] = Diff_Lts[nMinIndex];//i j l dz
		corr_dR[ii][3] = Diff_Zt_Zs[nMinIndex];//i j l dz

		if (Diff_Lts[nMinIndex] >= (R_neighbor - ii * dR))
		{
			corr_dR[ii][2] = R_neighbor - ii * dR;//i j l dz
		}
		if (fabs(Diff_Lts[nMinIndex] - (R_neighbor - ii * dR)) <= 0.1)
		{
			std_dR[ii] = *smallest;

		}
		//std::cout << "ii:" << ii << endl;
		reportProgress(20 + static_cast<int>(55 * (ii + 1) / num_dR),
			tr("Matching descriptors: radius %1 of %2 ...").arg(ii + 1).arg(num_dR));
	}


	auto lest = std::min_element(std::begin(std_dR), std::end(std_dR));                   // 获取最小值迭代器
	int minIndex = std::distance(std::begin(std_dR), lest);                             // 获取最小值索引
	logToLogger(logger, tr("minIndex:") + QString::number(minIndex) + "\n");
	reportProgress(75, tr("Computing feature correspondences ..."));

	// Descriptor data no longer needed, release memory
	target_rhombus_descriptors.clear();
	target_rhombus_descriptors.shrink_to_fit();
	source_rhombus_descriptors.clear();
	source_rhombus_descriptors.shrink_to_fit();

	pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr sourceOverlapAll(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr targetOverlapAll(new pcl::PointCloud<pcl::PointXYZ>);
	int src_rhombus_idx = static_cast<int>(corr_dR[minIndex][1] / dTheta);
	int tgt_rhombus_idx = static_cast<int>(corr_dR[minIndex][0] / dTheta);
	sourceOverlapAll = get_OverlapRhombus_pointclouds(source_cloud, source_rhombus_indices[src_rhombus_idx], corr_dR[minIndex][2], corr_dR[minIndex][1]);
	targetOverlapAll = get_OverlapRhombus_pointclouds(target_cloud, target_rhombus_indices[tgt_rhombus_idx], corr_dR[minIndex][2], corr_dR[minIndex][0]);

	// 菱形索引数据已不再需要，释放内存
	source_rhombus_indices.clear();
	source_rhombus_indices.shrink_to_fit();
	target_rhombus_indices.clear();
	target_rhombus_indices.shrink_to_fit();

	//std::cout << "T-Idx:" << corr_dR[minIndex][0] << " S-Idx:" << corr_dR[minIndex][1] << endl;
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setInputCloud(sourceOverlapAll);             // 设置源点云
	vg.setLeafSize(0.05f, 0.05f, 0.05f); // 设置体素大小
	vg.filter(*source);          // 体素滤波
	vg.setInputCloud(targetOverlapAll);             // 设置目标点云
	vg.setLeafSize(0.05f, 0.05f, 0.05f); // 设置体素大小
	vg.filter(*target);
	/*cout << "source_cloud after filtering: " << source->size() << endl;
	cout << "target_cloud after filtering: " << target->size() << endl;*/
	//---------------计算源点云和目标点云的 FPFH 特征------------------------
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	pcl::PointCloud<pcl::Normal>::Ptr source_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr target_normals(new pcl::PointCloud<pcl::Normal>);
	source_normals = compute_normal(source, tree);
	target_normals = compute_normal(target, tree);

	pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_fpfh = compute_fpfh_feature(source, source_normals, tree);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_fpfh = compute_fpfh_feature(target, target_normals, tree);

	pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> crude_cor_est;
	boost::shared_ptr<pcl::Correspondences> cru_correspondences(new pcl::Correspondences);
	crude_cor_est.setInputSource(source_fpfh);
	crude_cor_est.setInputTarget(target_fpfh);
	//  crude_cor_est.determineCorrespondences(cru_correspondences,1);***********11111111***********
	crude_cor_est.determineReciprocalCorrespondences(*cru_correspondences);
	//cout << "crude size is:" << cru_correspondences->size() << endl;

	Eigen::Vector3f v1_s(cos(corr_dR[minIndex][1] * M_PI / 180.0), sin(corr_dR[minIndex][1] * M_PI / 180.0), 0.0);
	Eigen::Vector3f v2_s(cos((corr_dR[minIndex][1] + 120.0)* M_PI / 180.0), sin((corr_dR[minIndex][1] + 120.0)* M_PI / 180.0), 0.0);
	Eigen::Vector3f v1_t(cos(corr_dR[minIndex][0] * M_PI / 180.0), sin(corr_dR[minIndex][0] * M_PI / 180.0), 0.0);
	Eigen::Vector3f v2_t(cos((corr_dR[minIndex][0] + 120.0)* M_PI / 180.0), sin((corr_dR[minIndex][0] + 120.0)* M_PI / 180.0), 0.0);
	boost::shared_ptr<pcl::Correspondences> correspondences(new pcl::Correspondences);
	pcl::PointCloud<pcl::PointXYZ>::Ptr source0(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr target0(new pcl::PointCloud<pcl::PointXYZ>);
	int total = 0;
	for (int i = 0; i < cru_correspondences->size(); i++)
	{

		Eigen::Vector3f pnt_s(source->points[cru_correspondences->at(i).index_query].x, source->points[cru_correspondences->at(i).index_query].y, 0.0);
		Eigen::Vector3f pnt_t(target->points[cru_correspondences->at(i).index_match].x, target->points[cru_correspondences->at(i).index_match].y, 0.0);


		float d1_s = pnt_s.cross(v1_s).norm();
		float d2_s = pnt_s.cross(v2_s).norm();
		float d1_t = pnt_t.cross(v1_t).norm();
		float d2_t = pnt_t.cross(v2_t).norm();
		float a = fabs(d1_s + d1_t - d2_s - d2_t);
		//cout << "fabs(d1_s+ d1_t- d2_s - d2_t):" << fabs(d1_s + d1_t - d2_s - d2_t) << endl;
		//cout << "fabs(source->points[cru_correspondences->at(i).index_query].z- target->points[cru_correspondences->at(i).index_match].z):" << fabs(source->points[cru_correspondences->at(i).index_query].z - target->points[cru_correspondences->at(i).index_match].z) << endl;
		float b = fabs(target->points[cru_correspondences->at(i).index_match].z - source->points[cru_correspondences->at(i).index_query].z - corr_dR[minIndex][3]);
		float c = fabs(acos(target_normals->points[cru_correspondences->at(i).index_match].normal_z) * 180 / M_PI - acos(source_normals->points[cru_correspondences->at(i).index_query].normal_z) * 180 / M_PI);
		if ((a <= corr_canshu1) && (b <= corr_canshu2) && (c <= corr_canshu3))
		{
			correspondences->push_back(cru_correspondences->at(i));
			total++;


		}
	}
	//cout << "After filter total correspondences is:" << total << endl;


	Eigen::Matrix4f Transform = Eigen::Matrix4f::Identity();
	const int MAX_RANSAC_ITERATIONS = 10000;
	int n_total_pairs = total * (total - 1) / 2;
	int n_ransac = (n_total_pairs > MAX_RANSAC_ITERATIONS) ? MAX_RANSAC_ITERATIONS : n_total_pairs;
	if (n_ransac == 0) {
		logToLogger(logger, tr("No usable feature correspondences found. Registration stopped.") + "\n");
		return;
	}
	reportProgress(80, tr("Estimating the initial transform ..."));
	std::vector<Eigen::Matrix4f> Transform_ransac(n_ransac);
	std::vector<float> score_ransac(n_ransac, 100.0);
	int iteration_ransac = 0;
	for (size_t i = 0; i < correspondences->size(); i++)
	{
		pcl::PointXYZ T1 = target->points[correspondences->at(i).index_match];
		pcl::PointXYZ S1 = source->points[correspondences->at(i).index_query];
		Eigen::Vector3f normal_s1(source_normals->points[correspondences->at(i).index_query].normal_x, source_normals->points[correspondences->at(i).index_query].normal_y, source_normals->points[correspondences->at(i).index_query].normal_z);
		Eigen::Vector3f normal_t1(target_normals->points[correspondences->at(i).index_match].normal_x, target_normals->points[correspondences->at(i).index_match].normal_y, target_normals->points[correspondences->at(i).index_match].normal_z);
		for (size_t j = i + 1; j < correspondences->size(); j++)
		{
			if (iteration_ransac >= n_ransac) break;
			Eigen::Vector3f normal_s2(source_normals->points[correspondences->at(j).index_query].normal_x, source_normals->points[correspondences->at(j).index_query].normal_y, source_normals->points[correspondences->at(j).index_query].normal_z);
			Eigen::Vector3f normal_t2(target_normals->points[correspondences->at(j).index_match].normal_x, target_normals->points[correspondences->at(j).index_match].normal_y, target_normals->points[correspondences->at(j).index_match].normal_z);
			pcl::PointXYZ T2 = target->points[correspondences->at(j).index_match];
			pcl::PointXYZ S2 = source->points[correspondences->at(j).index_query];
			Eigen::Vector3f source_pnt(S2.x - S1.x, S2.y - S1.y, S2.z - S1.z);
			Eigen::Vector3f target_pnt(T2.x - T1.x, T2.y - T1.y, T2.z - T1.z);
			float ds = source_pnt.norm();
			float dt = target_pnt.norm();
			float dD = fabs(dt - ds);
			float dA = fabs(acos(normal_s2.dot(normal_s1)) * 180 / M_PI - acos(normal_t2.dot(normal_t1)) * 180 / M_PI);
			if (ds >= 0.2 && dt >= 0.2 && dD <= 0.1 && dA <= 30.0)
			{
				Transform = calculateFourParameterTransformation(S1, S2, T1, T2);
				pcl::PointCloud<pcl::PointXYZ>::Ptr t_cloud(new pcl::PointCloud<pcl::PointXYZ>());
				// computeHausdorffDistance   ComputeRmse_PCR  computeChamferDistance
				pcl::transformPointCloud(*source, *t_cloud, Transform);
				score_ransac[iteration_ransac] = ComputeRmse_PCR(t_cloud, target, 0.1);
				Transform_ransac[iteration_ransac] = Transform;
			}
			iteration_ransac += 1;
			if (iteration_ransac % std::max(1, n_ransac / 10) == 0)
				reportProgress(80 + 10 * iteration_ransac / n_ransac,
					tr("Estimating transform: pair %1 of %2 ...").arg(iteration_ransac).arg(n_ransac));
		}
		if (iteration_ransac >= n_ransac) break;
	}
	reportProgress(90, tr("Refining registration with ICP ..."));
	auto lest_ransac = std::min_element(std::begin(score_ransac), std::end(score_ransac));                   // 获取最小值迭代器
	int minIndex_ransac = std::distance(std::begin(score_ransac), lest_ransac);                             // 获取最小值索引

	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_src(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*sourceOverlapAll, *transformed_src, Transform_ransac[minIndex_ransac]);
	float rr = sqrt(pow(Transform_ransac[minIndex_ransac](0, 3), 2) + pow(Transform_ransac[minIndex_ransac](1, 3), 2));
	float idx = atan2(Transform_ransac[minIndex_ransac](1, 3), Transform_ransac[minIndex_ransac](0, 3)) * 180.0 / M_PI - 60.0;
	if (idx < 0)
	{
		idx += 360.0;
	}

	vector<vector<pcl::PointCloud<pcl::PointXYZ>>> rhombus_source, rhombus_target;
	rhombus_source = get_PntInrhombus(transformed_src, idx, rr, 0.2);
	rhombus_target = get_PntInrhombus(targetOverlapAll, idx, rr, 0.2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr overlapTransformed_src(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr overlap_tgt(new pcl::PointCloud<pcl::PointXYZ>);

	for (size_t i = 0; i < static_cast<int>(rr * sin(60.0 * M_PI / 180.0) / 0.2) + 1; i++)
	{
		for (size_t j = 0; j < static_cast<int>(rr * sin(60.0 * M_PI / 180.0) / 0.2) + 1; j++)
		{
			if (rhombus_source[i][j].size() >= icp_canshu1 && rhombus_target[i][j].size() >= icp_canshu1)
			{
				float Z_src = 0.0;
				float Z_tgt = 0.0;

				for (size_t k = 0; k < rhombus_source[i][j].size(); k++)
				{
					Z_src += rhombus_source[i][j][k].z;
				}
				Z_src /= rhombus_source[i][j].size();
				for (size_t k = 0; k < rhombus_target[i][j].size(); k++)
				{
					Z_tgt += rhombus_target[i][j][k].z;
				}
				Z_tgt /= rhombus_target[i][j].size();
				if (fabs(Z_tgt - Z_src) <= icp_canshu2)
				{
					*overlap_tgt += rhombus_target[i][j];
					*overlapTransformed_src += rhombus_source[i][j];
				}

			}
		}
	}


	Eigen::Matrix4f transformation_matrix1;

	//-----------------计算参与 ICP 的点云法线-------------------
	pcl::PointCloud<pcl::PointNormal>::Ptr source_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	cloud_with_normal(overlapTransformed_src, source_with_normals);
	pcl::PointCloud<pcl::PointNormal>::Ptr target_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	cloud_with_normal(overlap_tgt, target_with_normals);
	//----------------点到面 ICP 配准-----------------
	pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal>p_icp;
	p_icp.setInputSource(source_with_normals);
	p_icp.setInputTarget(target_with_normals);
	p_icp.setTransformationEpsilon(1e-10);    // 变换收敛阈值
	p_icp.setMaxCorrespondenceDistance(0.03);   // 最大对应点距离
	p_icp.setEuclideanFitnessEpsilon(0.0001);  // 适应度收敛阈值
	//p_icp.setUseSymmetricObjective(true);   // 可选的对称目标函数
	p_icp.setMaximumIterations(35);           // 最大迭代次数
	pcl::PointCloud<pcl::PointNormal>::Ptr p_icp_cloud(new pcl::PointCloud<pcl::PointNormal>);
	p_icp.align(*p_icp_cloud);
	reportProgress(95, tr("Saving registration results ..."));

	transformation_matrix1 = p_icp.getFinalTransformation();
	transformation_matrix1 = transformation_matrix1 * Transform_ransac[minIndex_ransac];

	logToLogger(logger, tr("The Fine Registration finished!") + "\n");
	logToLogger(logger, tr("Point cloud registration completed!") + "\n");
	logToLogger(logger, tr("===================================") + "\n");
	logToLogger(logger, tr("Transformation Matrix:") + "\n");

	logToLogger(logger, QString::number(transformation_matrix1(0, 0), 'f', 6) + "\t" +
		QString::number(transformation_matrix1(0, 1), 'f', 6) + "\t" +
		QString::number(transformation_matrix1(0, 2), 'f', 6) + "\t" +
		QString::number(transformation_matrix1(0, 3), 'f', 6) + "\n");

	logToLogger(logger, QString::number(transformation_matrix1(1, 0), 'f', 6) + "\t" +
		QString::number(transformation_matrix1(1, 1), 'f', 6) + "\t" +
		QString::number(transformation_matrix1(1, 2), 'f', 6) + "\t" +
		QString::number(transformation_matrix1(1, 3), 'f', 6) + "\n");

	logToLogger(logger, QString::number(transformation_matrix1(2, 0), 'f', 6) + "\t" +
		QString::number(transformation_matrix1(2, 1), 'f', 6) + "\t" +
		QString::number(transformation_matrix1(2, 2), 'f', 6) + "\t" +
		QString::number(transformation_matrix1(2, 3), 'f', 6) + "\n");

	logToLogger(logger, QString::number(transformation_matrix1(3, 0), 'f', 6) + "\t" +
		QString::number(transformation_matrix1(3, 1), 'f', 6) + "\t" +
		QString::number(transformation_matrix1(3, 2), 'f', 6) + "\t" +
		QString::number(transformation_matrix1(3, 3), 'f', 6) + "\n");

	logToLogger(logger, tr("===================================") + "\n");

	std::filesystem::path sourcePath(params.sourceFile.toStdString());
	std::filesystem::path targetPath(params.targetFile.toStdString());

	std::string outputDir = params.outputDir.toStdString();
	std::ofstream dataOut(outputDir + "/" + sourcePath.stem().string() + "_to_" + targetPath.stem().string() + "_transformationMatrix.txt");

	if (!dataOut)
	{
		logToLogger(logger, tr("Failed to create transformation matrix file!") + "\n");
		return;
	}

	dataOut << std::fixed << std::setprecision(6);
	dataOut << transformation_matrix1(0, 0) << "\t" << transformation_matrix1(0, 1) << "\t" << transformation_matrix1(0, 2) << "\t" << transformation_matrix1(0, 3) << "\n";
	dataOut << transformation_matrix1(1, 0) << "\t" << transformation_matrix1(1, 1) << "\t" << transformation_matrix1(1, 2) << "\t" << transformation_matrix1(1, 3) << "\n";
	dataOut << transformation_matrix1(2, 0) << "\t" << transformation_matrix1(2, 1) << "\t" << transformation_matrix1(2, 2) << "\t" << transformation_matrix1(2, 3) << "\n";
	dataOut << transformation_matrix1(3, 0) << "\t" << transformation_matrix1(3, 1) << "\t" << transformation_matrix1(3, 2) << "\t" << transformation_matrix1(3, 3) << "\n";
	dataOut.close();

	logToLogger(logger, tr("Transformed information saved to: ") + QString::fromStdString(outputDir + "/" + sourcePath.stem().string() + "_to_" + targetPath.stem().string() + "_transformationMatrix.txt") + "\n");


	pcl::transformPointCloud(*source_cloud, *target_cloud, transformation_matrix1);

	std::string outputSource = outputDir + "/" + sourcePath.stem().string() + "_registered.pcd";

	pcl::io::savePCDFileBinary(outputSource, *target_cloud);

	logToLogger(logger, tr("Registered point cloud saved to: ") + QString::fromStdString(outputSource) + "\n");
	logToLogger(logger, tr("===================================") + "\n");
}

