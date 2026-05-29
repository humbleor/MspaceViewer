#include "../include/ForestRegistration.h"
#include <QScrollBar>

// Forest_TLS_Reg headers
#ifndef _HLP_H_Included_
#define _HLP_H_Included_
#include "../../Forest_TLS_Reg/include/utils/Hlp.h"
#endif

#ifndef _DST_H_Included_
#define _DST_H_Included_
#include "../../Forest_TLS_Reg/include/dst/DST.h"
#endif

#ifndef _HashRegObj_H_Included_
#define _HashRegObj_H_Included_
#include "../../Forest_TLS_Reg/include/utils/HashRegObj.h"
#endif

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>

static void logToLoggerForest(QTextEdit* logger, const QString& text)
{
	if (!logger) return;
	QMetaObject::invokeMethod(logger, [logger, text]() {
		logger->insertPlainText(text);
		logger->verticalScrollBar()->setValue(logger->verticalScrollBar()->maximum());
	}, Qt::QueuedConnection);
}

ForestRegistration::ForestRegistration(QWidget* parent)
	:QDialog(parent),
	Ui::ForestRegistration()
{
	this->setupUi(this);

	connect(_selectInputFileOfSource, &QPushButton::clicked, this, &ForestRegistration::selectInputFileOfSource);
	connect(_selectInputFileOfTarget, &QPushButton::clicked, this, &ForestRegistration::selectInputFileOfTarget);
	connect(_selectConfigFile, &QPushButton::clicked, this, &ForestRegistration::selectConfigFile);
	connect(_selectOutputDir, &QPushButton::clicked, this, &ForestRegistration::selectOutputDir);
	connect(_action_OK, &QPushButton::clicked, this, &ForestRegistration::apply);
	connect(_action_cancel, &QPushButton::clicked, this, &ForestRegistration::reject);

	initParam();
}

ForestRegistration::~ForestRegistration()
{
}

void ForestRegistration::selectInputFileOfSource()
{
	QStringList fileTypes;
	fileTypes << "Point Cloud Files (*.las *.laz *.ply *.pcd)"
			  << "LAS Files (*.las)"
			  << "LAZ Files (*.laz)"
			  << "PLY Files (*.ply)"
			  << "PCD Files (*.pcd)";
	QString file = QFileDialog::getOpenFileName(this, tr("Select Source File"), "", fileTypes.join(";;"));
	if (file.isEmpty())
		return;
	_inputFileOfSource->setText(file);
}

void ForestRegistration::selectInputFileOfTarget()
{
	QStringList fileTypes;
	fileTypes << "Point Cloud Files (*.las *.laz *.ply *.pcd)"
			  << "LAS Files (*.las)"
			  << "LAZ Files (*.laz)"
			  << "PLY Files (*.ply)"
			  << "PCD Files (*.pcd)";
	QString file = QFileDialog::getOpenFileName(this, tr("Select Target File"), "", fileTypes.join(";;"));
	if (file.isEmpty())
		return;
	_inputFileOfTarget->setText(file);
}

void ForestRegistration::selectConfigFile()
{
	QStringList fileTypes;
	fileTypes << "YAML Files (*.yaml)";
	QString file = QFileDialog::getOpenFileName(this, tr("Select Config File"), "", fileTypes.join(";;"));
	if (file.isEmpty())
		return;
	_configFile->setText(file);
}

void ForestRegistration::selectOutputDir()
{
	QString dir = QFileDialog::getExistingDirectory(this, tr("Select Output Directory"), "");
	if (dir.isEmpty())
		return;
	_outputFileOfDir->setText(dir);
}

void ForestRegistration::executeRegistration(QProgressDialog* progress, QTextEdit* logger)
{
	if (_inputFileOfSource->text().isEmpty() ||
		_inputFileOfTarget->text().isEmpty() ||
		_outputFileOfDir->text().isEmpty())
		return;

	ForestRegParams params;
	params.sourceFile = _inputFileOfSource->text();
	params.targetFile = _inputFileOfTarget->text();
	params.configFile = _configFile->text().isEmpty() ? "" : _configFile->text();
	params.outputDir = _outputFileOfDir->text();

	if (progress)
	{
		progress->setLabelText(tr("Performing forest point cloud registration ..."));
		progress->setWindowFlags(progress->windowFlags() & ~Qt::WindowCloseButtonHint & ~Qt::WindowContextHelpButtonHint);
		progress->setCancelButton(nullptr);
		progress->show();
		progress->raise();
	}

	// Use shared_ptr to keep this alive during the background task
	auto self = shared_from_this();
	QFuture<void> future = QtConcurrent::run([self, params, logger]() {
		self->registration(params, logger);
	});
	while (!future.isFinished())
	{
		if (progress)
		{
			progress->setValue(progress->value() + 1);
			QApplication::processEvents();
		}
	}
}

void ForestRegistration::apply()
{
	this->done(QDialog::Accepted);
}

void ForestRegistration::reject()
{
	this->done(QDialog::Rejected);
}

void ForestRegistration::initParam()
{
	_configFile->setText("");
}

void ForestRegistration::registration(ForestRegParams params, QTextEdit* logger)
{
	try {
		std::string sourceFile = params.sourceFile.toStdString();
		std::string targetFile = params.targetFile.toStdString();
		std::string outputDir = params.outputDir.toStdString();
		std::string configFile = params.configFile.toStdString();

		// --- Step 1: Load config settings ---
		ConfigSetting config_setting;
		if (!configFile.empty())
		{
			logToLoggerForest(logger, tr("Loading config file: ") + QString::fromStdString(configFile) + "\n");
			ReadParas(configFile, config_setting);
		}
		else
		{
			logToLoggerForest(logger, tr("No config file provided, using default parameters.\n"));
		}

		// --- Step 2: Load point clouds ---
		// Helper lambda: load point cloud by file extension
		auto loadPointCloud = [logger](const std::string& filePath, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) -> bool {
			std::string ext = filePath.size() > 4 ? filePath.substr(filePath.size() - 4) : "";
			std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

			if (ext == ".las" || ext == ".laz")
			{
				readTLSData(filePath, cloud);
			}
			else if (ext == ".ply")
			{
				pcl::PointCloud<pcl::PointXYZ> tmp;
				if (pcl::io::loadPLYFile(filePath, tmp) < 0)
				{
					logToLoggerForest(logger, tr("Failed to load PLY file: ") + QString::fromStdString(filePath) + "\n");
					return false;
				}
				*cloud = tmp;
			}
			else if (ext == ".pcd")
			{
				pcl::PointCloud<pcl::PointXYZ> tmp;
				if (pcl::io::loadPCDFile(filePath, tmp) < 0)
				{
					logToLoggerForest(logger, tr("Failed to load PCD file: ") + QString::fromStdString(filePath) + "\n");
					return false;
				}
				*cloud = tmp;
			}
			else
			{
				logToLoggerForest(logger, tr("Unsupported file format: ") + QString::fromStdString(ext) + "\n");
				return false;
			}
			return true;
		};

		logToLoggerForest(logger, tr("Loading target (reference) point cloud...") + "\n");
		pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		try {
			if (!loadPointCloud(targetFile, target_cloud)) return;
		} catch (const std::exception& e) {
			logToLoggerForest(logger, tr("Error loading target cloud: ") + QString::fromUtf8(e.what()) + "\n");
			return;
		}
		if (target_cloud->empty()) {
			logToLoggerForest(logger, tr("Target cloud is empty!\n"));
			return;
		}
		logToLoggerForest(logger, tr("Target points: %1\n").arg(target_cloud->size()));

		logToLoggerForest(logger, tr("Loading source point cloud...") + "\n");
		pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		try {
			if (!loadPointCloud(sourceFile, source_cloud)) return;
		} catch (const std::exception& e) {
			logToLoggerForest(logger, tr("Error loading source cloud: ") + QString::fromUtf8(e.what()) + "\n");
			return;
		}
		if (source_cloud->empty()) {
			logToLoggerForest(logger, tr("Source cloud is empty!\n"));
			return;
		}
		logToLoggerForest(logger, tr("Source points: %1\n").arg(source_cloud->size()));

		// --- Step 3: Generate descriptors for target ---
		logToLoggerForest(logger, tr("Generating triangle descriptors for target cloud...") + "\n");
		HashRegDescManager* hashReg = nullptr;
		try {
			hashReg = new HashRegDescManager(config_setting);
		} catch (const std::exception& e) {
			logToLoggerForest(logger, tr("Error creating HashRegDescManager: ") + QString::fromUtf8(e.what()) + "\n");
			return;
		}

		FrameInfo reference_info;
		try {
			hashReg->GenTriDescs(target_cloud, reference_info);
			hashReg->AddTriDescs(reference_info);
		} catch (const std::exception& e) {
			logToLoggerForest(logger, tr("Error generating target descriptors: ") + QString::fromUtf8(e.what()) + "\n");
			delete hashReg;
			return;
		}
		logToLoggerForest(logger, tr("Target descriptors: %1\n").arg(reference_info.desc_.size()));

		// --- Step 4: Generate descriptors for source ---
		logToLoggerForest(logger, tr("Generating triangle descriptors for source cloud...") + "\n");
		FrameInfo source_info;
		try {
			hashReg->GenTriDescs(source_cloud, source_info);
		} catch (const std::exception& e) {
			logToLoggerForest(logger, tr("Error generating source descriptors: ") + QString::fromUtf8(e.what()) + "\n");
			delete hashReg;
			return;
		}
		logToLoggerForest(logger, tr("Source descriptors: %1\n").arg(source_info.desc_.size()));

		// --- Step 5: Search for matching triangle pairs (coarse registration) ---
		logToLoggerForest(logger, tr("Searching for matching triangle pairs...") + "\n");
		std::pair<int, double> search_result(-1, 0);
		std::pair<Eigen::Vector3d, Eigen::Matrix3d> coarse_transform;
		coarse_transform.first << 0, 0, 0;
		coarse_transform.second = Eigen::Matrix3d::Identity();
		std::vector<std::pair<TriDesc, TriDesc>> loop_triangle_pair;

		try {
			hashReg->SearchPosition(source_info, search_result, coarse_transform, loop_triangle_pair);
		} catch (const std::exception& e) {
			logToLoggerForest(logger, tr("Error during search: ") + QString::fromUtf8(e.what()) + "\n");
			delete hashReg;
			return;
		}

		if (search_result.first == -1)
		{
			logToLoggerForest(logger, tr("Warning: No matching triangles found. Skipping coarse registration.\n"));
			delete hashReg;
			return;
		}

		logToLoggerForest(logger, tr("Match found! Frame ID: %1, Score: %2\n").arg(search_result.first).arg(search_result.second, 0, 'f', 4));

		// --- Step 6: Fine registration (small_gicp) ---
		logToLoggerForest(logger, tr("Performing fine registration (Small-GICP)...") + "\n");
		std::pair<Eigen::Vector3d, Eigen::Matrix3d> refine_transform;
		try {
			small_gicp_registration(source_cloud, target_cloud, refine_transform);
		} catch (const std::exception& e) {
			logToLoggerForest(logger, tr("Error during fine registration: ") + QString::fromUtf8(e.what()) + "\n");
			delete hashReg;
			return;
		}
		logToLoggerForest(logger, tr("Fine registration completed.\n"));

		// --- Step 7: Combine transforms ---
		Eigen::Matrix4d final_matrix = Eigen::Matrix4d::Identity();
		Eigen::Matrix4d coarse_matrix = Eigen::Matrix4d::Identity();
		coarse_matrix.block<3, 3>(0, 0) = coarse_transform.second;
		coarse_matrix.block<3, 1>(0, 3) = coarse_transform.first;

		Eigen::Matrix4d refine_matrix = Eigen::Matrix4d::Identity();
		refine_matrix.block<3, 3>(0, 0) = refine_transform.second;
		refine_matrix.block<3, 1>(0, 3) = refine_transform.first;

		final_matrix = refine_matrix * coarse_matrix;

		// --- Step 8: Save transformation matrix ---
		std::filesystem::path sourcePath(sourceFile);
		std::filesystem::path targetPath(targetFile);
		std::string outFile = outputDir + "/" + sourcePath.stem().string() + "_to_" + targetPath.stem().string() + "_transformationMatrix.txt";

		std::ofstream dataOut(outFile);
		if (!dataOut)
		{
			logToLoggerForest(logger, tr("Failed to create transformation matrix file!\n"));
			delete hashReg;
			return;
		}

		dataOut << std::fixed << std::setprecision(6);
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				dataOut << final_matrix(i, j);
				if (j < 3) dataOut << "\t";
			}
			dataOut << "\n";
		}
		dataOut.close();

		// --- Print matrix to logger window ---
		logToLoggerForest(logger, tr("Transformation Matrix (4x4):\n"));
		for (int i = 0; i < 4; i++)
		{
			QString line;
			for (int j = 0; j < 4; j++)
			{
				line += QString("%1").arg(final_matrix(i, j), 16, 'f', 6);
			}
			logToLoggerForest(logger, line + "\n");
		}
		logToLoggerForest(logger, tr("Transform matrix saved to: ") + QString::fromStdString(outFile) + "\n");
		logToLoggerForest(logger, tr("===================================\n"));
		logToLoggerForest(logger, tr("Forest registration completed!\n"));

		delete hashReg;
	} catch (...) {
		logToLoggerForest(logger, tr("Unknown exception occurred during registration!\n"));
	}
}
