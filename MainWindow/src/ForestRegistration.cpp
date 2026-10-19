#include "../include/ForestRegistration.h"
#include "../include/AsyncRegistration.h"
#include <QScrollBar>
#include "../include/RegistrationProcess.h"
#include <QComboBox>
#include <QStackedWidget>
#include <QDoubleSpinBox>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QDir>
#include <QFileInfo>
#include <cmath>

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
#include <pcl/common/transforms.h>

#include <algorithm>
#include "utils/ForestPairRegistration.h"

static void logToLoggerForest(QPointer<QTextEdit> logger, const QString& text)
{
	registrationui::log(logger, text, true);
}

ForestRegistration::ForestRegistration(QWidget* parent)
	:QDialog(parent),
	Ui::ForestRegistration()
{
	this->setupUi(this);
	_registrationMode = new QComboBox(this);
	_registrationMode->setObjectName("_registrationMode");
	_registrationMode->addItem(tr("双点云配准（Forest + GICP）"));
#ifdef FOREST_HAS_GLOBAL_OPTIMIZATION
	_registrationMode->addItem(tr("多点云全局优化（Forest + GICP + GTSAM）"));
#endif
	verticalLayout->insertWidget(0, _registrationMode);
	auto* globalGroup = new QGroupBox(tr("多点云输入目录（LAS / PCD / PLY）"), this);
	auto* globalLayout = new QVBoxLayout(globalGroup);
	auto* directoryRow = new QHBoxLayout;
	_pointCloudDirectory = new QLineEdit(globalGroup);
	_pointCloudDirectory->setObjectName("_pointCloudDirectory");
	auto* selectDirectory = new QPushButton(tr("选择目录"), globalGroup);
	selectDirectory->setObjectName("_selectPointCloudDirectory");
	directoryRow->addWidget(_pointCloudDirectory); directoryRow->addWidget(selectDirectory);
	globalLayout->addLayout(directoryRow);
	auto* outputRow = new QHBoxLayout;
	outputRow->addWidget(new QLabel(tr("LAS 合并点云分辨率（m，0 = 原始点）："), globalGroup));
	_mergeResolution = new QDoubleSpinBox(globalGroup);
	_mergeResolution->setObjectName("_mergeResolution");
	_mergeResolution->setDecimals(3); _mergeResolution->setRange(0, 10); _mergeResolution->setSingleStep(0.05); _mergeResolution->setValue(0.05);
	outputRow->addWidget(_mergeResolution); globalLayout->addLayout(outputRow);
	// The stack reserves the maximum size of both input pages. Switching modes
	// changes only the visible page, keeping the dialog geometry stable.
	verticalLayout->removeWidget(groupBox);
	auto* inputPages = new QStackedWidget(this);
	inputPages->addWidget(groupBox);
	inputPages->addWidget(globalGroup);
	verticalLayout->insertWidget(1, inputPages);
	connect(selectDirectory, &QPushButton::clicked, this, [this]() {
		const QString dir = QFileDialog::getExistingDirectory(this, tr("选择点云目录"));
		if (!dir.isEmpty()) _pointCloudDirectory->setText(dir);
	});
	connect(_registrationMode, &QComboBox::currentIndexChanged, inputPages, &QStackedWidget::setCurrentIndex);

	connect(_selectInputFileOfSource, &QPushButton::clicked, this, &ForestRegistration::selectInputFileOfSource);
	connect(_selectInputFileOfTarget, &QPushButton::clicked, this, &ForestRegistration::selectInputFileOfTarget);
	connect(_selectOutputDir, &QPushButton::clicked, this, &ForestRegistration::selectOutputDir);
	connect(_action_OK, &QPushButton::clicked, this, &ForestRegistration::apply);
	connect(_action_cancel, &QPushButton::clicked, this, &ForestRegistration::reject);

	initParam();
	// Use a common initial size, while allowing the user to resize the dialog.
	// The stacked pages keep that user-selected size when changing modes.
	setWindowFlag(Qt::MSWindowsFixedSizeDialogHint, false);
	setWindowFlag(Qt::WindowMaximizeButtonHint, true);
	setSizeGripEnabled(true);
	verticalLayout->setSizeConstraint(QLayout::SetDefaultConstraint);
	resize(QSize(760, 450).expandedTo(minimumSizeHint()));
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

void ForestRegistration::selectOutputDir()
{
	QString dir = QFileDialog::getExistingDirectory(this, tr("Select Output Directory"), "");
	if (dir.isEmpty())
		return;
	_outputFileOfDir->setText(dir);
}

bool ForestRegistration::executeRegistration(QProgressDialog* progress, QTextEdit* logger,
	std::function<void()> onFinished)
{
	ForestRegParams params;
	params.sourceFile = _inputFileOfSource->text();
	params.targetFile = _inputFileOfTarget->text();
	params.outputDir = _outputFileOfDir->text();
	bool numeric = true;
	const auto readNumber = [&numeric](QLineEdit* edit) { bool ok = false; double v = edit->text().toDouble(&ok); numeric = numeric && ok; return v; };
	params.descriptorNearNum = readNumber(_descriptorNearNum);
	params.descriptorMinLen  = readNumber(_descriptorMinLen);
	params.descriptorMaxLen  = readNumber(_descriptorMaxLen);
	params.disGeoVerify      = readNumber(_disGeoVerify);
	params.icpThreshold      = readNumber(_icpThreshold);
	params.normalGeoVerify   = readNumber(_normalGeoVerify);
	params.globalOptimization = _registrationMode->currentIndex() == 1;
	params.pointCloudDirectory = _pointCloudDirectory->text().trimmed();
	params.mergeResolution = _mergeResolution->value();
	if (!numeric || params.outputDir.trimmed().isEmpty() ||
		(params.globalOptimization ? !QDir(params.pointCloudDirectory).exists() || params.pointCloudDirectory.isEmpty() :
			!QFileInfo(params.sourceFile).isFile() || !QFileInfo(params.targetFile).isFile()) ||
		!std::isfinite(params.icpThreshold) || params.icpThreshold < 0 || params.icpThreshold > 1 ||
		!std::isfinite(params.descriptorNearNum) || params.descriptorNearNum < 3 || params.descriptorNearNum > 100 || std::floor(params.descriptorNearNum) != params.descriptorNearNum ||
		!std::isfinite(params.descriptorMinLen) || params.descriptorMinLen <= 0 ||
		!std::isfinite(params.descriptorMaxLen) || params.descriptorMaxLen <= params.descriptorMinLen ||
		!std::isfinite(params.disGeoVerify) || params.disGeoVerify <= 0 ||
		!std::isfinite(params.normalGeoVerify) || params.normalGeoVerify < 0 || params.normalGeoVerify > 2)
	{
		QMessageBox::warning(this, tr("参数错误"), tr("请选择有效输入、输出目录，并检查配准参数。"));
		return false;
	}
	if (!QDir().mkpath(params.outputDir)) {
		QMessageBox::warning(this, tr("参数错误"), tr("无法创建输出目录。")); return false;
	}

	if (!progress)
		return false;
#ifdef FOREST_HAS_GLOBAL_OPTIMIZATION
	if (params.globalOptimization) {
		const auto number = [](double v) { return QString::number(v, 'g', 12); };
		return registrationui::startProcess(progress, logger,
			QCoreApplication::applicationDirPath() + "/runtime/ForestGlobalOptimization.exe",
			{"--input", QDir(params.pointCloudDirectory).absolutePath(), "--output", QDir(params.outputDir).absolutePath(),
			 "--coarse-score", number(params.icpThreshold), "--neighbors", number(params.descriptorNearNum),
			 "--min-edge-length", number(params.descriptorMinLen), "--max-edge-length", number(params.descriptorMaxLen),
			 "--geo-distance", number(params.disGeoVerify), "--normal-difference", number(params.normalGeoVerify),
			 "--merge-resolution", number(params.mergeResolution), "--gui-progress"},
			"FOREST_PROGRESS\t", params.outputDir, std::move(onFinished));
	}
#endif
	registrationui::start(progress, logger, tr("Loading forest point clouds ..."),
		std::move(onFinished), [params](QPointer<QTextEdit> loggerGuard, registrationui::ProgressReporter report) {
			registration(params, loggerGuard, report);
		});
	return true;
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
	_descriptorNearNum->setText("10");
	_descriptorMinLen->setText("2.0");
	_descriptorMaxLen->setText("50.0");
	_disGeoVerify->setText("0.3");
	_icpThreshold->setText("0.25");
	_normalGeoVerify->setText("1.0");
}

void ForestRegistration::registration(ForestRegParams params, QPointer<QTextEdit> logger,
	std::function<void(int, const QString&)> reportProgress)
{
	try {
		std::string sourceFile = params.sourceFile.toStdString();
		std::string targetFile = params.targetFile.toStdString();
		std::string outputDir = params.outputDir.toStdString();

		// --- Step 1: Build config settings from UI values ---
		ConfigSetting config_setting;
		config_setting.descriptor_near_num  = static_cast<int>(params.descriptorNearNum);
		config_setting.descriptor_min_len   = params.descriptorMinLen;
		config_setting.descriptor_max_len   = params.descriptorMaxLen;
		config_setting.dis_geo_verify       = params.disGeoVerify;
		config_setting.icp_threshold       = params.icpThreshold;
		config_setting.normal_geo_verify   = params.normalGeoVerify;
		// ponytail: these three defaults differ between DST.h and all 8 known forest datasets;
		// pin to the dataset-common value so registration doesn't silently fail on defaults.
		config_setting.rough_dis_threshold        = 0.03;
		config_setting.dist_candi_frames_verify   = 1.0;
		config_setting.use_matched_num            = 4000;
		logToLoggerForest(logger, tr("Using UI parameters (no config file loaded).\n"));

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
		reportProgress(10, tr("Loading source point cloud ..."));
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

        const auto result = forest::registerPair(source_cloud, target_cloud, config_setting,
            [logger, reportProgress](int progress, const std::string& message) {
                const auto text = QString::fromStdString(message);
                logToLoggerForest(logger, text + "\n");
                reportProgress(progress, text);
            });
        const auto reportName = std::filesystem::path(sourceFile).stem().string() + "_to_" +
            std::filesystem::path(targetFile).stem().string() + "_forest_quality.txt";
        forest::savePairReport((std::filesystem::path(outputDir) / reportName).string(), result);
        if (!result.accepted) {
            logToLoggerForest(logger, "Forest registration rejected: " + QString::fromStdString(result.reason) + "\n");
            return;
        }
        const Eigen::Matrix4d final_matrix = result.transform;

		// --- Step 8: Save transformation matrix ---
		std::filesystem::path sourcePath(sourceFile);
		std::filesystem::path targetPath(targetFile);
		std::string outFile = outputDir + "/" + sourcePath.stem().string() + "_to_" + targetPath.stem().string() + "_transformationMatrix.txt";

		std::ofstream dataOut(outFile);
		reportProgress(90, tr("Saving transformation and point cloud ..."));
		if (!dataOut)
		{
			logToLoggerForest(logger, tr("Failed to create transformation matrix file!\n"));
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

		// --- Step 9: Transform source cloud and save ---
		pcl::PointCloud<pcl::PointXYZ>::Ptr registered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::transformPointCloud(*source_cloud, *registered_cloud, final_matrix);

		std::string registeredFile = outputDir + "/" + sourcePath.stem().string() + "_registered.las";
		writeLas(registeredFile, registered_cloud);
		logToLoggerForest(logger, tr("Registered point cloud saved to: ") + QString::fromStdString(registeredFile) + "\n");
		logToLoggerForest(logger, tr("Registered points: %1\n").arg(registered_cloud->size()));
		logToLoggerForest(logger, tr("===================================\n"));
		logToLoggerForest(logger, tr("Forest registration completed!\n"));

	} catch (const std::exception& e) {
        logToLoggerForest(logger, "Forest registration failed: " + QString::fromUtf8(e.what()) + "\n");
	} catch (...) {
		logToLoggerForest(logger, tr("Unknown exception occurred during registration!\n"));
	}
}
