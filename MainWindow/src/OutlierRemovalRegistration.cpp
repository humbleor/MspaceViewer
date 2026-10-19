#include "../include/OutlierRemovalRegistration.h"
#include "../include/AsyncRegistration.h"
#ifdef D4PCR_HAS_GLOBAL_OPTIMIZATION
#include <GlobalOptimizationPipeline.h>
#endif
#include <QMessageBox>
#include <QFileInfo>
#include <QFile>
#include <QDir>
#include <QCoreApplication>
#include <QProcess>
#include <QProcessEnvironment>
#include <memory>
#include <cmath>
#include <stdexcept>

static void logToLoggerORR(QPointer<QTextEdit> logger, const QString& text)
{
	registrationui::log(logger, text);
}

#ifdef D4PCR_HAS_GLOBAL_OPTIMIZATION
// Run the global pipeline in its own process so even blocking LAS/PCL calls
// can be stopped without terminating a thread inside the GUI process.
static bool startGlobalRegistration(QProgressDialog* progress, QTextEdit* logger,
	const QString& input, const QString& output, float resolution,
	std::function<void()> onFinished)
{
	const QString program = QCoreApplication::applicationDirPath() + "/runtime/D4PCRGlobalOptimization.exe";
	if (!QFileInfo::exists(program))
	{
		QMessageBox::warning(progress, OutlierRemovalRegistration::tr("Registration failed"),
			OutlierRemovalRegistration::tr("D4PCRGlobalOptimization.exe is missing. Please rebuild or reinstall the application."));
		return false;
	}
	struct ProcessState {
		bool canceled{ false };
		bool finished{ false };
		QByteArray pending;
	};
	auto state = std::make_shared<ProcessState>();
	QPointer<QProgressDialog> progressGuard(progress);
	QPointer<QTextEdit> loggerGuard(logger);
	auto* worker = new QProcess(progress);
	worker->setProcessChannelMode(QProcess::MergedChannels);
	worker->setWorkingDirectory(QCoreApplication::applicationDirPath());
	// D4PCRGlobalOptimization.exe 位于 runtime/ 子目录，而 DLL（PCL/GTSAM/Qt 等）在程序根目录；
	// 把根目录加入子进程 PATH，使其能按应用目录解析到这些 DLL。
	QProcessEnvironment environment = QProcessEnvironment::systemEnvironment();
	environment.insert("PATH",
		QCoreApplication::applicationDirPath() + ";" + environment.value("PATH"));
	worker->setProcessEnvironment(environment);
	progress->setRange(0, 100);
	progress->setValue(0);
	progress->setLabelText(OutlierRemovalRegistration::tr("Starting global registration ..."));
	progress->setCancelButton(nullptr);
	progress->setWindowFlags(progress->windowFlags() | Qt::WindowCloseButtonHint);
	progress->setAutoClose(false);
	progress->setAutoReset(false);
	progress->setWindowModality(Qt::NonModal);

	auto consumeOutput = [state, progressGuard, loggerGuard](QByteArray data, bool flush) {
		state->pending += data;
		while (!state->pending.isEmpty())
		{
			const auto newline = state->pending.indexOf('\n');
			if (newline < 0 && !flush)
				break;
			const QByteArray line = (newline < 0 ? state->pending : state->pending.left(newline)).trimmed();
			state->pending.remove(0, newline < 0 ? state->pending.size() : newline + 1);
			if (line.startsWith("D4PCR_PROGRESS\t"))
			{
				const auto separator = line.indexOf('\t', 15);
				bool valid = false;
				const int value = line.mid(15, separator - 15).toInt(&valid);
				if (separator < 0 || !valid)
					continue;
				const QString message = QString::fromLocal8Bit(line.mid(separator + 1));
				if (progressGuard && !state->canceled)
				{
					progressGuard->setLabelText(message);
					progressGuard->setValue(value);
				}
				logToLoggerORR(loggerGuard, message + "\n");
			}
			else if (!line.isEmpty())
				logToLoggerORR(loggerGuard, QString::fromLocal8Bit(line) + "\n");
		}
	};
	auto finish = [state, progressGuard, loggerGuard, worker, consumeOutput, output,
		onFinished = std::move(onFinished)](int exitCode, QProcess::ExitStatus exitStatus) {
		if (state->finished)
			return;
		state->finished = true;
		consumeOutput(worker->readAllStandardOutput(), true);
		if (state->canceled)
			logToLoggerORR(loggerGuard, OutlierRemovalRegistration::tr("Registration canceled. Any output files may be incomplete.") + "\n");
		else if (exitStatus != QProcess::NormalExit)
			logToLoggerORR(loggerGuard, OutlierRemovalRegistration::tr("Registration failed: ") + worker->errorString() + "\n");
		else if (exitCode != 0)
			logToLoggerORR(loggerGuard, OutlierRemovalRegistration::tr("Registration failed with exit code %1.").arg(exitCode) + "\n");
		else
			logToLoggerORR(loggerGuard, OutlierRemovalRegistration::tr("Global poses, pairwise edges and merged cloud saved to: ") + output + "\n");
		if (progressGuard)
		{
			progressGuard->close();
			progressGuard->deleteLater();
		}
		if (onFinished)
			onFinished();
	};
	QObject::connect(worker, &QProcess::readyReadStandardOutput, progress,
		[worker, consumeOutput]() { consumeOutput(worker->readAllStandardOutput(), false); });
	QObject::connect(worker, &QProcess::finished, progress, finish);
	QObject::connect(worker, &QProcess::started, progress,
		[state, worker]() { if (state->canceled) worker->kill(); });
	QObject::connect(worker, &QProcess::errorOccurred, progress,
		[finish](QProcess::ProcessError error) {
			if (error == QProcess::FailedToStart)
				finish(-1, QProcess::CrashExit);
		});
	QObject::connect(progress, &QProgressDialog::canceled, progress,
		[state, worker, progressGuard]() {
			if (state->finished || state->canceled)
				return;
			state->canceled = true;
			if (progressGuard)
			{
				progressGuard->setLabelText(OutlierRemovalRegistration::tr("Stopping registration ..."));
				progressGuard->setCancelButton(nullptr);
				progressGuard->show();
			}
			worker->kill();
		});
	logToLoggerORR(loggerGuard, OutlierRemovalRegistration::tr("Running D4PCR + 4-DOF ICP + GTSAM global optimization ...") + "\n");
	worker->start(program, { QDir(input).absolutePath(), QDir(output).absolutePath(),
		QString::number(resolution, 'g', 9), "--gui-progress" });
	progress->show();
	return true;
}
#endif

OutlierRemovalRegistration::OutlierRemovalRegistration(QWidget* parent)
	:QDialog(parent),
	Ui::OutliersRemovalRegistration()
{
	this->setupUi(this);
	_registrationMode = new QComboBox(this);
	_registrationMode->setObjectName("_registrationMode");
	_registrationMode->setSizeAdjustPolicy(QComboBox::AdjustToContents);
#ifdef D4PCR_HAS_GLOBAL_OPTIMIZATION
	_registrationMode->addItem(tr("双点云配准（D4PCR + ICP）"));
	_registrationMode->addItem(tr("多点云全局优化（D4PCR + ICP + GTSAM）"));
#else
	_registrationMode->addItem(tr("双点云配准（D4PCR）"));
#endif
	verticalLayout->insertWidget(0, _registrationMode);
	auto* directoryGroup = new QGroupBox(tr("多点云输入目录（PCD / PLY / LAS）"), this);
	auto* directoryLayout = new QHBoxLayout(directoryGroup);
	_pointCloudDirectory = new QLineEdit(directoryGroup);
	_pointCloudDirectory->setObjectName("_pointCloudDirectory");
	auto* selectDirectory = new QPushButton(tr("选择目录"), directoryGroup);
	directoryLayout->addWidget(_pointCloudDirectory);
	directoryLayout->addWidget(selectDirectory);
	verticalLayout->insertWidget(2, directoryGroup);
	directoryGroup->hide();
	connect(selectDirectory, &QPushButton::clicked, this, [this]() {
		const QString dir = QFileDialog::getExistingDirectory(this, tr("选择点云目录"));
		if (!dir.isEmpty())
			_pointCloudDirectory->setText(dir);
	});
	connect(_registrationMode, &QComboBox::currentIndexChanged, this,
		[this, directoryGroup](int mode) {
			groupBox->setVisible(mode == 0);
			directoryGroup->setVisible(mode == 1);
			_isCenter->setEnabled(mode == 0);
			adjustSize();
		});
	connect(_selectInputFileOfSource, &QPushButton::clicked, this, &OutlierRemovalRegistration::selectInputFileOfSource);
	connect(_selectInputFileOfTarget, &QPushButton::clicked, this, &OutlierRemovalRegistration::selectInputFileOfTarget);
	connect(_selectOutputDir, &QPushButton::clicked, this, &OutlierRemovalRegistration::selectOutputDir);
	connect(_action_OK, &QPushButton::clicked, this, &OutlierRemovalRegistration::apply);
	connect(_action_cancel, &QPushButton::clicked, this, &OutlierRemovalRegistration::reject);

	initParam();
}

OutlierRemovalRegistration::~OutlierRemovalRegistration()
{
}

void OutlierRemovalRegistration::selectInputFileOfSource()
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

void OutlierRemovalRegistration::selectInputFileOfTarget()
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

void OutlierRemovalRegistration::selectOutputDir()
{
	QString dir = QFileDialog::getExistingDirectory(this, tr("Select Output Directory"), "");
	if (dir.isEmpty())
		return;
	_outputFileOfDir->setText(dir);
}

bool OutlierRemovalRegistration::executeRegistration(QProgressDialog* progress, QTextEdit* logger,
	std::function<void()> onFinished)
{
	ORRRegParams params;
	params.sourceFile = _inputFileOfSource->text();
	params.targetFile = _inputFileOfTarget->text();
	params.outputDir = _outputFileOfDir->text();
	params.isCenter = _isCenter->isChecked();
	params.resolution_dis = _resolution_dis->text().isEmpty() ? 0.0f : _resolution_dis->text().toFloat();
	params.globalOptimization = _registrationMode->currentIndex() == 1;
	params.pointCloudDirectory = _pointCloudDirectory->text().trimmed();

	if (!std::isfinite(params.resolution_dis) || params.resolution_dis <= 0.0f ||
		params.outputDir.trimmed().isEmpty() ||
		(params.globalOptimization ? (params.pointCloudDirectory.isEmpty() || !QDir(params.pointCloudDirectory).exists()) :
			(!QFileInfo(params.sourceFile).isFile() || !QFileInfo(params.targetFile).isFile())))
	{
		QMessageBox::warning(this, tr("参数错误"),
			tr("请选择有效的点云输入和输出目录，并输入正数距离分辨率。"));
		return false;
	}

	if (!progress)
		return false;
#ifdef D4PCR_HAS_GLOBAL_OPTIMIZATION
	if (params.globalOptimization)
		return startGlobalRegistration(progress, logger, params.pointCloudDirectory,
			params.outputDir, params.resolution_dis, std::move(onFinished));
#endif
	registrationui::start(progress, logger, tr("Building point-cloud correspondences ..."),
		std::move(onFinished), [params](QPointer<QTextEdit> loggerGuard, registrationui::ProgressReporter report) {
			try {
				registration(params, loggerGuard, report);
			} catch (const std::exception& error) {
				logToLoggerORR(loggerGuard, tr("Registration failed: ") + QString::fromLocal8Bit(error.what()) + "\n");
			}
		});
	return true;
}

void OutlierRemovalRegistration::apply()
{
	this->done(QDialog::Accepted); // 等同于 this->accept();
}

void OutlierRemovalRegistration::reject()
{
	this->done(QDialog::Rejected); // 等同于 this->reject();
}

void OutlierRemovalRegistration::initParam()
{
	_resolution_dis->setText("0.1");
}

void OutlierRemovalRegistration::registration(ORRRegParams params, QPointer<QTextEdit> logger,
	std::function<void(int, const QString&)> reportProgress)
{

	if (!QDir().mkpath(params.outputDir))
		throw std::runtime_error("Cannot create output directory.");
	bool isCenter = params.isCenter;
	float resolution = params.resolution_dis;

	std::string sourceFile = QFile::encodeName(params.sourceFile).toStdString();
	std::string targetFile = QFile::encodeName(params.targetFile).toStdString();

	PointCloudPtr sampledCloudS(new pcl::PointCloud<pcl::PointXYZ>);
	PointCloudPtr sampledCloudT(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::Correspondences cors;
	//Real World
	std::shared_ptr<RelationshipConstruction> rc =
		std::make_shared<RelationshipConstruction>(
			sourceFile, targetFile, resolution, isCenter);
	reportProgress(50, tr("Solving robust transformation ..."));

	cors = rc->getCorrespondences();
	sampledCloudS = rc->getPointCloudS();
	sampledCloudT = rc->getPointCloudT();

	Decoupling decoupling(sampledCloudS, sampledCloudT, cors, resolution);
	if (!decoupling.isValid())
		throw std::runtime_error("D4PCR could not estimate a valid transformation.");
	Eigen::Matrix4d transformation_matrix = decoupling.getTransformation();
#ifdef D4PCR_HAS_GLOBAL_OPTIMIZATION
	reportProgress(65, tr("Refining registration with 4-DOF ICP ..."));
	d4pcr::global::PipelineOptions options;
	options.icpMaximumCorrespondenceDistance = 3.0 * resolution;
	const auto refined = d4pcr::global::refineFourDofIcp(
		sampledCloudS, sampledCloudT, transformation_matrix, options);
	if (refined.converged && refined.correspondenceCount >= static_cast<std::size_t>(options.minimumIcpCorrespondences) &&
		refined.overlap >= options.minimumIcpOverlap && std::isfinite(refined.rmse) &&
		refined.rmse <= options.icpMaximumCorrespondenceDistance)
	{
		transformation_matrix = refined.transformation;
		logToLoggerORR(logger, tr("4-DOF ICP completed. RMSE: ") + QString::number(refined.rmse, 'f', 6) + "\n");
	}
	else
		logToLoggerORR(logger, tr("ICP refinement did not pass validation; using the D4PCR transformation.") + "\n");
#endif
	if (isCenter)
		transformation_matrix.block<3, 1>(0, 3) += rc->getCenterT() -
			transformation_matrix.block<3, 3>(0, 0) * rc->getCenterS();
	reportProgress(85, tr("Saving registration results ..."));

	logToLoggerORR(logger, tr("===================================") + "\n");
	logToLoggerORR(logger, tr("Transformation Matrix:") + "\n");

	logToLoggerORR(logger, QString::number(transformation_matrix(0, 0), 'f', 6) + "\t" +
		QString::number(transformation_matrix(0, 1), 'f', 6) + "\t" +
		QString::number(transformation_matrix(0, 2), 'f', 6) + "\t" +
		QString::number(transformation_matrix(0, 3), 'f', 6) + "\n");

	logToLoggerORR(logger, QString::number(transformation_matrix(1, 0), 'f', 6) + "\t" +
		QString::number(transformation_matrix(1, 1), 'f', 6) + "\t" +
		QString::number(transformation_matrix(1, 2), 'f', 6) + "\t" +
		QString::number(transformation_matrix(1, 3), 'f', 6) + "\n");

	logToLoggerORR(logger, QString::number(transformation_matrix(2, 0), 'f', 6) + "\t" +
		QString::number(transformation_matrix(2, 1), 'f', 6) + "\t" +
		QString::number(transformation_matrix(2, 2), 'f', 6) + "\t" +
		QString::number(transformation_matrix(2, 3), 'f', 6) + "\n");

	logToLoggerORR(logger, QString::number(transformation_matrix(3, 0), 'f', 6) + "\t" +
		QString::number(transformation_matrix(3, 1), 'f', 6) + "\t" +
		QString::number(transformation_matrix(3, 2), 'f', 6) + "\t" +
		QString::number(transformation_matrix(3, 3), 'f', 6) + "\n");

	logToLoggerORR(logger, tr("===================================") + "\n");

	std::filesystem::path sourcePath(sourceFile);
	std::filesystem::path targetPath(targetFile);

	std::string outputDir = QFile::encodeName(params.outputDir).toStdString();
	std::ofstream dataOut(outputDir + "/" + sourcePath.stem().string() + "_to_" + targetPath.stem().string() + "_transformationMatrix.txt");

	if (!dataOut)
	{
		logToLoggerORR(logger, tr("Failed to create transformation matrix file!") + "\n");
		return;
	}

	dataOut << std::fixed << std::setprecision(6);
	dataOut << transformation_matrix(0, 0) << "\t" << transformation_matrix(0, 1) << "\t" << transformation_matrix(0, 2) << "\t" << transformation_matrix(0, 3) << "\n";
	dataOut << transformation_matrix(1, 0) << "\t" << transformation_matrix(1, 1) << "\t" << transformation_matrix(1, 2) << "\t" << transformation_matrix(1, 3) << "\n";
	dataOut << transformation_matrix(2, 0) << "\t" << transformation_matrix(2, 1) << "\t" << transformation_matrix(2, 2) << "\t" << transformation_matrix(2, 3) << "\n";
	dataOut << transformation_matrix(3, 0) << "\t" << transformation_matrix(3, 1) << "\t" << transformation_matrix(3, 2) << "\t" << transformation_matrix(3, 3) << "\n";
	dataOut.close();
	if (!dataOut)
		throw std::runtime_error("Cannot write transformation matrix file.");

	logToLoggerORR(logger, tr("Transformed information saved to: ") + QString::fromLocal8Bit((outputDir + "/" + sourcePath.stem().string() + "_to_" + targetPath.stem().string() + "_transformationMatrix.txt").c_str()) + "\n");

	rc->loadPointCloudFile(sourceFile, sampledCloudS);
	if (!sampledCloudS || sampledCloudS->empty())
		throw std::runtime_error("Cannot reload source point cloud for export.");

	
	pcl::transformPointCloud(*sampledCloudS, *sampledCloudT, transformation_matrix);

	std::string outputSource = outputDir + "/" + sourcePath.stem().string() + "_registered.pcd";

	if (pcl::io::savePCDFileBinary(outputSource, *sampledCloudT) < 0)
		throw std::runtime_error("Cannot save registered point cloud.");
	
	logToLoggerORR(logger, tr("Registered point cloud saved to: ") + QString::fromLocal8Bit(outputSource.c_str()) + "\n");
	logToLoggerORR(logger, tr("Point cloud registration completed!") + "\n");
	logToLoggerORR(logger, tr("===================================") + "\n");
}
