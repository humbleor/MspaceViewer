#include "../include/OutlierRemovalRegistration.h"

static void logToLoggerORR(QTextEdit* logger, const QString& text)
{
	QMetaObject::invokeMethod(logger, [logger, text]() {
		logger->insertPlainText(text);
	}, Qt::QueuedConnection);
}

OutlierRemovalRegistration::OutlierRemovalRegistration(QWidget* parent)
	:QDialog(parent),
	Ui::OutliersRemovalRegistration()
{
	this->setupUi(this);
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
	fileTypes << "PCD Files (*.pcd)";
	QString file = QFileDialog::getOpenFileName(this, tr("Select Source File"), "", fileTypes.join(";;"));
	if (file.isEmpty())
		return;
	_inputFileOfSource->setText(file);
}

void OutlierRemovalRegistration::selectInputFileOfTarget()
{
	QStringList fileTypes;
	fileTypes << "PCD Files (*.pcd)";
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

struct ORRRegParams {
	QString sourceFile;
	QString targetFile;
	QString outputDir;
	bool isCenter;
	float resolution_dis;
};

void OutlierRemovalRegistration::executeRegistration(QProgressDialog* progress, QTextEdit* logger)
{
	ORRRegParams params;
	params.sourceFile = _inputFileOfSource->text();
	params.targetFile = _inputFileOfTarget->text();
	params.outputDir = _outputFileOfDir->text();
	params.isCenter = _isCenter->isChecked();
	params.resolution_dis = _resolution_dis->text().isEmpty() ? 0.0f : _resolution_dis->text().toFloat();

	if (params.resolution_dis == 0.0f)
		return;

	if (progress)
	{
		progress->setLabelText(tr("Performing point cloud registration ..."));
		progress->setWindowFlags(progress->windowFlags() & ~Qt::WindowCloseButtonHint & ~Qt::WindowContextHelpButtonHint);
		progress->setCancelButton(nullptr);
		progress->show();
		progress->raise();
	}
	QFuture<void> future = QtConcurrent::run(std::bind(&OutlierRemovalRegistration::registration, this, params, logger));
	while (!future.isFinished())
	{
		if (progress)
		{
			progress->setValue(progress->value() + 1);
			QApplication::processEvents();
		}
	}
}

void OutlierRemovalRegistration::apply()
{
	this->done(QDialog::Accepted); // ����ֱ�� this->accept();
}

void OutlierRemovalRegistration::reject()
{
	this->done(QDialog::Rejected); // ����ֱ�� this->reject();
}

void OutlierRemovalRegistration::initParam()
{
	_resolution_dis->setText("0.1");
}

void OutlierRemovalRegistration::registration(ORRRegParams params, QTextEdit* logger)
{
	bool isCenter = params.isCenter;
	float resolution = params.resolution_dis;

	std::string sourceFile = params.sourceFile.toStdString();
	std::string targetFile = params.targetFile.toStdString();

	PointCloudPtr sampledCloudS(new pcl::PointCloud<pcl::PointXYZ>);
	PointCloudPtr sampledCloudT(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::Correspondences cors;
	//Real World
	std::shared_ptr<RelationshipConstruction> rc =
		std::make_shared<RelationshipConstruction>(
			sourceFile, targetFile, resolution, isCenter);

	cors = rc->getCorrespondences();
	sampledCloudS = rc->getPointCloudS();
	sampledCloudT = rc->getPointCloudT();

	Decoupling decoupling(sampledCloudS, sampledCloudT, cors, resolution);
	Eigen::Matrix4d transformation_matrix = decoupling.getTransformation();

	logToLoggerORR(logger, tr("Point cloud registration completed!") + "\n");
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

	std::filesystem::path sourcePath(params.sourceFile.toStdString());
	std::filesystem::path targetPath(params.targetFile.toStdString());

	std::string outputDir = params.outputDir.toStdString();
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

	logToLoggerORR(logger, tr("Transformed information saved to: ") + QString::fromStdString(outputDir + "/" + sourcePath.stem().string() + "_to_" + targetPath.stem().string() + "_transformationMatrix.txt") + "\n");

	pcl::io::loadPCDFile(sourceFile, *sampledCloudS);

	
	pcl::transformPointCloud(*sampledCloudS, *sampledCloudT, transformation_matrix);

	std::string outputSource = outputDir + "/" + sourcePath.stem().string() + "_registered.pcd";

	pcl::io::savePCDFileBinary(outputSource, *sampledCloudT);
	
	logToLoggerORR(logger, tr("Registered point cloud saved to: ") + QString::fromStdString(outputSource) + "\n");
	logToLoggerORR(logger, tr("===================================") + "\n");
}