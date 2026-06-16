#include "../include/RegistrationForm_ULS.h"

static void logToLoggerULS(QTextEdit* logger, const QString& text)
{
	QMetaObject::invokeMethod(logger, [logger, text]() {
		logger->insertPlainText(text);
	}, Qt::QueuedConnection);
}

RegistrationULS::RegistrationULS(QWidget* parent)
	:QDialog(parent),
	Ui::ULS_TLSRegistration()
{
	this->setupUi(this);

	connect(_selectInputFileOfSource, &QPushButton::clicked, this, &RegistrationULS::selectInputFileOfSource);
	connect(_selectInputFileOfTarget, &QPushButton::clicked, this, &RegistrationULS::selectInputFileOfTarget);
	connect(_selectOutputDir, &QPushButton::clicked, this, &RegistrationULS::selectOutputDir);
	connect(action_OK, &QPushButton::clicked, this, &RegistrationULS::apply);
	connect(action_cancel, &QPushButton::clicked, this, &RegistrationULS::reject);

	initParam();
}

RegistrationULS::~RegistrationULS()
{
}

void RegistrationULS::reject()
{
	this->done(QDialog::Rejected); // ����ֱ�� this->reject();
}

void RegistrationULS::selectInputFileOfSource()
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

void RegistrationULS::selectInputFileOfTarget()
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

void RegistrationULS::selectOutputDir()
{
	QString dir = QFileDialog::getExistingDirectory(this, tr("Select Output Directory"), "");
	if (dir.isEmpty())
		return;
	_outputFileOfDir->setText(dir);
}

struct ULSRegParams {
	QString sourceFile;
	QString targetFile;
	QString outputDir;
	float resolution;
	float gridStep;
	float searchRadius;
	float radiusStep;
	int numSectors;
	float angleThe;
	float a2DThe;
	float a3DThe;
};

void RegistrationULS::executeRegistration(QProgressDialog* progress, QTextEdit* logger)
{
	if (_resolution->text().isEmpty() ||
		_gridStep->text().isEmpty() ||
		_searchRadius->text().isEmpty() ||
		_radiusStep->text().isEmpty() ||
		_numSectors->text().isEmpty() ||
		_angleThe->text().isEmpty() ||
		_a2DThe->text().isEmpty() ||
		_a3DThe->text().isEmpty())
		return;

	ULSRegParams params;
	params.sourceFile = _inputFileOfSource->text();
	params.targetFile = _inputFileOfTarget->text();
	params.outputDir = _outputFileOfDir->text();
	params.resolution = _resolution->text().toFloat();
	params.gridStep = _gridStep->text().toFloat();
	params.searchRadius = _searchRadius->text().toFloat();
	params.radiusStep = _radiusStep->text().toFloat();
	params.numSectors = _numSectors->text().toInt();
	params.angleThe = _angleThe->text().toFloat();
	params.a2DThe = _a2DThe->text().toFloat();
	params.a3DThe = _a3DThe->text().toFloat();

	if (progress)
	{
		progress->setLabelText(tr("Performing point cloud registration ..."));
		progress->setWindowFlags(progress->windowFlags() & ~Qt::WindowCloseButtonHint & ~Qt::WindowContextHelpButtonHint);
		progress->setCancelButton(nullptr);
		progress->show();
		progress->raise();
	}
	QFuture<void> future = QtConcurrent::run(std::bind(&RegistrationULS::registration, this, params, logger));
	while (!future.isFinished())
	{
		if (progress)
		{
			progress->setValue(progress->value() + 1);
			QApplication::processEvents();
		}
	}
}

void RegistrationULS::initParam()
{
	_resolution->setText("0.2");
	_gridStep->setText("0.5");
	_searchRadius->setText("5.0");
	_radiusStep->setText("0.5");
	_numSectors->setText("360");
	_angleThe->setText("2.0");
	_a2DThe->setText("0.1");
	_a3DThe->setText("0.1");
}

void RegistrationULS::registration(ULSRegParams params, QTextEdit* logger)
{
	float resolution = params.resolution;
	float gridStep = params.gridStep;
	float searchRadius = params.searchRadius;
	float radiusStep = params.radiusStep;
	size_t numSectors = params.numSectors;
	float angleThe = params.angleThe;
	float a2DThe = params.a2DThe;
	float a3DThe = params.a3DThe;

	PointCloud3fPtr uav = std::make_shared<PointCloud3f>();
	PointCloud3fPtr tls = std::make_shared<PointCloud3f>();
	loadLasFile(params.sourceFile.toStdString(), uav);
	loadLasFile(params.targetFile.toStdString(), tls);

	std::shared_ptr<RegistrationU2T> u2t = std::make_shared<RegistrationU2T>(uav, tls);
	u2t->setGridFilterRes(resolution);
	u2t->gridStep(gridStep);
	u2t->setSearchRadius(searchRadius);
	u2t->setRadiusStep(radiusStep);
	u2t->setNumSectors(numSectors);
	u2t->descriptorsThreshold(angleThe, a2DThe, a3DThe);
	u2t->registration();
	std::array<std::array<float, 4>, 4> transMatrix = u2t->getTranslationMatrix();

	logToLoggerULS(logger, tr("Point cloud registration completed!") + "\n");
	logToLoggerULS(logger, tr("===================================") + "\n");
	logToLoggerULS(logger, tr("Transformation Matrix:") + "\n");
	logToLoggerULS(logger, QString::number(transMatrix[0][0], 'f', 6) + "\t" + QString::number(transMatrix[0][1], 'f', 6) + "\t" + QString::number(transMatrix[0][2], 'f', 6) + "\t" + QString::number(transMatrix[0][3], 'f', 6) + "\n");
	logToLoggerULS(logger, QString::number(transMatrix[1][0], 'f', 6) + "\t" + QString::number(transMatrix[1][1], 'f', 6) + "\t" + QString::number(transMatrix[1][2], 'f', 6) + "\t" + QString::number(transMatrix[1][3], 'f', 6) + "\n");
	logToLoggerULS(logger, QString::number(transMatrix[2][0], 'f', 6) + "\t" + QString::number(transMatrix[2][1], 'f', 6) + "\t" + QString::number(transMatrix[2][2], 'f', 6) + "\t" + QString::number(transMatrix[2][3], 'f', 6) + "\n");
	logToLoggerULS(logger, QString::number(transMatrix[3][0], 'f', 6) + "\t" + QString::number(transMatrix[3][1], 'f', 6) + "\t" + QString::number(transMatrix[3][2], 'f', 6) + "\t" + QString::number(transMatrix[3][3], 'f', 6) + "\n");
	logToLoggerULS(logger, tr("===================================") + "\n");

	std::filesystem::path inputPath_uav(params.sourceFile.toStdString());
	std::filesystem::path inputPath_tls(params.targetFile.toStdString());



	std::string outputDir = params.outputDir.toStdString();
	std::ofstream dataOut(outputDir + "/" + inputPath_uav.stem().string() + "_to_" + inputPath_tls.stem().string() + "_transformationMatrix.txt");
	dataOut << std::fixed << std::setprecision(6);
	dataOut << transMatrix[0][0] << " " << transMatrix[0][1] << " " << transMatrix[0][2] << " " << transMatrix[0][3] << std::endl;
	dataOut << transMatrix[1][0] << " " << transMatrix[1][1] << " " << transMatrix[1][2] << " " << transMatrix[1][3] << std::endl;
	dataOut << transMatrix[2][0] << " " << transMatrix[2][1] << " " << transMatrix[2][2] << " " << transMatrix[2][3] << std::endl;
	dataOut << transMatrix[3][0] << " " << transMatrix[3][1] << " " << transMatrix[3][2] << " " << transMatrix[3][3] << std::endl;
	dataOut.close();

	logToLoggerULS(logger, tr("Transformed information saved to: ") + QString::fromStdString(outputDir + "/" + inputPath_uav.stem().string() + "_to_" + inputPath_tls.stem().string() + "_transformationMatrix.txt") + "\n");

	std::array<std::array<float, 3>, 3> rotMatrix;
	rotMatrix[0][0] = transMatrix[0][0]; rotMatrix[0][1] = transMatrix[0][1]; rotMatrix[0][2] = transMatrix[0][2];
	rotMatrix[1][0] = transMatrix[1][0]; rotMatrix[1][1] = transMatrix[1][1]; rotMatrix[1][2] = transMatrix[1][2];
	rotMatrix[2][0] = transMatrix[2][0]; rotMatrix[2][1] = transMatrix[2][1]; rotMatrix[2][2] = transMatrix[2][2];
	tls->rotate(rotMatrix);

	Point3f translation(std::array<float, 3>{transMatrix[0][3], transMatrix[1][3], transMatrix[2][3]});
	tls->translate(translation);

	outputLasFile(outputDir + "/" + inputPath_tls.stem().string() + "_registered.las", tls);

	logToLoggerULS(logger, tr("Registered point cloud saved to: ") + QString::fromStdString(outputDir + "/" + inputPath_tls.stem().string() + "_registered.las") + "\n");

	logToLoggerULS(logger, tr("===================================") + "\n");
}

void RegistrationULS::apply()
{
	this->accept();
}