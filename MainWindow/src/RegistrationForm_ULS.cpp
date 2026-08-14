#include "../include/RegistrationForm_ULS.h"
#include "../include/RunFuture.h"

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
	runFutureBlocking(future);
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
	u2t->setGridStep(gridStep);
	u2t->setSearchRadius(searchRadius);
	u2t->setRadiusStep(radiusStep);
	u2t->setNumSectors(numSectors);
	u2t->descriptorsThreshold(angleThe, a2DThe, a3DThe);
	u2t->registration();
	std::array<std::array<float, 4>, 4> coarseMatrix = u2t->getCoarseMatrix();
	std::array<std::array<float, 4>, 4> icpMatrix = u2t->getIcpMatrix();
	std::array<std::array<float, 4>, 4> totalMatrix = u2t->getTotalMatrix();

	logToLoggerULS(logger, tr("Point cloud registration completed!") + "\n");
	logToLoggerULS(logger, tr("===================================") + "\n");

	// 输出粗配准矩阵
	logToLoggerULS(logger, tr("Coarse Registration Matrix:") + "\n");
	for (int i = 0; i < 4; i++)
		logToLoggerULS(logger, QString::number(coarseMatrix[i][0], 'f', 6) + "\t" + QString::number(coarseMatrix[i][1], 'f', 6) + "\t" + QString::number(coarseMatrix[i][2], 'f', 6) + "\t" + QString::number(coarseMatrix[i][3], 'f', 6) + "\n");
	logToLoggerULS(logger, tr("===================================") + "\n");

	// 输出 ICP 矩阵
	logToLoggerULS(logger, tr("ICP Transformation Matrix:") + "\n");
	for (int i = 0; i < 4; i++)
		logToLoggerULS(logger, QString::number(icpMatrix[i][0], 'f', 6) + "\t" + QString::number(icpMatrix[i][1], 'f', 6) + "\t" + QString::number(icpMatrix[i][2], 'f', 6) + "\t" + QString::number(icpMatrix[i][3], 'f', 6) + "\n");
	logToLoggerULS(logger, tr("===================================") + "\n");

	// 输出总配准矩阵
	logToLoggerULS(logger, tr("Total Transformation Matrix:") + "\n");
	for (int i = 0; i < 4; i++)
		logToLoggerULS(logger, QString::number(totalMatrix[i][0], 'f', 6) + "\t" + QString::number(totalMatrix[i][1], 'f', 6) + "\t" + QString::number(totalMatrix[i][2], 'f', 6) + "\t" + QString::number(totalMatrix[i][3], 'f', 6) + "\n");
	logToLoggerULS(logger, tr("===================================") + "\n");

	std::filesystem::path inputPath_uav(params.sourceFile.toStdString());
	std::filesystem::path inputPath_tls(params.targetFile.toStdString());

	std::string outputDir = params.outputDir.toStdString();
	std::ofstream dataOut(outputDir + "/" + inputPath_uav.stem().string() + "_to_" + inputPath_tls.stem().string() + "_transformationMatrix.txt");
	dataOut << std::fixed << std::setprecision(6);

	dataOut << "# Coarse Registration Matrix" << std::endl;
	for (int i = 0; i < 4; i++)
		dataOut << coarseMatrix[i][0] << " " << coarseMatrix[i][1] << " " << coarseMatrix[i][2] << " " << coarseMatrix[i][3] << std::endl;

	dataOut << "# ICP Transformation Matrix" << std::endl;
	for (int i = 0; i < 4; i++)
		dataOut << icpMatrix[i][0] << " " << icpMatrix[i][1] << " " << icpMatrix[i][2] << " " << icpMatrix[i][3] << std::endl;

	dataOut << "# Total Transformation Matrix" << std::endl;
	for (int i = 0; i < 4; i++)
		dataOut << totalMatrix[i][0] << " " << totalMatrix[i][1] << " " << totalMatrix[i][2] << " " << totalMatrix[i][3] << std::endl;

	dataOut.close();

	logToLoggerULS(logger, tr("Transform matrix saved to: ") + QString::fromStdString(outputDir + "/" + inputPath_uav.stem().string() + "_to_" + inputPath_tls.stem().string() + "_transformationMatrix.txt") + "\n");
	logToLoggerULS(logger, tr("===================================") + "\n");

	// Apply transformation to TLS point cloud and save
	std::array<std::array<float, 3>, 3> rotMatrix;
	rotMatrix[0][0] = totalMatrix[0][0]; rotMatrix[0][1] = totalMatrix[0][1]; rotMatrix[0][2] = totalMatrix[0][2];
	rotMatrix[1][0] = totalMatrix[1][0]; rotMatrix[1][1] = totalMatrix[1][1]; rotMatrix[1][2] = totalMatrix[1][2];
	rotMatrix[2][0] = totalMatrix[2][0]; rotMatrix[2][1] = totalMatrix[2][1]; rotMatrix[2][2] = totalMatrix[2][2];
	tls->rotate(rotMatrix);

	Point3f translation(std::array<float, 3>{totalMatrix[0][3], totalMatrix[1][3], totalMatrix[2][3]});
	tls->translate(translation);

	outputLasFile(outputDir + "/" + inputPath_tls.stem().string() + "_registered.las", tls);

	logToLoggerULS(logger, tr("Registered point cloud saved to: ") + QString::fromStdString(outputDir + "/" + inputPath_tls.stem().string() + "_registered.las") + "\n");
}

void RegistrationULS::apply()
{
	this->accept();
}