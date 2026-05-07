#include "../include/StemCurveEvaluation.h"

#include <QtWidgets/QProgressDialog>
#include <QtConcurrent/QtConcurrent>
#include <QProgressBar>
#include <QFileDialog>
#include <QHeaderView>
#include <QMainWindow>
#include <QToolTip>

StemCurveEvaluation::StemCurveEvaluation(QWidget* parent)
	:QDialog(parent, Qt::Tool),
	Ui::StemCurveEvaluation()
{
	this->setupUi(this);
	this->setWindowFlags(windowFlags() & ~Qt::WindowContextHelpButtonHint);

	connect(_selectInputFileOfExtractedTree, &QPushButton::clicked, this, &StemCurveEvaluation::selectedInputFileOfExtractedTree);
	connect(_selectedInputFileOfReferenceTree, &QPushButton::clicked, this, &StemCurveEvaluation::selectedInputFileOfReferenceTree);
	connect(_selectedInputFileOfIndex, &QPushButton::clicked, this, &StemCurveEvaluation::selectedInputFileOfIndex);
	connect(_selectedOututFile, &QPushButton::clicked, this, &StemCurveEvaluation::selectedOutputFile);
	connect(_btn_OK, &QPushButton::clicked, this, &StemCurveEvaluation::apply);
}

StemCurveEvaluation::~StemCurveEvaluation()
{
}

void StemCurveEvaluation::apply()
{
	this->accept();
}

void StemCurveEvaluation::process(QProgressDialog* progress, std::shared_ptr<Matching3D>& _matching3D, bool output_English)
{
	if (progress)
	{
		progress->setLabelText(tr("提取树结果评价……"));
		progress->setWindowFlags(progress->windowFlags() & ~Qt::WindowCloseButtonHint & ~Qt::WindowContextHelpButtonHint);
		progress->setCancelButton(nullptr);
		progress->show();
		progress->raise();
	}
	QFuture<void> future = QtConcurrent::run(std::bind(&StemCurveEvaluation::processingData, this, std::ref(_matching3D), output_English));
	while (!future.isFinished())
	{
		if (progress)
		{
			progress->setValue(progress->value() + 1);
			QApplication::processEvents();
		}
	}
}

void StemCurveEvaluation::selectedInputFileOfExtractedTree()
{
	QStringList fileTypes;
	fileTypes << "Text Files (*.txt)";
	QString file = QFileDialog::getOpenFileName(this, tr("选择提取树参数文件"), "", fileTypes.join(";;"));
	if (file.isEmpty())
		return;
	_inputFileOfExtractedTree->setText(file);
}

void StemCurveEvaluation::selectedInputFileOfReferenceTree()
{
	QStringList fileTypes;
	fileTypes << "Text Files (*.txt)";
	QString file = QFileDialog::getOpenFileName(this, tr("选择参考树参数文件"), "", fileTypes.join(";;"));
	if (file.isEmpty())
		return;
	_inputFileOfReferenceTree->setText(file);
}

void StemCurveEvaluation::selectedInputFileOfIndex()
{
	QStringList fileTypes;
	fileTypes << "Text Files (*.txt)";
	QString file = QFileDialog::getOpenFileName(this, tr("选择匹配索引文件"), "", fileTypes.join(";;"));
	if (file.isEmpty())
		return;
	_inputFileOfIndex->setText(file);
}

void StemCurveEvaluation::selectedOutputFile()
{
	QStringList fileTypes;
	fileTypes << "Excel Files (*.xlsx)";
	QString file = QFileDialog::getOpenFileName(this, tr("选择输出文件"), "", fileTypes.join(";;"));
	if (file.isEmpty())
		return;
	_outputFile->setText(file);
}

void StemCurveEvaluation::processingData(std::shared_ptr<Matching3D>& _matching3D, bool output_English)
{
	double height = 1.3, radius = 1.0;
	if (_inputFileOfIndex->text().isEmpty())
	{
		if (_inputHeight->text().isEmpty() || _radius->text().isEmpty())
		{
			return;
		}
		else
		{
			height = QString(_inputHeight->text()).toDouble();
			radius = QString(_radius->text()).toDouble();
		}
	}

	if (!_matching3D)
	{
		_matching3D = std::make_shared<Matching3D>();
		_matching3D->setDimensionOfExt(2);
		_matching3D->setColumnOfMF(0);
		_matching3D->setDimensionOfRef(2);
		_matching3D->setRadius(radius);
	}
	else
	{
		_matching3D->setDimensionOfExt(2);
		_matching3D->setDimensionOfExt(2);
		_matching3D->setColumnOfMF(0);
		_matching3D->setDimensionOfRef(2);
		_matching3D->setRadius(radius);
	}


	_fileOfExtractedTree = QString(_inputFileOfExtractedTree->text()).toLocal8Bit();
	_fileOfReferenceTree = QString(_inputFileOfReferenceTree->text()).toLocal8Bit();
	QByteArray cdata = _outputFile->text().toLocal8Bit();
	_outputPath = std::string(cdata);
	if (!_matching3D->loadStemCurveFile(_fileOfExtractedTree, _fileOfReferenceTree))
		return;
	if (!_matching3D->bestMatchingALL_StemCurve(std::string(_inputFileOfIndex->text().toLocal8Bit()), height))
		return;
	_matching3D->statisticalResults_StemCurve(std::string(_inputFileOfIndex->text().toLocal8Bit()), _outputPath, output_English);
	//_resOfExt = _matching3D->getResOfExt().size();
	//_resOfRef = _matching3D->getResOfRef().size();
}

