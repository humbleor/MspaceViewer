#pragma once

#include "ui_ForestRegistration.h"
#include <QtWidgets/QDialog>
#include <QFileDialog>
#include <QtWidgets/QProgressDialog>
#include <QtConcurrent/QtConcurrent>
#include <QTextEdit>
#include <QPointer>
#include <functional>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <Eigen/Core>
class QComboBox;
class QLineEdit;
class QDoubleSpinBox;

struct ForestRegParams {
	QString sourceFile;
	QString targetFile;
	QString outputDir;
	QString pointCloudDirectory;
	bool globalOptimization = false;
	double mergeResolution = 0.05;
	double descriptorNearNum = 10.0;
	double descriptorMinLen = 2.0;
	double descriptorMaxLen = 50.0;
	double disGeoVerify = 0.3;
	double icpThreshold = 0.25;
	double normalGeoVerify = 1.0;
};

class ForestRegistration :public QDialog, public Ui::ForestRegistration
{
	Q_OBJECT
public:
	explicit ForestRegistration(QWidget* parent = nullptr);
	~ForestRegistration();

	bool executeRegistration(QProgressDialog* progress, QTextEdit* logger, std::function<void()> onFinished);

private slots:
	void apply();
	void reject();
	void selectInputFileOfSource();
	void selectInputFileOfTarget();
	void selectOutputDir();

private:
	QComboBox* _registrationMode = nullptr;
	QLineEdit* _pointCloudDirectory = nullptr;
	QDoubleSpinBox* _mergeResolution = nullptr;
	void initParam();
	static void registration(ForestRegParams params, QPointer<QTextEdit> logger,
		std::function<void(int, const QString&)> reportProgress);

};
