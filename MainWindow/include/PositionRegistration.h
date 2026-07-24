#pragma once

#include "ui_PositionRegistration.h"
#include <QtWidgets/QDialog>
#include <QFileDialog>
#include <QtWidgets/QProgressDialog>
#include <QtConcurrent/QtConcurrent>
#include <QTextEdit>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <Eigen/Core>

struct PositionRegParams {
	QString sourceFile;
	QString targetFile;
	QString outputDir;
	double  descriptorMinLen  = 2.0;
	double  descriptorMaxLen  = 50.0;
	int     descriptorNearNum = 10;
	double  disGeoVerify      = 0.3;
	double  icpThreshold      = 0.5;
	int     bestPairsCount    = 7;
};

class PositionRegistration : public QDialog, public Ui::PositionRegistration, public std::enable_shared_from_this<PositionRegistration>
{
	Q_OBJECT
public:
	explicit PositionRegistration(QWidget* parent = nullptr);
	~PositionRegistration();

	void executeRegistration(QProgressDialog* progress, QTextEdit* logger);

private slots:
	void apply();
	void reject();
	void selectInputFileOfSource();
	void selectInputFileOfTarget();
	void selectOutputDir();

private:
	void initParam();
	void registration(PositionRegParams params, QTextEdit* logger);

};
