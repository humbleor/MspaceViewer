#pragma once

#include "ui_ForestRegistration.h"
#include <QtWidgets/QDialog>
#include <QFileDialog>
#include <QtWidgets/QProgressDialog>
#include <QtConcurrent/QtConcurrent>
#include <QTextEdit>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <Eigen/Core>

struct ForestRegParams {
	QString sourceFile;
	QString targetFile;
	QString configFile;
	QString outputDir;
};

class ForestRegistration :public QDialog, public Ui::ForestRegistration, public std::enable_shared_from_this<ForestRegistration>
{
	Q_OBJECT
public:
	explicit ForestRegistration(QWidget* parent = nullptr);
	~ForestRegistration();

	void executeRegistration(QProgressDialog* progress, QTextEdit* logger);

private slots:
	void apply();
	void reject();
	void selectInputFileOfSource();
	void selectInputFileOfTarget();
	void selectConfigFile();
	void selectOutputDir();

private:
	void initParam();
	void registration(ForestRegParams params, QTextEdit* logger);

};
