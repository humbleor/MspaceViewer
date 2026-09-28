#pragma once

#include "ui_TLSRegistration.h"
#include <QtWidgets/QDialog>
#include <QFileDialog>
#include <QtWidgets/QProgressDialog>
#include <QtConcurrent/QtConcurrent>
#include <QTextEdit>
#include <QPointer>

#include <filesystem>
#include <functional>

#include <Function.h>

class RegistrationForm_TLS :public QDialog, public Ui::TLSRegistration
{
public:
	explicit RegistrationForm_TLS(QWidget* parent = nullptr);
	~RegistrationForm_TLS();

	// 执行配准
	bool executeRegistration(QProgressDialog* progress, QTextEdit* logger, std::function<void()> onFinished);

private slots:
	void apply();
	void reject();
	// 选择源点云文件
	void selectInputFileOfSource();
	// 选择目标点云文件
	void selectInputFileOfTarget();
	// 选择输出目录
	void selectOutputDir();

private:
	void initParam();

	struct TLSRegParams {
		QString sourceFile;
		QString targetFile;
		QString outputDir;
		int sector_num;
		int pointsConstrain;
		float resolution_Radius;
		float maxRadius;
		float minRadius;
		float error_dis;
		float error_z;
		float error_ang;
		float zConstrain;
	};

	static void registration(TLSRegParams params, QPointer<QTextEdit> logger,
		std::function<void(int, const QString&)> reportProgress);

};
