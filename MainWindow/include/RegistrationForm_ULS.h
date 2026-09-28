#pragma once
#include "ui_TLS-ULSRegistration.h"
#include <QtWidgets/QDialog>
#include <QFileDialog>
#include <QtWidgets/QProgressDialog>
#include <QtConcurrent/QtConcurrent>
#include <QTextEdit>
#include <QPointer>
#include <functional>
#include <RegistrationU2T.h>
#include <LoadLasFile.h>



class RegistrationULS:public QDialog, public Ui::ULS_TLSRegistration
{
	Q_OBJECT
public:
	explicit RegistrationULS(QWidget* parent = nullptr);
	~RegistrationULS();

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

	void initParam();

	static void registration(ULSRegParams params, QPointer<QTextEdit> logger,
		std::function<void(int, const QString&)> reportProgress);

};
