#pragma once

#include "ui_TLSRegistration.h"
#include <QtWidgets/QDialog>
#include <QFileDialog>
#include <QtWidgets/QProgressDialog>
#include <QtConcurrent/QtConcurrent>
#include <QTextEdit>

#include <filesystem>

#include <Function.h>

class RegistrationForm_TLS :public QDialog, public Ui::TLSRegistration
{
public:
	explicit RegistrationForm_TLS(QWidget* parent = nullptr);
	~RegistrationForm_TLS();

	//ִ��ע��
	void executeRegistration(QProgressDialog* progress, QTextEdit* logger);

private slots:
	void apply();
	void reject();
	//ѡ�������Դ����
	void selectInputFileOfSource();
	//ѡ�������Ŀ�����
	void selectInputFileOfTarget();
	// ѡ�����·��
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

	void registration(TLSRegParams params, QTextEdit* logger);

};