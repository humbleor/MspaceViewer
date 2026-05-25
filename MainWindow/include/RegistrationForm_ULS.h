#pragma once
#include "ui_TLS-ULSRegistration.h"
#include <QtWidgets/QDialog>
#include <QFileDialog>
#include <QtWidgets/QProgressDialog>
#include <QtConcurrent/QtConcurrent>
#include <QTextEdit>
#include <RegistrationU2T.h>
#include <LoadLasFile.h>



class RegistrationULS:public QDialog, public Ui::ULS_TLSRegistration
{
	Q_OBJECT
public:
	explicit RegistrationULS(QWidget* parent = nullptr);
	~RegistrationULS();

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

	void registration(ULSRegParams params, QTextEdit* logger);

};