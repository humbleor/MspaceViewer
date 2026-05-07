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
	//选择输入的源点云
	void selectInputFileOfSource();
	//选择输入的目标点云
	void selectInputFileOfTarget();
	// 选择输出路径
	void selectOutputDir();


private:

	void initParam();

	void registration(QTextEdit* logger);

};