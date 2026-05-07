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

	//执行注册
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