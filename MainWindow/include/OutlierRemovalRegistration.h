#pragma once

#include <filesystem>

#include "ui_OutlierRemovalRegistration.h"
#include <QtWidgets/QDialog>
#include <QFileDialog>
#include <QTextEdit>
#include <QtWidgets/QProgressDialog>
#include <QtConcurrent/QtConcurrent>
#include <Preparation.h>
#include <RelationshipConstruction.h>
#include <IntervalStabbing.h>
#include <Decoupling.h>

#include <pcl/io/pcd_io.h>//pcd 读写类相关的头文件。
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

class OutlierRemovalRegistration :public QDialog, public Ui::OutliersRemovalRegistration
{
	Q_OBJECT
public:
	explicit OutlierRemovalRegistration(QWidget* parent = nullptr);
	~OutlierRemovalRegistration();

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