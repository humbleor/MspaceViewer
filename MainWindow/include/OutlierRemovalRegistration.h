#pragma once

#include <filesystem>

#include "ui_OutlierRemovalRegistration.h"
#include <QtWidgets/QDialog>
#include <QFileDialog>
#include <QTextEdit>
#include <QPointer>
#include <QComboBox>
#include <functional>
#include <QtWidgets/QProgressDialog>
#include <QtConcurrent/QtConcurrent>
#include <Preparation.h>
#include <RelationshipConstruction.h>
#include <IntervalStabbing.h>
#include <Decoupling.h>

#include <pcl/io/pcd_io.h>// PCD 文件读写头文件
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

class OutlierRemovalRegistration :public QDialog, public Ui::OutliersRemovalRegistration
{
	Q_OBJECT
public:
	explicit OutlierRemovalRegistration(QWidget* parent = nullptr);
	~OutlierRemovalRegistration();

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

	struct ORRRegParams {
		QString sourceFile;
		QString targetFile;
		QString outputDir;
		bool isCenter;
		float resolution_dis;
		bool globalOptimization{ false };
		QString pointCloudDirectory;
	};

	static void registration(ORRRegParams params, QPointer<QTextEdit> logger,
		std::function<void(int, const QString&)> reportProgress);
	QComboBox* _registrationMode{ nullptr };
	QLineEdit* _pointCloudDirectory{ nullptr };

};
