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

#include <pcl/io/pcd_io.h>//pcd ��д����ص�ͷ�ļ���
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

class OutlierRemovalRegistration :public QDialog, public Ui::OutliersRemovalRegistration
{
	Q_OBJECT
public:
	explicit OutlierRemovalRegistration(QWidget* parent = nullptr);
	~OutlierRemovalRegistration();

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

	struct ORRRegParams {
		QString sourceFile;
		QString targetFile;
		QString outputDir;
		bool isCenter;
		float resolution_dis;
	};

	void registration(ORRRegParams params, QTextEdit* logger);

};