#pragma once

#include <QtWidgets/QMainWindow>
#include <QtGui/QStandardItemModel>
#include <QtCore/QTranslator>
#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include "ui_MainWindow.h"
#include "FormSettings.h"
#include "RegistrationForm_ULS.h"
#include "OutlierRemovalRegistration.h"
#include "RegistrationForm_TLS.h"
#include "ForestRegistration.h"
//#include "TreeEvaluation.h"
//#include "StemCurveEvaluation.h"
//#include "../../TreeEvaluation/include/3DMatching.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();


private:
    void InitForm();
    void InitTree();
    void InitLanguage();
    void InitDataMangementMenu();
    //加载和显示数据
    void LoadAndShowFiles(std::string inputFiles);
    //隐藏数据
    void HiddenData(QStandardItem* item);
    //在左侧数据管理窗口中选中数据并右键可以出来删除数据的选项
    void handleContextMenuRequested(const QPoint& pos);
    //右键出来删除数据选项后删除数据
    void DeleteData();
    //用于测试的按钮
    void loadFile();
    //地空点云数据处理
    void registration_TLS_ULS();
    //离群值剔除点云配准
	void outlierRemovalRegistration();
    //TLS点云配准
	void registration_TLS();
    //森林点云配准
    void registration_Forest();
    //更新属性面板
    void updatePropertyPanel(const std::string& fileName, osg::ref_ptr<osg::MSpaceNode> node);
    //转化到中文
    void changeLanguage_Chinese();
    //转化到英文
    void changeLanguage_English();

    ////树木提取参数评价
    //void treeEvaluation();
    ////干曲线参数评价
    //void stemCurveEvaluation();
    ////显示树木评价的输出参数的表格（在右上方显示）
    //void showTreeInputInformation(std::shared_ptr<TreeEvaluation> treeEvaluation, std::vector<std::pair<size_t, size_t>> matchingIndex, std::vector<size_t> unMatchingIndexOfExt, std::vector<size_t> unMatchingIndexOfRef);

private:
    Ui::MainWindowClass ui;
    std::shared_ptr<FormSettings> _formSettings;
    std::shared_ptr<QStandardItemModel> _rootNode;
    //osg点云和窗口左侧数据管理树之间的对应关系
    std::map<QStandardItem*, osg::ref_ptr<osg::MSpaceNode>> _itemToPointCloud;
    //osg中的匹配模型和窗口左侧数据管理树之间的对应关系
    //std::map<QStandardItem*, osg::ref_ptr<osg::Group>> _itemToTreeModel;
    //在左侧数据管理窗口中选中数据并右键出来的菜单
    std::shared_ptr<QMenu> _dataMangementMenu;

    ////树木评价函数
    //std::shared_ptr<Matching3D> _matching3D;

    //语言控制
    QTranslator chinese;
    QTranslator english;

    bool output_English = false;
};
