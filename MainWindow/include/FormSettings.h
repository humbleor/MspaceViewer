#pragma once

#include <osgViewer/Viewer>
#include <osg/Node>
#include <osgDB/ReadFile>
#include <osgGA/TrackballManipulator>
#include <osgUtil/Optimizer>
#include <osg/MatrixTransform>
#include <osgDB/WriteFile>
#include <QtCore/QObject>
#include <QtWidgets/QVBoxLayout>

#include <osg/NodeVisitor>
#include <osg/Geometry>

#include <QtCore/QCoreApplication>
#include <QtCore/QDir>


#include "../../osgQt/include/osgQOpenGLWidget.h"
#include "../../MSpaceOctree/include/MSpaceOCTree.h"

class FormSettings  : public QObject
{
	Q_OBJECT

public:
	FormSettings(QVBoxLayout* layout, QWidget* parent = nullptr);
	~FormSettings();

    //设置背景色
    const osg::Vec4& getBackGroundColor();
    void setBackGroundColor(osg::Vec4 backGroundColor);
    //加载文件
    void loadFiles(std::vector<std::string> inputFiles, std::string outputFile, osg::ref_ptr<osg::MSpaceNode> inputFileNode);
    //显示树木提取评价结果点图
    //void showMatchingNode(osg::ref_ptr<osg::Group> inputNode, std::vector<string> IDOfExt, std::vector<std::vector<double>> postionOfExt,std::vector<string> IDOfRef, std::vector<std::vector   <double>> postionOfRef, std::vector<std::pair<size_t, size_t>> trueMatching, std::vector<size_t> resOfExt, std::vector<size_t> resOFRef);
    //删除节点
    void deletePointCloudNode(osg::ref_ptr<osg::MSpaceNode> Node, string outputDir);
    ////删除匹配树节点
    //void deeteTreeNode(osg::ref_ptr<osg::Group> Node);
    //获取加载节点工作是否完成
    bool isfinished() { return _finished; };
    void initWorkProgress() { _finished = false; };

protected slots:
    //调用osgQT初始化osg窗口
    void initVisualizationWindow();

private:
    //osgQT中的主要类
    std::shared_ptr<osgQOpenGLWidget> _osgQOpenGLWidget;
    //osg窗口视角类
    osg::ref_ptr<osgViewer::Viewer> _viewer;
    //根目录
    osg::ref_ptr<osg::Group> _root = new osg::Group();

    bool _finished = false;

};

class ColorVisitor : public osg::NodeVisitor
{
public:
    ColorVisitor(osg::Vec4 color) : osg::NodeVisitor(TRAVERSE_ALL_CHILDREN) { _color = color; }

    virtual void apply(osg::Geode& geode)
    {
        for (unsigned int i = 0; i < geode.getNumDrawables(); ++i)
        {
            osg::Geometry* geometry = dynamic_cast<osg::Geometry*>(geode.getDrawable(i));
            if (geometry)
            {
                osg::Vec4Array* colors = new osg::Vec4Array;
                colors->push_back(_color); // 设置为红色

                geometry->setColorArray(colors);
                geometry->setColorBinding(osg::Geometry::BIND_OVERALL);
            }
        }

        traverse(geode);
    }

private:
    osg::Vec4 _color;
};
