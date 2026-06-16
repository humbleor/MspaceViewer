#include "../include/FormSettings.h"
#include <filesystem>
#include <osg/ShapeDrawable>
#include <osgViewer/ViewerEventHandlers>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers> //事件监听
#include <osgGA/StateSetManipulator> //事件响应类，对渲染状态进行控制

#include <osgText/Text>
#include <osg/AutoTransform>
#include <osg/Material>
#include <osg/LineWidth>
#include <osg/Point>
#include <osg/MatrixTransform>


FormSettings::FormSettings(QVBoxLayout* layout, QWidget* parent)
{
    _osgQOpenGLWidget = std::make_shared<osgQOpenGLWidget>(parent);
    connect(_osgQOpenGLWidget.get(), &osgQOpenGLWidget::initialized, this, &FormSettings::initVisualizationWindow);
    layout->addWidget(_osgQOpenGLWidget.get());
}

FormSettings::~FormSettings()
{
    std::filesystem::remove_all(std::filesystem::path(QCoreApplication::applicationDirPath().toStdString()) / "tmp");
}

const osg::Vec4& FormSettings::getBackGroundColor()
{
    static const osg::Vec4 defaultColor(0.9412f, 0.9412f, 0.9412f, 1.0f);
    if (_viewer == nullptr)
        return defaultColor;
    return _viewer->getCamera()->getClearColor();
}

void FormSettings::setBackGroundColor(osg::Vec4 backGroundColor)
{
    if (_viewer != nullptr)
    {
        _viewer->getCamera()->setClearColor(backGroundColor);
    }

}
void FormSettings::initVisualizationWindow()
{
    //注册插件
    osgDB::Registry::instance()->addFileExtensionAlias("bin", "bin");
    _viewer = _osgQOpenGLWidget->getOsgViewer();
    //调整一些参数
    QSize _size = _osgQOpenGLWidget->sizeHint();
    float _aspectRatio = (float)_size.width() / (float)_size.height();
    _viewer->getCamera()->setProjectionMatrixAsPerspective(30.f, _aspectRatio, 0.001f, FLT_MAX);
   _viewer->getCamera()->setClearColor(osg::Vec4(0.9412, 0.9412, 0.9412,1.0));
    //_viewer->getCamera()->setClearColor(osg::Vec4(1.0, 1.0, 1.0, 1.0));
    _root->setDataVariance(osg::Object::DYNAMIC);
    _root->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    _viewer->setSceneData(_root.get());
    //_viewer->addEventHandler(new osgGA::StateSetManipulator(_viewer->getCamera()->getOrCreateStateSet()));

}

void FormSettings::loadFiles(std::vector<std::string> inputFiles, std::string outputDir, osg::ref_ptr<osg::MSpaceNode> inputFileNode)
{
    std::filesystem::create_directories(outputDir);
    std::shared_ptr<MSpaceOCTree> _MSpaceOCTree = std::make_shared<MSpaceOCTree>(inputFiles, outputDir);
    if (_MSpaceOCTree->filesConverter(inputFileNode))
    {
        osgUtil::Optimizer optimizer;
        optimizer.optimize(inputFileNode);

        _root->addChild(inputFileNode);

        osg::BoundingSphere boundingSphere = inputFileNode->getBound();
        osg::ref_ptr<osgGA::TrackballManipulator> manipulator = new osgGA::TrackballManipulator;
        osg::Vec3 cameraPosition = boundingSphere.center() + osg::Vec3(0.0, 0.0, boundingSphere.radius());
        manipulator->setHomePosition(cameraPosition, boundingSphere.center(), osg::Vec3(0.0, 1.0, 0.0));
        _viewer->setCameraManipulator(manipulator);
    }
}

//void FormSettings::showMatchingNode(osg::ref_ptr<osg::Group> _matching3DRelationship, std::vector<string> IDOfExt, std::vector<std::vector<double>> postionOfExt, std::vector<string> IDOfRef, std::vector<std::vector   <double>> postionOfRef, std::vector<std::pair<size_t, size_t>> trueMatching, std::vector<size_t> resOfExt, std::vector<size_t> resOfRef)
//{
//    //数据格式转化为osg格式
//    std::vector<osg::Vec3> posOfExt;
//    std::vector<osg::Vec3> posOfRef;
//
//    if (postionOfExt.size() == 0)
//        return;
//    if (postionOfExt[0].size() == 3)
//    {
//        for (size_t i = 0;i < postionOfExt.size(); i++)
//        {
//            posOfExt.push_back(osg::Vec3(postionOfExt[i][0], postionOfExt[i][1], postionOfExt[i][2]));
//        }
//        for (size_t i = 0; i < postionOfRef.size(); i++)
//        {
//            posOfRef.push_back(osg::Vec3(postionOfRef[i][0], postionOfRef[i][1], postionOfRef[i][2]));
//        }
//    }
//    if (postionOfExt[0].size() == 2)
//    {
//        for (size_t i = 0; i < postionOfExt.size(); i++)
//        {
//            posOfExt.push_back(osg::Vec3(postionOfExt[i][0], postionOfExt[i][1], 1.0));
//        }
//        for (size_t i = 0; i < postionOfRef.size(); i++)
//        {
//            posOfRef.push_back(osg::Vec3(postionOfRef[i][0], postionOfRef[i][1], 1.0));
//        }
//    }
//    if (postionOfExt[0].size() == 1)
//    {
//        for (size_t i = 0; i < postionOfExt.size(); i++)
//        {
//            posOfExt.push_back(osg::Vec3(postionOfExt[i][0], 1.0, 1.0));
//        }
//        for (size_t i = 0; i < postionOfRef.size(); i++)
//        {
//            posOfRef.push_back(osg::Vec3(postionOfRef[i][0], 1.0, 1.0));
//        }
//    }
//    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
//
//
//    osg::ref_ptr<osg::Camera> camera = new osg::Camera;
//    camera->setRenderOrder(osg::Camera::POST_RENDER);
//    camera->setClearMask(GL_DEPTH_BUFFER_BIT);
//    //osg::ref_ptr<osg::Node> tree = osgDB::readNodeFile("./resource/tree.obj");
//    double scaleFactor = 1.0;
//    for (size_t i = 0; i < trueMatching.size(); i++)
//    {
//        //osg::ref_ptr<osg::Node> modelOfExt = osgDB::readNodeFile("./resource/tree.obj");
//        //osg::ref_ptr<osg::MatrixTransform> transformOfExt = new osg::MatrixTransform;
//        //ColorVisitor colorVisitorOfExt(osg::Vec4(0.0, 0.8, 0.0, 1.0));
//        //modelOfExt->accept(colorVisitorOfExt);
//        //transformOfExt->setMatrix(osg::Matrix::scale(0.2, 0.2, 0.2) * osg::Matrix::translate(posOfExt[trueMatching[i].first]));
//        //transformOfExt->addChild(modelOfExt);
//        //geode->addChild(transformOfExt);
//
//        //osg::ref_ptr<osg::Node> modelOfRef = osgDB::readNodeFile("./resource/tree.obj");
//        //osg::ref_ptr<osg::MatrixTransform> transformOfRef = new osg::MatrixTransform;
//        //ColorVisitor colorVisitorOfRef(osg::Vec4(0.8, 0.8, 0.0, 1.0));
//        //modelOfRef->accept(colorVisitorOfRef);
//        //transformOfRef->setMatrix(osg::Matrix::scale(0.2, 0.2, 0.2) * osg::Matrix::translate(posOfRef[trueMatching[i].second]));
//        //transformOfRef->addChild(modelOfRef);
//        //geode->addChild(transformOfRef);
//
//        osg::ref_ptr<osg::Sphere> sphereOfExt = new osg::Sphere(posOfExt[trueMatching[i].first], 0.03);
//        // 创建一个形状绘制对象，并设置其形状数据
//        osg::ref_ptr<osg::ShapeDrawable> sphereDrawableOfExt = new osg::ShapeDrawable(sphereOfExt);
//        // 设置球的颜色
//        osg::ref_ptr<osg::Vec4Array> colorsOfExt = new osg::Vec4Array;
//        colorsOfExt->push_back(osg::Vec4(0.0, 0.8, 0.0, 1.0)); // 设置为绿色
//        sphereDrawableOfExt->setColorArray(colorsOfExt);
//        sphereDrawableOfExt->setColorBinding(osg::Geometry::BIND_OVERALL);
//
//        // 创建一个Geode节点，并添加形状绘制对象
//        geode->addDrawable(sphereDrawableOfExt);
//
//        osg::ref_ptr<osg::Sphere> sphereOfRef = new osg::Sphere(posOfRef[trueMatching[i].second], 0.03);
//        // 创建一个形状绘制对象，并设置其形状数据
//        osg::ref_ptr<osg::ShapeDrawable> sphereDrawableOfRef = new osg::ShapeDrawable(sphereOfRef);
//        // 设置球的颜色
//        osg::ref_ptr<osg::Vec4Array> colorsOfRef = new osg::Vec4Array;
//        colorsOfRef->push_back(osg::Vec4(0.8, 0.8, 0.0, 1.0)); // 设置为黄色
//        sphereDrawableOfRef->setColorArray(colorsOfRef);
//        sphereDrawableOfRef->setColorBinding(osg::Geometry::BIND_OVERALL);
//        // 创建一个Geode节点，并添加形状绘制对象
//        geode->addDrawable(sphereDrawableOfRef);
//
//
//        osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
//        vertices->push_back(posOfExt[trueMatching[i].first]);
//        vertices->push_back(posOfRef[trueMatching[i].second]);
//        osg::ref_ptr<osg::Geometry> lineGeom = new osg::Geometry;
//        lineGeom->setVertexArray(vertices);
//        lineGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, vertices->size()));
//
//        osg::ref_ptr<osg::LineWidth> lineWidth = new osg::LineWidth(0.05);// 设置线的宽度
//        //lineWidth->setWidth(0.05); 
//        lineGeom->getOrCreateStateSet()->setAttribute(lineWidth);
//
//        geode->addDrawable(lineGeom);
//
//        osg::ref_ptr<osgText::Text> textOfExt = new osgText::Text;
//        textOfExt->setText("E_" + IDOfExt[trueMatching[i].first]);
//        textOfExt->setCharacterSize(0.01); // 设置文本的大小
//        textOfExt->setColor(osg::Vec4(0.0, 0.0, 0.0, 1.0));
//        textOfExt->setAxisAlignment(osgText::TextBase::SCREEN);
//        textOfExt->setPosition(posOfExt[trueMatching[i].first]);
//
//        // 创建Camera节点，设置其为后渲染
//        camera->addChild(textOfExt);
//
//        osg::ref_ptr<osgText::Text> textOfRef = new osgText::Text;
//        textOfRef->setText("R_" + IDOfRef[trueMatching[i].second]);
//        textOfRef->setCharacterSize(0.01); // 设置文本的大小
//        textOfRef->setColor(osg::Vec4(0.0, 0.0, 0.0, 1.0));
//        textOfRef->setAxisAlignment(osgText::TextBase::SCREEN);
//        textOfRef->setPosition(posOfRef[trueMatching[i].second]);
//
//        // 创建Camera节点，设置其为后渲染
//        camera->addChild(textOfRef);
//
//    }
//
//    for (size_t i = 0; i < resOfExt.size(); i++)
//    {
//        //osg::ref_ptr<osg::Node> modelOfUExt = osgDB::readNodeFile("./resource/tree.obj");
//        //osg::ref_ptr<osg::MatrixTransform> transformOfUExt = new osg::MatrixTransform;
//        //ColorVisitor colorVisitor(osg::Vec4(0.8, 0.0, 0.0, 1.0));
//        //modelOfUExt->accept(colorVisitor);
//        //transformOfUExt->setMatrix(osg::Matrix::scale(0.2, 0.2, 0.2)* osg::Matrix::translate(posOfExt[resOfExt[i]]));
//        //transformOfUExt->addChild(modelOfUExt);
//        //geode->addChild(transformOfUExt);
//
//
//        osg::ref_ptr<osg::Sphere> sphereOfUExt = new osg::Sphere(posOfExt[resOfExt[i]], 0.03);
//        // 创建一个形状绘制对象，并设置其形状数据
//        osg::ref_ptr<osg::ShapeDrawable> sphereDrawableOfUExt = new osg::ShapeDrawable(sphereOfUExt);
//        // 设置球的颜色
//        osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
//        colors->push_back(osg::Vec4(0.8, 0.0, 0.0, 1.0)); // 设置为红色
//        sphereDrawableOfUExt->setColorArray(colors);
//        sphereDrawableOfUExt->setColorBinding(osg::Geometry::BIND_OVERALL);
//        // 创建一个Geode节点，并添加形状绘制对象
//        geode->addDrawable(sphereDrawableOfUExt);
//
//        osg::ref_ptr<osgText::Text> textOfUExt = new osgText::Text;
//        textOfUExt->setText("UE_" + IDOfExt[resOfExt[i]]);
//        textOfUExt->setCharacterSize(0.01); // 设置文本的大小
//        textOfUExt->setColor(osg::Vec4(0.0, 0.0, 0.0, 1.0));
//        textOfUExt->setAxisAlignment(osgText::TextBase::SCREEN);
//        textOfUExt->setPosition(posOfExt[resOfExt[i]]);
//
//        // 创建Camera节点，设置其为后渲染
//        camera->addChild(textOfUExt);
//    }
//    for (size_t i = 0; i < resOfRef.size(); i++)
//    {
//        //osg::ref_ptr<osg::Node> modelOfURef = osgDB::readNodeFile("./resource/tree.obj");
//        //osg::ref_ptr<osg::MatrixTransform> transformOfURef = new osg::MatrixTransform;
//        //ColorVisitor colorVisitor(osg::Vec4(0.8, 0.0, 0.0, 1.0));
//        //modelOfURef->accept(colorVisitor);
//        //transformOfURef->setMatrix(osg::Matrix::scale(0.2, 0.2, 0.2)* osg::Matrix::translate(posOfRef[resOfRef[i]]));
//        //transformOfURef->addChild(modelOfURef);
//        //geode->addChild(transformOfURef);
//
//        osg::ref_ptr<osg::Sphere> sphereOfURef = new osg::Sphere(posOfRef[resOfRef[i]], 0.03);
//        // 创建一个形状绘制对象，并设置其形状数据
//        osg::ref_ptr<osg::ShapeDrawable> sphereDrawableOfURef = new osg::ShapeDrawable(sphereOfURef);
//        // 设置球的颜色
//        osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
//        colors->push_back(osg::Vec4(1.0, 0.0, 0.0, 1.0)); // 设置为红色
//        sphereDrawableOfURef->setColorArray(colors);
//        sphereDrawableOfURef->setColorBinding(osg::Geometry::BIND_OVERALL);
//        // 创建一个Geode节点，并添加形状绘制对象
//        geode->addDrawable(sphereDrawableOfURef);
//
//        osg::ref_ptr<osgText::Text> textOfURef = new osgText::Text;
//        textOfURef->setText("UR_" + IDOfRef[resOfRef[i]]);
//        textOfURef->setCharacterSize(0.01); // 设置文本的大小
//        textOfURef->setColor(osg::Vec4(0.0, 0.0, 0.0, 1.0));
//        textOfURef->setAxisAlignment(osgText::TextBase::SCREEN);
//        textOfURef->setPosition(posOfRef[resOfRef[i]]);
//
//        // 创建Camera节点，设置其为后渲染
//        camera->addChild(textOfURef);
//    }
//
//
//
//    _matching3DRelationship->addChild(geode);
//    _matching3DRelationship->addChild(camera);
//    _matching3DRelationship->setName("TreeMatchingNode");
//
//    _root->addChild(_matching3DRelationship);
//    osg::BoundingSphere boundingSphere = _matching3DRelationship->getBound();
//    // 计算相机的位置：节点中心 + 半径在Z轴方向
//    osg::Vec3 cameraPosition = boundingSphere.center() + osg::Vec3(0.0, 0.0, boundingSphere.radius());
//    // 设置相机的视图矩阵
//    _viewer->getCamera()->setViewMatrixAsLookAt(cameraPosition, boundingSphere.center(), osg::Vec3(0.0, 1.0, 0.0));
//    _viewer->setCameraManipulator(new osgGA::TrackballManipulator);
//}


void FormSettings::deletePointCloudNode(osg::ref_ptr<osg::MSpaceNode> Node, string outputDir)
{
    _root->removeChild(Node);

    // Clear osgDB object cache to release cached .bin file references
    osgDB::Registry::instance()->clearObjectCache();

    // Small delay to let OSG's background pager thread finish
    OpenThreads::Thread::microSleep(100000);

    // Delete files from disk with retry for Windows file lock timing
    int retries = 3;
    while (retries > 0)
    {
        std::error_code ec;
        std::filesystem::remove_all(outputDir, ec);
        if (!ec)
            break;
        OpenThreads::Thread::microSleep(200000);
        --retries;
    }
}

//void FormSettings::deeteTreeNode(osg::ref_ptr<osg::Group> Node)
//{
//    _root->removeChild(Node);
//}
