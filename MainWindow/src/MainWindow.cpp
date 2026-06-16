#include "../include/MainWindow.h"
#include <QtWidgets/QProgressDialog>
#include <QtConcurrent/QtConcurrent>
#include <QtWidgets/QProgressBar>
#include <QtWidgets/QFileDialog>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);
    InitForm();
    InitTree();
    InitLanguage();
    InitDataMangementMenu();
    connect(_rootNode.get(), &QStandardItemModel::itemChanged, this, &MainWindow::HiddenData);
    connect(ui._fileTree, &QTreeView::customContextMenuRequested, this, &MainWindow::handleContextMenuRequested);
    connect(ui._action_inputFile, &QAction::triggered, this, &MainWindow::loadFile);
	connect(ui.action_registration_ULS, &QAction::triggered, this, &MainWindow::registration_TLS_ULS);
	connect(ui.action_registration_TLS, &QAction::triggered, this, &MainWindow::registration_TLS);
	connect(ui.action_registration_Fast, &QAction::triggered, this, &MainWindow::outlierRemovalRegistration);
	connect(ui.action_registration_Forest, &QAction::triggered, this, &MainWindow::registration_Forest);
    //connect(ui._action_treeEvaluation, &QAction::triggered, this, &MainWindow::treeEvaluation);
    //connect(ui._action_stemCurveEvaluation, &QAction::triggered, this, &MainWindow::stemCurveEvaluation);
    connect(ui._action_Chinese, &QAction::triggered, this, &MainWindow::changeLanguage_Chinese);
    connect(ui._action_English, &QAction::triggered, this, &MainWindow::changeLanguage_English);
}

MainWindow::~MainWindow()
{
}

void MainWindow::InitForm()
{
    _formSettings = std::make_shared<FormSettings>(ui._VisialArea);
    ui._Logger->setReadOnly(true);
    //ui.NodeInformation->setContentsMargins(0, 0, 0, 0);
    //ui.ProPertyInformation->setContentsMargins(0, 0, 0, 0);
    //ui.OutInformation->setContentsMargins(0, 0, 0, 0);
}

void MainWindow::InitTree()
{
    ui._fileTree->setEditTriggers(QTreeView::NoEditTriggers);
    ui._fileTree->setSelectionBehavior(QTreeView::SelectRows);
    ui._fileTree->setHeaderHidden(true);

    _rootNode = std::make_shared<QStandardItemModel>(this);
    ui._fileTree->setModel(_rootNode.get());
    ui._fileTree->setContextMenuPolicy(Qt::CustomContextMenu);
}

void MainWindow::InitLanguage()
{
    chinese.load("./resource/MainWindow_zh.qm");
    english.load("./resource/MainWindow_en.qm");
}

void MainWindow::InitDataMangementMenu()
{
    _dataMangementMenu = std::make_shared<QMenu>();
    QAction* deleteAction = new QAction(tr("delete"), this);
    // 将删除动作与槽函数进行连接
    connect(deleteAction, &QAction::triggered, this, &MainWindow::DeleteData);
    // 将删除动作添加到菜单中
    _dataMangementMenu->addAction(deleteAction);
}

void MainWindow::LoadAndShowFiles(std::string inputFiles)
{
    try
    {
        std::shared_ptr<QProgressDialog> pDlg = std::make_shared<QProgressDialog>(this);
        pDlg->setWindowTitle(tr("Notice"));
        pDlg->setModal(true);
        pDlg->setLabelText("Loading data...");
        pDlg->setWindowFlags(pDlg->windowFlags() & ~Qt::WindowCloseButtonHint & ~Qt::WindowContextHelpButtonHint);
        pDlg->setCancelButton(nullptr);
        pDlg->show();
        pDlg->raise();

        QProgressBar* pBar = new QProgressBar(pDlg.get());
        pBar->setRange(0, 0);
        pBar->setAlignment(Qt::AlignCenter);
        pDlg->setBar(pBar);

        std::vector<std::string> inputfile{ inputFiles };
        osg::ref_ptr<osg::MSpaceNode> inputFileNode = new osg::MSpaceNode;
        inputFileNode->setName(std::filesystem::path(inputFiles).stem().string());

        const std::string baseTmp = (QCoreApplication::applicationDirPath() + "/tmp/").toStdString();
        const std::string outDir = baseTmp + std::filesystem::path(inputFiles).stem().string();

        QFuture<void> future = QtConcurrent::run(std::bind(&FormSettings::loadFiles, _formSettings, inputfile, outDir, inputFileNode));
        while (!future.isFinished())
        {
            if (pDlg)
            {
                pDlg->setValue(pDlg->value() + 1);
                QApplication::processEvents();
            }
        }

        // 确认节点确实加载到了场景
        if (!inputFileNode.valid() || inputFileNode->getNumChildren() == 0)
        {
            ui._Logger->insertPlainText(QStringLiteral("Point cloud not loaded into scene (check plugins/resources)!\n"));
            return;
        }

        QStandardItem* fileNode = new QStandardItem(QString::fromStdString(inputFiles));
        fileNode->setCheckable(true);
        fileNode->setCheckState(Qt::Checked);
        _rootNode->appendRow(fileNode);
        _itemToPointCloud.insert(std::make_pair(fileNode, inputFileNode));
        ui._fileTree->show();

        updatePropertyPanel(inputFiles, inputFileNode);
    }
    catch (const std::exception&)
    {
        ui._Logger->insertPlainText(tr("Point cloud data loading and display failed!") + "\n");
    }
    


}

void MainWindow::HiddenData(QStandardItem* item)
{
    auto it = _itemToPointCloud.find(item);
    if (it == _itemToPointCloud.end())
        return;

    Qt::CheckState state = item->checkState();
    osg::ref_ptr<osg::Node> node = it->second;
    if (!node.valid())
        return;

    if (state == Qt::Checked)
    {
        node->setNodeMask(0xFFFFFFFF);
        ui._Logger->insertPlainText(QStringLiteral("The data \"%1\" has been shown！\n").arg(item->text()));
    }
    else
    {
        node->setNodeMask(0);
        ui._Logger->insertPlainText(QStringLiteral("The data \"%1\" has been hidden！\n").arg(item->text()));
    }
}

void MainWindow::handleContextMenuRequested(const QPoint& pos)
{
    QModelIndex index = ui._fileTree->indexAt(pos);

    if (index.isValid())
    {
        QStandardItem* item = _rootNode->itemFromIndex(index);
        if (item != nullptr && _itemToPointCloud.count(item) > 0)
        {
            _dataMangementMenu->exec(ui._fileTree->viewport()->mapToGlobal(pos));
        }
    }
}

void MainWindow::DeleteData()
{
    QModelIndex currentIndex = ui._fileTree->currentIndex();

    QStandardItem* item = _rootNode->itemFromIndex(currentIndex);
    if (item == nullptr)
    {
        ui._Logger->insertPlainText(tr("Unable to delete node！") + "\n");
        return;
    }

    auto it = _itemToPointCloud.find(item);
    if (it == _itemToPointCloud.end())
    {
        ui._Logger->insertPlainText(tr("Unable to delete node ！") + "\n");
        return;
    }

    osg::ref_ptr<osg::MSpaceNode> deleteNode = dynamic_cast<osg::MSpaceNode*>(it->second.get());
    if (deleteNode.valid())
    {
        _formSettings->deletePointCloudNode(deleteNode, "./tmp/" + deleteNode->getName());
        ui._Logger->insertPlainText(QStringLiteral("The node \"%1\" has been deleted！\n").arg(deleteNode->getName().c_str()));
    }

    _itemToPointCloud.erase(it);
    _rootNode->removeRow(item->row());
}

void MainWindow::loadFile()
{
    QStringList fileTypes;
    fileTypes << "All Point Cloud Files (*.las *.laz *.pcd *.ply)"
              << "LAS Files (*.las)"
              << "LAZ Files (*.laz)"
              << "PCD Files (*.pcd)"
              << "PLY Files (*.ply)";
    QString file = QFileDialog::getOpenFileName(this, tr("Select Point Cloud File"), "", fileTypes.join(";;"));
    if (file.isEmpty())
    {
        ui._Logger->insertPlainText(tr("Please select a valid file!") + "\n");
        return;
    }

    LoadAndShowFiles(file.toStdString());
}

void MainWindow::registration_TLS_ULS()
{
	std::shared_ptr<RegistrationULS> _registrationULS = std::make_shared<RegistrationULS>(this);
    if (!_registrationULS->exec())
		return;
	std::shared_ptr<QProgressDialog> pDlg = std::make_shared<QProgressDialog>(this);
	pDlg->setWindowTitle(tr("Tips"));
	pDlg->setModal(true); // 设置为模态对话框，阻塞用户操作
	QProgressBar* pBar = new QProgressBar(pDlg.get());
	pBar->setRange(0, 0);
	pBar->setAlignment(Qt::AlignCenter);
	pDlg->setBar(pBar);
	_registrationULS->executeRegistration(pDlg.get(), ui._Logger);
  
}

void MainWindow::outlierRemovalRegistration()
{
    std::shared_ptr<OutlierRemovalRegistration> _outlierRemovalRegistration = std::make_shared<OutlierRemovalRegistration>(this);
    if (!_outlierRemovalRegistration->exec())
        return;
    std::shared_ptr<QProgressDialog> pDlg = std::make_shared<QProgressDialog>(this);
    pDlg->setWindowTitle(tr("Tips"));
    pDlg->setModal(true); // 设置为模态对话框，阻塞用户操作
    QProgressBar* pBar = new QProgressBar(pDlg.get());
    pBar->setRange(0, 0);
    pBar->setAlignment(Qt::AlignCenter);
    pDlg->setBar(pBar);
	_outlierRemovalRegistration->executeRegistration(pDlg.get(), ui._Logger);
}

void MainWindow::registration_TLS()
{
    std::shared_ptr<RegistrationForm_TLS> _registrationTLS = std::make_shared<RegistrationForm_TLS>(this);
    if (!_registrationTLS->exec())
        return;
    std::shared_ptr<QProgressDialog> pDlg = std::make_shared<QProgressDialog>(this);
    pDlg->setWindowTitle(tr("Tips"));
    pDlg->setModal(true); // 设置为模态对话框，阻塞用户操作
    QProgressBar* pBar = new QProgressBar(pDlg.get());
    pBar->setRange(0, 0);
    pBar->setAlignment(Qt::AlignCenter);
    pDlg->setBar(pBar);
    _registrationTLS->executeRegistration(pDlg.get(), ui._Logger);
}

void MainWindow::registration_Forest()
{
	std::shared_ptr<ForestRegistration> _forestRegistration = std::make_shared<ForestRegistration>(this);
    if (!_forestRegistration->exec())
		return;
	std::shared_ptr<QProgressDialog> pDlg = std::make_shared<QProgressDialog>(this);
	pDlg->setWindowTitle(tr("Tips"));
	pDlg->setModal(true);
	QProgressBar* pBar = new QProgressBar(pDlg.get());
	pBar->setRange(0, 0);
	pBar->setAlignment(Qt::AlignCenter);
	pDlg->setBar(pBar);
	_forestRegistration->executeRegistration(pDlg.get(), ui._Logger);
}

void MainWindow::changeLanguage_Chinese()
{
    qApp->installTranslator(&chinese);
    ui.retranslateUi(this);
    output_English = false;
}

void MainWindow::changeLanguage_English()
{
    qApp->installTranslator(&english);
    ui.retranslateUi(this);
    output_English = true;
}

void MainWindow::updatePropertyPanel(const std::string& fileName, osg::ref_ptr<osg::MSpaceNode> node)
{
    ui._ProPertyTable->clear();
    ui._ProPertyTable->setColumnCount(2);
    ui._ProPertyTable->setHorizontalHeaderLabels({ tr("Property"), tr("Value") });
    ui._ProPertyTable->horizontalHeader()->setStretchLastSection(true);

    QStringList properties;
    QStringList values;

    properties << tr("File Name");
    values << QString::fromStdString(std::filesystem::path(fileName).filename().string());

    properties << tr("File Format");
    values << QString::fromStdString(std::filesystem::path(fileName).extension().string());

    if (node.valid())
    {
        osg::BoundingSphere bs = node->getBound();
        properties << tr("Center X") << tr("Center Y") << tr("Center Z");
        values << QString::number(bs.center().x(), 'f', 4)
               << QString::number(bs.center().y(), 'f', 4)
               << QString::number(bs.center().z(), 'f', 4);

        properties << tr("Bounding Radius");
        values << QString::number(bs.radius(), 'f', 4);
    }

    properties << tr("Children Count");
    values << QString::number(node->getNumChildren());

    ui._ProPertyTable->setRowCount(properties.size());
    for (int i = 0; i < properties.size(); i++)
    {
        ui._ProPertyTable->setItem(i, 0, new QTableWidgetItem(properties[i]));
        ui._ProPertyTable->setItem(i, 1, new QTableWidgetItem(values[i]));
    }
}

//void MainWindow::treeEvaluation()
//{
//    try
//    {
//        std::shared_ptr<TreeEvaluation> _treeEvaluation = std::make_shared<TreeEvaluation>(this);
//        if (!_treeEvaluation->exec())
//            return;
//        std::shared_ptr<QProgressDialog> pDlg = std::make_shared<QProgressDialog>(this);
//        pDlg->setWindowTitle(tr("提示"));
//        pDlg->setModal(true); // 设置为模态对话框，阻塞用户操作
//        QProgressBar* pBar = new QProgressBar(pDlg.get());
//        pBar->setRange(0, 0);
//        pBar->setAlignment(Qt::AlignCenter);
//        pDlg->setBar(pBar);
//        _treeEvaluation->process(pDlg.get(), _matching3D, output_English);
//        if (_treeEvaluation->isShowTable() && (_matching3D->getExtractedIDs_and_referenceIDs().size() != 0))
//        {
//            //根据ID来获取匹配索引
//            std::vector<std::pair<size_t, size_t>> matchingIndex;
//            for (size_t i = 0; i < _matching3D->getExtractedIDs_and_referenceIDs().size(); i++)
//            {
//                std::pair<size_t, size_t> matchingIndex_tmp;
//                for (size_t j = 0; j < _matching3D->getIDOfExtractedTree().size(); j++)
//                {
//                    if (_matching3D->getExtractedIDs_and_referenceIDs()[i].first == _matching3D->getIDOfExtractedTree()[j])
//                    {
//                        matchingIndex_tmp.first = j;
//                        break;
//                    }
//                }
//                for (size_t j = 0; j < _matching3D->getIDOfReferenceTree().size(); j++)
//                {
//                    if (_matching3D->getExtractedIDs_and_referenceIDs()[i].second == _matching3D->getIDOfReferenceTree()[j])
//                    {
//                        matchingIndex_tmp.second = j;
//                        break;
//                    }
//                }
//                matchingIndex.push_back(matchingIndex_tmp);
//            }
//
//            //根据ID来获取匹配索引
//            std::vector<size_t> unMatchingIndexOfExt;
//            for (size_t i = 0; i < _matching3D->getResOfExt().size(); i++)
//            {
//                for (size_t j = 0; j < _matching3D->getIDOfExtractedTree().size(); j++)
//                {
//                    if (_matching3D->getResOfExt()[i] == _matching3D->getIDOfExtractedTree()[j])
//                    {
//                        unMatchingIndexOfExt.push_back(j);
//                        break;
//                    }
//                }
//            }
//            std::vector<size_t> unMatchingIndexOfRef;
//            for (size_t i = 0; i < _matching3D->getResOfRef().size(); i++)
//            {
//                for (size_t j = 0; j < _matching3D->getIDOfReferenceTree().size(); j++)
//                {
//                    if (_matching3D->getResOfRef()[i] == _matching3D->getIDOfReferenceTree()[j])
//                    {
//                        unMatchingIndexOfRef.push_back(j);
//                        break;
//                    }
//                }
//            }
//            showTreeInputInformation(_treeEvaluation, matchingIndex, unMatchingIndexOfExt, unMatchingIndexOfRef);
//
//            osg::ref_ptr<osg::Group> inputFileNode = new osg::Group;
//            _formSettings->showMatchingNode(inputFileNode, _matching3D->getIDOfExtractedTree(), _matching3D->getPositionOfExtrctedTree(), _matching3D->getIDOfReferenceTree(), _matching3D->getPositionOfReferenceTree(), matchingIndex, unMatchingIndexOfExt, unMatchingIndexOfRef);
//
//            QStandardItem* fileNode = new QStandardItem(tr("树木匹配结果示意图"));
//            fileNode->setCheckable(true);
//            fileNode->setCheckState(Qt::Checked);
//            _rootNode->appendRow(fileNode);
//            _itemToPointCloud.insert(std::make_pair(fileNode, inputFileNode));
//            ui._fileTree->show();
//        }
//    }
//    catch (const std::exception&)
//    {
//        ui._Logger->insertPlainText(tr("树木参数预测结果评价：请检查文件格式和输入参数是否正确！") + "\n");
//    }
//
//
//
//}
//
//void MainWindow::stemCurveEvaluation()
//{
//    try
//    {
//        std::shared_ptr<StemCurveEvaluation> _stemCurveEvaluation = std::make_shared<StemCurveEvaluation>(this);
//        if (!_stemCurveEvaluation->exec())
//            return;
//        std::shared_ptr<QProgressDialog> pDlg = std::make_shared<QProgressDialog>(this);
//        pDlg->setWindowTitle(tr("提示"));
//        pDlg->setModal(true); // 设置为模态对话框，阻塞用户操作
//        QProgressBar* pBar = new QProgressBar(pDlg.get());
//        pBar->setRange(0, 0);
//        pBar->setAlignment(Qt::AlignCenter);
//        pDlg->setBar(pBar);
//        _stemCurveEvaluation->process(pDlg.get(), _matching3D, output_English);
//    }
//    catch (const std::exception&)
//    {
//        ui._Logger->insertPlainText(tr("杆曲线预测结果评价：请检查文件格式和输入参数是否正确！") + "\n");
//    }
//
//}
//
//void MainWindow::showTreeInputInformation(std::shared_ptr<TreeEvaluation> treeEvaluation, std::vector<std::pair<size_t, size_t>> matchingIndex, std::vector<size_t> unMatchingIndexOfExt, std::vector<size_t> unMatchingIndexOfRef)
//{
//
//    QFont font;
//    font.setPointSize(10);
//
//    QDockWidget* _dataShowOfExt = new QDockWidget(this);
//    _dataShowOfExt->setWindowTitle(tr("预测树数据表(图中模型用绿色表示,树木ID为：E_ID)"));
//    QWidget* tmpOfExt = new QWidget();
//
//    _dataShowOfExt->setFont(font);
//    _dataShowOfExt->setAutoFillBackground(true);
//    QVBoxLayout* verticalLayoutOfExt = new QVBoxLayout(tmpOfExt);
//    verticalLayoutOfExt->setSpacing(6);
//    verticalLayoutOfExt->setContentsMargins(11, 11, 11, 11);
//    QTableWidget* _resultOfExt;
//
//
//    _resultOfExt = new QTableWidget(_matching3D->getExtractedIDs_and_referenceIDs().size(), _matching3D->getDimensionOfExt() + _matching3D->getNumberOfFeatureTypeOfExt() + 2, tmpOfExt);
//
//    _resultOfExt->setStyleSheet("selection-background-color: lightBlue");
//    _resultOfExt->verticalHeader()->setHidden(true);
//    _resultOfExt->setFont(font);
//    _resultOfExt->horizontalHeader()->setStyleSheet("QHeaderView::section{background:lightGray;}"); //设置表头背景色
//    _resultOfExt->setSelectionBehavior(QAbstractItemView::SelectRows);
//    //_result->setMouseTracking(true);//开启捕获鼠标功能
//
//    verticalLayoutOfExt->addWidget(_resultOfExt);
//    _dataShowOfExt->setWidget(tmpOfExt);
//    this->addDockWidget(Qt::RightDockWidgetArea, _dataShowOfExt);
//    this->tabifyDockWidget(ui.ProPertyInformation, _dataShowOfExt);
//
//    _resultOfExt->setEditTriggers(QAbstractItemView::NoEditTriggers);
//
//    _resultOfExt->horizontalHeader()->setHighlightSections(false);
//    _resultOfExt->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
//
//    _resultOfExt->setSelectionMode(QAbstractItemView::ContiguousSelection);
//
//    QStringList header_ext;
//    header_ext << tr("ID");
//    if (_matching3D->getDimensionOfExt() == 3)
//    {
//        header_ext << tr("X") << tr("Y") << tr("Z");
//    }
//    else if (_matching3D->getDimensionOfExt() == 2)
//    {
//        header_ext << tr("X") << tr("Y");
//    }
//    else if (_matching3D->getDimensionOfExt() == 1)
//    {
//        header_ext << tr("X");
//    }
//    for (size_t i = 0; i < _matching3D->getNumberOfFeatureTypeOfExt(); i++)
//    {
//        string tmpName;
//        if (output_English == true)
//        {
//            tmpName = "Feature" + std::to_string(i + 1);
//        }
//        else
//        {
//            tmpName = "第" + std::to_string(i + 1) + "组特征";
//        }
//
//        header_ext << tmpName.c_str();
//    }
//    header_ext << tr("对应的参考树ID");
//    _resultOfExt->setHorizontalHeaderLabels(header_ext);
//
//    for (size_t i = 0; i < matchingIndex.size(); i++)
//    {
//        _resultOfExt->setItem(i, 0, new QTableWidgetItem(QString::fromStdString(_matching3D->getIDOfExtractedTree()[matchingIndex[i].first])));
//        _resultOfExt->item(i, 0)->setTextAlignment(Qt::AlignCenter);
//        for (size_t j = 0; j < _matching3D->getDimensionOfExt(); j++)
//        {
//            _resultOfExt->setItem(i, j + 1, new QTableWidgetItem(QString::number(_matching3D->getPositionOfExtrctedTree()[matchingIndex[i].first][j])));
//            _resultOfExt->item(i, j + 1)->setTextAlignment(Qt::AlignCenter);
//        }
//        for (size_t j = 0; j < _matching3D->getNumberOfFeatureTypeOfExt(); j++)
//        {
//            _resultOfExt->setItem(i, j + 1 + _matching3D->getDimensionOfExt(), new QTableWidgetItem(QString::number(_matching3D->getFeatureOfExtractedTree()[matchingIndex[i].first][j])));
//            _resultOfExt->item(i, j + 1 + _matching3D->getDimensionOfExt())->setTextAlignment(Qt::AlignCenter);
//        }
//        _resultOfExt->setItem(i, _matching3D->getNumberOfFeatureTypeOfExt() + 1 + _matching3D->getDimensionOfExt(), new QTableWidgetItem(QString::fromStdString(_matching3D->getIDOfReferenceTree()[matchingIndex[i].second])));
//        _resultOfExt->item(i, _matching3D->getNumberOfFeatureTypeOfExt() + 1 + _matching3D->getDimensionOfExt())->setTextAlignment(Qt::AlignCenter);
//    }
//
//    QDockWidget* _dataShowOfRef = new QDockWidget(this);
//    _dataShowOfRef->setWindowTitle(tr("参考树数据表(图中模型用黄色表示,树木ID为：R_ID)"));
//    QWidget* tmpOfRef = new QWidget();
//
//    _dataShowOfRef->setFont(font);
//    _dataShowOfRef->setAutoFillBackground(true);
//    QVBoxLayout* verticalLayoutOfRef = new QVBoxLayout(tmpOfRef);
//    verticalLayoutOfRef->setSpacing(6);
//    verticalLayoutOfRef->setContentsMargins(11, 11, 11, 11);
//    QTableWidget* _resultOfRef;
//
//    _resultOfRef = new QTableWidget(_matching3D->getExtractedIDs_and_referenceIDs().size(), _matching3D->getDimensionOfRef() + _matching3D->getNumberOfFeatureTypeOfRef() + 2, tmpOfRef);
//
//    _resultOfRef->setStyleSheet("selection-background-color: lightBlue");
//    _resultOfRef->verticalHeader()->setHidden(true);
//    _resultOfRef->setFont(font);
//    _resultOfRef->horizontalHeader()->setStyleSheet("QHeaderView::section{background:lightGray;}"); //设置表头背景色
//    _resultOfRef->setSelectionBehavior(QAbstractItemView::SelectRows);
//    //_result->setMouseTracking(true);//开启捕获鼠标功能
//
//    verticalLayoutOfRef->addWidget(_resultOfRef);
//    _dataShowOfRef->setWidget(tmpOfRef);
//    this->addDockWidget(Qt::RightDockWidgetArea, _dataShowOfRef);
//    this->tabifyDockWidget(_dataShowOfExt, _dataShowOfRef);
//
//    _resultOfRef->setEditTriggers(QAbstractItemView::NoEditTriggers);
//
//    _resultOfRef->horizontalHeader()->setHighlightSections(false);
//    _resultOfRef->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
//
//    _resultOfRef->setSelectionMode(QAbstractItemView::ContiguousSelection);
//
//    QStringList header_ref;
//    header_ref << tr("ID");
//    if (_matching3D->getDimensionOfRef() == 3)
//    {
//        header_ref << tr("X") << tr("Y") << tr("Z");
//    }
//    else if (_matching3D->getDimensionOfRef() == 2)
//    {
//        header_ref << tr("X") << tr("Y");
//    }
//    else if (_matching3D->getDimensionOfRef() == 1)
//    {
//        header_ref << tr("X");
//    }
//    for (size_t i = 0; i < _matching3D->getNumberOfFeatureTypeOfRef(); i++)
//    {
//        string tmpName;
//        if (output_English == true)
//        {
//            tmpName = "Feature" + std::to_string(i + 1);
//        }
//        else
//        {
//            tmpName = "第" + std::to_string(i + 1) + "组特征";
//        }
//
//        header_ref << tmpName.c_str();
//    }
//    header_ref << tr("对应的预测树ID");
//    _resultOfRef->setHorizontalHeaderLabels(header_ref);
//
//    for (size_t i = 0; i < matchingIndex.size(); i++)
//    {
//        _resultOfRef->setItem(i, 0, new QTableWidgetItem(QString::fromStdString(_matching3D->getIDOfReferenceTree()[matchingIndex[i].second])));
//        _resultOfRef->item(i, 0)->setTextAlignment(Qt::AlignCenter);
//        for (size_t j = 0; j < _matching3D->getDimensionOfRef(); j++)
//        {
//            _resultOfRef->setItem(i, j + 1, new QTableWidgetItem(QString::number(_matching3D->getPositionOfReferenceTree()[matchingIndex[i].second][j])));
//            _resultOfRef->item(i, j + 1)->setTextAlignment(Qt::AlignCenter);
//        }
//        for (size_t j = 0; j < _matching3D->getNumberOfFeatureTypeOfRef(); j++)
//        {
//            _resultOfRef->setItem(i, j + 1 + _matching3D->getDimensionOfRef(), new QTableWidgetItem(QString::number(_matching3D->getFeatureOfReferenceTree()[matchingIndex[i].second][j])));
//            _resultOfRef->item(i, j + 1 + _matching3D->getDimensionOfRef())->setTextAlignment(Qt::AlignCenter);
//        }
//        _resultOfRef->setItem(i, _matching3D->getNumberOfFeatureTypeOfRef() + 1 + _matching3D->getDimensionOfRef(), new QTableWidgetItem(QString::fromStdString(_matching3D->getIDOfExtractedTree()[matchingIndex[i].first])));
//        _resultOfRef->item(i, _matching3D->getNumberOfFeatureTypeOfRef() + 1 + _matching3D->getDimensionOfRef())->setTextAlignment(Qt::AlignCenter);
//    }
//
//
//    connect(_resultOfExt, &QTableWidget::itemClicked, [=](QTableWidgetItem* item) {
//        int row = item->row(); // 获取行号
//        _resultOfRef->setCurrentCell(row, QItemSelectionModel::Select);
//        });
//    connect(_resultOfRef, &QTableWidget::itemClicked, [=](QTableWidgetItem* item) {
//        int row = item->row(); // 获取行号
//        _resultOfExt->setCurrentCell(row, QItemSelectionModel::Select);
//        });
//
//    QDockWidget* _dataShowOfUExt = new QDockWidget(this);
//    _dataShowOfUExt->setWindowTitle(tr("未匹配的预测树数据表(图中模型用红色表示,树木ID为：UE_ID)"));
//    QWidget* tmpOfUExt = new QWidget();
//
//    _dataShowOfUExt->setFont(font);
//    _dataShowOfUExt->setAutoFillBackground(true);
//    QVBoxLayout* verticalLayoutOfUExt = new QVBoxLayout(tmpOfUExt);
//    verticalLayoutOfUExt->setSpacing(6);
//    verticalLayoutOfUExt->setContentsMargins(11, 11, 11, 11);
//    QTableWidget* _resultOfUExt;
//
//    _resultOfUExt = new QTableWidget(_matching3D->getResOfExt().size(),1 +  _matching3D->getDimensionOfExt() + _matching3D->getNumberOfFeatureTypeOfExt(), tmpOfUExt);
//
//    _resultOfUExt->verticalHeader()->setHidden(true);
//    _resultOfUExt->setFont(font);
//    _resultOfUExt->horizontalHeader()->setStyleSheet("QHeaderView::section{background:lightGray;}"); //设置表头背景色
//    _resultOfUExt->setSelectionBehavior(QAbstractItemView::SelectRows);
//    //_result->setMouseTracking(true);//开启捕获鼠标功能
//
//    verticalLayoutOfUExt->addWidget(_resultOfUExt);
//    _dataShowOfUExt->setWidget(tmpOfUExt);
//    this->addDockWidget(Qt::RightDockWidgetArea, _dataShowOfUExt);
//    this->tabifyDockWidget(_dataShowOfRef, _dataShowOfUExt);
//
//    _resultOfUExt->setEditTriggers(QAbstractItemView::NoEditTriggers);
//
//    _resultOfUExt->horizontalHeader()->setHighlightSections(false);
//    _resultOfUExt->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
//
//    _resultOfUExt->setSelectionMode(QAbstractItemView::ContiguousSelection);
//
//    QStringList header_uext;
//    header_uext << tr("ID");
//    if (_matching3D->getDimensionOfExt() == 3)
//    {
//        header_uext << tr("X") << tr("Y") << tr("Z");
//    }
//    else if (_matching3D->getDimensionOfExt() == 2)
//    {
//        header_uext << tr("X") << tr("Y");
//    }
//    else if (_matching3D->getDimensionOfExt() == 1)
//    {
//        header_uext << tr("X");
//    }
//    for (size_t i = 0; i < _matching3D->getNumberOfFeatureTypeOfExt(); i++)
//    {
//        string tmpName;
//        if (output_English == true)
//        {
//            tmpName = "Feature" + std::to_string(i + 1);
//        }
//        else
//        {
//            tmpName = "第" + std::to_string(i + 1) + "组特征";
//        }
//
//        header_uext << tmpName.c_str();
//    }
//
//    _resultOfUExt->setHorizontalHeaderLabels(header_uext);
//
//    for (size_t i = 0; i < unMatchingIndexOfExt.size(); i++)
//    {
//        _resultOfUExt->setItem(i, 0, new QTableWidgetItem(QString::fromStdString(_matching3D->getIDOfExtractedTree()[unMatchingIndexOfExt[i]])));
//        _resultOfUExt->item(i, 0)->setTextAlignment(Qt::AlignCenter);
//        for (size_t j = 0; j < _matching3D->getDimensionOfExt(); j++)
//        {
//            _resultOfUExt->setItem(i, j + 1, new QTableWidgetItem(QString::number(_matching3D->getPositionOfExtrctedTree()[unMatchingIndexOfExt[i]][j])));
//            _resultOfUExt->item(i, j + 1)->setTextAlignment(Qt::AlignCenter);
//        }
//        for (size_t j = 0; j < _matching3D->getNumberOfFeatureTypeOfExt(); j++)
//        {
//            _resultOfUExt->setItem(i, j + 1 + _matching3D->getDimensionOfExt(), new QTableWidgetItem(QString::number(_matching3D->getFeatureOfExtractedTree()[unMatchingIndexOfExt[i]][j])));
//            _resultOfUExt->item(i, j + 1 + _matching3D->getDimensionOfExt())->setTextAlignment(Qt::AlignCenter);
//        }
//    }
//
//    QDockWidget* _dataShowOfURef = new QDockWidget(this);
//    _dataShowOfURef->setWindowTitle(tr("未匹配的参考树数据表(图中模型用红色表示,树木ID为：UR_ID)"));
//    QWidget* tmpOfURef = new QWidget();
//
//    _dataShowOfURef->setFont(font);
//    _dataShowOfURef->setAutoFillBackground(true);
//    QVBoxLayout* verticalLayoutOfURef = new QVBoxLayout(tmpOfURef);
//    verticalLayoutOfURef->setSpacing(6);
//    verticalLayoutOfURef->setContentsMargins(11, 11, 11, 11);
//    QTableWidget* _resultOfURef;
//
//    _resultOfURef = new QTableWidget(_matching3D->getResOfRef().size(), _matching3D->getDimensionOfRef() + _matching3D->getNumberOfFeatureTypeOfRef() + 1, tmpOfURef);
//
//    _resultOfURef->setStyleSheet("selection-background-color: lightBlue");
//    _resultOfURef->verticalHeader()->setHidden(true);
//    _resultOfURef->setFont(font);
//    _resultOfURef->horizontalHeader()->setStyleSheet("QHeaderView::section{background:lightGray;}"); //设置表头背景色
//    _resultOfURef->setSelectionBehavior(QAbstractItemView::SelectRows);
//    //_result->setMouseTracking(true);//开启捕获鼠标功能
//
//    verticalLayoutOfURef->addWidget(_resultOfURef);
//    _dataShowOfURef->setWidget(tmpOfURef);
//    this->addDockWidget(Qt::RightDockWidgetArea, _dataShowOfURef);
//    this->tabifyDockWidget(_dataShowOfRef, _dataShowOfURef);
//
//    _resultOfURef->setEditTriggers(QAbstractItemView::NoEditTriggers);
//
//    _resultOfURef->horizontalHeader()->setHighlightSections(false);
//    _resultOfURef->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
//
//    _resultOfURef->setSelectionMode(QAbstractItemView::ContiguousSelection);
//
//    QStringList header_uref;
//    header_uref << tr("ID");
//    if (_matching3D->getDimensionOfRef() == 3)
//    {
//        header_uref << tr("X") << tr("Y") << tr("Z");
//    }
//    else if (_matching3D->getDimensionOfRef() == 2)
//    {
//        header_uref << tr("X") << tr("Y");
//    }
//    else if (_matching3D->getDimensionOfRef() == 1)
//    {
//        header_uref << tr("X");
//    }
//    for (size_t i = 0; i < _matching3D->getNumberOfFeatureTypeOfRef(); i++)
//    {
//        string tmpName;
//        if (output_English == true)
//        {
//            tmpName = "Feature" + std::to_string(i + 1);
//        }
//        else
//        {
//            tmpName = "第" + std::to_string(i + 1) + "组特征";
//        }
//
//        header_uref << tmpName.c_str();
//    }
//    _resultOfURef->setHorizontalHeaderLabels(header_uref);
//
//
//    for (size_t i = 0; i < unMatchingIndexOfRef.size(); i++)
//    {
//        _resultOfURef->setItem(i, 0, new QTableWidgetItem(QString::fromStdString(_matching3D->getIDOfReferenceTree()[unMatchingIndexOfRef[i]])));
//        _resultOfURef->item(i, 0)->setTextAlignment(Qt::AlignCenter);
//        for (size_t j = 0; j < _matching3D->getDimensionOfRef(); j++)
//        {
//            _resultOfURef->setItem(i, j + 1, new QTableWidgetItem(QString::number(_matching3D->getPositionOfReferenceTree()[unMatchingIndexOfRef[i]][j])));
//            _resultOfURef->item(i, j + 1)->setTextAlignment(Qt::AlignCenter);
//        }
//        for (size_t j = 0; j < _matching3D->getNumberOfFeatureTypeOfRef(); j++)
//        {
//            _resultOfURef->setItem(i, j + 1 + _matching3D->getDimensionOfRef(), new QTableWidgetItem(QString::number(_matching3D->getFeatureOfReferenceTree()[unMatchingIndexOfRef[i]][j])));
//            _resultOfURef->item(i, j + 1 + _matching3D->getDimensionOfRef())->setTextAlignment(Qt::AlignCenter);
//        }
//    }
//}
