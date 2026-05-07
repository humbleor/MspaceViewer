#pragma once
#include "ui_TreeEvaluation.h"
#include "../../TreeEvaluation/include/3DMatching.h"

#include <QtWidgets/QDialog>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QTableWidget>
#include <qprogressdialog.h>

class TreeEvaluation :public QDialog, public Ui::TreeEvaluation
{
	Q_OBJECT
public:
	explicit TreeEvaluation(QWidget* parent = nullptr);
	~TreeEvaluation();

	void process(QProgressDialog* progress, std::shared_ptr<Matching3D>& _matching3D, bool output_English);

	bool isShowTable();

private slots:
	void apply();
	//选择提取树的参数的输入路径
	void selectedInputFileOfExtractedTree();
	//选择参考树的参数的输入路径
	void selectedInputFileOfReferenceTree();
	//选择匹配索引
	void selectedInputFileOfIndex();
	//选择评价参数的输出路径
	void selectedOutputFile();
	//处理树木数据
	void processingData(std::shared_ptr<Matching3D>& _matching3D, bool output_English);



private:

	//以表格的形式展示匹配结果
	void showResult();


	std::string _fileOfExtractedTree;
	std::string _fileOfReferenceTree;
	std::string _outputPath;

	bool _isShowTable;
	size_t _resOfExt;
	size_t _resOfRef;

};