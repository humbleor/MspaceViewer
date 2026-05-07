#pragma once
#include "ui_StemCurveEvaluation.h"
#include "../../TreeEvaluation/include/3DMatching.h"

#include <QtWidgets/QDialog>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QTableWidget>
#include <qprogressdialog.h>

class StemCurveEvaluation :public QDialog, public Ui::StemCurveEvaluation
{
	Q_OBJECT
public:
		explicit StemCurveEvaluation(QWidget* parent = nullptr);
		~StemCurveEvaluation();

		void process(QProgressDialog* progress, std::shared_ptr<Matching3D>& _matching3D, bool output_English);

private slots:
	void apply();
	//选择提取树的参数的输入路径
	void selectedInputFileOfExtractedTree();
	//选择参考树的参数的输入路径
	void selectedInputFileOfReferenceTree();
	//选择匹配索引文件
	void selectedInputFileOfIndex();
	//选择评价参数的输出路径
	void selectedOutputFile();
	//处理树木数据
	void processingData(std::shared_ptr<Matching3D>& _matching3D, bool output_English);

private:
	std::string _fileOfExtractedTree;
	std::string _fileOfReferenceTree;
	std::string _outputPath;

	size_t _resOfExt;
	size_t _resOfRef;
};
