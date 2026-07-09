#include "../include/PositionRegistration.h"
#include <QScrollBar>

// Forest_TLS_Reg headers
#ifndef _HLP_H_Included_
#define _HLP_H_Included_
#include "../../Forest_TLS_Reg/include/utils/Hlp.h"
#endif

#ifndef _DST_H_Included_
#define _DST_H_Included_
#include "../../Forest_TLS_Reg/include/dst/DST.h"
#endif

#ifndef _HashRegObj_H_Included_
#define _HashRegObj_H_Included_
#include "../../Forest_TLS_Reg/include/utils/HashRegObj.h"
#endif

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <algorithm>
#include <sstream>
#include <cctype>
#include <cstring>
#include <vector>
#include <zlib.h>

static void logToLoggerPos(QTextEdit* logger, const QString& text)
{
	if (!logger) return;
	QMetaObject::invokeMethod(logger, [logger, text]() {
		logger->insertPlainText(text);
		logger->verticalScrollBar()->setValue(logger->verticalScrollBar()->maximum());
	}, Qt::QueuedConnection);
}

// ── Minimal ZIP / XLSX reader using zlib ──────────────────────────

// ── Minimal ZIP / XLSX reader using zlib ──────────────────────────
// Manual offset-based reading avoids struct-packing issues across compilers

static inline uint16_t r16(const char* b, size_t off) { uint16_t v; memcpy(&v, b+off, 2); return v; }
static inline uint32_t r32(const char* b, size_t off) { uint32_t v; memcpy(&v, b+off, 4); return v; }

static bool inflate_data(const uint8_t* src, uint32_t srcLen,
                         uint8_t* dst, uint32_t dstLen)
{
	z_stream strm = {};
	strm.next_in = const_cast<uint8_t*>(src);
	strm.avail_in = srcLen;
	strm.next_out = dst;
	strm.avail_out = dstLen;
	int ret = inflateInit2(&strm, -MAX_WBITS);  // raw deflate
	if (ret != Z_OK) return false;
	ret = inflate(&strm, Z_FINISH);
	inflateEnd(&strm);
	return (ret == Z_STREAM_END);
}

static bool zip_extract_entry(const std::string& zipPath,
                              const std::string& entryName,
                              std::vector<uint8_t>& out)
{
	std::ifstream f(zipPath, std::ios::binary);
	if (!f) return false;

	f.seekg(0, std::ios::end);
	size_t fileSize = f.tellg();
	size_t searchStart = (fileSize > 65558) ? (fileSize - 65558) : 0;
	f.seekg(searchStart);
	std::vector<char> tail(fileSize - searchStart);
	f.read(tail.data(), tail.size());

	// Find EOCD signature (0x06054b50)
	size_t eocdOff = 0;
	for (size_t i = 0; i + 22 <= tail.size(); i++) {
		if (r32(tail.data(), i) == 0x06054b50) { eocdOff = searchStart + i; break; }
	}
	if (eocdOff == 0) return false;

	// Read EOCD (manual offsets: 22 bytes)
	f.seekg(eocdOff);
	char eocdBuf[22];
	f.read(eocdBuf, 22);
	uint16_t cdEntries = r16(eocdBuf, 8);   // offset 8
	uint32_t cdOffset   = r32(eocdBuf, 16);  // offset 16

	// Read central directory
	f.seekg(cdOffset);
	for (uint16_t i = 0; i < cdEntries; i++) {
		char cdBuf[46];
		f.read(cdBuf, 46);
		if (r32(cdBuf, 0) != 0x02014b50) return false;

		uint16_t nameLen    = r16(cdBuf, 28);
		uint16_t extraLen   = r16(cdBuf, 30);
		uint16_t commentLen = r16(cdBuf, 32);
		std::string name(nameLen, '\0');
		f.read(&name[0], nameLen);
		f.seekg(extraLen + commentLen, std::ios::cur);

		if (name != entryName) continue;

		uint16_t method     = r16(cdBuf, 10);
		uint32_t compSize   = r32(cdBuf, 20);
		uint32_t uncompSize = r32(cdBuf, 24);
		uint32_t localOff   = r32(cdBuf, 42);

		// Read local file header (manual offsets: 30 bytes)
		f.seekg(localOff);
		char locBuf[30];
		f.read(locBuf, 30);
		if (r32(locBuf, 0) != 0x04034b50) return false;

		uint16_t locNameLen  = r16(locBuf, 26);
		uint16_t locExtraLen = r16(locBuf, 28);
		f.seekg(locNameLen + locExtraLen, std::ios::cur);

		if (compSize == 0) return true;

		std::vector<uint8_t> compData(compSize);
		f.read(reinterpret_cast<char*>(compData.data()), compSize);

		if (method == 0) {
			out.assign(compData.begin(), compData.end());
			return true;
		} else if (method == 8) {
			out.resize(uncompSize);
			return inflate_data(compData.data(), compSize, out.data(), uncompSize);
		}
		return false;
	}
	return false;
}

// Rudimentary XML helper: extract text content between <tag> and </tag>
static std::string xml_inner(const std::string& xml, const std::string& tag)
{
	std::string open = "<" + tag + ">";
	std::string close = "</" + tag + ">";
	size_t s = xml.find(open);
	if (s == std::string::npos) return "";
	s += open.size();
	size_t e = xml.find(close, s);
	if (e == std::string::npos) return "";
	return xml.substr(s, e - s);
}

// Rudimentary XML helper: extract attribute value  attr="value"
static std::string xml_attr(const std::string& xml, const std::string& attr)
{
	std::string pat = attr + "=\"";
	size_t s = xml.find(pat);
	if (s == std::string::npos) return "";
	s += pat.size();
	size_t e = xml.find("\"", s);
	if (e == std::string::npos) return "";
	return xml.substr(s, e - s);
}

// Parse .xlsx and extract id, X, Y columns
static bool parseTreeCentersFromExcel(
	const std::string& path,
	std::vector<int>& ids,
	Eigen::Matrix2Xd& coords)
{
	// Step 1: Extract shared strings
	std::vector<uint8_t> ssBuf;
	if (!zip_extract_entry(path, "xl/sharedStrings.xml", ssBuf))
		return false;
	std::string ssXml(ssBuf.begin(), ssBuf.end());

	// Parse shared strings: <si><t>value</t></si>
	std::vector<std::string> sharedStrings;
	size_t pos = 0;
	while ((pos = ssXml.find("<si>", pos)) != std::string::npos) {
		size_t end = ssXml.find("</si>", pos);
		if (end == std::string::npos) break;
		std::string si = ssXml.substr(pos, end - pos + 5);
		std::string val = xml_inner(si, "t");
		sharedStrings.push_back(val);
		pos = end + 5;
	}

	// Step 2: Extract sheet data
	std::vector<uint8_t> sheetBuf;
	if (!zip_extract_entry(path, "xl/worksheets/sheet1.xml", sheetBuf))
		return false;
	std::string sheetXml(sheetBuf.begin(), sheetBuf.end());

	// Parse rows and cells: <row> ... <c r="A1"><v>0</v></c> ... </row>
	std::vector<double> x_vals, y_vals;
	pos = 0;
	while ((pos = sheetXml.find("<row", pos)) != std::string::npos) {
		size_t rowEnd = sheetXml.find("</row>", pos);
		if (rowEnd == std::string::npos) break;
		std::string rowXml = sheetXml.substr(pos, rowEnd - pos);

		// Find cells in this row
		size_t cellPos = 0;
		std::string cellA, cellB, cellC;  // columns A=id, B=X, C=Y
		while ((cellPos = rowXml.find("<c ", cellPos)) != std::string::npos) {
			size_t cellEnd = rowXml.find(">", cellPos);
			size_t closeTag = rowXml.find("</c>", cellPos);
			if (cellEnd == std::string::npos || closeTag == std::string::npos) break;

			std::string cellTag = rowXml.substr(cellPos, cellEnd - cellPos + 1);
			std::string ref = xml_attr(cellTag, "r");  // e.g. "A1", "B1"
			std::string type = xml_attr(cellTag, "t");  // "s"=shared string, absent=number
			std::string valStr = xml_inner(rowXml.substr(cellPos, closeTag - cellPos + 5), "v");

			if (ref.empty() || valStr.empty()) { cellPos = closeTag + 4; continue; }

			// Get column letter
			char col = ref[0];
			std::string cellVal;
			if (type == "s") {
				// Shared string reference
				int idx = std::stoi(valStr);
				if (idx >= 0 && idx < (int)sharedStrings.size())
					cellVal = sharedStrings[idx];
			} else {
				cellVal = valStr;
			}

			if (col == 'A') cellA = cellVal;
			else if (col == 'B') cellB = cellVal;
			else if (col == 'C') cellC = cellVal;

			cellPos = closeTag + 4;
		}

		// If we have all three columns, try to parse
		if (!cellA.empty() && !cellB.empty() && !cellC.empty()) {
			try {
				int id = std::stoi(cellA);
				double x = std::stod(cellB);
				double y = std::stod(cellC);
				ids.push_back(id);
				x_vals.push_back(x);
				y_vals.push_back(y);
			} catch (const std::exception&) {}
		}
		pos = rowEnd + 6;
	}

	if (ids.empty()) return false;

	int N = static_cast<int>(ids.size());
	coords.resize(2, N);
	for (int i = 0; i < N; i++) {
		coords(0, i) = x_vals[i];
		coords(1, i) = y_vals[i];
	}
	return true;
}

// Forward declaration
static bool parseTreeCenters(const std::string& path, std::vector<int>& ids, Eigen::Matrix2Xd& coords);

// Auto-detect file type and parse (supports .txt, .csv, .xlsx)
static bool parseTreeCentersAuto(
	const std::string& path,
	std::vector<int>& ids,
	Eigen::Matrix2Xd& coords)
{
	std::string ext;
	size_t dot = path.rfind('.');
	if (dot != std::string::npos) {
		ext = path.substr(dot);
		for (auto& c : ext) c = std::tolower((unsigned char)c);
	}

	if (ext == ".xlsx")
		return parseTreeCentersFromExcel(path, ids, coords);
	else
		return parseTreeCenters(path, ids, coords);
}

static bool parseTreeCenters(
	const std::string& path,
	std::vector<int>& ids,
	Eigen::Matrix2Xd& coords)
{
	std::ifstream file(path);
	if (!file.is_open())
		return false;

	std::vector<double> x_vec, y_vec;
	std::string line;

	while (std::getline(file, line))
	{
		size_t start = 0;
		while (start < line.size() && std::isspace(static_cast<unsigned char>(line[start])))
			start++;
		size_t end = line.size();
		while (end > start && std::isspace(static_cast<unsigned char>(line[end - 1])))
			end--;
		if (start >= end) continue;

		if (line[start] == '#' || line[start] == '%') continue;

		std::string content = line.substr(start, end - start);
		bool hasComma = (content.find(',') != std::string::npos);

		std::vector<std::string> tokens;
		if (hasComma)
		{
			std::istringstream ss(content);
			std::string token;
			while (std::getline(ss, token, ','))
			{
				size_t ts = 0, te = token.size();
				while (ts < te && std::isspace(static_cast<unsigned char>(token[ts]))) ts++;
				while (te > ts && std::isspace(static_cast<unsigned char>(token[te - 1]))) te--;
				if (ts < te) tokens.push_back(token.substr(ts, te - ts));
			}
		}
		else
		{
			std::istringstream ss(content);
			std::string token;
			while (ss >> token) tokens.push_back(token);
		}

		if (tokens.size() < 3) continue;

		try {
			ids.push_back(std::stoi(tokens[0]));
			x_vec.push_back(std::stod(tokens[1]));
			y_vec.push_back(std::stod(tokens[2]));
		}
		catch (const std::exception&) { continue; }
	}

	file.close();

	int N = static_cast<int>(ids.size());
	coords.resize(2, N);
	for (int i = 0; i < N; i++)
	{
		coords(0, i) = x_vec[i];
		coords(1, i) = y_vec[i];
	}
	return N > 0;
}

static void icp_registration_2d(
	pcl::PointCloud<pcl::PointXYZ>::Ptr &source,
	pcl::PointCloud<pcl::PointXYZ>::Ptr &target,
	std::pair<Eigen::Vector3d, Eigen::Matrix3d> &refine_transform)
{
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setMaxCorrespondenceDistance(2.0);
	icp.setMaximumIterations(50);
	icp.setTransformationEpsilon(1e-8);
	icp.setEuclideanFitnessEpsilon(1);
	icp.setInputSource(source);
	icp.setInputTarget(target);

	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);

	std::cout << "ICP hasConverged: " << icp.hasConverged()
	          << " score: " << icp.getFitnessScore() << std::endl;

	Eigen::Matrix4f trans = icp.getFinalTransformation();
	matrix_to_pair(trans, refine_transform);
}

PositionRegistration::PositionRegistration(QWidget* parent)
	:QDialog(parent), Ui::PositionRegistration()
{
	this->setupUi(this);
	connect(_selectInputFileOfSource, &QPushButton::clicked, this, &PositionRegistration::selectInputFileOfSource);
	connect(_selectInputFileOfTarget, &QPushButton::clicked, this, &PositionRegistration::selectInputFileOfTarget);
	connect(_selectOutputDir, &QPushButton::clicked, this, &PositionRegistration::selectOutputDir);
	connect(_action_OK, &QPushButton::clicked, this, &PositionRegistration::apply);
	connect(_action_cancel, &QPushButton::clicked, this, &PositionRegistration::reject);
	initParam();
}

PositionRegistration::~PositionRegistration() {}

void PositionRegistration::selectInputFileOfSource()
{
	QStringList fileTypes;
	fileTypes << "Data Files (*.txt *.csv *.xlsx)"
	          << "Text Files (*.txt *.csv)"
	          << "Excel Files (*.xlsx)"
	          << "TXT Files (*.txt)"
	          << "CSV Files (*.csv)";
	QString file = QFileDialog::getOpenFileName(this, tr("Select Source File"), "", fileTypes.join(";;"));
	if (file.isEmpty()) return;
	_inputFileOfSource->setText(file);
}

void PositionRegistration::selectInputFileOfTarget()
{
	QStringList fileTypes;
	fileTypes << "Data Files (*.txt *.csv *.xlsx)"
	          << "Text Files (*.txt *.csv)"
	          << "Excel Files (*.xlsx)"
	          << "TXT Files (*.txt)"
	          << "CSV Files (*.csv)";
	QString file = QFileDialog::getOpenFileName(this, tr("Select Target File"), "", fileTypes.join(";;"));
	if (file.isEmpty()) return;
	_inputFileOfTarget->setText(file);
}

void PositionRegistration::selectOutputDir()
{
	QString dir = QFileDialog::getExistingDirectory(this, tr("Select Output Directory"), "");
	if (dir.isEmpty()) return;
	_outputFileOfDir->setText(dir);
}

void PositionRegistration::executeRegistration(QProgressDialog* progress, QTextEdit* logger)
{
	if (_inputFileOfSource->text().isEmpty() ||
		_inputFileOfTarget->text().isEmpty() ||
		_outputFileOfDir->text().isEmpty()) return;

	PositionRegParams params;
	params.sourceFile = _inputFileOfSource->text();
	params.targetFile = _inputFileOfTarget->text();
	params.outputDir = _outputFileOfDir->text();
	params.descriptorMinLen  = _descriptorMinLen->text().toDouble();
	params.descriptorMaxLen  = _descriptorMaxLen->text().toDouble();
	params.descriptorNearNum = _descriptorNearNum->text().toInt();
	params.disGeoVerify      = _disGeoVerify->text().toDouble();
	params.icpThreshold      = _icpThreshold->text().toDouble();

	if (progress)
	{
		progress->setLabelText(tr("Performing position-based registration ..."));
		progress->setWindowFlags(progress->windowFlags() & ~Qt::WindowCloseButtonHint & ~Qt::WindowContextHelpButtonHint);
		progress->setCancelButton(nullptr);
		progress->show();
		progress->raise();
	}

	auto self = shared_from_this();
	QFuture<void> future = QtConcurrent::run([self, params, logger]() {
		self->registration(params, logger);
	});
	while (!future.isFinished())
	{
		if (progress) { progress->setValue(progress->value() + 1); QApplication::processEvents(); }
	}
}

void PositionRegistration::apply()  { this->done(QDialog::Accepted); }
void PositionRegistration::reject() { this->done(QDialog::Rejected); }
void PositionRegistration::initParam()
{
	_descriptorMinLen->setText("2.0");
	_descriptorMaxLen->setText("50.0");
	_descriptorNearNum->setText("10");
	_disGeoVerify->setText("0.3");
	_icpThreshold->setText("0.5");
}

void PositionRegistration::registration(PositionRegParams params, QTextEdit* logger)
{
	try {
		std::string sourceFile = params.sourceFile.toStdString();
		std::string targetFile = params.targetFile.toStdString();
		std::string outputDir = params.outputDir.toStdString();
		ConfigSetting config_setting;
		config_setting.descriptor_min_len  = params.descriptorMinLen;
		config_setting.descriptor_max_len  = params.descriptorMaxLen;
		config_setting.descriptor_near_num = params.descriptorNearNum;
		config_setting.dis_geo_verify      = params.disGeoVerify;
		config_setting.icp_threshold       = params.icpThreshold;

		logToLoggerPos(logger, tr("Parsing target file: ") + QString::fromStdString(targetFile) + "\n");
		std::vector<int> target_ids;
		Eigen::Matrix2Xd target_centers;
		if (!parseTreeCentersAuto(targetFile, target_ids, target_centers))
			{ logToLoggerPos(logger, tr("Error: Failed to parse target file!\n")); return; }
		logToLoggerPos(logger, tr("Target tree centers: %1\n").arg(target_centers.cols()));

		logToLoggerPos(logger, tr("Parsing source file: ") + QString::fromStdString(sourceFile) + "\n");
		std::vector<int> source_ids;
		Eigen::Matrix2Xd source_centers;
		if (!parseTreeCentersAuto(sourceFile, source_ids, source_centers))
			{ logToLoggerPos(logger, tr("Error: Failed to parse source file!\n")); return; }
		logToLoggerPos(logger, tr("Source tree centers: %1\n").arg(source_centers.cols()));

		logToLoggerPos(logger, tr("Generating triangle descriptors for target...\n"));
		HashRegDescManager* hashReg = nullptr;
		try { hashReg = new HashRegDescManager(config_setting); }
		catch (const std::exception& e) {
			logToLoggerPos(logger, tr("Error creating HashRegDescManager: ") + QString::fromUtf8(e.what()) + "\n");
			return;
		}

		FrameInfo reference_info;
		try {
			hashReg->GenTriDescsFromCenters(target_centers, reference_info);
			hashReg->AddTriDescs(reference_info);
		} catch (const std::exception& e) {
			logToLoggerPos(logger, tr("Error generating target descriptors: ") + QString::fromUtf8(e.what()) + "\n");
			delete hashReg; return;
		}
		logToLoggerPos(logger, tr("Target descriptors: %1\n").arg(reference_info.desc_.size()));

		logToLoggerPos(logger, tr("Generating triangle descriptors for source...\n"));
		FrameInfo source_info;
		try { hashReg->GenTriDescsFromCenters(source_centers, source_info); }
		catch (const std::exception& e) {
			logToLoggerPos(logger, tr("Error generating source descriptors: ") + QString::fromUtf8(e.what()) + "\n");
			delete hashReg; return;
		}
		logToLoggerPos(logger, tr("Source descriptors: %1\n").arg(source_info.desc_.size()));

		logToLoggerPos(logger, tr("Searching for matching triangle pairs...\n"));
		std::pair<int, double> search_result(-1, 0);
		std::pair<Eigen::Vector3d, Eigen::Matrix3d> coarse_transform;
		coarse_transform.first << 0, 0, 0;
		coarse_transform.second = Eigen::Matrix3d::Identity();
		std::vector<std::pair<TriDesc, TriDesc>> loop_triangle_pair;

		try { hashReg->SearchPosition(source_info, search_result, coarse_transform, loop_triangle_pair); }
		catch (const std::exception& e) {
			logToLoggerPos(logger, tr("Error during search: ") + QString::fromUtf8(e.what()) + "\n");
			delete hashReg; return;
		}

		if (search_result.first == -1)
		{
			logToLoggerPos(logger, tr("Warning: No matching triangles found. Registration failed.\n"));
			delete hashReg; return;
		}
		logToLoggerPos(logger, tr("Match found! Frame ID: %1, Score: %2\n")
			.arg(search_result.first).arg(search_result.second, 0, 'f', 4));

		pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		for (int i = 0; i < source_centers.cols(); i++)
		{
			Eigen::Vector3d pt(source_centers(0, i), source_centers(1, i), 0.0);
			pt = coarse_transform.second * pt + coarse_transform.first;
			pcl::PointXYZ p; p.x = pt[0]; p.y = pt[1]; p.z = pt[2];
			source_cloud->push_back(p);
		}
		for (int i = 0; i < target_centers.cols(); i++)
		{
			pcl::PointXYZ p;
			p.x = target_centers(0, i); p.y = target_centers(1, i); p.z = 0.0;
			target_cloud->push_back(p);
		}
		logToLoggerPos(logger, tr("Source cloud: %1, Target cloud: %2\n")
			.arg(source_cloud->size()).arg(target_cloud->size()));

		logToLoggerPos(logger, tr("Performing ICP fine registration...\n"));
		std::pair<Eigen::Vector3d, Eigen::Matrix3d> refine_transform;
		try { icp_registration_2d(source_cloud, target_cloud, refine_transform); }
		catch (const std::exception& e) {
			logToLoggerPos(logger, tr("Error during ICP refinement: ") + QString::fromUtf8(e.what()) + "\n");
			delete hashReg; return;
		}
		logToLoggerPos(logger, tr("ICP refinement completed.\n"));

		Eigen::Matrix4d final_matrix = Eigen::Matrix4d::Identity();
		Eigen::Matrix4d coarse_matrix = Eigen::Matrix4d::Identity();
		coarse_matrix.block<3, 3>(0, 0) = coarse_transform.second;
		coarse_matrix.block<3, 1>(0, 3) = coarse_transform.first;
		Eigen::Matrix4d refine_matrix = Eigen::Matrix4d::Identity();
		refine_matrix.block<3, 3>(0, 0) = refine_transform.second;
		refine_matrix.block<3, 1>(0, 3) = refine_transform.first;
		final_matrix = refine_matrix * coarse_matrix;

		std::filesystem::path sourcePath(sourceFile), targetPath(targetFile);
		std::string outFile = outputDir + "/" + sourcePath.stem().string()
		                    + "_to_" + targetPath.stem().string() + "_transformationMatrix.txt";
		std::ofstream dataOut(outFile);
		if (!dataOut)
			{ logToLoggerPos(logger, tr("Failed to create transformation matrix file!\n")); delete hashReg; return; }

		dataOut << std::fixed << std::setprecision(6);
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				dataOut << final_matrix(i, j);
				if (j < 3) dataOut << "\t";
			}
			dataOut << "\n";
		}
		dataOut.close();

		logToLoggerPos(logger, tr("Transformation Matrix (4x4):\n"));
		for (int i = 0; i < 4; i++)
		{
			QString line;
			for (int j = 0; j < 4; j++)
				line += QString("%1").arg(final_matrix(i, j), 16, 'f', 6);
			logToLoggerPos(logger, line + "\n");
		}
		logToLoggerPos(logger, tr("Transform matrix saved to: ") + QString::fromStdString(outFile) + "\n");
		logToLoggerPos(logger, tr("===================================\n"));
		logToLoggerPos(logger, tr("Position-based registration completed!\n"));

		delete hashReg;
	} catch (...) {
		logToLoggerPos(logger, tr("Unknown exception occurred during registration!\n"));
	}
}
