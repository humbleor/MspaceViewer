#include "../include/PositionRegistration.h"
#include "../include/RunFuture.h"
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
#include <pcl/kdtree/kdtree_flann.h>

#include <algorithm>
#include <sstream>
#include <cctype>
#include <cstring>
#include <vector>
#include <limits>
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
	// 2D ICP constrained to proper XY rotations. Tree-center points lie in z = 0,
	// where PCL's 3D SVD is rank-deficient and can snap into a reflection.
	const double max_corr_dist = 2.0;
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(target);

	Eigen::Matrix2d R2 = Eigen::Matrix2d::Identity();
	Eigen::Vector2d t2 = Eigen::Vector2d::Zero();

	for (int iter = 0; iter < 50; iter++)
	{
		std::vector<Eigen::Vector2d> src2d, tgt2d;
		for (size_t i = 0; i < source->points.size(); i++)
		{
			const pcl::PointXYZ &pt = source->points[i];
			Eigen::Vector2d p(R2(0, 0) * pt.x + R2(0, 1) * pt.y + t2[0],
			                   R2(1, 0) * pt.x + R2(1, 1) * pt.y + t2[1]);
			std::vector<int> idx(1);
			std::vector<float> sq(1);
			if (kdtree.nearestKSearch(pcl::PointXYZ(p[0], p[1], 0.0f), 1, idx, sq) > 0 &&
			    std::sqrt(sq[0]) < max_corr_dist)
			{
				src2d.push_back(p);
				tgt2d.push_back(Eigen::Vector2d(target->points[idx[0]].x, target->points[idx[0]].y));
			}
		}
		if (src2d.size() < 3) break;

		Eigen::Vector2d cs = Eigen::Vector2d::Zero();
		Eigen::Vector2d ct = Eigen::Vector2d::Zero();
		for (size_t i = 0; i < src2d.size(); i++) { cs += src2d[i]; ct += tgt2d[i]; }
		cs /= (double)src2d.size();
		ct /= (double)src2d.size();

		Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
		for (size_t i = 0; i < src2d.size(); i++)
		{
			Eigen::Vector2d s = src2d[i] - cs;
			Eigen::Vector2d tt = tgt2d[i] - ct;
			H += s * tt.transpose();
		}

		Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::Matrix2d U = svd.matrixU();
		Eigen::Matrix2d V = svd.matrixV();
		Eigen::Matrix2d dR2 = V * U.transpose();
		if (dR2.determinant() < 0)
		{
			Eigen::Matrix2d K2;
			K2 << 1, 0, 0, -1;
			dR2 = V * K2 * U.transpose();
		}
		Eigen::Vector2d dt2 = ct - dR2 * cs;

		R2 = dR2 * R2;
		t2 = dR2 * t2 + dt2;

		if (dt2.norm() < 1e-6 && (dR2 - Eigen::Matrix2d::Identity()).norm() < 1e-6)
			break;
	}

	Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
	R.block<2, 2>(0, 0) = R2;
	refine_transform.first  = Eigen::Vector3d(t2[0], t2[1], 0.0);
	refine_transform.second = R;
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
	params.bestPairsCount    = _bestPairsCount->text().toInt();

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
	runFutureBlocking(future);
}

void PositionRegistration::apply()  { this->done(QDialog::Accepted); }
void PositionRegistration::reject() { this->done(QDialog::Rejected); }
void PositionRegistration::initParam()
{
	_descriptorMinLen->setText("2.0");
	_descriptorMaxLen->setText("50.0");
	_descriptorNearNum->setText("10");
	// 2D tree centers have no normals, so geometric_verify uses Euclidean distance;
	// centers can be a few decimeters off (several points per tree), so use 1.0 m.
	_disGeoVerify->setText("1.0");
	// Score = fraction of target trees covered. Partial overlap can cap it below 1.0
	// (e.g. 25/51) while wrong alignments cover < 0.2; 0.3 separates them.
	_icpThreshold->setText("0.3");
	_bestPairsCount->setText("7");
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

		// Centre each set by its own centroid: PCL stores coordinates as 32-bit
		// float, so UTM-scale values (~2e7 m) lose ~2 m of precision and corrupt
		// the triangle descriptors. Small centred values keep sub-millimetre float
		// precision; the translation is restored below.
		Eigen::Vector2d origin_src = source_centers.rowwise().mean();
		Eigen::Vector2d origin_tgt = target_centers.rowwise().mean();
		Eigen::Matrix2Xd target_centers_w = target_centers;
		target_centers_w.colwise() -= origin_tgt;
		Eigen::Matrix2Xd source_centers_w = source_centers;
		source_centers_w.colwise() -= origin_src;

		logToLoggerPos(logger, tr("Generating triangle descriptors for target...\n"));
		HashRegDescManager* hashReg = nullptr;
		try { hashReg = new HashRegDescManager(config_setting); }
		catch (const std::exception& e) {
			logToLoggerPos(logger, tr("Error creating HashRegDescManager: ") + QString::fromUtf8(e.what()) + "\n");
			return;
		}

		FrameInfo reference_info;
		try {
			hashReg->GenTriDescsFromCenters(target_centers_w, reference_info);
			hashReg->AddTriDescs(reference_info);
		} catch (const std::exception& e) {
			logToLoggerPos(logger, tr("Error generating target descriptors: ") + QString::fromUtf8(e.what()) + "\n");
			delete hashReg; return;
		}
		logToLoggerPos(logger, tr("Target descriptors: %1\n").arg(reference_info.desc_.size()));

		logToLoggerPos(logger, tr("Generating triangle descriptors for source...\n"));
		FrameInfo source_info;
		try { hashReg->GenTriDescsFromCenters(source_centers_w, source_info); }
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
		for (int i = 0; i < source_centers_w.cols(); i++)
		{
			Eigen::Vector3d pt(source_centers_w(0, i), source_centers_w(1, i), 0.0);
			pt = coarse_transform.second * pt + coarse_transform.first;
			pcl::PointXYZ p; p.x = pt[0]; p.y = pt[1]; p.z = pt[2];
			source_cloud->push_back(p);
		}
		for (int i = 0; i < target_centers_w.cols(); i++)
		{
			pcl::PointXYZ p;
			p.x = target_centers_w(0, i); p.y = target_centers_w(1, i); p.z = 0.0;
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

		// Restore original frames: t_true = t_centered + o_t - R * o_s.
		{
			Eigen::Vector3d originSrc3(origin_src[0], origin_src[1], 0.0);
			Eigen::Vector3d originTgt3(origin_tgt[0], origin_tgt[1], 0.0);
			Eigen::Matrix3d finalR = final_matrix.block<3, 3>(0, 0);
			final_matrix.block<3, 1>(0, 3) += originTgt3 - finalR * originSrc3;
		}

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

		// Append axis-angle rotation + translation.
		// This is a 2D UTM registration, so the rotation is a pure rotation about +Z:
		const double kPi = 3.14159265358979323846;
		double angleDeg = std::atan2(final_matrix(1, 0), final_matrix(0, 0)) * 180.0 / kPi;
		if (angleDeg < 0.0) angleDeg += 360.0;
		dataOut << std::setprecision(4)
		        << "rotation: (0.0000, 0.0000, 1.0000):" << angleDeg << " deg\n";
		dataOut << std::setprecision(6)
		        << "translation: (" << final_matrix(0, 3) << ", "
		        << final_matrix(1, 3) << ", " << final_matrix(2, 3) << ") m\n";
		dataOut.close();

		// Output the transformed source tree centers in the target UTM frame.
		// already aligned — no matrix needs to be applied by the user, which
		// avoids the UTM-scale precision/global-shift pitfalls of the matrix.
		{
			Eigen::Matrix3d finalR = final_matrix.block<3, 3>(0, 0);
			Eigen::Vector3d finalT = final_matrix.block<3, 1>(0, 3);
			std::string csvFile = outputDir + "/" + sourcePath.stem().string()
			                    + "_to_" + targetPath.stem().string() + "_transformedSource.csv";
			std::ofstream csvOut(csvFile);
			if (csvOut)
			{
				csvOut << "id,x,y\n";
				csvOut << std::fixed << std::setprecision(6);
				for (int i = 0; i < source_centers.cols(); i++)
				{
					Eigen::Vector3d pt(source_centers(0, i), source_centers(1, i), 0.0);
					pt = finalR * pt + finalT;
					csvOut << source_ids[i] << "," << pt[0] << "," << pt[1] << "\n";
				}
				csvOut.close();
				logToLoggerPos(logger, tr("Transformed source tree centers saved to: ")
					+ QString::fromStdString(csvFile) + "\n");
			}
		}

		// Output best point pairs CSV (in the original UTM frame)
		int numPairs = params.bestPairsCount;
		if (numPairs > 0)
		{
			// Nearest-neighbour error is computed in double precision here: PCL's
			// KdTreeFLANN stores pcl::PointXYZ as 32-bit float, which at UTM scale
			// (~2e7 m) quantises coordinates to ~2 m and reports spurious 0.0 errors.
			struct PointPair { double srcX, srcY, txX, txY, tgtX, tgtY, error; };
			std::vector<PointPair> pairs;
			pairs.reserve(source_centers.cols());

			Eigen::Matrix3d finalRot = final_matrix.block<3, 3>(0, 0);
			Eigen::Vector3d finalTrans = final_matrix.block<3, 1>(0, 3);

			const int ns = static_cast<int>(source_centers.cols());
			const int nt = static_cast<int>(target_centers.cols());

			for (int i = 0; i < ns; i++)
			{
				Eigen::Vector3d pt(source_centers(0, i), source_centers(1, i), 0.0);
				pt = finalRot * pt + finalTrans;

				int best = -1;
				double bestSq = std::numeric_limits<double>::max();
				for (int j = 0; j < nt; j++)
				{
					double dx = pt[0] - target_centers(0, j);
					double dy = pt[1] - target_centers(1, j);
					double sq = dx * dx + dy * dy;
					if (sq < bestSq) { bestSq = sq; best = j; }
				}
				if (best < 0) continue;

				PointPair pp;
				pp.srcX  = source_centers(0, i);
				pp.srcY  = source_centers(1, i);
				pp.txX   = pt[0];
				pp.txY   = pt[1];
				pp.tgtX  = target_centers(0, best);
				pp.tgtY  = target_centers(1, best);
				pp.error = std::sqrt(bestSq);
				pairs.push_back(pp);
			}

			std::sort(pairs.begin(), pairs.end(),
				[](const PointPair& a, const PointPair& b) { return a.error < b.error; });

			int writeCount = std::min(numPairs, static_cast<int>(pairs.size()));
			std::string csvFile = outputDir + "/" + sourcePath.stem().string()
			                    + "_to_" + targetPath.stem().string() + "_bestPairs.csv";
			std::ofstream csvOut(csvFile);
			if (csvOut)
			{
				csvOut << "source_x,source_y,transformed_x,transformed_y,target_x,target_y,error\n";
				csvOut << std::fixed << std::setprecision(6);
				for (int i = 0; i < writeCount; i++)
				{
					csvOut << pairs[i].srcX << "," << pairs[i].srcY << ","
					       << pairs[i].txX << "," << pairs[i].txY << ","
					       << pairs[i].tgtX << "," << pairs[i].tgtY << ","
					       << pairs[i].error << "\n";
				}
				csvOut.close();
				logToLoggerPos(logger, tr("Best point pairs CSV saved to: ") + QString::fromStdString(csvFile) + "\n");
			}

			logToLoggerPos(logger, tr("Best %1 point pairs (by registration error):\n").arg(writeCount));
			for (int i = 0; i < writeCount; i++)
			{
				logToLoggerPos(logger, QString("  #%1: src(%2, %3) -> tx(%4, %5) -> tgt(%6, %7)  err=%8\n")
					.arg(i + 1)
					.arg(pairs[i].srcX, 0, 'f', 4)
					.arg(pairs[i].srcY, 0, 'f', 4)
					.arg(pairs[i].txX, 0, 'f', 4)
					.arg(pairs[i].txY, 0, 'f', 4)
					.arg(pairs[i].tgtX, 0, 'f', 4)
					.arg(pairs[i].tgtY, 0, 'f', 4)
					.arg(pairs[i].error, 0, 'f', 6));
			}
		}

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
