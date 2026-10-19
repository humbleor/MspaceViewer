#include "Preparation.h"
#include "RelationshipConstruction.h"
#include "IntervalStabbing.h"
#include "Decoupling.h"
#include "Generator.h"
#include "../include/GravityExperiment.h"
#include "../include/NoiseExperiment.h"
#include "../include/OutlierStageExperiment.h"
#include "../include/ThreeDMatchExperiment.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <stdio.h>
#include <boost/thread/thread.hpp>

#include <pcl/visualization/cloud_viewer.h>
#include <iostream>//标准C++库中的输入输出类相关头文件。
#include <pcl/io/pcd_io.h>//pcd 读写类相关的头文件。
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h> //PCL中支持的点类型头文件。
#include <boost/random.hpp>  //高斯噪点测试
#include <fstream>  
#include <string>  
#include <vector> 
#include <iomanip>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/distances.h>

#include <pcl/registration/icp.h>

#include <pcl/common/transforms.h>
using namespace std;

namespace
{
int runHorizontalTranslationSimulationTest()
{
    constexpr std::size_t pointCount = 3000;
    constexpr double outlierRatio = 0.95;
    constexpr double noiseSigma = 0.005;
    constexpr float resolution = 0.1f;
    constexpr unsigned int randomSeed = 20260905U;

    constexpr double yawDegrees = 25.0;
    constexpr double tx = 1.5;
    constexpr double ty = -2.0;
    constexpr double tz = 0.35;

    const double yaw = yawDegrees * M_PI / 180.0;
    const std::size_t inlierCount =
        pointCount - static_cast<std::size_t>(pointCount * outlierRatio);

    Eigen::Matrix4d groundTruth = Eigen::Matrix4d::Identity();
    groundTruth(0, 0) = std::cos(yaw);
    groundTruth(0, 1) = -std::sin(yaw);
    groundTruth(1, 0) = std::sin(yaw);
    groundTruth(1, 1) = std::cos(yaw);
    groundTruth(0, 3) = tx;
    groundTruth(1, 3) = ty;
    groundTruth(2, 3) = tz;

    std::mt19937 generator(randomSeed);
    std::uniform_real_distribution<double> coordinateDistribution(-1.0, 1.0);
    std::normal_distribution<double> noiseDistribution(0.0, noiseSigma);

    PointCloudPtr source(new pcl::PointCloud<pcl::PointXYZ>);
    PointCloudPtr target(new pcl::PointCloud<pcl::PointXYZ>);
    source->reserve(pointCount);
    target->reserve(pointCount);

    pcl::Correspondences correspondences;
    correspondences.reserve(pointCount);

    for (std::size_t i = 0; i < pointCount; ++i)
    {
        Eigen::Vector3d basePoint(
            coordinateDistribution(generator),
            coordinateDistribution(generator),
            coordinateDistribution(generator));

        Eigen::Vector3d sourcePoint = basePoint;
        sourcePoint.x() += noiseDistribution(generator);
        sourcePoint.y() += noiseDistribution(generator);
        sourcePoint.z() += noiseDistribution(generator);

        Eigen::Vector3d targetPoint;
        if (i < inlierCount)
        {
            targetPoint = basePoint;
        }
        else
        {
            targetPoint = Eigen::Vector3d(
                coordinateDistribution(generator),
                coordinateDistribution(generator),
                coordinateDistribution(generator));
        }

        targetPoint.x() += noiseDistribution(generator);
        targetPoint.y() += noiseDistribution(generator);
        targetPoint.z() += noiseDistribution(generator);
        targetPoint = groundTruth.block<3, 3>(0, 0) * targetPoint +
            groundTruth.block<3, 1>(0, 3);

        source->emplace_back(
            static_cast<float>(sourcePoint.x()),
            static_cast<float>(sourcePoint.y()),
            static_cast<float>(sourcePoint.z()));
        target->emplace_back(
            static_cast<float>(targetPoint.x()),
            static_cast<float>(targetPoint.y()),
            static_cast<float>(targetPoint.z()));

        pcl::Correspondence correspondence;
        correspondence.index_query = static_cast<int>(i);
        correspondence.index_match = static_cast<int>(i);
        correspondence.distance = 0.0f;
        correspondences.emplace_back(correspondence);
    }

    source->width = static_cast<std::uint32_t>(source->size());
    source->height = 1;
    source->is_dense = true;
    target->width = static_cast<std::uint32_t>(target->size());
    target->height = 1;
    target->is_dense = true;

    Decoupling decoupling(source, target, correspondences, resolution);
    const double estimatedLength = decoupling.getHorizontalTranslationLength();
    const double expectedLength = std::hypot(tx, ty);
    const double oldFormulaLength = std::sqrt(estimatedLength);

    const Eigen::Matrix4d estimatedTransformation = decoupling.getTransformation();
    const Eigen::Matrix3d rotationDelta =
        estimatedTransformation.block<3, 3>(0, 0) *
        groundTruth.block<3, 3>(0, 0).transpose();
    const double rotationCosine = std::clamp(
        (rotationDelta.trace() - 1.0) * 0.5, -1.0, 1.0);
    const double rotationError = std::acos(rotationCosine) * 180.0 / M_PI;
    const double translationError =
        (estimatedTransformation.block<3, 1>(0, 3) -
            groundTruth.block<3, 1>(0, 3)).norm();

    const bool lengthPassed =
        std::isfinite(estimatedLength) &&
        std::fabs(estimatedLength - expectedLength) <= resolution;

    std::cout << std::fixed << std::setprecision(6)
        << "\n=== Horizontal translation length simulation test ===\n"
        << "seed: " << randomSeed << "\n"
        << "N: " << pointCount << "\n"
        << "outlier ratio: " << outlierRatio << "\n"
        << "noise sigma: " << noiseSigma << "\n"
        << "GT yaw (deg): " << yawDegrees << "\n"
        << "GT translation: [" << tx << ", " << ty << ", " << tz << "]\n"
        << "GT horizontal length: " << expectedLength << "\n"
        << "estimated horizontal length: " << estimatedLength << "\n"
        << "absolute length error: " << std::fabs(estimatedLength - expectedLength) << "\n"
        << "old sqrt formula result: " << oldFormulaLength << "\n"
        << "rotation error (deg): " << rotationError << "\n"
        << "translation error: " << translationError << "\n"
        << "remaining correspondences: " << decoupling.getCors().size() << "\n"
        << "length test: " << (lengthPassed ? "PASS" : "FAIL") << std::endl;

    return lengthPassed ? 0 : 2;
}
}

// =========================
// pair<int,int> 的哈希
// =========================
struct PairHash
{
	std::size_t operator()(const std::pair<int, int>& p) const
	{
		return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
	}
};

// =========================
// 可视化状态
// =========================
struct ViewerState
{
	pcl::visualization::PCLVisualizer::Ptr viewer;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudS;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudT;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudResultTarget;

	std::vector<pcl::Correspondences> stageCors;
	std::vector<std::string> stageNames;
	pcl::Correspondences corRes;

	int currentStage = -1;
	// -1 : 只显示 corRes
	//  0 : 显示 cors    + corRes
	//  1 : 显示 cors1   + corRes
	//  2 : 显示 cors1-1 + corRes
	//  3 : 显示 cors2   + corRes
	//  4 : 显示 cors3   + corRes
	//  5 : 显示 result_target + sampledCloudT
};

// =========================
// 删除所有旧线条
// =========================
void removeAllLines(
	pcl::visualization::PCLVisualizer::Ptr viewer,
	std::size_t maxLineCount = 50000)
{
	for (std::size_t i = 0; i < maxLineCount; ++i)
	{
		viewer->removeShape("red_line_" + std::to_string(i));
		viewer->removeShape("green_line_" + std::to_string(i));
	}
}

// =========================
// 设置点云透明度
// =========================
void setCloudOpacity(
	pcl::visualization::PCLVisualizer::Ptr viewer,
	const std::string& cloudId,
	double opacity)
{
	viewer->setPointCloudRenderingProperties(
		pcl::visualization::PCL_VISUALIZER_OPACITY,
		opacity,
		cloudId);
}

// =========================
// 添加一条线
// =========================
void addLineWithColor(
	pcl::visualization::PCLVisualizer::Ptr viewer,
	const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudS,
	const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudT,
	const pcl::Correspondence& c,
	const std::string& id,
	double r, double g, double b,
	double opacity,
	double lineWidth)
{
	if (c.index_query < 0 || c.index_query >= static_cast<int>(cloudS->points.size()) ||
		c.index_match < 0 || c.index_match >= static_cast<int>(cloudT->points.size()))
	{
		std::cerr << "Warning: correspondence index out of range: "
			<< c.index_query << ", " << c.index_match << std::endl;
		return;
	}

	viewer->addLine<pcl::PointXYZ>(
		cloudS->points[c.index_query],
		cloudT->points[c.index_match],
		id);

	viewer->setShapeRenderingProperties(
		pcl::visualization::PCL_VISUALIZER_COLOR,
		r, g, b, id);

	viewer->setShapeRenderingProperties(
		pcl::visualization::PCL_VISUALIZER_OPACITY,
		opacity, id);

	viewer->setShapeRenderingProperties(
		pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,
		lineWidth, id);
}

// =========================
// 按当前状态重绘
// =========================
void redraw(ViewerState& state)
{
	auto viewer = state.viewer;

	removeAllLines(viewer);
	viewer->removeShape("status_text");

	const int resultModeIndex = static_cast<int>(state.stageCors.size());

	// 最后一页：显示 result_target + sampledCloudT
	if (state.currentStage == resultModeIndex)
	{
		setCloudOpacity(viewer, "source_cloud", 0.0);
		setCloudOpacity(viewer, "target_cloud", 1.0);
		setCloudOpacity(viewer, "result_cloud", 1.0);

		viewer->addText(
			"Showing: result_target + sampledCloudT",
			20, 20, 18, 0.0, 0.0, 0.0, "status_text");
		return;
	}
	else
	{
		setCloudOpacity(viewer, "source_cloud", 1.0);
		setCloudOpacity(viewer, "target_cloud", 1.0);
		setCloudOpacity(viewer, "result_cloud", 0.0);
	}

	// 先构造 corRes 集合
	std::unordered_set<std::pair<int, int>, PairHash> inlierSet;
	inlierSet.reserve(state.corRes.size());
	for (const auto& c : state.corRes)
	{
		inlierSet.insert({ c.index_query, c.index_match });
	}

	// 只显示 corRes
	if (state.currentStage == -1)
	{
		for (std::size_t i = 0; i < state.corRes.size(); ++i)
		{
			addLineWithColor(
				viewer,
				state.cloudS,
				state.cloudT,
				state.corRes[i],
				"green_line_" + std::to_string(i),
				0.0, 0.7, 0.0,
				0.9,
				2.5);
		}

		viewer->addText(
			"Showing: corRes only",
			20, 20, 18, 0.0, 0.0, 0.0, "status_text");
		return;
	}

	// 显示当前阶段 + corRes
	const auto& currentCors = state.stageCors[state.currentStage];
	const std::string& currentName = state.stageNames[state.currentStage];

	// 只有 cors2 阶段，红线线宽与绿线一样粗
	double redLineWidth = 1.0;
	if (currentName == "cors2")
	{
		redLineWidth = 2.5;
	}

	// 先画红线：当前阶段里但不属于 corRes 的对应关系
	std::size_t redCount = 0;
	for (const auto& c : currentCors)
	{
		if (inlierSet.count({ c.index_query, c.index_match }) == 0)
		{
			addLineWithColor(
				viewer,
				state.cloudS,
				state.cloudT,
				c,
				"red_line_" + std::to_string(redCount++),
				1.0, 0.0, 0.0,
				0.25,
				redLineWidth);
		}
	}

	// 再画绿线：corRes
	for (std::size_t i = 0; i < state.corRes.size(); ++i)
	{
		addLineWithColor(
			viewer,
			state.cloudS,
			state.cloudT,
			state.corRes[i],
			"green_line_" + std::to_string(i),
			0.0, 0.7, 0.0,
			0.9,
			2.5);
	}

	viewer->addText(
		"Showing: " + currentName + " + corRes",
		20, 20, 18, 0.0, 0.0, 0.0, "status_text");
}

// =========================
// 键盘回调
// n / SPACE：下一页
// r         ：回到 corRes only
// =========================
void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* cookie)
{
	if (!event.keyDown())
		return;

	ViewerState* state = static_cast<ViewerState*>(cookie);
	if (!state)
		return;

	std::string key = event.getKeySym();
	const int resultModeIndex = static_cast<int>(state->stageCors.size());

	if (key == "n" || key == "space")
	{
		if (state->currentStage == -1)
		{
			state->currentStage = 0;
		}
		else if (state->currentStage < resultModeIndex)
		{
			state->currentStage += 1;
		}
		else
		{
			state->currentStage = -1;
		}

		redraw(*state);
	}
	else if (key == "r")
	{
		state->currentStage = -1;
		redraw(*state);
	}
}

// =========================
// 主可视化函数
// 内部自动生成 cors1-1
// =========================
void visualizeStagesWithSwitch(
	const pcl::PointCloud<pcl::PointXYZ>::Ptr& sampledCloudS,
	const pcl::PointCloud<pcl::PointXYZ>::Ptr& sampledCloudT,
	const pcl::PointCloud<pcl::PointXYZ>::Ptr& result_target,
	const pcl::Correspondences& cors,
	const pcl::Correspondences& cors1,
	const pcl::Correspondences& cors2,
	const pcl::Correspondences& cors3,
	const pcl::Correspondences& corRes)
{
	ViewerState state;
	state.viewer.reset(new pcl::visualization::PCLVisualizer("Correspondence Viewer"));
	state.cloudS = sampledCloudS;
	state.cloudT = sampledCloudT;
	state.cloudResultTarget = result_target;
	state.corRes = corRes;

	// 在函数内部构造 cors1-1：从 cors1 中抽取一半
	pcl::Correspondences cors1_1;
	cors1_1.reserve((cors1.size() + 1) / 2);
	for (std::size_t i = 0; i < cors1.size(); i += 2)
	{
		cors1_1.push_back(cors1[i]);
	}

	// 按显示顺序组织阶段
	state.stageCors = { cors, cors1, cors1_1, cors2, cors3 };
	state.stageNames = { "cors", "cors1", "cors1-1", "cors2", "cors3" };
	state.currentStage = -1;

	auto viewer = state.viewer;

	// 白底
	viewer->setBackgroundColor(1.0, 1.0, 1.0);

	// 点云颜色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
		sourceColor(sampledCloudS, 255, 165, 0); // 橙色

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
		targetColor(sampledCloudT, 0, 0, 255);   // 蓝色

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
		resultColor(result_target, 255, 165, 0); // 与 source 同色

	viewer->addPointCloud<pcl::PointXYZ>(sampledCloudS, sourceColor, "source_cloud");
	viewer->addPointCloud<pcl::PointXYZ>(sampledCloudT, targetColor, "target_cloud");
	viewer->addPointCloud<pcl::PointXYZ>(result_target, resultColor, "result_cloud");

	viewer->setPointCloudRenderingProperties(
		pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "source_cloud");
	viewer->setPointCloudRenderingProperties(
		pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "target_cloud");
	viewer->setPointCloudRenderingProperties(
		pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "result_cloud");

	// 初始隐藏 result_target
	setCloudOpacity(viewer, "result_cloud", 0.0);

	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	viewer->addText("Press 'n' or SPACE: next stage", 20, 50, 16, 0.0, 0.0, 0.0, "help_text1");
	viewer->addText("Press 'r': show corRes only", 20, 75, 16, 0.0, 0.0, 0.0, "help_text2");
	viewer->addText("Final mode: result_target + sampledCloudT", 20, 100, 16, 0.0, 0.0, 0.0, "help_text3");

	viewer->registerKeyboardCallback(keyboardCallback, static_cast<void*>(&state));

	redraw(state);

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(10);
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
}

int main(int argc, char* argv[])
{
    if (argc == 2 && std::string(argv[1]) == "--horizontal-translation-test")
        return runHorizontalTranslationSimulationTest();
    // 全局优化流程由独立的 GlobalOptimization 可执行程序驱动；
    // 本测试程序只包含 D4PCR 算法与各实验模式。
    if (argc >= 2 && std::string(argv[1]) == "--3dmatch-experiment")
    {
        if (argc > 9)
        {
            std::cerr
                << "Usage: D4PCRTest.exe --3dmatch-experiment "
                << "[correspondence_dir] [extracted_evaluation_dir] "
                << "[resolution] [angle_resolution_deg] [inlier_threshold] "
                << "[output_dir] [final_residual_filter:0|1]"
                << std::endl;
            return 2;
        }
        try
        {
            const std::string correspondenceDirectory = argc >= 3
                ? argv[2]
                : "H:/first_paper/data/VBReg-needing-result/needing-result";
            const std::string evaluationDirectory = argc >= 4
                ? argv[3]
                : "outputs/3dmatch/evaluation";
            const double resolution = argc >= 5 ? std::stod(argv[4]) : 0.1;
            const double angleResolutionDegrees = argc >= 6 ? std::stod(argv[5]) : 5.0;
            const double inlierThreshold = argc >= 7 ? std::stod(argv[6]) : 0.1;
            const std::string outputDirectory = argc >= 8
                ? argv[7]
                : "outputs/3dmatch";
            const bool enableFinalResidualFilter = argc >= 9
                ? std::stoi(argv[8]) != 0
                : false;
            return runThreeDMatchExperiment(
                correspondenceDirectory,
                evaluationDirectory,
                resolution,
                angleResolutionDegrees,
                inlierThreshold,
                outputDirectory,
                enableFinalResidualFilter);
        }
        catch (const std::exception& exception)
        {
            std::cerr << "Invalid 3DMatch experiment argument: "
                << exception.what() << std::endl;
            return 2;
        }
    }
    if (argc >= 2 &&
        std::string(argv[1]) == "--outlier-stage-experiment")
    {
        if (argc > 4)
        {
            std::cerr
                << "Usage: D4PCRTest.exe --outlier-stage-experiment "
                << "[resolution] [angle_resolution_deg]" << std::endl;
            return 2;
        }
        try
        {
            const double resolution = argc >= 3 ? std::stod(argv[2]) : 0.015;
            const double angleResolutionDegrees =
                argc == 4 ? std::stod(argv[3]) : 5.0;
            return runOutlierStageExperiment(
                resolution,
                angleResolutionDegrees);
        }
        catch (const std::exception& exception)
        {
            std::cerr << "Invalid argument: " << exception.what() << std::endl;
            return 2;
        }
    }
    if (argc == 2 &&
        std::string(argv[1]) == "--stage-order-ablation")
    {
        return runStageOrderAblationExperiment();
    }
    if (argc == 2 &&
        std::string(argv[1]) == "--angle-pruning-ablation")
    {
        return runAnglePruningAblationExperiment();
    }
    if (argc == 2 && std::string(argv[1]) == "--test-horizontal-length")
        return runHorizontalTranslationSimulationTest();
    if (argc >= 2 && std::string(argv[1]) == "--gravity-experiment")
    {
        if (argc > 4)
        {
            std::cerr
                << "Usage: D4PCRTest.exe --gravity-experiment "
                << "[outlier_ratio] [resolution]" << std::endl;
            return 2;
        }
        try
        {
            const double outlierRatio =
                argc >= 3 ? std::stod(argv[2]) : 0.95;
            const double resolution =
                argc >= 4 ? std::stod(argv[3]) : 0.1;
            return runGravitySensitivityExperiment(
                outlierRatio,
                resolution);
        }
        catch (const std::exception& exception)
        {
            std::cerr
                << "Invalid gravity experiment argument: "
                << exception.what() << std::endl;
            return 2;
        }
    }

    if (argc >= 2 && std::string(argv[1]) == "--noise-experiment")
    {
        if (argc > 4)
        {
            std::cerr
                << "Usage: D4PCRTest.exe --noise-experiment "
                << "[outlier_ratio] [resolution]" << std::endl;
            return 2;
        }
        try
        {
            const double outlierRatio =
                argc >= 3 ? std::stod(argv[2]) : 0.95;
            const double resolution =
                argc >= 4 ? std::stod(argv[3]) : 0.015;
            return runNoiseSensitivityExperiment(outlierRatio, resolution);
        }
        catch (const std::exception& exception)
        {
            std::cerr
                << "Invalid noise experiment argument: "
                << exception.what() << std::endl;
            return 2;
        }
    }
	//PointCloudPtr error = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	//PointCloudPtr truth = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	//pcl::io::loadPCDFile<pcl::PointXYZ>("seq04-70-error.pcd", *error);
	//pcl::io::loadPCDFile<pcl::PointXYZ>("seq04-70-truth.pcd", *truth);
	//std::vector<float> distanceSet;
	//for (size_t i = 0; i < error->points.size(); i++)
	//{
	//	distanceSet.emplace_back(pcl::euclideanDistance(error->points[i], truth->points[i]));
	//}

	//std::ofstream datafile;
	//datafile.open("error.txt", std::ofstream::out);
	//for (size_t i = 0; i < distanceSet.size(); i++)
	//{
	//	datafile << error->points[i].x << " " << error->points[i].y << " " << error->points[i].z << " " << distanceSet[i] << std::endl;
	//}
	//datafile.close();

	float resolution = 0.1f;
	bool center = false;

	// 默认入口（无子命令时）的输入点云：可用
	//   D4PCRTest.exe <source> <target> [resolution] [center:0|1]
	// 覆盖，支持 pcd / ply / las / laz。
	std::string sourcePath = "./049/2.pcd";
	std::string targetPath = "./049/1.pcd";
	if (argc >= 3)
	{
		sourcePath = argv[1];
		targetPath = argv[2];
	}
	if (argc >= 4)
		resolution = static_cast<float>(std::stod(argv[3]));
	if (argc >= 5)
		center = std::stoi(argv[4]) != 0;
	PointCloudPtr sampledCloudS(new pcl::PointCloud<pcl::PointXYZ>);
	PointCloudPtr sampledCloudT(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::Correspondences cors;
	//Real World
	std::shared_ptr<RelationshipConstruction> rc =
		std::make_shared<RelationshipConstruction>(
			sourcePath, targetPath, resolution, center);

	cors = rc->getCorrespondences();
	sampledCloudS = rc->getPointCloudS();
	sampledCloudT = rc->getPointCloudT();

	//std::ofstream datafile;
	//datafile.open("G:/first_paper/code/4DOF-Registration-main/data.txt", std::ofstream::out);

	//for (size_t i = 0; i < cors.size(); i++)
	//{
	//	datafile << sampledCloudS->points[cors[i].index_query].x << " "
	//		<< sampledCloudS->points[cors[i].index_query].y << " "
	//		<< sampledCloudS->points[cors[i].index_query].z << " "
	//		<< sampledCloudT->points[cors[i].index_match].x << " "
	//		<< sampledCloudT->points[cors[i].index_match].y << " "
	//		<< sampledCloudT->points[cors[i].index_match].z << std::endl;
	//}
	//datafile.close();

	////Simulation Data
	//unsigned int number = 50000000;
	//double outliersRatio = 0.95; 
	//

	//std::vector<float> errorRSet, errorTSet, timeSet;
	//for (size_t i = 0; i < 50; i++)
	//{
	//	PointCloudPtr sampledCloudS(new pcl::PointCloud<pcl::PointXYZ>);
	//	PointCloudPtr sampledCloudT(new pcl::PointCloud<pcl::PointXYZ>);
	//	Generator generator(number, outliersRatio);
	//	sampledCloudS = generator.getSource();
	//	sampledCloudT = generator.getTarget();
	//	Eigen::Matrix4d transformation = generator.getTransformation();
	//	pcl::Correspondences cors;
	//	pcl::Correspondence cor;
	//	for (size_t i = 0; i < sampledCloudT->points.size(); i++)
	//	{
	//		cor.index_match = i;
	//		cor.index_query = i;
	//		cors.emplace_back(cor);
	//	}
	//	std::cout << cors.size() << std::endl;
	auto timeS = std::chrono::system_clock::now();
	//float threshold = 0.3;
	Decoupling decoupling(sampledCloudS, sampledCloudT, cors, resolution);
	auto timeE = std::chrono::system_clock::now();
	Eigen::Matrix4d transformation_matrix = decoupling.getTransformation();
	//////transformation_matrix(2, 3) = optimalZ;

	//if (center)
	//{
	//	Eigen::Matrix3d R;
	//	Eigen::Vector3d t;
	//	R = transformation_matrix.block<3, 3>(0, 0);
	//	t = transformation_matrix.block<3, 1>(0, 3);

	//	Eigen::Matrix4d transtemp;
	//	transtemp.fill(0.0);

	//	transtemp.block<3, 3>(0, 0) = R;
	//	transtemp.block<3, 1>(0, 3) = - R * rc->getCenterS() + t + rc->getCenterT();
	//	transtemp(3, 3) = 1.0;

	//	transformation_matrix = transtemp;
	//}

	std::cout << "transformation matrix before ICP: \n" << transformation_matrix << std::endl;

	//Eigen::Matrix4d transformation;
	//transformation << -0.1677002098, -0.9848748721, 0.0435674868, 0.7267793869,
	//	0.9857250433, -0.1668480677, 0.0225357773, 8.8133397103,
	//	-0.0149257698, 0.0467248174, 0.9987962819, -1.6471664799,
	//	0   ,         0     ,       0   ,         1;

	//Eigen::Matrix4d inT = Eigen::Matrix4d::Zero();
	//inT = transformation_matrix.inverse();
	//Eigen::Matrix4d deltaT = transformation * inT;

	//Eigen::Matrix3d rota = deltaT.block<3, 3>(0, 0);

	//double trace = rota(0, 0) + rota(1, 1) + rota(2, 2) - 1;
	//if (trace > 2)
	//	trace = 4 - trace;
	////std::cout << "trace:" << trace << std::endl;
	//double errorR = acos(trace / 2);
	//errorR = errorR * 180.0 / M_PI;
	//std::cout << errorR << std::endl;
	//float errorT = std::sqrtf(std::powf(deltaT(0, 3), 2) + std::powf(deltaT(1, 3), 2) + std::powf(deltaT(2, 3), 2));
	//std::cout << errorT << std::endl;

	//	PointCloudPtr corS(new pcl::PointCloud<pcl::PointXYZ>);
	//	PointCloudPtr corT(new pcl::PointCloud<pcl::PointXYZ>);

	//	for (size_t i = 0; i < cors.size(); i++)
	//	{
	//		corS->points.emplace_back(sampledCloudS->points[cors[i].index_query]);
	//		corT->points.emplace_back(sampledCloudT->points[cors[i].index_match]);
	//	}
	//	corS->width = cors.size();
	//	corS->height = 1;
	//	corT->width = cors.size();
	//	corT->height = 1;

	//	PointCloudPtr resultS(new pcl::PointCloud<pcl::PointXYZ>);
	//	pcl::transformPointCloud(*corS, *resultS, transformation);

	//	size_t correctNumber = 0;
	//	for (size_t i = 0; i < cors.size(); i++)
	//	{
	//		if (pcl::euclideanDistance(resultS->points[i], corT->points[i]) <= resolution)
	//		{
	//			correctNumber++;
	//		}
	//	}
	//	std::cout << "correctNumber:" << correctNumber << std::endl;
	//	std::cout << "outlier ratio: " << std::to_string((double)(cors.size() - correctNumber) / (double)cors.size()) << std::endl;

	//	double time = double(std::chrono::duration_cast<std::chrono::microseconds>
	//		(timeE - timeS).count()) / 1000000.0;
	//	std::cout << time << std::endl;
	//	errorRSet.emplace_back(errorR);
	//	errorTSet.emplace_back(errorT);
	//	timeSet.emplace_back(time);
	//}

	//std::ofstream datafile;
	//datafile.open("dataFile.txt", std::ofstream::out);
	//for (size_t i = 0; i < errorRSet.size(); i++)
	//{
	//	datafile << errorRSet[i] << " ";
	//}
	//datafile << std::endl;
	//for (size_t i = 0; i < errorTSet.size(); i++)
	//{
	//	datafile << errorTSet[i] << " ";
	//}
	//datafile << std::endl;
	//for (size_t i = 0; i < timeSet.size(); i++)
	//{
	//	datafile << timeSet[i] << " ";
	//}
	//datafile.close();

	//	// 创建PCLVisualizer对象
	//pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
	//viewer.setBackgroundColor(1.0, 1.0, 1.0); // RGB值为(1,1,1)表示白色

	//// 将点云添加到可视化器
	//viewer.addPointCloud<pcl::PointXYZ>(sampledCloudS, "sample cloudS");

	//// 设置点云的颜色（可选）
	//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.67, 0.0, "sample cloudS");
	//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloudS");

	//// 将点云添加到可视化器
	//viewer.addPointCloud<pcl::PointXYZ>(sampledCloudT, "sample cloudT");

	//// 设置点云的颜色（可选）
	//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.67, 1.0, "sample cloudT");
	//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloudT");

	////Eigen::Matrix4d transformation;
	////transformation << 0.9809000554, - 0.1945041227, 0.0017965474, 1.5854004199,
	////	0.1944882123, 0.9808818581, 0.0067168253, 11.9651981853,
	////	- 0.0030686510, - 0.0062391270, 0.9999758280, - 0.0869550337,
	////	0, 0, 0, 1;

	////auto timeS = std::chrono::system_clock::now();
	////Decoupling decoupling(sampledCloudS, sampledCloudT, cors, resolution);
	////pcl::Correspondences corRes = decoupling.getCors();

	//for (size_t i = 0; i < cors.size(); i += 1)
	//{
	//	viewer.addLine<pcl::PointXYZ>(sampledCloudS->points[cors[i].index_query], sampledCloudT->points[cors[i].index_match], "line" + std::to_string(i));
	//	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "line" + std::to_string(i));
	//	// 设置直线的透明度
	//	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, "line" + std::to_string(i));  // 透明度设置为0.5
	//}

	//pcl::Correspondences corRes = decoupling.getCors();
	//for (size_t i = 0; i < corRes.size(); i += 1)
	//{
	//	viewer.addLine<pcl::PointXYZ>(sampledCloudS->points[cors[i].index_query], sampledCloudT->points[cors[i].index_match], "line" + std::to_string(i));
	//	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "line" + std::to_string(i));
	//	// 设置直线的透明度
	//	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, "line" + std::to_string(i));  // 透明度设置为0.5
	//}



	//// 主循环，直到窗口被关闭
	//while (!viewer.wasStopped())
	//{
	//	viewer.spinOnce(100);
	//}

	//pcl::Correspondences cors1 = decoupling.getCors1();
	//pcl::Correspondences cors2 = decoupling.getCors2();
	//pcl::Correspondences cors3 = decoupling.getCors3();
	//pcl::Correspondences corRes = decoupling.getCors();
	//pcl::PointCloud<pcl::PointXYZ>::Ptr result_target(new pcl::PointCloud<pcl::PointXYZ>);

	//pcl::transformPointCloud(*sampledCloudS, *result_target, decoupling.getTransformation());

	//visualizeStagesWithSwitch(sampledCloudS, sampledCloudT, result_target, cors, cors1, cors2, cors3, corRes);


	//Eigen::Matrix4d transformation_matrix = decoupling.getTransformation();
	//auto timeE = std::chrono::system_clock::now();
	//////transformation_matrix(2, 3) = optimalZ;
	//std::cout << "transformation matrix before ICP: \n" << transformation_matrix << std::endl;

	//Eigen::Matrix4d inT = Eigen::Matrix4d::Zero();
	//inT = transformation_matrix.inverse();
	//Eigen::Matrix4d deltaT = transformation * inT;

	//Eigen::Matrix3d rota = deltaT.block<3, 3>(0, 0);

	//double trace = rota(0, 0) + rota(1, 1) + rota(2, 2) - 1;
	//std::cout << "trace:" << trace << std::endl;
	//double errorR = acos(trace / 2);
	//errorR = errorR * 180.0 / M_PI;
	//std::cout << errorR << " ";
	//double errorT = std::sqrtf(std::powf(deltaT(0, 3), 2) + std::powf(deltaT(1, 3), 2) + std::powf(deltaT(2, 3), 2));
	//std::cout << errorT << " ";
	//double time = double(std::chrono::duration_cast<std::chrono::microseconds>
	//	(timeE - timeS).count()) / 1000000.0;
	//std::cout << time << std::endl;

	//Decoupling decoupling(sampledCloudS, sampledCloudT, cors, resolution);
	//Eigen::Matrix4d transformation = decoupling.getTransformation();
	//std::cout << transformation << std::endl;
}



