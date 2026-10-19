// Forest_TLS_Reg CLI. Mirrors MainWindow/src/ForestRegistration.cpp but
// drops Qt. Reads source/target point clouds (LAS/LAZ/PLY/PCD) and an
// optional OpenCV-YAML config, runs HashRegDescManager coarse matching
// + small_gicp fine registration, and writes the 4x4 transformation
// matrix + a registered LAS.

#include <iostream>
#include <string>
#include <vector>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <algorithm>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include "../Forest_TLS_Reg/include/utils/HashRegObj.h"
#include "utils/ForestPairRegistration.h"
#include "../Forest_TLS_Reg/include/utils/Hlp.h"
#include "../Forest_TLS_Reg/include/dst/DST.h"

namespace {

void log(const std::string& s) { std::cout << s << std::endl; }

void printUsage(const char* progName)
{
    std::cout << "ForestRegCLI - Forest point cloud registration" << std::endl;
    std::cout << "Usage: " << progName
              << " --source <source.{las,laz,ply,pcd}> --target <target.{las,laz,ply,pcd}> --output <output_dir>"
              << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  --source <path>    Source point cloud (required)" << std::endl;
    std::cout << "  --target <path>    Target / reference point cloud (required)" << std::endl;
    std::cout << "  --output <path>    Output directory (required)" << std::endl;
    std::cout << "  --matrix-only      Save matrix and quality report without exporting the full cloud" << std::endl;
    std::cout << "  --config <path>    OpenCV-YAML config overriding ConfigSetting (optional)" << std::endl;
}

struct CliArgs {
    std::string source;
    std::string target;
    std::string output;
    std::string config;
    bool matrixOnly = false;
};

bool parseArgs(int argc, char* argv[], CliArgs& p)
{
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto need = [&](const std::string& name) -> const char* {
            if (i + 1 >= argc) { std::cerr << "Missing value after " << name << std::endl; return nullptr; }
            return argv[++i];
        };
        if (a == "--source")  { auto v = need(a); if (!v) return false; p.source = v; }
        else if (a == "--target")  { auto v = need(a); if (!v) return false; p.target = v; }
        else if (a == "--output")  { auto v = need(a); if (!v) return false; p.output = v; }
        else if (a == "--config")  { auto v = need(a); if (!v) return false; p.config = v; }
        else if (a == "--matrix-only") { p.matrixOnly = true; }
        else if (a == "--help" || a == "-h") { return false; }
        else { std::cerr << "Unknown argument: " << a << std::endl; return false; }
    }
    if (p.source.empty() || p.target.empty() || p.output.empty()) {
        std::cerr << "--source, --target, --output are all required" << std::endl;
        return false;
    }
    if (!std::filesystem::exists(p.source)) { std::cerr << "source not found: " << p.source << std::endl; return false; }
    if (!std::filesystem::exists(p.target)) { std::cerr << "target not found: " << p.target << std::endl; return false; }
    std::filesystem::create_directories(p.output);
    return true;
}

std::string toLower(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(), ::tolower);
    return s;
}

bool loadPointCloud(const std::string& filePath,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    std::string ext;
    if (filePath.size() >= 4) ext = toLower(filePath.substr(filePath.size() - 4));

    if (ext == ".las" || ext == ".laz") {
        readTLSData(filePath, cloud);
        return !cloud->empty();
    }
    if (ext == ".ply") {
        pcl::PointCloud<pcl::PointXYZ> tmp;
        if (pcl::io::loadPLYFile(filePath, tmp) < 0) return false;
        *cloud = tmp;
        return true;
    }
    if (ext == ".pcd") {
        pcl::PointCloud<pcl::PointXYZ> tmp;
        if (pcl::io::loadPCDFile(filePath, tmp) < 0) return false;
        *cloud = tmp;
        return true;
    }
    std::cerr << "Unsupported file extension: " << ext << std::endl;
    return false;
}

void saveMatrix(const std::string& path, const Eigen::Matrix4d& T)
{
    std::ofstream out(path);
    if (!out) throw std::runtime_error("Cannot create matrix: " + path);
    out << std::fixed << std::setprecision(6);
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            out << T(r, c) << (c == 3 ? '\n' : '\t');
}

void printMatrix(const Eigen::Matrix4d& T)
{
    std::cout << std::fixed << std::setprecision(6);
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            std::cout << T(r, c) << (c == 3 ? "\n" : "\t");
}

} // namespace

int main(int argc, char* argv[])
{
    try {
    CliArgs args;
    if (!parseArgs(argc, argv, args)) { printUsage(argv[0]); return 1; }

    log("=== Forest Point Cloud Registration ===");
    log("Source:     " + args.source);
    log("Target:     " + args.target);
    log("Output dir: " + args.output);
    if (!args.config.empty()) log("Config:     " + args.config);
    log("------------------------------------------------");

    // --- Step 1: ConfigSetting ---
    ConfigSetting config_setting; // Same defaults as the GUI.
    config_setting.rough_dis_threshold = 0.03;
    config_setting.dist_candi_frames_verify = 1.0;
    config_setting.use_matched_num = 4000;
    config_setting.icp_threshold = 0.25;
    if (!args.config.empty()) {
        log("[config] loading " + args.config);
        ReadParas(args.config, config_setting);
    } else {
        log("[config] none provided, using defaults");
    }

    // --- Step 2: Load point clouds ---
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    log("[load] reading target LAS...");
    if (!loadPointCloud(args.target, target_cloud)) {
        std::cerr << "Failed to load target cloud" << std::endl; return 2;
    }
    log("[load] target: " + std::to_string(target_cloud->size()) + " points");

    log("[load] reading source LAS...");
    if (!loadPointCloud(args.source, source_cloud)) {
        std::cerr << "Failed to load source cloud" << std::endl; return 2;
    }
    log("[load] source: " + std::to_string(source_cloud->size()) + " points");

    const auto result = forest::registerPair(source_cloud, target_cloud, config_setting,
        [](int progress, const std::string& message) { log("[" + std::to_string(progress) + "%] " + message); });
    const std::string reportName = std::filesystem::path(args.source).stem().string() + "_to_" +
        std::filesystem::path(args.target).stem().string() + "_forest_quality.txt";
    forest::savePairReport((std::filesystem::path(args.output) / reportName).string(), result);
    if (!result.accepted) {
        std::cerr << "Forest registration rejected: " << result.reason << std::endl;
        return 3;
    }
    const Eigen::Matrix4d final_matrix = result.transform;

    // --- Step 8: Save matrix + registered cloud ---
    const std::filesystem::path sourcePath(args.source);
    const std::filesystem::path targetPath(args.target);
    const std::string base = sourcePath.stem().string() + "_to_" + targetPath.stem().string();
    const std::string matrix_path = args.output + "/" + base + "_transformationMatrix.txt";
    saveMatrix(matrix_path, final_matrix);

    log("===================================");
    log("Transformation Matrix (" + matrix_path + "):");
    log("===================================");
    printMatrix(final_matrix);
    log("===================================");

    if (args.matrixOnly) return 0;

    pcl::PointCloud<pcl::PointXYZ>::Ptr registered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*source_cloud, *registered, final_matrix.cast<float>());

    // PCD (always works) and LAS (when input was .las/.laz — preserves convention)
    const std::string pcd_path = args.output + "/" + base + "_registered.pcd";
    pcl::io::savePCDFileBinary(pcd_path, *registered);
    log("Registered PCD saved to: " + pcd_path);

    if (toLower(sourcePath.extension().string()) == ".las"
     || toLower(sourcePath.extension().string()) == ".laz") {
        const std::string las_path = args.output + "/" + base + "_registered.las";
        writeLas(las_path, registered);
        log("Registered LAS saved to: " + las_path);
    }

    return 0;
    } catch (const std::exception& e) { std::cerr << e.what() << std::endl; return 2; }
}
