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
    std::cout << "  --config <path>    OpenCV-YAML config overriding ConfigSetting (optional)" << std::endl;
}

struct CliArgs {
    std::string source;
    std::string target;
    std::string output;
    std::string config;
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
    if (!out) { std::cerr << "Failed to open " << path << " for writing" << std::endl; return; }
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
    CliArgs args;
    if (!parseArgs(argc, argv, args)) { printUsage(argv[0]); return 1; }

    log("=== Forest Point Cloud Registration ===");
    log("Source:     " + args.source);
    log("Target:     " + args.target);
    log("Output dir: " + args.output);
    if (!args.config.empty()) log("Config:     " + args.config);
    log("------------------------------------------------");

    // --- Step 1: ConfigSetting ---
    ConfigSetting config_setting; // defaults from struct member initializers
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

    // --- Step 3: Build HashRegDescManager + target descriptors ---
    log("[step1] generating triangle descriptors for target...");
    HashRegDescManager* hashReg = new HashRegDescManager(config_setting);

    FrameInfo reference_info;
    hashReg->GenTriDescs(target_cloud, reference_info);
    hashReg->AddTriDescs(reference_info);
    log("[step1] target descriptors: " + std::to_string(reference_info.desc_.size()));

    // --- Step 4: Source descriptors ---
    log("[step2] generating triangle descriptors for source...");
    FrameInfo source_info;
    hashReg->GenTriDescs(source_cloud, source_info);
    log("[step2] source descriptors: " + std::to_string(source_info.desc_.size()));

    // --- Step 5: Coarse matching ---
    log("[step3] searching for matching triangle pairs...");
    std::pair<int, double> search_result(-1, 0.0);
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> coarse_transform;
    coarse_transform.first << 0, 0, 0;
    coarse_transform.second = Eigen::Matrix3d::Identity();
    std::vector<std::pair<TriDesc, TriDesc>> loop_triangle_pair;

    hashReg->SearchPosition(source_info, search_result, coarse_transform, loop_triangle_pair);

    if (search_result.first == -1) {
        std::cerr << "Error: no matching triangles found, aborting" << std::endl;
        delete hashReg;
        return 3;
    }
    log("[step3] best frame=" + std::to_string(search_result.first)
        + " score=" + std::to_string(search_result.second));

    // --- Step 6: Fine registration (small_gicp) ---
    // ponytail: small_gicp_registration copies every point into Eigen::Vector3d; with
    // 100M+ points this OOMs. Downsample at 0.5m first — coarse alignment from the
    // triangle descriptors above already compensates for the lost resolution.
    log("[step4] downsampling to 0.5m for small_gicp...");
    pcl::PointCloud<pcl::PointXYZ>::Ptr src_ds(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_ds(new pcl::PointCloud<pcl::PointXYZ>);
    {
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setLeafSize(0.5f, 0.5f, 0.5f);
        vg.setInputCloud(source_cloud); vg.filter(*src_ds);
        vg.setInputCloud(target_cloud); vg.filter(*tgt_ds);
    }
    log("[step4] downsampled: src=" + std::to_string(src_ds->size())
        + " tgt=" + std::to_string(tgt_ds->size()));

    log("[step4] fine registration (small_gicp)...");
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> refine_transform;
    small_gicp_registration(src_ds, tgt_ds, refine_transform);

    // --- Step 7: Combine transforms ---
    Eigen::Matrix4d coarse_matrix = Eigen::Matrix4d::Identity();
    coarse_matrix.block<3, 3>(0, 0) = coarse_transform.second;
    coarse_matrix.block<3, 1>(0, 3) = coarse_transform.first;

    Eigen::Matrix4d refine_matrix = Eigen::Matrix4d::Identity();
    refine_matrix.block<3, 3>(0, 0) = refine_transform.second;
    refine_matrix.block<3, 1>(0, 3) = refine_transform.first;

    Eigen::Matrix4d final_matrix = refine_matrix * coarse_matrix;

    delete hashReg;

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
}
