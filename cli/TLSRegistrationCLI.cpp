// TLS-TLS registration CLI. Mirrors MainWindow/src/RegistrationForm_TLS.cpp
// but drops Qt dependencies. Reads two LAS files and writes the 4x4
// transformation matrix + a registered source PCD.

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
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/icp.h>
#include <pcl/search/kdtree.h>

#include <Function.h>
#include "PointCloudLoader.h"

namespace {

void log(const std::string& s) { std::cout << s << std::endl; }

void printUsage(const char* progName)
{
    std::cout << "TLSRegistrationCLI - TLS to TLS point cloud registration" << std::endl;
    std::cout << "Usage: " << progName
              << " --source <source.las> --target <target.las> --output <output_dir>"
              << std::endl;
    std::cout << "Options (all optional, default values match GUI):" << std::endl;
    std::cout << "  --sector-num <int>        Sector count for rhombus descriptor (default: 360)" << std::endl;
    std::cout << "  --resolution <float>      Rhombus resolution (default: 0.2)" << std::endl;
    std::cout << "  --max-radius <float>      Maximum neighbor radius (default: 10.0)" << std::endl;
    std::cout << "  --min-radius <float>      Minimum neighbor radius (default: 5.0)" << std::endl;
    std::cout << "  --error-dis <float>       Corresp. dist threshold (default: 0.2)" << std::endl;
    std::cout << "  --error-z <float>         Corresp. Z threshold (default: 0.1)" << std::endl;
    std::cout << "  --error-ang <float>       Corresp. normal angle threshold (default: 30.0)" << std::endl;
    std::cout << "  --points-constrain <int>  ICP min points per cell (default: 20)" << std::endl;
    std::cout << "  --z-constrain <float>     ICP Z threshold per cell (default: 0.2)" << std::endl;
}

struct Params {
    std::string source;
    std::string target;
    std::string output;
    int sector_num = 360;
    float resolution = 0.2f;
    float maxRadius = 10.0f;
    float minRadius = 5.0f;
    float error_dis = 0.2f;
    float error_z = 0.1f;
    float error_ang = 30.0f;
    int pointsConstrain = 20;
    float zConstrain = 0.2f;
};

bool parseArgs(int argc, char* argv[], Params& p)
{
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto need = [&](const std::string& name) -> const char* {
            if (i + 1 >= argc) { std::cerr << "Missing value after " << name << std::endl; return nullptr; }
            return argv[++i];
        };
        if (a == "--source") { auto v = need(a); if (!v) return false; p.source = v; }
        else if (a == "--target") { auto v = need(a); if (!v) return false; p.target = v; }
        else if (a == "--output") { auto v = need(a); if (!v) return false; p.output = v; }
        else if (a == "--sector-num") { auto v = need(a); if (!v) return false; p.sector_num = std::stoi(v); }
        else if (a == "--resolution") { auto v = need(a); if (!v) return false; p.resolution = std::stof(v); }
        else if (a == "--max-radius") { auto v = need(a); if (!v) return false; p.maxRadius = std::stof(v); }
        else if (a == "--min-radius") { auto v = need(a); if (!v) return false; p.minRadius = std::stof(v); }
        else if (a == "--error-dis") { auto v = need(a); if (!v) return false; p.error_dis = std::stof(v); }
        else if (a == "--error-z") { auto v = need(a); if (!v) return false; p.error_z = std::stof(v); }
        else if (a == "--error-ang") { auto v = need(a); if (!v) return false; p.error_ang = std::stof(v); }
        else if (a == "--points-constrain") { auto v = need(a); if (!v) return false; p.pointsConstrain = std::stoi(v); }
        else if (a == "--z-constrain") { auto v = need(a); if (!v) return false; p.zConstrain = std::stof(v); }
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

void saveMatrix(const std::string& path, const Eigen::Matrix4f& T)
{
    std::ofstream out(path);
    if (!out) { std::cerr << "Failed to open " << path << " for writing" << std::endl; return; }
    out << std::fixed << std::setprecision(6);
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            out << T(r, c) << (c == 3 ? '\n' : '\t');
}

} // namespace

int main(int argc, char* argv[])
{
    Params params;
    if (!parseArgs(argc, argv, params)) { printUsage(argv[0]); return 1; }

    log("=== TLS-TLS Point Cloud Registration ===");
    log("Source:     " + params.source);
    log("Target:     " + params.target);
    log("Output dir: " + params.output);
    log("Params: sector=" + std::to_string(params.sector_num)
        + " R=" + std::to_string(params.maxRadius)
        + " r=" + std::to_string(params.minRadius)
        + " dR=" + std::to_string(params.resolution));
    log("------------------------------------------------");

    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    log("[load] reading source LAS...");
    if (!pcutil::loadPointCloud(params.source, source_cloud)) {
        std::cerr << "Failed to load source LAS" << std::endl; return 2;
    }
    log("[load] source: " + std::to_string(source_cloud->size()) + " points");

    log("[load] reading target LAS...");
    if (!pcutil::loadPointCloud(params.target, target_cloud)) {
        std::cerr << "Failed to load target LAS" << std::endl; return 2;
    }
    log("[load] target: " + std::to_string(target_cloud->size()) + " points");

    const int num_sectors = params.sector_num;
    const float R_neighbor = params.maxRadius;
    const float r_neighbor = params.minRadius;
    const float dR = params.resolution;
    const float Step = dR;

    const float corr_canshu1 = params.error_dis;
    const float corr_canshu2 = params.error_z;
    const float corr_canshu3 = params.error_ang;
    const unsigned int icp_canshu1 = static_cast<unsigned int>(params.pointsConstrain);
    const float icp_canshu2 = params.zConstrain;

    log("[step1] computing rhombus sector indices...");
    std::vector<std::vector<int>> target_rhombus_indices =
        compute_rhombus_pointclouds(target_cloud, R_neighbor, num_sectors);
    std::vector<std::vector<int>> source_rhombus_indices =
        compute_rhombus_pointclouds(source_cloud, R_neighbor, num_sectors);

    const int fault_tolerant_iterations = 2;
    std::vector<std::vector<std::vector<pcl::PointXYZ>>> target_rhombus_descriptors(num_sectors);
    std::vector<std::vector<std::vector<pcl::PointXYZ>>> source_rhombus_descriptors(num_sectors);
    const float dTheta = 360.0f / static_cast<float>(num_sectors);
    const float StepScaled = Step * static_cast<float>(sqrt(3.0) / 2.0);

    log("[step2] computing rhombus descriptors per sector (this is the slow part)...");
    for (int i = 0; i < num_sectors; ++i) {
        target_rhombus_descriptors[i] =
            get_rhombus_descriptors(target_cloud, target_rhombus_indices[i], i * dTheta, R_neighbor, StepScaled);
        source_rhombus_descriptors[i] =
            get_rhombus_descriptors(source_cloud, source_rhombus_indices[i], i * dTheta, R_neighbor, StepScaled);
        if ((i + 1) % 60 == 0) {
            log("  descriptor sector " + std::to_string(i + 1) + "/" + std::to_string(num_sectors));
        }
    }

    const int num_combinations = num_sectors * num_sectors;
    const int num_dR = static_cast<int>((R_neighbor - r_neighbor) / dR) + 1;
    std::vector<std::vector<float>> corr_dR(num_dR, std::vector<float>(4));
    std::vector<float> std_dR(num_dR, 10.0f);
    std::vector<std::vector<int>> corr(num_combinations, std::vector<int>(2));
    std::vector<float> score(num_combinations, 10.0f);
    std::vector<float> Diff_Zt_Zs(num_combinations, 0.0f);
    std::vector<float> Diff_Lts(num_combinations, 0.0f);

    const int max_grid_line_num = static_cast<int>(R_neighbor * sin(60.0 * M_PI / 180.0) / StepScaled) + 1;
    const int max_neighbor_size = max_grid_line_num * max_grid_line_num * fault_tolerant_iterations;
    std::vector<float> neighbor_DZ(max_neighbor_size, 100.0f);
    std::vector<float> neighbor_Dist(max_neighbor_size, 100.0f);
    std::vector<float> neighbor_DiffD(max_neighbor_size, 100.0f);

    log("[step3] coarse sector match sweep...");
    for (int ii = 0; ii < num_dR; ++ii) {
        const int count = 0;
        const int grid_line_num =
            static_cast<int>((R_neighbor - ii * dR) * sin(60.0 * M_PI / 180.0) / StepScaled) + 1;
        std::vector<float> diff_Z;
        const int cur_neighbor_size = grid_line_num * grid_line_num * fault_tolerant_iterations;
        if (cur_neighbor_size > max_neighbor_size) {
            neighbor_DZ.assign(cur_neighbor_size, 100.0f);
            neighbor_Dist.assign(cur_neighbor_size, 100.0f);
            neighbor_DiffD.assign(cur_neighbor_size, 100.0f);
        }
        int running_count = 0;
        for (int i = 0; i < num_sectors; ++i) {
            for (int j = 0; j < num_sectors; ++j) {
                diff_Z.clear();
                float sum_dz = 0.0f;
                float sum_dl = 0.0f;
                for (int k = 0; k < grid_line_num; ++k) {
                    for (int l = 0; l < grid_line_num; ++l) {
                        const size_t index =
                            static_cast<size_t>(k) * grid_line_num * fault_tolerant_iterations
                            + static_cast<size_t>(l) * fault_tolerant_iterations;
                        for (int m = 0; m < fault_tolerant_iterations; ++m) {
                            if (grid_line_num - 1 - k - m < 0 || grid_line_num - 1 - l - m < 0) break;
                            const float dz_diff = target_rhombus_descriptors[i][k][l].z
                                - source_rhombus_descriptors[j][grid_line_num - 1 - k - m][grid_line_num - 1 - l - m].z;
                            if (std::isnormal(dz_diff)) {
                                neighbor_DZ[index + m] = dz_diff;
                                neighbor_Dist[index + m] = (target_rhombus_descriptors[i][k][l].x
                                    + source_rhombus_descriptors[j][grid_line_num - 1 - k - m][grid_line_num - 1 - l - m].x
                                    + target_rhombus_descriptors[i][k][l].y
                                    + source_rhombus_descriptors[j][grid_line_num - 1 - k - m][grid_line_num - 1 - l - m].y) / sqrtf(3.0f);
                                neighbor_DiffD[index + m] = fabsf(target_rhombus_descriptors[i][k][l].x
                                    + source_rhombus_descriptors[j][grid_line_num - 1 - k - m][grid_line_num - 1 - l - m].x
                                    - target_rhombus_descriptors[i][k][l].y
                                    - source_rhombus_descriptors[j][grid_line_num - 1 - k - m][grid_line_num - 1 - l - m].y);
                            } else {
                                neighbor_DZ[index + m] = 100.0f;
                                neighbor_Dist[index + m] = 100.0f;
                                neighbor_DiffD[index + m] = 100.0f;
                            }
                        }
                        const auto it = std::min_element(neighbor_DiffD.begin() + index,
                                                         neighbor_DiffD.begin() + index + fault_tolerant_iterations);
                        const size_t min_index = static_cast<size_t>(it - neighbor_DiffD.begin());
                        if (neighbor_DiffD[min_index] <= 0.05f) {
                            sum_dz += neighbor_DZ[min_index];
                            sum_dl += neighbor_Dist[min_index];
                            diff_Z.push_back(neighbor_DZ[min_index]);
                        }
                    }
                }
                if (diff_Z.size() > 10) {
                    Diff_Zt_Zs[running_count] = sum_dz / static_cast<float>(diff_Z.size());
                    Diff_Lts[running_count] = sum_dl / static_cast<float>(diff_Z.size());
                    const float Z_std = calculateStandardDeviation(diff_Z, 0.2f);
                    corr[running_count][0] = i;
                    corr[running_count][1] = j;
                    score[running_count] = Z_std;
                }
                ++running_count;
            }
        }
        const auto smallest = std::min_element(score.begin(), score.end());
        const int nMinIndex = static_cast<int>(std::distance(score.begin(), smallest));
        corr_dR[ii][0] = static_cast<float>(corr[nMinIndex][0]) * dTheta;
        corr_dR[ii][1] = static_cast<float>(corr[nMinIndex][1]) * dTheta;
        corr_dR[ii][2] = Diff_Lts[nMinIndex];
        corr_dR[ii][3] = Diff_Zt_Zs[nMinIndex];
        if (Diff_Lts[nMinIndex] >= (R_neighbor - ii * dR)) corr_dR[ii][2] = R_neighbor - ii * dR;
        if (fabsf(Diff_Lts[nMinIndex] - (R_neighbor - ii * dR)) <= 0.1f) std_dR[ii] = *smallest;
        (void)count;
    }

    const auto lest = std::min_element(std_dR.begin(), std_dR.end());
    const int minIndex = static_cast<int>(std::distance(std_dR.begin(), lest));
    log("[step3] best coarse radius index: " + std::to_string(minIndex));
    log("[step3] corr_dR = (tgt_ang=" + std::to_string(corr_dR[minIndex][0])
        + ", src_ang=" + std::to_string(corr_dR[minIndex][1])
        + ", L=" + std::to_string(corr_dR[minIndex][2])
        + ", dZ=" + std::to_string(corr_dR[minIndex][3]) + ")");

    target_rhombus_descriptors.clear();
    target_rhombus_descriptors.shrink_to_fit();
    source_rhombus_descriptors.clear();
    source_rhombus_descriptors.shrink_to_fit();

    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr sourceOverlapAll(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr targetOverlapAll(new pcl::PointCloud<pcl::PointXYZ>);
    const int src_rhombus_idx = static_cast<int>(corr_dR[minIndex][1] / dTheta);
    const int tgt_rhombus_idx = static_cast<int>(corr_dR[minIndex][0] / dTheta);
    sourceOverlapAll = get_OverlapRhombus_pointclouds(source_cloud, source_rhombus_indices[src_rhombus_idx],
                                                     corr_dR[minIndex][2], corr_dR[minIndex][1]);
    targetOverlapAll = get_OverlapRhombus_pointclouds(target_cloud, target_rhombus_indices[tgt_rhombus_idx],
                                                     corr_dR[minIndex][2], corr_dR[minIndex][0]);

    source_rhombus_indices.clear();
    source_rhombus_indices.shrink_to_fit();
    target_rhombus_indices.clear();
    target_rhombus_indices.shrink_to_fit();

    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setLeafSize(0.05f, 0.05f, 0.05f);
    vg.setInputCloud(sourceOverlapAll); vg.filter(*source);
    vg.setInputCloud(targetOverlapAll); vg.filter(*target);
    log("[step4] overlap voxel-downsampled: src=" + std::to_string(source->size())
        + " tgt=" + std::to_string(target->size()));

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::PointCloud<pcl::Normal>::Ptr source_normals = compute_normal(source, tree);
    pcl::PointCloud<pcl::Normal>::Ptr target_normals = compute_normal(target, tree);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_fpfh = compute_fpfh_feature(source, source_normals, tree);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_fpfh = compute_fpfh_feature(target, target_normals, tree);

    pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> crude_cor_est;
    pcl::CorrespondencesPtr cru_correspondences(new pcl::Correspondences);
    crude_cor_est.setInputSource(source_fpfh);
    crude_cor_est.setInputTarget(target_fpfh);
    crude_cor_est.determineReciprocalCorrespondences(*cru_correspondences);
    log("[step4] reciprocal FPFH correspondences: " + std::to_string(cru_correspondences->size()));

    const Eigen::Vector3f v1_s(std::cos(corr_dR[minIndex][1] * static_cast<float>(M_PI) / 180.0f),
                               std::sin(corr_dR[minIndex][1] * static_cast<float>(M_PI) / 180.0f), 0.0f);
    const Eigen::Vector3f v2_s(std::cos((corr_dR[minIndex][1] + 120.0f) * static_cast<float>(M_PI) / 180.0f),
                               std::sin((corr_dR[minIndex][1] + 120.0f) * static_cast<float>(M_PI) / 180.0f), 0.0f);
    const Eigen::Vector3f v1_t(std::cos(corr_dR[minIndex][0] * static_cast<float>(M_PI) / 180.0f),
                               std::sin(corr_dR[minIndex][0] * static_cast<float>(M_PI) / 180.0f), 0.0f);
    const Eigen::Vector3f v2_t(std::cos((corr_dR[minIndex][0] + 120.0f) * static_cast<float>(M_PI) / 180.0f),
                               std::sin((corr_dR[minIndex][0] + 120.0f) * static_cast<float>(M_PI) / 180.0f), 0.0f);

    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
    int total = 0;
    for (size_t i = 0; i < cru_correspondences->size(); ++i) {
        const auto& c = (*cru_correspondences)[i];
        const Eigen::Vector3f pnt_s(source->points[c.index_query].x, source->points[c.index_query].y, 0.0f);
        const Eigen::Vector3f pnt_t(target->points[c.index_match].x, target->points[c.index_match].y, 0.0f);
        const float d1_s = pnt_s.cross(v1_s).norm();
        const float d2_s = pnt_s.cross(v2_s).norm();
        const float d1_t = pnt_t.cross(v1_t).norm();
        const float d2_t = pnt_t.cross(v2_t).norm();
        const float a = fabsf(d1_s + d1_t - d2_s - d2_t);
        const float b = fabsf(target->points[c.index_match].z - source->points[c.index_query].z - corr_dR[minIndex][3]);
        const float c_ang = fabsf(std::acos(std::min(1.0f, std::max(-1.0f, target_normals->points[c.index_match].normal_z)))
                                  * 180.0f / static_cast<float>(M_PI)
                                  - std::acos(std::min(1.0f, std::max(-1.0f, source_normals->points[c.index_query].normal_z)))
                                    * 180.0f / static_cast<float>(M_PI));
        if (a <= corr_canshu1 && b <= corr_canshu2 && c_ang <= corr_canshu3) {
            correspondences->push_back(c);
            ++total;
        }
    }
    log("[step4] filtered correspondences: " + std::to_string(total));

    Eigen::Matrix4f Transform = Eigen::Matrix4f::Identity();
    const int MAX_RANSAC_ITERATIONS = 10000;
    const int n_total_pairs = total * (total - 1) / 2;
    const int n_ransac = (n_total_pairs > MAX_RANSAC_ITERATIONS) ? MAX_RANSAC_ITERATIONS : n_total_pairs;
    std::vector<Eigen::Matrix4f> Transform_ransac(n_ransac);
    std::vector<float> score_ransac(n_ransac, 100.0f);
    int iteration_ransac = 0;
    for (size_t i = 0; i < correspondences->size(); ++i) {
        const pcl::PointXYZ T1 = target->points[(*correspondences)[i].index_match];
        const pcl::PointXYZ S1 = source->points[(*correspondences)[i].index_query];
        const Eigen::Vector3f normal_s1(source_normals->points[(*correspondences)[i].index_query].normal_x,
                                        source_normals->points[(*correspondences)[i].index_query].normal_y,
                                        source_normals->points[(*correspondences)[i].index_query].normal_z);
        const Eigen::Vector3f normal_t1(target_normals->points[(*correspondences)[i].index_match].normal_x,
                                        target_normals->points[(*correspondences)[i].index_match].normal_y,
                                        target_normals->points[(*correspondences)[i].index_match].normal_z);
        for (size_t j = i + 1; j < correspondences->size(); ++j) {
            if (iteration_ransac >= n_ransac) break;
            const Eigen::Vector3f normal_s2(source_normals->points[(*correspondences)[j].index_query].normal_x,
                                            source_normals->points[(*correspondences)[j].index_query].normal_y,
                                            source_normals->points[(*correspondences)[j].index_query].normal_z);
            const Eigen::Vector3f normal_t2(target_normals->points[(*correspondences)[j].index_match].normal_x,
                                            target_normals->points[(*correspondences)[j].index_match].normal_y,
                                            target_normals->points[(*correspondences)[j].index_match].normal_z);
            const pcl::PointXYZ T2 = target->points[(*correspondences)[j].index_match];
            const pcl::PointXYZ S2 = source->points[(*correspondences)[j].index_query];
            const Eigen::Vector3f source_pnt(S2.x - S1.x, S2.y - S1.y, S2.z - S1.z);
            const Eigen::Vector3f target_pnt(T2.x - T1.x, T2.y - T1.y, T2.z - T1.z);
            const float ds = source_pnt.norm();
            const float dt = target_pnt.norm();
            const float dD = fabsf(dt - ds);
            const float dA = fabsf(std::acos(std::min(1.0f, std::max(-1.0f, normal_s2.dot(normal_s1))))
                                * 180.0f / static_cast<float>(M_PI)
                                - std::acos(std::min(1.0f, std::max(-1.0f, normal_t2.dot(normal_t1))))
                                  * 180.0f / static_cast<float>(M_PI));
            if (ds >= 0.2f && dt >= 0.2f && dD <= 0.1f && dA <= 30.0f) {
                Transform = calculateFourParameterTransformation(S1, S2, T1, T2);
                pcl::PointCloud<pcl::PointXYZ>::Ptr t_cloud(new pcl::PointCloud<pcl::PointXYZ>());
                pcl::transformPointCloud(*source, *t_cloud, Transform);
                score_ransac[iteration_ransac] = ComputeRmse_PCR(t_cloud, target, 0.1f);
                Transform_ransac[iteration_ransac] = Transform;
            }
            ++iteration_ransac;
        }
        if (iteration_ransac >= n_ransac) break;
    }
    const auto lest_ransac = std::min_element(score_ransac.begin(), score_ransac.end());
    const int minIndex_ransac = static_cast<int>(std::distance(score_ransac.begin(), lest_ransac));
    log("[step5] ransac best score: " + std::to_string(score_ransac[minIndex_ransac]));

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_src(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*sourceOverlapAll, *transformed_src, Transform_ransac[minIndex_ransac]);
    const float rr = std::sqrt(Transform_ransac[minIndex_ransac](0, 3) * Transform_ransac[minIndex_ransac](0, 3)
                             + Transform_ransac[minIndex_ransac](1, 3) * Transform_ransac[minIndex_ransac](1, 3));
    float idx = std::atan2(Transform_ransac[minIndex_ransac](1, 3), Transform_ransac[minIndex_ransac](0, 3))
                * 180.0f / static_cast<float>(M_PI) - 60.0f;
    if (idx < 0) idx += 360.0f;

    std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ>>> rhombus_source, rhombus_target;
    rhombus_source = get_PntInrhombus(transformed_src, idx, rr, 0.2f);
    rhombus_target = get_PntInrhombus(targetOverlapAll, idx, rr, 0.2f);

    pcl::PointCloud<pcl::PointXYZ>::Ptr overlapTransformed_src(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr overlap_tgt(new pcl::PointCloud<pcl::PointXYZ>);
    const int cell_n = static_cast<int>(rr * std::sin(60.0 * M_PI / 180.0) / 0.2f) + 1;
    for (int i = 0; i < cell_n; ++i) {
        for (int j = 0; j < cell_n; ++j) {
            if (rhombus_source[i][j].size() >= icp_canshu1 && rhombus_target[i][j].size() >= icp_canshu1) {
                float Z_src = 0.0f, Z_tgt = 0.0f;
                for (const auto& p : rhombus_source[i][j]) Z_src += p.z;
                Z_src /= static_cast<float>(rhombus_source[i][j].size());
                for (const auto& p : rhombus_target[i][j]) Z_tgt += p.z;
                Z_tgt /= static_cast<float>(rhombus_target[i][j].size());
                if (fabsf(Z_tgt - Z_src) <= icp_canshu2) {
                    *overlap_tgt += rhombus_target[i][j];
                    *overlapTransformed_src += rhombus_source[i][j];
                }
            }
        }
    }
    log("[step6] Z-constrained ICP inputs: src=" + std::to_string(overlapTransformed_src->size())
        + " tgt=" + std::to_string(overlap_tgt->size()));

    pcl::PointCloud<pcl::PointNormal>::Ptr source_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    cloud_with_normal(overlapTransformed_src, source_with_normals);
    pcl::PointCloud<pcl::PointNormal>::Ptr target_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    cloud_with_normal(overlap_tgt, target_with_normals);

    pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> p_icp;
    p_icp.setInputSource(source_with_normals);
    p_icp.setInputTarget(target_with_normals);
    p_icp.setTransformationEpsilon(1e-10f);
    p_icp.setMaxCorrespondenceDistance(0.03f);
    p_icp.setEuclideanFitnessEpsilon(0.0001f);
    p_icp.setMaximumIterations(35);
    pcl::PointCloud<pcl::PointNormal>::Ptr p_icp_cloud(new pcl::PointCloud<pcl::PointNormal>);
    p_icp.align(*p_icp_cloud);

    Eigen::Matrix4f transformation_matrix1 = p_icp.getFinalTransformation();
    transformation_matrix1 = transformation_matrix1 * Transform_ransac[minIndex_ransac];
    log("[step6] ICP fitness: " + std::to_string(p_icp.getFitnessScore())
        + " converged=" + std::string(p_icp.hasConverged() ? "yes" : "no"));

    const std::filesystem::path sourcePath(params.source);
    const std::filesystem::path targetPath(params.target);
    const std::string base = sourcePath.stem().string() + "_to_" + targetPath.stem().string();
    const std::string matrix_path = params.output + "/" + base + "_transformationMatrix.txt";
    saveMatrix(matrix_path, transformation_matrix1);

    log("===================================");
    log("Transformation Matrix (" + matrix_path + "):");
    log("===================================");
    std::cout << std::fixed << std::setprecision(6);
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            std::cout << transformation_matrix1(r, c) << (c == 3 ? "\n" : "\t");
        }
    }
    log("===================================");

    pcl::PointCloud<pcl::PointXYZ>::Ptr registered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*source_cloud, *registered, transformation_matrix1);
    const std::string pcd_path = params.output + "/" + base + "_registered.pcd";
    pcl::io::savePCDFileBinary(pcd_path, *registered);
    log("Registered source saved to: " + pcd_path);

    return 0;
}
