#include "utils/HashRegObj.h"
#include "utils/ForestPairRegistration.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/point_tests.h>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <cmath>

namespace forest {
namespace {
using Cloud = pcl::PointCloud<pcl::PointXYZ>;
bool rigid(const Eigen::Matrix4d& t) {
    const Eigen::Matrix3d r = t.topLeftCorner<3, 3>();
    return t.allFinite() && std::abs(r.determinant() - 1) < 1e-5 &&
        (r.transpose() * r - Eigen::Matrix3d::Identity()).norm() < 1e-5;
}
Cloud::Ptr sample(const Cloud::Ptr& cloud) {
    Cloud::Ptr out(new Cloud);
    pcl::VoxelGrid<pcl::PointXYZ> grid;
    grid.setLeafSize(0.15f, 0.15f, 0.15f);
    grid.setInputCloud(cloud);
    grid.filter(*out);
    return out;
}
GeometryQuality quality(const Cloud::Ptr& from, const Cloud::Ptr& to,
                        const Eigen::Matrix4d& transform) {
    GeometryQuality q;
    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(to);
    std::vector<int> index(1);
    std::vector<float> distance(1);
    double sum = 0;
    const size_t stride = std::max<size_t>(1, (from->size() + 49999) / 50000);
    for (size_t i = 0; i < from->size(); i += stride) {
        const auto& p = from->points[i];
        if (!pcl::isFinite(p)) continue;
        const Eigen::Vector3d v = transform.topLeftCorner<3, 3>() *
            Eigen::Vector3d(p.x, p.y, p.z) + transform.topRightCorner<3, 1>();
        pcl::PointXYZ query(static_cast<float>(v.x()), static_cast<float>(v.y()), static_cast<float>(v.z()));
        ++q.tested;
        if (tree.nearestKSearch(query, 1, index, distance) && distance[0] <= 0.09) {
            ++q.inliers;
            sum += distance[0];
        }
    }
    if (q.tested) q.overlap = double(q.inliers) / q.tested;
    if (q.inliers) q.rmse = std::sqrt(sum / q.inliers);
    return q;
}
}

ConfigSetting defaultForestConfig() {
    ConfigSetting config;
    config.rough_dis_threshold = 0.03;
    config.dist_candi_frames_verify = 1.0;
    config.use_matched_num = 4000;
    config.icp_threshold = 0.25;
    return config;
}

PreparedCloud prepareCloud(const Cloud::Ptr& raw, const ConfigSetting& config) {
    if (!raw || raw->empty()) throw std::runtime_error("Empty Forest input cloud");
    ConfigSetting settings = config;
    HashRegDescManager manager(settings);
    PreparedCloud prepared;
    prepared.originalPointCount = raw->size();
    manager.GenTriDescs(raw, prepared.features);
    // Only centers/triangles are needed for matching; release segmented point copies.
    prepared.features.currPoints.reset(new Cloud);
    prepared.sampled = sample(raw);
    return prepared;
}

PairResult registerPair(const Cloud::Ptr& source, const Cloud::Ptr& target,
                       const ConfigSetting& config,
                       std::function<void(int, const std::string&)> report) {
    if (!source || !target || source->empty() || target->empty()) {
        PairResult out; out.reason = "empty_input"; return out;
    }
    if (report) report(20, "Preparing target Forest features ...");
    auto preparedTarget = prepareCloud(target, config);
    if (report) report(40, "Preparing source Forest features ...");
    auto preparedSource = prepareCloud(source, config);
    return registerPreparedPair(preparedSource, preparedTarget, config, report);
}

PairResult registerPreparedPair(const PreparedCloud& source, const PreparedCloud& target,
                               const ConfigSetting& config,
                               std::function<void(int, const std::string&)> report) {
    PairResult out;
    out.coarseThreshold = config.icp_threshold;
    auto log = [&](int progress, const std::string& message) { if (report) report(progress, message); };
    FrameInfo reference = target.features, current = source.features;
    if (!reference.currCenter || !current.currCenter || !source.sampled || !target.sampled) {
        out.reason = "missing_prepared_features"; return out;
    }
    // Each two-frame search has reference=0, current=1, independently of global IDs.
    reference.frame_id_ = 0; current.frame_id_ = 1;
    for (auto& d : reference.desc_) d.frame_id_ = 0;
    for (auto& d : current.desc_) d.frame_id_ = 1;
    out.targetCenters = reference.currCenter->size(); out.sourceCenters = current.currCenter->size();
    out.targetDescriptors = reference.desc_.size(); out.sourceDescriptors = current.desc_.size();
    log(50, "Forest centers (source/target)=" + std::to_string(out.sourceCenters) + "/" + std::to_string(out.targetCenters));
    if (out.targetCenters < 3 || !out.targetDescriptors) {
        out.reason = "insufficient_target_forest_features"; return out;
    }
    if (out.sourceCenters < 3 || !out.sourceDescriptors) {
        out.reason = "insufficient_source_forest_features"; return out;
    }
    ConfigSetting settings = config;
    HashRegDescManager manager(settings);
    manager.AddTriDescs(reference);
    std::pair<int, double> match(-1, 0);
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> coarse{Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity()};
    std::vector<std::pair<TriDesc, TriDesc>> triangles;
    log(60, "Searching 3D triangle correspondences ...");
    manager.SearchPosition(current, match, coarse, triangles);
    out.coarseScore = match.second;
    out.matchedTriangles = triangles.size();
    out.coarse.topLeftCorner<3, 3>() = coarse.second;
    out.coarse.topRightCorner<3, 1>() = coarse.first;
    out.transform = out.coarse;
    log(65, "Coarse score=" + std::to_string(out.coarseScore) + ", matched triangles=" + std::to_string(out.matchedTriangles));
    if (match.first < 0) {
        out.reason = "forest_coarse_score_below_threshold";
        return out;
    }
    if (triangles.empty() || !rigid(out.coarse)) {
        out.reason = "invalid_or_missing_coarse_registration";
        return out;
    }
    log(68, "Using prepared 0.15 m GICP inputs ...");
    auto src = source.sampled, tgt = target.sampled;
    if (src->size() < 30 || tgt->size() < 30) {
        out.reason = "insufficient_sampled_points";
        return out;
    }
    out.coarseForward = quality(src, tgt, out.coarse);
    out.coarseReverse = quality(tgt, src, out.coarse.inverse());
    log(72, "Coarse overlap (source/target)=" + std::to_string(out.coarseForward.overlap) + "/" + std::to_string(out.coarseReverse.overlap));
    if (std::min(out.coarseForward.overlap, out.coarseReverse.overlap) < 0.05) {
        out.reason = "insufficient_coarse_geometric_overlap";
        return out;
    }
    std::vector<Eigen::Vector3d> srcPoints, tgtPoints;
    srcPoints.reserve(src->size()); tgtPoints.reserve(tgt->size());
    point_to_vector(src, srcPoints); point_to_vector(tgt, tgtPoints);
    small_gicp::RegistrationSetting setting;
    setting.num_threads = 4;
    setting.downsampling_resolution = 0.15;
    setting.max_correspondence_distance = 0.8;
    setting.max_iterations = 100;
    setting.rotation_eps = 1e-5;
    setting.translation_eps = 1e-4;
    Eigen::Isometry3d init = Eigen::Isometry3d::Identity();
    init.matrix() = out.coarse;
    log(75, "Running 6-DOF GICP with Forest coarse initialization ...");
    const auto result = small_gicp::align(tgtPoints, srcPoints, init, setting);
    // align returns the COMPLETE source -> target transform, not an increment.
    out.transform = result.T_target_source.matrix();
    out.converged = result.converged;
    out.iterations = result.iterations;
    out.optimizerInliers = result.num_inliers;
    if (!rigid(out.transform)) { out.reason = "invalid_gicp_transform"; return out; }
    out.finalForward = quality(src, tgt, out.transform);
    out.finalReverse = quality(tgt, src, out.transform.inverse());
    log(85, "GICP converged=" + std::to_string(out.converged) + ", iterations=" + std::to_string(out.iterations) +
        ", overlap=" + std::to_string(out.finalForward.overlap) + "/" + std::to_string(out.finalReverse.overlap) +
        ", RMSE(m)=" + std::to_string(out.finalForward.rmse) + "/" + std::to_string(out.finalReverse.rmse));
    if (!out.converged) out.reason = "gicp_not_converged";
    else if (out.optimizerInliers < 30 || std::min(out.finalForward.inliers, out.finalReverse.inliers) < 30)
        out.reason = "insufficient_gicp_correspondences";
    else if (std::min(out.finalForward.overlap, out.finalReverse.overlap) < 0.1)
        out.reason = "insufficient_final_overlap";
    else if (std::max(out.finalForward.rmse, out.finalReverse.rmse) > 0.25)
        out.reason = "excessive_final_rmse";
    else if (out.finalForward.overlap + 0.02 < out.coarseForward.overlap ||
             out.finalReverse.overlap + 0.02 < out.coarseReverse.overlap)
        out.reason = "overlap_degraded_after_gicp";
    else { out.accepted = true; out.reason = "accepted"; }
    return out;
}

void savePairReport(const std::filesystem::path& path, const PairResult& r) {
    std::ofstream out(path);
    if (!out) throw std::runtime_error("Cannot create Forest quality report: " + path.string());
    out << std::setprecision(12) << "accepted=" << r.accepted << "\nreason=" << r.reason
        << "\nsource_centers=" << r.sourceCenters << "\ntarget_centers=" << r.targetCenters
        << "\nsource_triangles=" << r.sourceDescriptors << "\ntarget_triangles=" << r.targetDescriptors
        << "\nmatched_triangles=" << r.matchedTriangles << "\ncoarse_score=" << r.coarseScore
        << "\ncoarse_score_threshold=" << r.coarseThreshold
        << "\ngicp_converged=" << r.converged << "\ngicp_iterations=" << r.iterations
        << "\ngicp_optimizer_inliers=" << r.optimizerInliers
        << "\nmetric_voxel_m=0.15\nmetric_distance_m=0.30\nmetric_max_queries_per_direction=50000\n";
    auto metric = [&](const char* name, const GeometryQuality& q) {
        out << name << "_tested=" << q.tested << '\n' << name << "_inliers=" << q.inliers << '\n'
            << name << "_overlap=" << q.overlap << '\n' << name << "_rmse_m=" << q.rmse << '\n';
    };
    metric("coarse_forward", r.coarseForward); metric("coarse_reverse", r.coarseReverse);
    metric("final_forward", r.finalForward); metric("final_reverse", r.finalReverse);
    out << "coarse_source_to_target:\n" << r.coarse << "\nfinal_source_to_target:\n" << r.transform << '\n';
    out.flush();
    if (!out) throw std::runtime_error("Failed writing Forest quality report: " + path.string());
}
} // namespace forest
