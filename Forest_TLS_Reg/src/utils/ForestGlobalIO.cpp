#include "utils/ForestGlobalIO.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/point_tests.h>
#include <liblas/liblas.hpp>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <array>
#include <stdexcept>
#include <algorithm>
#include <cwctype>

namespace forest::io {
namespace fs = std::filesystem;
std::string utf8(const fs::path& p) { const auto s = p.u8string(); return {s.begin(), s.end()}; }
namespace {
template<class T> void put(std::ostream& out, const T& v) { out.write(reinterpret_cast<const char*>(&v), sizeof(v)); }
template<class T> T get(std::istream& in) {
    T v{}; in.read(reinterpret_cast<char*>(&v), sizeof(v));
    if (!in) throw std::runtime_error("Truncated Forest cache"); return v;
}
void putString(std::ostream& out, const std::string& s) { put(out, uint64_t(s.size())); out.write(s.data(), s.size()); }
std::string getString(std::istream& in) {
    const auto n = get<uint64_t>(in);
    if (n > 1024 * 1024) throw std::runtime_error("Invalid Forest cache string");
    std::string s(size_t(n), '\0'); in.read(s.data(), s.size());
    if (!in) throw std::runtime_error("Truncated Forest cache string"); return s;
}
uint64_t count(std::istream& in, uint64_t max) {
    auto n = get<uint64_t>(in); if (n > max) throw std::runtime_error("Invalid Forest cache count"); return n;
}
std::string cacheKey(const fs::path& path, const ConfigSetting& c) {
    std::ostringstream o;
    o << "ForestPrepared-v1;" << utf8(fs::absolute(path).lexically_normal()) << ';'
      << fs::file_size(path) << ';' << fs::last_write_time(path).time_since_epoch().count() << ';' << std::setprecision(17);
    // Extraction/descriptor parameters only: changing matching or graph thresholds
    // can reuse exactly the same features without re-reading the original LAS.
#define FIELD(x) o << #x << '=' << c.x << ';';
    FIELD(imgRow) FIELD(imgCol) FIELD(range) FIELD(grid_num)
    FIELD(isRow_Or_Column) FIELD(dDif_Th) FIELD(nSch_Stp) FIELD(lGrp_Ele_Min)
    FIELD(descriptor_near_num) FIELD(descriptor_min_len) FIELD(descriptor_max_len)
    FIELD(descriptor_len_diff) FIELD(side_resolution)
    FIELD(min_component_size) FIELD(tolorance) FIELD(max_n) FIELD(merge_dist)
    FIELD(linearityThres) FIELD(upThres) FIELD(scateringThres) FIELD(clusterHeight) FIELD(centerSelection)
    FIELD(pointsNumThres) FIELD(planarityThres)
    FIELD(bSloopSmooth) FIELD(cloth_resolution) FIELD(rigidness) FIELD(time_step) FIELD(class_threshold) FIELD(interations)
    FIELD(gnd_grid_size) FIELD(gnd_points_num) FIELD(gnd_points_dist)
#undef FIELD
    return o.str();
}
void centersOut(std::ostream& out, const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& points) {
    put(out, uint64_t(points->size()));
    for (const auto& p : *points) for (float v : {p.x,p.y,p.z,p.normal_x,p.normal_y,p.normal_z,p.intensity}) put(out,v);
}
pcl::PointCloud<pcl::PointXYZINormal>::Ptr centersIn(std::istream& in) {
    auto n = count(in, 1000000);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr points(new pcl::PointCloud<pcl::PointXYZINormal>);
    points->reserve(size_t(n));
    for (uint64_t i=0;i<n;++i) {
        pcl::PointXYZINormal p;
        p.x=get<float>(in);p.y=get<float>(in);p.z=get<float>(in);
        p.normal_x=get<float>(in);p.normal_y=get<float>(in);p.normal_z=get<float>(in);p.intensity=get<float>(in);
        points->push_back(p);
    }
    return points;
}
void vectorsOut(std::ostream& out, const TriDesc& d) {
    for (const auto* v : {&d.side_length_,&d.angle_,&d.center_,&d.vertex_A_,&d.vertex_B_,&d.vertex_C_,&d.vertex_attached_})
        for(int i=0;i<3;++i) put(out,(*v)[i]);
}
TriDesc vectorsIn(std::istream& in) {
    TriDesc d; d.frame_id_=0;
    for(auto* v : {&d.side_length_,&d.angle_,&d.center_,&d.vertex_A_,&d.vertex_B_,&d.vertex_C_,&d.vertex_attached_})
        for(int i=0;i<3;++i) (*v)[i]=get<double>(in);
    return d;
}
void commit(std::ofstream& out, const fs::path& temp, const fs::path& path) {
    out.flush(); if (!out) throw std::runtime_error("Cannot write: " + utf8(path));
    out.close(); if (!out) throw std::runtime_error("Cannot close: " + utf8(path));
    if(fs::exists(path)) fs::remove(path); // Only this requested output file.
    fs::rename(temp,path);
}
}

Cloud::Ptr load(const fs::path& path, const GlobalProgress& report) {
    Cloud::Ptr cloud(new Cloud);
    auto ext=path.extension().wstring();std::transform(ext.begin(),ext.end(),ext.begin(),::towlower);
    if (ext==L".las" || ext==L".laz") {
        std::ifstream in(path,std::ios::binary);
        if(!in) throw std::runtime_error("Cannot open LAS: "+utf8(path));
        liblas::ReaderFactory factory; auto reader=factory.CreateWithStream(in);
        const auto expected=reader.GetHeader().GetPointRecordsCount();cloud->reserve(expected);
        uint64_t seen=0;
        while(reader.ReadNextPoint()) {
            const auto& p=reader.GetPoint();pcl::PointXYZ v(float(p.GetX()),float(p.GetY()),float(p.GetZ()));
            if(pcl::isFinite(v)) cloud->push_back(v);
            ++seen;
            if(report && seen%2000000==0) report(int(100*seen/std::max<uint64_t>(1,expected)),
                "Reading "+utf8(path.filename())+": "+std::to_string(seen)+"/"+std::to_string(expected));
        }
        if(seen!=expected) throw std::runtime_error("LAS point count mismatch: "+utf8(path));
    } else {
        const int status=ext==L".pcd" ? pcl::io::loadPCDFile(path.string(),*cloud) :
            ext==L".ply" ? pcl::io::loadPLYFile(path.string(),*cloud) : -1;
        if(status<0) throw std::runtime_error("Cannot load point cloud: "+utf8(path));
        auto end=std::remove_if(cloud->points.begin(),cloud->points.end(),[](const auto& p){return !pcl::isFinite(p);});
        cloud->points.erase(end,cloud->points.end());cloud->width=uint32_t(cloud->size());cloud->height=1;
    }
    if(cloud->empty()) throw std::runtime_error("Empty point cloud: "+utf8(path));
    cloud->is_dense=true;return cloud;
}

PreparedCloud prepareCached(const fs::path& input, const fs::path& cacheFile,
                            const ConfigSetting& config, const GlobalProgress& report) {
    const auto key=cacheKey(input,config);
    if(fs::exists(cacheFile)) try {
        std::ifstream in(cacheFile,std::ios::binary);
        if(getString(in)!=key) throw std::runtime_error("Cache fingerprint changed");
        PreparedCloud p;p.originalPointCount=size_t(count(in,4000000000ULL));
        const auto n=count(in,std::min<uint64_t>(500000000,fs::file_size(cacheFile)/12));
        p.sampled.reset(new Cloud);p.sampled->reserve(size_t(n));
        for(uint64_t i=0;i<n;++i) {
            pcl::PointXYZ v;v.x=get<float>(in);v.y=get<float>(in);v.z=get<float>(in);p.sampled->push_back(v);
        }
        p.features.currCenter=centersIn(in);p.features.currCenterFix=centersIn(in);
        p.features.currPoints.reset(new Cloud);p.features.frame_id_=0;
        const auto d=count(in,std::min<uint64_t>(10000000,fs::file_size(cacheFile)/(21*sizeof(double))));p.features.desc_.reserve(size_t(d));
        for(uint64_t i=0;i<d;++i) p.features.desc_.push_back(vectorsIn(in));
        if(p.sampled->empty() || p.originalPointCount<p.sampled->size()) throw std::runtime_error("Invalid cached points");
        if(report) report(100,"Reusing Forest feature cache: "+utf8(input.filename()));
        return p;
    } catch(const std::exception& e) {
        if(report) report(0,"Rebuilding feature cache: "+std::string(e.what()));
    }
    auto raw=load(input,[&](int value,const std::string& message){if(report)report(value*70/100,message);});
    if(report) report(72,"Extracting Forest features: "+utf8(input.filename()));
    auto p=prepareCloud(raw,config);
    if(cacheKey(input,config)!=key) throw std::runtime_error("Input changed during feature extraction");
    fs::create_directories(cacheFile.parent_path());auto temp=cacheFile;temp+=".tmp";
    std::ofstream out(temp,std::ios::binary);if(!out) throw std::runtime_error("Cannot create feature cache: "+utf8(temp));
    putString(out,key);put(out,uint64_t(p.originalPointCount));put(out,uint64_t(p.sampled->size()));
    for(const auto& v:*p.sampled) {put(out,v.x);put(out,v.y);put(out,v.z);}
    centersOut(out,p.features.currCenter);centersOut(out,p.features.currCenterFix);
    put(out,uint64_t(p.features.desc_.size()));for(const auto& d:p.features.desc_) vectorsOut(out,d);
    commit(out,temp,cacheFile);
    if(report) report(100,"Forest features ready: "+utf8(input.filename())+", centers="+
        std::to_string(p.features.currCenter->size())+", sampled="+std::to_string(p.sampled->size()));
    return p;
}

} // namespace forest::io
