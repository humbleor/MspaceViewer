#include "utils/ForestGlobalIO.h"
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <unordered_map>
#include <cwctype>
#include <climits>
#include <stdexcept>

namespace forest::io {
namespace fs=std::filesystem;
namespace {
static_assert(std::endian::native==std::endian::little);
constexpr size_t bucketCount=128,blockPoints=65536;
constexpr std::array<uint16_t,4> lengths={20,28,26,34};
template<class T>T read(const char* p){T v;std::memcpy(&v,p,sizeof v);return v;}
template<class T>void set(char* p,T v){std::memcpy(p,&v,sizeof v);}
struct Source {
    bool las=false;uint8_t format=0;uint16_t length=20;uint64_t count=0,size=0;
    uint32_t offset=0;fs::file_time_type time;
    Eigen::Vector3d scale=Eigen::Vector3d::Constant(.001),origin=Eigen::Vector3d::Zero(),lo,hi;
    std::vector<char> prefix;
};
std::wstring extension(const fs::path& p){auto e=p.extension().wstring();for(auto& c:e)c=std::towlower(c);return e;}
Source describe(const fs::path& p,uint64_t expected) {
    Source s;s.size=fs::file_size(p);s.time=fs::last_write_time(p);s.las=extension(p)==L".las";
    if(!s.las){auto cloud=load(p);s.count=cloud->size();s.lo.setConstant(INFINITY);s.hi.setConstant(-INFINITY);
        for(const auto& v:*cloud){Eigen::Vector3d q(v.x,v.y,v.z);s.lo=s.lo.cwiseMin(q);s.hi=s.hi.cwiseMax(q);}}
    else {
        std::ifstream in(p,std::ios::binary);std::array<char,227> h{};in.read(h.data(),h.size());
        if(!in||std::memcmp(h.data(),"LASF",4)||h[24]!=1||h[25]!=2||read<uint16_t>(h.data()+94)!=227)
            throw std::runtime_error("LAS export supports uncompressed LAS 1.2 formats 0-3: "+utf8(p));
        s.format=uint8_t(h[104]);s.length=read<uint16_t>(h.data()+105);s.offset=read<uint32_t>(h.data()+96);
        s.count=read<uint32_t>(h.data()+107);
        if(s.format>3||s.length!=lengths[s.format]||s.offset<227||s.offset>64*1024*1024||s.size<s.offset+s.count*s.length)
            throw std::runtime_error("Unsupported/truncated LAS record layout (extra dimensions require explicit conversion): "+utf8(p));
        for(int j=0;j<3;++j){s.scale[j]=read<double>(h.data()+131+8*j);s.origin[j]=read<double>(h.data()+155+8*j);
            s.hi[j]=read<double>(h.data()+179+16*j);s.lo[j]=read<double>(h.data()+187+16*j);}
        s.prefix.resize(s.offset);in.seekg(0);in.read(s.prefix.data(),s.prefix.size());if(!in)throw std::runtime_error("Cannot read LAS header");
    }
    if(s.count!=expected||!s.count||!s.scale.allFinite()||(s.scale.array()<=0).any()||!s.origin.allFinite()||
       !s.lo.allFinite()||!s.hi.allFinite()||(s.lo.array()>s.hi.array()).any())throw std::runtime_error("Input count/header changed: "+utf8(p));
    return s;
}
void validPose(const Eigen::Matrix4d& p){
    if(!p.allFinite()||!p.row(3).isApprox(Eigen::RowVector4d(0,0,0,1),1e-9)||
       !(p.topLeftCorner<3,3>().transpose()*p.topLeftCorner<3,3>()).isApprox(Eigen::Matrix3d::Identity(),1e-7)||
       std::abs(p.topLeftCorner<3,3>().determinant()-1)>1e-7)throw std::runtime_error("Invalid saved rigid pose");
}
struct Key {std::array<int64_t,3> a;bool operator==(const Key&)const=default;};
struct Hash {size_t operator()(const Key& k)const noexcept {
    uint64_t h=0x9e3779b97f4a7c15ULL;for(auto a:k.a){uint64_t v=uint64_t(a)+0x9e3779b97f4a7c15ULL;
        v=(v^(v>>30))*0xbf58476d1ce4e5b9ULL;v=(v^(v>>27))*0x94d049bb133111ebULL;h^=(v^(v>>31))+ (h<<6)+(h>>2);}return size_t(h);
}};
struct Entry {std::array<char,34> record{};double distance=0;};
using Cells=std::unordered_map<Key,Entry,Hash>;
std::vector<std::string> csv(const std::string& line){
    std::vector<std::string> fields;std::string field;bool quoted=false;
    for(size_t i=0;i<line.size();++i){char c=line[i];if(c=='"'){if(quoted&&i+1<line.size()&&line[i+1]=='"'){field+='"';++i;}else quoted=!quoted;}
        else if(c==','&&!quoted){fields.push_back(field);field.clear();}else if(c!='\r')field+=c;}
    if(quoted)throw std::runtime_error("Malformed inventory CSV");fields.push_back(field);return fields;
}
}

uint64_t writeLas(const fs::path& path,const std::vector<fs::path>& inputs,const std::vector<uint64_t>& counts,
                  const std::vector<Eigen::Matrix4d>& poses,double resolution,const GlobalProgress& report) {
    if(inputs.empty()||inputs.size()!=counts.size()||inputs.size()!=poses.size()||!std::isfinite(resolution)||resolution<0)
        throw std::runtime_error("Invalid LAS export arguments");
    auto tell=[&](int p,const std::string& m){if(report)report(p,m);};
    std::vector<Source> sources;uint64_t total=0;bool compatible=true;
    int gpsEncoding=-1;
    Eigen::Vector3d scale=Eigen::Vector3d::Constant(.001),lo=Eigen::Vector3d::Constant(INFINITY),hi=Eigen::Vector3d::Constant(-INFINITY);
    for(size_t i=0;i<inputs.size();++i){validPose(poses[i]);std::error_code ec;
        if(fs::absolute(path).lexically_normal()==fs::absolute(inputs[i]).lexically_normal()||fs::equivalent(path,inputs[i],ec))
            throw std::runtime_error("LAS output must differ from inputs");
        auto s=describe(inputs[i],counts[i]);total+=s.count;scale=scale.cwiseMin(s.scale);
        if(s.las&&(s.format==1||s.format==3)){int encoding=read<uint16_t>(s.prefix.data()+6)&1;
            if(gpsEncoding>=0&&gpsEncoding!=encoding)throw std::runtime_error("Cannot merge incompatible LAS GPS time encodings");gpsEncoding=encoding;}
        if(!s.las||(!sources.empty()&&s.format!=sources[0].format))compatible=false;
        for(int k=0;k<8;++k){Eigen::Vector3d q;for(int j=0;j<3;++j)q[j]=(k&(1<<j))?s.hi[j]:s.lo[j];
            q=(poses[i].topLeftCorner<3,3>()*q+poses[i].topRightCorner<3,1>()).eval();lo=lo.cwiseMin(q);hi=hi.cwiseMax(q);}
        sources.push_back(std::move(s));
    }
    const uint8_t format=compatible?sources[0].format:3;const uint16_t length=lengths[format];
    Eigen::Vector3d origin=(lo+hi)/2;
    for(int j=0;j<3;++j){scale[j]=std::max(scale[j],(hi[j]-lo[j])/(2.0*(INT32_MAX-2)));origin[j]=std::round(origin[j]/scale[j])*scale[j];}
    if(resolution>0&&(scale.array()>resolution/10).any())throw std::runtime_error("LAS coordinate scale is too coarse for requested voxel");
    auto parent=fs::absolute(path).parent_path();fs::create_directories(parent);
    if(fs::space(parent).available<total*length*(resolution>0?2:1)+64*1024*1024ULL)throw std::runtime_error("Insufficient space for LAS export and temporary voxel partitions");
    std::vector<char> header=sources[0].las?sources[0].prefix:std::vector<char>(227,0);
    std::memcpy(header.data(),"LASF",4);header[24]=1;header[25]=2;
    set<uint16_t>(header.data()+94,227);set<uint32_t>(header.data()+96,uint32_t(header.size()));
    header[104]=char(format);set<uint16_t>(header.data()+105,length);
    if(gpsEncoding>=0)set<uint16_t>(header.data()+6,uint16_t((read<uint16_t>(header.data()+6)&~1U)|unsigned(gpsEncoding)));
    std::memset(header.data()+58,0,32);constexpr char software[]="MspaceViewer Forest GTSAM";std::memcpy(header.data()+58,software,sizeof(software)-1);
    for(int j=0;j<3;++j){set<double>(header.data()+131+8*j,scale[j]);set<double>(header.data()+155+8*j,origin[j]);}
    auto temp=path;temp+=".tmp";
    const auto work=parent/(".forest-las-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::vector<fs::path> buckets;std::vector<std::ofstream> streams;std::vector<std::vector<char>> buffers;
    if(resolution>0){if(!fs::create_directory(work))throw std::runtime_error("Cannot create LAS work directory");
        streams.resize(bucketCount);buffers.resize(bucketCount);for(size_t i=0;i<bucketCount;++i){
            buckets.push_back(work/(std::to_string(i)+".bin"));streams[i].open(buckets.back(),std::ios::binary);
            if(!streams[i])throw std::runtime_error("Cannot create LAS partition");buffers[i].reserve(65536+length);}}
    auto cleanup=[&](){for(auto& s:streams)s.close();std::error_code ec;
        for(const auto& b:buckets)fs::remove(b,ec);if(!buckets.empty())fs::remove(work,ec);}; // Exact generated files only; never recursive.
    try {
        std::ofstream out(temp,std::ios::binary);if(!out)throw std::runtime_error("Cannot create LAS: "+utf8(temp));
        out.write(header.data(),header.size());uint64_t written=0,processed=0,nextReport=0;
        std::array<uint64_t,5> returns{};Eigen::Vector3d finalLo=Eigen::Vector3d::Constant(INFINITY),finalHi=Eigen::Vector3d::Constant(-INFINITY);
        std::vector<char> outputBuffer;outputBuffer.reserve(1024*1024+length);
        auto position=[&](const char* r){Eigen::Vector3d q;for(int j=0;j<3;++j)q[j]=double(read<int32_t>(r+4*j))*scale[j]+origin[j];return q;};
        auto output=[&](const char* r){if(++written>UINT32_MAX)throw std::runtime_error("LAS 1.2 output exceeds point-count limit");
            auto q=position(r);finalLo=finalLo.cwiseMin(q);finalHi=finalHi.cwiseMax(q);auto ret=uint8_t(r[14])&7;
            if(ret>=1&&ret<=5)++returns[ret-1];outputBuffer.insert(outputBuffer.end(),r,r+length);
            if(outputBuffer.size()>=1024*1024){out.write(outputBuffer.data(),outputBuffer.size());outputBuffer.clear();if(!out)throw std::runtime_error("LAS write failed");}};
        auto cell=[&](const char* r){const auto q=position(r);Key key;double distance=0;
            for(int j=0;j<3;++j){double v=std::floor(q[j]/resolution);if(!std::isfinite(v)||std::abs(v)>9e18)throw std::runtime_error("Voxel index out of range");
                key.a[j]=int64_t(v);double d=q[j]-(v+.5)*resolution;distance+=d*d;}return std::pair{key,distance};};
        auto insert=[&](Cells& cells,const char* r){auto [key,distance]=cell(r);auto [it,fresh]=cells.try_emplace(key);
            if(fresh||distance<it->second.distance){std::memcpy(it->second.record.data(),r,length);it->second.distance=distance;}};
        auto partition=[&](const Key& key,const Entry& e){auto bucket=Hash{}(key)%bucketCount;auto& b=buffers[bucket];
            b.insert(b.end(),e.record.data(),e.record.data()+length);if(b.size()>=65536){streams[bucket].write(b.data(),b.size());b.clear();
                if(!streams[bucket])throw std::runtime_error("LAS partition write failed");}};
        Cells local;local.reserve(blockPoints);
        for(size_t i=0;i<inputs.size();++i){const auto& s=sources[i];tell(int(80*processed/total),"Reading original LAS/points "+utf8(inputs[i].filename()));
            auto transform=[&](const char* raw,const Eigen::Vector3d& p){std::array<char,34> r{};
                if(raw){std::memcpy(r.data(),raw,20);if(format==s.format)std::memcpy(r.data(),raw,length);
                    else {if(s.format==1||s.format==3)std::memcpy(r.data()+20,raw+20,8);
                        if(s.format==2||s.format==3)std::memcpy(r.data()+28,raw+(s.format==2?20:28),6);}}
                Eigen::Vector3d q=poses[i].topLeftCorner<3,3>()*p+poses[i].topRightCorner<3,1>();
                for(int j=0;j<3;++j){double v=std::round((q[j]-origin[j])/scale[j]);if(!std::isfinite(v)||v<INT32_MIN||v>INT32_MAX)
                        throw std::runtime_error("Transformed LAS coordinate out of range");set<int32_t>(r.data()+4*j,int32_t(v));}
                if(resolution>0)insert(local,r.data());else output(r.data());
                ++processed;if(processed>=nextReport){tell(int(80*processed/total),utf8(inputs[i].filename())+": processed "+std::to_string(processed)+"/"+std::to_string(total));nextReport=processed+2000000;}
            };
            auto flushLocal=[&](){for(const auto& [key,e]:local)partition(key,e);local.clear();};
            if(s.las){std::ifstream in(inputs[i],std::ios::binary);in.seekg(s.offset);std::vector<char> block(blockPoints*s.length);
                for(uint64_t done=0;done<s.count;){auto n=std::min<uint64_t>(blockPoints,s.count-done);in.read(block.data(),n*s.length);
                    if(!in)throw std::runtime_error("Truncated LAS point data");for(size_t k=0;k<n;++k){auto r=block.data()+k*s.length;Eigen::Vector3d p;
                        for(int j=0;j<3;++j)p[j]=double(read<int32_t>(r+4*j))*s.scale[j]+s.origin[j];transform(r,p);}
                    if(resolution>0)flushLocal();done+=n;}}
            else {auto cloud=load(inputs[i]);size_t done=0;for(const auto& p:*cloud){transform(nullptr,Eigen::Vector3d(p.x,p.y,p.z));
                    if(resolution>0&&++done%blockPoints==0)flushLocal();}if(resolution>0)flushLocal();}
            if(fs::file_size(inputs[i])!=s.size||fs::last_write_time(inputs[i])!=s.time)throw std::runtime_error("Input changed during LAS export");
        }
        if(processed!=total)throw std::runtime_error("Original LAS point count mismatch");
        if(resolution>0){for(size_t i=0;i<bucketCount;++i){if(!buffers[i].empty())streams[i].write(buffers[i].data(),buffers[i].size());
                streams[i].flush();if(!streams[i])throw std::runtime_error("Cannot flush LAS partition");streams[i].close();}
            buffers.clear();local.clear();local.rehash(0);Cells cells;
            for(size_t i=0;i<bucketCount;++i){tell(80+int(19*i/bucketCount),"Merging voxel partition "+std::to_string(i+1)+"/"+std::to_string(bucketCount));
                auto size=fs::file_size(buckets[i]);if(size%length)throw std::runtime_error("Invalid LAS partition size");
                std::ifstream in(buckets[i],std::ios::binary);std::vector<char> block(blockPoints*length);cells.clear();
                for(uint64_t done=0;done<size;){auto bytes=std::min<uint64_t>(block.size(),size-done);in.read(block.data(),bytes);
                    if(!in)throw std::runtime_error("Cannot read LAS partition");for(size_t k=0;k<bytes;k+=length)insert(cells,block.data()+k);done+=bytes;}
                for(const auto& [key,e]:cells)output(e.record.data());in.close();fs::remove(buckets[i]);}
        }
        if(!written)throw std::runtime_error("No points exported to LAS");
        if(!outputBuffer.empty())out.write(outputBuffer.data(),outputBuffer.size());
        set<uint32_t>(header.data()+107,uint32_t(written));for(size_t j=0;j<5;++j)set<uint32_t>(header.data()+111+4*j,uint32_t(returns[j]));
        for(int j=0;j<3;++j){set<double>(header.data()+179+16*j,finalHi[j]);set<double>(header.data()+187+16*j,finalLo[j]);}
        out.seekp(0);out.write(header.data(),header.size());out.flush();if(!out)throw std::runtime_error("Cannot finish LAS header");out.close();
        if(!out||fs::file_size(temp)!=header.size()+written*length)throw std::runtime_error("LAS output size mismatch");
        if(fs::exists(path))fs::remove(path);fs::rename(temp,path);cleanup();
        tell(100,"LAS saved: "+utf8(path)+", points="+std::to_string(written)+", voxel_m="+std::to_string(resolution));return written;
    }catch(...){cleanup();std::error_code ec;fs::remove(temp,ec);throw;}
}

void exportLasFromRun(const fs::path& input,const fs::path& run,const fs::path& output,double resolution,const GlobalProgress& report) {
    std::ifstream status(run/"run_status.txt");std::string state;status>>state;
    if(state!="completed")throw std::runtime_error("Only completed registration poses can be exported");
    std::ifstream inventory(run/"cloud_inventory.csv");std::string line;std::getline(inventory,line);
    if(line.find("node,file,original_points")!=0)throw std::runtime_error("Missing completed-run inventory");
    std::vector<fs::path> inputs;std::vector<uint64_t> counts;std::vector<Eigen::Matrix4d> poses;
    while(std::getline(inventory,line)){if(line.empty())continue;auto fields=csv(line);if(fields.size()<3)throw std::runtime_error("Invalid inventory row");
        const auto name=fs::u8path(fields[1]);if(name.has_parent_path()||name.empty())throw std::runtime_error("Invalid inventory filename");
        auto path=fs::absolute(input/name).lexically_normal();std::ifstream cache(run/".forest_cache"/(name.wstring()+L".forestbin"),std::ios::binary);
        uint64_t size=0;cache.read(reinterpret_cast<char*>(&size),8);if(!cache||size>1024*1024)throw std::runtime_error("Missing source fingerprint for saved poses");
        std::string key(size,'\0');cache.read(key.data(),size);std::ostringstream prefix;
        prefix<<"ForestPrepared-v1;"<<utf8(path)<<';'<<fs::file_size(path)<<';'<<fs::last_write_time(path).time_since_epoch().count()<<';';
        if(!cache||!key.starts_with(prefix.str()))throw std::runtime_error("Saved poses do not match current input: "+utf8(path));
        auto matrixName=name;matrixName+="_global_transform.txt";std::ifstream matrix(run/"transforms"/matrixName);Eigen::Matrix4d pose;
        for(int r=0;r<4;++r)for(int c=0;c<4;++c)if(!(matrix>>pose(r,c)))throw std::runtime_error("Missing saved station pose");
        std::string extra;if(matrix>>extra)throw std::runtime_error("Invalid saved pose dimensions");validPose(pose);
        inputs.push_back(path);counts.push_back(std::stoull(fields[2]));poses.push_back(pose);
    }
    if(inputs.empty()||!poses[0].isApprox(Eigen::Matrix4d::Identity(),1e-9))throw std::runtime_error("Invalid saved reference station");
    writeLas(output,inputs,counts,poses,resolution,report);
}
} // namespace forest::io
