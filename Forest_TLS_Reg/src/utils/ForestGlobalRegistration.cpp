#include "utils/ForestGlobalRegistration.h"
#include "utils/ForestGlobalIO.h"
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <fstream>
#include <iomanip>
#include <numeric>
#include <queue>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <cwctype>
#include <tuple>

namespace forest {
namespace fs=std::filesystem;
namespace {
constexpr double pi=3.14159265358979323846;
using Matrix=Eigen::Matrix4d;
struct Edge {
    int source=0,target=0;
    PairResult pair;
    bool active=false,tree=false;
    int cycles=0,failedCycles=0;
    std::string state;
};
struct Residual {double rotation=0,translation=0;};
struct Cycle {std::string kind,route;int round=0;Residual error;bool passed=false;};
struct Optimization {std::vector<Matrix> poses;double before=0,after=0;size_t iterations=0;};
Residual residual(const Matrix& delta) {
    return {std::acos(std::clamp((delta.topLeftCorner<3,3>().trace()-1)/2,-1.0,1.0))*180/pi,
            delta.topRightCorner<3,1>().norm()};
}
Residual edgeError(const Edge& e,const std::vector<Matrix>& poses) {
    // P_i maps station i -> reference; T_target_source maps source -> target.
    return residual(e.pair.transform.inverse()*poses[e.target].inverse()*poses[e.source]);
}
bool good(const Residual& r,const GlobalOptions& o) {
    return std::isfinite(r.rotation)&&std::isfinite(r.translation)&&
        r.rotation<=o.cycleRotationDegrees&&r.translation<=o.cycleTranslationMeters;
}
double weight(const Edge& e) {
    double overlap=std::min(e.pair.finalForward.overlap,e.pair.finalReverse.overlap);
    double rmse=std::max(e.pair.finalForward.rmse,e.pair.finalReverse.rmse);
    return overlap/std::max(0.0025,rmse*rmse);
}
std::ofstream output(const fs::path& path) {
    std::ofstream out(path);if(!out) throw std::runtime_error("Cannot create output: "+io::utf8(path));
    out<<std::setprecision(12);return out;
}
void checked(std::ofstream& out) {out.flush();if(!out)throw std::runtime_error("Failed writing Forest global output");}
std::string csv(const std::string& s) {
    std::string q="\"";for(char c:s){if(c=='\"')q+='\"';q+=c;}return q+'\"';
}

std::vector<Matrix> spanningTree(int n,std::vector<Edge>& edges) {
    std::vector<int> order,dsu(n);std::iota(dsu.begin(),dsu.end(),0);
    for(int i=0;i<int(edges.size());++i){edges[i].tree=false;if(edges[i].active)order.push_back(i);}
    std::stable_sort(order.begin(),order.end(),[&](int a,int b){return weight(edges[a])>weight(edges[b]);});
    auto root=[&](int x){while(dsu[x]!=x)x=dsu[x];return x;};
    int used=0;
    for(int i:order){auto& e=edges[i];int a=root(e.source),b=root(e.target);if(a==b)continue;
        dsu[a]=b;e.tree=true;++used;}
    if(used!=n-1)throw std::runtime_error("Reliable Forest graph is disconnected; refusing partial global output");
    std::vector<Matrix> poses(n,Matrix::Identity());std::vector<bool> seen(n,false);
    std::queue<int> queue;queue.push(0);seen[0]=true;
    while(!queue.empty()) {
        int u=queue.front();queue.pop();
        for(const auto& e:edges)if(e.active&&e.tree) {
            if(e.source==u&&!seen[e.target]){poses[e.target]=poses[u]*e.pair.transform.inverse();seen[e.target]=true;queue.push(e.target);}
            else if(e.target==u&&!seen[e.source]){poses[e.source]=poses[u]*e.pair.transform;seen[e.source]=true;queue.push(e.source);}
        }
    }
    return poses;
}
int findEdge(const std::vector<Edge>& edges,int a,int b) {
    for(int i=0;i<int(edges.size());++i)if(edges[i].active&&
        ((edges[i].source==a&&edges[i].target==b)||(edges[i].source==b&&edges[i].target==a)))return i;
    return -1;
}
Matrix direction(const Edge& e,int from){return e.source==from?e.pair.transform:e.pair.transform.inverse().eval();}
std::string treeRoute(int from,int to,const std::vector<Edge>& edges,int n) {
    std::vector<int> parent(n,-1);parent[from]=from;std::queue<int> q;q.push(from);
    while(!q.empty()){int u=q.front();q.pop();for(const auto& e:edges)if(e.active&&e.tree){
        int v=e.source==u?e.target:e.target==u?e.source:-1;
        if(v>=0&&parent[v]<0){parent[v]=u;q.push(v);}
    }}
    std::vector<int> path;for(int u=to;u!=from;u=parent[u]){if(u<0||parent[u]<0)throw std::runtime_error("Missing tree route");path.push_back(u);}
    path.push_back(from);std::reverse(path.begin(),path.end());path.push_back(from);
    std::string s;for(int u:path){if(!s.empty())s+='-';s+=std::to_string(u);}return s;
}
std::vector<Matrix> validateCycles(int n,std::vector<Edge>& edges,const GlobalOptions& options,
                                  std::vector<Cycle>& audit) {
    // Repeated triangle failures help identify a bad edge supported by several
    // independent cycles. Never infer success from zero tested cycles.
    for(int round=0;round<int(edges.size());++round) {
        spanningTree(n,edges);
        for(auto& e:edges)if(e.active){e.cycles=0;e.failedCycles=0;}
        for(int a=0;a<n;++a)for(int b=a+1;b<n;++b)for(int c=b+1;c<n;++c) {
            int ab=findEdge(edges,a,b),bc=findEdge(edges,b,c),ca=findEdge(edges,c,a);
            if(ab<0||bc<0||ca<0)continue;
            auto r=residual(direction(edges[ca],c)*direction(edges[bc],b)*direction(edges[ab],a));
            bool pass=good(r,options);
            audit.push_back({"triangle",std::to_string(a)+"-"+std::to_string(b)+"-"+std::to_string(c)+"-"+std::to_string(a),round,r,pass});
            for(int i:{ab,bc,ca}){++edges[i].cycles;if(!pass)++edges[i].failedCycles;}
        }
        int worst=-1;
        for(int i=0;i<int(edges.size());++i){const auto& e=edges[i];
            if(!e.active||e.failedCycles<2||double(e.failedCycles)/e.cycles<0.6)continue;
            auto rank=[](const Edge& v){return std::tuple{double(v.failedCycles)/v.cycles,v.failedCycles,-weight(v)};};
            if(worst<0||rank(e)>rank(edges[worst]))worst=i;
        }
        if(worst<0)break;
        edges[worst].active=false;edges[worst].state="rejected_triangle_cycles";
    }
    // Check every remaining extra edge against a quality-prioritized tree.
    for(int round=0;round<=int(edges.size());++round) {
        auto poses=spanningTree(n,edges);int reject=-1;
        for(int i=0;i<int(edges.size());++i){const auto& e=edges[i];if(!e.active||e.tree)continue;
            auto r=edgeError(e,poses);bool pass=good(r,options);
            audit.push_back({"fundamental",treeRoute(e.source,e.target,edges,n),round,r,pass});
            if(!pass&&(reject<0||weight(e)<weight(edges[reject])))reject=i;
        }
        if(reject<0)return poses;
        edges[reject].active=false;edges[reject].state="rejected_fundamental_cycle";
    }
    throw std::runtime_error("Cycle validation did not terminate");
}
Optimization optimize(const std::vector<Matrix>& initial,const std::vector<Edge>& edges) {
    gtsam::NonlinearFactorGraph graph;gtsam::Values values;
    auto key=[](int i){return gtsam::Symbol('x',i);};
    gtsam::Vector6 prior;prior.setConstant(1e-6);
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(key(0),gtsam::Pose3(),gtsam::noiseModel::Diagonal::Sigmas(prior));
    for(int i=0;i<int(initial.size());++i)values.insert(key(i),gtsam::Pose3(initial[i]));
    for(const auto& e:edges)if(e.active) {
        const double sigma=std::max(0.05,std::max(e.pair.finalForward.rmse,e.pair.finalReverse.rmse));
        gtsam::Vector6 sigmas;sigmas<<pi/180,pi/180,pi/180,sigma,sigma,sigma;
        auto noise=gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.345),
            gtsam::noiseModel::Diagonal::Sigmas(sigmas));
        // Between(P_source,P_target)=source_from_target, the inverse of registration.
        graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(key(e.source),key(e.target),
            gtsam::Pose3(e.pair.transform.inverse().eval()),noise);
    }
    // All three rotational components are free. No UnitZ / gravity priors.
    gtsam::LevenbergMarquardtParams params;params.setMaxIterations(100);params.setLinearSolverType("MULTIFRONTAL_QR");
    gtsam::LevenbergMarquardtOptimizer optimizer(graph,values,params);
    auto result=optimizer.optimize();Optimization out;out.before=graph.error(values);out.after=graph.error(result);out.iterations=optimizer.iterations();
    if(!std::isfinite(out.after)||out.after>out.before+1e-8*std::max(1.0,out.before))throw std::runtime_error("Invalid GTSAM objective");
    const Matrix reference=result.at<gtsam::Pose3>(key(0)).matrix().inverse();
    for(int i=0;i<int(initial.size());++i){Matrix p=reference*result.at<gtsam::Pose3>(key(i)).matrix();
        if(!p.allFinite())throw std::runtime_error("Invalid optimized pose");out.poses.push_back(p);}
    out.poses[0]=Matrix::Identity();return out;
}
void writeEdges(const fs::path& path,const std::vector<Edge>& edges,const std::vector<fs::path>& files,
                const std::vector<Matrix>* initial=nullptr,const std::vector<Matrix>* final=nullptr) {
    auto out=output(path);
    out<<"source,target,source_file,target_file,state,coarse_score,coarse_threshold,matched_triangles,gicp_converged,gicp_iterations,forward_overlap,reverse_overlap,forward_rmse_m,reverse_rmse_m,triangle_cycles,failed_triangles,tree_edge,initial_rot_residual_deg,initial_translation_residual_m,optimized_rot_residual_deg,optimized_translation_residual_m";
    for(int r=0;r<4;++r)for(int c=0;c<4;++c)out<<",target_from_source_"<<r<<c;out<<'\n';
    for(const auto& e:edges){const auto& p=e.pair;
        out<<e.source<<','<<e.target<<','<<csv(io::utf8(files[e.source].filename()))<<','<<csv(io::utf8(files[e.target].filename()))<<','
            <<e.state<<','<<p.coarseScore<<','<<p.coarseThreshold<<','<<p.matchedTriangles<<','<<p.converged<<','<<p.iterations<<','
            <<p.finalForward.overlap<<','<<p.finalReverse.overlap<<','<<p.finalForward.rmse<<','<<p.finalReverse.rmse<<','
            <<e.cycles<<','<<e.failedCycles<<','<<e.tree;
        for(const auto* poses:{initial,final}){if(poses&&e.active){auto r=edgeError(e,*poses);out<<','<<r.rotation<<','<<r.translation;}else out<<",nan,nan";}
        for(int r=0;r<4;++r)for(int c=0;c<4;++c)out<<','<<p.transform(r,c);out<<'\n';
    }checked(out);
}
void writeCycles(const fs::path& path,const std::vector<Cycle>& audit) {
    auto out=output(path);out<<"kind,node_route,round,rotation_deg,translation_m,passed\n";
    for(const auto& c:audit)out<<c.kind<<','<<csv(c.route)<<','<<c.round<<','<<c.error.rotation<<','<<c.error.translation<<','<<c.passed<<'\n';
    checked(out);
}
std::vector<fs::path> discover(const fs::path& root) {
    std::vector<fs::path> files;
    for(const auto& e:fs::directory_iterator(root))if(e.is_regular_file()) {
        auto ext=e.path().extension().wstring();std::transform(ext.begin(),ext.end(),ext.begin(),::towlower);
        if(ext==L".las"||ext==L".pcd"||ext==L".ply")files.push_back(e.path());
    }
    std::sort(files.begin(),files.end(),[](const auto& a,const auto& b){
        auto x=a.stem().wstring(),y=b.stem().wstring();
        const auto numeric=[](const std::wstring& name){return !name.empty()&&name.size()<19&&
            std::all_of(name.begin(),name.end(),[](wchar_t c){return c>=L'0'&&c<=L'9';});};
        const bool nx=numeric(x),ny=numeric(y);
        if(nx!=ny)return nx;
        if(nx&&std::stoull(x)!=std::stoull(y))return std::stoull(x)<std::stoull(y);
        return a.filename()<b.filename();
    });return files;
}
}

void runGlobalRegistration(const fs::path& input,const fs::path& root,const GlobalOptions& options,const GlobalProgress& report) {
    if(!fs::is_directory(input))throw std::runtime_error("Invalid Forest input directory");
    // Lexical normalization avoids opening inaccessible ancestor directories on
    // removable drives. Direct equivalence still catches existing junction aliases.
    auto normalized=[](const fs::path& p){auto s=fs::absolute(p).lexically_normal().wstring();
        std::transform(s.begin(),s.end(),s.begin(),::towlower);return s;};
    std::error_code equivalenceError;
    if(normalized(input)==normalized(root)||fs::equivalent(input,root,equivalenceError))
        throw std::runtime_error("Input and output directories must differ");
    if(!std::isfinite(options.mergeResolution)||options.mergeResolution<0||options.mergeResolution>std::numeric_limits<float>::max()||
        !std::isfinite(options.config.icp_threshold)||options.config.icp_threshold<0||options.config.icp_threshold>1||
        !std::isfinite(options.cycleRotationDegrees)||options.cycleRotationDegrees<=0||
        !std::isfinite(options.cycleTranslationMeters)||options.cycleTranslationMeters<=0)
        throw std::runtime_error("Invalid Forest global parameters");
    fs::create_directories(root);fs::create_directories(root/"pairs");
    {auto out=output(root/"run_status.txt");out<<"running\n";checked(out);}
    auto tell=[&](int progress,const std::string& message){if(report)report(progress,message);};
    try {
        auto files=discover(input);const int n=int(files.size());
        if(n<2)throw std::runtime_error("At least two LAS/PCD/PLY files are required");
        {auto out=output(root/"parameters.txt");out<<"algorithm=Forest + 6DOF GICP + GTSAM Pose3\ninput_directory="<<io::utf8(input)<<"\nreference_node=0\nreference_file="<<io::utf8(files[0].filename())
            <<"\ncoarse_score_threshold="<<options.config.icp_threshold<<"\nregistration_voxel_m=0.15\nmerge_resolution_m="<<options.mergeResolution
            <<"\ndescriptor_neighbors="<<options.config.descriptor_near_num<<"\ndescriptor_min_edge_m="<<options.config.descriptor_min_len
            <<"\ndescriptor_max_edge_m="<<options.config.descriptor_max_len<<"\nforest_geo_distance_m="<<options.config.dis_geo_verify
            <<"\nforest_normal_difference="<<options.config.normal_geo_verify<<"\nforest_rough_distance="<<options.config.rough_dis_threshold
            <<"\ngicp_max_iterations=100\ngicp_max_correspondence_m=0.8\nmetric_distance_m=0.30\nminimum_final_overlap_per_direction=0.10\nmaximum_inlier_rmse_m=0.25"
            <<"\ncycle_rotation_deg="<<options.cycleRotationDegrees<<"\ncycle_translation_m="<<options.cycleTranslationMeters
            <<"\nrotation_sigma_deg=1 (all three axes)\ntranslation_sigma=max(0.05, pairwise bidirectional RMSE)\nrobust_huber=1.345\ngravity_constraint=none\nworld_frame=not_read\n";checked(out);}
        std::vector<PreparedCloud> clouds;clouds.reserve(n);
        for(int i=0;i<n;++i){auto cache=files[i].filename();cache+=".forestbin";
            tell(35*i/n,"Preparing station "+std::to_string(i+1)+"/"+std::to_string(n)+": "+io::utf8(files[i].filename()));
            clouds.push_back(io::prepareCached(files[i],root/".forest_cache"/cache,options.config,
                [&](int p,const std::string& m){tell((35*i+35*p/100)/n,m);}));
        }
        {auto out=output(root/"cloud_inventory.csv");out<<"node,file,original_points,sampled_points,forest_centers,triangles\n";
            for(int i=0;i<n;++i)out<<i<<','<<csv(io::utf8(files[i].filename()))<<','<<clouds[i].originalPointCount<<','<<clouds[i].sampled->size()<<','<<clouds[i].features.currCenter->size()<<','<<clouds[i].features.desc_.size()<<'\n';checked(out);}
        std::vector<Edge> edges;int done=0,total=n*(n-1)/2;
        for(int i=0;i<n;++i)for(int j=i+1;j<n;++j){
            for(int direction=0;direction<2;++direction){int source=direction?j:i,target=direction?i:j;
                const auto label=io::utf8(files[source].filename())+" -> "+io::utf8(files[target].filename());
                tell(35+50*done/total,"Forest pair: "+label);
                auto pair=registerPreparedPair(clouds[source],clouds[target],options.config,
                    [&](int p,const std::string& m){tell(35+(50*done+50*p/100)/total,label+": "+m);});
                savePairReport(root/"pairs"/fs::u8path(io::utf8(files[source].filename())+"_to_"+io::utf8(files[target].filename())+"_quality.txt"),pair);
                Edge edge;edge.source=source;edge.target=target;edge.active=pair.accepted;
                edge.state=pair.accepted?"reliable":pair.reason;edge.pair=std::move(pair);edges.push_back(std::move(edge));
                writeEdges(root/"pairwise_edges.csv",edges,files);
                if(edges.back().active)break;
            }++done;
        }
        tell(86,"Checking graph connectivity and SE3 cycle consistency ...");
        std::vector<Cycle> audit;auto initial=validateCycles(n,edges,options,audit);
        writeCycles(root/"cycle_checks.csv",audit);writeEdges(root/"pairwise_edges.csv",edges,files,&initial);
        int reliable=int(std::count_if(edges.begin(),edges.end(),[](const auto& e){return e.active;}));
        int loops=reliable-(n-1);
        tell(88,"Connected stations="+std::to_string(n)+", reliable edges="+std::to_string(reliable)+", independent loops="+std::to_string(loops));
        if(!loops)tell(88,"No validated loop available: graph has only tree constraints.");
        tell(90,"Optimizing six-degree-of-freedom poses with GTSAM ...");
        auto optimized=optimize(initial,edges);
        for(const auto& e:edges)if(e.active&&!good(edgeError(e,optimized.poses),options))throw std::runtime_error("Optimized graph failed edge residual validation");
        writeEdges(root/"pairwise_edges.csv",edges,files,&initial,&optimized.poses);
        fs::create_directories(root/"transforms");auto poses=output(root/"global_poses.csv");poses<<"node,file";
        for(int r=0;r<4;++r)for(int c=0;c<4;++c)poses<<",reference_from_station_"<<r<<c;poses<<'\n';
        for(int i=0;i<n;++i){poses<<i<<','<<csv(io::utf8(files[i].filename()));
            auto name=files[i].filename();name+="_global_transform.txt";auto matrix=output(root/"transforms"/name);
            for(int r=0;r<4;++r)for(int c=0;c<4;++c){poses<<','<<optimized.poses[i](r,c);matrix<<optimized.poses[i](r,c)<<(c==3?'\n':'\t');}
            poses<<'\n';checked(matrix);
        }checked(poses);
        tell(93,"Exporting point clouds in reference station coordinates ...");
        std::vector<uint64_t> originalCounts;for(const auto& c:clouds)originalCounts.push_back(c.originalPointCount);
        const auto exported=io::writeLas(root/"merged_global.las",files,originalCounts,optimized.poses,options.mergeResolution,
            [&](int p,const std::string& m){tell(93+6*p/100,m);});
        {auto out=output(root/"pose_graph_summary.txt");out<<"stations="<<n<<"\nreliable_edges="<<reliable<<"\nindependent_loops="<<loops
            <<"\ninitial_graph_error="<<optimized.before<<"\noptimized_graph_error="<<optimized.after<<"\ngtsam_iterations="<<optimized.iterations
            <<"\nreference_file="<<io::utf8(files[0].filename())<<"\nmerge_resolution_m="<<options.mergeResolution
            <<"\noutput_format=LAS\nexported_points="<<exported<<"\n";checked(out);}
        {auto out=output(root/"run_status.txt");out<<"completed\n";checked(out);}
        tell(100,"Forest global registration completed: "+io::utf8(root));
    } catch(const std::exception& e){auto out=output(root/"run_status.txt");out<<"failed\n"<<e.what()<<'\n';checked(out);throw;}
}

int runGlobalGraphSelfTest() {
    try {
        std::vector<Matrix> truth(4,Matrix::Identity());
        for(int i=1;i<4;++i){truth[i].topLeftCorner<3,3>()=(Eigen::AngleAxisd(0.17*i,Eigen::Vector3d::UnitX())*
            Eigen::AngleAxisd(-0.23*i,Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(0.31*i,Eigen::Vector3d::UnitZ())).toRotationMatrix();
            truth[i].topRightCorner<3,1>()=Eigen::Vector3d(4*i,-2*i,0.8*i);}
        std::vector<Edge> edges;
        for(auto [a,b]:{std::pair{0,1},std::pair{1,2},std::pair{2,3},std::pair{3,0},std::pair{0,2},std::pair{1,3}}){
            Edge e;e.source=a;e.target=b;e.active=true;e.state="reliable";e.pair.accepted=true;
            e.pair.transform=truth[b].inverse()*truth[a];e.pair.finalForward.overlap=e.pair.finalReverse.overlap=0.6;
            e.pair.finalForward.rmse=e.pair.finalReverse.rmse=0.1;edges.push_back(e);}
        GlobalOptions options;std::vector<Cycle> audit;auto initial=validateCycles(4,edges,options,audit);auto result=optimize(initial,edges);
        for(int i=0;i<4;++i)if((result.poses[i]-truth[i]).norm()>1e-7)throw std::runtime_error("SE3 factor direction / roll-pitch self-test failed");
        auto bad=edges;bad.back().pair.transform(0,3)+=5;bad.back().pair.finalForward.overlap=bad.back().pair.finalReverse.overlap=0.2;
        audit.clear();validateCycles(4,bad,options,audit);if(bad.back().active)throw std::runtime_error("Bad loop was not rejected");
        bool disconnected=false;try{std::vector<Edge> one{edges[0]};spanningTree(4,one);}catch(...){disconnected=true;}
        if(!disconnected)throw std::runtime_error("Disconnected graph was accepted");
        edges.back().pair.transform(0,3)+=0.04;initial=spanningTree(4,edges);result=optimize(initial,edges);
        if(!(result.after<result.before))throw std::runtime_error("GTSAM objective did not improve");
        std::cout<<"Forest graph self-test passed: SE3 direction, free roll/pitch, bad loop, disconnection, optimization."<<std::endl;return 0;
    }catch(const std::exception& e){std::cerr<<"Forest graph self-test failed: "<<e.what()<<std::endl;return 1;}
}
} // namespace forest
