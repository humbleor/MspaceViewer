#include "../include/NoiseExperiment.h"
#include "Decoupling.h"
#include "Preparation.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <unordered_set>
#include <vector>

namespace {
constexpr size_t pointCount=3000, trialCount=50;
constexpr double baseSigma=0.005, rotationThreshold=3.0, translationThreshold=0.5;
constexpr double angleResolutionDegrees=5.0;
constexpr unsigned experimentSeed=20260906U;
constexpr std::array<double,10> outlierRatios{
    0.50,0.60,0.70,0.80,0.90,0.95,0.96,0.97,0.98,0.99};
constexpr std::array<double,5> factors{1.0,2.0,3.0,4.0,5.0};

struct Instance {
    PointCloudPtr sourceBase,targetBase;
    std::vector<Eigen::Vector3f> sourceNoise,targetNoise;
    pcl::Correspondences correspondences;
    size_t inlierCount=0;
    Eigen::Matrix4d gt{Eigen::Matrix4d::Identity()};
};
struct Metrics { double re=0.0,te=0.0; bool success=false; };
struct Accumulator { double reSum=0.0,teSum=0.0; size_t successes=0; };
struct StageCounts { size_t tp=0,fp=0; };
struct StageAccumulator {
    double precisionSum=0.0,recallSum=0.0,timeMillisecondsSum=0.0;
};

Eigen::Vector3f sampleRadialNoise(
    std::mt19937& rng,
    std::normal_distribution<double>& normal,
    std::uniform_real_distribution<double>& angle)
{
    const double radialNoise=normal(rng);
    const double theta=angle(rng);
    const double phi=angle(rng);
    return Eigen::Vector3f(
        static_cast<float>(radialNoise*std::sin(phi)*std::cos(theta)),
        static_cast<float>(radialNoise*std::sin(phi)*std::sin(theta)),
        static_cast<float>(radialNoise*std::cos(phi)));
}

unsigned seedForOutlierRatio(double eta)
{
    for(size_t i=0;i<outlierRatios.size();++i) {
        if(std::abs(eta-outlierRatios[i])<1e-12)
            return experimentSeed+static_cast<unsigned>(i)*1009U;
    }
    return experimentSeed;
}

Instance generateInstance(std::mt19937& rng,double eta) {
    std::uniform_real_distribution<double> coordinate(-1.0,1.0),translation(-1.0,1.0);
    std::uniform_real_distribution<double> angle(-M_PI,M_PI);
    std::normal_distribution<double> normal(0.0,baseSigma);
    Instance result;
    result.sourceBase.reset(new pcl::PointCloud<pcl::PointXYZ>);
    result.targetBase.reset(new pcl::PointCloud<pcl::PointXYZ>);
    result.sourceBase->reserve(pointCount); result.targetBase->reserve(pointCount);
    result.sourceNoise.reserve(pointCount); result.targetNoise.reserve(pointCount);
    result.correspondences.reserve(pointCount);
    const size_t outlierCount=static_cast<size_t>(pointCount*eta);
    result.inlierCount=pointCount-outlierCount;
    for(size_t i=0;i<pointCount;++i) {
        pcl::PointXYZ sourcePoint(
            static_cast<float>(coordinate(rng)),
            static_cast<float>(coordinate(rng)),
            static_cast<float>(coordinate(rng)));
        pcl::PointXYZ targetPoint=sourcePoint;
        if(i>=result.inlierCount) {
            targetPoint.x=static_cast<float>(coordinate(rng));
            targetPoint.y=static_cast<float>(coordinate(rng));
            targetPoint.z=static_cast<float>(coordinate(rng));
        }
        result.sourceBase->push_back(sourcePoint);
        result.targetBase->push_back(targetPoint);
        result.sourceNoise.emplace_back(sampleRadialNoise(rng,normal,angle));
        result.targetNoise.emplace_back(sampleRadialNoise(rng,normal,angle));
        result.correspondences.emplace_back(static_cast<int>(i),static_cast<int>(i),0.0f);
    }
    const double yaw=angle(rng),c=std::cos(yaw),sn=std::sin(yaw);
    result.gt(0,0)=c; result.gt(0,1)=-sn; result.gt(1,0)=sn; result.gt(1,1)=c;
    result.gt(0,3)=translation(rng); result.gt(1,3)=translation(rng); result.gt(2,3)=translation(rng);
    return result;
}

void buildClouds(const Instance& instance,double factor,PointCloudPtr& source,PointCloudPtr& target) {
    source.reset(new pcl::PointCloud<pcl::PointXYZ>(*instance.sourceBase));
    target.reset(new pcl::PointCloud<pcl::PointXYZ>(*instance.targetBase));
    for(size_t i=0;i<pointCount;++i) {
        source->points[i].x+=static_cast<float>(factor*instance.sourceNoise[i].x());
        source->points[i].y+=static_cast<float>(factor*instance.sourceNoise[i].y());
        source->points[i].z+=static_cast<float>(factor*instance.sourceNoise[i].z());
        target->points[i].x+=static_cast<float>(factor*instance.targetNoise[i].x());
        target->points[i].y+=static_cast<float>(factor*instance.targetNoise[i].y());
        target->points[i].z+=static_cast<float>(factor*instance.targetNoise[i].z());
    }
    pcl::transformPointCloud(*target,*target,instance.gt.cast<float>());
    source->width=target->width=static_cast<uint32_t>(pointCount);
    source->height=target->height=1;
    source->is_dense=target->is_dense=true;
}

uint64_t correspondenceKey(const pcl::Correspondence& correspondence) {
    return (static_cast<uint64_t>(static_cast<uint32_t>(correspondence.index_query))<<32) |
        static_cast<uint32_t>(correspondence.index_match);
}

StageCounts countStage(const pcl::Correspondences& correspondences,size_t inlierCount) {
    StageCounts counts;
    std::unordered_set<uint64_t> observed;
    observed.reserve(correspondences.size());
    for(const pcl::Correspondence& correspondence:correspondences) {
        if(!observed.emplace(correspondenceKey(correspondence)).second) continue;
        const bool isTrueInlier=correspondence.index_query==correspondence.index_match &&
            correspondence.index_query>=0 &&
            static_cast<size_t>(correspondence.index_query)<inlierCount;
        if(isTrueInlier) ++counts.tp;
        else ++counts.fp;
    }
    return counts;
}

double safeRatio(size_t numerator,size_t denominator) {
    return denominator==0?0.0:static_cast<double>(numerator)/static_cast<double>(denominator);
}

Metrics evaluate(const Eigen::Matrix4d& estimated,const Eigen::Matrix4d& gt,bool valid) {
    const Eigen::Matrix4d delta=gt*estimated.inverse();
    const double cosine=std::clamp((delta.block<3,3>(0,0).trace()-1.0)*0.5,-1.0,1.0);
    Metrics result;
    result.re=std::acos(cosine)*180.0/M_PI;
    result.te=delta.block<3,1>(0,3).norm();
    result.success=valid&&std::isfinite(result.re)&&std::isfinite(result.te)&&
        result.re<rotationThreshold&&result.te<translationThreshold;
    return result;
}
}

int runNoiseSensitivityExperiment(double eta,double resolution) {
    if(!std::isfinite(eta)||eta<0.0||eta>=1.0||!std::isfinite(resolution)||resolution<=0.0) {
        std::cerr<<"Invalid parameters: eta must be in [0,1), resolution must be positive.\n";
        return 2;
    }
    std::mt19937 rng(seedForOutlierRatio(eta));
    std::vector<Instance> instances; instances.reserve(trialCount);
    for(size_t trial=0;trial<trialCount;++trial) instances.emplace_back(generateInstance(rng,eta));

    std::ofstream detail("noise_sensitivity_detail.csv"),summary("noise_sensitivity_summary.csv");
    std::ofstream stageDetail("noise_stage_detail.csv"),stageSummary("noise_stage_summary.csv");
    if(!detail||!summary||!stageDetail||!stageSummary) { std::cerr<<"Cannot create noise result files.\n"; return 3; }
    detail<<"trial,outlier_ratio,resolution,noise_factor,noise_sigma,rotation_error_deg,translation_error,success\n";
    summary<<"outlier_ratio,resolution,noise_factor,noise_sigma,mean_rotation_error_deg,mean_translation_error,success_rate_percent\n";
    stageDetail<<"trial,outlier_ratio,resolution,noise_factor,noise_sigma,stage,tp,fp,n,precision,recall,time_ms\n";
    stageSummary<<"outlier_ratio,resolution,noise_factor,noise_sigma,stage,mean_precision,mean_recall,mean_time_ms\n";
    std::array<Accumulator,factors.size()> accumulated{};
    std::array<std::array<StageAccumulator,4>,factors.size()> stageAccumulated{};

    for(size_t trial=0;trial<trialCount;++trial) {
        for(size_t fi=0;fi<factors.size();++fi) {
            const double sigma=baseSigma*factors[fi];
            const double effectiveResolution=resolution*factors[fi];
            PointCloudPtr source,target;
            buildClouds(instances[trial],factors[fi],source,target);
            Decoupling solver(source,target,instances[trial].correspondences,static_cast<float>(effectiveResolution),Eigen::Vector3d::UnitZ(),false,static_cast<float>(angleResolutionDegrees));
            const Metrics metrics=evaluate(solver.getTransformation(),instances[trial].gt,solver.isValid());
            accumulated[fi].reSum+=metrics.re; accumulated[fi].teSum+=metrics.te;
            if(metrics.success) ++accumulated[fi].successes;
            detail<<std::setprecision(10)<<trial<<','<<eta<<','<<effectiveResolution<<','<<factors[fi]<<','<<sigma<<','
                <<metrics.re<<','<<metrics.te<<','<<(metrics.success?1:0)<<'\n';

            const auto& stageTimes=solver.getStageTimesSeconds();
            const size_t groundTruthInliers=instances[trial].inlierCount;
            for(size_t stage=0;stage<4;++stage) {
                const StageCounts counts=countStage(solver.getStageCorrespondences(stage),instances[trial].inlierCount);
                const double precision=safeRatio(counts.tp,counts.tp+counts.fp);
                const double recall=safeRatio(counts.tp,groundTruthInliers);
                const double timeMilliseconds=stageTimes[stage]*1000.0;
                stageAccumulated[fi][stage].precisionSum+=precision;
                stageAccumulated[fi][stage].recallSum+=recall;
                stageAccumulated[fi][stage].timeMillisecondsSum+=timeMilliseconds;
                stageDetail<<std::setprecision(10)<<trial<<','<<eta<<','<<effectiveResolution<<','<<factors[fi]<<','<<sigma<<','
                    <<(stage+1)<<','<<counts.tp<<','<<counts.fp<<','<<(counts.tp+counts.fp)<<','
                    <<precision<<','<<recall<<','<<timeMilliseconds<<'\n';
            }
        }
    }

    std::cout<<std::fixed<<std::setprecision(6)<<"N="<<pointCount<<", eta="<<eta<<", base_sigma="<<baseSigma
        <<", base_resolution="<<resolution<<", resolution_over_sigma="<<(resolution/baseSigma)<<", angle_resolution_deg="<<angleResolutionDegrees<<", trials="<<trialCount<<'\n'
        <<"Noise factor, Sigma, Mean RE (deg), Mean TE, Success Rate (%)\n";
    for(size_t fi=0;fi<factors.size();++fi) {
        const double meanRE=accumulated[fi].reSum/trialCount,meanTE=accumulated[fi].teSum/trialCount;
        const double successRate=100.0*accumulated[fi].successes/trialCount,sigma=baseSigma*factors[fi];
        summary<<std::setprecision(10)<<eta<<','<<(resolution*factors[fi])<<','<<factors[fi]<<','<<sigma<<','<<meanRE<<','<<meanTE<<','<<successRate<<'\n';
        std::cout<<factors[fi]<<", "<<sigma<<", "<<meanRE<<", "<<meanTE<<", "<<successRate<<'\n';
    }
    for(size_t fi=0;fi<factors.size();++fi) {
        for(size_t stage=0;stage<4;++stage) {
            const StageAccumulator& value=stageAccumulated[fi][stage];
            stageSummary<<std::setprecision(10)<<eta<<','<<(resolution*factors[fi])<<','<<factors[fi]<<','
                <<(baseSigma*factors[fi])<<','<<(stage+1)<<','
                <<(value.precisionSum/trialCount)<<','<<(value.recallSum/trialCount)<<','
                <<(value.timeMillisecondsSum/trialCount)<<'\n';
        }
    }
    std::cout<<"Saved noise_sensitivity_detail.csv, noise_sensitivity_summary.csv, "
        <<"noise_stage_detail.csv and noise_stage_summary.csv\n";
    return 0;
}