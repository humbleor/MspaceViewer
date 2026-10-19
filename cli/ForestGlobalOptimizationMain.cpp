#include "utils/ForestGlobalRegistration.h"
#include "utils/ForestGlobalIO.h"
#include <QCoreApplication>
#include <QCommandLineParser>
#include <iostream>
#include <cmath>
#include <algorithm>

int main(int argc,char** argv) {
    QCoreApplication app(argc,argv);app.setApplicationName("ForestGlobalOptimization");
    QCommandLineParser parser;parser.setApplicationDescription("Forest + 6DOF GICP + GTSAM Pose3 global registration");parser.addHelpOption();
    parser.addOptions({{{"i","input"},"LAS/PCD/PLY input directory","directory"},
        {{"o","output"},"Output directory","directory"},
        {"coarse-score","Forest coarse match threshold","value","0.25"},
        {"neighbors","Triangle descriptor nearest neighbors","value","10"},
        {"min-edge-length","Minimum triangle edge (m)","value","2.0"},
        {"max-edge-length","Maximum triangle edge (m)","value","50.0"},
        {"geo-distance","Forest geometric verification distance (m)","value","0.3"},
        {"normal-difference","Forest normal verification threshold","value","1.0"},
        {"merge-resolution","LAS output voxel (m), 0 preserves all original points","value","0.05"},
        {"export-poses","Export LAS using a completed run without repeating registration","directory"},
        {"cycle-rotation","Cycle rotation threshold (deg)","value","2.0"},
        {"cycle-translation","Cycle translation threshold (m)","value","0.5"},
        {"gui-progress","Emit structured UTF-8 progress lines"},
        {"self-test","Verify SE3 graph direction, cycle rejection and optimization"}});
    parser.process(app);
    if(parser.isSet("self-test"))return forest::runGlobalGraphSelfTest();
    if(!parser.isSet("input")||!parser.isSet("output"))parser.showHelp(1);
    try {
        auto number=[&](const char* option){bool ok=false;double v=parser.value(option).toDouble(&ok);
            if(!ok||!std::isfinite(v))throw std::runtime_error(std::string("Invalid number: ")+option);return v;};
        forest::GlobalOptions options;
        options.config.icp_threshold=number("coarse-score");
        double neighbors=number("neighbors");
        if(neighbors!=std::floor(neighbors)||neighbors<3||neighbors>100)throw std::runtime_error("Neighbors must be an integer between 3 and 100");
        options.config.descriptor_near_num=int(neighbors);
        options.config.descriptor_min_len=number("min-edge-length");options.config.descriptor_max_len=number("max-edge-length");
        options.config.dis_geo_verify=number("geo-distance");options.config.normal_geo_verify=number("normal-difference");
        options.mergeResolution=number("merge-resolution");options.cycleRotationDegrees=number("cycle-rotation");options.cycleTranslationMeters=number("cycle-translation");
        if(options.config.descriptor_min_len<=0||options.config.descriptor_max_len<=options.config.descriptor_min_len||
            options.config.dis_geo_verify<=0||options.config.normal_geo_verify<0||options.config.normal_geo_verify>2||
            options.cycleRotationDegrees<=0||options.cycleRotationDegrees>180||options.cycleTranslationMeters<=0)
            throw std::runtime_error("Invalid Forest descriptor / geometry / cycle parameters");
        int last=0;const bool gui=parser.isSet("gui-progress");
        auto progress=[&](int percent,const std::string& message){last=std::max(last,std::clamp(percent,0,100));auto clean=message;
                for(char& c:clean)if(c=='\n'||c=='\r'||c=='\t')c=' ';
                if(gui)std::cout<<"FOREST_PROGRESS\t"<<last<<'\t'<<clean<<std::endl;
                else std::cout<<'['<<last<<"%] "<<clean<<std::endl;};
        const std::filesystem::path input(parser.value("input").toStdWString()),output(parser.value("output").toStdWString());
        if(parser.isSet("export-poses"))
            forest::io::exportLasFromRun(input,std::filesystem::path(parser.value("export-poses").toStdWString()),
                output/"merged_global.las",options.mergeResolution,progress);
        else forest::runGlobalRegistration(input,output,options,progress);
        return 0;
    }catch(const std::exception& e){std::cerr<<"Forest global registration failed: "<<e.what()<<std::endl;return 2;}
}
