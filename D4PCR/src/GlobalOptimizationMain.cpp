#include "GlobalOptimizationPipeline.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <stdexcept>
#include <pcl/io/pcd_io.h>

int main(int argc, char* argv[])
{
    if (argc == 2 && std::string(argv[1]) == "--self-test")
        return d4pcr::global::runGlobalOptimizationSelfTest();
    if (argc == 8 && std::string(argv[1]) == "--diagnose-icp")
    {
        try
        {
            PointCloudPtr source(new pcl::PointCloud<pcl::PointXYZ>);
            PointCloudPtr target(new pcl::PointCloud<pcl::PointXYZ>);
            if (pcl::io::loadPCDFile(argv[2], *source) != 0 ||
                pcl::io::loadPCDFile(argv[3], *target) != 0)
                throw std::runtime_error("Cannot read cached PCD clouds.");
            std::ifstream input{std::filesystem::path(argv[4])};
            Eigen::Matrix4d initial;
            for (int row = 0; row < 4; ++row)
                for (int column = 0; column < 4; ++column)
                    if (!(input >> initial(row, column)))
                        throw std::runtime_error("Cannot read initial 4x4 matrix.");
            d4pcr::global::PipelineOptions options;
            options.resolution = std::stod(argv[6]);
            options.icpMaximumCorrespondenceDistance = 3.0 * options.resolution;
            options.icpMaximumIterations = std::stoi(argv[7]);
            const auto result = d4pcr::global::refineFourDofIcp(source, target, initial, options);
            std::ofstream output{std::filesystem::path(argv[5])};
            output << std::setprecision(12);
            for (int row = 0; row < 4; ++row)
                for (int column = 0; column < 4; ++column)
                    output << result.transformation(row, column) << (column == 3 ? '\n' : '\t');
            output.close();
            if (!output)
                throw std::runtime_error("Cannot write ICP matrix.");
            const bool accepted = result.converged &&
                result.correspondenceCount >= static_cast<std::size_t>(options.minimumIcpCorrespondences) &&
                result.overlap >= options.minimumIcpOverlap &&
                std::isfinite(result.rmse) && result.rmse <= options.icpMaximumCorrespondenceDistance;
            std::cout << "ICP diagnostic: iterations=" << result.iterations
                << ", converged=" << result.converged << ", overlap=" << result.overlap
                << ", rmse=" << result.rmse << ", accepted=" << accepted << std::endl;
            return accepted ? 0 : 2;
        }
        catch (const std::exception& error)
        {
            std::cerr << error.what() << std::endl;
            return 2;
        }
    }
    if (argc == 6 && std::string(argv[1]) == "--diagnose-pair")
    {
        try
        {
            d4pcr::global::PipelineOptions options;
            options.resolution = std::stod(argv[5]);
            options.d4PcrResidualThreshold = 2.0 * options.resolution;
            options.icpMaximumCorrespondenceDistance = 3.0 * options.resolution;
            return d4pcr::global::runPairwiseDiagnostics(argv[2], argv[3], argv[4], options);
        }
        catch (const std::exception& error)
        {
            std::cerr << error.what() << std::endl;
            return 2;
        }
    }
    const bool guiProgress = argc == 5 && std::string(argv[4]) == "--gui-progress";
    if (argc < 3 || argc > 5 || (argc == 5 && !guiProgress))
    {
        std::cerr << "Usage: D4PCRGlobalOptimization <point_cloud_dir> <output_dir> [resolution]\n"
                     "       D4PCRGlobalOptimization --self-test\n"
                     "       D4PCRGlobalOptimization --diagnose-icp <source.pcd> <target.pcd> <initial.txt> <output.txt> <resolution> <iterations>\n"
                     "       D4PCRGlobalOptimization --diagnose-pair <source> <target> <output> <resolution>\n";
        return 2;
    }
    try
    {
        d4pcr::global::PipelineOptions options;
        if (argc >= 4)
            options.resolution = std::stod(argv[3]);
        options.d4PcrResidualThreshold = 2.0 * options.resolution;
        options.icpMaximumCorrespondenceDistance = 3.0 * options.resolution;
        options.cycleTranslationThreshold = 4.0 * options.resolution;
        options.minimumTranslationSigma = std::max(0.25 * options.resolution, 0.005);
        d4pcr::global::ProgressCallback progress;
        if (guiProgress)
            progress = [](int value, const std::string& message) {
                std::cout << "D4PCR_PROGRESS\t" << value << '\t' << message << std::endl;
            };
        return d4pcr::global::runGlobalOptimizationPipeline(argv[1], argv[2], options, progress);
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 2;
    }
}
