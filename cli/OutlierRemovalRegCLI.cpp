#include <iostream>
#include <string>
#include <fstream>
#include <iomanip>
#include <filesystem>
#include <vector>
#include <algorithm>

#include <Preparation.h>
#include <RelationshipConstruction.h>
#include <Decoupling.h>

void printUsage(const char* progName)
{
    std::cout << "OutlierRemovalRegCLI - Outlier Removal Point Cloud Registration" << std::endl;
    std::cout << "Usage: " << progName << " --source <source.las> --target <target.las> --output <output_dir>" << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  --source <path>     Source point cloud file (required)" << std::endl;
    std::cout << "  --target <path>     Target point cloud file (required)" << std::endl;
    std::cout << "  --output <path>     Output directory (required)" << std::endl;
    std::cout << "  --center            Use geographic coordinates centering (optional, default: false)" << std::endl;
    std::cout << "  --resolution <float> Distance resolution (optional, default: 0.1)" << std::endl;
}

bool parseArgs(int argc, char* argv[], std::string& source, std::string& target, std::string& output, bool& center, float& resolution)
{
    center = false;
    resolution = 0.1f;
    source.clear();
    target.clear();
    output.clear();

    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        if (arg == "--source" && i + 1 < argc) { source = argv[++i]; }
        else if (arg == "--target" && i + 1 < argc) { target = argv[++i]; }
        else if (arg == "--output" && i + 1 < argc) { output = argv[++i]; }
        else if (arg == "--center") { center = true; }
        else if (arg == "--resolution" && i + 1 < argc) { resolution = std::stof(argv[++i]); }
        else if (arg == "--help" || arg == "-h") { return false; }
        else { std::cerr << "Unknown argument: " << arg << std::endl; return false; }
    }

    if (source.empty() || target.empty() || output.empty())
    {
        std::cerr << "Error: --source, --target, and --output are all required." << std::endl;
        return false;
    }
    return true;
}

int main(int argc, char* argv[])
{
    std::string sourceFile, targetFile, outputDir;
    bool isCenter;
    float resolution;

    if (!parseArgs(argc, argv, sourceFile, targetFile, outputDir, isCenter, resolution))
    {
        printUsage(argv[0]);
        return 1;
    }

    if (resolution <= 0.0f)
    {
        std::cerr << "Error: resolution must be > 0, got " << resolution << std::endl;
        return 1;
    }

    std::cout << "=== Outlier Removal Point Cloud Registration ===" << std::endl;
    std::cout << "Source:      " << sourceFile << std::endl;
    std::cout << "Target:      " << targetFile << std::endl;
    std::cout << "Output dir:  " << outputDir << std::endl;
    std::cout << "Resolution:  " << resolution << std::endl;
    std::cout << "Center:      " << (isCenter ? "true" : "false") << std::endl;
    std::cout << "------------------------------------------------" << std::endl;

    // Step 1: Build correspondences (load, downsample, keypoints, FPFH, matching)
    std::cout << "[1/2] Building correspondences..." << std::endl;
    std::shared_ptr<RelationshipConstruction> rc =
        std::make_shared<RelationshipConstruction>(sourceFile, targetFile, resolution, isCenter);

    pcl::Correspondences cors = rc->getCorrespondences();
    PointCloudPtr sampledCloudS = rc->getPointCloudS();
    PointCloudPtr sampledCloudT = rc->getPointCloudT();
    std::cout << "  Correspondences found: " << cors.size() << std::endl;

    // Step 2: Decouple (outlier removal + transformation computation)
    std::cout << "[2/2] Running decoupling algorithm..." << std::endl;
    Decoupling decoupling(sampledCloudS, sampledCloudT, cors, resolution);
    Eigen::Matrix4d transformation = decoupling.getTransformation();

    std::cout << "Transformation Matrix:" << std::endl;
    std::cout << std::fixed << std::setprecision(6);
    for (int r = 0; r < 4; ++r)
    {
        std::cout << "  ";
        for (int c = 0; c < 4; ++c)
            std::cout << std::setw(12) << transformation(r, c);
        std::cout << std::endl;
    }

    // Write output file
    std::filesystem::path srcPath(sourceFile);
    std::filesystem::path tgtPath(targetFile);
    std::string outFileName = srcPath.stem().string() + "_to_" + tgtPath.stem().string() + "_transformationMatrix.txt";
    std::filesystem::path outPath = std::filesystem::path(outputDir) / outFileName;

    std::ofstream ofs(outPath);
    if (!ofs)
    {
        std::cerr << "Error: Cannot create output file: " << outPath << std::endl;
        return 1;
    }

    ofs << std::fixed << std::setprecision(6);
    for (int r = 0; r < 4; ++r)
    {
        for (int c = 0; c < 4; ++c)
        {
            if (c > 0) ofs << "\t";
            ofs << transformation(r, c);
        }
        ofs << "\n";
    }
    ofs.close();

    std::cout << "------------------------------------------------" << std::endl;
    std::cout << "Output saved to: " << outPath.string() << std::endl;
    std::cout << "Done." << std::endl;
    return 0;
}
