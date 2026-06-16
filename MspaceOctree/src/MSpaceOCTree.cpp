#include "../include/MSpaceOCTree.h"
#include <osgDB/WriteFile>
#include "../include/potree_to_osg.h"
#include <algorithm>

MSpaceOCTree::MSpaceOCTree(vector<string> inputPaths, string outputPath)
{
	_options.source = inputPaths;
	_options.outdir = outputPath;
}

MSpaceOCTree::~MSpaceOCTree()
{
}

bool MSpaceOCTree::isLasFile(const std::string& path)
{
    std::string ext = std::filesystem::path(path).extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    return ext == ".las" || ext == ".laz";
}

bool MSpaceOCTree::isPcdFile(const std::string& path)
{
    std::string ext = std::filesystem::path(path).extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    return ext == ".pcd";
}

bool MSpaceOCTree::isPlyFile(const std::string& path)
{
    std::string ext = std::filesystem::path(path).extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    return ext == ".ply";
}

std::string MSpaceOCTree::convertToLas(const std::string& filePath, const std::string& outputDir)
{
    std::string stem = std::filesystem::path(filePath).stem().string();
    std::string lasPath = outputDir + "/" + stem + ".las";

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (isPcdFile(filePath))
    {
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(filePath, *cloud) == -1)
        {
            std::cerr << "Error: Could not load PCD file " << filePath << std::endl;
            return "";
        }
    }
    else if (isPlyFile(filePath))
    {
        pcl::PointCloud<pcl::PointXYZ> tmp;
        if (pcl::io::loadPLYFile(filePath, tmp) < 0)
        {
            std::cerr << "Error: Could not load PLY file " << filePath << std::endl;
            return "";
        }
        *cloud = tmp;
    }
    else
    {
        return "";
    }

    if (cloud->empty())
    {
        std::cerr << "Error: Point cloud is empty " << filePath << std::endl;
        return "";
    }

    std::ofstream ofs;
    if (!liblas::Create(ofs, lasPath))
    {
        std::cerr << "Error: Could not create LAS file " << lasPath << std::endl;
        return "";
    }

    double min_x = cloud->points[0].x, max_x = cloud->points[0].x;
    double min_y = cloud->points[0].y, max_y = cloud->points[0].y;
    double min_z = cloud->points[0].z, max_z = cloud->points[0].z;

    for (const auto& p : cloud->points)
    {
        min_x = std::min(min_x, (double)p.x);
        max_x = std::max(max_x, (double)p.x);
        min_y = std::min(min_y, (double)p.y);
        max_y = std::max(max_y, (double)p.y);
        min_z = std::min(min_z, (double)p.z);
        max_z = std::max(max_z, (double)p.z);
    }

    int xoffset = int(min_x);
    int yoffset = int(min_y);
    int zoffset = int(min_z);

    liblas::Header header;
    header.SetVersionMajor(1);
    header.SetVersionMinor(2);
    header.SetOffset(xoffset, yoffset, zoffset);
    header.SetDataFormatId(liblas::ePointFormat3);
    header.SetPointRecordsCount(cloud->size());
    header.SetScale(0.0001, 0.0001, 0.0001);
    header.SetMin(min_x, min_y, min_z);
    header.SetMax(max_x, max_y, max_z);

    liblas::Writer writer(ofs, header);

    for (const auto& p : cloud->points)
    {
        liblas::Point point(&header);
        point.SetX(p.x);
        point.SetY(p.y);
        point.SetZ(p.z);
        writer.WritePoint(point);
    }

    writer.WriteHeader();
    ofs.close();

    return lasPath;
}

bool MSpaceOCTree::filesConverter(osg::ref_ptr<osg::MSpaceNode> lodStructure)
{
    //����Դ�ļ�·���������ļ��Ļ�����Ϣ
    auto [name, sources] = curateSources(_options.source);
    if (_options.name.size() == 0) {
        _options.name = name;
    }

    //��������_options�е���Ϣ��ȷ������ļ���������������
    auto outputAttributes = computeOutputAttributes(sources, _options.attributes);

    //��ȡ�������ݵ����ֵ����Сֵ�����Ƹ�����ռ�ÿռ�
    auto stats = computeStats(sources);
    //����ļ�·��
    string targetDir = _options.outdir;
    targetDir = targetDir;
    fs::create_directories(targetDir);
    State state;
    state.pointsTotal = stats.totalPoints;
    state.bytesProcessed = stats.totalBytes;

    { // this is the real important stuff

        chunking(_options, sources, targetDir, stats, state, outputAttributes);

        indexing(_options, targetDir, state);

    }

    potree_to_osg::convert(targetDir, lodStructure);

	return true;
}


//����·�����ƺ������ļ��Ļ�����Ϣ
Curated MSpaceOCTree::curateSources(vector<string> paths)
{

    string name = "";
    std::string tmpDir = _options.outdir + "/tmp_convert";
    fs::create_directories(tmpDir);

    vector<string> expanded;
    for (auto path : paths) {
        if (fs::is_directory(path)) {
            for (auto& entry : fs::directory_iterator(path)) {
                string str = entry.path().string();
                std::string ext = fs::path(str).extension().string();
                std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

                if (ext == ".las" || ext == ".laz") {
                    expanded.push_back(str);
                }
                else if (ext == ".pcd" || ext == ".ply") {
                    std::string lasPath = convertToLas(str, tmpDir);
                    if (!lasPath.empty()) {
                        expanded.push_back(lasPath);
                    }
                }
            }
        }
        else if (fs::is_regular_file(path)) {
            std::string ext = fs::path(path).extension().string();
            std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

            if (ext == ".las" || ext == ".laz") {
                expanded.push_back(path);
            }
            else if (ext == ".pcd" || ext == ".ply") {
                std::string lasPath = convertToLas(path, tmpDir);
                if (!lasPath.empty()) {
                    expanded.push_back(lasPath);
                }
            }
        }

        if (name.size() == 0) {
            name = fs::path(path).stem().string();
        }
    }
    paths = expanded;

    vector<Source> sources;
    sources.reserve(paths.size());

    mutex mtx;
    auto parallel = std::execution::par;
    for_each(parallel, paths.begin(), paths.end(), [&mtx, &sources](string path) {

        auto header = loadLasHeader(path);
        auto filesize = fs::file_size(path);

        Vector3 min = { header.min.x, header.min.y, header.min.z };
        Vector3 max = { header.max.x, header.max.y, header.max.z };

        Source source;
        source.path = path;
        source.min = min;
        source.max = max;
        source.numPoints = header.numPoints;
        source.filesize = filesize;

        lock_guard<mutex> lock(mtx);
        sources.push_back(source);
        });

    return { name, sources };
}

Stats MSpaceOCTree::computeStats(vector<Source> sources)
{

    Vector3 min = { Infinity , Infinity , Infinity };
    Vector3 max = { -Infinity , -Infinity , -Infinity };

    int64_t totalBytes = 0;
    int64_t totalPoints = 0;

    for (auto source : sources) {
        min.x = std::min(min.x, source.min.x);
        min.y = std::min(min.y, source.min.y);
        min.z = std::min(min.z, source.min.z);

        max.x = std::max(max.x, source.max.x);
        max.y = std::max(max.y, source.max.y);
        max.z = std::max(max.z, source.max.z);

        totalPoints += source.numPoints;
        totalBytes += source.filesize;
    }


    double cubeSize = (max - min).max();
    Vector3 size = { cubeSize, cubeSize, cubeSize };
    max = min + cubeSize;

    { // sanity check
        bool sizeError = (size.x == 0.0) || (size.y == 0.0) || (size.z == 0);
        if (sizeError) {
            exit(123);
        }

    }

    return { min, max, totalBytes, totalPoints };
}

void MSpaceOCTree::chunking(Options& options, vector<Source>& sources, string targetDir, Stats& stats, State& state, Attributes outputAttributes)
{
    chunker_countsort_laszip::doChunking(sources, targetDir, stats.min, stats.max, state, outputAttributes);
}

void MSpaceOCTree::indexing(Options& options, string targetDir, State& state)
{
    SamplerPoisson sampler;

    indexer::doIndexing(targetDir, state, options, sampler);

}
