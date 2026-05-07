#include "../include/MSpaceOCTree.h"
#include <osgDB/WriteFile>
#include "../include/potree_to_osg.h"

MSpaceOCTree::MSpaceOCTree(vector<string> inputPaths, string outputPath)
{
	_options.source = inputPaths;
	_options.outdir = outputPath;
}

MSpaceOCTree::~MSpaceOCTree()
{
}

bool MSpaceOCTree::filesConverter(osg::ref_ptr<osg::MSpaceNode> lodStructure)
{
    //返回源文件路径和所有文件的基本信息
    auto [name, sources] = curateSources(_options.source);
    if (_options.name.size() == 0) {
        _options.name = name;
    }

    //根据输入_options中的信息，确定输出文件中所包含的属性
    auto outputAttributes = computeOutputAttributes(sources, _options.attributes);

    //获取点云数据的最大值，最小值，点云个数和占用空间
    auto stats = computeStats(sources);
    //输出文件路径
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


//返回路径名称和所有文件的基本信息
Curated MSpaceOCTree::curateSources(vector<string> paths)
{

    string name = "";

    vector<string> expanded;
    for (auto path : paths) {
        if (fs::is_directory(path)) {
            for (auto& entry : fs::directory_iterator(path)) {
                string str = entry.path().string();

                if (iEndsWith(str, "las") || iEndsWith(str, "laz")) {
                    expanded.push_back(str);
                }
            }
        }
        else if (fs::is_regular_file(path)) {
            if (iEndsWith(path, "las") || iEndsWith(path, "laz")) {
                expanded.push_back(path);
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
