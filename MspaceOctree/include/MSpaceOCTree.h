#pragma once
#include "MSpaceOCTree_global.h"

#include <osg/PagedLOD>
#include "converter_utils.h"
#include <iostream>
#include <execution>

#include "../modules/unsuck/unsuck.hpp"
#include "chunker_countsort_laszip.h"
#include "indexer.h"
#include "sampler_poisson.h"
#include "sampler_poisson_average.h"
#include "sampler_random.h"
#include "Attributes.h"
#include "OCTree.h"
#include "MSpaceNode.h" 

struct Curated {
    string name;
    vector<Source> files;
};
struct Stats {
    Vector3 min = { Infinity , Infinity , Infinity };
    Vector3 max = { -Infinity , -Infinity , -Infinity };
    int64_t totalBytes = 0;
    int64_t totalPoints = 0;
};

class MSPACEOCTREE_LIB_EXPORT MSpaceOCTree
{
public:
	MSpaceOCTree(vector<string> inputPaths, string outputPath/*, string samplingMethod = "poisson", string encoding = "DEFAULT"*/);
	~MSpaceOCTree();

    bool filesConverter(osg::ref_ptr<osg::MSpaceNode> lodStructure);

private:
    Curated curateSources(vector<string> paths);
    Stats computeStats(vector<Source> sources);
    void chunking(Options& options, vector<Source>& sources, string targetDir, Stats& stats, State& state, Attributes outputAttributes);
    void indexing(Options& options, string targetDir, State& state);


private:
    Options _options;
};


