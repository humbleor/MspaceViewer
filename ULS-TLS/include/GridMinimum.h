#pragma once
#include "utils.h"

class GridMinimumFilter
{
public:

    GridMinimumFilter(float resolution)
        : _resolution(resolution)
    {
        _input = std::make_shared<PointCloud3f>();
    }

    void setInputCloud(const PointCloud3fPtr cloud)
    {
        _input = cloud;
    }

    void filter(PointCloud3fPtr output)
    {
        output->clear();
		Box3f box = _input->boundingBox();
        std::unordered_map<std::pair<int, int>, Point3f, pair_hash> grid_map;

        for (const auto& pt : _input->points())
        {
            int ix = static_cast<int>(std::floor((pt.coords()[0] - box.min().coords()[0]) / _resolution));
            int iy = static_cast<int>(std::floor((pt.coords()[1] - box.min().coords()[1]) / _resolution));
            std::pair<int, int> key = { ix, iy };

            auto it = grid_map.find(key);
            if (it == grid_map.end() || pt.coords()[2] < it->second.coords()[2])
            {
                grid_map[key] = pt;
            }
        }

        for (const auto& [_, pt] : grid_map)
            output->addPoint(pt);
    }

private:
    PointCloud3fPtr _input;
    float _resolution;
};