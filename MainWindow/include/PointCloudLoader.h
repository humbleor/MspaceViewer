#pragma once

#include <string>
#include <filesystem>
#include <algorithm>
#include <fstream>
#include <iostream>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <liblas/liblas.hpp>

namespace pcutil
{

inline std::string toLower(const std::string& str)
{
    std::string result = str;
    std::transform(result.begin(), result.end(), result.begin(), ::tolower);
    return result;
}

inline std::string getExtension(const std::string& filePath)
{
    return toLower(std::filesystem::path(filePath).extension().string());
}

inline bool isPointCloudFile(const std::string& filePath)
{
    std::string ext = getExtension(filePath);
    return ext == ".las" || ext == ".laz" || ext == ".pcd" || ext == ".ply";
}

inline bool loadLAS(const std::string& filePath, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    std::ifstream ifs(filePath.c_str(), std::ios::in | std::ios::binary);
    if (!ifs.is_open())
    {
        std::cerr << "Error: Could not open LAS file " << filePath << std::endl;
        return false;
    }

    liblas::ReaderFactory f;
    liblas::Reader reader = f.CreateWithStream(ifs);
    unsigned long int nbPoints = reader.GetHeader().GetPointRecordsCount();

    cloud->points.reserve(nbPoints);
    while (reader.ReadNextPoint())
    {
        pcl::PointXYZ p;
        p.x = reader.GetPoint().GetX();
        p.y = reader.GetPoint().GetY();
        p.z = reader.GetPoint().GetZ();
        cloud->points.push_back(p);
    }

    cloud->width = nbPoints;
    cloud->height = 1;
    cloud->is_dense = false;
    return true;
}

inline bool loadPCD(const std::string& filePath, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filePath, *cloud) == -1)
    {
        std::cerr << "Error: Could not load PCD file " << filePath << std::endl;
        return false;
    }
    return true;
}

inline bool loadPLY(const std::string& filePath, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    pcl::PointCloud<pcl::PointXYZ> tmp;
    if (pcl::io::loadPLYFile(filePath, tmp) < 0)
    {
        std::cerr << "Error: Could not load PLY file " << filePath << std::endl;
        return false;
    }
    *cloud = tmp;
    return true;
}

inline bool loadPointCloud(const std::string& filePath, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    std::string ext = getExtension(filePath);

    if (ext == ".las" || ext == ".laz")
    {
        return loadLAS(filePath, cloud);
    }
    else if (ext == ".pcd")
    {
        return loadPCD(filePath, cloud);
    }
    else if (ext == ".ply")
    {
        return loadPLY(filePath, cloud);
    }
    else
    {
        std::cerr << "Error: Unsupported file format " << ext << std::endl;
        return false;
    }
}

inline bool loadPointCloudToPCL(const std::string& filePath, pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr(new pcl::PointCloud<pcl::PointXYZ>);
    if (!loadPointCloud(filePath, ptr))
        return false;
    cloud = *ptr;
    return true;
}

} // namespace pcutil
