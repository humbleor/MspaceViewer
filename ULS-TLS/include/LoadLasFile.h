#pragma once
#include "dll_expoart.h"
#include "utils.h"
#include <laszip/laszip_api.h>

// 加载las文件
void  DLL_EXPORT loadLasFile(std::string inputPath, PointCloud3fPtr pointcloud);
// 输出las文件
void DLL_EXPORT outputLasFile(std::string outputPath, PointCloud3fPtr pointcloud);

