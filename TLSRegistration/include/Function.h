#pragma once

#include "dll_export.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// pcl/visualization not available in vcpkg PCL - commented out as unused
// #include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/common/pca.h>
#include <pcl/features/normal_3d.h>                // ���㷨����
#include <pcl/ModelCoefficients.h>                 // ģ��ϵ��
#include <pcl/sample_consensus/ransac.h>           // RANSAC
#include <pcl/sample_consensus/method_types.h>     // ����������Ʒ���
#include <pcl/sample_consensus/model_types.h>      // ģ�Ͷ���
#include <pcl/segmentation/sac_segmentation.h>     // RANSAC�ָ�
#include <pcl/sample_consensus/sac_model_cylinder.h>// Բ��
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>  
#include <pcl/registration/transformation_estimation_svd.h> //svd
#include <pcl/registration/icp.h> // icp�㷨
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/random_sample.h>//��ȡ�̶������ĵ���
#include <pcl/filters/passthrough.h>
#include <pcl/filters/uniform_sampling.h> // ���Ȳ���
#include <fstream>
#include <pcl/registration/ndt.h>               // NDT��׼
#include <pcl/filters/approximate_voxel_grid.h> // �����˲�
#include <pcl/features/fpfh_omp.h> //fpfh���ټ����omp(��˲��м���)

using namespace std;

DLL_EXPORT void cloud_with_normal(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_normals);

DLL_EXPORT vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> compute_rhombus_pointclouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float R, int num_sectors);

DLL_EXPORT pcl::PointCloud<pcl::PointXYZ>::Ptr get_OverlapRhombus_pointclouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float R, float angle_start);

DLL_EXPORT vector<vector<pcl::PointXYZ>> get_rhombus_descriptors(pcl::PointCloud<pcl::PointXYZ>::Ptr raw_patch, float Angle_starting, float r, float step);

DLL_EXPORT float calculateStandardDeviation(const std::vector<float>& vec1, float diff);

DLL_EXPORT vector<vector<pcl::PointCloud<pcl::PointXYZ>>> get_PntInrhombus(pcl::PointCloud<pcl::PointXYZ>::Ptr raw_patch, float Angle_starting, float r, float step);

DLL_EXPORT Eigen::Matrix4f calculateFourParameterTransformation(const pcl::PointXYZ& source_point1, const pcl::PointXYZ& source_point2, const pcl::PointXYZ& target_point1, const pcl::PointXYZ& target_point2);

DLL_EXPORT float ComputeRmse_PCR(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudSrc, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudTgt, float dist);

DLL_EXPORT pcl::PointCloud<pcl::Normal>::Ptr compute_normal(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::search::KdTree<pcl::PointXYZ>::Ptr tree);

DLL_EXPORT pcl::PointCloud<pcl::FPFHSignature33>::Ptr compute_fpfh_feature(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::search::KdTree<pcl::PointXYZ>::Ptr tree);

