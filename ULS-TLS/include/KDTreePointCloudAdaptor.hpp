#pragma once

#include "nanoflann.hpp"
#include "PointCloud.hpp"  // 您的 PointCloud 头文件

/**
 * @brief nanoflann KDTree 适配器，适配自定义的 PointCloud 类
 *
 * @tparam T 坐标数据类型（如 float/double）
 * @tparam D 点云维度（如3表示3D点云）
 * @tparam Distance 距离度量（默认L2）
 * @tparam IndexType 索引类型（默认size_t）
 */
template <
    typename T, int D = 3,
    class Distance = nanoflann::metric_L2,
    typename IndexType = size_t>
class KDTreePointCloudAdaptor
{
public:
    using self_t = KDTreePointCloudAdaptor<T, D, Distance, IndexType>;
    using metric_t = typename Distance::template traits<T, self_t>::distance_t;
    using index_t = nanoflann::KDTreeSingleIndexAdaptor<metric_t, self_t, D, IndexType>;

    // KDTree 索引指针
    index_t* index = nullptr;

    /**
     * @brief 构造函数
     *
     * @param cloud 点云数据（必须非空）
     * @param leaf_max_size 叶子节点最大点数（默认10）
     * @param n_thread_build 构建线程数（默认单线程）
     */
    explicit KDTreePointCloudAdaptor(
        const PointCloud<T, D>& cloud,
        int leaf_max_size = 10,
        unsigned int n_thread_build = 1
    ) : m_pointcloud(cloud)
    {
        // 检查点云有效性
        if (cloud.size() == 0)
            throw std::runtime_error("Point cloud is empty!");

        // 维度检查（如果D>0）
        if (D > 0 && cloud.points()[0].coords().size() != D)
            throw std::runtime_error("Point dimension mismatch!");

        // 创建索引
        index = new index_t(
            D,  // 维度
            *this,  // 适配器引用
            nanoflann::KDTreeSingleIndexAdaptorParams(
                leaf_max_size,
                nanoflann::KDTreeSingleIndexAdaptorFlags::None,
                n_thread_build
            )
        );

        // 构建索引
        index->buildIndex();
    }

    ~KDTreePointCloudAdaptor() { delete index; }

    /**
     * @brief 查询最近邻
     *
     * @param query_point 查询点坐标数组（长度=D）
     * @param num_closest 需要返回的最近邻数量
     * @param out_indices 输出：最近邻索引数组
     * @param out_distances_sq 输出：平方距离数组
     */
    void query(
        const T* query_point,
        size_t num_closest,
        IndexType* out_indices,
        T* out_distances_sq
    ) const
    {
        nanoflann::KNNResultSet<T, IndexType> result(num_closest);
        result.init(out_indices, out_distances_sq);
        index->findNeighbors(result, query_point);
    }

    // ----------------- nanoflann 接口实现 -----------------
    const self_t& derived() const { return *this; }
    self_t& derived() { return *this; }

    // 返回点云中的点数
    inline size_t kdtree_get_point_count() const {
        return m_pointcloud.size();
    }

    // 返回第idx个点的第dim维坐标
    inline T kdtree_get_pt(const size_t idx, const size_t dim) const {
        return m_pointcloud.points()[idx].coords()[dim];
    }

    // 可选：返回点云包围盒（加速构建）
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& bb) const {
        const auto& box = m_pointcloud.boundingBox();
        const auto& min = box.min().coords();
        const auto& max = box.max().coords();
        for (size_t i = 0; i < D; ++i) {
            bb[i].low = min[i];
            bb[i].high = max[i];
        }
        return true; // 表示已提供包围盒
    }

private:
    const PointCloud<T, D>& m_pointcloud; // 点云数据引用
};