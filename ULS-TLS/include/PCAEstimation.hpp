#include "PointCloud.hpp"

// PCAEstimator定义
template <typename T, size_t D>
class PCAEstimator
{
public:
	using Point = Point<T, D>;
	using PointCloud = PointCloud<T, D>;

    PCAEstimator(const PointCloud& cloud);
    ~PCAEstimator();

    // 输出：特征值 eigenvalues[D]（降序），特征向量 eigenvectors[D][D]（列为特征向量）
    bool compute();
	std::array<T, D> eigenvalues() const { return _eigenvalues; }
	std::array<std::array<T, D>, D> eigenvectors() const { return _eigenvectors; }

private:
    const PointCloud& _cloud;
    std::array<T, D> _eigenvalues;
	std::array<std::array<T, D>, D> _eigenvectors;

    bool jacobiEigenDecomposition(const std::array<std::array<T, D>, D>& A,
        std::array<T, D>& eigenvalues,
        std::array<std::array<T, D>, D>& eigenvectors,
        size_t max_iter = 100, T tol = 1e-10);
};

template<typename T, size_t D>
PCAEstimator<T, D>::PCAEstimator(const PointCloud& cloud) : _cloud(cloud) {
}

template<typename T, size_t D>
inline PCAEstimator<T, D>::~PCAEstimator()
{
}


template<typename T, size_t D>
bool PCAEstimator<T, D>::compute()
{

    // 2. 计算均值
    std::array<T, D> mean = { 0 };
    for (const auto& pt : _cloud.points())
    {
        for (size_t i = 0; i < D; ++i)
            mean[i] += pt.coords()[i];
    }
    for (size_t i = 0; i < D; ++i)
        mean[i] /= static_cast<T>(_cloud.points().size());

    // 3. 协方差矩阵
    std::array<std::array<T, D>, D> cov{};
    for (const auto& pt : _cloud.points())
    {
        std::array<T, D> diff;
        for (size_t i = 0; i < D; ++i)
            diff[i] = pt.coords()[i] - mean[i];

        for (size_t i = 0; i < D; ++i)
            for (size_t j = 0; j < D; ++j)
                cov[i][j] += diff[i] * diff[j];
    }
    if (_cloud.points().size() <= 1) return false;
    T scale = static_cast<T>(_cloud.points().size() - 1);
    for (size_t i = 0; i < D; ++i)
        for (size_t j = 0; j < D; ++j)
            cov[i][j] /= scale;

    // 4. 特征分解：Jacobi method（仅支持对称矩阵）
    return jacobiEigenDecomposition(cov, _eigenvalues, _eigenvectors);
}

template<typename T, size_t D>
bool PCAEstimator<T, D>::jacobiEigenDecomposition(const std::array<std::array<T, D>, D>& A, std::array<T, D>& eigenvalues, std::array<std::array<T, D>, D>& eigenvectors, size_t max_iter, T tol)
{
    std::array<std::array<T, D>, D> V = {};  // 初始为单位矩阵
    for (size_t i = 0; i < D; ++i)
        V[i][i] = 1;

    std::array<std::array<T, D>, D> A_curr = A;

    for (size_t iter = 0; iter < max_iter; ++iter)
    {
        // 找最大非对角元
        size_t p = 0, q = 1;
        T max_val = std::abs(A_curr[0][1]);
        for (size_t i = 0; i < D; ++i)
        {
            for (size_t j = i + 1; j < D; ++j)
            {
                T val = std::abs(A_curr[i][j]);
                if (val > max_val)
                {
                    max_val = val;
                    p = i;
                    q = j;
                }
            }
        }

        if (max_val < tol)
            break;

        T theta = 0.5 * std::atan2(2 * A_curr[p][q], A_curr[q][q] - A_curr[p][p]);
        T cos_t = std::cos(theta);
        T sin_t = std::sin(theta);

        // 旋转 A_curr
        std::array<std::array<T, D>, D> A_next = A_curr;
        for (size_t i = 0; i < D; ++i)
        {
            if (i != p && i != q)
            {
                T aip = A_curr[i][p], aiq = A_curr[i][q];
                A_next[i][p] = A_next[p][i] = cos_t * aip - sin_t * aiq;
                A_next[i][q] = A_next[q][i] = sin_t * aip + cos_t * aiq;
            }
        }

        T app = A_curr[p][p], aqq = A_curr[q][q], apq = A_curr[p][q];
        A_next[p][p] = cos_t * cos_t * app - 2 * sin_t * cos_t * apq + sin_t * sin_t * aqq;
        A_next[q][q] = sin_t * sin_t * app + 2 * sin_t * cos_t * apq + cos_t * cos_t * aqq;
        A_next[p][q] = A_next[q][p] = 0;

        A_curr = A_next;

        // 旋转 V
        for (size_t i = 0; i < D; ++i)
        {
            T vip = V[i][p], viq = V[i][q];
            V[i][p] = cos_t * vip - sin_t * viq;
            V[i][q] = sin_t * vip + cos_t * viq;
        }
    }

    // 提取特征值和特征向量
    for (size_t i = 0; i < D; ++i)
    {
        eigenvalues[i] = A_curr[i][i];
        for (size_t j = 0; j < D; ++j)
            eigenvectors[j][i] = V[j][i];  // 每列为特征向量
    }

    // 按特征值降序排序
    std::array<size_t, D> indices;
    for (size_t i = 0; i < D; ++i) indices[i] = i;

    std::sort(indices.begin(), indices.end(), [&](size_t a, size_t b) {
        return eigenvalues[a] > eigenvalues[b];
        });

    std::array<T, D> sorted_evals;
    std::array<std::array<T, D>, D> sorted_evecs;
    for (size_t i = 0; i < D; ++i)
    {
        sorted_evals[i] = eigenvalues[indices[i]];
        for (size_t j = 0; j < D; ++j)
            sorted_evecs[j][i] = eigenvectors[j][indices[i]];
    }

    eigenvalues = sorted_evals;
    eigenvectors = sorted_evecs;

    return true;
}
