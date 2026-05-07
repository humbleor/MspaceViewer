#pragma once
#include "Point.hpp"
#include "Box.hpp"

// PointCloud class temeplate for D-dimensional space
template<typename T, size_t D>
class PointCloud {
public:
    PointCloud();

    void addPoint(const Point<T, D>& point);
    void removePointAt(size_t index);
    size_t size() const;

    const std::vector<Point<T, D>>& points() const;
    Point<T, D>& operator[](size_t index);
    const Point<T, D>& operator[](size_t index) const;

    Box<T, D> boundingBox() const;
    void removeDuplicates(T epsilon = static_cast<T>(1e-6));

    void clear();
    void rotate(const std::array<std::array<T, D>, D>& rotationMatrix);
    void translate(const Point<T, D>& offset);
    void scale(T factor);
    void normalize();

private:
    std::vector<Point<T, D>> _points;
};

// ----------------- Implementation -----------------

template<typename T, size_t D>
PointCloud<T, D>::PointCloud() {}

template<typename T, size_t D>
void PointCloud<T, D>::addPoint(const Point<T, D>& point) {
    _points.emplace_back(point);
}

template<typename T, size_t D>
void PointCloud<T, D>::removePointAt(size_t index) {
    if (index >= _points.size()) throw std::out_of_range("Index out of range");
    _points.erase(_points.begin() + index);
}

template<typename T, size_t D>
size_t PointCloud<T, D>::size() const {
    return _points.size();
}

template<typename T, size_t D>
const std::vector<Point<T, D>>& PointCloud<T, D>::points() const {
    return _points;
}

template<typename T, size_t D>
Point<T, D>& PointCloud<T, D>::operator[](size_t index) {
    return _points[index];
}

template<typename T, size_t D>
const Point<T, D>& PointCloud<T, D>::operator[](size_t index) const {
    return _points[index];
}

template<typename T, size_t D>
Box<T, D> PointCloud<T, D>::boundingBox() const {
    if (_points.empty())
        throw std::runtime_error("Empty point cloud");

    Box<T, D> box(_points[0], _points[0]);
    for (const auto& pt : _points) {
        box.expandToInclude(pt);
    }
    return box;
}

template<typename T, size_t D>
void PointCloud<T, D>::removeDuplicates(T epsilon) {
    std::vector<Point<T, D>> unique;
    for (const auto& pt : _points) {
        bool exists = false;
        for (const auto& u : unique) {
            if ((pt - u).length() < epsilon) {
                exists = true;
                break;
            }
        }
        if (!exists) unique.push_back(pt);
    }
    _points = std::move(unique);
}

template<typename T, size_t D>
void PointCloud<T, D>::clear()
{
    _points.clear();
}
template<typename T, size_t D>
void PointCloud<T, D>::rotate(const std::array<std::array<T, D>, D>& rotationMatrix) {
    for (auto& point : _points) {
        const auto& coords = point.coords();
        std::array<T, D> rotatedCoords{};

        for (size_t i = 0; i < D; ++i) {
            T sum = T{};
            for (size_t j = 0; j < D; ++j) {
                sum += rotationMatrix[i][j] * coords[j];
            }
            rotatedCoords[i] = sum;
        }

        point.setCoords(rotatedCoords);
    }
}

template<typename T, size_t D>
void PointCloud<T, D>::translate(const Point<T, D>& offset) {
    for (auto& pt : _points) {
        pt = pt + offset;
    }
}

template<typename T, size_t D>
void PointCloud<T, D>::scale(T factor) {
    for (auto& pt : _points) {
        pt = pt * factor;
    }
}

template<typename T, size_t D>
void PointCloud<T, D>::normalize() {
    if (_points.empty()) return;

    Box<T, D> box = boundingBox();
    Point<T, D> min_pt = box.min();
    Point<T, D> range = box.extent();

    for (size_t i = 0; i < D; ++i) {
        if (range.coords()[i] == T{}) {
            throw std::runtime_error(std::string("Cannot normalize with zero extent in dimension ") + std::to_string(i));
        }
    }

    for (auto& pt : _points) {
        auto c = pt.coords();
        for (size_t i = 0; i < D; ++i) {
            c[i] = (c[i] - min_pt.coords()[i]) / range.coords()[i];
        }
        pt.setCoords(c);
    }
}