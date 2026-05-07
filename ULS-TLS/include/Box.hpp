#pragma once
#include "Point.hpp"

// Point class temeplate for D-dimensional space
template<typename T, size_t D>
class Box {
public:
    Box();
    Box(const Point<T, D>& minPt, const Point<T, D>& maxPt);

    const Point<T, D>& min() const;
    const Point<T, D>& max() const;

    void setMin(const Point<T, D>& pt);
    void setMax(const Point<T, D>& pt);

    Point<T, D> center() const;
    Point<T, D> extent() const;
    T volume() const;

    bool contains(const Point<T, D>& point, T epsilon = T(0)) const;
    bool intersects(const Box<T, D>& other, T epsilon = T(0)) const;

    void expandToInclude(const Point<T, D>& point);
    void expandToInclude(const Box<T, D>& other);

private:
    Point<T, D> _min;
    Point<T, D> _max;
};

// ---------------- Implementation ----------------

template<typename T, size_t D>
Box<T, D>::Box() {
    std::array<T, D> max_vals;
    max_vals.fill(std::numeric_limits<T>::lowest());
    std::array<T, D> min_vals;
    min_vals.fill(std::numeric_limits<T>::max());
    _min = Point<T, D>(min_vals);
    _max = Point<T, D>(max_vals);
}

template<typename T, size_t D>
Box<T, D>::Box(const Point<T, D>& minPt, const Point<T, D>& maxPt)
{
    auto min_coords = minPt.coords();
    auto max_coords = maxPt.coords();
    for (size_t i = 0; i < D; ++i) {
        if (min_coords[i] > max_coords[i])
            std::swap(min_coords[i], max_coords[i]);
    }
    _min = Point<T, D>(min_coords);
    _max = Point<T, D>(max_coords);
}

template<typename T, size_t D>
const Point<T, D>& Box<T, D>::min() const { return _min; }

template<typename T, size_t D>
const Point<T, D>& Box<T, D>::max() const { return _max; }

template<typename T, size_t D>
void Box<T, D>::setMin(const Point<T, D>& pt) 
{ 
    auto new_min = pt.coords();
    auto current_max = _max.coords();
    for (size_t i = 0; i < D; ++i) {
        if (new_min[i] > current_max[i])
            current_max[i] = new_min[i];
    }
    _min = Point<T, D>(new_min);
    _max = Point<T, D>(current_max);
}

template<typename T, size_t D>
void Box<T, D>::setMax(const Point<T, D>& pt) 
{ 
    auto new_max = pt.coords();
    auto current_min = _min.coords();
    for (size_t i = 0; i < D; ++i) {
        if (new_max[i] < current_min[i])
            current_min[i] = new_max[i];
    }
    _min = Point<T, D>(current_min);
    _max = Point<T, D>(new_max);
}

template<typename T, size_t D>
Point<T, D> Box<T, D>::center() const {
    return (_min + _max) / static_cast<T>(2);
}

template<typename T, size_t D>
Point<T, D> Box<T, D>::extent() const {
    return _max - _min;
}

template<typename T, size_t D>
T Box<T, D>::volume() const {
    T vol = static_cast<T>(1);
    auto ext = extent().coords();
    for (size_t i = 0; i < D; ++i)
        vol *= ext[i];
    return vol;
}

template<typename T, size_t D>
bool Box<T, D>::contains(const Point<T, D>& point, T epsilon) const {
    const auto& p = point.coords();
    const auto& min_c = _min.coords();
    const auto& max_c = _max.coords();
    for (size_t i = 0; i < D; ++i) {
        if (p[i] < min_c[i] - epsilon || p[i] > max_c[i] + epsilon)
            return false;
    }
    return true;
}

template<typename T, size_t D>
bool Box<T, D>::intersects(const Box<T, D>& other, T epsilon) const {
    const auto& a_min = _min.coords();
    const auto& a_max = _max.coords();
    const auto& b_min = other.min().coords();
    const auto& b_max = other.max().coords();

    for (size_t i = 0; i < D; ++i) {
        if (a_max[i] + epsilon < b_min[i] || b_max[i] + epsilon < a_min[i])
            return false;
    }
    return true;
}

template<typename T, size_t D>
void Box<T, D>::expandToInclude(const Point<T, D>& point) {
    auto p = point.coords();
    auto min_c = _min.coords();
    auto max_c = _max.coords();
    for (size_t i = 0; i < D; ++i) {
        if (p[i] < min_c[i]) min_c[i] = p[i];
        if (p[i] > max_c[i]) max_c[i] = p[i];
    }
    _min = Point<T, D>(min_c);
    _max = Point<T, D>(max_c);
}

template<typename T, size_t D>
void Box<T, D>::expandToInclude(const Box<T, D>& other) {
    expandToInclude(other.min());
    expandToInclude(other.max());
}
