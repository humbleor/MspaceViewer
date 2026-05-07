#pragma once

#include <cmath>
#include <array>
#include <vector>
#include <memory>
#include <limits>
#include <string>
#include <algorithm>
#include <filesystem>
#include <iostream>
#include <regex>

// Point class template for D-dimensional space
template<typename T, size_t D>
class Point
{
public:
	Point();
	Point(const std::array<T, D>& coords);
	~Point();

	const std::array<T, D>& coords() const;
	std::array<T, D>& coords();
	void setCoords(const std::array<T, D>& coords);
	Point<T, D> operator+(const Point<T, D>& other) const;
	Point<T, D> operator-(const Point<T, D>& other) const;
	Point<T, D> operator*(const T& scalar) const;
	Point<T, D> operator*(const Point<T, D>& other) const;
	Point<T, D> operator/(const T& scalar) const;
	Point<T, D> operator/(const Point<T, D>& other) const;
	T dot(const Point<T, D>& other) const;
	T lengthSquared() const;
	T length() const;
	Point<T, D> normalize() const;

private:
	// The coordinates of the point in D-dimensional space
	std::array<T, D> _coordinates;
};

// ---------------- Implementation ----------------

template<typename T, size_t D>
Point<T, D>::Point()
{
	_coordinates.fill(T{});
}

template<typename T, size_t D>
Point<T, D>::Point(const std::array<T, D>& coords) : _coordinates(coords) {
}

template<typename T, size_t D>
Point<T, D>::~Point()
{
}

template<typename T, size_t D>
const std::array<T, D>& Point<T, D>::coords() const
{
	return _coordinates;
}

template<typename T, size_t D>
std::array<T, D>& Point<T, D>::coords()
{
	return _coordinates;
}

template<typename T, size_t D>
void Point<T, D>::setCoords(const std::array<T, D>& coords)
{
	_coordinates = coords;
}

template<typename T, size_t D>
Point<T, D> Point<T, D>::operator+(const Point<T, D>& other) const {
	Point<T, D> result;
	for (size_t i = 0; i < D; ++i) {
		result._coordinates[i] = this->_coordinates[i] + other._coordinates[i];
	}
	return result;
}

template<typename T, size_t D>
Point<T, D> Point<T, D>::operator-(const Point<T, D>& other) const {
	Point<T, D> result;
	for (size_t i = 0; i < D; ++i) {
		result._coordinates[i] = this->_coordinates[i] - other._coordinates[i];
	}
	return result;
}

template<typename T, size_t D>
Point<T, D> Point<T, D>::operator*(const T& scalar) const
{
	Point<T, D> result;
	for (size_t i = 0; i < D; ++i) {
		result._coordinates[i] = this->_coordinates[i] * scalar;
	}
	return result;
}

template<typename T, size_t D>
Point<T, D> Point<T, D>::operator*(const Point<T, D>& other) const {
	Point<T, D> result;
	for (size_t i = 0; i < D; ++i) {
		result._coordinates[i] = this->_coordinates[i] * other._coordinates[i];
	}
	return result;
}

template<typename T, size_t D>
Point<T, D> Point<T, D>::operator/(const T& scalar) const
{
	if constexpr (std::is_floating_point<T>::value) {
		if (std::abs(scalar) < std::numeric_limits<T>::epsilon())
			throw std::runtime_error("Division by near-zero");
	}
	else {
		if (scalar == T{})
			throw std::runtime_error("Division by zero");
	}
	Point<T, D> result;
	for (size_t i = 0; i < D; ++i) {
		result._coordinates[i] = this->_coordinates[i] / scalar;
	}
	return result;
}

template<typename T, size_t D>
Point<T, D> Point<T, D>::operator/(const Point<T, D>& other) const {
	Point<T, D> result;
	for (size_t i = 0; i < D; ++i) {
		if (other._coordinates[i] == T{})
			throw std::runtime_error("Division by zero at dimension " + std::to_string(i));
		result._coordinates[i] = this->_coordinates[i] / other._coordinates[i];
	}
	return result;
}

template<typename T, size_t D>
T Point<T, D>::dot(const Point<T, D>& other) const
{
	T result = T{};
	for (size_t i = 0; i < D; ++i) {
		result += this->_coordinates[i] * other._coordinates[i];
	}
	return result;
}

template<typename T, size_t D>
T Point<T, D>::lengthSquared() const
{
	T result = T{};
	result = this->dot(*this);
	return result;
}

template<typename T, size_t D>
T Point<T, D>::length() const
{
	return sqrt(lengthSquared());
}

template<typename T, size_t D>
Point<T, D> Point<T, D>::normalize() const
{
	T len = this->length();
	if (len == T{})
		throw std::runtime_error("Cannot normalize a zero-length vector");
	return *this / len;
}


