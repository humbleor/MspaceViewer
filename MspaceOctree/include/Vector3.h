
#pragma once

#include <string>
#include <cmath>
#include <limits>
#include <sstream>
#include <algorithm>
#include <string>

using std::string;

//点云结构体
struct Vector3{

	double x = double(0.0);
	double y = double(0.0);
	double z = double(0.0);

	Vector3() {

	}

	Vector3(double x, double y, double z) {
		this->x = x;
		this->y = y;
		this->z = z;
	}

	Vector3(double value[3]) {
		this->x = value[0];
		this->y = value[1];
		this->z = value[2];
	}
	//计算计算this点到right点的平方距离
	double squaredDistanceTo(const Vector3& right) {
		double dx = right.x - x;
		double dy = right.y - y;
		double dz = right.z - z;

		double dd = dx * dx + dy * dy + dz * dz;

		return dd;
	}
	//计算this点到right点的距离
	double distanceTo(const Vector3& right) {
		double dx = right.x - x;
		double dy = right.y - y;
		double dz = right.z - z;

		double dd = dx * dx + dy * dy + dz * dz;
		double d = std::sqrt(dd);

		return d;
	}
	//计算this点到原点的长度
	double length() {
		return sqrt(x * x + y * y + z * z);
	}
	//获取this点三维坐标中的最大值
	double max() {
		double value = std::max(std::max(x, y), z);
		return value;
	}
	//重载-运算符(坐标向量)
	Vector3 operator-(const Vector3& right) const {
		return Vector3(x - right.x, y - right.y, z - right.z);
	}
	//重载+运算符(坐标向量)
	Vector3 operator+(const Vector3& right) const {
		return Vector3(x + right.x, y + right.y, z + right.z);
	}
	//重载+运算符(尺度标量)，这个重在一般用于地理数据无法进行高精度渲染显示问题。所以通常会减去一个极大值
	Vector3 operator+(const double& scalar) const {
		return Vector3(x + scalar, y + scalar, z + scalar);
	}
	//重载/运算符(尺度标量)，尺度缩小
	Vector3 operator/(const double& scalar) const {
		return Vector3(x / scalar, y / scalar, z / scalar);
	}
	//重载*运算符(坐标向量), 坐标相乘
	Vector3 operator*(const Vector3& right) const {
		return Vector3(x * right.x, y * right.y, z * right.z);
	}
	//重载*运算符(尺度标量)，尺度变大
	Vector3 operator*(const double& scalar) const {
		return Vector3(x * scalar, y * scalar, z * scalar);
	}
	//将坐标输出到控制台
	string toString() {

		auto digits = std::numeric_limits<double>::max_digits10;

		std::stringstream ss;
		ss << std::setprecision(digits);
		ss << x << ", " << y << ", " << z;

		return ss.str();

	}

};