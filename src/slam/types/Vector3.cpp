#include "../../../include/slam/types/Vector3.h"

namespace slam
{

Vector3::Vector3(float x, float y, float z)
	: Eigen::Vector4f(x, y, z, 0) {}

Vector3::Vector3(void)
	: Eigen::Vector4f(0, 0, 0, 0) {}


Vector3::Vector3(pcl::PointXYZI const& p)
	: Eigen::Vector4f(p.x, p.y, p.z, 0) {}

Vector3& Vector3::operator=(pcl::PointXYZ const& rhs)
{
	x() = rhs.x;
	y() = rhs.y;
	z() = rhs.z;
	return *this;
}

Vector3& Vector3::operator=(pcl::PointXYZI const& rhs)
{
	x() = rhs.x;
	y() = rhs.y;
	z() = rhs.z;
	return *this;
}

Vector3& Vector3::operator+=(pcl::PointXYZI const& rhs)
{
	x() += rhs.x;
	y() += rhs.y;
	z() += rhs.z;
	return *this;
}

float Vector3::x() const { return (*this)(0); }

float Vector3::y() const { return (*this)(1); }

float Vector3::z() const { return (*this)(2); }

float& Vector3::x() { return (*this)(0); }

float& Vector3::y() { return (*this)(1); }

float& Vector3::z() { return (*this)(2); }

// easy conversion
Vector3::operator pcl::PointXYZI()
{
	pcl::PointXYZI dst;
	dst.x = x();
	dst.y = y();
	dst.z = z();
	dst.intensity = 0;
	return dst;
}

Vector3 operator+(Vector3 const& lhs, pcl::PointXYZI const& rhs)
{
	return Vector3(lhs.x() + rhs.x, lhs.y() + rhs.y, lhs.z() + rhs.z);
}

Vector3 operator+(pcl::PointXYZI const& lhs, Vector3 const& rhs)
{
	return Vector3(lhs.x + rhs.x(), lhs.y + rhs.y(), lhs.z + rhs.z());
}

}
