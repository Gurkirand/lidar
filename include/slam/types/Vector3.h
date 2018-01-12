#ifndef SLAM_VECTOR3_H
#define SLAM_VECTOR3_H


#include <pcl/point_types.h>


namespace slam 
{

/** \brief Vector4f decorator for convenient handling.
*
*/
class Vector3 : public Eigen::Vector4f
{

public:
	Vector3(float x, float y, float z);

	Vector3(void);

	template<typename OtherDerived>
	Vector3(Eigen::MatrixBase <OtherDerived> const& other)
	: Eigen::Vector4f(other) {}

	Vector3(const pcl::PointXYZI &p);

	template<typename OtherDerived>
	Vector3& operator=(Eigen::MatrixBase <OtherDerived> const& rhs) {
		this->Eigen::Vector4f::operator=(rhs);
		return *this;
	}

	Vector3 &operator=(const pcl::PointXYZ &rhs);

	Vector3 &operator=(const pcl::PointXYZI &rhs);

	Vector3& operator+=(pcl::PointXYZI const& rhs);

	float x() const;

	float y() const;

	float z() const;

	float &x();

	float &y();

	float &z();

	// easy conversion
	operator pcl::PointXYZI();
};

Vector3 operator+(Vector3 const& lhs, pcl::PointXYZI const& rhs);

Vector3 operator+(pcl::PointXYZI const& lhs, Vector3 const& rhs);

} // end namespace slam

#endif //SLAM_VECTOR3_H

