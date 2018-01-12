#ifndef SLAM_MATH_UTILS_H
#define SLAM_MATH_UTILS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "common.h"
#include "types/Angle.h"
#include "types/Vector3.h"
#include "types/Rotation.h"

namespace slam
{

inline point_type point(float x, float y, float z, float intensity)
{
	point_type p;
	p.x = x;
	p.y = y;
	p.z = z;
	p.intensity = intensity;
	return p;
}

inline point_type& plus_eq (point_type& lhs, point_type const& rhs)
{
	lhs.x += rhs.x;
	lhs.y += rhs.y;
	lhs.z += rhs.z;
	return lhs;
}

inline point_type diff (point_type const& lhs, point_type const& rhs)
{
	point_type p;
	p.x = lhs.x - rhs.x;
	p.y = lhs.y - rhs.y;
	p.z = lhs.z - rhs.z;
	p.intensity = lhs.intensity - rhs.intensity;
	return p;
}

inline point_type scale (point_type const& p, float s)
{
	point_type sp;
	sp.x = p.x * s;
	sp.y = p.y * s;
	sp.z = p.z * s;
	sp.intensity = p.intensity * s;
	return sp;
}

inline point_type& scale_eq (point_type& p, float s)
{
	p.x *= s;
	p.y *= s;
	p.z *= s;
	p.intensity *= s;
	return p;
}

inline float sqr_diff (point_type const& lhs, point_type const& rhs)
{
	float x = lhs.x - rhs.x;
	float y = lhs.y - rhs.y;
	float z = lhs.z - rhs.z;
	return x*x + y*y + z*z;
}

inline float sqr_len (point_type const& p)
{
	return p.x*p.x + p.y*p.y + p.z*p.z;
}

inline float length (point_type const& p)
{
	return sqrt(sqr_len(p));
}

inline point_type& normalize (point_type& p)
{
	return scale_eq(p, 1 / length(p));
}

inline float dot_product (point_type const& lhs, point_type const& rhs)
{
	return (lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z);
}

inline point_type cross_product (point_type const& lhs, point_type const& rhs)
{
	float crossX = lhs.y * rhs.z - lhs.z * rhs.y;
	float crossY = lhs.x * rhs.z - lhs.z * rhs.x;
	float crossZ = lhs.x * rhs.y - lhs.y * rhs.x;
	point_type p;
	p.x = crossX;
	p.y = -crossY;
	p.z = crossZ;
	p.intensity = 0;
	return p;
}

inline double rad2deg(double radians)
{
  return radians * 180.0 / M_PI;
}

inline float rad2deg(float radians)
{
  return (float) (radians * 180.0 / M_PI);
}

inline double deg2rad(double degrees)
{
  return degrees * M_PI / 180.0;
}

inline float deg2rad(float degrees)
{
  return (float) (degrees * M_PI / 180.0);
}


inline void rotateX(Vector3& v, const Angle& angle)
{
	float y = v.y();
	v.y() = angle.cos() * y - angle.sin() * v.z();
	v.z() = angle.sin() * y + angle.cos() * v.z();
}

inline void rotateY(Vector3& v, const Angle& ang)
{
	float x = v.x();
	v.x() = ang.cos() * x + ang.sin() * v.z();
	v.z() = ang.cos() * v.z() - ang.sin() * x;
}

inline void rotateZ(Vector3& v, const Angle& ang)
{
	float x = v.x();
	v.x() = ang.cos() * x - ang.sin() * v.y();
	v.y() = ang.sin() * x + ang.cos() * v.y();
}

/** \brief Rotate the given vector by the specified angles around the z-, x- respectively y-axis.
 *
 * @param v the vector to rotate
 * @param angZ the rotation angle around the z-axis
 * @param angX the rotation angle around the x-axis
 * @param angY the rotation angle around the y-axis
 */
inline void rotateZXY(Vector3& v,
		const Angle& angleZ,
		const Angle& angleX,
		const Angle& angleY)
{
	rotateZ(v, angleZ);
	rotateX(v, angleX);
	rotateY(v, angleY);
}

/** \brief Rotate the given point by the specified angles around the z-, x- respectively y-axis.
 *
 * @param p the point to rotate
 * @param angleZ the rotation angle around the z-axis
 * @param angleX the rotation angle around the x-axis
 * @param angleY the rotation angle around the y-axis
 */
inline void rotateZXY(point_type& p,
		const Angle& angleZ,
		const Angle& angleX,
		const Angle& angleY)
{
	Vector3 v(p);
	rotateZ(v, angleZ);
	rotateX(v, angleX);
	rotateY(v, angleY);
}

inline void rotateZXY(Vector3& v,
		Rotation const& rot)
{
	rotateZXY(v, rot.z, rot.x, rot.y);
}

inline void rotateZXY(point_type& p,
		Rotation const& rot)
{
	rotateZXY(p, rot.z, rot.x, rot.y);
}



/** \brief Rotate the given vector by the specified angles around the y-, x- respectively z-axis.
 *
 * @param v the vector to rotate
 * @param angleY the rotation angle around the y-axis
 * @param angleX the rotation angle around the x-axis
 * @param angleZ the rotation angle around the z-axis
 */
inline void rotateYXZ(Vector3& v,
		const Angle& angleY,
		const Angle& angleX,
		const Angle& angleZ)
{
	rotateY(v, angleY);
	rotateX(v, angleX);
	rotateZ(v, angleZ);
}

/** \brief Rotate the given point by the specified angles around the y-, x- respectively z-axis.
 *
 * @param p the point to rotate
 * @param angleY the rotation angle around the y-axis
 * @param angleX the rotation angle around the x-axis
 * @param angleZ the rotation angle around the z-axis
 */
inline void rotateYXZ(point_type& p,
		const Angle& angleY,
		const Angle& angleX,
		const Angle& angleZ)
{
	Vector3 v(p);
	rotateY(v, angleY);
	rotateX(v, angleX);
	rotateZ(v, angleZ);
}

inline void rotateYXZ(Vector3& v,
		Rotation const& rot)
{
	rotateYXZ(v, rot.y, rot.x, rot.z);
}

inline void rotateYXZ(point_type& p,
		Rotation const& rot)
{
	rotateYXZ(p, rot.y, rot.x, rot.z);
}

} // slam

#endif // SLAM_MATH_UTILS_H
