#ifndef SLAM_ROTATION_H
#define SLAM_ROTATION_H

#include <pcl/point_types.h>
#include "Angle.h"
#include <tuple>


namespace slam {

/** \brief Vector4f decorator for convenient handling.
*
*/
class Rotation
{

public:

	Angle x;
	Angle y;
	Angle z;

	Rotation(Angle x, Angle y, Angle z);

	Rotation();

	Rotation& operator=(Rotation const& r);

};

Rotation operator*(Rotation const& lhs, float rhs);

} // end namespace slam

#endif //SLAM_ROTATION_H

