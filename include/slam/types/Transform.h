#ifndef SLAM_Transform_H
#define SLAM_Transform_H


#include "Rotation.h"
#include "Vector3.h"


namespace slam {


/** \brief Transform composed by three angles and a three-dimensional position.
*
*/
class Transform {
public:
	Transform();

	Rotation rotation;
	Vector3 translation;
};

} // end namespace slam

#endif //SLAM_Transform_H

