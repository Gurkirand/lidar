#ifndef SLAM_QUATERNION_H
#define SLAM_QUATERNION_H

namespace slam {

class Quaternion {
public:

	float x;
	float y;
	float z;
	float w;

	Quaternion();
	
	Quaternion(float x, float y, float z, float w);

};

Quaternion make_quaternion_ypr(float yaw, float pitch, float roll);

Quaternion make_quaternion_rpy(float roll, float pitch, float yaw);

} // end namespace slam

#endif //SLAM_QUATERNION_H

