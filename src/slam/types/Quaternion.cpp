#include "../../../include/slam/types/Quaternion.h"

#include <cmath>

namespace slam
{

Quaternion::Quaternion() {}

Quaternion::Quaternion(float x, float y, float z, float w)
	: x(x), y(y), z(z), w(w) {}

Quaternion make_quaternion_ypr(float yaw, float pitch, float roll)
{
	float sinYaw = sin(yaw / 2);
	float cosYaw = cos(yaw / 2);
	float sinPitch = sin(pitch / 2);
	float cosPitch = cos(pitch / 2);
	float sinRoll = sin(roll / 2);
	float cosRoll = cos(roll / 2);

	return Quaternion(cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw,
			cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw,
			sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw,
			cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw);
}

Quaternion make_quaternion_rpy(float roll, float pitch, float yaw)
{
	float sinRoll = sin(roll / 2);
	float cosRoll = cos(roll / 2);
	float sinPitch = sin(pitch / 2);
	float cosPitch = cos(pitch / 2);
	float sinYaw = sin(yaw / 2);
	float cosYaw = cos(yaw / 2);

	return Quaternion(sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw,
			cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw,
			cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw,
			cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw);
}

}
