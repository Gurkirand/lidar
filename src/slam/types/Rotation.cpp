#include "../../../include/slam/types/Rotation.h"

namespace slam {

Rotation::Rotation(Angle x, Angle y, Angle z)
	: x(x), y(y), z(z) {}

Rotation::Rotation() { Rotation(0, 0, 0); }

Rotation& Rotation::operator=(Rotation const& r)
{
	x = r.x;
	y = r.y;
	z = r.z;
	return *this;
}

Rotation operator*(Rotation const& lhs, float rhs)
{
	return Rotation(lhs.x * rhs, lhs.y * rhs, lhs.z * rhs);
}

}
