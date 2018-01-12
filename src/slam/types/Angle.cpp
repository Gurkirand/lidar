#include "../../../include/slam/types/Angle.h"

#include <cmath>

namespace slam{

Angle::Angle()
	: _radian(0.0),
	_cos(1.0),
	_sin(0.0) {}

Angle::Angle(float radValue)
	: _radian(radValue),
	_cos(std::cos(radValue)),
	_sin(std::sin(radValue)) {}

Angle::Angle(const Angle &other)
	: _radian(other._radian),
	_cos(other._cos),
	_sin(other._sin) {}

void Angle::operator=(const Angle &rhs) {
	_radian = (rhs._radian);
	_cos = (rhs._cos);
	_sin = (rhs._sin);
}

void Angle::operator+=(const float &radValue) { *this = (_radian + radValue); }

void Angle::operator+=(const Angle &other) { *this = (_radian + other._radian); }

void Angle::operator-=(const float &radValue) { *this = (_radian - radValue); }

void Angle::operator-=(const Angle &other) { *this = (_radian - other._radian); }

Angle Angle::operator-() const {
	Angle out;
	out._radian = -_radian;
	out._cos = _cos;
	out._sin = -(_sin);
	return out;
}

float Angle::rad() const { return _radian; }

float Angle::deg() const { return _radian * 180 / M_PI; }

float Angle::cos() const { return _cos; }

float Angle::sin() const { return _sin; }

Angle operator*(Angle const& lhs, float rhs)
{
	return Angle(lhs.rad() * rhs);
}

}
