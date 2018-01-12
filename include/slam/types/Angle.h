#ifndef SLAM_ANGLE_H
#define SLAM_ANGLE_H


namespace slam {


/** \brief Class for holding an angle.
 *
 * This class provides buffered access to sine and cosine values to the represented angular value.
 */
class Angle {

public:
	Angle ();

	Angle (float radValue);

	Angle (const Angle& other);

	void operator=(const Angle &rhs);

	void operator+=(const float &radValue);

	void operator+=(const Angle &other);

	void operator-=(const float &radValue);

	void operator-=(const Angle &other);

	Angle operator-() const;

	float rad() const;
	float deg() const;

	float cos() const;

	float sin() const;

private:
	float _radian;    ///< angle value in radian
	float _cos;       ///< cosine of the angle
	float _sin;       ///< sine of the angle
};

Angle operator*(Angle const& lhs, float rhs);

} // end namespace slam

#endif //SLAM_ANGLE_H

