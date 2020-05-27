#pragma once
#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include ".\Eigen\Eigen"
#include ".\Eigen\Geometry"

using namespace std;

class EulerAngle
{
public:
	float heading = 0;
	float attitude = 0;
	float bank = 0;

	EulerAngle()
	{
		float heading = 0;
		float attitude = 0;
		float bank = 0;
	}

	EulerAngle(float heading, float attitude,  float bank)
	{
		
		this->heading = heading;
		this->attitude = attitude;
		this->bank = bank;
	};

	void euler_to_eigen_quaternion(Eigen::Quaternionf& qua)
	{
		// Abbreviations for the various angular functions
		float cy = cos(attitude * 0.5);
		float sy = sin(attitude * 0.5);
		float cr = cos(bank * 0.5);
		float sr = sin(bank * 0.5);
		float cp = cos(heading * 0.5);
		float sp = sin(heading * 0.5);

		qua.w() = cy * cr * cp + sy * sr * sp;
		qua.x() = cy * sr * cp - sy * cr * sp;
		qua.y() = cy * cr * sp + sy * sr * cp;
		qua.z() = sy * cr * cp - cy * sr * sp;
	};

	void quaternion_to_euler(Eigen::Quaternionf& q)
	{
		// roll (x-axis rotation)
		float sinr = +2.0 * (q.w() * q.x() + q.y() * q.z());
		float cosr = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
		attitude = atan2(sinr, cosr);

		// pitch (y-axis rotation)
		float sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
		if (fabs(sinp) >= 1)
			heading = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
		else
			heading = asin(sinp);

		// yaw (z-axis rotation)
		float siny = +2.0 * (q.w() * q.z() + q.x() * q.y());
		float cosy = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
		bank = atan2(siny, cosy);
	};

	void euler_to_eigen_matrix(Eigen::Matrix4f& max)
	{
		float sa = sin(attitude);
		float ca = cos(attitude);
		float sb = sin(bank);
		float cb = cos(bank);
		float sh = sin(heading);
		float ch = cos(heading);

		max << ch*ca, sh*sb - ch*sa*cb, ch*sa*sb + sh*cb, 0,
			sa, ca*cb, -ca*sb, 0,
			-sh*ca, sh*sa*cb + ch*sb, -sh*sa*sb + ch*cb, 0,
			0, 0, 0, 1;
	};

	void matrix_to_euler(Eigen::Matrix4f m)
	{
		// Assuming the angles are in radians.
		if (m(1,0) > 0.998) { // singularity at north pole
			heading = atan2(m(0,2), m(2,2));
			attitude = (M_PI / 2);
			bank = 0;
			return;
		}
		if (m(1,0) < -0.998) { // singularity at south pole
			heading = atan2(m(0,2), m(2,2));
			attitude = (-M_PI / 2);
			bank = 0;
			return;
		}
		heading = atan2(-m(2,0), m(0,0));
		attitude = atan2(-m(1,2), m(1,1));
		bank = asin(m(1,0));
	};

	EulerAngle& operator+(EulerAngle& other);

	EulerAngle& operator-(EulerAngle& other);

	EulerAngle& operator/(EulerAngle& other);

	EulerAngle operator*(EulerAngle& other);
};