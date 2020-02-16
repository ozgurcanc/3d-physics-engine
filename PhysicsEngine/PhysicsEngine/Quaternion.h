#pragma once

#include "headers.h"

class Quaternion
{
public:

	real r, i, j, k;

	Quaternion() : r(1), i(0), j(0), k(0) {}

	Quaternion(const real _r,const real _i,const real _j,const real _k) : r(_r), i(_i), j(_j), k(_k) {}

	void Normalise()
	{
		real l = r * r + i * i + j * j + k * k;

		if (l < real_epsilon)
		{
			r = 1;
			return;
		}

		l = real_sqrt(l);

		r /= l;
		i /= l;
		j /= l;
		k /= l;
	}

	Quaternion Unit()
	{
		Quaternion q = *this;
		q.Normalise();

		return q;
	}

	Quaternion operator+(const Quaternion & q) const
	{
		return Quaternion(r + q.r, i + q.i, j + q.j, k + q.k);
	}

	void operator+=(const Quaternion & q) 
	{
		*this = *this * q;
	}

	Quaternion operator-(const Quaternion & q) const
	{
		return Quaternion(r - q.r, i - q.i, j - q.j, k - q.k);
	}

	void operator-=(const Quaternion & q)
	{
		*this = *this - q;
	}

	Quaternion operator*(const Quaternion & mul) const
	{
		real _r = r*mul.r - i*mul.i -
			j*mul.j - k*mul.k;
		real _i = r*mul.i + i*mul.r +
			j*mul.k - k*mul.j;
		real _j = r*mul.j + j*mul.r +
			k*mul.i - i*mul.k;
		real _k = r*mul.k + k*mul.r +
			i*mul.j - j*mul.i;

		return Quaternion(_r, _i, _j, _k);
	}

	void operator*=(const Quaternion & mul)
	{
		*this = *this * mul;
	}

	void AddScaledVector(const Vector3& v, real k)
	{
		Quaternion q(0,
			v.x * k,
			v.y * k,
			v.z * k);
		q *= *this;
		r += q.r * ((real)0.5);
		i += q.i * ((real)0.5);
		j += q.j * ((real)0.5);
		k += q.k * ((real)0.5);
	}

	void RotateByVector(const Vector3& v)
	{
		Quaternion q(0, v.x, v.y, v.z);
		(*this) *= q;
	}

	friend std::ostream& operator<<(std::ostream& os, const Quaternion& q)
	{
		os << q.r << '/' << q.i << '/' << q.j << '/' << q.k;
		return os;
	}


};