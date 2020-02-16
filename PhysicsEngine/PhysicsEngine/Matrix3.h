#pragma once

#include "headers.h"


class Matrix3
{
public:

	real data[9];

	Matrix3()
	{
		data[1] = data[2] = data[3] = data[5] =
			data[6] = data[7] = 0;
		
		data[0] = data[4] = data[8] = 1;
	}

	Matrix3(real d0, real d1, real d2, real d3, real d4, real d5, real d6, real d7, real d8)
	{
		data[0] = d0; data[1] = d1; data[2] = d2; 
		data[3] = d3; data[4] = d4; data[5] = d5; 
		data[6] = d6; data[7] = d7; data[8] = d8; 
	}

	void SetDiagonal(const real a, const real b, const real c)
	{
		data[0] = a;
		data[4] = b;
		data[8] = c;
	}

	void SetInertiaTensorCoeffs(real ix, real iy, real iz,
		real ixy = 0, real ixz = 0, real iyz = 0)
	{
		data[0] = ix;
		data[1] = data[3] = -ixy;
		data[2] = data[6] = -ixz;
		data[4] = iy;
		data[5] = data[7] = -iyz;
		data[8] = iz;
	}

	void SetBlockInertiaTensor(const Vector3 &halfSizes, real mass)
	{
		Vector3 v = halfSizes;
		Vector3 squares = v.ComponentProduct(v);
		this->SetInertiaTensorCoeffs(0.3f*mass*(squares.y + squares.z),
			0.3f*mass*(squares.x + squares.z),
			0.3f*mass*(squares.x + squares.y));
	}

	void SetSkewSymmetric(const Vector3 v)
	{
		data[0] = data[4] = data[8] = 0;
		data[1] = -v.z;
		data[2] = v.y;
		data[3] = v.z;
		data[5] = -v.x;
		data[6] = -v.y;
		data[7] = v.x;
	}

	Vector3 operator*(const Vector3 &v) const
	{
		return Vector3(
			v.x * data[0] + v.y * data[1] + v.z * data[2],
			v.x * data[3] + v.y * data[4] + v.z * data[5],
			v.x * data[6] + v.y * data[7] + v.z * data[8]
		);
	}

	Matrix3 operator*(const Matrix3 &o) const
	{
		return Matrix3(
			data[0] * o.data[0] + data[1] * o.data[3] + data[2] * o.data[6],
			data[0] * o.data[1] + data[1] * o.data[4] + data[2] * o.data[7],
			data[0] * o.data[2] + data[1] * o.data[5] + data[2] * o.data[8],

			data[3] * o.data[0] + data[4] * o.data[3] + data[5] * o.data[6],
			data[3] * o.data[1] + data[4] * o.data[4] + data[5] * o.data[7],
			data[3] * o.data[2] + data[4] * o.data[5] + data[5] * o.data[8],

			data[6] * o.data[0] + data[7] * o.data[3] + data[8] * o.data[6],
			data[6] * o.data[1] + data[7] * o.data[4] + data[8] * o.data[7],
			data[6] * o.data[2] + data[7] * o.data[5] + data[8] * o.data[8]
		);
	}

	void operator*=(const Matrix3 &o)
	{
		*this = *this * o;
	}

	Matrix3 operator+(const Matrix3 &o) const
	{
		Matrix3 m;
		for (int i = 0; i < 9; i++) m.data[i] = data[i] + o.data[i];

		return m;
	}

	void operator+=(const Matrix3 &o)
	{
		*this = *this + o;
	}

	Matrix3 operator-(const Matrix3 &o) const
	{
		Matrix3 m;
		for (int i = 0; i < 9; i++) m.data[i] = data[i] - o.data[i];

		return m;
	}

	void operator-=(const Matrix3 &o)
	{
		*this = *this - o;
	}

	Matrix3 operator*(const real k) const
	{
		Matrix3 m;
		for (int i = 0; i < 9; i++) m.data[i] = data[i] * k;

		return m;
	}

	void operator*=(const real k)
	{
		*this = *this * k;
	}

	void SetTranspose(const Matrix3 &m)
	{
		data[0] = m.data[0];
		data[1] = m.data[3];
		data[2] = m.data[6];
		data[3] = m.data[1];
		data[4] = m.data[4];
		data[5] = m.data[7];
		data[6] = m.data[2];
		data[7] = m.data[5];
		data[8] = m.data[8];
	}

	Matrix3 Transpose() const
	{
		Matrix3 m;
		m.SetTranspose(*this);
		return m;
	}

	real GetDeterminant() const
	{
		real t4 = data[0] * data[4];
		real t6 = data[0] * data[5];
		real t8 = data[1] * data[3];
		real t10 = data[2] * data[3];
		real t12 = data[1] * data[6];
		real t14 = data[2] * data[6];

		return (t4*data[8] - t6 * data[7] - t8 * data[8] +
			t10 * data[7] + t12 * data[5] - t14 * data[4]);
	}

	void SetInverse(const Matrix3 &m)
	{
		real t4 = m.data[0] * m.data[4];
		real t6 = m.data[0] * m.data[5];
		real t8 = m.data[1] * m.data[3];
		real t10 = m.data[2] * m.data[3];
		real t12 = m.data[1] * m.data[6];
		real t14 = m.data[2] * m.data[6];

		real t16 = (t4*m.data[8] - t6 * m.data[7] - t8 * m.data[8] +
			t10 * m.data[7] + t12 * m.data[5] - t14 * m.data[4]);

		if (t16 == (real)0.0f) return;
		real t17 = 1 / t16;

		data[0] = (m.data[4] * m.data[8] - m.data[5] * m.data[7])*t17;
		data[1] = -(m.data[1] * m.data[8] - m.data[2] * m.data[7])*t17;
		data[2] = (m.data[1] * m.data[5] - m.data[2] * m.data[4])*t17;
		data[3] = -(m.data[3] * m.data[8] - m.data[5] * m.data[6])*t17;
		data[4] = (m.data[0] * m.data[8] - t14)*t17;
		data[5] = -(t6 - t10)*t17;
		data[6] = (m.data[3] * m.data[7] - m.data[4] * m.data[6])*t17;
		data[7] = -(m.data[0] * m.data[7] - t12)*t17;
		data[8] = (t4 - t8)*t17;
	}

	Matrix3 Inverse() const
	{
		Matrix3 result;
		result.SetInverse(*this);
		return result;
	}

	void Invert()
	{
		SetInverse(*this);
	}

	Vector3 GetRowVector(int i) const
	{
		if (i > 2) throw "Invalid index";
		
		else return Vector3(data[i * 3], data[i * 3 + 1], data[i * 3 + 2]);
	}

	Vector3 GetAxisVector(int i) const
	{
		if (i > 2) throw "Invalid index";

		else return Vector3(data[i], data[i + 3], data[i + 6]);
	}

	Vector3 Transform(const Vector3 &v) const
	{
		return (*this) * v;
	}

	Vector3 TransformTranspose(const Vector3 &v) const
	{
		return Vector3(
			v.x * data[0] + v.y * data[3] + v.z * data[6],
			v.x * data[1] + v.y * data[4] + v.z * data[7],
			v.x * data[2] + v.y * data[5] + v.z * data[8]
		);
	}


	void SetOrientation(const Quaternion &q)
	{
		data[0] = 1 - (2 * q.j*q.j + 2 * q.k*q.k);
		data[1] = 2 * q.i*q.j + 2 * q.k*q.r;
		data[2] = 2 * q.i*q.k - 2 * q.j*q.r;
		data[3] = 2 * q.i*q.j - 2 * q.k*q.r;
		data[4] = 1 - (2 * q.i*q.i + 2 * q.k*q.k);
		data[5] = 2 * q.j*q.k + 2 * q.i*q.r;
		data[6] = 2 * q.i*q.k + 2 * q.j*q.r;
		data[7] = 2 * q.j*q.k - 2 * q.i*q.r;
		data[8] = 1 - (2 * q.i*q.i + 2 * q.j*q.j);
	}




	friend std::ostream& operator<<(std::ostream& os, const Matrix3& m)
	{
		os << m.data[0] << ' ' << m.data[1] << ' ' << m.data[2]  << std::endl;
		os << m.data[3] << ' ' << m.data[4] << ' ' << m.data[5]  << std::endl;
		os << m.data[6] << ' ' << m.data[7] << ' ' << m.data[8] << std::endl;
		return os;
	}

};