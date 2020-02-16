#pragma once
#include "headers.h"

class Matrix4
{
public:

	real data[12];

	Matrix4()
	{
		data[1] = data[2] = data[3] = data[4] = data[6] =
			data[7] = data[8] = data[9] = data[11] = 0;
		data[0] = data[5] = data[10] = 1;
	}

	Matrix4(real d0, real d1, real d2, real d3, real d4, real d5, real d6, real d7, real d8, real d9, real d10, real d11)
	{
		data[0] = d0; data[1] = d1; data[2] = d2; data[3] = d3;
		data[4] = d4; data[5] = d5; data[6] = d6; data[7] = d7;
		data[8] = d8; data[9] = d9; data[10] = d10; data[11] = d11;
	}
	
	void SetDiagonal(const real a, const real b, const real c)
	{
		data[0] = a;
		data[5] = b;
		data[10] = c;
	}

	Matrix4 operator*(const Matrix4 &o) const
	{
		Matrix4 result;
		result.data[0] = (o.data[0] * data[0]) + (o.data[4] * data[1]) + (o.data[8] * data[2]);
		result.data[4] = (o.data[0] * data[4]) + (o.data[4] * data[5]) + (o.data[8] * data[6]);
		result.data[8] = (o.data[0] * data[8]) + (o.data[4] * data[9]) + (o.data[8] * data[10]);

		result.data[1] = (o.data[1] * data[0]) + (o.data[5] * data[1]) + (o.data[9] * data[2]);
		result.data[5] = (o.data[1] * data[4]) + (o.data[5] * data[5]) + (o.data[9] * data[6]);
		result.data[9] = (o.data[1] * data[8]) + (o.data[5] * data[9]) + (o.data[9] * data[10]);

		result.data[2] = (o.data[2] * data[0]) + (o.data[6] * data[1]) + (o.data[10] * data[2]);
		result.data[6] = (o.data[2] * data[4]) + (o.data[6] * data[5]) + (o.data[10] * data[6]);
		result.data[10] = (o.data[2] * data[8]) + (o.data[6] * data[9]) + (o.data[10] * data[10]);

		result.data[3] = (o.data[3] * data[0]) + (o.data[7] * data[1]) + (o.data[11] * data[2]) + data[3];
		result.data[7] = (o.data[3] * data[4]) + (o.data[7] * data[5]) + (o.data[11] * data[6]) + data[7];
		result.data[11] = (o.data[3] * data[8]) + (o.data[7] * data[9]) + (o.data[11] * data[10]) + data[11];

		return result;
	}

	Vector3 operator*(const Vector3 &v) const
	{
		return Vector3(
			v.x * data[0] +
			v.y * data[1] +
			v.z * data[2] + data[3],

			v.x * data[4] +
			v.y * data[5] +
			v.z * data[6] + data[7],

			v.x * data[8] +
			v.y * data[9] +
			v.z * data[10] + data[11]
		);
	}

	void operator*=(const Matrix4 &o)
	{
		*this = *this * o;
	}

	Matrix4 operator+(const Matrix4 &o) const
	{
		Matrix4 m;
		for (int i = 0; i < 12; i++) m.data[i] = data[i] + o.data[i];

		return m;
	}

	void operator+=(const Matrix4 &o)
	{
		*this = *this + o;
	}

	Matrix4 operator-(const Matrix4 &o) const
	{
		Matrix4 m;
		for (int i = 0; i < 12; i++) m.data[i] = data[i] - o.data[i];

		return m;
	}

	void operator-=(const Matrix4 &o)
	{
		*this = *this - o;
	}


	real GetDeterminant() const
	{
		return -data[8] * data[5] * data[2] +
			data[4] * data[9] * data[2] +
			data[8] * data[1] * data[6] -
			data[0] * data[9] * data[6] -
			data[4] * data[1] * data[10] +
			data[0] * data[5] * data[10];
	}
		
	void SetInverse(const Matrix4 &m)
	{
		real det = this->GetDeterminant();
		if (det == 0) return;
		det = ((real)1.0) / det;

		data[0] = (-m.data[9] * m.data[6] + m.data[5] * m.data[10])*det;
		data[4] = (m.data[8] * m.data[6] - m.data[4] * m.data[10])*det;
		data[8] = (-m.data[8] * m.data[5] + m.data[4] * m.data[9])*det;

		data[1] = (m.data[9] * m.data[2] - m.data[1] * m.data[10])*det;
		data[5] = (-m.data[8] * m.data[2] + m.data[0] * m.data[10])*det;
		data[9] = (m.data[8] * m.data[1] - m.data[0] * m.data[9])*det;

		data[2] = (-m.data[5] * m.data[2] + m.data[1] * m.data[6])*det;
		data[6] = (+m.data[4] * m.data[2] - m.data[0] * m.data[6])*det;
		data[10] = (-m.data[4] * m.data[1] + m.data[0] * m.data[5])*det;

		data[3] = (m.data[9] * m.data[6] * m.data[3]
			- m.data[5] * m.data[10] * m.data[3]
			- m.data[9] * m.data[2] * m.data[7]
			+ m.data[1] * m.data[10] * m.data[7]
			+ m.data[5] * m.data[2] * m.data[11]
			- m.data[1] * m.data[6] * m.data[11])*det;
		data[7] = (-m.data[8] * m.data[6] * m.data[3]
			+ m.data[4] * m.data[10] * m.data[3]
			+ m.data[8] * m.data[2] * m.data[7]
			- m.data[0] * m.data[10] * m.data[7]
			- m.data[4] * m.data[2] * m.data[11]
			+ m.data[0] * m.data[6] * m.data[11])*det;
		data[11] = (m.data[8] * m.data[5] * m.data[3]
			- m.data[4] * m.data[9] * m.data[3]
			- m.data[8] * m.data[1] * m.data[7]
			+ m.data[0] * m.data[9] * m.data[7]
			+ m.data[4] * m.data[1] * m.data[11]
			- m.data[0] * m.data[5] * m.data[11])*det;
	}

	Matrix4 Inverse() const
	{
		Matrix4 m;
		m.SetInverse(*this);
		
		return m;
	}

	void Invert() 
	{
		this->SetInverse(*this);
	}

	Vector3 GetAxisVector(int i) const
	{
		if (i > 3) throw "Invalid column";

		else return Vector3(data[i], data[i + 4], data[i + 8]);
	}

	Vector3 TransformPoint(const Vector3 &v) const
	{
		return (*this) * v;
	}

	Vector3 TransformDirection(const Vector3 &v) const
	{
		return Vector3(
			v.x * data[0] +
			v.y * data[1] +
			v.z * data[2],

			v.x * data[4] +
			v.y * data[5] +
			v.z * data[6],

			v.x * data[8] +
			v.y * data[9] +
			v.z * data[10]
		);
	}

	Vector3 TransformInversePoint(const Vector3 &v) const
	{
		Vector3 tmp = v;
		tmp.x -= data[3];
		tmp.y -= data[7];
		tmp.z -= data[11];
		return Vector3(
			tmp.x * data[0] +
			tmp.y * data[4] +
			tmp.z * data[8],

			tmp.x * data[1] +
			tmp.y * data[5] +
			tmp.z * data[9],

			tmp.x * data[2] +
			tmp.y * data[6] +
			tmp.z * data[10]
		);
	}

	Vector3 TransformInverseDirection(const Vector3 &v) const
	{
		return Vector3(
			v.x * data[0] +
			v.y * data[4] +
			v.z * data[8],

			v.x * data[1] +
			v.y * data[5] +
			v.z * data[9],

			v.x * data[2] +
			v.y * data[6] +
			v.z * data[10]
		);
	}

	void SetOrientationAndPos(const Quaternion &q, const Vector3 &pos) 
	{
		data[0] = 1 - (2 * q.j*q.j + 2 * q.k*q.k);
		data[1] = 2 * q.i*q.j + 2 * q.k*q.r;
		data[2] = 2 * q.i*q.k - 2 * q.j*q.r;
		data[3] = pos.x;

		data[4] = 2 * q.i*q.j - 2 * q.k*q.r;
		data[5] = 1 - (2 * q.i*q.i + 2 * q.k*q.k);
		data[6] = 2 * q.j*q.k + 2 * q.i*q.r;
		data[7] = pos.y;

		data[8] = 2 * q.i*q.k + 2 * q.j*q.r;
		data[9] = 2 * q.j*q.k - 2 * q.i*q.r;
		data[10] = 1 - (2 * q.i*q.i + 2 * q.j*q.j);
		data[11] = pos.z;
	}


	friend std::ostream& operator<<(std::ostream& os, const Matrix4& m)
	{
		os << m.data[0] << ' ' << m.data[1] << ' ' << m.data[2] << ' ' << m.data[3] << std::endl;
		os << m.data[4] << ' ' << m.data[5] << ' ' << m.data[6] << ' ' << m.data[7] << std::endl;
		os << m.data[8] << ' ' << m.data[9] << ' ' << m.data[10] << ' ' << m.data[11] << std::endl;
		return os;
	}
};