#pragma once

#include "Vector3.h"
#include "Quaternion.h"
#include "Matrix4.h"
#include "Matrix3.h"

class RigidBody
{
protected:

	real inverseMass = 1;
	Matrix3 inverseInertiaTensor;
	Matrix3 inverseInertiaTensorWorld;

	real linearDamping = 1;
	real angularDamping = 1;

	Vector3 position;
	Quaternion orientation;

	Vector3 velocity;
	Vector3 rotation;

	Vector3 acceleration;
	Vector3 lastFrameAcceleration;

	Matrix4 transformMatrix;

	Vector3 forceAccum;
	Vector3 torqueAccum;


public:

	void SetMass(const real mass);
	real GetMass() const;

	void SetInverseMass(const real inverseMass);
	real GetInverseMass() const;

	bool HasFiniteMass() const;

	void SetDamping(const real linearDamping, const real angularDamping);
	void SetLinearDamping(const real linearDamping);
	void SetAngularDamping(const real angularDamping);
	real GetLinearDamping() const;
	real GetAngularDamping() const;

	void SetPosition(const Vector3 &position);
	void SetPosition(const real x,const real y,const real z);
	void GetPosition(Vector3* position);
	Vector3 GetPosition() const;

	void SetVelocity(const Vector3 &velocity);
	void SetVelocity(const real x, const real y, const real z);
	void GetVelocity(Vector3* velocity);
	Vector3 GetVelocity() const;
	void AddVelocity(const Vector3 &velocity);

	void SetRotation(const Vector3 &rotation);
	void SetRotation(const real x, const real y, const real z);
	void GetRotation(Vector3* rotation);
	Vector3 GetRotation() const;
	void AddRotation(const Vector3 &rotation);

	void SetAcceleration(const Vector3 &acceleration);
	void SetAcceleration(const real x, const real y, const real z);
	void GetAcceleration(Vector3* acceleration);
	Vector3 GetAcceleration() const;
	void AddAcceleration(const Vector3 &acceleration);

	void GetLastFrameAcceleration(Vector3 *lastFrameAcceleration) const;
	Vector3 GetLastFrameAcceleration() const;

	void AddForce(const Vector3 &force);
	void AddTorque(const Vector3 &torque);
	void AddForceAtPoint(const Vector3 &force, const Vector3 &point);
	void AddForceAtBodyPoint(const Vector3 &force, const Vector3 &point);
	void ClearAccumulators();

	void SetOrientation(const Quaternion &orientation);
	void SetOrientation(const real r, const real i, const real j, const real k);
	void GetOrientation(Quaternion *orientation) const;
	Quaternion GetOrientation() const;
	void GetOrientation(Matrix3 *matrix) const; 
	Matrix3 GetOrientation();

	void GetTransform(Matrix4 *transform);
	Matrix4 GetTransform() const;

	Vector3 GetPointInLocalSpace(const Vector3 &point) const;
	Vector3 GetPointInWorldSpace(const Vector3 &point) const;

	Vector3 GetDirectionInLocalSpace(const Vector3 &direction) const;
	Vector3 GetDirectionInWorldSpace(const Vector3 &direction) const;

	void SetInertiaTensor(const Matrix3 &inertiaTensor);
	void GetInertiaTensor(Matrix3 *inertiaTensor) const;
	Matrix3 GetInertiaTensor() const;
	void GetInertiaTensorWorld(Matrix3 *inertiaTensor) const;
	Matrix3 GetInertiaTensorWorld() const;

	void SetInverseInertiaTensor(const Matrix3 &inverseInertiaTensor);
	void GetInverseInertiaTensor(Matrix3 *inverseInertiaTensor) const;
	Matrix3 GetInverseInertiaTensor() const;
	void GetInverseInertiaTensorWorld(Matrix3 *inverseInertiaTensor) const;
	Matrix3 GetInverseInertiaTensorWorld() const;

	void CalculateDerivedData();
	void Integrate(real duration);
};

