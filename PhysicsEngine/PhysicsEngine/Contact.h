#pragma once

#include "RigidBody.h"

class Contact
{
public:
	
	RigidBody * body[2];

	real friction;
	real restitution;
	real penetration;

	Vector3 contactPoint;
	Vector3 contactNormal;

protected:

	Matrix3 contactToWorld;
	Vector3 contactVelocity;
	real desiredDeltaVelocity;
	Vector3 relativeContactPosition[2];

private:

	void CalculateInternals(real duration);
	void CalculateDesiredDeltaVelocity(real duration);
	Vector3 CalculateLocalVelocity(unsigned bodyIndex, real duration);
	void CalculateContactBasis();
	void ApplyVelocityChange(Vector3 velocityChange[2], Vector3 rotationChange[2]);
	void ApplyPositionChange(Vector3 linearChange[2], Vector3 angularChange[2], real penetration);
	Vector3 CalculateFrictionlessImpulse(Matrix3 *inverseInertiaTensor);
	Vector3 CalculateFrictionImpulse(Matrix3 *inverseInertiaTensor);

public:

	void ResolveCollision(real duration);
};

