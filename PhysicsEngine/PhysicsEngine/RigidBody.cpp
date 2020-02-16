#include "RigidBody.h"
#include <assert.h>
#include <memory.h>


static inline void _transformInertiaTensor(Matrix3 &iitWorld,
	const Quaternion &q,
	const Matrix3 &iitBody,
	const Matrix4 &rotmat)
{
	real t4 = rotmat.data[0] * iitBody.data[0] +
		rotmat.data[1] * iitBody.data[3] +
		rotmat.data[2] * iitBody.data[6];
	real t9 = rotmat.data[0] * iitBody.data[1] +
		rotmat.data[1] * iitBody.data[4] +
		rotmat.data[2] * iitBody.data[7];
	real t14 = rotmat.data[0] * iitBody.data[2] +
		rotmat.data[1] * iitBody.data[5] +
		rotmat.data[2] * iitBody.data[8];
	real t28 = rotmat.data[4] * iitBody.data[0] +
		rotmat.data[5] * iitBody.data[3] +
		rotmat.data[6] * iitBody.data[6];
	real t33 = rotmat.data[4] * iitBody.data[1] +
		rotmat.data[5] * iitBody.data[4] +
		rotmat.data[6] * iitBody.data[7];
	real t38 = rotmat.data[4] * iitBody.data[2] +
		rotmat.data[5] * iitBody.data[5] +
		rotmat.data[6] * iitBody.data[8];
	real t52 = rotmat.data[8] * iitBody.data[0] +
		rotmat.data[9] * iitBody.data[3] +
		rotmat.data[10] * iitBody.data[6];
	real t57 = rotmat.data[8] * iitBody.data[1] +
		rotmat.data[9] * iitBody.data[4] +
		rotmat.data[10] * iitBody.data[7];
	real t62 = rotmat.data[8] * iitBody.data[2] +
		rotmat.data[9] * iitBody.data[5] +
		rotmat.data[10] * iitBody.data[8];

	iitWorld.data[0] = t4 * rotmat.data[0] +
		t9 * rotmat.data[1] +
		t14 * rotmat.data[2];
	iitWorld.data[1] = t4 * rotmat.data[4] +
		t9 * rotmat.data[5] +
		t14 * rotmat.data[6];
	iitWorld.data[2] = t4 * rotmat.data[8] +
		t9 * rotmat.data[9] +
		t14 * rotmat.data[10];
	iitWorld.data[3] = t28 * rotmat.data[0] +
		t33 * rotmat.data[1] +
		t38 * rotmat.data[2];
	iitWorld.data[4] = t28 * rotmat.data[4] +
		t33 * rotmat.data[5] +
		t38 * rotmat.data[6];
	iitWorld.data[5] = t28 * rotmat.data[8] +
		t33 * rotmat.data[9] +
		t38 * rotmat.data[10];
	iitWorld.data[6] = t52 * rotmat.data[0] +
		t57 * rotmat.data[1] +
		t62 * rotmat.data[2];
	iitWorld.data[7] = t52 * rotmat.data[4] +
		t57 * rotmat.data[5] +
		t62 * rotmat.data[6];
	iitWorld.data[8] = t52 * rotmat.data[8] +
		t57 * rotmat.data[9] +
		t62 * rotmat.data[10];
}

static inline void _calculateTransformMatrix(Matrix4 &transformMatrix,
	const Vector3 &position,
	const Quaternion &orientation)
{
	transformMatrix.data[0] = 1 - 2 * orientation.j*orientation.j -
		2 * orientation.k*orientation.k;
	transformMatrix.data[1] = 2 * orientation.i*orientation.j -
		2 * orientation.r*orientation.k;
	transformMatrix.data[2] = 2 * orientation.i*orientation.k +
		2 * orientation.r*orientation.j;
	transformMatrix.data[3] = position.x;

	transformMatrix.data[4] = 2 * orientation.i*orientation.j +
		2 * orientation.r*orientation.k;
	transformMatrix.data[5] = 1 - 2 * orientation.i*orientation.i -
		2 * orientation.k*orientation.k;
	transformMatrix.data[6] = 2 * orientation.j*orientation.k -
		2 * orientation.r*orientation.i;
	transformMatrix.data[7] = position.y;

	transformMatrix.data[8] = 2 * orientation.i*orientation.k -
		2 * orientation.r*orientation.j;
	transformMatrix.data[9] = 2 * orientation.j*orientation.k +
		2 * orientation.r*orientation.i;
	transformMatrix.data[10] = 1 - 2 * orientation.i*orientation.i -
		2 * orientation.j*orientation.j;
	transformMatrix.data[11] = position.z;
}

void RigidBody::SetMass(const real mass)
{
	assert( mass != 0);
	this->inverseMass = (real)1 / mass;
}

real RigidBody::GetMass() const
{
	if (inverseMass == 0) return REAL_MAX;
	else return (real)1 / inverseMass;
}

void RigidBody::SetInverseMass(const real inverseMass)
{
	this->inverseMass = inverseMass;
}

real RigidBody::GetInverseMass() const
{
	return inverseMass;
}

bool RigidBody::HasFiniteMass() const
{
	return inverseMass >= (real)0;
}

void RigidBody::SetDamping(const real linearDamping, const real angularDamping)
{
	this->linearDamping = linearDamping;
	this->angularDamping = angularDamping;
}

void RigidBody::SetLinearDamping(const real linearDamping)
{
	this->linearDamping = linearDamping;
}

void RigidBody::SetAngularDamping(const real angularDamping)
{
	this->angularDamping = angularDamping;
}

real RigidBody::GetLinearDamping() const
{
	return linearDamping;
}

real RigidBody::GetAngularDamping() const
{
	return angularDamping;
}


void RigidBody::SetPosition(const Vector3 &position)
{
	this->position = position;
}

void RigidBody::SetPosition(const real x, const real y, const real z)
{
	this->position = Vector3(x, y, z);
}

void RigidBody::GetPosition(Vector3* position)
{
	*position = this->position;
}

Vector3 RigidBody::GetPosition() const
{
	return position;
}


void RigidBody::SetVelocity(const Vector3 &velocity)
{
	this->velocity = velocity;
}

void RigidBody::SetVelocity(const real x, const real y, const real z)
{
	this->velocity = Vector3(x, y, z);
}

void RigidBody::GetVelocity(Vector3* velocity)
{
	*velocity = this->velocity;
}

Vector3 RigidBody::GetVelocity() const
{
	return velocity;
}

void RigidBody::AddVelocity(const Vector3 &velocity)
{
	this->velocity += velocity;
}

void RigidBody::SetRotation(const Vector3 &rotation)
{
	this->rotation = rotation;
}

void RigidBody::SetRotation(const real x, const real y, const real z)
{
	this->rotation = Vector3(x, y, z);
}

void RigidBody::GetRotation(Vector3* rotation)
{
	*rotation = this->rotation;
}

Vector3 RigidBody::GetRotation() const
{
	return rotation;
}

void RigidBody::AddRotation(const Vector3 &rotation)
{
	this->rotation += rotation;
}


void RigidBody::SetAcceleration(const Vector3 &acceleration)
{
	this->acceleration = acceleration;
}

void RigidBody::SetAcceleration(const real x, const real y, const real z)
{
	this->acceleration = Vector3(x, y, z);
}

void RigidBody::GetAcceleration(Vector3* acceleration)
{
	*acceleration = this->acceleration;
}

Vector3 RigidBody::GetAcceleration() const
{
	return acceleration;
}

void RigidBody::AddAcceleration(const Vector3 &acceleration)
{
	this->acceleration += acceleration;
}


void RigidBody::GetLastFrameAcceleration(Vector3 *lastFrameAcceleration) const
{
	*lastFrameAcceleration = this->lastFrameAcceleration;
}

Vector3 RigidBody::GetLastFrameAcceleration() const
{
	return lastFrameAcceleration;
}

void RigidBody::AddForce(const Vector3 &force)
{
	this->forceAccum += force;
}

void RigidBody::AddTorque(const Vector3 &torque)
{
	this->torqueAccum = torque;
}

void RigidBody::AddForceAtPoint(const Vector3 &force, const Vector3 &point)
{
	Vector3 p = point - position;

	forceAccum += force;
	torqueAccum += p % force;

}

void RigidBody::AddForceAtBodyPoint(const Vector3 &force, const Vector3 &point)
{
	Vector3 p = GetPointInWorldSpace(point);
	AddForceAtPoint(force, p);
}

void RigidBody::ClearAccumulators()
{
	forceAccum.Clear();
	torqueAccum.Clear();
}

void RigidBody::SetOrientation(const Quaternion &orientation)
{
	this->orientation = orientation;
	this->orientation.Normalise();
}

void RigidBody::SetOrientation(const real r, const real i, const real j, const real k)
{
	this->orientation = Quaternion(r, i, j, k);
	this->orientation.Normalise();
}

void RigidBody::GetOrientation(Quaternion *orientation) const
{
	*orientation = this->orientation;
}

Quaternion RigidBody::GetOrientation() const
{
	return orientation;
}

void RigidBody::GetOrientation(Matrix3 *matrix) const
{
	matrix->data[0] = transformMatrix.data[0];
	matrix->data[1] = transformMatrix.data[1];
	matrix->data[2] = transformMatrix.data[2];

	matrix->data[3] = transformMatrix.data[4];
	matrix->data[4] = transformMatrix.data[5];
	matrix->data[5] = transformMatrix.data[6];

	matrix->data[6] = transformMatrix.data[8];
	matrix->data[7] = transformMatrix.data[9];
	matrix->data[8] = transformMatrix.data[10];
}

Matrix3 RigidBody::GetOrientation()
{
	Matrix3 m;
	GetOrientation(&m);

	return m;
}

void RigidBody::GetTransform(Matrix4 *transform)
{
	*transform = transformMatrix;
}

Matrix4 RigidBody::GetTransform() const
{
	return transformMatrix;
}

Vector3 RigidBody::GetPointInLocalSpace(const Vector3 &point) const
{
	return transformMatrix.TransformInversePoint(point);
}

Vector3 RigidBody::GetPointInWorldSpace(const Vector3 &point) const
{
	return transformMatrix.TransformPoint(point);
}

Vector3 RigidBody::GetDirectionInLocalSpace(const Vector3 &direction) const
{
	return transformMatrix.TransformInverseDirection(direction);
}

Vector3 RigidBody::GetDirectionInWorldSpace(const Vector3 &direction) const
{
	return transformMatrix.TransformDirection(direction);
}

void RigidBody::SetInertiaTensor(const Matrix3 &inertiaTensor)
{
	inverseInertiaTensor.SetInverse(inertiaTensor);
}

void RigidBody::GetInertiaTensor(Matrix3 *inertiaTensor) const
{
	inertiaTensor->SetInverse(inverseInertiaTensor);
}

Matrix3 RigidBody::GetInertiaTensor() const
{
	Matrix3 m;
	GetInertiaTensor(&m);

	return m;
}

void RigidBody::GetInertiaTensorWorld(Matrix3 *inertiaTensor) const
{
	inertiaTensor->SetInverse(inverseInertiaTensorWorld);
}

Matrix3 RigidBody::GetInertiaTensorWorld() const
{
	Matrix3 m;
	GetInertiaTensorWorld(&m);

	return m;
}

void RigidBody::SetInverseInertiaTensor(const Matrix3 &inverseInertiaTensor)
{
	this->inverseInertiaTensor = inverseInertiaTensor;
}

void RigidBody::GetInverseInertiaTensor(Matrix3 *inverseInertiaTensor) const
{
	*inverseInertiaTensor = this->inverseInertiaTensor;
}

Matrix3 RigidBody::GetInverseInertiaTensor() const
{
	return inverseInertiaTensor;
}

void RigidBody::GetInverseInertiaTensorWorld(Matrix3 *inverseInertiaTensor) const
{
	*inverseInertiaTensor = this->inverseInertiaTensorWorld;
}

Matrix3 RigidBody::GetInverseInertiaTensorWorld() const
{
	return inverseInertiaTensorWorld;
}

void RigidBody::CalculateDerivedData()
{
	orientation.Normalise();

	_calculateTransformMatrix(transformMatrix, position, orientation);
	_transformInertiaTensor(inverseInertiaTensorWorld, orientation, inverseInertiaTensor, transformMatrix);
}

void RigidBody::Integrate(real duration)
{
	lastFrameAcceleration = acceleration;
	lastFrameAcceleration.AddScaledVector(forceAccum, duration);


	Vector3 angularAcceleration = inverseInertiaTensorWorld.Transform(torqueAccum);

	velocity.AddScaledVector(lastFrameAcceleration, duration);
	rotation.AddScaledVector(angularAcceleration, duration);

	velocity *= real_pow(linearDamping, duration);
	rotation *= real_pow(angularDamping, duration);

	position.AddScaledVector(velocity, duration);
	orientation.AddScaledVector(rotation, duration);

	CalculateDerivedData();
	ClearAccumulators();

}


