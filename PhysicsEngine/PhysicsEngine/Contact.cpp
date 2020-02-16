#include "Contact.h"
#include <assert.h>

void Contact::CalculateInternals(real duration)
{
	if (!body[0])
	{
		body[0] = body[1];
		body[1] = NULL;
	}

	assert(body[0]);

	CalculateContactBasis();

	relativeContactPosition[0] = contactPoint - body[0]->GetPosition();
	if (body[1]) {
		relativeContactPosition[1] = contactPoint - body[1]->GetPosition();
	}

	contactVelocity = CalculateLocalVelocity(0, duration);
	if (body[1]) {
		contactVelocity -= CalculateLocalVelocity(1, duration);
	}

	CalculateDesiredDeltaVelocity(duration);
}

void Contact::CalculateDesiredDeltaVelocity(real duration)
{
	const static real velocityLimit = (real)0.25f;

	real velocityFromAcc = 0;

	velocityFromAcc += body[0]->GetLastFrameAcceleration() * duration * contactNormal;
	

	if (body[1])
	{
		velocityFromAcc -= body[1]->GetLastFrameAcceleration() * duration * contactNormal;
	}

	real thisRestitution = restitution;
	if (real_abs(contactVelocity.x) < velocityLimit)
	{
		thisRestitution = (real)0.0f;
	}

	desiredDeltaVelocity = -contactVelocity.x - thisRestitution * (contactVelocity.x - velocityFromAcc);
}

Vector3 Contact::CalculateLocalVelocity(unsigned bodyIndex, real duration)
{
	RigidBody *thisBody = body[bodyIndex];

	Vector3 velocity =
		thisBody->GetRotation() % relativeContactPosition[bodyIndex];
	velocity += thisBody->GetVelocity();

	Vector3 contactVelocity = contactToWorld.TransformTranspose(velocity);
	Vector3 accVelocity = thisBody->GetLastFrameAcceleration() * duration;

	accVelocity = contactToWorld.TransformTranspose(accVelocity);
	accVelocity.x = 0;

	contactVelocity += accVelocity;

	return contactVelocity;
}

void Contact::CalculateContactBasis()
{
	Vector3 contactTangent[2];

	if (real_abs(contactNormal.x) > real_abs(contactNormal.y))
	{
		const real s = (real)1.0f / real_sqrt(contactNormal.z*contactNormal.z +
			contactNormal.x*contactNormal.x);

		contactTangent[0].x = contactNormal.z*s;
		contactTangent[0].y = 0;
		contactTangent[0].z = -contactNormal.x*s;

		contactTangent[1].x = contactNormal.y*contactTangent[0].x;
		contactTangent[1].y = contactNormal.z*contactTangent[0].x - contactNormal.x*contactTangent[0].z;
		contactTangent[1].z = -contactNormal.y*contactTangent[0].x;
	}
	else
	{
		const real s = (real)1.0 / real_sqrt(contactNormal.z*contactNormal.z +
			contactNormal.y*contactNormal.y);

		contactTangent[0].x = 0;
		contactTangent[0].y = -contactNormal.z*s;
		contactTangent[0].z = contactNormal.y*s;

		contactTangent[1].x = contactNormal.y*contactTangent[0].z - contactNormal.z*contactTangent[0].y;
		contactTangent[1].y = -contactNormal.x*contactTangent[0].z;
		contactTangent[1].z = contactNormal.x*contactTangent[0].y;
	}

	contactToWorld = Matrix3(contactNormal.x, contactTangent[0].x, contactTangent[1].x,
		contactNormal.y, contactTangent[0].y, contactTangent[1].y,
		contactNormal.z, contactTangent[0].z, contactTangent[1].z);
}


void Contact::ApplyVelocityChange(Vector3 velocityChange[2], Vector3 rotationChange[2])
{
	Matrix3 inverseInertiaTensor[2];
	body[0]->GetInverseInertiaTensorWorld(&inverseInertiaTensor[0]);
	if (body[1])
		body[1]->GetInverseInertiaTensorWorld(&inverseInertiaTensor[1]);

	Vector3 impulseContact;

	if (friction == (real)0.0)
	{
		impulseContact = CalculateFrictionlessImpulse(inverseInertiaTensor);
	}
	else
	{
		impulseContact = CalculateFrictionImpulse(inverseInertiaTensor);
	}

	Vector3 impulse = contactToWorld.Transform(impulseContact);

	Vector3 impulsiveTorque = relativeContactPosition[0] % impulse;
	rotationChange[0] = inverseInertiaTensor[0].Transform(impulsiveTorque);
	velocityChange[0].Clear();
	velocityChange[0].AddScaledVector(impulse, body[0]->GetInverseMass());

	body[0]->AddVelocity(velocityChange[0]);
	body[0]->AddRotation(rotationChange[0]);

	if (body[1])
	{
		Vector3 impulsiveTorque = impulse % relativeContactPosition[1];
		rotationChange[1] = inverseInertiaTensor[1].Transform(impulsiveTorque);
		velocityChange[1].Clear();
		velocityChange[1].AddScaledVector(impulse, -body[1]->GetInverseMass());

		body[1]->AddVelocity(velocityChange[1]);
		body[1]->AddRotation(rotationChange[1]);
	}
}

void Contact::ApplyPositionChange(Vector3 linearChange[2], Vector3 angularChange[2], real penetration)
{
	const real angularLimit = (real)0.2f;
	real angularMove[2];
	real linearMove[2];

	real totalInertia = 0;
	real linearInertia[2];
	real angularInertia[2];

	for (unsigned i = 0; i < 2; i++) if (body[i])
	{
		Matrix3 inverseInertiaTensor;
		body[i]->GetInverseInertiaTensorWorld(&inverseInertiaTensor);

		Vector3 angularInertiaWorld =
			relativeContactPosition[i] % contactNormal;
		angularInertiaWorld =
			inverseInertiaTensor.Transform(angularInertiaWorld);
		angularInertiaWorld =
			angularInertiaWorld % relativeContactPosition[i];
		angularInertia[i] =
			angularInertiaWorld * contactNormal;

		linearInertia[i] = body[i]->GetInverseMass();

		totalInertia += linearInertia[i] + angularInertia[i];

	}

	for (unsigned i = 0; i < 2; i++) if (body[i])
	{

		real sign = (i == 0) ? 1 : -1;
		angularMove[i] =
			sign * penetration * (angularInertia[i] / totalInertia);
		linearMove[i] =
			sign * penetration * (linearInertia[i] / totalInertia);

		
		Vector3 projection = relativeContactPosition[i];
		projection.AddScaledVector(
			contactNormal,
			-relativeContactPosition[i].DotProduct(contactNormal)
		);

		
		real maxMagnitude = angularLimit * projection.Magnitude();

		if (angularMove[i] < -maxMagnitude)
		{
			real totalMove = angularMove[i] + linearMove[i];
			angularMove[i] = -maxMagnitude;
			linearMove[i] = totalMove - angularMove[i];
		}
		else if (angularMove[i] > maxMagnitude)
		{
			real totalMove = angularMove[i] + linearMove[i];
			angularMove[i] = maxMagnitude;
			linearMove[i] = totalMove - angularMove[i];
		}

		
		if (angularMove[i] == 0)
		{
			angularChange[i].Clear();
		}
		else
		{
			Vector3 targetAngularDirection =
				relativeContactPosition[i].CrossProduct(contactNormal);

			Matrix3 inverseInertiaTensor;
			body[i]->GetInverseInertiaTensorWorld(&inverseInertiaTensor);

			angularChange[i] =
				inverseInertiaTensor.Transform(targetAngularDirection) *
				(angularMove[i] / angularInertia[i]);
		}

		linearChange[i] = contactNormal * linearMove[i];

		Vector3 pos;
		body[i]->GetPosition(&pos);
		pos.AddScaledVector(contactNormal, linearMove[i]);
		body[i]->SetPosition(pos);

		Quaternion q;
		body[i]->GetOrientation(&q);
		q.AddScaledVector(angularChange[i], ((real)1.0));
		body[i]->SetOrientation(q);

		body[i]->CalculateDerivedData();
	}
}

Vector3 Contact::CalculateFrictionlessImpulse(Matrix3 *inverseInertiaTensor)
{
	Vector3 impulseContact;

	Vector3 deltaVelWorld = relativeContactPosition[0] % contactNormal;
	deltaVelWorld = inverseInertiaTensor[0].Transform(deltaVelWorld);
	deltaVelWorld = deltaVelWorld % relativeContactPosition[0];

	real deltaVelocity = deltaVelWorld * contactNormal;

	deltaVelocity += body[0]->GetInverseMass();

	if (body[1])
	{	
		Vector3 deltaVelWorld = relativeContactPosition[1] % contactNormal;
		deltaVelWorld = inverseInertiaTensor[1].Transform(deltaVelWorld);
		deltaVelWorld = deltaVelWorld % relativeContactPosition[1];
		
		deltaVelocity += deltaVelWorld * contactNormal;

		deltaVelocity += body[1]->GetInverseMass();
	}

	impulseContact.x = desiredDeltaVelocity / deltaVelocity;
	impulseContact.y = 0;
	impulseContact.z = 0;
	return impulseContact;
}

Vector3 Contact::CalculateFrictionImpulse(Matrix3 *inverseInertiaTensor)
{
	Vector3 impulseContact;
	real inverseMass = body[0]->GetInverseMass();

	Matrix3 impulseToTorque;
	impulseToTorque.SetSkewSymmetric(relativeContactPosition[0]);

	Matrix3 deltaVelWorld = impulseToTorque;
	deltaVelWorld *= inverseInertiaTensor[0];
	deltaVelWorld *= impulseToTorque;
	deltaVelWorld *= -1;

	if (body[1])
	{
		impulseToTorque.SetSkewSymmetric(relativeContactPosition[1]);

		Matrix3 deltaVelWorld2 = impulseToTorque;
		deltaVelWorld2 *= inverseInertiaTensor[1];
		deltaVelWorld2 *= impulseToTorque;
		deltaVelWorld2 *= -1;

		deltaVelWorld += deltaVelWorld2;

		inverseMass += body[1]->GetInverseMass();
	}

	Matrix3 deltaVelocity = contactToWorld.Transpose();
	deltaVelocity *= deltaVelWorld;
	deltaVelocity *= contactToWorld;

	deltaVelocity.data[0] += inverseMass;
	deltaVelocity.data[4] += inverseMass;
	deltaVelocity.data[8] += inverseMass;

	Matrix3 impulseMatrix = deltaVelocity.Inverse();

	Vector3 velKill(desiredDeltaVelocity,
		-contactVelocity.y,
		-contactVelocity.z);

	impulseContact = impulseMatrix.Transform(velKill);

	real planarImpulse = real_sqrt(
		impulseContact.y*impulseContact.y +
		impulseContact.z*impulseContact.z
	);
	if (planarImpulse > impulseContact.x * friction)
	{
		impulseContact.y /= planarImpulse;
		impulseContact.z /= planarImpulse;

		impulseContact.x = deltaVelocity.data[0] +
			deltaVelocity.data[1] * friction*impulseContact.y +
			deltaVelocity.data[2] * friction*impulseContact.z;
		impulseContact.x = desiredDeltaVelocity / impulseContact.x;
		impulseContact.y *= friction * impulseContact.x;
		impulseContact.z *= friction * impulseContact.x;
	}
	return impulseContact;
}

void Contact::ResolveCollision(real duration)
{
	Vector3 rb1[2],rb2[2];

	CalculateInternals(duration);
	ApplyPositionChange(rb1, rb2, penetration);
	ApplyVelocityChange(rb1, rb2);
}