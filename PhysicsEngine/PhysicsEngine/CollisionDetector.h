#pragma once
#include "Contact.h"
#include "Colliders.h"
#include <assert.h>


static inline real transformToAxis(
	const BoxCollider &box,
	const Vector3 &axis
)
{
	return
		box.halfSize.x * real_abs(axis * box.GetAxis(0)) +
		box.halfSize.y * real_abs(axis * box.GetAxis(1)) +
		box.halfSize.z * real_abs(axis * box.GetAxis(2));
}

static inline real penetrationOnAxis(
	const BoxCollider &one,
	const BoxCollider &two,
	const Vector3 &axis,
	const Vector3 &toCentre
)
{
	real oneProject = transformToAxis(one, axis);
	real twoProject = transformToAxis(two, axis);

	real distance = real_abs(toCentre * axis);

	return oneProject + twoProject - distance;
}

static inline bool tryAxis(
	const BoxCollider &one,
	const BoxCollider &two,
	Vector3 axis,
	const Vector3& toCentre,
	unsigned index,

	real& smallestPenetration,
	unsigned &smallestCase
)
{
	if (axis.SquareMagnitude() < 0.0001) return true;
	axis.Normalise();

	real penetration = penetrationOnAxis(one, two, axis, toCentre);

	if (penetration < 0) return false;
	if (penetration < smallestPenetration) {
		smallestPenetration = penetration;
		smallestCase = index;
	}
	return true;
}

static Contact* fillPointFaceBoxBox(
	const BoxCollider &one,
	const BoxCollider &two,
	const Vector3 &toCentre,
	unsigned best,
	real pen
)
{
	Contact* contact = new Contact();

	Vector3 normal = one.GetAxis(best);
	if (one.GetAxis(best) * toCentre > 0)
	{
		normal = normal * -1.0f;
	}

	Vector3 vertex = two.halfSize;
	if (two.GetAxis(0) * normal < 0) vertex.x = -vertex.x;
	if (two.GetAxis(1) * normal < 0) vertex.y = -vertex.y;
	if (two.GetAxis(2) * normal < 0) vertex.z = -vertex.z;

	contact->contactNormal = normal;
	contact->penetration = pen;
	contact->contactPoint = two.GetTransform() * vertex;

	contact->body[0] = one.rigidBody;
	contact->body[1] = two.rigidBody;

	//Material dependent, for now just set to constants
	contact->friction = globalFriction;
	contact->restitution = globalRestitution;

	return contact;
}


static inline Vector3 contactPoint(
	const Vector3 &pOne,
	const Vector3 &dOne,
	real oneSize,
	const Vector3 &pTwo,
	const Vector3 &dTwo,
	real twoSize,
	bool useOne)
{
	Vector3 toSt, cOne, cTwo;
	real dpStaOne, dpStaTwo, dpOneTwo, smOne, smTwo;
	real denom, mua, mub;

	smOne = dOne.SquareMagnitude();
	smTwo = dTwo.SquareMagnitude();
	dpOneTwo = dTwo * dOne;

	toSt = pOne - pTwo;
	dpStaOne = dOne * toSt;
	dpStaTwo = dTwo * toSt;

	denom = smOne * smTwo - dpOneTwo * dpOneTwo;

	if (real_abs(denom) < 0.0001f) {
		return useOne ? pOne : pTwo;
	}

	mua = (dpOneTwo * dpStaTwo - smTwo * dpStaOne) / denom;
	mub = (smOne * dpStaTwo - dpOneTwo * dpStaOne) / denom;

	if (mua > oneSize ||
		mua < -oneSize ||
		mub > twoSize ||
		mub < -twoSize)
	{
		return useOne ? pOne : pTwo;
	}
	else
	{
		cOne = pOne + dOne * mua;
		cTwo = pTwo + dTwo * mub;

		return cOne * 0.5 + cTwo * 0.5;
	}
}


class CollisionDetector
{
public:
	static Contact* DetectCollision(Collider* one, Collider* two)
	{
		
		if (one->colliderType == ColliderType::Sphere && two->colliderType == ColliderType::Sphere)
		{
			return SphereAndSphere(dynamic_cast<SphereCollider*>(one), dynamic_cast<SphereCollider*>(two));
		}
		else if (one->colliderType == ColliderType::Box && two->colliderType == ColliderType::Sphere)
		{
			return BoxAndSphere(dynamic_cast<BoxCollider*>(one), dynamic_cast<SphereCollider*>(two));
		}
		else if (one->colliderType == ColliderType::Sphere && two->colliderType == ColliderType::Box)
		{
			return BoxAndSphere(dynamic_cast<BoxCollider*>(two), dynamic_cast<SphereCollider*>(one));
		}
		else if (one->colliderType == ColliderType::Box && two->colliderType == ColliderType::Box)
		{
			return BoxAndBox(dynamic_cast<BoxCollider*>(one), dynamic_cast<BoxCollider*>(two));
		}
		else
		{
			return NULL;
		}

	}

private:

	static Contact* SphereAndSphere(SphereCollider* one,SphereCollider* two)
	{
		
		
		Vector3 positionOne = one->GetAxis(3);
		Vector3 positionTwo = two->GetAxis(3);

		Vector3 midline = positionOne - positionTwo;
		real size = midline.Magnitude();

		if (size <= 0.0f || size >= (one->radius + two->radius))
			return NULL;


		Vector3 normal = midline * ((real)1/size);

		Contact * contact = new Contact();

		contact->contactNormal = normal;
		contact->contactPoint = positionOne + midline * (real)0.5;
		contact->penetration = (one->radius + two->radius - size);
		contact->body[0] = one->rigidBody;
		contact->body[1] = two->rigidBody;

		//Material dependent, for now just set to constants
		contact->friction = globalFriction;
		contact->restitution = globalRestitution;

		return contact;
	}

	static Contact* BoxAndSphere(BoxCollider* box, SphereCollider* sphere)
	{
		Vector3 center = sphere->GetAxis(3);
		Vector3 realCenter = box->GetTransform().TransformInversePoint(center);

		if (real_abs(realCenter.x) - sphere->radius > box->halfSize.x ||
			real_abs(realCenter.y) - sphere->radius > box->halfSize.y ||
			real_abs(realCenter.z) - sphere->radius > box->halfSize.z)
		{
			return NULL;
		}

		Vector3 closestPt(0, 0, 0);
		real dist;

		dist = realCenter.x;
		if (dist > box->halfSize.x) dist = box->halfSize.x;
		if (dist < -box->halfSize.x) dist = -box->halfSize.x;
		closestPt.x = dist;

		dist = realCenter.y;
		if (dist > box->halfSize.y) dist = box->halfSize.y;
		if (dist < -box->halfSize.y) dist = -box->halfSize.y;
		closestPt.y = dist;

		dist = realCenter.z;
		if (dist > box->halfSize.z) dist = box->halfSize.z;
		if (dist < -box->halfSize.z) dist = -box->halfSize.z;
		closestPt.z = dist;

		dist = (closestPt - realCenter).SquareMagnitude();
		if (dist > sphere->radius * sphere->radius) 
			return NULL;

		Vector3 closestPtWorld = box->GetTransform().TransformPoint(closestPt);

		Contact* contact = new Contact();

		contact->contactNormal = (closestPtWorld - center);
		contact->contactNormal.Normalise();
		contact->contactPoint = closestPtWorld;
		contact->penetration = sphere->radius - real_sqrt(dist);

		contact->body[0] = box->rigidBody;
		contact->body[1] = sphere->rigidBody;

		//Material dependent, for now just set to constants
		contact->friction = globalFriction;
		contact->restitution = globalRestitution;

		return contact;
	}

	static Contact* BoxAndBox(BoxCollider* one, BoxCollider* two)
	{
		Vector3 toCentre = two->GetAxis(3) - one->GetAxis(3);

		real pen = REAL_MAX;
		unsigned best = 0xffffff;

		if (!tryAxis(*one, *two, (one->GetAxis(0)), toCentre, (0), pen, best)) return NULL;
		if (!tryAxis(*one, *two, (one->GetAxis(1)), toCentre, (1), pen, best)) return NULL;
		if (!tryAxis(*one, *two, (one->GetAxis(2)), toCentre, (2), pen, best)) return NULL;

		if (!tryAxis(*one, *two, (two->GetAxis(0)), toCentre, (3), pen, best)) return NULL;
		if (!tryAxis(*one, *two, (two->GetAxis(1)), toCentre, (4), pen, best)) return NULL;
		if (!tryAxis(*one, *two, (two->GetAxis(2)), toCentre, (5), pen, best)) return NULL;

		unsigned bestSingleAxis = best;

		if (!tryAxis(*one, *two, (one->GetAxis(0) % two->GetAxis(0)), toCentre, (6), pen, best)) return NULL;
		if (!tryAxis(*one, *two, (one->GetAxis(0) % two->GetAxis(1)), toCentre, (7), pen, best)) return NULL;
		if (!tryAxis(*one, *two, (one->GetAxis(0) % two->GetAxis(2)), toCentre, (8), pen, best)) return NULL;
		if (!tryAxis(*one, *two, (one->GetAxis(1) % two->GetAxis(0)), toCentre, (9), pen, best)) return NULL;
		if (!tryAxis(*one, *two, (one->GetAxis(1) % two->GetAxis(1)), toCentre, (10), pen, best)) return NULL;
		if (!tryAxis(*one, *two, (one->GetAxis(1) % two->GetAxis(2)), toCentre, (11), pen, best)) return NULL;
		if (!tryAxis(*one, *two, (one->GetAxis(2) % two->GetAxis(0)), toCentre, (12), pen, best)) return NULL;
		if (!tryAxis(*one, *two, (one->GetAxis(2) % two->GetAxis(1)), toCentre, (13), pen, best)) return NULL;
		if (!tryAxis(*one, *two, (one->GetAxis(2) % two->GetAxis(2)), toCentre, (14), pen, best)) return NULL;

		assert(best != 0xffffff);

		if (best < 3)
		{
			return fillPointFaceBoxBox(*one, *two, toCentre, best, pen);
		}
		else if (best < 6)
		{
			return fillPointFaceBoxBox(*two, *one, toCentre*-1.0f, best - 3, pen);
		}
		else
		{
			best -= 6;
			unsigned oneAxisIndex = best / 3;
			unsigned twoAxisIndex = best % 3;
			Vector3 oneAxis = one->GetAxis(oneAxisIndex);
			Vector3 twoAxis = two->GetAxis(twoAxisIndex);
			Vector3 axis = oneAxis % twoAxis;
			axis.Normalise();

			if (axis * toCentre > 0) axis = axis * -1.0f;

			Vector3 ptOnOneEdge = one->halfSize;
			Vector3 ptOnTwoEdge = two->halfSize;
			for (unsigned i = 0; i < 3; i++)
			{
				if (i == oneAxisIndex) ptOnOneEdge[i] = 0;
				else if (one->GetAxis(i) * axis > 0) ptOnOneEdge[i] = -ptOnOneEdge[i];

				if (i == twoAxisIndex) ptOnTwoEdge[i] = 0;
				else if (two->GetAxis(i) * axis < 0) ptOnTwoEdge[i] = -ptOnTwoEdge[i];
			}

			ptOnOneEdge = one->GetTransform() * ptOnOneEdge;
			ptOnTwoEdge = two->GetTransform() * ptOnTwoEdge;

			Vector3 vertex = contactPoint(
				ptOnOneEdge, oneAxis, one->halfSize[oneAxisIndex],
				ptOnTwoEdge, twoAxis, two->halfSize[twoAxisIndex],
				bestSingleAxis > 2
			);

			Contact* contact = new Contact();

			contact->penetration = pen;
			contact->contactNormal = axis;
			contact->contactPoint = vertex;

			contact->body[0] = one->rigidBody;
			contact->body[1] = two->rigidBody;

			//Material dependent, for now just set to constants
			contact->friction = globalFriction;
			contact->restitution = globalRestitution;

			return contact;
		}
		return NULL;
	}
};

