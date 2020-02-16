#pragma once
#include "RigidBody.h"
#include "Contact.h"
#include "CollisionDetector.h"
#include "Colliders.h"

class World
{
public:

	std::vector<RigidBody*> bodies;
	std::vector<Collider*> colliders;


	void RunPhysics(real duration)
	{
		for (int i = 0; i < bodies.size(); i++)
		{
			bodies[i]->Integrate(duration);
		}

		for (int i = 0; i < colliders.size(); i++)
		{
			for (int j = i + 1; j < colliders.size(); j++)
			{
				colliders[i]->calculateInternals();
				colliders[j]->calculateInternals();

				Contact * contact = CollisionDetector::DetectCollision(colliders[i], colliders[j]);

				if (contact)
				{
					contact->ResolveCollision(duration);

					delete contact;

				}
			}
		}

	}

	~World()
	{
		for (RigidBody* body : bodies)
		{
			delete body;
		}
		
		for (Collider* collider : colliders)
		{
			delete collider;
		}
	}

};