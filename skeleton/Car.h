#pragma once
#include <PxPhysicsAPI.h>
#include "RenderUtils.hpp" 

using namespace physx;

class Car
{
public:
	Car(PxPhysics* physics, PxScene* scene, const PxTransform& pose, const PxVec3& halfExtents);
	~Car();

	void update(float dt);

	void setThrottle(float v);
	void setSteer(float v);

	PxRigidDynamic* getActor() const { return actor_; }

	physx::PxTransform GetTransform() const
	{
		// Asumo que tu actor rígido se llama mVehicleActor, mActor, o similar.
		// Cambia 'mActor' por el nombre real de tu variable PxRigidDynamic* en la clase Car.
		if (actor_)
			return actor_->getGlobalPose();

		return physx::PxTransform(physx::PxIdentity);
	}

private:
	PxPhysics* physics_;
	PxScene* scene_;
	PxRigidDynamic* actor_;
	RenderItem* renderItem_;

	float throttle_;
	float steer_;

	float moveForce_;
	float turnTorque_;

	// Suspensión
	float suspensionRestLength_;
	float springStrength_;
	float springDamper_;
	float carHalfHeight_;

	// Debug
	int frameCounter_; // Para controlar los prints
};