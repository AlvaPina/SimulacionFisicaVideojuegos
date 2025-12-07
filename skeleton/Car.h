#ifndef CAR_H
#define CAR_H

#include <PxPhysicsAPI.h>
#include <vector>
#include "RenderUtils.hpp" 

class Car
{
public:
	Car(physx::PxPhysics* physics, physx::PxScene* scene, const physx::PxTransform& pose, const physx::PxVec3& halfExtents);
	~Car();

	void update(float dt);

	// Controles
	void setThrottle(float v);
	void setSteer(float v);

	// Método necesario para la cámara
	physx::PxTransform GetTransform() const {
		if (actor_) return actor_->getGlobalPose();
		return physx::PxTransform(physx::PxIdentity);
	}

private:
	physx::PxPhysics* physics_;
	physx::PxScene* scene_;
	physx::PxRigidDynamic* actor_;
	RenderItem* renderItem_;

	// Parámetros del coche
	float throttle_; // -1 a 1
	float steer_;    // -1 a 1

	float moveForce_;
	float turnTorque_;

	// Suspensión
	float suspensionRestLength_;
	float springStrength_;
	float springDamper_;
	float carHalfHeight_;

	int frameCounter_;
};

#endif