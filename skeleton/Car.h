#ifndef CAR_H
#define CAR_H

#include <PxPhysicsAPI.h>
#include <vector>
#include "RenderUtils.hpp" 
#include "ParticleGenerator.h"
#include "GravityForceGenerator.h"
#include "ForceRegistry.h"

class Car
{
public:
	Car(physx::PxPhysics* physics, physx::PxScene* scene, const physx::PxTransform& pose, const physx::PxVec3& halfExtents);
	~Car();

	void update(float dt);

	// Controles
	void setThrottle(float v);
	void setSteer(float v);
	void setTurbo(bool on);
	bool isTurbo() const { return turboActive_; }

	// Método necesario para la cámara
	physx::PxTransform GetTransform() const {
		if (actor_) return actor_->getGlobalPose();
		return physx::PxTransform(physx::PxIdentity);
	}

	physx::PxTransform GetGunTransform() const;

private:
	physx::PxPhysics* physics_;
	physx::PxScene* scene_;
	physx::PxRigidDynamic* actor_;
	RenderItem* renderItem_;

	// Arma
	physx::PxShape* gunShape_ = nullptr;
	RenderItem* gunRenderItem_ = nullptr;

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

	// Turbo
	bool turboActive_ = false;
	float turboForce_ = 200000.0f;     // empuje extra
	float turboSpeed_ = 1000.0f;        // velocidad partículas turbo ON
	float normalSpeed_ = 100.0f;        // velocidad partículas turbo OFF (si quieres que echen humo flojo)

	// Emisores (2)
	ParticleGenerator* turboLeft_ = nullptr;
	ParticleGenerator* turboRight_ = nullptr;

	// Fuerza (gravedad en el humo)
	GravityForceGenerator* turboGravity_ = nullptr;
	ForceRegistry* registry_ = nullptr; // guardamos el puntero que usamos

	// Offsets LOCALES de las toberas
	physx::PxVec3 turboLeftLocalOffset_;
	physx::PxVec3 turboRightLocalOffset_;

};

#endif