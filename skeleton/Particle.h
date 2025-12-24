#pragma once

#include "Vector3D.h"
#include <PxPhysicsAPI.h>
#include "RenderUtils.hpp"
#include "ForceGenerator.h"
#include "RigidBody.h"
#include <vector>

using namespace physx;

class Particle : public RigidBody
{
public:
	Particle(Vector3D iniPos, Vector3D iniVel, double iniMass,
		float radius = 1.0f, const physx::PxVec4& color = physx::PxVec4(0, 0, 1, 1));
	~Particle();

	void integrate(double t, int type = 0);

	// implementación de la interfaz RigidBody
	double getMass() const override;
	Vector3D getPosition() const override;
	Vector3D getVelocity() const override;
	void addForce(const Vector3D& force) override;
private:
	Vector3D _vel;
	Vector3D _aceleration;
	Vector3D _forceAccum = Vector3D(0.0, 0.0, 0.0);

	PxTransform* _tr;
	RenderItem* _renderItem;

	double _damping = 0.00001f;
	double _mass = 1.0f;

	float _radius = 1.0f;
	PxVec4 _color = PxVec4(0.f, 0.f, 1.f, 1.f);

	void clearForceAccumulator();
};

