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
	Particle(Vector3D iniPos, Vector3D iniVel, double iniMass);
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

	void clearForceAccumulator();
};

