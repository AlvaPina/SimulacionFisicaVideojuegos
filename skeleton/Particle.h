#pragma once

#include "Vector3D.h"
#include <PxPhysicsAPI.h>
#include "RenderUtils.hpp"

using namespace physx;

class Particle
{
public:
	Particle(Vector3D iniPos, Vector3D iniVel, Vector3D iniAcceleration, double iniMass);
	~Particle();

	void integrate(double t, int type = 0);
	void addForce(Vector3D force);
	void setGravity(bool value);
private:
	Vector3D _vel;
	Vector3D _aceleration;
	Vector3D _forceAccum = Vector3D(0.0, 0.0, 0.0);

	PxTransform* _tr;
	RenderItem* _renderItem;

	double _damping = 0.00001f;
	double _mass = 1.0f;
	double _gravityValue = -9.81;

	bool _gravity = true;

	void crealForceAccumulator();
};

