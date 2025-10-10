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
	void addAcceleration(Vector3D acceleration);
	void setGravity(bool value);
private:
	Vector3D _vel;
	Vector3D _aceleration;
	PxTransform* _tr;
	RenderItem* _renderItem;

	double _damping = 0.01f;
	double _mass = 1.0f;
};

