#pragma once

#include "Vector3D.h"
#include <PxPhysicsAPI.h>
#include "RenderUtils.hpp"

using namespace physx;

class Particle
{
public:
	Particle(Vector3D pos, Vector3D vel);
	~Particle();

	void integrate(double t);
	void addAcceleration(Vector3D acceleration);
private:
	Vector3D _vel;
	Vector3D _aceleration;
	PxTransform* _tr;
	RenderItem* _renderItem;

	double _damping = 0.01f;
};

