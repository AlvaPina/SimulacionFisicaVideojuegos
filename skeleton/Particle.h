#pragma once

#include "Vector3D.h"
#include <PxPhysicsAPI.h>
#include "RenderUtils.hpp"

class Particle
{
public:
	Particle(Vector3D Pos, Vector3D Vel);
	~Particle();

	void integrate(double t);
private:
	Vector3D vel;
	physx::PxTransform pose;
	RenderItem* renderItem;
};

