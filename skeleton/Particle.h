#pragma once

#include "Vector3D.h"
#include <PxPhysicsAPI.h>
#include "RenderUtils.hpp"
#include <memory>

using namespace physx;

class Particle
{
public:
	Particle(Vector3D pos, Vector3D vel);
	~Particle();

	void integrate(double t);
private:
	Vector3D _vel;
	std::unique_ptr<PxTransform> _tr;
	std::unique_ptr<RenderItem> _renderItem;
};

