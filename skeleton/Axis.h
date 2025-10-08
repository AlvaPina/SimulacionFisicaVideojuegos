#pragma once
#include <PxPhysicsAPI.h>

using namespace physx;

class Axis {
public:
    Axis(float size = 10.f);
    ~Axis();
private:
    PxTransform* _tr;
    void createSphere(const physx::PxTransform& transform, const physx::PxVec4& color, float size);
};


