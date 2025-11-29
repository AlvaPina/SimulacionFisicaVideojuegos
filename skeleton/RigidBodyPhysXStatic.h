#pragma once
#include "RigidBody.h"
#include <PxPhysicsAPI.h>
#include "Vector3D.h"

using namespace physx;

class RigidBodyPhysXStatic : public RigidBody {
public:
    RigidBodyPhysXStatic(PxRigidStatic* actor);

    double getMass() const override;
    Vector3D getPosition() const override;
    Vector3D getVelocity() const override;
    void addForce(const Vector3D& f) override;

private:
    PxRigidStatic* actor_;
};
