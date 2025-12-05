#pragma once
#include "RigidBody.h"
#include <PxPhysicsAPI.h>

using namespace physx;

class RigidBodyPhysX : public RigidBody {
public:
    RigidBodyPhysX(PxRigidDynamic* actor);

    double getMass() const override;
    Vector3D getPosition() const override;
    Vector3D getVelocity() const override;

    void addForce(const Vector3D& f) override;

private:
    PxRigidDynamic* actor_;

};
