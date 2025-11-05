#pragma once
#include "Vector3D.h"

class RigidBody {
public:
    virtual ~RigidBody() = default;
    virtual double getMass() const = 0;
    virtual Vector3D getPosition() const = 0;
    virtual Vector3D getVelocity() const = 0;

    // Suma una fuerza (N). La integración la hace el objeto (o tu integrador global).
    virtual void addForce(const Vector3D& f) = 0;
};

