#pragma once
#include "ForceGenerator.h"
#include "Vector3D.h"
#include "RigidBody.h"

class GravityForceGenerator : public ForceGenerator {
public:
    GravityForceGenerator(Vector3D g = { 0,-9.81,0 });
    void setG(const Vector3D& g);
    void apply(RigidBody& b, double dt) override;
private:
    Vector3D g_;
};


