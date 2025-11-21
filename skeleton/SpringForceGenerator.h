#pragma once

#include "ForceGenerator.h"
#include "RigidBody.h"
#include "core.hpp"

class SpringForceGenerator : public ForceGenerator {
public:
    SpringForceGenerator(double k, double resting_length, RigidBody* rigidbody);

    virtual void updateForce(RigidBody* rigidbody);

    inline void setK(double k) { _k = k; }

    virtual ~SpringForceGenerator() {}

protected:
    double k_; // Elastic Coeff.
    double resting_length_;
    RigidBody* rigidbody_;
};
