#pragma once

#include "ForceGenerator.h"
#include "RigidBody.h"
#include "core.hpp"

class SpringForceGenerator : public ForceGenerator {
public:
    SpringForceGenerator(double k, double resting_length, RigidBody* rigidbody);

	void apply(RigidBody& body, double dt) override;
    virtual void updateForce(RigidBody* rigidbody);

    inline void setK(double k) { k_ = k; }

    virtual ~SpringForceGenerator() {}

protected:
    double k_; // Elastic Coeff.
    double resting_length_;
    RigidBody* rigidbody_;
};
