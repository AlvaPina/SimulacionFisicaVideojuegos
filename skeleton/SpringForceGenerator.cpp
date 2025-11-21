#include "SpringForceGenerator.h"

SpringForceGenerator::SpringForceGenerator(double k, double resting_length, RigidBody* rigidbody) : ForceGenerator() {
    k_ = k;
    resting_length_ = resting_length;
    rigidbody_ = rigidbody;
}

void SpringForceGenerator::updateForce(RigidBody* rigidbody) {
    // Particle is the particle to apply the force
    Vector3D relative_pos_vector = rigidbody_->getPosition() - rigidbody_->getPosition();
    Vector3D force;

    // normalize: Normalize the relative_pos_vector and returns its length.
    const float length = relative_pos_vector.normalize();
    const float delta_x = length - resting_length_;

    force = relative_pos_vector * delta_x * k_;

    rigidbody->addForce(force);
}
