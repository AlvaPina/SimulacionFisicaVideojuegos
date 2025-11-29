#include "SpringForceGenerator.h"
#include "RigidBody.h"
#include "Vector3D.h"

SpringForceGenerator::SpringForceGenerator(double k, double resting_length, RigidBody* rigidbody)
    : k_(k), resting_length_(resting_length), rigidbody_(rigidbody)
{
}

// Este es el método que SÍ necesita la base ForceGenerator
void SpringForceGenerator::apply(RigidBody& body, double dt)
{
    if (!active_) return;

    // Convertimos la referencia a puntero para reutilizar updateForce(...)
    updateForce(&body, dt);
}

void SpringForceGenerator::updateForce(RigidBody* body, double dt)
{
    if (!body || !rigidbody_) return;

    Vector3D dir = body->getPosition() - rigidbody_->getPosition();
    double length = dir.normalize();
    if (length == 0.0) return;

    double delta_x = length - resting_length_;

    // Hooke
    Vector3D springForce = dir.scalarMul(-k_ * delta_x);

    // Amortiguación
    Vector3D relVel = body->getVelocity() - rigidbody_->getVelocity();
    double c = 2.0 * sqrt(k_);
    Vector3D dampingForce = relVel.scalarMul(-c * dt);

    // Total
    Vector3D force = springForce + dampingForce;

    body->addForce(force);
}
