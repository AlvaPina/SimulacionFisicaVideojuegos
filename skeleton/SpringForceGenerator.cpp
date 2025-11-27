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
    updateForce(&body);
}

void SpringForceGenerator::updateForce(RigidBody* body)
{
    if (!body || !rigidbody_) return;

    // Vector desde body hacia el rigidbody_ (anclaje)
    Vector3D dir = body->getPosition() - rigidbody_->getPosition();

    double length = dir.normalize(); // normaliza dir y devuelve su longitud original
    if (length == 0.0) return;

    double delta_x = length - resting_length_;

    // Ley de Hooke: F = -k * x * dirección
    Vector3D force = dir.scalarMul(-k_ * delta_x);

    body->addForce(force);
}
