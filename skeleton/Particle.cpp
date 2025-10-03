#include "Particle.h"

Particle::Particle(Vector3D pos, Vector3D vel)
{
    _vel = vel;
    // Transform
    _tr = std::make_unique <PxTransform>(physx::PxVec3(pos.getX(), pos.getY(), pos.getZ()));
    // Render Item
    _renderItem = std::make_unique<RenderItem>(CreateShape(PxSphereGeometry(1)), _tr, PxVec4(0 / 255.f, 0 / 255.f, 255 / 255.f, 1));
}

Particle::~Particle() = default;

void Particle::integrate(double t)
{
    // Aceleración
    Vector3D gravity(0.0, -9.81, 0.0);

    // Actualiza la velocidad (Euler semi-implícito)
    _vel = _vel + gravity.scalarMul(t);

    // calcular nueva pos
    Vector3D pos = Vector3D(_tr->p.x, _tr->p.y, _tr->p.z);
    pos = pos + _vel.scalarMul(t);

    // Actualiza la Transform de PhysX
    _tr->p = PxVec3(pos.getX(), pos.getY(), pos.getZ());
}
