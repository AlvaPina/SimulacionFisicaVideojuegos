#include "Particle.h"
#include <iostream>

extern std::vector<physx::PxShape*> gShapes;
extern std::vector<RenderItem*> gRenderItems;
extern std::vector<Particle*> gParticles;

Particle::Particle(Vector3D iniPos, Vector3D iniVel, Vector3D iniAcceleration, double iniMass)
{
    _vel = iniVel;
    _tr = new PxTransform(physx::PxVec3(iniPos.getX(), iniPos.getY(), iniPos.getZ()));
    _aceleration = iniAcceleration;
    _mass = iniMass;
    // Render Item
    _renderItem = new RenderItem(CreateShape(PxSphereGeometry(1)), _tr, PxVec4(0 / 255.f, 0 / 255.f, 255 / 255.f, 1));
    gRenderItems.push_back(_renderItem);
    // Añadimos gravedad a la particula
    Vector3D gravity(0.0, -9.81, 0.0);
    addAcceleration(gravity);
    // Añadimos la particula a nuestro vector de particulas
    gParticles.push_back(this);
}

Particle::~Particle() {
    delete _tr;
    // todos los _renderItem ya se borran en el main
}

void Particle::integrate(double t, int type) // 0: euler semi-implicito, 1: euler explicito
{
    Vector3D pos(_tr->p.x, _tr->p.y, _tr->p.z);
    switch (type) {
    case 0: // Euler semi-implícito
    {
        // Actualiza velocidad primero
        _vel = _vel + _aceleration.scalarMul(t);

        // Aplica damping
        _vel = _vel.scalarMul(1.0 - _damping);

        // Actualiza posición con la nueva velocidad
        pos = pos + _vel.scalarMul(t);
        break;
    }
    case 1: // Euler explícito
    {
        // Calcula nueva posición usando la velocidad actual
        pos = pos + _vel.scalarMul(t);

        // Actualiza velocidad después
        _vel = _vel + _aceleration.scalarMul(t);

        // Aplica damping
        _vel = _vel.scalarMul(1.0 - _damping);
        break;
    }
    default:
        break;
    }

    // Actualiza la Transform de PhysX
    _tr->p = PxVec3(pos.getX(), pos.getY(), pos.getZ());
}

void Particle::addAcceleration(Vector3D acceleration)
{
    _aceleration = _aceleration + acceleration;
}
