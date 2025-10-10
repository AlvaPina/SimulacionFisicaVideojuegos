#include "Particle.h"
#include <iostream>

extern std::vector<physx::PxShape*> gShapes;
extern std::vector<RenderItem*> gRenderItems;
extern std::vector<Particle*> gParticles;

Particle::Particle(Vector3D pos, Vector3D vel)
{
    _vel = vel;
    // Transform
    _tr = new PxTransform(physx::PxVec3(pos.getX(), pos.getY(), pos.getZ()));
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

void Particle::integrate(double t)
{
    // Actualiza la velocidad (Euler semi-implícito)
    Vector3D velInc = _aceleration.scalarMul(t);
    _vel = _vel + velInc;

    // Aplica damping
    _vel = _vel.scalarMul(1.0 - _damping);

    // calcular nueva pos
    Vector3D pos = Vector3D(_tr->p.x, _tr->p.y, _tr->p.z);
    pos = pos + _vel.scalarMul(t);

    // Actualiza la Transform de PhysX
    _tr->p = PxVec3(pos.getX(), pos.getY(), pos.getZ());
}

void Particle::addAcceleration(Vector3D acceleration)
{
    _aceleration = _aceleration + acceleration;
}
