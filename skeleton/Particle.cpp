#include "Particle.h"
#include <iostream>

Particle::Particle(Vector3D iniPos, Vector3D iniVel, double iniMass, float radius, const PxVec4& color)
{
    _vel = iniVel;
    _tr = new PxTransform(physx::PxVec3(iniPos.getX(), iniPos.getY(), iniPos.getZ()));
    _aceleration = Vector3D(0,0,0);
    _mass = iniMass;
    _radius = radius;
    _color = color;
    // Render Item
    _renderItem = new RenderItem(CreateShape(PxSphereGeometry(_radius)), _tr, _color);
}

Particle::~Particle() {
    if (_renderItem) {
        DeregisterRenderItem(_renderItem);  // si RenderUtils lo usa
        delete _renderItem;
        _renderItem = nullptr;
    }

    if (_tr) {
        delete _tr;
        _tr = nullptr;
    }
}

void Particle::integrate(double t, int type) // 0: euler semi-implicito, 1: euler explicito
{
    //Calcula la aceleración a partir de las fuerzas acumuladas
    _aceleration = _forceAccum.scalarMul(1.0 / _mass);

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
    // limpiar acumulador de fuerzas
    clearForceAccumulator();
}

double Particle::getMass() const
{
    return _mass;
}

Vector3D Particle::getPosition() const
{
    return Vector3D(_tr->p.x, _tr->p.y, _tr->p.z);
}

Vector3D Particle::getVelocity() const
{
    return _vel;
}

void Particle::addForce(const Vector3D& force)
{
    _forceAccum = _forceAccum + force;
}

void Particle::clearForceAccumulator()
{
	_forceAccum = Vector3D(0.0, 0.0, 0.0);
}
