#include "Particle.h"
#include <iostream>
#include <cmath>

Particle::Particle(Vector3D iniPos, Vector3D iniVelReal, double realMass, float radius, const PxVec4& color, float visualSpeedScale)
{
    _tr = new PxTransform(physx::PxVec3(iniPos.getX(), iniPos.getY(), iniPos.getZ()));
    _aceleration = Vector3D(0, 0, 0);

    _realMass = (realMass > 1e-6) ? realMass : 1e-6;
    _radius = radius;
    _color = color;

    _visualSpeedScale = clampScale(visualSpeedScale);

    // Guardamos “real” y calculamos “simulada”
    _velReal = iniVelReal;
    _vel = iniVelReal.scalarMul(_visualSpeedScale);

    // Masa efectiva para conservar ENERGÍA: m_eff = m_real / s^2
    const double s = static_cast<double>(_visualSpeedScale);
    _mass = _realMass / (s * s);

    // Render
    _renderItem = new RenderItem(CreateShape(PxSphereGeometry(_radius)), _tr, _color);
}

Particle::~Particle()
{
    if (_renderItem) {
        DeregisterRenderItem(_renderItem);
        delete _renderItem;
        _renderItem = nullptr;
    }

    if (_tr) {
        delete _tr;
        _tr = nullptr;
    }
}

void Particle::setVisualSpeedScale(float s)
{
    _visualSpeedScale = clampScale(s);

    // Mantén la dirección y velocidad real, recalcula velocidad visible
    _vel = _velReal.scalarMul(_visualSpeedScale);

    // Recalcula masa efectiva para conservar energía
    const double sd = static_cast<double>(_visualSpeedScale);
    _mass = _realMass / (sd * sd);
}

void Particle::integrate(double t, int type)
{
    // a = F / m_eff
    _aceleration = _forceAccum.scalarMul(1.0 / _mass);

    Vector3D pos(_tr->p.x, _tr->p.y, _tr->p.z);

    switch (type) {
    case 0: // Euler semi-implícito
    {
        _vel = _vel + _aceleration.scalarMul(t);
        _vel = _vel.scalarMul(1.0 - _damping);
        pos = pos + _vel.scalarMul(t);
        break;
    }
    case 1: // Euler explícito
    {
        pos = pos + _vel.scalarMul(t);
        _vel = _vel + _aceleration.scalarMul(t);
        _vel = _vel.scalarMul(1.0 - _damping);
        break;
    }
    default:
        break;
    }

    _tr->p = PxVec3(pos.getX(), pos.getY(), pos.getZ());

    // Si quieres que _velReal siga siendo la “real”, puedes reconstruirla así:
    // (asumiendo que “real” es “visible / scale”)
    const double s = static_cast<double>(_visualSpeedScale);
    _velReal = _vel.scalarMul(1.0 / s);

    clearForceAccumulator();
}

double Particle::getMass() const
{
    return _mass; // masa EFECTIVA (la que usan las fuerzas)
}

Vector3D Particle::getPosition() const
{
    return Vector3D(_tr->p.x, _tr->p.y, _tr->p.z);
}

Vector3D Particle::getVelocity() const
{
    return _vel; // velocidad SIMULADA (visible)
}

void Particle::addForce(const Vector3D& force)
{
    _forceAccum = _forceAccum + force;
}

void Particle::clearForceAccumulator()
{
    _forceAccum = Vector3D(0.0, 0.0, 0.0);
}
