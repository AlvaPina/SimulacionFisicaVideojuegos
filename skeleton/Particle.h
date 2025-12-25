#pragma once

#include "Vector3D.h"
#include <PxPhysicsAPI.h>
#include "RenderUtils.hpp"
#include "RigidBody.h"

using namespace physx;

class Particle : public RigidBody
{
public:
    // visualSpeedScale:
    //  - 1.0  => se mueve a velocidad real
    //  - 0.25 => se mueve 4x más lento, pero con masa efectiva 16x para conservar energía (m/s^2)
    Particle(
        Vector3D iniPos,
        Vector3D iniVelReal,
        double   realMass,
        float    radius = 1.0f,
        const physx::PxVec4& color = physx::PxVec4(0, 0, 1, 1),
        float    visualSpeedScale = 1.0f
    );

    ~Particle();

    void integrate(double t, int type = 0);

    // RigidBody interface
    double   getMass() const override;          // masa EFECTIVA (para fuerzas)
    Vector3D getPosition() const override;
    Vector3D getVelocity() const override;      // velocidad SIMULADA (visible)
    void     addForce(const Vector3D& force) override;

    // Extras útiles
    double   getRealMass() const { return _realMass; }
    double   getEffectiveMass() const { return _mass; }
    Vector3D getRealVelocity() const { return _velReal; }
    float    getVisualSpeedScale() const { return _visualSpeedScale; }

    // Cambiar en runtime (si lo necesitas)
    void setVisualSpeedScale(float s);

private:
    // Velocidad visible (la que integra)
    Vector3D _vel;

    // Velocidad “real” (la que representa físicamente)
    Vector3D _velReal;

    // Aceleración y fuerzas
    Vector3D _aceleration;
    Vector3D _forceAccum = Vector3D(0.0, 0.0, 0.0);

    PxTransform* _tr = nullptr;
    RenderItem*  _renderItem = nullptr;

    double _damping = 0.00001f;

    // realMass = masa física “de verdad”
    double _realMass = 1.0;

    // _mass = masa EFECTIVA usada en la simulación (m_real / s^2)
    double _mass = 1.0;

    float  _visualSpeedScale = 1.0f;

    float _radius = 1.0f;
    PxVec4 _color  = PxVec4(0.f, 0.f, 1.f, 1.f);

    void clearForceAccumulator();

    static float clampScale(float s)
    {
        // evita división por 0 y valores raros
        if (s < 0.01f) return 0.01f;
        if (s > 1.0f)  return 1.0f;
        return s;
    }
};
