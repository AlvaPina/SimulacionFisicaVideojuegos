// Car.h
#pragma once

#include <PxPhysicsAPI.h>
#include "RenderUtils.hpp"

using namespace physx;

class Car
{
public:
    struct Wheel
    {
        PxVec3 localPos;     // posición de la rueda en espacio local del chasis
        float  restLength;   // longitud de reposo del muelle
        float  maxLength;    // longitud máxima del raycast
        float  stiffness;    // constante elástica (k)
        float  damping;      // amortiguación
        float  radius;       // radio visual / "rueda" (para ajustar la distancia)
    };

    Car(PxPhysics* physics,
        PxScene* scene,
        const PxTransform& pose,
        const PxVec3& halfExtents);

    ~Car();

    void update(float dt);

    PxRigidDynamic* getActor() { return mChassis; }

    // Controles
    void setThrottle(float t) { mThrottle = t; }  // -1..1
    void setSteer(float s) { mSteer = s; }     // -1..1  (izq/der)

private:
    PxPhysics* mPhysics = nullptr;
    PxScene* mScene = nullptr;
    PxRigidDynamic* mChassis = nullptr;

    RenderItem* mChassisRender = nullptr;

    Wheel mWheels[4];

    float mEngineForce = 8000.0f;   // fuerza máxima del motor
    float mThrottle = 0.0f;      // entrada [-1,1]

    // Steering
    float mSteer = 0.0f;      // entrada [-1,1]
    float mSteerTorque = 5000.0f;   // magnitud del torque para girar

    void addSuspensionForce(Wheel& w, float dt);
    void addEngineForce(float dt);
    void addSteerTorque(float dt);
};
