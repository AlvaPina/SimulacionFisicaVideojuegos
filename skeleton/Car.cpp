// Car.cpp
#include "Car.h"
#include "core.hpp"              // Para CreateShape
#include "extensions/PxRigidBodyExt.h"

using namespace physx;

Car::Car(PxPhysics* physics, PxScene* scene,
    const PxTransform& pose, const PxVec3& halfExtents)
    : mPhysics(physics), mScene(scene)
{
    // Crear chasis como caja alargada
    mChassis = mPhysics->createRigidDynamic(pose);

    PxShape* chassisShape = CreateShape(PxBoxGeometry(halfExtents));
    mChassis->attachShape(*chassisShape);

    // Ajustar masa e inercia
    PxRigidBodyExt::updateMassAndInertia(*mChassis, 700.0f); // 800 kg, por ejemplo

    mScene->addActor(*mChassis);

    // Render del chasis (azul)
    mChassisRender = new RenderItem(chassisShape, mChassis, PxVec4(0, 0, 1, 1));

    // ----------------- Configuración de ruedas -----------------
    // Posiciones locales (delante/atrás, izquierda/derecha, abajo)
    const float sx = halfExtents.x - 0.5f;
    const float sz = halfExtents.z - 0.5f;
    const float sy = -halfExtents.y + 5.5f;         // parte baja del chasis

    const float restLen = 5.2f;             // longitud de reposo del muelle
    const float maxLen = 10.0f;             // máximo de raycast
    const float stiffness = 18000.0f;        // k del muelle
    const float damping = 1000.0f;         // amortiguación
    const float wheelR = 1.6f;            // radio "visual" rueda

    // Front-left
    mWheels[0].localPos = PxVec3(-sx, sy, sz);
    mWheels[0].restLength = restLen;
    mWheels[0].maxLength = maxLen;
    mWheels[0].stiffness = stiffness;
    mWheels[0].damping = damping;
    mWheels[0].radius = wheelR;

    // Front-right
    mWheels[1] = mWheels[0];
    mWheels[1].localPos.x = +sx;

    // Rear-left
    mWheels[2] = mWheels[0];
    mWheels[2].localPos.z = -sz;

    // Rear-right
    mWheels[3] = mWheels[2];
    mWheels[3].localPos.x = +sx;
}

Car::~Car()
{
    DeregisterRenderItem(mChassisRender);
    delete mChassisRender;
}

void Car::update(float dt)
{
    if (!mChassis) return;

    // Suspensiones (raycasts + muelles)
    for (int i = 0; i < 4; ++i)
        addSuspensionForce(mWheels[i], dt);

    // Motor sencillo
    addEngineForce(dt);

    // Giro sencillo
    addSteerTorque(dt);
}

// Aplica fuerza de suspensión mediante raycast
void Car::addSuspensionForce(Wheel& w, float dt)
{
    PxTransform pose = mChassis->getGlobalPose();

    // Posición mundial del punto de anclaje de la rueda
    PxVec3 wheelWorldPos = pose.transform(w.localPos);

    // Dirección del raycast: hacia abajo en mundo (y negativo).
    // Si quieres suspensiones "pegadas" al chasis, podrías usar el eje local,
    // pero para simplificar usamos vector global (0,-1,0).
    PxVec3 rayDir(0.0f, -1.0f, 0.0f);

    // Distancia máxima = longitud del muelle + algo de “slack”
    float maxDist = w.maxLength + w.radius;

    PxRaycastBuffer hit;
    bool blocked = mScene->raycast(wheelWorldPos, rayDir, maxDist,
        hit, PxHitFlag::ePOSITION | PxHitFlag::eNORMAL);
    if (!blocked) return;

    // Distancia real al contacto (desde el punto de anclaje)
    float dist = hit.block.distance - w.radius;  // compensar radio de rueda
    if (dist < 0.0f) dist = 0.0f;

    // Compresión del muelle: reposo - distancia
    float compression = w.restLength - dist;
    if (compression <= 0.0f) return;  // no comprimido -> no hay fuerza de resorte

    // Velocidad del punto donde está la rueda
    PxVec3 relPos = wheelWorldPos - mChassis->getGlobalPose().p;
    PxVec3 velAtPoint =
        mChassis->getLinearVelocity() +
        mChassis->getAngularVelocity().cross(relPos);

    float velAlongRay = velAtPoint.dot(rayDir); // componente en dirección del raycast

    // Hooke + amortiguación (en magnitud)
    float springForceMag = w.stiffness * compression;
    float dampingForceMag = w.damping * velAlongRay;

    float totalForceMag = springForceMag - dampingForceMag;
    if (totalForceMag < 0.0f) totalForceMag = 0.0f; // no “chupa” hacia abajo

    // La fuerza va en dirección contraria al raycast (hacia arriba)
    PxVec3 force = -rayDir * totalForceMag;

    // Aplicar fuerza en la posición de la rueda
    PxRigidBodyExt::addForceAtPos(*mChassis, force, wheelWorldPos, PxForceMode::eFORCE);
}

// Fuerza de motor muy simple: empuja el chasis hacia delante según throttle
void Car::addEngineForce(float dt)
{
    PX_UNUSED(dt);

    if (mThrottle == 0.0f) return;

    PxTransform pose = mChassis->getGlobalPose();

    // Suponemos que el eje "forward" del chasis es +Z
    PxVec3 forward = pose.q.rotate(PxVec3(0.0f, 0.0f, 1.0f));

    PxVec3 force = forward * (mEngineForce * mThrottle);
    mChassis->addForce(force, PxForceMode::eFORCE);
}

void Car::addSteerTorque(float dt)
{
    PX_UNUSED(dt);

    if (!mChassis) return;

    // Entrada de giro [-1,1]
    const float s = mSteer;
    if (fabs(s) < 1e-3f) return;

    // Escalar por velocidad, para que no gire loco estando parado
    float speed = mChassis->getLinearVelocity().magnitude();
    if (speed < 0.5f) return; // casi parado -> no giramos

    if (speed > 30.0f) speed = 30.0f; // clamp
    float factor = speed / 30.0f;     // 0..1

    float torqueMag = mSteerTorque * s * factor;

    // Torque alrededor de Y (giro yaw)
    PxVec3 torque(0.0f, torqueMag, 0.0f);
    mChassis->addTorque(torque, PxForceMode::eFORCE);
}
