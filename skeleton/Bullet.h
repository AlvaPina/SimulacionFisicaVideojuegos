#pragma once
#include <PxPhysicsAPI.h>
#include "RenderUtils.hpp"

class Bullet
{
public:
    Bullet(physx::PxPhysics* physics,
        physx::PxScene* scene,
        const physx::PxTransform& startPose,
        const physx::PxVec3& initialVelocity,
        float radius = 0.3f,
        float mass = 1.0f,
        const physx::PxVec4& color = physx::PxVec4(1, 1, 0, 1));

    ~Bullet();

    void update(float dt);
    bool isAlive() const { return alive_; }

    physx::PxRigidDynamic* getActor() const { return actor_; }

private:
    physx::PxRigidDynamic* actor_ = nullptr;
    physx::PxShape* shape_ = nullptr;
    RenderItem* renderItem_ = nullptr;

    float ttl_ = 5.0f;   // segundos
    float age_ = 0.0f;
    bool  alive_ = true;
};
