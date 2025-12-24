#include "Bullet.h"
#include <extensions/PxRigidBodyExt.h>

// globals que ya usas
extern physx::PxMaterial* gMaterial;
extern std::vector<RenderItem*> gRenderItems;
extern std::vector<physx::PxShape*> gShapes;

using namespace physx;

Bullet::Bullet(PxPhysics* physics,
    PxScene* scene,
    const PxTransform& startPose,
    const PxVec3& initialVelocity,
    float radius,
    float mass,
    const PxVec4& color)
{
    actor_ = physics->createRigidDynamic(startPose);

    // esfera
    shape_ = physics->createShape(PxSphereGeometry(radius), *gMaterial);
    actor_->attachShape(*shape_);

    // guardamos shape para cleanup global
    gShapes.push_back(shape_);

    PxRigidBodyExt::updateMassAndInertia(*actor_, mass);
    actor_->setLinearVelocity(initialVelocity);
    actor_->setAngularDamping(0.5f);
    actor_->setLinearDamping(0.0f);

    scene->addActor(*actor_);

    // render ligado al actor
    renderItem_ = new RenderItem(shape_, actor_, color);
    gRenderItems.push_back(renderItem_);
}

Bullet::~Bullet()
{
    alive_ = false;

    if (renderItem_) {
        renderItem_->release();
        renderItem_ = nullptr;
    }

    if (actor_) {
        actor_->release();
        actor_ = nullptr;
    }
}

void Bullet::update(float dt)
{
    if (!alive_) return;

    age_ += dt;
    if (age_ >= ttl_) {
        alive_ = false;
    }
}
