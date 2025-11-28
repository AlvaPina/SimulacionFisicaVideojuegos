#include "RigidBodyPhysX.h"

RigidBodyPhysX::RigidBodyPhysX(PxRigidDynamic* actor) : actor_(actor) {}

double RigidBodyPhysX::getMass() const {
    return actor_->getMass();
}

Vector3D RigidBodyPhysX::getPosition() const {
    PxVec3 p = actor_->getGlobalPose().p;
    return Vector3D(p.x, p.y, p.z);
}

Vector3D RigidBodyPhysX::getVelocity() const {
    PxVec3 v = actor_->getLinearVelocity();
    return Vector3D(v.x, v.y, v.z);
}

void RigidBodyPhysX::addForce(const Vector3D& f) {
    actor_->addForce(PxVec3(f.getX(), f.getY(), f.getZ()));
}
