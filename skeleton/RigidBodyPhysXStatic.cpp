#include "RigidBodyPhysXStatic.h"
#include <limits>

RigidBodyPhysXStatic::RigidBodyPhysXStatic(PxRigidStatic* actor)
    : actor_(actor)
{
}

double RigidBodyPhysXStatic::getMass() const
{
    // Masa "infinita"
    return std::numeric_limits<double>::infinity();
}

Vector3D RigidBodyPhysXStatic::getPosition() const
{
    PxVec3 p = actor_->getGlobalPose().p;
    return Vector3D(p.x, p.y, p.z);
}

Vector3D RigidBodyPhysXStatic::getVelocity() const
{
    // Un estático no se mueve
    return Vector3D::ZERO;
}

void RigidBodyPhysXStatic::addForce(const Vector3D& f)
{
    // no hacemos nada, un estático no responde a fuerzas
    (void)f;
}
