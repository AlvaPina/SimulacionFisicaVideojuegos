#include "Particle.h"

void Particle::integrate(double t)
{
    // Aceleraci�n (ejemplo: gravedad)
    Vector3D gravity(0.0, -9.81, 0.0);

    // Actualiza la velocidad (Euler semi-impl�cito)
    vel = vel + gravity.scalarMul(t);

    // Actualiza la posici�n usando la nueva velocidad
    Vector3D pos = Vector3D(pose.p.x, pose.p.y, pose.p.z);
    pos = pos + vel.scalarMul(t);

    // Actualiza la pose de PhysX
    pose = physx::PxTransform(physx::PxVec3(pos.getX(), pos.getY(), pos.getZ()));

    // Render Item
    
}
