#include "Axis.h"
#include "RenderUtils.hpp"

extern std::vector<physx::PxShape*> gShapes;
extern std::vector<RenderItem*> gRenderItems;

Axis::Axis(float length)
{
    // Origen (blanco)
    createSphere(PxTransform(PxVec3(0.f, 0.f, 0.f)), PxVec4(1.f, 1.f, 1.f, 1.f));

    // X (rojo)
    createSphere(PxTransform(PxVec3(length, 0.f, 0.f)), PxVec4(1.f, 0.f, 0.f, 1.f));

    // Y (verde)
    createSphere(PxTransform(PxVec3(0.f, length, 0.f)), PxVec4(0.f, 1.f, 0.f, 1.f));

    // Z (azul)
    createSphere(PxTransform(PxVec3(0.f, 0.f, length)), PxVec4(0.f, 0.f, 1.f, 1.f));
}

void Axis::createSphere(const physx::PxTransform& transform, const physx::PxVec4& color)
{
    PxShape* shape = CreateShape(PxSphereGeometry(1));
    gShapes.push_back(shape);

    PxTransform* tr = new PxTransform(transform);

    RenderItem* item = new RenderItem(shape, tr, color);
    gRenderItems.push_back(item);
}
