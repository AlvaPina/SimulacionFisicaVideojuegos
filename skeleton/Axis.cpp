#include "Axis.h"
#include "RenderUtils.hpp"

extern std::vector<physx::PxShape*> gShapes;
extern std::vector<RenderItem*> gRenderItems;

Axis::Axis(float size)
{
    // Origen (blanco)
    createSphere(PxTransform(PxVec3(0.f, 0.f, 0.f)), PxVec4(1.f, 1.f, 1.f, 1.f), size);

    // X (rojo)
    createSphere(PxTransform(PxVec3(size, 0.f, 0.f)), PxVec4(1.f, 0.f, 0.f, 1.f), size);

    // Y (verde)
    createSphere(PxTransform(PxVec3(0.f, size, 0.f)), PxVec4(0.f, 1.f, 0.f, 1.f), size);

    // Z (azul)
    createSphere(PxTransform(PxVec3(0.f, 0.f, size)), PxVec4(0.f, 0.f, 1.f, 1.f),size);
}

Axis::~Axis()
{
    delete _tr;
}

void Axis::createSphere(const physx::PxTransform& transform, const physx::PxVec4& color, float size)
{
    PxShape* shape = CreateShape(PxSphereGeometry(size * 0.1));
    gShapes.push_back(shape);

    _tr = new PxTransform(transform);

    RenderItem* item = new RenderItem(shape, _tr, color);
    gRenderItems.push_back(item);
}
