#include "GravityForceGenerator.h"

GravityForceGenerator::GravityForceGenerator(Vector3D g) : g_(g) 
{
}

void GravityForceGenerator::setG(const Vector3D& g)
{
	g_ = g;
}

void GravityForceGenerator::apply(RigidBody& b, double dt)
{
	const Vector3D F = g_.scalarMul(b.getMass()); // F = m * g
	b.addForce(F);
}
