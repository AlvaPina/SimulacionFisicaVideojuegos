#include "WindForceGenerator.h"
#include <cmath>

static inline double length(const Vector3D& v) {
    return std::sqrt(v.getX() * v.getX() + v.getY() * v.getY() + v.getZ() * v.getZ());
}

WindForceGenerator::WindForceGenerator(const Vector3D& windVel, double k1, double k2)
    : windVel_(windVel), k1_(k1), k2_(k2) {
}

void WindForceGenerator::setVolume(const Vector3D& minPt, const Vector3D& maxPt) {
    boxMin_ = minPt;
    boxMax_ = maxPt;
}

bool WindForceGenerator::inside(const Vector3D& p) const {
    return (p.getX() >= boxMin_.getX() && p.getX() <= boxMax_.getX()) &&
        (p.getY() >= boxMin_.getY() && p.getY() <= boxMax_.getY()) &&
        (p.getZ() >= boxMin_.getZ() && p.getZ() <= boxMax_.getZ());
}

void WindForceGenerator::apply(RigidBody& b, double /*dt*/) {
    if (!isActive()) return;

    Vector3D pos = b.getPosition();
    if (!inside(pos)) return;

    Vector3D vel = b.getVelocity();
    Vector3D w = windAt(pos, vel);    // viento efectivo en la posición
    Vector3D rel = w - vel;             // (v_viento - v_partícula)

    double speed = length(rel);
    // F = k1*rel + k2*|rel|*rel
    Vector3D F = rel.scalarMul(k1_);
    if (k2_ != 0.0) {
        Vector3D extra = rel.scalarMul(k2_ * speed);
        F = F + extra;
    }
    b.addForce(F);
}
