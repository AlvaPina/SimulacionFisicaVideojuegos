#include "ExplosionForceGenerator.h"
#include <cmath>

static inline double length(const Vector3D& v) {
    return std::sqrt(v.getX() * v.getX() + v.getY() * v.getY() + v.getZ() * v.getZ());
}

ExplosionForceGenerator::ExplosionForceGenerator(const Vector3D& center, double K, double R0, double tau, double ve)
    : center_(center), K_(K), R0_(R0), tau_(tau), ve_(ve) {
}

void ExplosionForceGenerator::resetTime() {
    elapsed_ = 0.0;
}

void ExplosionForceGenerator::apply(RigidBody& b, double dt) {
    if (!isActive()) return;

    elapsed_ += dt;
    double R = R0_ + ve_ * elapsed_; // si ve=0 => radio constante

    Vector3D r = b.getPosition() - center_;
    double dist = length(r);
    if (dist <= 1e-6 || dist > R) return;

    Vector3D dir = r.scalarMul(1.0 / dist);
    double decay = std::exp(-elapsed_ / (tau_ > 1e-6 ? tau_ : 1e-6));
    double mag = (K_ / (dist * dist)) * decay;

    b.addForce(dir.scalarMul(mag));
}
