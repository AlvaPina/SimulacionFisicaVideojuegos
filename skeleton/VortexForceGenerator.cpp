#include "VortexForceGenerator.h"
#include <cmath>

static inline double length(const Vector3D& v) {
    return std::sqrt(v.getX() * v.getX() + v.getY() * v.getY() + v.getZ() * v.getZ());
}

VortexForceGenerator::VortexForceGenerator(double K, const Vector3D& center,
    double radius, const Vector3D& axis)
    : K_(K), center_(center), radius_(radius), axis_(axis.normalized()) {
}

Vector3D VortexForceGenerator::windAt(const Vector3D& pos, const Vector3D& /*vel*/) const {
    // Vector desde el centro
    Vector3D r = pos - center_;

    // Proyección sobre el eje (axial) y componente radial en el plano perpendicular
    double axialMag = r.scalarProduct(axis_);
    Vector3D axial = axis_.scalarMul(axialMag);
    Vector3D radial = r - axial;

    double d = length(radial);
    if (radius_ > 0.0 && d > radius_) return Vector3D(0, 0, 0);
    if (d < 1e-6) return Vector3D(0, 0, 0);

    // Dirección tangencial: axis x radial (ya está implementado como operator*)
    Vector3D tang = (axis_ * radial).normalized();

    // Velocidad de viento tangencial proporcional a la distancia (modelo de la práctica)
    // v_wind = K * d * tang
    return tang.scalarMul(K_ * d);
}
