#pragma once
#include "WindForceGenerator.h"

// Torbellino: viento tangencial alrededor de un centro, limitado por un radio y con eje configurable.
class VortexForceGenerator : public WindForceGenerator {
public:
    VortexForceGenerator(double K,
        const Vector3D& center,
        double radius,
        const Vector3D& axis = Vector3D(0, 1, 0));

    void setCenter(const Vector3D& c) { center_ = c; }
    void setRadius(double r) { radius_ = r; }
    void setAxis(const Vector3D& a) { axis_ = a.normalized(); }
    void setIntensity(double K) { K_ = K; }

protected:
    Vector3D windAt(const Vector3D& pos, const Vector3D& vel) const override;

private:
    double   K_{ 5.0 };            // Intensidad (velocidad tangencial ~ K * d)
    Vector3D center_{ 0,0,0 };     // Centro del vórtice
    double   radius_{ 5.0 };       // Radio de acción (<=0 => infinito)
    Vector3D axis_{ 0,1,0 };       // Eje del vórtice (normalizado idealmente)
};
