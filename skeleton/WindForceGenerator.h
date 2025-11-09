#pragma once
#include "ForceGenerator.h"
#include "Vector3D.h"
#include "RigidBody.h"
#include <cfloat>

// Viento en un AABB. Fuerza ~ k1*(v_w - v) + k2*||v_w - v||*(v_w - v)
class WindForceGenerator : public ForceGenerator {
public:
    WindForceGenerator(const Vector3D& windVel = Vector3D(0, 0, 0),
        double k1 = 0.5, double k2 = 0.0);

    // Define el volumen de influencia (AABB)
    void setVolume(const Vector3D& minPt, const Vector3D& maxPt);

    // Parámetros
    void setWind(const Vector3D& w) { windVel_ = w; }
    void setK1(double k1) { k1_ = k1; }
    void setK2(double k2) { k2_ = k2; }

    void apply(RigidBody& b, double dt) override;

protected:
    // Para heredar (p.ej. torbellino): viento no uniforme
    virtual Vector3D windAt(const Vector3D& pos, const Vector3D& vel) const { (void)vel; return windVel_; }
    bool inside(const Vector3D& p) const;

    Vector3D windVel_{ 0,0,0 };
    double   k1_{ 0.5 };
    double   k2_{ 0.0 };

    Vector3D boxMin_{ -DBL_MAX, -DBL_MAX, -DBL_MAX };
    Vector3D boxMax_{ DBL_MAX,  DBL_MAX,  DBL_MAX };
};
