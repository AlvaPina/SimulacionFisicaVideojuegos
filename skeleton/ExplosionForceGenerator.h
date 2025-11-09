#pragma once
#include "ForceGenerator.h"
#include "Vector3D.h"
#include "RigidBody.h"

// Explosión radial: F = (K / r^2) * dir * exp(-t/tau), para r < R(t)
class ExplosionForceGenerator : public ForceGenerator {
public:
    ExplosionForceGenerator(const Vector3D& center, double K, double R0, double tau, double ve = 0.0);

    void setCenter(const Vector3D& c) { center_ = c; }
    void setIntensity(double K) { K_ = K; }
    void setTau(double tau) { tau_ = tau; }
    void setR0(double r0) { R0_ = r0; }
    void setExpansionSpeed(double ve) { ve_ = ve; }

    void resetTime(); // reinicia la explosión

    void apply(RigidBody& b, double dt) override;

private:
    Vector3D center_{ 0,0,0 };
    double   K_{ 1000.0 };    // Intensidad base de la explosión
    double   R0_{ 5.0 };      // Radio inicial de influencia
    double   tau_{ 1.0 };     // Constante de tiempo (tau): controla el decaimiento exponencial e^{-t/tau}. Tau pequeño => se apaga rápido.
    double   ve_{ 0.0 };      // velocidad de expansión del radio (onda de choque)
    double   elapsed_{ 0.0 }; // tiempo acumulado
};

