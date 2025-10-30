#pragma once
#include <vector>
#include <random>
#include "Vector2D.h"
#include "Vector3D.h"

class Particle;

enum GeneratorType {
    SMOKE,
    FIRE,
};

class ParticleGenerator
{
public:
    // particlesPerSecond: cuántas por segundo
    // spreadAngle: (yawDegRange, pitchDegRange) en grados
    // orientation: dirección base (3D, se normaliza internamente)
    // spawnPos: posición del emisor
    ParticleGenerator(int particlesPerSecond, Vector2D spreadAngle, Vector3D orientation, Vector3D spawnPos);
    ~ParticleGenerator();

    void setActive(bool value);
    void update(double deltaTime); // segundos

    // setters sencillos
    void setAverageSpeed(double v) { averageSpeed_ = v;  speedNormal_ = std::normal_distribution<double>(averageSpeed_, gaussianFactor_); }
    void setGaussianFactor(double s) { gaussianFactor_ = s; speedNormal_ = std::normal_distribution<double>(averageSpeed_, gaussianFactor_); }
    void setLifeTime(double seconds) { lifeTime_ = seconds; }
    void setSpawnPosition(const Vector3D& p) { spawnPos_ = p; }
    void setEmitRate(int perSec) { emitRate_ = perSec > 0 ? perSec : 1; }
    void clear();

private:
    // Estado y parámetros
    bool      isActive_ = true;
    int       emitRate_ = 10; // partículas/seg
    Vector2D  spreadAngle_; // (yaw,pitch) en grados (rangos)
    Vector3D  facingDir_; // dirección base 3D
    double    elapsed_ = 0.0; // tiempo acumulado (s)

    // Partículas propias del generador
    std::vector<Particle*> particles_;
    std::vector<double>    lifeLeft_; // vida restante (s)

    // Parámetros de emisión
    double    averageSpeed_ = 10.0; // m/s
    double    gaussianFactor_ = 2.0; // sigma velocidad (m/s)
    double    lifeTime_ = 5.0; // s
    Vector3D  spawnPos_ = Vector3D(0, 0, 0);

    // RNG
    std::mt19937 rng_;
    std::normal_distribution<double>      speedNormal_;
    std::uniform_real_distribution<double> uniform01_{ 0.0, 1.0 }; // ∈[0,1)

    // Utilidades
    void emitOne();
    Vector3D randomDirectionWithSpread();
    static double deg2rad(double d) { return d * 3.14159265358979323846 / 180.0; }
    static void dirToYawPitchDeg(const Vector3D& d, double& yawDeg, double& pitchDeg);
    static Vector3D dirFromYawPitchDeg(double yawDeg, double pitchDeg);
};
