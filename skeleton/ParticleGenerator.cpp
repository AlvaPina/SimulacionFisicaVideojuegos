#include "ParticleGenerator.h"
#include "Particle.h"
#include <algorithm>
#include <cmath>
#include <chrono>

ParticleGenerator::ParticleGenerator(int particlesPerSecond, Vector2D spreadAngle, Vector3D orientation, Vector3D spawnPos)
    : isActive_(true)
    , emitRate_(particlesPerSecond > 0 ? particlesPerSecond : 1)
    , spreadAngle_(spreadAngle)
    , facingDir_(orientation.normalized())
    , elapsed_(0.0)
    , rng_(static_cast<unsigned>(std::chrono::high_resolution_clock::now().time_since_epoch().count()))
    , speedNormal_(averageSpeed_, gaussianFactor_)
    , spawnPos_(spawnPos)
{
}

ParticleGenerator::~ParticleGenerator()
{
    clear();
}

void ParticleGenerator::clear()
{
    for (auto* p : particles_) delete p;
    particles_.clear();
    lifeLeft_.clear();
}

void ParticleGenerator::setActive(bool value)
{
    isActive_ = value;
}

void ParticleGenerator::update(double dtSeconds)
{
    if (!isActive_) return;

    // 1) Integrar + limpiar expiradas
    for (size_t i = 0; i < particles_.size(); /* manual */) {
        Particle* p = particles_[i];
        lifeLeft_[i] -= dtSeconds;

        if (p) p->integrate(dtSeconds);

        if (lifeLeft_[i] <= 0.0) {
            delete p;
            particles_[i] = particles_.back();
            lifeLeft_[i] = lifeLeft_.back();
            particles_.pop_back();
            lifeLeft_.pop_back();
        }
        else {
            ++i;
        }
    }

    // 2) Emisión a tasa fija
    elapsed_ += dtSeconds;
    const double period = 1.0 / static_cast<double>(emitRate_);

    while (elapsed_ >= period) {
        emitOne();
        elapsed_ -= period;
    }
}

// Dirección 3D a partir de facingDir_ con jitter independiente en yaw/pitch
Vector3D ParticleGenerator::randomDirectionWithSpread()
{
    // 1) Pasar dir base a yaw/pitch
    double baseYawDeg = 0.0, basePitchDeg = 0.0;
    dirToYawPitchDeg(facingDir_, baseYawDeg, basePitchDeg);

    // 2) Jitter uniforme en [-spread/2, +spread/2] por eje
    const double halfYaw = spreadAngle_.getX() * 0.5;
    const double halfPitch = spreadAngle_.getY() * 0.5;

    const double yawJitter = (uniform01_(rng_) * 2.0 - 1.0) * halfYaw;
    const double pitchJitter = (uniform01_(rng_) * 2.0 - 1.0) * halfPitch;

    const double yawDeg = baseYawDeg + yawJitter;
    const double pitchDeg = basePitchDeg + pitchJitter;

    // 3) Reconstruir dir y normalizar (usa métodos de Vector3D)
    return dirFromYawPitchDeg(yawDeg, pitchDeg).normalized();
}

void ParticleGenerator::emitOne()
{
    // Dirección aleatoria con spread (no cono perfecto)
    Vector3D dir = randomDirectionWithSpread();   // ya viene normalizada

    // Velocidad gaussiana (>= 0)
    const double speed = std::max<double>(0.0, speedNormal_(rng_));

    // Usamos método de Vector3D
    Vector3D vel = dir.scalarMul(speed);

    // Crear partícula
    Particle* p = new Particle(spawnPos_, vel, /*mass*/1.0);
    p->setGravity(true);

    particles_.push_back(p);
    lifeLeft_.push_back(lifeTime_);
}

// ---------- METODOS AXULIARES Conversión dir <-> yaw/pitch (grados) ----------

// dir -> (yaw,pitch) en grados usando métodos de Vector3D
void ParticleGenerator::dirToYawPitchDeg(const Vector3D& d, double& yawDeg, double& pitchDeg)
{
    // Normalizamos para robustez (por si acaso)
    Vector3D n = d.normalized();
    const double x = n.getX();
    const double y = n.getY();
    const double z = n.getZ();

    const double yawRad = std::atan2(x, z);               // yaw en plano XZ
    const double pitchRad = std::asin(std::max<double>(-1.0, std::min<double>(1.0, y))); // pitch por componente Y

    yawDeg = yawRad * 180.0 / 3.14159265358979323846;
    pitchDeg = pitchRad * 180.0 / 3.14159265358979323846;
}

// (yaw,pitch) -> dir 3D unitario
Vector3D ParticleGenerator::dirFromYawPitchDeg(double yawDeg, double pitchDeg)
{
    const double yaw = deg2rad(yawDeg);
    const double pitch = deg2rad(pitchDeg);

    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);

    // x = sin(yaw)*cos(pitch), y = sin(pitch), z = cos(yaw)*cos(pitch)
    Vector3D dir(sy * cp, sp, cy * cp);
    return dir.normalized(); // garantizamos unitario
}
