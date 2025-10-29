#pragma once
#include "Vector2D.h"
#include <vector>

class Particle;

enum GeneratorType {
	SMOKE,
	FIRE,
};

class ParticleGenerator
{
public:
	ParticleGenerator(int particlesPerSecond, Vector2D spreadAngle, Vector2D orientation);
	~ParticleGenerator();

	void setActive(bool value);
	void update(double deltaTime);

	void setAverageSpeed(double v) { averageSpeed_ = v;  speedNormal_ = std::normal_distribution<double>(averageSpeed_, gaussianFactor_); }
	void setGaussianFactor(double s) { gaussianFactor_ = s; speedNormal_ = std::normal_distribution<double>(averageSpeed_, gaussianFactor_); }
	void setLifeTime(double seconds) { lifeTime_ = seconds; }
	void setSpawnPosition(const Vector3D& p) { spawnPos_ = p; }
	void setEmitRate(int perSec) { emitRate_ = perSec > 0 ? perSec : 1; }
	void clear(); // elimina todas las partículas del generador
private:
	bool _enabled;
	int _particlesPerSecond;
	Vector2D _spreadAngle; // Rango de angulos posibles con los que sale la particula disparada.
						   // Rango desde (0º,0º) hasta (180º,180º)
	Vector2D _orientation; // Direccion a la que sale la particula disparada

	double _accumulatedTime = 0.0; // tiempo acumulado para generar partículas
	std::vector<Particle*> _particles;

	void generateParticle();
};

