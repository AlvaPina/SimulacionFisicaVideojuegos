#pragma once
#include "Particle.h"
#include "Vector2D.h"

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

