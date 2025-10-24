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
						   // Rango desde (0�,0�) hasta (180�,180�)
	Vector2D _orientation; // Direccion a la que sale la particula disparada

	double _accumulatedTime = 0.0; // tiempo acumulado para generar part�culas
	std::vector<Particle*> _particles;

	void generateParticle();
};

