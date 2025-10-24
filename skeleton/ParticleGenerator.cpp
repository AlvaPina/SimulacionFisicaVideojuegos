#include "ParticleGenerator.h"

ParticleGenerator::ParticleGenerator(int particlesPerSecond, Vector2D spreadAngle, Vector2D orientation)
	: _enabled(true),
	_particlesPerSecond(particlesPerSecond),
	_spreadAngle(spreadAngle),
	_orientation(orientation),
	_accumulatedTime(0.0)
{

}

ParticleGenerator::~ParticleGenerator()
{
	// Liberar memoria de las partículas
	for (Particle* p : _particles)
		delete p;
	_particles.clear();
}

void ParticleGenerator::setActive(bool value)
{
	_enabled = false;
}

void ParticleGenerator::update(double deltaTime)
{
	// Actualizamos particulas
	for (Particle* particle : _particles) {
		if (particle) {
			//std::cout << "integrate";
			particle->integrate(deltaTime);
		}
	}
	// Comprobamos si ha pasado el tiempo necesario para generar particula
	_accumulatedTime += deltaTime;
	if (_accumulatedTime < 1000 / _particlesPerSecond) return;
}

void ParticleGenerator::generateParticle()
{
	//Particle* particle = new Particle();
}
