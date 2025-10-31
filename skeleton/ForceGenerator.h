#pragma once
#include "Vector3D.h"

class ForceGenerator
{
public:
	ForceGenerator();
	~ForceGenerator();
	void setActive(bool value) { isActive_ = value; }
private:
	bool isActive_;
	bool isZoneBased_;
	Vector3D zone_;

	// Fuerzas
	virtual void applyForce(class Particle* p, double dt) = 0; // a futuro en vez de la particula se le puede pasar un GameObject
};

