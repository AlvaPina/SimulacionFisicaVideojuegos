#include "ForceRegistry.h"

void ForceRegistry::add(RigidBody* body, ForceGenerator* fg)
{
	pairs_.emplace_back(body, fg);
}

void ForceRegistry::clearFor(RigidBody* body)
{
	pairs_.erase(std::remove_if(pairs_.begin(), pairs_.end(),
		[body](auto& p) { return p.first == body; }), pairs_.end());
}

void ForceRegistry::update(double dt)
{
	
}