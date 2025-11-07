#include "ForceRegistry.h"
#include "RigidBody.h"
#include "ForceGenerator.h"

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
	for (auto& pr : pairs_) {
		RigidBody* body = pr.first;
		ForceGenerator* fg = pr.second;
		if (body && fg && fg->isActive()) fg->apply(*body, dt);
	}
}