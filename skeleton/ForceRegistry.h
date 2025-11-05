#pragma once
#include <vector>
#include <utility>
#include <memory>

class RigidBody;
class ForceGenerator;

class ForceRegistry {
public:
    void add(RigidBody* body, ForceGenerator* fg);
    void clearFor(RigidBody* body);
    void update(double dt);
private:
    std::vector<std::pair<RigidBody*, ForceGenerator*>> pairs_;
};
