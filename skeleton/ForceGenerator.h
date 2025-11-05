#pragma once
class RigidBody;
class ForceGenerator {
public:
    virtual ~ForceGenerator() = default;
    void setActive(bool v) { active_ = v; }
    bool isActive() const { return active_; }

    // Aplica fuerza al objeto durante dt (seg)
    virtual void apply(RigidBody& body, double dt) = 0;

protected:
    bool active_ = true;
};

