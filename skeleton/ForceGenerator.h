#pragma once
class Particle;

class ForceGenerator {
public:
    virtual ~ForceGenerator() {}
    virtual void updateForce(Particle* p, float dt) = 0;
};
