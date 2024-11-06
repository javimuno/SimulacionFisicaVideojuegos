#pragma once
#include <vector>
#include "Particle.h"
#include "ForceGenerator.h"

class ForceRegistry {
public:
    void add(Particle* particle, ForceGenerator* fg);
    void remove(Particle* particle, ForceGenerator* fg);
    void clear();
    void updateForces(float deltaTime);

private:
    struct ForceRegistration {
        Particle* particle;
        ForceGenerator* fg;
    };
    std::vector<ForceRegistration> registrations;
};

