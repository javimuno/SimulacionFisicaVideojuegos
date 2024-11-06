#pragma once
#include "Particle.h"

class ForceGenerator {
public:
    virtual ~ForceGenerator() = default;

    // Aplica una fuerza a la partícula (definido en las clases derivadas)
    virtual void updateForce(Particle* particle, float deltaTime) = 0;
};

