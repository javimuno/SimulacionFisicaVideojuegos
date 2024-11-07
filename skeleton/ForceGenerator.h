// ForceGenerator.h
#ifndef FORCEGENERATOR_H
#define FORCEGENERATOR_H

#include "Particle.h"

class ForceGenerator {
public:
    virtual void updateForce(Particle* particle, float deltaTime) = 0;
};

#endif

