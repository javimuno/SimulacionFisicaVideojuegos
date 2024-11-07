// GravityForceGenerator.h
#ifndef GRAVITYFORCEGENERATOR_H
#define GRAVITYFORCEGENERATOR_H

#include "ForceGenerator.h"
#include "Vector3D.h"

class GravityForceGenerator : public ForceGenerator {
public:
    GravityForceGenerator(const Vector3D& gravity);

    void updateForce(Particle* particle, float deltaTime) override;

private:
    Vector3D gravity;  // Vector de aceleración gravitatoria
};

#endif

