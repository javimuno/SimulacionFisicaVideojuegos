#pragma once
#include "ForceGenerator.h"
#include "Vector3D.h"

class GravityForceGenerator : public ForceGenerator {
public:
    GravityForceGenerator(const Vector3D& gravity) : gravity(gravity) {}

    void updateForce(Particle* particle, float deltaTime) override;

private:
    Vector3D gravity;
};
