// GravityForceGenerator.cpp
#include "GravityForceGenerator.h"

GravityForceGenerator::GravityForceGenerator(const Vector3D& gravity) : gravity(gravity) {}

void GravityForceGenerator::updateForce(Particle* particle, float deltaTime) {
    if (particle->getMass() <= 0) return;  // Evitamos aplicar fuerza en partículas sin masa

    // Fuerza gravitatoria: F = m * g
    Vector3D force = gravity * particle->getMass();
    particle->addForce(force);
}
