#include "GravityForceGenerator.h"

// Aplica la gravedad como una fuerza basada en la masa de la partícula
void GravityForceGenerator::updateForce(Particle* particle, float deltaTime) {
    // Verificamos si la partícula tiene masa infinita
    if (particle->getMass() <= 0.0f) return;

    // Calcula la fuerza = gravedad * masa
    Vector3D force = gravity * particle->getMass();
    particle->addForce(force);
}
