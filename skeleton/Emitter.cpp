#include "Emitter.h"
#include <random>


// Inicializaci�n del generador y la distribuci�n
std::default_random_engine Emitter::generator;
std::normal_distribution<float> Emitter::distribution(0.0, 1.0);

// Constructor del emisor
Emitter::Emitter(Vector3D pos, Vector3D vel, float rate, float life)
    : position(pos), velocity(vel), spawnRate(rate), lifeTime(life), timeSinceLastSpawn(0.0f) {}

// M�todo para emitir una nueva part�cula
void Emitter::EmitParticle() {
    Particle* newParticle = new Particle(position, generateRandomVelocity(10.0f, 1.0f), generateGaussianDispersion(0.1f), 0.99f);
    newParticle->SetLifeTime(lifeTime);
    particles.push_back(newParticle);

    //generateRandomVelocity(10.0f,1.0f)
}

// M�todo para actualizar las part�culas
void Emitter::Update(float deltaTime) {
    // Emitir part�culas dependiendo del spawnRate
    timeSinceLastSpawn += deltaTime;
    if (timeSinceLastSpawn >= 1.0f / spawnRate) {
        EmitParticle();
        timeSinceLastSpawn = 0.0f;
    }

    // Actualizar todas las part�culas activas
    for (auto it = particles.begin(); it != particles.end(); ) {
        (*it)->integrate(deltaTime);

        // Verificar si la part�cula ha excedido su tiempo de vida
        if ((*it)->IsDead()) {
            delete (*it);  // Liberar memoria de la part�cula
            it = particles.erase(it);  // Eliminarla de la lista
        }
        else {
            ++it;
        }
    }
}

Vector3D Emitter::generateGaussianDispersion(const Vector3D& baseVelocity) {
    Vector3D dispersion(
        baseVelocity.x + distribution(generator) * 0.1f,
        baseVelocity.y + distribution(generator) * 5.0f,
        baseVelocity.z + distribution(generator) * 0.1f
    );
    return dispersion;
}

// M�todo para generar una velocidad aleatoria con distribuci�n Gaussiana
Vector3D Emitter::generateRandomVelocity(float mean, float stddev) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> d(mean, stddev);

    float vx = 0.0f;
    float vy = d(gen);
    float vz = 0.0f;

    return Vector3D(vx, vy, vz);
}