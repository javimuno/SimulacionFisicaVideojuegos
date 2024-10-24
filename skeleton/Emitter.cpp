#include "Emitter.h"
#include <random>


// Inicialización del generador y la distribución
std::default_random_engine Emitter::generator;
std::normal_distribution<float> Emitter::distribution(0.0, 1.0);

// Constructor del emisor
Emitter::Emitter(Vector3D pos, Vector3D vel, float rate, float life)
    : position(pos), velocity(vel), spawnRate(rate), lifeTime(life), timeSinceLastSpawn(0.0f) {}

// Método para emitir una nueva partícula
void Emitter::EmitParticle() {
    Particle* newParticle = new Particle(position, generateRandomVelocity(20.0f, 1.0f), generateGaussianDispersion(0.1f), 0.99f);
    newParticle->SetLifeTime(lifeTime);
    particles.push_back(newParticle);

    //generateRandomVelocity(10.0f,1.0f)
}

// Método para actualizar las partículas
void Emitter::Update(float deltaTime) {
    // Emitir partículas dependiendo del spawnRate
    timeSinceLastSpawn += deltaTime;
    if (timeSinceLastSpawn >= 1.0f / spawnRate) {
        EmitParticle();
        timeSinceLastSpawn = 0.0f;
    }

    // Actualizar todas las partículas activas
    for (auto it = particles.begin(); it != particles.end(); ) {
        (*it)->integrate(deltaTime);
        

        // Verificar si la partícula ha excedido su tiempo de vida
        if ((*it)->IsDead()) {
            delete (*it);  // Liberar memoria de la partícula
            it = particles.erase(it);  // Eliminarla de la lista
        }
        else {
            ++it;
        }
    }
}


//generador Gaussiano
Vector3D Emitter::generateGaussianDispersion(const Vector3D& baseVelocity) {
    Vector3D dispersion(
        baseVelocity.x + distribution(generator) * 0.5f,
        baseVelocity.y + distribution(generator) * 0.05f,
        baseVelocity.z + distribution(generator) * 0.5f
    );
    return dispersion;
}


// Método para generar una velocidad aleatoria con distribución Gaussiana
Vector3D Emitter::generateRandomVelocity(float mean, float stddev) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> d(mean, stddev);

    float vx = -d(gen);
    float vy = d(gen);
    float vz = -d(gen);

    return Vector3D(vx, vy, vz);
}
