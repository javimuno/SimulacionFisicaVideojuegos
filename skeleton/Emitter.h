#ifndef EMITTER_H
#define EMITTER_H

#include "Particle.h"
#include <vector>
#include <random>

class Emitter {
public:
    Vector3D position;
    Vector3D velocity;
    float spawnRate;  // N�mero de part�culas generadas por segundo
    float lifeTime;   // Duraci�n de vida de las part�culas
    std::vector<Particle*> particles;  // Contenedor de part�culas
    float timeSinceLastSpawn;

    // Constructor
    Emitter(Vector3D pos, Vector3D vel, float rate, float life);

    // M�todo para emitir una nueva part�cula
    void EmitParticle();

    // M�todo para actualizar las part�culas
    void Update(float deltaTime);

    static std::default_random_engine generator;
    static std::normal_distribution<float> distribution;

    Vector3D generateGaussianDispersion(const Vector3D& baseVelocity);
    // Declaraci�n del m�todo para generar velocidades aleatorias
    Vector3D generateRandomVelocity(float mean, float stddev);

private:

    


};

#endif // EMITTER_H
