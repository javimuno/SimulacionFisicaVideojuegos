#ifndef EMITTER_H
#define EMITTER_H

#include "Particle.h"
#include <vector>
#include <random>

class Emitter {
public:
    Vector3D position;
    Vector3D velocity;
    float spawnRate;  // Número de partículas generadas por segundo
    float lifeTime;   // Duración de vida de las partículas
    std::vector<Particle*> particles;  // Contenedor de partículas
    std::vector<Particle*> particlesAux;  // Contenedor de partículas AUX FAIL
    
    float timeSinceLastSpawn;
    float typeEmitter;

    // Constructor
    Emitter(Vector3D pos, Vector3D vel, float rate, float life,float type);

    //destructor
    ~Emitter() {
        for (Particle* p : particles) {
            delete p;
        }
        particles.clear();
    }


    // Método para emitir una nueva partícula
    void EmitParticle();

    //Para aux

    void EmitAuxParticle();

    // Método para actualizar las partículas
    void Update(float deltaTime);

    static std::default_random_engine generator;
    static std::normal_distribution<float> distribution;

    Vector3D generateGaussianDispersion(const Vector3D& baseVelocity);
    Vector3D generateGaussianDispersionFog(const Vector3D& baseVelocity);
    Vector3D generateGaussianDispersionExplosion(const Vector3D& baseVelocity);
    // Declaración del método para generar velocidades aleatorias
    Vector3D generateRandomVelocity(float mean, float stddev);
    Vector3D generateRandomVelocityFog(float mean, float stddev);
    Vector3D generateRandomVelocityExplosion(float mean, float stddev);

private:

    


};

#endif // EMITTER_H

