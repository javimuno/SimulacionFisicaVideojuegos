#include "Emitter.h"
#include <random>


// Inicialización del generador y la distribución
std::default_random_engine Emitter::generator;
std::normal_distribution<float> Emitter::distribution(0.0, 1.0);

// Constructor del emisor
Emitter::Emitter(Vector3D pos, Vector3D vel, float rate, float life,float type)
    : position(pos), velocity(vel), spawnRate(rate), lifeTime(life), timeSinceLastSpawn(0.0f),typeEmitter(type) {}

// Método para emitir una nueva partícula
void Emitter::EmitParticle() {

    //Particle* newpParticle = nullptr;
    if (typeEmitter == 1) {
        Particle* newParticle = new Particle(position, velocity, generateGaussianDispersion(0.1f), 0.99f);
        newParticle->SetLifeTime(lifeTime);
        particles.push_back(newParticle);
    }
    else if(typeEmitter==2)
    {
        Particle* newParticle = new Particle(position, velocity, generateGaussianDispersionFog(2.0f), 0.99f);
        newParticle->SetLifeTime(lifeTime);
        particles.push_back(newParticle);
    }
    else if (typeEmitter==3)
    {
        Particle* newParticle = new Particle(position, generateRandomVelocityExplosion(2.0f,1.0f), generateGaussianDispersionExplosion(5.0f), 0.99f);
        newParticle->SetLifeTime(lifeTime);
        particles.push_back(newParticle);
    }
    

    //generateRandomVelocity(10.0f,1.0f)
}

void Emitter::EmitAuxParticle() {
    Particle* newParticle = new Particle(position, generateRandomVelocityExplosion(10.0f, 1.0f), generateGaussianDispersionExplosion(1.0f), 0.99f);
    newParticle->SetLifeTime(lifeTime);
    particlesAux.push_back(newParticle);
}

// Método para actualizar las partículas
void Emitter::Update(float deltaTime) {
    // Emitir partículas dependiendo del spawnRate
    timeSinceLastSpawn += deltaTime;
    if (timeSinceLastSpawn >= 1.0f / spawnRate) {
        EmitParticle();
        timeSinceLastSpawn = 0.0f;
    }

    // Actualizar todas las partículas A POSTERIORI DE MUERTE  activas FAIL
    for (auto it = particlesAux.begin(); it != particlesAux.end(); ) {
        (*it)->integrate(deltaTime);




        // Verificar si la partícula ha excedido su tiempo de vida
        if ((*it)->IsDead()) {
            
            delete (*it);  // Liberar memoria de la partícula
            it = particlesAux.erase(it);  // Eliminarla de la lista
        }
        else {

            ++it;
        }
    }

    // Actualizar todas las partículas activas
    for (auto it = particles.begin(); it != particles.end(); ) {
        (*it)->integrate(deltaTime);

       
        

        // Verificar si la partícula ha excedido su tiempo de vida
        if ((*it)->IsDead()) {
            //EmitAuxParticle(); //idea para los fuegos FAIL
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
}Vector3D Emitter::generateGaussianDispersionFog(const Vector3D& baseVelocity) {
    Vector3D dispersion(
        baseVelocity.x + distribution(generator) * 2.0f,
        baseVelocity.y + distribution(generator) * 0.01f,
        baseVelocity.z + distribution(generator) * 2.0f
    );
    return dispersion;
}Vector3D Emitter::generateGaussianDispersionExplosion(const Vector3D& baseVelocity) {
    Vector3D dispersion(
        baseVelocity.x + distribution(generator) * 10.0f,
        baseVelocity.y + distribution(generator) * 10.1f,
        baseVelocity.z + distribution(generator) * 10.0f
    );
    return dispersion;
}


// Método para generar una velocidad aleatoria con distribución Gaussiana
Vector3D Emitter::generateRandomVelocity(float mean, float stddev) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> d(mean, stddev);

    float vx = -d(gen)*0.5;
    float vy = d(gen)*0.5;
    float vz = -d(gen)*0.5;

    return Vector3D(vx, vy, vz);


}Vector3D Emitter::generateRandomVelocityFog(float mean, float stddev) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> d(mean, stddev);

    float vx = d(gen)*0.01;
    float vy = d(gen)*0.01;
    float vz = d(gen)*0.01;

    return Vector3D(vx, vy, vz);


}Vector3D Emitter::generateRandomVelocityExplosion(float mean, float stddev) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> d(mean, stddev);

    float vx = d(gen)*0.2;
    float vy = d(gen)*0.2;
    float vz = d(gen)*0.2;

    return Vector3D(vx, vy, vz);
}
