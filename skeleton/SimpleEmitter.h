#pragma once
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <vector>
#include <random>
#include <algorithm>
#include "Vector3D.h"
#include "Particle.h"

struct SimpleEmitterConfig {
    Vector3D position{ 0,0,0 };
    Vector3D posJitter{ 0,0,0 };     // +/- por eje
    float    speed = 5.0f;          // velocidad escalar (isotrópica)
    float    lifetime = 2.0f;       // s
    float    damping = 0.99f;
    physx::PxVec4 color{ 0.2f,0.6f,1.0f,1.0f };
    float    radius = 0.12f;
    float    rate = 6.0f;       // partículas por segundo
    int      maxAlive = 300;        // cap por emisor
    bool     active = false;      // inicia apagado
    float mass = 1.0f;
};

class ForceRegistry;
class ForceGenerator;

class SimpleEmitter {
public:
    explicit SimpleEmitter(const SimpleEmitterConfig& cfg);

    void setActive(bool on);
    bool isActive() const { return cfg.active; }
    void changeRate(float delta);            // +/- por segundo
    void update(float dt);                   // integra, borra muertas y emite
    void clear();                            // borra TODAS sus partículas
    void cullOutside(const class WorldBounds& world);
    size_t aliveCount() const { return alive.size(); }
    
    void registerForceForAlive(ForceRegistry* reg, ForceGenerator* fg);

private:
    struct Live {
        Particle* p = nullptr;
        float age = 0.0f;
    };

    SimpleEmitterConfig cfg;
    std::mt19937 rng;
    float emit_accum = 0.0f;                 // acumulador rate*dt
    std::vector<Live> alive;

    // random helpers
    float randUniform(float a, float b);
    float randNormal(float mean, float sigma);

    Vector3D samplePosition();
    Vector3D sampleVelocityIsotropic();      // dirección aleatoria en esfera * speed
};
