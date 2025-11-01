#ifndef NOMINMAX
#define NOMINMAX
#endif
#include "SimpleEmitter.h"
#include <cmath>
#include "WorldBounds.h"

using namespace physx;

static constexpr int   SPAWN_HARD_CAP = 24;     // seguridad: máximo por frame
static constexpr float GY = -10.0f;

SimpleEmitter::SimpleEmitter(const SimpleEmitterConfig& c)
    : cfg(c), rng(std::random_device{}()) {
}

void SimpleEmitter::setActive(bool on) {
    if (cfg.active == on) return;
    cfg.active = on;
    if (!on) clear();               // al apagar, limpiar lo suyo
}

void SimpleEmitter::changeRate(float delta) {
    cfg.rate = std::max(0.0f, cfg.rate + delta);
}

void SimpleEmitter::clear() {
    for (auto& l : alive) { delete l.p; l.p = nullptr; };
    alive.clear();
    emit_accum = 0.0f;
}

float SimpleEmitter::randUniform(float a, float b) {
    std::uniform_real_distribution<float> d(a, b);
    return d(rng);
}

float SimpleEmitter::randNormal(float mean, float sigma) {
    std::normal_distribution<float> d(mean, sigma);
    return d(rng);
}

Vector3D SimpleEmitter::samplePosition() {
    return {
        cfg.position.x + randUniform(-cfg.posJitter.x, +cfg.posJitter.x),
        cfg.position.y + randUniform(-cfg.posJitter.y, +cfg.posJitter.y),
        cfg.position.z + randUniform(-cfg.posJitter.z, +cfg.posJitter.z)
    };
}

Vector3D SimpleEmitter::sampleVelocityIsotropic() {
    // muestrea una dirección aleatoria (gaussiana) y normaliza
    Vector3D v(randNormal(0.f, 1.f), randNormal(0.f, 1.f), randNormal(0.f, 1.f));
    float n = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (n < 1e-6f) v = { 0,1,0 };
    else { v.x /= n; v.y /= n; v.z /= n; }
    // pequeño sesgo hacia arriba para no formar “discos” planos
    v.y = std::abs(v.y);
    // escala por speed
    v = v * cfg.speed;
    return v;
}

void SimpleEmitter::update(float dt) {
    if (dt <= 0.0f) return;

    // 1) integrar y borrar por lifetime
    for (auto it = alive.begin(); it != alive.end(); ) {
        it->p->integrate(dt);
        it->age += dt;
        if (it->age >= cfg.lifetime) {
            delete it->p;
            it = alive.erase(it);
        }
        else ++it;
    }

    if (!cfg.active) return;

    // 2) emitir suave por rate (pps) con acumulador
    emit_accum += cfg.rate * dt;

    int room = std::max(0, cfg.maxAlive - (int)alive.size());
    int quota = std::min({ (int)emit_accum, room, SPAWN_HARD_CAP });
    if (quota <= 0) return;
    emit_accum -= quota;

    for (int i = 0; i < quota; ++i) {
        Vector3D pos = samplePosition();
        Vector3D vel = sampleVelocityIsotropic();
        Vector3D acc(0.0f, GY, 0.0f);

        Particle* p = new Particle(pos, vel, acc, cfg.damping,
            IntegratorType::EulerSemiImplicit,
            1.0f, cfg.color, cfg.radius);
        alive.push_back({ p, 0.0f });
    }
}

void SimpleEmitter::cullOutside(const WorldBounds& world) {
    for (auto it = alive.begin(); it != alive.end(); ) {
        Particle* p = it->p;
        const Vector3D& pos = p->getPosition();
        if (!world.isFinite(pos) || world.isOutside(pos)) {
            delete p;              // ~Particle() -> DeregisterRenderItem(...)
            it = alive.erase(it);
        }
        else {
            ++it;
        }
    }
}
