#ifndef NOMINMAX
#define NOMINMAX
#endif
#include "SimpleEmitter.h"
#include <random>
#include <cmath>
#include "WorldBounds.h"
#include "ForceRegistry.h"
#include "ForceGenerator.h"
#include "GravityFG.h"
#include "WindFG.h"
#include "WhirlwindFG.h"
#include "ExplosionFG.h"
#include <iostream>



extern ForceRegistry* gForceReg;
extern GravityFG* gGravity;
extern WindFG* gWind;
extern WhirlwindFG* gWhirl;
extern ExplosionFG* gExpl;
extern SimpleEmitter* gEmit3; //el 3 será el gaussiano



using namespace physx;
using namespace std;

static constexpr int   SPAWN_HARD_CAP = 50;     // seguridad: máximo por frame que con mucho peta
static constexpr float GY = -10.0f;
static constexpr float M_PI = 3.14159265358979323846f;

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
    // quitar del ForceRegistry ANTES de borrar
    for (auto& l : alive) {
        if (gForceReg) {
            if (gGravity) gForceReg->remove(l.p, gGravity);
            if (gWind)    gForceReg->remove(l.p, gWind);
            if (gWhirl)   gForceReg->remove(l.p, gWhirl);
            if (gExpl)    gForceReg->remove(l.p, gExpl);
        }
        delete l.p; l.p = nullptr;
    }
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
    // X: uniforme por defecto; normal si se activa en cfg
    float x = cfg.posGaussX
        ? randNormal(cfg.position.x, std::max(0.0001f, cfg.posJitter.x))
        : cfg.position.x + randUniform(-cfg.posJitter.x, +cfg.posJitter.x);

    return {
        x,
        cfg.position.y + randUniform(-cfg.posJitter.y, +cfg.posJitter.y),
        cfg.position.z + randUniform(-cfg.posJitter.z, +cfg.posJitter.z)
    };
}

Vector3D SimpleEmitter::distribution() {
    const float speed = cfg.speed;

    // ----- Uniforme en esfera (por defecto) -----
    auto dirUniformSphere = [&]() -> Vector3D {
        const float u = randUniform(0.f, 1.f);
        const float v = randUniform(0.f, 1.f);
        const float phi = 2.0f * float(M_PI) * u;     // [0, 2pi)
        const float cosT = 2.0f * v - 1.0f;            // uniforme en [-1, 1]
        const float sinT = std::sqrt(std::max(0.f, 1.0f - cosT * cosT));
        return Vector3D(sinT * std::cos(phi),  // x
            cosT,                  // y
            sinT * std::sin(phi)  // z
        );
        };

    // ----- Gauss en cono alrededor de +Y (solo emisor 3) -----
    auto dirGaussianConeY = [&]() -> Vector3D {
        const float sigma = 0.01f;                    // abre/cierra el chorro
        const float theta = std::fabs(randNormal(0.f, sigma)); // desviación angular
        const float phi = 2.0f * float(M_PI) * randUniform(0.f, 1.f);

        const float cosT = std::cos(theta);
        const float sinT = std::sin(theta);
        const float cphi = std::cos(phi);
        const float sphi = std::sin(phi);

        // Eje +Y; base ortonormal trivial U=(1,0,0), V=(0,0,1), W=(0,1,0)
        Vector3D d(cphi * sinT,  // x
            cosT,         // y
            sphi * sinT   // z
        );
        // normaliza por robustez
        const float n = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
        return (n > 1e-6f) ? Vector3D(d.x / n, d.y / n, d.z / n) : Vector3D(0, 1, 0);
        };

    // Por defecto uniforme; si este objeto ES el emisor 3, usa gauss en cono
    extern SimpleEmitter* gEmit3;
    Vector3D dir = (gaussianCone_) ? dirGaussianConeY() : dirUniformSphere();

    // Devuelve velocidad = dirección * módulo
    return dir * speed;
}

void SimpleEmitter::update(float dt) {
    if (dt <= 0.0f) return;

    // integrar y borrar por lifetime
    for (auto it = alive.begin(); it != alive.end(); ) {
        it->p->integrate(dt);
        it->age += dt;
        if (it->age >= cfg.lifetime) {
            // Quitar del ForceRegistry ANTES de borrar
            if (gForceReg) {
                if (gGravity) gForceReg->remove(it->p, gGravity);
                if (gWind)    gForceReg->remove(it->p, gWind);
                if (gWhirl)   gForceReg->remove(it->p, gWhirl);
                if (gExpl)    gForceReg->remove(it->p, gExpl); 
            }
            delete it->p;
            it = alive.erase(it);
        }
        else ++it;
    }

    if (!cfg.active) return;

    // emisor suave
    emit_accum += cfg.rate * dt;

    int room = std::max(0, cfg.maxAlive - (int)alive.size());
    int quota = std::min({ (int)emit_accum, room, SPAWN_HARD_CAP });
    if (quota <= 0) return;
    emit_accum -= quota;

    for (int i = 0; i < quota; ++i) {
        //Posición
        Vector3D pos = samplePosition();

        //sacamos solo la direccion
        Vector3D v0 = distribution();
        float speed = v0.length();
        Vector3D dir = (speed > 1e-6f) ? (v0 * (1.0f / speed)) : Vector3D(0, 1, 0);

        // nace sin velocidad 
        Vector3D vel0(0.0f, 0.0f, 0.0f);
        Vector3D acc0(0.0f, 0.0f, 0.0f);

        Particle* p = new Particle(
            pos, vel0, acc0,
            cfg.damping,
            IntegratorType::EulerSemiImplicit,
            cfg.mass,
            cfg.color,
            cfg.radius
        );

        //  Fuerzas que afectan
        if (gForceReg && gGravity) gForceReg->add(p, gGravity);
        if (gForceReg && gWind)    gForceReg->add(p, gWind);
        if (gForceReg && gWhirl)   gForceReg->add(p, gWhirl);
        if (gForceReg && gExpl)    gForceReg->add(p, gExpl); // si la usas

       //impulso por fuerza (F = m * (cfg.speed / dt) * dir)
        if (dt > 0.0f) {
            float k = p->getMass() * (cfg.speed / dt);
            Vector3D F(dir.x * k, dir.y * k, dir.z * k);
            p->addForce(F);
        }

        //Almacenar en vivos
        alive.push_back({ p, 0.0f });;
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

void SimpleEmitter::registerForceForAlive(ForceRegistry* reg, ForceGenerator* fg) {
    if (!reg || !fg) return;
    for (auto& l : alive) reg->add(l.p, fg);
}


void SimpleEmitter::debugPrintHistogramX(int bins, int samples, float nsigma) {
    if (bins < 5) bins = 5;
    if (samples < 1000) samples = 1000;

    const float mu = cfg.position.x;
    const float sigma = std::max(0.0001f, cfg.posJitter.x);
    const float minX = cfg.posGaussX ? (mu - nsigma * sigma) : (mu - cfg.posJitter.x);
    const float maxX = cfg.posGaussX ? (mu + nsigma * sigma) : (mu + cfg.posJitter.x);
    const float w = (maxX - minX) / bins;

    std::vector<int> h(bins, 0);
    double sum = 0.0, sum2 = 0.0;

    for (int i = 0; i < samples; ++i) {
        float x = cfg.posGaussX ? randNormal(mu, sigma)
            : randUniform(mu - cfg.posJitter.x, mu + cfg.posJitter.x);
        sum += x; sum2 += x * x;
        int b = int((x - minX) / w);
        if (b < 0) b = 0; else if (b >= bins) b = bins - 1;
        h[b]++;
    }

    double mean = sum / samples;
    double var = sum2 / samples - mean * mean;
    double stdv = (var > 0) ? std::sqrt(var) : 0.0;

    std::cout << "\n[Emitter3] X-dist: " << (cfg.posGaussX ? "Normal" : "Uniform")
        << "  mu" << mean << "  sigma_est" << stdv << "  (cfg sigma=" << sigma << ")\n";

    int maxc = 1;
    for (int c : h) if (c > maxc) maxc = c;
    const int barMax = 50;

    for (int i = 0; i < bins; ++i) {
        float xl = minX + i * w, xr = xl + w;
        int bar = int(barMax * (h[i] / float(maxc)));
        std::cout << "[" << i << "] " << xl << " .. " << xr << " | "
            << std::string(bar, '#') << " (" << h[i] << ")\n";
    }
    std::cout.flush();
}