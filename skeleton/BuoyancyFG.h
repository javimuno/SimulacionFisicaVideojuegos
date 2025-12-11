#pragma once
#include "ForceGenerator.h"
#include "Vector3D.h"
#include "Particle.h"

// Buoyancy de icebergs y boyas
// - h0: nivel del agua (y del plano del agua)
// - height: altura efectiva del cuerpo (para calcular inmersión parcial)
// - volume: volumen total del cuerpo
// - rho: densidad del líquido (agua ~1000 kg/m3)
// Empuje: E = rho * g * V_sub, vertical +Y


class BuoyancyFG : public ForceGenerator {
public:
    BuoyancyFG(float h0, float height, float volume, float rho = 1000.0f)
        : h0_(h0), height_(height), volume_(volume), rho_(rho) {
    }

    void updateForce(Particle* p, float dt) override;

    // setters (versión antigua)
    void setLiquidLevel(float h0) { h0_ = h0; }
    void setHeight(float h) { height_ = (h < 0.f ? 0.f : h); }
    void setVolume(float v) { volume_ = (v < 0.f ? 0.f : v); }
    void setDensity(float rho) { rho_ = (rho < 0.f ? 0.f : rho); }

private:
    float h0_;      // nivel del agua (y)
    float height_;  // altura del "cubo" (para inmersión)
    float volume_;  // volumen total
    float rho_;     // densidad del líquido
};
