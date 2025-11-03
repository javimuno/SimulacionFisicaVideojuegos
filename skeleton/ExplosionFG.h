#pragma once
#include "ForceGenerator.h"
#include "Vector3D.h"

class Particle;

class ExplosionFG : public ForceGenerator {
public:
    ExplosionFG(const Vector3D& center, float radius, float K, float tau)
        : c_(center), R_(radius), K_(K), tau_(tau) {
    }

    void updateForce(Particle* p, float dt) override;

    // Control
    void setCenter(const Vector3D& c) { c_ = c; }
    void setParams(float radius, float K, float tau) { R_ = radius; K_ = K; tau_ = tau; }

    void trigger() { age_ = 0.0f; active_ = true; }
    void advance(float dt) { if (active_) { age_ += dt; if (age_ > 4.0f * tau_) active_ = false; } }
    bool isActive() const { return active_; }

private:
    Vector3D c_;
    float R_ = 5.0f;
    float K_ = 8000.0f;
    float tau_ = 0.7f;

    float age_ = 0.0f;
    bool active_ = false;
};
