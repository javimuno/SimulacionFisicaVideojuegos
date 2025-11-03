#include "ExplosionFG.h"
#include "Particle.h"
#include <cmath>

void ExplosionFG::updateForce(Particle* p, float /*dt*/) {
    if (!active_ || !p) return;

    Vector3D r = p->getPosition() - c_;
    float d = r.length();
    if (d <= 1e-5f || d >= R_) return;

    Vector3D dir = r * (1.0f / d);
    float decay = std::exp(-age_ / tau_);
    float mag = (K_ / (d * d)) * decay;

    p->addForce(dir * mag);
}
