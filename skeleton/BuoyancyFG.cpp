#include "BuoyancyFG.h"
#include <algorithm>
#include <cmath>

// Gravedad general
static inline float g_mag() { return 9.81f; }

void BuoyancyFG::updateForce(Particle* p, float /*dt*/)
{
    if (!p) return;
    if (p->getMass() <= 0.0f) return;
    if (height_ <= 0.0f || volume_ <= 0.0f || rho_ <= 0.0f) return;

    // Centro del cuerpo y sus extremos en Y
    const float y = p->getPosition().y;
    const float hh = 0.5f * height_;     // mitad
    const float top = y + hh;         // parte superior
    const float bottom = y - hh;         // parte inferior

    // Casos (inmersión parcial)
    float subRatio = 0.0f; // fracción sumergida [0,1]

    if (bottom >= h0_) {
        // totalmente por encima del agua -> 0
        subRatio = 0.0f;
    }
    else if (top <= h0_) {
        // totalmente sumergido -> 1
        subRatio = 1.0f;
    }
    else {
        // parcial: desde bottom hasta h0
        // fracción = (h0 - bottom) / height
        subRatio = (h0_ - bottom) / height_;
        if (subRatio < 0.0f) subRatio = 0.0f;
        if (subRatio > 1.0f) subRatio = 1.0f;
    }

    if (subRatio <= 0.0f) return;

    // Empuje hacia +Y: E = rho * g * V_sub
    const float V_sub = volume_ * subRatio;
    const float E = rho_ * g_mag() * V_sub;

    Vector3D F(0.0f, E, 0.0f);
    p->addForce(F);
}
