#include "WindFG.h"
#include "Particle.h"

void WindFG::updateForce(Particle* p, float /*dt*/) {
    if (!p) return;

    // v_rel = v_viento - v_particula
    Vector3D vRel = (vWind_ - p->getVelocity());

    // F = k1 * vRel + k2 * |vRel| * vRel   (enunciado P3)
    // (Es FUERZA, no aceleración)
    float speed = vRel.length();
    Vector3D F = vRel * k1_;
    if (k2_ != 0.0f && speed > 0.0f) F += vRel * (k2_ * speed);

    p->addForce(F);
}
