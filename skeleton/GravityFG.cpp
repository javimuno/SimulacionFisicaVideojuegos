#include "GravityFG.h"
#include "Particle.h"

void GravityFG::updateForce(Particle* p, float /*dt*/) { //para evitar el warning
    if (!p) return;
    // F = m * g   -> a = F/m
    p->addForce(p->getMass() * g_);
}
