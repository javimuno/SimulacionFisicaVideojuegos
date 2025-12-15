#include "ForceRegistry.h"
#include "ForceGenerator.h"
#include "Particle.h"
#include "Vector3D.h"

void ForceRegistry::add(Particle* p, ForceGenerator* fg) {
    if (p && fg) regs.push_back({ p, fg });
}

void ForceRegistry::remove(Particle* p, ForceGenerator* fg) {
    for (auto it = regs.begin(); it != regs.end(); ) {
        if (it->p == p && it->fg == fg) it = regs.erase(it);
        else ++it;
    }
}

void ForceRegistry::clear() { regs.clear(); }

void ForceRegistry::updateForces(float dt) {
    // 1) poner acumuladores de fuerza a 0
    for (auto& e : regs) if (e.p) e.p->clearForces();
    // 2) aplicar generadores (suman fuerzas a cada partícula)
    for (auto& e : regs) if (e.p && e.fg) e.fg->updateForce(e.p, dt);
}

void ForceRegistry::removeAll(Particle* p) {
    regs.erase(
        std::remove_if(regs.begin(), regs.end(),
            [p](const Entry& e) { return e.p == p; }),
        regs.end()
    );
}
