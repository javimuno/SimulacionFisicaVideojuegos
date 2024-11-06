#include "ForceRegistry.h"

void ForceRegistry::add(Particle* particle, ForceGenerator* fg) {
    registrations.push_back({ particle, fg });
}

void ForceRegistry::remove(Particle* particle, ForceGenerator* fg) {
    registrations.erase(std::remove_if(registrations.begin(), registrations.end(),
        [particle, fg](const ForceRegistration& reg) { return reg.particle == particle && reg.fg == fg; }),
        registrations.end());
}

void ForceRegistry::clear() {
    registrations.clear();
}

void ForceRegistry::updateForces(float deltaTime) {
    for (auto& reg : registrations) {
        reg.fg->updateForce(reg.particle, deltaTime);
    }
}
