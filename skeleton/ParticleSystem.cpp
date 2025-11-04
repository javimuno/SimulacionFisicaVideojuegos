#include "ParticleSystem.h"
#include "SimpleEmitter.h"
#include "WorldBounds.h"

void ParticleSystem::update(float dt) {
    for (auto* e : emitters_) if (e) e->update(dt);
    if (!world_) return;
    for (auto* e : emitters_) if (e) e->cullOutside(*world_);
}

