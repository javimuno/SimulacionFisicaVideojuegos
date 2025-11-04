#pragma once
#include <vector>

class SimpleEmitter;
class WorldBounds;

class ParticleSystem {
public:
    explicit ParticleSystem(const WorldBounds* world) : world_(world) {}

    void addEmitter(SimpleEmitter* e) { if (e) emitters_.push_back(e); }

    // Actualiza todos los emisores y hace culling con los límites del mundo
    void update(float dt);

private:
    const WorldBounds* world_ = nullptr;
    std::vector<SimpleEmitter*> emitters_;
};


