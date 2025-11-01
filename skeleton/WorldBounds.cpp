#include "WorldBounds.h"
#include "Particle.h"
#include <cmath>

WorldBounds::WorldBounds(const Vector3D& minB, const Vector3D& maxB)
    : minB_(minB), maxB_(maxB) {
}

void WorldBounds::set(const Vector3D& minB, const Vector3D& maxB) {
    minB_ = minB; maxB_ = maxB;
}

bool WorldBounds::isFinite(const Vector3D& p) const {
    auto finite = [](float x) { return std::isfinite(x); };
    return finite(p.x) && finite(p.y) && finite(p.z);
}

bool WorldBounds::isOutside(const Vector3D& p) const {
    return (p.x < minB_.x || p.x > maxB_.x ||
        p.y < minB_.y || p.y > maxB_.y ||
        p.z < minB_.z || p.z > maxB_.z);
}

bool WorldBounds::removeOutside(Particle*& p) const {
    if (!p) return false;
    const Vector3D& pos = p->getPosition();
    if (!isFinite(pos) || isOutside(pos)) {
        delete p;        // ~Particle SUPUESTAMENTE hará DeregisterRenderItem(...)
        p = nullptr;
        return true;
    }
    return false;
}

void WorldBounds::removeOutside(std::vector<Particle*>& v) const {
    for (auto it = v.begin(); it != v.end(); ) {
        Particle* p = *it;
        if (removeOutside(p)) {
            it = v.erase(it);
        }
        else {
            ++it;
        }
    }
}
