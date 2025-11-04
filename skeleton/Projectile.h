#pragma once
#include <cmath>
#include "Particle.h"
#include "Vector3D.h"

enum class ProjectileType { Light, Heavy };

class Projectile : public Particle {
public:
    Projectile(const Vector3D& pos, const Vector3D& vel, const Vector3D& acc,
        float damping, IntegratorType integrator, float mass,
        const physx::PxVec4& color, float radius, ProjectileType type)
        : Particle(pos, vel, acc, damping, integrator, mass, color, radius)
        , type_(type) {
    }

    // Escala velocidad (-> energía cinética -> k^2) y fuerzas acumuladas del frame
    void scale(float k) {
        if (!(k > 0.0f) || !std::isfinite(k)) return;
        setVelocity(getVelocity() * k);
        
    }

    ProjectileType type() const { return type_; }

private:
    ProjectileType type_;
};

