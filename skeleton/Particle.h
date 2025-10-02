#pragma once
#include "RenderUtils.hpp"
#include "core.hpp"
#include "Vector3D.h"


enum class IntegratorType { EulerExplicit, EulerSemiImplicit };

class Particle
{
public:
    // Constructora (velocidad constante): acc=0, damping=1
    Particle(const Vector3D& pos, const Vector3D& vel,
        const Vector3D& acc = Vector3D(0, 0, 0), float damping = 1.0f,
        IntegratorType integrator = IntegratorType::EulerSemiImplicit);

    ~Particle();

    // Llamar cada frame con dt en segundos (usa el dt real del engine)
    void integrate(float dt);

    // Setters útiles para Actividad 2 y 3
    void setAcceleration(const Vector3D& a) { acc = a; }
    void setDamping(float d) { damping = d; } // 0..1

    // (Opcional) cambia integrador en runtime
    void setIntegrator(IntegratorType t) { integrator = t; }

private:
    Vector3D pos;       // posición
    Vector3D vel;       // velocidad
    Vector3D acc;       // aceleración
    float    damping;   // 0..1 ; v <- v * d^dt

    IntegratorType integrator;

    // Render
    physx::PxTransform pose;
    RenderItem* renderItem;

    // Helpers
    void integrateEulerExplicit(float dt);
    void integrateEulerSemiImplicit(float dt);
    void applyDamping(float dt);
    void syncPoseToRender();
};

