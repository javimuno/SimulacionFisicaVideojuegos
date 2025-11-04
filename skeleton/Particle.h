#pragma once
#include "core.hpp"
#include "RenderUtils.hpp"
#include "Vector3D.h"


enum class IntegratorType { EulerExplicit, EulerSemiImplicit };

class Particle
{
public:
    // Constructora (velocidad constante): acc=0, damping=1
    Particle(const Vector3D& pos, const Vector3D& vel,
        const Vector3D& acc = Vector3D(0, 0, 0),
        float damping = 1.0f,
        IntegratorType integrator = IntegratorType::EulerSemiImplicit,
        float mass = 1.0f,
        physx::PxVec4 color = { 1.0f, 0.8f, 0.2f, 1.0f },
        float radius = 1.0f);

    ~Particle();

    // Llamar cada frame con dt en segundos (usa el dt real del engine)
    void integrate(float dt);

    // Setters útiles para Actividad 2 y 3
    void setAcceleration(const Vector3D& a) { acc = a; }
    void setDamping(float d) { damping = d; } // 0..1

    // (Opcional) cambia integrador en runtime
    void setIntegrator(IntegratorType t) { integrator = t; }

    // getter y setter de masa
    float getMass() const { return mass; }
    void setMass(float m) { mass = m; }

    // Para comprobar límites
    const Vector3D& getPosition() const { return pos; }

    //--FUERZAS--
    void addForce(const Vector3D& f) { forceAccum += f; }
    void clearForces() { forceAccum = Vector3D(0, 0, 0); }

    //viento (de momento)
    const Vector3D& getVelocity() const { return vel; }

    // Setter para permitir escalado de velocidad desde Projectile
    void setVelocity(const Vector3D& v) { vel = v; }

#if ROMPIAXFANTASMA
    // Evita copias/moves accidentales que causarían doble deregistro
    Particle(const Particle&) = delete;
    Particle& operator=(const Particle&) = delete;
    Particle(Particle&&) = delete;
    Particle& operator=(Particle&&) = delete;

    void setOrigin(char o) { origin = o; }
#endif // ROMPIAXFANTASMA


private:
private: char origin = '?';
    Vector3D pos;       // posición
    Vector3D vel;       // velocidad
    Vector3D acc;       // aceleración
    float    damping;   // 0..1 ; v <- v * d^dt
    float mass=1.0f;         // masa dela particula (aunque es independiente de g)

    IntegratorType integrator;

    // Render
    physx::PxTransform pose;
    RenderItem* renderItem;

    // Helpers
    void integrateEulerExplicit(float dt);
    void integrateEulerSemiImplicit(float dt);
    void applyDamping(float dt);
    void syncPoseToRender();
   
    
   


    Vector3D forceAccum; // acumulador de fuerzas del frame
};

