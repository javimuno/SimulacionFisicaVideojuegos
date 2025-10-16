#include "Particle.h"
#include <cmath>
using namespace physx;

Particle::Particle(const Vector3D& Pos, const Vector3D& Vel,
    const Vector3D& Acc, float Damping,
    IntegratorType integ,float Mass, PxVec4 color, float radius)
    : pos(Pos), vel(Vel), acc(Acc), damping(Damping), integrator(integ), mass(Mass),
    pose(PxTransform(Pos.x, Pos.y, Pos.z)), renderItem(nullptr)
{
    // Esfera de radio 1.0 y amarillo pacman
    renderItem = new RenderItem(CreateShape(PxSphereGeometry(1.0f)), &pose, color); //{ 1.0f, 0.8f, 0.2f, 1.0f } color base cambiado p2
    RegisterRenderItem(renderItem);
}

Particle::~Particle()
{
    if (renderItem) {
        DeregisterRenderItem(renderItem);
        renderItem = nullptr;
    }
}

void Particle::integrate(float dt)
{
    if (dt <= 0.0f) return;

    switch (integrator) {
    case IntegratorType::EulerExplicit:
        integrateEulerExplicit(dt);
        break;
    case IntegratorType::EulerSemiImplicit:
    default:
        integrateEulerSemiImplicit(dt);
        break;
    }

    applyDamping(dt);
    syncPoseToRender();
}

// x_{n+1} = x_n + v_n*dt
// v_{n+1} = v_n + a_n*dt
void Particle::integrateEulerExplicit(float dt)
{
    pos = pos + vel * dt;
    vel = vel + acc * dt;
}

// v_{n+1} = v_n + a_n*dt
// x_{n+1} = x_n + v_{n+1}*dt
void Particle::integrateEulerSemiImplicit(float dt)
{
    vel = vel + acc * dt;
    pos = pos + vel * dt;
}

// v <- v * damping^dt
void Particle::applyDamping(float dt)
{
    if (damping < 1.0f) {
        // std::pow del <cmath> viene desde Vector3D.h (ya incluido allí)
        float factor = std::pow(damping, dt);
        vel = vel * factor;
    }
}

void Particle::syncPoseToRender()
{
    pose = PxTransform(pos.x, pos.y, pos.z);
}
