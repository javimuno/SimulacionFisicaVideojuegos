#include "Projectile.h"

// Constructor
Projectile::Projectile(Vector3D Pos, Vector3D Vel, Vector3D Acc, float Mass)
    : vel(Vel), acc(Acc), mass(Mass) {

    // Inicializar la posici�n y la representaci�n gr�fica del proyectil
    pose = physx::PxTransform(Pos.x, Pos.y, Pos.z);
    renderItem = new RenderItem(CreateShape(physx::PxSphereGeometry(0.9)), &pose, { 0.8, 0.8, 0.8, 1 });
    RegisterRenderItem(renderItem);
}

// Destructor
Projectile::~Projectile() {
    DeregisterRenderItem(renderItem);
}

// M�todo para integrar la f�sica
void Projectile::integrate(double t) {
    // Actualizar la velocidad con la aceleraci�n
    vel.x += acc.x * t;
    vel.y += acc.y * t;
    vel.z += acc.z * t;

    // Actualizar la posici�n con la nueva velocidad
    pose.p.x += vel.x * t;
    pose.p.y += vel.y * t;
    pose.p.z += vel.z * t;

    // Actualizar la representaci�n gr�fica
    pose = physx::PxTransform(pose.p.x, pose.p.y, pose.p.z);
}
