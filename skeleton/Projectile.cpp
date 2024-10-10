#include "Projectile.h"

// Constructor
Projectile::Projectile(Vector3D Pos, Vector3D Vel, Vector3D Acc, float Mass)
    : vel(Vel), acc(Acc), mass(Mass) {

    // Inicializar la posición y la representación gráfica del proyectil
    pose = physx::PxTransform(Pos.x, Pos.y, Pos.z);
    renderItem = new RenderItem(CreateShape(physx::PxSphereGeometry(0.2)), &pose, { 0.8, 0.8, 0.8, 1 });
    RegisterRenderItem(renderItem);
}

// Destructor
Projectile::~Projectile() {
    DeregisterRenderItem(renderItem);
}

// Método para integrar la física
void Projectile::integrate(double t) {
    // Actualizar la velocidad con la aceleración
    vel.x += acc.x * t;
    vel.y += acc.y * t;
    vel.z += acc.z * t;

    // Actualizar la posición con la nueva velocidad
    pose.p.x += vel.x * t;
    pose.p.y += vel.y * t;
    pose.p.z += vel.z * t;

    // Actualizar la representación gráfica
    pose = physx::PxTransform(pose.p.x, pose.p.y, pose.p.z);
}
