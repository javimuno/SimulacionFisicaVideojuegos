#pragma once

#include "Vector3D.h"
#include "RenderUtils.hpp"
#include "core.hpp"

class Projectile {
public:
    // Constructor para inicializar el proyectil
    Projectile(Vector3D Pos, Vector3D Vel, Vector3D Acc, float Mass);

    // Destructor
    ~Projectile();

    // Método para integrar la física (actualizar la posición y velocidad del proyectil)
    void integrate(double t);

private:
    // Atributos del proyectil
    Vector3D vel;  // Velocidad
    Vector3D acc;  // Aceleración
    float mass;    // Masa
    physx::PxTransform pose; // Pose para la representación gráfica
    RenderItem* renderItem;  // Representación gráfica
};


