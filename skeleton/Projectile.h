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

    // M�todo para integrar la f�sica (actualizar la posici�n y velocidad del proyectil)
    void integrate(double t);

private:
    // Atributos del proyectil
    Vector3D vel;  // Velocidad
    Vector3D acc;  // Aceleraci�n
    float mass;    // Masa
    physx::PxTransform pose; // Pose para la representaci�n gr�fica
    RenderItem* renderItem;  // Representaci�n gr�fica
};


