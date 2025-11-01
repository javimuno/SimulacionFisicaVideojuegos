#pragma once
//para borrar las partículas que se salen de un offset 
//que yo mismo creo y customizo

#include "Vector3D.h"
#include <vector>

class Particle;

class WorldBounds {
public:
    WorldBounds(const Vector3D& minB, const Vector3D& maxB);

    void set(const Vector3D& minB, const Vector3D& maxB);

    // ¿El punto es finito? 
    bool isFinite(const Vector3D& p) const;

    // ¿Está fuera de los límites?
    bool isOutside(const Vector3D& p) const;

    // Borra (delete) y quita del vector todas las partículas fuera/no finitas.
    void removeOutside(std::vector<Particle*>& v) const;

    // Borra una sola partícula si está fuera/no finita. Devuelve true si la borro
    bool removeOutside(Particle*& p) const;

    const Vector3D& min() const { return minB_; }
    const Vector3D& max() const { return maxB_; }

private:
    Vector3D minB_;
    Vector3D maxB_;
};
