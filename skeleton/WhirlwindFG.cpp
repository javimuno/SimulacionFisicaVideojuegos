#include "WhirlwindFG.h"
#include "Particle.h"
#include <cmath>

static inline float fastLenXZ(const Vector3D& v) {
    return std::sqrt(v.x * v.x + v.z * v.z);
}

void WhirlwindFG::updateForce(Particle* p, float /*dt*/) {
    if (!p) return;

    const Vector3D& pos = p->getPosition();
    if (!zone_.contains(pos)) return;

    // Vector desde el centro
    Vector3D r = pos - zone_.getCenter();

    // Componente tangencial en el plano XZ (giro alrededor de Y)
    Vector3D rXZ(r.x, 0.0f, r.z);
    float rlen = fastLenXZ(rXZ);

    Vector3D vTang(0, 0, 0);
    if (rlen > 1e-6f) {
        // tangente = rotación 90º de rXZ en XZ: (-z, 0, x) normalizada
        Vector3D t = Vector3D(-rXZ.z, 0.0f, rXZ.x) * (1.0f / rlen);
        // v_tangencial = omega * r * t   (sólido rígido: velocidad y radio)
        vTang = t * (omega_ * rlen);
    }

    // Succión radial hacia el centro (opcional)
    Vector3D vRad(0, 0, 0);
    if (radialIn_ > 0.0f && rlen > 1e-6f) {
        Vector3D dirIn = (rXZ * (-1.0f / rlen)); // hacia el centro en XZ
        vRad = dirIn * radialIn_;
    }

    // Updraft vertical
    Vector3D vUp(0.0f, updraft_, 0.0f);

    // Velocidad de viento efectiva en la posición
    Vector3D vWind = vTang + vRad + vUp;

    // Fuerza tipo viento: F = k1 * vRel + k2 * |vRel| * vRel
    Vector3D vRel = vWind - p->getVelocity();
    float speed = vRel.length();
    Vector3D F = vRel * k1_;
    if (k2_ != 0.0f && speed > 0.0f) F += vRel * (k2_ * speed);

    p->addForce(F);
}
