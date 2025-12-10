#include "AnchoredSpringFG.h"
#include <cmath>

void AnchoredSpringFG::updateForce(Particle* p, float /*dt*/)
{
    if (!p) return;
    if (p->getMass() <= 0.0f) return;

    // d = pos - anclaje 
    Vector3D d = p->getPosition() - anchor_;

    // |d| (longitud)
    const float L2 = d.x * d.x + d.y * d.y + d.z * d.z;
    if (L2 <= 1e-12f) return;              // evita division por 0 = mal
    const float L = std::sqrt(L2);

    // para el unitario (n) de dirección n =(d/|d|)
    const float invL = 1.0f / L;
    Vector3D n(d.x * invL, d.y * invL, d.z * invL);

    // estiramiento (+) / compresión (-) respeco al reposo
    const float x = L - rest_;
    if (onlyExtend_ && x <= 0.0f) return;

    // Hooke
    const float Fk = -k_ * x;

    // amortiguamiento en el eje del muelle: vrel = v * n 
    const Vector3D v = p->getVelocity();
    const float vrel = v.x * n.x + v.y * n.y + v.z * n.z;
    const float Fc = (c_ > 0.0f) ? (-c_ * vrel) : 0.0f;

    // por si quiero que también haya en x y z meh
    const float mag = Fk + Fc;
    Vector3D F(n.x * mag, n.y * mag, n.z * mag);

    // fuerzas
    p->addForce(F);
}
