#pragma once
#include "ForceGenerator.h"
#include "Vector3D.h"
#include "Particle.h"

// Muelle anclado: F = -k (|d|-L0) n  -  c (v*n) n
class AnchoredSpringFG : public ForceGenerator {
public:
    AnchoredSpringFG(const Vector3D& anchor, float k, float restLen,
        float damping = 0.0f, bool onlyExtend = false)
        : anchor_(anchor), k_(k), rest_(restLen), c_(damping), onlyExtend_(onlyExtend) {
    }

    void updateForce(Particle* p, float dt) override;

    // ajustes en caliente
    void setK(float k) { k_ = (k < 0.f ? 0.f : k); }
    void setRest(float L0) { rest_ = (L0 < 0.f ? 0.f : L0); }
    void setDamping(float c) { c_ = (c < 0.f ? 0.f : c); }

    float k()    const { return k_; }
    float rest() const { return rest_; }
    float damp() const { return c_; }

private:
    Vector3D anchor_;   // punto fijo
    float    k_;        // rigidez
    float    rest_;     // longitud de reposo
    float    c_;        // amortiguamiento 
    bool     onlyExtend_; // true=sólo actúa al estirar
};
