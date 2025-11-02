#pragma once
#include "ForceGenerator.h"
#include "Vector3D.h"

class Particle;

class WindFG : public ForceGenerator {
public:
    WindFG(const Vector3D& windVel = Vector3D(0, 0, 0), float k1 = 1.0f, float k2 = 0.0f)
        : vWind_(windVel), k1_(k1), k2_(k2) {
    }

    void updateForce(Particle* p, float dt) override;

    // Helpers para probar desde teclas
    void setWind(const Vector3D& v) { vWind_ = v; }
    void setK1(float k) { k1_ = k; }
    void setK2(float k) { k2_ = k; }

private:
    Vector3D vWind_; // velocidad del viento (constante y global, de momento)
    float k1_;       // parte lineal
    float k2_;       // parte cuadrática (turbulenta); empezamos con 0
};


