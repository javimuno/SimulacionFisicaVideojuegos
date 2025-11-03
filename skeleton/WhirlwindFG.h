#pragma once
#include "ForceGenerator.h"
#include "ZoneSphere.h"
#include "Vector3D.h"

// Torbellino alrededor del eje Y.
// v_wind(p) = v_tangencial + v_radial_in + v_updraft  (dentro de la esfera)
class WhirlwindFG : public ForceGenerator {
public:
    WhirlwindFG(const ZoneSphere& zone,
        float omega = 5.0f,
        float updraft = 0.0f,
        float radialIn = 0.0f,
        float k1 = 2.0f,
        float k2 = 0.0f)
        : zone_(zone), omega_(omega), updraft_(updraft), radialIn_(radialIn), k1_(k1), k2_(k2) {
    }

    void updateForce(Particle* p, float dt) override;

    // Helpers 
    ZoneSphere& zone() { return zone_; }
    const ZoneSphere& zone() const { return zone_; }
    void setOmega(float w) { omega_ = w; }
    void setUpdraft(float u) { updraft_ = u; }
    void setRadialIn(float r) { radialIn_ = r; }
    void setK1(float k) { k1_ = k; }
    void setK2(float k) { k2_ = k; }

private:
    ZoneSphere zone_;
    float omega_;    // rad/s (giro en torno a Y)
    float updraft_;  // m/s (viento vertical +Y)
    float radialIn_; // m/s (succión hacia el centro en el plano XZ)
    float k1_, k2_;  // coeficientes laminar y turbulento
};
