#pragma once
#include "ForceGenerator.h"
#include "Vector3D.h"

class GravityFG : public ForceGenerator {
public:
    explicit GravityFG(const Vector3D& g) : g_(g) {}
    void updateForce(Particle* p, float dt) override;
    void setG(const Vector3D& g) { g_ = g; }
    const Vector3D& getG() const { return g_; }
private:
    Vector3D g_;
};
