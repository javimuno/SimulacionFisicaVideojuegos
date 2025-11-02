#pragma once
#include "ForceGenerator.h"
#include "Vector3D.h"

class GravityFG : public ForceGenerator {
public:
    explicit GravityFG(const Vector3D& g) : g_(g) {}
    void updateForce(Particle* p, float dt) override;
private:
    Vector3D g_;
};
