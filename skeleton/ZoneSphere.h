#pragma once
#include "Vector3D.h"

class ZoneSphere {
public:
    ZoneSphere(const Vector3D& c = Vector3D(0, 0, 0), float r = 1.0f)
        : center(c), radius(r) {
    }

    bool contains(const Vector3D& p) const;
    void setCenter(const Vector3D& c) { center = c; }
    void setRadius(float r) { radius = r; }

    const Vector3D& getCenter() const { return center; }
    float getRadius() const { return radius; }

private:
    Vector3D center;
    float radius;
};
