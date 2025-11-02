#include "ZoneSphere.h"
#include <cmath>

bool ZoneSphere::contains(const Vector3D& p) const {
    Vector3D d = p - center;
    return d.length() <= radius;
}
