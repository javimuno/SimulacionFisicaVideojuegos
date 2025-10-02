#include "Vector3D.h"

// Constructores
Vector3D::Vector3D() : x(0.0f), y(0.0f), z(0.0f) {}
Vector3D::Vector3D(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

// Magnitud y normalización
float Vector3D::length() const {
    return std::sqrt(x * x + y * y + z * z);
}

void Vector3D::normalize() {
    float m = length();
    if (m > 0.0f) { x /= m; y /= m; z /= m; }
}

Vector3D Vector3D::normalized() const {
    float m = length();
    if (m > 0.0f) return Vector3D(x / m, y / m, z / m);
    return Vector3D();
}

// Producto escalar
float Vector3D::dot(const Vector3D& other) const {
    return x * other.x + y * other.y + z * other.z;
}

// Operadores
Vector3D& Vector3D::operator+=(const Vector3D& r) {
    x += r.x; y += r.y; z += r.z; return *this;
}
Vector3D& Vector3D::operator-=(const Vector3D& r) {
    x -= r.x; y -= r.y; z -= r.z; return *this;
}
Vector3D Vector3D::operator+(const Vector3D& r) const {
    return Vector3D(x + r.x, y + r.y, z + r.z);
}
Vector3D Vector3D::operator-(const Vector3D& r) const {
    return Vector3D(x - r.x, y - r.y, z - r.z);
}
Vector3D Vector3D::operator*(float s) const {
    return Vector3D(x * s, y * s, z * s);
}
Vector3D operator*(float s, const Vector3D& v) {
    return Vector3D(v.x * s, v.y * s, v.z * s);
}
