#pragma once
#include <cmath>

class Vector3D {
public:
    float x, y, z;

    // Constructores
    Vector3D();                          // (0,0,0)
    Vector3D(float x, float y, float z); // (x,y,z)

    // Magnitud y normalizaci�n
    float length() const;        // |v|
    void normalize();            // v = v / |v| (si |v|>0)
    Vector3D normalized() const; // devuelve una copia normalizada

    // Producto escalar
    float dot(const Vector3D& other) const;

    // Operadores b�sicos
    Vector3D& operator+=(const Vector3D& r);
    Vector3D& operator-=(const Vector3D& r);
    Vector3D  operator+(const Vector3D& r) const;
    Vector3D  operator-(const Vector3D& r) const;

    // Multiplicaci�n por escalar (v*s y s*v)
    Vector3D  operator*(float s) const;
    friend Vector3D operator*(float s, const Vector3D& v);
};
