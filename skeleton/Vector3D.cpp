//#pragma once
//#include "Vector3D.h"
//#include <cmath>
//#include <iostream>
//
//class Vector3D {
//public:
//    float x, y, z;
//
//    // Constructor por defecto
//    Vector3D(float x_ = 0.0f, float y_ = 0.0f, float z_ = 0.0f) : x(x_), y(y_), z(z_) {}
//
//    // Normalizar el vector
//    Vector3D normalize() const {
//        float magnitude = mod();
//        return (magnitude > 0) ? Vector3D(x / magnitude, y / magnitude, z / magnitude) : Vector3D(0, 0, 0);
//    }
//
//    // Obtener el módulo del vector
//    float mod() const {
//        return std::sqrt(x * x + y * y + z * z);
//    }
//
//    // Producto escalar con otro vector
//    float dotProduct(const Vector3D& v) const {
//        return x * v.x + y * v.y + z * v.z;
//    }
//
//    // Multiplicar el vector por un escalar
//    Vector3D operator*(float scalar) const {
//        return Vector3D(x * scalar, y * scalar, z * scalar);
//    }
//
//    // Sobrecarga de operadores
//    Vector3D& operator=(const Vector3D& v) {
//        x = v.x;
//        y = v.y;
//        z = v.z;
//        return *this;
//    }
//
//    Vector3D operator+(const Vector3D& v) const {
//        return Vector3D(x + v.x, y + v.y, z + v.z);
//    }
//
//    Vector3D operator-(const Vector3D& v) const {
//        return Vector3D(x - v.x, y - v.y, z - v.z);
//    }
//
//    // Mostrar el vector (opcional para depuración)
//    void print() const {
//        std::cout << "Vector(" << x << ", " << y << ", " << z << ")" << std::endl;
//    }
//
//
//};

#include "Vector3D.h"
#include <cmath>  // Para sqrt
#include <iostream>

// Constructor
Vector3D::Vector3D(float x, float y, float z) : x(x), y(y), z(z) {}

// Obtener la magnitud del vector
float Vector3D::magnitude() const {
    return sqrt(x * x + y * y + z * z);
}

// Normalizar el vector
void Vector3D::normalize() {
    float mag = magnitude();
    if (mag > 0.0f) {
        x /= mag;
        y /= mag;
        z /= mag;
    }
}

// Producto escalar
float Vector3D::dotProduct(const Vector3D& vec) {
    return x * vec.x + y * vec.y + z * vec.z;
}

// Operador de suma de vectores
Vector3D Vector3D::operator+(const Vector3D& vec) const {
    return Vector3D(x + vec.x, y + vec.y, z + vec.z);
}

// Operador de resta de vectores
Vector3D Vector3D::operator-(const Vector3D& vec) const {
    return Vector3D(x - vec.x, y - vec.y, z - vec.z);
}

// Multiplicar el vector por un escalar
Vector3D Vector3D::operator*(float scalar) const {
    return Vector3D(x * scalar, y * scalar, z * scalar);

}

// Sobrecarga del operador de asignación
Vector3D& Vector3D::operator=(const Vector3D& vec) {
    if (this == &vec) return *this;  // Evitar auto-asignación
    x = vec.x;
    y = vec.y;
    z = vec.z;
    return *this;
}
