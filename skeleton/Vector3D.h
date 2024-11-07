#pragma once  // Prevenir múltiples inclusiones

class Vector3D {
public:
    // Atributos del vector
    float x, y, z;

    // Constructor
    Vector3D(float x = 0, float y = 0, float z = 0);

    // Métodos
    float magnitude() const;                // Obtener la magnitud
    void normalize();                       // Normalizar el vector
    float dotProduct(const Vector3D& vec);  // Producto escalar
    Vector3D operator+(const Vector3D& vec) const;  // Suma de vectores
    Vector3D operator-(const Vector3D& vec) const;  // Resta de vectores
    Vector3D operator*(float scalar) const;        // Multiplicación por un escalar
    Vector3D& operator=(const Vector3D& vec);      // Asignación

    // Nuevas sobrecargas
    Vector3D& operator+=(const Vector3D& vec);      // Suma acumulativa
    Vector3D& operator*=(float scalar);             // Multiplicación acumulativa

    // Operadores amigos
    friend Vector3D operator*(float scalar, const Vector3D& vec);  // Escalar * vector

    
};


