#pragma once  // Prevenir m�ltiples inclusiones

class Vector3D {
public:
    // Atributos del vector
    float x, y, z;

    // Constructor
    Vector3D(float x = 0, float y = 0, float z = 0);

    // M�todos
    float magnitude() const;                // Obtener la magnitud
    void normalize();                       // Normalizar el vector
    float dotProduct(const Vector3D& vec);  // Producto escalar
    Vector3D operator+(const Vector3D& vec) const;  // Suma de vectores
    Vector3D operator-(const Vector3D& vec) const;  // Resta de vectores
    Vector3D operator*(float scalar) const;        // Multiplicaci�n por un escalar
    Vector3D& operator=(const Vector3D& vec);      // Asignaci�n
};

