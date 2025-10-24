#pragma once
#include "PxPhysicsAPI.h"
using namespace physx;

class Vector2D {
public:
    Vector2D();
    Vector2D(double x, double y);
    Vector2D(const PxVec2& vec);
    ~Vector2D();

    Vector2D normalize() const;
    Vector2D scalarMul(double scalar) const;

    // Sobrecargas básicas
    Vector2D operator+(const Vector2D& other) const;
    Vector2D operator-(const Vector2D& other) const;

    // Getters y setters
    double getX() const;
    double getY() const;
    void set(double newX, double newY);

    // Direcciones comunes
    static const Vector2D UP;
    static const Vector2D DOWN;
    static const Vector2D RIGHT;
    static const Vector2D LEFT;
    static const Vector2D ZERO;

private:
    double _x;
    double _y;
};
