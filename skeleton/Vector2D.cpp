#include "Vector2D.h"
#include <cmath>

Vector2D::Vector2D() : _x(0), _y(0) {}
Vector2D::Vector2D(double x, double y) : _x(x), _y(y) {}
Vector2D::Vector2D(const PxVec2& vec) : _x(vec.x), _y(vec.y) {}
Vector2D::~Vector2D() {}

Vector2D Vector2D::normalize() const {
    double mag = std::sqrt(_x * _x + _y * _y);
    if (mag == 0) return Vector2D(0, 0);
    return Vector2D(_x / mag, _y / mag);
}

Vector2D Vector2D::scalarMul(double scalar) const {
    return Vector2D(_x * scalar, _y * scalar);
}

Vector2D Vector2D::operator+(const Vector2D& other) const {
    return Vector2D(_x + other._x, _y + other._y);
}

Vector2D Vector2D::operator-(const Vector2D& other) const {
    return Vector2D(_x - other._x, _y - other._y);
}

double Vector2D::getX() const { return _x; }
double Vector2D::getY() const { return _y; }
void Vector2D::set(double newX, double newY) { _x = newX; _y = newY; }


// Definición de las constantes direccionales
const Vector2D Vector2D::UP(0.0, 1.0);
const Vector2D Vector2D::DOWN(0.0, -1.0);
const Vector2D Vector2D::RIGHT(1.0, 0.0);
const Vector2D Vector2D::LEFT(-1.0, 0.0);
const Vector2D Vector2D::ZERO(0.0, 0.0);