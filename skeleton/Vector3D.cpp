#include "Vector3D.h"
#include <cmath>
#include <iostream>


Vector3D::Vector3D()
{
    _x = 0;
    _y = 0;
    _z = 0;
}

Vector3D::Vector3D(double x, double y, double z)
{
    _x = x;
    _y = y;
    _z = z;
}


Vector3D::Vector3D(const PxVec3& vec)
    : _x(vec.x), _y(vec.y), _z(vec.z)
{
}

Vector3D::~Vector3D()
{
}

double Vector3D::normalize()
{
    double magnitude = std::sqrt(_x * _x + _y * _y + _z * _z);
    if (magnitude == 0) return 0;

    _x /= magnitude;
    _y /= magnitude;
    _z /= magnitude;

    return magnitude;
}

Vector3D Vector3D::normalized() const
{
    double magnitude =  std::sqrt(_x * _x + _y * _y + _z * _z);
    if (magnitude == 0) return Vector3D(0, 0, 0);
    return Vector3D(_x / magnitude, _y / magnitude, _z / magnitude);
}

Vector3D Vector3D::scalarMul(double scalar) const
{
    return Vector3D(_x * scalar, _y * scalar, _z * scalar);
}

double Vector3D::scalarProduct(const Vector3D& other) const
{
    return _x * other._x + _y * other._y + _z * other._z;
}

Vector3D Vector3D::vectorialProduct(const Vector3D& other) const
{
    return Vector3D(_x * other._x, _y * other._y, _z * other._z);
}

Vector3D& Vector3D::operator=(const Vector3D& other)
{
    if (this != &other) {
        _x = other._x;
        _y = other._y;
        _z = other._z;
    }
    return *this;
}

Vector3D Vector3D::operator+(const Vector3D& other) const
{
    return Vector3D(_x + other._x, _y + other._y, _z + other._z);
}

Vector3D Vector3D::operator-(const Vector3D& other) const
{
    return Vector3D(_x - other._x, _y - other._y, _z - other._z);
}

Vector3D Vector3D::operator*(const Vector3D& other) const
{
    return Vector3D(_y * other._z - _z * other._y, _z * other._x - _x * other._z, _x * other._y - _y * other._x);
}

double Vector3D::getX() const { return _x; }
double Vector3D::getY() const { return _y; }
double Vector3D::getZ() const { return _z; }

void Vector3D::setX(double value) { _x = value; }
void Vector3D::setY(double value) { _y = value; }
void Vector3D::setZ(double value) { _z = value; }
void Vector3D::set(double newX, double newY, double newZ) { _x = newX; _y = newY; _z = newZ; }

// Definición de las constantes direccionales
const Vector3D Vector3D::UP(0.0, 1.0, 0.0);
const Vector3D Vector3D::DOWN(0.0, -1.0, 0.0);
const Vector3D Vector3D::RIGHT(1.0, 0.0, 0.0);
const Vector3D Vector3D::LEFT(-1.0, 0.0, 0.0);
const Vector3D Vector3D::FORWARD(0.0, 0.0, 1.0);
const Vector3D Vector3D::BACK(0.0, 0.0, -1.0);
const Vector3D Vector3D::ZERO(0.0, 0.0, 0.0);