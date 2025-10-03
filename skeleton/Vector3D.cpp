#include "Vector3D.h"
#include <cmath>

Vector3D::Vector3D()
{
    _x = 0;
    _y = 0;
    _z = 0;
}

Vector3D::Vector3D(int x, int y, int z)
{
    _x = x;
    _y = y;
    _z = z;
}

Vector3D::~Vector3D()
{
}

Vector3D Vector3D::normalice()
{
    double magnitude =  std::sqrt(_x * _x + _y * _y + _z * _z);
    if (magnitude == 0) return Vector3D(0, 0, 0);
    return Vector3D(_x / magnitude, _y / magnitude, _z / magnitude);
}

Vector3D Vector3D::scalarMul(int scalar)
{
    return Vector3D(_x * scalar, _y * scalar, _z * scalar);
}

Vector3D Vector3D::scalarMul(const Vector3D& other)
{
    return Vector3D(_x * other._x, _y * other._y, _z * other._z);
}

Vector3D& Vector3D::operator=(const Vector3D& other)
{
    return Vector3D(other._x, other._y, other._z);
}

Vector3D& Vector3D::operator+(const Vector3D& other)
{
    return Vector3D(_x + other._x, _y + other._y, _z + other._z);
}

Vector3D& Vector3D::operator-(const Vector3D& other)
{
    return Vector3D(_x - other._x, _y - other._y, _z - other._z);
}

Vector3D& Vector3D::operator*(const Vector3D& other)
{
    return Vector3D(_y * other._z - _z * other._y, _z * other._x - _x * other._z, _x * other._y - _y * other._x);
}

int Vector3D::getX() const { return _x; }
int Vector3D::getY() const { return _y; }
int Vector3D::getZ() const { return _z; }

void Vector3D::setX(int value) { _x = value; }
void Vector3D::setY(int value) { _y = value; }
void Vector3D::setZ(int value) { _z = value; }
void Vector3D::set(int newX, int newY, int newZ) { _x = newX; _y = newY; _z = newZ; }