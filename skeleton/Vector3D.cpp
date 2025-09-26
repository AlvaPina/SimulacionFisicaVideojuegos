#include "Vector3D.h"
#include <cmath>

Vector3D::Vector3D(int x, int y, int z)
{
}

Vector3D::~Vector3D()
{
}

Vector3D Vector3D::normalice()
{
    double magnitude =  std::sqrt(x * x + y * y + z * z);
    if (magnitude == 0) return Vector3D(0, 0, 0);
    return Vector3D(x / magnitude, y / magnitude, z / magnitude);
}

Vector3D Vector3D::scalarMul(int scalar)
{
    return Vector3D(x * scalar, y * scalar, z * scalar);
}

Vector3D Vector3D::scalarMul(const Vector3D& other)
{
    return Vector3D(x * other.x, y * other.y, z * other.z);
}

Vector3D& Vector3D::operator=(const Vector3D& other)
{
    return Vector3D(other.x, other.y, other.z);
}

Vector3D& Vector3D::operator+(const Vector3D& other)
{
    return Vector3D(x + other.x, y + other.y, z + other.z);
}

Vector3D& Vector3D::operator-(const Vector3D& other)
{
    return Vector3D(x - other.x, y - other.y, z - other.z);
}

Vector3D& Vector3D::operator*(const Vector3D& other)
{
    return Vector3D(y * other.z - z * other.y, z * other.x - x * other.z, x * other.y - y * other.x);
}

int Vector3D::getX() const { return x; }
int Vector3D::getY() const { return y; }
int Vector3D::getZ() const { return z; }

void Vector3D::setX(int value) { x = value; }
void Vector3D::setY(int value) { y = value; }
void Vector3D::setZ(int value) { z = value; }
void Vector3D::set(int newX, int newY, int newZ) { x = newX; y = newY; z = newZ; }