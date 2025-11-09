#pragma once

#include "PxPhysicsAPI.h"

using namespace physx;

class Vector3D
{
public:
	Vector3D();
	Vector3D(double x, double y, double z);
	Vector3D(const PxVec3& vec);
	~Vector3D();

	Vector3D normalized() const;
	Vector3D scalarMul(double scalar) const;

	double   scalarProduct(const Vector3D& other) const;
	Vector3D vectorialProduct(const Vector3D& other) const;
	
	// Sobrecargas
	Vector3D& operator=(const Vector3D& other);
	Vector3D operator+(const Vector3D& other) const;
	Vector3D operator-(const Vector3D& other) const;
	Vector3D operator*(const Vector3D& other) const;

    // Getters
	double getX() const;
	double getY() const;
	double getZ() const;

	// Setters
	void setX(double value);
	void setY(double value);
	void setZ(double value);
	void set(double newX, double newY, double newZ);

	// Direcciones comunes
	static const Vector3D UP;
	static const Vector3D DOWN;
	static const Vector3D RIGHT;
	static const Vector3D LEFT;
	static const Vector3D FORWARD;
	static const Vector3D BACK;
	static const Vector3D ZERO;

private:
	double _x;
	double _y;
	double _z;
};

