#pragma once
class Vector3D
{
public:
	Vector3D();
	Vector3D(double x, double y, double z);
	~Vector3D();

	Vector3D normalice();
	Vector3D scalarMul(double scalar);
	Vector3D scalarMul(const Vector3D& other);
	
	// Sobrecargas
	Vector3D& operator=(const Vector3D& other);
	Vector3D operator+(const Vector3D& other);
	Vector3D operator-(const Vector3D& other);
	Vector3D operator*(const Vector3D& other);

    // Getters
	double getX() const;
	double getY() const;
	double getZ() const;

	// Setters
	void setX(double value);
	void setY(double value);
	void setZ(double value);
	void set(double newX, double newY, double newZ);

private:
	double _x;
	double _y;
	double _z;
};

