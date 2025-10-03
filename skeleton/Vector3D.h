#pragma once
class Vector3D
{
public:
	Vector3D();
	Vector3D(int x, int y, int z);
	~Vector3D();

	Vector3D normalice();
	Vector3D scalarMul(int scalar);
	Vector3D scalarMul(const Vector3D& other);
	
	// Sobrecargas
	Vector3D& operator=(const Vector3D& other);
	Vector3D& operator+(const Vector3D& other);
	Vector3D& operator-(const Vector3D& other);
	Vector3D& operator*(const Vector3D& other);

    // Getters
    int getX() const;
    int getY() const;
    int getZ() const;

	// Setters
	void setX(int value);
	void setY(int value);
	void setZ(int value);
	void set(int newX, int newY, int newZ);

private:
	int _x;
	int _y;
	int _z;
};

