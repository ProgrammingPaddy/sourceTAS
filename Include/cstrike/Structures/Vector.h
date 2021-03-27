#pragma once

struct Vector {
	float X, Y, Z;

	Vector(float X = 0.f, float Y = 0.f, float Z = 0.f) {
		this->X = X;
		this->Y = Y;
		this->Z = Z;
	}

	Vector operator + (const Vector& input) {
		return Vector(X + input.X, Y + input.Y, Z + input.Z);
	}

	Vector operator - (const Vector& input) {
		return Vector(X - input.X, Y - input.Y, Z - input.Z);
	}

	Vector operator * (const Vector& input) {
		return Vector(X * input.X, Y * input.Y, Z * input.Z);
	}

	Vector operator / (const Vector& input) {
		return Vector(X / input.X, Y / input.Y, Z / input.Z);
	}

	inline float Length() const {
		return sqrtf(X * X + Y * Y + Z * Z);
	}
};

typedef Vector QAngle;