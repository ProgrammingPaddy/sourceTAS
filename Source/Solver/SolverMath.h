#pragma once

// Minimal engine-free math for the solver core. Field names mirror the DLL's
// Vector (X/Y/Z) so validated code moves between the two worlds by copy, not
// rewrite. The core must never include engine/SDK headers.

#include <cmath>

namespace Solver {

	struct Vec3 {
		float X = 0.f, Y = 0.f, Z = 0.f;
		Vec3() = default;
		Vec3(float x, float y, float z) : X(x), Y(y), Z(z) {}
		Vec3 operator+(const Vec3& o) const { return Vec3(X + o.X, Y + o.Y, Z + o.Z); }
		Vec3 operator-(const Vec3& o) const { return Vec3(X - o.X, Y - o.Y, Z - o.Z); }
	};

	inline Vec3 Scale(const Vec3& v, float s) { return Vec3(v.X * s, v.Y * s, v.Z * s); }
	inline float Dot(const Vec3& a, const Vec3& b) { return a.X * b.X + a.Y * b.Y + a.Z * b.Z; }
	inline float Len2(const Vec3& v) { return Dot(v, v); }
	inline float Len(const Vec3& v) { return sqrtf(Len2(v)); }
	inline float Len2D(const Vec3& v) { return sqrtf(v.X * v.X + v.Y * v.Y); }

	constexpr float kPi = 3.14159265358979323846f;
	inline float Deg2Rad(float d) { return d * (kPi / 180.f); }

} // namespace Solver
