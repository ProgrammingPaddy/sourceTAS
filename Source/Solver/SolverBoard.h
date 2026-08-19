#pragma once

// M1.2 - THE BOARD WINDOW (Docs/SolverRebuildChecklist.md).
// A "board" is the first clip against a surf face: the engine runs
// ClipVelocity(v1, n) with overbounce 1, where v1 is the velocity
// entering TryPlayerMove (post StartGravity + air wish). The clip
// removes exactly the into-plane component:
//
//     dot   = v1 . n            (must be < 0 to strike the face)
//     v1'   = v1 - n*dot        (+ one adjust pass, epsilon-level)
//     |v1'|^2 = |v1|^2 - dot^2  ->  SPEED^2 LOSS = dot^2   (EXACT)
//
// Writing v1 = (horizontal s at aim angle phi from the face's outward
// horizontal normal, vertical vz) and n = (h * nhat_xy, nz):
//
//     dot(phi) = s*h*cos(phi) + vz*nz
//
// which sweeps the interval [vz*nz - s*h, vz*nz + s*h] as the aim
// turns. Everything below is closed-form consequences of that line:
// the achievable dot range, the minimum possible loss (the testimony's
// tangency law: fall too long relative to horizontal speed and face
// steepness and a hard board is UNAVOIDABLE - and its converse, the
// yaw-90 fall case: a steep face, small nz, eats a long fall free),
// and the aim cone that keeps the loss under a cap.
//
// The REGION half of the window: the contact point must lie on the
// face polygon, with slack for the hull (the recorded contact_pos is
// the HULL CENTER; a 32x32x72 box touching a plane by a corner puts
// the center up to |(16,16,36)| = 42.5u from the touch point in-plane).

#include <float.h>

#include "SolverMove.h"
#include "SolverRoute.h"

namespace Solver {
namespace Board {

	constexpr float kHullCenterSlack = 43.f;   // ceil(|(16,16,36)|), geometric

	// Achievable clip-dot interval over all horizontal aim directions,
	// at horizontal speed s and vertical velocity vz against normal n.
	inline void DotRange(float s, float vz, const Vec3& n,
	                     float* lo, float* hi) {
		const float h = sqrtf(n.X * n.X + n.Y * n.Y);
		const float base = vz * n.Z;
		*lo = base - s * h;
		*hi = base + s * h;
	}

	// Minimum |dot| achievable with an APPROACHING contact (dot < 0).
	// 0       -> a tangent (lossless) board exists;
	// > 0     -> unavoidable board loss, speed^2 cost = value^2;
	// FLT_MAX -> no aim direction approaches the face at all.
	inline float MinApproachDot(float s, float vz, const Vec3& n) {
		float lo, hi;
		DotRange(s, vz, n, &lo, &hi);
		if (lo >= 0.f)
			return FLT_MAX;
		return hi >= 0.f ? 0.f : -hi;
	}

	// The aim cone that keeps dot in [-dot_cap, 0): the cos(phi) range,
	// phi measured from the face's OUTWARD horizontal normal azimuth.
	// (Boarding aims have cos(phi) < 0 - into the face - unless a fast
	// fall lets tangent-plus aims contact too.) False = cap unreachable.
	inline bool AimCone(float s, float vz, const Vec3& n, float dot_cap,
	                    float* cos_lo, float* cos_hi) {
		const float h = sqrtf(n.X * n.X + n.Y * n.Y);
		const float sh = s * h;
		if (sh <= 0.f)
			return vz * n.Z < 0.f && -vz * n.Z <= dot_cap;
		// s*h*c + vz*nz in [-cap, 0)  ->  c in [(-cap - vz*nz)/sh,
		//                                       (0   - vz*nz)/sh)
		float c0 = (-dot_cap - vz * n.Z) / sh;
		float c1 = (0.f - vz * n.Z) / sh;
		if (c0 < -1.f) c0 = -1.f;
		if (c1 > 1.f) c1 = 1.f;
		if (c0 > c1)
			return false;
		if (cos_lo) *cos_lo = c0;
		if (cos_hi) *cos_hi = c1;
		return true;
	}

	// Signed in-plane distance from point p to the face polygon:
	// <= 0 inside, > 0 = distance outside the nearest edge. p is
	// projected onto the plane first. Winding-agnostic (edge normals
	// oriented outward against the centroid).
	inline float EdgeDistOut(const Route::Face& f, const Vec3& p) {
		const float off = Dot(f.n, p) - f.d;
		const Vec3 q = p - Scale(f.n, off);
		float worst = -FLT_MAX;
		const size_t nv = f.verts.size();
		for (size_t i = 0; i < nv; ++i) {
			const Vec3& a = f.verts[i];
			const Vec3& b = f.verts[(i + 1) % nv];
			Vec3 m = Cross(b - a, f.n);
			const float ml = Len(m);
			if (ml < 1e-6f)
				continue;
			m = Scale(m, 1.f / ml);
			if (Dot(m, f.centroid - a) > 0.f)
				m = Scale(m, -1.f);
			const float d = Dot(m, q - a);
			if (d > worst)
				worst = d;
		}
		return worst;
	}

	// Full window test for a recorded contact: approaching, in-region,
	// loss under cap. (dot and pos from TickEvents contact_vel/pos.)
	inline bool InWindow(const Route::Face& f, const Vec3& contact_pos,
	                     const Vec3& v1, float dot_cap,
	                     float* out_dot = nullptr,
	                     float* out_edge = nullptr) {
		const float dot = Dot(v1, f.n);
		const float edge = EdgeDistOut(f, contact_pos);
		if (out_dot) *out_dot = dot;
		if (out_edge) *out_edge = edge;
		return dot < 0.f && -dot <= dot_cap
			&& edge <= kHullCenterSlack;
	}

	// ZERO-INPUT single-plane board tick, predicted end to end: the
	// certified tick with no wish and no buttons is EXACTLY
	//   StartGravity (vz -= g*gs*dt/2) -> clip(v1, n) -> slide ->
	//   FinishGravity (vz -= g*gs*dt/2)
	// so the post-tick velocity is closed-form. Used by the M1.2 gate
	// to spot-check window edge cases against the exact engine.
	inline Vec3 PredictClipTickVel(const Vec3& v, const Vec3& n,
	                               const MoveParams& p,
	                               float gravity_scale = 1.f) {
		Vec3 v1 = v;
		v1.Z -= gravity_scale * p.gravity * 0.5f * p.dt;   // StartGravity
		Vec3 out;
		Fn::ClipVelocity(v1, n, &out);
		out.Z -= gravity_scale * p.gravity * 0.5f * p.dt;  // FinishGravity
		return out;
	}

} // namespace Board
} // namespace Solver
