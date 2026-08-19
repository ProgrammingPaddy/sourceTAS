#pragma once

// M1.1 - THE REACHABILITY ENVELOPE (Docs/SolverRebuildChecklist.md).
// From an airborne exit state, what states are reachable n ticks later?
// Two structural facts of the certified engine make this sharp:
//
//  1. AIRBORNE VERTICAL MOTION IS PURE BALLISTIC. AirMove flattens the
//     wish (pitch never enters surf movement), so gravity is the only
//     vertical force and z after n ticks is EXACT closed form. With the
//     tick's half-gravity structure (StartGravity g*dt/2, move, then
//     FinishGravity g*dt/2), tick k's displacement uses
//     vz0 - g*dt*(k - 1/2):
//         z(n)  = z0 + n*dt*vz0 - g*dt^2*n^2/2          (EXACT)
//         vz(n) = vz0 - g*dt*n                          (EXACT)
//  2. HORIZONTAL GAIN obeys the proven strafe law (SolverStrafe.h,
//     float-ULP certified): per tick, speed^2 grows by at most cap^2
//     (900), in every legal regime. So after n ticks
//         s(n)^2 <= s0^2 + cap^2 * n                    (OUTER BOUND)
//     and the horizontal distance covered is at most the gain-optimal
//     straight line   D(n) = dt * sum_{k=1..n} sqrt(s0^2 + cap^2 k).
//
// LIMITS (declared): no base-velocity boosts mid-air (triggers change
// vz/vel and re-anchor the envelope), no water. Gravity uses the state's
// gravity_scale like the engine does.

#include "SolverMove.h"

namespace Solver {
namespace Envelope {

	inline float ZAfter(float z0, float vz0, int n, const MoveParams& p,
	                    float gravity_scale = 1.f) {
		const float g = p.gravity * gravity_scale;
		const float t = static_cast<float>(n);
		return z0 + t * p.dt * vz0 - 0.5f * g * p.dt * p.dt * t * t;
	}

	inline float VzAfter(float vz0, int n, const MoveParams& p,
	                     float gravity_scale = 1.f) {
		return vz0 - p.gravity * gravity_scale * p.dt
			* static_cast<float>(n);
	}

	// Max horizontal SPEED after n ticks (outer bound, gain law).
	inline float SMax(float s0, int n, const MoveParams& p) {
		const float c2 = p.air_speed_cap * p.air_speed_cap;
		return sqrtf(s0 * s0 + c2 * static_cast<float>(n));
	}

	// Max horizontal DISTANCE covered in n ticks (gain-optimal straight
	// line; each tick moves at the post-gain speed).
	inline float DMax(float s0, int n, const MoveParams& p) {
		const float c2 = p.air_speed_cap * p.air_speed_cap;
		float d = 0.f;
		for (int k = 1; k <= n; ++k)
			d += sqrtf(s0 * s0 + c2 * static_cast<float>(k));
		return d * p.dt;
	}

	// The tick window in which the ballistic z crosses the band
	// [z_lo, z_hi] (a face's height extent). Returns false if never
	// (within max_ticks). Conservative: any tick whose z lies in the
	// padded band counts.
	inline bool ZWindow(float z0, float vz0, float z_lo, float z_hi,
	                    const MoveParams& p, int max_ticks,
	                    int* n_first, int* n_last,
	                    float gravity_scale = 1.f) {
		int first = -1, last = -1;
		for (int n = 1; n <= max_ticks; ++n) {
			const float z = ZAfter(z0, vz0, n, p, gravity_scale);
			if (z >= z_lo && z <= z_hi) {
				if (first < 0)
					first = n;
				last = n;
			} else if (first >= 0 && z < z_lo) {
				break;   // fell through the band
			}
		}
		if (first < 0)
			return false;
		if (n_first) *n_first = first;
		if (n_last) *n_last = last;
		return true;
	}

	// Edge gate: can a transfer from exit state (pos, vel) plausibly
	// reach a target whose extent is [z_lo, z_hi] at horizontal distance
	// dist_xy? True = the analytic bounds allow it (stage 3 decides for
	// real); false = provably unreachable, prune.
	inline bool CanReach(const Vec3& pos, const Vec3& vel, float dist_xy,
	                     float z_lo, float z_hi, const MoveParams& p,
	                     int max_ticks, float gravity_scale = 1.f) {
		const float s0 = Len2D(vel);
		int n0 = 0, n1 = 0;
		// Pad the band by the hull height: a board can touch anywhere on
		// the box.
		if (!ZWindow(pos.Z, vel.Z, z_lo - 72.f, z_hi + 72.f, p, max_ticks,
			&n0, &n1, gravity_scale))
			return false;
		return dist_xy <= DMax(s0, n1, p) + 32.f;   // hull-width slack
	}

} // namespace Envelope
} // namespace Solver
