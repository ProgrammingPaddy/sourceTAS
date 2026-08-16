#pragma once

// The shared HEADING CONTROLLER (M1.3/M1.4): one tick of steering the
// exact engine toward a target heading, entirely derived from the
// certified strafe law (SolverStrafe.h). Used by the air primitive
// (SolverAir) and the carve primitive (SolverCarve) so both phases
// steer with the SAME law:
//  - demand within the perpendicular turn rate: land it exactly at
//    full gain (turn-curve inversion over cosa in [0, cap/v]);
//  - beyond it: the BRAKING TURN (cosa < 0; the accel budget dwarfs
//    the 30 cap, so a backward-biased wish rotates velocity far
//    faster, at speed cost the OPTIMIZER owns via the spline slope);
//  - side flips rate-limited to MoveParams::strafe_rate_max by
//    construction: a blocked flip weaves (small error) or coasts
//    (large error).

#include <math.h>

#include "SolverMove.h"
#include "SolverStrafe.h"

namespace Solver {
namespace Steer {

	constexpr float kSteerPi = 3.14159265358979f;

	inline float WrapPi(float a) {
		while (a > kSteerPi) a -= 2.f * kSteerPi;
		while (a < -kSteerPi) a += 2.f * kSteerPi;
		return a;
	}

	inline float TurnAt(const Strafe::TickLaw& law, float cosa) {
		const float s2 = 1.f - cosa * cosa;
		return law.TurnRad(cosa, s2 > 0.f ? sqrtf(s2) : 0.f);
	}

	// The cosa in [0, min(1, cap/v)] whose TurnRad equals want
	// (monotone decreasing in cosa).
	inline float CosForTurn(const Strafe::TickLaw& law, float want) {
		float lo = 0.f;
		float hi = law.v > law.cap ? law.cap / law.v : 1.f;
		for (int it = 0; it < 12; ++it) {
			const float mid = 0.5f * (lo + hi);
			if (TurnAt(law, mid) > want)
				lo = mid;
			else
				hi = mid;
		}
		return 0.5f * (lo + hi);
	}

	// The braking turn: cosa in [-1, 0] achieving want (or the max-turn
	// cosa if want exceeds the achievable peak). The turn curve rises
	// from TurnAt(0) to a peak then falls: scan the peak, bisect the
	// rising side.
	inline float CosForBrakeTurn(const Strafe::TickLaw& law, float want) {
		float best_c = 0.f, best_t = TurnAt(law, 0.f);
		for (int i = 1; i <= 16; ++i) {
			const float c = -static_cast<float>(i) / 16.f;
			const float tr = TurnAt(law, c);
			if (tr > best_t) {
				best_t = tr;
				best_c = c;
			}
		}
		if (want >= best_t)
			return best_c;
		float lo = 0.f, hi = best_c;
		for (int it = 0; it < 12; ++it) {
			const float mid = 0.5f * (lo + hi);
			if (TurnAt(law, mid) < want)
				lo = mid;
			else
				hi = mid;
		}
		return 0.5f * (lo + hi);
	}

	struct Controller {
		int side = 0;
		int last_flip = -1000;
		int flips = 0;

		// One tick of control toward heading theta. Returns the inputs
		// to feed MoveTick.
		void Tick(const PlayerState& s, const MoveParams& p,
		          float theta, int k,
		          float* yaw_deg, float* fmove, float* smove) {
			const int min_gap = static_cast<int>(
				ceilf((1.f / p.dt) / p.strafe_rate_max));
			const float s2d = Len2D(s.vel);
			const float h = s2d > 1.f
				? atan2f(s.vel.Y, s.vel.X) : theta;
			const float e = WrapPi(theta - h);
			Strafe::TickLaw law = Strafe::Law(p, s2d, 1.f, s.ducked);
			const float maxturn = law.TurnRad(0.f, 1.f);
			int want = side;
			if (e > 0.02f) want = 1;
			else if (e < -0.02f) want = -1;
			else if (side == 0) want = (e >= 0.f) ? 1 : -1;
			bool coast = false;
			if (want != side) {
				if (side == 0 || k - last_flip >= min_gap) {
					side = want;
					last_flip = k;
					flips += (k > 0) ? 1 : 0;
				} else if (fabsf(e) > 0.26f) {
					coast = true;
				}
				// else: weave on the old side until the flip is legal
			}
			*yaw_deg = h * 57.2957795f;
			*smove = 0.f;
			*fmove = 0.f;
			if (!coast && s2d > 1.f) {
				const float ae = fabsf(e);
				const bool with_side = (side > 0) == (e > 0.f)
					&& ae > 1e-4f;
				const float cosa = (with_side && ae < maxturn)
					? CosForTurn(law, ae)
					: (with_side ? CosForBrakeTurn(law, ae) : law.cap
						/ (s2d > law.cap ? s2d : law.cap));
				const float alpha = acosf(cosa > 1.f ? 1.f : cosa);
				const float wh = h + (side > 0 ? alpha : -alpha);
				*smove = -450.f * static_cast<float>(side);
				*yaw_deg = (wh + (side > 0 ? 1.f : -1.f)
					* kSteerPi * 0.5f) * 57.2957795f;
			} else if (!coast) {
				*yaw_deg = theta * 57.2957795f;
				*fmove = 450.f;
			}
		}
	};

} // namespace Steer
} // namespace Solver
