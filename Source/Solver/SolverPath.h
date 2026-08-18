#pragma once

// THE AERIAL PATH SOLVER (user-named build, 2026-08-17): from an exit
// state to a goal cell (position + arrival tangent heading + ballistic
// arrival tick), construct the efficient flight DIRECTLY - no search.
//
// Why construction is complete here: the flight time to a cell is
// fixed by ballistics (vertical motion is pure gravity), and every
// tick turned at-or-under the free rate banks the full strafe gain
// (the certified law's full-turn-full-gain family) - so ALL feasible
// smooth profiles tie on energy, and the problem is pure geometry.
// The family: TURN-HOLD-TURN - rotate from the current heading toward
// a held heading psi at the free rate, hold, rotate to the arrival
// tangent phi timed to finish at the goal tick. One free parameter
// (psi); a 1-D scan + refine finds the profile landing on the cell.
// The strafe rate limit (one direction change per >=12 ticks) is
// respected by construction: opposite-signed turns require the hold
// between them to cover the flip gap.
//
// The plan is a PER-TICK HEADING PROFILE the certified controller
// flies directly (dense knots). The exact engine remains the judge:
// the caller flies the plan once and accepts only a real strike.

#include <math.h>

#include <vector>

#include "SolverBoard.h"
#include "SolverEnvelope.h"
#include "SolverMove.h"
#include "SolverSteer.h"
#include "SolverStrafe.h"

namespace Solver {
namespace Path {

	struct Plan {
		bool  ok = false;
		int   n = 0;             // flight ticks (the goal's timing)
		float psi = 0.f;         // held heading
		float duty = 1.f;        // gain fraction (the effort dial)
		float end_dist = 1e9f;   // trace endpoint to goal (2D)
		std::vector<float> heading;   // per-tick commanded heading
	};

	// Trace one turn-hold-turn profile. duty = gain fraction (the
	// EFFORT dial - the second degree of freedom: three arrival
	// constraints need two parameters, and partial-gain strafing
	// shortens the path continuously, matching how real flights fly
	// below the full-gain bound). Returns the 2D endpoint distance
	// to the goal; fills prof when non-null.
	inline float TraceTHT(const Vec3& pos, float h0, float s0,
	                      const Vec3& q, float phi, float psi, int n,
	                      const MoveParams& p, bool ducked,
	                      std::vector<float>* prof,
	                      float duty = 1.f) {
		// Final-turn length from the late-speed free rate (speed only
		// grows, so the late rate is the slowest - conservative).
		const float s_late = Envelope::SMax(s0, n, p);
		Strafe::TickLaw ll = Strafe::Law(p, s_late, 1.f, ducked);
		const float wl = ll.TurnRad(0.f, 1.f);
		int m = wl > 1e-5f
			? static_cast<int>(fabsf(Steer::WrapPi(phi - psi)) / wl)
				+ 1
			: n;
		if (m > n)
			m = n;
		// Flip-gap law: opposite-signed turns need the hold between
		// them to cover the strafe rate limit.
		const float turn_a = Steer::WrapPi(psi - h0);
		const float turn_b = Steer::WrapPi(phi - psi);
		if (turn_a * turn_b < 0.f) {
			Strafe::TickLaw le = Strafe::Law(p, s0, 1.f, ducked);
			const float we = le.TurnRad(0.f, 1.f);
			const int ta = we > 1e-5f
				? static_cast<int>(fabsf(turn_a) / we) : n;
			if (n - m - ta < 12)
				return 1e9f;   // flip gap unaffordable
		}
		// Full gain per tick: the controller WEAVES on held headings
		// (side-to-side wish keeping the net heading), so holds gain
		// near-full - measured: the no-gain-on-hold model lost the
		// f0 row that full gain lands. (duty retained in the
		// signature for a future effort channel.)
		(void)duty;
		const float cap2 = p.air_speed_cap * p.air_speed_cap;
		float h = h0, x = pos.X, y = pos.Y, s = s0;
		if (prof)
			prof->resize(n);
		for (int k = 0; k < n; ++k) {
			const float tgt = k < n - m ? psi : phi;
			Strafe::TickLaw lk = Strafe::Law(p, s, 1.f, ducked);
			const float w = lk.TurnRad(0.f, 1.f);
			const float d = Steer::WrapPi(tgt - h);
			h = Steer::WrapPi(h + (d > w ? w : (d < -w ? -w : d)));
			x += cosf(h) * s * p.dt;
			y += sinf(h) * s * p.dt;
			s = sqrtf(s * s + cap2);
			if (prof)
				(*prof)[k] = h;
		}
		const float dx = q.X - x, dy = q.Y - y;
		return sqrtf(dx * dx + dy * dy);
	}

	// The constructed solve: coarse psi scan + refinement. ok when
	// the trace endpoint lands within hull reach of the goal.
	inline Plan Solve(const Vec3& pos, const Vec3& vel, const Vec3& q,
	                  float phi, int n, const MoveParams& p,
	                  bool ducked) {
		Plan best;
		if (n < 4 || n > 400)
			return best;
		const float s0 = Len2D(vel);
		if (s0 < 1.f)
			return best;
		const float h0 = atan2f(vel.Y, vel.X);
		best.n = n;
		const float kPi = 3.14159265f;
		// Second degree of freedom: the ARRIVAL TICK. The ballistic
		// contact is a band (hull height + duck offsets), not an
		// instant - scan n +/- 6 at FULL gain, so every plan is
		// executable exactly as traced (partial-gain plans cannot
		// be flown by heading commands: held headings gain fully).
		int best_n = n;
		for (int dn = -6; dn <= 6; dn += 2) {
			const int n2 = n + dn;
			if (n2 < 4 || n2 > 400)
				continue;
			float step = kPi / 12.f;
			float center = h0;
			float rb = 1e9f, rpsi = center;
			for (int round = 0; round < 4; ++round) {
				rb = 1e9f;
				const int span = round == 0 ? 12 : 3;
				for (int i = -span; i <= span; ++i) {
					const float psi = Steer::WrapPi(center
						+ step * static_cast<float>(i));
					const float d = TraceTHT(pos, h0, s0, q, phi,
						psi, n2, p, ducked, nullptr);
					if (d < rb) {
						rb = d;
						rpsi = psi;
					}
				}
				center = rpsi;
				step *= 0.33f;
			}
			if (rb < best.end_dist) {
				best.end_dist = rb;
				best.psi = rpsi;
				best_n = n2;
			}
		}
		best.n = best_n;
		if (best.end_dist
			<= Board::kHullCenterSlack + 21.f) {
			best.heading.clear();
			TraceTHT(pos, h0, s0, q, phi, best.psi, best.n, p,
				ducked, &best.heading);
			best.ok = true;
		}
		return best;
	}

} // namespace Path
} // namespace Solver
