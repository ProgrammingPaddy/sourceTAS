#pragma once

// The per-tick AIR-STRAFE LAW, closed form, derived from the certified
// AirAccelerate mirror (see SolverMove.cpp air path; engine parity:
// Docs/EngineParityReference.md). This is the mathematical bedrock of the
// rebuild's regret ledger (Docs/SolverRebuild.md #4): approach loss and
// air-gain shortfall are both measured against these bounds.
//
// Engine mechanics (SDK AirAccelerate, parity-certified):
//   wishspd  = min(|wishvel|, air_speed_cap)            // the "30 cap"
//   current  = dot(v_xy, wishdir)
//   addspeed = wishspd - current            (<= 0: no change)
//   budget   = airaccelerate * |wishvel| * dt * sfric   // UNCAPPED wishspeed
//   a        = min(addspeed, budget);  v_xy += a * wishdir
//
// With a full stick (|wishvel| = maxspeed 250, or 85 fully ducked) the
// budget is 562.5*sfric standing / 47.8 ducked-rising - ALWAYS above the
// 30 cap, so the tick is CAP-LIMITED in every legal regime and the law is:
//
//   let c = v * cos(alpha)      (alpha = angle wish-from-velocity)
//   a = cap - c   (when positive)
//   |v'|^2 = v^2 + (cap - c)(cap + c) = v^2 + cap^2 - c^2
//
//   => the optimum wish is PERPENDICULAR to velocity (c = 0):
//        max gain per tick:  |v'|^2 - v^2 = cap^2      (900)
//   => biasing the wish by beta off perpendicular (c = v sin beta):
//        gain^2 shortfall = (v sin beta)^2   - QUADRATIC in bias
//        heading rotation  = atan2(a sin alpha, v + a cos alpha) - LINEAR-ish
//   Speed trades quadratically for curvature linearly: the entire
//   approach-arc economics of surfing in one line.

#include "SolverMove.h"

namespace Solver {
namespace Strafe {

	struct TickLaw {
		float cap;      // effective per-tick projection cap (30)
		float budget;   // accel * wishspeed * dt * sfric
		float v;        // current horizontal speed

		// Accel magnitude applied for a wish at cos(alpha) = c/v.
		float Accel(float cosa) const {
			const float add = cap - v * cosa;
			if (add <= 0.f) return 0.f;
			return add < budget ? add : budget;
		}
		// Squared-speed after the tick for a wish at angle alpha.
		float NewSpeed2(float cosa) const {
			const float a = Accel(cosa);
			return v * v + 2.f * v * a * cosa + a * a;
		}
		// Heading rotation (radians, toward the wish side) for angle alpha.
		float TurnRad(float cosa, float sina) const {
			const float a = Accel(cosa);
			return atan2f(a * sina, v + a * cosa);
		}
		// The optimum: perpendicular wish in the cap-limited regime, else
		// the budget boundary v*cosa = cap - budget.
		float OptCos() const {
			if (budget >= cap || v <= 0.f) return 0.f;
			float c = (cap - budget) / v;
			return c > 1.f ? 1.f : c;
		}
		float OptGain2() const {
			const float c = OptCos() * v;
			const float a = Accel(OptCos());
			return 2.f * c * a + a * a;
		}
	};

	// Build the law for a state. ducked scales wishspeed by duck_speed_frac
	// (the crop, Docs/SolverRebuild.md #2.2 note on strafe rate).
	inline TickLaw Law(const MoveParams& p, float speed, float sfric,
	                   bool ducked) {
		TickLaw l;
		l.cap = p.air_speed_cap;
		const float wishspeed = ducked
			? p.maxspeed * p.duck_speed_frac : p.maxspeed;
		l.budget = p.airaccelerate * wishspeed * p.dt * sfric;
		l.v = speed;
		return l;
	}

} // namespace Strafe
} // namespace Solver
