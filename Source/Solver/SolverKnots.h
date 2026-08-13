#pragma once

// The knot control vocabulary, shared by the Phase 1 explorer and the Phase 2
// optimizer. A knot is {duration, side L/R/coast, jump/duck flags, ground turn
// rate}; within a knot the air yaw comes from the max-gain controller derived
// from the LIVE MoveParams every tick. Duck presses/releases happen AIRBORNE
// only (ground duck lifecycle is engine behavior the core doesn't carry -
// measured 2026-08-13); the rule is state-dependent, so reconstruction always
// reproduces exploration exactly.

#include "SolverMove.h"
#include "SolverTape.h"

#include <random>
#include <vector>

namespace Solver {

	struct Knot {
		short dur = 14;             // ticks (flip spacing guarded separately)
		signed char side = 0;       // +1 = A/left, -1 = D/right, 0 = coast
		unsigned char flags = 0;    // 1 = jump on first tick, 2 = duck held
		float ground_turn = 2.f;    // deg/tick yaw drift while grounded
	};

	float NormYawDeg(float y);

	// Optimal air-strafe yaw for the side, derived from live params.
	float MaxGainYaw(const Vec3& vel, int side, bool ducked,
	                 const MoveParams& p);

	// One knot-driven tick: computes the tick's inputs from the knot + state,
	// advances the state, optionally emits the frame.
	void KnotTick(PlayerState& s, float& yaw, const Knot& k, int tick_in_knot,
	              const World& w, const MoveParams& p, TickEvents* ev,
	              TapeFrame* emit);

	// The explorer's knot distribution (kept in one place so the optimizer's
	// resampling stays in-language with discovery).
	Knot SampleKnot(std::mt19937& rng, signed char last_side, short since_flip,
	                int min_knot);

	// Structural flip legality over a knot list: every L/R sign change must be
	// at least min_knot ticks after the previous one (coasts don't reset the
	// clock). last_side0/since_flip0 describe the state at the list's start.
	bool FlipsLegal(const std::vector<Knot>& knots, int min_knot,
	                signed char last_side0, short since_flip0);

} // namespace Solver
