#pragma once

// Phase 2: the route optimizer. Takes a finishing genome (seed-prefix length
// + knot list, flattened from an explorer chain) and hill-climbs it under the
// era's contract: fitness = FINISH TICK, nothing else; legality structural
// (flip spacing via FlipsLegal, airborne-only duck transitions inside
// KnotTick, startzone jump budget enforced in every evaluation); every
// evaluation is a full re-roll through the capture-proven core; a mutation
// survives only when its route finishes STRICTLY earlier. Evaluations abort
// the moment they reach the incumbent tick, so failed candidates are cheap.
//
// Operators: knot trims, knot deletion, suffix resampling (the explorer's own
// distribution), parameter jitter, knot growth, and seed-prefix erosion
// (branching earlier into a seeded route's human prefix - the optimizer eats
// backward into the tape).

#include "SolverExplore.h"
#include "SolverKnots.h"

#include <random>
#include <vector>

namespace Solver {

	struct OptimizeConfig {
		MoveParams params;
		int start_brush_idx = -1;   // World::brushes index (startzone rule)
		int end_brush_id = -1;      // BSP id (finish predicate)
		int end_brush_idx = -1;     // World::brushes index (footprint)
		int max_zone_jumps = 1;
		int min_knot = 14;
		int max_path_ticks = 4000;
		double budget_seconds = 60.0;
		unsigned rng_seed = 1;
	};

	struct OptimizeResult {
		bool ok = false;            // the input genome evaluated to a finish
		int initial_tick = 0;
		int best_tick = 0;
		long long evals = 0;
		long long ticks_simulated = 0;
		int improvements = 0;
		double seconds = 0.0;
	};

	class Optimizer {
	public:
		Optimizer(const World& w, const OptimizeConfig& cfg,
		          const PlayerState& root, float root_yaw,
		          const Tape* seed_tape);

		// Improves g in place; returns stats (ok=false when g never finishes).
		OptimizeResult Improve(Explorer::FlatGenome& g);

		// Roll a genome into engine-replayable frames, truncated AT the
		// finish tick. False when the genome doesn't finish.
		bool BuildFrames(const Explorer::FlatGenome& g,
		                 std::vector<TapeFrame>& out, int* finish_tick);

	private:
		struct Eval {
			bool finished = false;
			int tick = 0;
		};
		Eval Run(const Explorer::FlatGenome& g, int abort_at,
		         std::vector<TapeFrame>* emit, long long* ticks);
		bool InsideStartZone(const Vec3& p) const;
		void SeedFlipState(int prefix, signed char* side, short* since) const;

		const World& w_;
		OptimizeConfig cfg_;
		PlayerState root_;
		float root_yaw_ = 0.f;
		const Tape* seed_;
	};

} // namespace Solver
