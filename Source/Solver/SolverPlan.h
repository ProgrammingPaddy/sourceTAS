#pragma once

// THE CONSTRUCTED PLANNER (v2 solve loop, user 2026-08-17: "the state
// space is defined by the heatmaps. The routing is a closed equation,
// it just needs to be searched" - the legacy emergent-tap valuation is
// PARKED, the certified engine and laws are untouched).
//
// Per leg, no weight soup anywhere:
//   BOARD: candidate cells come from the board heatmap (hot first,
//     then warm; cold is not a candidate). Each cell carries its
//     tangent arrival heading and a LAW-DERIVED dot cap - the largest
//     loss that still keeps the warm ratio of the map's best energy
//     (cap^2 = e_eff + residual^2 - 0.6*e_hi).
//   FLIGHT: a boundary-value solve per cell (the gated air primitive:
//     arrive AT the cell under its cap) - multiple aerial solutions
//     per cell are kept and ranked by DELIVERED ENERGY, nothing else.
//   RIDE: departures come from the exit map (ride bookkeeping); each
//     is solved by the gated carve primitive as an exit spec, ranked
//     by energy at the exit.
//   ENDING: the zone-mode carve (its objective is already the clock).
//
// Anything that cannot board within its cell's cap is not a board and
// does not exist as a candidate.

#include <string>

#include "SolverAssemble.h"
#include "SolverMove.h"
#include "SolverRoute.h"
#include "SolverTape.h"

namespace Solver {
namespace Plan {

	struct Opts {
		int    cells_per_leg = 16;   // distinct target cells per board
		int    exits_per_leg = 6;    // departures tried per ride
		int    air_evals = 1500;     // per cell boundary-value budget
		int    ride_evals = 6000;    // per departure ride budget
		int    zone_evals = 24000;   // ending budget
		double wall_budget_s = 170.0;
		int    max_shapes = 8;
	};

	bool SolveMap(const World& w, const Route::Graph& g,
	              const MoveParams& p, const TapeAnchor& anchor,
	              const Opts& o, Assemble::RunResult* out,
	              std::string* err,
	              Assemble::RunResult* partial = nullptr);

} // namespace Plan
} // namespace Solver
