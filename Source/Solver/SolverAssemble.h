#pragma once

// M3 - TRANSFER REFINEMENT & ASSEMBLY (Docs/SolverRebuildChecklist.md).
// Chain the M1 primitives along the M2 shape pool into a full run on
// the exact engine, unseeded:
//
//   START plan (prestrafe circle + the single legal jump, chosen by
//   launch speed toward the first board) -> per shape leg: AIR
//   boundary-value solve into the face's board window, then CARVE to
//   an exit aimed at the NEXT leg's geometry (rolling context: the
//   carve's exit spec comes from where the route goes next, the
//   testimony's lookahead) -> final flight to the END ZONE, landing
//   and running the remainder if short.
//
// Every tick of the assembled run came out of MoveTick during the
// solve itself - the frames REPLAY exactly by construction. The zone
// proxy (this era, plugin zones being server-side): entering the
// column above the tape-derived finish brush.

#include <string>
#include <vector>

#include "SolverMove.h"
#include "SolverRoute.h"
#include "SolverTape.h"

namespace Solver {
namespace Assemble {

	struct Opts {
		double wall_budget_s = 170.0;
		int    air_evals = 3000;
		int    carve_evals = 24000;  // the family-dilution law: solve
		                             // quality tracks PER-FAMILY
		                             // budget (probe-measured: the
		                             // 0->2 -162 needs ~500/family =
		                             // 24k at 18 families x 2 domains;
		                             // 12k found only -390s). Prune
		                             // families to bring this down.
		int    max_shapes = 10;
	};

	struct RunResult {
		bool finished = false;
		std::vector<TapeFrame> frames;
		int  zone_tick = -1;         // frame index of end-zone entry
		std::vector<int> shape;
		float board_loss2 = 0.f;     // sum of leg board dot^2
		int  legs_done = 0;          // legs completed (partial chains)
	};

	// The shared zone test (same proxy for solver lines and reference
	// tapes, so comparisons are fair). The finish = ON TOP of the end
	// brush: hull-expanded xy, z from just under the top (no boarding
	// requirement, no side-entry finishes).
	void ZoneVolume(const WorldBrush& eb, Vec3* zmin, Vec3* zmax);
	bool InZone(const World& w, int end_brush, const Vec3& pos);

	// Solve the map: enumerate shapes, assemble each under the budget,
	// return the fastest finisher. partial (optional) receives the
	// deepest assembled chain even when no shape finishes - the
	// playable in-game export for unfinished campaigns.
	bool SolveMap(const World& w, const Route::Graph& g,
	              const MoveParams& p, const TapeAnchor& anchor,
	              const Opts& o, RunResult* out, std::string* err,
	              RunResult* partial = nullptr);

	// Zone-entry tick of an arbitrary frame list from the anchor
	// (-1 = never enters).
	int ZoneTick(const World& w, const Route::Graph& g,
	             const MoveParams& p, const TapeAnchor& anchor,
	             const std::vector<TapeFrame>& frames);

} // namespace Assemble
} // namespace Solver
