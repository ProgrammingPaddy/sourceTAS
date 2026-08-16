#pragma once

// M2 - ROUTE SEARCH (Docs/SolverRebuildChecklist.md).
// Stage 2 of the design: enumerate candidate FEATURE SEQUENCES
// (START -> faces... -> END) ranked by an ADMISSIBLE lower bound on
// ticks, with edges gated by the analytic reachability physics from
// M1 (ballistic z windows + gain-law distance bounds). No sim runs
// here - closed forms only; stage 3 refines the survivors on the
// exact engine.
//
// Edge bound A -> B (M2.1): interval arithmetic over the exit family
// of A (z extent, in-plane vz range at the carried speed bound) and
// the board window of B (z extent + hull slack): the smallest n with
//   z0 + n*dt*vz0 - g*dt^2*n^2/2  intersecting  B's z window, and
//   dist_xy(A, B) <= DMax(s_ub, n).
// Energy carries optimistically (UB): arrivals gain <= 900/tick, rides
// credit the face's full gravity drop. Optimistic energy NEVER falsely
// prunes; the ranking is by the tick LB alone.
//
// START: prestrafe ceiling MEASURED from the certified sim (probe
// circle-strafes on the platform - engine-derived, no fitted number)
// + the single legal jump. Jump edges elsewhere: not modeled in M2
// (exceptional moves, SolverRebuild.md #1).

#include <string>
#include <vector>

#include "SolverMove.h"
#include "SolverRoute.h"

namespace Solver {
namespace RouteSearch {

	struct Opts {
		int   max_len = 7;          // max ride nodes in a route
		int   top_k = 10;
		int   max_edge_ticks = 300; // per-edge flight horizon
		int   revisit_cap = 2;      // per-face visit budget
		float prestrafe_speed = 0.f;// 0 = measure via probe
		bool  verbose = false;      // print edge-bound diagnostics
	};

	struct Candidate {
		std::vector<int> faces;     // ride sequence (graph indices)
		float lb_ticks = 0.f;       // admissible time lower bound
		float end_s2_ub = 0.f;      // optimistic arrival energy at END
	};

	// Measured prestrafe ceiling: max ground speed the certified sim
	// reaches on the start platform over a family of circle-strafe
	// probes, x1.15 margin (a measured max is a lower estimate of the
	// true optimum; the margin keeps the bound safe for pruning).
	float MeasurePrestrafeCeiling(const World& w, const MoveParams& p,
	                              const Vec3& start_pos, bool ducked);

	bool Enumerate(const World& w, const Route::Graph& g,
	               const MoveParams& p, const Opts& o,
	               std::vector<Candidate>* out, std::string* err);

} // namespace RouteSearch
} // namespace Solver
