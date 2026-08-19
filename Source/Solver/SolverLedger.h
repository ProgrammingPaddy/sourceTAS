#pragma once

// M1.5 - THE LEDGER (Docs/SolverRebuildChecklist.md, design #4).
// A per-phase regret readout for ANY control line replayed on the
// exact engine. Phases: GROUND / AIR / RIDE(face). Every number is
// event-sourced from the certified sim; regrets are measured against
// the PROVEN laws, never against vibes:
//
//  AIR:  horizontal gain2 vs the strafe-law maximum 900/tick
//        -> air-gain shortfall (paid for turns - the route decides if
//           a turn was worth its price; the ledger prices it).
//  BOARD (first clip of a ride arriving from air): loss = dot^2,
//        fraction of |v1|^2, and the tangency minimum possible AT THAT
//        ARRIVAL (s, vz fixed): board regret = dot^2 - min^2.
//  RIDE: energy closure  v2_out + sum(dot^2) = v2_in + 2g*drop + W
//        -> gravity conversion, clip dissipation, wish work, each
//           exact within the derived half-gravity cross bound.
//  Totals: closure residual per phase (must sit inside the bound - the
//        ledger AUDITS ITSELF against the engine every time it runs).

#include <vector>

#include "SolverMove.h"
#include "SolverRoute.h"
#include "SolverTape.h"

namespace Solver {
namespace Ledger {

	enum class Kind { Ground, Air, Ride };

	struct Phase {
		Kind  kind = Kind::Air;
		int   t0 = 0, t1 = 0;      // inclusive tick range
		int   face = -1;           // rides: Route face index (-1 = none)
		float v2_in = 0.f, v2_out = 0.f;    // |v|^2 entering / leaving
		float s2d_in = 0.f, s2d_out = 0.f;  // horizontal speeds
		float z_in = 0.f, z_out = 0.f;
		// AIR
		float gain2 = 0.f;         // s2d_out^2 - s2d_in^2
		float max_gain2 = 0.f;     // 900 * ticks
		float shortfall = 0.f;     // max_gain2 - gain2 (>= 0 by law)
		// BOARD (rides entered from air)
		float board_dot = 0.f;     // the clip dot (negative)
		float board_dot2 = 0.f;
		float board_frac = 0.f;    // dot^2 / |v1|^2
		float board_min2 = 0.f;    // tangency minimum at that arrival
		// RIDE
		float ride_dot2 = 0.f;     // sum of clip dot^2 (incl. board)
		float grav_conv = 0.f;     // 2g * (z_in - z_out)
		float wish_work = 0.f;     // closure: v2_out+dots-v2_in-grav
		float cross_bound = 0.f;   // derived closure tolerance
		int   Ticks() const { return t1 - t0 + 1; }
	};

	struct Run {
		std::vector<Phase> phases;
		float total_dissipation = 0.f;  // sum of ride dot^2
		float total_shortfall = 0.f;    // sum of air shortfalls
		int   ticks = 0;
	};

	// Replay the tape on the exact engine and build the ledger.
	// sabotage_t0 < sabotage_t1: zero fmove/smove in [t0, t1) - the
	// gate's planted-loss probe.
	bool Build(const World& w, const Route::Graph& g, const MoveParams& p,
	           const Tape& tape, Run* out,
	           int sabotage_t0 = -1, int sabotage_t1 = -1);

	void Print(const Run& run, int detail_top_n);

} // namespace Ledger
} // namespace Solver
