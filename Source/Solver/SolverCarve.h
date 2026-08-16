#pragma once

// M1.4 - THE CARVE PRIMITIVE (Docs/SolverRebuildChecklist.md).
// The on-face phase of a transfer: from an ENTRY state riding a surf
// face (the post-board state), steer the exact engine along the face
// and EXIT into clean air with a chosen heading - the carve's three
// jobs from the testimony (convert energy along the downhill axis,
// position the exit, set up the flick).
//
// Controls are the same target-heading spline + certified controller
// as the air primitive (SolverSteer.h) - carving IS airborne movement
// pressed onto the plane by gravity; the per-tick clip keeps velocity
// in-plane and DISSIPATES dot^2 of speed^2 per contact (recorded in
// the result: the ride's energy ledger is
//   v2_exit = v2_entry + 2g*(z_entry - z_exit) - sum(dot^2) + wish work
// which the M1.4 gate verifies against the engine).
//
// Exit = three consecutive contact-free ticks (brief separations are
// part of a ride); the exit STATE is the first airborne tick, matching
// the air primitive's entry convention so stage 3 chains them.

#include <vector>

#include "SolverMove.h"
#include "SolverRoute.h"

namespace Solver {
namespace Carve {

	struct Target {
		int   face = -1;         // the ridden face (Route::Graph index)
		float exit_heading = 0.f; // desired horizontal heading at exit
		int   max_ticks = 300;   // ride horizon
		// Exit-time preference (the route plan owns timing).
		int   aim_tick = -1;
		float tick_w = 0.f;
		int   tick_tol = -1;
		// Optional exit-position preference.
		Vec3  aim_pos;
		float pos_w = 0.f;
		// A strike on THIS brush/side is a TAP TRANSFER, not a
		// failure: adjoining faces are boarded straight off the ride
		// (solved12's multi-taps) - scored by the tap's own clip loss.
		int   tap_brush = -1;
		int   tap_side = -1;
		// Full exit-velocity target (vel_w > 0 replaces the heading
		// term). This is the honest exit spec: a CREST LAUNCH separates
		// with mostly-vertical velocity, where horizontal heading is
		// ill-conditioned - and stage 3 chains the full vector anyway.
		Vec3  aim_vel;
		float vel_w = 0.f;
		// Allow the DUCK-OFF exit move: pressing duck while riding
		// applies the +8.5 air-duck origin shift and pops the hull off
		// the face, separating with the in-plane velocity intact (the
		// solved12 crest exit at frame 470 does exactly this). The
		// press tick is a searched genome dimension.
		bool  try_duck = true;
	};

	struct Result {
		bool  exited = false;    // left the face into clean air
		int   tick = -1;         // tick index of the first airborne tick
		Vec3  exit_pos, exit_vel;
		float exit_heading = 0.f;
		float speed2d = 0.f;
		int   flips = 0;
		float ride_loss2 = 0.f;  // sum of clip dot^2 over the ride
		int   ride_ticks = 0;    // ticks with face contact
		int   struck_brush = -1; // ended by striking something else
		int   struck_plane = -1;
		float strike_dot = 0.f;  // clip dot of that strike (the tap's
		                         // board loss when it IS the transfer)
		bool  grounded = false;
		Vec3  end_pos;
		float miss_dist = 1e9f;  // closest approach to aim_pos (when
		                         // pos_w > 0): the no-exit gradient
		int   duck_at = -1;      // duck-off press tick used (-1 = none)
		// The full state at the first airborne tick (= the next air
		// leg's entry). On exit the first `tick` control entries are
		// the ride's frames.
		PlayerState end_state;
		std::vector<float> yaw, fmove, smove;
	};

	// effort: optional per-knot duty cycle in [0,1] (same knot spacing
	// as the heading spline). A carve at effort < 1 COASTS a fraction
	// of its ticks (zero wish): the expert's speed control - slow rides
	// and setup carves are effort choices, not heading choices. Null =
	// full effort.
	// duck_at >= 0: hold IN_DUCK from that tick on (the duck-off).
	Result RideHeadingSpline(const PlayerState& entry, const World& w,
	                         const MoveParams& p, const Target& t,
	                         const Route::Graph& g,
	                         const std::vector<float>& knots,
	                         const std::vector<float>* effort,
	                         int horizon, int duck_at = -1);

	// Unseeded boundary-value solve: initial spline families (hold /
	// linear-to-exit / late-turn / downhill-bulge) + basin hopping on
	// the exact engine.
	Result SolveCarve(const PlayerState& entry, const World& w,
	                  const MoveParams& p, const Route::Graph& g,
	                  const Target& t, int knots_n = 4,
	                  int evals = 2000);

} // namespace Carve
} // namespace Solver
