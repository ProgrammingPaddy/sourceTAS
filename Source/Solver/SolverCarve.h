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
		// Route face index of the tap target. When set, no-strike
		// candidates are scored by their closest approach to the FACE
		// REGION (plane offset + outside-the-polygon shortfall - the
		// same gradient the air primitive's aim_region uses), so the
		// search walks rides toward the whole board window instead of
		// chasing a single point below the crossing band (the dive-
		// and-ground trap).
		int   tap_face = -1;
		// Full exit-velocity target (vel_w > 0 replaces the heading
		// term). This is the honest exit spec: a CREST LAUNCH separates
		// with mostly-vertical velocity, where horizontal heading is
		// ill-conditioned - and stage 3 chains the full vector anyway.
		Vec3  aim_vel;
		float vel_w = 0.f;
		// Terminal tangent tracking for the tap flight (testimony
		// 2.1 as construction). OFF by default (gate rows verify
		// tape reproduction); the solver turns it on.
		bool  terminal_tangent = false;
		// Field-guided pursuit aim (the hotspot map's runway-aware
		// best sample): replaces the centroid as the family aim
		// point when set. GUIDANCE only - families still compete.
		Vec3  field_aim;
		bool  have_field_aim = false;
		// Allow the DUCK-OFF exit move: pressing duck while riding
		// applies the +8.5 air-duck origin shift and pops the hull off
		// the face, separating with the in-plane velocity intact (the
		// solved12 crest exit at frame 470 does exactly this). The
		// press tick is a searched genome dimension.
		bool  try_duck = true;
		// ZONE MODE (the run's LAST transfer): the ride flows through
		// its exit and flight and ends by ENTERING THE END-ZONE VOLUME
		// - scored by arrival tick (the objective is time, nothing
		// else). The volume is the shared InZone proxy. The no-arrival
		// gradient is distance to the volume + ballistic shortfall,
		// so "climb the face to buy altitude" is discoverable.
		bool  to_zone = false;
		Vec3  zone_min, zone_max;
		// ROLLING CONTEXT (testimony: the flick targets the next board
		// SUCH THAT it sets up the one after): the leg-after target.
		// A tap strike is charged the exact climb cost its landing
		// state owes toward this target (v - sqrt(v^2 - 2g*shortfall),
		// pure energy law) - strikes that bottom out below the next
		// leg's reach stop looking cheap.
		int   next_face = -1;
		bool  next_is_zone = false;
	};

	struct Result {
		bool  exited = false;    // left the face into clean air
		int   tick = -1;         // tick index of the first airborne tick
		Vec3  exit_pos, exit_vel;
		float exit_heading = 0.f;
		float speed2d = 0.f;
		int   flips = 0;
		float ride_loss2 = 0.f;  // sum of clip dot^2 over the ride
		float graze_loss2 = 0.f; // sum of clip dot^2 over flight
		                         // GRAZES (non-target contacts that
		                         // continue) - the measured valley-
		                         // crease scrape was -712k of these,
		                         // invisible until accounted
		int   ride_ticks = 0;    // ticks with face contact
		int   struck_brush = -1; // ended by striking something else
		int   struck_plane = -1;
		float strike_dot = 0.f;  // clip dot of that strike (the tap's
		                         // board loss when it IS the transfer)
		bool  grounded = false;
		bool  zoned = false;     // zone mode: entered the end volume
		bool  doomed = false;    // terminated at a separation that
		                         // provably cannot reach the target
		                         // (the exitbench-validated cull)
		Vec3  end_pos;
		float miss_dist = 1e9f;  // closest approach to aim_pos (when
		                         // pos_w > 0): the no-exit gradient
		float reach_short = 0.f; // tap mode, no strike: how far BELOW
		                         // the target face the ballistic
		                         // arrival from the last separation
		                         // sits (M1.1 closed form, min over
		                         // face verts; 0 = reachable). The
		                         // gradient that prices "separate
		                         // higher / ascending".
		float next_cost = 0.f;   // tap strike with next_face/next_is_
		                         // zone set: the exact speed cost of
		                         // the climb the landing still owes
		                         // toward the leg-after target (u/s).
		int   duck_at = -1;      // duck-off press tick used (-1 = none)
		int   family = -1;       // init family that produced the win
		                         // (win-rate data for pruning)
		// The full state at the first airborne tick (= the next air
		// leg's entry). On exit the first `tick` control entries are
		// the ride's frames.
		PlayerState end_state;
		std::vector<float> yaw, fmove, smove;
	};

	// ENGINE-TRUTH EXIT RECORDS (exitbench): when g_exit_rec is set,
	// every evaluated candidate that separates from the ridden face
	// appends its FIRST exit state and its final outcome - the data
	// that grades exit-map models against the exact engine instead of
	// against any reference run.
	struct ExitRec {
		Vec3  xpos, xvel;      // first separation state (the exit)
		int   xtick = 0;
		int   outcome = 0;     // SearchLog::Outcome values
		float e_end = 0.f;     // v^2 + 2g*z at the end of the sim
		float graze2 = 0.f;    // total graze dot^2 after the exit
		int   end_tick = 0;
	};
	extern std::vector<ExitRec>* g_exit_rec;

	// THE DOOM CULL (wired 2026-08-17 after exitbench measured it:
	// 99%+ of dead candidates caught at ~0 false kills): true when a
	// separation state provably cannot reach the transfer's target
	// under the M1.1 bounds (ballistic z-window x gain-law distance,
	// hull-padded - all optimistic, so a kill is a proof). ONE source
	// of truth: the sim terminates on it, exitbench grades it (row
	// Mw). g_doom_cull disables in-sim termination (the bench needs
	// true fates).
	bool ExitDoomed(const Vec3& pos, const Vec3& vel,
	                const MoveParams& p, const Route::Face* tapf,
	                bool to_zone, const Vec3& zmin, const Vec3& zmax);
	extern bool g_doom_cull;

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
	// alts (optional): the top alts_k DIVERSE successful results
	// (tap strikes / zone entries / exits by mode, clustered by end
	// position, best first, alts[0] == the returned best) - the
	// chain composes each with its downstream leg and picks by the
	// WHOLE transfer, same law as the air primitive's board alts.
	Result SolveCarve(const PlayerState& entry, const World& w,
	                  const MoveParams& p, const Route::Graph& g,
	                  const Target& t, int knots_n = 4,
	                  int evals = 2000,
	                  std::vector<Result>* alts = nullptr,
	                  int alts_k = 3);

} // namespace Carve
} // namespace Solver
