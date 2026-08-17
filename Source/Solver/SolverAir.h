#pragma once

// M1.3 - THE AIR-PHASE PRIMITIVE (Docs/SolverRebuildChecklist.md).
// A transfer's air phase, solved as a boundary-value problem: from an
// airborne ENTRY state, steer the exact engine into a target BOARD
// WINDOW (face + aim point, near-tangent arrival), unseeded.
//
// Controls collapse to a TARGET-HEADING SPLINE theta(t) (the design of
// record, Docs/SolverRebuild.md #3): per tick the controller wishes at
// the certified strafe law's optimal angle toward the spline heading,
// backing off along the cosa in [0, cap/v] family (full-turn-full-gain
// -> no-turn-no-gain, the law's own tradeoff line) to land headings
// exactly. Strafe SIDE changes are rate-limited by construction to
// MoveParams::strafe_rate_max (the ONE aesthetic rule) - a blocked
// flip coasts (zero wish) instead.
//
// Vertical motion stays pure ballistic (M1.1): the primitive never
// ducks; arrival time is emergent from the flight, not scheduled.

#include <vector>

#include "SolverMove.h"
#include "SolverRoute.h"

namespace Solver {
namespace Air {

	struct Target {
		// Terminal tangent tracking (testimony 2.1 as construction):
		// when contact is ballistically imminent the commanded
		// heading becomes the closed-form tangent-arrival heading.
		// OFF by default - the gate rows verify tape REPRODUCTION,
		// which forced-optimal arrivals intentionally diverge from;
		// the solver turns it on.
		bool  terminal_tangent = false;
		int   face = -1;        // Route::Graph face index
		Vec3  aim;              // point on (or near) the face to reach
		float dot_cap = 100.f;  // success ceiling on arrival |dot|
		int   max_ticks = 200;  // flight horizon
		// Region mode (unseeded assembly): the miss gradient pulls
		// toward the NEAREST point of the face, not the aim point -
		// point pursuit forces head-on arrivals and fights tangency;
		// the aim then only seeds initial bearings.
		bool  aim_region = false;
		// Optional arrival-time preference (the route plan owns timing;
		// tick_w = 0 ignores it). aim_tick also SCALES THE SPLINE: knots
		// span [0, aim_tick] so every knot is a live parameter of the
		// expected flight, not of the sim cap. tick_tol >= 0 makes the
		// window hard (arrivals outside it score as misses).
		int   aim_tick = -1;
		float tick_w = 0.f;
		int   tick_tol = -1;
	};

	struct Result {
		bool  hit = false;      // single-plane strike on the target face
		int   tick = -1;        // MoveTicks executed up to the strike
		float dot = 0.f;        // arrival clip dot (negative = into face)
		float edge = 0.f;       // in-plane distance outside the polygon
		float speed = 0.f;      // |v1| at the clip
		float speed2d = 0.f;    // horizontal speed at the clip
		Vec3  pos;              // contact position (hull center)
		Vec3  v1;               // velocity entering the clip
		int   flips = 0;        // strafe side changes used
		float miss_dist = 1e9f; // closest approach to aim (diagnostics)
		int   struck_brush = -1; // on miss: what ended the flight
		bool  grounded = false;  // on miss: landed on walkable ground
		Vec3  end_pos;           // where the flight ended (diagnostics)
		// The full state after the strike tick (= the carve's entry;
		// the board tick belongs to this flight's last frame).
		PlayerState end_state;
		// The realized controls (for assembly/export; on a hit the
		// first `tick` entries are the flight's frames).
		std::vector<float> yaw;
		std::vector<float> fmove;
		std::vector<float> smove;
	};

	// Fly the exact engine along a heading spline. knots = target
	// headings (radians) at equal fractions of [0, horizon]. Returns the
	// first-contact outcome; any non-target strike or grounding is a
	// miss.
	Result FlyHeadingSpline(const PlayerState& entry, const World& w,
	                        const MoveParams& p, const Target& t,
	                        const Route::Graph& g,
	                        const std::vector<float>& knots, int horizon);

	// The unseeded boundary-value solve: geometric initial spline (the
	// closed-form tangent-arrival heading from M1.1 ballistics + M1.2
	// dot line), then pattern search on knot values against the exact
	// engine. evals bounds the number of simulated flights.
	// alts (optional): the top alts_k DIVERSE hits (deduped by strike
	// region, best first - alts[0] == the returned best). A board that
	// minimizes its own dot can sabotage the ride it feeds (measured:
	// the same face's soft-tangent board sets up a -400 transfer where
	// a harder board sets up -160); the CALLER composes each board
	// with its downstream carve and picks by the whole transfer.
	Result SolveTransfer(const PlayerState& entry, const World& w,
	                     const MoveParams& p, const Route::Graph& g,
	                     const Target& t, int knots_n = 4,
	                     int evals = 400,
	                     std::vector<Result>* alts = nullptr,
	                     int alts_k = 3);

} // namespace Air
} // namespace Solver
