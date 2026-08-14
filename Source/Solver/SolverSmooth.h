#pragma once

// LINE-SPACE OPTIMIZER (paradigm shift, user-directed 2026-08-14).
//
// Verdict on the knot paradigm: knots are a BANG-BANG controller. Every air
// tick strafes at exactly max-gain, so the turn rate is dictated by physics
// (~1719/speed deg/tick) and the only control authority is WHEN to flip
// sides. The efficient line - velocity arriving parallel to the next face,
// not just the view - needs CONTINUOUS modulation of how hard you strafe,
// trading gain for curvature. That family is unrepresentable in knot space:
// the archive ranked 100M rollouts of the same jittery class because the
// generator never proposed anything else (valuation audit 2026-08-14:
// mid-map bands unreachable-class). Results define conditions: the generator
// was the bottleneck, so the generator is what changes.
//
// The new control language: u(t), a Catmull-Rom spline over control points
// every cp_ticks ticks (>= 14 keeps flips <= 5/s STRUCTURALLY).
//   u = +-1  : max-gain strafe (the proven controller, phi = 90 deg)
//   |u| < 1  : partial strafe - the wish projection eats (1-|u|) of the cap,
//              so the per-tick speed ADD is linear in |u| and the turn is
//              gentler; u ~ 0 coasts (no key)
//   |u| > 1  : over-rotated wish (phi > 90) - a carve: harder turn, spends
//              speed. Real technique, capped at phi = 90 + overdrive_deg.
//   sign(u)  : strafe key side. One JUMP gene (tick, single press) covers
//              the startzone hop; the zone budget holds structurally.
// On the ground u arcs the yaw (prestrafe); in the air the view yaw is the
// engine's strafe-key geometry: yaw = heading + side*(phi - 90), phi from
// the LIVE cap and speed each tick - no hardcoded physics.
//
// Search: CMA-ES over the spline coefficients + jump gene. Full-run rollouts
// through the capture-proven core; evaluations are deterministic, so the
// whole optimizer reproduces at a fixed seed REGARDLESS of thread count
// (unlike the archive explorer). Objective (minimize):
//   clean finish:  scored ticks  - 1e5   (class bonus; eloss tie-break)
//   jump finish:   scored ticks  + 5e3   (never competitive with clean)
//   no finish:     1e4 + closest-approach/10 + wloss * eloss-at-approach
// The eloss shaping reuses the human-fitted loss weight (lossfit.py). The
// clean/jump classes are the user's acceptance rule, not a cull: everything
// is still simulated, ranked, and reported.

#include "SolverMove.h"
#include "SolverTape.h"

#include <string>
#include <vector>

namespace Solver {

	struct SmoothConfig {
		MoveParams params;
		int start_brush_id = -1;    // startzone platform (BSP id); -1 = none
		int end_brush_id = -1;      // finish platform (BSP id); REQUIRED
		int max_ticks = 1200;       // absolute rollout cap = spline domain
		int cp_ticks = 16;          // control-point spacing (clamped >= 14:
		                            // the 5 flips/sec rule, structural)
		int pop = 0;                // CMA population (0 = auto)
		double budget_seconds = 60.0;
		unsigned rng_seed = 1337;
		int threads = 0;            // eval workers; 0 = auto
		bool zone_clock = true;     // score = ticks since startzone exit
		float wloss = 6.5f;         // shaping weight on dissipation (fitted)
		float mix_mu = 0.4f;        // PE weight in the approach-value term
		                            // (the fitted human mix, lossfit.py)
		float overdrive_deg = 35.f; // wish-angle overshoot span for |u|>1
		int stall_gens = 400;       // restart after this many flat generations
		int max_restarts = 0;       // return early after N restarts (0 = run
		                            // the full budget; segments use 2)
		double sigma0 = 0.5;        // cold-start step size (u units)
		double sigma_seed = 0.15;   // seeded-polish step size
		// Follow the seed tape's frames VERBATIM for the first N ticks; the
		// spline takes over after. Turns the search into a boundary-value
		// polish from a checkpoint on a proven line (erode N to widen).
		int seed_follow = 0;
		// Carry the seed tape's per-tick duck buttons as a fixed overlay
		// (duck is not a gene yet; the human's pumps are part of the line).
		bool seed_duck = true;
		// SEGMENT MODE (the chain solver's building block): when
		// goal_face_brush >= 0, success = CONTACT with that face (brush
		// INDEX + plane index; plane -1 = any plane of the brush) instead
		// of grounding on the end platform.
		int goal_face_brush = -1;
		int goal_face_plane = -1;
		// Fitness: 0 = clean-finish scored ticks (the full-map contract);
		// 1 = junction value: maximize V = KE + mu*g*z - wloss-scaled waste
		// at the board, minus w_tick per tick spent (the user's compromise
		// equation, clause 1+2: board the next face with max energy having
		// left the previous cleanly).
		int fitness_mode = 0;
		float w_tick = 500.f;
		// Segment mode: ride N more ticks after the touch before handing
		// the junction off - the state passed onward is an ESTABLISHED
		// ride, not a corner clip; V is measured settled. A rollout that
		// dies during the settle window loses its touch (self-filtering).
		int touch_settle = 0;
		// CHAIN GENES: every chain segment (face AND end-landing) carries
		// 4 guidance genes (board/landing point, gain, release tick) and
		// the STRUCTURAL RIDE HOLD - while steep-face contact is fresh and
		// the release gene hasn't fired, u is forced INTO that face (the
		// spline modulates intensity). Measured: cold splines cannot hold
		// a ride (mean-zero u slides off before homing engages; children
		// stalled 278-492u from their targets).
		bool chain_genes = false;
		// Junction VALUE mix: on a ramp, riding makes PE and KE fungible,
		// so a junction's reach is its TOTAL energy (mu=1) minus waste.
		// (Measured with the route-quality mix 0.4: the "best board" of
		// ramp 2 was a 982 u/s BOTTOM-RIM graze at z=-68, outside the
		// face polygon, descending - altitude was priced too cheap for
		// the chain's purpose.) Separate knob from mix_mu on purpose.
		float chain_mu = 1.0f;
		// Board-point band: the ty gene maps into [ty_lo, ty_hi] of the
		// face's up-slope axis. The chain enumerates height bands per edge
		// (clause 3 of the user's compromise equation - a low-fast board
		// and a high-slow board are DIFFERENT reach options; total energy
		// is conserved in flight, so a single V pick collapses them).
		float ty_lo = 0.f;
		float ty_hi = 1.f;
		// END-landing aim margin: the guidance's aim plane sits this far
		// ABOVE the platform top, so flights arc OVER the lip and descend
		// into the footprint (measured d31 plateau: aiming AT the top
		// plane is satisfied by grazing the near lip with zero margin -
		// the under-lip family). Search knob, not physics.
		float land_margin = 40.f;
	};

	// Per-rollout observables (reported, and the objective's inputs).
	struct SmoothStats {
		bool finished = false;
		bool touched = false;       // segment mode: target face contacted
		bool clean = true;          // ending class (last departure not a jump)
		int tick = 0;               // absolute finish/touch tick
		int rel = 0;                // scored ticks (since zone exit)
		int exit_tick = -1;
		float vboard = 0.f;         // junction value at the touch
		PlayerState end_state;      // state AT the stop tick (junction pass)
		float end_yaw = 0.f;
		float dmin = 1e9f;          // closest approach to the landing box
		float eloss = 0.f;          // cumulative energy dissipated
		float eloss_at_dmin = 0.f;  // dissipation up to the closest approach
		float vmix_at_dmin = 0.f;   // KE + mu*g*z at the closest approach -
		                            // orders equal-dmin states by the energy
		                            // they bring (the chain signal: a fast
		                            // under-lip kiss is CLOSER to landing
		                            // than a slow one at the same distance)
		float finish_speed = 0.f;
		float max_impact = 0.f;     // largest one-tick clip loss (u/s)
		int nlandings = 0;
		int sim_ticks = 0;
	};

	struct SmoothResult {
		bool ok = false;            // some finisher exists
		bool clean = false;
		int best_tick = 0;
		int best_rel = 0;
		long long evals = 0;
		long long ticks_simulated = 0;
		int generations = 0;
		int restarts = 0;
		double seconds = 0.0;
		std::vector<double> best_x; // winning genome (spline CPs + jump gene)
		SmoothStats best_stats;
	};

	class SmoothOpt {
	public:
		SmoothOpt(const World& w, const SmoothConfig& cfg,
		          const TapeAnchor& anchor);

		// Chain segments start mid-air mid-map: override the root state
		// (skips the anchor ground settle).
		void SetRoot(const PlayerState& s, float yaw);
		// Silence the per-generation narration (chain runs many small
		// solves; the chain narrates at the node level instead).
		void SetQuiet(bool q) { quiet_ = q; }

		// Initialize the CMA mean by INVERTING the tape's controls into u(t):
		// a full core replay recovers each tick's regime - ground u = yaw
		// rate / arc rate; air u = the wish-angle map inverted against the
		// LIVE velocity heading - then CPs are window means of u(t). Also
		// captures the duck overlay, the exact jump tick, and the frames for
		// prefix-follow mode.
		bool SeedFromTape(const Tape& tape);

		SmoothResult Run();

		// Decode a genome into engine frames, truncated at the finish tick
		// (or the full domain when it never finishes).
		bool BuildFrames(const std::vector<double>& x,
		                 std::vector<TapeFrame>& out, SmoothStats* stats);

		// Deterministic rollout; returns the objective value.
		double Evaluate(const std::vector<double>& x, SmoothStats* stats,
		                std::vector<TapeFrame>* emit) const;

		// Genome = spline CPs + jump tick + duck-on tick + duck-off tick
		// (tick genes in 10-tick units; negative = disabled). The duck pair
		// is the pump: an airborne unduck lifts the origin duck_air_shift
		// (8.5u measured) - the human's lip-clearing tool, now a decision
		// variable instead of a stale overlay.
		// FACE-TARGET segments append 4 GUIDANCE genes: (tx, ty) = the
		// board point on the face rectangle, g = guidance gain, and
		// guide_from = the tick homing engages (10-tick units) - the
		// RELEASE decision: before it the spline rides the previous face,
		// after it the decoder blends in a closed-loop u that steers the
		// ballistic-predicted plane crossing onto the board point. This is
		// position control the open-loop spline cannot do (measured: cold
		// segments never found a face without it; always-on homing broke
		// rides at tick 0). The guided u passes through the SAME wish
		// mapping, flip guard, and yaw cap - structure rules hold.
		// Chain gene 5 (carve): the ride's wish direction blends from pure
		// into-face (carve 0) toward along-face travel (|carve| -> 1, sign
		// = which end). User calibration 2026-08-14: the fastest line
		// LAUNCHES OFF THE SIDE of the last ramp - the ride must be able
		// to traverse, not just climb.
		int Dims() const {
			return ncp_ + 3 + (cfg_.chain_genes ? 5 : 0);
		}

		// The seeded init mean (empty before SeedFromTape) - lets the lab
		// evaluate/inspect the PROJECTION itself, the thing CMA polishes.
		const std::vector<double>& SeedMean() const { return seed_mean_; }

	private:
		float SplineEval(const std::vector<double>& x, int t) const;
		// ENERGY-AWARE distance to the END LANDING: clamped-box XY to the
		// hull-expanded top footprint; the vertical term below the lip is
		// the ENERGY SHORTFALL height max(0, (ztop-z) - v^2/2g) - a fast
		// under-lip state reads near, a zero-speed lob at the same spot
		// reads its full shortfall. Kills the measured lob-at-the-lip trap
		// (best non-finisher apexed 6u short at SPEED 19) with pure physics
		// - an optimistic (admissible) reachability bound, no map rules.
		float DistToEnd(const Vec3& p, float v2) const;

		const World& w_;
		SmoothConfig cfg_;
		PlayerState root_;
		float root_yaw_ = 0.f;
		int ncp_ = 0;
		int start_idx_ = -1, end_idx_ = -1;
		Vec3 end_center_;
		Vec3 face_center_;          // segment mode: target-face shaping point
		Vec3 face_n_;               // target-face frame (guidance)
		Vec3 face_t1_, face_t2_;
		float face_e1_ = 0.f, face_e2_ = 0.f;
		float face_d_ = 0.f;
		float world_min_z_ = -8192.f;
		bool quiet_ = false;
		bool have_seed_mean_ = false;
		std::vector<double> seed_mean_;
		std::vector<TapeFrame> seed_frames_;      // prefix-follow source
		std::vector<unsigned char> duck_overlay_; // per-tick IN_DUCK carry
	};

} // namespace Solver
