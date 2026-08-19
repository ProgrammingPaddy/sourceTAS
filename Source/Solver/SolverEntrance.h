#pragma once

// THE ENTRANCE FIELD (Docs/OptimalBoardingHandoff.md, adopted
// 2026-08-18; ruling recorded in SolverRebuild.md 2.11).
//
// Per landing cell x vertical arrival branch, the stored value is the
// post-board energy of the BEST WITNESS-REALIZABLE flight from the
// start state - an ENGINE-REPLAYED outcome, never a bound and never
// an analytic trace value (the witnesscheck rule). Records carry the
// witness so a selector that picks a cell replays exactly what was
// proven: the cell and its witness are one atomic solution.
//
// H(Q) is DEFINED over ALL admissible controls. The two-held-heading
// family swept here is the initial REALIZATION STRATEGY, not the
// definition (user's ruling 2026-08-18: do not promote the family to
// "exact" because it is efficient). `fieldexact` compares this field
// against a broader spline-space reference search; the family is
// certified only if it saturates that frontier.
//
// FastestTangent is computed and stored SEPARATELY per cell; tangent
// dominance is an OPEN THEOREM (handoff 9.2) - tangent_gap > 0 means
// a lossy arrival genuinely out-carries every tangent arrival there.

#include <math.h>

#include <functional>
#include <vector>

#include "SolverAir.h"
#include "SolverMove.h"
#include "SolverRoute.h"
#include "SolverSteer.h"

namespace Solver {

	class World;

namespace Entrance {

	// Engine-verified winner at one (cell, branch). All H/tan_E
	// values are REPLAYED post-board energies:
	//   |v_post|^2 + 2g*(cp.z - face.zmin).
	struct Rec {
		bool  feasible = false;   // some family trace lands here
		bool  verified = false;   // an engine witness strike landed here
		int   n = 0;              // witness arrival tick
		float H = -1e30f;         // replayed post-board energy
		float theta = 0.f;        // arrival heading of the winner
		float s2d = 0.f;          // horizontal |v-| at contact
		float vz = 0.f;           // vertical velocity at contact
		Vec3  v_pre, v_post;      // exact pre-/post-clip velocities
		Vec3  cp;                 // exact contact position
		float dot = 0.f;          // clip dot (negative = into face)
		float loss2 = 0.f;        // energy lost at the board (dot^2)
		// Witness parameters (two-hold family): turn to psi1 at the
		// free rate, hold until `split`, turn to psi2, hold to contact.
		// prof_n = the dense profile length that was FLOWN (replay
		// with WitnessProfile(psi1, psi2, split, prof_n)).
		float psi1 = 0.f, psi2 = 0.f;
		int   split = 0;
		int   prof_n = 0;
		// DENSE-PROFILE witness (heading-command layer): when
		// non-empty, the witness is these knots flown over `horizon`
		// ticks and psi1/psi2/split are meaningless.
		std::vector<float> knots;
		int   horizon = 0;
		// WISH-BASIS witness (the canonical control space, ruling
		// 2026-08-18): when wside is non-empty the witness is the
		// per-tick (side, cosa) schedule replayed by
		// Air::FlyWishSchedule over `horizon` ticks - it takes
		// precedence over both other forms.
		std::vector<signed char> wside;
		std::vector<float> wcosa;
		// Which realization layer found the winner (0 = two-hold
		// family, 1 = reference solver) - production-family
		// completeness is measured from these tags (ruling
		// 2026-08-18: H is the best witnessed result regardless of
		// finder; the finder is recorded).
		int   source = 0;
		// FastestTangent at this cell/branch (|dot| <= kTanEps),
		// stored separately - NEVER replaces H.
		bool  tan_ok = false;
		float tan_E = -1e30f;
		float tan_theta = 0.f;
		float tan_s2d = 0.f;
		float tan_dot = 0.f;
		int   tan_n = 0;
		float tan_psi1 = 0.f, tan_psi2 = 0.f;
		int   tan_split = 0;
		int   tan_prof_n = 0;
	};

	struct FieldMap {
		int  face = -1;           // graph face index
		Vec3 origin, ud, vd;      // grid frame (same layout as Field)
		int  nu = 0, nv = 0;
		float du = 0.f, dv = 0.f;
		// Branch layers: [0] descending arrivals, [1] ascending.
		std::vector<Rec> rec[2];
		int   best = -1;          // flat index: branch*nu*nv + cell
		float H_lo = 0.f, H_hi = -1e30f;   // verified range
		int   traced = 0;         // family traces that landed in-face
		int   flights = 0;        // engine witness flights flown
		int   cells_feasible = 0;
		int   cells_verified = 0;
		// THE CLEAN-AIR LAW (ruling 2026-08-18): a direct entrance
		// witness must be collision-free before Q - an intermediate
		// strike rewrites the ballistics and the flight is no longer
		// the aerial operator (it is Air -> Contact -> Air, a route-
		// graph composition, not a field entry). Rejected strikes are
		// counted here, never credited.
		int   contact_assisted = 0;
	};

	struct Opts {
		float grid = 32.f;
		int   max_ticks = 300;
		int   psi_steps = 24;     // sweep resolution per held heading
		int   split_step = 12;    // ticks between split candidates
		int   refine_rounds = 2;
		int   flights_cap = 9000; // engine-flight budget per field
		bool  dwell_free = false; // dwellcost variant: no flip-gap law
		// Boundary control-history state (Invariant 9): constrains
		// the first reversal of every flight this field flies.
		Steer::CtlState entry_ctl;
	};

	// |dot| under this = tangent-valid in practice (loss^2 <= 625,
	// noise-level against 1e5..1e6 energies). The engine's own strike
	// predicate (dot < 0) stays the contact law - this is only the
	// classification width for the FastestTangent slot.
	constexpr float kTanEps = 25.f;

	FieldMap Build(const PlayerState& entry, const World& w,
	               const MoveParams& p, const Route::Graph& g,
	               int face_idx, const Opts& o);

	// Regenerate a witness's dense per-tick heading profile (the
	// commands the engine flight flew). Used for replay/verification.
	void WitnessProfile(const PlayerState& entry, const MoveParams& p,
	                    float psi1, float psi2, int split, int n,
	                    std::vector<float>* prof);

	// The segment-schedule encoding RefSolve searches: per segment,
	// turn at `rate` (signed fraction of the per-tick free rate) for
	// `len` ticks. Exposed for the representation-completeness unit
	// test (repfit): the same encoding, fed a fitted schedule.
	void ScheduleProfile(const PlayerState& entry, const MoveParams& p,
	                     const std::vector<int>& len,
	                     const std::vector<float>& rate,
	                     std::vector<float>* prof);

	// THE GENERAL REFERENCE SOLVE (ruling 2026-08-18: one general
	// boundary-value search over the admissible L/R control space -
	// segments of (length >= the dwell law, signed turn-rate
	// fraction) - NOT an expanding vocabulary of named geometric
	// families; the cheap families are proposal seeds only). The
	// objective is the best CLEAN-AIR engine strike within `radius`
	// of `q`: replayed post-board energy, maximized. tangent_mode
	// restricts the win condition to |dot| <= kTanEps (the explicit
	// FastestTangent solve). Misses/contact-assisted strikes guide
	// but never win.
	// THE EXACTNESS LADDER (advisor 2026-08-19): spatial recovery is
	// reported at these tolerances, coarse to fine - a benchmark
	// curve, not a single hit/miss bit.
	constexpr float kLadder[6] = { 32.f, 16.f, 8.f, 4.f, 2.f, 1.f };

	// The ONE position/heading trade weight (units of u per radian),
	// shared by the boundary search metric and the Gauss-Newton
	// residual so the optimizer descends exactly what the search ranks
	// (they were 250 and 200 - inconsistent).
	constexpr float kThetaW = 250.f;

	// ---- PURE HELPERS, shared by production and the property gate
	// (`airprops`) so a test can never drift from the implementation ----

	// Scout-pool endpoint dedupe radius for an active tolerance. MUST be
	// non-increasing in tol and MUST never exceed the historical 32u:
	// a formula introduced for the fine-tolerance case silently loosened
	// every coarse caller (Field::Build went to 115u with 3 slots).
	inline float ScoutDedupe(float tol) {
		float d = 4.f * tol;
		if (d < 8.f) d = 8.f;
		if (d > 32.f) d = 32.f;
		return d;
	}

	// Curvature to-go: signed total sweep of the circle tangent to the
	// current heading (local frame, velocity along +x) through the
	// target at (lx, ly).
	inline float ToGoSweep(float lx, float ly) {
		return 2.f * atan2f(ly, lx);
	}

	// ...and its arc length, BOUNDED. Behind the target (ly -> 0,
	// lx < 0) curvature -> 0 while |sweep| -> pi, so phi/kappa
	// diverges - measured at 1.3e10, which swamped every other guidance
	// term and made sidestepping look cheaper than turning. The arc can
	// never be shorter than the chord nor longer than the half-circle
	// through both points.
	inline float ToGoArcLen(float lx, float ly) {
		const float r2 = lx * lx + ly * ly;
		const float chord = sqrtf(r2);
		if (r2 <= 1e-3f)
			return 0.f;
		const float kap = 2.f * ly / r2;
		const float phi = ToGoSweep(lx, ly);
		float arc = fabsf(kap) > 1e-6f ? fabsf(phi / kap) : chord;
		const float arc_max = 1.57079633f * chord;
		if (arc < chord) arc = chord;
		if (arc > arc_max) arc = arc_max;
		return arc;
	}

	// Optional search-machinery tuning (advisor 2026-08-19). Defaults
	// reproduce the production solve; airrec sweeps shoot_m to measure
	// the recovery CURVE (the Level-B trigger is its shape).
	struct RefTune {
		// Sequential-shooting node ceiling: 0 = single-target shots
		// only, 1 = +one free node (+node-time), 2 = +second node +
		// outcome-space Gauss-Newton. Base seeds/frontier/deepen/
		// scouts always run.
		int shoot_m = 2;
		// ACTIVE TOLERANCE CONTINUATION (advisor 2026-08-19): the
		// finest positional tolerance this query actually wants. 0 =
		// off (region/field semantics: stop at `radius`). When set,
		// the refinement loop first maximizes value at `radius` as
		// usual, THEN tightens the active tolerance down the kLadder
		// rungs to this value, minimizing the residual at each rung -
		// so the value contract never regresses and the exact-point
		// rungs stop being reachable only by luck. Exact-point queries
		// (humanexact, ladder, airrec) set it; cell/field queries
		// (Build, fieldexact) must not.
		float precision = 0.f;
		// BOUNDARY MODE (Layer-1 recoverability): non-null = solve the
		// free-air boundary problem (S0, Q, T, theta) instead of a
		// face strike. Points at {th_lo, th_hi} (radians, absolute).
		// T is fixed = n_hint: schedules are exactly that long,
		// scored on the tick-T residual; success = within `radius` of
		// q with terminal heading inside the interval; H = terminal
		// horizontal speed (the s_A* quantity).
		const float* bnd_theta = nullptr;
	};

	struct RefResult {
		bool  ok = false;
		float H = -1e30f;
		Air::Result flight;       // the winning engine outcome
		// The winning wish-basis witness (per-tick side + cosa,
		// replayed by Air::FlyWishSchedule over `horizon`).
		std::vector<signed char> wside;
		std::vector<float> wcosa;
		int   horizon = 0;
		// FASTEST TANGENT from the SAME terminal-heading frontier
		// (ruling: one frontier yields both solves) - the max-energy
		// arrival with |dot| <= kTanEps.
		bool  tan_ok = false;
		float tan_H = -1e30f;
		Air::Result tan_flight;
		std::vector<signed char> tan_wside;
		std::vector<float> tan_wcosa;
		int   tan_horizon = 0;
		int   evals = 0;              // engine flights spent
		int   contact_assisted = 0;   // clean-air-law rejections seen
		// EXACTNESS LADDER (advisor 2026-08-19): closest clean-air
		// strike to q over the whole search (any distance), its
		// terminal heading, and the best replayed H per kLadder rung.
		float strike_rmin = 1e30f;
		float strike_rmin_th = 0.f;
		float rung_E[6] = { -1e30f, -1e30f, -1e30f, -1e30f, -1e30f,
			-1e30f };
		// PER-RUNG WITNESSES. A rung that can be reported but not
		// handed back is only observable, never usable - it could
		// neither be credited to a Rec nor banked. Each rung now
		// carries the schedule that achieved it and its realized
		// residual, so a tightened result is a first-class witness.
		std::vector<signed char> rung_side[6];
		std::vector<float> rung_cosa[6];
		float rung_res[6] = { 1e30f, 1e30f, 1e30f, 1e30f, 1e30f,
			1e30f };
		// PER-INTERVAL heading coverage {L_i, hits, shots} about
		// iv_ref (the closed-form arrival estimate). L_i = best
		// in-radius witness whose terminal heading lies in interval i;
		// hits = its witness count; shots = guided attempts targeted
		// at it. hits == 0 means UNRESOLVED - never "unreachable"
		// (estimates order, only certified bounds erase).
		float iv_L[6] = { -1e30f, -1e30f, -1e30f, -1e30f, -1e30f,
			-1e30f };
		int   iv_hits[6] = { 0, 0, 0, 0, 0, 0 };
		int   iv_shots[6] = { 0, 0, 0, 0, 0, 0 };
		float iv_ref = 0.f;
		// BOUNDARY MODE residuals (RefTune::bnd_theta): best tick-T
		// outcome by the search metric rp + 250*rth, and its terminal
		// horizontal speed.
		// OBSERVABILITY for the property gate: how the budget was
		// actually divided, which coordinate chart won, and the final
		// active tolerance. Reported, never used to decide anything.
		int   evals_cover = 0;    // the budget-INDEPENDENT prefix
		int   evals_phase1 = 0;   // spent before the first tightening
		int   win_chart = -1;     // 0 = chord, 1 = curvature, -1 = none
		float tol_final = 0.f;
		float bnd_rp = 1e30f;
		float bnd_rth = 1e30f;
		float bnd_s = 0.f;
	};
	// on_strike (optional): called for EVERY clean-air face strike the
	// search flies, wherever it lands - the caller may credit its
	// field ("every valid witness raises the lower bound"). Args:
	// the flight, the wish schedule (side, cosa), its horizon.
	// entry_ctl: the boundary control-history state (Invariant 9 -
	// the dwell law crosses operator seams; the first reversal of
	// this flight is constrained by the previous operator's last).
	// Every produced schedule is validated against it.
	// seed_side/seed_cosa (optional, DIAGNOSTIC ONLY - the recovery
	// ladder): a per-tick wish schedule injected as one extra seed
	// (compressed to side-runs + knots). Production solves never pass
	// these; the human witness enters only through the `ladder` gate.
	RefResult RefSolve(const PlayerState& entry, const World& w,
	                   const MoveParams& p, const Route::Graph& g,
	                   int face_idx, const Vec3& q, float radius,
	                   int n_hint, int budget, bool tangent_mode,
	                   int* flights_counter, float zmin,
	                   const Steer::CtlState& entry_ctl
	                       = Steer::CtlState(),
	                   const std::function<void(const Air::Result&,
	                       const std::vector<signed char>&,
	                       const std::vector<float>&, int)>&
	                       on_strike = nullptr,
	                   const std::vector<signed char>* seed_side
	                       = nullptr,
	                   const std::vector<float>* seed_cosa = nullptr,
	                   const RefTune* tune = nullptr);

} // namespace Entrance
} // namespace Solver
