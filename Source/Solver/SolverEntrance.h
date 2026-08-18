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

#include <vector>

#include "SolverMove.h"
#include "SolverRoute.h"

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
		// SPLINE-FAMILY witness (second realization layer, for cells
		// the two-hold family cannot express - corridor approaches):
		// when non-empty, the witness is these knots flown over
		// `horizon` ticks and psi1/psi2/split are meaningless.
		std::vector<float> knots;
		int   horizon = 0;
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
	};

	struct Opts {
		float grid = 32.f;
		int   max_ticks = 300;
		int   psi_steps = 24;     // sweep resolution per held heading
		int   split_step = 12;    // ticks between split candidates
		int   refine_rounds = 2;
		int   flights_cap = 9000; // engine-flight budget per field
		bool  dwell_free = false; // dwellcost variant: no flip-gap law
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

} // namespace Entrance
} // namespace Solver
