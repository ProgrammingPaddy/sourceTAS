#pragma once

// UNSEEDED TWO-LAYER SOLVER (user-directed 2026-08-14): exhaustive search
// at the level where exhaustiveness is meaningful.
//
// Layer 1 - SKELETON: the discrete structure of a line is its sequence of
// face contacts. Surfable faces are read from the BSP (any brush plane with
// 0 < n.z < walkable_z - too steep to walk on IS the definition of a surf
// ramp, derived from live params, no map rules). The skeleton space (face
// sequences up to a depth cap) is tiny - dozens to thousands - and is
// searched BEST-FIRST with junction-value ordering; within budget every
// expanded node attempts the finish, so skeletons are enumerated, not
// sampled. Human inputs appear NOWHERE.
//
// Layer 2 - SEGMENTS: each edge (board face A -> board face B, or face ->
// END landing) is a continuous boundary-value problem solved by the smooth
// spline + CMA machinery (SolverSmooth segment mode). Junction fitness is
// the user's compromise equation: board the next face with maximum value
// V = KE + mu*g*z, having left the previous one cleanly (fitted waste
// penalty), paying w_tick per scored tick. A beam of the top junction
// states per (depth, face) keeps arrival DIVERSITY (the handoff's
// load-bearing lesson) without culling anything - lower-value junctions
// just wait their turn in the queue.
//
// Assembly: streams are per-tick frames; segments start EXACTLY at the
// parent's end state, so concatenation replays identically from the anchor
// (verified). The best clean assembly gets a global smooth-CMA polish
// (machine stream -> spline inversion is near-lossless, unlike human
// tapes). Clean-first at every stage; the zone-jump budget is structural.

#include "SolverSmooth.h"

#include <string>
#include <vector>

namespace Solver {

	struct ChainConfig {
		MoveParams params;
		int start_brush_id = -1;
		int end_brush_id = -1;       // REQUIRED
		double total_seconds = 360.0;
		double seg_seconds = 8.0;    // per segment solve (early-stops sooner)
		double end_seconds = 15.0;   // END-landing attempts (the money segment)
		double final_seconds = 30.0; // global polish of the best assembly
		int beam = 3;                // junction states kept per (depth, face)
		int touch_settle = 16;       // ride ticks before a junction hands off
		float beam_sep = 96.f;       // junction diversity radius (units)
		int max_depth = 5;           // boards per skeleton
		int seg_ticks = 360;         // segment rollout domain
		int first_seg_ticks = 560;   // segment 0 domain (prestrafe included)
		int cp_ticks = 8;            // segment CP spacing (board windows)
		unsigned rng_seed = 1337;
		int threads = 0;
		float wloss = 6.5f;
		float mix_mu = 0.4f;
		float w_tick = 500.f;
		// v17: continuation-approach weight in segment fitness (overrides
		// the SmoothConfig default for chain segments).
		float w_fin_dmin = 250.f;
		// Diagnosis: write every dead segment's best attempt as a
		// replayable tape here (empty = off).
		std::string dump_dir;
	};

	struct ChainResult {
		bool ok = false;
		bool clean = false;
		int scored = 0;
		int abs_tick = 0;
		std::vector<TapeFrame> frames;
		std::vector<int> skeleton;   // face indices into Faces()
		int nodes_expanded = 0;
		int segments_solved = 0;
		int finishes = 0;
		int finisher_skels = 0;      // distinct skeletons among finishers
		long long ticks_simulated = 0;
		double seconds = 0.0;
	};

	class ChainSolver {
	public:
		struct Face {
			int brush = -1;      // World::brushes index
			int brush_id = -1;   // BSP id (printing)
			int plane = -1;
			Vec3 n;
			Vec3 center;
		};

		ChainSolver(const World& w, const ChainConfig& cfg,
		            const TapeAnchor& anchor);

		const std::vector<Face>& Faces() const { return faces_; }

		ChainResult Run();

	private:
		struct SegOut {
			bool ok = false;
			bool finished = false;   // END mode: clean finish
			float V = 0.f;           // junction value (board mode)
			float speed = 0.f;
			int ticks = 0;
			PlayerState end_state;
			float end_yaw = 0.f;
			std::vector<TapeFrame> frames;
			float dmin = 1e9f;
			float eloss = 0.f;       // segment energy dissipated (ledger units)
			float fin_dmin = 1e9f;   // continuation's closest end approach
			// v15: when the segment's CONTINUATION landed the end brush,
			// finished is true and fin_frames carries the full stream to the
			// landing (frames still stops at the junction for the child).
			std::vector<TapeFrame> fin_frames;
			// v19: the solve's best genome - escalations warm-start from it.
			std::vector<double> best_x;
		};
		// target_face < 0 = END landing attempt. [ty_lo, ty_hi] = board
		// height band on the face (clause-3 enumeration). budget_scale
		// multiplies the solve budget (near-miss END escalation); seed_x
		// warm-starts the CMA from a same-domain genome (v19: escalations
		// continue the found basin instead of re-rolling cold).
		SegOut SolveSegment(const PlayerState& root, float yaw,
		                    int target_face, bool first_segment,
		                    unsigned rng, long long* ticks,
		                    float ty_lo = 0.f, float ty_hi = 1.f,
		                    double budget_scale = 1.0,
		                    const std::vector<double>* seed_x = nullptr);
		// Replay an assembled stream from the anchor: authoritative scored
		// ticks / ending class for candidate ranking.
		struct AsmStats {
			int finish = -1, exit = -1;
			bool clean = true;
			float max_impact = 0.f;
		};
		AsmStats ReplayAssembly(const std::vector<TapeFrame>& frames) const;

		const World& w_;
		ChainConfig cfg_;
		TapeAnchor anchor_;
		PlayerState root_;
		float root_yaw_ = 0.f;
		std::vector<Face> faces_;
	};

} // namespace Solver
