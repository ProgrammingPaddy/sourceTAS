#pragma once

// Phase 1 route explorer: archive-driven search (Go-Explore family) over
// LEGAL knot genomes on the proven tick-exact core. Design contract
// (Docs/FullMapSolver.md):
//
//   - Controls are KNOTS {duration, side L/R/coast, jump, duck, ground turn
//     rate}; within a knot the air yaw comes from the closed-loop max-gain
//     controller (derived from the LIVE MoveParams each tick, never
//     constants). One-tick strafing is unrepresentable; the flips-per-sec
//     rule is enforced STRUCTURALLY, including across archive splice points
//     (entries carry ticks-since-flip).
//   - Archive cell = quantized position x speed bucket x contact mode. Best
//     entry per cell (earliest tick), full state snapshot (O(1) restore),
//     shared-prefix genomes (entries store only knots since their parent).
//   - Results define conditions: any reached cell is kept; the ONLY hard
//     rule is the user-sanctioned startzone jump budget. Progress bias is
//     straight-line distance to the finish (tournament selection weight
//     only, never a cull). BFS distance field deferred until a winding map
//     needs it - logged in the lab notes.
//   - Finish = grounded on the end platform (hull-expanded footprint).

#include "SolverKnots.h"
#include "SolverMove.h"
#include "SolverTape.h"

#include <string>
#include <vector>

namespace Solver {

	struct ExploreConfig {
		MoveParams params;
		int start_brush_id = -1;     // startzone platform (BSP id); -1 = none
		int end_brush_id = -1;       // finish platform (BSP id); REQUIRED
		int max_zone_jumps = 1;      // the one universal ban: startzone hops
		float flips_per_sec = 5.f;   // L/R direction-change budget
		double budget_seconds = 30.0;
		long long max_rollouts = 0;  // 0 = budget governs
		int max_path_ticks = 4000;   // longer routes are junk on any testbed
		unsigned rng_seed = 1337;    // fixed default = reproducible runs
		float cell_size = 64.f;      // position quantum
		float speed_bucket = 100.f;  // 2D-speed quantum
		// GOAL: false = grounded ON the end platform (full-map contract);
		// true = any contact with the end brush (segment experiments -
		// "fastest to ramp N" as a theory test lab).
		bool goal_touch = false;
		// Smooth-frontier selection axis: prefer low cumulative energy
		// dissipation near the finish. Measured 2026-08-13: fast runs
		// dissipate ~260k with 3 big-loss events; the slow unseeded route
		// 538k with 8 - dissipation-at-progress is the human-routing signal.
		// Bias only (fitness stays ticks); off = control arm for A/B.
		bool eloss_bias = true;
	};

	struct Finisher {
		int tick = 0;               // ticks from anchor to the goal
		float speed = 0.f;          // 2D speed at the finish tick
		float eloss = 0.f;          // cumulative energy dissipated en route
		Vec3 pos;                   // finish position (z proves ON-TOP)
		int entry = -1;             // archive index of the finishing entry
	};

	struct ExploreResult {
		long long rollouts = 0;
		long long ticks_simulated = 0;
		int entries = 0;
		int cells = 0;
		std::vector<Finisher> finishers;   // sorted by tick ascending
		double seconds = 0.0;
	};

	class Explorer {
	public:
		Explorer(const World& w, const ExploreConfig& cfg);

		// Root the archive at the run anchor (required, before Run/Seed).
		void SetAnchor(const TapeAnchor& a);

		// Optional: replay a human tape through the core, archiving its cells
		// as restart points (instant high-quality seeds). Returns ticks used.
		int SeedFromTape(const Tape& tape);

		ExploreResult Run();

		// Re-roll a finisher's genome from the anchor, emitting per-tick
		// frames (engine-replayable). Verifies the recorded finish tick
		// reproduces exactly; false on any mismatch (determinism fault).
		bool BuildFrames(int entry_index, std::vector<TapeFrame>& out,
		                 int* finish_tick);

		// Tick at which the seed tape itself finished (-1 = it didn't / no
		// seed). The incumbent the explorer is trying to beat.
		int SeedFinishTick() const { return seed_finish_tick_; }

		// Flatten a finisher's chain into an optimizer genome: seed-prefix
		// length + concatenated knots + the flip state at the branch point.
		struct FlatGenome {
			int seed_prefix = 0;
			signed char base_side = 0;
			short base_since_flip = 999;
			std::vector<Knot> knots;
		};
		bool ExtractGenome(int entry_index, FlatGenome& out) const;

		// The archive's root state (anchor after ground settling) + yaw -
		// the optimizer re-rolls genomes from exactly here.
		void RootState(PlayerState* s, float* yaw) const;

	private:
		struct Entry {
			PlayerState st;
			float yaw = 0.f;         // controller continuity (yaw is input,
			                         // not physics state - carried here)
			int tick = 0;
			int parent = -1;         // -1 = root or seed-prefix entry
			int seed_ticks = -1;     // >=0: state = seed tape replayed this far
			signed char last_side = 0;
			short ticks_since_flip = 999;
			short zone_jumps = 0;
			short cbrush = -1;          // cell contact brush (census/diagnostics)
			signed char ckind = 0;      // 0 air, 1 ground, 2 touch
			float eloss = 0.f;          // cumulative dissipation along the path
			bool finished = false;
			// Knots since the parent, INLINE (rollouts append <= 3): POD
			// entries, no per-entry heap churn at millions of records.
			Knot knots[3];
			unsigned char nknots = 0;
		};

		const World& w_;
		ExploreConfig cfg_;
		// Frontier indices for round-robin selection (the prior attempt's
		// validated lesson: multi-criteria diversity preserves the useful
		// extremes a single score collapses). Distance bands pull toward the
		// finish; speed bands pull along the speed pipeline; uniform keeps
		// everything reachable. Bias only - nothing is ever culled.
		std::vector<int> bands_[32];    // dist-to-end / 256
		std::vector<int> sbands_[16];   // 2D speed / 100
		float min_dist_seen_ = 1e9f;
		float max_speed_seen_ = 0.f;
		int min_dist_entry_ = -1;
		// Near-miss ring: entries that got close to the finish. A selection
		// mode backtracks 1-2 knots up their parent chains and resamples -
		// breeding the release-timing variations that convert a wall-kiss
		// below the lip into a landing.
		std::vector<int> near_ring_;
		// Free-air records get their own sub-cap: junk falls were filling the
		// whole archive before the contact pipeline matured (frontier froze at
		// 20s with min-dist still advancing). Contact/event cells keep the
		// full budget.
		long long air_entries_ = 0;
		int min_knot_ = 14;
		int start_idx_ = -1, end_idx_ = -1;
		int seed_finish_tick_ = -1;
		Vec3 end_center_;
		const Tape* seed_tape_ = nullptr;
		std::vector<Entry> entries_;
		// cell key -> entries_ index of the best (earliest-tick) entry
		struct CellMap;
		std::vector<std::pair<unsigned long long, int>> cells_flat_;   // unused; see cpp
		void* cellmap_ = nullptr;   // unordered_map behind a pimpl to keep the
		                            // header light; owned, freed in dtor
	public:
		~Explorer();
	private:
		unsigned long long CellKey(const PlayerState& s, int contact_kind,
		                           int contact_brush) const;
		bool InsideZoneXY(const Vec3& p, int brush_idx) const;
		bool InsideStartZone(const Vec3& p) const;
		bool IsFinish(const PlayerState& s) const;
		int RecordCell(const PlayerState& s, float yaw, int tick,
		               int parent, int seed_ticks,
		               const std::vector<Knot>& knots, int cur_knot,
		               int ticks_into_knot, signed char last_side,
		               short since_flip, short zone_jumps, float eloss,
		               const TickEvents& ev);
		bool GoalReached(const PlayerState& s, const TickEvents& ev) const;
	};

} // namespace Solver
