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
//
// Phase 2 v2b: the explorer is a WORKER POOL over one shared archive.
// Entries are immutable once published (the finished flag is set at
// creation, never after), the storage is preallocated so indices stay
// stable, the cell map is sharded by key hash, and selection frontiers take
// short per-band locks. Workers cross-pollinate through the shared archive -
// one worker's near-miss is every worker's breeding stock, which is the
// point of sharing rather than running isolated islands. Determinism: with
// --threads 1 a run reproduces the single-thread explorer exactly (same rng
// stream, same archive); with N workers the interleaving makes runs
// non-reproducible even at a fixed seed - by design, variance is the enemy.

#include "SolverKnots.h"
#include "SolverMove.h"
#include "SolverTape.h"

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace Solver {

	// 0 = auto (hardware threads minus one, min 1).
	int ResolveThreadCount(int requested);

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
		int threads = 0;             // worker count; 0 = auto
		// Seed only the first N tape ticks (0 = all). Lets an experiment hand
		// the archive JUST a route's opening (e.g. the human prestrafe+jump)
		// without the rest of the line.
		int seed_limit_ticks = 0;
		// ZONE CLOCK (user-directed 2026-08-13): the timer starts when the
		// player leaves the startzone - prestrafe time is FREE, so selection
		// optimizes exit energy/geometry instead of skimping the opening
		// (tick-greedy cells were structurally suppressing investment
		// starts). Fitness, cell hysteresis, caps, and finisher ranking all
		// use ticks-since-exit; absolute tick keeps bounding the simulation.
		bool zone_clock = true;
		// Cap on the SCORED (post-exit) length; 0 = off. Tighten rounds cap
		// this, not the absolute length, under the zone clock.
		int max_rel_ticks = 0;
		// GOAL: false = grounded ON the end platform (full-map contract);
		// true = any contact with the end brush (segment experiments -
		// "fastest to ramp N" as a theory test lab).
		bool goal_touch = false;
		// Smooth-frontier selection axis: prefer low cumulative energy
		// dissipation near the finish. Measured 2026-08-13 on basictest: fast
		// runs dissipated ~260k with 3 big-loss events; the slow unseeded
		// route 538k with 8 - dissipation-at-progress is the human-routing
		// signal ON THIS MAP (one measurement, not a law - keep testing).
		// Bias only (fitness stays ticks); off = control arm for A/B.
		bool eloss_bias = true;
		// VALUE-MIX frontier (user directive 2026-08-13: energy IS how humans
		// think about completing maps; control the PE/KE tradeoff and the
		// loss penalty rather than maximizing raw E): among the contact bands
		// nearest the finish, restart from the entry with the highest
		//     V = KE + mu*(g*z) - lambda*eloss
		// Within a band the absolute-z offset cancels, so only the MIX
		// matters - nothing map-specific. mu=1,lambda=0 = the measured-
		// harmful raw-E axis; mu=0,lambda=inf = the smooth frontier. The
		// knobs are swept by results; the winning mix is the default.
		// Defaults fitted from the HUMAN RUN + the 9-run quality ladder
		// (2026-08-13, scratchpad/mixfit.py): mu=0.9 lambda=2.0 maximizes
		// rank-agreement between V and final route quality at equal progress
		// (0.670 vs 0.581 for the hand-picked 0.25/0.5), and the human ranks
		// #1 in 7/9 shared bands under it. Reads as V = E_total - 2*waste:
		// total energy is the right quantity (the user's claim); the missing
		// piece of the failed raw-E axis was the loss penalty, not the PE
		// term (gz ALONE anti-predicts at -0.28; eloss alone = 0.63).
		// Joint refit with loss FUNCTIONALS (2026-08-14, lossfit.py): linear
		// cumulative loss beats quadratic (0.667) and worst-event (0.607) at
		// Spearman 0.691 with mu=0.40, lambda=6.5 - waste punished ~3x
		// harder than the first fit, moderate PE weight.
		bool energy_frontier = true;
		float efrontier_mu = 0.4f;
		float efrontier_lambda = 6.5f;
		// Micro-lookahead generator. Measured 2026-08-14: flips the archive
		// composition as designed (zone-jump census 500:1 starved -> 3:1
		// jump-DOMINANT) but the ~2.4x tick tax loses end-to-end - cold
		// primary 2726 vs incumbent-class, seeded tighten screens 706/dry vs
		// control 665/767. DEFAULT OFF (results define conditions); the
		// apparatus stays for cheaper variants (smaller C/H, contact-only
		// decisions). 0 = plain sampling.
		int lookahead_c = 0;
		int lookahead_h = 16;
		// FINISH-FLIGHT PROBE (user breakthrough directive 2026-08-14): every
		// accepted CONTACT record within probe_dist of the goal is checked -
		// release now and fly max-gain (coast/left/right, current duck held,
		// NO jump): does it land the finish? A passing state IS a finisher
		// (the flight appended as one knot). Collapses the horizon from
		// "reach red" to "reach any releasable state" and produces DIRECT
		// FACE-EXIT endings by construction. 0 = off.
		float probe_dist = 1280.f;
		int probe_max_ticks = 120;
	};

	struct Finisher {
		int tick = 0;               // ticks from anchor to the goal
		int rel = 0;                // ticks from STARTZONE EXIT (the score
		                            // under the zone clock; == tick without)
		float speed = 0.f;          // 2D speed at the finish tick
		float eloss = 0.f;          // cumulative energy dissipated en route
		Vec3 pos;                   // finish position (z proves ON-TOP)
		int entry = -1;             // archive index of the finishing entry
		// Ending class (user rule: a jump off a non-finish brush into the
		// landing is NOT an acceptable solution). Clean = the final launch
		// left a surface without jumping (face exit / probe flight). Clean
		// finishers ALWAYS outrank tainted ones.
		bool clean = true;
	};

	struct ExploreResult {
		long long rollouts = 0;
		long long ticks_simulated = 0;
		int entries = 0;
		int cells = 0;
		int threads = 1;
		std::vector<Finisher> finishers;   // sorted by tick ascending
		double seconds = 0.0;
	};

	class Explorer {
	public:
		Explorer(const World& w, const ExploreConfig& cfg);
		~Explorer();

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
		// The seed tape's startzone-exit tick (-1 = never left / no seed).
		int SeedExitTick() const { return seed_exit_tick_; }

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

		// VALUATION AUDIT (user-directed): replay a reference tape against
		// the LIVE archive and report every place the machinery ranks the
		// archive's lineages above the reference - cell rejections, per-band
		// V percentile, and the zone-jump census. Read-only; call after Run.
		void AuditTape(const Tape& tape);

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
			short exit_tick = -1;       // startzone-exit tick (-1 = inside)
			unsigned char launch_jump = 0;  // ending-classifier state: the
			                            // path's last surface departure was a
			                            // jump (inherited - rollouts must not
			                            // forget a prefix's spine jump)
			short cbrush = -1;          // cell contact brush (census/diagnostics)
			signed char ckind = 0;      // 0 air, 1 ground, 2 touch
			float eloss = 0.f;          // cumulative dissipation along the path
			bool finished = false;      // set at CREATION only (immutability
			                            // is what makes lock-free reads safe)
			// Knots since the parent, INLINE (rollouts append <= 3): POD
			// entries, no per-entry heap churn at millions of records.
			Knot knots[3];
			unsigned char nknots = 0;
		};

		// Archive limits. Entries live in one preallocated buffer so worker
		// threads publish by index with no reallocation ever moving them.
		static const int kMaxEntries = 4000000;   // frontier freeze point
		static const int kBufSlack = 4096;        // finisher force-adds + races
		static const long long kAirCap = 1500000; // free-air sub-cap

		const World& w_;
		ExploreConfig cfg_;

		// ---- shared archive (workers read lock-free, publish under the
		// owning shard/band lock) ----
		std::vector<Entry> entries_;              // preallocated, index-stable
		std::unique_ptr<std::atomic<unsigned char>[]> ready_;  // publish flags
		std::atomic<int> nentries_{ 0 };
		std::atomic<long long> air_entries_{ 0 };
		std::atomic<bool> cap_msg_{ false };

		// Frontier indices for round-robin selection (the prior attempt's
		// validated lesson: multi-criteria diversity preserves the useful
		// extremes a single score collapses). Distance bands pull toward the
		// finish; speed bands pull along the speed pipeline; uniform keeps
		// everything reachable. Bias only - nothing is ever culled.
		std::vector<int> bands_[32];    // dist-to-end / 256 (contact only)
		std::vector<int> sbands_[16];   // 2D speed / 100
		std::mutex band_mx_[32];
		std::mutex sband_mx_[16];
		std::atomic<int> band_n_[32];   // lock-free empty checks
		std::atomic<int> sband_n_[16];
		// Near-miss ring: entries that got close to the finish. A selection
		// mode backtracks 1-2 knot-chain links and resamples - breeding the
		// release-timing variations that convert a wall-kiss below the lip
		// into a landing. Fixed slots + atomic indices = benign races.
		std::atomic<int> ring_[512];
		std::atomic<int> ring_n_{ 0 };

		std::atomic<float> min_dist_seen_{ 1e9f };
		std::atomic<float> max_speed_seen_{ 0.f };
		int min_dist_entry_ = -1;       // guarded by stats_mx_
		std::mutex stats_mx_;

		std::vector<Finisher> fins_;    // guarded by fin_mx_
		std::mutex fin_mx_;
		std::atomic<int> fin_count_{ 0 };
		std::atomic<int> best_fin_tick_{ -1 };

		std::atomic<long long> rollouts_{ 0 };
		std::atomic<long long> ticks_sim_{ 0 };
		std::atomic<int> done_workers_{ 0 };

		int min_knot_ = 14;
		int start_idx_ = -1, end_idx_ = -1;
		int seed_finish_tick_ = -1;
		short seed_exit_tick_ = -1;
		Vec3 end_center_;
		const Tape* seed_tape_ = nullptr;
		void* cellmap_ = nullptr;   // sharded hash maps behind a pimpl to
		                            // keep the header light; owned, freed in
		                            // the dtor

		int Count() const {
			const int n = nentries_.load(std::memory_order_relaxed);
			const int cap = kMaxEntries + kBufSlack;
			return n < cap ? n : cap;
		}
		// Scored ticks under the active clock: since zone exit when the zone
		// clock is on (0 while still inside), absolute otherwise.
		int RelTicks(int tick, short exit_tick) const {
			if (!cfg_.zone_clock)
				return tick;
			return exit_tick >= 0 ? tick - exit_tick : 0;
		}
		unsigned long long CellKey(const PlayerState& s, int contact_kind,
		                           int contact_brush) const;
		bool InsideZoneXY(const Vec3& p, int brush_idx) const;
		bool InsideStartZone(const Vec3& p) const;
		bool IsFinish(const PlayerState& s) const;
		int AllocEntry(bool force);
		int RecordCell(const PlayerState& s, float yaw, int tick,
		               int parent, int seed_ticks,
		               const std::vector<Knot>& knots, int cur_knot,
		               int ticks_into_knot, signed char last_side,
		               short since_flip, short zone_jumps, short exit_tick,
		               bool launch_jump, float eloss, const TickEvents& ev,
		               bool finished_flag);
		bool GoalReached(const PlayerState& s, const TickEvents& ev) const;
		void WorkerLoop(int wid, std::chrono::steady_clock::time_point t0);
	};

} // namespace Solver
