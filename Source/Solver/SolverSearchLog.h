#pragma once

// SEARCH-SPACE RECORDER (user directive 2026-08-16: "I want to
// actually see the routes being searched ... a density state space
// based on real lines, whatever is most true to what is actually
// being searched").
//
// What is actually searched: every candidate the solvers evaluate is
// a REAL trajectory flown tick-by-tick through the certified engine
// (FlyHeadingSpline / RideHeadingSpline) - tens of thousands per
// solve. This sink samples them faithfully:
//   - the sims push every Nth position of every evaluated candidate,
//   - the solvers attach the candidate's score and flag
//     best-so-far improvements,
//   - per stage (start probes / leg N air / leg N carve / ...) ALL
//     notable candidates are kept plus a uniform reservoir of the
//     rest, so the kept set is an unbiased density sample of the
//     search with its milestones intact,
//   - references (human tape, the solved line) are stored at full
//     weight.
//
// WriteHtml turns one recorded solve into a SELF-CONTAINED
// interactive report: WebGL line density over the map geometry,
// stage/outcome/score filters, and a chronological scrub that
// replays the search eval-by-eval. No external assets - one file.

#include <string>
#include <vector>

#include "SolverMath.h"

namespace Solver {

	class World;
	namespace Route { struct Graph; }

namespace SearchLog {

	enum Outcome {
		kMiss = 0,      // horizon end, nothing reached
		kGrounded = 1,
		kStruck = 2,    // ended on a non-target strike
		kHit = 3,       // board/tap strike on the target face
		kExited = 4,    // clean-air exit (carve non-tap mode)
		kZoned = 5,     // entered the end-zone volume
		kRef = 6,       // reference overlay (tape / result)
		kDoomed = 7,    // terminated at separation: provably unable
		                // to reach the target (the doom cull)
	};

	struct Traj {
		int   stage = 0;
		int   ctx = -1;        // committed-chain context (lineage)
		int   eval = 0;        // global chronological index
		unsigned char outcome = 0;
		bool  notable = false; // best-so-far improvement (or ref)
		float score = 0.f;
		int   stride = 1;      // sim ticks between kept points (the
		                       // report's ideal-gain yardstick)
		std::vector<Vec3> pts;
		std::vector<float> spd;  // |v| at each kept point (u/s) - the
		                         // report shows total energy at any
		                         // selected point from it
		std::vector<int> marks;  // kept-point indices of FACE CONTACTS
		                         // (boards/taps) - drawn as markers
	};

	struct Stage {
		std::string name;
		int total = 0;         // evals seen (kept or not)
	};

	class Sink {
	public:
		// Caps: per-stage reservoir size and point downsampling.
		explicit Sink(int cap_per_stage = 500, int keep_every = 3)
			: cap_(cap_per_stage), keep_every_(keep_every) {}

		// Caller (assembler/probe) names the phases of the search.
		int BeginStage(const std::string& name);

		// The COMMITTED-CHAIN context: what route decisions fed the
		// candidates being evaluated right now (e.g. "[0 1 2] start
		// b-114 | L0 tap f1 -199 | L1 ..."). Every trajectory stores
		// it, so a picked line in the report answers "what route fed
		// into this". Deduplicated; empty clears.
		void SetContext(const std::string& ctx);

		// Sim side: one candidate = StartTraj .. Point.. .. EndTraj.
		void StartTraj(const Vec3& p0, float s0 = 0.f);
		void Point(const Vec3& p, float s = 0.f);
		// A board/tap contact: force-kept point (bypasses the stride,
		// so the line ends exactly at the wall) + a contact marker in
		// the report ("where exactly the board happens").
		void Contact(const Vec3& p, float s = 0.f);
		void EndTraj(int outcome);

		// Solver side, immediately after the sim call returns:
		void Score(float sc);
		void MarkBest();

		// Full-weight overlays (references, the chosen result).
		// spds (optional): |v| per point, for the energy readout.
		void AddRef(const std::string& name, const std::vector<Vec3>& pts,
		            const std::vector<float>* spds = nullptr);

		// 2g factor source for the report's energy readout.
		float gravity = 800.f;
		// Ideal wish-work rate (air_speed_cap^2, u^2/s^2 per tick) -
		// the report's efficiency coloring grades each line's energy
		// gain against stride*this.
		float wish_rate = 900.f;

		// Energy-field heat triangles (the hotspot maps): v01 in [0,1]
		// colors cold->hot; rendered as translucent surface overlay.
		struct HeatTri { Vec3 a, b, c; float v = 0.f; };
		void AddHeat(const Vec3& a, const Vec3& b, const Vec3& c,
		             float v01);
		const std::vector<HeatTri>& HeatRef() const { return heat_; }

		// Commit the pending candidate (also called on BeginStage /
		// StartTraj / at write time).
		void Flush();

		const std::vector<Stage>& StagesRef() const { return stages_; }
		const std::vector<Traj>&  TrajsRef()  const { return trajs_; }
		const std::vector<std::string>& ContextsRef() const {
			return contexts_;
		}

	private:
		void CommitPending();

		int cap_;
		int keep_every_;
		int cur_stage_ = -1;
		int cur_ctx_ = -1;
		std::vector<std::string> contexts_;
		std::vector<HeatTri> heat_;
		int tick_ = 0;
		int eval_counter_ = 0;
		bool open_ = false;
		bool have_pending_ = false;
		Traj pending_;
		unsigned rng_ = 20260816u;
		std::vector<Stage> stages_;
		std::vector<Traj> trajs_;
		// Per stage: indices (into trajs_) of kept NON-notable rows
		// (the reservoir) and how many non-notables were seen.
		std::vector<std::vector<int>> pool_;
		std::vector<int> pool_seen_;
	};

	// Null = recording off (the default; hot sims check one pointer).
	extern Sink* g_sink;

	// One recorded solve -> a self-contained interactive HTML report.
	bool WriteHtml(const std::string& path, const World& w,
	               const Route::Graph& g, const Sink& sink,
	               const std::string& title, std::string* err);

} // namespace SearchLog
} // namespace Solver
