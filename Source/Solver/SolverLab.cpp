// SolverLab: console harness for the solver core. Runs the engine-independent
// world + movement model without the game so search/physics work iterates at
// full speed. Commands:
//
//   SolverLab mapinfo <map.bsp>
//   SolverLab replay  <map.bsp> <run.tas> [options]
//   SolverLab bench   <map.bsp> [ticks]
//
// replay options:
//   --start-brush N   startzone platform (default: brush under the anchor)
//   --end-brush N     finish platform (finish = grounded on it, inside its
//                     footprint; per the contract "standing on red")
//   --csv <path>      dump per-tick state for plotting/diffing
//   --maxspeed X --gravity X --accel X --airaccel X --friction X --stopspeed X
//   --no-jump-fg      disable the SDK in-CheckJumpButton FinishGravity quirk

#include "SolverExplore.h"
#include "SolverMove.h"
#include "SolverOptimize.h"
#include "SolverChain.h"
#include "SolverRoute.h"
#include "SolverStrafe.h"
#include "SolverEnvelope.h"
#include "SolverBoard.h"
#include "SolverAir.h"
#include "SolverCarve.h"
#include "SolverSteer.h"
#include "SolverLedger.h"
#include "SolverParams.h"
#include "SolverSmooth.h"
#include "SolverTape.h"
#include "SolverWorld.h"

#include <windows.h>
#include <shlobj.h>
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <map>
#include <string>
#include <thread>
#include <vector>

using namespace Solver;

namespace {

	void PrintUsage() {
		printf("SolverLab commands:\n");
		printf("  mapinfo <map.bsp>\n");
		printf("  replay  <map.bsp> <run.tas> [--start-brush N] [--end-brush N]\n");
		printf("          [--csv path] [--maxspeed X] [--gravity X] [--accel X]\n");
		printf("          [--airaccel X] [--friction X] [--stopspeed X] [--no-jump-fg]\n");
		printf("  diff    <map.bsp> <run.tas> <engine_states.csv> [same options]\n");
		printf("          first-divergence report vs the Map Solve tab's export\n");
		printf("  solve   <map.bsp> --end-brush N [--seed-tas run.tas] [--budget-s S]\n");
		printf("          [--flips F] [--rng N] [--out path.tas] [--cell U]\n");
		printf("          [--rollouts N] [--max-ticks N] [--threads N] [--tighten N]\n");
		printf("          [--aim] [--clock zone|anchor] [--seed-ticks N]\n");
		printf("          [--out-eloss path] [physics options]\n");
		printf("          archive route search; best route written as a playable .tas\n");
			printf("  tracediff <queries.csv> <results.csv>\n");
		printf("          our TraceHull vs the in-game engine oracle, per trace\n");
		printf("          (replay --trace-log [--trace-window a b] writes queries;\n");
		printf("          the Map Solve tab's 'Run trace oracle' answers them)\n");
		printf("  smooth  <map.bsp> --end-brush N [--seed-tas run.tas] [--budget-s S]\n");
		printf("          [--pop N] [--cp-ticks N] [--wloss X] [--max-ticks N]\n");
		printf("          [--rng N] [--threads N] [--out path.tas] [physics options]\n");
		printf("          LINE-SPACE search: smooth strafe-intensity spline + CMA-ES\n");
		printf("          (deterministic; clean endings required; labeled output)\n");
		printf("  chain   <map.bsp> --end-brush N [--budget-s S] [--seg-s S] [--beam B]\n");
		printf("          [--depth D] [--optimize-s S] [--rng N] [physics options]\n");
		printf("          UNSEEDED two-layer solve: exhaustive skeleton search over\n");
		printf("          surf-face sequences x chained segment CMA (no tape inputs)\n");
		printf("  tracegen-map <map.bsp>  FULL-MAP geometry sweep queries (every\n");
		printf("          brush face/edge/corner) -> solver\\trace_queries.csv;\n");
		printf("          one in-game oracle click certifies the whole map\n");
		printf("  battery-gen <map.bsp>   write battery_* mechanism decks to recordings\n");
		printf("  battery <map.bsp>       score in-game battery captures vs the core\n");
		printf("          (in-game: Map Solve tab -> 'Run ALIGNMENT battery' plays all\n");
		printf("          decks + captures in ONE click)\n");
		printf("  bench   <map.bsp> [ticks]\n");
		printf("All commands auto-load Documents\\sourceTAS\\solver\\server_params.cfg\n");
		printf("(written by the Map Solve tab from LIVE server values); --params <file>\n");
		printf("overrides the path, CLI flags override individual values.\n");
	}

	// Probe the BSP tree at telling points so solidity is data, not theory.
	void ReportLeafProbes(const World& w) {
		struct P { const char* what; Vec3 p; };
		const P pts[] = {
			{ "spawn",                 w.spawn_origin },
			{ "inside floor slab",     Vec3(0.f, 0.f, -1070.f) },
			{ "just above ceiling",    Vec3(0.f, 0.f, 1290.f) },
			{ "far above map",         Vec3(0.f, 0.f, 4000.f) },
			{ "behind north wall",     Vec3(0.f, 1300.f, 0.f) },
			{ "far outside (y-3000)",  Vec3(0.f, -3000.f, 0.f) },
			{ "mid-air in play space", Vec3(0.f, -500.f, 200.f) },
		};
		printf("BSP leaf probes (CONTENTS_SOLID?):\n");
		for (const P& q : pts)
			printf("  %-24s (%7.0f,%7.0f,%7.0f)  solid=%d\n", q.what,
				q.p.X, q.p.Y, q.p.Z,
				w.PointInSolidLeaf(q.p) ? 1 : 0);
	}

	int CmdMapInfo(const std::string& path) {
		World w;
		std::string err;
		const auto t0 = std::chrono::steady_clock::now();
		if (!w.Load(path, Hulls(), &err)) {
			printf("LOAD FAILED: %s\n", err.c_str());
			return 1;
		}
		const double ms = std::chrono::duration<double, std::milli>(
			std::chrono::steady_clock::now() - t0).count();
		printf("VBSP v%d rev %d, parsed in %.1f ms\n", w.version, w.mapRevision, ms);
		printf("planes=%d brushes=%d (collidable world %d, entity-skipped %d) models=%d\n",
			w.nplanes, w.nbrushes_total, static_cast<int>(w.brushes.size()),
			w.nbrushes_entity, w.nmodels);
		if (w.have_spawn)
			printf("spawn (%.2f, %.2f, %.2f) pitch %.1f yaw %.1f\n",
				w.spawn_origin.X, w.spawn_origin.Y, w.spawn_origin.Z,
				w.spawn_pitch, w.spawn_yaw);
		else
			printf("spawn NOT FOUND in entities\n");
		if (w.brushes.size() <= 64) {
			for (const WorldBrush& b : w.brushes) {
				printf("brush %3d contents=0x%x planes=%d (sides %d) "
					"aabb (%.0f,%.0f,%.0f)..(%.0f,%.0f,%.0f)\n",
					b.id, b.contents, static_cast<int>(b.n.size()), b.nsides,
					b.bmin.X, b.bmin.Y, b.bmin.Z, b.bmax.X, b.bmax.Y, b.bmax.Z);
			}
		}
		ReportLeafProbes(w);
		return 0;
	}

	struct ReplayOpts {
		int start_brush = -1;      // BSP id; -1 = auto (brush under anchor)
		int end_brush = -1;        // BSP id; -1 = none (no finish check)
		std::string csv;
		MoveParams params;
		Hulls hulls;
		// solve-only options
		std::string seed_tas;
		std::string anchor_file;
		std::string anchor_tas;
		std::string sites_csv;   // funcgen --sites: mismatch manifest for
		                         // TARGETED probe batteries at disputed spots
		std::string out_tas;
		std::string out_eloss;      // also write the min-dissipation finisher
		std::string trace_log;      // replay: dump every TraceHull query+answer
		int trace_a = 0;            // trace-log tick window (inclusive)
		int trace_b = 1 << 28;
		int seed_ticks = 0;         // seed only the first N tape ticks (0=all)
		double budget_s = 30.0;
		double optimize_s = 60.0;   // Phase 2 budget after exploration (0 = off)
		long long rollouts = 0;
		float flips = 5.f;
		unsigned rng = 1337;
		float cell = 64.f;
		int max_ticks = 4000;
		int threads = 0;            // explorer/optimizer workers; 0 = auto
		int tighten = 0;            // capped re-explore rounds off the incumbent
		bool goal_touch = false;
		bool eloss_bias = true;
		bool energy_frontier = true;
		float emix_mu = 0.4f;      // joint refit w/ loss functionals (lossfit.py)
		float emix_lambda = 6.5f;
		int lookahead_c = 0;       // knot candidates per decision (0 = off;
		                           // measured: archive flip yes, ticks no)
		int lookahead_h = 16;      // lookahead horizon ticks
		float probe_dist = 1280.f; // finish-flight probe range (0 = off)
		std::string audit_tas;     // solve --audit <tape>: valuation audit
		bool aim = false;           // contact-anchored targeting (measured
		                            // neutral on segments; --aim to enable)
		bool zone_clock = true;     // score = ticks from startzone exit
		                            // (--clock anchor for the old absolute)
		bool edge_bevels = true;    // false = pre-fix clip set (control arm)
		int  fuzz_n = 0;            // fuzzgen: probe count (0 = default)
		bool fuzz_grav1 = false;    // fuzzdiff A/B: force gravity scale 1.0
		bool fuzz_nobv = false;     // fuzzdiff A/B: drop the seeded basevel
		int  fuzz_dump = 0;         // fuzzdiff: dump the first N mismatches
		bool fuzz_dump_reach = false;   // ...restricted to reachable states
		bool corner_true = true;    // false = legacy epsilon-padded hit test
		// smooth (line-space CMA) options
		int pop = 0;                // population (0 = auto)
		int cp_ticks = 16;          // spline control-point spacing
		float wloss = 6.5f;         // non-finisher dissipation shaping weight
		int seed_follow = 0;        // verbatim seed-tape prefix ticks
		bool seed_duck = true;      // carry the seed's duck overlay
		// chain (two-layer unseeded) options
		double seg_s = 8.0;         // per-segment CMA budget
		double end_s = 15.0;        // END-landing attempt budget
		int beam = 3;               // junction beam per (depth, face)
		int depth = 5;              // skeleton board cap
	};

	bool ParseCommon(int argc, char** argv, int first, ReplayOpts& o) {
		// Pass 0: the server-params snapshot (defaults < file < CLI flags).
		// The DLL's Map Solve export writes the canonical file from LIVE
		// server values; loading is loud, never silent.
		std::string params_path;
		bool params_explicit = false;
		for (int i = first; i < argc - 1; ++i)
			if (!strcmp(argv[i], "--params")) {
				params_path = argv[i + 1];
				params_explicit = true;
			}
		if (params_path.empty())
			params_path = CanonicalParamsPath();
		if (!params_path.empty()) {
			bool missing = false;
			std::string rep;
			LoadParamsFile(params_path, o.params, o.hulls, &missing, &rep);
			if (missing) {
				if (params_explicit) {
					printf("params: MISSING %s\n", params_path.c_str());
					return false;
				}
				printf("params: no %s - using built-in defaults\n",
					params_path.c_str());
			} else {
				printf("params: loaded %s\n%s", params_path.c_str(), rep.c_str());
			}
		}
		for (int i = first; i < argc; ++i) {
			const std::string a = argv[i];
			auto next_f = [&](float* dst) {
				if (i + 1 >= argc) return false;
				*dst = static_cast<float>(atof(argv[++i]));
				return true;
			};
			auto next_i = [&](int* dst) {
				if (i + 1 >= argc) return false;
				*dst = atoi(argv[++i]);
				return true;
			};
			bool ok = true;
			if (a == "--start-brush") ok = next_i(&o.start_brush);
			else if (a == "--end-brush") ok = next_i(&o.end_brush);
			else if (a == "--csv") { if (i + 1 < argc) o.csv = argv[++i]; else ok = false; }
			else if (a == "--params") { ++i; }   // consumed in pass 0
			else if (a == "--maxspeed") ok = next_f(&o.params.maxspeed);
			else if (a == "--gravity") ok = next_f(&o.params.gravity);
			else if (a == "--accel") ok = next_f(&o.params.accelerate);
			else if (a == "--airaccel") ok = next_f(&o.params.airaccelerate);
			else if (a == "--friction") ok = next_f(&o.params.friction);
			else if (a == "--stopspeed") ok = next_f(&o.params.stopspeed);
			else if (a == "--seed-tas") { if (i + 1 < argc) o.seed_tas = argv[++i]; else ok = false; }
			else if (a == "--seed-ticks") ok = next_i(&o.seed_ticks);
			else if (a == "--out-eloss") { if (i + 1 < argc) o.out_eloss = argv[++i]; else ok = false; }
			else if (a == "--trace-log") { if (i + 1 < argc) o.trace_log = argv[++i]; else ok = false; }
			else if (a == "--trace-window") {
				if (i + 2 < argc) { o.trace_a = atoi(argv[++i]); o.trace_b = atoi(argv[++i]); }
				else ok = false;
			}
			else if (a == "--anchor") { if (i + 1 < argc) o.anchor_file = argv[++i]; else ok = false; }
			else if (a == "--anchor-tas") { if (i + 1 < argc) o.anchor_tas = argv[++i]; else ok = false; }
			else if (a == "--out") { if (i + 1 < argc) o.out_tas = argv[++i]; else ok = false; }
			else if (a == "--budget-s") { if (i + 1 < argc) o.budget_s = atof(argv[++i]); else ok = false; }
			else if (a == "--optimize-s") { if (i + 1 < argc) o.optimize_s = atof(argv[++i]); else ok = false; }
			else if (a == "--rollouts") { if (i + 1 < argc) o.rollouts = atoll(argv[++i]); else ok = false; }
			else if (a == "--flips") ok = next_f(&o.flips);
			else if (a == "--rng") { if (i + 1 < argc) o.rng = static_cast<unsigned>(atoll(argv[++i])); else ok = false; }
			else if (a == "--n") ok = next_i(&o.fuzz_n);   // fuzzgen probe count
			else if (a == "--grav1") o.fuzz_grav1 = true;
			else if (a == "--nobv") o.fuzz_nobv = true;
			else if (a == "--dump") ok = next_i(&o.fuzz_dump);
			else if (a == "--dump-reach") o.fuzz_dump_reach = true;
			// funcgen: playback CSV whose grounded ticks become CONTROL probes
			else if (a == "--control") { if (i + 1 < argc) o.anchor_tas = argv[++i]; else ok = false; }
			else if (a == "--sites") { if (i + 1 < argc) o.sites_csv = argv[++i]; else ok = false; }
			else if (a == "--cell") ok = next_f(&o.cell);
			else if (a == "--max-ticks") ok = next_i(&o.max_ticks);
			else if (a == "--threads") ok = next_i(&o.threads);
			else if (a == "--goal") {
				if (i + 1 < argc) {
					const std::string gv = argv[++i];
					if (gv == "touch") o.goal_touch = true;
					else if (gv == "ground") o.goal_touch = false;
					else { printf("--goal must be touch|ground\n"); return false; }
				} else ok = false;
			}
			else if (a == "--no-eloss-bias") o.eloss_bias = false;
			else if (a == "--energy-frontier") o.energy_frontier = true;
			else if (a == "--no-energy-frontier") o.energy_frontier = false;
			else if (a == "--audit") { if (i + 1 < argc) o.audit_tas = argv[++i]; else ok = false; }
			else if (a == "--lookahead") {
				if (i + 2 < argc) {
					o.lookahead_c = atoi(argv[++i]);
					o.lookahead_h = atoi(argv[++i]);
				} else ok = false;
			}
			else if (a == "--no-lookahead") { o.lookahead_c = 0; }
			else if (a == "--probe-dist") ok = next_f(&o.probe_dist);
			else if (a == "--emix") {
				if (i + 2 < argc) {
					o.emix_mu = static_cast<float>(atof(argv[++i]));
					o.emix_lambda = static_cast<float>(atof(argv[++i]));
				} else ok = false;
			}
			else if (a == "--no-edge-bevels") o.edge_bevels = false;
			else if (a == "--legacy-corner") o.corner_true = false;
			else if (a == "--pop") ok = next_i(&o.pop);
			else if (a == "--cp-ticks") ok = next_i(&o.cp_ticks);
			else if (a == "--wloss") ok = next_f(&o.wloss);
			else if (a == "--seed-follow") ok = next_i(&o.seed_follow);
			else if (a == "--no-seed-duck") o.seed_duck = false;
			else if (a == "--hull-stand") ok = next_f(&o.hulls.stand_max.Z);
			else if (a == "--hull-duck") ok = next_f(&o.hulls.duck_max.Z);
			else if (a == "--no-unduck-defer") o.params.unduck_hull_defer = false;
			else if (a == "--stam-arm") ok = next_f(&o.params.stamina_jump_ms);
			else if (a == "--stam-scale") ok = next_f(&o.params.stamina_scale_per_ms);
			else if (a == "--stam-pow") ok = next_f(&o.params.stamina_pow_rate);
			else if (a == "--seg-s") { if (i + 1 < argc) o.seg_s = atof(argv[++i]); else ok = false; }
			else if (a == "--end-s") { if (i + 1 < argc) o.end_s = atof(argv[++i]); else ok = false; }
			else if (a == "--beam") ok = next_i(&o.beam);
			else if (a == "--depth") ok = next_i(&o.depth);
			else if (a == "--aim") o.aim = true;
			else if (a == "--no-aim") o.aim = false;
			else if (a == "--tighten") ok = next_i(&o.tighten);
			else if (a == "--clock") {
				if (i + 1 < argc) {
					const std::string cv = argv[++i];
					if (cv == "zone") o.zone_clock = true;
					else if (cv == "anchor") o.zone_clock = false;
					else { printf("--clock must be zone|anchor\n"); return false; }
				} else ok = false;
			}
			else { printf("unknown option: %s\n", a.c_str()); return false; }
			if (!ok) { printf("option %s needs a value\n", a.c_str()); return false; }
		}
		return true;
	}

	// Origin-space (hull-expanded) footprint: the engine grounds a player whose
	// ORIGIN is up to a hull half-width outside the raw brush box (measured:
	// the real run lands on red at x 1213.69 vs the box's 1216).
	bool InsideXY(const Vec3& p, const WorldBrush& b) {
		return p.X >= b.gmin_stand.X && p.X <= b.gmax_stand.X
			&& p.Y >= b.gmin_stand.Y && p.Y <= b.gmax_stand.Y;
	}

	// ---- Output labeling (user-directed 2026-08-14: outputs must identify
	// themselves for in-game testing). The filename carries the score and the
	// ending class; the test card repeats them loudly. ----

	std::string RecordingsDir() {
		char documents[MAX_PATH];
		if (SUCCEEDED(SHGetFolderPathA(nullptr, CSIDL_PERSONAL, nullptr,
			SHGFP_TYPE_CURRENT, documents)))
			return std::string(documents) + "\\sourceTAS\\recordings\\";
		return std::string();
	}

	std::string MapStem(const std::string& map_path) {
		std::string stem = map_path;
		const size_t sl = stem.find_last_of("\\/");
		if (sl != std::string::npos) stem = stem.substr(sl + 1);
		const size_t dot = stem.find_last_of('.');
		if (dot != std::string::npos) stem = stem.substr(0, dot);
		return stem;
	}

	// <recordings>\<map>_<tag>_S<scored>_A<abs>_<CLEAN|JUMP>_<MMDD-HHMM>.tas
	std::string LabeledOutPath(const std::string& map_path, const char* tag,
	                           int rel, int abs_tick, bool clean) {
		SYSTEMTIME t;
		GetLocalTime(&t);
		char name[256];
		_snprintf_s(name, sizeof(name), _TRUNCATE,
			"%s_%s_S%d_A%d_%s_%02d%02d-%02d%02d",
			MapStem(map_path).c_str(), tag, rel, abs_tick,
			clean ? "CLEAN" : "JUMP", t.wMonth, t.wDay, t.wHour, t.wMinute);
		const std::string dir = RecordingsDir();
		std::string path = dir.empty() ? std::string(name) + ".tas"
			: dir + name + ".tas";
		for (int n = 2; GetFileAttributesA(path.c_str())
			!= INVALID_FILE_ATTRIBUTES; ++n)
			path = (dir.empty() ? std::string(name) : dir + name)
				+ " (" + std::to_string(n) + ").tas";
		return path;
	}

	void PrintTestCard(const std::string& path, int rel, int abs_tick,
	                   bool clean, float dt) {
		std::string file = path;
		const size_t sl = file.find_last_of("\\/");
		if (sl != std::string::npos) file = file.substr(sl + 1);
		printf("================= TEST IN GAME =================\n");
		printf("  file   : %s\n", file.c_str());
		printf("  scored : %d ticks (%.3f s, zone clock)\n", rel, rel * dt);
		printf("  abs    : %d ticks from anchor\n", abs_tick);
		printf("  ending : %s\n", clean
			? "CLEAN (face exit)" : "JUMP-LAUNCHED - NOT acceptable");
		printf("  play   : Record tab -> library -> teleport-to-anchor + play\n");
		printf("================================================\n");
	}

	// ---- Tape quality scan (user definition of clean, 2026-08-14: "landing
	// hard on a ramp while pointing tangent to it is still a hard landing" -
	// the measure is the SPEED THE CLIP REMOVED at contact, not the view).
	// Replays frames through the core and reports every hard contact, the
	// air yaw-rate profile, flips/sec, and the ending class. ----

	struct QualityReport {
		int finish_tick = -1;
		int exit_tick = -1;
		bool clean = true;
		float total_clip_loss = 0.f;
		float max_impact = 0.f;
		float max_yaw_rate = 0.f;
		float p95_yaw_rate = 0.f;
		int flips = 0;
	};

	QualityReport QualityScan(const World& w, const MoveParams& p,
	                          const TapeAnchor& anchor,
	                          const std::vector<TapeFrame>& frames,
	                          int start_id, int end_id, bool print) {
		QualityReport q;
		PlayerState s;
		s.pos = anchor.origin;
		s.vel = anchor.velocity;
		s.ducked = anchor.ducked;
		s.hull_state = anchor.ducked ? 1 : 0;
		s.stamina = anchor.stamina;
		{
			TraceResult tr;
			const float gf = w.TraceHull(s.pos, s.pos - Vec3(0.f, 0.f, 2.f),
				s.ducked, &tr);
			if (gf < 1.f && tr.brush >= 0 && tr.normal.Z >= p.walkable_z) {
				s.pos.Z -= 2.f * gf;
				s.on_ground = true;
				s.ground_brush = tr.brush;
			}
		}
		const int start_idx = w.IndexOfBrushId(start_id);
		const int end_idx = w.IndexOfBrushId(end_id);
		struct Impact { int tick; int brush; float loss; float speed; };
		std::vector<Impact> impacts;
		std::vector<float> rates;
		bool launch_jump = false;
		float prev_yaw = anchor.yaw;
		float prev_smove = 0.f;
		for (int t = 0; t < static_cast<int>(frames.size()); ++t) {
			const TapeFrame& f = frames[t];
			TickEvents ev;
			MoveTick(s, w, p, f.pitch, f.yaw, f.fmove, f.smove, f.umove,
				f.buttons, &ev);
			float loss = 0.f;
			int cb = -1;
			for (int c = 0; c < ev.ncontacts; ++c) {
				loss += ev.contact_loss[c];
				if (cb < 0)
					cb = w.brushes[ev.contact_brush[c]].id;
			}
			q.total_clip_loss += loss;
			if (loss > q.max_impact)
				q.max_impact = loss;
			if (loss > 20.f)
				impacts.push_back({ t, cb, loss, Len2D(s.vel) });
			const float dy = fabsf(NormYawDeg(f.yaw - prev_yaw));
			prev_yaw = f.yaw;
			if (!s.on_ground) {
				rates.push_back(dy);
				if (dy > q.max_yaw_rate)
					q.max_yaw_rate = dy;
			}
			if (f.smove != 0.f && prev_smove != 0.f
				&& ((f.smove > 0.f) != (prev_smove > 0.f)))
				q.flips++;
			if (f.smove != 0.f)
				prev_smove = f.smove;
			if (ev.left_ground)
				launch_jump = ev.jumped;
			else if (!s.on_ground && ev.ncontacts > 0
				&& w.brushes[ev.contact_brush[0]].id != end_id)
				launch_jump = false;
			if (q.exit_tick < 0 && start_idx >= 0
				&& !InsideXY(s.pos, w.brushes[start_idx]))
				q.exit_tick = t;
			if (q.finish_tick < 0 && end_idx >= 0 && s.on_ground
				&& s.ground_brush >= 0
				&& w.brushes[s.ground_brush].id == end_id
				&& InsideXY(s.pos, w.brushes[end_idx])) {
				q.finish_tick = t;
				break;
			}
		}
		q.clean = !launch_jump;
		if (!rates.empty()) {
			std::sort(rates.begin(), rates.end());
			size_t k = rates.size() * 95 / 100;
			if (k >= rates.size())
				k = rates.size() - 1;
			q.p95_yaw_rate = rates[k];
		}
		if (print) {
			const float dur = frames.size() * p.dt;
			printf("quality: %d hard contacts (clip > 20 u/s) | total clip "
				"loss %.0f | max one-tick %.0f\n",
				static_cast<int>(impacts.size()), q.total_clip_loss,
				q.max_impact);
			std::sort(impacts.begin(), impacts.end(),
				[](const Impact& a, const Impact& b) {
					return a.loss > b.loss; });
			const int show = impacts.size() < 10
				? static_cast<int>(impacts.size()) : 10;
			for (int i = 0; i < show; ++i)
				printf("  t %5d  brush %-3d  clip %6.0f u/s  (speed %.0f)\n",
					impacts[i].tick, impacts[i].brush, impacts[i].loss,
					impacts[i].speed);
			printf("quality: air yaw rate max %.1f deg/tick, p95 %.2f | "
				"flips %.2f/s | ending %s\n",
				q.max_yaw_rate, q.p95_yaw_rate,
				dur > 0.f ? q.flips / dur : 0.f,
				q.clean ? "CLEAN" : "JUMP-LAUNCHED");
		}
		return q;
	}

	int CmdReplay(const std::string& map_path, const std::string& tas_path,
	              const ReplayOpts& o) {
		World w;
		std::string err;
		w.true_interval_corner = o.corner_true;
		if (!w.Load(map_path, o.hulls, &err, o.edge_bevels)) {
			printf("LOAD FAILED (bsp): %s\n", err.c_str());
			return 1;
		}
		Tape tape;
		if (!LoadTas(tas_path, tape, &err)) {
			printf("LOAD FAILED (tas): %s\n", err.c_str());
			return 1;
		}
		printf("tape: v%d map='%s' frames=%d segments=%d\n", tape.version,
			tape.map.c_str(), static_cast<int>(tape.frames.size()),
			static_cast<int>(tape.segment_starts.size()));
		if (!tape.start.valid)
			printf("WARN: tape anchor invalid - falling back to map spawn\n");

		PlayerState s;
		s.pos = tape.start.valid ? tape.start.origin : w.spawn_origin;
		s.vel = tape.start.valid ? tape.start.velocity : Vec3();
		s.ducked = tape.start.valid ? tape.start.ducked : false;
		s.hull_state = s.ducked ? 1 : 0;
		s.stamina = tape.start.valid ? tape.start.stamina : 0.f;
		printf("anchor (%.3f, %.3f, %.3f) vel (%.1f, %.1f, %.1f) ducked=%d stamina=%.1f\n",
			s.pos.X, s.pos.Y, s.pos.Z, s.vel.X, s.vel.Y, s.vel.Z,
			s.ducked ? 1 : 0, s.stamina);

		// Establish ground before tick 0 (the anchor stands on the platform).
		{
			TickEvents ev;
			// CategorizePosition only: a zero-input MoveTick would advance
			// physics. Down-probe by hand:
			TraceResult tr;
			const float gf = w.TraceHull(s.pos, s.pos - Vec3(0.f, 0.f, 2.f),
			                             s.ducked, &tr);
			if (gf < 1.f && tr.brush >= 0 && tr.normal.Z >= o.params.walkable_z) {
				s.pos.Z -= 2.f * gf;
				s.on_ground = true;
				s.ground_brush = tr.brush;
			}
			(void)ev;
		}

		const int start_id = (o.start_brush >= 0) ? o.start_brush
			: w.BrushUnder(s.pos, s.ducked);
		const int start_idx = w.IndexOfBrushId(start_id);
		const int end_idx = (o.end_brush >= 0) ? w.IndexOfBrushId(o.end_brush) : -1;
		printf("startzone brush %d%s, finish brush %d%s\n",
			start_id, start_idx < 0 ? " (NOT FOUND)" : "",
			o.end_brush, (o.end_brush >= 0 && end_idx < 0) ? " (NOT FOUND)" : "");

		FILE* csv = nullptr;
		if (!o.csv.empty()) {
			if (fopen_s(&csv, o.csv.c_str(), "w") == 0 && csv)
				fprintf(csv, "tick,x,y,z,vx,vy,vz,speed2d,ground,ducked,buttons,yaw\n");
			else
				printf("WARN: cannot open csv path\n");
		}

		std::vector<int> prev_touch;
		bool zone_exited = false;
		int zone_exit_tick = -1;
		int zone_jumps = 0;
		int first_finish = -1;
		float max_speed = 0.f;

		// Trace-oracle query capture: every TraceHull call in the window is
		// logged (query + OUR answer) for the in-game engine oracle to
		// re-answer with ITS collision code.
		std::vector<World::TraceProbeRow> probes;
		for (int t = 0; t < static_cast<int>(tape.frames.size()); ++t) {
			if (!o.trace_log.empty()) {
				w.trace_tick = t;
				w.trace_log = (t >= o.trace_a && t <= o.trace_b)
					? &probes : nullptr;
			}
			const TapeFrame& f = tape.frames[t];
			TickEvents ev;
			MoveTick(s, w, o.params, f.pitch, f.yaw, f.fmove, f.smove, f.umove,
			         f.buttons, &ev);
			const float sp2 = Len2D(s.vel);
			max_speed = fmaxf(max_speed, sp2);

			// Touch set this tick: slide/board contacts + the ground brush.
			std::vector<int> touch;
			for (int c = 0; c < ev.ncontacts; ++c)
				touch.push_back(w.brushes[ev.contact_brush[c]].id);
			if (s.ground_brush >= 0)
				touch.push_back(w.brushes[s.ground_brush].id);
			std::sort(touch.begin(), touch.end());
			touch.erase(std::unique(touch.begin(), touch.end()), touch.end());
			for (int id : touch)
				if (!std::binary_search(prev_touch.begin(), prev_touch.end(), id))
					printf("tick %5d  ENTER brush %-3d  pos (%8.1f,%8.1f,%7.1f) "
						"speed %6.1f vz %7.1f%s\n", t, id,
						s.pos.X, s.pos.Y, s.pos.Z, sp2, s.vel.Z,
						s.on_ground ? " [ground]" : "");
			for (int id : prev_touch)
				if (!std::binary_search(touch.begin(), touch.end(), id))
					printf("tick %5d  leave brush %-3d  speed %6.1f vz %7.1f\n",
						t, id, sp2, s.vel.Z);
			prev_touch = touch;

			if (ev.jumped) {
				const bool in_zone = (start_idx >= 0)
					&& InsideXY(s.pos, w.brushes[start_idx]);
				if (in_zone) zone_jumps++;
				printf("tick %5d  JUMP%s  speed %.1f\n", t,
					in_zone ? " (in startzone)" : "", sp2);
			}
			if (ev.duck_changed)
				printf("tick %5d  DUCK -> %s  pos z %.2f\n", t,
					s.ducked ? "ducked" : "standing", s.pos.Z);
			if (!zone_exited && start_idx >= 0
				&& !InsideXY(s.pos, w.brushes[start_idx])) {
				zone_exited = true;
				zone_exit_tick = t;
				printf("tick %5d  STARTZONE EXIT  pos (%.1f,%.1f,%.1f) speed %.1f\n",
					t, s.pos.X, s.pos.Y, s.pos.Z, sp2);
			}
			if (end_idx >= 0 && first_finish < 0 && s.on_ground
				&& s.ground_brush >= 0 && w.brushes[s.ground_brush].id == o.end_brush
				&& InsideXY(s.pos, w.brushes[end_idx])) {
				first_finish = t;
				printf("tick %5d  FINISH: grounded on brush %d  pos (%.1f,%.1f,%.2f) "
					"speed %.1f\n", t, o.end_brush, s.pos.X, s.pos.Y, s.pos.Z, sp2);
			}
			if (csv)
				fprintf(csv, "%d,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.2f,%d,%d,%d,%.4f\n",
					t, s.pos.X, s.pos.Y, s.pos.Z, s.vel.X, s.vel.Y, s.vel.Z,
					sp2, s.on_ground ? 1 : 0, s.ducked ? 1 : 0, f.buttons, f.yaw);
		}
		if (csv)
			fclose(csv);

		printf("---\n");
		printf("end of tape (%d ticks, %.3f s)\n",
			static_cast<int>(tape.frames.size()),
			tape.frames.size() * o.params.dt);
		printf("final pos (%.2f, %.2f, %.2f) vel (%.1f, %.1f, %.1f) speed %.1f "
			"ground=%d ducked=%d\n", s.pos.X, s.pos.Y, s.pos.Z,
			s.vel.X, s.vel.Y, s.vel.Z, Len2D(s.vel),
			s.on_ground ? (s.ground_brush >= 0 ? w.brushes[s.ground_brush].id : -1) : -1,
			s.ducked ? 1 : 0);
		printf("max 2D speed %.1f | startzone jumps %d | finish %s\n",
			max_speed, zone_jumps,
			first_finish >= 0 ? (std::string("tick ") + std::to_string(first_finish)
				+ " (" + std::to_string(first_finish * o.params.dt) + " s)").c_str()
			: (end_idx >= 0 ? "NOT reached" : "not checked"));
		// Zone-clock view (user-directed): the score everyone is compared on.
		if (first_finish >= 0 && zone_exit_tick >= 0)
			printf("zone clock: exit tick %d -> %d SCORED ticks (%.3f s)\n",
				zone_exit_tick, first_finish - zone_exit_tick,
				(first_finish - zone_exit_tick) * o.params.dt);
		// Tape quality truth (hard boards, yaw rates, ending class).
		{
			TapeAnchor qa = tape.start;
			if (!qa.valid) {
				qa.valid = true;
				qa.origin = w.spawn_origin;
				qa.yaw = w.spawn_yaw;
			}
			QualityScan(w, o.params, qa, tape.frames, start_id, o.end_brush,
				true);
		}
		if (!o.trace_log.empty()) {
			w.trace_log = nullptr;
			FILE* tl = nullptr;
			if (fopen_s(&tl, o.trace_log.c_str(), "w") == 0 && tl) {
				fprintf(tl, "id,tick,ax,ay,az,bx,by,bz,ducked,frac,nx,ny,nz,brush\n");
				for (size_t i = 0; i < probes.size(); ++i) {
					const World::TraceProbeRow& p = probes[i];
					fprintf(tl, "%d,%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%d,"
						"%.9g,%.6f,%.6f,%.6f,%d\n",
						static_cast<int>(i), p.tick,
						p.a.X, p.a.Y, p.a.Z, p.b.X, p.b.Y, p.b.Z,
						p.ducked ? 1 : 0, p.frac, p.n.X, p.n.Y, p.n.Z,
						p.brush_id);
				}
				fclose(tl);
				printf("trace log: %d queries -> %s\n",
					static_cast<int>(probes.size()), o.trace_log.c_str());
			} else {
				printf("trace log: cannot open %s\n", o.trace_log.c_str());
			}
		}
		fflush(stdout);
		return 0;
	}

	// ---- FULL-MAP GEOMETRY SWEEP (user directive 2026-08-15: perfect
	// parity for world geometry under ANY inputs, certified in ONE oracle
	// click). Generates hull-trace queries bracketing every brush face,
	// edge, and corner on the map - normal crossings, tangential skims,
	// seam-parallel passes - for both engine-traceable hulls. The in-game
	// oracle answers them all with IEngineTrace; every disagreement with
	// our TraceHull becomes a concrete fix case via `tracediff`, iterable
	// OFFLINE against the same results file. ----
	// ---- FUNCPROBE: per-function probes + differ --------------------------
	// One function at a time. Inputs are arbitrary values of differing
	// shapes; the verdict is whether OUR function returns the engine's
	// outputs for the same inputs. No composition, so a mismatch names its
	// own defect.
	int CmdFuncGen(const std::string& map_path, const ReplayOpts& o,
	               int nprobes, unsigned seed, const std::string& out_path) {
		World w;
		std::string err;
		w.true_interval_corner = o.corner_true;
		if (!w.Load(map_path, o.hulls, &err, o.edge_bevels)) {
			printf("LOAD FAILED (bsp): %s\n", err.c_str());
			return 1;
		}
		FILE* f = nullptr;
		if (fopen_s(&f, out_path.c_str(), "w") != 0 || !f) {
			printf("funcgen: cannot open %s\n", out_path.c_str());
			return 1;
		}
		unsigned rng = seed ? seed : 1u;
		auto next = [&rng]() {
			rng = rng * 1664525u + 1013904223u;
			return (rng >> 8) & 0xFFFFu;
		};
		auto frand = [&next]() { return next() / 65535.f; };
		auto span = [&frand](float lo, float hi) { return lo + (hi - lo) * frand(); };

		// Shapes that matter to a function reading a z-velocity against a
		// threshold and probing 2 units down: values straddling the gate by
		// one ULP, zero, denormals, and both signs at every magnitude.
		const float kVz[16] = {
			0.f, -1e-30f, 1e-30f, -1e-7f, 1e-7f, -0.03125f, 0.03125f,
			139.999985f, 140.f, 140.000015f, -140.f, 250.f, -250.f,
			3499.9998f, -3500.f, 3600.f };
		// Height above a surface, including exactly the probe distance.
		const float kDz[12] = {
			-2.f, -0.03125f, 0.f, 0.001f, 0.03125f, 0.5f, 1.f, 1.999f,
			2.f, 2.001f, 8.f, 64.f };
		const float kHull[2] = { 72.f, 54.f };

		// CONTROL PROBES FIRST. Every harness this project has built lied at
		// least once (FL_ONGROUND does not set ground; m_flGravity writes are
		// ignored), and each lie looked exactly like a healthy run. So the
		// batch opens with states the ENGINE ITSELF recorded as grounded
		// during a real captured run: same position, same velocity, same
		// duck state. If an isolated call says "not grounded" there, the
		// harness is not installing what the function reads and EVERY
		// verdict from the batch is void.
		std::vector<std::string> ctrl;
		if (!o.anchor_tas.empty()) {   // --control <playback csv>
			FILE* cf = nullptr;
			if (fopen_s(&cf, o.anchor_tas.c_str(), "r") == 0 && cf) {
				char l[512];
				while (fgets(l, sizeof(l), cf)) {
					if (l[0] == '#' || l[0] == 't') continue;
					int tk, g, dk, bt;
					float x, y, z, vx, vy, vz, sp, yw;
					if (sscanf_s(l, "%d,%f,%f,%f,%f,%f,%f,%f,%d,%d,%d,%f",
						&tk, &x, &y, &z, &vx, &vy, &vz, &sp, &g, &dk, &bt,
						&yw) < 12)
						continue;
					if (!g)            // only ticks the engine had GROUNDED
						continue;
					// The flag goes in CLEARED. A control that arrives already
					// grounded only proves the function did not CLEAR it -
					// which is not the direction under test and is exactly
					// how the first control block fooled me. Forcing 0 means
					// the engine must SET ground for the control to pass.
					char buf[400];
					_snprintf_s(buf, sizeof(buf), _TRUNCATE,
						"CategorizePosition,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"
						"0,0,0,0,%d,0,0,0,0,1,1,%.9g,%.9g,0,0,0",
						x, y, z, vx, vy, vz, dk, dk ? 54.f : 72.f, yw);
					ctrl.push_back(buf);
					if (ctrl.size() >= 400)
						break;
				}
				fclose(cf);
			}
		}

		fprintf(f, "# funcprobe probes v2 seed %u map %s\n", seed,
			MapStem(map_path).c_str());
		fprintf(f, "# control %d\n", static_cast<int>(ctrl.size()));
		fprintf(f, "fn,ox,oy,oz,vx,vy,vz,bx,by,bz,onground,ducked,ducking,"
			"buttons,ducktime,stamina,sfric,gravity,hullmaxz,yaw,fmove,smove,"
			"oldbuttons\n");
		for (const std::string& c : ctrl)
			fprintf(f, "%s\n", c.c_str());

		int wrote = static_cast<int>(ctrl.size());
		for (int i = 0; i < nprobes; ++i) {
			Vec3 pos;
			// Half the probes sit a chosen distance above a real face (any
			// face, any orientation); half are free-floating anywhere.
			if (!w.brushes.empty() && (i & 1)) {
				const WorldBrush& b = w.brushes[next() % w.brushes.size()];
				const int pi = static_cast<int>(next()
					% static_cast<unsigned>(b.nsides > 0 ? b.nsides : 1));
				const Vec3 n = b.n[pi];
				Vec3 q(span(b.bmin.X, b.bmax.X), span(b.bmin.Y, b.bmax.Y),
					span(b.bmin.Z, b.bmax.Z));
				q = q - Scale(n, Dot(n, q) - b.d[pi]);
				pos = q + Scale(n, kDz[next() % 12]);
			} else {
				pos = Vec3(span(-2600.f, 2600.f), span(-2600.f, 1600.f),
					span(-1400.f, 1600.f));
			}
			const float vz = kVz[next() % 16];
			const int ducked = (next() % 3u == 0u) ? 1 : 0;
			// The hull FOLLOWS the duck flag: the engine's trace box comes
			// from GetPlayerMins/Maxs (derived from m_bDucked), not from the
			// collision-bounds netvar we write, so varying them separately
			// would just make the two sides sweep different boxes.
			const float hull = ducked ? 54.f : 72.f;
			fprintf(f, "CategorizePosition,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"
				"0,0,0,%d,%d,%d,%d,%.9g,%.9g,%.9g,1,%.9g,%.9g,0,0,0\n",
				pos.X, pos.Y, pos.Z,
				span(-900.f, 900.f), span(-900.f, 900.f), vz,
				(next() & 1) ? 1 : 0,                       // onground
				ducked,
				(next() % 4u == 0u) ? 1 : 0,                // ducking
				0,                                          // buttons
				(next() % 3u == 0u) ? span(0.f, 1000.f) : 0.f,
				(next() % 3u == 0u) ? span(0.f, 1400.f) : 0.f,
				(next() & 1) ? 0.25f : 1.f,                 // sfric in
				hull, span(-180.f, 180.f));
			wrote++;

			// CheckJumpButton, on the same state: IN_JUMP held, stamina swept
			// across the ladder (the tax scales the WHOLE post-impulse vz),
			// ducked and standing, speeds straddling the bunnyhop clamp. The
			// pin declares a CategorizePosition prelude, so ground comes from
			// geometry - which is why these reuse the same positions.
			const float kStam[8] = { 0.f, 1.f, 100.f, 500.f,
				25000.f / 19.f - 1.f, 25000.f / 19.f, 1400.f, 5000.f };
			fprintf(f, "CheckJumpButton,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"
				"0,0,0,1,%d,%d,%d,%.9g,%.9g,1,1,%.9g,%.9g,0,0,0\n",
				pos.X, pos.Y, pos.Z,
				span(-900.f, 900.f), span(-900.f, 900.f),
				(next() & 1) ? 0.f : vz,
				ducked,
				(next() % 5u == 0u) ? 1 : 0,                // ducking
				2,                                          // IN_JUMP
				(next() % 3u == 0u) ? span(0.f, 1000.f) : 0.f,
				kStam[next() % 8],
				hull, span(-180.f, 180.f));
			wrote++;

			// Duck, on the same position: every press/hold/release EDGE
			// (buttons x oldbuttons over IN_DUCK), every duck-state combo,
			// the shared down-counter swept across both transition
			// boundaries (400ms duck, 200ms unduck), grounded and airborne,
			// with nonzero fmove/smove so the 0.34 HandleDuckingSpeedCrop is
			// observable in the fwd/side readbacks. Prelude derives ground.
			{
				const float kDt[13] = { 0.f, 1.f, 100.f, 200.f, 399.f, 400.f,
					401.f, 599.f, 600.f, 601.f, 800.f, 999.f, 1000.f };
				const int db = (next() & 1) ? IN_DUCK : 0;
				const int dob = (next() & 1) ? IN_DUCK : 0;
				const int dducked = (next() & 1) ? 1 : 0;
				const int dducking = (next() & 1) ? 1 : 0;
				fprintf(f, "Duck,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"
					"0,0,0,1,%d,%d,%d,%.9g,0,1,1,%.9g,%.9g,%.9g,%.9g,%d\n",
					pos.X, pos.Y, pos.Z,
					span(-450.f, 450.f), span(-450.f, 450.f),
					(next() & 1) ? 0.f : vz,
					dducked, dducking, db,
					kDt[next() % 13],
					dducked ? 54.f : 72.f, span(-180.f, 180.f),
					span(-450.f, 450.f), span(-450.f, 450.f), dob);
				wrote++;
			}

			// CanUnduck: ducked states, grounded and embedded and airborne -
			// the engine's shift comes from -0.5 * the hull delta and a real
			// sweep; ours from the capture-fitted -8.5 point test. This
			// grade arbitrates them (unduck_face 14/15 is the suspect).
			if ((i & 3) == 0) {
				fprintf(f, "CanUnduck,%.9g,%.9g,%.9g,0,0,%.9g,"
					"0,0,0,1,1,%d,%d,%.9g,0,1,1,54,%.9g,0,0,%d\n",
					pos.X, pos.Y, pos.Z,
					(next() & 1) ? 0.f : vz,
					(next() & 1) ? 1 : 0,                   // ducking
					IN_DUCK,
					(next() & 1) ? 600.f : 0.f,
					span(-180.f, 180.f),
					IN_DUCK);
				wrote++;
			}

			// The remaining pinned duck functions, standalone - each is a
			// leaf the engine's Duck composes, so each gets its own deck.
			if ((i & 3) == 1) {
				const int fd_d = (next() & 1) ? 1 : 0;
				fprintf(f, "FinishDuck,%.9g,%.9g,%.9g,0,0,%.9g,"
					"0,0,0,1,%d,%d,0,%.9g,0,1,1,%.9g,%.9g,0,0,0\n",
					pos.X, pos.Y, pos.Z, (next() & 1) ? 0.f : vz,
					fd_d, (next() & 1) ? 1 : 0,
					(next() & 1) ? 600.f : 999.f,
					fd_d ? 54.f : 72.f, span(-180.f, 180.f));
				wrote++;
			}
			if ((i & 3) == 2) {
				const int fu_d = (next() & 1) ? 1 : 0;
				fprintf(f, "FinishUnDuck,%.9g,%.9g,%.9g,0,0,%.9g,"
					"0,0,0,1,%d,%d,0,%.9g,0,1,1,%.9g,%.9g,0,0,0\n",
					pos.X, pos.Y, pos.Z, (next() & 1) ? 0.f : vz,
					fu_d, (next() & 1) ? 1 : 0,
					(next() & 1) ? 800.f : 0.f,
					fu_d ? 54.f : 72.f, span(-180.f, 180.f));
				wrote++;
			}
			if ((i & 3) == 3) {
				const int hc_d = (next() & 1) ? 1 : 0;
				fprintf(f, "HandleDuckingSpeedCrop,%.9g,%.9g,%.9g,0,0,0,"
					"0,0,0,1,%d,%d,%d,0,0,1,1,%.9g,%.9g,%.9g,%.9g,0\n",
					pos.X, pos.Y, pos.Z,
					hc_d, (next() & 1) ? 1 : 0,
					(next() & 1) ? IN_DUCK : 0,
					hc_d ? 54.f : 72.f, span(-180.f, 180.f),
					span(-450.f, 450.f), span(-450.f, 450.f));
				wrote++;
			}
		}

		// ReduceTimers: pure timer drains - value sweeps across every
		// boundary (zero, sub-tick remainders, the exact stamina arm).
		{
			const float kT[10] = { 0.f, 0.001f, 1.f, 14.999f, 15.f, 15.001f,
				200.f, 400.f, 1000.f, 25000.f / 19.f };
			for (int a = 0; a < 10; ++a)
			for (int b = 0; b < 10; ++b) {
				fprintf(f, "ReduceTimers,%.9g,%.9g,%.9g,0,0,0,"
					"0,0,0,1,0,%d,0,%.9g,%.9g,1,1,72,0,0,0,0\n",
					span(-2000.f, 2000.f), span(-2000.f, 1000.f),
					span(-1000.f, 1400.f),
					(a & 1),
					kT[a], kT[b]);
				wrote++;
			}
		}

		// TARGETED batteries at DISPUTED sites (--sites <mismatch csv>, the
		// manifest funcdiff wrote): each site gets a dense duck-family
		// battery on a small jitter fan - the isolated functions answer at
		// exactly the positions the model and engine disagree about.
		if (!o.sites_csv.empty()) {
			FILE* sf = nullptr;
			if (fopen_s(&sf, o.sites_csv.c_str(), "r") == 0 && sf) {
				char l[512];
				int sites = 0;
				const float sj[3] = { 0.f, -0.5f, 0.5f };
				while (fgets(l, sizeof(l), sf)) {
					if (l[0] == '#' || l[0] == 'f') continue;
					char fn[48] = "";
					float sx, sy, sz, cz;
					int sd, sg;
					if (sscanf_s(l, "%47[^,],%f,%f,%f,%d,%d,%f",
						fn, static_cast<unsigned>(sizeof(fn)),
						&sx, &sy, &sz, &sd, &sg, &cz) != 7)
						continue;
					sites++;
					for (int jx = 0; jx < 3; ++jx)
					for (int jz = 0; jz < 3; ++jz) {
						const float x = sx + sj[jx], y = sy, z = sz + sj[jz];
						// CanUnduck from ducked, both vz classes.
						fprintf(f, "CanUnduck,%.9g,%.9g,%.9g,0,0,0,"
							"0,0,0,1,1,0,%d,0,0,1,1,54,0,0,0,%d\n",
							x, y, z, IN_DUCK, IN_DUCK);
						wrote++;
						// FinishDuck from both already-states.
						fprintf(f, "FinishDuck,%.9g,%.9g,%.9g,0,0,0,"
							"0,0,0,1,%d,1,0,600,0,1,1,%.9g,0,0,0,0\n",
							x, y, z, jx & 1, (jx & 1) ? 54.f : 72.f);
						wrote++;
						// Duck release edge (the blocked-family shape).
						fprintf(f, "Duck,%.9g,%.9g,%.9g,0,0,0,"
							"0,0,0,1,1,0,0,%.9g,0,1,1,54,0,0,0,%d\n",
							x, y, z, (jz & 1) ? 999.f : 200.f, IN_DUCK);
						wrote++;
					}
				}
				fclose(sf);
				printf("funcgen: targeted batteries at %d disputed site(s)\n",
					sites);
			}
		}
		fclose(f);
		printf("funcgen: %d probes -> %s\n", wrote, out_path.c_str());
		printf("funcgen: in-game Map Solve tab -> 'Run FUNCPROBE', then\n"
			"  SolverLab funcdiff <map.bsp> \"%s\" <func_results.csv>\n",
			out_path.c_str());
		fflush(stdout);
		return 0;
	}

	int CmdFuncDiff(const std::string& map_path, const ReplayOpts& o,
	                const std::string& ppath, const std::string& rpath) {
		World w;
		std::string err;
		w.true_interval_corner = o.corner_true;
		if (!w.Load(map_path, o.hulls, &err, o.edge_bevels)) {
			printf("LOAD FAILED (bsp): %s\n", err.c_str());
			return 1;
		}
		struct P {
			char fn[48];
			float ox, oy, oz, vx, vy, vz, bx, by, bz;
			int onground, ducked, ducking, buttons;
			float ducktime, stamina, sfric, gravity, hullmaxz, yaw, fmove, smove;
			int oldbuttons;
		};
		std::vector<P> ps;
		{
			FILE* f = nullptr;
			if (fopen_s(&f, ppath.c_str(), "r") != 0 || !f) {
				printf("funcdiff: cannot read %s\n", ppath.c_str());
				return 1;
			}
			char l[512];
			while (fgets(l, sizeof(l), f)) {
				if (l[0] == '#' || l[0] == 'f') continue;
				P q = {};
				const int nf = sscanf_s(l,
					"%47[^,],%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d,%d,"
					"%d,%f,%f,%f,%f,%f,%f,%f,%f,%d",
					q.fn, static_cast<unsigned>(sizeof(q.fn)),
					&q.ox, &q.oy, &q.oz, &q.vx, &q.vy, &q.vz,
					&q.bx, &q.by, &q.bz, &q.onground, &q.ducked, &q.ducking,
					&q.buttons, &q.ducktime, &q.stamina, &q.sfric, &q.gravity,
					&q.hullmaxz, &q.yaw, &q.fmove, &q.smove, &q.oldbuttons);
				if (nf == 22 || nf == 23) {
					if (nf == 22)
						q.oldbuttons = 0;
					ps.push_back(q);
				}
			}
			fclose(f);
		}
		struct R {
			char fn[48];
			float ox, oy, oz, vx, vy, vz;
			int flags, ducked, ducking;
			float ducktime, stamina, sfric, maxz;
			int ret, ok, groundent;
			float fwd, side;   // v2: post-call mv fwd/side (the 0.34 crop)
		};
		int nctrl = 0;
		{
			FILE* f = nullptr;
			if (fopen_s(&f, ppath.c_str(), "r") == 0 && f) {
				char l[256];
				while (fgets(l, sizeof(l), f))
					if (sscanf_s(l, "# control %d", &nctrl) == 1)
						break;
				fclose(f);
			}
		}
		std::vector<R> rs;
		int ctxgate = -1;
		{
			FILE* f = nullptr;
			if (fopen_s(&f, rpath.c_str(), "r") != 0 || !f) {
				printf("funcdiff: cannot read %s\n", rpath.c_str());
				return 1;
			}
			char l[512];
			int ver = 1;
			while (fgets(l, sizeof(l), f)) {
				if (strncmp(l, "# funcprobe", 11) == 0) {
					sscanf_s(l, "# funcprobe v%d ctxgate %d", &ver, &ctxgate);
					continue;
				}
				if (l[0] == '#' || l[0] == 'i') continue;
				R r = {};
				int id = 0;
				r.groundent = 0;
				// v2 rows: ...,ret,ok,fwd,side,groundent. v1: ...,ret,ok,groundent.
				const int nf = (ver >= 2)
					? sscanf_s(l, "%d,%47[^,],%f,%f,%f,%f,%f,%f,%d,%d,%d,"
						"%f,%f,%f,%f,%d,%d,%f,%f,%d",
						&id, r.fn, static_cast<unsigned>(sizeof(r.fn)),
						&r.ox, &r.oy, &r.oz, &r.vx, &r.vy, &r.vz,
						&r.flags, &r.ducked, &r.ducking, &r.ducktime, &r.stamina,
						&r.sfric, &r.maxz, &r.ret, &r.ok, &r.fwd, &r.side,
						&r.groundent)
					: sscanf_s(l, "%d,%47[^,],%f,%f,%f,%f,%f,%f,%d,%d,%d,"
						"%f,%f,%f,%f,%d,%d,%d",
						&id, r.fn, static_cast<unsigned>(sizeof(r.fn)),
						&r.ox, &r.oy, &r.oz, &r.vx, &r.vy, &r.vz,
						&r.flags, &r.ducked, &r.ducking, &r.ducktime, &r.stamina,
						&r.sfric, &r.maxz, &r.ret, &r.ok, &r.groundent);
				if (nf >= 17)
					rs.push_back(r);
			}
			fclose(f);
		}
		if (ctxgate != 1) {
			printf("funcdiff: REFUSING to grade - context gate %d (need 1). "
				"The CGameMovement member offsets were not confirmed against "
				"the live hook, so every call wrote through unverified "
				"offsets.\n", ctxgate);
			return 1;
		}
		const size_t n = ps.size() < rs.size() ? ps.size() : rs.size();
		// CONTROL VERDICT BEFORE ANY GRADING.
		if (nctrl > 0) {
			int ok_g = 0, bad_g = 0;
			for (int i = 0; i < nctrl && i < static_cast<int>(n); ++i) {
				if (!rs[i].ok) continue;
				// m_hGroundEntity: a valid handle means grounded. An unset
				// EHANDLE reads 0xFFFFFFFF (-1); 0 also means none.
				const bool eg = rs[i].groundent != -1 && rs[i].groundent != 0;
				if (eg) ok_g++; else bad_g++;
			}
			printf("funcdiff: CONTROL %d states the engine recorded as "
				"GROUNDED in a real run, fed in with the flag CLEARED -> "
				"isolated call SET ground %d, left it clear %d\n",
				nctrl, ok_g, bad_g);
			if (bad_g > 0) {
				printf("funcdiff: HARNESS VOID - the isolated call failed to "
					"SET ground on states the engine itself recorded as "
					"grounded. It can evidently only preserve or clear the "
					"flag, so every 'ours grounds, engine does not' verdict "
					"in this batch is an artifact, not a defect.\n");
				return 1;
			}
			printf("funcdiff: harness VALIDATED on controls.\n");
		} else {
			printf("funcdiff: WARNING - no control probes in this batch "
				"(regenerate with --control <playback csv>); grading an "
				"unvalidated harness.\n");
		}
		int exact = 0, bad = 0, faulted = 0, shown = 0;
		int jexact = 0, jbad = 0, jshown = 0;
		int dexact = 0, dbad = 0, dshown = 0;
		int uexact = 0, ubad = 0, ushown = 0;
		int fdexact = 0, fdbad = 0, fdshown = 0;
		int fuexact = 0, fubad = 0;
		int hcexact = 0, hcbad = 0;
		int rtexact = 0, rtbad = 0;
		int crop_match = 0, crop_bad = 0;
		// Mismatch MANIFEST: every graded miss becomes one row that
		// tracegen-quads can turn into direct box-oracle questions.
		std::vector<std::string> mm;
		auto note_mm = [&](const char* fn, const P& q, bool pre_ground,
		                   float candz) {
			char b[256];
			_snprintf_s(b, sizeof(b), _TRUNCATE, "%s,%.9g,%.9g,%.9g,%d,%d,%.9g",
				fn, q.ox, q.oy, q.oz, q.ducked, pre_ground ? 1 : 0, candz);
			mm.push_back(b);
		};
		for (size_t i = 0; i < n; ++i) {
			if (!rs[i].ok) { faulted++; continue; }
			const bool is_jump = strcmp(ps[i].fn, "CheckJumpButton") == 0;
			const bool is_duck = strcmp(ps[i].fn, "Duck") == 0;
			const bool is_unduck = strcmp(ps[i].fn, "CanUnduck") == 0;
			PlayerState s;
			s.pos = Vec3(ps[i].ox, ps[i].oy, ps[i].oz);
			s.vel = Vec3(ps[i].vx, ps[i].vy, ps[i].vz);
			// Every probe enters ungrounded (see the runner: the ground input
			// is unsettable, so it is not pretended to vary). The function
			// under test must SET ground from geometry, and so must we.
			s.on_ground = false;
			s.ducked = ps[i].ducked != 0;
			s.ducking = ps[i].ducking != 0;
			s.duck_timer_ms = ps[i].ducktime;
			s.stamina = ps[i].stamina;
			s.gravity_scale = ps[i].gravity;
			s.surface_friction = ps[i].sfric;
			s.old_buttons = ps[i].oldbuttons;
			// HULL FOLLOWS THE DUCK FLAG, not our m_vecMaxs write: the engine
			// takes its trace box from GetPlayerMins/Maxs, which are derived
			// from m_bDucked and ignore the collision-bounds netvar. Reading
			// the hull from the probe's hullmaxz let the two sides disagree
			// about the box being swept.
			s.hull_state = ps[i].ducked ? 1 : 0;
			// Same call sequence the pin declares: the prelude first, then
			// the function under test.
			Fn::CategorizePosition(s, w, o.params);
			const bool pre_g = s.on_ground;
			const bool is_fduck = strcmp(ps[i].fn, "FinishDuck") == 0;
			const bool is_funduck = strcmp(ps[i].fn, "FinishUnDuck") == 0;
			const bool is_crop = strcmp(ps[i].fn, "HandleDuckingSpeedCrop") == 0;
			if (is_fduck || is_funduck) {
				if (is_fduck)
					Fn::FinishDuck(s, w, o.params);
				else
					Fn::FinishUnDuck(s, w, o.params);
				const bool eng_fl = (rs[i].flags & 2) != 0;
				const float dp = Len(s.pos - Vec3(rs[i].ox, rs[i].oy, rs[i].oz));
				const bool fok = (rs[i].ducked != 0) == s.ducked
					&& (rs[i].ducking != 0) == s.ducking
					&& fabsf(rs[i].ducktime - s.duck_timer_ms) <= 0.01f
					&& eng_fl == s.ducked
					&& dp <= 0.001f;
				if (is_fduck) { if (fok) fdexact++; else fdbad++; }
				else { if (fok) fuexact++; else fubad++; }
				if (!fok) {
					note_mm(ps[i].fn, ps[i], pre_g,
						pre_g ? ps[i].oz : ps[i].oz - o.params.duck_air_shift);
					if (is_fduck && fdshown < 6) {
						fdshown++;
						printf("  %s probe %d  in d%d k%d g%d\n", ps[i].fn,
							static_cast<int>(i), ps[i].ducked, ps[i].ducking,
							pre_g ? 1 : 0);
						printf("    eng  d%d k%d t%.1f pos(%.3f,%.3f,%.3f)\n",
							rs[i].ducked, rs[i].ducking, rs[i].ducktime,
							rs[i].ox, rs[i].oy, rs[i].oz);
						printf("    ours d%d k%d t%.1f pos(%.3f,%.3f,%.3f)\n",
							s.ducked ? 1 : 0, s.ducking ? 1 : 0,
							s.duck_timer_ms, s.pos.X, s.pos.Y, s.pos.Z);
					}
				}
				continue;
			}
			if (is_crop) {
				float fwd = ps[i].fmove, side = ps[i].smove;
				Fn::HandleDuckingSpeedCrop(s, ps[i].buttons, &fwd, &side);
				const bool cok = fabsf(rs[i].fwd - fwd) <= 0.01f
					&& fabsf(rs[i].side - side) <= 0.01f;
				if (cok) hcexact++; else { hcbad++;
					note_mm(ps[i].fn, ps[i], pre_g, ps[i].oz); }
				continue;
			}
			if (strcmp(ps[i].fn, "ReduceTimers") == 0) {
				Fn::ReduceTimers(s, o.params);
				const bool rok =
					fabsf(rs[i].ducktime - s.duck_timer_ms) <= 0.001f
					&& fabsf(rs[i].stamina - s.stamina) <= 0.001f;
				if (rok) rtexact++; else { rtbad++;
					note_mm(ps[i].fn, ps[i], pre_g, ps[i].oz); }
				continue;
			}
			if (is_duck) {
				Fn::Duck(s, w, o.params, ps[i].buttons);
				const bool eng_flduck = (rs[i].flags & 2) != 0;
				const float dp = Len(s.pos - Vec3(rs[i].ox, rs[i].oy, rs[i].oz));
				const bool dok = (rs[i].ducked != 0) == s.ducked
					&& (rs[i].ducking != 0) == s.ducking
					&& fabsf(rs[i].ducktime - s.duck_timer_ms) <= 0.01f
					&& eng_flduck == s.ducked
					&& dp <= 0.001f;
				// The 0.34 speed crop is DIAGNOSTIC this round: whether Duck
				// itself crops (vs a later PlayerMove call site) is exactly
				// what the engine's own fwd/side outputs are about to state.
				const bool crop = (ps[i].buttons & 4) || s.ducking || s.ducked;
				const float exp_fwd = ps[i].fmove * (crop ? 0.34f : 1.f);
				if (fabsf(rs[i].fwd - exp_fwd) <= 0.01f) crop_match++;
				else crop_bad++;
				if (dok) { dexact++; continue; }
				dbad++;
				note_mm("Duck", ps[i], pre_g,
					pre_g ? ps[i].oz : ps[i].oz - o.params.duck_air_shift);
				if (dshown < 8) {
					dshown++;
					printf("  DUCK probe %d  btn %d old %d in d%d k%d t%.0f "
						"ground %d\n", static_cast<int>(i), ps[i].buttons,
						ps[i].oldbuttons, ps[i].ducked, ps[i].ducking,
						ps[i].ducktime, s.on_ground ? 1 : 0);
					printf("    eng  d%d k%d t%.1f fl%d pos(%.3f,%.3f,%.3f)\n",
						rs[i].ducked, rs[i].ducking, rs[i].ducktime,
						eng_flduck ? 1 : 0, rs[i].ox, rs[i].oy, rs[i].oz);
					printf("    ours d%d k%d t%.1f fl%d pos(%.3f,%.3f,%.3f)\n",
						s.ducked ? 1 : 0, s.ducking ? 1 : 0, s.duck_timer_ms,
						s.ducked ? 1 : 0, s.pos.X, s.pos.Y, s.pos.Z);
				}
				continue;
			}
			if (is_unduck) {
				const bool ours = Fn::CanUnduck(s, w, o.params);
				const bool eng = (rs[i].ret & 0xff) != 0;
				if (ours == eng) { uexact++; continue; }
				ubad++;
				note_mm("CanUnduck", ps[i], pre_g,
					pre_g ? ps[i].oz : ps[i].oz - o.params.duck_air_shift);
				if (ushown < 8) {
					ushown++;
					printf("  CANUNDUCK probe %d  pos(%.3f,%.3f,%.3f) ground %d"
						"  eng %d ours %d\n", static_cast<int>(i), ps[i].ox,
						ps[i].oy, ps[i].oz, s.on_ground ? 1 : 0,
						eng ? 1 : 0, ours ? 1 : 0);
				}
				continue;
			}
			if (is_jump) {
				const Vec3 pre_v = s.vel;
				const bool jumped = Fn::CheckJumpButton(s, w, o.params);
				// The engine function returns bool in AL and never clears the
				// rest of EAX (xor al,al / mov al,1 in the body) - the upper
				// 24 bits of the captured int are stack garbage. AL is exact.
				const bool eng_jumped = (rs[i].ret & 0xff) != 0;
				const float dv = Len(s.vel - Vec3(rs[i].vx, rs[i].vy, rs[i].vz));
				const bool jok = jumped == eng_jumped && dv <= 0.01f
					&& fabsf(rs[i].stamina - s.stamina) <= 0.01f;
				if (jok) { jexact++; continue; }
				jbad++;
				note_mm("CheckJumpButton", ps[i], pre_g, ps[i].oz);
				if (jshown < 8) {
					jshown++;
					printf("  JUMP probe %d  pre vz %.3f stam %.2f duck %d "
						"ground %d\n", static_cast<int>(i), pre_v.Z,
						ps[i].stamina, ps[i].ducked, s.on_ground ? 1 : 0);
					printf("    eng  jumped %d vel(%.3f,%.3f,%.3f) stam %.2f\n",
						eng_jumped ? 1 : 0, rs[i].vx, rs[i].vy, rs[i].vz,
						rs[i].stamina);
					printf("    ours jumped %d vel(%.3f,%.3f,%.3f) stam %.2f\n",
						jumped ? 1 : 0, s.vel.X, s.vel.Y, s.vel.Z, s.stamina);
				}
				continue;
			}
			const bool eng_ground = rs[i].groundent != -1 && rs[i].groundent != 0;
			const float dp = Len(s.pos - Vec3(rs[i].ox, rs[i].oy, rs[i].oz));
			const float dv = Len(s.vel - Vec3(rs[i].vx, rs[i].vy, rs[i].vz));
			const bool ok = dp <= 0.001f && dv <= 0.001f
				&& eng_ground == s.on_ground
				&& fabsf(rs[i].sfric - s.surface_friction) <= 0.0001f;
			if (ok) { exact++; continue; }
			bad++;
			note_mm("CategorizePosition", ps[i], pre_g, ps[i].oz);
			if (shown < 10) {
				shown++;
				printf("  probe %d  in pos(%.3f,%.3f,%.3f) vz %.4f hull %.0f "
					"sfric %.2f\n", static_cast<int>(i), ps[i].ox, ps[i].oy,
					ps[i].oz, ps[i].vz, ps[i].hullmaxz, ps[i].sfric);
				printf("    eng  ground %d sfric %.4f pos(%.3f,%.3f,%.3f)\n",
					eng_ground ? 1 : 0, rs[i].sfric, rs[i].ox, rs[i].oy, rs[i].oz);
				printf("    ours ground %d sfric %.4f pos(%.3f,%.3f,%.3f)\n",
					s.on_ground ? 1 : 0, s.surface_friction,
					s.pos.X, s.pos.Y, s.pos.Z);
			}
		}
		printf("funcdiff: CategorizePosition | %d probes | EXACT %d | "
			"MISMATCH %d | engine-faulted %d\n",
			exact + bad, exact, bad, faulted);
		if (jexact + jbad > 0)
			printf("funcdiff: CheckJumpButton    | %d probes | EXACT %d | "
				"MISMATCH %d\n", jexact + jbad, jexact, jbad);
		if (dexact + dbad > 0) {
			printf("funcdiff: Duck               | %d probes | EXACT %d | "
				"MISMATCH %d\n", dexact + dbad, dexact, dbad);
			printf("  speed-crop diagnostic (Duck crops fwd by 0.34 itself?): "
				"consistent %d, inconsistent %d\n", crop_match, crop_bad);
		}
		if (uexact + ubad > 0)
			printf("funcdiff: CanUnduck          | %d probes | EXACT %d | "
				"MISMATCH %d\n", uexact + ubad, uexact, ubad);
		if (fdexact + fdbad > 0)
			printf("funcdiff: FinishDuck         | %d probes | EXACT %d | "
				"MISMATCH %d\n", fdexact + fdbad, fdexact, fdbad);
		if (fuexact + fubad > 0)
			printf("funcdiff: FinishUnDuck       | %d probes | EXACT %d | "
				"MISMATCH %d\n", fuexact + fubad, fuexact, fubad);
		if (hcexact + hcbad > 0)
			printf("funcdiff: HandleDuckingSpeedCrop | %d probes | EXACT %d | "
				"MISMATCH %d\n", hcexact + hcbad, hcexact, hcbad);
		if (rtexact + rtbad > 0)
			printf("funcdiff: ReduceTimers       | %d probes | EXACT %d | "
				"MISMATCH %d\n", rtexact + rtbad, rtexact, rtbad);
		// Write the mismatch manifest beside the results file - the input
		// tracegen-quads turns into direct box-oracle questions.
		{
			std::string dir = rpath;
			const size_t cut = dir.find_last_of("\\/");
			dir = cut == std::string::npos ? "" : dir.substr(0, cut + 1);
			const std::string mpath = dir + "func_mismatch.csv";
			FILE* mf = nullptr;
			if (fopen_s(&mf, mpath.c_str(), "w") == 0 && mf) {
				fprintf(mf, "fn,ox,oy,oz,ducked,grounded,candz\n");
				for (const std::string& l : mm)
					fprintf(mf, "%s\n", l.c_str());
				fclose(mf);
				printf("funcdiff: %d mismatch rows -> %s\n",
					static_cast<int>(mm.size()), mpath.c_str());
			}
		}
		// RULE EXTRACTION, not theory: bucket the engine's ground answer by
		// our own down-probe fraction and by the plane it struck. Whatever
		// separates ground from no-ground has to show up here.
		{
			struct B { int eng_g, eng_ng; };
			B by_frac[12] = {};
			B by_norm[3] = {};
			for (size_t i = 0; i < n; ++i) {
				if (!rs[i].ok) continue;
				TraceResult tr;
				const Vec3 a(ps[i].ox, ps[i].oy, ps[i].oz);
				const int hull = ps[i].hullmaxz < 60.f ? 1 : 0;
				const float fr = w.TraceHull3(a, a - Vec3(0.f, 0.f, 2.f),
					hull, &tr);
				const bool eg = (rs[i].flags & 1) != 0;
				int bi = static_cast<int>(fr * 10.f);
				if (bi < 0) bi = 0;
				if (bi > 10) bi = 10;
				if (fr >= 1.f) bi = 11;
				(eg ? by_frac[bi].eng_g : by_frac[bi].eng_ng)++;
				const int ni = tr.brush < 0 ? 0
					: (tr.normal.Z >= 0.7f ? 2 : 1);
				(eg ? by_norm[ni].eng_g : by_norm[ni].eng_ng)++;
			}
			printf("  engine GROUND by our down-probe fraction:\n");
			for (int b = 0; b <= 11; ++b) {
				if (!by_frac[b].eng_g && !by_frac[b].eng_ng) continue;
				char lbl[24];
				if (b == 11) _snprintf_s(lbl, sizeof(lbl), _TRUNCATE, "frac=1 (clear)");
				else _snprintf_s(lbl, sizeof(lbl), _TRUNCATE, "frac %.1f-%.1f",
					b * 0.1f, b * 0.1f + 0.1f);
				printf("    %-16s ground %6d  no-ground %6d\n", lbl,
					by_frac[b].eng_g, by_frac[b].eng_ng);
			}
			const char* nn[3] = { "no brush hit", "steep plane", "walkable" };
			printf("  engine GROUND by the plane we struck:\n");
			for (int b = 0; b < 3; ++b)
				printf("    %-16s ground %6d  no-ground %6d\n", nn[b],
					by_norm[b].eng_g, by_norm[b].eng_ng);
		}
		fflush(stdout);
		return bad > 0 ? 2 : 0;
	}

	// Compare our trace's SOLID FLAGS against the engine's, over the oracle
	// query file. These columns were captured all along and excluded from
	// scoring as "startsolid noise" - they are in fact the engine's own
	// definition of solidity, per input, and the only thing that can settle
	// it without inventing rules.
	int CmdSolidDiff(const std::string& map_path, const ReplayOpts& o,
	                 const std::string& qpath, const std::string& rpath) {
		World w;
		std::string err;
		w.true_interval_corner = o.corner_true;
		if (!w.Load(map_path, o.hulls, &err, o.edge_bevels)) {
			printf("LOAD FAILED (bsp): %s\n", err.c_str());
			return 1;
		}
		struct Q { Vec3 a, b; int ducked; };
		std::vector<Q> qs;
		{
			FILE* f = nullptr;
			if (fopen_s(&f, qpath.c_str(), "r") != 0 || !f) return 1;
			char l[512];
			while (fgets(l, sizeof(l), f)) {
				if (l[0] == 'i' || l[0] == '#') continue;
				int id, tk, dk, br; float ax, ay, az, bx, by, bz, fr, nx, ny, nz;
				if (sscanf_s(l, "%d,%d,%f,%f,%f,%f,%f,%f,%d,%f,%f,%f,%f,%d",
					&id, &tk, &ax, &ay, &az, &bx, &by, &bz, &dk,
					&fr, &nx, &ny, &nz, &br) == 14)
					qs.push_back({ Vec3(ax, ay, az), Vec3(bx, by, bz), dk });
			}
			fclose(f);
		}
		struct R { float frac; int startsolid, allsolid; };
		std::vector<R> rs;
		{
			FILE* f = nullptr;
			if (fopen_s(&f, rpath.c_str(), "r") != 0 || !f) return 1;
			char l[512];
			while (fgets(l, sizeof(l), f)) {
				if (l[0] == 'i' || l[0] == '#') continue;
				int id, ss, as; float fr, ex, ey, ez, nx, ny, nz, pd;
				if (sscanf_s(l, "%d,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d",
					&id, &fr, &ex, &ey, &ez, &nx, &ny, &nz, &pd, &ss, &as) == 11)
					rs.push_back({ fr, ss, as });
			}
			fclose(f);
		}
		const size_t n = qs.size() < rs.size() ? qs.size() : rs.size();
		int ss_match = 0, ss_we_only = 0, ss_eng_only = 0;
		int as_match = 0, as_we_only = 0, as_eng_only = 0;
		int shown = 0;
		for (size_t i = 0; i < n; ++i) {
			TraceResult tr;
			w.TraceHull3(qs[i].a, qs[i].b, qs[i].ducked ? 1 : 0, &tr);
			const bool ess = rs[i].startsolid != 0, eas = rs[i].allsolid != 0;
			if (tr.startsolid == ess) ss_match++;
			else if (tr.startsolid) ss_we_only++;
			else ss_eng_only++;
			if (tr.allsolid == eas) as_match++;
			else if (tr.allsolid) as_we_only++;
			else as_eng_only++;
			if ((tr.startsolid != ess || tr.allsolid != eas) && shown < 10) {
				shown++;
				printf("  q%-5d a(%9.2f,%9.2f,%9.2f) d%d  eng ss%d as%d  "
					"ours ss%d as%d\n", static_cast<int>(i),
					qs[i].a.X, qs[i].a.Y, qs[i].a.Z, qs[i].ducked,
					ess ? 1 : 0, eas ? 1 : 0,
					tr.startsolid ? 1 : 0, tr.allsolid ? 1 : 0);
			}
		}
		printf("soliddiff: %d queries\n", static_cast<int>(n));
		printf("  startsolid  match %6d | ours-only %5d | engine-only %5d\n",
			ss_match, ss_we_only, ss_eng_only);
		printf("  allsolid    match %6d | ours-only %5d | engine-only %5d\n",
			as_match, as_we_only, as_eng_only);
		fflush(stdout);
		return (ss_we_only || ss_eng_only || as_we_only || as_eng_only) ? 2 : 0;
	}

	int CmdTraceGenMap(const std::string& map_path, const ReplayOpts& o,
	                   const std::string& out_path) {
		World w;
		std::string err;
		w.true_interval_corner = o.corner_true;
		if (!w.Load(map_path, o.hulls, &err, o.edge_bevels)) {
			printf("LOAD FAILED (bsp): %s\n", err.c_str());
			return 1;
		}
		FILE* f = nullptr;
		if (fopen_s(&f, out_path.c_str(), "w") != 0 || !f) {
			printf("tracegen-map: cannot open %s\n", out_path.c_str());
			return 1;
		}
		fprintf(f, "id,tick,ax,ay,az,bx,by,bz,ducked,frac,nx,ny,nz,brush\n");
		int id = 0;
		auto emit = [&](const Vec3& a, const Vec3& b, int ducked) {
			// Our answer rides along (frac/n/brush) exactly like the replay
			// trace log; the oracle overwrites with engine truth.
			TraceResult tr;
			const float fr = w.TraceHull(a, b, ducked != 0, &tr);
			fprintf(f, "%d,%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%d,"
				"%.9g,%.6f,%.6f,%.6f,%d\n",
				id++, 0, a.X, a.Y, a.Z, b.X, b.Y, b.Z, ducked, fr,
				tr.normal.X, tr.normal.Y, tr.normal.Z,
				tr.brush >= 0 ? w.brushes[tr.brush].id : -1);
		};
		// Deterministic jitter (no wall-clock anywhere).
		unsigned rng = 0x5EED5EEDu;
		auto frand = [&rng]() {
			rng = rng * 1664525u + 1013904223u;
			return static_cast<float>((rng >> 8) & 0xFFFF) / 65535.f;
		};
		for (const WorldBrush& wb : w.brushes) {
			const Vec3 c(0.5f * (wb.bmin.X + wb.bmax.X),
				0.5f * (wb.bmin.Y + wb.bmax.Y),
				0.5f * (wb.bmin.Z + wb.bmax.Z));
			// (a) FACE probes: for every real side, points spread across the
			// brush AABB projected onto the face plane; traces crossing the
			// plane inward along -n from outside, and skims parallel to the
			// face just off the expanded surface.
			for (int pi = 0; pi < wb.nsides; ++pi) {
				const Vec3 n = wb.n[pi];
				const float d = wb.d[pi];
				for (int k = 0; k < 5; ++k) {
					// Spread point in the AABB, projected onto the plane.
					Vec3 q(wb.bmin.X + (wb.bmax.X - wb.bmin.X)
						* (k == 0 ? 0.5f : frand()),
						wb.bmin.Y + (wb.bmax.Y - wb.bmin.Y)
						* (k == 0 ? 0.5f : frand()),
						wb.bmin.Z + (wb.bmax.Z - wb.bmin.Z)
						* (k == 0 ? 0.5f : frand()));
					const float off = Dot(n, q) - d;
					q = q - Scale(n, off);   // on the raw plane
					for (int ducked = 0; ducked <= 1; ++ducked) {
						// Normal crossing: from 96u out to 32u past.
						emit(q + Scale(n, 96.f), q - Scale(n, 32.f), ducked);
						// Shallow crossing (glancing 15-degree class).
						Vec3 t(n.Y, n.Z, n.X);   // any non-parallel vector
						t = t - Scale(n, Dot(t, n));
						const float tl = Len(t);
						if (tl > 1e-4f) {
							t = Scale(t, 1.f / tl);
							emit(q + Scale(n, 24.f) - Scale(t, 200.f),
								q + Scale(t, 200.f), ducked);
						}
					}
				}
			}
			// (b) EDGE/CORNER probes: the 12 AABB edges, traces passing
			// PARALLEL to each edge just inside/outside the hull-expanded
			// boundary (the seam class the ramp-2 base divergence lives in),
			// plus diagonal corner crossings.
			const Vec3 mn = wb.bmin, mx = wb.bmax;
			const Vec3 corners[8] = {
				{mn.X, mn.Y, mn.Z}, {mx.X, mn.Y, mn.Z},
				{mn.X, mx.Y, mn.Z}, {mx.X, mx.Y, mn.Z},
				{mn.X, mn.Y, mx.Z}, {mx.X, mn.Y, mx.Z},
				{mn.X, mx.Y, mx.Z}, {mx.X, mx.Y, mx.Z} };
			static const int edges[12][2] = {
				{0,1},{2,3},{4,5},{6,7}, {0,2},{1,3},{4,6},{5,7},
				{0,4},{1,5},{2,6},{3,7} };
			for (int e = 0; e < 12; ++e) {
				const Vec3 a = corners[edges[e][0]];
				const Vec3 b = corners[edges[e][1]];
				Vec3 dir = b - a;
				const float el = Len(dir);
				if (el < 1.f)
					continue;
				dir = Scale(dir, 1.f / el);
				// Perpendicular offsets around the edge (8 clock positions
				// at hull-scale distances: skim outside, graze, overlap).
				Vec3 u(dir.Y, dir.Z, dir.X);
				u = u - Scale(dir, Dot(u, dir));
				u = Scale(u, 1.f / fmaxf(Len(u), 1e-4f));
				const Vec3 v = Vec3(
					dir.Y * u.Z - dir.Z * u.Y,
					dir.Z * u.X - dir.X * u.Z,
					dir.X * u.Y - dir.Y * u.X);
				const float offs[3] = { 12.f, 17.f, 40.f };
				for (int oi = 0; oi < 3; ++oi) {
					for (int ci = 0; ci < 4; ++ci) {
						const float ang = 0.7854f + 1.5708f * ci;
						const Vec3 off = Scale(u, cosf(ang) * offs[oi])
							+ Scale(v, sinf(ang) * offs[oi]);
						const Vec3 mid = Scale(a + b, 0.5f);
						for (int ducked = 0; ducked <= 1; ++ducked)
							emit(a + off - Scale(dir, 48.f),
								b + off + Scale(dir, 48.f), ducked);
						// Crossing THROUGH the edge at this clock position.
						emit(mid + Scale(off, 3.f), mid - off, 0);
					}
				}
			}
		}
		fclose(f);
		printf("tracegen-map: %d queries -> %s\n", id, out_path.c_str());
		printf("tracegen-map: in-game 'Run trace oracle' answers them; then\n"
			"  SolverLab tracediff \"%s\" <results.csv>\n", out_path.c_str());
		fflush(stdout);
		return 0;
	}

	// strafelaw: PROVE the closed-form air-strafe law (SolverStrafe.h)
	// against the certified engine mirror. Sweeps speed x sfric x duck x
	// wish angle, runs ONE real MoveTick per point high in open air, and
	// compares the analytic speed^2 delta and heading turn against what
	// the mirror actually did. This is the bedrock of the regret ledger -
	// it must be EXACT, not close.
	int CmdStrafeLaw(const std::string& map_path, const ReplayOpts& o) {
		World w;
		std::string err;
		w.true_interval_corner = o.corner_true;
		if (!w.Load(map_path, o.hulls, &err, o.edge_bevels)) {
			printf("LOAD FAILED (bsp): %s\n", err.c_str());
			return 1;
		}
		const float kV[8] = { 30.f, 100.f, 300.f, 600.f, 1000.f, 1800.f,
			2600.f, 3400.f };
		const float kSf[2] = { 1.f, 0.25f };
		float max_dg = 0.f, max_dt = 0.f;
		int n = 0;
		for (int vi = 0; vi < 8; ++vi)
		for (int si = 0; si < 2; ++si)
		for (int du = 0; du < 2; ++du)
		for (int ad = 5; ad < 180; ad += 5) {
			const float v = kV[vi];
			const float alpha = Deg2Rad(static_cast<float>(ad));
			PlayerState s;
			// z=3000: far above the map shell - the certified world traces
			// CLEAR out there for swept rays, so the tick is pure air
			// physics with no contact contamination (the first version sat
			// at (0,0,900), inside the spine's airspace, and graded clip
			// deflections as "law deviation").
			s.pos = Vec3(0.f, 0.f, 3000.f);
			s.vel = Vec3(v, 0.f, 0.f);
			s.on_ground = false;
			s.ducked = du != 0;
			s.hull_state = du ? 1 : 0;
			s.surface_friction = kSf[si];
			// Wish at angle alpha from velocity (velocity heading = 0 deg).
			// fmove-forward wish: yaw IS the wish direction.
			const float yaw = ad;   // degrees
			TickEvents ev;
			// Ducked samples HOLD the duck button - without it the tick
			// UNDUCKS at Duck() before the wish is computed and the sample
			// silently grades the standing law (the first version's only
			// "deviation" was exactly this validator bug).
			MoveTick(s, w, o.params, 0.f, yaw, 450.f, 0.f, 0.f,
				du ? IN_DUCK : 0, &ev);
			if (ev.ncontacts > 0 || s.on_ground)
				continue;   // contaminated tick: not an air-law sample
			const float sp2 = s.vel.X * s.vel.X + s.vel.Y * s.vel.Y;
			const float heading = atan2f(s.vel.Y, s.vel.X);
			Strafe::TickLaw law = Strafe::Law(o.params, v, kSf[si], du != 0);
			const float ana2 = law.NewSpeed2(cosf(alpha));
			const float anaT = law.TurnRad(cosf(alpha), sinf(alpha));
			const float dg = fabsf(sp2 - ana2);
			const float dt = fabsf(heading - anaT);
			if (dg > max_dg) {
				max_dg = dg;
				printf("  worst dg so far: v %.0f sfric %.2f duck %d a %d | "
					"eng sp2 %.1f ana %.1f | eng turn %.4f ana %.4f\n",
					v, kSf[si], du, ad, sp2, ana2, heading, anaT);
			}
			if (dt > max_dt) max_dt = dt;
			n++;
		}
		printf("strafelaw: %d points | max |dspeed^2| %.6f | max |dturn| %.8f rad\n",
			n, max_dg, max_dt);
		// Tolerance is ULP-relative: one ULP of speed^2 at 3400 u/s is ~1.0,
		// so a few units of absolute drift IS float-exact agreement.
		printf("strafelaw: %s\n", (max_dg <= 4.f && max_dt < 1e-5f)
			? "closed form EXACT against the certified mirror (float ULP)"
			: "LAW DEVIATES - do not build on it; find out why first");
		// The economics table for the record: per-tick gain vs turn at the
		// optimum and at biases, standing, sfric 1.
		printf("  v      gain(perp)  turn(perp)deg | gain(b=15deg) turn(b=15)\n");
		for (int vi = 0; vi < 8; ++vi) {
			const float v = kV[vi];
			Strafe::TickLaw law = Strafe::Law(o.params, v, 1.f, false);
			const float g0 = sqrtf(v * v + law.OptGain2()) - v;
			const float t0 = law.TurnRad(0.f, 1.f) * 57.29578f;
			const float cb = cosf(Deg2Rad(75.f));   // 15 deg off perp
			const float sb = sinf(Deg2Rad(75.f));
			const float gb = sqrtf(law.NewSpeed2(cb)) - v;
			const float tb = law.TurnRad(cb, sb) * 57.29578f;
			printf("  %6.0f  %9.4f  %12.4f | %12.4f  %9.4f\n",
				v, g0, t0, gb, tb);
		}
		fflush(stdout);
		return (max_dg < 0.05f && max_dt < 1e-5f) ? 0 : 2;
	}

	// routegraph: STAGE 1 of the rebuild - extract the feature graph and
	// dump it (summary + solver\route_features.csv for inspection).
	int CmdRouteGraph(const std::string& map_path, const ReplayOpts& o,
	                  const std::string& out_path) {
		World w;
		std::string err;
		w.true_interval_corner = o.corner_true;
		if (!w.Load(map_path, o.hulls, &err, o.edge_bevels)) {
			printf("LOAD FAILED (bsp): %s\n", err.c_str());
			return 1;
		}
		Route::Graph g;
		if (!Route::Build(w, &g, 2000.f, &err)) {
			printf("routegraph: %s\n", err.c_str());
			return 1;
		}
		printf("routegraph: %d surfable faces, %d candidate transfers\n",
			static_cast<int>(g.faces.size()),
			static_cast<int>(g.edges.size()));
		// Zone anchoring when an anchor tape is supplied (--anchor-tas /
		// --control) - start under its anchor; end via --end-brush, or
		// DERIVED from the tape's own finish (replay to the last tick and
		// take the brush under the finisher - zones are plugin-side on real
		// servers, so a human trace is the honest zone source; the expert
		// demo traces will serve the same role per map).
		if (!o.anchor_tas.empty()) {
			Tape at;
			int end_id = o.end_brush;
			const bool tape_ok =
				LoadTas(o.anchor_tas, at, &err) && at.start.valid;
			if (end_id < 0 && tape_ok) {
				PlayerState s;
				s.pos = at.start.origin;
				s.vel = at.start.velocity;
				s.ducked = at.start.ducked;
				s.hull_state = at.start.ducked ? 1 : 0;
				s.stamina = at.start.stamina;
				TraceResult tr;
				const float gf = w.TraceHull(s.pos,
					s.pos - Vec3(0.f, 0.f, 2.f), s.ducked, &tr);
				if (gf < 1.f && tr.brush >= 0
					&& tr.normal.Z >= o.params.walkable_z) {
					s.pos.Z -= 2.f * gf;
					s.on_ground = true;
					s.ground_brush = tr.brush;
				}
				for (size_t t = 0; t < at.frames.size(); ++t) {
					const TapeFrame& fr = at.frames[t];
					TickEvents ev;
					MoveTick(s, w, o.params, fr.pitch, fr.yaw, fr.fmove,
						fr.smove, fr.umove, fr.buttons, &ev);
				}
				end_id = w.BrushUnder(s.pos, s.ducked);
				printf("routegraph: end zone derived from tape finish: "
					"brush id %d at (%.0f,%.0f,%.0f)\n", end_id,
					s.pos.X, s.pos.Y, s.pos.Z);
			}
			if (tape_ok
				&& Route::AnchorZones(w, &g, at.start.origin,
					at.start.ducked, end_id, 2000.f, &err)) {
				printf("routegraph: start brush idx %d (under anchor), "
					"%d candidate first boards", g.start_brush,
					static_cast<int>(g.start_faces.size()));
				if (g.end_brush >= 0)
					printf("; end brush idx %d, %d feeder faces",
						g.end_brush,
						static_cast<int>(g.end_faces.size()));
				printf("\n");
			} else {
				printf("routegraph: zone anchoring FAILED: %s\n",
					err.c_str());
			}
		}
		for (size_t i = 0; i < g.faces.size() && i < 40; ++i) {
			const Route::Face& f = g.faces[i];
			printf("  face %2d  brush %4d side %d  n(%+.3f,%+.3f,%+.3f)  "
				"c(%8.1f,%8.1f,%7.1f)  area %8.0f  z[%7.1f..%7.1f]\n",
				static_cast<int>(i), w.brushes[f.brush].id, f.side,
				f.n.X, f.n.Y, f.n.Z,
				f.centroid.X, f.centroid.Y, f.centroid.Z,
				f.area, f.zmin, f.zmax);
		}
		FILE* f = nullptr;
		if (fopen_s(&f, out_path.c_str(), "w") == 0 && f) {
			fprintf(f, "face,brush,side,nx,ny,nz,cx,cy,cz,area,zmin,zmax,"
				"dhx,dhy,dhz,nverts\n");
			for (size_t i = 0; i < g.faces.size(); ++i) {
				const Route::Face& q = g.faces[i];
				fprintf(f, "%d,%d,%d,%.6f,%.6f,%.6f,%.3f,%.3f,%.3f,%.1f,"
					"%.3f,%.3f,%.6f,%.6f,%.6f,%d\n",
					static_cast<int>(i), w.brushes[q.brush].id, q.side,
					q.n.X, q.n.Y, q.n.Z,
					q.centroid.X, q.centroid.Y, q.centroid.Z, q.area,
					q.zmin, q.zmax,
					q.downhill.X, q.downhill.Y, q.downhill.Z,
					static_cast<int>(q.verts.size()));
			}
			fclose(f);
			printf("routegraph: features -> %s\n", out_path.c_str());
		}
		fflush(stdout);
		return 0;
	}

	// envelope: THE M1.1 ACCEPTANCE GATE (Docs/SolverRebuildChecklist.md).
	// Containment: replay the certified tapes; every airborne stretch
	// (surf exit -> next contact) must sit INSIDE the analytic envelope
	// (exact ballistic z, speed <= SMax, distance <= DMax).
	// Falsification: from sampled airborne states, run random control
	// sequences on the exact engine and verify NONE escapes the bounds.
	int CmdEnvelope(const std::string& map_path, const ReplayOpts& o,
	                const std::vector<std::string>& tapes) {
		World w;
		std::string err;
		w.true_interval_corner = o.corner_true;
		if (!w.Load(map_path, o.hulls, &err, o.edge_bevels)) {
			printf("LOAD FAILED (bsp): %s\n", err.c_str());
			return 1;
		}
		int stretches = 0, contained = 0, viol_z = 0, viol_s = 0, viol_d = 0;
		float worst_z = 0.f, worst_s = 0.f, worst_d = 0.f;
		for (const std::string& tp : tapes) {
			Tape tape;
			if (!LoadTas(tp, tape, &err)) {
				printf("LOAD FAILED (tas %s): %s\n", tp.c_str(), err.c_str());
				return 1;
			}
			PlayerState s;
			s.pos = tape.start.origin;
			s.vel = tape.start.velocity;
			s.ducked = tape.start.ducked;
			s.hull_state = tape.start.ducked ? 1 : 0;
			s.stamina = tape.start.stamina;
			{
				TraceResult tr;
				const float gf = w.TraceHull(s.pos,
					s.pos - Vec3(0.f, 0.f, 2.f), s.ducked, &tr);
				if (gf < 1.f && tr.brush >= 0
					&& tr.normal.Z >= o.params.walkable_z) {
					s.pos.Z -= 2.f * gf;
					s.on_ground = true;
					s.ground_brush = tr.brush;
				}
			}
			// Track airborne stretches between contacts. The envelope
			// bounds the AIR PHASE ONLY, so measurement ends at the LAST
			// AIRBORNE tick - the contact tick's clip converts vz into
			// horizontal speed (that conversion is the BOARD's physics,
			// M1.2's business, and it blew the speed bound by +57 when
			// measured post-clip).
			bool in_air = false;
			Vec3 a_pos, a_vel;
			int a_tick = 0;
			for (size_t t = 0; t < tape.frames.size(); ++t) {
				const TapeFrame& f = tape.frames[t];
				TickEvents ev;
				const bool was_air = in_air;
				const Vec3 pre_pos = s.pos, pre_vel = s.vel;
				MoveTick(s, w, o.params, f.pitch, f.yaw, f.fmove, f.smove,
					f.umove, f.buttons, &ev);
				const bool contact = ev.ncontacts > 0 || s.on_ground;
				if (!was_air && !contact) {
					in_air = true;
					a_pos = s.pos;   // first fully-airborne state
					a_vel = s.vel;
					a_tick = static_cast<int>(t);
				} else if (was_air && contact) {
					in_air = false;
					// End state = the last fully-airborne tick (pre this).
					const int n = static_cast<int>(t) - 1 - a_tick;
					if (n < 2)
						continue;
					stretches++;
					// pre_pos/pre_vel = the state entering the contact tick
					// = the state AFTER the last fully-airborne tick.
					const float zp = Envelope::ZAfter(a_pos.Z, a_vel.Z, n,
						o.params, s.gravity_scale);
					const float dzv = pre_pos.Z - zp;
					const float sm = Envelope::SMax(Len2D(a_vel), n,
						o.params);
					const float sa = Len2D(pre_vel);
					const float dm = Envelope::DMax(Len2D(a_vel), n,
						o.params);
					const float dx = pre_pos.X - a_pos.X;
					const float dy = pre_pos.Y - a_pos.Y;
					const float da = sqrtf(dx * dx + dy * dy);
					bool ok = true;
					// z band: ballistic plus the air-duck offset, which is
					// {-8.5, 0, +8.5} RELATIVE TO THE ENTRY DUCK STATE (a
					// stretch entered ducked that unducks mid-air sits at
					// -8.5 - the one violation of the first gate run).
					if (dzv < -9.5f || dzv > 9.5f) { viol_z++; ok = false;
						if (fabsf(dzv) > worst_z) worst_z = fabsf(dzv); }
					if (sa > sm + 1.f) { viol_s++; ok = false;
						if (sa - sm > worst_s) worst_s = sa - sm; }
					if (da > dm + 8.f) { viol_d++; ok = false;
						if (da - dm > worst_d) worst_d = da - dm; }
					if (ok) contained++;
				}
			}
			printf("envelope: %s replayed\n", tp.c_str());
		}
		printf("envelope: %d airborne stretches | %d contained | "
			"violations z %d s %d d %d (worst %.3f / %.3f / %.3f)\n",
			stretches, contained, viol_z, viol_s, viol_d,
			worst_z, worst_s, worst_d);

		// FALSIFICATION: random control sims must stay inside.
		unsigned rng = 77u;
		auto next = [&rng]() {
			rng = rng * 1664525u + 1013904223u;
			return (rng >> 8) & 0xFFFFu;
		};
		auto frand = [&next]() { return next() / 65535.f; };
		int fal_n = 0, fal_out = 0;
		float fal_worst = 0.f;
		for (int trial = 0; trial < 500; ++trial) {
			PlayerState s;
			s.pos = Vec3(-400.f + frand() * 800.f,
				-700.f + frand() * 800.f, 2600.f);
			const float sp = frand() * 2000.f;
			const float hd = frand() * 6.2831853f;
			s.pos.Z = 2600.f;
			s.vel = Vec3(sp * cosf(hd), sp * sinf(hd),
				-300.f + frand() * 500.f);
			s.on_ground = false;
			const Vec3 p0 = s.pos;
			const Vec3 v0 = s.vel;
			const int n = 10 + (next() % 50);
			bool escaped = false;
			for (int k = 0; k < n; ++k) {
				TickEvents ev;
				MoveTick(s, w, o.params, 0.f,
					frand() * 360.f - 180.f,
					frand() * 900.f - 450.f, frand() * 900.f - 450.f, 0.f,
					(next() % 8 == 0) ? IN_DUCK : 0, &ev);
				if (ev.ncontacts > 0 || s.on_ground) { escaped = true; break; }
				const int kk = k + 1;
				const float zp = Envelope::ZAfter(p0.Z, v0.Z, kk, o.params);
				const float sm = Envelope::SMax(Len2D(v0), kk, o.params);
				const float dm = Envelope::DMax(Len2D(v0), kk, o.params);
				const float dx = s.pos.X - p0.X, dy = s.pos.Y - p0.Y;
				const float over_s = Len2D(s.vel) - sm;
				const float over_d = sqrtf(dx * dx + dy * dy) - dm;
				// z: ballistic plus the air-duck offset band {0, +8.5}.
				const float zerr = s.pos.Z - zp;
				const float over_z = zerr > 9.f ? zerr - 9.f
					: (zerr < -0.5f ? -zerr : 0.f);
				if (over_s > 0.6f || over_d > 8.f || over_z > 0.f) {
					fal_out++;
					const float wv = over_s > over_d
						? (over_s > over_z ? over_s : over_z)
						: (over_d > over_z ? over_d : over_z);
					if (wv > fal_worst) fal_worst = wv;
					escaped = true;
					break;
				}
			}
			if (!escaped)
				fal_n++;
		}
		printf("envelope: falsification %d clean trials | %d ESCAPES "
			"(worst %.4f)\n", fal_n, fal_out, fal_worst);
		const bool pass = viol_z == 0 && viol_s == 0 && viol_d == 0
			&& fal_out == 0 && stretches > 0;
		printf("envelope: M1.1 GATE %s\n", pass ? "PASS" : "FAIL");
		fflush(stdout);
		return pass ? 0 : 2;
	}

	// boardwin: THE M1.2 ACCEPTANCE GATE (Docs/SolverRebuildChecklist.md).
	// Two halves:
	//  A. TAPE BOARDS - replay the certified tapes, and for every
	//     air->surf first clip check the recorded contact against the
	//     face's window: approaching (dot < 0), in-region (hull-slack
	//     edge distance), analytic min-loss lower bound holds, and the
	//     clip-loss MODEL is engine-exact (|v1|^2 - dot^2 vs the
	//     mirror's own measured loss). The measured |dot| distribution
	//     is REPORTED - that number is expert evidence, not a fit.
	//  B. SPOT CHECK - synthetic zero-input states sampled across every
	//     face and the aim spectrum (tangent grazes to head-on boards);
	//     the closed-form clip-tick prediction must match MoveTick and
	//     every observed dot must sit inside the analytic DotRange.
	int CmdBoardWin(const std::string& map_path, const ReplayOpts& o,
	                const std::vector<std::string>& tapes) {
		World w;
		std::string err;
		w.true_interval_corner = o.corner_true;
		if (!w.Load(map_path, o.hulls, &err, o.edge_bevels)) {
			printf("LOAD FAILED (bsp): %s\n", err.c_str());
			return 1;
		}
		Route::Graph g;
		if (!Route::Build(w, &g, 2000.f, &err)) {
			printf("boardwin: %s\n", err.c_str());
			return 1;
		}
		auto find_face = [&](int brush, int plane) {
			for (size_t i = 0; i < g.faces.size(); ++i)
				if (g.faces[i].brush == brush && g.faces[i].side == plane)
					return static_cast<int>(i);
			return -1;
		};

		// ---- A: tape boards -------------------------------------------
		int boards = 0, creases = 0, unmapped = 0;
		int bad_approach = 0, bad_region = 0, bad_minlaw = 0, bad_model = 0;
		float max_dot = 0.f, max_frac = 0.f, max_edge = -1e9f;
		float worst_model = 0.f;
		for (const std::string& tp : tapes) {
			Tape tape;
			if (!LoadTas(tp, tape, &err)) {
				printf("LOAD FAILED (tas %s): %s\n", tp.c_str(), err.c_str());
				return 1;
			}
			PlayerState s;
			s.pos = tape.start.origin;
			s.vel = tape.start.velocity;
			s.ducked = tape.start.ducked;
			s.hull_state = tape.start.ducked ? 1 : 0;
			s.stamina = tape.start.stamina;
			{
				TraceResult tr;
				const float gf = w.TraceHull(s.pos,
					s.pos - Vec3(0.f, 0.f, 2.f), s.ducked, &tr);
				if (gf < 1.f && tr.brush >= 0
					&& tr.normal.Z >= o.params.walkable_z) {
					s.pos.Z -= 2.f * gf;
					s.on_ground = true;
					s.ground_brush = tr.brush;
				}
			}
			bool was_air = false;
			for (size_t t = 0; t < tape.frames.size(); ++t) {
				const TapeFrame& f = tape.frames[t];
				TickEvents ev;
				MoveTick(s, w, o.params, f.pitch, f.yaw, f.fmove, f.smove,
					f.umove, f.buttons, &ev);
				const bool air_now = ev.ncontacts == 0 && !s.on_ground;
				// A board = the first SURF clip arriving from the air.
				if (was_air && ev.ncontacts > 0) {
					const int b = ev.contact_brush[0];
					const int pl = ev.contact_plane[0];
					if (b >= 0 && pl >= 0) {
						const Vec3& n = w.brushes[b].n[pl];
						if (n.Z > 0.f && n.Z < o.params.walkable_z) {
							const int fi = find_face(b, pl);
							if (fi < 0) {
								unmapped++;
							} else if (ev.ncontacts > 1) {
								creases++;   // multi-plane: not the
								             // window's single-clip claim
							} else {
								boards++;
								const Route::Face& fc = g.faces[fi];
								const Vec3& v1 = ev.contact_vel[0];
								const float dot = Dot(v1, fc.n);
								const float edge = Board::EdgeDistOut(fc,
									ev.contact_pos[0]);
								const float sp1 = Len(v1);
								if (dot >= 0.f) bad_approach++;
								if (edge > Board::kHullCenterSlack)
									bad_region++;
								if (edge > max_edge) max_edge = edge;
								if (-dot > max_dot) max_dot = -dot;
								const float frac = sp1 > 1.f
									? (dot * dot) / (sp1 * sp1) : 0.f;
								if (frac > max_frac) max_frac = frac;
								// Analytic min-loss lower bound.
								const float md = Board::MinApproachDot(
									Len2D(v1), v1.Z, fc.n);
								if (md > -dot + 1e-2f) bad_minlaw++;
								// Model exactness vs the mirror's own
								// measured post-clip speed.
								const float post_meas = sp1
									- ev.contact_loss[0];
								Vec3 pv;
								Fn::ClipVelocity(v1, fc.n, &pv);
								const float dmodel =
									fabsf(Len(pv) - post_meas);
								if (dmodel > worst_model)
									worst_model = dmodel;
								if (dmodel > 0.05f) bad_model++;
							}
						}
					}
				}
				was_air = air_now;
			}
			printf("boardwin: %s replayed\n", tp.c_str());
		}
		printf("boardwin: %d tape boards (%d crease, %d unmapped) | "
			"approach %d bad | region %d bad (max edge %.1fu) | "
			"min-law %d bad | model %d bad (worst %.4f u/s)\n",
			boards, creases, unmapped, bad_approach, bad_region,
			max_edge, bad_minlaw, bad_model, worst_model);
		printf("boardwin: measured expert boards: max |dot| %.1f u/s, "
			"max loss fraction %.4f\n", max_dot, max_frac);

		// ---- B: spot check across faces and the aim spectrum ----------
		unsigned rng = 1234u;
		auto next = [&rng]() {
			rng = rng * 1664525u + 1013904223u;
			return (rng >> 8) & 0xFFFFu;
		};
		auto frand = [&next]() { return next() / 65535.f; };
		int checked = 0, cone_bad = 0, pred_bad = 0;
		float worst_pred = 0.f, worst_dotp = 0.f;
		const float speeds[3] = { 300.f, 800.f, 1400.f };
		const float vzs[3] = { -400.f, -150.f, 150.f };
		const float phis_deg[8] = { -80.f, -55.f, -30.f, -8.f,
		                            8.f, 30.f, 55.f, 80.f };
		for (size_t fi = 0; fi < g.faces.size(); ++fi) {
			const Route::Face& fc = g.faces[fi];
			const float h = sqrtf(fc.n.X * fc.n.X + fc.n.Y * fc.n.Y);
			if (h < 1e-4f || fc.verts.size() < 3)
				continue;
			const float inx = -fc.n.X / h, iny = -fc.n.Y / h; // into face
			for (int trial = 0; trial < 3; ++trial) {
				// Random interior point (convex polygon).
				Vec3 q = fc.centroid;
				for (size_t vi = 0; vi < fc.verts.size(); ++vi)
					q = q + Scale(fc.verts[vi] - fc.centroid,
						frand() * 0.5f / fc.verts.size());
				for (int si = 0; si < 3; ++si)
				for (int zi = 0; zi < 3; ++zi)
				for (int pi = 0; pi < 8; ++pi) {
					const float sp = speeds[si];
					const float phi = phis_deg[pi] * 0.017453293f;
					const float c = cosf(phi), sn = sinf(phi);
					// Aim = into-face dir rotated by phi around z.
					const float ax = inx * c - iny * sn;
					const float ay = inx * sn + iny * c;
					PlayerState s;
					s.pos = q + Scale(fc.n, 25.f + frand() * 30.f);
					s.pos.Z += 20.f;
					s.vel = Vec3(sp * ax, sp * ay, vzs[zi]);
					s.on_ground = false;
					for (int k = 0; k < 80; ++k) {
						const Vec3 pre_vel = s.vel;
						TickEvents ev;
						MoveTick(s, w, o.params, 0.f, 0.f, 0.f, 0.f,
							0.f, 0, &ev);
						if (s.on_ground)
							break;
						if (ev.ncontacts == 0)
							continue;
						// First contact: only grade OUR face, single
						// plane (other geometry / creases: discard).
						if (ev.ncontacts == 1
							&& ev.contact_brush[0] == fc.brush
							&& ev.contact_plane[0] == fc.side) {
							checked++;
							const Vec3& v1 = ev.contact_vel[0];
							// v1 must equal pre_vel + StartGravity
							// (zero input), and dot must sit in the
							// analytic range for (s2d, vz).
							Vec3 v1p = pre_vel;
							v1p.Z -= s.gravity_scale * o.params.gravity
								* 0.5f * o.params.dt;
							const float ddp = fabsf(Dot(v1, fc.n)
								- Dot(v1p, fc.n));
							if (ddp > worst_dotp) worst_dotp = ddp;
							float lo, hi;
							Board::DotRange(Len2D(v1), v1.Z, fc.n,
								&lo, &hi);
							const float dot = Dot(v1, fc.n);
							if (dot < lo - 1e-3f || dot > hi + 1e-3f)
								cone_bad++;
							// Closed-form clip-tick vs the exact tick.
							const Vec3 pv = Board::PredictClipTickVel(
								pre_vel, fc.n, o.params,
								s.gravity_scale);
							const float dp = Len(pv - s.vel);
							if (dp > worst_pred) worst_pred = dp;
							if (dp > 0.05f) pred_bad++;
						}
						break;   // any contact ends the sample
					}
				}
			}
		}
		printf("boardwin: spot check %d single-plane strikes | cone %d "
			"bad | predict %d bad (worst vel %.4f u/s, worst dot "
			"%.5f)\n", checked, cone_bad, pred_bad, worst_pred,
			worst_dotp);
		const bool pass = boards > 0 && bad_approach == 0
			&& bad_region == 0 && bad_minlaw == 0 && bad_model == 0
			&& unmapped == 0 && checked >= 50 && cone_bad == 0
			&& pred_bad == 0;
		printf("boardwin: M1.2 GATE %s\n", pass ? "PASS" : "FAIL");
		fflush(stdout);
		return pass ? 0 : 2;
	}

	// airsolve: THE M1.3 ACCEPTANCE GATE (Docs/SolverRebuildChecklist.md).
	// For every tape transfer (airborne stretch ending in a single-plane
	// surf board on a mapped face), hand the UNSEEDED air primitive only
	// the entry state and the tape's landing window (face + contact
	// point + arrival tick preference) - never the tape's controls - and
	// demand it reproduce the board: same face, in-region, arrival tick
	// within +/-8, clip loss within +30 u/s of the tape's dot (better is
	// fine). Strafe-side flips are rate-limited by construction.
	int CmdAirSolve(const std::string& map_path, const ReplayOpts& o,
	                const std::vector<std::string>& tapes) {
		World w;
		std::string err;
		w.true_interval_corner = o.corner_true;
		if (!w.Load(map_path, o.hulls, &err, o.edge_bevels)) {
			printf("LOAD FAILED (bsp): %s\n", err.c_str());
			return 1;
		}
		Route::Graph g;
		if (!Route::Build(w, &g, 2000.f, &err)) {
			printf("airsolve: %s\n", err.c_str());
			return 1;
		}
		auto find_face = [&](int brush, int plane) {
			for (size_t i = 0; i < g.faces.size(); ++i)
				if (g.faces[i].brush == brush && g.faces[i].side == plane)
					return static_cast<int>(i);
			return -1;
		};

		struct Xfer {
			PlayerState entry;
			int n = 0, face = -1;
			float dot = 0.f, s2d = 0.f;
			Vec3 contact;
		};
		std::vector<Xfer> xfers;
		int skipped = 0;
		for (const std::string& tp : tapes) {
			Tape tape;
			if (!LoadTas(tp, tape, &err)) {
				printf("LOAD FAILED (tas %s): %s\n", tp.c_str(), err.c_str());
				return 1;
			}
			PlayerState s;
			s.pos = tape.start.origin;
			s.vel = tape.start.velocity;
			s.ducked = tape.start.ducked;
			s.hull_state = tape.start.ducked ? 1 : 0;
			s.stamina = tape.start.stamina;
			{
				TraceResult tr;
				const float gf = w.TraceHull(s.pos,
					s.pos - Vec3(0.f, 0.f, 2.f), s.ducked, &tr);
				if (gf < 1.f && tr.brush >= 0
					&& tr.normal.Z >= o.params.walkable_z) {
					s.pos.Z -= 2.f * gf;
					s.on_ground = true;
					s.ground_brush = tr.brush;
				}
			}
			bool was_air = false;
			PlayerState entry;
			int a_tick = 0;
			for (size_t t = 0; t < tape.frames.size(); ++t) {
				const TapeFrame& f = tape.frames[t];
				TickEvents ev;
				MoveTick(s, w, o.params, f.pitch, f.yaw, f.fmove, f.smove,
					f.umove, f.buttons, &ev);
				const bool air_now = ev.ncontacts == 0 && !s.on_ground;
				if (!was_air && air_now) {
					entry = s;
					a_tick = static_cast<int>(t);
				}
				if (was_air && ev.ncontacts > 0) {
					const int b = ev.contact_brush[0];
					const int pl = ev.contact_plane[0];
					if (b >= 0 && pl >= 0) {
						const Vec3& n = w.brushes[b].n[pl];
						const int fi = (n.Z > 0.f
							&& n.Z < o.params.walkable_z)
							? find_face(b, pl) : -1;
						const int nt = static_cast<int>(t) - a_tick;
						if (fi >= 0 && ev.ncontacts == 1 && nt >= 2) {
							Xfer x;
							x.entry = entry;
							x.n = nt;
							x.face = fi;
							x.dot = Dot(ev.contact_vel[0],
								g.faces[fi].n);
							x.s2d = Len2D(ev.contact_vel[0]);
							x.contact = ev.contact_pos[0];
							xfers.push_back(x);
						} else if (fi >= 0) {
							skipped++;
						}
					}
				}
				was_air = air_now;
			}
			printf("airsolve: %s replayed\n", tp.c_str());
		}

		int ok = 0;
		for (size_t i = 0; i < xfers.size(); ++i) {
			const Xfer& x = xfers[i];
			Air::Target tg;
			tg.face = x.face;
			tg.aim = x.contact;
			tg.dot_cap = (-x.dot > 50.f ? -x.dot : 50.f) + 30.f;
			tg.max_ticks = x.n + 40;
			tg.aim_tick = x.n;
			tg.tick_w = 0.2f;
			tg.tick_tol = 8;
			const Air::Result r = Air::SolveTransfer(x.entry, w,
				o.params, g, tg, 4, 3000);
			const bool good = r.hit
				&& r.edge <= Board::kHullCenterSlack
				&& -r.dot <= -x.dot + 30.f
				&& (r.tick - x.n <= 8 && x.n - r.tick <= 8);
			if (good) ok++;
			if (r.hit)
				printf("airsolve[%2d]: face %2d | tape n=%3d dot=%7.1f "
					"s2d=%6.1f | solver n=%3d dot=%7.1f s2d=%6.1f "
					"edge=%5.1f flips=%d | %s\n",
					static_cast<int>(i), x.face, x.n, x.dot, x.s2d,
					r.tick, r.dot, r.speed2d, r.edge, r.flips,
					good ? "OK" : "OFF-WINDOW");
			else
				printf("airsolve[%2d]: face %2d | tape n=%3d dot=%7.1f "
					"s2d=%6.1f | solver MISS (closest %.1fu, %s%d) "
					"entry(%.0f,%.0f,%.0f v %.0f,%.0f,%.0f) end"
					"(%.0f,%.0f,%.0f) aim(%.0f,%.0f,%.0f)\n",
					static_cast<int>(i), x.face, x.n, x.dot, x.s2d,
					r.miss_dist,
					r.grounded ? "grounded, brush " : "struck brush ",
					r.struck_brush >= 0
						? w.brushes[r.struck_brush].id : -1,
					x.entry.pos.X, x.entry.pos.Y, x.entry.pos.Z,
					x.entry.vel.X, x.entry.vel.Y, x.entry.vel.Z,
					r.end_pos.X, r.end_pos.Y, r.end_pos.Z,
					x.contact.X, x.contact.Y, x.contact.Z);
		}
		printf("airsolve: %d/%d transfers reproduced unseeded (%d "
			"skipped: crease/tiny)\n", ok,
			static_cast<int>(xfers.size()), skipped);
		const bool pass = !xfers.empty()
			&& ok == static_cast<int>(xfers.size());
		printf("airsolve: M1.3 GATE %s\n", pass ? "PASS" : "FAIL");
		fflush(stdout);
		return pass ? 0 : 2;
	}

	// carve: THE M1.4 ACCEPTANCE GATE (Docs/SolverRebuildChecklist.md).
	// Two halves:
	//  A. TAPE CARVES - every tape ride (>= 6 contact ticks on one
	//     mapped face, ending in clean air) is re-solved UNSEEDED: the
	//     primitive gets the entry state + the tape's exit heading/
	//     tick/position, never the controls. Reproduce = exit within
	//     0.20 rad, +/-8 ticks, 80u, speed within 30 u/s.
	//  B. MANIFOLD - sampled on-face states across faces x directions x
	//     speeds, zero-input: the carve ENERGY IDENTITY
	//       v2_end + sum(dot^2) = v2_0 + 2g*(z0 - z_end)
	//     must hold on the exact engine (clip work = -dot^2, gravity
	//     work = 2g*drop, nothing else moves energy); full-strafe rides
	//     must respect the law bound (wish work <= 900/tick on top).
	int CmdCarve(const std::string& map_path, const ReplayOpts& o,
	             const std::vector<std::string>& tapes) {
		World w;
		std::string err;
		w.true_interval_corner = o.corner_true;
		if (!w.Load(map_path, o.hulls, &err, o.edge_bevels)) {
			printf("LOAD FAILED (bsp): %s\n", err.c_str());
			return 1;
		}
		Route::Graph g;
		if (!Route::Build(w, &g, 2000.f, &err)) {
			printf("carve: %s\n", err.c_str());
			return 1;
		}
		auto find_face = [&](int brush, int plane) {
			for (size_t i = 0; i < g.faces.size(); ++i)
				if (g.faces[i].brush == brush && g.faces[i].side == plane)
					return static_cast<int>(i);
			return -1;
		};

		// ---- A: tape carves -------------------------------------------
		struct Ride {
			PlayerState entry;
			int face = -1, n = 0;
			float exit_heading = 0.f, exit_s2d = 0.f;
			Vec3 exit_pos;
			Vec3 exit_vel;
			int t0 = 0, tape_idx = 0;
		};
		std::vector<Ride> rides;
		for (const std::string& tp : tapes) {
			Tape tape;
			if (!LoadTas(tp, tape, &err)) {
				printf("LOAD FAILED (tas %s): %s\n", tp.c_str(), err.c_str());
				return 1;
			}
			PlayerState s;
			s.pos = tape.start.origin;
			s.vel = tape.start.velocity;
			s.ducked = tape.start.ducked;
			s.hull_state = tape.start.ducked ? 1 : 0;
			s.stamina = tape.start.stamina;
			{
				TraceResult tr;
				const float gf = w.TraceHull(s.pos,
					s.pos - Vec3(0.f, 0.f, 2.f), s.ducked, &tr);
				if (gf < 1.f && tr.brush >= 0
					&& tr.normal.Z >= o.params.walkable_z) {
					s.pos.Z -= 2.f * gf;
					s.on_ground = true;
					s.ground_brush = tr.brush;
				}
			}
			int ride_face = -1, contact_ticks = 0, entry_tick = 0;
			int air_streak = 0, exit_tick = 0;
			PlayerState entry_s, exit_s;
			bool have_exit = false;
			auto close_ride = [&](bool clean) {
				if (clean && ride_face >= 0 && contact_ticks >= 6
					&& have_exit) {
					Ride r;
					r.entry = entry_s;
					r.face = ride_face;
					r.n = exit_tick - entry_tick;
					r.exit_heading = atan2f(exit_s.vel.Y,
						exit_s.vel.X);
					r.exit_s2d = Len2D(exit_s.vel);
					r.exit_pos = exit_s.pos;
					r.exit_vel = exit_s.vel;
					r.t0 = entry_tick;
					if (r.n >= 2)
						rides.push_back(r);
				}
				ride_face = -1;
				contact_ticks = 0;
				air_streak = 0;
				have_exit = false;
			};
			for (size_t t = 0; t < tape.frames.size(); ++t) {
				const TapeFrame& f = tape.frames[t];
				TickEvents ev;
				MoveTick(s, w, o.params, f.pitch, f.yaw, f.fmove,
					f.smove, f.umove, f.buttons, &ev);
				if (s.on_ground) {
					close_ride(false);
					continue;
				}
				if (ev.ncontacts > 0) {
					int fi = -1;
					bool uniform = true;
					for (int c = 0; c < ev.ncontacts; ++c) {
						const int b = ev.contact_brush[c];
						const int pl = ev.contact_plane[c];
						if (b < 0 || pl < 0) { uniform = false; break; }
						const Vec3& n = w.brushes[b].n[pl];
						if (n.Z <= 0.f || n.Z >= o.params.walkable_z) {
							uniform = false;
							break;
						}
						const int fc = find_face(b, pl);
						if (fc < 0 || (fi >= 0 && fc != fi)) {
							uniform = false;
							break;
						}
						fi = fc;
					}
					if (!uniform) {
						close_ride(false);
					} else if (fi == ride_face) {
						contact_ticks++;
						air_streak = 0;
						have_exit = false;
					} else {
						close_ride(false);
						ride_face = fi;
						contact_ticks = 1;
						entry_s = s;   // post-board state
						entry_tick = static_cast<int>(t);
					}
				} else if (ride_face >= 0) {
					if (!have_exit) {
						exit_s = s;
						exit_tick = static_cast<int>(t);
						have_exit = true;
					}
					air_streak++;
					if (air_streak >= 3)
						close_ride(true);
				}
			}
			close_ride(false);
			printf("carve: %s replayed\n", tp.c_str());
		}

		int ok = 0;
		for (size_t i = 0; i < rides.size(); ++i) {
			const Ride& x = rides[i];
			Carve::Target tg;
			tg.face = x.face;
			tg.exit_heading = x.exit_heading;
			tg.max_ticks = x.n + 40;
			tg.aim_tick = x.n;
			tg.tick_w = 0.2f;
			tg.tick_tol = 8;
			tg.aim_pos = x.exit_pos;
			tg.pos_w = 0.2f;
			tg.aim_vel = x.exit_vel;
			tg.vel_w = 1.f;
			// Long rides get more knots: an 84-tick crest carve needs
			// a finer speed/heading profile than a 20-tick sweep.
			const int kn = x.n >= 60 ? 6 : 4;
			const Carve::Result r = Carve::SolveCarve(x.entry, w,
				o.params, g, tg, kn, 20000);
			const float dh = r.exited ? fabsf(Steer::WrapPi(
				r.exit_heading - x.exit_heading)) : 9.f;
			const float dp = r.exited ? Len(r.exit_pos - x.exit_pos)
				: 1e9f;
			const float dv = r.exited ? Len(r.exit_vel - x.exit_vel)
				: 1e9f;
			const bool good = r.exited && dv <= 60.f
				&& (r.tick - x.n <= 8 && x.n - r.tick <= 8)
				&& dp <= 80.f;
			if (good) ok++;
			if (r.exited)
				printf("carve[%2d]: face %2d | tape n=%3d h=%6.1f "
					"s2d=%6.1f | solver n=%3d h=%6.1f s2d=%6.1f "
					"dv=%5.1f dpos=%5.1f flips=%d | %s\n",
					static_cast<int>(i), x.face, x.n,
					x.exit_heading * 57.29578f, x.exit_s2d, r.tick,
					r.exit_heading * 57.29578f, r.speed2d, dv, dp,
					r.flips, good ? "OK" : "OFF");
			else
				printf("carve[%2d]: face %2d | tape n=%3d h=%6.1f "
					"s2d=%6.1f | solver NO EXIT (%s%d)\n",
					static_cast<int>(i), x.face, x.n,
					x.exit_heading * 57.29578f, x.exit_s2d,
					r.grounded ? "grounded, brush "
						: "struck brush ",
					r.struck_brush >= 0
						? w.brushes[r.struck_brush].id : -1);
			if (!good)
				printf("           entry(%.0f,%.0f,%.0f) v(%.0f,%.0f,"
					"%.0f s2d %.0f) -> tape exit(%.0f,%.0f,%.0f) "
					"vz %.0f dz %.0f | tape frames %d..%d\n",
					x.entry.pos.X, x.entry.pos.Y, x.entry.pos.Z,
					x.entry.vel.X, x.entry.vel.Y, x.entry.vel.Z,
					Len2D(x.entry.vel), x.exit_pos.X, x.exit_pos.Y,
					x.exit_pos.Z, x.exit_vel.Z,
					x.exit_pos.Z - x.entry.pos.Z, x.t0, x.t0 + x.n);
		}
		printf("carve: %d/%d tape carves reproduced unseeded\n", ok,
			static_cast<int>(rides.size()));

		// ---- B: manifold (energy identity + law bound) ----------------
		unsigned rng = 4242u;
		auto next = [&rng]() {
			rng = rng * 1664525u + 1013904223u;
			return (rng >> 8) & 0xFFFFu;
		};
		auto frand = [&next]() { return next() / 65535.f; };
		int id_samples = 0, id_fail = 0, skipped = 0;
		int law_samples = 0, law_fail = 0;
		float worst_rel = 0.f, worst_law = -1e9f;
		const float speeds[2] = { 300.f, 800.f };
		const float betas[5] = { -60.f, -30.f, 0.f, 30.f, 60.f };
		for (size_t fi = 0; fi < g.faces.size(); ++fi) {
			const Route::Face& fc = g.faces[fi];
			if (fc.verts.size() < 3)
				continue;
			const float support = 16.f * fabsf(fc.n.X)
				+ 16.f * fabsf(fc.n.Y) + 36.f * fabsf(fc.n.Z);
			const float dh_az = atan2f(fc.downhill.Y, fc.downhill.X);
			for (int pt = 0; pt < 2; ++pt) {
				Vec3 q = fc.centroid;
				for (size_t vi = 0; vi < fc.verts.size(); ++vi)
					q = q + Scale(fc.verts[vi] - fc.centroid,
						frand() * 0.4f / fc.verts.size());
				for (int si = 0; si < 2; ++si)
				for (int bi = 0; bi < 5; ++bi)
				for (int wish = 0; wish < 2; ++wish) {
					const float az = dh_az
						+ betas[bi] * 0.017453293f;
					Vec3 d(cosf(az), sinf(az), 0.f);
					Vec3 ip = d - Scale(fc.n, Dot(d, fc.n));
					const float il = Len(ip);
					if (il < 0.2f)
						continue;
					ip = Scale(ip, 1.f / il);
					PlayerState s;
					s.pos = q + Scale(fc.n, support + 0.5f);
					s.vel = Scale(ip, speeds[si]);
					s.on_ground = false;
					const float v20 = Len2(s.vel);
					const float z0 = s.pos.Z;
					float sum_dot2 = 0.f, cross_bound = 0.f;
					bool bad = false;
					int ticks = 0;
					Steer::Controller ctl;
					const float hold_az = atan2f(s.vel.Y, s.vel.X);
					for (int k = 0; k < 40; ++k) {
						float yaw_deg = 0.f, fmove = 0.f, smove = 0.f;
						if (wish)
							ctl.Tick(s, o.params, hold_az, k,
								&yaw_deg, &fmove, &smove);
						TickEvents ev;
						MoveTick(s, w, o.params, 0.f, yaw_deg, fmove,
							smove, 0.f, 0, &ev);
						ticks++;
						if (s.on_ground) { bad = true; break; }
						if (ev.ncontacts > 1) { bad = true; break; }
						if (ev.ncontacts == 1) {
							const int b = ev.contact_brush[0];
							const int pl = ev.contact_plane[0];
							if (b < 0 || pl < 0) { bad = true; break; }
							const Vec3& n = w.brushes[b].n[pl];
							const float dt2 = Dot(ev.contact_vel[0], n);
							sum_dot2 += dt2 * dt2;
							// The DERIVED discrete cross term: a clip
							// at fraction f inside a half-gravity tick
							// shifts E by g*dt*(2f-1)*nz*dot, so the
							// identity holds within
							// sum(g*dt*nz*|dot|) - law, not a fit.
							cross_bound += o.params.gravity
								* o.params.dt * fabsf(n.Z)
								* fabsf(dt2);
							if (Len2(s.vel) < 1.f) { bad = true; break; }
						}
					}
					if (bad) {
						skipped++;
						continue;
					}
					const float v2e = Len2(s.vel);
					const float drop = 2.f * o.params.gravity
						* (z0 - s.pos.Z);
					if (!wish) {
						id_samples++;
						const float lhs = v2e + sum_dot2;
						const float rhs = v20 + drop;
						const float err = fabsf(lhs - rhs);
						const float lim = cross_bound + 60.f;
						const float rel = err
							/ (v20 > v2e ? v20 : v2e);
						if (rel > worst_rel) worst_rel = rel;
						if (err > lim)
							id_fail++;
					} else {
						law_samples++;
						const float wish_work = v2e + sum_dot2
							- v20 - drop;
						const float bound = 900.f
							* static_cast<float>(ticks)
							+ cross_bound + 60.f;
						if (wish_work - bound > worst_law)
							worst_law = wish_work - bound;
						if (wish_work > bound)
							law_fail++;
					}
				}
			}
		}
		printf("carve: manifold zero-input %d samples %d FAIL (worst "
			"rel %.5f) | strafing %d samples %d over law bound (worst "
			"margin %.0f) | %d skipped\n", id_samples, id_fail,
			worst_rel, law_samples, law_fail, worst_law, skipped);
		const bool pass = !rides.empty()
			&& ok == static_cast<int>(rides.size())
			&& id_samples >= 30 && id_fail == 0 && law_fail == 0;
		printf("carve: M1.4 GATE %s\n", pass ? "PASS" : "FAIL");
		fflush(stdout);
		return pass ? 0 : 2;
	}

	// ledger: THE M1.5 READOUT - phase table + biggest boards for a tape.
	int CmdLedger(const std::string& map_path, const ReplayOpts& o,
	              const std::string& tape_path, int sab0, int sab1) {
		World w;
		std::string err;
		w.true_interval_corner = o.corner_true;
		if (!w.Load(map_path, o.hulls, &err, o.edge_bevels)) {
			printf("LOAD FAILED (bsp): %s\n", err.c_str());
			return 1;
		}
		Route::Graph g;
		if (!Route::Build(w, &g, 2000.f, &err)) {
			printf("ledger: %s\n", err.c_str());
			return 1;
		}
		Tape tape;
		if (!LoadTas(tape_path, tape, &err)) {
			printf("LOAD FAILED (tas): %s\n", err.c_str());
			return 1;
		}
		Ledger::Run run;
		Ledger::Build(w, g, o.params, tape, &run, sab0, sab1);
		Ledger::Print(run, 5);
		return 0;
	}

	// ledgergate: THE M1.5 ACCEPTANCE GATE.
	//  A. SELF-AUDIT: every ride phase's energy closure (wish work
	//     from the identity) must be plausible - wish work within
	//     [-(braking budget), +900/tick] with the derived cross bound.
	//     The phase table + largest-board arithmetic is printed for
	//     hand analysis (loss2 = dot^2, regret vs tangency minimum).
	//  B. PLANTED LOSS: re-run with a sabotage window (controls zeroed
	//     mid-air-phase). The ledger must LOCALIZE it: phases before
	//     the window identical, the sabotaged phase's shortfall grows
	//     by ~the stolen gain.
	int CmdLedgerGate(const std::string& map_path, const ReplayOpts& o,
	                  const std::string& tape_path) {
		World w;
		std::string err;
		w.true_interval_corner = o.corner_true;
		if (!w.Load(map_path, o.hulls, &err, o.edge_bevels)) {
			printf("LOAD FAILED (bsp): %s\n", err.c_str());
			return 1;
		}
		Route::Graph g;
		if (!Route::Build(w, &g, 2000.f, &err)) {
			printf("ledgergate: %s\n", err.c_str());
			return 1;
		}
		Tape tape;
		if (!LoadTas(tape_path, tape, &err)) {
			printf("LOAD FAILED (tas): %s\n", err.c_str());
			return 1;
		}
		Ledger::Run base;
		Ledger::Build(w, g, o.params, tape, &base);
		Ledger::Print(base, 3);

		// A: closure plausibility on every ride phase. Bounds are the
		// LAW (wish work <= 900/tick; braking drain <= per-tick
		// 2*v*budget + budget^2) plus the derived cross bound plus a
		// float-accumulation allowance (~ULP(v^2) per arithmetic op
		// per tick - at v^2 ~ 7e5 over 45 ticks that is ~130, and the
		// first gate run failed by exactly 66 on a flat 60 slack).
		int bad_closure = 0;
		for (const Ledger::Phase& ph : base.phases) {
			if (ph.kind != Ledger::Kind::Ride)
				continue;
			const float ticks = static_cast<float>(ph.Ticks());
			const float vmax2 = ph.v2_in > ph.v2_out
				? ph.v2_in : ph.v2_out;
			const float fslack = 60.f + vmax2 * ticks * 6e-6f;
			const float budget = 562.5f;
			const float brake = (2.f * sqrtf(vmax2) * budget
				+ budget * budget) * ticks;
			const float lo = -brake - ph.cross_bound - fslack;
			const float hi = 900.f * ticks + ph.cross_bound + fslack;
			if (ph.wish_work < lo || ph.wish_work > hi)
				bad_closure++;
		}
		printf("ledgergate: closure %d bad of %d phases\n", bad_closure,
			static_cast<int>(base.phases.size()));

		// B: plant a loss in the longest air phase.
		int longest = -1, lt = 0;
		for (size_t i = 0; i < base.phases.size(); ++i)
			if (base.phases[i].kind == Ledger::Kind::Air
				&& base.phases[i].Ticks() > lt) {
				lt = base.phases[i].Ticks();
				longest = static_cast<int>(i);
			}
		if (longest < 0 || lt < 20) {
			printf("ledgergate: no long air phase to sabotage\n");
			return 2;
		}
		const Ledger::Phase& ap = base.phases[longest];
		const int s0 = ap.t0 + ap.Ticks() / 2 - 7;
		const int s1 = s0 + 15;
		Ledger::Run sab;
		Ledger::Build(w, g, o.params, tape, &sab, s0, s1);
		// Locate the sabotaged air phase in the new run (same t0 -
		// prior phases must be bit-identical).
		int hit = -1;
		int pre_same = 0;
		for (size_t i = 0; i < sab.phases.size()
			&& i < base.phases.size(); ++i) {
			if (static_cast<int>(i) < longest) {
				const Ledger::Phase& a = base.phases[i];
				const Ledger::Phase& b = sab.phases[i];
				if (a.t0 == b.t0 && a.t1 == b.t1
					&& a.v2_out == b.v2_out)
					pre_same++;
			}
			if (sab.phases[i].kind == Ledger::Kind::Air
				&& sab.phases[i].t0 == ap.t0)
				hit = static_cast<int>(i);
		}
		float dshort = 0.f;
		if (hit >= 0)
			dshort = sab.phases[hit].shortfall - ap.shortfall;
		// Localization claim: every phase BEFORE the window is
		// bit-identical (the damage starts exactly where planted) and
		// the sabotaged phase's shortfall grew by at least most of the
		// window's direct theft. No upper cap: shortfall legitimately
		// exceeds the gain ceiling when the misaligned tape yaws act
		// as brakes after the window (observed: +40.9k of braking on
		// top of the 13.5k theft) - the ledger PRICES that too.
		printf("ledgergate: sabotage t%d..t%d in air[%d] | %d/%d prior "
			"phases identical | shortfall %0.f -> %0.f (delta %.0f, "
			"window theft %d)\n", s0, s1, longest, pre_same, longest,
			ap.shortfall,
			hit >= 0 ? sab.phases[hit].shortfall : -1.f, dshort,
			15 * 900);
		const bool pass = bad_closure == 0 && pre_same == longest
			&& hit >= 0 && dshort > 8000.f;
		printf("ledgergate: M1.5 GATE %s\n", pass ? "PASS" : "FAIL");
		fflush(stdout);
		return pass ? 0 : 2;
	}

	// ledger-trace: the EXPERT energy audit - how much does the human
	// line dissipate? Reads a demo run trace (sim,tick,x,y,z,...,hspeed
	// at ~4-tick cadence), derives vz from z differences, and audits
	// each snapshot pair against gravity + the strafe-law wish-work
	// allowance. Total dissipation is the expert-evidence number the
	// solver's lines are held against.
	int CmdLedgerTrace(const std::string& csv_path, const ReplayOpts& o) {
		FILE* f = fopen(csv_path.c_str(), "r");
		if (!f) {
			printf("LOAD FAILED: %s\n", csv_path.c_str());
			return 1;
		}
		char line[512];
		std::vector<double> sims;
		std::vector<int> ticks;
		std::vector<float> xs, ys, zs, hs;
		bool header = true;
		while (fgets(line, sizeof(line), f)) {
			if (header) { header = false; continue; }
			double sim;
			long tick;
			float x, y, z, pitch, yaw, hsp;
			if (sscanf(line, "%lf,%ld,%f,%f,%f,%f,%f,%f", &sim, &tick,
				&x, &y, &z, &pitch, &yaw, &hsp) == 8) {
				sims.push_back(sim);
				ticks.push_back(static_cast<int>(tick));
				xs.push_back(x);
				ys.push_back(y);
				zs.push_back(z);
				hs.push_back(hsp);
			}
		}
		fclose(f);
		const size_t n = sims.size();
		if (n < 10) {
			printf("ledger-trace: too few rows (%d)\n",
				static_cast<int>(n));
			return 1;
		}
		// vz per interval from z differencing with the half-gravity
		// midpoint correction: z1 - z0 = vz0*dt - g*dt^2/2 (ballistic)
		// => vz0 = dz/dt + g*dt/2. On contacts this is approximate -
		// the audit is an ESTIMATE with stated noise, not engine truth.
		const float gr = o.params.gravity;
		double total_loss = 0.0, total_wish_cap = 0.0;
		float worst = 0.f;
		int worst_at = 0, loss_events = 0, pairs = 0;
		for (size_t i = 0; i + 2 < n; ++i) {
			const float dt1 = static_cast<float>(sims[i + 1] - sims[i]);
			const float dt2 = static_cast<float>(sims[i + 2]
				- sims[i + 1]);
			if (dt1 <= 0.f || dt1 > 0.2f || dt2 <= 0.f || dt2 > 0.2f)
				continue;
			const float vz0 = (zs[i + 1] - zs[i]) / dt1
				+ 0.5f * gr * dt1;
			const float vz1 = (zs[i + 2] - zs[i + 1]) / dt2
				+ 0.5f * gr * dt2;
			const float v20 = hs[i + 1] * hs[i + 1] + vz0 * vz0;
			const float v21 = hs[i + 2] * hs[i + 2] + vz1 * vz1;
			const float dticks = static_cast<float>(ticks[i + 2]
				- ticks[i + 1]);
			if (dticks <= 0.f || dticks > 12.f)
				continue;
			pairs++;
			const float wish_cap = 900.f * dticks;
			total_wish_cap += wish_cap;
			const float grav = 2.f * gr * (zs[i + 1] - zs[i + 2]);
			// Loss = energy the pair SHED beyond gravity, after
			// granting the full wish allowance.
			const float loss = (v20 + grav + wish_cap) - v21;
			if (loss > 0.f)
				total_loss += loss;
			// Noise floor: vz estimate error ~ g*dt -> v2 error
			// ~ 2*vz*g*dt; call an EVENT only far above it.
			const float noise = 2.f * (fabsf(vz1) + 100.f) * gr
				* dt2 * 0.25f + 3000.f;
			if (loss > wish_cap + noise) {
				loss_events++;
				if (loss - wish_cap > worst) {
					worst = loss - wish_cap;
					worst_at = ticks[i + 1];
				}
			}
		}
		printf("ledger-trace: %s\n", csv_path.c_str());
		printf("ledger-trace: %d pairs | dissipation-beyond-allowance "
			"events %d (worst %.0f u2/s2 ~ dot %.0f u/s at t%d) | "
			"gross shed %.0fk vs wish allowance %.0fk\n", pairs,
			loss_events, worst, sqrtf(worst > 0.f ? worst : 0.f),
			worst_at, total_loss / 1000.0, total_wish_cap / 1000.0);
		fflush(stdout);
		return 0;
	}

	// ridedump: diagnostic - print a tape ride's per-tick trajectory and
	// CONTROLS (what maneuver does the primitive have to express?).
	int CmdRideDump(const std::string& map_path, const ReplayOpts& o,
	                const std::string& tape_path, int want_start,
	                int want_end) {
		World w;
		std::string err;
		w.true_interval_corner = o.corner_true;
		if (!w.Load(map_path, o.hulls, &err, o.edge_bevels)) {
			printf("LOAD FAILED (bsp): %s\n", err.c_str());
			return 1;
		}
		Tape tape;
		if (!LoadTas(tape_path, tape, &err)) {
			printf("LOAD FAILED (tas): %s\n", err.c_str());
			return 1;
		}
		PlayerState s;
		s.pos = tape.start.origin;
		s.vel = tape.start.velocity;
		s.ducked = tape.start.ducked;
		s.hull_state = tape.start.ducked ? 1 : 0;
		s.stamina = tape.start.stamina;
		{
			TraceResult tr;
			const float gf = w.TraceHull(s.pos,
				s.pos - Vec3(0.f, 0.f, 2.f), s.ducked, &tr);
			if (gf < 1.f && tr.brush >= 0
				&& tr.normal.Z >= o.params.walkable_z) {
				s.pos.Z -= 2.f * gf;
				s.on_ground = true;
				s.ground_brush = tr.brush;
			}
		}
		for (size_t t = 0; t < tape.frames.size(); ++t) {
			const TapeFrame& f = tape.frames[t];
			TickEvents ev;
			MoveTick(s, w, o.params, f.pitch, f.yaw, f.fmove, f.smove,
				f.umove, f.buttons, &ev);
			if (static_cast<int>(t) >= want_start
				&& static_cast<int>(t) <= want_end) {
				float dot = 0.f;
				int cb = -1;
				if (ev.ncontacts > 0) {
					cb = w.brushes[ev.contact_brush[0]].id;
					const Vec3& n =
						w.brushes[ev.contact_brush[0]]
							.n[ev.contact_plane[0]];
					dot = Dot(ev.contact_vel[0], n);
				}
				printf("t%5d pos(%7.1f,%7.1f,%6.1f) v(%6.1f,%6.1f,"
					"%6.1f) s2d %5.1f h %6.1f | yaw %6.1f f %4.0f "
					"s %4.0f b%d d%d | c%d dot %6.1f%s\n",
					static_cast<int>(t), s.pos.X, s.pos.Y, s.pos.Z,
					s.vel.X, s.vel.Y, s.vel.Z, Len2D(s.vel),
					atan2f(s.vel.Y, s.vel.X) * 57.29578f, f.yaw,
					f.fmove, f.smove, f.buttons, s.ducked ? 1 : 0,
					cb, dot, s.on_ground ? " GROUND" : "");
			}
		}
		fflush(stdout);
		return 0;
	}

	// facecover: THE M0.4 ACCEPTANCE GATE (Docs/SolverRebuildChecklist.md).
	// Replay certified tapes through the exact sim, collect every SURF
	// contact (contacted plane normal.z < walkable), and verify each
	// (brush, plane) pair maps to an extracted Route::Face. Missing pairs
	// are printed WITH THE REASON the extractor dropped them - the fix is
	// then a measured change, not a guess.
	int CmdFaceCover(const std::string& map_path, const ReplayOpts& o,
	                 const std::vector<std::string>& tapes) {
		World w;
		std::string err;
		w.true_interval_corner = o.corner_true;
		if (!w.Load(map_path, o.hulls, &err, o.edge_bevels)) {
			printf("LOAD FAILED (bsp): %s\n", err.c_str());
			return 1;
		}
		Route::Graph g;
		if (!Route::Build(w, &g, 2000.f, &err)) {
			printf("facecover: %s\n", err.c_str());
			return 1;
		}
		// (brush, plane) -> face index
		auto find_face = [&](int brush, int plane) {
			for (size_t i = 0; i < g.faces.size(); ++i)
				if (g.faces[i].brush == brush && g.faces[i].side == plane)
					return static_cast<int>(i);
			return -1;
		};

		struct Miss { int brush, plane, hits; float nz; Vec3 at; };
		std::vector<Miss> misses;
		int contacts = 0, covered = 0;
		for (const std::string& tp : tapes) {
			Tape tape;
			if (!LoadTas(tp, tape, &err)) {
				printf("LOAD FAILED (tas %s): %s\n", tp.c_str(), err.c_str());
				return 1;
			}
			PlayerState s;
			s.pos = tape.start.origin;
			s.vel = tape.start.velocity;
			s.ducked = tape.start.ducked;
			s.hull_state = tape.start.ducked ? 1 : 0;
			s.stamina = tape.start.stamina;
			{
				TraceResult tr;
				const float gf = w.TraceHull(s.pos,
					s.pos - Vec3(0.f, 0.f, 2.f), s.ducked, &tr);
				if (gf < 1.f && tr.brush >= 0
					&& tr.normal.Z >= o.params.walkable_z) {
					s.pos.Z -= 2.f * gf;
					s.on_ground = true;
					s.ground_brush = tr.brush;
				}
			}
			for (size_t t = 0; t < tape.frames.size(); ++t) {
				const TapeFrame& f = tape.frames[t];
				TickEvents ev;
				MoveTick(s, w, o.params, f.pitch, f.yaw, f.fmove, f.smove,
					f.umove, f.buttons, &ev);
				for (int c = 0; c < ev.ncontacts; ++c) {
					const int b = ev.contact_brush[c];
					const int p = ev.contact_plane[c];
					if (b < 0 || p < 0)
						continue;
					const Vec3& n = w.brushes[b].n[p];
					if (n.Z >= o.params.walkable_z || n.Z <= 0.f)
						continue;   // ground/ceiling contact, not surf
					contacts++;
					if (find_face(b, p) >= 0) {
						covered++;
						continue;
					}
					bool seen = false;
					for (Miss& m : misses)
						if (m.brush == b && m.plane == p) {
							m.hits++;
							seen = true;
							break;
						}
					if (!seen)
						misses.push_back({ b, p, 1, n.Z, s.pos });
				}
			}
			printf("facecover: %s replayed (%d frames)\n", tp.c_str(),
				static_cast<int>(tape.frames.size()));
		}
		printf("facecover: %d surf contacts | %d covered | %d distinct "
			"MISSING faces\n", contacts, covered,
			static_cast<int>(misses.size()));
		for (const Miss& m : misses) {
			const WorldBrush& b = w.brushes[m.brush];
			const char* why = "unknown - extractor bug";
			if (m.plane < static_cast<int>(b.pid.size())
				&& b.pid[m.plane] < 0)
				why = "side is a compiler BEVEL (pid < 0)";
			else if (m.nz <= 0.05f)
				why = "nz <= 0.05 window floor";
			else if (m.nz >= 0.7f)
				why = "nz >= 0.7 (walkable window)";
			else
				why = "passed filters - polygon clip or area dropped it";
			printf("  MISSING brush %d (id %d) plane %d nz %.4f hits %d "
				"near(%.0f,%.0f,%.0f)  <- %s\n",
				m.brush, b.id, m.plane, m.nz, m.hits,
				m.at.X, m.at.Y, m.at.Z, why);
		}
		fflush(stdout);
		return misses.empty() ? 0 : 2;
	}

	// tracegen-quads: turn funcdiff's MISMATCH MANIFEST into direct
	// box-oracle questions (15-field explicit-box rows). Per row:
	//  - CategorizePosition / CheckJumpButton sites: the full hull + four
	//    TracePlayerBBoxForGround quadrant boxes on an xy jitter fan
	//    (down-2u sweeps) - the ground-ordering family.
	//  - Duck-family sites: ZERO-LENGTH boxes - the STANDING hull at the
	//    unduck candidate origin and the DUCKED hull at the probe origin,
	//    on a small xy fan - the exact traces CanUnduck and
	//    FixPlayerCrouchStuck dispute. No meaning assigned: box in,
	//    frac/normal/startsolid out.
	int CmdTraceGenQuads(const std::string& map_path, const ReplayOpts& o,
	                     const std::string& mmpath,
	                     const std::string& out_path) {
		World w;
		std::string err;
		w.true_interval_corner = o.corner_true;
		if (!w.Load(map_path, o.hulls, &err, o.edge_bevels)) {
			printf("LOAD FAILED (bsp): %s\n", err.c_str());
			return 1;
		}
		struct M { char fn[48]; float ox, oy, oz, candz; int ducked, grounded; };
		std::vector<M> ms;
		{
			FILE* f = nullptr;
			if (fopen_s(&f, mmpath.c_str(), "r") != 0 || !f) {
				printf("tracegen-quads: cannot read %s (run funcdiff first - "
					"it writes the mismatch manifest)\n", mmpath.c_str());
				return 1;
			}
			char l[512];
			while (fgets(l, sizeof(l), f)) {
				if (l[0] == '#' || l[0] == 'f') continue;
				M q = {};
				if (sscanf_s(l, "%47[^,],%f,%f,%f,%d,%d,%f",
					q.fn, static_cast<unsigned>(sizeof(q.fn)),
					&q.ox, &q.oy, &q.oz, &q.ducked, &q.grounded,
					&q.candz) == 7)
					ms.push_back(q);
			}
			fclose(f);
		}
		FILE* f = nullptr;
		if (fopen_s(&f, out_path.c_str(), "w") != 0 || !f) {
			printf("tracegen-quads: cannot open %s\n", out_path.c_str());
			return 1;
		}
		fprintf(f, "id,tick,ax,ay,az,bx,by,bz,ducked,mnx,mny,mnz,mxx,mxy,mxz\n");
		int id = 0;
		const float jit[5] = { 0.f, -0.5f, 0.5f, -2.f, 2.f };
		const float jz[3] = { 0.f, -0.5f, 0.5f };
		// One battery per unique SITE - the manifest carries a row per
		// probe and the same position repeats across a battery fan.
		std::vector<long long> seen_sites;
		auto site_seen = [&](const M& m) {
			// 4u granularity: a battery's jitter fan (+/-2) collapses to
			// one site; distinct disputed spots stay distinct.
			const long long key =
				(static_cast<long long>(lroundf(m.ox * 0.25f)) << 32)
				^ (static_cast<long long>(lroundf(m.oy * 0.25f)) << 12)
				^ static_cast<long long>(lroundf(m.oz * 0.25f));
			for (long long s : seen_sites)
				if (s == key)
					return true;
			seen_sites.push_back(key);
			return false;
		};
		for (const M& m : ms) {
			if (site_seen(m))
				continue;
			const bool ground_site =
				strcmp(m.fn, "CategorizePosition") == 0
				|| strcmp(m.fn, "CheckJumpButton") == 0;
			if (ground_site) {
				const float top = m.ducked ? 54.f : 72.f;
				const Vec3 hmn(-16.f, -16.f, 0.f), hmx(16.f, 16.f, top);
				const Vec3 boxes[5][2] = {
					{ hmn, hmx },
					{ hmn, Vec3(0.f, 0.f, hmx.Z) },
					{ Vec3(0.f, 0.f, 0.f), hmx },
					{ Vec3(hmn.X, 0.f, 0.f), Vec3(0.f, hmx.Y, hmx.Z) },
					{ Vec3(0.f, hmn.Y, 0.f), Vec3(hmx.X, 0.f, hmx.Z) },
				};
				for (int jx = 0; jx < 5; ++jx)
				for (int jy = 0; jy < 5; ++jy) {
					const Vec3 a(m.ox + jit[jx], m.oy + jit[jy], m.oz);
					const Vec3 b = a - Vec3(0.f, 0.f, 2.f);
					for (int bi = 0; bi < 5; ++bi)
						fprintf(f, "%d,0,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%d,"
							"%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n",
							id++, a.X, a.Y, a.Z, b.X, b.Y, b.Z, m.ducked,
							boxes[bi][0].X, boxes[bi][0].Y, boxes[bi][0].Z,
							boxes[bi][1].X, boxes[bi][1].Y, boxes[bi][1].Z);
				}
			} else {
				// Duck-family site: the disputed tests, now including the
				// SWEPT ray the air rows actually run (CanUnduck decoded:
				// Ray pos -> cand with the standing hull - the engine's
				// swept startsolid behavior from inside a brush is the one
				// unmeasured law) and the ducked-box LADDER steps the
				// grounded FinishDuck rows dispute.
				for (int jx = 0; jx < 3; ++jx)
				for (int jy = 0; jy < 3; ++jy)
				for (int jzz = 0; jzz < 3; ++jzz) {
					const float dx = jz[jx], dy = jz[jy], dz = jz[jzz];
					// STANDING hull at the unduck candidate.
					fprintf(f, "%d,0,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,0,"
						"-16,-16,0,16,16,72\n",
						id++, m.ox + dx, m.oy + dy, m.candz + dz,
						m.ox + dx, m.oy + dy, m.candz + dz);
					// DUCKED hull at the probe origin (the stuck-fix test).
					fprintf(f, "%d,0,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,1,"
						"-16,-16,0,16,16,54\n",
						id++, m.ox + dx, m.oy + dy, m.oz + dz,
						m.ox + dx, m.oy + dy, m.oz + dz);
				}
				// The air unduck's ACTUAL swept ray: pos -> pos-8.5,
				// standing hull, small xy fan.
				for (int jx = 0; jx < 3; ++jx)
				for (int jy = 0; jy < 3; ++jy) {
					const float dx = jz[jx], dy = jz[jy];
					fprintf(f, "%d,0,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,0,"
						"-16,-16,0,16,16,72\n",
						id++, m.ox + dx, m.oy + dy, m.oz,
						m.ox + dx, m.oy + dy, m.oz - 8.5f);
				}
				// The crouch-stuck LADDER: ducked box at origin + k for
				// k = 1..6 (engine dz ladders measured up to +35; six
				// steps bracket every disputed site's first free rung).
				for (int k = 1; k <= 6; ++k)
					fprintf(f, "%d,0,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,1,"
						"-16,-16,0,16,16,54\n",
						id++, m.ox, m.oy, m.oz + k,
						m.ox, m.oy, m.oz + k);
			}
		}
		fclose(f);
		printf("tracegen-quads: %d manifest row(s) -> %d box queries -> %s\n",
			static_cast<int>(ms.size()), id, out_path.c_str());
		printf("tracegen-quads: in-game 'Run trace oracle' answers them; then\n"
			"  SolverLab quaddiff <map.bsp> \"%s\" <results.csv>\n",
			out_path.c_str());
		fflush(stdout);
		return 0;
	}

	// quaddiff: grade World::TraceHullBox against the engine's own answers
	// for explicit-box queries. Prints every disagreement (frac / normal.z /
	// startsolid / allsolid) - this is the instrument that settles quadrant
	// startsolid semantics by measurement.
	int CmdQuadDiff(const std::string& map_path, const ReplayOpts& o,
	                const std::string& qpath, const std::string& rpath) {
		World w;
		std::string err;
		w.true_interval_corner = o.corner_true;
		if (!w.Load(map_path, o.hulls, &err, o.edge_bevels)) {
			printf("LOAD FAILED (bsp): %s\n", err.c_str());
			return 1;
		}
		struct Q { Vec3 a, b, mn, mx; };
		std::vector<Q> qs;
		{
			FILE* f = nullptr;
			if (fopen_s(&f, qpath.c_str(), "r") != 0 || !f) return 1;
			char l[512];
			while (fgets(l, sizeof(l), f)) {
				if (l[0] == 'i' || l[0] == '#') continue;
				int id, tk, dk;
				float ax, ay, az, bx, by, bz, mnx, mny, mnz, mxx, mxy, mxz;
				if (sscanf_s(l, "%d,%d,%f,%f,%f,%f,%f,%f,%d,%f,%f,%f,%f,%f,%f",
					&id, &tk, &ax, &ay, &az, &bx, &by, &bz, &dk,
					&mnx, &mny, &mnz, &mxx, &mxy, &mxz) == 15)
					qs.push_back({ Vec3(ax, ay, az), Vec3(bx, by, bz),
						Vec3(mnx, mny, mnz), Vec3(mxx, mxy, mxz) });
			}
			fclose(f);
		}
		struct R { float frac, nz; int startsolid, allsolid; };
		std::vector<R> rs;
		{
			FILE* f = nullptr;
			if (fopen_s(&f, rpath.c_str(), "r") != 0 || !f) return 1;
			char l[512];
			while (fgets(l, sizeof(l), f)) {
				if (l[0] == 'i' || l[0] == '#') continue;
				int id, ss, as; float fr, ex, ey, ez, nx, ny, nz, pd;
				if (sscanf_s(l, "%d,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d",
					&id, &fr, &ex, &ey, &ez, &nx, &ny, &nz, &pd, &ss, &as) == 11)
					rs.push_back({ fr, nz, ss, as });
			}
			fclose(f);
		}
		const size_t n = qs.size() < rs.size() ? qs.size() : rs.size();
		int exact = 0, bad = 0;
		for (size_t i = 0; i < n; ++i) {
			TraceResult tr;
			const float frac = w.TraceHullBox(qs[i].a, qs[i].b, qs[i].mn,
				qs[i].mx, &tr);
			const bool ok = fabsf(frac - rs[i].frac) <= 1e-4f
				&& fabsf(tr.normal.Z - rs[i].nz) <= 1e-4f
				&& (tr.startsolid ? 1 : 0) == rs[i].startsolid
				&& (tr.allsolid ? 1 : 0) == rs[i].allsolid;
			if (ok) { exact++; continue; }
			bad++;
			if (bad <= 40)
				printf("  q%-5d a(%9.3f,%9.3f,%9.3f) box(%6.1f,%6.1f)..(%5.1f,%5.1f)"
					"  eng f%.4f nz%+.3f ss%d as%d | ours f%.4f nz%+.3f ss%d as%d\n",
					static_cast<int>(i),
					qs[i].a.X, qs[i].a.Y, qs[i].a.Z,
					qs[i].mn.X, qs[i].mn.Y, qs[i].mx.X, qs[i].mx.Y,
					rs[i].frac, rs[i].nz, rs[i].startsolid, rs[i].allsolid,
					frac, tr.normal.Z, tr.startsolid ? 1 : 0,
					tr.allsolid ? 1 : 0);
		}
		printf("quaddiff: %d box queries | EXACT %d | MISMATCH %d\n",
			static_cast<int>(n), exact, bad);
		fflush(stdout);
		return bad ? 2 : 0;
	}

	// ---- FUNCTION-LEVEL DIFFERENTIAL FUZZ (user directive 2026-08-15) ----
	// The unit under test is the TRANSITION FUNCTION, not a scenario: an
	// arbitrary player state + arbitrary inputs go in, the engine's exact
	// next states come out, and our MoveTick must reproduce every field.
	// Probes are generated to COVER the interaction classes the movement
	// code branches on (sub-epsilon grazes, inside-solid starts, creases,
	// every hull, every duck phase, every speed band) rather than to depict
	// a plausible run. Deterministic LCG: the corpus regenerates identically.
	// Tick 0 is the engine's SETTLE tick (its output seeds both sides);
	// ticks 1..kFuzzTicks-1 are compared. See CmdFuzzDiff.
	constexpr int kFuzzTicks = 5;

	int CmdFuzzGen(const std::string& map_path, const ReplayOpts& o,
	               int nprobes, unsigned seed, const std::string& out_path) {
		World w;
		std::string err;
		w.true_interval_corner = o.corner_true;
		if (!w.Load(map_path, o.hulls, &err, o.edge_bevels)) {
			printf("LOAD FAILED (bsp): %s\n", err.c_str());
			return 1;
		}
		FILE* f = nullptr;
		if (fopen_s(&f, out_path.c_str(), "w") != 0 || !f) {
			printf("fuzzgen: cannot open %s\n", out_path.c_str());
			return 1;
		}
		unsigned rng = seed ? seed : 1u;
		auto next = [&rng]() {
			rng = rng * 1664525u + 1013904223u;
			return (rng >> 8) & 0xFFFFu;
		};
		auto frand = [&next]() { return next() / 65535.f; };
		auto span = [&frand](float lo, float hi) {
			return lo + (hi - lo) * frand();
		};
		// PRECISION LADDER (user directive 2026-08-15: "Consider levels of
		// precision... extremely precise, super small and super large
		// numbers, negatives"). Speeds span denormal-scale through past
		// sv_maxvelocity, and every engine THRESHOLD is sampled from both
		// sides by one float ULP so any comparison that should be < vs <=
		// is forced to declare itself.
		const float kSpeeds[14] = {
			0.f, 1e-30f, 1e-7f, 0.0078125f, 1.f, 29.999998f, 30.f,
			30.000002f, 50.f, 250.f, 850.f, 1500.f, 3499.9998f, 3600.f };
		// Offsets from a face: deep inside, inside, exactly -DIST_EPSILON,
		// on, sub-epsilon, exactly DIST_EPSILON (and one ULP either side),
		// step height, clear.
		const float kOffs[15] = {
			-64.f, -1.f, -0.05f, -0.03125f, -1e-6f, 0.f, 1e-6f, 0.005f,
			0.031249998f, 0.03125f, 0.031250002f, 0.1f, 1.f, 18.f, 64.f };
		const float kHullTops[3] = { 72.f, 54.f, 62.5f };
		// Vertical velocities straddling NON_JUMP_VELOCITY (140) and zero -
		// the gates that own grounding and the surf-friction rule.
		const float kVz[10] = { -1e-7f, 0.f, 1e-7f, 139.999985f, 140.f,
			140.000015f, -140.f, 300.f, -3600.f, 3600.f };

		fprintf(f, "# fuzz probes v1 ticks=%d seed=%u map=%s\n",
			kFuzzTicks, seed, MapStem(map_path).c_str());
		fprintf(f, "id,ox,oy,oz,vx,vy,vz,bx,by,bz,onground,ducked,ducking,"
			"ducktime,stamina,gravity,sfric,hullminz,hullmaxz,hullxy");
		for (int t = 0; t < kFuzzTicks; ++t)
			fprintf(f, ",yaw%d,fmove%d,smove%d,buttons%d", t, t, t, t);
		fprintf(f, "\n");

		for (int i = 0; i < nprobes; ++i) {
			Vec3 pos, vel;
			const int cls = i % 10;
			if (cls == 9) {
				// PRECISION probes: land the hull on an exact engine
				// threshold in one axis with a pathological velocity, and
				// push coordinates out to the world extremes (negative and
				// positive) where float resolution is coarsest.
				const WorldBrush* b = w.brushes.empty() ? nullptr
					: &w.brushes[next() % w.brushes.size()];
				const float huge = (next() & 1u) ? 16384.f : -16384.f;
				if (b && (next() & 1u)) {
					const int pi = static_cast<int>(next()
						% static_cast<unsigned>(b->nsides > 0 ? b->nsides : 1));
					const Vec3 n = b->n[pi];
					Vec3 q(span(b->bmin.X, b->bmax.X),
						span(b->bmin.Y, b->bmax.Y),
						span(b->bmin.Z, b->bmax.Z));
					q = q - Scale(n, Dot(n, q) - b->d[pi]);
					pos = q + Scale(n, kOffs[next() % 15]);
				} else {
					pos = Vec3(huge * frand(), huge * frand(),
						span(-1000.f, 1200.f));
				}
				vel = Vec3(
					(static_cast<int>(next() % 3u) - 1) * kSpeeds[next() % 14],
					(static_cast<int>(next() % 3u) - 1) * kSpeeds[next() % 14],
					kVz[next() % 10]);
			} else if (cls < 5 && !w.brushes.empty()) {
				// GEOMETRY-RELATIVE: park the hull a chosen offset off a
				// real face, so grazes/creases/embeds are all exercised.
				const WorldBrush& b = w.brushes[next() % w.brushes.size()];
				const int pi = static_cast<int>(next() % (b.nsides > 0
					? static_cast<unsigned>(b.nsides) : 1u));
				const Vec3 n = b.n[pi];
				Vec3 q(span(b.bmin.X, b.bmax.X), span(b.bmin.Y, b.bmax.Y),
					span(b.bmin.Z, b.bmax.Z));
				const float d = Dot(n, q) - b.d[pi];
				q = q - Scale(n, d);                       // onto the plane
				// Bias to the OUTSIDE of the face (where a player rides):
				// negative offsets are embedded states, kept as a regression
				// sample rather than a third of the corpus.
				const float off = (next() % 5u == 0u)
					? kOffs[next() % 15]
					: kOffs[5 + next() % 10];
				pos = q + Scale(n, off);
				// Aim mostly INTO the face (the interesting half).
				Vec3 dir(span(-1.f, 1.f), span(-1.f, 1.f), span(-1.f, 1.f));
				const float dl = Len(dir);
				dir = dl > 1e-4f ? Scale(dir, 1.f / dl) : Vec3(1.f, 0.f, 0.f);
				if (Dot(dir, n) > 0.f && (next() & 3u))
					dir = dir - Scale(n, 2.f * Dot(dir, n));
				vel = Scale(dir, kSpeeds[next() % 14]);
			} else {
				// OPEN STATE inside the SEALED play volume. Out-of-world and
				// wall-interior states are already covered by the geometry
				// and precision classes above (and their rules are ported);
				// spending a third of the corpus there starves the states a
				// player can actually occupy.
				const Vec3 lo = w.world_min, hi = w.world_max;
				pos = Vec3(span(lo.X + 80.f, hi.X - 80.f),
					span(lo.Y + 80.f, hi.Y - 80.f),
					span(lo.Z + 80.f, hi.Z - 80.f));
				Vec3 dir(span(-1.f, 1.f), span(-1.f, 1.f), span(-1.f, 1.f));
				const float dl = Len(dir);
				dir = dl > 1e-4f ? Scale(dir, 1.f / dl) : Vec3(0.f, 0.f, -1.f);
				vel = Scale(dir, kSpeeds[next() % 14]);
			}
			const int ducked = (next() % 3u == 0u) ? 1 : 0;
			const int ducking = (next() % 4u == 0u) ? 1 : 0;
			const float hulltop = ducked ? 54.f : kHullTops[next() % 3];
			// Base velocity on a minority of probes (trigger_push physics).
			const bool push = (next() % 8u == 0u);
			const Vec3 bv = push
				? Vec3(span(-400.f, 400.f), span(-400.f, 400.f),
					span(-200.f, 400.f))
				: Vec3();
			fprintf(f, "%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"
				"%d,%d,%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g",
				i, pos.X, pos.Y, pos.Z, vel.X, vel.Y, vel.Z,
				bv.X, bv.Y, bv.Z,
				(next() % 2u) ? 1 : 0, ducked, ducking,
				(next() % 3u == 0u) ? span(0.f, 1000.f) : 0.f,   // ducktime
				(next() % 3u == 0u) ? span(0.f, 1400.f) : 0.f,   // stamina
				(next() % 6u == 0u) ? span(0.2f, 2.f) : 1.f,     // gravity
				(next() % 3u == 0u) ? 0.25f : 1.f,               // sfric
				0.f, hulltop, 16.f);
			for (int t = 0; t < kFuzzTicks; ++t) {
				int btn = 0;
				if (next() % 3u == 0u) btn |= IN_DUCK;
				if (next() % 4u == 0u) btn |= IN_JUMP;
				fprintf(f, ",%.9g,%.9g,%.9g,%d", span(-180.f, 180.f),
					(static_cast<int>(next() % 3u) - 1) * 450.f,
					(static_cast<int>(next() % 3u) - 1) * 450.f, btn);
			}
			fprintf(f, "\n");
		}
		fclose(f);
		printf("fuzzgen: %d probes x %d ticks -> %s\n", nprobes, kFuzzTicks,
			out_path.c_str());
		printf("fuzzgen: in-game Map Solve tab -> 'Run FUNCTION FUZZ', then\n"
			"  SolverLab fuzzdiff <map.bsp> \"%s\" <results.csv>\n",
			out_path.c_str());
		fflush(stdout);
		return 0;
	}

	// Replays every probe through OUR MoveTick and compares each engine
	// output field. Reports by INTERACTION CLASS so an all-green run is a
	// coverage statement, not just a pass count.
	int CmdFuzzDiff(const std::string& map_path, const ReplayOpts& o,
	                const std::string& probe_path,
	                const std::string& res_path) {
		World w;
		std::string err;
		w.true_interval_corner = o.corner_true;
		if (!w.Load(map_path, o.hulls, &err, o.edge_bevels)) {
			printf("LOAD FAILED (bsp): %s\n", err.c_str());
			return 1;
		}
		struct Probe {
			float ox, oy, oz, vx, vy, vz, bx, by, bz;
			int onground, ducked, ducking;
			float ducktime, stamina, gravity, sfric, hminz, hmaxz, hxy;
			float yaw[kFuzzTicks], fm[kFuzzTicks], sm[kFuzzTicks];
			int btn[kFuzzTicks];
		};
		std::vector<Probe> probes;
		{
			FILE* f = nullptr;
			if (fopen_s(&f, probe_path.c_str(), "r") != 0 || !f) {
				printf("fuzzdiff: cannot read %s\n", probe_path.c_str());
				return 1;
			}
			char line[1024];
			while (fgets(line, sizeof(line), f)) {
				if (line[0] == '#' || line[0] == 'i')
					continue;
				Probe p = {};
				int id = 0;
				const int n = sscanf_s(line,
					"%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d,%d,%f,%f,%f,%f,%f,"
					"%f,%f,%f,%f,%f,%d,%f,%f,%f,%d,%f,%f,%f,%d,%f,%f,%f,%d,"
					"%f,%f,%f,%d",
					&id, &p.ox, &p.oy, &p.oz, &p.vx, &p.vy, &p.vz,
					&p.bx, &p.by, &p.bz, &p.onground, &p.ducked, &p.ducking,
					&p.ducktime, &p.stamina, &p.gravity, &p.sfric,
					&p.hminz, &p.hmaxz, &p.hxy,
					&p.yaw[0], &p.fm[0], &p.sm[0], &p.btn[0],
					&p.yaw[1], &p.fm[1], &p.sm[1], &p.btn[1],
					&p.yaw[2], &p.fm[2], &p.sm[2], &p.btn[2],
					&p.yaw[3], &p.fm[3], &p.sm[3], &p.btn[3],
					&p.yaw[4], &p.fm[4], &p.sm[4], &p.btn[4]);
				if (n == 40)
					probes.push_back(p);
			}
			fclose(f);
		}
		struct Res {
			float ox, oy, oz, vx, vy, vz;
			int flags, ducked, ducking;
			float ducktime, stamina, sfric, maxz, bx, by, bz;
			int ok;
		};
		std::vector<Res> res;
		{
			FILE* f = nullptr;
			if (fopen_s(&f, res_path.c_str(), "r") != 0 || !f) {
				printf("fuzzdiff: cannot read %s\n", res_path.c_str());
				return 1;
			}
			char line[1024];
			while (fgets(line, sizeof(line), f)) {
				if (line[0] == '#' || line[0] == 'i')
					continue;
				Res r = {};
				int id = 0, tk = 0;
				const int n = sscanf_s(line,
					"%d,%d,%f,%f,%f,%f,%f,%f,%d,%d,%d,%f,%f,%f,%f,%f,%f,%f,%d",
					&id, &tk, &r.ox, &r.oy, &r.oz, &r.vx, &r.vy, &r.vz,
					&r.flags, &r.ducked, &r.ducking, &r.ducktime, &r.stamina,
					&r.sfric, &r.maxz, &r.bx, &r.by, &r.bz, &r.ok);
				if (n == 19)
					res.push_back(r);
			}
			fclose(f);
		}
		if (probes.empty() || res.size() < probes.size() * kFuzzTicks) {
			printf("fuzzdiff: %d probes, %d result rows - incomplete\n",
				static_cast<int>(probes.size()),
				static_cast<int>(res.size()));
			return 1;
		}
		// Interaction classes, assigned from the PROBE's own setup so the
		// report says what was covered, not just what passed.
		const char* kClassName[8] = {
			"air-open", "air-contact", "ground", "ducked", "duck-transition",
			"basevel", "gravity-scaled", "extreme-speed" };
		int cls_n[8] = {}, cls_bad[8] = {};
		int faulted = 0, mismatch = 0, exact = 0, dumped = 0;
		int reach_n = 0, reach_bad = 0;
		std::map<std::string, int> sig_count;
		int first_bad = -1, first_bad_tick = -1;
		float worst = 0.f;
		int worst_probe = -1;
		for (size_t i = 0; i < probes.size(); ++i) {
			const Probe& p = probes[i];
			int cls = 0;
			const float spd = sqrtf(p.vx * p.vx + p.vy * p.vy + p.vz * p.vz);
			if (spd > 3500.f) cls = 7;
			else if (p.gravity != 1.f) cls = 6;
			else if (p.bx != 0.f || p.by != 0.f || p.bz != 0.f) cls = 5;
			else if (p.ducking) cls = 4;
			else if (p.ducked) cls = 3;
			else if (p.onground) cls = 2;
			else {
				// contact vs open: is anything within a tick's reach?
				TraceResult tr;
				const Vec3 a(p.ox, p.oy, p.oz);
				const Vec3 b(p.ox + p.vx * o.params.dt,
					p.oy + p.vy * o.params.dt, p.oz + p.vz * o.params.dt);
				cls = (w.TraceHull3(a, b, p.hmaxz < 60.f ? 1
					: (p.hmaxz < 70.f ? 2 : 0), &tr) < 1.f) ? 1 : 0;
			}
			cls_n[cls]++;

			// SEED FROM THE ENGINE'S OWN SETTLE TICK (tick 0), never from
			// what the probe asked for: writing FL_ONGROUND does not ground
			// the player (ground is m_hGroundEntity) and the engine
			// re-derives duck state, so asserting an initial state made the
			// harness lie. Ticks 1..K-1 are the actual comparison.
			const Res& seed = res[i * kFuzzTicks];
			PlayerState s;
			s.pos = Vec3(seed.ox, seed.oy, seed.oz);
			s.vel = Vec3(seed.vx, seed.vy, seed.vz);
			// Base velocity comes from the PROBE, not the settle output: the
			// netvar reads zero after the move while the movement plainly
			// still used it (probe 5). Z is already consumed by tick 0's
			// StartGravity.
			s.basevel = Vec3(p.bx, p.by, 0.f);
			s.basevel_flag = (seed.flags & 0x800000) != 0;   // FL_BASEVELOCITY (1<<23)
			if (o.fuzz_nobv) { s.basevel = Vec3(); s.basevel_flag = false; }
			// Ground from GEOMETRY, not the flag: FL_ONGROUND lags the real
			// m_hGroundEntity (probe 111 - the engine executed a stamina-
			// taxed jump, vz 230.08, from a state whose flag read airborne).
			// This is the engine's own grounding rule applied to the seed.
			s.on_ground = false;
			{
				TraceResult gtr;
				const int gh = seed.maxz < 60.f ? 1 : (seed.maxz < 70.f ? 2 : 0);
				const float gfr = w.TraceHull3(Vec3(seed.ox, seed.oy, seed.oz),
					Vec3(seed.ox, seed.oy, seed.oz - 2.f), gh, &gtr);
				if (seed.vz <= o.params.non_jump_velocity && gfr < 1.f
					&& gtr.brush >= 0 && gtr.normal.Z >= o.params.walkable_z)
					s.on_ground = true;
			}
			s.ducked = seed.ducked != 0;
			s.ducking = seed.ducking != 0;
			s.duck_timer_ms = seed.ducktime;
			s.stamina = seed.stamina;
			// A/B: does the engine honour our m_flGravity write at all?
			s.gravity_scale = o.fuzz_grav1 ? 1.f : p.gravity;
			s.surface_friction = seed.sfric;
			s.hull_state = seed.maxz < 60.f ? 1 : (seed.maxz < 70.f ? 2 : 0);
			if (s.on_ground) {
				TraceResult tr;
				const float gf = w.TraceHull3(s.pos,
					s.pos - Vec3(0.f, 0.f, 2.f), s.hull_state, &tr);
				if (gf < 1.f && tr.brush >= 0)
					s.ground_brush = tr.brush;
			}
			// Reachability is known before the replay: seed position, seed
			// hull. --dump-reach restricts forensics to states a player can
			// actually occupy.
			const Vec3 seedp(seed.ox, seed.oy, seed.oz);
			const int seedhull = seed.maxz < 60.f ? 1 : (seed.maxz < 70.f ? 2 : 0);
			const Vec3 rhmn = seedhull == 1 ? o.hulls.duck_min
				: seedhull == 2 ? o.hulls.unduck_min : o.hulls.stand_min;
			const Vec3 rhmx = seedhull == 1 ? o.hulls.duck_max
				: seedhull == 2 ? o.hulls.unduck_max : o.hulls.stand_max;
			const bool reachable =
				seedp.X + rhmn.X >= w.world_min.X && seedp.X + rhmx.X <= w.world_max.X
				&& seedp.Y + rhmn.Y >= w.world_min.Y && seedp.Y + rhmx.Y <= w.world_max.Y
				&& seedp.Z + rhmn.Z >= w.world_min.Z && seedp.Z + rhmx.Z <= w.world_max.Z
				&& !w.OriginInSolid(seedp, seedhull == 1);
			bool bad = false;
			const bool dumping = dumped < o.fuzz_dump
				&& (!o.fuzz_dump_reach || reachable);
			PlayerState s_at_fail = s;
			for (int t = 1; t < kFuzzTicks; ++t) {
				const Res& r = res[i * kFuzzTicks + t];
				if (!r.ok) { faulted++; bad = false; break; }
				s_at_fail = s;
				MoveTick(s, w, o.params, 0.f, p.yaw[t], p.fm[t], p.sm[t],
					0.f, p.btn[t], nullptr);
				const float dp = Len(s.pos - Vec3(r.ox, r.oy, r.oz));
				const float dv = Len(s.vel - Vec3(r.vx, r.vy, r.vz));
				// GROUND: compare PHYSICAL grounding, derived from the
				// engine's own output position/velocity by the engine's own
				// rule - not FL_ONGROUND, which lags the real ground entity
				// (probe 111 jumped from a state whose flag read airborne;
				// probes 282/340/689 sat still on the floor with the flag
				// clear). Comparing the flag was comparing a stale netvar.
				bool eng_ground = false;
				if (r.vz <= o.params.non_jump_velocity) {
					TraceResult etr;
					const int eh = r.maxz < 60.f ? 1 : (r.maxz < 70.f ? 2 : 0);
					const float ef = w.TraceHull3(Vec3(r.ox, r.oy, r.oz),
						Vec3(r.ox, r.oy, r.oz - 2.f), eh, &etr);
					eng_ground = ef < 1.f && etr.brush >= 0
						&& etr.normal.Z >= o.params.walkable_z;
				}
				if (dp > 0.03f || dv > 0.05f
					|| eng_ground != s.on_ground
					|| (r.ducked != 0) != s.ducked
					|| fabsf(r.sfric - s.surface_friction) > 0.01f) {
					bad = true;
					{
						// Signature: WHO moved, and by how much, quantized.
						const Vec3 epos(r.ox, r.oy, r.oz);
						const Vec3 evel(r.vx, r.vy, r.vz);
						const float eng_dp = Len(epos - s_at_fail.pos);
						const float eng_dv = Len(evel - s_at_fail.vel);
						const float our_dp = Len(s.pos - s_at_fail.pos);
						const float our_dv = Len(s.vel - s_at_fail.vel);
						char sg[64];
						if (eng_dp < 1e-3f && eng_dv < 1e-3f)
							_snprintf_s(sg, sizeof(sg), _TRUNCATE,
								"engine INERT, we moved");
						else if (our_dp < 1e-3f && our_dv < 1e-3f)
							_snprintf_s(sg, sizeof(sg), _TRUNCATE,
								"we INERT, engine moved");
						else if (dp < 0.03f)
							_snprintf_s(sg, sizeof(sg), _TRUNCATE,
								"velocity only (dv %.1f)",
								dv < 1.f ? dv : floorf(dv));
						else if (dv < 0.05f)
							_snprintf_s(sg, sizeof(sg), _TRUNCATE,
								"position only (dp %.2f)", dp);
						else
							_snprintf_s(sg, sizeof(sg), _TRUNCATE,
								"both (dp %.1f dv %.0f)", dp, dv);
						sig_count[sg]++;
					}
					if (dp > worst) { worst = dp; worst_probe = static_cast<int>(i); }
					if (first_bad < 0) { first_bad = static_cast<int>(i); first_bad_tick = t; }
					if (dumping) {
						dumped++;
						printf("--- probe %d [%s] diverges at tick %d "
							"(dpos %.4f dvel %.4f)\n", static_cast<int>(i),
							kClassName[cls], t, dp, dv);
						printf("    pre   pos(%.4f,%.4f,%.4f) vel(%.3f,%.3f,%.3f) "
							"g%d d%d/%d hull%d sf%.2f stam%.1f dt%.1f\n",
							s_at_fail.pos.X, s_at_fail.pos.Y, s_at_fail.pos.Z,
							s_at_fail.vel.X, s_at_fail.vel.Y, s_at_fail.vel.Z,
							s_at_fail.on_ground ? 1 : 0,
							s_at_fail.ducked ? 1 : 0, s_at_fail.ducking ? 1 : 0,
							s_at_fail.hull_state, s_at_fail.surface_friction,
							s_at_fail.stamina, s_at_fail.duck_timer_ms);
						printf("    input yaw %.2f fmove %.0f smove %.0f btn %d\n",
							p.yaw[t], p.fm[t], p.sm[t], p.btn[t]);
						printf("    eng   pos(%.4f,%.4f,%.4f) vel(%.3f,%.3f,%.3f) "
							"g%d d%d/%d maxz%.1f sf%.2f\n",
							r.ox, r.oy, r.oz, r.vx, r.vy, r.vz,
							eng_ground ? 1 : 0, r.ducked, r.ducking,
							r.maxz, r.sfric);
						printf("    ours  pos(%.4f,%.4f,%.4f) vel(%.3f,%.3f,%.3f) "
							"g%d d%d/%d hull%d sf%.2f\n",
							s.pos.X, s.pos.Y, s.pos.Z,
							s.vel.X, s.vel.Y, s.vel.Z, s.on_ground ? 1 : 0,
							s.ducked ? 1 : 0, s.ducking ? 1 : 0,
							s.hull_state, s.surface_friction);
					}
					break;
				}
			}
			// Reachable-space split: a player can only ever occupy states
			// inside the sealed world with a hull that fits. Out-of-world
			// and embedded starts are still counted, never excused - but
			// reported separately so the number that governs the solver is
			// visible.
			const Vec3 shmn = s_at_fail.hull_state == 1 ? o.hulls.duck_min
				: s_at_fail.hull_state == 2 ? o.hulls.unduck_min
				: o.hulls.stand_min;
			const Vec3 shmx = s_at_fail.hull_state == 1 ? o.hulls.duck_max
				: s_at_fail.hull_state == 2 ? o.hulls.unduck_max
				: o.hulls.stand_max;
			const Vec3 sp(seed.ox, seed.oy, seed.oz);
			const bool in_world =
				sp.X + shmn.X >= w.world_min.X && sp.X + shmx.X <= w.world_max.X
				&& sp.Y + shmn.Y >= w.world_min.Y && sp.Y + shmx.Y <= w.world_max.Y
				&& sp.Z + shmn.Z >= w.world_min.Z && sp.Z + shmx.Z <= w.world_max.Z
				&& !w.OriginInSolid(sp, s_at_fail.hull_state == 1);
			if (in_world) reach_n++;
			if (bad) {
				mismatch++;
				cls_bad[cls]++;
				if (in_world) reach_bad++;
			} else exact++;
		}
		printf("fuzzdiff: %d probes x %d ticks | EXACT %d | MISMATCH %d | "
			"engine-faulted %d\n", static_cast<int>(probes.size()),
			kFuzzTicks, exact, mismatch, faulted);
		// DEFECT SIGNATURES, ranked by frequency. Every mismatch is a defect
		// in a function; picking probes by eye picks stories instead of
		// mass. This ranks what to fix next by how often it happens.
		printf("fuzzdiff: defect signatures (most frequent first)\n");
		{
			std::vector<std::pair<std::string, int>> sigs(
				sig_count.begin(), sig_count.end());
			std::sort(sigs.begin(), sigs.end(),
				[](const std::pair<std::string, int>& a,
				   const std::pair<std::string, int>& b) {
					return a.second > b.second;
				});
			for (size_t i = 0; i < sigs.size() && i < 14; ++i)
				printf("  %-40s %6d\n", sigs[i].first.c_str(), sigs[i].second);
		}
		printf("fuzzdiff: coverage by interaction class\n");
		for (int c = 0; c < 8; ++c)
			printf("  %-16s probes %6d   mismatches %6d%s\n", kClassName[c],
				cls_n[c], cls_bad[c], cls_n[c] == 0 ? "   <- EMPTY CLASS" : "");
		if (first_bad >= 0)
			printf("fuzzdiff: first mismatch probe %d tick %d; worst |dpos| "
				"%.3f u (probe %d)\n", first_bad, first_bad_tick, worst,
				worst_probe);
		fflush(stdout);
		return mismatch > 0 ? 2 : 0;
	}

	// ---- traceone: per-plane arithmetic dump for ONE hull trace (seam
	// forensics - shows every candidate brush's d0/d1/enterfracs so plane
	// selection disagreements are decided by numbers, not conjecture). ----
	int CmdTraceOne(const std::string& map_path, const ReplayOpts& o,
	                const Vec3& a, const Vec3& b, int hull) {
		World w;
		std::string err;
		w.true_interval_corner = o.corner_true;
		if (!w.Load(map_path, o.hulls, &err, o.edge_bevels)) {
			printf("LOAD FAILED (bsp): %s\n", err.c_str());
			return 1;
		}
		printf("traceone a(%.6f,%.6f,%.6f) -> b(%.6f,%.6f,%.6f) hull %d\n",
			a.X, a.Y, a.Z, b.X, b.Y, b.Z, hull);
		const float kEps = 0.03125f;
		for (const WorldBrush& bc : w.brushes) {
			const std::vector<float>& pd = hull == 1 ? bc.d_duck
				: hull == 2 ? bc.d_unduck : bc.d_stand;
			// Quick AABB reject in origin space.
			const Vec3& gmn = hull == 1 ? bc.gmin_duck
				: hull == 2 ? bc.gmin_unduck : bc.gmin_stand;
			const Vec3& gmx = hull == 1 ? bc.gmax_duck
				: hull == 2 ? bc.gmax_unduck : bc.gmax_stand;
			Vec3 lo(fminf(a.X, b.X), fminf(a.Y, b.Y), fminf(a.Z, b.Z));
			Vec3 hi(fmaxf(a.X, b.X), fmaxf(a.Y, b.Y), fmaxf(a.Z, b.Z));
			if (hi.X < gmn.X - 1.f || lo.X > gmx.X + 1.f
				|| hi.Y < gmn.Y - 1.f || lo.Y > gmx.Y + 1.f
				|| hi.Z < gmn.Z - 1.f || lo.Z > gmx.Z + 1.f)
				continue;
			printf(" brush id %d (%d planes, %d sides):\n", bc.id,
				static_cast<int>(bc.n.size()), bc.nsides);
			for (size_t pi = 0; pi < bc.n.size(); ++pi) {
				const float d0 = Dot(bc.n[pi], a) - pd[pi];
				const float d1 = Dot(bc.n[pi], b) - pd[pi];
				const char* role = d0 > 0.f
					? (d1 > 0.f ? "MISS(both out)" : "ENTER")
					: (d1 > 0.f ? "LEAVE" : "inside");
				float tt = 0.f, tn = 0.f;
				if (d0 != d1) {
					tt = (d0 - kEps) / (d0 - d1);
					tn = d0 / (d0 - d1);
				}
				printf("  p%-2d pid%-3d n(% .4f,% .4f,% .4f) pd %.4f "
					"d0 % .5f d1 % .5f %s tt % .5f tn % .5f\n",
					static_cast<int>(pi), bc.pid[pi],
					bc.n[pi].X, bc.n[pi].Y, bc.n[pi].Z, pd[pi],
					d0, d1, role, tt, tn);
			}
		}
		TraceResult tr;
		const float fr = w.TraceHull3(a, b, hull, &tr);
		printf("our answer: frac %.6f brush %d plane %d n(%.4f,%.4f,%.4f)\n",
			fr, tr.brush >= 0 ? w.brushes[tr.brush].id : -1, tr.plane,
			tr.normal.X, tr.normal.Y, tr.normal.Z);
		fflush(stdout);
		return 0;
	}

	// One row of the DLL's Map Solve ground-truth export (tick -1 = anchor).
	struct EngineRow {
		int tick = 0;
		Vec3 pos, vel;
		int ground = -1, ducked = 0, buttons = 0;
		float yaw = 0.f;
	};

	bool LoadEngineCsv(const std::string& path, std::vector<EngineRow>& out) {
		FILE* f = nullptr;
		if (fopen_s(&f, path.c_str(), "r") != 0 || !f)
			return false;
		char line[512];
		if (!fgets(line, sizeof(line), f)) {   // header
			fclose(f);
			return false;
		}
		while (fgets(line, sizeof(line), f)) {
			EngineRow r;
			float sp2;
			if (sscanf_s(line, "%d,%f,%f,%f,%f,%f,%f,%f,%d,%d,%d,%f",
				&r.tick, &r.pos.X, &r.pos.Y, &r.pos.Z,
				&r.vel.X, &r.vel.Y, &r.vel.Z, &sp2,
				&r.ground, &r.ducked, &r.buttons, &r.yaw) == 12)
				out.push_back(r);
		}
		fclose(f);
		return !out.empty();
	}

	// Replay the tape and diff every tick against the engine's exported states.
	// The payload is the FIRST divergence: everything after it is compounding,
	// so the fix target is always the earliest tick, not the biggest number.
	int CmdDiff(const std::string& map_path, const std::string& tas_path,
	            const std::string& csv_path, const ReplayOpts& o) {
		World w;
		std::string err;
		w.true_interval_corner = o.corner_true;
		if (!w.Load(map_path, o.hulls, &err, o.edge_bevels)) {
			printf("LOAD FAILED (bsp): %s\n", err.c_str());
			return 1;
		}
		Tape tape;
		if (!LoadTas(tas_path, tape, &err)) {
			printf("LOAD FAILED (tas): %s\n", err.c_str());
			return 1;
		}
		std::vector<EngineRow> eng;
		if (!LoadEngineCsv(csv_path, eng)) {
			printf("LOAD FAILED (csv): unreadable or empty\n");
			return 1;
		}

		// Anchor alignment: the export's tick -1 row must be the tape anchor,
		// or the diff below is comparing two different runs.
		int first_row = 0;
		if (eng[0].tick == -1) {
			const float da = Len(eng[0].pos - tape.start.origin);
			if (da > 0.01f)
				printf("WARN: anchor mismatch %.3f u (csv vs tape) - wrong pair?\n", da);
			first_row = 1;
		}
		const int n = static_cast<int>(tape.frames.size()) <
			static_cast<int>(eng.size()) - first_row
			? static_cast<int>(tape.frames.size())
			: static_cast<int>(eng.size()) - first_row;
		printf("diff: %d comparable ticks (tape %d, engine rows %d)\n",
			n, static_cast<int>(tape.frames.size()),
			static_cast<int>(eng.size()) - first_row);

		PlayerState s;
		s.pos = tape.start.origin;
		s.vel = tape.start.velocity;
		s.ducked = tape.start.ducked;
		s.hull_state = tape.start.ducked ? 1 : 0;
		s.stamina = tape.start.stamina;
		{
			TraceResult tr;
			const float gf = w.TraceHull(s.pos, s.pos - Vec3(0.f, 0.f, 2.f),
			                             s.ducked, &tr);
			if (gf < 1.f && tr.brush >= 0 && tr.normal.Z >= o.params.walkable_z) {
				s.pos.Z -= 2.f * gf;
				s.on_ground = true;
				s.ground_brush = tr.brush;
			}
		}

		std::vector<Vec3> core_pos(n), core_vel(n);
		std::vector<int> core_ground(n), core_duck(n);
		int input_mismatch = -1;
		int first01 = -1, first1 = -1, first10 = -1;
		int first_ground = -1, first_duck = -1;
		float max_d = 0.f;
		int max_d_tick = -1;

		for (int t = 0; t < n; ++t) {
			const TapeFrame& f = tape.frames[t];
			const EngineRow& e = eng[first_row + t];
			if (e.tick != t)
				printf("WARN: engine row order broken at %d (row says %d)\n", t, e.tick);
			if (input_mismatch < 0
				&& (e.buttons != f.buttons || fabsf(e.yaw - f.yaw) > 0.01f)) {
				input_mismatch = t;
				printf("INPUT MISMATCH at tick %d: tape yaw %.4f btn %d vs "
					"engine yaw %.4f btn %d - the project's frames differ from "
					"this .tas; state deltas beyond here are not meaningful.\n",
					t, f.yaw, f.buttons, e.yaw, e.buttons);
			}
			TickEvents ev;
			MoveTick(s, w, o.params, f.pitch, f.yaw, f.fmove, f.smove, f.umove,
			         f.buttons, &ev);
			core_pos[t] = s.pos;
			core_vel[t] = s.vel;
			core_ground[t] = s.on_ground ? 1 : 0;
			core_duck[t] = s.ducked ? 1 : 0;
			const float d = Len(s.pos - e.pos);
			if (d > max_d) { max_d = d; max_d_tick = t; }
			if (first01 < 0 && d > 0.1f) first01 = t;
			if (first1 < 0 && d > 1.f) first1 = t;
			if (first10 < 0 && d > 10.f) first10 = t;
			if (first_ground < 0 && e.ground >= 0 && core_ground[t] != e.ground)
				first_ground = t;
			if (first_duck < 0 && core_duck[t] != e.ducked)
				first_duck = t;
		}

		printf("---\n");
		printf("first |dpos| > 0.1u : %s\n", first01 < 0 ? "never"
			: std::to_string(first01).c_str());
		printf("first |dpos| > 1u   : %s\n", first1 < 0 ? "never"
			: std::to_string(first1).c_str());
		printf("first |dpos| > 10u  : %s\n", first10 < 0 ? "never"
			: std::to_string(first10).c_str());
		printf("first ground flag mismatch: %s\n", first_ground < 0 ? "never"
			: std::to_string(first_ground).c_str());
		printf("first duck flag mismatch  : %s\n", first_duck < 0 ? "never"
			: std::to_string(first_duck).c_str());
		printf("max |dpos| %.3f u at tick %d; final |dpos| %.3f u\n",
			max_d, max_d_tick,
			n > 0 ? Len(core_pos[n - 1] - eng[first_row + n - 1].pos) : 0.f);

		// Context dump around the earliest interesting tick: the localization
		// payload (which axis, which subsystem, drift or step).
		const int focus = first01 >= 0 ? first01 : max_d_tick;
		if (focus >= 0) {
			printf("--- context around tick %d (engine | core | delta):\n", focus);
			for (int t = focus - 3; t <= focus + 3; ++t) {
				if (t < 0 || t >= n)
					continue;
				const EngineRow& e = eng[first_row + t];
				const Vec3 dp = core_pos[t] - e.pos;
				const Vec3 dv = core_vel[t] - e.vel;
				printf("t %4d  eng (%9.3f,%9.3f,%9.3f) g%d d%d | core "
					"(%9.3f,%9.3f,%9.3f) g%d d%d | dp (%7.3f,%7.3f,%7.3f) "
					"dv (%7.2f,%7.2f,%7.2f)\n",
					t, e.pos.X, e.pos.Y, e.pos.Z, e.ground, e.ducked,
					core_pos[t].X, core_pos[t].Y, core_pos[t].Z,
					core_ground[t], core_duck[t],
					dp.X, dp.Y, dp.Z, dv.X, dv.Y, dv.Z);
			}
		}
		fflush(stdout);
		return 0;
	}

	// Phase 2 stage shared by the primary solve and every tighten round:
	// optimize up to `nthreads` distinct-tick finishers in parallel (full
	// budget each), then build frames for the winner. Falls back to the
	// explorer's best route when optimization is off or produced nothing.
	// False only when no frames could be built at all.
	bool BuildBestFrames(const World& w, const ExploreConfig& cfg,
	                     const ReplayOpts& o, Explorer& ex,
	                     const ExploreResult& res, int nthreads,
	                     const Tape* seed_tape, std::vector<TapeFrame>& frames,
	                     int* ftick, int* frel, bool* fclean = nullptr) {
		frames.clear();
		*ftick = 0;
		*frel = 0;
		if (fclean)
			*fclean = res.finishers[0].clean;
		if (o.optimize_s > 0.5) {
			OptimizeConfig ocfg;
			ocfg.params = cfg.params;
			ocfg.start_brush_idx = w.IndexOfBrushId(cfg.start_brush_id);
			ocfg.end_brush_id = cfg.end_brush_id;
			ocfg.end_brush_idx = w.IndexOfBrushId(cfg.end_brush_id);
			ocfg.max_zone_jumps = cfg.max_zone_jumps;
			ocfg.min_knot = static_cast<int>(1.f
				/ (cfg.params.dt * cfg.flips_per_sec)) + 1;
			ocfg.max_path_ticks = cfg.max_path_ticks;
			ocfg.goal_touch = o.goal_touch;
			ocfg.zone_clock = o.zone_clock;
			ocfg.aim_contacts = o.aim;
			PlayerState root;
			float root_yaw = 0.f;
			ex.RootState(&root, &root_yaw);

			// Distinct-tick subjects, one hill-climb THREAD each, every one
			// with the full budget (they run concurrently - the wall time is
			// o.optimize_s either way, so parallelism buys subject breadth
			// instead of splitting one budget three ways).
			std::vector<int> subjects;
			for (const Finisher& f : res.finishers) {
				bool dup = false;
				for (int si : subjects)
					if (res.finishers[si].tick == f.tick)
						{ dup = true; break; }
				if (!dup)
					subjects.push_back(
						static_cast<int>(&f - res.finishers.data()));
				if (static_cast<int>(subjects.size()) >= nthreads)
					break;
			}
			std::vector<Explorer::FlatGenome> gs(subjects.size());
			std::vector<OptimizeResult> ors(subjects.size());
			std::vector<char> valid(subjects.size(), 0);
			for (size_t si = 0; si < subjects.size(); ++si)
				valid[si] = ex.ExtractGenome(
					res.finishers[subjects[si]].entry, gs[si]) ? 1 : 0;
			printf("solve: OPTIMIZING %d subject(s) in parallel, %.0fs each%s:\n",
				static_cast<int>(subjects.size()), o.optimize_s,
				o.aim ? ", contact-anchored aim" : ", uniform aim (control)");
			fflush(stdout);
			ocfg.budget_seconds = o.optimize_s;
			{
				std::vector<std::thread> othreads;
				for (size_t si = 0; si < subjects.size(); ++si) {
					if (!valid[si])
						continue;
					OptimizeConfig scfg = ocfg;
					scfg.rng_seed = cfg.rng_seed + static_cast<unsigned>(si) + 1;
					othreads.emplace_back([&, si, scfg]() {
						Optimizer opt2(w, scfg, root, root_yaw, seed_tape);
						ors[si] = opt2.Improve(gs[si]);
					});
				}
				for (auto& th : othreads)
					th.join();
			}
			Explorer::FlatGenome best_g;
			int best_tick = -1;
			bool best_clean = false;
			for (size_t si = 0; si < subjects.size(); ++si) {
				if (!valid[si])
					continue;
				const OptimizeResult& orr = ors[si];
				printf("  subject #%d [%s]: %d -> %d ticks (%d improvements, "
					"%lld evals, %.1fM ticks/s)\n",
					static_cast<int>(si) + 1, orr.clean ? "clean" : "JUMP ",
					orr.initial_tick, orr.best_tick,
					orr.improvements, orr.evals,
					orr.ticks_simulated / (orr.seconds > 0 ? orr.seconds : 1)
						/ 1e6);
				// clean products ALWAYS outrank jump-launched (user rule)
				if (orr.ok && (best_tick < 0
					|| (orr.clean && !best_clean)
					|| (orr.clean == best_clean
						&& orr.best_tick < best_tick))) {
					best_tick = orr.best_tick;
					best_clean = orr.clean;
					best_g = gs[si];
				}
			}
			fflush(stdout);
			if (best_tick > 0) {
				Optimizer optb(w, ocfg, root, root_yaw, seed_tape);
				if (optb.BuildFrames(best_g, frames, ftick, frel)) {
					printf("solve: OPTIMIZED best [%s] %d scored ticks "
						"(%.3f s; %d abs) vs explorer best %d\n",
						best_clean ? "clean" : "JUMP-END",
						*frel, *frel * cfg.params.dt, *ftick,
						res.finishers[0].rel);
					if (fclean)
						*fclean = best_clean;
					return true;
				}
			}
		}
		if (!ex.BuildFrames(res.finishers[0].entry, frames, ftick)) {
			printf("solve: BuildFrames FAILED for the best route - not writing\n");
			return false;
		}
		*frel = res.finishers[0].rel;
		return true;
	}

	// Phase 1: archive-explorer route search. Finds legal start-to-finish
	// routes and writes the best as a .tas the game plays like any recording.
	int CmdSolve(const std::string& map_path, const ReplayOpts& o) {
		World w;
		std::string err;
		w.true_interval_corner = o.corner_true;
		if (!w.Load(map_path, o.hulls, &err, o.edge_bevels)) {
			printf("LOAD FAILED (bsp): %s\n", err.c_str());
			return 1;
		}
		if (o.end_brush < 0) {
			printf("solve needs --end-brush N (the finish platform's BSP id; "
				"mapinfo lists them)\n");
			return 1;
		}

		Tape seed;
		bool have_seed = false;
		if (!o.seed_tas.empty()) {
			if (!LoadTas(o.seed_tas, seed, &err)) {
				printf("LOAD FAILED (seed tas): %s\n", err.c_str());
				return 1;
			}
			have_seed = true;
		}

		// Anchor resolution - the tick-0 state the game and solver AGREE on:
		// explicit --anchor file > --anchor-tas tape > the Map Solve tab's
		// captured solve_anchor.cfg > seed tape's anchor > map spawn.
		TapeAnchor anchor;
		std::string anchor_src;
		bool miss = false;
		if (!o.anchor_file.empty()) {
			if (!LoadAnchorFile(o.anchor_file, anchor, &miss)) {
				printf("LOAD FAILED (anchor file): %s\n", o.anchor_file.c_str());
				return 1;
			}
			anchor_src = o.anchor_file;
		} else if (!o.anchor_tas.empty()) {
			Tape at;
			if (!LoadTas(o.anchor_tas, at, &err) || !at.start.valid) {
				printf("LOAD FAILED (anchor tas): %s\n", err.c_str());
				return 1;
			}
			anchor = at.start;
			anchor_src = std::string("anchor of ") + o.anchor_tas;
		} else if (LoadAnchorFile(CanonicalAnchorPath(), anchor, &miss)) {
			anchor_src = CanonicalAnchorPath() + " (Map Solve capture)";
		} else if (have_seed && seed.start.valid) {
			anchor = seed.start;
			anchor_src = "seed tape anchor";
		} else {
			anchor.valid = true;
			anchor.origin = w.spawn_origin;
			anchor.pitch = w.spawn_pitch;
			anchor.yaw = w.spawn_yaw;
			anchor_src = "map spawn (entities lump)";
		}
		printf("solve: anchor source = %s\n", anchor_src.c_str());
		// A seed replayed from a foreign anchor is garbage - refuse quietly
		// mismatched pairs instead of seeding nonsense cells.
		if (have_seed && seed.start.valid) {
			const float da = Len(seed.start.origin - anchor.origin);
			if (da > 0.5f) {
				printf("solve: WARNING - seed anchor differs from the solve "
					"anchor by %.1f u; SEEDING DISABLED for this run.\n", da);
				have_seed = false;
			}
		}

		ExploreConfig cfg;
		cfg.params = o.params;
		cfg.end_brush_id = o.end_brush;
		cfg.start_brush_id = (o.start_brush >= 0) ? o.start_brush
			: w.BrushUnder(anchor.origin, anchor.ducked);
		cfg.budget_seconds = o.budget_s;
		cfg.max_rollouts = o.rollouts;
		cfg.flips_per_sec = o.flips;
		cfg.rng_seed = o.rng;
		cfg.cell_size = o.cell;
		cfg.max_path_ticks = o.max_ticks;
		cfg.threads = o.threads;
		cfg.seed_limit_ticks = o.seed_ticks;
		cfg.zone_clock = o.zone_clock;
		cfg.goal_touch = o.goal_touch;
		cfg.eloss_bias = o.eloss_bias;
		cfg.energy_frontier = o.energy_frontier;
		cfg.efrontier_mu = o.emix_mu;
		cfg.efrontier_lambda = o.emix_lambda;
		cfg.lookahead_c = o.lookahead_c;
		cfg.lookahead_h = o.lookahead_h;
		cfg.probe_dist = o.probe_dist;
		if (o.probe_dist > 0.f)
			printf("solve: finish-flight probe ON (release check within %.0fu; "
				"clean face-exit endings outrank jump-launched)\n", o.probe_dist);
		if (o.energy_frontier)
			printf("solve: value-mix frontier ON (V = KE + %.2f*gz - %.2f*eloss)\n",
				o.emix_mu, o.emix_lambda);
		if (o.lookahead_c > 1)
			printf("solve: micro-lookahead ON (%d candidates x %d-tick horizon "
				"per knot)\n", o.lookahead_c, o.lookahead_h);
		const int nthreads = ResolveThreadCount(o.threads);
		printf("solve: clock = %s\n", o.zone_clock
			? "ZONE (score starts at startzone exit; prestrafe is free)"
			: "anchor (absolute ticks)");
		if (o.goal_touch)
			printf("solve: GOAL = TOUCH brush %d (segment experiment mode)\n",
				o.end_brush);
		if (!o.eloss_bias)
			printf("solve: dissipation bias OFF (control arm)\n");

		printf("solve: anchor (%.1f, %.1f, %.1f) yaw %.1f | start brush %d, "
			"end brush %d\n", anchor.origin.X, anchor.origin.Y, anchor.origin.Z,
			anchor.yaw, cfg.start_brush_id, cfg.end_brush_id);
		printf("solve: flips/sec %.1f (min knot %d ticks) | budget %.0fs | "
			"rng %u | cell %.0f | threads %d%s\n", cfg.flips_per_sec,
			static_cast<int>(1.f / (cfg.params.dt * cfg.flips_per_sec)) + 1,
			cfg.budget_seconds, cfg.rng_seed, cfg.cell_size,
			nthreads, o.threads > 0 ? "" : " (auto)");
		fflush(stdout);

		Explorer ex(w, cfg);
		ex.SetAnchor(anchor);
		if (have_seed) {
			const int n = ex.SeedFromTape(seed);
			printf("solve: seeded %d ticks from %s%s\n", n, o.seed_tas.c_str(),
				ex.SeedFinishTick() >= 0
					? (std::string(" (incumbent finish tick ")
						+ std::to_string(ex.SeedFinishTick()) + ")").c_str()
					: " (seed tape does NOT finish)");
			fflush(stdout);
		}

		ExploreResult res = ex.Run();
		printf("---\n");
		printf("solve: %lld rollouts, %lld ticks (%.1fM ticks/s), %d entries, "
			"%d cells, %.1fs\n", res.rollouts, res.ticks_simulated,
			res.ticks_simulated / (res.seconds > 0 ? res.seconds : 1) / 1e6,
			res.entries, res.cells, res.seconds);
		if (res.finishers.empty()) {
			printf("solve: NO finish found. Options: more budget (--budget-s), "
				"seed a human run (--seed-tas), different --rng.\n");
			return 2;
		}
		printf("solve: %d finishes; top routes (finish z proves ON-TOP):\n",
			static_cast<int>(res.finishers.size()));
		const int show = res.finishers.size() < 10
			? static_cast<int>(res.finishers.size()) : 10;
		{
			int nclean = 0;
			for (const Finisher& f : res.finishers)
				if (f.clean) nclean++;
			printf("solve: ending classes: %d CLEAN (face exit) / %d "
				"jump-launched%s\n", nclean,
				static_cast<int>(res.finishers.size()) - nclean,
				nclean == 0 ? "  ** NO CLEAN ENDING FOUND **" : "");
		}
		for (int i = 0; i < show; ++i)
			printf("  #%d %s %d scored (%.3f s; %d abs)  spd %.1f  eloss %.0fk  "
				"finish (%.0f, %.0f, z %.2f)\n", i + 1,
				res.finishers[i].clean ? "[clean]" : "[JUMP ]",
				res.finishers[i].rel,
				res.finishers[i].rel * cfg.params.dt, res.finishers[i].tick,
				res.finishers[i].speed,
				res.finishers[i].eloss / 1000.f,
				res.finishers[i].pos.X, res.finishers[i].pos.Y,
				res.finishers[i].pos.Z);
		if (ex.SeedFinishTick() >= 0) {
			const int srel = ex.SeedExitTick() >= 0
				? ex.SeedFinishTick() - ex.SeedExitTick()
				: ex.SeedFinishTick();
			printf("  incumbent (seed tape): %d scored (%.3f s; %d abs)\n",
				o.zone_clock ? srel : ex.SeedFinishTick(),
				(o.zone_clock ? srel : ex.SeedFinishTick()) * cfg.params.dt,
				ex.SeedFinishTick());
		}

		// Cleanest routes - the finisher list re-ranked by cumulative
		// dissipation (the segment lab's "get there while wasting the least
		// energy" view; user theory: energy + smooth boards are the primary
		// drivers). Report only - fitness stays ticks.
		{
			std::vector<int> order(res.finishers.size());
			for (size_t i = 0; i < order.size(); ++i)
				order[i] = static_cast<int>(i);
			std::sort(order.begin(), order.end(), [&](int a, int b) {
				return res.finishers[a].eloss < res.finishers[b].eloss; });
			const int showe = order.size() < 5
				? static_cast<int>(order.size()) : 5;
			printf("solve: cleanest routes (lowest dissipation):\n");
			for (int i = 0; i < showe; ++i) {
				const Finisher& f = res.finishers[order[i]];
				printf("  e%d  %d scored (%.3f s; %d abs)  spd %.1f  "
					"eloss %.1fk  finish (%.0f, %.0f, z %.2f)\n", i + 1,
					f.rel, f.rel * cfg.params.dt, f.tick, f.speed,
					f.eloss / 1000.f, f.pos.X, f.pos.Y, f.pos.Z);
			}
			if (!o.out_eloss.empty() && !order.empty()) {
				std::vector<TapeFrame> ef;
				int et = 0;
				if (ex.BuildFrames(res.finishers[order[0]].entry, ef, &et)) {
					std::string stem_e = map_path;
					const size_t sl = stem_e.find_last_of("\\/");
					if (sl != std::string::npos) stem_e = stem_e.substr(sl + 1);
					const size_t dot = stem_e.find_last_of('.');
					if (dot != std::string::npos) stem_e = stem_e.substr(0, dot);
					if (WriteTas(o.out_eloss, anchor, stem_e, ef, &err))
						printf("solve: cleanest route (%d ticks, %.1fk eloss) "
							"written -> %s\n", et,
							res.finishers[order[0]].eloss / 1000.f,
							o.out_eloss.c_str());
				}
			}
			fflush(stdout);
		}

		// Valuation audit mode: cross-reference a reference tape against the
		// live archive, then stop (no optimize/tighten/output).
		if (!o.audit_tas.empty()) {
			Tape ref;
			if (!LoadTas(o.audit_tas, ref, &err)) {
				printf("audit: cannot load %s: %s\n", o.audit_tas.c_str(),
					err.c_str());
				return 1;
			}
			if (ref.start.valid
				&& Len(ref.start.origin - anchor.origin) > 0.5f)
				printf("audit: WARNING reference anchor differs %.1fu\n",
					Len(ref.start.origin - anchor.origin));
			ex.AuditTape(ref);
			return 0;
		}

		// ---- Phase 2: optimize the best finishers (fitness = finish tick,
		// strict improvements only, every eval through the proven core).
		std::vector<TapeFrame> frames;
		int ftick = 0, frel = 0;
		bool fclean = false;
		if (!BuildBestFrames(w, cfg, o, ex, res, nthreads,
			have_seed ? &seed : nullptr, frames, &ftick, &frel, &fclean))
			return 3;

		// ---- Tighten rounds: re-explore with the SCORED cap just below the
		// incumbent, SEEDED from the incumbent's own tape (restart points all
		// along the best route - the search only has to find where to deviate,
		// not rediscover the whole line). Every finish in a capped round is a
		// strict improvement by construction. Under the zone clock the cap is
		// on post-exit ticks, so rounds may spend MORE prestrafe freely.
		for (int round = 1; round <= o.tighten; ++round) {
			ExploreConfig tcfg = cfg;
			if (o.zone_clock)
				tcfg.max_rel_ticks = frel - 1;
			else
				tcfg.max_path_ticks = ftick - 1;
			tcfg.rng_seed = o.rng + 7777u * static_cast<unsigned>(round);
			printf("--- tighten round %d/%d: cap %d scored ticks, rng %u ---\n",
				round, o.tighten, frel - 1, tcfg.rng_seed);
			fflush(stdout);
			Explorer ex2(w, tcfg);
			ex2.SetAnchor(anchor);
			Tape inc;
			inc.start = anchor;
			inc.start.valid = true;
			inc.frames = frames;
			const int sn = ex2.SeedFromTape(inc);
			printf("tighten: seeded %d restart points from the incumbent "
				"(%d scored, %d abs)\n", sn, frel, ftick);
			fflush(stdout);
			ExploreResult r2 = ex2.Run();
			printf("tighten: %lld rollouts, %d finishes%s\n", r2.rollouts,
				static_cast<int>(r2.finishers.size()),
				r2.finishers.empty()
					? " - no sub-incumbent route this round" : "");
			fflush(stdout);
			if (r2.finishers.empty())
				continue;
			std::vector<TapeFrame> f2;
			int t2 = 0, r2rel = 0;
			bool r2clean = false;
			if (!BuildBestFrames(w, tcfg, o, ex2, r2, nthreads, &inc, f2,
				&t2, &r2rel, &r2clean))
				continue;
			// Pipeline-level ending ratchet (user rule): never trade a clean
			// incumbent for a jump-launched product, no matter the ticks.
			if ((r2clean && !fclean)
				|| (r2clean == fclean && r2rel < frel)) {
				printf("tighten: IMPROVED %d -> %d scored ticks (%.3f s) "
					"[%s]\n", frel, r2rel, r2rel * cfg.params.dt,
					r2clean ? "clean" : "JUMP-END");
				frames = f2;
				ftick = t2;
				frel = r2rel;
				fclean = r2clean;
			}
			fflush(stdout);
		}
		// Labeled output (user-directed): the filename says what it is.
		const std::string out_path = o.out_tas.empty()
			? LabeledOutPath(map_path, "arch", frel, ftick, fclean)
			: o.out_tas;
		if (!WriteTas(out_path, anchor, MapStem(map_path), frames, &err)) {
			printf("solve: WriteTas failed: %s\n", err.c_str());
			return 3;
		}
		QualityScan(w, cfg.params, anchor, frames, cfg.start_brush_id,
			cfg.end_brush_id, true);
		printf("solve: best route [%s] (%d scored ticks, %d abs) written -> %s\n",
			fclean ? "CLEAN ending" : "JUMP-END - NOT an acceptable solution",
			frel, ftick, out_path.c_str());
		PrintTestCard(out_path, frel, ftick, fclean, cfg.params.dt);
		fflush(stdout);
		return 0;
	}

	// LINE-SPACE search (the paradigm shift, user-directed 2026-08-14): a
	// smooth strafe-intensity spline + CMA-ES over its control points. Every
	// candidate is smooth by construction - the efficient family is the ONLY
	// family this search can express. See SolverSmooth.h for the full design.
	int CmdSmooth(const std::string& map_path, const ReplayOpts& o) {
		World w;
		std::string err;
		w.true_interval_corner = o.corner_true;
		if (!w.Load(map_path, o.hulls, &err, o.edge_bevels)) {
			printf("LOAD FAILED (bsp): %s\n", err.c_str());
			return 1;
		}
		if (o.end_brush < 0) {
			printf("smooth needs --end-brush N (mapinfo lists brush ids)\n");
			return 1;
		}
		Tape seed;
		bool have_seed = false;
		if (!o.seed_tas.empty()) {
			if (!LoadTas(o.seed_tas, seed, &err)) {
				printf("LOAD FAILED (seed tas): %s\n", err.c_str());
				return 1;
			}
			have_seed = true;
		}
		// Anchor resolution - the same chain as solve.
		TapeAnchor anchor;
		std::string anchor_src;
		bool miss = false;
		if (!o.anchor_file.empty()) {
			if (!LoadAnchorFile(o.anchor_file, anchor, &miss)) {
				printf("LOAD FAILED (anchor file): %s\n",
					o.anchor_file.c_str());
				return 1;
			}
			anchor_src = o.anchor_file;
		} else if (!o.anchor_tas.empty()) {
			Tape at;
			if (!LoadTas(o.anchor_tas, at, &err) || !at.start.valid) {
				printf("LOAD FAILED (anchor tas): %s\n", err.c_str());
				return 1;
			}
			anchor = at.start;
			anchor_src = std::string("anchor of ") + o.anchor_tas;
		} else if (LoadAnchorFile(CanonicalAnchorPath(), anchor, &miss)) {
			anchor_src = CanonicalAnchorPath() + " (Map Solve capture)";
		} else if (have_seed && seed.start.valid) {
			anchor = seed.start;
			anchor_src = "seed tape anchor";
		} else {
			anchor.valid = true;
			anchor.origin = w.spawn_origin;
			anchor.pitch = w.spawn_pitch;
			anchor.yaw = w.spawn_yaw;
			anchor_src = "map spawn (entities lump)";
		}
		printf("smooth: anchor source = %s\n", anchor_src.c_str());
		if (have_seed && seed.start.valid
			&& Len(seed.start.origin - anchor.origin) > 0.5f) {
			printf("smooth: WARNING seed anchor differs %.1f u - seeding "
				"DISABLED\n", Len(seed.start.origin - anchor.origin));
			have_seed = false;
		}

		SmoothConfig cfg;
		cfg.params = o.params;
		cfg.end_brush_id = o.end_brush;
		cfg.start_brush_id = (o.start_brush >= 0) ? o.start_brush
			: w.BrushUnder(anchor.origin, anchor.ducked);
		cfg.max_ticks = o.max_ticks;
		cfg.cp_ticks = o.cp_ticks;
		cfg.pop = o.pop;
		cfg.budget_seconds = o.budget_s;
		cfg.rng_seed = o.rng;
		cfg.threads = o.threads;
		cfg.zone_clock = o.zone_clock;
		cfg.wloss = o.wloss;
		cfg.mix_mu = o.emix_mu;
		cfg.seed_follow = o.seed_follow;
		cfg.seed_duck = o.seed_duck;
		if (o.seed_follow > 0)
			printf("smooth: prefix-follow ON - first %d ticks verbatim from "
				"the seed tape\n", o.seed_follow);
		printf("smooth: anchor (%.1f, %.1f, %.1f) yaw %.1f | start brush %d, "
			"end brush %d | domain %d ticks\n",
			anchor.origin.X, anchor.origin.Y, anchor.origin.Z, anchor.yaw,
			cfg.start_brush_id, cfg.end_brush_id, cfg.max_ticks);
		fflush(stdout);

		SmoothOpt opt(w, cfg, anchor);
		if (have_seed) {
			opt.SeedFromTape(seed);
			printf("smooth: init mean = strafe skeleton of %s (%d frames)\n",
				o.seed_tas.c_str(), static_cast<int>(seed.frames.size()));
			// The projection's own truth: where the un-perturbed mean gets.
			SmoothStats ms;
			opt.Evaluate(opt.SeedMean(), &ms, nullptr);
			if (ms.finished)
				printf("smooth: projection rollout: %s FINISH %d scored  "
					"eloss %.0fk  max impact %.0f  exit %d\n",
					ms.clean ? "[clean]" : "[JUMP ]", ms.rel,
					ms.eloss / 1000.f, ms.max_impact, ms.exit_tick);
			else
				printf("smooth: projection rollout: no finish, dmin %.0f  "
					"eloss %.0fk  max impact %.0f  exit %d\n",
					ms.dmin, ms.eloss / 1000.f, ms.max_impact,
					ms.exit_tick);
			fflush(stdout);
		}
		SmoothResult res = opt.Run();
		printf("---\n");
		printf("smooth: %d generations, %lld evals, %lld ticks "
			"(%.1fM ticks/s), %d restarts, %.1fs\n",
			res.generations, res.evals, res.ticks_simulated,
			res.ticks_simulated / (res.seconds > 0 ? res.seconds : 1) / 1e6,
			res.restarts, res.seconds);
		if (!res.ok) {
			printf("smooth: NO finish found (closest approach %.0f u). More "
				"budget, --seed-tas, another --rng, or more --max-ticks.\n",
				res.best_stats.dmin);
			// Diagnosis payload: the best non-finisher, written + scanned so
			// the failure is a trajectory we can look at, not a number.
			std::vector<TapeFrame> nf;
			SmoothStats nst;
			if (opt.BuildFrames(res.best_x, nf, &nst) && !o.out_tas.empty()) {
				const std::string np = o.out_tas + ".nofin.tas";
				QualityScan(w, cfg.params, anchor, nf, cfg.start_brush_id,
					cfg.end_brush_id, true);
				if (WriteTas(np, anchor, MapStem(map_path), nf, &err))
					printf("smooth: best NON-finisher written -> %s\n",
						np.c_str());
			}
			return 2;
		}
		printf("smooth: best %s %d scored (%d abs)  finish speed %.0f  "
			"eloss %.0fk  max impact %.0f\n",
			res.clean ? "[clean]" : "[JUMP ]", res.best_rel, res.best_tick,
			res.best_stats.finish_speed, res.best_stats.eloss / 1000.f,
			res.best_stats.max_impact);

		std::vector<TapeFrame> frames;
		SmoothStats st;
		if (!opt.BuildFrames(res.best_x, frames, &st) || !st.finished) {
			printf("smooth: BuildFrames failed to reproduce the finish - "
				"not writing\n");
			return 3;
		}
		QualityScan(w, cfg.params, anchor, frames, cfg.start_brush_id,
			cfg.end_brush_id, true);
		const std::string out_path = o.out_tas.empty()
			? LabeledOutPath(map_path, "cma", st.rel, st.tick, st.clean)
			: o.out_tas;
		if (!WriteTas(out_path, anchor, MapStem(map_path), frames, &err)) {
			printf("smooth: WriteTas failed: %s\n", err.c_str());
			return 3;
		}
		printf("smooth: written -> %s\n", out_path.c_str());
		PrintTestCard(out_path, st.rel, st.tick, st.clean, cfg.params.dt);
		fflush(stdout);
		return st.clean ? 0 : 4;
	}

	// UNSEEDED two-layer solver: exhaustive skeleton search over surf-face
	// sequences x chained segment CMA. No human inputs anywhere.
	int CmdChain(const std::string& map_path, const ReplayOpts& o) {
		World w;
		std::string err;
		w.true_interval_corner = o.corner_true;
		if (!w.Load(map_path, o.hulls, &err, o.edge_bevels)) {
			printf("LOAD FAILED (bsp): %s\n", err.c_str());
			return 1;
		}
		if (o.end_brush < 0) {
			printf("chain needs --end-brush N (mapinfo lists brush ids)\n");
			return 1;
		}
		// Anchor: Map Solve capture > map spawn (no tape inputs by design).
		TapeAnchor anchor;
		bool miss = false;
		std::string anchor_src;
		if (!o.anchor_file.empty()) {
			if (!LoadAnchorFile(o.anchor_file, anchor, &miss)) {
				printf("LOAD FAILED (anchor file): %s\n",
					o.anchor_file.c_str());
				return 1;
			}
			anchor_src = o.anchor_file;
		} else if (LoadAnchorFile(CanonicalAnchorPath(), anchor, &miss)) {
			anchor_src = CanonicalAnchorPath() + " (Map Solve capture)";
		} else {
			anchor.valid = true;
			anchor.origin = w.spawn_origin;
			anchor.pitch = w.spawn_pitch;
			anchor.yaw = w.spawn_yaw;
			anchor_src = "map spawn (entities lump)";
		}
		printf("chain: anchor source = %s (UNSEEDED - no tape inputs)\n",
			anchor_src.c_str());

		ChainConfig cfg;
		cfg.params = o.params;
		cfg.end_brush_id = o.end_brush;
		cfg.start_brush_id = (o.start_brush >= 0) ? o.start_brush
			: w.BrushUnder(anchor.origin, anchor.ducked);
		cfg.total_seconds = o.budget_s;
		cfg.seg_seconds = o.seg_s;
		cfg.end_seconds = o.end_s;
		cfg.final_seconds = o.optimize_s;
		cfg.beam = o.beam;
		cfg.max_depth = o.depth;
		cfg.cp_ticks = o.cp_ticks;
		cfg.rng_seed = o.rng;
		cfg.threads = o.threads;
		cfg.wloss = o.wloss;
		cfg.mix_mu = o.emix_mu;
		cfg.dump_dir = o.csv;   // reuse --csv as the dump directory
		printf("chain: start brush %d, end brush %d\n",
			cfg.start_brush_id, cfg.end_brush_id);
		fflush(stdout);

		ChainSolver cs(w, cfg, anchor);
		ChainResult r = cs.Run();
		printf("---\n");
		printf("chain: %d nodes, %d segment solves, %d clean finishes, "
			"%lld ticks, %.1fs\n", r.nodes_expanded, r.segments_solved,
			r.finishes, r.ticks_simulated, r.seconds);
		if (!r.ok) {
			printf("chain: NO clean line assembled. More --budget-s, "
				"another --rng, more --seg-s.\n");
			if (!r.miss_frames.empty()) {
				// Export the closest NEAR-MISS for in-game review (user
				// request 2026-08-15: "I want to look at it in game to
				// more clearly say what it is doing wrong").
				std::string sk = "[";
				for (size_t i = 0; i < r.miss_skeleton.size(); ++i) {
					if (i) sk += ">";
					sk += std::to_string(
						cs.Faces()[r.miss_skeleton[i]].brush_id);
				}
				sk += "]";
				char tag[48];
				_snprintf_s(tag, sizeof(tag), _TRUNCATE, "chainMISSd%d",
					static_cast<int>(r.miss_dend));
				const std::string mp = LabeledOutPath(map_path, tag,
					static_cast<int>(r.miss_frames.size()),
					static_cast<int>(r.miss_frames.size()), true);
				if (WriteTas(mp, anchor, MapStem(map_path), r.miss_frames,
					&err)) {
					printf("chain: NEAR-MISS %s d%.0f written -> %s\n",
						sk.c_str(), r.miss_dend, mp.c_str());
					QualityScan(w, cfg.params, anchor, r.miss_frames,
						cfg.start_brush_id, cfg.end_brush_id, true);
					fflush(stdout);
				}
			}
			return 2;
		}
		std::string sk = "[";
		for (size_t i = 0; i < r.skeleton.size(); ++i) {
			if (i) sk += ">";
			sk += std::to_string(
				cs.Faces()[r.skeleton[i]].brush_id);
		}
		sk += "]";
		printf("chain: best skeleton %s - %s %d scored (%d abs)\n",
			sk.c_str(), r.clean ? "[clean]" : "[JUMP ]", r.scored,
			r.abs_tick);
		QualityScan(w, cfg.params, anchor, r.frames, cfg.start_brush_id,
			cfg.end_brush_id, true);
		const std::string out_path = o.out_tas.empty()
			? LabeledOutPath(map_path, "chain", r.scored, r.abs_tick,
				r.clean)
			: o.out_tas;
		if (!WriteTas(out_path, anchor, MapStem(map_path), r.frames,
			&err)) {
			printf("chain: WriteTas failed: %s\n", err.c_str());
			return 3;
		}
		printf("chain: written -> %s\n", out_path.c_str());
		PrintTestCard(out_path, r.scored, r.abs_tick, r.clean,
			cfg.params.dt);
		fflush(stdout);
		return r.clean ? 0 : 4;
	}

	// ---- ALIGNMENT BATTERY (user-directed 2026-08-14: "query anything else
	// you are unsure of ingame so we can confirm perfect engine alignment
	// without running into this issue over and over") ----
	//
	// battery-gen synthesizes one .tas INPUT DECK per mechanism the core
	// relies on, anchored at core-settled standing spots. The decks are
	// inputs only - the ENGINE decides what happens; the in-game battery
	// button plays them all with capture armed, and `battery` scores every
	// capture against the core tick by tick. A FAIL is not a bug report,
	// it is a measurement: the mechanism's engine truth is now on disk.

	struct BatteryDeck {
		std::string name;
		TapeAnchor anchor;
		std::vector<TapeFrame> frames;
	};

	void DeckFrame(BatteryDeck& d, float yaw, float fmove, float smove,
	               int buttons, int repeat) {
		TapeFrame f;
		f.yaw = yaw;
		f.fmove = fmove;
		f.smove = smove;
		f.buttons = buttons;
		for (int i = 0; i < repeat; ++i)
			d.frames.push_back(f);
	}

	// Core-settled standing anchor at (x, y, z_probe). False if not ground.
	bool SettleAnchor(const World& w, const MoveParams& p, float x, float y,
	                  float z, float yaw, TapeAnchor& out) {
		PlayerState s;
		s.pos = Vec3(x, y, z);
		TraceResult tr;
		const float gf = w.TraceHull(s.pos, s.pos - Vec3(0.f, 0.f, 64.f),
			false, &tr);
		if (gf >= 1.f || tr.brush < 0 || tr.normal.Z < p.walkable_z)
			return false;
		out.valid = true;
		out.origin = s.pos - Vec3(0.f, 0.f, 64.f * gf);
		out.origin.Z += 0.5f;   // settle margin; engine re-grounds on spawn
		out.velocity = Vec3();
		out.yaw = yaw;
		out.pitch = 0.f;
		out.ducked = false;
		out.stamina = 0.f;
		return true;
	}

	int CmdBatteryGen(const std::string& map_path, const ReplayOpts& o) {
		World w;
		std::string err;
		w.true_interval_corner = o.corner_true;
		if (!w.Load(map_path, o.hulls, &err, o.edge_bevels)) {
			printf("LOAD FAILED (bsp): %s\n", err.c_str());
			return 1;
		}
		const MoveParams& p = o.params;
		std::vector<BatteryDeck> decks;

		// Anchor spots (settle-verified below, coordinates from the map's
		// collision set: green top 448, ramp-2 spine ~256 at y -192).
		TapeAnchor a_green, a_edge, a_spine;
		const bool ok_green = SettleAnchor(w, p, -1600.f, -430.f, 452.f, 0.f,
			a_green);
		const bool ok_edge = SettleAnchor(w, p, -1300.f, -430.f, 452.f, 0.f,
			a_edge);
		bool ok_spine = false;
		for (float yy = -200.f; yy <= -184.f && !ok_spine; yy += 2.f)
			ok_spine = SettleAnchor(w, p, -352.f, yy, 260.f, 0.f, a_spine);
		printf("battery-gen: anchors green=%d edge=%d spine=%d\n",
			ok_green ? 1 : 0, ok_edge ? 1 : 0, ok_spine ? 1 : 0);

		if (ok_spine) {
			// STEPMOVE/EDGE WALKING: the spine walk that carried the 495's
			// ending. Straight hold, then weaves that cross the edge.
			BatteryDeck d;
			d.name = "battery_walk_spine";
			d.anchor = a_spine;
			DeckFrame(d, 0.f, 450.f, 0.f, IN_FORWARD, 50);
			for (int c = 0; c < 4; ++c) {
				DeckFrame(d, 10.f, 450.f, 0.f, IN_FORWARD, 15);
				DeckFrame(d, -10.f, 450.f, 0.f, IN_FORWARD, 15);
			}
			DeckFrame(d, 0.f, 450.f, 0.f, IN_FORWARD, 30);
			decks.push_back(d);
		}
		if (ok_edge) {
			// EDGE RIDE + WALK-OFF: along the platform lip, then off it.
			BatteryDeck d;
			d.name = "battery_walk_edge";
			d.anchor = a_edge;
			DeckFrame(d, 80.f, 450.f, 0.f, IN_FORWARD, 90);   // along +y, drifting to lip
			DeckFrame(d, 0.f, 450.f, 0.f, IN_FORWARD, 120);   // straight off the edge
			decks.push_back(d);
		}
		if (ok_green) {
			{
				// AIR DUCK/UNDUCK cycles clear of surfaces (+-8.5 shifts).
				BatteryDeck d;
				d.name = "battery_duck_air";
				d.anchor = a_green;
				for (int c = 0; c < 3; ++c) {
					DeckFrame(d, 0.f, 0.f, 0.f, 0, 10);
					DeckFrame(d, 0.f, 0.f, 0.f, IN_JUMP, 1);
					DeckFrame(d, 0.f, 0.f, 0.f, 0, 8 + 6 * c);
					DeckFrame(d, 0.f, 0.f, 0.f, IN_DUCK, 20);
					DeckFrame(d, 0.f, 0.f, 0.f, 0, 40);
				}
				decks.push_back(d);
			}
			{
				// STAMINA LADDER v2 (closed-loop): the v1 deck ran off the
				// platform after two jumps (measured), leaving arm and
				// jump-scale DEGENERATE (one hot jump = only the product
				// pinned). v2 REVERSES direction after each landing and
				// presses the next jump a staggered number of ticks after
				// GROUNDING (core-detected), so 6 grounded jumps at
				// distinct stamina residues land on the platform -
				// separating arm, jump scale, and walk scale exactly.
				BatteryDeck d;
				d.name = "battery_stamina";
				d.anchor = a_green;
				PlayerState s;
				s.pos = d.anchor.origin;
				float yaw = 0.f;
				// v3 waits: SHORT cycles keep stamina hot (a full hop cycle
				// is ~55 ticks = 825ms of drain; v2's long waits produced
				// only 2 hot jumps of 6). First and last stay cold as
				// controls; the middle six sample distinct hot residues.
				const int waits[8] = { 40, 2, 4, 7, 10, 14, 18, 90 };
				for (int j = 0; j < 8; ++j) {
					int grounded = 0;
					for (int t = 0; t < 400; ++t) {
						const bool press = s.on_ground
							&& grounded >= waits[j];
						int btn = IN_FORWARD | (press ? IN_JUMP : 0);
						DeckFrame(d, yaw, 450.f, 0.f, btn, 1);
						TickEvents ev;
						MoveTick(s, w, p, 0.f, yaw, 450.f, 0.f, 0.f, btn,
							&ev);
						if (ev.jumped)
							break;
						grounded = s.on_ground ? grounded + 1 : 0;
					}
					// Flight: coast to landing, then reverse heading.
					for (int t = 0; t < 200 && !s.on_ground; ++t) {
						DeckFrame(d, yaw, 0.f, 0.f, 0, 1);
						MoveTick(s, w, p, 0.f, yaw, 0.f, 0.f, 0.f, 0,
							nullptr);
					}
					yaw = yaw == 0.f ? 180.f : 0.f;
				}
				DeckFrame(d, yaw, 0.f, 0.f, 0, 60);
				decks.push_back(d);
			}
			{
				// FRICTION/STOPSPEED decel, fresh and stamina-dragged.
				BatteryDeck d;
				d.name = "battery_friction";
				d.anchor = a_green;
				DeckFrame(d, 0.f, 450.f, 0.f, IN_FORWARD, 60);
				DeckFrame(d, 0.f, 0.f, 0.f, 0, 100);
				DeckFrame(d, 0.f, 450.f, 0.f, IN_FORWARD, 30);
				DeckFrame(d, 0.f, 450.f, 0.f, IN_FORWARD | IN_JUMP, 1);
				DeckFrame(d, 0.f, 450.f, 0.f, IN_FORWARD, 50);
				DeckFrame(d, 0.f, 0.f, 0.f, 0, 90);
				decks.push_back(d);
			}
			{
				// PRESTRAFE ARCS both directions.
				BatteryDeck d;
				d.name = "battery_prestrafe";
				d.anchor = a_green;
				float yaw = 0.f;
				for (int i = 0; i < 80; ++i) {
					yaw += 2.f;
					DeckFrame(d, yaw, 450.f, -450.f,
						IN_FORWARD | IN_MOVELEFT, 1);
				}
				for (int i = 0; i < 80; ++i) {
					yaw -= 2.f;
					DeckFrame(d, yaw, 450.f, 450.f,
						IN_FORWARD | IN_MOVERIGHT, 1);
				}
				decks.push_back(d);
			}
			{
				// GROUND DUCK LIFECYCLE (documented model gap - measure it).
				BatteryDeck d;
				d.name = "battery_duck_ground";
				d.anchor = a_green;
				for (int c = 0; c < 3; ++c) {
					DeckFrame(d, 0.f, 0.f, 0.f, 0, 10);
					DeckFrame(d, 0.f, 0.f, 0.f, IN_DUCK, 50);
					DeckFrame(d, 0.f, 0.f, 0.f, 0, 30);
				}
				decks.push_back(d);
			}
			{
				// DUCK-JUMP FAMILY incl. the user's duck-before-landing:
				// duck-held jump; then jump with a mid-air duck released
				// shortly before touchdown (closed-loop via the core: the
				// release fires when a 3-tick fall reaches ground range).
				BatteryDeck d;
				d.name = "battery_duckjump";
				d.anchor = a_green;
				DeckFrame(d, 0.f, 0.f, 0.f, IN_DUCK | IN_JUMP, 1);
				DeckFrame(d, 0.f, 0.f, 0.f, IN_DUCK, 60);
				DeckFrame(d, 0.f, 0.f, 0.f, 0, 60);
				{
					PlayerState s;
					s.pos = d.anchor.origin;
					float yaw = 0.f;
					for (const TapeFrame& f : d.frames) {
						MoveTick(s, w, p, 0.f, f.yaw, f.fmove, f.smove, 0.f,
							f.buttons, nullptr);
						yaw = f.yaw;
					}
					// Phase 2: jump, duck at apex, release near landing.
					DeckFrame(d, yaw, 0.f, 0.f, IN_JUMP, 1);
					MoveTick(s, w, p, 0.f, yaw, 0.f, 0.f, 0.f, IN_JUMP,
						nullptr);
					bool ducked_phase = false;
					for (int t = 0; t < 160; ++t) {
						int btn = 0;
						if (t > 12)
							ducked_phase = true;
						if (ducked_phase) {
							TraceResult tr;
							const float dist = fmaxf(4.f,
								-s.vel.Z * p.dt * 3.f);
							const float gf = w.TraceHull(s.pos,
								s.pos - Vec3(0.f, 0.f, dist), true, &tr);
							const bool near_ground = gf < 1.f
								&& tr.normal.Z >= p.walkable_z
								&& s.vel.Z < 0.f;
							btn = near_ground ? 0 : IN_DUCK;
							if (near_ground)
								ducked_phase = false;
						}
						DeckFrame(d, yaw, 0.f, 0.f, btn, 1);
						MoveTick(s, w, p, 0.f, yaw, 0.f, 0.f, 0.f, btn,
							nullptr);
						if (s.on_ground && t > 30)
							break;
					}
					DeckFrame(d, yaw, 0.f, 0.f, 0, 30);
				}
				decks.push_back(d);
			}
		}
		if (ok_edge) {
			// NEVER-DUCKED wall slide (the hull-height DISCRIMINATOR): the
			// same fall past ramp 1's bottom edge with no duck anywhere.
			// Predicted release = the standing-72 boundary; a shorter
			// release here would mean the hull is universally short, not a
			// post-unduck transient.
			BatteryDeck d;
			d.name = "battery_wall_stand";
			d.anchor = a_edge;
			PlayerState s;
			s.pos = d.anchor.origin;
			const Vec3 target(-928.f, -440.f, 60.f);
			DeckFrame(d, 25.f, 450.f, 0.f, IN_FORWARD, 8);
			for (int i = 0; i < 8; ++i)
				MoveTick(s, w, p, 0.f, 25.f, 450.f, 0.f, 0.f, IN_FORWARD,
					nullptr);
			DeckFrame(d, 25.f, 450.f, 0.f, IN_FORWARD | IN_JUMP, 1);
			MoveTick(s, w, p, 0.f, 25.f, 450.f, 0.f, 0.f,
				IN_FORWARD | IN_JUMP, nullptr);
			for (int t = 0; t < 260; ++t) {
				const float yaw = atan2f(target.Y - s.pos.Y,
					target.X - s.pos.X) * (180.f / kPi);
				DeckFrame(d, yaw, 450.f, 0.f, IN_FORWARD, 1);
				MoveTick(s, w, p, 0.f, yaw, 450.f, 0.f, 0.f, IN_FORWARD,
					nullptr);
				if (s.pos.Z < -900.f)
					break;
			}
			decks.push_back(d);
		}
		if (ok_edge) {
			// UNDUCK ON A RAMP FACE - the tick-537 divergence scenario:
			// jump toward ramp 1 ducked, ride the face, unduck ON it.
			// Closed-loop synthesis via the core (aim at the face, hold).
			BatteryDeck d;
			d.name = "battery_unduck_face";
			d.anchor = a_edge;
			PlayerState s;
			s.pos = d.anchor.origin;
			const Vec3 target(-928.f, -440.f, 60.f);
			int contact_run = 0;
			bool released = false;
			DeckFrame(d, 25.f, 450.f, 0.f, IN_FORWARD, 8);
			for (int i = 0; i < 8; ++i)
				MoveTick(s, w, p, 0.f, 25.f, 450.f, 0.f, 0.f, IN_FORWARD,
					nullptr);
			DeckFrame(d, 25.f, 450.f, 0.f, IN_FORWARD | IN_JUMP, 1);
			MoveTick(s, w, p, 0.f, 25.f, 450.f, 0.f, 0.f,
				IN_FORWARD | IN_JUMP, nullptr);
			for (int t = 0; t < 320; ++t) {
				const float yaw = atan2f(target.Y - s.pos.Y,
					target.X - s.pos.X) * (180.f / kPi);
				int btn = released ? 0 : IN_DUCK;
				TickEvents ev;
				DeckFrame(d, yaw, 450.f, 0.f, btn | IN_FORWARD, 1);
				MoveTick(s, w, p, 0.f, yaw, 450.f, 0.f, 0.f,
					btn | IN_FORWARD, &ev);
				bool face = false;
				for (int c = 0; c < ev.ncontacts; ++c)
					if (w.brushes[ev.contact_brush[c]].id == 7)
						face = true;
				contact_run = face ? contact_run + 1 : 0;
				if (!released && contact_run >= 25)
					released = true;   // unduck ON the face
				if (s.pos.Z < -900.f)
					break;
			}
			decks.push_back(d);
		}

		// ---- FUZZ DECKS (user directive 2026-08-15: parity must hold "no
		// matter how insane the inputs"). Deterministic extreme inputs -
		// yaw snaps to +-180, duck/jump spam, full-stick reversals, a wall
		// ram - as OPEN-LOOP raw decks. Any core-vs-engine divergence here
		// is a mechanism gap surfaced NOW instead of inside a solver line.
		// LCG-seeded: decks regenerate bit-identically.
		{
			auto fuzz_deck = [&](const char* name, const TapeAnchor& anchor,
			                     unsigned seed, float base_yaw) {
				BatteryDeck d;
				d.name = name;
				d.anchor = anchor;
				unsigned r = seed;
				auto nxt = [&r]() {
					r = r * 1664525u + 1013904223u;
					return (r >> 8) & 0xFFFFu;
				};
				float yaw = base_yaw;
				for (int t = 0; t < 340; ++t) {
					if (nxt() % 7u == 0u)
						yaw += (static_cast<int>(nxt() % 3600u) - 1800)
							* 0.1f;   // snaps up to +-180
					const float fm =
						(static_cast<int>(nxt() % 3u) - 1) * 450.f;
					const float sm =
						(static_cast<int>(nxt() % 3u) - 1) * 450.f;
					int btn = 0;
					if (nxt() % 9u < 3u) btn |= IN_DUCK;
					if (nxt() % 11u < 2u) btn |= IN_JUMP;
					DeckFrame(d, yaw, fm, sm, btn, 1);
				}
				DeckFrame(d, base_yaw, 0.f, 0.f, 0, 30);
				decks.push_back(d);
			};
			if (ok_green) {
				fuzz_deck("battery_fuzz_a", a_green, 0x000F0221u, 0.f);
				fuzz_deck("battery_fuzz_b", a_green, 0x000BEEF7u, 135.f);
			}
			if (ok_edge)
				fuzz_deck("battery_fuzz_wall", a_edge, 0x00C0FFEEu, 25.f);
		}

		const std::string dir = RecordingsDir();
		int wrote = 0;
		for (const BatteryDeck& d : decks) {
			const std::string path = dir + d.name + ".tas";
			if (WriteTas(path, d.anchor, MapStem(map_path), d.frames, &err)) {
				printf("  %-24s %4d ticks -> %s\n", d.name.c_str(),
					static_cast<int>(d.frames.size()), path.c_str());
				wrote++;
			} else {
				printf("  %-24s WRITE FAILED: %s\n", d.name.c_str(),
					err.c_str());
			}
		}
		printf("battery-gen: %d decks written. In-game: inject (fresh library"
			" scan), Map Solve tab -> 'Run ALIGNMENT battery'. Then:\n"
			"  SolverLab battery <map.bsp>\n", wrote);
		fflush(stdout);
		return wrote > 0 ? 0 : 1;
	}

	// TAPE-SLICE deck: any divergence becomes a battery deck. Replays the
	// tape to <start> through the core (valid only while the core matches
	// the engine up to there - check the diff first), anchors a deck at
	// that state, and carries frames [start, end). One battery click later
	// the engine's truth for EXACTLY that window is on disk.
	int CmdBatterySlice(const std::string& map_path, const std::string& tas,
	                    int start, int end, const std::string& name,
	                    const ReplayOpts& o) {
		World w;
		std::string err;
		w.true_interval_corner = o.corner_true;
		if (!w.Load(map_path, o.hulls, &err, o.edge_bevels)) {
			printf("LOAD FAILED (bsp): %s\n", err.c_str());
			return 1;
		}
		Tape tape;
		if (!LoadTas(tas, tape, &err)) {
			printf("LOAD FAILED (tas): %s\n", err.c_str());
			return 1;
		}
		if (start < 0 || end <= start
			|| end > static_cast<int>(tape.frames.size())) {
			printf("battery-slice: bad window [%d, %d) of %d frames\n",
				start, end, static_cast<int>(tape.frames.size()));
			return 1;
		}
		PlayerState s;
		s.pos = tape.start.origin;
		s.vel = tape.start.velocity;
		s.ducked = tape.start.ducked;
		s.hull_state = tape.start.ducked ? 1 : 0;
		s.stamina = tape.start.stamina;
		{
			TraceResult tr;
			const float gf = w.TraceHull(s.pos,
				s.pos - Vec3(0.f, 0.f, 2.f), s.ducked, &tr);
			if (gf < 1.f && tr.brush >= 0
				&& tr.normal.Z >= o.params.walkable_z) {
				s.pos.Z -= 2.f * gf;
				s.on_ground = true;
				s.ground_brush = tr.brush;
			}
		}
		// Contact states cannot seed the engine sim (measured: a mid-ride
		// anchor triggered the engine's stuck resolution - velocity zeroed,
		// player extruded). Snap the start BACK to the last FREE-AIR tick
		// (no contacts, not grounded) at or before the requested one.
		PlayerState free_state = s;
		int free_tick = 0;
		for (int t = 0; t < start; ++t) {
			const TapeFrame& f = tape.frames[t];
			TickEvents ev;
			MoveTick(s, w, o.params, f.pitch, f.yaw, f.fmove, f.smove,
				f.umove, f.buttons, &ev);
			if (ev.ncontacts == 0 && !s.on_ground) {
				free_state = s;
				free_tick = t + 1;
			}
		}
		if (free_tick != start) {
			printf("battery-slice: start %d is a contact state - snapped "
				"back to free-air tick %d\n", start, free_tick);
			s = free_state;
			start = free_tick;
		}
		TapeAnchor a;
		a.valid = true;
		a.origin = s.pos;
		a.velocity = s.vel;
		a.ducked = s.ducked;
		a.stamina = s.stamina;
		a.yaw = tape.frames[start].yaw;
		a.pitch = 0.f;
		std::vector<TapeFrame> fr(tape.frames.begin() + start,
			tape.frames.begin() + end);
		const std::string path = RecordingsDir() + "battery_" + name
			+ ".tas";
		if (!WriteTas(path, a, MapStem(map_path), fr, &err)) {
			printf("battery-slice: write failed: %s\n", err.c_str());
			return 1;
		}
		printf("battery-slice: %s [%d, %d) anchored at "
			"(%.2f, %.2f, %.2f) v(%.0f, %.0f, %.0f) d%d -> %s\n",
			tas.c_str(), start, end, a.origin.X, a.origin.Y, a.origin.Z,
			a.velocity.X, a.velocity.Y, a.velocity.Z, a.ducked ? 1 : 0,
			path.c_str());
		return 0;
	}

	// Score every battery capture against the core, per mechanism.
	int CmdBattery(const std::string& map_path, const ReplayOpts& o) {
		World w;
		std::string err;
		w.true_interval_corner = o.corner_true;
		if (!w.Load(map_path, o.hulls, &err, o.edge_bevels)) {
			printf("LOAD FAILED (bsp): %s\n", err.c_str());
			return 1;
		}
		const std::string rec = RecordingsDir();
		char documents[MAX_PATH];
		std::string sol;
		if (SUCCEEDED(SHGetFolderPathA(nullptr, CSIDL_PERSONAL, nullptr,
			SHGFP_TYPE_CURRENT, documents)))
			sol = std::string(documents) + "\\sourceTAS\\solver\\";
		WIN32_FIND_DATAA fd;
		HANDLE h = FindFirstFileA((rec + "battery_*.tas").c_str(), &fd);
		if (h == INVALID_HANDLE_VALUE) {
			printf("battery: no battery_*.tas decks in %s - run battery-gen "
				"first.\n", rec.c_str());
			return 1;
		}
		printf("battery: %-22s %6s %11s %10s  %s\n", "mechanism", "ticks",
			"first>0.1u", "max dpos", "verdict");
		int pass = 0, fail = 0, missing = 0;
		do {
			std::string name = fd.cFileName;
			name = name.substr(0, name.size() - 4);   // strip .tas
			Tape tape;
			if (!LoadTas(rec + fd.cFileName, tape, &err))
				continue;
			// Newest matching capture from EITHER source: engine query
			// (enginesim_<name>*.csv) or physical playback
			// (playback_<name>*.csv). Newest wins; source is reported.
			std::string best_csv, src = "?";
			FILETIME best_t = { 0, 0 };
			const char* prefixes[2] = { "enginesim_", "playback_" };
			const char* srctag[2] = { "query", "play" };
			for (int pi = 0; pi < 2; ++pi) {
				WIN32_FIND_DATAA cd;
				HANDLE hc = FindFirstFileA(
					(sol + prefixes[pi] + name + "*.csv").c_str(), &cd);
				if (hc == INVALID_HANDLE_VALUE)
					continue;
				do {
					if (CompareFileTime(&cd.ftLastWriteTime, &best_t) > 0) {
						best_t = cd.ftLastWriteTime;
						best_csv = sol + cd.cFileName;
						src = srctag[pi];
					}
				} while (FindNextFileA(hc, &cd));
				FindClose(hc);
			}
			if (best_csv.empty()) {
				printf("battery: %-22s %6d %11s %10s  NO CAPTURE\n",
					name.c_str() + 8,
					static_cast<int>(tape.frames.size()), "-", "-");
				missing++;
				continue;
			}
			std::vector<EngineRow> eng;
			if (!LoadEngineCsv(best_csv, eng)) {
				printf("battery: %-22s capture unreadable\n",
					name.c_str() + 8);
				missing++;
				continue;
			}
			// Extended DIRECT-READ columns (hulltop, mspd_a, mspd_b, and the
			// engine's own per-tick m_flStamina) plus the SELF-DESCRIBING
			// header: "# map" preconditions the world; "# param <key> <val>"
			// lines carry the LIVE server settings that governed the capture
			// and OVERRIDE the cfg for this deck's replay (user directive
			// 2026-08-15: one click carries everything - a server setting
			// change adapts with zero manual steps).
			std::vector<float> eng_hull, eng_ma, eng_mb, eng_stam;
			std::string cap_map;
			MoveParams mp = o.params;
			Hulls mh = o.hulls;
			int nparam = 0;
			{
				FILE* xf = nullptr;
				if (fopen_s(&xf, best_csv.c_str(), "r") == 0 && xf) {
					char xline[512];
					while (fgets(xline, sizeof(xline), xf)) {
						if (strncmp(xline, "# map ", 6) == 0) {
							cap_map = xline + 6;
							while (!cap_map.empty()
								&& (cap_map.back() == '\n'
									|| cap_map.back() == '\r'))
								cap_map.pop_back();
							continue;
						}
						if (strncmp(xline, "# param ", 8) == 0) {
							char key[64] = "";
							float val = 0.f;
							if (sscanf_s(xline + 8, "%63s %f", key,
								static_cast<unsigned>(sizeof(key)), &val) == 2
								&& ApplyParamKey(mp, mh, key, val))
								nparam++;
							continue;
						}
						if (xline[0] == '#'
							|| strncmp(xline, "tick,", 5) == 0)
							continue;
						int tk, g, dk, bt;
						float x, y, z, vx, vy, vz, sp, yw, ht, ma, mb, stm;
						const int nf = sscanf_s(xline,
							"%d,%f,%f,%f,%f,%f,%f,%f,%d,%d,%d,%f,%f,%f,%f,%f",
							&tk, &x, &y, &z, &vx, &vy, &vz, &sp, &g, &dk,
							&bt, &yw, &ht, &ma, &mb, &stm);
						if (nf >= 15) {
							eng_hull.push_back(ht);
							eng_ma.push_back(ma);
							eng_mb.push_back(mb);
							eng_stam.push_back(nf >= 16 ? stm : -1.f);
						}
					}
					fclose(xf);
				}
			}
			// MAP PRECONDITION: refuse captures from the wrong world (they
			// are real engine values - of a different question).
			if (!cap_map.empty() && !tape.map.empty() && cap_map != tape.map) {
				printf("battery: %-22s %6d %11s %10s  WRONG-MAP capture "
					"(%s) - retake on %s\n", name.c_str() + 8,
					static_cast<int>(tape.frames.size()), "-", "-",
					cap_map.c_str(), tape.map.c_str());
				missing++;
				continue;
			}
			int first_row = eng[0].tick == -1 ? 1 : 0;
			PlayerState s;
			s.pos = tape.start.origin;
			s.vel = tape.start.velocity;
			s.ducked = tape.start.ducked;
			s.hull_state = tape.start.ducked ? 1 : 0;
			s.stamina = tape.start.stamina;
			{
				// Engine-sim start semantics: RequestSim begins at the RAW
				// anchor - categorize ground WITHOUT snapping the origin
				// (the physical-playback settle snap belongs to teleports,
				// not to sims; the snap read as a fake 0.5u tick-0 FAIL).
				TraceResult tr;
				const float gf = w.TraceHull(s.pos,
					s.pos - Vec3(0.f, 0.f, 2.f), s.ducked, &tr);
				if (gf < 1.f && tr.brush >= 0
					&& tr.normal.Z >= mp.walkable_z) {
					s.on_ground = true;
					s.ground_brush = tr.brush;
				}
			}
			const int n = static_cast<int>(tape.frames.size())
				< static_cast<int>(eng.size()) - first_row
				? static_cast<int>(tape.frames.size())
				: static_cast<int>(eng.size()) - first_row;
			int first01 = -1;
			float maxd = 0.f;
			// Per-tick STAMINA verification: the engine's own m_flStamina
			// (a READ) vs the model's clock - drift is caught at its birth
			// tick with both values on record.
			int stam_first = -1, stam_bad = 0;
			float stam_model_at = 0.f, stam_eng_at = 0.f;
			for (int t = 0; t < n; ++t) {
				const TapeFrame& f = tape.frames[t];
				MoveTick(s, w, mp, f.pitch, f.yaw, f.fmove, f.smove,
					f.umove, f.buttons, nullptr);
				const float dd = Len(s.pos - eng[first_row + t].pos);
				if (dd > maxd)
					maxd = dd;
				if (first01 < 0 && dd > 0.1f)
					first01 = t;
				if (t < static_cast<int>(eng_stam.size())
					&& eng_stam[t] >= 0.f
					&& fabsf(s.stamina - eng_stam[t]) > 0.1f) {
					stam_bad++;
					if (stam_first < 0) {
						stam_first = t;
						stam_model_at = s.stamina;
						stam_eng_at = eng_stam[t];
					}
				}
			}
			const bool okp = first01 < 0;
			char fbuf[16];
			if (okp)
				_snprintf_s(fbuf, sizeof(fbuf), _TRUNCATE, "never");
			else
				_snprintf_s(fbuf, sizeof(fbuf), _TRUNCATE, "%d", first01);
			printf("battery: %-22s %6d %11s %9.3fu  [%s] %s\n",
				name.c_str() + 8, n, fbuf, maxd, src.c_str(),
				okp ? "PASS" : "FAIL  <- engine truth on disk, fit it");
			if (!eng_ma.empty()) {
				// WEAPON GUARD (user question 2026-08-14): m_flMaxSpeed is
				// weapon-dependent - flag any capture whose engine-read
				// maxspeed disagrees with the params this replay USED (the
				// capture's own "# param maxspeed" when present, else cfg).
				bool mismatch = false;
				for (float v : eng_ma)
					if (fabsf(v - mp.maxspeed) > 0.5f)
						mismatch = true;
				if (mismatch)
					printf("battery:   WEAPON/MAXSPEED MISMATCH: engine "
						"m_flMaxSpeed differs from replay maxspeed %.0f - "
						"capture taken with a different weapon; retake.\n",
						mp.maxspeed);
			}
			if (!eng_hull.empty()) {
				// Distinct read values (identification data). The hulltop
				// NETVAR is a known constant 62 in every state - proven NOT
				// the movement hull (2026-08-14) - so it is reported as
				// data, never compared.
				std::vector<float> da, db, dh;
				for (size_t k = 0; k < eng_ma.size(); ++k) {
					bool fa = false, fb = false, fh = false;
					for (float v : da) if (fabsf(v - eng_ma[k]) < 0.01f) fa = true;
					for (float v : db) if (fabsf(v - eng_mb[k]) < 0.01f) fb = true;
					for (float v : dh) if (fabsf(v - eng_hull[k]) < 0.01f) fh = true;
					if (!fa && da.size() < 4) da.push_back(eng_ma[k]);
					if (!fb && db.size() < 4) db.push_back(eng_mb[k]);
					if (!fh && dh.size() < 4) dh.push_back(eng_hull[k]);
				}
				printf("battery:   reads: params %s | stam ",
					nparam > 0 ? "capture" : "cfg");
				const bool have_stam = !eng_stam.empty() && eng_stam[0] >= 0.f;
				if (!have_stam)
					printf("n/a");
				else if (stam_first < 0)
					printf("eng==model every tick");
				else
					printf("MISMATCH first t%d (%d ticks; model %.2f eng %.2f)",
						stam_first, stam_bad, stam_model_at, stam_eng_at);
				printf(" | hullnv {");
				for (size_t k = 0; k < dh.size(); ++k)
					printf("%s%.1f", k ? "," : "", dh[k]);
				printf("} mspd_a {");
				for (size_t k = 0; k < da.size(); ++k)
					printf("%s%.1f", k ? "," : "", da[k]);
				printf("} mspd_b {");
				for (size_t k = 0; k < db.size(); ++k)
					printf("%s%.1f", k ? "," : "", db[k]);
				printf("}\n");
			}
			if (okp) pass++; else fail++;
		} while (FindNextFileA(h, &fd));
		FindClose(h);
		printf("battery: %d PASS, %d FAIL, %d missing capture\n",
			pass, fail, missing);
		fflush(stdout);
		return fail > 0 ? 2 : 0;
	}

	int CmdBench(const std::string& map_path, long long ticks) {
		World w;
		std::string err;
		if (!w.Load(map_path, Hulls(), &err)) {
			printf("LOAD FAILED: %s\n", err.c_str());
			return 1;
		}
		MoveParams p;
		const Vec3 start = w.have_spawn ? w.spawn_origin + Vec3(0.f, 0.f, 100.f)
			: Vec3(0.f, 0.f, 500.f);

		PlayerState s;
		s.pos = start;
		s.vel = Vec3(400.f, 0.f, 0.f);
		float yaw = 0.f;
		int side = 1;
		long long done = 0;
		int resets = 0;
		const auto t0 = std::chrono::steady_clock::now();
		while (done < ticks) {
			// Synthetic legal-ish strafing: one side per 20 ticks, smooth yaw.
			if (done % 20 == 0) side = -side;
			yaw += 0.8f * static_cast<float>(side);
			TickEvents ev;
			MoveTick(s, w, p, 0.f, yaw, 0.f, side > 0 ? -450.f : 450.f, 0.f, 0, &ev);
			++done;
			if (s.on_ground || s.pos.Z < -2000.f) {
				s = PlayerState();
				s.pos = start;
				s.vel = Vec3(400.f, 0.f, 0.f);
				yaw = 0.f;
				resets++;
			}
		}
		const double sec = std::chrono::duration<double>(
			std::chrono::steady_clock::now() - t0).count();
		printf("bench: %lld ticks in %.3f s = %.0f ticks/s single-thread "
			"(%d resets)\n", ticks, sec, ticks / sec, resets);
		fflush(stdout);
		return 0;
	}

	// Run OUR TraceHull over a query file directly (results-format output) -
	// verifies a TraceHull change against stored engine answers without
	// depending on replay trajectories.
	int CmdTraceSelf(const std::string& map_path, const std::string& q_path,
	                 const std::string& out_path, const ReplayOpts& o) {
		World w;
		std::string err;
		w.true_interval_corner = o.corner_true;
		if (!w.Load(map_path, o.hulls, &err, o.edge_bevels)) {
			printf("LOAD FAILED (bsp): %s\n", err.c_str());
			return 1;
		}
		FILE* q = nullptr;
		if (fopen_s(&q, q_path.c_str(), "r") != 0 || !q) {
			printf("traceself: cannot open %s\n", q_path.c_str());
			return 1;
		}
		FILE* r = nullptr;
		if (fopen_s(&r, out_path.c_str(), "w") != 0 || !r) {
			fclose(q);
			printf("traceself: cannot write %s\n", out_path.c_str());
			return 1;
		}
		fprintf(r, "id,frac,ex,ey,ez,nx,ny,nz,pdist,startsolid,allsolid\n");
		char line[512];
		int n = 0;
		while (fgets(line, sizeof(line), q)) {
			int id = 0, tick = 0, ducked = 0;
			float ax, ay, az, bx, by, bz;
			if (sscanf_s(line, "%d,%d,%f,%f,%f,%f,%f,%f,%d", &id, &tick,
				&ax, &ay, &az, &bx, &by, &bz, &ducked) != 9)
				continue;
			TraceResult tr;
			const Vec3 a(ax, ay, az), b(bx, by, bz);
			const float f = w.TraceHull(a, b, ducked != 0, &tr);
			const Vec3 e = a + Scale(b - a, f);
			fprintf(r, "%d,%.9g,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,0,0,0\n",
				id, f, e.X, e.Y, e.Z,
				tr.brush >= 0 ? tr.normal.X : 0.f,
				tr.brush >= 0 ? tr.normal.Y : 0.f,
				tr.brush >= 0 ? tr.normal.Z : 0.f);
			n++;
		}
		fclose(q);
		fclose(r);
		printf("traceself: %d answers -> %s\n", n, out_path.c_str());
		return 0;
	}

	// Compare OUR TraceHull answers against the engine oracle's, trace by
	// trace. queries.csv = replay --trace-log output (has our answers);
	// results.csv = the Map Solve tab's engine answers for the same ids.
	int CmdTraceDiff(const std::string& q_path, const std::string& r_path) {
		struct Q { int tick; float ax, ay, az, bx, by, bz, frac, nx, ny, nz; int ducked, brush; };
		struct R { float frac, nx, ny, nz; int startsolid, allsolid; };
		std::vector<Q> qs;
		std::vector<R> rs;
		{
			FILE* f = nullptr;
			if (fopen_s(&f, q_path.c_str(), "r") != 0 || !f) {
				printf("tracediff: cannot open %s\n", q_path.c_str());
				return 1;
			}
			char line[512];
			while (fgets(line, sizeof(line), f)) {
				Q q;
				int id;
				if (sscanf_s(line, "%d,%d,%f,%f,%f,%f,%f,%f,%d,%f,%f,%f,%f,%d",
					&id, &q.tick, &q.ax, &q.ay, &q.az, &q.bx, &q.by, &q.bz,
					&q.ducked, &q.frac, &q.nx, &q.ny, &q.nz, &q.brush) == 14)
					qs.push_back(q);
			}
			fclose(f);
		}
		{
			FILE* f = nullptr;
			if (fopen_s(&f, r_path.c_str(), "r") != 0 || !f) {
				printf("tracediff: cannot open %s\n", r_path.c_str());
				return 1;
			}
			char line[512];
			while (fgets(line, sizeof(line), f)) {
				R r;
				int id;
				float ex, ey, ez, pd;
				if (sscanf_s(line, "%d,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d",
					&id, &r.frac, &ex, &ey, &ez, &r.nx, &r.ny, &r.nz, &pd,
					&r.startsolid, &r.allsolid) == 11)
					rs.push_back(r);
			}
			fclose(f);
		}
		const size_t n = qs.size() < rs.size() ? qs.size() : rs.size();
		if (n == 0) {
			printf("tracediff: no comparable rows (%zu queries, %zu results)\n",
				qs.size(), rs.size());
			return 1;
		}
		if (qs.size() != rs.size())
			printf("tracediff: WARNING row count mismatch (%zu vs %zu) - "
				"comparing the first %zu\n", qs.size(), rs.size(), n);
		int exact = 0, close = 0, loose = 0, bad = 0, ss = 0;
		struct Worst { float d; size_t i; };
		std::vector<Worst> worst;
		for (size_t i = 0; i < n; ++i) {
			const float d = fabsf(qs[i].frac - rs[i].frac);
			if (rs[i].startsolid) ss++;
			if (d < 1e-6f) exact++;
			else if (d < 1e-4f) close++;
			else if (d < 1e-3f) loose++;
			else {
				bad++;
				worst.push_back({ d, i });
			}
		}
		std::sort(worst.begin(), worst.end(),
			[](const Worst& a, const Worst& b) { return a.d > b.d; });
		printf("tracediff: %zu traces | exact(<1e-6) %d  close(<1e-4) %d  "
			"loose(<1e-3) %d  MISMATCH %d  (engine startsolid on %d)\n",
			n, exact, close, loose, bad, ss);
		const int show = static_cast<int>(worst.size()) < 12
			? static_cast<int>(worst.size()) : 12;
		for (int i = 0; i < show; ++i) {
			const size_t k = worst[i].i;
			printf("  tick %4d  dfrac %.6f  ours %.6f eng %.6f%s  "
				"a(%.2f,%.2f,%.2f) b(%.2f,%.2f,%.2f) d%d  our-brush %d  "
				"our-n(%.2f,%.2f,%.2f) eng-n(%.2f,%.2f,%.2f)\n",
				qs[k].tick, worst[i].d, qs[k].frac, rs[k].frac,
				rs[k].startsolid ? " [eng STARTSOLID]" : "",
				qs[k].ax, qs[k].ay, qs[k].az, qs[k].bx, qs[k].by, qs[k].bz,
				qs[k].ducked, qs[k].brush,
				qs[k].nx, qs[k].ny, qs[k].nz, rs[k].nx, rs[k].ny, rs[k].nz);
		}
		fflush(stdout);
		return bad > 0 ? 2 : 0;
	}

} // namespace

int main(int argc, char** argv) {
	if (argc < 2) {
		PrintUsage();
		return 1;
	}
	// Long campaigns run on WALL-time budgets; a machine that sleeps mid-
	// campaign silently eats them (measured 2026-08-15: a 900s chain run
	// expired with ~30s of real compute). Hold a system-required request
	// for the process lifetime - Windows clears it automatically on exit.
	SetThreadExecutionState(ES_CONTINUOUS | ES_SYSTEM_REQUIRED);
	const std::string cmd = argv[1];
	if (cmd == "mapinfo" && argc >= 3)
		return CmdMapInfo(argv[2]);
	if (cmd == "replay" && argc >= 4) {
		ReplayOpts o;
		if (!ParseCommon(argc, argv, 4, o))
			return 1;
		return CmdReplay(argv[2], argv[3], o);
	}
	if (cmd == "diff" && argc >= 5) {
		ReplayOpts o;
		if (!ParseCommon(argc, argv, 5, o))
			return 1;
		return CmdDiff(argv[2], argv[3], argv[4], o);
	}
	if (cmd == "solve" && argc >= 3) {
		ReplayOpts o;
		if (!ParseCommon(argc, argv, 3, o))
			return 1;
		return CmdSolve(argv[2], o);
	}
	if (cmd == "smooth" && argc >= 3) {
		ReplayOpts o;
		o.max_ticks = 1200;   // smooth default: spline domain, not the
		                      // explorer's 4000 junk bound (--max-ticks wins)
		if (!ParseCommon(argc, argv, 3, o))
			return 1;
		return CmdSmooth(argv[2], o);
	}
	if (cmd == "chain" && argc >= 3) {
		ReplayOpts o;
		o.budget_s = 240.0;   // chain default: a campaign, not a screen
		o.optimize_s = 30.0;  // final global polish
		o.cp_ticks = 8;       // board windows need the fine basis
		if (!ParseCommon(argc, argv, 3, o))
			return 1;
		return CmdChain(argv[2], o);
	}
	if (cmd == "battery-gen" && argc >= 3) {
		ReplayOpts o;
		if (!ParseCommon(argc, argv, 3, o))
			return 1;
		return CmdBatteryGen(argv[2], o);
	}
	if (cmd == "funcgen" && argc >= 3) {
		ReplayOpts o;
		if (!ParseCommon(argc, argv, 3, o))
			return 1;
		const int n = o.fuzz_n > 0 ? o.fuzz_n : 20000;
		const unsigned seed = o.rng ? o.rng : 20260815u;
		char documents[MAX_PATH];
		std::string outp = "func_probes.csv";
		if (SUCCEEDED(SHGetFolderPathA(nullptr, CSIDL_PERSONAL, nullptr,
			SHGFP_TYPE_CURRENT, documents)))
			outp = std::string(documents) + "\\sourceTAS\\solver\\func_probes.csv";
		return CmdFuncGen(argv[2], o, n, seed, outp);
	}
	if (cmd == "funcdiff" && argc >= 5) {
		ReplayOpts o;
		if (!ParseCommon(argc, argv, 5, o))
			return 1;
		return CmdFuncDiff(argv[2], o, argv[3], argv[4]);
	}
	if (cmd == "soliddiff" && argc >= 5) {
		ReplayOpts o;
		if (!ParseCommon(argc, argv, 5, o))
			return 1;
		return CmdSolidDiff(argv[2], o, argv[3], argv[4]);
	}
	if (cmd == "fuzzgen" && argc >= 3) {
		ReplayOpts o;
		if (!ParseCommon(argc, argv, 3, o))
			return 1;
		const int n = o.fuzz_n > 0 ? o.fuzz_n : 4000;
		const unsigned seed = o.rng ? o.rng : 20260815u;
		char documents[MAX_PATH];
		std::string outp = "fuzz_probes.csv";
		if (SUCCEEDED(SHGetFolderPathA(nullptr, CSIDL_PERSONAL, nullptr,
			SHGFP_TYPE_CURRENT, documents)))
			outp = std::string(documents) + "\\sourceTAS\\solver\\fuzz_probes.csv";
		return CmdFuzzGen(argv[2], o, n, seed, outp);
	}
	if (cmd == "fuzzdiff" && argc >= 5) {
		ReplayOpts o;
		if (!ParseCommon(argc, argv, 5, o))
			return 1;
		return CmdFuzzDiff(argv[2], o, argv[3], argv[4]);
	}
	if (cmd == "traceone" && argc >= 10) {
		ReplayOpts o;
		if (!ParseCommon(argc, argv, 10, o))
			return 1;
		const Vec3 a(static_cast<float>(atof(argv[3])),
			static_cast<float>(atof(argv[4])),
			static_cast<float>(atof(argv[5])));
		const Vec3 b(static_cast<float>(atof(argv[6])),
			static_cast<float>(atof(argv[7])),
			static_cast<float>(atof(argv[8])));
		return CmdTraceOne(argv[2], o, a, b, atoi(argv[9]));
	}
	if (cmd == "envelope" && argc >= 4) {
		ReplayOpts o;
		std::vector<std::string> tapes;
		int i = 3;
		for (; i < argc && argv[i][0] != '-'; ++i)
			tapes.push_back(argv[i]);
		if (!ParseCommon(argc, argv, i, o))
			return 1;
		return CmdEnvelope(argv[2], o, tapes);
	}
	if (cmd == "facecover" && argc >= 4) {
		ReplayOpts o;
		std::vector<std::string> tapes;
		int i = 3;
		for (; i < argc && argv[i][0] != '-'; ++i)
			tapes.push_back(argv[i]);
		if (!ParseCommon(argc, argv, i, o))
			return 1;
		return CmdFaceCover(argv[2], o, tapes);
	}
	if (cmd == "boardwin" && argc >= 4) {
		ReplayOpts o;
		std::vector<std::string> tapes;
		int i = 3;
		for (; i < argc && argv[i][0] != '-'; ++i)
			tapes.push_back(argv[i]);
		if (!ParseCommon(argc, argv, i, o))
			return 1;
		return CmdBoardWin(argv[2], o, tapes);
	}
	if (cmd == "airsolve" && argc >= 4) {
		ReplayOpts o;
		std::vector<std::string> tapes;
		int i = 3;
		for (; i < argc && argv[i][0] != '-'; ++i)
			tapes.push_back(argv[i]);
		if (!ParseCommon(argc, argv, i, o))
			return 1;
		return CmdAirSolve(argv[2], o, tapes);
	}
	if (cmd == "ledger" && argc >= 4) {
		ReplayOpts o;
		int sab0 = -1, sab1 = -1;
		int i = 4;
		if (argc >= 6 && argv[4][0] != '-') {
			sab0 = atoi(argv[4]);
			sab1 = atoi(argv[5]);
			i = 6;
		}
		if (!ParseCommon(argc, argv, i, o))
			return 1;
		return CmdLedger(argv[2], o, argv[3], sab0, sab1);
	}
	if (cmd == "ledgergate" && argc >= 4) {
		ReplayOpts o;
		if (!ParseCommon(argc, argv, 4, o))
			return 1;
		return CmdLedgerGate(argv[2], o, argv[3]);
	}
	if (cmd == "ledger-trace" && argc >= 3) {
		ReplayOpts o;
		if (!ParseCommon(argc, argv, 3, o))
			return 1;
		return CmdLedgerTrace(argv[2], o);
	}
	if (cmd == "ridedump" && argc >= 6) {
		ReplayOpts o;
		if (!ParseCommon(argc, argv, 6, o))
			return 1;
		return CmdRideDump(argv[2], o, argv[3], atoi(argv[4]),
			atoi(argv[5]));
	}
	if (cmd == "carve" && argc >= 4) {
		ReplayOpts o;
		std::vector<std::string> tapes;
		int i = 3;
		for (; i < argc && argv[i][0] != '-'; ++i)
			tapes.push_back(argv[i]);
		if (!ParseCommon(argc, argv, i, o))
			return 1;
		return CmdCarve(argv[2], o, tapes);
	}
	if (cmd == "strafelaw" && argc >= 3) {
		ReplayOpts o;
		if (!ParseCommon(argc, argv, 3, o))
			return 1;
		return CmdStrafeLaw(argv[2], o);
	}
	if (cmd == "routegraph" && argc >= 3) {
		const bool has_out = argc >= 4 && argv[3][0] != '-';
		ReplayOpts o;
		if (!ParseCommon(argc, argv, has_out ? 4 : 3, o))
			return 1;
		return CmdRouteGraph(argv[2], o,
			has_out ? argv[3] : "route_features.csv");
	}
	if (cmd == "tracegen-quads" && argc >= 5) {
		ReplayOpts o;
		if (!ParseCommon(argc, argv, 5, o))
			return 1;
		return CmdTraceGenQuads(argv[2], o, argv[3], argv[4]);
	}
	if (cmd == "quaddiff" && argc >= 5) {
		ReplayOpts o;
		if (!ParseCommon(argc, argv, 5, o))
			return 1;
		return CmdQuadDiff(argv[2], o, argv[3], argv[4]);
	}
	if (cmd == "tracegen-map" && argc >= 3) {
		ReplayOpts o;
		if (!ParseCommon(argc, argv, 3, o))
			return 1;
		char documents[MAX_PATH];
		std::string outp = "trace_queries.csv";
		if (SUCCEEDED(SHGetFolderPathA(nullptr, CSIDL_PERSONAL, nullptr,
			SHGFP_TYPE_CURRENT, documents)))
			outp = std::string(documents)
				+ "\\sourceTAS\\solver\\trace_queries.csv";
		return CmdTraceGenMap(argv[2], o, outp);
	}
	if (cmd == "battery-slice" && argc >= 7) {
		ReplayOpts o;
		if (!ParseCommon(argc, argv, 7, o))
			return 1;
		return CmdBatterySlice(argv[2], argv[3], atoi(argv[4]),
			atoi(argv[5]), argv[6], o);
	}
	if (cmd == "battery" && argc >= 3) {
		ReplayOpts o;
		if (!ParseCommon(argc, argv, 3, o))
			return 1;
		return CmdBattery(argv[2], o);
	}
	if (cmd == "bench" && argc >= 3) {
		const long long ticks = (argc >= 4) ? atoll(argv[3]) : 2000000LL;
		return CmdBench(argv[2], ticks);
	}
	if (cmd == "tracediff" && argc >= 4)
		return CmdTraceDiff(argv[2], argv[3]);
	if (cmd == "traceself" && argc >= 5) {
		ReplayOpts o;
		if (!ParseCommon(argc, argv, 5, o))
			return 1;
		return CmdTraceSelf(argv[2], argv[3], argv[4], o);
	}
	PrintUsage();
	return 1;
}


