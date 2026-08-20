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
#include "SolverRouteSearch.h"
#include "SolverAssemble.h"
#include "SolverPath.h"
#include "SolverPlan.h"
#include "SolverField.h"
#include "SolverEntrance.h"
#include "SolverRide.h"
#include "SolverExitField.h"
#include "SolverParams.h"
#include "SolverSearchLog.h"
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
		printf("  tapeinfo [pattern]   recordings inventory: map/frames/anchor\n");
		printf("          per tape (provenance check - which map is it FOR?)\n");
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

	// End zone id from MAP DATA first (the red-texture convention -
	// the path that generalizes to every marked map); fall back to
	// replaying the reference tape to its finish on unmarked maps
	// (zones are plugin-side on real servers, so a human trace is the
	// honest fallback source). Prints what it decided under `tag`.
	int DetectEndZone(const World& w, const MoveParams& p,
	                  const Tape& at, const char* tag) {
		std::vector<int> greens, reds;
		w.FindZoneBrushes(&greens, &reds);
		if (!reds.empty()) {
			printf("%s: end zone from texture marks: red brush", tag);
			for (int rid : reds)
				printf(" %d", rid);
			if (!greens.empty()) {
				printf(" | start marks (green):");
				for (int gid : greens)
					printf(" %d", gid);
			}
			printf("\n");
			return reds[0];
		}
		if (!at.start.valid || at.frames.empty())
			return -1;
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
			&& tr.normal.Z >= p.walkable_z) {
			s.pos.Z -= 2.f * gf;
			s.on_ground = true;
			s.ground_brush = tr.brush;
		}
		for (size_t t = 0; t < at.frames.size(); ++t) {
			const TapeFrame& fr = at.frames[t];
			TickEvents ev;
			MoveTick(s, w, p, fr.pitch, fr.yaw, fr.fmove,
				fr.smove, fr.umove, fr.buttons, &ev);
		}
		const int end_id = w.BrushUnder(s.pos, s.ducked);
		printf("%s: no texture zone marks - end derived from the "
			"reference finish: brush %d at (%.0f,%.0f,%.0f)\n", tag,
			end_id, s.pos.X, s.pos.Y, s.pos.Z);
		return end_id;
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
				// Distinct side textures: zones are identified by texture
				// (the red end zone), so the readout must show them.
				std::string tex;
				for (int s = 0; s < static_cast<int>(b.texd.size()); ++s) {
					const char* nm = w.SideTexName(b, s);
					if (!nm[0])
						continue;
					if (tex.find(nm) == std::string::npos) {
						if (!tex.empty())
							tex += ",";
						tex += nm;
					}
				}
				printf("brush %3d contents=0x%x planes=%d (sides %d) "
					"aabb (%.0f,%.0f,%.0f)..(%.0f,%.0f,%.0f) tex[%s]\n",
					b.id, b.contents, static_cast<int>(b.n.size()), b.nsides,
					b.bmin.X, b.bmin.Y, b.bmin.Z, b.bmax.X, b.bmax.Y, b.bmax.Z,
					tex.c_str());
			}
		}
		if (!w.texnames.empty()) {
			printf("texdata (%d):\n", static_cast<int>(w.texnames.size()));
			for (size_t i = 0; i < w.texnames.size(); ++i)
				printf("  [%2d] %-40s reflect (%.3f %.3f %.3f)\n",
					static_cast<int>(i), w.texnames[i].c_str(),
					w.texreflect[i].X, w.texreflect[i].Y, w.texreflect[i].Z);
		}
		ReportLeafProbes(w);
		return 0;
	}

	struct ReplayOpts {
		int start_brush = -1;      // BSP id; -1 = auto (brush under anchor)
		int end_brush = -1;        // BSP id; -1 = none (no finish check)
		std::string viz;           // search-space report path (--viz)
		std::string csv;
		// efield / fieldexact (the entrance-field laboratory)
		bool  ef_dwellcost = false;   // also build the no-flip-gap variant
		bool  ef_witnesscheck = false; // replay every stored witness
		int   ef_cells = 6;           // fieldexact: sampled cells per event
		int   ef_event = -1;          // ladder: which flight event (-1 = all)
		float ef_grid = 32.f;
		int   rec_m = -1;             // airrec: single machinery level
		                              // (-1 = sweep m0/m1/m2 for the curve)
		int   rec_budget = 1000;      // airrec: flat per-case eval budget
		int   suite_budget = 600;     // airsuite: per-case eval budget
		int   suite_sched = -1;       // airsuite: -1 = RefTune default
		int   suite_cover = -1;       // airsuite: coverage rollout depth
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
			else if (a == "--viz") { if (i + 1 < argc) o.viz = argv[++i]; else ok = false; }
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
			else if (a == "--dwellcost") o.ef_dwellcost = true;
			else if (a == "--witnesscheck") o.ef_witnesscheck = true;
			else if (a == "--cells") ok = next_i(&o.ef_cells);
			else if (a == "--event") ok = next_i(&o.ef_event);
			else if (a == "--efgrid") ok = next_f(&o.ef_grid);
			else if (a == "--m") ok = next_i(&o.rec_m);
			else if (a == "--budget") ok = next_i(&o.rec_budget);
			else if (a == "--suite-budget") ok = next_i(&o.suite_budget);
			else if (a == "--sched") ok = next_i(&o.suite_sched);
			else if (a == "--cover-top") ok = next_i(&o.suite_cover);
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
			const float ana2 = law.NewSpeed2(Strafe::TrueWishCos(cosf(alpha)));
			const float anaT = law.TurnRad(Strafe::TrueWishCos(cosf(alpha)), sinf(alpha));
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
		// ONE AUTHORITATIVE TOLERANCE (advisor 2026-08-19): the printed
		// verdict and the process exit code MUST derive from the same
		// thresholds. A gate that prints EXACT while returning failure is
		// not cosmetic once audit automation depends on it.
		constexpr float kLawMaxDG = 4.f;    // ULP-relative: one ULP of
		constexpr float kLawMaxDT = 1e-5f;  // speed^2 at 3400 u/s is ~1.0
		const bool law_ok = (max_dg <= kLawMaxDG && max_dt < kLawMaxDT);
		printf("strafelaw: %s\n", law_ok
			? "closed form EXACT against the certified mirror (float ULP)"
			: "LAW DEVIATES - do not build on it; find out why first");
		// The economics table for the record: per-tick gain vs turn at the
		// optimum and at biases, standing, sfric 1.
		printf("  v      gain(perp)  turn(perp)deg | gain(b=15deg) turn(b=15)\n");
		for (int vi = 0; vi < 8; ++vi) {
			const float v = kV[vi];
			Strafe::TickLaw law = Strafe::Law(o.params, v, 1.f, false);
			const float g0 = sqrtf(v * v + law.OptGain2()) - v;
			const float t0 = law.TurnRad(Strafe::kPerp, 1.f) * 57.29578f;
			const float cb = cosf(Deg2Rad(75.f));   // 15 deg off perp
			const float sb = sinf(Deg2Rad(75.f));
			const float gb = sqrtf(law.NewSpeed2(Strafe::TrueWishCos(cb))) - v;
			const float tb = law.TurnRad(Strafe::TrueWishCos(cb), sb) * 57.29578f;
			printf("  %6.0f  %9.4f  %12.4f | %12.4f  %9.4f\n",
				v, g0, t0, gb, tb);
		}
		fflush(stdout);
		return law_ok ? 0 : 2;
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
			if (end_id < 0 && tape_ok)
				end_id = DetectEndZone(w, o.params, at, "routegraph");
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

	// routesgate: THE M2 ACCEPTANCE GATE - unseeded route enumeration
	// by analytic lower bounds. Top-10 must include the HUMAN shape
	// (start -> face 0 -> end: Run 21's one-ride line) and the
	// solved12 shape (0 -> 1 -> 2 -> 3), under 5 seconds.
	int CmdRoutesGate(const std::string& map_path, const ReplayOpts& o,
	                  const std::string& anchor_tas) {
		World w;
		std::string err;
		w.true_interval_corner = o.corner_true;
		if (!w.Load(map_path, o.hulls, &err, o.edge_bevels)) {
			printf("LOAD FAILED (bsp): %s\n", err.c_str());
			return 1;
		}
		Route::Graph g;
		if (!Route::Build(w, &g, 2000.f, &err)) {
			printf("routesgate: %s\n", err.c_str());
			return 1;
		}
		Tape at;
		if (!LoadTas(anchor_tas, at, &err) || !at.start.valid) {
			printf("routesgate: anchor tape: %s\n", err.c_str());
			return 1;
		}
		const int end_id = DetectEndZone(w, o.params, at, "routesgate");
		if (!Route::AnchorZones(w, &g, at.start.origin,
			at.start.ducked, end_id, 2000.f, &err)) {
			printf("routesgate: %s\n", err.c_str());
			return 1;
		}
		const clock_t c0 = clock();
		RouteSearch::Opts ro;
		ro.top_k = 60;   // raw pool; ranked by DISTINCT BASE SHAPE
		std::vector<RouteSearch::Candidate> routes;
		if (!RouteSearch::Enumerate(w, g, o.params, ro, &routes,
			&err)) {
			printf("routesgate: %s\n", err.c_str());
			return 1;
		}
		const double secs = static_cast<double>(clock() - c0)
			/ CLOCKS_PER_SEC;
		// A route's SHAPE = its faces in first-occurrence order (cycle
		// variants and multi-taps collapse: solved12's own face-3 taps
		// are one shape). Rank the best lb per distinct shape - that
		// pool is what stage 3 consumes.
		struct Shape {
			std::vector<int> base;
			float lb = 0.f;
			float end_v = 0.f;
		};
		std::vector<Shape> shapes;
		for (const RouteSearch::Candidate& r : routes) {
			Shape s;
			for (int fidx : r.faces) {
				bool seen = false;
				for (int b : s.base)
					if (b == fidx)
						seen = true;
				if (!seen)
					s.base.push_back(fidx);
			}
			s.lb = r.lb_ticks;
			s.end_v = sqrtf(r.end_s2_ub);
			bool merged = false;
			for (Shape& e : shapes)
				if (e.base == s.base) {
					if (s.lb < e.lb)
						e = s;
					merged = true;
					break;
				}
			if (!merged)
				shapes.push_back(s);
		}
		std::sort(shapes.begin(), shapes.end(),
			[](const Shape& a, const Shape& b) {
				return a.lb < b.lb;
			});
		if (shapes.size() > 10)
			shapes.resize(10);
		bool have_human = false, have_s12 = false;
		for (size_t i = 0; i < shapes.size(); ++i) {
			const Shape& s = shapes[i];
			printf("routesgate[%2d]: lb %6.0f ticks | end v<= %6.0f "
				"| shape", static_cast<int>(i), s.lb, s.end_v);
			for (int fidx : s.base)
				printf(" %d", fidx);
			printf("\n");
			if (s.base.size() == 1 && s.base[0] == 0)
				have_human = true;
			if (s.base.size() == 4 && s.base[0] == 0
				&& s.base[1] == 1 && s.base[2] == 2
				&& s.base[3] == 3)
				have_s12 = true;
		}
		printf("routesgate: %d routes -> %d shapes in %.2fs | human "
			"shape [0] %s | solved12 shape [0 1 2 3] %s\n",
			static_cast<int>(routes.size()),
			static_cast<int>(shapes.size()), secs,
			have_human ? "PRESENT" : "MISSING",
			have_s12 ? "PRESENT" : "MISSING");
		const bool pass = have_human && have_s12 && secs < 5.0;
		printf("routesgate: M2 GATE %s\n", pass ? "PASS" : "FAIL");
		fflush(stdout);
		return pass ? 0 : 2;
	}

	// msolvegate: THE M3 ACCEPTANCE GATE - unseeded finisher on the
	// exact engine in < 2 minutes whose ledger STRICTLY DOMINATES the
	// old solver's best line: fewer zone ticks, less board loss, less
	// air shortfall. Writes the assembled .tas beside the anchor.
	int CmdMSolveGate(const std::string& map_path, const ReplayOpts& o,
	                  const std::string& anchor_tas,
	                  const std::string& out_tas) {
		World w;
		std::string err;
		w.true_interval_corner = o.corner_true;
		if (!w.Load(map_path, o.hulls, &err, o.edge_bevels)) {
			printf("LOAD FAILED (bsp): %s\n", err.c_str());
			return 1;
		}
		Route::Graph g;
		if (!Route::Build(w, &g, 2000.f, &err)) {
			printf("msolvegate: %s\n", err.c_str());
			return 1;
		}
		Tape at;
		if (!LoadTas(anchor_tas, at, &err) || !at.start.valid) {
			printf("msolvegate: anchor tape: %s\n", err.c_str());
			return 1;
		}
		const int end_id = DetectEndZone(w, o.params, at, "msolvegate");
		if (!Route::AnchorZones(w, &g, at.start.origin,
			at.start.ducked, end_id, 2000.f, &err)) {
			printf("msolvegate: %s\n", err.c_str());
			return 1;
		}
		// Search-space report: ALWAYS recorded (user 2026-08-16:
		// "save the solver runs so we can look back over
		// iterations") - timestamped under Output\reports unless
		// --viz names a path.
		std::string viz_path = o.viz;
		if (viz_path.empty()) {
			CreateDirectoryA("Output", nullptr);
			CreateDirectoryA("Output\\reports", nullptr);
			time_t now = time(nullptr);
			struct tm tmv;
			localtime_s(&tmv, &now);
			char st[64];
			strftime(st, sizeof(st), "%m%d-%H%M%S", &tmv);
			viz_path = "Output\\reports\\msolve_" + at.map + "_" + st
				+ ".html";
		}
		SearchLog::Sink viz_sink;
		SearchLog::g_sink = &viz_sink;
		// Reference replays for the report overlay (+ speeds for the
		// energy readout).
		auto CollectRef = [&](const TapeAnchor& anchor,
			const std::vector<TapeFrame>& frames,
			std::vector<Vec3>* pts, std::vector<float>* spds) {
			PlayerState s;
			s.pos = anchor.origin;
			s.vel = anchor.velocity;
			s.ducked = anchor.ducked;
			s.hull_state = anchor.ducked ? 1 : 0;
			s.stamina = anchor.stamina;
			TraceResult tr;
			const float gf = w.TraceHull(s.pos,
				s.pos - Vec3(0.f, 0.f, 2.f), s.ducked, &tr);
			if (gf < 1.f && tr.brush >= 0
				&& tr.normal.Z >= o.params.walkable_z) {
				s.pos.Z -= 2.f * gf;
				s.on_ground = true;
				s.ground_brush = tr.brush;
			}
			pts->push_back(s.pos);
			if (spds) spds->push_back(Len(s.vel));
			for (size_t t = 0; t < frames.size(); ++t) {
				const TapeFrame& fr = frames[t];
				TickEvents ev;
				MoveTick(s, w, o.params, fr.pitch, fr.yaw, fr.fmove,
					fr.smove, fr.umove, fr.buttons, &ev);
				if (t % 2 == 0) {
					pts->push_back(s.pos);
					if (spds) spds->push_back(Len(s.vel));
				}
			}
		};
		const clock_t c0 = clock();
		Assemble::Opts ao;
		Assemble::RunResult rr, partial;
		const bool solved = Assemble::SolveMap(w, g, o.params,
			at.start, ao, &rr, &err, &partial);
		{
			SearchLog::g_sink = nullptr;
			viz_sink.Flush();
			viz_sink.gravity = o.params.gravity;
			viz_sink.wish_rate = o.params.air_speed_cap
				* o.params.air_speed_cap;
			std::vector<Vec3> ref;
			std::vector<float> refs;
			CollectRef(at.start, at.frames, &ref, &refs);
			viz_sink.AddRef("REF reference tape", ref, &refs);
			if (solved) {
				std::vector<Vec3> res;
				std::vector<float> ress;
				CollectRef(at.start, rr.frames, &res, &ress);
				viz_sink.AddRef("RESULT solved line", res, &ress);
			}
			std::string verr;
			if (SearchLog::WriteHtml(viz_path, w, g, viz_sink,
				"msolvegate search space - " + at.map, &verr))
				printf("msolvegate: search report -> %s\n",
					viz_path.c_str());
			else
				printf("msolvegate: VIZ WRITE FAILED: %s\n",
					verr.c_str());
		}
		if (!solved) {
			printf("msolvegate: SOLVE FAILED: %s\n", err.c_str());
			// The deepest partial chain still exports, clearly named,
			// beside the anchor tape (playable in-game like any
			// recording; it will NOT finish - it is the best
			// assembled prefix for eyes-on review).
			if (!partial.frames.empty()) {
				std::string dir = anchor_tas;
				const size_t ds = dir.find_last_of("\\/");
				dir = ds == std::string::npos ? std::string()
					: dir.substr(0, ds + 1);
				std::string shp;
				for (int fidx : partial.shape)
					shp += (shp.empty() ? "" : "-")
						+ std::to_string(fidx);
				time_t pn = time(nullptr);
				struct tm ptm;
				localtime_s(&ptm, &pn);
				char pst[64];
				strftime(pst, sizeof(pst), "%m%d-%H%M", &ptm);
				const std::string pp = dir + at.map + "_PARTIAL_shape"
					+ shp + "_legs"
					+ std::to_string(partial.legs_done) + "_" + pst
					+ ".tas";
				std::string perr;
				if (WriteTas(pp, at.start, at.map, partial.frames,
					&perr))
					printf("msolvegate: partial chain (%d/%d legs) "
						"-> %s (%d frames)\n", partial.legs_done,
						static_cast<int>(partial.shape.size()),
						pp.c_str(),
						static_cast<int>(partial.frames.size()));
				else
					printf("msolvegate: PARTIAL WRITE FAILED: %s\n",
						perr.c_str());
			}
			return 2;
		}
		const double secs = static_cast<double>(clock() - c0)
			/ CLOCKS_PER_SEC;
		if (!out_tas.empty()) {
			if (WriteTas(out_tas, at.start, at.map, rr.frames, &err))
				printf("msolvegate: wrote %s (%d frames)\n",
					out_tas.c_str(),
					static_cast<int>(rr.frames.size()));
			else
				printf("msolvegate: WRITE FAILED: %s\n", err.c_str());
		}
		// Ledgers + zone clocks, ours vs the old line, same detector.
		Tape ours;
		ours.start = at.start;
		ours.map = at.map;
		ours.frames = rr.frames;
		Ledger::Run lo, lr;
		Ledger::Build(w, g, o.params, ours, &lo);
		Ledger::Build(w, g, o.params, at, &lr);
		auto board_sum = [](const Ledger::Run& r) {
			float s = 0.f;
			for (const Ledger::Phase& ph : r.phases)
				if (ph.kind == Ledger::Kind::Ride)
					s += ph.board_dot2;
			return s;
		};
		const int our_zone = Assemble::ZoneTick(w, g, o.params,
			at.start, rr.frames);
		const int old_zone = Assemble::ZoneTick(w, g, o.params,
			at.start, at.frames);
		const float our_board = board_sum(lo);
		const float old_board = board_sum(lr);
		printf("msolvegate: OURS shape");
		for (int fidx : rr.shape)
			printf(" %d", fidx);
		printf(" | zone %d ticks | board loss2 %.0f | shortfall %.0f "
			"| dissipation %.0f\n", our_zone, our_board,
			lo.total_shortfall, lo.total_dissipation);
		printf("msolvegate: OLD (292)   | zone %d ticks | board loss2 "
			"%.0f | shortfall %.0f | dissipation %.0f\n", old_zone,
			old_board, lr.total_shortfall, lr.total_dissipation);
		printf("msolvegate: wall %.1fs\n", secs);
		const bool pass = our_zone > 0 && old_zone > 0
			&& our_zone < old_zone && our_board < old_board
			&& lo.total_shortfall < lr.total_shortfall
			&& secs < 120.0;
		printf("msolvegate: M3 GATE %s\n", pass ? "PASS" : "FAIL");
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

	// tapprobe: ISOLATION instrument for the unified tap transfer (the
	// FUNCPROBE method at transfer granularity). Replay a tape to a
	// tick, then run the tap-mode carve solve from that EXACT state:
	// can the primitive find ride->flight->strike on the target face
	// from a known-good entry? Dumps the winner's trajectory so a
	// failure is attributable (dive? wall? wrong side?).
	int CmdTapProbe(const std::string& map_path, const ReplayOpts& o,
	                const std::string& tape_path, int at_tick,
	                int ride_face, int tap_face, int evals) {
		World w;
		std::string err;
		w.true_interval_corner = o.corner_true;
		if (!w.Load(map_path, o.hulls, &err, o.edge_bevels)) {
			printf("LOAD FAILED (bsp): %s\n", err.c_str());
			return 1;
		}
		Route::Graph g;
		if (!Route::Build(w, &g, 2000.f, &err)) {
			printf("tapprobe: %s\n", err.c_str());
			return 1;
		}
		Tape tape;
		if (!LoadTas(tape_path, tape, &err)) {
			printf("LOAD FAILED (tas): %s\n", err.c_str());
			return 1;
		}
		// tap_face -1 = ZONE MODE: the ending transfer into the red-
		// texture end zone volume.
		const bool zone_mode = tap_face < 0;
		if (ride_face < 0
			|| ride_face >= static_cast<int>(g.faces.size())
			|| (!zone_mode
				&& tap_face >= static_cast<int>(g.faces.size()))) {
			printf("tapprobe: face out of range (%d faces)\n",
				static_cast<int>(g.faces.size()));
			return 1;
		}
		int zone_idx = -1;
		if (zone_mode) {
			std::vector<int> reds;
			w.FindZoneBrushes(nullptr, &reds);
			if (reds.empty()) {
				printf("tapprobe: zone mode needs a red-texture end "
					"zone\n");
				return 1;
			}
			zone_idx = w.IndexOfBrushId(reds[0]);
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
		for (int t = 0; t < at_tick
			&& t < static_cast<int>(tape.frames.size()); ++t) {
			const TapeFrame& f = tape.frames[t];
			TickEvents ev;
			MoveTick(s, w, o.params, f.pitch, f.yaw, f.fmove, f.smove,
				f.umove, f.buttons, &ev);
		}
		Carve::Target ct;
		ct.face = ride_face;
		ct.max_ticks = 320;
		Vec3 aim_at;
		if (zone_mode) {
			const WorldBrush& zb = w.brushes[zone_idx];
			ct.to_zone = true;
			Assemble::ZoneVolume(zb, &ct.zone_min, &ct.zone_max);
			// Nearest rim, not the sprawling platform's center.
			aim_at = s.pos;
			if (aim_at.X < ct.zone_min.X) aim_at.X = ct.zone_min.X;
			if (aim_at.X > ct.zone_max.X) aim_at.X = ct.zone_max.X;
			if (aim_at.Y < ct.zone_min.Y) aim_at.Y = ct.zone_min.Y;
			if (aim_at.Y > ct.zone_max.Y) aim_at.Y = ct.zone_max.Y;
			aim_at.Z = ct.zone_min.Z;
			printf("tapprobe: entry t%d pos(%.1f,%.1f,%.1f) v(%.1f,"
				"%.1f,%.1f) s2d %.1f | ride face %d (brush %d) -> "
				"ZONE brush %d vol (%.0f,%.0f,%.0f)..(%.0f,%.0f,"
				"%.0f)\n", at_tick, s.pos.X, s.pos.Y, s.pos.Z,
				s.vel.X, s.vel.Y, s.vel.Z, Len2D(s.vel), ride_face,
				g.faces[ride_face].brush, zb.id, ct.zone_min.X,
				ct.zone_min.Y, ct.zone_min.Z, ct.zone_max.X,
				ct.zone_max.Y, ct.zone_max.Z);
		} else {
			const Route::Face& nf = g.faces[tap_face];
			ct.tap_brush = nf.brush;
			ct.tap_side = nf.side;
			ct.tap_face = tap_face;
			aim_at = nf.centroid;
			printf("tapprobe: entry t%d pos(%.1f,%.1f,%.1f) v(%.1f,"
				"%.1f,%.1f) s2d %.1f | ride face %d (brush %d) -> "
				"tap face %d (brush %d side %d, centroid %.0f,%.0f,"
				"%.0f)\n", at_tick, s.pos.X, s.pos.Y, s.pos.Z,
				s.vel.X, s.vel.Y, s.vel.Z, Len2D(s.vel), ride_face,
				g.faces[ride_face].brush, tap_face, nf.brush, nf.side,
				nf.centroid.X, nf.centroid.Y, nf.centroid.Z);
		}
		ct.exit_heading = atan2f(aim_at.Y - s.pos.Y,
			aim_at.X - s.pos.X);
		const float s_est = Len2D(s.vel);
		const float est = s_est > 100.f
			? Len(aim_at - s.pos) / (s_est * o.params.dt) : 80.f;
		ct.aim_tick = est < 10.f ? 10
			: (est > 240.f ? 240 : static_cast<int>(est));
		ct.tick_w = 0.02f;
		SearchLog::Sink viz_sink(1200, 3);
		if (!o.viz.empty()) {
			SearchLog::g_sink = &viz_sink;
			viz_sink.BeginStage(zone_mode ? "zone solve"
				: "tap solve");
		}
		Carve::Result cr = Carve::SolveCarve(s, w, o.params, g, ct, 6,
			evals);
		SearchLog::g_sink = nullptr;
		const bool tap_hit = cr.zoned
			|| (!zone_mode && !cr.exited
				&& cr.struck_brush == ct.tap_brush
				&& cr.struck_plane == ct.tap_side);
		printf("tapprobe: %s | tick %d | strike dot %.1f | end (%.1f,"
			"%.1f,%.1f) v(%.1f,%.1f,%.1f) s2d %.1f | miss %.1f | "
			"reach_short %.1f | grounded %d struck %d\n",
			cr.zoned ? "ZONE ENTRY" : (tap_hit ? "TAP STRIKE"
				: (cr.exited ? "exited(?)" : "NO STRIKE")), cr.tick,
			cr.strike_dot, cr.end_pos.X,
			cr.end_pos.Y, cr.end_pos.Z, cr.end_state.vel.X,
			cr.end_state.vel.Y, cr.end_state.vel.Z,
			Len2D(cr.end_state.vel), cr.miss_dist, cr.reach_short,
			cr.grounded ? 1 : 0, cr.struck_brush);
		// The winner's trajectory, tick by tick (replayed controls).
		std::vector<Vec3> win_pts;
		PlayerState rs = s;
		for (size_t k = 0; k < cr.yaw.size(); ++k) {
			TickEvents ev;
			MoveTick(rs, w, o.params, 0.f, cr.yaw[k], cr.fmove[k],
				cr.smove[k], 0.f,
				(s.ducked ? IN_DUCK : 0)
					| (cr.duck_at >= 0
						&& static_cast<int>(k) >= cr.duck_at
						? IN_DUCK : 0), &ev);
			int cb = -1;
			float dot = 0.f;
			if (ev.ncontacts > 0) {
				cb = w.brushes[ev.contact_brush[0]].id;
				dot = Dot(ev.contact_vel[0],
					w.brushes[ev.contact_brush[0]]
						.n[ev.contact_plane[0]]);
			}
			win_pts.push_back(rs.pos);
			printf("  k%4d pos(%7.1f,%7.1f,%6.1f) v(%6.1f,%6.1f,%6.1f)"
				" s2d %5.1f | c%d dot %6.1f%s\n",
				static_cast<int>(k), rs.pos.X, rs.pos.Y, rs.pos.Z,
				rs.vel.X, rs.vel.Y, rs.vel.Z, Len2D(rs.vel), cb, dot,
				rs.on_ground ? " GROUND" : "");
			if (static_cast<int>(k) + 1 >= cr.tick && cr.tick > 0)
				break;
		}
		if (!o.viz.empty()) {
			viz_sink.AddRef("RESULT winner", win_pts);
			std::string verr;
			char ttl[128];
			snprintf(ttl, sizeof(ttl), "tapprobe t%d face %d -> %s",
				at_tick, ride_face,
				zone_mode ? "ZONE" : "tap");
			if (SearchLog::WriteHtml(o.viz, w, g, viz_sink, ttl,
				&verr))
				printf("tapprobe: search report -> %s\n",
					o.viz.c_str());
			else
				printf("tapprobe: VIZ WRITE FAILED: %s\n",
					verr.c_str());
		}
		fflush(stdout);
		return tap_hit ? 0 : 2;
	}

	// stateprobe: the tap/zone transfer solve from an EXPLICIT state
	// (position + velocity) - the fast iteration loop for mid-chain
	// entries the tape probes cannot reach. tap_face -1 = zone mode.
	int CmdStateProbe(const std::string& map_path, const ReplayOpts& o,
	                  const Vec3& pos, const Vec3& vel, int ride_face,
	                  int tap_face, int evals) {
		World w;
		std::string err;
		w.true_interval_corner = o.corner_true;
		if (!w.Load(map_path, o.hulls, &err, o.edge_bevels)) {
			printf("LOAD FAILED (bsp): %s\n", err.c_str());
			return 1;
		}
		Route::Graph g;
		if (!Route::Build(w, &g, 2000.f, &err)) {
			printf("stateprobe: %s\n", err.c_str());
			return 1;
		}
		const bool zone_mode = tap_face < 0;
		if (ride_face < 0
			|| ride_face >= static_cast<int>(g.faces.size())
			|| (!zone_mode
				&& tap_face >= static_cast<int>(g.faces.size()))) {
			printf("stateprobe: face out of range\n");
			return 1;
		}
		int zone_idx = -1;
		if (zone_mode) {
			std::vector<int> reds;
			w.FindZoneBrushes(nullptr, &reds);
			if (reds.empty()) {
				printf("stateprobe: no red end zone\n");
				return 1;
			}
			zone_idx = w.IndexOfBrushId(reds[0]);
		}
		PlayerState s;
		s.pos = pos;
		s.vel = vel;
		Carve::Target ct;
		ct.face = ride_face;
		ct.max_ticks = 320;
		Vec3 aim_at;
		if (zone_mode) {
			const WorldBrush& zb = w.brushes[zone_idx];
			ct.to_zone = true;
			Assemble::ZoneVolume(zb, &ct.zone_min, &ct.zone_max);
			aim_at = s.pos;
			if (aim_at.X < ct.zone_min.X) aim_at.X = ct.zone_min.X;
			if (aim_at.X > ct.zone_max.X) aim_at.X = ct.zone_max.X;
			if (aim_at.Y < ct.zone_min.Y) aim_at.Y = ct.zone_min.Y;
			if (aim_at.Y > ct.zone_max.Y) aim_at.Y = ct.zone_max.Y;
			aim_at.Z = ct.zone_min.Z;
			// The ending's spline domain spans the runway path.
			const Route::Face& rf = g.faces[ride_face];
			float far_d = 0.f;
			Vec3 farv = rf.centroid;
			for (const Vec3& v : rf.verts) {
				const float dx = v.X - s.pos.X;
				const float dy = v.Y - s.pos.Y;
				const float d = sqrtf(dx * dx + dy * dy);
				if (d > far_d) {
					far_d = d;
					farv = v;
				}
			}
			Vec3 rim2 = farv;
			if (rim2.X < ct.zone_min.X) rim2.X = ct.zone_min.X;
			if (rim2.X > ct.zone_max.X) rim2.X = ct.zone_max.X;
			if (rim2.Y < ct.zone_min.Y) rim2.Y = ct.zone_min.Y;
			if (rim2.Y > ct.zone_max.Y) rim2.Y = ct.zone_max.Y;
			const float fdx = rim2.X - farv.X;
			const float fdy = rim2.Y - farv.Y;
			const float fly_d = sqrtf(fdx * fdx + fdy * fdy);
			// Crest-decelerated domain estimate (see assembler).
			float zmax_f = rf.centroid.Z;
			for (const Vec3& v : rf.verts)
				if (v.Z > zmax_f)
					zmax_f = v.Z;
			const float s2d0 = Len2D(s.vel);
			const float dzc = zmax_f - s.pos.Z;
			const float sc2 = s2d0 * s2d0
				- 2.f * o.params.gravity * (dzc > 0.f ? dzc : 0.f);
			const float s_crest = sc2 > 10000.f ? sqrtf(sc2) : 100.f;
			const float ride_t = far_d / (0.5f * (s2d0 + s_crest));
			const float fly_t = fly_d / s_crest;
			const float ez = (ride_t + fly_t) / o.params.dt;
			ct.aim_tick = ez < 10.f ? 10
				: (ez > 300.f ? 300 : static_cast<int>(ez));
		} else {
			const Route::Face& nf = g.faces[tap_face];
			ct.tap_brush = nf.brush;
			ct.tap_side = nf.side;
			ct.tap_face = tap_face;
			aim_at = nf.centroid;
			const float s2d0 = Len2D(s.vel);
			const float ez = s2d0 > 100.f
				? Len(aim_at - s.pos) / (s2d0 * o.params.dt) : 80.f;
			ct.aim_tick = ez < 10.f ? 10
				: (ez > 240.f ? 240 : static_cast<int>(ez));
		}
		ct.exit_heading = atan2f(aim_at.Y - s.pos.Y,
			aim_at.X - s.pos.X);
		ct.tick_w = 0.02f;
		printf("stateprobe: entry (%.0f,%.0f,%.0f) v(%.0f,%.0f,%.0f) "
			"s2d %.0f | ride face %d -> %s | aim_tick %d\n",
			s.pos.X, s.pos.Y, s.pos.Z, s.vel.X, s.vel.Y, s.vel.Z,
			Len2D(s.vel), ride_face,
			zone_mode ? "ZONE" : "tap", ct.aim_tick);
		Carve::Result cr = Carve::SolveCarve(s, w, o.params, g, ct, 6,
			evals);
		const bool ok = cr.zoned || (!zone_mode && !cr.exited
			&& cr.struck_brush == ct.tap_brush
			&& cr.struck_plane == ct.tap_side);
		printf("stateprobe: %s | tick %d | dot %.1f | end (%.0f,%.0f,"
			"%.0f) v(%.0f,%.0f,%.0f) | miss %.1f reach %.1f | "
			"grounded %d struck %d | family %d\n",
			cr.zoned ? "ZONE ENTRY" : (ok ? "TAP STRIKE"
				: "FAIL"), cr.tick, cr.strike_dot, cr.end_pos.X,
			cr.end_pos.Y, cr.end_pos.Z, cr.end_state.vel.X,
			cr.end_state.vel.Y, cr.end_state.vel.Z, cr.miss_dist,
			cr.reach_short, cr.grounded ? 1 : 0, cr.struck_brush,
			cr.family);
		// The winner, tick by tick (every 4th + endings).
		PlayerState rs = s;
		const int nfr = static_cast<int>(cr.yaw.size());
		for (int k = 0; k < nfr; ++k) {
			TickEvents ev;
			MoveTick(rs, w, o.params, 0.f, cr.yaw[k], cr.fmove[k],
				cr.smove[k], 0.f,
				(s.ducked ? IN_DUCK : 0)
					| (cr.duck_at >= 0 && k >= cr.duck_at
						? IN_DUCK : 0), &ev);
			int cb = -1;
			float dot = 0.f;
			if (ev.ncontacts > 0) {
				cb = w.brushes[ev.contact_brush[0]].id;
				dot = Dot(ev.contact_vel[0],
					w.brushes[ev.contact_brush[0]]
						.n[ev.contact_plane[0]]);
			}
			if (k % 4 == 0 || ev.ncontacts > 0 || rs.on_ground
				|| k + 1 == nfr)
				printf("  k%4d pos(%7.1f,%7.1f,%6.1f) v(%6.1f,"
					"%6.1f,%6.1f) s2d %5.1f | c%d dot %6.1f%s\n",
					k, rs.pos.X, rs.pos.Y, rs.pos.Z, rs.vel.X,
					rs.vel.Y, rs.vel.Z, Len2D(rs.vel), cb, dot,
					rs.on_ground ? " GROUND" : "");
			if (rs.on_ground)
				break;
		}
		fflush(stdout);
		return ok ? 0 : 2;
	}

	// fieldgate: validate THE HOTSPOT FIELD against real runs (tapes
	// are VALIDATION ONLY, never seeding). For every transfer in each
	// tape (airborne stretch >= 8 ticks ending on a graph surf face),
	// compute the field from the separation state and rank the ACTUAL
	// strike point inside it. A correct field lights up where a good
	// player actually boards and stays cold where the old solver
	// slammed. Writes a heat-overlay report beside the run reports.
	int CmdFieldGate(const std::string& map_path, const ReplayOpts& o,
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
			printf("fieldgate: %s\n", err.c_str());
			return 1;
		}
		SearchLog::Sink sink;
		int ev_total = 0, hot = 0, warm = 0, cold = 0;
		for (const std::string& tp : tapes) {
			Tape tape;
			if (!LoadTas(tp, tape, &err)) {
				printf("fieldgate: %s: %s\n", tp.c_str(), err.c_str());
				continue;
			}
			std::string tname = tp;
			const size_t sl = tname.find_last_of("\\/");
			if (sl != std::string::npos)
				tname = tname.substr(sl + 1);
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
			std::vector<Vec3> refpts;
			refpts.push_back(s.pos);
			// PASS 1: replay, collect transfer events (the leg-after
			// context needs the NEXT event, so process afterward).
			struct FEvent {
				int tick = 0;
				int fidx = -1;
				PlayerState sep;
				Vec3 cp, v1;
			};
			std::vector<FEvent> events;
			int air = 0;
			PlayerState sep;
			for (size_t t = 0; t < tape.frames.size(); ++t) {
				const TapeFrame& fr = tape.frames[t];
				TickEvents ev;
				MoveTick(s, w, o.params, fr.pitch, fr.yaw, fr.fmove,
					fr.smove, fr.umove, fr.buttons, &ev);
				if (t % 2 == 0)
					refpts.push_back(s.pos);
				if (ev.ncontacts > 0) {
					int fidx = -1;
					for (size_t fi = 0; fi < g.faces.size(); ++fi)
						if (g.faces[fi].brush == ev.contact_brush[0]
							&& g.faces[fi].side
								== ev.contact_plane[0])
							fidx = static_cast<int>(fi);
					if (fidx >= 0 && air >= 8) {
						FEvent fe;
						fe.tick = static_cast<int>(t);
						fe.fidx = fidx;
						fe.sep = sep;
						fe.cp = ev.contact_pos[0];
						fe.v1 = ev.contact_vel[0];
						events.push_back(fe);
					}
					air = 0;
				} else if (!s.on_ground) {
					if (air == 0)
						sep = s;
					air++;
				} else {
					air = 0;
				}
			}
			// PASS 2: field + runway per event, leg-after context
			// from the following event (or the end zone).
			std::vector<int> zreds;
			w.FindZoneBrushes(nullptr, &zreds);
			for (size_t ei = 0; ei < events.size(); ++ei) {
				const FEvent& fe = events[ei];
				const int fidx = fe.fidx;
				const PlayerState& fsep = fe.sep;
				{
						const Route::Face& fc = g.faces[fidx];
						Field::NextCtx nctx;
						if (ei + 1 < events.size()) {
							nctx.has = true;
							nctx.is_zone = false;
							nctx.pt = g.faces[events[ei + 1].fidx]
								.centroid;
						} else if (!zreds.empty()) {
							const int zi =
								w.IndexOfBrushId(zreds[0]);
							if (zi >= 0) {
								nctx.has = true;
								nctx.is_zone = true;
								Assemble::ZoneVolume(w.brushes[zi],
									&nctx.zmin, &nctx.zmax);
							}
						}
						Field::FaceMap fm = Field::Compute(fsep.pos,
							fsep.vel, fc, o.params, fsep.ducked,
							24.f, 300, 1.f, &nctx);
						const Vec3& cp = fe.cp;
						const float adot = Dot(fe.v1, fc.n);
						std::string klass = "NO-MAP";
						float ratio = -1.f;
						if (fm.best >= 0
							&& !fm.samples.empty()) {
							const Vec3 dq = cp - fm.origin;
							int iu = static_cast<int>(
								Dot(dq, fm.ud) / fm.du + 0.5f);
							int iv = static_cast<int>(
								Dot(dq, fm.vd) / fm.dv + 0.5f);
							if (iu < 0) iu = 0;
							if (iu >= fm.nu) iu = fm.nu - 1;
							if (iv < 0) iv = 0;
							if (iv >= fm.nv) iv = fm.nv - 1;
							const Field::Sample& hs =
								fm.samples[static_cast<size_t>(
									iv) * fm.nu + iu];
							const Field::Sample& bs =
								fm.samples[fm.best];
							if (hs.reachable && bs.e_eff > 0.f) {
								ratio = hs.e_eff / bs.e_eff;
								klass = ratio >= 0.85f ? "HOT"
									: (ratio >= 0.6f ? "WARM"
										: "COLD");
							} else {
								klass = "UNREACHABLE";
							}
							if (klass == "HOT") hot++;
							else if (klass == "WARM") warm++;
							else cold++;
							printf("fieldgate: %s t%4d face %d | "
								"strike (%.0f,%.0f,%.0f) dot %.1f "
								"| ratio %.2f freeturn %d | RUNWAY "
								"have %.0f need %.0f %s | best e "
								"%.0f @(%.0f,%.0f,%.0f) res %.1f "
								"rwy %.0f/%.0f | %s\n",
								tname.c_str(), fe.tick, fidx,
								cp.X, cp.Y, cp.Z, adot, ratio,
								hs.free_turn ? 1 : 0,
								hs.run_avail, hs.run_req,
								hs.run_viable ? "OK" : "SHORT",
								sqrtf(bs.e_eff > 0.f
									? bs.e_eff : 0.f),
								bs.q.X, bs.q.Y, bs.q.Z,
								bs.residual, bs.run_avail,
								bs.run_req, klass.c_str());
							// Heat overlay for the first few maps.
							if (ev_total < 8) {
								for (int qv = 0; qv < fm.nv; ++qv)
								for (int qu = 0; qu < fm.nu;
									++qu) {
									const Field::Sample& sm =
										fm.samples[
										static_cast<size_t>(qv)
										* fm.nu + qu];
									if (!sm.reachable)
										continue;
									float v01 = fm.e_hi > fm.e_lo
										? (sm.e_eff - fm.e_lo)
											/ (fm.e_hi - fm.e_lo)
										: 1.f;
									if (!sm.free_turn)
										v01 *= 0.4f;
									// Runway-short landings render
									// dim: energy without room.
									if (!sm.run_viable)
										v01 *= 0.35f;
									const Vec3 lift =
										Scale(fc.n, 2.f);
									const Vec3 c00 = sm.q + lift
										- Scale(fm.ud,
											fm.du * 0.5f)
										- Scale(fm.vd,
											fm.dv * 0.5f);
									const Vec3 c10 = c00
										+ Scale(fm.ud, fm.du);
									const Vec3 c01 = c00
										+ Scale(fm.vd, fm.dv);
									const Vec3 c11 = c10
										+ Scale(fm.vd, fm.dv);
									sink.AddHeat(c00, c10, c11,
										v01);
									sink.AddHeat(c00, c11, c01,
										v01);
								}
								// Strike marker: a small cross.
								std::vector<Vec3> cross;
								cross.push_back(cp
									+ Vec3(-14.f, 0.f, 0.f));
								cross.push_back(cp
									+ Vec3(14.f, 0.f, 0.f));
								cross.push_back(cp);
								cross.push_back(cp
									+ Vec3(0.f, -14.f, 0.f));
								cross.push_back(cp
									+ Vec3(0.f, 14.f, 0.f));
								cross.push_back(cp);
								cross.push_back(cp
									+ Vec3(0.f, 0.f, -14.f));
								cross.push_back(cp
									+ Vec3(0.f, 0.f, 14.f));
								sink.AddRef("strike markers",
									cross);
							}
							ev_total++;
						}
				}
			}
			sink.AddRef("REF " + tname, refpts);
		}
		printf("fieldgate: %d events | HOT %d WARM %d COLD %d\n",
			ev_total, hot, warm, cold);
		CreateDirectoryA("Output", nullptr);
		CreateDirectoryA("Output\\reports", nullptr);
		time_t now = time(nullptr);
		struct tm tmv;
		localtime_s(&tmv, &now);
		char st[64];
		strftime(st, sizeof(st), "%m%d-%H%M%S", &tmv);
		const std::string vp = std::string("Output\\reports\\field_")
			+ st + ".html";
		std::string verr;
		if (SearchLog::WriteHtml(vp, w, g, sink,
			"hotspot field validation", &verr))
			printf("fieldgate: heat report -> %s\n", vp.c_str());
		else
			printf("fieldgate: VIZ WRITE FAILED: %s\n", verr.c_str());
		fflush(stdout);
		return 0;
	}

	// exitgate: validate THE EXIT MAP against real runs (tapes are
	// VALIDATION ONLY, never seeding). For every RIDE in each tape
	// (board event -> next separation), compute the exit map from the
	// post-board state and rank the ACTUAL exit the player took under
	// each candidate functional (fmax / farea / fmass - which one
	// matches expert play is the open question this gate answers).
	// Also validates the ride energy bookkeeping (predicted exit speed
	// vs actual) and quantifies the doom cull (exits reaching nothing).
	// --bmap renders the B-overlay (marginal landing robustness across
	// exits) INSTEAD of the exit heat - report-only, per the user's
	// "careful with B" directive.
	int CmdExitGate(const std::string& map_path, const ReplayOpts& o,
	                const std::vector<std::string>& tapes, bool bmap) {
		World w;
		std::string err;
		w.true_interval_corner = o.corner_true;
		if (!w.Load(map_path, o.hulls, &err, o.edge_bevels)) {
			printf("LOAD FAILED (bsp): %s\n", err.c_str());
			return 1;
		}
		Route::Graph g;
		if (!Route::Build(w, &g, 2000.f, &err)) {
			printf("exitgate: %s\n", err.c_str());
			return 1;
		}
		SearchLog::Sink sink;
		std::vector<int> zreds;
		w.FindZoneBrushes(nullptr, &zreds);
		int heat_rides = 0;
		for (const std::string& tp : tapes) {
			Tape tape;
			if (!LoadTas(tp, tape, &err)) {
				printf("exitgate: %s: %s\n", tp.c_str(), err.c_str());
				continue;
			}
			std::string tname = tp;
			const size_t sl = tname.find_last_of("\\/");
			if (sl != std::string::npos)
				tname = tname.substr(sl + 1);
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
			std::vector<Vec3> refpts;
			std::vector<float> refspd;
			refpts.push_back(s.pos);
			refspd.push_back(Len(s.vel));
			// PASS 1: replay, collect BOARD events (long-air arrivals
			// on graph faces) + the separation that starts each long
			// stretch (the ride's actual EXIT).
			struct FEvent {
				int tick = 0;
				int fidx = -1;
				int sep_tick = 0;
				PlayerState sep;
				Vec3 cp, v1;
				bool ducked = false;
			};
			std::vector<FEvent> events;
			// Every separation that starts a LONG (>= 8 tick) airborne
			// stretch - the ride exits. The k-th ride's exit is the
			// FIRST of these after its board tick (a post-finish hop
			// must never be mistaken for the ending flight).
			struct SepRec { int tick; PlayerState st; };
			std::vector<SepRec> seps;
			int air = 0, sep_tick = 0;
			PlayerState sep;
			for (size_t t = 0; t < tape.frames.size(); ++t) {
				const TapeFrame& fr = tape.frames[t];
				TickEvents ev;
				MoveTick(s, w, o.params, fr.pitch, fr.yaw, fr.fmove,
					fr.smove, fr.umove, fr.buttons, &ev);
				if (t % 2 == 0) {
					refpts.push_back(s.pos);
					refspd.push_back(Len(s.vel));
				}
				if (ev.ncontacts > 0) {
					int fidx = -1;
					for (size_t fi = 0; fi < g.faces.size(); ++fi)
						if (g.faces[fi].brush == ev.contact_brush[0]
							&& g.faces[fi].side
								== ev.contact_plane[0])
							fidx = static_cast<int>(fi);
					if (fidx >= 0 && air >= 8) {
						FEvent fe;
						fe.tick = static_cast<int>(t);
						fe.fidx = fidx;
						fe.sep_tick = sep_tick;
						fe.sep = sep;
						fe.cp = ev.contact_pos[0];
						fe.v1 = ev.contact_vel[0];
						fe.ducked = s.ducked;
						events.push_back(fe);
					}
					air = 0;
				} else if (!s.on_ground) {
					if (air == 0) {
						sep = s;
						sep_tick = static_cast<int>(t);
					}
					air++;
					if (air == 8) {
						SepRec sr;
						sr.tick = sep_tick;
						sr.st = sep;
						seps.push_back(sr);
					}
				} else {
					air = 0;
				}
			}
			sink.gravity = o.params.gravity;
			sink.wish_rate = o.params.air_speed_cap
				* o.params.air_speed_cap;
			sink.AddRef("REF " + tname, refpts, &refspd);
			// PASS 2: one exit map per RIDE (board k -> exit toward
			// board k+1's face, or the zone for the last ride).
			for (size_t k = 0; k < events.size(); ++k) {
				const FEvent& fe = events[k];
				const Route::Face& fc = g.faces[fe.fidx];
				const bool last = k + 1 >= events.size();
				Field::NextTarget nt;
				if (!last) {
					nt.face = &g.faces[events[k + 1].fidx];
					nt.face_idx = events[k + 1].fidx;
					if (k + 2 < events.size()) {
						nt.after.has = true;
						nt.after.pt = g.faces[events[k + 2].fidx]
							.centroid;
					} else if (!zreds.empty()) {
						const int zi = w.IndexOfBrushId(zreds[0]);
						if (zi >= 0) {
							nt.after.has = true;
							nt.after.is_zone = true;
							Assemble::ZoneVolume(w.brushes[zi],
								&nt.after.zmin, &nt.after.zmax);
						}
					}
				} else {
					if (zreds.empty())
						continue;
					const int zi = w.IndexOfBrushId(zreds[0]);
					if (zi < 0)
						continue;
					nt.is_zone = true;
					Assemble::ZoneVolume(w.brushes[zi], &nt.zmin,
						&nt.zmax);
				}
				// The board state: contact point + POST-CLIP velocity.
				Vec3 bv;
				Fn::ClipVelocity(fe.v1, fc.n, &bv);
				const clock_t ck0 = clock();
				Field::ExitMap em = Field::ComputeExit(fe.cp, bv, fc,
					nt, o.params, fe.ducked, 48.f, 12, 48.f, 300);
				const double ck = static_cast<double>(clock() - ck0)
					/ CLOCKS_PER_SEC;
				int doomed = 0, tok = 0;
				for (const Field::ExitSample& es : em.samples) {
					if (!es.q.any) doomed++;
					if (es.turn_ok) tok++;
				}
				const int ns = static_cast<int>(em.samples.size());
				printf("exitgate: %s ride f%d (t%d..%s) -> %s | %d "
					"exits (%.1fs) | doomed %d%% | turn_ok %d%%\n",
					tname.c_str(), fe.fidx, fe.tick,
					last ? "end" : std::to_string(
						events[k + 1].sep_tick).c_str(),
					last ? "ZONE" : ("f" + std::to_string(
						nt.face_idx)).c_str(),
					ns, ck,
					ns > 0 ? doomed * 100 / ns : 0,
					ns > 0 ? tok * 100 / ns : 0);
				if (ns == 0)
					continue;
				// The HUMAN's actual exit: the separation that starts
				// the next long stretch (first one after this board -
				// never a later post-finish hop).
				bool have_hx = false;
				PlayerState hx;
				int hx_tick = -1;
				if (!last) {
					hx = events[k + 1].sep;
					hx_tick = events[k + 1].sep_tick;
					have_hx = true;
				} else {
					for (const auto& sr : seps) {
						if (sr.tick > fe.tick) {
							hx = sr.st;
							hx_tick = sr.tick;
							have_hx = true;
							break;
						}
					}
				}
				// The human's exit scored BESPOKE in the map's own
				// currency: their exact departure point (projected to
				// the plane) + exact in-plane direction, run through
				// the same EvalExitSample as every map sample - no
				// nearest-bucket distortion (30-degree direction
				// buckets shift vz by hundreds of u/s).
				Field::ExitSample hes;
				bool have_hes = false;
				if (have_hx) {
					const Vec3 hv = hx.vel;
					const Vec3 hpp = hx.pos - Scale(fc.n,
						Dot(hx.pos - fc.centroid, fc.n));
					Vec3 hvp = hv - Scale(fc.n, Dot(hv, fc.n));
					const float hvl = Len(hvp);
					if (hvl > 1.f)
						have_hes = Field::EvalExitSample(fe.cp, bv,
							fc, nt, o.params, fe.ducked, hpp,
							Scale(hvp, 1.f / hvl), 48.f, 300, &hes);
					const float g2 = 2.f * o.params.gravity;
					const float he = Dot(hv, hv)
						+ g2 * (hx.pos.Z - fc.zmin);
					printf("exitgate:   human exit t%d (%.0f,%.0f,"
						"%.0f) v(%.0f,%.0f,%.0f) | E %.0fk (%.0f "
						"u/s @ z%+.0f) | ride %d ticks\n",
						hx_tick, hx.pos.X, hx.pos.Y, hx.pos.Z,
						hv.X, hv.Y, hv.Z, he / 1000.f, Len(hv),
						hx.pos.Z - fc.zmin, hx_tick - fe.tick);
					if (have_hes)
						printf("exitgate:   bookkeeping: pred exit "
							"%.0f u/s vs actual %.0f (d%+.0f) | pred "
							"ride %.0f ticks vs actual %d | turn_ok "
							"%d\n",
							hes.pv, Len(hv), hes.pv - Len(hv),
							hes.ride_ticks, hx_tick - fe.tick,
							hes.turn_ok ? 1 : 0);
				}
				// BASELINE: the induced quality from the human's
				// ACTUAL exit state (their real position + velocity,
				// no discretization) - the honest field check, and
				// the diagnostic when the matched sample disagrees.
				if (have_hx) {
					if (!last && nt.face) {
						Field::FaceMap afm = Field::Compute(hx.pos,
							hx.vel, *nt.face, o.params, hx.ducked,
							48.f, 300, 1.f, &nt.after);
						Field::Quality aq = Field::MapQuality(afm,
							o.params);
						if (aq.any)
							printf("exitgate:   actual-state "
								"induced: qmax %.0fk qpot %.0fk "
								"area %.0f | argmax (%.0f,%.0f,"
								"%.0f)\n",
								aq.q_max / 1000.f, aq.q_pot / 1000.f,
								aq.area, aq.argmax.X, aq.argmax.Y,
								aq.argmax.Z);
						else
							printf("exitgate:   actual-state "
								"induced: DOOMED (field says their "
								"real exit reaches nothing - FIELD "
								"BUG, investigate)\n");
					} else if (last) {
						float mz = 0.f;
						const float azn = Field::ZoneReach(hx.pos,
							Len2D(hx.vel), hx.vel.Z, nt.zmin,
							nt.zmax, o.params, 300, &mz);
						if (azn >= 0.f)
							printf("exitgate:   actual-state zone "
								"reach: fly %.0f ticks, margin "
								"%.0fk (%.0f z)\n", azn,
								2.f * o.params.gravity * mz / 1000.f,
								mz);
						else
							printf("exitgate:   actual-state zone "
								"reach: DOOMED (their real ending "
								"flight says no - BOUND BUG)\n");
					}
				}
				// Functional report: the human's rank under each.
				auto pct = [&](float v, int which) {
					int below = 0, tot = 0;
					for (const Field::ExitSample& es : em.samples) {
						if (!es.q.any)
							continue;
						tot++;
						const float x = which == 0 ? es.q.q_max
							: (which == 1 ? es.q.area
							: (which == 2 ? es.q.mass : es.fpot));
						if (x <= v)
							below++;
					}
					return tot > 0 ? below * 100 / tot : 0;
				};
				if (!last) {
					const char* fn[4] = { "fmax ", "farea", "fmass",
						"fpot " };
					const int bi[4] = { em.best_max, em.best_area,
						em.best_mass, em.best_pot };
					for (int f = 0; f < 4; ++f) {
						if (bi[f] < 0) {
							printf("exitgate:   [%s] ALL DOOMED\n",
								fn[f]);
							continue;
						}
						const Field::ExitSample& bs =
							em.samples[bi[f]];
						const float bv2 = f == 0 ? bs.q.q_max
							: (f == 1 ? bs.q.area
							: (f == 2 ? bs.q.mass : bs.fpot));
						float hv2 = -1.f;
						int hp = -1;
						if (have_hes && hes.q.any) {
							hv2 = f == 0 ? hes.q.q_max
								: (f == 1 ? hes.q.area
								: (f == 2 ? hes.q.mass : hes.fpot));
							hp = pct(hv2, f);
						}
						const float dscale = (f == 0 || f == 3)
							? 1000.f : (f == 2 ? 1e6f : 1.f);
						printf("exitgate:   [%s] human %.0f "
							"(pct %d%%, ratio %.3f) | best %.0f "
							"@(%.0f,%.0f,%.0f) az%.0f spd %.0f "
							"rt %.0f\n",
							fn[f], hv2 / dscale, hp,
							bv2 != 0.f ? hv2 / bv2 : 0.f,
							bv2 / dscale, bs.pt.X, bs.pt.Y,
							bs.pt.Z, atan2f(bs.dir.Y, bs.dir.X)
								* 57.3f, bs.pv, bs.ride_ticks);
					}
					if (have_hes && !hes.q.any)
						printf("exitgate:   human bespoke sample "
							"DOOMED - the map's own currency rejects "
							"their real exit, investigate\n");
				} else {
					// Zone target: TIME is the functional.
					if (em.best_max >= 0) {
						const Field::ExitSample& bs =
							em.samples[em.best_max];
						printf("exitgate:   [ztime] best est %.0f "
							"ticks (ride %.0f + fly %.0f) @(%.0f,"
							"%.0f,%.0f) az%.0f spd %.0f | E_exit "
							"%.0fk (%.0f u/s @ z%+.0f)\n",
							bs.ride_ticks + bs.zn, bs.ride_ticks,
							bs.zn, bs.pt.X, bs.pt.Y, bs.pt.Z,
							atan2f(bs.dir.Y, bs.dir.X) * 57.3f,
							bs.pv, bs.e_exit / 1000.f, bs.pv,
							bs.pt.Z - fc.zmin);
						if (have_hes) {
							if (hes.q.any)
								printf("exitgate:   [ztime] human "
									"bespoke est %.0f ticks (ride "
									"%.0f + fly %.0f)\n",
									hes.ride_ticks + hes.zn,
									hes.ride_ticks, hes.zn);
							else
								printf("exitgate:   [ztime] human "
									"bespoke DOOMED (reach says no) "
									"- bound too tight?\n");
						}
					} else {
						printf("exitgate:   [ztime] ALL EXITS "
							"DOOMED\n");
					}
				}
				// Render: exit heat (best fmass over dirs per cell,
				// dimmed when only non-turn_ok dirs) or the B-overlay.
				if (heat_rides < 4 && !bmap) {
					// Heat = the time-priced functional (fpot) per
					// departure cell, best over directions.
					std::vector<float> cell(static_cast<size_t>(
						em.nu) * em.nv, -1e30f);
					std::vector<char> cok(static_cast<size_t>(
						em.nu) * em.nv, 0);
					float hi_v = -1e30f, lo_v = 1e30f;
					for (const Field::ExitSample& es : em.samples) {
						if (!es.q.any)
							continue;
						const size_t ci = static_cast<size_t>(
							es.iv) * em.nu + es.iu;
						const float v = es.fpot;
						if (v > cell[ci]) {
							cell[ci] = v;
							cok[ci] = es.turn_ok ? 1 : 0;
						}
						if (v > hi_v) hi_v = v;
						if (v < lo_v) lo_v = v;
					}
					for (int cv = 0; cv < em.nv; ++cv)
					for (int cu = 0; cu < em.nu; ++cu) {
						const size_t ci = static_cast<size_t>(cv)
							* em.nu + cu;
						if (cell[ci] < -1e29f)
							continue;
						float v01 = hi_v > lo_v
							? (cell[ci] - lo_v) / (hi_v - lo_v)
							: 1.f;
						if (!cok[ci])
							v01 *= 0.4f;
						const Vec3 q = em.origin
							+ Scale(em.ud, static_cast<float>(cu)
								* em.du)
							+ Scale(em.vd, static_cast<float>(cv)
								* em.dv);
						const Vec3 lift = Scale(fc.n, 2.f);
						const Vec3 c00 = q + lift
							- Scale(em.ud, em.du * 0.5f)
							- Scale(em.vd, em.dv * 0.5f);
						const Vec3 c10 = c00
							+ Scale(em.ud, em.du);
						const Vec3 c01 = c00
							+ Scale(em.vd, em.dv);
						const Vec3 c11 = c10
							+ Scale(em.vd, em.dv);
						sink.AddHeat(c00, c10, c11, v01);
						sink.AddHeat(c00, c11, c01, v01);
					}
					heat_rides++;
				}
				// B-overlay (report-only, --bmap): mean induced heat
				// per landing cell on the NEXT face across exits -
				// "landing zones that stay hot no matter how you
				// leave" (marginal robustness).
				if (bmap && !last && nt.face && heat_rides < 4) {
					std::vector<float> bsum;
					std::vector<int> bcnt;
					Field::FaceMap frame;
					bool have_frame = false;
					int used = 0;
					const int stride = ns > 120 ? ns / 120 : 1;
					for (int i = 0; i < ns; i += stride) {
						const Field::ExitSample& es = em.samples[i];
						if (!es.q.any)
							continue;
						Field::FaceMap fm = Field::Compute(es.pt,
							Scale(es.dir, es.pv), *nt.face,
							o.params, fe.ducked, 48.f, 300, 1.f,
							&nt.after);
						if (!have_frame) {
							frame = fm;
							bsum.assign(fm.samples.size(), 0.f);
							bcnt.assign(fm.samples.size(), 0);
							have_frame = true;
						}
						for (size_t j = 0; j < fm.samples.size()
							&& j < bsum.size(); ++j) {
							if (fm.samples[j].reachable) {
								bsum[j] += fm.samples[j].e_eff;
								bcnt[j]++;
							}
						}
						used++;
					}
					if (have_frame && used > 0) {
						float bhi = 0.f;
						for (size_t j = 0; j < bsum.size(); ++j)
							if (bcnt[j] > 0
								&& bsum[j] / bcnt[j] > bhi)
								bhi = bsum[j] / bcnt[j];
						for (int cv = 0; cv < frame.nv; ++cv)
						for (int cu = 0; cu < frame.nu; ++cu) {
							const size_t j = static_cast<size_t>(
								cv) * frame.nu + cu;
							if (bcnt[j] == 0 || bhi <= 0.f)
								continue;
							const float v01 = (bsum[j] / bcnt[j])
								/ bhi;
							const Vec3 q = frame.origin
								+ Scale(frame.ud,
									static_cast<float>(cu)
									* frame.du)
								+ Scale(frame.vd,
									static_cast<float>(cv)
									* frame.dv);
							const Vec3 lift =
								Scale(nt.face->n, 2.f);
							const Vec3 c00 = q + lift
								- Scale(frame.ud, frame.du * 0.5f)
								- Scale(frame.vd, frame.dv * 0.5f);
							const Vec3 c10 = c00
								+ Scale(frame.ud, frame.du);
							const Vec3 c01 = c00
								+ Scale(frame.vd, frame.dv);
							const Vec3 c11 = c10
								+ Scale(frame.vd, frame.dv);
							sink.AddHeat(c00, c10, c11, v01);
							sink.AddHeat(c00, c11, c01, v01);
						}
						printf("exitgate:   B-overlay: %d exits "
							"averaged onto f%d\n", used,
							nt.face_idx);
						heat_rides++;
					}
				}
				// Markers: human exit cross + per-functional best
				// exit arrows.
				if (have_hx) {
					std::vector<Vec3> cross;
					cross.push_back(hx.pos + Vec3(-14.f, 0.f, 0.f));
					cross.push_back(hx.pos + Vec3(14.f, 0.f, 0.f));
					cross.push_back(hx.pos);
					cross.push_back(hx.pos + Vec3(0.f, -14.f, 0.f));
					cross.push_back(hx.pos + Vec3(0.f, 14.f, 0.f));
					cross.push_back(hx.pos);
					cross.push_back(hx.pos + Vec3(0.f, 0.f, -14.f));
					cross.push_back(hx.pos + Vec3(0.f, 0.f, 14.f));
					sink.AddRef("human exit r" + std::to_string(k),
						cross);
				}
				const int bl[4] = { em.best_max, em.best_area,
					em.best_mass, em.best_pot };
				const char* bn[4] = { "best fmax r", "best farea r",
					"best fmass r", "best fpot r" };
				for (int f = 0; f < (last ? 1 : 4); ++f) {
					if (bl[f] < 0)
						continue;
					const Field::ExitSample& bs = em.samples[bl[f]];
					std::vector<Vec3> arrow;
					arrow.push_back(bs.pt);
					arrow.push_back(bs.pt + Scale(bs.dir, 150.f));
					sink.AddRef(std::string(last ? "best ztime r"
						: bn[f]) + std::to_string(k), arrow);
				}
			}
		}
		CreateDirectoryA("Output", nullptr);
		CreateDirectoryA("Output\\reports", nullptr);
		time_t now = time(nullptr);
		struct tm tmv;
		localtime_s(&tmv, &now);
		char st[64];
		strftime(st, sizeof(st), "%m%d-%H%M%S", &tmv);
		const std::string vp = std::string("Output\\reports\\exit_")
			+ (bmap ? "B_" : "") + st + ".html";
		std::string verr;
		if (SearchLog::WriteHtml(vp, w, g, sink,
			bmap ? "exit map B-overlay (marginal landing robustness)"
				: "exit map validation", &verr))
			printf("exitgate: report -> %s\n", vp.c_str());
		else
			printf("exitgate: VIZ WRITE FAILED: %s\n", verr.c_str());
		fflush(stdout);
		return 0;
	}

	// boardproof: THE BOARD-HEATMAP PROOF (user 2026-08-17: "I want
	// proof of effective, perfect, and inexpensive board heatmaps as
	// they were described originally"). Three claims, each measured
	// against the exact engine:
	//   PERFECT (bounds hold): probe-fly cells with the air primitive;
	//     at every strike the field's constituent laws must hold -
	//     arrival speed under the gain bound, |dot| at or above the
	//     tangency-law minimum, post-board total energy under the
	//     field's formula at the actual arrival. Cells the field calls
	//     UNREACHABLE are probed for falsification (a hit = a broken
	//     bound; the turn cap's 0.5 brake factor is the known suspect).
	//   EFFECTIVE: hit-rate by heat tier + correlation of cell energy
	//     vs achieved post-board energy.
	//   INEXPENSIVE: wall time per map at grids 48/32/24.
	int CmdBoardProof(const std::string& map_path, const ReplayOpts& o,
	                  const std::string& tape_path, int probe_evals) {
		World w;
		std::string err;
		w.true_interval_corner = o.corner_true;
		if (!w.Load(map_path, o.hulls, &err, o.edge_bevels)) {
			printf("LOAD FAILED (bsp): %s\n", err.c_str());
			return 1;
		}
		Route::Graph g;
		if (!Route::Build(w, &g, 2000.f, &err)) {
			printf("boardproof: %s\n", err.c_str());
			return 1;
		}
		Tape tape;
		if (!LoadTas(tape_path, tape, &err)) {
			printf("boardproof: %s\n", err.c_str());
			return 1;
		}
		std::vector<int> zreds;
		w.FindZoneBrushes(nullptr, &zreds);
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
		struct FEvent {
			int tick = 0;
			int fidx = -1;
			PlayerState sep;
			Vec3 cp;
		};
		std::vector<FEvent> events;
		int air = 0;
		PlayerState sep;
		for (size_t t = 0; t < tape.frames.size(); ++t) {
			const TapeFrame& fr = tape.frames[t];
			TickEvents ev;
			MoveTick(s, w, o.params, fr.pitch, fr.yaw, fr.fmove,
				fr.smove, fr.umove, fr.buttons, &ev);
			if (ev.ncontacts > 0) {
				int fidx = -1;
				for (size_t fi = 0; fi < g.faces.size(); ++fi)
					if (g.faces[fi].brush == ev.contact_brush[0]
						&& g.faces[fi].side == ev.contact_plane[0])
						fidx = static_cast<int>(fi);
				if (fidx >= 0 && air >= 8) {
					FEvent fe;
					fe.tick = static_cast<int>(t);
					fe.fidx = fidx;
					fe.sep = sep;
					fe.cp = ev.contact_pos[0];
					events.push_back(fe);
				}
				air = 0;
			} else if (!s.on_ground) {
				if (air == 0)
					sep = s;
				air++;
			} else {
				air = 0;
			}
		}
		// INEXPENSIVE: cost at three grids on the first event.
		if (!events.empty()) {
			const FEvent& fe = events[0];
			const float grids[3] = { 48.f, 32.f, 24.f };
			for (int gi = 0; gi < 3; ++gi) {
				const clock_t c0 = clock();
				int cells = 0;
				for (int rep = 0; rep < 20; ++rep) {
					Field::FaceMap fm = Field::Compute(fe.sep.pos,
						fe.sep.vel, g.faces[fe.fidx], o.params,
						fe.sep.ducked, grids[gi], 300);
					cells = static_cast<int>(fm.samples.size());
				}
				const double ms = 1000.0
					* static_cast<double>(clock() - c0)
					/ CLOCKS_PER_SEC / 20.0;
				printf("boardproof: COST grid %.0f = %.2f ms/map "
					"(%d cells, %.1f us/cell)\n", grids[gi], ms,
					cells, cells > 0 ? ms * 1000.0 / cells : 0.0);
			}
		}
		int probes = 0, hits = 0;
		int v_speed = 0, v_loss = 0, v_energy = 0;
		int fals_probe = 0, fals_hit = 0;
		std::vector<float> ce, me;   // cell e_eff vs measured E (hits)
		int tier_hit[3] = { 0, 0, 0 }, tier_try[3] = { 0, 0, 0 };
		for (const FEvent& fe : events) {
			const Route::Face& fc = g.faces[fe.fidx];
			Field::FaceMap fm = Field::Compute(fe.sep.pos, fe.sep.vel,
				fc, o.params, fe.sep.ducked, 32.f, 300);
			std::vector<int> reach;
			for (size_t i = 0; i < fm.samples.size(); ++i)
				if (fm.samples[i].reachable)
					reach.push_back(static_cast<int>(i));
			if (reach.empty())
				continue;
			std::sort(reach.begin(), reach.end(),
				[&](int a, int b) {
					return fm.samples[a].e_eff > fm.samples[b].e_eff;
				});
			// Probe set: hottest, 25/50/75 percentile, nearest to the
			// human strike, and three stride-spread cells.
			std::vector<int> pick;
			auto add_unique = [&](int idx) {
				for (int p2 : pick)
					if (p2 == idx)
						return;
				pick.push_back(idx);
			};
			add_unique(reach[0]);
			add_unique(reach[reach.size() / 4]);
			add_unique(reach[reach.size() / 2]);
			add_unique(reach[(reach.size() * 3) / 4]);
			{
				int ni = reach[0];
				float nd = 1e30f;
				for (int i : reach) {
					const float d = Len(fm.samples[i].q - fe.cp);
					if (d < nd) {
						nd = d;
						ni = i;
					}
				}
				add_unique(ni);
			}
			for (size_t k = 1; k + 1 < reach.size() && pick.size() < 8;
				k += reach.size() / 4 + 1)
				add_unique(reach[k]);
			const float s0 = Len2D(fe.sep.vel);
			const float vz0 = fe.sep.vel.Z;
			const float g2 = 2.f * o.params.gravity;
			for (int ci : pick) {
				const Field::Sample& sm = fm.samples[ci];
				Air::Target t;
				t.face = fe.fidx;
				t.aim = sm.q;
				t.aim_region = false;
				t.dot_cap = 2000.f;
				t.max_ticks = 300;
				Air::Result ar = Air::SolveTransfer(fe.sep, w,
					o.params, g, t, 4, probe_evals);
				probes++;
				const float rel = fm.e_hi > fm.e_lo
					? (sm.e_eff - fm.e_lo) / (fm.e_hi - fm.e_lo)
					: 1.f;
				const int tier = rel >= 0.85f ? 0
					: (rel >= 0.5f ? 1 : 2);
				tier_try[tier]++;
				if (!ar.hit)
					continue;
				hits++;
				tier_hit[tier]++;
				const int n_act = ar.tick;
				const float s_bound = Envelope::SMax(s0, n_act,
					o.params);
				if (ar.speed2d > s_bound + 1.f) {
					v_speed++;
					printf("boardproof: SPEED BOUND BROKEN f%d "
						"cell(%.0f,%.0f,%.0f): %.1f > %.1f @ n%d\n",
						fe.fidx, sm.q.X, sm.q.Y, sm.q.Z, ar.speed2d,
						s_bound, n_act);
				}
				const float min_dot = Board::MinApproachDot(
					ar.speed2d, ar.v1.Z, fc.n);
				if (min_dot != FLT_MAX
					&& fabsf(ar.dot) < min_dot - 0.5f) {
					v_loss++;
					printf("boardproof: MIN-LOSS LAW BROKEN f%d: "
						"|dot| %.2f < law %.2f\n", fe.fidx,
						fabsf(ar.dot), min_dot);
				}
				const float e_post = ar.speed * ar.speed
					- ar.dot * ar.dot
					+ g2 * (ar.pos.Z - fc.zmin);
				const float vz_act = Envelope::VzAfter(vz0, n_act,
					o.params);
				const float res_b = Board::MinApproachDot(s_bound,
					vz_act, fc.n);
				const float e_bound = s_bound * s_bound
					+ vz_act * vz_act
					- (res_b == FLT_MAX ? 0.f : res_b * res_b)
					+ g2 * (ar.pos.Z - fc.zmin);
				if (e_post > e_bound + 500.f) {
					v_energy++;
					printf("boardproof: ENERGY BOUND BROKEN f%d: "
						"%.0f > %.0f @ n%d\n", fe.fidx, e_post,
						e_bound, n_act);
				}
				// Effectiveness pair: the STRIKE's own cell.
				{
					const Vec3 dq = ar.pos - fm.origin;
					int iu = static_cast<int>(Dot(dq, fm.ud)
						/ fm.du + 0.5f);
					int iv = static_cast<int>(Dot(dq, fm.vd)
						/ fm.dv + 0.5f);
					if (iu >= 0 && iu < fm.nu && iv >= 0
						&& iv < fm.nv) {
						const Field::Sample& hs = fm.samples[
							static_cast<size_t>(iv) * fm.nu + iu];
						if (hs.reachable) {
							ce.push_back(hs.e_eff);
							me.push_back(e_post);
						}
					}
				}
			}
			// FALSIFICATION: frontier unreachable cells, classified by
			// the bound that rejected them; probe the falsifiable ones
			// (distance / turn - apex and no-approach are exact math).
			int fcount = 0;
			for (size_t i = 0; i < fm.samples.size() && fcount < 5;
				++i) {
				const Field::Sample& sm = fm.samples[i];
				if (!sm.in_face || sm.reachable)
					continue;
				// Adjacent to a reachable cell?
				const int iu = static_cast<int>(i)
					% fm.nu;
				const int iv = static_cast<int>(i) / fm.nu;
				bool frontier = false;
				for (int du2 = -1; du2 <= 1 && !frontier; ++du2)
				for (int dv2 = -1; dv2 <= 1 && !frontier; ++dv2) {
					const int ju = iu + du2, jv = iv + dv2;
					if (ju < 0 || ju >= fm.nu || jv < 0
						|| jv >= fm.nv)
						continue;
					if (fm.samples[static_cast<size_t>(jv) * fm.nu
						+ ju].reachable)
						frontier = true;
				}
				if (!frontier)
					continue;
				// Reason: exact-math rejections are not probed.
				const float dzq = sm.q.Z - fe.sep.pos.Z;
				const float disc = vz0 * vz0
					- 2.f * o.params.gravity * dzq;
				if (disc < 0.f)
					continue;   // above apex - exact
				fcount++;
				fals_probe++;
				Air::Target t;
				t.face = fe.fidx;
				t.aim = sm.q;
				t.aim_region = false;
				t.dot_cap = 2000.f;
				t.max_ticks = 300;
				Air::Result ar = Air::SolveTransfer(fe.sep, w,
					o.params, g, t, 4, probe_evals);
				if (ar.hit && Len(ar.pos - sm.q) < 48.f) {
					fals_hit++;
					printf("boardproof: UNREACHABLE CLAIM BROKEN "
						"f%d cell(%.0f,%.0f,%.0f): engine hit it "
						"(dot %.1f, %.0fu away)\n", fe.fidx,
						sm.q.X, sm.q.Y, sm.q.Z, ar.dot,
						Len(ar.pos - sm.q));
				}
			}
		}
		// Correlation (effectiveness).
		float corr = 0.f;
		if (ce.size() >= 3) {
			double mc = 0, mm = 0;
			for (size_t i = 0; i < ce.size(); ++i) {
				mc += ce[i];
				mm += me[i];
			}
			mc /= ce.size();
			mm /= me.size();
			double sc = 0, sm2 = 0, sx = 0;
			for (size_t i = 0; i < ce.size(); ++i) {
				sc += (ce[i] - mc) * (ce[i] - mc);
				sm2 += (me[i] - mm) * (me[i] - mm);
				sx += (ce[i] - mc) * (me[i] - mm);
			}
			if (sc > 0 && sm2 > 0)
				corr = static_cast<float>(sx / sqrt(sc * sm2));
		}
		printf("boardproof: %d probes, %d hits | hit-rate hot %d/%d "
			"warm %d/%d cold %d/%d\n", probes, hits,
			tier_hit[0], tier_try[0], tier_hit[1], tier_try[1],
			tier_hit[2], tier_try[2]);
		printf("boardproof: BOUNDS speed %d loss %d energy %d "
			"violations | FALSIFICATION %d/%d unreachable claims "
			"broken | energy corr %.3f (%d pairs)\n",
			v_speed, v_loss, v_energy, fals_hit, fals_probe, corr,
			static_cast<int>(ce.size()));
		printf("boardproof: %s\n",
			(v_speed + v_loss + v_energy + fals_hit) == 0
				? "GATE PASS" : "GATE FAIL");
		fflush(stdout);
		return (v_speed + v_loss + v_energy + fals_hit) == 0 ? 0 : 1;
	}

	// exitbench: EXIT-MAP MODEL COMPARISON ON ENGINE TRUTH (user
	// 2026-08-17: "a comparison across different ramp exit heatmap
	// models so we can tune this to actually be helpful as a variable
	// resolution state space shrinker"). For each ride of the tape,
	// run the assembler's own carve search from the real post-board
	// state with the exit collector on - every evaluated candidate
	// yields (first exit state -> actual fate). Then each model
	// predicts alive/dead + deliverable energy FROM THE EXIT STATE
	// ALONE, and is graded against what the engine actually did:
	//   false-kill  = model says dead, engine boarded CLEAN  (must ~0)
	//   scrapecatch = model says dead on scraped boards
	//   deadcatch   = model says dead on grounded/missed
	//   shrink      = fraction of exit space the model removes
	//   E bias/MAE/corr on clean boards.
	// Models: M0 = feasibility only (the shipped map);
	//         M1 = M0 + flight-arc world clearance;
	//         M2 = M0 + braking-law turn pricing;
	//         M3 = M1 + M2.
	int CmdExitBench(const std::string& map_path, const ReplayOpts& o,
	                 const std::string& tape_path, int bench_evals) {
		World w;
		std::string err;
		w.true_interval_corner = o.corner_true;
		if (!w.Load(map_path, o.hulls, &err, o.edge_bevels)) {
			printf("LOAD FAILED (bsp): %s\n", err.c_str());
			return 1;
		}
		Route::Graph g;
		if (!Route::Build(w, &g, 2000.f, &err)) {
			printf("exitbench: %s\n", err.c_str());
			return 1;
		}
		Tape tape;
		if (!LoadTas(tape_path, tape, &err)) {
			printf("exitbench: %s\n", err.c_str());
			return 1;
		}
		std::vector<int> zreds;
		w.FindZoneBrushes(nullptr, &zreds);
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
		struct FEvent {
			int tick = 0;
			int fidx = -1;
			PlayerState at;      // state right after the board tick
		};
		std::vector<FEvent> events;
		int air = 0;
		for (size_t t = 0; t < tape.frames.size(); ++t) {
			const TapeFrame& fr = tape.frames[t];
			TickEvents ev;
			MoveTick(s, w, o.params, fr.pitch, fr.yaw, fr.fmove,
				fr.smove, fr.umove, fr.buttons, &ev);
			if (ev.ncontacts > 0) {
				int fidx = -1;
				for (size_t fi = 0; fi < g.faces.size(); ++fi)
					if (g.faces[fi].brush == ev.contact_brush[0]
						&& g.faces[fi].side == ev.contact_plane[0])
						fidx = static_cast<int>(fi);
				if (fidx >= 0 && air >= 8) {
					FEvent fe;
					fe.tick = static_cast<int>(t);
					fe.fidx = fidx;
					fe.at = s;
					events.push_back(fe);
				}
				air = 0;
			} else if (!s.on_ground) {
				air++;
			} else {
				air = 0;
			}
		}
		const float wr = o.params.air_speed_cap * o.params.air_speed_cap;
		const float g2 = 2.f * o.params.gravity;
		for (size_t k = 0; k < events.size(); ++k) {
			const FEvent& fe = events[k];
			const Route::Face& fc = g.faces[fe.fidx];
			const bool last = k + 1 >= events.size();
			const Route::Face* ntface = last ? nullptr
				: &g.faces[events[k + 1].fidx];
			Carve::Target ct;
			ct.face = fe.fidx;
			ct.max_ticks = 320;
			Field::NextCtx after;
			Vec3 zmin, zmax;
			if (!zreds.empty()) {
				const int zi = w.IndexOfBrushId(zreds[0]);
				if (zi >= 0)
					Assemble::ZoneVolume(w.brushes[zi], &zmin, &zmax);
			}
			if (!last) {
				ct.tap_brush = ntface->brush;
				ct.tap_side = ntface->side;
				ct.tap_face = events[k + 1].fidx;
				if (k + 2 < events.size()) {
					ct.next_face = events[k + 2].fidx;
					after.has = true;
					after.pt = g.faces[events[k + 2].fidx].centroid;
				} else {
					ct.next_is_zone = true;
					ct.zone_min = zmin;
					ct.zone_max = zmax;
					after.has = true;
					after.is_zone = true;
					after.zmin = zmin;
					after.zmax = zmax;
				}
				const Vec3 lnext = ntface->centroid;
				ct.exit_heading = atan2f(lnext.Y - fe.at.pos.Y,
					lnext.X - fe.at.pos.X);
				const float se = Len2D(fe.at.vel);
				const float es = se > 100.f
					? Len(lnext - fe.at.pos) / (se * o.params.dt)
					: 80.f;
				ct.aim_tick = es < 10.f ? 10
					: (es > 240.f ? 240 : static_cast<int>(es));
				ct.tick_w = 0.02f;
			} else {
				ct.to_zone = true;
				ct.zone_min = zmin;
				ct.zone_max = zmax;
				Vec3 rim = fe.at.pos;
				if (rim.X < zmin.X) rim.X = zmin.X;
				if (rim.X > zmax.X) rim.X = zmax.X;
				if (rim.Y < zmin.Y) rim.Y = zmin.Y;
				if (rim.Y > zmax.Y) rim.Y = zmax.Y;
				ct.exit_heading = atan2f(rim.Y - fe.at.pos.Y,
					rim.X - fe.at.pos.X);
				ct.aim_tick = 200;
			}
			std::vector<Carve::ExitRec> recs;
			recs.reserve(65536);
			Carve::g_exit_rec = &recs;
			Carve::g_doom_cull = false;   // the bench needs TRUE fates
			Carve::SolveCarve(fe.at, w, o.params, g, ct, 6,
				bench_evals);
			Carve::g_doom_cull = true;
			Carve::g_exit_rec = nullptr;
			// Truth classes.
			int n_clean = 0, n_scrape = 0, n_dead = 0;
			auto truth = [&](const Carve::ExitRec& er) -> int {
				const bool ok = last
					? er.outcome == SearchLog::kZoned
					: er.outcome == SearchLog::kHit;
				if (ok && er.graze2 < 1000.f)
					return 0;   // CLEAN
				if (ok)
					return 1;   // SCRAPED
				return 2;       // DEAD
			};
			for (const auto& er : recs) {
				const int tc = truth(er);
				if (tc == 0) n_clean++;
				else if (tc == 1) n_scrape++;
				else n_dead++;
			}
			printf("exitbench: ride f%d -> %s | %d records: clean %d "
				"scrape %d dead %d\n", fe.fidx,
				last ? "ZONE" : ("f" + std::to_string(
					events[k + 1].fidx)).c_str(),
				static_cast<int>(recs.size()), n_clean, n_scrape,
				n_dead);
			if (recs.empty())
				continue;
			// Model evaluation. Per record and model: dead? deliverable?
			struct MStat {
				int fk = 0, sc = 0, dc = 0, dead = 0;
				double be = 0, ae = 0;
				int ne = 0;
				std::vector<float> pe, te;
				double us = 0;
			};
			MStat ms[5];   // + Mw = the WIRED doom test (must match
			               // the sim's own cull exactly)
			for (const auto& er : recs) {
				const int tc = truth(er);
				const float s2d = Len2D(er.xvel);
				// Shared: induced map from the exact exit state.
				bool alive0 = false;
				float deliver0 = -1e30f;
				const Field::Sample* argm = nullptr;
				Field::FaceMap fm;
				float zn = -1.f;
				const clock_t ck0 = clock();
				if (!last && ntface) {
					fm = Field::Compute(er.xpos, er.xvel, *ntface,
						o.params, fe.at.ducked, 32.f, 300, 1.f,
						&after);
					// Laddered priced argmax (viability first).
					for (int tier = 0; tier < 4 && !argm; ++tier) {
						float best = -1e30f;
						for (const Field::Sample& sm : fm.samples) {
							if (!sm.reachable)
								continue;
							if (tier == 0 && !(sm.run_viable
								&& sm.free_turn))
								continue;
							if (tier == 1 && !sm.run_viable)
								continue;
							if (tier == 2 && !sm.free_turn)
								continue;
							const float vp = sm.e_eff - wr * sm.n;
							if (vp > best) {
								best = vp;
								argm = &sm;
							}
						}
						if (argm)
							deliver0 = best;
					}
					alive0 = argm != nullptr;
				} else {
					float mz = 0.f;
					zn = Field::ZoneReach(er.xpos, s2d, er.xvel.Z,
						zmin, zmax, o.params, 300, &mz);
					alive0 = zn >= 0.f;
					deliver0 = alive0 ? g2 * mz : -1e30f;
				}
				const double us0 = 1e6
					* static_cast<double>(clock() - ck0)
					/ CLOCKS_PER_SEC;
				// M1: ballistic-arc world clearance toward the
				// model's own landing pick (modal straight arc).
				bool clear1 = true;
				if (alive0) {
					Vec3 tgt = last
						? Vec3((zmin.X + zmax.X) * 0.5f,
							(zmin.Y + zmax.Y) * 0.5f, zmin.Z)
						: argm->q;
					if (last) {
						Vec3 rim = er.xpos;
						if (rim.X < zmin.X) rim.X = zmin.X;
						if (rim.X > zmax.X) rim.X = zmax.X;
						if (rim.Y < zmin.Y) rim.Y = zmin.Y;
						if (rim.Y > zmax.Y) rim.Y = zmax.Y;
						tgt = Vec3(rim.X, rim.Y, zmin.Z);
					}
					const float dx = tgt.X - er.xpos.X;
					const float dy = tgt.Y - er.xpos.Y;
					const float l2d = sqrtf(dx * dx + dy * dy);
					const float ux = l2d > 1.f ? dx / l2d : 1.f;
					const float uy = l2d > 1.f ? dy / l2d : 0.f;
					const float nmax = !last && argm
						? argm->n + 6.f
						: (zn > 0.f ? zn + 6.f : 120.f);
					Vec3 prev = er.xpos;
					for (float n = 4.f; n <= nmax && clear1;
						n += 4.f) {
						const float d = s2d * n * o.params.dt;
						Vec3 pt(er.xpos.X + ux * (d < l2d
								? d : l2d),
							er.xpos.Y + uy * (d < l2d ? d : l2d),
							Envelope::ZAfter(er.xpos.Z, er.xvel.Z,
								static_cast<int>(n), o.params));
						TraceResult tr;
						const float f2 = w.TraceHull(prev, pt,
							fe.at.ducked, &tr);
						if (f2 < 1.f) {
							const bool is_tgt = !last && ntface
								&& tr.brush == ntface->brush
								&& tr.plane == ntface->side;
							if (!is_tgt)
								clear1 = false;
							break;   // reached something - done
						}
						prev = pt;
						if (d >= l2d)
							break;
					}
				}
				// M2: braking-law price for turn beyond the free
				// budget (certified NewSpeed2 - no fitted numbers).
				float deliver2 = deliver0;
				bool alive2 = alive0;
				if (alive0 && !last && argm && s2d > 1.f) {
					const float h0 = atan2f(er.xvel.Y, er.xvel.X);
					const float theta = atan2f(
						argm->q.Y - er.xpos.Y,
						argm->q.X - er.xpos.X);
					const float need = Field::TurnNeed(h0, theta,
						argm->phi);
					const float free = Field::FreeTurnBudget(s2d,
						argm->n, o.params, fe.at.ducked);
					float excess = need - free;
					if (excess > 0.f) {
						float v = s2d;
						int it = 0;
						while (excess > 0.f && it++ < 300
							&& v > 100.f) {
							Strafe::TickLaw law = Strafe::Law(
								o.params, v, 1.f, fe.at.ducked);
							float btr = 0.f, bc = 0.f;
							for (int ci = 1; ci <= 16; ++ci) {
								const float c = -static_cast<float>(
									ci) / 16.f;
								const float s2 = 1.f - c * c;
								const float tr2 = law.TurnRad(
									Strafe::TrueWishCos(c),
									s2 > 0.f ? sqrtf(s2) : 0.f);
								if (tr2 > btr) {
									btr = tr2;
									bc = c;
								}
							}
							if (btr <= 1e-5f)
								break;
							excess -= btr;
							const float v2 = law.NewSpeed2(
										Strafe::TrueWishCos(bc));
							v = v2 > 0.f ? sqrtf(v2) : 0.f;
						}
						deliver2 = deliver0 - (s2d * s2d - v * v);
						alive2 = v > 100.f;
					}
				}
				const double us1 = 1e6
					* static_cast<double>(clock() - ck0)
					/ CLOCKS_PER_SEC;
				const bool dead_w = Carve::ExitDoomed(er.xpos,
					er.xvel, o.params, ntface, last, zmin, zmax);
				const bool dead_m[5] = {
					!alive0,
					!alive0 || !clear1,
					!alive2,
					!alive2 || !clear1,
					dead_w };
				const float del_m[5] = { deliver0, deliver0,
					deliver2, deliver2, deliver0 };
				for (int m = 0; m < 5; ++m) {
					MStat& st = ms[m];
					st.us += m == 0 ? us0 : us1;
					if (dead_m[m]) {
						st.dead++;
						if (tc == 0) st.fk++;
						else if (tc == 1) st.sc++;
						else st.dc++;
					}
					if (tc == 0 && !dead_m[m] && !last) {
						const double e = del_m[m] - er.e_end;
						st.be += e;
						st.ae += e < 0 ? -e : e;
						st.ne++;
						st.pe.push_back(del_m[m]);
						st.te.push_back(er.e_end);
					}
				}
			}
			const char* mn[5] = { "M0-feas ", "M1-clear", "M2-turn ",
				"M3-both ", "Mw-WIRED" };
			printf("exitbench:   model     falsekill scrapecatch "
				"deadcatch shrink  Ebias   EMAE  corr  us/rec\n");
			for (int m = 0; m < 5; ++m) {
				MStat& st = ms[m];
				float corr = 0.f;
				if (st.pe.size() >= 3) {
					double mp = 0, mt = 0;
					for (size_t i = 0; i < st.pe.size(); ++i) {
						mp += st.pe[i];
						mt += st.te[i];
					}
					mp /= st.pe.size();
					mt /= st.te.size();
					double sp = 0, st2 = 0, sx = 0;
					for (size_t i = 0; i < st.pe.size(); ++i) {
						sp += (st.pe[i] - mp) * (st.pe[i] - mp);
						st2 += (st.te[i] - mt) * (st.te[i] - mt);
						sx += (st.pe[i] - mp) * (st.te[i] - mt);
					}
					if (sp > 0 && st2 > 0)
						corr = static_cast<float>(
							sx / sqrt(sp * st2));
				}
				printf("exitbench:   %s  %5.1f%%    %5.1f%%     "
					"%5.1f%%   %5.1f%%  %+5.0fk  %5.0fk  %.2f  "
					"%5.0f\n", mn[m],
					n_clean > 0 ? 100.f * st.fk / n_clean : 0.f,
					n_scrape > 0 ? 100.f * st.sc / n_scrape : 0.f,
					n_dead > 0 ? 100.f * st.dc / n_dead : 0.f,
					!recs.empty()
						? 100.f * st.dead / recs.size() : 0.f,
					st.ne > 0 ? st.be / st.ne / 1000.0 : 0.0,
					st.ne > 0 ? st.ae / st.ne / 1000.0 : 0.0,
					corr,
					!recs.empty() ? st.us / recs.size() : 0.0);
			}
			// Resolution validity: how pure are coarse exit bins in
			// TRUTH (a pure bin can be represented by one sample).
			for (int rz = 0; rz < 2; ++rz) {
				const float cell = rz == 0 ? 64.f : 128.f;
				const float db = rz == 0 ? 30.f : 45.f;
				std::map<long long, int[3]> bins;
				for (const auto& er : recs) {
					const int tc = truth(er);
					const long long bx = static_cast<long long>(
						floorf(er.xpos.X / cell)) + 4096;
					const long long by = static_cast<long long>(
						floorf(er.xpos.Y / cell)) + 4096;
					const float az = atan2f(er.xvel.Y, er.xvel.X)
						* 57.29578f + 180.f;
					const long long bd = static_cast<long long>(
						az / db);
					bins[(bx << 32) | (by << 16) | bd][tc]++;
				}
				long long tot = 0, maj = 0;
				int nb = 0;
				for (auto& kv : bins) {
					const int n = kv.second[0] + kv.second[1]
						+ kv.second[2];
					if (n < 3)
						continue;
					int mx = kv.second[0];
					if (kv.second[1] > mx) mx = kv.second[1];
					if (kv.second[2] > mx) mx = kv.second[2];
					tot += n;
					maj += mx;
					nb++;
				}
				printf("exitbench:   bin purity %.0fu/%.0fdeg: "
					"%.0f%% (%d bins)\n", cell, db,
					tot > 0 ? 100.0 * maj / tot : 0.0, nb);
			}
		}
		fflush(stdout);
		return 0;
	}

	// autopsy: FACEPLANT FORENSICS (user 2026-08-17: "trace back from
	// a sample of faceplants and find the bug"). Runs the assembler's
	// own leg search (same field construction: blind-entry fallback,
	// heat cells, aims) with the exit collector on, then for every
	// strike on the tap face answers, from ITS OWN exit state:
	//   - did it strike the cell it was scheduled to (ON/OFF-cell)?
	//   - was a tangent arrival even feasible from that exit (the
	//     strike cell's law minimum |dot| + turn need vs free budget)?
	//   - how hard did it actually hit?
	// The dominant class IS the bug.
	int CmdAutopsy(const std::string& map_path, const ReplayOpts& o,
	               const std::vector<std::string>& tapes,
	               int bench_evals) {
		World w;
		std::string err;
		w.true_interval_corner = o.corner_true;
		if (!w.Load(map_path, o.hulls, &err, o.edge_bevels)) {
			printf("LOAD FAILED (bsp): %s\n", err.c_str());
			return 1;
		}
		Route::Graph g;
		if (!Route::Build(w, &g, 2000.f, &err)) {
			printf("autopsy: %s\n", err.c_str());
			return 1;
		}
		std::vector<int> zreds;
		w.FindZoneBrushes(nullptr, &zreds);
		Vec3 zmin, zmax;
		if (!zreds.empty()) {
			const int zi = w.IndexOfBrushId(zreds[0]);
			if (zi >= 0)
				Assemble::ZoneVolume(w.brushes[zi], &zmin, &zmax);
		}
		for (const std::string& tp : tapes) {
			Tape tape;
			if (!LoadTas(tp, tape, &err)) {
				printf("autopsy: %s: %s\n", tp.c_str(), err.c_str());
				continue;
			}
			std::string tname = tp;
			const size_t sl = tname.find_last_of("\\/");
			if (sl != std::string::npos)
				tname = tname.substr(sl + 1);
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
			struct FEvent { int tick; int fidx; PlayerState at; };
			std::vector<FEvent> events;
			int air = 0;
			for (size_t t = 0; t < tape.frames.size(); ++t) {
				const TapeFrame& fr = tape.frames[t];
				TickEvents ev;
				MoveTick(s, w, o.params, fr.pitch, fr.yaw, fr.fmove,
					fr.smove, fr.umove, fr.buttons, &ev);
				if (ev.ncontacts > 0) {
					int fidx = -1;
					for (size_t fi = 0; fi < g.faces.size(); ++fi)
						if (g.faces[fi].brush == ev.contact_brush[0]
							&& g.faces[fi].side
								== ev.contact_plane[0])
							fidx = static_cast<int>(fi);
					if (fidx >= 0 && air >= 8) {
						FEvent fe;
						fe.tick = static_cast<int>(t);
						fe.fidx = fidx;
						fe.at = s;
						events.push_back(fe);
					}
					air = 0;
				} else if (!s.on_ground) {
					air++;
				} else {
					air = 0;
				}
			}
			for (size_t k = 0; k + 1 < events.size(); ++k) {
				const FEvent& fe = events[k];
				const Route::Face& lfc = g.faces[fe.fidx];
				const int nfi = events[k + 1].fidx;
				const Route::Face& nf = g.faces[nfi];
				// Assembler-equivalent target (field block included).
				Carve::Target ct;
				ct.face = fe.fidx;
				ct.max_ticks = 320;
				ct.tap_brush = nf.brush;
				ct.tap_side = nf.side;
				ct.tap_face = nfi;
				Field::NextCtx fctx;
				if (k + 2 < events.size()) {
					fctx.has = true;
					fctx.pt = g.faces[events[k + 2].fidx].centroid;
					ct.next_face = events[k + 2].fidx;
				} else {
					fctx.has = true;
					fctx.is_zone = true;
					fctx.zmin = zmin;
					fctx.zmax = zmax;
					ct.next_is_zone = true;
					ct.zone_min = zmin;
					ct.zone_max = zmax;
				}
				const Vec3 lnext = nf.centroid;
				ct.exit_heading = atan2f(lnext.Y - fe.at.pos.Y,
					lnext.X - fe.at.pos.X);
				const float se = Len2D(fe.at.vel);
				const float es = se > 100.f
					? Len(lnext - fe.at.pos) / (se * o.params.dt)
					: 80.f;
				ct.aim_tick = es < 10.f ? 10
					: (es > 240.f ? 240 : static_cast<int>(es));
				ct.tick_w = 0.02f;
				Field::FaceMap fmap = Field::Compute(fe.at.pos,
					fe.at.vel, nf, o.params, fe.at.ducked, 32.f,
					300, 1.f, &fctx);
				if (fmap.best < 0) {
					Field::NextTarget xnt;
					xnt.face = &nf;
					xnt.face_idx = nfi;
					xnt.after = fctx;
					Field::ExitMap em = Field::ComputeExit(
						fe.at.pos, fe.at.vel, lfc, xnt, o.params,
						fe.at.ducked, 64.f, 12, 64.f, 300);
					if (em.best_pot >= 0) {
						const Field::ExitSample& bs =
							em.samples[em.best_pot];
						fmap = Field::Compute(bs.pt,
							Scale(bs.dir, bs.pv), nf, o.params,
							fe.at.ducked, 32.f, 300, 1.f, &fctx);
					}
				}
				if (fmap.best >= 0) {
					ct.field_aim = fmap.samples[fmap.best].q;
					ct.have_field_aim = true;
					ct.field_phi = fmap.samples[fmap.best].phi;
					ct.field_n = fmap.samples[fmap.best].n;
					ct.have_field_arr = true;
					for (int pass = 0; pass < 2
						&& ct.cells.empty(); ++pass)
					for (const Field::Sample& sm : fmap.samples) {
						if (!sm.reachable || sm.e_eff <= 0.f)
							continue;
						if (pass == 0 && !sm.run_viable)
							continue;
						const float rel = fmap.e_hi > 0.f
							? sm.e_eff / fmap.e_hi : 0.f;
						const int rep = rel >= 0.85f ? 3
							: (rel >= 0.6f ? 1 : 0);
						for (int rp = 0; rp < rep
							&& ct.cells.size() < 192; ++rp) {
							Carve::Target::Cell cc;
							cc.q = sm.q;
							cc.phi = sm.phi;
							cc.n = sm.n;
							ct.cells.push_back(cc);
						}
					}
				}
				std::vector<Carve::ExitRec> recs;
				recs.reserve(65536);
				Carve::g_exit_rec = &recs;
				Carve::SolveCarve(fe.at, w, o.params, g, ct, 6,
					bench_evals);
				Carve::g_exit_rec = nullptr;
				// Classify every tap-face strike.
				int nb[4] = { 0, 0, 0, 0 };   // |dot| buckets
				int oncell[4] = { 0, 0, 0, 0 };
				int feas[4] = { 0, 0, 0, 0 };
				int turnok[4] = { 0, 0, 0, 0 };
				struct Worst { float dot; Carve::ExitRec er; };
				std::vector<Worst> worst;
				int hits = 0;
				for (const auto& er : recs) {
					if (er.outcome != SearchLog::kHit)
						continue;
					hits++;
					const float ad = fabsf(er.sdot);
					const int b = ad < 50.f ? 0
						: (ad < 150.f ? 1 : (ad < 300.f ? 2 : 3));
					nb[b]++;
					if (er.have_cell
						&& Len(er.spos - er.cell) < 64.f)
						oncell[b]++;
					// From ITS exit state: what was possible at the
					// strike cell?
					const float s2d = Len2D(er.xvel);
					const int fly = er.end_tick - er.xtick;
					const float vz_arr = Envelope::VzAfter(
						er.xvel.Z, fly, o.params);
					const float s_arr = Envelope::SMax(s2d, fly,
						o.params);
					const float res = Board::MinApproachDot(s_arr,
						vz_arr, nf.n);
					if (res != FLT_MAX && res < 50.f)
						feas[b]++;
					// Turn need to the tangent heading vs the free
					// budget over the actual flight.
					const float hn2 = sqrtf(nf.n.X * nf.n.X
						+ nf.n.Y * nf.n.Y);
					float cphi = s_arr * hn2 > 1e-4f
						? -vz_arr * nf.n.Z / (s_arr * hn2) : 0.f;
					if (cphi > 1.f) cphi = 1.f;
					if (cphi < -1.f) cphi = -1.f;
					const float psi2 = atan2f(nf.n.Y, nf.n.X);
					const float h0 = atan2f(er.xvel.Y, er.xvel.X);
					const float th2 = atan2f(
						er.spos.Y - er.xpos.Y,
						er.spos.X - er.xpos.X);
					const float pa = Steer::WrapPi(psi2
						+ acosf(cphi));
					const float pb = Steer::WrapPi(psi2
						- acosf(cphi));
					const float na = Field::TurnNeed(h0, th2, pa);
					const float nb2 = Field::TurnNeed(h0, th2, pb);
					const float need = na < nb2 ? na : nb2;
					const float freeb = Field::FreeTurnBudget(s2d,
						static_cast<float>(fly), o.params,
						fe.at.ducked);
					if (need <= freeb)
						turnok[b]++;
					if (b >= 2) {
						Worst ww;
						ww.dot = ad;
						ww.er = er;
						worst.push_back(ww);
					}
				}
				printf("autopsy: %s ride f%d -> f%d | %d records, "
					"%d tap hits\n", tname.c_str(), fe.fidx, nfi,
					static_cast<int>(recs.size()), hits);
				const char* bn[4] = { "|dot|<50  ", "50-150    ",
					"150-300   ", ">300 SLAM " };
				printf("autopsy:   bucket      n     on-cell  "
					"tangent-feasible  turn-in-budget\n");
				for (int b = 0; b < 4; ++b)
					printf("autopsy:   %s %5d  %5d     %5d          "
						"   %5d\n", bn[b], nb[b], oncell[b],
						feas[b], turnok[b]);
				std::sort(worst.begin(), worst.end(),
					[](const Worst& a, const Worst& b2) {
						return a.dot > b2.dot;
					});
				for (size_t i2 = 0; i2 < worst.size() && i2 < 5;
					++i2) {
					const Carve::ExitRec& er = worst[i2].er;
					printf("autopsy:   SAMPLE dot %.0f strike (%.0f,"
						"%.0f,%.0f) cell (%.0f,%.0f,%.0f) d %.0fu | "
						"exit (%.0f,%.0f,%.0f) v(%.0f,%.0f,%.0f) "
						"fly %d\n", -worst[i2].dot, er.spos.X,
						er.spos.Y, er.spos.Z, er.cell.X, er.cell.Y,
						er.cell.Z,
						er.have_cell ? Len(er.spos - er.cell) : -1.f,
						er.xpos.X, er.xpos.Y, er.xpos.Z, er.xvel.X,
						er.xvel.Y, er.xvel.Z,
						er.end_tick - er.xtick);
				}
			}
		}
		fflush(stdout);
		return 0;
	}

	// pathgate: THE CONSTRUCTED-PATH VALIDATION (SolverPath). For
	// every long-air transfer of the tape: from the real separation
	// state, target the HUMAN'S OWN strike cell (its tangent heading
	// + ballistic tick from the heatmap), construct the turn-hold-
	// turn profile, fly it ONCE on the exact engine, and compare the
	// board it produces against the tape's. Also times the
	// construction. PASS = every constructed flight strikes the face.
	int CmdPathGate(const std::string& map_path, const ReplayOpts& o,
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
			printf("pathgate: %s\n", err.c_str());
			return 1;
		}
		Tape tape;
		if (!LoadTas(tape_path, tape, &err)) {
			printf("pathgate: %s\n", err.c_str());
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
		struct FEvent {
			int tick = 0;
			int fidx = -1;
			PlayerState sep;
			Vec3 cp, v1;
		};
		std::vector<FEvent> events;
		int air = 0;
		PlayerState sep;
		for (size_t t = 0; t < tape.frames.size(); ++t) {
			const TapeFrame& fr = tape.frames[t];
			TickEvents ev;
			MoveTick(s, w, o.params, fr.pitch, fr.yaw, fr.fmove,
				fr.smove, fr.umove, fr.buttons, &ev);
			if (ev.ncontacts > 0) {
				int fidx = -1;
				for (size_t fi = 0; fi < g.faces.size(); ++fi)
					if (g.faces[fi].brush == ev.contact_brush[0]
						&& g.faces[fi].side == ev.contact_plane[0])
						fidx = static_cast<int>(fi);
				if (fidx >= 0 && air >= 8) {
					FEvent fe;
					fe.tick = static_cast<int>(t);
					fe.fidx = fidx;
					fe.sep = sep;
					fe.cp = ev.contact_pos[0];
					fe.v1 = ev.contact_vel[0];
					events.push_back(fe);
				}
				air = 0;
			} else if (!s.on_ground) {
				if (air == 0)
					sep = s;
				air++;
			} else {
				air = 0;
			}
		}
		int rows = 0, hits = 0;
		for (const FEvent& fe : events) {
			const Route::Face& fc = g.faces[fe.fidx];
			// The human's strike cell in the map from their sep.
			Field::FaceMap fm = Field::Compute(fe.sep.pos,
				fe.sep.vel, fc, o.params, fe.sep.ducked, 32.f, 300);
			if (fm.best < 0)
				continue;
			const Vec3 dq = fe.cp - fm.origin;
			int iu = static_cast<int>(Dot(dq, fm.ud) / fm.du + 0.5f);
			int iv = static_cast<int>(Dot(dq, fm.vd) / fm.dv + 0.5f);
			if (iu < 0) iu = 0;
			if (iu >= fm.nu) iu = fm.nu - 1;
			if (iv < 0) iv = 0;
			if (iv >= fm.nv) iv = fm.nv - 1;
			const Field::Sample& cell = fm.samples[
				static_cast<size_t>(iv) * fm.nu + iu];
			if (!cell.reachable)
				continue;
			rows++;
			const clock_t c0 = clock();
			// BOTH ballistic branches: the map stores its better-
			// energy root per cell, but a real arrival may use the
			// other (the human's f2 board is the descending root
			// where the map kept the ascending one) - no tick-band
			// correction bridges a branch gap.
			int n_try[2] = { static_cast<int>(cell.n), -1 };
			{
				const float dzq = cell.q.Z - fe.sep.pos.Z;
				const float g2r = o.params.gravity;
				const float disc = fe.sep.vel.Z * fe.sep.vel.Z
					- 2.f * g2r * dzq;
				if (disc >= 0.f) {
					const float sq = sqrtf(disc);
					const int ra = static_cast<int>(
						(fe.sep.vel.Z - sq)
						/ (g2r * o.params.dt));
					const int rb = static_cast<int>(
						(fe.sep.vel.Z + sq)
						/ (g2r * o.params.dt));
					const int other = fabsf(static_cast<float>(ra)
						- cell.n) > fabsf(static_cast<float>(rb)
						- cell.n) ? ra : rb;
					if (other >= 4 && other <= 300
						&& other != n_try[0])
						n_try[1] = other;
				}
			}
			Path::Plan pl;
			for (int bi2 = 0; bi2 < 2; ++bi2) {
				if (n_try[bi2] < 4)
					continue;
				Path::Plan cand = Path::Solve(fe.sep.pos,
					fe.sep.vel, cell.q, cell.phi, n_try[bi2],
					o.params, fe.sep.ducked);
				printf("pathgate:   t%4d f%d branch n=%d trace "
					"end_dist %.0f psi %.2f\n", fe.tick, fe.fidx,
					n_try[bi2], cand.end_dist, cand.psi);
				if (cand.end_dist < pl.end_dist)
					pl = cand;
			}
			const double us = 1e6
				* static_cast<double>(clock() - c0) / CLOCKS_PER_SEC;
			if (pl.end_dist >= 150.f || pl.heading.empty()) {
				printf("pathgate: t%4d f%d NO PLAN (end_dist %.0f) "
					"| tape dot %.1f\n", fe.tick, fe.fidx,
					pl.end_dist, Dot(fe.v1, fc.n));
				continue;
			}
			Air::Target at;
			at.face = fe.fidx;
			at.aim = cell.q;
			at.aim_region = false;
			at.dot_cap = 2000.f;
			at.max_ticks = pl.n + 12;
			// ENGINE LINE-SEARCH on the one knob (the shipped
			// behavior): the trace cannot see geometry (grazes
			// block the dive and model-based corrections turn INTO
			// the surface) - so psi is searched on the REAL engine:
			// a handful of flights around the planned value, first
			// on-cell strike wins.
			bool struck = false;
			float best_close = 1e9f;
			Vec3 best_pt;
			const float offs2[9] = { 0.f, 0.12f, -0.12f, 0.24f,
				-0.24f, 0.4f, -0.4f, 0.6f, -0.6f };
			for (int att = 0; att < 9 && !struck; ++att) {
				pl.heading.clear();
				Path::TraceTHT(fe.sep.pos,
					atan2f(fe.sep.vel.Y, fe.sep.vel.X),
					Len2D(fe.sep.vel), cell.q, cell.phi,
					Steer::WrapPi(pl.psi + offs2[att]), pl.n,
					o.params, fe.sep.ducked, &pl.heading);
				if (pl.heading.empty())
					continue;
				Air::Result ar = Air::FlyHeadingSpline(fe.sep, w,
					o.params, at, g, pl.heading, pl.n + 8);
				if (ar.hit && Len(ar.pos - cell.q) <= 64.f) {
					struck = true;
					hits++;
					printf("pathgate: t%4d f%d CONSTRUCTED dot "
						"%.1f @ (%.0f,%.0f,%.0f) miss-to-cell "
						"%.0fu, %d ticks, eval %d | tape dot %.1f "
						"| plan %.0fus\n",
						fe.tick, fe.fidx, ar.dot, ar.pos.X,
						ar.pos.Y, ar.pos.Z, Len(ar.pos - cell.q),
						ar.tick, att + 1, Dot(fe.v1, fc.n), us);
					break;
				}
				const Vec3 endp = ar.hit ? ar.pos
					: (ar.miss_dist < 1e8f ? ar.closest
						: ar.end_pos);
				const float cd = Len(endp - cell.q);
				if (cd < best_close) {
					best_close = cd;
					best_pt = endp;
				}
			}
			if (!struck)
				printf("pathgate: t%4d f%d MISSED line-search "
					"(best %.0fu at %.0f,%.0f,%.0f; cell %.0f,"
					"%.0f,%.0f) | tape dot %.1f\n",
					fe.tick, fe.fidx, best_close, best_pt.X,
					best_pt.Y, best_pt.Z, cell.q.X, cell.q.Y,
					cell.q.Z, Dot(fe.v1, fc.n));
		}
		printf("pathgate: %d/%d constructed flights struck | %s\n",
			hits, rows, hits == rows && rows > 0
				? "GATE PASS" : "GATE FAIL");
		fflush(stdout);
		return hits == rows && rows > 0 ? 0 : 1;
	}

	// ---- THE ENTRANCE-FIELD LABORATORY (Docs/OptimalBoardingHandoff
	// .md, Phase A; ruling 2026-08-18). Shared per-flight-event
	// extraction: replay the tape, record each long-air transfer's
	// separation state + real contact. ----
	struct EFEvent {
		int tick = 0;      // contact frame
		int sep_tick = 0;  // separation frame (flight start)
		int fidx = -1;
		PlayerState sep;
		// Boundary control-history state at separation (Invariant 9:
		// the dwell law crosses operator seams) - measured from the
		// tape's smove stream. Age is capped at the dwell gap (exact
		// state reduction); raw_age keeps the uncapped value for
		// legacy-key bank migration.
		Steer::CtlState sep_ctl;
		int raw_age = 1000;
		Vec3 cp, v1;
	};

	static bool EFReplay(const World& w, const Route::Graph& g,
	                     const ReplayOpts& o, const Tape& tape,
	                     std::vector<EFEvent>* events,
	                     std::vector<Vec3>* refpts,
	                     std::vector<float>* refspd) {
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
		int air = 0;
		PlayerState sep;
		int sep_t = 0;
		Steer::CtlState sep_ctl;
		int sep_raw_age = 1000;
		signed char sm_sign = 0;
		int sm_change_t = -1000;
		if (refpts) {
			refpts->push_back(s.pos);
			refspd->push_back(Len(s.vel));
		}
		for (size_t t = 0; t < tape.frames.size(); ++t) {
			const TapeFrame& fr = tape.frames[t];
			// Strafe side from the input stream (controller
			// convention: smove < 0 = side +1).
			const signed char sg = fr.smove < -1.f ? 1
				: (fr.smove > 1.f ? -1 : 0);
			if (sg != 0) {
				if (sm_sign != 0 && sg != sm_sign)
					sm_change_t = static_cast<int>(t);
				sm_sign = sg;
			}
			TickEvents ev;
			MoveTick(s, w, o.params, fr.pitch, fr.yaw, fr.fmove,
				fr.smove, fr.umove, fr.buttons, &ev);
			if (refpts) {
				refpts->push_back(s.pos);
				refspd->push_back(Len(s.vel));
			}
			if (ev.ncontacts > 0) {
				int fidx = -1;
				for (size_t fi = 0; fi < g.faces.size(); ++fi)
					if (g.faces[fi].brush == ev.contact_brush[0]
						&& g.faces[fi].side == ev.contact_plane[0])
						fidx = static_cast<int>(fi);
				if (fidx >= 0 && air >= 8) {
					EFEvent fe;
					fe.tick = static_cast<int>(t);
					fe.sep_tick = sep_t;
					fe.fidx = fidx;
					fe.sep = sep;
					fe.sep_ctl = sep_ctl;
					fe.raw_age = sep_raw_age;
					fe.cp = ev.contact_pos[0];
					fe.v1 = ev.contact_vel[0];
					events->push_back(fe);
				}
				air = 0;
			} else if (!s.on_ground) {
				if (air == 0) {
					sep = s;
					sep_t = static_cast<int>(t);
					sep_ctl.side = sm_sign;
					// Exact state reduction (advisor): ages at or
					// beyond the dwell gap are future-equivalent -
					// cap so equal boundary states hash equal.
					const int mg = static_cast<int>(ceilf(
						(1.f / o.params.dt)
						/ o.params.strafe_rate_max));
					const int raw_age = static_cast<int>(t)
						- sm_change_t;
					sep_ctl.age = raw_age > mg ? mg : raw_age;
					sep_raw_age = raw_age;
				}
				air++;
			} else {
				air = 0;
			}
		}
		return !events->empty();
	}

	// ---- THE WITNESS BANK v2 (ruling 2026-08-18): entries are keyed
	// by canonical start-state hash + map/params hashes + model/law
	// versions + operator + event. A witness is a floor ONLY under
	// the exact context that produced it. `src` records the finder
	// (production vs diagnostic): diagnostic floors are real physics,
	// but the accessibility gate requires production to refind them
	// cold, and banked schedules are NEVER handed to the search as
	// seeds. ----
	struct EFBankEntry {
		float H = -1e30f;
		int horizon = 0;
		std::string src;
		// FULL raw start state + realized contact (advisor: entries
		// are witnesses, not face floors; the raw state enables
		// replay-and-reindex migration when key semantics change,
		// and the realized contact gives region/exact queries their
		// correct domain).
		Vec3 spos, svel;
		int sducked = 0, sside = 0, sage = 1000;
		Vec3 cp;
		unsigned long long whash = 0;
		std::vector<signed char> side;
		std::vector<float> cosa;
	};

	static unsigned long long EFFnv64(const void* data, size_t n,
		unsigned long long h = 1469598103934665603ULL) {
		const unsigned char* b =
			static_cast<const unsigned char*>(data);
		for (size_t i = 0; i < n; ++i) {
			h ^= b[i];
			h *= 1099511628211ULL;
		}
		return h;
	}

	static unsigned long long EFStateHash(const PlayerState& s,
		const Steer::CtlState& c) {
		unsigned long long h = EFFnv64(&s.pos, sizeof(s.pos));
		h = EFFnv64(&s.vel, sizeof(s.vel), h);
		const int flags[4] = { s.ducked ? 1 : 0, s.hull_state,
			static_cast<int>(c.side), c.age };
		return EFFnv64(flags, sizeof(flags), h);
	}

	static unsigned long long EFParamsHash(const MoveParams& p) {
		const float v[9] = { p.dt, p.gravity, p.airaccelerate,
			p.air_speed_cap, p.maxspeed, p.accelerate, p.friction,
			p.stopspeed, p.strafe_rate_max };
		return EFFnv64(v, sizeof(v));
	}

	static unsigned long long EFFileHash(const std::string& path) {
		FILE* f = nullptr;
		if (fopen_s(&f, path.c_str(), "rb") != 0 || !f)
			return 0;
		unsigned long long h = 1469598103934665603ULL;
		unsigned char buf[65536];
		size_t n;
		while ((n = fread(buf, 1, sizeof(buf), f)) > 0)
			h = EFFnv64(buf, n, h);
		fclose(f);
		return h;
	}

	static std::string EFBankKey(const char* op,
		unsigned long long sh, unsigned long long mh,
		unsigned long long ph, int tick, int face) {
		char b[256];
		snprintf(b, sizeof(b), "BANK2 %s st%016llx mp%016llx "
			"pr%016llx %s %s %s t%d_f%d", op, sh, mh, ph,
			kEngineModelVersion, kControlLawVersion,
			kCollisionLawVersion, tick, face);
		return b;
	}

	static void EFBankLoad(const std::string& path,
		std::map<std::string, EFBankEntry>* bank) {
		FILE* f = nullptr;
		if (fopen_s(&f, path.c_str(), "r") != 0 || !f)
			return;
		char line[512];
		while (fgets(line, sizeof(line), f)) {
			std::string ln = line;
			while (!ln.empty()
				&& (ln.back() == '\n' || ln.back() == '\r'))
				ln.pop_back();
			if (ln.rfind("BANK2 ", 0) != 0)
				continue;
			const size_t p1 = ln.find('|');
			if (p1 == std::string::npos)
				continue;
			EFBankEntry be;
			int n = 0;
			char srcbuf[32] = { 0 };
			const int got = sscanf_s(ln.c_str() + p1 + 1,
				"%31[^|]|%f|%d|%d|%f %f %f|%f %f %f|%d %d %d|"
				"%f %f %f|%llx",
				srcbuf, static_cast<unsigned>(sizeof(srcbuf)),
				&be.H, &be.horizon, &n, &be.spos.X, &be.spos.Y,
				&be.spos.Z, &be.svel.X, &be.svel.Y, &be.svel.Z,
				&be.sducked, &be.sside, &be.sage, &be.cp.X,
				&be.cp.Y, &be.cp.Z, &be.whash);
			if (got < 4 || n < 0 || n > 4096)
				continue;
			be.src = srcbuf;
			be.side.resize(static_cast<size_t>(n));
			be.cosa.resize(static_cast<size_t>(n));
			bool ok2 = true;
			for (int i = 0; i < n; ++i) {
				int sv;
				float cv;
				if (fscanf_s(f, "%d %f", &sv, &cv) != 2) {
					ok2 = false;
					break;
				}
				be.side[static_cast<size_t>(i)] =
					static_cast<signed char>(sv);
				be.cosa[static_cast<size_t>(i)] = cv;
			}
			if (fgets(line, sizeof(line), f)) { /* eat newline */ }
			if (!ok2)
				break;
			(*bank)[ln.substr(0, p1)] = be;
		}
		fclose(f);
	}

	static void EFBankSave(const std::string& path,
		const std::map<std::string, EFBankEntry>& bank) {
		FILE* f = nullptr;
		if (fopen_s(&f, path.c_str(), "w") != 0 || !f)
			return;
		for (const auto& kv : bank) {
			fprintf(f, "%s|%s|%.1f|%d|%d|%.4f %.4f %.4f|%.4f %.4f "
				"%.4f|%d %d %d|%.2f %.2f %.2f|%016llx\n",
				kv.first.c_str(), kv.second.src.c_str(),
				kv.second.H, kv.second.horizon,
				static_cast<int>(kv.second.side.size()),
				kv.second.spos.X, kv.second.spos.Y,
				kv.second.spos.Z, kv.second.svel.X,
				kv.second.svel.Y, kv.second.svel.Z,
				kv.second.sducked, kv.second.sside,
				kv.second.sage, kv.second.cp.X, kv.second.cp.Y,
				kv.second.cp.Z, kv.second.whash);
			for (size_t i = 0; i < kv.second.side.size(); ++i)
				fprintf(f, "%d %.6f\n",
					static_cast<int>(kv.second.side[i]),
					kv.second.cosa[i]);
		}
		fclose(f);
	}

	// efield: THE PHASE-A FIELD REPORT. For every flight event of the
	// tape: build the witness-backed entrance field from the real
	// separation state, then measure it against (1) the human's own
	// arrival at their strike cell - the family must cover it, (2) the
	// Tier-0 bound field - H must sit under the certified bound, and
	// (3) the tangent survey - tangent_gap counterexamples reported,
	// never averaged away. Writes a heat + winning-heading-arrow
	// report. --dwellcost adds the no-flip-gap variant; --witnesscheck
	// replays every stored witness and demands the recorded H.
	int CmdEField(const std::string& map_path, const ReplayOpts& o,
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
			printf("efield: %s\n", err.c_str());
			return 1;
		}
		Tape tape;
		if (!LoadTas(tape_path, tape, &err)) {
			printf("efield: %s\n", err.c_str());
			return 1;
		}
		std::vector<EFEvent> events;
		std::vector<Vec3> refpts;
		std::vector<float> refspd;
		if (!EFReplay(w, g, o, tape, &events, &refpts, &refspd)) {
			printf("efield: no flight events in tape\n");
			return 1;
		}
		SearchLog::Sink sink;
		sink.gravity = o.params.gravity;
		sink.wish_rate = o.params.air_speed_cap * o.params.air_speed_cap;
		sink.AddRef("REF " + tape.map, refpts, &refspd);
		int red_flags = 0;
		for (const EFEvent& fe : events) {
			const Route::Face& fc = g.faces[fe.fidx];
			Entrance::Opts eo;
			eo.grid = o.ef_grid;
			eo.entry_ctl = fe.sep_ctl;
			const clock_t c0 = clock();
			Entrance::FieldMap em = Entrance::Build(fe.sep, w,
				o.params, g, fe.fidx, eo);
			const double ms = 1e3
				* static_cast<double>(clock() - c0) / CLOCKS_PER_SEC;
			if (em.face < 0 || em.cells_verified == 0) {
				printf("efield: t%4d f%d EMPTY FIELD (traced %d, "
					"flights %d)\n", fe.tick, fe.fidx, em.traced,
					em.flights);
				red_flags++;
				continue;
			}
			const size_t ncell = static_cast<size_t>(em.nu)
				* static_cast<size_t>(em.nv);
			int src_ref = 0;
			for (int b2 = 0; b2 < 2; ++b2)
				for (size_t i = 0; i < ncell; ++i)
					if (em.rec[b2][i].verified
						&& em.rec[b2][i].source == 1)
						src_ref++;
			printf("efield: t%4d f%d | %dx%d cells | feas %d verif %d "
				"(%d ref-found) | traces %d flights %d | contact-"
				"assisted rejected %d | H %.0fk..%.0fk | %.0f ms\n",
				fe.tick, fe.fidx, em.nu, em.nv, em.cells_feasible,
				em.cells_verified, src_ref, em.traced, em.flights,
				em.contact_assisted,
				em.H_lo / 1e3f, em.H_hi / 1e3f, ms);
			// (1) THE HUMAN COVERAGE ROW: the family must offer, at
			// the human's own cell, at least what the human achieved.
			{
				const Vec3 dq = fe.cp - em.origin;
				int iu = static_cast<int>(Dot(dq, em.ud) / em.du
					+ 0.5f);
				int iv = static_cast<int>(Dot(dq, em.vd) / em.dv
					+ 0.5f);
				if (iu < 0) iu = 0;
				if (iu >= em.nu) iu = em.nu - 1;
				if (iv < 0) iv = 0;
				if (iv >= em.nv) iv = em.nv - 1;
				const int br = fe.v1.Z > 0.f ? 1 : 0;
				const Entrance::Rec& hr = em.rec[br][
					static_cast<size_t>(iv) * em.nu + iu];
				Vec3 vpost;
				Fn::ClipVelocity(fe.v1, fc.n, &vpost);
				const float eh = Dot(vpost, vpost)
					+ 2.f * o.params.gravity
					* (fe.cp.Z - fc.zmin);
				// Percentile of the human's cell among verified.
				int under = 0, tot = 0;
				for (int b2 = 0; b2 < 2; ++b2)
					for (size_t i = 0; i < ncell; ++i) {
						const Entrance::Rec& r = em.rec[b2][i];
						if (!r.verified)
							continue;
						tot++;
						if (hr.verified && r.H <= hr.H)
							under++;
					}
				if (!hr.verified) {
					printf("efield:   HUMAN CELL (%d,%d br%d) "
						"UNVERIFIED by the family (traced: %s) | "
						"human E %.0fk dot %.1f <- FAMILY HOLE\n",
						iu, iv, br, hr.feasible ? "yes" : "NO",
						eh / 1e3f, Dot(fe.v1, fc.n));
					red_flags++;
				} else {
					const bool covers = hr.H >= eh - 2000.f;
					printf("efield:   human cell (%d,%d br%d): H "
						"%.0fk vs human E %.0fk (dot %.1f vs their "
						"%.1f) %s | cell rank %.2f | field max "
						"%.0fk\n", iu, iv, br, hr.H / 1e3f,
						eh / 1e3f, hr.dot, Dot(fe.v1, fc.n),
						covers ? "COVERS" : "<- BELOW HUMAN "
						"(family gap)", tot > 1
							? static_cast<float>(under)
								/ static_cast<float>(tot) : 1.f,
						em.H_hi / 1e3f);
					if (!covers)
						red_flags++;
				}
			}
			// (2) THE BOUND SANDWICH: every replayed H must sit under
			// the Tier-0 certified bound at its cell; and a verified
			// cell the bound calls unreachable is a false cull.
			{
				Field::FaceMap t0 = Field::Compute(fe.sep.pos,
					fe.sep.vel, fc, o.params, fe.sep.ducked,
					o.ef_grid, 300);
				int viol = 0, false_cull = 0, ceil_viol = 0;
				float worst = 0.f;
				for (int b2 = 0; b2 < 2; ++b2)
					for (size_t i = 0; i < ncell; ++i) {
						const Entrance::Rec& r = em.rec[b2][i];
						if (!r.verified)
							continue;
						if (i >= t0.samples.size())
							continue;
						const Field::Sample& sm = t0.samples[i];
						if (!sm.reachable) {
							false_cull++;
							// CEILING RE-CERT (advisor): the reach
							// predicate is disproven - verify the ENERGY
							// ceiling independently on these cells via
							// the loss-free ballistic closed forms.
							{
								const float dzq = r.cp.Z - fe.sep.pos.Z;
								const float disc = fe.sep.vel.Z
									* fe.sep.vel.Z
									- 2.f * o.params.gravity * dzq;
								float ue = -1e30f;
								if (disc >= 0.f) {
									const float sq2 = sqrtf(disc);
									const float g2 = o.params.gravity
										* o.params.dt;
									const float rts[2] = {
										(fe.sep.vel.Z - sq2) / g2,
										(fe.sep.vel.Z + sq2) / g2 };
									const float s02 = Len2D(fe.sep.vel);
									for (int ri2 = 0; ri2 < 2; ++ri2) {
										const int nn = static_cast<int>(
											rts[ri2]);
										if (nn < 1 || nn > 400)
											continue;
										const float sa2 = Envelope::SMax(
											s02, nn, o.params);
										const float va2 = Envelope::VzAfter(
											fe.sep.vel.Z, nn, o.params);
										const float e2 = sa2 * sa2 + va2 * va2
											+ 2.f * o.params.gravity
											* (r.cp.Z - fc.zmin);
										if (e2 > ue)
											ue = e2;
									}
								}
								if (ue < -1e29f || r.H > ue + 2000.f)
									ceil_viol++;
							}
							continue;
						}
						const float over = r.H - sm.e_eff;
						if (over > 2000.f) {
							viol++;
							if (over > worst)
								worst = over;
						}
					}
				printf("efield:   bound sandwich: %d over-bound "
					"(worst +%.0fk) | %d bound-unreachable (reach "
					"predicate uncertified) | ceiling re-cert on "
					"those: %d violations%s\n", viol, worst / 1e3f,
					false_cull, ceil_viol, viol || ceil_viol
						? " <- CEILING BROKEN" : "");
				if (viol || ceil_viol)
					red_flags++;
			}
			// (3) THE TANGENT SURVEY: counterexamples, not averages.
			{
				float gmax = -1e30f;
				int gi = -1, gb = 0, gcount = 0, tancells = 0;
				for (int b2 = 0; b2 < 2; ++b2)
					for (size_t i = 0; i < ncell; ++i) {
						const Entrance::Rec& r = em.rec[b2][i];
						if (!r.verified || !r.tan_ok)
							continue;
						tancells++;
						const float gap = r.H - r.tan_E;
						if (gap > 2000.f)
							gcount++;
						if (gap > gmax) {
							gmax = gap;
							gi = static_cast<int>(i);
							gb = b2;
						}
					}
				if (gi >= 0) {
					const Entrance::Rec& r = em.rec[gb][
						static_cast<size_t>(gi)];
					printf("efield:   tangent survey: %d cells with "
						"a tangent arrival | %d where lossy wins "
						"(gap > 2k) | max gap %.0fk at (%d,%d br%d)"
						": H %.0fk dot %.1f vs tan %.0fk dot %.1f"
						"\n", tancells, gcount, gmax / 1e3f,
						gi % em.nu, gi / em.nu, gb, r.H / 1e3f,
						r.dot, r.tan_E / 1e3f, r.tan_dot);
				} else {
					// Case-D discriminator: softest achieved |dot|
					// vs the closed-form minimum possible at those
					// arrival states. min_pos ~ 0 with a hard
					// min_ach = the walker/realization fell short;
					// min_pos > eps = no tangent EXISTS here
					// (handoff Case D) and the empty survey is the
					// true answer.
					float min_ach = 1e9f, min_pos = 1e9f;
					for (int b2 = 0; b2 < 2; ++b2)
						for (size_t i = 0; i < ncell; ++i) {
							const Entrance::Rec& r =
								em.rec[b2][i];
							if (!r.verified)
								continue;
							if (-r.dot < min_ach)
								min_ach = -r.dot;
							const float mp =
								Board::MinApproachDot(r.s2d,
									r.vz, fc.n);
							if (mp < min_pos)
								min_pos = mp;
						}
					printf("efield:   tangent survey: none under "
						"|dot| <= %.0f | softest achieved %.1f | "
						"closed-form min possible %.1f %s\n",
						Entrance::kTanEps, min_ach, min_pos,
						min_pos > Entrance::kTanEps
							? "(CANDIDATE Case D - not proven until "
							  "the reference frontier certifies)"
							: "(tangent exists - realization "
							  "fell short)");
				}
			}
			// --witnesscheck: replay every stored witness; the
			// recorded H must be the replayed H (determinism).
			if (o.ef_witnesscheck) {
				int mism = 0, checked = 0;
				std::vector<float> prof;
				Air::Target vt;
				vt.face = fe.fidx;
				vt.dot_cap = 2000.f;
				for (int b2 = 0; b2 < 2; ++b2)
					for (size_t i = 0; i < ncell; ++i) {
						const Entrance::Rec& r = em.rec[b2][i];
						if (!r.verified)
							continue;
						checked++;
						Air::Result cr;
						vt.aim = r.cp;
						if (!r.wside.empty()) {
							vt.max_ticks = r.horizon;
							cr = Air::FlyWishSchedule(fe.sep, w,
								o.params, vt, g, r.wside,
								r.wcosa, r.horizon);
						} else {
							int hz;
							if (!r.knots.empty()) {
								prof = r.knots;
								hz = r.horizon;
							} else {
								Entrance::WitnessProfile(fe.sep,
									o.params, r.psi1, r.psi2,
									r.split, r.prof_n, &prof);
								hz = r.prof_n + 8;
							}
							vt.max_ticks = hz;
							cr = Air::FlyHeadingSpline(fe.sep, w,
								o.params, vt, g, prof, hz);
						}
						if (!cr.hit) {
							mism++;
							continue;
						}
						const float H2 = Dot(cr.end_state.vel,
							cr.end_state.vel) + 2.f
							* o.params.gravity
							* (cr.pos.Z - fc.zmin);
						if (fabsf(H2 - r.H) > 1.f)
							mism++;
					}
				printf("efield:   witnesscheck: %d/%d replays "
					"reproduce H%s\n", checked - mism, checked,
					mism ? " <- NONDETERMINISM" : "");
				if (mism)
					red_flags++;
			}
			// --dwellcost: the unrestricted-reversal variant.
			if (o.ef_dwellcost) {
				Entrance::Opts eo2 = eo;
				eo2.dwell_free = true;
				Entrance::FieldMap ef = Entrance::Build(fe.sep, w,
					o.params, g, fe.fidx, eo2);
				int improved = 0;
				float dmax = 0.f;
				for (int b2 = 0; b2 < 2; ++b2)
					for (size_t i = 0; i < ncell
						&& i < static_cast<size_t>(ef.nu)
							* static_cast<size_t>(ef.nv); ++i) {
						const Entrance::Rec& a = em.rec[b2][i];
						const Entrance::Rec& b = ef.rec[b2][i];
						if (!b.verified)
							continue;
						const float d = b.H - (a.verified
							? a.H : -1e30f);
						if (a.verified && d > 2000.f)
							improved++;
						if (a.verified && d > dmax)
							dmax = d;
					}
				if (ef.cells_verified > 0)
					printf("efield:   dwellcost: unrestricted "
						"H_hi %.0fk vs %.0fk | %d cells improve "
						"> 2k (max +%.0fk)\n", ef.H_hi / 1e3f,
						em.H_hi / 1e3f, improved, dmax / 1e3f);
				else
					printf("efield:   dwellcost: unrestricted "
						"variant verified nothing here\n");
			}
			// Render: heat = H (verified cells), arrows = winning
			// terminal headings, markers at the argmax + human strike.
			for (size_t i = 0; i < ncell; ++i) {
				const Entrance::Rec* r = &em.rec[0][i];
				if (em.rec[1][i].verified && (!r->verified
					|| em.rec[1][i].H > r->H))
					r = &em.rec[1][i];
				if (!r->verified)
					continue;
				const int iu = static_cast<int>(i)
					% em.nu;
				const int iv = static_cast<int>(i) / em.nu;
				const Vec3 q = em.origin
					+ Scale(em.ud, static_cast<float>(iu) * em.du)
					+ Scale(em.vd, static_cast<float>(iv) * em.dv);
				const float v01 = em.H_hi > em.H_lo
					? (r->H - em.H_lo) / (em.H_hi - em.H_lo) : 1.f;
				const Vec3 lift = Scale(fc.n, 2.f);
				const Vec3 c00 = q + lift
					- Scale(em.ud, em.du * 0.5f)
					- Scale(em.vd, em.dv * 0.5f);
				const Vec3 c10 = c00 + Scale(em.ud, em.du);
				const Vec3 c01 = c00 + Scale(em.vd, em.dv);
				const Vec3 c11 = c10 + Scale(em.vd, em.dv);
				sink.AddHeat(c00, c10, c11, v01);
				sink.AddHeat(c00, c11, c01, v01);
				// The winning-heading arrow (in-plane projection).
				Vec3 dir(cosf(r->theta), sinf(r->theta), 0.f);
				dir = dir - Scale(fc.n, Dot(dir, fc.n));
				const float dl = Len(dir);
				if (dl > 1e-3f) {
					dir = Scale(dir, 1.f / dl);
					const Vec3 a = q + Scale(fc.n, 4.f);
					const Vec3 tip = a + Scale(dir, em.du * 0.85f);
					const Vec3 side = Cross(fc.n, dir);
					std::vector<Vec3> arrow;
					arrow.push_back(a);
					arrow.push_back(tip);
					arrow.push_back(tip - Scale(dir, em.du * 0.3f)
						+ Scale(side, em.du * 0.18f));
					arrow.push_back(tip);
					arrow.push_back(tip - Scale(dir, em.du * 0.3f)
						- Scale(side, em.du * 0.18f));
					char nm[48];
					snprintf(nm, sizeof(nm), "EF arrows f%d",
						fe.fidx);
					sink.AddRef(nm, arrow);
				}
			}
			{
				std::vector<Vec3> cross;
				cross.push_back(fe.cp + Vec3(-16.f, 0.f, 0.f));
				cross.push_back(fe.cp + Vec3(16.f, 0.f, 0.f));
				cross.push_back(fe.cp);
				cross.push_back(fe.cp + Vec3(0.f, -16.f, 0.f));
				cross.push_back(fe.cp + Vec3(0.f, 16.f, 0.f));
				sink.AddRef("human strikes", cross);
			}
		}
		CreateDirectoryA("Output", nullptr);
		CreateDirectoryA("Output\\reports", nullptr);
		time_t now = time(nullptr);
		struct tm tmv;
		localtime_s(&tmv, &now);
		char st[64];
		strftime(st, sizeof(st), "%m%d-%H%M%S", &tmv);
		const std::string vp = std::string("Output\\reports\\efield_")
			+ st + ".html";
		std::string verr;
		if (SearchLog::WriteHtml(vp, w, g, sink,
			"entrance field (witness-backed)", &verr))
			printf("efield: report -> %s\n", vp.c_str());
		else
			printf("efield: VIZ WRITE FAILED: %s\n", verr.c_str());
		printf("efield: %d red flags | %s\n", red_flags,
			red_flags == 0 ? "CLEAN" : "FINDINGS ABOVE");
		fflush(stdout);
		return 0;
	}

	// fieldexact: THE FAMILY-COMPLETENESS GATE (the ruling's added
	// requirement). H(Q) is defined over ALL admissible controls; the
	// production family is certified only if a BROADER reference
	// search (5-knot heading splines - multi-turn curves the two-hold
	// family cannot express) fails to beat it. Reference flights are
	// engine flights; scores are replayed post-board energies.
	int CmdFieldExact(const std::string& map_path, const ReplayOpts& o,
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
			printf("fieldexact: %s\n", err.c_str());
			return 1;
		}
		Tape tape;
		if (!LoadTas(tape_path, tape, &err)) {
			printf("fieldexact: %s\n", err.c_str());
			return 1;
		}
		std::vector<EFEvent> events;
		if (!EFReplay(w, g, o, tape, &events, nullptr, nullptr)) {
			printf("fieldexact: no flight events in tape\n");
			return 1;
		}
		float worst_gap = 0.f;
		int rows = 0, sat = 0;
		for (const EFEvent& fe : events) {
			const Route::Face& fc = g.faces[fe.fidx];
			Entrance::Opts eo;
			eo.grid = o.ef_grid;
			Entrance::FieldMap em = Entrance::Build(fe.sep, w,
				o.params, g, fe.fidx, eo);
			if (em.cells_verified == 0)
				continue;
			const size_t ncell = static_cast<size_t>(em.nu)
				* static_cast<size_t>(em.nv);
			// Sample: top cells by H + an even spread.
			struct CellRef { int br; int idx; float H; };
			std::vector<CellRef> verified;
			for (int b2 = 0; b2 < 2; ++b2)
				for (size_t i = 0; i < ncell; ++i)
					if (em.rec[b2][i].verified)
						verified.push_back({ b2,
							static_cast<int>(i),
							em.rec[b2][i].H });
			std::sort(verified.begin(), verified.end(),
				[](const CellRef& a, const CellRef& b) {
					return a.H > b.H;
				});
			std::vector<CellRef> picks;
			const int k = o.ef_cells;
			for (int i = 0; i < k / 2
				&& i < static_cast<int>(verified.size()); ++i)
				picks.push_back(verified[i]);
			if (verified.size() > static_cast<size_t>(k / 2)) {
				const size_t rest = verified.size() - k / 2;
				const size_t step = rest / (k - k / 2) + 1;
				for (size_t i = k / 2; i < verified.size()
					&& static_cast<int>(picks.size()) < k;
					i += step)
					picks.push_back(verified[i]);
			}
			for (const CellRef& cref : picks) {
				const Entrance::Rec& r = em.rec[cref.br][
					static_cast<size_t>(cref.idx)];
				const int iu = cref.idx % em.nu;
				const int iv = cref.idx / em.nu;
				const Vec3 q = em.origin
					+ Scale(em.ud, static_cast<float>(iu) * em.du)
					+ Scale(em.vd, static_cast<float>(iv) * em.dv);
				// ---- The reference: the GENERAL admissible-control
				// solve (segment schedules over the dwell law) with
				// an independent budget - it must fail to beat the
				// stored H for the family to certify. ----
				int flights = 0;
				Entrance::RefResult rr = Entrance::RefSolve(fe.sep,
					w, o.params, g, fe.fidx, q, eo.grid * 0.9f,
					r.n, 700, false, &flights, fc.zmin,
					fe.sep_ctl);
				const float H_ref = rr.ok ? rr.H : -1e30f;
				const float ref_dot = rr.ok ? rr.flight.dot : 0.f;
				rows++;
				const float gap = H_ref > -1e29f
					? H_ref - r.H : -1e30f;
				const char* verdict;
				if (H_ref < -1e29f)
					verdict = "REF NO STRIKE (production stands)";
				else if (gap > 2000.f)
					verdict = "<- FAMILY GAP";
				else if (gap < -2000.f)
					verdict = "REF SHORT (production stands)";
				else
					verdict = "SATURATED";
				if (gap <= 2000.f)
					sat++;
				if (gap > worst_gap)
					worst_gap = gap;
				printf("fieldexact: t%4d f%d cell (%d,%d br%d) | "
					"H_prod %.0fk (dot %.1f) vs H_ref %s (dot "
					"%.1f) | gap %s | %d flights | %s\n",
					fe.tick, fe.fidx, iu, iv, cref.br,
					r.H / 1e3f, r.dot,
					H_ref > -1e29f
						? (std::to_string(static_cast<int>(
							H_ref / 1e3f)) + "k").c_str()
						: "none",
					ref_dot,
					H_ref > -1e29f
						? (std::to_string(static_cast<int>(
							gap)).c_str())
						: "n/a",
					flights, verdict);
			}
		}
		printf("fieldexact: %d/%d cells saturated | worst family gap "
			"%.0f | %s\n", sat, rows, worst_gap,
			rows > 0 && sat == rows
				? "FAMILY HOLDS THE FRONTIER (so far)"
				: "FAMILY GAPS FOUND - do not certify");
		fflush(stdout);
		return rows > 0 && sat == rows ? 0 : 1;
	}

	// humanexact: THE EXACT-POINT LOWER-BOUND GATE (ruling 2026-08-18).
	// The human tape is QC only, never a seed - but at its exact
	// contact coordinate it is a constructive feasible witness: from
	// the same separation state, the admissible solve must deliver
	//     H(Q_h) >= E_h  (and ordinarily EXCEED it).
	// H(Q_h) < E_h is definitive incompleteness - no theoretical
	// argument survives a witness. No cell-center ambiguity: the solve
	// targets the literal recorded contact. Also reports whether the
	// human's own inputs violate the dwell law the solver is
	// restricted to, and runs the explicit FastestTangent solve at the
	// same point (tangent "none" stays CANDIDATE Case D only).
	int CmdHumanExact(const std::string& map_path, const ReplayOpts& o,
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
			printf("humanexact: %s\n", err.c_str());
			return 1;
		}
		Tape tape;
		if (!LoadTas(tape_path, tape, &err)) {
			printf("humanexact: %s\n", err.c_str());
			return 1;
		}
		std::vector<EFEvent> events;
		if (!EFReplay(w, g, o, tape, &events, nullptr, nullptr)) {
			printf("humanexact: no flight events in tape\n");
			return 1;
		}
		// THE WITNESS BANK v2: context-hashed monotonic floors
		// (state + map + params + model/law versions).*
		std::map<std::string, EFBankEntry> bank;
		const std::string bank_path = tape_path + ".bank";
		EFBankLoad(bank_path, &bank);
		const unsigned long long maph = EFFileHash(map_path);
		const unsigned long long prh = EFParamsHash(o.params);
		const int min_gap = static_cast<int>(
			ceilf((1.f / o.params.dt) / o.params.strafe_rate_max));
		int rows = 0, pass = 0, na = 0;
		for (const EFEvent& fe : events) {
			const Route::Face& fc = g.faces[fe.fidx];
			Vec3 vpost;
			Fn::ClipVelocity(fe.v1, fc.n, &vpost);
			const float eh = Dot(vpost, vpost)
				+ 2.f * o.params.gravity
				* (fe.cp.Z - fc.zmin);
			const int n_h = fe.tick - fe.sep_tick;
			int flights = 0;
			// One frontier solve yields BOTH BestBoard and
			// FastestTangent (the ruling's one-frontier design).
			// EXACT-POINT QUERY: continue the tolerance to this
			// command's own stated bar (4u) so H(Q_h) >= E_h is
			// finally evaluated AT Q_h instead of up to 16u away.
			Entrance::RefTune hx_tune;
			hx_tune.precision = 4.f;
			Entrance::RefResult rr = Entrance::RefSolve(fe.sep, w,
				o.params, g, fe.fidx, fe.cp, 16.f, n_h, 2000,
				false, &flights, fc.zmin, fe.sep_ctl, nullptr,
				nullptr, nullptr, &hx_tune);
			// The human's own reversal cadence vs the dwell law.
			int reversals = 0, viol = 0, mind = 100000;
			{
				int last_sign = 0, last_t = -100000;
				for (int t2 = fe.sep_tick + 1; t2 <= fe.tick
					&& t2 < static_cast<int>(tape.frames.size());
					++t2) {
					const float sm = tape.frames[
						static_cast<size_t>(t2)].smove;
					const int sg = sm > 1.f ? 1
						: (sm < -1.f ? -1 : 0);
					if (!sg)
						continue;
					if (last_sign && sg != last_sign) {
						reversals++;
						const int gp = t2 - last_t;
						if (gp < mind)
							mind = gp;
						if (gp < min_gap)
							viol++;
						last_t = t2;
					} else if (!last_sign) {
						last_t = t2;
					}
					last_sign = sg;
				}
			}
			// Classification (advisor 2026-08-18): a witness that
			// violates the dwell law is OUTSIDE the solver's
			// admissible space - N/A for the lower bound, never a
			// mandatory failure. Admissible-but-not-found = FAIL.
			const bool outside_law = viol > 0;
			const bool covers = rr.ok && rr.H >= eh - 2000.f;
			if (outside_law) {
				na++;
			} else {
				rows++;
				if (covers)
					pass++;
			}
			const char* verdict = outside_law
				? "N/A - HUMAN OUTSIDE CONTROL LAW"
				: (covers ? (rr.ok && rr.H > eh + 2000.f
					? "PASS (exceeds)" : "PASS (matches)")
					: "FAIL - ADMISSIBLE WITNESS NOT MATCHED");
			if (rr.ok)
				printf("humanexact: t%4d f%d | E_h %.0fk (dot %.1f)"
					" | L(near16u) %.0fk (dot %.1f, %.0fu off, %d "
					"flights) | gap %+.0fk | %s\n",
					fe.tick, fe.fidx, eh / 1e3f,
					Dot(fe.v1, fc.n), rr.H / 1e3f, rr.flight.dot,
					Len(rr.flight.pos - fe.cp), flights,
					(rr.H - eh) / 1e3f, verdict);
			else
				printf("humanexact: t%4d f%d | E_h %.0fk (dot %.1f)"
					" | H(Q_h) NONE in %d flights (no clean-air "
					"strike within 16u) | %s\n",
					fe.tick, fe.fidx, eh / 1e3f,
					Dot(fe.v1, fc.n), flights, verdict);
			// Near/exact split (advisor: a 7u-near strike is QC,
			// not the literal H(Q_h) >= E_h floor test).
			if (rr.ok) {
				const float doff = Len(rr.flight.pos - fe.cp);
				printf("humanexact:   near-contact (<=16u): %.0fk"
					" | exact-point (<=4u): %s\n", rr.H / 1e3f,
					doff <= 4.f ? "RESOLVED at the literal "
						"coordinate" : "floor unresolved (best "
						"strike off by more than 4u)");
			}
			if (rr.tan_ok)
				printf("humanexact:   tangent at Q_h (same "
					"frontier): %.0fk (dot %.1f) | delta_tan "
					"%+.0fk\n", rr.tan_H / 1e3f,
					rr.tan_flight.dot,
					((rr.ok ? rr.H : eh) - rr.tan_H) / 1e3f);
			else
				printf("humanexact:   tangent at Q_h: none found "
					"(CANDIDATE Case D at this point - not "
					"proven)\n");
			printf("humanexact:   human inputs: %d reversals, min "
				"dwell %s ticks, %d under the %d-tick law%s\n",
				reversals, reversals ? std::to_string(mind).c_str()
					: "n/a", viol, min_gap, viol
					? " <- the human uses controls outside the "
					  "solver's restriction" : "");
			// Bank: replay the stored floor, verify monotonicity,
			// absorb improvements.
			{
				const std::string bkey = EFBankKey("air-entry",
					EFStateHash(fe.sep, fe.sep_ctl), maph, prh,
					fe.tick, fe.fidx);
				// LEGACY-KEY MIGRATION (the age-cap change altered
				// state hashes and orphaned floors - the measured
				// lesson: key-semantics changes need migration, and
				// entries should carry their full start state).
				{
					Steer::CtlState legacy = fe.sep_ctl;
					legacy.age = fe.raw_age;
					const std::string lkey = EFBankKey(
						"air-entry", EFStateHash(fe.sep,
							legacy), maph, prh, fe.tick,
						fe.fidx);
					auto lit = bank.find(lkey);
					if (lkey != bkey && lit != bank.end()) {
						auto cit = bank.find(bkey);
						if (cit == bank.end()
							|| lit->second.H > cit->second.H)
							bank[bkey] = lit->second;
						bank.erase(lkey);
						printf("humanexact:   bank: migrated "
							"legacy-key floor %.0fk\n",
							bank[bkey].H / 1e3f);
					}
				}
				float bank_floor = -1e30f;
				auto bit = bank.find(bkey);
				if (bit != bank.end()) {
					Air::Target bt;
					bt.face = fe.fidx;
					bt.dot_cap = 3000.f;
					bt.aim = fe.cp;
					bt.max_ticks = bit->second.horizon;
					Air::Result br2 = Air::FlyWishSchedule(fe.sep,
						w, o.params, bt, g, bit->second.side,
						bit->second.cosa, bit->second.horizon);
					if (br2.hit && br2.dot < 0.f
						&& br2.struck_brush < 0) {
						bank_floor = Dot(br2.end_state.vel,
							br2.end_state.vel) + 2.f
							* o.params.gravity
							* (br2.pos.Z - fc.zmin);
						if (bank_floor < bit->second.H - 1000.f)
							printf("humanexact:   BANK "
								"MONOTONICITY BREAK: replay %.0fk"
								" < recorded %.0fk\n",
								bank_floor / 1e3f,
								bit->second.H / 1e3f);
					} else {
						printf("humanexact:   BANK MONOTONICITY "
							"BREAK: stored witness no longer "
							"strikes\n");
					}
				}
				if (rr.ok && rr.H > bank_floor + 1.f
					&& (bit == bank.end()
						|| rr.H > bit->second.H + 1.f)) {
					EFBankEntry nb;
					nb.H = rr.H;
					nb.src = "production";
					nb.horizon = rr.horizon;
					nb.side = rr.wside;
					nb.cosa = rr.wcosa;
					nb.spos = fe.sep.pos;
					nb.svel = fe.sep.vel;
					nb.sducked = fe.sep.ducked ? 1 : 0;
					nb.sside = fe.sep_ctl.side;
					nb.sage = fe.sep_ctl.age;
					nb.cp = rr.flight.pos;
					nb.whash = EFFnv64(rr.wcosa.data(),
						rr.wcosa.size() * sizeof(float),
						EFFnv64(rr.wside.data(),
							rr.wside.size()));
					bank[bkey] = nb;
					printf("humanexact:   bank: new floor %.0fk "
						"recorded\n", rr.H / 1e3f);
				} else if (bank_floor > -1e29f) {
					printf("humanexact:   bank: floor %.0fk "
						"stands\n", bank_floor / 1e3f);
				}
			}
		}
		EFBankSave(bank_path, bank);
		printf("humanexact: witness bank -> %s (%d entries)\n",
			bank_path.c_str(), static_cast<int>(bank.size()));
		printf("humanexact: %d/%d mandatory exact points covered "
			"(%d N/A outside the control law) | %s\n",
			pass, rows, na, rows > 0 && pass == rows
				? "LOWER BOUND HOLDS"
				: "INCOMPLETE - admissible human witnesses beat "
				  "the solver");
		fflush(stdout);
		return rows > 0 && pass == rows ? 0 : 1;
	}

	// repfit: THE REPRESENTATION-COMPLETENESS UNIT TEST (advisor
	// 2026-08-18). Before blaming the optimizer, prove the human's
	// ADMISSIBLE flights are inside the solver's control basis:
	//   U_human in U_solver, not just U_human in U_game.
	// MEASURED en route (kept as the record, WITH ITS 2026-08-19
	// CORRECTION): the heading-command channel could not reproduce
	// flights even when handed the human's realized headings (66-282u
	// misses). That was later root-caused to the CONTROLLER EMITTER's
	// wish-convention bug (requested braking turns were emitted as
	// inert wishes), NOT to any incompleteness of the heading channel.
	// The basis below is still the right one to store and replay -
	// it is the INPUT space itself:
	// per tick, wish direction as (side, cosa) about the current
	// velocity heading, executed open-loop by Air::FlyWishSchedule.
	// Rows per event: (a) the per-tick wish schedule (the basis
	// ceiling - must reproduce), (b) the segment-compressed schedule
	// (what a schedule search would fly). Never a production seed.
	int CmdRepFit(const std::string& map_path, const ReplayOpts& o,
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
			printf("repfit: %s\n", err.c_str());
			return 1;
		}
		Tape tape;
		if (!LoadTas(tape_path, tape, &err)) {
			printf("repfit: %s\n", err.c_str());
			return 1;
		}
		std::vector<EFEvent> events;
		if (!EFReplay(w, g, o, tape, &events, nullptr, nullptr)) {
			printf("repfit: no flight events in tape\n");
			return 1;
		}
		// Second replay capturing per-tick velocity + duck through
		// every frame (the flight windows index into this).
		std::vector<Vec3> vels;
		std::vector<unsigned char> ducked;
		{
			PlayerState s;
			s.pos = tape.start.origin;
			s.vel = tape.start.velocity;
			s.ducked = tape.start.ducked;
			s.hull_state = tape.start.ducked ? 1 : 0;
			s.stamina = tape.start.stamina;
			TraceResult tr;
			const float gf = w.TraceHull(s.pos,
				s.pos - Vec3(0.f, 0.f, 2.f), s.ducked, &tr);
			if (gf < 1.f && tr.brush >= 0
				&& tr.normal.Z >= o.params.walkable_z) {
				s.pos.Z -= 2.f * gf;
				s.on_ground = true;
				s.ground_brush = tr.brush;
			}
			vels.push_back(s.vel);
			ducked.push_back(s.ducked ? 1 : 0);
			for (const TapeFrame& fr : tape.frames) {
				MoveTick(s, w, o.params, fr.pitch, fr.yaw,
					fr.fmove, fr.smove, fr.umove, fr.buttons,
					nullptr);
				vels.push_back(s.vel);
				ducked.push_back(s.ducked ? 1 : 0);
			}
		}
		const int min_gap = static_cast<int>(
			ceilf((1.f / o.params.dt) / o.params.strafe_rate_max));
		int reps = 0, rows = 0;
		for (const EFEvent& fe : events) {
			const Route::Face& fc = g.faces[fe.fidx];
			const int t0 = fe.sep_tick + 1;
			const int t1 = fe.tick;
			if (t1 - t0 < 6
				|| t1 >= static_cast<int>(vels.size()))
				continue;
			// Realized turn-rate stats (vs the free rate) + duck.
			int above_rate = 0, duck_ticks = 0;
			float fmax = 0.f;
			for (int t2 = t0; t2 < t1; ++t2) {
				const Vec3& va = vels[static_cast<size_t>(t2)];
				const Vec3& vb = vels[static_cast<size_t>(t2) + 1];
				const float sa = Len2D(va);
				if (sa < 1.f || Len2D(vb) < 1.f)
					continue;
				const float dh = Steer::WrapPi(
					atan2f(vb.Y, vb.X) - atan2f(va.Y, va.X));
				Strafe::TickLaw law = Strafe::Law(o.params, sa,
					1.f, ducked[static_cast<size_t>(t2)] != 0);
				const float wr = law.TurnRad(Strafe::kPerp, 1.f);
				const float ft = wr > 1e-6f ? dh / wr : 0.f;
				if (fabsf(ft) > fmax)
					fmax = fabsf(ft);
				if (fabsf(ft) > 1.001f)
					above_rate++;
				if (ducked[static_cast<size_t>(t2)]
					!= ducked[static_cast<size_t>(t0)])
					duck_ticks++;
			}
			// THE WISH-BASIS RECONSTRUCTION: the tape's per-tick
			// wish DIRECTION re-expressed as (side, cosa) about the
			// current velocity heading - exactly what
			// FlyWishSchedule executes. Null wishes = side 0.
			std::vector<signed char> wside;
			std::vector<float> wcosa;
			int wrev = 0, wviol = 0, wmind = 100000;
			{
				int last_sign = 0, last_t = -100000;
				for (int t2 = t0; t2 < t1; ++t2) {
					const TapeFrame& fr = tape.frames[
						static_cast<size_t>(t2)];
					const float yr = fr.yaw * 0.0174532925f;
					const float wx = cosf(yr) * fr.fmove
						+ sinf(yr) * fr.smove;
					const float wy = sinf(yr) * fr.fmove
						- cosf(yr) * fr.smove;
					const Vec3& vv = vels[static_cast<size_t>(t2)];
					const float s2v = Len2D(vv);
					if (wx * wx + wy * wy < 1.f || s2v < 1.f) {
						wside.push_back(0);
						wcosa.push_back(1.f);
						continue;
					}
					const float omega = atan2f(wy, wx);
					const float h = atan2f(vv.Y, vv.X);
					// The executor's input mapping (the controller's
					// convention) realizes its wish at wh + pi in
					// WishFromInput coordinates - invert through the
					// SAME mapping, not the raw engine form.
					const float d = Steer::WrapPi(omega
						- 3.14159265f - h);
					const int sg = d >= 0.f ? 1 : -1;
					wside.push_back(static_cast<signed char>(sg));
					wcosa.push_back(cosf(d));
					const int i = t2 - t0;
					if (last_sign && sg != last_sign) {
						wrev++;
						const int gp = i - last_t;
						if (gp < wmind)
							wmind = gp;
						if (gp < min_gap)
							wviol++;
						last_t = i;
					} else if (!last_sign) {
						last_t = i;
					}
					last_sign = sg;
				}
			}
			Air::Target vt;
			vt.face = fe.fidx;
			vt.dot_cap = 3000.f;
			vt.aim = fe.cp;
			Vec3 vpost;
			Fn::ClipVelocity(fe.v1, fc.n, &vpost);
			const float eh = Dot(vpost, vpost)
				+ 2.f * o.params.gravity
				* (fe.cp.Z - fc.zmin);
			rows++;
			printf("repfit: t%4d f%d | window %d ticks | max|f| "
				"%.2f (%d above-rate) | wish reversals %d, min "
				"dwell %s (%d under law) | duck %d\n",
				fe.tick, fe.fidx, t1 - t0, fmax, above_rate,
				wrev, wrev ? std::to_string(wmind).c_str()
					: "n/a", wviol, duck_ticks);
			// (a) EXACT ROW: the per-tick wish schedule - the basis
			// ceiling. Must reproduce the flight.
			vt.max_ticks = static_cast<int>(wside.size()) + 8;
			Air::Result ex = Air::FlyWishSchedule(fe.sep, w,
				o.params, vt, g, wside, wcosa,
				static_cast<int>(wside.size()) + 8);
			bool exact_ok = false;
			if (ex.hit && ex.dot < 0.f) {
				const float H2 = Dot(ex.end_state.vel,
					ex.end_state.vel) + 2.f * o.params.gravity
					* (ex.pos.Z - fc.zmin);
				exact_ok = Len(ex.pos - fe.cp) <= 24.f
					&& fabsf(H2 - eh) <= 0.03f * eh;
				// Full terminal-state error (advisor: contact
				// distance alone is not exactness).
				printf("repfit:   per-tick wish schedule: dp %.1fu "
					"| dv (%.1f, %.1f, %.1f) | dE %+.1fk | "
					"d(n.v) %+.1f | %s\n",
					Len(ex.pos - fe.cp),
					ex.v1.X - fe.v1.X, ex.v1.Y - fe.v1.Y,
					ex.v1.Z - fe.v1.Z, (H2 - eh) / 1e3f,
					ex.dot - Dot(fe.v1, fc.n),
					exact_ok ? "EXACT-ROW OK"
						: "EXACT-ROW DIVERGED <- executor gap");
				printf("repfit:   basis_representable %s | "
					"6tick_admissible %s\n",
					exact_ok ? "YES" : "NO",
					wviol == 0 ? "YES" : "NO (dwell violation "
						"in the human witness)");
			} else {
				printf("repfit:   per-tick wish schedule: %s "
					"(closest %.0fu) <- executor gap\n",
					ex.hit ? "contact-assisted"
						: (ex.grounded ? "grounded" : "missed"),
					ex.miss_dist < 1e8f ? ex.miss_dist : -1.f);
			}
			// (b) SEGMENT ROW: compress (side, cosa) into segments
			// (boundaries at side changes + greedy splits on cosa
			// error, cap ~24) and fly - the shape a schedule search
			// would actually discover.
			{
				std::vector<int> cuts;
				cuts.push_back(0);
				for (size_t i = 1; i < wside.size(); ++i)
					if (wside[i] != wside[i - 1])
						cuts.push_back(static_cast<int>(i));
				cuts.push_back(static_cast<int>(wside.size()));
				auto seg_stat = [&](int a, int b, float* mean) {
					float sum = 0.f;
					int cnt = 0;
					for (int i = a; i < b; ++i)
						if (wside[static_cast<size_t>(i)]) {
							sum += wcosa[static_cast<size_t>(i)];
							cnt++;
						}
					*mean = cnt ? sum / static_cast<float>(cnt)
						: 1.f;
					float worst = 0.f;
					int at = (a + b) / 2;
					for (int i = a; i < b; ++i)
						if (wside[static_cast<size_t>(i)]) {
							const float e2 = fabsf(wcosa[
								static_cast<size_t>(i)] - *mean);
							if (e2 > worst) {
								worst = e2;
								at = i;
							}
						}
					return std::make_pair(worst, at);
				};
				for (int it = 0; it < 40
					&& static_cast<int>(cuts.size()) < 60; ++it) {
					float worst = 0.f;
					int wat = -1, wseg = -1;
					for (size_t c = 0; c + 1 < cuts.size(); ++c) {
						float mn;
						std::pair<float, int> pr = seg_stat(
							cuts[c], cuts[c + 1], &mn);
						if (pr.first > worst
							&& cuts[c + 1] - cuts[c] >= 2) {
							worst = pr.first;
							wat = pr.second;
							wseg = static_cast<int>(c);
						}
					}
					if (worst < 0.004f || wseg < 0)
						break;
					if (wat <= cuts[static_cast<size_t>(wseg)]
						|| wat >= cuts[static_cast<size_t>(wseg)
							+ 1])
						break;
					cuts.insert(cuts.begin() + wseg + 1, wat);
				}
				std::vector<signed char> sside;
				std::vector<float> scosa;
				for (size_t c = 0; c + 1 < cuts.size(); ++c) {
					const int a = cuts[c], b = cuts[c + 1];
					float mn;
					seg_stat(a, b, &mn);
					int pos = 0, neg = 0;
					for (int i = a; i < b; ++i) {
						if (wside[static_cast<size_t>(i)] > 0)
							pos++;
						else if (wside[static_cast<size_t>(i)] < 0)
							neg++;
					}
					signed char sd = 0;
					if (pos || neg)
						sd = pos >= neg ? 1 : -1;
					for (int i = a; i < b; ++i) {
						sside.push_back(sd);
						scosa.push_back(mn);
					}
				}
				Air::Result ar = Air::FlyWishSchedule(fe.sep, w,
					o.params, vt, g, sside, scosa,
					static_cast<int>(sside.size()) + 8);
				if (ar.hit && ar.dot < 0.f
					&& ar.struck_brush < 0) {
					const float H2 = Dot(ar.end_state.vel,
						ar.end_state.vel) + 2.f * o.params.gravity
						* (ar.pos.Z - fc.zmin);
					const bool rep = Len(ar.pos - fe.cp) <= 24.f
						&& fabsf(H2 - eh) <= 0.03f * eh;
					if (rep && exact_ok)
						reps++;
					printf("repfit:   %d-segment schedule: strike "
						"%.0fu off | dot %.1f vs %.1f | E %.0fk "
						"vs %.0fk | %s\n",
						static_cast<int>(cuts.size()) - 1,
						Len(ar.pos - fe.cp), ar.dot,
						Dot(fe.v1, fc.n), H2 / 1e3f, eh / 1e3f,
						rep ? (wviol == 0
							? "REPRESENTABLE"
							: "reproduced (human outside law)")
						: "DIVERGED <- segment quantization");
				} else {
					printf("repfit:   %d-segment schedule: %s "
						"(closest %.0fu) <- segment quantization"
						"\n", static_cast<int>(cuts.size()) - 1,
						ar.hit ? "contact-assisted"
							: (ar.grounded ? "grounded"
								: "missed"),
						ar.miss_dist < 1e8f ? ar.miss_dist
							: -1.f);
				}
			}
		}
		printf("repfit: %d/%d flights representable in the wish "
			"basis (exact row AND segment row reproduce)\n",
			reps, rows);
		fflush(stdout);
		return 0;
	}

	// ladder: THE RECOVERY LADDER (advisor 2026-08-18) - the
	// controlled diagnostic around a known-admissible human witness.
	// DIAGNOSTIC ONLY, never a production seed. Steps: (1) how many
	// cosa knots until the compressed representation reproduces the
	// witness (resolution test); (2) can the optimizer hold/improve
	// when handed the exact schedule (basin-stability test); (3)
	// perturbation radii until recovery fails (local-move strength);
	// ordinary-seed performance is the global-seeding reference.
	int CmdLadder(const std::string& map_path, const ReplayOpts& o,
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
			printf("ladder: %s\n", err.c_str());
			return 1;
		}
		Tape tape;
		if (!LoadTas(tape_path, tape, &err)) {
			printf("ladder: %s\n", err.c_str());
			return 1;
		}
		std::vector<EFEvent> events;
		if (!EFReplay(w, g, o, tape, &events, nullptr, nullptr)) {
			printf("ladder: no flight events in tape\n");
			return 1;
		}
		// Bank context (diagnostic floors land here, src-tagged).
		std::map<std::string, EFBankEntry> bank;
		const std::string bank_path = tape_path + ".bank";
		EFBankLoad(bank_path, &bank);
		const unsigned long long maph = EFFileHash(map_path);
		const unsigned long long prh = EFParamsHash(o.params);
		bool bank_dirty = false;
		std::vector<Vec3> vels;
		{
			PlayerState s;
			s.pos = tape.start.origin;
			s.vel = tape.start.velocity;
			s.ducked = tape.start.ducked;
			s.hull_state = tape.start.ducked ? 1 : 0;
			s.stamina = tape.start.stamina;
			TraceResult tr;
			const float gf = w.TraceHull(s.pos,
				s.pos - Vec3(0.f, 0.f, 2.f), s.ducked, &tr);
			if (gf < 1.f && tr.brush >= 0
				&& tr.normal.Z >= o.params.walkable_z) {
				s.pos.Z -= 2.f * gf;
				s.on_ground = true;
				s.ground_brush = tr.brush;
			}
			vels.push_back(s.vel);
			for (const TapeFrame& fr : tape.frames) {
				MoveTick(s, w, o.params, fr.pitch, fr.yaw,
					fr.fmove, fr.smove, fr.umove, fr.buttons,
					nullptr);
				vels.push_back(s.vel);
			}
		}
		for (size_t ei = 0; ei < events.size(); ++ei) {
			if (o.ef_event >= 0
				&& static_cast<int>(ei) != o.ef_event)
				continue;
			const EFEvent& fe = events[ei];
			const Route::Face& fc = g.faces[fe.fidx];
			const int t0 = fe.sep_tick + 1;
			const int t1 = fe.tick;
			if (t1 - t0 < 6 || t1 >= static_cast<int>(vels.size()))
				continue;
			// The human wish schedule (executor-convention
			// inversion, as validated by repfit).
			std::vector<signed char> hside;
			std::vector<float> hcosa;
			for (int t2 = t0; t2 < t1; ++t2) {
				const TapeFrame& fr = tape.frames[
					static_cast<size_t>(t2)];
				const float yr = fr.yaw * 0.0174532925f;
				const float wx = cosf(yr) * fr.fmove
					+ sinf(yr) * fr.smove;
				const float wy = sinf(yr) * fr.fmove
					- cosf(yr) * fr.smove;
				const Vec3& vv = vels[static_cast<size_t>(t2)];
				if (wx * wx + wy * wy < 1.f || Len2D(vv) < 1.f) {
					hside.push_back(0);
					hcosa.push_back(1.f);
					continue;
				}
				const float omega = atan2f(wy, wx);
				const float h = atan2f(vv.Y, vv.X);
				const float d = Steer::WrapPi(omega
					- 3.14159265f - h);
				hside.push_back(static_cast<signed char>(
					d >= 0.f ? 1 : -1));
				hcosa.push_back(cosf(d));
			}
			Vec3 vpost;
			Fn::ClipVelocity(fe.v1, fc.n, &vpost);
			const float eh = Dot(vpost, vpost)
				+ 2.f * o.params.gravity
				* (fe.cp.Z - fc.zmin);
			printf("ladder: event %d t%4d f%d | E_h %.0fk | window "
				"%d ticks\n", static_cast<int>(ei), fe.tick,
				fe.fidx, eh / 1e3f, t1 - t0);
			Air::Target vt;
			vt.face = fe.fidx;
			vt.dot_cap = 3000.f;
			vt.aim = fe.cp;
			// (1) THE RESOLUTION LADDER: topology fixed at the
			// human's side runs; K cosa knots per run.
			auto fly_fit = [&](int K, float* H_out, float* d_out) {
				std::vector<signed char> ss;
				std::vector<float> sc;
				size_t a = 0;
				while (a < hside.size()) {
					size_t b = a;
					while (b < hside.size()
						&& hside[b] == hside[a])
						b++;
					const int len = static_cast<int>(b - a);
					const int kk = K > len ? (len < 1 ? 1 : len)
						: K;
					for (int i = 0; i < len; ++i) {
						float c;
						if (kk <= 1) {
							c = hcosa[a + static_cast<size_t>(
								len / 2)];
						} else {
							const float x = static_cast<float>(i)
								/ static_cast<float>(len - 1)
								* static_cast<float>(kk - 1);
							int k0 = static_cast<int>(x);
							if (k0 > kk - 2)
								k0 = kk - 2;
							const float frx = x
								- static_cast<float>(k0);
							const size_t at0 = a
								+ static_cast<size_t>((len - 1)
									* k0 / (kk - 1));
							const size_t at1 = a
								+ static_cast<size_t>((len - 1)
									* (k0 + 1) / (kk - 1));
							c = hcosa[at0] * (1.f - frx)
								+ hcosa[at1] * frx;
						}
						ss.push_back(hside[a]);
						sc.push_back(c);
					}
					a = b;
				}
				vt.max_ticks = static_cast<int>(ss.size()) + 8;
				Air::Result ar = Air::FlyWishSchedule(fe.sep, w,
					o.params, vt, g, ss, sc,
					static_cast<int>(ss.size()) + 8);
				if (ar.hit && ar.dot < 0.f
					&& ar.struck_brush < 0) {
					*H_out = Dot(ar.end_state.vel,
						ar.end_state.vel) + 2.f
						* o.params.gravity
						* (ar.pos.Z - fc.zmin);
					*d_out = Len(ar.pos - fe.cp);
					return true;
				}
				*H_out = -1e30f;
				*d_out = ar.miss_dist < 1e8f ? ar.miss_dist
					: -1.f;
				return false;
			};
			int k_needed = -1;
			const int Ks[7] = { 1, 2, 3, 5, 9, 17, 9999 };
			for (int kidx = 0; kidx < 7; ++kidx) {
				float H2, d2;
				const bool hit = fly_fit(Ks[kidx], &H2, &d2);
				const bool rep = hit && d2 <= 24.f
					&& fabsf(H2 - eh) <= 0.03f * eh;
				printf("ladder:   K=%2d knots/run: %s H %.0fk, "
					"%.0fu off%s\n", Ks[kidx],
					hit ? "strike" : "miss",
					hit ? H2 / 1e3f : 0.f, d2,
					rep ? "  <- REPRODUCES" : "");
				if (rep && k_needed < 0)
					k_needed = Ks[kidx];
			}
			// (2) BASIN STABILITY: optimizer handed the exact
			// schedule.
			int fl = 0;
			// The basin test is a SOUNDNESS issue at 16u: the seed's
			// own residual is ~0, so a coarse radius lets the
			// optimizer walk 15u away and still score as holding the
			// basin.
			Entrance::RefTune ld_tune;
			ld_tune.precision = 4.f;
			Entrance::RefResult r0 = Entrance::RefSolve(fe.sep, w,
				o.params, g, fe.fidx, fe.cp, 16.f, t1 - t0, 800,
				false, &fl, fc.zmin, fe.sep_ctl, nullptr, &hside,
				&hcosa, &ld_tune);
			printf("ladder:   optimizer from exact schedule: %s "
				"%.0fk (dot %.1f) vs E_h %.0fk | %s\n",
				r0.ok ? "H" : "NONE",
				r0.ok ? r0.H / 1e3f : 0.f,
				r0.ok ? r0.flight.dot : 0.f, eh / 1e3f,
				r0.ok && r0.H >= eh - 2000.f
					? "HOLDS THE BASIN"
					: "<- LOSES THE BASIN (destructive moves or "
					  "compression)");
			// Bank the diagnostic floor (ruling: real physics, a
			// permanent lower bound - src-tagged, never a seed).
			if (r0.ok) {
				const std::string bkey = EFBankKey("air-entry",
					EFStateHash(fe.sep, fe.sep_ctl), maph, prh,
					fe.tick, fe.fidx);
				auto bit = bank.find(bkey);
				if (bit == bank.end()
					|| r0.H > bit->second.H + 1.f) {
					EFBankEntry nb;
					nb.H = r0.H;
					nb.horizon = r0.horizon;
					nb.src = "diagnostic";
					nb.side = r0.wside;
					nb.cosa = r0.wcosa;
					nb.spos = fe.sep.pos;
					nb.svel = fe.sep.vel;
					nb.sducked = fe.sep.ducked ? 1 : 0;
					nb.sside = fe.sep_ctl.side;
					nb.sage = fe.sep_ctl.age;
					nb.cp = r0.flight.pos;
					nb.whash = EFFnv64(r0.wcosa.data(),
						r0.wcosa.size() * sizeof(float),
						EFFnv64(r0.wside.data(),
							r0.wside.size()));
					bank[bkey] = nb;
					bank_dirty = true;
					printf("ladder:   banked %.0fk (diagnostic "
						"floor)\n", r0.H / 1e3f);
				}
			}
			// (3) PERTURBATION RECOVERY.
			unsigned rng = 0xC0FFEEu
				+ static_cast<unsigned>(ei) * 977u;
			auto frand = [&]() {
				rng = rng * 1664525u + 1013904223u;
				return static_cast<float>((rng >> 8) & 0xFFFF)
					/ 65535.f * 2.f - 1.f;
			};
			const float radii[4] = { 0.05f, 0.12f, 0.25f, 0.45f };
			for (int ri = 0; ri < 4; ++ri) {
				int rec = 0;
				for (int trial = 0; trial < 2; ++trial) {
					std::vector<float> pc = hcosa;
					for (float& c : pc) {
						c += radii[ri] * frand();
						if (c > 1.f) c = 1.f;
						if (c < -1.f) c = -1.f;
					}
					int fl2 = 0;
					Entrance::RefResult rp = Entrance::RefSolve(
						fe.sep, w, o.params, g, fe.fidx, fe.cp,
						16.f, t1 - t0, 800, false, &fl2,
						fc.zmin, fe.sep_ctl, nullptr, &hside,
						&pc, &ld_tune);
					if (rp.ok && rp.H >= eh - 2000.f)
						rec++;
				}
				printf("ladder:   perturb r=%.2f: %d/2 recovered "
					"to >= E_h\n", radii[ri], rec);
			}
		}
		if (bank_dirty) {
			EFBankSave(bank_path, bank);
			printf("ladder: witness bank -> %s\n",
				bank_path.c_str());
		}
		fflush(stdout);
		return 0;
	}

	// airsuite: THE GENERAL VALIDATION MATRIX (advisor 2026-08-18:
	// the certification bed is a synthetic matrix over the problem
	// class - start speed x vertical state x horizon x face
	// orientation x target width - with tapes nowhere in it. The
	// generality law's teeth: production must solve arbitrary
	// boundary-value instances, not one map's three points.) Per
	// case: place a clean-air start whose ballistics reach the
	// target at the chosen horizon, run the production solve, and
	// report strike class, replayed H, the certified optimistic
	// ceiling U, and the sandwich gap G = U - H. Aggregates expose
	// which corners of the class the current machinery fails.
	int CmdAirSuite(const std::string& map_path, const ReplayOpts& o) {
		World w;
		std::string err;
		w.true_interval_corner = o.corner_true;
		if (!w.Load(map_path, o.hulls, &err, o.edge_bevels)) {
			printf("LOAD FAILED (bsp): %s\n", err.c_str());
			return 1;
		}
		Route::Graph g;
		if (!Route::Build(w, &g, 2000.f, &err)) {
			printf("airsuite: %s\n", err.c_str());
			return 1;
		}
		const float grav = o.params.gravity;
		const float dt = o.params.dt;
		int total = 0, near_ok = 0, exact_ok = 0;
		double gap_sum = 0.0;
		int gap_n = 0;
		// EXACTNESS LADDER (advisor 2026-08-19): per-case closest
		// clean strike, aggregated per tolerance rung.
		int rung_n[6] = { 0, 0, 0, 0, 0, 0 };
		std::vector<float> rmins;
		struct Agg {
			int n = 0, hit = 0;
		};
		Agg by_hz[2], by_sp[2], by_vz[3];
		// Crediting stays pinned to the 28u acceptance radius (this
		// is the feasibility/bound suite); continuation only ADDS
		// pressure toward the exactness rungs the report measures,
		// so no value metric can regress from it.
		Entrance::RefTune as_tune;
		as_tune.precision = 1.f;
		// Controlled-comparison arms (--sched / --cover-top /
		// --suite-budget). Defaults reproduce the standing baseline.
		if (o.suite_sched >= 0)
			as_tune.scheduler = o.suite_sched;
		if (o.suite_cover >= 0)
			as_tune.cover_top = o.suite_cover;
		const int as_budget = o.suite_budget > 0 ? o.suite_budget : 600;
		// MEASURED coverage cost - the mandatory prefix is an
		// operational-efficiency claim, so it gets counted, not asserted.
		long long cov_ev = 0, cov_n = 0, all_ev = 0;
		const int hzs[2] = { 30, 55 };
		const float sps[2] = { 400.f, 900.f };
		const float vzs[3] = { 150.f, -100.f, -400.f };
		for (size_t fi = 0; fi < g.faces.size(); ++fi) {
			const Route::Face& fc = g.faces[fi];
			const float hn = sqrtf(fc.n.X * fc.n.X
				+ fc.n.Y * fc.n.Y);
			if (hn < 1e-4f)
				continue;
			const Vec3 q = fc.centroid;
			for (int hi2 = 0; hi2 < 2; ++hi2)
				for (int si2 = 0; si2 < 2; ++si2)
					for (int vi2 = 0; vi2 < 3; ++vi2) {
						const int Hn = hzs[hi2];
						const float s0 = sps[si2];
						const float vz0 = vzs[vi2];
						const float T = static_cast<float>(Hn)
							* dt;
						// Ballistic start altitude for arrival at
						// the target after Hn ticks.
						const float z0 = q.Z - vz0 * T
							+ 0.5f * grav * T * T;
						// Start in front of the face at a fraction
						// of the gain-law reach; lateral sign
						// alternates by case for spread.
						const float reach = Envelope::DMax(s0, Hn,
							o.params);
						const float dist = 0.8f * reach;
						const float paz = atan2f(fc.n.Y, fc.n.X);
						const float latsgn = ((fi + hi2 + si2
							+ vi2) & 1) ? 0.45f : -0.45f;
						const float outaz = paz + latsgn;
						PlayerState st;
						st.pos = Vec3(q.X + cosf(outaz) * dist,
							q.Y + sinf(outaz) * dist, z0);
						const float inaz = atan2f(
							q.Y - st.pos.Y, q.X - st.pos.X);
						const float voff = ((fi + vi2) & 1)
							? 0.3f : -0.3f;
						st.vel = Vec3(cosf(inaz + voff) * s0,
							sinf(inaz + voff) * s0, vz0);
						st.ducked = false;
						st.hull_state = 0;
						total++;
						int fl = 0;
						Entrance::RefResult rr =
							Entrance::RefSolve(st, w, o.params, g,
								static_cast<int>(fi), q, 28.f, Hn,
								as_budget, false, &fl, fc.zmin,
								Steer::CtlState(), nullptr, nullptr,
								nullptr, &as_tune);
						cov_ev += rr.sched_cover_ev;
						cov_n += rr.sched_cover;
						all_ev += rr.evals;
						const float sa = Envelope::SMax(s0, Hn,
							o.params);
						const float va = Envelope::VzAfter(vz0,
							Hn, o.params);
						const float U = sa * sa + va * va
							+ 2.f * grav * (q.Z - fc.zmin);
						by_hz[hi2].n++;
						by_sp[si2].n++;
						by_vz[vi2].n++;
						for (int r2 = 0; r2 < 6; ++r2)
							if (rr.strike_rmin
								<= Entrance::kLadder[r2])
								rung_n[r2]++;
						if (rr.strike_rmin < 1e29f)
							rmins.push_back(rr.strike_rmin);
						if (rr.strike_rmin <= 4.f)
							exact_ok++;
						if (rr.ok) {
							near_ok++;
							by_hz[hi2].hit++;
							by_sp[si2].hit++;
							by_vz[vi2].hit++;
							const float d = Len(rr.flight.pos
								- q);
							gap_sum += static_cast<double>(
								U - rr.H);
							gap_n++;
							printf("airsuite: f%d hz%d s%.0f "
								"vz%+.0f | H %.0fk (dot %.1f, "
								"%.0fu) rmin %.1fu | U %.0fk G "
								"%.0fk | %d ev\n",
								static_cast<int>(fi), Hn,
								s0, vz0, rr.H / 1e3f,
								rr.flight.dot, d,
								rr.strike_rmin, U / 1e3f,
								(U - rr.H) / 1e3f, rr.evals);
						} else {
							printf("airsuite: f%d hz%d s%.0f "
								"vz%+.0f | NO STRIKE (rmin "
								"%.0fu, %d ev) | U %.0fk\n",
								static_cast<int>(fi),
								Hn, s0, vz0,
								rr.strike_rmin < 1e29f
									? rr.strike_rmin : -1.f,
								rr.evals, U / 1e3f);
						}
					}
		}
		printf("airsuite: arm sched %d cover_top %d budget %d | coverage "
			"%lld actions %lld ev = %.1f%% of %lld total\n",
			o.suite_sched, o.suite_cover, as_budget, cov_n, cov_ev,
			all_ev ? 100.0 * static_cast<double>(cov_ev)
				/ static_cast<double>(all_ev) : 0.0, all_ev);
		printf("airsuite: %d/%d cases struck (%d exact<=4u) | mean "
			"sandwich gap %.0fk\n", near_ok, total, exact_ok,
			gap_n ? gap_sum / gap_n / 1e3 : 0.0);
		// The tolerance ladder: how close the search gets, not just
		// whether it got inside one radius.
		if (!rmins.empty()) {
			std::sort(rmins.begin(), rmins.end());
			const float p50 = rmins[rmins.size() / 2];
			const float p95 = rmins[(rmins.size() * 95) / 100
				>= rmins.size() ? rmins.size() - 1
				: (rmins.size() * 95) / 100];
			printf("airsuite: ladder <=32u:%d <=16u:%d <=8u:%d "
				"<=4u:%d <=2u:%d <=1u:%d of %d | rmin p50 %.1fu "
				"p95 %.1fu\n", rung_n[0], rung_n[1], rung_n[2],
				rung_n[3], rung_n[4], rung_n[5], total, p50, p95);
		}
		printf("airsuite: by horizon: hz30 %d/%d hz55 %d/%d | by "
			"speed: s400 %d/%d s900 %d/%d | by vz: +150 %d/%d "
			"-100 %d/%d -400 %d/%d\n",
			by_hz[0].hit, by_hz[0].n, by_hz[1].hit, by_hz[1].n,
			by_sp[0].hit, by_sp[0].n, by_sp[1].hit, by_sp[1].n,
			by_vz[0].hit, by_vz[0].n, by_vz[1].hit, by_vz[1].n,
			by_vz[2].hit, by_vz[2].n);
		fflush(stdout);
		return 0;
	}

	// ================= airrec: THE CONSTRUCTIVE RECOVERABILITY
	// SUITE (advisor 2026-08-19; Docs/AirRecSpec.md is the design of
	// record). Hidden LEGAL wish schedules create known-reachable
	// problems; production must recover them cold. Layer 1 = free-air
	// boundary problems (S0, Q, T, theta) - tests the numerical
	// shooting core. Layer 2 = clean face strikes - tests the actual
	// Air->Board operator. QUARANTINE: oracle schedules/features live
	// only in this harness; production receives (S0, Q, T[, theta]);
	// nothing is banked and nothing is seeded. Every oracle is also a
	// constructive floor: any certified ceiling below it is FALSIFIED
	// (empirical validation only - passing is not certification).

	namespace {

	struct RecOracle {
		bool  ok = false;
		const char* why = "";
		PlayerState S0;
		std::vector<signed char> side;
		std::vector<float> cosa;
		int   T = 0;          // oracle arrival tick
		Vec3  Q;              // boundary point / contact
		float th = 0.f;       // terminal heading
		float sT = 0.f;       // horizontal speed at T
		float vzT = 0.f;
		float E = 0.f;        // post-board energy (layer 2)
		int   face = -1;      // layer 2 target face
		float zmin = 0.f;
		// Stratum labels + measured complexity features (harness
		// only; production never sees these).
		int   rev = 0, prof = 0, dwell = 0;
		int   min_dwell = 0;
		float dh_tot = 0.f, dh_max = 0.f, sac = 0.f;
		int   brake = 0;
	};

	// Deterministic legal schedule for one control-complexity
	// stratum: rev+1 alternating side segments (each >= min_gap, so
	// legal from a fresh CtlState by construction), cosa profile by
	// class. Encodes control-space coverage, never trajectory shapes.
	void RecSchedule(int rev, int prof, int dwell_style, int T,
	                 int min_gap, unsigned seed,
	                 std::vector<signed char>* side,
	                 std::vector<float>* cosa, int* min_dwell) {
		side->assign(static_cast<size_t>(T), 0);
		cosa->assign(static_cast<size_t>(T), 1.f);
		unsigned rng = seed * 2654435761u + 0x9e3779b9u;
		auto fr = [&]() {
			rng = rng * 1664525u + 1013904223u;
			return static_cast<float>((rng >> 8) & 0xFFFF)
				/ 65535.f;
		};
		const int nseg = rev + 1;
		std::vector<int> len(static_cast<size_t>(nseg), 0);
		int used = 0;
		if (dwell_style == 0) {
			// Near-minimum dwell: min_gap + 0..3, remainder to the
			// last segment.
			for (int i = 0; i + 1 < nseg; ++i) {
				len[static_cast<size_t>(i)] = min_gap
					+ static_cast<int>(fr() * 3.99f);
				used += len[static_cast<size_t>(i)];
			}
			len[static_cast<size_t>(nseg) - 1] = T - used;
		} else {
			for (int i = 0; i < nseg; ++i)
				len[static_cast<size_t>(i)] = T / nseg;
			len[static_cast<size_t>(nseg) - 1] += T
				- (T / nseg) * nseg;
		}
		bool bad = false;
		for (int i = 0; i < nseg; ++i)
			if (len[static_cast<size_t>(i)] < min_gap)
				bad = true;
		if (bad) {
			for (int i = 0; i < nseg; ++i)
				len[static_cast<size_t>(i)] = T / nseg;
			len[static_cast<size_t>(nseg) - 1] += T
				- (T / nseg) * nseg;
		}
		*min_dwell = 1 << 20;
		for (int i = 0; i < nseg; ++i)
			if (len[static_cast<size_t>(i)] < *min_dwell)
				*min_dwell = len[static_cast<size_t>(i)];
		// STORED-BASIS CONVENTION (measured 2026-08-19, smoke run):
		// the canonical (side, cosa) executor realizes its wish at
		// wh + pi, so stored cosa = +1 wishes BACKWARD (max brake)
		// and the active gain band is small-NEGATIVE stored cosa
		// (~ -cap/s). Profiles below are written in STORED units so
		// the strata mean what they claim: gain flights gain.
		const float ph = fr() * 6.2831853f;
		signed char s = fr() < 0.5f ? static_cast<signed char>(-1)
			: static_cast<signed char>(1);
		int k = 0;
		for (int i = 0; i < nseg; ++i) {
			for (int j = 0; j < len[static_cast<size_t>(i)];
				++j, ++k) {
				(*side)[static_cast<size_t>(k)] = s;
				float c = -0.02f;
				switch (prof) {
				case 0: c = -0.02f; break;               // near-max gain
				case 1: c = 0.24f + 0.26f                // smooth vary:
					* sinf(6.2831853f * static_cast<float>(k)
						/ static_cast<float>(T) + ph);   // [-0.02, 0.5]
					break;
				case 2: c = (i & 1) ? 0.65f : -0.02f;    // aggressive
					break;                               // gain<->brake
				case 3: c = k < T / 3 ? 0.7f : -0.02f;   // brake->gain
					break;
				}
				(*cosa)[static_cast<size_t>(k)] = c;
			}
			s = static_cast<signed char>(-s);
		}
	}

	// Open-loop free flight with per-tick visibility (harness-side
	// feature extraction). Fails on any contact or grounding. Fills
	// the oracle's terminal state + complexity features.
	bool RecFreeFly(const PlayerState& S0, const World& w,
	                const MoveParams& p,
	                const std::vector<signed char>& side,
	                const std::vector<float>& cosa,
	                RecOracle* out, std::vector<Vec3>* traj) {
		PlayerState s = S0;
		const int hold = s.ducked ? IN_DUCK : 0;
		const float cap2 = p.air_speed_cap * p.air_speed_cap;
		out->dh_tot = 0.f;
		out->dh_max = 0.f;
		out->sac = 0.f;
		out->brake = 0;
		const int T = static_cast<int>(side.size());
		float h_prev = atan2f(S0.vel.Y, S0.vel.X);
		for (int k = 0; k < T; ++k) {
			const float s2d = Len2D(s.vel);
			const float h = s2d > 1.f
				? atan2f(s.vel.Y, s.vel.X) : h_prev;
			float yaw = h * 57.2957795f, fm = 0.f, sm = 0.f;
			if (side[static_cast<size_t>(k)] != 0 && s2d > 1.f) {
				Air::WishInputs(h,
					static_cast<int>(
						side[static_cast<size_t>(k)]),
					cosa[static_cast<size_t>(k)], &yaw, &fm,
					&sm);
				// Stored convention: POSITIVE cosa = braking wish.
				if (cosa[static_cast<size_t>(k)] > 0.3f)
					out->brake++;
			}
			const float pre2 = s2d * s2d + cap2;
			TickEvents ev;
			MoveTick(s, w, p, 0.f, yaw, fm, sm, 0.f, hold, &ev);
			if (traj)
				traj->push_back(s.pos);
			if (ev.ncontacts > 0 || s.on_ground)
				return false;
			const float post = Len2D(s.vel);
			const float dsac = pre2 - post * post;
			if (dsac > 0.f)
				out->sac += dsac;
			const float h2 = post > 1.f
				? atan2f(s.vel.Y, s.vel.X) : h;
			const float dh = fabsf(Steer::WrapPi(h2 - h));
			out->dh_tot += dh;
			if (dh > out->dh_max)
				out->dh_max = dh;
			h_prev = h2;
		}
		out->Q = s.pos;
		out->sT = Len2D(s.vel);
		out->vzT = s.vel.Z;
		out->th = out->sT > 1.f ? atan2f(s.vel.Y, s.vel.X) : 0.f;
		return true;
	}

	// Layer-2 oracle construction: free flight is TRANSLATION
	// INVARIANT, so probe the hidden schedule once in open air, then
	// translate the start so the trajectory crosses the target face
	// near its centroid, and demand a clean single-plane strike from
	// the TRUE replay. Retries over approach bearing and plane
	// offset; failure is reported, never silently dropped.
	bool RecBoardOracle(const World& w, const MoveParams& p,
	                    const Route::Graph& g, int face_idx,
	                    float s0, float vz0, int T,
	                    const std::vector<signed char>& side,
	                    const std::vector<float>& cosa,
	                    float zhigh, RecOracle* out) {
		const Route::Face& fc = g.faces[static_cast<size_t>(
			face_idx)];
		const float hn = sqrtf(fc.n.X * fc.n.X + fc.n.Y * fc.n.Y);
		if (hn < 1e-4f) {
			out->why = "vertical-normal face";
			return false;
		}
		const float paz = atan2f(fc.n.Y, fc.n.X);
		const float voffs[5] = { 0.f, 0.3f, -0.3f, 0.6f, -0.6f };
		const float eps[4] = { 4.f, 10.f, 1.f, 16.f };
		for (int vo = 0; vo < 5; ++vo) {
			const float inaz = Steer::WrapPi(paz + 3.14159265f
				+ voffs[vo]);
			PlayerState st;
			st.pos = Vec3(0.f, 0.f, zhigh);
			st.vel = Vec3(cosf(inaz) * s0, sinf(inaz) * s0, vz0);
			st.ducked = false;
			st.hull_state = 0;
			RecOracle probe;
			std::vector<Vec3> traj;
			if (!RecFreeFly(st, w, p, side, cosa, &probe, &traj))
				continue;
			// The terminal velocity must actually approach the
			// plane, or no translation can make it strike.
			const float vdot = probe.sT * (cosf(probe.th) * fc.n.X
				+ sinf(probe.th) * fc.n.Y) + probe.vzT * fc.n.Z;
			if (vdot > -50.f)
				continue;
			const Vec3 dT = traj[static_cast<size_t>(T) - 1]
				- st.pos;
			for (int ei = 0; ei < 4; ++ei) {
				PlayerState real = st;
				real.pos = fc.centroid + Scale(fc.n, eps[ei])
					- dT;
				Air::Target vt;
				vt.face = face_idx;
				vt.dot_cap = 3000.f;
				vt.aim = fc.centroid;
				vt.max_ticks = T + 10;
				Air::Result ar = Air::FlyWishSchedule(real, w, p,
					vt, g, side, cosa, T + 10);
				if (!ar.hit || ar.dot >= 0.f
					|| ar.struck_brush >= 0)
					continue;
				if (ar.tick < T - 6 || ar.tick > T + 8)
					continue;
				out->ok = true;
				out->S0 = real;
				out->T = ar.tick;
				out->Q = ar.pos;
				out->th = atan2f(ar.v1.Y, ar.v1.X);
				out->sT = Len2D(ar.v1);
				out->vzT = ar.v1.Z;
				out->E = Dot(ar.end_state.vel, ar.end_state.vel)
					+ 2.f * p.gravity * real.gravity_scale
					* (ar.pos.Z - fc.zmin);
				out->face = face_idx;
				out->zmin = fc.zmin;
				out->dh_tot = probe.dh_tot;
				out->dh_max = probe.dh_max;
				out->sac = probe.sac;
				out->brake = probe.brake;
				return true;
			}
		}
		out->why = "no clean-strike construction";
		return false;
	}

	} // namespace

	// ================= airprops: THE PROPERTY GATE (advisor
	// 2026-08-19: "convert each of the seven adversarial-review defects
	// into a regression/property test where possible"). Each property is
	// an INVARIANT of the operator checked against the real production
	// path - the two pure helpers are shared with production precisely
	// so a test can never drift from it. One authoritative threshold set
	// drives both the printed verdict and the exit code.
	// ================= exitfit: RIDE REPRESENTATION COMPLETENESS
	// (ExitField stage 2; design of record Docs/ExitFieldSpec.md 8).
	// Validates the pair (S_B, U_R) as sufficient to reproduce the
	// exact transition relation R_F(B). Two hypotheses, kept separate:
	// X1 control completeness (hidden NATIVE inputs, never generated in
	// the canonical basis, must invert and replay exactly) and X2
	// operator-state completeness (aimed at the SEAM - serialization,
	// carried dwell legality, duck/hull sensitivity - not at MoveTick
	// purity, which is trivial). No optimizer, no controller, no
	// ExitDoomed, no human data in generation (X7 quarantined).

	namespace {

	// One native input tick (what a player's client actually sends).
	struct NativeTick {
		float yaw = 0.f;
		float fmove = 0.f;
		float smove = 0.f;
		int   btn = 0;
	};

	// Deterministic native ride-input generator. Patterns are composed
	// of SEGMENTS over raw keys and view yaw - held key combos, yaw
	// ramps, coasts, duck press windows - so the canonical basis is
	// never the generating language (that would make X1 a tautology).
	void RideNative(int stratum, unsigned seed, float h0, int M,
	                std::vector<NativeTick>* out) {
		out->clear();
		unsigned rng = seed * 2654435761u + 0x9e3779b9u;
		auto fr = [&]() {
			rng = rng * 1664525u + 1013904223u;
			return static_cast<float>((rng >> 8) & 0xFFFF) / 65535.f;
		};
		const float d2r = 0.0174532925f;
		float yaw = (h0 + (fr() - 0.5f)) * 57.2957795f;
		int duck_a = -1, duck_b = -1;
		if (stratum == 4) {              // mid-ride duck pulse
			duck_a = 10 + static_cast<int>(fr() * 10.f);
			duck_b = duck_a + 8 + static_cast<int>(fr() * 8.f);
		}
		if (stratum == 5)                // duck-off exit: press and hold
			duck_a = 14 + static_cast<int>(fr() * 12.f);
		int k = 0;
		while (k < M) {
			int len = 8 + static_cast<int>(fr() * 12.f);
			if (len > M - k)
				len = M - k;
			float fm = 0.f, sm = 0.f, rate = 0.f;
			switch (stratum) {
			case 0:                      // held strafe key, fixed yaw
				sm = fr() < 0.5f ? -450.f : 450.f;
				break;
			case 1:                      // yaw-ramp carve, held key
				sm = fr() < 0.5f ? -450.f : 450.f;
				rate = (0.5f + fr() * 2.5f) * (fr() < 0.5f ? 1.f : -1.f);
				break;
			case 2:                      // W-forward push + slow ramp
				fm = 450.f;
				rate = (fr() - 0.5f) * 1.2f;
				break;
			case 3:                      // coast interludes between keys
				if (k % 2 == 0) {
					sm = fr() < 0.5f ? -450.f : 450.f;
				}
				break;
			case 4:                      // duck pulse over a held carve
			case 5:                      // duck-off exit
				sm = fr() < 0.5f ? -450.f : 450.f;
				rate = (fr() - 0.5f) * 2.f;
				break;
			case 6:                      // key chord (W+A / W+D)
				fm = 450.f;
				sm = fr() < 0.5f ? -450.f : 450.f;
				rate = (fr() - 0.5f) * 1.5f;
				break;
			case 7:                      // ANALOG stratum: sub-maxspeed
				sm = fr() < 0.5f ? -137.5f : 137.5f;
				rate = (fr() - 0.5f) * 1.5f;
				break;
			case 8:                      // hard brake (wish backward):
				fm = -450.f;             // slides down-face toward the
				rate = 0.f;              // valley / floor geometry
				break;
			case 9:                      // downhill dive: strong ramp
				sm = fr() < 0.5f ? -450.f : 450.f;
				rate = (3.f + fr() * 2.f) * (fr() < 0.5f ? 1.f : -1.f);
				break;
			}
			for (int j = 0; j < len; ++j, ++k) {
				NativeTick nt;
				nt.yaw = yaw;
				nt.fmove = fm;
				nt.smove = sm;
				nt.btn = (duck_a >= 0 && k >= duck_a
					&& (duck_b < 0 || k < duck_b)) ? IN_DUCK : 0;
				out->push_back(nt);
				yaw += rate;
				(void)d2r;
			}
		}
	}

	// The native runner: executes raw inputs through the exact engine
	// with THE SAME boundary classification as the canonical executor,
	// and inverts every tick through the authoritative formulas as it
	// goes. Returns the event (kind, tick) plus the per-boundary
	// trajectory and the inverted canonical schedule (consumed prefix
	// semantics identical to Ride::FlyRideSchedule).
	struct NativeRun {
		int  kind = Ride::kHorizon;
		int  ticks = 0;
		bool degenerate = false;   // near-zero speed tick seen
		std::vector<PlayerState> traj;
		std::vector<signed char> side;
		std::vector<float> cosa;
		std::vector<unsigned char> duck;
		std::vector<float> mag;
		bool any_analog = false;
		PlayerState end_state;
	};

	void RunNative(const Ride::BoundaryState& B, const World& w,
	               const MoveParams& p, const Route::Graph& g,
	               const std::vector<NativeTick>& in, NativeRun* out) {
		const Route::Face& fc = g.faces[static_cast<size_t>(B.face)];
		PlayerState s = B.ps;
		PlayerState prev = s;
		int prev_side = static_cast<int>(B.ctl.side);
		for (size_t k = 0; k < in.size(); ++k) {
			prev = s;
			const float s2d = Len2D(s.vel);
			if (s2d < 1.f) {
				out->degenerate = true;
				return;
			}
			const float h = atan2f(s.vel.Y, s.vel.X);
			// Invert BEFORE stepping (same phase as canonical emission).
			Ride::CanonTick ct;
			Ride::InvertWishInput(h, in[k].yaw, in[k].fmove,
				in[k].smove, p, prev_side, &ct);
			if (ct.side != 0)
				prev_side = ct.side;
			TickEvents ev;
			MoveTick(s, w, p, 0.f, in[k].yaw, in[k].fmove, in[k].smove,
				0.f, in[k].btn, &ev);
			bool on_face = false;
			int other = -1;
			for (int c = 0; c < ev.ncontacts; ++c) {
				if (ev.contact_brush[c] == fc.brush
					&& ev.contact_plane[c] == fc.side)
					on_face = true;
				else if (other < 0)
					other = c;
			}
			// The inverted tick joins the schedule only when the ride
			// consumes it (AIR_EXIT's peek tick is Air's, not ours).
			auto push_tick = [&]() {
				out->side.push_back(
					static_cast<signed char>(ct.side));
				out->cosa.push_back(ct.cosa);
				out->duck.push_back(in[k].btn & IN_DUCK ? 1 : 0);
				out->mag.push_back(ct.mag);
				if (ct.analog)
					out->any_analog = true;
				out->traj.push_back(s);
			};
			if (s.on_ground) {
				push_tick();
				out->kind = Ride::kGround;
				out->ticks = static_cast<int>(k) + 1;
				out->end_state = s;
				return;
			}
			if (other >= 0) {
				push_tick();
				out->kind = Ride::kContactTransfer;
				out->ticks = static_cast<int>(k) + 1;
				out->end_state = s;
				return;
			}
			if (!on_face) {
				// THE SEPARATION TICK IS BOOKED (seam re-amendment
				// 2026-08-19g): the ride owns the tick that actually
				// produced clean air; S+ is the boundary after it.
				push_tick();
				out->kind = Ride::kAirExit;
				out->ticks = static_cast<int>(k) + 1;
				out->end_state = s;
				return;
			}
			push_tick();
		}
		out->kind = Ride::kHorizon;
		out->ticks = static_cast<int>(in.size());
		out->end_state = s;
	}

	// Board-state factory: a boundary state is only ever obtained by
	// running the REAL engine into the face (never hand-assembled).
	// entry_kind: 0 = standing, 1 = ducked, 2 = the post-air-unduck
	// transient (constructed by actually unducking in flight and
	// letting the engine set hull_state = 2 itself).
	bool RideBoard(const World& w, const MoveParams& p,
	               const Route::Graph& g, int face_idx, float s0,
	               float vz0, float along, int entry_kind,
	               const Steer::CtlState& ctl,
	               Ride::BoundaryState* out, float down_frac = 0.f,
	               bool aim_downhill = false) {
		const Route::Face& fc = g.faces[static_cast<size_t>(face_idx)];
		const float hn = sqrtf(fc.n.X * fc.n.X + fc.n.Y * fc.n.Y);
		if (hn < 1e-4f)
			return false;
		const float paz = atan2f(fc.n.Y, fc.n.X);
		// Approach from off the face: mostly into the plane with an
		// along-face component so rides can run in both directions.
		// down_frac places the board part-way toward the face's low
		// edge and aim_downhill points the entry velocity down-slope -
		// the deterministic construction for CONTACT_TRANSFER / GROUND
		// event fixtures (rides that run off the bottom into whatever
		// geometry is there).
		float inaz = Steer::WrapPi(paz + 3.14159265f + along);
		Vec3 base = fc.centroid;
		const float dl = Len(fc.downhill);
		if (down_frac != 0.f && dl > 1e-4f) {
			const Vec3 dn = Scale(fc.downhill, 1.f / dl);
			float ext = 0.f;
			for (const Vec3& v : fc.verts) {
				// signed extent along downhill; negative down_frac
				// places UP-slope (the event fixtures start high and
				// ride the whole face down into the bottom geometry)
				const float e = Dot(v - fc.centroid, dn)
					* (down_frac < 0.f ? -1.f : 1.f);
				if (e > ext)
					ext = e;
			}
			base = fc.centroid + Scale(dn, ext * down_frac);
			if (aim_downhill)
				inaz = Steer::WrapPi(atan2f(dn.Y, dn.X) + along);
		}
		PlayerState st;
		// The down-slope construction runs nearly parallel to the
		// plane, so a 40u standoff overshoots the polygon before the
		// fall reaches the surface (measured: 16/16 board failures).
		// Hug the plane instead.
		st.pos = base + Scale(fc.n, aim_downhill ? 10.f : 40.f);
		st.pos.Z += aim_downhill ? 5.f : 20.f;
		st.vel = Vec3(cosf(inaz) * s0, sinf(inaz) * s0, vz0);
		st.ducked = entry_kind == 1;
		st.hull_state = entry_kind == 1 ? 1 : 0;
		if (entry_kind == 2) {
			// Fly ducked, then release: the ENGINE produces the
			// transient (never hand-set exotic states).
			st.ducked = true;
			st.hull_state = 1;
			TickEvents ev0;
			MoveTick(st, w, p, 0.f, inaz * 57.2957795f, 0.f, 0.f, 0.f,
				IN_DUCK, &ev0);
			if (ev0.ncontacts > 0 || st.on_ground)
				return false;
			TickEvents ev1;
			MoveTick(st, w, p, 0.f, inaz * 57.2957795f, 0.f, 0.f, 0.f,
				0, &ev1);
			if (ev1.ncontacts > 0 || st.on_ground)
				return false;
			if (st.hull_state != 2)
				return false;
		}
		const int hold = entry_kind == 1 ? IN_DUCK : 0;
		for (int k = 0; k < 60; ++k) {
			TickEvents ev;
			MoveTick(st, w, p, 0.f, inaz * 57.2957795f, 0.f, 0.f, 0.f,
				hold, &ev);
			if (st.on_ground)
				return false;
			for (int c = 0; c < ev.ncontacts; ++c)
				if (ev.contact_brush[c] == fc.brush
					&& ev.contact_plane[c] == fc.side) {
					out->ps = st;   // boundary AFTER the board tick
					out->ctl = ctl;
					out->face = face_idx;
					return true;
				}
			if (ev.ncontacts > 0)
				return false;   // struck something else first
		}
		return false;
	}

	} // namespace

	int CmdExitFit(const std::string& map_path, const ReplayOpts& o) {
		World w;
		std::string err;
		w.true_interval_corner = o.corner_true;
		if (!w.Load(map_path, o.hulls, &err, o.edge_bevels)) {
			printf("LOAD FAILED (bsp): %s\n", err.c_str());
			return 1;
		}
		Route::Graph g;
		if (!Route::Build(w, &g, 2000.f, &err)) {
			printf("exitfit: %s\n", err.c_str());
			return 1;
		}
		const MoveParams& p = o.params;
		int pass = 0, fail = 0;
		char buf[300];
		auto check = [&](const char* name, bool ok, const char* detail) {
			printf("exitfit: %-34s %s | %s\n", name,
				ok ? "PASS" : "FAIL", detail);
			if (ok) pass++; else fail++;
		};

		// ---- X1a INVERSION IDENTITY: canonical -> authoritative
		// emission -> authoritative wish -> inversion -> the same
		// canonical tick. The property that makes X1's failures mean
		// "basis incomplete" rather than "inverter buggy".
		{
			bool ok = true;
			int n = 0, bad = 0;
			float worst = 0.f;
			for (int hi2 = 0; hi2 < 8; ++hi2)
				for (int si = 0; si < 2; ++si)
					for (int ci = 0; ci <= 10; ++ci) {
						const float h = -3.14159265f
							+ 6.2831853f * static_cast<float>(hi2)
							/ 8.f;
						const int sd = si == 0 ? 1 : -1;
						const float ca = -1.f + 0.2f
							* static_cast<float>(ci);
						float yaw = 0.f, fm = 0.f, sm = 0.f;
						Air::WishInputs(h, sd, ca, &yaw, &fm, &sm);
						Ride::CanonTick ct;
						Ride::InvertWishInput(h, yaw, fm, sm, p, 0,
							&ct);
						n++;
						const float dc = fabsf(ct.cosa - ca);
						if (dc > worst)
							worst = dc;
						// side is physically meaningless at ca = +-1
						const bool degen = ca > 0.999f || ca < -0.999f;
						if (dc > 1e-4f || (!degen && ct.side != sd)
							|| ct.analog) {
							ok = false;
							bad++;
						}
					}
			snprintf(buf, sizeof(buf), "%d canonical ticks round-trip "
				"through WishInputs+WishFromInput; %d mismatches, "
				"worst |dcosa| %.2e", n, bad, worst);
			check("X1a inversion identity", ok, buf);
		}

		// ---- FIXTURES: native rides over every surfable face x
		// stratum x entry variation. Discards are REPORTED, never
		// silent.
		struct Fix {
			Ride::BoundaryState B;
			NativeRun nat;
			std::vector<NativeTick> raw;   // the hidden native inputs
			int face = 0, stratum = 0, entry = 0;
		};
		std::vector<Fix> fixes;
		int gen = 0, disc_board = 0, disc_short = 0, disc_degen = 0,
			disc_illegal = 0, disc_h2 = 0;
		const int kM = 160;
		const int min_gap = static_cast<int>(
			ceilf((1.f / p.dt) / p.strafe_rate_max));
		for (size_t fi2 = 0; fi2 < g.faces.size(); ++fi2) {
			const Route::Face& fc = g.faces[fi2];
			if (sqrtf(fc.n.X * fc.n.X + fc.n.Y * fc.n.Y) < 1e-4f)
				continue;
			for (int st2 = 0; st2 < 10; ++st2)
				for (int en = 0; en < 3; ++en, ++gen) {
					if (en != 0 && st2 > 3)
						continue;   // entry variants on core strata
					// Strata 8/9 are the EVENT fixtures: boards placed
					// down-face aimed down-slope, so rides run off the
					// bottom into whatever geometry is there.
					const bool evfix = st2 >= 8;
					const float s0 = (gen % 2) ? 900.f : 450.f;
					const float vz0 = (gen % 3 == 2) ? -350.f : -80.f;
					const float along = ((gen % 4) - 1.5f) * 0.45f;
					Steer::CtlState ctl;
					if (gen % 3 == 1) {
						ctl.side = 1;
						ctl.age = 6;
					} else if (gen % 3 == 2) {
						ctl.side = -1;
						ctl.age = 2;
					}
					Fix fx;
					fx.face = static_cast<int>(fi2);
					fx.stratum = st2;
					fx.entry = en;
					if (!RideBoard(w, p, g, fx.face, s0, vz0, along,
						en, ctl, &fx.B, evfix ? 0.55f : 0.f, evfix)) {
						if (en == 2) disc_h2++; else disc_board++;
						continue;
					}
					const float h0 = atan2f(fx.B.ps.vel.Y,
						fx.B.ps.vel.X);
					RideNative(st2, static_cast<unsigned>(
						gen * 104729 + 31), h0, kM, &fx.raw);
					RunNative(fx.B, w, p, g, fx.raw, &fx.nat);
					if (fx.nat.degenerate) {
						disc_degen++;
						continue;
					}
					if (fx.nat.kind == Ride::kAirExit
						&& fx.nat.ticks < 6) {
						disc_short++;
						continue;   // barely a ride
					}
					// The canonical side sequence must be dwell-legal
					// from the carried state, or the fixture is not a
					// legal ride (generator emits raw keys, so this
					// is a post-hoc filter - reported).
					if (!Ride::SchedLegal(fx.B.ctl, fx.nat.side,
						min_gap)) {
						disc_illegal++;
						continue;
					}
					fixes.push_back(fx);
				}
		}
		{
			int by_st[10] = { 0,0,0,0,0,0,0,0,0,0 };
			int by_kind[5] = { 0,0,0,0,0 };
			int faces_hit = 0, last_face = -1;
			int with_ctl = 0, hull2 = 0;
			for (const Fix& fx : fixes) {
				by_st[fx.stratum]++;
				by_kind[fx.nat.kind]++;
				if (fx.face != last_face) {
					faces_hit++;
					last_face = fx.face;
				}
				if (fx.B.ctl.side != 0)
					with_ctl++;
				if (fx.entry == 2)
					hull2++;
			}
			printf("exitfit: fixtures %d (gen %d | discard board %d "
				"short %d degen %d illegal %d hull2-fail %d)\n",
				static_cast<int>(fixes.size()), gen, disc_board,
				disc_short, disc_degen, disc_illegal, disc_h2);
			printf("exitfit:   strata hold/ramp/fwd/coast/duck/duckoff/"
				"chord/analog/brake/dive %d/%d/%d/%d/%d/%d/%d/%d/%d/"
				"%d | events exit/xfer/ground/end/hzn %d/%d/%d/%d/%d "
				"| faces %d | carried-ctl %d | hull2 entries %d\n",
				by_st[0], by_st[1], by_st[2], by_st[3], by_st[4],
				by_st[5], by_st[6], by_st[7], by_st[8], by_st[9],
				by_kind[0], by_kind[1], by_kind[2], by_kind[3],
				by_kind[4], faces_hit, with_ctl, hull2);
		}

		// ---- X1 CONTROL COMPLETENESS. Bitwise reproduction of a native
		// ride is IMPOSSIBLE for any single-encoding basis: two float
		// input encodings of the same wish differ in the last ulp, and
		// clips amplify that (measured: up to ~0.7u of positional drift
		// over a full ride). The representation claim is therefore made
		// at the INPUT CHANNEL: every tick's canonical emission must
		// realize the SAME PHYSICAL WISH as the native input - direction
		// to float precision, the same magnitude class (the engine only
		// distinguishes |wishvel| below maxspeed), the same duck - and
		// the replayed trajectory must track inside a small envelope
		// with the same event kind, tick and legality.
		{
			bool ok = fixes.size() >= 20;
			int replays = 0, bad_kind = 0, bad_tick = 0, bad_traj = 0;
			int bad_wish = 0, bad_class = 0, bad_duck = 0;
			float worst_pos = 0.f, worst_end = 0.f, worst_dir = 0.f;
			int analog_fx = 0;
			for (const Fix& fx : fixes) {
				// (a) the input-channel equivalence, tick by tick
				for (size_t k = 0; k < fx.nat.side.size(); ++k) {
					const PlayerState& sb = k == 0 ? fx.B.ps
						: fx.nat.traj[k - 1];
					const float h = atan2f(sb.vel.Y, sb.vel.X);
					float nx = 0.f, ny = 0.f;
					Fn::WishFromInput(fx.raw[k].yaw, fx.raw[k].fmove,
						fx.raw[k].smove, &nx, &ny);
					const float nmag = sqrtf(nx * nx + ny * ny);
					const bool nduck =
						(fx.raw[k].btn & IN_DUCK) != 0;
					if (nduck != (fx.nat.duck[k] != 0))
						bad_duck++;
					if (nmag < 1e-3f) {
						if (fx.nat.side[k] != 0)
							bad_wish++;
						continue;
					}
					float yc = 0.f, fc2 = 0.f, sc2 = 0.f;
					Air::WishInputs(h,
						static_cast<int>(fx.nat.side[k]),
						fx.nat.cosa[k], &yc, &fc2, &sc2);
					if (fx.nat.any_analog) {
						fc2 *= fx.nat.mag[k];
						sc2 *= fx.nat.mag[k];
					}
					float cx = 0.f, cy = 0.f;
					Fn::WishFromInput(yc, fc2, sc2, &cx, &cy);
					const float cmag = sqrtf(cx * cx + cy * cy);
					const float dd = fabsf(Steer::WrapPi(
						atan2f(ny, nx) - atan2f(cy, cx)));
					if (dd > worst_dir)
						worst_dir = dd;
					// The cosa chart QUANTIZES angle near the
					// degenerate directions: cos then acos loses
					// resolution as sqrt(ulp) at alpha in {0, pi}
					// (measured 8.3e-05 rad on brake-stratum ticks
					// with |cosa| ~ 1). That is a property of the
					// stored parameterization, not of the inverter -
					// and the packet witness (X1b) is bitwise exact
					// regardless. Well-conditioned band: 2e-5.
					const float tol_dir =
						fabsf(fx.nat.cosa[k]) > 0.999f ? 2e-4f
							: 2e-5f;
					if (dd > tol_dir)
						bad_wish++;
					if ((nmag >= p.maxspeed) != (cmag >= p.maxspeed)
						|| (nmag < p.maxspeed
							&& fabsf(nmag - cmag) > 0.05f))
						bad_class++;
				}
				// (b) the replay envelope + exact event equivalence
				std::vector<PlayerState> traj;
				Ride::Result rr = Ride::FlyRideSchedule(fx.B, w, p, g,
					fx.nat.side, fx.nat.cosa, fx.nat.duck,
					fx.nat.any_analog ? &fx.nat.mag : nullptr, &traj);
				if (fx.nat.any_analog)
					analog_fx++;
				replays++;
				if (!rr.legal || rr.kind != fx.nat.kind) {
					bad_kind++;
					continue;
				}
				if (rr.ticks != fx.nat.ticks) {
					bad_tick++;
					continue;
				}
				float wp = 0.f;
				const size_t nt = traj.size() < fx.nat.traj.size()
					? traj.size() : fx.nat.traj.size();
				for (size_t t2 = 0; t2 < nt; ++t2) {
					const float dp = Len(traj[t2].pos
						- fx.nat.traj[t2].pos);
					if (dp > wp)
						wp = dp;
				}
				const float de = Len(rr.end_state.pos
					- fx.nat.end_state.pos);
				if (wp > worst_pos)
					worst_pos = wp;
				if (de > worst_end)
					worst_end = de;
				if (wp > 2.f || de > 2.f)
					bad_traj++;
			}
			ok = ok && bad_kind == 0 && bad_tick == 0 && bad_traj == 0
				&& bad_wish == 0 && bad_class == 0 && bad_duck == 0;
			snprintf(buf, sizeof(buf), "%d native rides (%d analog) | "
				"wish-equiv: worst dir %.2e rad, mismatches dir/class/duck "
				"%d/%d/%d | event kind/tick %d/%d | traj envelope worst "
				"%.4fu S+ %.4fu (gate 2u), over %d", replays, analog_fx,
				static_cast<double>(worst_dir),
				bad_wish, bad_class, bad_duck, bad_kind, bad_tick,
				worst_pos, worst_end, bad_traj);
			check("X1 control completeness", ok, buf);
		}

		// ---- XH HORIZON SEMANTICS (standing law B-C): a schedule that
		// ends while still riding returns HORIZON, contributes no
		// transition, and is never an exit.
		{
			bool ok = false;
			char det[120] = "no long ride available";
			for (const Fix& fx : fixes) {
				if (fx.nat.kind != Ride::kAirExit
					|| fx.nat.ticks < 20)
					continue;
				const int cut = fx.nat.ticks / 2;
				std::vector<signed char> sd(fx.nat.side.begin(),
					fx.nat.side.begin() + cut);
				std::vector<float> cs(fx.nat.cosa.begin(),
					fx.nat.cosa.begin() + cut);
				std::vector<unsigned char> dk(fx.nat.duck.begin(),
					fx.nat.duck.begin() + cut);
				Ride::Result rr = Ride::FlyRideSchedule(fx.B, w, p, g,
					sd, cs, dk, nullptr);
				ok = rr.kind == Ride::kHorizon && rr.ticks == cut
					&& rr.legal;
				snprintf(det, sizeof(det), "schedule cut %d -> %d "
					"ticks returns HORIZON (kind %d), UNRESOLVED - "
					"not an exit", fx.nat.ticks, cut, rr.kind);
				break;
			}
			check("XH horizon is never an exit", ok, det);
		}

		// ---- X2a SERIALIZE/RESUME: checkpoint interior ticks,
		// serialize the COMPLETE S_B to bytes, reconstruct fresh,
		// replay the frozen suffix, demand the uninterrupted ride
		// bitwise - trajectory, event, tick and continuation state.
		// This is what actually crosses the operator boundary in the
		// future global solver.
		{
			bool ok = true;
			int n = 0, bad = 0;
			for (const Fix& fx : fixes) {
				std::vector<PlayerState> traj;
				std::vector<Steer::CtlState> ctlt;
				Ride::Result full = Ride::FlyRideSchedule(fx.B, w, p,
					g, fx.nat.side, fx.nat.cosa, fx.nat.duck,
					fx.nat.any_analog ? &fx.nat.mag : nullptr, &traj,
					&ctlt);
				if (!full.legal || full.ticks < 8)
					continue;
				for (int q2 = 1; q2 <= 3; ++q2) {
					const int j = full.ticks * q2 / 4;
					if (j < 1 || j > full.ticks - 1)
						continue;
					// serialize (the whole POD, bit-exact)
					std::vector<unsigned char> bytes(
						sizeof(PlayerState)
						+ sizeof(Steer::CtlState) + sizeof(int));
					memcpy(&bytes[0], &traj[static_cast<size_t>(
						j - 1)], sizeof(PlayerState));
					memcpy(&bytes[sizeof(PlayerState)],
						&ctlt[static_cast<size_t>(j - 1)],
						sizeof(Steer::CtlState));
					memcpy(&bytes[sizeof(PlayerState)
						+ sizeof(Steer::CtlState)], &fx.B.face,
						sizeof(int));
					Ride::BoundaryState R;
					memcpy(&R.ps, &bytes[0], sizeof(PlayerState));
					memcpy(&R.ctl, &bytes[sizeof(PlayerState)],
						sizeof(Steer::CtlState));
					memcpy(&R.face, &bytes[sizeof(PlayerState)
						+ sizeof(Steer::CtlState)], sizeof(int));
					std::vector<signed char> sd(
						fx.nat.side.begin() + j, fx.nat.side.end());
					std::vector<float> cs(
						fx.nat.cosa.begin() + j, fx.nat.cosa.end());
					std::vector<unsigned char> dk(
						fx.nat.duck.begin() + j, fx.nat.duck.end());
					std::vector<float> mg;
					const std::vector<float>* mgp = nullptr;
					if (fx.nat.any_analog) {
						mg.assign(fx.nat.mag.begin() + j,
							fx.nat.mag.end());
						mgp = &mg;
					}
					std::vector<PlayerState> straj;
					Ride::Result sub = Ride::FlyRideSchedule(R, w, p,
						g, sd, cs, dk, mgp, &straj);
					n++;
					bool same = sub.legal
						&& sub.kind == full.kind
						&& sub.ticks == full.ticks - j
						&& memcmp(&sub.end_state, &full.end_state,
							sizeof(PlayerState)) == 0;
					for (size_t t2 = 0; same && t2 < straj.size();
						++t2)
						if (memcmp(&straj[t2],
							&traj[static_cast<size_t>(j) + t2],
							sizeof(PlayerState)) != 0)
							same = false;
					if (!same) {
						bad++;
						ok = false;
					}
				}
			}
			snprintf(buf, sizeof(buf), "%d interior checkpoints "
				"serialize -> reconstruct -> replay BITWISE equal "
				"(traj, event, tick, S+); %d diverged", n, bad);
			check("X2a serialized resume equivalence", ok && n >= 30,
				buf);
		}

		// ---- X2b SEAM LEGALITY: the carried CtlState decides which
		// suffixes are ADMISSIBLE. A reversal legal under the true
		// carried dwell must be accepted; the same schedule under a
		// corrupted-younger dwell must be REFUSED at that tick. This is
		// the state the Air seam bug lived in - it changes legality,
		// never physics.
		{
			Ride::BoundaryState B0;
			bool have = false;
			for (const Fix& fx : fixes)
				if (fx.nat.kind == Ride::kAirExit
					&& fx.nat.ticks >= 12) {
					B0 = fx.B;
					have = true;
					break;
				}
			bool ok = false;
			char det[160] = "no fixture";
			if (have) {
				B0.ctl.side = 1;
				B0.ctl.age = 2;
				std::vector<signed char> sd(12,
					static_cast<signed char>(1));
				for (int k = 4; k < 12; ++k)
					sd[static_cast<size_t>(k)] =
						static_cast<signed char>(-1);
				std::vector<float> cs(12, 0.1f);
				std::vector<unsigned char> dk(12, 0);
				// carried age 2 + 4 ticks = 6 >= min_gap: legal
				Ride::Result a = Ride::FlyRideSchedule(B0, w, p, g,
					sd, cs, dk);
				Ride::BoundaryState B1 = B0;
				B1.ctl.age = 1;   // corrupted-younger: 1 + 4 = 5 < 6
				Ride::Result b = Ride::FlyRideSchedule(B1, w, p, g,
					sd, cs, dk);
				ok = a.legal && !b.legal && b.illegal_tick == 4;
				snprintf(det, sizeof(det), "reversal@4 with carried "
					"age 2 ACCEPTED (legal=%d); corrupted age 1 "
					"REFUSED at tick %d (legal=%d) - dwell crosses "
					"the seam", a.legal ? 1 : 0, b.illegal_tick,
					b.legal ? 1 : 0);
			}
			check("X2b carried-dwell seam legality", ok, det);
		}

		// ---- X2c DUCK/HULL SENSITIVITY: boundary states differing
		// ONLY in duck/hull fields must produce diverging rides on at
		// least one fixture - proof the fields are load-bearing and
		// that this matrix would catch their omission from S_B. (Both
		// values are legal at these kinematics; this is a sensitivity
		// probe, not a banked witness.)
		{
			int div_hull = 0, div_ducking = 0, n2 = 0, nh = 0;
			int first_div = -1;
			for (const Fix& fx : fixes) {
				// The hull probe pairs an ENGINE-CONSTRUCTED transient
				// (entry_kind 2: the factory actually unducked in
				// flight and the engine set hull_state = 2 itself)
				// against the same state normalized to the standing
				// hull. Poking hull_state UP on a standing state is
				// NOT a legal transient - it lacks the companion duck
				// fields and the engine normalizes it away (measured,
				// first exitfit run: 0/12 divergence that way).
				const bool hull_fx = fx.entry == 2
					&& fx.B.ps.hull_state == 2;
				if ((fx.entry != 0 && !hull_fx) || fx.nat.ticks < 10)
					continue;
				if (n2 >= 12 && nh > 0)
					break;
				n2++;
				std::vector<PlayerState> ta, tb, tc;
				Ride::Result ra = Ride::FlyRideSchedule(fx.B, w, p, g,
					fx.nat.side, fx.nat.cosa, fx.nat.duck, nullptr,
					&ta);
				if (hull_fx) {
					nh++;
					Ride::BoundaryState Bh = fx.B;
					Bh.ps.hull_state = 0;   // normalize the transient
					Ride::Result rb = Ride::FlyRideSchedule(Bh, w, p,
						g, fx.nat.side, fx.nat.cosa, fx.nat.duck,
						nullptr, &tb);
					bool dv = rb.kind != ra.kind
						|| rb.ticks != ra.ticks;
					int at2 = dv ? 0 : -1;
					const size_t n3 = tb.size() < ta.size()
						? tb.size() : ta.size();
					for (size_t t2 = 0; !dv && t2 < n3; ++t2)
						if (Len(tb[t2].pos - ta[t2].pos) > 0.01f) {
							dv = true;
							at2 = static_cast<int>(t2);
						}
					if (dv) {
						div_hull++;
						if (first_div < 0)
							first_div = at2;
					}
					continue;
				}
				Ride::BoundaryState Bd = fx.B;
				Bd.ps.ducking = true;   // mid-transition timer state
				Bd.ps.duck_timer_ms = 800.f;
				Ride::Result rc = Ride::FlyRideSchedule(Bd, w, p, g,
					fx.nat.side, fx.nat.cosa, fx.nat.duck, nullptr,
					&tc);
				auto diverges = [&](const std::vector<PlayerState>& x,
					const Ride::Result& rx, int* at) {
					if (rx.kind != ra.kind || rx.ticks != ra.ticks) {
						*at = 0;
						return true;
					}
					const size_t n3 = x.size() < ta.size()
						? x.size() : ta.size();
					for (size_t t2 = 0; t2 < n3; ++t2)
						if (Len(x[t2].pos - ta[t2].pos) > 0.01f) {
							*at = static_cast<int>(t2);
							return true;
						}
					return false;
				};
				int at = -1;
				if (diverges(tc, rc, &at))
					div_ducking++;
			}
			// The hull expectation asserts CONSISTENCY WITH THE
			// DECLARED MODEL, not an assumed sensitivity: Hulls::
			// unduck_* == stand_* since 2026-08-15 (the 62.5 transient
			// was removed as fit-noise), so the engine-built transient
			// MUST ride identically to the normalized state - any
			// divergence here means the collision model drifted. The
			// ducking/timer fields are the load-bearing ones (the
			// mid-ride transition applies the origin shift).
			const bool ok = nh > 0 && div_hull == 0 && div_ducking > 0;
			snprintf(buf, sizeof(buf), "%d probes (%d hull2) | engine "
				"transient vs normalized hull diverges on %d (first "
				"at tick %d, model expects 0) | ducking+timer diverges "
				"on %d - "
				"ducking/timer fields are load-bearing; hull2 rides as "
				"standing per the declared model", n2, nh,
				div_hull, first_div, div_ducking);
			check("X2c duck/hull fields load-bearing", ok, buf);
		}

		// ---- X7 REFERENCE REGRESSION (quarantined): a carve-generated
		// ride's realized NATIVE inputs invert and replay through the
		// canonical executor to the same separation. Reference rides
		// test the representation; they never define generation.
		{
			bool ok = false;
			char det[200] = "no fixture board";
			for (const Fix& fx : fixes) {
				if (fx.entry != 0 || fx.stratum != 1)
					continue;
				Carve::Target ct;
				ct.face = fx.face;
				const float h0 = atan2f(fx.B.ps.vel.Y,
					fx.B.ps.vel.X);
				ct.exit_heading = Steer::WrapPi(h0 + 0.5f);
				ct.max_ticks = 140;
				std::vector<float> knots;
				knots.push_back(h0);
				knots.push_back(Steer::WrapPi(h0 + 0.25f));
				knots.push_back(Steer::WrapPi(h0 + 0.5f));
				Carve::Result cr = Carve::RideHeadingSpline(fx.B.ps,
					w, p, ct, g, knots, nullptr, 140);
				if (!cr.exited || cr.tick < 8
					|| static_cast<int>(cr.yaw.size()) < cr.tick)
					continue;
				std::vector<NativeTick> nat;
				for (int k = 0; k < cr.tick; ++k) {
					NativeTick nt;
					nt.yaw = cr.yaw[static_cast<size_t>(k)];
					nt.fmove = cr.fmove[static_cast<size_t>(k)];
					nt.smove = cr.smove[static_cast<size_t>(k)];
					nt.btn = fx.B.ps.ducked ? IN_DUCK : 0;
					nat.push_back(nt);
				}
				NativeRun nr;
				RunNative(fx.B, w, p, g, nat, &nr);
				if (nr.degenerate)
					continue;
				if (!Ride::SchedLegal(fx.B.ctl, nr.side, min_gap))
					continue;
				Ride::Result rr = Ride::FlyRideSchedule(fx.B, w, p, g,
					nr.side, nr.cosa, nr.duck,
					nr.any_analog ? &nr.mag : nullptr);
				// carve books `tick` = first airborne tick = the
				// executor's peek tick, so ticks should agree exactly
				// (+-1 honesty margin for the seam bookkeeping).
				const int dt2 = rr.ticks - (cr.tick);
				ok = rr.legal && rr.kind == Ride::kAirExit
					&& (dt2 >= -1 && dt2 <= 1);
				snprintf(det, sizeof(det), "carve ride f%d exits "
					"t%d; canonical invert+replay exits t%d "
					"(kind %d) | speed2d carve %.1f vs exit_vel "
					"%.1f", fx.face, cr.tick, rr.ticks, rr.kind,
					cr.speed2d, Len2D(rr.exit_vel));
				break;
			}
			check("X7 carve reference regression", ok, det);
		}

		// ---- X1b EXACT WITNESS REPLAY (advisor 2026-08-19g): search
		// coordinates are compact and approximate-friendly; the
		// ACCEPTED witness is the frozen MoveInput packet sequence and
		// must replay BIT-FOR-BIT through FlyRideInputs. Two claims:
		// (a) a canonical run's captured packets reproduce it exactly;
		// (b) a native ride's own packets reproduce it exactly - the
		// 0.7u inversion drift lives in re-EMISSION, and the packet
		// witness removes it entirely.
		{
			bool ok = true;
			int n = 0, bad_can = 0, bad_nat = 0;
			for (const Fix& fx : fixes) {
				n++;
				// (a) canonical -> packets -> bitwise replay
				std::vector<PlayerState> traj;
				std::vector<Ride::MoveInput> pk;
				Ride::Result ra = Ride::FlyRideSchedule(fx.B, w, p, g,
					fx.nat.side, fx.nat.cosa, fx.nat.duck,
					fx.nat.any_analog ? &fx.nat.mag : nullptr, &traj,
					nullptr, &pk);
				std::vector<PlayerState> tb;
				Ride::Result rb = Ride::FlyRideInputs(fx.B, w, p, g,
					pk, &tb);
				bool same = rb.kind == ra.kind && rb.ticks == ra.ticks
					&& memcmp(&rb.end_state, &ra.end_state,
						sizeof(PlayerState)) == 0
					&& tb.size() == traj.size();
				for (size_t t2 = 0; same && t2 < tb.size(); ++t2)
					if (memcmp(&tb[t2], &traj[t2],
						sizeof(PlayerState)) != 0)
						same = false;
				if (!same) {
					bad_can++;
					ok = false;
				}
				// (b) native packets -> bitwise replay of the native
				// ride itself
				std::vector<Ride::MoveInput> np;
				for (int k = 0; k < fx.nat.ticks; ++k) {
					Ride::MoveInput mi;
					mi.yaw = fx.raw[static_cast<size_t>(k)].yaw;
					mi.fmove = fx.raw[static_cast<size_t>(k)].fmove;
					mi.smove = fx.raw[static_cast<size_t>(k)].smove;
					mi.buttons = fx.raw[static_cast<size_t>(k)].btn;
					np.push_back(mi);
				}
				std::vector<PlayerState> tn;
				Ride::Result rn = Ride::FlyRideInputs(fx.B, w, p, g,
					np, &tn);
				bool same2 = rn.kind == fx.nat.kind
					&& rn.ticks == fx.nat.ticks
					&& memcmp(&rn.end_state, &fx.nat.end_state,
						sizeof(PlayerState)) == 0
					&& tn.size() == fx.nat.traj.size();
				for (size_t t2 = 0; same2 && t2 < tn.size(); ++t2)
					if (memcmp(&tn[t2], &fx.nat.traj[t2],
						sizeof(PlayerState)) != 0)
						same2 = false;
				if (!same2) {
					bad_nat++;
					ok = false;
				}
			}
			snprintf(buf, sizeof(buf), "%d fixtures: canonical packets "
				"bitwise %d bad, NATIVE packets bitwise %d bad - the "
				"packet witness is the executable proof object", n,
				bad_can, bad_nat);
			check("X1b exact witness replay", ok && n >= 20, buf);
		}

		// ---- X3 COAST-THROUGH-DWELL (advisor 2026-08-19g): coasting
		// must not erase strafe history. side = 0 keeps the last
		// nonzero side and the dwell age CONTINUES INCREMENTING - a
		// reversal after coasts is judged against the age accumulated
		// through them, in both directions (too-young refused, aged
		// exactly to the minimum accepted). And fields the executor
		// ignores on a coast tick are physically meaningless: two
		// schedules differing only in coast-tick cosa must ride
		// bitwise identically, and CanonSchedule maps them to one
		// canonical form.
		{
			Ride::BoundaryState B0;
			bool have = false;
			for (const Fix& fx : fixes)
				if (fx.nat.kind == Ride::kAirExit
					&& fx.nat.ticks >= 12) {
					B0 = fx.B;
					have = true;
					break;
				}
			bool ok = false;
			char det[240] = "no fixture";
			if (have) {
				B0.ctl.side = 1;
				B0.ctl.age = 1;
				// [+1, +1, coast, coast, -1]: age 1+4 = 5 < 6 REFUSED
				std::vector<signed char> sa;
				sa.push_back(1); sa.push_back(1);
				sa.push_back(0); sa.push_back(0);
				sa.push_back(-1);
				std::vector<float> ca(5, 0.1f);
				std::vector<unsigned char> da(5, 0);
				Ride::Result r1 = Ride::FlyRideSchedule(B0, w, p, g,
					sa, ca, da);
				// [+1, +1, coast, coast, coast, -1]: age 1+5 = 6
				// ACCEPTED - proving coasts INCREMENT age (a frozen
				// age would refuse this one too)
				std::vector<signed char> sb = sa;
				sb.insert(sb.begin() + 2, static_cast<signed char>(0));
				std::vector<float> cb(6, 0.1f);
				std::vector<unsigned char> db(6, 0);
				Ride::Result r2 = Ride::FlyRideSchedule(B0, w, p, g,
					sb, cb, db);
				// coast-cosa physical identity + canonicalization
				std::vector<float> cb2 = cb;
				cb2[2] = 0.37f;
				cb2[3] = -0.61f;
				std::vector<PlayerState> ta, tb;
				Ride::Result r3a = Ride::FlyRideSchedule(B0, w, p, g,
					sb, cb, db, nullptr, &ta);
				Ride::Result r3b = Ride::FlyRideSchedule(B0, w, p, g,
					sb, cb2, db, nullptr, &tb);
				bool ident = r3a.kind == r3b.kind
					&& r3a.ticks == r3b.ticks
					&& ta.size() == tb.size();
				for (size_t t2 = 0; ident && t2 < ta.size(); ++t2)
					if (memcmp(&ta[t2], &tb[t2],
						sizeof(PlayerState)) != 0)
						ident = false;
				Ride::CanonSchedule(sb, &cb2, nullptr);
				bool canon = true;
				for (size_t t2 = 0; t2 < sb.size(); ++t2)
					if (sb[t2] == 0 && cb2[t2] != 1.f)
						canon = false;
				ok = !r1.legal && r1.illegal_tick == 4 && r2.legal
					&& ident && canon;
				snprintf(det, sizeof(det), "reversal after 2 coasts "
					"(age 5) REFUSED@%d; after 3 coasts (age 6) "
					"ACCEPTED=%d - coasts keep side, age increments "
					"through them | coast-cosa variants ride bitwise "
					"identical=%d, CanonSchedule pins them=%d",
					r1.illegal_tick, r2.legal ? 1 : 0, ident ? 1 : 0,
					canon ? 1 : 0);
			}
			check("X3 coast-through-dwell invariant", ok, det);
		}

		// ---- XE EVENT COVERAGE + TYPED TRANSITIONS (advisor
		// 2026-08-19g): CONTACT_TRANSFER / GROUND / END must be
		// exercised deterministically, not awaited from blind
		// generation, and each physical event must build a typed
		// ExitTransition whose witness carries exactly the booked
		// ticks. HORIZON must REFUSE to build one.
		{
			int n_xfer = 0, n_ground = 0, n_end = 0;
			int mk_bad = 0;
			int xfer_face = -2;
			// Targeted rides, ADJACENCY-DRIVEN (map-generic): for every
			// candidate transfer edge in the route graph, board `from`
			// up-slope aimed at `to`'s centroid and hold a gain-band
			// wish (stored cosa ~ -0.02, the wish that presses INTO the
			// face the way real rides do - a pure coast micro-skips off
			// within ~10 ticks). Rides that reach the neighbouring
			// brush produce CONTACT_TRANSFER; rides that die into the
			// valley floor produce GROUND. Both are exact engine
			// events, not constructions.
			for (const Route::Edge& eg : g.edges) {
				if (eg.from < 0 || eg.to < 0)
					continue;
				const Route::Face& fa = g.faces[
					static_cast<size_t>(eg.from)];
				const Route::Face& fb = g.faces[
					static_cast<size_t>(eg.to)];
				const float aim = atan2f(fb.centroid.Y
					- fa.centroid.Y, fb.centroid.X - fa.centroid.X);
				for (int v2 = 0; v2 < 2; ++v2) {
					Ride::BoundaryState B2;
					// `along` rotates the board approach so the
					// post-board velocity leans toward the target.
					const float paz2 = atan2f(fa.n.Y, fa.n.X);
					const float lean = Steer::WrapPi(aim
						- Steer::WrapPi(paz2 + 3.14159265f));
					if (!RideBoard(w, p, g, eg.from, 420.f, -150.f,
						0.6f * lean, 0, Steer::CtlState(), &B2,
						-0.3f, false))
						continue;
					const int Mv = 240;
					std::vector<signed char> sd(Mv,
						static_cast<signed char>(v2 ? -1 : 1));
					std::vector<float> cs(Mv, -0.02f);
					std::vector<unsigned char> dk(Mv, 0);
					std::vector<Ride::MoveInput> pk;
					Ride::Result rr = Ride::FlyRideSchedule(B2, w, p,
						g, sd, cs, dk, nullptr, nullptr, nullptr,
						&pk);
					if (rr.kind == Ride::kContactTransfer
						|| rr.kind == Ride::kGround) {
						Ride::ExitTransition tr;
						if (!Ride::MakeTransition(rr, g, pk, &tr)
							|| tr.dt != rr.ticks
							|| static_cast<int>(tr.witness.size())
								!= rr.ticks
							|| tr.kind != rr.kind) {
							mk_bad++;
							continue;
						}
						if (rr.kind == Ride::kContactTransfer) {
							n_xfer++;
							if (xfer_face == -2)
								xfer_face = tr.board_face;
						} else {
							n_ground++;
						}
					}
				}
			}
			// SYNTHETIC VALLEY (advisor 2026-08-19g: tiny controlled
			// geometry as a physics/event UNIT fixture, not a map
			// route). Two wedges whose surf slopes meet at a crease:
			// riding one slope down MUST contact the other brush -
			// the deterministic CONTACT_TRANSFER construction that
			// open real maps cannot supply (measured: 24/24 adjacency
			// probes on basictest end in air; its ramps never touch
			// within ride scope).
			{
				World sw;
				const float nz2 = 0.624695f, ny2 = 0.780869f;
				std::vector<Vec3> na;
				std::vector<float> da;
				// Wedge A: slope outward n = (0, -ny, +nz) through the
				// origin, box x in [-512, 512], y in [0, 500], z in
				// [-260, 320].
				na.push_back(Vec3(0.f, -ny2, nz2)); da.push_back(0.f);
				na.push_back(Vec3(1.f, 0.f, 0.f)); da.push_back(512.f);
				na.push_back(Vec3(-1.f, 0.f, 0.f)); da.push_back(512.f);
				na.push_back(Vec3(0.f, 1.f, 0.f)); da.push_back(500.f);
				na.push_back(Vec3(0.f, -1.f, 0.f)); da.push_back(0.f);
				na.push_back(Vec3(0.f, 0.f, 1.f)); da.push_back(320.f);
				na.push_back(Vec3(0.f, 0.f, -1.f)); da.push_back(260.f);
				bool okw = sw.AddTestBrush(na, da, o.hulls);
				// Wedge B: mirrored slope n = (0, +ny, +nz), y in
				// [-500, 0] - the crease is the y = 0, z = 0 line.
				std::vector<Vec3> nb;
				std::vector<float> db;
				nb.push_back(Vec3(0.f, ny2, nz2)); db.push_back(0.f);
				nb.push_back(Vec3(1.f, 0.f, 0.f)); db.push_back(512.f);
				nb.push_back(Vec3(-1.f, 0.f, 0.f)); db.push_back(512.f);
				nb.push_back(Vec3(0.f, 1.f, 0.f)); db.push_back(0.f);
				nb.push_back(Vec3(0.f, -1.f, 0.f)); db.push_back(500.f);
				nb.push_back(Vec3(0.f, 0.f, 1.f)); db.push_back(320.f);
				nb.push_back(Vec3(0.f, 0.f, -1.f)); db.push_back(260.f);
				okw = okw && sw.AddTestBrush(nb, db, o.hulls);
				sw.FinalizeTestWorld();
				Route::Graph sg;
				std::string serr;
				okw = okw && Route::Build(sw, &sg, 2000.f, &serr)
					&& sg.faces.size() >= 2;
				int fa = -1, fb2 = -1;
				for (size_t i2 = 0; okw && i2 < sg.faces.size(); ++i2) {
					if (sg.faces[i2].n.Y < -0.5f)
						fa = static_cast<int>(i2);
					if (sg.faces[i2].n.Y > 0.5f)
						fb2 = static_cast<int>(i2);
				}
				if (okw && fa >= 0 && fb2 >= 0) {
					Ride::BoundaryState B2;
					// vz matched to the slope (down-slope 420 u/s on a
					// 0.78/0.62 plane needs vz ~ -525 to run ALONG it;
					// -480 leans gently in) so the drop-in boards near
					// the placement instead of flying the crease.
					const bool bok = RideBoard(sw, p, sg, fa, 420.f,
						-480.f, 0.f, 0, Steer::CtlState(), &B2,
						-0.7f, true);
					if (bok) {
						const int Mv = 300;
						for (int v2 = 0; v2 < 2; ++v2) {
							std::vector<signed char> sd(Mv,
								static_cast<signed char>(
									v2 ? -1 : 1));
							std::vector<float> cs(Mv, -0.02f);
							std::vector<unsigned char> dk(Mv, 0);
							std::vector<Ride::MoveInput> pk;
							Ride::Result rr = Ride::FlyRideSchedule(
								B2, sw, p, sg, sd, cs, dk, nullptr,
								nullptr, nullptr, &pk);
							if (rr.kind != Ride::kContactTransfer)
								continue;
							Ride::ExitTransition tr;
							if (!Ride::MakeTransition(rr, sg, pk,
								&tr) || tr.dt != rr.ticks
								|| tr.kind != rr.kind) {
								mk_bad++;
								continue;
							}
							n_xfer++;
							if (xfer_face == -2)
								xfer_face = tr.board_face;
							break;
						}
					}
				}
			}
			// Natural fixtures count too (the reshuffled pool found
			// GROUND on its own).
			for (const Fix& fx : fixes) {
				if (fx.nat.kind == Ride::kContactTransfer)
					n_xfer++;
				if (fx.nat.kind == Ride::kGround)
					n_ground++;
			}
			// END: arm a zone straddling a known ride's mid path.
			bool end_ok = false;
			int end_tick = -1;
			for (const Fix& fx : fixes) {
				if (fx.nat.kind != Ride::kAirExit
					|| fx.nat.ticks < 10)
					continue;
				const PlayerState& mid = fx.nat.traj[
					static_cast<size_t>(fx.nat.ticks / 2)];
				const Vec3 zmin(mid.pos.X - 48.f, mid.pos.Y - 48.f,
					mid.pos.Z - 48.f);
				const Vec3 zmax(mid.pos.X + 48.f, mid.pos.Y + 48.f,
					mid.pos.Z + 48.f);
				std::vector<Ride::MoveInput> pk;
				Ride::Result rr = Ride::FlyRideSchedule(fx.B, w, p, g,
					fx.nat.side, fx.nat.cosa, fx.nat.duck,
					fx.nat.any_analog ? &fx.nat.mag : nullptr,
					nullptr, nullptr, &pk, &zmin, &zmax);
				Ride::ExitTransition tr;
				if (rr.kind == Ride::kEnd
					&& Ride::MakeTransition(rr, g, pk, &tr)
					&& tr.kind == Ride::kEnd
					&& tr.dt == rr.ticks) {
					end_ok = true;
					end_tick = rr.ticks;
					n_end++;
				}
				break;
			}
			// HORIZON refuses to become a transition.
			bool hzn_refused = false;
			for (const Fix& fx : fixes) {
				if (fx.nat.ticks < 12)
					continue;
				const int cut = fx.nat.ticks / 2;
				std::vector<signed char> sd(fx.nat.side.begin(),
					fx.nat.side.begin() + cut);
				std::vector<float> cs(fx.nat.cosa.begin(),
					fx.nat.cosa.begin() + cut);
				std::vector<unsigned char> dk(fx.nat.duck.begin(),
					fx.nat.duck.begin() + cut);
				std::vector<Ride::MoveInput> pk;
				Ride::Result rr = Ride::FlyRideSchedule(fx.B, w, p, g,
					sd, cs, dk, nullptr, nullptr, nullptr, &pk);
				Ride::ExitTransition tr;
				hzn_refused = rr.kind == Ride::kHorizon
					&& !Ride::MakeTransition(rr, g, pk, &tr);
				break;
			}
			const bool ok = n_xfer > 0 && n_ground > 0 && end_ok
				&& hzn_refused && mk_bad == 0;
			snprintf(buf, sizeof(buf), "CONTACT_TRANSFER x%d (first "
				"resolves to route face %d) | GROUND x%d | END x%d "
				"(zone entered t%d) | HORIZON refused a transition=%d "
				"| %d MakeTransition defects", n_xfer, xfer_face,
				n_ground, n_end, end_tick, hzn_refused ? 1 : 0,
				mk_bad);
			check("XE event coverage + typed transitions", ok, buf);
		}

		printf("exitfit: %d passed, %d failed | %s\n", pass, fail,
			fail == 0 ? "RIDE REPRESENTATION GATE GREEN"
				: "RIDE REPRESENTATION GATE RED");
		fflush(stdout);
		return fail == 0 ? 0 : 2;
	}

	// ================= exitfrontier: STAGE-3 GATES (advisor
	// 2026-08-19h). The witnessed frontier W_F(B) is a LOWER
	// approximation of the operator R_F(B): these gates prove the
	// finite representation's laws - exact replay, event typing,
	// monotone knowledge, no false dominance, reopening,
	// CONTACT_TRANSFER composition parity, proposal quarantine - and
	// close with a constructive recoverability smoke test so
	// representation success is not confused with search coverage.
	int CmdExitFrontier(const std::string& map_path,
	                    const ReplayOpts& o) {
		World w;
		std::string err;
		w.true_interval_corner = o.corner_true;
		if (!w.Load(map_path, o.hulls, &err, o.edge_bevels)) {
			printf("LOAD FAILED (bsp): %s\n", err.c_str());
			return 1;
		}
		Route::Graph g;
		if (!Route::Build(w, &g, 2000.f, &err)) {
			printf("exitfrontier: %s\n", err.c_str());
			return 1;
		}
		const MoveParams& p = o.params;
		int pass = 0, fail = 0;
		char buf[300];
		auto check = [&](const char* name, bool ok, const char* det) {
			printf("exitfrontier: %-34s %s | %s\n", name,
				ok ? "PASS" : "FAIL", det);
			if (ok) pass++; else fail++;
		};
		int fi = -1;
		for (size_t i = 0; i < g.faces.size(); ++i)
			if (sqrtf(g.faces[i].n.X * g.faces[i].n.X
				+ g.faces[i].n.Y * g.faces[i].n.Y) > 1e-4f) {
				fi = static_cast<int>(i);
				break;
			}
		if (fi < 0) {
			printf("exitfrontier: no surfable face\n");
			return 1;
		}
		Ride::BoundaryState B;
		if (!RideBoard(w, p, g, fi, 700.f, -120.f, 0.3f, 0,
			Steer::CtlState(), &B)) {
			printf("exitfrontier: board construction failed\n");
			return 1;
		}

		// ---- build at level 0, then refine twice on the SAME object.
		ExitField::WitnessFrontier wf;
		wf.B = B;
		ExitField::BuildFrontier(&wf, w, p, g, 0);
		// snapshot of level-0 knowledge for F3
		std::vector<unsigned long long> seen0 = wf.seen_hash;
		std::vector<unsigned long long> act0;
		int comp0 = 0;
		unsigned reopen_key = 0;
		int reopen_active0 = -1;
		for (const ExitField::Partition& P : wf.parts) {
			for (const Ride::ExitTransition& t : P.active)
				act0.push_back(ExitField::WitnessHash(t.witness));
			if (P.status == ExitField::kCompressed) {
				comp0++;
				if (!reopen_key && P.omitted_by_cap
					&& !P.deferred_hash.empty()) {
					reopen_key = P.key;
					reopen_active0 =
						static_cast<int>(P.active.size());
				}
			}
		}
		ExitField::BuildFrontier(&wf, w, p, g, 1);
		ExitField::BuildFrontier(&wf, w, p, g, 2);

		// ---- summary
		{
			int by_kind[5] = { 0,0,0,0,0 };
			int actives = 0, deferred = 0, unsupported = 0;
			for (const ExitField::Partition& P : wf.parts) {
				by_kind[P.kind < 5 ? P.kind : 4]++;
				actives += static_cast<int>(P.active.size());
				deferred += static_cast<int>(P.deferred_hash.size());
				if (P.continuation_unsupported)
					unsupported++;
			}
			printf("exitfrontier: W_F(B) after L0->L2 | proposals %d "
				"(illegal %d, horizon %d) | known %d | partitions %d "
				"(air %d xfer %d ground %d end %d) | active %d "
				"deferred %d | ground-unsupported %d\n",
				wf.proposals, wf.illegal, wf.horizon, wf.known,
				static_cast<int>(wf.parts.size()), by_kind[0],
				by_kind[1], by_kind[2], by_kind[3], actives, deferred,
				unsupported);
		}

		// ---- F1 EXACT FRONTIER TRUTH: every active member replays
		// packet-exactly to its recorded event, dt and S+.
		{
			int n = 0, bad = 0;
			for (const ExitField::Partition& P : wf.parts)
				for (const Ride::ExitTransition& t : P.active) {
					n++;
					Ride::Result rr = Ride::FlyRideInputs(B, w, p, g,
						t.witness);
					if (!(rr.legal && rr.kind == t.kind
						&& rr.ticks == t.dt
						&& memcmp(&rr.end_state, &t.s_plus,
							sizeof(PlayerState)) == 0))
						bad++;
				}
			snprintf(buf, sizeof(buf), "%d active members replay "
				"bitwise to (kind, dt, S+); %d failures", n, bad);
			check("F1 exact frontier truth", n > 0 && bad == 0, buf);
		}

		// ---- F2 EVENT PARTITION INTEGRITY.
		{
			int bad = 0, ground_flagged = 0, ground_parts = 0;
			for (const ExitField::Partition& P : wf.parts) {
				for (const Ride::ExitTransition& t : P.active) {
					if (t.kind != P.kind)
						bad++;
					if (t.kind == Ride::kContactTransfer
						&& t.contact_brush < 0)
						bad++;
				}
				if (P.kind == Ride::kGround) {
					ground_parts++;
					if (P.continuation_unsupported)
						ground_flagged++;
				}
			}
			snprintf(buf, sizeof(buf), "kind/metadata mismatches %d | "
				"GROUND partitions %d, all marked "
				"continuation-unsupported (%d) - preserved, never "
				"dead", bad, ground_parts, ground_flagged);
			check("F2 event partition integrity",
				bad == 0 && ground_flagged == ground_parts, buf);
		}

		// ---- F3 MONOTONE WITNESS KNOWLEDGE: refinement recompresses,
		// never forgets. Every level-0 hash survives; every level-0
		// ACTIVE member is still active or deferred in its partition.
		{
			bool ok = true;
			for (unsigned long long h : seen0) {
				bool found = false;
				for (unsigned long long s : wf.seen_hash)
					if (s == h) {
						found = true;
						break;
					}
				if (!found)
					ok = false;
			}
			int lost = 0;
			for (unsigned long long h : act0) {
				bool found = false;
				for (const ExitField::Partition& P : wf.parts) {
					for (const Ride::ExitTransition& t : P.active)
						if (ExitField::WitnessHash(t.witness) == h)
							found = true;
					for (unsigned long long dh : P.deferred_hash)
						if (dh == h)
							found = true;
				}
				if (!found) {
					lost++;
					ok = false;
				}
			}
			snprintf(buf, sizeof(buf), "L0 knowledge %d hashes all "
				"survive to L2 (known %d); L0 actives lost %d",
				static_cast<int>(seen0.size()), wf.known, lost);
			check("F3 monotone witness knowledge", ok, buf);
		}

		// ---- F4 NO FALSE DOMINANCE: compression marks members
		// DEFERRED, never erases them - a capped partition still
		// carries every known member as active or deferred hash.
		{
			bool ok = true;
			int comp = 0;
			for (const ExitField::Partition& P : wf.parts)
				if (P.status == ExitField::kCompressed) {
					comp++;
					if (P.deferred_hash.empty()
						&& static_cast<int>(P.active.size())
							<= ExitField::ActiveCap(wf.level))
						ok = false;
				}
			snprintf(buf, sizeof(buf), "%d compressed partitions (L0 "
				"had %d), every one carries deferred hashes - "
				"status says COMPRESSED, nothing says dominated",
				comp, comp0);
			check("F4 no false dominance", ok, buf);
		}

		// ---- F5 REOPENING: take a partition the L2 cap actually
		// compressed, refine once more, and require a previously
		// DEFERRED member to MATERIALIZE as an active representative -
		// the deterministic provenance regenerating exactly what it
		// deferred.
		{
			unsigned key2 = 0;
			int a_before = -1;
			std::vector<unsigned long long> was_deferred;
			for (const ExitField::Partition& P : wf.parts)
				if (P.status == ExitField::kCompressed
					&& !P.deferred_hash.empty()) {
					key2 = P.key;
					a_before = static_cast<int>(P.active.size());
					was_deferred = P.deferred_hash;
					break;
				}
			bool ok = false;
			int a_after = -1;
			bool mat = false;
			if (key2) {
				ExitField::BuildFrontier(&wf, w, p, g, 3);
				for (const ExitField::Partition& P : wf.parts)
					if (P.key == key2) {
						a_after =
							static_cast<int>(P.active.size());
						// ANY member of the L2 deferred set now
						// active = the reopen provenance worked;
						// which one wins slots is a resolution
						// detail, not the law.
						for (const Ride::ExitTransition& t
							: P.active) {
							const unsigned long long th2 =
								ExitField::WitnessHash(t.witness);
							for (unsigned long long dh
								: was_deferred)
								if (dh == th2)
									mat = true;
						}
					}
				ok = a_after > a_before && mat;
			}
			snprintf(buf, sizeof(buf), "compressed partition %08x: "
				"active %d at L2 -> %d at L3; a previously deferred "
				"member materialized as active = %d", key2,
				a_before, a_after, mat ? 1 : 0);
			check("F5 partition reopening", ok, buf);
		}

		// ---- F6 CONTACT_TRANSFER COMPOSITION PARITY (the chained
		// seam): continuous execution across the A->B contact plus a
		// suffix on B must equal chaining ExitTransition(A) into a
		// fresh ride context built from its S+.
		{
			bool ok = false;
			char det[240] = "synthetic valley unavailable";
			World sw;
			const float nz2 = 0.624695f, ny2 = 0.780869f;
			std::vector<Vec3> na;
			std::vector<float> da;
			na.push_back(Vec3(0.f, -ny2, nz2)); da.push_back(0.f);
			na.push_back(Vec3(1.f, 0.f, 0.f)); da.push_back(512.f);
			na.push_back(Vec3(-1.f, 0.f, 0.f)); da.push_back(512.f);
			na.push_back(Vec3(0.f, 1.f, 0.f)); da.push_back(500.f);
			na.push_back(Vec3(0.f, -1.f, 0.f)); da.push_back(0.f);
			na.push_back(Vec3(0.f, 0.f, 1.f)); da.push_back(320.f);
			na.push_back(Vec3(0.f, 0.f, -1.f)); da.push_back(260.f);
			bool okw = sw.AddTestBrush(na, da, o.hulls);
			std::vector<Vec3> nb;
			std::vector<float> db;
			nb.push_back(Vec3(0.f, ny2, nz2)); db.push_back(0.f);
			nb.push_back(Vec3(1.f, 0.f, 0.f)); db.push_back(512.f);
			nb.push_back(Vec3(-1.f, 0.f, 0.f)); db.push_back(512.f);
			nb.push_back(Vec3(0.f, 1.f, 0.f)); db.push_back(0.f);
			nb.push_back(Vec3(0.f, -1.f, 0.f)); db.push_back(500.f);
			nb.push_back(Vec3(0.f, 0.f, 1.f)); db.push_back(320.f);
			nb.push_back(Vec3(0.f, 0.f, -1.f)); db.push_back(260.f);
			okw = okw && sw.AddTestBrush(nb, db, o.hulls);
			sw.FinalizeTestWorld();
			Route::Graph sg;
			std::string serr;
			okw = okw && Route::Build(sw, &sg, 2000.f, &serr)
				&& sg.faces.size() >= 2;
			int fa = -1;
			for (size_t i2 = 0; okw && i2 < sg.faces.size(); ++i2)
				if (sg.faces[i2].n.Y < -0.5f)
					fa = static_cast<int>(i2);
			Ride::BoundaryState BA;
			if (okw && fa >= 0 && RideBoard(sw, p, sg, fa, 420.f,
				-480.f, 0.f, 0, Steer::CtlState(), &BA, -0.7f,
				true)) {
				const int Mv = 300;
				std::vector<signed char> sd(Mv,
					static_cast<signed char>(1));
				std::vector<float> cs(Mv, -0.02f);
				std::vector<unsigned char> dk(Mv, 0);
				std::vector<Ride::MoveInput> pkA;
				Ride::Result ra = Ride::FlyRideSchedule(BA, sw, p, sg,
					sd, cs, dk, nullptr, nullptr, nullptr, &pkA);
				Ride::ExitTransition tr;
				if (ra.kind == Ride::kContactTransfer
					&& Ride::MakeTransition(ra, sg, pkA, &tr)
					&& tr.board_face >= 0) {
					// CONTINUOUS: raw MoveTick through the transfer
					// tick, then 12 more canonical gain ticks emitted
					// from the LIVE state (recorded as packets).
					PlayerState s = BA.ps;
					for (int k = 0; k < ra.ticks; ++k) {
						TickEvents ev;
						MoveTick(s, sw, p, pkA[
							static_cast<size_t>(k)].pitch,
							pkA[static_cast<size_t>(k)].yaw,
							pkA[static_cast<size_t>(k)].fmove,
							pkA[static_cast<size_t>(k)].smove,
							pkA[static_cast<size_t>(k)].umove,
							pkA[static_cast<size_t>(k)].buttons,
							&ev);
					}
					const bool handoff_exact = memcmp(&s, &tr.s_plus,
						sizeof(PlayerState)) == 0;
					std::vector<Ride::MoveInput> pkS;
					std::vector<PlayerState> cont;
					std::vector<TickEvents> cev;
					for (int k = 0; k < 12; ++k) {
						const float s2d = Len2D(s.vel);
						const float h2 = s2d > 1.f
							? atan2f(s.vel.Y, s.vel.X) : 0.f;
						Ride::MoveInput mi;
						mi.yaw = h2 * 57.2957795f;
						if (s2d > 1.f)
							Air::WishInputs(h2, 1, -0.02f, &mi.yaw,
								&mi.fmove, &mi.smove);
						TickEvents ev;
						MoveTick(s, sw, p, mi.pitch, mi.yaw,
							mi.fmove, mi.smove, mi.umove,
							mi.buttons, &ev);
						pkS.push_back(mi);
						cont.push_back(s);
						cev.push_back(ev);
					}
					// COMPOSED: rebuild the ride context from the
					// transition's typed continuation alone. The
					// composed run may lawfully STOP EARLY at its own
					// next boundary event (at a crease the hull can
					// touch wedge A again while riding B - a real
					// transition, not a parity failure); parity
					// demands (a) bitwise handoff, (b) bitwise state
					// agreement for every tick the composed run
					// booked, (c) the composed event corresponds to a
					// real contact/ground in the continuous run at
					// that same tick.
					Ride::BoundaryState BB;
					BB.ps = tr.s_plus;
					BB.ctl = tr.ctl_plus;
					BB.face = tr.board_face;
					std::vector<PlayerState> comp;
					Ride::Result rb = Ride::FlyRideInputs(BB, sw, p,
						sg, pkS, &comp);
					bool same = handoff_exact && rb.ticks >= 1
						&& comp.size() <= cont.size();
					for (size_t k = 0; same && k < comp.size(); ++k)
						if (memcmp(&comp[k], &cont[k],
							sizeof(PlayerState)) != 0)
							same = false;
					// STRENGTHENED (advisor 2026-08-19i): the composed
					// event must be the SAME EARLIEST post-handoff
					// boundary event as the instrumented continuous
					// run - no earlier boundary skipped, same tick,
					// same kind, same contacted face where applicable.
					// Walk the continuous suffix through the SAME
					// classification the executor uses (vs face B).
					int exp_tick = -1, exp_kind = -1, exp_brush = -1;
					const Route::Face& fcB = sg.faces[
						static_cast<size_t>(tr.board_face)];
					for (size_t k = 0; k < cev.size()
						&& exp_tick < 0; ++k) {
						bool onB = false;
						int oth = -1;
						for (int c2 = 0; c2 < cev[k].ncontacts;
							++c2) {
							if (cev[k].contact_brush[c2]
								== fcB.brush
								&& cev[k].contact_plane[c2]
									== fcB.side)
								onB = true;
							else if (oth < 0)
								oth = c2;
						}
						if (cont[k].on_ground) {
							exp_tick = static_cast<int>(k) + 1;
							exp_kind = Ride::kGround;
						} else if (oth >= 0) {
							exp_tick = static_cast<int>(k) + 1;
							exp_kind = Ride::kContactTransfer;
							exp_brush = cev[k].contact_brush[oth];
						} else if (!onB) {
							exp_tick = static_cast<int>(k) + 1;
							exp_kind = Ride::kAirExit;
						}
					}
					bool ev_ok = exp_tick == rb.ticks
						&& exp_kind == rb.kind
						&& (rb.kind != Ride::kContactTransfer
							|| exp_brush == rb.contact_brush);
					ok = same && ev_ok;
					snprintf(det, sizeof(det), "A rides %d ticks, "
						"contacts B; handoff S+ bitwise=%d; composed "
						"suffix books %d/%d ticks bitwise=%d; "
						"composed event (kind %d t%d) == earliest "
						"continuous boundary (kind %d t%d)=%d",
						ra.ticks, handoff_exact ? 1 : 0, rb.ticks,
						static_cast<int>(cont.size()), same ? 1 : 0,
						rb.kind, rb.ticks, exp_kind, exp_tick,
						ev_ok ? 1 : 0);
				}
			}
			check("F6 transfer composition parity", ok, det);
		}

		// ---- F7 PROPOSAL QUARANTINE: the production frontier reads
		// no tapes and no Carve machinery; toggling the ExitDoomed
		// in-sim cull changes nothing, and two builds are bitwise
		// deterministic.
		{
			ExitField::WitnessFrontier w2;
			w2.B = B;
			const bool saved = Carve::g_doom_cull;
			Carve::g_doom_cull = !saved;
			ExitField::BuildFrontier(&w2, w, p, g, 0);
			ExitField::BuildFrontier(&w2, w, p, g, 1);
			ExitField::BuildFrontier(&w2, w, p, g, 2);
			Carve::g_doom_cull = saved;
			bool same = w2.known == wf.known
				&& w2.seen_hash.size() == wf.seen_hash.size();
			for (size_t i = 0; same && i < w2.seen_hash.size(); ++i)
				if (w2.seen_hash[i] != wf.seen_hash[i])
					same = false;
			snprintf(buf, sizeof(buf), "rebuild with ExitDoomed cull "
				"toggled: identical knowledge (%d transitions, "
				"hash-for-hash) - no tape, no Carve, no cull on the "
				"proposal path", w2.known);
			check("F7 proposal quarantine", same, buf);
		}

		// ---- F8 CONSTRUCTIVE RECOVERABILITY SMOKE: hidden legal
		// canonical rides OUTSIDE the proposal families; production
		// gets only B. Recovery = a frontier member (active or
		// deferred) in the hidden transition's partition with a
		// nearby duration. Exposes structural coverage holes, not a
		// benchmark to perfect.
		{
			int n = 0, rec = 0;
			char miss[80] = "";
			for (int j = 0; j < 8; ++j) {
				const int M = 48 + 24 * (j % 3);
				std::vector<signed char> sd(
					static_cast<size_t>(M),
					static_cast<signed char>(j & 1 ? -1 : 1));
				std::vector<float> cs(static_cast<size_t>(M),
					j % 3 == 0 ? 0.14f : (j % 3 == 1 ? 0.5f
						: -0.06f));
				std::vector<unsigned char> dk(
					static_cast<size_t>(M), 0);
				// off-family structure: odd-tick reversal, coast
				// pulse, duck pulse
				const int r = 11 + 2 * (j % 4);
				if (j < 4)
					for (int k = r; k < M; ++k)
						sd[static_cast<size_t>(k)] =
							static_cast<signed char>(
								j & 1 ? 1 : -1);
				if (j >= 4 && j < 6)
					for (int k = r; k < r + 5 && k < M; ++k)
						sd[static_cast<size_t>(k)] = 0;
				if (j >= 6)
					for (int k = r; k < r + 9 && k < M; ++k)
						dk[static_cast<size_t>(k)] = 1;
				Ride::CanonSchedule(sd, &cs, nullptr);
				std::vector<Ride::MoveInput> pk;
				Ride::Result rr = Ride::FlyRideSchedule(B, w, p, g,
					sd, cs, dk, nullptr, nullptr, nullptr, &pk);
				Ride::ExitTransition zt;
				if (!rr.legal || rr.kind == Ride::kHorizon
					|| !Ride::MakeTransition(rr, g, pk, &zt))
					continue;
				n++;
				const unsigned key = ExitField::PartKey(zt, g, B.face);
				bool found = false;
				for (const ExitField::Partition& P : wf.parts) {
					if (P.key != key)
						continue;
					for (const Ride::ExitTransition& t : P.active)
						if (abs(t.dt - zt.dt)
							<= (zt.dt / 5 > 6 ? zt.dt / 5 : 6))
							found = true;
					if (!P.deferred_hash.empty())
						found = true;   // known members live here too
				}
				if (found)
					rec++;
				else if (miss[0] == 0)
					snprintf(miss, sizeof(miss), " | first miss: "
						"kind %d dt %d key %08x", zt.kind, zt.dt,
						key);
			}
			snprintf(buf, sizeof(buf), "%d hidden legal rides "
				"(off-family: odd reversals, coast pulses, duck "
				"pulses); %d recovered by partition+duration%s", n,
				rec, miss);
			check("F8 recoverability smoke", n >= 5 && rec * 2 >= n,
				buf);
		}

		// ---- F9 SIMULTANEOUS-EVENT PRECEDENCE (advisor 2026-08-19i):
		// when one tick carries several boundary events the winner must
		// be DETERMINISTIC AND SEMANTIC, not whichever branch runs
		// first. The documented order is END > GROUND >
		// CONTACT_TRANSFER > AIR_EXIT: entering the finish during a
		// tick means the run is DONE regardless of what else that tick
		// touched, grounding outranks an incidental brush contact, and
		// any contact outranks separation. Constructed here: the
		// synthetic-valley transfer tick with an END zone straddling
		// that same tick's position - the executor must say END.
		{
			bool ok = false;
			char det[200] = "no transfer fixture";
			World sw;
			const float nz2 = 0.624695f, ny2 = 0.780869f;
			std::vector<Vec3> na;
			std::vector<float> da;
			na.push_back(Vec3(0.f, -ny2, nz2)); da.push_back(0.f);
			na.push_back(Vec3(1.f, 0.f, 0.f)); da.push_back(512.f);
			na.push_back(Vec3(-1.f, 0.f, 0.f)); da.push_back(512.f);
			na.push_back(Vec3(0.f, 1.f, 0.f)); da.push_back(500.f);
			na.push_back(Vec3(0.f, -1.f, 0.f)); da.push_back(0.f);
			na.push_back(Vec3(0.f, 0.f, 1.f)); da.push_back(320.f);
			na.push_back(Vec3(0.f, 0.f, -1.f)); da.push_back(260.f);
			bool okw = sw.AddTestBrush(na, da, o.hulls);
			std::vector<Vec3> nb;
			std::vector<float> db;
			nb.push_back(Vec3(0.f, ny2, nz2)); db.push_back(0.f);
			nb.push_back(Vec3(1.f, 0.f, 0.f)); db.push_back(512.f);
			nb.push_back(Vec3(-1.f, 0.f, 0.f)); db.push_back(512.f);
			nb.push_back(Vec3(0.f, 1.f, 0.f)); db.push_back(0.f);
			nb.push_back(Vec3(0.f, -1.f, 0.f)); db.push_back(500.f);
			nb.push_back(Vec3(0.f, 0.f, 1.f)); db.push_back(320.f);
			nb.push_back(Vec3(0.f, 0.f, -1.f)); db.push_back(260.f);
			okw = okw && sw.AddTestBrush(nb, db, o.hulls);
			sw.FinalizeTestWorld();
			Route::Graph sg;
			std::string serr;
			okw = okw && Route::Build(sw, &sg, 2000.f, &serr)
				&& sg.faces.size() >= 2;
			int fa = -1;
			for (size_t i2 = 0; okw && i2 < sg.faces.size(); ++i2)
				if (sg.faces[i2].n.Y < -0.5f)
					fa = static_cast<int>(i2);
			Ride::BoundaryState BA;
			if (okw && fa >= 0 && RideBoard(sw, p, sg, fa, 420.f,
				-480.f, 0.f, 0, Steer::CtlState(), &BA, -0.7f,
				true)) {
				const int Mv = 300;
				std::vector<signed char> sd(Mv,
					static_cast<signed char>(1));
				std::vector<float> cs(Mv, -0.02f);
				std::vector<unsigned char> dk(Mv, 0);
				std::vector<PlayerState> traj;
				Ride::Result r1 = Ride::FlyRideSchedule(BA, sw, p, sg,
					sd, cs, dk, nullptr, &traj);
				if (r1.kind == Ride::kContactTransfer
					&& r1.ticks >= 1) {
					const Vec3& q2 = traj[static_cast<size_t>(
						r1.ticks - 1)].pos;
					// +-3u: tick spacing is ~7u, so the zone is
					// entered ON the transfer tick, not before -
					// genuine simultaneity.
					const Vec3 zmin(q2.X - 3.f, q2.Y - 3.f,
						q2.Z - 3.f);
					const Vec3 zmax(q2.X + 3.f, q2.Y + 3.f,
						q2.Z + 3.f);
					Ride::Result r2 = Ride::FlyRideSchedule(BA, sw, p,
						sg, sd, cs, dk, nullptr, nullptr, nullptr,
						nullptr, &zmin, &zmax);
					ok = r2.kind == Ride::kEnd
						&& r2.ticks == r1.ticks;
					snprintf(det, sizeof(det), "transfer tick t%d "
						"with an END zone on the same tick "
						"classifies END (kind %d t%d) - precedence "
						"END > GROUND > TRANSFER > AIR is "
						"deterministic", r1.ticks, r2.kind,
						r2.ticks);
				}
			}
			check("F9 simultaneous-event precedence", ok, det);
		}

		printf("exitfrontier: %d passed, %d failed | %s\n", pass, fail,
			fail == 0 ? "STAGE-3 FRONTIER GATE GREEN"
				: "STAGE-3 FRONTIER GATE RED");
		fflush(stdout);
		return fail == 0 ? 0 : 2;
	}

	// ================= exitenv: STAGE-4 ENVELOPE CERTIFICATION
	// (advisor 2026-08-19i; derivation in SolverExitField.h). The
	// constructive gates FALSIFY the bound - the derivation is the
	// proof - and E8 deliberately breaks the semantics to verify the
	// suite is strong enough to notice (the B5-passed-while-broken
	// lesson).
	int CmdExitEnv(const std::string& map_path, const ReplayOpts& o) {
		World w;
		std::string err;
		w.true_interval_corner = o.corner_true;
		if (!w.Load(map_path, o.hulls, &err, o.edge_bevels)) {
			printf("LOAD FAILED (bsp): %s\n", err.c_str());
			return 1;
		}
		Route::Graph g;
		if (!Route::Build(w, &g, 2000.f, &err)) {
			printf("exitenv: %s\n", err.c_str());
			return 1;
		}
		const MoveParams& p = o.params;
		int pass = 0, fail = 0;
		char buf[300];
		auto check = [&](const char* name, bool ok, const char* det) {
			printf("exitenv: %-36s %s | %s\n", name,
				ok ? "PASS" : "FAIL", det);
			if (ok) pass++; else fail++;
		};
		int fi = -1;
		for (size_t i = 0; i < g.faces.size(); ++i)
			if (sqrtf(g.faces[i].n.X * g.faces[i].n.X
				+ g.faces[i].n.Y * g.faces[i].n.Y) > 1e-4f) {
				fi = static_cast<int>(i);
				break;
			}
		Ride::BoundaryState B;
		if (fi < 0 || !RideBoard(w, p, g, fi, 700.f, -120.f, 0.3f, 0,
			Steer::CtlState(), &B)) {
			printf("exitenv: no board fixture\n");
			return 1;
		}
		const float cap2 = p.air_speed_cap * p.air_speed_cap;
		const float hull_slack = 2.f * p.gravity
			* B.ps.gravity_scale * p.duck_air_shift;

		// A clean-air state factory for single-tick probes: high above
		// the map so nothing contacts (falsification probes, never
		// witnesses).
		auto air_state = [&](float sp, float hd, float vz) {
			PlayerState s = B.ps;
			s.pos.Z += 4000.f;
			s.vel = Vec3(cosf(hd) * sp, sinf(hd) * sp, vz);
			s.on_ground = false;
			s.ducked = false;
			s.ducking = false;
			s.duck_timer_ms = 0.f;
			s.hull_state = 0;
			return s;
		};

		// ---- E1 EXACT PHASE + BALLISTIC CONSERVATION: pure coast
		// flight conserves E at tick boundaries to float rounding; the
		// measured drift calibrates kUEnergyEpsTick.
		float worst_drift = 0.f;
		{
			for (int c = 0; c < 6; ++c) {
				PlayerState s = air_state(c % 2 ? 900.f : 300.f,
					0.7f * static_cast<float>(c),
					c % 3 == 0 ? -400.f : 150.f);
				float e0 = ExitField::EBoundary(s, p);
				for (int k = 0; k < 60; ++k) {
					TickEvents ev;
					MoveTick(s, w, p, 0.f, 0.f, 0.f, 0.f, 0.f, 0,
						&ev);
					if (ev.ncontacts > 0 || s.on_ground)
						break;
					const float e1 = ExitField::EBoundary(s, p);
					const float dr = fabsf(e1 - e0);
					if (dr > worst_drift)
						worst_drift = dr;
					e0 = e1;
				}
			}
			snprintf(buf, sizeof(buf), "360 coast ticks at the "
				"declared boundary phase: worst |dE| %.4f (eps/tick "
				"%.2f) - the leapfrog conserves E, phase confirmed",
				worst_drift, ExitField::kUEnergyEpsTick);
			check("E1 exact phase + conservation",
				worst_drift < ExitField::kUEnergyEpsTick, buf);
		}

		// ---- E2 WISH-WORK CEILING: no legal single wish tick exceeds
		// cap^2, across speeds, headings, vz, cosa, analog magnitudes
		// and native key chords.
		float wish_max = -1e9f;
		{
			int n = 0, viol = 0;
			for (int si = 0; si < 4; ++si)
				for (int hi2 = 0; hi2 < 4; ++hi2)
					for (int vi = 0; vi < 3; ++vi)
						for (int sd = -1; sd <= 1; sd += 2)
							for (int ci = 0; ci < 6; ++ci) {
								const float sp = si == 0 ? 50.f
									: (si == 1 ? 250.f
									: (si == 2 ? 600.f : 1200.f));
								const float hd = 1.57f
									* static_cast<float>(hi2);
								const float vz = vi == 0 ? -300.f
									: (vi == 1 ? 0.f : 300.f);
								const float ca = ci == 0 ? -1.f
									: (ci == 1 ? -0.5f
									: (ci == 2 ? -0.02f
									: (ci == 3 ? 0.3f
									: (ci == 4 ? 0.85f : 1.f))));
								PlayerState s = air_state(sp, hd,
									vz);
								const float e0 =
									ExitField::EBoundary(s, p);
								float yaw = 0.f, fm = 0.f,
									sm = 0.f;
								Air::WishInputs(hd, sd, ca, &yaw,
									&fm, &sm);
								if (n % 3 == 1) {
									fm *= 0.3f;   // analog
									sm *= 0.3f;
								}
								if (n % 5 == 2)
									fm = 450.f;   // key chord
								TickEvents ev;
								MoveTick(s, w, p, 0.f, yaw, fm, sm,
									0.f, 0, &ev);
								const float de =
									ExitField::EBoundary(s, p)
									- e0;
								if (de > wish_max)
									wish_max = de;
								if (de > cap2
									+ ExitField::kUEnergyEpsTick)
									viol++;
								n++;
							}
			snprintf(buf, sizeof(buf), "%d legal wish ticks (grid + "
				"analog + chords): max dE %.2f vs ceiling %.0f; %d "
				"violations", n, wish_max, cap2, viol);
			check("E2 wish-work ceiling", viol == 0 && wish_max > 0.f,
				buf);
		}

		// ---- E3 CLIP OPTIMISM: the collision response never adds
		// speed.
		{
			unsigned rng = 0x9e3779b9u;
			auto fr = [&]() {
				rng = rng * 1664525u + 1013904223u;
				return static_cast<float>((rng >> 8) & 0xFFFF)
					/ 65535.f * 2.f - 1.f;
			};
			int viol = 0;
			float worst = 0.f;
			for (int i = 0; i < 2000; ++i) {
				Vec3 v(fr() * 1500.f, fr() * 1500.f, fr() * 800.f);
				Vec3 n(fr(), fr(), fr());
				const float nl = Len(n);
				if (nl < 1e-3f)
					continue;
				n = Scale(n, 1.f / nl);
				Vec3 out;
				Fn::ClipVelocity(v, n, &out);
				const float d = Dot(out, out) - Dot(v, v);
				if (d > worst)
					worst = d;
				if (d > 0.01f)
					viol++;
			}
			snprintf(buf, sizeof(buf), "2000 random clips through the "
				"authoritative helper: worst |v'|^2-|v|^2 = %.4f; %d "
				"expansions", worst, viol);
			check("E3 clip optimism", viol == 0, buf);
		}

		// ---- E4 DUCK/HULL BOOKKEEPING: rides with duck transitions
		// stay inside the envelope, and the duck-origin jump is
		// visible (so the hull term is doing real work).
		float duck_jump_seen = 0.f;
		int e4_viol = 0;
		{
			float worst_margin = 1e30f;
			int ticks_checked = 0;
			for (int c = 0; c < 6; ++c) {
				const int M = 120;
				std::vector<signed char> sd(
					static_cast<size_t>(M),
					static_cast<signed char>(c & 1 ? -1 : 1));
				std::vector<float> cs(static_cast<size_t>(M),
					-0.02f);
				std::vector<unsigned char> dk(
					static_cast<size_t>(M), 0);
				const int a = 8 + 6 * c;
				const int b = a + 10 + 3 * c;
				for (int k = a; k < b && k < M; ++k)
					dk[static_cast<size_t>(k)] = 1;   // duck pulse
				std::vector<PlayerState> traj;
				Ride::Result rr = Ride::FlyRideSchedule(B, w, p, g,
					sd, cs, dk, nullptr, &traj);
				if (!rr.legal)
					continue;
				const float e0 = ExitField::EBoundary(B.ps, p);
				float eprev = e0;
				for (size_t k2 = 0; k2 < traj.size(); ++k2) {
					const float en = ExitField::EBoundary(
						traj[k2], p);
					const float ub = e0 + cap2
						* static_cast<float>(k2 + 1) + hull_slack
						+ ExitField::kUEnergyEpsTick
						* static_cast<float>(k2 + 1);
					const float mg = ub - en;
					if (mg < worst_margin)
						worst_margin = mg;
					if (mg < 0.f)
						e4_viol++;
					const float jump = en - eprev;
					if (jump > duck_jump_seen)
						duck_jump_seen = jump;
					eprev = en;
					ticks_checked++;
				}
			}
			snprintf(buf, sizeof(buf), "%d boundary states across duck "
				"pulses: %d violations, worst margin %.0f; largest "
				"single-tick +dE %.0f (the duck-origin jump - the "
				"hull term earns its place)", ticks_checked, e4_viol,
				worst_margin, duck_jump_seen);
			check("E4 duck/hull bookkeeping",
				e4_viol == 0 && duck_jump_seen > 5000.f, buf);
		}

		// ---- E5 MULTISTEP CONSTRUCTIVE FALSIFICATION: every proposal
		// family schedule, the hidden off-family rides, and random
		// legal schedules - every booked boundary state obeys the
		// envelope at its own tick count.
		float e5_worst = 1e30f;
		int e5_ticks = 0, e5_viol = 0;
		{
			auto run_check = [&](const std::vector<signed char>& sd,
				const std::vector<float>& cs,
				const std::vector<unsigned char>& dk) {
				std::vector<PlayerState> traj;
				Ride::Result rr = Ride::FlyRideSchedule(B, w, p, g,
					sd, cs, dk, nullptr, &traj);
				if (!rr.legal)
					return;
				const float e0 = ExitField::EBoundary(B.ps, p);
				for (size_t k2 = 0; k2 < traj.size(); ++k2) {
					const float en = ExitField::EBoundary(
						traj[k2], p);
					const float ub = e0 + cap2
						* static_cast<float>(k2 + 1) + hull_slack
						+ ExitField::kUEnergyEpsTick
						* static_cast<float>(k2 + 1);
					if (ub - en < e5_worst)
						e5_worst = ub - en;
					if (en > ub)
						e5_viol++;
					e5_ticks++;
				}
			};
			for (int i = 0; i < ExitField::ProposalCount(2); ++i) {
				ExitField::Proposal pr;
				ExitField::MakeProposal(i, &pr);
				run_check(pr.side, pr.cosa, pr.duck);
			}
			unsigned rng = 12345u;
			auto fr = [&]() {
				rng = rng * 1664525u + 1013904223u;
				return static_cast<float>((rng >> 8) & 0xFFFF)
					/ 65535.f;
			};
			for (int j = 0; j < 40; ++j) {
				const int M = 30 + static_cast<int>(fr() * 200.f);
				std::vector<signed char> sd(
					static_cast<size_t>(M), 0);
				std::vector<float> cs(static_cast<size_t>(M), 1.f);
				std::vector<unsigned char> dk(
					static_cast<size_t>(M), 0);
				signed char cur = fr() < 0.5f
					? static_cast<signed char>(-1)
					: static_cast<signed char>(1);
				int age = 1000;
				for (int k = 0; k < M; ++k) {
					if (fr() < 0.08f && age >= 6) {
						cur = static_cast<signed char>(-cur);
						age = 0;
					}
					sd[static_cast<size_t>(k)] = fr() < 0.15f
						? static_cast<signed char>(0) : cur;
					cs[static_cast<size_t>(k)] = -0.1f
						+ fr() * 1.05f;
					dk[static_cast<size_t>(k)] = fr() < 0.1f
						? 1 : 0;
					age++;
				}
				Ride::CanonSchedule(sd, &cs, nullptr);
				run_check(sd, cs, dk);
			}
			snprintf(buf, sizeof(buf), "%d booked boundary states "
				"(204 family + 40 random-legal schedules): %d "
				"violations, tightest margin %.0f", e5_ticks,
				e5_viol, e5_worst);
			check("E5 multistep constructive", e5_viol == 0
				&& e5_ticks > 1500, buf);
		}

		// ---- E6 WITNESSED FRONTIER CONTAINMENT.
		{
			ExitField::WitnessFrontier wf;
			wf.B = B;
			ExitField::BuildFrontier(&wf, w, p, g, 2);
			int n = 0, viol = 0;
			float worst = 1e30f;
			for (const ExitField::Partition& P : wf.parts)
				for (const Ride::ExitTransition& t : P.active) {
					const float en = ExitField::EBoundary(t.s_plus,
						p);
					const float ub = ExitField::UEnergy(B, p,
						t.dt);
					if (ub - en < worst)
						worst = ub - en;
					if (en > ub)
						viol++;
					if (en > ExitField::UEnergy(B, p, 240))
						viol++;
					n++;
				}
			snprintf(buf, sizeof(buf), "%d frontier transitions: "
				"E(S+) <= U_E(B, dt) and <= U_E(B, 240); %d "
				"violations, tightest margin %.0f", n, viol, worst);
			check("E6 frontier containment", n > 0 && viol == 0, buf);
		}

		// ---- E7 HORIZON LAW: M is domain, not refinement - the
		// envelope grows monotonically with the horizon.
		{
			bool mono = true;
			for (int M = 10; M < 240; M += 10)
				if (ExitField::UEnergy(B, p, M)
					> ExitField::UEnergy(B, p, M + 10))
					mono = false;
			snprintf(buf, sizeof(buf), "U_E(B, M) nondecreasing over "
				"M = 10..240 (domain expansion may loosen U; "
				"refinement at fixed (B, M) may only tighten)");
			check("E7 horizon law", mono, buf);
		}

		// ---- E8 DELIBERATE SEMANTIC MUTATION: the suite must turn
		// red when the bound is broken (a suite that passes while too
		// weak proves nothing - B5's history). Mutation 1: drop the
		// duck-origin hull term. Mutation 2: understate the one-tick
		// wish law by 50.
		{
			int viol_hull = 0;
			{
				const int M = 120;
				std::vector<signed char> sd(
					static_cast<size_t>(M),
					static_cast<signed char>(1));
				std::vector<float> cs(static_cast<size_t>(M),
					-0.02f);
				std::vector<unsigned char> dk(
					static_cast<size_t>(M), 0);
				for (int k = 8; k < 24; ++k)
					dk[static_cast<size_t>(k)] = 1;
				std::vector<PlayerState> traj;
				Ride::Result rr = Ride::FlyRideSchedule(B, w, p, g,
					sd, cs, dk, nullptr, &traj);
				const float e0 = ExitField::EBoundary(B.ps, p);
				for (size_t k2 = 0; rr.legal && k2 < traj.size();
					++k2) {
					const float en = ExitField::EBoundary(
						traj[k2], p);
					const float ub_bad = e0 + cap2
						* static_cast<float>(k2 + 1)
						+ ExitField::kUEnergyEpsTick
						* static_cast<float>(k2 + 1);   // NO hull
					if (en > ub_bad)
						viol_hull++;
				}
			}
			// Mutation 2 uses E2's measured maximum directly: the law
			// must be TIGHT enough that understating it is detectable.
			const bool m2_detect = wish_max > cap2 - 50.f;
			const bool ok = viol_hull > 0 && m2_detect;
			snprintf(buf, sizeof(buf), "hull term removed -> %d "
				"violations (RED as required); measured one-tick max "
				"%.1f > mutated ceiling %.0f -> wish-law mutation "
				"detectable = %d", viol_hull, wish_max, cap2 - 50.f,
				m2_detect ? 1 : 0);
			check("E8 broken-semantics detection", ok, buf);
		}

		printf("exitenv: %d passed, %d failed | %s\n", pass, fail,
			fail == 0
				? "ENVELOPE CERTIFICATION GREEN - U_E(B, M) enters "
					"the certified registry (bound id 45580001, "
					"domain D_static-surf)"
				: "ENVELOPE CERTIFICATION RED");
		fflush(stdout);
		return fail == 0 ? 0 : 2;
	}

	// ================= efrefine: THE OPERATOR-LEVEL ANYTIME GATE
	// (advisor ruling 2026-08-19d). The anytime contract lives ABOVE
	// the local optimizer: a whole fixed local solve is one atomic
	// refinement action, and what must be monotone is the SANDWICH
	// L <= H* <= U, not the internal evaluation order of two
	// differently-configured solves. This gate proves exactly that,
	// and proves that search failure never becomes physical
	// impossibility.
	int CmdEfRefine(const std::string& map_path, const ReplayOpts& o) {
		World w;
		std::string err;
		w.true_interval_corner = o.corner_true;
		if (!w.Load(map_path, o.hulls, &err, o.edge_bevels)) {
			printf("LOAD FAILED (bsp): %s\n", err.c_str());
			return 1;
		}
		Route::Graph g;
		if (!Route::Build(w, &g, 2000.f, &err)) {
			printf("efrefine: %s\n", err.c_str());
			return 1;
		}
		const MoveParams& p = o.params;
		int pass = 0, fail = 0;
		char buf[256];
		auto check = [&](const char* name, bool ok, const char* detail) {
			printf("efrefine: %-34s %s | %s\n", name,
				ok ? "PASS" : "FAIL", detail);
			if (ok) pass++; else fail++;
		};
		int fi = -1;
		for (size_t i = 0; i < g.faces.size(); ++i)
			if (sqrtf(g.faces[i].n.X * g.faces[i].n.X
				+ g.faces[i].n.Y * g.faces[i].n.Y) > 1e-4f) {
				fi = static_cast<int>(i);
				break;
			}
		if (fi < 0) {
			printf("efrefine: no surfable face - cannot run\n");
			return 1;
		}
		const Route::Face& fc = g.faces[static_cast<size_t>(fi)];
		const Vec3 q = fc.centroid;
		const int Hn = 44;
		const float s0 = 700.f, vz0 = -100.f;
		const float T = static_cast<float>(Hn) * p.dt;
		const float z0 = q.Z - vz0 * T + 0.5f * p.gravity * T * T;
		const float paz = atan2f(fc.n.Y, fc.n.X);
		const float dist = 0.8f * Envelope::DMax(s0, Hn, p);
		PlayerState st;
		st.pos = Vec3(q.X + cosf(paz + 0.45f) * dist,
			q.Y + sinf(paz + 0.45f) * dist, z0);
		const float inaz = atan2f(q.Y - st.pos.Y, q.X - st.pos.X);
		st.vel = Vec3(cosf(inaz + 0.3f) * s0,
			sinf(inaz + 0.3f) * s0, vz0);
		st.ducked = false;
		st.hull_state = 0;

		int nprof = 0;
		const Entrance::RefProfile* prof = Entrance::Profiles(&nprof);
		// Run the ladder, snapshotting after every atomic action.
		std::vector<Entrance::QueryState> snap;
		Entrance::QueryState qs;
		snap.push_back(qs);
		while (Entrance::RefineStep(&qs, st, w, p, g, fi, q, 28.f, Hn,
			fc.zmin))
			snap.push_back(qs);
		const int nsteps = static_cast<int>(snap.size()) - 1;

		// ---- R1 L MONOTONE. A later, stronger profile may never
		// report a worse lower bound than an earlier one - that is the
		// whole point of merging rather than replacing.
		bool r1 = true;
		for (size_t i = 1; i < snap.size(); ++i)
			if (snap[i].has_L && snap[i - 1].has_L
				&& snap[i].L < snap[i - 1].L - 1e-3f)
				r1 = false;
			else if (snap[i - 1].has_L && !snap[i].has_L)
				r1 = false;   // a witness was FORGOTTEN
		{
			std::string s;
			for (size_t i = 1; i < snap.size(); ++i) {
				char t[48];
				snprintf(t, sizeof(t), "%s%.0fk", i > 1 ? " -> " : "",
					snap[i].has_L ? snap[i].L / 1e3f : 0.f);
				s += t;
			}
			snprintf(buf, sizeof(buf), "%d profiles | L %s", nsteps,
				s.c_str());
		}
		check("R1 L monotone non-decreasing", r1, buf);

		// ---- R2 U MONOTONE. The certified ceiling may only tighten.
		bool r2 = true;
		for (size_t i = 1; i < snap.size(); ++i)
			if (snap[i].U > snap[i - 1].U + 1e-3f)
				r2 = false;
		snprintf(buf, sizeof(buf), "U %.0fk -> %.0fk (bound id %08x), "
			"never loosened", snap.size() > 1 ? snap[1].U / 1e3f : 0.f,
			snap.back().U / 1e3f, snap.back().U_bound_id);
		check("R2 U monotone non-increasing", r2, buf);

		// ---- R3 THE SANDWICH HOLDS AND TIGHTENS. L <= U at every
		// level, and the gap never widens.
		bool r3 = true;
		float gap0 = -1.f, gapN = -1.f;
		for (size_t i = 1; i < snap.size(); ++i) {
			if (!snap[i].has_L)
				continue;
			if (snap[i].L > snap[i].U + 1.f)
				r3 = false;          // the bound would be FALSIFIED
			const float gp = snap[i].U - snap[i].L;
			if (gap0 < 0.f)
				gap0 = gp;
			gapN = gp;
		}
		if (gap0 >= 0.f && gapN > gap0 + 1.f)
			r3 = false;
		snprintf(buf, sizeof(buf), "L <= U at every level; gap %.0fk -> "
			"%.0fk", gap0 < 0.f ? 0.f : gap0 / 1e3f,
			gapN < 0.f ? 0.f : gapN / 1e3f);
		check("R3 sandwich holds and tightens", r3, buf);

		// ---- R4 WITNESSES SURVIVE AND REPLAY EXACTLY. The stored
		// witness is the operator's whole claim: replay it through the
		// exact engine and demand the recorded L back. A value that
		// cannot be handed back is not a lower bound.
		bool r4 = qs.has_L && !qs.wside.empty();
		float replay_H = 0.f, replay_err = 0.f;
		if (r4) {
			Air::Target vt;
			vt.face = fi;
			vt.dot_cap = 3000.f;
			vt.aim = q;
			vt.max_ticks = qs.horizon + 8;
			Air::Result ar = Air::FlyWishSchedule(st, w, p, vt, g,
				qs.wside, qs.wcosa, qs.horizon + 8);
			if (!ar.hit || ar.dot >= 0.f || ar.struck_brush >= 0) {
				r4 = false;
			} else {
				replay_H = Dot(ar.end_state.vel, ar.end_state.vel)
					+ 2.f * p.gravity * st.gravity_scale
					* (ar.pos.Z - fc.zmin);
				replay_err = fabsf(replay_H - qs.L);
				if (replay_err > 1.f)
					r4 = false;
			}
		}
		snprintf(buf, sizeof(buf), "final witness %d ticks replays to "
			"%.0fk vs stored %.0fk (|err| %.2f)", qs.horizon,
			replay_H / 1e3f, qs.has_L ? qs.L / 1e3f : 0.f, replay_err);
		check("R4 witness survives and replays", r4, buf);

		// ---- R5 DETERMINISM. The same action sequence from the same
		// start must reproduce the same cached state, byte for byte in
		// the parts the global solver reads.
		Entrance::QueryState qb;
		while (Entrance::RefineStep(&qb, st, w, p, g, fi, q, 28.f, Hn,
			fc.zmin)) {}
		bool r5 = qb.level == qs.level && qb.has_L == qs.has_L
			&& fabsf(qb.L - qs.L) < 1e-3f
			&& fabsf(qb.U - qs.U) < 1e-3f
			&& qb.evals == qs.evals
			&& qb.wside.size() == qs.wside.size()
			&& qb.applied.size() == qs.applied.size();
		for (size_t i = 0; r5 && i < qb.wside.size(); ++i)
			if (qb.wside[i] != qs.wside[i]
				|| fabsf(qb.wcosa[i] - qs.wcosa[i]) > 1e-6f)
				r5 = false;
		for (size_t i = 0; r5 && i < qb.applied.size(); ++i)
			if (qb.applied[i] != qs.applied[i])
				r5 = false;
		snprintf(buf, sizeof(buf), "repeat ladder: same L/U/evals/"
			"witness/profile-ids = %d (%d evals, %d profiles)",
			r5 ? 1 : 0, qb.evals, qb.level);
		check("R5 deterministic for the same sequence", r5, buf);

		// ---- R6 FAILURE IS NEVER IMPOSSIBILITY. Point the operator at
		// a target no flight can reach. Every profile must fail, and
		// the query must still come back UNRESOLVED with a real
		// certified U - never PROVED_IRRELEVANT, and never a claim that
		// the region is unreachable.
		Entrance::QueryState qu;
		const Vec3 qbad(q.X + 90000.f, q.Y + 90000.f, q.Z);
		while (Entrance::RefineStep(&qu, st, w, p, g, fi, qbad, 28.f,
			Hn, fc.zmin)) {}
		const bool unres = !qu.has_L
			&& qu.status == Entrance::QueryState::UNRESOLVED
			&& qu.U < 1e29f && qu.U_bound_id != 0
			&& qu.level == nprof;
		// And the ONLY door to PROVED_IRRELEVANT is a certified ceiling
		// beaten by an incumbent: refused below its own U, granted above.
		const bool gate_lo = !Entrance::MarkIrrelevant(&qu, qu.U - 1.f);
		const bool gate_hi = Entrance::MarkIrrelevant(&qu, qu.U + 1.f);
		const bool r6 = unres && gate_lo && gate_hi;
		snprintf(buf, sizeof(buf), "unreachable target: %d profiles all "
			"failed, status UNRESOLVED, U %.0fk still certified "
			"(%08x); MarkIrrelevant refused below U=%d granted above=%d",
			qu.level, qu.U / 1e3f, qu.U_bound_id, gate_lo ? 1 : 0,
			gate_hi ? 1 : 0);
		check("R6 failure never becomes impossibility", r6, buf);

		// ---- R7 CONTINUATION FRONTIER (ExitField stage 0). BestBoard
		// is a projection; BoardTransition witnesses are the transition
		// payload. The query must expose MULTIPLE distinct replayable
		// lanes (a lower-energy board can win the route), merged
		// monotonically: a lane once exposed is never lost and its L
		// only rises. Every lane must hand back S_B by authoritative
		// replay at exactly its recorded value.
		{
			// (a) monotone lane merge across the refinement ladder
			bool mono2 = true;
			for (size_t i2 = 2; i2 < snap.size(); ++i2)
				for (const Entrance::BoardTransition& t0
					: snap[i2 - 1].transitions) {
					bool found = false;
					for (const Entrance::BoardTransition& t1
						: snap[i2].transitions)
						if (t1.lane == t0.lane) {
							found = true;
							if (t1.L < t0.L - 1e-3f)
								mono2 = false;
						}
					if (!found)
						mono2 = false;   // a lane was LOST
				}
			// (b) diversity: several lanes, including heading intervals
			int n_lane = static_cast<int>(qs.transitions.size());
			int n_iv = 0;
			for (const Entrance::BoardTransition& t : qs.transitions)
				if ((t.lane & 0xff00u) == 0x0200u)
					n_iv++;
			// (c) every lane replays to exactly its recorded value
			int bad_replay = 0;
			float worstL = 0.f;
			for (const Entrance::BoardTransition& t : qs.transitions) {
				Air::Result ar = Entrance::ReplayBoardTransition(t, st, w,
					p, g, fi, q);
				if (!ar.hit || ar.dot >= 0.f || ar.struck_brush >= 0) {
					bad_replay++;
					continue;
				}
				const float Hr = Dot(ar.end_state.vel, ar.end_state.vel)
					+ 2.f * p.gravity * st.gravity_scale
					* (ar.pos.Z - fc.zmin);
				const float dl = fabsf(Hr - t.L);
				if (dl > worstL)
					worstL = dl;
				if (dl > 1.f)
					bad_replay++;
			}
			const bool r7 = mono2 && n_lane >= 3 && n_iv >= 2
				&& bad_replay == 0;
			snprintf(buf, sizeof(buf), "%d lanes (%d heading intervals) | "
				"monotone per lane, none lost = %d | all replay to their "
				"recorded L (worst |dL| %.2f), %d failures", n_lane, n_iv,
				mono2 ? 1 : 0, worstL, bad_replay);
			check("R7 continuation frontier", r7, buf);
		}

		// ---- The profile ladder itself, for the record.
		for (int i = 0; i < nprof; ++i)
			printf("efrefine:   profile %d %-11s budget %5d precision "
				"%.1fu m%d id %08x\n", i, prof[i].name, prof[i].budget,
				prof[i].precision, prof[i].shoot_m, prof[i].id);
		printf("efrefine: %d passed, %d failed | %s\n", pass, fail,
			fail == 0 ? "OPERATOR ANYTIME GATE GREEN"
				: "OPERATOR ANYTIME GATE RED");
		fflush(stdout);
		return fail == 0 ? 0 : 2;
	}

	int CmdAirProps(const std::string& map_path, const ReplayOpts& o) {
		World w;
		std::string err;
		w.true_interval_corner = o.corner_true;
		if (!w.Load(map_path, o.hulls, &err, o.edge_bevels)) {
			printf("LOAD FAILED (bsp): %s\n", err.c_str());
			return 1;
		}
		Route::Graph g;
		if (!Route::Build(w, &g, 2000.f, &err)) {
			printf("airprops: %s\n", err.c_str());
			return 1;
		}
		const MoveParams& p = o.params;
		int pass = 0, fail = 0;
		char buf[256];
		auto check = [&](const char* name, bool ok, const char* detail) {
			printf("airprops: %-32s %s | %s\n", name,
				ok ? "PASS" : "FAIL", detail);
			if (ok) pass++; else fail++;
		};
		int fi = -1;
		for (size_t i = 0; i < g.faces.size(); ++i) {
			const Route::Face& f2 = g.faces[i];
			if (sqrtf(f2.n.X * f2.n.X + f2.n.Y * f2.n.Y) > 1e-4f) {
				fi = static_cast<int>(i);
				break;
			}
		}
		if (fi < 0) {
			printf("airprops: no surfable face - cannot run\n");
			return 1;
		}
		const Route::Face& fc = g.faces[static_cast<size_t>(fi)];
		const Vec3 q = fc.centroid;
		const int Hn = 44;
		const float s0 = 700.f, vz0 = -100.f;
		const float T = static_cast<float>(Hn) * p.dt;
		const float z0 = q.Z - vz0 * T + 0.5f * p.gravity * T * T;
		const float paz = atan2f(fc.n.Y, fc.n.X);
		const float dist = 0.8f * Envelope::DMax(s0, Hn, p);
		PlayerState st;
		st.pos = Vec3(q.X + cosf(paz + 0.45f) * dist,
			q.Y + sinf(paz + 0.45f) * dist, z0);
		const float inaz = atan2f(q.Y - st.pos.Y, q.X - st.pos.X);
		st.vel = Vec3(cosf(inaz + 0.3f) * s0,
			sinf(inaz + 0.3f) * s0, vz0);
		st.ducked = false;
		st.hull_state = 0;

		// ---- P4 (review defect 4): the scout dedupe radius must be
		// non-decreasing in tol AND never exceed the historical 32u.
		// The original formula fixed the fine case and silently
		// tripled the coarse one.
		{
			bool mono = true, capped = true;
			float prev = -1.f;
			for (int i = 1; i <= 64; ++i) {
				const float t2 = static_cast<float>(i) * 0.5f;
				const float d = Entrance::ScoutDedupe(t2);
				if (d > 32.f + 1e-6f)
					capped = false;
				if (d + 1e-6f < prev)
					mono = false;
				prev = d;
			}
			const bool finer = Entrance::ScoutDedupe(1.f)
				< Entrance::ScoutDedupe(28.f);
			snprintf(buf, sizeof(buf), "capped@32u=%d nondecreasing=%d"
				" finer(1u)<coarse(28u)=%d", capped ? 1 : 0,
				mono ? 1 : 0, finer ? 1 : 0);
			check("P4 dedupe bounded+monotone",
				capped && mono && finer, buf);
		}
		// ---- P6 (review defect 6): the curvature to-go arc length is
		// BOUNDED everywhere, including targets directly behind, where
		// the unclamped form diverged to ~1.3e10.
		{
			bool bounded = true;
			float worst = 0.f;
			for (int i = -40; i <= 40; ++i)
				for (int j = -40; j <= 40; ++j) {
					const float lx = static_cast<float>(i) * 25.f;
					const float ly = static_cast<float>(j) * 25.f;
					const float chord = sqrtf(lx * lx + ly * ly);
					if (chord < 1e-3f)
						continue;
					const float arc = Entrance::ToGoArcLen(lx, ly);
					const float ratio = arc / chord;
					if (ratio > worst)
						worst = ratio;
					if (!(arc >= chord - 1e-2f)
						|| ratio > 1.5708f + 1e-3f)
						bounded = false;
				}
			snprintf(buf, sizeof(buf), "worst arc/chord %.4f (bound "
				"1.5708) over a full grid incl. behind", worst);
			check("P6 curvature to-go bounded", bounded, buf);
		}
		// ---- P1 (review defect 1): the boundary value tier must be
		// the FEASIBILITY PREDICATE on both channels, never a
		// threshold on their weighted sum.
		{
			PlayerState b0 = st;
			b0.pos.Z += 2500.f;
			std::vector<signed char> sd(static_cast<size_t>(Hn), 1);
			std::vector<float> cs(static_cast<size_t>(Hn),
				Strafe::ToStoredWishCos(Strafe::TrueWishCos(0.f)));
			Air::Target vt;
			vt.face = -1;
			vt.aim = b0.pos;
			vt.max_ticks = Hn;
			Air::Result ar = Air::FlyWishSchedule(b0, w, p, vt, g, sd,
				cs, Hn);
			if (ar.struck_brush >= 0 || ar.grounded) {
				check("P1 boundary feasibility predicate", false,
					"probe flight hit geometry - not exercised");
			} else {
				const Vec3 bq = ar.end_pos;
				const float bth = atan2f(ar.end_state.vel.Y,
					ar.end_state.vel.X);
				float thiv[2] = { Steer::WrapPi(bth - 0.12f),
					Steer::WrapPi(bth + 0.12f) };
				Entrance::RefTune tn;
				tn.bnd_theta = thiv;
				tn.precision = 1.f;
				int fl = 0;
				Entrance::RefResult rr = Entrance::RefSolve(b0, w, p,
					g, -1, bq, 8.f, Hn, 700, false, &fl, 0.f,
					Steer::CtlState(), nullptr, nullptr, nullptr,
					&tn);
				const bool ok = !rr.ok || (rr.bnd_rp <= 8.f
					&& rr.bnd_rth <= 1e-4f);
				snprintf(buf, sizeof(buf), "ok=%d rp %.2fu rth %.6f "
					"rad (sum-proxy would admit rth<=%.4f)",
					rr.ok ? 1 : 0, rr.bnd_rp, rr.bnd_rth,
					8.f / Entrance::kThetaW);
				check("P1 boundary feasibility predicate", ok, buf);
			}
		}
		// ---- P2 (the monotone-compute law), P3 (deterministic budget
		// ownership, review defect 3) and P5 (the winning chart is
		// recorded and used, review defect 5) ----
		{
			int f1 = 0, f2 = 0;
			Entrance::RefTune tn;
			tn.precision = 4.f;
			Entrance::RefResult r1 = Entrance::RefSolve(st, w, p, g,
				fi, q, 28.f, Hn, 300, false, &f1, fc.zmin,
				Steer::CtlState(), nullptr, nullptr, nullptr, &tn);
			Entrance::RefResult r2 = Entrance::RefSolve(st, w, p, g,
				fi, q, 28.f, Hn, 1200, false, &f2, fc.zmin,
				Steer::CtlState(), nullptr, nullptr, nullptr, &tn);
			const float L1 = r1.ok ? r1.H : -1e30f;
			const float L2 = r2.ok ? r2.H : -1e30f;
			const bool mono = L2 >= L1 - 1.f;
			snprintf(buf, sizeof(buf), "L(300)=%.0fk L(1200)=%.0fk",
				L1 > -1e29f ? L1 / 1e3f : 0.f,
				L2 > -1e29f ? L2 / 1e3f : 0.f);
			check("P2 monotonic compute", mono, buf);
			// The reserved share bounds the DISCRETIONARY remainder
			// after the budget-independent coverage prefix (bounding
			// coverage itself would break P2).
			const int disc = 1200 - r2.evals_cover;
			const int cap = r2.evals_cover
				+ (disc > 0 ? (disc * 3) / 5 : 0);
			const bool owned = r2.evals_phase1 > 0
				&& r2.evals_phase1 <= cap + 80
				&& r2.tol_final <= 16.f;
			snprintf(buf, sizeof(buf), "cover %d, phase1 %d of 1200 "
				"(cap %d + one round), tol_final %.1fu",
				r2.evals_cover, r2.evals_phase1, cap,
				r2.tol_final);
			check("P3 budget ownership bounded", owned, buf);
			const bool charted = r2.win_chart == 0
				|| r2.win_chart == 1;
			snprintf(buf, sizeof(buf), "win_chart %d (0=chord, "
				"1=curvature, -1=never set)", r2.win_chart);
			check("P5 winning chart recorded", charted, buf);
		}
		// ---- P7 (review defect 7): the residual machinery stays
		// active on hopeless queries - the dry counter must not
		// declare convergence on round 1 when nothing ever strikes.
		{
			const Vec3 far_q(q.X + 100000.f, q.Y + 100000.f, q.Z);
			int fl = 0;
			Entrance::RefResult rr = Entrance::RefSolve(st, w, p, g,
				fi, far_q, 28.f, Hn, 400, false, &fl, fc.zmin);
			const bool spent = rr.evals >= 300;
			snprintf(buf, sizeof(buf), "unreachable target: %d of 400 "
				"evals spent (round-1 dry exit spends ~40)",
				rr.evals);
			check("P7 hard-case search stays active", spent, buf);
		}
		// ---- SCHEDULER GATES (advisor 2026-08-19). These prove the
		// SEMANTICS of the anytime process, not its recovery quality.
		// S1 supersedes the temporary phase-share P3 for the scheduler
		// path: a budget FRACTION is exactly what the anytime law
		// removes, so "phase 1 owns a bounded share" cannot be the
		// correctness statement any more.
		{
			auto run_at = [&](int B, Entrance::RefResult* out) {
				Entrance::RefTune tn;
				tn.precision = 4.f;
				tn.scheduler = 1;
				int fl = 0;
				*out = Entrance::RefSolve(st, w, p, g, fi, q, 28.f,
					Hn, B, false, &fl, fc.zmin, Steer::CtlState(),
					nullptr, nullptr, nullptr, &tn);
			};
			Entrance::RefResult r1, r2, r3, r2b;
			run_at(300, &r1);
			run_at(700, &r2);
			run_at(1400, &r3);
			run_at(700, &r2b);
			// S1: TRACE PREFIX on semantic action hashes.
			auto is_prefix = [](const std::vector<unsigned long long>& a,
				const std::vector<unsigned long long>& b) {
				if (a.size() > b.size())
					return false;
				for (size_t i = 0; i < a.size(); ++i)
					if (a[i] != b[i])
						return false;
				return true;
			};
			const bool pref = is_prefix(r1.trace, r2.trace)
				&& is_prefix(r2.trace, r3.trace);
			snprintf(buf, sizeof(buf), "|T(300)|=%d |T(700)|=%d "
				"|T(1400)|=%d, each a literal prefix of the next",
				static_cast<int>(r1.trace.size()),
				static_cast<int>(r2.trace.size()),
				static_cast<int>(r3.trace.size()));
			check("S1 trace prefix containment", pref, buf);
			// S2: lower-bound monotonicity (no persistent bank is
			// involved in RefSolve at all - this is the in-query law).
			const float L1 = r1.ok ? r1.H : -1e30f;
			const float L2 = r2.ok ? r2.H : -1e30f;
			const float L3 = r3.ok ? r3.H : -1e30f;
			const bool mono = L2 >= L1 - 1.f && L3 >= L2 - 1.f;
			snprintf(buf, sizeof(buf), "L=%.0fk/%.0fk/%.0fk",
				L1 > -1e29f ? L1 / 1e3f : 0.f,
				L2 > -1e29f ? L2 / 1e3f : 0.f,
				L3 > -1e29f ? L3 / 1e3f : 0.f);
			check("S2 lower-bound monotonicity", mono, buf);
			// S3: determinism - identical query, identical stream.
			bool det = r2.trace.size() == r2b.trace.size()
				&& (r2.ok == r2b.ok)
				&& (!r2.ok || fabsf(r2.H - r2b.H) < 1e-3f);
			for (size_t i = 0; det && i < r2.trace.size(); ++i)
				if (r2.trace[i] != r2b.trace[i])
					det = false;
			snprintf(buf, sizeof(buf), "repeat run: %d actions, same "
				"hashes=%d, same H=%d",
				static_cast<int>(r2b.trace.size()), det ? 1 : 0,
				(r2.ok == r2b.ok) ? 1 : 0);
			check("S3 deterministic action stream", det, buf);
			// S4: FAIRNESS/PROGRESS (replaces the phase-share P3 on
			// this path). Coverage may not monopolise the stream: at a
			// generous budget every domain gets its cheap scout AND the
			// competitive ones escalate through m0 -> m1 -> m2, with
			// precision continuation reached.
			const bool fair = r3.sched_m_used[0] > 0
				&& r3.sched_m_used[1] > 0
				&& r3.sched_m_used[2] > 0;
			snprintf(buf, sizeof(buf), "actions %d | m0 %d m1 %d m2 %d "
				"| precision steps %d | tol_final %.1fu",
				r3.sched_actions, r3.sched_m_used[0],
				r3.sched_m_used[1], r3.sched_m_used[2],
				r3.sched_prec, r3.tol_final);
			check("S4 scheduler fairness/progress", fair, buf);
			// ---- S10 COVERAGE CONSERVATISM AND PROGRESSION ----
			// The cheap coverage action exists to make a speculative
			// query affordable. The danger it introduces is that it
			// quietly becomes a COARSE REACHABILITY PRUNE: a scout that
			// finds nothing declaring the heading interval dead. Only a
			// certified bound may eliminate a domain, so this gate
			// pins three things at once:
			//   (a) every initially-unresolved domain is covered before
			//       any method-specific escalation runs;
			//   (b) a domain whose coverage found nothing is still
			//       UNRESOLVED - never PROVED_IRRELEVANT - and every
			//       elimination that did happen names a certified bound;
			//   (c) competitive domains remain eligible for the ordinary
			//       m0/m1/m2/GN/deepen/precision ladder afterwards.
			{
				// A tiny budget can only afford the coverage prefix; it
				// is the sharpest test of (a) and (b) because nothing
				// else has run yet.
				Entrance::RefResult rc0;
				{
					Entrance::RefTune tn;
					tn.precision = 4.f;
					tn.scheduler = 2;
					int fl = 0;
					rc0 = Entrance::RefSolve(st, w, p, g, fi, q, 28.f,
						Hn, 40, false, &fl, fc.zmin, Steer::CtlState(),
						nullptr, nullptr, nullptr, &tn);
				}
				// (a) coverage is the literal prefix, one per domain.
				const int ndom = 6;
				bool prefix_ok = rc0.sched_cover > 0
					&& rc0.sched_cover == rc0.sched_actions
					&& rc0.sched_m_used[1] == 0
					&& rc0.sched_m_used[2] == 0
					&& rc0.sched_deep == 0 && rc0.sched_gn == 0
					&& rc0.sched_prec == 0;
				for (int i = 0; i < rc0.sched_cover && prefix_ok; ++i)
					if (rc0.dom_cover[i] != 1)
						prefix_ok = false;
				// A full-coverage run must reach every domain exactly
				// once and never twice.
				bool once_each = r3.sched_cover == ndom;
				for (int i = 0; i < ndom && once_each; ++i)
					if (r3.dom_cover[i] != 1)
						once_each = false;
				// (b) conservatism: nothing eliminated without a
				// certified bound, at either budget.
				bool conservative = true;
				for (int i = 0; i < ndom; ++i)
					if (rc0.dom_state[i] == 1 || r3.dom_state[i] == 1) {
						// an eliminated domain must have a prune record
						bool named = false;
						for (size_t k = 0; k < r3.prunes.size(); ++k)
							if (r3.prunes[k].domain == i
								&& r3.prunes[k].bound_id != 0
								&& r3.prunes[k].U
									<= r3.prunes[k].L_star
										+ r3.prunes[k].eps)
								named = true;
						if (!named)
							conservative = false;
					}
				if (!rc0.prunes.empty())
					conservative = false;   // nothing certified can fire
				                            // before any witness exists
				// (c) escalation still reachable after coverage.
				const bool escalates = r3.sched_m_used[0] > 0
					&& r3.sched_m_used[1] > 0 && r3.sched_m_used[2] > 0;
				const bool s10 = prefix_ok && once_each && conservative
					&& escalates;
				snprintf(buf, sizeof(buf), "@40: %d actions all coverage "
					"| @1400: %d coverage (1/domain=%d) %d ev = %d%% of "
					"%d | 0 uncertified eliminations | escalates=%d",
					rc0.sched_actions, r3.sched_cover, once_each ? 1 : 0,
					r3.sched_cover_ev,
					r3.evals > 0 ? 100 * r3.sched_cover_ev / r3.evals : 0,
					r3.evals, escalates ? 1 : 0);
				check("S10 coverage conservatism", s10, buf);
			}
			// S5: proof-carrying prunes. Every certified elimination
			// carries the bound identity AND the incumbent witness.
			bool proofs = true;
			for (size_t i = 0; i < r3.prunes.size(); ++i) {
				const Entrance::RefResult::PruneRec& pr = r3.prunes[i];
				if (pr.bound_id == 0 || !(pr.U <= pr.L_star + pr.eps))
					proofs = false;
			}
			snprintf(buf, sizeof(buf), "%d certified eliminations, all "
				"carrying bound id + incumbent witness + the numeric "
				"inequality", static_cast<int>(r3.prunes.size()));
			check("S5 proof-carrying prunes", proofs, buf);
			// S6 ACTION COMPLETENESS (advisor 2026-08-19): given a
			// long unrestricted run and a query whose state demands
			// them, the stream must actually reach every refinement
			// mechanism - m0, m1, m2, DEEPEN, GN and precision. This
			// is the remaining half of "the scheduler can expose the
			// capability the methods already have".
			{
				Entrance::RefResult rc;
				run_at(6000, &rc);
				const bool complete = rc.sched_m_used[0] > 0
					&& rc.sched_m_used[1] > 0
					&& rc.sched_m_used[2] > 0
					&& rc.sched_deep > 0 && rc.sched_gn > 0
					&& rc.sched_prec > 0;
				snprintf(buf, sizeof(buf), "at 6000: m0 %d m1 %d m2 "
					"%d deepen %d gn %d precision %d | %d actions",
					rc.sched_m_used[0], rc.sched_m_used[1],
					rc.sched_m_used[2], rc.sched_deep, rc.sched_gn,
					rc.sched_prec, rc.sched_actions);
				check("S6 action completeness", complete, buf);
				// CAPABILITY PROBE (not a gate yet): can the scheduler
				// expose what the legacy path reaches on the same
				// query at the same compute? Different action order is
				// expected and fine.
				Entrance::RefTune lg;
				lg.scheduler = 0;   // pinned: the legacy baseline
				lg.precision = 4.f;
				int flg = 0;
				Entrance::RefResult rl = Entrance::RefSolve(st, w, p,
					g, fi, q, 28.f, Hn, 6000, false, &flg, fc.zmin,
					Steer::CtlState(), nullptr, nullptr, nullptr,
					&lg);
				printf("airprops: capability probe @6000 | legacy H "
					"%.0fk rmin %.1fu | scheduled H %.0fk rmin "
					"%.1fu\n",
					rl.ok ? rl.H / 1e3f : 0.f,
					rl.strike_rmin < 1e29f ? rl.strike_rmin : -1.f,
					rc.ok ? rc.H / 1e3f : 0.f,
					rc.strike_rmin < 1e29f ? rc.strike_rmin : -1.f);
				// ---- THE ALLOCATION DIFFERENTIAL: identical query,
				// bounds and methods; ONLY the service discipline
				// changes between v0 and v1.
				Entrance::RefTune t1;
				t1.precision = 4.f;
				t1.scheduler = 2;         // v1: weighted-fair service
				int f1b = 0;
				Entrance::RefResult rv1 = Entrance::RefSolve(st, w, p,
					g, fi, q, 28.f, Hn, 6000, false, &f1b, fc.zmin,
					Steer::CtlState(), nullptr, nullptr, nullptr, &t1);
				printf("airprops: allocation differential @6000 | v0 H "
					"%.0fk rmin %.1fu acts %d (m2 %d prec %d gn %d) | "
					"v1 H %.0fk rmin %.1fu acts %d (m2 %d prec %d gn "
					"%d)\n",
					rc.ok ? rc.H / 1e3f : 0.f,
					rc.strike_rmin < 1e29f ? rc.strike_rmin : -1.f,
					rc.sched_actions, rc.sched_m_used[2],
					rc.sched_prec, rc.sched_gn,
					rv1.ok ? rv1.H / 1e3f : 0.f,
					rv1.strike_rmin < 1e29f ? rv1.strike_rmin : -1.f,
					rv1.sched_actions, rv1.sched_m_used[2],
					rv1.sched_prec, rv1.sched_gn);
				// PER-DOMAIN SPREAD: largest tightening versus the old
				// global ceiling is NOT the same as spread BETWEEN
				// domains, so measure the spread directly.
				float umin = 1e30f, umax = -1e30f;
				int served = 0, maxshare = 0, tot_acts = 0;
				for (int i = 0; i < 6; ++i) {
					if (rv1.dom_U[i] > 0.f) {
						if (rv1.dom_U[i] < umin) umin = rv1.dom_U[i];
						if (rv1.dom_U[i] > umax) umax = rv1.dom_U[i];
					}
					tot_acts += rv1.dom_acts[i];
					if (rv1.dom_acts[i] > 0) served++;
					if (rv1.dom_acts[i] > maxshare)
						maxshare = rv1.dom_acts[i];
				}
				printf("airprops: U_D spread min %.0fk max %.0fk range "
					"%.0fk | v1 domains served %d/6, largest share "
					"%d%%\n",
					umin < 1e29f ? umin / 1e3f : 0.f,
					umax > -1e29f ? umax / 1e3f : 0.f,
					(umax > -1e29f && umin < 1e29f)
						? (umax - umin) / 1e3f : 0.f,
					served, tot_acts ? maxshare * 100 / tot_acts : 0);
				// S7 no-monopoly: importance may bias service, never
				// imply starvation.
				const bool s7 = served >= 2 && tot_acts > 0
					&& maxshare * 100 < tot_acts * 90;
				snprintf(buf, sizeof(buf), "%d of 6 domains served; "
					"largest share %d%% of %d actions", served,
					tot_acts ? maxshare * 100 / tot_acts : 0,
					tot_acts);
				check("S7 cross-domain no-monopoly", s7, buf);
				// S8 within-domain method fairness: precision and GN
				// must not be starved by endless shooting.
				// S9 PRECISION PROGRESSION: rungs advance in order and
				// never skip, and a step cannot fire while the active
				// rung is infeasible. The number of PrecisionSteps
				// must equal exactly the number of kLadder rungs
				// strictly between the acceptance radius and the final
				// active tolerance.
				{
					int expect = 0;
					for (int r2 = 0; r2 < 6; ++r2)
						if (Entrance::kLadder[r2] < 28.f
							&& Entrance::kLadder[r2] >= rv1.tol_final
								- 1e-3f)
							expect++;
					const bool ordered = rv1.sched_prec == expect;
					// ...and a query that never becomes feasible must
					// take no precision steps at all (no runaway).
					Entrance::RefTune tf;
					tf.precision = 1.f;
					tf.scheduler = 2;
					int ff = 0;
					const Vec3 far_q2(q.X + 100000.f, q.Y + 100000.f,
						q.Z);
					Entrance::RefResult rf = Entrance::RefSolve(st, w,
						p, g, fi, far_q2, 28.f, Hn, 800, false, &ff,
						fc.zmin, Steer::CtlState(), nullptr, nullptr,
						nullptr, &tf);
					const bool no_runaway = rf.sched_prec == 0;
					snprintf(buf, sizeof(buf), "steps %d = rungs "
						"28u->%.0fu (%d), no-runaway on an "
						"unreachable target: %d steps",
						rv1.sched_prec, rv1.tol_final, expect,
						rf.sched_prec);
					check("S9 precision progression", ordered
						&& no_runaway, buf);
				}
				const bool s8 = rv1.sched_prec > 0 && rv1.sched_gn > 0
					&& rv1.sched_deep > 0;
				snprintf(buf, sizeof(buf), "v1 precision %d gn %d "
					"deepen %d (v0 had %d/%d/%d)", rv1.sched_prec,
					rv1.sched_gn, rv1.sched_deep, rc.sched_prec,
					rc.sched_gn, rc.sched_deep);
				check("S8 within-domain method fairness", s8, buf);
			}
		}
		// ---- BOUND GATES for the interval-specific board ceiling.
		// An unsafe upper bound is far worse than a loose one, so these
		// run before the bound is allowed to prune anything. Every gate
		// below measures the ceiling against the quantity production
		// actually credits: H = |end_state.vel|^2 + 2 g gs (z - zmin),
		// taken AFTER FinishGravity.
		{
			const float gimp = 0.5f * p.gravity * p.dt;
			const float vzT = Envelope::VzAfter(st.vel.Z, Hn, p);
			const float sT = Envelope::SMax(700.f, Hn, p);
			const float pot = 2.f * p.gravity * (q.Z - fc.zmin);
			const float U_speed = sT * sT + (vzT - gimp) * (vzT - gimp)
				+ pot;
			// THE AUTHORITATIVE BOARD RESPONSE. Board::
			// PredictClipTickVel is the production helper and owns the
			// whole tick: StartGravity -> ClipVelocity -> FinishGravity.
			// The relaxed set is written in PRE-CLIP (post StartGravity)
			// velocities, so the pre-StartGravity state that realises a
			// given v- is v- + (0,0,G). Going through the helper - not a
			// re-derivation - is the entire point of B7.
			auto board_E = [&](const Vec3& v1) {
				const Vec3 vpre(v1.X, v1.Y, v1.Z + gimp);
				const Vec3 ve = Board::PredictClipTickVel(vpre, fc.n, p,
					1.f);
				return Dot(ve, ve) + pot;
			};
			// Recover a heading INSIDE the interval that realises a
			// given a = h cos(theta - phi). If neither branch lands in
			// the interval the maximiser is not a member of the domain
			// and the bound is meaningless, so this is a hard failure.
			auto theta_in = [&](float a, float lo, float hi,
				float* th_out) {
				const float hh = sqrtf(fc.n.X * fc.n.X
					+ fc.n.Y * fc.n.Y);
				if (hh < 1e-5f)
					return false;
				float c = a / hh;
				if (c > 1.f) c = 1.f;
				if (c < -1.f) c = -1.f;
				const float phi = atan2f(fc.n.Y, fc.n.X);
				const float da = acosf(c);
				const float cd[2] = { phi + da, phi - da };
				float wdt = hi - lo;
				while (wdt < 0.f) wdt += 6.28318531f;
				// The maximiser sits ON an interval endpoint whenever the
				// extremum of cos over the arc is an endpoint - the common
				// case - so containment must be tested SYMMETRICALLY about
				// the arc midpoint. Measuring the offset from the lower end
				// instead wraps a -1e-7 rounding error round to 2pi and
				// rejects a legal heading.
				const float mid = lo + 0.5f * wdt;
				for (int i = 0; i < 2; ++i) {
					float t = cd[i] - mid;
					while (t > 3.14159265f) t -= 6.28318531f;
					while (t < -3.14159265f) t += 6.28318531f;
					if (fabsf(t) <= 0.5f * wdt + 1e-3f) {
						*th_out = cd[i];
						return true;
					}
				}
				return false;
			};
			// B1 nested monotonicity: the interval bound may never
			// exceed the broad speed ceiling it refines.
			bool b1 = true;
			float worst_gap = 0.f;
			for (int i = 0; i < 12; ++i) {
				const float lo = -3.14159265f
					+ 6.2831853f * static_cast<float>(i) / 12.f;
				const float hi = lo + 0.6f;
				const float ub = Entrance::UBoardHeading(fc.n, vzT,
					gimp, sT, lo, hi, pot);
				if (ub > -1e29f && ub > U_speed + 1.f)
					b1 = false;
				if (ub > -1e29f && U_speed - ub > worst_gap)
					worst_gap = U_speed - ub;
			}
			snprintf(buf, sizeof(buf), "U_board <= U_speed on 12 "
				"intervals; largest tightening %.0fk", worst_gap
				/ 1e3f);
			check("B1 nested bound monotonicity", b1, buf);
			// B2 dense falsification: sample the RELAXED set directly,
			// push every member through the AUTHORITATIVE board helper,
			// and check none exceeds the claimed maximum.
			bool b2 = true;
			float worst_over = 0.f;
			for (int i = 0; i < 8; ++i) {
				const float lo = -3.14159265f
					+ 6.2831853f * static_cast<float>(i) / 8.f;
				const float hi = lo + 0.9f;
				const float ub = Entrance::UBoardHeading(fc.n, vzT,
					gimp, sT, lo, hi, pot);
				for (int ti = 0; ti <= 40; ++ti)
					for (int si = 0; si <= 40; ++si) {
						const float th = lo + (hi - lo)
							* static_cast<float>(ti) / 40.f;
						const float sp = sT
							* static_cast<float>(si) / 40.f;
						const Vec3 v(sp * cosf(th), sp * sinf(th),
							vzT);
						if (Dot(v, fc.n) > 0.f)
							continue;   // not approaching
						const float E = board_E(v);
						if (E > ub + 1.f) {
							b2 = false;
							if (E - ub > worst_over)
								worst_over = E - ub;
						}
					}
			}
			snprintf(buf, sizeof(buf), "13448 relaxed-set members through "
				"the board helper over 8 intervals; worst excess %.1f",
				worst_over);
			check("B2 dense relaxed-set falsification", b2, buf);
			// B3 partition exactness: an exhaustive split of the same
			// relaxation must give exactly the parent's maximum.
			bool b3 = true;
			float worst_split = 0.f;
			for (int i = 0; i < 10; ++i) {
				const float lo = -3.f + 0.6f * static_cast<float>(i);
				const float hi = lo + 1.1f;
				const float mid = 0.5f * (lo + hi);
				const float up = Entrance::UBoardHeading(fc.n, vzT,
					gimp, sT, lo, hi, pot);
				const float u1 = Entrance::UBoardHeading(fc.n, vzT,
					gimp, sT, lo, mid, pot);
				const float u2 = Entrance::UBoardHeading(fc.n, vzT,
					gimp, sT, mid, hi, pot);
				const float cmax = u1 > u2 ? u1 : u2;
				if (up < -1e29f && cmax < -1e29f)
					continue;
				const float diff = fabsf(up - cmax);
				if (diff > worst_split)
					worst_split = diff;
				if (up + 1.f < cmax || diff > 2000.f)
					b3 = false;
			}
			snprintf(buf, sizeof(buf), "parent vs max(children) over "
				"10 splits; worst |difference| %.1f", worst_split);
			check("B3 heading partition exactness", b3, buf);
			// B4 full-circle consistency.
			const float full = Entrance::UBoardHeading(fc.n, vzT, gimp,
				sT, -3.14159265f, 3.14159265f, pot);
			float quart = -1e30f;
			for (int i = 0; i < 4; ++i) {
				const float lo = -3.14159265f
					+ 1.57079633f * static_cast<float>(i);
				const float u = Entrance::UBoardHeading(fc.n, vzT, gimp,
					sT, lo, lo + 1.57079633f, pot);
				if (u > quart)
					quart = u;
			}
			const bool b4 = fabsf(full - quart) < 2000.f;
			snprintf(buf, sizeof(buf), "full circle %.0fk vs max of 4 "
				"quadrants %.0fk", full / 1e3f, quart / 1e3f);
			check("B4 full-circle consistency", b4, buf);
			// B6 tangent sanity: an interval that admits n.v = 0 at
			// s = smax must return exactly smax^2 + (vz - G)^2 + pot.
			const float h2 = sqrtf(fc.n.X * fc.n.X + fc.n.Y * fc.n.Y);
			bool b6 = true;
			if (h2 > 1e-4f && sT > 1e-3f) {
				const float a_star = -(fc.n.Z * vzT) / sT;
				if (fabsf(a_star) <= h2) {
					const float dth = acosf(a_star / h2);
					const float phi = atan2f(fc.n.Y, fc.n.X);
					const float th_t = phi + dth;
					const float ub = Entrance::UBoardHeading(fc.n,
						vzT, gimp, sT, th_t - 0.05f, th_t + 0.05f,
						pot);
					const float want = sT * sT + (vzT - gimp)
						* (vzT - gimp) + pot;
					b6 = fabsf(ub - want) < 2000.f;
					snprintf(buf, sizeof(buf), "tangent-admitting "
						"interval: U %.0fk vs smax^2+(vz-G)^2+pot "
						"%.0fk", ub / 1e3f, want / 1e3f);
				} else {
					snprintf(buf, sizeof(buf), "no tangent arrival "
						"exists at this vz/smax (|a*| > h)");
				}
			} else {
				snprintf(buf, sizeof(buf), "degenerate normal - not "
					"exercised");
			}
			check("B6 tangent sanity", b6, buf);
			// ---- B7 AUTHORITATIVE BOARD PARITY ----
			// Agreement on the NUMBER is not enough: the analytical
			// argmax must be a LEGAL member of the domain (its heading
			// inside the interval, its dot reproduced by the real
			// normal convention) and the authoritative helper must
			// return exactly the ceiling minus the one term the
			// derivation explicitly drops,
			//     U - E_auth = 2 G n_z |d|   (>= 0, zero at tangency),
			// across ALL THREE candidate classes. A mismatch here is a
			// semantic disagreement about velocity phase / gravity
			// placement / clipping, never a tolerance to widen.
			{
				bool b7 = true;
				int seen[3] = { 0, 0, 0 };
				int n7 = 0, bad_th = 0, bad_d = 0, bad_e = 0;
				float worst7 = 0.f;
				const float vzs7[4] = { 0.f, -100.f, -300.f, -600.f };
				const float sms7[3] = { 300.f, 650.f, 1000.f };
				for (int vi = 0; vi < 4; ++vi)
					for (int si = 0; si < 3; ++si)
						for (int ii = 0; ii < 16; ++ii) {
							const float lo = -3.14159265f + 6.2831853f
								* static_cast<float>(ii) / 16.f;
							const float hi = lo + (ii & 1 ? 0.9f
								: 0.25f);
							Entrance::UBoardArg ag;
							const float ub = Entrance::UBoardHeading(
								fc.n, vzs7[vi], gimp, sms7[si], lo, hi,
								pot, &ag);
							if (ub < -1e29f || ag.cand < 0)
								continue;
							n7++;
							seen[ag.cand]++;
							float th7 = 0.f;
							if (!theta_in(ag.a, lo, hi, &th7)) {
								b7 = false;
								bad_th++;
								continue;
							}
							const Vec3 v1(ag.s * cosf(th7),
								ag.s * sinf(th7), vzs7[vi]);
							const float dd = Dot(v1, fc.n);
							if (fabsf(dd - ag.d) > 1e-2f
								* (1.f + fabsf(ag.d))) {
								b7 = false;
								bad_d++;
							}
							const float Ea = board_E(v1);
							const float drop = 2.f * gimp * fc.n.Z
								* (-ag.d);
							const float resid = fabsf(ub - Ea - drop);
							if (resid > worst7)
								worst7 = resid;
							if (resid > 0.5f + 1e-5f * fabsf(ub)) {
								b7 = false;
								bad_e++;
							}
						}
				// Candidate A (s = 0) only wins where the interval
				// admits no approaching arrival at any positive speed,
				// which needs b ~ 0 - the vz = 0 row exists to reach
				// it. Coverage is reported either way.
				const bool cover = seen[0] > 0 && seen[1] > 0
					&& seen[2] > 0;
				b7 = b7 && cover;
				snprintf(buf, sizeof(buf), "%d maximisers | candidates "
					"s0/smax/bnd %d/%d/%d | bad theta %d dot %d energy "
					"%d | worst |U-Eauth-2Gnzd| %.2f", n7, seen[0],
					seen[1], seen[2], bad_th, bad_d, bad_e, worst7);
				check("B7 authoritative board parity", b7, buf);
			}
			// ---- B5 CONSTRUCTIVE ORACLE FALSIFICATION ----
			// AirRec builds LEGAL flights whose realized post-board
			// energy is known exactly. Any certified stage that sits
			// below one of them is FALSIFIED. Passing is empirical
			// validation of the implementation against constructive
			// counterexamples - it is NOT the proof, which comes from
			// the derivation. Run against EVERY stage, not just the
			// final bound, so an unsafe rung cannot hide behind a safe
			// one above it.
			{
				const int min_gap = static_cast<int>(
					ceilf((1.f / p.dt) / p.strafe_rate_max));
				float zhigh = -1e30f;
				for (const Route::Face& f3 : g.faces)
					if (f3.centroid.Z > zhigh)
						zhigh = f3.centroid.Z;
				const int hzs5[2] = { 34, 52 };
				const float sps5[2] = { 500.f, 800.f };
				const float vzs5[2] = { -60.f, -240.f };
				std::vector<RecOracle> ocs;
				int gen = 0;
				for (size_t f4 = 0; f4 < g.faces.size()
					&& static_cast<int>(ocs.size()) < 12; ++f4) {
					const Route::Face& ff = g.faces[f4];
					if (sqrtf(ff.n.X * ff.n.X + ff.n.Y * ff.n.Y)
						< 1e-4f)
						continue;
					for (int cc = 0; cc < 2; ++cc, ++gen) {
						RecOracle oc;
						oc.rev = cc;
						oc.prof = cc == 0 ? 0 : 2;
						oc.dwell = 1;
						const int T = hzs5[gen % 2];
						if (T < (oc.rev + 1) * min_gap)
							continue;
						RecSchedule(oc.rev, oc.prof, oc.dwell, T,
							min_gap, static_cast<unsigned>(
								gen * 104729 + 11), &oc.side,
							&oc.cosa, &oc.min_dwell);
						if (RecBoardOracle(w, p, g,
							static_cast<int>(f4), sps5[gen % 2],
							vzs5[(gen / 2) % 2], T, oc.side, oc.cosa,
							zhigh, &oc))
							ocs.push_back(oc);
					}
				}
				bool b5 = ocs.size() >= 4;
				int viol[3] = { 0, 0, 0 };
				float slack[3] = { 1e30f, 1e30f, 1e30f };
				for (const RecOracle& oc : ocs) {
					const Route::Face& ff = g.faces[
						static_cast<size_t>(oc.face)];
					const float gs5 = oc.S0.gravity_scale;
					const float g5 = 0.5f * p.gravity * gs5 * p.dt;
					const float s05 = Len2D(oc.S0.vel);
					const float rad5 = 28.f;
					const int kc = oc.T + 48;
					int ka = 1, kb = kc;
					if (!Envelope::ZWindow(oc.S0.pos.Z, oc.S0.vel.Z,
						oc.Q.Z - rad5, oc.Q.Z + rad5, p, kc, &ka, &kb,
						gs5)) {
						ka = 1;
						kb = kc;
					}
					const float pot5 = 2.f * p.gravity * gs5
						* (oc.Q.Z + rad5 - oc.zmin);
					// Stage 1: the broad certified energy ceiling.
					float u1 = -1e30f, u2 = -1e30f;
					for (int k = ka; k <= kb; ++k) {
						const float sk = Envelope::SMax(s05, k, p);
						const float vk = Envelope::VzAfter(
							oc.S0.vel.Z, k, p, gs5) - g5;
						const float e1 = sk * sk + vk * vk + pot5;
						if (e1 > u1)
							u1 = e1;
						// Stage 2: the heading-agnostic board ceiling.
						const float e2 = Entrance::UBoardHeading(ff.n,
							Envelope::VzAfter(oc.S0.vel.Z, k, p, gs5),
							g5, sk, -3.14159265f, 3.14159265f, pot5);
						if (e2 > u2)
							u2 = e2;
					}
					// Stage 3: the SHARP form - the oracle's own
					// arrival tick, its own contact height, a narrow
					// interval around its realized heading. No
					// acceptance slack at all; this is the rung that
					// the missing gravity phase would break.
					const float u3 = Entrance::UBoardHeading(ff.n,
						Envelope::VzAfter(oc.S0.vel.Z, oc.T, p, gs5),
						g5, Envelope::SMax(s05, oc.T, p),
						oc.th - 0.02f, oc.th + 0.02f,
						2.f * p.gravity * gs5 * (oc.Q.Z - oc.zmin));
					const float us[3] = { u1, u2, u3 };
					for (int sidx = 0; sidx < 3; ++sidx) {
						if (us[sidx] < -1e29f) {
							viol[sidx]++;
							b5 = false;
							continue;
						}
						const float m = us[sidx] - oc.E;
						if (m < slack[sidx])
							slack[sidx] = m;
						if (m < -1.f) {
							viol[sidx]++;
							b5 = false;
						}
					}
				}
				snprintf(buf, sizeof(buf), "%d legal oracles | "
					"violations speed/full/sharp %d/%d/%d | tightest "
					"margin %.0fk/%.0fk/%.0fk",
					static_cast<int>(ocs.size()), viol[0], viol[1],
					viol[2],
					slack[0] < 1e29f ? slack[0] / 1e3f : 0.f,
					slack[1] < 1e29f ? slack[1] / 1e3f : 0.f,
					slack[2] < 1e29f ? slack[2] / 1e3f : 0.f);
				check("B5 constructive oracle falsification", b5, buf);
			}
		}
		printf("airprops: %d passed, %d failed | %s\n", pass, fail,
			fail == 0 ? "PROPERTY GATE GREEN"
				: "PROPERTY GATE RED - a review defect regressed");
		fflush(stdout);
		return fail == 0 ? 0 : 2;
	}

	// ================= wishparity: THE CONVENTION PARITY GATE
	// (advisor 2026-08-19: "make it difficult for this class of bug to
	// exist again"). The stored canonical (side, cosa) basis and the
	// closed-form Strafe::TickLaw are written in DIFFERENT semantic
	// units, and one mirrored bridge silently crippled the guided
	// shooter's whole action vocabulary. This gate settles the
	// convention EMPIRICALLY - one exact engine tick against the law
	// over a grid of (speed, stored cosa, side) - and is a permanent
	// regression gate at the physics/search interface.
	//
	// It discriminates two hypotheses instead of assuming one:
	//   H1: true wish cos = +stored, realized rotation = +side
	//   H2: true wish cos = -stored, realized rotation = -side
	// Whichever reproduces the engine IS the convention. Reported, not
	// assumed - the same discipline that caught the original bug.
	int CmdWishParity(const std::string& map_path, const ReplayOpts& o) {
		World w;
		std::string err;
		w.true_interval_corner = o.corner_true;
		if (!w.Load(map_path, o.hulls, &err, o.edge_bevels)) {
			printf("LOAD FAILED (bsp): %s\n", err.c_str());
			return 1;
		}
		Route::Graph g;
		if (!Route::Build(w, &g, 2000.f, &err)) {
			printf("wishparity: %s\n", err.c_str());
			return 1;
		}
		const MoveParams& p = o.params;
		float zhigh = -1e30f;
		for (const Route::Face& fc : g.faces)
			if (fc.centroid.Z > zhigh)
				zhigh = fc.centroid.Z;
		zhigh += 3000.f;
		const float cap = p.air_speed_cap;
		const float sps[6] = { 50.f, 150.f, 400.f, 900.f, 1600.f,
			2400.f };
		// Absolute cos samples PLUS band-relative ones: the active
		// free-turn band is only cap/v wide in cos units (0.033 at
		// v=900), so a fixed grid cannot resolve it - the parity gate
		// must probe inside it or it proves nothing about gain.
		struct Row {
			float s0, c;
			int side;
			float ds2_meas, dh_meas;
			float e1s, e1h, e2s, e2h;
		};
		std::vector<Row> rows;
		double m1s = 0.0, m1h = 0.0, m2s = 0.0, m2h = 0.0;
		int n1s = 0, n2s = 0, sign1 = 0, sign2 = 0, nsign = 0;
		for (int si = 0; si < 6; ++si) {
			const float s0 = sps[si];
			const float bw = s0 > cap ? cap / s0 : 1.f;
			float cs[14] = { 1.f, 0.9f, 0.6f, 0.3f, 0.05f, 0.f,
				-0.05f, -0.3f, -0.6f, -0.9f, -1.f,
				-bw, -0.5f * bw, 0.5f * bw };
			for (int ci = 0; ci < 14; ++ci)
				for (int sd = -1; sd <= 1; sd += 2) {
					PlayerState st;
					st.pos = Vec3(0.f, 0.f, zhigh);
					st.vel = Vec3(s0, 0.f, 0.f);
					st.ducked = false;
					st.hull_state = 0;
					st.on_ground = false;
					float yaw = 0.f, fm = 0.f, sm = 0.f;
					Air::WishInputs(0.f, sd, cs[ci], &yaw, &fm,
						&sm);
					TickEvents ev;
					MoveTick(st, w, p, 0.f, yaw, fm, sm, 0.f, 0,
						&ev);
					if (ev.ncontacts > 0 || st.on_ground) {
						printf("wishparity: CONTACT in open air "
							"at s%.0f c%.2f side%+d - probe "
							"altitude invalid\n", s0, cs[ci],
							sd);
						return 1;
					}
					const float s1 = Len2D(st.vel);
					Row r;
					r.s0 = s0;
					r.c = cs[ci];
					r.side = sd;
					r.ds2_meas = s1 * s1 - s0 * s0;
					r.dh_meas = Steer::WrapPi(atan2f(st.vel.Y,
						st.vel.X));
					Strafe::TickLaw law = Strafe::Law(p, s0, 1.f,
						false);
					// H1: stored IS true, rotation follows +side.
					{
						const Strafe::TrueWishCos c(cs[ci]);
						const float s2 = 1.f - c.v * c.v;
						const float tr = law.TurnRad(c,
							s2 > 0.f ? sqrtf(s2) : 0.f);
						const float nv2 = law.NewSpeed2(c);
						r.e1s = (nv2 - s0 * s0) - r.ds2_meas;
						r.e1h = (sd > 0 ? tr : -tr) - r.dh_meas;
					}
					// H2 (the DOCUMENTED convention): true = -stored,
					// rotation follows -side. Routed through the same
					// bridge functions production uses, so this row is
					// a genuine test of them, not a restatement.
					{
						const Strafe::TrueWishCos c =
							Strafe::ToTrueWishCos(cs[ci]);
						const int rot = Strafe::ToTrueRotSide(sd);
						const float s2 = 1.f - c.v * c.v;
						const float tr = law.TurnRad(c,
							s2 > 0.f ? sqrtf(s2) : 0.f);
						const float nv2 = law.NewSpeed2(c);
						r.e2s = (nv2 - s0 * s0) - r.ds2_meas;
						r.e2h = (rot > 0 ? tr : -tr) - r.dh_meas;
					}
					if (fabsf(r.e1s) > m1s) { m1s = fabsf(r.e1s); n1s = static_cast<int>(rows.size()); }
					if (fabsf(r.e2s) > m2s) { m2s = fabsf(r.e2s); n2s = static_cast<int>(rows.size()); }
					if (fabsf(r.e1h) > m1h) m1h = fabsf(r.e1h);
					if (fabsf(r.e2h) > m2h) m2h = fabsf(r.e2h);
					if (fabsf(r.dh_meas) > 1e-6f) {
						nsign++;
						const float p1 = (sd > 0 ? 1.f : -1.f);
						if (p1 * r.dh_meas > 0.f) sign1++;
						else sign2++;
					}
					rows.push_back(r);
				}
		}
		printf("=== WISHPARITY (one exact engine tick vs the closed-"
			"form law; %d rows) ===\n", static_cast<int>(rows.size()));
		printf("wishparity: H1 (true=+stored, rot=+side): max |dspeed2 "
			"err| %.3f, max |dheading err| %.3e rad\n", m1s, m1h);
		printf("wishparity: H2 (true=-stored, rot=-side): max |dspeed2 "
			"err| %.3f, max |dheading err| %.3e rad\n", m2s, m2h);
		printf("wishparity: measured rotation sign follows +side in "
			"%d/%d turning rows, -side in %d/%d\n", sign1, nsign,
			sign2, nsign);
		// The engine decides. Tolerances are the strafelaw prover's own
		// certified scale (Docs/EngineParityReference.md): float-ULP in
		// speed^2 of ~1e7, ~1e-8 rad.
		const bool h1 = m1s < 1.0 && m1h < 1e-5;
		const bool h2 = m2s < 1.0 && m2h < 1e-5;
		printf("wishparity: VERDICT %s\n", h1 && !h2
			? "H1 - the stored basis IS the law's true-wish basis"
			: (h2 && !h1
				? "H2 - stored cos and stored side are BOTH mirrored "
				  "relative to the law (full pi rotation)"
				: (h1 && h2 ? "AMBIGUOUS (grid too weak)"
					: "NEITHER - the law does not model the engine "
					  "under either convention")));
		if (!h1 && !h2) {
			printf("wishparity:   worst H1 row s%.0f c%.2f side%+d "
				"(meas dspeed2 %.1f dh %+.5f)\n",
				rows[static_cast<size_t>(n1s)].s0,
				rows[static_cast<size_t>(n1s)].c,
				rows[static_cast<size_t>(n1s)].side,
				rows[static_cast<size_t>(n1s)].ds2_meas,
				rows[static_cast<size_t>(n1s)].dh_meas);
			printf("wishparity:   worst H2 row s%.0f c%.2f side%+d "
				"(meas dspeed2 %.1f dh %+.5f)\n",
				rows[static_cast<size_t>(n2s)].s0,
				rows[static_cast<size_t>(n2s)].c,
				rows[static_cast<size_t>(n2s)].side,
				rows[static_cast<size_t>(n2s)].ds2_meas,
				rows[static_cast<size_t>(n2s)].dh_meas);
		}
		// Sample rows, including inside the free-turn band, so the
		// report shows what each convention CLAIMS vs what happened.
		printf("wishparity: sample rows (stored c | measured dspeed2, "
			"dheading | H1 err | H2 err)\n");
		for (size_t i = 0; i < rows.size(); i += 7) {
			const Row& r = rows[i];
			printf("wishparity:   s%-5.0f c%+.4f side%+d | %+9.1f "
				"%+.5f | %+8.1f %+.5f | %+8.1f %+.5f\n", r.s0, r.c,
				r.side, r.ds2_meas, r.dh_meas, r.e1s, r.e1h, r.e2s,
				r.e2h);
		}
		// ---- Closed-loop probe: does Steer::Controller actually
		// converge on a requested heading? (The controller carries a
		// DUPLICATE of the emitter mapping; if its units are mirrored
		// its per-tick correction pushes the wrong way.)
		printf("wishparity: --- Steer::Controller convergence probe "
			"---\n");
		const float tgts[4] = { 0.15f, 0.5f, 1.2f, -0.8f };
		int conv = 0;
		for (int ti = 0; ti < 4; ++ti) {
			PlayerState st;
			st.pos = Vec3(0.f, 0.f, zhigh);
			st.vel = Vec3(800.f, 0.f, 0.f);
			st.ducked = false;
			st.hull_state = 0;
			st.on_ground = false;
			Steer::Controller ctl;
			const float theta = tgts[ti];
			float e0 = fabsf(Steer::WrapPi(theta));
			float e5 = e0, e24 = e0;
			for (int k = 0; k < 25; ++k) {
				float yaw = 0.f, fm = 0.f, sm = 0.f;
				ctl.Tick(st, p, theta, k, &yaw, &fm, &sm);
				TickEvents ev;
				MoveTick(st, w, p, 0.f, yaw, fm, sm, 0.f, 0, &ev);
				const float e = fabsf(Steer::WrapPi(theta
					- atan2f(st.vel.Y, st.vel.X)));
				if (k == 4) e5 = e;
				if (k == 24) e24 = e;
			}
			const bool ok = e24 < e0;
			if (ok) conv++;
			printf("wishparity:   target %+.2f rad | |e| %.4f -> "
				"%.4f (t5) -> %.4f (t25) | %s\n", theta, e0, e5,
				e24, ok ? "converges" : "DIVERGES");
		}
		printf("wishparity: controller %d/4 targets converge\n", conv);
		printf("wishparity: %s\n", ((h1 != h2) && conv == 4)
			? "PASS - convention determined and the controller tracks"
			: "FAIL - see rows above");
		fflush(stdout);
		return (h1 != h2) && conv == 4 ? 0 : 2;
	}

	int CmdAirRec(const std::string& map_path, const ReplayOpts& o) {
		World w;
		std::string err;
		w.true_interval_corner = o.corner_true;
		if (!w.Load(map_path, o.hulls, &err, o.edge_bevels)) {
			printf("LOAD FAILED (bsp): %s\n", err.c_str());
			return 1;
		}
		Route::Graph g;
		if (!Route::Build(w, &g, 2000.f, &err)) {
			printf("airrec: %s\n", err.c_str());
			return 1;
		}
		const MoveParams& p = o.params;
		const int min_gap = static_cast<int>(
			ceilf((1.f / p.dt) / p.strafe_rate_max));
		const int budget = o.rec_budget > 100 ? o.rec_budget : 1000;
		float zhigh = -1e30f;
		for (const Route::Face& fc : g.faces)
			if (fc.centroid.Z > zhigh)
				zhigh = fc.centroid.Z;
		zhigh += 3000.f;
		auto legal = [&](const std::vector<signed char>& sd) {
			signed char cur = 0;
			int age = 1000;
			for (signed char v : sd) {
				if (v != 0 && cur != 0 && v != cur) {
					if (age < min_gap)
						return false;
					age = 1;
				} else {
					age++;
				}
				if (v != 0)
					cur = v;
			}
			return true;
		};
		const int hzs[3] = { 28, 48, 76 };
		const float sps[3] = { 350.f, 700.f, 1000.f };
		const float vzs[3] = { 180.f, -60.f, -380.f };
		// ---------- Layer-1 oracles (free-air boundary) ----------
		std::vector<RecOracle> L1;
		int ci = 0;
		for (int rev = 0; rev < 4; ++rev)
			for (int prof = 0; prof < 4; ++prof)
				for (int dw = 0; dw < 2; ++dw, ++ci) {
					RecOracle oc;
					oc.rev = rev;
					oc.prof = prof;
					oc.dwell = dw;
					const int T = hzs[ci % 3];
					const float s0 = sps[(ci / 3) % 3];
					const float vz0 = vzs[(ci / 9) % 3];
					if (T < (rev + 1) * min_gap) {
						oc.why = "horizon too short for strata";
						L1.push_back(oc);
						continue;
					}
					RecSchedule(rev, prof, dw, T, min_gap,
						static_cast<unsigned>(ci * 7919 + 13),
						&oc.side, &oc.cosa, &oc.min_dwell);
					if (!legal(oc.side)) {
						oc.why = "ILLEGAL SCHEDULE (generator "
							"bug)";
						L1.push_back(oc);
						continue;
					}
					PlayerState st;
					st.pos = Vec3(1200.f * cosf(
						static_cast<float>(ci) * 2.39996f),
						1200.f * sinf(static_cast<float>(ci)
							* 2.39996f), zhigh);
					const float az = static_cast<float>(ci)
						* 0.71f;
					st.vel = Vec3(cosf(az) * s0, sinf(az) * s0,
						vz0);
					st.ducked = false;
					st.hull_state = 0;
					oc.S0 = st;
					oc.T = T;
					if (!RecFreeFly(st, w, p, oc.side, oc.cosa,
						&oc, nullptr)) {
						oc.why = "contact in free layer";
						L1.push_back(oc);
						continue;
					}
					oc.ok = true;
					L1.push_back(oc);
				}
		// ---------- Layer-2 oracles (clean face strikes) ----------
		std::vector<RecOracle> L2;
		int fci = 0;
		for (size_t fi = 0; fi < g.faces.size(); ++fi) {
			const Route::Face& fc = g.faces[fi];
			const float hn = sqrtf(fc.n.X * fc.n.X
				+ fc.n.Y * fc.n.Y);
			if (hn < 1e-4f)
				continue;
			for (int cc = 0; cc < 3; ++cc, ++fci) {
				RecOracle oc;
				oc.rev = cc == 0 ? 0 : (cc == 1 ? 1 : 2);
				oc.prof = cc == 0 ? 0 : (cc == 1 ? 1 : 3);
				oc.dwell = 1;
				oc.face = static_cast<int>(fi);
				const int T = hzs[fci % 3];
				const float s0 = sps[(fci / 3) % 3];
				const float vz0 = vzs[(fci / 9) % 3];
				if (T < (oc.rev + 1) * min_gap) {
					oc.why = "horizon too short for strata";
					L2.push_back(oc);
					continue;
				}
				RecSchedule(oc.rev, oc.prof, oc.dwell, T, min_gap,
					static_cast<unsigned>(fci * 104729 + 7),
					&oc.side, &oc.cosa, &oc.min_dwell);
				if (!legal(oc.side)) {
					oc.why = "ILLEGAL SCHEDULE (generator bug)";
					L2.push_back(oc);
					continue;
				}
				RecBoardOracle(w, p, g, static_cast<int>(fi), s0,
					vz0, T, oc.side, oc.cosa, zhigh, &oc);
				L2.push_back(oc);
			}
		}
		int n1 = 0, n2 = 0;
		for (const RecOracle& oc : L1)
			if (oc.ok)
				n1++;
		for (const RecOracle& oc : L2)
			if (oc.ok)
				n2++;
		printf("airrec: oracles L1 %d/%d L2 %d/%d (budget %d/case, "
			"dwell law min_gap %d)\n", n1,
			static_cast<int>(L1.size()), n2,
			static_cast<int>(L2.size()), budget, min_gap);
		// ---- FIXTURE VERSION + MANIFEST (advisor 2026-08-19) ----
		// airrec is becoming the Stage-A yardstick, so every m-curve
		// must be compared against IDENTICAL known-reachable problems.
		// The generator is versioned; the manifest is a hash over the
		// full oracle set (strata, horizon, start state, schedule and
		// the realized boundary condition, all by BIT PATTERN - the
		// bank's decimal-text idiom is lossy and could not certify
		// identity). Changing generator semantics, strata, ranges or
		// the physics convention creates a NEW version.
		{
			const char* kFixtureVer = "airrec-fixture-v2";
			unsigned long long fh = 1469598103934665603ULL;
			auto mixf = [&](float v) {
				unsigned u;
				memcpy(&u, &v, sizeof(u));
				fh = EFFnv64(&u, sizeof(u), fh);
			};
			auto mixi = [&](int v) {
				fh = EFFnv64(&v, sizeof(v), fh);
			};
			auto mix_oracle = [&](const RecOracle& oc, int layer) {
				mixi(layer);
				mixi(oc.ok ? 1 : 0);
				mixi(oc.rev); mixi(oc.prof); mixi(oc.dwell);
				mixi(oc.T); mixi(oc.face); mixi(oc.min_dwell);
				mixf(oc.S0.pos.X); mixf(oc.S0.pos.Y);
				mixf(oc.S0.pos.Z);
				mixf(oc.S0.vel.X); mixf(oc.S0.vel.Y);
				mixf(oc.S0.vel.Z);
				mixf(oc.Q.X); mixf(oc.Q.Y); mixf(oc.Q.Z);
				mixf(oc.th); mixf(oc.sT); mixf(oc.vzT); mixf(oc.E);
				for (size_t k2 = 0; k2 < oc.side.size(); ++k2) {
					mixi(static_cast<int>(
						oc.side[k2]));
					mixf(oc.cosa[k2]);
				}
			};
			for (const RecOracle& oc : L1)
				mix_oracle(oc, 1);
			for (const RecOracle& oc : L2)
				mix_oracle(oc, 2);
			// Params and law versions belong to fixture identity: the
			// same schedule under different physics is a different
			// problem.
			mixf(p.dt); mixf(p.gravity); mixf(p.airaccelerate);
			mixf(p.air_speed_cap); mixf(p.maxspeed);
			mixf(p.strafe_rate_max);
			std::string manifest = std::string(
				"C:\\Users\\Connor\\Documents\\SourceTAS\\"
				"recordings\\") + kFixtureVer + ".manifest";
			char line[256];
			snprintf(line, sizeof(line),
				"%s|L1 %d|L2 %d|hash %016llx|em1 dwell6 clip1\n",
				kFixtureVer, static_cast<int>(L1.size()),
				static_cast<int>(L2.size()), fh);
			std::string prev;
			if (FILE* fr = fopen(manifest.c_str(), "rb")) {
				char buf[256];
				if (fgets(buf, sizeof(buf), fr))
					prev = buf;
				fclose(fr);
			}
			if (prev.empty()) {
				if (FILE* fw = fopen(manifest.c_str(), "wb")) {
					fputs(line, fw);
					fclose(fw);
				}
				printf("airrec: FIXTURE %s hash %016llx (manifest "
					"created - all later curves compare against "
					"THIS set)\n", kFixtureVer, fh);
			} else if (prev == line) {
				printf("airrec: FIXTURE %s hash %016llx VERIFIED "
					"against the manifest (curves are "
					"apples-to-apples)\n", kFixtureVer, fh);
			} else {
				printf("airrec: FIXTURE DRIFT - manifest says [%s] "
					"but this run generated [%s]. The oracle set "
					"changed: bump the fixture version; do NOT "
					"compare curves across versions.\n",
					prev.c_str(), line);
			}
		}
		for (const RecOracle& oc : L1)
			if (!oc.ok)
				printf("airrec: L1 UNCONSTRUCTIBLE r%d p%d dw%d: "
					"%s\n", oc.rev, oc.prof, oc.dwell, oc.why);
		for (const RecOracle& oc : L2)
			if (!oc.ok)
				printf("airrec: L2 f%d UNCONSTRUCTIBLE r%d p%d: "
					"%s\n", oc.face, oc.rev, oc.prof, oc.why);
		// ---------- cold solves per machinery level ----------
		// TWO CURVES, NOT ONE (advisor 2026-08-19). A single fixed-
		// budget curve cannot distinguish "m=2 barely helps" from
		// "m=2 helps a lot but cannibalises its own refinement
		// budget" - completely different architectural conclusions,
		// and budget dilution is now MEASURED (at 600-1000 evals the
		// later proposal families never execute at all).
		//   ISO - identical total compute per level: production
		//         efficiency, recovery bought per unit compute.
		//   CAP - each level funded so its own added degrees of
		//         freedom are actually exercised rather than starving
		//         the earlier lanes: does extra shooting structure
		//         genuinely enlarge the reachable basin?
		// A budget-diluted curve is NOT a Level-B verdict.
		const int m_lo = o.rec_m >= 0 ? o.rec_m : 0;
		const int m_hi = o.rec_m >= 0 ? o.rec_m : 2;
		int curve1[2][3] = { { -1, -1, -1 }, { -1, -1, -1 } };
		int curve2[2][3] = { { -1, -1, -1 }, { -1, -1, -1 } };
		for (int mode = 0; mode <= 1; ++mode)
		for (int m = m_lo; m <= m_hi; ++m) {
			// CAP multiplier m0/m1/m2 = 1x/2x/3x, matching the
			// proposal families each level adds (single-target shots;
			// + the two-chart node sweep and node-time adaptation;
			// + the second node and both Gauss-Newton passes).
			// Reported so the funding is never implicit.
			const int bud = mode == 0 ? budget : budget * (m + 1);
			const char* mtag = mode == 0 ? "iso" : "cap";
			struct MAgg {
				int n = 0, ok = 0, fals = 0;
				int rung[6] = { 0, 0, 0, 0, 0, 0 };
				std::vector<float> rp, rth;
				std::vector<float> efrac;
				long long ev = 0;
				int rev_n[4] = { 0, 0, 0, 0 };
				int rev_ok[4] = { 0, 0, 0, 0 };
				int prof_n[4] = { 0, 0, 0, 0 };
				int prof_ok[4] = { 0, 0, 0, 0 };
			};
			MAgg a1, a2;
			// dh_tot median split (feature correlation, L1).
			std::vector<float> dhs;
			for (const RecOracle& oc : L1)
				if (oc.ok)
					dhs.push_back(oc.dh_tot);
			std::sort(dhs.begin(), dhs.end());
			const float dh_med = dhs.empty() ? 0.f
				: dhs[dhs.size() / 2];
			int lo_n = 0, lo_ok = 0, hi_n = 0, hi_ok = 0;
			for (const RecOracle& oc : L1) {
				if (!oc.ok)
					continue;
				float thiv[2] = { Steer::WrapPi(oc.th - 0.12f),
					Steer::WrapPi(oc.th + 0.12f) };
				Entrance::RefTune tn;
				tn.shoot_m = m;
				tn.bnd_theta = thiv;
				tn.precision = 1.f;   // measured to 1u; drive to it
				if (o.suite_sched >= 0)
					tn.scheduler = o.suite_sched;
				if (o.suite_cover >= 0)
					tn.cover_top = o.suite_cover;
				int fl = 0;
				Entrance::RefResult rr = Entrance::RefSolve(
					oc.S0, w, p, g, -1, oc.Q, 8.f, oc.T, bud,
					false, &fl, 0.f, Steer::CtlState(), nullptr,
					nullptr, nullptr, &tn);
				const bool rec = rr.ok;
				a1.n++;
				if (rec)
					a1.ok++;
				a1.ev += rr.evals;
				a1.rev_n[oc.rev]++;
				a1.prof_n[oc.prof]++;
				if (rec) {
					a1.rev_ok[oc.rev]++;
					a1.prof_ok[oc.prof]++;
				}
				if (oc.dh_tot <= dh_med) {
					lo_n++;
					if (rec)
						lo_ok++;
				} else {
					hi_n++;
					if (rec)
						hi_ok++;
				}
				const float rp = rr.bnd_rp;
				const float rth = rr.bnd_rth;
				a1.rp.push_back(rp);
				a1.rth.push_back(rth);
				for (int r2 = 0; r2 < 6; ++r2)
					if (rp <= Entrance::kLadder[r2]
						&& rth <= 1e-4f)
						a1.rung[r2]++;
				if (oc.sT > 1.f)
					a1.efrac.push_back(rr.bnd_s / oc.sT);
				// Falsification: the kinetic ceiling must clear
				// the oracle's realized terminal energy.
				const float sa = Envelope::SMax(Len2D(oc.S0.vel),
					oc.T, p);
				const float va = Envelope::VzAfter(oc.S0.vel.Z,
					oc.T, p);
				const float Uk = sa * sa + va * va;
				const float Ek = oc.sT * oc.sT
					+ oc.vzT * oc.vzT;
				if (Uk < Ek) {
					a1.fals++;
					printf("airrec: m%d L1 BOUND FALSIFIED: U_kin "
						"%.0fk < oracle %.0fk (r%d p%d T%d)\n",
						m, Uk / 1e3f, Ek / 1e3f, oc.rev,
						oc.prof, oc.T);
				}
				printf("airrec: m%d L1 r%d p%d dw%d T%d s%.0f "
					"vz%+.0f | rp %.1fu rth %.2f | s %.0f/%.0f "
					"| %s | %d ev\n", m, oc.rev, oc.prof,
					oc.dwell, oc.T, Len2D(oc.S0.vel),
					oc.S0.vel.Z, rp, rth, rr.bnd_s, oc.sT,
					rec ? "OK" : "MISS", rr.evals);
			}
			for (const RecOracle& oc : L2) {
				if (!oc.ok)
					continue;
				Entrance::RefTune tn;
				tn.shoot_m = m;
				tn.precision = 1.f;   // measured to 1u; drive to it
				if (o.suite_sched >= 0)
					tn.scheduler = o.suite_sched;
				if (o.suite_cover >= 0)
					tn.cover_top = o.suite_cover;
				int fl = 0;
				Entrance::RefResult rr = Entrance::RefSolve(
					oc.S0, w, p, g, oc.face, oc.Q, 32.f, oc.T,
					bud, false, &fl, oc.zmin,
					Steer::CtlState(), nullptr, nullptr,
					nullptr, &tn);
				const float rmin = rr.strike_rmin;
				const bool rec = rmin <= 16.f;
				a2.n++;
				if (rec)
					a2.ok++;
				a2.ev += rr.evals;
				a2.rev_n[oc.rev]++;
				a2.prof_n[oc.prof]++;
				if (rec) {
					a2.rev_ok[oc.rev]++;
					a2.prof_ok[oc.prof]++;
				}
				a2.rp.push_back(rmin < 1e29f ? rmin : 9999.f);
				const float rth = rmin < 1e29f
					? fabsf(Steer::WrapPi(rr.strike_rmin_th
						- oc.th)) : 9.f;
				a2.rth.push_back(rth);
				for (int r2 = 0; r2 < 6; ++r2)
					if (rmin <= Entrance::kLadder[r2])
						a2.rung[r2]++;
				// Oracle-energy recovery at the 16u rung.
				const float E16 = rr.rung_E[1];
				if (oc.E > 1.f && E16 > -1e29f)
					a2.efrac.push_back(E16 / oc.E);
				const float sa = Envelope::SMax(Len2D(oc.S0.vel),
					oc.T, p);
				const float va = Envelope::VzAfter(oc.S0.vel.Z,
					oc.T, p);
				const float U = sa * sa + va * va + 2.f
					* p.gravity * (oc.Q.Z - oc.zmin);
				if (U < oc.E) {
					a2.fals++;
					printf("airrec: m%d L2 BOUND FALSIFIED: U "
						"%.0fk < oracle E %.0fk (f%d r%d p%d)\n",
						m, U / 1e3f, oc.E / 1e3f, oc.face,
						oc.rev, oc.prof);
				}
				printf("airrec: m%d L2 f%d r%d p%d T%d | rmin "
					"%.1fu rth %.2f | E16 %s/%.0fk | U %.0fk | "
					"%s | %d ev\n", m, oc.face, oc.rev, oc.prof,
					oc.T, rmin < 1e29f ? rmin : -1.f, rth,
					E16 > -1e29f ? std::to_string(
						static_cast<int>(E16 / 1e3f)).c_str()
						: "-", oc.E / 1e3f, U / 1e3f,
					rec ? "OK" : "MISS", rr.evals);
			}
			auto pct = [](std::vector<float>& v, float q) {
				if (v.empty())
					return 0.f;
				std::sort(v.begin(), v.end());
				size_t i = static_cast<size_t>(
					static_cast<float>(v.size() - 1) * q);
				return v[i];
			};
			auto agg_print = [&](const char* tag, MAgg& a) {
				if (!a.n)
					return;
				printf("airrec: [%s] m%d %s: recover %d/%d | ladder "
					"32:%d 16:%d 8:%d 4:%d 2:%d 1:%d | rp p50 "
					"%.1f p95 %.1f | rth p50 %.3f p95 %.3f | "
					"E-rec p50 %.1f%% | ev/case %lld | "
					"falsified %d\n", mtag, m, tag, a.ok, a.n,
					a.rung[0], a.rung[1], a.rung[2], a.rung[3],
					a.rung[4], a.rung[5], pct(a.rp, 0.5f),
					pct(a.rp, 0.95f), pct(a.rth, 0.5f),
					pct(a.rth, 0.95f),
					100.f * pct(a.efrac, 0.5f),
					a.n ? a.ev / a.n : 0, a.fals);
				printf("airrec: [%s] m%d %s by rev: 0:%d/%d 1:%d/%d "
					"2:%d/%d 3:%d/%d | by prof: const %d/%d "
					"smooth %d/%d aggr %d/%d brake %d/%d\n", mtag, m,
					tag, a.rev_ok[0], a.rev_n[0], a.rev_ok[1],
					a.rev_n[1], a.rev_ok[2], a.rev_n[2],
					a.rev_ok[3], a.rev_n[3], a.prof_ok[0],
					a.prof_n[0], a.prof_ok[1], a.prof_n[1],
					a.prof_ok[2], a.prof_n[2], a.prof_ok[3],
					a.prof_n[3]);
			};
			agg_print("L1", a1);
			printf("airrec: [%s] m%d L1 features: dh_tot<=%.2frad "
				"%d/%d vs > %d/%d\n", mtag, m, dh_med, lo_ok, lo_n,
				hi_ok, hi_n);
			agg_print("L2", a2);
			if (m >= 0 && m < 3) {
				curve1[mode][m] = a1.ok;
				curve2[mode][m] = a2.ok;
			}
			fflush(stdout);
		}
		if (m_lo != m_hi) {
			for (int mo = 0; mo <= 1; ++mo)
				printf("airrec: CURVE[%s] L1 m0 %d m1 %d m2 %d of "
					"%d | L2 m0 %d m1 %d m2 %d of %d%s\n",
					mo == 0 ? "iso" : "cap",
					curve1[mo][0], curve1[mo][1], curve1[mo][2],
					n1, curve2[mo][0], curve2[mo][1],
					curve2[mo][2], n2,
					mo == 0 ? "  (equal compute: efficiency)"
						: "  (funded 1x/2x/3x: capability)");
			printf("airrec: LEVEL-B READS THE SHAPE OF THE CAPABILITY "
				"CURVE. Scaling (e.g. 31%%->64%%->93%%) = sequential "
				"shooting suffices; flat with a persistent "
				"known-reachable class = build defect-based "
				"multiple shooting.\n");
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

	// Inventory the recordings library: each tape's MAP, length, and
	// start anchor. A tape is only evidence about a solve when it was
	// recorded ON that map (2026-08-16: a wrong-map tape briefly stood
	// in as the basictest human benchmark) - this is the one-click
	// provenance check.
	int CmdTapeInfo(const char* filter) {
		const std::string rec = RecordingsDir();
		const std::string pat = filter ? filter : "*.tas";
		WIN32_FIND_DATAA fd;
		HANDLE h = FindFirstFileA((rec + pat).c_str(), &fd);
		if (h == INVALID_HANDLE_VALUE) {
			printf("tapeinfo: no %s in %s\n", pat.c_str(), rec.c_str());
			return 1;
		}
		printf("tapeinfo: %-58s %-20s %2s %6s %3s  %s\n", "tape", "map",
			"v", "frames", "seg", "start anchor");
		do {
			Tape tape;
			std::string err;
			if (!LoadTas(rec + fd.cFileName, tape, &err)) {
				printf("tapeinfo: %-58s UNREADABLE (%s)\n", fd.cFileName,
					err.c_str());
				continue;
			}
			char anchor[128] = "-";
			if (tape.start.valid)
				sprintf_s(anchor, "(%.0f %.0f %.0f) yaw %.0f%s",
					tape.start.origin.X, tape.start.origin.Y,
					tape.start.origin.Z, tape.start.yaw,
					tape.start.ducked ? " ducked" : "");
			printf("tapeinfo: %-58s %-20s %2d %6d %3d  %s\n", fd.cFileName,
				tape.map.empty() ? "?" : tape.map.c_str(), tape.version,
				static_cast<int>(tape.frames.size()),
				static_cast<int>(tape.segment_starts.size()), anchor);
		} while (FindNextFileA(h, &fd));
		FindClose(h);
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
	if (cmd == "tapeinfo")
		return CmdTapeInfo(argc >= 3 ? argv[2] : nullptr);
	if (cmd == "stateprobe" && argc >= 11) {
		ReplayOpts o;
		int evals = 12000;
		int next = 11;
		if (argc >= 12 && argv[11][0] != '-') {
			evals = atoi(argv[11]);
			next = 12;
		}
		if (!ParseCommon(argc, argv, next, o))
			return 1;
		const Vec3 pp(static_cast<float>(atof(argv[3])),
			static_cast<float>(atof(argv[4])),
			static_cast<float>(atof(argv[5])));
		const Vec3 vv(static_cast<float>(atof(argv[6])),
			static_cast<float>(atof(argv[7])),
			static_cast<float>(atof(argv[8])));
		return CmdStateProbe(argv[2], o, pp, vv, atoi(argv[9]),
			atoi(argv[10]), evals);
	}
	if (cmd == "fieldgate" && argc >= 4) {
		ReplayOpts o;
		std::vector<std::string> tapes;
		int i = 3;
		for (; i < argc && argv[i][0] != '-'; ++i)
			tapes.push_back(argv[i]);
		if (!ParseCommon(argc, argv, i, o))
			return 1;
		return CmdFieldGate(argv[2], o, tapes);
	}
	if (cmd == "efield" && argc >= 4) {
		ReplayOpts o;
		if (!ParseCommon(argc, argv, 4, o))
			return 1;
		return CmdEField(argv[2], o, argv[3]);
	}
	if (cmd == "fieldexact" && argc >= 4) {
		ReplayOpts o;
		if (!ParseCommon(argc, argv, 4, o))
			return 1;
		return CmdFieldExact(argv[2], o, argv[3]);
	}
	if (cmd == "humanexact" && argc >= 4) {
		ReplayOpts o;
		if (!ParseCommon(argc, argv, 4, o))
			return 1;
		return CmdHumanExact(argv[2], o, argv[3]);
	}
	if (cmd == "repfit" && argc >= 4) {
		ReplayOpts o;
		if (!ParseCommon(argc, argv, 4, o))
			return 1;
		return CmdRepFit(argv[2], o, argv[3]);
	}
	if (cmd == "airsuite" && argc >= 3) {
		ReplayOpts o;
		if (!ParseCommon(argc, argv, 3, o))
			return 1;
		return CmdAirSuite(argv[2], o);
	}
	if (cmd == "airrec" && argc >= 3) {
		ReplayOpts o;
		if (!ParseCommon(argc, argv, 3, o))
			return 1;
		return CmdAirRec(argv[2], o);
	}
	if (cmd == "exitenv" && argc >= 3) {
		ReplayOpts o;
		if (!ParseCommon(argc, argv, 3, o))
			return 1;
		return CmdExitEnv(argv[2], o);
	}
	if (cmd == "exitfrontier" && argc >= 3) {
		ReplayOpts o;
		if (!ParseCommon(argc, argv, 3, o))
			return 1;
		return CmdExitFrontier(argv[2], o);
	}
	if (cmd == "exitfit" && argc >= 3) {
		ReplayOpts o;
		if (!ParseCommon(argc, argv, 3, o))
			return 1;
		return CmdExitFit(argv[2], o);
	}
	if (cmd == "efrefine" && argc >= 3) {
		ReplayOpts o;
		if (!ParseCommon(argc, argv, 3, o))
			return 1;
		return CmdEfRefine(argv[2], o);
	}
	if (cmd == "airprops" && argc >= 3) {
		ReplayOpts o;
		if (!ParseCommon(argc, argv, 3, o))
			return 1;
		return CmdAirProps(argv[2], o);
	}
	if (cmd == "wishparity" && argc >= 3) {
		ReplayOpts o;
		if (!ParseCommon(argc, argv, 3, o))
			return 1;
		return CmdWishParity(argv[2], o);
	}
	if (cmd == "ladder" && argc >= 4) {
		ReplayOpts o;
		if (!ParseCommon(argc, argv, 4, o))
			return 1;
		return CmdLadder(argv[2], o, argv[3]);
	}
	if (cmd == "msolve2" && argc >= 4) {
		ReplayOpts o;
		if (!ParseCommon(argc, argv, 4, o))
			return 1;
		World w;
		std::string err;
		w.true_interval_corner = o.corner_true;
		if (!w.Load(argv[2], o.hulls, &err, o.edge_bevels)) {
			printf("LOAD FAILED (bsp): %s\n", err.c_str());
			return 1;
		}
		Route::Graph g;
		if (!Route::Build(w, &g, 2000.f, &err)) {
			printf("msolve2: %s\n", err.c_str());
			return 1;
		}
		Tape at;
		if (!LoadTas(argv[3], at, &err) || !at.start.valid) {
			printf("msolve2: anchor tape: %s\n", err.c_str());
			return 1;
		}
		const int end_id = DetectEndZone(w, o.params, at, "msolve2");
		if (!Route::AnchorZones(w, &g, at.start.origin,
			at.start.ducked, end_id, 2000.f, &err)) {
			printf("msolve2: %s\n", err.c_str());
			return 1;
		}
		CreateDirectoryA("Output", nullptr);
		CreateDirectoryA("Output\\reports", nullptr);
		time_t now = time(nullptr);
		struct tm tmv;
		localtime_s(&tmv, &now);
		char st[64];
		strftime(st, sizeof(st), "%m%d-%H%M%S", &tmv);
		SearchLog::Sink sink;
		sink.gravity = o.params.gravity;
		sink.wish_rate = o.params.air_speed_cap
			* o.params.air_speed_cap;
		SearchLog::g_sink = &sink;
		Plan::Opts po;
		Assemble::RunResult rr, part;
		const bool ok = Plan::SolveMap(w, g, o.params, at.start, po,
			&rr, &err, &part);
		SearchLog::g_sink = nullptr;
		sink.Flush();
		const std::string vp = std::string(
			"Output\\reports\\msolve2_") + at.map + "_" + st
			+ ".html";
		std::string verr;
		if (SearchLog::WriteHtml(vp, w, g, sink,
			"v2 constructed planner - " + at.map, &verr))
			printf("msolve2: report -> %s\n", vp.c_str());
		if (!ok) {
			printf("msolve2: SOLVE FAILED: %s\n", err.c_str());
			if (!part.frames.empty()) {
				std::string dir = argv[3];
				const size_t ds = dir.find_last_of("\\/");
				dir = ds == std::string::npos ? std::string()
					: dir.substr(0, ds + 1);
				std::string shp;
				for (int fidx : part.shape)
					shp += (shp.empty() ? "" : "-")
						+ std::to_string(fidx);
				const std::string pp = dir + at.map
					+ "_V2PARTIAL_shape" + shp + "_legs"
					+ std::to_string(part.legs_done) + "_" + st
					+ ".tas";
				std::string perr;
				if (WriteTas(pp, at.start, at.map, part.frames,
					&perr))
					printf("msolve2: partial (%d legs) -> %s\n",
						part.legs_done, pp.c_str());
			}
			return 2;
		}
		printf("msolve2: FINISHED zone tick %d, board loss2 %.0f\n",
			rr.zone_tick, rr.board_loss2);
		std::string dir = argv[3];
		const size_t ds = dir.find_last_of("\\/");
		dir = ds == std::string::npos ? std::string()
			: dir.substr(0, ds + 1);
		const std::string sp = dir + at.map + "_V2SOLVED_" + st
			+ ".tas";
		if (WriteTas(sp, at.start, at.map, rr.frames, &err))
			printf("msolve2: wrote %s\n", sp.c_str());
		return 0;
	}
	if (cmd == "autopsy" && argc >= 4) {
		ReplayOpts o;
		std::vector<std::string> tapes;
		int i = 3;
		for (; i < argc && argv[i][0] != '-'; ++i)
			tapes.push_back(argv[i]);
		if (!ParseCommon(argc, argv, i, o))
			return 1;
		return CmdAutopsy(argv[2], o, tapes, 24000);
	}
	if (cmd == "pathgate" && argc >= 4) {
		ReplayOpts o;
		if (!ParseCommon(argc, argv, 4, o))
			return 1;
		return CmdPathGate(argv[2], o, argv[3]);
	}
	if (cmd == "boardproof" && argc >= 4) {
		ReplayOpts o;
		if (!ParseCommon(argc, argv, 4, o))
			return 1;
		return CmdBoardProof(argv[2], o, argv[3], 1200);
	}
	if (cmd == "exitbench" && argc >= 4) {
		ReplayOpts o;
		int ev = 24000;
		std::vector<char*> rest;
		for (int j = 0; j < argc; ++j) {
			if (j >= 4 && std::string(argv[j]) == "--evals"
				&& j + 1 < argc) {
				ev = atoi(argv[j + 1]);
				++j;
				continue;
			}
			rest.push_back(argv[j]);
		}
		int ri = 4;
		if (!ParseCommon(static_cast<int>(rest.size()), rest.data(),
			ri, o))
			return 1;
		return CmdExitBench(argv[2], o, argv[3], ev);
	}
	if (cmd == "exitgate" && argc >= 4) {
		ReplayOpts o;
		std::vector<std::string> tapes;
		bool bmap = false;
		int i = 3;
		for (; i < argc && argv[i][0] != '-'; ++i)
			tapes.push_back(argv[i]);
		std::vector<char*> rest;
		for (int j = 0; j < argc; ++j) {
			if (j >= i && std::string(argv[j]) == "--bmap") {
				bmap = true;
				continue;
			}
			rest.push_back(argv[j]);
		}
		int ri = i;
		if (!ParseCommon(static_cast<int>(rest.size()), rest.data(),
			ri, o))
			return 1;
		return CmdExitGate(argv[2], o, tapes, bmap);
	}
	if (cmd == "tapprobe" && argc >= 7) {
		ReplayOpts o;
		int evals = 4000;
		int next = 7;
		if (argc >= 8 && argv[7][0] != '-') {
			evals = atoi(argv[7]);
			next = 8;
		}
		if (!ParseCommon(argc, argv, next, o))
			return 1;
		return CmdTapProbe(argv[2], o, argv[3], atoi(argv[4]),
			atoi(argv[5]), atoi(argv[6]), evals);
	}
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
	if (cmd == "routesgate" && argc >= 4) {
		ReplayOpts o;
		if (!ParseCommon(argc, argv, 4, o))
			return 1;
		return CmdRoutesGate(argv[2], o, argv[3]);
	}
	if (cmd == "msolvegate" && argc >= 4) {
		ReplayOpts o;
		std::string out_path;
		int i = 4;
		if (argc >= 5 && argv[4][0] != '-') {
			out_path = argv[4];
			i = 5;
		}
		if (!ParseCommon(argc, argv, i, o))
			return 1;
		return CmdMSolveGate(argv[2], o, argv[3], out_path);
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


