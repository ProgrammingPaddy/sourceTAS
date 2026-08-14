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
#include "SolverParams.h"
#include "SolverTape.h"
#include "SolverWorld.h"

#include <windows.h>
#include <shlobj.h>
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
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
		printf("  bench   <map.bsp> [ticks]\n");
		printf("All commands auto-load Documents\\sourceTAS\\solver\\server_params.cfg\n");
		printf("(written by the Map Solve tab from LIVE server values); --params <file>\n");
		printf("overrides the path, CLI flags override individual values.\n");
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
		bool energy_frontier = false;
		bool aim = false;           // contact-anchored targeting (measured
		                            // neutral on segments; --aim to enable)
		bool zone_clock = true;     // score = ticks from startzone exit
		                            // (--clock anchor for the old absolute)
		bool edge_bevels = true;    // false = pre-fix clip set (control arm)
		bool corner_true = true;    // false = legacy epsilon-padded hit test
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
			else if (a == "--no-jump-fg") o.params.jump_finishgravity = false;
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
			else if (a == "--no-edge-bevels") o.edge_bevels = false;
			else if (a == "--legacy-corner") o.corner_true = false;
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
	                     int* ftick, int* frel) {
		frames.clear();
		*ftick = 0;
		*frel = 0;
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
			for (size_t si = 0; si < subjects.size(); ++si) {
				if (!valid[si])
					continue;
				const OptimizeResult& orr = ors[si];
				printf("  subject #%d: %d -> %d ticks (%d improvements, "
					"%lld evals, %.1fM ticks/s)\n",
					static_cast<int>(si) + 1, orr.initial_tick, orr.best_tick,
					orr.improvements, orr.evals,
					orr.ticks_simulated / (orr.seconds > 0 ? orr.seconds : 1)
						/ 1e6);
				if (orr.ok && (best_tick < 0 || orr.best_tick < best_tick)) {
					best_tick = orr.best_tick;
					best_g = gs[si];
				}
			}
			fflush(stdout);
			if (best_tick > 0) {
				Optimizer optb(w, ocfg, root, root_yaw, seed_tape);
				if (optb.BuildFrames(best_g, frames, ftick, frel)) {
					printf("solve: OPTIMIZED best %d scored ticks (%.3f s; "
						"%d abs) vs explorer best %d\n", *frel,
						*frel * cfg.params.dt, *ftick, res.finishers[0].rel);
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
		for (int i = 0; i < show; ++i)
			printf("  #%d  %d scored (%.3f s; %d abs)  spd %.1f  eloss %.0fk  "
				"finish (%.0f, %.0f, z %.2f)\n", i + 1, res.finishers[i].rel,
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

		// ---- Phase 2: optimize the best finishers (fitness = finish tick,
		// strict improvements only, every eval through the proven core).
		std::vector<TapeFrame> frames;
		int ftick = 0, frel = 0;
		if (!BuildBestFrames(w, cfg, o, ex, res, nthreads,
			have_seed ? &seed : nullptr, frames, &ftick, &frel))
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
			if (!BuildBestFrames(w, tcfg, o, ex2, r2, nthreads, &inc, f2,
				&t2, &r2rel))
				continue;
			if (r2rel < frel) {
				printf("tighten: IMPROVED %d -> %d scored ticks (%.3f s)\n",
					frel, r2rel, r2rel * cfg.params.dt);
				frames = f2;
				ftick = t2;
				frel = r2rel;
			}
			fflush(stdout);
		}
		std::string out_path = o.out_tas;
		if (out_path.empty()) {
			char documents[MAX_PATH];
			if (SUCCEEDED(SHGetFolderPathA(nullptr, CSIDL_PERSONAL, nullptr,
				SHGFP_TYPE_CURRENT, documents))) {
				// Map stem for the default name.
				std::string stem = map_path;
				const size_t sl = stem.find_last_of("\\/");
				if (sl != std::string::npos) stem = stem.substr(sl + 1);
				const size_t dot = stem.find_last_of('.');
				if (dot != std::string::npos) stem = stem.substr(0, dot);
				// Never overwrite a shipped tape (captures may reference it).
				const std::string base = std::string(documents)
					+ "\\sourceTAS\\recordings\\" + stem + "_solved";
				out_path = base + ".tas";
				for (int n = 2; GetFileAttributesA(out_path.c_str())
					!= INVALID_FILE_ATTRIBUTES; ++n)
					out_path = base + " (" + std::to_string(n) + ").tas";
			} else {
				out_path = "solved.tas";
			}
		}
		std::string stem_map = map_path;
		{
			const size_t sl = stem_map.find_last_of("\\/");
			if (sl != std::string::npos) stem_map = stem_map.substr(sl + 1);
			const size_t dot = stem_map.find_last_of('.');
			if (dot != std::string::npos) stem_map = stem_map.substr(0, dot);
		}
		if (!WriteTas(out_path, anchor, stem_map, frames, &err)) {
			printf("solve: WriteTas failed: %s\n", err.c_str());
			return 3;
		}
		printf("solve: best route (%d scored ticks, %d abs) written -> %s\n",
			frel, ftick, out_path.c_str());
		printf("solve: play it in-game from the Record tab (it loads like any "
			"recording; teleport-to-anchor + play).\n");
		fflush(stdout);
		return 0;
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
