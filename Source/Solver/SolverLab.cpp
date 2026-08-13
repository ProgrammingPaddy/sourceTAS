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
		printf("          [--rollouts N] [--max-ticks N] [--threads N] [physics options]\n");
		printf("          archive route search; best route written as a playable .tas\n");
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
		double budget_s = 30.0;
		double optimize_s = 60.0;   // Phase 2 budget after exploration (0 = off)
		long long rollouts = 0;
		float flips = 5.f;
		unsigned rng = 1337;
		float cell = 64.f;
		int max_ticks = 4000;
		int threads = 0;            // explorer/optimizer workers; 0 = auto
		bool goal_touch = false;
		bool eloss_bias = true;
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
		if (!w.Load(map_path, o.hulls, &err)) {
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
		int zone_jumps = 0;
		int first_finish = -1;
		float max_speed = 0.f;

		for (int t = 0; t < static_cast<int>(tape.frames.size()); ++t) {
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
		if (!w.Load(map_path, o.hulls, &err)) {
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

	// Phase 1: archive-explorer route search. Finds legal start-to-finish
	// routes and writes the best as a .tas the game plays like any recording.
	int CmdSolve(const std::string& map_path, const ReplayOpts& o) {
		World w;
		std::string err;
		if (!w.Load(map_path, o.hulls, &err)) {
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
		cfg.goal_touch = o.goal_touch;
		cfg.eloss_bias = o.eloss_bias;
		const int nthreads = ResolveThreadCount(o.threads);
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
			printf("  #%d  %d ticks (%.3f s)  spd %.1f  eloss %.0fk  finish "
				"(%.0f, %.0f, z %.2f)\n", i + 1, res.finishers[i].tick,
				res.finishers[i].tick * cfg.params.dt, res.finishers[i].speed,
				res.finishers[i].eloss / 1000.f,
				res.finishers[i].pos.X, res.finishers[i].pos.Y,
				res.finishers[i].pos.Z);
		if (ex.SeedFinishTick() >= 0)
			printf("  incumbent (seed tape): %d ticks (%.3f s)\n",
				ex.SeedFinishTick(), ex.SeedFinishTick() * cfg.params.dt);

		// ---- Phase 2: optimize the best finishers (fitness = finish tick,
		// strict improvements only, every eval through the proven core).
		std::vector<TapeFrame> frames;
		int ftick = 0;
		bool have_optimized = false;
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
			printf("solve: OPTIMIZING %d subject(s) in parallel, %.0fs each:\n",
				static_cast<int>(subjects.size()), o.optimize_s);
			fflush(stdout);
			ocfg.budget_seconds = o.optimize_s;
			{
				std::vector<std::thread> othreads;
				for (size_t si = 0; si < subjects.size(); ++si) {
					if (!valid[si])
						continue;
					OptimizeConfig scfg = ocfg;
					scfg.rng_seed = o.rng + static_cast<unsigned>(si) + 1;
					othreads.emplace_back([&, si, scfg]() {
						Optimizer opt2(w, scfg, root, root_yaw,
							have_seed ? &seed : nullptr);
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
				Optimizer optb(w, ocfg, root, root_yaw,
					have_seed ? &seed : nullptr);
				if (optb.BuildFrames(best_g, frames, &ftick)) {
					have_optimized = true;
					printf("solve: OPTIMIZED best %d ticks (%.3f s) vs "
						"explorer best %d\n", ftick, ftick * cfg.params.dt,
						res.finishers[0].tick);
				}
			}
		}
		if (!have_optimized
			&& !ex.BuildFrames(res.finishers[0].entry, frames, &ftick)) {
			printf("solve: BuildFrames FAILED for the best route - not writing\n");
			return 3;
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
		printf("solve: best route (%d ticks) written -> %s\n", ftick,
			out_path.c_str());
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
	PrintUsage();
	return 1;
}
