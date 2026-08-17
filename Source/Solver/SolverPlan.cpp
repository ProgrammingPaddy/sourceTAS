#include "SolverPlan.h"

#include <float.h>
#include <math.h>
#include <time.h>

#include <algorithm>
#include <vector>

#include "SolverAir.h"
#include "SolverBoard.h"
#include "SolverCarve.h"
#include "SolverField.h"
#include "SolverRouteSearch.h"
#include "SolverSearchLog.h"

namespace Solver {
namespace Plan {

	namespace {

		struct CellPick {
			Vec3  q;
			float phi = 0.f;
			float n = 0.f;
			float e_eff = 0.f;
			float cap = 0.f;     // law-derived arrival |dot| cap
		};

		// Candidate cells for a board, hot tier first then warm, each
		// with its cap: the largest loss that still keeps the warm
		// ratio of the map's best (cap^2 = e_eff + res^2 - 0.6*e_hi;
		// e_eff = arrival energy minus res^2, so this is exact).
		void PickCells(const Field::FaceMap& fm, int want,
		               std::vector<CellPick>* out) {
			struct Row { const Field::Sample* sm; float rel; };
			std::vector<Row> rows;
			for (const Field::Sample& sm : fm.samples) {
				if (!sm.reachable || sm.e_eff <= 0.f)
					continue;
				const float rel = fm.e_hi > 0.f
					? sm.e_eff / fm.e_hi : 0.f;
				if (rel < 0.6f)
					continue;   // cold is not a candidate
				Row r;
				r.sm = &sm;
				r.rel = rel;
				rows.push_back(r);
			}
			// Viable + hot first, then viable warm, then the rest.
			std::sort(rows.begin(), rows.end(),
				[](const Row& a, const Row& b) {
					const int ta = (a.sm->run_viable ? 2 : 0)
						+ (a.rel >= 0.85f ? 1 : 0);
					const int tb = (b.sm->run_viable ? 2 : 0)
						+ (b.rel >= 0.85f ? 1 : 0);
					if (ta != tb)
						return ta > tb;
					return a.sm->e_eff > b.sm->e_eff;
				});
			for (const Row& r : rows) {
				if (static_cast<int>(out->size()) >= want)
					break;
				const float cap2 = r.sm->e_eff
					+ r.sm->residual * r.sm->residual
					- 0.6f * fm.e_hi;
				if (cap2 <= 0.f)
					continue;
				CellPick c;
				c.q = r.sm->q;
				c.phi = r.sm->phi;
				c.n = r.sm->n;
				c.e_eff = r.sm->e_eff;
				c.cap = sqrtf(cap2);
				out->push_back(c);
			}
		}

		void RenderHeat(const Field::FaceMap& fm, const Route::Face& f) {
			if (!SearchLog::g_sink)
				return;
			for (const Field::Sample& sm : fm.samples) {
				if (!sm.reachable)
					continue;
				float v01 = fm.e_hi > fm.e_lo
					? (sm.e_eff - fm.e_lo) / (fm.e_hi - fm.e_lo)
					: 1.f;
				if (!sm.run_viable)
					v01 *= 0.35f;
				const Vec3 lift = Scale(f.n, 2.f);
				const Vec3 c00 = sm.q + lift
					- Scale(fm.ud, fm.du * 0.5f)
					- Scale(fm.vd, fm.dv * 0.5f);
				const Vec3 c10 = c00 + Scale(fm.ud, fm.du);
				const Vec3 c01 = c00 + Scale(fm.vd, fm.dv);
				const Vec3 c11 = c10 + Scale(fm.vd, fm.dv);
				SearchLog::g_sink->AddHeat(c00, c10, c11, v01);
				SearchLog::g_sink->AddHeat(c00, c11, c01, v01);
			}
		}

		float EnergyAt(const PlayerState& s, float zref,
		               const MoveParams& p) {
			return Dot(s.vel, s.vel)
				+ 2.f * p.gravity * (s.pos.Z - zref);
		}

		void Stage(const char* name) {
			if (SearchLog::g_sink)
				SearchLog::g_sink->BeginStage(name);
		}

	} // namespace

	bool SolveMap(const World& w, const Route::Graph& g,
	              const MoveParams& p, const TapeAnchor& anchor,
	              const Opts& o, Assemble::RunResult* out,
	              std::string* err, Assemble::RunResult* partial) {
		const clock_t c0 = clock();
		RouteSearch::Opts ro;
		ro.top_k = 60;
		std::vector<RouteSearch::Candidate> routes;
		if (!RouteSearch::Enumerate(w, g, p, ro, &routes, err))
			return false;
		std::vector<std::vector<int>> shapes;
		for (const RouteSearch::Candidate& r : routes) {
			std::vector<int> base;
			for (int fidx : r.faces) {
				bool seen = false;
				for (int b : base)
					if (b == fidx)
						seen = true;
				if (!seen)
					base.push_back(fidx);
			}
			bool have = false;
			for (const std::vector<int>& s : shapes)
				if (s == base)
					have = true;
			if (!have)
				shapes.push_back(base);
		}
		if (static_cast<int>(shapes.size()) > o.max_shapes)
			shapes.resize(o.max_shapes);
		if (g.end_brush < 0
			|| g.end_brush >= static_cast<int>(w.brushes.size())) {
			if (err) *err = "no end brush";
			return false;
		}
		const WorldBrush& eb = w.brushes[g.end_brush];
		Vec3 zmin, zmax;
		Assemble::ZoneVolume(eb, &zmin, &zmax);
		bool any = false;
		Assemble::RunResult best;
		for (const std::vector<int>& shape : shapes) {
			const double spent = static_cast<double>(clock() - c0)
				/ CLOCKS_PER_SEC;
			if (spent > o.wall_budget_s)
				break;
			if (shape.empty())
				continue;
			{
				std::string tag = "v2 [";
				for (size_t i = 0; i < shape.size(); ++i) {
					char b[16];
					snprintf(b, sizeof(b), "%s%d", i ? " " : "",
						shape[i]);
					tag += b;
				}
				tag += "]";
				printf("plan: %s\n", tag.c_str());
				if (SearchLog::g_sink)
					SearchLog::g_sink->SetContext(tag);
			}
			Assemble::RunResult rr;
			rr.shape = shape;
			// START: prestrafe candidates ranked by the CONSTRUCTED
			// first board (fly to the first face's best cell under
			// its cap; delivered energy decides).
			Stage("v2 start");
			const PlayerState spawn = Assemble::Spawn(w, p, anchor);
			const Route::Face& f0 = g.faces[shape[0]];
			const float b0 = atan2f(f0.centroid.Y - spawn.pos.Y,
				f0.centroid.X - spawn.pos.X) * 57.29578f;
			PlayerState state;
			bool have_start = false;
			float start_e = -1e30f;
			Air::Result start_fly;
			Assemble::StartCand start_sc;
			const float offs[3] = { 0.f, -35.f, 35.f };
			const float rates[4] = { 3.f, 4.5f, 6.f, 8.f };
			const int holds[2] = { 45, 75 };
			for (int oi = 0; oi < 3; ++oi)
			for (int ri = 0; ri < 4; ++ri)
			for (int di = -1; di <= 1; di += 2)
			for (int hi = 0; hi < 2; ++hi) {
				Assemble::StartCand sc = Assemble::StartOne(w, p,
					spawn, b0 + offs[oi],
					rates[ri] * static_cast<float>(di), holds[hi]);
				if (!sc.ok)
					continue;
				Field::NextCtx fctx;
				fctx.has = true;
				if (shape.size() > 1)
					fctx.pt = g.faces[shape[1]].centroid;
				else {
					fctx.is_zone = true;
					fctx.zmin = zmin;
					fctx.zmax = zmax;
				}
				Field::FaceMap fm = Field::Compute(sc.entry.pos,
					sc.entry.vel, f0, p, sc.entry.ducked, 32.f,
					300, 1.f, &fctx);
				std::vector<CellPick> cells;
				PickCells(fm, 4, &cells);
				for (const CellPick& c : cells) {
					Air::Target at;
					at.face = shape[0];
					at.aim = c.q;
					at.aim_region = false;
					at.dot_cap = c.cap;
					at.max_ticks = 240;
					Air::Result ar = Air::SolveTransfer(sc.entry,
						w, p, g, at, 4, 500);
					if (!ar.hit || fabsf(ar.dot) > c.cap)
						continue;
					const float e = EnergyAt(ar.end_state,
						f0.zmin, p);
					if (e > start_e) {
						start_e = e;
						start_fly = ar;
						start_sc = sc;
						have_start = true;
					}
				}
			}
			if (!have_start) {
				printf("plan: no start board under cap for shape\n");
				continue;
			}
			rr.frames = start_sc.frames;
			Assemble::AppendAirFrames(&rr.frames, start_fly,
				start_sc.entry.ducked);
			state = start_fly.end_state;
			rr.board_loss2 += start_fly.dot * start_fly.dot;
			printf("plan: start board f%d dot %.1f E %.0fk (%.0f "
				"u/s)\n", shape[0], start_fly.dot, start_e / 1000.f,
				Len(state.vel));
			bool dead = false;
			for (size_t li = 0; li < shape.size() && !dead; ++li) {
				const int fi = shape[li];
				const Route::Face& fc = g.faces[fi];
				const bool last = li + 1 >= shape.size();
				char sb[48];
				// ===== RIDE on face fi =====
				if (last) {
					// ENDING: zone-mode carve (objective = clock).
					snprintf(sb, sizeof(sb), "v2 leg%d zone",
						static_cast<int>(li));
					Stage(sb);
					Carve::Target ct;
					ct.face = fi;
					ct.max_ticks = 320;
					ct.to_zone = true;
					ct.zone_min = zmin;
					ct.zone_max = zmax;
					Vec3 rim = state.pos;
					if (rim.X < zmin.X) rim.X = zmin.X;
					if (rim.X > zmax.X) rim.X = zmax.X;
					if (rim.Y < zmin.Y) rim.Y = zmin.Y;
					if (rim.Y > zmax.Y) rim.Y = zmax.Y;
					ct.exit_heading = atan2f(rim.Y - state.pos.Y,
						rim.X - state.pos.X);
					ct.aim_tick = 200;
					Carve::Result cr = Carve::SolveCarve(state, w,
						p, g, ct, 6, o.zone_evals);
					if (cr.zoned) {
						Assemble::AppendCarveFrames(&rr.frames, cr,
							state.ducked);
						rr.finished = true;
						rr.zone_tick =
							static_cast<int>(rr.frames.size()) - 1;
						rr.legs_done =
							static_cast<int>(shape.size());
						printf("plan: ZONE ENTRY tick %d\n",
							cr.tick);
						break;
					}
					printf("plan: ending failed (miss %.0f)\n",
						cr.miss_dist);
					dead = true;
					break;
				}
				// Mid ride: departures from the exit map, solved as
				// exit specs, ranked by ENERGY DELIVERED TO THE NEXT
				// BOARD (the flight is solved per departure).
				const int nfi = shape[li + 1];
				const Route::Face& nf = g.faces[nfi];
				snprintf(sb, sizeof(sb), "v2 leg%d ride",
					static_cast<int>(li));
				Stage(sb);
				Field::NextTarget xnt;
				xnt.face = &nf;
				xnt.face_idx = nfi;
				if (li + 2 < shape.size()) {
					xnt.after.has = true;
					xnt.after.pt = g.faces[shape[li + 2]].centroid;
				} else {
					xnt.after.has = true;
					xnt.after.is_zone = true;
					xnt.after.zmin = zmin;
					xnt.after.zmax = zmax;
				}
				Field::ExitMap em = Field::ComputeExit(state.pos,
					state.vel, fc, xnt, p, state.ducked, 48.f, 12,
					48.f, 300);
				// Rank exits by the priced potential, keep diverse
				// top (both handednesses arise naturally: different
				// departure directions are different samples).
				std::vector<int> xs;
				for (size_t i = 0; i < em.samples.size(); ++i)
					if (em.samples[i].q.any
						&& em.samples[i].turn_ok)
						xs.push_back(static_cast<int>(i));
				if (xs.empty())
					for (size_t i = 0; i < em.samples.size(); ++i)
						if (em.samples[i].q.any)
							xs.push_back(static_cast<int>(i));
				std::sort(xs.begin(), xs.end(), [&](int a, int b) {
					return em.samples[a].fpot > em.samples[b].fpot;
				});
				struct Best {
					bool ok = false;
					Carve::Result ride;
					Air::Result fly;
					float e = -1e30f;
				};
				Best bestleg;
				int tried = 0;
				for (int xi : xs) {
					if (tried >= o.exits_per_leg)
						break;
					const Field::ExitSample& ex = em.samples[xi];
					// Skip exits clustered on an already-tried spot.
					tried++;
					Carve::Target ct;
					ct.face = fi;
					ct.max_ticks = 320;
					ct.aim_pos = ex.pt;
					ct.pos_w = 0.02f;
					ct.exit_heading = atan2f(ex.dir.Y, ex.dir.X);
					ct.aim_vel = Scale(ex.dir, ex.pv);
					ct.vel_w = 0.8f;
					ct.aim_tick = static_cast<int>(ex.ride_ticks)
						+ 8;
					if (ct.aim_tick < 12)
						ct.aim_tick = 12;
					Carve::Result ride = Carve::SolveCarve(state,
						w, p, g, ct, 5, o.ride_evals);
					if (!ride.exited)
						continue;
					// FLIGHT: boundary-value solves to the next
					// face's cells from the REAL exit state.
					Field::NextCtx fctx2 = xnt.after;
					Field::FaceMap fm = Field::Compute(
						ride.end_state.pos, ride.end_state.vel,
						nf, p, ride.end_state.ducked, 32.f, 300,
						1.f, &fctx2);
					if (fm.best < 0)
						continue;
					if (li == 0)
						RenderHeat(fm, nf);
					std::vector<CellPick> cells;
					PickCells(fm, o.cells_per_leg, &cells);
					char fb[48];
					snprintf(fb, sizeof(fb), "v2 leg%d fly",
						static_cast<int>(li));
					Stage(fb);
					for (const CellPick& c : cells) {
						Air::Target at;
						at.face = nfi;
						at.aim = c.q;
						at.aim_region = false;
						at.dot_cap = c.cap;
						at.max_ticks = 260;
						Air::Result ar = Air::SolveTransfer(
							ride.end_state, w, p, g, at, 4,
							o.air_evals);
						if (!ar.hit || fabsf(ar.dot) > c.cap)
							continue;
						const float e = EnergyAt(ar.end_state,
							nf.zmin, p);
						if (e > bestleg.e) {
							bestleg.ok = true;
							bestleg.e = e;
							bestleg.ride = ride;
							bestleg.fly = ar;
						}
					}
				}
				if (!bestleg.ok) {
					printf("plan: leg %d NO BOARD UNDER CAP "
						"(f%d -> f%d, %d exits tried)\n",
						static_cast<int>(li), fi, nfi, tried);
					dead = true;
					break;
				}
				const float e_in = EnergyAt(state, nf.zmin, p);
				Assemble::AppendCarveFrames(&rr.frames, bestleg.ride,
					state.ducked);
				Assemble::AppendAirFrames(&rr.frames, bestleg.fly,
					bestleg.ride.end_state.ducked);
				state = bestleg.fly.end_state;
				rr.board_loss2 += bestleg.fly.dot * bestleg.fly.dot;
				rr.legs_done = static_cast<int>(li) + 1;
				printf("plan: leg %d ride exit v%.0f -> board f%d "
					"dot %.1f | E in %.0fk out %.0fk (%.0f u/s @ "
					"z%.0f)\n", static_cast<int>(li),
					Len2D(bestleg.ride.exit_vel), nfi,
					bestleg.fly.dot, e_in / 1000.f,
					bestleg.e / 1000.f, Len(state.vel),
					state.pos.Z);
			}
			if (partial) {
				if (rr.legs_done > partial->legs_done
					|| (rr.legs_done == partial->legs_done
						&& rr.frames.size()
							> partial->frames.size())) {
					*partial = rr;
					partial->finished = false;
					if (!rr.finished)
						partial->zone_tick = -1;
				}
			}
			if (rr.finished) {
				if (!any || rr.zone_tick < best.zone_tick) {
					best = rr;
					any = true;
				}
			}
		}
		if (!any) {
			if (err) *err = "no shape planned to the zone";
			return false;
		}
		*out = best;
		return true;
	}

} // namespace Plan
} // namespace Solver
