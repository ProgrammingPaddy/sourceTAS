#include "SolverAssemble.h"

#include <float.h>
#include <math.h>
#include <time.h>

#include <algorithm>
#include <map>
#include <utility>

#include "SolverAir.h"
#include "SolverCarve.h"
#include "SolverEnvelope.h"
#include "SolverField.h"
#include "SolverRouteSearch.h"
#include "SolverSearchLog.h"
#include "SolverSteer.h"

namespace Solver {
namespace Assemble {

	namespace {

		// Carve init-family win counts across a solve (win-rate data
		// for pruning; single-threaded).
		int g_fam_wins[32] = { 0 };
		// Board-heatmap report rendering: once per tap face per solve.
		bool g_heat_done[64] = { false };

		PlayerState SpawnState(const World& w, const MoveParams& p,
		                       const TapeAnchor& a) {
			PlayerState s;
			s.pos = a.origin;
			s.vel = a.velocity;
			s.ducked = a.ducked;
			s.hull_state = a.ducked ? 1 : 0;
			s.stamina = a.stamina;
			TraceResult tr;
			const float gf = w.TraceHull(s.pos,
				s.pos - Vec3(0.f, 0.f, 2.f), s.ducked, &tr);
			if (gf < 1.f && tr.brush >= 0
				&& tr.normal.Z >= p.walkable_z) {
				s.pos.Z -= 2.f * gf;
				s.on_ground = true;
				s.ground_brush = tr.brush;
			}
			return s;
		}

		Vec3 ClosestVert(const std::vector<Vec3>& verts,
		                 const Vec3& to) {
			Vec3 best = verts.empty() ? to : verts[0];
			float bd = FLT_MAX;
			for (const Vec3& v : verts) {
				const float dx = v.X - to.X, dy = v.Y - to.Y;
				const float d = dx * dx + dy * dy;
				if (d < bd) {
					bd = d;
					best = v;
				}
			}
			return best;
		}

		// Ballistic feasibility of a flight from (z0, in-plane vz
		// family at s2d) covering dist into the z band [zlo, zhi] -
		// the M1.1 closed forms, used to pick departures the NEXT leg
		// can actually fly.
		bool CanFly(float z0, float tan_slope, float s2d, float dist,
		            float zlo, float zhi, const MoveParams& p,
		            int max_n) {
			const float c2 = p.air_speed_cap * p.air_speed_cap;
			const float vz_hi = tan_slope * s2d;
			const float vz_lo = -vz_hi;
			float d_cov = 0.f;
			for (int n = 1; n <= max_n; ++n) {
				const float t = static_cast<float>(n);
				const float drop = 0.5f * p.gravity * p.dt * p.dt
					* t * t;
				const float z_hi = z0 + t * p.dt * vz_hi - drop;
				const float z_lo = z0 + t * p.dt * vz_lo - drop;
				d_cov += sqrtf(s2d * s2d + c2 * t) * p.dt;
				if (z_hi < zlo - 40.f)
					return false;
				if (z_lo <= zhi + 40.f && z_hi >= zlo - 40.f
					&& d_cov >= dist)
					return true;
			}
			return false;
		}

		// The departure anchor for leaving `fc` toward the next target
		// region: the 2D-closest vertex among those the next flight is
		// FEASIBLE from; fallback = the highest vertex (altitude buys
		// reach).
		Vec3 PickDeparture(const Route::Face& fc, const Vec3& next_t,
		                   float next_zlo, float next_zhi, float s2d,
		                   const MoveParams& p) {
			const float h = sqrtf(fc.n.X * fc.n.X + fc.n.Y * fc.n.Y);
			const float tan_slope = fc.n.Z > 0.05f ? h / fc.n.Z : 3.f;
			Vec3 best;
			float bd = FLT_MAX;
			bool have = false;
			Vec3 high = fc.verts.empty() ? fc.centroid : fc.verts[0];
			for (const Vec3& v : fc.verts) {
				if (v.Z > high.Z)
					high = v;
				const float dx = v.X - next_t.X;
				const float dy = v.Y - next_t.Y;
				const float d = sqrtf(dx * dx + dy * dy);
				if (!CanFly(v.Z, tan_slope, s2d, d, next_zlo,
					next_zhi, p, 240))
					continue;
				if (d < bd) {
					bd = d;
					best = v;
					have = true;
				}
			}
			return have ? best : high;
		}

		struct StartPlan {
			bool ok = false;
			PlayerState entry;   // first airborne state
			std::vector<TapeFrame> frames;
		};

		// One prestrafe-circle + jump candidate: spin toward
		// launch_deg at rate, jump at tick `hold`.
		StartPlan PlanStartOne(const World& w, const MoveParams& p,
		                       const PlayerState& spawn,
		                       float launch_deg, float rate, int hold) {
			StartPlan sp;
			PlayerState s = spawn;
			for (int k = 0; k <= hold + 6; ++k) {
				TapeFrame f;
				f.yaw = launch_deg - rate
					* static_cast<float>(hold - k);
				f.fmove = 450.f;
				f.smove = -450.f * (rate >= 0.f ? 1.f : -1.f);
				f.buttons = (k >= hold) ? IN_JUMP : 0;
				TickEvents ev;
				MoveTick(s, w, p, 0.f, f.yaw, f.fmove, f.smove,
					0.f, f.buttons, &ev);
				sp.frames.push_back(f);
				if (!s.on_ground) {
					sp.ok = true;
					sp.entry = s;
					return sp;
				}
			}
			return sp;
		}

		void AppendAir(std::vector<TapeFrame>* frames,
		               const Air::Result& r, bool ducked) {
			for (int i = 0; i < r.tick
				&& i < static_cast<int>(r.yaw.size()); ++i) {
				TapeFrame f;
				f.yaw = r.yaw[i];
				f.fmove = r.fmove[i];
				f.smove = r.smove[i];
				f.buttons = ducked ? IN_DUCK : 0;
				frames->push_back(f);
			}
		}

		void AppendCarve(std::vector<TapeFrame>* frames,
		                 const Carve::Result& r, bool ducked) {
			for (int i = 0; i < r.tick
				&& i < static_cast<int>(r.yaw.size()); ++i) {
				TapeFrame f;
				f.yaw = r.yaw[i];
				f.fmove = r.fmove[i];
				f.smove = r.smove[i];
				f.buttons = (ducked
					|| (r.duck_at >= 0 && i >= r.duck_at))
					? IN_DUCK : 0;
				frames->push_back(f);
			}
		}

	} // namespace

	// The finish is ON TOP of the end brush (user 2026-08-16: "all it
	// needs to do is make it on top of the brush, there is no boarding
	// requirement") - hull-expanded xy so standing on the edge counts,
	// z from just under the TOP so side approaches below the top are
	// NOT a finish.
	void ZoneVolume(const WorldBrush& eb, Vec3* zmin, Vec3* zmax) {
		*zmin = Vec3(eb.bmin.X - 16.f, eb.bmin.Y - 16.f,
			eb.bmax.Z - 4.f);
		*zmax = Vec3(eb.bmax.X + 16.f, eb.bmax.Y + 16.f,
			eb.bmax.Z + 120.f);
	}

	bool InZone(const World& w, int end_brush, const Vec3& pos) {
		if (end_brush < 0)
			return false;
		Vec3 zmin, zmax;
		ZoneVolume(w.brushes[end_brush], &zmin, &zmax);
		return pos.X >= zmin.X && pos.X <= zmax.X
			&& pos.Y >= zmin.Y && pos.Y <= zmax.Y
			&& pos.Z >= zmin.Z && pos.Z <= zmax.Z;
	}

	int ZoneTick(const World& w, const Route::Graph& g,
	             const MoveParams& p, const TapeAnchor& anchor,
	             const std::vector<TapeFrame>& frames) {
		PlayerState s = SpawnState(w, p, anchor);
		for (size_t t = 0; t < frames.size(); ++t) {
			const TapeFrame& f = frames[t];
			TickEvents ev;
			MoveTick(s, w, p, f.pitch, f.yaw, f.fmove, f.smove,
				f.umove, f.buttons, &ev);
			if (InZone(w, g.end_brush, s.pos))
				return static_cast<int>(t);
		}
		return -1;
	}

	namespace {

		// The final leg: fly at the zone center; if grounded short,
		// run the rest. Frames appended; returns the zone tick offset
		// within the appended segment (-1 = failed).
		int FlyToZone(const World& w, const Route::Graph& g,
		              const MoveParams& p, PlayerState s,
		              std::vector<TapeFrame>* frames) {
			const WorldBrush& eb = w.brushes[g.end_brush];
			const Vec3 center(0.5f * (eb.bmin.X + eb.bmax.X),
				0.5f * (eb.bmin.Y + eb.bmax.Y), eb.bmax.Z);
			Steer::Controller ctl;
			const int hold = s.ducked ? IN_DUCK : 0;
			for (int k = 0; k < 800; ++k) {
				const float theta = atan2f(center.Y - s.pos.Y,
					center.X - s.pos.X);
				TapeFrame f;
				if (!s.on_ground) {
					ctl.Tick(s, p, theta, k, &f.yaw, &f.fmove,
						&f.smove);
					f.buttons = hold;
				} else {
					f.yaw = theta * 57.29578f;
					f.fmove = 450.f;
					f.smove = 0.f;
					f.buttons = 0;
				}
				TickEvents ev;
				MoveTick(s, w, p, 0.f, f.yaw, f.fmove, f.smove, 0.f,
					f.buttons, &ev);
				frames->push_back(f);
				if (InZone(w, g.end_brush, s.pos))
					return k;
			}
			return -1;
		}

		// keep_partial: called at every failure exit so the deepest
		// playable chain survives even when the shape never finishes.
		bool AssembleShape(const World& w, const Route::Graph& g,
		                   const MoveParams& p, const TapeAnchor& anchor,
		                   const std::vector<int>& shape,
		                   const Opts& o, RunResult* out,
		                   std::map<std::pair<int, int>, StartPlan>*
		                       start_cache,
		                   RunResult* partial = nullptr) {
			const WorldBrush& eb = w.brushes[g.end_brush];
			const Vec3 zone_c(0.5f * (eb.bmin.X + eb.bmax.X),
				0.5f * (eb.bmin.Y + eb.bmax.Y), eb.bmax.Z);
			// Recorder stage prefix: which shape this work belongs to.
			std::string tag = "[";
			for (size_t i = 0; i < shape.size(); ++i) {
				char b[16];
				snprintf(b, sizeof(b), "%s%d", i ? " " : "", shape[i]);
				tag += b;
			}
			tag += "]";
			auto Stage = [&](const std::string& what) {
				if (SearchLog::g_sink)
					SearchLog::g_sink->BeginStage(tag + " " + what);
			};
			// The COMMITTED chain so far - every evaluated candidate
			// is stamped with it, so a picked line in the report
			// answers "what route fed into this".
			std::string chain_ctx = tag + " from spawn";
			auto Ctx = [&]() {
				if (SearchLog::g_sink)
					SearchLog::g_sink->SetContext(chain_ctx);
			};
			Ctx();
			// START: pick the (bearing offset, spin rate, jump tick)
			// whose flight produces the SOFTEST first board - launch
			// speed toward the face is exactly the head-on setup that
			// plunges; the board is the objective, so score plans by
			// a quick air solve each. The choice depends only on the
			// first TWO faces, so it is CACHED across shapes (the
			// repeated searches were eating the wall budget before
			// the deep shapes ran).
			const std::pair<int, int> skey(shape[0],
				shape.size() > 1 ? shape[1] : -1);
			const auto sc_it = start_cache->find(skey);
			if (sc_it != start_cache->end() && !sc_it->second.ok) {
				printf("assemble: shape[0]=%d START PLAN FAILED "
					"(cached)\n", shape[0]);
				return false;
			}
			StartPlan sp;
			if (sc_it != start_cache->end()) {
				sp = sc_it->second;
			} else {
			const Route::Face& f0 = g.faces[shape[0]];
			const PlayerState spawn = SpawnState(w, p, anchor);
			const float b0 = atan2f(f0.centroid.Y - spawn.pos.Y,
				f0.centroid.X - spawn.pos.X) * 57.29578f;
			Air::Target at0;
			at0.face = shape[0];
			at0.aim = f0.centroid;
			at0.aim_region = true;
			at0.dot_cap = 300.f;
			at0.max_ticks = 240;
			// Stage 1: rank plans by their BOARD (cheap probes).
			Stage("start probes");
			struct StartCand { StartPlan sp; float sc; };
			std::vector<StartCand> cands;
			const float offs[3] = { 0.f, -35.f, 35.f };
			const float rates[4] = { 3.f, 4.5f, 6.f, 8.f };
			const int holds[2] = { 45, 75 };
			for (int oi = 0; oi < 3; ++oi)
			for (int ri = 0; ri < 4; ++ri)
			for (int di = -1; di <= 1; di += 2)
			for (int hi = 0; hi < 2; ++hi) {
				StartPlan cand = PlanStartOne(w, p, spawn,
					b0 + offs[oi],
					rates[ri] * static_cast<float>(di), holds[hi]);
				if (!cand.ok)
					continue;
				const Air::Result probe = Air::SolveTransfer(
					cand.entry, w, p, g, at0, 4, 800);
				if (!probe.hit)
					continue;
				cands.push_back({ cand,
					-probe.dot - 0.01f * probe.speed2d });
			}
			if (cands.empty()) {
				printf("assemble: shape[0]=%d START PLAN FAILED\n",
					shape[0]);
				(*start_cache)[skey] = StartPlan();
				return false;
			}
			std::sort(cands.begin(), cands.end(),
				[](const StartCand& a, const StartCand& b) {
					return a.sc < b.sc;
				});
			// Stage 2 (testimony: the board must SET UP the next
			// transfer): for the best few boards, run the ACTUAL
			// first transfer and choose the start by its strike -
			// the first board of the map serves the whole chain,
			// not its own dot.
			sp = cands[0].sp;
			if (shape.size() > 1) {
				Stage("start deep");
				float best_deep = FLT_MAX;
				const int deep_n = static_cast<int>(cands.size()) < 4
					? static_cast<int>(cands.size()) : 4;
				for (int ci = 0; ci < deep_n; ++ci) {
					const Vec3 next_t0 =
						g.faces[shape[1]].centroid;
					const Vec3 va = ClosestVert(f0.verts,
						cands[ci].sp.entry.pos);
					const Vec3 vn = ClosestVert(f0.verts, next_t0);
					Air::Target at;
					at.face = shape[0];
					at.aim = Scale(f0.centroid, 0.4f)
						+ Scale(va, 0.3f) + Scale(vn, 0.3f);
					at.aim_region = true;
					at.dot_cap = 300.f;
					at.max_ticks = 240;
					Air::Result ar = Air::SolveTransfer(
						cands[ci].sp.entry, w, p, g, at, 4,
						o.air_evals);
					if (!ar.hit)
						continue;
					const Route::Face& nf0 = g.faces[shape[1]];
					Carve::Target ct0;
					ct0.face = shape[0];
					ct0.max_ticks = 320;
					ct0.tap_brush = nf0.brush;
					ct0.tap_side = nf0.side;
					ct0.tap_face = shape[1];
					ct0.exit_heading = atan2f(
						next_t0.Y - ar.end_state.pos.Y,
						next_t0.X - ar.end_state.pos.X);
					const float se = Len2D(ar.end_state.vel);
					const float es = se > 100.f
						? Len(next_t0 - ar.end_state.pos)
							/ (se * p.dt) : 80.f;
					ct0.aim_tick = es < 10.f ? 10
						: (es > 240.f ? 240
							: static_cast<int>(es));
					ct0.tick_w = 0.02f;
					Carve::Result c0 = Carve::SolveCarve(
						ar.end_state, w, p, g, ct0, 6, 8000);
					const bool tap0 = !c0.exited && c0.tick > 0
						&& c0.struck_brush == ct0.tap_brush
						&& c0.struck_plane == ct0.tap_side;
					const float dsc = tap0
						? 0.6f * fabsf(c0.strike_dot)
							- 0.01f * Len2D(c0.end_state.vel)
						: 1e5f + c0.miss_dist;
					printf("assemble: start cand %d board %.1f -> "
						"transfer %s%.1f\n", ci, -cands[ci].sc,
						tap0 ? "dot " : "MISS ",
						tap0 ? c0.strike_dot : c0.miss_dist);
					if (dsc < best_deep) {
						best_deep = dsc;
						sp = cands[ci].sp;
					}
				}
			}
			(*start_cache)[skey] = sp;
			}
			RunResult rr;
			rr.shape = shape;
			rr.frames = sp.frames;
			PlayerState cur = sp.entry;
			chain_ctx += " | start jump";
			Ctx();
			bool skip_air = false;   // set by a TAP transfer: the
			                         // carve boarded the next face
			auto keep_partial = [&](int legs) {
				if (!partial)
					return;
				if (legs > partial->legs_done
					|| (legs == partial->legs_done
						&& rr.frames.size() > partial->frames.size())) {
					*partial = rr;
					partial->legs_done = legs;
					partial->finished = false;
					partial->zone_tick = -1;
				}
			};
			for (size_t li = 0; li < shape.size(); ++li) {
				const int fi = shape[li];
				const Route::Face& fc = g.faces[fi];
				const PlayerState leg_in = cur;
				// Rolling context: the aim blends the approach side,
				// the face center, and the side facing the NEXT
				// target (face or zone).
				const Vec3 next_t = li + 1 < shape.size()
					? g.faces[shape[li + 1]].centroid : zone_c;
				const Vec3 va = ClosestVert(fc.verts, cur.pos);
				const Vec3 vn = ClosestVert(fc.verts, next_t);
				Vec3 aim = Scale(fc.centroid, 0.4f)
					+ Scale(va, 0.3f) + Scale(vn, 0.3f);
				float leg_board_dot = 0.f;
				// A leg's carve (tap / zone / lob fallback) from a
				// given entry, for ANY leg index - shared by every
				// board alternative AND by the depth-2 probe, so the
				// choice IS the executed thing. Mid legs run the
				// UNIFIED TRANSFER (ride flows into flight, ends at
				// the strike, priced with the leg-after climb); the
				// last leg runs ZONE MODE (ride -> flight -> end-zone
				// entry, priced by arrival tick).
				struct LegOut {
					Carve::Result cr;
					Carve::Target ct;
					float sc = 1e9f;
				};
				auto SolveLegAt = [&](const PlayerState& entry,
					size_t lidx, int knots, int evals,
					std::vector<LegOut>* multi) -> LegOut {
					const int lfi = shape[lidx];
					const Route::Face& lfc = g.faces[lfi];
					const Vec3 lnext = lidx + 1 < shape.size()
						? g.faces[shape[lidx + 1]].centroid
						: zone_c;
					LegOut lo;
					Carve::Target& ct = lo.ct;
					ct.face = lfi;
					ct.max_ticks = 320;
					const float s_est = Len2D(entry.vel);
					const float est = s_est > 100.f
						? Len(lnext - entry.pos) / (s_est * p.dt)
						: 80.f;
					ct.aim_tick = est < 10.f ? 10
						: (est > 240.f ? 240
							: static_cast<int>(est));
					ct.tick_w = 0.02f;
					if (lidx + 1 < shape.size()) {
						const Route::Face& nf =
							g.faces[shape[lidx + 1]];
						ct.tap_brush = nf.brush;
						ct.tap_side = nf.side;
						ct.tap_face = shape[lidx + 1];
						if (lidx + 2 < shape.size())
							ct.next_face = shape[lidx + 2];
						else {
							ct.next_is_zone = true;
							ZoneVolume(eb, &ct.zone_min,
								&ct.zone_max);
						}
						ct.exit_heading = atan2f(
							lnext.Y - entry.pos.Y,
							lnext.X - entry.pos.X);
						// THE BOARD HEATMAP CONTROLS CONTACT (user):
						// hot cells scheduled 3x, warm 1x, cold and
						// non-viable never - the search's density IS
						// the map's density. Also renders the map
						// into the solve report (once per face).
						Field::FaceMap fmap;
						{
							Field::NextCtx fctx;
							if (lidx + 2 < shape.size()) {
								fctx.has = true;
								fctx.pt = g.faces[shape[lidx + 2]]
									.centroid;
							} else {
								fctx.has = true;
								fctx.is_zone = true;
								ZoneVolume(eb, &fctx.zmin,
									&fctx.zmax);
							}
							fmap = Field::Compute(
								entry.pos, entry.vel, nf, p,
								entry.ducked, 32.f, 300, 1.f,
								&fctx);
							// BLIND-ENTRY FIX: when the raw entry
							// cannot ballistically see the tap face
							// (the ride must climb first), the map
							// comes from the EXIT MAP's best
							// departure - its ride bookkeeping sees
							// the climb the entry cannot. Without
							// this, the leg silently loses cells,
							// bar, and aim (the -545 slam's path).
							if (fmap.best < 0) {
								Field::NextTarget xnt;
								xnt.face = &nf;
								xnt.face_idx = shape[lidx + 1];
								xnt.after = fctx;
								Field::ExitMap em =
									Field::ComputeExit(entry.pos,
									entry.vel, lfc, xnt, p,
									entry.ducked, 64.f, 12, 64.f,
									300);
								if (em.best_pot >= 0) {
									const Field::ExitSample& bs =
										em.samples[em.best_pot];
									fmap = Field::Compute(bs.pt,
										Scale(bs.dir, bs.pv), nf,
										p, entry.ducked, 32.f, 300,
										1.f, &fctx);
								}
							}
							if (fmap.best >= 0) {
								ct.field_aim =
									fmap.samples[fmap.best].q;
								ct.have_field_aim = true;
								ct.field_phi =
									fmap.samples[fmap.best].phi;
								ct.field_n =
									fmap.samples[fmap.best].n;
								ct.have_field_arr = true;
								for (int pass = 0; pass < 2
									&& ct.cells.empty(); ++pass)
								for (const Field::Sample& sm
									: fmap.samples) {
									if (!sm.reachable
										|| sm.e_eff <= 0.f)
										continue;
									if (pass == 0 && !sm.run_viable)
										continue;
									const float rel = fmap.e_hi > 0.f
										? sm.e_eff / fmap.e_hi : 0.f;
									const int rep = rel >= 0.85f ? 3
										: (rel >= 0.6f ? 1 : 0);
									for (int rp = 0; rp < rep
										&& ct.cells.size() < 192;
										++rp) {
										Carve::Target::Cell cc;
										cc.q = sm.q;
										cc.phi = sm.phi;
										cc.n = sm.n;
										ct.cells.push_back(cc);
									}
								}
							}
							// THE BALLISTIC PAIR (v2 post-mortem, the
							// piece worth keeping): on adjoining
							// faces the crossing flight is pure
							// ballistics - from each candidate
							// exit the landing point, arrival
							// velocity, and exact board dot are
							// COMPUTED, not searched. The best
							// pair (landing in-polygon, kept
							// energy over the map-best bar,
							// maximal) replaces the optimistic
							// argmax as the aim; its exit
							// direction becomes the exit heading;
							// its true tangent heading feeds the
							// arrival gradient.
							{
								Field::NextTarget bnt;
								bnt.face = &nf;
								bnt.face_idx = shape[lidx + 1];
								if (lidx + 2 < shape.size()) {
									bnt.after.has = true;
									bnt.after.pt =
										g.faces[shape[lidx + 2]]
										.centroid;
								} else {
									bnt.after.has = true;
									bnt.after.is_zone = true;
									ZoneVolume(eb, &bnt.after.zmin,
										&bnt.after.zmax);
								}
								Field::ExitMap bem =
									Field::ComputeExit(entry.pos,
									entry.vel, lfc, bnt, p,
									entry.ducked, 48.f, 12, 64.f,
									300);
								float bkept = -1e30f;
								for (const Field::ExitSample& ex
									: bem.samples) {
									// Ballistic crossing: straight
									// xy at exit speed, exact z; the
									// first front-side plane
									// crossing is the landing.
									const float s2d = ex.pv
										* Len2D(ex.dir);
									if (s2d < 100.f)
										continue;
									const float vz0 = ex.pv
										* ex.dir.Z;
									const float off0 = Dot(nf.n,
										ex.pt) - nf.d;
									if (off0 <= 0.f)
										continue;
									bool landed = false;
									Vec3 lp, lv;
									for (int n2 = 1; n2 <= 60;
										++n2) {
										const float t2 = p.dt
											* static_cast<float>(
												n2);
										Vec3 q2(ex.pt.X + ex.dir.X
											* ex.pv * t2,
											ex.pt.Y + ex.dir.Y
											* ex.pv * t2, 0.f);
										q2.Z = ex.pt.Z + vz0 * t2
											- 0.5f * p.gravity * t2
											* t2;
										const float off = Dot(nf.n,
											q2) - nf.d;
										if (off <= 0.f) {
											lp = q2;
											lv = Vec3(ex.dir.X
												* ex.pv, ex.dir.Y
												* ex.pv, vz0
												- p.gravity * t2);
											landed = true;
											break;
										}
									}
									if (!landed)
										continue;
									if (Board::EdgeDistOut(nf, lp)
										> 0.f)
										continue;
									const float dot2 = Dot(lv,
										nf.n);
									if (dot2 >= 0.f)
										continue;
									const float kept = Dot(lv, lv)
										- dot2 * dot2
										+ 2.f * p.gravity
										* (lp.Z - nf.zmin);
									if (fmap.best >= 0
										&& fmap.e_hi > 0.f
										&& kept < 0.6f * fmap.e_hi)
										continue;
									if (kept > bkept) {
										bkept = kept;
										ct.field_aim = lp;
										ct.have_field_aim = true;
										// True tangent heading at
										// the computed arrival.
										const float hn3 = sqrtf(
											nf.n.X * nf.n.X
											+ nf.n.Y * nf.n.Y);
										const float s3 = Len2D(lv);
										float cph = s3 * hn3 > 1e-4f
											? -lv.Z * nf.n.Z
											/ (s3 * hn3) : 0.f;
										if (cph > 1.f) cph = 1.f;
										if (cph < -1.f) cph = -1.f;
										const float ps3 = atan2f(
											nf.n.Y, nf.n.X);
										const float ha = atan2f(
											lv.Y, lv.X);
										const float pa2 =
											Steer::WrapPi(ps3
											+ acosf(cph));
										const float pb2 =
											Steer::WrapPi(ps3
											- acosf(cph));
										ct.field_phi = fabsf(
											Steer::WrapPi(pa2 - ha))
											<= fabsf(Steer::WrapPi(
												pb2 - ha))
											? pa2 : pb2;
										ct.have_field_arr = true;
										ct.exit_heading = atan2f(
											ex.dir.Y, ex.dir.X);
									}
								}
								if (bkept > -1e29f)
									printf("assemble: leg %d "
										"BALLISTIC PAIR kept %.0fk "
										"aim (%.0f,%.0f,%.0f)\n",
										static_cast<int>(lidx),
										bkept / 1000.f,
										ct.field_aim.X,
										ct.field_aim.Y,
										ct.field_aim.Z);
							}
							if (SearchLog::g_sink
								&& shape[lidx + 1] < 64
								&& !g_heat_done[shape[lidx + 1]]
								&& fmap.best >= 0) {
								g_heat_done[shape[lidx + 1]] = true;
								for (const Field::Sample& sm
									: fmap.samples) {
									if (!sm.reachable)
										continue;
									float v01 = fmap.e_hi > fmap.e_lo
										? (sm.e_eff - fmap.e_lo)
											/ (fmap.e_hi - fmap.e_lo)
										: 1.f;
									if (!sm.run_viable)
										v01 *= 0.35f;
									const Vec3 lift =
										Scale(nf.n, 2.f);
									const Vec3 c00 = sm.q + lift
										- Scale(fmap.ud,
											fmap.du * 0.5f)
										- Scale(fmap.vd,
											fmap.dv * 0.5f);
									const Vec3 c10 = c00
										+ Scale(fmap.ud, fmap.du);
									const Vec3 c01 = c00
										+ Scale(fmap.vd, fmap.dv);
									const Vec3 c11 = c10
										+ Scale(fmap.vd, fmap.dv);
									SearchLog::g_sink->AddHeat(
										c00, c10, c11, v01);
									SearchLog::g_sink->AddHeat(
										c00, c11, c01, v01);
								}
							}
						}
						std::vector<Carve::Result> calts;
						lo.cr = Carve::SolveCarve(entry, w, p, g,
							ct, knots, evals,
							multi ? &calts : nullptr, 3);
						if (lo.cr.family >= 0
							&& lo.cr.family < 32)
							g_fam_wins[lo.cr.family]++;
						auto TapScore =
							[&](const Carve::Result& c) -> float {
							const bool tp = !c.exited && c.tick > 0
								&& c.struck_brush == ct.tap_brush
								&& c.struck_plane == ct.tap_side;
							if (!tp)
								return 1e6f + c.miss_dist;
							// A BOARD MUST KEEP THE WARM RATIO OF
							// THE MAP'S BEST (user: non-smooth
							// boards are not an option, period).
							// Judged against e_hi, not the local
							// cell - a low-promise cell cannot
							// launder a slam through its own small
							// threshold, and cold cells are
							// subsumed (their bound sits below the
							// bar by definition). 0.6 = the
							// standing fieldgate warm ratio.
							if (fmap.best >= 0 && fmap.e_hi > 0.f) {
								const float kept =
									Dot(c.end_state.vel,
										c.end_state.vel)
									+ 2.f * p.gravity
									* (c.end_state.pos.Z - nf.zmin);
								if (kept < 0.6f * fmap.e_hi)
									return 8e5f
										+ fabsf(c.strike_dot);
							}
							// SELECTION prices by ENERGY (potential-
							// ledger units), separate from the
							// search-shaping score inside SolveCarve
							// (softness+carry, which finds diverse
							// rideable strikes). Measured: the soft
							// -7.5 tap carries 440 where the ending
							// needs ~850; the energy pick keeps the
							// fast rideable alternative.
							const float cv = Len(c.end_state.vel)
								- c.next_cost;
							return -(cv * cv) * 0.001f + 0.9f
								* static_cast<float>(c.tick);
						};
						lo.sc = TapScore(lo.cr);
						if (multi) {
							for (Carve::Result& c : calts) {
								LegOut m;
								m.ct = ct;
								m.cr = c;
								m.sc = TapScore(c);
								multi->push_back(m);
							}
							if (multi->empty())
								multi->push_back(lo);
						}
					} else {
						ct.to_zone = true;
						ZoneVolume(eb, &ct.zone_min, &ct.zone_max);
						// Aim at the zone's NEAREST RIM from here -
						// the center of a sprawling platform is a
						// misleading target.
						Vec3 rim = entry.pos;
						if (rim.X < ct.zone_min.X)
							rim.X = ct.zone_min.X;
						if (rim.X > ct.zone_max.X)
							rim.X = ct.zone_max.X;
						if (rim.Y < ct.zone_min.Y)
							rim.Y = ct.zone_min.Y;
						if (rim.Y > ct.zone_max.Y)
							rim.Y = ct.zone_max.Y;
						ct.exit_heading = atan2f(
							rim.Y - entry.pos.Y,
							rim.X - entry.pos.X);
						// The ending's spline domain spans the
						// RUNWAY PATH (ride the face's length
						// banking wish work, then fly) - a straight
						// line to the rim cuts the long ride's
						// knots dead.
						float far_d = 0.f;
						Vec3 farv = lfc.centroid;
						for (const Vec3& v : lfc.verts) {
							const float dx = v.X - entry.pos.X;
							const float dy = v.Y - entry.pos.Y;
							const float d = sqrtf(dx * dx
								+ dy * dy);
							if (d > far_d) {
								far_d = d;
								farv = v;
							}
						}
						Vec3 rim2 = farv;
						if (rim2.X < ct.zone_min.X)
							rim2.X = ct.zone_min.X;
						if (rim2.X > ct.zone_max.X)
							rim2.X = ct.zone_max.X;
						if (rim2.Y < ct.zone_min.Y)
							rim2.Y = ct.zone_min.Y;
						if (rim2.Y > ct.zone_max.Y)
							rim2.Y = ct.zone_max.Y;
						const float fdx = rim2.X - farv.X;
						const float fdy = rim2.Y - farv.Y;
						const float fly_d = sqrtf(fdx * fdx
							+ fdy * fdy);
						// The climbing ride DECELERATES (2g*dz of
						// speed^2 spent to the crest) - an entry-
						// speed estimate halves the real domain
						// and the runway families turn out
						// mid-face. Crest speed from the energy
						// law prices the ride; the flight flies
						// at crest speed.
						float zmax_f = lfc.centroid.Z;
						for (const Vec3& v : lfc.verts)
							if (v.Z > zmax_f)
								zmax_f = v.Z;
						const float dzc = zmax_f - entry.pos.Z;
						const float sc2 = s_est * s_est
							- 2.f * p.gravity
							* (dzc > 0.f ? dzc : 0.f);
						const float s_crest = sc2 > 10000.f
							? sqrtf(sc2) : 100.f;
						const float ride_t = far_d
							/ (0.5f * (s_est + s_crest));
						const float fly_t = fly_d / s_crest;
						const float ez = (ride_t + fly_t) / p.dt;
						ct.aim_tick = ez < 10.f ? 10
							: (ez > 300.f ? 300
								: static_cast<int>(ez));
						std::vector<Carve::Result> calts;
						lo.cr = Carve::SolveCarve(entry, w, p, g,
							ct, knots, evals,
							multi ? &calts : nullptr, 2);
						if (!lo.cr.zoned) {
							// Fallback: the old lob exit + fly/run.
							Carve::Target lt = ct;
							lt.to_zone = false;
							const float dh_az = atan2f(
								lfc.downhill.Y, lfc.downhill.X);
							float bearing = atan2f(
								lnext.Y - entry.pos.Y,
								lnext.X - entry.pos.X);
							const float rel = Steer::WrapPi(
								bearing - dh_az);
							if (cosf(rel) > -0.05f)
								bearing = dh_az
									+ (rel >= 0.f ? 1.f : -1.f)
									* (Steer::kSteerPi * 0.5f
										+ 0.1f);
							lt.exit_heading =
								Steer::WrapPi(bearing);
							lt.aim_vel = Vec3(
								cosf(lt.exit_heading) * s_est,
								sinf(lt.exit_heading) * s_est,
								140.f);
							lt.vel_w = 0.8f;
							Carve::Result fb = Carve::SolveCarve(
								entry, w, p, g, lt, 4, evals);
							if (fb.exited) {
								lo.cr = fb;
								lo.ct = lt;
							}
						}
						auto ZoneScore =
							[](const Carve::Result& c) -> float {
							return c.zoned
								? -2000.f
									+ static_cast<float>(c.tick)
								: (c.exited
									? 500.f - 0.01f
										* Len2D(c.exit_vel)
									: 1e6f + c.miss_dist);
						};
						lo.sc = ZoneScore(lo.cr);
						if (multi) {
							for (Carve::Result& c : calts) {
								LegOut m;
								m.ct = ct;
								m.cr = c;
								m.sc = ZoneScore(c);
								multi->push_back(m);
							}
							if (multi->empty())
								multi->push_back(lo);
						}
					}
					return lo;
				};
				char stbuf[32];
				snprintf(stbuf, sizeof(stbuf), "leg%d carve",
					static_cast<int>(li));
				// PER-LEG BEAM (the rolling window at EVERY leg):
				// gather (board alt x carve alt) candidates, price
				// the promising ones with a DEPTH-2 probe of the
				// next leg from their landing, commit the best WHOLE
				// transfer. A landing that strands the leg after it
				// is priced by that leg's own failure, not guessed.
				struct Cand { int ai; LegOut lo; float comp; };
				std::vector<Cand> cpool;
				std::vector<Air::Result> alts;
				if (skip_air) {
					skip_air = false;   // tap already boarded us
					Stage(stbuf);
					std::vector<LegOut> m;
					SolveLegAt(cur, li, 6, o.carve_evals, &m);
					for (LegOut& mo : m)
						cpool.push_back({ -1, mo, mo.sc });
				} else {
					char abuf[32];
					snprintf(abuf, sizeof(abuf), "leg%d air",
						static_cast<int>(li));
					Stage(abuf);
					Air::Target at;
					at.face = fi;
					at.aim = aim;
					at.aim_region = true;
					at.dot_cap = 300.f;
					at.max_ticks = 240;
					// Field-guided aim: runway-aware hotspot best
					// for this entry (guidance only - the region
					// gradient and families still govern).
					{
						Field::NextCtx fctx;
						fctx.has = true;
						if (li + 1 < shape.size()) {
							fctx.pt =
								g.faces[shape[li + 1]].centroid;
						} else {
							fctx.is_zone = true;
							ZoneVolume(eb, &fctx.zmin,
								&fctx.zmax);
						}
						Field::FaceMap fmap = Field::Compute(
							cur.pos, cur.vel, fc, p, cur.ducked,
							32.f, 300, 1.f, &fctx);
						if (fmap.best >= 0) {
							at.aim = fmap.samples[fmap.best].q;
							// The cell's arrival heading + timing
							// feed the CONSTRUCTED path (SolverPath)
							// - the air solve flies it first.
							at.arr_phi =
								fmap.samples[fmap.best].phi;
							at.arr_n = static_cast<int>(
								fmap.samples[fmap.best].n);
							at.have_arr = true;
						}
					}
					Air::Result ar = Air::SolveTransfer(cur, w, p,
						g, at, 4, o.air_evals, &alts, 3);
					if (!ar.hit) {
						at.aim = fc.centroid;
						ar = Air::SolveTransfer(cur, w, p, g, at, 4,
							o.air_evals, &alts, 3);
					}
					if (!ar.hit) {
						printf("assemble: leg %d AIR MISS to face "
							"%d (closest %.0f, %s%d) from (%.0f,"
							"%.0f,%.0f v %.0f)\n",
							static_cast<int>(li), fi, ar.miss_dist,
							ar.grounded ? "grounded " : "struck ",
							ar.struck_brush, cur.pos.X, cur.pos.Y,
							cur.pos.Z, Len2D(cur.vel));
						keep_partial(static_cast<int>(li));
						return false;
					}
					for (size_t ai = 0; ai < alts.size(); ++ai) {
						Stage(stbuf);
						std::vector<LegOut> m;
						SolveLegAt(alts[ai].end_state, li, 6,
							o.carve_evals, &m);
						for (LegOut& mo : m)
							cpool.push_back({
								static_cast<int>(ai), mo,
								mo.sc
									+ 0.6f * fabsf(alts[ai].dot) });
					}
				}
				if (cpool.empty()) {
					printf("assemble: leg %d NO CANDIDATES\n",
						static_cast<int>(li));
					keep_partial(static_cast<int>(li));
					return false;
				}
				std::sort(cpool.begin(), cpool.end(),
					[](const Cand& a, const Cand& b) {
						return a.comp < b.comp;
					});
				if (cpool.size() > 3)
					cpool.resize(3);
				for (Cand& c : cpool) {
					const bool tp = !c.lo.cr.exited
						&& c.lo.ct.tap_brush >= 0
						&& c.lo.cr.struck_brush
							== c.lo.ct.tap_brush
						&& c.lo.cr.struck_plane
							== c.lo.ct.tap_side;
					float d2 = 0.f;
					if (tp && li + 1 < shape.size()) {
						char pbuf[32];
						snprintf(pbuf, sizeof(pbuf), "leg%d probe",
							static_cast<int>(li) + 1);
						Stage(pbuf);
						LegOut nx = SolveLegAt(c.lo.cr.end_state,
							li + 1, 6, (o.carve_evals * 2) / 3,
							nullptr);
						d2 = nx.sc;
						c.comp += d2;
					}
					printf("assemble: leg %d cand board %d leg %.1f "
						"depth2 %.1f\n", static_cast<int>(li),
						c.ai, c.lo.sc, d2);
				}
				std::sort(cpool.begin(), cpool.end(),
					[](const Cand& a, const Cand& b) {
						return a.comp < b.comp;
					});
				// A leg with NO bar-passing board FAILS - the bar
				// marks failures (8e5+) but the commit previously
				// took the least-bad anyway, which is how a -644
				// slam with 180k kept re-entered committed chains.
				if (cpool[0].lo.sc >= 7.9e5f) {
					printf("assemble: leg %d NO BAR-PASSING BOARD "
						"(best %.0f)\n", static_cast<int>(li),
						cpool[0].lo.sc);
					keep_partial(static_cast<int>(li));
					return false;
				}
				LegOut lo = cpool[0].lo;
				if (cpool[0].ai >= 0) {
					const Air::Result& ba = alts[cpool[0].ai];
					AppendAir(&rr.frames, ba, cur.ducked);
					rr.board_loss2 += ba.dot * ba.dot;
					leg_board_dot = ba.dot;
					cur = ba.end_state;
					char cb[64];
					snprintf(cb, sizeof(cb), " | L%d board f%d %.0f",
						static_cast<int>(li), fi, ba.dot);
					chain_ctx += cb;
					Ctx();
				}
				Carve::Result& cr = lo.cr;
				const Carve::Target& ct = lo.ct;
				auto tapped = [&](const Carve::Result& c) {
					return !c.exited && c.tick > 0 && ct.tap_brush >= 0
						&& c.struck_brush == ct.tap_brush
						&& c.struck_plane == ct.tap_side;
				};
				if (cr.zoned) {
					AppendCarve(&rr.frames, cr, cur.ducked);
					rr.finished = true;
					rr.zone_tick =
						static_cast<int>(rr.frames.size()) - 1;
					printf("assemble: leg %d ZONE ENTRY at tick %d "
						"(%.0f,%.0f,%.0f v %.0f)\n",
						static_cast<int>(li), cr.tick,
						cr.end_pos.X, cr.end_pos.Y, cr.end_pos.Z,
						Len2D(cr.end_state.vel));
					*out = rr;
					return true;
				}
				if (!cr.exited && !tapped(cr)) {
					printf("assemble: leg %d CARVE FAILED on face "
						"%d (%s%d, miss %.0f)\n",
						static_cast<int>(li), fi,
						cr.grounded ? "grounded " : "struck ",
						cr.struck_brush, cr.miss_dist);
					keep_partial(static_cast<int>(li));
					return false;
				}
				AppendCarve(&rr.frames, cr, cur.ducked);
				cur = cr.end_state;
				char cb[96];
				if (tapped(cr)) {
					rr.board_loss2 += cr.strike_dot
						* cr.strike_dot;
					skip_air = true;
					printf("assemble: leg %d TRANSFER -> face %d "
						"dot %.1f at (%.0f,%.0f,%.0f) v(%.0f,"
						"%.0f,%.0f)\n",
						static_cast<int>(li), shape[li + 1],
						cr.strike_dot, cur.pos.X, cur.pos.Y,
						cur.pos.Z, cur.vel.X, cur.vel.Y,
						cur.vel.Z);
					snprintf(cb, sizeof(cb),
						" | L%d tap f%d %.0f @(%.0f,%.0f,%.0f) "
						"v%.0f", static_cast<int>(li),
						shape[li + 1], cr.strike_dot, cur.pos.X,
						cur.pos.Y, cur.pos.Z, Len2D(cur.vel));
					chain_ctx += cb;
					Ctx();
				} else {
					printf("assemble: leg %d carve exit (%.0f,%.0f,"
						"%.0f) v(%.0f,%.0f,%.0f)\n",
						static_cast<int>(li), cur.pos.X, cur.pos.Y,
						cur.pos.Z, cur.vel.X, cur.vel.Y,
						cur.vel.Z);
					snprintf(cb, sizeof(cb),
						" | L%d exit v%.0f", static_cast<int>(li),
						Len2D(cur.vel));
					chain_ctx += cb;
					Ctx();
				}
				// THE LEG ENERGY LEDGER (user 2026-08-17: report in
				// energy, and show where it goes). Total mechanical
				// energy E = v^2 + 2g*z; losses attributed to the
				// entry board, the ride's clips, and the tap out; the
				// residual is wish work banked MINUS anything
				// unattributed - a strongly NEGATIVE resid means
				// unpriced loss (flight grazes, or ground friction:
				// the measured valley-walk signature was -712k).
				{
					const float g2 = 2.f * p.gravity;
					const float e_in = Dot(leg_in.vel, leg_in.vel)
						+ g2 * leg_in.pos.Z;
					const float e_out = Dot(cur.vel, cur.vel)
						+ g2 * cur.pos.Z;
					const float b2 = leg_board_dot * leg_board_dot;
					const float t2 = tapped(cr)
						? cr.strike_dot * cr.strike_dot : 0.f;
					const float wish = e_out - e_in + b2
						+ cr.ride_loss2 + cr.graze_loss2 + t2;
					printf("assemble: leg %d ENERGY in %.0fk (%.0f "
						"u/s @ z%.0f) -> out %.0fk (%.0f @ z%.0f) | "
						"board -%.1fk | clips -%.1fk | grazes -%.1fk "
						"| tap -%.1fk | resid %+.1fk\n",
						static_cast<int>(li), e_in / 1000.f,
						Len(leg_in.vel), leg_in.pos.Z,
						e_out / 1000.f, Len(cur.vel), cur.pos.Z,
						b2 / 1000.f, cr.ride_loss2 / 1000.f,
						cr.graze_loss2 / 1000.f, t2 / 1000.f,
						wish / 1000.f);
				}
			}
			const int zk = FlyToZone(w, g, p, cur, &rr.frames);
			if (zk < 0) {
				printf("assemble: FLY-TO-ZONE FAILED from "
					"(%.0f,%.0f,%.0f v %.0f)\n", cur.pos.X,
					cur.pos.Y, cur.pos.Z, Len2D(cur.vel));
				keep_partial(static_cast<int>(shape.size()));
				return false;
			}
			rr.finished = true;
			rr.zone_tick = static_cast<int>(rr.frames.size()) - 1;
			*out = rr;
			return true;
		}

	} // namespace

	PlayerState Spawn(const World& w, const MoveParams& p,
	                  const TapeAnchor& a) {
		return SpawnState(w, p, a);
	}

	StartCand StartOne(const World& w, const MoveParams& p,
	                   const PlayerState& spawn, float launch_deg,
	                   float rate, int hold) {
		const StartPlan sp = PlanStartOne(w, p, spawn, launch_deg,
			rate, hold);
		StartCand c;
		c.ok = sp.ok;
		c.entry = sp.entry;
		c.frames = sp.frames;
		return c;
	}

	void AppendAirFrames(std::vector<TapeFrame>* frames,
	                     const Air::Result& r, bool ducked) {
		AppendAir(frames, r, ducked);
	}

	void AppendCarveFrames(std::vector<TapeFrame>* frames,
	                       const Carve::Result& r, bool ducked) {
		AppendCarve(frames, r, ducked);
	}

	bool SolveMap(const World& w, const Route::Graph& g,
	              const MoveParams& p, const TapeAnchor& anchor,
	              const Opts& o, RunResult* out, std::string* err,
	              RunResult* partial) {
		const clock_t c0 = clock();
		RouteSearch::Opts ro;
		ro.top_k = 60;
		std::vector<RouteSearch::Candidate> routes;
		if (!RouteSearch::Enumerate(w, g, p, ro, &routes, err))
			return false;
		// Distinct base shapes in lb order (the M2 pool semantics).
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
		bool any = false;
		RunResult best;
		std::map<std::pair<int, int>, StartPlan> start_cache;
		for (const std::vector<int>& shape : shapes) {
			const double spent = static_cast<double>(clock() - c0)
				/ CLOCKS_PER_SEC;
			if (spent > o.wall_budget_s)
				break;
			RunResult rr;
			if (!AssembleShape(w, g, p, anchor, shape, o, &rr,
				&start_cache, partial))
				continue;
			printf("assemble: shape");
			for (int fidx : shape)
				printf(" %d", fidx);
			printf(" -> FINISHED in %d ticks (board loss2 %.0f)\n",
				rr.zone_tick, rr.board_loss2);
			fflush(stdout);
			if (!any || rr.zone_tick < best.zone_tick) {
				best = rr;
				any = true;
			}
		}
		// Family win-rate readout (the dilution law: families must
		// earn their budget share; prune from this data).
		printf("assemble: carve family wins:");
		for (int i = 0; i < 32; ++i)
			if (g_fam_wins[i] > 0)
				printf(" f%d:%d", i, g_fam_wins[i]);
		printf("\n");
		for (int i = 0; i < 32; ++i)
			g_fam_wins[i] = 0;
		for (int i = 0; i < 64; ++i)
			g_heat_done[i] = false;
		if (!any) {
			if (err) *err = "no shape assembled to the zone";
			return false;
		}
		*out = best;
		return true;
	}

} // namespace Assemble
} // namespace Solver
