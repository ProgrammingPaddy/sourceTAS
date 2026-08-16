#include "SolverAssemble.h"

#include <float.h>
#include <math.h>
#include <time.h>

#include <algorithm>

#include "SolverAir.h"
#include "SolverCarve.h"
#include "SolverEnvelope.h"
#include "SolverRouteSearch.h"
#include "SolverSteer.h"

namespace Solver {
namespace Assemble {

	namespace {

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

	bool InZone(const World& w, int end_brush, const Vec3& pos) {
		if (end_brush < 0)
			return false;
		const WorldBrush& eb = w.brushes[end_brush];
		return pos.X >= eb.bmin.X && pos.X <= eb.bmax.X
			&& pos.Y >= eb.bmin.Y && pos.Y <= eb.bmax.Y
			&& pos.Z >= eb.bmin.Z - 4.f
			&& pos.Z <= eb.bmax.Z + 120.f;
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

		bool AssembleShape(const World& w, const Route::Graph& g,
		                   const MoveParams& p, const TapeAnchor& anchor,
		                   const std::vector<int>& shape,
		                   const Opts& o, RunResult* out) {
			const WorldBrush& eb = w.brushes[g.end_brush];
			const Vec3 zone_c(0.5f * (eb.bmin.X + eb.bmax.X),
				0.5f * (eb.bmin.Y + eb.bmax.Y), eb.bmax.Z);
			// START: pick the (bearing offset, spin rate, jump tick)
			// whose flight produces the SOFTEST first board - launch
			// speed toward the face is exactly the head-on setup that
			// plunges; the board is the objective, so score plans by
			// a quick air solve each.
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
			StartPlan sp;
			float best_sc = FLT_MAX;
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
				const float sc = -probe.dot
					- 0.01f * probe.speed2d;
				if (sc < best_sc) {
					best_sc = sc;
					sp = cand;
				}
			}
			if (!sp.ok) {
				printf("assemble: shape[0]=%d START PLAN FAILED\n",
					shape[0]);
				return false;
			}
			RunResult rr;
			rr.shape = shape;
			rr.frames = sp.frames;
			PlayerState cur = sp.entry;
			bool skip_air = false;   // set by a TAP transfer: the
			                         // carve boarded the next face
			for (size_t li = 0; li < shape.size(); ++li) {
				const int fi = shape[li];
				const Route::Face& fc = g.faces[fi];
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
				if (skip_air) {
					skip_air = false;   // tap already boarded us
				} else {
					Air::Target at;
					at.face = fi;
					at.aim = aim;
					at.aim_region = true;
					at.dot_cap = 300.f;
					at.max_ticks = 240;
					Air::Result ar = Air::SolveTransfer(cur, w, p,
						g, at, 4, o.air_evals);
					if (!ar.hit) {
						at.aim = fc.centroid;
						ar = Air::SolveTransfer(cur, w, p, g, at, 4,
							o.air_evals);
					}
					if (!ar.hit) {
						printf("assemble: leg %d AIR MISS to face "
							"%d (closest %.0f, %s%d) from (%.0f,"
							"%.0f,%.0f v %.0f)\n",
							static_cast<int>(li), fi, ar.miss_dist,
							ar.grounded ? "grounded " : "struck ",
							ar.struck_brush, cur.pos.X, cur.pos.Y,
							cur.pos.Z, Len2D(cur.vel));
						return false;
					}
					AppendAir(&rr.frames, ar, cur.ducked);
					rr.board_loss2 += ar.dot * ar.dot;
					leg_board_dot = ar.dot;
					cur = ar.end_state;
				}
				// The carve. Mid legs run the UNIFIED TRANSFER
				// primitive: the ride flows through its exit and the
				// flight, ending at the strike on the next face - the
				// genome shapes the whole transfer and is scored by
				// the strike itself (the design's stage-3 unit). The
				// last leg exits toward the zone with a lob target.
				Carve::Target ct;
				ct.face = fi;
				ct.max_ticks = 320;
				const float s_est = Len2D(cur.vel);
				const float est = s_est > 100.f
					? Len(next_t - cur.pos) / (s_est * p.dt)
					: 80.f;
				ct.aim_tick = est < 10.f ? 10
					: (est > 240.f ? 240 : static_cast<int>(est));
				ct.tick_w = 0.02f;
				Carve::Result cr;
				if (li + 1 < shape.size()) {
					const Route::Face& nf = g.faces[shape[li + 1]];
					ct.tap_brush = nf.brush;
					ct.tap_side = nf.side;
					// The strike region: the next face's corner
					// nearest THIS face lifted off the valley floor
					// (taps land low on the adjacent edge, not at
					// the centroid mid-ramp).
					Vec3 tap_at = ClosestVert(nf.verts, fc.centroid);
					if (tap_at.Z < nf.zmin + 40.f)
						tap_at.Z = nf.zmin + 40.f;
					ct.aim_pos = tap_at;
					ct.pos_w = 0.15f;
					ct.exit_heading = atan2f(
						next_t.Y - cur.pos.Y, next_t.X - cur.pos.X);
					cr = Carve::SolveCarve(cur, w, p, g, ct, 4,
						o.carve_evals);
				} else {
					const float dh_az = atan2f(fc.downhill.Y,
						fc.downhill.X);
					float bearing = atan2f(next_t.Y - cur.pos.Y,
						next_t.X - cur.pos.X);
					const float rel = Steer::WrapPi(bearing - dh_az);
					if (cosf(rel) > -0.05f)
						bearing = dh_az + (rel >= 0.f ? 1.f : -1.f)
							* (Steer::kSteerPi * 0.5f + 0.1f);
					ct.exit_heading = Steer::WrapPi(bearing);
					ct.aim_vel = Vec3(cosf(ct.exit_heading) * s_est,
						sinf(ct.exit_heading) * s_est, 140.f);
					ct.vel_w = 0.8f;
					cr = Carve::SolveCarve(cur, w, p, g, ct, 4,
						o.carve_evals);
				}
				auto tapped = [&](const Carve::Result& c) {
					return !c.exited && ct.tap_brush >= 0
						&& c.struck_brush == ct.tap_brush
						&& c.struck_plane == ct.tap_side;
				};
				if (!cr.exited && !tapped(cr)) {
					printf("assemble: leg %d CARVE FAILED on face "
						"%d (%s%d, miss %.0f)\n",
						static_cast<int>(li), fi,
						cr.grounded ? "grounded " : "struck ",
						cr.struck_brush, cr.miss_dist);
					return false;
				}
				AppendCarve(&rr.frames, cr, cur.ducked);
				cur = cr.end_state;
				if (tapped(cr)) {
					rr.board_loss2 += cr.strike_dot
						* cr.strike_dot;
					skip_air = true;
					printf("assemble: leg %d TRANSFER -> face %d "
						"dot %.1f at (%.0f,%.0f,%.0f v %.0f)\n",
						static_cast<int>(li), shape[li + 1],
						cr.strike_dot, cur.pos.X, cur.pos.Y,
						cur.pos.Z, Len2D(cur.vel));
				} else {
					printf("assemble: leg %d carve exit (%.0f,%.0f,"
						"%.0f) v(%.0f,%.0f,%.0f)\n",
						static_cast<int>(li), cur.pos.X, cur.pos.Y,
						cur.pos.Z, cur.vel.X, cur.vel.Y,
						cur.vel.Z);
				}
			}
			const int zk = FlyToZone(w, g, p, cur, &rr.frames);
			if (zk < 0) {
				printf("assemble: FLY-TO-ZONE FAILED from "
					"(%.0f,%.0f,%.0f v %.0f)\n", cur.pos.X,
					cur.pos.Y, cur.pos.Z, Len2D(cur.vel));
				return false;
			}
			rr.finished = true;
			rr.zone_tick = static_cast<int>(rr.frames.size()) - 1;
			*out = rr;
			return true;
		}

	} // namespace

	bool SolveMap(const World& w, const Route::Graph& g,
	              const MoveParams& p, const TapeAnchor& anchor,
	              const Opts& o, RunResult* out, std::string* err) {
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
		for (const std::vector<int>& shape : shapes) {
			const double spent = static_cast<double>(clock() - c0)
				/ CLOCKS_PER_SEC;
			if (spent > o.wall_budget_s)
				break;
			RunResult rr;
			if (!AssembleShape(w, g, p, anchor, shape, o, &rr))
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
		if (!any) {
			if (err) *err = "no shape assembled to the zone";
			return false;
		}
		*out = best;
		return true;
	}

} // namespace Assemble
} // namespace Solver
