#include "SolverCarve.h"

#include <float.h>
#include <math.h>

#include <algorithm>

#include "SolverBoard.h"
#include "SolverSearchLog.h"
#include "SolverSteer.h"

namespace Solver {
namespace Carve {

	namespace {

		using Steer::WrapPi;
		constexpr float kPi = Steer::kSteerPi;

		float SplineEval(const std::vector<float>& knots, int k,
		                 int horizon) {
			const int K = static_cast<int>(knots.size());
			if (K == 1 || horizon <= 0)
				return knots[0];
			float x = static_cast<float>(k) / static_cast<float>(horizon)
				* static_cast<float>(K - 1);
			if (x < 0.f) x = 0.f;
			if (x > static_cast<float>(K - 1))
				x = static_cast<float>(K - 1);
			const int i = static_cast<int>(x) < K - 2
				? static_cast<int>(x) : K - 2;
			const float f = x - static_cast<float>(i);
			return knots[i] + WrapPi(knots[i + 1] - knots[i]) * f;
		}

	} // namespace

	Result RideHeadingSpline(const PlayerState& entry, const World& w,
	                         const MoveParams& p, const Target& t,
	                         const Route::Graph& g,
	                         const std::vector<float>& knots,
	                         const std::vector<float>* effort,
	                         int horizon, int duck_at) {
		Result r;
		if (t.face < 0 || t.face >= static_cast<int>(g.faces.size()))
			return r;
		const Route::Face& face = g.faces[t.face];
		const Route::Face* tapf = (t.tap_face >= 0
			&& t.tap_face < static_cast<int>(g.faces.size()))
			? &g.faces[t.tap_face] : nullptr;
		PlayerState s = entry;
		const int hold = s.ducked ? IN_DUCK : 0;
		r.duck_at = duck_at;
		Steer::Controller ctl;
		const int sh = (t.aim_tick > 0 && t.aim_tick < horizon)
			? t.aim_tick : horizon;
		int air_streak = 0;
		bool have_exit = false;
		bool ever_exit = false;
		PlayerState exit_s;
		int exit_tick = 0;
		// Ballistic shortfall from a separation state to the target
		// (exact discrete z law, current-speed flight time - speed
		// gain only shortens it, so this under-promises, never over).
		// Targets: the tap face's verts, or in zone mode the NEAREST
		// point of the volume's bottom rim (xy-clamped - a corner set
		// misreads a wide zone as unreachable).
		auto BallShort = [&](const PlayerState& es, const Vec3& v)
			-> float {
			const float s2d = Len2D(es.vel);
			if (s2d < 1.f)
				return 1e6f;
			const float dx = v.X - es.pos.X;
			const float dy = v.Y - es.pos.Y;
			const float n = sqrtf(dx * dx + dy * dy) / (s2d * p.dt);
			const float z = es.pos.Z + n * p.dt * es.vel.Z
				- 0.5f * p.gravity * p.dt * p.dt * n * n;
			return v.Z - z > 0.f ? v.Z - z : 0.f;
		};
		auto ReachShort = [&](const PlayerState& es) -> float {
			if (t.to_zone) {
				Vec3 v(es.pos.X, es.pos.Y, t.zone_min.Z);
				if (v.X < t.zone_min.X) v.X = t.zone_min.X;
				if (v.X > t.zone_max.X) v.X = t.zone_max.X;
				if (v.Y < t.zone_min.Y) v.Y = t.zone_min.Y;
				if (v.Y > t.zone_max.Y) v.Y = t.zone_max.Y;
				return BallShort(es, v);
			}
			if (!tapf)
				return 0.f;
			float best = FLT_MAX;
			for (const Vec3& v : tapf->verts) {
				const float sh = BallShort(es, v);
				if (sh < best)
					best = sh;
			}
			return best;
		};
		float duty = 1.f;   // effort accumulator (first tick strafes)
		SearchLog::Sink* lg = SearchLog::g_sink;
		if (lg)
			lg->StartTraj(s.pos);
		r.yaw.reserve(horizon);
		r.fmove.reserve(horizon);
		r.smove.reserve(horizon);
		for (int k = 0; k < horizon; ++k) {
			const float theta = SplineEval(knots, k, sh);
			float yaw_deg = 0.f, fmove = 0.f, smove = 0.f;
			bool strafe_tick = true;
			if (effort) {
				float eff = SplineEval(*effort, k, sh);
				if (eff < 0.f) eff = 0.f;
				if (eff > 1.f) eff = 1.f;
				duty += eff;
				if (duty >= 1.f)
					duty -= 1.f;
				else
					strafe_tick = false;   // coast this tick
			}
			if (strafe_tick)
				ctl.Tick(s, p, theta, k, &yaw_deg, &fmove, &smove);
			const int btn = hold
				| (duck_at >= 0 && k >= duck_at ? IN_DUCK : 0);
			TickEvents ev;
			MoveTick(s, w, p, 0.f, yaw_deg, fmove, smove, 0.f, btn, &ev);
			if (lg)
				lg->Point(s.pos);
			r.yaw.push_back(yaw_deg);
			r.fmove.push_back(fmove);
			r.smove.push_back(smove);
			if (t.to_zone) {
				// Zone entry ends the transfer - the run is DONE at
				// this tick (the clock's own event).
				if (s.pos.X >= t.zone_min.X && s.pos.X <= t.zone_max.X
					&& s.pos.Y >= t.zone_min.Y
					&& s.pos.Y <= t.zone_max.Y
					&& s.pos.Z >= t.zone_min.Z
					&& s.pos.Z <= t.zone_max.Z) {
					r.zoned = true;
					r.tick = k + 1;
					r.end_state = s;
					r.end_pos = s.pos;
					r.flips = ctl.flips;
					if (lg)
						lg->EndTraj(SearchLog::kZoned);
					return r;
				}
				// No-arrival gradient: 3D distance to the volume.
				const float dx = s.pos.X < t.zone_min.X
					? t.zone_min.X - s.pos.X
					: (s.pos.X > t.zone_max.X
						? s.pos.X - t.zone_max.X : 0.f);
				const float dy = s.pos.Y < t.zone_min.Y
					? t.zone_min.Y - s.pos.Y
					: (s.pos.Y > t.zone_max.Y
						? s.pos.Y - t.zone_max.Y : 0.f);
				const float dz = s.pos.Z < t.zone_min.Z
					? t.zone_min.Z - s.pos.Z
					: (s.pos.Z > t.zone_max.Z
						? s.pos.Z - t.zone_max.Z : 0.f);
				const float da = sqrtf(dx * dx + dy * dy + dz * dz);
				if (da < r.miss_dist)
					r.miss_dist = da;
			} else if (tapf) {
				// Closest approach to the tap face REGION - the
				// no-strike gradient (air primitive's aim_region).
				// FRONT SIDE ONLY: a clip strikes from off > 0 with
				// v·n < 0; positions behind the plane (threading a
				// corridor past the face, diving under it) can never
				// convert to a strike and must not read as progress.
				const float off = Dot(tapf->n, s.pos) - tapf->d;
				if (off > 0.f) {
					// Aim the SAFE interior, not the marginal edge:
					// a hull-center at the raw polygon boundary is
					// still kHullCenterSlack away from a guaranteed
					// strike (M1.2), and 20-50u edge-skim misses
					// were exactly what the chains died of.
					const float eo = Board::EdgeDistOut(*tapf, s.pos)
						+ Board::kHullCenterSlack;
					const float da = sqrtf(off * off
						+ (eo > 0.f ? eo * eo : 0.f));
					if (da < r.miss_dist)
						r.miss_dist = da;
				}
			} else if (t.pos_w > 0.f) {
				const float da = Len(s.pos - t.aim_pos);
				if (da < r.miss_dist)
					r.miss_dist = da;
			}
			if (s.on_ground) {
				r.grounded = true;
				r.end_pos = s.pos;
				r.flips = ctl.flips;
				if (ever_exit)
					r.reach_short = ReachShort(exit_s);
				if (lg)
					lg->EndTraj(SearchLog::kGrounded);
				return r;
			}
			if (ev.ncontacts > 0) {
				// Riding while every contact is our face; anything else
				// ends the carve (that strike is the next transfer's
				// business).
				bool ours = true;
				for (int c = 0; c < ev.ncontacts; ++c)
					if (ev.contact_brush[c] != face.brush
						|| ev.contact_plane[c] != face.side)
						ours = false;
				if (!ours) {
					// In tap mode only the TAP face ends the
					// transfer; any other contact is a GRAZE - clip
					// and keep going (the air primitive's law: ending
					// on grazes killed chains short of their face;
					// the clip cost shows up in the arrival itself).
					// Zone mode grazes EVERYTHING - only the volume,
					// the ground, or the horizon end it.
					int tc = 0;
					if (t.to_zone || t.tap_brush >= 0) {
						tc = -1;
						if (!t.to_zone)
							for (int c = 0; c < ev.ncontacts; ++c)
								if (ev.contact_brush[c] == t.tap_brush
									&& ev.contact_plane[c]
										== t.tap_side)
									tc = c;
						if (tc < 0) {
							if (r.struck_brush < 0) {
								r.struck_brush = ev.contact_brush[0];
								r.struck_plane = ev.contact_plane[0];
							}
							air_streak = 0;
							have_exit = false;
							continue;
						}
					}
					r.struck_brush = ev.contact_brush[tc];
					r.struck_plane = ev.contact_plane[tc];
					if (r.struck_brush >= 0 && r.struck_plane >= 0) {
						const Vec3& sn = w.brushes[r.struck_brush]
							.n[r.struck_plane];
						r.strike_dot = Dot(ev.contact_vel[tc], sn);
					}
					r.tick = k + 1;
					r.end_state = s;   // post-strike = the next
					                   // leg's post-board entry
					r.end_pos = s.pos;
					r.flips = ctl.flips;
					// Rolling context: the climb this landing still
					// owes toward the LEG-AFTER target, priced by
					// the exact energy law v' = sqrt(v^2 - 2g*sh).
					float nsh = -1.f;
					if (t.next_face >= 0 && t.next_face
						< static_cast<int>(g.faces.size())) {
						nsh = FLT_MAX;
						for (const Vec3& v
							: g.faces[t.next_face].verts) {
							const float sh = BallShort(s, v);
							if (sh < nsh)
								nsh = sh;
						}
					} else if (t.next_is_zone) {
						Vec3 v(s.pos.X, s.pos.Y, t.zone_min.Z);
						if (v.X < t.zone_min.X) v.X = t.zone_min.X;
						if (v.X > t.zone_max.X) v.X = t.zone_max.X;
						if (v.Y < t.zone_min.Y) v.Y = t.zone_min.Y;
						if (v.Y > t.zone_max.Y) v.Y = t.zone_max.Y;
						nsh = BallShort(s, v);
					}
					if (nsh > 0.f && nsh < 1e6f) {
						const float v0 = Len2D(s.vel);
						const float rem = v0 * v0
							- 2.f * p.gravity * nsh;
						r.next_cost = rem > 0.f
							? v0 - sqrtf(rem) : v0;
					}
					if (lg)
						lg->EndTraj(t.tap_brush >= 0
							? SearchLog::kHit : SearchLog::kStruck);
					return r;
				}
				air_streak = 0;
				have_exit = false;
				r.ride_ticks++;
				for (int c = 0; c < ev.ncontacts; ++c) {
					const float d = Dot(ev.contact_vel[c], face.n);
					r.ride_loss2 += d * d;
				}
			} else {
				if (!have_exit) {
					exit_s = s;   // first airborne tick = the exit state
					exit_tick = k + 1;
					have_exit = true;
					ever_exit = true;
				}
				air_streak++;
				// With a TAP target the transfer is ONE primitive:
				// the ride continues through the clean-air exit into
				// the flight, ending at the strike on the tap face
				// (handled above as the !ours contact). Without one,
				// three clean ticks end the carve as before.
				if (air_streak >= 3 && t.tap_brush < 0
					&& !t.to_zone) {
					r.exited = true;
					r.tick = exit_tick;
					r.exit_pos = exit_s.pos;
					r.exit_vel = exit_s.vel;
					r.exit_heading = atan2f(exit_s.vel.Y,
						exit_s.vel.X);
					r.speed2d = Len2D(exit_s.vel);
					r.end_pos = s.pos;
					r.end_state = exit_s;
					r.flips = ctl.flips;
					if (lg)
						lg->EndTraj(SearchLog::kExited);
					return r;
				}
			}
		}
		r.end_pos = s.pos;
		r.flips = ctl.flips;
		if (ever_exit)
			r.reach_short = ReachShort(exit_s);
		if (lg)
			lg->EndTraj(SearchLog::kMiss);
		return r;
	}

	namespace {

		float Score(const Result& r, const Target& t) {
			if (t.to_zone) {
				// The last transfer: arrival tick IS the objective.
				// Any arrival beats any non-arrival.
				if (r.zoned)
					return -2000.f + static_cast<float>(r.tick);
				return 600.f
					+ (r.miss_dist < 1e8f ? 2.f * r.miss_dist : 1e6f)
					+ 2.f * r.reach_short;
			}
			if (!r.exited) {
				// A strike on the TAP face is the transfer itself:
				// scored by its own board loss (soft taps beat any
				// lob that still has a board ahead of it).
				if (t.tap_brush >= 0
					&& r.struck_brush == t.tap_brush
					&& r.struck_plane == t.tap_side)
					// Strike loss + the climb the landing owes the
					// next leg - both u/s, same weight (the whole
					// transfer pays, not just the touch).
					return 0.6f * (fabsf(r.strike_dot) + r.next_cost)
						- 0.01f * Len2D(r.end_state.vel);
				// A non-exit CLOSE to the aim must be able to
				// outscore an exit FAR from it, or the search can
				// never walk through the stall region bordering a
				// crest exit. Floor 600 keeps decent exits dominant.
				// reach_short prices separations the ballistic law
				// says can never arrive at the face's altitude.
				return 600.f
					+ (r.miss_dist < 1e8f ? 2.f * r.miss_dist : 1e6f)
					+ 2.f * r.reach_short;
			}
			float j = t.vel_w > 0.f
				? t.vel_w * Len(r.exit_vel - t.aim_vel)
				: 300.f * fabsf(WrapPi(r.exit_heading
					- t.exit_heading)) - 0.02f * r.speed2d;
			if (t.aim_tick >= 0) {
				const int ad = r.tick > t.aim_tick
					? r.tick - t.aim_tick : t.aim_tick - r.tick;
				if (t.tick_tol >= 0 && ad > t.tick_tol)
					// Keep a gradient on the plateau: tick overshoot
					// AND exit-position error both point back toward
					// the window.
					j += 1e4f + static_cast<float>(ad - t.tick_tol)
						+ (t.pos_w > 0.f
							? 0.1f * Len(r.exit_pos - t.aim_pos)
							: 0.f);
				else
					j += t.tick_w * static_cast<float>(ad);
			}
			if (t.pos_w > 0.f)
				j += t.pos_w * Len(r.exit_pos - t.aim_pos);
			return j;
		}

	} // namespace

	Result SolveCarve(const PlayerState& entry, const World& w,
	                  const MoveParams& p, const Route::Graph& g,
	                  const Target& t_in, int knots_n, int evals,
	                  std::vector<Result>* alts, int alts_k) {
		// Local mutable copy: the solve runs DUAL SPLINE DOMAINS
		// (given aim_tick and its half) because the straight-line-at-
		// entry-speed estimate overshoots when the ride doubles the
		// speed - late knots go dead exactly like the M1.3 air case,
		// and the up-turn of an S-carve lands inside a fraction of
		// the first segment. Polish runs in the winning domain.
		Target t = t_in;
		Result best;
		if (t.face < 0 || t.face >= static_cast<int>(g.faces.size()))
			return best;
		const Route::Face& face = g.faces[t.face];
		const float s2d0 = Len2D(entry.vel);
		const float h0 = s2d0 > 1.f
			? atan2f(entry.vel.Y, entry.vel.X) : t.exit_heading;
		const float dh_az = atan2f(face.downhill.Y, face.downhill.X);

		if (knots_n < 2) knots_n = 2;
		// Genome = heading knots + effort knots (the expert's two carve
		// dials: where to point, how hard to work). two_phase_b > 0
		// rides TOWARD the aim position until fraction b of the ride,
		// then turns to the exit heading - the natural carve structure
		// (position first, heading last) for curvy rides.
		const float az_aim = t.tap_face >= 0
			&& t.tap_face < static_cast<int>(g.faces.size())
			? atan2f(g.faces[t.tap_face].centroid.Y - entry.pos.Y,
				g.faces[t.tap_face].centroid.X - entry.pos.X)
			: (t.pos_w > 0.f
				? atan2f(t.aim_pos.Y - entry.pos.Y,
					t.aim_pos.X - entry.pos.X)
				: t.exit_heading);
		struct Fam {
			float end; float pow; float bulge; float eff; float b;
		};
		const float up_az = WrapPi(dh_az + kPi);   // up-slope azimuth
		// The S-carve exit: the aim direction rotated halfway toward
		// up-slope - dive for speed, then carve up and separate
		// ascending (the tape transfers' measured shape).
		const float s_az = WrapPi(az_aim
			+ 0.5f * WrapPi(up_az - az_aim));
		struct FamD { Fam f; int duck; };
		const FamD fams[11] = {
			{ { h0, 1.f, 0.f, 1.f, 0.f }, -1 },       // hold, full effort
			{ { t.exit_heading, 1.f, 0.f, 1.f, 0.f }, -1 },  // linear
			{ { t.exit_heading, 2.f, 0.f, 1.f, 0.f }, -1 },  // late turn
			{ { t.exit_heading, 1.f, 0.5f, 1.f, 0.f }, -1 }, // bulge
			{ { t.exit_heading, 1.f, 0.f, 0.35f, 0.f }, -1 },// slow
			{ { t.exit_heading, 1.f, 0.f, 1.f, 0.7f }, -1 }, // via aim
			{ { t.exit_heading, 1.f, 0.f, 0.3f, 0.7f }, -1 },// slow via
			{ { up_az, 1.f, 0.f, 0.6f, 0.f }, -1 },   // climb (crest)
			// The DUCK-OFF family: slow pursuit of the aim, pop off at
			// the end (the solved12 crest exit pattern).
			{ { t.exit_heading, 1.f, 0.f, 0.4f, 0.7f }, 0 },
			// S-carves: bulge deep toward downhill (the dive), end
			// with an up-slope component (the ascending separation).
			{ { s_az, 1.f, 0.7f, 1.f, 0.f }, -1 },
			{ { s_az, 2.f, 0.5f, 1.f, 0.f }, -1 },    // later up-turn
		};
		int used = 0;
		// Tap transfers: the ride doubles the speed, so the estimate
		// runs LONG - pair it with its half. Zone endings: ride+arc
		// runs LONGER than the straight line - pair with 1.7x.
		const int dom_full = t.aim_tick;
		int dom_alt = t.to_zone
			? static_cast<int>(dom_full * 1.7f)
			: (dom_full > 20 ? dom_full / 2 : dom_full);
		if (dom_alt > t.max_ticks)
			dom_alt = t.max_ticks;
		const int dom_half = dom_alt;
		const int ndom = dom_half != dom_full ? 2 : 1;
		const int per_fam = evals / (12 * ndom) > 8
			? evals / (12 * ndom) : 8;
		float best_score = FLT_MAX;
		std::vector<float> best_th, best_ef;
		int best_duck = -1;
		int best_dom = dom_full;
		Result best_r;
		unsigned rng = 1409u;
		auto frand_pm = [&rng]() {
			rng = rng * 1664525u + 1013904223u;
			return static_cast<float>((rng >> 8) & 0xFFFFu)
				/ 32767.5f - 1.f;
		};
		auto Clamp01 = [](float v) {
			return v < 0.f ? 0.f : (v > 1.f ? 1.f : v);
		};
		// Diverse-success pool for the caller's downstream
		// composition: best result per END-POSITION cluster (48u).
		struct PR { Result r; float sc; };
		std::vector<PR> pool;
		auto EvalOne = [&](const std::vector<float>& th,
			const std::vector<float>& ef, int duck, float* sc_out)
			-> Result {
			Result rr = RideHeadingSpline(entry, w, p, t, g, th, &ef,
				t.max_ticks, duck);
			used++;
			const float sc = Score(rr, t);
			if (SearchLog::g_sink)
				SearchLog::g_sink->Score(sc);
			if (sc < best_score) {
				best_score = sc;
				best_th = th;
				best_ef = ef;
				best_duck = duck;
				best_dom = t.aim_tick;
				best_r = rr;
				if (SearchLog::g_sink)
					SearchLog::g_sink->MarkBest();
			}
			if (alts) {
				const bool poolable = t.to_zone ? rr.zoned
					: (t.tap_brush >= 0
						? (!rr.exited
							&& rr.struck_brush == t.tap_brush
							&& rr.struck_plane == t.tap_side)
						: rr.exited);
				if (poolable) {
					const Vec3& key = rr.exited ? rr.exit_pos
						: rr.end_pos;
					bool merged = false;
					for (PR& pr : pool) {
						const Vec3& pk = pr.r.exited ? pr.r.exit_pos
							: pr.r.end_pos;
						if (Len(pk - key) < 48.f) {
							if (sc < pr.sc) {
								pr.r = rr;
								pr.sc = sc;
							}
							merged = true;
							break;
						}
					}
					if (!merged)
						pool.push_back({ rr, sc });
				}
			}
			if (sc_out) *sc_out = sc;
			return rr;
		};
		// Coordinate descent over heading knots, effort knots (0.4x
		// step, clamped [0,1]) and the duck-off tick (integer moves).
		auto Descend = [&](std::vector<float>& th,
			std::vector<float>& ef, int& duck, float& kn_score,
			float delta, int budget) -> int {
			int spent = 0;
			while (spent < budget && used < evals && delta > 0.02f) {
				bool improved = false;
				for (int i = 0; i < 2 * knots_n && spent < budget
					&& used < evals; ++i) {
					for (int sgn = -1; sgn <= 1 && spent < budget
						&& used < evals; sgn += 2) {
						std::vector<float> tth = th, tef = ef;
						if (i < knots_n)
							tth[i] = WrapPi(tth[i]
								+ delta * static_cast<float>(sgn));
						else
							tef[i - knots_n] = Clamp01(
								tef[i - knots_n] + 0.4f * delta
									* static_cast<float>(sgn));
						float sc;
						EvalOne(tth, tef, duck, &sc);
						spent++;
						if (sc < kn_score) {
							th = tth;
							ef = tef;
							kn_score = sc;
							improved = true;
						}
					}
				}
				if (t.try_duck) {
					const int moves[4] = { -3, -1, 1, 3 };
					for (int m = 0; m < 4 && spent < budget
						&& used < evals; ++m) {
						int td = duck < 0
							? (moves[m] > 0
								? (t.aim_tick > 0 ? t.aim_tick - 1
									: t.max_ticks / 2) : -1)
							: duck + moves[m];
						if (td < -1) td = -1;
						if (td >= t.max_ticks) td = t.max_ticks - 1;
						if (td == duck)
							continue;
						float sc;
						EvalOne(th, ef, td, &sc);
						spent++;
						if (sc < kn_score) {
							duck = td;
							kn_score = sc;
							improved = true;
						}
					}
				}
				if (!improved)
					delta *= 0.5f;
			}
			return spent;
		};
		for (int di = 0; di < ndom; ++di) {
		t.aim_tick = di == 0 ? dom_full : dom_half;
		const int duck_guess = t.aim_tick > 0 ? t.aim_tick - 1
			: t.max_ticks / 2;
		for (int fam = 0; fam < 11 && used < evals; ++fam) {
			std::vector<float> th(knots_n), ef(knots_n,
				fams[fam].f.eff);
			int duck = fams[fam].duck == 0 ? duck_guess : -1;
			for (int i = 0; i < knots_n; ++i) {
				float f = static_cast<float>(i)
					/ static_cast<float>(knots_n - 1);
				if (fams[fam].f.b > 0.f) {
					// Two-phase: pursue the aim position, then turn
					// out to the exit heading.
					if (f <= fams[fam].f.b)
						th[i] = az_aim;
					else
						th[i] = az_aim
							+ WrapPi(fams[fam].f.end - az_aim)
							* (f - fams[fam].f.b)
							/ (1.f - fams[fam].f.b);
					continue;
				}
				const float bf = sinf(kPi * f) * fams[fam].f.bulge;
				f = powf(f, fams[fam].f.pow);
				th[i] = h0 + WrapPi(fams[fam].f.end - h0) * f;
				if (bf > 0.f)
					th[i] = th[i] + WrapPi(dh_az - th[i]) * bf;
			}
			float kscore;
			EvalOne(th, ef, duck, &kscore);
			int spent = 1;
			spent += Descend(th, ef, duck, kscore, 0.5f,
				per_fam - spent);
			while (spent + 1 < per_fam && used < evals) {
				std::vector<float> hth = th, hef = ef;
				int hduck = duck;
				for (int i = 0; i < knots_n; ++i) {
					hth[i] = WrapPi(hth[i] + 0.3f * frand_pm());
					hef[i] = Clamp01(hef[i] + 0.2f * frand_pm());
				}
				float hscore;
				EvalOne(hth, hef, hduck, &hscore);
				spent++;
				spent += Descend(hth, hef, hduck, hscore, 0.25f,
					per_fam - spent);
				if (hscore < kscore) {
					th = hth;
					ef = hef;
					duck = hduck;
					kscore = hscore;
				}
			}
		}
		}
		t.aim_tick = best_dom;   // polish in the winning domain
		while (used < evals && !best_th.empty()) {
			std::vector<float> hth = best_th, hef = best_ef;
			int hduck = best_duck;
			for (int i = 0; i < knots_n; ++i) {
				hth[i] = WrapPi(hth[i] + 0.1f * frand_pm());
				hef[i] = Clamp01(hef[i] + 0.08f * frand_pm());
			}
			float hscore;
			EvalOne(hth, hef, hduck, &hscore);
			Descend(hth, hef, hduck, hscore, 0.12f, evals - used);
		}
		best = best_r;
		if (alts) {
			std::sort(pool.begin(), pool.end(),
				[](const PR& a, const PR& b) { return a.sc < b.sc; });
			alts->clear();
			for (const PR& pr : pool) {
				if (static_cast<int>(alts->size()) >= alts_k)
					break;
				alts->push_back(pr.r);
			}
		}
		return best;
	}

} // namespace Carve
} // namespace Solver
