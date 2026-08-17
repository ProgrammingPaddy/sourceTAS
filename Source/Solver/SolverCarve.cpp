#include "SolverCarve.h"

#include <float.h>
#include <math.h>

#include <algorithm>

#include "SolverBoard.h"
#include "SolverField.h"
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

	std::vector<ExitRec>* g_exit_rec = nullptr;
	bool g_doom_cull = true;

	bool ExitDoomed(const Vec3& pos, const Vec3& vel,
	                const MoveParams& p, const Route::Face* tapf,
	                bool to_zone, const Vec3& zmin, const Vec3& zmax) {
		if (to_zone)
			return Field::ZoneReach(pos, Len2D(vel), vel.Z, zmin,
				zmax, p, 300, nullptr) < 0.f;
		if (!tapf)
			return false;
		// 2D distance to the face: min over edges (point-segment),
		// zero inside the xy bounding box (optimistic containment
		// proxy - over-covering keeps the test admissible).
		float zlo = FLT_MAX, zhi = -FLT_MAX;
		float xlo = FLT_MAX, xhi = -FLT_MAX;
		float ylo = FLT_MAX, yhi = -FLT_MAX;
		for (const Vec3& v : tapf->verts) {
			if (v.Z < zlo) zlo = v.Z;
			if (v.Z > zhi) zhi = v.Z;
			if (v.X < xlo) xlo = v.X;
			if (v.X > xhi) xhi = v.X;
			if (v.Y < ylo) ylo = v.Y;
			if (v.Y > yhi) yhi = v.Y;
		}
		float dmin = 0.f;
		if (pos.X < xlo || pos.X > xhi || pos.Y < ylo
			|| pos.Y > yhi) {
			dmin = FLT_MAX;
			const size_t nv = tapf->verts.size();
			for (size_t i = 0; i < nv; ++i) {
				const Vec3& a = tapf->verts[i];
				const Vec3& b = tapf->verts[(i + 1) % nv];
				const float ex = b.X - a.X, ey = b.Y - a.Y;
				const float l2 = ex * ex + ey * ey;
				float t2 = l2 > 1e-6f
					? ((pos.X - a.X) * ex + (pos.Y - a.Y) * ey) / l2
					: 0.f;
				if (t2 < 0.f) t2 = 0.f;
				if (t2 > 1.f) t2 = 1.f;
				const float px = a.X + ex * t2 - pos.X;
				const float py = a.Y + ey * t2 - pos.Y;
				const float d = sqrtf(px * px + py * py);
				if (d < dmin)
					dmin = d;
			}
		}
		return !Envelope::CanReach(pos, vel, dmin, zlo, zhi, p, 300);
	}

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
		// Engine-truth exit record: the FIRST separation after real
		// ride ticks + the candidate's eventual fate (exitbench data).
		bool have_first = false;
		PlayerState first_exit;
		int first_tick = 0;
		auto RecExit = [&](int oc, int end_tick) {
			if (!g_exit_rec || !have_first)
				return;
			ExitRec er;
			er.xpos = first_exit.pos;
			er.xvel = first_exit.vel;
			er.xtick = first_tick;
			er.outcome = oc;
			er.e_end = Dot(s.vel, s.vel)
				+ 2.f * p.gravity * s.pos.Z;
			er.graze2 = r.graze_loss2;
			er.end_tick = end_tick;
			g_exit_rec->push_back(er);
		};
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
			lg->StartTraj(s.pos, Len(s.vel));
		r.yaw.reserve(horizon);
		r.fmove.reserve(horizon);
		r.smove.reserve(horizon);
		// Terminal tangent tracking for the tap flight (testimony 2.1
		// as construction - same law as the air primitive).
		const float t_hn = tapf ? sqrtf(tapf->n.X * tapf->n.X
			+ tapf->n.Y * tapf->n.Y) : 0.f;
		const float t_psi = tapf ? atan2f(tapf->n.Y, tapf->n.X) : 0.f;
		// LATCHED terminal tracking (see SolverAir - a naive
		// engagement condition chatters as turning reduces closing).
		bool term_latch = false;
		float term_phi = 0.f;
		for (int k = 0; k < horizon; ++k) {
			float theta = SplineEval(knots, k, sh);
			if (t.terminal_tangent && tapf && have_exit
				&& t_hn > 1e-4f) {
				const float off = Dot(tapf->n, s.pos) - tapf->d;
				const float closing = -Dot(tapf->n, s.vel);
				if (off > 0.f && closing > 1.f) {
					const float n_hit = off / (closing * p.dt);
					if (n_hit < 90.f) {
						const float s2dn = Len2D(s.vel);
						const float vz_hit = s.vel.Z
							- p.gravity * s.gravity_scale * p.dt
							* n_hit;
						float cphi = s2dn * t_hn > 1e-4f
							? -vz_hit * tapf->n.Z / (s2dn * t_hn)
							: 0.f;
						if (cphi > 1.f) cphi = 1.f;
						if (cphi < -1.f) cphi = -1.f;
						const float po = acosf(cphi);
						const float hcur = s2dn > 1.f
							? atan2f(s.vel.Y, s.vel.X) : theta;
						const float ca = WrapPi(t_psi + po);
						const float cb = WrapPi(t_psi - po);
						const float phi_t =
							fabsf(WrapPi(ca - hcur))
								<= fabsf(WrapPi(cb - hcur))
							? ca : cb;
						const float need =
							fabsf(WrapPi(phi_t - hcur));
						Strafe::TickLaw tl = Strafe::Law(p, s2dn,
							1.f, s.ducked);
						const float rate = tl.TurnRad(0.f, 1.f);
						if (!term_latch && rate > 1e-5f
							&& n_hit <= need / rate + 1.f)
							term_latch = true;
						if (term_latch)
							term_phi = phi_t;
					}
				}
			} else if (!have_exit) {
				term_latch = false;   // back on the face: reset
			}
			if (term_latch && have_exit)
				theta = term_phi;
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
				lg->Point(s.pos, Len(s.vel));
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
					RecExit(SearchLog::kZoned, k + 1);
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
				RecExit(SearchLog::kGrounded, k + 1);
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
							// Grazes dissipate dot^2 like any clip -
							// account them (the energy ledger's
							// -712k crease-scrape was invisible).
							for (int c = 0; c < ev.ncontacts; ++c) {
								if (ev.contact_brush[c] < 0
									|| ev.contact_plane[c] < 0)
									continue;
								const Vec3& gn =
									w.brushes[ev.contact_brush[c]]
									.n[ev.contact_plane[c]];
								const float gd = Dot(
									ev.contact_vel[c], gn);
								r.graze_loss2 += gd * gd;
							}
							air_streak = 0;
							have_exit = false;
							continue;
						}
						// RIDEABILITY: a tap that lands on the
						// exiting branch (off the polygon within
						// the 3-tick catch window) is a GRAZE,
						// not a board - the next leg cannot ride
						// it (measured: -4.7 tangent touches at
						// the bottom lip bouncing straight off).
						// Do NOT record it as the struck face -
						// that poisons the caller's tap check.
						if (tapf) {
							const Vec3 proj = s.pos
								+ Scale(s.vel, 3.f * p.dt);
							if (Board::EdgeDistOut(*tapf, proj)
								> 0.f) {
								const Vec3& gn =
									w.brushes[t.tap_brush]
									.n[t.tap_side];
								const float gd = Dot(
									ev.contact_vel[tc], gn);
								r.graze_loss2 += gd * gd;
								air_streak = 0;
								have_exit = false;
								continue;
							}
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
					RecExit(t.tap_brush >= 0
						? SearchLog::kHit : SearchLog::kStruck,
						k + 1);
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
					if (!have_first && r.ride_ticks > 0) {
						first_exit = s;
						first_tick = k + 1;
						have_first = true;
					}
					// THE DOOM CULL: a separation that provably
					// cannot reach the target ends the candidate
					// NOW (exitbench: 99%+ of dead candidates, ~0
					// false kills). Re-tested after every graze
					// (the state changed). One-shot region distance
					// keeps the search gradient for dead branches.
					if (g_doom_cull && r.ride_ticks > 0
						&& (t.to_zone || tapf)
						&& ExitDoomed(s.pos, s.vel, p, tapf,
							t.to_zone, t.zone_min, t.zone_max)) {
						r.doomed = true;
						if (tapf) {
							const float off = Dot(tapf->n, s.pos)
								- tapf->d;
							if (off > 0.f) {
								const float eo = Board::EdgeDistOut(
									*tapf, s.pos)
									+ Board::kHullCenterSlack;
								r.miss_dist = sqrtf(off * off
									+ (eo > 0.f ? eo * eo : 0.f));
							}
						} else {
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
							r.miss_dist = sqrtf(dx * dx + dy * dy
								+ dz * dz);
						}
						r.reach_short = ReachShort(exit_s);
						r.end_pos = s.pos;
						r.flips = ctl.flips;
						RecExit(SearchLog::kDoomed, k + 1);
						if (lg)
							lg->EndTraj(SearchLog::kDoomed);
						return r;
					}
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
		RecExit(SearchLog::kMiss, horizon);
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
				if (t.tap_brush >= 0 && r.tick > 0
					&& r.struck_brush == t.tap_brush
					&& r.struck_plane == t.tap_side)
					// Strike softness + carry (the energy-currency
					// form was retried WITH the rideability law
					// 2026-08-17 and STILL regressed to hard taps -
					// the score shapes the search trajectory, not
					// just the pick; this weighting produced the
					// best measured transfers: -7.5/-54 rideable
					// landings).
					return 0.6f * fabsf(r.strike_dot)
						- 0.01f * (Len2D(r.end_state.vel)
							- r.next_cost);
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
		// Tap-mode pursuit aims. az_aim = the face centroid (neutral).
		// az_obj = the centroid slid along the face's LATERAL axis
		// toward the momentum-ahead vert nearest the leg-after target
		// (user testimony: boarding chooses how the space left on the
		// ramp serves the next objective; land AHEAD of your motion).
		// az_obj is a FAMILY variant, not the default - forcing it as
		// the only aim measurably biased every solve to one side; as
		// one basin among many, carry/tangency scoring picks the side
		// per situation.
		float az_aim = t.exit_heading;
		float az_obj = t.exit_heading;
		if (t.tap_face >= 0
			&& t.tap_face < static_cast<int>(g.faces.size())) {
			const Route::Face& tf = g.faces[t.tap_face];
			Vec3 aim_pt = t.have_field_aim ? t.field_aim
				: tf.centroid;
			Vec3 next_pt;
			bool have_next = false;
			if (t.next_face >= 0
				&& t.next_face < static_cast<int>(g.faces.size())) {
				next_pt = g.faces[t.next_face].centroid;
				have_next = true;
			} else if (t.next_is_zone) {
				// A zone's honest aim point is its NEAREST RIM from
				// the tap face - the AABB center of a sprawling
				// platform drags aims toward its far overhang.
				next_pt = tf.centroid;
				if (next_pt.X < t.zone_min.X) next_pt.X = t.zone_min.X;
				if (next_pt.X > t.zone_max.X) next_pt.X = t.zone_max.X;
				if (next_pt.Y < t.zone_min.Y) next_pt.Y = t.zone_min.Y;
				if (next_pt.Y > t.zone_max.Y) next_pt.Y = t.zone_max.Y;
				next_pt.Z = t.zone_min.Z;
				have_next = true;
			}
			if (have_next) {
				// "Land AHEAD of your motion, serving the
				// objective" (user 2026-08-16: they all turn RIGHT
				// into the last ramp instead of LEFT - more space
				// to launch off of means a smoother landing and
				// better conversion). Only verts DOWN-STREAM of the
				// entry heading qualify - a landing you turn back
				// for is a slam; among those, nearest the leg-after
				// target. Falls back to all verts if none is ahead.
				const float hx = cosf(h0), hy = sinf(h0);
				Vec3 best_v = tf.centroid;
				float bd = FLT_MAX;
				bool found = false;
				for (int pass = 0; pass < 2 && !found; ++pass) {
					for (const Vec3& v : tf.verts) {
						if (pass == 0) {
							const float ax = v.X - entry.pos.X;
							const float ay = v.Y - entry.pos.Y;
							if (ax * hx + ay * hy <= 0.f)
								continue;
						}
						const float dx = v.X - next_pt.X;
						const float dy = v.Y - next_pt.Y;
						const float d = dx * dx + dy * dy;
						if (d < bd) {
							bd = d;
							best_v = v;
							found = true;
						}
					}
				}
				// Slide the objective aim along the face's LATERAL
				// (contour) axis only - the side choice is one-
				// dimensional, and blending toward a vert in full
				// 3D drags the pursuit beyond the strike plane
				// (measured: 1->2 taps overshot and grounded on
				// the brush top).
				const float nh = sqrtf(tf.n.X * tf.n.X
					+ tf.n.Y * tf.n.Y);
				Vec3 obj_pt = tf.centroid;
				if (nh > 1e-4f) {
					const float lax = -tf.n.Y / nh;
					const float lay = tf.n.X / nh;
					const float lat =
						(best_v.X - tf.centroid.X) * lax
						+ (best_v.Y - tf.centroid.Y) * lay;
					obj_pt.X += lax * 0.5f * lat;
					obj_pt.Y += lay * 0.5f * lat;
				}
				az_obj = atan2f(obj_pt.Y - entry.pos.Y,
					obj_pt.X - entry.pos.X);
			}
			az_aim = atan2f(aim_pt.Y - entry.pos.Y,
				aim_pt.X - entry.pos.X);
			if (!have_next)
				az_obj = az_aim;
		} else if (t.pos_w > 0.f) {
			az_aim = atan2f(t.aim_pos.Y - entry.pos.Y,
				t.aim_pos.X - entry.pos.X);
		}
		struct Fam {
			float end; float pow; float bulge; float eff; float b;
			int via;   // two-phase pursuit: 0 = the aim point,
			           // 1 = the runway (face's far end),
			           // 2 = the objective-side point
		};
		const float up_az = WrapPi(dh_az + kPi);   // up-slope azimuth
		// The S-carve exit: the aim direction rotated halfway toward
		// up-slope - dive for speed, then carve up and separate
		// ascending (the tape transfers' measured shape).
		const float s_az = WrapPi(az_aim
			+ 0.5f * WrapPi(up_az - az_aim));
		// TANGENT ARRIVAL at the tap face - testimony 2.1, the
		// original boarding law: "velocity tangent to the ramp face
		// normal IN THE APPROACH DIRECTION ... closer to parallel
		// with the ramp." The flat-arrival tangent headings are the
		// two face-parallel directions (psi +/- 90, the M1.2 dot line
		// at vz->0); pick the one that CONTINUES the approach motion
		// - the same geometric init the air primitive already uses.
		float az_tan = az_aim;
		if (t.tap_face >= 0
			&& t.tap_face < static_cast<int>(g.faces.size())) {
			const Route::Face& tf2 = g.faces[t.tap_face];
			const float tpsi = atan2f(tf2.n.Y, tf2.n.X);
			const float ta = WrapPi(tpsi + kPi * 0.5f);
			const float tb = WrapPi(tpsi - kPi * 0.5f);
			az_tan = fabsf(WrapPi(ta - h0)) <= fabsf(WrapPi(tb - h0))
				? ta : tb;
		}
		// ZONE endings: "use the space available on the ramp" (user
		// testimony) - the runway azimuth points at the ride face's
		// FARTHEST vert, so a family can ride the whole face gaining
		// wish work (the measured human ending banks ~54k of wish
		// work over its 60-tick climb - the energy the short direct
		// crest launch lacks) before turning out.
		float az_far = t.exit_heading;
		if (t.to_zone) {
			// The runway is DOWN-STREAM of the motion too: the far
			// end of the face ahead of the entry heading (riding
			// back against the approach is not a runway).
			const float hx0 = cosf(h0), hy0 = sinf(h0);
			float bd2 = -1.f;
			for (int pass = 0; pass < 2 && bd2 < 0.f; ++pass) {
				for (const Vec3& v : face.verts) {
					const float dx = v.X - entry.pos.X;
					const float dy = v.Y - entry.pos.Y;
					if (pass == 0 && dx * hx0 + dy * hy0 <= 0.f)
						continue;
					const float d = dx * dx + dy * dy;
					if (d > bd2) {
						bd2 = d;
						az_far = atan2f(dy, dx);
					}
				}
			}
		}
		struct FamD { Fam f; int duck; };
		const FamD fams[17] = {
			{ { h0, 1.f, 0.f, 1.f, 0.f, 0 }, -1 },    // hold, full effort
			{ { t.exit_heading, 1.f, 0.f, 1.f, 0.f, 0 }, -1 },  // linear
			{ { t.exit_heading, 2.f, 0.f, 1.f, 0.f, 0 }, -1 },  // late turn
			{ { t.exit_heading, 1.f, 0.5f, 1.f, 0.f, 0 }, -1 }, // bulge
			{ { t.exit_heading, 1.f, 0.f, 0.35f, 0.f, 0 }, -1 },// slow
			{ { t.exit_heading, 1.f, 0.f, 1.f, 0.7f, 0 }, -1 }, // via aim
			{ { t.exit_heading, 1.f, 0.f, 0.3f, 0.7f, 0 }, -1 },// slow via
			{ { up_az, 1.f, 0.f, 0.6f, 0.f, 0 }, -1 },// climb (crest)
			// The DUCK-OFF family: slow pursuit of the aim, pop off at
			// the end (the solved12 crest exit pattern).
			{ { t.exit_heading, 1.f, 0.f, 0.4f, 0.7f, 0 }, 0 },
			// S-carves: bulge deep toward downhill (the dive), end
			// with an up-slope component (the ascending separation).
			{ { s_az, 1.f, 0.7f, 1.f, 0.f, 0 }, -1 },
			{ { s_az, 2.f, 0.5f, 1.f, 0.f, 0 }, -1 }, // later up-turn
			// RUNWAY families (zone endings): ride the face's length
			// gaining wish work, then turn out to the exit.
			{ { t.exit_heading, 1.f, 0.f, 1.f, 0.6f, 1 }, -1 },
			{ { s_az, 1.f, 0.f, 1.f, 0.6f, 1 }, -1 },
			// OBJECTIVE-SIDE families: land on the side of the face
			// that serves the leg-after target (momentum-ahead).
			{ { az_obj, 1.f, 0.f, 1.f, 0.f, 0 }, -1 },
			{ { az_obj, 1.f, 0.f, 1.f, 0.6f, 2 }, -1 },
			// TANGENT-ARRIVAL families (testimony 2.1): end heading
			// = face-parallel continuing the approach, so the strike
			// lands along the plane instead of banking into the
			// normal - straight in, and via the objective side.
			{ { az_tan, 1.f, 0.f, 1.f, 0.f, 0 }, -1 },
			{ { az_tan, 1.f, 0.f, 1.f, 0.6f, 2 }, -1 },
		};
		int used = 0;
		int cur_fam = -1;   // family provenance for win-rate data
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
		const int per_fam = evals / (18 * ndom) > 8
			? evals / (18 * ndom) : 8;
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
				best_r.family = cur_fam;
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
		for (int fam = 0; fam < 17 && used < evals; ++fam) {
			cur_fam = fam;
			std::vector<float> th(knots_n), ef(knots_n,
				fams[fam].f.eff);
			int duck = fams[fam].duck == 0 ? duck_guess : -1;
			const float via_az = fams[fam].f.via == 1 ? az_far
				: (fams[fam].f.via == 2 ? az_obj : az_aim);
			for (int i = 0; i < knots_n; ++i) {
				float f = static_cast<float>(i)
					/ static_cast<float>(knots_n - 1);
				if (fams[fam].f.b > 0.f) {
					// Two-phase: pursue the via point (aim or
					// runway), then turn out to the exit heading.
					if (f <= fams[fam].f.b)
						th[i] = via_az;
					else
						th[i] = via_az
							+ WrapPi(fams[fam].f.end - via_az)
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
		cur_fam = best_r.family; // polish keeps the winner's lineage
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
