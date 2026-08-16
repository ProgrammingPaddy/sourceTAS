#include "SolverCarve.h"

#include <float.h>
#include <math.h>

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
		PlayerState s = entry;
		const int hold = s.ducked ? IN_DUCK : 0;
		r.duck_at = duck_at;
		Steer::Controller ctl;
		const int sh = (t.aim_tick > 0 && t.aim_tick < horizon)
			? t.aim_tick : horizon;
		int air_streak = 0;
		bool have_exit = false;
		PlayerState exit_s;
		int exit_tick = 0;
		float duty = 1.f;   // effort accumulator (first tick strafes)
		r.yaw.reserve(horizon);
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
			r.yaw.push_back(yaw_deg);
			r.smove.push_back(smove);
			if (t.pos_w > 0.f) {
				const float da = Len(s.pos - t.aim_pos);
				if (da < r.miss_dist)
					r.miss_dist = da;
			}
			if (s.on_ground) {
				r.grounded = true;
				r.end_pos = s.pos;
				r.flips = ctl.flips;
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
					r.struck_brush = ev.contact_brush[0];
					r.end_pos = s.pos;
					r.flips = ctl.flips;
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
				}
				air_streak++;
				if (air_streak >= 3) {
					r.exited = true;
					r.tick = exit_tick;
					r.exit_pos = exit_s.pos;
					r.exit_vel = exit_s.vel;
					r.exit_heading = atan2f(exit_s.vel.Y,
						exit_s.vel.X);
					r.speed2d = Len2D(exit_s.vel);
					r.end_pos = s.pos;
					r.flips = ctl.flips;
					return r;
				}
			}
		}
		r.end_pos = s.pos;
		r.flips = ctl.flips;
		return r;
	}

	namespace {

		float Score(const Result& r, const Target& t) {
			if (!r.exited)
				// A non-exit CLOSE to the aim must be able to outscore
				// an exit FAR from it, or the search can never walk
				// through the stall region that borders a crest exit
				// (a hard 1e6 wall froze [7]/[8] across three fix
				// rounds). Floor 600 keeps any decent exit dominant.
				return 600.f
					+ (r.miss_dist < 1e8f ? 2.f * r.miss_dist : 1e6f);
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
	                  const Target& t, int knots_n, int evals) {
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
		const float az_aim = t.pos_w > 0.f
			? atan2f(t.aim_pos.Y - entry.pos.Y,
				t.aim_pos.X - entry.pos.X)
			: t.exit_heading;
		struct Fam {
			float end; float pow; float bulge; float eff; float b;
		};
		const float up_az = WrapPi(dh_az + kPi);   // up-slope azimuth
		struct FamD { Fam f; int duck; };
		const int duck_guess = t.aim_tick > 0 ? t.aim_tick - 1
			: t.max_ticks / 2;
		const FamD fams[9] = {
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
		};
		int used = 0;
		const int per_fam = evals / 10 > 8 ? evals / 10 : 8;
		float best_score = FLT_MAX;
		std::vector<float> best_th, best_ef;
		int best_duck = -1;
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
		auto EvalOne = [&](const std::vector<float>& th,
			const std::vector<float>& ef, int duck, float* sc_out)
			-> Result {
			Result rr = RideHeadingSpline(entry, w, p, t, g, th, &ef,
				t.max_ticks, duck);
			used++;
			const float sc = Score(rr, t);
			if (sc < best_score) {
				best_score = sc;
				best_th = th;
				best_ef = ef;
				best_duck = duck;
				best_r = rr;
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
		for (int fam = 0; fam < 9 && used < evals; ++fam) {
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
		return best;
	}

} // namespace Carve
} // namespace Solver
