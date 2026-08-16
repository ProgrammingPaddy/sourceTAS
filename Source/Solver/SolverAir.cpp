#include "SolverAir.h"

#include <float.h>
#include <math.h>

#include "SolverBoard.h"
#include "SolverEnvelope.h"
#include "SolverStrafe.h"

namespace Solver {
namespace Air {

	namespace {

		constexpr float kPi = 3.14159265358979f;

		float WrapPi(float a) {
			while (a > kPi) a -= 2.f * kPi;
			while (a < -kPi) a += 2.f * kPi;
			return a;
		}

		// Linear interpolation over knots placed at equal fractions of
		// [0, horizon].
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
			// Interpolate on the wrapped difference so splines can cross
			// the +/-pi seam.
			return knots[i] + WrapPi(knots[i + 1] - knots[i]) * f;
		}

		float TurnAt(const Strafe::TickLaw& law, float cosa) {
			const float s2 = 1.f - cosa * cosa;
			return law.TurnRad(cosa, s2 > 0.f ? sqrtf(s2) : 0.f);
		}

		// Invert the law's turn curve: the cosa in [0, min(1, cap/v)]
		// whose TurnRad equals want (monotone decreasing in cosa).
		float CosForTurn(const Strafe::TickLaw& law, float want) {
			float lo = 0.f;
			float hi = law.v > law.cap ? law.cap / law.v : 1.f;
			// TurnRad(lo) = max turn; TurnRad(hi) = 0.
			for (int it = 0; it < 12; ++it) {
				const float mid = 0.5f * (lo + hi);
				if (TurnAt(law, mid) > want)
					lo = mid;
				else
					hi = mid;
			}
			return 0.5f * (lo + hi);
		}

		// THE BRAKING TURN ("eat the energy in the turn", testimony
		// 2.2/2.3): with cosa < 0 the accel budget (airaccel * wishspeed
		// * dt = 562.5 standing) dwarfs the 30 cap, so a backward-biased
		// wish rotates the velocity far faster than the perpendicular
		// strafe - at real speed cost. Returns the cosa in [-1, 0]
		// achieving 'want' (or the max-turn cosa if 'want' exceeds it).
		// The turn curve rises from TurnAt(0) to a peak then falls to 0
		// at cosa=-1: scan for the peak, then bisect the rising side.
		float CosForBrakeTurn(const Strafe::TickLaw& law, float want) {
			float best_c = 0.f, best_t = TurnAt(law, 0.f);
			for (int i = 1; i <= 16; ++i) {
				const float c = -static_cast<float>(i) / 16.f;
				const float tr = TurnAt(law, c);
				if (tr > best_t) {
					best_t = tr;
					best_c = c;
				}
			}
			if (want >= best_t)
				return best_c;
			float lo = 0.f, hi = best_c;   // turn(lo) < want <= turn(hi)
			for (int it = 0; it < 12; ++it) {
				const float mid = 0.5f * (lo + hi);
				if (TurnAt(law, mid) < want)
					lo = mid;
				else
					hi = mid;
			}
			return 0.5f * (lo + hi);
		}

	} // namespace

	Result FlyHeadingSpline(const PlayerState& entry, const World& w,
	                        const MoveParams& p, const Target& t,
	                        const Route::Graph& g,
	                        const std::vector<float>& knots, int horizon) {
		Result r;
		if (t.face < 0 || t.face >= static_cast<int>(g.faces.size()))
			return r;
		const Route::Face& face = g.faces[t.face];
		PlayerState s = entry;
		// Hold the entry duck state for the whole flight (duck edges are
		// not this primitive's business; changing mid-air would shift z
		// off the tape's ballistic family).
		const int hold = s.ducked ? IN_DUCK : 0;
		const int min_gap = static_cast<int>(
			ceilf((1.f / p.dt) / p.strafe_rate_max));
		int side = 0, last_flip = -1000;
		// The spline spans the EXPECTED flight (aim_tick), not the sim
		// cap - otherwise late knots are dead parameters.
		const int sh = (t.aim_tick > 0 && t.aim_tick < horizon)
			? t.aim_tick : horizon;
		r.yaw.reserve(horizon);
		r.smove.reserve(horizon);
		for (int k = 0; k < horizon; ++k) {
			const float s2d = Len2D(s.vel);
			const float h = s2d > 1.f
				? atan2f(s.vel.Y, s.vel.X)
				: SplineEval(knots, k, sh);
			const float theta = SplineEval(knots, k, sh);
			const float e = WrapPi(theta - h);
			Strafe::TickLaw law = Strafe::Law(p, s2d, 1.f, s.ducked);
			const float maxturn = law.TurnRad(0.f, 1.f);
			int want = side;
			if (e > 0.02f) want = 1;
			else if (e < -0.02f) want = -1;
			else if (side == 0) want = (e >= 0.f) ? 1 : -1;
			bool coast = false;
			if (want != side) {
				if (side == 0 || k - last_flip >= min_gap) {
					side = want;
					last_flip = k;
					r.flips += (k > 0) ? 1 : 0;
				} else if (fabsf(e) > 0.26f) {
					coast = true;   // blocked flip, big error: coast
				}
				// else: weave on the old side until the flip is legal
			}
			float yaw_deg = h * 57.2957795f;
			float smove = 0.f;
			if (!coast && s2d > 1.f) {
				const float ae = fabsf(e);
				const bool with_side = (side > 0) == (e > 0.f)
					&& ae > 1e-4f;
				// Demand within the perp turn rate: land it exactly at
				// full gain. Beyond it: the BRAKING TURN family (speed
				// for curvature - the optimizer owns that tradeoff via
				// the spline slope). Against the side: hold the no-op
				// point (a = 0) and weave.
				const float cosa = (with_side && ae < maxturn)
					? CosForTurn(law, ae)
					: (with_side ? CosForBrakeTurn(law, ae) : law.cap
						/ (s2d > law.cap ? s2d : law.cap));
				// Wish at angle alpha from velocity on 'side'; realize
				// with a pure side-strafe (smove sign = -side keeps the
				// yaw within 90 deg of travel, the human idiom).
				const float alpha = acosf(cosa > 1.f ? 1.f : cosa);
				const float wh = h + (side > 0 ? alpha : -alpha);
				smove = -450.f * static_cast<float>(side);
				yaw_deg = (wh + (side > 0 ? 1.f : -1.f) * kPi * 0.5f)
					* 57.2957795f;
			} else if (!coast) {
				// Near-zero horizontal speed: push straight at the
				// spline heading.
				yaw_deg = theta * 57.2957795f;
				smove = 0.f;
			}
			const float fmove = (!coast && s2d <= 1.f) ? 450.f : 0.f;
			TickEvents ev;
			MoveTick(s, w, p, 0.f, yaw_deg, fmove, smove, 0.f, hold, &ev);
			r.yaw.push_back(yaw_deg);
			r.smove.push_back(smove);
			const float dxa = s.pos.X - t.aim.X;
			const float dya = s.pos.Y - t.aim.Y;
			const float dza = s.pos.Z - t.aim.Z;
			const float da = sqrtf(dxa * dxa + dya * dya + dza * dza);
			if (da < r.miss_dist)
				r.miss_dist = da;
			if (ev.ncontacts > 0) {
				if (ev.ncontacts == 1
					&& ev.contact_brush[0] == face.brush
					&& ev.contact_plane[0] == face.side) {
					r.hit = true;
					r.tick = k + 1;
					r.pos = ev.contact_pos[0];
					r.v1 = ev.contact_vel[0];
					r.dot = Dot(r.v1, face.n);
					r.edge = Board::EdgeDistOut(face, r.pos);
					r.speed = Len(r.v1);
					r.speed2d = Len2D(r.v1);
				}
				else
					r.struck_brush = ev.contact_brush[0];
				r.end_pos = s.pos;
				return r;   // any other strike = miss, flight over
			}
			if (s.on_ground) {
				r.grounded = true;
				r.end_pos = s.pos;
				return r;
			}
		}
		r.end_pos = s.pos;
		return r;
	}

	namespace {

		float Score(const Result& r, const Target& t) {
			if (!r.hit)
				return 1e6f + r.miss_dist;
			float j = -r.dot - 0.01f * r.speed2d;
			if (r.edge > 0.f)
				j += 10.f * r.edge;
			if (-r.dot > t.dot_cap)
				j += (-r.dot - t.dot_cap);
			if (t.aim_tick >= 0) {
				const int ad = r.tick > t.aim_tick
					? r.tick - t.aim_tick : t.aim_tick - r.tick;
				if (t.tick_tol >= 0 && ad > t.tick_tol)
					j += 1e4f + static_cast<float>(ad - t.tick_tol);
				else
					j += t.tick_w * static_cast<float>(ad);
			}
			return j;
		}

	} // namespace

	Result SolveTransfer(const PlayerState& entry, const World& w,
	                     const MoveParams& p, const Route::Graph& g,
	                     const Target& t, int knots_n, int evals) {
		Result best;
		if (t.face < 0 || t.face >= static_cast<int>(g.faces.size()))
			return best;
		const Route::Face& face = g.faces[t.face];
		const float s2d0 = Len2D(entry.vel);
		const float h0 = s2d0 > 1.f
			? atan2f(entry.vel.Y, entry.vel.X)
			: atan2f(t.aim.Y - entry.pos.Y, t.aim.X - entry.pos.X);

		// Geometric arrival headings: the closed-form tangent line
		// (M1.2): dot = s*h*cos(phi) + vz*nz = 0 at the ballistic
		// arrival tick (M1.1).
		const float hn = sqrtf(face.n.X * face.n.X
			+ face.n.Y * face.n.Y);
		const float psi = atan2f(face.n.Y, face.n.X);
		int n0 = 0, n1 = 0;
		int n_arr = t.max_ticks / 2;
		if (Envelope::ZWindow(entry.pos.Z, entry.vel.Z, t.aim.Z - 50.f,
			t.aim.Z + 50.f, p, t.max_ticks, &n0, &n1,
			entry.gravity_scale))
			n_arr = (n0 + n1) / 2;
		if (n_arr < 1) n_arr = 1;
		const float vz_arr = Envelope::VzAfter(entry.vel.Z, n_arr, p,
			entry.gravity_scale);
		const float s_arr = sqrtf(s2d0 * s2d0
			+ p.air_speed_cap * p.air_speed_cap
				* static_cast<float>(n_arr) * 0.7f);
		float cphi = hn > 1e-4f && s_arr > 1.f
			? -vz_arr * face.n.Z / (s_arr * hn) : 0.f;
		if (cphi > 1.f) cphi = 1.f;
		if (cphi < -1.f) cphi = -1.f;
		const float phi = acosf(cphi);
		const float bearing = atan2f(t.aim.Y - entry.pos.Y,
			t.aim.X - entry.pos.X);
		const float candA = psi + phi, candB = psi - phi;
		const float endA = fabsf(WrapPi(candA - bearing));
		const float endB = fabsf(WrapPi(candB - bearing));
		const float ends[2] = { endA <= endB ? candA : candB,
		                        endA <= endB ? candB : candA };

		if (knots_n < 2) knots_n = 2;
		// RESTART FAMILIES: pattern search on heading knots is local, and
		// the basin is set by the initial path's curvature profile (an
		// early turn and a late turn arrive at very different boards).
		// Four unseeded initials share the budget: linear-to-tangent,
		// late-turn-to-tangent (ease-in), pure pursuit of the aim
		// bearing, and the mirrored tangent candidate.
		struct Fam { float end; float pow; float bulge; };
		const Fam fams[5] = {
			{ ends[0], 1.f, 0.f },
			{ ends[0], 2.f, 0.f },      // late turn
			{ bearing, 1.f, 0.f },      // pure pursuit
			{ ends[1], 1.f, 0.f },      // mirrored tangent
			{ ends[0], 1.f, 0.5f },     // outward bump (same-face hops:
			                            // leave the plane, then return)
		};
		int used = 0;
		const int per_fam = evals / 6 > 8 ? evals / 6 : 8;
		float best_score = FLT_MAX;
		std::vector<float> best_knots;
		Result best_r;
		// Deterministic jitter source (basin hops must replay exactly).
		unsigned rng = 977u;
		auto frand_pm = [&rng]() {   // uniform in [-1, 1]
			rng = rng * 1664525u + 1013904223u;
			return static_cast<float>((rng >> 8) & 0xFFFFu)
				/ 32767.5f - 1.f;
		};
		auto EvalOne = [&](const std::vector<float>& kn, float* sc_out)
			-> Result {
			Result rr = FlyHeadingSpline(entry, w, p, t, g, kn,
				t.max_ticks);
			used++;
			const float sc = Score(rr, t);
			if (sc < best_score) {
				best_score = sc;
				best_knots = kn;
				best_r = rr;
			}
			if (sc_out) *sc_out = sc;
			return rr;
		};
		// Coordinate descent from kn with annealed step; stops on
		// convergence or when its budget slice is gone. Returns evals
		// spent; kn/kn_score updated in place.
		auto Descend = [&](std::vector<float>& kn, float& kn_score,
			float delta, int budget) -> int {
			int spent = 0;
			while (spent < budget && used < evals && delta > 0.02f) {
				bool improved = false;
				for (int i = 0; i < knots_n && spent < budget
					&& used < evals; ++i) {
					for (int sgn = -1; sgn <= 1 && spent < budget
						&& used < evals; sgn += 2) {
						std::vector<float> trial = kn;
						trial[i] = WrapPi(trial[i]
							+ delta * static_cast<float>(sgn));
						float sc;
						EvalOne(trial, &sc);
						spent++;
						if (sc < kn_score) {
							kn = trial;
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
		for (int fam = 0; fam < 5 && used < evals; ++fam) {
			std::vector<float> knots(knots_n);
			for (int i = 0; i < knots_n; ++i) {
				float f = static_cast<float>(i)
					/ static_cast<float>(knots_n - 1);
				const float bf = sinf(kPi * f) * fams[fam].bulge;
				f = powf(f, fams[fam].pow);
				knots[i] = h0 + WrapPi(fams[fam].end - h0) * f;
				if (bf > 0.f)
					knots[i] = knots[i]
						+ WrapPi(psi - knots[i]) * bf;
			}
			float kscore;
			EvalOne(knots, &kscore);
			int spent = 1;
			spent += Descend(knots, kscore, 0.6f, per_fam - spent);
			// BASIN HOPPING: the descent converges long before the
			// budget - spend the rest restarting from the family best
			// plus jitter (a converged descent is a local statement;
			// the window problem is multimodal).
			while (spent + 1 < per_fam && used < evals) {
				std::vector<float> hop = knots;
				for (int i = 0; i < knots_n; ++i)
					hop[i] = WrapPi(hop[i] + 0.35f * frand_pm());
				float hscore;
				EvalOne(hop, &hscore);
				spent++;
				spent += Descend(hop, hscore, 0.3f, per_fam - spent);
				if (hscore < kscore) {
					knots = hop;
					kscore = hscore;
				}
			}
		}
		// Polish share: hop tightly around the global best to the end
		// of the budget.
		while (used < evals && !best_knots.empty()) {
			std::vector<float> hop = best_knots;
			for (int i = 0; i < knots_n; ++i)
				hop[i] = WrapPi(hop[i] + 0.12f * frand_pm());
			float hscore;
			EvalOne(hop, &hscore);
			Descend(hop, hscore, 0.15f, evals - used);
		}
		best = best_r;
		return best;
	}

} // namespace Air
} // namespace Solver
