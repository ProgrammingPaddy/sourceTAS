#include "SolverAir.h"

#include <float.h>
#include <math.h>

#include <algorithm>

#include "SolverBoard.h"
#include "SolverEnvelope.h"
#include "SolverSearchLog.h"
#include "SolverSteer.h"
#include "SolverStrafe.h"

namespace Solver {
namespace Air {

	namespace {

		constexpr float kPi = Steer::kSteerPi;
		using Steer::WrapPi;

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
		Steer::Controller ctl;
		// The spline spans the EXPECTED flight (aim_tick), not the sim
		// cap - otherwise late knots are dead parameters.
		const int sh = (t.aim_tick > 0 && t.aim_tick < horizon)
			? t.aim_tick : horizon;
		SearchLog::Sink* lg = SearchLog::g_sink;
		if (lg)
			lg->StartTraj(s.pos, Len(s.vel));
		r.yaw.reserve(horizon);
		r.fmove.reserve(horizon);
		r.smove.reserve(horizon);
		// Terminal tangent tracking (testimony 2.1 as CONSTRUCTION):
		// when contact is ballistically imminent, the commanded
		// heading becomes the closed-form tangent-arrival heading for
		// the CURRENT state - the arrival attitude stops being a
		// searched dimension. Engages inside the law-derived turn
		// horizon (ticks needed at the free-turn rate + 1).
		const float t_hn = sqrtf(face.n.X * face.n.X
			+ face.n.Y * face.n.Y);
		const float t_psi = atan2f(face.n.Y, face.n.X);
		// LATCHED: turning toward tangent reduces the closing rate,
		// which grows the predicted impact time, which would
		// disengage a naive condition - bang-bang chatter that lands
		// mid-hard (measured -381 vs -162). Once inside the turn
		// horizon, track the (moving) tangent target until contact.
		bool term_latch = false;
		float term_phi = 0.f;
		for (int k = 0; k < horizon; ++k) {
			float theta = SplineEval(knots, k, sh);
			if (t.terminal_tangent && t_hn > 1e-4f) {
				const float off = Dot(face.n, s.pos) - face.d;
				const float closing = -Dot(face.n, s.vel);
				if (off > 0.f && closing > 1.f) {
					const float n_hit = off / (closing * p.dt);
					if (n_hit < 90.f) {
						const float s2dn = Len2D(s.vel);
						const float vz_hit = s.vel.Z
							- p.gravity * s.gravity_scale * p.dt
							* n_hit;
						float cphi = s2dn * t_hn > 1e-4f
							? -vz_hit * face.n.Z / (s2dn * t_hn)
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
			}
			if (term_latch)
				theta = term_phi;
			float yaw_deg, fmove, smove;
			ctl.Tick(s, p, theta, k, &yaw_deg, &fmove, &smove);
			TickEvents ev;
			MoveTick(s, w, p, 0.f, yaw_deg, fmove, smove, 0.f, hold, &ev);
			if (lg)
				lg->Point(s.pos, Len(s.vel));
			r.yaw.push_back(yaw_deg);
			r.fmove.push_back(fmove);
			r.smove.push_back(smove);
			float da;
			if (t.aim_region) {
				// Distance to the face itself: plane offset +
				// outside-the-SAFE-INTERIOR shortfall (edge minus
				// the hull-center slack, M1.2 - the raw boundary is
				// still a marginal strike). FRONT SIDE ONLY - a
				// strike comes from off > 0 (same law as the carve
				// tap gradient); behind-the-plane closeness is not
				// approach.
				const float off = Dot(face.n, s.pos) - face.d;
				const float eo = Board::EdgeDistOut(face, s.pos)
					+ Board::kHullCenterSlack;
				da = off > 0.f
					? sqrtf(off * off + (eo > 0.f ? eo * eo : 0.f))
					: 1e9f;
			} else {
				const float dxa = s.pos.X - t.aim.X;
				const float dya = s.pos.Y - t.aim.Y;
				const float dza = s.pos.Z - t.aim.Z;
				da = sqrtf(dxa * dxa + dya * dya + dza * dza);
			}
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
					r.end_state = s;
					r.end_pos = s.pos;
					r.flips = ctl.flips;
					if (lg)
						lg->EndTraj(SearchLog::kHit);
					return r;
				}
				// Any other contact is a GRAZE: clip and keep flying
				// (wall-grazes are legal surf; ending the flight here
				// killed chains 37u short of their face). The clip
				// cost shows up in the arrival speed on its own.
				if (r.struck_brush < 0)
					r.struck_brush = ev.contact_brush[0];
			}
			if (s.on_ground) {
				r.grounded = true;
				r.end_pos = s.pos;
				r.flips = ctl.flips;
				if (lg)
					lg->EndTraj(SearchLog::kGrounded);
				return r;
			}
		}
		r.end_pos = s.pos;
		r.flips = ctl.flips;
		if (lg)
			lg->EndTraj(SearchLog::kMiss);
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
	                     const Target& t, int knots_n, int evals,
	                     std::vector<Result>* alts, int alts_k) {
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
		// UNSEEDED solves have no aim_tick: the spline must still span
		// the EXPECTED flight, not the sim cap (the M1.3 dead-knot
		// lesson) - use the ballistic arrival estimate.
		Target t2 = t;
		if (t2.aim_tick <= 0) {
			t2.aim_tick = n_arr;
			t2.tick_w = 0.f;
			t2.tick_tol = -1;
		}
		const Target& tt = t2;
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
		// Unseeded solves also search TWO spline domains: the direct
		// ballistic arrival and a 1.7x longer swing (a tangent board
		// often needs the wide approach the direct fall cannot shape -
		// the tape's own first transfer is a 73-tick swing over a
		// ~45-tick drop).
		const bool dual_dom = t.aim_tick <= 0;
		const int n_dom = dual_dom ? 2 : 1;
		const int per_fam = evals / (6 * n_dom) > 8
			? evals / (6 * n_dom) : 8;
		float best_score = FLT_MAX;
		std::vector<float> best_knots;
		int best_dom_tick = t2.aim_tick;
		Result best_r;
		// Deterministic jitter source (basin hops must replay exactly).
		unsigned rng = 977u;
		auto frand_pm = [&rng]() {   // uniform in [-1, 1]
			rng = rng * 1664525u + 1013904223u;
			return static_cast<float>((rng >> 8) & 0xFFFFu)
				/ 32767.5f - 1.f;
		};
		// Diverse-hit pool for the caller's downstream composition:
		// best result per STRIKE REGION (48u clusters).
		struct PR { Result r; float sc; };
		std::vector<PR> pool;
		auto EvalOne = [&](const std::vector<float>& kn, float* sc_out)
			-> Result {
			Result rr = FlyHeadingSpline(entry, w, p, tt, g, kn,
				tt.max_ticks);
			used++;
			const float sc = Score(rr, tt);
			if (SearchLog::g_sink)
				SearchLog::g_sink->Score(sc);
			if (sc < best_score) {
				best_score = sc;
				best_knots = kn;
				best_dom_tick = tt.aim_tick;
				best_r = rr;
				if (SearchLog::g_sink)
					SearchLog::g_sink->MarkBest();
			}
			if (alts && rr.hit) {
				bool merged = false;
				for (PR& pr : pool) {
					if (Len(pr.r.pos - rr.pos) < 48.f) {
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
		const int dom_base = t2.aim_tick;
		for (int dom = 0; dom < n_dom && used < evals; ++dom) {
			t2.aim_tick = dom == 0 ? dom_base
				: static_cast<int>(dom_base * 1.7f);
			if (t2.aim_tick >= t2.max_ticks)
				t2.aim_tick = t2.max_ticks - 1;
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
				spent += Descend(knots, kscore, 0.6f,
					per_fam - spent);
				// BASIN HOPPING: the descent converges long before
				// the budget - spend the rest restarting from the
				// family best plus jitter.
				while (spent + 1 < per_fam && used < evals) {
					std::vector<float> hop = knots;
					for (int i = 0; i < knots_n; ++i)
						hop[i] = WrapPi(hop[i]
							+ 0.35f * frand_pm());
					float hscore;
					EvalOne(hop, &hscore);
					spent++;
					spent += Descend(hop, hscore, 0.3f,
						per_fam - spent);
					if (hscore < kscore) {
						knots = hop;
						kscore = hscore;
					}
				}
			}
		}
		// Polish share: hop tightly around the global best (in ITS
		// domain) to the end of the budget.
		t2.aim_tick = best_dom_tick;
		while (used < evals && !best_knots.empty()) {
			std::vector<float> hop = best_knots;
			for (int i = 0; i < knots_n; ++i)
				hop[i] = WrapPi(hop[i] + 0.12f * frand_pm());
			float hscore;
			EvalOne(hop, &hscore);
			Descend(hop, hscore, 0.15f, evals - used);
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

} // namespace Air
} // namespace Solver
