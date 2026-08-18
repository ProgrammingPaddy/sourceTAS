#include "SolverEntrance.h"

#include <float.h>
#include <math.h>

#include <algorithm>

#include "SolverAir.h"
#include "SolverBoard.h"
#include "SolverEnvelope.h"
#include "SolverSteer.h"
#include "SolverStrafe.h"
#include "SolverWorld.h"

namespace Solver {
namespace Entrance {

	namespace {

		// One analytic landing of the two-hold family (seed only - the
		// engine flight is the recorded truth).
		struct Cand {
			bool  ok = false;
			float psi1 = 0.f, psi2 = 0.f;
			int   split = 0;
			int   n = 0;
			float theta = 0.f;   // heading at the crossing
			float s = 0.f;       // 2D speed at the crossing
			float vz = 0.f;
			float E_est = -1e30f;
			float dot_est = 0.f;
			int   iu = 0, iv = 0, br = 0;
		};

		struct BinScratch {
			Cand best;     // max estimated post-board energy
			Cand second;   // heading-diverse runner-up
			Cand tan;      // min |dot| (FastestTangent seed)
		};

		// Heading evolution shared by the analytic trace and the
		// witness profile so they cannot diverge: turn toward psi1
		// until `split`, then toward psi2, at the per-tick free rate,
		// full gain (position moves at the pre-gain speed, matching
		// the validated TraceTHT order).
		inline void StepHeading(float& h, float& s, float tgt,
		                        const MoveParams& p, bool ducked) {
			Strafe::TickLaw lk = Strafe::Law(p, s, 1.f, ducked);
			const float w = lk.TurnRad(0.f, 1.f);
			const float d = Steer::WrapPi(tgt - h);
			h = Steer::WrapPi(h + (d > w ? w : (d < -w ? -w : d)));
		}

		// Analytic trace of one profile to its first front-side,
		// in-polygon crossing of the face plane. Returns ok=false on
		// a miss (never crossed in-polygon within max_ticks).
		Cand TraceProfile(const Vec3& pos, float h0, float s0,
		                  float vz0, float gs, const Route::Face& face,
		                  const MoveParams& p, bool ducked,
		                  float psi1, float psi2, int split,
		                  int max_ticks, const Vec3& origin,
		                  const Vec3& ud, const Vec3& vd,
		                  float du, float dv, int nu, int nv) {
			Cand c;
			c.psi1 = psi1;
			c.psi2 = psi2;
			c.split = split;
			const float support = 16.f * (fabsf(face.n.X)
				+ fabsf(face.n.Y)) + 36.f * fabsf(face.n.Z) + 2.f;
			const float hn = sqrtf(face.n.X * face.n.X
				+ face.n.Y * face.n.Y);
			const float psi_az = atan2f(face.n.Y, face.n.X);
			const float cap2 = p.air_speed_cap * p.air_speed_cap;
			float h = h0, s = s0;
			float x = pos.X, y = pos.Y;
			float off_prev = Dot(face.n, pos) - face.d;
			for (int k = 1; k <= max_ticks; ++k) {
				StepHeading(h, s, k <= split ? psi1 : psi2, p, ducked);
				x += cosf(h) * s * p.dt;
				y += sinf(h) * s * p.dt;
				s = sqrtf(s * s + cap2);
				const float z = Envelope::ZAfter(pos.Z, vz0, k, p, gs);
				const float vz = Envelope::VzAfter(vz0, k, p, gs);
				const Vec3 q(x, y, z);
				const float off = Dot(face.n, q) - face.d;
				if (off <= support && off_prev > support) {
					// Front-side crossing this tick.
					if (Board::EdgeDistOut(face, q)
						<= Board::kHullCenterSlack) {
						const float dot = s * hn
							* cosf(Steer::WrapPi(h - psi_az))
							+ vz * face.n.Z;
						if (dot < 0.f) {
							c.ok = true;
							c.n = k;
							c.theta = h;
							c.s = s;
							c.vz = vz;
							c.dot_est = dot;
							c.E_est = s * s + vz * vz - dot * dot
								+ 2.f * p.gravity * gs
									* (z - face.zmin);
							const Vec3 dq = q - origin;
							int iu = static_cast<int>(
								Dot(dq, ud) / du + 0.5f);
							int iv = static_cast<int>(
								Dot(dq, vd) / dv + 0.5f);
							if (iu < 0) iu = 0;
							if (iu >= nu) iu = nu - 1;
							if (iv < 0) iv = 0;
							if (iv >= nv) iv = nv - 1;
							c.iu = iu;
							c.iv = iv;
							c.br = vz > 0.f ? 1 : 0;
						}
						return c;
					}
					// Crossed the plane outside the polygon: flew
					// past the face edge - keep flying (a real brush
					// there is the engine's business).
				}
				off_prev = off;
				if (z < face.zmin - 300.f && vz < 0.f)
					break;   // fell hopelessly below the face
			}
			return c;
		}

	} // namespace

	void WitnessProfile(const PlayerState& entry, const MoveParams& p,
	                    float psi1, float psi2, int split, int n,
	                    std::vector<float>* prof) {
		prof->clear();
		prof->reserve(static_cast<size_t>(n));
		float h = atan2f(entry.vel.Y, entry.vel.X);
		float s = Len2D(entry.vel);
		for (int k = 1; k <= n; ++k) {
			StepHeading(h, s, k <= split ? psi1 : psi2, p,
				entry.ducked);
			prof->push_back(h);
			s = sqrtf(s * s + p.air_speed_cap * p.air_speed_cap);
		}
	}

	namespace {

		// A segment schedule: the admissible L/R control space. Each
		// segment turns at `rate` (fraction of the per-tick free rate,
		// signed) for `len` ticks; a sign change between segments is a
		// strafe reversal, so every segment holds at least the dwell
		// law (conservatively applied to all segments).
		struct SegSched {
			std::vector<int>   len;
			std::vector<float> rate;
		};

		void SegProfile(const PlayerState& entry, const MoveParams& p,
		                const SegSched& ss, std::vector<float>* prof) {
			prof->clear();
			float h = atan2f(entry.vel.Y, entry.vel.X);
			float s = Len2D(entry.vel);
			const float cap2 = p.air_speed_cap * p.air_speed_cap;
			for (size_t j = 0; j < ss.len.size(); ++j)
				for (int k = 0; k < ss.len[j]; ++k) {
					Strafe::TickLaw lk = Strafe::Law(p, s, 1.f,
						entry.ducked);
					h = Steer::WrapPi(h + lk.TurnRad(0.f, 1.f)
						* ss.rate[j]);
					prof->push_back(h);
					s = sqrtf(s * s + cap2);
				}
		}

	} // namespace

	void ScheduleProfile(const PlayerState& entry, const MoveParams& p,
	                     const std::vector<int>& len,
	                     const std::vector<float>& rate,
	                     std::vector<float>* prof) {
		SegSched ss;
		ss.len = len;
		ss.rate = rate;
		SegProfile(entry, p, ss, prof);
	}

	RefResult RefSolve(const PlayerState& entry, const World& w,
	                   const MoveParams& p, const Route::Graph& g,
	                   int face_idx, const Vec3& q, float radius,
	                   int n_hint, int budget, bool tangent_mode,
	                   int* flights_counter, float zmin,
	                   const std::function<void(const Air::Result&,
	                       const std::vector<float>&, int)>&
	                       on_strike) {
		RefResult best;
		if (face_idx < 0
			|| face_idx >= static_cast<int>(g.faces.size()))
			return best;
		const Route::Face& face = g.faces[face_idx];
		const int min_gap = static_cast<int>(
			ceilf((1.f / p.dt) / p.strafe_rate_max));
		const float s0 = Len2D(entry.vel);
		if (s0 < 1.f)
			return best;
		const float h0 = atan2f(entry.vel.Y, entry.vel.X);
		const float gs = entry.gravity_scale;
		const int n_cap = (n_hint > 8 ? n_hint : 60) + 30;
		Air::Target vt;
		vt.face = face_idx;
		vt.dot_cap = 3000.f;
		vt.aim = q;
		int used = 0;
		std::vector<float> prof;
		// One engine flight of a schedule; score drives the search,
		// only clean-air in-radius strikes can WIN.
		auto ev = [&](const SegSched& ss, float* score) {
			int total = 0;
			for (int l : ss.len)
				total += l;
			if (total < 8 || total > n_cap) {
				*score = -1e9f;
				return;
			}
			SegProfile(entry, p, ss, &prof);
			vt.max_ticks = total + 8;
			used++;
			if (flights_counter)
				(*flights_counter)++;
			Air::Result ar = Air::FlyHeadingSpline(entry, w, p, vt,
				g, prof, total + 8);
			if (!ar.hit || ar.dot >= 0.f) {
				*score = -(ar.miss_dist < 1e8f ? ar.miss_dist
					: 1e8f);
				return;
			}
			if (ar.struck_brush >= 0) {
				// Contact-assisted: guides (it reached the face)
				// but can never win the clean-air field.
				best.contact_assisted++;
				*score = -Len(ar.pos - q) - 500.f;
				return;
			}
			if (on_strike)
				on_strike(ar, prof, total + 8);
			const float d = Len(ar.pos - q);
			if (d > radius) {
				*score = -d;
				return;
			}
			const float H = Dot(ar.end_state.vel, ar.end_state.vel)
				+ 2.f * p.gravity * gs * (ar.pos.Z - zmin);
			if (tangent_mode) {
				if (-ar.dot > kTanEps) {
					*score = 1e5f - fabsf(ar.dot);
					return;
				}
				*score = 1e7f + H;
			} else {
				*score = 1e7f + H;
			}
			if (*score >= 1e7f && H > best.H) {
				best.ok = true;
				best.H = H;
				best.flight = ar;
				best.prof = prof;
				best.horizon = total + 8;
			}
		};
		// Seeds: turn-to-arrival-then-hold at the closed-form tangent
		// heading and at the bearing, plus outward-bulge reversals.
		const float bear = atan2f(q.Y - entry.pos.Y,
			q.X - entry.pos.X);
		float th_arr = bear;
		{
			const float hn = sqrtf(face.n.X * face.n.X
				+ face.n.Y * face.n.Y);
			const float sa = Envelope::SMax(s0, n_hint > 8
				? n_hint : 60, p);
			const float va = Envelope::VzAfter(entry.vel.Z,
				n_hint > 8 ? n_hint : 60, p, gs);
			if (sa * hn > 1e-4f) {
				float cphi = -va * face.n.Z / (sa * hn);
				if (cphi > 1.f) cphi = 1.f;
				if (cphi < -1.f) cphi = -1.f;
				const float po = acosf(cphi);
				const float paz = atan2f(face.n.Y, face.n.X);
				const float ca = Steer::WrapPi(paz + po);
				const float cb = Steer::WrapPi(paz - po);
				th_arr = fabsf(Steer::WrapPi(ca - bear))
					<= fabsf(Steer::WrapPi(cb - bear)) ? ca : cb;
			}
		}
		const int N = n_hint > 8 ? n_hint : 60;
		auto turn_hold = [&](float tgt, float frac) {
			SegSched ss;
			const float d = Steer::WrapPi(tgt - h0);
			int tl = static_cast<int>(fabsf(d)
				/ (0.05f * (frac > 0.1f ? frac : 0.1f))) + 1;
			// Rough turn length; the search reshapes it. Clamp into
			// the schedule.
			if (tl < min_gap) tl = min_gap;
			if (tl > N) tl = N;
			ss.len.push_back(tl);
			ss.rate.push_back(d >= 0.f ? frac : -frac);
			if (N - tl >= min_gap) {
				ss.len.push_back(N - tl);
				ss.rate.push_back(0.f);
			}
			return ss;
		};
		std::vector<SegSched> pool;
		pool.push_back(turn_hold(th_arr, 1.f));
		pool.push_back(turn_hold(th_arr, 0.5f));
		pool.push_back(turn_hold(bear, 1.f));
		for (int sg = -1; sg <= 1; sg += 2) {
			// Bulge: swing away, swing back, hold - one reversal.
			SegSched ss;
			const int a = N / 3 < min_gap ? min_gap : N / 3;
			ss.len.push_back(a);
			ss.rate.push_back(0.6f * static_cast<float>(sg));
			ss.len.push_back(a);
			ss.rate.push_back(-0.9f * static_cast<float>(sg));
			if (N - 2 * a >= min_gap) {
				ss.len.push_back(N - 2 * a);
				ss.rate.push_back(0.f);
			}
			pool.push_back(ss);
		}
		float cur_best_score = -1e30f;
		SegSched cur;
		for (SegSched& ss : pool) {
			if (used >= budget)
				break;
			float sc;
			ev(ss, &sc);
			if (sc > cur_best_score) {
				cur_best_score = sc;
				cur = ss;
			}
		}
		// Coordinate descent over the schedule + growth moves. The
		// space is the full admissible one: any L/R schedule is a
		// sequence of (len >= dwell, rate) segments, and splits let
		// the search add reversals wherever they pay.
		const float rsteps[3] = { 0.35f, 0.15f, 0.06f };
		for (int round = 0; round < 3 && used < budget; ++round) {
			bool moved = true;
			int guard = 0;
			while (moved && used < budget && guard++ < 6) {
				moved = false;
				for (size_t j = 0; j < cur.len.size()
					&& used < budget; ++j) {
					// Rate nudges. Rates beyond |1| are LEGAL: the
					// commanded heading outruns full-gain tracking
					// and the controller's braking-turn branch
					// executes the excess at the law's exact speed
					// cost (measured: the human's flights spend
					// ticks at 2.4-5.3x the free rate). Ceiling 3
					// spans the useful exchange; the optimizer
					// pays for every over-rate tick in energy.
					for (int sg = -1; sg <= 1; sg += 2) {
						SegSched t2 = cur;
						float nr = t2.rate[j]
							+ rsteps[round]
							* static_cast<float>(sg);
						if (nr > 3.f) nr = 3.f;
						if (nr < -3.f) nr = -3.f;
						t2.rate[j] = nr;
						float sc;
						ev(t2, &sc);
						if (sc > cur_best_score) {
							cur_best_score = sc;
							cur = t2;
							moved = true;
						}
					}
					// Length nudges.
					const int lsteps[2] = { 6, 2 };
					for (int li = 0; li < 2; ++li)
						for (int sg = -1; sg <= 1; sg += 2) {
							SegSched t2 = cur;
							int nl = t2.len[j] + lsteps[li] * sg;
							if (nl < min_gap)
								nl = min_gap;
							t2.len[j] = nl;
							float sc;
							ev(t2, &sc);
							if (sc > cur_best_score) {
								cur_best_score = sc;
								cur = t2;
								moved = true;
							}
						}
				}
				// Growth: split the longest segment with a reversal;
				// append a tail segment.
				if (!moved && cur.len.size() < 8
					&& used < budget) {
					size_t lj = 0;
					for (size_t j = 1; j < cur.len.size(); ++j)
						if (cur.len[j] > cur.len[lj])
							lj = j;
					if (cur.len[lj] >= 2 * min_gap) {
						SegSched t2 = cur;
						const int half = t2.len[lj] / 2;
						const float f = t2.rate[lj];
						t2.len[lj] = half;
						t2.len.insert(t2.len.begin()
							+ static_cast<long long>(lj) + 1,
							t2.len[lj] == half
								? cur.len[lj] - half : half);
						t2.rate.insert(t2.rate.begin()
							+ static_cast<long long>(lj) + 1,
							f == 0.f ? 0.5f : -f * 0.7f);
						float sc;
						ev(t2, &sc);
						if (sc > cur_best_score) {
							cur_best_score = sc;
							cur = t2;
							moved = true;
						}
					}
					SegSched t3 = cur;
					t3.len.push_back(min_gap + 2);
					t3.rate.push_back(t3.rate.empty() ? 0.5f
						: -t3.rate.back() * 0.5f);
					float sc;
					ev(t3, &sc);
					if (sc > cur_best_score) {
						cur_best_score = sc;
						cur = t3;
						moved = true;
					}
				}
			}
		}
		return best;
	}

	FieldMap Build(const PlayerState& entry, const World& w,
	               const MoveParams& p, const Route::Graph& g,
	               int face_idx, const Opts& o) {
		FieldMap m;
		if (face_idx < 0
			|| face_idx >= static_cast<int>(g.faces.size()))
			return m;
		const Route::Face& face = g.faces[face_idx];
		const float hn = sqrtf(face.n.X * face.n.X
			+ face.n.Y * face.n.Y);
		if (hn < 1e-4f || face.verts.size() < 3)
			return m;
		// Grid frame - identical construction to Field::Compute so
		// cell indices line up across the two tiers.
		m.ud = Vec3(-face.n.Y / hn, face.n.X / hn, 0.f);
		Vec3 vdax = Cross(face.n, m.ud);
		if (vdax.Z < 0.f)
			vdax = Scale(vdax, -1.f);
		const float vl = Len(vdax);
		if (vl < 1e-4f)
			return m;
		m.vd = Scale(vdax, 1.f / vl);
		float ulo = FLT_MAX, uhi = -FLT_MAX;
		float wlo = FLT_MAX, whi = -FLT_MAX;
		for (const Vec3& v : face.verts) {
			const Vec3 dvtx = v - face.centroid;
			const float cu = Dot(dvtx, m.ud);
			const float cv = Dot(dvtx, m.vd);
			if (cu < ulo) ulo = cu;
			if (cu > uhi) uhi = cu;
			if (cv < wlo) wlo = cv;
			if (cv > whi) whi = cv;
		}
		m.du = o.grid;
		m.dv = o.grid;
		m.nu = static_cast<int>((uhi - ulo) / o.grid) + 1;
		m.nv = static_cast<int>((whi - wlo) / o.grid) + 1;
		if (m.nu < 1) m.nu = 1;
		if (m.nv < 1) m.nv = 1;
		m.origin = face.centroid + Scale(m.ud, ulo + 0.5f * o.grid)
			+ Scale(m.vd, wlo + 0.5f * o.grid);
		m.face = face_idx;
		const size_t ncell = static_cast<size_t>(m.nu)
			* static_cast<size_t>(m.nv);
		m.rec[0].resize(ncell);
		m.rec[1].resize(ncell);

		const float s0 = Len2D(entry.vel);
		if (s0 < 1.f)
			return m;
		const float h0 = atan2f(entry.vel.Y, entry.vel.X);
		const float vz0 = entry.vel.Z;
		const float gs = entry.gravity_scale;
		const float kPi = 3.14159265f;

		std::vector<BinScratch> scratch[2];
		scratch[0].resize(ncell);
		scratch[1].resize(ncell);

		auto consider = [&](const Cand& c) {
			if (!c.ok)
				return;
			m.traced++;
			BinScratch& b = scratch[c.br][
				static_cast<size_t>(c.iv) * m.nu + c.iu];
			if (c.E_est > b.best.E_est || !b.best.ok) {
				// Old best may become the diverse runner-up.
				if (b.best.ok && fabsf(Steer::WrapPi(
					b.best.theta - c.theta)) > 0.35f
					&& b.best.E_est > b.second.E_est)
					b.second = b.best;
				b.best = c;
			} else if (b.best.ok
				&& fabsf(Steer::WrapPi(b.best.theta - c.theta))
					> 0.35f
				&& (c.E_est > b.second.E_est || !b.second.ok)) {
				b.second = c;
			}
			if (fabsf(c.dot_est) < fabsf(b.tan.dot_est)
				|| !b.tan.ok)
				b.tan = c;
		};

		const int min_gap = static_cast<int>(
			ceilf((1.f / p.dt) / p.strafe_rate_max));
		auto trace = [&](float psi1, float psi2, int split) {
			// Flip-gap prune (soft: the controller enforces the real
			// law during engine flights - this only avoids seeding
			// profiles the controller would coast through).
			if (!o.dwell_free) {
				const float ta = Steer::WrapPi(psi1 - h0);
				const float tb = Steer::WrapPi(psi2 - psi1);
				if (ta * tb < 0.f && split > 0) {
					Strafe::TickLaw ll = Strafe::Law(p,
						Envelope::SMax(s0, split, p), 1.f,
						entry.ducked);
					const float wl = ll.TurnRad(0.f, 1.f);
					const int tat = wl > 1e-5f
						? static_cast<int>(fabsf(ta) / wl) + 1
						: split;
					if (split - tat < min_gap)
						return;
				}
			}
			consider(TraceProfile(entry.pos, h0, s0, vz0, gs, face,
				p, entry.ducked, psi1, psi2, split, o.max_ticks,
				m.origin, m.ud, m.vd, m.du, m.dv, m.nu, m.nv));
		};

		// ---- THE FAMILY SWEEP (realization strategy, not the
		// definition of the field - see header) ----
		const int ps = o.psi_steps;
		for (int i2 = 0; i2 < ps; ++i2) {
			const float psi2 = Steer::WrapPi(h0
				+ 2.f * kPi * static_cast<float>(i2)
					/ static_cast<float>(ps));
			trace(psi2, psi2, 0);   // single-hold members
			for (int i1 = 0; i1 < ps; ++i1) {
				const float psi1 = Steer::WrapPi(h0
					+ 2.f * kPi * static_cast<float>(i1)
						/ static_cast<float>(ps));
				for (int sp = o.split_step; sp <= 200;
					sp += o.split_step)
					trace(psi1, psi2, sp);
			}
		}

		// ---- LOCAL REFINEMENT (trace-level, stays in the bin) ----
		auto refine = [&](Cand& c, bool tangent_mode) {
			if (!c.ok)
				return;
			float sp = 2.f * kPi / static_cast<float>(ps) * 0.5f;
			int ss = o.split_step / 2;
			for (int round = 0; round < o.refine_rounds; ++round) {
				for (int it = 0; it < 8; ++it) {
					Cand best = c;
					const float d1[6] = { sp, -sp, 0.f, 0.f, 0.f,
						0.f };
					const float d2[6] = { 0.f, 0.f, sp, -sp, 0.f,
						0.f };
					const int dsp[6] = { 0, 0, 0, 0, ss, -ss };
					bool moved = false;
					for (int v = 0; v < 6; ++v) {
						int nsplit = c.split + dsp[v];
						if (nsplit < 0)
							nsplit = 0;
						Cand t = TraceProfile(entry.pos, h0, s0,
							vz0, gs, face, p, entry.ducked,
							Steer::WrapPi(c.psi1 + d1[v]),
							Steer::WrapPi(c.psi2 + d2[v]),
							nsplit, o.max_ticks, m.origin, m.ud,
							m.vd, m.du, m.dv, m.nu, m.nv);
						if (!t.ok || t.iu != c.iu || t.iv != c.iv
							|| t.br != c.br)
							continue;
						const bool better = tangent_mode
							? (fabsf(t.dot_est)
								< fabsf(best.dot_est) - 1e-3f)
							: (t.E_est > best.E_est + 1e-3f);
						if (better) {
							best = t;
							moved = true;
						}
					}
					c = best;
					if (!moved)
						break;
				}
				sp *= 0.33f;
				if (ss > 1)
					ss /= 3;
			}
		};
		for (int br = 0; br < 2; ++br)
			for (size_t i = 0; i < ncell; ++i) {
				refine(scratch[br][i].best, false);
				refine(scratch[br][i].tan, true);
			}

		// ---- ENGINE VERIFICATION: the witness flight IS the value
		// (witnesscheck rule: recorded H = replayed outcome, never
		// the trace estimate). Flights land where they land - each
		// result is credited to its ACTUAL cell. ----
		struct FlightJob {
			const Cand* c;
			float prio;
		};
		std::vector<FlightJob> jobs;
		for (int br = 0; br < 2; ++br)
			for (size_t i = 0; i < ncell; ++i) {
				BinScratch& b = scratch[br][i];
				if (b.best.ok) {
					m.rec[br][i].feasible = true;
					jobs.push_back({ &b.best, b.best.E_est });
				}
				if (b.second.ok)
					jobs.push_back({ &b.second,
						b.second.E_est - 1.f });
				if (b.tan.ok && (!b.best.ok
					|| fabsf(b.tan.psi1 - b.best.psi1) > 1e-4f
					|| fabsf(b.tan.psi2 - b.best.psi2) > 1e-4f
					|| b.tan.split != b.best.split))
					jobs.push_back({ &b.tan, b.tan.E_est - 0.5f });
			}
		std::sort(jobs.begin(), jobs.end(),
			[](const FlightJob& a, const FlightJob& b) {
				return a.prio > b.prio;
			});

		Air::Target vt;
		vt.face = face_idx;
		vt.dot_cap = 2000.f;
		vt.aim_region = false;
		std::vector<float> prof;
		// One engine flight; on a CLEAN-AIR face strike, credit the
		// ACTUAL landing cell. A strike after any intermediate
		// contact is rejected (the clean-air law): the ballistic
		// reduction no longer holds for that flight - it belongs to
		// the route graph as Air -> Contact -> Air, not to this
		// field.
		auto fly = [&](float psi1, float psi2, int split, int n,
			Air::Result* out) {
			WitnessProfile(entry, p, psi1, psi2, split, n, &prof);
			if (prof.empty())
				return false;
			vt.max_ticks = n + 8;
			m.flights++;
			*out = Air::FlyHeadingSpline(entry, w, p, vt, g, prof,
				n + 8);
			if (out->hit && out->dot < 0.f
				&& out->struck_brush >= 0) {
				m.contact_assisted++;
				return false;
			}
			return out->hit && out->dot < 0.f;
		};
		auto credit = [&](const Air::Result& cr, float psi1,
			float psi2, int split, int n,
			const std::vector<float>* spline_knots,
			int spline_horizon, int source) {
			const Vec3 dq = cr.pos - m.origin;
			int iu = static_cast<int>(Dot(dq, m.ud) / m.du + 0.5f);
			int iv = static_cast<int>(Dot(dq, m.vd) / m.dv + 0.5f);
			if (iu < 0) iu = 0;
			if (iu >= m.nu) iu = m.nu - 1;
			if (iv < 0) iv = 0;
			if (iv >= m.nv) iv = m.nv - 1;
			const int br = cr.v1.Z > 0.f ? 1 : 0;
			Rec& r = m.rec[br][static_cast<size_t>(iv) * m.nu + iu];
			const Vec3 vp = cr.end_state.vel;
			const float H = Dot(vp, vp) + 2.f * p.gravity * gs
				* (cr.pos.Z - face.zmin);
			r.feasible = true;
			if (H > r.H || !r.verified) {
				r.verified = true;
				r.n = cr.tick;
				r.H = H;
				r.theta = atan2f(cr.v1.Y, cr.v1.X);
				r.s2d = cr.speed2d;
				r.vz = cr.v1.Z;
				r.v_pre = cr.v1;
				r.v_post = vp;
				r.cp = cr.pos;
				r.dot = cr.dot;
				r.loss2 = cr.dot * cr.dot;
				r.psi1 = psi1;
				r.psi2 = psi2;
				r.split = split;
				r.prof_n = n;
				if (spline_knots) {
					r.knots = *spline_knots;
					r.horizon = spline_horizon;
				} else {
					r.knots.clear();
					r.horizon = 0;
				}
				r.source = source;
			}
			if (-cr.dot <= kTanEps && (H > r.tan_E || !r.tan_ok)) {
				r.tan_ok = true;
				r.tan_E = H;
				r.tan_theta = atan2f(cr.v1.Y, cr.v1.X);
				r.tan_s2d = cr.speed2d;
				r.tan_dot = cr.dot;
				r.tan_n = cr.tick;
				r.tan_psi1 = psi1;
				r.tan_psi2 = psi2;
				r.tan_split = split;
				r.tan_prof_n = n;
			}
		};
		for (const FlightJob& j : jobs) {
			if (m.flights >= o.flights_cap)
				break;
			const Cand& c = *j.c;
			// ENGINE LINE-SEARCH around the seed (the pathgate
			// lesson: the trace is geometry-blind and full-gain
			// optimistic - real flights land short/shifted; a
			// handful of psi2 offsets on the real engine recovers
			// them). Every strike is credited wherever it lands.
			const float offs[7] = { 0.f, 0.1f, -0.1f, 0.22f,
				-0.22f, 0.38f, -0.38f };
			Air::Result cr;
			float hit_psi2 = 0.f;
			bool struck = false;
			for (int att = 0; att < 7; ++att) {
				if (m.flights >= o.flights_cap)
					break;
				const float p2 = Steer::WrapPi(c.psi2
					+ offs[att]);
				if (fly(c.psi1, p2, c.split, c.n, &cr)) {
					credit(cr, c.psi1, p2, c.split, c.n,
						nullptr, 0, 0);
					hit_psi2 = p2;
					struck = true;
					break;
				}
			}
			// ENGINE-SIDE TANGENT REFINEMENT: secant steps on psi2
			// against the MEASURED dot (the trace's tangent seed
			// does not survive the flight). A step that overshoots
			// past tangency into no-contact BISECTS back toward the
			// last striking heading (the strike boundary IS the
			// tangent frontier); every strike credits its cell.
			if (struck && -cr.dot > kTanEps) {
				float p2a = hit_psi2, dota = cr.dot;
				float step = 0.08f * (cr.dot < 0.f ? 1.f : -1.f);
				for (int it = 0; it < 7 && m.flights
					< o.flights_cap; ++it) {
					const float p2b = Steer::WrapPi(p2a + step);
					Air::Result tr2;
					if (!fly(c.psi1, p2b, c.split, c.n, &tr2)) {
						step *= 0.5f;   // overshot the frontier
						continue;
					}
					credit(tr2, c.psi1, p2b, c.split, c.n,
						nullptr, 0, 0);
					if (-tr2.dot <= kTanEps)
						break;
					const float dd = tr2.dot - dota;
					p2a = p2b;
					dota = tr2.dot;
					if (fabsf(dd) < 1e-3f)
						break;
					float ns = -tr2.dot * step / dd;
					if (ns > 0.3f) ns = 0.3f;
					if (ns < -0.3f) ns = -0.3f;
					step = ns;
				}
			}
		}

		// ---- THE SECOND FAMILY: bounded spline solves for cells the
		// two-hold family cannot express (corridor approaches - the
		// f2/f3 verdict: geometry-blind open-air profiles converge to
		// the corridor wall; multi-turn spline curves do not). The
		// field's definition is ALL admissible controls - families
		// are realization layers, and every strike is still judged
		// and credited by the exact engine. ----
		{
			// Targets: feasible-but-unverified bins whose trace
			// estimate says they matter, best-first; when nothing
			// verified at all (corridor faces), every feasible bin
			// qualifies.
			struct SpTarget {
				int br;
				int idx;
				float est;
				int n;
			};
			std::vector<SpTarget> targets, bound_targets;
			for (int br = 0; br < 2; ++br)
				for (size_t i = 0; i < ncell; ++i) {
					const BinScratch& b = scratch[br][i];
					if (!b.best.ok)
						continue;
					const Rec& r = m.rec[br][i];
					if (r.verified)
						continue;
					targets.push_back({ br,
						static_cast<int>(i), b.best.E_est,
						b.best.n });
				}
			// BOUND-PROPOSED targets: cells the family trace never
			// reached but the ballistic closed forms say are in
			// reach (the human's f3 board was such a cell - traced
			// NO). Tier-0 proposes, the engine disposes.
			for (size_t i = 0; i < ncell; ++i) {
				if (scratch[0][i].best.ok || scratch[1][i].best.ok)
					continue;
				const int iu = static_cast<int>(i) % m.nu;
				const int iv = static_cast<int>(i) / m.nu;
				const Vec3 q = m.origin
					+ Scale(m.ud, static_cast<float>(iu) * m.du)
					+ Scale(m.vd, static_cast<float>(iv) * m.dv);
				if (Board::EdgeDistOut(face, q)
					> Board::kHullCenterSlack)
					continue;
				const float dzq = q.Z - entry.pos.Z;
				const float grav = p.gravity * gs;
				const float disc = vz0 * vz0 - 2.f * grav * dzq;
				if (disc < 0.f)
					continue;
				const float sq = sqrtf(disc);
				const float roots[2] = {
					(vz0 - sq) / (grav * p.dt),
					(vz0 + sq) / (grav * p.dt) };
				const float dx = q.X - entry.pos.X;
				const float dy = q.Y - entry.pos.Y;
				const float dist = sqrtf(dx * dx + dy * dy);
				for (int ri = 0; ri < 2; ++ri) {
					const float nf = roots[ri];
					if (nf < 4.f || nf > static_cast<float>(
						o.max_ticks))
						continue;
					const int n2 = static_cast<int>(nf);
					if (dist > Envelope::DMax(s0, n2 + 1, p)
						+ Board::kHullCenterSlack)
						continue;
					const float sa = Envelope::SMax(s0, n2, p);
					const float va = Envelope::VzAfter(vz0, n2,
						p, gs);
					const float est = sa * sa + va * va
						+ 2.f * p.gravity * gs
						* (q.Z - face.zmin);
					bound_targets.push_back({ ri == 0 ? 0 : 1,
						static_cast<int>(i), est, n2 });
				}
			}
			// SEPARATE QUOTAS: the trace estimate prices its landing
			// (dot^2 subtracted); the bound estimate is a loss-free
			// ceiling. Merged sorting let the optimistic scale evict
			// the realistic one (measured: f0 lost its human-cell
			// verification) - each pool keeps its own slots.
			auto by_est = [](const SpTarget& a, const SpTarget& b) {
				return a.est > b.est;
			};
			std::sort(targets.begin(), targets.end(), by_est);
			if (targets.size() > 16)
				targets.resize(16);
			std::sort(bound_targets.begin(), bound_targets.end(),
				by_est);
			if (bound_targets.size() > 6)
				bound_targets.resize(6);
			targets.insert(targets.end(), bound_targets.begin(),
				bound_targets.end());
			// Every clean-air strike the reference search flies
			// anywhere raises the field's lower bound (source=1).
			auto ref_credit = [&](const Air::Result& ar,
				const std::vector<float>& kprof, int hz) {
				credit(ar, 0.f, 0.f, 0, ar.tick, &kprof, hz, 1);
			};
			for (const SpTarget& tg : targets) {
				if (m.flights >= o.flights_cap)
					break;
				const int iu = tg.idx % m.nu;
				const int iv = tg.idx / m.nu;
				const Vec3 q = m.origin
					+ Scale(m.ud, static_cast<float>(iu) * m.du)
					+ Scale(m.vd, static_cast<float>(iv) * m.dv);
				RefResult rr = RefSolve(entry, w, p, g, face_idx,
					q, o.grid * 0.9f, tg.n, 220, false,
					&m.flights, face.zmin, ref_credit);
				m.contact_assisted += rr.contact_assisted;
			}
		}

		// ---- Stats + best ----
		m.H_lo = FLT_MAX;
		for (int br = 0; br < 2; ++br)
			for (size_t i = 0; i < ncell; ++i) {
				const Rec& r = m.rec[br][i];
				if (r.feasible)
					m.cells_feasible++;
				if (!r.verified)
					continue;
				m.cells_verified++;
				if (r.H < m.H_lo)
					m.H_lo = r.H;
				if (r.H > m.H_hi) {
					m.H_hi = r.H;
					m.best = br * static_cast<int>(ncell)
						+ static_cast<int>(i);
				}
			}
		if (m.cells_verified == 0)
			m.H_lo = 0.f;
		return m;
	}

} // namespace Entrance
} // namespace Solver
