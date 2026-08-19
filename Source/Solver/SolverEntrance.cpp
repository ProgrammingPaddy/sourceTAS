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

	namespace {

		// Layer-1 search representation over the canonical wish space
		// (ruling 2026-08-18): side-RUNS - each a strafe side held for
		// `len` ticks carrying a small cosa KNOT LADDER (linear across
		// the run). Runs expand to the per-tick (side, cosa) schedule
		// Air::FlyWishSchedule executes; the ladder refines toward
		// per-tick resolution. A RESOLUTION DIAL, never a trajectory
		// family. All nonzero runs hold >= the dwell law
		// (conservative: applied at every run, not only sign flips).
		struct WishRun {
			int len = 0;
			signed char side = 0;
			std::vector<float> ck;
		};

		void ExpandRuns(const std::vector<WishRun>& runs,
		                std::vector<signed char>* side,
		                std::vector<float>* cosa) {
			side->clear();
			cosa->clear();
			for (const WishRun& r : runs) {
				const int K = static_cast<int>(r.ck.size());
				for (int i = 0; i < r.len; ++i) {
					float c = 1.f;
					if (K == 1) {
						c = r.ck[0];
					} else if (K > 1) {
						float x = r.len > 1
							? static_cast<float>(i)
								/ static_cast<float>(r.len - 1)
								* static_cast<float>(K - 1)
							: 0.f;
						int k0 = static_cast<int>(x);
						if (k0 > K - 2)
							k0 = K - 2;
						const float fr = x
							- static_cast<float>(k0);
						c = r.ck[static_cast<size_t>(k0)]
							* (1.f - fr)
							+ r.ck[static_cast<size_t>(k0) + 1]
							* fr;
					}
					if (c > 1.f) c = 1.f;
					if (c < -1.f) c = -1.f;
					side->push_back(r.side);
					cosa->push_back(c);
				}
			}
		}



		// SEQUENTIAL MULTI-NODE GUIDED SHOOTING (Level A, ruling
		// 2026-08-18): a GENERIC conditioning transform over the same
		// canonical wish-schedule problem - never a waypoint family.
		// Intermediate nodes are free numerical variables the CALLER
		// searches (neutral interpolation + symmetric lateral
		// offsets from start/target geometry; nothing map- or
		// tape-derived). Simulation is continuous (no state resets):
		// guide toward node 1, then node 2, then the final
		// (Q, theta). Guidance per tick uses a SHORT-HORIZON
		// law-model lookahead (hold a candidate wish ~6 ticks, then
		// straight-run to the active target's tick) instead of pure
		// one-tick greed - the direct-pull rule cuts corners the
		// optimum may need. The dwell law is hard inline; gain is
		// SOFT (deliberate braking stays reachable). Output = the
		// frozen realized schedule; the caller replays it open-loop
		// and trusts nothing else.
		struct GuideTarget {
			float x = 0.f, y = 0.f;  // horizontal target
			// Terminal-heading INTERVAL (advisor: the solve is
			// conditioned on (Q, T, theta) jointly - each heading
			// interval independently establishes its best reachable
			// class; cost = squared distance OUTSIDE the interval).
			float th_lo = 0.f, th_hi = 0.f;
			bool  use_theta = false; // only the final target sets it
			int   until = 0;         // guide toward this through k < until
		};

		// Squared angular distance from h to the interval
		// [th_lo, th_hi] (wrapped; 0 inside).
		inline float ThetaIntervalDist2(float h, float lo, float hi) {
			const float mid = Steer::WrapPi(lo
				+ 0.5f * Steer::WrapPi(hi - lo));
			const float half = 0.5f * fabsf(Steer::WrapPi(hi - lo));
			const float d = fabsf(Steer::WrapPi(h - mid));
			const float out = d - half;
			return out > 0.f ? out * out : 0.f;
		}

		struct GuideWeights {
			float wp = 2.f;      // endpoint attraction
			float wt = 1.f;      // terminal-heading weight
			float blend = 0.35f; // final fraction where wt engages
			float we = 0.3f;     // gain-sacrifice reluctance (SOFT)
			// Second-stage EXACT-ENGINE rollout ranking (advisor:
			// cheap closed form -> shortlist -> exact short rollout
			// -> choose). Search machinery only; costs are charged
			// by the caller. 0 = closed-form only.
			int exact_top = 0;   // exact-rollout the top-K candidates
			// Extra sim ticks spent on exact rollouts (the caller
			// converts to flight-equivalents for budget charging).
			int rollout_ticks = 0;
		};

		void GuidedShootSeq(const PlayerState& entry, const World& w,
		                    const MoveParams& p,
		                    const std::vector<GuideTarget>& targets,
		                    int N, int min_gap,
		                    const Steer::CtlState& ctl0,
		                    const GuideWeights& gw,
		                    std::vector<signed char>* side_out,
		                    std::vector<float>* cosa_out,
		                    int* rollout_out = nullptr) {
			side_out->clear();
			cosa_out->clear();
			if (targets.empty())
				return;
			PlayerState s = entry;
			const int hold = s.ducked ? IN_DUCK : 0;
			signed char cur = ctl0.side;
			int age = ctl0.age;
			const float cap2 = p.air_speed_cap * p.air_speed_cap;
			static const float cgrid[7] = { 0.97f, 0.8f, 0.5f, 0.2f,
				0.f, -0.35f, -0.7f };
			const int kLook = 6;   // lookahead ticks (search knob)
			for (int k = 0; k < N; ++k) {
				// Active target: first whose window covers k.
				size_t ti = 0;
				while (ti + 1 < targets.size()
					&& k >= targets[ti].until)
					ti++;
				const GuideTarget& tg = targets[ti];
				const bool final_tg = ti + 1 == targets.size();
				const int t_end = final_tg ? N : tg.until;
				const float s2d = Len2D(s.vel);
				int bs = 0;
				float bc = 1.f;
				if (s2d > 1.f) {
					const float h = atan2f(s.vel.Y, s.vel.X);
					const int lk = kLook < t_end - k
						? kLook : (t_end - k > 1 ? t_end - k : 1);
					// Shared cost on a predicted lookahead-end state.
					auto costOf = [&](float hh, float ss, float px,
						float py, float sac) {
						const int rem = t_end - k - lk;
						if (rem > 0) {
							const float sav = sqrtf(ss * ss + cap2
								* static_cast<float>(rem) * 0.5f);
							const float D = sav * p.dt
								* static_cast<float>(rem);
							px += cosf(hh) * D;
							py += sinf(hh) * D;
						}
						const float dxq = tg.x - px;
						const float dyq = tg.y - py;
						float J = gw.wp
							* (dxq * dxq + dyq * dyq) * 1e-4f;
						if (final_tg && tg.use_theta
							&& static_cast<float>(N - k)
								<= gw.blend
								* static_cast<float>(N))
							J += gw.wt * ThetaIntervalDist2(hh,
								tg.th_lo, tg.th_hi) * 100.f;
						if (!final_tg) {
							// REMAINING-RESIDUAL: a node reached
							// beautifully that leaves an
							// unsolvable final problem is a bad
							// node (closed-form feasibility).
							const GuideTarget& fin =
								targets.back();
							const int rem2 = N - t_end;
							if (rem2 > 0) {
								const float mid = Steer::WrapPi(
									fin.th_lo + 0.5f
									* Steer::WrapPi(fin.th_hi
										- fin.th_lo));
								const float need = fabsf(
									Steer::WrapPi(mid - hh));
								Strafe::TickLaw lr = Strafe::Law(
									p, ss, 1.f, s.ducked);
								const float tfree = lr.TurnRad(
									0.f, 1.f)
									* static_cast<float>(rem2);
								const float tshort = need - tfree;
								if (tshort > 0.f)
									J += 50.f * tshort * tshort;
								const float ddx = fin.x - px;
								const float ddy = fin.y - py;
								const float drem = sqrtf(ddx * ddx
									+ ddy * ddy);
								const float reach = Envelope::DMax(
									ss, rem2, p);
								const float rshort = drem - reach;
								if (rshort > 0.f)
									J += 4e-4f * rshort * rshort;
							}
						}
						J += gw.we * sac * 1e-3f
							/ static_cast<float>(lk);
						return J;
					};
					// Candidates: constant cosa AND within-side
					// linear (a -> b) profiles - the dwell law fixes
					// the SIDE for six ticks, never the wish angle;
					// braking turns may need the angle to evolve
					// inside one side.
					struct Cand2 {
						int si;
						float a, b, J;
					};
					std::vector<Cand2> cands;
					for (int si = -1; si <= 1; ++si) {
						if (si != 0 && cur != 0 && si != cur
							&& age < min_gap)
							continue;
						if (si == 0) {
							cands.push_back({ 0, 1.f, 1.f, 0.f });
							continue;
						}
						for (int ci = 0; ci < 7; ++ci)
							cands.push_back({ si, cgrid[ci],
								cgrid[ci], 0.f });
						static const float pg[4][2] = {
							{ 0.9f, 0.2f }, { 0.2f, 0.9f },
							{ 0.f, -0.5f }, { -0.5f, 0.3f } };
						for (int pi3 = 0; pi3 < 4; ++pi3)
							cands.push_back({ si, pg[pi3][0],
								pg[pi3][1], 0.f });
					}
					// Stage 1: closed-form law rollout ranks all.
					for (Cand2& cd : cands) {
						float hh = h, ss = s2d;
						float px = s.pos.X, py = s.pos.Y;
						float sac = 0.f;
						for (int j = 0; j < lk; ++j) {
							const float c = cd.si == 0 ? 1.f
								: cd.a + (cd.b - cd.a)
								* (lk > 1 ? static_cast<float>(j)
									/ static_cast<float>(lk - 1)
									: 0.f);
							if (cd.si != 0) {
								Strafe::TickLaw law = Strafe::Law(
									p, ss, 1.f, s.ducked);
								const float s2c = 1.f - c * c;
								const float tr = law.TurnRad(c,
									s2c > 0.f ? sqrtf(s2c) : 0.f);
								hh = Steer::WrapPi(hh
									+ (cd.si > 0 ? tr : -tr));
								const float nv2 = law.NewSpeed2(c);
								sac += ss * ss + cap2
									- (nv2 > 0.f ? nv2 : 0.f);
								ss = nv2 > 0.f ? sqrtf(nv2) : 0.f;
							} else {
								sac += cap2;
							}
							px += cosf(hh) * ss * p.dt;
							py += sinf(hh) * ss * p.dt;
						}
						cd.J = costOf(hh, ss, px, py, sac);
					}
					std::sort(cands.begin(), cands.end(),
						[](const Cand2& x, const Cand2& y) {
							return x.J < y.J;
						});
					size_t pick = 0;
					// Stage 2: EXACT-ENGINE short rollout of the
					// shortlist (search machinery; replay remains
					// the only truth; ticks are charged).
					if (gw.exact_top > 0 && cands.size() > 1) {
						const size_t K = static_cast<size_t>(
							gw.exact_top) < cands.size()
							? static_cast<size_t>(gw.exact_top)
							: cands.size();
						float bestJx = FLT_MAX;
						for (size_t qi = 0; qi < K; ++qi) {
							PlayerState sim = s;
							float sac = 0.f;
							for (int j = 0; j < lk; ++j) {
								const float sj = Len2D(sim.vel);
								if (sj < 1.f)
									break;
								const float hj = atan2f(sim.vel.Y,
									sim.vel.X);
								const float c = cands[qi].si == 0
									? 1.f
									: cands[qi].a + (cands[qi].b
										- cands[qi].a)
									* (lk > 1
										? static_cast<float>(j)
										/ static_cast<float>(
											lk - 1) : 0.f);
								float yj = hj * 57.2957795f;
								float fj = 0.f, sm = 0.f;
								if (cands[qi].si != 0)
									Air::WishInputs(hj,
										cands[qi].si, c, &yj,
										&fj, &sm);
								const float pre2 = sj * sj + cap2;
								MoveTick(sim, w, p, 0.f, yj, fj,
									sm, 0.f, hold, nullptr);
								const float post = Len2D(sim.vel);
								sac += pre2 - post * post;
								if (rollout_out)
									(*rollout_out)++;
								if (sim.on_ground)
									break;
							}
							const float sx = Len2D(sim.vel);
							const float hx = sx > 1.f
								? atan2f(sim.vel.Y, sim.vel.X)
								: h;
							const float Jx = costOf(hx, sx,
								sim.pos.X, sim.pos.Y, sac);
							if (Jx < bestJx) {
								bestJx = Jx;
								pick = qi;
							}
						}
					}
					bs = cands[pick].si;
					bc = cands[pick].a;
				}
				float yaw_deg = s2d > 1.f
					? atan2f(s.vel.Y, s.vel.X) * 57.2957795f : 0.f;
				float fmove = 0.f, smove = 0.f;
				if (bs != 0 && s2d > 1.f)
					Air::WishInputs(atan2f(s.vel.Y, s.vel.X), bs,
						bc, &yaw_deg, &fmove, &smove);
				TickEvents ev2;
				MoveTick(s, w, p, 0.f, yaw_deg, fmove, smove, 0.f,
					hold, &ev2);
				side_out->push_back(static_cast<signed char>(bs));
				cosa_out->push_back(bc);
				if (bs != 0) {
					if (cur != 0 && bs != cur)
						age = 1;
					else
						age++;
					cur = static_cast<signed char>(bs);
				} else {
					age++;
				}
				if (ev2.ncontacts > 0 || s.on_ground)
					break;   // any contact ends generation
			}
		}

	} // namespace

	RefResult RefSolve(const PlayerState& entry, const World& w,
	                   const MoveParams& p, const Route::Graph& g,
	                   int face_idx, const Vec3& q, float radius,
	                   int n_hint, int budget, bool tangent_mode,
	                   int* flights_counter, float zmin,
	                   const Steer::CtlState& entry_ctl,
	                   const std::function<void(const Air::Result&,
	                       const std::vector<signed char>&,
	                       const std::vector<float>&, int)>&
	                       on_strike,
	                   const std::vector<signed char>* seed_side,
	                   const std::vector<float>* seed_cosa) {
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
		const int N = n_hint > 8 ? n_hint : 60;
		const int n_cap = N + 40;
		Air::Target vt;
		vt.face = face_idx;
		vt.dot_cap = 3000.f;
		vt.aim = q;
		std::vector<signed char> sbuf;
		std::vector<float> cbuf;
		// ---- THE TERMINAL-HEADING FRONTIER + THE SCOUT POOL (ruling
		// 2026-08-18): bins preserve terminal-state diversity; scouts
		// maximize the probability of entering difficult endpoint
		// basins. Both feed the same canonical witness frontier. ----
		constexpr int kBins = 24;
		struct BinElite {
			bool  has = false;
			float sc = -1e30f;
			float H = -1e30f;
			float dot = 0.f;
			Vec3  epos;
			std::vector<WishRun> runs;
		};
		std::vector<BinElite> bins(kBins);
		std::vector<BinElite> scouts(3);
		const float kPi2 = 3.14159265f;
		// Schedule admissibility against the BOUNDARY control state
		// (Invariant 9: the dwell law crosses operator seams).
		auto sched_legal = [&](const std::vector<signed char>& sd) {
			signed char cur = entry_ctl.side;
			int age = entry_ctl.age;
			for (signed char v : sd) {
				if (v != 0 && cur != 0 && v != cur) {
					if (age < min_gap)
						return false;
					age = 1;
				} else {
					age++;
				}
				if (v != 0)
					cur = v;
			}
			return true;
		};
		// Compress a per-tick schedule into side-runs with per-tick
		// knots (lossless; the deepen moves then operate on it).
		auto runs_from_sched = [&](const std::vector<signed char>& sd,
			const std::vector<float>& cs) {
			std::vector<WishRun> runs;
			size_t a = 0;
			while (a < sd.size()) {
				size_t b = a;
				while (b < sd.size() && sd[b] == sd[a])
					b++;
				WishRun r;
				r.len = static_cast<int>(b - a);
				r.side = sd[a];
				for (size_t i = a; i < b; ++i)
					r.ck.push_back(cs[i]);
				runs.push_back(r);
				a = b;
			}
			return runs;
		};
		// The CORE evaluation: fly one per-tick schedule. Score drives
		// the search; only clean-air in-radius strikes can WIN. Every
		// eval feeds the scout pool; every strike feeds its terminal-
		// heading bin and the tangent slot.
		auto ev_sched = [&](const std::vector<signed char>& sd,
			const std::vector<float>& cs,
			const std::vector<WishRun>* src_runs) {
			const int total = static_cast<int>(sd.size());
			if (total < 8 || total > n_cap)
				return -1e9f;
			if (!sched_legal(sd))
				return -1e9f;
			vt.max_ticks = total + 8;
			best.evals++;
			if (flights_counter)
				(*flights_counter)++;
			Air::Result ar = Air::FlyWishSchedule(entry, w, p, vt,
				g, sd, cs, total + 8);
			const Vec3 epos = ar.hit ? ar.pos
				: (ar.miss_dist < 1e8f ? ar.closest : ar.end_pos);
			float sc;
			bool strike = false;
			float H = -1e30f;
			if (!ar.hit || ar.dot >= 0.f) {
				sc = -(ar.miss_dist < 1e8f ? ar.miss_dist : 1e8f);
			} else if (ar.struck_brush >= 0) {
				best.contact_assisted++;
				sc = -Len(ar.pos - q) - 500.f;
			} else {
				if (on_strike)
					on_strike(ar, sd, cs, total + 8);
				const float d = Len(ar.pos - q);
				if (d > radius) {
					sc = -d;
				} else {
					strike = true;
					H = Dot(ar.end_state.vel, ar.end_state.vel)
						+ 2.f * p.gravity * gs
						* (ar.pos.Z - zmin);
					if (tangent_mode && -ar.dot > kTanEps)
						sc = 1e5f - fabsf(ar.dot);
					else
						sc = 1e7f + H;
				}
			}
			// Scout pool: endpoint-diverse top-3 by raw score.
			{
				int slot = -1;
				for (int i2 = 0; i2 < 3; ++i2)
					if (scouts[static_cast<size_t>(i2)].has
						&& Len(scouts[static_cast<size_t>(i2)]
							.epos - epos) < 32.f) {
						slot = i2;
						break;
					}
				if (slot < 0)
					for (int i2 = 0; i2 < 3; ++i2)
						if (!scouts[static_cast<size_t>(i2)].has) {
							slot = i2;
							break;
						}
				if (slot < 0) {
					slot = 0;
					for (int i2 = 1; i2 < 3; ++i2)
						if (scouts[static_cast<size_t>(i2)].sc
							< scouts[static_cast<size_t>(slot)].sc)
							slot = i2;
				}
				BinElite& sl = scouts[static_cast<size_t>(slot)];
				if (!sl.has || sc > sl.sc) {
					sl.has = true;
					sl.sc = sc;
					sl.H = H;
					sl.epos = epos;
					sl.runs = src_runs ? *src_runs
						: runs_from_sched(sd, cs);
				}
			}
			if (strike) {
				const float th = atan2f(ar.v1.Y, ar.v1.X);
				int bi = static_cast<int>((th + kPi2)
					/ (2.f * kPi2) * static_cast<float>(kBins));
				if (bi < 0) bi = 0;
				if (bi >= kBins) bi = kBins - 1;
				BinElite& be = bins[static_cast<size_t>(bi)];
				if (sc > be.sc || !be.has) {
					be.has = true;
					be.sc = sc;
					be.H = H;
					be.dot = ar.dot;
					be.epos = epos;
					be.runs = src_runs ? *src_runs
						: runs_from_sched(sd, cs);
				}
				if (sc >= 1e7f && H > best.H) {
					best.ok = true;
					best.H = H;
					best.flight = ar;
					best.wside = sd;
					best.wcosa = cs;
					best.horizon = total + 8;
				}
				if (-ar.dot <= kTanEps && H > best.tan_H) {
					best.tan_ok = true;
					best.tan_H = H;
					best.tan_flight = ar;
					best.tan_wside = sd;
					best.tan_wcosa = cs;
					best.tan_horizon = total + 8;
				}
			}
			return sc;
		};
		auto ev = [&](const std::vector<WishRun>& runs) {
			ExpandRuns(runs, &sbuf, &cbuf);
			return ev_sched(sbuf, cbuf, &runs);
		};
		// ---- SEED GRID (deterministic) ----
		const float bear = atan2f(q.Y - entry.pos.Y,
			q.X - entry.pos.X);
		float th_arr = bear;
		{
			const float hn = sqrtf(face.n.X * face.n.X
				+ face.n.Y * face.n.Y);
			const float sa = Envelope::SMax(s0, N, p);
			const float va = Envelope::VzAfter(entry.vel.Z, N, p,
				gs);
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
		const signed char s_to = static_cast<signed char>(
			Steer::WrapPi(th_arr - h0) >= 0.f ? 1 : -1);
		auto one_run = [&](signed char sd, float c0, float c1) {
			std::vector<WishRun> runs(1);
			runs[0].len = N;
			runs[0].side = sd;
			runs[0].ck.push_back(c0);
			runs[0].ck.push_back(c1);
			return runs;
		};
		auto two_run = [&](signed char sd0, float c0, int l0,
			signed char sd1, float c1a, float c1b) {
			std::vector<WishRun> runs(2);
			if (l0 < min_gap) l0 = min_gap;
			if (N - l0 < min_gap) l0 = N - min_gap;
			runs[0].len = l0;
			runs[0].side = sd0;
			runs[0].ck.push_back(c0);
			runs[1].len = N - l0;
			runs[1].side = sd1;
			runs[1].ck.push_back(c1a);
			runs[1].ck.push_back(c1b);
			return runs;
		};
		std::vector<std::vector<WishRun>> seeds;
		for (int m2 = 0; m2 < 2; ++m2) {
			const signed char sd = m2 ? static_cast<signed char>(
				-s_to) : s_to;
			seeds.push_back(one_run(sd, 0.0f, 0.9f));
			seeds.push_back(one_run(sd, 0.4f, 0.9f));
			seeds.push_back(one_run(sd, 0.9f, 0.9f));
			seeds.push_back(one_run(sd, 0.0f, -0.4f));
			seeds.push_back(one_run(sd, -0.5f, 0.5f));
		}
		seeds.push_back(two_run(static_cast<signed char>(-s_to),
			0.5f, N / 2, s_to, 0.2f, 0.9f));
		seeds.push_back(two_run(static_cast<signed char>(-s_to),
			0.9f, N / 3, s_to, 0.0f, 0.8f));
		seeds.push_back(two_run(s_to, 0.0f, N / 2,
			static_cast<signed char>(-s_to), 0.3f, 0.9f));
		{
			std::vector<WishRun> coast(1);
			coast[0].len = N;
			coast[0].side = 0;
			coast[0].ck.push_back(1.f);
			seeds.push_back(coast);
		}
		for (const std::vector<WishRun>& sd : seeds) {
			if (best.evals >= budget)
				break;
			ev(sd);
		}
		// DIAGNOSTIC SEED (the recovery ladder): injected losslessly.
		if (seed_side && seed_cosa && !seed_side->empty()
			&& best.evals < budget)
			ev_sched(*seed_side, *seed_cosa, nullptr);
		// ---- GUIDED + SEQUENTIAL MULTI-NODE SHOOTING (ruling
		// 2026-08-18): a GENERIC conditioning transform. Node
		// positions come from start/target geometry only - neutral
		// interpolation plus symmetric lateral offsets, both signs,
		// several magnitudes (nothing map- or tape-derived; bulges,
		// S-curves, direct lines all EMERGE). m is adaptive: m=0
		// single-target shots first, then m=1 with one free node.
		// Every frozen schedule enters only through open-loop replay;
		// generation passes are charged to the budget. ----
		{
			const int gbudget = budget / 3;
			const int fl0 = best.evals;
			std::vector<signed char> gsd;
			std::vector<float> gcs;
			const float rx = q.X - entry.pos.X;
			const float ry = q.Y - entry.pos.Y;
			const float rl = sqrtf(rx * rx + ry * ry);
			const float epy = rl > 1.f ? ry / rl : 0.f;
			const float epx = rl > 1.f ? rx / rl : 1.f;
			const float eqx = -epy, eqy = epx;
			auto shoot = [&](const std::vector<GuideTarget>& tgts,
				const GuideWeights& gw) {
				if (best.evals - fl0 >= gbudget
					|| best.evals >= budget)
					return;
				GuideWeights g2 = gw;
				// Second-stage exact rollouts only when the budget
				// affords them; their sim ticks are charged as
				// flight-equivalents.
				g2.exact_top = budget >= 600 ? 4 : 0;
				int ro = 0;
				GuidedShootSeq(entry, w, p, tgts, N, min_gap,
					entry_ctl, g2, &gsd, &gcs, &ro);
				const int cost = 1 + ro / (N > 0 ? N : 1);
				best.evals += cost;
				if (flights_counter)
					(*flights_counter) += cost;
				if (!gsd.empty())
					ev_sched(gsd, gcs, nullptr);
			};
			auto final_tgt = [&](float lo, float hi) {
				GuideTarget t2;
				t2.x = q.X;
				t2.y = q.Y;
				t2.th_lo = Steer::WrapPi(th_arr + lo);
				t2.th_hi = Steer::WrapPi(th_arr + hi);
				t2.use_theta = true;
				t2.until = N;
				return t2;
			};
			// Terminal-heading INTERVALS (generic coarse partition
			// around the closed-form arrival estimate; each interval
			// independently establishes its reachable class - the
			// numerical implementation of theta -> s_A*(theta)).
			const float iv[6][2] = { { -0.25f, 0.25f },
				{ 0.2f, 0.8f }, { -0.8f, -0.2f }, { 0.8f, 1.9f },
				{ -1.9f, -0.8f }, { 1.9f, -1.9f } };
			for (int ii = 0; ii < 3; ++ii) {
				std::vector<GuideTarget> tg(1,
					final_tgt(iv[ii][0], iv[ii][1]));
				GuideWeights gw;
				gw.wp = 2.f;
				gw.we = 0.3f;
				shoot(tg, gw);
				gw.wp = 6.f;
				gw.we = 0.05f;
				shoot(tg, gw);
			}
			// FULL-DOMAIN coverage (advisor: estimates may order,
			// never erase headings) - the far intervals each get a
			// shot so no terminal class silently disappears.
			for (int ii = 3; ii < 6; ++ii) {
				std::vector<GuideTarget> tg(1,
					final_tgt(iv[ii][0], iv[ii][1]));
				GuideWeights gw;
				gw.wp = 2.f;
				gw.we = 0.3f;
				shoot(tg, gw);
			}
			// m = 1: intervals x symmetric laterals; then node-TIME
			// adaptation around whichever shot advanced the best.
			const float lam0 = 0.45f;
			const float bmag[7] = { 0.f, 0.3f, -0.3f, 0.6f,
				-0.6f, 0.95f, -0.95f };
			float track_best = best.H;
			int win_ii = 0, win_bi = 0;
			auto node_at = [&](float lam, float bm) {
				GuideTarget n1;
				n1.x = entry.pos.X + lam * rx
					+ bm * rl * 0.5f * eqx;
				n1.y = entry.pos.Y + lam * ry
					+ bm * rl * 0.5f * eqy;
				n1.until = static_cast<int>(lam
					* static_cast<float>(N));
				return n1;
			};
			for (int ii = 0; ii < 3; ++ii)
				for (int bi2 = 0; bi2 < 7; ++bi2) {
					std::vector<GuideTarget> tg;
					tg.push_back(node_at(lam0, bmag[bi2]));
					tg.push_back(final_tgt(iv[ii][0],
						iv[ii][1]));
					GuideWeights gw;
					gw.wp = 2.f;
					gw.we = 0.3f;
					shoot(tg, gw);
					if (best.H > track_best + 1.f) {
						track_best = best.H;
						win_ii = ii;
						win_bi = bi2;
					}
				}
			// Node-time adaptation (a numerical resolution axis,
			// not admissibility): retry the winning node
			// earlier/later.
			{
				const float lams[4] = { 0.3f, 0.38f, 0.55f,
					0.65f };
				for (int li = 0; li < 4; ++li) {
					std::vector<GuideTarget> tg;
					tg.push_back(node_at(lams[li],
						bmag[win_bi]));
					tg.push_back(final_tgt(iv[win_ii][0],
						iv[win_ii][1]));
					GuideWeights gw;
					gw.wp = 2.f;
					gw.we = 0.3f;
					shoot(tg, gw);
				}
			}
			// m = 2 (generic): first node at the winner's corridor,
			// second free node closer to the target with symmetric
			// lateral variants - depart -> develop -> close, with
			// nothing about the shape encoded.
			{
				const float l2s[3] = { 0.f, 0.4f, -0.4f };
				for (int li = 0; li < 3; ++li) {
					float b2 = bmag[win_bi] + l2s[li];
					if (b2 > 0.95f) b2 = 0.95f;
					if (b2 < -0.95f) b2 = -0.95f;
					std::vector<GuideTarget> tg;
					tg.push_back(node_at(0.3f,
						bmag[win_bi] * 0.6f));
					tg.push_back(node_at(0.62f, b2));
					tg.push_back(final_tgt(iv[win_ii][0],
						iv[win_ii][1]));
					GuideWeights gw;
					gw.wp = 2.f;
					gw.we = 0.3f;
					shoot(tg, gw);
				}
			}
			// OUTCOME-SPACE GAUSS-NEWTON on the winning node
			// (advisor: attack the ill-conditioning in state
			// coordinates - finite-difference the replay residual
			// w.r.t. the node position, damped least-squares step).
			// Evaluations replay directly; the final polished
			// schedule is fed through ev_sched for crediting.
			if (best.ok && budget - best.evals > 20) {
				float nx = entry.pos.X + 0.45f * rx
					+ bmag[win_bi] * rl * 0.5f * eqx;
				float ny = entry.pos.Y + 0.45f * ry
					+ bmag[win_bi] * rl * 0.5f * eqy;
				const float th_mid = Steer::WrapPi(th_arr
					+ 0.5f * (iv[win_ii][0] + iv[win_ii][1]));
				auto eval_node = [&](float x2, float y2,
					float* r_out) {
					GuideTarget n1;
					n1.x = x2;
					n1.y = y2;
					n1.until = static_cast<int>(0.45f
						* static_cast<float>(N));
					std::vector<GuideTarget> tg;
					tg.push_back(n1);
					tg.push_back(final_tgt(iv[win_ii][0],
						iv[win_ii][1]));
					GuideWeights gw;
					gw.wp = 4.f;
					gw.we = 0.15f;
					gw.exact_top = 0;
					std::vector<signed char> s2;
					std::vector<float> c2;
					GuidedShootSeq(entry, w, p, tg, N, min_gap,
						entry_ctl, gw, &s2, &c2);
					best.evals += 2;
					if (flights_counter)
						(*flights_counter) += 2;
					if (s2.empty()) {
						r_out[0] = 1e4f;
						r_out[1] = 1e4f;
						r_out[2] = 1e4f;
						return;
					}
					vt.max_ticks = static_cast<int>(s2.size())
						+ 8;
					Air::Result ar = Air::FlyWishSchedule(entry,
						w, p, vt, g, s2, c2,
						static_cast<int>(s2.size()) + 8);
					if (!ar.hit || ar.dot >= 0.f
						|| ar.struck_brush >= 0) {
						r_out[0] = 1e4f;
						r_out[1] = 1e4f;
						r_out[2] = 1e4f;
						return;
					}
					ev_sched(s2, c2, nullptr);
					r_out[0] = ar.pos.X - q.X;
					r_out[1] = ar.pos.Y - q.Y;
					const float tha = atan2f(ar.v1.Y, ar.v1.X);
					r_out[2] = sqrtf(ThetaIntervalDist2(tha,
						Steer::WrapPi(th_mid - 0.15f),
						Steer::WrapPi(th_mid + 0.15f))) * 200.f;
				};
				float R0[3];
				eval_node(nx, ny, R0);
				for (int it = 0; it < 3
					&& budget - best.evals > 8; ++it) {
					if (fabsf(R0[0]) > 5e3f)
						break;
					const float dstep = 24.f;
					float R1[3], R2[3];
					eval_node(nx + dstep, ny, R1);
					eval_node(nx, ny + dstep, R2);
					float Jm[3][2];
					for (int r2 = 0; r2 < 3; ++r2) {
						Jm[r2][0] = (R1[r2] - R0[r2]) / dstep;
						Jm[r2][1] = (R2[r2] - R0[r2]) / dstep;
					}
					// (J^T J + lambda I) dx = -J^T R  (2x2 solve)
					float a11 = 0.5f, a12 = 0.f, a22 = 0.5f;
					float b1 = 0.f, b2 = 0.f;
					for (int r2 = 0; r2 < 3; ++r2) {
						a11 += Jm[r2][0] * Jm[r2][0];
						a12 += Jm[r2][0] * Jm[r2][1];
						a22 += Jm[r2][1] * Jm[r2][1];
						b1 -= Jm[r2][0] * R0[r2];
						b2 -= Jm[r2][1] * R0[r2];
					}
					const float det = a11 * a22 - a12 * a12;
					if (fabsf(det) < 1e-6f)
						break;
					float dx = (b1 * a22 - b2 * a12) / det;
					float dy = (b2 * a11 - b1 * a12) / det;
					const float dl = sqrtf(dx * dx + dy * dy);
					if (dl > 160.f) {
						dx *= 160.f / dl;
						dy *= 160.f / dl;
					}
					nx += dx;
					ny += dy;
					eval_node(nx, ny, R0);
				}
			}
		}
		// deepen(): local refinement of one elite (knot/length moves,
		// the resolution ladder, reversal insertion).
		auto deepen = [&](BinElite& pe, float kstep, int lstep) {
			if (!pe.has)
				return;
			bool moved = true;
			int guard = 0;
			while (moved && best.evals < budget && guard++ < 8) {
				moved = false;
				for (size_t j = 0; j < pe.runs.size(); ++j) {
					for (size_t ki = 0; ki < pe.runs[j].ck.size();
						++ki)
						for (int sg = -1; sg <= 1; sg += 2) {
							if (best.evals >= budget)
								return;
							std::vector<WishRun> t2 = pe.runs;
							float nv = t2[j].ck[ki] + kstep
								* static_cast<float>(sg);
							if (nv > 1.f) nv = 1.f;
							if (nv < -1.f) nv = -1.f;
							t2[j].ck[ki] = nv;
							const float sc = ev(t2);
							if (sc > pe.sc) {
								pe.sc = sc;
								pe.runs = t2;
								moved = true;
							}
						}
					for (int sg = -1; sg <= 1; sg += 2) {
						if (best.evals >= budget)
							return;
						std::vector<WishRun> t2 = pe.runs;
						const int dl = lstep * sg;
						if (j + 1 < t2.size()) {
							if (t2[j].len + dl < min_gap
								|| t2[j + 1].len - dl < min_gap)
								continue;
							t2[j].len += dl;
							t2[j + 1].len -= dl;
						} else {
							if (t2[j].len + dl < min_gap)
								continue;
							t2[j].len += dl;
						}
						const float sc = ev(t2);
						if (sc > pe.sc) {
							pe.sc = sc;
							pe.runs = t2;
							moved = true;
						}
					}
				}
				if (moved)
					continue;
				size_t lj = 0;
				for (size_t j = 1; j < pe.runs.size(); ++j)
					if (pe.runs[j].len > pe.runs[lj].len)
						lj = j;
				// The resolution ladder: refine the longest run's
				// cosa knots toward per-tick.
				if (static_cast<int>(pe.runs[lj].ck.size()) < 17
					&& static_cast<int>(pe.runs[lj].ck.size())
						< pe.runs[lj].len) {
					std::vector<WishRun> t2 = pe.runs;
					std::vector<float> nk;
					const std::vector<float>& ok2 = t2[lj].ck;
					for (size_t i = 0; i + 1 < ok2.size(); ++i) {
						nk.push_back(ok2[i]);
						nk.push_back(0.5f * (ok2[i]
							+ ok2[i + 1]));
					}
					nk.push_back(ok2.back());
					if (nk.size() < 2) {
						nk = ok2;
						nk.push_back(ok2.back());
					}
					t2[lj].ck = nk;
					const float sc = ev(t2);
					if (sc >= pe.sc) {
						pe.sc = sc;
						pe.runs = t2;
						moved = true;
					}
				}
				if (!moved && pe.runs.size() < 7
					&& pe.runs[lj].len >= 2 * min_gap
					&& best.evals < budget) {
					std::vector<WishRun> t2 = pe.runs;
					WishRun tail;
					tail.len = t2[lj].len / 2;
					t2[lj].len -= tail.len;
					tail.side = static_cast<signed char>(
						-t2[lj].side);
					tail.ck.push_back(t2[lj].ck.back() * 0.6f);
					t2.insert(t2.begin()
						+ static_cast<long long>(lj) + 1, tail);
					const float sc = ev(t2);
					if (sc > pe.sc) {
						pe.sc = sc;
						pe.runs = t2;
						moved = true;
					}
				}
			}
		};
		// FRONTIER REFINEMENT: full deepening for the SCOUT POOL and
		// the top bins each round (bins own diversity; scouts own
		// basin entry), then a deterministic perturb-restart of the
		// current leaders. Dry counting is on the GLOBAL best; the
		// caller's convergence measure remains the sandwich gap.
		unsigned rng = 0x9e3779b9u
			^ static_cast<unsigned>(face_idx * 7919)
			^ static_cast<unsigned>(static_cast<int>(q.X) * 131)
			^ static_cast<unsigned>(static_cast<int>(q.Y));
		auto frand = [&]() {
			rng = rng * 1664525u + 1013904223u;
			return static_cast<float>((rng >> 8) & 0xFFFF)
				/ 65535.f * 2.f - 1.f;
		};
		auto perturb = [&](const BinElite& src) {
			BinElite pe = src;
			for (WishRun& r : pe.runs) {
				for (float& c : r.ck) {
					c += 0.25f * frand();
					if (c > 1.f) c = 1.f;
					if (c < -1.f) c = -1.f;
				}
				const int dl = static_cast<int>(4.f * frand());
				if (r.len + dl >= min_gap)
					r.len += dl;
			}
			pe.sc = ev(pe.runs);
			deepen(pe, 0.12f, 2);
		};
		const float ksteps[3] = { 0.3f, 0.12f, 0.05f };
		const int lsteps[3] = { 6, 2, 2 };
		int dry = 0;
		float last_best = -1e30f;
		while (best.evals < budget && dry < 4) {
			// Scouts always get the full schedule.
			for (int rd = 0; rd < 3 && best.evals < budget; ++rd)
				for (int si2 = 0; si2 < 3; ++si2)
					deepen(scouts[static_cast<size_t>(si2)],
						ksteps[rd], lsteps[rd]);
			// Then the top bins by H + the softest-|dot| bin.
			std::vector<int> order;
			for (int b2 = 0; b2 < kBins; ++b2)
				if (bins[static_cast<size_t>(b2)].has)
					order.push_back(b2);
			if (!order.empty()) {
				std::sort(order.begin(), order.end(),
					[&](int a2, int b3) {
						return bins[static_cast<size_t>(a2)].H
							> bins[static_cast<size_t>(b3)].H;
					});
				int soft = order[0];
				for (int b2 : order)
					if (fabsf(bins[static_cast<size_t>(b2)].dot)
						< fabsf(bins[static_cast<size_t>(soft)]
							.dot))
						soft = b2;
				std::vector<int> work(order.begin(),
					order.begin() + static_cast<long long>(
						order.size() > 4 ? 4 : order.size()));
				bool have_soft = false;
				for (int b2 : work)
					if (b2 == soft)
						have_soft = true;
				if (!have_soft)
					work.push_back(soft);
				for (int rd = 0; rd < 3 && best.evals < budget;
					++rd)
					for (int b2 : work) {
						if (best.evals >= budget)
							break;
						deepen(bins[static_cast<size_t>(b2)],
							ksteps[rd], lsteps[rd]);
					}
				if (best.evals < budget)
					perturb(bins[static_cast<size_t>(work[0])]);
			}
			if (best.evals < budget && scouts[0].has)
				perturb(scouts[0]);
			if (best.H <= last_best + 1e-3f)
				dry++;
			else
				dry = 0;
			last_best = best.H;
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
			int spline_horizon, int source,
			const std::vector<signed char>* wish_side = nullptr,
			const std::vector<float>* wish_cosa = nullptr) {
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
				if (wish_side) {
					r.wside = *wish_side;
					r.wcosa = *wish_cosa;
					r.knots.clear();
					r.horizon = spline_horizon;
				} else if (spline_knots) {
					r.knots = *spline_knots;
					r.wside.clear();
					r.wcosa.clear();
					r.horizon = spline_horizon;
				} else {
					r.knots.clear();
					r.wside.clear();
					r.wcosa.clear();
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
			// anywhere raises the field's lower bound (source=1),
			// witnessed in the canonical wish basis.
			auto ref_credit = [&](const Air::Result& ar,
				const std::vector<signed char>& wsd,
				const std::vector<float>& wcs, int hz) {
				credit(ar, 0.f, 0.f, 0, ar.tick, nullptr, hz, 1,
					&wsd, &wcs);
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
					&m.flights, face.zmin, o.entry_ctl,
					ref_credit);
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
