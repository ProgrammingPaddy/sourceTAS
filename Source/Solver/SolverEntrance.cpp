#include "SolverEntrance.h"

#include <float.h>
#include <math.h>

#include <algorithm>

#include "SolverAir.h"
#include "SolverBoard.h"
#include "SolverEnvelope.h"
#include "SolverField.h"
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
			const float w = lk.TurnRad(Strafe::kPerp, 1.f);
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
					h = Steer::WrapPi(h + lk.TurnRad(Strafe::kPerp, 1.f)
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
							// REMAINING-RESIDUAL, CURVATURE-AWARE
							// (advisor 2026-08-19). The old form asked
							// "can a STRAIGHT line still reach the
							// final target, and is there enough
							// free-turn budget for the heading?" - which
							// is structurally misleading for exactly the
							// class that fails: long sustained-turn
							// flights. Instead, put the predicted state
							// at the origin with velocity along +x and
							// the target at (x, y); the circle tangent
							// to the current heading through the target
							// has curvature and sweep
							//     kappa = 2y / (x^2 + y^2)
							//     phi   = 2 * atan2(y, x)
							// giving required turn, arc length and the
							// implied terminal tangent in closed form.
							// Advisory guidance only - never a bound,
							// and it encodes no map knowledge.
							const GuideTarget& fin =
								targets.back();
							const int rem2 = N - t_end;
							if (rem2 > 0) {
								const float ddx = fin.x - px;
								const float ddy = fin.y - py;
								const float ch = cosf(hh);
								const float sh2 = sinf(hh);
								const float lx = ddx * ch + ddy * sh2;
								const float ly = -ddx * sh2 + ddy * ch;
								// Sweep and BOUNDED arc length via the shared
								// pure helpers, so the property gate
								// (`airprops`) tests the same code that runs.
								const float phi = ToGoSweep(lx, ly);
								const float arc = ToGoArcLen(lx, ly);
								Strafe::TickLaw lr = Strafe::Law(
									p, ss, 1.f, s.ducked);
								const float tfree = lr.TurnRad(
									Strafe::kPerp, 1.f)
									* static_cast<float>(rem2);
								// Braking turns buy more rotation per
								// tick than the free rate (at a speed
								// cost the optimizer owns), so the
								// admissible turn budget is the larger.
								const float tbrake = Field::
									BrakeTurnPeak(ss, p, s.ducked)
									* static_cast<float>(rem2);
								const float tcap = tfree > tbrake
									? tfree : tbrake;
								const float tshort = fabsf(phi) - tcap;
								if (tshort > 0.f)
									J += 50.f * tshort * tshort;
								// Distance debt measured along the ARC
								// the closure actually requires.
								const float reach = Envelope::DMax(
									ss, rem2, p);
								const float rshort = arc - reach;
								if (rshort > 0.f)
									J += 4e-4f * rshort * rshort;
								// Implied terminal tangent vs the
								// requested interval: closing on this
								// circle arrives at heading hh + phi.
								if (fin.use_theta)
									J += 25.f * ThetaIntervalDist2(
										Steer::WrapPi(hh + phi),
										fin.th_lo, fin.th_hi);
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
					// THE ACTION VOCABULARY, SPEED-NORMALIZED
					// (wishparity 2026-08-19). The law's ACTIVE band
					// is true cos in [0, cap/v] - only 0.033 wide at
					// v=900 - so a fixed grid over [-1,1] cannot
					// resolve it: the old cgrid was, in true units,
					// a spread of BRAKING turns plus one max-gain
					// point, which is why sustained-gain arcs were
					// unreachable. Candidates are therefore generated
					// in TRUE units as fractions of the band (gain
					// side: max-free-turn -> nearly straight) plus
					// absolute braking values (turn harder than free,
					// pay speed), then converted to STORED units at
					// the boundary. No fitted constants: the band
					// edge is the law's own cap/v.
					std::vector<Cand2> cands;
					Strafe::TickLaw law0 = Strafe::Law(p, s2d, 1.f,
						s.ducked);
					const float band = law0.BandCos();
					static const float gfrac[5] = { 0.f, 0.25f, 0.5f,
						0.75f, 0.95f };
					static const float bcos[4] = { -0.15f, -0.35f,
						-0.6f, -0.85f };
					for (int si = -1; si <= 1; ++si) {
						if (si != 0 && cur != 0 && si != cur
							&& age < min_gap)
							continue;
						if (si == 0) {
							cands.push_back({ 0, 1.f, 1.f, 0.f });
							continue;
						}
						for (int gi = 0; gi < 5; ++gi) {
							const float st = Strafe::ToStoredWishCos(
								Strafe::TrueWishCos(gfrac[gi]
									* band));
							cands.push_back({ si, st, st, 0.f });
						}
						for (int bi3 = 0; bi3 < 4; ++bi3) {
							const float st = Strafe::ToStoredWishCos(
								Strafe::TrueWishCos(bcos[bi3]));
							cands.push_back({ si, st, st, 0.f });
						}
						// WITHIN-SIDE evolving profiles (a -> b): the
						// dwell law fixes the SIDE for six ticks,
						// never the wish angle - braking turns may
						// need the angle to evolve inside one side.
						// Expressed as (true a -> true b) pairs.
						static const float pg[4][2] = {
							{ -0.6f, 0.f }, { 0.f, -0.6f },
							{ 0.5f, -0.3f }, { -0.3f, 0.9f } };
						for (int pi3 = 0; pi3 < 4; ++pi3) {
							const float a2 = pg[pi3][0] < 0.f
								? pg[pi3][0]
								: pg[pi3][0] * band;
							const float b2 = pg[pi3][1] < 0.f
								? pg[pi3][1]
								: pg[pi3][1] * band;
							cands.push_back({ si,
								Strafe::ToStoredWishCos(
									Strafe::TrueWishCos(a2)),
								Strafe::ToStoredWishCos(
									Strafe::TrueWishCos(b2)),
								0.f });
						}
					}
					// Stage 1: closed-form law rollout ranks all.
					// THE CONVENTION BRIDGE, BOTH HALVES (measured by
					// `wishparity`, 2026-08-19: 168 rows, rotation
					// sign -side in 98/98 turning rows). The stored
					// basis realizes its wish at wh + pi, so the true
					// cosine is the negation AND the realized rotation
					// is the negated side. Session 11 fixed only the
					// cosine and left every modelled turn pointing the
					// wrong way - the guided shooter was ranking the
					// side that steers AWAY from its target.
					for (Cand2& cd : cands) {
						float hh = h, ss = s2d;
						float px = s.pos.X, py = s.pos.Y;
						float sac = 0.f;
						const int rot = Strafe::ToTrueRotSide(cd.si);
						for (int j = 0; j < lk; ++j) {
							const float c = cd.si == 0 ? 1.f
								: cd.a + (cd.b - cd.a)
								* (lk > 1 ? static_cast<float>(j)
									/ static_cast<float>(lk - 1)
									: 0.f);
							if (cd.si != 0) {
								Strafe::TickLaw law = Strafe::Law(
									p, ss, 1.f, s.ducked);
								const Strafe::TrueWishCos ce =
									Strafe::ToTrueWishCos(c);
								const float s2c = 1.f - ce.v * ce.v;
								const float tr = law.TurnRad(ce,
									s2c > 0.f ? sqrtf(s2c) : 0.f);
								hh = Steer::WrapPi(hh
									+ (rot > 0 ? tr : -tr));
								const float nv2 = law.NewSpeed2(
									ce);
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
	                   const std::vector<float>* seed_cosa,
	                   const RefTune* tune) {
		RefResult best;
		// BOUNDARY MODE (advisor 2026-08-19): tune->bnd_theta swaps
		// the win condition from "clean face strike near q" to "state
		// at exactly tick N within radius of q with terminal heading
		// inside the interval" - the free-air (S0, Q, T, theta)
		// problem. All search machinery runs unchanged on the score.
		const bool bnd = tune && tune->bnd_theta;
		const int tune_m = tune ? tune->shoot_m : 2;
		// The anytime SCHEDULER path (immutable config, never
		// budget-derived). While 0, the legacy phased path runs and
		// remains the regression baseline.
		const int  sched_ver = tune ? tune->scheduler : 0;
		const bool use_sched = sched_ver != 0;
		// v1 = SERVICE DISCIPLINE. Importance still comes only from
		// U_D - L*; v1 adds deterministic weighted-fair service so
		// raw max-gap cannot monopolise the stream, plus within-domain
		// method yielding. No budget enters either.
		const bool fair_sched = sched_ver >= 2;
		// Coverage rollout depth: 0 = ScoutCheap. Immutable config,
		// never budget-derived.
		const int  cover_top = tune ? tune->cover_top : 0;
		const bool legacy_seq = !use_sched;
		if (!bnd && (face_idx < 0
			|| face_idx >= static_cast<int>(g.faces.size())))
			return best;
		const Route::Face* facep = bnd ? nullptr
			: &g.faces[static_cast<size_t>(face_idx)];
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
		vt.face = bnd ? -1 : face_idx;
		vt.dot_cap = 3000.f;
		vt.aim = q;
		// Arrival-heading reference + the heading-interval table,
		// hoisted above ev_sched so every strike can be binned into
		// its interval (per-interval {L, hits, shots} coverage).
		const float bear = atan2f(q.Y - entry.pos.Y,
			q.X - entry.pos.X);
		float th_arr = bear;
		if (bnd) {
			th_arr = Steer::WrapPi(tune->bnd_theta[0] + 0.5f
				* Steer::WrapPi(tune->bnd_theta[1]
					- tune->bnd_theta[0]));
		} else {
			const float hn = sqrtf(facep->n.X * facep->n.X
				+ facep->n.Y * facep->n.Y);
			const float sa = Envelope::SMax(s0, N, p);
			const float va = Envelope::VzAfter(entry.vel.Z, N, p,
				gs);
			if (sa * hn > 1e-4f) {
				float cphi = -va * facep->n.Z / (sa * hn);
				if (cphi > 1.f) cphi = 1.f;
				if (cphi < -1.f) cphi = -1.f;
				const float po = acosf(cphi);
				const float paz = atan2f(facep->n.Y, facep->n.X);
				const float ca = Steer::WrapPi(paz + po);
				const float cb = Steer::WrapPi(paz - po);
				th_arr = fabsf(Steer::WrapPi(ca - bear))
					<= fabsf(Steer::WrapPi(cb - bear)) ? ca : cb;
			}
		}
		best.iv_ref = th_arr;
		// Terminal-heading INTERVALS (full-circle conservative cover
		// about th_arr; estimates order search, never erase). Boundary
		// mode collapses row 0 to the GIVEN interval.
		float ivt[6][2] = { { -0.25f, 0.25f }, { 0.2f, 0.8f },
			{ -0.8f, -0.2f }, { 0.8f, 1.9f }, { -1.9f, -0.8f },
			{ 1.9f, -1.9f } };
		if (bnd) {
			ivt[0][0] = Steer::WrapPi(tune->bnd_theta[0] - th_arr);
			ivt[0][1] = Steer::WrapPi(tune->bnd_theta[1] - th_arr);
		}
		std::vector<signed char> sbuf;
		std::vector<float> cbuf;
		// ---- ACTIVE TOLERANCE CONTINUATION (advisor 2026-08-19) ----
		// The measured cliff (~30/48 within 16u, ZERO within 8u) was
		// partly solver-induced: the score discarded the positional
		// residual the instant it crossed the acceptance radius, so a
		// candidate had every reason to reach 15.9u and then spend the
		// rest of the budget on speed. `tol` is the ACTIVE tolerance
		// the value phase switches at; it is tightened between
		// refinement rounds down to `prec` so the residual gradient
		// never flattens:
		//     minimise residual until feasible at tol,
		//     then maximise value subject to residual <= tol.
		// CREDITING IS SEPARATE AND STAYS PINNED TO `radius`: best.H,
		// the bins, iv_L and the tangent slot keep their original
		// region contract, so no external consumer (fieldexact
		// saturation, humanexact's floor, the bank) silently changes
		// meaning, and continuation can only ADD tight witnesses.
		float tol = radius;
		// Last evaluation's raw outcome (published for deepen) and the
		// best residual over EVERY outcome class (always live).
		float ev_res = 1e30f, ev_val = -1e30f;
		float ev_rp = 1e30f, ev_rth = 0.f;
		float best_any_res = 1e30f;
		const float prec = tune && tune->precision > 0.f
			? tune->precision : radius;
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
			// RAW OUTCOME kept alongside the cached key so tightening
			// the active tolerance can re-key every elite for free
			// instead of re-flying it (the stale-cache hazard the
			// plateau audit flagged). rp/rth are kept SEPARATELY from
			// the ordering residual because feasibility is a
			// PREDICATE on both channels, not a threshold on their
			// weighted sum (see key_of).
			float res = 1e30f;   // monotone residual (ordering only)
			float val = -1e30f;  // value inside the tolerance
			float rp = 1e30f;    // positional residual
			float rth = 0.f;     // heading residual (radians)
		};
		std::vector<BinElite> bins(kBins);
		std::vector<BinElite> scouts(3);
		const float kPi2 = 3.14159265f;
		// THE SEARCH KEY. One monotone residual channel all the way in
		// (the audit found three incommensurate quantities sharing the
		// negative band, so a converging candidate's score could DROP
		// before it jumped - a barrier immediately before the success
		// set), then a value tier that engages only inside the active
		// tolerance. Tiers: clean strike < no-strike < contact-assisted
		// rejection, so any strike outranks any miss.
		// Heading feasibility width: the terminal heading must lie
		// INSIDE the requested interval (ThetaIntervalDist2 returns 0
		// there), so the admissible heading residual is ~0, not a
		// tolerance-scaled slack.
		const float kThFeas = 1e-4f;
		// FEASIBILITY IS A PREDICATE ON BOTH CHANNELS. Keying the value
		// tier on the weighted sum (rp + kThetaW*rth <= tol) admits a
		// strict SUPERSET of the success set - at tol=8 it would accept
		// rth up to 0.032 rad while success needs 1e-4 - which
		// re-creates the very plateau this continuation exists to
		// remove, relocated from the position channel into the heading
		// channel. The weighted sum stays as the ORDERING residual
		// (it gives a usable descent direction); the tier switch uses
		// the predicate.
		auto key_of = [&](float res, float val, float rp, float rth) {
			const bool feas = rp <= tol && rth <= kThFeas;
			return feas ? 1e7f + val : -res;
		};
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
			if (bnd && total != N)
				return -1e9f;   // T is FIXED in boundary mode
			if (!sched_legal(sd))
				return -1e9f;
			vt.max_ticks = bnd ? total : total + 8;
			best.evals++;
			if (flights_counter)
				(*flights_counter)++;
			Air::Result ar = Air::FlyWishSchedule(entry, w, p, vt,
				g, sd, cs, bnd ? total : total + 8);
			const Vec3 epos = bnd ? ar.end_pos
				: (ar.hit ? ar.pos
					: (ar.miss_dist < 1e8f ? ar.closest
						: ar.end_pos));
			// resid = ONE monotone channel (tier offsets keep any
			// strike above any miss above any clean-air rejection);
			// val = what the value phase maximizes inside tol.
			float resid = 1e9f;
			float val = -1e30f;
			float res_p = 1e30f;   // positional channel
			float res_th = 0.f;    // heading channel
			bool strike = false;      // in-RADIUS (crediting contract)
			float H = -1e30f;
			float th_term = 0.f;
			if (bnd) {
				// Tick-T boundary residual: any contact/grounding is
				// a hard failure; otherwise score the terminal state
				// against (Q, theta-interval).
				if (ar.struck_brush >= 0 || ar.grounded) {
					resid = 1e6f + (ar.miss_dist < 1e8f
						? ar.miss_dist : 1e8f);
				} else {
					const float rp = Len(ar.end_pos - q);
					const Vec3& vT = ar.end_state.vel;
					const float sT = Len2D(vT);
					th_term = sT > 1.f ? atan2f(vT.Y, vT.X) : 0.f;
					const float rth = sqrtf(ThetaIntervalDist2(
						th_term,
						Steer::WrapPi(th_arr + ivt[0][0]),
						Steer::WrapPi(th_arr + ivt[0][1])));
					// One weight for the heading/position trade,
					// shared with the Gauss-Newton residual so the
					// optimizer descends the metric the search ranks.
					resid = rp + kThetaW * rth;
					res_p = rp;
					res_th = rth;
					val = sT;   // the s_A* quantity
					if (resid < best.bnd_rp + kThetaW * best.bnd_rth) {
						best.bnd_rp = rp;
						best.bnd_rth = rth;
						best.bnd_s = sT;
					}
					// Boundary mode gets the same exactness ladder as
					// face mode (it had none - the two modes reported
					// the ladder through different fields).
					if (rp < best.strike_rmin) {
						best.strike_rmin = rp;
						best.strike_rmin_th = th_term;
					}
					for (int r2 = 0; r2 < 6; ++r2)
						if (resid <= kLadder[r2]
							&& sT > best.rung_E[r2]) {
							best.rung_E[r2] = sT;
							best.rung_side[r2] = sd;
							best.rung_cosa[r2] = cs;
							best.rung_res[r2] = resid;
						}
					if (rp <= radius && rth <= 1e-4f) {
						strike = true;
						H = sT;
					}
				}
			} else if (!ar.hit || ar.dot >= 0.f) {
				resid = 1e5f + (ar.miss_dist < 1e8f
					? ar.miss_dist : 1e8f);
			} else if (ar.struck_brush >= 0) {
				best.contact_assisted++;
				resid = 1e6f + Len(ar.pos - q);
			} else {
				if (on_strike)
					on_strike(ar, sd, cs, total + 8);
				const float d = Len(ar.pos - q);
				th_term = atan2f(ar.v1.Y, ar.v1.X);
				const float Hs = Dot(ar.end_state.vel,
					ar.end_state.vel) + 2.f * p.gravity * gs
					* (ar.pos.Z - zmin);
				resid = d;
				res_p = d;
				res_th = 0.f;
				val = Hs;
				// Exactness ladder: EVERY clean strike reports its
				// distance to q; per-rung best value AND ITS WITNESS
				// (a rung you cannot hand back is only observable,
				// never usable - advisor 2026-08-19).
				if (d < best.strike_rmin) {
					best.strike_rmin = d;
					best.strike_rmin_th = th_term;
				}
				for (int r2 = 0; r2 < 6; ++r2)
					if (d <= kLadder[r2] && Hs > best.rung_E[r2]) {
						best.rung_E[r2] = Hs;
						best.rung_side[r2] = sd;
						best.rung_cosa[r2] = cs;
						best.rung_res[r2] = d;
					}
				if (d <= radius) {
					strike = true;
					H = Hs;
				}
			}
			// tangent_mode keeps its own admission band explicitly
			// rather than through a score constant, so a lexicographic
			// key cannot let a hard strike poison best.H.
			const bool tan_block = tangent_mode && !bnd
				&& -ar.dot > kTanEps;
			const float sc = tan_block ? -resid
				: key_of(resid, val, res_p, res_th);
			// Published so deepen()/perturb() can keep an accepted
			// elite's RAW outcome in step with its cached key - the
			// key is recomputed on every tolerance advance, and a
			// stale raw triple would demote a tight witness as if it
			// were the loose schedule it replaced.
			ev_res = resid;
			ev_val = val;
			ev_rp = res_p;
			ev_rth = res_th;
			// A MISS RESIDUAL that is always live: without it,
			// shot_key()/progress() read strike_rmin (never written
			// when no clean strike exists at all), so the node winner
			// stayed pinned and dry hit 4 within a few rounds on
			// exactly the hardest face-mode cases.
			if (resid < best_any_res)
				best_any_res = resid;
			// Scout pool: endpoint-diverse top-3 by raw score. The
			// dedupe radius SCALES WITH THE ACTIVE TOLERANCE: a fixed
			// 32u was coarser than every rung below 32u, so all
			// candidates converging on q collapsed into one slot and
			// the pool could not hold a diverse set at 8u/4u/1u.
			{
				// Finer as the tolerance tightens, but never COARSER than
			// the original 32u: the formula must not loosen endpoint
			// diversity for the coarse callers (Field::Build at grid
			// 32 would have gone to 115u with only 3 slots).
				const float dedup = ScoutDedupe(tol);
				int slot = -1;
				for (int i2 = 0; i2 < 3; ++i2)
					if (scouts[static_cast<size_t>(i2)].has
						&& Len(scouts[static_cast<size_t>(i2)]
							.epos - epos) < dedup) {
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
					sl.res = resid;
					sl.val = val;
					sl.rp = res_p;
					sl.rth = res_th;
					sl.epos = epos;
					sl.runs = src_runs ? *src_runs
						: runs_from_sched(sd, cs);
				}
			}
			if (strike) {
				const float th = th_term;
				int bi = static_cast<int>((th + kPi2)
					/ (2.f * kPi2) * static_cast<float>(kBins));
				if (bi < 0) bi = 0;
				if (bi >= kBins) bi = kBins - 1;
				BinElite& be = bins[static_cast<size_t>(bi)];
				if (sc > be.sc || !be.has) {
					be.has = true;
					be.sc = sc;
					be.H = H;
					be.res = resid;
					be.val = val;
					be.rp = res_p;
					be.rth = res_th;
					be.dot = ar.dot;
					be.epos = epos;
					be.runs = src_runs ? *src_runs
						: runs_from_sched(sd, cs);
				}
				// Per-interval witness coverage {L_i, hits}: each
				// in-radius witness credits every heading interval
				// containing its terminal heading.
				const float rel = Steer::WrapPi(th - th_arr);
				for (int i3 = 0; i3 < 6; ++i3)
					if (ThetaIntervalDist2(rel, ivt[i3][0],
						ivt[i3][1]) <= 0.f) {
						best.iv_hits[i3]++;
						if (H > best.iv_L[i3]) {
							best.iv_L[i3] = H;
							// The lane WITNESS travels with the value
							// (BoardTransition frontier, ExitField 0).
							best.iv_side[i3] = sd;
							best.iv_cosa[i3] = cs;
							best.iv_res[i3] = res_p;
							best.iv_th[i3] = th;
						}
					}
				// THE CREDITING GATE. Keyed on the in-radius `strike`
				// bool plus the explicit tangent condition - never on
				// "sc >= 1e7", which silently dropped any strike whose
				// H was negative (reachable whenever a caller's zmin
				// sits above the contact z, e.g. airrec's zmin = 0)
				// and which a lexicographic key would have broken
				// outright.
				if (!tan_block && H > best.H) {
					best.ok = true;
					best.H = H;
					best.flight = ar;
					best.wside = sd;
					best.wcosa = cs;
					best.horizon = bnd ? total : total + 8;
				}
				if (!bnd && -ar.dot <= kTanEps
					&& H > best.tan_H) {
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
		// ---- SEED GRID (deterministic; th_arr hoisted above) ----
		// s_to = the STORED side that rotates TOWARD th_arr. The
		// wanted rotation sign is sign(th_arr - h0); the stored label
		// is its mirror (wishparity 2026-08-19). Before this the seed
		// grid's "toward" side turned away - masked for the one_run
		// family (both signs are enumerated) but not for the composite
		// two_run seeds, whose turn-then-turn ordering was reversed.
		const signed char s_to = static_cast<signed char>(
			Strafe::ToStoredSide(
				Steer::WrapPi(th_arr - h0) >= 0.f ? 1 : -1));
		// Seed cosa values are generated in TRUE units and converted,
		// so "gain" seeds actually gain: the active band is only
		// cap/v wide (0.033 at v=900), and the previous literal stored
		// values (0.9, 0.4, -0.4 ...) were, in true units, a set of
		// hard brakes and inert ticks with no sustained-gain member.
		const float band0 = Strafe::Law(p, s0, 1.f,
			entry.ducked).BandCos();
		auto st_gain = [&](float frac) {
			return Strafe::ToStoredWishCos(
				Strafe::TrueWishCos(frac * band0));
		};
		auto st_brake = [&](float true_c) {
			return Strafe::ToStoredWishCos(
				Strafe::TrueWishCos(true_c));
		};
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
			seeds.push_back(one_run(sd, st_gain(0.f), st_gain(0.f)));
			seeds.push_back(one_run(sd, st_gain(0.f), st_gain(0.9f)));
			seeds.push_back(one_run(sd, st_gain(0.9f), st_gain(0.f)));
			seeds.push_back(one_run(sd, st_brake(-0.3f), st_gain(0.f)));
			seeds.push_back(one_run(sd, st_brake(-0.7f), st_brake(-0.2f)));
		}
		seeds.push_back(two_run(static_cast<signed char>(-s_to),
			st_brake(-0.3f), N / 2, s_to, st_gain(0.5f), st_gain(0.f)));
		seeds.push_back(two_run(static_cast<signed char>(-s_to),
			st_brake(-0.7f), N / 3, s_to, st_gain(0.f), st_gain(0.8f)));
		seeds.push_back(two_run(s_to, st_gain(0.f), N / 2,
			static_cast<signed char>(-s_to), st_brake(-0.4f), st_gain(0.f)));
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
			// PREFIX CONTAINMENT (advisor's monotone-compute law): the
			// guided phase is a FIXED sequence of shots, not a
			// fraction of the budget, so a larger budget flies the
			// same first shots and can only ever ADD to what a
			// smaller one found. `budget/3` and a 600-eval rollout
			// threshold both made the action SEQUENCE depend on the
			// total, which is how L(1200) came out below L(300).
			// PREFIX CONTAINMENT, precisely stated: run(B1) must be a
			// PREFIX of run(B2). Truncating the shot sequence at a
			// budget-derived point preserves that (the larger budget
			// flies the same shots and more). What BROKE it was
			// exact_top flipping at 600 evals, which changed the
			// CONTENT of every shot, so the two runs flew different
			// schedules entirely - measured as L(1200) < L(300).
			// exact_top is therefore fixed, and the sequence keeps a
			// truncation point so small budgets still reach the
			// refinement rounds (a fixed 40-shot prefix costs ~1000
			// eval-equivalents and starved every 600-eval query:
			// airsuite fell 48/48 -> 38/48).
			const int kGuidedShots = 40;
			const int gbudget = budget / 3;
			int gshots = 0;
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
				const GuideWeights& gw, int iv_i, int etop = 4) {
				// The legacy phase self-limits; the SCHEDULER owns
				// ordering, so on that path only the runner may stop
				// the stream (a self-limit here made scheduled
				// actions silent no-ops - caught by S1).
				if (legacy_seq && (gshots >= kGuidedShots
					|| best.evals - fl0 >= gbudget))
					return;
				if (best.evals >= budget)
					return;
				gshots++;
				if (iv_i >= 0 && iv_i < 6)
					best.iv_shots[iv_i]++;
				GuideWeights g2 = gw;
				// Second-stage exact rollouts always run so the
				// action sequence is budget-independent; their sim
				// ticks are charged as flight-equivalents.
				// Fixed by the ACTION, never by the budget. Escalation keeps
				// the full four-way exact ranking; the mandatory coverage
				// action passes 0 and runs closed-form only, which is where
				// almost all of a shot's cost lives.
				g2.exact_top = etop;
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
			// Terminal-heading INTERVALS: the table (ivt) is hoisted
			// to function scope so ev_sched bins witnesses into it;
			// each interval independently establishes its reachable
			// class - the numerical implementation of
			// theta -> s_A*(theta). Boundary mode has ONE given
			// interval (row 0).
			const int n_near = bnd ? 1 : 3;
			// ---- GN single-node evaluator, hoisted so the SCHEDULER
			// can run ONE Gauss-Newton iteration as one atomic action
			// (the contract: an iteration, never "run GN until done").
			auto gn_eval1 = [&](int ivx, float x2, float y2,
				float* r_out) {
				GuideTarget n1;
				n1.x = x2;
				n1.y = y2;
				n1.until = static_cast<int>(0.45f
					* static_cast<float>(N));
				std::vector<GuideTarget> tg;
				tg.push_back(n1);
				tg.push_back(final_tgt(ivt[ivx][0], ivt[ivx][1]));
				GuideWeights gw;
				gw.wp = 4.f;
				gw.we = 0.15f;
				gw.exact_top = 0;
				std::vector<signed char> s2;
				std::vector<float> c2;
				GuidedShootSeq(entry, w, p, tg, N, min_gap, entry_ctl,
					gw, &s2, &c2);
				best.evals += 2;
				if (flights_counter)
					(*flights_counter) += 2;
				r_out[0] = 1e4f;
				r_out[1] = 1e4f;
				r_out[2] = 1e4f;
				if (s2.empty())
					return;
				const int hz2 = bnd ? static_cast<int>(s2.size())
					: static_cast<int>(s2.size()) + 8;
				vt.max_ticks = hz2;
				Air::Result ar = Air::FlyWishSchedule(entry, w, p, vt,
					g, s2, c2, hz2);
				float px, py, tha;
				if (bnd) {
					if (ar.struck_brush >= 0 || ar.grounded
						|| static_cast<int>(s2.size()) != N)
						return;
					px = ar.end_pos.X;
					py = ar.end_pos.Y;
					const Vec3& vT = ar.end_state.vel;
					tha = Len2D(vT) > 1.f ? atan2f(vT.Y, vT.X) : 0.f;
				} else {
					if (!ar.hit || ar.dot >= 0.f
						|| ar.struck_brush >= 0)
						return;
					px = ar.pos.X;
					py = ar.pos.Y;
					tha = atan2f(ar.v1.Y, ar.v1.X);
				}
				ev_sched(s2, c2, nullptr);
				r_out[0] = px - q.X;
				r_out[1] = py - q.Y;
				r_out[2] = sqrtf(ThetaIntervalDist2(tha,
					Steer::WrapPi(th_arr + ivt[ivx][0]),
					Steer::WrapPi(th_arr + ivt[ivx][1]))) * kThetaW;
			};
			// LEGACY FIXED SEQUENCE (regression baseline). Under the
			// scheduler these same primitives are dispatched action by
			// action instead of run as a phase.
			for (int ii = 0; legacy_seq && ii < n_near; ++ii) {
				std::vector<GuideTarget> tg(1,
					final_tgt(ivt[ii][0], ivt[ii][1]));
				GuideWeights gw;
				gw.wp = 2.f;
				gw.we = 0.3f;
				shoot(tg, gw, ii);
				gw.wp = 6.f;
				gw.we = 0.05f;
				shoot(tg, gw, ii);
			}
			// FULL-DOMAIN coverage (advisor: estimates may order,
			// never erase headings) - the far intervals each get a
			// shot so no terminal class silently disappears. One
			// shot is COVERAGE ONLY: an interval without a witness
			// stays UNRESOLVED (iv_hits == 0), never "unreachable".
			if (!bnd && legacy_seq)
				for (int ii = 3; ii < 6; ++ii) {
					std::vector<GuideTarget> tg(1,
						final_tgt(ivt[ii][0], ivt[ii][1]));
					GuideWeights gw;
					gw.wp = 2.f;
					gw.we = 0.3f;
					shoot(tg, gw, ii);
				}
			// m = 1: intervals x symmetric laterals; then node-TIME
			// adaptation around whichever shot advanced the best.
			const float lam0 = 0.45f;
			const float bmag[7] = { 0.f, 0.3f, -0.3f, 0.6f,
				-0.6f, 0.95f, -0.95f };
			// NODE-WINNER KEY. Was `best.H` with a +1.f epsilon: on any
			// case that had not yet struck, best.H stayed at -1e30f
			// forever, so win_ii/win_bi were PINNED at (0, 0) and both
			// the m=2 pass and both Gauss-Newton passes were locked to
			// heading interval 0 with zero lateral offset - exactly the
			// cases that needed the extra structure got none of it. The
			// key is now residual-first and always live (and the same
			// quantity in a residual phase).
			auto shot_key = [&]() {
				const float rmin = best.strike_rmin < 1e29f
					? best.strike_rmin
					: (best.bnd_rp < 1e29f ? best.bnd_rp : 1e9f);
				return best.ok ? 1e7f + best.H : -rmin;
			};
			float track_best = shot_key();
			int win_ii = 0, win_bi = 0;
			int win_chart = 0;
			float win_p1 = 0.f;
			float win_lam = 0.45f;
			// ---- TWO COORDINATE CHARTS OVER THE SAME FREE NODES
			// (advisor 2026-08-19) ----
			// Neither chart defines admissibility; neither may exclude
			// the other's candidates. If some future case is poorly
			// represented by both, ADD coordinates - never declare the
			// trajectory impossible.
			//
			// The lateral extent now comes from a BROAD OPTIMISTIC
			// REACHABLE SLICE, not a fixed fraction of the chord: at
			// lambda*N ticks the node can be at most DMax(s0, n1) from
			// the start, and must still be within DMax(SMax, N-n1) of
			// the target. The old +-0.5*chord was a HARD
			// representational limit (the only lateral generator in the
			// code), so no node outside it could ever be proposed.
			auto slice_w = [&](float lam) {
				const int n1 = static_cast<int>(lam
					* static_cast<float>(N));
				if (n1 < 1 || n1 >= N)
					return 0.25f * rl;
				const float r0 = Envelope::DMax(s0, n1, p);
				const float r1 = Envelope::DMax(
					Envelope::SMax(s0, n1, p), N - n1, p);
				const float d0 = lam * rl;
				const float d1 = (1.f - lam) * rl;
				const float w0 = r0 * r0 > d0 * d0
					? sqrtf(r0 * r0 - d0 * d0) : 0.f;
				const float w1 = r1 * r1 > d1 * d1
					? sqrtf(r1 * r1 - d1 * d1) : 0.f;
				float w = w0 < w1 ? w0 : w1;
				// Keep a floor so the chart still proposes when the
				// optimistic slice is degenerate (a proposal is never
				// an admissibility claim).
				if (w < 0.15f * rl)
					w = 0.15f * rl;
				return w;
			};
			// CHART 0 - chord: P0 + lam*(Q-P0) + b*w(lam)*e_perp.
			auto node_chord = [&](float lam, float bfrac) {
				const float w = slice_w(lam) * bfrac;
				GuideTarget n1;
				n1.x = entry.pos.X + lam * rx + w * eqx;
				n1.y = entry.pos.Y + lam * ry + w * eqy;
				n1.until = static_cast<int>(lam
					* static_cast<float>(N));
				return n1;
			};
			// CHART 1 - curvature: the point at arc fraction lam along
			// the circular arc from P0 to Q whose TOTAL SIGNED SWEEP is
			// phi. Excellent exactly where the chord chart is worst
			// (large accumulated turn). Closed form in the chord frame:
			// R = L / (2 sin(phi/2)), centre (L/2, -R cos(phi/2)),
			// angle(t) = pi/2 + phi/2 - t*phi.
			auto node_arc = [&](float lam, float phi) {
				GuideTarget n1;
				const float sp = sinf(0.5f * phi);
				if (fabsf(sp) < 1e-3f || rl < 1.f)
					return node_chord(lam, 0.f);
				const float R = rl / (2.f * sp);
				const float cx = 0.5f * rl;
				const float cy = -R * cosf(0.5f * phi);
				const float th2 = 1.57079633f + 0.5f * phi
					- lam * phi;
				const float lx = cx + R * cosf(th2);
				const float ly = cy + R * sinf(th2);
				n1.x = entry.pos.X + lx * epx + ly * eqx;
				n1.y = entry.pos.Y + lx * epy + ly * eqy;
				n1.until = static_cast<int>(lam
					* static_cast<float>(N));
				return n1;
			};
			auto node_from = [&](int chart, float lam, float p1) {
				return chart == 1 ? node_arc(lam, p1)
					: node_chord(lam, p1);
			};
			if (legacy_seq && tune_m >= 1) {
				// INTERLEAVED PROPOSAL ORDER. The guided phase has a
				// hard budget share and each shot costs ~25 eval-
				// equivalents once exact rollouts are on, so a family
				// APPENDED after another is systematically starved
				// (measured: at 600-1000 evals only the first ~7-13 of
				// the enumerated shots ever ran, which is why the m=2
				// pass and the node-time adaptation had never executed
				// at production budgets). Charts and intervals are
				// therefore round-robined, coarse parameters first.
				struct NodeProp { int chart; int ii; float p1; };
				std::vector<NodeProp> props;
				static const float bf[7] = { 0.f, 0.6f, -0.6f, 0.3f,
					-0.3f, 0.95f, -0.95f };
				static const float phis[6] = { 0.9f, -0.9f, 1.9f,
					-1.9f, 2.7f, -2.7f };
				for (int r3 = 0; r3 < 7; ++r3)
					for (int ii = 0; ii < n_near; ++ii) {
						props.push_back({ 0, ii, bf[r3] });
						if (r3 < 6)
							props.push_back({ 1, ii, phis[r3] });
					}
				for (const NodeProp& np : props) {
					std::vector<GuideTarget> tg;
					tg.push_back(node_from(np.chart, lam0, np.p1));
					tg.push_back(final_tgt(ivt[np.ii][0],
						ivt[np.ii][1]));
					GuideWeights gw;
					gw.wp = 2.f;
					gw.we = 0.3f;
					shoot(tg, gw, np.ii);
					if (shot_key() > track_best) {
						track_best = shot_key();
						win_ii = np.ii;
						win_chart = np.chart;
						best.win_chart = np.chart;
						win_p1 = np.p1;
					}
				}
				// Node-time adaptation (a numerical resolution
				// axis, not admissibility): retry the winning node
				// earlier/later, on its own chart.
				const float lams[4] = { 0.3f, 0.38f, 0.55f,
					0.65f };
				for (int li = 0; li < 4; ++li) {
					std::vector<GuideTarget> tg;
					tg.push_back(node_from(win_chart, lams[li],
						win_p1));
					tg.push_back(final_tgt(ivt[win_ii][0],
						ivt[win_ii][1]));
					GuideWeights gw;
					gw.wp = 2.f;
					gw.we = 0.3f;
					shoot(tg, gw, win_ii);
					if (shot_key() > track_best) {
						track_best = shot_key();
						win_lam = lams[li];
					}
				}
			}
			// m = 2 (generic): first node at the winner's corridor,
			// second free node closer to the target with symmetric
			// lateral variants - depart -> develop -> close, with
			// nothing about the shape encoded.
			if (legacy_seq && tune_m >= 2) {
				// depart -> develop -> close, on the WINNER'S CHART and
				// around its winning node time, with the second node's
				// chart parameter varied both ways.
				const float l2s[3] = { 0.f, 0.4f, -0.4f };
				float lamA = win_lam * 0.65f;
				float lamB = win_lam * 0.65f + 0.32f;
				if (lamA < 0.12f) lamA = 0.12f;
				if (lamB > 0.88f) lamB = 0.88f;
				for (int li = 0; li < 3; ++li) {
					float p2 = win_p1 + (win_chart == 1
						? l2s[li] * 1.5f : l2s[li]);
					if (win_chart == 0) {
						if (p2 > 0.95f) p2 = 0.95f;
						if (p2 < -0.95f) p2 = -0.95f;
					}
					std::vector<GuideTarget> tg;
					tg.push_back(node_from(win_chart, lamA,
						win_p1 * 0.6f));
					tg.push_back(node_from(win_chart, lamB, p2));
					tg.push_back(final_tgt(ivt[win_ii][0],
						ivt[win_ii][1]));
					GuideWeights gw;
					gw.wp = 2.f;
					gw.we = 0.3f;
					shoot(tg, gw, win_ii);
				}
			}
			// OUTCOME-SPACE GAUSS-NEWTON (advisor 2026-08-19): the
			// correction solves the COMPLETE boundary residual
			// R = (x_T - x_Q, y_T - y_Q, w_th * d(theta_T, I)) -
			// never an XY-only polish with terminal heading left in
			// another basin. Pass 1 = single node (2 vars, the
			// cheap fast pass); pass 2 = the FULL ACTIVE NODE
			// VECTOR xi = (x1, y1, x2, y2) of the m=2
			// configuration - J is 3x4 (no squareness requirement),
			// damped least squares (J^T J + lambda I) dxi = -J^T R.
			// Every evaluation replays and is charged; schedules
			// enter the frontier only through ev_sched.
			// GN GATE, RESIDUAL-BASED IN BOTH MODES. Face mode gated on
			// `best.ok` ("a strike inside radius already exists"),
			// which switched the one true residual minimiser OFF
			// exactly when a tight tolerance is requested and the
			// search is still 20u out. Boundary mode was already
			// residual-gated; now they are symmetric.
			if (legacy_seq && tune_m >= 2
				&& ((bnd ? best.bnd_rp : best.strike_rmin) < 500.f
					|| best.ok)
				&& budget - best.evals > 20) {
				const float th_lo_w = Steer::WrapPi(th_arr
					+ ivt[win_ii][0]);
				const float th_hi_w = Steer::WrapPi(th_arr
					+ ivt[win_ii][1]);
				// Replay a node configuration; boundary residual.
				auto eval_nodes = [&](const GuideTarget* nds,
					int nn, float* r_out) {
					std::vector<GuideTarget> tg;
					for (int i4 = 0; i4 < nn; ++i4)
						tg.push_back(nds[i4]);
					tg.push_back(final_tgt(ivt[win_ii][0],
						ivt[win_ii][1]));
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
					r_out[0] = 1e4f;
					r_out[1] = 1e4f;
					r_out[2] = 1e4f;
					if (s2.empty())
						return;
					const int hz2 = bnd
						? static_cast<int>(s2.size())
						: static_cast<int>(s2.size()) + 8;
					vt.max_ticks = hz2;
					Air::Result ar = Air::FlyWishSchedule(entry,
						w, p, vt, g, s2, c2, hz2);
					float px, py, tha;
					if (bnd) {
						if (ar.struck_brush >= 0 || ar.grounded
							|| static_cast<int>(s2.size()) != N)
							return;
						px = ar.end_pos.X;
						py = ar.end_pos.Y;
						const Vec3& vT = ar.end_state.vel;
						tha = Len2D(vT) > 1.f
							? atan2f(vT.Y, vT.X) : 0.f;
					} else {
						if (!ar.hit || ar.dot >= 0.f
							|| ar.struck_brush >= 0)
							return;
						px = ar.pos.X;
						py = ar.pos.Y;
						tha = atan2f(ar.v1.Y, ar.v1.X);
					}
					ev_sched(s2, c2, nullptr);
					r_out[0] = px - q.X;
					r_out[1] = py - q.Y;
					r_out[2] = sqrtf(ThetaIntervalDist2(tha,
						th_lo_w, th_hi_w)) * kThetaW;
				};
				// ---- Pass 1: single node, 2 variables ----
				{
					GuideTarget seed1 = node_from(win_chart, win_lam,
						win_p1);
					float nx = seed1.x;
					float ny = seed1.y;
					auto eval1 = [&](float x2, float y2,
						float* r_out) {
						GuideTarget n1;
						n1.x = x2;
						n1.y = y2;
						n1.until = static_cast<int>(0.45f
							* static_cast<float>(N));
						eval_nodes(&n1, 1, r_out);
					};
					float R0[3];
					eval1(nx, ny, R0);
					for (int it = 0; it < 3
						&& budget - best.evals > 8; ++it) {
						if (fabsf(R0[0]) > 5e3f)
							break;
						const float dstep = 24.f;
						float R1[3], R2[3];
						eval1(nx + dstep, ny, R1);
						eval1(nx, ny + dstep, R2);
						float Jm[3][2];
						for (int r2 = 0; r2 < 3; ++r2) {
							Jm[r2][0] = (R1[r2] - R0[r2])
								/ dstep;
							Jm[r2][1] = (R2[r2] - R0[r2])
								/ dstep;
						}
						// (J^T J + lambda I) dx = -J^T R (2x2)
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
						const float dl = sqrtf(dx * dx
							+ dy * dy);
						if (dl > 160.f) {
							dx *= 160.f / dl;
							dy *= 160.f / dl;
						}
						nx += dx;
						ny += dy;
						eval1(nx, ny, R0);
					}
				}
				// ---- Pass 2: FULL ACTIVE NODE VECTOR (m=2) ----
				if (budget - best.evals > 60) {
					// Seeds from the WINNING CHART and node time. The old
					// form hard-coded the chord centreline via
					// bmag[win_bi], and win_bi is no longer assigned at
					// all - so pass 2 was pinned regardless of which
					// chart or lateral actually won.
					GuideTarget s2a = node_from(win_chart,
						win_lam * 0.65f, win_p1 * 0.6f);
					GuideTarget s2b = node_from(win_chart,
						win_lam * 0.65f + 0.32f, win_p1);
					float xi[4] = { s2a.x, s2a.y, s2b.x, s2b.y };
					auto evalxi = [&](const float* x4,
						float* r_out) {
						GuideTarget nds[2];
						nds[0].x = x4[0];
						nds[0].y = x4[1];
						nds[0].until = static_cast<int>(0.3f
							* static_cast<float>(N));
						nds[1].x = x4[2];
						nds[1].y = x4[3];
						nds[1].until = static_cast<int>(0.62f
							* static_cast<float>(N));
						eval_nodes(nds, 2, r_out);
					};
					float R0[3];
					evalxi(xi, R0);
					for (int it = 0; it < 3
						&& budget - best.evals > 12; ++it) {
						if (fabsf(R0[0]) > 5e3f)
							break;
						const float dstep = 24.f;
						float Jm[3][4];
						for (int c4 = 0; c4 < 4; ++c4) {
							float xp[4] = { xi[0], xi[1], xi[2],
								xi[3] };
							xp[c4] += dstep;
							float Rp[3];
							evalxi(xp, Rp);
							for (int r2 = 0; r2 < 3; ++r2)
								Jm[r2][c4] = (Rp[r2] - R0[r2])
									/ dstep;
						}
						// (J^T J + lambda I) dxi = -J^T R via
						// 4x4 elimination, partial pivoting.
						float A[4][5];
						for (int i4 = 0; i4 < 4; ++i4) {
							for (int j4 = 0; j4 < 4; ++j4) {
								float acc = i4 == j4
									? 0.5f : 0.f;
								for (int r2 = 0; r2 < 3; ++r2)
									acc += Jm[r2][i4]
										* Jm[r2][j4];
								A[i4][j4] = acc;
							}
							float bb = 0.f;
							for (int r2 = 0; r2 < 3; ++r2)
								bb -= Jm[r2][i4] * R0[r2];
							A[i4][4] = bb;
						}
						bool sing = false;
						for (int c4 = 0; c4 < 4 && !sing;
							++c4) {
							int piv = c4;
							for (int r3 = c4 + 1; r3 < 4; ++r3)
								if (fabsf(A[r3][c4])
									> fabsf(A[piv][c4]))
									piv = r3;
							if (fabsf(A[piv][c4]) < 1e-7f) {
								sing = true;
								break;
							}
							if (piv != c4)
								for (int j4 = c4; j4 < 5;
									++j4) {
									const float t3 = A[c4][j4];
									A[c4][j4] = A[piv][j4];
									A[piv][j4] = t3;
								}
							for (int r3 = c4 + 1; r3 < 4;
								++r3) {
								const float f4 = A[r3][c4]
									/ A[c4][c4];
								for (int j4 = c4; j4 < 5;
									++j4)
									A[r3][j4] -= f4
										* A[c4][j4];
							}
						}
						if (sing)
							break;
						float dx4[4];
						for (int c4 = 3; c4 >= 0; --c4) {
							float acc = A[c4][4];
							for (int j4 = c4 + 1; j4 < 4; ++j4)
								acc -= A[c4][j4] * dx4[j4];
							dx4[c4] = acc / A[c4][c4];
						}
						for (int nn = 0; nn < 2; ++nn) {
							const float dl = sqrtf(
								dx4[nn * 2] * dx4[nn * 2]
								+ dx4[nn * 2 + 1]
								* dx4[nn * 2 + 1]);
							if (dl > 160.f) {
								dx4[nn * 2] *= 160.f / dl;
								dx4[nn * 2 + 1] *= 160.f / dl;
							}
						}
						for (int c4 = 0; c4 < 4; ++c4)
							xi[c4] += dx4[c4];
						evalxi(xi, R0);
					}
				}
			}
		// (the guided scope stays OPEN: deepen/perturb and the
		// scheduler below dispatch these same primitives)
		// deepen(): local refinement of one elite (knot/length moves,
		// the resolution ladder, reversal insertion).
		// EXPLICIT BUDGET RESERVATION. The value phase gets a bounded,
		// documented share and the requested tolerance gets the rest -
		// rather than the value phase running until it happens to
		// converge (measured: at 600 evals it never did, so continuation
		// never fired at all) or being preempted mid-round by a race on
		// the eval counter. Truth is unaffected either way: this splits
		// RESOLUTION, and crediting stays pinned to the caller's
		// original radius.
		// The coverage prefix (seeds + the fixed guided shot sequence) is
		// BUDGET-INDEPENDENT by construction - that is what makes compute
		// monotonic - so it cannot be bounded by a fraction of the budget.
		// The reserved share therefore applies to the DISCRETIONARY
		// REMAINDER after coverage. (Measured consequence worth keeping:
		// the 40-shot prefix costs ~1000 eval-equivalents with exact
		// rollouts on, so budgets below ~1500 are almost entirely
		// coverage - precisely the case for the scheduler's
		// cheap-scout-first escalation over flying every shot everywhere.)
		best.evals_cover = best.evals;
		const int disc_rem = budget - best.evals_cover;
		const int phase1_cap = prec < radius
			? best.evals_cover + (disc_rem > 0 ? (disc_rem * 3) / 5 : 0)
			: budget;
		int rung_i = 0;   // next kLadder rung the continuation targets
		// The refinement ROUND must respect the reserved share too:
		// deepen()/perturb() run until the budget is gone, so gating
		// the tightening on budget-remaining-after-a-round meant it
		// never fired at all. budget_cur is the phase's ceiling and
		// lifts to the full budget once the tolerance tightens.
		int budget_cur = phase1_cap;
		// max_ev bounds ONE deepen action so its cost is fixed and
		// known in advance (the anytime contract): the same first
		// max_ev evaluations run every time from the same elite state,
		// and repeated actions resume from the improved state.
		auto deepen = [&](BinElite& pe, float kstep, int lstep,
			int max_ev = 1 << 28) {
			if (!pe.has)
				return;
			const int ev_start = best.evals;
			bool moved = true;
			int guard = 0;
			while (moved && best.evals < budget_cur && guard++ < 8
				&& best.evals - ev_start < max_ev) {
				moved = false;
				for (size_t j = 0; j < pe.runs.size(); ++j) {
					for (size_t ki = 0; ki < pe.runs[j].ck.size();
						++ki)
						for (int sg = -1; sg <= 1; sg += 2) {
							if (best.evals >= budget_cur)
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
								pe.res = ev_res;
								pe.val = ev_val;
								pe.rp = ev_rp;
								pe.rth = ev_rth;
								moved = true;
							}
						}
					for (int sg = -1; sg <= 1; sg += 2) {
						if (best.evals >= budget_cur)
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
							pe.res = ev_res;
							pe.val = ev_val;
							pe.rp = ev_rp;
							pe.rth = ev_rth;
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
						pe.res = ev_res;
						pe.val = ev_val;
						pe.rp = ev_rp;
						pe.rth = ev_rth;
						moved = true;
					}
				}
				if (!moved && pe.runs.size() < 7
					&& pe.runs[lj].len >= 2 * min_gap
					&& best.evals < budget_cur) {
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
						pe.res = ev_res;
						pe.val = ev_val;
						pe.rp = ev_rp;
						pe.rth = ev_rth;
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
			pe.res = ev_res;
			pe.val = ev_val;
			pe.rp = ev_rp;
			pe.rth = ev_rth;
			deepen(pe, 0.12f, 2);
		};
		// Resolution rungs; the finest is SPEED-DERIVED (airrec
		// 2026-08-19: the active gain band spans ~cap/s of stored
		// cosa, so a fixed 0.05 floor is coarser than the whole band
		// at speed and long arcs could not be aimed).
		const float kfine = p.air_speed_cap
			/ (2.f * (s0 > p.air_speed_cap ? s0
				: p.air_speed_cap));
		const float ksteps[4] = { 0.3f, 0.12f, 0.05f, kfine };
		const int lsteps[4] = { 6, 2, 2, 1 };
		int dry = 0;
		// MULTI-CHANNEL CONVERGENCE. dry used to increment on best.H
		// alone, which (a) never moves during a residual-reduction
		// phase, so a continuation search would "converge" instantly,
		// and (b) had a live bug: best.H and last_best both start at
		// -1e30f and -1e30f + 1e-3f == -1e30f in single precision, so
		// any solve that never struck incremented dry on its FIRST
		// round and got exactly 4 refinement rounds - cutting off the
		// hardest cases, the ones with the most budget left.
		float last_H = -1e30f;
		float last_res = 1e30f;
		int   last_hits = 0;
		auto progress = [&]() {
			int hits = 0;
			for (int i4 = 0; i4 < 6; ++i4)
				hits += best.iv_hits[i4] > 0 ? 1 : 0;
			for (int b2 = 0; b2 < kBins; ++b2)
				hits += bins[static_cast<size_t>(b2)].has ? 1 : 0;
			for (int s3 = 0; s3 < 3; ++s3)
				hits += scouts[static_cast<size_t>(s3)].has ? 1 : 0;
			const float rmin = best.strike_rmin < 1e29f
				? best.strike_rmin
				: (best.bnd_rp < 1e29f ? best.bnd_rp
					: best_any_res);
			// Relative epsilon on H: 1e-3 absolute is below float ULP
			// at H ~ 1e6 and therefore meaningless there.
			const float hEps = fabsf(last_H) > 1.f
				? fabsf(last_H) * 1e-6f : 1e-3f;
			bool moved = false;
			if (best.H > last_H + hEps) moved = true;
			if (rmin < last_res - 1e-4f) moved = true;
			if (hits > last_hits) moved = true;
			last_H = best.H;
			last_res = rmin;
			last_hits = hits;
			return moved;
		};
		// Re-key every surviving elite after the active tolerance
		// changes: each carries its raw (residual, value) so the key is
		// recomputed for free rather than re-flying ~27 schedules.
		auto rekey = [&]() {
			for (BinElite& b2 : bins)
				if (b2.has)
					b2.sc = key_of(b2.res, b2.val, b2.rp,
						b2.rth);
			for (BinElite& s3 : scouts)
				if (s3.has)
					s3.sc = key_of(s3.res, s3.val, s3.rp,
						s3.rth);
		};
		while (legacy_seq && best.evals < budget && dry < 4) {
			// Scouts always get the full schedule.
			for (int rd = 0; rd < 4 && best.evals < budget_cur; ++rd)
				for (int si2 = 0; si2 < 3; ++si2)
					deepen(scouts[static_cast<size_t>(si2)],
						ksteps[rd], lsteps[rd]);
			// Then the top bins by the SEARCH KEY (was by .H alone,
			// which spent the refinement budget on the fastest bins
			// rather than the ones that can still improve) + the
			// softest-|dot| bin, which is meaningless in boundary mode
			// where free flight never sets a clip dot.
			std::vector<int> order;
			for (int b2 = 0; b2 < kBins; ++b2)
				if (bins[static_cast<size_t>(b2)].has)
					order.push_back(b2);
			if (!order.empty()) {
				std::sort(order.begin(), order.end(),
					[&](int a2, int b3) {
						return bins[static_cast<size_t>(a2)].sc
							> bins[static_cast<size_t>(b3)].sc;
					});
				int soft = order[0];
				if (!bnd)
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
				for (int rd = 0; rd < 4 && best.evals < budget_cur;
					++rd)
					for (int b2 : work) {
						if (best.evals >= budget_cur)
							break;
						deepen(bins[static_cast<size_t>(b2)],
							ksteps[rd], lsteps[rd]);
					}
				if (best.evals < budget_cur)
					perturb(bins[static_cast<size_t>(work[0])]);
			}
			if (best.evals < budget_cur && scouts[0].has)
				perturb(scouts[0]);
			if (!progress())
				dry++;
			else
				dry = 0;
			// TOLERANCE CONTINUATION: when this round converged and a
			// finer tolerance was requested, tighten to the next rung
			// and keep going - the residual gradient is restored
			// instead of the search declaring victory at the coarse
			// radius. Advancing only BETWEEN rounds (never inside
			// deepen) keeps the elite cache coherent; rekey() refreshes
			// the keys from the raw outcomes.
			// A RESERVED SHARE for the precision phase. Measured
			// 2026-08-19: at 600 evals/case the value phase never
			// converged, so continuation never fired and the sub-8u
			// rungs stayed at zero - the cliff persisted for a pure
			// ALLOCATION reason. The requested tolerance therefore gets
			// a guaranteed minimum share of the budget (the advisor's
			// "every unresolved domain receives minimum exploration",
			// applied to resolution rather than to regions).
			// NOTE the guard tests the FULL budget, never the phase
			// ceiling: gating the tightening on the ceiling it is
			// supposed to LIFT is a self-lock (measured: phase1 ran
			// to 1015 of 1200 and the tolerance never moved).
			if ((dry >= 4 || best.evals >= phase1_cap) && prec < tol
				&& best.evals < budget) {
				float next = prec;
				while (rung_i < 6 && kLadder[rung_i] >= tol)
					rung_i++;
				if (rung_i < 6 && kLadder[rung_i] > prec)
					next = kLadder[rung_i];
				if (next < tol) {
					budget_cur = budget;   // lift the phase ceiling
					// Record how the budget actually divided the
					// first time the tolerance tightens (property
					// gate: phase 1 must own a bounded share).
					if (best.evals_phase1 == 0)
						best.evals_phase1 = best.evals;
					tol = next;
					rekey();
					dry = 0;
					last_res = 1e30f;
					last_hits = 0;
				}
			}
		}
			// ================= SCHEDULER v0 (advisor 2026-08-19,
			// contract in Docs/AirRecSpec.md 17) =================
			// ONE deterministic action stream. NextAction() is a pure
			// function of solver state - it CANNOT see the requested or
			// remaining budget - and the Runner has exactly one
			// interaction with the budget: ask what happens next, look
			// up its fixed known charge, execute it if it fits,
			// otherwise STOP. It never skips an unaffordable action in
			// favour of a cheaper later one, because that reorders the
			// stream and breaks run(B1) < run(B2) as a literal prefix.
			if (use_sched) {
				// deepen()'s legacy loop consults budget_cur, which is
				// budget-derived: inside a scheduler action that makes
				// the SAME action behave differently at different
				// budgets, so the streams diverge (caught by S1 the
				// moment the interval bound changed the action mix).
				// On this path only max_ev bounds a deepen action, and
				// the runner has already guaranteed affordability.
				budget_cur = 1 << 28;
				// Domain = (Q region, T branch, I_theta). Q and T are
				// fixed for one RefSolve call, so the live axis is the
				// heading interval. THREE terminal states, not two:
				// PARTITIONED is not RESOLVED (a split replaces a
				// domain with children that cover it).
				enum { DOM_UNRESOLVED = 0, DOM_IRRELEVANT = 1,
					DOM_PARTITIONED = 2 };
				struct Dom {
					int   iv = 0;
					float L = -1e30f;    // best witnessed value here
					float U = 1e30f;     // CERTIFIED ceiling (prunes)
					float U_adv = 1e30f; // advisory interval bound
					float U_order = 1e30f;  // ordering signal
					float R = 1e30f;     // best residual here
					float kappa = 0.f;   // conditioning estimate
					int   spent = 0;
					int   m = 0;         // current shooting level
					int   scouts = 0;
					int   shots[3] = { 0, 0, 0 };
					int   stall = 0;     // shots since R improved
					int   deep = 0;      // DEEPEN actions run here
					int   gn = 0;        // GN iterations run here
					float gx = 0.f, gy = 0.f;   // GN iterate
					float V = 0.f;       // virtual service time
					int   lane = 0;      // current method lane
					int   lane_stall = 0;
					int   rung = 0;      // precision rungs driven
					bool  gn_init = false;
					int   state = 0;   // DOM_UNRESOLVED
				};
				const int n_dom = bnd ? 1 : 6;
				std::vector<Dom> dom(static_cast<size_t>(n_dom));
				for (int i = 0; i < n_dom; ++i)
					dom[static_cast<size_t>(i)].iv = i;
				// ---- CERTIFIED INPUTS, NOT NOMINAL ONES (B5/B7,
				// 2026-08-19). Both ceilings below bound the SAME
				// quantity the operator credits: H = |end_state.vel|^2
				// + 2 g gs (contact z - zmin), measured AFTER
				// FinishGravity. Two nominal-vs-credited mismatches had
				// to be closed before either could be called certified:
				//
				//  * ARRIVAL TICK. A face-mode strike is credited at any
				//    tick a schedule of length [8, n_cap] can reach (the
				//    flight runs to total + 8), not only at N. Vertical
				//    motion in air is pure ballistics - no control
				//    touches v_z - so the admissible window is EXACT:
				//    only ticks whose ballistic z lands inside the
				//    acceptance ball can produce a credited strike. The
				//    ceiling is maximised over that window rather than
				//    evaluated at one guessed endpoint (the candidate
				//    structure is not monotone in v_z).
				//  * CONTACT HEIGHT. Any contact within `radius` of q is
				//    credited, so the potential term must use the top of
				//    the acceptance ball, not q itself. At radius 28 on
				//    surf gravity that is 44.8k of energy the nominal
				//    form silently omitted.
				// The recipe itself lives in UCertifiedFace (header) so the
				// refinement layer and this path can never drift apart.
				const int k_cap = n_cap + 8;
				unsigned bid_g = 0;
				const float U_global = bnd
					? Envelope::SMax(s0, N, p)
					: UCertifiedFace(facep->n, entry.pos, entry.vel, gs, p,
						q, radius, zmin, k_cap, -3.14159265f, 3.14159265f,
						&bid_g);
				for (Dom& d : dom)
					d.U = U_global;
				// PER-DOMAIN CEILING: the interval-specific relaxed
				// board bound. Unlike the global energy ceiling this
				// DIFFERS between heading intervals, which is the
				// discrimination the scheduler was missing (measured:
				// with a constant U_D it spread work by tie-break and
				// lost the capability differential 1030k vs 965k).
				// Only the certified bound may prune; while this one
				// is ADVISORY it may order work but never establish
				// PROVED_IRRELEVANT.
				if (!bnd) {
					for (int i = 0; i < n_dom; ++i) {
						Dom& d = dom[static_cast<size_t>(i)];
						const float tlo = Steer::WrapPi(th_arr
							+ ivt[i][0]);
						const float thi = Steer::WrapPi(th_arr
							+ ivt[i][1]);
						unsigned bid_i = 0;
						const float ub = UCertifiedFace(facep->n, entry.pos,
							entry.vel, gs, p, q, radius, zmin, k_cap, tlo,
							thi, &bid_i);
						d.U_adv = ub;
						// PROMOTED TO CERTIFIED (2026-08-19, gates B1-B7).
						// The interval ceiling now replaces the broad energy
						// ceiling as the domain's certified U, so it may
						// establish PROVED_IRRELEVANT and not merely order
						// work. It is only ever taken when it is TIGHTER, so
						// the certified value can never regress.
						if (ub > -1e29f && ub < d.U)
							d.U = ub;
						d.U_order = d.U;
					}
				} else {
					for (Dom& d : dom)
						d.U_order = d.U;
				}
				// ---- action algebra ----
					// A_COVER is the MANDATORY CHEAP TOUCH of a heading domain;
					// A_SHOOT is escalation. They are distinct action IDENTITIES
					// because they carry different information and different
					// cost, and a trace must never conflate them.
					enum { A_COVER = 1, A_SHOOT = 2, A_PRECISION = 3,
						A_DEEPEN = 4, A_GNITER = 5 };
				struct Act {
					int type = 0;
					int iv = 0;
					int m = 0;
					int chart = 0;
					int idx = 0;
					int cost = 0;
					unsigned long long hash = 0;
				};
				// A SEMANTIC content hash: what the action IS, never
				// where it lives. Includes the solver-identity bits so
				// traces from different configurations never compare
				// equal.
				auto act_hash = [&](const Act& a) {
					unsigned long long h = 1469598103934665603ULL;
					auto mix = [&](unsigned long long v) {
						h ^= v;
						h *= 1099511628211ULL;
					};
					mix(static_cast<unsigned long long>(a.type));
					mix(static_cast<unsigned long long>(a.iv + 1));
					mix(static_cast<unsigned long long>(a.m + 1));
					mix(static_cast<unsigned long long>(a.chart + 1));
					mix(static_cast<unsigned long long>(a.idx + 1));
					mix(static_cast<unsigned long long>(tune_m + 1));
					mix(static_cast<unsigned long long>(N));
					mix(static_cast<unsigned long long>(
						bnd ? 7u : 3u));
					return h;
				};
				// Conservative FIXED charge per action class. The
				// runner refuses to start an action it cannot pay for
				// in full; if leftover budget ever becomes wasteful the
				// answer is SMALLER resumable units, never "find a
				// cheaper action that fits".
				const int kCostShot = 40;
				// Coverage runs closed-form only (no second-stage exact
				// ranking), so it costs one scoring flight plus change.
				// Priced with headroom, still an order below a shot.
				const int kCostCover = 4;
				const int kCostPrec = 1;
				const int kCostDeep = 10;   // deepen bounded to 8 evals
				const int kCostGN = 8;      // one iteration = 3 probes
				// ---- NextAction: pure function of solver state ----
				// (no budget, no remaining, no budget-derived flag)
				auto next_action = [&](Act* out) {
					// 1. minimum cheap exposure: every unresolved
					//    domain gets a scout before anything escalates.
					for (int i = 0; i < n_dom; ++i) {
						Dom& d = dom[static_cast<size_t>(i)];
						if (d.state == DOM_UNRESOLVED
							&& d.scouts == 0) {
							out->type = A_COVER;
							out->iv = i;
							out->m = 0;
							out->chart = 0;
							out->idx = 0;
							out->cost = kCostCover;
							out->hash = act_hash(*out);
							return true;
						}
					}
					// 2. certified irrelevance: a domain whose ceiling
					//    cannot beat the incumbent is PROVED_IRRELEVANT
					//    and its elimination is recorded with both the
					//    bound identity and the incumbent witness.
					const float Lstar = best.ok ? best.H : -1e30f;
					for (int i = 0; i < n_dom; ++i) {
						Dom& d = dom[static_cast<size_t>(i)];
						if (d.state != DOM_UNRESOLVED || !best.ok)
							continue;
						if (d.U <= Lstar) {
							d.state = DOM_IRRELEVANT;
							RefResult::PruneRec pr;
							pr.domain = i;
							// Which ceiling actually did the eliminating - a
							// prune that cannot name its bound is not
							// proof-carrying.
							pr.bound_id = (d.U_adv > -1e29f
								&& d.U <= d.U_adv + 1e-3f)
								? 0x55300002u    // interval board ceiling
								: 0x55300001u;   // broad energy ceiling
							pr.U = d.U;
							pr.L_star = Lstar;
							pr.eps = 0.f;
							pr.witness_id = best.wcosa.empty() ? 0ULL
								: static_cast<unsigned long long>(
									best.horizon) * 1000003ULL
									+ best.wside.size();
							best.prunes.push_back(pr);
						}
					}
					// 3. pick the competitive domain: importance is the
					//    certified competitive gap ONLY. kappa, stall
					//    and residual history choose HOW to refine it,
					//    never WHETHER it matters. Ties break on the
					//    stable (spent, index) order - never on
					//    container iteration or wall-clock state.
					int pick = -1;
					float best_gap = -1e30f;
					float best_V = 1e30f;
					for (int i = 0; i < n_dom; ++i) {
						const Dom& d = dom[static_cast<size_t>(i)];
						if (d.state != DOM_UNRESOLVED)
							continue;
						// ORDERING may use the advisory bound;
						// PRUNING below uses only the certified one.
						const float gap = d.U_order
							- (best.ok ? best.H : -1e30f);
						if (fair_sched) {
							// WEIGHTED-FAIR SERVICE: pick the least
							// virtual time. Importance biases service
							// (a bigger gap accrues virtual time more
							// slowly) but can never starve a peer -
							// raw argmax(gap) serviced ONE domain
							// forever whenever its gap never moved.
							if (pick < 0 || d.V < best_V - 1e-9f
								|| (fabsf(d.V - best_V) <= 1e-9f
									&& i < pick)) {
								pick = i;
								best_V = d.V;
								best_gap = gap;
							}
						} else if (pick < 0 || gap > best_gap + 1e-3f
							|| (fabsf(gap - best_gap) <= 1e-3f
								&& (d.spent < dom[static_cast<size_t>(
									pick)].spent))) {
							pick = i;
							best_gap = gap;
						}
					}
					if (pick < 0)
						return false;   // every domain resolved
					Dom& d = dom[static_cast<size_t>(pick)];
					// 4. method selection from the domain's own state.
					//    Exploration debt activates PROGRESSIVELY: a
					//    domain earns chart/m-level debt only after
					//    cheap coverage shows it is still competitive.
					if (fair_sched) {
						// LANE LADDER. A stalled method yields; a
						// stalled DOMAIN does not become irrelevant
						// (only a certified bound may do that).
						// Lanes: 0 m0, 1 m1, 2 m2, 3 deepen, 4 gn,
						// 5 precision - after two unproductive
						// actions the lane advances, wrapping over
						// the refinement lanes so precision and GN
						// cannot be starved by more shooting.
						int ln = d.lane;
						for (int tries = 0; tries < 6; ++tries) {
							bool ok_lane = true;
							if (ln == 0 && d.shots[0] >= 2)
								ok_lane = false;
							if (ln == 1 && d.shots[1] >= 2)
								ok_lane = false;
							if (ln == 4 && !best.ok)
								ok_lane = false;
							if (ln == 5) {
								// A domain acquires PRECISION DEBT only
								// once it OWNS a witness at the active
								// rung; the step then tightens exactly
								// one rung and yields back to ordinary
								// refinement until that tighter rung is
								// itself satisfied. find-feasible ->
								// tighten -> refine -> find-feasible.
								const float rnow = bnd ? best.bnd_rp
									: best.strike_rmin;
								if (!(prec < tol) || rnow > tol)
									ok_lane = false;
							}
							if (ok_lane)
								break;
							ln = ln >= 5 ? 2 : ln + 1;
						}
						d.lane = ln;
						if (ln == 5) {
							out->type = A_PRECISION;
							out->idx = static_cast<int>(tol);
						} else if (ln == 4) {
							out->type = A_GNITER;
							out->idx = d.gn;
						} else if (ln == 3) {
							out->type = A_DEEPEN;
							out->idx = d.deep;
						} else {
							out->type = A_SHOOT;
							out->m = ln;
							out->chart = d.shots[ln] % 2;
							out->idx = d.shots[ln];
						}
						out->iv = pick;
						out->m = ln <= 2 ? ln : d.m;
						out->cost = out->type == A_PRECISION
							? kCostPrec
							: (out->type == A_DEEPEN ? kCostDeep
								: (out->type == A_GNITER ? kCostGN
									: kCostShot));
						out->hash = act_hash(*out);
						return true;
					}
					if (d.shots[0] < 2) {
						out->type = A_SHOOT;
						out->m = 0;
						out->chart = d.shots[0] % 2;
						out->idx = d.shots[0];
					} else if (d.m < 1 || (d.m == 1 && d.stall < 2)) {
						out->type = A_SHOOT;
						out->m = 1;
						out->chart = d.shots[1] % 2;
						out->idx = d.shots[1];
					} else if (d.m < 2 || d.stall < 4) {
						out->type = A_SHOOT;
						out->m = 2;
						out->chart = d.shots[2] % 2;
						out->idx = d.shots[2];
					} else if (d.deep < 2 + d.gn) {
						// local polish where a witness exists
						out->type = A_DEEPEN;
						out->m = d.m;
						out->chart = 0;
						out->idx = d.deep;
					} else if (d.gn < 3 && best.ok) {
						// outcome-space correction, ONE iteration
						out->type = A_GNITER;
						out->m = d.m;
						out->chart = 0;
						out->idx = d.gn;
					} else if (prec < tol) {
						out->type = A_PRECISION;
						out->m = d.m;
						out->chart = 0;
						out->idx = static_cast<int>(tol);
					} else {
						// keep refining the hardest competitive domain
						out->type = A_SHOOT;
						out->m = 2;
						out->chart = (d.shots[2] + 1) % 2;
						out->idx = d.shots[2];
					}
					out->iv = pick;
					out->cost = out->type == A_PRECISION ? kCostPrec
						: (out->type == A_DEEPEN ? kCostDeep
							: (out->type == A_GNITER ? kCostGN
								: kCostShot));
					out->hash = act_hash(*out);
					return true;
				};
				// ---- the Runner: the ONLY place the budget is read ----
				int guard_actions = 0;
				while (guard_actions++ < 4096) {
					Act a;
					if (!next_action(&a))
						break;
					if (best.evals + a.cost > budget)
						break;   // stop, never skip
					best.trace.push_back(a.hash);
					best.sched_actions++;
					Dom& d = dom[static_cast<size_t>(a.iv)];
					const int ev0 = best.evals;
					const float r0 = best.strike_rmin < 1e29f
						? best.strike_rmin
						: (best.bnd_rp < 1e29f ? best.bnd_rp
							: best_any_res);
					if (a.type == A_PRECISION) {
						float next = prec;
						while (rung_i < 6 && kLadder[rung_i] >= tol)
							rung_i++;
						if (rung_i < 6 && kLadder[rung_i] > prec)
							next = kLadder[rung_i];
						if (next < tol) {
							tol = next;
							rekey();
						}
						best.sched_prec++;
						d.rung++;
					} else if (a.type == A_DEEPEN) {
						// one BOUNDED deepen unit on this domain's
						// best elite (bins hold strikes; scouts hold
						// the residual-descent memory).
						int bi2 = -1;
						for (int b3 = 0; b3 < kBins; ++b3)
							if (bins[static_cast<size_t>(b3)].has
								&& (bi2 < 0
									|| bins[static_cast<size_t>(b3)].sc
										> bins[static_cast<size_t>(bi2)].sc))
								bi2 = b3;
						if (bi2 >= 0)
							deepen(bins[static_cast<size_t>(bi2)],
								0.12f, 2, 8);
						else if (scouts[0].has)
							deepen(scouts[0], 0.12f, 2, 8);
						d.deep++;
						best.sched_deep++;
					} else if (a.type == A_GNITER) {
						// ONE Gauss-Newton iteration, state carried
						// on the domain so the action is atomic.
						if (!d.gn_init) {
							GuideTarget sd0 = node_from(0, 0.45f,
								0.f);
							d.gx = sd0.x;
							d.gy = sd0.y;
							d.gn_init = true;
						}
						float R0[3], R1[3], R2[3];
						gn_eval1(a.iv, d.gx, d.gy, R0);
						if (fabsf(R0[0]) < 5e3f) {
							const float ds = 24.f;
							gn_eval1(a.iv, d.gx + ds, d.gy, R1);
							gn_eval1(a.iv, d.gx, d.gy + ds, R2);
							float a11 = 0.5f, a12 = 0.f, a22 = 0.5f;
							float b1 = 0.f, b2 = 0.f;
							for (int r2 = 0; r2 < 3; ++r2) {
								const float j0 = (R1[r2] - R0[r2])
									/ ds;
								const float j1 = (R2[r2] - R0[r2])
									/ ds;
								a11 += j0 * j0;
								a12 += j0 * j1;
								a22 += j1 * j1;
								b1 -= j0 * R0[r2];
								b2 -= j1 * R0[r2];
							}
							const float det = a11 * a22 - a12 * a12;
							if (fabsf(det) > 1e-6f) {
								float dx = (b1 * a22 - b2 * a12)
									/ det;
								float dy = (b2 * a11 - b1 * a12)
									/ det;
								const float dl = sqrtf(dx * dx
									+ dy * dy);
								if (dl > 160.f) {
									dx *= 160.f / dl;
									dy *= 160.f / dl;
								}
								d.gx += dx;
								d.gy += dy;
							}
						}
						d.gn++;
						best.sched_gn++;
					} else if (a.type == A_COVER) {
						// ScoutCheap(D): the ONE job is to initialise this heading
						// domain enough for the scheduler to decide what deserves
						// more - best residual seen, whether any approach looks
						// plausible, a witness if lucky, a stall/conditioning seed.
						// No intermediate nodes, no second-stage exact ranking of
						// four candidates, no m1/m2/GN/deepen/precision: those are
						// escalation and belong AFTER the domain earns them.
						//
						// CONSERVATISM (S10): finding nothing here means UNRESOLVED
						// and never "this interval is unreachable". Only a certified
						// bound may eliminate a domain - which is exactly what makes
						// it safe for coverage to be this cheap.
						std::vector<GuideTarget> tg;
						tg.push_back(final_tgt(ivt[a.iv][0], ivt[a.iv][1]));
						GuideWeights gw;
						gw.wp = 2.f;
						gw.we = 0.3f;
						shoot(tg, gw, a.iv, cover_top);
						d.scouts++;
						best.sched_cover++;
						best.sched_cover_ev += best.evals - ev0;
						best.dom_cover[a.iv < 6 ? a.iv : 5]++;
					} else {
						const int ivx = a.iv < n_near ? a.iv : a.iv;
						std::vector<GuideTarget> tg;
						if (a.m >= 1)
							tg.push_back(node_from(a.chart,
								a.m >= 2 ? 0.3f : 0.45f,
								a.chart == 1
									? (a.idx % 2 ? 1.9f : -1.9f)
									: (a.idx % 2 ? 0.6f : -0.6f)));
						if (a.m >= 2)
							tg.push_back(node_from(a.chart, 0.62f,
								a.chart == 1
									? (a.idx % 2 ? -0.9f : 0.9f)
									: (a.idx % 2 ? -0.3f : 0.3f)));
						tg.push_back(final_tgt(ivt[ivx][0],
							ivt[ivx][1]));
						GuideWeights gw;
						gw.wp = 4.f;
						gw.we = 0.15f;
						shoot(tg, gw, ivx);
						if (a.m >= 0 && a.m < 3)
							d.shots[a.m]++;
						if (a.m > d.m)
							d.m = a.m;
						if (a.m >= 0 && a.m < 3)
							best.sched_m_used[a.m]++;
					}
					d.spent += best.evals - ev0;
					best.dom_acts[a.iv < 6 ? a.iv : 5]++;
					if (a.type == A_SHOOT && a.m == 2)
						best.dom_m2[a.iv < 6 ? a.iv : 5]++;
					if (fair_sched) {
						// Virtual time advances by cost weighted by
						// the domain's unresolved potential: a bigger
						// gap accrues more slowly and is therefore
						// serviced more often, without starving peers.
						float gref = 1.f;
						for (int i2 = 0; i2 < n_dom; ++i2) {
							const Dom& dd = dom[static_cast<size_t>(
								i2)];
							if (dd.state != DOM_UNRESOLVED)
								continue;
							const float gg = dd.U_order
								- (best.ok ? best.H : 0.f);
							if (gg > gref)
								gref = gg;
						}
						float gd = d.U_order
							- (best.ok ? best.H : 0.f);
						if (gd < 1.f)
							gd = 1.f;
						d.V += static_cast<float>(a.cost)
							* (gref / gd);
					}
					const float r1 = best.strike_rmin < 1e29f
						? best.strike_rmin
						: (best.bnd_rp < 1e29f ? best.bnd_rp
							: best_any_res);
					if (r1 < r0 - 1e-3f) {
						d.stall = 0;
						d.lane_stall = 0;
						d.R = r1;
					} else {
						d.stall++;
						d.lane_stall++;
						// A stalled LANE yields to the next eligible
						// method; the domain itself stays competitive.
						if (fair_sched && d.lane_stall >= 2) {
							d.lane = d.lane >= 5 ? 2 : d.lane + 1;
							d.lane_stall = 0;
						}
					}
					d.kappa = static_cast<float>(d.stall);
					if (best.ok && best.H > d.L)
						d.L = best.H;
				}
				for (int i = 0; i < n_dom && i < 6; ++i) {
					best.dom_U[i] = dom[static_cast<size_t>(i)].U_order;
					best.dom_state[i] = dom[static_cast<size_t>(i)].state;
					best.dom_L[i] = dom[static_cast<size_t>(i)].L;
					best.dom_spent[i] =
						dom[static_cast<size_t>(i)].spent;
				}
			}
		}
		best.tol_final = tol;
		if (best.evals_phase1 == 0)
			best.evals_phase1 = best.evals;
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
					const float wl = ll.TurnRad(Strafe::kPerp, 1.f);
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


	// ================= LAZY REFINEMENT (operator level) =================
	// See SolverEntrance.h for the contract. The engine underneath is the
	// PRODUCTION legacy RefSolve: sessions 12e-12k measured it at 29/32 on
	// the frozen known-reachable AirRec L1 fixture against scheduler-v1's
	// 18/32 at equal compute, in boundary mode where domain coverage and
	// fairness cannot be the explanation. So the strong local optimizer
	// stays, and the anytime contract wraps it.

	static const RefProfile kProfiles[] = {
		// name        budget  precision  shoot_m  id
		{ "coarse",       600,      4.f,        2, 0x45460100u },
		{ "medium",      1800,      2.f,        2, 0x45460101u },
		{ "fine",        3600,      1.f,        2, 0x45460102u },
		{ "exhaustive",  9000,      1.f,        2, 0x45460103u },
	};

	const RefProfile* Profiles(int* count) {
		if (count)
			*count = static_cast<int>(sizeof(kProfiles)
				/ sizeof(kProfiles[0]));
		return kProfiles;
	}

	bool RefineStep(QueryState* st, const PlayerState& entry,
	                const World& w, const MoveParams& p,
	                const Route::Graph& g, int face_idx, const Vec3& q,
	                float radius, int n_hint, float zmin,
	                const Steer::CtlState& entry_ctl) {
		int np = 0;
		const RefProfile* prof = Profiles(&np);
		if (!st || st->level >= np)
			return false;
		if (face_idx < 0 || face_idx >= static_cast<int>(g.faces.size()))
			return false;
		const RefProfile& pr = prof[st->level];
		const Route::Face& fc = g.faces[static_cast<size_t>(face_idx)];

		// ---- U first: it does not depend on the search at all, so the
		// ceiling is available even at level 0 with no witness. A query
		// that finds nothing still returns a real (L, U, UNRESOLVED).
		const int N = n_hint > 8 ? n_hint : 60;
		unsigned bid = 0;
		const float u_run = UCertifiedFace(fc.n, entry.pos, entry.vel,
			entry.gravity_scale, p, q, radius, zmin, N + 48,
			-3.14159265f, 3.14159265f, &bid);
		if (u_run < st->U) {
			st->U = u_run;
			st->U_bound_id = bid;
		}

		// ---- the atomic action: one COMPLETE local solve, immutably
		// configured. The caller's remaining budget never reaches in.
		RefTune tn;
		tn.shoot_m = pr.shoot_m;
		tn.precision = pr.precision;
		tn.scheduler = 0;         // production local engine
		int fl = 0;
		RefResult rr = RefSolve(entry, w, p, g, face_idx, q, radius, N,
			pr.budget, false, &fl, zmin, entry_ctl, nullptr, nullptr,
			nullptr, &tn);
		st->evals += rr.evals;
		st->level++;
		st->applied.push_back(pr.id);

		// ---- monotone merge. A worse run can never displace a better
		// witness, and a search that failed changes nothing but the
		// level counter.
		if (rr.ok && rr.H > st->L) {
			st->L = rr.H;
			st->has_L = true;
			st->wside = rr.wside;
			st->wcosa = rr.wcosa;
			st->horizon = rr.horizon;
			st->L_res = rr.strike_rmin;
		}
		// ---- the CONTINUATION FRONTIER, merged monotonically PER
		// LANE: winner, tangent, exactness rungs, heading intervals.
		// A lane once exposed is never lost; its L only rises. This is
		// the payload that composes with ExitField - the headline L
		// above is only the heatmap projection.
		auto merge_lane = [&](unsigned lane, float L,
			const std::vector<signed char>& sd,
			const std::vector<float>& cs, int hz2, float res,
			float th) {
			if (sd.empty() || L <= -1e29f)
				return;
			// The replay horizon is a property of the WITNESS, not of
			// whichever lane happened to win the solve: schedule
			// length + the same slack the search flew with.
			const int hz = static_cast<int>(sd.size()) + 8;
			(void)hz2;
			for (BoardTransition& t : st->transitions)
				if (t.lane == lane) {
					if (L > t.L) {
						t.L = L;
						t.side = sd;
						t.cosa = cs;
						t.horizon = hz;
						t.res = res;
						t.th = th;
					}
					return;
				}
			BoardTransition t;
			t.lane = lane;
			t.L = L;
			t.side = sd;
			t.cosa = cs;
			t.horizon = hz;
			t.res = res;
			t.th = th;
			st->transitions.push_back(t);
		};
		if (rr.ok)
			merge_lane(0x0001u, rr.H, rr.wside, rr.wcosa, rr.horizon,
				rr.strike_rmin, atan2f(rr.flight.v1.Y,
					rr.flight.v1.X));
		if (rr.tan_ok)
			merge_lane(0x0300u, rr.tan_H, rr.tan_wside, rr.tan_wcosa,
				rr.tan_horizon, 0.f, atan2f(rr.tan_flight.v1.Y,
					rr.tan_flight.v1.X));
		for (int r2 = 0; r2 < 6; ++r2)
			merge_lane(0x0100u + static_cast<unsigned>(r2),
				rr.rung_E[r2], rr.rung_side[r2], rr.rung_cosa[r2],
				rr.horizon > 0 ? rr.horizon : N + 8, rr.rung_res[r2],
				0.f);
		for (int i2 = 0; i2 < 6; ++i2)
			merge_lane(0x0200u + static_cast<unsigned>(i2),
				rr.iv_L[i2], rr.iv_side[i2], rr.iv_cosa[i2],
				rr.horizon > 0 ? rr.horizon : N + 8, rr.iv_res[i2],
				rr.iv_th[i2]);
		// Status is NEVER downgraded by a failed search. Only
		// MarkIrrelevant, with a certified ceiling and an incumbent,
		// may leave UNRESOLVED.
		return true;
	}

	Air::Result ReplayBoardTransition(const BoardTransition& tr,
	                                  const PlayerState& entry,
	                                  const World& w,
	                                  const MoveParams& p,
	                                  const Route::Graph& g,
	                                  int face_idx, const Vec3& q) {
		Air::Target vt;
		vt.face = face_idx;
		vt.dot_cap = 3000.f;
		vt.aim = q;
		vt.max_ticks = tr.horizon;
		return Air::FlyWishSchedule(entry, w, p, vt, g, tr.side,
			tr.cosa, tr.horizon);
	}
} // namespace Entrance
} // namespace Solver
