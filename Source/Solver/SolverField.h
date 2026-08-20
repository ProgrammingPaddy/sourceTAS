#pragma once

// THE HOTSPOT FIELD (user's framework observation, 2026-08-17):
// from a state (position + velocity) and a target face, every
// reachable landing point Q has an EFFECTIVE ENERGY under the best
// possible approach and the optimal board at Q. The field is the
// design's exit-manifold x board-window intersection (SolverRebuild
// 3.2) made concrete as a scalar map over the face:
//
//   - flight time to Q's altitude: EXACT ballistic roots (M1.1;
//     vertical air motion is pure gravity) - ascending + descending
//     arrivals both evaluated;
//   - reachability: 2D distance vs the gain-optimal DMax (M1.1);
//   - arrival speed: the gain-law bound s^2 <= s0^2 + cap^2 n;
//   - optimal board at Q: the M1.2 tangency law - residual |dot|
//     max(0, |vz*nz| - s*h), and the tangent heading
//     cos(phi) = -vz*nz/(s*h) about the face normal azimuth;
//   - turn feasibility: the total heading change the path must
//     absorb (reach Q AND arrive at the tangent heading) vs the
//     certified free-turn budget (Strafe::Law TurnRad at full gain,
//     integrated over the flight) and the braking-turn ceiling.
//
// The field is an OPTIMISTIC estimator (admissible bounds; occlusion
// not modeled): it GUIDES and CULLS, the exact engine decides.
// Braking cost is not priced in v1 - samples needing braking are
// FLAGGED (free_turn = false) and disfavored, not forbidden.

#include <float.h>
#include <math.h>

#include <vector>

#include "SolverBoard.h"
#include "SolverEnvelope.h"
#include "SolverRoute.h"
#include "SolverSteer.h"
#include "SolverStrafe.h"

namespace Solver {
namespace Field {

	// The LEG-AFTER context: what the ride at this face must set up.
	// Enables RUNWAY (testimony 2.8b): per landing point, the space
	// remaining on the face toward the departure serving this target
	// vs the space the required action needs.
	struct NextCtx {
		bool has = false;
		bool is_zone = false;
		Vec3 pt;                 // next face centroid (is_zone=false)
		Vec3 zmin, zmax;         // zone volume (is_zone=true)
	};

	struct Sample {
		Vec3  q;                 // landing point (on the face plane)
		bool  in_face = false;   // inside the polygon (grid cells
		                         // outside are kept for the frame)
		bool  reachable = false;
		bool  free_turn = false; // tangent arrival within the free-
		                         // turn budget (no braking needed)
		bool  ascending = false; // arrives before apex
		// RUNWAY (with a NextCtx): in-plane space from Q toward the
		// next objective vs the space the action needs (turn arc +
		// climb distance, law-derived). viable = enough room.
		float run_avail = 0.f;
		float run_req = 0.f;
		bool  run_viable = true; // true when no ctx (nothing owed)
		float n = 0.f;           // flight ticks (real-valued root)
		float s_arr = 0.f;       // gain-law arrival speed bound (2D)
		float vz_arr = 0.f;
		float phi = 0.f;         // optimal board heading (world az)
		float residual = 0.f;    // unavoidable |dot| at Q (0 = tangent)
		float turn_need = 0.f;   // radians the path must absorb
		float turn_free = 0.f;   // free-turn budget over the flight
		// TOTAL mechanical energy after the best board: kinetic
		// (s^2 + vz^2 - residual^2) PLUS the potential kept above
		// the face bottom (2g*(q.z - zmin)) - a high tangent board
		// keeps its height for the ride to convert losslessly, so
		// counting only kinetic wrongly favored falling to the
		// bottom and slamming (first fieldgate run showed exactly
		// that: every best sample sat at zmin).
		float e_eff = -1e30f;
	};

	struct FaceMap {
		int  face = -1;
		// Grid frame on the plane (for rendering + neighbor lookup).
		Vec3 origin;             // grid (0,0) sample position
		Vec3 ud, vd;             // in-plane axes (unit)
		int  nu = 0, nv = 0;
		float du = 0.f, dv = 0.f;
		std::vector<Sample> samples;   // nu*nv, u-major
		int  best = -1;          // argmax e_eff (free_turn preferred)
		float e_lo = 0.f, e_hi = 0.f;  // reachable e_eff range
	};

	// Minimal total heading change for a path that starts at heading
	// h0, must make net progress along bearing theta, and arrives at
	// heading phi_w: monotone sweep when theta lies inside the wrapped
	// arc h0->phi_w, else the dogleg sum.
	inline float TurnNeed(float h0, float theta, float phi_w) {
		const float a = Steer::WrapPi(phi_w - h0);
		const float b = Steer::WrapPi(theta - h0);
		const bool inside = (a >= 0.f) ? (b >= 0.f && b <= a)
		                               : (b <= 0.f && b >= a);
		if (inside)
			return fabsf(a);
		return fabsf(b) + fabsf(Steer::WrapPi(phi_w - theta));
	}

	// Free-turn budget over an n-tick flight from speed s0 (full-gain
	// turning only, per the certified law; 8-bucket integral).
	inline float FreeTurnBudget(float s0, float n, const MoveParams& p,
	                            bool ducked) {
		if (n <= 0.f)
			return 0.f;
		float total = 0.f;
		const int buckets = 8;
		for (int b = 0; b < buckets; ++b) {
			const float k = n * (static_cast<float>(b) + 0.5f)
				/ static_cast<float>(buckets);
			const float s = Envelope::SMax(s0,
				static_cast<int>(k), p);
			Strafe::TickLaw law = Strafe::Law(p, s, 1.f, ducked);
			total += law.TurnRad(Strafe::kPerp, 1.f) * (n
				/ static_cast<float>(buckets));
		}
		return total;
	}

	// In-plane distance from Q along direction u (unit, in-plane) to
	// the face polygon boundary - the raw RUNWAY ray. Winding-
	// agnostic (edge normals oriented outward against the centroid).
	inline float RunwayRay(const Route::Face& f, const Vec3& q,
	                       const Vec3& u) {
		float best = FLT_MAX;
		const size_t nv = f.verts.size();
		for (size_t i = 0; i < nv; ++i) {
			const Vec3& a = f.verts[i];
			const Vec3& b = f.verts[(i + 1) % nv];
			Vec3 m = Cross(b - a, f.n);
			const float ml = Len(m);
			if (ml < 1e-6f)
				continue;
			m = Scale(m, 1.f / ml);
			if (Dot(m, f.centroid - a) > 0.f)
				m = Scale(m, -1.f);
			const float denom = Dot(m, u);
			if (denom <= 1e-6f)
				continue;   // ray moves away from this edge
			const float t = (Dot(m, a) - Dot(m, q)) / denom;
			if (t >= 0.f && t < best)
				best = t;
		}
		return best == FLT_MAX ? 0.f : best;
	}

	// The braking-turn per-tick ceiling at speed s (peak of the
	// cosa < 0 branch of the certified turn curve).
	inline float BrakeTurnPeak(float s, const MoveParams& p,
	                           bool ducked) {
		Strafe::TickLaw law = Strafe::Law(p, s, 1.f, ducked);
		float best = law.TurnRad(Strafe::kPerp, 1.f);
		for (int i = 1; i <= 16; ++i) {
			// Literal TRUE-basis scan points (no stored value can reach
			// this function - it takes only speed/params/ducked, so the
			// bound is convention-clean by construction).
			const float c = -static_cast<float>(i) / 16.f;
			const float s2 = 1.f - c * c;
			const float tr = law.TurnRad(Strafe::TrueWishCos(c),
				s2 > 0.f ? sqrtf(s2) : 0.f);
			if (tr > best)
				best = tr;
		}
		return best;
	}

	inline FaceMap Compute(const Vec3& pos, const Vec3& vel,
	                       const Route::Face& face, const MoveParams& p,
	                       bool ducked, float grid = 32.f,
	                       int max_ticks = 300,
	                       float gravity_scale = 1.f,
	                       const NextCtx* ctx = nullptr) {
		FaceMap m;
		m.face = -1;
		// In-plane frame: ud = horizontal lateral (contour), vd = the
		// in-plane up-slope direction.
		const float hn = sqrtf(face.n.X * face.n.X
			+ face.n.Y * face.n.Y);
		if (hn < 1e-4f || face.verts.size() < 3)
			return m;
		m.ud = Vec3(-face.n.Y / hn, face.n.X / hn, 0.f);
		Vec3 vd = Cross(face.n, m.ud);
		if (vd.Z < 0.f)
			vd = Scale(vd, -1.f);
		const float vl = Len(vd);
		if (vl < 1e-4f)
			return m;
		m.vd = Scale(vd, 1.f / vl);
		// Polygon bounds in the frame.
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
		m.du = grid;
		m.dv = grid;
		m.nu = static_cast<int>((uhi - ulo) / grid) + 1;
		m.nv = static_cast<int>((whi - wlo) / grid) + 1;
		if (m.nu < 1) m.nu = 1;
		if (m.nv < 1) m.nv = 1;
		m.origin = face.centroid + Scale(m.ud, ulo + 0.5f * grid)
			+ Scale(m.vd, wlo + 0.5f * grid);
		const float g = p.gravity * gravity_scale;
		const float s0 = Len2D(vel);
		const float vz0 = vel.Z;
		const float h0 = s0 > 1.f ? atan2f(vel.Y, vel.X) : 0.f;
		const float psi = atan2f(face.n.Y, face.n.X);
		m.samples.resize(static_cast<size_t>(m.nu)
			* static_cast<size_t>(m.nv));
		float best_e = -1e30f, best_e_any = -1e30f;
		int best_i = -1, best_i_any = -1;
		m.e_lo = FLT_MAX;
		m.e_hi = -FLT_MAX;
		for (int iv = 0; iv < m.nv; ++iv)
		for (int iu = 0; iu < m.nu; ++iu) {
			Sample& sm = m.samples[static_cast<size_t>(iv) * m.nu
				+ iu];
			sm.q = m.origin
				+ Scale(m.ud, static_cast<float>(iu) * grid)
				+ Scale(m.vd, static_cast<float>(iv) * grid);
			sm.in_face = Board::EdgeDistOut(face, sm.q) <= 0.f;
			if (!sm.in_face)
				continue;
			// Ballistic roots to Q's altitude (M1.1, exact).
			const float dzq = sm.q.Z - pos.Z;
			const float disc = vz0 * vz0 - 2.f * g * dzq;
			if (disc < 0.f)
				continue;   // above the reachable apex
			const float sq = sqrtf(disc);
			const float roots[2] = { (vz0 - sq) / (g * p.dt),
			                         (vz0 + sq) / (g * p.dt) };
			const float dx = sm.q.X - pos.X;
			const float dy = sm.q.Y - pos.Y;
			const float dist = sqrtf(dx * dx + dy * dy);
			const float theta = atan2f(dy, dx);
			for (int ri = 0; ri < 2; ++ri) {
				const float n = roots[ri];
				if (n < 2.f
					|| n > static_cast<float>(max_ticks))
					continue;
				if (dist > Envelope::DMax(s0,
					static_cast<int>(n) + 1, p)
					+ Board::kHullCenterSlack)
					continue;
				const float s_arr = Envelope::SMax(s0,
					static_cast<int>(n), p);
				const float vz_arr = vz0 - g * p.dt * n;
				const float res = Board::MinApproachDot(
					s_arr, vz_arr, face.n);
				if (res == FLT_MAX)
					continue;   // no approaching aim exists
				// Tangent (or min-loss) heading about the face
				// normal azimuth; both branches, pick the one
				// the path can absorb cheapest.
				float cphi = s_arr * hn > 1e-4f
					? -vz_arr * face.n.Z / (s_arr * hn) : 0.f;
				if (cphi > 1.f) cphi = 1.f;
				if (cphi < -1.f) cphi = -1.f;
				const float phi_off = acosf(cphi);
				const float cands[2] = {
					Steer::WrapPi(psi + phi_off),
					Steer::WrapPi(psi - phi_off) };
				float need = FLT_MAX, phi_w = cands[0];
				for (int ci = 0; ci < 2; ++ci) {
					const float tn = TurnNeed(h0, theta,
						cands[ci]);
					if (tn < need) {
						need = tn;
						phi_w = cands[ci];
					}
				}
				const float tfree = FreeTurnBudget(s0, n, p,
					ducked);
				const float tmax = tfree + BrakeTurnPeak(
					s_arr, p, ducked) * n * 0.5f;
				if (need > tmax)
					continue;   // not absorbable at all
				// THE PATH PRICE (user: the efficient frontier is
				// baked into the map as VALUE, reachability stays
				// physics-true - over-rate approaches stay in,
				// priced). Turn demanded beyond the free budget is
				// bought at the certified braking exchange
				// (max turn per tick vs its exact speed cost);
				// the energy spent comes off this cell's value.
				// Straight+snap stays reachable and reads COLD.
				// (Two exclusion forms were boardproof-falsified:
				// the engine reaches such cells by braking.)
				float brake_cost2 = 0.f;
				if (need > tfree) {
					float excess = need - tfree;
					float vb2 = s_arr;
					int it = 0;
					while (excess > 0.f && it++ < 120
						&& vb2 > 100.f) {
						Strafe::TickLaw lb = Strafe::Law(p, vb2,
							1.f, ducked);
						// Cheapest COST PER RADIAN, not fastest
						// turn - the exchange is convex and a real
						// approach buys turn at the efficient
						// angle (pricing at the max-rate point
						// overcharged the human's own board to
						// COLD - fieldgate-caught).
						float btr = 0.f;
						float beff = FLT_MAX, bcost = 0.f;
						for (int ci = 1; ci <= 8; ++ci) {
							const float cc = -static_cast<float>(
								ci) / 8.f;
							const float ss = 1.f - cc * cc;
							// Literal TRUE-basis scan points (nothing
							// stored reaches the priced-brake bound).
							const float tr2 = lb.TurnRad(
								Strafe::TrueWishCos(cc),
								ss > 0.f ? sqrtf(ss) : 0.f);
							if (tr2 <= 1e-6f)
								continue;
							const float nv2 = lb.NewSpeed2(
								Strafe::TrueWishCos(cc));
							const float c2 = vb2 * vb2 - nv2;
							const float eff = c2 / tr2;
							if (eff < beff) {
								beff = eff;
								btr = tr2;
								bcost = c2;
							}
						}
						if (btr <= 1e-5f)
							break;
						excess -= btr;
						const float nvv = vb2 * vb2 - bcost;
						vb2 = nvv > 0.f ? sqrtf(nvv) : 0.f;
					}
					brake_cost2 = s_arr * s_arr - vb2 * vb2;
					if (brake_cost2 < 0.f)
						brake_cost2 = 0.f;
				}
				const float e = s_arr * s_arr + vz_arr * vz_arr
					- res * res - brake_cost2
					+ 2.f * g * (sm.q.Z - face.zmin);
				const bool ft = need <= tfree;
				const bool better = e > sm.e_eff
					|| (!sm.reachable);
				if (better) {
					sm.reachable = true;
					sm.free_turn = ft;
					sm.ascending = ri == 0;
					sm.n = n;
					sm.s_arr = s_arr;
					sm.vz_arr = vz_arr;
					sm.phi = phi_w;
					sm.residual = res;
					sm.turn_need = need;
					sm.turn_free = tfree;
					sm.e_eff = e;
				}
			}
			// RUNWAY (testimony 2.8b): space from Q toward the next
			// objective vs the space the required action needs -
			// turn arc at the free rate + climb distance at the
			// face slope, both law-derived. Non-viable landings
			// leave no room to do what's needed.
			if (sm.reachable && ctx && ctx->has) {
				Vec3 tgt = ctx->pt;
				float z_t = ctx->pt.Z;
				if (ctx->is_zone) {
					tgt = sm.q;
					if (tgt.X < ctx->zmin.X) tgt.X = ctx->zmin.X;
					if (tgt.X > ctx->zmax.X) tgt.X = ctx->zmax.X;
					if (tgt.Y < ctx->zmin.Y) tgt.Y = ctx->zmin.Y;
					if (tgt.Y > ctx->zmax.Y) tgt.Y = ctx->zmax.Y;
					tgt.Z = ctx->zmin.Z;
					z_t = ctx->zmin.Z;
				}
				Vec3 dip = tgt - sm.q;
				dip = dip - Scale(face.n, Dot(dip, face.n));
				const float dl = Len(dip);
				if (dl > 1.f) {
					const Vec3 u = Scale(dip, 1.f / dl);
					sm.run_avail = RunwayRay(face, sm.q, u);
					const float post = sqrtf(
						sm.s_arr * sm.s_arr
						+ sm.vz_arr * sm.vz_arr
						- sm.residual * sm.residual > 0.f
						? sm.s_arr * sm.s_arr
							+ sm.vz_arr * sm.vz_arr
							- sm.residual * sm.residual : 0.f);
					const float pv = post > 1.f ? post : 1.f;
					// Turn arc: arrival heading -> departure
					// heading at the free-turn rate.
					const float az_dep = atan2f(u.Y, u.X);
					const float turn = fabsf(
						Steer::WrapPi(az_dep - sm.phi));
					Strafe::TickLaw tl = Strafe::Law(p, pv, 1.f,
						ducked);
					const float rate = tl.TurnRad(Strafe::kPerp, 1.f);
					const float turn_dist = rate > 1e-5f
						? turn / rate * pv * p.dt : 1e9f;
					// Climb: the ride TRAVELS the runway first -
					// the departure is at the far boundary along
					// the action direction, so the ballistic
					// altitude requirement is measured from THERE
					// (flat-exit bound), and the climb is priced
					// at the face's max climb rate (hn per unit
					// in-plane distance - the up-slope Z).
					const Vec3 dp = sm.q
						+ Scale(u, sm.run_avail);
					const float dx2 = tgt.X - dp.X;
					const float dy2 = tgt.Y - dp.Y;
					const float l2d = sqrtf(dx2 * dx2 + dy2 * dy2);
					// Flat exit overstates the requirement: a
					// departure may ascend up to the in-plane
					// limit (vz <= pv*hn), which buys hn*L of
					// effective altitude over the flight.
					const float need_z = z_t
						+ 0.5f * g * (l2d * l2d) / (pv * pv)
						- hn * l2d;
					const float dz_req = need_z - dp.Z > 0.f
						? need_z - dp.Z : 0.f;
					float climb_dist = 0.f;
					if (dz_req > 0.f) {
						climb_dist = hn > 0.02f
							? dz_req / hn : 1e9f;
					}
					sm.run_req = turn_dist + climb_dist;
					sm.run_viable = sm.run_avail >= sm.run_req;
				}
			}
			if (sm.reachable) {
				if (sm.e_eff < m.e_lo) m.e_lo = sm.e_eff;
				if (sm.e_eff > m.e_hi) m.e_hi = sm.e_eff;
				const int idx = iv * m.nu + iu;
				// Selection: viable+free-turn first (runway decides
				// among energy-equivalent hot points); fallback
				// ladder below.
				if (sm.run_viable && sm.free_turn
					&& sm.e_eff > best_e) {
					best_e = sm.e_eff;
					best_i = idx;
				}
				if (sm.e_eff > best_e_any) {
					best_e_any = sm.e_eff;
					best_i_any = idx;
				}
			}
		}
		// Fallback ladder when no viable+free-turn sample exists:
		// best viable, then best free-turn, then best overall.
		if (best_i < 0) {
			float be = -1e30f;
			for (size_t i = 0; i < m.samples.size(); ++i) {
				const Sample& sm = m.samples[i];
				if (sm.reachable && sm.run_viable
					&& sm.e_eff > be) {
					be = sm.e_eff;
					best_i = static_cast<int>(i);
				}
			}
		}
		if (best_i < 0) {
			float be = -1e30f;
			for (size_t i = 0; i < m.samples.size(); ++i) {
				const Sample& sm = m.samples[i];
				if (sm.reachable && sm.free_turn
					&& sm.e_eff > be) {
					be = sm.e_eff;
					best_i = static_cast<int>(i);
				}
			}
		}
		m.best = best_i >= 0 ? best_i : best_i_any;
		m.face = face.brush;   // caller knows the graph index
		return m;
	}

	// ================= THE EXIT MAP =================
	// (user's exit-heatmap proposal, 2026-08-17 - the dual of the
	// board field). From a BOARD STATE on a face (post-board position
	// + velocity), every candidate departure = a point on the face x
	// an IN-PLANE exit direction, valued by THE QUALITY OF THE BOARD
	// MAP IT INDUCES on the next target: an optimal exit is one whose
	// induced next-face hotspot map is best. Exits whose induced map
	// has NO reachable landing are DOOMED (the user's "off the bottom
	// into the void" cull - measurable before exiting).
	//
	// Physics of a departure (measured, session 2026-08-16): exit
	// velocity is IN-PLANE along the chosen direction (the duck-off
	// separation keeps in-plane velocity; an edge crossing does too).
	// A DESCENDING in-plane direction stays pressed to the plane -
	// mid-face departures must ASCEND (the tapes' "cross flat-or-
	// ascending" fact); descending directions only depart at the
	// boundary. Ride bookkeeping to the departure is law-priced:
	// 2g potential exchange + wish work <= cap^2/tick, free-turn
	// check at ride speed. All OPTIMISTIC closed forms; guidance +
	// doom culling only, the exact engine decides.

	// Functional candidates for "how good is an induced board map"
	// (to be settled by exitgate validation against the human run,
	// never by argument):
	//   q_max  - best induced e_eff (best-case board);
	//   area   - u^2 of cells within hot_frac of q_max (margin /
	//            robustness - runway's sibling: a big hot zone is
	//            loose margins);
	//   mass   - sum(e_eff * cell_area) over those hot cells
	//            (area x quality).
	// hot_frac 0.85 = the same threshold fieldgate uses to classify
	// a board HOT (consistent, not fitted).
	struct Quality {
		bool  any = false;       // any reachable landing (false = DOOMED)
		float q_max = -1e30f;
		// POTENTIAL-priced best: e_eff - wish_rate * flight ticks (the
		// M2 telescoping-anchor currency; raw e_eff rewards long paths
		// because the optimistic bookkeeping credits cap^2/tick - the
		// potential subtracts exactly that, leaving the REAL
		// differences: residual loss, height kept, gravity exchange).
		float q_pot = -1e30f;
		float area = 0.f;
		float mass = 0.f;
		float viable_area = 0.f; // hot AND runway-viable
		Vec3  argmax;            // argmax of q_pot (the priced field)
		bool  have_argmax = false;
	};

	inline Quality MapQuality(const FaceMap& m, const MoveParams& p,
	                          float hot_frac = 0.85f) {
		Quality q;
		const float wr = p.air_speed_cap * p.air_speed_cap;
		// The priced argmax honors the RUNWAY LAW ladder exactly like
		// FaceMap.best (2.8b: runway is the selector among energy-
		// equivalent hot points): viable+free-turn > viable > free-
		// turn > any. Without the ladder the argmax picks energetic
		// DESCENDING BOTTOM landings that strand the next leg
		// (measured: leg-0 taps at -60 @ 946 whose leg 1 could only
		// walk the valley floor on friction, -712k).
		float hi = -1e30f, hip = -1e30f;
		int argp = -1;
		for (int tier = 0; tier < 4 && argp < 0; ++tier) {
			for (size_t i = 0; i < m.samples.size(); ++i) {
				const Sample& sm = m.samples[i];
				if (!sm.reachable)
					continue;
				if (tier == 0 && !(sm.run_viable && sm.free_turn))
					continue;
				if (tier == 1 && !sm.run_viable)
					continue;
				if (tier == 2 && !sm.free_turn)
					continue;
				const float vp = sm.e_eff - wr * sm.n;
				if (vp > hip) {
					hip = vp;
					argp = static_cast<int>(i);
				}
			}
		}
		for (const Sample& sm : m.samples)
			if (sm.reachable && sm.e_eff > hi)
				hi = sm.e_eff;
		if (argp < 0)
			return q;
		q.any = true;
		q.q_max = hi;
		q.q_pot = hip;
		q.argmax = m.samples[argp].q;
		q.have_argmax = true;
		const float ca = m.du * m.dv;
		const float thr = hi * hot_frac;   // e_eff >= 0 by construction
		for (const Sample& sm : m.samples) {
			if (!sm.reachable || sm.e_eff < thr)
				continue;
			q.area += ca;
			q.mass += sm.e_eff * ca;
			if (sm.run_viable)
				q.viable_area += ca;
		}
		return q;
	}

	// What the exit serves: the next board face (+ its own leg-after
	// context for induced runway), or the end zone.
	struct NextTarget {
		const Route::Face* face = nullptr;
		int  face_idx = -1;
		bool is_zone = false;
		Vec3 zmin, zmax;
		NextCtx after;
	};

	// Earliest tick a ballistic flight from (pos, s2d horizontal, vz0)
	// can be inside the zone volume xy at-or-above its floor
	// (optimistic: gain-law coverage of the clamped-rim distance).
	// Returns -1 = DOOMED (never covers while at altitude).
	inline float ZoneReach(const Vec3& pos, float s2d, float vz0,
	                       const Vec3& zmin, const Vec3& zmax,
	                       const MoveParams& p, int max_ticks,
	                       float* margin_z) {
		Vec3 rim = pos;
		if (rim.X < zmin.X) rim.X = zmin.X;
		if (rim.X > zmax.X) rim.X = zmax.X;
		if (rim.Y < zmin.Y) rim.Y = zmin.Y;
		if (rim.Y > zmax.Y) rim.Y = zmax.Y;
		const float dx = rim.X - pos.X;
		const float dy = rim.Y - pos.Y;
		const float l2d = sqrtf(dx * dx + dy * dy);
		const float floor_z = zmin.Z;
		for (int n = 1; n <= max_ticks; ++n) {
			const float z = Envelope::ZAfter(pos.Z, vz0, n, p);
			if (z < floor_z) {
				if (Envelope::VzAfter(vz0, n, p) < 0.f)
					break;   // falling below the floor - done
				continue;
			}
			if (Envelope::DMax(s2d, n, p)
				+ Board::kHullCenterSlack >= l2d) {
				if (margin_z)
					*margin_z = z - floor_z;
				return static_cast<float>(n);
			}
		}
		return -1.f;
	}

	// STANDING LAW B-E (ExitField, 2026-08-19, Docs/ExitFieldSpec.md):
	// ExitSample/ExitMap below are an ANALYTIC ADVISORY ESTIMATOR (law-
	// booked ride time, closed-form exit speed). They order work; they
	// are NOT the ExitField transition relation R_F(B), whose truth is
	// only ever an exact Ride::FlyRideSchedule witness.
	struct ExitSample {
		Vec3  pt;                // departure point (on the face)
		Vec3  dir;               // in-plane unit exit direction
		float pv = 0.f;          // exit speed (3D, in-plane)
		float e_exit = 0.f;      // pv^2 + 2g*(pt.z - face.zmin)
		float ride_ticks = 0.f;  // law-booked ride time board -> pt
		bool  edge = false;      // boundary departure (vs mid-face
		                         // ascending)
		bool  turn_ok = true;    // heading reachable at the free rate
		Quality q;               // induced quality on the next target
		// The time-priced functional: induced q_pot minus the ride's
		// own wish credit (potential-ledger units end to end).
		float fpot = -1e30f;
		float zn = -1.f;         // zone target: est arrival ticks
		Vec3  q_argmax;          // induced best landing (face target)
		int   iu = 0, iv = 0;    // anchor grid cell (rendering)
	};

	struct ExitMap {
		int  face = -1;
		Vec3 origin;             // grid frame (as FaceMap)
		Vec3 ud, vd;
		int  nu = 0, nv = 0;
		float du = 0.f, dv = 0.f;
		int  ndirs = 0;
		std::vector<ExitSample> samples;
		// argmax per functional (zone target: all = earliest arrival).
		// -1 = every exit doomed.
		int  best_max = -1, best_area = -1, best_mass = -1;
		int  best_pot = -1;
	};

	// One exit candidate evaluated in the map's own currency: ride
	// bookkeeping bq -> pt, then the induced map along dir. Used by
	// ComputeExit per grid cell x direction, and by exitgate to score
	// an ACTUAL exit (exact point + direction) on the same footing.
	// ok=false (returned flag) when the ride can't deliver this exit.
	inline bool EvalExitSample(const Vec3& bq, const Vec3& bvel,
	                           const Route::Face& face,
	                           const NextTarget& nt,
	                           const MoveParams& p, bool ducked,
	                           const Vec3& pt, const Vec3& dir,
	                           float igrid, int max_ticks,
	                           ExitSample* out) {
		const float g = p.gravity;
		const float wish_rate = p.air_speed_cap * p.air_speed_cap;
		const float vb2 = Dot(bvel, bvel);
		const float vb = sqrtf(vb2);
		const float s2b = Len2D(bvel);
		const float az0 = s2b > 1.f ? atan2f(bvel.Y, bvel.X) : 0.f;
		const float L = Len(pt - bq);
		const float dz = pt.Z - bq.Z;
		float pv2 = vb2 - 2.f * g * dz;
		float ticks = L / (0.5f * (vb
			+ sqrtf(pv2 > 10000.f ? pv2 : 10000.f)) * p.dt);
		pv2 = vb2 - 2.f * g * dz + wish_rate * ticks;
		if (pv2 < 100.f * 100.f)
			return false;   // the ride dies before this exit
		ticks = L / (0.5f * (vb + sqrtf(pv2)) * p.dt);
		pv2 = vb2 - 2.f * g * dz + wish_rate * ticks;
		const float pv = sqrtf(pv2);
		ExitSample es;
		es.pt = pt;
		es.dir = dir;
		es.pv = pv;
		es.e_exit = pv2 + 2.f * g * (pt.Z - face.zmin);
		es.ride_ticks = ticks;
		const float d2 = Len2D(dir);
		if (s2b > 1.f && d2 > 1e-3f) {
			const float need = fabsf(Steer::WrapPi(
				atan2f(dir.Y, dir.X) - az0));
			es.turn_ok = need <= FreeTurnBudget(s2b, ticks, p, ducked);
		}
		if (nt.is_zone) {
			float mz = 0.f;
			es.zn = ZoneReach(pt, pv * d2, pv * dir.Z,
				nt.zmin, nt.zmax, p, max_ticks, &mz);
			es.q.any = es.zn >= 0.f;
			if (es.q.any) {
				const float sa = Envelope::SMax(pv * d2,
					static_cast<int>(es.zn), p);
				es.q.q_max = sa * sa + 2.f * g * mz;
				es.fpot = -(es.ride_ticks + es.zn);
			}
		} else if (nt.face) {
			FaceMap fm = Compute(pt, Scale(dir, pv), *nt.face, p,
				ducked, igrid, max_ticks, 1.f, &nt.after);
			es.q = MapQuality(fm, p);
			if (es.q.have_argmax)
				es.q_argmax = es.q.argmax;
			if (es.q.any)
				es.fpot = es.q.q_pot - wish_rate * es.ride_ticks;
		}
		*out = es;
		return true;
	}

	inline ExitMap ComputeExit(const Vec3& bq, const Vec3& bvel,
	                           const Route::Face& face,
	                           const NextTarget& nt,
	                           const MoveParams& p, bool ducked,
	                           float grid = 48.f, int ndirs = 12,
	                           float igrid = 48.f, int max_ticks = 300) {
		ExitMap em;
		const float hn = sqrtf(face.n.X * face.n.X
			+ face.n.Y * face.n.Y);
		if (hn < 1e-4f || face.verts.size() < 3)
			return em;
		em.ud = Vec3(-face.n.Y / hn, face.n.X / hn, 0.f);
		Vec3 vd = Cross(face.n, em.ud);
		if (vd.Z < 0.f)
			vd = Scale(vd, -1.f);
		const float vl = Len(vd);
		if (vl < 1e-4f)
			return em;
		em.vd = Scale(vd, 1.f / vl);
		float ulo = FLT_MAX, uhi = -FLT_MAX;
		float wlo = FLT_MAX, whi = -FLT_MAX;
		for (const Vec3& v : face.verts) {
			const Vec3 dvtx = v - face.centroid;
			const float cu = Dot(dvtx, em.ud);
			const float cv = Dot(dvtx, em.vd);
			if (cu < ulo) ulo = cu;
			if (cu > uhi) uhi = cu;
			if (cv < wlo) wlo = cv;
			if (cv > whi) whi = cv;
		}
		em.du = grid;
		em.dv = grid;
		em.nu = static_cast<int>((uhi - ulo) / grid) + 1;
		em.nv = static_cast<int>((whi - wlo) / grid) + 1;
		if (em.nu < 1) em.nu = 1;
		if (em.nv < 1) em.nv = 1;
		em.origin = face.centroid + Scale(em.ud, ulo + 0.5f * grid)
			+ Scale(em.vd, wlo + 0.5f * grid);
		em.ndirs = ndirs;
		em.face = face.brush;
		const float kPi2 = 6.2831853f;
		for (int iv = 0; iv < em.nv; ++iv)
		for (int iu = 0; iu < em.nu; ++iu) {
			const Vec3 pt = em.origin
				+ Scale(em.ud, static_cast<float>(iu) * grid)
				+ Scale(em.vd, static_cast<float>(iv) * grid);
			if (Board::EdgeDistOut(face, pt) > 0.f)
				continue;
			for (int d = 0; d < ndirs; ++d) {
				const float beta = kPi2 * static_cast<float>(d)
					/ static_cast<float>(ndirs);
				const Vec3 dir = Scale(em.ud, cosf(beta))
					+ Scale(em.vd, sinf(beta));
				Vec3 dp = pt;
				bool edge = false;
				if (dir.Z <= 0.02f) {
					// Descending/flat stays pressed to the plane:
					// only the boundary cell along this direction
					// owns the edge departure.
					const float run = RunwayRay(face, pt, dir);
					if (run > grid * 1.5f)
						continue;
					dp = pt + Scale(dir, run);
					edge = true;
				}
				ExitSample es;
				if (!EvalExitSample(bq, bvel, face, nt, p, ducked,
					dp, dir, igrid, max_ticks, &es))
					continue;
				es.edge = edge;
				es.iu = iu;
				es.iv = iv;
				em.samples.push_back(es);
			}
		}
		// Per-functional argmax, turn_ok preferred then any.
		for (int pass = 0; pass < 2; ++pass) {
			if (em.best_max >= 0)
				break;
			float bm = -1e30f, ba = -1e30f, bs = -1e30f;
			float bp = -1e30f;
			float bzn = 1e30f;
			for (size_t i = 0; i < em.samples.size(); ++i) {
				const ExitSample& es = em.samples[i];
				if (!es.q.any)
					continue;
				if (pass == 0 && !es.turn_ok)
					continue;
				if (nt.is_zone) {
					// The ending's functional is TIME: total ticks
					// to the platform (ride + flight).
					const float tot = es.ride_ticks + es.zn;
					if (tot < bzn) {
						bzn = tot;
						em.best_max = static_cast<int>(i);
						em.best_area = em.best_max;
						em.best_mass = em.best_max;
						em.best_pot = em.best_max;
					}
					continue;
				}
				if (es.q.q_max > bm) {
					bm = es.q.q_max;
					em.best_max = static_cast<int>(i);
				}
				if (es.q.area > ba) {
					ba = es.q.area;
					em.best_area = static_cast<int>(i);
				}
				if (es.q.mass > bs) {
					bs = es.q.mass;
					em.best_mass = static_cast<int>(i);
				}
				if (es.fpot > bp) {
					bp = es.fpot;
					em.best_pot = static_cast<int>(i);
				}
			}
		}
		return em;
	}

} // namespace Field
} // namespace Solver
