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

	struct Sample {
		Vec3  q;                 // landing point (on the face plane)
		bool  in_face = false;   // inside the polygon (grid cells
		                         // outside are kept for the frame)
		bool  reachable = false;
		bool  free_turn = false; // tangent arrival within the free-
		                         // turn budget (no braking needed)
		bool  ascending = false; // arrives before apex
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
			total += law.TurnRad(0.f, 1.f) * (n
				/ static_cast<float>(buckets));
		}
		return total;
	}

	// The braking-turn per-tick ceiling at speed s (peak of the
	// cosa < 0 branch of the certified turn curve).
	inline float BrakeTurnPeak(float s, const MoveParams& p,
	                           bool ducked) {
		Strafe::TickLaw law = Strafe::Law(p, s, 1.f, ducked);
		float best = law.TurnRad(0.f, 1.f);
		for (int i = 1; i <= 16; ++i) {
			const float c = -static_cast<float>(i) / 16.f;
			const float s2 = 1.f - c * c;
			const float tr = law.TurnRad(c,
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
	                       float gravity_scale = 1.f) {
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
				const float e = s_arr * s_arr + vz_arr * vz_arr
					- res * res
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
			if (sm.reachable) {
				if (sm.e_eff < m.e_lo) m.e_lo = sm.e_eff;
				if (sm.e_eff > m.e_hi) m.e_hi = sm.e_eff;
				const int idx = iv * m.nu + iu;
				if (sm.free_turn && sm.e_eff > best_e) {
					best_e = sm.e_eff;
					best_i = idx;
				}
				if (sm.e_eff > best_e_any) {
					best_e_any = sm.e_eff;
					best_i_any = idx;
				}
			}
		}
		m.best = best_i >= 0 ? best_i : best_i_any;
		m.face = face.brush;   // caller knows the graph index
		return m;
	}

} // namespace Field
} // namespace Solver
