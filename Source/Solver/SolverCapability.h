#pragma once

// THE CAPABILITY LIBRARY (program opened 2026-08-20f, user + advisor
// ruling; design of record Docs/CapabilityLibrary.md).
//
// GOVERNING PRINCIPLE: for every movement capability, answer "given an
// exact initial state and a small number of constraints, what are the
// MINIMUM and MAXIMUM physically achievable outcomes?" - as a unit
// function with an explicit domain, an exhaustive engine-exact
// surface, constructive witnesses, falsification, and a stable
// map-independent API. Composition comes later; the functions outlive
// any particular full-map architecture.
//
// Solved primitives already in the codebase (registry rows; see
// Docs/CapabilityLibrary.md):
//   - vertical state:   z(N), vz(N) closed-form (deterministic per
//                       tick; no duck/jump input) - used throughout.
//   - one-tick air law: Strafe::TickLaw (turn/gain response; the
//                       wishparity gate is its standing check).
//   - board clip:       v' = v - (v.n)n at overbounce 1; loss law
//                       analytic (the exitenv premise).
//   - face slice:       a tilted face at fixed z is a 1-D lambda
//                       interval (session 21's (T, lambda) cells).
//
// THIS FILE: the first UNSOLVED primitive, being solved -
//
//   CapAir::VStar : V*(v0, N, dpsi, d0, a0) = max terminal speed
//     over legal dwell-6 control schedules with net velocity-heading
//     change dpsi after exactly N ticks of clean-air flight,
//   and its dual PsiStar(N, Vmin) = max |dpsi| with v_N >= Vmin,
//   both extracted from ONE forward dynamic program whose every
//   transition is an authoritative MoveTick in a clean-air world.
//
// EXACTNESS DISCIPLINE: the simulator is never approximated. The DP's
// only approximations are REPRESENTATIONAL (heading bins; one max-v
// representative per (heading bin, side, dwell) cell) and both are
// stated conjectures under adversarial falsification:
//   CONJECTURE M (dominance): at equal (heading bin, side, dwell) and
//     equal tick, a higher-speed exact state can always match or beat
//     a lower-speed one's reachable future in (v, psi).
//   The falsifier throws random legal schedules at the surface; any
//   schedule beating V* in its bin REFUTES M (or the bin pitch) and
//   prints RED. Witnesses replay through the engine bitwise.
//
// Position is carried in the representatives but is provably inert
// here: no term of the clean-air acceleration law reads position, and
// the world used for the build has no geometry within reach. (The
// falsifier's random schedules exercise the same fact.)

#include <algorithm>
#include <cstring>
#include <map>
#include <unordered_map>
#include <vector>

#include "SolverAir.h"
#include "SolverMove.h"
#include "SolverSteer.h"
#include "SolverWorld.h"

namespace Solver {
namespace CapAir {

	// ---- the build domain (explicit, per the deliverable contract)
	struct VStarParams {
		float v0 = 600.f;        // initial horizontal speed (u/s)
		int   n_max = 90;        // tick horizon of the surface
		int   psi_bins = 1441;   // heading bins over [-pi, +pi]
		signed char d0 = 0;      // carried strafe side (0 = fresh)
		int   a0 = 1000;         // dwell age (>= 6 = unconstrained)
		float z0 = 8000.f;       // build altitude (clean air; inert)
	};

	// One DP cell: the max-speed exact representative reaching this
	// (heading bin, side, dwell age) at this tick layer.
	struct VCell {
		float vx = 0.f, vy = 0.f;   // exact velocity (the payload)
		float px = 0.f, py = 0.f;   // carried position (inert)
		float pz = 0.f;
		int   parent = -1;          // cell index in the previous layer
		short action = -1;          // action id that produced it
		unsigned char occ = 0;
	};

	// The surface: layers[t][cell]; cell index =
	// ((psi_bin * 3) + side_idx) * 7 + (age_capped - 1).
	struct VStarSurface {
		VStarParams p;
		int cells_per_layer = 0;
		std::vector<std::vector<VCell> > layers;
		long long moveticks = 0;
		double build_ms = 0.0;
		// action table (id -> side, cosa; id 0 = coast)
		std::vector<signed char> act_side;
		std::vector<float> act_cosa;
	};

	inline int CellIndex(const VStarSurface& S, int psi_bin,
	                     int side_idx, int age) {
		const int a = age < 1 ? 1 : (age > 7 ? 7 : age);
		return (psi_bin * 3 + side_idx) * 7 + (a - 1);
	}
	inline int PsiBinOf(const VStarSurface& S, float psi) {
		const float u = (psi + 3.14159265f) / 6.2831853f;
		int b = static_cast<int>(u * static_cast<float>(
			S.p.psi_bins));
		if (b < 0) b = 0;
		if (b >= S.p.psi_bins) b = S.p.psi_bins - 1;
		return b;
	}
	inline float BinPsi(const VStarSurface& S, int bin) {
		return (static_cast<float>(bin) + 0.5f)
			/ static_cast<float>(S.p.psi_bins) * 6.2831853f
			- 3.14159265f;
	}

	// The clean-air build world: one tiny brush far below the flight
	// band so World structures are non-degenerate; nothing reachable.
	inline bool MakeCleanAirWorld(World* w, const Hulls& hulls) {
		std::vector<Vec3> n;
		std::vector<float> d;
		n.push_back(Vec3(1.f, 0.f, 0.f));  d.push_back(100.f);
		n.push_back(Vec3(-1.f, 0.f, 0.f)); d.push_back(100.f);
		n.push_back(Vec3(0.f, 1.f, 0.f));  d.push_back(100.f);
		n.push_back(Vec3(0.f, -1.f, 0.f)); d.push_back(100.f);
		n.push_back(Vec3(0.f, 0.f, 1.f));  d.push_back(-15800.f);
		n.push_back(Vec3(0.f, 0.f, -1.f)); d.push_back(16000.f);
		if (!w->AddTestBrush(n, d, hulls))
			return false;
		w->FinalizeTestWorld();
		return true;
	}

	// One engine-exact air tick under action (side, cosa) from the
	// current velocity heading. Returns the advanced state.
	inline void AirTick(PlayerState* s, const World& w,
	                    const MoveParams& p, signed char side,
	                    float cosa) {
		const float s2d = Len2D(s->vel);
		const float h = s2d > 1.f ? atan2f(s->vel.Y, s->vel.X)
			: 0.f;
		float yaw = h * 57.2957795f;
		float fm = 0.f, sm = 0.f;
		if (side != 0 && s2d > 1.f)
			Air::WishInputs(h, static_cast<int>(side), cosa, &yaw,
				&fm, &sm);
		TickEvents ev;
		MoveTick(*s, w, p, 0.f, yaw, fm, sm, 0.f, 0, &ev);
	}

	// Build the whole surface: one forward DP, every transition an
	// authoritative MoveTick. All (N <= n_max, dpsi) answers come from
	// the one build.
	inline void BuildVStar(const VStarParams& pp, const World& w,
	                       const MoveParams& p, VStarSurface* S) {
		S->p = pp;
		S->cells_per_layer = pp.psi_bins * 3 * 7;
		S->layers.assign(static_cast<size_t>(pp.n_max) + 1,
			std::vector<VCell>());
		// action table: coast + {L, R} x cosa samples over the full
		// legal wish circle (gain, neutral, braking-turn regions).
		static const float kCosa[21] = {
			1.f, 0.9995f, 0.998f, 0.995f, 0.99f, 0.98f, 0.96f,
			0.93f, 0.9f, 0.85f, 0.8f, 0.7f, 0.6f, 0.45f, 0.3f,
			0.15f, 0.f, -0.2f, -0.5f, -0.8f, -1.f };
		S->act_side.clear();
		S->act_cosa.clear();
		S->act_side.push_back(0);
		S->act_cosa.push_back(1.f);
		for (int sd = 0; sd < 2; ++sd)
			for (int ci = 0; ci < 21; ++ci) {
				S->act_side.push_back(sd == 0
					? static_cast<signed char>(1)
					: static_cast<signed char>(-1));
				S->act_cosa.push_back(kCosa[ci]);
			}
		const int n_act = static_cast<int>(S->act_side.size());
		const int min_gap = static_cast<int>(
			ceilf((1.f / p.dt) / p.strafe_rate_max));
		const long long mt0 = g_movetick_count;
		const auto t0 = std::chrono::steady_clock::now();
		// layer 0: the single start state at psi = 0.
		S->layers[0].assign(static_cast<size_t>(
			S->cells_per_layer), VCell());
		{
			VCell c;
			c.vx = pp.v0;
			c.vy = 0.f;
			c.px = 0.f;
			c.py = 0.f;
			c.pz = pp.z0;
			c.parent = -1;
			c.action = -1;
			c.occ = 1;
			const int side_idx = pp.d0 < 0 ? 0
				: (pp.d0 == 0 ? 1 : 2);
			const int idx = CellIndex(*S, PsiBinOf(*S, 0.f),
				side_idx, pp.a0 > 7 ? 7 : pp.a0);
			S->layers[0][static_cast<size_t>(idx)] = c;
		}
		for (int t = 0; t < pp.n_max; ++t) {
			std::vector<VCell>& curL = S->layers[
				static_cast<size_t>(t)];
			S->layers[static_cast<size_t>(t) + 1].assign(
				static_cast<size_t>(S->cells_per_layer),
				VCell());
			std::vector<VCell>& nxtL = S->layers[
				static_cast<size_t>(t) + 1];
			for (int ci = 0; ci < S->cells_per_layer; ++ci) {
				const VCell cell = curL[static_cast<size_t>(ci)];
				if (!cell.occ)
					continue;
				const int side_idx = (ci / 7) % 3;
				const signed char cside = side_idx == 0
					? static_cast<signed char>(-1)
					: (side_idx == 1 ? static_cast<signed char>(0)
						: static_cast<signed char>(1));
				const int cage = (ci % 7) + 1;
				for (int ai = 0; ai < n_act; ++ai) {
					const signed char ds = S->act_side[
						static_cast<size_t>(ai)];
					const float ca = S->act_cosa[
						static_cast<size_t>(ai)];
					if (ds != 0 && cside != 0 && ds != cside
						&& cage < min_gap)
						continue;   // illegal reversal
					PlayerState s;
					s.pos = Vec3(cell.px, cell.py, cell.pz);
					s.vel = Vec3(cell.vx, cell.vy, 0.f);
					// vertical is inert for this capability;
					// rebuild vz each layer so z stays in band
					s.vel.Z = 0.f;
					AirTick(&s, w, p, ds, ca);
					const float nvx = s.vel.X, nvy = s.vel.Y;
					const float nsp = sqrtf(nvx * nvx
						+ nvy * nvy);
					const float npsi = nsp > 1.f
						? atan2f(nvy, nvx) : 0.f;
					const signed char nside = ds != 0 ? ds
						: cside;
					const int nage = (ds != 0 && cside != 0
						&& ds != cside) ? 1
						: (cage < 7 ? cage + 1 : 7);
					const int nsidx = nside < 0 ? 0
						: (nside == 0 ? 1 : 2);
					const int ni = CellIndex(*S,
						PsiBinOf(*S, npsi), nsidx, nage);
					VCell& dst = nxtL[static_cast<size_t>(ni)];
					const float dsp = sqrtf(dst.vx * dst.vx
						+ dst.vy * dst.vy);
					if (!dst.occ || nsp > dsp) {
						dst.vx = nvx;
						dst.vy = nvy;
						dst.px = s.pos.X;
						dst.py = s.pos.Y;
						dst.pz = s.pos.Z;
						dst.parent = ci;
						dst.action = static_cast<short>(ai);
						dst.occ = 1;
					}
				}
			}
		}
		S->moveticks = g_movetick_count - mt0;
		S->build_ms = std::chrono::duration<double, std::milli>(
			std::chrono::steady_clock::now() - t0).count();
	}

	// V*(N, dpsi): max terminal speed over every (side, dwell) at the
	// heading bin containing dpsi. -1 if unreached.
	inline float QueryVStar(const VStarSurface& S, int N, float dpsi,
	                        int* cell_out = nullptr) {
		if (N < 0 || N > S.p.n_max)
			return -1.f;
		const int pb = PsiBinOf(S, dpsi);
		float best = -1.f;
		for (int si = 0; si < 3; ++si)
			for (int ag = 1; ag <= 7; ++ag) {
				const int ci = CellIndex(S, pb, si, ag);
				const VCell& c = S.layers[static_cast<size_t>(
					N)][static_cast<size_t>(ci)];
				if (!c.occ)
					continue;
				const float sp = sqrtf(c.vx * c.vx
					+ c.vy * c.vy);
				if (sp > best) {
					best = sp;
					if (cell_out)
						*cell_out = ci;
				}
			}
		return best;
	}

	// The dual: Psi*(N, Vmin) = max |dpsi| with v_N >= Vmin.
	inline float QueryPsiStar(const VStarSurface& S, int N,
	                          float vmin) {
		if (N < 0 || N > S.p.n_max)
			return -1.f;
		float best = -1.f;
		for (int pb = 0; pb < S.p.psi_bins; ++pb)
			for (int si = 0; si < 3; ++si)
				for (int ag = 1; ag <= 7; ++ag) {
					const int ci = CellIndex(S, pb, si, ag);
					const VCell& c = S.layers[
						static_cast<size_t>(N)][
						static_cast<size_t>(ci)];
					if (!c.occ)
						continue;
					const float sp = sqrtf(c.vx * c.vx
						+ c.vy * c.vy);
					if (sp < vmin)
						continue;
					const float ap = fabsf(BinPsi(S, pb));
					if (ap > best)
						best = ap;
				}
		return best;
	}

	// ---- THE v-STRATIFIED BUILD (session 22, after the falsifier
	// REFUTED Conjecture M on the scalar build: random schedules beat
	// max-speed-per-heading-cell by up to +220 u/s, because turn rate
	// scales as 1/v - at equal heading a slower state can out-turn-
	// then-accelerate a faster one when the remaining task is
	// turn-heavy). Speed joins the lattice: cells =
	// (psi bin, v stratum, side, age<=6), max-v representative per
	// cell. The stratum width is the stated residual conjecture: the
	// falsifier's beat margin must shrink to O(stratum width) or the
	// representation is still wrong.
	struct VCellS {
		float vx = 0.f, vy = 0.f;
		float px = 0.f, py = 0.f, pz = 0.f;
		long long parent = -1;      // previous-layer cell key
		short action = -1;
	};
	struct VStarSurfaceS {
		VStarParams p;
		float v_bin = 10.f;
		int   psi_bins = 721;       // 0.5 deg with v strata absorbing
		std::vector<std::map<long long, VCellS> > layers;
		long long moveticks = 0;
		int  peak_cells = 0;
		bool aborted = false;
		double build_ms = 0.0;
		std::vector<signed char> act_side;
		std::vector<float> act_cosa;
	};
	inline int PsiBinS(const VStarSurfaceS& S, float psi) {
		const float u = (psi + 3.14159265f) / 6.2831853f;
		int b = static_cast<int>(u * static_cast<float>(
			S.psi_bins));
		if (b < 0) b = 0;
		if (b >= S.psi_bins) b = S.psi_bins - 1;
		return b;
	}
	inline float BinPsiS(const VStarSurfaceS& S, int bin) {
		return (static_cast<float>(bin) + 0.5f)
			/ static_cast<float>(S.psi_bins) * 6.2831853f
			- 3.14159265f;
	}
	inline long long KeyS(const VStarSurfaceS& S, int psi_bin,
	                      int vb, int side_idx, int age) {
		const int a = age < 1 ? 1 : (age > 6 ? 6 : age);
		return (((static_cast<long long>(psi_bin) * 512 + vb) * 3
			+ side_idx) * 6) + (a - 1);
	}
	inline void BuildVStarStrat(const VStarParams& pp, const World& w,
	                            const MoveParams& p,
	                            VStarSurfaceS* S) {
		S->p = pp;
		S->layers.assign(static_cast<size_t>(pp.n_max) + 1,
			std::map<long long, VCellS>());
		static const float kCosa[21] = {
			1.f, 0.9995f, 0.998f, 0.995f, 0.99f, 0.98f, 0.96f,
			0.93f, 0.9f, 0.85f, 0.8f, 0.7f, 0.6f, 0.45f, 0.3f,
			0.15f, 0.f, -0.2f, -0.5f, -0.8f, -1.f };
		S->act_side.clear();
		S->act_cosa.clear();
		S->act_side.push_back(0);
		S->act_cosa.push_back(1.f);
		for (int sd = 0; sd < 2; ++sd)
			for (int ci = 0; ci < 21; ++ci) {
				S->act_side.push_back(sd == 0
					? static_cast<signed char>(1)
					: static_cast<signed char>(-1));
				S->act_cosa.push_back(kCosa[ci]);
			}
		const int n_act = static_cast<int>(S->act_side.size());
		const int min_gap = static_cast<int>(
			ceilf((1.f / p.dt) / p.strafe_rate_max));
		const int kAbortCells = 400000;
		const long long mt0 = g_movetick_count;
		const auto t0 = std::chrono::steady_clock::now();
		{
			VCellS c;
			c.vx = pp.v0;
			c.vy = 0.f;
			c.pz = pp.z0;
			const int side_idx = pp.d0 < 0 ? 0
				: (pp.d0 == 0 ? 1 : 2);
			const int vb = static_cast<int>(pp.v0 / S->v_bin);
			S->layers[0][KeyS(*S, PsiBinS(*S, 0.f),
				vb < 511 ? vb : 511, side_idx,
				pp.a0 > 6 ? 6 : pp.a0)] = c;
		}
		for (int t = 0; t < pp.n_max; ++t) {
			const std::map<long long, VCellS>& curL = S->layers[
				static_cast<size_t>(t)];
			std::map<long long, VCellS>& nxtL = S->layers[
				static_cast<size_t>(t) + 1];
			for (std::map<long long, VCellS>::const_iterator it =
				curL.begin(); it != curL.end(); ++it) {
				const long long ckey = it->first;
				const VCellS cell = it->second;
				const int side_code = static_cast<int>(
					(ckey / 6) % 3);
				const signed char cside = side_code == 0
					? static_cast<signed char>(-1)
					: (side_code == 1
						? static_cast<signed char>(0)
						: static_cast<signed char>(1));
				const int cage = static_cast<int>(ckey % 6) + 1;
				for (int ai = 0; ai < n_act; ++ai) {
					const signed char ds = S->act_side[
						static_cast<size_t>(ai)];
					const float ca = S->act_cosa[
						static_cast<size_t>(ai)];
					if (ds != 0 && cside != 0 && ds != cside
						&& cage < min_gap)
						continue;
					PlayerState s;
					s.pos = Vec3(cell.px, cell.py, cell.pz);
					s.vel = Vec3(cell.vx, cell.vy, 0.f);
					AirTick(&s, w, p, ds, ca);
					const float nvx = s.vel.X, nvy = s.vel.Y;
					const float nsp = sqrtf(nvx * nvx
						+ nvy * nvy);
					const float npsi = nsp > 1.f
						? atan2f(nvy, nvx) : 0.f;
					const signed char nside = ds != 0 ? ds
						: cside;
					const int nage = (ds != 0 && cside != 0
						&& ds != cside) ? 1
						: (cage < 6 ? cage + 1 : 6);
					const int nsidx = nside < 0 ? 0
						: (nside == 0 ? 1 : 2);
					int vb = static_cast<int>(nsp / S->v_bin);
					if (vb > 511) vb = 511;
					const long long nk = KeyS(*S,
						PsiBinS(*S, npsi), vb, nsidx, nage);
					std::map<long long, VCellS>::iterator jt =
						nxtL.find(nk);
					if (jt == nxtL.end()
						|| nsp > sqrtf(jt->second.vx
							* jt->second.vx + jt->second.vy
							* jt->second.vy)) {
						VCellS nc;
						nc.vx = nvx;
						nc.vy = nvy;
						nc.px = s.pos.X;
						nc.py = s.pos.Y;
						nc.pz = s.pos.Z;
						nc.parent = ckey;
						nc.action = static_cast<short>(ai);
						nxtL[nk] = nc;
					}
				}
			}
			if (static_cast<int>(nxtL.size()) > S->peak_cells)
				S->peak_cells = static_cast<int>(nxtL.size());
			if (static_cast<int>(nxtL.size()) > kAbortCells) {
				S->aborted = true;
				break;
			}
		}
		S->moveticks = g_movetick_count - mt0;
		S->build_ms = std::chrono::duration<double, std::milli>(
			std::chrono::steady_clock::now() - t0).count();
	}
	inline float QueryVStarS(const VStarSurfaceS& S, int N,
	                         float dpsi, long long* key_out = nullptr) {
		if (N < 0 || N > S.p.n_max)
			return -1.f;
		const int pb = PsiBinS(S, dpsi);
		float best = -1.f;
		const std::map<long long, VCellS>& L = S.layers[
			static_cast<size_t>(N)];
		// cells of this psi bin span keys [pb*512*18, (pb+1)*512*18)
		std::map<long long, VCellS>::const_iterator it =
			L.lower_bound(static_cast<long long>(pb) * 512 * 18);
		const long long hi = (static_cast<long long>(pb) + 1) * 512
			* 18;
		for (; it != L.end() && it->first < hi; ++it) {
			const float sp = sqrtf(it->second.vx * it->second.vx
				+ it->second.vy * it->second.vy);
			if (sp > best) {
				best = sp;
				if (key_out)
					*key_out = it->first;
			}
		}
		return best;
	}
	inline void WitnessS(const VStarSurfaceS& S, int N, long long key,
	                     std::vector<signed char>* side,
	                     std::vector<float>* cosa) {
		side->assign(static_cast<size_t>(N), 0);
		cosa->assign(static_cast<size_t>(N), 1.f);
		long long k = key;
		for (int t = N; t >= 1; --t) {
			std::map<long long, VCellS>::const_iterator it =
				S.layers[static_cast<size_t>(t)].find(k);
			if (it == S.layers[static_cast<size_t>(t)].end())
				break;
			(*side)[static_cast<size_t>(t) - 1] =
				it->second.action >= 0
				? S.act_side[static_cast<size_t>(
					it->second.action)] : 0;
			(*cosa)[static_cast<size_t>(t) - 1] =
				it->second.action >= 0
				? S.act_cosa[static_cast<size_t>(
					it->second.action)] : 1.f;
			k = it->second.parent;
			if (k < 0)
				break;
		}
	}

	// ==================================================================
	// A4-PROD: THE EXACT CLEAN-AIR KERNEL (session 24).
	// Partial evaluation of the verified engine under the proven
	// clean-air domain: no world, no trace, no hull - the SAME float
	// operations in the SAME order as AirTick's WishInputs+MoveTick
	// chain, gated bitwise by the airkparity suite. NOT approximation:
	// a kernel that fails parity by one ulp does not ship.
	// The kernel body is defined in KernelTick below (filled from the
	// engine-recon op-order extraction; see Docs/CapabilityLibrary.md
	// Phase 0).
	// ==================================================================
	// DOMAIN (stamped): the exact domain CapAir::AirTick executes - a
	// fresh PlayerState (surface_friction 1, gravity_scale 1, basevel 0,
	// no duck, airborne, buttons 0, no water) in clean air (no contact
	// within the swept tick - a CALLER GUARANTEE: the shipped builds
	// fly at z0 = 8000 with the only brush at z <= -15800, and the
	// suite's engine witness replay enforces the elision downstream).
	// Each elision below cites the engine line it partially evaluates:
	//   - timers/Duck (SolverMove.cpp 887-929): state-only at buttons 0.
	//   - jump gate (936-939): airborne CheckJumpButton is state-only.
	//   - friction + grounded vz zeroes (940-943, 952-953): on_ground
	//     false in this domain.
	//   - TryPlayerMove (55-173): no contact => one full-fraction bump;
	//     velocity untouched EXCEPT the exact-zero tail (64, 171-172);
	//     position advances by the engine's re-derived delta (96/106).
	//   - CategorizePosition (949): no ground in clean air; the
	//     surface_friction it leaves is next-tick state, which this
	//     domain resets (fresh states) - carried as a ctx input.
	//   - triggers (964-988): none in the build worlds.
	// The basevel +/- ride and the *1.f surface_friction multiply are
	// kept as RUNTIME values so signed-zero and rounding semantics match
	// the engine bit for bit.
	struct AirKernelCtx {
		MoveParams p;
		Vec3 basevel;                   // runtime zeros (the +/- ride)
		float surface_friction = 1.f;   // fresh-state value
		float gravity_scale = 1.f;
	};
	inline AirKernelCtx MakeAirKernel(const MoveParams& p) {
		AirKernelCtx k;
		k.p = p;
		return k;
	}
	// CheckVelocity, velocity channel, re-coded 1-to-1 from
	// SolverMove.cpp:22-36 (the origin check there is unreachable in
	// this model's arithmetic; component order X, Y, Z).
	inline void KernelCheckVelocity(const MoveParams& p, float* vx,
	                                float* vy, float* vz) {
		float* v[3] = { vx, vy, vz };
		for (int i = 0; i < 3; ++i) {
			unsigned bits;
			memcpy(&bits, v[i], 4);
			if ((bits & 0x7f800000u) == 0x7f800000u)
				*v[i] = 0.f;
			if (*v[i] > p.maxvelocity)
				*v[i] = p.maxvelocity;
			else if (*v[i] < -p.maxvelocity)
				*v[i] = -p.maxvelocity;
		}
	}
	inline void KernelTick(const AirKernelCtx& k,
	                       float px, float py, float pz,
	                       float vx, float vy, float vz,
	                       signed char side, float cosa,
	                       float* npx, float* npy, float* npz,
	                       float* nvx, float* nvy, float* nvz) {
		const MoveParams& p = k.p;
		// ---- the AirTick preamble (this file, AirTick - same calls)
		Vec3 vel(vx, vy, vz);
		Vec3 pos(px, py, pz);
		const float s2d = Len2D(vel);
		const float h = s2d > 1.f ? atan2f(vel.Y, vel.X) : 0.f;
		float yaw = h * 57.2957795f;
		float fm = 0.f, sm = 0.f;
		if (side != 0 && s2d > 1.f)
			Air::WishInputs(h, static_cast<int>(side), cosa, &yaw,
				&fm, &sm);
		// ---- MoveTick airborne chain (SolverMove.cpp:931-957)
		// StartGravity half + basevel.Z integrate-and-CLEAR (933-935):
		// the ride below therefore carries Z = 0, exactly as the
		// engine's cleared s.basevel does (review finding, session 24)
		vel.Z -= k.gravity_scale * p.gravity * 0.5f * p.dt;
		vel.Z += k.basevel.Z * p.dt;
		const Vec3 bv(k.basevel.X, k.basevel.Y, 0.f);
		KernelCheckVelocity(p, &vel.X, &vel.Y, &vel.Z);      // 944
		// AirMove (533-569); cap_ducked false in this domain
		float wx, wy;
		Fn::WishFromInput(yaw, fm, sm, &wx, &wy);
		float wishspeed = sqrtf(wx * wx + wy * wy);
		Vec3 wishdir(0.f, 0.f, 0.f);
		if (wishspeed > 1e-6f)
			wishdir = Vec3(wx / wishspeed, wy / wishspeed, 0.f);
		const float effmax = p.maxspeed;
		if (wishspeed > effmax)
			wishspeed = effmax;
		const float wishspd = (wishspeed > p.air_speed_cap)
			? p.air_speed_cap : wishspeed;
		const float cur = Dot(vel, wishdir);
		const float add = wishspd - cur;
		if (add > 0.f) {
			float accelspeed = p.airaccelerate * wishspeed * p.dt
				* k.surface_friction;
			if (accelspeed > add)
				accelspeed = add;
			vel = vel + Scale(wishdir, accelspeed);
		}
		// basevel rides TryPlayerMove (566-568; Z already cleared)
		vel = vel + bv;
		// TryPlayerMove under no contact (55-173)
		if (Len2(vel) == 0.f) {
			vel = Vec3();          // the all_fraction == 0 tail (171)
		} else {
			const float time_left = p.dt;
			const Vec3 end = pos + Scale(vel, time_left);
			const float frac = 1.f;   // the trace misses everything
			pos = pos + Scale(end - pos, frac);   // 106
		}
		vel = vel - bv;
		// CategorizePosition: state-only in clean air (949)
		KernelCheckVelocity(p, &vel.X, &vel.Y, &vel.Z);      // 950
		// FinishGravity half, MoveTick's inline pairing (951)
		vel.Z -= k.gravity_scale * p.gravity * 0.5f * p.dt;
		KernelCheckVelocity(p, &vel.X, &vel.Y, &vel.Z);      // 957
		*npx = pos.X;
		*npy = pos.Y;
		*npz = pos.Z;
		*nvx = vel.X;
		*nvy = vel.Y;
		*nvz = vel.Z;
	}

	// ==================================================================
	// A6/A7 (N-TICK AIR TURN/GAIN V* AND ITS DUAL) - THE EXACT-BAND
	// FLAT-LATTICE BUILD (session 24).
	// The corrected representation after Conjecture M's refutation:
	// cells = (psi bin, v stratum, side, dwell age), on DENSE arrays
	// (the std::map build hit its 400k-cell honest abort). Every
	// transition is one A4-prod KernelTick (bitwise-equal to MoveTick
	// by the airkparity gate), so the surface remains engine-exact.
	// Witness replay of extrema still runs through the REAL MoveTick
	// chain and must reproduce the node's terminal velocity BITWISE.
	// ==================================================================
	struct VNodeX {
		float vx = 0.f, vy = 0.f;   // exact velocity (the payload)
		int   parent = -1;          // node index in the PREVIOUS layer
		int   key = -1;             // slot key (pb, vb, side, age)
		short action = -1;
	};
	struct VStarSurfaceX {
		VStarParams p;
		float v_bin = 10.f;
		int   psi_bins = 721;
		int   vb_max = 512;
		int   slots = 0;            // psi_bins * vb_max * 3 * 6
		std::vector<std::vector<VNodeX> > layers;  // compact per layer
		// per-(layer, psi bin) query accelerators
		std::vector<std::vector<float> > pb_vmax;  // max speed
		std::vector<std::vector<int> >   pb_node;  // best node index
		long long kernel_ticks = 0;
		long long node_budget = 150000000; // honest abort (~3 GB)
		long long total_nodes = 0;
		int   peak_layer_cells = 0;
		bool  aborted = false;
		double build_ms = 0.0;
		std::vector<signed char> act_side;
		std::vector<float> act_cosa;
	};
	inline int KeyX(const VStarSurfaceX& S, int pb, int vb,
	                int side_idx, int age) {
		const int a = age < 1 ? 1 : (age > 6 ? 6 : age);
		return (((pb * S.vb_max) + vb) * 3 + side_idx) * 6 + (a - 1);
	}
	inline int PsiBinX(const VStarSurfaceX& S, float psi) {
		const float u = (psi + 3.14159265f) / 6.2831853f;
		int b = static_cast<int>(u * static_cast<float>(S.psi_bins));
		if (b < 0) b = 0;
		if (b >= S.psi_bins) b = S.psi_bins - 1;
		return b;
	}
	inline float BinPsiX(const VStarSurfaceX& S, int bin) {
		return (static_cast<float>(bin) + 0.5f)
			/ static_cast<float>(S.psi_bins) * 6.2831853f
			- 3.14159265f;
	}
	inline void BuildVStarX(const VStarParams& pp, const MoveParams& p,
	                        const AirKernelCtx& k, VStarSurfaceX* S) {
		S->p = pp;
		S->slots = S->psi_bins * S->vb_max * 3 * 6;
		S->layers.assign(static_cast<size_t>(pp.n_max) + 1,
			std::vector<VNodeX>());
		S->pb_vmax.assign(static_cast<size_t>(pp.n_max) + 1,
			std::vector<float>());
		S->pb_node.assign(static_cast<size_t>(pp.n_max) + 1,
			std::vector<int>());
		static const float kCosa[21] = {
			1.f, 0.9995f, 0.998f, 0.995f, 0.99f, 0.98f, 0.96f,
			0.93f, 0.9f, 0.85f, 0.8f, 0.7f, 0.6f, 0.45f, 0.3f,
			0.15f, 0.f, -0.2f, -0.5f, -0.8f, -1.f };
		S->act_side.clear();
		S->act_cosa.clear();
		S->act_side.push_back(0);
		S->act_cosa.push_back(1.f);
		for (int sd = 0; sd < 2; ++sd)
			for (int ci = 0; ci < 21; ++ci) {
				S->act_side.push_back(sd == 0
					? static_cast<signed char>(1)
					: static_cast<signed char>(-1));
				S->act_cosa.push_back(kCosa[ci]);
			}
		const int n_act = static_cast<int>(S->act_side.size());
		const int min_gap = static_cast<int>(
			ceilf((1.f / p.dt) / p.strafe_rate_max));
		const auto t0 = std::chrono::steady_clock::now();
		// transient slot map, reused per layer
		std::vector<int> slot(static_cast<size_t>(S->slots), -1);
		{
			VNodeX c;
			c.vx = pp.v0;
			c.vy = 0.f;
			const int side_idx = pp.d0 < 0 ? 0
				: (pp.d0 == 0 ? 1 : 2);
			int vb = static_cast<int>(pp.v0 / S->v_bin);
			if (vb >= S->vb_max) vb = S->vb_max - 1;
			c.key = KeyX(*S, PsiBinX(*S, 0.f), vb, side_idx,
				pp.a0 > 6 ? 6 : pp.a0);
			S->layers[0].push_back(c);
			S->total_nodes = 1;
		}
		for (int t = 0; t < pp.n_max; ++t) {
			const std::vector<VNodeX>& curL = S->layers[
				static_cast<size_t>(t)];
			std::vector<VNodeX>& nxtL = S->layers[
				static_cast<size_t>(t) + 1];
			std::fill(slot.begin(), slot.end(), -1);
			for (size_t ni = 0; ni < curL.size(); ++ni) {
				const VNodeX cell = curL[ni];
				const int side_code = (cell.key / 6) % 3;
				const signed char cside = side_code == 0
					? static_cast<signed char>(-1)
					: (side_code == 1
						? static_cast<signed char>(0)
						: static_cast<signed char>(1));
				const int cage = (cell.key % 6) + 1;
				for (int ai = 0; ai < n_act; ++ai) {
					const signed char ds = S->act_side[
						static_cast<size_t>(ai)];
					const float ca = S->act_cosa[
						static_cast<size_t>(ai)];
					if (ds != 0 && cside != 0 && ds != cside
						&& cage < min_gap)
						continue;   // illegal reversal
					float dpx, dpy, dpz, nvz;
					float nvx, nvy;
					KernelTick(k, 0.f, 0.f, pp.z0,
						cell.vx, cell.vy, 0.f, ds, ca,
						&dpx, &dpy, &dpz, &nvx, &nvy, &nvz);
					++S->kernel_ticks;
					const float nsp = sqrtf(nvx * nvx
						+ nvy * nvy);
					const float npsi = nsp > 1.f
						? atan2f(nvy, nvx) : 0.f;
					const signed char nside = ds != 0 ? ds
						: cside;
					const int nage = (ds != 0 && cside != 0
						&& ds != cside) ? 1
						: (cage < 6 ? cage + 1 : 6);
					const int nsidx = nside < 0 ? 0
						: (nside == 0 ? 1 : 2);
					int vb = static_cast<int>(nsp / S->v_bin);
					if (vb >= S->vb_max) vb = S->vb_max - 1;
					const int nk = KeyX(*S, PsiBinX(*S, npsi),
						vb, nsidx, nage);
					const int have = slot[static_cast<size_t>(
						nk)];
					if (have < 0) {
						VNodeX nc;
						nc.vx = nvx;
						nc.vy = nvy;
						nc.parent = static_cast<int>(ni);
						nc.key = nk;
						nc.action = static_cast<short>(ai);
						slot[static_cast<size_t>(nk)] =
							static_cast<int>(nxtL.size());
						nxtL.push_back(nc);
					} else {
						VNodeX& dst = nxtL[
							static_cast<size_t>(have)];
						const float dsp = sqrtf(dst.vx * dst.vx
							+ dst.vy * dst.vy);
						if (nsp > dsp) {
							dst.vx = nvx;
							dst.vy = nvy;
							dst.parent = static_cast<int>(ni);
							dst.action =
								static_cast<short>(ai);
						}
					}
				}
			}
			if (static_cast<int>(nxtL.size())
				> S->peak_layer_cells)
				S->peak_layer_cells = static_cast<int>(
					nxtL.size());
			S->total_nodes += static_cast<long long>(nxtL.size());
			if (S->total_nodes > S->node_budget) {
				S->aborted = true;
				break;
			}
		}
		// query accelerators: per-(layer, psi bin) best node
		for (size_t t = 0; t < S->layers.size(); ++t) {
			S->pb_vmax[t].assign(static_cast<size_t>(S->psi_bins),
				-1.f);
			S->pb_node[t].assign(static_cast<size_t>(S->psi_bins),
				-1);
			const std::vector<VNodeX>& L = S->layers[t];
			for (size_t ni = 0; ni < L.size(); ++ni) {
				const int pb = L[ni].key / (S->vb_max * 18);
				const float sp = sqrtf(L[ni].vx * L[ni].vx
					+ L[ni].vy * L[ni].vy);
				if (sp > S->pb_vmax[t][static_cast<size_t>(pb)]) {
					S->pb_vmax[t][static_cast<size_t>(pb)] = sp;
					S->pb_node[t][static_cast<size_t>(pb)] =
						static_cast<int>(ni);
				}
			}
		}
		S->build_ms = std::chrono::duration<double, std::milli>(
			std::chrono::steady_clock::now() - t0).count();
	}
	// V*(N, dpsi) from the exact-band surface. -1 if unreached.
	inline float QueryVStarX(const VStarSurfaceX& S, int N, float dpsi,
	                         int* node_out = nullptr) {
		if (N < 0 || N > S.p.n_max)
			return -1.f;
		const int pb = PsiBinX(S, dpsi);
		const float v = S.pb_vmax[static_cast<size_t>(N)][
			static_cast<size_t>(pb)];
		if (v >= 0.f && node_out)
			*node_out = S.pb_node[static_cast<size_t>(N)][
				static_cast<size_t>(pb)];
		return v;
	}
	// Psi*(N, Vmin) from the same accelerators.
	inline float QueryPsiStarX(const VStarSurfaceX& S, int N,
	                           float vmin) {
		if (N < 0 || N > S.p.n_max)
			return -1.f;
		float best = -1.f;
		for (int pb = 0; pb < S.psi_bins; ++pb) {
			if (S.pb_vmax[static_cast<size_t>(N)][
				static_cast<size_t>(pb)] < vmin)
				continue;
			const float ap = fabsf(BinPsiX(S, pb));
			if (ap > best)
				best = ap;
		}
		return best;
	}
	// Reconstruct the exact control schedule reaching (N, node).
	inline void WitnessX(const VStarSurfaceX& S, int N, int node,
	                     std::vector<signed char>* side,
	                     std::vector<float>* cosa) {
		side->assign(static_cast<size_t>(N), 0);
		cosa->assign(static_cast<size_t>(N), 1.f);
		int ni = node;
		for (int t = N; t >= 1; --t) {
			const VNodeX& c = S.layers[static_cast<size_t>(t)][
				static_cast<size_t>(ni)];
			(*side)[static_cast<size_t>(t) - 1] = c.action >= 0
				? S.act_side[static_cast<size_t>(c.action)] : 0;
			(*cosa)[static_cast<size_t>(t) - 1] = c.action >= 0
				? S.act_cosa[static_cast<size_t>(c.action)] : 1.f;
			ni = c.parent;
			if (ni < 0)
				break;
		}
	}

	// Reconstruct the exact control schedule reaching (N, cell).
	inline void Witness(const VStarSurface& S, int N, int cell,
	                    std::vector<signed char>* side,
	                    std::vector<float>* cosa) {
		side->assign(static_cast<size_t>(N), 0);
		cosa->assign(static_cast<size_t>(N), 1.f);
		int ci = cell;
		for (int t = N; t >= 1; --t) {
			const VCell& c = S.layers[static_cast<size_t>(t)][
				static_cast<size_t>(ci)];
			(*side)[static_cast<size_t>(t) - 1] = c.action >= 0
				? S.act_side[static_cast<size_t>(c.action)] : 0;
			(*cosa)[static_cast<size_t>(t) - 1] = c.action >= 0
				? S.act_cosa[static_cast<size_t>(c.action)] : 1.f;
			ci = c.parent;
			if (ci < 0)
				break;
		}
	}

} // namespace CapAir

// ======================================================================
// CAPBOARD - registry A15 (board clip, solved), A16/A17 (best/worst
// board + feasibility, the analytic layer), and the A18 one-tick ride
// law harness (session 24). A15's authoritative implementation is the
// engine's own exported Fn::ClipVelocity - never re-derived here. The
// analytic law it realizes (overbounce 1):
//     out = in - n * (in.n)  (+ one adjust pass when out.n < 0)
//     loss = |in.n| ; post_speed^2 = |in|^2 - (in.n)^2   (real math)
// A16/A17 optimize that law over arrival headings in closed form and
// are verified against dense enumeration of the EXACT clip.
// ======================================================================
namespace CapBoard {

	// v.n as a function of arrival heading theta at fixed horizontal
	// speed s_h and vertical speed vz:
	//     v.n = s_h * n_h * cos(theta - phi_n) + vz * n_z
	// with n_h = sqrt(nx^2 + ny^2), phi_n = atan2(ny, nx). Boarding
	// requires v.n < 0 (moving into the face).
	inline float BoardVdotN(float s_h, float theta, float vz,
	                        const Vec3& n) {
		return s_h * (n.X * cosf(theta) + n.Y * sinf(theta))
			+ vz * n.Z;
	}

	struct BoardOpt {
		float theta = 0.f;       // argopt heading
		float vdotn = 0.f;       // v.n there (real math)
		bool  boardable = false; // v.n <= 0 at the optimum
		bool  grazing = false;   // the zero-loss boundary is inside
	};

	// A16: over theta in [t0, t1] (t0 <= t1, radians, may span the
	// circle), the least-loss and most-loss boardable arrivals.
	// Closed-form candidate set: interval endpoints, the cos extrema
	// (phi, phi+pi), and the zero crossings phi +/- acos(-B/A).
	inline void BestWorstBoard(float s_h, float vz, const Vec3& n,
	                           float t0, float t1, BoardOpt* best,
	                           BoardOpt* worst) {
		const float A = s_h * sqrtf(n.X * n.X + n.Y * n.Y);
		const float B = vz * n.Z;
		const float phi = atan2f(n.Y, n.X);
		float cand[10];
		int nc = 0;
		cand[nc++] = t0;
		cand[nc++] = t1;
		const float kTau = 6.2831853f;
		auto addwrap = [&](float th) {
			for (int m = -2; m <= 2; ++m) {
				const float t = th + kTau * static_cast<float>(m);
				if (t >= t0 && t <= t1 && nc < 10)
					cand[nc++] = t;
			}
		};
		addwrap(phi);
		addwrap(phi + 3.14159265f);
		bool zero_inside = false;
		if (A > 0.f && fabsf(B) <= A) {
			const float d = acosf(-B / A);
			const int nc0 = nc;
			addwrap(phi + d);
			addwrap(phi - d);
			zero_inside = nc > nc0;
		}
		float lo = 1e30f, hi = -1e30f;
		float tlo = t0, thi = t0;
		for (int i = 0; i < nc; ++i) {
			const float v = BoardVdotN(s_h, cand[i], vz, n);
			if (v < lo) {
				lo = v;
				tlo = cand[i];
			}
			if (v > hi) {
				hi = v;
				thi = cand[i];
			}
		}
		// worst boardable = most negative v.n (max loss)
		worst->theta = tlo;
		worst->vdotn = lo;
		worst->boardable = lo < 0.f;
		worst->grazing = false;
		// best boardable = v.n closest to 0 from below; if the zero
		// boundary is inside the interval, the best loss is 0 exactly
		// (grazing) at the crossing.
		if (zero_inside && lo < 0.f) {
			// pick the crossing candidate with |v.n| smallest
			float bt = tlo, bv = lo;
			for (int i = 0; i < nc; ++i) {
				const float v = BoardVdotN(s_h, cand[i], vz, n);
				if (v <= 0.f && v > bv) {
					bv = v;
					bt = cand[i];
				}
			}
			best->theta = bt;
			best->vdotn = bv;
			best->boardable = true;
			best->grazing = true;
		} else {
			// no zero inside: v.n keeps one sign on [t0, t1]
			if (hi < 0.f) {
				best->theta = thi;   // all boardable; least loss
				best->vdotn = hi;
				best->boardable = true;
			} else if (lo >= 0.f) {
				best->theta = tlo;   // nothing boards
				best->vdotn = lo;
				best->boardable = false;
			} else {
				best->theta = thi;
				best->vdotn = hi;
				best->boardable = hi <= 0.f;
			}
			best->grazing = false;
		}
	}

	// A17 (inverse): the heading set where the arrival BOARDS with
	// loss <= L:   -L <= v.n < 0. With v.n = A cos(theta-phi) + B the
	// set is { theta : cos(theta-phi) in [(-L-B)/A, -B/A) } - at most
	// two arcs per period, returned as up to 2 [lo, hi] intervals
	// around phi (theta = phi +/- acos(c)). Count returned.
	inline int LossFeasibleArcs(float s_h, float vz, const Vec3& n,
	                            float L, float arcs[2][2]) {
		const float A = s_h * sqrtf(n.X * n.X + n.Y * n.Y);
		const float B = vz * n.Z;
		const float phi = atan2f(n.Y, n.X);
		if (A <= 0.f) {
			// heading-independent: v.n = B everywhere
			if (B < 0.f && -L <= B) {
				arcs[0][0] = -3.14159265f;
				arcs[0][1] = 3.14159265f;
				return 1;
			}
			return 0;
		}
		float clo = (-L - B) / A;   // cos lower bound
		float chi = (-B) / A;       // cos upper bound (exclusive)
		if (clo > 1.f || chi < -1.f || clo >= chi)
			return 0;
		if (clo < -1.f) clo = -1.f;
		if (chi > 1.f) chi = 1.f;
		// cos(u) in [clo, chi] with u = theta - phi in [-pi, pi]:
		// u in [-acos(clo), -acos(chi)] U [acos(chi), acos(clo)]
		const float ulo = acosf(chi);   // smaller angle
		const float uhi = acosf(clo);   // larger angle
		int na = 0;
		arcs[na][0] = phi + ulo;
		arcs[na][1] = phi + uhi;
		na++;
		if (ulo > 1e-7f) {   // the mirrored arc is distinct
			arcs[na][0] = phi - uhi;
			arcs[na][1] = phi - ulo;
			na++;
		}
		return na;
	}

	// The synthetic ramp world for the A18 harness: one big tilted
	// slab whose top face has normal
	//     n = (sin a cos b, sin a sin b, cos a)
	// with a > 45.57 deg so nz < 0.7 (surfable, never walkable).
	inline bool MakeRampWorld(World* w, const Hulls& hulls,
	                          float alpha_deg, float beta_deg,
	                          Vec3* n_out) {
		const float a = alpha_deg * 0.0174533f;
		const float b = beta_deg * 0.0174533f;
		const Vec3 n(sinf(a) * cosf(b), sinf(a) * sinf(b), cosf(a));
		if (n_out)
			*n_out = n;
		std::vector<Vec3> ns;
		std::vector<float> ds;
		ns.push_back(n);
		ds.push_back(0.f);
		ns.push_back(Vec3(-n.X, -n.Y, -n.Z));
		ds.push_back(2000.f);
		ns.push_back(Vec3(1.f, 0.f, 0.f));
		ds.push_back(20000.f);
		ns.push_back(Vec3(-1.f, 0.f, 0.f));
		ds.push_back(20000.f);
		ns.push_back(Vec3(0.f, 1.f, 0.f));
		ds.push_back(20000.f);
		ns.push_back(Vec3(0.f, -1.f, 0.f));
		ds.push_back(20000.f);
		// AddTestBrush requires full axial coverage; these far caps
		// only clip the slab thousands of units from the ride region
		ns.push_back(Vec3(0.f, 0.f, 1.f));
		ds.push_back(20000.f);
		ns.push_back(Vec3(0.f, 0.f, -1.f));
		ds.push_back(20000.f);
		if (!w->AddTestBrush(ns, ds, hulls))
			return false;
		w->FinalizeTestWorld();
		return true;
	}

	// A18 GRAY-BOX one-tick ride prediction (the decomposition
	// HYPOTHESIS under test): for an airborne player in contact with
	// ONE plane, the tick's velocity law is
	//   start-gravity half -> clamp -> air-accelerate (stale
	//   surface_friction) -> single ClipVelocity against the plane ->
	//   clamp -> finish-gravity half -> clamp
	// (velocity is frac-independent on a single-plane tick: a partial
	// move re-bases the clip set but not the velocity). Every
	// arithmetic piece is the engine's own exported function; only the
	// COMPOSITION is hypothesized, and the harness tests it BITWISE.
	inline void RideGrayTick(const MoveParams& p, const Vec3& n,
	                         float sf, const Vec3& basevel, Vec3 vel,
	                         signed char side, float cosa, Vec3* out,
	                         float* pre_clip_vdotn = nullptr,
	                         float* vz_at_categorize = nullptr,
	                         const Vec3* pos_in = nullptr,
	                         Vec3* pos_out = nullptr) {
		const float s2d = Len2D(vel);
		const float h = s2d > 1.f ? atan2f(vel.Y, vel.X) : 0.f;
		float yaw = h * 57.2957795f;
		float fm = 0.f, sm = 0.f;
		if (side != 0 && s2d > 1.f)
			Air::WishInputs(h, static_cast<int>(side), cosa, &yaw,
				&fm, &sm);
		vel.Z -= 1.f * p.gravity * 0.5f * p.dt;
		// basevel.Z integrates once and clears (MoveTick 934-935);
		// the ride sandwich below then carries Z = 0.
		vel.Z += basevel.Z * p.dt;
		const Vec3 bv(basevel.X, basevel.Y, 0.f);
		CapAir::KernelCheckVelocity(p, &vel.X, &vel.Y, &vel.Z);
		float wx, wy;
		Fn::WishFromInput(yaw, fm, sm, &wx, &wy);
		float wishspeed = sqrtf(wx * wx + wy * wy);
		Vec3 wishdir(0.f, 0.f, 0.f);
		if (wishspeed > 1e-6f)
			wishdir = Vec3(wx / wishspeed, wy / wishspeed, 0.f);
		if (wishspeed > p.maxspeed)
			wishspeed = p.maxspeed;
		const float wishspd = (wishspeed > p.air_speed_cap)
			? p.air_speed_cap : wishspeed;
		const float cur = Dot(vel, wishdir);
		const float add = wishspd - cur;
		if (add > 0.f) {
			float accelspeed = p.airaccelerate * wishspeed * p.dt
				* sf;
			if (accelspeed > add)
				accelspeed = add;
			vel = vel + Scale(wishdir, accelspeed);
		}
		// the basevel ride around TryPlayerMove (AirMove 566-568)
		vel = vel + bv;
		if (pre_clip_vdotn)
			*pre_clip_vdotn = Dot(vel, n);   // >= 0: leaves the face
		Vec3 clipped;
		Fn::ClipVelocity(vel, n, &clipped);
		vel = clipped;
		// THE POSITION LAW on a zero-fraction contact tick: the bump
		// at the plane advances nothing (frac 0 keeps time_left), the
		// clipped velocity (still carrying basevel) slides the FULL
		// tick, and the engine re-derives the delta (TryPlayerMove
		// 96/106). Computed HERE, before the basevel subtraction,
		// from the exact slide velocity.
		if (pos_in && pos_out) {
			const Vec3 end = *pos_in + Scale(vel, p.dt);
			*pos_out = *pos_in + Scale(end - *pos_in, 1.f);
		}
		vel = vel - bv;
		// CategorizePosition reads vz HERE (before CheckVelocity 950
		// and the finish-gravity half): rising with no walkable plane
		// sets next tick's surface_friction to 0.25
		if (vz_at_categorize)
			*vz_at_categorize = vel.Z;
		CapAir::KernelCheckVelocity(p, &vel.X, &vel.Y, &vel.Z);
		vel.Z -= 1.f * p.gravity * 0.5f * p.dt;
		CapAir::KernelCheckVelocity(p, &vel.X, &vel.Y, &vel.Z);
		*out = vel;
	}

} // namespace CapBoard

// ======================================================================
// CAPREACH - registry A8, the reach family R_N (session 24): the
// fixed-time reachable set in (x, y, vx, vy, side, age) swept forward
// from a canonical start with the A4-prod kernel, vz = 0 per tick (the
// horizontal channels are vz-invariant - certified by capkern's
// vz-decoupling gate; the vertical channel is the A1 closed form).
// D*(N, phi) is served as an INTERVAL:
//   LB = witnessed (max projection over swept states; every state has
//        a replayable schedule), and
//   UB = certified (the session-20 cone: per-tick |dv| <= air_speed_cap
//        = 30 exactly, so the position deviates from the coast point by
//        at most sum_i 30*dt*i = 0.225*N*(N+1); the coast projection
//        plus that slack bounds every schedule).
// CONJECTURE R1 (stated, under falsification): the max-SPEED
// representative per (pos@hp, vel@hv, side, age) cell loses at most
// O(lattice pitch) of D*. The falsifier measures the actual gap.
// ======================================================================
namespace CapReach {

	struct RParams {
		float v0 = 600.f;
		int   n_max = 90;
		signed char d0 = 0;
		int   a0 = 1000;
		float z0 = 8000.f;
		float hp = 32.f;      // position pitch (session-21 validated)
		float hv = 8.f;       // velocity pitch
	};
	struct RNode {
		float x = 0.f, y = 0.f;
		float vx = 0.f, vy = 0.f;
		int   parent = -1;
		short action = -1;
		unsigned char sa = 0;   // side_idx * 8 + age
	};
	struct RSurface {
		RParams p;
		std::vector<std::vector<RNode> > layers;
		// per-layer witnessed D* LB at 16 sampled directions
		std::vector<std::vector<float> > dslb;   // [layer][16]
		std::vector<std::vector<int> >   dsnode; // [layer][16]
		long long kernel_ticks = 0;
		long long total_nodes = 0;
		long long node_budget = 40000000;  // honest abort ceiling
		int  peak_layer = 0;
		bool aborted = false;
		int  aborted_at = -1;              // last COMPLETE layer
		double build_ms = 0.0;
		std::vector<signed char> act_side;
		std::vector<float> act_cosa;
	};
	inline long long RKey(const RParams& pp, float x, float y,
	                      float vx, float vy, int sidx, int age) {
		int ix = static_cast<int>((x + 4800.f) / pp.hp);
		int iy = static_cast<int>((y + 4800.f) / pp.hp);
		int ivx = static_cast<int>((vx + 3600.f) / pp.hv);
		int ivy = static_cast<int>((vy + 3600.f) / pp.hv);
		if (ix < 0) ix = 0;
		if (ix > 399) ix = 399;
		if (iy < 0) iy = 0;
		if (iy > 399) iy = 399;
		if (ivx < 0) ivx = 0;
		if (ivx > 999) ivx = 999;
		if (ivy < 0) ivy = 0;
		if (ivy > 999) ivy = 999;
		const int a = age < 1 ? 1 : (age > 6 ? 6 : age);
		return ((((static_cast<long long>(ix) * 400 + iy) * 1000
			+ ivx) * 1000 + ivy) * 18) + sidx * 6 + (a - 1);
	}
	// THE CERTIFIED MAX-SPEED-INTEGRAL UB (session 24). This REPLACES
	// the session-20 "reach cone", which capreach's falsifier REFUTED
	// with 61,513 real-schedule violations and the sweep's own
	// witnessed LB exceeded by up to 190 u at backward directions:
	// the cone's premise "per-tick |dv| <= air_speed_cap" is FALSE
	// for braking wishes, where add = cap + |v| and the accel budget
	// (airaccelerate * wishspeed * dt * sf, ~562 u/s at surf params)
	// binds instead. The sound lemma, from the engine's own accel
	// algebra: per tick,
	//     |v'|^2 - |v|^2 = a * (2*cur + a) <= wishspd^2 <= cap^2
	// (since 0 <= a <= add = wishspd - cur gives (a + cur)^2 <=
	// wishspd^2, and a past the sign flip only decreases speed), so
	//     |v_i| <= sqrt(v0^2 + cap^2 * i)
	// (sf-independent, clamp-safe), and any displacement projection
	// is at most dt * sum_i |v_i|. Direction-aware tightening
	// (braking-time analysis) is this capability's open theorem step.
	// NOTE the retroactive consequence recorded in the registry: B2's
	// old cone (FFReachable/FFReachAny) is DEMOTED from certified to
	// refuted-as-stated; nothing may hard-prune with it.
	inline float DStarUB(const RParams& pp, const MoveParams& p,
	                     int N, float phi) {
		(void)phi;   // v1 is direction-independent (loose backward)
		const float cap2 = p.air_speed_cap * p.air_speed_cap;
		float s = 0.f;
		for (int i = 1; i <= N; ++i)
			s += sqrtf(pp.v0 * pp.v0
				+ cap2 * static_cast<float>(i));
		return s * p.dt;
	}
	inline void BuildReach(const RParams& pp, const MoveParams& p,
	                       const CapAir::AirKernelCtx& k,
	                       RSurface* S) {
		S->p = pp;
		S->layers.assign(static_cast<size_t>(pp.n_max) + 1,
			std::vector<RNode>());
		S->dslb.assign(static_cast<size_t>(pp.n_max) + 1,
			std::vector<float>());
		S->dsnode.assign(static_cast<size_t>(pp.n_max) + 1,
			std::vector<int>());
		static const float kCosa[21] = {
			1.f, 0.9995f, 0.998f, 0.995f, 0.99f, 0.98f, 0.96f,
			0.93f, 0.9f, 0.85f, 0.8f, 0.7f, 0.6f, 0.45f, 0.3f,
			0.15f, 0.f, -0.2f, -0.5f, -0.8f, -1.f };
		S->act_side.clear();
		S->act_cosa.clear();
		S->act_side.push_back(0);
		S->act_cosa.push_back(1.f);
		for (int sd = 0; sd < 2; ++sd)
			for (int ci = 0; ci < 21; ++ci) {
				S->act_side.push_back(sd == 0
					? static_cast<signed char>(1)
					: static_cast<signed char>(-1));
				S->act_cosa.push_back(kCosa[ci]);
			}
		const int n_act = static_cast<int>(S->act_side.size());
		const int min_gap = static_cast<int>(
			ceilf((1.f / p.dt) / p.strafe_rate_max));
		const auto t0 = std::chrono::steady_clock::now();
		{
			RNode c;
			c.vx = pp.v0;
			const int sidx = pp.d0 < 0 ? 0 : (pp.d0 == 0 ? 1 : 2);
			const int a = pp.a0 > 6 ? 6 : (pp.a0 < 1 ? 1 : pp.a0);
			c.sa = static_cast<unsigned char>(sidx * 8 + a);
			S->layers[0].push_back(c);
			S->total_nodes = 1;
		}
		std::unordered_map<long long, int> slot;
		for (int t = 0; t < pp.n_max; ++t) {
			const std::vector<RNode>& curL = S->layers[
				static_cast<size_t>(t)];
			std::vector<RNode>& nxtL = S->layers[
				static_cast<size_t>(t) + 1];
			slot.clear();
			slot.reserve(curL.size() * 8 + 64);
			for (size_t ni = 0; ni < curL.size(); ++ni) {
				const RNode cell = curL[ni];
				const int sidx = cell.sa / 8;
				const signed char cside = sidx == 0
					? static_cast<signed char>(-1)
					: (sidx == 1 ? static_cast<signed char>(0)
						: static_cast<signed char>(1));
				const int cage = cell.sa % 8;
				for (int ai = 0; ai < n_act; ++ai) {
					const signed char ds = S->act_side[
						static_cast<size_t>(ai)];
					const float ca = S->act_cosa[
						static_cast<size_t>(ai)];
					if (ds != 0 && cside != 0 && ds != cside
						&& cage < min_gap)
						continue;
					float nx, ny, nz2, nvx, nvy, nvz;
					CapAir::KernelTick(k, cell.x, cell.y, pp.z0,
						cell.vx, cell.vy, 0.f, ds, ca,
						&nx, &ny, &nz2, &nvx, &nvy, &nvz);
					++S->kernel_ticks;
					const signed char nside = ds != 0 ? ds
						: cside;
					const int nage = (ds != 0 && cside != 0
						&& ds != cside) ? 1
						: (cage < 6 ? cage + 1 : 6);
					const int nsidx = nside < 0 ? 0
						: (nside == 0 ? 1 : 2);
					const long long nk = RKey(pp, nx, ny, nvx,
						nvy, nsidx, nage);
					std::unordered_map<long long, int>::iterator
						jt = slot.find(nk);
					if (jt == slot.end()) {
						RNode nc;
						nc.x = nx;
						nc.y = ny;
						nc.vx = nvx;
						nc.vy = nvy;
						nc.parent = static_cast<int>(ni);
						nc.action = static_cast<short>(ai);
						nc.sa = static_cast<unsigned char>(
							nsidx * 8 + nage);
						slot[nk] = static_cast<int>(nxtL.size());
						nxtL.push_back(nc);
					} else {
						RNode& dst = nxtL[static_cast<size_t>(
							jt->second)];
						const float nsp = nvx * nvx + nvy * nvy;
						const float dsp = dst.vx * dst.vx
							+ dst.vy * dst.vy;
						if (nsp > dsp) {
							dst.x = nx;
							dst.y = ny;
							dst.vx = nvx;
							dst.vy = nvy;
							dst.parent = static_cast<int>(ni);
							dst.action = static_cast<short>(ai);
						}
					}
				}
			}
			if (static_cast<int>(nxtL.size()) > S->peak_layer)
				S->peak_layer = static_cast<int>(nxtL.size());
			S->total_nodes += static_cast<long long>(nxtL.size());
			if (S->total_nodes > S->node_budget) {
				S->aborted = true;
				S->aborted_at = t + 1;
				break;
			}
		}
		// the witnessed D* LB tables at 16 directions per layer
		for (size_t t = 0; t < S->layers.size(); ++t) {
			S->dslb[t].assign(16, -1e30f);
			S->dsnode[t].assign(16, -1);
			const std::vector<RNode>& L = S->layers[t];
			for (size_t ni = 0; ni < L.size(); ++ni)
				for (int ph = 0; ph < 16; ++ph) {
					const float phi = static_cast<float>(ph)
						* 0.39269908f;
					const float pr = L[ni].x * cosf(phi)
						+ L[ni].y * sinf(phi);
					if (pr > S->dslb[t][static_cast<size_t>(
						ph)]) {
						S->dslb[t][static_cast<size_t>(ph)] = pr;
						S->dsnode[t][static_cast<size_t>(ph)] =
							static_cast<int>(ni);
					}
				}
		}
		S->build_ms = std::chrono::duration<double, std::milli>(
			std::chrono::steady_clock::now() - t0).count();
	}
	inline void WitnessR(const RSurface& S, int N, int node,
	                     std::vector<signed char>* side,
	                     std::vector<float>* cosa) {
		side->assign(static_cast<size_t>(N), 0);
		cosa->assign(static_cast<size_t>(N), 1.f);
		int ni = node;
		for (int t = N; t >= 1; --t) {
			const RNode& c = S.layers[static_cast<size_t>(t)][
				static_cast<size_t>(ni)];
			(*side)[static_cast<size_t>(t) - 1] = c.action >= 0
				? S.act_side[static_cast<size_t>(c.action)] : 0;
			(*cosa)[static_cast<size_t>(t) - 1] = c.action >= 0
				? S.act_cosa[static_cast<size_t>(c.action)] : 1.f;
			ni = c.parent;
			if (ni < 0)
				break;
		}
	}

} // namespace CapReach

// ======================================================================
// CAPP2P - registry A11 (fixed-tick point-to-point air solve) and the
// A12 iteration over candidate tick counts (session 26; design:
// Docs/CapabilityMethods.md section 2.3).
//
// Canonical frame: start at the origin with velocity (v0, 0), fresh
// state, clean air. The control family is MAXIMUM-GAIN wishes (stored
// cosa 0 = wish perpendicular to velocity = the engine's best gain,
// +900 to squared speed per tick) whose only freedom is the strafe
// side: k <= 3 reversal times, all >= 6 ticks apart (the dwell law).
// An EXACT-HIT TAIL then frees the stored cosa of the last tail_m
// ticks (two constant values over two halves) so the endpoint moves
// continuously and Newton's method can land on the target exactly.
// Every candidate is evaluated as an exact rollout through the
// bitwise-certified A4 kernel - the answer is an exact, replayable
// input schedule, never an approximation.
// ======================================================================
namespace CapP2P {

	struct P2PSchedule {
		signed char side[128];
		float cosa[128];
		int n = 0;
	};
	struct P2PResult {
		bool solved = false;
		float ex = 0.f, ey = 0.f;    // achieved endpoint
		float evx = 0.f, evy = 0.f;  // achieved terminal velocity
		float residual = 1e30f;      // |endpoint - target|
		int rollouts = 0;            // exact kernel rollouts spent
		long long law_evals = 0;     // O(1) segment-composition evals
		int start_side = 1;
		int nrev = 0;
		int revs[3] = { 0, 0, 0 };
		// v2 range control: a braking run (stored cosa near +1 =
		// true wish opposing velocity) of brake_len ticks starting
		// at brake_start; 0 length = none
		int brake_start = 0, brake_len = 0;
		float brake_cosa = 0.9f;
		float tail_c1 = 0.f, tail_c2 = 0.f;
		P2PSchedule sched;
	};

	// Schedule text: constant stored-cosa-0 strafing on start_side,
	// flipping at each reversal tick; the last tail_m ticks override
	// stored cosa with c1 (first half) and c2 (second half).
	inline void BuildSchedule(int N, int start_side, const int* revs,
	                          int nrev, int brake_start, int brake_len,
	                          float brake_cosa, int tail_m, float c1,
	                          float c2, P2PSchedule* s) {
		s->n = N;
		signed char cur = static_cast<signed char>(start_side);
		int ri = 0;
		for (int t = 0; t < N; ++t) {
			if (ri < nrev && t == revs[ri]) {
				cur = static_cast<signed char>(-cur);
				ri++;
			}
			s->side[t] = cur;
			s->cosa[t] = 0.f;
		}
		// the brake run overrides stored cosa only (side changes are
		// what dwell constrains; cosa is free every tick)
		for (int t = brake_start;
			t < brake_start + brake_len && t < N; ++t)
			if (t >= 0)
				s->cosa[t] = brake_cosa;
		if (tail_m > 0) {
			const int t0 = N - tail_m;
			const int half = tail_m / 2;
			for (int t = t0; t < N; ++t)
				if (t >= 0)
					s->cosa[t] = (t < t0 + half) ? c1 : c2;
		}
	}

	// One exact rollout from the canonical start (positions live in
	// the kernel's own advance; vz = 0 per tick, the stamped domain).
	inline void Roll(const CapAir::AirKernelCtx& k, float v0,
	                 const P2PSchedule& s, float* ex, float* ey,
	                 float* evx, float* evy) {
		float px = 0.f, py = 0.f;
		float vx = v0, vy = 0.f;
		for (int t = 0; t < s.n; ++t) {
			float npx, npy, npz, nvx, nvy, nvz;
			CapAir::KernelTick(k, px, py, 8000.f, vx, vy, 0.f,
				s.side[t], s.cosa[t], &npx, &npy, &npz, &nvx,
				&nvy, &nvz);
			px = npx;
			py = npy;
			vx = nvx;
			vy = nvy;
		}
		*ex = px;
		*ey = py;
		*evx = vx;
		*evy = vy;
	}

	// The fixed-tick solve (registry A11), v2. The v1 measurement
	// (session 26) drove three changes, all recorded honestly:
	// (a) single-candidate hill-climbing stalled in local minima
	//     (55-87% hits on the solver's OWN family targets) ->
	//     stage 1 now keeps the top-6 coarse candidates and refines
	//     each;
	// (b) the family-sufficiency falsifier showed max-gain paths
	//     live on a thin arc-length shell (general targets ~0% hit,
	//     interior by up to ~1300u) -> the BRAKE RUN range control:
	//     a run of ticks with the stored cosa near +1 (true wish
	//     opposing velocity) sheds speed at up to the accel budget
	//     per tick, shortening the path; max-gain ticks re-gain
	//     afterward. Two integers (start, length) span the range
	//     from the max-gain shell deep into the interior;
	// (c) the exact-hit tail gets more authority (longer tail, more
	//     Newton iterations, restarts).
	// ---- v4 (session 27): EXACT SEGMENT COMPOSITION for reversal
	// schedules. Under maximum gain the per-tick turn magnitude
	// atan(cap/s_n) and step length s_{n+1}*dt are schedule-
	// independent, so a constant-side run [a, b) is always the SAME
	// spiral segment (or its mirror for the other side). Precomputing
	// the all-plus trajectory's prefix headings/positions therefore
	// lets ANY reversal schedule's endpoint be composed in O(k) with
	// no trigonometry (rotation composition uses the tabled cos/sin).
	// This is the robotics motion-primitive concatenation trick
	// specialized to our spirals (the family itself is the Dubins
	// arc-segment structure with speed-varying turn radius). It makes
	// the candidate enumeration EXHAUSTIVE over k <= 2 and dense over
	// k = 3 at ~100 ns per candidate; the certified law-identity gate
	// bounds the law-vs-kernel deviation (~1u over 90 ticks), and the
	// top candidates are kernel-verified before refinement.
	struct SegTables {
		int N = 0;
		std::vector<double> cpsi, spsi;   // cos/sin of prefix heading
		std::vector<double> wx, wy;       // prefix position
	};
	inline void BuildSegTables(const MoveParams& p, float v0, int N,
	                           SegTables* T) {
		T->N = N;
		T->cpsi.assign(static_cast<size_t>(N) + 1, 1.0);
		T->spsi.assign(static_cast<size_t>(N) + 1, 0.0);
		T->wx.assign(static_cast<size_t>(N) + 1, 0.0);
		T->wy.assign(static_cast<size_t>(N) + 1, 0.0);
		double psi = 0.0, x = 0.0, y = 0.0;
		double s2 = static_cast<double>(v0) * v0;
		const double cap = p.air_speed_cap;
		const double dt = p.dt;
		for (int n = 0; n < N; ++n) {
			const double s = sqrt(s2);
			psi += atan2(cap, s);       // turn BEFORE the step
			s2 += cap * cap;
			const double snew = sqrt(s2);
			x += snew * dt * cos(psi);  // step along the NEW heading
			y += snew * dt * sin(psi);
			T->cpsi[static_cast<size_t>(n) + 1] = cos(psi);
			T->spsi[static_cast<size_t>(n) + 1] = sin(psi);
			T->wx[static_cast<size_t>(n) + 1] = x;
			T->wy[static_cast<size_t>(n) + 1] = y;
		}
	}
	inline void EvalSegs(const SegTables& T, int s0sign,
	                     const int* revs, int nrev, float* ex,
	                     float* ey) {
		double cphi = 1.0, sphi = 0.0;
		double px = 0.0, py = 0.0;
		int a = 0;
		int sign = s0sign;
		for (int j = 0; j <= nrev; ++j) {
			const int b = j < nrev ? revs[j] : T.N;
			if (b > a) {
				const double dx = T.wx[static_cast<size_t>(b)]
					- T.wx[static_cast<size_t>(a)];
				const double dy = T.wy[static_cast<size_t>(b)]
					- T.wy[static_cast<size_t>(a)];
				const double ca = T.cpsi[static_cast<size_t>(a)];
				const double sa = T.spsi[static_cast<size_t>(a)];
				const double cb = T.cpsi[static_cast<size_t>(b)];
				const double sb = T.spsi[static_cast<size_t>(b)];
				// the segment in its own starting frame
				double lx = ca * dx + sa * dy;
				double ly = -sa * dx + ca * dy;
				// the segment's rotation e^{i(psi_b - psi_a)}
				double cd = cb * ca + sb * sa;
				double sd = sb * ca - cb * sa;
				// THE BASIS BRIDGE (SolverStrafe.h): a STORED side
				// of +1 rotates the heading NEGATIVE (true rotation
				// side = -stored side, measured 98/98). The tables
				// are built positive-rotation, so stored +1 takes
				// the mirror.
				if (sign > 0) {
					ly = -ly;
					sd = -sd;
				}
				px += cphi * lx - sphi * ly;
				py += sphi * lx + cphi * ly;
				const double nc = cphi * cd - sphi * sd;
				sphi = cphi * sd + sphi * cd;
				cphi = nc;
			}
			sign = -sign;
			a = b;
		}
		*ex = static_cast<float>(px);
		*ey = static_cast<float>(py);
	}

	struct P2PCand {
		float res = 1e30f;
		int ss = 1;
		int nrev = 0;
		int revs[3] = { 0, 0, 0 };
		int bs = 0, bl = 0;
	};
	inline void SolveFixedN(const CapAir::AirKernelCtx& k,
	                        float v0, float tx, float ty, int N,
	                        P2PResult* out) {
		P2PResult best;
		P2PSchedule s;
		const float bc = 0.9f;   // brake strength (stored cosa)
		auto eval = [&](int ss, const int* revs, int nrev, int bs2,
			int bl2, int tail_m, float c1, float c2) {
			BuildSchedule(N, ss, revs, nrev, bs2, bl2, bc, tail_m,
				c1, c2, &s);
			float ex, ey, evx, evy;
			Roll(k, v0, s, &ex, &ey, &evx, &evy);
			best.rollouts++;
			const float dx = ex - tx, dy = ey - ty;
			const float r = sqrtf(dx * dx + dy * dy);
			if (r < best.residual) {
				best.residual = r;
				best.ex = ex;
				best.ey = ey;
				best.evx = evx;
				best.evy = evy;
				best.start_side = ss;
				best.nrev = nrev;
				for (int i = 0; i < 3; ++i)
					best.revs[i] = i < nrev ? revs[i] : 0;
				best.brake_start = bs2;
				best.brake_len = bl2;
				best.brake_cosa = bc;
				best.tail_c1 = c1;
				best.tail_c2 = c2;
				best.sched = s;
			}
			return r;
		};
		// ---- stage 1: coarse grid over both start sides, keeping
		// the top-6 candidates for refinement
		const int K = 6;
		P2PCand top[6];
		auto offer = [&](float r, int ss, const int* revs, int nrev,
			int bs2, int bl2) {
			int worst = 0;
			for (int i = 1; i < K; ++i)
				if (top[i].res > top[worst].res)
					worst = i;
			if (r < top[worst].res) {
				top[worst].res = r;
				top[worst].ss = ss;
				top[worst].nrev = nrev;
				for (int i = 0; i < 3; ++i)
					top[worst].revs[i] = i < nrev ? revs[i] : 0;
				top[worst].bs = bs2;
				top[worst].bl = bl2;
			}
		};
		// segment tables for this (v0, N): the law endpoints of any
		// reversal schedule in O(k) each
		SegTables T;
		BuildSegTables(k.p, v0, N, &T);
		auto offerSeg = [&](int ss, const int* revs, int nrev) {
			float ex, ey;
			EvalSegs(T, ss, revs, nrev, &ex, &ey);
			best.law_evals++;
			const float dx = ex - tx, dy = ey - ty;
			offer(sqrtf(dx * dx + dy * dy), ss, revs, nrev, 0, 0);
		};
		for (int ss = -1; ss <= 1; ss += 2) {
			int revs[3] = { 0, 0, 0 };
			offerSeg(ss, revs, 0);
			// k = 1 and k = 2: EXHAUSTIVE (every legal reversal
			// placement)
			for (int r1 = 1; r1 < N; ++r1) {
				revs[0] = r1;
				offerSeg(ss, revs, 1);
				for (int r2 = r1 + 6; r2 < N; ++r2) {
					revs[1] = r2;
					offerSeg(ss, revs, 2);
				}
			}
			// k = 3: dense (strides 2/2/3; the stage-2 climb spans
			// the gaps)
			for (int r1 = 1; r1 < N; r1 += 2) {
				revs[0] = r1;
				for (int r2 = r1 + 6; r2 < N; r2 += 2) {
					revs[1] = r2;
					for (int r3 = r2 + 6; r3 < N; r3 += 3) {
						revs[2] = r3;
						offerSeg(ss, revs, 3);
					}
				}
			}
			// the brake probes (range control): straight-line and
			// one-reversal shapes with a brake run at several
			// start positions (short flights need the variety)
			const int bss[3] = { 1, N / 4, N / 2 };
			// stride clamped: N < 8 made N/8 = 0, an infinite loop
			// (latent until A12's small-N scan, session 33)
			const int blstep = N / 8 < 1 ? 1 : N / 8;
			for (int bi = 0; bi < 3; ++bi)
				for (int bl = blstep; bl <= (3 * N) / 4;
					bl += blstep) {
					if (bl < 2)
						continue;
					int rv0[3] = { 0, 0, 0 };
					offer(eval(ss, rv0, 0, bss[bi], bl, 0, 0.f,
						0.f), ss, rv0, 0, bss[bi], bl);
					for (int r1 = 6; r1 < N; r1 += 12) {
						rv0[0] = r1;
						offer(eval(ss, rv0, 1, bss[bi], bl, 0,
							0.f, 0.f), ss, rv0, 1, bss[bi],
							bl);
					}
				}
		}
		// ---- stage 2: refine EACH top candidate (single reversal
		// steps, PAIRED translation of the whole pattern - coupled
		// valleys stall one-at-a-time moves - and brake steps),
		// writing the refined shape back into the candidate list
		for (int ci = 0; ci < K; ++ci) {
			if (top[ci].res > 1e29f)
				continue;
			P2PCand c = top[ci];
			// kernel-truth seed evaluation of the candidate's own
			// shape (stage 1's law ranking is ~1u off the kernel;
			// v3 also never kernel-evaluated an unimproved winner)
			float cur = eval(c.ss, c.revs, c.nrev, c.bs, c.bl, 0,
				0.f, 0.f);
			bool improved = true;
			int guard = 0;
			while (improved && guard++ < 12) {
				improved = false;
				for (int i = 0; i < c.nrev; ++i)
					for (int d = -6; d <= 6; ++d) {
						if (d == 0)
							continue;
						int revs[3] = { c.revs[0], c.revs[1],
							c.revs[2] };
						revs[i] += d;
						bool ok = true;
						for (int j = 0; j < c.nrev; ++j) {
							if (revs[j] < 1 || revs[j] >= N)
								ok = false;
							if (j > 0 && revs[j] - revs[j - 1]
								< 6)
								ok = false;
						}
						if (!ok)
							continue;
						const float r = eval(c.ss, revs, c.nrev,
							c.bs, c.bl, 0, 0.f, 0.f);
						if (r < cur - 1e-4f) {
							cur = r;
							for (int j = 0; j < 3; ++j)
								c.revs[j] = revs[j];
							improved = true;
						}
					}
				// paired translation of the whole pattern
				if (c.nrev > 0) {
					const int dts[4] = { -6, -3, 3, 6 };
					for (int m = 0; m < 4; ++m) {
						int revs[3] = { c.revs[0], c.revs[1],
							c.revs[2] };
						bool ok = true;
						for (int j = 0; j < c.nrev; ++j) {
							revs[j] += dts[m];
							if (revs[j] < 1 || revs[j] >= N)
								ok = false;
						}
						if (!ok)
							continue;
						const float r = eval(c.ss, revs, c.nrev,
							c.bs, c.bl, 0, 0.f, 0.f);
						if (r < cur - 1e-4f) {
							cur = r;
							for (int j = 0; j < 3; ++j)
								c.revs[j] = revs[j];
							improved = true;
						}
					}
				}
				if (c.bl > 0) {
					const int dbs[4] = { -4, -2, 2, 4 };
					for (int m = 0; m < 4; ++m) {
						const int nb = c.bs + dbs[m];
						if (nb < 0 || nb >= N)
							continue;
						const float r = eval(c.ss, c.revs,
							c.nrev, nb, c.bl, 0, 0.f, 0.f);
						if (r < cur - 1e-4f) {
							cur = r;
							c.bs = nb;
							improved = true;
						}
					}
					const int dbl[4] = { -6, -3, 3, 6 };
					for (int m = 0; m < 4; ++m) {
						const int nl = c.bl + dbl[m];
						if (nl < 0 || c.bs + nl > N)
							continue;
						const float r = eval(c.ss, c.revs,
							c.nrev, c.bs, nl, 0, 0.f, 0.f);
						if (r < cur - 1e-4f) {
							cur = r;
							c.bl = nl;
							improved = true;
						}
					}
				}
			}
			top[ci] = c;
			top[ci].res = cur;
		}
		// ---- stage 3: the exact-hit tail - 2-parameter Newton on
		// the two stored-cosa values of the tail halves, from three
		// starting points, on the TOP THREE refined candidates
		// (running it only on the global best left basins unclosed -
		// the v2 machinery measurement)
		{
			const int tail_m = N >= 48 ? 24
				: (N >= 24 ? 12 : (N >= 12 ? 8 : 0));
			for (int rank = 0; rank < 3 && tail_m > 0
				&& best.residual > 0.25f; ++rank) {
				// select the best not-yet-consumed refined
				// candidate (consumed ones are marked -1)
				int sel = -1;
				float selr = 1e30f;
				for (int i = 0; i < K; ++i)
					if (top[i].res >= 0.f && top[i].res < selr) {
						selr = top[i].res;
						sel = i;
					}
				if (sel < 0)
					break;
				top[sel].res = -1.f;   // mark consumed
				int revs[3] = { top[sel].revs[0],
					top[sel].revs[1], top[sel].revs[2] };
				const int nrev = top[sel].nrev;
				const int ss = top[sel].ss;
				const int bs2 = top[sel].bs;
				const int bl2 = top[sel].bl;
				const float starts[3][2] = {
					{ 0.f, 0.f }, { 0.35f, -0.35f },
					{ -0.35f, 0.35f } };
				for (int st = 0; st < 3
					&& best.residual > 0.25f; ++st) {
					float c1 = starts[st][0];
					float c2 = starts[st][1];
					for (int it = 0; it < 8; ++it) {
						BuildSchedule(N, ss, revs, nrev, bs2,
							bl2, bc, tail_m, c1, c2, &s);
						float ex, ey, evx, evy;
						Roll(k, v0, s, &ex, &ey, &evx, &evy);
						best.rollouts++;
						const float fx = ex - tx, fy = ey - ty;
						const float r0 = sqrtf(fx * fx
							+ fy * fy);
						if (r0 < best.residual) {
							best.residual = r0;
							best.ex = ex;
							best.ey = ey;
							best.evx = evx;
							best.evy = evy;
							best.tail_c1 = c1;
							best.tail_c2 = c2;
							best.sched = s;
						}
						if (r0 <= 0.25f)
							break;
						const float h = 0.02f;
						float ex1, ey1, ex2, ey2, dvx, dvy;
						BuildSchedule(N, ss, revs, nrev, bs2,
							bl2, bc, tail_m, c1 + h, c2, &s);
						Roll(k, v0, s, &ex1, &ey1, &dvx, &dvy);
						BuildSchedule(N, ss, revs, nrev, bs2,
							bl2, bc, tail_m, c1, c2 + h, &s);
						Roll(k, v0, s, &ex2, &ey2, &dvx, &dvy);
						best.rollouts += 2;
						const float j11 = (ex1 - ex) / h;
						const float j21 = (ey1 - ey) / h;
						const float j12 = (ex2 - ex) / h;
						const float j22 = (ey2 - ey) / h;
						const float det = j11 * j22
							- j12 * j21;
						if (fabsf(det) < 1e-6f)
							break;
						float d1 = (-fx * j22 + fy * j12)
							/ det;
						float d2 = (-j11 * fy + j21 * fx)
							/ det;
						if (d1 > 0.3f) d1 = 0.3f;
						if (d1 < -0.3f) d1 = -0.3f;
						if (d2 > 0.3f) d2 = 0.3f;
						if (d2 < -0.3f) d2 = -0.3f;
						c1 += d1;
						c2 += d2;
						if (c1 > 0.95f) c1 = 0.95f;
						if (c1 < -0.95f) c1 = -0.95f;
						if (c2 > 0.95f) c2 = 0.95f;
						if (c2 < -0.95f) c2 = -0.95f;
					}
				}
			}
		}
		best.solved = best.residual <= 0.5f;
		*out = best;
	}

	// ---- BATCH MODE (session 30): organize the family ONCE per
	// start, answer many targets in near-constant time each. All
	// enumerated law endpoints go into a spatial hash; a target seeds
	// from its cell neighborhood, kernel-verifies the top seeds,
	// climbs the best, and finishes with the exact-hit tail. This is
	// the shape the transfer matrices (registry C7) consume: one
	// start, many exit targets.
	struct P2PBatchCand {
		float ex = 0.f, ey = 0.f;
		signed char ss = 1;
		signed char nrev = 0;
		int revs[3] = { 0, 0, 0 };
	};
	struct P2PBatch {
		float v0 = 0.f;
		int N = 0;
		SegTables T;
		std::vector<P2PBatchCand> cands;
		std::unordered_map<long long, std::vector<int> > grid;
		float cell = 64.f;
		double build_ms = 0.0;
	};
	inline long long BCellOf(const P2PBatch& B, float x, float y) {
		const long long ix = static_cast<long long>(
			floorf(x / B.cell));
		const long long iy = static_cast<long long>(
			floorf(y / B.cell));
		return ix * 1000003ll + iy;
	}
	inline void BuildP2PBatch(const CapAir::AirKernelCtx& k, float v0,
	                          int N, P2PBatch* B) {
		const auto t0 = std::chrono::steady_clock::now();
		B->v0 = v0;
		B->N = N;
		BuildSegTables(k.p, v0, N, &B->T);
		B->cands.clear();
		B->grid.clear();
		auto add = [&](int ss, const int* revs, int nrev) {
			P2PBatchCand c;
			EvalSegs(B->T, ss, revs, nrev, &c.ex, &c.ey);
			c.ss = static_cast<signed char>(ss);
			c.nrev = static_cast<signed char>(nrev);
			for (int i = 0; i < 3; ++i)
				c.revs[i] = i < nrev ? revs[i] : 0;
			const int idx = static_cast<int>(B->cands.size());
			B->cands.push_back(c);
			B->grid[BCellOf(*B, c.ex, c.ey)].push_back(idx);
		};
		for (int ss = -1; ss <= 1; ss += 2) {
			int revs[3] = { 0, 0, 0 };
			add(ss, revs, 0);
			for (int r1 = 1; r1 < N; ++r1) {
				revs[0] = r1;
				add(ss, revs, 1);
				for (int r2 = r1 + 6; r2 < N; ++r2) {
					revs[1] = r2;
					add(ss, revs, 2);
				}
			}
			for (int r1 = 1; r1 < N; r1 += 2) {
				revs[0] = r1;
				for (int r2 = r1 + 6; r2 < N; r2 += 2) {
					revs[1] = r2;
					for (int r3 = r2 + 6; r3 < N; r3 += 3) {
						revs[2] = r3;
						add(ss, revs, 3);
					}
				}
			}
		}
		B->build_ms = std::chrono::duration<double, std::milli>(
			std::chrono::steady_clock::now() - t0).count();
	}
	inline void SolveTargetBatch(const CapAir::AirKernelCtx& k,
	                             const P2PBatch& B, float tx, float ty,
	                             P2PResult* out) {
		const int N = B.N;
		P2PResult best;
		P2PSchedule s;
		const float bc = 0.9f;
		auto eval = [&](int ss, const int* revs, int nrev, int bs2,
			int bl2, int tail_m, float c1, float c2) {
			BuildSchedule(N, ss, revs, nrev, bs2, bl2, bc, tail_m,
				c1, c2, &s);
			float ex, ey, evx, evy;
			Roll(k, B.v0, s, &ex, &ey, &evx, &evy);
			best.rollouts++;
			const float dx = ex - tx, dy = ey - ty;
			const float r = sqrtf(dx * dx + dy * dy);
			if (r < best.residual) {
				best.residual = r;
				best.ex = ex;
				best.ey = ey;
				best.evx = evx;
				best.evy = evy;
				best.start_side = ss;
				best.nrev = nrev;
				for (int i = 0; i < 3; ++i)
					best.revs[i] = i < nrev ? revs[i] : 0;
				best.brake_start = bs2;
				best.brake_len = bl2;
				best.tail_c1 = c1;
				best.tail_c2 = c2;
				best.sched = s;
			}
			return r;
		};
		// ---- seeds from the spatial hash (3x3, expand to 5x5)
		int seed_idx[6] = { -1, -1, -1, -1, -1, -1 };
		float seed_res[6];
		for (int i = 0; i < 6; ++i)
			seed_res[i] = 1e30f;
		auto offer_seed = [&](int idx) {
			const P2PBatchCand& c = B.cands[
				static_cast<size_t>(idx)];
			const float dx = c.ex - tx, dy = c.ey - ty;
			const float r = sqrtf(dx * dx + dy * dy);
			int worst = 0;
			for (int i = 1; i < 6; ++i)
				if (seed_res[i] > seed_res[worst])
					worst = i;
			if (r < seed_res[worst]) {
				seed_res[worst] = r;
				seed_idx[worst] = idx;
			}
		};
		const long long cx = static_cast<long long>(
			floorf(tx / B.cell));
		const long long cy = static_cast<long long>(
			floorf(ty / B.cell));
		for (int ring = 1; ring <= 2; ++ring) {
			for (long long da = -ring; da <= ring; ++da)
				for (long long db = -ring; db <= ring; ++db) {
					std::unordered_map<long long,
						std::vector<int> >::const_iterator jt =
						B.grid.find((cx + da) * 1000003ll
							+ (cy + db));
					if (jt == B.grid.end())
						continue;
					for (size_t q = 0; q < jt->second.size();
						++q)
						offer_seed(jt->second[q]);
				}
			if (seed_idx[0] >= 0 && ring == 1)
				break;
		}
		// kernel-verify the seeds
		for (int i = 0; i < 6; ++i) {
			if (seed_idx[i] < 0)
				continue;
			const P2PBatchCand& c = B.cands[
				static_cast<size_t>(seed_idx[i])];
			eval(c.ss, c.revs, c.nrev, 0, 0, 0, 0.f, 0.f);
		}
		// interior fallback: the brake probes when the family shell
		// cannot reach the target
		if (best.residual > 48.f) {
			// stride clamped (the same small-N zero-stride hazard as
			// the fixed-N brake probes)
			const int blstep2 = N / 8 < 1 ? 1 : N / 8;
			for (int ss = -1; ss <= 1; ss += 2)
				for (int bl = blstep2; bl <= (3 * N) / 4;
					bl += blstep2) {
					if (bl < 2)
						continue;
					int rv0[3] = { 0, 0, 0 };
					eval(ss, rv0, 0, 1, bl, 0, 0.f, 0.f);
					for (int r1 = 6; r1 < N; r1 += 12) {
						rv0[0] = r1;
						eval(ss, rv0, 1, 1, bl, 0, 0.f, 0.f);
					}
				}
		}
		// climb the best shape (translation + per-reversal steps +
		// brake steps)
		{
			bool improved = true;
			int guard = 0;
			while (improved && guard++ < 10) {
				improved = false;
				const float prev = best.residual;
				const int nrev = best.nrev;
				for (int i = 0; i < nrev; ++i)
					for (int d = -6; d <= 6; ++d) {
						if (d == 0)
							continue;
						int revs[3] = { best.revs[0],
							best.revs[1], best.revs[2] };
						revs[i] += d;
						bool ok = true;
						for (int j = 0; j < nrev; ++j) {
							if (revs[j] < 1 || revs[j] >= N)
								ok = false;
							if (j > 0 && revs[j] - revs[j - 1]
								< 6)
								ok = false;
						}
						if (ok)
							eval(best.start_side, revs, nrev,
								best.brake_start,
								best.brake_len, 0, 0.f, 0.f);
					}
				if (best.brake_len > 0) {
					const int dbs[4] = { -4, -2, 2, 4 };
					for (int m = 0; m < 4; ++m) {
						const int nb = best.brake_start + dbs[m];
						if (nb >= 0 && nb < N)
							eval(best.start_side, best.revs,
								best.nrev, nb, best.brake_len,
								0, 0.f, 0.f);
					}
					const int dbl[4] = { -6, -3, 3, 6 };
					for (int m = 0; m < 4; ++m) {
						const int nl = best.brake_len + dbl[m];
						if (nl >= 0 && best.brake_start + nl
							<= N)
							eval(best.start_side, best.revs,
								best.nrev, best.brake_start,
								nl, 0, 0.f, 0.f);
					}
				}
				if (best.residual < prev - 1e-4f)
					improved = true;
			}
		}
		// the exact-hit tail (three starts)
		{
			const int tail_m = N >= 48 ? 24
				: (N >= 24 ? 12 : (N >= 12 ? 8 : 0));
			if (tail_m > 0 && best.residual > 0.25f) {
				int revs[3] = { best.revs[0], best.revs[1],
					best.revs[2] };
				const int nrev = best.nrev;
				const int ss = best.start_side;
				const int bs2 = best.brake_start;
				const int bl2 = best.brake_len;
				const float starts[3][2] = {
					{ 0.f, 0.f }, { 0.35f, -0.35f },
					{ -0.35f, 0.35f } };
				for (int st = 0; st < 3
					&& best.residual > 0.25f; ++st) {
					float c1 = starts[st][0];
					float c2 = starts[st][1];
					for (int it = 0; it < 8; ++it) {
						BuildSchedule(N, ss, revs, nrev, bs2,
							bl2, bc, tail_m, c1, c2, &s);
						float ex, ey, evx, evy;
						Roll(k, B.v0, s, &ex, &ey, &evx, &evy);
						best.rollouts++;
						const float fx = ex - tx, fy = ey - ty;
						const float r0 = sqrtf(fx * fx
							+ fy * fy);
						if (r0 < best.residual) {
							best.residual = r0;
							best.ex = ex;
							best.ey = ey;
							best.evx = evx;
							best.evy = evy;
							best.tail_c1 = c1;
							best.tail_c2 = c2;
							best.sched = s;
						}
						if (r0 <= 0.25f)
							break;
						const float h = 0.02f;
						float ex1, ey1, ex2, ey2, dvx, dvy;
						BuildSchedule(N, ss, revs, nrev, bs2,
							bl2, bc, tail_m, c1 + h, c2, &s);
						Roll(k, B.v0, s, &ex1, &ey1, &dvx,
							&dvy);
						BuildSchedule(N, ss, revs, nrev, bs2,
							bl2, bc, tail_m, c1, c2 + h, &s);
						Roll(k, B.v0, s, &ex2, &ey2, &dvx,
							&dvy);
						best.rollouts += 2;
						const float j11 = (ex1 - ex) / h;
						const float j21 = (ey1 - ey) / h;
						const float j12 = (ex2 - ex) / h;
						const float j22 = (ey2 - ey) / h;
						const float det = j11 * j22
							- j12 * j21;
						if (fabsf(det) < 1e-6f)
							break;
						float d1 = (-fx * j22 + fy * j12)
							/ det;
						float d2 = (-j11 * fy + j21 * fx)
							/ det;
						if (d1 > 0.3f) d1 = 0.3f;
						if (d1 < -0.3f) d1 = -0.3f;
						if (d2 > 0.3f) d2 = 0.3f;
						if (d2 < -0.3f) d2 = -0.3f;
						c1 += d1;
						c2 += d2;
						if (c1 > 0.95f) c1 = 0.95f;
						if (c1 < -0.95f) c1 = -0.95f;
						if (c2 > 0.95f) c2 = 0.95f;
						if (c2 < -0.95f) c2 = -0.95f;
					}
				}
			}
		}
		best.solved = best.residual <= 0.5f;
		// completeness backstop: the rare target whose basin the
		// seed neighborhood misses falls back to the full solver
		// (guaranteed not worse; keeps the batch average fast)
		if (!best.solved) {
			P2PResult full;
			SolveFixedN(k, B.v0, tx, ty, B.N, &full);
			full.rollouts += best.rollouts;
			if (full.residual < best.residual)
				best = full;
			best.solved = best.residual <= 0.5f;
		}
		*out = best;
	}

} // namespace CapP2P

// ======================================================================
// CAPGROUND - registry A25 (flat-ground movement law) and A30 (jump
// law), session 27. Partial evaluation of the GROUNDED MoveTick chain
// under the FLAT-FLOOR domain: one axial floor plane, standing hull,
// no duck (A29 debt), no water (A32), no walls within a tick's sweep,
// surface_friction 1 on ground. Every elision cites the engine line:
//   - walk tick (MoveTick 931-957 grounded branch): stamina drain
//     (887-891) -> StartGravity half (933) -> basevel.Z (934-935) ->
//     jump gate -> grounded vz zero + Friction (940-943) ->
//     CheckVelocity -> WalkMove (465-531: stamina pow scale, wish,
//     ground accelerate with NO 30 cap, vz zero, <1 u/s stop, flat
//     trace frac 1, StayOnGround, vz zero) -> CategorizePosition
//     (state-only on flat ground, no origin snap) -> CheckVelocity ->
//     FinishGravity half (951) -> grounded vz zero -> CheckVelocity.
//   - jump tick (CheckJumpButton 295-379, ADD path): gates ->
//     leaves ground -> vz = float(double(impulse) + double(startz))
//     -> stamina scale -> stamina armed -> FinishGravity FUNCTION
//     (275-289, the (g*e)*(dt*0.5) pairing + CheckVelocity) -> then
//     the AIRBORNE chain of the tick (the A4 kernel's own middle).
// Gates: capground (parity vs the engine on the flat world, bitwise).
// ======================================================================
namespace CapGround {

	struct GroundCtx {
		MoveParams p;
		Vec3 basevel;                 // runtime zeros
		float rest_z = 0.f;           // settled feet height (measured)
		float floor_top_padded = 0.f; // rest-frame plane for the
		                              // StayOnGround analytic trace
	};
	inline GroundCtx MakeGroundCtx(const MoveParams& p, float rest_z) {
		GroundCtx g;
		g.p = p;
		g.rest_z = rest_z;
		// the settled hull rests ONE trace epsilon (1/32) above the
		// hull-expanded floor plane - the analytic StayOnGround trace
		// measures distances to the PLANE (measured: engine keeps
		// feet z 0.03125 with the plane at 0)
		g.floor_top_padded = rest_z - 0.03125f;
		return g;
	}
	// the flat-floor test world: an axial slab, top at z = 0
	inline bool MakeFlatWorld(World* w, const Hulls& hulls) {
		std::vector<Vec3> ns;
		std::vector<float> ds;
		ns.push_back(Vec3(0.f, 0.f, 1.f));
		ds.push_back(0.f);
		ns.push_back(Vec3(0.f, 0.f, -1.f));
		ds.push_back(200.f);
		ns.push_back(Vec3(1.f, 0.f, 0.f));
		ds.push_back(20000.f);
		ns.push_back(Vec3(-1.f, 0.f, 0.f));
		ds.push_back(20000.f);
		ns.push_back(Vec3(0.f, 1.f, 0.f));
		ds.push_back(20000.f);
		ns.push_back(Vec3(0.f, -1.f, 0.f));
		ds.push_back(20000.f);
		if (!w->AddTestBrush(ns, ds, hulls))
			return false;
		w->FinalizeTestWorld();
		return true;
	}
	// Friction, re-coded 1-to-1 (SolverMove.cpp 260-272; the function
	// is file-local there)
	inline void KFriction(const MoveParams& p, float sf, Vec3* vel) {
		const float speed = Len(*vel);
		if (speed < 0.1f)
			return;
		const float control = (speed < p.stopspeed) ? p.stopspeed
			: speed;
		const float drop = control * p.friction * sf * p.dt;
		float newspeed = speed - drop;
		if (newspeed < 0.f)
			newspeed = 0.f;
		if (newspeed != speed)
			*vel = Scale(*vel, newspeed / speed);
	}
	// FinishGravity FUNCTION pairing (SolverMove.cpp 275-289): fires
	// only inside a successful jump
	inline void KFinishGravityFn(const MoveParams& p,
	                             float gravity_scale, Vec3* vel) {
		float ent_gravity = gravity_scale;
		if (ent_gravity == 0.f)
			ent_gravity = 1.f;
		vel->Z -= (p.gravity * ent_gravity) * (p.dt * 0.5f);
		CapAir::KernelCheckVelocity(p, &vel->X, &vel->Y, &vel->Z);
	}
	// The flat-ground WALK tick (buttons = 0 or a refused jump).
	// Outputs the full next state; stamina is carried state.
	inline void WalkKernelTick(const GroundCtx& g, float px, float py,
	                           float pz, float vx, float vy,
	                           float stamina, float yaw, float fm,
	                           float sm, float* npx, float* npy,
	                           float* npz, float* nvx, float* nvy,
	                           float* nst) {
		const MoveParams& p = g.p;
		Vec3 pos(px, py, pz);
		Vec3 vel(vx, vy, 0.f);
		// stamina drain (887-891)
		float st = stamina;
		if (st > 0.f) {
			st -= p.dt * 1000.f;
			if (st < 0.f)
				st = 0.f;
		}
		// StartGravity half + basevel.Z (933-935)
		vel.Z -= 1.f * p.gravity * 0.5f * p.dt;
		vel.Z += g.basevel.Z * p.dt;
		// grounded: vz zero + Friction (940-943)
		vel.Z = 0.f;
		KFriction(p, 1.f, &vel);
		CapAir::KernelCheckVelocity(p, &vel.X, &vel.Y, &vel.Z);
		// WalkMove (465-531)
		if (st > 0.f) {
			const float base = 1.f - st * p.stamina_scale_per_ms;
			const float ratio = powf(base, p.dt
				* p.stamina_pow_rate);
			vel.X *= ratio;
			vel.Y *= ratio;
		}
		float wx, wy;
		Fn::WishFromInput(yaw, fm, sm, &wx, &wy);
		float wishspeed = sqrtf(wx * wx + wy * wy);
		Vec3 wishdir(0.f, 0.f, 0.f);
		if (wishspeed > 1e-6f)
			wishdir = Vec3(wx / wishspeed, wy / wishspeed, 0.f);
		if (wishspeed > p.maxspeed)
			wishspeed = p.maxspeed;
		const float cur = Dot(vel, wishdir);
		const float add = wishspeed - cur;
		if (add > 0.f) {
			float accelspeed = p.accelerate * p.dt * wishspeed
				* 1.f;
			if (accelspeed > add)
				accelspeed = add;
			vel = vel + Scale(wishdir, accelspeed);
		}
		vel.Z = 0.f;
		bool early_stop = false;
		if (Len(vel) < 1.f) {
			vel = Vec3();
			early_stop = true;   // WalkMove returns before the move
		}
		if (!early_stop) {
			const Vec3 bv(g.basevel.X, g.basevel.Y, 0.f);
			vel = vel + bv;
			// the straight trace is free on the flat floor
			// (frac 1, 519-524)
			pos = Vec3(pos.X + vel.X * p.dt, pos.Y + vel.Y * p.dt,
				pos.Z);
			vel = vel - bv;
			// StayOnGround (446-463) against the single floor
			// plane: up-trace free (fu = 1), down-trace fraction
			// mirrors TraceHull3's padded entry
			{
				const float startz = pos.Z + 2.f;
				const float endz = pos.Z - p.stepsize;
				const float d0 = startz - g.floor_top_padded;
				const float d1 = endz - g.floor_top_padded;
				if (d0 > 0.f && d1 <= 0.f) {
					float tt = (d0 - 0.03125f) / (d0 - d1);
					if (tt < 0.f)
						tt = 0.f;
					if (tt < 1.f) {
						const float landz = startz
							+ tt * (endz - startz);
						if (fabsf(pos.Z - landz) > 0.015625f)
							pos.Z = landz;
					}
				}
			}
			vel.Z = 0.f;
		}
		// CategorizePosition: state-only on flat ground (no snap);
		// CheckVelocity; FinishGravity half; grounded vz zero;
		// CheckVelocity (949-957)
		CapAir::KernelCheckVelocity(p, &vel.X, &vel.Y, &vel.Z);
		vel.Z -= 1.f * p.gravity * 0.5f * p.dt;
		vel.Z = 0.f;
		CapAir::KernelCheckVelocity(p, &vel.X, &vel.Y, &vel.Z);
		*npx = pos.X;
		*npy = pos.Y;
		*npz = pos.Z;
		*nvx = vel.X;
		*nvy = vel.Y;
		*nst = st;
	}
	// The JUMP tick (grounded, IN_JUMP accepted: fresh press, or the
	// autobunnyhopping bypass). ADD path only - the standing domain
	// (the SET path belongs to the A29 duck debt).
	inline void JumpKernelTick(const GroundCtx& g, float px, float py,
	                           float pz, float vx, float vy,
	                           float stamina, float yaw, float fm,
	                           float sm, float* npx, float* npy,
	                           float* npz, float* nvx, float* nvy,
	                           float* nvz, float* nst) {
		const MoveParams& p = g.p;
		Vec3 pos(px, py, pz);
		Vec3 vel(vx, vy, 0.f);
		float st = stamina;
		if (st > 0.f) {
			st -= p.dt * 1000.f;
			if (st < 0.f)
				st = 0.f;
		}
		vel.Z -= 1.f * p.gravity * 0.5f * p.dt;
		vel.Z += g.basevel.Z * p.dt;
		// CheckJumpButton, ADD path (295-379); enablebunnyhopping
		// true in the surf params so no 1.1x clamp
		const float startz = vel.Z;
		vel.Z = static_cast<float>(
			static_cast<double>(1.f) * p.jump_impulse_d
			+ static_cast<double>(startz));
		if (st > 0.f) {
			const float ratio = 1.f - st * p.stamina_scale_per_ms;
			vel.Z *= ratio;
		}
		st = p.stamina_jump_ms;
		KFinishGravityFn(p, 1.f, &vel);
		// now airborne: friction skipped; CheckVelocity (944); the
		// A4 air chain for the rest of the tick
		CapAir::KernelCheckVelocity(p, &vel.X, &vel.Y, &vel.Z);
		float wx, wy;
		Fn::WishFromInput(yaw, fm, sm, &wx, &wy);
		float wishspeed = sqrtf(wx * wx + wy * wy);
		Vec3 wishdir(0.f, 0.f, 0.f);
		if (wishspeed > 1e-6f)
			wishdir = Vec3(wx / wishspeed, wy / wishspeed, 0.f);
		if (wishspeed > p.maxspeed)
			wishspeed = p.maxspeed;
		const float wishspd = (wishspeed > p.air_speed_cap)
			? p.air_speed_cap : wishspeed;
		const float cur = Dot(vel, wishdir);
		const float add = wishspd - cur;
		if (add > 0.f) {
			float accelspeed = p.airaccelerate * wishspeed * p.dt
				* 1.f;
			if (accelspeed > add)
				accelspeed = add;
			vel = vel + Scale(wishdir, accelspeed);
		}
		const Vec3 bv(g.basevel.X, g.basevel.Y, 0.f);
		vel = vel + bv;
		if (Len2(vel) == 0.f) {
			vel = Vec3();
		} else {
			const Vec3 end = pos + Scale(vel, p.dt);
			pos = pos + Scale(end - pos, 1.f);
		}
		vel = vel - bv;
		// CategorizePosition: vz > non_jump_velocity skips the
		// ground probe (state-only); the airborne tail (950-957)
		CapAir::KernelCheckVelocity(p, &vel.X, &vel.Y, &vel.Z);
		vel.Z -= 1.f * p.gravity * 0.5f * p.dt;
		CapAir::KernelCheckVelocity(p, &vel.X, &vel.Y, &vel.Z);
		*npx = pos.X;
		*npy = pos.Y;
		*npz = pos.Z;
		*nvx = vel.X;
		*nvy = vel.Y;
		*nvz = vel.Z;
		*nst = st;
	}

} // namespace CapGround

// ======================================================================
// CAPWINDOW - registry B0 (packaged), B3, B4: the per-tick vertical
// evolution iterated EXACTLY (the engine's own float recurrence, two
// operations per tick - "closed form" real-math bracketing would need
// a float-error argument, and at ~180 flops for 90 ticks exact
// iteration is already free), intersected with the certified travel
// bound (B2). CAPEND - registry A27: the earliest tick a trajectory's
// feet point enters a Minkowski-expanded END box, with the z-window
// pruning the scan. Session 28.
// ======================================================================
namespace CapWindow {

	// the exact clean-air vertical recurrence for one tick, mirroring
	// MoveTick's order: StartGravity half -> position advances with
	// the mid velocity -> FinishGravity half (933/951; position uses
	// the post-AirMove vz, which carries only the first half)
	inline void VTick(const MoveParams& p, float* z, float* vz) {
		*vz -= 1.f * p.gravity * 0.5f * p.dt;
		if (*vz > p.maxvelocity)
			*vz = p.maxvelocity;
		else if (*vz < -p.maxvelocity)
			*vz = -p.maxvelocity;   // the engine's per-component clamp
		*z += *vz * p.dt;
		*vz -= 1.f * p.gravity * 0.5f * p.dt;
		if (*vz > p.maxvelocity)
			*vz = p.maxvelocity;
		else if (*vz < -p.maxvelocity)
			*vz = -p.maxvelocity;
	}
	// B0 packaged: the tick window where z lies in [zlo, zhi],
	// scanning at most n_max ticks with the natural early exit (once
	// z < zlo with vz < 0 the window is closed forever in clean air).
	// Returns [t_first, t_last] inclusive, or t_first = -1.
	inline void ZWindow(const MoveParams& p, float z0, float vz0,
	                    float zlo, float zhi, int n_max, int* t_first,
	                    int* t_last) {
		*t_first = -1;
		*t_last = -1;
		float z = z0, vz = vz0;
		for (int t = 0; t <= n_max; ++t) {
			if (z >= zlo && z <= zhi) {
				if (*t_first < 0)
					*t_first = t;
				*t_last = t;
			}
			if (z < zlo && vz < 0.f)
				return;   // never returns (B4's closure)
			VTick(p, &z, &vz);
		}
	}
	// B2's travel bound as prefix sums: reachable horizontal distance
	// after t ticks (the certified max-speed integral, session 24)
	inline void TravelPrefix(const MoveParams& p, float v0, int n_max,
	                         std::vector<float>* travel) {
		travel->assign(static_cast<size_t>(n_max) + 1, 0.f);
		const float cap2 = p.air_speed_cap * p.air_speed_cap;
		float s = 0.f;
		for (int i = 1; i <= n_max; ++i) {
			s += sqrtf(v0 * v0 + cap2 * static_cast<float>(i))
				* p.dt;
			(*travel)[static_cast<size_t>(i)] = s;
		}
	}
	// B3: the earliest tick a face at horizontal distance dist with
	// vertical extent [zlo, zhi] can possibly be contacted; B4: the
	// last useful tick. Both CERTIFIED: the z window is the exact
	// recurrence, the distance test is the never-beaten travel bound.
	inline void ContactWindow(const MoveParams& p, float z0, float vz0,
	                          float v0, float dist, float zlo,
	                          float zhi, int n_max, int* t_earliest,
	                          int* t_latest) {
		int zf, zl;
		ZWindow(p, z0, vz0, zlo, zhi, n_max, &zf, &zl);
		*t_earliest = -1;
		*t_latest = -1;
		if (zf < 0)
			return;
		std::vector<float> travel;
		TravelPrefix(p, v0, n_max, &travel);
		for (int t = zf; t <= zl; ++t)
			if (travel[static_cast<size_t>(t)] + 0.001f >= dist) {
				*t_earliest = t;
				break;
			}
		if (*t_earliest >= 0)
			*t_latest = zl;
	}

} // namespace CapWindow

namespace CapEnd {

	struct Box {
		float xlo = 0.f, xhi = 0.f;
		float ylo = 0.f, yhi = 0.f;
		float zlo = 0.f, zhi = 0.f;
	};
	// A27: earliest tick the feet point of an exact kernel rollout
	// enters the (already Minkowski-expanded) box; -1 if never. The
	// z-window from the exact vertical recurrence prunes the scan -
	// only ticks inside the window run the horizontal test.
	inline int EarliestBoxCrossing(const CapAir::AirKernelCtx& k,
	                               float px, float py, float pz,
	                               float vx, float vy, float vz,
	                               const signed char* side,
	                               const float* cosa, int n,
	                               const Box& box,
	                               long long* ticks_tested) {
		int zf, zl;
		CapWindow::ZWindow(k.p, pz, vz, box.zlo, box.zhi, n, &zf,
			&zl);
		if (ticks_tested)
			*ticks_tested = 0;
		if (zf < 0)
			return -1;
		float x = px, y = py, z = pz;
		float cvx = vx, cvy = vy, cvz = vz;
		// tick 0 is the start state itself
		if (zf == 0 && x >= box.xlo && x <= box.xhi && y >= box.ylo
			&& y <= box.yhi && z >= box.zlo && z <= box.zhi) {
			if (ticks_tested)
				*ticks_tested = 1;
			return 0;
		}
		for (int t = 0; t < n && t < zl; ++t) {
			float nx, ny, nz2, nvx, nvy, nvz;
			CapAir::KernelTick(k, x, y, z, cvx, cvy, cvz, side[t],
				cosa[t], &nx, &ny, &nz2, &nvx, &nvy, &nvz);
			x = nx;
			y = ny;
			z = nz2;
			cvx = nvx;
			cvy = nvy;
			cvz = nvz;
			const int tt = t + 1;
			if (tt < zf)
				continue;   // outside the vertical window
			if (ticks_tested)
				(*ticks_tested)++;
			if (x >= box.xlo && x <= box.xhi && y >= box.ylo
				&& y <= box.yhi && z >= box.zlo && z <= box.zhi)
				return tt;
		}
		return -1;
	}

} // namespace CapEnd

// ======================================================================
// CAPRIDE - registry A19: the N-tick RIDE turn/gain maximum
// V*_ride(S0, N, dpsi | ramp normal), session 28. Built on the PROVEN
// A18 one-tick ride law (1246/1246 bitwise) with surface_friction as
// composed carried state (the vz-at-categorize rule) and an explicit
// STAY-ON-FACE domain guard: a transition whose pre-clip velocity
// points out of the face (v.n >= 0) EXITS the ride domain (that event
// is A23/A26 material, not a ride transition). The DP starts from an
// exact ENGINE-REACHED state (captured after a real contact tick), so
// witness replay is a pure continuation of a real run - the
// composition gate is bitwise. Representative per cell: max 3-D speed
// (stated LB semantics; the falsifier attacks it, M-style).
// ======================================================================
namespace CapRide {

	struct RideParams {
		int n_max = 60;
		int psi_bins = 721;
		int vb_max = 640;        // 10 u/s strata up to 6400 (3-D)
		float v_bin = 10.f;
	};
	struct RNodeR {
		float vx = 0.f, vy = 0.f, vz = 0.f;
		int parent = -1;
		int key = -1;
		short action = -1;
	};
	struct RideSurface {
		RideParams p;
		Vec3 n;                   // the ramp normal
		Vec3 v0;                  // the exact engine start velocity
		float sf0 = 1.f;
		int slots = 0;
		std::vector<std::vector<RNodeR> > layers;
		std::vector<std::vector<float> > pb_vmax;   // [layer][psi bin]
		std::vector<std::vector<int> >   pb_node;
		long long ride_ticks = 0;
		long long leave_transitions = 0;   // domain exits (measured)
		long long total_nodes = 0;
		long long node_budget = 60000000;
		int peak_layer = 0;
		bool aborted = false;
		double build_ms = 0.0;
		std::vector<signed char> act_side;
		std::vector<float> act_cosa;
	};
	inline int KeyR(const RideSurface& S, int pb, int vb, int sidx,
	                int age, int sfbit) {
		const int a = age < 1 ? 1 : (age > 6 ? 6 : age);
		return ((((pb * S.p.vb_max) + vb) * 3 + sidx) * 6 + (a - 1))
			* 2 + sfbit;
	}
	inline int PsiBinR(const RideSurface& S, float psi) {
		const float u = (psi + 3.14159265f) / 6.2831853f;
		int b = static_cast<int>(u * static_cast<float>(
			S.p.psi_bins));
		if (b < 0) b = 0;
		if (b >= S.p.psi_bins) b = S.p.psi_bins - 1;
		return b;
	}
	inline float BinPsiR(const RideSurface& S, int bin) {
		return (static_cast<float>(bin) + 0.5f)
			/ static_cast<float>(S.p.psi_bins) * 6.2831853f
			- 3.14159265f;
	}
	inline void BuildRide(const RideParams& pp, const MoveParams& p,
	                      const Vec3& n, const Vec3& v0, float sf0,
	                      RideSurface* S) {
		S->p = pp;
		S->n = n;
		S->v0 = v0;
		S->sf0 = sf0;
		S->slots = pp.psi_bins * pp.vb_max * 3 * 6 * 2;
		S->layers.assign(static_cast<size_t>(pp.n_max) + 1,
			std::vector<RNodeR>());
		S->pb_vmax.assign(static_cast<size_t>(pp.n_max) + 1,
			std::vector<float>());
		S->pb_node.assign(static_cast<size_t>(pp.n_max) + 1,
			std::vector<int>());
		static const float kCosa[21] = {
			1.f, 0.9995f, 0.998f, 0.995f, 0.99f, 0.98f, 0.96f,
			0.93f, 0.9f, 0.85f, 0.8f, 0.7f, 0.6f, 0.45f, 0.3f,
			0.15f, 0.f, -0.2f, -0.5f, -0.8f, -1.f };
		S->act_side.clear();
		S->act_cosa.clear();
		S->act_side.push_back(0);
		S->act_cosa.push_back(1.f);
		for (int sd = 0; sd < 2; ++sd)
			for (int ci = 0; ci < 21; ++ci) {
				S->act_side.push_back(sd == 0
					? static_cast<signed char>(1)
					: static_cast<signed char>(-1));
				S->act_cosa.push_back(kCosa[ci]);
			}
		const int n_act = static_cast<int>(S->act_side.size());
		const int min_gap = static_cast<int>(
			ceilf((1.f / p.dt) / p.strafe_rate_max));
		const auto t0 = std::chrono::steady_clock::now();
		std::vector<int> slot(static_cast<size_t>(S->slots), -1);
		{
			RNodeR c;
			c.vx = v0.X;
			c.vy = v0.Y;
			c.vz = v0.Z;
			const float sp2d = Len2D(v0);
			const float psi = sp2d > 1.f ? atan2f(v0.Y, v0.X) : 0.f;
			const float sp = Len(v0);
			int vb = static_cast<int>(sp / pp.v_bin);
			if (vb >= pp.vb_max) vb = pp.vb_max - 1;
			const int sfbit = sf0 < 0.5f ? 1 : 0;
			c.key = KeyR(*S, PsiBinR(*S, psi), vb, 1, 6, sfbit);
			S->layers[0].push_back(c);
			S->total_nodes = 1;
		}
		const Vec3 bv0;
		for (int t = 0; t < pp.n_max; ++t) {
			const std::vector<RNodeR>& curL = S->layers[
				static_cast<size_t>(t)];
			std::vector<RNodeR>& nxtL = S->layers[
				static_cast<size_t>(t) + 1];
			std::fill(slot.begin(), slot.end(), -1);
			for (size_t ni = 0; ni < curL.size(); ++ni) {
				const RNodeR cell = curL[ni];
				const int sfbit = cell.key % 2;
				const int rem = cell.key / 2;
				const int side_code = (rem / 6) % 3;
				const signed char cside = side_code == 0
					? static_cast<signed char>(-1)
					: (side_code == 1
						? static_cast<signed char>(0)
						: static_cast<signed char>(1));
				const int cage = (rem % 6) + 1;
				const float sf = sfbit ? p.air_friction_up : 1.f;
				for (int ai = 0; ai < n_act; ++ai) {
					const signed char ds = S->act_side[
						static_cast<size_t>(ai)];
					const float ca = S->act_cosa[
						static_cast<size_t>(ai)];
					if (ds != 0 && cside != 0 && ds != cside
						&& cage < min_gap)
						continue;
					Vec3 out;
					float vdn = 0.f, vzc = 0.f;
					CapBoard::RideGrayTick(p, n, sf, bv0,
						Vec3(cell.vx, cell.vy, cell.vz), ds, ca,
						&out, &vdn, &vzc);
					++S->ride_ticks;
					// STAY-ON-FACE: the rider hovers one trace
					// epsilon (1/32) off the plane, so a tick only
					// re-contacts when the inward motion closes
					// that gap within dt: v.n * dt <= -1/32, i.e.
					// v.n <= -2.083 u/s at 66.67 tps (measured:
					// separation observed at v.n = -1.80)
					if (vdn * p.dt > -0.03125f) {
						++S->leave_transitions;   // hover or exit
						continue;
					}
					const float sp2d = Len2D(out);
					const float npsi = sp2d > 1.f
						? atan2f(out.Y, out.X) : 0.f;
					const float nsp = Len(out);
					const signed char nside = ds != 0 ? ds
						: cside;
					const int nage = (ds != 0 && cside != 0
						&& ds != cside) ? 1
						: (cage < 6 ? cage + 1 : 6);
					const int nsidx = nside < 0 ? 0
						: (nside == 0 ? 1 : 2);
					int vb = static_cast<int>(nsp / pp.v_bin);
					if (vb >= pp.vb_max) vb = pp.vb_max - 1;
					const int nsf = vzc > 0.f ? 1 : 0;
					const int nk = KeyR(*S, PsiBinR(*S, npsi), vb,
						nsidx, nage, nsf);
					const int have = slot[static_cast<size_t>(nk)];
					if (have < 0) {
						RNodeR nc;
						nc.vx = out.X;
						nc.vy = out.Y;
						nc.vz = out.Z;
						nc.parent = static_cast<int>(ni);
						nc.key = nk;
						nc.action = static_cast<short>(ai);
						slot[static_cast<size_t>(nk)] =
							static_cast<int>(nxtL.size());
						nxtL.push_back(nc);
					} else {
						RNodeR& dst = nxtL[static_cast<size_t>(
							have)];
						const float dsp = sqrtf(dst.vx * dst.vx
							+ dst.vy * dst.vy
							+ dst.vz * dst.vz);
						if (nsp > dsp) {
							dst.vx = out.X;
							dst.vy = out.Y;
							dst.vz = out.Z;
							dst.parent = static_cast<int>(ni);
							dst.action = static_cast<short>(ai);
						}
					}
				}
			}
			if (static_cast<int>(nxtL.size()) > S->peak_layer)
				S->peak_layer = static_cast<int>(nxtL.size());
			S->total_nodes += static_cast<long long>(nxtL.size());
			if (S->total_nodes > S->node_budget) {
				S->aborted = true;
				break;
			}
		}
		for (size_t t = 0; t < S->layers.size(); ++t) {
			S->pb_vmax[t].assign(static_cast<size_t>(pp.psi_bins),
				-1.f);
			S->pb_node[t].assign(static_cast<size_t>(pp.psi_bins),
				-1);
			const std::vector<RNodeR>& L = S->layers[t];
			for (size_t ni = 0; ni < L.size(); ++ni) {
				const int pb = L[ni].key / (pp.vb_max * 36);
				const float sp = sqrtf(L[ni].vx * L[ni].vx
					+ L[ni].vy * L[ni].vy + L[ni].vz * L[ni].vz);
				if (sp > S->pb_vmax[t][static_cast<size_t>(pb)]) {
					S->pb_vmax[t][static_cast<size_t>(pb)] = sp;
					S->pb_node[t][static_cast<size_t>(pb)] =
						static_cast<int>(ni);
				}
			}
		}
		S->build_ms = std::chrono::duration<double, std::milli>(
			std::chrono::steady_clock::now() - t0).count();
	}
	inline float QueryRide(const RideSurface& S, int N, float dpsi,
	                       int* node_out = nullptr) {
		if (N < 0 || N > S.p.n_max)
			return -1.f;
		const int pb = PsiBinR(S, dpsi);
		const float v = S.pb_vmax[static_cast<size_t>(N)][
			static_cast<size_t>(pb)];
		if (v >= 0.f && node_out)
			*node_out = S.pb_node[static_cast<size_t>(N)][
				static_cast<size_t>(pb)];
		return v;
	}
	inline void WitnessRide(const RideSurface& S, int N, int node,
	                        std::vector<signed char>* side,
	                        std::vector<float>* cosa) {
		side->assign(static_cast<size_t>(N), 0);
		cosa->assign(static_cast<size_t>(N), 1.f);
		int ni = node;
		for (int t = N; t >= 1; --t) {
			const RNodeR& c = S.layers[static_cast<size_t>(t)][
				static_cast<size_t>(ni)];
			(*side)[static_cast<size_t>(t) - 1] = c.action >= 0
				? S.act_side[static_cast<size_t>(c.action)] : 0;
			(*cosa)[static_cast<size_t>(t) - 1] = c.action >= 0
				? S.act_cosa[static_cast<size_t>(c.action)] : 1.f;
			ni = c.parent;
			if (ni < 0)
				break;
		}
	}

} // namespace CapRide

// ======================================================================
// CAPRIDEREACH - registry A20 (ride directional reach) and the v1 of
// A22 (board->exit target queries), session 29. A forward sweep on
// the proven A18 ride law with POSITIONS (the position law measured
// bitwise-or-3.5e-5u on steady ticks - far below lattice pitch),
// from an exact engine-reached start, with the contact-epsilon
// stay-on-face guard and composed surface friction. Positions are
// binned in the ramp's in-plane basis (downslope, cross-slope).
// D*_ride is served as a witnessed lower bound at 8 in-plane
// directions; A22 v1 answers a target point at tick N by cell-witness
// lookup at lattice resolution (the exact-hit refinement inherits the
// A11 tail machinery later).
// ======================================================================
namespace CapRideReach {

	struct RRParams {
		// the ride-appropriate pitch (session 29: the air-derived
		// pitch of 32u/5deg/25u-s hit 4.2M cells per layer and
		// aborted by layer ~14 - ride speeds fan positions much
		// faster than the air sweep's targeted flights)
		int n_max = 48;
		float hp = 64.f;         // in-plane position pitch
		int   pos_bins = 150;    // +/-4800 over hp
		int   psi_bins = 36;     // horizontal-heading bins (10 deg)
		int   vb_max = 128;      // 50 u/s strata (3-D speed)
		float v_bin = 50.f;
	};
	struct RRNode {
		float px = 0.f, py = 0.f, pz = 0.f;
		float vx = 0.f, vy = 0.f, vz = 0.f;
		int parent = -1;
		short action = -1;
		unsigned char sa = 0;    // side_idx * 8 + age
		unsigned char sf = 0;    // 1 = quarter-strength (rising)
	};
	struct RRSurface {
		RRParams p;
		Vec3 n, dsl, csl;        // normal, downslope, cross-slope
		Vec3 pos0, v0;
		float sf0 = 1.f;
		std::vector<std::vector<RRNode> > layers;
		std::vector<std::vector<float> > dslb;    // [layer][8]
		std::vector<std::vector<int> >   dsnode;  // [layer][8]
		// the constant-time query index (session 30): per layer,
		// in-plane cell -> node (max-speed representative)
		std::vector<std::unordered_map<int, int> > cell_index;
		long long ride_ticks = 0;
		long long leave_transitions = 0;
		long long total_nodes = 0;
		long long node_budget = 40000000;
		int peak_layer = 0;
		bool aborted = false;
		double build_ms = 0.0;
		std::vector<signed char> act_side;
		std::vector<float> act_cosa;
	};
	inline long long RRKey(const RRParams& pp, float u, float v,
	                       float psi, float sp, int sidx, int age,
	                       int sfbit) {
		int ia = static_cast<int>((u + 4800.f) / pp.hp);
		int ib = static_cast<int>((v + 4800.f) / pp.hp);
		if (ia < 0) ia = 0;
		if (ia >= pp.pos_bins) ia = pp.pos_bins - 1;
		if (ib < 0) ib = 0;
		if (ib >= pp.pos_bins) ib = pp.pos_bins - 1;
		const float uu = (psi + 3.14159265f) / 6.2831853f;
		int ip = static_cast<int>(uu * static_cast<float>(
			pp.psi_bins));
		if (ip < 0) ip = 0;
		if (ip >= pp.psi_bins) ip = pp.psi_bins - 1;
		int iv = static_cast<int>(sp / pp.v_bin);
		if (iv >= pp.vb_max) iv = pp.vb_max - 1;
		const int a = age < 1 ? 1 : (age > 6 ? 6 : age);
		return ((((((static_cast<long long>(ia) * pp.pos_bins + ib)
			* pp.psi_bins + ip) * pp.vb_max + iv) * 3 + sidx) * 6
			+ (a - 1)) * 2) + sfbit;
	}
	inline void BuildRideReach(const RRParams& pp, const MoveParams& p,
	                           const Vec3& n, const Vec3& pos0,
	                           const Vec3& v0, float sf0,
	                           RRSurface* S) {
		S->p = pp;
		S->n = n;
		S->pos0 = pos0;
		S->v0 = v0;
		S->sf0 = sf0;
		// the in-plane basis
		Vec3 dsl(n.Z * n.X, n.Z * n.Y, n.Z * n.Z - 1.f);
		const float dl = Len(dsl);
		S->dsl = Scale(dsl, 1.f / dl);
		S->csl = Cross(n, S->dsl);
		S->layers.assign(static_cast<size_t>(pp.n_max) + 1,
			std::vector<RRNode>());
		S->dslb.assign(static_cast<size_t>(pp.n_max) + 1,
			std::vector<float>());
		S->dsnode.assign(static_cast<size_t>(pp.n_max) + 1,
			std::vector<int>());
		static const float kCosa[21] = {
			1.f, 0.9995f, 0.998f, 0.995f, 0.99f, 0.98f, 0.96f,
			0.93f, 0.9f, 0.85f, 0.8f, 0.7f, 0.6f, 0.45f, 0.3f,
			0.15f, 0.f, -0.2f, -0.5f, -0.8f, -1.f };
		S->act_side.clear();
		S->act_cosa.clear();
		S->act_side.push_back(0);
		S->act_cosa.push_back(1.f);
		for (int sd = 0; sd < 2; ++sd)
			for (int ci = 0; ci < 21; ++ci) {
				S->act_side.push_back(sd == 0
					? static_cast<signed char>(1)
					: static_cast<signed char>(-1));
				S->act_cosa.push_back(kCosa[ci]);
			}
		const int n_act = static_cast<int>(S->act_side.size());
		const int min_gap = static_cast<int>(
			ceilf((1.f / p.dt) / p.strafe_rate_max));
		const auto t0 = std::chrono::steady_clock::now();
		{
			RRNode c;
			c.px = pos0.X;
			c.py = pos0.Y;
			c.pz = pos0.Z;
			c.vx = v0.X;
			c.vy = v0.Y;
			c.vz = v0.Z;
			c.sa = static_cast<unsigned char>(1 * 8 + 6);
			c.sf = sf0 < 0.5f ? 1 : 0;
			S->layers[0].push_back(c);
			S->total_nodes = 1;
		}
		const Vec3 bv0;
		std::unordered_map<long long, int> slot;
		for (int t = 0; t < pp.n_max; ++t) {
			const std::vector<RRNode>& curL = S->layers[
				static_cast<size_t>(t)];
			std::vector<RRNode>& nxtL = S->layers[
				static_cast<size_t>(t) + 1];
			slot.clear();
			slot.reserve(curL.size() * 8 + 64);
			for (size_t ni = 0; ni < curL.size(); ++ni) {
				const RRNode cell = curL[ni];
				const int sidx = cell.sa / 8;
				const signed char cside = sidx == 0
					? static_cast<signed char>(-1)
					: (sidx == 1 ? static_cast<signed char>(0)
						: static_cast<signed char>(1));
				const int cage = cell.sa % 8;
				const float sf = cell.sf ? p.air_friction_up
					: 1.f;
				for (int ai = 0; ai < n_act; ++ai) {
					const signed char ds = S->act_side[
						static_cast<size_t>(ai)];
					const float ca = S->act_cosa[
						static_cast<size_t>(ai)];
					if (ds != 0 && cside != 0 && ds != cside
						&& cage < min_gap)
						continue;
					Vec3 out, npos;
					float vdn = 0.f, vzc = 0.f;
					const Vec3 pin(cell.px, cell.py, cell.pz);
					CapBoard::RideGrayTick(p, n, sf, bv0,
						Vec3(cell.vx, cell.vy, cell.vz), ds, ca,
						&out, &vdn, &vzc, &pin, &npos);
					++S->ride_ticks;
					// the contact-epsilon guard with a drift
					// margin: reach extremals press on every
					// boundary, and the carried-gap micro-drift
					// (measured 3.5e-5 u/tick) makes exact-
					// boundary transitions hover in the engine -
					// the margin trades a hair of lower bound for
					// a sound domain (the exact carried-gap law
					// is the recorded refinement)
					if (vdn * p.dt > -0.03525f) {
						++S->leave_transitions;
						continue;
					}
					const float rx = npos.X - pos0.X;
					const float ry = npos.Y - pos0.Y;
					const float rz = npos.Z - pos0.Z;
					const float u = rx * S->dsl.X + ry * S->dsl.Y
						+ rz * S->dsl.Z;
					const float v = rx * S->csl.X + ry * S->csl.Y
						+ rz * S->csl.Z;
					const float sp2d = Len2D(out);
					const float npsi = sp2d > 1.f
						? atan2f(out.Y, out.X) : 0.f;
					const float nsp = Len(out);
					const signed char nside = ds != 0 ? ds
						: cside;
					const int nage = (ds != 0 && cside != 0
						&& ds != cside) ? 1
						: (cage < 6 ? cage + 1 : 6);
					const int nsidx = nside < 0 ? 0
						: (nside == 0 ? 1 : 2);
					const int nsf = vzc > 0.f ? 1 : 0;
					const long long nk = RRKey(pp, u, v, npsi,
						nsp, nsidx, nage, nsf);
					std::unordered_map<long long, int>::iterator
						jt = slot.find(nk);
					if (jt == slot.end()) {
						RRNode nc;
						nc.px = npos.X;
						nc.py = npos.Y;
						nc.pz = npos.Z;
						nc.vx = out.X;
						nc.vy = out.Y;
						nc.vz = out.Z;
						nc.parent = static_cast<int>(ni);
						nc.action = static_cast<short>(ai);
						nc.sa = static_cast<unsigned char>(
							nsidx * 8 + nage);
						nc.sf = static_cast<unsigned char>(nsf);
						slot[nk] = static_cast<int>(nxtL.size());
						nxtL.push_back(nc);
					} else {
						RRNode& dst = nxtL[static_cast<size_t>(
							jt->second)];
						const float dsp = sqrtf(dst.vx * dst.vx
							+ dst.vy * dst.vy
							+ dst.vz * dst.vz);
						if (nsp > dsp) {
							dst.px = npos.X;
							dst.py = npos.Y;
							dst.pz = npos.Z;
							dst.vx = out.X;
							dst.vy = out.Y;
							dst.vz = out.Z;
							dst.parent = static_cast<int>(ni);
							dst.action = static_cast<short>(ai);
							dst.sa = static_cast<unsigned char>(
								nsidx * 8 + nage);
							dst.sf = static_cast<unsigned char>(
								nsf);
						}
					}
				}
			}
			if (static_cast<int>(nxtL.size()) > S->peak_layer)
				S->peak_layer = static_cast<int>(nxtL.size());
			S->total_nodes += static_cast<long long>(nxtL.size());
			if (S->total_nodes > S->node_budget) {
				S->aborted = true;
				break;
			}
		}
		// D*_ride lower bounds at 8 in-plane directions
		for (size_t t = 0; t < S->layers.size(); ++t) {
			S->dslb[t].assign(8, -1e30f);
			S->dsnode[t].assign(8, -1);
			const std::vector<RRNode>& L = S->layers[t];
			for (size_t ni = 0; ni < L.size(); ++ni) {
				const float rx = L[ni].px - S->pos0.X;
				const float ry = L[ni].py - S->pos0.Y;
				const float rz = L[ni].pz - S->pos0.Z;
				const float u = rx * S->dsl.X + ry * S->dsl.Y
					+ rz * S->dsl.Z;
				const float v = rx * S->csl.X + ry * S->csl.Y
					+ rz * S->csl.Z;
				for (int ph = 0; ph < 8; ++ph) {
					const float phi = static_cast<float>(ph)
						* 0.78539816f;
					const float pr = u * cosf(phi)
						+ v * sinf(phi);
					if (pr > S->dslb[t][static_cast<size_t>(
						ph)]) {
						S->dslb[t][static_cast<size_t>(ph)] = pr;
						S->dsnode[t][static_cast<size_t>(ph)] =
							static_cast<int>(ni);
					}
				}
			}
		}
		// the constant-time query index: in-plane cell -> max-speed
		// node, per layer (organize once, query O(1))
		S->cell_index.assign(S->layers.size(),
			std::unordered_map<int, int>());
		for (size_t t = 0; t < S->layers.size(); ++t) {
			const std::vector<RRNode>& L = S->layers[t];
			std::unordered_map<int, int>& ix = S->cell_index[t];
			ix.reserve(L.size() * 2 + 16);
			for (size_t ni = 0; ni < L.size(); ++ni) {
				const float rx = L[ni].px - S->pos0.X;
				const float ry = L[ni].py - S->pos0.Y;
				const float rz = L[ni].pz - S->pos0.Z;
				const float u = rx * S->dsl.X + ry * S->dsl.Y
					+ rz * S->dsl.Z;
				const float v = rx * S->csl.X + ry * S->csl.Y
					+ rz * S->csl.Z;
				int ia = static_cast<int>((u + 4800.f) / pp.hp);
				int ib = static_cast<int>((v + 4800.f) / pp.hp);
				if (ia < 0) ia = 0;
				if (ia >= pp.pos_bins) ia = pp.pos_bins - 1;
				if (ib < 0) ib = 0;
				if (ib >= pp.pos_bins) ib = pp.pos_bins - 1;
				const int cid = ia * pp.pos_bins + ib;
				std::unordered_map<int, int>::iterator jt =
					ix.find(cid);
				if (jt == ix.end()) {
					ix[cid] = static_cast<int>(ni);
				} else {
					const RRNode& a = L[static_cast<size_t>(
						jt->second)];
					const RRNode& b = L[ni];
					const float sa = a.vx * a.vx + a.vy * a.vy
						+ a.vz * a.vz;
					const float sb = b.vx * b.vx + b.vy * b.vy
						+ b.vz * b.vz;
					if (sb > sa)
						jt->second = static_cast<int>(ni);
				}
			}
		}
		S->build_ms = std::chrono::duration<double, std::milli>(
			std::chrono::steady_clock::now() - t0).count();
	}
	inline int QueryTarget(const RRSurface& S, int N, float tu,
	                       float tv, float* res_out);
	// A22 v2: constant-time target query - the 3x3 cell neighborhood
	// around the target (expanding to 5x5, then the full scan as the
	// last resort). Returns the node index; res_out = in-plane
	// residual.
	inline int QueryTargetFast(const RRSurface& S, int N, float tu,
	                           float tv, float* res_out) {
		if (N < 0 || N > S.p.n_max)
			return -1;
		const std::vector<RRNode>& L = S.layers[
			static_cast<size_t>(N)];
		const std::unordered_map<int, int>& ix = S.cell_index[
			static_cast<size_t>(N)];
		int ia = static_cast<int>((tu + 4800.f) / S.p.hp);
		int ib = static_cast<int>((tv + 4800.f) / S.p.hp);
		int best = -1;
		float bres = 1e30f;
		for (int ring = 1; ring <= 2 && best < 0; ++ring) {
			for (int da = -ring; da <= ring; ++da)
				for (int db = -ring; db <= ring; ++db) {
					const int ca = ia + da, cb = ib + db;
					if (ca < 0 || ca >= S.p.pos_bins || cb < 0
						|| cb >= S.p.pos_bins)
						continue;
					std::unordered_map<int, int>::const_iterator
						jt = ix.find(ca * S.p.pos_bins + cb);
					if (jt == ix.end())
						continue;
					const RRNode& nd = L[static_cast<size_t>(
						jt->second)];
					const float rx = nd.px - S.pos0.X;
					const float ry = nd.py - S.pos0.Y;
					const float rz = nd.pz - S.pos0.Z;
					const float u = rx * S.dsl.X + ry * S.dsl.Y
						+ rz * S.dsl.Z;
					const float v = rx * S.csl.X + ry * S.csl.Y
						+ rz * S.csl.Z;
					const float du = u - tu, dv = v - tv;
					const float r = sqrtf(du * du + dv * dv);
					if (r < bres) {
						bres = r;
						best = jt->second;
					}
				}
		}
		if (best >= 0) {
			if (res_out)
				*res_out = bres;
			return best;
		}
		return QueryTarget(S, N, tu, tv, res_out);
	}
	// A22 v1: the nearest swept node to an in-plane target at tick N
	// (lattice-resolution point solve; witness by parent chain)
	inline int QueryTarget(const RRSurface& S, int N, float tu,
	                       float tv, float* res_out) {
		if (N < 0 || N > S.p.n_max)
			return -1;
		const std::vector<RRNode>& L = S.layers[
			static_cast<size_t>(N)];
		int best = -1;
		float bres = 1e30f;
		for (size_t ni = 0; ni < L.size(); ++ni) {
			const float rx = L[ni].px - S.pos0.X;
			const float ry = L[ni].py - S.pos0.Y;
			const float rz = L[ni].pz - S.pos0.Z;
			const float u = rx * S.dsl.X + ry * S.dsl.Y
				+ rz * S.dsl.Z;
			const float v = rx * S.csl.X + ry * S.csl.Y
				+ rz * S.csl.Z;
			const float du = u - tu, dv = v - tv;
			const float r = sqrtf(du * du + dv * dv);
			if (r < bres) {
				bres = r;
				best = static_cast<int>(ni);
			}
		}
		if (res_out)
			*res_out = bres;
		return best;
	}
	inline void WitnessRR(const RRSurface& S, int N, int node,
	                      std::vector<signed char>* side,
	                      std::vector<float>* cosa) {
		side->assign(static_cast<size_t>(N), 0);
		cosa->assign(static_cast<size_t>(N), 1.f);
		int ni = node;
		for (int t = N; t >= 1; --t) {
			const RRNode& c = S.layers[static_cast<size_t>(t)][
				static_cast<size_t>(ni)];
			(*side)[static_cast<size_t>(t) - 1] = c.action >= 0
				? S.act_side[static_cast<size_t>(c.action)] : 0;
			(*cosa)[static_cast<size_t>(t) - 1] = c.action >= 0
				? S.act_cosa[static_cast<size_t>(c.action)] : 1.f;
			ni = c.parent;
			if (ni < 0)
				break;
		}
	}

} // namespace CapRideReach

// ======================================================================
// CAPBOUNDS - registry B13 (minimum departure resource for an air
// gap) and B14 (successor's minimum incoming resource), session 30.
// Constant-time certified NECESSARY conditions built by inverting the
// closed-form engine truths: the speed ceiling sqrt(v0^2 + cap^2 N)
// (tight, session 24) and the max-speed travel integral (never
// beaten, session 24), both monotone in the departure speed - so the
// minimum resource is a closed form plus at most a short bisection.
// Semantics: BELOW the bound the requirement is IMPOSSIBLE with a
// proof; above it nothing is promised (these are culls, not
// constructions). Falsifiers: the A11 solver attacking from just
// below the bound (a found schedule would REFUTE the proof), and
// exact-clip sampling for the board composition.
// ======================================================================
namespace CapBounds {

	// travel bound after N ticks from speed v0 (B2's integral)
	inline float TravelN(const MoveParams& p, float v0, int N) {
		const float cap2 = p.air_speed_cap * p.air_speed_cap;
		float s = 0.f;
		for (int i = 1; i <= N; ++i)
			s += sqrtf(v0 * v0 + cap2 * static_cast<float>(i))
				* p.dt;
		return s;
	}
	// B13: the minimum departure speed such that BOTH necessary
	// conditions hold for "cover horizontal distance D within N ticks
	// and arrive with speed >= V": the ceiling gives a closed form,
	// the travel bound a bisection (monotone in v0). Returns 0 when
	// nothing is required.
	inline float MinDepartSpeed(const MoveParams& p, float D, float V,
	                            int N) {
		const float cap2 = p.air_speed_cap * p.air_speed_cap;
		// ceiling: v0 >= sqrt(max(0, V^2 - cap^2 N))
		float v_ceiling = 0.f;
		const float q = V * V - cap2 * static_cast<float>(N);
		if (q > 0.f)
			v_ceiling = sqrtf(q);
		// travel: smallest v0 with TravelN(v0) >= D
		float v_travel = 0.f;
		if (TravelN(p, 0.f, N) < D) {
			float lo = 0.f, hi = 4000.f;
			if (TravelN(p, hi, N) < D) {
				v_travel = 1e30f;   // unreachable at any speed
			} else {
				for (int it = 0; it < 40; ++it) {
					const float mid = 0.5f * (lo + hi);
					if (TravelN(p, mid, N) >= D)
						hi = mid;
					else
						lo = mid;
				}
				v_travel = hi;
			}
		}
		return v_ceiling > v_travel ? v_ceiling : v_travel;
	}
	// B14: the minimum INCOMING speed at a face (normal n, fixed
	// arrival vz, arrival heading restricted to [t0, t1]) such that
	// the post-board speed can possibly reach v_need. Post-board
	// speed^2 = s_in^2 + vz^2... the horizontal speed s_h relates by
	// s_in^2 = s_h^2 + vz^2; post^2 = s_in^2 - (v.n)^2 with |v.n|
	// minimized over the window (A16's closed form). Monotone in s_h
	// on the physical range - bisection, with the monotonicity
	// verified across the bracket by sampling (falls back to a scan
	// if violated).
	inline float BestPost2(const MoveParams& /*p*/, float s_h,
	                       float vz, const Vec3& n, float t0,
	                       float t1) {
		CapBoard::BoardOpt bo, wo;
		CapBoard::BestWorstBoard(s_h, vz, n, t0, t1, &bo, &wo);
		if (!bo.boardable)
			return -1.f;   // nothing in the window boards
		const float s2 = s_h * s_h + vz * vz;
		const float loss = bo.vdotn;   // <= 0, nearest zero
		return s2 - loss * loss;
	}
	// The EXACT infimum (session 30, after the falsifier refuted the
	// monotone-bisection version with 20/127 beats - the best
	// post-board speed is NOT monotone in horizontal speed; a slow-
	// horizontal fast-vertical arrival can retain more). On each
	// branch of the window's minimum-loss formula the feasibility
	// condition "s_h^2 + vz^2 - L(s_h)^2 >= need^2" is a QUADRATIC in
	// s_h, so the infimum is closed-form: collect the branch
	// boundaries and quadratic roots as candidates, classify the
	// intervals exactly, take the first feasible point. Domain: vz <=
	// 0 arrivals (the boarding case). The returned bound is shaved by
	// 0.05 u/s - a smaller necessary bound is the SOUND direction.
	inline float MinIncomingSpeed(const MoveParams& /*p*/,
	                              float v_need, float vz,
	                              const Vec3& n, float t0, float t1) {
		const double need2 = static_cast<double>(v_need) * v_need;
		const double vz2 = static_cast<double>(vz) * vz;
		const double nh = sqrt(static_cast<double>(n.X) * n.X
			+ static_cast<double>(n.Y) * n.Y);
		const double B = static_cast<double>(vz) * n.Z;   // <= 0
		const double phi = atan2(static_cast<double>(n.Y),
			static_cast<double>(n.X));
		// the window's cos(theta - phi) range [c_lo, c_hi]
		double u0 = static_cast<double>(t0) - phi;
		double u1 = static_cast<double>(t1) - phi;
		// wrap the interval start into (-pi, pi]
		const double kTau = 6.283185307179586;
		while (u0 > 3.141592653589793) { u0 -= kTau; u1 -= kTau; }
		while (u0 <= -3.141592653589793) { u0 += kTau; u1 += kTau; }
		double c_hi, c_lo;
		{
			const double ca = cos(u0), cb = cos(u1);
			c_hi = (u0 <= 0.0 && u1 >= 0.0) ? 1.0
				: (ca > cb ? ca : cb);
			const bool has_pi = (u0 <= 3.141592653589793
				&& u1 >= 3.141592653589793)
				|| (u0 <= -3.141592653589793);
			c_lo = has_pi ? -1.0 : (ca < cb ? ca : cb);
		}
		// exact minimum loss at horizontal speed s (A = s*nh):
		//   c* = clamp(-B/A, [c_lo, c_hi]); L = |A*c* + B|
		auto feasible = [&](double s) {
			const double A = s * nh;
			double c = c_hi;
			if (A > 1e-12) {
				const double cz = -B / A;
				c = cz < c_lo ? c_lo : (cz > c_hi ? c_hi : cz);
			}
			const double L = fabs(A * c + B);
			return s * s + vz2 - L * L >= need2;
		};
		// candidate boundaries: 0, the zero-loss band edges, and the
		// roots of the two edge quadratics
		double cand[14];
		int nc = 0;
		cand[nc++] = 0.0;
		// the zero-loss band's own feasibility crossing (the root
		// the first version missed - the falsifier caught it)
		if (need2 > vz2)
			cand[nc++] = sqrt(need2 - vz2);
		const double negB = -B;   // >= 0
		if (nh > 1e-12) {
			if (c_hi > 1e-12)
				cand[nc++] = negB / (nh * c_hi);
			if (c_lo > 1e-12)
				cand[nc++] = negB / (nh * c_lo);
		}
		// edge quadratic for c in {c_lo, c_hi}:
		//   s^2 (1 - nh^2 c^2) - 2 nh c B s + (vz^2 - B^2 - need2) = 0
		const double edges[2] = { c_lo, c_hi };
		for (int e = 0; e < 2; ++e) {
			const double c = edges[e];
			const double a2 = 1.0 - nh * nh * c * c;
			const double a1 = -2.0 * nh * c * B;
			const double a0 = vz2 - B * B - need2;
			if (fabs(a2) < 1e-12) {
				if (fabs(a1) > 1e-12)
					cand[nc++] = -a0 / a1;
			} else {
				const double disc = a1 * a1 - 4.0 * a2 * a0;
				if (disc >= 0.0) {
					const double rd = sqrt(disc);
					cand[nc++] = (-a1 - rd) / (2.0 * a2);
					cand[nc++] = (-a1 + rd) / (2.0 * a2);
				}
			}
		}
		// sort the valid candidates and take the first feasible
		// point (interval classification is exact: the condition is
		// one fixed quadratic between adjacent candidates)
		for (int i = 0; i < nc; ++i)
			for (int j = i + 1; j < nc; ++j)
				if (cand[j] < cand[i]) {
					const double tmp = cand[i];
					cand[i] = cand[j];
					cand[j] = tmp;
				}
		// every boundary of the feasible set is now a candidate, so
		// the infimum is the first candidate whose right side is
		// feasible
		double best_s = -1.0;
		for (int i = 0; i < nc; ++i) {
			const double s = cand[i];
			if (s < 0.0 || s > 4000.0)
				continue;
			if (feasible(s + 1e-6)) {
				best_s = s;
				break;
			}
		}
		if (best_s < 0.0) {
			if (feasible(4000.0))
				best_s = 4000.0;
			else
				return 1e30f;
		}
		const double s_in = sqrt(best_s * best_s + vz2);
		const double shaved = s_in - 0.05;
		return static_cast<float>(shaved > 0.0 ? shaved : 0.0);
	}

} // namespace CapBounds

// ======================================================================
// CAPHULL - registry A3 (hull contact geometry), session 32. The
// engine never traces the player as a point: every brush plane (n, d)
// is pre-expanded to (n, d + off(n)) where off is the Minkowski
// support of the NEGATED player hull (SolverWorld.cpp HullExpand),
// and the trace moves the hull ORIGIN (the feet point) against the
// expanded set. So every origin-space prediction - the A1 vertical
// timetable, A2 face slices, C7 boarding targets - must aim at
// d + off(n), NOT d. Aiming at the raw plane is exactly the measured
// C7 v1 systematic (contacts 1.8-3.7 ticks early: the hull's leading
// edge reaches the face before the origin does). Float arithmetic
// mirrors HullExpand exactly (same products, same left-to-right sum,
// one negation) so packaged offsets are BITWISE equal to the ones the
// world loader bakes into d_stand/d_duck/d_unduck.
namespace CapHull {

	// off(n) for hull box [mn, mx]: -min over hull corners of n.corner.
	inline float PlaneOffset(const Vec3& n, const Vec3& mn,
	                         const Vec3& mx) {
		return -((n.X > 0.f ? n.X * mn.X : n.X * mx.X)
			+ (n.Y > 0.f ? n.Y * mn.Y : n.Y * mx.Y)
			+ (n.Z > 0.f ? n.Z * mn.Z : n.Z * mx.Z));
	}

	// The player hulls by movement hull state (0 standing, 1 ducked,
	// 2 post-air-unduck transient, which is identical to standing).
	inline float PlaneOffsetHull(const Hulls& h, const Vec3& n,
	                             int hull) {
		const Vec3& mn = hull == 1 ? h.duck_min
			: (hull == 2 ? h.unduck_min : h.stand_min);
		const Vec3& mx = hull == 1 ? h.duck_max
			: (hull == 2 ? h.unduck_max : h.stand_max);
		return PlaneOffset(n, mn, mx);
	}

} // namespace CapHull

// ======================================================================
// CAPLABEL - registry C0 (canonical transition label) + C1 (local
// Pareto comparison), session 32. C0 is the ONE struct every kernel
// emits when it crosses a boundary (C7 exit->board, C6 board->exit),
// so downstream composition consumes a single exact format. Fields
// split into two roles:
//   COMPATIBILITY (kind, boundary position u/v, heading psi, vz,
//     sf_quarter): where/how the route continues. NOT ordered - two
//     labels with different compatibility reach different successor
//     states, and no monotonicity theorem in psi/vz/position has been
//     certified, so C1 never compares across them.
//   QUALITY (dt down, s2 up, loss2 down): certified-direction fields.
//     Fewer ticks is better by decree (C10: cost IS ticks); more s2
//     is better by the coast monotonicity theorem's direction; less
//     boundary loss is better because loss subtracts from s2 exactly.
// C1 dominance is therefore CELL-LOCAL: labels are comparable only
// when their compatibility fields fall in the same cell at the
// caller's chosen pitch, and within a cell a dominates b iff a is >=
// in every quality direction and > in at least one. Scalarizing the
// tuple into one number remains banned as truth (C11: order only).
namespace CapLabel {

	struct Transition {
		unsigned char kind = 0;     // 0 = exit->board (C7),
		                            // 1 = board->exit (C6)
		unsigned char sf_quarter = 0; // 1 = quarter-strength friction
		int   dt = 0;               // ticks consumed (the route cost)
		float u = 0.f, v = 0.f;     // boundary position, face frame
		float psi = 0.f;            // horizontal heading, radians
		float vz = 0.f;             // vertical velocity at boundary
		float s2 = 0.f;             // horizontal speed^2 at boundary
		float loss2 = 0.f;          // s^2 given up crossing it (>= 0)
	};

	// The compatibility cell at pitch (pos_pitch u, psi_pitch rad,
	// vz_pitch u/s). Same cell = comparable.
	inline bool SameCell(const Transition& a, const Transition& b,
	                     float pos_pitch, float psi_pitch,
	                     float vz_pitch) {
		if (a.kind != b.kind || a.sf_quarter != b.sf_quarter)
			return false;
		if (static_cast<int>(floorf(a.u / pos_pitch))
			!= static_cast<int>(floorf(b.u / pos_pitch)))
			return false;
		if (static_cast<int>(floorf(a.v / pos_pitch))
			!= static_cast<int>(floorf(b.v / pos_pitch)))
			return false;
		if (static_cast<int>(floorf(a.psi / psi_pitch))
			!= static_cast<int>(floorf(b.psi / psi_pitch)))
			return false;
		if (static_cast<int>(floorf(a.vz / vz_pitch))
			!= static_cast<int>(floorf(b.vz / vz_pitch)))
			return false;
		return true;
	}

	// C1: strict dominance inside one compatibility cell. Returns
	// true iff a is at least as good in every quality direction and
	// strictly better in one. Irreflexive by construction.
	inline bool Dominates(const Transition& a, const Transition& b) {
		if (a.dt > b.dt || a.s2 < b.s2 || a.loss2 > b.loss2)
			return false;
		return a.dt < b.dt || a.s2 > b.s2 || a.loss2 < b.loss2;
	}

} // namespace CapLabel

// ======================================================================
// CAPFRAME - registry A0 (canonical frame transform), session 33.
// The one rotation+translation every solver family uses: solves run
// in the canonical frame (start at the origin, initial velocity
// along +x), witnesses stay world-frame. The transform enters ONLY
// through the target: schedules are side/cosa control laws that
// re-derive wish inputs from the CURRENT velocity heading each tick,
// so a schedule is frame-free by construction. Float rotation is NOT
// exact - that is WHY witnesses are never rotated: the suite MEASURES
// the rotation-commutation deviation instead of assuming it away.
namespace CapFrame {

	struct Frame2D {
		float c = 1.f, s = 0.f;   // cos/sin of -psi0 (world->canon)
		float ox = 0.f, oy = 0.f; // world origin of the frame
	};

	// The exact float expressions the solvers inline (same ops, same
	// order): c = cosf(-psi0), s = sinf(-psi0).
	inline Frame2D MakeCanonical(float psi0, float ox, float oy) {
		Frame2D f;
		f.c = cosf(-psi0);
		f.s = sinf(-psi0);
		f.ox = ox;
		f.oy = oy;
		return f;
	}
	inline void ToCanonical(const Frame2D& f, float wx, float wy,
	                        float* cx, float* cy) {
		const float dx = wx - f.ox;
		const float dy = wy - f.oy;
		*cx = f.c * dx - f.s * dy;
		*cy = f.s * dx + f.c * dy;
	}
	inline void FromCanonical(const Frame2D& f, float cx, float cy,
	                          float* wx, float* wy) {
		// inverse rotation: transpose (angle +psi0)
		*wx = f.ox + f.c * cx + f.s * cy;
		*wy = f.oy - f.s * cx + f.c * cy;
	}

} // namespace CapFrame

// ======================================================================
// CAPP2P additions - registry A12 (free-N point-to-point) and A10
// (minimum air time), session 33. A12 is A11 iterated over N with
// the certified B13 precull (MinDepartSpeed > v0 is a NECESSARY-
// condition skip - it can never hide a feasible N); A10 intersects
// the A1 vertical timetable's z-window with that scan and stops at
// the FIRST feasible N.
namespace CapP2P {

	// A12: smallest N in [n_lo, n_hi] whose fixed-N solve reaches
	// (tx, ty) within the solver contract (residual <= 0.5). Returns
	// the N, or -1. *culled counts B13 precull skips.
	inline int SolveFreeN(const CapAir::AirKernelCtx& k, float v0,
	                      float tx, float ty, int n_lo, int n_hi,
	                      P2PResult* out, int* culled = nullptr) {
		const float D = sqrtf(tx * tx + ty * ty);
		if (culled)
			*culled = 0;
		for (int N = n_lo; N <= n_hi; ++N) {
			if (N < 1)
				continue;
			const float vmin = CapBounds::MinDepartSpeed(k.p, D,
				0.f, N);
			if (vmin > v0) {
				if (culled)
					(*culled)++;
				continue;
			}
			P2PResult r;
			SolveFixedN(k, v0, tx, ty, N, &r);
			if (r.solved) {
				*out = r;
				return N;
			}
		}
		return -1;
	}

	// A10: minimum air time to a target whose arrival must land the
	// A1 vertical state inside [z_lo, z_hi]. The vertical timetable
	// is the exact recurrence from (z0, vz0); horizontal feasibility
	// is the A12 scan restricted to vertically-legal N.
	inline int MinAirTime(const CapAir::AirKernelCtx& k, float v0,
	                      float tx, float ty, float z0, float vz0,
	                      float z_lo, float z_hi, int n_hi,
	                      P2PResult* out, int* culled = nullptr) {
		const float D = sqrtf(tx * tx + ty * ty);
		if (culled)
			*culled = 0;
		float z = z0, vz = vz0;
		for (int N = 0; N <= n_hi; ++N) {
			const bool zok = z >= z_lo && z <= z_hi;
			if (N >= 1 && zok) {
				const float vmin = CapBounds::MinDepartSpeed(k.p,
					D, 0.f, N);
				if (vmin > v0) {
					if (culled)
						(*culled)++;
				} else {
					P2PResult r;
					SolveFixedN(k, v0, tx, ty, N, &r);
					if (r.solved) {
						*out = r;
						return N;
					}
				}
			}
			CapWindow::VTick(k.p, &z, &vz);
		}
		return -1;
	}

} // namespace CapP2P

// ======================================================================
// CAPFACESOLVE - registry A13 (point-to-face solve), session 33.
// The packaged form of the C7 composition's solving core: candidate
// arrival ticks from the A1 vertical timetable, the face's 1-D slice
// line per tick on the HULL-EXPANDED plane (A2 + A3), the certified
// B13 cull, batch A11 per tick (A12 restricted to the slice), and
// the EXACT contact prediction (A4 kernel roll + the engine's trace
// clip law: fraction (d1 - 1/32)/(d1 - d2), end exactly 1/32 above
// the traced plane, clipped move velocity slides the remaining
// time). capxfer consumes THIS implementation - its gates are the
// acceptance tests.
namespace CapFaceSolve {

	struct FaceSolveCfg {
		Vec3 n;                     // face normal (unit, engine's)
		float d_exp = 0.f;          // HULL-EXPANDED plane distance
		int t_lo = 1, t_hi = 90, t_step = 1;
		float z_lo = -1e30f, z_hi = 1e30f;  // boarding band (origin z)
		float lam_lo = 0.f, lam_hi = 0.f, lam_step = 1.f;
		int horizon = 95;           // kernel-roll limit
	};
	struct FaceHit {
		int T = 0;                  // requested arrival tick
		float lam = 0.f;            // slice-line coordinate
		float wx = 0.f, wy = 0.f;   // world target on the line
		float zT = 0.f;             // timetable z at T
		int pred_tick = -1;         // EXACT predicted contact tick
		Vec3 pred_end;              // EXACT predicted end position
		CapP2P::P2PResult res;      // the A11 solution
	};

	// The exact contact prediction for one schedule against one
	// expanded plane. Returns the predicted contact tick (engine
	// tick index, first contact) or -1; *end_out = the engine's
	// end-of-tick position under the clip law.
	inline int PredictContact(const CapAir::AirKernelCtx& k,
	                          const Vec3& n, float d_exp,
	                          const Vec3& xpos, const Vec3& xvel,
	                          const CapP2P::P2PSchedule& sched,
	                          int horizon, Vec3* end_out) {
		float kx = xpos.X, ky = xpos.Y, kz = xpos.Z;
		float kvx = xvel.X, kvy = xvel.Y, kvz = xvel.Z;
		for (int t = 0; t < horizon; ++t) {
			const signed char ds = t < sched.n ? sched.side[t] : 0;
			const float cca = t < sched.n ? sched.cosa[t] : 1.f;
			float nx2, ny2, nz2, nvx2, nvy2, nvz2;
			CapAir::KernelTick(k, kx, ky, kz, kvx, kvy, kvz, ds,
				cca, &nx2, &ny2, &nz2, &nvx2, &nvy2, &nvz2);
			const float da = n.X * kx + n.Y * ky + n.Z * kz
				- d_exp;
			const float db = n.X * nx2 + n.Y * ny2 + n.Z * nz2
				- d_exp;
			if (db <= 0.f && da > 0.f) {
				float fe = (da - 0.03125f) / (da - db);
				if (fe < 0.f)
					fe = 0.f;
				const Vec3 C(kx + (nx2 - kx) * fe,
					ky + (ny2 - ky) * fe,
					kz + (nz2 - kz) * fe);
				const Vec3 vm((nx2 - kx) / k.p.dt,
					(ny2 - ky) / k.p.dt,
					(nz2 - kz) / k.p.dt);
				Vec3 vc;
				Fn::ClipVelocity(vm, n, &vc);
				const float rem = (1.f - fe) * k.p.dt;
				if (end_out)
					*end_out = Vec3(C.X + vc.X * rem,
						C.Y + vc.Y * rem, C.Z + vc.Z * rem);
				return t + 1;
			}
			kx = nx2;
			ky = ny2;
			kz = nz2;
			kvx = nvx2;
			kvy = nvy2;
			kvz = nvz2;
		}
		return -1;
	}

	// A13: enumerate (T, lambda) targets on the face and solve each
	// through batch A11, predicting every solved schedule's contact
	// exactly. out receives one FaceHit per SOLVED target;
	// *targets/*culled count the enumeration; *organize_us the batch
	// builds; *solve_us the per-target solving.
	inline void SolveToFace(const CapAir::AirKernelCtx& k,
	                        const FaceSolveCfg& c, const Vec3& xpos,
	                        const Vec3& xvel,
	                        std::vector<FaceHit>* out, int* targets,
	                        int* culled, double* organize_us,
	                        double* solve_us) {
		out->clear();
		if (targets)
			*targets = 0;
		if (culled)
			*culled = 0;
		if (organize_us)
			*organize_us = 0.0;
		if (solve_us)
			*solve_us = 0.0;
		const float v0h = sqrtf(xvel.X * xvel.X + xvel.Y * xvel.Y);
		const float psi0 = atan2f(xvel.Y, xvel.X);
		const CapFrame::Frame2D F = CapFrame::MakeCanonical(psi0,
			xpos.X, xpos.Y);
		// the A1 vertical timetable (t range clamped to the table)
		float zt[192];
		const int t_hi_eff = c.t_hi < 191 ? c.t_hi : 191;
		{
			float z = xpos.Z, vz = xvel.Z;
			for (int t = 0; t <= t_hi_eff; ++t) {
				zt[t] = z;
				CapWindow::VTick(k.p, &z, &vz);
			}
		}
		// the slice line's horizontal geometry
		const float nh2 = c.n.X * c.n.X + c.n.Y * c.n.Y;
		if (nh2 <= 0.f)
			return;   // horizontal face has no slice line
		const float nhl = sqrtf(nh2);
		const float ex = -c.n.Y / nhl, ey = c.n.X / nhl;
		CapP2P::P2PBatch B;
		int batchN = -1;
		for (int T = c.t_lo; T <= t_hi_eff; T += c.t_step) {
			const float zT = zt[T];
			if (zT < c.z_lo || zT > c.z_hi)
				continue;
			// base point of the slice line: n_h . p = rhs
			const float rhs = c.d_exp - c.n.Z * zT;
			const float bx = c.n.X * rhs / nh2;
			const float by = c.n.Y * rhs / nh2;
			for (float lam = c.lam_lo; lam <= c.lam_hi;
				lam += c.lam_step) {
				if (targets)
					(*targets)++;
				const float wx = bx + ex * lam;
				const float wy = by + ey * lam;
				const float dx = wx - xpos.X;
				const float dy = wy - xpos.Y;
				const float D = sqrtf(dx * dx + dy * dy);
				if (CapBounds::MinDepartSpeed(k.p, D, 0.f, T)
					> v0h) {
					if (culled)
						(*culled)++;
					continue;
				}
				float tx, ty;
				CapFrame::ToCanonical(F, wx, wy, &tx, &ty);
				if (batchN != T) {
					const auto tb =
						std::chrono::steady_clock::now();
					CapP2P::BuildP2PBatch(k, v0h, T, &B);
					if (organize_us)
						*organize_us += std::chrono::duration<
							double, std::micro>(
							std::chrono::steady_clock::now()
							- tb).count();
					batchN = T;
				}
				const auto ts = std::chrono::steady_clock::now();
				CapP2P::P2PResult res;
				CapP2P::SolveTargetBatch(k, B, tx, ty, &res);
				if (solve_us)
					*solve_us += std::chrono::duration<double,
						std::micro>(
						std::chrono::steady_clock::now()
						- ts).count();
				if (!res.solved)
					continue;
				FaceHit h;
				h.T = T;
				h.lam = lam;
				h.wx = wx;
				h.wy = wy;
				h.zT = zT;
				h.res = res;
				h.pred_tick = PredictContact(k, c.n, c.d_exp,
					xpos, xvel, res.sched, c.horizon,
					&h.pred_end);
				out->push_back(h);
			}
		}
	}

} // namespace CapFaceSolve

// ======================================================================
// CAPRIDEREACH addition - registry A21 (minimum ride time), session
// 33: the inverse query of the A20 surface. The first layer whose
// reachable set answers an in-plane target within the lattice
// contract IS the minimum ride time at that resolution; the layer's
// witness (parent chain) is the constructive schedule.
namespace CapRideReach {

	// Smallest N in [1, S.p.n_max] whose layer answers (tu, tv)
	// within `contract` units in-plane. Returns N (with *node_out /
	// *res_out from that layer's query) or -1. The indexed query
	// answers first; a layer is SKIPPED only after the exhaustive
	// scan confirms the miss (the ring representative can sit
	// farther than another cell's node), so the result equals the
	// brute per-layer minimum exactly.
	inline int MinRideTime(const RRSurface& S, float tu, float tv,
	                       float contract, int* node_out = nullptr,
	                       float* res_out = nullptr) {
		for (int N = 1; N <= S.p.n_max; ++N) {
			float res = 1e30f;
			int node = QueryTargetFast(S, N, tu, tv, &res);
			if (node >= 0 && res > contract) {
				float rex = 1e30f;
				const int nex = QueryTarget(S, N, tu, tv, &rex);
				if (nex >= 0 && rex < res) {
					node = nex;
					res = rex;
				}
			}
			if (node >= 0 && res <= contract) {
				if (node_out)
					*node_out = node;
				if (res_out)
					*res_out = res;
				return N;
			}
		}
		return -1;
	}

} // namespace CapRideReach

// ======================================================================
// CAPLAUNCH - registry A26 (edge launch), session 33. The unified
// leave-ground event detector: feed the per-tick PlayerState stream
// of ANY exact rollout; the detector reports the first grounded ->
// airborne transition and captures both boundary states. The
// capability owns no physics - the states come from the engine (or
// a certified kernel), so the captured launch state is exact by
// construction; the suite gates re-anchored bitwise continuation,
// the A3 hull-overhang law at the edge, and the A1 handoff on the
// first airborne tick.
namespace CapLaunch {

	struct Launch {
		bool fired = false;
		int tick = -1;            // tick index of the FIRST airborne
		                          // state (post)
		PlayerState pre;          // last grounded state
		PlayerState post;         // first airborne state
	};

	struct Detector {
		bool seen_ground = false;
		bool prev_ground = false;
		bool have_prev = false;
		PlayerState prev;
		Launch out;
		// Feed the state AFTER tick `tick` executed.
		void Feed(int tick, const PlayerState& s) {
			if (out.fired) {
				return;
			}
			const bool g = s.on_ground;
			if (have_prev && seen_ground && prev_ground && !g) {
				out.fired = true;
				out.tick = tick;
				out.pre = prev;
				out.post = s;
			}
			if (g)
				seen_ground = true;
			prev = s;
			prev_ground = g;
			have_prev = true;
		}
	};

} // namespace CapLaunch

// ======================================================================
// CAPCONTACT - registry A14 (first-contact prediction), session 34.
// The engine's own per-brush clip (TraceHull3's verified loop: the
// enter-clamp tie-break, DIST_EPSILON padding, real-side corner
// release, the start-solid law) run against a PREBUILT LOCAL BRUSH
// SET instead of the whole world - the shape route legs consume: one
// corridor gather, then per-tick segment queries over the 1-3
// brushes that matter. Arithmetic is transcribed from
// SolverWorld.cpp VERBATIM so answers are BITWISE the full trace's;
// the suite gates that identity on every query. A query whose
// segment leaves the corridor box DECLINES (returns -1) rather than
// answering from an incomplete set. Ordering note: the engine's
// processing order is observable only through mid-loop allsolid
// zeroing; a first-contact flight query never starts inside a brush,
// and the bitwise gate would catch any ordering divergence.
namespace CapContact {

	struct LocalSet {
		const World* w = nullptr;
		int hull = 0;
		Vec3 lo, hi;               // the corridor box (origin space)
		std::vector<int> brushes;  // indices into w->brushes
	};

	// Gather every brush whose hull-expanded AABB gate touches the
	// corridor box.
	inline void BuildLocalSet(const World& w, int hull,
	                          const Vec3& lo, const Vec3& hi,
	                          LocalSet* out) {
		out->w = &w;
		out->hull = hull;
		out->lo = lo;
		out->hi = hi;
		out->brushes.clear();
		for (size_t bi = 0; bi < w.brushes.size(); ++bi) {
			const WorldBrush& bc = w.brushes[bi];
			const Vec3& gmn = hull == 1 ? bc.gmin_duck
				: hull == 2 ? bc.gmin_unduck : bc.gmin_stand;
			const Vec3& gmx = hull == 1 ? bc.gmax_duck
				: hull == 2 ? bc.gmax_unduck : bc.gmax_stand;
			if (hi.X < gmn.X || lo.X > gmx.X || hi.Y < gmn.Y
				|| lo.Y > gmx.Y || hi.Z < gmn.Z || lo.Z > gmx.Z)
				continue;
			out->brushes.push_back(static_cast<int>(bi));
		}
	}

	// Exact clip of the origin segment a->b against the local set.
	// Mirrors TraceHull3's per-brush loop bitwise. Returns the
	// fraction (1 = clean), or -1 DECLINE when the segment leaves
	// the corridor. brush/plane/startsolid report as the full trace
	// would.
	inline float ClipSegment(const LocalSet& S, const Vec3& a,
	                         const Vec3& b, int* brush_out = nullptr,
	                         int* plane_out = nullptr,
	                         bool* startsolid_out = nullptr,
	                         bool* allsolid_out = nullptr) {
		const Vec3 lo(fminf(a.X, b.X), fminf(a.Y, b.Y),
			fminf(a.Z, b.Z));
		const Vec3 hi(fmaxf(a.X, b.X), fmaxf(a.Y, b.Y),
			fmaxf(a.Z, b.Z));
		if (lo.X < S.lo.X || hi.X > S.hi.X || lo.Y < S.lo.Y
			|| hi.Y > S.hi.Y || lo.Z < S.lo.Z || hi.Z > S.hi.Z)
			return -1.f;   // DECLINE: outside the gathered corridor
		const World& w = *S.w;
		const int hull = S.hull;
		float best = 1.f;
		int best_brush = -1, best_plane = -1;
		bool l_ss = false, l_as = false;
		for (size_t oi = 0; oi < S.brushes.size(); ++oi) {
			const int bi = S.brushes[oi];
			const WorldBrush& bc = w.brushes[static_cast<size_t>(
				bi)];
			const Vec3& gmn = hull == 1 ? bc.gmin_duck
				: hull == 2 ? bc.gmin_unduck : bc.gmin_stand;
			const Vec3& gmx = hull == 1 ? bc.gmax_duck
				: hull == 2 ? bc.gmax_unduck : bc.gmax_stand;
			if (hi.X < gmn.X || lo.X > gmx.X || hi.Y < gmn.Y
				|| lo.Y > gmx.Y || hi.Z < gmn.Z || lo.Z > gmx.Z)
				continue;
			const std::vector<float>& pd = hull == 1 ? bc.d_duck
				: hull == 2 ? bc.d_unduck : bc.d_stand;
			float tmin = -1.f, tmax = 1.f;
			float tmin_t = -1.f, tmax_t = 1e9f;
			int enter = -1;
			bool outside = false, miss = false;
			bool getout = false;
			const int np = static_cast<int>(bc.n.size());
			for (int pi = 0; pi < np; ++pi) {
				const float d0 = Dot(bc.n[pi], a) - pd[pi];
				const float d1 = Dot(bc.n[pi], b) - pd[pi];
				if (d1 > 0.f)
					getout = true;
				if (d0 > 0.f) {
					outside = true;
					if (d1 > 0.f) {
						miss = true;
						break;
					}
					float tt = (d0 - 0.03125f) / (d0 - d1);
					if (tt < 0.f)
						tt = 0.f;
					if (tt > tmin) {
						tmin = tt;
						enter = pi;
					}
					const float tn = d0 / (d0 - d1);
					if (tn > tmin_t)
						tmin_t = tn;
				} else if (d1 > 0.f) {
					float tt = (d0 + 0.03125f) / (d0 - d1);
					if (tt > 1.f)
						tt = 1.f;
					if (tt < tmax)
						tmax = tt;
					if (bc.pid[pi] >= 0) {
						const float tn = (d0 - 0.03125f)
							/ (d0 - d1);
						if (tn < tmax_t)
							tmax_t = tn;
					}
				}
			}
			if (!miss && !outside) {
				l_ss = true;
				if (!getout) {
					l_as = true;
					best = 0.f;   // the mid-loop allsolid zeroing
				}
				continue;
			}
			if (miss || !outside || enter < 0 || tmin >= tmax)
				continue;
			if (w.true_interval_corner && tmin_t >= tmax_t)
				continue;   // corner release (engine-measured rule)
			if (tmin < best) {
				best = (tmin > 0.f) ? tmin : 0.f;
				best_brush = bi;
				best_plane = enter;
			}
		}
		if (brush_out)
			*brush_out = best_brush;
		if (plane_out)
			*plane_out = best_plane;
		if (startsolid_out)
			*startsolid_out = l_ss;
		if (allsolid_out)
			*allsolid_out = l_as;
		return best;
	}

	// ------------------------------------------------------------------
	// A28 (corridor traversal), session 35: the A12 + A14 composition.
	// Roll a control schedule through the certified air kernel and clip
	// every motion segment against the local set. Returns -1 when the
	// path is CLEAN through `horizon` ticks, else the first contact
	// tick (engine tick index); the obstruction reports exactly
	// (fraction, brush, plane). Returns -2 (DECLINE) if any segment
	// leaves the corridor - never a silent answer from an incomplete
	// set.
	inline int FirstContactOnPath(const CapAir::AirKernelCtx& k,
	                              const LocalSet& S, const Vec3& pos,
	                              const Vec3& vel,
	                              const signed char* sides,
	                              const float* cosas, int n_sched,
	                              int horizon,
	                              float* frac_out = nullptr,
	                              int* brush_out = nullptr,
	                              int* plane_out = nullptr) {
		float kx = pos.X, ky = pos.Y, kz = pos.Z;
		float kvx = vel.X, kvy = vel.Y, kvz = vel.Z;
		for (int t = 0; t < horizon; ++t) {
			const signed char ds = t < n_sched ? sides[t] : 0;
			const float cca = t < n_sched ? cosas[t] : 1.f;
			float nx2, ny2, nz2, nvx2, nvy2, nvz2;
			CapAir::KernelTick(k, kx, ky, kz, kvx, kvy, kvz, ds,
				cca, &nx2, &ny2, &nz2, &nvx2, &nvy2, &nvz2);
			int lb, lp;
			bool lss;
			const float lf = ClipSegment(S, Vec3(kx, ky, kz),
				Vec3(nx2, ny2, nz2), &lb, &lp, &lss);
			if (lf < 0.f)
				return -2;   // DECLINE: left the corridor
			if (lf < 1.f) {
				if (frac_out)
					*frac_out = lf;
				if (brush_out)
					*brush_out = lb;
				if (plane_out)
					*plane_out = lp;
				return t + 1;
			}
			kx = nx2;
			ky = ny2;
			kz = nz2;
			kvx = nvx2;
			kvy = nvy2;
			kvz = nvz2;
		}
		return -1;   // clean through the horizon
	}

} // namespace CapContact

// ======================================================================
// CAPP2P addition - registry A9 (displacement with a terminal
// constraint), session 34: reach (tx, ty) at tick N with terminal
// heading psi within a tolerance. The family enumeration ranks by
// endpoint through the O(k) segment tables, kernel-rolls the
// heading-feasible candidates for their EXACT terminal state, then
// drives the best one to the endpoint contract with a compact
// reversal climb + two-parameter exact-hit tail. ACCEPTS only when
// BOTH contracts hold (endpoint <= 0.5, |psi error| <= tol);
// otherwise DECLINES honestly - the caller never receives a
// silently-wrong schedule.
namespace CapP2P {

	inline float WrapAngle(float a) {
		while (a > 3.14159265f)
			a -= 6.2831853f;
		while (a < -3.14159265f)
			a += 6.2831853f;
		return a;
	}

	inline bool SolveTerminal(const CapAir::AirKernelCtx& k, float v0,
	                          float tx, float ty, int N,
	                          float psi_req, float psi_tol,
	                          P2PResult* out, float* psi_err_out,
	                          int* diag_cands = nullptr,
	                          int* diag_feas = nullptr,
	                          float* diag_best_perr = nullptr,
	                          float* diag_best_res = nullptr,
	                          float* diag_res_perr = nullptr) {
		struct Cand {
			int ss = 1, nrev = 0;
			int revs[3] = { 0, 0, 0 };
			int bs = 0, bl = 0;
			float bcv = 0.9f;   // brake strength (stored cosa) - a
			                    // continuous range lever
			float res = 1e30f;
		};
		SegTables T;
		BuildSegTables(k.p, v0, N, &T);
		// stage 1: rank the reversal family by endpoint residual
		std::vector<Cand> cands;
		auto offerSeg = [&](int ss, const int* revs, int nrev) {
			float ex, ey;
			EvalSegs(T, ss, revs, nrev, &ex, &ey);
			const float dx = ex - tx, dy = ey - ty;
			const float r = sqrtf(dx * dx + dy * dy);
			if (r > 128.f)
				return;
			Cand c;
			c.ss = ss;
			c.nrev = nrev;
			for (int i = 0; i < 3; ++i)
				c.revs[i] = i < nrev ? revs[i] : 0;
			c.res = r;
			cands.push_back(c);
		};
		for (int ss = -1; ss <= 1; ss += 2) {
			int revs[3] = { 0, 0, 0 };
			offerSeg(ss, revs, 0);
			for (int r1 = 1; r1 < N; ++r1) {
				revs[0] = r1;
				offerSeg(ss, revs, 1);
				for (int r2 = r1 + 6; r2 < N; ++r2) {
					revs[1] = r2;
					offerSeg(ss, revs, 2);
				}
			}
			for (int r1 = 1; r1 < N; r1 += 2) {
				revs[0] = r1;
				for (int r2 = r1 + 6; r2 < N; r2 += 2) {
					revs[1] = r2;
					for (int r3 = r2 + 6; r3 < N; r3 += 3) {
						revs[2] = r3;
						offerSeg(ss, revs, 3);
					}
				}
			}
		}
		// brake probes are the HEADING control: where the brake run
		// sits decides how much of the turning happens slow (before)
		// vs fast (after, turn rate atan(30/s)) - so the probe grid
		// varies brake START and length, with and without a reversal
		{
			const int blstep = N / 8 < 1 ? 1 : N / 8;
			const int bss[3] = { 1, N / 4, N / 2 };
			P2PSchedule s;
			auto offerBrake = [&](int ss, const int* revs, int nrev,
				int bs, int bl, float bcv) {
				BuildSchedule(N, ss, revs, nrev, bs, bl, bcv, 0,
					0.f, 0.f, &s);
				float ex, ey, evx, evy;
				Roll(k, v0, s, &ex, &ey, &evx, &evy);
				const float dx = ex - tx, dy = ey - ty;
				const float r = sqrtf(dx * dx + dy * dy);
				if (r > 128.f)
					return;
				Cand c;
				c.ss = ss;
				c.nrev = nrev;
				for (int i = 0; i < 3; ++i)
					c.revs[i] = i < nrev ? revs[i] : 0;
				c.bs = bs;
				c.bl = bl;
				c.bcv = bcv;
				c.res = r;
				cands.push_back(c);
			};
			for (int ss = -1; ss <= 1; ss += 2)
				for (int bi = 0; bi < 3; ++bi)
					for (int bl = blstep; bl <= (3 * N) / 4;
						bl += blstep) {
						if (bl < 2)
							continue;
						int rv0[3] = { 0, 0, 0 };
						offerBrake(ss, rv0, 0, bss[bi], bl, 0.9f);
						offerBrake(ss, rv0, 0, bss[bi], bl, 0.6f);
						for (int r1 = 6; r1 < N; r1 += 12) {
							rv0[0] = r1;
							offerBrake(ss, rv0, 1, bss[bi], bl,
								0.9f);
							offerBrake(ss, rv0, 1, bss[bi], bl,
								0.6f);
						}
					}
		}
		// stage 2: kernel-roll for exact terminal heading, keep the
		// heading-feasible candidates ranked by |psi error|
		struct Scored {
			Cand c;
			float perr = 1e30f;
		};
		std::vector<Scored> feas;
		P2PSchedule s;
		for (size_t i = 0; i < cands.size(); ++i) {
			const Cand& c = cands[i];
			BuildSchedule(N, c.ss, c.revs, c.nrev, c.bs, c.bl,
				c.bcv, 0, 0.f, 0.f, &s);
			float ex, ey, evx, evy;
			Roll(k, v0, s, &ex, &ey, &evx, &evy);
			const float perr = fabsf(WrapAngle(atan2f(evy, evx)
				- psi_req));
			if (perr <= psi_tol + 0.35f) {
				Scored sc;
				sc.c = c;
				sc.perr = perr;
				feas.push_back(sc);
			}
		}
		std::sort(feas.begin(), feas.end(),
			[](const Scored& a, const Scored& b) {
				return a.perr < b.perr;
			});
		if (diag_cands)
			*diag_cands = static_cast<int>(cands.size());
		if (diag_feas)
			*diag_feas = static_cast<int>(feas.size());
		if (diag_best_perr)
			*diag_best_perr = feas.empty() ? 1e30f
				: feas[0].perr;
		// stage 3: for up to 10 heading-nearest candidates - fix the
		// endpoint (reversal climb + FD tail Newton), then STEER THE
		// HEADING: single-parameter neighbor steps over reversal
		// times and the brake run's start/length, each step re-fixing
		// the endpoint before the heading is judged. Accept the first
		// candidate meeting BOTH contracts.
		for (size_t fi = 0; fi < feas.size() && fi < 24; ++fi) {
			Cand c = feas[fi].c;
			float tc1 = 0.f, tc2 = 0.f;
			// the exact-hit tail must live in the POST-BRAKE free
			// region: a blanket tail overwrote brake-run ticks and
			// moved brake candidates far from their stage-2 ranking
			auto tmOf = [&]() {
				const int free_t = N - (c.bl ? c.bs + c.bl : 0);
				return free_t >= 14 ? 12
					: (free_t >= 10 ? 8 : (free_t >= 5 ? 4 : 0));
			};
			auto evalC = [&](float* pex, float* pey, float* ppsi) {
				BuildSchedule(N, c.ss, c.revs, c.nrev, c.bs, c.bl,
					c.bcv, tmOf(), tc1, tc2, &s);
				float ex, ey, evx, evy;
				Roll(k, v0, s, &ex, &ey, &evx, &evy);
				*pex = ex;
				*pey = ey;
				if (ppsi)
					*ppsi = atan2f(evy, evx);
			};
			auto resNow = [&]() {
				float ex, ey;
				evalC(&ex, &ey, nullptr);
				return sqrtf((ex - tx) * (ex - tx)
					+ (ey - ty) * (ey - ty));
			};
			// FD tail Newton on (tc1, tc2), THREE STARTS (the proven
			// SolveFixedN shape); returns the best residual
			auto newtonFrom = [&](float s1, float s2, int iters) {
				tc1 = s1;
				tc2 = s2;
				float res = resNow();
				if (tmOf() == 0)
					return res;
				for (int it = 0; it < iters && res > 0.25f;
					++it) {
					const float h = 0.02f;
					float ex0, ey0;
					evalC(&ex0, &ey0, nullptr);
					float exa, eya, exb, eyb;
					tc1 += h;
					evalC(&exa, &eya, nullptr);
					tc1 -= h;
					tc2 += h;
					evalC(&exb, &eyb, nullptr);
					tc2 -= h;
					const float j11 = (exa - ex0) / h;
					const float j21 = (eya - ey0) / h;
					const float j12 = (exb - ex0) / h;
					const float j22 = (eyb - ey0) / h;
					const float det = j11 * j22 - j12 * j21;
					if (fabsf(det) < 1e-6f)
						break;
					const float rx = tx - ex0, ry = ty - ey0;
					float d1 = (rx * j22 - ry * j12) / det;
					float d2 = (ry * j11 - rx * j21) / det;
					if (d1 > 0.6f) d1 = 0.6f;
					if (d1 < -0.6f) d1 = -0.6f;
					if (d2 > 0.6f) d2 = 0.6f;
					if (d2 < -0.6f) d2 = -0.6f;
					tc1 += d1;
					tc2 += d2;
					if (tc1 > 1.f) tc1 = 1.f;
					if (tc1 < -1.f) tc1 = -1.f;
					if (tc2 > 1.f) tc2 = 1.f;
					if (tc2 < -1.f) tc2 = -1.f;
					res = resNow();
				}
				return res;
			};
			auto fixEndpoint = [&](int iters) {
				const float s0a = tc1, s0b = tc2;
				float best_res = newtonFrom(s0a, s0b, iters);
				float b1 = tc1, b2 = tc2;
				if (best_res > 0.25f) {
					const float r2 = newtonFrom(0.45f, -0.45f,
						iters);
					if (r2 < best_res) {
						best_res = r2;
						b1 = tc1;
						b2 = tc2;
					}
				}
				if (best_res > 0.25f) {
					const float r3 = newtonFrom(-0.45f, 0.45f,
						iters);
					if (r3 < best_res) {
						best_res = r3;
						b1 = tc1;
						b2 = tc2;
					}
				}
				tc1 = b1;
				tc2 = b2;
				return best_res;
			};
			auto legal = [&](const Cand& cc) {
				for (int i = 0; i < cc.nrev; ++i) {
					if (cc.revs[i] < 1 || cc.revs[i] >= N)
						return false;
					if (i > 0 && cc.revs[i]
						< cc.revs[i - 1] + 6)
						return false;
				}
				if (cc.bl != 0) {
					if (cc.bs < 1 || cc.bl < 2
						|| cc.bs + cc.bl > N)
						return false;
				}
				return true;
			};
			// coarse SCORED climb: large integer steps toward the
			// endpoint that never sell out the heading - one kernel
			// roll per probe yields BOTH residual and terminal
			// heading, so the cheap climb judges the combined score
			// directly (heading-blind climbing measurably walked
			// candidates into far basins; heading-only judging
			// measurably stranded them 50u out - this is the middle)
			auto probe = [&](float* pres, float* pperr) {
				float ex0, ey0, psi0;
				evalC(&ex0, &ey0, &psi0);
				*pres = sqrtf((ex0 - tx) * (ex0 - tx)
					+ (ey0 - ty) * (ey0 - ty));
				*pperr = fabsf(WrapAngle(psi0 - psi_req));
			};
			auto cheapScore = [&](float r2, float pe2) {
				return (r2 > 0.5f ? (r2 - 0.5f) * 0.02f : 0.f)
					+ (pe2 > psi_tol ? (pe2 - psi_tol) * 1.f
						: 0.f);
			};
			{
				float cr, cp;
				probe(&cr, &cp);
				float cscore = cheapScore(cr, cp);
				bool improved = true;
				int guard = 0;
				while (improved && guard++ < 12) {
					improved = false;
					auto tryStep = [&](const Cand& c2) {
						if (!legal(c2))
							return;
						Cand save = c;
						c = c2;
						float r2, p2;
						probe(&r2, &p2);
						const float s2c = cheapScore(r2, p2);
						if (s2c < cscore - 1e-4f) {
							cscore = s2c;
							improved = true;
						} else {
							c = save;
						}
					};
					for (int i = 0; i < c.nrev; ++i)
						for (int d = -4; d <= 4; ++d) {
							if (d == 0)
								continue;
							Cand c2 = c;
							c2.revs[i] += d;
							tryStep(c2);
						}
					if (c.bl != 0) {
						for (int d = -4; d <= 4; d += 2) {
							if (d == 0)
								continue;
							Cand c2 = c;
							c2.bs += d;
							tryStep(c2);
							c2 = c;
							c2.bl += d;
							tryStep(c2);
						}
						// the brake-strength range lever
						const float dbs[4] = { -0.15f, -0.05f,
							0.05f, 0.15f };
						for (int di = 0; di < 4; ++di) {
							Cand c2 = c;
							c2.bcv += dbs[di];
							if (c2.bcv < 0.2f)
								c2.bcv = 0.2f;
							if (c2.bcv > 0.999f)
								c2.bcv = 0.999f;
							tryStep(c2);
						}
					}
					// paired translation (coupled valleys stall
					// one-at-a-time moves - the recorded lesson)
					for (int d = -4; d <= 4; ++d) {
						if (d == 0)
							continue;
						Cand c2 = c;
						for (int i = 0; i < c2.nrev; ++i)
							c2.revs[i] += d;
						tryStep(c2);
						if (c.bl != 0) {
							c2 = c;
							for (int i = 0; i < c2.nrev; ++i)
								c2.revs[i] += d;
							c2.bs += d;
							tryStep(c2);
						}
					}
				}
			}
			float res = fixEndpoint(5);
			// brake-to-the-end candidates have no tail room; buy a
			// 4-tick exact-hit tail by shrinking the brake
			for (int shrink = 0; shrink < 2 && res > 0.5f
				&& c.bl != 0 && tmOf() == 0; ++shrink) {
				Cand c2 = c;
				c2.bl -= 4;
				if (c2.bl < 2 || !legal(c2))
					break;
				c = c2;
				tc1 = 0.f;
				tc2 = 0.f;
				res = fixEndpoint(5);
			}
			float ex, ey, psi;
			evalC(&ex, &ey, &psi);
			float perr = fabsf(WrapAngle(psi - psi_req));
			// constrained local search: one combined score (heading
			// error + penalized endpoint violation), integer
			// neighbors over reversal times and the brake run, the
			// exact-hit Newton re-fixing the endpoint inside every
			// probe. Runs whether or not the endpoint has closed -
			// candidates stuck at res > 0.5 get discrete help too.
			int guard2 = 0;
			// violations only: inside both contracts the score is 0;
			// heading within tol is FREE (no pull away from the
			// endpoint), heading beyond tol costs like ~2u/deg
			auto scoreOf = [&](float r2, float pe2) {
				return (r2 > 0.5f ? (r2 - 0.5f) * 0.02f : 0.f)
					+ (pe2 > psi_tol ? (pe2 - psi_tol) * 1.f
						: 0.f);
			};
			float score = scoreOf(res, perr);
			while ((res > 0.5f || perr > psi_tol)
				&& guard2++ < 14) {
				Cand best_c = c;
				float best_tc1 = tc1, best_tc2 = tc2;
				float best_res = res, best_perr = perr;
				float best_score = score;
				bool moved = false;
				const int steps[4] = { -3, -1, 1, 3 };
				const Cand base = c;
				const float base_tc1 = tc1, base_tc2 = tc2;
				auto tryNeighbor = [&](const Cand& c2) {
					if (!legal(c2))
						return;
					c = c2;
					tc1 = base_tc1;
					tc2 = base_tc2;
					const float r2 = fixEndpoint(3);
					float ex2, ey2, psi2;
					evalC(&ex2, &ey2, &psi2);
					const float pe2 = fabsf(WrapAngle(psi2
						- psi_req));
					const float sc2 = scoreOf(r2, pe2);
					if (sc2 < best_score - 1e-4f) {
						best_score = sc2;
						best_res = r2;
						best_perr = pe2;
						best_c = c;
						best_tc1 = tc1;
						best_tc2 = tc2;
						moved = true;
					}
					c = base;
					tc1 = base_tc1;
					tc2 = base_tc2;
				};
				for (int i = 0; i < base.nrev; ++i)
					for (int si = 0; si < 4; ++si) {
						Cand c2 = base;
						c2.revs[i] += steps[si];
						tryNeighbor(c2);
					}
				if (base.bl != 0) {
					for (int si = 0; si < 4; ++si) {
						Cand c2 = base;
						c2.bs += steps[si] * 2;
						tryNeighbor(c2);
						c2 = base;
						c2.bl += steps[si] * 2;
						tryNeighbor(c2);
					}
					const float dbs[4] = { -0.1f, -0.03f, 0.03f,
						0.1f };
					for (int di = 0; di < 4; ++di) {
						Cand c2 = base;
						c2.bcv += dbs[di];
						if (c2.bcv < 0.2f)
							c2.bcv = 0.2f;
						if (c2.bcv > 0.999f)
							c2.bcv = 0.999f;
						tryNeighbor(c2);
					}
				}
				if (!moved)
					break;
				c = best_c;
				tc1 = best_tc1;
				tc2 = best_tc2;
				res = best_res;
				perr = best_perr;
				score = best_score;
			}
			// the FINISHER: three continuous controls (tail c1, tail
			// c2, brake strength) against exactly three constraints
			// (endpoint x, endpoint y, terminal heading) - a square
			// 3x3 FD Newton closes what discrete steps cannot
			if (c.bl != 0 && tmOf() > 0 && res <= 24.f
				&& perr <= psi_tol + 0.25f) {
				for (int it = 0; it < 10; ++it) {
					float ex0, ey0, ps0;
					evalC(&ex0, &ey0, &ps0);
					const float rx = tx - ex0, ry = ty - ey0;
					const float rp = WrapAngle(psi_req - ps0);
					const float rr = sqrtf(rx * rx + ry * ry);
					if (rr <= 0.25f && fabsf(rp)
						<= psi_tol * 0.5f)
						break;
					const float h1 = 0.02f, h3 = 0.01f;
					float exa, eya, psa, exb, eyb, psb, exc,
						eyc, psc;
					tc1 += h1;
					evalC(&exa, &eya, &psa);
					tc1 -= h1;
					tc2 += h1;
					evalC(&exb, &eyb, &psb);
					tc2 -= h1;
					const float sb = c.bcv;
					c.bcv = sb + h3 > 0.999f ? sb - h3 : sb + h3;
					const float hb = c.bcv - sb;
					evalC(&exc, &eyc, &psc);
					c.bcv = sb;
					const float j11 = (exa - ex0) / h1;
					const float j21 = (eya - ey0) / h1;
					const float j31 = WrapAngle(psa - ps0) / h1;
					const float j12 = (exb - ex0) / h1;
					const float j22 = (eyb - ey0) / h1;
					const float j32 = WrapAngle(psb - ps0) / h1;
					const float j13 = (exc - ex0) / hb;
					const float j23 = (eyc - ey0) / hb;
					const float j33 = WrapAngle(psc - ps0) / hb;
					const float det =
						j11 * (j22 * j33 - j23 * j32)
						- j12 * (j21 * j33 - j23 * j31)
						+ j13 * (j21 * j32 - j22 * j31);
					if (fabsf(det) < 1e-8f)
						break;
					float d1 = (rx * (j22 * j33 - j23 * j32)
						- j12 * (ry * j33 - j23 * rp)
						+ j13 * (ry * j32 - j22 * rp)) / det;
					float d2 = (j11 * (ry * j33 - j23 * rp)
						- rx * (j21 * j33 - j23 * j31)
						+ j13 * (j21 * rp - ry * j31)) / det;
					float d3 = (j11 * (j22 * rp - ry * j32)
						- j12 * (j21 * rp - ry * j31)
						+ rx * (j21 * j32 - j22 * j31)) / det;
					if (d1 > 0.5f) d1 = 0.5f;
					if (d1 < -0.5f) d1 = -0.5f;
					if (d2 > 0.5f) d2 = 0.5f;
					if (d2 < -0.5f) d2 = -0.5f;
					if (d3 > 0.08f) d3 = 0.08f;
					if (d3 < -0.08f) d3 = -0.08f;
					tc1 += d1;
					tc2 += d2;
					c.bcv += d3;
					if (tc1 > 1.f) tc1 = 1.f;
					if (tc1 < -1.f) tc1 = -1.f;
					if (tc2 > 1.f) tc2 = 1.f;
					if (tc2 < -1.f) tc2 = -1.f;
					if (c.bcv < 0.2f) c.bcv = 0.2f;
					if (c.bcv > 0.999f) c.bcv = 0.999f;
				}
			}
			evalC(&ex, &ey, &psi);
			res = sqrtf((ex - tx) * (ex - tx)
				+ (ey - ty) * (ey - ty));
			perr = fabsf(WrapAngle(psi - psi_req));
			if (diag_best_res && res < *diag_best_res) {
				*diag_best_res = res;
				if (diag_res_perr)
					*diag_res_perr = perr;
			}
			if (res <= 0.5f && perr <= psi_tol) {
				P2PResult r;
				BuildSchedule(N, c.ss, c.revs, c.nrev, c.bs, c.bl,
					c.bcv, tmOf(), tc1, tc2, &s);
				r.brake_cosa = c.bcv;
				float evx, evy;
				Roll(k, v0, s, &r.ex, &r.ey, &evx, &evy);
				r.evx = evx;
				r.evy = evy;
				r.solved = true;
				r.residual = res;
				r.start_side = c.ss;
				r.nrev = c.nrev;
				for (int i = 0; i < 3; ++i)
					r.revs[i] = c.revs[i];
				r.brake_start = c.bs;
				r.brake_len = c.bl;
				r.tail_c1 = tc1;
				r.tail_c2 = tc2;
				r.sched = s;
				*out = r;
				if (psi_err_out)
					*psi_err_out = perr;
				return true;
			}
		}
		return false;   // DECLINE
	}

} // namespace CapP2P

// ======================================================================
// CAPEDGE - registry A24 (direct contact transfer), session 35. The
// adjacent-face crossing tick's velocity laws, transcribed from the
// verified TryPlayerMove (SolverMove.cpp 55-173). A crossing takes
// one of two engine shapes:
//   SEQUENTIAL - the bump against face B comes after a partial move
//     (fraction > 0), which REBASES the clip set: two single-plane
//     clips compose, clip(clip(v, A), B), no guard between them.
//   CREASE - both planes accumulate with no progress: the engine
//     clips the REBASED velocity against each plane alone, and when
//     neither clears the other it projects the LAST clip result
//     (not the rebased velocity - float-distinct, dir.nB is not
//     exactly zero) onto the crease line cross(A, B); the stop-dead
//     guard zeroes anything turned past perpendicular to the tick's
//     primal velocity.
// The suite predicts every engine wedge crossing with BOTH laws and
// gates that one matches bitwise - the shape census is printed, and
// a crossing neither law explains is a RED.
namespace CapEdge {

	// TryPlayerMove 137-164, float-exact.
	inline void ResolveTwoPlanes(const Vec3& v_rebased, const Vec3& nA,
	                             const Vec3& nB, const Vec3& primal,
	                             Vec3* out) {
		Vec3 v;
		Fn::ClipVelocity(v_rebased, nA, &v);
		if (Dot(v, nB) < 0.f) {
			Fn::ClipVelocity(v_rebased, nB, &v);
			if (Dot(v, nA) < 0.f) {
				Vec3 dir(nA.Y * nB.Z - nA.Z * nB.Y,
					nA.Z * nB.X - nA.X * nB.Z,
					nA.X * nB.Y - nA.Y * nB.X);
				const float dl = Len(dir);
				if (dl > 1e-6f)
					dir = Scale(dir, 1.f / dl);
				// the engine projects s.vel = the LAST clip result
				v = Scale(dir, Dot(dir, v));
			}
		}
		if (Dot(v, primal) <= 0.f)
			v = Vec3();
		*out = v;
	}

	// The rebased sequential composition (airborne single-plane path
	// has no guard).
	inline void SequentialTransfer(const Vec3& v_m, const Vec3& nA,
	                               const Vec3& nB, Vec3* out) {
		Vec3 v1;
		Fn::ClipVelocity(v_m, nA, &v1);
		Fn::ClipVelocity(v1, nB, out);
	}

	// The verified TryPlayerMove (SolverMove.cpp 55-173) as a PURE
	// LOCAL function: every trace answered by the bitwise-certified
	// local clip (A14), so the whole bump chain - partial-move
	// rebases, the crease resolution, the stop-dead guard, the
	// mid-loop allsolid zeroing, the unswept stuck guard - is
	// reproduced without the engine or the world grid. The
	// two-plane laws above are the analytical decomposition; THIS is
	// the executable one. Returns false on a corridor DECLINE.
	inline bool TryMoveLocal(const CapContact::LocalSet& S,
	                         const MoveParams& p, bool on_ground,
	                         Vec3* pos, Vec3* vel,
	                         int* nbumps_out = nullptr) {
		float time_left = p.dt;
		Vec3 planes[5];
		int numplanes = 0;
		Vec3 original_v = *vel;
		const Vec3 primal_v = *vel;
		float all_fraction = 0.f;
		int nb = 0;
		for (int bump = 0; bump < 4; ++bump) {
			if (Len2(*vel) == 0.f)
				break;
			const Vec3 end = *pos + Scale(*vel, time_left);
			int tb, tp;
			bool tss, tas;
			const float frac = CapContact::ClipSegment(S, *pos, end,
				&tb, &tp, &tss, &tas);
			if (frac < 0.f)
				return false;   // DECLINE
			if (tas) {
				*vel = Vec3();
				nb++;
				break;
			}
			all_fraction += frac;
			if (frac > 0.f) {
				if (frac >= 1.f) {
					const Vec3 dest = *pos
						+ Scale(end - *pos, frac);
					int qb, qp;
					bool qss, qas;
					const float sf = CapContact::ClipSegment(S,
						dest, dest, &qb, &qp, &qss, &qas);
					if (sf < 0.f)
						return false;
					if (qss || sf != 1.f) {
						*pos = dest;
						*vel = Vec3();
						break;
					}
				}
				*pos = *pos + Scale(end - *pos, frac);
				original_v = *vel;
				numplanes = 0;
			}
			if (frac >= 1.f || tb < 0)
				break;
			time_left -= time_left * frac;
			const Vec3 n = S.w->brushes[static_cast<size_t>(tb)].n[
				static_cast<size_t>(tp)];
			nb++;
			if (numplanes >= 5) {
				*vel = Vec3();
				break;
			}
			planes[numplanes++] = n;
			if (numplanes == 1 && !on_ground) {
				Fn::ClipVelocity(original_v, planes[0], vel);
				original_v = *vel;
			} else {
				int i = 0;
				for (; i < numplanes; ++i) {
					Fn::ClipVelocity(original_v, planes[i], vel);
					int j = 0;
					for (; j < numplanes; ++j)
						if (j != i && Dot(*vel, planes[j]) < 0.f)
							break;
					if (j == numplanes)
						break;
				}
				if (i == numplanes) {
					if (numplanes != 2) {
						*vel = Vec3();
						break;
					}
					Vec3 dir(planes[0].Y * planes[1].Z
						- planes[0].Z * planes[1].Y,
						planes[0].Z * planes[1].X
						- planes[0].X * planes[1].Z,
						planes[0].X * planes[1].Y
						- planes[0].Y * planes[1].X);
					const float dl = Len(dir);
					if (dl > 1e-6f)
						dir = Scale(dir, 1.f / dl);
					*vel = Scale(dir, Dot(dir, *vel));
				}
				if (Dot(*vel, primal_v) <= 0.f) {
					*vel = Vec3();
					break;
				}
			}
		}
		if (all_fraction == 0.f)
			*vel = Vec3();
		if (nbumps_out)
			*nbumps_out = nb;
		return true;
	}

	// The full crossing-tick prediction, position AND velocity:
	// airborne, no wish, no basevel - start-gravity half -> clamp ->
	// TryMoveLocal (the whole bump chain) -> clamp -> finish half ->
	// clamp. CategorizePosition is state-only in this domain (no
	// walkable plane on ride-steep faces).
	inline bool EdgeTickFull(const CapContact::LocalSet& S,
	                         const MoveParams& p, float gscale,
	                         const Vec3& pos_pre, const Vec3& v_pre,
	                         Vec3* pos_out, Vec3* v_out,
	                         int* nbumps_out = nullptr) {
		Vec3 vel = v_pre;
		Vec3 pos = pos_pre;
		vel.Z -= gscale * p.gravity * 0.5f * p.dt;
		CapAir::KernelCheckVelocity(p, &vel.X, &vel.Y, &vel.Z);
		if (!TryMoveLocal(S, p, false, &pos, &vel, nbumps_out))
			return false;
		CapAir::KernelCheckVelocity(p, &vel.X, &vel.Y, &vel.Z);
		vel.Z -= gscale * p.gravity * 0.5f * p.dt;
		CapAir::KernelCheckVelocity(p, &vel.X, &vel.Y, &vel.Z);
		*pos_out = pos;
		*v_out = vel;
		return true;
	}

} // namespace CapEdge

// ======================================================================
// CAPRIDE addition - THE EXACT CARRIED-GAP ROLLER (registry A19/A20's
// named refinement + the A18 boundary sliver), session 36. The ride
// family's conservative stay-on-face guards existed because the
// contact boundary - the carried 1/32 standoff, its float drift, the
// hover band, re-contact fractions - had no exact law outside the
// engine. It does now, by COMPOSITION OF CERTIFIED PIECES: the
// airborne MoveTick chain's preamble (gravity half, clamp, the wish
// accel under the STATEFUL stale sf), the A24 local move mirror
// (TryMoveLocal - every trace answered by the bitwise A14 clip, so
// entry fractions, zero-frac steady ticks, hover misses, crease
// chains, and the stuck guard all resolve exactly), and the sf state
// law (0.25 when rising at categorize, applied stale). The carried
// gap needs no separate recurrence: it IS n.pos - d_exp of a
// bitwise-tracked position. This roller also lifts the air kernel's
// sf domain limit - sf is CARRIED, so braking wishes on rising ticks
// are exact here.
// Domain: airborne throughout (no walkable plane in the local set -
// surf-steep faces only), zero basevel, standing hull.
namespace CapRide {

	struct MirrorState {
		Vec3 pos;
		Vec3 vel;
		float sf = 1.f;   // stale surface friction (LAST tick's)
	};

	// One full airborne tick against the local set. Mirrors the
	// MoveTick airborne chain float-for-float (the kernel preamble's
	// order, then TryMoveLocal, then the finish chain), with the sf
	// state updated from vz at categorize.
	inline bool MirrorTick(const CapContact::LocalSet& S,
	                       const MoveParams& p, MirrorState* st,
	                       signed char side, float cosa,
	                       int* nbumps_out = nullptr) {
		Vec3 vel = st->vel;
		Vec3 pos = st->pos;
		const float s2d = Len2D(vel);
		const float h = s2d > 1.f ? atan2f(vel.Y, vel.X) : 0.f;
		float yaw = h * 57.2957795f;
		float fm = 0.f, sm = 0.f;
		if (side != 0 && s2d > 1.f)
			Air::WishInputs(h, static_cast<int>(side), cosa, &yaw,
				&fm, &sm);
		// StartGravity half (gravity_scale 1, basevel zero domain)
		vel.Z -= p.gravity * 0.5f * p.dt;
		CapAir::KernelCheckVelocity(p, &vel.X, &vel.Y, &vel.Z);
		// AirMove accel under the CARRIED sf
		float wx, wy;
		Fn::WishFromInput(yaw, fm, sm, &wx, &wy);
		float wishspeed = sqrtf(wx * wx + wy * wy);
		Vec3 wishdir(0.f, 0.f, 0.f);
		if (wishspeed > 1e-6f)
			wishdir = Vec3(wx / wishspeed, wy / wishspeed, 0.f);
		const float effmax = p.maxspeed;
		if (wishspeed > effmax)
			wishspeed = effmax;
		const float wishspd = (wishspeed > p.air_speed_cap)
			? p.air_speed_cap : wishspeed;
		const float cur = Dot(vel, wishdir);
		const float add = wishspd - cur;
		if (add > 0.f) {
			float accelspeed = p.airaccelerate * wishspeed * p.dt
				* st->sf;
			if (accelspeed > add)
				accelspeed = add;
			vel = vel + Scale(wishdir, accelspeed);
		}
		// TryPlayerMove via the certified local mirror
		if (!CapEdge::TryMoveLocal(S, p, false, &pos, &vel,
			nbumps_out))
			return false;   // corridor DECLINE
		// CategorizePosition: state-only in this domain; vz HERE
		// decides next tick's stale sf. The EXACT rule
		// (SolverMove.cpp 178-229): every categorize RESETS sf to 1;
		// the down-probe runs only when vz <= non_jump_velocity; a
		// probe with no walkable plane under a RISING player sets
		// sf = air_friction_up.
		const float vz_cat = vel.Z;
		CapAir::KernelCheckVelocity(p, &vel.X, &vel.Y, &vel.Z);
		vel.Z -= p.gravity * 0.5f * p.dt;
		CapAir::KernelCheckVelocity(p, &vel.X, &vel.Y, &vel.Z);
		st->pos = pos;
		st->vel = vel;
		st->sf = (vz_cat <= p.non_jump_velocity && vz_cat > 0.f)
			? p.air_friction_up : 1.f;
		return true;
	}

} // namespace CapRide

// ======================================================================
// CAPTRIGGER - registry A31 (trigger interactions), session 37. The
// engine mirror's trigger application (MoveTick's post-move block,
// total-parity port 2026-08-15) packaged as ONE pure transform so
// planners can apply a touch without running a tick. The three rules,
// verbatim: gravity touch overwrites the persistent gravity scale;
// a push accumulates onto base velocity (upward pushes unground and
// nudge the origin 1u up); a teleport sets the origin and ALWAYS
// zeroes velocity. The carried state then flows through the already-
// certified pieces - the air kernel's ctx takes (basevel,
// gravity_scale), and the basevel.Z integrate-and-clear is the
// airborne chain's first act.
namespace CapTrigger {

	inline void ApplyHit(const World::TriggerHitS& th, Vec3* pos,
	                     Vec3* vel, Vec3* basevel, bool* basevel_flag,
	                     float* gravity_scale, bool* on_ground,
	                     int* ground_brush, bool* teleported_out) {
		if (th.grav_touched)
			*gravity_scale = th.gravity;
		if (th.pushed) {
			Vec3 push = th.push_vec;
			if (*basevel_flag)
				push = push + *basevel;
			if (push.Z > 0.f && *on_ground) {
				*on_ground = false;
				*ground_brush = -1;
				pos->Z += 1.f;
			}
			*basevel = push;
			*basevel_flag = true;
		}
		if (th.teleported) {
			*pos = th.tp_origin;
			*vel = Vec3();
			if (teleported_out)
				*teleported_out = true;
		}
	}

} // namespace CapTrigger

// ======================================================================
// CAPRIDEREACH addition - registry A23 (ride edge interception),
// session 35: the earliest ride tick at which each sample point
// along an in-plane edge segment becomes reachable at the lattice
// contract - the A21 minimum-time query batched along the edge
// line. Exact crossing mechanics (actually leaving the face) belong
// to the A24/A26 composition; this row answers WHERE and WHEN the
// edge comes into reach.
namespace CapRideReach {

	inline int EdgeIntercept(const RRSurface& S, float eu0, float ev0,
	                         float eu1, float ev1, int samples,
	                         float contract, int* n_out, int* node_out,
	                         float* res_out) {
		int answered = 0;
		for (int i = 0; i < samples; ++i) {
			const float t = samples > 1
				? static_cast<float>(i)
					/ static_cast<float>(samples - 1) : 0.f;
			const float tu = eu0 + (eu1 - eu0) * t;
			const float tv = ev0 + (ev1 - ev0) * t;
			int node = -1;
			float res = 1e30f;
			const int N = MinRideTime(S, tu, tv, contract, &node,
				&res);
			n_out[i] = N;
			node_out[i] = node;
			res_out[i] = res;
			if (N > 0)
				answered++;
		}
		return answered;
	}

} // namespace CapRideReach
} // namespace Solver
