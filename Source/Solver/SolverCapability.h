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
	                         signed char side, float cosa, Vec3* out) {
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
		Vec3 clipped;
		Fn::ClipVelocity(vel, n, &clipped);
		vel = clipped;
		vel = vel - bv;
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
		int rollouts = 0;            // exact rollouts spent
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
		for (int ss = -1; ss <= 1; ss += 2) {
			int revs[3] = { 0, 0, 0 };
			offer(eval(ss, revs, 0, 0, 0, 0, 0.f, 0.f), ss, revs,
				0, 0, 0);
			for (int r1 = 1; r1 < N; r1 += 3) {
				revs[0] = r1;
				offer(eval(ss, revs, 1, 0, 0, 0, 0.f, 0.f), ss,
					revs, 1, 0, 0);
			}
			for (int r1 = 1; r1 < N; r1 += 6) {
				revs[0] = r1;
				for (int r2 = r1 + 6; r2 < N; r2 += 6) {
					revs[1] = r2;
					offer(eval(ss, revs, 2, 0, 0, 0, 0.f, 0.f),
						ss, revs, 2, 0, 0);
					for (int r3 = r2 + 6; r3 < N; r3 += 12) {
						revs[2] = r3;
						offer(eval(ss, revs, 3, 0, 0, 0, 0.f,
							0.f), ss, revs, 3, 0, 0);
					}
				}
			}
			// the brake probes (range control): straight-line and
			// one-reversal shapes with a brake run at several
			// start positions (short flights need the variety)
			const int bss[3] = { 1, N / 4, N / 2 };
			for (int bi = 0; bi < 3; ++bi)
				for (int bl = N / 8; bl <= (3 * N) / 4;
					bl += N / 8) {
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
			float cur = c.res;
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

} // namespace CapP2P
} // namespace Solver
