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

#include <map>
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
} // namespace Solver
