#pragma once

// THE EXIT WITNESS FRONTIER (ExitField stage 3; design of record
// Docs/ExitFieldSpec.md, advisor ruling 2026-08-19h).
//
// THE OPERATOR IS NOT THE FRONTIER. The mathematical ExitField remains
// the complete legal relation R_F(B). Stage 3 constructs a WITNESSED
// LOWER APPROXIMATION
//
//     W_F(B)  SUBSET OF  R_F(B)
//
// plus unresolved/deferred partitions - the set-valued analogue of
// EntranceField's L. Stage 4 adds the certified outer approximation
// U_F(B, M). Absence from the witnessed frontier NEVER acquires
// unreachable semantics: "the frontier has no transition here" and "no
// transition exists here" are different statements, and only certified
// bounds may make the second one.
//
// COMPRESSION IS RESOLUTION ONLY. It controls how many representatives
// are actively materialized, never which physical transitions are
// declared possible. A partition carries enough deterministic
// provenance (family/index ranges evaluated, refine level, caps hit)
// to reopen and regenerate anything it deferred:
//
//     compressed  != dominated
//     not active  != impossible
//
// Those distinctions live in STATUS ENUMS, not comments.
//
// PROPOSALS ORDER, THE ENGINE DECIDES. The generator below enumerates
// canonical search-coordinate schedules deterministically; every
// retained member is an exact packet-replayed Ride::ExitTransition.
// Carve / Steer::Controller / Field::ExitMap may later join as
// proposal sources under the same rule - and any adapter over legacy
// Carve MUST disable its ExitDoomed hard termination: an uncertified
// cull may influence priority, never proposal coverage (standing law
// B-A sharpened, advisor 2026-08-19h).

#include <vector>

#include "SolverRide.h"

namespace Solver {
namespace ExitField {

	// Partition status. There is deliberately no "dead" or
	// "impossible" value here - that is what certified bounds are for.
	enum PartStatus {
		kUnexplored = 0,       // no proposal has landed here yet
		kActive = 1,           // materialized representatives below
		kCompressed = 2,       // known members were deferred by caps
	};

	// One event-typed partition of the witnessed frontier. Keys are
	// COARSE INITIAL RESOLUTION SETTINGS, not physics: AIR_EXIT
	// partitions by exit heading/duration/vz/duck class,
	// CONTACT_TRANSFER by target face and duration, END and GROUND by
	// kind alone. Splitting/reopening is refinement's job.
	struct Partition {
		unsigned key = 0;
		int  kind = Ride::kHorizon;
		int  status = kUnexplored;
		// GROUND continuations are real, exact and PRESERVED, but no
		// Ground operator exists yet: the global layer must not prove
		// a route irrelevant because its next operator is unbuilt.
		bool continuation_unsupported = false;
		// Diversity slots (no single local objective exists): earliest
		// exit, fastest, highest vz, most energetic. The active cap
		// grows with refinement; displaced or capped members become
		// DEFERRED HASHES - known, reopenable, never erased.
		std::vector<Ride::ExitTransition> active;
		std::vector<unsigned long long> deferred_hash;
		// Reopen provenance: the deterministic proposal index range
		// this partition has seen, and whether members were omitted
		// solely because of the active cap.
		int  seen_lo = 1 << 30, seen_hi = -1;
		bool omitted_by_cap = false;
	};

	// PROVENANCE VERSIONING (advisor 2026-08-19i): "MakeProposal is a
	// pure function" is sufficient within one immutable configuration,
	// not as persistent provenance - if the proposal vocabulary changes,
	// index 412 no longer means what it meant when a deferred hash was
	// recorded. Every frontier binds (proposal-version, config hash,
	// physics/model versions); a deferred hash is regenerable only
	// under the same provenance, otherwise it requires migration by
	// replay. Same lesson the witness bank learned.
	constexpr unsigned kProposalVersion = 1;

	inline int ProposalCount(int level);

	inline unsigned long long ProvenanceHash(const MoveParams& p) {
		unsigned long long h = 1469598103934665603ULL;
		auto mix = [&](unsigned long long v) {
			h ^= v;
			h *= 1099511628211ULL;
		};
		mix(kProposalVersion);
		auto mixs = [&](const char* s) {
			for (; *s; ++s)
				mix(static_cast<unsigned long long>(
					static_cast<unsigned char>(*s)));
		};
		mixs(kEngineModelVersion);
		mixs(kControlLawVersion);
		mixs(kCollisionLawVersion);
		unsigned u = 0;
		memcpy(&u, &p.air_speed_cap, 4);
		mix(u);
		memcpy(&u, &p.strafe_rate_max, 4);
		mix(u);
		memcpy(&u, &p.dt, 4);
		mix(u);
		mix(static_cast<unsigned long long>(ProposalCount(0)));
		mix(static_cast<unsigned long long>(ProposalCount(1)));
		mix(static_cast<unsigned long long>(ProposalCount(2)));
		return h;
	}

	struct WitnessFrontier {
		Ride::BoundaryState B;
		unsigned prov_version = 0;      // kProposalVersion at build
		unsigned long long prov_hash = 0;  // full provenance identity
		int  level = 0;            // refinement level executed
		int  proposals = 0;        // deterministic proposal count run
		int  illegal = 0;          // refused by the dwell law
		int  horizon = 0;          // rides that outran their schedule
		int  known = 0;            // distinct legal transitions ever seen
		std::vector<Partition> parts;
		// every witness hash ever seen (dedupe + the monotone-knowledge
		// invariant F3: refinement may recompress, never forget)
		std::vector<unsigned long long> seen_hash;
	};

	// FNV over the exact packet bytes: the witness IS the packets, so
	// the hash is over the executable proof object itself.
	inline unsigned long long WitnessHash(
		const std::vector<Ride::MoveInput>& pk) {
		unsigned long long h = 1469598103934665603ULL;
		for (const Ride::MoveInput& mi : pk) {
			const unsigned char* b =
				reinterpret_cast<const unsigned char*>(&mi);
			for (size_t i = 0; i < sizeof(Ride::MoveInput); ++i) {
				h ^= b[i];
				h *= 1099511628211ULL;
			}
		}
		return h;
	}

	// Face-local 2D frame for spatial projections: u = the in-plane
	// horizontal-ish axis, v = the in-plane downhill-ish axis. Stable
	// per face; the frame is a projection convention, not physics.
	inline void FaceFrame(const Route::Face& fc, Vec3* u, Vec3* v) {
		Vec3 a = fabsf(fc.n.Z) < 0.9f ? Cross(fc.n, Vec3(0.f, 0.f, 1.f))
			: Cross(fc.n, Vec3(1.f, 0.f, 0.f));
		const float al = Len(a);
		*u = al > 1e-6f ? Scale(a, 1.f / al) : Vec3(1.f, 0.f, 0.f);
		*v = Cross(fc.n, *u);
	}

	// The event-typed partition key (coarse initial projections).
	// POSITION IS FUTURE-RELEVANT (advisor 2026-08-19i): two exits with
	// identical heading/duration/vz from opposite ends of a 1000u ramp
	// have completely different next-face reachability, and without
	// spatial structure the global solver has no principled subdomain
	// to request when it wants the omitted continuation. AIR_EXIT keys
	// carry the exit region in the RIDDEN face's local frame;
	// CONTACT_TRANSFER keys carry the contact region in the TARGET
	// face's frame. The 128u cell is a coarse initial RESOLUTION
	// setting; finer spatial splitting belongs to later refinement.
	inline unsigned RegionBits(const Route::Graph& g, int face,
	                           const Vec3& pos) {
		if (face < 0 || face >= static_cast<int>(g.faces.size()))
			return 0xff;
		const Route::Face& fc = g.faces[static_cast<size_t>(face)];
		Vec3 u, v;
		FaceFrame(fc, &u, &v);
		const Vec3 r = pos - fc.centroid;
		int iu = static_cast<int>(floorf(Dot(r, u) / 128.f)) + 8;
		int iv = static_cast<int>(floorf(Dot(r, v) / 128.f)) + 8;
		if (iu < 0) iu = 0;
		if (iu > 15) iu = 15;
		if (iv < 0) iv = 0;
		if (iv > 15) iv = 15;
		return static_cast<unsigned>((iu << 4) | iv);
	}

	inline unsigned PartKey(const Ride::ExitTransition& t,
	                        const Route::Graph& g, int ridden_face) {
		const unsigned kindb = static_cast<unsigned>(t.kind) << 28;
		if (t.kind == Ride::kAirExit) {
			const float th = atan2f(t.s_plus.vel.Y, t.s_plus.vel.X);
			const int thb = static_cast<int>((th + 3.14159265f)
				/ 0.5235988f);           // 12 x 30 degrees
			const int dtb = t.dt <= 20 ? 0 : (t.dt <= 60 ? 1 : 2);
			const int vzb = t.s_plus.vel.Z >= 0.f ? 1 : 0;
			const int dkb = (t.s_plus.ducked || t.s_plus.ducking)
				? 1 : 0;
			const unsigned reg = RegionBits(g, ridden_face,
				t.exit_pos);
			return kindb | (reg << 16)
				| (static_cast<unsigned>(thb & 15) << 8)
				| (static_cast<unsigned>(dtb) << 4)
				| (static_cast<unsigned>(vzb) << 1)
				| static_cast<unsigned>(dkb);
		}
		if (t.kind == Ride::kContactTransfer) {
			const int dtb = t.dt <= 20 ? 0 : (t.dt <= 60 ? 1 : 2);
			// Contact region in the TARGET face's frame (falls back to
			// the ridden face when the contact is not a graph face).
			const unsigned reg = RegionBits(g,
				t.board_face >= 0 ? t.board_face : ridden_face,
				t.s_plus.pos);
			return kindb | (reg << 16)
				| ((static_cast<unsigned>(t.board_face + 1) & 0xffu)
					<< 8)
				| (static_cast<unsigned>(dtb) << 4);
		}
		return kindb;   // END / GROUND: one partition per kind
	}

	// Active-representative cap per refinement level: a RESOLUTION
	// setting, never a physical filter.
	inline int ActiveCap(int level) {
		const int c = 2 + 2 * level;
		return c > 12 ? 12 : c;
	}

	// Diversity criteria within a partition (no single local
	// objective): earliest exit / fastest / highest vz / max energy.
	inline float SlotScore(const Ride::ExitTransition& t, int slot) {
		switch (slot & 3) {
		case 0: return -static_cast<float>(t.dt);
		case 1: return Len2D(t.s_plus.vel);
		case 2: return t.s_plus.vel.Z;
		default: return Dot(t.s_plus.vel, t.s_plus.vel);
		}
	}

	// Insert one exact transition. Displaced or capped members become
	// deferred hashes: known, reopenable, never erased (F3/F4).
	inline void Insert(WitnessFrontier* wf, Ride::ExitTransition&& t,
	                   const std::vector<Ride::MoveInput>& pk,
	                   int prop_idx, const Route::Graph& g) {
		const unsigned long long h = WitnessHash(pk);
		bool seen = false;
		for (unsigned long long s : wf->seen_hash)
			if (s == h) {
				seen = true;
				break;
			}
		// A re-encounter is NOT a no-op: refinement re-enumerates the
		// deterministic families, and a previously deferred member must
		// be able to MATERIALIZE under the now-larger active cap (F5).
		// That regeneration IS the reopen provenance working.
		if (!seen) {
			wf->seen_hash.push_back(h);
			wf->known++;
		}
		const unsigned key = PartKey(t, g, wf->B.face);
		Partition* P = nullptr;
		for (Partition& q : wf->parts)
			if (q.key == key) {
				P = &q;
				break;
			}
		if (!P) {
			wf->parts.push_back(Partition());
			P = &wf->parts.back();
			P->key = key;
			P->kind = t.kind;
			P->continuation_unsupported = t.kind == Ride::kGround;
		}
		if (prop_idx < P->seen_lo) P->seen_lo = prop_idx;
		if (prop_idx > P->seen_hi) P->seen_hi = prop_idx;
		// Already active? (hash equality on the executable witness)
		for (const Ride::ExitTransition& a : P->active)
			if (WitnessHash(a.witness) == h)
				return;
		const int cap = ActiveCap(wf->level);
		// Does it win any diversity slot?
		bool placed = false;
		for (int slot = 0; slot < cap && !placed; ++slot) {
			if (slot >= static_cast<int>(P->active.size())) {
				P->active.push_back(t);
				placed = true;
				break;
			}
			if (SlotScore(t, slot)
				> SlotScore(P->active[static_cast<size_t>(slot)],
					slot)) {
				// The displaced member stays KNOWN - and a partition
				// holding ANY deferred member is COMPRESSED, whether
				// the deferral came from displacement or the cap:
				// compression means "not all known members are
				// active", nothing else.
				P->deferred_hash.push_back(WitnessHash(
					P->active[static_cast<size_t>(slot)].witness));
				P->status = kCompressed;
				P->active[static_cast<size_t>(slot)] = t;
				placed = true;
			}
		}
		if (placed) {
			// If this member was previously deferred, it is now
			// materialized: clear its deferred record. A partition
			// with no deferred members left is fully ACTIVE again.
			for (size_t i = 0; i < P->deferred_hash.size(); ++i)
				if (P->deferred_hash[i] == h) {
					P->deferred_hash.erase(
						P->deferred_hash.begin()
						+ static_cast<long long>(i));
					break;
				}
			if (P->deferred_hash.empty()
				|| P->status == kUnexplored)
				P->status = P->deferred_hash.empty() ? kActive
					: kCompressed;
		} else if (!seen) {
			P->deferred_hash.push_back(h);
			P->omitted_by_cap = true;
			P->status = kCompressed;
		}
	}

	// ---- THE DETERMINISTIC PROPOSAL FAMILIES (production search
	// coordinates; no oracle, no human data, no Carve, no ExitDoomed).
	// The enumeration is a pure function of (level, index), which IS
	// the reopen provenance: re-running a range regenerates exactly
	// the candidates it generated before.
	struct Proposal {
		std::vector<signed char> side;
		std::vector<float> cosa;
		std::vector<unsigned char> duck;
	};

	inline int ProposalCount(int level) {
		return level <= 0 ? 72 : (level == 1 ? 132 : 204);
	}

	inline void MakeProposal(int idx, Proposal* out) {
		out->side.clear();
		out->cosa.clear();
		out->duck.clear();
		// Family A [0, 36): constant side x cosa x horizon.
		if (idx < 36) {
			const int sd = (idx & 1) ? -1 : 1;
			const float ca = (idx / 2) % 3 == 0 ? -0.02f
				: ((idx / 2) % 3 == 1 ? 0.3f : 0.85f);
			const int M = (idx / 6) % 3 == 0 ? 40
				: ((idx / 6) % 3 == 1 ? 120 : 240);
			const int dko = idx / 18;   // 0 none, 1 duck-hold late
			out->side.assign(static_cast<size_t>(M),
				static_cast<signed char>(sd));
			out->cosa.assign(static_cast<size_t>(M), ca);
			out->duck.assign(static_cast<size_t>(M), 0);
			if (dko)
				for (int k = M * 2 / 3; k < M; ++k)
					out->duck[static_cast<size_t>(k)] = 1;
			return;
		}
		// Family B [36, 72): one reversal at r, both orders, two cosa.
		if (idx < 72) {
			const int j = idx - 36;
			const int sd = (j & 1) ? -1 : 1;
			const int r = 8 + 8 * ((j / 2) % 3);      // 8 / 16 / 24
			const float ca = (j / 6) % 2 ? 0.3f : -0.02f;
			const int M = j < 18 ? 120 : 240;
			out->side.assign(static_cast<size_t>(M),
				static_cast<signed char>(sd));
			for (int k = r; k < M; ++k)
				out->side[static_cast<size_t>(k)] =
					static_cast<signed char>(-sd);
			out->cosa.assign(static_cast<size_t>(M), ca);
			out->duck.assign(static_cast<size_t>(M), 0);
			return;
		}
		// Family C [72, 132) (level >= 1): coast prefix then gain, and
		// early duck-offs.
		if (idx < 132) {
			const int j = idx - 72;
			const int sd = (j & 1) ? -1 : 1;
			const int c = 4 + 4 * ((j / 2) % 3);      // coast 4/8/12
			const int M = 60 + 60 * ((j / 6) % 2);
			const bool duckoff = (j / 12) % 2 != 0;
			out->side.assign(static_cast<size_t>(M),
				static_cast<signed char>(sd));
			for (int k = 0; k < c; ++k)
				out->side[static_cast<size_t>(k)] = 0;
			out->cosa.assign(static_cast<size_t>(M), -0.02f);
			out->duck.assign(static_cast<size_t>(M), 0);
			if (duckoff)
				for (int k = c + 6; k < M; ++k)
					out->duck[static_cast<size_t>(k)] = 1;
			return;
		}
		// Family D [132, 204) (level >= 2): finer cosa ladder + late
		// reversals.
		{
			const int j = idx - 132;
			const int sd = (j & 1) ? -1 : 1;
			const float ca = -0.1f + 0.15f * ((j / 2) % 6);
			const int r = 32 + 16 * ((j / 12) % 3);
			const int M = 240;
			out->side.assign(static_cast<size_t>(M),
				static_cast<signed char>(sd));
			if ((j / 36) % 2)
				for (int k = r; k < M; ++k)
					out->side[static_cast<size_t>(k)] =
						static_cast<signed char>(-sd);
			out->cosa.assign(static_cast<size_t>(M), ca);
			out->duck.assign(static_cast<size_t>(M), 0);
		}
	}

	// Build / refine the witnessed frontier. Deterministic: the same
	// (B, level) always yields the same frontier. Refinement re-runs
	// the deterministic enumeration with a higher level: previously
	// known transitions dedupe by hash, new ones join, the active cap
	// grows - so previously deferred members can MATERIALIZE and
	// nothing is ever forgotten (F3/F5).
	inline void BuildFrontier(WitnessFrontier* wf, const World& w,
	                          const MoveParams& p,
	                          const Route::Graph& g, int level,
	                          const Vec3* zone_min = nullptr,
	                          const Vec3* zone_max = nullptr) {
		if (level < wf->level)
			level = wf->level;
		wf->level = level;
		wf->prov_version = kProposalVersion;
		wf->prov_hash = ProvenanceHash(p);
		const int n = ProposalCount(level);
		for (int i = 0; i < n; ++i) {
			Proposal pr;
			MakeProposal(i, &pr);
			Ride::CanonSchedule(pr.side, &pr.cosa, nullptr);
			std::vector<Ride::MoveInput> pk;
			Ride::Result rr = Ride::FlyRideSchedule(wf->B, w, p, g,
				pr.side, pr.cosa, pr.duck, nullptr, nullptr, nullptr,
				&pk, zone_min, zone_max);
			wf->proposals++;
			if (!rr.legal) {
				wf->illegal++;
				continue;
			}
			if (rr.kind == Ride::kHorizon) {
				wf->horizon++;
				continue;   // UNRESOLVED, not a transition
			}
			Ride::ExitTransition tr;
			if (!Ride::MakeTransition(rr, g, pk, &tr))
				continue;
			Insert(wf, static_cast<Ride::ExitTransition&&>(tr), pk,
				i, g);
		}
	}

	// ================= STAGE 4: THE FIRST CERTIFIED OUTER-ENVELOPE
	// COMPONENT U_E(B, M) (advisor 2026-08-19i; gates E1-E8 in
	// `exitenv`) =================
	//
	// CERTIFIED STATEMENT (bound id kUEnergyBoundId):
	//     for all Z in R_F(B) with dt(Z) <= M:
	//         E_boundary(S+(Z)) <= UEnergy(B, M)
	// and for every intermediate booked tick n <= M of any legal ride.
	//
	// DOMAIN D_static-surf, scoped honestly: static world geometry,
	// constant gravity_scale through the ride, basevel == 0, no
	// triggers/boosters/moving brushes, ride inputs are wish + duck
	// only (no jump), every booked tick starts airborne (a grounded
	// tick ends the ride by definition). Features outside this domain
	// need the environment in the operator state and a new bound
	// version.
	//
	// PHASE FIRST (the B7 lesson): E is defined at the EXACT tick
	// boundary phase ExitTransition::s_plus stores - after
	// FinishGravity, position integrated. E := |v|^2 + 2 g gs z with z
	// the RAW origin; duck-origin bookkeeping is handled explicitly
	// below, never smuggled into the potential.
	//
	// THE ONE-TICK RIDE LEDGER, from the authoritative movement order
	// (StartGravity -> duck -> AirAccelerate -> TryPlayerMove ->
	// FinishGravity):
	//
	//  * GRAVITY/INTEGRATION: the half-tick leapfrog conserves E
	//    EXACTLY at boundaries. With G = g gs dt: vz" = vz - G and
	//    z' = z + (vz - G/2) dt, so d(vz^2) = -2 G vz + G^2 and
	//    d(2 g gs z) = 2 (G/dt)(vz - G/2) dt = 2 G vz - G^2. Sum = 0,
	//    to float rounding (measured by E1; the per-tick eps below).
	//  * WISH WORK (AirAccelerate): dE = 2 a p + a^2 with
	//    p = v.wishdir and, when addspeed > 0, the applied
	//    0 <= a <= addspeed = c - p where c = min(wishspd, cap) <= cap.
	//    (PROOF REPAIRED 2026-08-19j: the earlier intermediate step
	//    "dE <= c^2 - p^2" is INVALID for p < -c, where the convex
	//    f(a) = 2ap + a^2 can beat a negative c^2 - p^2 at a = 0.)
	//    The valid chain: a <= c - p gives p <= c - a, so
	//        dE = 2 a p + a^2 <= 2 a (c - a) + a^2
	//           = 2 a c - a^2 = c^2 - (a - c)^2 <= c^2 <= cap^2.
	//    When addspeed <= 0, a = 0 and dE = 0. So dE_wish <= cap^2
	//    = 900 PER TICK for EVERY legal wish - any wishspeed, any
	//    surface friction, any speed, braking included - the Air
	//    law's number DERIVED in the ride domain, not copied.
	//  * CLIPS (TryPlayerMove) - THE ANALYTICAL PREMISE (2026-08-19j):
	//    the general normal update v' = v - beta (v.n) n with unit n
	//    gives |v'|^2 - |v|^2 = beta (beta - 2) (v.n)^2, non-expansive
	//    for 0 <= beta <= 2. The authoritative ride-domain helper
	//    (Fn::ClipVelocity / EngineClipVelocity) uses overbounce
	//    beta = 1 exactly, so each clip contributes -(v.n)^2 <= 0;
	//    crease resolution is repeated beta = 1 projections and the
	//    allsolid path ZEROES velocity. E3's 2000 exact-helper probes
	//    falsify implementation drift; this premise is the proof.
	//  * DUCK ORIGIN: an in-air duck shifts the origin +duck_air_shift
	//    and an unduck -duck_air_shift WITHOUT physical work - raw-z
	//    potential jumps by +-2 g gs * 8.5 (~13.6k) per transition.
	//    Transitions alternate, so the PREFIX SUM of shifts is at most
	//    ONE net +8.5 whatever the schedule does: the certified slack
	//    is a single 2 g gs duck_air_shift term, NOT per tick (gate
	//    E4, and E8 verifies removing it turns the suite red).
	//
	// Summing over n <= M booked ticks:
	//     E(S_n) <= E(B) + cap^2 n + 2 g gs duck_air_shift + eps n.
	//
	// The horizon M is PART OF THE QUERY DOMAIN, not the refinement
	// profile: R(B, 40) is a subset of R(B, 80), so U may legitimately
	// grow with M (gate E7 pins monotonicity). Refinement for FIXED
	// (B, M) tightens; extending M is domain expansion and must be an
	// explicit ExtendHorizon, never a profile side effect.
	//
	// Constructive gates falsify this bound; the derivation above is
	// the proof.

	constexpr unsigned kUEnergyBoundId = 0x45580001u;
	// FLOATING-POINT SLACK, DERIVED NOT MEASURED (proof repair
	// 2026-08-19j: "we haven't seen a violation" != "there cannot be
	// one" - the B5 lesson applied to numerics). Conservative forward
	// error inventory over one tick, under the certified engine
	// magnitude clamps (each velocity component <= maxvelocity = 3500,
	// so |v|^2 <= 3*3500^2 = 36.75M; BSP coordinates |z| <= 16384, so
	// with gs <= 2 the potential term <= 52.4M and |E| < 2^27, where
	// one float ulp is 8). Each float op errs <= 0.5 ulp of its
	// result; a velocity-component error dv costs <= 2*3500*dv in E, a
	// z error dz costs <= 2*g*gs*dz <= 3200*dz:
	//   gravity halves      2 ops on vz     -> E-effect <  2
	//   wish accel          ~6 ops/comp x2  -> E-effect < 11
	//   clips, <= 4 bumps   ~10 ops/bump    -> E-effect < 240
	//   position integrate  2 ops on z      -> E-effect <  7
	//   E evaluation twice  ~9 ops at ulp 8 -> E-effect < 72
	// Inventory total < 332; certified with headroom at 512. Loose is
	// irrelevant against 900 legal wish work per tick; E1's observed
	// ~0.5/tick stays as evidence the certified slack is conservative,
	// never as its justification.
	constexpr float kUEnergyEpsTick = 512.0f;

	inline float EBoundary(const PlayerState& s, const MoveParams& p) {
		return Dot(s.vel, s.vel)
			+ 2.f * p.gravity * s.gravity_scale * s.pos.Z;
	}

	inline float UEnergy(const Ride::BoundaryState& B,
	                     const MoveParams& p, int M) {
		const float cap2 = p.air_speed_cap * p.air_speed_cap;
		const float hull = 2.f * p.gravity * B.ps.gravity_scale
			* p.duck_air_shift;
		return EBoundary(B.ps, p) + cap2 * static_cast<float>(M)
			+ hull + kUEnergyEpsTick * static_cast<float>(M);
	}

	// ================= STAGE 5: THE LAZY QUERY (advisor 2026-08-19j)
	// =================
	//
	// The physical query domain is FIXED: (exact B, ridden face, M,
	// world identity, movement/model versions). Refinement profiles
	// change proposal effort, partition resolution and active caps -
	// NEVER B, M, movement laws, event classification, control
	// legality or the world. Horizon extension is a separate
	// domain-expansion operation on a DIFFERENT key.
	//
	// W_known IS PARTITION-INDEPENDENT: the authoritative set of exact
	// witnessed transitions lives outside the partition layout, so
	// changing resolution is repartition(W_known) and CANNOT forget a
	// witness. W_materialized (the active representatives in the view)
	// is a derived resolution artifact and may recompress freely:
	//     W_known only grows;  W_materialized may churn.
	// The types make the distinction so no future monotonicity
	// assertion is written over the active-vector size by mistake.
	//
	// STATUSES ARE HONEST: possession of scalar U_E is ONE certified
	// projection of the outer envelope, not a characterization of
	// R_F(B, M) - so there is deliberately no RESOLVED/COMPLETE value.
	// Failure to discover exits remains UNRESOLVED unless a concrete
	// certified predicate proves irrelevance at the global layer.
	enum QueryStatus {
		kQUnexplored = 0,   // no profile has run
		kQUnresolved = 1,   // searched; the domain is not characterized
		kQRefined = 2,      // witnesses exist; still not complete
	};

	// Stable immutable profile identities (the EntranceField
	// discipline): a profile is an effort/resolution setting.
	constexpr unsigned kQProfileId[4] = {
		0x51460100u, 0x51460101u, 0x51460102u, 0x51460103u };

	// World identity for the domain key: brush count + per-brush
	// geometry bytes. Two queries merge knowledge only if they are the
	// same physical problem (the witness-bank semantic-key lesson).
	inline unsigned long long WorldIdent(const World& w) {
		unsigned long long h = 1469598103934665603ULL;
		auto mix = [&](const void* ptr, size_t n) {
			const unsigned char* b =
				static_cast<const unsigned char*>(ptr);
			for (size_t i = 0; i < n; ++i) {
				h ^= b[i];
				h *= 1099511628211ULL;
			}
		};
		const unsigned nb = static_cast<unsigned>(w.brushes.size());
		mix(&nb, sizeof(nb));
		for (const WorldBrush& b2 : w.brushes) {
			mix(&b2.nsides, sizeof(b2.nsides));
			mix(&b2.bmin, sizeof(b2.bmin));
			mix(&b2.bmax, sizeof(b2.bmax));
		}
		return h;
	}

	struct ExitQuery {
		// ---- THE FIXED PHYSICAL DOMAIN (immutable after Init)
		Ride::BoundaryState B;
		int M = 0;
		unsigned long long domain_key = 0;
		unsigned long long prov = 0;      // ProvenanceHash at Init
		float UE = 0.f;                   // certified U_E(B, M)
		// ---- AUTHORITATIVE KNOWLEDGE (partition-independent;
		// monotone: only ever appended)
		std::vector<Ride::ExitTransition> known;
		std::vector<unsigned long long> known_hash;
		int  horizon_runs = 0;            // UNRESOLVED evidence
		// ---- DERIVED VIEW (materialization; may recompress)
		std::vector<Partition> view;
		int  view_level = 0;
		// ---- audit
		int  status = kQUnexplored;
		std::vector<unsigned> trail;
	};

	inline unsigned long long QueryDomainKey(
		const Ride::BoundaryState& B, int M, const World& w,
		const MoveParams& p) {
		unsigned long long h = 1469598103934665603ULL;
		auto mix = [&](const void* ptr, size_t n) {
			const unsigned char* b2 =
				static_cast<const unsigned char*>(ptr);
			for (size_t i = 0; i < n; ++i) {
				h ^= b2[i];
				h *= 1099511628211ULL;
			}
		};
		mix(&B.ps, sizeof(PlayerState));
		mix(&B.ctl, sizeof(Steer::CtlState));
		mix(&B.face, sizeof(int));
		mix(&M, sizeof(int));
		const unsigned long long wid = WorldIdent(w);
		mix(&wid, sizeof(wid));
		const unsigned long long pv = ProvenanceHash(p);
		mix(&pv, sizeof(pv));
		return h;
	}

	// Deterministic rebin of the AUTHORITATIVE set into a fresh view
	// at the given resolution level. Placement is a pure function of
	// (known order, level), so the same knowledge rebins byte-
	// identically; a member not active is in deferred_hash - KNOWN,
	// never lost.
	inline void Rebin(ExitQuery* Q, const Route::Graph& g, int level) {
		Q->view.clear();
		Q->view_level = level;
		const int cap = ActiveCap(level);
		for (size_t i = 0; i < Q->known.size(); ++i) {
			const Ride::ExitTransition& t = Q->known[i];
			const unsigned key = PartKey(t, g, Q->B.face);
			Partition* P = nullptr;
			for (Partition& q2 : Q->view)
				if (q2.key == key) {
					P = &q2;
					break;
				}
			if (!P) {
				Q->view.push_back(Partition());
				P = &Q->view.back();
				P->key = key;
				P->kind = t.kind;
				P->continuation_unsupported =
					t.kind == Ride::kGround;
			}
			bool placed = false;
			for (int slot = 0; slot < cap && !placed; ++slot) {
				if (slot >= static_cast<int>(P->active.size())) {
					P->active.push_back(t);
					placed = true;
					break;
				}
				if (SlotScore(t, slot) > SlotScore(
					P->active[static_cast<size_t>(slot)], slot)) {
					P->deferred_hash.push_back(WitnessHash(
						P->active[static_cast<size_t>(
							slot)].witness));
					P->status = kCompressed;
					P->active[static_cast<size_t>(slot)] = t;
					placed = true;
				}
			}
			if (placed) {
				if (P->status == kUnexplored)
					P->status = kActive;
			} else {
				P->deferred_hash.push_back(Q->known_hash[i]);
				P->omitted_by_cap = true;
				P->status = kCompressed;
			}
		}
	}

	inline void QueryInit(ExitQuery* Q, const Ride::BoundaryState& B,
	                      int M, const World& w, const MoveParams& p) {
		Q->B = B;
		Q->M = M;
		Q->UE = UEnergy(B, p, M);
		Q->prov = ProvenanceHash(p);
		Q->domain_key = QueryDomainKey(B, M, w, p);
		Q->status = kQUnexplored;
	}

	// ONE refinement step: run the given profile on the FIXED domain
	// (schedules clamped to M - any transition with dt <= M found by a
	// legal schedule is in R(B, M)), append genuinely new transitions
	// to the authoritative set, rebin the view. REFUSES to run under
	// mismatched provenance: deferred indices from another
	// proposal/model version are never silently regenerated under new
	// semantics - that is an explicit migration.
	inline bool QueryRefine(ExitQuery* Q, const World& w,
	                        const MoveParams& p, const Route::Graph& g,
	                        int profile) {
		if (ProvenanceHash(p) != Q->prov)
			return false;
		if (profile < 0) profile = 0;
		if (profile > 3) profile = 3;
		const int n = ProposalCount(profile);
		for (int i = 0; i < n; ++i) {
			Proposal pr;
			MakeProposal(i, &pr);
			if (static_cast<int>(pr.side.size()) > Q->M) {
				pr.side.resize(static_cast<size_t>(Q->M));
				pr.cosa.resize(static_cast<size_t>(Q->M));
				pr.duck.resize(static_cast<size_t>(Q->M));
			}
			Ride::CanonSchedule(pr.side, &pr.cosa, nullptr);
			std::vector<Ride::MoveInput> pk;
			Ride::Result rr = Ride::FlyRideSchedule(Q->B, w, p, g,
				pr.side, pr.cosa, pr.duck, nullptr, nullptr,
				nullptr, &pk);
			if (!rr.legal)
				continue;
			if (rr.kind == Ride::kHorizon) {
				Q->horizon_runs++;
				continue;
			}
			Ride::ExitTransition tr;
			if (!Ride::MakeTransition(rr, g, pk, &tr))
				continue;
			if (tr.dt > Q->M)
				continue;   // outside the fixed domain
			const unsigned long long h = WitnessHash(tr.witness);
			bool have = false;
			for (unsigned long long s : Q->known_hash)
				if (s == h) {
					have = true;
					break;
				}
			if (!have) {
				Q->known.push_back(tr);
				Q->known_hash.push_back(h);
			}
		}
		Rebin(Q, g, profile);
		Q->status = Q->known.empty() ? kQUnresolved : kQRefined;
		Q->trail.push_back(kQProfileId[static_cast<size_t>(profile)]);
		return true;
	}

} // namespace ExitField
} // namespace Solver
