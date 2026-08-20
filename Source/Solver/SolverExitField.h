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

	struct WitnessFrontier {
		Ride::BoundaryState B;
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

	// The event-typed partition key (coarse initial projections).
	inline unsigned PartKey(const Ride::ExitTransition& t) {
		const unsigned kindb = static_cast<unsigned>(t.kind) << 28;
		if (t.kind == Ride::kAirExit) {
			const float th = atan2f(t.s_plus.vel.Y, t.s_plus.vel.X);
			const int thb = static_cast<int>((th + 3.14159265f)
				/ 0.5235988f);           // 12 x 30 degrees
			const int dtb = t.dt <= 20 ? 0 : (t.dt <= 60 ? 1 : 2);
			const int vzb = t.s_plus.vel.Z >= 0.f ? 1 : 0;
			const int dkb = (t.s_plus.ducked || t.s_plus.ducking)
				? 1 : 0;
			return kindb | (static_cast<unsigned>(thb & 15) << 8)
				| (static_cast<unsigned>(dtb) << 4)
				| (static_cast<unsigned>(vzb) << 1)
				| static_cast<unsigned>(dkb);
		}
		if (t.kind == Ride::kContactTransfer) {
			const int dtb = t.dt <= 20 ? 0 : (t.dt <= 60 ? 1 : 2);
			return kindb
				| (static_cast<unsigned>(t.board_face + 1) << 8)
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
	                   int prop_idx) {
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
		const unsigned key = PartKey(t);
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
				i);
		}
	}

} // namespace ExitField
} // namespace Solver
