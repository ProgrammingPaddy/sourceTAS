#pragma once

// THE GLOBAL LAYER, G0 (advisor ruling 2026-08-20; design of record
// Docs/ExitFieldSpec.md session 17). This is the point where the
// project stops being operator development and becomes the full-map
// solver: a best-first explorer over exact BoardBoundaryState nodes
// connected by exact witnessed TransferEdges.
//
// THE CENTRAL LAW: a discovered edge is exact and witnessed; the
// successor set is NOT complete. Absence from a node's edge list never
// means "no such edge exists" - it means "not currently witnessed".
// Node expansion never implies successor completeness; the type
// carries the law (`succ_complete` exists and is ALWAYS false in v0 -
// nothing in this build may set it).
//
// GLOBAL COST IS TICKS. g = elapsed ticks from the start; the ONLY
// objective is min finish tick. Energy, exit height, heading quality,
// ExitMap scores are state variables and ORDERING advice - they never
// enter the objective and never admit or reject anything. h_order
// (advisory) and h_cert (certified, = 0 in G0) are SEPARATE variables;
// only g + h_cert may hard-prune, and every hard prune emits a proof
// record. The one certified elimination available in G0 is EXACT
// duplicate-state dominance: identical canonical board state reached
// at an earlier g dominates the later arrival (static-world domain:
// future physics is time-invariant). Hash equality over the exact
// state bytes, never approximate clustering.
//
// TWO KINDS OF WORK: traversing discovered edges (state work) and
// spending more local search on a node's unresolved successor space
// (refinement work). Refining an already-expanded node can discover
// new edges and enqueue new children - that is the lazy architecture
// operating at the top level, where the scheduler concepts finally
// belong.
//
// COMPETITIVE HORIZONS: before an incumbent exists, ride horizons are
// explicit finite domains (M0 = 240). Once a production finish sets
// T*, a node reached at g cannot spend dt >= T* - g and still improve
// the incumbent, so ExitQuery(B, min(M0, T* - g - 1)) is a PHYSICAL
// consequence of the objective, not a compute budget. T* is never
// seeded from human runs (the deletion invariant); only a
// production-generated exact finish establishes or improves it.

#include <vector>

#include "SolverEntrance.h"
#include "SolverExitField.h"
#include "SolverRide.h"

namespace Solver {
namespace GlobalSearch {

	// Edge kinds mirror the event typing that produces them.
	enum EdgeKind {
		kEdgeAir = 0,        // Exit -> Air -> Entrance (board on face_j)
		kEdgeContact = 1,    // direct CONTACT_TRANSFER (no Air segment)
		kEdgeEnd = 2,        // Exit -> END (the finish; graph sink)
	};

	// THE PRODUCTION EDGE CONTRACT: everything needed to cold-replay
	//     ReplayEdge(B_from, e) == (B_to, dt)
	// with no hidden search state. AIR edges carry ride packets + the
	// canonical Air schedule; contact/END edges carry ride packets
	// only. Provenance stamps the proposal/model identity the segments
	// were generated under.
	struct TransferEdge {
		int  kind = kEdgeAir;
		unsigned long long from_hash = 0;
		unsigned long long to_hash = 0;   // 0 for END
		int  dt = 0;                      // exact ticks, = dt_exit + dt_air
		int  dt_exit = 0;
		int  dt_air = 0;                  // 0 for contact/END
		std::vector<Ride::MoveInput> ride;
		std::vector<signed char> air_side;
		std::vector<float> air_cosa;
		int  air_horizon = 0;
		int  face_j = -1;                 // -1 for END
		Ride::BoundaryState Bj;           // for END: the finish state
		unsigned long long prov = 0;
	};

	// Exact canonical state identity: the complete boundary payload,
	// by bytes. Two nodes merge ONLY if they are the same physical
	// state (the domain-key discipline applied globally).
	inline unsigned long long NodeHash(const Ride::BoundaryState& B) {
		unsigned long long h = 1469598103934665603ULL;
		auto mix = [&](const void* ptr, size_t n) {
			const unsigned char* b =
				static_cast<const unsigned char*>(ptr);
			for (size_t i = 0; i < n; ++i) {
				h ^= b[i];
				h *= 1099511628211ULL;
			}
		};
		mix(&B.ps, sizeof(PlayerState));
		mix(&B.ctl, sizeof(Steer::CtlState));
		mix(&B.face, sizeof(int));
		return h;
	}

	// SESSION 18 (advisor 2026-08-20b): STABLE UNRESOLVED-DOMAIN
	// IDENTITY. `succ_complete == false` says only "more may exist";
	// eventual proof accounting needs to know WHICH search domains were
	// attempted, at what resolution, and why each produced nothing -
	// every unresolved domain must one day be refined or proven unable
	// to beat the incumbent. One record per domain per node; repeat
	// attempts UPDATE the record (level / M / outcome evolve), the
	// identity never changes. `kDomWitnessed` means "this domain has
	// produced at least one exact edge" - never completeness. There is
	// deliberately no "impossible" status.
	enum DomainKind {
		kDomExit = 0,       // the node's ExitQuery at a profile level
		kDomEntrance = 1,   // exit witness -> Air -> board on face_j
		kDomEnd = 2,        // exit witness -> zone probe (END-via-air)
		kDomLaunch = 3,     // a launch direction (root construction)
	};
	enum DomainStatus {
		kDomUnexplored = 0,
		kDomUnresolved = 1, // attempted; no edge; NEVER "unreachable"
		kDomWitnessed = 2,  // >= 1 exact edge produced
	};
	// Reason code of the LAST attempt (diagnosis, not judgment).
	enum DomainOutcome {
		kOutNone = 0,
		kOutEdge = 1,           // produced an exact edge
		kOutNoExitWitness = 2,  // the ExitQuery has no active witness yet
		kOutNoStrike = 3,       // Entrance found no board (UNRESOLVED)
		kOutSchedIllegal = 4,   // control seam refused the handoff
		kOutReplayReject = 5,   // authoritative replay voided the witness
		kOutHorizonDead = 6,    // competitive horizon left no room
		kOutZoneMiss = 7,       // probe contacted geometry short of zone
		kOutNotAirborne = 8,    // launch run never left the ground
	};
	struct DomainAudit {
		unsigned long long id = 0;  // stable FNV over the identity
		int kind = 0;               // DomainKind
		int face_j = -1;            // entrance target face (-1 = n/a)
		int variant = 0;            // aim / schedule / direction index
		int level = -1;             // deepest local profile attempted
		int M = 0;                  // horizon at the last attempt
		int status = kDomUnexplored;
		int outcome = kOutNone;
		int attempts = 0;
		int edges_found = 0;        // cumulative exact edges
	};
	inline unsigned long long DomainId(unsigned long long owner_hash,
	                                   int kind, int face_j,
	                                   int variant) {
		unsigned long long h = 1469598103934665603ULL;
		auto mix = [&](unsigned long long v) {
			for (int i = 0; i < 8; ++i) {
				h ^= (v >> (i * 8)) & 0xFF;
				h *= 1099511628211ULL;
			}
		};
		mix(owner_hash);
		mix(static_cast<unsigned long long>(kind));
		mix(static_cast<unsigned long long>(
			static_cast<long long>(face_j)));
		mix(static_cast<unsigned long long>(
			static_cast<long long>(variant)));
		return h;
	}
	inline DomainAudit* TouchDomain(std::vector<DomainAudit>* a,
	                                unsigned long long owner_hash,
	                                int kind, int face_j, int variant) {
		const unsigned long long id = DomainId(owner_hash, kind,
			face_j, variant);
		for (size_t i = 0; i < a->size(); ++i)
			if ((*a)[i].id == id)
				return &(*a)[i];
		DomainAudit d;
		d.id = id;
		d.kind = kind;
		d.face_j = face_j;
		d.variant = variant;
		a->push_back(d);
		return &a->back();
	}

	struct Node {
		Ride::BoundaryState B;
		unsigned long long hash = 0;
		int  g = 0;                 // exact elapsed ticks from START
		int  refine_level = 0;      // local profiles consumed so far
		int  parent = -1;           // node index (route reconstruction)
		int  parent_edge = -1;      // edge index into Graph::edges
		std::vector<int> edges;     // discovered outgoing edges
		// ALWAYS false in v0: nothing may set it. The field exists so
		// the law "expansion != successor completeness" lives in the
		// type, not in a comment.
		bool succ_complete = false;
		// The domain ledger (session 18): what has been tried here,
		// at what resolution, with what outcome. The granular form of
		// "successors are not complete".
		std::vector<DomainAudit> audit;
	};

	// Certified eliminations only, each carrying its proof.
	struct PruneProof {
		unsigned long long state_hash = 0;
		int  g_pruned = 0;          // the later arrival's g
		int  g_kept = 0;            // the dominating earlier g
		unsigned bound_id = 0;      // 0x474C0001 = exact-dup dominance
		int  h_cert = 0;
		int  incumbent = -1;        // T* at prune time (-1 = none)
	};

	// One accepted incumbent, in discovery order (session 18: the
	// full history is a first-class measurement - T_1 > T_2 > ... >
	// T*, and whether the positive feedback loop actually happens).
	struct FinishEvent {
		int Tf = -1;
		int node = -1;
		int edge = -1;
	};

	struct Graph {
		std::vector<Node> nodes;
		std::vector<TransferEdge> edges;
		std::vector<PruneProof> prunes;
		int  Tstar = -1;            // incumbent finish tick (-1 = inf)
		int  Tstar_node = -1;       // node whose END edge finished
		int  Tstar_edge = -1;
		int  incumbent_updates = 0; // G8 observability
		int  m_explicit = 240;      // pre-incumbent horizon domain
		std::vector<FinishEvent> history;  // accepted incumbents
		int  finish_offers = 0;     // every OfferFinish call
		// Topology falsification (session 18): a realized edge whose
		// face_j was NOT in the conservative candidate set is a
		// topology correctness FAILURE, not a bonus edge. Nonzero here
		// means the topology is unsafe.
		int  topo_violations = 0;
	};

	// Offer a production finish to the incumbent: the first
	// establishes T*, a strictly faster witness replaces it, anything
	// else changes nothing. The ONE code path every END edge goes
	// through (ExpandNode and any future assembler both call here).
	inline bool OfferFinish(Graph* G, int node, int edge, int Tf) {
		G->finish_offers++;
		if (G->Tstar < 0 || Tf < G->Tstar) {
			G->Tstar = Tf;
			G->Tstar_node = node;
			G->Tstar_edge = edge;
			G->incumbent_updates++;
			FinishEvent fe;
			fe.Tf = Tf;
			fe.node = node;
			fe.edge = edge;
			G->history.push_back(fe);
			return true;
		}
		return false;
	}

	// The competitive local horizon (a physical restriction, not a
	// budget): with an incumbent, spending dt >= T* - g cannot improve
	// it. Before an incumbent: the explicit finite domain.
	inline int HorizonFor(const Graph& G, int g) {
		if (G.Tstar < 0)
			return G.m_explicit;
		int m = G.Tstar - g - 1;
		if (m > G.m_explicit)
			m = G.m_explicit;
		return m < 0 ? 0 : m;
	}

	// Insert a node with EXACT duplicate-state dominance: the same
	// canonical state at an earlier g dominates (static-world theorem;
	// hash equality, never clustering). Returns the node index, or -1
	// with a proof record when the arrival is dominated.
	inline int AddNode(Graph* G, const Ride::BoundaryState& B, int g,
	                   int parent, int parent_edge) {
		const unsigned long long h = NodeHash(B);
		for (size_t i = 0; i < G->nodes.size(); ++i)
			if (G->nodes[i].hash == h) {
				if (G->nodes[i].g <= g) {
					PruneProof pr;
					pr.state_hash = h;
					pr.g_pruned = g;
					pr.g_kept = G->nodes[i].g;
					pr.bound_id = 0x474C0001u;
					pr.h_cert = 0;
					pr.incumbent = G->Tstar;
					G->prunes.push_back(pr);
					return -1;
				}
				// The NEW arrival is earlier: it dominates. Replace
				// g/parent and keep the discovered edges (they remain
				// exact from the same state).
				PruneProof pr;
				pr.state_hash = h;
				pr.g_pruned = G->nodes[i].g;
				pr.g_kept = g;
				pr.bound_id = 0x474C0001u;
				pr.h_cert = 0;
				pr.incumbent = G->Tstar;
				G->prunes.push_back(pr);
				G->nodes[i].g = g;
				G->nodes[i].parent = parent;
				G->nodes[i].parent_edge = parent_edge;
				return static_cast<int>(i);
			}
		Node n;
		n.B = B;
		n.hash = h;
		n.g = g;
		n.parent = parent;
		n.parent_edge = parent_edge;
		G->nodes.push_back(n);
		return static_cast<int>(G->nodes.size()) - 1;
	}

	// THE CANONICAL AIR LEG (session 18: promoted from the fixture
	// harness so launch and route replay share ONE realization): fly
	// the (side, cosa) schedule from *s toward face_j, deterministic
	// per-tick input derivation (coast = no input; WishInputs realizes
	// the wish exactly), stopping at the first contact with face_j
	// (the strike tick is booked, matching Air::Result::tick). Returns
	// booked ticks, or -1 if the horizon runs out without a strike.
	inline int FlyAirLeg(PlayerState* s, const World& w,
	                     const MoveParams& p, const Route::Graph& g,
	                     int face_j,
	                     const std::vector<signed char>& side,
	                     const std::vector<float>& cosa, int horizon) {
		const int hold = s->ducked ? IN_DUCK : 0;
		const int n = static_cast<int>(side.size());
		const Route::Face& fj = g.faces[static_cast<size_t>(face_j)];
		for (int k = 0; k < horizon; ++k) {
			const float s2d = Len2D(s->vel);
			const float h = s2d > 1.f ? atan2f(s->vel.Y, s->vel.X)
				: 0.f;
			const int sd = k < n
				? static_cast<int>(side[static_cast<size_t>(k)])
				: (n > 0 ? static_cast<int>(
					side[static_cast<size_t>(n) - 1]) : 0);
			float yaw = h * 57.2957795f;
			float fm = 0.f, sm = 0.f;
			if (sd != 0 && s2d > 1.f) {
				const float ca = k < n
					? cosa[static_cast<size_t>(k)]
					: (n > 0 ? cosa[static_cast<size_t>(n) - 1]
						: 1.f);
				Air::WishInputs(h, sd, ca, &yaw, &fm, &sm);
			}
			TickEvents ev;
			MoveTick(*s, w, p, 0.f, yaw, fm, sm, 0.f, hold, &ev);
			for (int c = 0; c < ev.ncontacts; ++c)
				if (ev.contact_brush[c] == fj.brush
					&& ev.contact_plane[c] == fj.side)
					return k + 1;
		}
		return -1;
	}

	// END-VIA-AIR (session 18): the finish approach that leaves the
	// last face and touches the END set in FREE FLIGHT. Air targets
	// faces; the END set is a position box, so it needs its own
	// deterministic zone-probe flight. Same input realization as
	// FlyAirLeg; the zone is tested FIRST each tick (ClassifyTick's
	// END > GROUND > CONTACT precedence applied to a zone-only
	// flight); any brush contact or landing stops the probe as a
	// non-finish. A contact with the end brush OUTSIDE the sound zone
	// box is the caller's zone-model-miss counter, never a claimed
	// finish.
	struct ZoneProbeResult {
		bool reached = false;
		int  tick = 0;            // booked ticks at the stop
		PlayerState end_state;
		bool contact = false;
		int  contact_brush = -1;
		bool grounded = false;
	};
	inline ZoneProbeResult FlyZoneSchedule(const PlayerState& s0,
	                                       const World& w,
	                                       const MoveParams& p,
	                                       const std::vector<signed char>& side,
	                                       const std::vector<float>& cosa,
	                                       int horizon, const Vec3& zmin,
	                                       const Vec3& zmax) {
		ZoneProbeResult r;
		PlayerState s = s0;
		const int hold = s.ducked ? IN_DUCK : 0;
		const int n = static_cast<int>(side.size());
		for (int k = 0; k < horizon; ++k) {
			const float s2d = Len2D(s.vel);
			const float h = s2d > 1.f ? atan2f(s.vel.Y, s.vel.X)
				: 0.f;
			const int sd = k < n
				? static_cast<int>(side[static_cast<size_t>(k)])
				: (n > 0 ? static_cast<int>(
					side[static_cast<size_t>(n) - 1]) : 0);
			float yaw = h * 57.2957795f;
			float fm = 0.f, sm = 0.f;
			if (sd != 0 && s2d > 1.f) {
				const float ca = k < n
					? cosa[static_cast<size_t>(k)]
					: (n > 0 ? cosa[static_cast<size_t>(n) - 1]
						: 1.f);
				Air::WishInputs(h, sd, ca, &yaw, &fm, &sm);
			}
			TickEvents ev;
			MoveTick(s, w, p, 0.f, yaw, fm, sm, 0.f, hold, &ev);
			if (s.pos.X >= zmin.X && s.pos.X <= zmax.X
				&& s.pos.Y >= zmin.Y && s.pos.Y <= zmax.Y
				&& s.pos.Z >= zmin.Z && s.pos.Z <= zmax.Z) {
				r.reached = true;
				r.tick = k + 1;
				r.end_state = s;
				return r;
			}
			if (ev.ncontacts > 0 || s.on_ground) {
				r.contact = ev.ncontacts > 0;
				r.contact_brush = ev.ncontacts > 0
					? ev.contact_brush[0]
					: (s.on_ground ? s.ground_brush : -1);
				r.grounded = s.on_ground;
				r.tick = k + 1;
				r.end_state = s;
				return r;
			}
		}
		r.tick = horizon;
		r.end_state = s0;
		if (horizon > 0)
			r.end_state = s;
		return r;
	}

	// COLD EDGE REPLAY: reproduce the edge from its stored witness
	// segments alone, verifying against the stored destination
	// bitwise. The zone box arms END edges. An END edge with
	// dt_air > 0 is END-VIA-AIR: the ride segment must exit clean and
	// the stored zone-probe schedule must reach the zone at exactly
	// dt_air with the stored finish state.
	inline bool ReplayEdge(const Ride::BoundaryState& Bfrom,
	                       const TransferEdge& e, const World& w,
	                       const MoveParams& p, const Route::Graph& g,
	                       const Vec3* zone_min, const Vec3* zone_max) {
		Ride::Result rr = Ride::FlyRideInputs(Bfrom, w, p, g, e.ride,
			nullptr, zone_min, zone_max);
		if (!rr.legal || rr.ticks != e.dt_exit)
			return false;
		if (e.kind == kEdgeEnd) {
			if (e.dt_air <= 0)
				return rr.kind == Ride::kEnd
					&& memcmp(&rr.end_state, &e.Bj.ps,
						sizeof(PlayerState)) == 0;
			if (rr.kind != Ride::kAirExit || !zone_min || !zone_max)
				return false;
			ZoneProbeResult zr = FlyZoneSchedule(rr.end_state, w, p,
				e.air_side, e.air_cosa, e.air_horizon, *zone_min,
				*zone_max);
			return zr.reached && zr.tick == e.dt_air
				&& memcmp(&zr.end_state, &e.Bj.ps,
					sizeof(PlayerState)) == 0;
		}
		if (e.kind == kEdgeContact)
			return rr.kind == Ride::kContactTransfer
				&& memcmp(&rr.end_state, &e.Bj.ps,
					sizeof(PlayerState)) == 0;
		if (rr.kind != Ride::kAirExit)
			return false;
		Air::Target vt;
		vt.face = e.face_j;
		vt.dot_cap = 3000.f;
		vt.aim = g.faces[static_cast<size_t>(e.face_j)].centroid;
		vt.max_ticks = e.air_horizon;
		Air::Result ar = Air::FlyWishSchedule(rr.end_state, w, p, vt,
			g, e.air_side, e.air_cosa, e.air_horizon);
		return ar.hit && ar.dot < 0.f && ar.struck_brush < 0
			&& ar.tick == e.dt_air
			&& memcmp(&ar.end_state, &e.Bj.ps,
				sizeof(PlayerState)) == 0;
	}

	// THE LAUNCH (session 18): from the anchored spawn state to the
	// first exact board. Ground segment = exact MoveInput packets (the
	// run to the edge); air segment = the (side, cosa) witness whose
	// per-tick inputs FlyAirLeg re-derives deterministically. g0 is
	// the route clock: EVERY tick from the anchored spawn state,
	// including the board tick. Production machinery only - a launch
	// is never seeded from a tape.
	struct LaunchWitness {
		std::vector<Ride::MoveInput> ground;
		std::vector<signed char> air_side;
		std::vector<float> air_cosa;
		int  air_horizon = 0;
		int  face = -1;             // first boarded face
		int  dt_air = 0;            // flight ticks incl. the board tick
		Ride::BoundaryState B0;     // the boarded boundary state
		int  g0 = 0;                // ground.size() + dt_air
		int  variant = -1;          // launch-direction index (audit)
	};

	// Cold launch replay: anchor -> ground packets -> airborne -> air
	// leg -> bitwise B0 at exactly g0 ticks.
	inline bool ReplayLaunch(const LaunchWitness& lw,
	                         const PlayerState& anchor, const World& w,
	                         const MoveParams& p,
	                         const Route::Graph& g) {
		if (lw.face < 0
			|| lw.face >= static_cast<int>(g.faces.size()))
			return false;
		PlayerState s = anchor;
		for (size_t k = 0; k < lw.ground.size(); ++k) {
			TickEvents ev;
			MoveTick(s, w, p, lw.ground[k].pitch, lw.ground[k].yaw,
				lw.ground[k].fmove, lw.ground[k].smove,
				lw.ground[k].umove, lw.ground[k].buttons, &ev);
		}
		if (s.on_ground)
			return false;
		const int dt = FlyAirLeg(&s, w, p, g, lw.face, lw.air_side,
			lw.air_cosa, lw.air_horizon);
		return dt == lw.dt_air
			&& static_cast<int>(lw.ground.size()) + dt == lw.g0
			&& memcmp(&s, &lw.B0.ps, sizeof(PlayerState)) == 0;
	}

	// THE HIERARCHICAL ROUTE WITNESS (advisor 2026-08-20b): the
	// constructive proof object of an incumbent -
	//     LaunchWitness + TransferEdge[] + the END event.
	// Replay walks the pieces IN SEQUENCE through ONE PlayerState with
	// no regeneration and no re-instantiation at boundaries; the
	// finish tick must equal the graph's claim exactly. This artifact
	// proves "a legal finish in T* ticks exists"; the certified bound
	// ledger will one day prove "nothing finishes sooner".
	struct RouteWitness {
		LaunchWitness launch;
		std::vector<TransferEdge> edges;   // self-contained copies
		int Tf = -1;                       // claimed finish tick
	};

	// Continuous whole-route replay through ONE PlayerState. Verifies
	// every stored boundary bitwise; *ticks_out counts every booked
	// tick from the anchor. Success requires *ticks_out == rw.Tf and
	// zero boundary mismatches.
	inline bool ReplayRouteWitness(const RouteWitness& rw,
	                               const PlayerState& anchor,
	                               const World& w, const MoveParams& p,
	                               const Route::Graph& g,
	                               const Vec3& zmin, const Vec3& zmax,
	                               int* ticks_out, int* mismatches) {
		PlayerState s = anchor;
		int tk = 0, bad = 0;
		for (size_t k = 0; k < rw.launch.ground.size(); ++k, ++tk) {
			TickEvents ev;
			MoveTick(s, w, p, rw.launch.ground[k].pitch,
				rw.launch.ground[k].yaw, rw.launch.ground[k].fmove,
				rw.launch.ground[k].smove, rw.launch.ground[k].umove,
				rw.launch.ground[k].buttons, &ev);
		}
		if (s.on_ground) {
			if (ticks_out) *ticks_out = tk;
			if (mismatches) *mismatches = 1;
			return false;
		}
		const int ldt = FlyAirLeg(&s, w, p, g, rw.launch.face,
			rw.launch.air_side, rw.launch.air_cosa,
			rw.launch.air_horizon);
		if (ldt < 0) {
			if (ticks_out) *ticks_out = tk;
			if (mismatches) *mismatches = 1;
			return false;
		}
		tk += ldt;
		if (memcmp(&s, &rw.launch.B0.ps, sizeof(PlayerState)) != 0)
			bad++;
		for (const TransferEdge& e : rw.edges) {
			for (size_t k = 0; k < e.ride.size(); ++k, ++tk) {
				TickEvents ev;
				MoveTick(s, w, p, e.ride[k].pitch, e.ride[k].yaw,
					e.ride[k].fmove, e.ride[k].smove,
					e.ride[k].umove, e.ride[k].buttons, &ev);
			}
			if (e.kind == kEdgeAir) {
				const int adt = FlyAirLeg(&s, w, p, g, e.face_j,
					e.air_side, e.air_cosa, e.air_horizon);
				if (adt < 0) {
					bad++;
					break;
				}
				tk += adt;
			} else if (e.kind == kEdgeEnd && e.dt_air > 0) {
				ZoneProbeResult zr = FlyZoneSchedule(s, w, p,
					e.air_side, e.air_cosa, e.air_horizon, zmin,
					zmax);
				if (!zr.reached) {
					bad++;
					break;
				}
				s = zr.end_state;
				tk += zr.tick;
			}
			if (memcmp(&s, &e.Bj.ps, sizeof(PlayerState)) != 0)
				bad++;
		}
		if (ticks_out) *ticks_out = tk;
		if (mismatches) *mismatches = bad;
		return bad == 0 && tk == rw.Tf;
	}

} // namespace GlobalSearch
} // namespace Solver
