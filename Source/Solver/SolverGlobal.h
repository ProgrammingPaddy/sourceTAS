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

	struct Graph {
		std::vector<Node> nodes;
		std::vector<TransferEdge> edges;
		std::vector<PruneProof> prunes;
		int  Tstar = -1;            // incumbent finish tick (-1 = inf)
		int  Tstar_node = -1;       // node whose END edge finished
		int  Tstar_edge = -1;
		int  incumbent_updates = 0; // G8 observability
		int  m_explicit = 240;      // pre-incumbent horizon domain
	};

	// Offer a production finish to the incumbent: the first
	// establishes T*, a strictly faster witness replaces it, anything
	// else changes nothing. The ONE code path every END edge goes
	// through (ExpandNode and any future assembler both call here).
	inline bool OfferFinish(Graph* G, int node, int edge, int Tf) {
		if (G->Tstar < 0 || Tf < G->Tstar) {
			G->Tstar = Tf;
			G->Tstar_node = node;
			G->Tstar_edge = edge;
			G->incumbent_updates++;
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

	// COLD EDGE REPLAY: reproduce the edge from its stored witness
	// segments alone, verifying against the stored destination
	// bitwise. The zone box arms END edges.
	inline bool ReplayEdge(const Ride::BoundaryState& Bfrom,
	                       const TransferEdge& e, const World& w,
	                       const MoveParams& p, const Route::Graph& g,
	                       const Vec3* zone_min, const Vec3* zone_max) {
		Ride::Result rr = Ride::FlyRideInputs(Bfrom, w, p, g, e.ride,
			nullptr, zone_min, zone_max);
		if (!rr.legal || rr.ticks != e.dt_exit)
			return false;
		if (e.kind == kEdgeEnd)
			return rr.kind == Ride::kEnd
				&& memcmp(&rr.end_state, &e.Bj.ps,
					sizeof(PlayerState)) == 0;
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

} // namespace GlobalSearch
} // namespace Solver
