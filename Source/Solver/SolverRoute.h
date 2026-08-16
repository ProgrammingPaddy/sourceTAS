#pragma once

// STAGE 1 of the rebuild (Docs/SolverRebuild.md #5): the feature graph.
// Extract surfable features from the loaded World - ramp faces as
// polygons with planes, extents, and edges - and enumerate candidate
// transfers. Routes are paths through this graph; the transfer primitive
// (#3.2) composes along its edges.

#include "SolverWorld.h"
#include <vector>

namespace Solver {
namespace Route {

	// One surfable ramp face: a REAL brush side with a sloped, non-walkable
	// plane (0.05 < nz < 0.7 - vertical walls hold nothing, walkable tops
	// are ground, not surf).
	struct Face {
		int   brush = -1;        // World::brushes index
		int   side = -1;         // plane index within the brush
		Vec3  n;                 // outward plane normal
		float d = 0.f;
		std::vector<Vec3> verts; // polygon on the plane (world space)
		Vec3  centroid;
		float area = 0.f;
		float zmin = 0.f, zmax = 0.f;
		// The face's downhill direction projected into the plane -
		// gravity's in-plane component, the carve's conversion axis.
		Vec3  downhill;
	};

	// Candidate transfer A -> B (coarse gate; refined by the transfer
	// solver in stage 3).
	struct Edge {
		int   from = -1, to = -1;  // Face indices (-1 from = start zone)
		float dist = 0.f;          // centroid distance
		float dz = 0.f;            // to.centroid.z - from.centroid.z
	};

	struct Graph {
		std::vector<Face> faces;
		std::vector<Edge> edges;
		// Zone anchoring (M0.5): the run's endpoints as graph nodes.
		int  start_brush = -1;      // World::brushes index (the platform)
		Vec3 start_pos;             // the solve anchor's origin
		int  end_brush = -1;        // World::brushes index (finish zone)
		Vec3 end_center;
		std::vector<int> start_faces;   // candidate first boards
		std::vector<int> end_faces;     // faces that can feed the finish
	};

	// Build the graph from a loaded world. max_edge_dist gates candidate
	// transfers by centroid distance (coarse reachability; the analytic
	// envelope replaces this in stage 2).
	bool Build(const World& w, Graph* out, float max_edge_dist,
	           std::string* err);

	// Resolve start/end zones into the graph: start = the brush under the
	// anchor origin, end = a brush id (the finish platform/zone marker the
	// era already uses). Candidate lists are distance-gated like edges.
	bool AnchorZones(const World& w, Graph* g, const Vec3& anchor_pos,
	                 bool ducked, int end_brush_id, float max_edge_dist,
	                 std::string* err);

} // namespace Route
} // namespace Solver
