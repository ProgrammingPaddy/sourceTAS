#pragma once

// BSP collision world for the solver core. Parses the .bsp FILE (the same
// bytes the engine loaded) and keeps every player-solid WORLD brush as a
// hull-expanded convex plane set, queried by swept-hull traces over a uniform
// grid. This mirrors the DLL's month-validated collision model exactly:
//
//   - clip planes = ALL non-bevel brush sides (winding-surviving or not)
//   - PLUS the compiler's EXACT AXIAL bevel planes from the brushside lump
//   - non-axial edge bevels are NOT loaded (deliberate: including them
//     deflected proven-real lines; sides + axial extents ARE the engine's
//     effective swept-hull behavior per the dpos=0.000 trace history)
//   - only worldspawn (model 0) brushes collide; entity brushes (triggers,
//     illusionaries) are CONTENTS_SOLID on disk but block nothing
//   - expansion = Minkowski support of the NEGATED player hull, feet origin
//   - trace = CM_ClipBoxToBrush semantics: enterfrac starts at -1 and the
//     compare is UNCLAMPED (started-touching corners pick the least-negative
//     plane), DIST_EPSILON 1/32 backoff, final result clamped to [0,1]
//
// Every brush carries TWO expanded distance sets (standing + ducked hull) so
// the mover can swap hulls mid-flight (air crouch).

#include "SolverMath.h"

#include <string>
#include <vector>

namespace Solver {

	struct Hulls {
		Vec3 stand_min = Vec3(-16.f, -16.f, 0.f);
		Vec3 stand_max = Vec3(16.f, 16.f, 72.f);  // canonical CS:S standing
		                                          // (VDC dimensions; eye 64)
		Vec3 duck_min = Vec3(-16.f, -16.f, 0.f);
		Vec3 duck_max = Vec3(16.f, 16.f, 54.f);   // 72-54 = the measured ~18u
		                                          // in-air FinishDuck origin lift
		// The hull TOP is a KNOWN CONSTANT (72 standing / 54 ducked, VDC
		// dimensions) and in practice is never the contacted surface, so
		// fitting it from trajectory behaviour fits noise into a value we
		// already know. A "post-unduck transient 62.5" was invented here
		// on that basis and is REMOVED 2026-08-15: hull state 2 is now
		// identical to standing. Whatever produced the release bracket
		// (56.7, 63.7) is something other than hull height, and will be
		// identified by isolating the function that owns it.
		Vec3 unduck_min = Vec3(-16.f, -16.f, 0.f);
		Vec3 unduck_max = Vec3(16.f, 16.f, 72.f);
	};

	struct WorldBrush {
		int  id = -1;             // brush index in the BSP lumps
		int  contents = 0;
		Vec3 bmin, bmax;          // brush AABB from the compiler's AXIAL planes
		                          // (exact, unlike winding-derived boxes)
		// Plane set: real sides first ([0, nsides)), then axial bevels (pid -2).
		std::vector<Vec3>  n;
		std::vector<float> d;         // raw plane distances (unexpanded)
		std::vector<float> d_stand;   // hull-expanded, standing
		std::vector<float> d_duck;    // hull-expanded, ducked
		std::vector<float> d_unduck;  // hull-expanded, post-air-unduck
		                              // transient (top 62.5; see Hulls)
		std::vector<int>   pid;       // BSP plane id (sides), -2 for bevels
		int nsides = 0;
		// Origin-space AABB gates (brush box minus hull), per hull.
		Vec3 gmin_stand, gmax_stand;
		Vec3 gmin_duck, gmax_duck;
		Vec3 gmin_unduck, gmax_unduck;
	};

	struct TraceResult {
		float frac = 1.f;
		int   brush = -1;        // index into World::brushes (NOT the BSP id)
		int   plane = -1;        // plane index within that brush
		Vec3  normal;
		// CM_ClipBoxToBrush solid reporting: startsolid = the sweep began
		// inside a brush; allsolid = it began inside and never got out.
		// TryPlayerMove ZEROES VELOCITY on allsolid - the surf ramp bug.
		bool  startsolid = false;
		bool  allsolid = false;
	};

	class World {
	public:
		// Loads collision + spawn from the .bsp. False on any structural error.
		// edge_bevels: load the compiler's NON-AXIAL bevel sides too (corner
		// separating planes for AABB hulls). Measured 2026-08-13: without
		// them the core clips a ramp face ONE TICK LONGER than the engine
		// when the box slides off the brush end mid-surf (604 tape, tick 284
		// vs real playback - the engine flew free because the edge bevel
		// separated the box at the corner). false = the pre-fix clip set.
		// (err gets the reason). Never trusts sizeof() for on-disk strides.
		bool Load(const std::string& bsp_path, const Hulls& hulls, std::string* err,
		          bool edge_bevels = true);

		// Swept trace of the hull ORIGIN from a to b (planes are pre-expanded;
		// `ducked` picks the distance set). Returns the earliest entry fraction
		// in [0,1] (1 = clear) and fills out (out may be null).
		float TraceHull(const Vec3& a, const Vec3& b, bool ducked, TraceResult* out) const;
		// Three-state hull variant: 0 = standing, 1 = ducked, 2 = the
		// post-air-unduck transient (top 62.5 until grounding).
		float TraceHull3(const Vec3& a, const Vec3& b, int hull, TraceResult* out) const;

		// Arbitrary box sweep - planes expanded per query instead of from
		// the three precomputed hulls. Needed by TracePlayerBBoxForGround,
		// whose quadrant boxes are not any player hull.
		float TraceHullBox(const Vec3& a, const Vec3& b, const Vec3& mins,
		                   const Vec3& maxs, TraceResult* out) const;

		// True when the hull at origin o overlaps any collidable brush (static
		// containment against the expanded plane sets). Duck/unduck validation.
		bool OriginInSolid(const Vec3& o, bool ducked) const;

		// BSP id of the brush directly beneath o (long down-trace), or -1.
		int BrushUnder(const Vec3& o, bool ducked) const;

		// Index into brushes for a BSP brush id, or -1.
		int IndexOfBrushId(int bsp_id) const;

		const Hulls& HullDims() const { return hulls_; }

		// ---- TRIGGER VOLUMES (user directive 2026-08-15: total parity
		// includes triggers - gravity, push, teleport). Parsed from the
		// entity lump + brush models; the touch test is the same
		// support-corner Minkowski math as the DLL's BspWorld::
		// CheckTriggers (raw planes incl. ALL compiler bevels - with
		// bevels this IS the exact hull-vs-brush sum). Trigger geometry
		// NEVER collides; it only touches. ----
		struct TrigBrush {
			std::vector<Vec3>  n;
			std::vector<float> d;
		};
		struct TriggerVol {
			int  model = -1;
			int  spawnflags = 0;
			Vec3 ent_origin;
			bool teleport = false, push = false;   // neither = gravity
			bool dest_ok = false, landmark_ok = false;
			Vec3 dest_origin, landmark_origin;
			float dest_pitch = 0.f, dest_yaw = 0.f;
			Vec3 push_dir;
			float push_speed = 0.f;
			float gravity = 1.f;
			std::vector<int> tb;      // indices into trig_brushes
		};
		struct TriggerHitS {
			bool  teleported = false;
			Vec3  tp_origin;
			bool  tp_set_angles = false;
			float tp_pitch = 0.f, tp_yaw = 0.f;
			bool  pushed = false;
			Vec3  push_vec;
			bool  grav_touched = false;
			float gravity = 1.f;
		};
		std::vector<TriggerVol> triggers;
		std::vector<TrigBrush>  trig_brushes;
		bool HasTriggers() const { return !triggers.empty(); }
		// Hull (by movement hull state 0/1/2) vs every trigger volume;
		// mirrors BspWorld::CheckTriggers semantics exactly (client
		// spawnflag gate, ent-origin local space, first teleport wins).
		bool CheckTriggers(const Vec3& origin, int hull_state,
		                   TriggerHitS* out) const;

		// Map facts.
		int version = 0;
		int mapRevision = 0;
		int nplanes = 0;
		int nbrushes_total = 0;
		int nmodels = 0;
		int nbrushes_entity = 0;     // solid brushes skipped as non-world
		bool have_spawn = false;
		Vec3 spawn_origin;
		float spawn_pitch = 0.f, spawn_yaw = 0.f;
		std::string entities_text;   // raw entities lump (zone/trigger work later)

		std::vector<WorldBrush> brushes;   // collidable set only

		// The map's brush-box union. NOT the sealed volume - a protruding
		// brush (basictest's finish platform reaches y -2336, past the wall
		// at -2048) makes this far larger than the room, which is why an
		// AABB test let every behind-the-wall probe through. Kept for the
		// grid only; solidity comes from the BSP tree below.
		Vec3 world_min, world_max;

		// BSP TREE (model 0), the engine's own authority on what is solid:
		// outside the sealed world every leaf is CONTENTS_SOLID, so a hull
		// out there traces startsolid+allsolid and TryPlayerMove freezes
		// the player. Walking the tree is what CM_PointLeafnum does.
		struct BspNode { int plane = 0; int child[2] = { 0, 0 }; };
		std::vector<BspNode> nodes;
		std::vector<int>     leaf_contents;
		std::vector<Vec3>    tree_n;   // plane normals, by BSP plane index
		std::vector<float>   tree_d;
		int headnode = 0;

		// ENGINE BRUSH ORDER (leafbrushes lump): the trace clips brushes in
		// leaf-traversal order, per-leaf list order - CM_TraceToLeaf's own
		// organization. Order is OBSERVABLE: an allsolid brush zeroes the
		// fraction the moment it is processed, so which recordings survive
		// depends on processing order (measured: at one site the engine kept
		// the floor plane past an allsolid wall in one box and dropped a
		// neighbor's plane in another - one fixed list order explains both).
		std::vector<int> leaf_firstbrush;   // per leaf, into leafbrush_ours
		std::vector<int> leaf_numbrushes;
		std::vector<int> leafbrush_ours;    // lump 17 mapped to OUR brush
		                                    // indices (-1 = filtered brush)
		// Swept-box leaf walk, near child first (the side the sweep starts
		// on), appending each leaf's brushes in lump order, first occurrence
		// only (the engine's checkcount). Returns count written to out_list.
		int OrderedLeafBrushes(const Vec3& a, const Vec3& b, const Vec3& mins,
		                       const Vec3& maxs, int* out_list, int cap) const;

		// True when the box at origin overlaps any CONTENTS_SOLID leaf.
		// UNSWEPT tests consult LEAF CONTENTS (box oracle 2026-08-15: a
		// zero-length box above the ceiling slab - overlapping NO brush -
		// returns startsolid+allsolid, while swept traces out there return
		// clear); swept traces clip brushes only.
		bool BoxInSolidLeaf(const Vec3& origin, const Vec3& mins,
		                    const Vec3& maxs) const;

		// CONTENTS_SOLID of the leaf containing p (0 when the tree is
		// absent). Mirrors CM_PointLeafnum's descent exactly.
		bool PointInSolidLeaf(const Vec3& p) const;
		// True when any corner of the hull at origin o lands in a solid
		// leaf - the engine sweeps the BOX, not the point.
		bool HullInSolidLeaf(const Vec3& o, int hull) const;

		// Corner-release semantics (engine-measured 2026-08-13, 604-tape
		// capture): the hit test uses the TRUE un-padded crossing interval;
		// DIST_EPSILON pads the reported position only. false = the legacy
		// epsilon-padded hit test (pre-fix control arm).
		bool true_interval_corner = true;

		// TRACE ORACLE hook: when set, every TraceHull call appends its query
		// + OUR answer here (single-threaded replay only). The rows double as
		// the engine oracle's query file - the in-game Map Solve tab answers
		// them with IEngineTrace and `tracediff` compares us trace by trace.
		struct TraceProbeRow {
			int tick = 0;
			Vec3 a, b;
			bool ducked = false;
			float frac = 1.f;
			Vec3 n;
			int brush_id = -1;
		};
		std::vector<TraceProbeRow>* trace_log = nullptr;
		int trace_tick = 0;

	private:
		Hulls hulls_;
		// Uniform grid over the STANDING expanded AABBs (a superset of the
		// ducked ones, so one gate is conservative-correct for both hulls).
		float cell_ = 256.f;
		Vec3  gmin_, gmax_;
		int   nx_ = 0, ny_ = 0, nz_ = 0;
		std::vector<std::vector<int>> cells_;
		void BuildGrid();
		void CellRange(const Vec3& lo, const Vec3& hi,
		               int* x0, int* x1, int* y0, int* y1, int* z0, int* z1) const;
	};

} // namespace Solver
