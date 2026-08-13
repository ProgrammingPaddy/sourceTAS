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
		Vec3 stand_max = Vec3(16.f, 16.f, 72.f);
		Vec3 duck_min = Vec3(-16.f, -16.f, 0.f);
		Vec3 duck_max = Vec3(16.f, 16.f, 54.f);   // 72-54 = the measured ~18u
		                                          // in-air FinishDuck origin lift
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
		std::vector<int>   pid;       // BSP plane id (sides), -2 for bevels
		int nsides = 0;
		// Origin-space AABB gates (brush box minus hull), per hull.
		Vec3 gmin_stand, gmax_stand;
		Vec3 gmin_duck, gmax_duck;
	};

	struct TraceResult {
		float frac = 1.f;
		int   brush = -1;        // index into World::brushes (NOT the BSP id)
		int   plane = -1;        // plane index within that brush
		Vec3  normal;
	};

	class World {
	public:
		// Loads collision + spawn from the .bsp. False on any structural error
		// (err gets the reason). Never trusts sizeof() for on-disk strides.
		bool Load(const std::string& bsp_path, const Hulls& hulls, std::string* err);

		// Swept trace of the hull ORIGIN from a to b (planes are pre-expanded;
		// `ducked` picks the distance set). Returns the earliest entry fraction
		// in [0,1] (1 = clear) and fills out (out may be null).
		float TraceHull(const Vec3& a, const Vec3& b, bool ducked, TraceResult* out) const;

		// True when the hull at origin o overlaps any collidable brush (static
		// containment against the expanded plane sets). Duck/unduck validation.
		bool OriginInSolid(const Vec3& o, bool ducked) const;

		// BSP id of the brush directly beneath o (long down-trace), or -1.
		int BrushUnder(const Vec3& o, bool ducked) const;

		// Index into brushes for a BSP brush id, or -1.
		int IndexOfBrushId(int bsp_id) const;

		const Hulls& HullDims() const { return hulls_; }

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
