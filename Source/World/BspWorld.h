#pragma once

#include <cstrike/Structures/Vector.h>

// 1.1 world geometry: the map's brush collision data, parsed from the BSP file
// (the same bytes the engine loaded - planes/brushes/brushsides are static, and
// the file format is documented and version-stable, so this is the reliable
// access path; the in-memory engine structures stay a fallback if runtime-only
// data is ever needed). Faces are computed by winding each brush side's plane
// against the others, kept with their plane index so surfaces can later be
// tagged as ramp faces for the board/pathing solvers.
namespace BspWorld {
	// Menu-controlled draw settings.
	extern bool  draw_wireframe;    // master toggle
	extern float draw_radius;       // only brushes within this range of the player
	extern int   line_budget;       // max overlay lines per frame (nearest first)
	extern bool  show_solid;        // CONTENTS_SOLID brushes
	extern bool  show_playerclip;   // CONTENTS_PLAYERCLIP (invisible, but collides)
	extern bool  show_ladder;       // CONTENTS_LADDER
	extern bool  show_markers;      // tagged faces + board targets
	extern int   highlight_target;  // target index drawn emphasized (-1 = none)

	// --- picking (crosshair ray vs the parsed brushes) -----------------------
	struct RayHit {
		bool hit = false;
		int brush = -1;
		int plane = -1;          // dplane index of the struck face
		int contents = 0;        // struck brush's contents (solid vs clip etc.)
		float dist = 0.f;
		Vector point;
		Vector normal;
	};
	// Respects the show_* contents toggles, so what you see is what you pick.
	RayHit Pick(const Vector& eye, const Vector& dir, float max_dist);

	// Remember the last pick for debugging: the ray + hit draw in-world for a
	// couple of seconds so a bad pick shows WHERE it actually went.
	void NotePickDebug(const Vector& eye, const Vector& dir, const RayHit& hit);

	// --- surf-face tags (persisted per map) ----------------------------------
	bool ToggleFaceTag(int brush, int plane);   // returns the new tagged state
	bool IsTagged(int brush, int plane);
	int  TagCount();
	void ClearTags();
	bool GetTag(int index, int* brush, int* plane);   // enumerate (stable order)
	void RemoveTag(int index);
	extern int highlight_tag;                   // tag index drawn emphasized (-1 = none)

	// --- geometry queries (for the solvers) ----------------------------------
	bool GetPlane(int plane, Vector* normal, float* dist);
	// Pitch (Source sign: + = down) that puts the view direction for `yaw`
	// exactly IN the face's plane - the "pitch locked to the ramp" view.
	bool LockedPitch(int plane, float yaw, float* out_pitch);
	// First face of `brush` lying on `plane`: polygon points + centroid.
	bool GetFacePolygon(int brush, int plane, const Vector** points, int* count);
	// Is `point` (projected onto the plane) within the face polygon, allowing
	// `expand` units of slack past the edges?
	bool PointOnFace(int brush, int plane, const Vector& point, float expand);

	// --- board targets: solver destinations (persisted per map) --------------
	// A target is the player hull rested tangent against a face (the same
	// contact the movement collision produces when dropped there), plus the
	// view angles - the future perfect-board allowance range hangs off these.
	struct BoardTarget {
		Vector pos;              // hull feet origin, resting on the face
		float yaw = 0.f;
		float pitch = 0.f;
		bool pitch_lock = false; // pitch auto-follows the face: view stays in the ramp plane
		bool ducked = false;
		int brush = -1;
		int plane = -1;
	};
	int  AddTargetFromHit(const RayHit& hit, float yaw, float pitch, bool ducked);
	int  TargetCount();
	BoardTarget* GetTarget(int index);          // null when out of range
	void RemoveTarget(int index);
	void RestTargetOnFace(int index);           // re-seat after a duck change
	void SaveGeo();                             // call after editing a target

	// Parse the currently loaded map. Called automatically by Render() when the
	// level changes; exposed for the menu's Reload button.
	bool LoadCurrentMap();
	void Unload();

	// Queue wireframe overlays around `center`. Self-guards when disabled, out
	// of game, or unloaded (auto-loads on level change).
	void Render(const Vector& center, bool have_center, float duration);

	// Status for the menu.
	struct Status {
		bool loaded = false;
		char map[128] = {};
		int  bsp_version = 0;
		int  brush_count = 0;      // parsed (after contents filter at load: none)
		int  face_count = 0;
		int  edge_count = 0;
		float parse_ms = 0.f;
		int  drawn_brushes = 0;    // last frame
		int  drawn_lines = 0;      // last frame
		char error[256] = {};      // includes the attempted path on open failure
	};
	Status GetStatus();
}
