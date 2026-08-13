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
	// A brush's COMPLETE convex plane set (the same set the ray clip uses), for
	// hull-vs-brush sweeps in the route model.
	int  BrushClipPlaneCount(int brush);
	bool GetBrushClipPlane(int brush, int index, int* plane_id, Vector* n, float* d);
	// Enumeration for the route model's collision world: every parsed brush,
	// with contents + AABB, so the solver can gather all solids along a flight
	// corridor (the engine's TracePlayerBBox collides with all of them).
	int  BrushCount();
	bool GetBrushInfo(int brush, int* contents, Vector* mins, Vector* maxs);
	// True when the brush belongs to worldspawn (model 0) - the only brushes
	// the engine traces player movement against. Brush ENTITIES (triggers,
	// illusionaries) are CONTENTS_SOLID on disk but block nothing.
	bool IsWorldBrush(int brush);
	int  BrushBevelPlaneCount(int brush);
	bool GetBrushBevelPlane(int brush, int index, Vector* n, float* d);
	// Pitch (Source sign: + = down) that puts the view direction for `yaw`
	// exactly IN the face's plane - the "pitch locked to the ramp" view.
	bool LockedPitch(int plane, float yaw, float* out_pitch);
	// First face of `brush` lying on `plane`: polygon points + centroid.
	bool GetFacePolygon(int brush, int plane, const Vector** points, int* count);
	// Is `point` (projected onto the plane) within the face polygon, allowing
	// `expand` units of slack past the edges?
	bool PointOnFace(int brush, int plane, const Vector& point, float expand);
	// Richer variant: returns true when within `margin` of the face (inside or
	// in the outside edge zone); *interior reports strictly-inside-by-margin.
	// The gap between the two is the EDGE ZONE - where a hull catches the
	// brush's crest/corner instead of boarding the face.
	bool PointOnFaceQuery(int brush, int plane, const Vector& point,
	                      float margin, bool* interior);

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

	// --- TRIGGERS (teleport + gravity) ---------------------------------------
	// Parsed from the BSP's entity lump (the same keyvalues the server spawns
	// from): trigger_teleport / trigger_gravity brush entities, their volumes
	// (per-model brush sets), destinations and landmarks. The client's
	// prediction NEVER runs server trigger logic, so the editor sim applies
	// these itself - semantics copied from the SDK's CTriggerTeleport::Touch /
	// CTriggerGravity::GravityTouch exactly.
	extern bool apply_triggers;     // sim applies trigger effects (persisted)
	extern bool show_triggers;      // master: color-coded trigger volumes. The
	                                // volume wires draw INDEPENDENTLY of the
	                                // full brush wireframe, per-type below.
	extern bool show_trig_tp;       // teleport volumes (gold)
	extern bool show_trig_push;     // push/booster volumes (green)
	extern bool show_trig_grav;     // gravity volumes (blue)
	extern bool show_trig_links;    // teleport dest arrows + booster rays

	struct TriggerHit {
		bool   teleported = false;
		Vector tp_origin;            // destination (landmark math already applied)
		bool   tp_set_angles = false;
		float  tp_pitch = 0.f, tp_yaw = 0.f;
		bool   grav_touched = false;
		float  gravity = 1.f;        // player gravity SCALE (trigger keyvalue)
		bool   pushed = false;       // trigger_push (booster)
		Vector push_vec;             // pushdir * speed (base velocity per SDK)
	};
	// Hull-vs-trigger-volume test for one sim tick. `origin` is the player
	// feet origin, mins/maxs the current collision hull. First touched
	// teleport wins (engine touch order); gravity reports the last touched
	// value. Returns true when anything fired.
	bool CheckTriggers(const Vector& origin, const Vector& mins, const Vector& maxs,
	                   TriggerHit* out);
	int  TriggerTeleportCount();
	int  TriggerGravityCount();
	int  TriggerPushCount();

	// Parse the currently loaded map. Called automatically by Render() when the
	// level changes; exposed for the menu's Reload button.
	bool LoadCurrentMap();
	void Unload();

	// The bare current level name ("maps/x.bsp" -> "x") via the validated
	// engine call. False when not in game - callers store "" then.
	bool CurrentMapName(char* out, int cap);

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
