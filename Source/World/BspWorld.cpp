#include "BspWorld.h"
#include "WorldDraw.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <set>
#include <string>
#include <vector>
#include <windows.h>
#include <shlobj.h>

#include <cstrike/sdk.h>
#include <cstrike/Interfaces/IVDebugOverlay.h>

// Contents bits (Valve's, mirrored - only the ones we filter on).
#define BSP_CONTENTS_SOLID      0x1
#define BSP_CONTENTS_GRATE      0x8
#define BSP_CONTENTS_PLAYERCLIP 0x10000
#define BSP_CONTENTS_LADDER     0x20000000

namespace BspWorld {
	bool  draw_wireframe  = false;
	float draw_radius     = 1500.f;
	int   line_budget     = 800;
	bool  show_solid      = true;
	bool  show_playerclip = true;
	bool  show_ladder     = false;
	bool  show_markers    = true;
	int   highlight_target = -1;
	int   highlight_tag    = -1;
	bool  apply_triggers  = true;
	bool  show_triggers   = true;
	bool  show_trig_tp    = true;
	bool  show_trig_push  = true;
	bool  show_trig_grav  = true;
	bool  show_trig_links = true;
}

namespace {
	// --- VBSP on-disk structures (verified against v19 and v20 maps) ---------
	constexpr int kLumpPlanes = 1;
	constexpr int kLumpBrushes = 18;
	constexpr int kLumpBrushSides = 19;
	// Model 0 is worldspawn: ONLY its brushes are the collision world. Brushes
	// reachable from models 1..N belong to brush ENTITIES (triggers, illusion-
	// aries, doors) - a trigger_teleport slab is CONTENTS_SOLID on disk but
	// blocks nothing, and surf maps carpet the void with them. Walking the
	// world tree is how the engine itself decides, so it is exact.
	constexpr int kLumpEntities = 0;   // plain-text keyvalue blocks, the same
	                                   // data the server spawns entities from
	constexpr int kLumpNodes = 5;
	constexpr int kLumpLeafs = 10;
	constexpr int kLumpModels = 14;
	constexpr int kLumpLeafBrushes = 17;

	// SDK trigger constants (public 2013 SDK, triggers.cpp/entityoutput):
	constexpr int kSfTriggerAllowClients = 0x01;   // players may fire the trigger
	constexpr int kSfTeleportPreserveAngles = 0x20;

#pragma pack(push, 1)
	struct dlump_t { int32_t fileofs; int32_t filelen; int32_t version; char fourCC[4]; };
	struct dheader_t { int32_t ident; int32_t version; dlump_t lumps[64]; int32_t revision; };
	struct dplane_t { float nx, ny, nz; float dist; int32_t type; };
	struct dbrush_t { int32_t firstside; int32_t numsides; int32_t contents; };
	struct dbrushside_t { uint16_t planenum; int16_t texinfo; int16_t dispinfo; int16_t bevel; };
	struct dnode_t {
		int32_t planenum; int32_t children[2];
		int16_t mins[3]; int16_t maxs[3];
		uint16_t firstface; uint16_t numfaces; int16_t area; int16_t padding;
	};
	struct dmodel_t {
		float mins[3], maxs[3], origin[3];
		int32_t headnode; int32_t firstface; int32_t numfaces;
	};
	// dleaf_t grew/shrank across versions: lump version 0 carries a 24-byte
	// ambient cube, version 1+ does not. Only the leafbrush span is needed and
	// it sits at the same offset in both, so the stride is all that differs.
	struct dleaf_common_t {
		int32_t contents; int16_t cluster; int16_t area_flags;
		int16_t mins[3]; int16_t maxs[3];
		uint16_t firstleafface; uint16_t numleaffaces;
		uint16_t firstleafbrush; uint16_t numleafbrushes;
		int16_t leafWaterDataID;
	};
#pragma pack(pop)

	// --- parsed world ---------------------------------------------------------
	struct Face {
		int plane;                    // dplane index, for later ramp tagging
		std::vector<Vector> pts;
	};
	struct Brush {
		int contents;
		Vector mins, maxs;
		std::vector<Face> faces;
		std::vector<int> clip_planes;   // ALL non-bevel side planes - the ray
		                                // clip needs the complete convex set even
		                                // when a sliver face's winding was dropped
		std::vector<int> bevel_planes;  // the compiler's EXACT bevel sides (axial
		                                // + edge bevels) - what CM_ClipBoxToBrush
		                                // actually clips swept boxes against; the
		                                // apex top-vs-face branch flips on their
		                                // exact dists, so approximations are out
		bool world = true;              // reachable from model 0 = real collision
		int  model = -1;                // BSP model this brush belongs to (-1 =
		                                // unreached; 0 = worldspawn; 1+ = entity)
		std::vector<std::pair<Vector, Vector>> edges;   // deduped outline
	};

	std::vector<Brush> g_brushes;
	std::vector<std::pair<Vector, float>> g_planes;   // dplane normals/dists (for pick/rest)
	std::string g_level;              // level name the cache was built for
	BspWorld::Status g_status;

	// Tags + board targets for the loaded map (persisted to a per-map .geo file).
	std::set<std::pair<int, int>> g_tags;             // (brush, plane)
	std::vector<BspWorld::BoardTarget> g_targets;

	// Geo undo (session 45): every destructive edit (remove, clear-all,
	// project import) snapshots the whole tag+target set first; UndoGeo
	// restores the newest snapshot. Session-scoped, depth-capped - the
	// same shape as the recordings tab's "Undo delete".
	struct GeoSnapshot {
		std::set<std::pair<int, int>> tags;
		std::vector<BspWorld::BoardTarget> targets;
	};
	std::vector<GeoSnapshot> g_geo_undo;   // newest last

	void PushGeoUndo() {
		GeoSnapshot snap;
		snap.tags = g_tags;
		snap.targets = g_targets;
		g_geo_undo.push_back(std::move(snap));
		if (g_geo_undo.size() > 8)
			g_geo_undo.erase(g_geo_undo.begin());
	}

	// --- triggers (parsed from the entity lump) ------------------------------
	struct TriggerVol {
		int    model = -1;            // "*N" brush model holding the volume
		bool   teleport = false;
		bool   push = false;          // neither = gravity
		int    spawnflags = 0;
		float  gravity = 1.f;         // trigger_gravity keyvalue (player scale)
		Vector push_dir;              // trigger_push: world-space direction
		float  push_speed = 0.f;      // trigger_push "speed"
		Vector ent_origin;            // entity "origin" key (brush offset; usually 0)
		// teleport destination, resolved at parse:
		bool   dest_ok = false;
		Vector dest_origin;
		float  dest_pitch = 0.f, dest_yaw = 0.f;
		bool   landmark_ok = false;   // landmark mode: dest + (player - landmark)
		Vector landmark_origin;
		Vector center;                // volume center (drawing)
		std::vector<int> brushes;     // member brush indices (per-tick test set)
	};
	std::vector<TriggerVol> g_trigvols;
	int g_trig_teleports = 0, g_trig_gravities = 0, g_trig_pushes = 0;
	std::vector<int> g_brush_model;   // parse-time: brush index -> model index

	// Last pick's ray + hit, drawn briefly for debugging.
	bool g_dbg_pick = false;
	ULONGLONG g_dbg_until = 0;
	Vector g_dbg_eye, g_dbg_end;
	bool g_dbg_hit = false;
	Vector g_dbg_normal;

	void LoadGeo();   // defined below (needs the level name helpers)

	void SetError(const char* message) {
		strncpy_s(g_status.error, message, _TRUNCATE);
	}

	// --- winding math (mirrors the offline-validated python) -------------------
	Vector Cross(const Vector& a, const Vector& b) {
		return Vector(a.Y * b.Z - a.Z * b.Y, a.Z * b.X - a.X * b.Z, a.X * b.Y - a.Y * b.X);
	}
	float Dot(const Vector& a, const Vector& b) { return a.X * b.X + a.Y * b.Y + a.Z * b.Z; }
	Vector Scale(const Vector& a, float s) { return Vector(a.X * s, a.Y * s, a.Z * s); }

	// A huge quad lying on the plane, to be chopped down by the other sides.
	void BaseWinding(const Vector& n, float d, std::vector<Vector>& out) {
		int axis = 0;
		float best = fabsf(n.X);
		if (fabsf(n.Y) < best) { best = fabsf(n.Y); axis = 1; }
		if (fabsf(n.Z) < best) { axis = 2; }
		Vector up(axis == 0 ? 1.f : 0.f, axis == 1 ? 1.f : 0.f, axis == 2 ? 1.f : 0.f);

		Vector right = Cross(up, n);
		const float len = right.Length();
		if (len < 1e-6f) { out.clear(); return; }
		right = Scale(right, 1.f / len);
		const Vector up2 = Cross(n, right);
		const Vector org = Scale(n, d);

		const float S = 65536.f;
		out.clear();
		out.push_back(org - Scale(right, S) + Scale(up2, S));
		out.push_back(org + Scale(right, S) + Scale(up2, S));
		out.push_back(org + Scale(right, S) - Scale(up2, S));
		out.push_back(org - Scale(right, S) - Scale(up2, S));
	}

	// Keep the half-space dot(n,p) - d <= eps (brush interiors are behind every
	// side plane; normals point outward).
	void Chop(std::vector<Vector>& w, const Vector& n, float d) {
		const float eps = 0.01f;
		static std::vector<Vector> out;
		out.clear();
		const size_t m = w.size();
		for (size_t i = 0; i < m; ++i) {
			const Vector& a = w[i];
			const Vector& b = w[(i + 1) % m];
			const float da = Dot(n, a) - d;
			const float db = Dot(n, b) - d;
			if (da <= eps)
				out.push_back(a);
			if ((da < -eps && db > eps) || (da > eps && db < -eps)) {
				const float t = da / (da - db);
				out.push_back(a + Scale(b - a, t));
			}
		}
		w = out;
	}

	long long EdgeKeyPart(const Vector& v) {
		// Quantize to a 0.25u grid for dedup.
		const int x = static_cast<int>(floorf(v.X * 4.f + 0.5f));
		const int y = static_cast<int>(floorf(v.Y * 4.f + 0.5f));
		const int z = static_cast<int>(floorf(v.Z * 4.f + 0.5f));
		return (static_cast<long long>(x) & 0x1FFFFF)
			| ((static_cast<long long>(y) & 0x1FFFFF) << 21)
			| ((static_cast<long long>(z) & 0x1FFFFF) << 42);
	}

	// Read the whole file with maximally permissive sharing - the engine keeps
	// its own handle on the loaded .bsp.
	bool ReadWholeFile(const std::string& path, std::vector<char>& out, char* error, size_t error_size) {
		HANDLE file = CreateFileA(path.c_str(), GENERIC_READ,
			FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
			nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
		if (file == INVALID_HANDLE_VALUE) {
			_snprintf_s(error, error_size, _TRUNCATE, "open failed (err %lu): %s", GetLastError(), path.c_str());
			return false;
		}

		LARGE_INTEGER size = {};
		if (!GetFileSizeEx(file, &size) || size.QuadPart <= 0 || size.QuadPart > (256ll << 20)) {
			_snprintf_s(error, error_size, _TRUNCATE, "bad file size: %s", path.c_str());
			CloseHandle(file);
			return false;
		}

		out.resize(static_cast<size_t>(size.QuadPart));
		DWORD read = 0;
		const bool ok = ReadFile(file, out.data(), static_cast<DWORD>(out.size()), &read, nullptr)
			&& read == out.size();
		CloseHandle(file);
		if (!ok)
			_snprintf_s(error, error_size, _TRUNCATE, "read failed (err %lu): %s", GetLastError(), path.c_str());
		return ok;
	}

	// --- entity lump: triggers + destinations --------------------------------
	// Plain text, repeated { "key" "value" ... } blocks - the same data the
	// server spawns entities from. Only the classes the sim needs are kept;
	// every named entity's origin/angles is collected so destination and
	// landmark lookup matches the server's FindEntityByName (any class).
	Vector ParseV3(const std::string& s) {
		Vector v(0.f, 0.f, 0.f);
		sscanf_s(s.c_str(), "%f %f %f", &v.X, &v.Y, &v.Z);
		return v;
	}

	void ParseEntities(const char* txt, size_t len) {
		g_trigvols.clear();
		g_trig_teleports = g_trig_gravities = g_trig_pushes = 0;
		if (!txt || !len)
			return;

		struct Ent {
			std::string classname, targetname, target, landmark;
			std::string model, origin, angles, pushdir;
			int spawnflags = 0;
			float gravity = 1.f;
			float speed = 0.f;
		};
		std::vector<Ent> trigs;
		struct Named { std::string name; Vector origin; float pitch, yaw; };
		std::vector<Named> named;

		size_t i = 0;
		int guard = 0;
		while (i < len && guard++ < 65536) {
			while (i < len && txt[i] != '{') ++i;
			if (i >= len) break;
			++i;
			Ent e;
			while (i < len && txt[i] != '}') {
				while (i < len && txt[i] != '"' && txt[i] != '}') ++i;
				if (i >= len || txt[i] == '}') break;
				++i;
				const size_t k0 = i;
				while (i < len && txt[i] != '"') ++i;
				std::string key(txt + k0, i - k0);
				if (i < len) ++i;
				while (i < len && txt[i] != '"' && txt[i] != '}') ++i;
				if (i >= len || txt[i] == '}') break;
				++i;
				const size_t v0 = i;
				while (i < len && txt[i] != '"') ++i;
				std::string val(txt + v0, i - v0);
				if (i < len) ++i;
				if (key == "classname") e.classname = val;
				else if (key == "targetname") e.targetname = val;
				else if (key == "target") e.target = val;
				else if (key == "landmark") e.landmark = val;
				else if (key == "model") e.model = val;
				else if (key == "origin") e.origin = val;
				else if (key == "angles") e.angles = val;
				else if (key == "spawnflags") e.spawnflags = atoi(val.c_str());
				else if (key == "gravity") e.gravity = static_cast<float>(atof(val.c_str()));
				else if (key == "pushdir") e.pushdir = val;
				else if (key == "speed") e.speed = static_cast<float>(atof(val.c_str()));
			}
			if (i < len) ++i;   // past '}'
			if (!e.targetname.empty() && !e.origin.empty()) {
				Named n;
				n.name = e.targetname;
				n.origin = ParseV3(e.origin);
				const Vector a = ParseV3(e.angles);   // "pitch yaw roll"
				n.pitch = a.X;
				n.yaw = a.Y;
				named.push_back(std::move(n));
			}
			if (e.classname == "trigger_teleport" || e.classname == "trigger_gravity"
				|| e.classname == "trigger_push")
				trigs.push_back(std::move(e));
		}

		for (const Ent& e : trigs) {
			if (e.model.size() < 2 || e.model[0] != '*')
				continue;   // no brush model: nothing to touch
			TriggerVol tv;
			tv.model = atoi(e.model.c_str() + 1);
			tv.spawnflags = e.spawnflags;
			tv.ent_origin = e.origin.empty() ? Vector(0.f, 0.f, 0.f) : ParseV3(e.origin);
			tv.teleport = (e.classname == "trigger_teleport");
			tv.push = (e.classname == "trigger_push");
			if (tv.teleport) {
				for (const Named& n : named)
					if (n.name == e.target) {
						tv.dest_ok = true;
						tv.dest_origin = n.origin;
						tv.dest_pitch = n.pitch;
						tv.dest_yaw = n.yaw;
						break;
					}
				if (!e.landmark.empty())
					for (const Named& n : named)
						if (n.name == e.landmark) {
							tv.landmark_ok = true;
							tv.landmark_origin = n.origin;
							break;
						}
				if (!tv.dest_ok)
					continue;   // unresolvable destination: skip the trigger
				g_trig_teleports++;
			} else if (tv.push) {
				// SDK: "pushdir" is angles; the world direction is the forward
				// vector (brush triggers carry no rotation of their own).
				const Vector a = ParseV3(e.pushdir);   // "pitch yaw roll"
				const float pr = a.X * 3.14159265f / 180.f;
				const float yr = a.Y * 3.14159265f / 180.f;
				tv.push_dir = Vector(cosf(pr) * cosf(yr), cosf(pr) * sinf(yr), -sinf(pr));
				tv.push_speed = e.speed;
				g_trig_pushes++;
			} else {
				tv.gravity = e.gravity;
				g_trig_gravities++;
			}
			g_trigvols.push_back(tv);
		}
	}

	bool LoadFromFile(const std::string& path) {
		g_brushes.clear();
		g_status = BspWorld::Status();

		const ULONGLONG t0 = GetTickCount64();

		std::vector<char> data;
		if (!ReadWholeFile(path, data, g_status.error, sizeof(g_status.error)))
			return false;
		if (data.size() < sizeof(dheader_t)) { SetError("file too small"); return false; }

		const dheader_t* header = reinterpret_cast<const dheader_t*>(data.data());
		if (header->ident != 0x50534256) { SetError("not a VBSP file"); return false; }
		if (header->version < 17 || header->version > 21) { SetError("unsupported BSP version"); return false; }

		auto lump_ok = [&](int index, size_t stride) -> bool {
			const dlump_t& l = header->lumps[index];
			return l.fileofs >= 0 && l.filelen >= 0
				&& static_cast<size_t>(l.fileofs) + static_cast<size_t>(l.filelen) <= data.size()
				&& (l.filelen % stride) == 0;
		};
		if (!lump_ok(kLumpPlanes, sizeof(dplane_t)) ||
			!lump_ok(kLumpBrushes, sizeof(dbrush_t)) ||
			!lump_ok(kLumpBrushSides, sizeof(dbrushside_t))) {
			SetError("lump table is inconsistent");
			return false;
		}

		const dplane_t* planes = reinterpret_cast<const dplane_t*>(data.data() + header->lumps[kLumpPlanes].fileofs);
		const int nplanes = header->lumps[kLumpPlanes].filelen / sizeof(dplane_t);
		const dbrush_t* brushes = reinterpret_cast<const dbrush_t*>(data.data() + header->lumps[kLumpBrushes].fileofs);
		const int nbrushes = header->lumps[kLumpBrushes].filelen / sizeof(dbrush_t);
		const dbrushside_t* sides = reinterpret_cast<const dbrushside_t*>(data.data() + header->lumps[kLumpBrushSides].fileofs);
		const int nsides = header->lumps[kLumpBrushSides].filelen / sizeof(dbrushside_t);

		g_planes.clear();
		g_planes.reserve(nplanes);
		for (int i = 0; i < nplanes; ++i)
			g_planes.push_back({ Vector(planes[i].nx, planes[i].ny, planes[i].nz), planes[i].dist });

		// Which brushes belong to the WORLD model? Walk model 0's BSP tree,
		// collect its leaves, and union their leafbrush spans - the same set
		// the engine traces movement against. Everything else is entity
		// geometry (trigger_teleport slabs, func_illusionary, ...) which is
		// CONTENTS_SOLID on disk but blocks nothing, and surf maps carpet the
		// void below the ramps with exactly those.
		std::vector<bool> world_brush(nbrushes, false);
		bool world_set_ok = false;
		if (lump_ok(kLumpModels, sizeof(dmodel_t)) && lump_ok(kLumpNodes, sizeof(dnode_t))
			&& lump_ok(kLumpLeafBrushes, sizeof(uint16_t))
			&& header->lumps[kLumpModels].filelen >= static_cast<int>(sizeof(dmodel_t))) {
			const dmodel_t* models = reinterpret_cast<const dmodel_t*>(
				data.data() + header->lumps[kLumpModels].fileofs);
			const dnode_t* nodes = reinterpret_cast<const dnode_t*>(
				data.data() + header->lumps[kLumpNodes].fileofs);
			const int nnodes = header->lumps[kLumpNodes].filelen / sizeof(dnode_t);
			const uint16_t* leafbrushes = reinterpret_cast<const uint16_t*>(
				data.data() + header->lumps[kLumpLeafBrushes].fileofs);
			const int nleafbrushes = header->lumps[kLumpLeafBrushes].filelen / sizeof(uint16_t);
			const dlump_t& ll = header->lumps[kLumpLeafs];
			// dleaf_t ON-DISK stride is the DOCUMENTED size, NOT sizeof() of a
			// packed struct: version 1 = 32 bytes (a 2-byte tail pad the packed
			// struct drops), version 0 = 56 (that pad + a 24-byte ambient cube).
			// The packed-30 sizeof was off by 2, so the walk read every leaf
			// misaligned, marked brushes at random, and the trigger floors
			// stayed in the corridor - which is exactly what broke ride solves
			// (search_162612: 752 present, entity_brushes_skipped=0). Verify the
			// chosen stride divides the lump; if not, try the other before
			// giving up (which safely falls back to all-world).
			size_t leaf_stride = (ll.version == 0) ? 56 : 32;
			if (ll.filelen > 0 && (static_cast<size_t>(ll.filelen) % leaf_stride) != 0) {
				const size_t alt = (leaf_stride == 32) ? 56 : 32;
				if ((static_cast<size_t>(ll.filelen) % alt) == 0)
					leaf_stride = alt;
			}
			const int nleafs = (ll.filelen > 0
				&& (static_cast<size_t>(ll.filelen) % leaf_stride) == 0)
				? static_cast<int>(static_cast<size_t>(ll.filelen) / leaf_stride) : 0;
			if (nleafs > 0 && ll.fileofs >= 0
				&& static_cast<size_t>(ll.fileofs) + static_cast<size_t>(ll.filelen) <= data.size()) {
				// Walk EVERY model's tree (not just worldspawn) and record which
				// model each brush belongs to: model 0 = the collision world,
				// 1+ = brush entities - which is exactly what trigger volumes
				// need ("*N" in the entity lump names model N's brush set).
				const int nmodels = header->lumps[kLumpModels].filelen
					/ static_cast<int>(sizeof(dmodel_t));
				g_brush_model.assign(nbrushes, -1);
				std::vector<int> stack;
				for (int m = 0; m < nmodels; ++m) {
					stack.clear();
					stack.push_back(models[m].headnode);
					int guard = 0;
					while (!stack.empty() && guard++ < 4000000) {
						const int idx = stack.back();
						stack.pop_back();
						if (idx >= 0) {
							if (idx >= nnodes) continue;
							stack.push_back(nodes[idx].children[0]);
							stack.push_back(nodes[idx].children[1]);
							continue;
						}
						const int leaf = -1 - idx;
						if (leaf < 0 || leaf >= nleafs) continue;
						const dleaf_common_t* lf = reinterpret_cast<const dleaf_common_t*>(
							data.data() + ll.fileofs + static_cast<size_t>(leaf) * leaf_stride);
						for (int b = 0; b < lf->numleafbrushes; ++b) {
							const int lbi = lf->firstleafbrush + b;
							if (lbi < 0 || lbi >= nleafbrushes) continue;
							const int br = leafbrushes[lbi];
							if (br >= 0 && br < nbrushes) {
								if (m == 0)
									world_brush[br] = true;
								if (g_brush_model[br] < 0)
									g_brush_model[br] = m;
							}
						}
					}
				}
				world_set_ok = true;
			}
		}

		g_tags.clear();
		g_targets.clear();

		g_brushes.reserve(nbrushes);
		std::vector<std::pair<Vector, float>> brush_planes;
		std::vector<int> brush_plane_ids;
		std::vector<Vector> winding;

		int face_total = 0, edge_total = 0;
		for (int bi = 0; bi < nbrushes; ++bi) {
			const dbrush_t& db = brushes[bi];
			if (db.firstside < 0 || db.numsides <= 0 || db.firstside + db.numsides > nsides)
				continue;

			brush_planes.clear();
			brush_plane_ids.clear();
			std::vector<int> bevel_ids;
			for (int s = db.firstside; s < db.firstside + db.numsides; ++s) {
				if (sides[s].planenum >= nplanes)
					continue;
				if (sides[s].bevel) {
					// AXIAL bevel sides only: the compiler's exact box-extent
					// planes (the apex top-vs-face branch flips on their exact
					// dists - winding AABBs were 0.004 off). The NON-axial edge
					// bevels are NOT what the measured engine clips against:
					// loading them deflected the real log's proven 355.2 winner
					// off a slanted apex "lid" (nz .94) mid-flight to 401 - while
					// months of dpos=0.000 traces validate regular sides + axial
					// extents as the engine's effective swept-hull behavior.
					const dplane_t& bp = planes[sides[s].planenum];
					const float ax = fabsf(bp.nx), ay = fabsf(bp.ny), az = fabsf(bp.nz);
					if (ax > 0.999f || ay > 0.999f || az > 0.999f)
						bevel_ids.push_back(sides[s].planenum);
					continue;
				}
				const dplane_t& p = planes[sides[s].planenum];
				brush_planes.push_back({ Vector(p.nx, p.ny, p.nz), p.dist });
				brush_plane_ids.push_back(sides[s].planenum);
			}
			if (brush_planes.size() < 4 || brush_planes.size() > 128)
				continue;

			Brush brush;
			brush.contents = db.contents;
			brush.clip_planes = brush_plane_ids;
			brush.bevel_planes = bevel_ids;
			// If the world set couldn't be built, assume every brush is world
			// (the old behavior) rather than silently emptying the map.
			brush.world = world_set_ok ? world_brush[bi] : true;
			brush.model = (world_set_ok && bi < static_cast<int>(g_brush_model.size()))
				? g_brush_model[bi] : (brush.world ? 0 : -1);
			brush.mins = Vector(1e9f, 1e9f, 1e9f);
			brush.maxs = Vector(-1e9f, -1e9f, -1e9f);

			std::set<std::pair<long long, long long>> edge_keys;
			for (size_t i = 0; i < brush_planes.size(); ++i) {
				BaseWinding(brush_planes[i].first, brush_planes[i].second, winding);
				for (size_t j = 0; j < brush_planes.size() && !winding.empty(); ++j) {
					if (j == i)
						continue;
					Chop(winding, brush_planes[j].first, brush_planes[j].second);
				}
				if (winding.size() < 3)
					continue;

				Face face;
				face.plane = brush_plane_ids[i];
				face.pts = winding;
				for (const Vector& p : winding) {
					brush.mins.X = (std::min)(brush.mins.X, p.X);
					brush.mins.Y = (std::min)(brush.mins.Y, p.Y);
					brush.mins.Z = (std::min)(brush.mins.Z, p.Z);
					brush.maxs.X = (std::max)(brush.maxs.X, p.X);
					brush.maxs.Y = (std::max)(brush.maxs.Y, p.Y);
					brush.maxs.Z = (std::max)(brush.maxs.Z, p.Z);
				}

				// Outline edges, deduped across the brush's faces.
				for (size_t k = 0; k < winding.size(); ++k) {
					const Vector& a = winding[k];
					const Vector& b = winding[(k + 1) % winding.size()];
					long long ka = EdgeKeyPart(a), kb = EdgeKeyPart(b);
					const std::pair<long long, long long> key =
						(ka < kb) ? std::make_pair(ka, kb) : std::make_pair(kb, ka);
					if (edge_keys.insert(key).second)
						brush.edges.push_back({ a, b });
				}
				brush.faces.push_back(std::move(face));
			}

			if (!brush.faces.empty()) {
				face_total += static_cast<int>(brush.faces.size());
				edge_total += static_cast<int>(brush.edges.size());
				g_brushes.push_back(std::move(brush));
			}
		}

		// Triggers: parse the entity lump, then index each trigger's member
		// brushes + volume center from the model-tagged brush list.
		{
			const dlump_t& el = header->lumps[kLumpEntities];
			if (el.fileofs >= 0 && el.filelen > 0
				&& static_cast<size_t>(el.fileofs) + static_cast<size_t>(el.filelen) <= data.size())
				ParseEntities(reinterpret_cast<const char*>(data.data() + el.fileofs),
					static_cast<size_t>(el.filelen));
			else
				ParseEntities(nullptr, 0);
			for (TriggerVol& tv : g_trigvols) {
				Vector mn(1e9f, 1e9f, 1e9f), mx(-1e9f, -1e9f, -1e9f);
				for (int bi2 = 0; bi2 < static_cast<int>(g_brushes.size()); ++bi2) {
					const Brush& b = g_brushes[bi2];
					if (b.model != tv.model)
						continue;
					tv.brushes.push_back(bi2);
					if (b.mins.X < mn.X) mn.X = b.mins.X;
					if (b.mins.Y < mn.Y) mn.Y = b.mins.Y;
					if (b.mins.Z < mn.Z) mn.Z = b.mins.Z;
					if (b.maxs.X > mx.X) mx.X = b.maxs.X;
					if (b.maxs.Y > mx.Y) mx.Y = b.maxs.Y;
					if (b.maxs.Z > mx.Z) mx.Z = b.maxs.Z;
				}
				tv.center = tv.brushes.empty()
					? tv.ent_origin
					: Vector((mn.X + mx.X) * 0.5f + tv.ent_origin.X,
					         (mn.Y + mx.Y) * 0.5f + tv.ent_origin.Y,
					         (mn.Z + mx.Z) * 0.5f + tv.ent_origin.Z);
			}
		}

		g_status.loaded = true;
		g_status.bsp_version = header->version;
		g_status.brush_count = static_cast<int>(g_brushes.size());
		g_status.face_count = face_total;
		g_status.edge_count = edge_total;
		g_status.parse_ms = static_cast<float>(GetTickCount64() - t0);
		return true;
	}

	// GetLevelName through a guard: the vtable index comes from the original SDK
	// header and is otherwise unused, so validate the result before trusting it.
	const char* RawLevelName() {
		__try {
			return GetVirtualFunction<const char*(*)(IVEngineClient*)>(engine, 51)(engine);
		}
		__except (EXCEPTION_EXECUTE_HANDLER) {
			return nullptr;
		}
	}

	bool SafeLevelName(std::string& out) {
		const char* raw = RawLevelName();
		if (!raw)
			return false;
		char buffer[192] = {};
		__try {
			for (int i = 0; i < 190 && raw[i]; ++i) {
				if (static_cast<unsigned char>(raw[i]) < 32 || static_cast<unsigned char>(raw[i]) > 126)
					return false;
				buffer[i] = raw[i];
			}
		}
		__except (EXCEPTION_EXECUTE_HANDLER) {
			return false;
		}
		out = buffer;
		// Expected form: "maps/<name>.bsp"
		return out.size() > 8 && out.size() < 160
			&& out.compare(0, 5, "maps/") == 0
			&& out.compare(out.size() - 4, 4, ".bsp") == 0;
	}

	// IVEngineClient::GetGameDirectory - the absolute "...\cstrike" path. This
	// build's engine vtable is the 2013 layout shifted +1 (four verified
	// anchors: 12/20/26/51), putting GetGameDirectory at 35. Guarded and
	// validated like GetLevelName, so a wrong slot degrades to a fallback.
	const char* RawGameDir() {
		__try {
			return GetVirtualFunction<const char*(*)(IVEngineClient*)>(engine, 35)(engine);
		}
		__except (EXCEPTION_EXECUTE_HANDLER) {
			return nullptr;
		}
	}

	bool SafeGameDir(std::string& out) {
		const char* raw = RawGameDir();
		if (!raw)
			return false;
		char buffer[256] = {};
		__try {
			for (int i = 0; i < 250 && raw[i]; ++i) {
				if (static_cast<unsigned char>(raw[i]) < 32 || static_cast<unsigned char>(raw[i]) > 126)
					return false;
				buffer[i] = raw[i];
			}
		}
		__except (EXCEPTION_EXECUTE_HANDLER) {
			return false;
		}
		out = buffer;
		while (!out.empty() && (out.back() == '\\' || out.back() == '/'))
			out.pop_back();
		// Expect an absolute path to an existing directory.
		if (out.size() < 4 || out.size() > 240 || out[1] != ':')
			return false;
		const DWORD attributes = GetFileAttributesA(out.c_str());
		return attributes != INVALID_FILE_ATTRIBUTES && (attributes & FILE_ATTRIBUTE_DIRECTORY);
	}

	// SteamPipe mounts maps from several roots under the game dir: maps\,
	// download\maps\ (server-downloaded customs - where surf maps usually
	// live), and custom\<anything>\maps\.
	void AddSearchRoots(const std::string& base, const std::string& rel, std::vector<std::string>& out) {
		out.push_back(base + "\\" + rel);
		out.push_back(base + "\\download\\" + rel);

		WIN32_FIND_DATAA find = {};
		HANDLE handle = FindFirstFileA((base + "\\custom\\*").c_str(), &find);
		if (handle != INVALID_HANDLE_VALUE) {
			int added = 0;
			do {
				if ((find.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
					&& strcmp(find.cFileName, ".") != 0 && strcmp(find.cFileName, "..") != 0
					&& added < 16) {
					out.push_back(base + "\\custom\\" + find.cFileName + "\\" + rel);
					added++;
				}
			} while (FindNextFileA(handle, &find));
			FindClose(handle);
		}
	}

	// --- per-map tag/target persistence --------------------------------------
	std::string GeoFilePathFor(const std::string& level) {
		if (level.empty())
			return {};
		char documents[MAX_PATH];
		if (FAILED(SHGetFolderPathA(nullptr, CSIDL_PERSONAL, nullptr, SHGFP_TYPE_CURRENT, documents)))
			return {};
		std::string base = std::string(documents) + "\\sourceTAS";
		CreateDirectoryA(base.c_str(), nullptr);
		std::string dir = base + "\\geometry";
		CreateDirectoryA(dir.c_str(), nullptr);

		std::string stem = level;                       // "maps/x.bsp" -> "x"
		const size_t slash = stem.find_last_of("/\\");
		if (slash != std::string::npos)
			stem = stem.substr(slash + 1);
		const size_t dot = stem.find_last_of('.');
		if (dot != std::string::npos)
			stem = stem.substr(0, dot);
		std::string safe;
		for (char c : stem)
			safe += std::strchr("\\/:*?\"<>|", c) ? '_' : c;
		if (safe.empty())
			safe = "map";
		return dir + "\\" + safe + ".geo";
	}

	constexpr char kGeoMagic[4] = { 'S', 'T', 'G', 'E' };
	// v2: targets append pitch_lock.
	constexpr uint32_t kGeoVersion = 2;

	void SaveGeoImpl() {
		const std::string path = GeoFilePathFor(g_level);
		if (path.empty())
			return;
		std::ofstream out(path, std::ios::binary | std::ios::trunc);
		if (!out)
			return;
		out.write(kGeoMagic, sizeof(kGeoMagic));
		out.write(reinterpret_cast<const char*>(&kGeoVersion), sizeof(kGeoVersion));

		const uint32_t ntags = static_cast<uint32_t>(g_tags.size());
		out.write(reinterpret_cast<const char*>(&ntags), sizeof(ntags));
		for (const std::pair<int, int>& tag : g_tags) {
			const int32_t brush = tag.first, plane = tag.second;
			out.write(reinterpret_cast<const char*>(&brush), sizeof(brush));
			out.write(reinterpret_cast<const char*>(&plane), sizeof(plane));
		}

		const uint32_t ntargets = static_cast<uint32_t>(g_targets.size());
		out.write(reinterpret_cast<const char*>(&ntargets), sizeof(ntargets));
		for (const BspWorld::BoardTarget& t : g_targets) {
			out.write(reinterpret_cast<const char*>(&t.pos), sizeof(float) * 3);
			out.write(reinterpret_cast<const char*>(&t.yaw), sizeof(t.yaw));
			out.write(reinterpret_cast<const char*>(&t.pitch), sizeof(t.pitch));
			const uint8_t ducked = t.ducked ? 1 : 0;
			out.write(reinterpret_cast<const char*>(&ducked), sizeof(ducked));
			const int32_t brush = t.brush, plane = t.plane;
			out.write(reinterpret_cast<const char*>(&brush), sizeof(brush));
			out.write(reinterpret_cast<const char*>(&plane), sizeof(plane));
			const uint8_t lock = t.pitch_lock ? 1 : 0;
			out.write(reinterpret_cast<const char*>(&lock), sizeof(lock));
		}
	}

	void LoadGeo() {
		g_tags.clear();
		g_targets.clear();
		const std::string path = GeoFilePathFor(g_level);
		if (path.empty())
			return;
		std::ifstream in(path, std::ios::binary);
		if (!in)
			return;
		char magic[4];
		uint32_t version = 0;
		if (!in.read(magic, 4) || memcmp(magic, kGeoMagic, 4) != 0
			|| !in.read(reinterpret_cast<char*>(&version), sizeof(version))
			|| version < 1 || version > kGeoVersion)
			return;

		uint32_t ntags = 0;
		if (!in.read(reinterpret_cast<char*>(&ntags), sizeof(ntags)) || ntags > 100000)
			return;
		for (uint32_t i = 0; i < ntags; ++i) {
			int32_t brush = 0, plane = 0;
			if (!in.read(reinterpret_cast<char*>(&brush), sizeof(brush)) ||
				!in.read(reinterpret_cast<char*>(&plane), sizeof(plane)))
				return;
			g_tags.insert({ brush, plane });
		}

		uint32_t ntargets = 0;
		if (!in.read(reinterpret_cast<char*>(&ntargets), sizeof(ntargets)) || ntargets > 100000)
			return;
		for (uint32_t i = 0; i < ntargets; ++i) {
			BspWorld::BoardTarget t;
			uint8_t ducked = 0;
			int32_t brush = 0, plane = 0;
			if (!in.read(reinterpret_cast<char*>(&t.pos), sizeof(float) * 3) ||
				!in.read(reinterpret_cast<char*>(&t.yaw), sizeof(t.yaw)) ||
				!in.read(reinterpret_cast<char*>(&t.pitch), sizeof(t.pitch)) ||
				!in.read(reinterpret_cast<char*>(&ducked), sizeof(ducked)) ||
				!in.read(reinterpret_cast<char*>(&brush), sizeof(brush)) ||
				!in.read(reinterpret_cast<char*>(&plane), sizeof(plane)))
				return;
			t.ducked = ducked != 0;
			t.brush = brush;
			t.plane = plane;
			if (version >= 2) {
				uint8_t lock = 0;
				if (!in.read(reinterpret_cast<char*>(&lock), sizeof(lock)))
					return;
				t.pitch_lock = lock != 0;
			}
			g_targets.push_back(t);
		}
	}

	// Centroid of the hull corners that touch a plane with normal `n` when the
	// hull rests on it, as an offset from the hull origin. Per axis: the
	// touching corners take the min-dot side; a ~zero normal component means
	// both sides tie, so the average (their midpoint) is used - a single corner
	// on a general slope, the touching edge's center on a ramp, the bottom-face
	// center on flat ground.
	Vector ContactOffset(const Vector& n, float height) {
		const float eps = 1e-4f;
		return Vector(
			n.X > eps ? -16.f : n.X < -eps ? 16.f : 0.f,
			n.Y > eps ? -16.f : n.Y < -eps ? 16.f : 0.f,
			n.Z > eps ? 0.f : n.Z < -eps ? height : height * 0.5f);
	}

	// Rest the player AABB against a plane so that the CONTACT CENTROID lands
	// exactly at `contact_point` (the aimed point) - not the hull origin. The
	// touching corners all share the minimal plane distance, so translating the
	// hull by (contact_point - centroid) keeps it tangent with the contact
	// centered where aimed.
	bool RestOnPlane(int plane, bool ducked, const Vector& contact_point, Vector& out) {
		if (plane < 0 || plane >= static_cast<int>(g_planes.size()))
			return false;
		const Vector n = g_planes[plane].first;
		const float d = g_planes[plane].second;
		const float height = ducked ? 54.f : 72.f;

		// Snap the anchor onto the plane (the aim point already is; re-rests
		// pass a derived anchor).
		const Vector on_plane = contact_point - Scale(n, Dot(n, contact_point) - d);
		out = on_plane - ContactOffset(n, height);
		return true;
	}

	// Tagged-face outlines (gold, with a normal stub) + board targets (magenta
	// hull, origin dot, view arrow). Drawn whenever markers are on - these are
	// the solver's inputs, not debug geometry.
	void RenderMarkers(float duration) {
		int marker_lines = 0;
		int tag_index = 0;
		for (const std::pair<int, int>& tag : g_tags) {
			if (marker_lines >= 700)
				break;
			const bool emphasized = (tag_index++ == BspWorld::highlight_tag);
			if (tag.first < 0 || tag.first >= static_cast<int>(g_brushes.size()))
				continue;
			const int r = emphasized ? 255 : 255;
			const int g = emphasized ? 245 : 200;
			const int b = emphasized ? 130 : 40;
			for (const Face& face : g_brushes[tag.first].faces) {
				if (face.plane != tag.second || face.pts.size() < 3)
					continue;
				Vector centroid(0.f, 0.f, 0.f);
				for (size_t k = 0; k < face.pts.size(); ++k) {
					debugoverlay->AddLineOverlay(face.pts[k], face.pts[(k + 1) % face.pts.size()],
						r, g, b, false, duration);
					marker_lines++;
					centroid = centroid + face.pts[k];
				}
				centroid = Scale(centroid, 1.f / static_cast<float>(face.pts.size()));
				if (face.plane < static_cast<int>(g_planes.size()))
					debugoverlay->AddLineOverlay(centroid,
						centroid + Scale(g_planes[face.plane].first, emphasized ? 40.f : 24.f),
						r, g, b, false, duration);
			}
		}

		// Debug ray from the last pick: shows exactly where it went.
		if (g_dbg_pick && GetTickCount64() < g_dbg_until) {
			debugoverlay->AddLineOverlay(g_dbg_eye, g_dbg_end, 255, 60, 180, false, duration);
			debugoverlay->AddBoxOverlay(g_dbg_end, Vector(-2.f, -2.f, -2.f), Vector(2.f, 2.f, 2.f),
				Vector(0.f, 0.f, 0.f), 255, 60, 180, 255, duration);
			if (g_dbg_hit)
				debugoverlay->AddLineOverlay(g_dbg_end, g_dbg_end + Scale(g_dbg_normal, 32.f),
					255, 120, 220, false, duration);
		}

		for (int i = 0; i < static_cast<int>(g_targets.size()); ++i) {
			const BspWorld::BoardTarget& t = g_targets[i];
			const bool selected = (i == BspWorld::highlight_target);
			const float height = t.ducked ? 54.f : 72.f;
			const Vector no_rotation(0.f, 0.f, 0.f);
			debugoverlay->AddBoxOverlay(t.pos, Vector(-16.f, -16.f, 0.f), Vector(16.f, 16.f, height),
				no_rotation, 230, 70, 255, selected ? 70 : 30, duration);
			debugoverlay->AddBoxOverlay(t.pos, Vector(-1.5f, -1.5f, -1.5f), Vector(1.5f, 1.5f, 1.5f),
				no_rotation, 230, 70, 255, 255, duration);

			// Pitch-locked targets preview with the face-parallel pitch, so the
			// arrow rides the ramp surface no matter the yaw.
			float pitch = t.pitch;
			if (t.pitch_lock)
				BspWorld::LockedPitch(t.plane, t.yaw, &pitch);
			const float deg2rad = 3.14159265f / 180.f;
			const float pr = pitch * deg2rad;
			const float yr = t.yaw * deg2rad;
			const Vector dir(cosf(pr) * cosf(yr), cosf(pr) * sinf(yr), -sinf(pr));
			const Vector tip(t.pos.X + dir.X * 40.f, t.pos.Y + dir.Y * 40.f, t.pos.Z + dir.Z * 40.f);
			debugoverlay->AddLineOverlay(t.pos, tip, selected ? 255 : 200, 70, 255, false, duration);

			// Per-corner view lines (shares the hull-corner toggles): shows what
			// the view direction does at each bottom corner touching the ramp.
			static const Vector kCorner[4] = {
				Vector(16.f, 16.f, 0.f), Vector(16.f, -16.f, 0.f),
				Vector(-16.f, 16.f, 0.f), Vector(-16.f, -16.f, 0.f),
			};
			for (int c = 0; c < 4; ++c) {
				if (!WorldDraw::corner_trails[c])
					continue;
				const Vector base = t.pos + kCorner[c];
				debugoverlay->AddLineOverlay(base,
					base + Scale(dir, 40.f), selected ? 200 : 150, 110, 230, false, duration);
			}
		}
	}

	// Candidate .bsp locations, best-informed first: the engine's own game dir,
	// then exe-relative guesses (in case the exe doesn't sit in the game root).
	void MapFileCandidates(const std::string& level, std::vector<std::string>& out) {
		out.clear();

		std::string rel = level;
		for (char& c : rel)
			if (c == '/')
				c = '\\';

		std::string gamedir;
		if (SafeGameDir(gamedir))
			AddSearchRoots(gamedir, rel, out);

		char exe[MAX_PATH] = {};
		if (GetModuleFileNameA(nullptr, exe, sizeof(exe))) {
			std::string dir = exe;
			const size_t slash = dir.find_last_of("\\/");
			if (slash != std::string::npos) {
				dir = dir.substr(0, slash);
				// exe dir, then up to two parents, each treating \cstrike as base.
				for (int up = 0; up < 3; ++up) {
					const std::string base = dir + "\\cstrike";
					if (gamedir.empty() || _stricmp(base.c_str(), gamedir.c_str()) != 0)
						AddSearchRoots(base, rel, out);
					const size_t parent = dir.find_last_of("\\/");
					if (parent == std::string::npos)
						break;
					dir = dir.substr(0, parent);
				}
			}
		}
	}
}

bool BspWorld::LoadCurrentMap() {
	std::string level;
	if (!engine || !engine->IsInGame() || !SafeLevelName(level)) {
		SetError("not in game / couldn't resolve level name");
		return false;
	}

	std::vector<std::string> candidates;
	MapFileCandidates(level, candidates);
	if (candidates.empty()) {
		SetError("couldn't build any map path candidate");
		return false;
	}

	char last_error[256] = "no candidates tried";
	for (const std::string& path : candidates) {
		if (LoadFromFile(path)) {
			g_level = level;
			strncpy_s(g_status.map, level.c_str(), _TRUNCATE);
			LoadGeo();   // restore this map's tags + board targets
			return true;
		}
		strncpy_s(last_error, g_status.error, _TRUNCATE);
	}

	strncpy_s(g_status.error, last_error, _TRUNCATE);
	return false;
}

void BspWorld::Unload() {
	g_brushes.clear();
	g_planes.clear();
	g_tags.clear();
	g_targets.clear();
	g_trigvols.clear();
	g_brush_model.clear();
	g_trig_teleports = g_trig_gravities = g_trig_pushes = 0;
	g_level.clear();
	g_status = Status();
}

// Hull-vs-trigger-volume, engine-style: the brush planes (regular + the
// compiler's exact bevels) expanded by the hull via the support corner - with
// bevels included this IS the Minkowski sum, the same math CM_BoxTrace uses,
// so the touch condition matches the engine's trigger touch exactly.
bool BspWorld::CheckTriggers(const Vector& origin, const Vector& mins,
                             const Vector& maxs, TriggerHit* out) {
	if (!apply_triggers || g_trigvols.empty() || !g_status.loaded || !out)
		return false;
	bool any = false;
	for (const TriggerVol& tv : g_trigvols) {
		// SDK: players only fire triggers with the Clients spawnflag.
		if (!(tv.spawnflags & kSfTriggerAllowClients))
			continue;
		// Entity "origin" offsets the brush model: test in local space.
		const Vector p(origin.X - tv.ent_origin.X,
		               origin.Y - tv.ent_origin.Y,
		               origin.Z - tv.ent_origin.Z);
		bool touching = false;
		for (const int bi : tv.brushes) {
			const Brush& b = g_brushes[bi];
			bool inside = true;
			auto plane_ok = [&](int pid) {
				if (pid < 0 || pid >= static_cast<int>(g_planes.size()))
					return true;
				const Vector& n = g_planes[pid].first;
				const float d = g_planes[pid].second;
				// Support corner minimizing n.x over the hull.
				const float cx = p.X + (n.X > 0.f ? mins.X : maxs.X);
				const float cy = p.Y + (n.Y > 0.f ? mins.Y : maxs.Y);
				const float cz = p.Z + (n.Z > 0.f ? mins.Z : maxs.Z);
				return n.X * cx + n.Y * cy + n.Z * cz - d <= 0.f;
			};
			for (const int pid : b.clip_planes)
				if (!plane_ok(pid)) { inside = false; break; }
			if (inside)
				for (const int pid : b.bevel_planes)
					if (!plane_ok(pid)) { inside = false; break; }
			if (inside) { touching = true; break; }
		}
		if (!touching)
			continue;
		any = true;
		if (tv.teleport && !out->teleported) {
			// SDK CTriggerTeleport::Touch: landmark keeps the player's offset
			// from the landmark and never sets angles; otherwise destination
			// origin (player origin is at the feet, WorldAlignMins().z == 0,
			// so no z adjustment) and destination angles unless the preserve
			// spawnflag. Velocity is ALWAYS zeroed (vec3_origin is passed).
			Vector dst = tv.dest_origin;
			if (tv.landmark_ok) {
				dst.X += origin.X - tv.landmark_origin.X;
				dst.Y += origin.Y - tv.landmark_origin.Y;
				dst.Z += origin.Z - tv.landmark_origin.Z;
				out->tp_set_angles = false;
			} else {
				out->tp_set_angles = !(tv.spawnflags & kSfTeleportPreserveAngles);
				out->tp_pitch = tv.dest_pitch;
				out->tp_yaw = tv.dest_yaw;
			}
			out->teleported = true;
			out->tp_origin = dst;
		} else if (tv.push) {
			// SDK CTriggerPush::Touch: base velocity = pushdir * speed (the
			// FL_BASEVELOCITY accumulation happens at the apply site, which
			// knows the player's current flags/base velocity).
			out->pushed = true;
			out->push_vec = Vector(tv.push_dir.X * tv.push_speed,
			                       tv.push_dir.Y * tv.push_speed,
			                       tv.push_dir.Z * tv.push_speed);
		} else if (!tv.teleport) {
			out->grav_touched = true;
			out->gravity = tv.gravity;
		}
	}
	return any;
}

int BspWorld::TriggerTeleportCount() { return g_trig_teleports; }
int BspWorld::TriggerGravityCount() { return g_trig_gravities; }
int BspWorld::TriggerPushCount() { return g_trig_pushes; }

void BspWorld::Render(const Vector& center, bool have_center, float duration) {
	g_status.drawn_brushes = 0;
	g_status.drawn_lines = 0;

	if (!engine || !debugoverlay || !engine->IsInGame())
		return;
	if (!draw_wireframe && !show_markers && !show_triggers)
		return;

	// Teleport destination arrows: volume center -> destination, plus a small
	// box at the landing point. Budgeted like everything else. Gated by the
	// per-type toggles so a hidden type hides its links too.
	if (show_triggers && show_trig_links && g_status.loaded) {
		for (const TriggerVol& tv : g_trigvols) {
			if (tv.teleport && tv.dest_ok && show_trig_tp) {
				if (!WorldDraw::OverlayTake(2))
					break;
				debugoverlay->AddLineOverlay(tv.center, tv.dest_origin,
					255, 170, 40, false, duration);
				debugoverlay->AddBoxOverlay(tv.dest_origin,
					Vector(-4.f, -4.f, 0.f), Vector(4.f, 4.f, 8.f), Vector(0.f, 0.f, 0.f),
					255, 170, 40, 90, duration);
			} else if (tv.push && tv.push_speed > 0.f && show_trig_push) {
				// Booster direction ray from the volume center.
				if (!WorldDraw::OverlayTake(1))
					break;
				float len = tv.push_speed * 0.25f;
				if (len > 160.f) len = 160.f;
				debugoverlay->AddLineOverlay(tv.center,
					Vector(tv.center.X + tv.push_dir.X * len,
					       tv.center.Y + tv.push_dir.Y * len,
					       tv.center.Z + tv.push_dir.Z * len),
					90, 255, 130, false, duration);
			}
		}
	}

	// Trigger VOLUME wires, per type, drawn INDEPENDENTLY of the full brush
	// wireframe - so a booster route stays visible without paying for the
	// whole map's lines. Same radius + shared overlay budget as everything
	// else. The generic wireframe loop below skips these brushes to avoid a
	// double draw.
	if (show_triggers && g_status.loaded && have_center) {
		bool budget_dry = false;
		for (const TriggerVol& tv : g_trigvols) {
			if (budget_dry)
				break;
			const bool type_on = tv.teleport ? show_trig_tp
				: tv.push ? show_trig_push : show_trig_grav;
			if (!type_on)
				continue;
			int r = 80, g = 170, b = 255;                        // gravity: blue
			if (tv.teleport)  { r = 255; g = 170; b = 40; }      // gold
			else if (tv.push) { r = 90;  g = 255; b = 130; }     // green
			for (int bi : tv.brushes) {
				if (bi < 0 || bi >= static_cast<int>(g_brushes.size()))
					continue;
				const Brush& brush = g_brushes[bi];
				float dist2 = 0.f;
				const float cx[3] = { center.X, center.Y, center.Z };
				const float mn[3] = { brush.mins.X, brush.mins.Y, brush.mins.Z };
				const float mx[3] = { brush.maxs.X, brush.maxs.Y, brush.maxs.Z };
				for (int a = 0; a < 3; ++a) {
					float dd = 0.f;
					if (cx[a] < mn[a]) dd = mn[a] - cx[a];
					else if (cx[a] > mx[a]) dd = cx[a] - mx[a];
					dist2 += dd * dd;
				}
				if (dist2 > draw_radius * draw_radius)
					continue;
				if (!WorldDraw::OverlayTake(static_cast<int>(brush.edges.size()))) {
					budget_dry = true;
					break;
				}
				for (const std::pair<Vector, Vector>& edge : brush.edges)
					debugoverlay->AddLineOverlay(edge.first, edge.second, r, g, b, false, duration);
				g_status.drawn_brushes++;
				g_status.drawn_lines += static_cast<int>(brush.edges.size());
			}
		}
	}

	// Auto-(re)load when the level changes. Markers alone only trigger a parse
	// when this map has saved geometry, so uninvolved maps never pay for it.
	std::string level;
	if (SafeLevelName(level) && level != g_level) {
		bool has_geo = false;
		const std::string geo = GeoFilePathFor(level);
		if (!geo.empty())
			has_geo = GetFileAttributesA(geo.c_str()) != INVALID_FILE_ATTRIBUTES;
		if (draw_wireframe || has_geo)
			LoadCurrentMap();
	}
	if (g_brushes.empty())
		return;

	if (show_markers)
		RenderMarkers(duration);

	if (!draw_wireframe || !have_center)
		return;

	const float radius = draw_radius;
	const int budget = line_budget;

	// Nearest brushes first, so the budget spends where the player is looking.
	static std::vector<std::pair<float, int>> order;
	order.clear();
	for (int i = 0; i < static_cast<int>(g_brushes.size()); ++i) {
		const Brush& brush = g_brushes[i];

		const bool clip = (brush.contents & BSP_CONTENTS_PLAYERCLIP) != 0;
		const bool ladder = (brush.contents & BSP_CONTENTS_LADDER) != 0;
		const bool solid = (brush.contents & (BSP_CONTENTS_SOLID | BSP_CONTENTS_GRATE)) != 0;
		if (!((solid && show_solid) || (clip && show_playerclip) || (ladder && show_ladder)))
			continue;

		// Distance from center to the AABB (0 when inside).
		float dist2 = 0.f;
		const float cx[3] = { center.X, center.Y, center.Z };
		const float mn[3] = { brush.mins.X, brush.mins.Y, brush.mins.Z };
		const float mx[3] = { brush.maxs.X, brush.maxs.Y, brush.maxs.Z };
		for (int a = 0; a < 3; ++a) {
			float d = 0.f;
			if (cx[a] < mn[a]) d = mn[a] - cx[a];
			else if (cx[a] > mx[a]) d = cx[a] - mx[a];
			dist2 += d * d;
		}
		if (dist2 > radius * radius)
			continue;
		order.push_back({ dist2, i });
	}
	std::sort(order.begin(), order.end());

	int lines = 0;
	for (const std::pair<float, int>& entry : order) {
		const Brush& brush = g_brushes[entry.second];
		if (lines + static_cast<int>(brush.edges.size()) > budget)
			break;
		// Wires spend from the SHARED per-frame overlay budget too (they draw
		// after the run line, so the line always wins the contention).
		if (!WorldDraw::OverlayTake(static_cast<int>(brush.edges.size())))
			break;

		int r = 200, g = 200, b = 210;                       // solid: light gray
		if (brush.contents & BSP_CONTENTS_PLAYERCLIP) { r = 255; g = 80;  b = 80; }
		else if (brush.contents & BSP_CONTENTS_LADDER) { r = 80;  g = 255; b = 120; }
		else if (brush.contents & BSP_CONTENTS_GRATE) { r = 90;  g = 200; b = 255; }
		// BRUSH ENTITIES (triggers, illusionaries) are CONTENTS_SOLID on disk
		// but the engine never traces movement against them, so they are NOT
		// in the physics model either. Draw them dim purple so a wireframe
		// slab can never be mistaken for a floor that the solver ignores.
		// Recognized TRIGGER volumes are the dedicated per-type pass's job
		// while Show triggers is on (skipped here - a disabled type stays
		// hidden); with the master off they fall back to dim purple.
		if (!brush.world) {
			r = 90; g = 60; b = 110;
			if (show_triggers) {
				bool is_trigger = false;
				for (const TriggerVol& tv : g_trigvols)
					if (tv.model == brush.model) { is_trigger = true; break; }
				if (is_trigger)
					continue;
			}
		}

		for (const std::pair<Vector, Vector>& edge : brush.edges) {
			debugoverlay->AddLineOverlay(edge.first, edge.second, r, g, b, false, duration);
			lines++;
		}
		g_status.drawn_brushes++;
	}
	g_status.drawn_lines += lines;   // on top of the trigger pass's wires
}

BspWorld::Status BspWorld::GetStatus() {
	return g_status;
}

bool BspWorld::CurrentMapName(char* out, int cap) {
	if (!out || cap <= 0)
		return false;
	out[0] = 0;
	if (!engine)
		return false;
	std::string level;
	if (!SafeLevelName(level))
		return false;
	// SafeLevelName validated the "maps/<name>.bsp" shape.
	const std::string name = level.substr(5, level.size() - 9);
	strncpy_s(out, static_cast<rsize_t>(cap), name.c_str(), _TRUNCATE);
	return true;
}

BspWorld::RayHit BspWorld::Pick(const Vector& eye, const Vector& dir, float max_dist) {
	RayHit best;
	best.dist = max_dist;

	for (int bi = 0; bi < static_cast<int>(g_brushes.size()); ++bi) {
		const Brush& brush = g_brushes[bi];

		const bool clip = (brush.contents & BSP_CONTENTS_PLAYERCLIP) != 0;
		const bool ladder = (brush.contents & BSP_CONTENTS_LADDER) != 0;
		const bool solid = (brush.contents & (BSP_CONTENTS_SOLID | BSP_CONTENTS_GRATE)) != 0;
		if (!((solid && show_solid) || (clip && show_playerclip) || (ladder && show_ladder)))
			continue;

		// Clip the ray's parametric interval against the brush's COMPLETE plane
		// set (clip_planes) - using only winding-surviving faces would let the
		// ray pass straight through a brush whose front sliver was dropped.
		float t0 = 0.f;
		float t1 = best.dist;
		int enter_plane = -1;
		bool miss = false;
		for (const int plane_id : brush.clip_planes) {
			const Vector& n = g_planes[plane_id].first;
			const float d = g_planes[plane_id].second;
			const float dn = Dot(n, dir);
			const float ds = Dot(n, eye) - d;
			if (fabsf(dn) < 1e-6f) {
				if (ds > 0.f) { miss = true; break; }
				continue;
			}
			const float t = -ds / dn;
			if (dn < 0.f) {
				if (t > t0) { t0 = t; enter_plane = plane_id; }
			} else {
				if (t < t1) t1 = t;
			}
			if (t0 > t1) { miss = true; break; }
		}
		if (miss || enter_plane < 0 || t0 <= 0.f || t0 >= best.dist)
			continue;

		best.hit = true;
		best.dist = t0;
		best.brush = bi;
		best.plane = enter_plane;
		best.contents = brush.contents;
		best.point = eye + Scale(dir, t0);
		best.normal = g_planes[enter_plane].first;
	}
	return best;
}

void BspWorld::NotePickDebug(const Vector& eye, const Vector& dir, const RayHit& hit) {
	g_dbg_pick = true;
	g_dbg_until = GetTickCount64() + 2500;
	g_dbg_eye = eye;
	g_dbg_end = hit.hit ? hit.point : (eye + Scale(dir, 512.f));
	g_dbg_hit = hit.hit;
	g_dbg_normal = hit.normal;
}

bool BspWorld::ToggleFaceTag(int brush, int plane) {
	const std::pair<int, int> key = { brush, plane };
	bool tagged;
	if (g_tags.count(key)) {
		g_tags.erase(key);
		tagged = false;
	} else {
		g_tags.insert(key);
		tagged = true;
	}
	SaveGeoImpl();
	return tagged;
}

bool BspWorld::IsTagged(int brush, int plane) {
	return g_tags.count({ brush, plane }) != 0;
}

int BspWorld::TagCount() {
	return static_cast<int>(g_tags.size());
}

void BspWorld::ClearTags() {
	PushGeoUndo();
	g_tags.clear();
	SaveGeoImpl();
}

void BspWorld::ClearTargets() {
	PushGeoUndo();
	g_targets.clear();
	SaveGeoImpl();
}

bool BspWorld::CanUndoGeo() {
	return !g_geo_undo.empty();
}

bool BspWorld::UndoGeo() {
	if (g_geo_undo.empty())
		return false;
	g_tags = g_geo_undo.back().tags;
	g_targets = g_geo_undo.back().targets;
	g_geo_undo.pop_back();
	SaveGeoImpl();
	return true;
}

void BspWorld::ExportGeo(std::vector<std::pair<int, int>>* tags,
                         std::vector<BoardTarget>* targets) {
	if (tags)
		tags->assign(g_tags.begin(), g_tags.end());
	if (targets)
		*targets = g_targets;
}

int BspWorld::ImportGeo(const std::vector<std::pair<int, int>>& tags,
                        const std::vector<BoardTarget>& targets) {
	// Project geo REPLACES the map's working set (undoable like any other
	// destructive edit). Entries whose brush/plane indices fall outside the
	// loaded map are dropped and counted honestly - a project from a
	// different map version can never index out of bounds.
	PushGeoUndo();
	g_tags.clear();
	g_targets.clear();
	int dropped = 0;
	const int nb = static_cast<int>(g_brushes.size());
	const int np = static_cast<int>(g_planes.size());
	for (const auto& t : tags) {
		if (t.first >= 0 && t.first < nb && t.second >= 0 && t.second < np)
			g_tags.insert(t);
		else
			dropped++;
	}
	for (const BoardTarget& t : targets) {
		if (t.brush >= 0 && t.brush < nb && t.plane >= 0 && t.plane < np)
			g_targets.push_back(t);
		else
			dropped++;
	}
	SaveGeoImpl();
	return dropped;
}

int BspWorld::AddTargetFromHit(const RayHit& hit, float yaw, float pitch, bool ducked) {
	if (!hit.hit)
		return -1;
	BoardTarget target;
	if (!RestOnPlane(hit.plane, ducked, hit.point, target.pos))
		return -1;
	target.yaw = yaw;
	target.pitch = pitch;
	target.ducked = ducked;
	target.brush = hit.brush;
	target.plane = hit.plane;
	g_targets.push_back(target);
	SaveGeoImpl();
	return static_cast<int>(g_targets.size()) - 1;
}

int BspWorld::TargetCount() {
	return static_cast<int>(g_targets.size());
}

BspWorld::BoardTarget* BspWorld::GetTarget(int index) {
	if (index < 0 || index >= static_cast<int>(g_targets.size()))
		return nullptr;
	return &g_targets[index];
}

void BspWorld::RemoveTarget(int index) {
	if (index < 0 || index >= static_cast<int>(g_targets.size()))
		return;
	PushGeoUndo();
	g_targets.erase(g_targets.begin() + index);
	SaveGeoImpl();
}

void BspWorld::RestTargetOnFace(int index) {
	if (index < 0 || index >= static_cast<int>(g_targets.size()))
		return;
	BoardTarget& target = g_targets[index];
	Vector n;
	float d = 0.f;
	if (!GetPlane(target.plane, &n, &d))
		return;
	// Re-seat around the CURRENT contact centroid so the touching point stays
	// put when the hull height changes.
	const Vector anchor = target.pos + ContactOffset(n, target.ducked ? 54.f : 72.f);
	RestOnPlane(target.plane, target.ducked, anchor, target.pos);
}

void BspWorld::SaveGeo() {
	SaveGeoImpl();
}

bool BspWorld::GetTag(int index, int* brush, int* plane) {
	if (index < 0 || index >= static_cast<int>(g_tags.size()))
		return false;
	auto it = g_tags.begin();
	std::advance(it, index);
	if (brush) *brush = it->first;
	if (plane) *plane = it->second;
	return true;
}

void BspWorld::RemoveTag(int index) {
	if (index < 0 || index >= static_cast<int>(g_tags.size()))
		return;
	PushGeoUndo();
	auto it = g_tags.begin();
	std::advance(it, index);
	g_tags.erase(it);
	SaveGeoImpl();
}

bool BspWorld::GetPlane(int plane, Vector* normal, float* dist) {
	if (plane < 0 || plane >= static_cast<int>(g_planes.size()))
		return false;
	if (normal) *normal = g_planes[plane].first;
	if (dist) *dist = g_planes[plane].second;
	return true;
}

int BspWorld::BrushClipPlaneCount(int brush) {
	if (brush < 0 || brush >= static_cast<int>(g_brushes.size()))
		return 0;
	return static_cast<int>(g_brushes[brush].clip_planes.size());
}

bool BspWorld::GetBrushClipPlane(int brush, int index, int* plane_id, Vector* n, float* d) {
	if (brush < 0 || brush >= static_cast<int>(g_brushes.size()))
		return false;
	const Brush& b = g_brushes[brush];
	if (index < 0 || index >= static_cast<int>(b.clip_planes.size()))
		return false;
	const int pid = b.clip_planes[index];
	if (pid < 0 || pid >= static_cast<int>(g_planes.size()))
		return false;
	if (plane_id) *plane_id = pid;
	if (n) *n = g_planes[pid].first;
	if (d) *d = g_planes[pid].second;
	return true;
}

bool BspWorld::IsWorldBrush(int brush) {
	if (brush < 0 || brush >= static_cast<int>(g_brushes.size()))
		return false;
	return g_brushes[brush].world;
}

int BspWorld::BrushBevelPlaneCount(int brush) {
	if (brush < 0 || brush >= static_cast<int>(g_brushes.size()))
		return 0;
	return static_cast<int>(g_brushes[brush].bevel_planes.size());
}

bool BspWorld::GetBrushBevelPlane(int brush, int index, Vector* n, float* d) {
	if (brush < 0 || brush >= static_cast<int>(g_brushes.size()))
		return false;
	const Brush& b = g_brushes[brush];
	if (index < 0 || index >= static_cast<int>(b.bevel_planes.size()))
		return false;
	const int pid = b.bevel_planes[index];
	if (pid < 0 || pid >= static_cast<int>(g_planes.size()))
		return false;
	if (n) *n = g_planes[pid].first;
	if (d) *d = g_planes[pid].second;
	return true;
}

int BspWorld::BrushCount() {
	return static_cast<int>(g_brushes.size());
}

bool BspWorld::GetBrushInfo(int brush, int* contents, Vector* mins, Vector* maxs) {
	if (brush < 0 || brush >= static_cast<int>(g_brushes.size()))
		return false;
	const Brush& b = g_brushes[brush];
	if (contents) *contents = b.contents;
	if (mins) *mins = b.mins;
	if (maxs) *maxs = b.maxs;
	return true;
}

bool BspWorld::LockedPitch(int plane, float yaw, float* out_pitch) {
	if (plane < 0 || plane >= static_cast<int>(g_planes.size()) || !out_pitch)
		return false;
	const Vector n = g_planes[plane].first;
	// View dir d = (cos p cos y, cos p sin y, -sin p); solve n.d = 0 for p:
	// tan p = (n_x cos y + n_y sin y) / n_z  (Source pitch: + = down).
	const float yr = yaw * (3.14159265f / 180.f);
	const float h = n.X * cosf(yr) + n.Y * sinf(yr);
	float p = atan2f(h, n.Z) * (180.f / 3.14159265f);
	if (p > 89.f) p = 89.f;
	if (p < -89.f) p = -89.f;
	*out_pitch = p;
	return true;
}

bool BspWorld::GetFacePolygon(int brush, int plane, const Vector** points, int* count) {
	if (brush < 0 || brush >= static_cast<int>(g_brushes.size()))
		return false;
	for (const Face& face : g_brushes[brush].faces) {
		if (face.plane != plane || face.pts.size() < 3)
			continue;
		if (points) *points = face.pts.data();
		if (count) *count = static_cast<int>(face.pts.size());
		return true;
	}
	return false;
}

namespace {
	// Shared point-vs-face computation: 2D point-in-polygon on the plane plus
	// the distance to the nearest polygon edge.
	bool FacePointQuery(int brush, int plane, const Vector& point,
	                    bool* out_inside, float* out_edge_dist) {
		const Vector* pts = nullptr;
		int count = 0;
		if (!BspWorld::GetFacePolygon(brush, plane, &pts, &count))
			return false;
		Vector n;
		float d = 0.f;
		if (!BspWorld::GetPlane(plane, &n, &d))
			return false;

		// Project onto the plane, then drop the normal's dominant axis for a
		// 2D point-in-polygon test.
		const Vector p = point - Scale(n, Dot(n, point) - d);
		int drop = 0;
		float best = fabsf(n.X);
		if (fabsf(n.Y) > best) { best = fabsf(n.Y); drop = 1; }
		if (fabsf(n.Z) > best) { drop = 2; }
		auto u = [&](const Vector& v) { return drop == 0 ? v.Y : v.X; };
		auto w = [&](const Vector& v) { return drop == 2 ? v.Y : v.Z; };

		bool inside = false;
		float min_edge_dist = 1e9f;
		const float pu = u(p), pw = w(p);
		for (int i = 0, j = count - 1; i < count; j = i++) {
			const float iu = u(pts[i]), iw = w(pts[i]);
			const float ju = u(pts[j]), jw = w(pts[j]);
			if ((iw > pw) != (jw > pw) &&
				pu < (ju - iu) * (pw - iw) / (jw - iw) + iu)
				inside = !inside;

			const float ex = ju - iu, ew = jw - iw;
			const float len2 = ex * ex + ew * ew;
			float t = len2 > 1e-9f ? ((pu - iu) * ex + (pw - iw) * ew) / len2 : 0.f;
			if (t < 0.f) t = 0.f;
			if (t > 1.f) t = 1.f;
			const float du = pu - (iu + ex * t), dw = pw - (iw + ew * t);
			const float dist = sqrtf(du * du + dw * dw);
			if (dist < min_edge_dist)
				min_edge_dist = dist;
		}
		*out_inside = inside;
		*out_edge_dist = min_edge_dist;
		return true;
	}
}

bool BspWorld::PointOnFace(int brush, int plane, const Vector& point, float expand) {
	bool inside = false;
	float edge_dist = 1e9f;
	if (!FacePointQuery(brush, plane, point, &inside, &edge_dist))
		return false;
	return inside || edge_dist <= expand;
}

bool BspWorld::PointOnFaceQuery(int brush, int plane, const Vector& point,
                                float margin, bool* interior) {
	bool inside = false;
	float edge_dist = 1e9f;
	if (interior)
		*interior = false;
	if (!FacePointQuery(brush, plane, point, &inside, &edge_dist))
		return false;
	if (interior)
		*interior = inside && edge_dist >= margin;
	return inside || edge_dist <= margin;
}
