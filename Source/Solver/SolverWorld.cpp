#include "SolverWorld.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>

namespace Solver {

	namespace {
		constexpr int kLumpEntities = 0;
		constexpr int kLumpPlanes = 1;
		constexpr int kLumpNodes = 5;
		constexpr int kLumpLeafs = 10;
		constexpr int kLumpModels = 14;
		constexpr int kLumpLeafBrushes = 17;
		constexpr int kLumpBrushes = 18;
		constexpr int kLumpBrushSides = 19;

		// What MASK_PLAYERSOLID hits: SOLID | WINDOW | GRATE | PLAYERCLIP.
		constexpr int kPlayerSolid = 0x1 | 0x2 | 0x8 | 0x10000;

		constexpr float kDistEpsilon = 0.03125f;   // engine DIST_EPSILON (1/32)

		struct Lump { int ofs = 0, len = 0, ver = 0; };

		// Minkowski support of the NEGATED player hull for plane normal n:
		// the hull (offsets b in [mins,maxs]) touches plane n.x <= d iff
		// n.origin <= d - min_b(n.b); min_b picks mins where n is positive.
		float HullExpand(const Vec3& n, const Vec3& mn, const Vec3& mx) {
			return -((n.X > 0.f ? n.X * mn.X : n.X * mx.X)
				+ (n.Y > 0.f ? n.Y * mn.Y : n.Y * mx.Y)
				+ (n.Z > 0.f ? n.Z * mn.Z : n.Z * mx.Z));
		}

		bool Fail(std::string* err, const char* msg) {
			if (err) *err = msg;
			return false;
		}
	}

	bool World::Load(const std::string& bsp_path, const Hulls& hulls, std::string* err,
	                 bool edge_bevels) {
		hulls_ = hulls;
		brushes.clear();

		std::ifstream in(bsp_path, std::ios::binary);
		if (!in)
			return Fail(err, "cannot open bsp file");
		std::vector<char> data((std::istreambuf_iterator<char>(in)),
		                       std::istreambuf_iterator<char>());
		if (data.size() < 8 + 64 * 16 + 4)
			return Fail(err, "file too small for a VBSP header");

		const char* p = data.data();
		const size_t size = data.size();
		int32_t ident = 0;
		std::memcpy(&ident, p, 4);
		std::memcpy(&version, p + 4, 4);
		if (ident != 0x50534256)
			return Fail(err, "not a VBSP file");

		Lump lumps[64];
		for (int i = 0; i < 64; ++i) {
			std::memcpy(&lumps[i].ofs, p + 8 + i * 16 + 0, 4);
			std::memcpy(&lumps[i].len, p + 8 + i * 16 + 4, 4);
			std::memcpy(&lumps[i].ver, p + 8 + i * 16 + 8, 4);
			if (lumps[i].ofs < 0 || lumps[i].len < 0
				|| static_cast<size_t>(lumps[i].ofs) + static_cast<size_t>(lumps[i].len) > size)
				return Fail(err, "lump table exceeds file bounds");
		}
		std::memcpy(&mapRevision, p + 8 + 64 * 16, 4);

		// ---- entities (text) + spawn -----------------------------------------
		entities_text.assign(p + lumps[kLumpEntities].ofs,
		                     static_cast<size_t>(lumps[kLumpEntities].len));
		if (!entities_text.empty() && entities_text.back() == '\0')
			entities_text.pop_back();
		{
			// Block scan: first info_player_terrorist, else counterterrorist,
			// else info_player_start.
			const char* want[3] = { "info_player_terrorist",
			                        "info_player_counterterrorist",
			                        "info_player_start" };
			for (int w = 0; w < 3 && !have_spawn; ++w) {
				size_t pos = 0;
				while (!have_spawn) {
					const size_t open = entities_text.find('{', pos);
					if (open == std::string::npos) break;
					const size_t close = entities_text.find('}', open);
					if (close == std::string::npos) break;
					const std::string block = entities_text.substr(open, close - open);
					pos = close + 1;
					if (block.find(std::string("\"") + want[w] + "\"") == std::string::npos)
						continue;
					const size_t o = block.find("\"origin\"");
					if (o == std::string::npos) continue;
					const size_t q1 = block.find('"', o + 8);
					if (q1 == std::string::npos) continue;
					float x, y, z;
					if (sscanf_s(block.c_str() + q1 + 1, "%f %f %f", &x, &y, &z) != 3)
						continue;
					spawn_origin = Vec3(x, y, z);
					const size_t a = block.find("\"angles\"");
					if (a != std::string::npos) {
						const size_t q2 = block.find('"', a + 8);
						float ap, ay, ar;
						if (q2 != std::string::npos
							&& sscanf_s(block.c_str() + q2 + 1, "%f %f %f", &ap, &ay, &ar) == 3) {
							spawn_pitch = ap;
							spawn_yaw = ay;
						}
					}
					have_spawn = true;
				}
			}
		}

		// ---- planes ----------------------------------------------------------
		nplanes = lumps[kLumpPlanes].len / 20;
		std::vector<Vec3> plane_n(nplanes);
		std::vector<float> plane_d(nplanes);
		for (int i = 0; i < nplanes; ++i) {
			const char* pp = p + lumps[kLumpPlanes].ofs + i * 20;
			std::memcpy(&plane_n[i], pp, 12);
			std::memcpy(&plane_d[i], pp + 12, 4);
		}

		// ---- model 0 world-brush walk ----------------------------------------
		// Strides are DOCUMENTED on-disk sizes, never sizeof() of a packed
		// struct (the 30-vs-32-byte leaf bug silently corrupted a whole walk).
		nmodels = lumps[kLumpModels].len / 48;
		nbrushes_total = lumps[kLumpBrushes].len / 12;
		// Per-brush MODEL tag (was a world-only 0/1): model 0 = worldspawn
		// collision; higher models are brush entities - which is exactly
		// what trigger volumes reference ("*N" names model N's brush set).
		std::vector<int32_t> brush_model(nbrushes_total, -1);
		if (nmodels < 1)
			return Fail(err, "no models in bsp");
		{
			int leaf_stride = (lumps[kLumpLeafs].ver == 0) ? 56 : 32;
			if (lumps[kLumpLeafs].len % leaf_stride != 0) {
				const int other = (leaf_stride == 56) ? 32 : 56;
				if (lumps[kLumpLeafs].len % other == 0)
					leaf_stride = other;
				else
					return Fail(err, "leaf lump length matches no known stride");
			}
			const int nleafs = lumps[kLumpLeafs].len / leaf_stride;
			const int nnodes = lumps[kLumpNodes].len / 32;
			const int nleafbrushes = lumps[kLumpLeafBrushes].len / 2;

			std::vector<int32_t> stack;
			for (int m = 0; m < nmodels; ++m) {
				int32_t headnode = 0;
				std::memcpy(&headnode, p + lumps[kLumpModels].ofs + m * 48 + 36, 4);
				stack.clear();
				stack.push_back(headnode);
				int guard = 0;
				while (!stack.empty() && guard++ < 4000000) {
					const int32_t node = stack.back();
					stack.pop_back();
					if (node < 0) {
						const int leaf = -1 - node;
						if (leaf < 0 || leaf >= nleafs)
							continue;
						const char* lp = p + lumps[kLumpLeafs].ofs + leaf * leaf_stride;
						uint16_t first = 0, count = 0;
						// dleaf_t: firstleafBRUSH@24 / numleafBRUSHES@26. This
						// pass read the FACE fields at +20/+22 for its whole
						// life - leaf faces misread as brush ids. Visible world
						// brushes mostly got marked by index coincidence, but
						// brushes whose ids never appear among face indices
						// stayed unmarked and were DROPPED as "entity" brushes.
						// Clip brushes have no faces at all - prime victims.
						// Found when the box oracle reported startsolid at duck
						// sites our world called free (1,089 of 1,296
						// zero-length residual queries, 2026-08-15).
						std::memcpy(&first, lp + 24, 2);
						std::memcpy(&count, lp + 26, 2);
						for (int i = 0; i < count; ++i) {
							const int lbi = first + i;
							if (lbi < 0 || lbi >= nleafbrushes)
								continue;
							uint16_t bid = 0;
							std::memcpy(&bid, p + lumps[kLumpLeafBrushes].ofs + lbi * 2, 2);
							if (bid < nbrushes_total && brush_model[bid] < 0)
								brush_model[bid] = m;
						}
						continue;
					}
					if (node >= nnodes)
						continue;
					const char* np = p + lumps[kLumpNodes].ofs + node * 32;
					int32_t c0 = 0, c1 = 0;
					std::memcpy(&c0, np + 4, 4);
					std::memcpy(&c1, np + 8, 4);
					stack.push_back(c0);
					stack.push_back(c1);
				}
			}
		}
		std::vector<uint8_t> is_world(nbrushes_total, 0);
		for (int b = 0; b < nbrushes_total; ++b)
			if (brush_model[b] == 0)
				is_world[b] = 1;

		// ---- BSP TREE for solid-leaf queries (engine's own authority) ----
		{
			int leaf_stride = (lumps[kLumpLeafs].ver == 0) ? 56 : 32;
			if (lumps[kLumpLeafs].len % leaf_stride != 0) {
				const int other = (leaf_stride == 56) ? 32 : 56;
				if (lumps[kLumpLeafs].len % other == 0)
					leaf_stride = other;
			}
			const int nleafs = lumps[kLumpLeafs].len / leaf_stride;
			const int nnodes = lumps[kLumpNodes].len / 32;
			nodes.resize(nnodes);
			for (int i = 0; i < nnodes; ++i) {
				const char* np = p + lumps[kLumpNodes].ofs + i * 32;
				int32_t pl = 0, c0 = 0, c1 = 0;
				std::memcpy(&pl, np, 4);
				std::memcpy(&c0, np + 4, 4);
				std::memcpy(&c1, np + 8, 4);
				nodes[i].plane = pl;
				nodes[i].child[0] = c0;
				nodes[i].child[1] = c1;
			}
			leaf_contents.resize(nleafs);
			for (int i = 0; i < nleafs; ++i) {
				int32_t c = 0;
				std::memcpy(&c, p + lumps[kLumpLeafs].ofs + i * leaf_stride, 4);
				leaf_contents[i] = c;
			}
			tree_n = plane_n;
			tree_d = plane_d;
			std::memcpy(&headnode, p + lumps[kLumpModels].ofs + 36, 4);
			// Engine brush order: per-leaf brush lists, lump order (raw BSP
			// ids here; remapped to our filtered indices after the brush
			// build below).
			leaf_firstbrush.resize(nleafs);
			leaf_numbrushes.resize(nleafs);
			for (int i = 0; i < nleafs; ++i) {
				uint16_t fb = 0, nb = 0;
				const char* lp = p + lumps[kLumpLeafs].ofs + i * leaf_stride;
				// dleaf_t: contents@0, cluster@4, area/flags@6, mins@8,
				// maxs@14, firstleafFACE@20, numleafFACES@22,
				// firstleafBRUSH@24, numleafBRUSHES@26. The face fields at
				// +20/+22 parse as plausible indices and break everything
				// downstream - measured: battery 0/15 and 1,857 missing
				// startsolid flags before this offset was corrected.
				std::memcpy(&fb, lp + 24, 2);
				std::memcpy(&nb, lp + 26, 2);
				leaf_firstbrush[i] = fb;
				leaf_numbrushes[i] = nb;
			}
			const int nleafbrushes = lumps[kLumpLeafBrushes].len / 2;
			leafbrush_ours.resize(nleafbrushes);
			for (int i = 0; i < nleafbrushes; ++i) {
				uint16_t bid = 0;
				std::memcpy(&bid, p + lumps[kLumpLeafBrushes].ofs + i * 2, 2);
				leafbrush_ours[i] = bid;
			}
		}

		// ---- brushes ---------------------------------------------------------
		const int nsides_total = lumps[kLumpBrushSides].len / 8;
		nbrushes_entity = 0;
		for (int b = 0; b < nbrushes_total; ++b) {
			int32_t firstside = 0, numsides = 0, contents = 0;
			const char* bp = p + lumps[kLumpBrushes].ofs + b * 12;
			std::memcpy(&firstside, bp, 4);
			std::memcpy(&numsides, bp + 4, 4);
			std::memcpy(&contents, bp + 8, 4);
			if (!(contents & kPlayerSolid))
				continue;
			if (!is_world[b]) {
				nbrushes_entity++;
				continue;
			}

			WorldBrush wb;
			wb.id = b;
			wb.contents = contents;
			bool have_min[3] = { false, false, false };
			bool have_max[3] = { false, false, false };
			float mn[3] = { 0, 0, 0 }, mx[3] = { 0, 0, 0 };
			auto note_axial = [&](const Vec3& n, float d) {
				for (int a = 0; a < 3; ++a) {
					const float c = (a == 0) ? n.X : (a == 1) ? n.Y : n.Z;
					if (c > 0.999f) {
						if (!have_max[a] || d > mx[a]) { mx[a] = d; have_max[a] = true; }
					} else if (c < -0.999f) {
						if (!have_min[a] || -d < mn[a]) { mn[a] = -d; have_min[a] = true; }
					}
				}
			};

			for (int si = 0; si < numsides; ++si) {
				const int sidx = firstside + si;
				if (sidx < 0 || sidx >= nsides_total)
					continue;
				const char* sp = p + lumps[kLumpBrushSides].ofs + sidx * 8;
				uint16_t planenum = 0;
				int16_t texinfo = 0, dispinfo = 0, bevel = 0;
				std::memcpy(&planenum, sp + 0, 2);
				std::memcpy(&texinfo, sp + 2, 2);
				std::memcpy(&dispinfo, sp + 4, 2);
				std::memcpy(&bevel, sp + 6, 2);
				if (planenum >= nplanes)
					continue;
				const Vec3 n = plane_n[planenum];
				const float d = plane_d[planenum];
				const bool axial = (fabsf(n.X) > 0.999f || fabsf(n.Y) > 0.999f
					|| fabsf(n.Z) > 0.999f);
				if (bevel == 0) {
					// Real side: always part of the clip set (winding-surviving
					// or not - dropped sliver faces must still clip).
					wb.n.push_back(n);
					wb.d.push_back(d);
					wb.pid.push_back(planenum);
					if (axial) note_axial(n, d);
				} else if (axial) {
					// The compiler's EXACT axial extents. Deferred: appended
					// after the sides so [0, nsides) stays "real sides".
					// (Non-axial edge bevels deliberately NOT loaded.)
					note_axial(n, d);
				}
			}
			wb.nsides = static_cast<int>(wb.n.size());
			// Second pass appends the axial bevels behind the real sides.
			for (int si = 0; si < numsides; ++si) {
				const int sidx = firstside + si;
				if (sidx < 0 || sidx >= nsides_total)
					continue;
				const char* sp = p + lumps[kLumpBrushSides].ofs + sidx * 8;
				uint16_t planenum = 0;
				int16_t bevel = 0;
				std::memcpy(&planenum, sp + 0, 2);
				std::memcpy(&bevel, sp + 6, 2);
				if (bevel == 0 || planenum >= nplanes)
					continue;
				const Vec3 n = plane_n[planenum];
				const bool axial = (fabsf(n.X) > 0.999f || fabsf(n.Y) > 0.999f
					|| fabsf(n.Z) > 0.999f);
				if (axial) {
					wb.n.push_back(n);
					wb.d.push_back(plane_d[planenum]);
					wb.pid.push_back(-2);
					continue;
				}
				// NON-AXIAL edge bevels: the corner separating planes VBSP
				// adds so AABB hulls release at brush edges instead of
				// snagging. Proven necessary by the 604-tape capture (core
				// clipped a ramp face one tick past the engine when sliding
				// off the brush end). Dedup against the real sides - a bevel
				// side can share a real side's plane and add nothing.
				if (!edge_bevels)
					continue;
				bool dup = false;
				for (int rp : wb.pid)
					if (rp == static_cast<int>(planenum))
						{ dup = true; break; }
				if (dup)
					continue;
				wb.n.push_back(n);
				wb.d.push_back(plane_d[planenum]);
				wb.pid.push_back(-3);
			}
			if (wb.n.empty())
				continue;
			if (!(have_min[0] && have_max[0] && have_min[1] && have_max[1]
				&& have_min[2] && have_max[2])) {
				// A compiled brush always has full axial coverage via sides +
				// bevels; a miss means the parse is wrong for this map.
				fprintf(stderr, "WARN: brush %d lacks axial coverage; using +/-1e9 gate\n", b);
				wb.bmin = Vec3(-1e9f, -1e9f, -1e9f);
				wb.bmax = Vec3(1e9f, 1e9f, 1e9f);
			} else {
				wb.bmin = Vec3(mn[0], mn[1], mn[2]);
				wb.bmax = Vec3(mx[0], mx[1], mx[2]);
			}

			// Hull expansions (all three hulls) + origin-space AABB gates.
			wb.d_stand.resize(wb.n.size());
			wb.d_duck.resize(wb.n.size());
			wb.d_unduck.resize(wb.n.size());
			for (size_t i = 0; i < wb.n.size(); ++i) {
				wb.d_stand[i] = wb.d[i] + HullExpand(wb.n[i], hulls_.stand_min, hulls_.stand_max);
				wb.d_duck[i] = wb.d[i] + HullExpand(wb.n[i], hulls_.duck_min, hulls_.duck_max);
				wb.d_unduck[i] = wb.d[i] + HullExpand(wb.n[i], hulls_.unduck_min, hulls_.unduck_max);
			}
			wb.gmin_stand = wb.bmin - hulls_.stand_max;
			wb.gmax_stand = wb.bmax - hulls_.stand_min;
			wb.gmin_duck = wb.bmin - hulls_.duck_max;
			wb.gmax_duck = wb.bmax - hulls_.duck_min;
			wb.gmin_unduck = wb.bmin - hulls_.unduck_max;
			wb.gmax_unduck = wb.bmax - hulls_.unduck_min;

			brushes.push_back(std::move(wb));
		}
		// Remap the leafbrush lists from BSP brush ids to OUR filtered brush
		// indices (-1 for brushes the build dropped: non-solid / entity).
		{
			std::vector<int> bsp2ours(nbrushes_total, -1);
			for (size_t i = 0; i < brushes.size(); ++i)
				bsp2ours[brushes[i].id] = static_cast<int>(i);
			for (size_t i = 0; i < leafbrush_ours.size(); ++i) {
				const int raw = leafbrush_ours[i];
				leafbrush_ours[i] =
					(raw >= 0 && raw < nbrushes_total) ? bsp2ours[raw] : -1;
			}
		}

		// ---- TRIGGER VOLUMES (mirrors BspWorld::ParseEntities +
		// CheckTriggers 1:1 - the DLL implementation the RequestSim path
		// applies; user directive 2026-08-15: total parity incl. triggers).
		triggers.clear();
		trig_brushes.clear();
		{
			struct TEnt {
				std::string classname, targetname, target, landmark, model;
				std::string origin, angles, pushdir;
				int spawnflags = 0;
				float gravity = 1.f, speed = 0.f;
			};
			struct TNamed { std::string name; Vec3 origin; float pitch, yaw; };
			auto parse_v3 = [](const std::string& s) {
				Vec3 v;
				sscanf_s(s.c_str(), "%f %f %f", &v.X, &v.Y, &v.Z);
				return v;
			};
			std::vector<TEnt> trigs;
			std::vector<TNamed> named;
			size_t pos = 0;
			while (true) {
				const size_t open = entities_text.find('{', pos);
				if (open == std::string::npos)
					break;
				const size_t close = entities_text.find('}', open);
				if (close == std::string::npos)
					break;
				const std::string block =
					entities_text.substr(open, close - open);
				pos = close + 1;
				TEnt e;
				size_t q = 0;
				while (true) {
					const size_t k0 = block.find('"', q);
					if (k0 == std::string::npos) break;
					const size_t k1 = block.find('"', k0 + 1);
					if (k1 == std::string::npos) break;
					const size_t v0 = block.find('"', k1 + 1);
					if (v0 == std::string::npos) break;
					const size_t v1 = block.find('"', v0 + 1);
					if (v1 == std::string::npos) break;
					const std::string key = block.substr(k0 + 1, k1 - k0 - 1);
					const std::string val = block.substr(v0 + 1, v1 - v0 - 1);
					q = v1 + 1;
					if (key == "classname") e.classname = val;
					else if (key == "targetname") e.targetname = val;
					else if (key == "target") e.target = val;
					else if (key == "landmark") e.landmark = val;
					else if (key == "model") e.model = val;
					else if (key == "origin") e.origin = val;
					else if (key == "angles") e.angles = val;
					else if (key == "spawnflags") e.spawnflags = atoi(val.c_str());
					else if (key == "gravity")
						e.gravity = static_cast<float>(atof(val.c_str()));
					else if (key == "pushdir") e.pushdir = val;
					else if (key == "speed")
						e.speed = static_cast<float>(atof(val.c_str()));
				}
				if (!e.targetname.empty() && !e.origin.empty()) {
					TNamed n;
					n.name = e.targetname;
					n.origin = parse_v3(e.origin);
					const Vec3 a = parse_v3(e.angles);   // "pitch yaw roll"
					n.pitch = a.X;
					n.yaw = a.Y;
					named.push_back(std::move(n));
				}
				if (e.classname == "trigger_teleport"
					|| e.classname == "trigger_gravity"
					|| e.classname == "trigger_push")
					trigs.push_back(std::move(e));
			}
			for (const TEnt& e : trigs) {
				if (e.model.size() < 2 || e.model[0] != '*')
					continue;   // no brush model: nothing to touch
				TriggerVol tv;
				tv.model = atoi(e.model.c_str() + 1);
				tv.spawnflags = e.spawnflags;
				tv.ent_origin = e.origin.empty() ? Vec3() : parse_v3(e.origin);
				tv.teleport = (e.classname == "trigger_teleport");
				tv.push = (e.classname == "trigger_push");
				if (tv.teleport) {
					for (const TNamed& n : named)
						if (n.name == e.target) {
							tv.dest_ok = true;
							tv.dest_origin = n.origin;
							tv.dest_pitch = n.pitch;
							tv.dest_yaw = n.yaw;
							break;
						}
					if (!e.landmark.empty())
						for (const TNamed& n : named)
							if (n.name == e.landmark) {
								tv.landmark_ok = true;
								tv.landmark_origin = n.origin;
								break;
							}
					if (!tv.dest_ok)
						continue;   // unresolvable destination: skip
				} else if (tv.push) {
					// SDK: "pushdir" is angles; world dir = forward vector.
					const Vec3 a = parse_v3(e.pushdir);
					const float pr = a.X * 3.14159265f / 180.f;
					const float yr = a.Y * 3.14159265f / 180.f;
					tv.push_dir = Vec3(cosf(pr) * cosf(yr),
						cosf(pr) * sinf(yr), -sinf(pr));
					tv.push_speed = e.speed;
				} else {
					tv.gravity = e.gravity;
				}
				// The model's brushes as RAW plane sets, ALL sides + ALL
				// bevels (with bevels the support-corner containment IS the
				// exact Minkowski hull-vs-brush test, per the DLL).
				for (int b = 0; b < nbrushes_total; ++b) {
					if (brush_model[b] != tv.model)
						continue;
					int32_t firstside = 0, numsides = 0;
					const char* bp = p + lumps[kLumpBrushes].ofs + b * 12;
					std::memcpy(&firstside, bp, 4);
					std::memcpy(&numsides, bp + 4, 4);
					TrigBrush tb;
					for (int si = 0; si < numsides; ++si) {
						const int sidx = firstside + si;
						if (sidx < 0 || sidx >= nsides_total)
							continue;
						const char* sp = p + lumps[kLumpBrushSides].ofs
							+ sidx * 8;
						uint16_t planenum = 0;
						std::memcpy(&planenum, sp + 0, 2);
						if (planenum >= nplanes)
							continue;
						tb.n.push_back(plane_n[planenum]);
						tb.d.push_back(plane_d[planenum]);
					}
					if (tb.n.empty())
						continue;
					tv.tb.push_back(static_cast<int>(trig_brushes.size()));
					trig_brushes.push_back(std::move(tb));
				}
				if (!tv.tb.empty())
					triggers.push_back(std::move(tv));
			}
		}

		BuildGrid();
		return true;
	}

	bool World::CheckTriggers(const Vec3& origin, int hull_state,
	                          TriggerHitS* out) const {
		if (triggers.empty() || !out)
			return false;
		// SDK SF_TRIGGER_ALLOW_CLIENTS.
		constexpr int kSfAllowClients = 0x01;
		// SDK SF_TELEPORT_PRESERVE_ANGLES.
		constexpr int kSfPreserveAngles = 0x20;
		const Vec3 hmin = hull_state == 1 ? hulls_.duck_min
			: hull_state == 2 ? hulls_.unduck_min : hulls_.stand_min;
		const Vec3 hmax = hull_state == 1 ? hulls_.duck_max
			: hull_state == 2 ? hulls_.unduck_max : hulls_.stand_max;
		bool any = false;
		for (const TriggerVol& tv : triggers) {
			if (!(tv.spawnflags & kSfAllowClients))
				continue;
			// Entity "origin" offsets the brush model: test in local space.
			const Vec3 lp(origin.X - tv.ent_origin.X,
				origin.Y - tv.ent_origin.Y, origin.Z - tv.ent_origin.Z);
			bool touching = false;
			for (const int bi : tv.tb) {
				const TrigBrush& b = trig_brushes[bi];
				bool inside = true;
				for (size_t i = 0; i < b.n.size(); ++i) {
					const Vec3& n = b.n[i];
					// Support corner minimizing n over the hull.
					const float cx = lp.X + (n.X > 0.f ? hmin.X : hmax.X);
					const float cy = lp.Y + (n.Y > 0.f ? hmin.Y : hmax.Y);
					const float cz = lp.Z + (n.Z > 0.f ? hmin.Z : hmax.Z);
					if (n.X * cx + n.Y * cy + n.Z * cz - b.d[i] > 0.f) {
						inside = false;
						break;
					}
				}
				if (inside) { touching = true; break; }
			}
			if (!touching)
				continue;
			any = true;
			if (tv.teleport && !out->teleported) {
				// SDK CTriggerTeleport::Touch: landmark keeps the player's
				// offset and never sets angles; else destination origin +
				// angles unless preserve flag. Velocity ALWAYS zeroed.
				Vec3 dst = tv.dest_origin;
				if (tv.landmark_ok) {
					dst.X += origin.X - tv.landmark_origin.X;
					dst.Y += origin.Y - tv.landmark_origin.Y;
					dst.Z += origin.Z - tv.landmark_origin.Z;
					out->tp_set_angles = false;
				} else {
					out->tp_set_angles = !(tv.spawnflags & kSfPreserveAngles);
					out->tp_pitch = tv.dest_pitch;
					out->tp_yaw = tv.dest_yaw;
				}
				out->teleported = true;
				out->tp_origin = dst;
			} else if (tv.push) {
				out->pushed = true;
				out->push_vec = Vec3(tv.push_dir.X * tv.push_speed,
					tv.push_dir.Y * tv.push_speed,
					tv.push_dir.Z * tv.push_speed);
			} else if (!tv.teleport) {
				out->grav_touched = true;
				out->gravity = tv.gravity;
			}
		}
		return any;
	}

	void World::BuildGrid() {
		cells_.clear();
		nx_ = ny_ = nz_ = 0;
		if (brushes.empty())
			return;
		world_min = brushes[0].bmin;
		world_max = brushes[0].bmax;
		for (const WorldBrush& b : brushes) {
			world_min.X = fminf(world_min.X, b.bmin.X);
			world_min.Y = fminf(world_min.Y, b.bmin.Y);
			world_min.Z = fminf(world_min.Z, b.bmin.Z);
			world_max.X = fmaxf(world_max.X, b.bmax.X);
			world_max.Y = fmaxf(world_max.Y, b.bmax.Y);
			world_max.Z = fmaxf(world_max.Z, b.bmax.Z);
		}
		gmin_ = brushes[0].gmin_stand;
		gmax_ = brushes[0].gmax_stand;
		for (const WorldBrush& b : brushes) {
			gmin_.X = fminf(gmin_.X, b.gmin_stand.X);
			gmin_.Y = fminf(gmin_.Y, b.gmin_stand.Y);
			gmin_.Z = fminf(gmin_.Z, b.gmin_stand.Z);
			gmax_.X = fmaxf(gmax_.X, b.gmax_stand.X);
			gmax_.Y = fmaxf(gmax_.Y, b.gmax_stand.Y);
			gmax_.Z = fmaxf(gmax_.Z, b.gmax_stand.Z);
		}
		nx_ = static_cast<int>((gmax_.X - gmin_.X) / cell_) + 1;
		ny_ = static_cast<int>((gmax_.Y - gmin_.Y) / cell_) + 1;
		nz_ = static_cast<int>((gmax_.Z - gmin_.Z) / cell_) + 1;
		// Safety: enormous maps degrade to a coarser grid, never to OOM.
		while (static_cast<long long>(nx_) * ny_ * nz_ > 4'000'000) {
			cell_ *= 2.f;
			nx_ = static_cast<int>((gmax_.X - gmin_.X) / cell_) + 1;
			ny_ = static_cast<int>((gmax_.Y - gmin_.Y) / cell_) + 1;
			nz_ = static_cast<int>((gmax_.Z - gmin_.Z) / cell_) + 1;
		}
		cells_.resize(static_cast<size_t>(nx_) * ny_ * nz_);
		for (int i = 0; i < static_cast<int>(brushes.size()); ++i) {
			int x0, x1, y0, y1, z0, z1;
			CellRange(brushes[i].gmin_stand, brushes[i].gmax_stand,
			          &x0, &x1, &y0, &y1, &z0, &z1);
			for (int z = z0; z <= z1; ++z)
				for (int y = y0; y <= y1; ++y)
					for (int x = x0; x <= x1; ++x)
						cells_[(static_cast<size_t>(z) * ny_ + y) * nx_ + x].push_back(i);
		}
	}

	void World::CellRange(const Vec3& lo, const Vec3& hi,
	                      int* x0, int* x1, int* y0, int* y1, int* z0, int* z1) const {
		auto clampi = [](int v, int n) { return v < 0 ? 0 : (v >= n ? n - 1 : v); };
		*x0 = clampi(static_cast<int>((lo.X - gmin_.X) / cell_), nx_);
		*x1 = clampi(static_cast<int>((hi.X - gmin_.X) / cell_), nx_);
		*y0 = clampi(static_cast<int>((lo.Y - gmin_.Y) / cell_), ny_);
		*y1 = clampi(static_cast<int>((hi.Y - gmin_.Y) / cell_), ny_);
		*z0 = clampi(static_cast<int>((lo.Z - gmin_.Z) / cell_), nz_);
		*z1 = clampi(static_cast<int>((hi.Z - gmin_.Z) / cell_), nz_);
	}

	float World::TraceHull(const Vec3& a, const Vec3& b, bool ducked,
	                       TraceResult* out) const {
		return TraceHull3(a, b, ducked ? 1 : 0, out);
	}

	// CM_RecursiveHullCheck's ORDER, reduced to brush enumeration: walk the
	// tree with the swept box, NEAR child first (the side the sweep starts
	// on), and append each touched leaf's brushes in leafbrush-lump order,
	// first occurrence only (the engine's checkcount). Processing order is
	// OBSERVABLE - an allsolid brush zeroes the fraction the moment it is
	// processed, so which earlier recording survives depends on it. Neither
	// ascending-index nor global-best matched the engine's answers; the leaf
	// walk is the engine's actual organization, so order falls out of shape.
	int World::OrderedLeafBrushes(const Vec3& a, const Vec3& b,
	                              const Vec3& mins, const Vec3& maxs,
	                              int* out_list, int cap) const {
		if (nodes.empty() || leaf_firstbrush.empty() || leafbrush_ours.empty())
			return -1;   // tables absent - caller falls back to the grid
		struct Walker {
			const World* w;
			Vec3 ca, cb, e;
			int* out;
			int cap, n;
			void Visit(int node) {
				while (node >= 0) {
					if (node >= static_cast<int>(w->nodes.size()))
						return;
					const BspNode& nd = w->nodes[node];
					if (nd.plane < 0
						|| nd.plane >= static_cast<int>(w->tree_n.size()))
						return;
					const Vec3& pn = w->tree_n[nd.plane];
					const float pd = w->tree_d[nd.plane];
					const float r = fabsf(pn.X) * e.X + fabsf(pn.Y) * e.Y
						+ fabsf(pn.Z) * e.Z;
					const float d0 = Dot(pn, ca) - pd;
					const float d1 = Dot(pn, cb) - pd;
					// Descend both sides whenever the swept box touches the
					// plane (strict >). TRIED AND REVERTED: CM's literal
					// '>= offset' front-only prune dropped 20 real startsolid
					// flags (engine-only 20 on the certified sweep) and took
					// funcdiff from 0+0 to 20+27 - the engine's own clip
					// visits those exactly-touching back sides, because its
					// recursion clips SEGMENTS, not whole subtrees. The one
					// residual ordering nuance (25 of 125 box-oracle rows,
					// full-hull boxes at one embedded site, normal-only, no
					// function output affected) needs true segment-splitting
					// recursion - an open item, not a fitted knob.
					if (d0 > r && d1 > r) { node = nd.child[0]; continue; }
					if (d0 < -r && d1 < -r) { node = nd.child[1]; continue; }
					const int near_c = (d0 >= 0.f) ? 0 : 1;
					Visit(nd.child[near_c]);
					node = nd.child[near_c ^ 1];
				}
				const int leaf = -1 - node;
				if (leaf < 0
					|| leaf >= static_cast<int>(w->leaf_firstbrush.size()))
					return;
				const int fb = w->leaf_firstbrush[leaf];
				const int nb = w->leaf_numbrushes[leaf];
				for (int i = 0; i < nb; ++i) {
					const int idx = fb + i;
					if (idx < 0
						|| idx >= static_cast<int>(w->leafbrush_ours.size()))
						continue;
					const int bi = w->leafbrush_ours[idx];
					if (bi < 0)
						continue;
					bool seen = false;
					for (int k = 0; k < n; ++k)
						if (out[k] == bi) { seen = true; break; }
					if (!seen && n < cap)
						out[n++] = bi;
				}
			}
		};
		Walker wk{ this,
			a + Vec3(0.5f * (mins.X + maxs.X), 0.5f * (mins.Y + maxs.Y),
			         0.5f * (mins.Z + maxs.Z)),
			b + Vec3(0.5f * (mins.X + maxs.X), 0.5f * (mins.Y + maxs.Y),
			         0.5f * (mins.Z + maxs.Z)),
			Vec3(0.5f * (maxs.X - mins.X), 0.5f * (maxs.Y - mins.Y),
			     0.5f * (maxs.Z - mins.Z)),
			out_list, cap, 0 };
		wk.Visit(headnode);
		return wk.n;
	}

	// Box-vs-tree walk: does the box at origin touch any CONTENTS_SOLID
	// leaf? Radius-expanded plane tests, both children when spanning.
	bool World::BoxInSolidLeaf(const Vec3& origin, const Vec3& mins,
	                           const Vec3& maxs) const {
		if (nodes.empty() || leaf_contents.empty())
			return false;
		const Vec3 c(origin.X + 0.5f * (mins.X + maxs.X),
		             origin.Y + 0.5f * (mins.Y + maxs.Y),
		             origin.Z + 0.5f * (mins.Z + maxs.Z));
		const Vec3 e(0.5f * (maxs.X - mins.X), 0.5f * (maxs.Y - mins.Y),
		             0.5f * (maxs.Z - mins.Z));
		struct Walker {
			const World* w;
			Vec3 c, e;
			bool Visit(int node) const {
				while (node >= 0) {
					if (node >= static_cast<int>(w->nodes.size()))
						return false;
					const BspNode& nd = w->nodes[node];
					if (nd.plane < 0
						|| nd.plane >= static_cast<int>(w->tree_n.size()))
						return false;
					const Vec3& pn = w->tree_n[nd.plane];
					const float pd = w->tree_d[nd.plane];
					const float r = fabsf(pn.X) * e.X + fabsf(pn.Y) * e.Y
						+ fabsf(pn.Z) * e.Z;
					const float d0 = Dot(pn, c) - pd;
					// TOUCHING a leaf boundary is NOT inside it: at
					// |d0| == r the box only meets the plane, so descend
					// one side. The both-sides variant made every box
					// standing exactly ON a floor "overlap" the solid
					// leaf below - FinishDuck's stuck ladder fired on
					// ordinary grounded states and broke a tape (544u).
					if (d0 >= r) { node = nd.child[0]; continue; }
					if (d0 <= -r) { node = nd.child[1]; continue; }
					if (Visit(nd.child[0]))
						return true;
					node = nd.child[1];
				}
				const int leaf = -1 - node;
				if (leaf < 0
					|| leaf >= static_cast<int>(w->leaf_contents.size()))
					return false;
				return (w->leaf_contents[leaf] & 1) != 0;   // CONTENTS_SOLID
			}
		};
		const Walker wk{ this, c, e };
		return wk.Visit(headnode);
	}

	float World::TraceHull3(const Vec3& a, const Vec3& b, int hull,
	                        TraceResult* out) const {
		const bool ducked = hull == 1;   // trace-log row semantics
		if (out) {
			out->frac = 1.f;
			out->brush = -1;
			out->plane = -1;
			out->startsolid = false;
			out->allsolid = false;
		}
		if (nx_ == 0)
			return 1.f;
		// NO out-of-world solidity rule. Measured directly against the
		// engine's own trace outputs (soliddiff over the oracle queries):
		// for positions outside the map the engine returns startsolid 0,
		// allsolid 0 - and across 6,624 queries there is NOT ONE input
		// where the engine reports solid and we do not. Every rule I added
		// here (AABB, then BSP leaf contents) only ever over-reported.
		// Solidity comes from the brush clip below, nowhere else.
		const Vec3 lo(fminf(a.X, b.X), fminf(a.Y, b.Y), fminf(a.Z, b.Z));
		const Vec3 hi(fmaxf(a.X, b.X), fmaxf(a.Y, b.Y), fmaxf(a.Z, b.Z));
		int x0, x1, y0, y1, z0, z1;
		CellRange(lo, hi, &x0, &x1, &y0, &y1, &z0, &z1);

		float best = 1.f;
		int best_brush = -1, best_plane = -1;
		// THE START-SOLID LAW (engine's own answers, 2026-08-15): a brush the
		// sweep starts inside contributes FLAGS only - it is geometrically
		// invisible (0 of 1,236 ss-only engine rows zeroed the fraction). An
		// ALLSOLID brush (start AND end inside) zeroes the fraction THE
		// MOMENT IT IS PROCESSED (789/789 rows frac 0) - so recordings made
		// BEFORE it survive and later brushes cannot record (their clamped
		// enter fraction is never < 0). That mid-loop mechanism is exactly
		// CM_ClipBoxToBrush + CM_TraceToLeaf, and it makes PROCESSING ORDER
		// observable - so brushes are clipped in the engine's own order: the
		// leaf walk, near side first, per-leaf lump order.
		bool l_ss = false, l_as = false;
		const Vec3& wmn = hull == 1 ? hulls_.duck_min
			: hull == 2 ? hulls_.unduck_min : hulls_.stand_min;
		const Vec3& wmx = hull == 1 ? hulls_.duck_max
			: hull == 2 ? hulls_.unduck_max : hulls_.stand_max;
		// THE UNSWEPT QUESTION (open, precisely named): the raw-ray oracle
		// and grounded CanUnduck consult LEAF CONTENTS on zero-length tests
		// (212 blocked-while-brush-clear rows on the ceiling slab), but
		// FixPlayerCrouchStuck's ladder and TryPlayerMove's stuck-guard
		// measurably do NOT (wiring the leaf rule here took FinishDuck
		// 496 -> 1,147 and broke the solved12 tape at its void-crossing
		// finish flight). Two zero-length consumers, two behaviors - the
		// distinction lives in CanUnduck's grounded branch at client.dll
		// 0x1f4c5f, and READING that body is the next instrument; until
		// then this movement-path trace stays brushes-only and the 212
		// stay counted as open, not papered over. BoxInSolidLeaf carries
		// the leaf law for the instruments that need it.
		int order[1024];
		const int norder = OrderedLeafBrushes(a, b, wmn, wmx, order, 1024);
		// Fallback when leaf tables are absent: grid gathering, cell-major
		// (order then unproven - real BSPs always have the tables).
		std::vector<int> fb;
		if (norder < 0) {
			for (int z = z0; z <= z1; ++z)
			for (int y = y0; y <= y1; ++y)
			for (int x = x0; x <= x1; ++x) {
				const std::vector<int>& cell =
					cells_[(static_cast<size_t>(z) * ny_ + y) * nx_ + x];
				for (int bi : cell)
					if (std::find(fb.begin(), fb.end(), bi) == fb.end())
						fb.push_back(bi);
			}
		}
		const int* blist = norder >= 0 ? order : fb.data();
		const int bcount = norder >= 0 ? norder : static_cast<int>(fb.size());
		{
			for (int oi = 0; oi < bcount; ++oi) {
				const int bi = blist[oi];
				const WorldBrush& bc = brushes[bi];
				const Vec3& gmn = hull == 1 ? bc.gmin_duck
					: hull == 2 ? bc.gmin_unduck : bc.gmin_stand;
				const Vec3& gmx = hull == 1 ? bc.gmax_duck
					: hull == 2 ? bc.gmax_unduck : bc.gmax_stand;
				if (hi.X < gmn.X || lo.X > gmx.X ||
				    hi.Y < gmn.Y || lo.Y > gmx.Y ||
				    hi.Z < gmn.Z || lo.Z > gmx.Z)
					continue;
				const std::vector<float>& pd = hull == 1 ? bc.d_duck
					: hull == 2 ? bc.d_unduck : bc.d_stand;
				float tmin = -1.f, tmax = 1.f;   // engine: enterfrac -1, leavefrac 1
				// CORNER RELEASE - ORACLE-PINNED (2026-08-13, 3,555 engine
				// answers, zero disagreements): the brush is skipped iff the
				// TRUE (un-padded) entry time is >= the earliest REAL-SIDE
				// leave extended by one DIST_EPSILON ((d0 - eps)/(d0 - d1)).
				// BEVEL sides never release (they exist to smooth corners;
				// releasing through them would punch holes at every corner -
				// and the engine measurably does not: solved7 t512, the spine
				// climb, hits where a bevel-leave would release). The padded
				// interval still decides the fraction. Empirical boundary
				// pinned by two independent sweeps to one 0.005u step.
				float tmin_t = -1.f, tmax_t = 1e9f;
				int enter = -1;
				bool outside = false, miss = false;
				bool getout = false;   // any plane with the END point outside
				const int np = static_cast<int>(bc.n.size());
				for (int pi = 0; pi < np; ++pi) {
					const float d0 = Dot(bc.n[pi], a) - pd[pi];
					const float d1 = Dot(bc.n[pi], b) - pd[pi];
					if (d1 > 0.f)
						getout = true;   // engine: endpoint not in solid
					if (d0 > 0.f) {
						outside = true;
						if (d1 > 0.f) { miss = true; break; }
						// ENGINE-EXACT enter selection (oracle query 6654,
						// 2026-08-15): CM_ClipBoxToBrush CLAMPS the padded
						// enter fraction to 0 BEFORE the compare, so every
						// started-touching plane ties at exactly 0 and the
						// strict '>' keeps the FIRST in brushside (lump)
						// order - SIDE ORDER is the tie-break, not least-
						// negative. At ramp 2's base the wall (side 2,
						// tt -0.029) beats the slope (side 5, tt exactly
						// 0.000 = one DIST_EPSILON standoff); the unclamped
						// H' rule picked the slope and manufactured a climb
						// the engine never grants. H' survived its 3,555
						// answers because two started-touching entries never
						// competed there; the clamp rule reproduces all of
						// those AND this seam.
						float tt = (d0 - kDistEpsilon) / (d0 - d1);
						if (tt < 0.f)
							tt = 0.f;
						if (tt > tmin) { tmin = tt; enter = pi; }
						const float tn = d0 / (d0 - d1);
						if (tn > tmin_t) tmin_t = tn;
					} else if (d1 > 0.f) {
						float tt = (d0 + kDistEpsilon) / (d0 - d1);
						if (tt > 1.f) tt = 1.f;
						if (tt < tmax) tmax = tt;
						if (bc.pid[pi] >= 0) {   // real sides only
							const float tn = (d0 - kDistEpsilon) / (d0 - d1);
							if (tn < tmax_t) tmax_t = tn;
						}
					}
				}
				// ENGINE-EXACT solid reporting (CM_ClipBoxToBrush): a brush
				// the sweep STARTS inside (no plane had d0 > 0) sets
				// startsolid and returns WITHOUT touching the fraction; if
				// no plane had d1 > 0 either, the box never gets out and
				// allsolid is set - and TryPlayerMove zeroes velocity on
				// allsolid. That zeroing IS the surf "ramp bug": measured
				// in 32.7% of fuzz probes and in the d34 tape's t193, where
				// the engine froze for 18 ticks at v(0,0,-6) while our
				// model flew on at 880 u/s.
				if (!miss && !outside) {
					l_ss = true;
					if (!getout) {
						l_as = true;
						// The engine's mid-loop zeroing: fraction drops to 0
						// HERE, the recorded plane stays, and everything
						// processed after this cannot beat 0.
						best = 0.f;
					}
					continue;
				}
				if (miss || !outside || enter < 0 || tmin >= tmax)
					continue;
				if (true_interval_corner && tmin_t >= tmax_t)
					continue;   // corner release (engine-measured rule)
				if (tmin < best) {
					best = (tmin > 0.f) ? tmin : 0.f;
					best_brush = bi;
					best_plane = enter;
				}
			}
		}
		if (out) {
			out->startsolid = l_ss;
			out->allsolid = l_as;
			out->frac = best;   // 0 when an allsolid brush zeroed it
		}
		if (out && best_brush >= 0) {
			out->brush = best_brush;
			out->plane = best_plane;
			out->normal = brushes[best_brush].n[best_plane];
		}
		if (trace_log) {
			TraceProbeRow row;
			row.tick = trace_tick;
			row.a = a;
			row.b = b;
			row.ducked = ducked;
			row.frac = best;
			if (best_brush >= 0)
				row.n = brushes[best_brush].n[best_plane];
			row.brush_id = best_brush >= 0 ? brushes[best_brush].id : -1;
			trace_log->push_back(row);
		}
		return best;
	}

	// Arbitrary-box sweep. Same clip semantics as TraceHull3 (unclamped
	// enterfrac start, engine clamp at the compare, corner release, solid
	// reporting) but with the plane expansion computed per query, because
	// TracePlayerBBoxForGround sweeps quadrant boxes that are not hulls.
	float World::TraceHullBox(const Vec3& a, const Vec3& b, const Vec3& mins,
	                          const Vec3& maxs, TraceResult* out) const {
		if (out) {
			out->frac = 1.f;
			out->brush = -1;
			out->plane = -1;
			out->startsolid = false;
			out->allsolid = false;
		}
		if (nx_ == 0)
			return 1.f;
		const Vec3 lo(fminf(a.X, b.X), fminf(a.Y, b.Y), fminf(a.Z, b.Z));
		const Vec3 hi(fmaxf(a.X, b.X), fmaxf(a.Y, b.Y), fmaxf(a.Z, b.Z));
		int x0, x1, y0, y1, z0, z1;
		CellRange(lo + mins, hi + maxs, &x0, &x1, &y0, &y1, &z0, &z1);
		float best = 1.f;
		int best_brush = -1, best_plane = -1;
		// Same start-solid law and ENGINE ORDER as TraceHull3. UNLIKE
		// TraceHull3 (the movement-path trace), this function models the
		// RAW IEngineTrace::TraceRay - the oracle's own mechanism and
		// CanUnduck's (decoded @0x1f4ed6) - so the unswept leaf-contents
		// law applies here: a zero-length box in a CONTENTS_SOLID leaf is
		// startsolid+allsolid even with no brush overlap (measured: 1,089
		// then 11,574-row box-oracle batches).
		if (a.X == b.X && a.Y == b.Y && a.Z == b.Z
			&& BoxInSolidLeaf(a, mins, maxs)) {
			if (out) {
				out->startsolid = true;
				out->allsolid = true;
				out->frac = 0.f;
			}
			return 0.f;
		}
		bool l_ss = false, l_as = false;
		int order[1024];
		const int norder = OrderedLeafBrushes(a, b, mins, maxs, order, 1024);
		std::vector<int> fbx;
		if (norder < 0) {
			for (int z = z0; z <= z1; ++z)
			for (int y = y0; y <= y1; ++y)
			for (int x = x0; x <= x1; ++x) {
				const std::vector<int>& cell =
					cells_[(static_cast<size_t>(z) * ny_ + y) * nx_ + x];
				for (int bi : cell)
					if (std::find(fbx.begin(), fbx.end(), bi) == fbx.end())
						fbx.push_back(bi);
			}
		}
		const int* blist = norder >= 0 ? order : fbx.data();
		const int bcount = norder >= 0 ? norder : static_cast<int>(fbx.size());
		{
			for (int oi = 0; oi < bcount; ++oi) {
				const int bi = blist[oi];
				const WorldBrush& bc = brushes[bi];
				float tmin = -1.f, tmax = 1.f;
				float tmin_t = -1.f, tmax_t = 1e9f;
				int enter = -1;
				bool outside = false, miss = false, getout = false;
				const int np = static_cast<int>(bc.n.size());
				for (int pi = 0; pi < np; ++pi) {
					const float pd = bc.d[pi] + HullExpand(bc.n[pi], mins, maxs);
					const float d0 = Dot(bc.n[pi], a) - pd;
					const float d1 = Dot(bc.n[pi], b) - pd;
					if (d1 > 0.f)
						getout = true;
					if (d0 > 0.f) {
						outside = true;
						if (d1 > 0.f) { miss = true; break; }
						float tt = (d0 - kDistEpsilon) / (d0 - d1);
						if (tt < 0.f) tt = 0.f;
						if (tt > tmin) { tmin = tt; enter = pi; }
						const float tn = d0 / (d0 - d1);
						if (tn > tmin_t) tmin_t = tn;
					} else if (d1 > 0.f) {
						float tt = (d0 + kDistEpsilon) / (d0 - d1);
						if (tt > 1.f) tt = 1.f;
						if (tt < tmax) tmax = tt;
						if (bc.pid[pi] >= 0) {
							const float tn = (d0 - kDistEpsilon) / (d0 - d1);
							if (tn < tmax_t) tmax_t = tn;
						}
					}
				}
				if (!miss && !outside) {
					l_ss = true;
					if (!getout) {
						l_as = true;
						best = 0.f;   // mid-loop zeroing, plane survives
					}
					continue;
				}
				if (miss || !outside || enter < 0 || tmin >= tmax)
					continue;
				if (true_interval_corner && tmin_t >= tmax_t)
					continue;
				if (tmin < best) {
					best = (tmin > 0.f) ? tmin : 0.f;
					best_brush = bi;
					best_plane = enter;
				}
			}
		}
		if (out) {
			out->startsolid = l_ss;
			out->allsolid = l_as;
			out->frac = best;
		}
		if (out && best_brush >= 0) {
			out->brush = best_brush;
			out->plane = best_plane;
			out->normal = brushes[best_brush].n[best_plane];
		}
		return best;
	}

	bool World::PointInSolidLeaf(const Vec3& p) const {
		if (nodes.empty() || leaf_contents.empty())
			return false;
		int n = headnode;
		int guard = 0;
		while (n >= 0 && guard++ < 4096) {
			if (n >= static_cast<int>(nodes.size()))
				return false;
			const BspNode& nd = nodes[n];
			if (nd.plane < 0 || nd.plane >= static_cast<int>(tree_n.size()))
				return false;
			const float d = Dot(tree_n[nd.plane], p) - tree_d[nd.plane];
			n = nd.child[d >= 0.f ? 0 : 1];
		}
		const int leaf = -1 - n;
		if (leaf < 0 || leaf >= static_cast<int>(leaf_contents.size()))
			return false;
		return (leaf_contents[leaf] & 1) != 0;   // CONTENTS_SOLID
	}

	bool World::HullInSolidLeaf(const Vec3& o, int hull) const {
		const Vec3& mn = hull == 1 ? hulls_.duck_min
			: hull == 2 ? hulls_.unduck_min : hulls_.stand_min;
		const Vec3& mx = hull == 1 ? hulls_.duck_max
			: hull == 2 ? hulls_.unduck_max : hulls_.stand_max;
		// Engine sweeps the BOX: the void behind a wall is reached by a
		// corner long before the origin crosses.
		for (int i = 0; i < 8; ++i) {
			const Vec3 c(o.X + ((i & 1) ? mx.X : mn.X),
			             o.Y + ((i & 2) ? mx.Y : mn.Y),
			             o.Z + ((i & 4) ? mx.Z : mn.Z));
			if (PointInSolidLeaf(c))
				return true;
		}
		return PointInSolidLeaf(o);
	}

	bool World::OriginInSolid(const Vec3& o, bool ducked) const {
		if (nx_ == 0)
			return false;
		int x0, x1, y0, y1, z0, z1;
		CellRange(o, o, &x0, &x1, &y0, &y1, &z0, &z1);
		for (int z = z0; z <= z1; ++z)
		for (int y = y0; y <= y1; ++y)
		for (int x = x0; x <= x1; ++x) {
			const std::vector<int>& cell =
				cells_[(static_cast<size_t>(z) * ny_ + y) * nx_ + x];
			for (int bi : cell) {
				const WorldBrush& bc = brushes[bi];
				const std::vector<float>& pd = ducked ? bc.d_duck : bc.d_stand;
				bool inside = true;
				for (size_t pi = 0; pi < bc.n.size() && inside; ++pi)
					// Strictly inside by more than DIST_EPSILON: resting ON a
					// face (dist 0) must not read as stuck.
					if (Dot(bc.n[pi], o) - pd[pi] >= -kDistEpsilon)
						inside = false;
				if (inside)
					return true;
			}
		}
		return false;
	}

	int World::BrushUnder(const Vec3& o, bool ducked) const {
		TraceResult tr;
		TraceHull(o, o - Vec3(0.f, 0.f, 4096.f), ducked, &tr);
		return (tr.brush >= 0) ? brushes[tr.brush].id : -1;
	}

	int World::IndexOfBrushId(int bsp_id) const {
		for (int i = 0; i < static_cast<int>(brushes.size()); ++i)
			if (brushes[i].id == bsp_id)
				return i;
		return -1;
	}

} // namespace Solver
