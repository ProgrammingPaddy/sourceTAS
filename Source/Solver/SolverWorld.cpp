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
		std::vector<uint8_t> is_world(nbrushes_total, 0);
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

			int32_t headnode = 0;
			std::memcpy(&headnode, p + lumps[kLumpModels].ofs + 36, 4);

			std::vector<int32_t> stack;
			stack.push_back(headnode);
			while (!stack.empty()) {
				const int32_t node = stack.back();
				stack.pop_back();
				if (node < 0) {
					const int leaf = -1 - node;
					if (leaf < 0 || leaf >= nleafs)
						continue;
					const char* lp = p + lumps[kLumpLeafs].ofs + leaf * leaf_stride;
					uint16_t first = 0, count = 0;
					std::memcpy(&first, lp + 20, 2);
					std::memcpy(&count, lp + 22, 2);
					for (int i = 0; i < count; ++i) {
						const int lbi = first + i;
						if (lbi < 0 || lbi >= nleafbrushes)
							continue;
						uint16_t bid = 0;
						std::memcpy(&bid, p + lumps[kLumpLeafBrushes].ofs + lbi * 2, 2);
						if (bid < nbrushes_total)
							is_world[bid] = 1;
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

			// Hull expansions (both hulls) + origin-space AABB gates.
			wb.d_stand.resize(wb.n.size());
			wb.d_duck.resize(wb.n.size());
			for (size_t i = 0; i < wb.n.size(); ++i) {
				wb.d_stand[i] = wb.d[i] + HullExpand(wb.n[i], hulls_.stand_min, hulls_.stand_max);
				wb.d_duck[i] = wb.d[i] + HullExpand(wb.n[i], hulls_.duck_min, hulls_.duck_max);
			}
			wb.gmin_stand = wb.bmin - hulls_.stand_max;
			wb.gmax_stand = wb.bmax - hulls_.stand_min;
			wb.gmin_duck = wb.bmin - hulls_.duck_max;
			wb.gmax_duck = wb.bmax - hulls_.duck_min;

			brushes.push_back(std::move(wb));
		}

		BuildGrid();
		return true;
	}

	void World::BuildGrid() {
		cells_.clear();
		nx_ = ny_ = nz_ = 0;
		if (brushes.empty())
			return;
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
		if (out) {
			out->frac = 1.f;
			out->brush = -1;
			out->plane = -1;
		}
		if (nx_ == 0)
			return 1.f;
		const Vec3 lo(fminf(a.X, b.X), fminf(a.Y, b.Y), fminf(a.Z, b.Z));
		const Vec3 hi(fmaxf(a.X, b.X), fmaxf(a.Y, b.Y), fmaxf(a.Z, b.Z));
		int x0, x1, y0, y1, z0, z1;
		CellRange(lo, hi, &x0, &x1, &y0, &y1, &z0, &z1);

		float best = 1.f;
		int best_brush = -1, best_plane = -1;
		// Duplicate visits across cells re-run an idempotent min - cheaper than
		// a mutable visited stamp, and keeps the trace const + thread-safe.
		for (int z = z0; z <= z1; ++z)
		for (int y = y0; y <= y1; ++y)
		for (int x = x0; x <= x1; ++x) {
			const std::vector<int>& cell =
				cells_[(static_cast<size_t>(z) * ny_ + y) * nx_ + x];
			for (int bi : cell) {
				const WorldBrush& bc = brushes[bi];
				const Vec3& gmn = ducked ? bc.gmin_duck : bc.gmin_stand;
				const Vec3& gmx = ducked ? bc.gmax_duck : bc.gmax_stand;
				if (hi.X < gmn.X || lo.X > gmx.X ||
				    hi.Y < gmn.Y || lo.Y > gmx.Y ||
				    hi.Z < gmn.Z || lo.Z > gmx.Z)
					continue;
				const std::vector<float>& pd = ducked ? bc.d_duck : bc.d_stand;
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
				const int np = static_cast<int>(bc.n.size());
				for (int pi = 0; pi < np; ++pi) {
					const float d0 = Dot(bc.n[pi], a) - pd[pi];
					const float d1 = Dot(bc.n[pi], b) - pd[pi];
					if (d0 > 0.f) {
						outside = true;
						if (d1 > 0.f) { miss = true; break; }
						// UNCLAMPED compare: started-touching corners pick the
						// least-negative enterfrac (the engine's tie-break).
						const float tt = (d0 - kDistEpsilon) / (d0 - d1);
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
		if (out && best_brush >= 0) {
			out->frac = best;
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
