#include "SolverRoute.h"

#include <math.h>
#include <string.h>

namespace Solver {
namespace Route {

	namespace {

		Vec3 Cross(const Vec3& a, const Vec3& b) {
			return Vec3(a.Y * b.Z - a.Z * b.Y,
			            a.Z * b.X - a.X * b.Z,
			            a.X * b.Y - a.Y * b.X);
		}
		Vec3 Norm(const Vec3& v) {
			const float l = Len(v);
			return l > 1e-12f ? Scale(v, 1.f / l) : Vec3(0, 0, 0);
		}

		// Clip a polygon by the half-space n.x <= d (keep inside).
		void ClipPoly(std::vector<Vec3>& poly, const Vec3& n, float d) {
			if (poly.empty())
				return;
			std::vector<Vec3> out;
			out.reserve(poly.size() + 2);
			const size_t m = poly.size();
			for (size_t i = 0; i < m; ++i) {
				const Vec3& a = poly[i];
				const Vec3& b = poly[(i + 1) % m];
				const float da = Dot(n, a) - d;
				const float db = Dot(n, b) - d;
				if (da <= 0.f)
					out.push_back(a);
				if ((da < 0.f && db > 0.f) || (da > 0.f && db < 0.f)) {
					const float t = da / (da - db);
					out.push_back(a + Scale(b - a, t));
				}
			}
			poly.swap(out);
		}

	} // namespace

	bool Build(const World& w, Graph* out, float max_edge_dist,
	           std::string* err) {
		if (!out) return false;
		out->faces.clear();
		out->edges.clear();

		for (size_t bi = 0; bi < w.brushes.size(); ++bi) {
			const WorldBrush& b = w.brushes[bi];
			const int np = static_cast<int>(b.n.size());
			for (int pi = 0; pi < np; ++pi) {
				if (pi < static_cast<int>(b.pid.size()) && b.pid[pi] < 0)
					continue;   // compiler bevel, not a real surface
				const Vec3& n = b.n[pi];
				if (n.Z <= 0.05f || n.Z >= 0.7f)
					continue;   // walls hold nothing; walkable is ground

				// Start from a large quad on the plane and clip by the
				// brush's other half-spaces.
				Vec3 u = fabsf(n.Z) < 0.9f ? Cross(n, Vec3(0, 0, 1))
				                           : Cross(n, Vec3(1, 0, 0));
				u = Norm(u);
				const Vec3 v = Cross(n, u);
				const Vec3 c0 = Scale(n, b.d[pi]);
				const float R = 8192.f;
				std::vector<Vec3> poly = {
					c0 + Scale(u, R) + Scale(v, R),
					c0 - Scale(u, R) + Scale(v, R),
					c0 - Scale(u, R) - Scale(v, R),
					c0 + Scale(u, R) - Scale(v, R),
				};
				for (int pj = 0; pj < np && !poly.empty(); ++pj) {
					if (pj == pi)
						continue;
					ClipPoly(poly, b.n[pj], b.d[pj]);
				}
				if (poly.size() < 3)
					continue;

				Face f;
				f.brush = static_cast<int>(bi);
				f.side = pi;
				f.n = n;
				f.d = b.d[pi];
				f.verts = poly;
				f.zmin = 1e9f;
				f.zmax = -1e9f;
				Vec3 c(0, 0, 0);
				for (const Vec3& p : poly) {
					c = c + p;
					if (p.Z < f.zmin) f.zmin = p.Z;
					if (p.Z > f.zmax) f.zmax = p.Z;
				}
				f.centroid = Scale(c, 1.f / static_cast<float>(poly.size()));
				float area2 = 0.f;
				for (size_t k = 1; k + 1 < poly.size(); ++k)
					area2 += Len(Cross(poly[k] - poly[0],
						poly[k + 1] - poly[0]));
				f.area = 0.5f * area2;
				if (f.area < 64.f)
					continue;   // slivers: not rideable surface
				// Gravity's in-plane component: g projected onto the plane.
				const Vec3 g(0, 0, -1);
				const Vec3 dh = g - Scale(n, Dot(g, n));
				f.downhill = Len(dh) > 1e-6f ? Norm(dh) : Vec3(0, 0, 0);
				out->faces.push_back(std::move(f));
			}
		}

		// Coarse candidate transfers: centroid proximity, either direction
		// of travel (stage 2's analytic envelopes replace this gate).
		const size_t nf = out->faces.size();
		for (size_t i = 0; i < nf; ++i)
			for (size_t j = 0; j < nf; ++j) {
				if (i == j)
					continue;
				const Face& A = out->faces[i];
				const Face& B = out->faces[j];
				if (A.brush == B.brush)
					continue;
				const float dist = Len(B.centroid - A.centroid);
				if (dist > max_edge_dist)
					continue;
				Edge e;
				e.from = static_cast<int>(i);
				e.to = static_cast<int>(j);
				e.dist = dist;
				e.dz = B.centroid.Z - A.centroid.Z;
				out->edges.push_back(e);
			}

		if (out->faces.empty()) {
			if (err) *err = "no surfable faces found";
			return false;
		}
		return true;
	}

	bool AnchorZones(const World& w, Graph* g, const Vec3& anchor_pos,
	                 bool ducked, int end_brush_id, float max_edge_dist,
	                 std::string* err) {
		if (!g)
			return false;
		g->start_pos = anchor_pos;
		const int start_id = w.BrushUnder(anchor_pos, ducked);
		g->start_brush = w.IndexOfBrushId(start_id);
		if (g->start_brush < 0) {
			if (err) *err = "no brush under the anchor origin";
			return false;
		}
		g->end_brush =
			end_brush_id >= 0 ? w.IndexOfBrushId(end_brush_id) : -1;
		if (end_brush_id >= 0 && g->end_brush < 0) {
			if (err) *err = "end brush id not found in world";
			return false;
		}
		if (g->end_brush >= 0) {
			const WorldBrush& e = w.brushes[g->end_brush];
			g->end_center = Scale(e.bmin + e.bmax, 0.5f);
		}
		g->start_faces.clear();
		g->end_faces.clear();
		for (size_t i = 0; i < g->faces.size(); ++i) {
			const Face& f = g->faces[i];
			if (Len(f.centroid - anchor_pos) <= max_edge_dist)
				g->start_faces.push_back(static_cast<int>(i));
			if (g->end_brush >= 0
				&& Len(f.centroid - g->end_center) <= max_edge_dist)
				g->end_faces.push_back(static_cast<int>(i));
		}
		return true;
	}

} // namespace Route
} // namespace Solver
