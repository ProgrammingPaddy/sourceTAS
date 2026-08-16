#include "SolverRouteSearch.h"

#include <float.h>
#include <math.h>

#include <algorithm>

#include "SolverEnvelope.h"
#include "SolverSteer.h"

namespace Solver {
namespace RouteSearch {

	namespace {

		// Horizontal min distance between two vertex sets, with the
		// closest pair reported (departure/arrival anchors for ride
		// traversal pricing). Vertex-pair min is a fine LB source.
		float MinDist2D(const std::vector<Vec3>& a,
		                const std::vector<Vec3>& b,
		                Vec3* pa = nullptr, Vec3* pb = nullptr) {
			float best = FLT_MAX;
			for (const Vec3& va : a)
				for (const Vec3& vb : b) {
					const float dx = va.X - vb.X;
					const float dy = va.Y - vb.Y;
					const float d = sqrtf(dx * dx + dy * dy);
					if (d < best) {
						best = d;
						if (pa) *pa = va;
						if (pb) *pb = vb;
					}
				}
			return best == FLT_MAX ? 0.f : best;
		}

		struct Region {
			std::vector<Vec3> pts;   // representative points (verts)
			float zmin = 0.f, zmax = 0.f;
			float tan_slope = 0.f;   // |in-plane vz| per unit of s2d
		};

		Region FaceRegion(const Route::Face& f) {
			Region r;
			r.pts = f.verts;
			r.zmin = f.zmin;
			r.zmax = f.zmax;
			const float h = sqrtf(f.n.X * f.n.X + f.n.Y * f.n.Y);
			r.tan_slope = f.n.Z > 0.05f ? h / f.n.Z : 3.f;
			return r;
		}

		// The M2.1 edge bound under the POTENTIAL LEDGER: departure is
		// a POINT (z0, vz family, s2d bound); the cheapest arrival at
		// region B is a ballistic strike, or - for END - landing short
		// and running the remainder. Reports the tick cost, the flight
		// n used, and the LOW edge of the feasible arrival window (the
		// energy anchor: lowest arrival = most kinetic energy; anchors
		// telescope so any consistent choice is sound).
		bool EdgeBound(float z0, float vz_lo, float vz_hi, float s2d_ub,
		               const Region& B, const MoveParams& p, int max_n,
		               float dist, bool is_end, float* cost_ticks,
		               int* n_used, float* z_arr) {
			const float g = p.gravity;
			const float c2 = p.air_speed_cap * p.air_speed_cap;
			const float zb_lo = B.zmin - (is_end ? 400.f : 40.f);
			const float zb_hi = B.zmax + 40.f;
			const float run_per_tick = p.maxspeed * p.dt;
			float d_cov = 0.f;
			float best = -1.f;
			int best_n = 0;
			float best_z = 0.f;
			for (int n = 1; n <= max_n; ++n) {
				const float t = static_cast<float>(n);
				const float drop = 0.5f * g * p.dt * p.dt * t * t;
				const float z_lo = z0 + t * p.dt * vz_lo - drop;
				const float z_hi = z0 + t * p.dt * vz_hi - drop;
				d_cov += sqrtf(s2d_ub * s2d_ub + c2 * t) * p.dt;
				if (z_hi < zb_lo)
					break;   // whole family fell through
				if (z_lo > zb_hi)
					continue;
				const float za = (z_lo > zb_lo ? z_lo : zb_lo);
				if (d_cov >= dist) {
					if (best < 0.f || t < best) {
						best = t;
						best_n = n;
						best_z = za;
					}
					break;   // later n only costs more
				}
				if (is_end) {
					const float cost = t + (dist - d_cov)
						/ run_per_tick;
					if (best < 0.f || cost < best) {
						best = cost;
						best_n = n;
						best_z = za;
					}
				}
			}
			if (best < 0.f)
				return false;
			*cost_ticks = best;
			*n_used = best_n;
			*z_arr = best_z;
			return true;
		}

	} // namespace

	float MeasurePrestrafeCeiling(const World& w, const MoveParams& p,
	                              const Vec3& start_pos, bool ducked) {
		float best = 0.f;
		const float rates[6] = { 2.f, 3.f, 4.5f, 6.f, 8.f, 10.f };
		for (int ri = 0; ri < 6; ++ri) {
			for (int sgn = -1; sgn <= 1; sgn += 2) {
				PlayerState s;
				s.pos = start_pos;
				s.ducked = ducked;
				s.hull_state = ducked ? 1 : 0;
				{
					TraceResult tr;
					const float gf = w.TraceHull(s.pos,
						s.pos - Vec3(0.f, 0.f, 2.f), s.ducked, &tr);
					if (gf < 1.f && tr.brush >= 0
						&& tr.normal.Z >= p.walkable_z) {
						s.pos.Z -= 2.f * gf;
						s.on_ground = true;
						s.ground_brush = tr.brush;
					}
				}
				float yaw = 0.f;
				for (int k = 0; k < 150; ++k) {
					yaw += rates[ri] * static_cast<float>(sgn);
					TickEvents ev;
					MoveTick(s, w, p, 0.f, yaw, 450.f,
						450.f * static_cast<float>(sgn), 0.f, 0, &ev);
					if (!s.on_ground)
						break;
					const float sp = Len2D(s.vel);
					if (sp > best)
						best = sp;
				}
			}
		}
		return best * 1.15f;
	}

	bool Enumerate(const World& w, const Route::Graph& g,
	               const MoveParams& p, const Opts& o,
	               std::vector<Candidate>* out, std::string* err) {
		out->clear();
		if (g.start_brush < 0 || g.end_brush < 0) {
			if (err) *err = "graph missing start/end anchoring";
			return false;
		}
		const int nf = static_cast<int>(g.faces.size());

		// Regions: faces, START (jump family off the platform), END
		// (the finish brush AABB).
		std::vector<Region> freg(nf);
		for (int i = 0; i < nf; ++i)
			freg[i] = FaceRegion(g.faces[i]);
		Region start;
		start.pts.push_back(g.start_pos);
		start.zmin = g.start_pos.Z;
		start.zmax = g.start_pos.Z;
		Region endr;
		{
			const WorldBrush& eb = w.brushes[g.end_brush];
			endr.pts.push_back(Vec3(eb.bmin.X, eb.bmin.Y, eb.bmin.Z));
			endr.pts.push_back(Vec3(eb.bmax.X, eb.bmin.Y, eb.bmin.Z));
			endr.pts.push_back(Vec3(eb.bmin.X, eb.bmax.Y, eb.bmin.Z));
			endr.pts.push_back(Vec3(eb.bmax.X, eb.bmax.Y, eb.bmin.Z));
			endr.zmin = eb.bmin.Z;
			endr.zmax = eb.bmax.Z + 72.f;
		}

		const float pre_s = o.prestrafe_speed > 0.f
			? o.prestrafe_speed
			: MeasurePrestrafeCeiling(w, p, g.start_pos, false);

		// Best-first over (node, path); admissible lb -> popped
		// completions are the true top-K by lb. State = THE POTENTIAL
		// LEDGER: total energy E (v^2 UB) + the arrival z anchor +
		// the 2D arrival point. Everything - flights and rides alike -
		// obeys E' = E + 900*ticks + 2g*(z - z'); per-node anchors
		// telescope, so cycles net exactly their wish work and cannot
		// harvest fake energy (the corner-bounce exploit of the first
		// two attempts).
		struct Node {
			std::vector<int> faces;
			int at = -2;             // -2 START, -1 END, else face
			float lb = 0.f;
			float E = 0.f;           // total v^2 UB at (at_pt, z_ref)
			float z_ref = 0.f;
			Vec3 at_pt;              // 2D arrival anchor
			bool operator<(const Node& b) const { return lb > b.lb; }
		};
		std::vector<Node> open;
		Node root;
		root.at = -2;
		// The single legal jump folds into the start energy.
		const float jvz = static_cast<float>(p.jump_impulse_d);
		root.E = pre_s * pre_s + jvz * jvz;
		root.z_ref = g.start_pos.Z;
		root.at_pt = g.start_pos;
		open.push_back(root);
		std::vector<Candidate> done;
		int safety = 0;
		while (!open.empty() && safety++ < 200000
			&& static_cast<int>(done.size()) < o.top_k) {
			std::pop_heap(open.begin(), open.end());
			Node cur = open.back();
			open.pop_back();
			if (cur.at == -1) {
				Candidate c;
				c.faces = cur.faces;
				c.lb_ticks = cur.lb;
				c.end_s2_ub = cur.E;
				done.push_back(c);
				continue;
			}
			const Region& from = cur.at == -2 ? start
				: freg[cur.at];
			for (int to = -1; to < nf; ++to) {
				if (to == cur.at)
					continue;
				if (to >= 0) {
					int visits = 0;
					for (int fidx : cur.faces)
						if (fidx == to)
							visits++;
					if (visits >= o.revisit_cap)
						continue;
					if (static_cast<int>(cur.faces.size())
						>= o.max_len)
						continue;
				}
				const Region& dst = to < 0 ? endr : freg[to];
				Vec3 dep_pt, arr_pt;
				const float dist = MinDist2D(from.pts, dst.pts,
					&dep_pt, &arr_pt);
				// Ride: arrival anchor -> departure anchor (the 2D
				// closest vertex toward the destination), priced at
				// the ledger speed with the ride's own gain folded
				// in by one fixed-point pass.
				float ride_ticks = 0.f;
				float E_dep = cur.E;
				float z_dep = cur.z_ref;
				if (cur.at >= 0) {
					z_dep = dep_pt.Z;
					E_dep = cur.E + 2.f * p.gravity
						* (cur.z_ref - z_dep);
					if (E_dep < 100.f) E_dep = 100.f;
					const float dxr = dep_pt.X - cur.at_pt.X;
					const float dyr = dep_pt.Y - cur.at_pt.Y;
					const float dr = sqrtf(dxr * dxr + dyr * dyr);
					ride_ticks = dr / (sqrtf(E_dep) * p.dt);
					ride_ticks = dr / (sqrtf(E_dep
						+ 900.f * ride_ticks) * p.dt);
					if (ride_ticks < 1.f) ride_ticks = 1.f;
					E_dep += 900.f * ride_ticks;
				}
				// Departure vz family: in-plane exits for faces; the
				// jump arc for START.
				const float s2d_ub = sqrtf(E_dep);
				float vz_lo, vz_hi;
				if (cur.at == -2) {
					vz_lo = 0.f;
					vz_hi = jvz;
				} else {
					vz_lo = -from.tan_slope * s2d_ub;
					vz_hi = from.tan_slope * s2d_ub;
				}
				float cost = 0.f, z_arr = 0.f;
				int n_used = 0;
				const bool ok = EdgeBound(z_dep, vz_lo, vz_hi,
					s2d_ub, dst, p, o.max_edge_ticks, dist, to < 0,
					&cost, &n_used, &z_arr);
				if (o.verbose && cur.faces.size() <= 1)
					printf("  edge %d->%d: dist %.0f E_dep %.0f "
						"z_dep %.0f ride %.0f | dstZ [%.0f,%.0f] | "
						"%s cost %.0f (n %d, z_arr %.0f)\n",
						cur.at, to, dist, E_dep, z_dep, ride_ticks,
						dst.zmin, dst.zmax, ok ? "OK" : "REJECT",
						cost, n_used, z_arr);
				if (!ok)
					continue;
				Node nx;
				nx.faces = cur.faces;
				if (to >= 0)
					nx.faces.push_back(to);
				nx.at = to < 0 ? -1 : to;
				nx.at_pt = arr_pt;
				nx.z_ref = z_arr;
				nx.E = E_dep + 900.f * static_cast<float>(n_used)
					+ 2.f * p.gravity * (z_dep - z_arr);
				if (nx.E < 100.f) nx.E = 100.f;
				nx.lb = cur.lb + cost + ride_ticks;
				open.push_back(nx);
				std::push_heap(open.begin(), open.end());
			}
		}
		*out = done;
		return true;
	}

} // namespace RouteSearch
} // namespace Solver
