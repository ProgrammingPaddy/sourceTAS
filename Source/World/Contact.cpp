#include "Contact.h"
#include "BspWorld.h"

#include <cmath>
#include <cstrike/Definitions/Const.h>

namespace {
	float Dot(const Vector& a, const Vector& b) {
		return a.X * b.X + a.Y * b.Y + a.Z * b.Z;
	}
	float Len(const Vector& v) { return sqrtf(Dot(v, v)); }
	Vector Sub(const Vector& a, const Vector& b) {
		return Vector(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
	}
	Vector Scale(const Vector& v, float s) {
		return Vector(v.X * s, v.Y * s, v.Z * s);
	}

	// The hull corner farthest INTO a face of normal n (minimizes n·corner):
	// the point of the box that touches the plane first.
	Vector SupportCorner(const Vector& n, const Vector& mins, const Vector& maxs) {
		return Vector(n.X > 0.f ? mins.X : maxs.X,
		              n.Y > 0.f ? mins.Y : maxs.Y,
		              n.Z > 0.f ? mins.Z : maxs.Z);
	}

	// Match an estimated contact (position + normal direction) against the
	// parsed world brushes: a face whose plane the hull's support corner sits
	// on (within tolerance), whose normal agrees, and whose polygon contains
	// the touch. Events are rare, so a full brush sweep with an AABB
	// prefilter is fine.
	bool MatchFace(const Vector& pos, const Vector& n_est,
	               const Vector& mins, const Vector& maxs,
	               int* out_brush, int* out_plane, Vector* out_n) {
		const int nb = BspWorld::BrushCount();
		const float kBoxSlack = 96.f;   // AABB prefilter reach around pos
		const float kPlaneTol = 8.f;    // support corner's distance to the plane
		const float kNormalDot = 0.80f; // residual-estimated normals are noisy
		float best = 1e9f;
		for (int b = 0; b < nb; ++b) {
			int contents = 0;
			Vector bmin, bmax;
			if (!BspWorld::GetBrushInfo(b, &contents, &bmin, &bmax))
				continue;
			if (!BspWorld::IsWorldBrush(b))
				continue;
			if (pos.X < bmin.X - kBoxSlack || pos.X > bmax.X + kBoxSlack ||
			    pos.Y < bmin.Y - kBoxSlack || pos.Y > bmax.Y + kBoxSlack ||
			    pos.Z < bmin.Z - kBoxSlack || pos.Z > bmax.Z + kBoxSlack)
				continue;
			const int np = BspWorld::BrushClipPlaneCount(b);
			for (int p = 0; p < np; ++p) {
				int plane_id = -1;
				Vector n;
				float d = 0.f;
				if (!BspWorld::GetBrushClipPlane(b, p, &plane_id, &n, &d))
					continue;
				if (Dot(n, n_est) < kNormalDot)
					continue;
				const Vector corner = SupportCorner(n, mins, maxs);
				const Vector touch(pos.X + corner.X, pos.Y + corner.Y, pos.Z + corner.Z);
				const float dist = fabsf(Dot(n, touch) - d);
				if (dist > kPlaneTol)
					continue;
				if (!BspWorld::PointOnFace(b, plane_id, touch, 12.f))
					continue;
				if (dist < best) {
					best = dist;
					*out_brush = b;
					*out_plane = plane_id;
					*out_n = n;
				}
			}
		}
		return best < 1e8f;
	}
}

int Contact::Analyze(const Prediction::SimState* states, int count,
                     float interval, float gravity, float max_add,
                     BoardEvent* out, int max_out, int min_air) {
	if (!states || count < 2 || !out || max_out < 1)
		return 0;
	if (interval <= 0.f) interval = 0.015f;
	if (gravity <= 0.f)  gravity = 800.f;
	if (max_add <= 0.f)  max_add = 70.f;

	// Player hull (engine-measured standing box when available).
	Vector hull_min(-16.f, -16.f, 0.f), hull_max(16.f, 16.f, 72.f);
	Prediction::PlayerHull(&hull_min, &hull_max);

	// Tagged faces snapshot for the exact crossing detector.
	struct TagPlane { int brush, plane; Vector n; float d; };
	TagPlane tags[128];
	int ntags = 0;
	for (int i = 0; i < BspWorld::TagCount() && ntags < 128; ++i) {
		TagPlane t;
		if (!BspWorld::GetTag(i, &t.brush, &t.plane))
			continue;
		if (!BspWorld::GetPlane(t.plane, &t.n, &t.d))
			continue;
		tags[ntags++] = t;
	}

	// Anything free flight can't explain beyond this is a plane clip. The
	// margin absorbs trigger-gravity scale differences and float slop.
	const float gdt = gravity * interval;
	const float kResidualThresh = max_add + gdt + 20.f;

	int nout = 0;
	int air_run = 0;   // consecutive contact-free airborne ticks so far
	for (int i = 1; i < count && nout < max_out; ++i) {
		const Prediction::SimState& s0 = states[i - 1];
		const Prediction::SimState& s1 = states[i];
		const bool ground0 = (s0.flags & FL_ONGROUND) != 0;
		const bool ground1 = (s1.flags & FL_ONGROUND) != 0;

		Vector hmax = hull_max;
		if (s0.flags & FL_DUCKING)
			hmax.Z = 54.f;

		// Velocity entering the impact: pre-tick velocity plus this tick's
		// gravity - what the plane actually clips.
		const Vector v_pre(s0.velocity.X, s0.velocity.Y, s0.velocity.Z - gdt);

		// (1) Exact: hull support corner crossing a tagged face this tick.
		bool  hit = false, hit_face_known = false, hit_ground = false;
		int   hit_brush = -1, hit_plane = -1;
		Vector hit_n(0.f, 0.f, 1.f);
		Vector hit_pos = s1.origin;
		for (int t = 0; t < ntags && !hit; ++t) {
			const Vector corner = SupportCorner(tags[t].n, hull_min, hmax);
			const float d_origin = tags[t].d - Dot(tags[t].n, corner);
			const float prev_phi = Dot(tags[t].n, s0.origin) - d_origin;
			const float phi = Dot(tags[t].n, s1.origin) - d_origin;
			if (prev_phi < 1.f || phi >= 1.f)
				continue;
			// Interpolate the touch along the tick's velocity step (the same
			// recipe as the solver's real-pass measurement).
			const Vector step = Scale(s0.velocity, interval);
			const Vector stepped(s0.origin.X + step.X, s0.origin.Y + step.Y,
			                     s0.origin.Z + step.Z);
			const float pe = Dot(tags[t].n, stepped) - d_origin;
			float tau = 1.f;
			if (prev_phi - pe > 1e-6f)
				tau = prev_phi / (prev_phi - pe);
			if (tau < 0.f) tau = 0.f;
			if (tau > 1.f) tau = 1.f;
			const Vector touch(s0.origin.X + step.X * tau,
			                   s0.origin.Y + step.Y * tau,
			                   s0.origin.Z + step.Z * tau);
			const Vector corner_touch(touch.X + corner.X, touch.Y + corner.Y,
			                          touch.Z + corner.Z);
			if (!BspWorld::PointOnFace(tags[t].brush, tags[t].plane, corner_touch, 6.f))
				continue;
			hit = true;
			hit_face_known = true;
			hit_brush = tags[t].brush;
			hit_plane = tags[t].plane;
			hit_n = tags[t].n;
			hit_pos = touch;
		}

		// (2) Ground onset after air.
		if (!hit && ground1 && !ground0) {
			hit = true;
			hit_ground = true;
			hit_n = Vector(0.f, 0.f, 1.f);
			hit_face_known = MatchFace(s1.origin, hit_n, hull_min, hmax,
				&hit_brush, &hit_plane, &hit_n);
		}

		// (3) General: velocity residual beyond free flight -> a clip
		// happened; match the estimated normal against world faces.
		bool residual_contact = false;
		if (!hit && !ground0 && !ground1) {
			Vector r(s1.velocity.X - v_pre.X,
			         s1.velocity.Y - v_pre.Y,
			         s1.velocity.Z - v_pre.Z);
			const float rl = Len(r);
			if (rl > kResidualThresh) {
				residual_contact = true;
				const Vector n_est = Scale(r, 1.f / rl);
				hit = true;
				hit_n = n_est;
				hit_face_known = MatchFace(s1.origin, n_est, hull_min, hmax,
					&hit_brush, &hit_plane, &hit_n);
			}
		}

		if (hit && air_run >= min_air) {
			BoardEvent& e = out[nout++];
			e.tick = i;
			e.pos = hit_pos;
			e.normal = hit_n;
			e.brush = hit_brush;
			e.plane = hit_plane;
			e.face_known = hit_face_known;
			e.grounded = hit_ground;
			e.arrive_speed = Len(v_pre);
			e.arrive_speed2d = sqrtf(v_pre.X * v_pre.X + v_pre.Y * v_pre.Y);
			// Speed the clip removed: the same |v| - |v - n(v.n)| the solver's
			// pass measurement reports; face-unknown falls back to the raw
			// before/after speed drop (floored at 0 - boards often GAIN).
			if (hit_face_known || hit_ground) {
				const float into = Dot(hit_n, v_pre);
				if (into < 0.f) {
					const Vector vc = Sub(v_pre, Scale(hit_n, into));
					e.clip_loss = Len(v_pre) - Len(vc);
				}
			} else {
				const float drop = Len(v_pre) - Len(s1.velocity);
				e.clip_loss = drop > 0.f ? drop : 0.f;
			}
		}

		// State bookkeeping: a tick counts toward the airborne run only when
		// nothing touched it (a ramp slide clips EVERY tick and stays at 0).
		const bool touched = hit || ground1 || residual_contact;
		air_run = touched ? 0 : air_run + 1;
	}
	return nout;
}
