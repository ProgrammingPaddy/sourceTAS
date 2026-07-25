#include "WorldDraw.h"
#include "NetVars.h"
#include "Prediction.h"
#include "BspWorld.h"
#include "../Editor/TasEditor.h"

#include <cstdint>
#include <vector>
#include <windows.h>

#include <cstrike/sdk.h>
#include <cstrike/Interfaces/IClientEntityList.h>
#include <cstrike/Interfaces/IVDebugOverlay.h>

namespace WorldDraw {
	bool draw_test_marker   = false;
	bool draw_player_box    = true;
	bool draw_player_marker = true;
	bool draw_prediction    = false;

	int   player_box_alpha   = 30;    // low: mostly wireframe, never reads as solid
	float overlay_life_scale = 1.5f;  // ~one frame of lifetime (see FrameDuration)
	bool  show_replay_hud    = true;
	bool  pause_draw_busy    = true;  // crash isolation: no overlays while solving

	int   pred_ticks       = 66;      // ~1s at 66-tick
	bool  pred_live_input  = true;    // reflect what you're actually pressing
	bool  pred_autobhop    = false;   // jump on ticks that start grounded
	float pred_forwardmove = 450.f;   // override: full forward; movement clamps to maxspeed
	float pred_sidemove    = 0.f;
	bool  pred_jump        = false;
	bool  pred_duck        = false;

	bool corner_trails[4]  = { false, false, false, false };

	bool tag_seg_end_speed = true;
	bool tag_cursor_speed  = true;
	bool tag_seg_end_eff   = true;
	bool tag_seg_end_time  = true;
	bool tag_cursor_time   = true;
}

namespace {
	// Standard CS:S player collision hull. Valve's dimensions, mirrored (not tuned):
	// 32x32 footprint, 72u standing / 54u ducked, feet at the origin. The hull is
	// world-axis-aligned, so orientation is zero.
	const Vector kHullMins(-16.f, -16.f, 0.f);
	const float  kHullWidth   = 16.f;
	const float  kStandHeight = 72.f;
	const float  kDuckHeight  = 54.f;

	// Half-extent of the feet dot (a small cube centred on the origin point).
	const float  kMarkerHalf  = 1.5f;

	// Bottom hull corners as world offsets (the hull is axis-aligned, so these
	// never rotate). Order matches WorldDraw::corner_trails.
	const Vector kCornerOff[4] = {
		Vector( kHullWidth,  kHullWidth, 0.f), Vector( kHullWidth, -kHullWidth, 0.f),
		Vector(-kHullWidth,  kHullWidth, 0.f), Vector(-kHullWidth, -kHullWidth, 0.f),
	};

	WorldDraw::Diagnostics g_diag;

	// ---- line-drawn speed tags -------------------------------------------
	// Seven-segment digits built from the PINNED AddLineOverlay - no new
	// engine surface needed for text. Drawn on a camera-facing vertical
	// plane (billboarded horizontally toward the eye).
	//    segment bits: 0=top 1=top-right 2=bot-right 3=bottom 4=bot-left
	//                  5=top-left 6=middle
	const unsigned char kSevenSeg[10] = {
		0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F };

	void DrawSegDigit(int digit, const Vector& org, const Vector& right,
	                  float h, int r, int g, int b, float dur) {
		if (digit < 0 || digit > 9)
			return;
		const float w = h * 0.55f;
		auto P = [&](float x, float y) {
			return Vector(org.X + right.X * x, org.Y + right.Y * x, org.Z + y);
		};
		const unsigned char m = kSevenSeg[digit];
		if (m & 0x01) debugoverlay->AddLineOverlay(P(0, h),      P(w, h),      r, g, b, false, dur);
		if (m & 0x02) debugoverlay->AddLineOverlay(P(w, h),      P(w, h*0.5f), r, g, b, false, dur);
		if (m & 0x04) debugoverlay->AddLineOverlay(P(w, h*0.5f), P(w, 0),      r, g, b, false, dur);
		if (m & 0x08) debugoverlay->AddLineOverlay(P(0, 0),      P(w, 0),      r, g, b, false, dur);
		if (m & 0x10) debugoverlay->AddLineOverlay(P(0, h*0.5f), P(0, 0),      r, g, b, false, dur);
		if (m & 0x20) debugoverlay->AddLineOverlay(P(0, h),      P(0, h*0.5f), r, g, b, false, dur);
		if (m & 0x40) debugoverlay->AddLineOverlay(P(0, h*0.5f), P(w, h*0.5f), r, g, b, false, dur);
	}

	// Integer speed floating at `at`, readable from `eye`.
	void DrawSpeedTag(float speed, const Vector& at, const Vector& eye,
	                  int r, int g, int b, float dur) {
		int v = static_cast<int>(speed + 0.5f);
		if (v < 0) v = 0;
		if (v > 99999) v = 99999;
		int digits[6];
		int nd = 0;
		do {
			digits[nd++] = v % 10;
			v /= 10;
		} while (v > 0 && nd < 6);

		// Horizontal right vector perpendicular to the eye direction.
		Vector to(eye.X - at.X, eye.Y - at.Y, 0.f);
		const float tl = sqrtf(to.X * to.X + to.Y * to.Y);
		if (tl < 1.f)
			return;
		to.X /= tl;
		to.Y /= tl;
		const Vector right(-to.Y, to.X, 0.f);

		const float h = 7.f;             // digit height (units)
		const float adv = h * 0.55f + 2.5f;
		const float total = adv * static_cast<float>(nd) - 2.5f;
		Vector org(at.X - right.X * total * 0.5f,
		           at.Y - right.Y * total * 0.5f, at.Z);
		for (int i = nd - 1; i >= 0; --i) {
			DrawSegDigit(digits[i], org, right, h, r, g, b, dur);
			org.X += right.X * adv;
			org.Y += right.Y * adv;
		}
	}

	// Fixed-point number with `dec` decimals (0 = integer) - seven-segment
	// digits plus a baseline dot. Times draw in thousandths, percentages in
	// hundredths, speeds as integers.
	void DrawNumTag(float value, int dec, const Vector& at, const Vector& eye,
	                int r, int g, int b, float dur) {
		if (value < 0.f || value != value)   // negatives/NaN are not tag data
			return;
		if (dec < 0) dec = 0;
		if (dec > 3) dec = 3;
		int pow10 = 1;
		for (int i = 0; i < dec; ++i) pow10 *= 10;
		double scaled = static_cast<double>(value) * pow10 + 0.5;
		if (scaled > 99999999.0) scaled = 99999999.0;
		long long v = static_cast<long long>(scaled);
		int fd[3] = {};
		if (dec > 0) {
			long long f = v % pow10;
			for (int i = dec - 1; i >= 0; --i) { fd[i] = static_cast<int>(f % 10); f /= 10; }
			v /= pow10;
		}
		int wd[8];
		int nw = 0;
		do {
			wd[nw++] = static_cast<int>(v % 10);
			v /= 10;
		} while (v > 0 && nw < 8);

		Vector to(eye.X - at.X, eye.Y - at.Y, 0.f);
		const float tl = sqrtf(to.X * to.X + to.Y * to.Y);
		if (tl < 1.f)
			return;
		to.X /= tl;
		to.Y /= tl;
		const Vector right(-to.Y, to.X, 0.f);

		const float h = 7.f;
		const float adv = h * 0.55f + 2.5f;
		const float dotw = 3.f;   // dot cell width
		const float total = adv * static_cast<float>(nw + dec)
			+ (dec > 0 ? dotw : 0.f) - 2.5f;
		Vector org(at.X - right.X * total * 0.5f,
		           at.Y - right.Y * total * 0.5f, at.Z);
		auto advance = [&](float dx) {
			org.X += right.X * dx;
			org.Y += right.Y * dx;
		};
		for (int i = nw - 1; i >= 0; --i) {
			DrawSegDigit(wd[i], org, right, h, r, g, b, dur);
			advance(adv);
		}
		if (dec > 0) {
			// The decimal dot: a short baseline tick.
			debugoverlay->AddLineOverlay(
				Vector(org.X, org.Y, org.Z),
				Vector(org.X + right.X * 1.5f, org.Y + right.Y * 1.5f, org.Z),
				r, g, b, false, dur);
			advance(dotw);
			for (int i = 0; i < dec; ++i) {
				DrawSegDigit(fd[i], org, right, h, r, g, b, dur);
				advance(adv);
			}
		}
	}

	// A predicted point is only drawable if it's finite and inside the world. This
	// keeps a diverged prediction from feeding NaN/huge coords to the overlay
	// renderer (which isn't fault-guarded).
	bool FiniteWorldPoint(const Vector& v) {
		const float kMax = 16384.f;   // Source map coordinate limit
		return v.X == v.X && v.Y == v.Y && v.Z == v.Z &&   // reject NaN
		       v.X > -kMax && v.X < kMax &&
		       v.Y > -kMax && v.Y < kMax &&
		       v.Z > -kMax && v.Z < kMax;
	}

	// PLAYER-ATTACHED overlays (hull, origin dot, live prediction) use the
	// RAW per-frame interval: they track a moving entity, and any lifetime
	// stretching leaves visible trails on it ("ghosting on the active player
	// hitbox"). Refreshed by FrameDuration each frame.
	float g_dur_live = 0.03f;

	// Adaptive overlay lifetime. A fixed duration leaves a moving "ghost trail":
	// stale copies from previous positions stay alive while the player moves, and
	// stacked translucent copies read as a solid box. A static overlay hides this
	// because its copies overlap exactly - which is why the world-origin marker
	// looked clean. Measuring the render interval and keeping each overlay alive
	// ~one frame means exactly one instance renders per frame at any framerate:
	// crisp tracking, no ghosting, no flicker.
	float FrameDuration() {
		static LARGE_INTEGER freq = {};
		static LARGE_INTEGER last = {};
		if (freq.QuadPart == 0)
			QueryPerformanceFrequency(&freq);

		LARGE_INTEGER now;
		QueryPerformanceCounter(&now);

		float dt = 0.f;
		if (last.QuadPart != 0 && freq.QuadPart != 0)
			dt = static_cast<float>(now.QuadPart - last.QuadPart) / static_cast<float>(freq.QuadPart);
		last = now;

		float scale = WorldDraw::overlay_life_scale;
		if (scale < 0.5f) scale = 0.5f;
		if (scale > 6.0f) scale = 6.0f;

		// Player-attached lifetime: raw interval only.
		g_dur_live = (dt > 0.f) ? dt * scale : 0.03f;
		if (g_dur_live < 0.01f) g_dur_live = 0.01f;
		if (g_dur_live > 0.20f) g_dur_live = 0.20f;

		// Static-drawing lifetime: the MAX of the last few intervals, so a
		// spiky frame (solver work slices, sims landing unevenly) doesn't
		// strand the editor line - flicker-resistant without trailing, since
		// those drawings don't move between frames.
		static float hist[8] = {};
		static int hi = 0;
		hist[hi] = dt;
		hi = (hi + 1) & 7;
		float base = dt;
		for (int i = 0; i < 8; ++i)
			if (hist[i] > base) base = hist[i];

		float duration = (base > 0.f) ? base * scale : 0.03f;
		if (duration < 0.01f) duration = 0.01f;   // survive to the next frame's draw
		if (duration > 0.30f) duration = 0.30f;   // cap ghosting if frames hitch hard
		return duration;
	}
}

void WorldDraw::Render() {
	Diagnostics d;

	d.interfaces_ready = (engine && debugoverlay && entitylist && clientdll);
	if (!d.interfaces_ready) { g_diag = d; return; }

	d.in_game = engine->IsInGame();
	if (!d.in_game) { g_diag = d; return; }

	// Crash isolation: while the solver's HEAVY phases churn (search /
	// verify / batch), submit NOTHING. The engine expires its overlay list
	// on the game thread while we add from the render hook; if the silent
	// heap-corruption crashes stop with this pause active, that race is
	// confirmed (then the real fix is game-thread submission). Brief
	// corrections don't pause - a stalled one used to blank the world.
	if (pause_draw_busy && TasEditor::SearchHeavy()) {
		g_diag = d;
		return;
	}

	const float   duration = FrameDuration();
	const QAngle  kNoRotation(0.f, 0.f, 0.f);

	// (1) Fixed world-origin marker: validates the overlay dispatch. Static, so it
	// never ghosts regardless of lifetime.
	if (draw_test_marker) {
		const Vector at(0.f, 0.f, 0.f);
		debugoverlay->AddBoxOverlay(at, Vector(-8.f, -8.f, 0.f), Vector(8.f, 8.f, 64.f),
		                            kNoRotation, 0, 255, 0, 48, duration);
		debugoverlay->AddLineOverlay(at, at + Vector(0.f, 0.f, 128.f), 0, 255, 0, false, duration);
	}

	// (2) Local player: collision hull + a single dot at the feet origin.
	d.local_index   = engine->GetLocalPlayer();
	void* player    = entitylist->GetClientEntity(d.local_index);
	d.origin_offset = NetVars::Offset("DT_CSPlayer", "m_vecOrigin");
	d.flags_offset  = NetVars::Offset("DT_CSPlayer", "m_fFlags");

	if (player && d.origin_offset) {
		d.have_player = true;
		d.origin  = NetVars::Get<Vector>(player, d.origin_offset);
		d.flags   = d.flags_offset ? NetVars::Get<int>(player, d.flags_offset) : 0;
		d.ducking = (d.flags & FL_DUCKING) != 0;

		const float height = d.ducking ? kDuckHeight : kStandHeight;

		// Player-attached draws track a MOVING entity: raw per-frame lifetime,
		// or every stretched frame leaves a trail of stale copies behind it.
		if (draw_player_box) {
			int a = player_box_alpha;
			if (a < 0)   a = 0;
			if (a > 255) a = 255;
			const Vector maxs(kHullWidth, kHullWidth, height);
			debugoverlay->AddBoxOverlay(d.origin, kHullMins, maxs, kNoRotation,
			                            255, 64, 64, a, g_dur_live);
		}

		// A single dot at the feet origin. Per tick this is the path vertex; when a
		// locked-in run is drawn later, these dots connect into the movement line.
		if (draw_player_marker) {
			const Vector dmin(-kMarkerHalf, -kMarkerHalf, -kMarkerHalf);
			const Vector dmax( kMarkerHalf,  kMarkerHalf,  kMarkerHalf);
			debugoverlay->AddBoxOverlay(d.origin, dmin, dmax, kNoRotation,
			                            255, 255, 0, 255, g_dur_live);
		}

		// (3) Predicted path (Phase 1b): the look-ahead computed once per tick inside
		// the engine's prediction pass (see Prediction.cpp); we just draw the cached
		// per-tick feet dots as a green line starting at the live origin.
		if (draw_prediction) {
			std::vector<Vector> path;
			Prediction::GetPath(path);
			const Vector dmin(-1.f, -1.f, -1.f);
			const Vector dmax( 1.f,  1.f,  1.f);
			Vector prev = d.origin;
			for (const Vector& p : path) {
				if (!FiniteWorldPoint(p))
					break;   // prediction diverged; stop before feeding garbage to the overlay
				debugoverlay->AddLineOverlay(prev, p, 40, 220, 90, false, g_dur_live);
				debugoverlay->AddBoxOverlay(p, dmin, dmax, kNoRotation, 40, 255, 90, 255, g_dur_live);
				for (int c = 0; c < 4; ++c)
					if (corner_trails[c])
						debugoverlay->AddLineOverlay(prev + kCornerOff[c], p + kCornerOff[c],
						                             120, 170, 200, false, g_dur_live);
				prev = p;
			}
		}
	}

	// (4) TAS editor run line (Phase 2): the simulated run drawn as an absolute
	// polyline. The playhead is the edit boundary: ticks before it draw gray
	// (locked history); the selected segment is colored by strafe efficiency
	// (green = at/above the min-eff threshold, orange = below); other future
	// segments blue; boundary markers white; ghost hull at the cursor tick.
	// Decimated so long runs stay within the overlay system's budget.
	TasEditor::DrawData ed;
	if (TasEditor::GetDrawData(ed)) {
		if (ed.anchor.valid && FiniteWorldPoint(ed.anchor.origin))
			debugoverlay->AddBoxOverlay(ed.anchor.origin, Vector(-3.f, -3.f, 0.f), Vector(3.f, 3.f, 8.f),
			                            kNoRotation, 60, 255, 60, 140, duration);

		const int stride = ed.count > 400 ? ed.count / 400 : 1;
		Vector prev = ed.anchor.origin;
		bool prev_ok = FiniteWorldPoint(prev);
		int next_seg = 0;
		for (int i = 0; i < ed.count; ++i) {
			bool boundary = false;
			while (next_seg < ed.seg_count && i == ed.seg_starts[next_seg]) {
				boundary = true;
				next_seg++;
			}
			const bool pass_edge = ed.have_pass && i == ed.pass_tick;
			if (!boundary && !pass_edge && (i % stride) != 0 && i != ed.count - 1 && i != ed.cursor)
				continue;

			const Vector p = ed.states[i].origin;
			if (!FiniteWorldPoint(p))
				break;

			// Splice the measured pass point in as a vertex (and never decimate
			// its tick): the drawn line visibly runs through the exact point
			// where the real path crosses the target's height.
			if (pass_edge && prev_ok && FiniteWorldPoint(ed.pass_point)) {
				debugoverlay->AddLineOverlay(prev, ed.pass_point, 255, 235, 60, false, duration);
				for (int c = 0; c < 4; ++c)
					if (corner_trails[c])
						debugoverlay->AddLineOverlay(prev + kCornerOff[c], ed.pass_point + kCornerOff[c],
						                             120, 170, 200, false, duration);
				debugoverlay->AddBoxOverlay(ed.pass_point, Vector(-1.2f, -1.2f, -1.2f),
				                            Vector(1.2f, 1.2f, 1.2f), kNoRotation,
				                            255, 235, 60, 255, duration);
				prev = ed.pass_point;
			}

			// The whole future path is colored by strafe efficiency (the line is
			// always the exact simulated path; color is information, not a
			// constraint). The selected segment draws at full brightness, the
			// rest dimmed; ticks before the playhead are locked history.
			int r, g, b;
			if (i < ed.cursor) {
				r = 120; g = 120; b = 120;
			} else {
				if (ed.eff && ed.maxgain && ed.maxgain[i] > 0.001f) {
					if (ed.eff[i] >= ed.min_eff) { r = 60; g = 255; b = 120; }   // optimal enough
					else                          { r = 255; g = 140; b = 30; }  // below threshold
				} else {
					r = 50; g = 190; b = 110;                    // not scoreable (ground/landing)
				}
				const bool selected = (i >= ed.sel_start && i < ed.sel_end);
				if (!selected) { r = (r * 11) / 20; g = (g * 11) / 20; b = (b * 11) / 20; }
			}
			if (prev_ok) {
				debugoverlay->AddLineOverlay(prev, p, r, g, b, false, duration);
				for (int c = 0; c < 4; ++c)
					if (corner_trails[c])
						debugoverlay->AddLineOverlay(prev + kCornerOff[c], p + kCornerOff[c],
						                             120, 170, 200, false, duration);
			}
			prev = p;
			prev_ok = true;

			if (boundary)
				debugoverlay->AddBoxOverlay(p, Vector(-1.5f, -1.5f, -1.5f), Vector(1.5f, 1.5f, 1.5f),
				                            kNoRotation, 255, 255, 255, 255, duration);
		}

		if (ed.cursor >= 0 && ed.cursor < ed.count) {
			// The hull for tick t is the state ENTERING t: states[t-1], and the
			// ANCHOR for t=0. Drawing states[t] put the "start hitbox" one tick
			// of movement AHEAD of the anchor - the teleport measured EXACT
			// (delta 0.000) while the marker sat ~1u forward: the marker lied.
			const Vector cp = (ed.cursor == 0)
				? ed.anchor.origin
				: ed.states[ed.cursor - 1].origin;
			const unsigned cflags = (ed.cursor == 0)
				? (ed.anchor.ducked ? FL_DUCKING : 0)
				: ed.states[ed.cursor - 1].flags;
			if (FiniteWorldPoint(cp)) {
				// Origin marker always draws at the playhead; the hull is a toggle.
				debugoverlay->AddBoxOverlay(cp, Vector(-1.5f, -1.5f, -1.5f), Vector(1.5f, 1.5f, 1.5f),
				                            kNoRotation, 255, 255, 0, 255, duration);
				if (ed.show_hull) {
					const float h = (cflags & FL_DUCKING) ? 54.f : 72.f;
					debugoverlay->AddBoxOverlay(cp, Vector(-16.f, -16.f, 0.f), Vector(16.f, 16.f, h),
					                            kNoRotation, 255, 255, 0, 40, duration);
				}

				// View-direction arrow out of the origin (for lining up ramps).
				if (ed.have_view) {
					const float deg2rad = 3.14159265f / 180.f;
					const float pr = ed.view_pitch * deg2rad;   // Source pitch: + = down
					const float yr = ed.view_yaw * deg2rad;
					const Vector dir(cosf(pr) * cosf(yr), cosf(pr) * sinf(yr), -sinf(pr));
					const float len = 48.f;
					const Vector tip(cp.X + dir.X * len, cp.Y + dir.Y * len, cp.Z + dir.Z * len);
					debugoverlay->AddLineOverlay(cp, tip, 0, 220, 255, false, duration);

					// Arrowhead barbs, perpendicular in the horizontal plane.
					const Vector perp(-sinf(yr), cosf(yr), 0.f);
					const Vector base(tip.X - dir.X * 10.f, tip.Y - dir.Y * 10.f, tip.Z - dir.Z * 10.f);
					debugoverlay->AddLineOverlay(tip,
						Vector(base.X + perp.X * 6.f, base.Y + perp.Y * 6.f, base.Z),
						0, 220, 255, false, duration);
					debugoverlay->AddLineOverlay(tip,
						Vector(base.X - perp.X * 6.f, base.Y - perp.Y * 6.f, base.Z),
						0, 220, 255, false, duration);
				}
			}
		}

		// Tag stacks (line-drawn digits). Segment end, bottom-up: time (white,
		// +56), % of optimal gain (green, +70), 2D speed (yellow, +84). The
		// end tick is the PLAN end - for a solver segment that IS the pass
		// tick, the same state a solution row's `speed` claims (never the
		// post-pass slide/overrun). Playhead: speed (cyan, +96), time (white,
		// +110).
		const Vector eye(d.origin.X, d.origin.Y, d.origin.Z + 64.f);
		if (ed.sel_end > 0 && ed.sel_end <= ed.count) {
			const int ei = ed.sel_end - 1;
			const Vector& ev = ed.states[ei].velocity;
			const Vector eb = ed.states[ei].origin;
			if (tag_seg_end_speed) {
				const Vector ep(eb.X, eb.Y, eb.Z + 84.f);
				if (FiniteWorldPoint(ep))
					DrawSpeedTag(sqrtf(ev.X * ev.X + ev.Y * ev.Y), ep, eye,
						255, 220, 60, duration);
			}
			if (tag_seg_end_eff && ed.seg_gain_pct >= 0.f) {
				const Vector gp(eb.X, eb.Y, eb.Z + 70.f);
				if (FiniteWorldPoint(gp))
					DrawNumTag(ed.seg_gain_pct, 2, gp, eye, 90, 235, 120, duration);
			}
			if (tag_seg_end_time && ed.seg_time >= 0.f) {
				const Vector tp(eb.X, eb.Y, eb.Z + 56.f);
				if (FiniteWorldPoint(tp))
					DrawNumTag(ed.seg_time, 3, tp, eye, 235, 235, 235, duration);
			}
		}
		if (ed.cursor >= 0 && ed.cursor < ed.count) {
			const Vector& cv = (ed.cursor == 0)
				? ed.anchor.velocity : ed.states[ed.cursor - 1].velocity;
			const Vector cb = (ed.cursor == 0)
				? ed.anchor.origin : ed.states[ed.cursor - 1].origin;
			if (tag_cursor_speed) {
				const Vector cp2(cb.X, cb.Y, cb.Z + 96.f);
				if (FiniteWorldPoint(cp2))
					DrawSpeedTag(sqrtf(cv.X * cv.X + cv.Y * cv.Y), cp2, eye,
						90, 220, 255, duration);
			}
			if (tag_cursor_time && ed.cursor_time >= 0.f) {
				const Vector tp(cb.X, cb.Y, cb.Z + 110.f);
				if (FiniteWorldPoint(tp))
					DrawNumTag(ed.cursor_time, 3, tp, eye, 235, 235, 235, duration);
			}
		}
	}

	// (5) World geometry (1.1): brush wireframes from the map's BSP collision
	// data, centered on the player. Self-guards and auto-loads on level change.
	BspWorld::Render(d.origin, d.have_player, duration);

	g_diag = d;
}

WorldDraw::Diagnostics WorldDraw::LastDiagnostics() {
	return g_diag;
}
