#include "WorldDraw.h"
#include "NetVars.h"
#include "Prediction.h"

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

	int   pred_ticks       = 66;      // ~1s at 66-tick
	bool  pred_live_input  = true;    // reflect what you're actually pressing
	float pred_forwardmove = 450.f;   // override: full forward; movement clamps to maxspeed
	float pred_sidemove    = 0.f;
	bool  pred_jump        = false;
	bool  pred_duck        = false;
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

	WorldDraw::Diagnostics g_diag;

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

		float duration = (dt > 0.f) ? dt * scale : 0.03f;
		if (duration < 0.01f) duration = 0.01f;   // survive to the next frame's draw
		if (duration > 0.20f) duration = 0.20f;   // cap ghosting if a frame hitches
		return duration;
	}
}

void WorldDraw::Render() {
	Diagnostics d;

	d.interfaces_ready = (engine && debugoverlay && entitylist && clientdll);
	if (!d.interfaces_ready) { g_diag = d; return; }

	d.in_game = engine->IsInGame();
	if (!d.in_game) { g_diag = d; return; }

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

		if (draw_player_box) {
			int a = player_box_alpha;
			if (a < 0)   a = 0;
			if (a > 255) a = 255;
			const Vector maxs(kHullWidth, kHullWidth, height);
			debugoverlay->AddBoxOverlay(d.origin, kHullMins, maxs, kNoRotation,
			                            255, 64, 64, a, duration);
		}

		// A single dot at the feet origin. Per tick this is the path vertex; when a
		// locked-in run is drawn later, these dots connect into the movement line.
		if (draw_player_marker) {
			const Vector dmin(-kMarkerHalf, -kMarkerHalf, -kMarkerHalf);
			const Vector dmax( kMarkerHalf,  kMarkerHalf,  kMarkerHalf);
			debugoverlay->AddBoxOverlay(d.origin, dmin, dmax, kNoRotation,
			                            255, 255, 0, 255, duration);
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
				debugoverlay->AddLineOverlay(prev, p, 40, 220, 90, false, duration);
				debugoverlay->AddBoxOverlay(p, dmin, dmax, kNoRotation, 40, 255, 90, 255, duration);
				prev = p;
			}
		}
	}

	g_diag = d;
}

WorldDraw::Diagnostics WorldDraw::LastDiagnostics() {
	return g_diag;
}
