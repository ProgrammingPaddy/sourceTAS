#pragma once

#include "../Structures/Vector.h"
#include "../Utilities/Virtuals.h"

// Engine debug-overlay interface (engine.dll, "VDebugOverlay003"). This is how
// we draw depth-correct geometry into the game's 3D scene without touching the
// renderer or any Valve draw code - we only queue overlays the engine already
// knows how to render.
//
// The classic Source SDK method order does NOT match this build, so we dispatch
// by vtable index pinned from static RE of CIVDebugOverlay (vtable rva 0x38bd60,
// 20 methods). Each Add* method writes its OverlayType_t into the node it
// allocates; that constant plus the node size identified the geometry methods:
//
//     index 1 = AddBoxOverlay    (node type OVERLAY_BOX = 0, 88-byte node)
//     index 3 = AddLineOverlay    (7-arg no-alpha form; thunk inserts alpha=255)
//     index 2 = line, alpha form  (direct impl, type OVERLAY_LINE = 3)
//
// The indices - and the line calling form - are adjustable globals so the single
// remaining in-game check is a bounded one-click tweak rather than a rebuild.
extern int  g_overlay_box_index;   // default 1
extern int  g_overlay_line_index;  // default 3
extern bool g_overlay_line_alpha;  // default false: call the 7-arg no-alpha form

// OVERLAY MARSHAL (session 46e). The 2026-08-25 CS:S update made
// submitting debug overlays from the RENDER thread (EndScene, where
// WorldDraw runs) hang the game - confirmed by the master-gate bisect.
// AddBoxOverlay/AddLineOverlay now ENQUEUE onto a thread-safe buffer;
// OverlayQueue::Drain(), called from the CreateMove hook (the game /
// overlay-expiry thread), makes the REAL engine vtable calls there.
// Every debugoverlay->Add* caller across WorldDraw + BspWorld is
// marshaled by this single chokepoint. Defined in WorldDraw.cpp.
namespace OverlayQueue {
	void PushBox(const Vector& origin, const Vector& mins, const Vector& maxs,
	             const QAngle& orientation, int r, int g, int b, int a, float duration);
	void PushLine(const Vector& origin, const Vector& dest,
	              int r, int g, int b, bool noDepthTest, float duration, bool alpha);
	// Game thread only: dispatch queued overlays to the engine vtable.
	void Drain();
	// Diagnostics: lifetime pushed/drained totals, last drain size, whether
	// the engine overlay interface resolved, the engine-call fault count,
	// and the last fault's engine.dll RVA.
	void Stats(long* pushed, long* drained, long* last_drain, bool* overlay_ready,
	           long* faults, unsigned long long* fault_rva);
}

class IVDebugOverlay {
public:
	// Axis-aligned when 'orientation' is zero. The box spans
	// [origin + mins, origin + maxs] and is rotated about 'origin'.
	void AddBoxOverlay(const Vector& origin, const Vector& mins, const Vector& maxs,
	                   const QAngle& orientation, int r, int g, int b, int a, float duration) {
		OverlayQueue::PushBox(origin, mins, maxs, orientation, r, g, b, a, duration);
	}

	// A world-space line from 'origin' to 'dest'. Two forms exist across builds;
	// g_overlay_line_alpha picks the 8-arg (explicit alpha) variant when needed.
	void AddLineOverlay(const Vector& origin, const Vector& dest,
	                    int r, int g, int b, bool noDepthTest, float duration) {
		OverlayQueue::PushLine(origin, dest, r, g, b, noDepthTest, duration, g_overlay_line_alpha);
	}
};

extern IVDebugOverlay* debugoverlay;
