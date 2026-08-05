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

class IVDebugOverlay {
public:
	// Axis-aligned when 'orientation' is zero. The box spans
	// [origin + mins, origin + maxs] and is rotated about 'origin'.
	void AddBoxOverlay(const Vector& origin, const Vector& mins, const Vector& maxs,
	                   const QAngle& orientation, int r, int g, int b, int a, float duration) {
		GetVirtualFunction<void(*)(IVDebugOverlay*, const Vector&, const Vector&, const Vector&,
		                           const QAngle&, int, int, int, int, float)>(this, g_overlay_box_index)
			(this, origin, mins, maxs, orientation, r, g, b, a, duration);
	}

	// A world-space line from 'origin' to 'dest'. Two forms exist across builds;
	// g_overlay_line_alpha picks the 8-arg (explicit alpha) variant when needed.
	void AddLineOverlay(const Vector& origin, const Vector& dest,
	                    int r, int g, int b, bool noDepthTest, float duration) {
		if (g_overlay_line_alpha) {
			GetVirtualFunction<void(*)(IVDebugOverlay*, const Vector&, const Vector&,
			                           int, int, int, int, bool, float)>(this, g_overlay_line_index)
				(this, origin, dest, r, g, b, 255, noDepthTest, duration);
		} else {
			GetVirtualFunction<void(*)(IVDebugOverlay*, const Vector&, const Vector&,
			                           int, int, int, bool, float)>(this, g_overlay_line_index)
				(this, origin, dest, r, g, b, noDepthTest, duration);
		}
	}
};

extern IVDebugOverlay* debugoverlay;
