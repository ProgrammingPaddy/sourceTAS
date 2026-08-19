#pragma once

#include "../Structures/Vector.h"
#include "../Utilities/Virtuals.h"

// Engine trace interface (engine.dll, "EngineTraceClient004" with 003 as the
// fallback). THE TRACE ORACLE: instead of fitting collision knife-edge rules
// from playback captures one divergence at a time (corner release, plane
// settle, ...), the Map Solve tab batch-answers the solver's own trace
// queries with the ENGINE'S collision code and the lab diffs our TraceHull
// against those answers trace by trace. The engine is the arbiter; nothing
// is inferred.
//
// ABI notes (this is an x64 2013-lineage build):
//  - TraceRay sits at vtable index 5 in the 2013 IEngineTrace layout
//    (0 GetPointContents, 1 GetPointContents_WorldOnly,
//     2 GetPointContents_Collideable, 3 ClipRayToEntity,
//     4 ClipRayToCollideable, 5 TraceRay). The 2007 layout lacks #1, so
//    TraceRay lands at 4 there. Adjustable global + a validation battery of
//    known-answer traces (run before ANY batch) pin it at runtime, and the
//    probe call itself is SEH-guarded so a wrong index reports instead of
//    crashing the game - the IVDebugOverlay probe-then-pin discipline.
//  - Ray_t: 2013 carries m_pWorldAxisTransform between m_Extents and the
//    bools; 2007 does not. g_ray_has_transform picks the layout (default
//    true), also validated by the battery.
//  - trace_t results are read at fixed CBaseTrace offsets (endpos 0x0C,
//    plane normal 0x18, plane dist 0x24, fraction 0x2C, allsolid 0x36,
//    startsolid 0x37); the out-buffer is oversized so the engine writing
//    CGameTrace fields past those stays in bounds.

extern int  g_trace_ray_index;     // default 5 (2013); 4 = 2007 fallback
extern bool g_ray_has_transform;   // default true (2013 Ray_t)

namespace EngineTraceABI {

	// Player-movement solid mask (MASK_PLAYERSOLID): SOLID | MOVEABLE |
	// WINDOW | MONSTER | GRATE | PLAYERCLIP.
	constexpr unsigned kMaskPlayerSolid =
		0x1u | 0x4000u | 0x2u | 0x2000000u | 0x8u | 0x10000u;

	constexpr int kTraceWorldOnly = 1;   // TraceType_t::TRACE_WORLD_ONLY

	// 16-byte-aligned vector as the engine's VectorAligned.
	struct alignas(16) VecAligned {
		float x = 0.f, y = 0.f, z = 0.f, w = 0.f;
	};

	// Superset Ray_t buffer (2013 layout). For the 2007 layout the bools sit
	// where the transform pointer starts; BuildRay writes whichever layout is
	// active into the same storage.
	struct alignas(16) RayBuf {
		VecAligned start;         // 0x00
		VecAligned delta;         // 0x10
		VecAligned start_offset;  // 0x20
		VecAligned extents;       // 0x30
		unsigned char tail[16];   // transform ptr + bools (layout-dependent)
	};

	inline void BuildRay(RayBuf& r, const Vector& a, const Vector& b,
	                     const Vector& mins, const Vector& maxs) {
		r = RayBuf();
		r.delta = { b.X - a.X, b.Y - a.Y, b.Z - a.Z, 0.f };
		const float cx = 0.5f * (mins.X + maxs.X);
		const float cy = 0.5f * (mins.Y + maxs.Y);
		const float cz = 0.5f * (mins.Z + maxs.Z);
		r.extents = { 0.5f * (maxs.X - mins.X), 0.5f * (maxs.Y - mins.Y),
		              0.5f * (maxs.Z - mins.Z), 0.f };
		// SDK Ray_t::Init: m_Start = start + center, m_StartOffset = -center.
		r.start = { a.X + cx, a.Y + cy, a.Z + cz, 0.f };
		r.start_offset = { -cx, -cy, -cz, 0.f };
		const bool swept = (r.delta.x != 0.f || r.delta.y != 0.f
			|| r.delta.z != 0.f);
		for (int i = 0; i < 16; ++i) r.tail[i] = 0;
		if (g_ray_has_transform) {
			r.tail[8] = 0;             // m_IsRay = false (box)
			r.tail[9] = swept ? 1 : 0; // m_IsSwept
		} else {
			r.tail[0] = 0;
			r.tail[1] = swept ? 1 : 0;
		}
	}

	// World-only trace filter: two-entry vtable {ShouldHitEntity,
	// GetTraceType}. TRACE_WORLD_ONLY makes the engine skip entities
	// entirely, which is exactly the solver's world model (worldspawn
	// brushes only).
	class WorldOnlyFilter {
	public:
		virtual bool ShouldHitEntity(void* /*entity*/, int /*mask*/) {
			return false;
		}
		virtual int GetTraceType() {
			return kTraceWorldOnly;
		}
	};

	// Oversized trace_t out-buffer + fixed-offset readers.
	struct TraceOut {
		unsigned char raw[0x100];
		float Frac() const { return *reinterpret_cast<const float*>(raw + 0x2C); }
		Vector End() const {
			return Vector(*reinterpret_cast<const float*>(raw + 0x0C),
			              *reinterpret_cast<const float*>(raw + 0x10),
			              *reinterpret_cast<const float*>(raw + 0x14));
		}
		Vector Normal() const {
			return Vector(*reinterpret_cast<const float*>(raw + 0x18),
			              *reinterpret_cast<const float*>(raw + 0x1C),
			              *reinterpret_cast<const float*>(raw + 0x20));
		}
		float PlaneDist() const { return *reinterpret_cast<const float*>(raw + 0x24); }
		bool AllSolid() const { return raw[0x36] != 0; }
		bool StartSolid() const { return raw[0x37] != 0; }
	};

} // namespace EngineTraceABI

class IEngineTrace {
public:
	// Raw dispatch at the pinned index; callers go through the SEH-guarded
	// helper in TasEditor.cpp, never directly.
	void TraceRayAt(int index, const void* ray, unsigned mask, void* filter,
	                void* trace_out) {
		GetVirtualFunction<void(*)(IEngineTrace*, const void*, unsigned,
		                           void*, void*)>(this, index)
			(this, ray, mask, filter, trace_out);
	}
};
