#include "WorldDraw.h"
#include "NetVars.h"
#include "Prediction.h"
#include "BspWorld.h"
#include "Contact.h"
#include "../Editor/TasEditor.h"
#include "../Menu/Breadcrumb.h"

#include <cmath>
#include <cstdint>
#include <vector>
#include <windows.h>

#include <cstrike/sdk.h>
#include <cstrike/Interfaces/IClientEntityList.h>
#include <cstrike/Interfaces/IVDebugOverlay.h>

// ============================================================================
// OVERLAY QUEUE (session 46e, retargeted 46k). All debugoverlay->Add* calls
// enqueue here; Drain makes the real engine vtable calls, each under per-call
// SEH with fault counting. Since 46k the whole draw pass runs on the GAME
// thread (SubmitFrame from the OverrideView hook), so producer and consumer
// are usually the same thread and Drain follows Render immediately - the
// queue survives as the fault-isolation layer and as the thread-safe path
// for the few render-thread producers left (the Debug tab's test box). The
// CRITICAL_SECTION guards only the brief append/swap; engine calls run
// outside it. Render-thread engine calls stay banned - the 2026-08-25 game
// update made them freeze the game.
// ============================================================================
namespace {
	struct OvlCmd {
		int    type;   // 0 = box, 1 = line (no-alpha), 2 = line (alpha)
		Vector a, b, c;
		QAngle ang;
		int    r, g, bb, aa;
		bool   noDepth;
		float  dur;
	};
	CRITICAL_SECTION* OvlCS() {
		static CRITICAL_SECTION cs;
		static bool once = [] { InitializeCriticalSection(&cs); return true; }();
		(void)once;
		return &cs;
	}
	std::vector<OvlCmd> g_ovl_back;    // producer (render thread) appends
	std::vector<OvlCmd> g_ovl_front;   // consumer (game thread) private after swap
	const size_t kOvlCap = 20000;      // safety bound if the game thread stalls
	// Diagnostics (session 46f): lifetime totals + last-drain size, so the
	// menu/journal can say WHERE "no draw" breaks - enqueue, drain, or the
	// engine call. volatile long; increments are cheap and racy-tolerant
	// (these are counters, not control).
	volatile long g_ovl_pushed = 0;
	volatile long g_ovl_drained = 0;
	volatile long g_ovl_last_drain = 0;
	volatile long g_ovl_backlog = 0;
	// Did the engine vtable call itself FAULT (session 46g)? If drained
	// climbs but faults climb WITH it, the calls are being made but the
	// engine rejects them (wrong index/signature after the update) - which
	// looks identical to "no draw" because the fault is SEH-swallowed. The
	// RVA localizes it inside engine.dll.
	volatile long g_ovl_faults = 0;
	volatile unsigned long long g_ovl_fault_rva = 0;

	long OvlFaultFilter(EXCEPTION_POINTERS* ep) {
		g_ovl_faults++;
		static uintptr_t base = reinterpret_cast<uintptr_t>(
			GetModuleHandleA("engine.dll"));
		const uintptr_t rip = reinterpret_cast<uintptr_t>(
			ep->ExceptionRecord->ExceptionAddress);
		g_ovl_fault_rva = (base && rip >= base) ? (rip - base) : rip;
		return EXCEPTION_EXECUTE_HANDLER;
	}

	// One engine call, PER-CALL SEH (session 46g): a bad overlay is isolated
	// (the rest of the drain still renders) and its fault is counted, not
	// hidden. POD-only inside the __try (no C++ unwinding).
	void OvlCallOne(const OvlCmd& c) {
		__try {
			if (c.type == 0) {
				GetVirtualFunction<void(*)(IVDebugOverlay*, const Vector&, const Vector&,
					const Vector&, const QAngle&, int, int, int, int, float)>(
					debugoverlay, g_overlay_box_index)(
					debugoverlay, c.a, c.b, c.c, c.ang, c.r, c.g, c.bb, c.aa, c.dur);
			} else if (c.type == 2) {
				GetVirtualFunction<void(*)(IVDebugOverlay*, const Vector&, const Vector&,
					int, int, int, int, bool, float)>(
					debugoverlay, g_overlay_line_index)(
					debugoverlay, c.a, c.b, c.r, c.g, c.bb, 255, c.noDepth, c.dur);
			} else {
				GetVirtualFunction<void(*)(IVDebugOverlay*, const Vector&, const Vector&,
					int, int, int, bool, float)>(
					debugoverlay, g_overlay_line_index)(
					debugoverlay, c.a, c.b, c.r, c.g, c.bb, c.noDepth, c.dur);
			}
		} __except (OvlFaultFilter(GetExceptionInformation())) {
		}
	}

	void OvlDrainSEH(const OvlCmd* cmds, size_t n) {
		if (!debugoverlay)
			return;
		for (size_t i = 0; i < n; ++i)
			OvlCallOne(cmds[i]);
	}
}

void OverlayQueue::PushBox(const Vector& o, const Vector& mn, const Vector& mx,
                           const QAngle& ang, int r, int g, int b, int a, float dur) {
	CRITICAL_SECTION* cs = OvlCS();
	EnterCriticalSection(cs);
	if (g_ovl_back.size() < kOvlCap) {
		OvlCmd c;
		c.type = 0; c.a = o; c.b = mn; c.c = mx; c.ang = ang;
		c.r = r; c.g = g; c.bb = b; c.aa = a; c.noDepth = false; c.dur = dur;
		g_ovl_back.push_back(c);
		g_ovl_pushed++;
	}
	LeaveCriticalSection(cs);
}

void OverlayQueue::PushLine(const Vector& o, const Vector& d,
                            int r, int g, int b, bool noDepth, float dur, bool alpha) {
	CRITICAL_SECTION* cs = OvlCS();
	EnterCriticalSection(cs);
	if (g_ovl_back.size() < kOvlCap) {
		OvlCmd c;
		c.type = alpha ? 2 : 1; c.a = o; c.b = d; c.c = Vector(); c.ang = QAngle();
		c.r = r; c.g = g; c.bb = b; c.aa = 255; c.noDepth = noDepth; c.dur = dur;
		g_ovl_back.push_back(c);
		g_ovl_pushed++;
	}
	LeaveCriticalSection(cs);
}

void OverlayQueue::Drain() {
	CRITICAL_SECTION* cs = OvlCS();
	EnterCriticalSection(cs);
	g_ovl_front.swap(g_ovl_back);
	g_ovl_back.clear();
	LeaveCriticalSection(cs);
	const long n = static_cast<long>(g_ovl_front.size());
	if (n > 0)
		OvlDrainSEH(g_ovl_front.data(), g_ovl_front.size());
	g_ovl_front.clear();
	g_ovl_last_drain = n;
	g_ovl_drained += n;
	g_ovl_backlog = static_cast<long>(g_ovl_back.size());
}

void OverlayQueue::Stats(long* pushed, long* drained, long* last_drain,
                         bool* overlay_ready, long* faults,
                         unsigned long long* fault_rva) {
	if (pushed)        *pushed = g_ovl_pushed;
	if (drained)       *drained = g_ovl_drained;
	if (last_drain)    *last_drain = g_ovl_last_drain;
	if (overlay_ready) *overlay_ready = (debugoverlay != nullptr);
	if (faults)        *faults = g_ovl_faults;
	if (fault_rva)     *fault_rva = g_ovl_fault_rva;
}

namespace WorldDraw {
	// Session 46j: default ON + persisted ("draw_master" in STAS_UI_PREFS).
	// It began life as the 46c freeze-bisect kill switch; the game-thread
	// overlay marshal (46e) fixed that freeze, so now it's just the normal
	// user toggle. Warm-up + budget ramp still gate the first frames.
	bool draw_master        = true;
	bool draw_test_marker   = false;
	bool draw_player_box    = true;
	bool draw_player_marker = true;
	bool draw_prediction    = false;
	bool show_hitmarkers_pred = true;   // session 46n: board-contact crosses
	bool show_hitmarkers_run  = true;
	bool show_demo_line       = true;   // session 46w: demo-trace reference line

	int   player_box_alpha   = 30;    // low: mostly wireframe, never reads as solid
	bool  overlay_zero_life  = false; // Debug tab: duration-0 engine idiom (see header)
	bool  show_replay_hud    = true;
	bool  pause_draw_busy    = true;  // perf valve: no overlays while solving

	// Once-per-present latch: SubmitFrame takes it, OnPresent (EndScene)
	// releases it, so reflection/portal extra view passes within one frame
	// don't double-submit.
	volatile long g_frame_submitted = 0;

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

	bool show_trig_events  = true;
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

	// Shared overlay budget (reset each Render frame; see WorldDraw.h).
	int g_ovl_left = 0;

	void DrawSegDigit(int digit, const Vector& org, const Vector& right,
	                  float h, int r, int g, int b, float dur) {
		if (digit < 0 || digit > 9)
			return;
		if (!WorldDraw::OverlayTake(7))
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

	// Hitmarker (session 46n): an X drawn IN the contact plane plus a short
	// normal spike, colored by the fraction of arrival speed the clip ate -
	// green = clean board, amber = scrubbed, red = hard hit. Caller pays the
	// overlay budget (3 lines).
	void DrawHitmarker(const Contact::BoardEvent& e, float dur) {
		if (!FiniteWorldPoint(e.pos))
			return;
		const float frac = (e.arrive_speed > 1.f) ? e.clip_loss / e.arrive_speed : 0.f;
		int r, g, b;
		if (frac < 0.02f)      { r = 70;  g = 255; b = 120; }
		else if (frac < 0.08f) { r = 255; g = 200; b = 60;  }
		else                   { r = 255; g = 70;  b = 70;  }
		// Two tangents spanning the contact plane.
		const Vector n = e.normal;
		const Vector up = (n.Z > -0.9f && n.Z < 0.9f) ? Vector(0.f, 0.f, 1.f)
		                                              : Vector(1.f, 0.f, 0.f);
		Vector t1(n.Y * up.Z - n.Z * up.Y,
		          n.Z * up.X - n.X * up.Z,
		          n.X * up.Y - n.Y * up.X);
		const float l1 = sqrtf(t1.X * t1.X + t1.Y * t1.Y + t1.Z * t1.Z);
		if (l1 < 1e-4f)
			return;
		t1 = Vector(t1.X / l1, t1.Y / l1, t1.Z / l1);
		const Vector t2(n.Y * t1.Z - n.Z * t1.Y,
		                n.Z * t1.X - n.X * t1.Z,
		                n.X * t1.Y - n.Y * t1.X);
		const float h = 9.f;
		const Vector p = e.pos;
		debugoverlay->AddLineOverlay(
			Vector(p.X - (t1.X + t2.X) * h, p.Y - (t1.Y + t2.Y) * h, p.Z - (t1.Z + t2.Z) * h),
			Vector(p.X + (t1.X + t2.X) * h, p.Y + (t1.Y + t2.Y) * h, p.Z + (t1.Z + t2.Z) * h),
			r, g, b, false, dur);
		debugoverlay->AddLineOverlay(
			Vector(p.X - (t1.X - t2.X) * h, p.Y - (t1.Y - t2.Y) * h, p.Z - (t1.Z - t2.Z) * h),
			Vector(p.X + (t1.X - t2.X) * h, p.Y + (t1.Y - t2.Y) * h, p.Z + (t1.Z - t2.Z) * h),
			r, g, b, false, dur);
		debugoverlay->AddLineOverlay(p,
			Vector(p.X + n.X * 7.f, p.Y + n.Y * 7.f, p.Z + n.Z * 7.f),
			r, g, b, false, dur);
	}

	// Player-attached overlays share the same one-frame lifetime as
	// everything else now; kept as a separate variable only because the
	// emitters read it by name.
	float g_dur_live = 0.f;

	// Overlay lifetime, FRAME-ALIGNED (session 46k). Submission happens
	// once per rendered frame (SubmitFrame via the OverrideView hook),
	// added BEFORE the engine draws that frame's overlays - so the only
	// correct lifetime is "this frame, then gone": drawn exactly once,
	// purged before the next frame's fresh batch. Anything longer overlaps
	// the next batch (double-drawn translucent hulls = the black flashes;
	// offset copies = misplaced elements); anything keyed to a clock we
	// guess at gaps or stacks (the whole 46i-46j strobe saga). Two ways to
	// say "one frame" to the engine, switchable in-game from the Debug tab:
	//   epsilon (default) - end time = now + 2 ms: alive for this frame's
	//     draw (curtime is constant within a frame), expired by the next
	//     frame's clock advance whenever the frame time exceeds 2 ms
	//     (i.e. anything under ~500 fps).
	//   duration 0 - the Source "one frame overlay" idiom (drawn once,
	//     purged after the pass), if this engine build still honors it.
	float FrameDuration() {
		const float d = WorldDraw::overlay_zero_life ? 0.f : 0.002f;
		g_dur_live = d;
		return d;
	}
}

void WorldDraw::OverlayStats(long* pushed, long* drained, long* last_drain,
                             bool* overlay_ready, long* faults,
                             unsigned long long* fault_rva) {
	OverlayQueue::Stats(pushed, drained, last_drain, overlay_ready, faults,
		fault_rva);
}

bool WorldDraw::OverlayTake(int count) {
	if (g_ovl_left < count)
		return false;
	g_ovl_left -= count;
	return true;
}

namespace {
	// SEH shell for the whole in-world pass (POD frame only). A fault in
	// sampling/enqueue/drain becomes a logged report, not a dead game.
	bool GuardedSubmit(int* code) {
		__try {
			WorldDraw::Render();
			OverlayQueue::Drain();
			return true;
		} __except (EXCEPTION_EXECUTE_HANDLER) {
			*code = static_cast<int>(GetExceptionCode());
			return false;
		}
	}
}

// Called from the OverrideView hook: game thread, once per rendered frame,
// before the engine draws it (see the header for why that alignment is the
// whole design). The latch collapses extra view passes within one frame.
void WorldDraw::SubmitFrame() {
	if (InterlockedExchange(&g_frame_submitted, 1))
		return;
	static int s_reported = 0;
	int code = 0;
	if (!GuardedSubmit(&code) && s_reported < 3) {
		s_reported++;
		TasEditor::NoteExternalFault("WorldDraw::SubmitFrame", code);
	}
}

// Called from the EndScene hook (render thread) once per present.
void WorldDraw::OnPresent() {
	InterlockedExchange(&g_frame_submitted, 0);
}

void WorldDraw::Render() {
	Diagnostics d;
	g_ovl_left = 2000;   // this frame's shared overlay budget

	// Frames since injection (this counter IS the warm-up clock below).
	static int s_frames_since_inject = 0;
	if (s_frames_since_inject < (1 << 30))
		s_frames_since_inject++;

	d.interfaces_ready = (engine && debugoverlay && entitylist && clientdll);
	if (!d.interfaces_ready) { g_diag = d; return; }

	d.in_game = engine->IsInGame();
	if (!d.in_game) { g_diag = d; return; }

	// MASTER GATE (session 46c): no overlay submission at all until the
	// user arms it from the menu. Guarantees injection cannot freeze on
	// the overlay path - the tool (menu/editor/recording) is fully usable
	// with drawing off.
	if (!draw_master) { g_diag = d; return; }

	// INJECTION WARM-UP (session 46). Injecting while ALREADY IN A SERVER
	// froze the whole game inside this pass's first frames (breadcrumb
	// 2026-08-25: "endscene: worlddraw" never returned; with queued
	// rendering the game thread then blocks behind the render thread -
	// task-manager kill, no crash record, the known "frozen game" family
	// the thread-marshal lesson in TasEditor.h documents). That first
	// in-game pass used to do EVERYTHING cold at once ON THE RENDER
	// THREAD: the first netvar-tree walk, the whole BSP file read+parse,
	// and the first mass overlay submission into an engine that expires
	// the list on the game thread. Hold ALL in-world work for the first
	// ~90 hooked frames after injection - by then injection transients
	// are over and the engine is in a steady frame loop. Menu injections
	// spend the warm-up at the menu (not in game) and feel nothing.
	if (s_frames_since_inject < 90) { g_diag = d; return; }
	// ...and ramp the overlay budget for the first drawing seconds so the
	// initial submission burst into the game-thread-expired list stays
	// small (steady state returns to the full budget).
	if (s_frames_since_inject < 300)
		g_ovl_left = 300;

	// (No cadence throttle: this runs once per rendered frame by
	// construction - SubmitFrame's once-per-present latch - and every
	// overlay lives ~one frame, so the paused-game overflow the old
	// per-tick throttle guarded against can't accumulate either.)

	// Perf valve: while the solver's HEAVY phases churn (search / verify /
	// batch), submit nothing - the frame budget belongs to the search.
	// Brief corrections don't pause; a stalled one used to blank the world.
	if (pause_draw_busy && TasEditor::SearchHeavy()) {
		g_diag = d;
		return;
	}

	// Fine-grained stage breadcrumbs through the whole pass (session 46):
	// this function froze the game once with only the coarse "endscene:
	// worlddraw" note to go on - after any future freeze the SlotFrame
	// line names the exact stage that never returned.
	Breadcrumb::Note(Breadcrumb::SlotWorld, "wd: live overlays");

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
			// First overlay of the frame. Since session 46e this only
			// ENQUEUES (the real engine call happens on the game thread in
			// OverlayQueue::Drain), so it can no longer hang here - the
			// breadcrumb just marks the draw code reached the first submit.
			Breadcrumb::Note(Breadcrumb::SlotWorld, "wd: box0 (enqueue)");
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
				if (!OverlayTake(2))
					break;   // shared frame budget spent
				debugoverlay->AddLineOverlay(prev, p, 40, 220, 90, false, g_dur_live);
				debugoverlay->AddBoxOverlay(p, dmin, dmax, kNoRotation, 40, 255, 90, 255, g_dur_live);
				for (int c = 0; c < 4; ++c)
					if (corner_trails[c] && OverlayTake(1))
						debugoverlay->AddLineOverlay(prev + kCornerOff[c], p + kCornerOff[c],
						                             120, 170, 200, false, g_dur_live);
				prev = p;
			}

			// Hitmarkers on the prediction line (session 46n): where the
			// predicted path makes contact after being airborne - the
			// ramp-board dial-in aid. Face-matched contacts and ground
			// landings only (a face-unknown residual here could be a booster
			// impulse - the look-ahead doesn't record trigger events).
			if (show_hitmarkers_pred) {
				static Prediction::SimState st[257];
				const int nst = Prediction::GetPathStates(st, 257);
				if (nst >= 2) {
					float grav = 800.f, madd = 70.f;
					TasEditor::ContactTuning(&grav, &madd);
					Contact::BoardEvent hv[16];
					const int ne = Contact::Analyze(st, nst,
						Prediction::LastDiag().interval_per_tick, grav, madd,
						hv, 16);
					for (int i = 0; i < ne; ++i)
						if ((hv[i].face_known || hv[i].grounded) && OverlayTake(3))
							DrawHitmarker(hv[i], g_dur_live);
				}
			}
		}
	}

	Breadcrumb::Note(Breadcrumb::SlotWorld, "wd: editor line");

	// (4) TAS editor run line (Phase 2): the simulated run drawn as an absolute
	// polyline. The playhead is the edit boundary: ticks before it draw gray
	// (locked history); the selected segment is colored by strafe efficiency
	// (green = at/above the min-eff threshold, orange = below); other future
	// segments blue; boundary markers white; ghost hull at the cursor tick.
	// Decimated so long runs stay within the overlay system's budget.
	// DEMO TRACE LINE (46w): a captured expert run drawn as a violet
	// reference polyline - loaded from the Map Solve tab, independent of the
	// run's own line (it draws with no sim at all).
	if (show_demo_line) {
		const Vector* dl = nullptr;
		int dn = 0;
		if (TasEditor::GetDemoLine(&dl, &dn) && dn > 1) {
			const int dstride = dn > 400 ? dn / 400 : 1;
			Vector dprev = dl[0];
			bool dprev_ok = FiniteWorldPoint(dprev);
			for (int i = 1; i < dn; ++i) {
				if (i != dn - 1 && (i % dstride) != 0)
					continue;
				const Vector dp = dl[i];
				if (!FiniteWorldPoint(dp))
					break;
				if (!OverlayTake(1))
					break;
				if (dprev_ok)
					debugoverlay->AddLineOverlay(dprev, dp, 190, 140, 255,
						false, duration);
				dprev = dp;
				dprev_ok = true;
			}
		}
	}

	TasEditor::DrawData ed;
	if (TasEditor::GetDrawData(ed)) {
		if (ed.anchor.valid && FiniteWorldPoint(ed.anchor.origin))
			debugoverlay->AddBoxOverlay(ed.anchor.origin, Vector(-3.f, -3.f, 0.f), Vector(3.f, 3.f, 8.f),
			                            kNoRotation, 60, 255, 60, 140, duration);

		// Hitmarkers on the run line (session 46n): the editor's board events,
		// already filtered of trigger impulses by the contact stage.
		if (show_hitmarkers_run && ed.board_events) {
			for (int i = 0; i < ed.board_event_count; ++i) {
				const Contact::BoardEvent& e = ed.board_events[i];
				if ((e.face_known || e.grounded) && OverlayTake(3))
					DrawHitmarker(e, duration);
			}
		}

		// Ceiling division: count 401..799 used to floor to stride 1 (no
		// decimation), exhausting the overlay budget at ~400 points - the
		// drawn line CUT OFF at exactly tick 400 (user-reported). Rounding
		// up keeps the whole run under the budget at every length.
		const int stride = ed.count > 400 ? (ed.count + 399) / 400 : 1;
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
			if (!OverlayTake(2))
				break;   // shared frame budget spent - the line ends here

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
					// Efficiency GRADIENT: everything at/above the min-eff cap
					// draws the same bright green; below it the color slides
					// through orange down to red across a 15%-wide band, so
					// the line itself says HOW far off optimal a stretch is.
					const float hi = ed.min_eff;
					const float lo = hi - 0.15f;
					float t = (hi > lo) ? (ed.eff[i] - lo) / (hi - lo) : 1.f;
					if (t < 0.f) t = 0.f;
					if (t > 1.f) t = 1.f;
					r = static_cast<int>(255.f + (60.f - 255.f) * t);
					g = static_cast<int>(60.f + (255.f - 60.f) * t);
					b = static_cast<int>(40.f + (120.f - 40.f) * t);
				} else {
					r = 50; g = 190; b = 110;                    // not scoreable (ground/landing)
				}
				const bool selected = (i >= ed.sel_start && i < ed.sel_end);
				if (!selected) { r = (r * 11) / 20; g = (g * 11) / 20; b = (b * 11) / 20; }
			}
			if (prev_ok) {
				debugoverlay->AddLineOverlay(prev, p, r, g, b, false, duration);
				for (int c = 0; c < 4; ++c)
					if (corner_trails[c] && OverlayTake(1))
						debugoverlay->AddLineOverlay(prev + kCornerOff[c], p + kCornerOff[c],
						                             120, 170, 200, false, duration);
			}
			prev = p;
			prev_ok = true;

			if (boundary)
				debugoverlay->AddBoxOverlay(p, Vector(-1.5f, -1.5f, -1.5f), Vector(1.5f, 1.5f, 1.5f),
				                            kNoRotation, 255, 255, 255, 255, duration);
		}

		// COAST line: gray "if you let go of everything at the run's end"
		// trajectory. Continues from the last run state through the peeled
		// no-input origins. Draw-only, decimated like the main line, on the
		// shared budget - purely a visual aid for placing the next segment.
		if (ed.coast_count > 0 && ed.count > 0 && FiniteWorldPoint(ed.states[ed.count - 1].origin)) {
			Vector cprev = ed.states[ed.count - 1].origin;
			bool cprev_ok = true;
			const int cstride = ed.coast_count > 200 ? ed.coast_count / 200 : 1;
			// A small tick at the seam so the release point is legible.
			if (OverlayTake(1))
				debugoverlay->AddBoxOverlay(cprev, Vector(-1.2f, -1.2f, -1.2f),
				                            Vector(1.2f, 1.2f, 1.2f), kNoRotation, 170, 170, 170, 200, duration);
			for (int i = 0; i < ed.coast_count; ++i) {
				if (i != ed.coast_count - 1 && (i % cstride) != 0)
					continue;
				const Vector cp = ed.coast[i];
				if (!FiniteWorldPoint(cp))
					break;
				if (!OverlayTake(1))
					break;
				if (cprev_ok)
					debugoverlay->AddLineOverlay(cprev, cp, 150, 150, 150, false, duration);
				cprev = cp;
				cprev_ok = true;
			}
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
		// Worst divergence of the last test play (> 0.5 u): a red marker at
		// the spot where reality left the sim.
		{
			Vector dp;
			if (TasEditor::DivergencePoint(&dp) && FiniteWorldPoint(dp) && OverlayTake(1))
				debugoverlay->AddBoxOverlay(dp, Vector(-2.5f, -2.5f, -2.5f),
					Vector(2.5f, 2.5f, 2.5f), kNoRotation, 255, 50, 50, 220, duration);
		}

		// Trigger events from the last sim: gold markers where the line
		// teleported / entered a gravity zone.
		if (show_trig_events)
		for (int ti = 0; ti < Prediction::TriggerEventCount(); ++ti) {
			const Prediction::TriggerEvent* ev = Prediction::TriggerEventAt(ti);
			if (!ev || !FiniteWorldPoint(ev->to) || !OverlayTake(1))
				break;
			if (ev->type == 1)
				debugoverlay->AddBoxOverlay(ev->to, Vector(-3.f, -3.f, 0.f),
					Vector(3.f, 3.f, 6.f), kNoRotation, 255, 170, 40, 180, duration);
			else if (ev->type == 3)
				debugoverlay->AddBoxOverlay(ev->to, Vector(-2.f, -2.f, -2.f),
					Vector(2.f, 2.f, 2.f), kNoRotation, 90, 255, 130, 180, duration);
			else
				debugoverlay->AddBoxOverlay(ev->to, Vector(-2.f, -2.f, -2.f),
					Vector(2.f, 2.f, 2.f), kNoRotation, 80, 170, 255, 180, duration);
		}

		Vector eye(d.origin.X, d.origin.Y, d.origin.Z + 64.f);
		TasEditor::FreecamEye(&eye);   // freecam engaged: tags face the camera
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
	Breadcrumb::Note(Breadcrumb::SlotWorld, "wd: bsp render");
	BspWorld::Render(d.origin, d.have_player, duration);
	Breadcrumb::Note(Breadcrumb::SlotWorld, "wd: done");

	g_diag = d;
}

WorldDraw::Diagnostics WorldDraw::LastDiagnostics() {
	return g_diag;
}
