#pragma once

#include <cstrike/Structures/Vector.h>
#include <vector>

#include "../../shareddefs.h"   // Frame, StartState

// Observational movement prediction, run from INSIDE the engine's own
// prediction pass. We hook IPrediction::FinishMove; once per frame (on the
// newest, first-time-predicted command) the local player is in a fully valid
// movement context. From there we can run hypothetical ticks of the real
// SetupMove -> ProcessMovement -> FinishMove pipeline and read the results,
// then restore the player, move-helper touch list, and globals byte-for-byte.
// It watches the real prediction; it changes nothing.
//
// Two consumers:
//   - Live look-ahead (Phase 1b): N ticks of the current/overridden input from
//     the live player state, drawn as the green path.
//   - Editor simulation (Phase 2): N ticks of authored input from an ABSOLUTE
//     StartState anchor, with a per-tick provider callback so closed-loop
//     generators (auto-strafe, auto-bhop) can react to the simulated state.
namespace Prediction {
	constexpr int kMaxSimTicks = 4096;

	struct SimState {
		Vector origin;     // feet origin after the tick
		Vector velocity;   // velocity after the tick
		int    flags;      // player m_fFlags after the tick (FL_ONGROUND etc.)
		// DIRECT READS (user directive 2026-08-14: no guessing - the hooked
		// engine is queryable): the collision hull top the engine actually
		// carries after the tick (CCollisionProperty m_vecMaxs.z; -1 if the
		// netvar is unresolved), and the two CMoveData float candidates for
		// m_flMaxSpeed read right after ProcessMovement - exported raw so
		// the capture itself identifies the field, no layout inference.
		float  hull_top = -1.f;
		float  mspd_a = -1.f;      // movedata +0x3C
		float  mspd_b = -1.f;      // movedata +0x40
		// The engine's OWN m_flStamina after the tick (-1 if unresolved):
		// makes the stamina laws per-tick observable instead of inferred -
		// any model drift is caught at its birth tick with the true value.
		float  stamina_ms = -1.f;
	};

	// Fills `out` with the input for `tick`, given the simulated state after the
	// previous tick. Runs on the game thread inside the sim; must be pure math.
	using SimFrameFn = void(*)(int tick, const SimState& prev, Frame* out);

	// Resolve interfaces and install the FinishMove hook (from basehook_init).
	void Install();

	// Live look-ahead path (per-tick feet origins) for drawing.
	void GetPath(std::vector<Vector>& out);

	// --- editor simulation (async request -> next prediction pass) ----------
	bool RequestSim(const StartState& anchor, int ticks, SimFrameFn provider);
	bool SimBusy();       // a request is pending execution
	bool SimReady();      // a result is waiting to be taken
	bool SimFaulted();    // last sim recovered from a fault (result empty)
	int  SimCount();      // ticks in the waiting result
	void TakeSim(Frame* frames, SimState* states);   // copies SimCount() items

	// Capture the live local player as a StartState (netvar reads; callable
	// from the render thread). Returns false when not in game.
	bool CaptureStartState(StartState& out);

	// The live local player's m_fFlags (FL_ONGROUND etc.), same access path as
	// CaptureStartState. False when not in game / offsets unresolved - callers
	// must then leave the input untouched (autohop's fail-passive rule).
	bool LiveFlags(int* out);

	// m_fFlags captured INSIDE the FinishMove hook after the newest
	// first-time-predicted command - the tick-exact grounded signal (the
	// netvar read above can lag prediction by a tick at CreateMove time).
	bool PredictedFlags(int* out);

	// Origin of the newest real command's movedata. Compared against the netvar
	// origin in the menu to verify both draw paths share one basis (read it
	// standing still - in motion they differ by one tick of movement).
	bool LastRealMoveOrigin(Vector& out);

	// m_flMaxSpeed of the newest real command (weapon-dependent: knife 250,
	// NO weapon 260 - the surf standard). Read, never assumed.
	bool LastRealMaxSpeed(float* out);

	// The engine's live gpGlobals->curtime (pinned RVA; see Prediction.cpp).
	// Frozen while the game is paused - overlay submission keys off it, since
	// debug overlays expire against curtime and a frozen clock means nothing
	// ever expires. Returns a constant sentinel when unavailable.
	float CurTime();

	// Trigger events the last editor sim fired (teleports/gravity zones the
	// client's prediction would otherwise ignore - see BspWorld::CheckTriggers).
	struct TriggerEvent {
		int    tick;
		int    type;       // 1 = teleport, 2 = gravity
		Vector to;         // teleport destination (type 1)
		float  gravity;    // player gravity scale (type 2)
	};
	int TriggerEventCount();
	const TriggerEvent* TriggerEventAt(int i);

	// Live diagnostics for the menu.
	struct Diag {
		bool  installed = false;
		bool  ran = false;
		int   ticks = 0;
		float interval_per_tick = 0.f;
		float sim_ms = 0.f;            // wall time of the last editor sim
		// Minimal fault net (engine physics can hit untested states). These stay
		// 0 in normal use; if one trips, the RVA/access pinpoint it.
		int                fault_count = 0;
		unsigned long long last_fault_rva = 0;
		unsigned long long last_fault_access = 0;
	};
	Diag LastDiag();

	// The player's STANDING collision hull as the engine reports it
	// (CCollisionProperty m_vecMins/m_vecMaxs, sampled while not ducked).
	// False = never measured yet and the standard box is being returned.
	bool PlayerHull(Vector* mins, Vector* maxs);
}
