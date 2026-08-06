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

	// Origin of the newest real command's movedata. Compared against the netvar
	// origin in the menu to verify both draw paths share one basis (read it
	// standing still - in motion they differ by one tick of movement).
	bool LastRealMoveOrigin(Vector& out);

	// The engine's live gpGlobals->curtime (pinned RVA; see Prediction.cpp).
	// Frozen while the game is paused - overlay submission keys off it, since
	// debug overlays expire against curtime and a frozen clock means nothing
	// ever expires. Returns a constant sentinel when unavailable.
	float CurTime();

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
