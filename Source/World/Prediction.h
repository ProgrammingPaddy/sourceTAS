#pragma once

#include <cstrike/Structures/Vector.h>
#include <vector>

// Phase 1b: observational movement prediction, run from INSIDE the engine's own
// prediction pass. We hook IPrediction::FinishMove; once per frame (on the newest,
// first-time-predicted command) the local player is in a fully valid movement
// context, so we snapshot it, run N hypothetical ticks of the real
// SetupMove -> ProcessMovement -> FinishMove holding the current input, read each
// tick's feet origin, then restore the player, the move-helper touch list, and the
// touched globals byte-for-byte. It watches the real prediction; it changes nothing.
namespace Prediction {
	// Resolve interfaces and install the FinishMove hook. Call once from
	// basehook_init after the engine interfaces are up.
	void Install();

	// Copy the latest predicted path (per-tick feet origins) for drawing.
	void GetPath(std::vector<Vector>& out);

	// Live diagnostics for the menu.
	struct Diag {
		bool  installed = false;
		bool  ran = false;
		int   ticks = 0;
		float interval_per_tick = 0.f;
		// The look-ahead keeps a minimal fault net (engine physics can hit untested
		// states: water, ladders, other maps). These stay 0 in normal use; if one
		// ever trips, the RVA/access pinpoint it for a quick fix.
		int                fault_count = 0;
		unsigned long long last_fault_rva = 0;
		unsigned long long last_fault_access = 0;
	};
	Diag LastDiag();
}
