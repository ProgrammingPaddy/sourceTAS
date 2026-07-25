#pragma once

#include "../World/Prediction.h"

// Phase 2: the TAS authoring layer. An edited run is an absolute StartState
// anchor plus an ordered list of segments; each segment is either raw per-tick
// frames (imported recordings / baked / fine-tuned) or a generated span (rate
// yaw, optimal auto-strafe, auto-bhop, pitch-follow). Compiling a run IS
// simulating it: generators are closed-loop (each tick's input is derived from
// the simulated state after the previous tick), run through the engine's real
// movement pipeline via Prediction::RequestSim. The result is both the exact
// per-tick frames (replayable through TasEngine) and per-tick world states
// (drawn as the run line; stepped with the cursor).
namespace TasEditor {
	// Per-frame orchestration: fetch finished sims, issue new ones (debounced).
	// Called from OnEndScene every frame, menu visible or not.
	void Update();

	// The editor window (call while the menu/cursor is available).
	void DrawWindow();
	bool IsOpen();
	void Toggle();

	// Tick cursor control (also bound to hotkeys).
	void StepCursor(int delta);
	void StepSegment(int direction);

	// Crosshair pick (also bound to a hotkey): acts per the Targets tab's mode -
	// tag/untag the aimed surf face, or place a board target where the hull
	// would rest on the aimed surface.
	void PickAtCrosshair();

	// Snapshot of everything WorldDraw needs to render the run line.
	struct DrawData {
		const Prediction::SimState* states;   // per-tick world states
		int count;
		const int* seg_starts;                // start tick of each segment
		int seg_count;
		const float* eff;                     // per-tick strafe efficiency (null if none)
		const float* maxgain;                 // per-tick max possible gain; <=0 = not scoreable
		float min_eff;                        // "optimal enough" threshold for coloring
		int cursor;                           // playhead: ticks before it draw locked/gray
		int sel_start, sel_end;               // selected segment range [start,end)
		StartState anchor;
		bool show_hull;                       // draw the collision hull ghost at the playhead
		bool have_view;                       // view angles at the playhead are valid
		float view_pitch, view_yaw;           // for the view-direction arrow
		// The REAL sim's measured pass point (where its path crosses the solver
		// target's height). Spliced into the drawn line as a vertex so the line
		// visibly runs through it - and decimation never skips that tick.
		bool have_pass;
		int pass_tick;                        // global tick whose edge gets spliced
		Vector pass_point;
		// Tag data: cumulative strafe quality + elapsed time (seconds).
		float seg_gain_pct;                   // % of optimal gain over the selected
		                                      // segment (-1 = not scoreable)
		float seg_time;                       // time at the segment's end (-1 = n/a)
		float cursor_time;                    // time at the playhead (-1 = n/a)
	};
	bool GetDrawData(DrawData& out);          // false when closed or no sim yet
	// True while the solver machinery hitches frames (search/verify/batch/
	// correction) - overlay lifetimes pin longer so the 3D draws don't flicker.
	bool SearchBusy();
	// The HEAVY phases only (search/verify/batch - the overlay-churn storms).
	// The draw pause keys off this so a brief correction doesn't blank the
	// world.
	bool SearchHeavy();
	// Record a fault caught by an OUTSIDE guard (hook-level SEH around Update/
	// WorldDraw): writes the where+code to the status line and solver_fault.log.
	void NoteExternalFault(const char* where, int code);
}
