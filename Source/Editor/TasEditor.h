#pragma once

#include "../World/Prediction.h"
#include "../World/Contact.h"

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

	// Undo/redo over segments + anchor (snapshot-based; user edits coalesce
	// per ~0.8 s burst, machine refinements never open a step). Ctrl+Z/Ctrl+Y
	// in the input hook and buttons in the Segments row both land here.
	void Undo();
	void Redo();

	// Teleport the player to the run's anchor (the Run tab button and the
	// "Editor: Teleport To Anchor" hotkey share this; needs sv_cheats 1).
	// AnchorValid gates the bind's availability.
	bool AnchorValid();
	void TeleportToAnchor();

	// Crosshair pick (also bound to a hotkey): acts per the Targets tab's mode -
	// tag/untag the aimed surf face, or place a board target where the hull
	// would rest on the aimed surface. While the freecam is active the ray and
	// the stored view angles are the CAMERA's.
	void PickAtCrosshair();

	// FREECAM: detached inspection camera (WASD + mouse, menu closed). While
	// active, player input is blocked and OverrideView repositions the render
	// view. Both the OverrideView SLOT and the CViewSetup field layout are
	// pinned from engine data before any write - and the toggle refuses to
	// engage (and to block input) until they are.
	void ToggleFreecam();
	bool FreecamActive();

	// COAST line: gray no-input trajectory off the run's end (see DrawData).
	// Toggle is on the Rendering tab, persisted, and bindable as a hotkey.
	void ToggleCoastLine();
	// Called by every view-probe thunk: sample candidate slot's 2nd argument.
	void ViewSlotSample(int slot, void* arg);
	// While the freecam is engaged, overwrite *eye with the camera position
	// and return true - in-world tags/billboards must face the CAMERA then.
	bool FreecamEye(Vector* eye);

	// Test-play divergence: the CreateMove hook reports each replayed frame's
	// index; the editor compares the REAL player origin against the sim and
	// verdicts when playback ends. DivergencePoint exposes the worst spot of
	// the last test play (> 0.5 u) for the in-world marker.
	void NotePlaybackTick(int index);
	bool DivergencePoint(Vector* p);

	// Map Solve: real-playback capture. The hook stashes each replayed cmd's
	// inputs so the NEXT tick's captured row (the state that cmd produced)
	// carries them; rows export as the solver diff's ground-truth CSV.
	void NotePlaybackInputs(int buttons, float yaw);

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
		// COAST line: a gray "release everything now" trajectory off the run's
		// end (last segment). Peeled from the sim so it never touches the run
		// states above; draw-only. Empty unless the toggle is on.
		const Vector* coast;                  // per-tick origins after the run end
		int coast_count;                      // 0 = none
		// Board events from the run's contact analysis (session 46n): every
		// AIR -> SURFACE transition, already filtered of trigger impulses.
		// WorldDraw renders these as hitmarkers.
		const Contact::BoardEvent* board_events;
		int board_event_count;
	};
	bool GetDrawData(DrawData& out);          // false when closed or no sim yet
	// The loaded demo-trace reference line (a DemoCap solver\demo_traces CSV:
	// the spectated runner's per-snapshot feet positions). Independent of the
	// run/sim state so it draws even with no line of our own; false when no
	// trace is loaded or the editor is closed.
	bool GetDemoLine(const Vector** pts, int* count);
	// The anchor picker's selected demo point (drawn as a gold in-world
	// mark). False when no trace is loaded or the editor is closed.
	bool GetDemoMark(Vector* out);
	// The editor's measured physics bounds for contact detection (gravity
	// u/s^2 and the max per-tick air-accel add u/s) - so the live prediction
	// line's analyzer uses the same calibrated numbers as the run line's.
	void ContactTuning(float* gravity, float* max_add);
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

	// THREAD MARSHAL for engine commands (crash fix 2026-08-16): with
	// multicore rendering on, EndScene (where all UI runs) is NOT the game
	// thread - the breadcrumb journal proved it (EndScene thr != CreateMove
	// thr). Small commands survived that; CONNECTION TRANSITIONS (playdemo,
	// disconnect, map loads) issued from the render thread deadlock the
	// engine - the whole "frozen game, no crash report" family. Rule: hooks
	// and UI PUSH commands; the CreateMove hook (game thread) DRAINS them.
	void PushEngineCmd(const char* cmd);
	void DrainEngineCmds();   // call ONLY from the main/game thread
	void ClearEngineCmds();   // drop pending commands (abort paths)
}
