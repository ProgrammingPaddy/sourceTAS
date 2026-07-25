#pragma once

#include <cstrike/Structures/Vector.h>

// Phase 1a world drawing. Renders the local player's collision hull and helper
// lines into the game's 3D scene via the engine debug-overlay system. This is
// strictly observational: it reads the player's networked origin/flags and
// queues overlays. It never writes game state or touches movement/prediction.
namespace WorldDraw {
	// Menu-controlled toggles.
	extern bool draw_test_marker;    // fixed world-origin marker (validates the call)
	extern bool draw_player_box;     // local player's collision AABB
	extern bool draw_player_marker;  // single dot at the player's feet origin
	extern bool draw_prediction;     // predicted path for a held input (Phase 1b)

	// Menu-controlled tuning.
	extern int   player_box_alpha;   // hull fill alpha; 0 = wireframe only
	extern float overlay_life_scale; // overlay lifetime as a multiple of frame time
	extern bool  show_replay_hud;    // status HUD while a run replays
	// Crash-isolation switch: skip ALL overlay submission while the solver
	// machinery runs. The engine walks/expires its overlay list on the game
	// thread while we submit from the render hook - a probabilistic race that
	// scales with churn; pausing our traffic during solver work for one
	// session proves or clears the theory.
	extern bool  pause_draw_busy;

	// Bottom hull-corner trails: parallel path lines offset to each bottom corner
	// of the (axis-aligned) player hull, for lining up and checking ramp boards.
	// Applies to both the live prediction line and the editor run line.
	// Order: [0] +X+Y   [1] +X-Y   [2] -X+Y   [3] -X-Y.
	extern bool corner_trails[4];

	// Prediction input (held for the whole predicted window).
	extern int   pred_ticks;         // number of ticks to predict
	extern bool  pred_live_input;    // use the live command's input (else the overrides below)
	extern bool  pred_autobhop;      // press jump on ticks that start grounded
	extern float pred_forwardmove;   // override wish forward move (units/s)
	extern float pred_sidemove;      // override wish side move (units/s)
	extern bool  pred_jump;          // override: hold IN_JUMP
	extern bool  pred_duck;          // override: hold IN_DUCK

	// Queue overlays for whatever is enabled. Safe to call every frame; it
	// no-ops when out of game or when an interface is missing.
	void Render();

	// Snapshot of the most recent Render(), surfaced in the menu so the overlay
	// indices and netvar offsets can be verified in-game without a rebuild.
	struct Diagnostics {
		bool   interfaces_ready = false;
		bool   in_game = false;
		int    local_index = -1;
		bool   have_player = false;
		int    origin_offset = 0;
		int    flags_offset = 0;
		Vector origin;
		int    flags = 0;
		bool   ducking = false;
	};
	Diagnostics LastDiagnostics();
}
