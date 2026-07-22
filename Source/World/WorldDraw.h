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

	// Menu-controlled tuning.
	extern int   player_box_alpha;   // hull fill alpha; 0 = wireframe only
	extern float overlay_life_scale; // overlay lifetime as a multiple of frame time

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
