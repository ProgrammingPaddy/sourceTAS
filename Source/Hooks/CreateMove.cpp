#include "Hooks.h"
#include "../shareddefs.h"
#include "../World/Prediction.h"
#include "../Editor/TasEditor.h"
#include "../Menu/Breadcrumb.h"
#include "../Menu/RecordPanel.h"
#include "../World/BspWorld.h"
#include <cstrike/Definitions/Buttons.h>
#include <cstrike/Definitions/Const.h>

// Raw + leaked on purpose: the atexit destructor restored the vtable into
// game memory that is already gone at process exit (crash.log 2026-07-25,
// ??__Fclientmode_hook). The hook lives exactly as long as the process.
VMTHook* clientmode_hook = nullptr;


bool Hooks::CreateMove(ClientModeShared* thisptr, float frametime, CUserCmd* command) {
	// Don't do anything when called from CInput::ExtraMouseSample.
	if (!command->command_number)
		return false;

	// Game-thread breadcrumb: enter-without-exit after a freeze = the game
	// thread died in here (recording/replay); a stale pair with a LOWER seq
	// than the frame/update slots = the game thread stopped being scheduled
	// first (the engine froze elsewhere, not in our hook).
	Breadcrumb::Note(Breadcrumb::SlotGame, "createmove: enter");

	// Autohop (server-style autobhop for HUMAN input), the sim JM_AutoBhop's
	// exact button transform: while jump is HELD, the button is stripped by
	// default and pressed only on ticks that start GROUNDED with no press let
	// through the previous tick - the hook makes its own fresh edges. The
	// v1 strip-while-airborne version kept the button down across grounded
	// ticks, so the engine's old-buttons check blocked the hop whenever jump
	// was already held on ground (standing starts, any eaten landing jump =
	// "doesn't work perfectly"). Never touches playback; recording captures
	// the transform. Fails passive when the flags read is unavailable.
	static bool s_hop_prev = false;   // did the previous tick keep IN_JUMP?
	if (!g_tas.IsPlaying() && RecordPanel::AutohopEnabled()
		&& (command->buttons & IN_JUMP)) {
		// PredictedFlags first: captured in the FinishMove hook right after
		// the newest predicted command, so the landing shows up the very
		// next CreateMove. The plain netvar read (fallback) can lag a tick
		// - the "hop feels a tick late" / friction-eats-speed report.
		int fl = 0;
		const bool have = Prediction::PredictedFlags(&fl) || Prediction::LiveFlags(&fl);
		const bool grounded = have && (fl & FL_ONGROUND) != 0;
		if (!(grounded && !s_hop_prev))
			command->buttons &= ~IN_JUMP;
	}
	s_hop_prev = (command->buttons & IN_JUMP) != 0;

	if (g_tas.IsRecording()) {
		// Anchor every segment to the absolute world state at its FIRST tick
		// (not once per session), so overwrite/delete-back-to-empty re-captures
		// instead of keeping a stale start. The command's view angles are exact
		// (netvar eye angles are lowres).
		if (g_tas.NeedsStartCapture()) {
			StartState start;
			if (Prediction::CaptureStartState(start)) {
				start.pitch = command->viewangles.X;
				start.yaw   = command->viewangles.Y;
				g_tas.SetPendingStart(start);
			}
			// Stamp the level name once so the recording carries its map (the
			// record list shows it, like the project browser).
			char map[64];
			if (BspWorld::CurrentMapName(map, sizeof(map)))
				g_tas.SetSessionMap(map);
		}
		g_tas.RecordFrame(command);
	} else if (g_tas.IsPlaying()) {
		// Divergence tracking: the player's origin RIGHT NOW is the outcome
		// of the previously replayed frame - compare it against the sim.
		TasEditor::NotePlaybackTick(static_cast<int>(g_tas.PlaybackPosition()));
		// The replayed cmd carries its own viewangles, which is all the engine
		// needs for exact movement; SetViewAngles only syncs the local CAMERA
		// to the run. With freecam up that sync would stomp the engine angles
		// the freecam reads back as its look every frame - so skip it and the
		// freecam stays a third-party observer while the run plays underneath.
		if (g_tas.ReplayFrame(command)) {
			if (!TasEditor::FreecamActive())
				engine->SetViewAngles(command->viewangles);
		} else if (g_tas.IsPlaying()) {
			// Test-play GRACE tick (the teleport is settling; ReplayFrame fed
			// nothing yet): neutralize movement so no input - live keys or a
			// stuck engine key state - can walk the player off the anchor
			// before the first replayed frame. That walk was a real
			// divergence source, aimed wherever the freecam looked.
			command->forwardmove = 0.f;
			command->sidemove = 0.f;
			command->upmove = 0.f;
			command->buttons = 0;
		}
	}

	Breadcrumb::Note(Breadcrumb::SlotGame, "createmove: exit");
	return false;
}

namespace {
	// Slot 16 = ClientModeShared::OverrideView on this client.dll build
	// (measured by the retired discovery sweep). The wrapper forwards all
	// four x64 register args (OverrideView takes 2 - forwarding spares is
	// harmless) and hands the CViewSetup to TasEditor, whose hook-context
	// half only ever does guarded memcpys.
	constexpr int kViewSlot = 16;
	using GenFn = void* (*)(void*, void*, void*, void*);
	uintptr_t g_view_original = 0;

	void* ViewThunk(void* thisptr, void* a, void* b, void* c) {
		void* const r = reinterpret_cast<GenFn>(g_view_original)(thisptr, a, b, c);
		TasEditor::ViewSlotSample(kViewSlot, a);
		return r;
	}
}

void Hooks::InstallViewHook(VMTHook* hook) {
	if (static_cast<std::size_t>(kViewSlot) >= hook->GetTotalFunctions())
		return;
	g_view_original = reinterpret_cast<uintptr_t>(
		hook->GetOriginalFunction<void*>(static_cast<std::size_t>(kViewSlot)));
	hook->HookFunction(reinterpret_cast<void*>(&ViewThunk),
		static_cast<std::size_t>(kViewSlot));
}
