#include "Hooks.h"
#include "../shareddefs.h"
#include "../World/Prediction.h"

// Raw + leaked on purpose: the atexit destructor restored the vtable into
// game memory that is already gone at process exit (crash.log 2026-07-25,
// ??__Fclientmode_hook). The hook lives exactly as long as the process.
VMTHook* clientmode_hook = nullptr;


bool Hooks::CreateMove(ClientModeShared* thisptr, float frametime, CUserCmd* command) {
	// Don't do anything when called from CInput::ExtraMouseSample.
	if (!command->command_number)
		return false;

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
		}
		g_tas.RecordFrame(command);
	} else if (g_tas.IsPlaying()) {
		if (g_tas.ReplayFrame(command))
			engine->SetViewAngles(command->viewangles);
	}

	return false;
}
