#include "Hooks.h"
#include "../shareddefs.h"
#include "../World/Prediction.h"

std::unique_ptr<VMTHook> clientmode_hook;


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
