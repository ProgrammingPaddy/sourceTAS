#include "Hooks.h"
#include "../shareddefs.h"

std::unique_ptr<VMTHook> clientmode_hook;


bool Hooks::CreateMove(ClientModeShared* thisptr, float frametime, CUserCmd* command) {
	// Don't do anything when called from CInput::ExtraMouseSample.
	if (!command->command_number)
		return false;

	if (g_tas.IsRecording()) {
		g_tas.RecordFrame(command);
	} else if (g_tas.IsPlaying()) {
		if (g_tas.ReplayFrame(command))
			engine->SetViewAngles(command->viewangles);
	}

	return false;
}
