#include "Hooks.h"
#include <stdexcept>
#include "../shareddefs.h"

std::unique_ptr<VMTHook> clientmode_hook;


bool __fastcall Hooks::CreateMove(ClientModeShared* thisptr, void* edx, float frametime, CUserCmd* command) {
	// Don't do anything when called from CInput::ExtraMouseSample.
	if (!command->command_number)
		return false;

	bool is_playback_active = RecordedFrames::playback.IsPlaybackActive();
	bool is_recording_active = RecordedFrames::recorder.IsRecordingActive();



	if (is_recording_active)
		RecordedFrames::Frames.push_back({ command });

	if (is_playback_active) {
		const size_t current_playback_frame = RecordedFrames::playback.GetCurrentFrame();

		try {
			RecordedFrames::Frames.at(current_playback_frame).Replay(command);
			engine->SetViewAngles(command->viewangles);

			if (current_playback_frame + 1 == RecordedFrames::Frames.size()) {
				RecordedFrames::playback.StopPlayback();
			}
			else {
				RecordedFrames::playback.SetCurrentFrame(current_playback_frame + 1);
			}
		}
		catch (std::out_of_range) {
			RecordedFrames::playback.StopPlayback();
		}
	}


	return false;
}

