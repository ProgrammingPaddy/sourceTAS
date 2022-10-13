#include "Interface.h"
#include"../../shareddefs.h"

void BasehookInterface::OnEndScene() {

	if (!is_menu_visible)
		return;
	if (RecordedFrames::recorder.IsRecordingActive()) {
		if (ImGui::Button("Stop Recording"))
			RecordedFrames::recorder.StopRecording();

	} else if (RecordedFrames::recorder.IsRerecordingActive()) {
		if (ImGui::Button("Save Re-recording"))
			RecordedFrames::recorder.StopRerecording(true);

		if (ImGui::Button("Clear Re-recording"))
			RecordedFrames::recorder.StopRerecording(false);
	} else {
		if (ImGui::Button("Start Recording"))
			RecordedFrames::recorder.StartRecording();

		if (!RecordedFrames::recorder.GetActiveRecording().empty()) {
			if (ImGui::Button("Clear Recorded Frames"))
				RecordedFrames::recorder.GetActiveRecording().clear();

			if (RecordedFrames::playback.IsPlaybackActive() && ImGui::Button("Stop Playback"))
				RecordedFrames::playback.StopPlayback();

			if (!RecordedFrames::playback.IsPlaybackActive() && ImGui::Button("Start Playback"))
				RecordedFrames::playback.StartPlayback(RecordedFrames::recorder.GetActiveRecording());
		}
	}
}


bool BasehookInterface::OnInputMessage(UINT type, WPARAM w_param, LPARAM l_param) {	

	if (type == WM_KEYUP && w_param == VK_INSERT) {
		
		is_menu_visible = !is_menu_visible;

		
		ImGui::GetIO().MouseDrawCursor = is_menu_visible;

		
		matsurface->SetCursorAlwaysVisible(is_menu_visible);
	}

	if (is_menu_visible)
		ImGui_ImplDX9_WndProcHandler(window, type, w_param, l_param);

	return !is_menu_visible;
}
