#include "Interface.h"
#include "../../shareddefs.h"

#include <cstdio>
#include <cstring>

namespace {
	// A run command, its bound hotkey, and the state(s) it is allowed in.
	// available() gates BOTH the menu button and hotkey dispatch, which enforces
	// the safety rules (only Emergency Stop in Playback; segment edits only while
	// recording). The set mirrors the hardware tool's HotkeyAction list.
	struct TasCommand {
		const char* name;
		int key;                 // virtual-key code; 0 = unbound
		bool (*available)();
		void (*execute)();
	};

	bool Idle()      { return g_tas.State() == TasState::Idle; }
	bool Recording() { return g_tas.State() == TasState::Recording; }

	TasCommand g_commands[] = {
		{ "Start Recording", 0, [] { return Idle(); }, [] { g_tas.StartRecording(); } },
		{ "Save Segment", 0, [] { return Recording(); }, [] { g_tas.SaveSegment(); } },
		{ "Overwrite Current Segment", 0, [] { return Recording(); }, [] { g_tas.OverwriteCurrentSegment(); } },
		{ "Delete Previous Segment", 0, [] { return Recording(); }, [] { g_tas.DeletePreviousSegment(); } },
		{ "Stop & Save Recording", 0, [] { return Recording(); }, [] { if (g_tas.StopRecordingAndSave()) g_tas.PersistSelected(); } },
		{ "Play Selected Recording", 0, [] { return Idle(); }, [] { g_tas.PlaySelected(); } },
		{ "Select Next Recording", 0, [] { return Idle(); }, [] { g_tas.SelectNext(); } },
		{ "Emergency Stop", 0, [] { return true; }, [] { g_tas.EmergencyStop(); } },
	};

	constexpr int kCommandCount = static_cast<int>(sizeof(g_commands) / sizeof(g_commands[0]));

	// Index of the command currently capturing a key, or -1 when not rebinding.
	int g_binding = -1;

	const ImGuiWindowFlags kOverlayFlags =
		ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
		ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize |
		ImGuiWindowFlags_NoSavedSettings;

	const char* KeyName(int vk) {
		if (vk == 0)
			return "Unbound";

		static char buffer[32];
		UINT scancode = MapVirtualKeyA(static_cast<UINT>(vk), MAPVK_VK_TO_VSC);

		bool extended = false;
		switch (vk) {
		case VK_LEFT: case VK_RIGHT: case VK_UP: case VK_DOWN:
		case VK_PRIOR: case VK_NEXT: case VK_HOME: case VK_END:
		case VK_INSERT: case VK_DELETE: case VK_DIVIDE: case VK_NUMLOCK:
			extended = true;
			break;
		}

		LONG lparam = static_cast<LONG>((scancode & 0xFF) << 16);
		if (extended)
			lparam |= (1 << 24);

		if (GetKeyNameTextA(lparam, buffer, sizeof(buffer)) > 0)
			return buffer;

		sprintf_s(buffer, "Key %d", vk);
		return buffer;
	}
}

void BasehookInterface::OnEndScene() {

	// Compact indicator while the menu is closed, so hotkey-only use has feedback.
	if (!is_menu_visible) {
		if (g_tas.IsRecording() || g_tas.IsPlaying()) {
			ImGui::SetNextWindowPos(ImVec2(12, 12));
			ImGui::Begin("##stas_indicator", nullptr, kOverlayFlags);
			if (g_tas.IsRecording())
				ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "REC   seg %d   +%d frames",
					static_cast<int>(g_tas.SessionSegmentCount()), static_cast<int>(g_tas.ActiveSegmentSize()));
			else
				ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f), "PLAY   %d / %d",
					static_cast<int>(g_tas.PlaybackPosition()), static_cast<int>(g_tas.PlaybackTotal()));
			ImGui::End();
		}
		return;
	}

	ImGui::SetNextWindowSize(ImVec2(480, 600), ImGuiSetCond_FirstUseEver);
	ImGui::Begin("sourceTAS", nullptr, ImGuiWindowFlags_NoSavedSettings);

	// --- state + status --------------------------------------------------
	const char* state_name =
		g_tas.IsRecording() ? "RECORDING" : g_tas.IsPlaying() ? "PLAYBACK" : "IDLE";
	ImGui::Text("State: %s", state_name);

	if (g_tas.IsRecording())
		ImGui::Text("Session: %d committed segment(s), %d frames in current",
			static_cast<int>(g_tas.SessionSegmentCount()), static_cast<int>(g_tas.ActiveSegmentSize()));
	else if (g_tas.IsPlaying())
		ImGui::Text("Progress: %d / %d frames",
			static_cast<int>(g_tas.PlaybackPosition()), static_cast<int>(g_tas.PlaybackTotal()));

	ImGui::TextWrapped("%s", g_tas.Status());
	ImGui::Separator();

	// --- recordings library (finalized, immutable runs) -----------------
	ImGui::Text("Recordings  (select one to play)");
	ImGui::BeginChild("runs", ImVec2(0, 120), true);
	const std::vector<Run>& library = g_tas.Library();
	if (library.empty()) {
		ImGui::TextDisabled("(none yet - Start Recording to make one)");
	} else {
		for (int i = 0; i < static_cast<int>(library.size()); ++i) {
			char label[160];
			sprintf_s(label, "%s   (%d seg, %d frames)##run%d",
				library[i].name.c_str(),
				static_cast<int>(library[i].segments.size()),
				static_cast<int>(library[i].FrameCount()), i);
			if (ImGui::Selectable(label, g_tas.Selected() == i))
				g_tas.Select(i);
		}
	}
	ImGui::EndChild();

	// Rename / delete the selected run (run mode only; finalized runs are
	// otherwise immutable). Renaming renames the .tas file on disk.
	if (g_tas.State() == TasState::Idle && g_tas.Selected() >= 0
		&& g_tas.Selected() < static_cast<int>(library.size())) {
		static int synced = -1;
		static char name_buffer[128] = "";
		const int sel = g_tas.Selected();
		if (synced != sel) {
			synced = sel;
			strncpy_s(name_buffer, sizeof(name_buffer), library[sel].name.c_str(), _TRUNCATE);
		}

		ImGui::PushItemWidth(250);
		if (ImGui::InputText("##rename", name_buffer, sizeof(name_buffer), ImGuiInputTextFlags_EnterReturnsTrue)) {
			g_tas.RenameSelected(name_buffer);
			synced = -1;
		}
		ImGui::PopItemWidth();
		ImGui::SameLine();
		if (ImGui::Button("Rename")) {
			g_tas.RenameSelected(name_buffer);
			synced = -1;
		}
		ImGui::SameLine();
		if (ImGui::Button("Delete")) {
			g_tas.DeleteSelected();
			synced = -1;
		}
	}

	ImGui::Separator();

	// --- actions + hotkeys ----------------------------------------------
	for (int i = 0; i < kCommandCount; ++i) {
		TasCommand& command = g_commands[i];
		const bool available = command.available();

		if (!available) {
			const ImVec4 dim(0.25f, 0.25f, 0.25f, 1.0f);
			ImGui::PushStyleColor(ImGuiCol_Button, dim);
			ImGui::PushStyleColor(ImGuiCol_ButtonHovered, dim);
			ImGui::PushStyleColor(ImGuiCol_ButtonActive, dim);
			ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.6f, 0.6f, 0.6f, 1.0f));
		}

		const bool clicked = ImGui::Button(command.name, ImVec2(250, 0));

		if (!available)
			ImGui::PopStyleColor(4);

		if (clicked && available)
			command.execute();

		ImGui::SameLine();
		ImGui::PushID(i);
		const char* label = (g_binding == i) ? "press a key..." : KeyName(command.key);
		if (ImGui::Button(label, ImVec2(150, 0)))
			g_binding = (g_binding == i) ? -1 : i;
		ImGui::PopID();
	}

	ImGui::Separator();
	ImGui::TextWrapped("Click a key, then press one to bind it (Escape clears). Hotkeys work with the menu closed. F8 toggles this menu.");

	ImGui::End();
}


bool BasehookInterface::OnInputMessage(UINT type, WPARAM w_param, LPARAM l_param) {

	// Toggle the menu with F8.
	if (type == WM_KEYUP && w_param == VK_F8) {
		is_menu_visible = !is_menu_visible;
		ImGui::GetIO().MouseDrawCursor = is_menu_visible;
		matsurface->SetCursorAlwaysVisible(is_menu_visible);
	}

	// Capture and dispatch hotkeys on the first key-down (ignore auto-repeat).
	// Dispatch is gated by each command's availability, so e.g. only Emergency
	// Stop fires during playback. Replayed input never reaches here, so playback
	// can't drive the engine.
	const bool key_down = (type == WM_KEYDOWN || type == WM_SYSKEYDOWN);
	const bool first_press = key_down && !(l_param & (1 << 30));

	if (first_press) {
		if (g_binding >= 0) {
			// Assign the pressed key (Escape clears it), then stop capturing.
			g_commands[g_binding].key = (w_param == VK_ESCAPE) ? 0 : static_cast<int>(w_param);
			g_binding = -1;
			return false;
		}

		if (w_param != VK_F8) {
			for (TasCommand& command : g_commands) {
				if (command.key == static_cast<int>(w_param) && command.available()) {
					command.execute();
					break;
				}
			}
		}
	}

	if (is_menu_visible)
		ImGui_ImplDX9_WndProcHandler(window, type, w_param, l_param);

	return !is_menu_visible;
}
