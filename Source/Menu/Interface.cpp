#include "Interface.h"
#include "../../shareddefs.h"
#include "../World/WorldDraw.h"
#include "../World/Prediction.h"
#include "../World/NetVars.h"
#include "../World/BspWorld.h"
#include "../Editor/TasEditor.h"
#include "Theme.h"
#include "RecordPanel.h"
#include "Breadcrumb.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>

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

	// Master arm switch (declared before the command table - the arm-toggle
	// command's lambda flips it). When false, bound hotkeys dispatch NOTHING;
	// the toggle command itself bypasses the gate (see dispatch).
	bool g_hotkeys_armed = true;

	// Autohop for human play (declared before the command table too). Applied
	// in the CreateMove hook; see RecordPanel::AutohopEnabled.
	bool g_autohop = false;

	TasCommand g_commands[] = {
		{ "Start Recording", 0, [] { return Idle(); }, [] { g_tas.StartRecording(); } },
		{ "Save Segment", 0, [] { return Recording(); }, [] { g_tas.SaveSegment(); } },
		{ "Overwrite Current Segment", 0, [] { return Recording(); }, [] { g_tas.OverwriteCurrentSegment(); } },
		{ "Delete Previous Segment", 0, [] { return Recording(); }, [] { g_tas.DeletePreviousSegment(); } },
		{ "Stop & Save Recording", 0, [] { return Recording(); }, [] { if (g_tas.StopRecordingAndSave()) g_tas.PersistSelected(); } },
		{ "Play Selected Recording", 0, [] { return Idle(); }, [] { g_tas.PlaySelected(); } },
		{ "Select Next Recording", 0, [] { return Idle(); }, [] { g_tas.SelectNext(); } },
		{ "Emergency Stop", 0, [] { return true; }, [] { g_tas.EmergencyStop(); } },
		{ "Editor: Step Back", 0, [] { return true; }, [] { TasEditor::StepCursor(-1); } },
		{ "Editor: Step Forward", 0, [] { return true; }, [] { TasEditor::StepCursor(1); } },
		{ "Editor: Prev Segment", 0, [] { return true; }, [] { TasEditor::StepSegment(-1); } },
		{ "Editor: Next Segment", 0, [] { return true; }, [] { TasEditor::StepSegment(1); } },
		{ "Editor: Pick At Crosshair", 0, [] { return true; }, [] { TasEditor::PickAtCrosshair(); } },
		{ "Freecam Toggle", 0, [] { return true; }, [] { TasEditor::ToggleFreecam(); } },
		{ "Toggle Autohop", 0, [] { return true; }, [] { g_autohop = !g_autohop; } },
		{ "Toggle Coast Line", 0, [] { return true; }, [] { TasEditor::ToggleCoastLine(); } },
		// Default-bound to TILDE: the key that opens the console also flips
		// the arm state, so opening the console disarms every other bind and
		// closing it (tilde again) re-arms - no detection, purely key-driven.
		{ "Toggle Hotkey Arm", VK_OEM_3, [] { return true; }, [] { g_hotkeys_armed = !g_hotkeys_armed; } },
	};

	constexpr int kCommandCount = static_cast<int>(sizeof(g_commands) / sizeof(g_commands[0]));

	// Index of the command currently capturing a key, or -1 when not rebinding.
	int g_binding = -1;

	// NOTE: console auto-disarm DETECTION was attempted twice and removed for
	// good (2026-08-06): a derived Con_IsVisible vtable index returned
	// nothing, and the cursor-visibility heuristic reported the cursor
	// showing during normal gameplay - silently gating EVERY hotkey. The
	// replacement is the "Toggle Hotkey Arm" COMMAND default-bound to tilde:
	// the console key itself flips the arm state. Key-driven, no detection.

	// ---- hotkey bind persistence (Documents\sourceTAS\binds.cfg) ----------
	// One line per command: "<vk> <command name>". Matched by NAME on load, so
	// reordering or extending the command table never mis-binds anything.
	std::string BindsPath() {
		char documents[MAX_PATH] = {};
		if (FAILED(SHGetFolderPathA(nullptr, CSIDL_PERSONAL, nullptr,
			SHGFP_TYPE_CURRENT, documents)))
			return {};
		std::string base = std::string(documents) + "\\sourceTAS";
		CreateDirectoryA(base.c_str(), nullptr);
		return base + "\\binds.cfg";
	}

	void SaveBinds() {
		const std::string p = BindsPath();
		if (p.empty())
			return;
		std::ofstream f(p, std::ios::trunc);
		if (!f)
			return;
		for (const TasCommand& command : g_commands)
			f << command.key << " " << command.name << "\n";
	}

	void LoadBinds() {
		const std::string p = BindsPath();
		if (p.empty())
			return;
		std::ifstream f(p);
		if (!f)
			return;
		int vk = 0;
		std::string name;
		while (f >> vk && std::getline(f, name)) {
			const size_t first = name.find_first_not_of(' ');
			if (first == std::string::npos)
				continue;
			name = name.substr(first);
			for (TasCommand& command : g_commands)
				if (name == command.name)
					command.key = vk;
		}
	}

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

	// Hook-level SEH shells (POD frames only): a fault in the editor's frame
	// work is logged and survived instead of killing the game.
	bool GuardedEditorUpdate(int* code) {
		__try {
			TasEditor::Update();
			return true;
		} __except (EXCEPTION_EXECUTE_HANDLER) {
			*code = static_cast<int>(GetExceptionCode());
			return false;
		}
	}
	bool GuardedMenuHud(BasehookInterface* self, int* code) {
		__try {
			self->MenuHudBody();
			return true;
		} __except (EXCEPTION_EXECUTE_HANDLER) {
			*code = static_cast<int>(GetExceptionCode());
			return false;
		}
	}
	bool GuardedWorldDrawRender(int* code) {
		__try {
			WorldDraw::Render();
			return true;
		} __except (EXCEPTION_EXECUTE_HANDLER) {
			*code = static_cast<int>(GetExceptionCode());
			return false;
		}
	}
}

// Load Segoe UI at 16px and apply the graphite/pink theme once, right after
// ImGui has its device but before the first frame builds the font atlas.
void BasehookInterface::OnInitialize() {
	ImGuiIO& io = ImGui::GetIO();
	io.Fonts->Clear();
	if (!io.Fonts->AddFontFromFileTTF("C:\\Windows\\Fonts\\segoeui.ttf", 16.0f))
		io.Fonts->AddFontDefault();   // fall back if the TTF is unavailable
	// Bold 18px for section headings (null = headings reuse the base font).
	Theme::HeadingFont() =
		io.Fonts->AddFontFromFileTTF("C:\\Windows\\Fonts\\segoeuib.ttf", 18.0f);
	Theme::Apply();
	LoadBinds();   // hotkeys persist across restarts
}

bool& RecordPanel::HotkeysArmed() {
	return g_hotkeys_armed;
}

bool& RecordPanel::AutohopEnabled() {
	return g_autohop;
}

// The recording / run controls, drawn as a tab inside the editor window.
void RecordPanel::Draw() {
	// --- state + status --------------------------------------------------
	const char* state_name =
		g_tas.IsRecording() ? "RECORDING" : g_tas.IsPlaying() ? "PLAYBACK" : "IDLE";
	const ImVec4 state_col = g_tas.IsRecording() ? Theme::Error
		: g_tas.IsPlaying() ? Theme::Success : Theme::Muted;
	ImGui::TextColored(state_col, "%s", state_name);
	ImGui::SameLine();
	if (g_tas.IsRecording())
		ImGui::TextDisabled("%d segment(s), %d frames in current",
			static_cast<int>(g_tas.SessionSegmentCount()), static_cast<int>(g_tas.ActiveSegmentSize()));
	else if (g_tas.IsPlaying())
		ImGui::TextDisabled("%d / %d frames",
			static_cast<int>(g_tas.PlaybackPosition()), static_cast<int>(g_tas.PlaybackTotal()));

	Theme::Heading("Recordings");
	ImGui::BeginChild("runs", ImVec2(0, 168), true);
	const std::vector<Run>& library = g_tas.Library();
	float itick = Prediction::LastDiag().interval_per_tick;
	if (itick <= 0.f) itick = 0.015f;
	if (library.empty()) {
		ImGui::TextDisabled("(none yet - Start Recording to make one)");
	} else {
		for (int i = 0; i < static_cast<int>(library.size()); ++i) {
			const Run& r = library[i];
			// Same at-a-glance stats as the project browser: map, run time,
			// date. Map/date come from the .tas v3 header + file time; older
			// recordings show "?"/no date.
			char when[32] = {};
			if (r.mtime) {
				FILETIME ft;
				ft.dwLowDateTime = static_cast<DWORD>(r.mtime & 0xFFFFFFFF);
				ft.dwHighDateTime = static_cast<DWORD>(r.mtime >> 32);
				FILETIME local = {};
				SYSTEMTIME st = {};
				if (FileTimeToLocalFileTime(&ft, &local) && FileTimeToSystemTime(&local, &st))
					sprintf_s(when, "%04d-%02d-%02d %02d:%02d",
						st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute);
			}
			const int frames = static_cast<int>(r.FrameCount());
			char label[224];
			sprintf_s(label, "%s   [%s]   %d seg   %d ticks (%.1f s)   %s##run%d",
				r.name.c_str(), r.map.empty() ? "?" : r.map.c_str(),
				static_cast<int>(r.segments.size()), frames, frames * itick,
				when[0] ? when : "-", i);
			if (ImGui::Selectable(label, g_tas.Selected() == i))
				g_tas.Select(i);
		}
	}
	ImGui::EndChild();

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
		if (Theme::Danger("Delete")) {
			g_tas.DeleteSelected();
			synced = -1;
		}
	}
	// Own undo for record-tab deletes - outside the selected-run row so it
	// stays reachable when the delete emptied the library. Session-scoped.
	if (g_tas.CanUndelete()) {
		if (ImGui::Button("Undo delete")) {
			g_tas.UndeleteLast();
		}
		Theme::Help("Restores the most recently deleted recording (this session) "
			"and rewrites its file on disk.");
	}

	Theme::Heading("Actions & hotkeys");
	ImGui::Checkbox("Arm hotkeys", &g_hotkeys_armed);
	Theme::Help("Master switch for every bound hotkey. Disarm before typing in "
		"console or chat - bound keys then dispatch nothing. Buttons here and "
		"F8 always work; the state persists with the UI settings.");
	if (!g_hotkeys_armed) {
		ImGui::SameLine();
		ImGui::TextColored(Theme::Warning, "DISARMED");
	}
	ImGui::Checkbox("Autohop", &g_autohop);
	Theme::Help("Server-style autobhop for YOUR play: hold jump and the hook "
		"strips the button while airborne, so every landing reads as a fresh "
		"press and hops. Applies to free play and recording (recordings keep "
		"the hops); replayed runs are never touched. Note: jump is also held "
		"off on ladders/in water while this is on. Bindable as 'Toggle "
		"Autohop'.");
	if (g_autohop) {
		ImGui::SameLine();
		ImGui::TextColored(Theme::Pink, "AUTOHOP");
	}

	// A muted section divider (grey label + thin underline), the same visual
	// language as the editor's sub-headings.
	auto GroupLine = [](const char* label) {
		ImGui::Spacing();
		ImGui::TextDisabled("%s", label);
		const ImVec2 p = ImGui::GetCursorScreenPos();
		const float w = ImGui::GetContentRegionAvailWidth();
		ImGui::GetWindowDrawList()->AddLine(
			ImVec2(p.x, p.y + 1.f), ImVec2(p.x + w, p.y + 1.f),
			ImGui::ColorConvertFloat4ToU32(ImVec4(0.55f, 0.55f, 0.62f, 0.30f)), 1.0f);
		ImGui::Dummy(ImVec2(0.f, 3.f));
	};
	// Command index -> section (mirrors the g_commands order): the loop prints
	// a heading when the section changes and starts a fresh 2-up row.
	auto GroupOf = [](int i) -> const char* {
		if (i <= 4)  return "Recording";
		if (i <= 7)  return "Playback & control";
		if (i <= 12) return "Editor navigation";
		return "Toggles";
	};

	// Two command+bind pairs per row - single file was a full screen tall.
	const char* cur_group = nullptr;
	int col = 0;
	for (int i = 0; i < kCommandCount; ++i) {
		TasCommand& command = g_commands[i];
		const char* grp = GroupOf(i);
		if (grp != cur_group) {
			cur_group = grp;
			GroupLine(grp);
			col = 0;
		}
		if (col % 2 == 1)
			ImGui::SameLine(392.f);
		col++;
		const bool available = command.available();
		// Accent (pink) for the two headline actions; neutral otherwise.
		const bool primary = available
			&& (std::strcmp(command.name, "Start Recording") == 0
				|| std::strcmp(command.name, "Play Selected Recording") == 0);
		const bool danger = available && std::strcmp(command.name, "Emergency Stop") == 0;

		bool clicked = false;
		if (!available) {
			const ImVec4 dim(0.16f, 0.16f, 0.18f, 1.0f);
			ImGui::PushStyleColor(ImGuiCol_Button, dim);
			ImGui::PushStyleColor(ImGuiCol_ButtonHovered, dim);
			ImGui::PushStyleColor(ImGuiCol_ButtonActive, dim);
			ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.5f, 0.5f, 0.55f, 1.0f));
			clicked = ImGui::Button(command.name, ImVec2(230, 0));
			ImGui::PopStyleColor(4);
		} else if (primary) {
			clicked = Theme::Accent(command.name, ImVec2(230, 0));
		} else if (danger) {
			clicked = Theme::Danger(command.name, ImVec2(230, 0));
		} else {
			clicked = ImGui::Button(command.name, ImVec2(230, 0));
		}
		if (clicked && available)
			command.execute();

		ImGui::SameLine();
		ImGui::PushID(i);
		const char* label = (g_binding == i) ? "press a key..." : KeyName(command.key);
		if (ImGui::Button(label, ImVec2(130, 0)))
			g_binding = (g_binding == i) ? -1 : i;
		ImGui::PopID();
	}
	ImGui::TextDisabled("binds: click the key button, press a key (Esc clears)");
	Theme::Help("Hotkeys work with the menu closed; F8 toggles the menu. Binds save "
		"to binds.cfg the moment you set them and load again on every restart.");
	// (The old dev-diagnostics section lived here - retired 2026-08-12: the
	// overlay indices have been pinned since Phase 1a and the readouts were
	// duplicated by the crash journal + solver gates probe.)
}

void BasehookInterface::OnEndScene() {

	// Editor sim orchestration + in-world overlays run every frame, independent
	// of the menu. Both self-guard when out of game - and both run under
	// hook-level SEH so a fault becomes a logged report, not a dead game
	// (transient: the next frame tries again; the log has the evidence).
	{
		static int upd_reported = 0, wd_reported = 0;
		int code = 0;
		Breadcrumb::Note(Breadcrumb::SlotFrame, "endscene: editor update");
		if (!GuardedEditorUpdate(&code) && upd_reported < 3) {
			upd_reported++;
			TasEditor::NoteExternalFault("TasEditor::Update", code);
		}
		Breadcrumb::Note(Breadcrumb::SlotFrame, "endscene: worlddraw");
		if (!GuardedWorldDrawRender(&code) && wd_reported < 3) {
			wd_reported++;
			TasEditor::NoteExternalFault("WorldDraw::Render", code);
		}
		Breadcrumb::Note(Breadcrumb::SlotFrame, "endscene: hud + menu");
	}

	// The HUD + menu body runs under the same hook-level SEH as Update and
	// WorldDraw (crash fix 2026-08-16): it ran UNGUARDED for its whole life,
	// so any fault in the (large) UI code was a straight process kill - the
	// old crash.log "PROCESS CRASH ... stage=(none)" rows. Same tradeoff as
	// the other guards: a fault is logged and the next frame retries.
	{
		static int mh_reported = 0;
		int code = 0;
		if (!GuardedMenuHud(this, &code) && mh_reported < 3) {
			mh_reported++;
			TasEditor::NoteExternalFault("Menu/HUD draw", code);
		}
	}
}

void BasehookInterface::MenuHudBody() {
	// Replay HUD: small, translucent, information-focused. Shown while a run
	// plays back whether or not the menu is open.
	if (WorldDraw::show_replay_hud && g_tas.IsPlaying()) {
		ImGui::SetNextWindowPos(ImVec2(12, 52));
		ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.05f, 0.05f, 0.07f, 0.55f));
		ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.93f, 0.29f, 0.60f, 0.30f));
		ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(8.f, 6.f));
		ImGui::Begin("##stas_replay_hud", nullptr, kOverlayFlags | ImGuiWindowFlags_NoInputs);

		float spd2 = 0.f; bool air = false, duck = false; bool have = false;
		if (engine && entitylist && engine->IsInGame()) {
			void* player = entitylist->GetClientEntity(engine->GetLocalPlayer());
			const int vel_off = NetVars::Offset("DT_CSPlayer", "m_vecVelocity[0]");
			const int flags_off = NetVars::Offset("DT_CSPlayer", "m_fFlags");
			if (player && vel_off) {
				const Vector v = NetVars::Get<Vector>(player, vel_off);
				spd2 = sqrtf(v.X * v.X + v.Y * v.Y);
				have = true;
			}
			if (player && flags_off) {
				const int fl = NetVars::Get<int>(player, flags_off);
				air = (fl & FL_ONGROUND) == 0;
				duck = (fl & FL_DUCKING) != 0;
			}
		}
		// Speed headline (accent), then a compact progress/state line.
		if (have)
			ImGui::TextColored(Theme::Pink, "%.0f u/s", spd2);
		ImGui::SameLine();
		ImGui::TextDisabled("%s%s", air ? "air" : "ground", duck ? " +duck" : "");
		ImGui::Text("seg %d   tick %d / %d", g_tas.PlaybackSegment() + 1,
			static_cast<int>(g_tas.PlaybackPosition()), static_cast<int>(g_tas.PlaybackTotal()));
		const int ptot = static_cast<int>(g_tas.PlaybackTotal());
		ImGui::ProgressBar(ptot > 0 ? static_cast<float>(g_tas.PlaybackPosition()) / ptot : 0.f,
			ImVec2(150.f, 6.f), "");
		ImGui::End();
		ImGui::PopStyleVar();
		ImGui::PopStyleColor(2);
	}

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
		Breadcrumb::Note(Breadcrumb::SlotFrame, "endscene: end (no menu)");
		return;
	}


	// The editor is its own window (needs the cursor, so menu-visible only).
	TasEditor::DrawWindow();
	Breadcrumb::Note(Breadcrumb::SlotFrame, "endscene: end");
}


bool BasehookInterface::OnInputMessage(UINT type, WPARAM w_param, LPARAM l_param) {

	// Engine-command marshal drain #2: the message pump lives on the MAIN
	// thread and runs in EVERY app state - including the main menu, where
	// CreateMove (drain #1) never fires. Without this, queued transitions
	// sat until the next map load and then fired late ("changed map and it
	// immediately put me in a demo"). Empty-queue cost: one mutex peek.
	TasEditor::DrainEngineCmds();

	// Key + mouse-button breadcrumb (console typing and clicks flow through
	// this hook too, so a death right after one names the exact message that
	// preceded it - the 2026-08-05 freeze followed a single click).
	if (type == WM_KEYDOWN || type == WM_KEYUP || type == WM_SYSKEYDOWN
		|| type == WM_SYSKEYUP || type == WM_CHAR
		|| (type >= WM_LBUTTONDOWN && type <= 0x020E))
		Breadcrumb::Note(Breadcrumb::SlotInput, "msg=0x%03X vk=%u menu=%d typing=%d",
			type, static_cast<unsigned>(w_param), is_menu_visible ? 1 : 0,
			(is_menu_visible && ImGui::GetIO().WantTextInput) ? 1 : 0);

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

	// While a text field has focus (e.g. the editor's project name), keys are
	// typing, not hotkeys or bind captures.
	const bool typing = is_menu_visible && ImGui::GetIO().WantTextInput;

	if (first_press && !typing) {
		// Ctrl+Z / Ctrl+Y: editor undo/redo (works with the menu open or
		// closed; skipped while a text field has focus via !typing above).
		if (GetKeyState(VK_CONTROL) & 0x8000) {
			if (w_param == 'Z') { TasEditor::Undo(); return false; }
			if (w_param == 'Y') { TasEditor::Redo(); return false; }
		}
		if (g_binding >= 0) {
			// Assign the pressed key (Escape clears it), then stop capturing.
			g_commands[g_binding].key = (w_param == VK_ESCAPE) ? 0 : static_cast<int>(w_param);
			g_binding = -1;
			SaveBinds();   // persist immediately - restarts keep the binds
			return false;
		}

		if (w_param != VK_F8) {
			for (TasCommand& command : g_commands) {
				if (command.key != static_cast<int>(w_param) || !command.available())
					continue;
				// "Toggle Hotkey Arm" bypasses the master gate - it must fire
				// while DISARMED, or disarming would be a one-way trap. Every
				// other command needs the switch armed.
				if (g_hotkeys_armed
					|| std::strcmp(command.name, "Toggle Hotkey Arm") == 0) {
					// EXEC without a matching DONE = the process died inside
					// this command.
					Breadcrumb::Note(Breadcrumb::SlotCommand, "EXEC %s", command.name);
					command.execute();
					Breadcrumb::Note(Breadcrumb::SlotCommand, "DONE %s", command.name);
				}
				break;
			}
		}
	}

	if (is_menu_visible)
		ImGui_ImplDX9_WndProcHandler(window, type, w_param, l_param);

	// Freecam captures the KEYBOARD only (the player must not walk while the
	// camera flies). The MOUSE passes through untouched: CInput's mouse
	// pipeline is polled (GetCursorPos), not WM-driven, so blocking WM mouse
	// only starves ImGui - the engine keeps turning mouse into the (frozen)
	// player's view angles, and the freecam reads those back as its look.
	// Fighting that pipeline with our own cursor recentering was the v1 bug:
	// two recenter loops feeding each other constant fake deltas.
	if (!is_menu_visible && TasEditor::FreecamActive()) {
		// Block key DOWNS only - key UPS must reach the engine, or a key held
		// when freecam engaged stays stuck down in the engine's key state
		// forever (its release was swallowed). A stuck +forward then WALKED
		// the player during the test-play grace window, aimed by the freecam
		// look yaw - the "deviation follows the freecam" report. Ups for keys
		// the engine never saw pressed are no-ops, so passing them is free.
		const bool keyboard = type == WM_KEYDOWN
			|| type == WM_SYSKEYDOWN || type == WM_CHAR;
		// Mouse BUTTONS and wheel are blocked too (no shooting/weapon-switch
		// from the flying camera); 0x0201..0x020E spans L/R/M/X down/up/dblclk
		// + both wheels. WM_MOUSEMOVE (0x0200) stays live for CInput's look.
		// Button UPS pass for the same stuck-state reason as key ups.
		const bool mouse_btn = type == WM_LBUTTONDOWN || type == WM_LBUTTONDBLCLK
			|| type == WM_RBUTTONDOWN || type == WM_RBUTTONDBLCLK
			|| type == WM_MBUTTONDOWN || type == WM_MBUTTONDBLCLK
			|| type == 0x020B /*XBUTTONDOWN*/ || type == 0x020D /*XBUTTONDBLCLK*/
			|| type == 0x020A /*MOUSEWHEEL*/ || type == 0x020E /*MOUSEHWHEEL*/;
		return !(keyboard || mouse_btn);
	}
	return !is_menu_visible;
}
