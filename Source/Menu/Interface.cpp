#include "Interface.h"
#include "../../shareddefs.h"
#include "../World/WorldDraw.h"
#include "../World/Prediction.h"
#include "../World/NetVars.h"
#include "../World/BspWorld.h"
#include "../Editor/TasEditor.h"

#include <cmath>
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
		{ "Editor: Step Back", 0, [] { return true; }, [] { TasEditor::StepCursor(-1); } },
		{ "Editor: Step Forward", 0, [] { return true; }, [] { TasEditor::StepCursor(1); } },
		{ "Editor: Prev Segment", 0, [] { return true; }, [] { TasEditor::StepSegment(-1); } },
		{ "Editor: Next Segment", 0, [] { return true; }, [] { TasEditor::StepSegment(1); } },
		{ "Editor: Pick At Crosshair", 0, [] { return true; }, [] { TasEditor::PickAtCrosshair(); } },
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

void BasehookInterface::OnEndScene() {

	// Editor sim orchestration + in-world overlays run every frame, independent
	// of the menu. Both self-guard when out of game - and both run under
	// hook-level SEH so a fault becomes a logged report, not a dead game
	// (transient: the next frame tries again; the log has the evidence).
	{
		static int upd_reported = 0, wd_reported = 0;
		int code = 0;
		if (!GuardedEditorUpdate(&code) && upd_reported < 3) {
			upd_reported++;
			TasEditor::NoteExternalFault("TasEditor::Update", code);
		}
		if (!GuardedWorldDrawRender(&code) && wd_reported < 3) {
			wd_reported++;
			TasEditor::NoteExternalFault("WorldDraw::Render", code);
		}
	}

	// Replay diagnostics HUD: non-interactive status while a run plays back,
	// shown whether or not the menu is open.
	if (WorldDraw::show_replay_hud && g_tas.IsPlaying()) {
		ImGui::SetNextWindowPos(ImVec2(12, 52));
		ImGui::Begin("##stas_replay_hud", nullptr, kOverlayFlags | ImGuiWindowFlags_NoInputs);

		const std::vector<Run>& lib = g_tas.Library();
		const int sel = g_tas.Selected();
		if (g_tas.IsTestPlayback())
			ImGui::Text("run: (editor test)   segment %d", g_tas.PlaybackSegment() + 1);
		else if (sel >= 0 && sel < static_cast<int>(lib.size()))
			ImGui::Text("run: %s   segment %d / %d", lib[sel].name.c_str(),
				g_tas.PlaybackSegment() + 1, static_cast<int>(lib[sel].segments.size()));
		ImGui::Text("tick %d / %d", static_cast<int>(g_tas.PlaybackPosition()),
			static_cast<int>(g_tas.PlaybackTotal()));

		if (engine && entitylist && engine->IsInGame()) {
			void* player = entitylist->GetClientEntity(engine->GetLocalPlayer());
			const int vel_off = NetVars::Offset("DT_CSPlayer", "m_vecVelocity[0]");
			const int org_off = NetVars::Offset("DT_CSPlayer", "m_vecOrigin");
			const int flags_off = NetVars::Offset("DT_CSPlayer", "m_fFlags");
			if (player && vel_off) {
				const Vector v = NetVars::Get<Vector>(player, vel_off);
				ImGui::Text("speed %.1f u/s 2D   %.1f 3D",
					sqrtf(v.X * v.X + v.Y * v.Y), v.Length());
			}
			if (player && org_off) {
				const Vector o = NetVars::Get<Vector>(player, org_off);
				ImGui::Text("pos %.1f %.1f %.1f", o.X, o.Y, o.Z);
			}
			if (player && flags_off) {
				const int fl = NetVars::Get<int>(player, flags_off);
				ImGui::Text("%s%s", (fl & FL_ONGROUND) ? "ground" : "air",
					(fl & FL_DUCKING) ? " +duck" : "");
			}
		}
		ImGui::End();
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
	if (ImGui::Button(TasEditor::IsOpen() ? "Close TAS Editor" : "Open TAS Editor", ImVec2(250, 0)))
		TasEditor::Toggle();

	ImGui::Separator();
	ImGui::TextWrapped("Click a key, then press one to bind it (Escape clears). Hotkeys work with the menu closed. F8 toggles this menu. Rendering toggles live in the editor's Rendering tab.");

	// --- diagnostics -----------------------------------------------------
	// Dev readouts + the overlay vtable indices. All the visual toggles live in
	// the editor's Rendering tab now.
	ImGui::Separator();
	if (ImGui::CollapsingHeader("Diagnostics")) {
		ImGui::PushItemWidth(120);
		ImGui::InputInt("Box overlay index", &g_overlay_box_index);
		ImGui::InputInt("Line overlay index", &g_overlay_line_index);
		ImGui::PopItemWidth();
		ImGui::Checkbox("Line uses alpha form (8-arg)", &g_overlay_line_alpha);

		// Keep indices inside the vtable (20 methods) so a mistap can't dispatch
		// out of bounds.
		if (g_overlay_box_index < 0)   g_overlay_box_index = 0;
		if (g_overlay_box_index > 19)  g_overlay_box_index = 19;
		if (g_overlay_line_index < 0)  g_overlay_line_index = 0;
		if (g_overlay_line_index > 19) g_overlay_line_index = 19;

		ImGui::Separator();
		const WorldDraw::Diagnostics d = WorldDraw::LastDiagnostics();
		ImGui::Text("interfaces: %s", d.interfaces_ready ? "ready" : "MISSING");
		ImGui::Text("in game: %s", d.in_game ? "yes" : "no");
		ImGui::Text("local player index: %d", d.local_index);
		ImGui::Text("m_vecOrigin offset: %d", d.origin_offset);
		ImGui::Text("m_fFlags offset: %d", d.flags_offset);
		if (d.have_player) {
			ImGui::Text("origin: %.1f  %.1f  %.1f", d.origin.X, d.origin.Y, d.origin.Z);
			ImGui::Text("flags: 0x%08X  %s", d.flags, d.ducking ? "(ducking)" : "(standing)");

			// Basis check for the sim/preview: the movement pipeline's origin vs
			// the netvar origin. Read STANDING STILL - any persistent dz here
			// means the preview and live hull really do use different bases.
			Vector move_origin;
			if (Prediction::LastRealMoveOrigin(move_origin))
				ImGui::Text("basis delta (net - movedata): %.2f  %.2f  %.2f",
					d.origin.X - move_origin.X, d.origin.Y - move_origin.Y,
					d.origin.Z - move_origin.Z);
		} else {
			ImGui::TextDisabled("no local player entity");
		}

		ImGui::Separator();
		const Prediction::Diag pd = Prediction::LastDiag();
		const char* pred_state =
			pd.ran       ? "running" :
			pd.installed ? "installed (idle)" : "not installed";
		ImGui::Text("prediction: %s", pred_state);
		ImGui::Text("interval/tick: %.4f   window: %d ticks (%.2f s)",
			pd.interval_per_tick, pd.ticks, pd.ticks * pd.interval_per_tick);
		if (pd.fault_count > 0) {
			ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.3f, 1.0f),
				"recovered faults: %d   last: client.dll+0x%llX  access 0x%llX",
				pd.fault_count, pd.last_fault_rva, pd.last_fault_access);
		}
	}

	ImGui::End();

	// The editor is its own window (needs the cursor, so menu-visible only).
	TasEditor::DrawWindow();
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

	// While a text field has focus (e.g. the editor's project name), keys are
	// typing, not hotkeys or bind captures.
	const bool typing = is_menu_visible && ImGui::GetIO().WantTextInput;

	if (first_press && !typing) {
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
