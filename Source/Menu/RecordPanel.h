#pragma once

// The movement recording / run controls (library, actions, hotkeys, dev
// diagnostics). Implemented in Interface.cpp where the command table and
// bind state live; drawn as a tab inside the TAS editor window.
namespace RecordPanel {
	void Draw();

	// Master hotkey arm switch (persisted with the UI prefs). Disarmed =
	// bound keys dispatch nothing, so typing in console/chat is safe.
	bool& HotkeysArmed();

	// Autohop for HUMAN input (persisted with the UI prefs): hold jump and the
	// CreateMove hook strips IN_JUMP while airborne, so landings hop like a
	// server autobhop plugin. Playback is never touched.
	bool& AutohopEnabled();
}
