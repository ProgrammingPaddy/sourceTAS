#pragma once

// The movement recording / run controls (library, actions, hotkeys, dev
// diagnostics). Implemented in Interface.cpp where the command table and
// bind state live; drawn as a tab inside the TAS editor window.
namespace RecordPanel {
	void Draw();

	// Master hotkey arm switch (persisted with the UI prefs). Disarmed =
	// bound keys dispatch nothing, so typing in console/chat is safe.
	bool& HotkeysArmed();
}
