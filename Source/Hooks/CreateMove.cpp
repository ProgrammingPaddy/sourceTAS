#include "Hooks.h"
#include "../shareddefs.h"
#include "../World/Prediction.h"
#include "../Editor/TasEditor.h"
#include "../Menu/Breadcrumb.h"

// Raw + leaked on purpose: the atexit destructor restored the vtable into
// game memory that is already gone at process exit (crash.log 2026-07-25,
// ??__Fclientmode_hook). The hook lives exactly as long as the process.
VMTHook* clientmode_hook = nullptr;


bool Hooks::CreateMove(ClientModeShared* thisptr, float frametime, CUserCmd* command) {
	// Don't do anything when called from CInput::ExtraMouseSample.
	if (!command->command_number)
		return false;

	// Game-thread breadcrumb: enter-without-exit after a freeze = the game
	// thread died in here (recording/replay); a stale pair with a LOWER seq
	// than the frame/update slots = the game thread stopped being scheduled
	// first (the engine froze elsewhere, not in our hook).
	Breadcrumb::Note(Breadcrumb::SlotGame, "createmove: enter");

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

	Breadcrumb::Note(Breadcrumb::SlotGame, "createmove: exit");
	return false;
}

namespace {
	// Slot 16 = ClientModeShared::OverrideView on this client.dll build
	// (measured by the retired discovery sweep). The wrapper forwards all
	// four x64 register args (OverrideView takes 2 - forwarding spares is
	// harmless) and hands the CViewSetup to TasEditor, whose hook-context
	// half only ever does guarded memcpys.
	constexpr int kViewSlot = 16;
	using GenFn = void* (*)(void*, void*, void*, void*);
	uintptr_t g_view_original = 0;

	void* ViewThunk(void* thisptr, void* a, void* b, void* c) {
		void* const r = reinterpret_cast<GenFn>(g_view_original)(thisptr, a, b, c);
		TasEditor::ViewSlotSample(kViewSlot, a);
		return r;
	}
}

void Hooks::InstallViewHook(VMTHook* hook) {
	if (static_cast<std::size_t>(kViewSlot) >= hook->GetTotalFunctions())
		return;
	g_view_original = reinterpret_cast<uintptr_t>(
		hook->GetOriginalFunction<void*>(static_cast<std::size_t>(kViewSlot)));
	hook->HookFunction(reinterpret_cast<void*>(&ViewThunk),
		static_cast<std::size_t>(kViewSlot));
}
