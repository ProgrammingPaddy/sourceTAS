#include "Hooks.h"
#include "../shareddefs.h"
#include "../World/Prediction.h"
#include "../Editor/TasEditor.h"

// Raw + leaked on purpose: the atexit destructor restored the vtable into
// game memory that is already gone at process exit (crash.log 2026-07-25,
// ??__Fclientmode_hook). The hook lives exactly as long as the process.
VMTHook* clientmode_hook = nullptr;


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

namespace {
	// View-slot discovery thunks. Each candidate slot forwards all four x64
	// register args to its original (transparent for any <=4-arg virtual),
	// then hands the second argument to the prober. Originals are captured
	// BEFORE hooking; the template gives each slot its own function identity.
	constexpr int kViewSlotLo = 12;
	constexpr int kViewSlotHi = 20;   // CreateMove(21) excluded
	using GenFn = void* (*)(void*, void*, void*, void*);
	uintptr_t g_view_originals[32] = {};

	template <int SLOT>
	void* ViewProbeThunk(void* thisptr, void* a, void* b, void* c) {
		void* const r = reinterpret_cast<GenFn>(g_view_originals[SLOT])(thisptr, a, b, c);
		TasEditor::ViewSlotSample(SLOT, a);
		return r;
	}
}

void Hooks::InstallViewProbes(VMTHook* hook) {
	static GenFn const thunks[] = {
		&ViewProbeThunk<12>, &ViewProbeThunk<13>, &ViewProbeThunk<14>,
		&ViewProbeThunk<15>, &ViewProbeThunk<16>, &ViewProbeThunk<17>,
		&ViewProbeThunk<18>, &ViewProbeThunk<19>, &ViewProbeThunk<20>,
	};
	const std::size_t total = hook->GetTotalFunctions();
	for (int s = kViewSlotLo; s <= kViewSlotHi; ++s) {
		if (static_cast<std::size_t>(s) >= total)
			break;
		g_view_originals[s] = reinterpret_cast<uintptr_t>(
			hook->GetOriginalFunction<void*>(static_cast<std::size_t>(s)));
		hook->HookFunction(reinterpret_cast<void*>(thunks[s - kViewSlotLo]),
			static_cast<std::size_t>(s));
	}
}
