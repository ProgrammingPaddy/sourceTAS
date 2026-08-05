#pragma once

#include <cstrike/sdk.h>
#include <vmthook/vmthook.h>

extern VMTHook* clientmode_hook;   // leaked: no atexit vtable-restore into a dead game

namespace Hooks {
	// ClientModeShared::CreateMove (vtable index 21). On x64 there is a single
	// calling convention, so 'this' is the first parameter (no __fastcall/edx).
	bool CreateMove(ClientModeShared*, float, CUserCmd*);

	// OverrideView slot DISCOVERY (the fixed-index guess of 18 was never
	// called on this build - see freecam_probe.log 2026-08-04): every
	// candidate ClientModeShared virtual below the verified CreateMove(21)
	// gets a transparent 4-register forwarder that hands its second argument
	// to TasEditor's prober. The slot whose argument repeatedly matches the
	// engine's OWN view (GetViewAngles + player eye position) IS OverrideView
	// and gets pinned; every other wrapper stays a pure passthrough forever.
	void InstallViewProbes(VMTHook* hook);
}
