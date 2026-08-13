#pragma once

#include <cstrike/sdk.h>
#include <vmthook/vmthook.h>

extern VMTHook* clientmode_hook;   // leaked: no atexit vtable-restore into a dead game

namespace Hooks {
	// ClientModeShared::CreateMove (vtable index 21). On x64 there is a single
	// calling convention, so 'this' is the first parameter (no __fastcall/edx).
	bool CreateMove(ClientModeShared*, float, CUserCmd*);

	// Freecam view hook: ONLY OverrideView's slot (16 on this build, measured
	// by the retired 12..20 discovery sweep - freecam_probe.log, four
	// sessions). The broad sweep wrapped hot per-entity/key virtuals and was
	// convicted by bisect for the map-load freezes (2026-08-05: probes out =
	// freezes gone, ~100% repro before). One slot, memcpy-only wrapper;
	// TasEditor re-verifies the layout against engine truth every session
	// before any write, so a game update turns freecam off loudly instead of
	// writing blind.
	void InstallViewHook(VMTHook* hook);
}
