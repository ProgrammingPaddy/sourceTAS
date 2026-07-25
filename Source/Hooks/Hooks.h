#pragma once

#include <cstrike/sdk.h>
#include <vmthook/vmthook.h>

extern VMTHook* clientmode_hook;   // leaked: no atexit vtable-restore into a dead game

namespace Hooks {
	// ClientModeShared::CreateMove (vtable index 21). On x64 there is a single
	// calling convention, so 'this' is the first parameter (no __fastcall/edx).
	bool CreateMove(ClientModeShared*, float, CUserCmd*);
}
