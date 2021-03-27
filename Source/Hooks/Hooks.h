#pragma once

#include <cstrike/sdk.h>
#include <vmthook/vmthook.h>

extern std::unique_ptr<VMTHook> clientmode_hook;

typedef bool (__thiscall *CreateMove_t) (ClientModeShared*, float, CUserCmd*);

namespace Hooks {
	// ClientModeShared::CreateMove
	bool __fastcall CreateMove(ClientModeShared*, void*, float, CUserCmd*);
}
