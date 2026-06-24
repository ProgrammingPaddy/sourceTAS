#pragma once

#include <windows.h>

// Each Source module exports 'CreateInterface', which hands back an engine
// interface singleton by exact version string. We request the versions this
// CS:S build registers; if a game update bumps one, scan the module's strings
// for the new "V..." name and update the caller. This is architecture-neutral
// (no jump/displacement parsing), so it works the same on x86 and x64.
using CreateInterfaceFn = void* (*)(const char* name, int* return_code);

template <typename T = void>
inline T* GetInterface(const char* module, const char* version) {
	HMODULE handle = GetModuleHandleA(module);

	if (!handle)
		return nullptr;

	auto create_interface = reinterpret_cast<CreateInterfaceFn>(GetProcAddress(handle, "CreateInterface"));

	if (!create_interface)
		return nullptr;

	return reinterpret_cast<T*>(create_interface(version, nullptr));
}
