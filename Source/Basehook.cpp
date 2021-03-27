#include "Hooks/Hooks.h"
#include "Menu/Interface.h"
#include "Utilities/FindPattern.h"
#include "../shareddefs.h"

Recorder RecordedFrames::recorder;
Playback RecordedFrames::playback;

IBaseClientDLL* clientdll = nullptr;
IVEngineClient* engine = nullptr;
ISurface* matsurface = nullptr;
IClientEntityList* entitylist = nullptr;
ClientModeShared* clientmode = nullptr;

DX9RenderMgr& renderer = BasehookInterface::GetInstance();

void WINAPI basehook_init(LPVOID dll_instance) {
	// Get interfaces from game libraries using partial version strings.
	clientdll = GetInterface<IBaseClientDLL>("client.dll", "VClient0");
	engine = GetInterface<IVEngineClient>("engine.dll", "VEngineClient0");
	matsurface = GetInterface<ISurface>("vguimatsurface.dll", "VGUI_Surface0");
	entitylist = GetInterface<IClientEntityList>("client.dll", "VClientEntityList0");

	// Get a pointer to ClientModeShared from IBaseClientDLL::HudUpdate.
	clientmode = **reinterpret_cast<ClientModeShared***>(
		GetVirtualFunction<uintptr_t>(clientdll, 10) + 5
	);
	
	// Hook 'CreateMove' from ClientModeShared.
	clientmode_hook = std::make_unique<VMTHook>(clientmode);
	clientmode_hook->HookFunction(Hooks::CreateMove, 21);
	
	// Pattern scan for the Direct3D9 device pointer.
	IDirect3DDevice9* d3d9_device = **reinterpret_cast<IDirect3DDevice9***>(
		FindPattern("shaderapidx9.dll", "A1 ? ? ? ? 50 8B 08 FF 51 0C") + 1
	);
	
	// Initialize our renderer.
	renderer.Initialize(d3d9_device, FindWindowA("Valve001", NULL));
}

bool WINAPI DllMain(HINSTANCE dll_instance, DWORD call_reason, LPVOID reserved) {
	if (call_reason == DLL_PROCESS_ATTACH)
		CreateThread(0, 0, LPTHREAD_START_ROUTINE(basehook_init), dll_instance, 0, 0);

	return true;
}
