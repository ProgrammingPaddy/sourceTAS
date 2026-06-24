#include "Hooks/Hooks.h"
#include "Menu/Interface.h"
#include "../shareddefs.h"

#include <cstdint>

TasEngine g_tas;

IBaseClientDLL* clientdll = nullptr;
IVEngineClient* engine = nullptr;
ISurface* matsurface = nullptr;
ClientModeShared* clientmode = nullptr;

DX9RenderMgr& renderer = BasehookInterface::GetInstance();

// Find the game's main window from inside the process. This is more reliable
// than FindWindow("Valve001"), whose class-name lookup can fail across the
// atom table for another module's locally-registered class.
struct WindowSearch { HWND first; HWND valve; };

static BOOL CALLBACK EnumWindowProc(HWND window, LPARAM lparam) {
	DWORD window_pid = 0;
	GetWindowThreadProcessId(window, &window_pid);

	if (window_pid != GetCurrentProcessId() || !IsWindowVisible(window) || GetWindow(window, GW_OWNER))
		return TRUE;

	auto* search = reinterpret_cast<WindowSearch*>(lparam);

	if (!search->first)
		search->first = window;

	// Prefer the engine's known window class, but fall back to any main window.
	char class_name[64] = {};
	GetClassNameA(window, class_name, sizeof(class_name));

	if (lstrcmpA(class_name, "Valve001") == 0) {
		search->valve = window;
		return FALSE;
	}

	return TRUE;
}

static HWND GetGameWindow() {
	WindowSearch search = { nullptr, nullptr };
	EnumWindows(EnumWindowProc, reinterpret_cast<LPARAM>(&search));
	return search.valve ? search.valve : search.first;
}

DWORD WINAPI basehook_init(LPVOID dll_instance) {
	// Resolve engine interfaces by their exact version strings. If a game update
	// bumps a version, scan the module's strings for the new "V..." name.
	clientdll  = GetInterface<IBaseClientDLL>("client.dll", "VClient017");
	engine     = GetInterface<IVEngineClient>("engine.dll", "VEngineClient014");
	matsurface = GetInterface<ISurface>("vguimatsurface.dll", "VGUI_Surface030");

	// Recover g_pClientMode from CHLClient::HudUpdate (vtable index 10), which is
	// a thunk that begins:
	//     48 8B 0D <disp32>   mov rcx, [rip+disp32]   ; rcx = g_pClientMode
	// Read the RIP-relative displacement to reach the global and dereference it.
	if (clientdll) {
		uintptr_t hud_update = GetVirtualFunction<uintptr_t>(clientdll, 10);
		const uint8_t* code = reinterpret_cast<const uint8_t*>(hud_update);

		if (code[0] == 0x48 && code[1] == 0x8B && code[2] == 0x0D) {
			const int32_t displacement = *reinterpret_cast<const int32_t*>(hud_update + 3);
			clientmode = *reinterpret_cast<ClientModeShared**>(hud_update + 7 + displacement);
		}
	}

	// Hook 'CreateMove' from ClientModeShared (vtable index 21).
	if (clientmode) {
		clientmode_hook = std::make_unique<VMTHook>(clientmode);
		clientmode_hook->HookFunction(&Hooks::CreateMove, 21);
	}

	// Initialize the renderer on the game window (it finds the device itself).
	renderer.Initialize(GetGameWindow());

	return 0;
}

bool WINAPI DllMain(HINSTANCE dll_instance, DWORD call_reason, LPVOID reserved) {
	if (call_reason == DLL_PROCESS_ATTACH) {
		// We never get per-thread notifications, so opt out of them.
		DisableThreadLibraryCalls(dll_instance);

		// Run initialization off the loader lock on its own thread.
		if (HANDLE thread = CreateThread(nullptr, 0, basehook_init, dll_instance, 0, nullptr))
			CloseHandle(thread);
	}

	return true;
}
