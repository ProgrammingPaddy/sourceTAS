#include "Hooks/Hooks.h"
#include "Menu/Interface.h"
#include "Menu/Breadcrumb.h"
#include "World/Prediction.h"
#include "../shareddefs.h"

#include <cstdint>

TasEngine g_tas;

IBaseClientDLL* clientdll = nullptr;
IVEngineClient* engine = nullptr;
ISurface* matsurface = nullptr;
ClientModeShared* clientmode = nullptr;
IClientEntityList* entitylist = nullptr;
IVDebugOverlay* debugoverlay = nullptr;

// Overlay vtable indices pinned by static RE (see IVDebugOverlay.h). Adjustable
// at runtime from the menu so any final in-game verification is a one-click tweak.
int  g_overlay_box_index  = 1;
int  g_overlay_line_index = 3;
bool g_overlay_line_alpha = false;

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
	entitylist = GetInterface<IClientEntityList>("client.dll", "VClientEntityList003");

	// Debug-overlay interface for in-world drawing; version bumped across builds,
	// so try the known candidates and keep the first that resolves.
	debugoverlay = GetInterface<IVDebugOverlay>("engine.dll", "VDebugOverlay003");
	if (!debugoverlay)
		debugoverlay = GetInterface<IVDebugOverlay>("engine.dll", "VDebugOverlay004");

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
		clientmode_hook = new VMTHook(clientmode);
		clientmode_hook->HookFunction(&Hooks::CreateMove, 21);
		// Freecam: hook ONLY OverrideView's slot (the 12..20 discovery sweep
		// was convicted by bisect for the map-load freezes and is retired).
		Hooks::InstallViewHook(clientmode_hook);
	}

	// Install the movement look-ahead (hooks IPrediction::FinishMove so it runs
	// inside the engine's own prediction context).
	Prediction::Install();

	// Load any previously saved recordings from disk.
	g_tas.LoadFromDisk();

	// Initialize the renderer on the game window (it finds the device itself).
	// The window here is only the EnumWindows GUESS (null when the game is
	// alt-tabbed in fullscreen - the hidden-window case that killed input on
	// in-server injections); RenderFrame re-targets to the device's own
	// focus window on the first EndScene. Journal both so a dead-input
	// session names its cause immediately.
	HWND guess = GetGameWindow();
	Breadcrumb::Note(Breadcrumb::SlotCommand, "init: window guess=%p",
		reinterpret_cast<void*>(guess));
	const bool rend_ok = renderer.Initialize(guess);
	Breadcrumb::Note(Breadcrumb::SlotCommand,
		"init: renderer=%d input src=%d (0=DEAD until EndScene retarget)",
		rend_ok ? 1 : 0, renderer.window_source);

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
