#include "renderer.h"

#include <tlhelp32.h>
#include <cstring>
#include <vector>

namespace {
	// --- minimal x64 inline hook ----------------------------------------------
	// Patch a function's first 12 bytes with `mov rax, detour; jmp rax`. The game
	// renders on a single thread, so at runtime we can lift the hook, call the
	// original, and re-apply it without racing another caller.
	struct InlineHook {
		void* target = nullptr;
		unsigned char original[12] = {};
		unsigned char patch[12] = {};

		void Prepare(void* function, void* detour) {
			target = function;
			std::memcpy(original, function, sizeof(original));
			patch[0] = 0x48;                          // mov rax, imm64
			patch[1] = 0xB8;
			std::memcpy(&patch[2], &detour, sizeof(detour));
			patch[10] = 0xFF;                         // jmp rax
			patch[11] = 0xE0;
		}

		void Write(const unsigned char* bytes) const {
			if (!target)
				return;
			DWORD protect;
			VirtualProtect(target, sizeof(original), PAGE_EXECUTE_READWRITE, &protect);
			std::memcpy(target, bytes, sizeof(original));
			VirtualProtect(target, sizeof(original), protect, &protect);
			FlushInstructionCache(GetCurrentProcess(), target, sizeof(original));
		}

		void Enable() const { Write(patch); }
		void Disable() const { Write(original); }
	};

	InlineHook g_endscene_hook;
	InlineHook g_reset_hook;

	// Apply both hooks with every other thread suspended, retrying if a thread is
	// parked inside a region we are about to overwrite (which would corrupt it).
	bool InstallHooks() {
		const DWORD pid = GetCurrentProcessId();
		const DWORD self = GetCurrentThreadId();

		for (int attempt = 0; attempt < 25; ++attempt) {
			HANDLE snapshot = CreateToolhelp32Snapshot(TH32CS_SNAPTHREAD, 0);
			if (snapshot == INVALID_HANDLE_VALUE)
				return false;

			std::vector<HANDLE> suspended;
			bool conflict = false;

			THREADENTRY32 thread = { sizeof(thread) };
			if (Thread32First(snapshot, &thread)) {
				do {
					if (thread.th32OwnerProcessID != pid || thread.th32ThreadID == self)
						continue;

					HANDLE handle = OpenThread(THREAD_SUSPEND_RESUME | THREAD_GET_CONTEXT, FALSE, thread.th32ThreadID);
					if (!handle)
						continue;

					SuspendThread(handle);
					suspended.push_back(handle);

					CONTEXT context = {};
					context.ContextFlags = CONTEXT_CONTROL;
					if (GetThreadContext(handle, &context)) {
						for (const InlineHook* hook : { &g_endscene_hook, &g_reset_hook }) {
							const auto base = reinterpret_cast<DWORD64>(hook->target);
							if (base && context.Rip >= base && context.Rip < base + 12)
								conflict = true;
						}
					}
				} while (Thread32Next(snapshot, &thread));
			}

			if (!conflict) {
				g_endscene_hook.Enable();
				g_reset_hook.Enable();
			}

			for (HANDLE handle : suspended) {
				ResumeThread(handle);
				CloseHandle(handle);
			}
			CloseHandle(snapshot);

			if (!conflict)
				return true;

			Sleep(2);
		}

		return false;
	}
}

namespace DX9GenericHooks {
	// Replacement 'WndProc' function for intercepting all window messages.
	LRESULT STDMETHODCALLTYPE WndProc(HWND window, UINT message_type, WPARAM w_param, LPARAM l_param) {
		if (!renderer.OnInputMessage(message_type, w_param, l_param))
			return true;

		return CallWindowProc(renderer.WndProc, window, message_type, w_param, l_param);
	}

	// Inline-hook detour for IDirect3DDevice9::EndScene.
	HRESULT STDMETHODCALLTYPE EndScene(IDirect3DDevice9* device) {
		renderer.RenderFrame(device);

		// Call the real function with the hook briefly lifted.
		g_endscene_hook.Disable();
		HRESULT result = reinterpret_cast<EndScene_t>(g_endscene_hook.target)(device);
		g_endscene_hook.Enable();

		return result;
	}

	// Inline-hook detour for IDirect3DDevice9::Reset.
	HRESULT STDMETHODCALLTYPE Reset(IDirect3DDevice9* device, D3DPRESENT_PARAMETERS* params) {
		ImGui_ImplDX9_InvalidateDeviceObjects();
		renderer.OnDeviceReset(false);

		g_reset_hook.Disable();
		HRESULT result = reinterpret_cast<Reset_t>(g_reset_hook.target)(device, params);
		g_reset_hook.Enable();

		// Rebuild ONLY on a successful reset (crash fix 2026-08-16): while
		// tabbed out of fullscreen the game retries Reset and gets
		// D3DERR_DEVICELOST repeatedly - rebuilding ImGui device objects on
		// a lost device every retry corrupted the device state, the
		// "tabbed out too long = hard crash" family.
		if (SUCCEEDED(result)) {
			ImGui_ImplDX9_CreateDeviceObjects();
			renderer.OnDeviceReset(true);
		}

		return result;
	}
}

// Restore the patched functions and the original WndProc on destruction.
DX9RenderMgr::~DX9RenderMgr() {
	g_endscene_hook.Disable();
	g_reset_hook.Disable();

	if (this->window && this->WndProc)
		SetWindowLongPtr(this->window, GWLP_WNDPROC, LONG_PTR(this->WndProc));
}

void DX9RenderMgr::RenderFrame(IDirect3DDevice9* device) {
	// ImGui needs the game's real device, which we first see here in EndScene.
	if (!this->initialized) {
		// THE AUTHORITATIVE INPUT WINDOW (session 46b): ask the game's own
		// device for its focus window instead of trusting the injection-time
		// EnumWindows guess. The guess is null when the game is alt-tabbed
		// in fullscreen (hidden window) - exactly how injection happens -
		// which left EndScene hooked but ALL input dead: "injected, no menu,
		// no anything" with a perfectly healthy journal. If the guess and
		// the device disagree (or the guess never hooked), re-target here.
		D3DDEVICE_CREATION_PARAMETERS cp = {};
		if (SUCCEEDED(device->GetCreationParameters(&cp)) && cp.hFocusWindow) {
			const bool wrong_window = cp.hFocusWindow != this->window;
			const bool never_hooked = this->WndProc == nullptr;
			if (wrong_window || never_hooked) {
				// Undo any hook on the guessed window first.
				if (this->window && this->WndProc) {
					SetWindowLongPtr(this->window, GWLP_WNDPROC,
						LONG_PTR(this->WndProc));
					this->WndProc = nullptr;
				}
				this->window = cp.hFocusWindow;
				this->WndProc = reinterpret_cast<WNDPROC>(SetWindowLongPtr(
					this->window, GWLP_WNDPROC,
					LONG_PTR(&DX9GenericHooks::WndProc)));
				if (this->WndProc)
					this->window_source = 2;
			}
		}

		if (!ImGui_ImplDX9_Init(this->window, device))
			return;

		this->d3d9_device = device;
		this->initialized = true;
		this->OnInitialize();
	}

	// Draw NOTHING while the device is lost (crash fix 2026-08-16): during
	// a long tab-out the device sits in D3DERR_DEVICELOST and every draw
	// against it is undefined - the frame skips until Reset succeeds.
	if (device->TestCooperativeLevel() != D3D_OK)
		return;

	ImGui_ImplDX9_NewFrame();
	this->OnEndScene();
	ImGui::Render();
}

bool DX9RenderMgr::Initialize(const HWND& window) {
	this->window = window;

	// Pick the d3d9 runtime the game renders with (DXVK if present, else system).
	HMODULE d3d9_module = GetModuleHandleA("dxvk_d3d9.dll");

	if (!d3d9_module)
		d3d9_module = GetModuleHandleA("d3d9.dll");

	if (!d3d9_module) {
		this->init_error = "d3d9 module not loaded";
		return false;
	}

	// A throwaway window with a real (non-zero) client area for the dummy device.
	WNDCLASSEXA window_class = { sizeof(window_class) };
	window_class.lpfnWndProc = DefWindowProcA;
	window_class.hInstance = GetModuleHandleA(nullptr);
	window_class.lpszClassName = "sourceTAS_dummy";
	RegisterClassExA(&window_class);

	HWND dummy_window = CreateWindowA(window_class.lpszClassName, "", WS_OVERLAPPEDWINDOW,
		0, 0, 128, 128, nullptr, nullptr, window_class.hInstance, nullptr);

	// Released at the end regardless of which path created them.
	IDirect3D9Ex* d3d9ex = nullptr;
	IDirect3DDevice9Ex* device_ex = nullptr;
	IDirect3D9* d3d9 = nullptr;
	IDirect3DDevice9* device = nullptr;
	void** vtable = nullptr;

	// Modern Source uses Direct3DCreate9Ex. We only need the device's vtable to
	// read the EndScene/Reset *function* addresses, so either device type works.
	using Create9Ex_t = HRESULT (WINAPI*)(UINT, IDirect3D9Ex**);
	auto create_9ex = reinterpret_cast<Create9Ex_t>(GetProcAddress(d3d9_module, "Direct3DCreate9Ex"));

	if (create_9ex && SUCCEEDED(create_9ex(D3D_SDK_VERSION, &d3d9ex)) && d3d9ex) {
		D3DDISPLAYMODE mode = {};
		d3d9ex->GetAdapterDisplayMode(D3DADAPTER_DEFAULT, &mode);

		D3DPRESENT_PARAMETERS params = {};
		params.Windowed = TRUE;
		params.SwapEffect = D3DSWAPEFFECT_DISCARD;
		params.hDeviceWindow = dummy_window;
		params.BackBufferWidth = 128;
		params.BackBufferHeight = 128;
		params.BackBufferCount = 1;
		params.BackBufferFormat = mode.Format ? mode.Format : D3DFMT_X8R8G8B8;

		// D3DCREATE_FPU_PRESERVE keeps Source's float determinism intact.
		HRESULT hr = d3d9ex->CreateDeviceEx(D3DADAPTER_DEFAULT, D3DDEVTYPE_HAL, dummy_window,
			D3DCREATE_HARDWARE_VERTEXPROCESSING | D3DCREATE_FPU_PRESERVE, &params, nullptr, &device_ex);

		if (FAILED(hr))
			hr = d3d9ex->CreateDeviceEx(D3DADAPTER_DEFAULT, D3DDEVTYPE_HAL, dummy_window,
				D3DCREATE_SOFTWARE_VERTEXPROCESSING | D3DCREATE_FPU_PRESERVE, &params, nullptr, &device_ex);

		if (SUCCEEDED(hr) && device_ex) {
			vtable = *reinterpret_cast<void***>(device_ex);
			this->init_error = "Direct3DCreate9Ex";
		} else {
			this->init_error = "CreateDeviceEx failed";
			this->init_hr = hr;
		}
	}

	if (!vtable) {
		using Create9_t = IDirect3D9* (WINAPI*)(UINT);
		auto create_9 = reinterpret_cast<Create9_t>(GetProcAddress(d3d9_module, "Direct3DCreate9"));

		if (create_9)
			d3d9 = create_9(D3D_SDK_VERSION);

		if (d3d9) {
			D3DDISPLAYMODE mode = {};
			d3d9->GetAdapterDisplayMode(D3DADAPTER_DEFAULT, &mode);

			D3DPRESENT_PARAMETERS params = {};
			params.Windowed = TRUE;
			params.SwapEffect = D3DSWAPEFFECT_DISCARD;
			params.hDeviceWindow = dummy_window;
			params.BackBufferWidth = 128;
			params.BackBufferHeight = 128;
			params.BackBufferCount = 1;
			params.BackBufferFormat = mode.Format ? mode.Format : D3DFMT_X8R8G8B8;

			HRESULT hr = d3d9->CreateDevice(D3DADAPTER_DEFAULT, D3DDEVTYPE_HAL, dummy_window,
				D3DCREATE_HARDWARE_VERTEXPROCESSING | D3DCREATE_FPU_PRESERVE, &params, &device);

			if (FAILED(hr))
				hr = d3d9->CreateDevice(D3DADAPTER_DEFAULT, D3DDEVTYPE_HAL, dummy_window,
					D3DCREATE_SOFTWARE_VERTEXPROCESSING | D3DCREATE_FPU_PRESERVE, &params, &device);

			if (SUCCEEDED(hr) && device) {
				vtable = *reinterpret_cast<void***>(device);
				this->init_error = "Direct3DCreate9";
			} else {
				this->init_error = "CreateDevice failed";
				this->init_hr = hr;
			}
		}
	}

	// EndScene is index 42, Reset is index 16. These point to the module's shared
	// implementations, so inline-hooking them catches the game's device however
	// its vtable is laid out.
	void* endscene_function = vtable ? vtable[42] : nullptr;
	void* reset_function = vtable ? vtable[16] : nullptr;

	if (device_ex) device_ex->Release();
	if (d3d9ex) d3d9ex->Release();
	if (device) device->Release();
	if (d3d9) d3d9->Release();

	DestroyWindow(dummy_window);
	UnregisterClassA(window_class.lpszClassName, window_class.hInstance);

	if (!endscene_function)
		return false;

	g_endscene_hook.Prepare(endscene_function, &DX9GenericHooks::EndScene);
	g_reset_hook.Prepare(reset_function, &DX9GenericHooks::Reset);

	if (!InstallHooks()) {
		this->init_error = "InstallHooks failed";
		return false;
	}

	// Route window messages through us for menu toggling and input capture.
	// NOTE (session 46b): `window` here is only the injection-time
	// EnumWindows GUESS - it is null when the game is alt-tabbed in
	// fullscreen (hidden window), which silently killed all input while
	// EndScene kept running ("injected but no menu"). RenderFrame
	// re-targets to the device's own focus window on the first EndScene,
	// so this hook is best-effort only.
	this->WndProc = reinterpret_cast<WNDPROC>(
		SetWindowLongPtr(window, GWLP_WNDPROC, LONG_PTR(&DX9GenericHooks::WndProc))
	);
	if (this->window && this->WndProc)
		this->window_source = 1;

	return true;
}
