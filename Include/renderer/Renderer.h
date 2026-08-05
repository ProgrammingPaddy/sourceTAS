#pragma once

#include <d3d9.h>
#include <windows.h>

#include <imgui/imgui.h>
#include <imgui/imgui_impl_dx9.h>

// Function prototypes for IDirect3DDevice9::Reset and IDirect3DDevice9::EndScene.
typedef HRESULT (STDMETHODCALLTYPE *EndScene_t) (IDirect3DDevice9*);
typedef HRESULT (STDMETHODCALLTYPE *Reset_t) (IDirect3DDevice9*, D3DPRESENT_PARAMETERS*);

// Make the ImGui WndProc handler accessible.
extern LRESULT ImGui_ImplDX9_WndProcHandler(HWND, UINT, WPARAM, LPARAM);

class DX9RenderMgr {
	protected:
		DX9RenderMgr(void) = default;
		~DX9RenderMgr(void);

		IDirect3DDevice9* d3d9_device = nullptr;

		// The shared device vtable we patch, so we can restore it on unload.
		void** d3d9_vtable = nullptr;

		// ImGui is initialised lazily on the first EndScene (we need the game's
		// real device, which the dummy-device hook setup never sees).
		bool initialized = false;

		HWND window = nullptr;
	public:
		// Disable copy and assignment constructors.
		DX9RenderMgr(DX9RenderMgr const&) = delete;
		DX9RenderMgr& operator=(DX9RenderMgr const&) = delete;

		// The original WndProc function from the specified window handle.
		WNDPROC WndProc = nullptr;

		// The original Reset and EndScene functions.
		Reset_t Reset = nullptr;
		EndScene_t EndScene = nullptr;

		// Diagnostics filled in by Initialize when something fails.
		const char* init_error = nullptr;
		HRESULT init_hr = 0;

		// Hooks Reset/EndScene via a throwaway device and replaces WndProc.
		bool Initialize(const HWND&);

		// Lazily initialises ImGui with the game's device, then draws a frame.
		void RenderFrame(IDirect3DDevice9*);

		// Called after 'ImGui_ImplDX9_Init' succeeds.
		virtual void OnInitialize() {};

		// Called before and after the device is reset.
		virtual void OnDeviceReset(bool) {};

		// Called just before EndScene.
		virtual void OnEndScene() {};

		// Called when a window message is received from WndProc.
		virtual bool OnInputMessage(UINT, WPARAM, LPARAM) {
			// Return true to pass the message to the game.
			return true;
		};
};

extern DX9RenderMgr& renderer;
