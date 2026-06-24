// sourceTAS injector — pick a process and a DLL, inject via CreateRemoteThread.
//
// Build as x64 (the target cstrike_win64.exe is 64-bit): run build_injector.bat,
// or: cl /O2 /MT /DUNICODE /D_UNICODE Injector.cpp /link /SUBSYSTEM:WINDOWS

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <tlhelp32.h>
#include <commdlg.h>
#include <string>

#pragma comment(lib, "user32.lib")
#pragma comment(lib, "comdlg32.lib")
#pragma comment(lib, "gdi32.lib")

// Note: intentionally no Common-Controls v6 manifest dependency. It only themes
// the controls cosmetically, but an external/malformed manifest causes a
// side-by-side startup failure. Classic controls keep this a single, portable exe.

enum {
	IDC_COMBO_PROC = 1001,
	IDC_BTN_REFRESH,
	IDC_EDIT_DLL,
	IDC_BTN_BROWSE,
	IDC_BTN_INJECT,
	IDC_STATIC_STATUS,
};

static HWND g_combo, g_edit, g_status;

static void SetStatus(const std::wstring& text) {
	SetWindowTextW(g_status, text.c_str());
}

// Populate the process dropdown, preselecting the game if it is running.
static void PopulateProcesses() {
	SendMessageW(g_combo, CB_RESETCONTENT, 0, 0);

	HANDLE snapshot = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
	if (snapshot == INVALID_HANDLE_VALUE)
		return;

	PROCESSENTRY32W entry = { sizeof(entry) };
	int preselect = -1;

	if (Process32FirstW(snapshot, &entry)) {
		do {
			wchar_t label[320];
			wsprintfW(label, L"%s  (%lu)", entry.szExeFile, entry.th32ProcessID);

			int item = (int)SendMessageW(g_combo, CB_ADDSTRING, 0, (LPARAM)label);
			SendMessageW(g_combo, CB_SETITEMDATA, item, (LPARAM)entry.th32ProcessID);

			if (lstrcmpiW(entry.szExeFile, L"cstrike_win64.exe") == 0)
				preselect = item;
		} while (Process32NextW(snapshot, &entry));
	}

	CloseHandle(snapshot);
	SendMessageW(g_combo, CB_SETCURSEL, preselect >= 0 ? preselect : 0, 0);
}

static DWORD SelectedPid() {
	int item = (int)SendMessageW(g_combo, CB_GETCURSEL, 0, 0);
	if (item == CB_ERR)
		return 0;
	return (DWORD)SendMessageW(g_combo, CB_GETITEMDATA, item, 0);
}

// Returns true once the target process reports the module as loaded.
static bool ModuleLoaded(DWORD pid, const std::wstring& full_path) {
	std::wstring base = full_path.substr(full_path.find_last_of(L"\\/") + 1);

	HANDLE snapshot = CreateToolhelp32Snapshot(TH32CS_SNAPMODULE | TH32CS_SNAPMODULE32, pid);
	if (snapshot == INVALID_HANDLE_VALUE)
		return false;

	MODULEENTRY32W module = { sizeof(module) };
	bool found = false;

	if (Module32FirstW(snapshot, &module)) {
		do {
			if (lstrcmpiW(module.szModule, base.c_str()) == 0) {
				found = true;
				break;
			}
		} while (Module32NextW(snapshot, &module));
	}

	CloseHandle(snapshot);
	return found;
}

static bool Inject(DWORD pid, const std::wstring& dll, std::wstring& message) {
	wchar_t full[MAX_PATH];
	if (!GetFullPathNameW(dll.c_str(), MAX_PATH, full, nullptr)) {
		message = L"Invalid DLL path.";
		return false;
	}

	if (GetFileAttributesW(full) == INVALID_FILE_ATTRIBUTES) {
		message = L"DLL not found: " + std::wstring(full);
		return false;
	}

	HANDLE process = OpenProcess(
		PROCESS_CREATE_THREAD | PROCESS_QUERY_INFORMATION |
		PROCESS_VM_OPERATION | PROCESS_VM_WRITE | PROCESS_VM_READ,
		FALSE, pid);

	if (!process) {
		message = L"OpenProcess failed (try running the injector as administrator).";
		return false;
	}

	bool ok = false;
	SIZE_T size = (lstrlenW(full) + 1) * sizeof(wchar_t);
	void* remote = VirtualAllocEx(process, nullptr, size, MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE);

	if (remote && WriteProcessMemory(process, remote, full, size, nullptr)) {
		// kernel32 is mapped at the same address in every process of the session,
		// so LoadLibraryW's address here is valid in the target too.
		auto load_library = reinterpret_cast<LPTHREAD_START_ROUTINE>(
			GetProcAddress(GetModuleHandleW(L"kernel32.dll"), "LoadLibraryW"));

		HANDLE thread = CreateRemoteThread(process, nullptr, 0, load_library, remote, 0, nullptr);

		if (thread) {
			WaitForSingleObject(thread, INFINITE);
			CloseHandle(thread);

			// On x64 the thread exit code is only the low 32 bits of the HMODULE,
			// so confirm success by checking the target's module list instead.
			if (ModuleLoaded(pid, full)) {
				ok = true;
				message = L"Injected " + std::wstring(full) + L"  (PID " + std::to_wstring(pid) + L").";
			} else {
				message = L"Remote thread ran but the DLL is not loaded "
				          L"(architecture mismatch, or a dependent load failed).";
			}
		} else {
			message = L"CreateRemoteThread failed.";
		}
	} else {
		message = L"VirtualAllocEx / WriteProcessMemory failed.";
	}

	if (remote)
		VirtualFreeEx(process, remote, 0, MEM_RELEASE);

	CloseHandle(process);
	return ok;
}

static void BrowseForDll(HWND owner) {
	wchar_t file[MAX_PATH] = L"";
	GetWindowTextW(g_edit, file, MAX_PATH);

	OPENFILENAMEW ofn = { sizeof(ofn) };
	ofn.hwndOwner = owner;
	ofn.lpstrFilter = L"DLL Files\0*.dll\0All Files\0*.*\0";
	ofn.lpstrFile = file;
	ofn.nMaxFile = MAX_PATH;
	ofn.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_HIDEREADONLY;

	if (GetOpenFileNameW(&ofn))
		SetWindowTextW(g_edit, file);
}

// Default the DLL field to the first candidate that exists on disk.
static void SetDefaultDllPath() {
	wchar_t exe[MAX_PATH];
	GetModuleFileNameW(nullptr, exe, MAX_PATH);
	std::wstring dir(exe);
	dir.resize(dir.find_last_of(L"\\/") + 1);

	const std::wstring candidates[] = {
		dir + L"Basehook.dll",
		dir + L"..\\Output\\Release\\Basehook.dll",
		dir + L"..\\Output\\Debug\\Basehook.dll",
	};

	for (const std::wstring& candidate : candidates) {
		if (GetFileAttributesW(candidate.c_str()) != INVALID_FILE_ATTRIBUTES) {
			wchar_t full[MAX_PATH];
			if (GetFullPathNameW(candidate.c_str(), MAX_PATH, full, nullptr)) {
				SetWindowTextW(g_edit, full);
				return;
			}
		}
	}

	SetWindowTextW(g_edit, candidates[1].c_str());
}

static BOOL CALLBACK ApplyFont(HWND child, LPARAM font) {
	SendMessageW(child, WM_SETFONT, (WPARAM)font, TRUE);
	return TRUE;
}

static LRESULT CALLBACK WndProc(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam) {
	switch (message) {
	case WM_CREATE: {
		CreateWindowW(L"static", L"Process:", WS_CHILD | WS_VISIBLE,
			14, 16, 56, 20, hwnd, nullptr, nullptr, nullptr);
		g_combo = CreateWindowW(L"combobox", nullptr,
			WS_CHILD | WS_VISIBLE | WS_VSCROLL | CBS_DROPDOWNLIST,
			74, 13, 250, 320, hwnd, (HMENU)IDC_COMBO_PROC, nullptr, nullptr);
		CreateWindowW(L"button", L"Refresh", WS_CHILD | WS_VISIBLE,
			332, 12, 78, 26, hwnd, (HMENU)IDC_BTN_REFRESH, nullptr, nullptr);

		CreateWindowW(L"static", L"DLL:", WS_CHILD | WS_VISIBLE,
			14, 54, 56, 20, hwnd, nullptr, nullptr, nullptr);
		g_edit = CreateWindowW(L"edit", L"",
			WS_CHILD | WS_VISIBLE | WS_BORDER | ES_AUTOHSCROLL,
			74, 51, 250, 24, hwnd, (HMENU)IDC_EDIT_DLL, nullptr, nullptr);
		CreateWindowW(L"button", L"Browse...", WS_CHILD | WS_VISIBLE,
			332, 50, 78, 26, hwnd, (HMENU)IDC_BTN_BROWSE, nullptr, nullptr);

		CreateWindowW(L"button", L"Inject", WS_CHILD | WS_VISIBLE | BS_DEFPUSHBUTTON,
			74, 88, 120, 32, hwnd, (HMENU)IDC_BTN_INJECT, nullptr, nullptr);
		g_status = CreateWindowW(L"static", L"Ready.", WS_CHILD | WS_VISIBLE,
			14, 132, 396, 44, hwnd, (HMENU)IDC_STATIC_STATUS, nullptr, nullptr);

		EnumChildWindows(hwnd, ApplyFont, (LPARAM)GetStockObject(DEFAULT_GUI_FONT));
		SetDefaultDllPath();
		PopulateProcesses();
		return 0;
	}
	case WM_COMMAND:
		switch (LOWORD(wparam)) {
		case IDC_BTN_REFRESH:
			PopulateProcesses();
			return 0;
		case IDC_BTN_BROWSE:
			BrowseForDll(hwnd);
			return 0;
		case IDC_BTN_INJECT: {
			DWORD pid = SelectedPid();
			if (!pid) {
				SetStatus(L"Select a process first.");
				return 0;
			}

			wchar_t dll[MAX_PATH];
			GetWindowTextW(g_edit, dll, MAX_PATH);

			std::wstring message;
			SetStatus(L"Injecting...");
			Inject(pid, dll, message);
			SetStatus(message);
			return 0;
		}
		}
		return 0;
	case WM_DESTROY:
		PostQuitMessage(0);
		return 0;
	}

	return DefWindowProcW(hwnd, message, wparam, lparam);
}

int WINAPI wWinMain(HINSTANCE instance, HINSTANCE, LPWSTR, int show) {
	WNDCLASSW window_class = {};
	window_class.lpfnWndProc = WndProc;
	window_class.hInstance = instance;
	window_class.hCursor = LoadCursor(nullptr, IDC_ARROW);
	window_class.hbrBackground = (HBRUSH)(COLOR_BTNFACE + 1);
	window_class.lpszClassName = L"sourceTAS_injector";
	RegisterClassW(&window_class);

	int width = 440, height = 224;
	int x = (GetSystemMetrics(SM_CXSCREEN) - width) / 2;
	int y = (GetSystemMetrics(SM_CYSCREEN) - height) / 2;

	HWND hwnd = CreateWindowW(window_class.lpszClassName, L"sourceTAS injector",
		WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX,
		x, y, width, height, nullptr, nullptr, instance, nullptr);

	ShowWindow(hwnd, show);
	UpdateWindow(hwnd);

	MSG msg;
	while (GetMessageW(&msg, nullptr, 0, 0)) {
		if (!IsDialogMessageW(hwnd, &msg)) {
			TranslateMessage(&msg);
			DispatchMessageW(&msg);
		}
	}

	return 0;
}
