#pragma once

#include <Windows.h>
#include <Shlobj.h>
#include <cstdio>
#include <cstdarg>

// Breadcrumb journal for process deaths that raise NO exception - the class
// that leaves crash.log empty every time (heap-corruption fastfail, engine
// teardown, instant closes). Each subsystem overwrites its OWN fixed line in
// calibration\breadcrumb.log on every event; after a silent close the line
// with the HIGHEST seq is the last thing that ran. WriteFile lands in the OS
// file cache, which the KERNEL owns, so the record survives the process dying
// (only machine power loss could eat it) - no flush needed, so a note costs
// microseconds. Diagnostic only: no behavior change anywhere.
namespace Breadcrumb {

	enum Slot {
		SlotInput   = 0,   // last key message seen by the input hook
		SlotCommand = 1,   // last hotkey command dispatched (EXEC -> DONE)
		SlotFrame   = 2,   // last render-frame section reached
		SlotCount
	};

	constexpr int kWidth = 192;   // fixed record size (space-padded, \r\n)

	inline HANDLE File() {
		// Magic-static init is thread-safe; the handle deliberately lives for
		// the whole process, like the hooks. CREATE_ALWAYS = fresh journal per
		// injection (the previous one is stale once it has been read).
		static HANDLE h = [] {
			char documents[MAX_PATH] = {};
			if (FAILED(SHGetFolderPathA(nullptr, CSIDL_PERSONAL, nullptr,
				SHGFP_TYPE_CURRENT, documents)))
				return INVALID_HANDLE_VALUE;
			char path[MAX_PATH];
			_snprintf_s(path, _TRUNCATE, "%s\\sourceTAS\\calibration\\breadcrumb.log", documents);
			HANDLE f = CreateFileA(path, GENERIC_WRITE, FILE_SHARE_READ, nullptr,
				CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
			if (f != INVALID_HANDLE_VALUE) {
				char legend[kWidth];
				memset(legend, ' ', kWidth);
				const int n = _snprintf_s(legend, _TRUNCATE,
					"breadcrumbs: one line per subsystem, overwritten in place - after a "
					"silent close, the line with the HIGHEST seq ran last");
				if (n > 0)
					legend[n] = ' ';   // drop the NUL: keep the record printable
				legend[kWidth - 2] = '\r';
				legend[kWidth - 1] = '\n';
				DWORD written = 0;
				WriteFile(f, legend, kWidth, &written, nullptr);
			}
			return f;
		}();
		return h;
	}

	inline void Note(Slot slot, const char* fmt, ...) {
		HANDLE h = File();
		if (h == INVALID_HANDLE_VALUE)
			return;
		static volatile LONG s_seq = 0;
		const LONG seq = InterlockedIncrement(&s_seq);

		char body[128];
		va_list ap;
		va_start(ap, fmt);
		_vsnprintf_s(body, sizeof(body), _TRUNCATE, fmt, ap);
		va_end(ap);

		char rec[kWidth];
		memset(rec, ' ', kWidth);
		const int n = _snprintf_s(rec, _TRUNCATE,
			"seq=%06ld  t=%10llu  thr=%-6lu  %s",
			static_cast<long>(seq), GetTickCount64(), GetCurrentThreadId(), body);
		rec[(n >= 0 && n < kWidth) ? n : kWidth - 1] = ' ';
		rec[kWidth - 2] = '\r';
		rec[kWidth - 1] = '\n';

		// Positioned write: each slot owns one fixed line (line 0 is the
		// legend), so concurrent threads never interleave records.
		OVERLAPPED ov = {};
		ov.Offset = static_cast<DWORD>((slot + 1) * kWidth);
		DWORD written = 0;
		WriteFile(h, rec, kWidth, &written, &ov);
	}

}   // namespace Breadcrumb
