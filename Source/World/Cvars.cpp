#include "Cvars.h"

#include <Windows.h>
#include <Shlobj.h>
#include <cstdio>
#include <cstring>
#include <cstdint>

// How this works (all data, no engine calls):
//   1. Find the target name string ("sv_gravity") inside client.dll's image.
//   2. Scan the image for pointer-sized slots holding that string's address -
//      each is a candidate m_pszName field inside a static ConVar object.
//      (Both scans run ONCE per name and are cached; the layout search below
//      never rescans the image.)
//   3. Walk back to the object head (its vtable pointer - a pointer into the
//      module's own image). head..name distance is the NAME OFFSET.
//   4. From the object head, look for the engine's m_fValue/m_nValue pair -
//      a float immediately followed by an int equal to (int)float, with the
//      float inside that cvar's PLAUSIBLE range (sv_gravity is never 0, so
//      zero padding can't fake it). A candidate VALUE OFFSET is only accepted
//      when the SAME (name off, value off) pair validates on EVERY probe
//      cvar. If the object itself doesn't carry values (parent indirection),
//      its early pointer fields are tried the same way.
//   5. Reads then SEH-copy the float at object+value_off.
// Everything is logged to calibration\cvar_probe.log so the discovery is
// inspectable data, not trust.
namespace {

	// Per-cvar plausibility window for validation-time reads. Live servers
	// keep these well inside the window; zeros and garbage fall outside.
	struct ProbeSpec { const char* name; float lo, hi; };
	const ProbeSpec kProbes[] = {
		{ "sv_gravity",       1.f,   4000.f },
		{ "sv_airaccelerate", 0.1f, 100000.f },
	};
	constexpr int kProbeCount = static_cast<int>(sizeof(kProbes) / sizeof(kProbes[0]));

	struct Found {
		char name[64];
		const uint8_t* obj;      // object whose fields hold the live value
	};

	bool        g_ready = false;
	bool        g_tried = false;
	int         g_name_off = -1;
	int         g_val_off = -1;
	Found       g_found[16];
	int         g_found_count = 0;
	char        g_status[256] = "cvars: not probed yet";

	// ---- crash-proof primitives (POD SEH frames) -----------------------
	bool SafeRead(const void* src, void* dst, size_t n) {
		__try {
			memcpy(dst, src, n);
			return true;
		} __except (EXCEPTION_EXECUTE_HANDLER) {
			return false;
		}
	}

	void Log(const char* line) {
		char documents[MAX_PATH] = {};
		if (FAILED(SHGetFolderPathA(nullptr, CSIDL_PERSONAL, nullptr,
			SHGFP_TYPE_CURRENT, documents)))
			return;
		char path[MAX_PATH];
		_snprintf_s(path, _TRUNCATE, "%s\\sourceTAS\\calibration", documents);
		CreateDirectoryA(path, nullptr);
		_snprintf_s(path, _TRUNCATE, "%s\\sourceTAS\\calibration\\cvar_probe.log", documents);
		FILE* f = nullptr;
		if (fopen_s(&f, path, "a") == 0 && f) {
			fputs(line, f);
			fputc('\n', f);
			fclose(f);
		}
	}

	// ---- module geometry ----------------------------------------------
	struct Span { const uint8_t* lo; const uint8_t* hi; };

	bool ModuleSpan(HMODULE mod, Span* image) {
		if (!mod)
			return false;
		const uint8_t* base = reinterpret_cast<const uint8_t*>(mod);
		IMAGE_DOS_HEADER dos = {};
		if (!SafeRead(base, &dos, sizeof(dos)) || dos.e_magic != IMAGE_DOS_SIGNATURE)
			return false;
		IMAGE_NT_HEADERS64 nt = {};
		if (!SafeRead(base + dos.e_lfanew, &nt, sizeof(nt)) || nt.Signature != IMAGE_NT_SIGNATURE)
			return false;
		image->lo = base;
		image->hi = base + nt.OptionalHeader.SizeOfImage;
		return true;
	}

	bool InImage(const Span& s, const void* p) {
		return p >= s.lo && p < s.hi;
	}

	// The expensive part, run ONCE per name: every occurrence of the
	// NUL-terminated string, then every pointer-slot referencing one.
	// Readability is probed per page so unmapped gaps are skipped.
	struct SlotSet { const uint8_t* slots[64]; int n; };

	void FindNameSlots(const Span& img, const char* name, SlotSet* out) {
		out->n = 0;
		const size_t nlen = strlen(name) + 1;   // include the NUL
		const uint8_t* str_hits[32];
		int nstr = 0;

		static char page[0x1000];
		for (const uint8_t* p = img.lo; p + 0x1000 <= img.hi && nstr < 32; p += 0x1000) {
			if (!SafeRead(p, page, sizeof(page)))
				continue;
			// Overlap-safe: a match must fit inside this page copy.
			for (size_t i = 0; i + nlen <= sizeof(page); ++i) {
				if (page[i] == name[0] && memcmp(page + i, name, nlen) == 0) {
					str_hits[nstr++] = p + i;
					if (nstr >= 32) break;
				}
			}
		}
		if (!nstr)
			return;

		for (const uint8_t* p = img.lo; p + 0x1000 <= img.hi && out->n < 64; p += 0x1000) {
			if (!SafeRead(p, page, sizeof(page)))
				continue;
			const uint64_t* q = reinterpret_cast<const uint64_t*>(page);
			for (size_t i = 0; i < sizeof(page) / 8; ++i) {
				for (int s = 0; s < nstr; ++s) {
					if (q[i] == reinterpret_cast<uint64_t>(str_hits[s])) {
						if (out->n < 64)
							out->slots[out->n++] = p + i * 8;
						break;
					}
				}
			}
		}
	}

	// m_fValue/m_nValue adjacency at obj+off: float f then int n with
	// n == (int)f, f inside [lo, hi]. The engine keeps both fields in sync
	// on every cvar write, so a live ConVar always passes at the real
	// offset; padding zeros and garbage fail the range.
	bool ValuePairAt(const uint8_t* obj, int off, float lo, float hi, float* out) {
		float f = 0.f;
		int32_t n = 0;
		if (!SafeRead(obj + off, &f, 4) || !SafeRead(obj + off + 4, &n, 4))
			return false;
		if (f != f)   // NaN
			return false;
		if (f < lo || f > hi)
			return false;
		if (n != static_cast<int32_t>(f))
			return false;
		if (out)
			*out = f;
		return true;
	}

	// BINARY-DECODED parent hop (x64 server.dll Friction @1bd4a7/@20edb9,
	// disassembled 2026-08-15): the ENGINE reads cvar values through
	// ConVar::m_pParent (+0x38) -> m_fValue (+0x54). The child object can
	// keep its registered default forever - sv_stopspeed proved it: the
	// registration default is "100", the game's own SetValue(75.0f)
	// (@2f6f6c) lands on the PARENT, and Friction runs on 75 while a
	// parent-skipping read still returns 100. Validation, not trust: a real
	// parent is SELF-parented (+0x38 points to itself) and carries the SAME
	// name string; anything else falls back to the child. The parent may
	// legitimately live in ANOTHER module (server.dll on a listen server),
	// so there is no image-bounds check - SafeRead gates every dereference.
	const uint8_t* ParentOf(const uint8_t* obj, const char* name, int name_off) {
		const uint8_t* parent = nullptr;
		if (!SafeRead(obj + 0x38, &parent, sizeof(parent)) || !parent
			|| parent == obj)
			return obj;
		const uint8_t* pp = nullptr;
		if (!SafeRead(parent + 0x38, &pp, sizeof(pp)) || pp != parent)
			return obj;
		if (name && name_off >= 0) {
			const char* pname = nullptr;
			char buf[64] = {};
			if (!SafeRead(parent + name_off, &pname, sizeof(pname)) || !pname)
				return obj;
			if (!SafeRead(pname, buf, sizeof(buf) - 1))
				return obj;
			if (_stricmp(buf, name) != 0)
				return obj;
		}
		return parent;
	}

	// Live read = parent's value pair, child as fallback (a validated parent
	// whose pair fails plausibility would be out-of-sync mid-write; retry on
	// the child rather than failing the read).
	bool ReadLive(const uint8_t* obj, const char* name, int name_off,
	              int val_off, float* out) {
		const uint8_t* par = ParentOf(obj, name, name_off);
		if (ValuePairAt(par, val_off, -1e6f, 1e6f, out))
			return true;
		return par != obj && ValuePairAt(obj, val_off, -1e6f, 1e6f, out);
	}

	// Resolve one name from its CACHED slots with a given layout; returns
	// the object (self or parent) whose fields hold the value.
	const uint8_t* ResolveFromSlots(const Span& img, const SlotSet& set,
	                                float lo, float hi, int name_off, int val_off) {
		for (int i = 0; i < set.n; ++i) {
			const uint8_t* obj = set.slots[i] - name_off;
			if (!InImage(img, obj))
				continue;
			// Object head sanity: first slot (vtable) points into the image.
			uint64_t vt = 0;
			if (!SafeRead(obj, &vt, 8) || !InImage(img, reinterpret_cast<void*>(vt)))
				continue;
			if (ValuePairAt(obj, val_off, lo, hi, nullptr))
				return obj;
			// Parent indirection: early pointer fields may carry the values.
			for (int po = 8; po <= 0x40; po += 8) {
				uint64_t pp = 0;
				if (!SafeRead(obj + po, &pp, 8))
					continue;
				const uint8_t* par = reinterpret_cast<const uint8_t*>(pp);
				if (!InImage(img, par) || par == obj)
					continue;
				uint64_t pvt = 0;
				if (!SafeRead(par, &pvt, 8) || !InImage(img, reinterpret_cast<void*>(pvt)))
					continue;
				if (ValuePairAt(par, val_off, lo, hi, nullptr))
					return par;
			}
		}
		return nullptr;
	}

	bool Discover() {
		Span img = {};
		if (!ModuleSpan(GetModuleHandleA("client.dll"), &img)) {
			_snprintf_s(g_status, _TRUNCATE, "cvars: client.dll not mapped");
			Log(g_status);
			return false;
		}

		char line[256];

		// Image scans once per probe name (the expensive part)...
		static SlotSet sets[kProbeCount];
		for (int c = 0; c < kProbeCount; ++c) {
			FindNameSlots(img, kProbes[c].name, &sets[c]);
			_snprintf_s(line, _TRUNCATE, "cvar probe: %s -> %d candidate slot(s)",
				kProbes[c].name, sets[c].n);
			Log(line);
			if (!sets[c].n) {
				_snprintf_s(g_status, _TRUNCATE,
					"cvars: no reference to \"%s\" found in client.dll", kProbes[c].name);
				Log(g_status);
				return false;
			}
		}

		// ...then the cheap layout search over the cached slots. Every
		// combination must validate on ALL probe cvars to be trusted.
		for (int name_off = 8; name_off <= 0x28; name_off += 8) {
			for (int val_off = 0x20; val_off <= 0xA0; val_off += 4) {
				const uint8_t* objs[kProbeCount] = {};
				bool all = true;
				for (int c = 0; c < kProbeCount; ++c) {
					objs[c] = ResolveFromSlots(img, sets[c],
						kProbes[c].lo, kProbes[c].hi, name_off, val_off);
					if (!objs[c]) { all = false; break; }
				}
				if (!all)
					continue;

				g_name_off = name_off;
				g_val_off = val_off;
				g_found_count = 0;
				float grav = 0.f;
				for (int c = 0; c < kProbeCount; ++c) {
					Found& fd = g_found[g_found_count++];
					strcpy_s(fd.name, kProbes[c].name);
					fd.obj = objs[c];
					float v = 0.f;
					ValuePairAt(fd.obj, g_val_off, -1e6f, 1e6f, &v);
					if (c == 0)
						grav = v;
					_snprintf_s(line, _TRUNCATE,
						"cvar probe: %s obj=%p name_off=0x%X val_off=0x%X value=%.2f",
						fd.name, static_cast<const void*>(fd.obj), name_off, val_off, v);
					Log(line);
				}
				_snprintf_s(g_status, _TRUNCATE,
					"cvars: live reads OK (name+0x%X value+0x%X, sv_gravity %.0f)",
					name_off, val_off, grav);
				Log(g_status);
				return true;
			}
		}

		_snprintf_s(g_status, _TRUNCATE,
			"cvars: layout did not validate on this client.dll - live reads unavailable");
		Log(g_status);
		return false;
	}

}   // namespace

namespace Cvars {

	bool Init() {
		if (g_tried)
			return g_ready;
		g_tried = true;
		g_ready = Discover();
		return g_ready;
	}

	bool GetFloat(const char* name, float* out) {
		if (!Init())
			return false;
		for (int i = 0; i < g_found_count; ++i)
			if (_stricmp(g_found[i].name, name) == 0)
				return ReadLive(g_found[i].obj, name, g_name_off, g_val_off, out);
		// Not one of the probe cvars: resolve on demand with the validated
		// layout (wide plausibility - the layout itself is already proven).
		Span img = {};
		if (!ModuleSpan(GetModuleHandleA("client.dll"), &img))
			return false;
		SlotSet set;
		FindNameSlots(img, name, &set);
		const uint8_t* obj = ResolveFromSlots(img, set, -1e6f, 1e6f, g_name_off, g_val_off);
		if (!obj)
			return false;
		if (g_found_count < static_cast<int>(sizeof(g_found) / sizeof(g_found[0]))) {
			Found& fd = g_found[g_found_count++];
			strncpy_s(fd.name, name, _TRUNCATE);
			fd.obj = obj;
		}
		return ReadLive(obj, name, g_name_off, g_val_off, out);
	}

	const char* Status() {
		return g_status;
	}

}   // namespace Cvars
