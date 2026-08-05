#pragma once

// Live server-cvar reads WITHOUT calling into the engine: replicated ConVar
// objects (sv_airaccelerate, sv_gravity, ...) live as static objects in
// client.dll's data section, and the server writes the live values into them
// on connect/change. Discovery is pure data walking in the netvar-walk
// spirit - find the name string, find the object pointing at it, validate the
// value layout on MULTIPLE cvars before trusting it - every read SEH-guarded,
// every step logged to calibration\cvar_probe.log. No vtable calls, no
// assumed offsets: an offset is only used after it validates on live data.
namespace Cvars {

	// Discover the ConVar layout (idempotent; cheap after the first call).
	// Returns true when reads are available.
	bool Init();

	// Read a cvar's current float value. Lazily runs Init(). Returns false if
	// discovery failed or this cvar can't be located/validated.
	bool GetFloat(const char* name, float* out);

	// One-line, UI-ready summary of what discovery found (or why it failed).
	const char* Status();

}   // namespace Cvars
