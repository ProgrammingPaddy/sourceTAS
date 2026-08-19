#pragma once

// Read-only .tas (STAS v1..v3) consumer for the solver core, mirroring
// Source/Recording/RecordingStore.cpp byte for byte. The DLL remains the
// format's owner; this reader must never diverge from it.

#include "SolverMath.h"

#include <string>
#include <vector>

namespace Solver {

	struct TapeAnchor {
		bool valid = false;
		Vec3 origin;
		Vec3 velocity;
		float pitch = 0.f, yaw = 0.f;
		bool ducked = false;
		float stamina = 0.f;
	};

	struct TapeFrame {
		float pitch = 0.f, yaw = 0.f;
		float fmove = 0.f, smove = 0.f, umove = 0.f;
		int buttons = 0;
	};

	struct Tape {
		TapeAnchor start;
		std::string map;
		std::vector<TapeFrame> frames;      // all segments flattened
		std::vector<int> segment_starts;    // frame index where each segment begins
		int version = 0;
	};

	bool LoadTas(const std::string& path, Tape& out, std::string* err);

	// Writes a STAS v3 file (one segment) the game loads like any recording -
	// solver output goes straight into the in-game library for test play.
	bool WriteTas(const std::string& path, const TapeAnchor& anchor,
	              const std::string& map, const std::vector<TapeFrame>& frames,
	              std::string* err);

} // namespace Solver
