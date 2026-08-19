#pragma once

// Server-settings adaptation for the solver core. NOTHING physics-shaped is
// compile-time-fixed: the DLL snapshots the LIVE server state into
// Documents\sourceTAS\solver\server_params.cfg with every Map Solve export,
// and every harness command loads it. Precedence: built-in defaults < params
// file < explicit CLI flags - and every load prints where each value came
// from, never silently.
//
// File format: "key value" per line, '#' starts a comment. Keys map onto
// MoveParams/Hulls fields; unknown keys are reported, not fatal.

#include "SolverMove.h"
#include "SolverTape.h"

#include <string>

namespace Solver {

	// Loads keys from a params file into p/h. Returns false only when the
	// file exists but is unreadable; a missing file is fine (miss=true).
	// `report` (optional) receives one line per applied key for the loud
	// print, plus notes for unknown keys.
	bool LoadParamsFile(const std::string& path, MoveParams& p, Hulls& h,
	                    bool* missing, std::string* report);

	// The ONE key->field mapping, shared by the cfg loader above and the
	// self-describing capture headers ("# param key value" lines). Returns
	// false for unknown keys.
	bool ApplyParamKey(MoveParams& p, Hulls& h, const std::string& key,
	                   float val);

	// Canonical location the DLL writes: Documents\sourceTAS\solver\
	// server_params.cfg ("" if the Documents folder can't resolve).
	std::string CanonicalParamsPath();

	// The solve anchor the Map Solve tab captures: the tick-0 state the
	// solver and the game agree on. Same directory, solve_anchor.cfg.
	std::string CanonicalAnchorPath();
	bool LoadAnchorFile(const std::string& path, TapeAnchor& out, bool* missing);

} // namespace Solver
