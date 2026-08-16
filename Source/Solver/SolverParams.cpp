#include "SolverParams.h"

#include <windows.h>
#include <shlobj.h>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>

#pragma comment(lib, "shell32.lib")

namespace Solver {

	std::string CanonicalParamsPath() {
		char documents[MAX_PATH];
		if (FAILED(SHGetFolderPathA(nullptr, CSIDL_PERSONAL, nullptr,
			SHGFP_TYPE_CURRENT, documents)))
			return {};
		return std::string(documents) + "\\sourceTAS\\solver\\server_params.cfg";
	}

	std::string CanonicalAnchorPath() {
		char documents[MAX_PATH];
		if (FAILED(SHGetFolderPathA(nullptr, CSIDL_PERSONAL, nullptr,
			SHGFP_TYPE_CURRENT, documents)))
			return {};
		return std::string(documents) + "\\sourceTAS\\solver\\solve_anchor.cfg";
	}

	bool LoadAnchorFile(const std::string& path, TapeAnchor& out, bool* missing) {
		if (missing) *missing = false;
		std::ifstream in(path);
		if (!in) {
			if (missing) *missing = true;
			return false;
		}
		TapeAnchor a;
		bool have_origin = false;
		std::string line;
		while (std::getline(in, line)) {
			int d = 0;
			if (sscanf_s(line.c_str(), "origin %f %f %f",
				&a.origin.X, &a.origin.Y, &a.origin.Z) == 3)
				have_origin = true;
			else if (sscanf_s(line.c_str(), "velocity %f %f %f",
				&a.velocity.X, &a.velocity.Y, &a.velocity.Z) == 3) {}
			else if (sscanf_s(line.c_str(), "pitch %f", &a.pitch) == 1) {}
			else if (sscanf_s(line.c_str(), "yaw %f", &a.yaw) == 1) {}
			else if (sscanf_s(line.c_str(), "ducked %d", &d) == 1)
				a.ducked = d != 0;
			else if (sscanf_s(line.c_str(), "stamina %f", &a.stamina) == 1) {}
		}
		if (!have_origin)
			return false;
		a.valid = true;
		out = a;
		return true;
	}

	bool LoadParamsFile(const std::string& path, MoveParams& p, Hulls& h,
	                    bool* missing, std::string* report) {
		if (missing) *missing = false;
		std::ifstream in(path);
		if (!in) {
			if (missing) *missing = true;
			return true;   // absent file = defaults, not an error
		}
		std::string line;
		int applied = 0;
		while (std::getline(in, line)) {
			const size_t hash = line.find('#');
			if (hash != std::string::npos)
				line.resize(hash);
			std::istringstream ls(line);
			std::string key;
			float val = 0.f;
			if (!(ls >> key >> val))
				continue;
			const bool known = ApplyParamKey(p, h, key, val);
			if (known)
				applied++;
			if (report) {
				char buf[128];
				if (known)
					snprintf(buf, sizeof(buf), "  %s = %g\n", key.c_str(), val);
				else
					snprintf(buf, sizeof(buf), "  (unknown key '%s' ignored)\n",
						key.c_str());
				*report += buf;
			}
		}
		(void)applied;
		return true;
	}

	// ONE key->field mapping shared by server_params.cfg and the capture
	// headers ("# param key value" - user directive 2026-08-15: captures
	// are self-describing so consumers adapt to any server setting change).
	bool ApplyParamKey(MoveParams& p, Hulls& h, const std::string& key,
	                   float val) {
		bool known = true;
		if (key == "tickinterval") p.dt = val;
		else if (key == "gravity") p.gravity = val;
		else if (key == "accelerate") p.accelerate = val;
		else if (key == "airaccelerate") p.airaccelerate = val;
		else if (key == "friction") p.friction = val;
		else if (key == "stopspeed") p.stopspeed = val;
		else if (key == "maxspeed") p.maxspeed = val;
		else if (key == "maxvelocity") p.maxvelocity = val;
		else if (key == "stepsize") p.stepsize = val;
		else if (key == "air_speed_cap") p.air_speed_cap = val;
		else if (key == "jump_impulse") p.jump_impulse_d = val;
		else if (key == "enablebunnyhopping") p.enablebunnyhopping = val != 0.f;
		else if (key == "autobunnyhopping") p.autobunnyhopping = val != 0.f;
		else if (key == "strafe_rate_max") p.strafe_rate_max = val;
		else if (key == "duck_air_shift") p.duck_air_shift = val;
		else if (key == "duck_speed_frac") p.duck_speed_frac = val;
		else if (key == "time_to_duck_ms") p.time_to_duck_ms = val;
		else if (key == "stamina_jump_ms") p.stamina_jump_ms = val;
		else if (key == "stamina_scale_per_ms") p.stamina_scale_per_ms = val;
		else if (key == "stamina_pow_rate") p.stamina_pow_rate = val;
		else if (key == "hull_stand") h.stand_max.Z = val;
		else if (key == "hull_duck") h.duck_max.Z = val;
		else known = false;
		return known;
	}

} // namespace Solver
