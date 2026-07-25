#include "TasEditor.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <utility>
#include <vector>
#include <windows.h>
#include <shlobj.h>

#include <cstrike/sdk.h>
#include <imgui/imgui.h>
#include <imgui/imgui_internal.h>   // window-stack rebalance after a UI fault

#include "../World/WorldDraw.h"
#include "../World/BspWorld.h"

namespace {
	constexpr float kPi = 3.14159265358979f;
	float Deg(float r) { return r * (180.f / kPi); }
	float NormYaw(float y) {
		while (y > 180.f) y -= 360.f;
		while (y < -180.f) y += 360.f;
		return y;
	}
	float Speed2D(const Vector& v) { return sqrtf(v.X * v.X + v.Y * v.Y); }

	// --------------------------------------------------------------- widgets --
	// Every tunable value = a LONG slider (coarse feel) + a numeric box with
	// +-step arrows (precision), both bound to the same variable.
	constexpr float kSliderW = 520.f;
	constexpr float kInputW = 110.f;

	bool FloatRow(const char* label, float* v, float lo, float hi,
	              const char* fmt, float step = 0.1f, float fast = 1.f, int decimals = 2) {
		bool changed = false;
		ImGui::PushID(label);
		ImGui::PushItemWidth(kSliderW);
		changed |= ImGui::SliderFloat("##s", v, lo, hi, fmt);
		ImGui::PopItemWidth();
		ImGui::SameLine();
		ImGui::PushItemWidth(kInputW);
		// Fine adjustment: two-decimal rows step 0.01 on the arrows (the
		// slider keeps its coarse feel; the arrows are the precision path).
		const float arrow = (decimals >= 2 && step > 0.011f) ? 0.01f : step;
		changed |= ImGui::InputFloat("##i", v, arrow, fast, decimals);
		ImGui::PopItemWidth();
		ImGui::SameLine();
		if (ImGui::Button("0##z")) {
			*v = 0.f;
			changed = true;
		}
		ImGui::SameLine();
		ImGui::Text("%s", label);
		ImGui::PopID();
		if (*v < lo) *v = lo;
		if (*v > hi) *v = hi;
		return changed;
	}

	bool IntRow(const char* label, int* v, int lo, int hi) {
		bool changed = false;
		ImGui::PushID(label);
		ImGui::PushItemWidth(kSliderW);
		changed |= ImGui::SliderInt("##s", v, lo, hi);
		ImGui::PopItemWidth();
		ImGui::SameLine();
		ImGui::PushItemWidth(kInputW);
		changed |= ImGui::InputInt("##i", v, 1, 10);
		ImGui::PopItemWidth();
		ImGui::SameLine();
		ImGui::Text("%s", label);
		ImGui::PopID();
		if (*v < lo) *v = lo;
		if (*v > hi) *v = hi;
		return changed;
	}

	// ---------------------------------------------------------------- model --
	// One segment type: held keyboard keys + a view that either holds or turns
	// at a fixed rate. While airborne and turning, the strafe key (A/D) is
	// auto-selected from the turn direction unless A or D is explicitly held.
	enum PitchMode { PM_Const = 0, PM_Follow };
	enum JumpMode  { JM_None = 0, JM_Hold, JM_AutoBhop };

	struct GenParams {
		int   ticks = 66;
		bool  key_w = true, key_a = false, key_s = false, key_d = false;
		bool  auto_key = true;        // A/D from view turn direction (airborne)
		bool  no_w_air = true;        // release W while airborne (surf default)
		int   yaw_mode = 1;           // 1 = turn at yaw_rate (0 = legacy hold);
		                              // 2 = OPTIMAL strafe: yaw follows the velocity
		                              //     heading (max gain), yaw_rate = bias deg
		                              // 3 = STRAIGHT SYNC: alternating optimal strafes
		                              //     every opt_period ticks; the net path holds
		                              //     its heading, yaw_rate = steer bias deg
		float yaw_rate = 0.f;         // deg/tick, screen sign (+ = right); mode 2/3: bias
		int   opt_dir = 1;            // mode 2 direction / mode 3 first side (+1 A, -1 D)
		int   opt_period = 8;         // mode 3: ticks per strafe side
		bool  yaw_abs = false;
		float yaw_start = 0.f;
		int   pitch_mode = PM_Const;
		float pitch_val = 0.f;
		float pitch_mult = 1.f;
		bool  duck = false;
		int   jump_mode = JM_None;
	};

	// A solver segment's applied solution: alternating strafes routed to a board
	// target. rates are deg/tick in screen sign; side0 is the first strafe's key
	// (+1 = A/left, -1 = D/right); splits are local transition ticks.
	struct SolverData {
		int   target = -1;            // board target index (used as a pure point + yaw)
		int   strafes = 3;            // alternating strafe count (1..4); 3 = exactly determined
		bool  start_jump = false;     // jump on the segment's first tick (grounded start)
		bool  opt = false;            // optimal-rate mode: rates[] hold per-phase BIASES
		                              // off the max-gain line
		int   blend = 8;              // opt: view blends onto the target yaw over the
		                              // last N ticks (max gain until then, yaw exact at
		                              // the pass - welding view to heading all the way
		                              // cost ~65 u/s vs the free-yaw runs)
		float tyaw = 0.f;             // opt: target yaw snapshot for the provider's blend
		bool  solved = false;
		int   ticks = 0;              // N: the pass is inside the segment's final tick
		int   side0 = 1;
		int   nsplits = 0;
		int   splits[3] = {};
		float rates[4] = {};
		float pass_frac = 1.f;        // fraction into the final tick where z hits target z
		// The model's pass point for the applied solution - compared against
		// the REAL sim's measured pass to expose any model drift.
		Vector model_pass = Vector(0.f, 0.f, 0.f);
		float  model_err = 0.f;
	};

	struct EditSegment {
		bool raw = false;             // raw = explicit per-tick frames
		bool is_solver = false;       // solver-driven strafe route
		GenParams gen;                // pitch/duck settings shared by all kinds
		SolverData solver;
		std::vector<Frame> frames;
		// Per-tick jump overrides for generated segments: (local tick, 0=suppress
		// / 1=force). Lets a single tick jump without baking the segment.
		std::vector<std::pair<int, int>> jump_ovr;
		int Ticks() const {
			if (raw) return static_cast<int>(frames.size());
			if (is_solver) return solver.solved ? solver.ticks : 0;
			return gen.ticks;
		}
	};

	// Truncating formatter for UI-facing strings (labels, status rows) where
	// clipping is cosmetic: sprintf_s KILLS THE PROCESS when the output
	// outgrows the buffer (two shipped crashes came from exactly that).
	template <size_t N>
	void Sfmt(char (&dst)[N], const char* fmt, ...) {
		va_list args;
		va_start(args, fmt);
		_vsnprintf_s(dst, N, _TRUNCATE, fmt, args);
		va_end(args);
	}

	// EXACT formatter for DATA (logs, calibration lines): measures the needed
	// size and allocates it - no fixed buffer, no overflow, no truncation.
	std::string FmtV(const char* fmt, va_list args) {
		va_list copy;
		va_copy(copy, args);
		const int need = _vscprintf(fmt, copy);
		va_end(copy);
		if (need <= 0)
			return std::string();
		std::string s(static_cast<size_t>(need), '\0');
		vsnprintf(&s[0], s.size() + 1, fmt, args);
		return s;
	}
	std::string FmtStr(const char* fmt, ...) {
		va_list args;
		va_start(args, fmt);
		std::string s = FmtV(fmt, args);
		va_end(args);
		return s;
	}

	// ---------------------------------------------------------------- state --
	bool g_open = true;   // the editor IS the tool - visible whenever the menu is
	int  g_tab = 0;                   // 0 Run, 1 Targets, 2 Rendering, 3 Project
	char g_name[64] = "untitled";
	StartState g_anchor;
	std::vector<EditSegment> g_segs;
	int g_sel = -1;                   // selected segment
	int g_cursor = 0;                 // playhead tick
	bool g_show_hull = true;          // ghost hull at the playhead
	std::string g_status = "No segments. Capture an anchor and add one.";

	// Picking (Targets tab).
	int g_pick_mode = 0;              // 0 = tag surf face, 1 = place board target
	int g_sel_target = -1;
	int g_sel_tag = -1;
	char g_pick_info[512] = {};       // last pick forensics (ray + hit)

	// Air-strafe model (match server settings; defaults = stock CS:S) + the
	// visual "optimal enough" threshold used by the line coloring.
	float g_air_cap = 30.f;
	float g_air_accel = 10.f;
	float g_wishspeed = 250.f;
	float g_gravity = 800.f;
	float g_min_eff = 0.98f;
	float g_sol_pos_tol = 1.f;        // "exact" pass-distance tag (u)
	float g_sol_max_bump = 10.f;      // max clip for the "exact" tag (u/s)
	float g_sol_collect = 8.f;        // COLLECT everything passing within this (calibration data)

	// Engine probe values harvested from every landed sim's states - DISPLAY
	// and explicit one-click adoption only, never silently used (the model
	// runs exclusively on the visible inputs).
	float g_meas_max_add = 0.f;       // largest per-tick accel the engine granted
	float g_meas_grav = 0.f;          // latest gravity fit
	int   g_meas_grav_n = 0;
	float g_meas_jump_vz = 0.f;       // jump velocity reproducing the measured
	                                  // first-tick vz (captures stamina exactly)
	float g_jump_vz = 0.f;            // model jump velocity input (0 = sqrt(2g*57))

	// Layout (recomputed on any edit) + sim caches (filled when a sim lands).
	int g_total = 0;
	std::vector<int> g_starts;
	bool g_valid = false;
	std::vector<Frame> g_frames;
	std::vector<Prediction::SimState> g_states;
	std::vector<float> g_speed, g_gain, g_maxgain, g_eff;

	bool g_dirty = false;
	unsigned long long g_dirty_ms = 0;
	bool g_sim_requested = false;
	bool g_sim_fault = false;

	// Project file list (Documents\sourceTAS\projects).
	std::vector<std::string> g_files;
	int g_sel_file = -1;

	// ------------------------------------------------------------------ plan --
	// Immutable copy of the compiled plan, consumed by the provider on the game
	// thread (never mutated while a sim is in flight).
	constexpr int kMaxPlanSegs = 64;
	constexpr int kMaxPlanOvr = 1024;
	struct PlanSeg {
		int start; int ticks; bool raw; int raw_off; GenParams gen;
		bool solver; SolverData sdata;
	};
	struct PlanOvr { int seg; int local; int value; };
	PlanSeg g_plan[kMaxPlanSegs];
	int     g_plan_count = 0;
	PlanOvr g_plan_ovr[kMaxPlanOvr];
	int     g_plan_ovr_count = 0;
	Frame   g_plan_raw[Prediction::kMaxSimTicks];
	float   g_plan_anchor_yaw = 0.f;
	float   g_plan_cap = 30.f;
	float   g_plan_accel = 37.5f;     // airaccel * wishspeed * interval

	// Provider walk state (reset at tick 0; game thread only).
	int   g_pv_seg = 0;
	float g_pv_last_yaw = 0.f;
	float g_pv_entry_yaw = 0.f;
	bool  g_pv_prev_jump = false;

	void Layout() {
		g_starts.clear();
		g_total = 0;
		for (const EditSegment& s : g_segs) {
			g_starts.push_back(g_total);
			g_total += s.Ticks();
		}
		if (g_total > Prediction::kMaxSimTicks)
			g_total = Prediction::kMaxSimTicks;
		if (g_cursor >= g_total)
			g_cursor = g_total > 0 ? g_total - 1 : 0;
		if (g_cursor < 0)
			g_cursor = 0;
	}

	void MarkDirty() {
		Layout();
		g_dirty = true;
		g_dirty_ms = GetTickCount64();
	}

	// Max possible speed gain for one air tick at 2D speed `speed` (see the
	// prediction-re notes for the derivation). Used by the efficiency coloring
	// and diagnostics.
	float MaxAirGain(float speed) {
		const float L = g_plan_cap;
		const float a = g_plan_accel;
		float cstar = L - a;
		if (cstar < 0.f) cstar = 0.f;
		float add = L - cstar;
		if (add > a) add = a;
		if (add < 0.f) add = 0.f;
		return sqrtf(speed * speed + 2.f * cstar * add + add * add) - speed;
	}

	int FindJumpOverride(int seg, int local) {
		for (int i = 0; i < g_plan_ovr_count; ++i)
			if (g_plan_ovr[i].seg == seg && g_plan_ovr[i].local == local)
				return g_plan_ovr[i].value;
		return -1;
	}

	// Horizontal velocity heading in degrees (Source yaw space); falls back
	// when there is no meaningful horizontal speed. The OPTIMAL strafe yaw is
	// heading + side*bias: wish exactly perpendicular to the velocity is the
	// max per-tick gain whenever accel*wishspeed*dt >= the 30 air cap
	// (gain^2 = cap^2 - (v*sin bias)^2 - bias trades speed QUADRATICALLY for
	// LINEARLY more or less curvature, so slightly-biased strafes stay near
	// optimal). Model and provider both evaluate this exact expression on
	// their own velocity; model == engine makes the yaw streams identical.
	float HeadingDeg(const Vector& v, float fallback) {
		if (v.X * v.X + v.Y * v.Y < 1.f)
			return fallback;
		return NormYaw(atan2f(v.Y, v.X) * (180.f / 3.14159265358979f));
	}

	// Per-tick view yaw for OPTIMAL solver mode: ride the max-gain line
	// (heading + side*bias) until the BLEND window, then sweep the view onto
	// the target yaw with a telescoping step that lands EXACTLY on it at
	// tick N and holds it through the overrun. Welding the view to the
	// heading for the whole last phase (the old eliminated-bias scheme) cost
	// ~65 u/s - the blend confines the off-optimal ticks to a short tail.
	// Deterministic in (prev yaw, prev velocity, t): the model and the
	// provider evaluate the identical expression on identical state.
	float OptYawStep(float prev_yaw, const Vector& prev_vel, int t, int N,
	                 int blend, float bias, int side, float target_yaw) {
		if (t >= N)
			return target_yaw;
		if (t >= N - blend) {
			const int rem = N - t;
			return NormYaw(prev_yaw - NormYaw(prev_yaw - target_yaw) / static_cast<float>(rem));
		}
		return NormYaw(HeadingDeg(prev_vel, prev_yaw) + static_cast<float>(side) * bias);
	}

	// Closed-loop frame generation: called by the sim (game thread) per tick,
	// with the simulated state after the previous tick.
	void Provider(int tick, const Prediction::SimState& prev, Frame* out) {
		if (tick == 0) {
			g_pv_seg = 0;
			g_pv_last_yaw = g_plan_anchor_yaw;
			g_pv_entry_yaw = g_plan_anchor_yaw;
			g_pv_prev_jump = false;
		}
		while (g_pv_seg + 1 < g_plan_count && tick >= g_plan[g_pv_seg + 1].start)
			g_pv_seg++;
		const PlanSeg& ps = g_plan[g_pv_seg];
		const int t = tick - ps.start;

		if (ps.raw) {
			// Clamp for overrun ticks (defensive; overrun only extends solver ends).
			const int tc = (t < ps.ticks) ? t : (ps.ticks > 0 ? ps.ticks - 1 : 0);
			*out = g_plan_raw[ps.raw_off + tc];
			g_pv_last_yaw = out->viewangles[1];
			g_pv_prev_jump = (out->buttons & IN_JUMP) != 0;
			return;
		}

		const GenParams& g = ps.gen;
		const bool onground = (prev.flags & FL_ONGROUND) != 0;

		if (t == 0)
			g_pv_entry_yaw = g.yaw_abs ? g.yaw_start : g_pv_last_yaw;

		float yaw = g_pv_entry_yaw;
		float fmove = 0.f;
		float smove = 0.f;

		if (ps.solver) {
			// Solver segment: alternating-strafe schedule from the applied
			// solution, only the side key held (W never). Constant mode: the
			// view turns at the phase's rate - a pure input stream. Optimal
			// mode: rates[] are per-phase BIASES and the yaw follows the
			// SIMULATED velocity heading + side*bias - the exact expression
			// the model evaluated on its own (identical) velocity, so the yaw
			// streams still match. Overrun ticks hold the final phase.
			const SolverData& sd = ps.sdata;
			int k = 0;
			while (k < sd.nsplits && t >= sd.splits[k]) k++;
			const int side = (k % 2 == 0) ? sd.side0 : -sd.side0;
			if (sd.opt)
				yaw = OptYawStep(g_pv_last_yaw, prev.velocity, t, sd.ticks,
					(sd.blend < 1) ? 1 : sd.blend, sd.rates[k], side, sd.tyaw);
			else
				yaw = NormYaw(g_pv_last_yaw - sd.rates[k]);
			smove = (side > 0) ? -450.f : 450.f;   // +1 = A (left), -1 = D (right)
		} else {
			// View: turn at a fixed rate (screen sign: + = right = Source yaw
			// decrease), or OPTIMAL strafe: yaw = velocity heading + dir*bias
			// (wish exactly perpendicular to the velocity at bias 0 = max
			// per-tick gain; the bias trades speed quadratically for linearly
			// more/less curvature - the "wrangle" control).
			if (g.yaw_mode == 1)
				yaw = g_pv_entry_yaw - g.yaw_rate * static_cast<float>(t);
			else if (g.yaw_mode == 2)
				yaw = HeadingDeg(prev.velocity, g_pv_last_yaw)
					+ static_cast<float>(g.opt_dir) * g.yaw_rate;
			else if (g.yaw_mode == 3) {
				// STRAIGHT SYNC: optimal wish (perpendicular to the velocity)
				// with the side alternating every opt_period ticks - the half-
				// curls cancel and the net path holds its heading. The steer
				// bias tilts the wish the SAME direction on both sides, which
				// makes one half-curl stronger than the other: the mean path
				// bends while every tick stays on the biased-optimal line.
				yaw = HeadingDeg(prev.velocity, g_pv_last_yaw) + g.yaw_rate;
			}
			yaw = NormYaw(yaw);

			// Held keys. W releases while airborne by default (surf): the forward
			// wish is useless in air and fights the strafe.
			fmove = (g.key_w ? 450.f : 0.f) + (g.key_s ? -450.f : 0.f);
			smove = (g.key_d ? 450.f : 0.f) + (g.key_a ? -450.f : 0.f);
			if (g.no_w_air && g.key_w && !onground)
				fmove -= 450.f;

			// Auto strafe key: while airborne and turning, hold the A/D matching the
			// turn direction - unless A or D is explicitly held (the override).
			const bool turning = (g.yaw_mode == 1) && fabsf(g.yaw_rate) > 0.01f;
			if (g.auto_key && turning && !onground && !g.key_a && !g.key_d)
				smove = (g.yaw_rate > 0.f) ? 450.f : -450.f;   // right turn -> D, left -> A

			// Optimal modes drive the strafe key themselves - the side IS the
			// mechanic. Mode 3 alternates it every opt_period ticks.
			if (g.yaw_mode == 2)
				smove = (g.opt_dir > 0) ? -450.f : 450.f;
			else if (g.yaw_mode == 3) {
				const int per = (g.opt_period < 1) ? 1 : g.opt_period;
				const int side = (((t / per) % 2) == 0) ? g.opt_dir : -g.opt_dir;
				smove = (side > 0) ? -450.f : 450.f;
			}
		}

		float pitch = g.pitch_val;
		if (g.pitch_mode == PM_Follow) {
			const float v2 = Speed2D(prev.velocity);
			pitch = (v2 > 1.f || fabsf(prev.velocity.Z) > 1.f)
				? -Deg(atan2f(prev.velocity.Z, v2)) * g.pitch_mult : 0.f;
			if (pitch > 89.f) pitch = 89.f;
			if (pitch < -89.f) pitch = -89.f;
		}

		int buttons = 0;
		if (g.duck) buttons |= IN_DUCK;
		if (g.jump_mode == JM_Hold)
			buttons |= IN_JUMP;
		else if (g.jump_mode == JM_AutoBhop && onground && !g_pv_prev_jump)
			buttons |= IN_JUMP;   // press only on ticks that start grounded
		if (ps.solver && ps.sdata.start_jump && t == 0)
			buttons |= IN_JUMP;   // solver's grounded-start hop

		// Per-tick override: force or suppress jump on exactly this tick.
		const int ovr = FindJumpOverride(g_pv_seg, t);
		if (ovr == 1) buttons |= IN_JUMP;
		else if (ovr == 0) buttons &= ~IN_JUMP;

		out->viewangles[0] = pitch;
		out->viewangles[1] = yaw;
		out->forwardmove = fmove;
		out->sidemove = smove;
		out->upmove = 0.f;
		out->buttons = buttons;
		out->impulse = 0;
		out->mousedx = 0;
		out->mousedy = 0;

		g_pv_last_yaw = yaw;
		g_pv_prev_jump = (buttons & IN_JUMP) != 0;
	}

	bool BuildPlan() {
		g_plan_count = 0;
		g_plan_ovr_count = 0;
		int raw_off = 0;
		int start = 0;
		for (const EditSegment& s : g_segs) {
			if (g_plan_count >= kMaxPlanSegs || start >= Prediction::kMaxSimTicks)
				break;
			PlanSeg& p = g_plan[g_plan_count];
			p.start = start;
			p.raw = s.raw;
			p.gen = s.gen;
			p.raw_off = raw_off;
			p.solver = s.is_solver && s.solver.solved;
			p.sdata = s.solver;

			int ticks = s.Ticks();
			if (start + ticks > Prediction::kMaxSimTicks)
				ticks = Prediction::kMaxSimTicks - start;
			if (s.raw && ticks > 0) {
				memcpy(&g_plan_raw[raw_off], s.frames.data(), sizeof(Frame) * ticks);
				raw_off += ticks;
			}
			if (!s.raw) {
				for (const std::pair<int, int>& o : s.jump_ovr)
					if (o.first < ticks && g_plan_ovr_count < kMaxPlanOvr)
						g_plan_ovr[g_plan_ovr_count++] = { g_plan_count, o.first, o.second };
			}
			p.ticks = ticks;
			start += ticks;
			g_plan_count++;
		}

		g_plan_anchor_yaw = g_anchor.yaw;
		float interval = Prediction::LastDiag().interval_per_tick;
		if (interval <= 0.f) interval = 0.015f;
		g_plan_cap = g_air_cap;
		g_plan_accel = g_air_accel * g_wishspeed * interval;
		return start > 0;
	}

	void RecomputeDiag() {
		const int n = static_cast<int>(g_states.size());
		g_speed.assign(n, 0.f);
		g_gain.assign(n, 0.f);
		g_maxgain.assign(n, 0.f);
		g_eff.assign(n, 0.f);

		// The efficiency coloring must always score against the CURRENT
		// engine constants - the same cap/accel inputs the solver runs with -
		// not whatever the last plan build happened to see. MaxAirGain is the
		// exact per-tick optimum of AirAccelerate: gain = sqrt(v^2 + 2 c* a'
		// + a'^2) - v with c* = max(cap - amt, 0), a' = min(cap - c*, amt).
		float interval = Prediction::LastDiag().interval_per_tick;
		if (interval <= 0.f) interval = 0.015f;
		g_plan_cap = g_air_cap;
		g_plan_accel = g_air_accel * g_wishspeed * interval;

		float prev_speed = Speed2D(g_anchor.velocity);
		for (int i = 0; i < n; ++i) {
			const float sp = Speed2D(g_states[i].velocity);
			g_speed[i] = sp;
			g_gain[i] = sp - prev_speed;
			const bool air = (i > 0) && ((g_states[i - 1].flags & FL_ONGROUND) == 0);
			if (air) {
				g_maxgain[i] = MaxAirGain(prev_speed);
				if (g_maxgain[i] > 0.001f)
					g_eff[i] = g_gain[i] / g_maxgain[i];
			}
			prev_speed = sp;
		}
	}

	int SegmentAtTick(int tick) {
		int seg = 0;
		for (int k = 0; k < static_cast<int>(g_starts.size()); ++k)
			if (g_starts[k] <= tick)
				seg = k;
		return g_starts.empty() ? -1 : seg;
	}

	// ============================================================= solver v2 --
	// REBUILT FROM SCRATCH. The one and only contract: find alternating A/D
	// strafe schedules whose path passes through the EXACT target point, with
	// the view yaw at that moment EXACTLY the target's set yaw. No board or
	// face logic anywhere - the target is a pure point + yaw.
	//
	// Structure of the problem:
	//   - Z is ballistic and strafe-independent, so the fractional tick t*
	//     where the path crosses the target's HEIGHT is computed in closed
	//     form per crossing - never searched.
	//   - View yaw is an INPUT stream, not physics: final yaw = start_yaw -
	//     sum(n_k * r_k) is LINEAR in the rates. The last rate is eliminated
	//     algebraically, and since the provider replays the identical stream,
	//     yaw exactness in the real sim is BY CONSTRUCTION.
	//   - The remaining rates solve the 2D pass-point residual XY(t*) ==
	//     target_XY by damped Newton. 3 strafes = exactly determined (2 eqs /
	//     2 free rates); 2 strafes = 1 free rate, exact only when an integer
	//     timing lines up; 4 strafes = a one-parameter family (first rate
	//     gridded, all kept, fastest first).
	//
	// The search runs on the fast model (validated offline: every accepted
	// solution re-simulates through the target exactly); the applied solution
	// runs through the REAL engine sim, whose pass distance is measured and,
	// if needed, closed with a small re-solve against the measured miss.
	float Clampf(float v, float lo, float hi) { return v < lo ? lo : v > hi ? hi : v; }
	Vector Scale(const Vector& v, float s) { return Vector(v.X * s, v.Y * s, v.Z * s); }
	float Dot3(const Vector& v) { return v.X * v.X + v.Y * v.Y + v.Z * v.Z; }
	float Dot(const Vector& a, const Vector& b) { return a.X * b.X + a.Y * b.Y + a.Z * b.Z; }

	struct RouteSolution {
		int   ticks = 0;              // N: the pass happens between ticks N-1 and N
		float frac = 0.f;             // fraction into that tick where z hits target z
		int   side0 = 1;              // first strafe key: +1 = A (left), -1 = D (right)
		int   nsplits = 0;
		int   splits[3] = {};         // local ticks where the strafe key flips
		float rates[4] = {};          // deg/tick per phase, screen sign (+ = right)
		int   wrap = 0;               // which 360-degree wrap of the yaw equation
		float pass_err = 0.f;         // model: |XY(t*) - target XY|
		float speed = 0.f;            // 2D speed at the pass
		Vector pass_pos;              // model pass point (XY at t*, target z)
		int   bump_ticks = 0;         // ramp kissed this many ticks before the pass
		float bump_loss = 0.f;        // speed the kiss clipped away (u/s)
		bool  exact = false;          // inside tolerance with a clean board
		// Filled by the automatic real-sim verification after the search:
		float real_err = -1.f;        // REAL pass distance (-1 = not verified yet)
		bool  real_on_target = false;
		int   real_tick = -1;
		float real_bump_loss = 0.f;
		float real_yaw_err = -1.f;    // REAL view-yaw miss at the pass (deg)
	};

	struct SearchCtx {
		bool active = false;
		bool done = false;
		int  seg = -1;
		// snapshot of inputs (immune to edits mid-search)
		Vector start_pos, start_vel;
		float  start_yaw = 0.f;
		Vector target_pos;
		float  target_yaw = 0.f;
		// The pass EVENT (model AND real measurement, symmetric): the CLOSEST
		// APPROACH of the collision-truncated path to the EXACT target point.
		// A perfect solution passes THROUGH the point (perigee -> 0), which is
		// also the instant the hull touches the face there (the support plane
		// runs through the target). The perigee moves SMOOTHLY with the rates -
		// unlike a first-contact event, whose position jumps from the graze
		// point to far down-slope the moment an arc stops touching (the cliff
		// that froze the search when the flight model became exact). Without a
		// face: the height crossing (fallback).
		bool   have_face = false;
		Vector face_n = Vector(0.f, 0.f, 1.f);
		float  face_d = 0.f;          // Dot(face_n, target_pos): support plane
		Vector e1, e2;                // residual basis (in-plane, or world X/Y)
		// COLLIDERS: the target's face plus every tagged board surface, each as
		// its hull-support plane + touch-corner offset + face polygon. The model
		// flight ENDS at the first on-face contact - exactly like the engine -
		// so candidates that would board early never masquerade as solutions.
		struct Collider {
			int brush = -1, plane = -1;
			Vector n;
			float d_origin = 0.f;     // plane the hull ORIGIN touches at contact
			Vector corner;            // origin -> touching corner offset
			bool is_target = false;
		};
		std::vector<Collider> colliders;   // per-face (real-pass measurement)
		// FULL-BRUSH collision world for the model: every solid brush along the
		// flight corridor (target's brush, tagged brushes, ramps, the valley
		// floor, walls - whatever TracePlayerBBox would hit), each as its
		// complete hull-expanded convex plane set. The hull origin sweeps
		// against these; the plane it ENTERS through decides board vs crash -
		// edges and back faces resolve exactly like the engine.
		struct BrushCollider {
			int  brush = -1;
			bool is_target = false;
			int  target_plane = -1;   // the boardable face when is_target
			Vector bmin, bmax;        // origin-space AABB (hull-expanded) gate
			std::vector<Vector> pn;   // plane normals
			std::vector<float>  pd;   // hull-expanded (origin) plane distances
			std::vector<int>    pid;  // original plane ids
		};
		std::vector<BrushCollider> bcolliders;
		// Warm starts per first-side: neighboring timings have neighboring
		// solutions, and canonical starts often sit in infeasible territory
		// (early-boarding curves) that the LM must otherwise crawl out of.
		bool   opt = false;           // optimal-rate mode: rates are biases (yaw exact
		                              // via the blend tail below)
		int    blend = 8;             // opt: view-blend window before the pass tick
		int    cap_dropped = 0;       // candidates lost to the collection backstop
		int    dedupe_kept = 0;       // distinct lines after dedupe (pre-trim)
		float  warm_fr[2][2] = {};
		bool   have_warm[2] = {};
		float  max_bump = 10.f;       // bump-loss gate for the "exact" tag (u/s)
		float  collect = 8.f;         // keep candidates passing within this (data!)
		int    early_hits = 0;        // flights deflected by a non-target corridor brush
		int    cnt_edge = 0;          // flights deflected by the target brush's other planes
		int    reject_retime = 0;     // hit the target but couldn't align the contact tick
		int    cnt_bump = 0;          // hit the target but the ramp kiss cost too much speed
		// Per-candidate outcome counters + the closest rejects with full detail,
		// for the search diagnostics log (the search is otherwise a black box).
		int    cnt_infeasible = 0;    // wrap/timing couldn't produce the yaw at all
		int    cnt_no_contact = 0;    // flight never contacted the target's face
		int    cnt_floor = 0;         // converged but above tolerance
		struct RejectRec {
			float err = 1e9f;
			int   N = 0, n_eff = 0, tick = 0, side0 = 0, wrap = 0;
			int   splits[3] = {};
			float rates[4] = {};
			Vector pass;
			int   reason = 0;          // 0 = position floor, 1 = tick align
		};
		std::vector<RejectRec> top_rejects;
		int    strafes = 3;
		float  dt = 0.015f, accel_amt = 37.5f, cap = 30.f, gravity = 800.f;
		float  tol = 1.0f;            // pass-distance acceptance gate (u)
		// z-crossing candidates (N, frac): computed once, then enumerated
		struct PassCand { int N; float frac; };
		std::vector<PassCand> cands;
		int cand_idx = 0;
		int side_idx = 0;
		std::vector<std::array<int, 3>> splits_list;
		int split_idx = 0;
		int splits_built_for = -1;
		// diagnostics for the "why empty" verdicts
		int jobs_done = 0, jobs_total = 0;
		float reject_best = 1e9f;     // closest exact-yaw pass that missed the gate
		bool  yaw_ever_feasible = false;
		int   jump_start = 0;         // whether the model start included the jump
		std::vector<RouteSolution> results;
	};
	SearchCtx g_search;
	int g_solution_pick = 0;

	// --------------------------------------------------- real pass analysis --
	// The drawn line is the REAL engine sim. After every landed sim, find
	// where ITS path crosses the target's height and measure the pass distance
	// and pass-tick view yaw - the ground truth the model's claim is checked
	// against. No face, no plane: pure point pass-through.
	struct RealPass {
		bool computed = false;        // a solved solver segment existed
		bool crossed = false;         // the real path reached a pass event
		bool on_target = true;        // ...on the TARGET's face (false = early board)
		int  hit_brush = -1, hit_plane = -1;   // which face was boarded
		int  tick = -1;               // global tick whose edge contains the pass
		int  bump_tick = -1;          // first target-face kiss before the pass
		int  bump_ticks = 0;
		float bump_loss = 0.f;        // speed the kiss clipped away (estimated)
		Vector pass;                  // measured real pass point (closest approach)
		float err = 0.f;              // |real pass - target|
		float yaw_err = 0.f;          // |pass-tick view yaw - target yaw| (should be ~0)
		float drift = 0.f;            // |real pass - model pass|
		float closest_dz = 1e9f;      // no-face fallback: nearest height approach
		float closest_phi = 1e9f;     // face: nearest approach to the support plane
		float min_dist = 1e9f;        // closest 3D approach of the line to the target
		bool  jump_missing = false;   // start_jump set but the sim never left the ground
		float real_vz0 = 0.f;         // vertical velocity after the segment's first tick
		float grav_fit = 0.f;         // engine gravity measured off the simmed states
		float max_add = 0.f;          // engine per-tick accel observed along the wish
		float cap_fit = 0.f;          // engine air-speed cap observed (cur + add)
		int   fit_samples = 0;
	};
	RealPass g_realpass;

	// -------------------------------------------------- real-sim correction --
	// The model is exact against itself; if the engine disagrees slightly, the
	// measured miss is fed straight back: re-solve the SAME candidate against
	// a virtual target shifted by the miss (secant), reapply, resim. Converges
	// in 1-2 rounds when the model is locally faithful; reports honestly when
	// it isn't. Yaw needs no correction - the input stream IS the yaw.
	struct CorrectCtx {
		bool auto_run = true;
		bool active = false, done = false;
		int  seg = -1;
		int  rounds = 0;
		int  skip = 0;                // sims in flight at start = stale, ignore
		Vector virt;                  // current virtual target (secant-shifted)
		float best_err = 1e9f;
		float best_rates[4] = {};     // rates that produced the best measurement
		bool  have_best = false;
		// Roomy: sprintf_s KILLS the process on overflow, and the early-board
		// message once outgrew a 160-byte buffer (the post-verify crash).
		char note[512] = {};
	};
	CorrectCtx g_correct;
	// Solution-slider settle: auto-correction fires only once the user stops
	// scrubbing (ApplySolution stays immediate for the line preview).
	bool g_pick_correct_pending = false;
	unsigned long long g_pick_settle_ms = 0;
	// Deferred teleport verification: measure where the player ACTUALLY landed
	// vs the anchor and report it - data instead of guessing at setpos lore.
	bool g_tp_check = false;
	unsigned long long g_tp_ms = 0;

	// --------------------------------------- calibration batch (TEMPORARY) --
	// Manual button while the model/engine gap gets tuned: runs EVERY found
	// solution through the real sim with its correction rounds and writes the
	// full story (model claim, each real measurement, each re-solve) to a log
	// file under Documents\sourceTAS\calibration - data instead of guesses.
	struct BatchCtx {
		bool active = false;
		int  seg = -1;
		int  idx = 0;                 // current solution
		int  round = 0;               // 0 = initial measurement, then corrections
		int  skip = 0;
		Vector virt;                  // secant-shifted virtual target
		std::string log;
		char path[MAX_PATH] = {};
	};
	BatchCtx g_batch;
	void FinishBatch(bool aborted);   // fwd (used by StartSearch)
	char g_searchlog_path[MAX_PATH] = {};
	void WriteSearchLog();            // fwd (search diagnostics, defined with the batch)

	// --------------------------------------- automatic real verification --
	// After every search, EVERY found solution gets one real-engine sim pass;
	// only solutions whose REAL line reaches the target survive into the list
	// (the user sees real-verified solutions only, with their real numbers).
	struct VerifyCtx {
		bool active = false;
		int  seg = -1;
		int  idx = 0;
		int  skip = 0;
		int  dropped = 0;
		float best_failed = 1e9f;     // closest real miss among dropped ones
	};
	VerifyCtx g_verify;
	void StartVerify(int seg_index); // fwd

	struct FastState { Vector p, v; float yaw; };

	// One airborne tick WITHOUT collision: set the tick's view yaw, half
	// gravity, air-accelerate along the side key's wishdir, integrate, half
	// gravity. (The collision-aware EngineMoveTick supersedes this for face
	// targets; this remains the no-face fallback.)
	void FastTick(FastState& s, float yaw_abs, int side) {
		s.yaw = yaw_abs;
		s.v.Z -= g_search.gravity * g_search.dt * 0.5f;
		const float wish = (s.yaw + static_cast<float>(side) * 90.f) * (kPi / 180.f);
		const float wx = cosf(wish), wy = sinf(wish);
		const float cur = s.v.X * wx + s.v.Y * wy;
		float add = g_search.cap - cur;
		if (add > 0.f) {
			if (add > g_search.accel_amt) add = g_search.accel_amt;
			s.v.X += add * wx;
			s.v.Y += add * wy;
		}
		s.p = s.p + Scale(s.v, g_search.dt);
		s.v.Z -= g_search.gravity * g_search.dt * 0.5f;
	}

	// One contact found by the hull sweep inside a tick.
	struct TickContact {
		int collider = -1;     // index into g_search.bcolliders
		int plane = -1;        // plane index within that collider
		float loss = 0.f;      // speed the clip resolution removed (u/s)
		Vector point;
	};

	// Swept-hull trace against the corridor brush world (AABB-gated convex
	// slab clips). Mirrors CM_ClipBoxToBrush exactly: enterfrac starts BELOW
	// zero so a contact at fraction 0 REGISTERS (the resting re-contact every
	// slide tick produces exactly f=0 - rejecting it made the model sink
	// through the face and stop colliding), enter side backs off DIST_EPSILON
	// (1/32), leave side extends by it. Returns the earliest entry fraction
	// in [0,1] (1 = clear) and the struck collider/plane.
	float TraceHullWorld(const Vector& a, const Vector& b, int* out_b, int* out_pl) {
		*out_b = -1;
		*out_pl = -1;
		const int nb = static_cast<int>(g_search.bcolliders.size());
		const float sminx = fminf(a.X, b.X), smaxx = fmaxf(a.X, b.X);
		const float sminy = fminf(a.Y, b.Y), smaxy = fmaxf(a.Y, b.Y);
		const float sminz = fminf(a.Z, b.Z), smaxz = fmaxf(a.Z, b.Z);
		float best = 1.f;
		for (int bi = 0; bi < nb; ++bi) {
			const SearchCtx::BrushCollider& bc = g_search.bcolliders[bi];
			if (smaxx < bc.bmin.X || sminx > bc.bmax.X ||
			    smaxy < bc.bmin.Y || sminy > bc.bmax.Y ||
			    smaxz < bc.bmin.Z || sminz > bc.bmax.Z)
				continue;
			float tmin = -1.f, tmax = 1.f;   // engine: enterfrac = -1, leavefrac = 1
			int enter = -1;
			bool outside = false, miss = false;
			const int np = static_cast<int>(bc.pn.size());
			for (int pi = 0; pi < np; ++pi) {
				const float d0 = Dot(bc.pn[pi], a) - bc.pd[pi];
				const float d1 = Dot(bc.pn[pi], b) - bc.pd[pi];
				if (d0 > 0.f) {
					outside = true;
					if (d1 > 0.f) { miss = true; break; }
					float tt = (d0 - 0.03125f) / (d0 - d1);   // DIST_EPSILON
					if (tt < 0.f) tt = 0.f;
					if (tt > tmin) { tmin = tt; enter = pi; }
				} else if (d1 > 0.f) {
					float tt = (d0 + 0.03125f) / (d0 - d1);
					if (tt > 1.f) tt = 1.f;
					if (tt < tmax) tmax = tt;
				}
			}
			if (miss || !outside || enter < 0 || tmin >= tmax)
				continue;
			if (tmin < best) {
				best = (tmin > 0.f) ? tmin : 0.f;
				*out_b = bi;
				*out_pl = enter;
			}
		}
		return best;
	}

	// CGameMovement::ClipVelocity with overbounce 1 (sv_bounce 0 makes walls
	// 1.0 too): out = in - n*dot(in,n), then the engine's one adjust
	// iteration so the result never still points into the plane.
	void EngineClipVelocity(const Vector& in, const Vector& n, Vector& out) {
		const float backoff = Dot(in, n);
		out = in - Scale(n, backoff);
		const float adjust = Dot(out, n);
		if (adjust < 0.f)
			out = out - Scale(n, adjust);
	}

	// One full airborne engine tick against the brush world - FullWalkMove's
	// exact order: view step, StartGravity, AirAccelerate, TryPlayerMove
	// (up to 4 bumps; partial moves re-base the clip set; first-impact plain
	// clip; multi-plane crease resolution; stop-dead guards), FinishGravity.
	// Every sub-segment the hull actually travels is written to seg_pts
	// ([nseg+1] points) so the caller can track the perigee THROUGH bumps;
	// contacts report the collider plane struck and the speed its clip took.
	// grounded = a floor-like plane (nz >= 0.7) was struck while falling
	// slower than NON_JUMP_VELOCITY (140) - CategorizePosition would set
	// ground and leave AirMove, so the model flight ends there.
	int EngineMoveTick(FastState& s, float yaw_abs, int side,
	                   Vector* seg_pts, TickContact* contacts, int* ncontacts,
	                   bool* grounded) {
		s.yaw = yaw_abs;                 // the tick's view yaw, set by the caller
		s.v.Z -= g_search.gravity * g_search.dt * 0.5f;          // StartGravity
		const float wish = (s.yaw + static_cast<float>(side) * 90.f) * (kPi / 180.f);
		const float wx = cosf(wish), wy = sinf(wish);            // AirAccelerate
		const float cur = s.v.X * wx + s.v.Y * wy;
		float add = g_search.cap - cur;
		if (add > 0.f) {
			if (add > g_search.accel_amt) add = g_search.accel_amt;
			s.v.X += add * wx;
			s.v.Y += add * wy;
		}

		// TryPlayerMove.
		*ncontacts = 0;
		*grounded = false;
		int nseg = 0;
		seg_pts[0] = s.p;
		float time_left = g_search.dt;
		Vector planes[5];
		int numplanes = 0;
		Vector original_v = s.v;
		const Vector primal_v = s.v;
		for (int bump = 0; bump < 4; ++bump) {
			if (Dot3(s.v) == 0.f)
				break;
			const Vector end = s.p + Scale(s.v, time_left);
			int hb, hpl;
			const float frac = TraceHullWorld(s.p, end, &hb, &hpl);
			if (frac > 0.f) {
				s.p = s.p + Scale(end - s.p, frac);
				seg_pts[++nseg] = s.p;
				original_v = s.v;    // engine re-bases the clip set on partial moves
				numplanes = 0;
			}
			if (frac >= 1.f || hb < 0)
				break;
			time_left -= time_left * frac;
			const Vector n = g_search.bcolliders[hb].pn[hpl];
			if (*ncontacts < 4) {
				TickContact& c = contacts[(*ncontacts)++];
				c.collider = hb;
				c.plane = hpl;
				c.point = s.p;
				c.loss = 0.f;        // filled after this bump's clip below
			}
			if (numplanes >= 5) {
				s.v = Vector(0.f, 0.f, 0.f);
				break;
			}
			planes[numplanes++] = n;
			const float sp_before = sqrtf(Dot3(s.v));
			if (numplanes == 1) {
				EngineClipVelocity(original_v, planes[0], s.v);
				original_v = s.v;
			} else {
				int i = 0;
				for (; i < numplanes; ++i) {
					EngineClipVelocity(original_v, planes[i], s.v);
					int j = 0;
					for (; j < numplanes; ++j)
						if (j != i && Dot(s.v, planes[j]) < 0.f)
							break;
					if (j == numplanes)
						break;
				}
				if (i == numplanes) {
					if (numplanes != 2) {
						s.v = Vector(0.f, 0.f, 0.f);
						break;
					}
					// Slide along the crease of the two planes.
					Vector dir(planes[0].Y * planes[1].Z - planes[0].Z * planes[1].Y,
					           planes[0].Z * planes[1].X - planes[0].X * planes[1].Z,
					           planes[0].X * planes[1].Y - planes[0].Y * planes[1].X);
					const float dl = sqrtf(Dot3(dir));
					if (dl > 1e-6f) dir = Scale(dir, 1.f / dl);
					s.v = Scale(dir, Dot(dir, s.v));
				}
				if (Dot(s.v, primal_v) <= 0.f) {
					s.v = Vector(0.f, 0.f, 0.f);
					break;
				}
			}
			if (*ncontacts > 0)
				contacts[*ncontacts - 1].loss = sp_before - sqrtf(Dot3(s.v));
			if (n.Z >= 0.7f && s.v.Z <= 140.f)
				*grounded = true;
		}
		if (nseg == 0)
			seg_pts[++nseg] = s.p;   // fully blocked: zero-length segment
		s.v.Z -= g_search.gravity * g_search.dt * 0.5f;          // FinishGravity
		return nseg;
	}

	// Ballistic height after j ticks (matches FastTick's half-gravity order:
	// z_j = z0 + vz0*dt*j - g*dt^2*j^2/2), used to COMPUTE the pass tick.
	float ZAfter(float z0, float vz0, int j) {
		const float dt = g_search.dt;
		return z0 + vz0 * dt * static_cast<float>(j)
			- 0.5f * g_search.gravity * dt * dt * static_cast<float>(j) * static_cast<float>(j);
	}

	// Tick count of each strafe phase within N ticks. Convention (shared with
	// the provider): tick t belongs to phase k iff splits[k-1] <= t < splits[k].
	void PhaseLens(int N, const int* splits, int nsplits, int* out_n) {
		int prev = 0;
		for (int k = 0; k < nsplits; ++k) {
			out_n[k] = splits[k] - prev;
			prev = splits[k];
		}
		out_n[nsplits] = N - prev;
	}

	void RateBounds(int side0, int K, float* lo, float* hi) {
		for (int i = 0; i < K; ++i) {
			const int side = (i % 2 == 0) ? side0 : -side0;
			if (side > 0) { lo[i] = -8.f; hi[i] = 0.f; }   // A: turns left only
			else          { lo[i] = 0.f;  hi[i] = 8.f; }   // D: turns right only
		}
	}

	// Orthonormal in-plane basis for the 2D residual against a face.
	void PlaneBasis(const Vector& n, Vector& e1, Vector& e2) {
		const Vector ref = (fabsf(n.Z) < 0.9f) ? Vector(0.f, 0.f, 1.f) : Vector(1.f, 0.f, 0.f);
		const Vector c(n.Y * ref.Z - n.Z * ref.Y, n.Z * ref.X - n.X * ref.Z, n.X * ref.Y - n.Y * ref.X);
		const float l = sqrtf(Dot3(c));
		e1 = Scale(c, 1.f / (l > 1e-6f ? l : 1.f));
		e2 = Vector(n.Y * e1.Z - n.Z * e1.Y, n.Z * e1.X - n.X * e1.Z, n.X * e1.Y - n.Y * e1.X);
	}

	// Residual basis for the exact-point event: two axes TRANSVERSE to the
	// nominal flight direction. The perigee's miss vector projected on these
	// is the residual (the along-path component vanishes at a closest
	// approach), so zeroing both means the arc passes THROUGH the point -
	// off-plane clearance can't hide the way it could with an in-plane basis.
	void TransverseBasis(const Vector& start, const Vector& tgt, Vector& e1, Vector& e2) {
		Vector d = tgt - start;
		const float l = sqrtf(Dot3(d));
		if (l < 1e-3f) {
			e1 = Vector(1.f, 0.f, 0.f);
			e2 = Vector(0.f, 1.f, 0.f);
			return;
		}
		d = Scale(d, 1.f / l);
		Vector u1(d.Y, -d.X, 0.f);          // d x up: horizontal transverse
		const float ul = sqrtf(Dot3(u1));
		u1 = (ul > 1e-3f) ? Scale(u1, 1.f / ul) : Vector(1.f, 0.f, 0.f);
		e1 = u1;
		e2 = Vector(d.Y * u1.Z - d.Z * u1.Y,  // d x u1: the other transverse
		            d.Z * u1.X - d.X * u1.Z,
		            d.X * u1.Y - d.Y * u1.X);
	}

	// Unified pass evaluation for one schedule.
	//  - face targets: the pass = the CLOSEST APPROACH of the path to the exact
	//    target point. The path is collision-truncated against the brush world:
	//    entry through the target's boardable face = the board (Source clip,
	//    loss recorded, short slide continues the tracking); entry through any
	//    other plane = the engine would deflect there, so the free arc ends and
	//    the perigee reached so far is the honest pass. The perigee varies
	//    smoothly with the rates - no first-contact cliff - so the Newton can
	//    walk a grazing arc down onto the point.
	//  - no face: interpolate at the precomputed height-crossing fraction.
	struct PassEval {
		Vector pass;
		int   tick = 0;
		float frac = 0.f;
		float speed = 0.f;      // 2D speed at the pass
		float yaw = 0.f;        // view yaw on the pass tick (for the yaw constraint)
		int   bump_ticks = 0;   // pass tick minus board tick (0 = board IS the pass)
		float bump_loss = 0.f;  // speed the board clip took away (u/s)
	};
	bool EvalPass(int N, float height_frac, int side0, const int* splits, int nsplits,
	              const float* rates, PassEval& out) {
		FastState s{ g_search.start_pos, g_search.start_vel, g_search.start_yaw };
		int k = 0;
		if (g_search.have_face) {
			const Vector tgt = g_search.target_pos;
			// Closest approach of the collision-resolved path to the point.
			// Integration = EngineMoveTick: the REAL TryPlayerMove semantics
			// against the corridor world - boards, kisses, slides, bounces and
			// creases all emerge from the geometry (no glued-plane slide, no
			// polygon heuristics). The flight only ENDS at grounding (the
			// engine would leave AirMove) or the window's end.
			Vector best_p = s.p;
			int best_tick = 0;
			float best_frac = 0.f;
			float best_v2 = sqrtf(s.v.X * s.v.X + s.v.Y * s.v.Y);
			float best_yaw = s.yaw;
			float best_d2 = Dot3(s.p - tgt);
			auto track = [&](const Vector& a, const Vector& b2, int tick2, float v2, float yawv) {
				const Vector d = b2 - a;
				const float len2 = Dot3(d);
				float tt = (len2 > 1e-9f) ? Dot(tgt - a, d) / len2 : 0.f;
				tt = Clampf(tt, 0.f, 1.f);
				const Vector cp = a + Scale(d, tt);
				const float dd = Dot3(cp - tgt);
				if (dd < best_d2) {
					best_d2 = dd;
					best_p = cp;
					best_tick = tick2;
					best_frac = tt;
					best_v2 = v2;
					best_yaw = yawv;
				}
			};
			out.bump_ticks = 0;
			out.bump_loss = 0.f;
			int contact_tick = -1;
			bool foreign_counted = false;
			for (int t = 0; t < N + 8; ++t) {
				while (k < nsplits && t >= splits[k]) k++;
				const int side = (k % 2 == 0) ? side0 : -side0;
				// rates[] = deg/tick turn rates, or per-phase BIASES off the
				// exact max-gain line in optimal mode (blend tail lands the
				// view exactly on the target yaw).
				const float yaw_next = g_search.opt
					? OptYawStep(s.yaw, s.v, t, N, g_search.blend, rates[k], side,
						g_search.target_yaw)
					: NormYaw(s.yaw - rates[k]);
				Vector segp[6];
				TickContact tc[4];
				int ntc = 0;
				bool grounded = false;
				const int nseg = EngineMoveTick(s, yaw_next, side,
				                                segp, tc, &ntc, &grounded);
				const float v2n = sqrtf(s.v.X * s.v.X + s.v.Y * s.v.Y);
				for (int si = 0; si < nseg; ++si)
					track(segp[si], segp[si + 1], t + 1, v2n, s.yaw);
				for (int ci = 0; ci < ntc; ++ci) {
					const SearchCtx::BrushCollider& bc = g_search.bcolliders[tc[ci].collider];
					if (bc.is_target && bc.pid[tc[ci].plane] == bc.target_plane) {
						if (contact_tick < 0) {
							contact_tick = t + 1;   // first board through the face
							out.bump_loss = tc[ci].loss;
						}
					} else if (!foreign_counted) {
						// Deflected by other geometry - counted for diagnostics,
						// but the flight CONTINUES exactly like the engine's.
						foreign_counted = true;
						if (bc.is_target)
							g_search.cnt_edge++;
						else
							g_search.early_hits++;
					}
				}
				if (grounded)
					break;   // engine grounds + leaves AirMove: dead line for surf
			}
			if (best_tick < 1)
				return false;   // the start itself was the nearest point - no pass
			out.pass = best_p;
			out.tick = best_tick;
			out.frac = best_frac;
			out.speed = best_v2;
			out.yaw = best_yaw;
			out.bump_ticks = (contact_tick >= 0 && best_tick > contact_tick)
				? best_tick - contact_tick : 0;
			if (contact_tick >= 0 && contact_tick > best_tick)
				out.bump_loss = 0.f;   // board AFTER the pass: post-event, not a kiss
			return true;
		}
		Vector prev = s.p;
		for (int t = 0; t < N; ++t) {
			while (k < nsplits && t >= splits[k]) k++;
			const int side = (k % 2 == 0) ? side0 : -side0;
			const float yaw_next = g_search.opt
				? OptYawStep(s.yaw, s.v, t, N, g_search.blend, rates[k], side,
					g_search.target_yaw)
				: NormYaw(s.yaw - rates[k]);
			prev = s.p;
			FastTick(s, yaw_next, side);
		}
		out.pass = prev + Scale(s.p - prev, height_frac);
		out.tick = N;
		out.frac = height_frac;
		out.speed = sqrtf(s.v.X * s.v.X + s.v.Y * s.v.Y);
		out.yaw = s.yaw;
		return true;
	}

	// Build the full rate vector for one candidate: fixed leading rates + the
	// free rates + the LAST rate eliminated EXACTLY from the yaw equation
	// sum(n_k * r_k) = T_w. False when the eliminated rate falls outside its
	// key's turn direction (that timing/wrap can't produce the required yaw).
	bool BuildRatesFromYaw(const int* n, int K, const float* lo, const float* hi,
	                       float T_w, const float* lead, int nlead,
	                       const float* free_r, int nfree, float* out_r) {
		for (int i = 0; i < nlead; ++i) out_r[i] = lead[i];
		for (int i = 0; i < nfree; ++i) out_r[nlead + i] = free_r[i];
		float acc = 0.f;
		for (int i = 0; i < K - 1; ++i) acc += static_cast<float>(n[i]) * out_r[i];
		const float rK = (T_w - acc) / static_cast<float>(n[K - 1]);
		if (rK < lo[K - 1] - 1e-6f || rK > hi[K - 1] + 1e-6f)
			return false;
		out_r[K - 1] = Clampf(rK, lo[K - 1], hi[K - 1]);
		for (int i = K; i < 4; ++i) out_r[i] = 0.f;
		return true;
	}

	// Solve one (N, frac, side0, timing) candidate: enumerate the feasible
	// 360-degree wraps of the yaw equation, and for each solve the 2D pass
	// residual over the free rates by damped Newton (LM). Exact solutions
	// (pass_err <= tol AND exact yaw by construction) append to the results;
	// the closest miss feeds the verdict.
	void SolveCandidate(int N, float frac, int side0, const int* splits, int nsplits) {
		const int K = nsplits + 1;
		int n[4];
		PhaseLens(N, splits, nsplits, n);
		for (int i = 0; i < K; ++i)
			if (n[i] < 1)
				return;
		float lo[4], hi[4];
		RateBounds(side0, K, lo, hi);

		float Tmin = 0.f, Tmax = 0.f;
		for (int i = 0; i < K; ++i) {
			Tmin += static_cast<float>(n[i]) * lo[i];
			Tmax += static_cast<float>(n[i]) * hi[i];
		}
		const float base = g_search.start_yaw - g_search.target_yaw;

		const Vector tgt = g_search.target_pos;
		float r[4];
		PassEval pe;
		float T_w = 0.f;
		int cur_wrap = 0;
		int n_eff = N;                // schedule length; retimed to the contact tick
		float lead[1] = { 0.f };
		int nlead = 0;

		auto resid = [&](const float* fr, int nfree, float* F, PassEval* pout) -> bool {
			// SELF-CONSISTENT schedule length: the yaw is eliminated over T
			// ticks, and T must equal the tick the pass actually lands in
			// (a bump-and-slide can shift it). Iterate the fixed point here,
			// per evaluation - the old outer "retimer" oscillated between the
			// direct-hit and long-slide branches and discarded perfect hits.
			// Accepted solutions are yaw-exact at the pass BY CONSTRUCTION.
			int T = n_eff;
			for (int fp = 0; fp < 4; ++fp) {
				int n_local[4];
				PhaseLens(T, splits, nsplits, n_local);
				bool ok = true;
				for (int i = 0; i < K; ++i)
					if (n_local[i] < 1) ok = false;
				for (int i = 0; i < nsplits; ++i)
					if (splits[i] >= T) ok = false;
				if (!ok)
					return false;
				float tmn = 0.f, tmx = 0.f;
				for (int i = 0; i < K; ++i) {
					tmn += static_cast<float>(n_local[i]) * lo[i];
					tmx += static_cast<float>(n_local[i]) * hi[i];
				}
				if (T_w < tmn - 1e-6f || T_w > tmx + 1e-6f)
					return false;
				if (!BuildRatesFromYaw(n_local, K, lo, hi, T_w, lead, nlead, fr, nfree, r))
					return false;
				g_search.yaw_ever_feasible = true;
				if (!EvalPass(T, frac, side0, splits, nsplits, r, pe))
					return false;
				if (pe.tick == T)
					break;
				if (fp == 3)
					return false;   // 2-cycle between branches: no consistent tick
				T = pe.tick;
			}
			const Vector d = pe.pass - tgt;
			F[0] = Dot(g_search.e1, d);
			F[1] = Dot(g_search.e2, d);
			if (pout) *pout = pe;
			return true;
		};

		auto record_reject = [&](const float* rr, float err, const PassEval& p,
		                         int n_used, int reason) {
			if (reason == 0) {
				g_search.cnt_floor++;
				if (err < g_search.reject_best)
					g_search.reject_best = err;
			} else if (reason == 1) {
				g_search.reject_retime++;
			} else {
				g_search.cnt_bump++;
			}
			// Keep the ~24 closest rejects with full detail for the log.
			if (g_search.top_rejects.size() < 24
				|| err < g_search.top_rejects.back().err) {
				SearchCtx::RejectRec rec;
				rec.err = err;
				rec.N = N;
				rec.n_eff = n_used;
				rec.tick = p.tick;
				rec.side0 = side0;
				rec.wrap = cur_wrap;
				for (int i = 0; i < 3; ++i) rec.splits[i] = (i < nsplits) ? splits[i] : 0;
				memcpy(rec.rates, rr, sizeof(float) * 4);
				rec.pass = p.pass;
				rec.reason = reason;
				if (g_search.top_rejects.size() >= 24)
					g_search.top_rejects.back() = rec;
				else
					g_search.top_rejects.push_back(rec);
				std::sort(g_search.top_rejects.begin(), g_search.top_rejects.end(),
					[](const SearchCtx::RejectRec& a, const SearchCtx::RejectRec& b) {
						return a.err < b.err;
					});
			}
		};

		auto consider = [&](const float* rr, float err, const PassEval& p, int expected_tick) {
			// COLLECT liberally - the calibration loop needs data, and an empty
			// list teaches nothing. Anything passing within `collect` with a
			// non-violent board enters the list; the tight gates only decide
			// the `exact` tag. Hard rejects remain for structure only.
			const bool yaw_ok = (p.tick == expected_tick);
			if (yaw_ok && err <= g_search.collect && p.bump_loss <= 60.f) {
				// EVERY distinct schedule that reaches the point is a solution
				// (the user explores the space) - no outcome-merging here; the
				// post-search DedupeResults collapses only near-identical
				// schedules (rates within 0.05 deg, splits within 2 ticks).
				// The cap is a memory backstop, far above any real search, and
				// COUNTED when hit instead of silently dropping lines.
				if (static_cast<int>(g_search.results.size()) < 20000) {
					RouteSolution s;
					s.ticks = p.tick;
					s.frac = p.frac;
					s.side0 = side0;
					s.nsplits = nsplits;
					for (int i = 0; i < 3; ++i) s.splits[i] = (i < nsplits) ? splits[i] : 0;
					memcpy(s.rates, rr, sizeof(float) * 4);
					s.wrap = cur_wrap;
					s.pass_err = err;
					s.speed = p.speed;
					s.pass_pos = p.pass;
					s.bump_ticks = p.bump_ticks;
					s.bump_loss = p.bump_loss;
					s.exact = (err <= g_search.tol && p.bump_loss <= g_search.max_bump);
					g_search.results.push_back(s);
				} else {
					g_search.cap_dropped++;
				}
			} else if (yaw_ok && err <= g_search.collect) {
				record_reject(rr, err, p, expected_tick, 2);   // violent board
			} else if (!yaw_ok) {
				record_reject(rr, err, p, expected_tick, 1);
			} else {
				record_reject(rr, err, p, expected_tick, 0);
			}
		};

		// LM over 1-2 free rates; fmap maps free index -> rate index (bounds).
		// Parametrized over the residual and bounds so the optimal-bias mode
		// reuses it unchanged. FAILURE-TOLERANT: an infeasible evaluation
		// (early board / no contact / yaw out of range) is a WALL to walk
		// along, never a reason to abort - aborting froze the whole search at
		// its starting guesses (the frozen -6.4/+6.4 rates visible across
		// three diagnostics logs).
		auto newton = [&](auto&& rfn, const float* rlo, const float* rhi,
		                  float* fr, int nfree, const int* fmap) -> float {
			float F[2];
			if (!rfn(fr, nfree, F, nullptr))
				return 1e9f;
			float err = sqrtf(F[0] * F[0] + F[1] * F[1]);
			float lambda = 0.05f;
			const float h = 0.04f;
			for (int it = 0; it < 30 && err > 0.02f; ++it) {
				float J[2][2] = {};
				bool have_col[2] = { false, false };
				for (int i = 0; i < nfree; ++i) {
					for (int attempt = 0; attempt < 2 && !have_col[i]; ++attempt) {
						float f2[2] = { fr[0], (nfree > 1) ? fr[1] : 0.f };
						const int bi = fmap[i];
						const float want = f2[i] + ((attempt == 0) ? h : -h);
						const float clamped = Clampf(want, rlo[bi], rhi[bi]);
						const float hs = clamped - f2[i];
						if (fabsf(hs) < 1e-6f)
							continue;
						f2[i] = clamped;
						float F2[2];
						if (rfn(f2, nfree, F2, nullptr)) {
							J[0][i] = (F2[0] - F[0]) / hs;
							J[1][i] = (F2[1] - F[1]) / hs;
							have_col[i] = true;
						}
					}
				}
				if (!have_col[0] && (nfree < 2 || !have_col[1]))
					break;   // boxed in on every side - genuinely stuck
				bool stepped = false;
				for (int attempt = 0; attempt < 6 && !stepped; ++attempt) {
					float d[2] = {};
					if (nfree == 1) {
						const float a = J[0][0] * J[0][0] + J[1][0] * J[1][0] + lambda;
						d[0] = -(J[0][0] * F[0] + J[1][0] * F[1]) / a;
					} else {
						const float a00 = J[0][0] * J[0][0] + J[1][0] * J[1][0] + lambda;
						const float a11 = J[0][1] * J[0][1] + J[1][1] * J[1][1] + lambda;
						const float a01 = J[0][0] * J[0][1] + J[1][0] * J[1][1];
						const float b0 = -(J[0][0] * F[0] + J[1][0] * F[1]);
						const float b1 = -(J[0][1] * F[0] + J[1][1] * F[1]);
						const float det = a00 * a11 - a01 * a01;
						if (fabsf(det) < 1e-12f) { lambda *= 5.f; continue; }
						d[0] = (b0 * a11 - b1 * a01) / det;
						d[1] = (b1 * a00 - b0 * a01) / det;
					}
					const float m = fmaxf(fabsf(d[0]), fabsf(d[1]));
					if (m > 2.f) { d[0] *= 2.f / m; d[1] *= 2.f / m; }
					float f2[2] = { fr[0], (nfree > 1) ? fr[1] : 0.f };
					for (int i = 0; i < nfree; ++i)
						f2[i] = Clampf(f2[i] + d[i], rlo[fmap[i]], rhi[fmap[i]]);
					float F2[2];
					if (rfn(f2, nfree, F2, nullptr)) {
						const float e2 = sqrtf(F2[0] * F2[0] + F2[1] * F2[1]);
						if (e2 < err) {
							for (int i = 0; i < nfree; ++i) fr[i] = f2[i];
							F[0] = F2[0]; F[1] = F2[1];
							err = e2;
							lambda = fmaxf(lambda * 0.4f, 1e-4f);
							stepped = true;
						}
					}
					if (!stepped)
						lambda *= 5.f;
				}
				if (!stepped)
					break;
			}
			return err;
		};

		// OPTIMAL-RATE MODE: per-phase yaw BIASES off the exact max-gain line
		// (wish perpendicular to the velocity). The view lands on the target
		// yaw via the BLEND TAIL inside OptYawStep - exact by construction,
		// no eliminated bias (welding the view to the heading for a whole
		// phase to hit the yaw cost ~65 u/s; the blend confines the loss to
		// a few ticks). The last (up to) two phases carry the free biases;
		// earlier phases fly the pure optimal curve.
		if (g_search.opt) {
			cur_wrap = 0;
			g_search.yaw_ever_feasible = true;
			float blo[4], bhi[4];
			for (int i = 0; i < 4; ++i) { blo[i] = -20.f; bhi[i] = 20.f; }
			const int nfree = (K < 2) ? K : 2;
			const int bbase = K - nfree;
			const int fmapo[2] = { bbase, bbase + 1 };
			auto oresid = [&](const float* fr, int nf, float* F, PassEval* pout) -> bool {
				float bias[4] = { 0.f, 0.f, 0.f, 0.f };
				for (int i = 0; i < nf; ++i) bias[bbase + i] = fr[i];
				if (!EvalPass(N, frac, side0, splits, nsplits, bias, pe))
					return false;
				const Vector dd = pe.pass - tgt;
				F[0] = Dot(g_search.e1, dd);
				F[1] = Dot(g_search.e2, dd);
				if (pout) *pout = pe;
				return true;
			};
			const float ostarts[5][2] = {
				{0.f, 0.f}, {5.f, -5.f}, {-5.f, 5.f}, {9.f, 9.f}, {-9.f, -9.f} };
			float best_err = 1e9f;
			float best_b[4] = {};
			PassEval best_po{};
			for (int s0 = 0; s0 < 5; ++s0) {
				float fr[2] = { ostarts[s0][0], (nfree > 1) ? ostarts[s0][1] : 0.f };
				if (nfree > 0)
					newton(oresid, blo, bhi, fr, nfree, fmapo);
				float F[2];
				PassEval p;
				if (oresid(fr, nfree, F, &p)) {
					const float e = sqrtf(F[0] * F[0] + F[1] * F[1]);
					if (e < best_err) {
						best_err = e;
						memset(best_b, 0, sizeof(best_b));
						for (int i = 0; i < nfree; ++i) best_b[bbase + i] = fr[i];
						best_po = p;
					}
				}
				if (best_err <= g_search.tol)
					break;
			}
			if (best_err > 1e8f) {
				g_search.cnt_no_contact++;
				return;
			}
			consider(best_b, sqrtf(Dot3(best_po.pass - tgt)), best_po, best_po.tick);
			return;
		}

		// Enumerate the feasible wraps: T_w = base + 360*w within [Tmin, Tmax].
		int w = static_cast<int>(ceilf((Tmin - base) / 360.f));
		for (; base + 360.f * static_cast<float>(w) <= Tmax + 1e-6f; ++w) {
			T_w = base + 360.f * static_cast<float>(w);
			cur_wrap = w;

			if (K <= 3) {
				const int nfree = K - 1;
				const int fmap[2] = { 0, 1 };
				n_eff = N;
				for (int pass = 0; pass < 1; ++pass) {
					// (Re)build the phase lengths + feasibility for n_eff.
					PhaseLens(n_eff, splits, nsplits, n);
					bool ok = true;
					for (int i = 0; i < K; ++i)
						if (n[i] < 1) ok = false;
					for (int i = 0; i < nsplits; ++i)
						if (splits[i] >= n_eff) ok = false;
					if (ok) {
						float tmn = 0.f, tmx = 0.f;
						for (int i = 0; i < K; ++i) {
							tmn += static_cast<float>(n[i]) * lo[i];
							tmx += static_cast<float>(n[i]) * hi[i];
						}
						if (T_w < tmn - 1e-6f || T_w > tmx + 1e-6f)
							ok = false;
					}
					if (!ok) {
						g_search.cnt_infeasible++;
						break;
					}

					float best_err = 1e9f;
					float best_r[4] = {};
					PassEval best_po{};
					if (nfree == 0) {
						float F[2];
						PassEval p;
						if (resid(nullptr, 0, F, &p)) {
							best_err = sqrtf(F[0] * F[0] + F[1] * F[1]);
							memcpy(best_r, r, sizeof(best_r));
							best_po = p;
						}
					} else {
						const int sidx = (side0 > 0) ? 0 : 1;
						const float mixes8[8][2] = {
							{0.5f, 0.5f}, {0.2f, 0.8f}, {0.8f, 0.2f}, {0.3f, 0.3f},
							{0.7f, 0.7f}, {0.1f, 0.4f}, {0.4f, 0.1f}, {0.9f, 0.6f} };
						const bool warm = g_search.have_warm[sidx];
						const int nstarts = warm ? 9 : 8;
						for (int s0 = 0; s0 < nstarts; ++s0) {
							float fr[2];
							if (warm && s0 == 0) {
								fr[0] = Clampf(g_search.warm_fr[sidx][0], lo[0], hi[0]);
								fr[1] = (nfree > 1)
									? Clampf(g_search.warm_fr[sidx][1], lo[1], hi[1]) : 0.f;
							} else {
								const int mi = warm ? s0 - 1 : s0;
								for (int i = 0; i < nfree; ++i)
									fr[i] = lo[i] + (hi[i] - lo[i]) * mixes8[mi][i];
							}
							newton(resid, lo, hi, fr, nfree, fmap);
							float F[2];
							PassEval p;
							if (resid(fr, nfree, F, &p)) {
								const float e = sqrtf(F[0] * F[0] + F[1] * F[1]);
								if (e < best_err) {
									best_err = e;
									memcpy(best_r, r, sizeof(best_r));
									best_po = p;
								}
							}
							if (best_err <= g_search.tol && best_po.tick == n_eff)
								break;
						}
						if (best_err < 60.f) {
							g_search.warm_fr[sidx][0] = best_r[0];
							g_search.warm_fr[sidx][1] = (nfree > 1) ? best_r[1] : 0.f;
							g_search.have_warm[sidx] = true;
						}
					}
					if (best_err > 1e8f) {
						g_search.cnt_no_contact++;
						break;
					}
					// resid's self-consistent fixed point guarantees the pass
					// tick matches the yaw schedule, whatever tick that is.
					// Gate on the FULL 3D miss (the residual is transverse-only;
					// at a true perigee the along-path component vanishes).
					consider(best_r, sqrtf(Dot3(best_po.pass - tgt)), best_po, best_po.tick);
					break;
				}
				continue;
			}
			// K == 4: grid the first rate; Newton over rates 2-3, last eliminated.
			n_eff = N;
			PhaseLens(N, splits, nsplits, n);
			nlead = 1;
			const int fmap4[2] = { 1, 2 };
			for (int g = 0; g < 5; ++g) {
				lead[0] = lo[0] + (hi[0] - lo[0]) * (static_cast<float>(g) + 0.5f) / 5.f;
				float fr[2] = { 0.5f * (lo[1] + hi[1]), 0.5f * (lo[2] + hi[2]) };
				newton(resid, lo, hi, fr, 2, fmap4);
				float F[2];
				PassEval p;
				if (resid(fr, 2, F, &p))
					consider(r, sqrtf(Dot3(p.pass - tgt)), p, p.tick);
			}
			nlead = 0;
		}
	}




	// Schedule lengths worth trying, from the ballistic height (which no strafe
	// can change). An ACCEPTED contact sits AT the target, so its z is the
	// target's z - and that pins the contact tick to the ballistic crossing of
	// the target's height. Only those ticks (+-1 for fraction-boundary safety)
	// can ever be accepted; seeding anything else just solves the yaw equation
	// over an impossible tick count (the diagnostics showed those flooring a
	// few units short in the wrong rate family). No face: same crossings, with
	// their exact fractions.
	void BuildPassCandidates() {
		g_search.cands.clear();
		const float z0 = g_search.start_pos.Z;
		const float vz0 = g_search.start_vel.Z;
		const float zt = g_search.target_pos.Z;
		float prev = z0;
		for (int j = 1; j <= 300; ++j) {
			const float z = ZAfter(z0, vz0, j);
			if ((prev - zt) * (z - zt) <= 0.f && prev != z) {
				if (g_search.have_face) {
					// +4 on the late side: a pre-target kiss slides, and the
					// slide descends slower than free fall, delaying the pass.
					for (int dj = -1; dj <= 4; ++dj) {
						const int cand = j + dj;
						if (cand < 1)
							continue;
						bool dup = false;
						for (const SearchCtx::PassCand& e : g_search.cands)
							if (e.N == cand) { dup = true; break; }
						if (!dup) {
							SearchCtx::PassCand c;
							c.N = cand;
							c.frac = 1.f;   // unused: the contact defines its own fraction
							g_search.cands.push_back(c);
						}
					}
				} else {
					const float frac = Clampf((zt - prev) / (z - prev), 0.f, 1.f);
					SearchCtx::PassCand c;
					c.N = j;
					c.frac = frac;
					g_search.cands.push_back(c);
				}
			}
			prev = z;
		}
	}

	// Full timing enumeration for one pass candidate. Accuracy first: stride 1
	// wherever affordable (the whole search may take a few seconds - fine).
	void BuildSplits(int N) {
		g_search.splits_list.clear();
		const int K = g_search.strafes;
		if (K == 1) {
			g_search.splits_list.push_back({ 0, 0, 0 });
		} else if (K == 2) {
			for (int a = 1; a < N; ++a)
				g_search.splits_list.push_back({ a, 0, 0 });
		} else if (K == 3) {
			const int stride = (N > 120) ? 2 : 1;
			for (int a = 1; a < N - 1; a += stride)
				for (int b = a + 1; b < N; b += stride)
					g_search.splits_list.push_back({ a, b, 0 });
		} else {
			const int stride = (N > 72) ? (N / 24) : 3;
			for (int a = 1; a < N - 2; a += stride)
				for (int b = a + 1; b < N - 1; b += stride)
					for (int c = b + 1; c < N; c += stride)
						g_search.splits_list.push_back({ a, b, c });
		}
		g_search.splits_built_for = N;
		g_search.split_idx = 0;
	}

	void ApplySolution(int seg_index, const RouteSolution& sol) {
		if (seg_index < 0 || seg_index >= static_cast<int>(g_segs.size()))
			return;
		SolverData& sd = g_segs[seg_index].solver;
		sd.solved = true;
		sd.tyaw = g_search.target_yaw;   // provider's blend target (opt mode)
		sd.ticks = sol.ticks;
		sd.side0 = sol.side0;
		sd.nsplits = sol.nsplits;
		memcpy(sd.splits, sol.splits, sizeof(sd.splits));
		memcpy(sd.rates, sol.rates, sizeof(sd.rates));
		sd.pass_frac = sol.frac;
		sd.model_pass = sol.pass_pos;
		sd.model_err = sol.pass_err;
		MarkDirty();
	}

	bool SolverStartState(int seg_index, Vector& pos, Vector& vel, float& yaw) {
		const int start = (seg_index < static_cast<int>(g_starts.size())) ? g_starts[seg_index] : 0;
		if (start == 0) {
			if (!g_anchor.valid)
				return false;
			pos = g_anchor.origin;
			vel = g_anchor.velocity;
			yaw = g_anchor.yaw;
			return true;
		}
		if (!g_valid || g_dirty || start > static_cast<int>(g_states.size()))
			return false;
		pos = g_states[start - 1].origin;
		vel = g_states[start - 1].velocity;
		yaw = g_frames[start - 1].viewangles[1];
		return true;
	}

	// Grounded start with 'Jump at start': in FullWalkMove the jump check runs
	// BEFORE the ground-friction block - CheckJumpButton clears the ground
	// entity, so Friction() is skipped and the jump tick is a normal AIR tick
	// with the vertical velocity SET by the jump. Run-up speed carries over
	// untouched (calibration trace 2026-07-24: the old friction pre-tick here
	// was exactly the measured tick-0 model-real gap, 15.7 u/s along heading).
	// The exact engine ladder for a standing jump, fresh stamina:
	//   StartGravity      vz  = -g*dt/2                      (-6)
	//   CheckJumpButton   vz += sqrt(2*800*57)               (hardcoded 800!)
	//                     vz *= stamina ratio (1 when fresh), FinishGravity -6
	//   tail FinishGravity                                    -6
	// The model's preset must be the value BEFORE the tick's own two half-
	// gravities, i.e. sqrt(2*800*57) - g*dt/2 = 295.9934 at dt 0.015 - which
	// is exactly the measured value the probe reports. Stamina from a recent
	// jump scales it; the measured vz input captures that case.
	void PrepStartJump(Vector& vel) {
		vel.Z = (g_jump_vz > 100.f)
			? g_jump_vz
			: sqrtf(2.f * 800.f * 57.f) - 0.5f * g_search.gravity * g_search.dt;
	}

	// Fill the collider set: the target's face first (its support plane passes
	// through the target EXACTLY - the target is a rest position), then every
	// tagged board surface with its computed hull-support offset.
	void BuildCollidersInto(const BspWorld::BoardTarget* target,
	                        std::vector<SearchCtx::Collider>& out) {
		out.clear();
		auto corner_for = [](const Vector& nn) {
			const float eps = 1e-4f;
			return Vector(
				nn.X > eps ? -16.f : nn.X < -eps ? 16.f : 0.f,
				nn.Y > eps ? -16.f : nn.Y < -eps ? 16.f : 0.f,
				nn.Z > eps ? 0.f : nn.Z < -eps ? 72.f : 36.f);
		};
		Vector n;
		if (BspWorld::GetPlane(target->plane, &n, nullptr)) {
			SearchCtx::Collider tc;
			tc.brush = target->brush;
			tc.plane = target->plane;
			tc.n = n;
			tc.d_origin = Dot(n, target->pos);
			tc.corner = corner_for(n);
			tc.is_target = true;
			out.push_back(tc);
		}
		for (int i = 0; i < BspWorld::TagCount() && static_cast<int>(out.size()) < 8; ++i) {
			int brush = -1, plane = -1;
			if (!BspWorld::GetTag(i, &brush, &plane))
				continue;
			if (brush == target->brush && plane == target->plane)
				continue;
			Vector tn;
			float td = 0.f;
			if (!BspWorld::GetPlane(plane, &tn, &td))
				continue;
			SearchCtx::Collider c;
			c.brush = brush;
			c.plane = plane;
			c.n = tn;
			// Origin-touch plane for a standing hull against this face.
			c.d_origin = td + 16.f * fabsf(tn.X) + 16.f * fabsf(tn.Y)
				+ (tn.Z < 0.f ? 72.f * fabsf(tn.Z) : 0.f);
			c.corner = corner_for(tn);
			out.push_back(c);
		}
	}

	// FULL-BRUSH collision world for the model: the target's brush, every
	// tagged face's brush, and EVERY solid brush along the flight corridor -
	// the same set TracePlayerBBox would hit (the calibration traces showed
	// real paths landing on the valley floor and side brushes the model knew
	// nothing about). Each brush = its complete hull-expanded plane set; the
	// plane the sweep ENTERS through decides board vs crash.
	void BuildBrushColliders(const BspWorld::BoardTarget* target,
	                         std::vector<SearchCtx::BrushCollider>& out) {
		out.clear();
		auto add_brush = [&](int brush, bool is_target, int target_plane) {
			for (const SearchCtx::BrushCollider& e : out)
				if (e.brush == brush)
					return;
			SearchCtx::BrushCollider bc;
			bc.brush = brush;
			bc.is_target = is_target;
			bc.target_plane = target_plane;
			const int n = BspWorld::BrushClipPlaneCount(brush);
			for (int i = 0; i < n; ++i) {
				int pid = -1;
				Vector pn;
				float pd = 0.f;
				if (!BspWorld::GetBrushClipPlane(brush, i, &pid, &pn, &pd))
					continue;
				// Hull expansion (standing 32x32x72, feet origin).
				float d_origin = pd + 16.f * fabsf(pn.X) + 16.f * fabsf(pn.Y)
					+ (pn.Z < 0.f ? 72.f * fabsf(pn.Z) : 0.f);
				// The boardable face uses the target's EXACT rest distance so
				// the sweep contact and the slide/residual share one plane.
				if (is_target && pid == target_plane)
					d_origin = Dot(pn, target->pos);
				bc.pn.push_back(pn);
				bc.pd.push_back(d_origin);
				bc.pid.push_back(pid);
			}
			if (bc.pn.empty())
				return;
			// Origin-space AABB gate: brush box grown by the hull (feet origin
			// reaches 16 out sideways and 72 down-to-feet above the box).
			Vector bmin, bmax;
			if (BspWorld::GetBrushInfo(brush, nullptr, &bmin, &bmax)) {
				bc.bmin = bmin - Vector(16.f, 16.f, 72.f);
				bc.bmax = bmax + Vector(16.f, 16.f, 0.f);
				// BEVEL planes: the engine's box clip always carries the six
				// axial planes of the brush AABB (bevel sides). Without them,
				// per-plane hull expansion over-collides in the wedge regions
				// past edges - the bevels cut those wedges so a hull passing an
				// edge resolves exactly like the engine (and an edge hit
				// reports an AXIAL normal, exactly like the engine's trace).
				const Vector bn[6] = {
					Vector(1.f, 0.f, 0.f), Vector(-1.f, 0.f, 0.f),
					Vector(0.f, 1.f, 0.f), Vector(0.f, -1.f, 0.f),
					Vector(0.f, 0.f, 1.f), Vector(0.f, 0.f, -1.f) };
				const float bd[6] = { bmax.X, -bmin.X, bmax.Y, -bmin.Y, bmax.Z, -bmin.Z };
				for (int i = 0; i < 6; ++i) {
					const float d_origin = bd[i]
						+ 16.f * fabsf(bn[i].X) + 16.f * fabsf(bn[i].Y)
						+ (bn[i].Z < 0.f ? 72.f : 0.f);
					bc.pn.push_back(bn[i]);
					bc.pd.push_back(d_origin);
					bc.pid.push_back(-2);   // bevel: never a boardable face
				}
			} else {
				bc.bmin = Vector(-1e9f, -1e9f, -1e9f);
				bc.bmax = Vector(1e9f, 1e9f, 1e9f);
			}
			out.push_back(bc);
		};
		add_brush(target->brush, true, target->plane);
		for (int i = 0; i < BspWorld::TagCount(); ++i) {
			int brush = -1, plane = -1;
			if (BspWorld::GetTag(i, &brush, &plane))
				add_brush(brush, false, -1);
		}
		// The corridor: everything solid the flight could touch between the
		// start and the target, padded for overshoot past the target and the
		// drop below it (grazing arcs land far down-slope before the search
		// reels them in - the model must see the same world there).
		const Vector a = g_search.start_pos, b = g_search.target_pos;
		const Vector cmin(fminf(a.X, b.X) - 256.f, fminf(a.Y, b.Y) - 256.f,
		                  fminf(a.Z, b.Z) - 288.f);
		const Vector cmax(fmaxf(a.X, b.X) + 256.f, fmaxf(a.Y, b.Y) + 256.f,
		                  fmaxf(a.Z, b.Z) + 128.f);
		const int nbr = BspWorld::BrushCount();
		for (int bi = 0; bi < nbr && static_cast<int>(out.size()) < 96; ++bi) {
			int contents = 0;
			Vector bmin, bmax;
			if (!BspWorld::GetBrushInfo(bi, &contents, &bmin, &bmax))
				continue;
			// SOLID | WINDOW | GRATE | PLAYERCLIP: what MASK_PLAYERSOLID hits.
			if (!(contents & (0x1 | 0x2 | 0x8 | 0x10000)))
				continue;
			if (bmax.X < cmin.X - 16.f || bmin.X > cmax.X + 16.f ||
			    bmax.Y < cmin.Y - 16.f || bmin.Y > cmax.Y + 16.f ||
			    bmax.Z < cmin.Z - 16.f || bmin.Z > cmax.Z + 72.f)
				continue;
			add_brush(bi, false, -1);
		}
	}

	void StartSearch(int seg_index) {
		if (g_batch.active)
			FinishBatch(true);      // a new search invalidates the batch's results
		g_verify.active = false;
		g_search = SearchCtx();
		g_solution_pick = 0;
		g_correct.active = false;   // a new search supersedes any in-flight correction
		g_correct.done = false;
		g_correct.note[0] = 0;
		if (seg_index < 0 || seg_index >= static_cast<int>(g_segs.size()) || !g_segs[seg_index].is_solver)
			return;
		g_search.seg = seg_index;

		const SolverData& sd = g_segs[seg_index].solver;
		const BspWorld::BoardTarget* target = BspWorld::GetTarget(sd.target);
		if (!target) {
			g_status = "Solver: select a board target first.";
			g_search.done = true;
			return;
		}
		if (!SolverStartState(seg_index, g_search.start_pos, g_search.start_vel, g_search.start_yaw)) {
			g_status = "Solver: this segment's start state isn't computed yet - the line refreshes "
				"by itself in a moment, then press Search again. (No anchor? Capture one first.)";
			g_search.done = true;
			return;
		}

		g_search.target_pos = target->pos;
		g_search.target_yaw = target->yaw;
		// The pass event: closest approach to the EXACT target point, on the
		// collision-truncated path (Dot(n, target->pos) IS the support plane
		// through the target - touching it there and passing through the point
		// are the same instant). Residual basis: transverse to the flight.
		Vector fn;
		g_search.have_face = BspWorld::GetPlane(target->plane, &fn, nullptr);
		if (g_search.have_face) {
			g_search.face_n = fn;
			g_search.face_d = Dot(fn, target->pos);
			TransverseBasis(g_search.start_pos, g_search.target_pos, g_search.e1, g_search.e2);
			BuildCollidersInto(target, g_search.colliders);
			BuildBrushColliders(target, g_search.bcolliders);
		} else {
			g_search.face_n = Vector(0.f, 0.f, 1.f);
			g_search.face_d = 0.f;
			g_search.e1 = Vector(1.f, 0.f, 0.f);
			g_search.e2 = Vector(0.f, 1.f, 0.f);
		}
		g_search.strafes = sd.strafes < 1 ? 1 : sd.strafes > 4 ? 4 : sd.strafes;
		g_search.opt = sd.opt;
		g_search.blend = (sd.blend < 1) ? 1 : (sd.blend > 24 ? 24 : sd.blend);

		float interval = Prediction::LastDiag().interval_per_tick;
		if (interval <= 0.f) interval = 0.015f;
		g_search.dt = interval;
		g_search.cap = g_air_cap;
		g_search.accel_amt = g_air_accel * g_wishspeed * interval;
		g_search.gravity = g_gravity;
		g_search.tol = g_sol_pos_tol;
		g_search.max_bump = g_sol_max_bump;
		g_search.collect = fmaxf(g_sol_collect, g_sol_pos_tol);
		g_search.jump_start = sd.start_jump ? 1 : 0;

		if (sd.start_jump)
			PrepStartJump(g_search.start_vel);

		BuildPassCandidates();
		if (g_search.cands.empty()) {
			g_search.done = true;
			g_status = "Solver: the ballistic arc NEVER reaches the target's height from this "
				"start - enable 'Jump at start' if the start is grounded, otherwise the "
				"target is too high (or the approach needs more airtime).";
			return;
		}
		g_search.jobs_total = 0;
		for (const SearchCtx::PassCand& c : g_search.cands) {
			BuildSplits(c.N);
			g_search.jobs_total += 2 * static_cast<int>(g_search.splits_list.size());
		}
		g_search.splits_built_for = -1;
		g_search.split_idx = 0;
		g_search.active = true;
		g_status = "Solver: searching (exact point, exact yaw)...";
	}

	// Pre-verification order: exact-tagged candidates first (by speed), then
	// the near candidates by model pass error. (After real verification the
	// list re-sorts by the REAL miss - provisional ranking throughout.)
	void SortResults(std::vector<RouteSolution>& v) {
		std::sort(v.begin(), v.end(),
			[](const RouteSolution& a, const RouteSolution& b) {
				if (a.exact != b.exact) return a.exact;
				if (a.exact) return a.speed > b.speed;
				return a.pass_err < b.pass_err;
			});
	}

	// Neighboring timings converge onto the same route; collapse near-identical
	// entries so the solution slider walks genuinely different lines.
	void DedupeResults(std::vector<RouteSolution>& v) {
		std::vector<RouteSolution> kept;
		kept.reserve(v.size());
		for (const RouteSolution& s : v) {
			bool dup = false;
			for (const RouteSolution& k : kept) {
				if (k.side0 != s.side0 || k.nsplits != s.nsplits || k.ticks != s.ticks)
					continue;
				bool same = true;
				for (int i = 0; i <= s.nsplits && same; ++i)
					same = fabsf(k.rates[i] - s.rates[i]) <= 0.05f;
				for (int i = 0; i < s.nsplits && same; ++i)
					same = abs(k.splits[i] - s.splits[i]) <= 2;
				if (same) { dup = true; break; }
			}
			if (!dup)
				kept.push_back(s);
		}
		v.swap(kept);
	}

	void StartCorrect(int seg_index);   // fwd (real-sim correction, below)

	void FinishSearch() {
		SortResults(g_search.results);
		DedupeResults(g_search.results);
		// A slider with thousands of near-ties is unusable and real-verifying
		// them all takes minutes: keep the best 600 of the DEDUPED ranking
		// (exact-first, fastest-first). The trim is REPORTED, never silent.
		g_search.dedupe_kept = static_cast<int>(g_search.results.size());
		if (g_search.results.size() > 600)
			g_search.results.resize(600);
		g_search.done = true;
		g_search.active = false;
		g_solution_pick = 0;
		if (!g_search.results.empty()) {
			// Model search done; now every candidate gets a REAL sim pass and
			// only real-verified ones survive into the presented list.
			StartVerify(g_search.seg);
			g_status = "Solver: " + std::to_string(g_search.dedupe_kept)
				+ " distinct model passes"
				+ (g_search.dedupe_kept > 600 ? " (verifying the best 600)" : "")
				+ " - verifying each on the real sim...";
		} else {
			// An empty search is exactly when the diagnostics matter - write
			// them without being asked.
			WriteSearchLog();
			g_status = std::string("Solver: no exact pass - search diagnostics written: ")
				+ (g_searchlog_path[0] ? g_searchlog_path : "(write failed)");
		}
	}

	// Time-sliced search pump (~5 ms per frame from Update()). Accuracy first:
	// the enumeration is allowed to take seconds.
	// SEH guard: a fault inside one solve job becomes a logged, named failure
	// (the exact candidate parameters ARE the repro) instead of a dead game.
	// This frame holds only PODs so __try is legal here; the search aborts.
	int g_guard_code = 0;
	const char* g_stage_name = "";
	std::string CalibDir();   // fwd (defined with the batch)

	// LAST-RESORT crash writer: fires when a fault escapes every frame guard
	// (any thread - game, render, engine internals). Writes the code, the
	// faulting address and its MODULE+OFFSET, which is enough to name the
	// culprit binary and function even after the process dies. Keeps the
	// previous filter chained so nothing else breaks.
	LPTOP_LEVEL_EXCEPTION_FILTER g_prev_uef = nullptr;
	LONG WINAPI CrashWriter(EXCEPTION_POINTERS* ep) {
		char path[MAX_PATH] = {};
		char documents[MAX_PATH] = {};
		if (SUCCEEDED(SHGetFolderPathA(nullptr, CSIDL_PERSONAL, nullptr,
			SHGFP_TYPE_CURRENT, documents))) {
			_snprintf_s(path, _TRUNCATE, "%s\\sourceTAS\\calibration\\crash.log", documents);
			HANDLE f = CreateFileA(path, FILE_APPEND_DATA, FILE_SHARE_READ, nullptr,
				OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
			if (f != INVALID_HANDLE_VALUE) {
				char mod[MAX_PATH] = "(unknown)";
				unsigned long long base = 0;
				HMODULE hm = nullptr;
				void* addr = ep && ep->ExceptionRecord
					? ep->ExceptionRecord->ExceptionAddress : nullptr;
				if (addr && GetModuleHandleExA(
					GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS
					| GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
					static_cast<LPCSTR>(addr), &hm) && hm) {
					GetModuleFileNameA(hm, mod, sizeof(mod));
					base = reinterpret_cast<unsigned long long>(hm);
				}
				char line[640];
				_snprintf_s(line, _TRUNCATE,
					"PROCESS CRASH code=0x%08X addr=0x%llX module=%s+0x%llX thread=%lu stage=%s\r\n",
					ep && ep->ExceptionRecord ? ep->ExceptionRecord->ExceptionCode : 0,
					reinterpret_cast<unsigned long long>(addr),
					mod,
					base ? reinterpret_cast<unsigned long long>(addr) - base : 0ull,
					GetCurrentThreadId(),
					g_stage_name && g_stage_name[0] ? g_stage_name : "(none)");
				DWORD written = 0;
				WriteFile(f, line, static_cast<DWORD>(strlen(line)), &written, nullptr);
				CloseHandle(f);
			}
		}
		return g_prev_uef ? g_prev_uef(ep) : EXCEPTION_CONTINUE_SEARCH;
	}

	// Vectored FIRST-CHANCE logging for the killer classes the unhandled
	// filter never sees (heap corruption and friends terminate the process
	// without normal dispatch; the game's own crash handler can eat the
	// rest). Fatal-looking codes only - the prediction system ROUTINELY
	// recovers access violations, so those stay out to keep the log clean.
	PVOID g_veh_handle = nullptr;
	LONG CALLBACK FirstChanceWriter(EXCEPTION_POINTERS* ep) {
		if (!ep || !ep->ExceptionRecord)
			return EXCEPTION_CONTINUE_SEARCH;
		const DWORD code = ep->ExceptionRecord->ExceptionCode;
		const bool fatal_class =
			code == 0xC0000374          // STATUS_HEAP_CORRUPTION
			|| code == 0xC00000FD      // STATUS_STACK_OVERFLOW
			|| code == 0xC0000409      // STATUS_STACK_BUFFER_OVERRUN (fastfail)
			|| (ep->ExceptionRecord->ExceptionFlags & EXCEPTION_NONCONTINUABLE) != 0;
		if (!fatal_class)
			return EXCEPTION_CONTINUE_SEARCH;
		CrashWriter(ep);   // same module+offset line, tagged by its code
		return EXCEPTION_CONTINUE_SEARCH;
	}
	void RecomputeDiag();     // fwd
	void AnalyzeRealPass();   // fwd
	void BatchPump();         // fwd
	void VerifyPump();        // fwd
	void CorrectPump();       // fwd

	// Everything that runs when a sim lands, with a stage marker so a fault
	// names its location. Lives behind SimLandedGuarded's POD-only SEH frame.
	void SimLandedStage() {
		g_stage_name = "TakeSim";
		const int n = Prediction::SimCount();
		g_sim_fault = Prediction::SimFaulted();
		g_frames.resize(n);
		g_states.resize(n);
		Prediction::TakeSim(g_frames.data(), g_states.data());
		g_valid = n > 0;
		g_sim_requested = false;
		g_stage_name = "RecomputeDiag";
		RecomputeDiag();
		g_stage_name = "AnalyzeRealPass";
		AnalyzeRealPass();
		if (g_batch.active) {
			g_stage_name = "BatchPump";
			BatchPump();
		} else if (g_verify.active) {
			g_stage_name = "VerifyPump";
			VerifyPump();
		} else {
			g_stage_name = "CorrectPump";
			CorrectPump();
		}
		g_stage_name = "";
	}
	bool SimLandedGuarded() {
		__try {
			SimLandedStage();
			return true;
		} __except (EXCEPTION_EXECUTE_HANDLER) {
			g_guard_code = static_cast<int>(GetExceptionCode());
			return false;
		}
	}
	bool SolveCandidateGuarded(int N, float frac, int side0, const int* splits, int nsplits) {
		__try {
			SolveCandidate(N, frac, side0, splits, nsplits);
			return true;
		} __except (EXCEPTION_EXECUTE_HANDLER) {
			g_guard_code = static_cast<int>(GetExceptionCode());
			return false;
		}
	}
	// FinishSearch sorts/dedupes the whole pool and kicks off verification -
	// it runs at EXACTLY "the search just completed", so it gets its own guard
	// (it used to sit outside every guard; a fault there was a dead game).
	bool FinishSearchGuarded() {
		__try {
			FinishSearch();
			return true;
		} __except (EXCEPTION_EXECUTE_HANDLER) {
			g_guard_code = static_cast<int>(GetExceptionCode());
			return false;
		}
	}

	void StepSearch() {
		if (!g_search.active || g_search.done)
			return;
		const ULONGLONG t0 = GetTickCount64();
		while (GetTickCount64() - t0 < 5) {
			if (g_search.cand_idx >= static_cast<int>(g_search.cands.size())) {
				if (!FinishSearchGuarded()) {
					char note[256];
					Sfmt(note, "EDITOR FAULT 0x%08X in stage FinishSearch "
						"(results=%d) - search aborted; report this line.",
						g_guard_code, static_cast<int>(g_search.results.size()));
					g_status = note;
					const std::string dir = CalibDir();
					if (!dir.empty()) {
						std::ofstream f(dir + "\\solver_fault.log", std::ios::app);
						if (f)
							f << note << "\n";
					}
					g_search.done = true;
					g_search.active = false;
					g_verify.active = false;
				}
				return;
			}
			const SearchCtx::PassCand pc = g_search.cands[g_search.cand_idx];
			if (g_search.splits_built_for != pc.N)
				BuildSplits(pc.N);
			if (g_search.split_idx >= static_cast<int>(g_search.splits_list.size())) {
				g_search.split_idx = 0;
				g_search.side_idx++;
				if (g_search.side_idx > 1) {
					g_search.side_idx = 0;
					g_search.cand_idx++;
					g_search.splits_built_for = -1;
				}
				continue;
			}
			const std::array<int, 3>& sp = g_search.splits_list[g_search.split_idx++];
			g_search.jobs_done++;
			if (!SolveCandidateGuarded(pc.N, pc.frac, g_search.side_idx == 0 ? 1 : -1,
			                           sp.data(), g_search.strafes - 1)) {
				char note[512];
				Sfmt(note, "SOLVER FAULT 0x%08X at N=%d side=%+d splits=%d,%d,%d "
					"strafes=%d%s - search aborted safely; this candidate is the repro.",
					g_guard_code, pc.N, g_search.side_idx == 0 ? 1 : -1,
					sp[0], sp[1], sp[2], g_search.strafes, g_search.opt ? " OPT" : "");
				g_status = note;
				const std::string dir = CalibDir();
				if (!dir.empty()) {
					std::ofstream f(dir + "\\solver_fault.log", std::ios::app);
					if (f)
						f << note << "\n";
				}
				g_search.done = true;
				g_search.active = false;
				return;
			}
		}
	}

	// Extra sim ticks past the plan when it ends on a solved solver segment:
	// the drawn line then flies visibly THROUGH the pass point instead of
	// stopping on it, and the pass analysis has slack for tick drift.
	constexpr int kSolverOverrun = 16;

	int FindSolverSeg() {
		for (int k = static_cast<int>(g_segs.size()) - 1; k >= 0; --k)
			if (g_segs[k].is_solver && g_segs[k].solver.solved && !g_segs[k].raw)
				return k;
		return -1;
	}

	// Measure the REAL sim's pass event. Face targets: the entry into the
	// support plane's 1u shell (the engine stops the origin ~0.03u above the
	// plane at contact and never lets it cross), with the exact touch point
	// recovered by intersecting the incoming free-flight segment with the
	// plane; among several entries (the infinite plane can be skimmed far
	// away) the one nearest the target wins. No face: the height crossing.
	// Plus probes that measure the engine's actual constants from the states.
	void AnalyzeRealPass() {
		g_realpass = RealPass();
		const int L = FindSolverSeg();
		if (L < 0 || L >= static_cast<int>(g_starts.size()) || g_states.empty())
			return;
		const SolverData& sd = g_segs[L].solver;
		const BspWorld::BoardTarget* target = BspWorld::GetTarget(sd.target);
		if (!target)
			return;
		const Vector tgt = target->pos;
		Vector fn;
		const bool have_face = BspWorld::GetPlane(target->plane, &fn, nullptr);
		float interval = Prediction::LastDiag().interval_per_tick;
		if (interval <= 0.f) interval = 0.015f;
		// The sub-tick reconstruction below runs the model tick on real
		// states; keep its parameters current (same values a search uses).
		g_search.dt = interval;
		g_search.cap = g_air_cap;
		g_search.accel_amt = g_air_accel * g_wishspeed * interval;
		g_search.gravity = g_gravity;

		g_realpass.computed = true;
		const int b = g_starts[L];
		const int count = static_cast<int>(g_states.size());
		const int want = b + sd.ticks - 1;   // global index of the model's pass tick
		int best_gap = 0x7fffffff;
		std::vector<float> grav_samples;
		if (b < count)
			g_realpass.real_vz0 = g_states[b].velocity.Z;

		// Mirror the model: the pass = the interpolated CLOSEST APPROACH of the
		// whole real path (which, after a pre-target kiss, slides along the
		// ramp through the target). Face entries are tracked separately: the
		// first foreign-face board fails the run; the first target-face entry
		// before the pass is the "kiss" (bump), reported with its clip loss.
		std::vector<SearchCtx::Collider> cols;
		if (have_face)
			BuildCollidersInto(target, cols);
		const int ncols = (std::min)(static_cast<int>(cols.size()), 8);
		float cb_d2 = 1e30f;
		Vector cb_pass;
		int cb_tick = -1;
		int fe_tick = -1, fe_brush = -1, fe_plane = -1;
		Vector fe_pass;
		float fe_err = 0.f;
		int te_tick = -1;
		float te_loss = 0.f;
		int we_tick = -1, we_brush = -1;   // world-brush deflection (model-symmetric)
		Vector we_pass;

		for (int i = b; i < count; ++i) {
			const Vector o = g_states[i].origin;
			const Vector prev_o = (i > 0) ? g_states[i - 1].origin : g_anchor.origin;
			const Vector prev_v = (i > 0) ? g_states[i - 1].velocity : g_anchor.velocity;

			// First world deflection (metadata for the on_target verdict): the
			// same sweep the model runs. The path itself CONTINUES - both the
			// engine and the model clip and keep flying through deflections.
			if (have_face && te_tick < 0 && fe_tick < 0 && we_tick < 0) {
				int hit_b = -1, hit_pl = -1;
				const float hit_t = TraceHullWorld(prev_o, o, &hit_b, &hit_pl);
				if (hit_b >= 0 && hit_t < 1.f) {
					const SearchCtx::BrushCollider& bc = g_search.bcolliders[hit_b];
					if (!(bc.is_target && bc.pid[hit_pl] == bc.target_plane)) {
						we_tick = i;
						we_brush = bc.brush;
						we_pass = prev_o + Scale(o - prev_o, hit_t);
					}
				}
			}

			// Interpolated closest approach of the segment. The scan runs
			// through deflections (the model's integration does too) and only
			// stops when the player GROUNDS - the engine leaves AirMove there
			// and so does the model. NEAR THE TARGET the straight chord
			// between end-of-tick states cuts the corner the engine actually
			// turned mid-tick (a board bends the path INSIDE the tick) - that
			// chord was a constant ~0.3u false miss on perfect landings. So
			// within range, the tick is re-integrated with the engine tick
			// from the previous real state (model == engine, proven by the
			// traces) and the closest approach runs over the true sub-segments.
			{
				const Vector d = o - prev_o;
				const float len2 = Dot3(d);
				float tt = (len2 > 1e-9f) ? Dot(tgt - prev_o, d) / len2 : 0.f;
				tt = Clampf(tt, 0.f, 1.f);
				const Vector cp = prev_o + Scale(d, tt);
				float dd = Dot3(cp - tgt);
				Vector best_cp = cp;
				const bool solver_style = (i < static_cast<int>(g_frames.size()))
					&& fabsf(g_frames[i].forwardmove) < 10.f
					&& fabsf(g_frames[i].sidemove) > 10.f;
				if (have_face && dd < 3600.f && solver_style
					&& !g_search.bcolliders.empty()) {
					FastState rs{ prev_o, prev_v, g_frames[i].viewangles[1] };
					Vector rsp[6];
					TickContact rtc[4];
					int rnc = 0;
					bool rgr = false;
					const int rseg = EngineMoveTick(rs, g_frames[i].viewangles[1],
						(g_frames[i].sidemove < 0.f) ? 1 : -1, rsp, rtc, &rnc, &rgr);
					for (int si = 0; si < rseg; ++si) {
						const Vector d2 = rsp[si + 1] - rsp[si];
						const float l2 = Dot3(d2);
						float t2 = (l2 > 1e-9f) ? Dot(tgt - rsp[si], d2) / l2 : 0.f;
						t2 = Clampf(t2, 0.f, 1.f);
						const Vector cp2 = rsp[si] + Scale(d2, t2);
						const float dd2 = Dot3(cp2 - tgt);
						if (dd2 < dd) {
							dd = dd2;
							best_cp = cp2;
						}
					}
				}
				if (dd < cb_d2) {
					cb_d2 = dd;
					cb_pass = best_cp;
					cb_tick = i;
				}
				const float dist = sqrtf(dd);
				if (dist < g_realpass.min_dist)
					g_realpass.min_dist = dist;
			}
			if (have_face && i > b && (g_states[i].flags & FL_ONGROUND) != 0)
				break;   // grounded: the airborne line is over (both sides stop here)

			if (have_face && we_tick < 0) {
				for (int c = 0; c < ncols; ++c) {
					const SearchCtx::Collider& col = cols[c];
					const float phi = Dot(col.n, o) - col.d_origin;
					const float prev_phi = Dot(col.n, prev_o) - col.d_origin;
					if (col.is_target && phi < g_realpass.closest_phi)
						g_realpass.closest_phi = phi;
					if (prev_phi >= 1.f && phi < 1.f) {
						const Vector step = Scale(prev_v, interval);
						const float pe = Dot(col.n, prev_o + step) - col.d_origin;
						float tau = 1.f;
						if (prev_phi - pe > 1e-6f)
							tau = Clampf(prev_phi / (prev_phi - pe), 0.f, 1.f);
						const Vector touch = prev_o + Scale(step, tau);
						if (BspWorld::PointOnFace(col.brush, col.plane, touch + col.corner, 6.f)) {
							if (col.is_target) {
								if (te_tick < 0) {
									te_tick = i;
									const float into = Dot(col.n, prev_v);
									if (into < 0.f) {
										const Vector vc = prev_v - Scale(col.n, into);
										te_loss = sqrtf(Dot3(prev_v)) - sqrtf(Dot3(vc));
									}
								}
							} else if (fe_tick < 0) {
								fe_tick = i;
								fe_brush = col.brush;
								fe_plane = col.plane;
								fe_pass = touch;
								fe_err = sqrtf(Dot3(touch - tgt));
							}
						}
					}
				}
			} else {
				const float dz = fabsf(o.Z - tgt.Z);
				if (dz < g_realpass.closest_dz)
					g_realpass.closest_dz = dz;
				if ((prev_o.Z - tgt.Z) * (o.Z - tgt.Z) <= 0.f && prev_o.Z != o.Z) {
					const int gap = abs(i - want);
					if (gap < best_gap) {
						best_gap = gap;
						const float frac = Clampf((tgt.Z - prev_o.Z) / (o.Z - prev_o.Z), 0.f, 1.f);
						g_realpass.pass = prev_o + Scale(o - prev_o, frac);
						const float dx = g_realpass.pass.X - tgt.X;
						const float dy = g_realpass.pass.Y - tgt.Y;
						g_realpass.err = sqrtf(dx * dx + dy * dy);
						g_realpass.tick = i;
						g_realpass.crossed = true;
						if (i < static_cast<int>(g_frames.size()))
							g_realpass.yaw_err = fabsf(NormYaw(g_frames[i].viewangles[1] - target->yaw));
					}
				}
			}

			// Engine-constant probes from fully-airborne pre-contact tick pairs:
			// gravity (dvz/dt), the observed per-tick accel along the wish, and
			// the observed speed cap (cur + add). These NAME a model mismatch
			// (e.g. server sv_airaccelerate above the model's) from data.
			const bool air_prev = (i > b) && ((g_states[i - 1].flags & FL_ONGROUND) == 0);
			const bool air_now = (g_states[i].flags & FL_ONGROUND) == 0;
			if (air_prev && air_now && !g_realpass.crossed
				&& te_tick < 0 && fe_tick < 0 && we_tick < 0) {
				grav_samples.push_back((g_states[i - 1].velocity.Z - g_states[i].velocity.Z) / interval);
				if (i < static_cast<int>(g_frames.size()) && fabsf(g_frames[i].sidemove) > 10.f) {
					const int side = (g_frames[i].sidemove < 0.f) ? 1 : -1;
					const float wr = (g_frames[i].viewangles[1] + side * 90.f) * (kPi / 180.f);
					const float wx = cosf(wr), wy = sinf(wr);
					const float cur = prev_v.X * wx + prev_v.Y * wy;
					const float addv = (g_states[i].velocity.X - prev_v.X) * wx
						+ (g_states[i].velocity.Y - prev_v.Y) * wy;
					if (addv > 0.5f) {
						if (addv > g_realpass.max_add) g_realpass.max_add = addv;
						if (cur + addv > g_realpass.cap_fit) g_realpass.cap_fit = cur + addv;
					}
				}
			}
		}

		if (have_face) {
			const bool truncated = (fe_tick >= 0) || (we_tick >= 0);
			const int trunc_tick = (fe_tick >= 0) ? fe_tick : we_tick;
			if (truncated && (cb_tick < 0 || cb_tick >= trunc_tick)) {
				// Deflected at (or before) the nearest it ever got - an early
				// board / world hit ends the run; report the honest closest
				// point of the path that existed.
				g_realpass.crossed = true;
				g_realpass.on_target = false;
				g_realpass.hit_brush = (fe_tick >= 0) ? fe_brush : we_brush;
				g_realpass.hit_plane = (fe_tick >= 0) ? fe_plane : -1;
				g_realpass.tick = trunc_tick;
				g_realpass.pass = (fe_tick >= 0) ? fe_pass : we_pass;
				g_realpass.err = sqrtf(Dot3(g_realpass.pass - tgt));
				if (cb_tick >= 0 && cb_d2 < Dot3(g_realpass.pass - tgt) ) {
					g_realpass.pass = cb_pass;
					g_realpass.err = sqrtf(cb_d2);
				}
			} else if (cb_tick >= 0 && sqrtf(cb_d2) <= 150.f) {
				// Clean pass (a deflection strictly AFTER the perigee is post-
				// event and doesn't spoil it - the model sees the same thing).
				g_realpass.crossed = true;
				g_realpass.on_target = true;
				g_realpass.pass = cb_pass;
				g_realpass.err = sqrtf(cb_d2);
				g_realpass.tick = cb_tick;
				if (cb_tick < static_cast<int>(g_frames.size()))
					g_realpass.yaw_err = fabsf(NormYaw(g_frames[cb_tick].viewangles[1] - target->yaw));
				if (te_tick >= 0 && te_tick < cb_tick) {
					g_realpass.bump_tick = te_tick;
					g_realpass.bump_ticks = cb_tick - te_tick;
					g_realpass.bump_loss = te_loss;
				}
			}
		}
		if (g_realpass.crossed)
			g_realpass.drift = (Dot3(sd.model_pass) > 1e-6f)
				? sqrtf(Dot3(g_realpass.pass - sd.model_pass)) : -1.f;
		// start_jump sanity: the segment's first tick must carry the jump
		// velocity, else the sim start wasn't actually on the ground.
		if (sd.start_jump && b < count)
			g_realpass.jump_missing = g_states[b].velocity.Z < 150.f;
		if (!grav_samples.empty()) {
			std::nth_element(grav_samples.begin(),
				grav_samples.begin() + grav_samples.size() / 2, grav_samples.end());
			g_realpass.grav_fit = grav_samples[grav_samples.size() / 2];
			g_realpass.fit_samples = static_cast<int>(grav_samples.size());
		}

		// Harvest the measured engine values for the model to adopt.
		if (g_realpass.max_add > g_meas_max_add)
			g_meas_max_add = g_realpass.max_add;
		if (g_realpass.fit_samples >= 8) {
			g_meas_grav = g_realpass.grav_fit;
			g_meas_grav_n = g_realpass.fit_samples;
		}
		if (sd.start_jump && !g_realpass.jump_missing && b < count) {
			// The model's first tick computes state_vz = jump_vz - g*dt; invert
			// it from the measurement so the model reproduces reality exactly
			// (this captures the stamina scaling without knowing its formula).
			const float g_used = (g_meas_grav_n >= 8) ? g_meas_grav : g_gravity;
			g_meas_jump_vz = g_realpass.real_vz0 + g_used * interval;
		}
	}

	// Re-solve the APPLIED schedule's free rates against a shifted virtual
	// target (the secant correction). Timing, wrap and yaw stay untouched:
	// T_w is recomputed EXACTLY from the stored rates, so the corrected
	// schedule still ends on the target yaw. Warm-started from the applied
	// rates; K=4 keeps its first rate fixed, K=2 corrects with 1 DOF (partial).
	bool ReSolveApplied(int seg_index, const Vector& virt) {
		if (seg_index < 0 || seg_index >= static_cast<int>(g_segs.size()))
			return false;
		EditSegment& es = g_segs[seg_index];
		if (!es.is_solver || !es.solver.solved)
			return false;
		SolverData& sd = es.solver;
		const int K = sd.nsplits + 1;
		if (K < 2 && !sd.opt)
			return false;   // no free parameters left after the yaw constraint
		const BspWorld::BoardTarget* target = BspWorld::GetTarget(sd.target);
		if (!target)
			return false;

		Vector pos, vel;
		float yaw;
		if (!SolverStartState(seg_index, pos, vel, yaw))
			return false;
		float interval = Prediction::LastDiag().interval_per_tick;
		if (interval <= 0.f) interval = 0.015f;
		g_search.dt = interval;
		g_search.cap = g_air_cap;
		g_search.accel_amt = g_air_accel * g_wishspeed * interval;
		g_search.gravity = g_gravity;
		if (sd.start_jump)
			PrepStartJump(vel);
		g_search.start_pos = pos;
		g_search.start_vel = vel;
		g_search.start_yaw = yaw;
		g_search.target_pos = target->pos;
		Vector fn;
		g_search.have_face = BspWorld::GetPlane(target->plane, &fn, nullptr);
		if (g_search.have_face) {
			g_search.face_n = fn;
			g_search.face_d = Dot(fn, target->pos);   // face stays the REAL rest plane
			TransverseBasis(g_search.start_pos, target->pos, g_search.e1, g_search.e2);
			BuildCollidersInto(target, g_search.colliders);
			BuildBrushColliders(target, g_search.bcolliders);
		} else {
			g_search.e1 = Vector(1.f, 0.f, 0.f);
			g_search.e2 = Vector(0.f, 1.f, 0.f);
		}
		// Track the perigee against the VIRTUAL point so the event and the
		// residual agree about what "the pass" is during correction.
		g_search.target_pos = virt;

		int n[4];
		PhaseLens(sd.ticks, sd.splits, sd.nsplits, n);
		float lo[4], hi[4];
		RateBounds(sd.side0, K, lo, hi);
		g_search.opt = sd.opt;   // EvalPass steps by bias when set
		g_search.blend = (sd.blend < 1) ? 1 : (sd.blend > 24 ? 24 : sd.blend);
		g_search.target_yaw = target->yaw;
		if (sd.opt)
			for (int i = 0; i < 4; ++i) { lo[i] = -20.f; hi[i] = 20.f; }
		float T_w = 0.f;
		for (int i = 0; i < K; ++i)
			T_w += static_cast<float>(n[i]) * sd.rates[i];

		// Free parameters. Constant mode: K=2 -> {r0}; K=3 -> {r0,r1};
		// K=4 -> {r1,r2} (r0 fixed, last eliminated from the yaw equation).
		// Optimal mode: the LAST (up to) two biases (the blend tail keeps the
		// yaw exact); untouched phases keep their applied values.
		const int nfree = sd.opt ? ((K < 2) ? K : 2) : ((K == 2) ? 1 : 2);
		const int nlead = sd.opt ? (K - nfree) : ((K == 4) ? 1 : 0);
		float lead[1] = { sd.rates[0] };
		int fmap[2] = { nlead, nlead + 1 };
		float fr[2] = { sd.rates[nlead], (nfree > 1) ? sd.rates[nlead + 1] : 0.f };

		float r[4];
		PassEval pe;
		auto resid = [&](const float* f, float* F) -> bool {
			if (sd.opt) {
				for (int i = 0; i < 4; ++i) r[i] = sd.rates[i];
				for (int i = 0; i < nfree; ++i) r[nlead + i] = f[i];
				// The blend tail keeps the pass-tick yaw exact regardless of
				// the biases - just evaluate.
				if (!EvalPass(sd.ticks, sd.pass_frac, sd.side0, sd.splits, sd.nsplits, r, pe))
					return false;
			} else {
				if (!BuildRatesFromYaw(n, K, lo, hi, T_w, lead, nlead, f, nfree, r))
					return false;
				if (!EvalPass(sd.ticks, sd.pass_frac, sd.side0, sd.splits, sd.nsplits, r, pe))
					return false;
			}
			const Vector d = pe.pass - virt;
			F[0] = Dot(g_search.e1, d);
			F[1] = Dot(g_search.e2, d);
			return true;
		};

		float F[2];
		if (!resid(fr, F)) {
			g_search.target_pos = target->pos;   // undo the virt tracking
			return false;
		}
		float err = sqrtf(F[0] * F[0] + F[1] * F[1]);
		float best_r[4];
		memcpy(best_r, r, sizeof(best_r));
		float best_err = err;
		float lambda = 0.05f;
		const float h = 0.04f;
		for (int it = 0; it < 24 && err > 0.02f; ++it) {
			// Failure-tolerant FD, same as the search's: infeasible probes are
			// walls, not exits (aborting here froze every correction round at
			// "RESOLVE-UNCHANGED").
			float J[2][2] = {};
			bool have_col[2] = { false, false };
			for (int i = 0; i < nfree; ++i) {
				for (int attempt = 0; attempt < 2 && !have_col[i]; ++attempt) {
					float f2[2] = { fr[0], (nfree > 1) ? fr[1] : 0.f };
					const int bi = fmap[i];
					const float want = f2[i] + ((attempt == 0) ? h : -h);
					const float clamped = Clampf(want, lo[bi], hi[bi]);
					const float hs = clamped - f2[i];
					if (fabsf(hs) < 1e-6f)
						continue;
					f2[i] = clamped;
					float F2[2];
					if (resid(f2, F2)) {
						J[0][i] = (F2[0] - F[0]) / hs;
						J[1][i] = (F2[1] - F[1]) / hs;
						have_col[i] = true;
					}
				}
			}
			if (!have_col[0] && (nfree < 2 || !have_col[1]))
				break;
			bool stepped = false;
			for (int attempt = 0; attempt < 6 && !stepped; ++attempt) {
				float d[2] = {};
				if (nfree == 1) {
					const float a = J[0][0] * J[0][0] + J[1][0] * J[1][0] + lambda;
					d[0] = -(J[0][0] * F[0] + J[1][0] * F[1]) / a;
				} else {
					const float a00 = J[0][0] * J[0][0] + J[1][0] * J[1][0] + lambda;
					const float a11 = J[0][1] * J[0][1] + J[1][1] * J[1][1] + lambda;
					const float a01 = J[0][0] * J[0][1] + J[1][0] * J[1][1];
					const float b0 = -(J[0][0] * F[0] + J[1][0] * F[1]);
					const float b1 = -(J[0][1] * F[0] + J[1][1] * F[1]);
					const float det = a00 * a11 - a01 * a01;
					if (fabsf(det) < 1e-12f) { lambda *= 5.f; continue; }
					d[0] = (b0 * a11 - b1 * a01) / det;
					d[1] = (b1 * a00 - b0 * a01) / det;
				}
				const float m = fmaxf(fabsf(d[0]), fabsf(d[1]));
				if (m > 1.f) { d[0] /= m; d[1] /= m; }
				float f2[2] = { fr[0], (nfree > 1) ? fr[1] : 0.f };
				for (int i = 0; i < nfree; ++i)
					f2[i] = Clampf(f2[i] + d[i], lo[fmap[i]], hi[fmap[i]]);
				float F2[2];
				if (resid(f2, F2)) {
					const float e2 = sqrtf(F2[0] * F2[0] + F2[1] * F2[1]);
					if (e2 < err) {
						for (int i = 0; i < nfree; ++i) fr[i] = f2[i];
						F[0] = F2[0]; F[1] = F2[1];
						err = e2;
						if (e2 < best_err) {
							best_err = e2;
							memcpy(best_r, r, sizeof(best_r));
						}
						lambda = fmaxf(lambda * 0.4f, 1e-4f);
						stepped = true;
					}
				}
				if (!stepped)
					lambda *= 5.f;
			}
			if (!stepped)
				break;
		}

		memcpy(sd.rates, best_r, sizeof(float) * 4);
		g_search.target_pos = target->pos;   // undo the virt tracking (logs read this)
		MarkDirty();   // the resim measures the corrected schedule
		return true;
	}

	void StartCorrect(int seg_index) {
		const bool keep = g_correct.auto_run;
		g_correct = CorrectCtx();
		g_correct.auto_run = keep;
		if (seg_index < 0 || seg_index >= static_cast<int>(g_segs.size()))
			return;
		if (!g_segs[seg_index].is_solver || !g_segs[seg_index].solver.solved)
			return;
		const BspWorld::BoardTarget* target = BspWorld::GetTarget(g_segs[seg_index].solver.target);
		if (!target)
			return;
		g_correct.seg = seg_index;
		g_correct.virt = target->pos;
		g_correct.skip = Prediction::SimBusy() ? 1 : 0;   // in-flight sim = stale rates
		g_correct.active = true;
	}

	void FinishCorrect(const char* how, float final_err) {
		g_correct.active = false;
		g_correct.done = true;
		// Never end on rates worse than the best measured - restore them.
		if (g_correct.have_best && final_err > g_correct.best_err + 0.05f
			&& g_correct.seg >= 0 && g_correct.seg < static_cast<int>(g_segs.size())
			&& g_segs[g_correct.seg].is_solver) {
			memcpy(g_segs[g_correct.seg].solver.rates, g_correct.best_rates, sizeof(float) * 4);
			MarkDirty();
			Sfmt(g_correct.note, "real-sim check: %s at %.2f u; restored best rates (%.2f u).",
				how, final_err, g_correct.best_err);
			return;
		}
		Sfmt(g_correct.note, "real-sim check: %s - pass %.2f u from target (%d round(s)).",
			how, final_err, g_correct.rounds);
	}

	// One landed sim = one measurement. If the REAL pass misses, shift the
	// virtual target by a DAMPED fraction of the miss and re-solve the same
	// schedule (secant). Stops on convergence, no-change, or divergence -
	// always ending on the best rates seen.
	void CorrectPump() {
		if (!g_correct.active)
			return;
		if (g_correct.seg < 0 || g_correct.seg >= static_cast<int>(g_segs.size())
			|| !g_segs[g_correct.seg].is_solver || !g_segs[g_correct.seg].solver.solved) {
			g_correct.active = false;   // segment vanished under us
			return;
		}
		if (g_correct.skip > 0) {
			g_correct.skip--;
			return;
		}
		SolverData& sd = g_segs[g_correct.seg].solver;
		const BspWorld::BoardTarget* target = BspWorld::GetTarget(sd.target);
		if (!target || !g_realpass.computed || !g_realpass.crossed) {
			g_correct.active = false;
			g_correct.done = true;
			Sfmt(g_correct.note, "correction stopped: the real path never reaches the pass "
				"event%s.",
				g_realpass.jump_missing
					? " (the start jump never fired - was the sim start actually on the ground?)"
					: "");
			return;
		}
		if (!g_realpass.on_target) {
			g_correct.active = false;
			g_correct.done = true;
			Sfmt(g_correct.note, "correction stopped: the real path boards another tagged "
				"surface first (brush %d plane %d, tick %d) - structural; re-search (the model "
				"now rejects early-boarding candidates).",
				g_realpass.hit_brush, g_realpass.hit_plane, g_realpass.tick);
			return;
		}
		const float err = g_realpass.err;
		if (err < g_correct.best_err) {
			g_correct.best_err = err;
			memcpy(g_correct.best_rates, sd.rates, sizeof(float) * 4);
			g_correct.have_best = true;
		}
		if (err <= 0.25f) { FinishCorrect("converged", err); return; }
		if (g_correct.rounds >= 3) { FinishCorrect("round limit", err); return; }
		if (g_correct.have_best && err > 2.f * g_correct.best_err + 1.f) {
			FinishCorrect("diverging", err);
			return;
		}

		// Damped secant (full-gain secant oscillated between contact ticks).
		g_correct.virt = g_correct.virt + Scale(target->pos - g_realpass.pass, 0.7f);
		float before[4];
		memcpy(before, sd.rates, sizeof(before));
		if (!ReSolveApplied(g_correct.seg, g_correct.virt)) {
			FinishCorrect("couldn't re-solve", err);
			return;
		}
		float delta = 0.f;
		for (int i = 0; i < 4; ++i)
			delta = fmaxf(delta, fabsf(sd.rates[i] - before[i]));
		if (delta < 1e-4f) {
			FinishCorrect("re-solve unchanged (model at its floor)", err);
			return;
		}
		g_correct.rounds++;
	}

	void StartVerify(int seg_index) {
		g_verify = VerifyCtx();
		if (seg_index < 0 || seg_index >= static_cast<int>(g_segs.size())
			|| g_search.results.empty())
			return;
		g_verify.seg = seg_index;
		g_verify.idx = 0;
		g_correct.active = false;   // verification owns the sim pipeline
		ApplySolution(seg_index, g_search.results[0]);
		g_verify.skip = Prediction::SimBusy() ? 1 : 0;
		g_verify.active = true;
	}

	// ---------------------------------------------- solution ranking (tiers) --
	// The user picks up to three sort criteria; equal values (within a sane
	// quantum) fall through to the next tier. real_on_target always partitions
	// first - a deflected line never outranks a clean one.
	int g_sort_key[3] = { 1, 0, 2 };   // default: Speed > Real miss > Yaw err
	const char* const kSortKeyItems = "Real miss (u)\0Speed (u/s)\0Yaw err (deg)\0Board loss (u/s)\0Model miss (u)\0";

	float SortValue(const RouteSolution& s, int key) {
		switch (key) {
		case 1: return -s.speed;                              // faster first
		case 2: return (s.real_yaw_err >= 0.f) ? s.real_yaw_err : 999.f;
		case 3: return s.bump_loss;                           // cleaner board first
		case 4: return s.pass_err;
		default: return (s.real_err >= 0.f) ? s.real_err : 1e9f;
		}
	}
	int SortBucket(const RouteSolution& s, int key) {
		if (key < 0 || key > 4)
			key = 0;
		float v = SortValue(s, key);
		float q = 0.05f;                    // miss/yaw quantum
		if (key == 1 || key == 3) q = 0.5f; // speed/loss quantum (u/s)
		// CLAMP before the int cast. Unmeasured entries carry 1e9 sentinels;
		// 1e9/0.05 = 2e10 does NOT fit an int and float->int overflow is UB -
		// the optimizer may evaluate it DIFFERENTLY at different inline sites,
		// which makes the same entry compare inconsistently, and std::sort
		// with an inconsistent comparator writes OUT OF BOUNDS: silent heap-
		// corruption death, no catchable exception (the sort-click crash).
		if (v > 1e6f) v = 1e6f;
		if (v < -1e6f) v = -1e6f;
		return static_cast<int>(floorf(v / q));   // |result| <= 2e7: safe
	}
	void SortVerifiedResults() {
		std::sort(g_search.results.begin(), g_search.results.end(),
			[](const RouteSolution& a, const RouteSolution& b) {
				if (a.real_on_target != b.real_on_target) return a.real_on_target;
				for (int i = 0; i < 3; ++i) {
					const int ba = SortBucket(a, g_sort_key[i]);
					const int bb = SortBucket(b, g_sort_key[i]);
					if (ba != bb) return ba < bb;
				}
				if (a.real_err != b.real_err) return a.real_err < b.real_err;
				return a.speed > b.speed;
			});
	}

	// One landed sim = one candidate verified against the REAL engine.
	void VerifyPump() {
		if (!g_verify.active)
			return;
		if (g_verify.seg < 0 || g_verify.seg >= static_cast<int>(g_segs.size())
			|| g_verify.idx >= static_cast<int>(g_search.results.size())) {
			g_verify.active = false;
			return;
		}
		if (g_verify.skip > 0) {
			g_verify.skip--;
			return;
		}
		RouteSolution& s = g_search.results[g_verify.idx];
		const bool measured = g_realpass.computed && g_realpass.crossed;
		s.real_on_target = measured && g_realpass.on_target;
		// Record the honest miss even for deflected runs - the calibration
		// columns need the number; the on_target flag carries the verdict.
		s.real_err = measured ? g_realpass.err : 1e9f;
		s.real_tick = g_realpass.tick;
		s.real_bump_loss = g_realpass.bump_loss;
		s.real_yaw_err = measured ? g_realpass.yaw_err : -1.f;
		g_verify.idx++;
		if (g_verify.idx < static_cast<int>(g_search.results.size())) {
			ApplySolution(g_verify.seg, g_search.results[g_verify.idx]);
			return;
		}

		// All measured. NOTHING gets dropped - the calibration loop needs the
		// data. Sort by the user's tiered criteria; always write the
		// diagnostics log (model vs real for every candidate).
		SortVerifiedResults();
		int hit_tol = 0, hit_near = 0;
		for (const RouteSolution& r : g_search.results) {
			if (r.real_on_target && r.real_err <= g_search.tol)
				hit_tol++;
			else if (r.real_on_target && r.real_err <= 3.f)
				hit_near++;
		}
		WriteSearchLog();
		g_solution_pick = 0;
		g_verify.active = false;
		if (!g_search.results.empty()) {
			ApplySolution(g_verify.seg, g_search.results[0]);
			if (g_correct.auto_run)
				StartCorrect(g_verify.seg);
			char msg[512];   // holds a full MAX_PATH log path safely
			Sfmt(msg, "Solver: %d candidates real-measured: %d within tol, %d within 3u. "
				"Fastest within-tolerance line first, rest by real miss. Log: %s",
				static_cast<int>(g_search.results.size()), hit_tol, hit_near,
				g_searchlog_path[0] ? g_searchlog_path : "(write failed)");
			g_status = msg;
		}
	}

	// ------------------------------------------- calibration batch driver --
	std::string CalibDir() {
		char documents[MAX_PATH];
		if (FAILED(SHGetFolderPathA(nullptr, CSIDL_PERSONAL, nullptr, SHGFP_TYPE_CURRENT, documents)))
			return {};
		std::string base = std::string(documents) + "\\sourceTAS";
		CreateDirectoryA(base.c_str(), nullptr);
		std::string dir = base + "\\calibration";
		CreateDirectoryA(dir.c_str(), nullptr);
		return dir;
	}

	void BatchAppendf(const char* fmt, ...) {
		// Calibration lines are DATA: exact-size formatting, never truncated.
		va_list args;
		va_start(args, fmt);
		g_batch.log += FmtV(fmt, args);
		va_end(args);
	}

	// Search-space diagnostics: outcome counters + the closest rejects in full
	// detail. Auto-written when a search finds nothing; button-written anytime.
	void WriteSearchLog() {
		g_searchlog_path[0] = 0;
		const std::string dir = CalibDir();
		if (dir.empty())
			return;
		SYSTEMTIME st;
		GetLocalTime(&st);
		char name[128];
		Sfmt(name, "search_%04d%02d%02d_%02d%02d%02d.log",
			st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
		const std::string full = dir + "\\" + name;
		std::ofstream out(full, std::ios::trunc);
		if (!out)
			return;

		// Data lines: EXACT dynamic formatting (FmtStr) - never truncated,
		// never a fixed buffer to outgrow (one of those killed the process).
		out << FmtStr("== sourceTAS search diagnostics ==\n"
			"map=%s  strafes=%d%s  tol=%.2f  jobs=%d/%d  results=%d (of %d distinct post-dedupe)\n"
			"start: pos=(%.2f %.2f %.2f) vel=(%.2f %.2f %.2f) yaw=%.3f jump=%d\n"
			"target: pos=(%.2f %.2f %.2f) yaw=%.3f\n"
			"model in-use: dt=%.6f cap=%.1f amt=%.1f grav=%.1f jump_vz_input=%.2f\n"
			"colliders=%d corridor brushes (target brush first)\n"
			"outcomes: infeasible=%d  no_eval=%d  floor(>collect)=%d  tick_align=%d  violent_board=%d  deflect_target_brush=%d  deflect_world=%d  cap_dropped=%d\n"
			"closest positional miss: %.3f u   max_bump=%.1f\n\n",
			BspWorld::GetStatus().map, g_search.strafes,
			g_search.opt ? " OPTIMAL-BIAS (rates are biases; last one solved for EXACT yaw)" : "",
			g_search.tol,
			g_search.jobs_done, g_search.jobs_total,
			static_cast<int>(g_search.results.size()),
			g_search.dedupe_kept > 0 ? g_search.dedupe_kept
			                         : static_cast<int>(g_search.results.size()),
			g_search.start_pos.X, g_search.start_pos.Y, g_search.start_pos.Z,
			g_search.start_vel.X, g_search.start_vel.Y, g_search.start_vel.Z,
			g_search.start_yaw, g_search.jump_start,
			g_search.target_pos.X, g_search.target_pos.Y, g_search.target_pos.Z,
			g_search.target_yaw,
			g_search.dt, g_search.cap, g_search.accel_amt,
			g_search.gravity, g_jump_vz,
			static_cast<int>(g_search.bcolliders.size()),
			g_search.cnt_infeasible, g_search.cnt_no_contact, g_search.cnt_floor,
			g_search.reject_retime, g_search.cnt_bump, g_search.cnt_edge, g_search.early_hits,
			g_search.cap_dropped,
			g_search.reject_best < 1e8f ? g_search.reject_best : -1.f,
			g_search.max_bump);

		const size_t ndump = (std::min)(g_search.bcolliders.size(), static_cast<size_t>(12));
		if (ndump < g_search.bcolliders.size()) {
			out << FmtStr("collision world: %d corridor brushes (first %d listed)\n",
				static_cast<int>(g_search.bcolliders.size()), static_cast<int>(ndump));
		}
		for (size_t i = 0; i < ndump; ++i) {
			const SearchCtx::BrushCollider& c = g_search.bcolliders[i];
			out << FmtStr("brush collider %d: brush=%d planes=%d%s\n",
				static_cast<int>(i), c.brush, static_cast<int>(c.pn.size()),
				c.is_target ? "  TARGET (board face plane below)" : "");
			if (c.is_target) {
				Vector fn;
				float fd = 0.f;
				if (BspWorld::GetPlane(c.target_plane, &fn, &fd)) {
					// d_rest (through the target's rest origin) vs d_expand (the
					// brush plane + hull support offset): any gap here is a
					// target-seating offset that shifts board timing sub-tick.
					float d_rest = 0.f;
					for (size_t pi = 0; pi < c.pid.size(); ++pi)
						if (c.pid[pi] == c.target_plane) { d_rest = c.pd[pi]; break; }
					const float d_expand = fd + 16.f * fabsf(fn.X) + 16.f * fabsf(fn.Y)
						+ (fn.Z < 0.f ? 72.f * fabsf(fn.Z) : 0.f);
					out << FmtStr("  board face: plane=%d n=(%.4f %.4f %.4f) d_rest=%.4f d_expand=%.4f (gap %+0.4f)\n",
						c.target_plane, fn.X, fn.Y, fn.Z, d_rest, d_expand, d_rest - d_expand);
				}
			}
		}
		out << "\n";

		for (size_t i = 0; i < g_search.top_rejects.size(); ++i) {
			const SearchCtx::RejectRec& rec = g_search.top_rejects[i];
			out << FmtStr("[rej %02d] err=%.4f %s  N=%d n_eff=%d contact_tick=%d side0=%+d "
				"splits=%d,%d,%d wrap=%+d rates=%+.4f,%+.4f,%+.4f,%+.4f pass=(%.2f %.2f %.2f)\n",
				static_cast<int>(i), rec.err,
				rec.reason == 0 ? "FLOOR" : rec.reason == 1 ? "TICK-ALIGN" : "BUMP-LOSS",
				rec.N, rec.n_eff, rec.tick, rec.side0,
				rec.splits[0], rec.splits[1], rec.splits[2], rec.wrap,
				rec.rates[0], rec.rates[1], rec.rates[2], rec.rates[3],
				rec.pass.X, rec.pass.Y, rec.pass.Z);
		}

		const int nsol = (std::min)(static_cast<int>(g_search.results.size()), 100);
		if (nsol > 0)
			out << "\nsolutions (model + real for each - the calibration dataset):\n";
		for (int i = 0; i < nsol; ++i) {
			const RouteSolution& s = g_search.results[i];
			out << FmtStr("[sol %02d]%s err=%.4f real=%.3f N=%d frac=%.3f side0=%+d splits=%d,%d,%d "
				"wrap=%+d speed=%.1f bump=%dt/%.2f rates=%+.4f,%+.4f,%+.4f,%+.4f pass=(%.2f %.2f %.2f)\n",
				i, s.exact ? " EXACT" : " near ", s.pass_err, s.real_err, s.ticks, s.frac, s.side0,
				s.splits[0], s.splits[1], s.splits[2], s.wrap, s.speed,
				s.bump_ticks, s.bump_loss,
				s.rates[0], s.rates[1], s.rates[2], s.rates[3],
				s.pass_pos.X, s.pass_pos.Y, s.pass_pos.Z);
		}
		strncpy_s(g_searchlog_path, full.c_str(), _TRUNCATE);
	}

	// Per-tick model-vs-engine divergence trace for the applied schedule: the
	// FIRST tick they disagree - and in which component - names the exact
	// physics term that's wrong, from data instead of inference.
	void BatchAppendTrace(int seg) {
		if (seg < 0 || seg >= static_cast<int>(g_segs.size()))
			return;
		const SolverData& sd = g_segs[seg].solver;
		if (!sd.solved || seg >= static_cast<int>(g_starts.size()))
			return;
		Vector pos, vel;
		float yaw;
		if (!SolverStartState(seg, pos, vel, yaw))
			return;
		float interval = Prediction::LastDiag().interval_per_tick;
		if (interval <= 0.f) interval = 0.015f;
		g_search.dt = interval;
		g_search.cap = g_air_cap;
		g_search.accel_amt = g_air_accel * g_wishspeed * interval;
		g_search.gravity = g_gravity;
		if (sd.start_jump)
			PrepStartJump(vel);

		FastState s{ pos, vel, yaw };
		const int b = g_starts[seg];
		int k = 0;
		int first_div = -1;
		float worst = 0.f;
		Vector bfn;
		float bfd = 0.f;
		const BspWorld::BoardTarget* btgt = BspWorld::GetTarget(sd.target);
		const bool bface = btgt && BspWorld::GetPlane(btgt->plane, &bfn, nullptr);
		if (bface)
			bfd = Dot(bfn, btgt->pos);
		for (int t = 0; t < sd.ticks + 8 && b + t < static_cast<int>(g_states.size()); ++t) {
			while (k < sd.nsplits && t >= sd.splits[k]) k++;
			// The SAME collision integration the solver uses - the trace
			// compares the actual model, boards and slides included.
			const int side = (k % 2 == 0) ? sd.side0 : -sd.side0;
			const float yaw_next = sd.opt
				? OptYawStep(s.yaw, s.v, t, sd.ticks, (sd.blend < 1) ? 1 : sd.blend,
					sd.rates[k], side, sd.tyaw)
				: NormYaw(s.yaw - sd.rates[k]);
			Vector tsp[6];
			TickContact ttc[4];
			int tnc = 0;
			bool tgr = false;
			EngineMoveTick(s, yaw_next, side, tsp, ttc, &tnc, &tgr);
			const Vector ro = g_states[b + t].origin;
			const Vector rv = g_states[b + t].velocity;
			const float dp = sqrtf(Dot3(s.p - ro));
			const float dv = sqrtf(Dot3(s.v - rv));
			if (dp > worst)
				worst = dp;
			// Board-window forensics: distance to the target's rest plane and
			// normal velocity, model vs real, plus which brush plane the model
			// struck - pins seam/edge order questions with data.
			if (bface && t >= sd.ticks - 4 && t <= sd.ticks + 6) {
				char cbuf[96] = "";
				if (tnc > 0) {
					const SearchCtx::BrushCollider& cc = g_search.bcolliders[ttc[0].collider];
					Sfmt(cbuf, "  mdl-hit brush=%d pid=%d loss=%.1f%s", cc.brush,
						(ttc[0].plane >= 0 && ttc[0].plane < static_cast<int>(cc.pid.size()))
							? cc.pid[ttc[0].plane] : -1,
						ttc[0].loss, (tnc > 1) ? " +more" : "");
				}
				BatchAppendf("  brd t=%03d phi m=%+.3f r=%+.3f  n.v m=%+.1f r=%+.1f%s\n",
					t, Dot(bfn, s.p) - bfd, Dot(bfn, ro) - bfd,
					Dot(bfn, s.v), Dot(bfn, rv), cbuf);
			}
			if (first_div < 0 && dp > 0.1f) {
				first_div = t;
				BatchAppendf("  trace: FIRST divergence at local tick %d: dpos=%.3f (%+.3f %+.3f %+.3f) dvel=%.3f (%+.3f %+.3f %+.3f)\n"
				             "         model p=(%.2f %.2f %.2f) v=(%.1f %.1f %.1f) yaw=%.2f | real p=(%.2f %.2f %.2f) v=(%.1f %.1f %.1f) frame_yaw=%.2f\n",
					t, dp, s.p.X - ro.X, s.p.Y - ro.Y, s.p.Z - ro.Z,
					dv, s.v.X - rv.X, s.v.Y - rv.Y, s.v.Z - rv.Z,
					s.p.X, s.p.Y, s.p.Z, s.v.X, s.v.Y, s.v.Z, s.yaw,
					ro.X, ro.Y, ro.Z, rv.X, rv.Y, rv.Z,
					(b + t < static_cast<int>(g_frames.size())) ? g_frames[b + t].viewangles[1] : 0.f);
			}
			if ((t % 16) == 15)
				BatchAppendf("  trace t=%03d dpos=%.3f dvel=%.3f\n", t, dp, dv);
		}
		BatchAppendf("  trace: worst dpos=%.3f over %d ticks%s\n", worst, sd.ticks,
			first_div < 0 ? "  (MODEL==ENGINE within 0.1u pre-contact)" : "");
	}

	void BatchApplyCurrent() {
		const RouteSolution& sol = g_search.results[g_batch.idx];
		ApplySolution(g_batch.seg, sol);
		g_batch.round = 0;
		g_batch.virt = g_search.target_pos;
		BatchAppendf("[sol %03d] N=%d frac=%.3f side0=%+d nsplits=%d splits=%d,%d,%d wrap=%+d speed=%.1f\n"
		             "  model: rates=%+.4f,%+.4f,%+.4f,%+.4f  pass_err=%.4f  pass=(%.2f %.2f %.2f)\n",
			g_batch.idx, sol.ticks, sol.frac, sol.side0, sol.nsplits,
			sol.splits[0], sol.splits[1], sol.splits[2], sol.wrap, sol.speed,
			sol.rates[0], sol.rates[1], sol.rates[2], sol.rates[3],
			sol.pass_err, sol.pass_pos.X, sol.pass_pos.Y, sol.pass_pos.Z);
	}

	void FinishBatch(bool aborted) {
		if (!g_batch.active && g_batch.log.empty())
			return;
		g_batch.active = false;
		if (aborted)
			g_batch.log += "\n== ABORTED (partial data) ==\n";
		const std::string dir = CalibDir();
		g_batch.path[0] = 0;
		if (!dir.empty()) {
			SYSTEMTIME st;
			GetLocalTime(&st);
			char name[128];
			Sfmt(name, "calib_%04d%02d%02d_%02d%02d%02d.log",
				st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
			const std::string full = dir + "\\" + name;
			std::ofstream out(full, std::ios::trunc);
			if (out) {
				out << g_batch.log;
				strncpy_s(g_batch.path, full.c_str(), _TRUNCATE);
			}
		}
		g_batch.log.clear();
		// Put the user's selected solution back and resume normal behavior.
		if (g_batch.seg >= 0 && g_batch.seg < static_cast<int>(g_segs.size())
			&& g_solution_pick >= 0 && g_solution_pick < static_cast<int>(g_search.results.size())) {
			ApplySolution(g_batch.seg, g_search.results[g_solution_pick]);
			if (g_correct.auto_run)
				StartCorrect(g_batch.seg);
		}
		g_status = std::string("Calibration log written: ")
			+ (g_batch.path[0] ? g_batch.path : "(write FAILED)");
	}

	void StartBatch() {
		if (!g_search.done || g_search.results.empty()
			|| g_search.seg < 0 || g_search.seg >= static_cast<int>(g_segs.size()))
			return;
		g_batch = BatchCtx();
		g_batch.seg = g_search.seg;
		g_batch.idx = 0;
		g_correct.active = false;   // the batch owns the correction loop
		g_correct.done = false;
		BatchAppendf("== sourceTAS solver calibration ==\n"
		             "map=%s  solutions=%d  strafes=%d%s  accept_tol=%.2f  correct_target=0.25\n"
		             "start: pos=(%.2f %.2f %.2f) vel=(%.2f %.2f %.2f) yaw=%.3f  jump_at_start=%d\n"
		             "target: pos=(%.2f %.2f %.2f) yaw=%.3f\n"
		             "model: dt=%.6f cap=%.1f accel_amt=%.1f (sv_airaccelerate=%.0f wishspeed=%.0f) gravity=%.0f\n\n",
			BspWorld::GetStatus().map, static_cast<int>(g_search.results.size()),
			g_search.strafes,
			g_search.opt ? " OPTIMAL-BIAS (rates are biases; last one solved for EXACT yaw)" : "",
			g_sol_pos_tol,
			g_search.start_pos.X, g_search.start_pos.Y, g_search.start_pos.Z,
			g_search.start_vel.X, g_search.start_vel.Y, g_search.start_vel.Z,
			g_search.start_yaw, g_search.jump_start,
			g_search.target_pos.X, g_search.target_pos.Y, g_search.target_pos.Z,
			g_search.target_yaw,
			g_search.dt, g_search.cap, g_search.accel_amt, g_air_accel, g_wishspeed, g_gravity);
		if (g_search.have_face)
			BatchAppendf("event=EXACT-POINT closest approach (collision-truncated)  "
				"face_n=(%.4f %.4f %.4f) d=%.2f\n",
				g_search.face_n.X, g_search.face_n.Y, g_search.face_n.Z, g_search.face_d);
		else
			BatchAppendf("event=HEIGHT CROSSING (no face plane)\n");
		BatchAppendf("in-use: amt=%.1f grav=%.1f jump_vz_input=%.2f brush_colliders=%d\n\n",
			g_search.accel_amt, g_search.gravity, g_jump_vz,
			static_cast<int>(g_search.bcolliders.size()));
		BatchApplyCurrent();
		g_batch.skip = Prediction::SimBusy() ? 1 : 0;
		g_batch.active = true;
	}

	void BatchNext() {
		g_batch.idx++;
		if (g_batch.idx >= static_cast<int>(g_search.results.size())) {
			FinishBatch(false);
			return;
		}
		BatchApplyCurrent();
	}

	// One landed sim = one measurement of the current solution/round.
	void BatchPump() {
		if (!g_batch.active)
			return;
		if (g_batch.seg < 0 || g_batch.seg >= static_cast<int>(g_segs.size())
			|| !g_search.done || g_batch.idx >= static_cast<int>(g_search.results.size())) {
			FinishBatch(true);
			return;
		}
		if (g_batch.skip > 0) {
			g_batch.skip--;
			return;
		}
		const BspWorld::BoardTarget* target = BspWorld::GetTarget(g_segs[g_batch.seg].solver.target);
		if (!target) {
			FinishBatch(true);
			return;
		}

		if (!g_realpass.computed || !g_realpass.crossed) {
			BatchAppendf("  r%d: NO PASS EVENT  closest_phi=%.2f  closest_dz=%.2f  min3d=%.2f%s\n  FAIL\n\n",
				g_batch.round,
				g_realpass.closest_phi < 1e8f ? g_realpass.closest_phi : -1.f,
				g_realpass.closest_dz < 1e8f ? g_realpass.closest_dz : -1.f,
				g_realpass.min_dist < 1e8f ? g_realpass.min_dist : -1.f,
				g_realpass.jump_missing ? "  JUMP_MISSING" : "");
			BatchNext();
			return;
		}

		BatchAppendf("  r%d: real_err=%.4f  pass=(%.2f %.2f %.2f)  tick=%d  yaw_err=%.4f  drift=%.3f%s\n",
			g_batch.round, g_realpass.err,
			g_realpass.pass.X, g_realpass.pass.Y, g_realpass.pass.Z,
			g_realpass.tick, g_realpass.yaw_err,
			g_realpass.drift >= 0.f ? g_realpass.drift : -1.f,
			g_realpass.on_target ? "" : "  EARLY-BOARD");
		if (g_realpass.on_target && g_realpass.bump_tick >= 0)
			BatchAppendf("      bump: tick=%d (%d before the pass)  clip~%.2f u/s\n",
				g_realpass.bump_tick, g_realpass.bump_ticks, g_realpass.bump_loss);
		if (g_batch.round == 0) {
			BatchAppendf("  probe: vz0=%.2f  grav_fit=%.1f  max_add=%.2f (model amt=%.1f)  cap_fit=%.2f (model cap=%.1f)\n",
				g_realpass.real_vz0, g_realpass.grav_fit,
				g_realpass.max_add, g_search.accel_amt,
				g_realpass.cap_fit, g_search.cap);
			BatchAppendTrace(g_batch.seg);
		}
		if (!g_realpass.on_target) {
			BatchAppendf("  FAIL early board: brush=%d plane=%d tick=%d (%.1f u from target)\n\n",
				g_realpass.hit_brush, g_realpass.hit_plane, g_realpass.tick, g_realpass.err);
			BatchNext();
			return;
		}

		if (g_realpass.err <= 0.25f || g_batch.round >= 4) {
			BatchAppendf("  DONE err=%.4f rounds=%d%s\n\n", g_realpass.err, g_batch.round,
				g_realpass.err <= 0.25f ? "  PERFECT" : "");
			BatchNext();
			return;
		}

		g_batch.virt = g_batch.virt + Scale(target->pos - g_realpass.pass, 0.7f);
		SolverData& sd = g_segs[g_batch.seg].solver;
		float before[4];
		memcpy(before, sd.rates, sizeof(before));
		if (!ReSolveApplied(g_batch.seg, g_batch.virt)) {
			BatchAppendf("  RESOLVE FAILED at round %d\n\n", g_batch.round + 1);
			BatchNext();
			return;
		}
		float delta = 0.f;
		for (int i = 0; i < 4; ++i)
			delta = fmaxf(delta, fabsf(sd.rates[i] - before[i]));
		if (delta < 1e-4f) {
			BatchAppendf("  DONE err=%.4f rounds=%d  RESOLVE-UNCHANGED\n\n",
				g_realpass.err, g_batch.round);
			BatchNext();
			return;
		}
		BatchAppendf("  resolve%d: virt=(%.2f %.2f %.2f)  rates=%+.4f,%+.4f,%+.4f,%+.4f\n",
			g_batch.round + 1, g_batch.virt.X, g_batch.virt.Y, g_batch.virt.Z,
			sd.rates[0], sd.rates[1], sd.rates[2], sd.rates[3]);
		g_batch.round++;
	}

	bool TargetItemGetter(void*, int index, const char** out_text) {
		static char buffer[128];
		const BspWorld::BoardTarget* t = BspWorld::GetTarget(index);
		if (!t)
			return false;
		Sfmt(buffer, "#%d  %.0f %.0f %.0f  yaw %.0f", index, t->pos.X, t->pos.Y, t->pos.Z, t->yaw);
		*out_text = buffer;
		return true;
	}

	void InsertSegment(EditSegment segment) {
		const int at = (g_sel >= 0 && g_sel < static_cast<int>(g_segs.size())) ? g_sel + 1
			: static_cast<int>(g_segs.size());
		g_segs.insert(g_segs.begin() + at, std::move(segment));
		g_sel = at;
		Layout();
		if (g_sel < static_cast<int>(g_starts.size()))
			g_cursor = g_starts[g_sel];
		MarkDirty();
	}

	void SplitOverrides(EditSegment& prefix, EditSegment& suffix, int local) {
		suffix.jump_ovr.clear();
		for (const std::pair<int, int>& o : prefix.jump_ovr)
			if (o.first >= local)
				suffix.jump_ovr.push_back({ o.first - local, o.second });
		prefix.jump_ovr.erase(
			std::remove_if(prefix.jump_ovr.begin(), prefix.jump_ovr.end(),
				[local](const std::pair<int, int>& o) { return o.first >= local; }),
			prefix.jump_ovr.end());
	}

	void SplitAtCursor() {
		const int k = SegmentAtTick(g_cursor);
		if (k < 0 || k >= static_cast<int>(g_segs.size()))
			return;
		if (g_segs[k].is_solver && !g_segs[k].raw) {
			g_status = "Solver segments can't be split - bake to raw frames first, or re-search.";
			return;
		}
		const int local = g_cursor - g_starts[k];
		const int total = g_segs[k].Ticks();
		if (local <= 0 || local >= total) {
			g_status = "Cursor is on a segment boundary - nothing to split.";
			return;
		}

		EditSegment suffix = g_segs[k];
		EditSegment& prefix = g_segs[k];
		if (prefix.raw) {
			suffix.frames.assign(prefix.frames.begin() + local, prefix.frames.end());
			prefix.frames.resize(local);
		} else {
			prefix.gen.ticks = local;
			suffix.gen.ticks = total - local;
			suffix.gen.yaw_abs = false;   // suffix continues from the split state
			SplitOverrides(prefix, suffix, local);
		}
		g_segs.insert(g_segs.begin() + k + 1, std::move(suffix));
		g_sel = k + 1;
		g_status = "Split at tick " + std::to_string(g_cursor) + ".";
		MarkDirty();
	}

	// Forward-only editing: a parameter change while the playhead is inside the
	// segment auto-splits there - the prefix keeps the OLD values (the path
	// behind the playhead cannot move), the suffix gets the NEW ones.
	void AutoSplitApply(int k, int local, const GenParams& before, const GenParams& after) {
		EditSegment& seg = g_segs[k];

		EditSegment suffix;
		suffix.raw = false;
		suffix.gen = after;
		int suffix_ticks = after.ticks - local;
		if (suffix_ticks < 1) suffix_ticks = 1;
		suffix.gen.ticks = suffix_ticks;
		suffix.gen.yaw_abs = false;

		SplitOverrides(seg, suffix, local);

		seg.gen = before;
		seg.gen.ticks = local;

		g_segs.insert(g_segs.begin() + k + 1, std::move(suffix));
		g_sel = k + 1;
		g_status = "Auto-split at playhead - the change applies forward only.";
	}

	// ------------------------------------------------------------ project io --
	std::string ProjectDir() {
		char documents[MAX_PATH];
		if (FAILED(SHGetFolderPathA(nullptr, CSIDL_PERSONAL, nullptr, SHGFP_TYPE_CURRENT, documents)))
			return {};
		std::string base = std::string(documents) + "\\sourceTAS";
		CreateDirectoryA(base.c_str(), nullptr);
		std::string dir = base + "\\projects";
		CreateDirectoryA(dir.c_str(), nullptr);
		return dir;
	}

	std::string SanitizeName(const std::string& name) {
		std::string out;
		for (char c : name) {
			if (static_cast<unsigned char>(c) < 32)
				continue;
			out += std::strchr("\\/:*?\"<>|", c) ? '_' : c;
		}
		const size_t first = out.find_first_not_of(" .");
		const size_t last = out.find_last_not_of(" .");
		if (first == std::string::npos)
			return {};
		return out.substr(first, last - first + 1);
	}

	void RefreshFiles() {
		g_files.clear();
		g_sel_file = -1;
		const std::string dir = ProjectDir();
		if (dir.empty())
			return;
		WIN32_FIND_DATAA find = {};
		HANDLE handle = FindFirstFileA((dir + "\\*.tasproj").c_str(), &find);
		if (handle == INVALID_HANDLE_VALUE)
			return;
		do {
			if (!(find.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY))
				g_files.push_back(find.cFileName);
		} while (FindNextFileA(handle, &find));
		FindClose(handle);
	}

	constexpr char kProjMagic[4] = { 'S', 'T', 'P', 'J' };
	// v2-v4: strafe-mode GenParams eras (see loaders). v5: single keyboard
	// segment type with auto strafe key. v6: segment-type byte (0 gen / 1 raw /
	// 2 solver), gen gains no_w_air, solver segments append their solution.
	// v7/v8: first-generation solver eras (couple, contact fields) - read and
	// discarded. v9: rebuilt solver block (start_jump, pass_frac, model pass).
	// v10: optimal strafe - gen gains opt_dir, solver block gains the opt flag
	// (rates[] hold per-phase biases when set). v11: gen gains opt_period
	// (straight-sync mode); solver gains blend + tyaw (the opt yaw-blend tail).
	constexpr uint32_t kProjVersion = 11;

	template <typename T>
	void W(std::ofstream& o, const T& v) { o.write(reinterpret_cast<const char*>(&v), sizeof(T)); }
	template <typename T>
	bool R(std::ifstream& i, T& v) { return static_cast<bool>(i.read(reinterpret_cast<char*>(&v), sizeof(T))); }

	void WriteFrame(std::ofstream& o, const Frame& f) {
		o.write(reinterpret_cast<const char*>(f.viewangles), sizeof(f.viewangles));
		W(o, f.forwardmove); W(o, f.sidemove); W(o, f.upmove);
		W(o, f.buttons); W(o, f.impulse); W(o, f.mousedx); W(o, f.mousedy);
	}
	bool ReadFrame(std::ifstream& i, Frame& f) {
		return static_cast<bool>(i.read(reinterpret_cast<char*>(f.viewangles), sizeof(f.viewangles)))
			&& R(i, f.forwardmove) && R(i, f.sidemove) && R(i, f.upmove)
			&& R(i, f.buttons) && R(i, f.impulse) && R(i, f.mousedx) && R(i, f.mousedy);
	}

	bool SaveProject() {
		const std::string dir = ProjectDir();
		std::string base = SanitizeName(g_name);
		if (dir.empty() || base.empty()) { g_status = "Invalid project name."; return false; }

		std::ofstream out(dir + "\\" + base + ".tasproj", std::ios::binary | std::ios::trunc);
		if (!out) { g_status = "Couldn't write project file."; return false; }

		out.write(kProjMagic, sizeof(kProjMagic));
		W(out, kProjVersion);

		const uint8_t av = g_anchor.valid ? 1 : 0, ad = g_anchor.ducked ? 1 : 0;
		W(out, av);
		out.write(reinterpret_cast<const char*>(&g_anchor.origin), sizeof(float) * 3);
		out.write(reinterpret_cast<const char*>(&g_anchor.velocity), sizeof(float) * 3);
		W(out, g_anchor.pitch); W(out, g_anchor.yaw); W(out, ad); W(out, g_anchor.stamina);
		W(out, g_air_cap); W(out, g_air_accel); W(out, g_wishspeed); W(out, g_min_eff);

		const uint32_t nsegs = static_cast<uint32_t>(g_segs.size());
		W(out, nsegs);
		for (const EditSegment& s : g_segs) {
			const uint8_t type = s.raw ? 1 : (s.is_solver ? 2 : 0);
			W(out, type);
			if (s.raw) {
				const uint32_t n = static_cast<uint32_t>(s.frames.size());
				W(out, n);
				for (const Frame& f : s.frames)
					WriteFrame(out, f);
			} else {
				const GenParams& g = s.gen;
				const uint8_t kw = g.key_w, ka = g.key_a, ks = g.key_s, kd = g.key_d;
				const uint8_t autok = g.auto_key ? 1 : 0, yabs = g.yaw_abs ? 1 : 0, duck = g.duck ? 1 : 0;
				const uint8_t noW = g.no_w_air ? 1 : 0;
				W(out, g.ticks);
				W(out, kw); W(out, ka); W(out, ks); W(out, kd); W(out, autok);
				W(out, g.yaw_mode); W(out, g.yaw_rate); W(out, yabs); W(out, g.yaw_start);
				W(out, g.pitch_mode); W(out, g.pitch_val); W(out, g.pitch_mult);
				W(out, duck); W(out, g.jump_mode); W(out, noW);
				W(out, g.opt_dir);     // v10
				W(out, g.opt_period);  // v11

				const uint32_t novr = static_cast<uint32_t>(s.jump_ovr.size());
				W(out, novr);
				for (const std::pair<int, int>& o : s.jump_ovr) {
					const int32_t local = o.first;
					const uint8_t val = static_cast<uint8_t>(o.second);
					W(out, local); W(out, val);
				}

				if (s.is_solver) {
					const SolverData& sd = s.solver;
					const uint8_t solved = sd.solved ? 1 : 0;
					const uint8_t sjump = sd.start_jump ? 1 : 0;
					const uint8_t sopt = sd.opt ? 1 : 0;
					W(out, sd.target); W(out, sd.strafes); W(out, solved);
					W(out, sd.ticks); W(out, sd.side0); W(out, sd.nsplits);
					for (int i = 0; i < 3; ++i) W(out, sd.splits[i]);
					for (int i = 0; i < 4; ++i) W(out, sd.rates[i]);
					W(out, sjump);
					W(out, sd.pass_frac);
					W(out, sd.model_pass.X); W(out, sd.model_pass.Y); W(out, sd.model_pass.Z);
					W(out, sd.model_err);
					W(out, sopt);        // v10
					W(out, sd.blend);    // v11
					W(out, sd.tyaw);     // v11
				}
			}
		}

		g_status = out ? ("Saved " + base + ".tasproj.") : "Write failed.";
		RefreshFiles();
		return static_cast<bool>(out);
	}

	// Deserialized fields feed array indexing in the solver, the provider (on
	// the GAME thread) and the UI - a corrupt or hand-edited file must never
	// be able to index out of bounds. Clamp everything into its legal range.
	void SanitizeGen(GenParams& g) {
		if (g.ticks < 1) g.ticks = 1;
		if (g.ticks > 100000) g.ticks = 100000;
		if (g.yaw_mode < 0 || g.yaw_mode > 3) g.yaw_mode = 1;
		g.opt_dir = (g.opt_dir < 0) ? -1 : 1;
		if (g.opt_period < 1) g.opt_period = 1;
		if (g.opt_period > 128) g.opt_period = 128;
		if (!(g.yaw_rate == g.yaw_rate)) g.yaw_rate = 0.f;       // NaN guard
		if (g.yaw_rate < -180.f) g.yaw_rate = -180.f;
		if (g.yaw_rate > 180.f) g.yaw_rate = 180.f;
		if (g.pitch_mode < 0 || g.pitch_mode > 1) g.pitch_mode = PM_Const;
		if (g.jump_mode < 0 || g.jump_mode > 2) g.jump_mode = JM_None;
	}
	void SanitizeSolver(SolverData& sd) {
		if (sd.strafes < 1) sd.strafes = 1;
		if (sd.strafes > 4) sd.strafes = 4;
		if (sd.nsplits < 0) sd.nsplits = 0;
		if (sd.nsplits > 3) sd.nsplits = 3;                      // splits[3]/rates[4]
		sd.side0 = (sd.side0 < 0) ? -1 : 1;
		if (sd.ticks < 0) sd.ticks = 0;
		if (sd.ticks > Prediction::kMaxSimTicks) sd.ticks = Prediction::kMaxSimTicks;
		for (int i = 0; i < 3; ++i) {
			if (sd.splits[i] < 0) sd.splits[i] = 0;
			if (sd.splits[i] > Prediction::kMaxSimTicks) sd.splits[i] = Prediction::kMaxSimTicks;
		}
		for (int i = 0; i < 4; ++i) {
			if (!(sd.rates[i] == sd.rates[i])) sd.rates[i] = 0.f;  // NaN guard
			if (sd.rates[i] < -180.f) sd.rates[i] = -180.f;
			if (sd.rates[i] > 180.f) sd.rates[i] = 180.f;
		}
		if (!(sd.pass_frac == sd.pass_frac)) sd.pass_frac = 1.f;
		if (sd.blend < 1) sd.blend = 1;
		if (sd.blend > 24) sd.blend = 24;
		if (!(sd.tyaw == sd.tyaw)) sd.tyaw = 0.f;                // NaN guard
		if (sd.ticks == 0)
			sd.solved = false;                                   // an empty schedule isn't a solution
	}

	// v1: the very first generator layout.
	bool ReadGenV1(std::ifstream& in, GenParams& g) {
		int yaw_mode = 0;
		float fmove = 450.f, smove = 0.f;
		uint8_t yabs = 0, duck = 0;
		if (!R(in, g.ticks) || !R(in, yaw_mode) || !R(in, g.yaw_rate) || !R(in, yabs) || !R(in, g.yaw_start) ||
			!R(in, g.pitch_mode) || !R(in, g.pitch_val) || !R(in, g.pitch_mult) ||
			!R(in, fmove) || !R(in, smove) || !R(in, duck) || !R(in, g.jump_mode))
			return false;
		g.yaw_abs = yabs != 0;
		g.duck = duck != 0;
		g.yaw_rate = -g.yaw_rate;   // old + = left; screen + = right
		if (yaw_mode >= 2) {
			// Old strafe modes: became view-turn segments with the auto key.
			g.yaw_mode = 1;
			g.key_w = g.key_a = g.key_s = g.key_d = false;
		} else {
			g.yaw_mode = yaw_mode;
			g.key_w = fmove > 100.f;
			g.key_s = fmove < -100.f;
			g.key_d = smove > 100.f;
			g.key_a = smove < -100.f;
		}
		return true;
	}

	// v2-v4: the strafe-mode era layout; migrated onto the keyboard model
	// (strafe segments become view-turn segments with the auto strafe key).
	bool ReadGenLegacy(std::ifstream& in, GenParams& g, uint32_t version) {
		int mode = 0, strafe_key = 0;
		float turn_total = 0.f, bias = 0.f;
		uint8_t lock = 0, yabs = 0, duck = 0, kw = 0, ka = 0, ks = 0, kd = 0;
		if (!R(in, g.ticks) || !R(in, mode) ||
			!R(in, turn_total) || !R(in, lock) || !R(in, strafe_key) || !R(in, bias) ||
			!R(in, kw) || !R(in, ka) || !R(in, ks) || !R(in, kd) ||
			!R(in, g.yaw_mode) || !R(in, g.yaw_rate) || !R(in, yabs) || !R(in, g.yaw_start) ||
			!R(in, g.pitch_mode) || !R(in, g.pitch_val) || !R(in, g.pitch_mult) ||
			!R(in, duck) || !R(in, g.jump_mode))
			return false;
		g.yaw_abs = yabs != 0;
		g.duck = duck != 0;
		g.key_w = kw != 0; g.key_a = ka != 0; g.key_s = ks != 0; g.key_d = kd != 0;
		if (version == 2)
			turn_total = -turn_total;      // v2 stored + = left
		if (version <= 3)
			g.yaw_rate = -g.yaw_rate;      // <=v3 stored + = left
		if (mode == 1) {                   // old auto-strafe segment
			g.yaw_mode = 1;
			g.yaw_rate = (g.ticks > 0) ? turn_total / static_cast<float>(g.ticks) : 0.f;
			g.key_w = g.key_a = g.key_s = g.key_d = false;
		}
		return true;
	}

	bool ReadGenV5(std::ifstream& in, GenParams& g, uint32_t version) {
		uint8_t kw = 0, ka = 0, ks = 0, kd = 0, autok = 1, yabs = 0, duck = 0, noW = 1;
		if (!R(in, g.ticks) ||
			!R(in, kw) || !R(in, ka) || !R(in, ks) || !R(in, kd) || !R(in, autok) ||
			!R(in, g.yaw_mode) || !R(in, g.yaw_rate) || !R(in, yabs) || !R(in, g.yaw_start) ||
			!R(in, g.pitch_mode) || !R(in, g.pitch_val) || !R(in, g.pitch_mult) ||
			!R(in, duck) || !R(in, g.jump_mode))
			return false;
		if (version >= 6 && !R(in, noW))
			return false;
		if (version >= 10 && !R(in, g.opt_dir))
			return false;
		if (version >= 11 && !R(in, g.opt_period))
			return false;
		g.key_w = kw != 0; g.key_a = ka != 0; g.key_s = ks != 0; g.key_d = kd != 0;
		g.auto_key = autok != 0;
		g.no_w_air = noW != 0;
		g.yaw_abs = yabs != 0;
		g.duck = duck != 0;
		return true;
	}

	bool LoadProject(const std::string& filename) {
		const std::string dir = ProjectDir();
		if (dir.empty())
			return false;
		std::ifstream in(dir + "\\" + filename, std::ios::binary);
		if (!in) { g_status = "Couldn't open " + filename + "."; return false; }

		char magic[4];
		uint32_t version = 0;
		if (!in.read(magic, 4) || memcmp(magic, kProjMagic, 4) != 0 || !R(in, version)
			|| version < 1 || version > kProjVersion) {
			g_status = "Not a valid project file.";
			return false;
		}

		StartState anchor;
		uint8_t av = 0, ad = 0;
		float cap = 30.f, accel = 10.f, wish = 250.f, min_eff = 0.98f;
		if (!R(in, av) ||
			!in.read(reinterpret_cast<char*>(&anchor.origin), sizeof(float) * 3) ||
			!in.read(reinterpret_cast<char*>(&anchor.velocity), sizeof(float) * 3) ||
			!R(in, anchor.pitch) || !R(in, anchor.yaw) || !R(in, ad) || !R(in, anchor.stamina) ||
			!R(in, cap) || !R(in, accel) || !R(in, wish)) {
			g_status = "Truncated project file.";
			return false;
		}
		if (version >= 2 && !R(in, min_eff)) {
			g_status = "Truncated project file.";
			return false;
		}
		anchor.valid = av != 0;
		anchor.ducked = ad != 0;

		uint32_t nsegs = 0;
		if (!R(in, nsegs) || nsegs > 4096) { g_status = "Truncated project file."; return false; }

		std::vector<EditSegment> segs;
		for (uint32_t k = 0; k < nsegs; ++k) {
			uint8_t type = 0;
			if (!R(in, type)) return false;
			EditSegment s;
			s.raw = (type == 1);
			s.is_solver = (type == 2);
			if (s.raw) {
				uint32_t n = 0;
				if (!R(in, n) || n > static_cast<uint32_t>(Prediction::kMaxSimTicks)) return false;
				s.frames.resize(n);
				for (uint32_t f = 0; f < n; ++f)
					if (!ReadFrame(in, s.frames[f])) return false;
			} else {
				if (version >= 5) {
					if (!ReadGenV5(in, s.gen, version)) return false;
				} else if (version >= 2) {
					if (!ReadGenLegacy(in, s.gen, version)) return false;
				} else {
					if (!ReadGenV1(in, s.gen)) return false;
				}
				// "Hold" retired: identical to turning at 0 deg/tick.
				if (s.gen.yaw_mode == 0) {
					s.gen.yaw_mode = 1;
					s.gen.yaw_rate = 0.f;
				}
				SanitizeGen(s.gen);
				if (version >= 4) {
					uint32_t novr = 0;
					if (!R(in, novr) || novr > 4096) return false;
					for (uint32_t o = 0; o < novr; ++o) {
						int32_t local = 0;
						uint8_t val = 0;
						if (!R(in, local) || !R(in, val)) return false;
						s.jump_ovr.push_back({ local, val ? 1 : 0 });
					}
				}
				if (s.is_solver) {
					SolverData& sd = s.solver;
					uint8_t solved = 0;
					if (!R(in, sd.target) || !R(in, sd.strafes) || !R(in, solved) ||
						!R(in, sd.ticks) || !R(in, sd.side0) || !R(in, sd.nsplits))
						return false;
					for (int i = 0; i < 3; ++i)
						if (!R(in, sd.splits[i])) return false;
					for (int i = 0; i < 4; ++i)
						if (!R(in, sd.rates[i])) return false;
					sd.solved = solved != 0;
					if (version >= 9) {
						uint8_t sjump = 0;
						float px = 0.f, py = 0.f, pz = 0.f;
						if (!R(in, sjump) || !R(in, sd.pass_frac)
							|| !R(in, px) || !R(in, py) || !R(in, pz) || !R(in, sd.model_err))
							return false;
						sd.start_jump = sjump != 0;
						sd.model_pass = Vector(px, py, pz);
						if (version >= 10) {
							uint8_t sopt = 0;
							if (!R(in, sopt))
								return false;
							sd.opt = sopt != 0;
						}
						if (version >= 11) {
							if (!R(in, sd.blend) || !R(in, sd.tyaw))
								return false;
						}
					} else {
						// v7/v8 first-generation solver blocks: consume their extra
						// fields; the raw schedule itself still replays fine.
						if (version >= 7) {
							uint8_t couple = 1, sjump = 0;
							if (!R(in, couple) || !R(in, sjump)) return false;
							sd.start_jump = sjump != 0;
						}
						if (version >= 8) {
							float dummy = 0.f;
							for (int i = 0; i < 4; ++i)
								if (!R(in, dummy)) return false;
						}
						sd.pass_frac = 1.f;
					}
					SanitizeSolver(sd);
				}
			}
			segs.push_back(std::move(s));
		}

		g_anchor = anchor;
		g_air_cap = cap;
		g_air_accel = accel;
		g_wishspeed = wish;
		g_min_eff = min_eff;
		g_segs = std::move(segs);
		g_sel = g_segs.empty() ? -1 : 0;
		g_cursor = 0;
		g_valid = false;
		g_frames.clear();
		g_states.clear();

		std::string stem = filename;
		const size_t dot = stem.find_last_of('.');
		if (dot != std::string::npos)
			stem = stem.substr(0, dot);
		strncpy_s(g_name, stem.c_str(), _TRUNCATE);

		g_status = "Loaded " + filename + ".";
		MarkDirty();
		return true;
	}

	void DeleteProjectFile() {
		if (g_sel_file < 0 || g_sel_file >= static_cast<int>(g_files.size()))
			return;
		const std::string dir = ProjectDir();
		if (dir.empty())
			return;
		const std::string name = g_files[g_sel_file];
		if (DeleteFileA((dir + "\\" + name).c_str()))
			g_status = "Deleted " + name + ".";
		else
			g_status = "Couldn't delete " + name + ".";
		RefreshFiles();
	}

	// ------------------------------------------------------- library bridge --
	bool BuildCompiledRun(Run& run) {
		if (!g_valid || g_dirty || g_frames.empty())
			return false;
		run.name = g_name;
		run.start = g_anchor;
		run.segments.clear();
		for (int k = 0; k < static_cast<int>(g_starts.size()); ++k) {
			const int s = g_starts[k];
			int e = (k + 1 < static_cast<int>(g_starts.size())) ? g_starts[k + 1] : g_total;
			if (e > static_cast<int>(g_frames.size()))
				e = static_cast<int>(g_frames.size());
			if (e <= s)
				continue;
			run.segments.emplace_back(g_frames.begin() + s, g_frames.begin() + e);
		}
		return !run.segments.empty();
	}

	void TestPlay() {
		if (!g_anchor.valid) { g_status = "No anchor - capture one first."; return; }
		Run run;
		if (!BuildCompiledRun(run)) {
			g_status = "Sim not current - wait for the resim, then test.";
			return;
		}
		char cmd[192];
		// setpos_exact, full precision (see the Teleport button note).
		Sfmt(cmd, "setpos_exact %.6f %.6f %.6f; setang %.2f %.2f 0",
			g_anchor.origin.X, g_anchor.origin.Y, g_anchor.origin.Z,
			g_anchor.pitch, g_anchor.yaw);
		if (engine)
			engine->ClientCmd_Unrestricted(cmd);
		if (g_tas.PlayEphemeral(std::move(run), 12))
			g_status = "Test playback from anchor (teleport needs sv_cheats 1). Emergency Stop aborts.";
		else
			g_status = "Can't test now - recorder busy.";
	}

	void ExportToLibrary() {
		Run run;
		if (!BuildCompiledRun(run)) {
			g_status = "Sim not current - wait for the resim before exporting.";
			return;
		}
		if (g_tas.AddRun(run) >= 0) {
			g_tas.PersistSelected();
			g_status = "Exported to library as \"" + std::string(g_name) + "\".";
		} else {
			g_status = "Export failed (run mode only).";
		}
	}

	void ImportFromLibrary() {
		const std::vector<Run>& lib = g_tas.Library();
		const int sel = g_tas.Selected();
		if (sel < 0 || sel >= static_cast<int>(lib.size())) {
			g_status = "Select a run in the recordings list first.";
			return;
		}
		const Run& r = lib[sel];
		for (const Segment& s : r.segments) {
			EditSegment es;
			es.raw = true;
			es.frames = s;
			g_segs.push_back(std::move(es));
		}
		if (r.start.valid && !g_anchor.valid)
			g_anchor = r.start;
		g_sel = static_cast<int>(g_segs.size()) - 1;
		g_status = "Imported \"" + r.name + "\" (" + std::to_string(r.segments.size()) + " raw segment(s))"
			+ (r.start.valid ? "." : " - no anchor in file (v1), capture one.");
		MarkDirty();
	}

	// ----------------------------------------------------------------- tabs --
	void DrawRunTab() {
		// --- anchor + test play ------------------------------------------
		if (g_anchor.valid)
			ImGui::Text("Anchor: %.1f %.1f %.1f   yaw %.1f   vel %.0f u/s%s",
				g_anchor.origin.X, g_anchor.origin.Y, g_anchor.origin.Z,
				g_anchor.yaw, Speed2D(g_anchor.velocity), g_anchor.ducked ? "   (ducked)" : "");
		else
			ImGui::TextColored(ImVec4(1.f, 0.75f, 0.3f, 1.f), "No anchor - capture one to simulate.");

		if (ImGui::Button("Capture anchor from player")) {
			StartState s;
			if (Prediction::CaptureStartState(s)) {
				g_anchor = s;
				g_status = "Anchor captured. (Capture at a standstill for replayable runs.)";
				MarkDirty();
			} else {
				g_status = "Not in game - can't capture.";
			}
		}
		ImGui::SameLine();
		if (ImGui::Button("Teleport player to anchor") && g_anchor.valid && engine) {
			char cmd[192];
			// setpos_exact, full precision, no nudges: plain setpos ran the
			// engine's unstick (1u forward); a +0.25z guess flipped it to 1u
			// BEHIND. No more guessing - the deferred check below MEASURES the
			// landing delta and prints it, so the next report is data.
			Sfmt(cmd, "setpos_exact %.6f %.6f %.6f; setang %.2f %.2f 0",
				g_anchor.origin.X, g_anchor.origin.Y, g_anchor.origin.Z,
				g_anchor.pitch, g_anchor.yaw);
			engine->ClientCmd_Unrestricted(cmd);
			g_tp_check = true;
			g_tp_ms = GetTickCount64();
			g_status = "Teleported (needs sv_cheats 1); measuring the landing...";
		}
		ImGui::SameLine();
		if (ImGui::Button("Test play from anchor"))
			TestPlay();
		ImGui::SameLine();
		if (ImGui::Button("Stop test") && g_tas.IsPlaying())
			g_tas.EmergencyStop();

		ImGui::Separator();

		// --- segment list -------------------------------------------------
		ImGui::Text("Segments   (%d ticks total, %.2f s)", g_total,
			g_total * (Prediction::LastDiag().interval_per_tick > 0.f ? Prediction::LastDiag().interval_per_tick : 0.015f));
		ImGui::BeginChild("segs", ImVec2(0, 100), true);
		for (int k = 0; k < static_cast<int>(g_segs.size()); ++k) {
			const EditSegment& s = g_segs[k];
			char kind[64];
			if (s.raw) {
				strcpy_s(kind, "RAW");
			} else if (s.is_solver) {
				Sfmt(kind, "SOLVER ->#%d K%d%s", s.solver.target, s.solver.strafes,
					s.solver.solved ? "" : " (unsolved)");
			} else {
				const bool turning = s.gen.yaw_mode == 1 && fabsf(s.gen.yaw_rate) > 0.01f;
				Sfmt(kind, "MOVE %s%s%s%s%s%s",
					s.gen.key_w ? "W" : "", s.gen.key_a ? "A" : "",
					s.gen.key_s ? "S" : "", s.gen.key_d ? "D" : "",
					(turning && s.gen.auto_key && !s.gen.key_a && !s.gen.key_d) ? "+auto" : "",
					turning ? " turn" : "");
			}
			char label[256];
			Sfmt(label, "#%d  %s  %d ticks%s##seg%d", k, kind, s.Ticks(),
				(!s.raw && !s.jump_ovr.empty()) ? "  [jump ovr]" : "", k);
			if (ImGui::Selectable(label, g_sel == k)) {
				g_sel = k;
				if (k < static_cast<int>(g_starts.size()))
					g_cursor = g_starts[k];
			}
		}
		ImGui::EndChild();

		if (ImGui::Button("+ Segment")) {
			EditSegment s;
			// A segment added right after a SOLVED solver segment starts
			// mid-board. Bare defaults (no keys, frozen yaw) let go of the
			// ramp there, which visibly changed the ride vs the solver's own
			// overrun (it holds the final phase's key + turn) - the "adding a
			// segment changes the outcome" report. Default the new segment to
			// CONTINUE the ride: optimal strafe in the final phase's
			// direction, zero bias - all visible and editable in its UI.
			if (g_sel >= 0 && g_sel < static_cast<int>(g_segs.size())
				&& g_segs[g_sel].is_solver && g_segs[g_sel].solver.solved) {
				const SolverData& psd = g_segs[g_sel].solver;
				const int kk = psd.nsplits;
				const int last_side = (kk % 2 == 0) ? psd.side0 : -psd.side0;
				s.gen.key_w = false;
				s.gen.yaw_mode = 2;                    // optimal strafe (ride)
				s.gen.opt_dir = last_side;
				s.gen.yaw_rate = 0.f;                  // pure max-gain line
			}
			InsertSegment(s);
		}
		ImGui::SameLine();
		if (ImGui::Button("+ Solver segment")) {
			EditSegment s;
			s.is_solver = true;
			s.gen.jump_mode = JM_None;
			InsertSegment(s);
		}
		ImGui::SameLine();
		if (ImGui::Button("Split at playhead"))
			SplitAtCursor();
		ImGui::SameLine();
		if (ImGui::Button("Remove selected") && g_sel >= 0 && g_sel < static_cast<int>(g_segs.size())) {
			g_segs.erase(g_segs.begin() + g_sel);
			if (g_sel >= static_cast<int>(g_segs.size()))
				g_sel = static_cast<int>(g_segs.size()) - 1;
			MarkDirty();
		}

		// --- selected segment parameters ----------------------------------
		if (g_sel >= 0 && g_sel < static_cast<int>(g_segs.size())) {
			ImGui::Separator();
			if (g_segs[g_sel].is_solver && !g_segs[g_sel].raw) {
				EditSegment& s = g_segs[g_sel];
				ImGui::TextWrapped(
					"Solver segment (rebuilt): searches alternating A/D strafe schedules "
					"whose path passes through the EXACT target point with the view yaw "
					"at that moment EXACTLY the target's set yaw. The pass tick is "
					"computed from gravity (never searched), the yaw equation is solved "
					"algebraically (the real sim replays the same view stream, so yaw "
					"cannot drift), and the remaining rates solve the 2D pass point. "
					"The drawn line is the REAL engine sim; its measured pass is shown "
					"below and auto-corrected against the target.");

				ImGui::PushItemWidth(260);
				int target = s.solver.target;
				if (ImGui::Combo("Board target", &target, TargetItemGetter, nullptr, BspWorld::TargetCount()))
					s.solver.target = target;
				int strafes = s.solver.strafes;
				if (ImGui::InputInt("Strafes (alternating)", &strafes))
					s.solver.strafes = strafes < 1 ? 1 : strafes > 4 ? 4 : strafes;
				ImGui::PopItemWidth();
				bool sopt = s.solver.opt;
				if (ImGui::Checkbox("Optimal-rate strafes (max gain; search per-phase biases)", &sopt)) {
					s.solver.opt = sopt;
					MarkDirty();   // an applied schedule's numbers change meaning - re-search
				}
				if (s.solver.opt) {
					ImGui::TextDisabled("each phase rides the max-gain line +- a solved bias; the view "
						"BLENDS onto the target yaw over the last ticks (exact at the pass)");
					if (IntRow("Yaw blend window (ticks before the pass)", &s.solver.blend, 1, 24))
						MarkDirty();   // the applied schedule's replay changes with it
				}
				else if (s.solver.strafes < 3)
					ImGui::TextDisabled("(with the yaw constraint, %d strafe(s) only hit the point when "
						"a timing happens to line up - 3+ solves it structurally)", s.solver.strafes);

				ImGui::Checkbox("Jump at start (grounded start)", &s.solver.start_jump);
				ImGui::SameLine();
				ImGui::TextDisabled("(the model is airborne-only - a grounded start needs this)");

				ImGui::PushItemWidth(150);
				ImGui::InputFloat("Pass tolerance (u)", &g_sol_pos_tol, 0.25f, 1.f, 2);
				ImGui::InputFloat("Max bump loss (u/s)", &g_sol_max_bump, 1.f, 5.f, 1);
				ImGui::InputFloat("Collect within (u)", &g_sol_collect, 0.5f, 2.f, 1);
				ImGui::PopItemWidth();
				ImGui::TextDisabled("(everything passing within 'Collect' is kept and real-measured - "
					"the tight gates only decide the 'exact' tag; data first, filtering later)");
				ImGui::TextDisabled("(a kiss of the ramp shortly before the target is allowed and slides "
					"into it - as long as the clip costs less than this)");
				ImGui::PushItemWidth(150);
				ImGui::InputFloat("Jump vz (0 = formula)", &g_jump_vz, 1.f, 10.f, 1);
				ImGui::PopItemWidth();
				// The engine probe: measured facts from the real sim's states.
				// NEVER silently used - Adopt copies them into the inputs above.
				if (g_meas_max_add > 0.f || g_meas_grav_n > 0) {
					float interval = Prediction::LastDiag().interval_per_tick;
					if (interval <= 0.f) interval = 0.015f;
					const float model_amt = g_air_accel * g_wishspeed * interval;
					ImGui::TextDisabled("engine probe: add/tick up to %.1f (model allows %.1f) | gravity %.0f [%d] | jump vz %.1f",
						g_meas_max_add, model_amt, g_meas_grav, g_meas_grav_n, g_meas_jump_vz);
					if (g_meas_max_add > model_amt + 2.f)
						ImGui::TextColored(ImVec4(1.f, 0.6f, 0.2f, 1.f),
							"the engine granted MORE accel than the model allows -> the "
							"sv_airaccelerate set here is too low for this server (surf servers run ~150)");
					if (ImGui::Button("Adopt probe into model settings")) {
						if (g_meas_max_add > 45.f)
							g_air_accel = 150.f;
						if (g_meas_grav_n >= 8)
							g_gravity = g_meas_grav;
						if (g_meas_jump_vz > 100.f)
							g_jump_vz = g_meas_jump_vz;
						MarkDirty();
						g_status = "Adopted the probed engine values into the model settings (visible above/in the strafe model).";
					}
				} else {
					ImGui::TextDisabled("engine probe: no data yet - it fills in after any sim runs "
						"with a solved solver segment");
				}

				if (ImGui::Button("Search solutions"))
					StartSearch(g_sel);

				if (g_search.seg == g_sel) {
					if (g_search.active) {
						ImGui::Text("searching...  %d%%   (%d passes collected - near-identical lines merge at the end)",
							g_search.jobs_total ? (g_search.jobs_done * 100 / g_search.jobs_total) : 0,
							static_cast<int>(g_search.results.size()));
					} else if (g_search.done && g_search.results.empty()) {
						ImGui::PushTextWrapPos(0.f);
						if (g_search.cands.empty()) {
							ImGui::TextColored(ImVec4(1.f, 0.45f, 0.45f, 1.f),
								"NO solutions: the ballistic arc never reaches the target's HEIGHT "
								"from this start. If the start is on the ground, enable 'Jump at "
								"start'; otherwise the target is too high for this approach.");
						} else if (!g_search.yaw_ever_feasible) {
							ImGui::TextColored(ImVec4(1.f, 0.45f, 0.45f, 1.f),
								"NO solutions: the required view-yaw change can't be produced by any "
								"%d-strafe timing within +-8 deg/tick over this flight. More "
								"strafes, a different target yaw, or a longer flight.",
								g_search.strafes);
						} else if (g_search.reject_best < 1e8f) {
							ImGui::TextColored(ImVec4(1.f, 0.45f, 0.45f, 1.f),
								"NO exact pass with %d strafe(s): the closest exact-yaw pass came "
								"%.1f u from the target (need <= %.2f u). %s",
								g_search.strafes, g_search.reject_best, g_sol_pos_tol,
								g_search.strafes < 3
									? "With the yaw constraint this strafe count only hits when a timing lines up - use 3+."
									: "The required view turn may fight the path curvature - try a different strafe count or target yaw.");
							if (g_search.reject_retime > 0)
								ImGui::TextDisabled("(%d more reached the target but couldn't align "
									"the contact tick with the yaw schedule)", g_search.reject_retime);
						} else if (g_search.reject_retime > 0) {
							ImGui::TextColored(ImVec4(1.f, 0.45f, 0.45f, 1.f),
								"NO exact passes kept: %d candidate(s) reached the target but the "
								"contact tick couldn't be aligned with the yaw schedule. Try one "
								"more/fewer strafe, or nudge the target yaw slightly.",
								g_search.reject_retime);
						} else {
							ImGui::TextColored(ImVec4(1.f, 0.45f, 0.45f, 1.f),
								"NO solutions: no timing/wrap combination was even solvable for %d "
								"strafe(s).", g_search.strafes);
						}
						if (g_search.early_hits > 0)
							ImGui::TextDisabled("(%d flights were deflected by corridor geometry "
								"before the pass - part of the approach corridor is blocked)",
								g_search.early_hits);
						ImGui::TextDisabled("candidate outcomes: %d infeasible | %d no-eval | "
							"%d beyond-collect | %d tick-align | %d violent-board | %d target-brush deflects",
							g_search.cnt_infeasible, g_search.cnt_no_contact,
							g_search.cnt_floor, g_search.reject_retime, g_search.cnt_bump,
							g_search.cnt_edge);
						if (g_searchlog_path[0])
							ImGui::TextWrapped("search diagnostics: %s", g_searchlog_path);
						ImGui::PopTextWrapPos();
					} else if (g_search.done && g_verify.active) {
						ImGui::Text("verifying candidates on the REAL sim:  %d / %d ...",
							g_verify.idx, static_cast<int>(g_search.results.size()));
					} else if (g_search.done) {
						const int count = static_cast<int>(g_search.results.size());
						// Tiered sort controls: equal values (quantized) fall
						// through to the next tier; clean lines always outrank
						// deflected ones.
						{
							bool resort = false;
							ImGui::Text("Sort:");
							ImGui::PushItemWidth(150);
							for (int i = 0; i < 3; ++i) {
								ImGui::SameLine();
								char lbl[16];
								Sfmt(lbl, "##sort%d", i);
								resort |= ImGui::Combo(lbl, &g_sort_key[i], kSortKeyItems);
							}
							ImGui::PopItemWidth();
							ImGui::SameLine();
							ImGui::TextDisabled("(ties: 0.5 u/s, 0.05 u, 0.05 deg)");
							if (resort && count > 0) {
								SortVerifiedResults();
								g_solution_pick = 0;
								ApplySolution(g_sel, g_search.results[0]);
								g_correct.active = false;
								g_pick_correct_pending = g_correct.auto_run;
								g_pick_settle_ms = GetTickCount64();
							}
						}
						int pick = g_solution_pick;
						if (IntRow("Solution (0 = best by sort)", &pick, 0, count - 1) && pick != g_solution_pick) {
							g_solution_pick = pick;
							ApplySolution(g_sel, g_search.results[pick]);
							// Correction starts only after the slider SETTLES -
							// scrubbing quickly fired a correction (with its
							// collider rebuilds and re-solves) per notch on top
							// of the resim churn.
							g_correct.active = false;
							g_pick_correct_pending = g_correct.auto_run;
							g_pick_settle_ms = GetTickCount64();
						}
						if (g_solution_pick >= count)
							g_solution_pick = 0;
						const RouteSolution& sol = g_search.results[g_solution_pick];
						ImGui::Text("model: pass %.3f u from target inside tick %d (frac %.2f)   speed %.0f u/s   wrap %+d%s",
							sol.pass_err, sol.ticks, sol.frac, sol.speed, sol.wrap,
							sol.exact ? "" : "   [near]");
						if (sol.bump_ticks > 0)
							ImGui::TextColored(ImVec4(0.7f, 0.9f, 1.f, 1.f),
								"   kisses the ramp %d tick(s) before the pass, clipping %.2f u/s, then slides in",
								sol.bump_ticks, sol.bump_loss);
						else if (sol.bump_loss > 0.01f)
							ImGui::TextDisabled("   boards at the pass, clip %.2f u/s", sol.bump_loss);
						if (sol.real_err >= 0.f && sol.real_err < 1e8f)
							ImGui::TextColored(sol.real_err <= g_sol_pos_tol
									? ImVec4(0.4f, 1.f, 0.55f, 1.f) : ImVec4(1.f, 0.85f, 0.4f, 1.f),
								"REAL-verified: passes %.2f u from the target (tick %d)%s",
								sol.real_err, sol.real_tick,
								sol.real_bump_loss > 0.01f ? "   (kissed the ramp on the way)" : "");

						char breakdown[256] = {};
						int prev = 0;
						for (int k = 0; k <= sol.nsplits; ++k) {
							const int end = (k < sol.nsplits) ? sol.splits[k] : sol.ticks;
							const int side = (k % 2 == 0) ? sol.side0 : -sol.side0;
							char part[96];
							Sfmt(part, "%s%+.3f deg/t x %dt (%s)", k ? "  |  " : "",
								sol.rates[k], end - prev, side > 0 ? "A" : "D");
							strcat_s(breakdown, part);
							prev = end;
						}
						ImGui::TextWrapped("%s", breakdown);

						// Staleness: the search snapshot vs the segment's current start.
						Vector now_pos, now_vel;
						float now_yaw;
						if (SolverStartState(g_sel, now_pos, now_vel, now_yaw)) {
							const Vector dp = now_pos - g_search.start_pos;
							if (sqrtf(Dot3(dp)) > 2.f)
								ImGui::TextColored(ImVec4(1.f, 0.7f, 0.3f, 1.f),
									"start state changed since the search - Re-search.");
						}
					}
				}

				// TEMPORARY calibration harness: manual button - runs EVERY found
				// solution through the real sim (with correction rounds) and
				// writes the whole story to a log file for offline tuning.
				if (g_batch.active) {
					ImGui::TextColored(ImVec4(0.6f, 0.85f, 1.f, 1.f),
						"CALIBRATING: solution %d/%d  round %d ... (log writes when done)",
						g_batch.idx + 1, static_cast<int>(g_search.results.size()), g_batch.round);
					ImGui::SameLine();
					if (ImGui::SmallButton("abort##calib"))
						FinishBatch(true);
				} else {
					if (g_search.seg == g_sel && g_search.done && !g_search.results.empty()
						&& ImGui::Button("CALIBRATE: run ALL solutions on the real sim -> log file"))
						StartBatch();
					if (g_search.seg == g_sel && g_search.done) {
						if (!g_search.results.empty())
							ImGui::SameLine();
						if (ImGui::Button("Write search diagnostics log")) {
							WriteSearchLog();
							g_status = std::string("Search diagnostics written: ")
								+ (g_searchlog_path[0] ? g_searchlog_path : "(write failed)");
						}
					}
					if (g_batch.path[0])
						ImGui::TextWrapped("calibration log: %s", g_batch.path);
				}

				// --- ground truth: what the DRAWN line actually does -----------
				if (s.solver.solved) {
					ImGui::Separator();
					ImGui::Checkbox("Auto-correct against the REAL sim", &g_correct.auto_run);
					ImGui::SameLine();
					if (ImGui::Button("Re-check / correct now"))
						StartCorrect(g_sel);
					if (g_correct.active)
						ImGui::Text("measuring/correcting on the real sim...  round %d", g_correct.rounds);
					else if (g_correct.done && g_correct.note[0])
						ImGui::TextWrapped("%s", g_correct.note);
					if (g_realpass.computed) {
						if (g_realpass.crossed && !g_realpass.on_target) {
							ImGui::TextColored(ImVec4(1.f, 0.45f, 0.45f, 1.f),
								"REAL line: EARLY BOARD onto another tagged face (brush %d plane %d) "
								"at tick %d, %.0f u from the target - the flight never reaches the "
								"target's face. Re-search: the model now rejects such paths.",
								g_realpass.hit_brush, g_realpass.hit_plane,
								g_realpass.tick, g_realpass.err);
						} else if (g_realpass.crossed) {
							char drift[48] = {};
							if (g_realpass.drift >= 0.f)
								Sfmt(drift, "   drift vs model %.1f u", g_realpass.drift);
							ImGui::TextColored(g_realpass.err <= g_sol_pos_tol
									? ImVec4(0.4f, 1.f, 0.55f, 1.f) : ImVec4(1.f, 0.7f, 0.3f, 1.f),
								"REAL line: passes %.2f u from the target (tick %d)   yaw at pass %+.3f deg off%s",
								g_realpass.err, g_realpass.tick, g_realpass.yaw_err, drift);
							if (g_realpass.bump_tick >= 0)
								ImGui::TextDisabled("   kissed the ramp %d tick(s) before the pass (clip ~%.1f u/s), slid in",
									g_realpass.bump_ticks, g_realpass.bump_loss);
							ImGui::Text("   closest 3D approach of the whole line: %.2f u", g_realpass.min_dist);
						} else {
							ImGui::TextColored(ImVec4(1.f, 0.45f, 0.45f, 1.f),
								"REAL line: never reaches the pass event (closest to plane %.1f u / "
								"height %.1f u, closest 3D %.1f u).%s",
								g_realpass.closest_phi < 1e8f ? g_realpass.closest_phi : 0.f,
								g_realpass.closest_dz < 1e8f ? g_realpass.closest_dz : 0.f,
								g_realpass.min_dist < 1e8f ? g_realpass.min_dist : 0.f,
								g_realpass.jump_missing
									? " The start jump never fired - the sim start wasn't on the ground."
									: "");
						}
						if (g_realpass.fit_samples >= 4)
							ImGui::TextDisabled(
								"engine fit: gravity ~%.0f (model %.0f)   add/tick <= %.1f (model amt %.1f)   "
								"cap ~%.1f (model %.1f)   [%d air ticks]",
								g_realpass.grav_fit, g_gravity, g_realpass.max_add, g_search.accel_amt,
								g_realpass.cap_fit, g_air_cap, g_realpass.fit_samples);
					} else {
						ImGui::TextDisabled("REAL line: no measurement yet - it appears automatically after the "
							"line recomputes (this solver segment must be the chain's last).");
					}
				}

				// Pitch still applies (static or trajectory-follow, as usual).
				bool ch = false;
				ImGui::PushItemWidth(180);
				ch |= ImGui::Combo("Pitch", &s.gen.pitch_mode, "Constant\0Follow trajectory\0");
				ImGui::PopItemWidth();
				if (s.gen.pitch_mode == PM_Const)
					ch |= FloatRow("Pitch value", &s.gen.pitch_val, -89.f, 89.f, "%.1f deg", 0.1f, 5.f, 1);
				else
					ch |= FloatRow("Pitch multiplier", &s.gen.pitch_mult, -2.f, 2.f, "%.2f", 0.05f, 0.2f);
				if (ch)
					MarkDirty();

				if (s.solver.solved && ImGui::Button("Bake to raw frames") && g_valid && !g_dirty
					&& g_sel < static_cast<int>(g_starts.size())) {
					const int b = g_starts[g_sel];
					int e = (g_sel + 1 < static_cast<int>(g_starts.size())) ? g_starts[g_sel + 1] : g_total;
					if (e > static_cast<int>(g_frames.size()))
						e = static_cast<int>(g_frames.size());
					if (e > b) {
						s.raw = true;
						s.is_solver = false;
						s.frames.assign(g_frames.begin() + b, g_frames.begin() + e);
						s.jump_ovr.clear();
						g_status = "Baked solver segment to raw frames.";
						MarkDirty();
					}
				}
			} else if (!g_segs[g_sel].raw) {
				const GenParams before = g_segs[g_sel].gen;
				GenParams g = before;
				bool ch = false;

				ImGui::Text("Held keys:");
				ImGui::SameLine(); ch |= ImGui::Checkbox("W", &g.key_w);
				ImGui::SameLine(); ch |= ImGui::Checkbox("A", &g.key_a);
				ImGui::SameLine(); ch |= ImGui::Checkbox("S", &g.key_s);
				ImGui::SameLine(); ch |= ImGui::Checkbox("D", &g.key_d);
				ImGui::SameLine();
				ch |= ImGui::Checkbox("Auto A/D from turn direction", &g.auto_key);
				ImGui::SameLine();
				ch |= ImGui::Checkbox("Release W in air", &g.no_w_air);
				if (g.auto_key && !g.key_a && !g.key_d)
					ImGui::TextDisabled("airborne + turning: the strafe key is picked for you (check A or D to override)");

				ch |= IntRow("Ticks (duration)", &g.ticks, 1, 1000);
				// "Hold" retired: it's identical to turning at 0 deg/tick (old
				// files are normalized at load). Mode 2 = OPTIMAL strafe (yaw
				// rides the velocity heading + bias). Mode 3 = STRAIGHT SYNC
				// (alternating optimal strafes; the net path holds heading).
				if (g.yaw_mode < 1 || g.yaw_mode > 3)
					g.yaw_mode = 1;
				int vmode = (g.yaw_mode == 2) ? 1 : (g.yaw_mode == 3) ? 2 : 0;
				ImGui::PushItemWidth(260);
				if (ImGui::Combo("View mode", &vmode,
					"Turn rate\0Optimal strafe (max gain)\0Straight sync strafe\0")) {
					g.yaw_mode = (vmode == 1) ? 2 : (vmode == 2) ? 3 : 1;
					ch = true;
				}
				ImGui::PopItemWidth();
				if (g.yaw_mode == 2) {
					int dir_idx = (g.opt_dir > 0) ? 0 : 1;
					ImGui::PushItemWidth(140);
					if (ImGui::Combo("Direction", &dir_idx, "Left (A)\0Right (D)\0")) {
						g.opt_dir = (dir_idx == 0) ? 1 : -1;
						ch = true;
					}
					ImGui::PopItemWidth();
					ch |= FloatRow("Turn bias   (+ tighter / - wider, 0 = pure optimal)",
						&g.yaw_rate, -20.f, 20.f, "%+.2f deg", 0.05f, 0.5f);
					ImGui::TextDisabled("bias trades speed (quadratic loss) for curvature (linear) - stays optimal for that curvature");
				} else if (g.yaw_mode == 3) {
					int dir_idx = (g.opt_dir > 0) ? 0 : 1;
					ImGui::PushItemWidth(140);
					if (ImGui::Combo("First side", &dir_idx, "Left (A)\0Right (D)\0")) {
						g.opt_dir = (dir_idx == 0) ? 1 : -1;
						ch = true;
					}
					ImGui::PopItemWidth();
					ch |= IntRow("Ticks per strafe side (frequency)", &g.opt_period, 1, 64);
					ch |= FloatRow("Heading steer bias   (0 = straight)",
						&g.yaw_rate, -20.f, 20.f, "%+.2f deg", 0.05f, 0.5f);
					ImGui::TextDisabled("alternating max-gain strafes; half-curls cancel so the path holds its "
						"heading - the bias strengthens one side and curves the mean");
				} else {
					ch |= FloatRow("View turn   (- left / + right, 0 = hold)", &g.yaw_rate, -15.f, 15.f, "%+.2f deg/tick", 0.05f, 0.5f);
				}
				ch |= ImGui::Checkbox("Absolute start yaw", &g.yaw_abs);
				if (g.yaw_abs) {
					ImGui::SameLine();
					ImGui::PushItemWidth(kInputW);
					ch |= ImGui::InputFloat("##yawstart", &g.yaw_start, 0.1f, 5.f, 1);
					ImGui::PopItemWidth();
				}

				ImGui::PushItemWidth(180);
				ch |= ImGui::Combo("Pitch", &g.pitch_mode, "Constant\0Follow trajectory\0");
				ImGui::PopItemWidth();
				if (g.pitch_mode == PM_Const)
					ch |= FloatRow("Pitch value", &g.pitch_val, -89.f, 89.f, "%.1f deg", 0.1f, 5.f, 1);
				else
					ch |= FloatRow("Pitch multiplier", &g.pitch_mult, -2.f, 2.f, "%.2f", 0.05f, 0.2f);
				ch |= ImGui::Checkbox("Duck", &g.duck);
				ImGui::SameLine();
				ImGui::PushItemWidth(220);
				ch |= ImGui::Combo("Jump", &g.jump_mode, "None (walk)\0Hold\0Auto-bhop on landing\0");
				ImGui::PopItemWidth();

				if (ch) {
					const int start = (g_sel < static_cast<int>(g_starts.size())) ? g_starts[g_sel] : 0;
					const int local = g_cursor - start;
					if (local > 0 && local < g_segs[g_sel].Ticks() && SegmentAtTick(g_cursor) == g_sel)
						AutoSplitApply(g_sel, local, before, g);
					else
						g_segs[g_sel].gen = g;
					MarkDirty();
				}

				if (ImGui::Button("Bake to raw frames") && g_valid && !g_dirty
					&& g_sel < static_cast<int>(g_starts.size())) {
					const int b = g_starts[g_sel];
					int e = (g_sel + 1 < static_cast<int>(g_starts.size())) ? g_starts[g_sel + 1] : g_total;
					if (e > static_cast<int>(g_frames.size()))
						e = static_cast<int>(g_frames.size());
					if (e > b) {
						EditSegment& s = g_segs[g_sel];
						s.raw = true;
						s.frames.assign(g_frames.begin() + b, g_frames.begin() + e);
						s.jump_ovr.clear();   // baked frames already contain them
						g_status = "Baked segment to raw frames (per-tick editable).";
						MarkDirty();
					}
				}
			} else {
				EditSegment& s = g_segs[g_sel];
				ImGui::Text("Raw segment: %d frames. Fine-tune the playhead tick:", static_cast<int>(s.frames.size()));
				const int base = (g_sel < static_cast<int>(g_starts.size())) ? g_starts[g_sel] : 0;
				const int local = g_cursor - base;
				if (local >= 0 && local < static_cast<int>(s.frames.size())) {
					Frame& f = s.frames[local];
					bool ch = false;
					ImGui::PushItemWidth(120);
					ch |= ImGui::InputFloat("Yaw##tick", &f.viewangles[1], 0.1f, 5.f, 2);
					ImGui::SameLine();
					ch |= ImGui::InputFloat("Pitch##tick", &f.viewangles[0], 0.1f, 5.f, 2);
					ch |= ImGui::InputFloat("Fwd##tick", &f.forwardmove, 10.f, 100.f, 0);
					ImGui::SameLine();
					ch |= ImGui::InputFloat("Side##tick", &f.sidemove, 10.f, 100.f, 0);
					bool jump = (f.buttons & IN_JUMP) != 0;
					bool duck = (f.buttons & IN_DUCK) != 0;
					if (ImGui::Checkbox("Jump##tick", &jump)) { f.buttons = jump ? (f.buttons | IN_JUMP) : (f.buttons & ~IN_JUMP); ch = true; }
					ImGui::SameLine();
					if (ImGui::Checkbox("Duck##tick", &duck)) { f.buttons = duck ? (f.buttons | IN_DUCK) : (f.buttons & ~IN_DUCK); ch = true; }
					ImGui::PopItemWidth();
					if (ch)
						MarkDirty();
				} else {
					ImGui::TextDisabled("(playhead is outside this segment)");
				}
			}
		}

		ImGui::Separator();

		// --- playhead ------------------------------------------------------
		{
			IntRow("Playhead (tick)", &g_cursor, 0, g_total > 0 ? g_total - 1 : 0);

			const int seg = SegmentAtTick(g_cursor);
			if (ImGui::SmallButton("|<")) g_cursor = 0;
			ImGui::SameLine(); if (ImGui::SmallButton("<seg")) TasEditor::StepSegment(-1);
			ImGui::SameLine(); if (ImGui::SmallButton("-10")) TasEditor::StepCursor(-10);
			ImGui::SameLine(); if (ImGui::SmallButton("-1")) TasEditor::StepCursor(-1);
			ImGui::SameLine(); if (ImGui::SmallButton("+1")) TasEditor::StepCursor(1);
			ImGui::SameLine(); if (ImGui::SmallButton("+10")) TasEditor::StepCursor(10);
			ImGui::SameLine(); if (ImGui::SmallButton("seg>")) TasEditor::StepSegment(1);
			ImGui::SameLine(); if (ImGui::SmallButton(">|")) g_cursor = g_total > 0 ? g_total - 1 : 0;
			ImGui::SameLine();
			ImGui::Text("segment %d   (gray = locked history)", seg);

			// Per-tick jump override at the playhead (generated segments).
			if (seg >= 0 && seg < static_cast<int>(g_segs.size()) && !g_segs[seg].raw) {
				EditSegment& ks = g_segs[seg];
				const int local = g_cursor - g_starts[seg];
				int current = 0;   // 0 = inherit, 1 = suppress, 2 = force
				for (const std::pair<int, int>& o : ks.jump_ovr)
					if (o.first == local)
						current = (o.second == 0) ? 1 : 2;
				int pick = current;
				ImGui::PushItemWidth(220);
				ImGui::Combo("Jump this tick", &pick, "Inherit (segment rule)\0Suppress jump\0Force jump\0");
				ImGui::PopItemWidth();
				if (pick != current) {
					ks.jump_ovr.erase(
						std::remove_if(ks.jump_ovr.begin(), ks.jump_ovr.end(),
							[local](const std::pair<int, int>& o) { return o.first == local; }),
						ks.jump_ovr.end());
					if (pick == 1) ks.jump_ovr.push_back({ local, 0 });
					else if (pick == 2) ks.jump_ovr.push_back({ local, 1 });
					MarkDirty();
				}
			}

			if (g_valid && g_cursor >= 0 && g_cursor < static_cast<int>(g_states.size())) {
				const Prediction::SimState& st = g_states[g_cursor];
				const bool air = (st.flags & FL_ONGROUND) == 0;
				ImGui::Text("pos %.1f %.1f %.1f   %s%s", st.origin.X, st.origin.Y, st.origin.Z,
					air ? "air" : "ground", (st.flags & FL_DUCKING) ? " +duck" : "");
				if (g_maxgain[g_cursor] > 0.001f)
					ImGui::Text("speed %.1f u/s   gain %+.2f (max %+.2f)   eff %.0f%%   yaw %.1f",
						g_speed[g_cursor], g_gain[g_cursor], g_maxgain[g_cursor],
						g_eff[g_cursor] * 100.f, g_frames[g_cursor].viewangles[1]);
				else
					ImGui::Text("speed %.1f u/s   gain %+.2f   eff n/a (not an air tick)   yaw %.1f",
						g_speed[g_cursor], g_gain[g_cursor], g_frames[g_cursor].viewangles[1]);

				if (g_sel >= 0 && g_sel < static_cast<int>(g_starts.size())) {
					const int b = g_starts[g_sel];
					int e = (g_sel + 1 < static_cast<int>(g_starts.size())) ? g_starts[g_sel + 1] : g_total;
					if (e > static_cast<int>(g_states.size()))
						e = static_cast<int>(g_states.size());
					float eff_sum = 0.f; int eff_n = 0;
					for (int i = b; i < e; ++i)
						if (g_maxgain[i] > 0.001f) { eff_sum += g_eff[i]; eff_n++; }
					const float v0 = (b > 0 && b - 1 < static_cast<int>(g_speed.size())) ? g_speed[b - 1] : Speed2D(g_anchor.velocity);
					const float v1 = (e > b) ? g_speed[e - 1] : v0;
					ImGui::Text("segment: %.1f -> %.1f u/s (%+.1f)   avg air eff %.0f%% (%d ticks)",
						v0, v1, v1 - v0, eff_n ? (eff_sum / eff_n) * 100.f : 0.f, eff_n);
				}
			}
		}

		ImGui::Separator();

		if (ImGui::CollapsingHeader("Strafe model (match server cvars)")) {
			bool ch = false;
			ImGui::PushItemWidth(120);
			ch |= ImGui::InputFloat("Air speed cap", &g_air_cap, 1.f, 10.f, 1);
			ch |= ImGui::InputFloat("sv_airaccelerate", &g_air_accel, 1.f, 10.f, 1);
			ch |= ImGui::InputFloat("Wish speed", &g_wishspeed, 10.f, 50.f, 0);
			ch |= ImGui::InputFloat("sv_gravity", &g_gravity, 10.f, 50.f, 0);
			ImGui::PopItemWidth();
			if (ch)
				MarkDirty();
		}

		const float sim_ms = Prediction::LastDiag().sim_ms;
		const char* sim_state =
			g_dirty ? "line update queued - it recomputes by itself, nothing to do" :
			g_sim_requested ? "recomputing the line..." :
			g_sim_fault ? "SIM FAULTED (recovered - see prediction diagnostics)" :
			g_valid ? "line up to date" : "no sim yet";
		if (g_valid && sim_ms > 0.f)
			ImGui::Text("Sim: %s   (%.1f ms/pass - %s)", sim_state, sim_ms,
				sim_ms < 8.f ? "live preview" : "throttled for length");
		else
			ImGui::Text("Sim: %s", sim_state);
	}

	void DrawTargetsTab() {
		const BspWorld::Status bs = BspWorld::GetStatus();
		if (!bs.loaded) {
			ImGui::TextColored(ImVec4(1.f, 0.75f, 0.3f, 1.f),
				"Map geometry not loaded yet.");
			if (ImGui::Button("Load map geometry"))
				BspWorld::LoadCurrentMap();
			if (bs.error[0]) {
				ImGui::PushTextWrapPos(0.f);
				ImGui::TextColored(ImVec4(1.f, 0.6f, 0.2f, 1.f), "BSP: %s", bs.error);
				ImGui::PopTextWrapPos();
			}
			return;
		}

		ImGui::TextWrapped(
			"Aim at a surface and Pick. Tag mode marks a face as surfable (gold "
			"outline); Target mode rests the player hull against the face exactly "
			"where the prediction would drop it, storing position + view angles - "
			"these are the solver's destinations. Bind the 'Editor: Pick At "
			"Crosshair' hotkey to pick with the menu closed.");

		ImGui::RadioButton("Tag surf face", &g_pick_mode, 0);
		ImGui::SameLine();
		ImGui::RadioButton("Place board target", &g_pick_mode, 1);
		ImGui::SameLine();
		if (ImGui::Button("Pick at crosshair"))
			TasEditor::PickAtCrosshair();
		if (g_pick_info[0]) {
			ImGui::PushTextWrapPos(0.f);
			ImGui::TextDisabled("%s", g_pick_info);
			ImGui::PopTextWrapPos();
		}

		ImGui::Separator();
		ImGui::Text("Tagged surf faces: %d", BspWorld::TagCount());
		ImGui::SameLine();
		if (ImGui::Button("Clear all tags")) {
			BspWorld::ClearTags();
			g_sel_tag = -1;
		}
		ImGui::BeginChild("tags", ImVec2(0, 90), true);
		for (int i = 0; i < BspWorld::TagCount(); ++i) {
			int brush = -1, plane = -1;
			if (!BspWorld::GetTag(i, &brush, &plane))
				continue;
			Vector n;
			float d = 0.f;
			BspWorld::GetPlane(plane, &n, &d);
			const float slope = Deg(acosf(Clampf(n.Z, -1.f, 1.f)));
			Vector centroid(0.f, 0.f, 0.f);
			const Vector* pts = nullptr;
			int count = 0;
			if (BspWorld::GetFacePolygon(brush, plane, &pts, &count) && count > 0) {
				for (int p = 0; p < count; ++p)
					centroid = centroid + pts[p];
				centroid = Scale(centroid, 1.f / static_cast<float>(count));
			}
			char label[256];
			Sfmt(label, "#%d  brush %d plane %d   slope %.0f deg   @ %.0f %.0f %.0f##tag%d",
				i, brush, plane, slope, centroid.X, centroid.Y, centroid.Z, i);
			if (ImGui::Selectable(label, g_sel_tag == i))
				g_sel_tag = i;
		}
		if (BspWorld::TagCount() == 0)
			ImGui::TextDisabled("(none - tag one with Pick)");
		ImGui::EndChild();
		if (g_sel_tag >= 0 && g_sel_tag < BspWorld::TagCount()) {
			if (ImGui::Button("Untag selected face")) {
				BspWorld::RemoveTag(g_sel_tag);
				g_sel_tag = -1;
			}
		}

		ImGui::Separator();
		ImGui::Text("Board targets: %d", BspWorld::TargetCount());
		ImGui::BeginChild("targets", ImVec2(0, 90), true);
		for (int i = 0; i < BspWorld::TargetCount(); ++i) {
			const BspWorld::BoardTarget* t = BspWorld::GetTarget(i);
			char label[256];
			Sfmt(label, "#%d  %.0f %.0f %.0f   yaw %.1f%s##tgt%d",
				i, t->pos.X, t->pos.Y, t->pos.Z, t->yaw, t->ducked ? "  duck" : "", i);
			if (ImGui::Selectable(label, g_sel_target == i))
				g_sel_target = i;
		}
		if (BspWorld::TargetCount() == 0)
			ImGui::TextDisabled("(none - place one with Pick)");
		ImGui::EndChild();

		BspWorld::BoardTarget* t = BspWorld::GetTarget(g_sel_target);
		if (t) {
			bool ch = false;
			ch |= FloatRow("Target view yaw", &t->yaw, -180.f, 180.f, "%.1f deg", 0.1f, 5.f, 1);
			bool lock = t->pitch_lock;
			if (ImGui::Checkbox("Pitch locked to ramp surface", &lock)) {
				t->pitch_lock = lock;
				ch = true;
			}
			ImGui::SameLine();
			ImGui::TextDisabled("(view line stays parallel to the face as yaw changes)");
			if (t->pitch_lock) {
				// Pitch is derived from the face for whatever yaw is set.
				BspWorld::LockedPitch(t->plane, t->yaw, &t->pitch);
				ImGui::Text("Target view pitch: %.1f deg (locked to face)", t->pitch);
			} else {
				ch |= FloatRow("Target view pitch", &t->pitch, -89.f, 89.f, "%.1f deg", 0.1f, 5.f, 1);
			}
			bool ducked = t->ducked;
			if (ImGui::Checkbox("Ducked hull", &ducked)) {
				t->ducked = ducked;
				BspWorld::RestTargetOnFace(g_sel_target);
				ch = true;
			}
			ImGui::SameLine();
			if (ImGui::Button("Use my view angles")) {
				StartState s;
				if (Prediction::CaptureStartState(s)) {
					t->yaw = s.yaw;
					if (!t->pitch_lock)
						t->pitch = s.pitch;
					ch = true;
				}
			}
			ImGui::SameLine();
			if (ImGui::Button("Remove target")) {
				BspWorld::RemoveTarget(g_sel_target);
				g_sel_target = -1;
				ch = false;
			}
			if (ch)
				BspWorld::SaveGeo();
		}

		ImGui::TextDisabled("Tags and targets auto-save per map (Documents\\sourceTAS\\geometry).");
	}

	void DrawRenderingTab() {
		ImGui::Text("Player & world");
		ImGui::Checkbox("Test marker at world origin", &WorldDraw::draw_test_marker);
		ImGui::SameLine();
		ImGui::Checkbox("Player collision hull", &WorldDraw::draw_player_box);
		ImGui::SameLine();
		ImGui::Checkbox("Feet marker", &WorldDraw::draw_player_marker);
		IntRow("Hull alpha (0 = wireframe)", &WorldDraw::player_box_alpha, 0, 160);
		FloatRow("Overlay lifetime (x frame)", &WorldDraw::overlay_life_scale, 0.5f, 4.f, "%.2f", 0.05f, 0.2f);
		ImGui::Checkbox("Pause in-world draws while the solver works (crash isolation)",
			&WorldDraw::pause_draw_busy);
		if (WorldDraw::pause_draw_busy)
			ImGui::TextDisabled("lines/hulls vanish during search+verify+correction; if the "
				"silent crashes stop with this ON, the engine overlay race is confirmed");

		ImGui::Separator();
		ImGui::Text("Live prediction (from the player)");
		ImGui::Checkbox("Predict path", &WorldDraw::draw_prediction);
		ImGui::SameLine();
		ImGui::Checkbox("Live input", &WorldDraw::pred_live_input);
		ImGui::SameLine();
		ImGui::Checkbox("Auto-bhop", &WorldDraw::pred_autobhop);
		IntRow("Predict ticks", &WorldDraw::pred_ticks, 1, 200);
		if (!WorldDraw::pred_live_input) {
			FloatRow("Forward move", &WorldDraw::pred_forwardmove, -450.f, 450.f, "%.0f", 10.f, 50.f, 0);
			FloatRow("Side move", &WorldDraw::pred_sidemove, -450.f, 450.f, "%.0f", 10.f, 50.f, 0);
			ImGui::Checkbox("Jump", &WorldDraw::pred_jump);
			ImGui::SameLine();
			ImGui::Checkbox("Duck", &WorldDraw::pred_duck);
		}

		ImGui::Separator();
		ImGui::Text("Hull corner trails (bottom corners of the hull, along every path line)");
		ImGui::Checkbox("+X+Y##ct", &WorldDraw::corner_trails[0]);
		ImGui::SameLine();
		ImGui::Checkbox("+X-Y##ct", &WorldDraw::corner_trails[1]);
		ImGui::SameLine();
		ImGui::Checkbox("-X+Y##ct", &WorldDraw::corner_trails[2]);
		ImGui::SameLine();
		ImGui::Checkbox("-X-Y##ct", &WorldDraw::corner_trails[3]);
		ImGui::SameLine();
		if (ImGui::SmallButton("all##ct"))
			for (int c = 0; c < 4; ++c) WorldDraw::corner_trails[c] = true;
		ImGui::SameLine();
		if (ImGui::SmallButton("none##ct"))
			for (int c = 0; c < 4; ++c) WorldDraw::corner_trails[c] = false;

		ImGui::Separator();
		ImGui::Text("Editor run line");
		ImGui::Checkbox("Ghost hull at playhead", &g_show_hull);
		ImGui::SameLine();
		ImGui::Checkbox("Replay HUD", &WorldDraw::show_replay_hud);
		FloatRow("Min efficiency (line coloring only)", &g_min_eff, 0.90f, 1.f, "%.2f", 0.005f, 0.02f);

		ImGui::Separator();
		ImGui::Text("World geometry (BSP)");
		ImGui::Checkbox("Brush wireframes", &BspWorld::draw_wireframe);
		ImGui::SameLine();
		ImGui::Checkbox("Tags & targets", &BspWorld::show_markers);
		FloatRow("Wire radius", &BspWorld::draw_radius, 200.f, 6000.f, "%.0f", 50.f, 250.f, 0);
		IntRow("Wire line budget", &BspWorld::line_budget, 100, 3000);
		ImGui::Checkbox("Solid##bsp", &BspWorld::show_solid);
		ImGui::SameLine();
		ImGui::Checkbox("Player clip##bsp", &BspWorld::show_playerclip);
		ImGui::SameLine();
		ImGui::Checkbox("Ladder##bsp", &BspWorld::show_ladder);
		ImGui::SameLine();
		if (ImGui::Button("Reload##bsp"))
			BspWorld::LoadCurrentMap();

		const BspWorld::Status bs = BspWorld::GetStatus();
		if (bs.loaded) {
			ImGui::Text("%s (v%d): %d brushes, %d faces, %d edges (%.0f ms parse)",
				bs.map, bs.bsp_version, bs.brush_count, bs.face_count, bs.edge_count, bs.parse_ms);
			ImGui::Text("drawing %d brushes / %d lines", bs.drawn_brushes, bs.drawn_lines);
		} else if (bs.error[0]) {
			ImGui::PushTextWrapPos(0.f);
			ImGui::TextColored(ImVec4(1.f, 0.6f, 0.2f, 1.f), "BSP: %s", bs.error);
			ImGui::PopTextWrapPos();
		} else {
			ImGui::TextDisabled("BSP: not loaded (enable wireframes while in a map)");
		}
	}

	void DrawProjectTab() {
		ImGui::PushItemWidth(180);
		ImGui::InputText("##projname", g_name, sizeof(g_name));
		ImGui::PopItemWidth();
		ImGui::SameLine();
		if (ImGui::Button("Save project"))
			SaveProject();
		ImGui::SameLine();
		if (ImGui::Button("Refresh list"))
			RefreshFiles();

		ImGui::BeginChild("projfiles", ImVec2(0, 90), true);
		for (int i = 0; i < static_cast<int>(g_files.size()); ++i) {
			if (ImGui::Selectable((g_files[i] + "##proj").c_str(), g_sel_file == i))
				g_sel_file = i;
		}
		if (g_files.empty())
			ImGui::TextDisabled("(no saved projects)");
		ImGui::EndChild();

		if (ImGui::Button("Load selected project") && g_sel_file >= 0 && g_sel_file < static_cast<int>(g_files.size()))
			LoadProject(g_files[g_sel_file]);
		ImGui::SameLine();
		if (ImGui::Button("Delete selected project"))
			DeleteProjectFile();
		ImGui::SameLine();
		if (ImGui::Button("Export to library"))
			ExportToLibrary();
		ImGui::SameLine();
		if (ImGui::Button("Import selected recording"))
			ImportFromLibrary();
	}
}

// ------------------------------------------------------------------- public --
void TasEditor::Update() {
	// Last-resort crash writer for faults OUTSIDE every frame guard (any
	// thread): names module+offset in calibration\crash.log before dying.
	static bool s_crash_filter = false;
	if (!s_crash_filter) {
		s_crash_filter = true;
		g_prev_uef = SetUnhandledExceptionFilter(&CrashWriter);
		g_veh_handle = AddVectoredExceptionHandler(1, &FirstChanceWriter);
	}

	// Pump the solver search (time-sliced; no-op when idle).
	StepSearch();

	// Teleport landing check: report the measured delta from the anchor, and
	// persist it (a later crash or status overwrite must not eat the number).
	if (g_tp_check && GetTickCount64() - g_tp_ms >= 400) {
		g_tp_check = false;
		StartState now;
		if (g_anchor.valid && Prediction::CaptureStartState(now)) {
			const Vector d = now.origin - g_anchor.origin;
			const float yr = g_anchor.yaw * (kPi / 180.f);
			const std::string line = FmtStr(
				"teleport: anchor=(%.4f %.4f %.4f) landed=(%.4f %.4f %.4f) "
				"delta=(%.4f %.4f %.4f) = %.4f fwd, %.4f side, %.4f up (yaw %.2f)",
				g_anchor.origin.X, g_anchor.origin.Y, g_anchor.origin.Z,
				now.origin.X, now.origin.Y, now.origin.Z,
				d.X, d.Y, d.Z,
				d.X * cosf(yr) + d.Y * sinf(yr),
				-d.X * sinf(yr) + d.Y * cosf(yr),
				d.Z, g_anchor.yaw);
			g_status = line;
			const std::string dir = CalibDir();
			if (!dir.empty()) {
				std::ofstream f(dir + "\\teleport.log", std::ios::app);
				if (f)
					f << line << "\n";
			}
		}
	}

	// Slider settled? Start the deferred auto-correction (one, not per notch).
	if (g_pick_correct_pending && GetTickCount64() - g_pick_settle_ms >= 350
		&& !g_verify.active && !g_batch.active && !g_search.active) {
		g_pick_correct_pending = false;
		if (g_correct.auto_run && g_sel >= 0 && g_sel < static_cast<int>(g_segs.size())
			&& g_segs[g_sel].is_solver)
			StartCorrect(g_sel);
	}

	// Corrections are PUMPED by landing sims - a correction armed while no
	// sim is in flight would never take its first step and g_correct.active
	// would stick true forever (which also kept SearchBusy true, so the
	// draw-pause blanked the world until a menu interaction forced a resim).
	// Kick a stalled correction once the current line is landed and clean.
	if (g_correct.active && g_correct.skip == 0 && g_valid && !g_dirty
		&& !g_sim_requested && !Prediction::SimBusy()
		&& !g_verify.active && !g_batch.active) {
		g_stage_name = "CorrectPump(kick)";
		CorrectPump();
		g_stage_name = "";
	}

	// Land a finished sim, measure the REAL pass, feed the correction loop.
	// The whole chain runs under the same fault guard as the search jobs: a
	// crash anywhere in it becomes a NAMED stage in solver_fault.log and the
	// pumps stop, instead of a dead game with no evidence.
	if (Prediction::SimReady()) {
		if (!SimLandedGuarded()) {
			char note[256];
			Sfmt(note, "EDITOR FAULT 0x%08X in stage %s (verify idx %d, batch idx %d) "
				"- pumps stopped; report this line.",
				g_guard_code, g_stage_name[0] ? g_stage_name : "TakeSim",
				g_verify.idx, g_batch.idx);
			g_status = note;
			const std::string dir = CalibDir();
			if (!dir.empty()) {
				std::ofstream f(dir + "\\solver_fault.log", std::ios::app);
				if (f)
					f << note << "\n";
			}
			g_verify.active = false;
			g_batch.active = false;
			g_correct.active = false;
		}
	}

	// Live preview: resim as fast as the sim itself allows (measured, not
	// guessed); only long runs get a short debounce to protect the framerate.
	const float sim_ms = Prediction::LastDiag().sim_ms;
	const unsigned long long debounce_ms =
		(sim_ms <= 8.f) ? 0 : (sim_ms <= 25.f) ? 60 : 200;
	if (g_dirty && !g_sim_requested && !Prediction::SimBusy()
		&& GetTickCount64() - g_dirty_ms >= debounce_ms
		&& g_total > 0 && g_anchor.valid) {
		if (BuildPlan()) {
			// Ending on a solved solver segment? Sim a little past the plan so
			// the real line reaches the ramp and shows the board (the plan and
			// the playhead range stay g_total; the extra states are analysis).
			int sim_ticks = g_total;
			if (g_plan_count > 0 && g_plan[g_plan_count - 1].solver
				&& g_plan[g_plan_count - 1].ticks > 0)
				sim_ticks += kSolverOverrun;
			if (sim_ticks > Prediction::kMaxSimTicks)
				sim_ticks = Prediction::kMaxSimTicks;
			if (Prediction::RequestSim(g_anchor, sim_ticks, &Provider)) {
				g_sim_requested = true;
				g_dirty = false;
			}
		}
	}
}

bool TasEditor::IsOpen() { return g_open; }
void TasEditor::Toggle() {
	g_open = !g_open;
	if (g_open)
		RefreshFiles();
}

void TasEditor::StepCursor(int delta) {
	g_cursor += delta;
	if (g_cursor < 0) g_cursor = 0;
	if (g_cursor >= g_total) g_cursor = g_total > 0 ? g_total - 1 : 0;
}

void TasEditor::StepSegment(int direction) {
	if (g_starts.empty())
		return;
	int seg = SegmentAtTick(g_cursor) + direction;
	if (seg < 0) seg = 0;
	if (seg >= static_cast<int>(g_starts.size())) seg = static_cast<int>(g_starts.size()) - 1;
	g_cursor = g_starts[seg];
}

void TasEditor::PickAtCrosshair() {
	if (!BspWorld::GetStatus().loaded && !BspWorld::LoadCurrentMap()) {
		g_status = "Pick: map geometry not loaded.";
		return;
	}

	StartState s;
	if (!Prediction::CaptureStartState(s)) {
		g_status = "Pick: not in game.";
		return;
	}

	const float d2r = kPi / 180.f;
	const float pr = s.pitch * d2r;
	const float yr = s.yaw * d2r;
	const Vector dir(cosf(pr) * cosf(yr), cosf(pr) * sinf(yr), -sinf(pr));
	const Vector eye = s.origin + Vector(0.f, 0.f, s.ducked ? 46.f : 64.f);

	const BspWorld::RayHit hit = BspWorld::Pick(eye, dir, 8192.f);
	BspWorld::NotePickDebug(eye, dir, hit);

	// Forensics for the Targets tab: what the ray was and what it struck.
	{
		const char* what = !hit.hit ? "MISS"
			: (hit.contents & 0x10000) ? "PLAYERCLIP (invisible)"
			: (hit.contents & 0x8) ? "grate" : "solid";
		Sfmt(g_pick_info, "last pick: yaw %.1f pitch %.1f | eye %.0f %.0f %.0f | %s"
			" brush %d plane %d dist %.0f @ %.0f %.0f %.0f",
			s.yaw, s.pitch, eye.X, eye.Y, eye.Z, what,
			hit.brush, hit.plane, hit.dist, hit.point.X, hit.point.Y, hit.point.Z);
	}

	if (!hit.hit) {
		g_status = "Pick: nothing hit within 8192 units.";
		return;
	}

	char message[384];
	if (g_pick_mode == 0) {
		const bool tagged = BspWorld::ToggleFaceTag(hit.brush, hit.plane);
		Sfmt(message, "%s face (brush %d, plane %d) at %.0f %.0f %.0f",
			tagged ? "Tagged" : "Untagged", hit.brush, hit.plane,
			hit.point.X, hit.point.Y, hit.point.Z);
	} else {
		const int index = BspWorld::AddTargetFromHit(hit, s.yaw, s.pitch, false);
		if (index >= 0) {
			g_sel_target = index;
			Sfmt(message, "Board target #%d placed (surface normal %.2f %.2f %.2f)",
				index, hit.normal.X, hit.normal.Y, hit.normal.Z);
		} else {
			Sfmt(message, "Couldn't place a target there.");
		}
	}
	g_status = message;
}

bool TasEditor::SearchBusy() {
	return g_search.active || g_verify.active || g_batch.active || g_correct.active;
}

bool TasEditor::SearchHeavy() {
	return g_search.active || g_verify.active || g_batch.active;
}

void TasEditor::NoteExternalFault(const char* where, int code) {
	char note[256];
	Sfmt(note, "EDITOR FAULT 0x%08X in stage %s - report this line.", code, where);
	g_status = note;
	const std::string dir = CalibDir();
	if (!dir.empty()) {
		std::ofstream f(dir + "\\solver_fault.log", std::ios::app);
		if (f)
			f << note << "\n";
	}
}

bool TasEditor::GetDrawData(DrawData& out) {
	if (!g_open || !g_valid || g_states.empty())
		return false;
	out.states = g_states.data();
	out.count = static_cast<int>(g_states.size());
	out.seg_starts = g_starts.data();
	out.seg_count = static_cast<int>(g_starts.size());
	out.eff = g_eff.empty() ? nullptr : g_eff.data();
	out.maxgain = g_maxgain.empty() ? nullptr : g_maxgain.data();
	out.min_eff = g_min_eff;
	out.cursor = g_cursor;
	if (g_sel >= 0 && g_sel < static_cast<int>(g_starts.size())) {
		out.sel_start = g_starts[g_sel];
		out.sel_end = (g_sel + 1 < static_cast<int>(g_starts.size())) ? g_starts[g_sel + 1] : g_total;
	} else {
		out.sel_start = out.sel_end = -1;
	}
	out.anchor = g_anchor;
	out.show_hull = g_show_hull;
	out.have_view = false;
	if (g_cursor >= 0 && g_cursor < static_cast<int>(g_frames.size())) {
		out.have_view = true;
		out.view_pitch = g_frames[g_cursor].viewangles[0];
		out.view_yaw = g_frames[g_cursor].viewangles[1];
	}
	out.have_pass = g_realpass.computed && g_realpass.crossed;
	out.pass_tick = g_realpass.tick;
	out.pass_point = g_realpass.pass;
	return true;
}

namespace {
	// The whole editor UI, guarded: a fault anywhere inside becomes a named
	// report; the ImGui window stack is rebalanced so EndFrame survives, and
	// the editor UI disables itself instead of killing the game.
	bool g_ui_disabled = false;
	void DrawWindowImpl();
	bool DrawWindowGuarded() {
		__try {
			DrawWindowImpl();
			return true;
		} __except (EXCEPTION_EXECUTE_HANDLER) {
			g_guard_code = static_cast<int>(GetExceptionCode());
			return false;
		}
	}
}

void TasEditor::DrawWindow() {
	if (g_ui_disabled) {
		ImGui::SetNextWindowPos(ImVec2(12, 12), ImGuiSetCond_FirstUseEver);
		ImGui::Begin("sourceTAS Editor FAULT", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
		ImGui::TextColored(ImVec4(1.f, 0.4f, 0.4f, 1.f),
			"The editor UI hit a fault and disabled itself (game kept alive).");
		ImGui::TextWrapped("%s", g_status.c_str());
		ImGui::End();
		return;
	}
	if (!DrawWindowGuarded()) {
		g_ui_disabled = true;
		// Pop any windows the faulting frame left unbalanced - an unmatched
		// Begin would assert (and kill us) in ImGui::EndFrame otherwise.
		ImGuiContext& ctx = *GImGui;
		while (ctx.CurrentWindowStack.Size > 1)
			ImGui::End();
		char note[256];
		Sfmt(note, "EDITOR FAULT 0x%08X in stage DrawWindow - UI disabled, game kept alive.",
			g_guard_code);
		g_status = note;
		const std::string dir = CalibDir();
		if (!dir.empty()) {
			std::ofstream f(dir + "\\solver_fault.log", std::ios::app);
			if (f)
				f << note << "\n";
		}
	}
}

namespace {
void DrawWindowImpl() {
	if (!g_open)
		return;

	BspWorld::highlight_target = g_sel_target;
	BspWorld::highlight_tag = g_sel_tag;

	// Default-open skips Toggle(), so refresh the project list once lazily.
	static bool s_refreshed = false;
	if (!s_refreshed) {
		s_refreshed = true;
		RefreshFiles();
	}

	// Bottom-left by default (NoSavedSettings means this applies every launch).
	{
		const ImVec2 disp = ImGui::GetIO().DisplaySize;
		float wy = disp.y - 820.f - 12.f;
		if (wy < 10.f) wy = 10.f;
		ImGui::SetNextWindowPos(ImVec2(12.f, wy), ImGuiSetCond_FirstUseEver);
	}
	ImGui::SetNextWindowSize(ImVec2(920, 820), ImGuiSetCond_FirstUseEver);
	if (!ImGui::Begin("sourceTAS Editor", &g_open, ImGuiWindowFlags_NoSavedSettings)) {
		ImGui::End();
		return;
	}

	// Tab row.
	const char* tabs[] = { "Run", "Targets", "Rendering", "Project" };
	for (int i = 0; i < 4; ++i) {
		const bool active = (g_tab == i);
		if (active)
			ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.26f, 0.46f, 0.80f, 1.f));
		if (ImGui::Button(tabs[i], ImVec2(110, 0)))
			g_tab = i;
		if (active)
			ImGui::PopStyleColor();
		if (i < 3)
			ImGui::SameLine();
	}
	ImGui::Separator();

	switch (g_tab) {
	case 0: DrawRunTab(); break;
	case 1: DrawTargetsTab(); break;
	case 2: DrawRenderingTab(); break;
	default: DrawProjectTab(); break;
	}

	ImGui::Separator();
	ImGui::TextWrapped("%s", g_status.c_str());
	ImGui::End();
}
}   // namespace (DrawWindowImpl)
