#include "TasEditor.h"

#include <algorithm>
#include <array>
#include <cmath>
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
		changed |= ImGui::InputFloat("##i", v, step, fast, decimals);
		ImGui::PopItemWidth();
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
		int   yaw_mode = 0;           // 0 = hold, 1 = turn at yaw_rate
		float yaw_rate = 0.f;         // deg/tick, screen sign (+ = right)
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
		int   target = -1;            // board target index
		int   strafes = 2;            // alternating strafe count (1..4)
		bool  couple = true;          // perfect-board coupling (off for floors/pads)
		bool  start_jump = false;     // jump on the segment's first tick (grounded start)
		bool  solved = false;
		int   ticks = 0;
		int   side0 = 1;
		int   nsplits = 0;
		int   splits[3] = {};
		float rates[4] = {};
		// The model's predicted exact-contact point for the applied solution -
		// compared against the REAL sim's contact to expose model drift.
		Vector model_contact = Vector(0.f, 0.f, 0.f);
		float  model_land_err = 0.f;
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

	// ---------------------------------------------------------------- state --
	bool g_open = false;
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
	char g_pick_info[256] = {};       // last pick forensics (ray + hit)

	// Air-strafe model (match server settings; defaults = stock CS:S) + the
	// visual "optimal enough" threshold used by the line coloring.
	float g_air_cap = 30.f;
	float g_air_accel = 10.f;
	float g_wishspeed = 250.f;
	float g_gravity = 800.f;
	float g_min_eff = 0.98f;
	float g_sol_pos_tol = 2.f;        // solver EXACT landing tolerance (u) - hard gate
	float g_sol_max_loss = 100.f;     // coupled boards losing more than this are NOT solutions
	float g_sol_head_pref = 0.3f;     // ranking: u/s of score per degree off the target yaw

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
		// Target face support plane (through the target's rest origin), for the
		// provider's post-board surf-hold.
		bool have_face; Vector face_n; float face_d;
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
			// Solver segment: piecewise alternating strafes from the applied
			// solution - the view turns at the strafe's rate each tick, only the
			// side key is held (W never, airborne throughout).
			const SolverData& sd = ps.sdata;

			// Once the hull is actually RIDING the target face (inside the
			// support-plane shell with the approach spent), switch to smooth
			// surf-hold: aim along the slide, hold the strafe key whose wish
			// presses INTO the ramp - the energy-preserving ride, instead of
			// blindly continuing the final strafe. Detected per tick from the
			// simulated state, so it also covers the real board landing a tick
			// or two off the model's prediction.
			bool surfing = false;
			if (sd.couple && ps.have_face) {
				const float phi = ps.face_n.X * prev.origin.X + ps.face_n.Y * prev.origin.Y
					+ ps.face_n.Z * prev.origin.Z - ps.face_d;
				const float nvel = ps.face_n.X * prev.velocity.X + ps.face_n.Y * prev.velocity.Y
					+ ps.face_n.Z * prev.velocity.Z;
				surfing = (phi < 2.f) && (nvel > -20.f);
			}
			if (surfing) {
				const float sp2 = Speed2D(prev.velocity);
				yaw = (sp2 > 10.f) ? NormYaw(Deg(atan2f(prev.velocity.Y, prev.velocity.X)))
				                   : g_pv_last_yaw;
				const float ar = (yaw + 90.f) * (kPi / 180.f);
				const float into_a = cosf(ar) * ps.face_n.X + sinf(ar) * ps.face_n.Y;
				smove = (into_a < 0.f) ? -450.f : 450.f;   // press into the face
			} else {
				int k = 0;
				while (k < sd.nsplits && t >= sd.splits[k]) k++;
				const int side = (k % 2 == 0) ? sd.side0 : -sd.side0;
				yaw = NormYaw(g_pv_last_yaw - sd.rates[k]);
				smove = (side > 0) ? -450.f : 450.f;   // +1 = A (left), -1 = D (right)
			}
		} else {
			// View: hold or turn at a fixed rate (screen sign: + = right = Source
			// yaw decrease).
			if (g.yaw_mode == 1)
				yaw = g_pv_entry_yaw - g.yaw_rate * static_cast<float>(t);
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
			p.have_face = false;
			p.face_n = Vector(0.f, 0.f, 1.f);
			p.face_d = 0.f;
			if (p.solver) {
				const BspWorld::BoardTarget* bt = BspWorld::GetTarget(s.solver.target);
				Vector fn;
				if (bt && BspWorld::GetPlane(bt->plane, &fn, nullptr)) {
					p.have_face = true;
					p.face_n = fn;
					p.face_d = fn.X * bt->pos.X + fn.Y * bt->pos.Y + fn.Z * bt->pos.Z;
				}
			}

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

	// ================================================================ solver --
	// Two-tier: the SEARCH runs on a fast in-process model (exact Source air
	// accel + half-tick gravity, no collision - solver segments are airborne),
	// time-sliced so the game never hitches. The APPLIED solution compiles into
	// a real segment and runs through the engine sim, so the drawn line is
	// always ground truth.
	float Clampf(float v, float lo, float hi) { return v < lo ? lo : v > hi ? hi : v; }
	Vector Scale(const Vector& v, float s) { return Vector(v.X * s, v.Y * s, v.Z * s); }
	float Dot3(const Vector& v) { return v.X * v.X + v.Y * v.Y + v.Z * v.Z; }
	float Dot(const Vector& a, const Vector& b) { return a.X * b.X + a.Y * b.Y + a.Z * b.Z; }

	struct RouteSolution {
		int   ticks = 0;
		int   side0 = 1;
		int   nsplits = 0;
		int   splits[3] = {};
		float rates[4] = {};
		float pos_err = 0.f, head_err = 0.f, arrive_speed = 0.f;
		float landing_err = 0.f;      // distance of the interpolated contact from target
		Vector contact_pos = Vector(0.f, 0.f, 0.f);
		// board coupling
		bool  contact = false;
		float loss = 0.f;
		bool  retained = false;
		float post_speed = 0.f;
		bool  perfect = false;
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
		int    tbrush = -1, tplane = -1;
		Vector face_n;
		float  face_d = 0.f;
		Vector e1, e2;                // in-plane basis for the landing residual
		bool   have_face = false;
		int    strafes = 2;
		bool   couple = true;
		float  dt = 0.015f, accel_amt = 37.5f, cap = 30.f, gravity = 800.f;
		float  exact_tol = 2.f;       // hard landing gate (u): position is non-negotiable
		float  coarse_basin = 120.f;  // how near tick-end must be to attempt exact refine
		float  max_loss = 100.f;      // coupled board gate: lossier boards are rejected
		float  head_pref = 0.3f;      // soft ranking preference toward the target yaw
		// closest rejected candidate, for the "why infeasible" readout
		bool   have_reject = false;
		float  reject_land = 1e9f, reject_coarse = 1e9f;
		int    reject_ticks = 0;
		int    reject_board_n = 0;    // exact landings rejected for board quality
		float  reject_board_best = 1e9f;   // their lowest loss
		// job enumeration
		std::vector<int> ticks_band;
		int job_tick_idx = 0;
		int job_side = 0;
		std::vector<std::array<int, 3>> splits_list;
		int job_split_idx = 0;
		int splits_built_for = -1;
		int jobs_done = 0, jobs_total = 0;
		std::vector<RouteSolution> results;
	};
	// Board "perfect" tag threshold (u/s of speed lost at the clip). Display
	// only - ranking is continuous by post-board speed, so nothing gates on it.
	constexpr float kPerfectLoss = 1.0f;
	SearchCtx g_search;
	int g_solution_pick = 0;

	// ------------------------------------------------ real-landing analysis --
	// The drawn line is the REAL engine sim. These are ITS landing numbers
	// against the applied solver target - the ground truth that the model's
	// "landing X u" claim is checked against, recomputed every time a sim lands.
	struct RealLanding {
		bool computed = false;        // a solved solver segment + face existed
		bool contacted = false;
		bool on_face = false;
		int  tick = -1;               // global sim tick of first face contact
		Vector touch;
		float err = 0.f;              // |touch - target| (the number that matters)
		float approach_normal = 0.f;  // u/s into the face just before contact
		float before2d = 0.f, after2d = 0.f;
		float drift = 0.f;            // |model's predicted contact - real touch|
		float closest_phi = 1e9f;     // when !contacted: nearest approach to the plane
		float grav_fit = 0.f;         // engine gravity measured off the simmed states
		float cap_fit = 0.f;          // engine air-speed cap measured likewise
		int   fit_samples = 0;
	};
	RealLanding g_real;

	// ------------------------------------------------- real-sim landing polish --
	// The search lands exactly in the FAST MODEL; any model-vs-engine drift
	// shows up as the real line missing the target. Rather than chasing
	// constants, close the loop: after a solution applies, descend its last two
	// strafe rates ON THE REAL SIM (one evaluation per landed sim, a few dozen
	// frames total) until the REAL contact point sits on the target. Same LM
	// structure as RefineNewton, paced by the sim pipeline.
	struct PolishCtx {
		bool auto_run = true;         // UI: polish automatically after apply/pick
		bool active = false, done = false;
		int  seg = -1;
		int  K = 0, ncols = 0;
		int  cols[2] = {};
		float lo[4] = {}, hi[4] = {};
		Vector e1, e2, tgt;
		float center[4] = {};         // accepted rates
		float F[2] = {};              // residual at center
		float err = 1e9f;
		int   phase = 0;              // 0 center eval, 1/2 jacobian cols, 3 step eval
		float J[2][2] = {};
		float hs[2] = {};             // FD step actually used per column
		float lambda = 0.4f;
		float trial[4] = {};          // rates of the eval in flight
		int   skip = 0;               // sims in flight at start = stale, ignore them
		int   iter = 0, evals = 0;
		float best_err = 1e9f;
		float best_rates[4] = {};
		char  note[192] = {};
	};
	PolishCtx g_polish;

	struct FastState { Vector p, v; float yaw; };

	// One airborne tick: turn, half gravity, air-accelerate along the side key's
	// wishdir, integrate, half gravity. Mirrors the engine's order closely.
	void FastTick(FastState& s, float rate, int side) {
		s.yaw = NormYaw(s.yaw - rate);   // screen + = right = Source yaw decrease
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

	FastState FastRun(int ticks, int side0, const int* splits, int nsplits, const float* rates) {
		FastState s{ g_search.start_pos, g_search.start_vel, g_search.start_yaw };
		int k = 0;
		for (int t = 0; t < ticks; ++t) {
			while (k < nsplits && t >= splits[k]) k++;
			FastTick(s, rates[k], (k % 2 == 0) ? side0 : -side0);
		}
		return s;
	}

	void ScoreArrival(const FastState& s, float& pos_err, float& head_err, float& speed) {
		const float dx = s.p.X - g_search.target_pos.X;
		const float dy = s.p.Y - g_search.target_pos.Y;
		pos_err = sqrtf(dx * dx + dy * dy);
		speed = sqrtf(s.v.X * s.v.X + s.v.Y * s.v.Y);
		head_err = fabsf(NormYaw(Deg(atan2f(s.v.Y, s.v.X)) - g_search.target_yaw));
	}

	// Coordinate descent on the per-strafe rates for one (ticks, side, timing).
	// Each strafe's rate is HARD-clamped to its key's turn direction (A only
	// turns left, D only right) - keyboard-honest by construction, and it
	// halves the search space (offline tests showed the old soft penalty let
	// key-fighting degenerates through acceptance).
	//
	// POSITION ONLY. The old heading-weighted cost (he^2 * 8, in degrees) could
	// dominate the position term whenever the placed target yaw was far from the
	// route's natural arrival - dragging the descent away from the target and
	// failing the basin gate. That is exactly why 1-2 strafes "never" solved:
	// with only 1-2 rates there isn't freedom to satisfy both, and position is
	// the one that's non-negotiable. Heading is free (reported, never a cost).
	bool SolveRates(int ticks, int side0, const int* splits, int nsplits, RouteSolution& out) {
		const int K = nsplits + 1;
		float rates[4] = {};
		float rate_lo[4] = {}, rate_hi[4] = {};
		for (int i = 0; i < K; ++i) {
			const int side = (i % 2 == 0) ? side0 : -side0;
			if (side > 0) { rate_lo[i] = -8.f; rate_hi[i] = 0.f; rates[i] = -0.8f; }   // A: left
			else          { rate_lo[i] = 0.f;  rate_hi[i] = 8.f; rates[i] = 0.8f; }    // D: right
		}

		float pe = 0.f, he = 0.f, sp = 0.f;
		auto cost = [&](const float* r) {
			const FastState s = FastRun(ticks, side0, splits, nsplits, r);
			ScoreArrival(s, pe, he, sp);
			return pe;
		};

		float best = cost(rates);
		float best_pe = pe, best_he = he, best_sp = sp;
		float step = 1.2f;
		for (int iter = 0; iter < 160 && step > 0.002f; ++iter) {
			bool improved = false;
			for (int i = 0; i < K; ++i) {
				for (int dir = -1; dir <= 1; dir += 2) {
					const float saved = rates[i];
					rates[i] = Clampf(saved + dir * step, rate_lo[i], rate_hi[i]);
					const float c = cost(rates);
					if (c < best) {
						best = c; best_pe = pe; best_he = he; best_sp = sp;
						improved = true;
					} else {
						rates[i] = saved;
					}
				}
			}
			if (!improved)
				step *= 0.72f;
		}

		out.pos_err = best_pe;
		out.head_err = best_he;
		out.arrive_speed = best_sp;
		// Coarse gate: only "in the basin" of the exact-contact refinement.
		if (best_pe > g_search.coarse_basin)
			return false;

		out.ticks = ticks;
		out.side0 = side0;
		out.nsplits = nsplits;
		for (int i = 0; i < 3; ++i) out.splits[i] = (i < nsplits) ? splits[i] : 0;
		for (int i = 0; i < 4; ++i) out.rates[i] = (i < K) ? rates[i] : 0.f;
		return true;
	}

	// In-plane orthonormal basis for the 2D landing residual.
	void PlaneBasis(const Vector& n, Vector& e1, Vector& e2) {
		const Vector ref = (fabsf(n.Z) < 0.9f) ? Vector(0.f, 0.f, 1.f) : Vector(1.f, 0.f, 0.f);
		const Vector c(n.Y * ref.Z - n.Z * ref.Y, n.Z * ref.X - n.X * ref.Z, n.X * ref.Y - n.Y * ref.X);
		const float l = sqrtf(Dot3(c));
		e1 = Scale(c, 1.f / (l > 1e-6f ? l : 1.f));
		e2 = Vector(n.Y * e1.Z - n.Z * e1.Y, n.Z * e1.X - n.X * e1.Z, n.X * e1.Y - n.Y * e1.X);
	}

	struct ContactHit { bool ok = false; Vector c, v; float yaw = 0.f; int tick = 0; int side = 0; };

	// Simulate one candidate to its interpolated support-plane crossing. The
	// plane the hull ORIGIN crosses at contact passes through the target (which
	// IS a hull-origin rest position), parallel to the face.
	ContactHit SimToContact(const RouteSolution& sol, const float* r) {
		ContactHit out;
		const Vector n = g_search.face_n;
		const float d_origin = Dot(n, g_search.target_pos);
		FastState s{ g_search.start_pos, g_search.start_vel, g_search.start_yaw };
		int k = 0;
		float prev_phi = Dot(n, s.p) - d_origin;
		const int maxT = sol.ticks + 16;
		for (int t = 0; t < maxT; ++t) {
			while (k < sol.nsplits && t >= sol.splits[k]) k++;
			const int side = (k % 2 == 0) ? sol.side0 : -sol.side0;
			const Vector before = s.p;
			FastTick(s, r[k], side);
			const float ph = Dot(n, s.p) - d_origin;
			if (prev_phi > 0.f && ph <= 0.f) {
				const float tau = prev_phi / (prev_phi - ph);
				out.c = before + Scale(s.p - before, tau);
				out.v = s.v; out.yaw = s.yaw; out.tick = t + 1; out.side = side; out.ok = true;
				return out;
			}
			prev_phi = ph;
		}
		return out;
	}

	// Board metrics at the exact contact (Source-style clip against the face).
	void BoardMetrics(const ContactHit& hit, RouteSolution& sol) {
		const Vector n = g_search.face_n;
		const float d_origin = Dot(n, g_search.target_pos);
		const float into = Dot(n, hit.v);
		Vector clipped = hit.v;
		if (into < 0.f)
			clipped = hit.v - Scale(n, into);
		sol.loss = sqrtf(Dot3(hit.v)) - sqrtf(Dot3(clipped));
		FastState next{ hit.c, clipped, hit.yaw };
		FastTick(next, 0.f, hit.side);
		sol.retained = (Dot(n, next.p) - d_origin) <= 2.f;
		sol.post_speed = sqrtf(Dot3(next.v));
		sol.contact = true;
		sol.perfect = (sol.loss <= kPerfectLoss) && sol.retained;
	}

	void FillFromContact(RouteSolution& sol, const ContactHit& hit, float err) {
		sol.landing_err = err;
		sol.pos_err = err;
		sol.contact_pos = hit.c;
		sol.ticks = hit.tick;
		sol.arrive_speed = sqrtf(hit.v.X * hit.v.X + hit.v.Y * hit.v.Y);
		sol.head_err = fabsf(NormYaw(Deg(atan2f(hit.v.Y, hit.v.X)) - g_search.target_yaw));
		BoardMetrics(hit, sol);
	}

	// EXACT-CONTACT solve (position is non-negotiable). Integer ticks quantize
	// tick-END positions, but the hull ORIGIN crosses the plane through the
	// target continuously mid-tick, and that crossing moves smoothly with the
	// last strafe rates. The old coordinate descent stalled here: both rates
	// bend the path the same rotational way, so the 2D landing problem is a
	// narrow ill-conditioned valley (why 1-2 strafes "never" landed inside the
	// gate). A damped Newton step - finite-difference Jacobian + Levenberg
	// damping, rates bounded to their key's turn direction - follows that
	// valley directly. Heading stays free throughout (settles optimal).
	bool RefineNewton(RouteSolution& sol) {
		if (!g_search.have_face) { sol.landing_err = 1e9f; return false; }
		const int K = sol.nsplits + 1;
		const int ncols = (K >= 2) ? 2 : 1;
		const int cols[2] = { (K >= 2) ? K - 2 : K - 1, K - 1 };
		float lo[4], hi[4];
		for (int i = 0; i < K; ++i) {
			const int side = (i % 2 == 0) ? sol.side0 : -sol.side0;
			if (side > 0) { lo[i] = -8.f; hi[i] = 0.f; } else { lo[i] = 0.f; hi[i] = 8.f; }
		}
		float r[4];
		memcpy(r, sol.rates, sizeof(r));

		const Vector tgt = g_search.target_pos;
		ContactHit hit;
		auto resid = [&](const float* rr, float* F) -> bool {
			hit = SimToContact(sol, rr);
			if (!hit.ok)
				return false;
			const Vector d = hit.c - tgt;
			F[0] = Dot(g_search.e1, d);
			F[1] = Dot(g_search.e2, d);
			return true;
		};

		float F[2];
		if (!resid(r, F)) { sol.landing_err = 1e9f; return false; }
		float err = sqrtf(F[0] * F[0] + F[1] * F[1]);
		ContactHit best_hit = hit;
		float best_err = err;
		float best_r[4];
		memcpy(best_r, r, sizeof(best_r));
		float lambda = 0.05f;
		const float h = 0.05f;

		for (int iter = 0; iter < 28 && err > 0.10f; ++iter) {
			// FD Jacobian over the refined columns.
			float J[2][2] = {};
			bool jac_ok = true;
			for (int jc = 0; jc < ncols; ++jc) {
				const int j = cols[jc];
				float rj[4];
				memcpy(rj, r, sizeof(rj));
				const float hs = (rj[j] + h <= hi[j]) ? h : -h;
				rj[j] = Clampf(rj[j] + hs, lo[j], hi[j]);
				float Fj[2];
				if (!resid(rj, Fj)) { jac_ok = false; break; }
				J[0][jc] = (Fj[0] - F[0]) / hs;
				J[1][jc] = (Fj[1] - F[1]) / hs;
			}
			if (!jac_ok)
				break;

			bool stepped = false;
			for (int attempt = 0; attempt < 5 && !stepped; ++attempt) {
				float dr[2] = {};
				if (ncols == 1) {
					const float a = J[0][0] * J[0][0] + J[1][0] * J[1][0] + lambda;
					dr[0] = -(J[0][0] * F[0] + J[1][0] * F[1]) / a;
				} else {
					const float a00 = J[0][0] * J[0][0] + J[1][0] * J[1][0] + lambda;
					const float a11 = J[0][1] * J[0][1] + J[1][1] * J[1][1] + lambda;
					const float a01 = J[0][0] * J[0][1] + J[1][0] * J[1][1];
					const float b0 = -(J[0][0] * F[0] + J[1][0] * F[1]);
					const float b1 = -(J[0][1] * F[0] + J[1][1] * F[1]);
					const float det = a00 * a11 - a01 * a01;
					if (fabsf(det) < 1e-12f) { lambda *= 4.f; continue; }
					dr[0] = (b0 * a11 - b1 * a01) / det;
					dr[1] = (b1 * a00 - b0 * a01) / det;
				}
				const float m = fmaxf(fabsf(dr[0]), fabsf(dr[1]));
				if (m > 1.5f) { dr[0] *= 1.5f / m; dr[1] *= 1.5f / m; }
				float rn[4];
				memcpy(rn, r, sizeof(rn));
				for (int jc = 0; jc < ncols; ++jc)
					rn[cols[jc]] = Clampf(rn[cols[jc]] + dr[jc], lo[cols[jc]], hi[cols[jc]]);
				float Fn[2];
				if (resid(rn, Fn)) {
					const float en = sqrtf(Fn[0] * Fn[0] + Fn[1] * Fn[1]);
					if (en < err) {
						memcpy(r, rn, sizeof(r));
						F[0] = Fn[0]; F[1] = Fn[1]; err = en;
						if (en < best_err) {
							best_err = en;
							best_hit = hit;
							memcpy(best_r, r, sizeof(best_r));
						}
						lambda = fmaxf(lambda * 0.4f, 1e-4f);
						stepped = true;
					}
				}
				if (!stepped)
					lambda *= 4.f;
			}
			if (!stepped)
				break;   // fresh Jacobian wouldn't differ; the valley floor is here
		}

		for (int i = 0; i < 4; ++i) sol.rates[i] = (i < K) ? best_r[i] : 0.f;
		FillFromContact(sol, best_hit, best_err);
		return best_err <= g_search.exact_tol;
	}

	// Ranking: exact landing is already a hard gate on every entry, so score by
	// what happens NEXT: post-board speed (coupled) or arrival speed, minus a
	// soft preference for boards whose arrival heading follows the target's set
	// yaw (a 180-degree reversal board stays in the space but must be
	// meaningfully faster to outrank an aligned one).
	float RankScore(const RouteSolution& s) {
		const float base = g_search.couple ? s.post_speed : s.arrive_speed;
		return base - g_search.head_pref * s.head_err;
	}

	// Squeeze more post-board (or arrival) speed out of a landed solution by
	// walking the FREE rates (all but the last two) - the landing is re-solved
	// by Newton after every trial, so position stays exact. K<=2 has no free
	// rates; its variety comes from the (ticks, side, timing) enumeration.
	void OuterOptimize(RouteSolution& sol) {
		const int K = sol.nsplits + 1;
		if (K < 3)
			return;
		float lo[4], hi[4];
		for (int i = 0; i < K; ++i) {
			const int side = (i % 2 == 0) ? sol.side0 : -sol.side0;
			if (side > 0) { lo[i] = -8.f; hi[i] = 0.f; } else { lo[i] = 0.f; hi[i] = 8.f; }
		}
		float step = 0.6f;
		for (int round = 0; round < 12 && step >= 0.04f; ++round) {
			bool improved = false;
			for (int i = 0; i + 2 < K; ++i) {
				for (int dir = -1; dir <= 1; dir += 2) {
					RouteSolution trial = sol;
					trial.rates[i] = Clampf(trial.rates[i] + dir * step, lo[i], hi[i]);
					if (trial.rates[i] == sol.rates[i])
						continue;
					if (!RefineNewton(trial))
						continue;
					if (g_search.couple && (!trial.retained || trial.loss > g_search.max_loss))
						continue;   // never trade into a board that fails the gates
					if (RankScore(trial) > RankScore(sol) + 0.01f) {
						sol = trial;
						improved = true;
					}
				}
			}
			if (!improved)
				step *= 0.55f;
		}
	}

	// While airborne there is no vertical control, so arrival height depends
	// only on tick count - this closed form (matching FastTick's half-gravity)
	// prunes the tick search to a narrow feasible band.
	void BuildTicksBand() {
		g_search.ticks_band.clear();
		const float z0 = g_search.start_pos.Z;
		const float vz0 = g_search.start_vel.Z;
		const float dt = g_search.dt;
		const float grav = g_search.gravity;
		for (int t = 2; t <= 300; ++t) {
			const float z = z0 + vz0 * dt * t - 0.5f * grav * dt * dt * static_cast<float>(t) * static_cast<float>(t);
			if (fabsf(z - g_search.target_pos.Z) < 24.f)
				g_search.ticks_band.push_back(t);
		}
	}

	void BuildSplits(int ticks) {
		g_search.splits_list.clear();
		const int K = g_search.strafes;
		const int mn = 4;
		if (K == 1) {
			g_search.splits_list.push_back({ 0, 0, 0 });
		} else if (K == 2) {
			const int stride = (ticks > 60) ? 2 : 1;
			for (int a = mn; a <= ticks - mn; a += stride)
				g_search.splits_list.push_back({ a, 0, 0 });
		} else if (K == 3) {
			const int stride = (ticks > 80) ? 4 : 2;
			for (int a = mn; a < ticks - 2 * mn; a += stride)
				for (int b = a + mn; b <= ticks - mn; b += stride)
					g_search.splits_list.push_back({ a, b, 0 });
		} else {
			const int stride = (ticks > 60) ? 6 : 4;
			for (int a = mn; a < ticks - 3 * mn; a += stride)
				for (int b = a + mn; b < ticks - 2 * mn; b += stride)
					for (int c = b + mn; c <= ticks - mn; c += stride)
						g_search.splits_list.push_back({ a, b, c });
		}
		g_search.splits_built_for = ticks;
		g_search.job_split_idx = 0;
	}

	void ApplySolution(int seg_index, const RouteSolution& sol) {
		if (seg_index < 0 || seg_index >= static_cast<int>(g_segs.size()))
			return;
		SolverData& sd = g_segs[seg_index].solver;
		sd.solved = true;
		sd.ticks = sol.ticks;
		sd.side0 = sol.side0;
		sd.nsplits = sol.nsplits;
		memcpy(sd.splits, sol.splits, sizeof(sd.splits));
		memcpy(sd.rates, sol.rates, sizeof(sd.rates));
		sd.model_contact = sol.contact_pos;
		sd.model_land_err = sol.landing_err;
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

	void StartSearch(int seg_index) {
		g_search = SearchCtx();
		g_solution_pick = 0;
		g_polish.active = false;   // a new search supersedes any in-flight polish
		g_polish.done = false;
		g_polish.note[0] = 0;
		if (seg_index < 0 || seg_index >= static_cast<int>(g_segs.size()) || !g_segs[seg_index].is_solver)
			return;
		g_search.seg = seg_index;

		const BspWorld::BoardTarget* target = BspWorld::GetTarget(g_segs[seg_index].solver.target);
		if (!target) {
			g_status = "Solver: select a board target first.";
			g_search.done = true;
			return;
		}
		if (!SolverStartState(seg_index, g_search.start_pos, g_search.start_vel, g_search.start_yaw)) {
			g_status = "Solver: the prefix sim isn't current (or no anchor) - wait for the resim.";
			g_search.done = true;
			return;
		}

		g_search.target_pos = target->pos;
		g_search.target_yaw = target->yaw;
		g_search.tbrush = target->brush;
		g_search.tplane = target->plane;
		Vector n;
		float d = 0.f;
		g_search.have_face = BspWorld::GetPlane(target->plane, &n, &d);
		g_search.face_n = n;
		g_search.face_d = d;
		if (!g_search.have_face) {
			// Exact landing is defined as the mid-tick crossing of the target
			// face's support plane - no plane, nothing to solve against.
			g_status = "Solver: the target's face plane isn't available (stale geometry?) - re-place the target.";
			g_search.done = true;
			return;
		}
		PlaneBasis(g_search.face_n, g_search.e1, g_search.e2);

		g_search.strafes = g_segs[seg_index].solver.strafes;
		if (g_search.strafes < 1) g_search.strafes = 1;
		if (g_search.strafes > 4) g_search.strafes = 4;

		float interval = Prediction::LastDiag().interval_per_tick;
		if (interval <= 0.f) interval = 0.015f;
		g_search.dt = interval;
		g_search.cap = g_air_cap;
		g_search.accel_amt = g_air_accel * g_wishspeed * interval;
		g_search.gravity = g_gravity;
		g_search.exact_tol = g_sol_pos_tol;   // hard landing gate (default 2u)
		g_search.max_loss = g_sol_max_loss;
		g_search.head_pref = g_sol_head_pref;

		// Coupling only decides ranking (best board first) and the outer
		// optimizer's objective. Position is exact in BOTH modes.
		g_search.couple = g_segs[seg_index].solver.couple && g_search.have_face;

		// Grounded start: jump on the first tick (the fast model is airborne-
		// only, so a flat start otherwise collapses the tick band instantly).
		if (g_segs[seg_index].solver.start_jump) {
			// The engine runs one tick of ground friction before the jump
			// impulse (Friction -> CheckJumpButton -> AirMove, same tick);
			// mirror it so the model doesn't run hot by ~15-20 u/s.
			const float sp = sqrtf(g_search.start_vel.X * g_search.start_vel.X
				+ g_search.start_vel.Y * g_search.start_vel.Y);
			if (sp > 0.1f) {
				const float control = (sp < 100.f) ? 100.f : sp;   // sv_stopspeed (only matters below it)
				float keep = sp - control * 4.f * g_search.dt;     // sv_friction 4
				if (keep < 0.f) keep = 0.f;
				g_search.start_vel.X *= keep / sp;
				g_search.start_vel.Y *= keep / sp;
			}
			g_search.start_vel.Z += sqrtf(2.f * g_search.gravity * 57.f);   // ~302 at 800
		}

		BuildTicksBand();
		if (g_search.ticks_band.empty()) {
			g_search.done = true;
			g_status = "Solver: NO tick count reaches the target's height - unreachable under gravity from this start.";
			return;
		}
		g_search.jobs_total = 0;
		for (int t : g_search.ticks_band) {
			BuildSplits(t);
			g_search.jobs_total += 2 * static_cast<int>(g_search.splits_list.size());
		}
		g_search.splits_built_for = -1;
		g_search.job_split_idx = 0;
		g_search.active = true;
		g_status = "Solver: searching...";
	}

	void SortResults(std::vector<RouteSolution>& v) {
		const bool couple = g_search.couple;
		std::sort(v.begin(), v.end(),
			[couple](const RouteSolution& a, const RouteSolution& b) {
				if (couple && a.retained != b.retained)
					return a.retained;
				return RankScore(a) > RankScore(b);
			});
	}

	// Neighboring (ticks, timing) candidates converge onto the same route; keep
	// the solution space slider meaningful by collapsing near-identical ones.
	void DedupeResults(std::vector<RouteSolution>& v) {
		std::vector<RouteSolution> kept;
		kept.reserve(v.size());
		for (const RouteSolution& s : v) {
			bool dup = false;
			for (const RouteSolution& k : kept) {
				if (k.side0 != s.side0 || k.nsplits != s.nsplits || abs(k.ticks - s.ticks) > 1)
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

	void StartPolish(int seg_index);   // fwd (real-sim landing polish, below)

	void FinishSearch() {
		// Rank, collapse duplicates, then spend the expensive post-board
		// optimization only on the head of the list (it re-solves the exact
		// landing after every trial - too costly for every candidate).
		SortResults(g_search.results);
		DedupeResults(g_search.results);
		const int top = (std::min)(static_cast<int>(g_search.results.size()), 16);
		for (int i = 0; i < top; ++i)
			OuterOptimize(g_search.results[i]);
		SortResults(g_search.results);
		g_search.done = true;
		g_search.active = false;
		g_solution_pick = 0;
		if (!g_search.results.empty()) {
			ApplySolution(g_search.seg, g_search.results[0]);
			if (g_polish.auto_run)
				StartPolish(g_search.seg);
			g_status = "Solver: " + std::to_string(g_search.results.size())
				+ " exact landings, best board/speed first. Slider walks them.";
		} else {
			g_status = "Solver: search complete - NO exact landings with this strafe count.";
		}
	}

	// Time-sliced search pump (~5 ms per frame from Update()).
	void StepSearch() {
		if (!g_search.active || g_search.done)
			return;
		const ULONGLONG t0 = GetTickCount64();
		while (GetTickCount64() - t0 < 5) {
			if (g_search.job_tick_idx >= static_cast<int>(g_search.ticks_band.size())) {
				FinishSearch();
				return;
			}
			const int ticks = g_search.ticks_band[g_search.job_tick_idx];
			if (g_search.splits_built_for != ticks)
				BuildSplits(ticks);
			if (g_search.job_split_idx >= static_cast<int>(g_search.splits_list.size())) {
				g_search.job_split_idx = 0;
				g_search.job_side++;
				if (g_search.job_side > 1) {
					g_search.job_side = 0;
					g_search.job_tick_idx++;
					g_search.splits_built_for = -1;
				}
				continue;
			}
			const std::array<int, 3>& splits = g_search.splits_list[g_search.job_split_idx++];
			g_search.jobs_done++;

			RouteSolution sol;
			if (SolveRates(ticks, g_search.job_side == 0 ? 1 : -1, splits.data(),
			               g_search.strafes - 1, sol)) {
				// Coarse got into the basin; Newton drives the contact onto the
				// EXACT target coordinate.
				if (RefineNewton(sol)) {
					// Board quality is part of "is a solution" when coupled: a
					// board that bounces off or eats too much speed is rejected,
					// not ranked last.
					if (g_search.couple && (!sol.retained || sol.loss > g_search.max_loss)) {
						g_search.reject_board_n++;
						if (sol.loss < g_search.reject_board_best)
							g_search.reject_board_best = sol.loss;
					} else if (static_cast<int>(g_search.results.size()) < 400) {
						g_search.results.push_back(sol);
					}
				} else if (sol.landing_err < g_search.reject_land) {
					g_search.have_reject = true;
					g_search.reject_land = sol.landing_err;
					g_search.reject_ticks = ticks;
				}
			} else if (sol.pos_err < g_search.reject_coarse) {
				// Didn't even get near - track the coarse tick-end miss.
				g_search.reject_coarse = sol.pos_err;
				g_search.reject_ticks = ticks;
			}
		}
	}

	// How many extra ticks past the plan the sim runs when it ends on a solved
	// solver segment: the real line then always REACHES the ramp (instead of
	// stopping mid-air on the model's predicted contact tick) and shows the
	// board result, and the landing analysis has slack for contact-tick drift.
	constexpr int kSolverOverrun = 16;

	int FindSolverSeg() {
		for (int k = static_cast<int>(g_segs.size()) - 1; k >= 0; --k)
			if (g_segs[k].is_solver && g_segs[k].solver.solved && !g_segs[k].raw)
				return k;
		return -1;
	}

	void AnalyzeRealLanding() {
		g_real = RealLanding();
		const int L = FindSolverSeg();
		if (L < 0 || L >= static_cast<int>(g_starts.size()) || g_states.empty())
			return;
		const SolverData& sd = g_segs[L].solver;
		const BspWorld::BoardTarget* target = BspWorld::GetTarget(sd.target);
		Vector n;
		float pd = 0.f;
		if (!target || !BspWorld::GetPlane(target->plane, &n, &pd))
			return;
		const Vector tgt = target->pos;
		const float d_origin = Dot(n, tgt);
		float interval = Prediction::LastDiag().interval_per_tick;
		if (interval <= 0.f) interval = 0.015f;

		g_real.computed = true;
		const int b = g_starts[L];
		const int count = static_cast<int>(g_states.size());
		std::vector<float> grav_samples;

		for (int i = b; i < count; ++i) {
			const Vector o = g_states[i].origin;
			const float phi = Dot(n, o) - d_origin;
			const Vector prev_o = (i > 0) ? g_states[i - 1].origin : g_anchor.origin;
			const Vector prev_v = (i > 0) ? g_states[i - 1].velocity : g_anchor.velocity;
			const float prev_phi = Dot(n, prev_o) - d_origin;

			if (!g_real.contacted && phi < 1.0f && prev_phi >= 1.0f) {
				// The engine collided (or entered the 1u shell) inside this
				// tick; recover the touch point by intersecting the incoming
				// straight segment with the plane. prev_v lacks this tick's
				// accel/half-gravity, so the recovered point is good to ~0.5u.
				const Vector step = Scale(prev_v, interval);
				const float pe = Dot(n, prev_o + step) - d_origin;
				float tau = 1.f;
				if (prev_phi - pe > 1e-6f)
					tau = Clampf(prev_phi / (prev_phi - pe), 0.f, 1.f);
				g_real.touch = prev_o + Scale(step, tau);
				g_real.err = sqrtf(Dot3(g_real.touch - tgt));
				g_real.tick = i;
				g_real.approach_normal = Dot(n, prev_v);
				g_real.before2d = Speed2D(prev_v);
				g_real.after2d = Speed2D(g_states[i].velocity);
				g_real.on_face = BspWorld::PointOnFace(target->brush, target->plane, g_real.touch, 4.f);
				// Drift needs a stored model contact (v8 projects / fresh solves).
				g_real.drift = (Dot3(sd.model_contact) > 1e-6f)
					? sqrtf(Dot3(g_real.touch - sd.model_contact)) : -1.f;
				g_real.contacted = true;
			}
			if (!g_real.contacted && phi < g_real.closest_phi)
				g_real.closest_phi = phi;

			// Engine-constant fits from fully-airborne pre-contact ticks: these
			// name the drifting constant when the model and the engine disagree.
			const bool air_prev = (i > b) && ((g_states[i - 1].flags & FL_ONGROUND) == 0);
			const bool air_now = (g_states[i].flags & FL_ONGROUND) == 0;
			if (air_prev && air_now && !g_real.contacted) {
				grav_samples.push_back((g_states[i - 1].velocity.Z - g_states[i].velocity.Z) / interval);
				if (i < static_cast<int>(g_frames.size())) {
					const float smove = g_frames[i].sidemove;
					if (fabsf(smove) > 10.f) {
						const int side = (smove < 0.f) ? 1 : -1;   // -sidemove = A = +90
						const float wr = (g_frames[i].viewangles[1] + side * 90.f) * (kPi / 180.f);
						const float wx = cosf(wr), wy = sinf(wr);
						const Vector pv = g_states[i - 1].velocity;
						const float cur = pv.X * wx + pv.Y * wy;
						const float add = (g_states[i].velocity.X - pv.X) * wx
							+ (g_states[i].velocity.Y - pv.Y) * wy;
						if (add > 0.5f && cur + add > g_real.cap_fit)
							g_real.cap_fit = cur + add;
					}
				}
			}
		}
		if (!grav_samples.empty()) {
			std::nth_element(grav_samples.begin(),
				grav_samples.begin() + grav_samples.size() / 2, grav_samples.end());
			g_real.grav_fit = grav_samples[grav_samples.size() / 2];
			g_real.fit_samples = static_cast<int>(grav_samples.size());
		}
	}

	void RequestPolishEval(const float* rates) {
		SolverData& sd = g_segs[g_polish.seg].solver;
		memcpy(sd.rates, rates, sizeof(float) * 4);
		memcpy(g_polish.trial, rates, sizeof(float) * 4);
		MarkDirty();   // the normal Update flow resims; the next landed sim = this eval
	}

	void FinishPolish() {
		g_polish.active = false;
		g_polish.done = true;
		if (g_polish.best_err < 1e8f) {
			if (g_polish.seg >= 0 && g_polish.seg < static_cast<int>(g_segs.size())
				&& g_segs[g_polish.seg].is_solver) {
				SolverData& sd = g_segs[g_polish.seg].solver;
				memcpy(sd.rates, g_polish.best_rates, sizeof(float) * 4);
				MarkDirty();   // final verification sim refreshes the REAL readout
			}
			sprintf_s(g_polish.note, "polish: REAL landing driven to %.2f u from target (%d sims, %d steps).",
				g_polish.best_err, g_polish.evals, g_polish.iter);
		} else {
			sprintf_s(g_polish.note, "polish: the real line never reaches the face plane "
				"(closest %.0f u). Model drift too large or the route hits geometry - "
				"check the engine-fit readout and the strafe model settings.",
				g_real.closest_phi < 1e8f ? g_real.closest_phi : 0.f);
		}
	}

	void PolishComputeStep() {
		// LM step from the stored Jacobian/residual at the current lambda.
		float dr[2] = {};
		if (g_polish.ncols == 1) {
			const float a = g_polish.J[0][0] * g_polish.J[0][0]
				+ g_polish.J[1][0] * g_polish.J[1][0] + g_polish.lambda;
			dr[0] = -(g_polish.J[0][0] * g_polish.F[0] + g_polish.J[1][0] * g_polish.F[1]) / a;
		} else {
			const float a00 = g_polish.J[0][0] * g_polish.J[0][0] + g_polish.J[1][0] * g_polish.J[1][0] + g_polish.lambda;
			const float a11 = g_polish.J[0][1] * g_polish.J[0][1] + g_polish.J[1][1] * g_polish.J[1][1] + g_polish.lambda;
			const float a01 = g_polish.J[0][0] * g_polish.J[0][1] + g_polish.J[1][0] * g_polish.J[1][1];
			const float b0 = -(g_polish.J[0][0] * g_polish.F[0] + g_polish.J[1][0] * g_polish.F[1]);
			const float b1 = -(g_polish.J[0][1] * g_polish.F[0] + g_polish.J[1][1] * g_polish.F[1]);
			const float det = a00 * a11 - a01 * a01;
			if (fabsf(det) < 1e-12f) { FinishPolish(); return; }
			dr[0] = (b0 * a11 - b1 * a01) / det;
			dr[1] = (b1 * a00 - b0 * a01) / det;
		}
		const float m = fmaxf(fabsf(dr[0]), fabsf(dr[1]));
		if (m > 1.0f) { dr[0] /= m; dr[1] /= m; }
		float rn[4];
		memcpy(rn, g_polish.center, sizeof(rn));
		for (int jc = 0; jc < g_polish.ncols; ++jc) {
			const int j = g_polish.cols[jc];
			rn[j] = Clampf(rn[j] + dr[jc], g_polish.lo[j], g_polish.hi[j]);
		}
		g_polish.phase = 3;
		RequestPolishEval(rn);
	}

	void PolishBeginJacobianCol(int jc) {
		const int j = g_polish.cols[jc];
		float rj[4];
		memcpy(rj, g_polish.center, sizeof(rj));
		const float h = 0.06f;
		g_polish.hs[jc] = (rj[j] + h <= g_polish.hi[j]) ? h : -h;
		rj[j] = Clampf(rj[j] + g_polish.hs[jc], g_polish.lo[j], g_polish.hi[j]);
		g_polish.phase = 1 + jc;
		RequestPolishEval(rj);
	}

	void StartPolish(int seg_index) {
		const bool keep_auto = g_polish.auto_run;
		g_polish = PolishCtx();
		g_polish.auto_run = keep_auto;
		if (seg_index < 0 || seg_index >= static_cast<int>(g_segs.size()))
			return;
		EditSegment& seg = g_segs[seg_index];
		if (!seg.is_solver || !seg.solver.solved)
			return;
		const BspWorld::BoardTarget* target = BspWorld::GetTarget(seg.solver.target);
		Vector n;
		float pd = 0.f;
		if (!target || !BspWorld::GetPlane(target->plane, &n, &pd))
			return;
		g_polish.seg = seg_index;
		g_polish.tgt = target->pos;
		PlaneBasis(n, g_polish.e1, g_polish.e2);
		g_polish.K = seg.solver.nsplits + 1;
		g_polish.ncols = (g_polish.K >= 2) ? 2 : 1;
		g_polish.cols[0] = (g_polish.K >= 2) ? g_polish.K - 2 : g_polish.K - 1;
		g_polish.cols[1] = g_polish.K - 1;
		for (int i = 0; i < g_polish.K; ++i) {
			const int side = (i % 2 == 0) ? seg.solver.side0 : -seg.solver.side0;
			if (side > 0) { g_polish.lo[i] = -8.f; g_polish.hi[i] = 0.f; }
			else          { g_polish.lo[i] = 0.f;  g_polish.hi[i] = 8.f; }
		}
		g_polish.active = true;
		g_polish.phase = 0;
		g_polish.skip = Prediction::SimBusy() ? 1 : 0;   // an in-flight sim has stale rates
		RequestPolishEval(seg.solver.rates);
	}

	// One landed sim = one evaluation. Consumes g_real (already refreshed for
	// this sim) and advances the LM state machine. User edits mid-polish just
	// perturb one evaluation; the accept/reject step absorbs the noise.
	void PolishPump() {
		if (!g_polish.active)
			return;
		if (g_polish.seg < 0 || g_polish.seg >= static_cast<int>(g_segs.size())
			|| !g_segs[g_polish.seg].is_solver || !g_segs[g_polish.seg].solver.solved) {
			g_polish.active = false;   // segment vanished under us; stop quietly
			return;
		}
		if (g_polish.skip > 0) {       // stale sim from before the polish started
			g_polish.skip--;
			return;
		}
		g_polish.evals++;
		float Fe[2] = {};
		float err_e = 1e9f;
		const bool ok = g_real.computed && g_real.contacted;
		if (ok) {
			const Vector d = g_real.touch - g_polish.tgt;
			Fe[0] = Dot(g_polish.e1, d);
			Fe[1] = Dot(g_polish.e2, d);
			err_e = sqrtf(Fe[0] * Fe[0] + Fe[1] * Fe[1]);
			if (err_e < g_polish.best_err) {
				g_polish.best_err = err_e;
				memcpy(g_polish.best_rates, g_polish.trial, sizeof(float) * 4);
			}
		}

		const bool budget_out = (g_polish.evals >= 70) || (g_polish.iter >= 14);
		switch (g_polish.phase) {
		case 0:   // center evaluation
			if (!ok) { FinishPolish(); return; }
			memcpy(g_polish.center, g_polish.trial, sizeof(float) * 4);
			g_polish.F[0] = Fe[0]; g_polish.F[1] = Fe[1];
			g_polish.err = err_e;
			if (err_e <= 0.5f || budget_out) { FinishPolish(); return; }
			PolishBeginJacobianCol(0);
			return;
		case 1:   // jacobian column 0
			if (!ok) { FinishPolish(); return; }
			g_polish.J[0][0] = (Fe[0] - g_polish.F[0]) / g_polish.hs[0];
			g_polish.J[1][0] = (Fe[1] - g_polish.F[1]) / g_polish.hs[0];
			if (g_polish.ncols == 2) PolishBeginJacobianCol(1);
			else PolishComputeStep();
			return;
		case 2:   // jacobian column 1
			if (!ok) { FinishPolish(); return; }
			g_polish.J[0][1] = (Fe[0] - g_polish.F[0]) / g_polish.hs[1];
			g_polish.J[1][1] = (Fe[1] - g_polish.F[1]) / g_polish.hs[1];
			PolishComputeStep();
			return;
		default:  // step evaluation
			if (ok && err_e < g_polish.err) {
				memcpy(g_polish.center, g_polish.trial, sizeof(float) * 4);
				g_polish.F[0] = Fe[0]; g_polish.F[1] = Fe[1];
				g_polish.err = err_e;
				g_polish.lambda = fmaxf(g_polish.lambda * 0.45f, 1e-4f);
				g_polish.iter++;
				if (err_e <= 0.5f || budget_out) { FinishPolish(); return; }
				PolishBeginJacobianCol(0);
			} else {
				g_polish.lambda *= 4.f;
				if (g_polish.lambda > 3e4f || budget_out) { FinishPolish(); return; }
				PolishComputeStep();   // same Jacobian, heavier damping
			}
			return;
		}
	}

	bool TargetItemGetter(void*, int index, const char** out_text) {
		static char buffer[64];
		const BspWorld::BoardTarget* t = BspWorld::GetTarget(index);
		if (!t)
			return false;
		sprintf_s(buffer, "#%d  %.0f %.0f %.0f  yaw %.0f", index, t->pos.X, t->pos.Y, t->pos.Z, t->yaw);
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
	// v7: solver gains couple + start_jump. v8: solver appends the model's
	// predicted contact point + landing err (real-vs-model drift readout).
	constexpr uint32_t kProjVersion = 8;

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
					const uint8_t couple = sd.couple ? 1 : 0;
					const uint8_t sjump = sd.start_jump ? 1 : 0;
					W(out, sd.target); W(out, sd.strafes); W(out, solved);
					W(out, sd.ticks); W(out, sd.side0); W(out, sd.nsplits);
					for (int i = 0; i < 3; ++i) W(out, sd.splits[i]);
					for (int i = 0; i < 4; ++i) W(out, sd.rates[i]);
					W(out, couple); W(out, sjump);
					W(out, sd.model_contact.X); W(out, sd.model_contact.Y); W(out, sd.model_contact.Z);
					W(out, sd.model_land_err);
				}
			}
		}

		g_status = out ? ("Saved " + base + ".tasproj.") : "Write failed.";
		RefreshFiles();
		return static_cast<bool>(out);
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
					if (version >= 7) {
						uint8_t couple = 1, sjump = 0;
						if (!R(in, couple) || !R(in, sjump)) return false;
						sd.couple = couple != 0;
						sd.start_jump = sjump != 0;
					}
					if (version >= 8) {
						if (!R(in, sd.model_contact.X) || !R(in, sd.model_contact.Y)
							|| !R(in, sd.model_contact.Z) || !R(in, sd.model_land_err))
							return false;
					}
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
		char cmd[160];
		sprintf_s(cmd, "setpos %.2f %.2f %.2f; setang %.2f %.2f 0",
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
			char cmd[160];
			sprintf_s(cmd, "setpos %.2f %.2f %.2f; setang %.2f %.2f 0",
				g_anchor.origin.X, g_anchor.origin.Y, g_anchor.origin.Z,
				g_anchor.pitch, g_anchor.yaw);
			engine->ClientCmd_Unrestricted(cmd);
			g_status = "Teleported (needs sv_cheats 1).";
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
			char kind[48];
			if (s.raw) {
				strcpy_s(kind, "RAW");
			} else if (s.is_solver) {
				sprintf_s(kind, "SOLVER ->#%d K%d%s", s.solver.target, s.solver.strafes,
					s.solver.solved ? "" : " (unsolved)");
			} else {
				const bool turning = s.gen.yaw_mode == 1 && fabsf(s.gen.yaw_rate) > 0.01f;
				sprintf_s(kind, "MOVE %s%s%s%s%s%s",
					s.gen.key_w ? "W" : "", s.gen.key_a ? "A" : "",
					s.gen.key_s ? "S" : "", s.gen.key_d ? "D" : "",
					(turning && s.gen.auto_key && !s.gen.key_a && !s.gen.key_d) ? "+auto" : "",
					turning ? " turn" : "");
			}
			char label[128];
			sprintf_s(label, "#%d  %s  %d ticks%s##seg%d", k, kind, s.Ticks(),
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
					"Solver segment: enumerates alternating-strafe routes (timings x "
					"tick counts, physics-banded by gravity), Newton-drives every "
					"candidate's mid-tick face contact onto the EXACT target "
					"coordinate (hard gate; heading stays free and settles optimal), "
					"then ranks by post-board speed. The applied solution runs "
					"through the REAL engine sim and its landing is re-measured and "
					"polished against that ground truth.");

				ImGui::PushItemWidth(260);
				int target = s.solver.target;
				if (ImGui::Combo("Board target", &target, TargetItemGetter, nullptr, BspWorld::TargetCount()))
					s.solver.target = target;
				int strafes = s.solver.strafes;
				if (ImGui::InputInt("Strafes (alternating)", &strafes))
					s.solver.strafes = strafes < 1 ? 1 : strafes > 4 ? 4 : strafes;
				ImGui::PopItemWidth();

				ImGui::Checkbox("Perfect-board coupling", &s.solver.couple);
				ImGui::SameLine();
				ImGui::TextDisabled("(off for floors/pads/waypoints; on = heading goes soft, board decides)");
				ImGui::Checkbox("Jump at start (grounded start)", &s.solver.start_jump);
				ImGui::SameLine();
				ImGui::TextDisabled("(the route model is airborne-only - flat starts need this)");

				ImGui::PushItemWidth(150);
				ImGui::InputFloat("Exact landing tol (u)", &g_sol_pos_tol, 0.25f, 1.f, 2);
				ImGui::InputFloat("Max board loss (u/s)", &g_sol_max_loss, 5.f, 25.f, 0);
				ImGui::InputFloat("Target-yaw preference (u/s per deg)", &g_sol_head_pref, 0.05f, 0.25f, 2);
				ImGui::PopItemWidth();
				ImGui::TextDisabled("Position is exact (hard gate). Heading settles optimal - never gated; the yaw "
					"preference only orders the list (0 = pure speed, higher = follow the set yaw).");

				if (ImGui::Button("Search solutions"))
					StartSearch(g_sel);

				if (g_search.seg == g_sel) {
					if (g_search.active) {
						ImGui::Text("searching...  %d%%   (%d found so far)",
							g_search.jobs_total ? (g_search.jobs_done * 100 / g_search.jobs_total) : 0,
							static_cast<int>(g_search.results.size()));
					} else if (g_search.done && g_search.results.empty()) {
						ImGui::PushTextWrapPos(0.f);
						if (g_search.ticks_band.empty()) {
							ImGui::TextColored(ImVec4(1.f, 0.45f, 0.45f, 1.f),
								"NO feasible solutions: no tick count reaches the target's HEIGHT "
								"under gravity from this start. If the start is on the ground, "
								"enable 'Jump at start'; otherwise the target needs a different "
								"approach (more speed or a closer/lower target).");
						} else if (g_search.reject_board_n > 0) {
							ImGui::TextColored(ImVec4(1.f, 0.45f, 0.45f, 1.f),
								"%d exact landing(s) found, but every board failed the quality gate "
								"(best lost %.0f u/s; max allowed %.0f). The approach angles this "
								"strafe count can reach all slam the ramp - try more strafes, more "
								"approach speed, a target lower on the ramp, or raise 'Max board loss'.",
								g_search.reject_board_n, g_search.reject_board_best, g_sol_max_loss);
						} else if (g_search.have_reject) {
							ImGui::TextColored(ImVec4(1.f, 0.45f, 0.45f, 1.f),
								"NO exact landing with %d strafe(s). Closest contact came within "
								"%.1f u of the target (need <= %.2f u). Add a strafe for more "
								"steering freedom, or loosen 'Exact landing tol' slightly.",
								g_search.strafes, g_search.reject_land, g_sol_pos_tol);
							if (g_search.strafes == 1)
								ImGui::TextDisabled("(1 strafe = 1 control for a 2D landing - an exact "
									"hit is a coincidence; 2+ strafes solve it structurally)");
						} else if (g_search.reject_coarse < 1e8f) {
							ImGui::TextColored(ImVec4(1.f, 0.45f, 0.45f, 1.f),
								"NO feasible solutions with %d strafe(s): the route can't even get "
								"near the target (closest tick-end was %.0f u away at %d ticks). "
								"Not enough speed/airtime for the distance - more speed into the "
								"segment, a closer target, or 'Jump at start' from the ground.",
								g_search.strafes, g_search.reject_coarse, g_search.reject_ticks);
						} else {
							ImGui::TextColored(ImVec4(1.f, 0.45f, 0.45f, 1.f),
								"NO feasible solutions: the tick band produced no viable strafe "
								"timings (band too small for %d strafes).", g_search.strafes);
						}
						ImGui::PopTextWrapPos();
					} else if (g_search.done) {
						const int count = static_cast<int>(g_search.results.size());
						int pick = g_solution_pick;
						if (IntRow("Solution (0 = best)", &pick, 0, count - 1) && pick != g_solution_pick) {
							g_solution_pick = pick;
							ApplySolution(g_sel, g_search.results[pick]);
							if (g_polish.auto_run)
								StartPolish(g_sel);
						}
						if (g_solution_pick >= count)
							g_solution_pick = 0;
						const RouteSolution& sol = g_search.results[g_solution_pick];
						ImGui::Text("model: %d ticks   arrive %.0f u/s   landing %.3f u   heading %+.1f deg vs target yaw",
							sol.ticks, sol.arrive_speed, sol.landing_err, sol.head_err);

						char breakdown[256] = {};
						int prev = 0;
						for (int k = 0; k <= sol.nsplits; ++k) {
							const int end = (k < sol.nsplits) ? sol.splits[k] : sol.ticks;
							const int side = (k % 2 == 0) ? sol.side0 : -sol.side0;
							char part[64];
							sprintf_s(part, "%s%+.2f deg/t x %dt (%s)", k ? "  |  " : "",
								sol.rates[k], end - prev, side > 0 ? "A" : "D");
							strcat_s(breakdown, part);
							prev = end;
						}
						ImGui::TextWrapped("%s", breakdown);

						if (sol.contact) {
							if (sol.perfect)
								ImGui::TextColored(ImVec4(0.4f, 1.f, 0.55f, 1.f),
									"model board: PERFECT   loss %.3f u/s   post-board %.0f u/s   contact retained",
									sol.loss, sol.post_speed);
							else
								ImGui::TextColored(ImVec4(1.f, 0.7f, 0.3f, 1.f),
									"model board: loss %.2f u/s   retained %s   post %.0f u/s",
									sol.loss, sol.retained ? "yes" : "no", sol.post_speed);
						} else {
							ImGui::TextColored(ImVec4(1.f, 0.7f, 0.3f, 1.f),
								"no board contact near arrival (face/yaw mismatch?)");
						}

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

				// --- ground truth: what the DRAWN line actually does -----------
				// The model numbers above describe the fast search model; the line
				// in the world is the real engine sim. This is its landing.
				if (s.solver.solved) {
					ImGui::Separator();
					ImGui::Checkbox("Auto-polish landing on the REAL sim", &g_polish.auto_run);
					ImGui::SameLine();
					if (ImGui::Button("Polish now"))
						StartPolish(g_sel);
					if (g_polish.active)
						ImGui::Text("polishing on the real sim...  step %d  sim %d  err %.2f u",
							g_polish.iter, g_polish.evals, g_polish.err < 1e8f ? g_polish.err : 0.f);
					else if (g_polish.done && g_polish.note[0])
						ImGui::TextWrapped("%s", g_polish.note);
					if (g_real.computed) {
						if (g_real.contacted) {
							char drift[48] = {};
							if (g_real.drift >= 0.f)
								sprintf_s(drift, "   drift vs model %.1f u", g_real.drift);
							ImGui::TextColored(g_real.err <= g_sol_pos_tol
									? ImVec4(0.4f, 1.f, 0.55f, 1.f) : ImVec4(1.f, 0.7f, 0.3f, 1.f),
								"REAL line: touched %.2f u from target (tick %d%s)%s",
								g_real.err, g_real.tick,
								g_real.on_face ? ", on target face" : ", OFF the target face",
								drift);
							ImGui::Text("   approach %.0f u/s into face   speed %.0f -> %.0f u/s (2D)",
								-g_real.approach_normal, g_real.before2d, g_real.after2d);
						} else {
							ImGui::TextColored(ImVec4(1.f, 0.45f, 0.45f, 1.f),
								"REAL line: never reaches the face plane - closest %.1f u. Model and "
								"engine disagree badly; compare the engine fit below to the strafe "
								"model settings.",
								g_real.closest_phi < 1e8f ? g_real.closest_phi : 0.f);
						}
						if (g_real.fit_samples >= 4)
							ImGui::TextDisabled(
								"engine fit: gravity ~%.0f (model %.0f)   air cap ~%.1f (model %.1f)   [%d air ticks]",
								g_real.grav_fit, g_gravity, g_real.cap_fit, g_air_cap, g_real.fit_samples);
					} else {
						ImGui::TextDisabled("REAL line: waiting for a sim with this segment last "
							"(needs the target's face plane).");
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
				ImGui::PushItemWidth(180);
				ch |= ImGui::Combo("View", &g.yaw_mode, "Hold\0Turn (deg/tick)\0");
				ImGui::PopItemWidth();
				if (g.yaw_mode == 1)
					ch |= FloatRow("View turn   (- left / + right)", &g.yaw_rate, -15.f, 15.f, "%+.2f deg/tick", 0.05f, 0.5f);
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
			g_dirty ? "edited - resim pending..." :
			g_sim_requested ? "simulating..." :
			g_sim_fault ? "SIM FAULTED (recovered - see prediction diagnostics)" :
			g_valid ? "sim up to date" : "no sim yet";
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
			char label[128];
			sprintf_s(label, "#%d  brush %d plane %d   slope %.0f deg   @ %.0f %.0f %.0f##tag%d",
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
			char label[128];
			sprintf_s(label, "#%d  %.0f %.0f %.0f   yaw %.1f%s##tgt%d",
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
	// Pump the solver search (time-sliced; no-op when idle).
	StepSearch();

	// Land a finished sim, remeasure the REAL landing, feed the polish loop.
	if (Prediction::SimReady()) {
		const int n = Prediction::SimCount();
		g_sim_fault = Prediction::SimFaulted();
		g_frames.resize(n);
		g_states.resize(n);
		Prediction::TakeSim(g_frames.data(), g_states.data());
		g_valid = n > 0;
		g_sim_requested = false;
		RecomputeDiag();
		AnalyzeRealLanding();
		PolishPump();
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
		sprintf_s(g_pick_info, "last pick: yaw %.1f pitch %.1f | eye %.0f %.0f %.0f | %s"
			" brush %d plane %d dist %.0f @ %.0f %.0f %.0f",
			s.yaw, s.pitch, eye.X, eye.Y, eye.Z, what,
			hit.brush, hit.plane, hit.dist, hit.point.X, hit.point.Y, hit.point.Z);
	}

	if (!hit.hit) {
		g_status = "Pick: nothing hit within 8192 units.";
		return;
	}

	char message[192];
	if (g_pick_mode == 0) {
		const bool tagged = BspWorld::ToggleFaceTag(hit.brush, hit.plane);
		sprintf_s(message, "%s face (brush %d, plane %d) at %.0f %.0f %.0f",
			tagged ? "Tagged" : "Untagged", hit.brush, hit.plane,
			hit.point.X, hit.point.Y, hit.point.Z);
	} else {
		const int index = BspWorld::AddTargetFromHit(hit, s.yaw, s.pitch, false);
		if (index >= 0) {
			g_sel_target = index;
			sprintf_s(message, "Board target #%d placed (surface normal %.2f %.2f %.2f)",
				index, hit.normal.X, hit.normal.Y, hit.normal.Z);
		} else {
			sprintf_s(message, "Couldn't place a target there.");
		}
	}
	g_status = message;
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
	out.have_board = g_real.computed && g_real.contacted;
	out.board_tick = g_real.tick;
	out.board_point = g_real.touch;
	return true;
}

void TasEditor::DrawWindow() {
	if (!g_open)
		return;

	BspWorld::highlight_target = g_sel_target;
	BspWorld::highlight_tag = g_sel_tag;

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
