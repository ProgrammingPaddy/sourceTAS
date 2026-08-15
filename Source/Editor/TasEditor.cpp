#include "TasEditor.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <exception>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <windows.h>
#include <shlobj.h>

#include <cstrike/sdk.h>
#include <cstrike/Interfaces/IEngineTrace.h>
#include <imgui/imgui.h>
#include <imgui/imgui_internal.h>   // window-stack rebalance after a UI fault

#include "../World/WorldDraw.h"
#include "../World/BspWorld.h"
#include "../World/Cvars.h"
#include "../Menu/Theme.h"
#include "../Menu/RecordPanel.h"
#include "../Menu/Breadcrumb.h"

#include <sstream>

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
	// Wide enough for a sign + 4 digits + 2 decimals NEXT TO the InputFloat's
	// two step buttons (which eat ~50px of the item width) - 110 clipped
	// "-889.50"-class values.
	constexpr float kInputW = 150.f;

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

	// ImGui 1.50 swallows the wheel over a NoScrollWithMouse child instead of
	// bubbling it up, so fixed-content boxes would dead-zone page scrolling.
	// Call straight after EndChild(): if the pointer is over that child, hand
	// the wheel to the window we are back inside (the scrolling tab body).
	void PassWheel() {
		ImGuiIO& io = ImGui::GetIO();
		if (io.MouseWheel != 0.f
			&& ImGui::IsMouseHoveringRect(ImGui::GetItemRectMin(), ImGui::GetItemRectMax()))
			ImGui::SetScrollY(ImGui::GetScrollY()
				- io.MouseWheel * ImGui::GetTextLineHeight() * 5.f);
	}

	// The scrollABLE-child variant: such a child consumes the wheel itself,
	// which dead-zones page scrolling even once the child has hit its end.
	// Call straight after EndChild() with the scroll values captured INSIDE
	// the child, plus a per-callsite `prev` holding LAST frame's scroll: this
	// frame's wheel already landed on the child before our code ran, so the
	// current position can't tell "was already at the end" from "this notch
	// just clamped into the end". Only the former passes to the outer window
	// - a notch that lands inside the child (even clamping into its edge)
	// belongs to the child alone, so one wheel step can never move both.
	void PassWheelAtEdge(float* prev, float scroll_y, float scroll_max) {
		ImGuiIO& io = ImGui::GetIO();
		const float was = *prev;    // where the child stood BEFORE this wheel
		*prev = scroll_y;
		if (io.MouseWheel == 0.f
			|| !ImGui::IsMouseHoveringRect(ImGui::GetItemRectMin(), ImGui::GetItemRectMax()))
			return;
		const bool was_top    = was <= 0.5f;
		const bool was_bottom = was >= scroll_max - 0.5f;
		if ((io.MouseWheel > 0.f && was_top) || (io.MouseWheel < 0.f && was_bottom))
			ImGui::SetScrollY(ImGui::GetScrollY()
				- io.MouseWheel * ImGui::GetTextLineHeight() * 5.f);
	}

	// Exclusive-choice row: one button per option, the selected one pink -
	// combo semantics (pick exactly one) without the extra click into a
	// dropdown, for the options that get flipped constantly. Only a CHANGE
	// returns true, so re-clicking the active option never re-dirties the
	// project. The label sits to the right like every other row.
	bool ChoiceRow(const char* label, int* v, const char* const* opts, int count) {
		bool changed = false;
		ImGui::PushID(label);
		for (int i = 0; i < count; ++i) {
			if (i) ImGui::SameLine();
			ImGui::PushID(i);
			if (Theme::Tab(opts[i], *v == i, ImVec2(0, 0)) && *v != i) {
				*v = i;
				changed = true;
			}
			ImGui::PopID();
		}
		if (label[0] && !(label[0] == '#' && label[1] == '#')) {
			ImGui::SameLine();
			ImGui::Text("%s", label);
		}
		ImGui::PopID();
		return changed;
	}

	// Sub-section divider: a muted label with a thin grey full-width underline,
	// the quieter sibling of Theme::Heading's pink accent line. Groups related
	// controls without boxing them (a full border doubled up on the child's
	// own frame and read as heavy).
	void SubHeading(const char* label) {
		ImGui::Spacing();
		ImGui::TextDisabled("%s", label);
		const ImVec2 p = ImGui::GetCursorScreenPos();
		const float w = ImGui::GetContentRegionAvailWidth();
		ImGui::GetWindowDrawList()->AddLine(
			ImVec2(p.x, p.y + 1.f), ImVec2(p.x + w, p.y + 1.f),
			ImGui::ColorConvertFloat4ToU32(ImVec4(0.55f, 0.55f, 0.62f, 0.30f)), 1.0f);
		ImGui::Dummy(ImVec2(0.f, 3.f));
	}

	// ---------------------------------------------------------------- model --
	// One segment type: held keyboard keys + a view that either holds or turns
	// at a fixed rate. While airborne and turning, the strafe key (A/D) is
	// auto-selected from the turn direction unless A or D is explicitly held.
	enum PitchMode { PM_Const = 0, PM_Follow, PM_Dynamic };
	enum JumpMode  { JM_None = 0, JM_Hold, JM_AutoBhop };

	struct GenParams {
		int   ticks = 66;
		bool  key_w = true, key_a = false, key_s = false, key_d = false;
		bool  auto_key = true;        // A/D from view turn direction (airborne)
		bool  no_w_air = true;        // release W while airborne (surf default)
		int   yaw_mode = 2;           // DEFAULT optimal. 1 = turn at yaw_rate (0 = hold);
		                              // 2 = OPTIMAL strafe: yaw follows the velocity
		                              //     heading (max gain), yaw_rate = bias deg
		                              // 3 = STRAIGHT SYNC: alternating optimal strafes
		                              //     every opt_period ticks; the net path holds
		                              //     its heading, yaw_rate = steer bias (view tilt)
		                              // 4 = STRAIGHT SKEW: alternating optimal strafes
		                              //     with ASYMMETRIC dwell (left ticks != right)
		                              //     so the mean path bends by dwell, view stays
		                              //     pure max-gain; yaw_rate = tick skew
		float yaw_rate = 0.f;         // deg/tick, screen sign (+ = right); mode 2/3: bias
		int   opt_dir = 1;            // mode 2 direction / mode 3 first side (+1 A, -1 D)
		int   opt_period = 8;         // mode 3: ticks per strafe side
		int   bias_kind = 2;          // mode 2 bias meaning: 0 = TOTAL heading change
		                              // over the segment (evenly distributed - LENGTH
		                              // DEPENDENT: resizing/splitting re-spreads the
		                              // turn and moves the drawn path); 1 = per-tick,
		                              // relative to the strafe side (legacy);
		                              // 2 = STEERING L/R (default): a fixed angle off
		                              // the max-gain line in screen sign, identical
		                              // every tick, so the path never depends on the
		                              // segment's tick count
		bool  yaw_abs = false;
		float yaw_start = 0.f;
		int   pitch_mode = PM_Dynamic;   // DYNAMIC (human recipe) is the default
		float pitch_val = 20.f;          // dynamic base / constant value, + = down
		float pitch_mult = 1.f;
		// DYNAMIC pitch recipe (v23), parameterized from the 7 recorded runs
		// (scratchpad pitch_analyze.py, 2026-08-12): humans surf looking DOWN
		// - straight-flight base ~+8..+27, another +20..+48 while carving
		// (pitch tracks |yaw rate|; dyaw~dpitch corr up to +0.70), deeper
		// when descending (watching the landing), motion rate-limited
		// (|dpitch| p90 ~1, p99 ~1.7 deg/tick) and smooth.
		float pitch_turn = 8.f;      // extra down-deg per deg/tick of |yaw rate|
		float pitch_rate = 1.2f;     // max response, deg/tick
		float pitch_desc = 0.35f;    // descent-follow strength (0..1.5)
		bool  duck = false;
		int   jump_mode = JM_None;
		// Boundary SMOOTHING (straddle): >0 blends the view across this
		// segment's START over this many ticks - half in the previous
		// segment's tail, half in this segment's head - so the per-tick view
		// rate is continuous through the cut instead of snapping. In optimal
		// modes any wish stays max-gain, so the blend costs almost no speed.
		int   smooth_ticks = 0;

		// ---- PRESTRAFE recipe (deterministic startzone exit) --------------
		// WA ground accel -> jump + release the ground key -> steerable optimal
		// air-strafe -> crouch at the last airborne tick to skim the floor. The
		// engine sims the real physics (ground accel, jump, air accel, and the
		// mid-air FinishDuck origin lift) from these inputs; this only emits
		// the button/view stream.
		bool  prestrafe = false;
		int   ps_ground_ticks = 70;   // legacy pre-v18 duration; the provider now
		                              // ends the ground phase by DISTANCE
		float ps_ground_turn = 75.f;  // WINDUP swing (deg): the view swings out this
		                              // far opposite the carve, then returns to the
		                              // start aim exactly at the jump (measured 60-90)
		float ps_jump_dist = 500.f;   // legacy (v18/v19 goal solver) - serialized,
		float ps_goal_dist = 900.f;   // unused since the v3 recipe rework
		int   ps_ground_dir = 1;      // ground strafe key (+1 A/left, -1 D/right)
		int   ps_air_dir = 1;         // FIRST air strafe key
		int   ps_strafes = 2;         // AIR recipe: N alternating optimal strafes
		                              // (spec default: two, optional 1..8)
		int   ps_strafe_ticks = 24;   // AIR: FIXED ticks per strafe (human first
		                              // strafes measured 22-25). Fixed-length
		                              // alternation means the auto-duration can
		                              // never re-shuffle the strafe layout - the
		                              // old span-divided split flip-flopped the
		                              // sides whenever the duration snapped
		float ps_air_bias = 0.f;      // AIR steering tilt (deg, + = right, 0 =
		                              // straight; bends the air path)
		int   ps_air_ticks = 0;       // AIR duration; 0 = AUTO: the segment ends
		                              // one tick past the landing (timing-based,
		                              // slider for non-flat startzones)
		int   ps_ground_extra = 0;    // extra build ticks past max ground speed
		                              // (0 = the minimum-distance default)
		bool  ps_crouch_end = true;   // crouch near the end (dodge the floor)
		bool  ps_crouch_auto = true;  // legacy (serialized, ignored since the
		                              // crouch went pure-timing: its landing
		                              // detection re-timed against the hull
		                              // lift = a feedback loop, "solving")
		int   ps_crouch_lead = 2;     // manual: ticks before the segment end to crouch
		int   ps_crouch_cached = -1;  // auto: last-airborne tick from the last sim (runtime)
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
		// What the NEXT search runs. opt/blend above belong to the APPLIED
		// schedule (they say how sd.rates replay); the checkbox must not
		// reinterpret an applied line - flipping it used to turn applied
		// biases into turn rates, the corrector then "fixed" the wreck.
		bool  want_opt = false;
		int   want_blend = 8;
		// RIDE solve: the segment STARTS ON the target's face and rides it to
		// the point - same-side bias phases pressing into the face, and the
		// (duration, split) family spreads the approach yaw (the flick
		// lead-in). Flight ballistic candidate seeding is meaningless here.
		// `ride` = what the NEXT search runs (the checkbox); `applied_ride`
		// = how the APPLIED schedule replays (stamped at Apply, same
		// separation as opt/want_opt - a checkbox must never reinterpret
		// numbers that are already applied).
		bool  ride = false;
		bool  applied_ride = false;
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

	// Per-tick INPUT override bits (any input, not just jump): mask says which
	// inputs this tick overrides, value carries the forced state for those bits.
	enum : int {
		OV_W = 1, OV_A = 2, OV_S = 4, OV_D = 8, OV_JUMP = 16, OV_DUCK = 32,
	};
	struct InputOvr {
		int tick = 0;    // local tick within the segment
		int mask = 0;    // which OV_ bits are overridden at this tick
		int value = 0;   // forced state for the masked bits
	};

	struct EditSegment {
		bool raw = false;             // raw = explicit per-tick frames
		bool is_solver = false;       // solver-driven strafe route
		GenParams gen;                // pitch/duck settings shared by all kinds
		SolverData solver;
		std::vector<Frame> frames;
		// Per-tick input overrides for generated/solver segments: flip a single
		// tick's key/jump/crouch without baking. Segment edits clear them.
		std::vector<InputOvr> ovr;
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
	int  g_tab = 2;                   // 0 Project, 1 Record, 2 Run, 3 Targets, 4 Rendering
	char g_name[64] = "untitled";
	StartState g_anchor;
	std::vector<EditSegment> g_segs;
	int g_sel = -1;                   // selected segment

	// Test-play divergence tracking: during an editor test play, the REAL
	// player origin is compared per tick against the sim's states - the
	// direct measurement of the "line is engine-exact" promise.
	bool   g_testplay = false;
	int    g_play_count = 0;
	float  g_play_max = 0.f;
	int    g_play_max_tick = -1;
	int    g_play_first_tick = -1;    // first tick deviating > 0.5 u
	Vector g_play_div_pos = Vector(0.f, 0.f, 0.f);
	int g_cursor = 0;                 // playhead tick
	bool g_show_hull = true;          // ghost hull at the playhead
	int  g_dur_max = 300;             // Duration slider range cap (persisted;
	                                  // render tab). A segment already longer
	                                  // keeps its value - the row's max floors
	                                  // at it so nothing ever silently clamps.
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
	// Deep search: the tournament FULL config (wider N window, ungated
	// polish). Costs ~3.8 u/s mean when OFF (worst scenario -35) but the
	// default stays inside the 10-15s budget; deep is the final-line hunt.
	bool g_deep_search = false;

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

	// COAST line: a gray "if you let go of everything now" trajectory off the
	// run's END (= end of the last segment). It is simmed as extra no-input
	// ticks in the SAME pass, then PEELED into this separate buffer before any
	// analysis runs - so g_states stays exactly the run and nothing else is
	// touched. Draw-only. g_coast_from is the sim tick the coast begins at,
	// set per sim request and read by the provider (-1 = off).
	bool g_coast_line = false;         // toggle (render tab + bindable)
	int  g_coast_ticks = 132;          // length in ticks (~2 s at 66t)
	int  g_coast_from = -1;            // sim tick coast begins (-1 = disabled)
	std::vector<Vector> g_coast_path;  // peeled coast origins (draw only)

	bool g_dirty = false;
	unsigned long long g_dirty_ms = 0;
	bool g_sim_requested = false;
	bool g_sim_fault = false;

	// Project file list (Documents\sourceTAS\projects). Leaked on purpose:
	// this DLL lives until the process dies, and its atexit destructor ran
	// into torn-down CRT state at game exit (crash.log 2026-07-24, offset
	// resolved to the g_files ??__F thunk via the linker map). A reference
	// to a heap object registers no destructor at all.
	// One entry per .tasproj on disk, with the v22 summary header parsed up
	// front so the Project pane can show what each file IS without loading it.
	struct ProjFileInfo {
		std::string name;                 // filename (with extension)
		unsigned long long size = 0;      // bytes
		unsigned long long mtime_raw = 0; // last write FILETIME (sort key)
		SYSTEMTIME mtime = {};            // last write, local time
		bool     has_info = false;        // v22+ summary header present
		uint32_t version = 0;
		char     map[64] = {};            // level at save time ("" = unknown)
		uint32_t total_ticks = 0;
		std::vector<std::pair<uint8_t, int32_t>> segs;   // kind code, ticks
		bool     anchor_valid = false;    // v23+: the anchor rides the summary
		float    anchor[3] = {};
	};
	std::vector<ProjFileInfo>& g_files = *new std::vector<ProjFileInfo>();
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
	struct PlanOvr { int seg; int local; int mask; int value; };

	// NATURAL (pre-override) key state per GLOBAL tick, stamped by the
	// provider on every sim. The override UI must show KEYS, not the collapsed
	// move floats: A+D both held nets sidemove 0, and the display still has to
	// light both keys pink.
	uint8_t g_nat_keys[Prediction::kMaxSimTicks] = {};

	// ---- FREECAM state (functions live near the draw code) -------------
	bool     g_freecam = false;
	Vector   g_fc_pos = Vector(0.f, 0.f, 0.f);
	float    g_fc_pitch = 0.f, g_fc_yaw = 0.f;
	float    g_fc_speed = 600.f;          // u/s (Shift = x3); persisted pref
	// Slot + layout discovery: every candidate virtual is sampled; a slot
	// pins when its argument matches the engine view (angles + eye) at
	// STABLE offsets over several samples. Per-slot state, first pin wins.
	// The CViewSetup layout for THIS client.dll build, measured by the
	// retired 12..20 discovery sweep (freecam_probe.log, four consistent
	// sessions, 2026-08-05): OverrideView = slot 16, view origin at +0x40,
	// view angles at +0x4C. NEVER trusted blind: every session re-verifies
	// them against engine truth (below) before a single write, so a game
	// update that moves anything turns freecam off loudly.
	constexpr int kFcSlot = 16;
	constexpr int kFcOrgOff = 0x40;
	constexpr int kFcAngOff = 0x4C;
	// TRUSTED, not re-derived: six independent sessions measured the SAME
	// slot and offsets (freecam_probe.log). The background check below is a
	// diagnostic that can only LOG - it never gates the camera. (It used to,
	// and it locked freecam out: OverrideView fires several times per frame -
	// main view, 3D SKYBOX camera, others - so single-sample-per-frame
	// checking kept grabbing the sky setup, failing, and finally tripping a
	// give-up counter that disabled the feature outright.)
	bool     g_fc_confirmed = false;      // background sanity check passed
	int      g_fc_checks = 0;             // sanity-check attempts
	// Hook-context stash RING: the wrapper memcpys EVERY call of the frame
	// into its own slot, so the pump sees the main view even when the sky
	// camera is also flowing through.
	constexpr int kFcRing = 4;
	unsigned char g_fc_stash[kFcRing][0x140] = {};
	volatile LONG g_fc_stash_idx = 0;
	// High-resolution frame timing (GetTickCount64's ~15.6 ms resolution
	// quantized flight into visible bursts at high fps) + optional visual
	// position smoothing. g_fc_pos is the true integrator; g_fc_pos_vis is
	// what the camera renders (and what picks ray from, so the crosshair
	// never lies) - with smoothing off they are identical.
	int64_t  g_fc_qpc_last = 0;
	bool     g_fc_smooth = true;          // persisted pref
	Vector   g_fc_pos_vis = Vector(0.f, 0.f, 0.f);
	PlanSeg g_plan[kMaxPlanSegs];
	int     g_plan_count = 0;
	PlanOvr g_plan_ovr[kMaxPlanOvr];
	int     g_plan_ovr_count = 0;
	Frame   g_plan_raw[Prediction::kMaxSimTicks];
	float   g_plan_anchor_yaw = 0.f;
	float   g_plan_anchor_pitch = 0.f;
	float   g_plan_cap = 30.f;
	float   g_plan_accel = 37.5f;     // airaccel * wishspeed * interval

	// Provider walk state (reset at tick 0; game thread only).
	int   g_pv_seg = 0;
	float g_pv_last_yaw = 0.f;
	float g_pv_entry_yaw = 0.f;
	float g_pv_entry_heading = 0.f;   // velocity heading at segment entry (opt modes)
	bool  g_pv_prev_jump = false;
	float g_pv_pitch = 0.f;           // HUMAN pitch: eased value carried per tick
	// Prestrafe walk state: segment-entry origin (the ray + jump distance are
	// measured from here), the tick the air phase actually started at (strafe
	// alternation split), the tick the ground CARVE began (-1 = still in the
	// build phase), and the view at the jump (the air blend's start).
	Vector g_pv_ps_org = Vector(0.f, 0.f, 0.f);
	int    g_pv_ps_gt = 0;
	int    g_pv_ps_carve = -1;
	int    g_pv_ps_plateau = -1;      // tick ground speed stopped climbing
	float  g_pv_ps_prev_spd = -1.f;   // previous tick's 2D speed (plateau detect)
	float  g_pv_ps_jump_view = 0.f;

	// --- UNDO / REDO ---------------------------------------------------------
	// Snapshot-based: segments + anchor + selection. A "pre" snapshot is kept
	// refreshed while idle; the first USER mutation of a burst (MarkDirty with
	// >800 ms since the previous one) pushes that pre-burst state, so slider
	// drags coalesce into one undo step. Machine edits (the auto loops inside
	// SimLandedStage, undo/redo itself) never push.
	struct UndoState {
		std::vector<EditSegment> segs;
		StartState anchor;
		int sel = -1;
	};
	void MarkDirty();   // defined below (calls back into UndoOnUserEdit)

	std::vector<UndoState> g_undo_stack, g_redo_stack;
	UndoState g_undo_pre;
	bool     g_undo_pre_valid = false;
	bool     g_machine_edit = false;
	uint64_t g_last_user_edit_ms = 0;
	bool     g_autosave_pending = false;

	void CaptureUndoPre() {
		g_undo_pre.segs = g_segs;
		g_undo_pre.anchor = g_anchor;
		g_undo_pre.sel = g_sel;
		g_undo_pre_valid = true;
	}

	void UndoOnUserEdit() {
		g_autosave_pending = true;   // any mutation (machine too) re-arms autosave
		if (g_machine_edit)
			return;
		const uint64_t now = GetTickCount64();
		if (now - g_last_user_edit_ms > 800 && g_undo_pre_valid) {
			g_undo_stack.push_back(g_undo_pre);
			if (g_undo_stack.size() > 50)
				g_undo_stack.erase(g_undo_stack.begin());
			g_redo_stack.clear();
		}
		g_last_user_edit_ms = now;
	}

	// Refresh the pre-burst snapshot while idle (throttled - raw segments can
	// carry thousands of frames).
	void UndoIdleTick() {
		static uint64_t s_last_cap = 0;
		const uint64_t now = GetTickCount64();
		if (now - g_last_user_edit_ms > 800 && now - s_last_cap > 500) {
			s_last_cap = now;
			CaptureUndoPre();
		}
	}

	void ApplyUndoState(const UndoState& s) {
		g_machine_edit = true;
		g_segs = s.segs;
		g_anchor = s.anchor;
		g_sel = s.sel;
		if (g_sel >= static_cast<int>(g_segs.size()))
			g_sel = static_cast<int>(g_segs.size()) - 1;
		MarkDirty();
		g_machine_edit = false;
		CaptureUndoPre();
		g_last_user_edit_ms = 0;
	}

	void DoUndo() {
		if (g_undo_stack.empty()) {
			g_status = "Nothing to undo.";
			return;
		}
		UndoState cur;
		cur.segs = g_segs; cur.anchor = g_anchor; cur.sel = g_sel;
		g_redo_stack.push_back(std::move(cur));
		ApplyUndoState(g_undo_stack.back());
		g_undo_stack.pop_back();
		g_status = "Undo.";
	}

	void DoRedo() {
		if (g_redo_stack.empty()) {
			g_status = "Nothing to redo.";
			return;
		}
		UndoState cur;
		cur.segs = g_segs; cur.anchor = g_anchor; cur.sel = g_sel;
		g_undo_stack.push_back(std::move(cur));
		ApplyUndoState(g_redo_stack.back());
		g_redo_stack.pop_back();
		g_status = "Redo.";
	}

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

	void UndoOnUserEdit();   // defined with the undo machinery below

	void MarkDirty() {
		UndoOnUserEdit();
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

	const PlanOvr* FindOverride(int seg, int local) {
		for (int i = 0; i < g_plan_ovr_count; ++i)
			if (g_plan_ovr[i].seg == seg && g_plan_ovr[i].local == local)
				return &g_plan_ovr[i];
		return nullptr;
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

	// The generated-segment view yaw at local tick t, for the current sim
	// velocity/heading and the segment's entry captures. Extracted so the
	// boundary smoother can evaluate a NEIGHBOR segment at the same tick and
	// blend. entryYaw/entryHeading only matter for turn-rate (mode 1) and
	// total-heading (kind 0); heading-relative modes (optimal steering, the
	// default) ignore them, so the smoother is exact there.
	float GenSegYaw(const GenParams& g, int t, const Vector& vel, float lastYaw,
	                float entryYaw, float entryHeading) {
		float yaw = entryYaw;
		if (g.yaw_mode == 1)
			yaw = entryYaw - g.yaw_rate * static_cast<float>(t);
		else if (g.yaw_mode == 2) {
			const float h = HeadingDeg(vel, lastYaw);
			if (g.bias_kind == 0) {
				const float want = NormYaw((entryHeading
					+ static_cast<float>(g.opt_dir) * g.yaw_rate) - h);
				const int rem = (g.ticks - t) > 0 ? (g.ticks - t) : 1;
				const float th_des = want / static_cast<float>(rem);
				const float v2 = Speed2D(vel);
				const float th_nat = Deg(atan2f(g_plan_cap, (v2 > 1.f) ? v2 : 1.f));
				float bias = static_cast<float>(g.opt_dir) * th_des - th_nat;
				if (bias < -20.f) bias = -20.f;
				if (bias > 20.f) bias = 20.f;
				yaw = h + static_cast<float>(g.opt_dir) * bias;
			} else if (g.bias_kind == 2) {
				yaw = h - g.yaw_rate;
			} else {
				yaw = h + static_cast<float>(g.opt_dir) * g.yaw_rate;
			}
		} else if (g.yaw_mode == 3) {
			// Straight sync: steer in SCREEN sign (+ = right), same convention
			// as the L/R steering bias, and tick-independent (splice-safe).
			yaw = HeadingDeg(vel, lastYaw) - g.yaw_rate;
		} else if (g.yaw_mode == 4) {
			// Straight SKEW: the view is pure max-gain (heading); the steering
			// comes entirely from the asymmetric strafe dwell (below), so the
			// bias never tilts the view here.
			yaw = HeadingDeg(vel, lastYaw);
		}
		return NormYaw(yaw);
	}

	// Closed-loop frame generation: called by the sim (game thread) per tick,
	// with the simulated state after the previous tick.
	void Provider(int tick, const Prediction::SimState& prev, Frame* out) {
		if (tick == 0) {
			g_pv_seg = 0;
			g_pv_last_yaw = g_plan_anchor_yaw;
			g_pv_entry_yaw = g_plan_anchor_yaw;
			g_pv_prev_jump = false;
			g_pv_pitch = g_plan_anchor_pitch;   // HUMAN pitch eases from the anchor
		}

		// COAST tail: once past the run (and any solver overrun), emit a pure
		// NO-INPUT frame - view frozen at the last emitted angles (no mouse),
		// no movement keys, no buttons. The engine then sims friction/gravity
		// naturally. These ticks are peeled off after the sim, so they never
		// enter the run's states or analysis.
		if (g_coast_from >= 0 && tick >= g_coast_from) {
			out->viewangles[0] = g_pv_pitch;     // held (no view change)
			out->viewangles[1] = g_pv_last_yaw;
			out->forwardmove = 0.f;
			out->sidemove = 0.f;
			out->upmove = 0.f;
			out->buttons = 0;
			out->impulse = 0;
			out->mousedx = 0;
			out->mousedy = 0;
			return;   // leaves g_pv_last_yaw / g_pv_pitch frozen for the next tick
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
			g_pv_pitch = out->viewangles[0];   // HUMAN pitch continues seamlessly
			return;
		}

		const GenParams& g = ps.gen;
		const bool onground = (prev.flags & FL_ONGROUND) != 0;

		if (t == 0) {
			g_pv_entry_yaw = g.yaw_abs ? g.yaw_start : g_pv_last_yaw;
			g_pv_entry_heading = HeadingDeg(prev.velocity, g_pv_last_yaw);
		}

		float yaw = g_pv_entry_yaw;
		float fmove = 0.f;
		float smove = 0.f;
		bool  ps_jump = false, ps_duck = false;   // prestrafe's own jump/duck

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
			// Rides hold one side (into the ramp); flights alternate.
			const int side = sd.applied_ride ? sd.side0
				: ((k % 2 == 0) ? sd.side0 : -sd.side0);
			if (sd.applied_ride)
				yaw = NormYaw(HeadingDeg(prev.velocity, g_pv_last_yaw)
					+ static_cast<float>(side) * sd.rates[k]);   // no blend: see EvalPass
			else if (sd.opt)
				yaw = OptYawStep(g_pv_last_yaw, prev.velocity, t, sd.ticks,
					(sd.blend < 1) ? 1 : sd.blend, sd.rates[k], side, sd.tyaw);
			else
				yaw = NormYaw(g_pv_last_yaw - sd.rates[k]);
			smove = (side > 0) ? -450.f : 450.f;   // +1 = A (left), -1 = D (right)
		} else if (g.prestrafe) {
			// PRESTRAFE v3 (user spec 2026-08-06): TWO RECIPES, sliders, NO
			// solvers - "the final location is all that matters, the path is
			// the mixture of recipes."
			//   GROUND recipe: W+A speed-build until ground speed PLATEAUS
			//   (max ground speed in the minimum distance) plus an optional
			//   extra, then the carve sweeps the VELOCITY onto the segment
			//   aim as hard as the server's ground accel allows, and the jump
			//   fires the moment the heading ALIGNS with the aim. Getting max
			//   speed then jumping is the whole job.
			//   AIR recipe: N alternating optimal strafes (default two) with
			//   a constant steering tilt - a convenient default shape. Its
			//   duration is the air slider, or AUTO = one tick past landing
			//   (the segment length snaps to it - see the duration snap).
			//   Crouch: the tick before landing by default (adjustable).
			// View per tick = whatever makes the WISH lead the velocity by
			// the steer command (W+A's wish sits 45 deg to the A-side of the
			// view): view = vel_heading + steer - dir*45.
			if (t == 0) {
				g_pv_ps_gt = 0;
				g_pv_ps_carve = -1;
				g_pv_ps_plateau = -1;
				g_pv_ps_prev_spd = -1.f;
				g_pv_ps_jump_view = g_pv_entry_yaw;
			}
			const float aim = g_pv_entry_yaw;   // straight-ahead by default
			const float spd = Speed2D(prev.velocity);
			// Plateau = the build stopped gaining: max ground speed reached
			// in the minimum distance.
			if (g_pv_ps_plateau < 0 && onground && t >= 4
				&& spd > 100.f && spd - g_pv_ps_prev_spd < 0.25f)
				g_pv_ps_plateau = t;
			g_pv_ps_prev_spd = spd;
			const int extra = (g.ps_ground_extra < 0) ? 0 : g.ps_ground_extra;
			if (g_pv_ps_carve < 0 && g_pv_ps_plateau >= 0
				&& t >= g_pv_ps_plateau + extra)
				g_pv_ps_carve = t;
			// The carve completes by ALIGNMENT, not by a clock: the jump fires
			// the moment the velocity heading reaches the aim. The sweep runs
			// at whatever rate the server's ground accel physically allows
			// (the steer clamp saturates), so changing sv_accelerate reshapes
			// the carve's length but can never break the recipe - the launch
			// is ALWAYS down the aim. (The old timed sweep was tuned to one
			// accel setting and launched off-aim on any other.)
			const float vh = HeadingDeg(prev.velocity,
				NormYaw(aim - static_cast<float>(g.ps_ground_dir) * g.ps_ground_turn));
			const bool aligned = g_pv_ps_carve >= 0
				&& fabsf(NormYaw(aim - vh)) < 2.f;
			const bool ground_phase = onground && !aligned && t < ps.ticks - 2;
			if (ground_phase) {
				g_pv_ps_gt = t + 1;   // the air phase starts after this tick
				// Build holds the velocity off-aim; the carve steers straight
				// onto the aim (the clamped P gives a natural fast-then-ease
				// sweep as the error shrinks).
				const float off = (g_pv_ps_carve >= 0) ? 0.f : g.ps_ground_turn;
				const float desired = NormYaw(aim
					- static_cast<float>(g.ps_ground_dir) * off);
				float steer = NormYaw(desired - vh) * 0.6f;
				if (steer > 70.f) steer = 70.f;
				if (steer < -70.f) steer = -70.f;
				yaw = NormYaw(vh + steer - static_cast<float>(g.ps_ground_dir) * 45.f);
				g_pv_ps_jump_view = yaw;   // the air blend starts from here
				fmove = 450.f;   // W
				smove = (g.ps_ground_dir > 0) ? -450.f : 450.f;   // A / D
			} else {
				// Air: N alternating optimal strafes HOMING on the goal point.
				// Same view-tilt steering law as the straight modes, but the
				// desired heading is recomputed toward the goal every tick, so
				// the flight line passes through the ray tip no matter how the
				// ground phase went. Near the goal the steering freezes (the
				// bearing would flip as it passes); the tilt cap keeps the
				// cost tiny (cos^2 4 deg = 99.5% of max gain).
				// FIXED-LENGTH strafes: each lasts ps_strafe_ticks, alternation
				// runs through N strafes, then the last side HOLDS. The layout
				// depends only on ticks-since-jump - the segment's auto-sizing
				// can never re-shuffle it (the old span-divided split moved the
				// side boundaries every time the duration snapped, flip-
				// flopping the path: the reported "solver behavior").
				const int at = t - g_pv_ps_gt;
				const int N = (g.ps_strafes < 1) ? 1 : g.ps_strafes;
				const int per = (g.ps_strafe_ticks < 6) ? 6 : g.ps_strafe_ticks;
				int si = (at < 0) ? 0 : at / per;
				if (si >= N) si = N - 1;
				const int side = (si % 2 == 0) ? g.ps_air_dir : -g.ps_air_dir;
				const float h = HeadingDeg(prev.velocity, g_pv_last_yaw);
				// Max-gain line + a constant steering tilt (+ = right): the
				// path continues the launch direction, bent only by the slider.
				float ay = NormYaw(h - g.ps_air_bias);
				// Blend from the LAST GROUND VIEW onto the optimal air line
				// over a few ticks so the jump has no visible snap (same
				// straddle idea as boundary smoothing; the brief off-optimal
				// wish costs ~nothing).
				const int ab = 8;
				if (at >= 0 && at < ab) {
					const float w = static_cast<float>(at) / static_cast<float>(ab);
					ay = NormYaw(ay + NormYaw(g_pv_ps_jump_view - ay) * (1.f - w));
				}
				yaw = ay;
				fmove = 0.f;
				smove = (side > 0) ? -450.f : 450.f;
			}
			// Launch on the first air-phase tick while still grounded (releases
			// the ground key that same tick since smove already switched sides).
			if (t == g_pv_ps_gt && onground)
				ps_jump = true;
			// Crouch: PURE TIMING - this many ticks before the segment end,
			// nothing else. The end sits one tick past the landing by default
			// (the duration snap), so lead 2 = crouch the tick before landing
			// with zero landing detection here. (The old auto-timer looked at
			// the sim's ground flags, and the crouch's own ~18u hull lift then
			// MOVED the landing it was timed against - a feedback loop the
			// user rightly called solving behavior.)
			if (g.ps_crouch_end) {
				const int ctick = ps.ticks - (g.ps_crouch_lead < 0 ? 0 : g.ps_crouch_lead);
				if (t >= ctick)
					ps_duck = true;
			}
		} else {
			// Normal generated segment: view from GenSegYaw (mode 1/2/3).
			yaw = GenSegYaw(g, t, prev.velocity, g_pv_last_yaw,
				g_pv_entry_yaw, g_pv_entry_heading);

			// BOUNDARY SMOOTHING (straddle): if this tick is within a smoothing
			// window around a boundary, blend the two segments' natural yaw so
			// the per-tick view rate is continuous. The smooth_ticks belongs to
			// the LATER segment; the window is half in the earlier segment's
			// tail and half in the later segment's head. Exact for heading-
			// relative modes (the neighbor's entry captures are unused there).
			if (!ps.raw) {
				int Bstart = -1, Pseg = -1, Nseg = -1, W = 0;
				if (g.smooth_ticks > 0 && g_pv_seg > 0) {
					const int before = g.smooth_ticks / 2;
					if (t < g.smooth_ticks - before) {   // head of a smoothed seg
						Bstart = ps.start; Pseg = g_pv_seg - 1; Nseg = g_pv_seg;
						W = g.smooth_ticks;
					}
				}
				if (Nseg < 0 && g_pv_seg + 1 < g_plan_count) {
					const PlanSeg& nx = g_plan[g_pv_seg + 1];
					if (!nx.raw && !nx.solver && !nx.gen.prestrafe && nx.gen.smooth_ticks > 0) {
						const int before = nx.gen.smooth_ticks / 2;
						if (tick >= nx.start - before) {   // tail before a smoothed seg
							Bstart = nx.start; Pseg = g_pv_seg; Nseg = g_pv_seg + 1;
							W = nx.gen.smooth_ticks;
						}
					}
				}
				const bool pP = (Pseg >= 0) && !g_plan[Pseg].solver && !g_plan[Pseg].raw
					&& g_plan[Pseg].gen.prestrafe;
				const bool pN = (Nseg >= 0) && g_plan[Nseg].gen.prestrafe;
				if (Nseg >= 0 && W > 0 && Pseg >= 0 && !pP && !pN) {
					const int before = W / 2;
					const float yawP = (g_plan[Pseg].solver || g_plan[Pseg].raw)
						? g_pv_last_yaw   // solver/raw neighbor: hold its last view
						: GenSegYaw(g_plan[Pseg].gen, tick - g_plan[Pseg].start,
							prev.velocity, g_pv_last_yaw, g_pv_entry_yaw, g_pv_entry_heading);
					const float yawN = GenSegYaw(g_plan[Nseg].gen, tick - g_plan[Nseg].start,
						prev.velocity, g_pv_last_yaw, g_pv_entry_yaw, g_pv_entry_heading);
					float s = static_cast<float>(tick - (Bstart - before)) / static_cast<float>(W);
					s = (s < 0.f) ? 0.f : (s > 1.f ? 1.f : s);
					yaw = NormYaw(yawP + NormYaw(yawN - yawP) * s);
				}
			}

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
			else if (g.yaw_mode == 4) {
				// Asymmetric dwell (view stays max-gain). yaw_rate = tick skew,
				// screen sign: + holds the RIGHT (D) side longer -> bends right,
				// - holds LEFT (A) longer -> bends left. opt_dir-independent so
				// the slider always means the same thing.
				const int base = (g.opt_period < 1) ? 1 : g.opt_period;
				int skew = static_cast<int>(g.yaw_rate + (g.yaw_rate >= 0.f ? 0.5f : -0.5f));
				int pA = base - skew, pD = base + skew;   // A = left, D = right
				if (pA < 1) pA = 1;
				if (pD < 1) pD = 1;
				const int cycle = pA + pD;
				const int phase = ((t % cycle) + cycle) % cycle;
				smove = (phase < pA) ? -450.f : 450.f;   // A for pA ticks, then D
			}
		}

		float pitch = g.pitch_val;
		if (g.pitch_mode == PM_Follow) {
			const float v2 = Speed2D(prev.velocity);
			pitch = (v2 > 1.f || fabsf(prev.velocity.Z) > 1.f)
				? -Deg(atan2f(prev.velocity.Z, v2)) * g.pitch_mult : 0.f;
			if (pitch > 89.f) pitch = 89.f;
			if (pitch < -89.f) pitch = -89.f;
		} else if (g.pitch_mode == PM_Dynamic) {
			// Dynamic recipe (measured; see GenParams): target = base
			// + turn-coupling * |this tick's yaw rate|
			// + descent-follow * downslope while falling,
			// then ease toward it at most pitch_rate deg/tick. Source sign:
			// + = down. Deterministic - same inputs, same stream.
			float dyaw = fabsf(NormYaw(yaw - g_pv_last_yaw));
			if (dyaw > 6.f) dyaw = 6.f;    // boundary snaps aren't "turning"
			float target = g.pitch_val + g.pitch_turn * dyaw;
			const float v2 = Speed2D(prev.velocity);
			if (!onground && prev.velocity.Z < 0.f && (v2 > 1.f || -prev.velocity.Z > 1.f))
				target += Deg(atan2f(-prev.velocity.Z, v2)) * g.pitch_desc;
			if (target > 85.f) target = 85.f;
			if (target < -30.f) target = -30.f;
			float step = target - g_pv_pitch;
			const float rate = (g.pitch_rate < 0.05f) ? 0.05f : g.pitch_rate;
			if (step > rate) step = rate;
			if (step < -rate) step = -rate;
			g_pv_pitch += step;
			pitch = g_pv_pitch;
		}

		int buttons = 0;
		if (g.prestrafe) {
			// The prestrafe drives its own jump and end-crouch; the generic
			// jump setting does not apply, but a held crouch (g.duck) does.
			if (ps_jump) buttons |= IN_JUMP;
			if (ps_duck || g.duck) buttons |= IN_DUCK;
		} else {
			if (g.duck) buttons |= IN_DUCK;
			if (g.jump_mode == JM_Hold)
				buttons |= IN_JUMP;
			else if (g.jump_mode == JM_AutoBhop && onground && !g_pv_prev_jump)
				buttons |= IN_JUMP;   // press only on ticks that start grounded
		}
		if (ps.solver && ps.sdata.start_jump && t == 0)
			buttons |= IN_JUMP;   // solver's grounded-start hop

		// Per-tick INPUT overrides, applied at KEY level. The natural key
		// state is derived from the composed movement (generators emit one
		// side at +-450, so the reconstruction is lossless), stamped for the
		// override UI, then the overridden bits replace their key - opposing
		// keys COMBINE like the real input stack (A+D held nets sidemove 0).
		{
			uint8_t nat = 0;
			if (fmove > 0.f) nat |= OV_W;
			if (fmove < 0.f) nat |= OV_S;
			if (smove < 0.f) nat |= OV_A;
			if (smove > 0.f) nat |= OV_D;
			if (buttons & IN_JUMP) nat |= OV_JUMP;
			if (buttons & IN_DUCK) nat |= OV_DUCK;
			if (tick >= 0 && tick < Prediction::kMaxSimTicks)
				g_nat_keys[tick] = nat;
			if (const PlanOvr* ov = FindOverride(g_pv_seg, t)) {
				const int eff = (nat & ~ov->mask) | (ov->value & ov->mask);
				fmove = ((eff & OV_W) ? 450.f : 0.f) + ((eff & OV_S) ? -450.f : 0.f);
				smove = ((eff & OV_A) ? -450.f : 0.f) + ((eff & OV_D) ? 450.f : 0.f);
				buttons = (eff & OV_JUMP) ? (buttons | IN_JUMP) : (buttons & ~IN_JUMP);
				buttons = (eff & OV_DUCK) ? (buttons | IN_DUCK) : (buttons & ~IN_DUCK);
			}
		}

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
		// Track the EMITTED pitch in every mode, so a HUMAN segment following
		// a const/follow/raw neighbor eases from where the view actually was
		// instead of snapping from a stale value.
		g_pv_pitch = pitch;
	}

	bool BuildPlan() {
		g_plan_count = 0;
		g_plan_ovr_count = 0;
		memset(g_nat_keys, 0, sizeof(g_nat_keys));
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
				for (const InputOvr& o : s.ovr)
					if (o.tick < ticks && o.mask && g_plan_ovr_count < kMaxPlanOvr)
						g_plan_ovr[g_plan_ovr_count++] = { g_plan_count, o.tick, o.mask, o.value };
			}
			p.ticks = ticks;
			start += ticks;
			g_plan_count++;
		}

		g_plan_anchor_yaw = g_anchor.yaw;
		g_plan_anchor_pitch = g_anchor.pitch;
		float interval = Prediction::LastDiag().interval_per_tick;
		if (interval <= 0.f) interval = 0.015f;
		g_plan_cap = g_air_cap;
		g_plan_accel = g_air_accel * g_wishspeed * interval;
		return start > 0;
	}

	// RAMP (surf board) efficiency: AirAccelerate is IDENTICAL on a ramp -
	// the wish add doesn't know the plane exists; the clip afterwards just
	// removes whatever points into the surface, and gravity converts along
	// the slope for free. So the score is the AIR COMPONENT of the tick:
	// measured gain MINUS the slope's zero-input physics, against the same
	// MaxAirGain optimum free flight uses. The plane is recovered from the
	// sim data itself (consecutive slide velocities lie IN the ramp plane,
	// so their cross product is its normal) and is used ONLY to compute that
	// zero-input baseline. Free flight never reaches here (RecomputeDiag's
	// gravity check runs first); every gate below leaves the tick
	// UNSCOREABLE rather than guessing. Toggleable on the Rendering tab.
	bool g_ramp_eff = true;   // persisted; also gates the gradient on boards
	void RampScore(int i, float prev_speed) {
		if (!g_ramp_eff)
			return;
		const Vector& v1 = g_states[i - 1].velocity;   // entering the tick
		const Vector& v2 = g_states[i].velocity;       // leaving it (in-plane)
		const float m1 = sqrtf(v1.X * v1.X + v1.Y * v1.Y + v1.Z * v1.Z);
		const float m2 = sqrtf(v2.X * v2.X + v2.Y * v2.Y + v2.Z * v2.Z);
		if (m1 < 50.f || m2 < 50.f)
			return;                                    // too slow: normal too noisy
		Vector nrm(v1.Y * v2.Z - v1.Z * v2.Y,
		           v1.Z * v2.X - v1.X * v2.Z,
		           v1.X * v2.Y - v1.Y * v2.X);
		const float nm = sqrtf(nrm.X * nrm.X + nrm.Y * nrm.Y + nrm.Z * nrm.Z);
		if (nm < m1 * m2 * 0.004f)
			return;                                    // near-parallel: no stable plane
		nrm.X /= nm; nrm.Y /= nm; nrm.Z /= nm;
		if (nrm.Z < 0.f) { nrm.X = -nrm.X; nrm.Y = -nrm.Y; nrm.Z = -nrm.Z; }
		if (nrm.Z < 0.02f || nrm.Z > 0.999f)
			return;                                    // wall / floor: not a ride

		float interval = Prediction::LastDiag().interval_per_tick;
		if (interval <= 0.f) interval = 0.015f;
		// Pre-clip velocity entering the tick (gravity applied, no input yet).
		const Vector pre(v1.X, v1.Y, v1.Z - g_gravity * interval);
		const float pen = pre.X * nrm.X + pre.Y * nrm.Y + pre.Z * nrm.Z;
		if (pen > -1.f)
			return;                                    // not moving INTO the plane: no slide
		// BASELINE: the engine's clip (overbounce 1) with zero input - the
		// slope physics the tick gives for free.
		const Vector base(pre.X - pen * nrm.X, pre.Y - pen * nrm.Y, pre.Z - pen * nrm.Z);
		const float base2d = sqrtf(base.X * base.X + base.Y * base.Y);

		g_maxgain[i] = MaxAirGain(prev_speed);         // the SAME yardstick as flight
		if (g_maxgain[i] > 0.001f) {
			g_eff[i] = (g_gain[i] - (base2d - prev_speed)) / g_maxgain[i];
			if (g_eff[i] > 1.f) g_eff[i] = 1.f;        // baseline-estimate guard
		} else {
			g_maxgain[i] = 0.f;
		}
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
		float prev_vz = g_anchor.velocity.Z;
		for (int i = 0; i < n; ++i) {
			const float sp = Speed2D(g_states[i].velocity);
			g_speed[i] = sp;
			g_gain[i] = sp - prev_speed;
			const bool air = (i > 0) && ((g_states[i - 1].flags & FL_ONGROUND) == 0);
			// FREE FLIGHT (gravity alone explains the vertical change): score
			// the raw gain against the air-accel optimum - nothing else
			// touched the tick. Everything else airborne goes to RampScore,
			// which separates slope physics from strafing and scores only the
			// controllable part (raw scoring of slide ticks read >1000%).
			if (air) {
				const float expect_vz = prev_vz - g_gravity * interval;
				if (fabsf(g_states[i].velocity.Z - expect_vz) < 1.5f) {
					g_maxgain[i] = MaxAirGain(prev_speed);
					if (g_maxgain[i] > 0.001f)
						g_eff[i] = g_gain[i] / g_maxgain[i];
				} else {
					RampScore(i, prev_speed);
				}
			}
			prev_speed = sp;
			prev_vz = g_states[i].velocity.Z;
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
		bool  opt = false;            // rates[] are biases (opt) or turn rates (const)
		bool  ride = false;           // same-side phases (ride) or alternating (flight)
		int   blend = 8;              // opt: the blend window this line was solved with
		int   nsplits = 0;
		int   splits[3] = {};         // local ticks where the strafe key flips
		float rates[4] = {};          // deg/tick per phase, screen sign (+ = right)
		int   wrap = 0;               // which 360-degree wrap of the yaw equation
		float pass_err = 0.f;         // model: |XY(t*) - target XY|
		float speed = 0.f;            // 2D speed at the pass
		float arr_heading = 0.f;      // velocity HEADING at the pass (approach yaw)
		float arr_vz = 0.f;           // vertical velocity at the pass (flick height)
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
		int    entity_brushes = 0;    // solid-flagged BRUSH ENTITIES skipped (triggers
		                              // etc. - not collision, per the world model)
		int    corridor_dropped = 0;  // solid brushes culled by the collider cap - NEVER
		                              // silent: logged, because a missing brush is a
		                              // model-real divergence factory
		int    diverged = 0;          // lines dropped at verify: the REAL measurement
		                              // contradicted the model claim (apex band lines,
		                              // model 0.1-0.4u vs real ~2u landed). Geometric
		                              // gates were tried TWICE and measured wrong - the
		                              // proven 355.2 winner's endpoint sits in the same
		                              // band and reproduces; real agreement is the
		                              // arbiter, per line, from data.
		std::vector<RouteSolution> dropped_diverged;   // kept for the log
		// OPT+B refinement wave (offline tournament 2026-07-25: the full opt
		// stack = +14.2 u/s mean over the shipped algorithm, 40/40 scenarios).
		// Warm seeds and per-job blend/seed state live in thread_locals.
		struct RefineTask {           // re-solve a fast line at one blend
			int ticks, side0, nsplits, blend;
			int splits[3];
			float frac;
			float rates[4];
		};
		std::vector<RefineTask> refine;
		int    refine_idx = -1;       // -1 = wave not built yet
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
		bool  ride = false;           // on-face ride solve (single into-face strafe)
		std::vector<RouteSolution> results;
	};
	SearchCtx g_search;
	int g_solution_pick = 0;

	// ------------------------- threaded search runtime -----------------------
	// The enumeration used to run in 5 ms render-thread slices - minutes of
	// wall time once the collider fix (96->168) and the tournament-winning
	// opt config landed. Jobs are now built FLAT at kickoff and chewed by a
	// static-partitioned worker pool: contiguous job ranges per worker, so
	// per-worker warm seeds stay deterministic for a given thread count.
	// Kept OUTSIDE SearchCtx: atomics/mutexes would break its value-reset.
	struct SearchJob {
		int N; float frac; int side0; int nsplits; int splits[3];
		bool refine; int refine_blend; float seed[4];
	};
	std::vector<SearchJob> g_jobs;
	std::vector<std::thread> g_search_workers;
	std::atomic<int>  g_jobs_done_a{ 0 };
	std::atomic<int>  g_results_n{ 0 };     // race-safe "passes collected" mirror
	std::atomic<int>  g_workers_left{ 0 };
	std::atomic<bool> g_sfault{ false };
	std::atomic<bool> g_sabort{ false };
	std::mutex g_results_mtx;               // guards results + top_rejects + reject_best
	char g_fault_detail[160] = {};
	void LaunchSearchWorkers();   // defined with the worker, below
	void StopSearchWorkers();
	// Per-worker solver state (was on g_search - workers would race it).
	// Outcome COUNTERS stay plain ints: concurrent increments can drop a few
	// counts, which is acceptable for diagnostics and keeps EvalPass lock-free.
	thread_local float tl_warm_fr[2][2];
	thread_local bool  tl_have_warm[2] = { false, false };
	thread_local float tl_warm_ob[2][3];
	thread_local bool  tl_have_warm_ob[2] = { false, false };
	thread_local float tl_best_speed = 0.f;   // polish gate: only lines near the
	                                          // worker's speed lead get walked
	thread_local bool  tl_refine_active = false;
	thread_local int   tl_refine_blend = 8;
	thread_local float tl_refine_seed[4] = {};
	// True on a search worker thread. A fault here is NEVER the prediction
	// path's routine-and-recovered AV, so the VEH logs it (those are normally
	// filtered out to keep crash.log clean). Names a worker crash that slips
	// past the per-job SEH guard (the loop scaffolding, the mutex, ...).
	thread_local bool  tl_is_worker = false;
	// The blend window is iterated PER JOB - through a global it would race
	// across workers straight into EvalPass. 0 = fall back to g_search.blend
	// (the main thread's context, e.g. the corrector re-solving an applied
	// schedule).
	thread_local int   tl_blend = 0;
	int CurBlend() { return tl_blend > 0 ? tl_blend : g_search.blend; }

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
		float pass_speed = -1.f;      // REAL 2D speed at the pass tick - the number
		                              // the model's per-line `speed` claims
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

	// sfric = m_surfaceFriction, which scales the air-accel budget. The
	// engine sets 0.25 when its end-of-move ground probe finds no walkable
	// plane while the player is RISING (0 < vz <= 140; above that the probe
	// is skipped and the value stays 1). Solver-core measurement 2026-08-15
	// (fuzz decks, engine applied exactly 150*85*0.015*0.25 = 47.8125 u/s);
	// carried per-state because the value that governs a tick is the one the
	// PREVIOUS tick's end-of-move probe left.
	struct FastState { Vector p, v; float yaw; float sfric = 1.f; };

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
			// Budget scales by m_surfaceFriction (see FastState::sfric).
			// NOTE: the OPTIMAL strafe angle and its per-tick gain are
			// unaffected - with budget >= the 30 cap in both regimes the
			// optimum is the perpendicular wish either way - so the planner
			// math needs no change; this only corrects OFF-optimal wishes.
			const float budget = g_search.accel_amt * s.sfric;
			if (add > budget) add = budget;
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
					// UNCLAMPED, like the engine: when the sweep starts touching
					// several planes (apex corner), the LEAST-NEGATIVE enterfrac
					// picks the contact plane. Clamping to 0 before the compare
					// made the first-listed plane win ties - the measured apex
					// landings hit the top bevel (-0.011) over the face (-0.025);
					// the clamp handed them to the face and the model surfed off.
					const float tt = (d0 - 0.03125f) / (d0 - d1);   // DIST_EPSILON
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
	// grounded = CategorizePosition's end-of-move probe (2u down-trace onto
	// a walkable plane with vz <= 140) set ground - the engine's ONLY
	// grounding rule - so the model flight ends there, origin snapped.
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
			// accel budget scales by m_surfaceFriction (see FastState).
			const float budget = g_search.accel_amt * s.sfric;
			if (add > budget) add = budget;
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
		}
		if (nseg == 0)
			seg_pts[++nseg] = s.p;   // fully blocked: zero-length segment
		// CategorizePosition - FullWalkMove runs it between the move and
		// FinishGravity, and it is the ONLY thing that sets ground: falling
		// no faster up than NON_JUMP_VELOCITY (140), a 2-unit down-trace
		// hitting a walkable plane (nz >= 0.7) sets the ground entity AND
		// SNAPS the origin to the trace end. A flight skimming the ramp
		// APEX lands on the top face WITHOUT TryPlayerMove ever striking a
		// plane (measured 2026-07-24: the side0=-1 ~2.06u real-vs-model
		// cluster - real vz -6 walking the top at z 255.98 from tick 60
		// while the model slid the face below). Striking a walkable plane
		// mid-bump leaves the hull ON it, so this probe also covers what
		// the old direct-contact grounding caught.
		// This probe also OWNS m_surfaceFriction for the NEXT tick (see
		// FastState::sfric): reset to 1, and 0.25 when nothing walkable is
		// under a RISING player. Above vz 140 the engine skips the probe
		// entirely, so the previous value stands.
		if (s.v.Z <= 140.f) {
			s.sfric = 1.f;
			int gb, gpl;
			const float gf = TraceHullWorld(s.p, s.p - Vector(0.f, 0.f, 2.f), &gb, &gpl);
			if (gf < 1.f && gb >= 0
				&& g_search.bcolliders[gb].pn[gpl].Z >= 0.7f) {
				s.p.Z -= 2.f * gf;           // engine: origin = trace endpos
				seg_pts[++nseg] = s.p;       // the perigee sees the snapped point
				*grounded = true;
			} else if (s.v.Z > 0.f) {
				s.sfric = 0.25f;
			}
		}
		s.v.Z -= g_search.gravity * g_search.dt * 0.5f;          // FinishGravity
		if (*grounded)
			s.v.Z = 0.f;   // FullWalkMove: grounded ticks end with vz cleared
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
		float vyaw = 0.f;       // velocity HEADING at the pass (the approach yaw)
		float vz = 0.f;         // vertical velocity at the pass (flick height feed)
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
			float best_vy = HeadingDeg(s.v, s.yaw);
			float best_vz = s.v.Z;
			float best_d2 = Dot3(s.p - tgt);
			auto track = [&](const Vector& a, const Vector& b2, int tick2, float v2,
			                 float yawv, float vyawv, float vzv) {
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
					best_vy = vyawv;
					best_vz = vzv;
				}
			};
			out.bump_ticks = 0;
			out.bump_loss = 0.f;
			int contact_tick = -1;
			bool foreign_counted = false;
			for (int t = 0; t < N + 8; ++t) {
				while (k < nsplits && t >= splits[k]) k++;
				// A RIDE holds ONE side (the key pressing into the ramp) and
				// varies only the per-phase bias; flights alternate A/D.
				const int side = g_search.ride ? side0
					: ((k % 2 == 0) ? side0 : -side0);
				// rates[] = deg/tick turn rates, or per-phase BIASES off the
				// exact max-gain line in optimal mode (blend tail lands the
				// view exactly on the target yaw).
				// A RIDE never blends: forcing the view onto a board yaw at
				// ride speed puts the wish ~136 deg off the velocity and the
				// engine answers with a 562 u/s backward impulse that destroys
				// the line (measured). The arrival yaw is an OUTPUT of a ride -
				// the spread the flick chooses from - not a constraint.
				const float yaw_next = g_search.ride
					? NormYaw(HeadingDeg(s.v, s.yaw)
						+ static_cast<float>(side) * rates[k])
					: g_search.opt
					? OptYawStep(s.yaw, s.v, t, N, CurBlend(), rates[k], side,
						g_search.target_yaw)
					: NormYaw(s.yaw - rates[k]);
				Vector segp[6];
				TickContact tc[4];
				int ntc = 0;
				bool grounded = false;
				const int nseg = EngineMoveTick(s, yaw_next, side,
				                                segp, tc, &ntc, &grounded);
				const float v2n = sqrtf(s.v.X * s.v.X + s.v.Y * s.v.Y);
				const float vyn = HeadingDeg(s.v, s.yaw);
				for (int si = 0; si < nseg; ++si)
					track(segp[si], segp[si + 1], t + 1, v2n, s.yaw, vyn, s.v.Z);
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
			out.vyaw = best_vy;
			out.vz = best_vz;
			out.bump_ticks = (contact_tick >= 0 && best_tick > contact_tick)
				? best_tick - contact_tick : 0;
			if (contact_tick >= 0 && contact_tick > best_tick)
				out.bump_loss = 0.f;   // board AFTER the pass: post-event, not a kiss
			return true;
		}
		Vector prev = s.p;
		for (int t = 0; t < N; ++t) {
			while (k < nsplits && t >= splits[k]) k++;
			const int side = g_search.ride ? side0
				: ((k % 2 == 0) ? side0 : -side0);
			const float yaw_next = g_search.opt
				? OptYawStep(s.yaw, s.v, t, N, CurBlend(), rates[k], side,
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
		out.vyaw = HeadingDeg(s.v, s.yaw);
		out.vz = s.v.Z;
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
			} else if (reason == 1) {
				g_search.reject_retime++;
			} else {
				g_search.cnt_bump++;
			}
			// Keep the ~24 closest rejects with full detail for the log.
			std::lock_guard<std::mutex> lk(g_results_mtx);
			if (err < g_search.reject_best && reason == 0)
				g_search.reject_best = err;   // exact under the lock (workers race)
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
				RouteSolution s;
				s.ticks = p.tick;
				s.frac = p.frac;
				s.side0 = side0;
				s.opt = g_search.opt;       // how rates[] replay - travels with the line
				s.ride = g_search.ride;     // same-side vs alternating - ditto
				s.blend = CurBlend();       // the window this line solved with
				s.nsplits = nsplits;
				for (int i = 0; i < 3; ++i) s.splits[i] = (i < nsplits) ? splits[i] : 0;
				memcpy(s.rates, rr, sizeof(float) * 4);
				s.wrap = cur_wrap;
				s.pass_err = err;
				s.speed = p.speed;
				s.arr_heading = p.vyaw;
				s.arr_vz = p.vz;
				s.pass_pos = p.pass;
				s.bump_ticks = p.bump_ticks;
				s.bump_loss = p.bump_loss;
				s.exact = (err <= g_search.tol && p.bump_loss <= g_search.max_bump);
				{
					std::lock_guard<std::mutex> lk(g_results_mtx);
					if (static_cast<int>(g_search.results.size()) < 20000) {
						g_search.results.push_back(s);
						g_results_n.fetch_add(1, std::memory_order_relaxed);
					} else {
						g_search.cap_dropped++;
					}
				}
			} else if (yaw_ok && err <= g_search.collect) {
				record_reject(rr, err, p, expected_tick, 2);   // violent board
			} else if (!yaw_ok) {
				record_reject(rr, err, p, expected_tick, 1);
			} else {
				record_reject(rr, err, p, expected_tick, 0);
			}
		};

		// LM over 1-3 free parameters; fmap maps free index -> bound index.
		// Parametrized over the residual and bounds so the optimal-bias mode
		// reuses it unchanged (3 free biases won every offline scenario).
		// FAILURE-TOLERANT: an infeasible evaluation (early board / no
		// contact / yaw out of range) is a WALL to walk along, never a reason
		// to abort - aborting froze the whole search at its starting guesses
		// (the frozen -6.4/+6.4 rates visible across three diagnostics logs).
		auto newton = [&](auto&& rfn, const float* rlo, const float* rhi,
		                  float* fr, int nfree, const int* fmap) -> float {
			float F[2];
			if (!rfn(fr, nfree, F, nullptr))
				return 1e9f;
			float err = sqrtf(F[0] * F[0] + F[1] * F[1]);
			float lambda = 0.05f;
			const float h = 0.04f;
			for (int it = 0; it < 30 && err > 0.02f; ++it) {
				float J[2][3] = {};
				bool have_col[3] = { false, false, false };
				for (int i = 0; i < nfree; ++i) {
					for (int attempt = 0; attempt < 2 && !have_col[i]; ++attempt) {
						float f2[3] = { fr[0], (nfree > 1) ? fr[1] : 0.f,
						                (nfree > 2) ? fr[2] : 0.f };
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
				bool any = false;
				for (int i = 0; i < nfree; ++i)
					any |= have_col[i];
				if (!any)
					break;   // boxed in on every side - genuinely stuck
				bool stepped = false;
				for (int attempt = 0; attempt < 6 && !stepped; ++attempt) {
					// (J^T J + lambda I) d = -J^T F, nfree x nfree (<= 3),
					// by Gauss-Jordan with partial pivoting.
					float M[3][4] = {};
					for (int i = 0; i < nfree; ++i) {
						for (int j = 0; j < nfree; ++j)
							M[i][j] = J[0][i] * J[0][j] + J[1][i] * J[1][j];
						M[i][i] += lambda;
						M[i][nfree] = -(J[0][i] * F[0] + J[1][i] * F[1]);
					}
					bool sing = false;
					for (int c = 0; c < nfree && !sing; ++c) {
						int piv = c;
						for (int r2 = c + 1; r2 < nfree; ++r2)
							if (fabsf(M[r2][c]) > fabsf(M[piv][c]))
								piv = r2;
						if (fabsf(M[piv][c]) < 1e-12f) { sing = true; break; }
						if (piv != c)
							for (int j = 0; j <= nfree; ++j) {
								const float tmp = M[piv][j];
								M[piv][j] = M[c][j];
								M[c][j] = tmp;
							}
						for (int r2 = 0; r2 < nfree; ++r2) {
							if (r2 == c) continue;
							const float f = M[r2][c] / M[c][c];
							for (int j = 0; j <= nfree; ++j)
								M[r2][j] -= f * M[c][j];
						}
					}
					if (sing) { lambda *= 5.f; continue; }
					float d[3] = {};
					for (int i = 0; i < nfree; ++i)
						d[i] = M[i][nfree] / M[i][i];
					float m = 0.f;
					for (int i = 0; i < nfree; ++i)
						m = fmaxf(m, fabsf(d[i]));
					if (m > 2.f)
						for (int i = 0; i < nfree; ++i)
							d[i] *= 2.f / m;
					float f2[3] = { fr[0], (nfree > 1) ? fr[1] : 0.f,
					                (nfree > 2) ? fr[2] : 0.f };
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
			// ALL biases free (up to the LM's 3) - the offline lab won every
			// scenario with 3 free over 2 (+1..3 u/s) - and TWO blend windows
			// tried per candidate (worth several more); each window's best is
			// collected as its own line, carrying its blend for the apply.
			const int nfree = (K < 3) ? K : 3;
			const int bbase = K - nfree;
			const int fmapo[3] = { bbase, bbase + 1, bbase + 2 };
			const float ostarts[5][3] = {
				{0.f, 0.f, 0.f}, {5.f, -5.f, 0.f}, {-5.f, 5.f, 0.f},
				{0.f, 5.f, -5.f}, {0.f, -5.f, 5.f} };
			const int save_blend = g_search.blend;
			// FOUR blend windows per candidate (offline tournament: windows +
			// polish + stride 2 = +11.2 u/s mean over the shipped pair, 39/40
			// scenarios, sign p ~ 2e-12). A refinement task instead solves
			// exactly ONE window, seeded with its line's own biases.
			int blendset[4] = { save_blend, save_blend + 6,
			                    save_blend - 3, save_blend + 12 };
			for (int i = 0; i < 4; ++i)
				blendset[i] = blendset[i] < 2 ? 2 : (blendset[i] > 24 ? 24 : blendset[i]);
			// A ride has no blend tail, so the blend windows are all the
			// same line - solve it once.
			const int nblend = (tl_refine_active || g_search.ride) ? 1 : 4;
			if (tl_refine_active)
				blendset[0] = tl_refine_blend;
			for (int bsel = 0; bsel < nblend; ++bsel) {
				bool bdup = false;
				for (int i = 0; i < bsel; ++i)
					bdup |= (blendset[i] == blendset[bsel]);
				if (bdup)
					continue;
				tl_blend = blendset[bsel];
				// SELF-CONSISTENT schedule length, same as constant mode: the
				// blend is anchored to the schedule length T, and the APPLIED
				// solution replays with ticks = the pass tick - if those
				// differ, the provider blends toward a different tick than
				// the search solved and the real line diverges in the final
				// ticks (measured: model 0.04u vs real 9.7u, plus the early
				// boards that shifted stream caused). Solve AT T, require the
				// perigee to land AT T, re-anchor and re-solve otherwise.
				int T = N;
				for (int fp = 0; fp < 3; ++fp) {
					bool feasible = true;
					for (int i = 0; i < nsplits; ++i)
						if (splits[i] >= T)
							feasible = false;
					if (!feasible || T < 8)
						break;
					float best_err = 1e9f;
					float best_b[4] = {};
					PassEval best_po{};
					auto tresid = [&](const float* fr, int nf, float* F, PassEval* pout) -> bool {
						float bias[4] = { 0.f, 0.f, 0.f, 0.f };
						for (int i = 0; i < nf; ++i) bias[bbase + i] = fr[i];
						if (!EvalPass(T, frac, side0, splits, nsplits, bias, pe))
							return false;
						const Vector dd = pe.pass - tgt;
						F[0] = Dot(g_search.e1, dd);
						F[1] = Dot(g_search.e2, dd);
						if (pout) *pout = pe;
						return true;
					};
					const int widx = (side0 > 0) ? 0 : 1;
					const int nstarts = tl_refine_active
						? 2 : (tl_have_warm_ob[widx] ? 6 : 5);
					for (int s0 = 0; s0 < nstarts; ++s0) {
						float fr[3];
						if (tl_refine_active) {
							// Start 0 = the line's own biases; start 1 = zeros.
							for (int i = 0; i < 3; ++i)
								fr[i] = (s0 == 0 && bbase + i < 4)
									? tl_refine_seed[bbase + i] : 0.f;
						} else if (s0 == 5) {
							memcpy(fr, tl_warm_ob[widx], sizeof(fr));
						} else {
							fr[0] = ostarts[s0][0];
							fr[1] = (nfree > 1) ? ostarts[s0][1] : 0.f;
							fr[2] = (nfree > 2) ? ostarts[s0][2] : 0.f;
						}
						for (int i = 0; i < nfree; ++i)
							fr[i] = Clampf(fr[i], blo[fmapo[i]], bhi[fmapo[i]]);
						if (nfree > 0)
							newton(tresid, blo, bhi, fr, nfree, fmapo);
						float F[2];
						PassEval p;
						if (tresid(fr, nfree, F, &p)) {
							const float e = sqrtf(F[0] * F[0] + F[1] * F[1]);
							if (e < best_err) {
								best_err = e;
								memset(best_b, 0, sizeof(best_b));
								for (int i = 0; i < nfree; ++i) best_b[bbase + i] = fr[i];
								best_po = p;
							}
						}
						if (best_err <= g_search.tol && best_po.tick == T)
							break;
					}
					if (best_err > 1e8f) {
						g_search.cnt_no_contact++;
						break;
					}
					// A RIDE's trajectory does not depend on the schedule
					// length at all (no blend tail), so there is nothing to
					// align: the arrival tick IS wherever the perigee lands
					// and the segment simply ends there. Requiring tick == T
					// made the residual undefined almost everywhere and the
					// solve stalled at 171 u; anchoring to the perigee closes
					// the same case to 0.004 u. Both phases must actually run.
					if (g_search.ride) {
						const float acc_err = sqrtf(Dot3(best_po.pass - tgt));
						if (best_po.tick > splits[0])
							consider(best_b, acc_err, best_po, best_po.tick);
						else
							record_reject(best_b, acc_err, best_po, best_po.tick, 1);
						break;
					}
					if (best_po.tick == T) {
						const float acc_err = sqrtf(Dot3(best_po.pass - tgt));
						consider(best_b, acc_err, best_po, T);
						// Polish only within reach of this worker's speed lead:
						// walking every mediocre basin was most of the minutes.
						// Deep search polishes everything (the tournament FULL
						// config, worth ~+3.8 u/s mean / up to +35).
						const bool near_lead = g_deep_search
							|| best_po.speed > tl_best_speed - 3.f;
						if (acc_err <= g_search.tol
							&& best_po.speed > tl_best_speed)
							tl_best_speed = best_po.speed;
						if (acc_err <= g_search.tol && nfree == 3) {
							memcpy(tl_warm_ob[widx], best_b + bbase,
								sizeof(float) * 3);
							tl_have_warm_ob[widx] = true;
							// NULL-SPACE SPEED POLISH: 2 residuals over 3 free
							// biases leave a 1-D curve of solutions that all hit
							// the point; cross(J0,J1) is its tangent. Walk it
							// both ways, LM-repair the miss each step, and every
							// replayed speed gain becomes its own line.
							if (near_lead)
							for (int dirsel = 0; dirsel < 2; ++dirsel) {
								float pt[3] = { best_b[bbase], best_b[bbase + 1],
								                best_b[bbase + 2] };
								float pspeed = best_po.speed;
								for (int pstep = 0; pstep < 6; ++pstep) {
									float F0[2];
									if (!tresid(pt, 3, F0, nullptr))
										break;
									float J[2][3];
									bool jok = true;
									for (int i = 0; i < 3 && jok; ++i) {
										float f2[3] = { pt[0], pt[1], pt[2] };
										const float cl = Clampf(f2[i] + 0.04f,
											blo[fmapo[i]], bhi[fmapo[i]]);
										const float hs = cl - f2[i];
										if (fabsf(hs) < 1e-6f) { jok = false; break; }
										f2[i] = cl;
										float F2[2];
										if (!tresid(f2, 3, F2, nullptr)) { jok = false; break; }
										J[0][i] = (F2[0] - F0[0]) / hs;
										J[1][i] = (F2[1] - F0[1]) / hs;
									}
									if (!jok)
										break;
									Vector nu(J[0][1] * J[1][2] - J[0][2] * J[1][1],
									          J[0][2] * J[1][0] - J[0][0] * J[1][2],
									          J[0][0] * J[1][1] - J[0][1] * J[1][0]);
									const float nl = sqrtf(Dot3(nu));
									if (nl < 1e-9f)
										break;
									nu = Scale(nu, (dirsel == 0 ? 1.5f : -1.5f) / nl);
									float nxt[3] = {
										Clampf(pt[0] + nu.X, blo[fmapo[0]], bhi[fmapo[0]]),
										Clampf(pt[1] + nu.Y, blo[fmapo[1]], bhi[fmapo[1]]),
										Clampf(pt[2] + nu.Z, blo[fmapo[2]], bhi[fmapo[2]]) };
									newton(tresid, blo, bhi, nxt, 3, fmapo);
									float Fx[2];
									PassEval px;
									if (!tresid(nxt, 3, Fx, &px))
										break;
									if (px.tick != T)
										break;
									const float ex = sqrtf(Dot3(px.pass - tgt));
									if (ex > g_search.tol)
										break;
									if (px.speed > pspeed + 0.05f) {
										memcpy(pt, nxt, sizeof(pt));
										pspeed = px.speed;
										float pb[4];
										memcpy(pb, best_b, sizeof(pb));
										for (int i = 0; i < 3; ++i)
											pb[bbase + i] = nxt[i];
										consider(pb, ex, px, T);
									} else
										break;
								}
							}
						}
						break;
					}
					if (fp == 2) {
						record_reject(best_b, sqrtf(Dot3(best_po.pass - tgt)), best_po, T, 1);
						break;
					}
					T = best_po.tick;   // re-anchor the blend to the real pass tick
				}
			}
			tl_blend = 0;
			(void)save_blend;
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
						const bool warm = tl_have_warm[sidx];
						const int nstarts = warm ? 9 : 8;
						for (int s0 = 0; s0 < nstarts; ++s0) {
							float fr[2];
							if (warm && s0 == 0) {
								fr[0] = Clampf(tl_warm_fr[sidx][0], lo[0], hi[0]);
								fr[1] = (nfree > 1)
									? Clampf(tl_warm_fr[sidx][1], lo[1], hi[1]) : 0.f;
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
							tl_warm_fr[sidx][0] = best_r[0];
							tl_warm_fr[sidx][1] = (nfree > 1) ? best_r[1] : 0.f;
							tl_have_warm[sidx] = true;
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
	// RIDE candidates: a ride's duration is paced by ALONG-FACE progress, not
	// free fall (the 13:30 attempt seeded ballistically: N 33 vs real contact
	// 57+, every candidate TICK-ALIGN-rejected 400u short). Rides accelerate
	// down-slope and pressing-up lines take longer, so the window is wide and
	// the in-resid retimer fills the gaps.
	void RideCandidates() {
		g_search.cands.clear();
		// With no blend tail the ride's path is independent of the schedule
		// length - only the split and the two biases shape it, and the
		// arrival tick is read off the perigee. So ONE generous evaluation
		// window is enough; BuildSplits enumerates the split inside it and
		// each solve reports its own N. (Ballistic seeding was the original
		// failure: N 33 against a real contact at 57+.)
		const float dist = sqrtf(Dot3(g_search.target_pos - g_search.start_pos));
		const float spd = fmaxf(sqrtf(Dot3(g_search.start_vel)), 50.f);
		const int n0 = static_cast<int>(dist / (spd * g_search.dt) + 0.5f);
		int win = static_cast<int>(static_cast<float>(n0) * 1.8f) + 12;
		if (win < 24) win = 24;
		if (win > 280) win = 280;
		SearchCtx::PassCand c;
		c.N = win;
		c.frac = 1.f;
		g_search.cands.push_back(c);
	}

	// The strafe key whose wish direction presses INTO the face (wish = yaw
	// +- 90, exactly the AirAccelerate formula the tick uses).
	int RideSideIntoFace(float heading, const Vector& n) {
		int best = 1;
		float bestd = -1e9f;
		for (int side = -1; side <= 1; side += 2) {
			const float w = (heading + 90.f * static_cast<float>(side)) * (kPi / 180.f);
			const float d = -(cosf(w) * n.X + sinf(w) * n.Y);
			if (d > bestd) {
				bestd = d;
				best = side;
			}
		}
		return best;
	}

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
					// +2 on the late side: a pre-target kiss slides and passes
					// late, but the in-resid retimer re-anchors from any seed -
					// the old +4 window only manufactured sibling-N duplicates
					// (six copies of every line in the 11:47 log's rejects).
					// Deep search restores the full window.
					for (int dj = -1; dj <= (g_deep_search ? 4 : 2); ++dj) {
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
			// Optimal mode: the biases do the shaping, so near-identical
			// timings converge to the same line - a coarser stride buys the
			// blend-pair + 3-bias budget at the same total search time.
			const int stride = g_search.opt ? 2 : ((N > 120) ? 2 : 1);
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
		sd.opt = sol.opt;                // the SOLUTION carries its own semantics
		sd.applied_ride = sol.ride;
		sd.tyaw = g_search.target_yaw;   // provider's blend target (opt mode)
		sd.blend = (sol.blend < 1) ? 1 : (sol.blend > 24 ? 24 : sol.blend);
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
	// Origin-space expansion of a brush plane: the Minkowski sum of the brush
	// with the NEGATED player hull. For plane n.p <= d the hull (offsets b in
	// [mins,maxs]) touches iff n.origin <= d - min_b(n.b), and min_b picks
	// mins on the axes where n is positive, maxs where it is negative. The
	// hull comes from the ENGINE (CCollisionProperty), not a constant - so a
	// mod/server with different player bounds is modelled correctly.
	float HullExpand(const Vector& n) {
		Vector mn, mx;
		Prediction::PlayerHull(&mn, &mx);
		return -((n.X > 0.f ? n.X * mn.X : n.X * mx.X)
			+ (n.Y > 0.f ? n.Y * mn.Y : n.Y * mx.Y)
			+ (n.Z > 0.f ? n.Z * mn.Z : n.Z * mx.Z));
	}

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
				float d_origin = pd + HullExpand(pn);   // engine hull, feet origin
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
			// BEVEL planes: CM_ClipBoxToBrush clips swept boxes against the
			// compiler's bevel sides (axial + edge bevels). Use the EXACT lump
			// planes - the apex top-land vs face-surf branch flips on their
			// dists (a winding-derived top at 256.004 vs the compiler's 256.000
			// put the model on the wrong side of the measured landings).
			const int nbev = BspWorld::BrushBevelPlaneCount(brush);
			for (int i = 0; i < nbev; ++i) {
				Vector pn;
				float pd = 0.f;
				if (!BspWorld::GetBrushBevelPlane(brush, i, &pn, &pd))
					continue;
				const float d_origin = pd + HullExpand(pn);
				bc.pn.push_back(pn);
				bc.pd.push_back(d_origin);
				bc.pid.push_back(-2);   // bevel: never a boardable face
			}
			// Origin-space AABB gate: brush box grown by the hull (feet origin
			// reaches 16 out sideways and 72 down-to-feet above the box).
			Vector bmin, bmax;
			if (BspWorld::GetBrushInfo(brush, nullptr, &bmin, &bmax)) {
				Vector hmn, hmx;
				Prediction::PlayerHull(&hmn, &hmx);
				bc.bmin = bmin - hmx;   // origin-space box: brush (-) hull
				bc.bmax = bmax - hmn;
				if (nbev == 0) {
					// No lump bevels (shouldn't happen on compiled maps):
					// fall back to axial planes of the winding AABB.
					const Vector bn[6] = {
						Vector(1.f, 0.f, 0.f), Vector(-1.f, 0.f, 0.f),
						Vector(0.f, 1.f, 0.f), Vector(0.f, -1.f, 0.f),
						Vector(0.f, 0.f, 1.f), Vector(0.f, 0.f, -1.f) };
					const float bd[6] = { bmax.X, -bmin.X, bmax.Y, -bmin.Y, bmax.Z, -bmin.Z };
					for (int i = 0; i < 6; ++i) {
						const float d_origin = bd[i] + HullExpand(bn[i]);
						bc.pn.push_back(bn[i]);
						bc.pd.push_back(d_origin);
						bc.pid.push_back(-2);
					}
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
		g_search.corridor_dropped = 0;
		g_search.entity_brushes = 0;
		for (int bi = 0; bi < nbr; ++bi) {
			int contents = 0;
			Vector bmin, bmax;
			if (!BspWorld::GetBrushInfo(bi, &contents, &bmin, &bmax))
				continue;
			// SOLID | WINDOW | GRATE | PLAYERCLIP: what MASK_PLAYERSOLID hits.
			if (!(contents & (0x1 | 0x2 | 0x8 | 0x10000)))
				continue;
			// ...but only if it is WORLD geometry. Brush entities (trigger
			// volumes, illusionaries) are CONTENTS_SOLID on disk and block
			// nothing - surf maps carpet the void under the ramps with them,
			// and treating them as collision invented floors that made real
			// targets look unreachable.
			if (!BspWorld::IsWorldBrush(bi)) {
				g_search.entity_brushes++;
				continue;
			}
			if (bmax.X < cmin.X - 16.f || bmin.X > cmax.X + 16.f ||
			    bmax.Y < cmin.Y - 16.f || bmin.Y > cmax.Y + 16.f ||
			    bmax.Z < cmin.Z - 16.f || bmin.Z > cmax.Z + 72.f)
				continue;
			// A capped-out brush is a hole in the model's world - count it
			// loudly (96 used to cull 72 brushes of this very corridor).
			if (static_cast<int>(out.size()) >= 256) {
				g_search.corridor_dropped++;
				continue;
			}
			add_brush(bi, false, -1);
		}
	}

	void StartSearch(int seg_index) {
		StopSearchWorkers();        // never reset g_search under running workers
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
		// RIDE solves are single-strafe bias solves by construction; otherwise
		// the search runs what the CHECKBOX says (want_*). sd.opt/sd.blend
		// describe the currently APPLIED schedule and may differ until the
		// next Apply stamps them from the chosen solution.
		g_search.ride = sd.ride;
		// A ride needs TWO same-side bias phases: the residual is 2D (the
		// pass point) and one bias is one DOF - the single-phase attempt
		// could only get within 63 u (15:12 log, every candidate rejected).
		// Two phases = exactly determined; the (duration, split) family is
		// the spread of approach yaw / lift the flick picks from.
		g_search.strafes = g_search.ride ? 2
			: (sd.strafes < 1 ? 1 : sd.strafes > 4 ? 4 : sd.strafes);
		g_search.opt = g_search.ride ? true : sd.want_opt;
		g_search.blend = (sd.want_blend < 1) ? 1 : (sd.want_blend > 24 ? 24 : sd.want_blend);

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

		if (g_search.ride) {
			RideCandidates();
			if (g_search.have_face) {
				const float phi0 = Dot(g_search.face_n, g_search.start_pos)
					- g_search.face_d;
				if (fabsf(phi0) > 8.f) {
					g_search.done = true;
					char note[256];
					Sfmt(note, "RIDE solve: the segment start is %.1f u off the "
						"target's face plane - a ride must START ON the ramp "
						"(end the previous segment on it, or uncheck RIDE).",
						phi0);
					g_status = note;
					return;
				}
				// Is the target a position a player can actually OCCUPY?
				// The origin belongs OFF the surface (the hull is what rests
				// on it), so the brush's raw box says nothing - the test is
				// whether the HULL at that origin intersects another solid,
				// which in origin space means the point is inside that
				// brush's EXPANDED plane set (what the colliders already
				// carry). Deep inside only (> 1 u past every plane) so a
				// legitimate rest position, which sits exactly ON its face,
				// is never flagged. 2026-07-25: the reported ride target was
				// clear of its own ramp but 23 u inside the floor brushes -
				// the ramp continues below the floor there, so no ride could
				// ever reach it and the search just came back empty.
				for (size_t ci = 0; ci < g_search.bcolliders.size(); ++ci) {
					const SearchCtx::BrushCollider& bc = g_search.bcolliders[ci];
					bool inside = !bc.pn.empty();
					for (size_t pi = 0; pi < bc.pn.size() && inside; ++pi)
						if (Dot(bc.pn[pi], target->pos) - bc.pd[pi] > -1.f)
							inside = false;
					if (inside) {
						g_search.done = true;
						char note[400];
						Sfmt(note, "RIDE solve: no player can stand at this target - "
							"the collision hull there is INSIDE brush %d (the ramp "
							"continues under it). The usable surface ends where the "
							"ramp meets that geometry; re-place the target higher up "
							"the exposed part of the ramp.", bc.brush);
						g_status = note;
						WriteSearchLog();
						return;
					}
				}
			}
		} else {
			BuildPassCandidates();
		}
		if (g_search.cands.empty()) {
			g_search.done = true;
			g_status = g_search.ride
				? "RIDE solve: no duration candidates - the start and target are "
				  "practically the same point on the face."
				: "Solver: the ballistic arc NEVER reaches the target's height from this "
				  "start - enable 'Jump at start' if the start is grounded, otherwise the "
				  "target is too high (or the approach needs more airtime).";
			return;
		}
		// Flat job list for the worker pool (side-major inside each N so a
		// contiguous partition still sees both families early for its warm).
		// A RIDE has exactly one meaningful side: the strafe pressing INTO
		// the face.
		const int ride_side = (g_search.ride && g_search.have_face)
			? RideSideIntoFace(HeadingDeg(g_search.start_vel, g_search.start_yaw),
				g_search.face_n)
			: 0;
		g_jobs.clear();
		for (const SearchCtx::PassCand& c : g_search.cands) {
			BuildSplits(c.N);
			for (int side = 0; side < 2; ++side) {
				const int s0 = (side == 0) ? 1 : -1;
				if (g_search.ride && ride_side != 0 && s0 != ride_side)
					continue;
				for (const std::array<int, 3>& sp : g_search.splits_list) {
					SearchJob j{};
					j.N = c.N;
					j.frac = c.frac;
					j.side0 = s0;
					j.nsplits = g_search.strafes - 1;
					j.splits[0] = sp[0]; j.splits[1] = sp[1]; j.splits[2] = sp[2];
					j.refine = false;
					g_jobs.push_back(j);
				}
			}
		}
		g_search.jobs_total = static_cast<int>(g_jobs.size());
		g_jobs_done_a.store(0);
		g_results_n.store(0);
		g_sfault.store(false);
		g_fault_detail[0] = 0;
		g_search.active = true;
		LaunchSearchWorkers();
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
				if (k.side0 != s.side0 || k.nsplits != s.nsplits || k.ticks != s.ticks
					|| k.blend != s.blend)
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

	// OPT+B refinement wave: after the enumeration, the fastest solved lines
	// each get re-solved across the whole blend range (their own biases as
	// the seed) - the blend window trades tail curvature for speed and the
	// sweep finds each line's own best trade (offline: +3.0 u/s mean, 36/40).
	void BuildRefineTasks() {
		g_search.refine.clear();
		if (!g_search.opt)
			return;
		std::vector<const RouteSolution*> byspeed;
		for (const RouteSolution& s : g_search.results)
			if (s.pass_err <= g_search.tol)
				byspeed.push_back(&s);
		std::sort(byspeed.begin(), byspeed.end(),
			[](const RouteSolution* a, const RouteSolution* b) {
				return a->speed > b->speed;
			});
		const size_t ntop = (std::min)(byspeed.size(), static_cast<size_t>(30));
		for (size_t i = 0; i < ntop; ++i) {
			const RouteSolution& s = *byspeed[i];
			for (int blend = 4; blend <= 20; blend += 2) {
				if (blend == s.blend)
					continue;
				SearchCtx::RefineTask t;
				t.ticks = s.ticks;
				t.side0 = s.side0;
				t.nsplits = s.nsplits;
				t.blend = blend;
				memcpy(t.splits, s.splits, sizeof(t.splits));
				t.frac = s.frac;
				memcpy(t.rates, s.rates, sizeof(t.rates));
				g_search.refine.push_back(t);
			}
		}
	}

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

	// Append one line to crash.log (same file the fault filters write). Used
	// by the handlers below, which fire for deaths that raise NO exception at
	// all and therefore left crash.log EMPTY every time (2026-07-25: two
	// silent instant closes, nothing in any log).
	void WriteCrashLine(const char* what, const char* detail) {
		char documents[MAX_PATH] = {};
		if (!SUCCEEDED(SHGetFolderPathA(nullptr, CSIDL_PERSONAL, nullptr,
			SHGFP_TYPE_CURRENT, documents)))
			return;
		char path[MAX_PATH] = {};
		_snprintf_s(path, _TRUNCATE, "%s\\sourceTAS\\calibration\\crash.log", documents);
		HANDLE f = CreateFileA(path, FILE_APPEND_DATA, FILE_SHARE_READ, nullptr,
			OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
		if (f == INVALID_HANDLE_VALUE)
			return;
		char line[512];
		_snprintf_s(line, _TRUNCATE, "PROCESS DEATH (%s) %s thread=%lu stage=%s\r\n",
			what, detail ? detail : "", GetCurrentThreadId(),
			g_stage_name && g_stage_name[0] ? g_stage_name : "(none)");
		DWORD written = 0;
		WriteFile(f, line, static_cast<DWORD>(strlen(line)), &written, nullptr);
		CloseHandle(f);
	}

	// The three silent killers, none of which raise a catchable exception:
	//   * a failing *_s CRT call (buffer too small, bad arg) -> the invalid
	//     parameter handler, which by DEFAULT kills the process outright;
	//   * an uncaught throw or a destroyed joinable std::thread -> terminate;
	//   * a pure-virtual call.
	// Each now names itself in crash.log before the process goes down.
	void __cdecl InvalidParamHandler(const wchar_t*, const wchar_t*,
	                                 const wchar_t*, unsigned int, uintptr_t) {
		WriteCrashLine("CRT invalid parameter (a *_s call failed)", "");
	}
	void __cdecl TerminateHandler() {
		WriteCrashLine("std::terminate (uncaught throw / joinable thread destroyed)", "");
	}
	void __cdecl PureCallHandler() {
		WriteCrashLine("pure virtual call", "");
	}
	void SignalHandler(int sig) {
		char d[32];
		_snprintf_s(d, _TRUNCATE, "signal=%d", sig);
		WriteCrashLine("abort/signal", d);
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
		// C++ throws (0xE06D7363) are rare in this codebase; an UNCAUGHT one is
		// std::terminate = instant silent death (the 13:2x hard close). Log the
		// first few first-chance so the thrower names itself even if something
		// downstream eats the process.
		if (code == 0xE06D7363) {
			static volatile LONG cxx_logged = 0;
			if (InterlockedIncrement(&cxx_logged) <= 3)
				CrashWriter(ep);
			return EXCEPTION_CONTINUE_SEARCH;
		}
		// A fault on a SEARCH WORKER is always our bug (the prediction path
		// that legitimately faults-and-recovers only runs on the game thread),
		// so log ANY exception there - that is the class of the tab-out crash
		// that left crash.log empty, since a plain AV in a worker is skipped
		// by the fatal-class filter below.
		if (tl_is_worker) {
			static volatile LONG worker_logged = 0;
			if (InterlockedIncrement(&worker_logged) <= 6) {
				g_stage_name = "search-worker";
				CrashWriter(ep);
			}
			return EXCEPTION_CONTINUE_SEARCH;
		}
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

	// (The old auto-crouch solver is GONE - 2026-08-06, user: "stop solving
	// behavior for prestrafe". It timed the crouch off the sim's ground
	// flags, and the crouch's own hull lift then moved the landing it was
	// timed against - an oscillating feedback loop. Crouch is now a pure
	// ticks-before-end lead applied in the provider.)

	// PRESTRAFE duration snap (v3: recipes, NOT solvers). The recipes are
	// feed-forward; the only thing measured from the sim is WHERE things
	// happened, to place the segment END:
	//   air slider > 0 -> end = jump tick + slider ticks (timing-based, for
	//                     non-flat startzones and edge cases);
	//   auto (0)       -> end = ONE TICK PAST the landing (clearing the edge).
	// No goal feedback loop exists anymore, so this converges in a sim or two.
	void UpdatePrestrafeDuration() {
		static int settle = 0;
		if (!g_valid || g_states.empty()) { settle = 0; return; }
		bool changed = false;
		for (size_t k = 0; k < g_segs.size(); ++k) {
			EditSegment& s = g_segs[k];
			if (s.raw || s.is_solver || !s.gen.prestrafe)
				continue;
			const int start = (k < g_starts.size()) ? g_starts[k] : 0;
			int end = start + s.Ticks();
			if (end > static_cast<int>(g_states.size())) end = static_cast<int>(g_states.size());
			if (end <= start + 2)
				continue;
			int jt = -1, lt = -1;
			for (int i = start; i < end; ++i) {
				const bool on = (g_states[i].flags & FL_ONGROUND) != 0;
				if (jt < 0) {
					if (!on) jt = i;
				} else if (on) {
					lt = i;
					break;
				}
			}
			int want;
			if (jt >= 0 && s.gen.ps_air_ticks > 0)
				want = (jt - start) + s.gen.ps_air_ticks;   // manual air duration
			else if (lt >= 0)
				want = (lt - start) + 2;                    // one tick past landing
			else if (jt >= 0)
				want = (jt - start) + 220;                  // auto, still airborne:
				                                            // generous flight window
			else
				want = s.gen.ticks + s.gen.ticks / 2 + 30;  // never even jumped: grow
			if (want < 16) want = 16;
			if (want > 1200) want = 1200;
			const int dtk = want - s.gen.ticks;
			if (dtk > 1 || dtk < -1) {
				s.gen.ticks = want;
				changed = true;
			}
		}
		if (changed && settle < 10) { settle++; MarkDirty(); }
		else settle = 0;
	}

	// Everything that runs when a sim lands, with a stage marker so a fault
	// names its location. Lives behind SimLandedGuarded's POD-only SEH frame.
	void SimLandedStage() {
		// Everything in here is MACHINE refinement (auto duration, pumps,
		// corrections) - none of it may open a new undo step.
		g_machine_edit = true;
		g_stage_name = "TakeSim";
		const int n = Prediction::SimCount();
		g_sim_fault = Prediction::SimFaulted();
		g_frames.resize(n);
		g_states.resize(n);
		Prediction::TakeSim(g_frames.data(), g_states.data());
		// Peel the no-input COAST tail off BEFORE anything else looks at
		// g_states, so the run's states/frames are exactly what they'd be
		// without the coast - every analysis below is untouched.
		g_coast_path.clear();
		if (g_coast_from >= 0 && n > g_coast_from) {
			g_coast_path.reserve(n - g_coast_from);
			for (int i = g_coast_from; i < n; ++i)
				g_coast_path.push_back(g_states[i].origin);
			g_states.resize(g_coast_from);
			g_frames.resize(g_coast_from);
		}
		g_valid = !g_states.empty();
		g_sim_requested = false;
		g_stage_name = "RecomputeDiag";
		RecomputeDiag();
		g_stage_name = "AnalyzeRealPass";
		AnalyzeRealPass();
		g_stage_name = "PrestrafeDuration";
		UpdatePrestrafeDuration();
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
		g_machine_edit = false;
	}
	bool SimLandedGuarded() {
		__try {
			SimLandedStage();
			return true;
		} __except (EXCEPTION_EXECUTE_HANDLER) {
			g_guard_code = static_cast<int>(GetExceptionCode());
			g_machine_edit = false;   // never leave the undo gate stuck shut
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

	// Worker main: chew a contiguous range of the flat job list. Fresh warm
	// seeds per worker (deterministic for a fixed partition); the per-job
	// blend/seed state travels through thread_locals into SolveCandidate.
	void SearchWorkerMain(int lo, int hi) {
		tl_is_worker = true;
		tl_have_warm[0] = tl_have_warm[1] = false;
		tl_have_warm_ob[0] = tl_have_warm_ob[1] = false;
		tl_best_speed = 0.f;
		tl_blend = 0;
		// C++ catch as well as the per-job SEH guard: an uncaught throw in a
		// worker is std::terminate = INSTANT process death with nothing in
		// crash.log (the 13:2x hard close left zero forensics).
		try {
			for (int i = lo; i < hi; ++i) {
				if (g_sabort.load(std::memory_order_relaxed)
					|| g_sfault.load(std::memory_order_relaxed))
					break;
				const SearchJob& j = g_jobs[i];
				tl_refine_active = j.refine;
				tl_refine_blend = j.refine_blend;
				memcpy(tl_refine_seed, j.seed, sizeof(tl_refine_seed));
				if (!SolveCandidateGuarded(j.N, j.frac, j.side0, j.splits, j.nsplits)) {
					Sfmt(g_fault_detail, "N=%d side=%+d splits=%d,%d,%d%s blend=%d",
						j.N, j.side0, j.splits[0], j.splits[1], j.splits[2],
						j.refine ? " refine" : "", j.refine_blend);
					g_sfault.store(true);
					break;
				}
				g_jobs_done_a.fetch_add(1, std::memory_order_relaxed);
			}
		} catch (...) {
			Sfmt(g_fault_detail, "C++ exception in a search worker");
			g_sfault.store(true);
		}
		tl_refine_active = false;
		tl_blend = 0;
		g_workers_left.fetch_sub(1);
	}

	void LaunchSearchWorkers() {
		const int hw = static_cast<int>(std::thread::hardware_concurrency());
		int nw = (hw > 4) ? hw - 2 : 2;
		if (nw > 12) nw = 12;
		const int njobs = static_cast<int>(g_jobs.size());
		if (njobs < 1)
			return;
		if (nw > njobs) nw = njobs;
		g_workers_left.store(nw);
		for (int w = 0; w < nw; ++w) {
			const int lo = njobs * w / nw;
			const int hi = njobs * (w + 1) / nw;
			g_search_workers.emplace_back(SearchWorkerMain, lo, hi);
		}
	}

	void StopSearchWorkers() {
		g_sabort.store(true);
		for (std::thread& t : g_search_workers)
			if (t.joinable())
				t.join();
		g_search_workers.clear();
		g_sabort.store(false);
	}

	void StepSearch() {
		if (!g_search.active || g_search.done)
			return;
		g_search.jobs_done = g_jobs_done_a.load(std::memory_order_relaxed);
		if (g_workers_left.load() > 0)
			return;                     // the pool is still chewing this wave
		StopSearchWorkers();            // join the finished threads
		if (g_sfault.load()) {
			char note[512];
			Sfmt(note, "SOLVER FAULT 0x%08X in a worker at %s - search aborted "
				"safely; this candidate is the repro.",
				g_guard_code, g_fault_detail[0] ? g_fault_detail : "(unknown job)");
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
		// Second wave (opt): blend refinement of the fastest lines, same pool.
		if (g_search.opt && g_search.refine_idx < 0) {
			BuildRefineTasks();
			g_search.refine_idx = 0;
			if (!g_search.refine.empty()) {
				g_jobs.clear();
				for (const SearchCtx::RefineTask& t : g_search.refine) {
					SearchJob j{};
					j.N = t.ticks;
					j.frac = t.frac;
					j.side0 = t.side0;
					j.nsplits = t.nsplits;
					memcpy(j.splits, t.splits, sizeof(j.splits));
					j.refine = true;
					j.refine_blend = t.blend;
					memcpy(j.seed, t.rates, sizeof(j.seed));
					g_jobs.push_back(j);
				}
				g_search.jobs_total += static_cast<int>(g_jobs.size());
				LaunchSearchWorkers();
				return;
			}
		}
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
		if (g_realpass.crossed && g_realpass.tick >= 1
			&& g_realpass.tick <= count)
			g_realpass.pass_speed = Speed2D(g_states[g_realpass.tick - 1].velocity);
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
		// The corrector BORROWS the live search context for its EvalPass
		// calls. Whatever it flips must come back on EVERY exit, or a
		// finished search's results get re-labeled (the 23:14 "const" log
		// was the opt run wearing the corrected segment's mode). RAII so
		// no return path can forget.
		struct CtxRestore {
			bool opt; bool ride; int blend; float tyaw; Vector tpos;
			CtxRestore()
				: opt(g_search.opt), ride(g_search.ride), blend(g_search.blend),
				  tyaw(g_search.target_yaw), tpos(g_search.target_pos) {}
			~CtxRestore() {
				g_search.opt = opt;
				g_search.ride = ride;
				g_search.blend = blend;
				g_search.target_yaw = tyaw;
				g_search.target_pos = tpos;
			}
		} ctx_restore;
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
		g_search.ride = sd.applied_ride;   // ...and holds one side for a ride
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
		if (g_search.active)
			return;   // the corrector borrows g_search - never while jobs run
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
	const char* const kSortKeyItems = "Real miss (u)\0Speed (u/s)\0Yaw err (deg)\0Board loss (u/s)\0Model miss (u)\0Pass vz (flick lift)\0";

	float SortValue(const RouteSolution& s, int key) {
		switch (key) {
		case 1: return -s.speed;                              // faster first
		case 2: return (s.real_yaw_err >= 0.f) ? s.real_yaw_err : 999.f;
		case 3: return s.bump_loss;                           // cleaner board first
		case 4: return s.pass_err;
		case 5: return -s.arr_vz;                             // more lift first
		default: return (s.real_err >= 0.f) ? s.real_err : 1e9f;
		}
	}
	int SortBucket(const RouteSolution& s, int key) {
		if (key < 0 || key > 5)
			key = 0;
		float v = SortValue(s, key);
		float q = 0.05f;                    // miss/yaw quantum
		if (key == 1 || key == 3) q = 0.5f; // speed/loss quantum (u/s)
		if (key == 5) q = 5.f;              // lift quantum (u/s vertical)
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

		// All measured. THE REAL ENGINE IS THE ARBITER: real replay is
		// deterministic per anchor, so a line whose real measurement
		// CONTRADICTS its model claim (the apex-band lines: model 0.1-0.4u,
		// real ~2u landed on the top) can never be flown as promised - it
		// moves OUT of the presented list into the log's DROPPED section.
		// A line that agrees stays even if it grazes the apex: agreement IS
		// the reproducibility proof (two geometric fragility gates were
		// tried first and both measurably culled the proven 355.2 winner).
		{
			// A line PROMISES (pass point, pass tick, view yaw) - but the
			// analyzer measures the closest approach over the WHOLE sim
			// including the +16-tick overrun, so a healthy board-and-slide
			// line legitimately gets its perigee a few ticks PAST N (and in
			// const mode the overrun yaw keeps turning). The 13:10 build
			// compared the promise against that later measurement and dropped
			// ALL 600 lines, perfect-agreement ones included. Corrected:
			//  - miss:   model vs real perigee distance (always comparable)
			//  - tick:   only a perigee far outside the overrun window is a
			//            contradiction (the apex walkers: tick 112 vs N 62)
			//  - yaw:    only checkable when the perigee IS at the promised
			//            tick - past-N perigees carry post-plan yaw.
			// real_tick is GLOBAL; r.ticks is segment-local.
			const int segbase = (g_verify.seg >= 0
				&& g_verify.seg < static_cast<int>(g_starts.size()))
				? g_starts[g_verify.seg] : 0;
			std::vector<RouteSolution> kept, dropped;
			kept.reserve(g_search.results.size());
			for (const RouteSolution& r : g_search.results) {
				const int dtick = (r.real_tick >= 0)
					? (r.real_tick - segbase) - r.ticks : 0;
				// A RIDE promises no view yaw (the arrival angle is its
				// output), so only the point and the timing are checkable.
				const bool disagree = (r.real_err > 1e8f)
					|| fabsf(r.real_err - r.pass_err) > 0.75f
					|| dtick > kSolverOverrun + 2 || dtick < -3
					|| (!r.ride && dtick >= -2 && dtick <= 2
						&& r.real_yaw_err >= 0.f && r.real_yaw_err > 1.f);
				(disagree ? dropped : kept).push_back(r);
			}
			g_search.diverged = static_cast<int>(dropped.size());
			g_search.results.swap(kept);
			g_search.dropped_diverged.swap(dropped);
		}
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
			Sfmt(msg, "Solver: %d candidates real-measured: %d within tol, %d within 3u, "
				"%d dropped (real run contradicted the model - see log). "
				"Fastest within-tolerance line first, rest by real miss. Log: %s",
				static_cast<int>(g_search.results.size()), hit_tol, hit_near,
				g_search.diverged,
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
		Vector hull_mn, hull_mx;
		const bool hull_measured = Prediction::PlayerHull(&hull_mn, &hull_mx);
		out << FmtStr("== sourceTAS search diagnostics ==\n"
			"map=%s  %s%s  tol=%.2f  jobs=%d/%d  results=%d (of %d distinct post-dedupe)\n"
			"start: pos=(%.2f %.2f %.2f) vel=(%.2f %.2f %.2f) yaw=%.3f jump=%d\n"
			"target: pos=(%.2f %.2f %.2f) yaw=%.3f\n"
			"model in-use: dt=%.6f cap=%.1f amt=%.1f grav=%.1f jump_vz_input=%.2f\n"
			"colliders=%d corridor brushes (target brush first)\n"
			"outcomes: infeasible=%d  no_eval=%d  floor(>collect)=%d  tick_align=%d  violent_board=%d  real_diverged=%d  deflect_target_brush=%d  deflect_world=%d  cap_dropped=%d  corridor_dropped=%d\n"
			"closest positional miss: %.3f u   max_bump=%.1f   entity_brushes_skipped=%d   hull=(%.0f %.0f %.0f)-(%.0f %.0f %.0f)%s\n\n",
			BspWorld::GetStatus().map,
			g_search.ride ? FmtStr("RIDE 1 strafe (%d steering phases)", g_search.strafes).c_str()
			              : FmtStr("strafes=%d", g_search.strafes).c_str(),
			g_search.ride ? "" : (g_search.opt ? " OPTIMAL-BIAS (rates are per-phase biases; the blend tail lands the yaw exactly)" : ""),
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
			g_search.reject_retime, g_search.cnt_bump, g_search.diverged,
			g_search.cnt_edge, g_search.early_hits, g_search.cap_dropped,
			g_search.corridor_dropped,
			g_search.reject_best < 1e8f ? g_search.reject_best : -1.f,
			g_search.max_bump, g_search.entity_brushes,
			hull_mn.X, hull_mn.Y, hull_mn.Z, hull_mx.X, hull_mx.Y, hull_mx.Z,
			hull_measured ? " (engine)" : " (DEFAULT - never read from a live player)");

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
				"wrap=%+d speed=%.1f arr=%.1f vz=%+.0f bump=%dt/%.2f %s rates=%+.4f,%+.4f,%+.4f,%+.4f pass=(%.2f %.2f %.2f)\n",
				i, s.exact ? " EXACT" : " near ", s.pass_err, s.real_err, s.ticks, s.frac, s.side0,
				s.splits[0], s.splits[1], s.splits[2], s.wrap, s.speed,
				s.arr_heading, s.arr_vz,
				s.bump_ticks, s.bump_loss,
				s.opt ? FmtStr("OPT/bl%d", s.blend).c_str() : "CONST",
				s.rates[0], s.rates[1], s.rates[2], s.rates[3],
				s.pass_pos.X, s.pass_pos.Y, s.pass_pos.Z);
		}
		if (!g_search.dropped_diverged.empty()) {
			out << FmtStr("\nDROPPED - real run contradicted the model (%d lines; the "
				"real engine is the arbiter, these cannot be flown as promised):\n",
				static_cast<int>(g_search.dropped_diverged.size()));
			const int ndrop = (std::min)(
				static_cast<int>(g_search.dropped_diverged.size()), 40);
			for (int i = 0; i < ndrop; ++i) {
				const RouteSolution& s = g_search.dropped_diverged[i];
				out << FmtStr("[drp %02d] err=%.4f real=%.3f N=%d real_tick=%d real_yaw=%.2f "
					"side0=%+d splits=%d,%d,%d speed=%.1f %s rates=%+.4f,%+.4f,%+.4f,%+.4f\n",
					i, s.pass_err, s.real_err < 1e8f ? s.real_err : -1.f,
					s.ticks, s.real_tick, s.real_yaw_err, s.side0,
					s.splits[0], s.splits[1], s.splits[2], s.speed,
					s.opt ? FmtStr("OPT/bl%d", s.blend).c_str() : "CONST",
					s.rates[0], s.rates[1], s.rates[2], s.rates[3]);
			}
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
		BatchAppendf("[sol %03d] N=%d frac=%.3f side0=%+d nsplits=%d splits=%d,%d,%d wrap=%+d speed=%.1f %s\n"
		             "  model: rates=%+.4f,%+.4f,%+.4f,%+.4f  pass_err=%.4f  pass=(%.2f %.2f %.2f)\n",
			g_batch.idx, sol.ticks, sol.frac, sol.side0, sol.nsplits,
			sol.splits[0], sol.splits[1], sol.splits[2], sol.wrap, sol.speed,
			sol.opt ? FmtStr("OPT/bl%d", sol.blend).c_str() : "CONST",
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
			g_search.opt ? " OPTIMAL-BIAS (rates are per-phase biases; the blend tail lands the yaw exactly)" : "",
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
		suffix.ovr.clear();
		for (const InputOvr& o : prefix.ovr)
			if (o.tick >= local)
				suffix.ovr.push_back({ o.tick - local, o.mask, o.value });
		prefix.ovr.erase(
			std::remove_if(prefix.ovr.begin(), prefix.ovr.end(),
				[local](const InputOvr& o) { return o.tick >= local; }),
			prefix.ovr.end());
	}

	// TOTAL-HEADING bias (kind 0) spreads its turn over the segment's tick
	// count, so cutting a segment shorter re-spreads the SAME total over
	// fewer ticks and the path behind the playhead visibly jumps. Re-fit both
	// halves from the SIMULATED headings instead: the prefix keeps exactly
	// the heading change it actually achieved by the split tick, and the
	// suffix carries the remainder to the original target heading. Nothing to
	// do for kinds 1/2 - their per-tick expression never sees the length.
	void RefitTotalHeadingSplit(GenParams& prefix, GenParams* suffix,
	                            int seg_index, int local) {
		if (prefix.yaw_mode != 2 || prefix.bias_kind != 0)
			return;
		if (fabsf(prefix.yaw_rate) < 1e-4f)
			return;                       // pure optimal: no turn to divide
		const int start = (seg_index < static_cast<int>(g_starts.size()))
			? g_starts[seg_index] : -1;
		if (start < 0 || local < 1)
			return;
		const int split = start + local;   // global tick index of the cut
		float h_entry = 0.f, h_split = 0.f;
		if (g_valid && !g_dirty && split >= 1
			&& split <= static_cast<int>(g_states.size())) {
			// Heading ENTERING the segment (state before its first tick) and
			// heading at the cut - the real numbers the provider fed on.
			const Vector& v_entry = (start >= 1)
				? g_states[start - 1].velocity : g_anchor.velocity;
			h_entry = HeadingDeg(v_entry, 0.f);
			h_split = HeadingDeg(g_states[split - 1].velocity, h_entry);
		} else {
			// No usable sim: fall back to the proportional split, which is
			// what the even spread intends anyway.
			const int total = (prefix.ticks > 0) ? prefix.ticks : 1;
			const float frac = Clampf(static_cast<float>(local)
				/ static_cast<float>(total), 0.f, 1.f);
			const float done = prefix.yaw_rate * frac;
			if (suffix) suffix->yaw_rate = prefix.yaw_rate - done;
			prefix.yaw_rate = done;
			return;
		}
		const float dir = static_cast<float>(prefix.opt_dir);
		const float done = dir * NormYaw(h_split - h_entry);         // achieved
		const float target_h = h_entry + dir * prefix.yaw_rate;      // original goal
		if (suffix)
			suffix->yaw_rate = dir * NormYaw(target_h - h_split);    // remainder
		prefix.yaw_rate = done;
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
			RefitTotalHeadingSplit(prefix.gen, &suffix.gen, k, local);
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
		// The prefix must keep the path it already drew - re-fit its
		// total-heading target to what the sim actually reached by `local`
		// (the suffix carries the user's NEW values, so it isn't re-fitted).
		RefitTotalHeadingSplit(seg.gen, nullptr, k, local);
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

	// Documents\sourceTAS\solver: ground-truth exports and solver artifacts.
	// The offline SolverLab harness reads from here (Map Solve tab workflow).
	std::string SolverDir() {
		char documents[MAX_PATH];
		if (FAILED(SHGetFolderPathA(nullptr, CSIDL_PERSONAL, nullptr, SHGFP_TYPE_CURRENT, documents)))
			return {};
		std::string base = std::string(documents) + "\\sourceTAS";
		CreateDirectoryA(base.c_str(), nullptr);
		std::string dir = base + "\\solver";
		CreateDirectoryA(dir.c_str(), nullptr);
		return dir;
	}

	// Global UI preferences (not per-project), one "key value" line each in
	// Documents\sourceTAS\ui.cfg: the menu opacity plus every Rendering-tab
	// option. Unknown keys are ignored so old and new builds can share the
	// file; a legacy single-float file still loads as the opacity.
	std::string SettingsPath() {
		char documents[MAX_PATH];
		if (FAILED(SHGetFolderPathA(nullptr, CSIDL_PERSONAL, nullptr, SHGFP_TYPE_CURRENT, documents)))
			return {};
		std::string base = std::string(documents) + "\\sourceTAS";
		CreateDirectoryA(base.c_str(), nullptr);
		return base + "\\ui.cfg";
	}

	// Every persisted option: X(key, lvalue). Serialize, parse, and the
	// defaults snapshot all derive from this one list.
	#define STAS_UI_PREFS(X) \
		X("opacity",       Theme::MenuOpacity()) \
		X("hotkeys_armed", RecordPanel::HotkeysArmed()) \
		X("freecam_speed", g_fc_speed) \
		X("freecam_smooth", g_fc_smooth) \
		X("test_marker",   WorldDraw::draw_test_marker) \
		X("player_box",    WorldDraw::draw_player_box) \
		X("feet_marker",   WorldDraw::draw_player_marker) \
		X("hull_alpha",    WorldDraw::player_box_alpha) \
		X("overlay_life",  WorldDraw::overlay_life_scale) \
		X("tag_end_speed", WorldDraw::tag_seg_end_speed) \
		X("tag_cur_speed", WorldDraw::tag_cursor_speed) \
		X("tag_end_eff",   WorldDraw::tag_seg_end_eff) \
		X("tag_end_time",  WorldDraw::tag_seg_end_time) \
		X("tag_cur_time",  WorldDraw::tag_cursor_time) \
		X("pause_busy",    WorldDraw::pause_draw_busy) \
		X("pred_path",     WorldDraw::draw_prediction) \
		X("pred_live",     WorldDraw::pred_live_input) \
		X("pred_bhop",     WorldDraw::pred_autobhop) \
		X("pred_ticks",    WorldDraw::pred_ticks) \
		X("pred_fwd",      WorldDraw::pred_forwardmove) \
		X("pred_side",     WorldDraw::pred_sidemove) \
		X("pred_jump",     WorldDraw::pred_jump) \
		X("pred_duck",     WorldDraw::pred_duck) \
		X("trail0",        WorldDraw::corner_trails[0]) \
		X("trail1",        WorldDraw::corner_trails[1]) \
		X("trail2",        WorldDraw::corner_trails[2]) \
		X("trail3",        WorldDraw::corner_trails[3]) \
		X("ghost_hull",    g_show_hull) \
		X("replay_hud",    WorldDraw::show_replay_hud) \
		X("coast_line",    g_coast_line) \
		X("coast_ticks",   g_coast_ticks) \
		X("ramp_eff",      g_ramp_eff) \
		X("dur_max",       g_dur_max) \
		X("min_eff",       g_min_eff) \
		X("bsp_wire",      BspWorld::draw_wireframe) \
		X("bsp_markers",   BspWorld::show_markers) \
		X("bsp_radius",    BspWorld::draw_radius) \
		X("bsp_budget",    BspWorld::line_budget) \
		X("bsp_solid",     BspWorld::show_solid) \
		X("bsp_clip",      BspWorld::show_playerclip) \
		X("bsp_ladder",    BspWorld::show_ladder) \
		X("triggers_sim",  BspWorld::apply_triggers) \
		X("triggers_show", BspWorld::show_triggers) \
		X("trig_tp",       BspWorld::show_trig_tp) \
		X("trig_push",     BspWorld::show_trig_push) \
		X("trig_grav",     BspWorld::show_trig_grav) \
		X("trig_links",    BspWorld::show_trig_links) \
		X("trig_events",   WorldDraw::show_trig_events) \
		X("autohop",       RecordPanel::AutohopEnabled())

	void PrefPut(std::string& out, const char* key, bool v) {
		out += key; out += v ? " 1\n" : " 0\n";
	}
	void PrefPut(std::string& out, const char* key, int v) {
		char b[64]; Sfmt(b, "%s %d\n", key, v); out += b;
	}
	void PrefPut(std::string& out, const char* key, float v) {
		char b[64]; Sfmt(b, "%s %.6g\n", key, v); out += b;
	}
	void PrefSet(bool& dst, double v)  { dst = v != 0.0; }
	void PrefSet(int& dst, double v)   { dst = static_cast<int>(v); }
	void PrefSet(float& dst, double v) { dst = static_cast<float>(v); }

	std::string SerializePrefs() {
		std::string out;
		#define X(key, lval) PrefPut(out, key, lval);
		STAS_UI_PREFS(X)
		#undef X
		return out;
	}

	void ApplyPrefLine(const std::string& key, double val) {
		#define X(k, lval) if (key == k) { PrefSet(lval, val); return; }
		STAS_UI_PREFS(X)
		#undef X
	}

	std::string g_prefs_defaults;   // serialized compile-time defaults
	std::string g_prefs_saved;      // last state written to (or read from) disk

	void SaveUiSettings() {
		const std::string p = SettingsPath();
		if (p.empty())
			return;
		g_prefs_saved = SerializePrefs();
		std::ofstream f(p, std::ios::trunc);
		if (f)
			f << g_prefs_saved;
	}

	void LoadUiSettings() {
		static bool s_loaded = false;
		if (s_loaded)
			return;
		s_loaded = true;
		// Snapshot the compile-time defaults BEFORE the file overrides them -
		// this is what the reset button restores.
		g_prefs_defaults = SerializePrefs();
		g_prefs_saved = g_prefs_defaults;
		const std::string p = SettingsPath();
		if (p.empty())
			return;
		std::ifstream f(p);
		if (!f)
			return;
		std::string key;
		double val = 0.0;
		bool any = false;
		while (f >> key) {
			// Legacy format: the whole file is one bare float (the opacity).
			char* end = nullptr;
			const double asnum = strtod(key.c_str(), &end);
			if (!any && end && *end == '\0') {
				if (asnum >= 0.05 && asnum <= 1.0)
					Theme::MenuOpacity() = static_cast<float>(asnum);
				break;
			}
			if (!(f >> val))
				break;
			ApplyPrefLine(key, val);
			any = true;
		}
		float& op = Theme::MenuOpacity();
		if (op < 0.05f) op = 0.05f;
		if (op > 1.f) op = 1.f;
		g_prefs_saved = SerializePrefs();
	}

	void ResetUiSettings() {
		std::istringstream in(g_prefs_defaults);
		std::string key;
		double val = 0.0;
		while (in >> key >> val)
			ApplyPrefLine(key, val);
		SaveUiSettings();
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

	bool ReadProjSummary(const std::string& path, ProjFileInfo& fi);   // below

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
			if (find.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
				continue;
			ProjFileInfo fi;
			fi.name = find.cFileName;
			fi.size = (static_cast<unsigned long long>(find.nFileSizeHigh) << 32)
				| find.nFileSizeLow;
			fi.mtime_raw = (static_cast<unsigned long long>(find.ftLastWriteTime.dwHighDateTime) << 32)
				| find.ftLastWriteTime.dwLowDateTime;
			FILETIME local = {};
			if (FileTimeToLocalFileTime(&find.ftLastWriteTime, &local))
				FileTimeToSystemTime(&local, &fi.mtime);
			ReadProjSummary(dir + "\\" + fi.name, fi);
			g_files.push_back(std::move(fi));
		} while (FindNextFileA(handle, &find));
		FindClose(handle);
		// Most recently saved first - the project being hunted for is almost
		// always a recent one.
		std::sort(g_files.begin(), g_files.end(),
			[](const ProjFileInfo& a, const ProjFileInfo& b) {
				return a.mtime_raw > b.mtime_raw;
			});
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
	// v12: gen gains bias_kind (total-heading vs per-tick optimal bias).
	// v13: solver gains want_opt/want_blend (the NEXT search's mode) so the
	//      checkbox no longer reinterprets the APPLIED schedule's numbers.
	// v14: solver gains ride (on-face ride solve).
	// v15: gen gains smooth_ticks + the prestrafe recipe fields.
	// v16: prestrafe gains ps_strafes (N alternating air strafes).
	// v17: per-tick JUMP overrides generalized to INPUT overrides
	//      (tick, mask, value) - old jump pairs load as OV_JUMP entries.
	// v18: prestrafe gains ps_jump_dist (ground phase ends by DISTANCE).
	// v19: prestrafe gains ps_goal_dist (point-and-shoot goal ray; ps_air_bias
	//      is reinterpreted as the ray's direction off the aim).
	// v20: prestrafe v3 recipes - gains ps_air_ticks + ps_ground_extra;
	//      ps_air_bias returns to a steering tilt; jump/goal dists legacy.
	// v21: prestrafe gains ps_strafe_ticks (fixed-length air strafes).
	// v22: SUMMARY HEADER between the version and the anchor - map name,
	//      total ticks, per-segment (kind, ticks) - so the Project pane can
	//      browse files without walking the versioned body. Display-only;
	//      the body stays the truth. Pre-v22 files load fine and just show
	//      name + date in the browser.
	// v23: gen gains the DYNAMIC pitch recipe fields (pitch_turn, pitch_rate,
	//      pitch_desc) appended at the gen block's tail; the summary header
	//      gains the anchor (valid flag + origin) after the segment list.
	constexpr uint32_t kProjVersion = 23;

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

	// Segment KIND codes for the v22 summary header, mirroring the segment
	// list's labels (display-only - the loader never branches on these).
	uint8_t SegKindCode(const EditSegment& s) {
		if (s.raw) return 1;
		if (s.is_solver) return s.solver.applied_ride ? 4 : (s.solver.solved ? 3 : 2);
		if (s.gen.prestrafe) return 5;
		if (s.gen.yaw_mode == 2) return s.gen.opt_dir > 0 ? 6 : 7;
		if (s.gen.yaw_mode == 3) return 8;
		if (s.gen.yaw_mode == 4) return 9;
		return 0;
	}
	const char* SegKindName(uint8_t code) {
		switch (code) {
		case 1: return "RAW";
		case 2: return "SOLVER (unsolved)";
		case 3: return "SOLVER";
		case 4: return "RIDE";
		case 5: return "PRESTRAFE";
		case 6: return "OPTIMAL left(A)";
		case 7: return "OPTIMAL right(D)";
		case 8: return "OPTIMAL straight";
		case 9: return "OPTIMAL straight-skew";
		default: return "MOVE";
		}
	}

	// Light header read for the Project pane's browser: magic + version +
	// (v22+) the summary block. Never walks the versioned body, so it stays
	// cheap enough to run for every file on a list refresh.
	bool ReadProjSummary(const std::string& path, ProjFileInfo& fi) {
		std::ifstream in(path, std::ios::binary);
		if (!in)
			return false;
		char magic[4];
		if (!in.read(magic, 4) || memcmp(magic, kProjMagic, 4) != 0 || !R(in, fi.version))
			return false;
		if (fi.version < 22)
			return false;                     // pre-summary file: name/date only
		if (!in.read(fi.map, sizeof(fi.map)))
			return false;
		fi.map[sizeof(fi.map) - 1] = 0;
		uint32_t ns = 0;
		if (!R(in, fi.total_ticks) || !R(in, ns) || ns > 4096)
			return false;
		fi.segs.clear();
		fi.segs.reserve(ns);
		for (uint32_t i = 0; i < ns; ++i) {
			uint8_t kind = 0;
			int32_t t = 0;
			if (!R(in, kind) || !R(in, t))
				return false;
			fi.segs.push_back({ kind, t });
		}
		if (fi.version >= 23) {
			uint8_t av = 0;
			if (!R(in, av) || !in.read(reinterpret_cast<char*>(fi.anchor), sizeof(float) * 3))
				return false;
			fi.anchor_valid = av != 0;
		}
		fi.has_info = true;
		return true;
	}

	bool WriteProjectFile(const std::string& path) {
		std::ofstream out(path, std::ios::binary | std::ios::trunc);
		if (!out)
			return false;

		out.write(kProjMagic, sizeof(kProjMagic));
		W(out, kProjVersion);

		// v22 summary header (see kProjVersion note): what the Project pane
		// shows without loading the file. Map name comes from the live engine;
		// saving outside a map stores "" and the browser shows "?".
		{
			char map[64] = {};
			BspWorld::CurrentMapName(map, sizeof(map));
			out.write(map, sizeof(map));
			uint32_t total = 0;
			for (const EditSegment& s : g_segs)
				total += static_cast<uint32_t>(s.Ticks());
			W(out, total);
			const uint32_t ns = static_cast<uint32_t>(g_segs.size());
			W(out, ns);
			for (const EditSegment& s : g_segs) {
				const uint8_t kind = SegKindCode(s);
				const int32_t t = s.Ticks();
				W(out, kind);
				W(out, t);
			}
			// v23: anchor world position in the summary too, so the browser
			// can say WHERE on the map a project starts.
			const uint8_t sav = g_anchor.valid ? 1 : 0;
			W(out, sav);
			out.write(reinterpret_cast<const char*>(&g_anchor.origin), sizeof(float) * 3);
		}

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
				W(out, g.bias_kind);   // v12
				W(out, g.smooth_ticks);      // v15
				const uint8_t pstrafe = g.prestrafe ? 1 : 0;
				const uint8_t pcend = g.ps_crouch_end ? 1 : 0;
				const uint8_t pcauto = g.ps_crouch_auto ? 1 : 0;
				W(out, pstrafe);             // v15
				W(out, g.ps_ground_ticks);
				W(out, g.ps_ground_turn);
				W(out, g.ps_ground_dir);
				W(out, g.ps_air_dir);
				W(out, g.ps_air_bias);
				W(out, pcend);
				W(out, pcauto);
				W(out, g.ps_crouch_lead);
					W(out, g.ps_strafes);        // v16
				W(out, g.ps_jump_dist);      // v18
				W(out, g.ps_goal_dist);      // v19
				W(out, g.ps_air_ticks);      // v20
				W(out, g.ps_ground_extra);   // v20
				W(out, g.ps_strafe_ticks);   // v21
				W(out, g.pitch_turn);        // v23
				W(out, g.pitch_rate);        // v23
				W(out, g.pitch_desc);        // v23

				const uint32_t novr = static_cast<uint32_t>(s.ovr.size());
				W(out, novr);
				for (const InputOvr& o : s.ovr) {
					const int32_t local = o.tick;
					const uint8_t mask = static_cast<uint8_t>(o.mask);
					const uint8_t val = static_cast<uint8_t>(o.value);
					W(out, local); W(out, mask); W(out, val);
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
					const uint8_t wopt = sd.want_opt ? 1 : 0;
					W(out, wopt);          // v13
					W(out, sd.want_blend); // v13
					const uint8_t ride = sd.ride ? 1 : 0;
					const uint8_t aride = sd.applied_ride ? 1 : 0;
					W(out, ride);          // v14
					W(out, aride);         // v14
				}
			}
		}

		return static_cast<bool>(out);
	}

	bool SaveProject() {
		const std::string dir = ProjectDir();
		std::string base = SanitizeName(g_name);
		if (dir.empty() || base.empty()) { g_status = "Invalid project name."; return false; }
		const bool ok = WriteProjectFile(dir + "\\" + base + ".tasproj");
		g_status = ok ? ("Saved " + base + ".tasproj.") : "Write failed.";
		RefreshFiles();
		return ok;
	}

	// Rolling crash backup: same format, fixed name, written quietly a couple
	// of seconds after edits settle. Load it like any project to recover.
	void SaveAutosave() {
		const std::string dir = ProjectDir();
		if (dir.empty())
			return;
		WriteProjectFile(dir + "\\_autosave.tasproj");
	}

	// Deserialized fields feed array indexing in the solver, the provider (on
	// the GAME thread) and the UI - a corrupt or hand-edited file must never
	// be able to index out of bounds. Clamp everything into its legal range.
	void SanitizeGen(GenParams& g) {
		if (g.ticks < 1) g.ticks = 1;
		if (g.ticks > 100000) g.ticks = 100000;
		if (g.yaw_mode < 1 || g.yaw_mode > 4) g.yaw_mode = 2;
		g.opt_dir = (g.opt_dir < 0) ? -1 : 1;
		if (g.opt_period < 1) g.opt_period = 1;
		if (g.opt_period > 128) g.opt_period = 128;
		if (g.bias_kind < 0 || g.bias_kind > 2) g.bias_kind = 2;
		if (!(g.yaw_rate == g.yaw_rate)) g.yaw_rate = 0.f;       // NaN guard
		if (g.yaw_rate < -180.f) g.yaw_rate = -180.f;
		if (g.yaw_rate > 180.f) g.yaw_rate = 180.f;
		if (g.pitch_mode < 0 || g.pitch_mode > 2) g.pitch_mode = PM_Const;
		if (g.jump_mode < 0 || g.jump_mode > 2) g.jump_mode = JM_None;
		if (!(g.pitch_turn == g.pitch_turn)) g.pitch_turn = 8.f;     // NaN
		if (g.pitch_turn < 0.f) g.pitch_turn = 0.f;
		if (g.pitch_turn > 20.f) g.pitch_turn = 20.f;
		if (!(g.pitch_rate == g.pitch_rate)) g.pitch_rate = 1.2f;    // NaN
		if (g.pitch_rate < 0.05f) g.pitch_rate = 0.05f;
		if (g.pitch_rate > 6.f) g.pitch_rate = 6.f;
		if (!(g.pitch_desc == g.pitch_desc)) g.pitch_desc = 0.35f;   // NaN
		if (g.pitch_desc < 0.f) g.pitch_desc = 0.f;
		if (g.pitch_desc > 1.5f) g.pitch_desc = 1.5f;
		if (g.smooth_ticks < 0) g.smooth_ticks = 0;
		if (g.smooth_ticks > 64) g.smooth_ticks = 64;
		if (g.ps_ground_ticks < 0) g.ps_ground_ticks = 0;
		if (g.ps_ground_ticks > 2000) g.ps_ground_ticks = 2000;
		// Windup is a magnitude now (the swing side comes from ps_ground_dir);
		// old projects' signed totals fold into the same range.
		if (!(g.ps_ground_turn == g.ps_ground_turn)) g.ps_ground_turn = 75.f;   // NaN
		g.ps_ground_turn = fabsf(g.ps_ground_turn);
		// v2 semantics: windup is the VELOCITY's offset from the goal heading
		// during the build - past ~85 deg the build stops closing on the goal
		// and the carve can never trigger.
		if (g.ps_ground_turn > 85.f) g.ps_ground_turn = 85.f;
		if (!(g.ps_jump_dist == g.ps_jump_dist)) g.ps_jump_dist = 500.f;   // NaN
		if (g.ps_jump_dist < 10.f) g.ps_jump_dist = 10.f;
		if (g.ps_jump_dist > 10000.f) g.ps_jump_dist = 10000.f;
		g.ps_ground_dir = (g.ps_ground_dir < 0) ? -1 : 1;
		g.ps_air_dir = (g.ps_air_dir < 0) ? -1 : 1;
		if (g.ps_strafes < 1) g.ps_strafes = 1;
		if (g.ps_strafes > 8) g.ps_strafes = 8;
		if (!(g.ps_air_bias == g.ps_air_bias)) g.ps_air_bias = 0.f;
		if (g.ps_air_bias < -20.f) g.ps_air_bias = -20.f;   // air steering tilt
		if (g.ps_air_bias > 20.f) g.ps_air_bias = 20.f;
		if (!(g.ps_goal_dist == g.ps_goal_dist)) g.ps_goal_dist = 900.f;   // legacy
		if (g.ps_air_ticks < 0) g.ps_air_ticks = 0;
		if (g.ps_air_ticks > 1000) g.ps_air_ticks = 1000;
		if (g.ps_strafe_ticks < 6) g.ps_strafe_ticks = 6;
		if (g.ps_strafe_ticks > 64) g.ps_strafe_ticks = 64;
		if (g.ps_ground_extra < 0) g.ps_ground_extra = 0;
		if (g.ps_ground_extra > 200) g.ps_ground_extra = 200;
		if (g.ps_crouch_lead < 0) g.ps_crouch_lead = 0;
		if (g.ps_crouch_lead > 200) g.ps_crouch_lead = 200;
		g.ps_crouch_cached = -1;   // runtime-only
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
		if (sd.want_blend < 1) sd.want_blend = 1;
		if (sd.want_blend > 24) sd.want_blend = 24;
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
		if (version >= 12) {
			if (!R(in, g.bias_kind))
				return false;
		} else {
			g.bias_kind = 1;   // pre-v12 optimal segments were per-tick bias
		}
		if (version >= 15) {
			uint8_t pstrafe = 0, pcend = 1, pcauto = 1;
			if (!R(in, g.smooth_ticks) || !R(in, pstrafe)
				|| !R(in, g.ps_ground_ticks) || !R(in, g.ps_ground_turn)
				|| !R(in, g.ps_ground_dir) || !R(in, g.ps_air_dir)
				|| !R(in, g.ps_air_bias) || !R(in, pcend) || !R(in, pcauto)
				|| !R(in, g.ps_crouch_lead))
				return false;
			g.prestrafe = pstrafe != 0;
			g.ps_crouch_end = pcend != 0;
			g.ps_crouch_auto = pcauto != 0;
			g.ps_crouch_cached = -1;   // runtime-only; recomputed on the next sim
			if (version >= 16 && !R(in, g.ps_strafes))
				return false;
			if (version >= 18 && !R(in, g.ps_jump_dist))
				return false;
			if (version >= 19 && !R(in, g.ps_goal_dist))
				return false;
			if (version >= 20
				&& (!R(in, g.ps_air_ticks) || !R(in, g.ps_ground_extra)))
				return false;
			if (version >= 21 && !R(in, g.ps_strafe_ticks))
				return false;
			// Pre-v20 files carried the RAY meaning in ps_air_bias (up to
			// +-180); as a steering tilt that would slam the air path - reset.
			if (version < 20 && fabsf(g.ps_air_bias) > 20.f)
				g.ps_air_bias = 0.f;
		}
		if (version >= 23
			&& (!R(in, g.pitch_turn) || !R(in, g.pitch_rate) || !R(in, g.pitch_desc)))
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

		// v22+: skip the display-only summary header; the body is the truth.
		if (version >= 22) {
			char map[64];
			uint32_t total = 0, ns = 0;
			if (!in.read(map, sizeof(map)) || !R(in, total) || !R(in, ns) || ns > 4096) {
				g_status = "Truncated project file.";
				return false;
			}
			for (uint32_t i = 0; i < ns; ++i) {
				uint8_t kind = 0;
				int32_t t = 0;
				if (!R(in, kind) || !R(in, t)) {
					g_status = "Truncated project file.";
					return false;
				}
			}
			if (version >= 23) {
				uint8_t sav = 0;
				float so[3] = {};
				if (!R(in, sav) || !in.read(reinterpret_cast<char*>(so), sizeof(so))) {
					g_status = "Truncated project file.";
					return false;
				}
			}
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
				if (version >= 17) {
					uint32_t novr = 0;
					if (!R(in, novr) || novr > 4096) return false;
					for (uint32_t o = 0; o < novr; ++o) {
						int32_t local = 0;
						uint8_t mask = 0, val = 0;
						if (!R(in, local) || !R(in, mask) || !R(in, val)) return false;
						s.ovr.push_back({ local, mask, val });
					}
				} else if (version >= 4) {
					// Old format: per-tick JUMP pairs -> OV_JUMP input overrides.
					uint32_t novr = 0;
					if (!R(in, novr) || novr > 4096) return false;
					for (uint32_t o = 0; o < novr; ++o) {
						int32_t local = 0;
						uint8_t val = 0;
						if (!R(in, local) || !R(in, val)) return false;
						s.ovr.push_back({ local, OV_JUMP, val ? OV_JUMP : 0 });
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
						if (version >= 13) {
							uint8_t wopt = 0;
							if (!R(in, wopt) || !R(in, sd.want_blend))
								return false;
							sd.want_opt = wopt != 0;
						} else {
							// One flag served both roles before v13.
							sd.want_opt = sd.opt;
							sd.want_blend = sd.blend;
						}
						if (version >= 14) {
							uint8_t ride = 0, aride = 0;
							if (!R(in, ride) || !R(in, aride))
								return false;
							sd.ride = ride != 0;
							sd.applied_ride = aride != 0;
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
		const std::string name = g_files[g_sel_file].name;
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
		// setpos_exact, full precision (see the Teleport button note). With
		// the freecam up, skip the setang half: playback replays each cmd's
		// viewangles exactly regardless, and setang would snap the freecam's
		// look (it reads the engine angles back every frame - third-party
		// observer means the run never drives the camera).
		if (g_freecam)
			Sfmt(cmd, "setpos_exact %.6f %.6f %.6f",
				g_anchor.origin.X, g_anchor.origin.Y, g_anchor.origin.Z);
		else
			Sfmt(cmd, "setpos_exact %.6f %.6f %.6f; setang %.2f %.2f 0",
				g_anchor.origin.X, g_anchor.origin.Y, g_anchor.origin.Z,
				g_anchor.pitch, g_anchor.yaw);
		if (engine)
			engine->ClientCmd_Unrestricted(cmd);
		if (g_tas.PlayEphemeral(std::move(run), 12)) {
			g_testplay = true;
			g_play_count = 0;
			g_play_max = 0.f;
			g_play_max_tick = -1;
			g_play_first_tick = -1;
			g_status = "Test playback from anchor (teleport needs sv_cheats 1). Emergency Stop aborts.";
		} else {
			g_status = "Can't test now - recorder busy.";
		}
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
	// Start/end stats for a segment's REAL simmed range: 2D + 3D speed, view
	// yaw/pitch, and average air efficiency. "Start" is the state ENTERING the
	// segment; "end" is the state after its LAST tick.
	// ---- FREECAM ------------------------------------------------------
	// The OverrideView hook feeds every frame's CViewSetup through here. The
	// struct layout is NEVER assumed: the origin/angles offsets are pinned
	// only after the struct's own floats MATCH engine truth (GetViewAngles +
	// the player's eye position) at the SAME offsets over several consecutive
	// frames. Until pinned - or if probing fails - nothing is ever written,
	// so even a wrong vtable slot degrades to "freecam unavailable".
	bool FcSafeCopy(void* dst, const void* src, size_t n) {
		__try {
			memcpy(dst, src, n);
			return true;
		} __except (EXCEPTION_EXECUTE_HANDLER) {
			return false;
		}
	}

	void FcLog(const char* line) {
		const std::string dir = CalibDir();
		if (dir.empty())
			return;
		std::ofstream f(dir + "\\freecam_probe.log", std::ios::app);
		if (f)
			f << line << "\n";
	}


	// Hook-context half: called by the single OverrideView wrapper with the
	// frame's CViewSetup. May only do guarded MEMCPYS - verified: pinned
	// writes the camera; unverified: stash a copy for Update's checker.
	void FreecamSample(int slot, void* arg) {
		if (slot != kFcSlot || !arg)
			return;
		// Camera write (guarded memcpys, the only thing this must do).
		if (g_freecam) {
			float org[3] = { g_fc_pos_vis.X, g_fc_pos_vis.Y, g_fc_pos_vis.Z };
			float ang[3] = { g_fc_pitch, g_fc_yaw, 0.f };
			unsigned char* base = static_cast<unsigned char*>(arg);
			FcSafeCopy(base + kFcOrgOff, org, sizeof(org));
			FcSafeCopy(base + kFcAngOff, ang, sizeof(ang));
		}
		// Diagnostic stash (until the sanity check confirms once).
		if (!g_fc_confirmed) {
			const LONG i = InterlockedIncrement(&g_fc_stash_idx) - 1;
			FcSafeCopy(g_fc_stash[((i % kFcRing) + kFcRing) % kFcRing], arg,
				sizeof(g_fc_stash[0]));
		}
	}

	// Background SANITY CHECK on the editor's frame path: does ANY of this
	// frame's stashed CViewSetups show the engine's own view at the measured
	// offsets? Purely informational - it flips a status line and stops
	// sampling once satisfied. It can never disable the camera (that gating
	// is exactly what locked freecam out).
	void FreecamVerifyPump() {
		if (g_fc_confirmed || g_freecam)
			return;   // while flying, the stash holds OUR values, not the engine's
		if (!engine || !engine->IsInGame())
			return;
		StartState st;
		if (!Prediction::CaptureStartState(st))
			return;
		g_fc_checks++;
		QAngle va(0.f, 0.f, 0.f);
		engine->GetViewAngles(va);
		for (int i = 0; i < kFcRing; ++i) {
			const float* fo = reinterpret_cast<const float*>(g_fc_stash[i] + kFcOrgOff);
			const float* fa = reinterpret_cast<const float*>(g_fc_stash[i] + kFcAngOff);
			const float dz = fo[2] - st.origin.Z;
			// Cross-frame tolerances: the stash is up to a frame older than
			// these engine reads, so allow a few degrees / units of drift.
			if (fabsf(fo[0] - st.origin.X) < 8.f
				&& fabsf(fo[1] - st.origin.Y) < 8.f
				&& dz > 16.f && dz < 100.f
				&& fabsf(NormYaw(fa[0] - va.X)) < 3.f
				&& fabsf(NormYaw(fa[1] - va.Y)) < 3.f
				&& fabsf(fa[2]) < 0.5f) {   // roll ~ 0
				g_fc_confirmed = true;
				char line[192];
				Sfmt(line, "freecam: layout confirmed (slot %d, origin+0x%X angles+0x%X, %d checks)",
					kFcSlot, kFcOrgOff, kFcAngOff, g_fc_checks);
				FcLog(line);
				return;
			}
		}
		if (g_fc_checks == 300) {
			const float* fo = reinterpret_cast<const float*>(g_fc_stash[0] + kFcOrgOff);
			const float* fa = reinterpret_cast<const float*>(g_fc_stash[0] + kFcAngOff);
			char line[256];
			Sfmt(line, "freecam: sanity check unconfirmed after %d checks (camera still "
				"active) - ring[0] org (%.1f %.1f %.1f) vs eye (%.1f %.1f %.1f+z), "
				"ang (%.1f %.1f %.1f) vs view (%.1f %.1f)",
				g_fc_checks, fo[0], fo[1], fo[2],
				st.origin.X, st.origin.Y, st.origin.Z,
				fa[0], fa[1], fa[2], va.X, va.Y);
			FcLog(line);
		}
	}

	// Fly the camera (called every frame from Update). LOOK comes straight
	// from the engine: the mouse still feeds the (frozen, input-blocked)
	// player's view angles through CInput's own pipeline - native
	// sensitivity/accel, and no cursor-recenter war (the v1 bug: CInput
	// polls + recenters the cursor itself, so a second recenter loop here
	// fed both sides constant fake deltas). MOVE is WASD/Space/Ctrl relative
	// to that view, Shift for speed; keyboard is captured by the input hook.
	void FreecamMove() {
		if (!g_freecam) {
			g_fc_qpc_last = 0;
			return;
		}
		// High-resolution dt: GetTickCount64 ticks every ~15.6 ms, which at
		// high fps made most frames dt = 0 and then a burst - the "updating
		// on tickrate" jerk. QPC is sub-microsecond.
		LARGE_INTEGER qn, qf;
		QueryPerformanceCounter(&qn);
		QueryPerformanceFrequency(&qf);
		float dt = 0.f;
		if (g_fc_qpc_last)
			dt = static_cast<float>(qn.QuadPart - g_fc_qpc_last)
				/ static_cast<float>(qf.QuadPart);
		g_fc_qpc_last = qn.QuadPart;
		if (dt > 0.1f) dt = 0.1f;
		if (dt < 0.f) dt = 0.f;
		// The engine's view angles ARE the freecam look.
		if (engine) {
			QAngle va(0.f, 0.f, 0.f);
			engine->GetViewAngles(va);
			g_fc_pitch = va.X;
			g_fc_yaw = va.Y;
		}
		if (ImGui::GetIO().MouseDrawCursor)   // menu open: hold position
			return;
		const float sp = g_fc_speed * ((GetAsyncKeyState(VK_SHIFT) & 0x8000) ? 3.f : 1.f);
		const float pr = g_fc_pitch * (kPi / 180.f);
		const float yr = g_fc_yaw * (kPi / 180.f);
		const float fwd[3] = { cosf(pr) * cosf(yr), cosf(pr) * sinf(yr), -sinf(pr) };
		const float right[3] = { sinf(yr), -cosf(yr), 0.f };
		float wish[3] = { 0.f, 0.f, 0.f };
		if (GetAsyncKeyState('W') & 0x8000) { wish[0] += fwd[0]; wish[1] += fwd[1]; wish[2] += fwd[2]; }
		if (GetAsyncKeyState('S') & 0x8000) { wish[0] -= fwd[0]; wish[1] -= fwd[1]; wish[2] -= fwd[2]; }
		if (GetAsyncKeyState('D') & 0x8000) { wish[0] += right[0]; wish[1] += right[1]; }
		if (GetAsyncKeyState('A') & 0x8000) { wish[0] -= right[0]; wish[1] -= right[1]; }
		if (GetAsyncKeyState(VK_SPACE) & 0x8000)   wish[2] += 1.f;
		if (GetAsyncKeyState(VK_CONTROL) & 0x8000) wish[2] -= 1.f;
		g_fc_pos.X += wish[0] * sp * dt;
		g_fc_pos.Y += wish[1] * sp * dt;
		g_fc_pos.Z += wish[2] * sp * dt;
		// Visual smoothing (toggle): ease the RENDERED position toward the
		// integrator with a ~55 ms time constant, framerate-independent.
		// Look stays raw (the engine's angles are already per-frame smooth,
		// and smoothing them would put lag between crosshair and picks).
		if (g_fc_smooth) {
			const float a = 1.f - expf(-dt * 18.f);
			g_fc_pos_vis.X += (g_fc_pos.X - g_fc_pos_vis.X) * a;
			g_fc_pos_vis.Y += (g_fc_pos.Y - g_fc_pos_vis.Y) * a;
			g_fc_pos_vis.Z += (g_fc_pos.Z - g_fc_pos_vis.Z) * a;
		} else {
			g_fc_pos_vis = g_fc_pos;
		}
	}

	// Copy the engine probe's MEASURED values into the model inputs. The probe
	// fills from real sim states (data, never guessed); with no data yet this
	// is a no-op that says so.
	void AdoptProbe() {
		if (g_meas_max_add <= 0.f && g_meas_grav_n <= 0) {
			g_status = "Engine probe has no data yet - run a sim with a solved solver segment first.";
			return;
		}
		if (g_meas_max_add > 45.f)
			g_air_accel = 150.f;
		if (g_meas_grav_n >= 8)
			g_gravity = g_meas_grav;
		if (g_meas_jump_vz > 100.f)
			g_jump_vz = g_meas_jump_vz;
		MarkDirty();
		g_status = "Adopted the probed engine values into the model settings.";
	}

	void DrawSegmentStats(int sel) {
		if (!g_valid || g_states.empty() || sel < 0 || sel >= static_cast<int>(g_starts.size()))
			return;
		const int b = g_starts[sel];
		int e = (sel + 1 < static_cast<int>(g_starts.size())) ? g_starts[sel + 1] : g_total;
		if (e > static_cast<int>(g_states.size())) e = static_cast<int>(g_states.size());
		if (e <= b)
			return;
		const int nf = static_cast<int>(g_frames.size());
		const Vector v0 = (b > 0 && b - 1 < static_cast<int>(g_states.size()))
			? g_states[b - 1].velocity : g_anchor.velocity;
		const Vector v1 = g_states[e - 1].velocity;
		auto sp2 = [](const Vector& v) { return sqrtf(v.X * v.X + v.Y * v.Y); };
		auto sp3 = [](const Vector& v) { return sqrtf(v.X * v.X + v.Y * v.Y + v.Z * v.Z); };
		const float y0 = (b < nf) ? g_frames[b].viewangles[1] : g_anchor.yaw;
		const float p0 = (b < nf) ? g_frames[b].viewangles[0] : g_anchor.pitch;
		const float y1 = (e - 1 < nf) ? g_frames[e - 1].viewangles[1] : y0;
		const float p1 = (e - 1 < nf) ? g_frames[e - 1].viewangles[0] : p0;
		float eff_sum = 0.f; int eff_n = 0;
		for (int i = b; i < e; ++i)
			if (i < static_cast<int>(g_maxgain.size()) && g_maxgain[i] > 0.001f) {
				eff_sum += g_eff[i]; eff_n++;
			}
		// World positions: state[i] is the state AFTER tick i, so the segment's
		// start position is the previous tick's origin (anchor for tick 0) -
		// the same convention v0 already uses for the start velocity.
		const Vector o0 = (b > 0 && b - 1 < static_cast<int>(g_states.size()))
			? g_states[b - 1].origin : g_anchor.origin;
		const Vector o1 = g_states[e - 1].origin;
		const Vector d = o1 - o0;
		ImGui::TextColored(ImVec4(0.7f, 0.85f, 1.f, 1.f),
			"Segment stats  (ticks %d - %d, %d ticks)", b, e - 1, e - b);
		ImGui::Text("  START   speed %.1f u/s  (3D %.1f)   yaw %.2f   pitch %.2f",
			sp2(v0), sp3(v0), y0, p0);
		ImGui::Text("  END     speed %.1f u/s  (3D %.1f)   yaw %.2f   pitch %.2f",
			sp2(v1), sp3(v1), y1, p1);
		ImGui::Text("  change  %+.1f u/s        turned %+.2f deg   avg air eff %.2f%% (%d air ticks)",
			sp2(v1) - sp2(v0), NormYaw(y1 - y0),
			eff_n ? (eff_sum / eff_n) * 100.f : 0.f, eff_n);
		ImGui::Text("  START pos  %.1f  %.1f  %.1f", o0.X, o0.Y, o0.Z);
		ImGui::Text("  END   pos  %.1f  %.1f  %.1f", o1.X, o1.Y, o1.Z);
		ImGui::Text("  moved   %.1f u horizontal   %+.1f u height   (3D %.1f)",
			sp2(d), d.Z, sp3(d));
	}

	void DrawRunTab() {
		// --- anchor + test play ------------------------------------------
		Theme::Heading("Anchor & test play");
		if (g_anchor.valid)
			ImGui::Text("Anchor: %.1f %.1f %.1f   yaw %.1f   vel %.0f u/s%s",
				g_anchor.origin.X, g_anchor.origin.Y, g_anchor.origin.Z,
				g_anchor.yaw, Speed2D(g_anchor.velocity), g_anchor.ducked ? "   (ducked)" : "");
		else
			ImGui::TextColored(Theme::Warning, "No anchor - capture one to simulate.");

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
			// Freecam up: position only - setang would snap the observer's look.
			if (g_freecam)
				Sfmt(cmd, "setpos_exact %.6f %.6f %.6f",
					g_anchor.origin.X, g_anchor.origin.Y, g_anchor.origin.Z);
			else
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

		// --- CURSOR (playhead controls left, tick stats right) ------------
		{
			Theme::Heading("Cursor");
			// Scrubbing anywhere auto-selects the segment under the playhead, so
			// the segment scrubber and the Selected-segment section always talk
			// about the tick being looked at.
			static int s_prev_cursor = -1;
			if (g_cursor != s_prev_cursor) {
				s_prev_cursor = g_cursor;
				const int cs = SegmentAtTick(g_cursor);
				if (cs >= 0)
					g_sel = cs;
			}
			ImGui::BeginChild("cursorctl", ImVec2(-286.f, 190), true,
				ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
			const int last = g_total > 0 ? g_total - 1 : 0;
			ImGui::TextDisabled("Run playhead");
			ImGui::PushItemWidth(-1);
			ImGui::SliderInt("##cursorall", &g_cursor, 0, last);
			ImGui::PopItemWidth();

			if (ImGui::SmallButton("|<")) g_cursor = 0;
			ImGui::SameLine(); if (ImGui::SmallButton("<seg")) TasEditor::StepSegment(-1);
			ImGui::SameLine(); if (ImGui::SmallButton("-10")) TasEditor::StepCursor(-10);
			ImGui::SameLine(); if (ImGui::SmallButton("-1")) TasEditor::StepCursor(-1);
			ImGui::SameLine(); if (ImGui::SmallButton("+1")) TasEditor::StepCursor(1);
			ImGui::SameLine(); if (ImGui::SmallButton("+10")) TasEditor::StepCursor(10);
			ImGui::SameLine(); if (ImGui::SmallButton("seg>")) TasEditor::StepSegment(1);
			ImGui::SameLine(); if (ImGui::SmallButton(">|")) g_cursor = last;

			// Selected-segment scrubber, two-way tied to the playhead: dragging
			// here moves the global playhead inside the segment, and moving the
			// playhead any other way is reflected straight back here.
			if (g_sel >= 0 && g_sel < static_cast<int>(g_starts.size())) {
				const int b = g_starts[g_sel];
				int e = (g_sel + 1 < static_cast<int>(g_starts.size())) ? g_starts[g_sel + 1] : g_total;
				if (e > b) {
					int local = g_cursor - b;
					if (local < 0) local = 0;
					if (local > e - b - 1) local = e - b - 1;
					ImGui::TextDisabled("Segment #%d scrub - tick %d / %d", g_sel, local, e - b);
					ImGui::PushItemWidth(-1);
					if (ImGui::SliderInt("##cursorseg", &local, 0, e - b - 1))
						g_cursor = b + local;
					ImGui::PopItemWidth();
				}
			}

			// Per-tick INPUT overrides at the playhead: pink = held this tick
			// (read from the composed plan), * = overridden. Clicking flips that
			// input on exactly this tick; clicking an overridden key removes the
			// override again. Segment edits clear the segment's overrides.
			const int seg = SegmentAtTick(g_cursor);
			if (seg >= 0 && seg < static_cast<int>(g_segs.size()) && !g_segs[seg].raw
				&& g_valid && g_cursor >= 0 && g_cursor < static_cast<int>(g_frames.size())
				&& g_cursor < Prediction::kMaxSimTicks) {
				EditSegment& ks = g_segs[seg];
				const int local = g_cursor - g_starts[seg];
				static const int bits[6] = { OV_W, OV_A, OV_S, OV_D, OV_JUMP, OV_DUCK };
				static const char* names[6] = { "W", "A", "S", "D", "Jump", "Crouch" };
				int ci = -1;
				for (int i = 0; i < static_cast<int>(ks.ovr.size()); ++i)
					if (ks.ovr[i].tick == local) { ci = i; break; }
				// Effective key state = natural keys with the overridden bits
				// replaced. KEY level, not move floats: adding D on a tick that
				// naturally holds A keeps BOTH pink (they null sidemove, exactly
				// like holding both keys in game).
				const uint8_t nat = g_nat_keys[g_cursor];
				const int eff = (ci >= 0)
					? ((nat & ~ks.ovr[ci].mask) | (ks.ovr[ci].value & ks.ovr[ci].mask))
					: nat;
				bool held[6];
				for (int i = 0; i < 6; ++i)
					held[i] = (eff & bits[i]) != 0;
				ImGui::TextDisabled("Tick inputs");
				Theme::Help("Every input's state at the playhead tick - pink = held. "
					"Click to flip that input on exactly this tick (marked *); click "
					"again to remove the override. Editing the segment's parameters "
					"clears its per-tick overrides.");
				for (int i = 0; i < 6; ++i) {
					if (i) ImGui::SameLine();
					const bool ovr_bit = ci >= 0 && (ks.ovr[ci].mask & bits[i]) != 0;
					char lbl[16];
					Sfmt(lbl, "%s%s##ov%d", names[i], ovr_bit ? "*" : "", i);
					if (held[i]) {
						ImGui::PushStyleColor(ImGuiCol_Button, Theme::Pink);
						ImGui::PushStyleColor(ImGuiCol_ButtonHovered, Theme::PinkHi);
						ImGui::PushStyleColor(ImGuiCol_ButtonActive, Theme::PinkHi);
						ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.f, 1.f, 1.f, 1.f));
					}
					const bool clicked = ImGui::SmallButton(lbl);
					if (held[i])
						ImGui::PopStyleColor(4);
					if (!clicked)
						continue;
					if (ovr_bit) {
						// Second click: back to what the segment does naturally.
						ks.ovr[ci].mask &= ~bits[i];
						ks.ovr[ci].value &= ~bits[i];
						if (ks.ovr[ci].mask == 0) {
							ks.ovr.erase(ks.ovr.begin() + ci);
							ci = -1;
						}
					} else {
						if (ci < 0) {
							ks.ovr.push_back({ local, 0, 0 });
							ci = static_cast<int>(ks.ovr.size()) - 1;
						}
						ks.ovr[ci].mask |= bits[i];
						if (!held[i]) ks.ovr[ci].value |= bits[i];
						else          ks.ovr[ci].value &= ~bits[i];
					}
					MarkDirty();
				}
			}
			ImGui::EndChild();
			PassWheel();

			// Stats for the tick the playhead sits on, in a narrow box so the
			// scrub sliders keep most of the row's width.
			ImGui::SameLine();
			ImGui::BeginChild("cursorstats", ImVec2(0, 190), true,
				ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
			if (g_valid && g_cursor >= 0 && g_cursor < static_cast<int>(g_states.size())) {
				const Prediction::SimState& st = g_states[g_cursor];
				const bool air = (st.flags & FL_ONGROUND) == 0;
				const float interval = Prediction::LastDiag().interval_per_tick > 0.f
					? Prediction::LastDiag().interval_per_tick : 0.015f;
				const float vz = st.velocity.Z;
				const float sp3 = sqrtf(st.velocity.X * st.velocity.X
					+ st.velocity.Y * st.velocity.Y + vz * vz);
				ImGui::TextColored(ImVec4(0.7f, 0.85f, 1.f, 1.f),
					"Cursor - tick %d, seg %d", g_cursor, seg);
				ImGui::Text("t %.3f s   %s%s", g_cursor * interval,
					air ? "air" : "ground", (st.flags & FL_DUCKING) ? " + duck" : "");
				ImGui::Text("pos %.1f %.1f %.1f", st.origin.X, st.origin.Y, st.origin.Z);
				ImGui::Text("speed %.1f   vz %+.0f", g_speed[g_cursor], vz);
				ImGui::Text("3D speed %.1f", sp3);
				ImGui::Text("yaw %.2f   pitch %.2f",
					g_frames[g_cursor].viewangles[1], g_frames[g_cursor].viewangles[0]);
				if (g_maxgain[g_cursor] > 0.001f) {
					ImGui::Text("gain %+.2f   max %+.2f", g_gain[g_cursor], g_maxgain[g_cursor]);
					ImGui::Text("eff %.2f%%", g_eff[g_cursor] * 100.f);
				} else {
					ImGui::Text("gain %+.2f (not air)", g_gain[g_cursor]);
				}
			} else {
				ImGui::TextDisabled("stats appear after a sim runs");
			}
			ImGui::EndChild();
			PassWheel();
		}

		// --- segment list (left) + selected-segment stats (right) ---------
		Theme::Heading("Segments");
		ImGui::TextDisabled("%d ticks total, %.2f s", g_total,
			g_total * (Prediction::LastDiag().interval_per_tick > 0.f ? Prediction::LastDiag().interval_per_tick : 0.015f));
		ImGui::BeginChild("segs", ImVec2(380, 172), true);
		for (int k = 0; k < static_cast<int>(g_segs.size()); ++k) {
			const EditSegment& s = g_segs[k];
			char kind[64];
			if (s.raw) {
				strcpy_s(kind, "RAW");
			} else if (s.is_solver) {
				Sfmt(kind, "%s ->#%d%s", s.solver.applied_ride ? "RIDE" : "SOLVER",
					s.solver.target, s.solver.solved ? "" : " (unsolved)");
			} else if (s.gen.prestrafe) {
				strcpy_s(kind, "PRESTRAFE");
			} else if (s.gen.yaw_mode == 2) {
				Sfmt(kind, "OPTIMAL %s", s.gen.opt_dir > 0 ? "left(A)" : "right(D)");
			} else if (s.gen.yaw_mode == 3) {
				strcpy_s(kind, "OPTIMAL straight");
			} else if (s.gen.yaw_mode == 4) {
				strcpy_s(kind, "OPTIMAL straight-skew");
			} else {
				const bool turning = fabsf(s.gen.yaw_rate) > 0.01f;
				Sfmt(kind, "MOVE %s%s%s%s%s",
					s.gen.key_w ? "W" : "", s.gen.key_a ? "A" : "",
					s.gen.key_s ? "S" : "", s.gen.key_d ? "D" : "",
					turning ? " turn" : "");
			}
			char label[256];
			Sfmt(label, "#%d  %s  %d ticks%s##seg%d", k, kind, s.Ticks(),
				(!s.raw && !s.ovr.empty()) ? "  [ovr]" : "", k);
			if (ImGui::Selectable(label, g_sel == k)) {
				g_sel = k;
				if (k < static_cast<int>(g_starts.size()))
					g_cursor = g_starts[k];
			}
		}
		const float seg_sy = ImGui::GetScrollY(), seg_sm = ImGui::GetScrollMaxY();
		ImGui::EndChild();
		static float s_segs_pw = 0.f;
		PassWheelAtEdge(&s_segs_pw, seg_sy, seg_sm);
		// Stats for the selected segment, in the free space to the right.
		ImGui::SameLine();
		ImGui::BeginChild("segstats", ImVec2(0, 172), true,
			ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
		if (g_valid && !g_states.empty())
			DrawSegmentStats(g_sel);
		else
			ImGui::TextDisabled("segment stats appear here after a sim runs");
		ImGui::EndChild();
		PassWheel();

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
		if (ImGui::Button("Duplicate") && g_sel >= 0 && g_sel < static_cast<int>(g_segs.size())) {
			EditSegment copy = g_segs[g_sel];
			g_segs.insert(g_segs.begin() + g_sel + 1, std::move(copy));
			g_sel++;
			MarkDirty();
		}
		ImGui::SameLine();
		if (ImGui::Button("Bake to raw") && g_sel >= 0 && g_sel < static_cast<int>(g_segs.size())
			&& g_valid && !g_dirty && g_sel < static_cast<int>(g_starts.size())) {
			// Freeze the selected segment's SIMMED frames as a raw (per-tick
			// editable) segment. Solver segments only once solved - unsolved
			// ones are just the hold-pattern placeholder.
			EditSegment& s = g_segs[g_sel];
			if (!s.raw && (!s.is_solver || s.solver.solved)) {
				const int b = g_starts[g_sel];
				int e = (g_sel + 1 < static_cast<int>(g_starts.size())) ? g_starts[g_sel + 1] : g_total;
				if (e > static_cast<int>(g_frames.size()))
					e = static_cast<int>(g_frames.size());
				if (e > b) {
					s.raw = true;
					s.is_solver = false;
					s.frames.assign(g_frames.begin() + b, g_frames.begin() + e);
					s.ovr.clear();   // baked frames already contain them
					g_status = "Baked segment to raw frames (per-tick editable).";
					MarkDirty();
				}
			}
		}
		Theme::Help("Freezes the selected segment's simmed ticks into RAW frames "
			"(per-tick editable). Needs a current sim; solver segments bake "
			"once solved.");
		ImGui::SameLine();
		if (Theme::Danger("Remove selected") && g_sel >= 0 && g_sel < static_cast<int>(g_segs.size())) {
			g_segs.erase(g_segs.begin() + g_sel);
			if (g_sel >= static_cast<int>(g_segs.size()))
				g_sel = static_cast<int>(g_segs.size()) - 1;
			MarkDirty();
		}
		if (ImGui::Button("Move up") && g_sel > 0
			&& g_sel < static_cast<int>(g_segs.size())) {
			std::swap(g_segs[g_sel - 1], g_segs[g_sel]);
			g_sel--;
			MarkDirty();
		}
		ImGui::SameLine();
		if (ImGui::Button("Move down") && g_sel >= 0
			&& g_sel + 1 < static_cast<int>(g_segs.size())) {
			std::swap(g_segs[g_sel], g_segs[g_sel + 1]);
			g_sel++;
			MarkDirty();
		}
		ImGui::SameLine();
		if (ImGui::Button("Undo"))
			DoUndo();
		ImGui::SameLine();
		if (ImGui::Button("Redo"))
			DoRedo();
		ImGui::SameLine();
		ImGui::TextDisabled("Ctrl+Z / Ctrl+Y");

		// --- selected segment parameters ----------------------------------
		if (g_sel >= 0 && g_sel < static_cast<int>(g_segs.size())) {
			Theme::Heading("Selected segment");
			if (g_segs[g_sel].is_solver && !g_segs[g_sel].raw) {
				EditSegment& s = g_segs[g_sel];
				// Target & mode on the left, search gates + probe on the right -
				// side by side so the solver block wastes less vertical space.
				ImGui::BeginChild("solvermode", ImVec2(-372.f, 240), true,
					ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
				ImGui::TextDisabled("Target & mode");
				Theme::Help("How the solver works: it searches alternating A/D strafe "
					"schedules whose path passes through the EXACT target point with the "
					"view yaw at that moment EXACTLY the target's set yaw. The pass tick "
					"is computed from gravity (never searched), the yaw equation is "
					"solved algebraically (the real sim replays the same view stream, so "
					"yaw cannot drift), and the remaining rates solve the 2D pass point. "
					"The drawn line is the REAL engine sim; its measured pass is shown "
					"below and auto-corrected against the target.");
				ImGui::PushItemWidth(260);
				int target = s.solver.target;
				if (ImGui::Combo("Board target", &target, TargetItemGetter, nullptr, BspWorld::TargetCount()))
					s.solver.target = target;
				// A ride is ALWAYS one continuous strafe - the strafe count is
				// a flight-only control, so hide it in ride mode (it used to
				// read "2", which looked like two strafes to the user; a ride
				// is one press with a single mid-course steering adjustment).
				if (!s.solver.ride) {
					int strafes = s.solver.strafes;
					if (ImGui::InputInt("Alternating strafes", &strafes))
						s.solver.strafes = strafes < 1 ? 1 : strafes > 4 ? 4 : strafes;
				}
				ImGui::PopItemWidth();
				ImGui::Checkbox("Ride solve: start on the target's ramp", &s.solver.ride);
				Theme::Help("For a segment that STARTS ON the target's ramp: ONE continuous "
					"strafe pressing into the ramp, riding the face to the exact point. "
					"The search allows a single mid-course steering change, and the "
					"solution family spreads the approach yaw and pass vz - the flick "
					"lead-in. Strafe-count/optimal settings are ignored; 'Jump at start' "
					"should be OFF.");
				// The checkbox picks what the NEXT search runs. The applied
				// schedule keeps ITS OWN mode (stamped at Apply) - toggling
				// here must never reinterpret applied numbers.
				ImGui::Checkbox("Optimal-rate strafes", &s.solver.want_opt);
				Theme::Help("Max gain: each phase rides the max-gain line +- a solved "
					"bias; the view BLENDS onto the target yaw over the last ticks "
					"(exact at the pass). Unchecked = constant-rate strafes; with the "
					"yaw constraint, 1-2 strafes only hit the point when a timing lines "
					"up - 3+ solves it structurally.");
				if (s.solver.solved && s.solver.opt != s.solver.want_opt)
					ImGui::TextColored(ImVec4(1.f, 0.75f, 0.3f, 1.f),
						"applied line stays %s until the next search+apply",
						s.solver.opt ? "OPTIMAL" : "CONSTANT-RATE");
				if (s.solver.want_opt) {
					ImGui::PushItemWidth(200);
					ImGui::SliderInt("Yaw blend window", &s.solver.want_blend, 1, 24);
					ImGui::PopItemWidth();
					Theme::Help("Ticks before the pass over which the view blends onto "
						"the target yaw.");
				}
				ImGui::Checkbox("Jump at start", &s.solver.start_jump);
				Theme::Help("The model is airborne-only - a grounded start needs this "
					"first-tick hop to leave the floor.");
				ImGui::SameLine();
				if (ImGui::Checkbox("Hold crouch", &s.gen.duck)) {
					s.ovr.clear();
					MarkDirty();
				}
				Theme::Help("Hold duck for the whole segment. Off by default; the engine "
					"sim carries the smaller hull and duck physics through the solve.");
				ImGui::EndChild();
				PassWheel();
				ImGui::SameLine();

				ImGui::BeginChild("solvergates", ImVec2(0, 240), true);
				ImGui::TextDisabled("Search gates & engine probe");
				Theme::Help("Advanced. The gates only decide which passes earn the "
					"'exact' tag - candidates are collected wider and real-measured "
					"either way. The probe reports engine facts measured from the last "
					"real sim.");
				ImGui::PushItemWidth(120);
				ImGui::InputFloat("Pass tolerance", &g_sol_pos_tol, 0.25f, 1.f, 2);
				Theme::Help("Units from the target within which a pass earns the "
					"'exact' tag. Filtering only - candidates are collected wider.");
				ImGui::InputFloat("Max bump loss", &g_sol_max_bump, 1.f, 5.f, 1);
				Theme::Help("A kiss of the ramp shortly before the target is allowed and "
					"slides into it - as long as the clip costs less speed than this "
					"(u/s).");
				ImGui::InputFloat("Collect radius", &g_sol_collect, 0.5f, 2.f, 1);
				Theme::Help("Everything passing within this many units is kept and "
					"real-measured; the tight gates above only decide the 'exact' tag. "
					"Data first, filtering later.");
				ImGui::PopItemWidth();
				// The engine probe: measured facts from the real sim's states.
				// NEVER silently used - Adopt copies them into the inputs.
				ImGui::PushTextWrapPos(0.f);
				if (g_meas_max_add > 0.f || g_meas_grav_n > 0) {
					float interval = Prediction::LastDiag().interval_per_tick;
					if (interval <= 0.f) interval = 0.015f;
					const float model_amt = g_air_accel * g_wishspeed * interval;
					ImGui::TextDisabled("probe: add/tick <= %.1f (model %.1f) | gravity %.0f [%d] | jump vz %.1f",
						g_meas_max_add, model_amt, g_meas_grav, g_meas_grav_n, g_meas_jump_vz);
					if (g_meas_max_add > model_amt + 2.f)
						ImGui::TextColored(ImVec4(1.f, 0.6f, 0.2f, 1.f),
							"the engine granted MORE accel than the model allows -> "
							"sv_airaccelerate here is too low for this server (~150 on surf)");
					if (ImGui::Button("Adopt probe into model settings"))
						AdoptProbe();
				} else {
					ImGui::TextDisabled("probe: no data yet - it fills in after any sim "
						"runs with a solved solver segment");
				}
				ImGui::PopTextWrapPos();
				const float sg_sy = ImGui::GetScrollY(), sg_sm = ImGui::GetScrollMaxY();
				ImGui::EndChild();
				static float s_gates_pw = 0.f;
				PassWheelAtEdge(&s_gates_pw, sg_sy, sg_sm);

				ImGui::Separator();
				if (Theme::Accent("Search solutions"))
					StartSearch(g_sel);
				ImGui::SameLine();
				ImGui::Checkbox("Deep search", &g_deep_search);
				Theme::Help("Tournament-full search config: ~+4 u/s mean, up to +35 on "
					"some scenarios. Slower search.");

				if (g_search.seg == g_sel) {
					if (g_search.active) {
						const float frac = g_search.jobs_total
							? static_cast<float>(g_search.jobs_done) / g_search.jobs_total : 0.f;
						char ov[64];
						Sfmt(ov, "searching  %d%%", static_cast<int>(frac * 100.f));
						ImGui::ProgressBar(frac, ImVec2(-1.f, 0.f), ov);
						ImGui::TextDisabled("%d passes collected (near-identical lines merge at the end)",
							g_results_n.load(std::memory_order_relaxed));
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
						ImGui::TextDisabled("   arrival: velocity heading %.1f deg, vz %+.0f u/s "
							"(the flick lead-in - sort tier 'Pass vz' spreads these)",
							sol.arr_heading, sol.arr_vz);
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
							// The one number every speed comparison should use:
							// the REAL 2D speed at the pass tick. A solution
							// row's `speed` is the model's claim for exactly
							// this state; the in-world END tag draws the plan's
							// final tick, which for a solver segment IS the
							// pass tick. Post-pass slide/overrun speeds are a
							// different (later) quantity - never compare those
							// against a row's pass speed.
							if (g_realpass.pass_speed >= 0.f)
								ImGui::TextDisabled("   real pass-tick speed: %.1f u/s "
									"(= what a row's `speed` claims and the end tag draws)",
									g_realpass.pass_speed);
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

				// Pitch still applies (static, trajectory-follow, or the human
				// recipe, exactly as in normal segments).
				bool ch = false;
				static const char* kPitchModesSv[] = { "Constant", "Follow trajectory", "Dynamic" };
				if (ChoiceRow("Pitch", &s.gen.pitch_mode, kPitchModesSv, 3)) {
					if (s.gen.pitch_mode == PM_Dynamic && s.gen.pitch_val == 0.f)
						s.gen.pitch_val = 20.f;   // measured straight-flight base
					ch = true;
				}
				if (s.gen.pitch_mode == PM_Const) {
					ch |= FloatRow("Pitch value", &s.gen.pitch_val, -89.f, 89.f, "%.1f deg", 0.1f, 5.f, 1);
				} else if (s.gen.pitch_mode == PM_Follow) {
					ch |= FloatRow("Pitch multiplier", &s.gen.pitch_mult, -2.f, 2.f, "%.2f", 0.05f, 0.2f);
				} else {
					ch |= FloatRow("Base pitch", &s.gen.pitch_val, -30.f, 85.f, "%.1f deg", 0.1f, 5.f, 1);
					ch |= FloatRow("Turn coupling", &s.gen.pitch_turn, 0.f, 20.f, "%.1f deg", 0.1f, 1.f, 1);
					ch |= FloatRow("Descent follow", &s.gen.pitch_desc, 0.f, 1.5f, "%.2f", 0.01f, 0.1f);
					ch |= FloatRow("Response", &s.gen.pitch_rate, 0.05f, 4.f, "%.2f deg/tick", 0.05f, 0.2f);
					Theme::Help("The dynamic pitch recipe, measured from your recorded "
						"runs - see the tooltips on a normal segment's pitch rows.");
				}
				if (ch)
					MarkDirty();

			} else if (!g_segs[g_sel].raw) {
				const GenParams before = g_segs[g_sel].gen;
				GenParams g = before;
				bool ch = false;

				ch |= ImGui::Checkbox("Prestrafe recipe", &g.prestrafe);
				Theme::Help("Point and shoot: aim a 2D ray from the segment start "
					"(direction + distance = where the run should LAND) and the "
					"recipe emits the whole input set - human-shaped ground carve "
					"aimed down the ray, jump at an auto-solved distance so the "
					"landing hits the ray tip, goal-homing optimal air strafes, "
					"crouch skim. The engine sims the real physics; only inputs "
					"are emitted.");
				if (g.prestrafe) {
					// Direction anchor: the recipes follow the segment AIM,
					// world-anchored by default so neighbor edits can't rotate
					// a tuned prestrafe.
					ch |= ImGui::Checkbox("  anchor aim to absolute yaw", &g.yaw_abs);
					Theme::Help("OFF (default): the recipe aims straight ahead from "
						"wherever the segment starts - the normal map/stage-start "
						"case. ON: pin a fixed world yaw instead, for prestrafes "
						"mid-run where upstream edits could swing the entry aim.");
					if (g.yaw_abs) {
						ImGui::SameLine();
						ImGui::PushItemWidth(kInputW);
						ch |= ImGui::InputFloat("##psyawabs", &g.yaw_start, 0.1f, 5.f, 1);
						ImGui::PopItemWidth();
					}

					ImGui::TextDisabled("Ground recipe");
					Theme::Help("W + strafe key builds to MAX ground speed in the "
						"minimum distance (the build ends the moment speed stops "
						"climbing), then the carve sweeps the velocity onto the aim "
						"as hard as the server's ground accel allows, and the jump "
						"fires the moment the heading reaches the aim - so changing "
						"sv_accelerate reshapes the carve, never the launch "
						"direction. Recipe, not a solver.");
					static const char* kSides[] = { "A (left)", "D (right)" };
					int gdir = (g.ps_ground_dir > 0) ? 0 : 1;
					if (ChoiceRow("  ground strafe side", &gdir, kSides, 2)) {
						g.ps_ground_dir = (gdir == 0) ? 1 : -1; ch = true;
					}
					ch |= FloatRow("  windup swing",
						&g.ps_ground_turn, 0.f, 85.f, "%.0f deg", 1.f, 10.f, 0);
					Theme::Help("How far the velocity runs off the aim during the "
						"build before the carve sweeps it back on. Bigger swing = "
						"longer, harder carve = more jump speed; measured human runs "
						"used 60-90. The carve always ends exactly on the aim.");
					ch |= IntRow("  extra build ticks", &g.ps_ground_extra, 0, 100);
					Theme::Help("Extends the speed-build past the max-speed point "
						"(0 = the minimum-distance default). Use it to place the "
						"jump deeper into a long startzone.");

					ImGui::TextDisabled("Air recipe");
					ch |= IntRow("  alternating strafes", &g.ps_strafes, 1, 8);
					ch |= IntRow("  ticks per strafe", &g.ps_strafe_ticks, 6, 64);
					Theme::Help("Fixed length of each air strafe (human first strafes "
						"measured 22-25 ticks). Fixed-length alternation means the "
						"segment's auto-duration can never re-shuffle the strafes; "
						"after the last one, the final side holds.");
					int adir = (g.ps_air_dir > 0) ? 0 : 1;
					if (ChoiceRow("  first strafe side", &adir, kSides, 2)) {
						g.ps_air_dir = (adir == 0) ? 1 : -1; ch = true;
					}
					ch |= FloatRow("  steer bias",
						&g.ps_air_bias, -20.f, 20.f, "%+.2f deg", 0.05f, 0.5f);
					Theme::Help("Constant tilt off the optimal line: + bends the air "
						"path right, - left, 0 = straight. Same feel as the straight "
						"modes' steering.");
					ch |= IntRow("  air duration", &g.ps_air_ticks, 0, 400);
					Theme::Help("Ticks of flight after the jump. 0 = AUTO: the segment "
						"ends ONE TICK PAST the landing (clearing the startzone edge). "
						"Set it manually for non-flat startzones and other edge cases - "
						"purely timing-based.");

					ImGui::TextDisabled("Crouch");
					ch |= ImGui::Checkbox("Hold crouch for the whole segment", &g.duck);
					ch |= ImGui::Checkbox("Crouch at the end", &g.ps_crouch_end);
					Theme::Help("Crouch right before the floor to skim past it at maximum "
						"speed (the last-moment startzone exit).");
					if (g.ps_crouch_end) {
						ch |= IntRow("  crouch lead", &g.ps_crouch_lead, 0, 60);
						Theme::Help("Ticks before the segment END to crouch - pure "
							"timing, no landing detection. The end sits one tick past "
							"the landing by default, so lead 2 = crouch the tick "
							"before landing. Adjust for edge cases.");
					}
					// ---- recipe telemetry: the "am I at max" numbers ----------
					// Ground and air speed limits with tick timing, plus the
					// landing vs the goal ray - everything measured from the
					// REAL sim, so a shortfall is visible immediately.
					if (g_valid && !g_dirty && g_sel < static_cast<int>(g_starts.size())) {
						const int start = g_starts[g_sel];
						int end = start + g_segs[g_sel].Ticks();
						if (end > static_cast<int>(g_states.size()))
							end = static_cast<int>(g_states.size());
						if (end > start + 2) {
							const Vector ro = (start > 0) ? g_states[start - 1].origin : g_anchor.origin;
							int jt = -1, lt = -1;
							float peak_ground = 0.f;
							for (int i = start; i < end; ++i) {
								const bool on = (g_states[i].flags & FL_ONGROUND) != 0;
								if (jt < 0) {
									const float s2 = Speed2D(g_states[i].velocity);
									if (s2 > peak_ground) peak_ground = s2;
									if (!on) jt = i;
								} else if (on) {
									lt = i;
									break;
								}
							}
							ImGui::TextDisabled("Recipe telemetry");
							if (jt >= 0) {
								ImGui::TextColored(ImVec4(0.6f, 0.9f, 1.f, 1.f),
									"ground %d ticks   jump speed %.1f   peak ground %.1f",
									jt - start, g_speed[jt], peak_ground);
								float esum = 0.f;
								int en = 0;
								const int ae = (lt >= 0) ? lt : end;
								for (int i = jt; i < ae; ++i)
									if (i < static_cast<int>(g_maxgain.size()) && g_maxgain[i] > 0.001f) {
										esum += g_eff[i];
										en++;
									}
								ImGui::TextColored(ImVec4(0.6f, 0.9f, 1.f, 1.f),
									"air %d ticks   avg eff %.2f%% of optimal gain",
									ae - jt, en ? (esum / en) * 100.f : 0.f);
								if (lt >= 0) {
									const float ldx = g_states[lt].origin.X - ro.X;
									const float ldy = g_states[lt].origin.Y - ro.Y;
									const float ld = sqrtf(ldx * ldx + ldy * ldy);
									ImGui::TextColored(ImVec4(0.6f, 0.9f, 1.f, 1.f),
										"lands tick %d   %.0f u out   ends %s",
										lt - start, ld,
										g.ps_air_ticks > 0 ? "at the air-duration slider"
										                   : "one tick past the landing");
								} else {
									ImGui::TextDisabled(g.ps_air_ticks > 0
										? "airborne at segment end (manual air duration)"
										: "sizing the segment to the recipe (auto)...");
								}
							} else {
								ImGui::TextDisabled("building ground speed (segment auto-sizing)...");
							}
						}
					}
					// Live 3D-speed readout at the crouch tick (the "475 total"):
					// the real simmed velocity magnitude including the downward vz.
					if (g_valid && !g_dirty && g_sel < static_cast<int>(g_starts.size())) {
						const int start = g_starts[g_sel];
						int ct = g.ticks - (g.ps_crouch_lead < 0 ? 0 : g.ps_crouch_lead);
						if (ct < 0) ct = g.ticks - 1;
						const int gi = start + ct;
						if (gi >= 0 && gi < static_cast<int>(g_states.size())) {
							const Vector& v = g_states[gi].velocity;
							const float sp3 = sqrtf(v.X * v.X + v.Y * v.Y + v.Z * v.Z);
							const float sp2 = sqrtf(v.X * v.X + v.Y * v.Y);
							ImGui::TextColored(ImVec4(0.6f, 0.9f, 1.f, 1.f),
								"at the crouch tick: 3D speed %.1f u/s  (horizontal %.1f, vz %+.0f)",
								sp3, sp2, v.Z);
						}
					}
				} else {

				SubHeading("Keys & duration");
				ImGui::Text("Held keys:");
				ImGui::SameLine(); ch |= ImGui::Checkbox("W", &g.key_w);
				ImGui::SameLine(); ch |= ImGui::Checkbox("A", &g.key_a);
				ImGui::SameLine(); ch |= ImGui::Checkbox("S", &g.key_s);
				ImGui::SameLine(); ch |= ImGui::Checkbox("D", &g.key_d);
				ImGui::SameLine(); ch |= ImGui::Checkbox("Crouch", &g.duck);
				// Jump is a held key too: checked = hold jump (JM_Hold). The
				// Auto-bhop box beside it upgrades that to press-on-landing
				// (JM_AutoBhop) - unchecking Jump clears both.
				{
					bool jump_held = g.jump_mode != JM_None;
					ImGui::SameLine();
					if (ImGui::Checkbox("Jump", &jump_held)) {
						g.jump_mode = jump_held ? JM_Hold : JM_None;
						ch = true;
					}
					if (jump_held) {
						bool autobhop = g.jump_mode == JM_AutoBhop;
						ImGui::SameLine();
						if (ImGui::Checkbox("Auto-bhop", &autobhop)) {
							g.jump_mode = autobhop ? JM_AutoBhop : JM_Hold;
							ch = true;
						}
						Theme::Help("Jump held = jump is pressed every tick. Auto-bhop = "
							"press only on ticks that start grounded, so it re-hops on "
							"each landing instead of holding.");
					}
				}
				ImGui::SameLine();
				ch |= ImGui::Checkbox("Auto A/D from turn direction", &g.auto_key);
				Theme::Help("Airborne + turning: the strafe key is picked from the turn "
					"direction for you (check A or D to override).");
				ImGui::SameLine();
				ch |= ImGui::Checkbox("Release W in air", &g.no_w_air);

				ch |= IntRow("Duration (ticks)", &g.ticks, 1,
					(g.ticks > g_dur_max) ? g.ticks : g_dur_max);

				SubHeading("Steering");
				// Two view modes now: TURN RATE (mode 1) and OPTIMAL (max gain).
				// Optimal covers strafing LEFT, RIGHT, or STRAIGHT (alternating
				// max-gain strafes whose net path holds its heading, mode 3) -
				// the direction selector picks which, so straight-sync lives in
				// optimal like every other max-gain path.
				if (g.yaw_mode < 1 || g.yaw_mode > 4)
					g.yaw_mode = 2;
				int vmode = (g.yaw_mode == 1) ? 0 : 1;
				static const char* kViewModes[] = { "Turn rate", "Optimal strafe" };
				if (ChoiceRow("View mode", &vmode, kViewModes, 2)) {
					g.yaw_mode = (vmode == 0) ? 1 : ((g.yaw_mode == 3 || g.yaw_mode == 4) ? g.yaw_mode : 2);
					ch = true;
				}
				if (vmode == 0) {
					ch |= FloatRow("View turn", &g.yaw_rate, -15.f, 15.f, "%+.2f deg/tick", 0.05f, 0.5f);
					Theme::Help("- turns left, + turns right, 0 holds the view still.");
				} else {
					// Optimal: Left / Right / Straight (view-tilt) / Straight (skew).
					int dir_idx = (g.yaw_mode == 4) ? 3 : (g.yaw_mode == 3) ? 2 : ((g.opt_dir > 0) ? 0 : 1);
					static const char* kDirs[] =
						{ "Left (A)", "Right (D)", "Straight (view)", "Straight (dwell)" };
					if (ChoiceRow("Direction", &dir_idx, kDirs, 4)) {
						if (dir_idx == 3) g.yaw_mode = 4;
						else if (dir_idx == 2) g.yaw_mode = 3;
						else { g.yaw_mode = 2; g.opt_dir = (dir_idx == 0) ? 1 : -1; }
						ch = true;
					}
					// One tooltip, text matching the selected direction mode.
					Theme::Help(
						g.yaw_mode == 3
							? "STRAIGHT (steer by view): alternating max-gain strafes - the "
							  "half-curls cancel so the mean path holds its heading; the bias "
							  "tilts the VIEW to steer, identical every tick. Splice/resize safe."
						: g.yaw_mode == 4
							? "STRAIGHT (steer by dwell): the view stays pure max-gain; one side "
							  "is held longer (base+skew ticks vs base-skew) and that asymmetric "
							  "dwell bends the mean path - no view tilt at all."
							: "OPTIMAL curl: yaw follows the velocity heading on the max-gain "
							  "line toward the chosen side; the bias below steers off that line.");
					if (g.yaw_mode == 3) {
						ch |= IntRow("Ticks per strafe side", &g.opt_period, 1, 64);
						ch |= FloatRow("Steering bias",
							&g.yaw_rate, -20.f, 20.f, "%+.2f deg", 0.05f, 0.5f);
						Theme::Help("+ steers right, - steers left, 0 = dead straight.");
					} else if (g.yaw_mode == 4) {
						ch |= IntRow("Base ticks per strafe side", &g.opt_period, 1, 64);
						ch |= FloatRow("Dwell skew",
							&g.yaw_rate, -32.f, 32.f, "%+.0f ticks", 1.f, 4.f, 0);
						Theme::Help("+ holds the right side longer, - the left side; the "
							"asymmetric dwell bends the mean path.");
					} else {
						// Stored values: 0=Total, 1=Per-tick, 2=Steering. Display the
						// DEFAULT (Steering) first without changing the stored codes.
						int bk_disp = (g.bias_kind == 2) ? 0 : (g.bias_kind == 0) ? 1 : 2;
						ImGui::PushItemWidth(210);
						if (ImGui::Combo("Steering type", &bk_disp,
							"Rate L/R\0Total heading (deg over segment)\0Per-tick (legacy)\0")) {
							g.bias_kind = (bk_disp == 0) ? 2 : (bk_disp == 1) ? 0 : 1;
							ch = true;
						}
						ImGui::PopItemWidth();
						// One tooltip on the bias row, text matching the bias kind;
						// the length-dependence WARNING stays visible in the panel.
						if (g.bias_kind == 2) {
							ch |= FloatRow("Steering bias",
								&g.yaw_rate, -20.f, 20.f, "%+.2f deg", 0.05f, 0.5f);
							Theme::Help("+ steers right, - left, 0 = pure optimal. A fixed "
								"angle off the max-gain line in screen sign, identical every "
								"tick - the tick COUNT never enters the geometry, so splitting "
								"or resizing leaves the existing path exactly where it was.");
						} else if (g.bias_kind == 0) {
							ch |= FloatRow("Total heading change",
								&g.yaw_rate, -180.f, 180.f, "%+.2f deg", 0.05f, 5.f);
							Theme::Help("Degrees turned into the strafe side over the whole "
								"segment, 0 = pure optimal. The turn is re-spread over the "
								"remaining ticks every tick - max speed for that curvature.");
							ImGui::TextColored(ImVec4(1.f, 0.75f, 0.3f, 1.f),
								"length-DEPENDENT: changing this segment's tick count re-spreads the turn "
								"and moves the path (splits re-fit both halves to the simulated headings).");
						} else {
							ch |= FloatRow("Turn bias",
								&g.yaw_rate, -20.f, 20.f, "%+.2f deg", 0.05f, 0.5f);
							Theme::Help("+ turns tighter, - wider, 0 = pure optimal. Bias "
								"trades speed (quadratic loss) for curvature (linear) - stays "
								"optimal for that curvature.");
						}
					}
				}
				ch |= ImGui::Checkbox("Absolute start yaw", &g.yaw_abs);
				if (g.yaw_abs) {
					ImGui::SameLine();
					ImGui::PushItemWidth(kInputW);
					ch |= ImGui::InputFloat("##yawstart", &g.yaw_start, 0.1f, 5.f, 1);
					ImGui::PopItemWidth();
				}

				SubHeading("Smoothing");
				// Boundary smoothing: blend the view across THIS segment's start
				// so the per-tick rate is continuous with the previous segment
				// (half the window in each). In optimal modes the wish stays
				// max-gain, so it costs almost no speed.
				ch |= IntRow("Boundary smoothing (ticks)", &g.smooth_ticks, 0, 32);
				Theme::Help("Straddles the boundary: half the window eases the view in the "
					"previous segment's tail, half in this segment's head, so the per-tick "
					"view rate is continuous through the cut instead of snapping. In optimal "
					"modes the wish stays max-gain, so it costs almost no speed.");

				}   // end else (normal generated-segment controls)

				// --- PITCH, its own section below smoothing - shared by the
				// normal AND prestrafe branches (the recipe covers a prestrafe's
				// ground+air arc too). DYNAMIC is the default: the human recipe
				// measured from the recorded runs.
				SubHeading("Pitch");
				static const char* kPitchModes[] = { "Constant", "Follow trajectory", "Dynamic" };
				if (ChoiceRow("##pitchmode", &g.pitch_mode, kPitchModes, 3)) {
					// Old segments switched onto DYNAMIC with an untouched 0
					// base: seed the measured straight-flight base (visible +
					// editable; new segments already default to it).
					if (g.pitch_mode == PM_Dynamic && g.pitch_val == 0.f)
						g.pitch_val = 20.f;
					ch = true;
				}
				Theme::Help("Dynamic is a recipe measured from your recorded runs: "
					"pitch rides at a base, dips further with the turn rate (the "
					"dominant human pattern - carving = looking down the ramp), sinks "
					"toward the fall line while descending, and never moves faster "
					"than the response limit. All sliders, no solving.");
				if (g.pitch_mode == PM_Const) {
					ch |= FloatRow("Pitch value", &g.pitch_val, -89.f, 89.f, "%.1f deg", 0.1f, 5.f, 1);
				} else if (g.pitch_mode == PM_Follow) {
					ch |= FloatRow("Pitch multiplier", &g.pitch_mult, -2.f, 2.f, "%.2f", 0.05f, 0.2f);
				} else {
					ch |= FloatRow("Base pitch", &g.pitch_val, -30.f, 85.f, "%.1f deg", 0.1f, 5.f, 1);
					Theme::Help("Pitch while flying straight, + = down. Your runs sat "
						"around +8..+27 when not turning.");
					ch |= FloatRow("Turn coupling", &g.pitch_turn, 0.f, 20.f, "%.1f deg", 0.1f, 1.f, 1);
					Theme::Help("Extra down-pitch per deg/tick of yaw rate. Your runs "
						"looked +20..+48 deeper while carving than while straight; "
						"typical strafe rates ~2-4 deg/tick make 8 a good middle.");
					ch |= FloatRow("Descent follow", &g.pitch_desc, 0.f, 1.5f, "%.2f", 0.01f, 0.1f);
					Theme::Help("How much of the falling trajectory's slope is added "
						"while descending (watching the landing). 0 = ignore the "
						"fall, 1 = look straight down the fall line. Rising never "
						"pitches up - the runs never did.");
					ch |= FloatRow("Response", &g.pitch_rate, 0.05f, 4.f, "%.2f deg/tick", 0.05f, 0.2f);
					Theme::Help("Rate limit toward the target. Your runs moved the "
						"pitch under ~1 deg/tick 90% of the time, ~1.7 at the 99th "
						"percentile.");
				}

				if (ch) {
					// Segment edits overwrite the per-tick input overrides (the
					// override is a tweak of ONE composed plan, not a promise
					// that survives re-planning).
					g_segs[g_sel].ovr.clear();
					// A prestrafe recipe is a whole-segment behavior - never
					// auto-split it; other gen edits keep the forward-only split.
					const int start = (g_sel < static_cast<int>(g_starts.size())) ? g_starts[g_sel] : 0;
					const int local = g_cursor - start;
					if (!g.prestrafe && !before.prestrafe
						&& local > 0 && local < g_segs[g_sel].Ticks() && SegmentAtTick(g_cursor) == g_sel)
						AutoSplitApply(g_sel, local, before, g);
					else
						g_segs[g_sel].gen = g;
					MarkDirty();
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

		ImGui::TextDisabled("Aim at a surface and Pick.");
		Theme::Help("Tag mode marks a face as surfable (gold outline); Target mode "
			"rests the player hull against the face exactly where the prediction "
			"would drop it, storing position + view angles - these are the solver's "
			"destinations. Bind the 'Editor: Pick At Crosshair' hotkey to pick with "
			"the menu closed.");

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
		Theme::Heading("Menu appearance");
		ImGui::SliderFloat("Menu opacity", &Theme::MenuOpacity(), 0.20f, 1.00f, "%.2f");
		Theme::Help("Backgrounds and buttons go see-through so the game shows "
			"through the panel; text and dropdown lists stay opaque so readings "
			"stay crisp. Saved to ui.cfg with every other option on this tab.");

		Theme::Heading("Player & world");
		ImGui::Checkbox("Test marker at world origin", &WorldDraw::draw_test_marker);
		ImGui::SameLine();
		ImGui::Checkbox("Player collision hull", &WorldDraw::draw_player_box);
		ImGui::SameLine();
		ImGui::Checkbox("Feet marker", &WorldDraw::draw_player_marker);
		IntRow("Hull alpha", &WorldDraw::player_box_alpha, 0, 160);
		Theme::Help("Fill opacity of the drawn hull; 0 draws wireframe only.");
		FloatRow("Overlay lifetime", &WorldDraw::overlay_life_scale, 0.5f, 4.f, "%.2f", 0.05f, 0.2f);
		Theme::Help("Multiplier of the frame time each overlay stays alive.");
		ImGui::Checkbox("Speed tag: segment end", &WorldDraw::tag_seg_end_speed);
		ImGui::SameLine();
		ImGui::Checkbox("Speed tag: playhead", &WorldDraw::tag_cursor_speed);
		ImGui::Checkbox("Tag: efficiency at segment end", &WorldDraw::tag_seg_end_eff);
		Theme::Help("Green in-world tag showing % of optimal gain at the segment end.");
		ImGui::Checkbox("Tag: time at segment end", &WorldDraw::tag_seg_end_time);
		ImGui::SameLine();
		ImGui::Checkbox("Tag: time at playhead", &WorldDraw::tag_cursor_time);
		ImGui::Checkbox("Pause in-world draws while the solver works (crash isolation)",
			&WorldDraw::pause_draw_busy);
		Theme::Help("Lines/hulls vanish during search+verify+correction. If the "
			"silent crashes stop with this ON, the engine overlay race is confirmed.");

		Theme::Heading("Live prediction",
			"Predicted path drawn from the live player state every frame.");
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

		Theme::Heading("Hull corner trails",
			"Trails of the hull's bottom corners, drawn along every path line.");
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

		Theme::Heading("Editor run line");
		ImGui::Checkbox("Ghost hull at playhead", &g_show_hull);
		ImGui::SameLine();
		ImGui::Checkbox("Replay HUD", &WorldDraw::show_replay_hud);
		FloatRow("Min efficiency", &g_min_eff, 0.90f, 1.f, "%.2f", 0.005f, 0.02f);
		Theme::Help("Line coloring only: ticks below this efficiency color the "
			"run line toward red.");
		IntRow("Duration slider max", &g_dur_max, 50, 1000);
		Theme::Help("Range cap of the segment Duration slider - most segments "
			"live well under 300 ticks, so the default keeps the slider's "
			"resolution useful. Segments already longer than the cap keep "
			"their length (the slider just tops out at it).");
		if (ImGui::Checkbox("Score ramp ticks", &g_ramp_eff))
			RecomputeDiag();   // pure math over the existing sim - instant
		Theme::Help("ON: board/slide ticks score like flight - the slope's free "
			"physics is subtracted and your strafing's air component is measured "
			"against the SAME air-accel optimum (AirAccelerate is identical on a "
			"ramp; the clip just eats whatever points into the surface). Feeds "
			"the efficiency coloring and the segment %. OFF: ramp ticks are "
			"unscoreable - neutral color, excluded from the %.");

		if (ImGui::Checkbox("Coast line (release-all off the run end)", &g_coast_line))
			MarkDirty();   // resim so the tail appears/clears now
		Theme::Help("Gray trajectory off the END of the last segment showing where "
			"you'd travel if you let go of everything - no yaw change, no keys, no "
			"crouch/jump - so it's easy to line up the next segment's start. "
			"Visual only; never part of the run. Bindable as 'Toggle Coast Line'.");
		if (g_coast_line) {
			if (IntRow("Coast length (ticks)", &g_coast_ticks, 8, 600))
				MarkDirty();
			ImGui::SameLine();
			float itick = Prediction::LastDiag().interval_per_tick;
			if (itick <= 0.f) itick = 0.015f;
			ImGui::TextDisabled("(%.1f s)", g_coast_ticks * itick);
		}

		Theme::Heading("World geometry",
			"Brush geometry parsed straight from the map's BSP file.");
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
			ImGui::Text("drawing %d brushes / %d lines",
				bs.drawn_brushes, bs.drawn_lines);
		} else if (bs.error[0]) {
			ImGui::PushTextWrapPos(0.f);
			ImGui::TextColored(ImVec4(1.f, 0.6f, 0.2f, 1.f), "BSP: %s", bs.error);
			ImGui::PopTextWrapPos();
		} else {
			ImGui::TextDisabled("BSP: not loaded (enable wireframes while in a map)");
		}

		Theme::Heading("Triggers",
			"Parsed from the map's entity lump - the same keyvalues the server "
			"spawns from. The sim fires them with the SDK's exact Touch "
			"semantics; the volume wires draw independently of the brush "
			"wireframe.");
		ImGui::Checkbox("Apply triggers in sim", &BspWorld::apply_triggers);
		Theme::Help("trigger_teleport, trigger_gravity and trigger_push fire "
			"inside the editor sim with the SDK's exact Touch semantics, so the "
			"line matches the real run through them - the client's own "
			"prediction never runs server triggers.");
		ImGui::SameLine();
		ImGui::Checkbox("Show trigger volumes", &BspWorld::show_triggers);
		Theme::Help("Master switch for the color-coded volume wires. They draw "
			"even with brush wireframes OFF - within the wire radius, on the "
			"shared line budget - so a route's triggers stay visible without "
			"paying for the whole map.");
		if (BspWorld::show_triggers) {
			ImGui::Checkbox("Teleport##trig", &BspWorld::show_trig_tp);
			ImGui::SameLine();
			ImGui::TextColored(ImVec4(1.f, 0.67f, 0.16f, 1.f), "gold");
			ImGui::SameLine();
			ImGui::Checkbox("Push##trig", &BspWorld::show_trig_push);
			ImGui::SameLine();
			ImGui::TextColored(ImVec4(0.35f, 1.f, 0.51f, 1.f), "green");
			ImGui::SameLine();
			ImGui::Checkbox("Gravity##trig", &BspWorld::show_trig_grav);
			ImGui::SameLine();
			ImGui::TextColored(ImVec4(0.31f, 0.67f, 1.f, 1.f), "blue");
			ImGui::SameLine();
			ImGui::Checkbox("Links##trig", &BspWorld::show_trig_links);
			Theme::Help("Teleport destination arrows (volume to landing box) and "
				"booster direction rays. A hidden type hides its links too.");
		}
		ImGui::Checkbox("Sim event markers", &WorldDraw::show_trig_events);
		Theme::Help("Small boxes on the run line where the last sim teleported "
			"(gold), boosted (green) or entered a gravity zone (blue).");
		ImGui::Text("volumes: %d teleport, %d push, %d gravity",
			BspWorld::TriggerTeleportCount(), BspWorld::TriggerPushCount(),
			BspWorld::TriggerGravityCount());

		Theme::Heading("Freecam");
		{
			bool fc = g_freecam;
			if (ImGui::Checkbox("Freecam", &fc))
				TasEditor::ToggleFreecam();
			Theme::Help("Detached inspection camera: close the menu, then WASD + "
				"mouse to fly, Space/Ctrl vertical, Shift for speed. Player input "
				"is blocked while active; opening the menu pauses flight. Picks "
				"aim from the CAMERA, so targets store the freecam's view angles. "
				"Bind 'Freecam Toggle' in the Record tab to flip it without the "
				"menu. The render-view layout is validated against engine data "
				"before the camera ever writes.");
			FloatRow("Fly speed", &g_fc_speed, 100.f, 3000.f, "%.0f u/s", 10.f, 100.f, 0);
			ImGui::Checkbox("Smooth motion", &g_fc_smooth);
			Theme::Help("Eases the rendered camera position (~55 ms) for buttery "
				"flight. Look stays raw so the crosshair and picks never lag. "
				"Off = the camera renders the integrator directly.");
			ImGui::TextDisabled("view write: slot %d, origin+0x%X, angles+0x%X%s",
				kFcSlot, kFcOrgOff, kFcAngOff,
				g_fc_confirmed ? " (confirmed)" : " (sanity check pending)");
		}

		Theme::Heading("Defaults");
		if (Theme::Danger("Reset rendering options to defaults")) {
			ResetUiSettings();
			g_status = "Rendering options reset to defaults.";
		}
		Theme::Help("Restores every option on this tab (including menu opacity) "
			"to its built-in default and saves.");
	}

	// Server-side setup + the strafe model's assumed values, on their own tab
	// (they are per-server configuration, not per-run editing).
	void DrawServerTab() {
		Theme::Heading("Server setup",
			"Listen-server cvars the physics depend on. Surf setup applies the "
			"standard surf server config in one click; Refresh reads the LIVE "
			"replicated values back into the model.");
		if (ImGui::Button("Surf setup")) {
			if (engine) {
				engine->ClientCmd_Unrestricted(
					"sv_cheats 1; sv_accelerate 10; sv_airaccelerate 150; "
					"sv_enablebunnyhopping 1");
				g_air_accel = 150.f;   // the model follows what was just set
				MarkDirty();
				g_status = "Sent sv_cheats 1, sv_accelerate 10, sv_airaccelerate 150, "
					"sv_enablebunnyhopping 1 - model updated.";
			}
		}
		Theme::Help("One click for a listen-server surf setup: sv_cheats 1, "
			"sv_accelerate 10, sv_airaccelerate 150, and sv_enablebunnyhopping 1 "
			"(the server cvar that removes the jump speed cap - PreventBunnyJumping "
			"- so hops keep speed). The strafe model's sv_airaccelerate follows "
			"immediately.");
		ImGui::SameLine();
		if (ImGui::Button("Refresh")) {
			float aa = 0.f, gr = 0.f;
			const bool ok_a = Cvars::GetFloat("sv_airaccelerate", &aa);
			const bool ok_g = Cvars::GetFloat("sv_gravity", &gr);
			if (ok_a || ok_g) {
				if (ok_a) g_air_accel = aa;
				if (ok_g) g_gravity = gr;
				MarkDirty();
				g_status = FmtStr("Read live server cvars: sv_airaccelerate %.1f, sv_gravity %.1f.",
					aa, gr);
			} else {
				// No validated live reads on this build - fall back to the
				// engine probe's MEASURED values (never guesses).
				AdoptProbe();
			}
		}
		Theme::Help("Reads the LIVE replicated server cvars (sv_airaccelerate, "
			"sv_gravity) straight out of client.dll - the values the server "
			"enforces right now, no solver run needed. The ConVar layout is "
			"discovered and validated from data at runtime and logged to "
			"cvar_probe.log; if validation fails, Refresh falls back to the "
			"engine probe's sim-measured values.");
		ImGui::TextDisabled("%s", Cvars::Status());

		Theme::Heading("Strafe model",
			"The values the editor's optimal-strafe math and the solver assume. "
			"Refresh above adopts the live server values; the engine probe "
			"measures them from real sim data.");
		bool ch = false;
		ImGui::PushItemWidth(120);
		ch |= ImGui::InputFloat("Air speed cap", &g_air_cap, 1.f, 10.f, 1);
		ch |= ImGui::InputFloat("sv_airaccelerate", &g_air_accel, 1.f, 10.f, 1);
		ch |= ImGui::InputFloat("Wish speed", &g_wishspeed, 10.f, 50.f, 0);
		ch |= ImGui::InputFloat("sv_gravity", &g_gravity, 10.f, 50.f, 0);
		// Model jump velocity, part of the model - not a search gate (it
		// used to hide in the solver's gates section, which read as one).
		ImGui::InputFloat("Jump vz (0 = engine formula)", &g_jump_vz, 1.f, 10.f, 1);
		Theme::Help("Vertical velocity the model gives a jump. 0 uses the engine "
			"formula sqrt(2*g*57). The engine probe measures the real value "
			"from sim data; Refresh adopts it.");
		ImGui::PopItemWidth();
		if (ch)
			MarkDirty();
		if (g_meas_max_add > 0.f || g_meas_grav_n > 0)
			ImGui::TextDisabled("probe: add/tick <= %.1f | gravity %.0f [%d samples] | jump vz %.1f",
				g_meas_max_add, g_meas_grav, g_meas_grav_n, g_meas_jump_vz);
	}

	// ------------------------------------------------------------- map solve --
	// The full-map solver era's home (design log: Docs\FullMapSolver.md).
	// Phase 0 wires the ground-truth loop: export the ENGINE-exact per-tick
	// states of the compiled run so the offline core (SolverLab.exe) can be
	// diffed against them tick by tick - fixes land where the data says, not
	// where theory guesses.
	std::string g_solver_export;   // last written path (session display only)
	// The tab's export-name field: governs BOTH the playback capture and the
	// sim export, so nothing ever lands as "untitled" again.
	char g_solver_export_name[64] = "";

	// Exports never overwrite and never leave identification to guesswork:
	// unique self-describing names + one manifest line per export in
	// solver\exports.log (timestamp, kind, map, source, ticks, path).
	std::string UniqueSolverCsv(const std::string& base) {
		const std::string dir = SolverDir();
		if (dir.empty())
			return {};
		std::string path = dir + "\\" + base + ".csv";
		for (int n = 2; GetFileAttributesA(path.c_str()) != INVALID_FILE_ATTRIBUTES; ++n)
			path = dir + "\\" + base + " (" + std::to_string(n) + ").csv";
		return path;
	}

	void AppendExportLog(const char* kind, const std::string& path,
	                     const char* source, int ticks) {
		const std::string dir = SolverDir();
		if (dir.empty())
			return;
		std::ofstream log(dir + "\\exports.log", std::ios::app);
		if (!log)
			return;
		SYSTEMTIME st;
		GetLocalTime(&st);
		char map[64] = "?";
		BspWorld::CurrentMapName(map, sizeof(map));
		char line[512];
		Sfmt(line, "%04d-%02d-%02d %02d:%02d:%02d | %-8s | map %-24s | %-28s | %5d ticks | %s\n",
			st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond,
			kind, map, source, ticks, path.c_str());
		log << line;
	}

	// Physics context for the offline solver. TRUST DISCIPLINE (learned the
	// hard way 2026-08-13): the generic ConVar reads returned DEFAULTS while
	// the live server demonstrably ran surf settings, and the hull netvar
	// read said 62 on a parity-proven-72 build - so this file carries ONLY
	// values with a trustworthy source: the measured tick interval and the
	// EDITOR'S STRAFE MODEL (user-visible on the Server tab; Surf setup /
	// Refresh / the engine probe update them). Everything else is left to
	// SolverLab's built-in defaults, which are engine-parity-proven.
	bool ExportServerParams() {
		const std::string dir = SolverDir();
		if (dir.empty())
			return false;
		const std::string path = dir + "\\server_params.cfg";
		std::ofstream out(path, std::ios::trunc);
		if (!out)
			return false;
		out << "# sourceTAS solver params - written by the Map Solve tab.\n";
		out << "# Only trustworthy sources: measured interval + the editor's\n";
		out << "# strafe model. Unlisted physics = SolverLab parity-proven\n";
		out << "# defaults. CLI flags override everything.\n";
		char line[160];
		const float itick = Prediction::LastDiag().interval_per_tick;
		Sfmt(line, "tickinterval %g  # %s\n", itick > 0.f ? itick : 0.015f,
			itick > 0.f ? "measured" : "default (no sim yet)");
		out << line;
		Sfmt(line, "gravity %g  # editor strafe model\n", g_gravity);
		out << line;
		Sfmt(line, "airaccelerate %g  # editor strafe model\n", g_air_accel);
		out << line;
		{
			// Weapon-dependent movement cap, READ from the newest real
			// command when available (knife 250, NO weapon 260 - the surf
			// standard); the editor model value is only the fallback.
			float ms;
			if (Prediction::LastRealMaxSpeed(&ms) && ms > 1.f)
				Sfmt(line, "maxspeed %g  # live m_flMaxSpeed (weapon)\n", ms);
			else
				Sfmt(line, "maxspeed %g  # editor strafe model wishspeed\n",
					g_wishspeed);
			out << line;
		}
		Sfmt(line, "air_speed_cap %g  # editor strafe model\n", g_air_cap);
		out << line;
		// LIVE server cvars (user directive 2026-08-14: read, never assume
		// - "I am acceleration 10 ingame"): exported whenever readable so
		// the solver runs the server's ACTUAL ground physics.
		float v;
		if (Cvars::GetFloat("sv_accelerate", &v)) {
			Sfmt(line, "accelerate %g  # live server cvar\n", v);
			out << line;
		}
		if (Cvars::GetFloat("sv_friction", &v)) {
			Sfmt(line, "friction %g  # live server cvar\n", v);
			out << line;
		}
		if (Cvars::GetFloat("sv_stopspeed", &v)) {
			// CONFLICT RESOLVED by disassembly (2026-08-15): the old "reads
			// 100 while behaving 75" was OUR read skipping ConVar::m_pParent
			// - the child keeps the registered default "100" while the
			// game's own SetValue(75.0f) (server.dll @2f6f6c) updates the
			// parent, and Friction (@20edb9) reads the PARENT live. Cvars::
			// GetFloat now hops the parent, so this value IS the movement
			// truth. Battery decks continue to arbitrate.
			Sfmt(line, "stopspeed %g  # live server cvar (parent-hopped)\n", v);
			out << line;
		}
		if (Cvars::GetFloat("sv_maxvelocity", &v)) {
			Sfmt(line, "maxvelocity %g  # live server cvar\n", v);
			out << line;
		}
		if (Cvars::GetFloat("sv_stepsize", &v)) {
			Sfmt(line, "stepsize %g  # live server cvar\n", v);
			out << line;
		}
		if (Cvars::GetFloat("sv_enablebunnyhopping", &v)) {
			Sfmt(line, "enablebunnyhopping %g  # live server cvar\n", v);
			out << line;
		}
		if (Cvars::GetFloat("sv_gravity", &v)) {
			Sfmt(line, "gravity %g  # live server cvar (overrides model line)\n", v);
			out << line;
		}
		if (Cvars::GetFloat("sv_airaccelerate", &v)) {
			Sfmt(line, "airaccelerate %g  # live server cvar (overrides model line)\n", v);
			out << line;
		}
		return static_cast<bool>(out);
	}

	bool ExportSimStates() {
		if (!g_valid || g_states.empty() || g_frames.empty()) {
			g_status = "Map Solve: no compiled sim to export - load a project first.";
			return false;
		}
		const std::string dir = SolverDir();
		if (dir.empty()) {
			g_status = "Map Solve: couldn't resolve Documents\\sourceTAS\\solver.";
			return false;
		}
		std::string base = SanitizeName(g_solver_export_name[0]
			? g_solver_export_name : g_name);
		if (base.empty())
			base = "project";
		const std::string path = UniqueSolverCsv("sim_" + base);
		std::ofstream out(path, std::ios::trunc);
		if (!out) {
			g_status = "Map Solve: couldn't write " + path;
			return false;
		}
		// Column layout shared with SolverLab's replay CSV, plus yaw so the
		// differ can verify input alignment before trusting a state delta.
		// Row 0 = the anchor as tick -1 (ground unknown there: -1).
		out << "tick,x,y,z,vx,vy,vz,speed2d,ground,ducked,buttons,yaw\n";
		char line[320];
		Sfmt(line, "-1,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.2f,-1,%d,0,%.4f\n",
			g_anchor.origin.X, g_anchor.origin.Y, g_anchor.origin.Z,
			g_anchor.velocity.X, g_anchor.velocity.Y, g_anchor.velocity.Z,
			sqrtf(g_anchor.velocity.X * g_anchor.velocity.X
				+ g_anchor.velocity.Y * g_anchor.velocity.Y),
			g_anchor.ducked ? 1 : 0, g_anchor.yaw);
		out << line;
		const int n = static_cast<int>(g_states.size() < g_frames.size()
			? g_states.size() : g_frames.size());
		for (int i = 0; i < n; ++i) {
			const Prediction::SimState& st = g_states[i];
			const Frame& fr = g_frames[i];
			Sfmt(line, "%d,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.2f,%d,%d,%d,%.4f\n",
				i, st.origin.X, st.origin.Y, st.origin.Z,
				st.velocity.X, st.velocity.Y, st.velocity.Z,
				sqrtf(st.velocity.X * st.velocity.X
					+ st.velocity.Y * st.velocity.Y),
				(st.flags & FL_ONGROUND) ? 1 : 0,
				(st.flags & FL_DUCKING) ? 1 : 0,
				fr.buttons, fr.viewangles[1]);
			out << line;
		}
		const bool ok = static_cast<bool>(out);
		const bool params_ok = ExportServerParams();
		g_solver_export = path;
		if (ok)
			AppendExportLog("sim", path, g_name, n);
		g_status = ok
			? FmtStr("Map Solve: exported %d engine ticks + anchor -> %s%s",
				n, path.c_str(),
				params_ok ? " (+ server_params.cfg)" : " (params write FAILED)")
			: "Map Solve: write failed mid-file.";
		return ok;
	}

	// ---------------- map solve: quick-iteration instruments (QOL) ----------
	// SOLVE ANCHOR: the tick-0 state the solver and the game AGREE on.
	// Capture writes it (plus the live server params) for the harness;
	// Teleport puts the player back on it before playback, so both worlds
	// start from the identical state - the start-position ambiguity that
	// muddied the first unseeded tests dies here.
	StartState g_solve_anchor;
	bool g_solve_anchor_valid = false;

	std::string SolveAnchorPath() {
		const std::string dir = SolverDir();
		return dir.empty() ? std::string() : dir + "\\solve_anchor.cfg";
	}

	bool WriteSolveAnchor() {
		StartState st;
		if (!Prediction::CaptureStartState(st)) {
			g_status = "Map Solve: can't capture the player - not in game?";
			return false;
		}
		const std::string path = SolveAnchorPath();
		if (path.empty())
			return false;
		std::ofstream out(path, std::ios::trunc);
		if (!out) {
			g_status = "Map Solve: couldn't write solve_anchor.cfg";
			return false;
		}
		char line[256];
		out << "# sourceTAS solve anchor - the agreed tick-0 state.\n";
		Sfmt(line, "origin %.6f %.6f %.6f\n",
			st.origin.X, st.origin.Y, st.origin.Z);
		out << line;
		Sfmt(line, "velocity %.6f %.6f %.6f\n",
			st.velocity.X, st.velocity.Y, st.velocity.Z);
		out << line;
		Sfmt(line, "pitch %.4f\nyaw %.4f\nducked %d\nstamina %.2f\n",
			st.pitch, st.yaw, st.ducked ? 1 : 0, st.stamina);
		out << line;
		g_solve_anchor = st;
		g_solve_anchor_valid = true;
		ExportServerParams();
		g_status = FmtStr("Map Solve: solve anchor (%.1f, %.1f, %.1f) + live "
			"server params written to solver\\.", st.origin.X, st.origin.Y,
			st.origin.Z);
		return true;
	}

	bool LoadSolveAnchorFile() {
		std::ifstream in(SolveAnchorPath());
		if (!in)
			return false;
		StartState st;
		bool have_origin = false;
		std::string line;
		while (std::getline(in, line)) {
			int d = 0;
			if (sscanf_s(line.c_str(), "origin %f %f %f",
				&st.origin.X, &st.origin.Y, &st.origin.Z) == 3)
				have_origin = true;
			else if (sscanf_s(line.c_str(), "velocity %f %f %f",
				&st.velocity.X, &st.velocity.Y, &st.velocity.Z) == 3) {}
			else if (sscanf_s(line.c_str(), "pitch %f", &st.pitch) == 1) {}
			else if (sscanf_s(line.c_str(), "yaw %f", &st.yaw) == 1) {}
			else if (sscanf_s(line.c_str(), "ducked %d", &d) == 1)
				st.ducked = d != 0;
			else if (sscanf_s(line.c_str(), "stamina %f", &st.stamina) == 1) {}
		}
		if (!have_origin)
			return false;
		st.valid = true;
		g_solve_anchor = st;
		g_solve_anchor_valid = true;
		return true;
	}

	void TeleportToSolveAnchor() {
		if (!g_solve_anchor_valid && !LoadSolveAnchorFile()) {
			g_status = "Map Solve: no solve anchor - capture one first.";
			return;
		}
		if (!engine)
			return;
		char cmd[192];
		Sfmt(cmd, "setpos_exact %.6f %.6f %.6f; setang %.2f %.2f 0",
			g_solve_anchor.origin.X, g_solve_anchor.origin.Y,
			g_solve_anchor.origin.Z, g_solve_anchor.pitch, g_solve_anchor.yaw);
		engine->ClientCmd_Unrestricted(cmd);
		g_status = "Map Solve: teleported to the solve anchor (needs sv_cheats 1).";
	}

	// REAL PLAYBACK CAPTURE: while armed, every playback tick's ACTUAL player
	// state (netvar basis, same as the divergence verdict) is recorded; on
	// playback end the rows export as the solver diff's ground-truth CSV.
	// This is the instrument that settles core-vs-engine claims with data.
	struct PlayCapRow {
		int tick;
		Vector pos, vel;
		int ground, ducked, buttons;
		float yaw;
	};
	std::vector<PlayCapRow> g_playcap;
	bool g_playcap_armed = false;
	bool g_playcap_active = false;
	int g_playcap_buttons = 0;
	float g_playcap_yaw = 0.f;
	std::string g_playcap_run;    // name of the run being captured
	std::string g_playcap_name_override;   // tab-chosen export name (wins)
	std::string g_playcap_last;

	// Alignment-battery queue state (see TeleportPlayCapture below).
	std::vector<int> g_battery_q;
	size_t g_battery_next = 0;
	bool g_battery_on = false;
	int g_battery_wait = 0;

	// ENGINE-QUERY battery (user-directed 2026-08-14: "can't you just query
	// the engine code for the response?"): each battery deck runs through
	// Prediction::RequestSim - the engine's OWN SetupMove/ProcessMovement/
	// FinishMove pipeline from the deck's anchor, with the live player
	// restored byte-for-byte afterward. No teleports, no physical playback,
	// nobody falls off a platform; the whole battery takes seconds. The
	// physical playback battery below remains as the arbiter path (it goes
	// through the authoritative server tick; a disagreement between the two
	// would itself be a finding).
	std::vector<Frame> g_batsim_frames;   // current deck, flattened
	std::vector<int> g_batsim_q;
	size_t g_batsim_next = 0;
	bool g_batsim_on = false;
	bool g_batsim_own = false;            // a battery-owned sim is in flight
	int g_batsim_done = 0;
	std::string g_batsim_name;

	void BatSimProvider(int tick, const Prediction::SimState&, Frame* out) {
		if (tick >= 0 && tick < static_cast<int>(g_batsim_frames.size())) {
			*out = g_batsim_frames[tick];
		} else {
			Frame f;
			memset(&f, 0, sizeof(f));
			*out = f;
		}
	}

	void ExportBatSim(const std::vector<Frame>& fr,
	                  const std::vector<Prediction::SimState>& st) {
		const std::string dir = SolverDir();
		if (dir.empty() || st.empty())
			return;
		const std::string path = UniqueSolverCsv("enginesim_"
			+ SanitizeName(g_batsim_name));
		std::ofstream out(path, std::ios::trunc);
		if (!out)
			return;
		{
			// MAP PRECONDITION HEADER (user finding 2026-08-14: a battery
			// run on the wrong map produced garbage that were still "real
			// engine values" - of the wrong world). The scorer refuses
			// captures whose map does not match the deck's.
			char mapname[128] = "";
			BspWorld::CurrentMapName(mapname, sizeof(mapname));
			out << "# map " << mapname << "\n";
		}
		{
			// SELF-DESCRIBING CAPTURE (user directive 2026-08-15: the one
			// click carries everything - if server settings change, every
			// consumer adapts). The LIVE params that governed this capture,
			// as "# param <key> <value>" lines the scorer prefers over any
			// cfg on disk. Same keys as server_params.cfg.
			char pl[160];
			const float itick = Prediction::LastDiag().interval_per_tick;
			Sfmt(pl, "# param tickinterval %g\n", itick > 0.f ? itick : 0.015f);
			out << pl;
			float ms;
			if (Prediction::LastRealMaxSpeed(&ms) && ms > 1.f) {
				Sfmt(pl, "# param maxspeed %g\n", ms);
				out << pl;
			}
			static const struct { const char* cvar; const char* key; }
			kLive[] = {
				{ "sv_gravity",            "gravity" },
				{ "sv_accelerate",         "accelerate" },
				{ "sv_airaccelerate",      "airaccelerate" },
				{ "sv_friction",           "friction" },
				{ "sv_stopspeed",          "stopspeed" },
				{ "sv_maxvelocity",        "maxvelocity" },
				{ "sv_stepsize",           "stepsize" },
				{ "sv_enablebunnyhopping", "enablebunnyhopping" },
			};
			for (const auto& kv : kLive) {
				float v;
				if (Cvars::GetFloat(kv.cvar, &v)) {
					Sfmt(pl, "# param %s %g\n", kv.key, v);
					out << pl;
				}
			}
		}
		out << "tick,x,y,z,vx,vy,vz,speed2d,ground,ducked,buttons,yaw,"
			"hulltop,mspd_a,mspd_b,stamina\n";
		char line[384];
		for (size_t t = 0; t < st.size(); ++t) {
			const Prediction::SimState& s = st[t];
			const Frame& f = t < fr.size() ? fr[t] : fr.back();
			Sfmt(line, "%d,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.2f,%d,%d,%d,"
				"%.4f,%.3f,%.3f,%.3f,%.4f\n",
				static_cast<int>(t), s.origin.X, s.origin.Y, s.origin.Z,
				s.velocity.X, s.velocity.Y, s.velocity.Z,
				sqrtf(s.velocity.X * s.velocity.X
					+ s.velocity.Y * s.velocity.Y),
				(s.flags & 1) ? 1 : 0,        // FL_ONGROUND
				(s.flags & 2) ? 1 : 0,        // FL_DUCKING
				f.buttons, f.viewangles[1],
				s.hull_top, s.mspd_a, s.mspd_b, s.stamina_ms);
			out << line;
		}
		AppendExportLog("enginesim", path, g_batsim_name.c_str(),
			static_cast<int>(st.size()));
	}

	// Combo getter over the recording library (Map Solve capture picker).
	bool RunItemGetter(void*, int idx, const char** out) {
		static char buf[160];
		const std::vector<Run>& lib = g_tas.Library();
		if (idx < 0 || idx >= static_cast<int>(lib.size()))
			return false;
		Sfmt(buf, "%s   [%s]   %d ticks", lib[idx].name.c_str(),
			lib[idx].map.empty() ? "?" : lib[idx].map.c_str(),
			static_cast<int>(lib[idx].FrameCount()));
		*out = buf;
		return true;
	}

	void FinalizePlaybackCapture() {
		g_playcap_active = false;
		if (g_playcap.empty())
			return;
		const std::string dir = SolverDir();
		if (dir.empty())
			return;
		std::string base = SanitizeName(g_playcap_run);
		if (base.empty())
			base = "run";
		const std::string path = UniqueSolverCsv("playback_" + base);
		std::ofstream out(path, std::ios::trunc);
		if (!out) {
			g_status = "Map Solve: couldn't write playback_states.csv";
			g_playcap.clear();
			return;
		}
		out << "tick,x,y,z,vx,vy,vz,speed2d,ground,ducked,buttons,yaw\n";
		char line[320];
		for (const PlayCapRow& r : g_playcap) {
			Sfmt(line, "%d,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.2f,%d,%d,%d,%.4f\n",
				r.tick, r.pos.X, r.pos.Y, r.pos.Z, r.vel.X, r.vel.Y, r.vel.Z,
				sqrtf(r.vel.X * r.vel.X + r.vel.Y * r.vel.Y),
				r.ground, r.ducked, r.buttons, r.yaw);
			out << line;
		}
		const int n = static_cast<int>(g_playcap.size());
		g_playcap.clear();
		g_playcap_last = path;
		AppendExportLog("playback", path, g_playcap_run.c_str(), n);
		ExportServerParams();   // the params that were LIVE for this capture
		g_status = FmtStr("Map Solve: captured %d REAL playback ticks of '%s' -> %s",
			n, g_playcap_run.c_str(), path.c_str());
		g_playcap_name_override.clear();
		if (g_battery_on)
			g_battery_wait = 30;   // settle frames before the next deck
	}

	// ---- ALIGNMENT BATTERY (user-directed 2026-08-14): ONE click plays
	// every 'battery_*' recording in sequence with capture armed; the
	// offline harness (SolverLab battery) scores every capture against the
	// core per mechanism. ----
	bool TeleportPlayCapture(int lib_idx, const char* export_name) {
		const std::vector<Run>& lib = g_tas.Library();
		if (lib_idx < 0 || lib_idx >= static_cast<int>(lib.size())
			|| lib[lib_idx].Empty())
			return false;
		const Run& r = lib[lib_idx];
		if (r.start.valid && engine) {
			char cmd[192];
			if (g_freecam)
				Sfmt(cmd, "setpos_exact %.6f %.6f %.6f",
					r.start.origin.X, r.start.origin.Y, r.start.origin.Z);
			else
				Sfmt(cmd, "setpos_exact %.6f %.6f %.6f; setang %.2f %.2f 0",
					r.start.origin.X, r.start.origin.Y, r.start.origin.Z,
					r.start.pitch, r.start.yaw);
			engine->ClientCmd_Unrestricted(cmd);
		}
		Run copy = lib[lib_idx];
		g_playcap_name_override = (export_name && export_name[0])
			? export_name : lib[lib_idx].name;
		g_playcap_armed = true;
		if (!g_tas.PlayEphemeral(std::move(copy), 12)) {
			g_playcap_armed = false;
			g_playcap_name_override.clear();
			return false;
		}
		return true;
	}

	// ---- TRACE ORACLE (engine-truth collision, user-directed 2026-08-13) ----
	// "We have the actual physics engine right here, why not take from that
	// instead of guessing?" - batch-answer the solver's trace queries with the
	// ENGINE's own collision code. SolverLab writes solver\trace_queries.csv
	// (replay --trace-log), this processes it through IEngineTrace::TraceRay
	// (world-only) into trace_results.csv, and `SolverLab tracediff` compares
	// our TraceHull against the engine answer for every single trace.

	IEngineTrace* g_engine_trace = nullptr;
}

// Adjustable ABI pins declared extern in IEngineTrace.h (the overlay-index
// pattern: probe-then-pin, one-click adjustable instead of a rebuild).
int  g_trace_ray_index = 5;      // 2013 IEngineTrace layout
bool g_ray_has_transform = true; // 2013 Ray_t (carries m_pWorldAxisTransform)

namespace {

	// SEH-guarded raw call: a mispinned vtable index REPORTS instead of
	// crashing the game (no C++ objects in scope - SEH rule).
	static bool SafeTraceCall(IEngineTrace* tr, int index, const void* ray,
	                          unsigned mask, void* filter, void* out) {
		__try {
			tr->TraceRayAt(index, ray, mask, filter, out);
			return true;
		} __except (EXCEPTION_EXECUTE_HANDLER) {
			return false;
		}
	}

	bool AcquireEngineTrace() {
		if (g_engine_trace)
			return true;
		g_engine_trace = GetInterface<IEngineTrace>("engine.dll",
			"EngineTraceClient004");
		if (!g_engine_trace)
			g_engine_trace = GetInterface<IEngineTrace>("engine.dll",
				"EngineTraceClient003");
		return g_engine_trace != nullptr;
	}

	// One engine trace with the ACTIVE (index, layout) pin. False on fault.
	bool OracleTrace(const Vector& a, const Vector& b, bool ducked,
	                 float* frac, Vector* end, Vector* nrm, float* pdist,
	                 bool* startsolid, bool* allsolid) {
		using namespace EngineTraceABI;
		static WorldOnlyFilter s_filter;
		const Vector mins(-16.f, -16.f, 0.f);
		const Vector maxs(16.f, 16.f, ducked ? 54.f : 72.f);
		RayBuf ray;
		BuildRay(ray, a, b, mins, maxs);
		TraceOut out;
		memset(out.raw, 0, sizeof(out.raw));
		if (!SafeTraceCall(g_engine_trace, g_trace_ray_index, &ray,
			kMaskPlayerSolid, &s_filter, out.raw))
			return false;
		if (frac) *frac = out.Frac();
		if (end) *end = out.End();
		if (nrm) *nrm = out.Normal();
		if (pdist) *pdist = out.PlaneDist();
		if (startsolid) *startsolid = out.StartSolid();
		if (allsolid) *allsolid = out.AllSolid();
		return true;
	}

	// Known-answer validation battery from the solve anchor (on the start
	// platform): pins (vtable index, Ray_t layout) by RESULT, the
	// IVDebugOverlay probe-then-pin discipline. Never processes a batch
	// until a config passes.
	bool ValidateOraclePin(char* why, size_t why_n) {
		if (!(g_solve_anchor_valid || LoadSolveAnchorFile())) {
			_snprintf_s(why, why_n, _TRUNCATE, "no solve anchor (capture one first)");
			return false;
		}
		const Vector az(g_solve_anchor.origin.X, g_solve_anchor.origin.Y,
		                g_solve_anchor.origin.Z);
		const int idx_try[2] = { g_trace_ray_index, g_trace_ray_index == 5 ? 4 : 5 };
		const bool lay_try[2] = { g_ray_has_transform, !g_ray_has_transform };
		for (int li = 0; li < 2; ++li) {
			for (int ii = 0; ii < 2; ++ii) {
				g_trace_ray_index = idx_try[ii];
				g_ray_has_transform = lay_try[li];
				float f1 = -1.f, f2 = -1.f, f3 = -1.f;
				Vector n1;
				bool ss1 = true;
				// V1: straight down onto the platform: expect ~0.5, n=(0,0,1)
				if (!OracleTrace(az + Vector(0.f, 0.f, 100.f),
					az + Vector(0.f, 0.f, -100.f), false, &f1, nullptr, &n1,
					nullptr, &ss1, nullptr))
					continue;
				// V2: clear air well above: expect fraction 1
				if (!OracleTrace(az + Vector(0.f, 0.f, 200.f),
					az + Vector(0.f, 0.f, 150.f), false, &f2, nullptr,
					nullptr, nullptr, nullptr, nullptr))
					continue;
				// V3: ducked hull, same down-trace: same feet-based fraction
				if (!OracleTrace(az + Vector(0.f, 0.f, 100.f),
					az + Vector(0.f, 0.f, -100.f), true, &f3, nullptr,
					nullptr, nullptr, nullptr, nullptr))
					continue;
				if (fabsf(f1 - 0.5f) < 0.05f && n1.Z > 0.9f && !ss1
					&& f2 > 0.999f && fabsf(f3 - f1) < 0.01f) {
					_snprintf_s(why, why_n, _TRUNCATE, "pinned index %d, %s Ray_t (v1 %.4f)",
						g_trace_ray_index,
						g_ray_has_transform ? "2013" : "2007", f1);
					return true;
				}
			}
		}
		_snprintf_s(why, why_n, _TRUNCATE, "no (index, layout) config passed the battery");
		return false;
	}

	// Batch: solver\trace_queries.csv -> solver\trace_results.csv.
	// Query row: id,tick,ax,ay,az,bx,by,bz,ducked[,...ours - ignored here]
	int ProcessTraceOracle(char* status, size_t status_n) {
		if (!AcquireEngineTrace()) {
			_snprintf_s(status, status_n, _TRUNCATE, "Trace oracle: EngineTraceClient interface "
				"NOT found.");
			return -1;
		}
		char pin[128];
		if (!ValidateOraclePin(pin, sizeof(pin))) {
			_snprintf_s(status, status_n, _TRUNCATE, "Trace oracle: validation FAILED (%s) - "
				"no batch run, game untouched.", pin);
			return -1;
		}
		const std::string dir = SolverDir();
		const std::string qpath = dir + "\\trace_queries.csv";
		const std::string rpath = dir + "\\trace_results.csv";
		FILE* q = nullptr;
		fopen_s(&q, qpath.c_str(), "r");
		if (!q) {
			_snprintf_s(status, status_n, _TRUNCATE, "Trace oracle: %s not found (generate it "
				"with SolverLab replay --trace-log).", qpath.c_str());
			return -1;
		}
		FILE* r = nullptr;
		fopen_s(&r, rpath.c_str(), "w");
		if (!r) {
			fclose(q);
			_snprintf_s(status, status_n, _TRUNCATE, "Trace oracle: cannot write %s", rpath.c_str());
			return -1;
		}
		fprintf(r, "id,frac,ex,ey,ez,nx,ny,nz,pdist,startsolid,allsolid\n");
		char line[512];
		int n = 0, faults = 0;
		while (fgets(line, sizeof(line), q)) {
			int id = 0, tick = 0, ducked = 0;
			float ax, ay, az2, bx, by, bz;
			if (sscanf_s(line, "%d,%d,%f,%f,%f,%f,%f,%f,%d", &id, &tick,
				&ax, &ay, &az2, &bx, &by, &bz, &ducked) != 9)
				continue;   // header / malformed
			float frac = 1.f, pdist = 0.f;
			Vector end, nrm;
			bool ss = false, as = false;
			if (!OracleTrace(Vector(ax, ay, az2), Vector(bx, by, bz),
				ducked != 0, &frac, &end, &nrm, &pdist, &ss, &as)) {
				faults++;
				continue;
			}
			fprintf(r, "%d,%.9g,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d\n",
				id, frac, end.X, end.Y, end.Z, nrm.X, nrm.Y, nrm.Z, pdist,
				ss ? 1 : 0, as ? 1 : 0);
			n++;
		}
		fclose(q);
		fclose(r);
		AppendExportLog("oracle", rpath, "trace_results", n);
		_snprintf_s(status, status_n, _TRUNCATE, "Trace oracle: %d engine answers written "
			"(%s; %d faults) -> trace_results.csv", n, pin, faults);
		return n;
	}

	void DrawMapSolveTab() {
		Theme::Heading("Map Solve",
			"Home of the full-map solver (design log: Docs\\FullMapSolver.md). "
			"The ground-truth loop between the ENGINE and the offline core - "
			"everything the solver grows lands in this tab.");

		SubHeading("Consistent start (solve anchor)");
		if (g_solve_anchor_valid || LoadSolveAnchorFile())
			ImGui::Text("Solve anchor: (%.1f, %.1f, %.1f) yaw %.1f%s",
				g_solve_anchor.origin.X, g_solve_anchor.origin.Y,
				g_solve_anchor.origin.Z, g_solve_anchor.yaw,
				g_solve_anchor.ducked ? " (ducked)" : "");
		else
			ImGui::TextDisabled("No solve anchor captured yet.");
		if (ImGui::Button("Capture solve anchor", ImVec2(170, 0)))
			WriteSolveAnchor();
		ImGui::SameLine();
		if (ImGui::Button("Teleport to anchor", ImVec2(150, 0)))
			TeleportToSolveAnchor();
		ImGui::SameLine();
		if (ImGui::Button("Surf setup", ImVec2(100, 0))) {
			if (engine) {
				engine->ClientCmd_Unrestricted(
					"sv_cheats 1; sv_accelerate 10; sv_airaccelerate 150; "
					"sv_enablebunnyhopping 1");
				g_air_accel = 150.f;
				MarkDirty();
				g_status = "Map Solve: surf server cvars sent - model updated.";
			}
		}
		ImGui::SameLine();
		if (ImGui::Button("God (toggle)", ImVec2(110, 0))) {
			if (engine) {
				engine->ClientCmd_Unrestricted("god");
				g_status = "Map Solve: sent 'god' (TOGGLES each click; needs "
					"sv_cheats 1). Fall damage off = playback can't die.";
			}
		}
		Theme::Help("God is a console TOGGLE, so it stays a separate button "
			"instead of hiding inside Surf setup (double-clicking setup would "
			"silently turn it back off). Fall-damage death breaks playback "
			"sync - keep god on while testing solver tapes.");
		Theme::Help("The agreed tick-0 state. Capture writes solver\\"
			"solve_anchor.cfg + the LIVE server params; offline solves auto-"
			"start from it, and Teleport puts the player back on it before "
			"playback - solver and game share one origin of truth.");
		// Playback alignment for ANY tape: its own anchor, one click.
		{
			const std::vector<Run>& lib = g_tas.Library();
			const int sel = g_tas.Selected();
			const bool have = sel >= 0 && sel < static_cast<int>(lib.size())
				&& lib[sel].start.valid;
			if (have) {
				char lbl[128];
				Sfmt(lbl, "Teleport to '%s' anchor", lib[sel].name.c_str());
				if (ImGui::Button(lbl, ImVec2(0, 0)) && engine) {
					const StartState& a = lib[sel].start;
					char cmd[192];
					Sfmt(cmd, "setpos_exact %.6f %.6f %.6f; setang %.2f %.2f 0",
						a.origin.X, a.origin.Y, a.origin.Z, a.pitch, a.yaw);
					engine->ClientCmd_Unrestricted(cmd);
					g_status = FmtStr("Map Solve: teleported to '%s' anchor.",
						lib[sel].name.c_str());
				}
				Theme::Help("Puts the player exactly on the SELECTED "
					"recording's tick-0 state (Record tab selection) so its "
					"playback starts from the state the tape assumes.");
			} else {
				ImGui::TextDisabled("(select a recording in the Record tab to "
					"teleport to its anchor)");
			}
		}

		SubHeading("Trace oracle (engine truth)");
		{
			static char s_oracle_status[256] = "";
			if (ImGui::Button("Run trace oracle", ImVec2(150, 0)))
				ProcessTraceOracle(s_oracle_status, sizeof(s_oracle_status));
			ImGui::SameLine();
			ImGui::Text("idx %d, %s Ray_t", g_trace_ray_index,
				g_ray_has_transform ? "2013" : "2007");
			if (s_oracle_status[0])
				ImGui::TextWrapped("%s", s_oracle_status);
			Theme::Help("Answers solver\\trace_queries.csv with the ENGINE'S "
				"own collision traces (IEngineTrace, world-only) into "
				"trace_results.csv. A known-answer battery from the solve "
				"anchor pins the vtable index + Ray_t layout before any batch "
				"runs (SEH-guarded probe - a wrong pin reports, never "
				"crashes). Generate queries with: SolverLab replay <map> "
				"<tape> --trace-log; compare with: SolverLab tracediff.");
		}

		SubHeading("Capture & export");
		{
			const std::vector<Run>& lib = g_tas.Library();
			static int s_cap_run = -1;
			if (s_cap_run >= static_cast<int>(lib.size()))
				s_cap_run = static_cast<int>(lib.size()) - 1;
			if (s_cap_run < 0 && !lib.empty()) {
				s_cap_run = (g_tas.Selected() >= 0) ? g_tas.Selected() : 0;
				Sfmt(g_solver_export_name, "%s", lib[s_cap_run].name.c_str());
			}
			ImGui::PushItemWidth(340);
			if (ImGui::Combo("Recording", &s_cap_run, RunItemGetter, nullptr,
				static_cast<int>(lib.size()))
				&& s_cap_run >= 0 && s_cap_run < static_cast<int>(lib.size()))
				Sfmt(g_solver_export_name, "%s", lib[s_cap_run].name.c_str());
			ImGui::InputText("Export name", g_solver_export_name,
				sizeof(g_solver_export_name));
			ImGui::PopItemWidth();
			if (g_playcap_active) {
				ImGui::TextColored(Theme::Warning,
					"CAPTURING '%s' (%d ticks)...", g_playcap_run.c_str(),
					static_cast<int>(g_playcap.size()));
				if (ImGui::Button("Abort capture", ImVec2(240, 0))) {
					g_playcap_armed = false;
					g_playcap_active = false;
					g_playcap.clear();
					g_playcap_name_override.clear();
					g_tas.EmergencyStop();
				}
			} else if (s_cap_run >= 0 && s_cap_run < static_cast<int>(lib.size())
				&& !lib[s_cap_run].Empty()) {
				if (ImGui::Button("Teleport, play & capture", ImVec2(240, 0))) {
					if (!TeleportPlayCapture(s_cap_run,
						g_solver_export_name[0] ? g_solver_export_name
							: nullptr)) {
						g_status = "Map Solve: can't play now - recorder busy.";
					} else {
						g_status = FmtStr("Map Solve: playing '%s', exporting "
							"as '%s' on finish.", lib[s_cap_run].name.c_str(),
							g_playcap_name_override.c_str());
					}
				}
			} else {
				ImGui::TextDisabled(lib.empty()
					? "(no recordings in the library)" : "(pick a recording)");
			}
			Theme::Help("One click: teleports to the recording's own anchor "
				"(sv_cheats 1), plays it, records the ACTUAL engine state of "
				"every tick, and exports solver\\playback_<export name>.csv + "
				"the live params + a manifest line in exports.log when it "
				"ends. The export name also governs 'Export sim states CSV' "
				"below.");
			if (!g_playcap_last.empty())
				ImGui::TextDisabled("last capture: %s", g_playcap_last.c_str());

			// ONE-CLICK ALIGNMENT BATTERY (user-directed 2026-08-14).
			// PRIMARY: engine QUERY - the decks run through the engine's
			// own movement pipeline (Prediction::RequestSim), player
			// untouched. Seconds, no teleports, no falling.
			if (g_batsim_on) {
				ImGui::TextColored(Theme::Warning,
					"BATTERY (query): %d/%d...",
					static_cast<int>(g_batsim_next),
					static_cast<int>(g_batsim_q.size()));
				ImGui::SameLine();
				if (ImGui::Button("Abort##batsim")) {
					g_batsim_on = false;
					g_batsim_own = false;
				}
			} else if (ImGui::Button(
				"Run ALIGNMENT battery (engine query)", ImVec2(300, 0))) {
				g_batsim_q.clear();
				g_batsim_next = 0;
				g_batsim_done = 0;
				// MAP PRECONDITION: decks anchor to coordinates in THEIR
				// map; simulated in another world the engine answers a
				// different question (measured: a wrong-map run pinned the
				// player in solid). Queue only matching decks.
				char cur[128] = "";
				BspWorld::CurrentMapName(cur, sizeof(cur));
				int wrong_map = 0;
				const std::vector<Run>& blib = g_tas.Library();
				for (int i = 0; i < static_cast<int>(blib.size()); ++i) {
					if (blib[i].name.rfind("battery_", 0) != 0
						|| blib[i].Empty() || !blib[i].start.valid)
						continue;
					if (!blib[i].map.empty() && cur[0]
						&& blib[i].map != cur) {
						wrong_map++;
						continue;
					}
					g_batsim_q.push_back(i);
				}
				if (g_batsim_q.empty()) {
					g_status = wrong_map > 0
						? FmtStr("Battery: %d decks found but NONE are for "
							"this map (%s) - load the decks' map first.",
							wrong_map, cur[0] ? cur : "?")
						: "Battery: no 'battery_*' recordings - run "
						  "SolverLab battery-gen <map.bsp>, then reinject "
						  "(fresh library scan).";
				} else {
					g_batsim_on = true;
					g_status = FmtStr("Battery(query): %d decks queued%s.",
						static_cast<int>(g_batsim_q.size()),
						wrong_map > 0 ? FmtStr(" (%d skipped: other map)",
							wrong_map).c_str() : "");
				}
			}
			Theme::Help("ONE CLICK: runs every battery_* deck through the "
				"ENGINE'S OWN movement code (SetupMove/ProcessMovement/"
				"FinishMove via the prediction hook) from each deck's "
				"anchor. You stay where you are; the player is restored "
				"byte-for-byte. Exports solver\\enginesim_battery_*.csv in "
				"seconds. Score offline: SolverLab battery <map.bsp>.");

			// FALLBACK: physical playback (authoritative server path) -
			// the arbiter if a query verdict is ever in doubt.
			if (g_battery_on) {
				ImGui::TextColored(Theme::Warning,
					"BATTERY (physical): run %d/%d...",
					static_cast<int>(g_battery_next),
					static_cast<int>(g_battery_q.size()));
				ImGui::SameLine();
				if (ImGui::Button("Abort##batphys")) {
					g_battery_on = false;
					g_playcap_armed = false;
					if (g_playcap_active) {
						g_playcap_active = false;
						g_playcap.clear();
					}
					g_playcap_name_override.clear();
					g_tas.EmergencyStop();
				}
			} else if (ImGui::Button(
				"Battery via physical playback (arbiter)", ImVec2(300, 0))) {
				g_battery_q.clear();
				g_battery_next = 0;
				const std::vector<Run>& blib = g_tas.Library();
				for (int i = 0; i < static_cast<int>(blib.size()); ++i)
					if (blib[i].name.rfind("battery_", 0) == 0
						&& !blib[i].Empty())
						g_battery_q.push_back(i);
				if (g_battery_q.empty()) {
					g_status = "Battery: no 'battery_*' recordings - run "
						"SolverLab battery-gen first.";
				} else {
					g_battery_on = true;
					g_battery_wait = 0;
					g_status = FmtStr("Battery(physical): %d decks queued.",
						static_cast<int>(g_battery_q.size()));
				}
			}
			Theme::Help("Same decks, physically played on the server with "
				"teleports + real-state capture (playback_battery_*.csv). "
				"Slower; use when a query verdict needs the authoritative "
				"server tick as the final word.");
		}

		SubHeading("Ground truth export");
		if (g_valid && !g_states.empty())
			ImGui::Text("Compiled run: %d ticks, anchor (%.1f, %.1f, %.1f)%s",
				static_cast<int>(g_states.size()),
				g_anchor.origin.X, g_anchor.origin.Y, g_anchor.origin.Z,
				g_dirty ? "  [SIM UPDATING - wait]" : "");
		else
			ImGui::TextDisabled("No compiled sim - load the project first (e.g. basictest).");
		const bool can_export = g_valid && !g_dirty && !g_states.empty();
		if (can_export) {
			if (ImGui::Button("Export sim states CSV", ImVec2(220, 0)))
				ExportSimStates();
		} else {
			ImGui::TextDisabled("(export unlocks once the sim is READY)");
		}
		Theme::Help("Writes the ENGINE-exact per-tick states (origin, velocity, "
			"ground/duck flags) of the compiled run to Documents\\sourceTAS\\"
			"solver\\<project>_states.csv, anchor included as tick -1. The "
			"offline core diffs itself against this file tick by tick to "
			"localize its first divergence.");
		if (!g_solver_export.empty())
			ImGui::TextDisabled("last export: %s", g_solver_export.c_str());

		SubHeading("Offline harness");
		ImGui::TextDisabled("SolverLab.exe (repo Output\\SolverLab\\):");
		ImGui::TextDisabled("  solve  <map.bsp> --end-brush N            route search (auto-uses solve anchor)");
		ImGui::TextDisabled("  diff   <map.bsp> <run.tas> <states.csv>   first-divergence report");
		ImGui::TextDisabled("  replay <map.bsp> <run.tas> --end-brush N  event timeline");
		ImGui::TextDisabled("Playback capture + sim export + params all land in Documents\\sourceTAS\\solver\\.");
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

		const float itick = Prediction::LastDiag().interval_per_tick > 0.f
			? Prediction::LastDiag().interval_per_tick : 0.015f;

		// The browser: newest first, each row carrying enough to know WHICH
		// project it is without loading it (map, size of the run, saved when).
		ImGui::BeginChild("projfiles", ImVec2(0, 200), true);
		for (int i = 0; i < static_cast<int>(g_files.size()); ++i) {
			const ProjFileInfo& fi = g_files[i];
			char row[320];
			if (fi.has_info)
				Sfmt(row, "%s    [%s]    %d seg   %u ticks (%.1f s)    %04d-%02d-%02d %02d:%02d##proj%d",
					fi.name.c_str(), fi.map[0] ? fi.map : "?",
					static_cast<int>(fi.segs.size()), fi.total_ticks,
					fi.total_ticks * itick,
					fi.mtime.wYear, fi.mtime.wMonth, fi.mtime.wDay,
					fi.mtime.wHour, fi.mtime.wMinute, i);
			else
				Sfmt(row, "%s    (pre-update save)    %04d-%02d-%02d %02d:%02d##proj%d",
					fi.name.c_str(),
					fi.mtime.wYear, fi.mtime.wMonth, fi.mtime.wDay,
					fi.mtime.wHour, fi.mtime.wMinute, i);
			if (ImGui::Selectable(row, g_sel_file == i))
				g_sel_file = i;
		}
		if (g_files.empty())
			ImGui::TextDisabled("(no saved projects)");
		const float pf_sy = ImGui::GetScrollY(), pf_sm = ImGui::GetScrollMaxY();
		ImGui::EndChild();
		static float s_pfiles_pw = 0.f;
		PassWheelAtEdge(&s_pfiles_pw, pf_sy, pf_sm);

		if (ImGui::Button("Load selected project") && g_sel_file >= 0 && g_sel_file < static_cast<int>(g_files.size()))
			LoadProject(g_files[g_sel_file].name);
		ImGui::SameLine();
		if (ImGui::Button("Delete selected project"))
			DeleteProjectFile();
		ImGui::SameLine();
		if (ImGui::Button("Export to library"))
			ExportToLibrary();
		ImGui::SameLine();
		if (ImGui::Button("Import selected recording"))
			ImportFromLibrary();

		// Details for the highlighted file - the expanded stats plus a
		// read-only picture of its segments, so the right project is known
		// BEFORE it replaces the one being edited.
		Theme::Heading("Selected project");
		if (g_sel_file >= 0 && g_sel_file < static_cast<int>(g_files.size())) {
			const ProjFileInfo& fi = g_files[g_sel_file];
			ImGui::Text("%s   (%.1f KB, saved %04d-%02d-%02d %02d:%02d)",
				fi.name.c_str(), fi.size / 1024.0,
				fi.mtime.wYear, fi.mtime.wMonth, fi.mtime.wDay,
				fi.mtime.wHour, fi.mtime.wMinute);
			if (fi.has_info) {
				ImGui::Text("map: %s", fi.map[0] ? fi.map : "? (saved outside a map)");
				if (fi.version >= 23) {
					if (fi.anchor_valid)
						ImGui::Text("anchor: %.0f  %.0f  %.0f  (saved with the project)",
							fi.anchor[0], fi.anchor[1], fi.anchor[2]);
					else
						ImGui::TextDisabled("no anchor stored - capture one after loading");
				}
				ImGui::Text("%d segment(s), %u ticks total (%.2f s)",
					static_cast<int>(fi.segs.size()), fi.total_ticks,
					fi.total_ticks * itick);
				ImGui::BeginChild("projsegs", ImVec2(0, 190), true);
				for (int k = 0; k < static_cast<int>(fi.segs.size()); ++k)
					ImGui::Text("#%d   %s   %d ticks  (%.2f s)", k,
						SegKindName(fi.segs[k].first),
						fi.segs[k].second, fi.segs[k].second * itick);
				if (fi.segs.empty())
					ImGui::TextDisabled("(no segments in this file)");
				const float ps_sy = ImGui::GetScrollY(), ps_sm = ImGui::GetScrollMaxY();
				ImGui::EndChild();
				static float s_psegs_pw = 0.f;
				PassWheelAtEdge(&s_psegs_pw, ps_sy, ps_sm);
			} else {
				ImGui::TextDisabled("no summary in pre-update saves - load it and "
					"Save once to index it.");
			}
		} else {
			ImGui::TextDisabled("select a project above to inspect it before loading");
		}
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
		// Deaths that raise no exception at all (these left crash.log empty).
		_set_invalid_parameter_handler(&InvalidParamHandler);
		std::set_terminate(&TerminateHandler);
		_set_purecall_handler(&PureCallHandler);
		signal(SIGABRT, &SignalHandler);
		// Rendering/UI prefs apply from the first frame, menu open or not.
		LoadUiSettings();
	}

	// Stage breadcrumbs through the WHOLE of Update: the 2026-08-05 freeze's
	// last record was "endscene: editor update" with nothing finer - the hang
	// is somewhere in here, and the next one must name its exact stage.
	Breadcrumb::Note(Breadcrumb::SlotUpdate, "update: search");
	// Pump the solver search (time-sliced; no-op when idle).
	StepSearch();

	Breadcrumb::Note(Breadcrumb::SlotUpdate, "update: freecam");
	// Freecam flight + layout verification (no-ops once irrelevant; the
	// verification runs HERE, never inside the view hook).
	FreecamMove();
	FreecamVerifyPump();

	Breadcrumb::Note(Breadcrumb::SlotUpdate, "update: teleport");
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

	Breadcrumb::Note(Breadcrumb::SlotUpdate, "update: correct");
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
	UndoIdleTick();

	// Test-play verdict: when the playback ends, report how faithfully the
	// real run followed the sim (the engine-exact promise, measured).
	{
		static bool s_was_playing = false;
		const bool playing = g_tas.IsPlaying();
		if (s_was_playing && !playing && g_testplay) {
			g_testplay = false;
			if (g_play_count > 0) {
				char line[224];
				if (g_play_max <= 0.5f)
					Sfmt(line, "Test play FOLLOWED the sim: max deviation %.3f u over %d ticks.",
						g_play_max, g_play_count);
				else
					Sfmt(line, "Test play DIVERGED: max %.1f u at tick %d (first past 0.5 u at "
						"tick %d) - check cvars, stamina, unmodeled triggers.",
						g_play_max, g_play_max_tick, g_play_first_tick);
				g_status = line;
			}
		}
		s_was_playing = playing;
	}

	// Autosave: a rolling crash backup, 2 s after the last mutation settles.
	if (g_autosave_pending && GetTickCount64() - g_last_user_edit_ms > 2000
		&& !g_dirty && !g_segs.empty()) {
		g_autosave_pending = false;
		SaveAutosave();
	}

	if (g_correct.active && g_correct.skip == 0 && g_valid && !g_dirty
		&& !g_sim_requested && !Prediction::SimBusy()
		&& !g_verify.active && !g_batch.active) {
		g_stage_name = "CorrectPump(kick)";
		g_machine_edit = true;
		CorrectPump();
		g_machine_edit = false;
		g_stage_name = "";
	}

	Breadcrumb::Note(Breadcrumb::SlotUpdate, "update: simland");
	// Land a finished sim, measure the REAL pass, feed the correction loop.
	// The whole chain runs under the same fault guard as the search jobs: a
	// crash anywhere in it becomes a NAMED stage in solver_fault.log and the
	// pumps stop, instead of a dead game with no evidence.
	if (Prediction::SimReady() && !g_batsim_own) {
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

	Breadcrumb::Note(Breadcrumb::SlotUpdate, "update: simreq");
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
			// COAST: append no-input ticks past the run end. g_coast_from tells
			// the provider where to start emitting nothing, and SimLandedStage
			// where to peel the tail off. -1 = off (nothing appended).
			g_coast_from = -1;
			if (g_coast_line && sim_ticks > 0 && sim_ticks < Prediction::kMaxSimTicks) {
				g_coast_from = sim_ticks;
				sim_ticks += g_coast_ticks;
				if (sim_ticks > Prediction::kMaxSimTicks)
					sim_ticks = Prediction::kMaxSimTicks;
			}
			if (Prediction::RequestSim(g_anchor, sim_ticks, &Provider)) {
				g_sim_requested = true;
				g_dirty = false;
			}
		}
	}
	// Map Solve playback capture: playback ended -> write the CSV. (The last
	// frame's outcome row is intentionally omitted; post-playback physics
	// would pollute it - the diff loses exactly one tick.)
	if (g_playcap_active && !g_tas.IsPlaying())
		FinalizePlaybackCapture();
	// ENGINE-QUERY battery pump: take our sim, export, request the next.
	if (g_batsim_on) {
		if (g_batsim_own && Prediction::SimReady()) {
			const int n = Prediction::SimCount();
			const bool faulted = Prediction::SimFaulted();
			std::vector<Frame> fr(n > 0 ? n : 1);
			std::vector<Prediction::SimState> st(n > 0 ? n : 1);
			if (n > 0)
				Prediction::TakeSim(fr.data(), st.data());
			g_batsim_own = false;
			if (!faulted && n > 0) {
				fr.resize(n);
				st.resize(n);
				ExportBatSim(fr, st);
				g_batsim_done++;
			}
		}
		if (!g_batsim_own && !Prediction::SimBusy()
			&& !Prediction::SimReady() && !g_sim_requested) {
			if (g_batsim_next < g_batsim_q.size()) {
				const std::vector<Run>& blib = g_tas.Library();
				const int idx = g_batsim_q[g_batsim_next++];
				if (idx >= 0 && idx < static_cast<int>(blib.size())
					&& blib[idx].start.valid && !blib[idx].Empty()) {
					const Run& r = blib[idx];
					g_batsim_frames.clear();
					for (const Segment& sg : r.segments)
						g_batsim_frames.insert(g_batsim_frames.end(),
							sg.begin(), sg.end());
					int ticks = static_cast<int>(g_batsim_frames.size());
					if (ticks > Prediction::kMaxSimTicks)
						ticks = Prediction::kMaxSimTicks;
					g_batsim_name = r.name;
					if (Prediction::RequestSim(r.start, ticks,
						&BatSimProvider)) {
						g_batsim_own = true;
						g_status = FmtStr("Battery(query): %s (%d/%d)...",
							r.name.c_str(),
							static_cast<int>(g_batsim_next),
							static_cast<int>(g_batsim_q.size()));
					}
					// RequestSim false: slot busy this frame - retry next
					// pump pass (g_batsim_next already advanced; step back).
					if (!g_batsim_own)
						g_batsim_next--;
				}
			} else {
				g_batsim_on = false;
				ExportServerParams();
				g_status = FmtStr("ENGINE-QUERY BATTERY COMPLETE: %d/%d "
					"decks exported to solver\\enginesim_*.csv. Score: "
					"SolverLab battery <map.bsp>", g_batsim_done,
					static_cast<int>(g_batsim_q.size()));
			}
		}
	}
	// Alignment-battery sequencer: after each capture lands (and a settle
	// gap), start the next queued deck. Menu-independent by design.
	if (g_battery_on && !g_playcap_active && !g_playcap_armed
		&& !g_tas.IsPlaying()) {
		if (g_battery_wait > 0) {
			--g_battery_wait;
		} else if (g_battery_next < g_battery_q.size()) {
			const int idx = g_battery_q[g_battery_next++];
			if (!TeleportPlayCapture(idx, nullptr)) {
				g_battery_on = false;
				g_status = "Battery: playback failed - aborted.";
			}
		} else {
			g_battery_on = false;
			g_status = FmtStr("ALIGNMENT BATTERY COMPLETE: %d captures in "
				"solver\\. Score offline: SolverLab battery <map.bsp>",
				static_cast<int>(g_battery_q.size()));
		}
	}

	Breadcrumb::Note(Breadcrumb::SlotUpdate, "update: end");
}

void TasEditor::Undo() { DoUndo(); }
void TasEditor::Redo() { DoRedo(); }

void TasEditor::NotePlaybackTick(int index) {
	// REAL playback capture (Map Solve): each row is the actual engine
	// outcome of frame index-1, on the same netvar basis as the divergence
	// verdict below. Grace-period repeats overwrite the tick -1 row, so the
	// anchor row is the settled post-teleport state - the direct check that
	// game and solver agree on tick 0.
	if (g_playcap_armed && index == 0) {
		g_playcap_armed = false;
		g_playcap_active = true;
		g_playcap.clear();
		// Name the capture: the tab's chosen export name wins; otherwise the
		// run being played. Identification is a manifest lookup, never a
		// guess.
		if (!g_playcap_name_override.empty()) {
			g_playcap_run = g_playcap_name_override;
		} else if (g_tas.IsTestPlayback()) {
			g_playcap_run = "testplay";
		} else {
			const int sel = g_tas.Selected();
			const std::vector<Run>& lib = g_tas.Library();
			g_playcap_run = (sel >= 0 && sel < static_cast<int>(lib.size()))
				? lib[sel].name : "run";
		}
	}
	if (g_playcap_active) {
		StartState pst;
		if (Prediction::CaptureStartState(pst) && g_playcap.size() < 20000) {
			PlayCapRow r;
			r.tick = index - 1;
			r.pos = pst.origin;
			r.vel = pst.velocity;
			int fl = 0;
			r.ground = Prediction::LiveFlags(&fl)
				? ((fl & FL_ONGROUND) ? 1 : 0) : -1;
			r.ducked = pst.ducked ? 1 : 0;
			r.buttons = (index == 0) ? 0 : g_playcap_buttons;
			r.yaw = (index == 0) ? pst.yaw : g_playcap_yaw;
			if (!g_playcap.empty() && g_playcap.back().tick == r.tick)
				g_playcap.back() = r;
			else
				g_playcap.push_back(r);
		}
	}

	// Called by the CreateMove hook (game thread - the same thread that runs
	// the sims, so g_states can't be resized under this read) with the index
	// of the frame ABOUT to replay: the player's current origin must equal
	// the sim's state after tick index-1.
	if (!g_testplay || !g_valid || index <= 0
		|| index - 1 >= static_cast<int>(g_states.size()))
		return;
	StartState st;
	if (!Prediction::CaptureStartState(st))
		return;
	const Vector& want = g_states[index - 1].origin;
	const float dx = st.origin.X - want.X;
	const float dy = st.origin.Y - want.Y;
	const float dz = st.origin.Z - want.Z;
	const float dev = sqrtf(dx * dx + dy * dy + dz * dz);
	if (dev > g_play_max) {
		g_play_max = dev;
		g_play_max_tick = index - 1;
		g_play_div_pos = st.origin;
	}
	if (g_play_first_tick < 0 && dev > 0.5f)
		g_play_first_tick = index - 1;
	g_play_count++;
}

void TasEditor::NotePlaybackInputs(int buttons, float yaw) {
	g_playcap_buttons = buttons;
	g_playcap_yaw = yaw;
}

bool TasEditor::DivergencePoint(Vector* p) {
	if (g_play_max <= 0.5f || g_play_max_tick < 0)
		return false;
	if (p)
		*p = g_play_div_pos;
	return true;
}

void TasEditor::ToggleCoastLine() {
	g_coast_line = !g_coast_line;
	MarkDirty();   // resim so the coast tail appears/clears immediately
	g_status = g_coast_line ? "Coast line on." : "Coast line off.";
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

	// FREECAM line of sight: while detached, the pick ray AND the stored view
	// angles are the CAMERA's - inspecting from the freecam must place targets
	// exactly where (and how) the camera is looking.
	if (g_freecam) {
		s.pitch = g_fc_pitch;
		s.yaw = g_fc_yaw;
	}
	const float d2r = kPi / 180.f;
	const float pr = s.pitch * d2r;
	const float yr = s.yaw * d2r;
	const Vector dir(cosf(pr) * cosf(yr), cosf(pr) * sinf(yr), -sinf(pr));
	const Vector eye = g_freecam
		? g_fc_pos_vis
		: s.origin + Vector(0.f, 0.f, s.ducked ? 46.f : 64.f);

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

void TasEditor::ToggleFreecam() {
	g_freecam = !g_freecam;
	if (g_freecam) {
		// Seed the camera from the CURRENT view (engine truth).
		QAngle va(0.f, 0.f, 0.f);
		if (engine)
			engine->GetViewAngles(va);
		g_fc_pitch = va.X;
		g_fc_yaw = va.Y;
		StartState st;
		if (Prediction::CaptureStartState(st))
			g_fc_pos = st.origin + Vector(0.f, 0.f, st.ducked ? 46.f : 64.f);
		g_fc_pos_vis = g_fc_pos;
		g_status = "Freecam ON: mouse looks, WASD flies, Space/Ctrl vertical, Shift fast.";
	} else {
		g_status = "Freecam off.";
	}
}

bool TasEditor::FreecamActive() {
	return g_freecam;
}

bool TasEditor::FreecamEye(Vector* eye) {
	if (!g_freecam)
		return false;
	if (eye)
		*eye = g_fc_pos_vis;   // what the camera renders is what tags face
	return true;
}

void TasEditor::ViewSlotSample(int slot, void* arg) {
	FreecamSample(slot, arg);
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
	out.coast = g_coast_path.empty() ? nullptr : g_coast_path.data();
	out.coast_count = static_cast<int>(g_coast_path.size());
	// Tag data: % of optimal gain over the selected segment + elapsed time.
	float interval = Prediction::LastDiag().interval_per_tick;
	if (interval <= 0.f) interval = 0.015f;
	out.cursor_time = (g_cursor >= 0) ? static_cast<float>(g_cursor) * interval : -1.f;
	out.seg_time = (out.sel_end > 0) ? static_cast<float>(out.sel_end) * interval : -1.f;
	out.seg_gain_pct = -1.f;
	if (out.sel_start >= 0 && out.sel_end > out.sel_start && !g_gain.empty()) {
		float gs = 0.f, ms = 0.f;
		const int hi = (std::min)(out.sel_end,
			(std::min)(static_cast<int>(g_eff.size()),
			           static_cast<int>(g_maxgain.size())));
		for (int i = out.sel_start; i < hi; ++i) {
			if (g_maxgain[i] <= 0.f)
				continue;   // unscoreable ticks (grounded/degenerate) stay out
			// eff*maxgain = the tick's SCORED gain: the raw gain in free
			// flight, the strafe-controllable part on a ramp slide - so the
			// segment % stays out of 100 across both.
			gs += g_eff[i] * g_maxgain[i];
			ms += g_maxgain[i];
		}
		if (ms > 1.f)
			out.seg_gain_pct = 100.f * gs / ms;
		// Out of 100 means OUT OF 100: free-flight gating above keeps physics
		// gains out, this keeps float residue from reading 100.3%.
		if (out.seg_gain_pct > 100.f)
			out.seg_gain_pct = 100.f;
	}
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
		LoadUiSettings();
	}

	// Bottom-left by default (NoSavedSettings means this applies every launch).
	{
		const ImVec2 disp = ImGui::GetIO().DisplaySize;
		float wy = disp.y - 820.f - 12.f;
		if (wy < 10.f) wy = 10.f;
		ImGui::SetNextWindowPos(ImVec2(12.f, wy), ImGuiSetCond_FirstUseEver);
	}
	ImGui::SetNextWindowSize(ImVec2(920, 820), ImGuiSetCond_FirstUseEver);
	// Translucent surfaces (backgrounds + buttons) so the game shows through the
	// panel; text stays opaque. Restored after End() so the replay HUD and the
	// in-world overlays are unaffected.
	Theme::ApplyOpacity(Theme::MenuOpacity());
	// Single panel now (record/run is a tab) - F8 controls visibility, so no
	// per-window close button.
	if (!ImGui::Begin("sourceTAS v1.1###sourceTAS", nullptr, ImGuiWindowFlags_NoSavedSettings)) {
		ImGui::End();
		Theme::ApplyOpacity(1.f);
		return;
	}

	// Compact status strip at the top: sim state chip + the editor status line.
	{
		const char* simst = g_dirty ? "UPDATING" : g_sim_requested ? "SIMMING"
			: g_sim_fault ? "SIM FAULT" : g_valid ? "READY" : "NO SIM";
		const ImVec4 simcol = (g_dirty || g_sim_requested) ? Theme::Warning
			: g_sim_fault ? Theme::Error : g_valid ? Theme::Success : Theme::Muted;
		ImGui::TextColored(simcol, "%s", simst);
		ImGui::SameLine();
		ImGui::TextDisabled("|");
		ImGui::SameLine();
		ImGui::TextUnformatted(g_status.c_str());
	}
	ImGui::Spacing();

	// Tab row (selected = pink accent).
	const char* tabs[] = { "Project", "Record", "Run", "Targets", "Server",
	                       "Map Solve", "Rendering" };
	for (int i = 0; i < 7; ++i) {
		if (Theme::Tab(tabs[i], g_tab == i, ImVec2(104, 0)))
			g_tab = i;
		if (i < 6)
			ImGui::SameLine();
	}
	ImGui::Spacing();

	// The tab body scrolls inside its own child, so the status strip and the
	// tab row above stay PINNED while the content scrolls. Transparent child
	// bg: the window's translucent surface is already behind it.
	ImGui::PushStyleColor(ImGuiCol_ChildWindowBg, ImVec4(0.f, 0.f, 0.f, 0.f));
	ImGui::BeginChild("##tabbody", ImVec2(0, 0), false);
	switch (g_tab) {
	case 0: DrawProjectTab(); break;
	case 1: RecordPanel::Draw(); break;
	case 2: DrawRunTab(); break;
	case 3: DrawTargetsTab(); break;
	case 4: DrawServerTab(); break;
	case 5: DrawMapSolveTab(); break;
	default: DrawRenderingTab(); break;
	}
	ImGui::EndChild();
	ImGui::PopStyleColor();

	// Persist any UI-pref change (any tab) once the interaction is done -
	// the options survive game restarts via ui.cfg.
	if (!ImGui::IsAnyItemActive()) {
		const std::string cur = SerializePrefs();
		if (cur != g_prefs_saved)
			SaveUiSettings();
	}

	ImGui::End();
	Theme::ApplyOpacity(1.f);   // restore the solid palette for HUD/overlays
}
}   // namespace (DrawWindowImpl)
