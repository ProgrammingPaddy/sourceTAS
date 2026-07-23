#include "TasEditor.h"

#include <algorithm>
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

namespace {
	constexpr float kPi = 3.14159265358979f;
	float Deg(float r) { return r * (180.f / kPi); }
	float Rad(float d) { return d * (kPi / 180.f); }
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
	// Inputs are modeled as KEYBOARD state: movement is which of W/A/S/D is
	// held, turning is the view angle. Auto-strafe is not a separate mode - it
	// is Move with the strafe key and view angle chosen per tick for you.
	enum SegMode   { SM_Move = 0, SM_Strafe };   // SM_Strafe == auto-strafe ON
	enum StrafeKey { SK_Auto = 0, SK_A, SK_D };  // dormant (auto picks per tick)
	enum PitchMode { PM_Const = 0, PM_Follow };
	enum JumpMode  { JM_None = 0, JM_Hold, JM_AutoBhop };

	struct GenParams {
		int   ticks = 66;
		int   mode = SM_Move;         // SM_Strafe = auto-strafe checkbox on

		// Turn intent, SCREEN sign (+ = curve right, - = curve left). Stored as
		// the TOTAL over the segment; the per-tick rate = total / ticks is what
		// the optimizer actually holds each tick.
		float turn_total = 0.f;
		bool  lock_rate = false;      // duration changes preserve rate, not total
		int   strafe_key = SK_Auto;   // dormant: auto-strafe picks A/D per tick
		float bias = 0.f;             // dormant: curve is followed exactly when possible

		// Manual movement (auto-strafe off): held keys + optional view turning.
		bool  key_w = true, key_a = false, key_s = false, key_d = false;
		int   yaw_mode = 0;           // 0 = hold, 1 = turn at yaw_rate
		float yaw_rate = 0.f;         // deg/tick, screen sign (+ = right)
		bool  yaw_abs = false;
		float yaw_start = 0.f;

		// Shared.
		int   pitch_mode = PM_Const;
		float pitch_val = 0.f;
		float pitch_mult = 1.f;
		bool  duck = false;
		int   jump_mode = JM_None;
	};

	struct EditSegment {
		bool raw = false;             // raw = explicit per-tick frames
		GenParams gen;
		std::vector<Frame> frames;
		// Per-tick jump overrides for generated segments: (local tick, 0=suppress
		// / 1=force). Lets a single tick jump without baking the segment.
		std::vector<std::pair<int, int>> jump_ovr;
		int Ticks() const { return raw ? static_cast<int>(frames.size()) : gen.ticks; }
	};

	// ---------------------------------------------------------------- state --
	bool g_open = false;
	char g_name[64] = "untitled";
	StartState g_anchor;
	std::vector<EditSegment> g_segs;
	int g_sel = -1;                   // selected segment
	int g_cursor = 0;                 // playhead tick
	bool g_show_hull = true;          // ghost hull at the playhead
	std::string g_status = "No segments. Capture an anchor and add one.";

	// Air-strafe model (match server settings; defaults = stock CS:S) + the
	// visual "optimal enough" threshold.
	float g_air_cap = 30.f;
	float g_air_accel = 10.f;
	float g_wishspeed = 250.f;
	float g_min_eff = 0.98f;

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
	struct PlanSeg { int start; int ticks; bool raw; int raw_off; GenParams gen; };
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

	// Max possible speed gain for one air tick at 2D speed `speed` (see
	// prediction-re notes for the derivation).
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

	// One air tick of the turn-following optimizer. The contract: achieve the
	// requested per-tick heading change EXACTLY whenever physics allows, using
	// the candidate (key side + view angle) with the best speed gain; when the
	// request exceeds what one tick of strafing can turn, turn as hard as
	// possible toward it. Scans both strafe keys (A and D) and the whole
	// keyboard-legal wishdir range - nothing analog.
	struct TurnPick { float yaw; int side; };
	TurnPick BestTurnYaw(const Vector& vel, float speed, float want_dh_src) {
		const float vel_yaw = Deg(atan2f(vel.Y, vel.X));
		const float L = g_plan_cap;
		const float a = g_plan_accel;
		float max_gain = MaxAirGain(speed);
		if (max_gain < 0.01f) max_gain = 0.01f;

		float best_cost = 1e9f, best_theta = 90.f, best_gain = 0.f;
		int best_side = (want_dh_src >= 0.f) ? 1 : -1;

		auto evaluate = [&](int side, float theta) {
			const float wish = vel_yaw + side * theta;
			float add = L - speed * cosf(Rad(theta));
			if (add > a) add = a;
			if (add < 0.f) add = 0.f;
			const float vx = vel.X + add * cosf(Rad(wish));
			const float vy = vel.Y + add * sinf(Rad(wish));
			const float gain = sqrtf(vx * vx + vy * vy) - speed;
			const float dh = NormYaw(Deg(atan2f(vy, vx)) - vel_yaw);
			const float err = fabsf(dh - want_dh_src);
			// Exact-turn candidates compete on gain; otherwise get as close to
			// the requested turn as possible (gain only breaks ties).
			const float cost = (err <= 0.1f)
				? (1.f - gain / max_gain)
				: (100.f + err - gain * 0.001f);
			if (cost < best_cost) {
				best_cost = cost;
				best_theta = theta;
				best_side = side;
				best_gain = gain;
			}
		};

		for (int side = -1; side <= 1; side += 2)
			for (float t = 0.f; t <= 178.f; t += 2.f)
				evaluate(side, t);
		for (float t = best_theta - 2.f; t <= best_theta + 2.f; t += 0.25f)
			if (t >= 0.f && t <= 178.f)
				evaluate(best_side, t);

		TurnPick pick;
		// Side-only key: A -> wishdir = yaw + 90, D -> wishdir = yaw - 90.
		pick.yaw = NormYaw(vel_yaw + best_side * best_theta - best_side * 90.f);
		pick.side = best_side;
		return pick;
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
			*out = g_plan_raw[ps.raw_off + t];
			g_pv_last_yaw = out->viewangles[1];
			g_pv_prev_jump = (out->buttons & IN_JUMP) != 0;
			return;
		}

		const GenParams& g = ps.gen;
		const float speed = Speed2D(prev.velocity);
		const bool onground = (prev.flags & FL_ONGROUND) != 0;

		if (t == 0)
			g_pv_entry_yaw = g.yaw_abs ? g.yaw_start : g_pv_last_yaw;

		float yaw = g_pv_entry_yaw;
		float fmove = 0.f;
		float smove = 0.f;

		if (g.mode == SM_Strafe) {
			// Auto-strafe: hold the requested turn rate THIS tick, relative to
			// the actual current heading (no cumulative schedule - the curve
			// stays uniform in time and splits stay seamless). Screen + = right
			// = Source yaw decrease.
			const float rate_ui = (ps.ticks > 0) ? g.turn_total / static_cast<float>(ps.ticks) : 0.f;
			const float want_dh_src = -rate_ui;

			if (speed <= 1.f) {
				yaw = g_pv_last_yaw;
				fmove = 450.f;
			} else {
				const float heading = Deg(atan2f(prev.velocity.Y, prev.velocity.X));
				if (onground) {
					// Ground tick between hops: run straight while still turning
					// the view at the curve rate, so the path bends smoothly.
					yaw = NormYaw(heading + want_dh_src);
					fmove = 450.f;
				} else {
					const TurnPick pick = BestTurnYaw(prev.velocity, speed, want_dh_src);
					yaw = pick.yaw;
					smove = (pick.side > 0) ? -450.f : 450.f;   // A / D
				}
			}
		} else {
			// Manual movement: held keys, view either held or turning at a
			// fixed rate (screen sign: + = right = Source yaw decrease).
			if (g.yaw_mode == 1)
				yaw = g_pv_entry_yaw - g.yaw_rate * static_cast<float>(t);
			yaw = NormYaw(yaw);
			fmove = (g.key_w ? 450.f : 0.f) + (g.key_s ? -450.f : 0.f);
			smove = (g.key_d ? 450.f : 0.f) + (g.key_a ? -450.f : 0.f);
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

	// How much the simulated path actually turned over ticks [b, e), in screen
	// sign (+ = right) - accumulated per tick so >180 degree turns don't wrap.
	float AchievedTurnUI(int b, int e) {
		if (e > static_cast<int>(g_states.size()))
			e = static_cast<int>(g_states.size());
		float sum = 0.f;
		float prev_heading = 0.f;
		bool have = false;
		const Vector v0 = (b > 0) ? g_states[b - 1].velocity : g_anchor.velocity;
		if (Speed2D(v0) > 1.f) {
			prev_heading = Deg(atan2f(v0.Y, v0.X));
			have = true;
		}
		for (int i = b; i < e; ++i) {
			const Vector& v = g_states[i].velocity;
			if (Speed2D(v) <= 1.f) { have = false; continue; }
			const float h = Deg(atan2f(v.Y, v.X));
			if (have)
				sum += NormYaw(h - prev_heading);
			prev_heading = h;
			have = true;
		}
		return -sum;   // Source + = left; screen + = right
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

	// Split helpers preserve the per-tick RATE on both halves, so the path is
	// bit-identical before and after a split (the per-tick turn target has no
	// schedule anchor to disagree about).
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
			const float rate = (total > 0) ? prefix.gen.turn_total / static_cast<float>(total) : 0.f;
			prefix.gen.ticks = local;
			prefix.gen.turn_total = rate * static_cast<float>(local);
			suffix.gen.ticks = total - local;
			suffix.gen.turn_total = rate * static_cast<float>(total - local);
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
	// behind the playhead cannot move), the suffix gets the NEW ones. Rates are
	// preserved on both halves.
	void AutoSplitApply(int k, int local, const GenParams& before, const GenParams& after) {
		EditSegment& seg = g_segs[k];

		EditSegment suffix;
		suffix.raw = false;
		suffix.gen = after;
		const float after_rate = (after.ticks > 0) ? after.turn_total / static_cast<float>(after.ticks) : 0.f;
		int suffix_ticks = after.ticks - local;
		if (suffix_ticks < 1) suffix_ticks = 1;
		suffix.gen.ticks = suffix_ticks;
		suffix.gen.turn_total = after_rate * static_cast<float>(suffix_ticks);
		suffix.gen.yaw_abs = false;

		SplitOverrides(seg, suffix, local);

		const float before_rate = (before.ticks > 0) ? before.turn_total / static_cast<float>(before.ticks) : 0.f;
		seg.gen = before;
		seg.gen.ticks = local;
		seg.gen.turn_total = before_rate * static_cast<float>(local);

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
	// v2: keyboard/turn-intent GenParams (turn stored + = left).
	// v3: turn_total flipped to screen sign (+ = right).
	// v4: yaw_rate also screen sign; per-tick jump overrides appended.
	constexpr uint32_t kProjVersion = 4;

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
			const uint8_t raw = s.raw ? 1 : 0;
			W(out, raw);
			if (s.raw) {
				const uint32_t n = static_cast<uint32_t>(s.frames.size());
				W(out, n);
				for (const Frame& f : s.frames)
					WriteFrame(out, f);
			} else {
				const GenParams& g = s.gen;
				const uint8_t lock = g.lock_rate ? 1 : 0, yabs = g.yaw_abs ? 1 : 0, duck = g.duck ? 1 : 0;
				const uint8_t kw = g.key_w, ka = g.key_a, ks = g.key_s, kd = g.key_d;
				W(out, g.ticks); W(out, g.mode);
				W(out, g.turn_total); W(out, lock); W(out, g.strafe_key); W(out, g.bias);
				W(out, kw); W(out, ka); W(out, ks); W(out, kd);
				W(out, g.yaw_mode); W(out, g.yaw_rate); W(out, yabs); W(out, g.yaw_start);
				W(out, g.pitch_mode); W(out, g.pitch_val); W(out, g.pitch_mult);
				W(out, duck); W(out, g.jump_mode);

				const uint32_t novr = static_cast<uint32_t>(s.jump_ovr.size());
				W(out, novr);
				for (const std::pair<int, int>& o : s.jump_ovr) {
					const int32_t local = o.first;
					const uint8_t val = static_cast<uint8_t>(o.second);
					W(out, local); W(out, val);
				}
			}
		}

		g_status = out ? ("Saved " + base + ".tasproj.") : "Write failed.";
		RefreshFiles();
		return static_cast<bool>(out);
	}

	// v1 files stored the very first generator (Hold/Rate/StrafeL/StrafeR +
	// analog fmove/smove); map them onto the keyboard/turn-intent model.
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
			g.mode = SM_Strafe;
			g.turn_total = 0.f;
		} else {
			g.mode = SM_Move;
			g.yaw_mode = yaw_mode;
			g.key_w = fmove > 100.f;
			g.key_s = fmove < -100.f;
			g.key_d = smove > 100.f;
			g.key_a = smove < -100.f;
		}
		return true;
	}

	bool ReadGenV2(std::ifstream& in, GenParams& g) {
		uint8_t lock = 0, yabs = 0, duck = 0, kw = 0, ka = 0, ks = 0, kd = 0;
		if (!R(in, g.ticks) || !R(in, g.mode) ||
			!R(in, g.turn_total) || !R(in, lock) || !R(in, g.strafe_key) || !R(in, g.bias) ||
			!R(in, kw) || !R(in, ka) || !R(in, ks) || !R(in, kd) ||
			!R(in, g.yaw_mode) || !R(in, g.yaw_rate) || !R(in, yabs) || !R(in, g.yaw_start) ||
			!R(in, g.pitch_mode) || !R(in, g.pitch_val) || !R(in, g.pitch_mult) ||
			!R(in, duck) || !R(in, g.jump_mode))
			return false;
		g.lock_rate = lock != 0;
		g.yaw_abs = yabs != 0;
		g.duck = duck != 0;
		g.key_w = kw != 0; g.key_a = ka != 0; g.key_s = ks != 0; g.key_d = kd != 0;
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
			uint8_t raw = 0;
			if (!R(in, raw)) return false;
			EditSegment s;
			s.raw = raw != 0;
			if (s.raw) {
				uint32_t n = 0;
				if (!R(in, n) || n > static_cast<uint32_t>(Prediction::kMaxSimTicks)) return false;
				s.frames.resize(n);
				for (uint32_t f = 0; f < n; ++f)
					if (!ReadFrame(in, s.frames[f])) return false;
			} else if (version >= 2) {
				if (!ReadGenV2(in, s.gen)) return false;
				if (version == 2)
					s.gen.turn_total = -s.gen.turn_total;   // v2 stored + = left
				if (version <= 3)
					s.gen.yaw_rate = -s.gen.yaw_rate;       // <=v3 stored + = left
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
			} else {
				if (!ReadGenV1(in, s.gen)) return false;
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
	// Compiled frames -> a Run (segment boundaries preserved, anchor attached).
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
}

// ------------------------------------------------------------------- public --
void TasEditor::Update() {
	// Land a finished sim.
	if (Prediction::SimReady()) {
		const int n = Prediction::SimCount();
		g_sim_fault = Prediction::SimFaulted();
		g_frames.resize(n);
		g_states.resize(n);
		Prediction::TakeSim(g_frames.data(), g_states.data());
		g_valid = n > 0;
		g_sim_requested = false;
		RecomputeDiag();
	}

	// Live preview: resim as fast as the sim itself allows. A pass is typically
	// a few ms, so slider drags resim every frame at full accuracy; only long
	// runs (measured, not guessed) get a short debounce to protect the
	// framerate. One request is in flight at most - new edits queue the next.
	const float sim_ms = Prediction::LastDiag().sim_ms;
	const unsigned long long debounce_ms =
		(sim_ms <= 8.f) ? 0 : (sim_ms <= 25.f) ? 60 : 200;
	if (g_dirty && !g_sim_requested && !Prediction::SimBusy()
		&& GetTickCount64() - g_dirty_ms >= debounce_ms
		&& g_total > 0 && g_anchor.valid) {
		if (BuildPlan() && Prediction::RequestSim(g_anchor, g_total, &Provider)) {
			g_sim_requested = true;
			g_dirty = false;
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
	return true;
}

void TasEditor::DrawWindow() {
	if (!g_open)
		return;

	ImGui::SetNextWindowSize(ImVec2(920, 820), ImGuiSetCond_FirstUseEver);
	if (!ImGui::Begin("sourceTAS Editor", &g_open, ImGuiWindowFlags_NoSavedSettings)) {
		ImGui::End();
		return;
	}

	// --- anchor + test play --------------------------------------------------
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

	// --- segment list --------------------------------------------------------
	ImGui::Text("Segments   (%d ticks total, %.2f s)", g_total,
		g_total * (Prediction::LastDiag().interval_per_tick > 0.f ? Prediction::LastDiag().interval_per_tick : 0.015f));
	ImGui::BeginChild("segs", ImVec2(0, 100), true);
	for (int k = 0; k < static_cast<int>(g_segs.size()); ++k) {
		const EditSegment& s = g_segs[k];
		char kind[48];
		if (s.raw) {
			strcpy_s(kind, "RAW");
		} else if (s.gen.mode == SM_Strafe) {
			const float turn = s.gen.turn_total;
			sprintf_s(kind, "STRAFE %.0f%s deg", fabsf(turn),
				turn > 0.5f ? " R" : turn < -0.5f ? " L" : "");
		} else {
			sprintf_s(kind, "MOVE %s%s%s%s",
				s.gen.key_w ? "W" : "", s.gen.key_a ? "A" : "",
				s.gen.key_s ? "S" : "", s.gen.key_d ? "D" : "");
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

	if (ImGui::Button("+ Strafe")) {
		EditSegment s;
		s.gen.mode = SM_Strafe;
		s.gen.jump_mode = JM_AutoBhop;
		InsertSegment(s);
	}
	ImGui::SameLine();
	if (ImGui::Button("+ Move")) {
		EditSegment s;
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

	// --- selected segment parameters ------------------------------------------
	// Edits while the playhead sits inside the segment auto-split first, so a
	// change can never move the path behind the playhead.
	if (g_sel >= 0 && g_sel < static_cast<int>(g_segs.size())) {
		ImGui::Separator();
		if (!g_segs[g_sel].raw) {
			const GenParams before = g_segs[g_sel].gen;
			GenParams g = before;
			bool ch = false;

			bool auto_strafe = (g.mode == SM_Strafe);
			if (ImGui::Checkbox("Auto-strafe (picks A/D and the view angle per tick)", &auto_strafe)) {
				g.mode = auto_strafe ? SM_Strafe : SM_Move;
				ch = true;
			}

			if (auto_strafe) {
				ImGui::TextWrapped(
					"Curve = how much the heading changes per tick, applied uniformly over "
					"the whole segment (higher speed just covers more ground per degree). "
					"The optimizer turns EXACTLY at that rate whenever physics allows, "
					"using whichever key and view angle gains the most speed; if the curve "
					"is tighter than one tick of strafing can turn, the tick turns as hard "
					"as possible and the readout below shows the shortfall.");

				float rate = (g.ticks > 0) ? g.turn_total / static_cast<float>(g.ticks) : 0.f;
				if (FloatRow("Curve   (- left / + right)", &rate, -10.f, 10.f, "%+.2f deg/tick", 0.1f, 1.f)) {
					g.turn_total = rate * static_cast<float>(g.ticks);
					ch = true;
				}
				ch |= FloatRow("Total turn over segment", &g.turn_total, -720.f, 720.f, "%+.1f deg", 0.1f, 5.f, 1);

				const int prev_ticks = g.ticks;
				const bool ticks_changed = IntRow("Ticks (duration)", &g.ticks, 1, 1000);
				ch |= ticks_changed;
				if (ticks_changed && g.lock_rate && prev_ticks > 0)
					g.turn_total = (g.turn_total / static_cast<float>(prev_ticks)) * static_cast<float>(g.ticks);
				ImGui::Text("= %+.2f deg/tick average", g.ticks > 0 ? g.turn_total / g.ticks : 0.f);
				ImGui::SameLine();
				ch |= ImGui::Checkbox("Duration changes preserve rate (not total)", &g.lock_rate);

				if (g_valid && !g_dirty && g_sel < static_cast<int>(g_starts.size())) {
					const int b = g_starts[g_sel];
					int e = (g_sel + 1 < static_cast<int>(g_starts.size())) ? g_starts[g_sel + 1] : g_total;
					const float achieved = AchievedTurnUI(b, e);
					ImGui::Text("requested %+.1f deg -> achieved %+.1f deg", g.turn_total, achieved);
					if (fabsf(achieved - g.turn_total) > 5.f && fabsf(g.turn_total) > 1.f) {
						ImGui::SameLine();
						ImGui::TextColored(ImVec4(1.f, 0.6f, 0.2f, 1.f),
							"(curve too tight for the air time - add ticks or accept)");
					}
				}
			} else {
				ImGui::Text("Held keys:");
				ImGui::SameLine(); ch |= ImGui::Checkbox("W", &g.key_w);
				ImGui::SameLine(); ch |= ImGui::Checkbox("A", &g.key_a);
				ImGui::SameLine(); ch |= ImGui::Checkbox("S", &g.key_s);
				ImGui::SameLine(); ch |= ImGui::Checkbox("D", &g.key_d);
				ch |= IntRow("Ticks (duration)", &g.ticks, 1, 1000);
				ImGui::PushItemWidth(180);
				ch |= ImGui::Combo("View", &g.yaw_mode, "Hold\0Turn (deg/tick)\0");
				ImGui::PopItemWidth();
				if (g.yaw_mode == 1)
					ch |= FloatRow("View turn   (- left / + right)", &g.yaw_rate, -15.f, 15.f, "%+.2f deg/tick", 0.1f, 1.f);
				ch |= ImGui::Checkbox("Absolute start yaw", &g.yaw_abs);
				if (g.yaw_abs) {
					ImGui::SameLine();
					ImGui::PushItemWidth(kInputW);
					ch |= ImGui::InputFloat("##yawstart", &g.yaw_start, 0.1f, 5.f, 1);
					ImGui::PopItemWidth();
				}
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

	// --- playhead: slider is the primary control -------------------------------
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

		ImGui::Checkbox("Ghost hull at playhead", &g_show_hull);
		ImGui::SameLine();
		ImGui::TextDisabled("(origin dot + view arrow always draw)");

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

	// --- strafe model + sim status ---------------------------------------------
	if (ImGui::CollapsingHeader("Strafe model (match server cvars)")) {
		bool ch = false;
		ImGui::PushItemWidth(120);
		ch |= ImGui::InputFloat("Air speed cap", &g_air_cap, 1.f, 10.f, 1);
		ch |= ImGui::InputFloat("sv_airaccelerate", &g_air_accel, 1.f, 10.f, 1);
		ch |= ImGui::InputFloat("Wish speed", &g_wishspeed, 10.f, 50.f, 0);
		ImGui::PopItemWidth();
		ch |= FloatRow("Min efficiency (line coloring only)", &g_min_eff, 0.90f, 1.f, "%.2f", 0.005f, 0.02f);
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

	ImGui::Separator();

	// --- project + library bridge ------------------------------------------------
	ImGui::PushItemWidth(180);
	ImGui::InputText("##projname", g_name, sizeof(g_name));
	ImGui::PopItemWidth();
	ImGui::SameLine();
	if (ImGui::Button("Save project"))
		SaveProject();
	ImGui::SameLine();
	if (ImGui::Button("Refresh list"))
		RefreshFiles();

	ImGui::BeginChild("projfiles", ImVec2(0, 70), true);
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

	ImGui::TextWrapped("%s", g_status.c_str());
	ImGui::End();
}
