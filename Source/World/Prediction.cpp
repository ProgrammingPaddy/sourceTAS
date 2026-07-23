#include "Prediction.h"
#include "NetVars.h"

#include <cstdint>
#include <cstring>
#include <memory>
#include <vector>
#include <windows.h>

#include <cstrike/sdk.h>
#include <cstrike/Interfaces/IClientEntityList.h>
#include <vmthook/vmthook.h>

// Menu-controlled flags/params (defined in WorldDraw.cpp), read by the hook.
namespace WorldDraw {
	extern bool  draw_prediction;
	extern int   pred_ticks;
	extern bool  pred_live_input;
	extern bool  pred_autobhop;
	extern float pred_forwardmove;
	extern float pred_sidemove;
	extern bool  pred_jump;
	extern bool  pred_duck;
}

// ---------------------------------------------------------------------------
// Constants pinned by static RE of cstrike/bin/x64/client.dll (see the
// prediction-re memory). We run inside the engine's valid prediction context
// (hooked FinishMove), so nothing has to be faked - only save/restore of what
// we disturb.
// ---------------------------------------------------------------------------
namespace {
	constexpr uintptr_t kMoveHelperRva       = 0x5B4FF0;  // CMoveHelperClient singleton
	constexpr uintptr_t kMoveHelperVtableRva = 0x450EF0;  // its vtable (sanity check)
	constexpr uintptr_t kGpGlobalsPtrRva     = 0x5AE280;  // holds gpGlobals pointer

	constexpr int kSetupMoveIdx            = 18;  // IPrediction
	constexpr int kFinishMoveIdx           = 19;  // IPrediction (hooked)
	constexpr int kIsFirstTimePredictedIdx = 15;  // IPrediction
	constexpr int kProcessMovementIdx      = 1;   // IGameMovement

	constexpr int kTouchCountOff     = 0x18;    // CMoveHelperClient touch-list count
	constexpr int kMoveDataOriginOff = 0x9C;    // CMoveData::m_vecAbsOrigin
	constexpr int kMoveDataVelOff    = 0x44;    // CMoveData::m_vecVelocity
	constexpr int kPlayerSize        = 0x1C50;  // sizeof(C_CSPlayer)
	constexpr int kMoveDataBufSize   = 0x200;
	constexpr int kFrametimeOff = 0x10;
	constexpr int kCurtimeOff   = 0x0C;
	constexpr int kIntervalOff  = 0x1C;

	// Resolved handles.
	void*     g_gm = nullptr;
	void*     g_pred = nullptr;
	void*     g_helper = nullptr;
	void**    g_gpg_holder = nullptr;
	uintptr_t g_client_base = 0;
	std::unique_ptr<VMTHook> g_pred_hook;
	using FinishMoveFn = void(*)(void*, void*, void*, void*);
	FinishMoveFn g_orig_finishmove = nullptr;

	// Live look-ahead output.
	Vector g_path[257];
	int    g_path_count = 0;

	// Origin of the newest REAL command's movedata (basis diagnostic: lets the
	// menu compare the movement pipeline's origin against the netvar origin).
	Vector g_real_move_origin;
	bool   g_real_move_valid = false;
	bool   g_in_hook_work = false;
	float  g_interval = 0.f;
	int    g_fault_count = 0;
	unsigned long long g_last_fault_rva = 0;
	unsigned long long g_last_fault_access = 0;
	Prediction::Diag g_diag;

	// Editor sim request/result. Render thread writes the request and takes the
	// result; the game thread executes. The pending/ready flags are the
	// single-producer/single-consumer handshake.
	volatile long s_sim_pending = 0;
	volatile long s_sim_ready = 0;
	volatile long s_sim_fault = 0;
	StartState             s_sim_anchor;
	int                    s_sim_req_ticks = 0;
	Prediction::SimFrameFn s_sim_provider = nullptr;
	int                    s_sim_count = 0;
	Frame                  s_sim_frames[Prediction::kMaxSimTicks];
	Prediction::SimState   s_sim_states[Prediction::kMaxSimTicks];

	// SEH restore state (statics so __except can undo without locals).
	unsigned char s_snapshot[kPlayerSize];
	void*  s_player = nullptr;
	float* s_p_frametime = nullptr;
	float* s_p_curtime = nullptr;
	int*   s_p_touch = nullptr;
	float  s_save_frametime = 0.f;
	float  s_save_curtime = 0.f;
	int    s_save_touch = 0;
	bool   s_restore_ready = false;

	// Player field offsets resolved from the netvar tree (0 = unresolved).
	struct PlayerOffsets {
		int origin = 0, velocity = 0, flags = 0, eye_angles = 0;
		int stamina = 0, ducked = 0, ducking = 0, ducktime = 0;
		bool init = false;
	} g_off;

	void ResolveOffsets() {
		if (g_off.init) return;
		g_off.origin     = NetVars::Offset("DT_CSPlayer", "m_vecOrigin");
		g_off.velocity   = NetVars::Offset("DT_CSPlayer", "m_vecVelocity[0]");
		g_off.flags      = NetVars::Offset("DT_CSPlayer", "m_fFlags");
		g_off.eye_angles = NetVars::Offset("DT_CSPlayer", "m_angEyeAngles[0]");
		g_off.stamina    = NetVars::Offset("DT_CSPlayer", "m_flStamina");
		g_off.ducked     = NetVars::Offset("DT_CSPlayer", "m_bDucked");
		g_off.ducking    = NetVars::Offset("DT_CSPlayer", "m_bDucking");
		g_off.ducktime   = NetVars::Offset("DT_CSPlayer", "m_flDucktime");
		g_off.init = true;
	}

	// Minimal fault net: record where a fault landed, then recover.
	long FaultFilter(EXCEPTION_POINTERS* ep) {
		const EXCEPTION_RECORD* r = ep->ExceptionRecord;
		const uintptr_t rip = reinterpret_cast<uintptr_t>(r->ExceptionAddress);
		g_last_fault_rva = (g_client_base && rip >= g_client_base) ? (rip - g_client_base) : rip;
		g_last_fault_access = (r->ExceptionCode == EXCEPTION_ACCESS_VIOLATION && r->NumberParameters >= 2)
			? r->ExceptionInformation[1] : 0;
		return EXCEPTION_EXECUTE_HANDLER;
	}

	template <int Idx, typename... Args>
	void CallV(void* obj, Args... args) {
		using Fn = void(*)(void*, Args...);
		GetVirtualFunction<Fn>(obj, Idx)(obj, args...);
	}

	bool BeginStateGuard(void* player) {
		void* gpg = *g_gpg_holder;
		if (!gpg) return false;
		char* g = reinterpret_cast<char*>(gpg);
		s_p_frametime = reinterpret_cast<float*>(g + kFrametimeOff);
		s_p_curtime   = reinterpret_cast<float*>(g + kCurtimeOff);
		const float interval = *reinterpret_cast<float*>(g + kIntervalOff);
		if (interval <= 0.f || interval > 1.f) return false;
		g_interval = interval;

		s_player = player;
		memcpy(s_snapshot, player, kPlayerSize);
		s_save_frametime = *s_p_frametime;
		s_save_curtime   = *s_p_curtime;
		s_p_touch    = reinterpret_cast<int*>(reinterpret_cast<char*>(g_helper) + kTouchCountOff);
		s_save_touch = *s_p_touch;
		s_restore_ready = true;

		*s_p_frametime = interval;   // drive movement at exactly one tick
		return true;
	}

	void EndStateGuard() {
		memcpy(s_player, s_snapshot, kPlayerSize);
		*s_p_frametime = s_save_frametime;
		*s_p_curtime   = s_save_curtime;
		*s_p_touch     = s_save_touch;   // discard touches queued by hypothetical ticks
	}

	// --- live look-ahead (Phase 1b) -----------------------------------------
	int RunLookaheadSEH(void* player, void* ucmd, int ticks) {
		__try {
			if (!BeginStateGuard(player)) return -1;

			alignas(16) unsigned char cmdbuf[sizeof(CUserCmd)];
			memcpy(cmdbuf, ucmd, sizeof(CUserCmd));
			CUserCmd* cmd = reinterpret_cast<CUserCmd*>(cmdbuf);
			if (!WorldDraw::pred_live_input) {
				cmd->forwardmove = WorldDraw::pred_forwardmove;
				cmd->sidemove    = WorldDraw::pred_sidemove;
				cmd->buttons     = (WorldDraw::pred_jump ? IN_JUMP : 0) | (WorldDraw::pred_duck ? IN_DUCK : 0);
			}

			// Auto-bhop: replace jump handling entirely - press IN_JUMP only on
			// ticks that start on the ground (held jump is ignored by the engine).
			const bool autobhop = WorldDraw::pred_autobhop && g_off.flags;
			int base_buttons = cmd->buttons;
			if (autobhop) base_buttons &= ~IN_JUMP;
			bool prev_jump = false;

			unsigned char movebuf[kMoveDataBufSize];
			for (int i = 0; i < ticks; ++i) {
				memset(movebuf, 0, sizeof(movebuf));
				cmd->command_number = i + 1;
				cmd->tick_count     = i + 1;

				int buttons = base_buttons;
				if (autobhop) {
					const int fl = *reinterpret_cast<int*>(reinterpret_cast<char*>(player) + g_off.flags);
					if ((fl & FL_ONGROUND) && !prev_jump)
						buttons |= IN_JUMP;
				}
				cmd->buttons = buttons;
				prev_jump = (buttons & IN_JUMP) != 0;

				CallV<kSetupMoveIdx>(g_pred, player, cmd, g_helper, movebuf);
				CallV<kProcessMovementIdx>(g_gm, player, movebuf);
				g_path[i] = *reinterpret_cast<Vector*>(movebuf + kMoveDataOriginOff);
				g_orig_finishmove(g_pred, player, cmd, movebuf);
				*s_p_curtime += g_interval;
			}

			EndStateGuard();
			return ticks;
		}
		__except (FaultFilter(GetExceptionInformation())) {
			if (s_restore_ready)
				EndStateGuard();
			return -100;
		}
	}

	// --- editor simulation (Phase 2) ------------------------------------------
	// Same pipeline, but the start state is teleported to an absolute anchor and
	// each tick's input comes from the provider (closed loop: it sees the
	// simulated state after the previous tick).
	int RunEditorSimSEH(void* player) {
		__try {
			if (!BeginStateGuard(player)) return -1;

			char* pb = reinterpret_cast<char*>(player);

			// Teleport the (snapshotted) player to the anchor. Ground state is
			// re-derived by CategorizePosition on the first tick, so flags only
			// need the duck bit; duck timers/stamina are set for determinism.
			*reinterpret_cast<Vector*>(pb + g_off.origin)   = s_sim_anchor.origin;
			*reinterpret_cast<Vector*>(pb + g_off.velocity) = s_sim_anchor.velocity;
			int flags = 0;
			if (g_off.flags) {
				flags = *reinterpret_cast<int*>(pb + g_off.flags);
				if (s_sim_anchor.ducked) flags |= FL_DUCKING; else flags &= ~FL_DUCKING;
				*reinterpret_cast<int*>(pb + g_off.flags) = flags;
			}
			if (g_off.ducked)   *reinterpret_cast<bool*>(pb + g_off.ducked)   = s_sim_anchor.ducked;
			if (g_off.ducking)  *reinterpret_cast<bool*>(pb + g_off.ducking)  = false;
			if (g_off.ducktime) *reinterpret_cast<float*>(pb + g_off.ducktime) = 0.f;
			if (g_off.stamina)  *reinterpret_cast<float*>(pb + g_off.stamina)  = s_sim_anchor.stamina;

			Prediction::SimState prev;
			prev.origin   = s_sim_anchor.origin;
			prev.velocity = s_sim_anchor.velocity;
			prev.flags    = flags;

			alignas(16) unsigned char cmdbuf[sizeof(CUserCmd)];
			CUserCmd* cmd = reinterpret_cast<CUserCmd*>(cmdbuf);
			unsigned char movebuf[kMoveDataBufSize];
			Frame f;

			const int n = s_sim_req_ticks;
			for (int i = 0; i < n; ++i) {
				s_sim_provider(i, prev, &f);

				memset(cmdbuf, 0, sizeof(cmdbuf));
				memset(movebuf, 0, sizeof(movebuf));
				cmd->command_number = i + 1;
				cmd->tick_count     = i + 1;
				cmd->viewangles     = QAngle(f.viewangles[0], f.viewangles[1], 0.f);
				cmd->forwardmove    = f.forwardmove;
				cmd->sidemove       = f.sidemove;
				cmd->upmove         = f.upmove;
				cmd->buttons        = f.buttons;
				cmd->impulse        = f.impulse;

				CallV<kSetupMoveIdx>(g_pred, player, cmd, g_helper, movebuf);
				CallV<kProcessMovementIdx>(g_gm, player, movebuf);

				Prediction::SimState st;
				st.origin   = *reinterpret_cast<Vector*>(movebuf + kMoveDataOriginOff);
				st.velocity = *reinterpret_cast<Vector*>(movebuf + kMoveDataVelOff);

				g_orig_finishmove(g_pred, player, cmd, movebuf);
				*s_p_curtime += g_interval;

				st.flags = g_off.flags ? *reinterpret_cast<int*>(pb + g_off.flags) : 0;
				s_sim_frames[i] = f;
				s_sim_states[i] = st;
				prev = st;
			}

			EndStateGuard();
			return n;
		}
		__except (FaultFilter(GetExceptionInformation())) {
			if (s_restore_ready)
				EndStateGuard();
			return -100;
		}
	}

	// Hook for IPrediction::FinishMove. Runs the real move, then one job per
	// frame in the same valid context: a pending editor sim, else the live
	// look-ahead.
	void OnFinishMove(void* thisptr, void* player, void* ucmd, void* movedata) {
		g_orig_finishmove(thisptr, player, ucmd, movedata);   // real movement, unchanged

		if (g_in_hook_work || !g_pred || !g_gm || !g_helper || !g_gpg_holder)
			return;
		// Only the newest command (skip re-predicted history) -> once per frame.
		if (!GetVirtualFunction<bool(*)(void*)>(thisptr, kIsFirstTimePredictedIdx)(thisptr))
			return;

		if (movedata) {
			g_real_move_origin = *reinterpret_cast<Vector*>(reinterpret_cast<char*>(movedata) + kMoveDataOriginOff);
			g_real_move_valid = true;
		}

		ResolveOffsets();

		if (s_sim_pending) {
			LARGE_INTEGER freq, t0, t1;
			QueryPerformanceFrequency(&freq);
			QueryPerformanceCounter(&t0);

			g_in_hook_work = true;
			s_restore_ready = false;
			const int n = RunEditorSimSEH(player);
			g_in_hook_work = false;

			QueryPerformanceCounter(&t1);
			const float sim_ms = freq.QuadPart
				? static_cast<float>(t1.QuadPart - t0.QuadPart) * 1000.f / static_cast<float>(freq.QuadPart)
				: 0.f;

			if (n < 0) {
				if (n == -100) { ++g_fault_count; s_sim_fault = 1; }
				s_sim_count = 0;
			} else {
				s_sim_count = n;
				s_sim_fault = 0;
			}
			s_sim_pending = 0;
			s_sim_ready = 1;

			g_diag.installed = true;
			g_diag.interval_per_tick = g_interval;
			g_diag.sim_ms = sim_ms;
			g_diag.fault_count = g_fault_count;
			g_diag.last_fault_rva = g_last_fault_rva;
			g_diag.last_fault_access = g_last_fault_access;
			return;
		}

		if (!WorldDraw::draw_prediction)
			return;

		int ticks = WorldDraw::pred_ticks;
		if (ticks < 1)   ticks = 1;
		if (ticks > 256) ticks = 256;

		g_in_hook_work = true;
		s_restore_ready = false;
		const int n = RunLookaheadSEH(player, ucmd, ticks);
		g_in_hook_work = false;

		if (n == -100)
			++g_fault_count;
		g_path_count = (n > 0) ? n : 0;

		Prediction::Diag d;
		d.installed = true;
		d.ran = (n > 0);
		d.ticks = (n > 0) ? n : 0;
		d.interval_per_tick = g_interval;
		d.sim_ms = g_diag.sim_ms;   // keep the last editor-sim timing visible
		d.fault_count = g_fault_count;
		d.last_fault_rva = g_last_fault_rva;
		d.last_fault_access = g_last_fault_access;
		g_diag = d;
	}
}

void Prediction::Install() {
	HMODULE client = GetModuleHandleA("client.dll");
	if (!client)
		return;

	g_client_base = reinterpret_cast<uintptr_t>(client);
	g_gm   = GetInterface<void>("client.dll", "GameMovement001");
	g_pred = GetInterface<void>("client.dll", "VClientPrediction001");
	g_gpg_holder = reinterpret_cast<void**>(g_client_base + kGpGlobalsPtrRva);

	// Only trust the move-helper singleton if its vtable pointer matches the one
	// we RE'd; a stale RVA (game update) fails here instead of crashing later.
	void* helper = reinterpret_cast<void*>(g_client_base + kMoveHelperRva);
	if (*reinterpret_cast<uintptr_t*>(helper) == g_client_base + kMoveHelperVtableRva)
		g_helper = helper;

	Diag d;
	if (g_pred) {
		g_pred_hook = std::make_unique<VMTHook>(g_pred);
		g_orig_finishmove = g_pred_hook->GetOriginalFunction<FinishMoveFn>(kFinishMoveIdx);
		g_pred_hook->HookFunction(reinterpret_cast<void*>(&OnFinishMove), kFinishMoveIdx);
		d.installed = (g_orig_finishmove != nullptr);
	}
	g_diag = d;
}

void Prediction::GetPath(std::vector<Vector>& out) {
	out.assign(g_path, g_path + g_path_count);
}

bool Prediction::RequestSim(const StartState& anchor, int ticks, SimFrameFn provider) {
	if (s_sim_pending || !provider || !anchor.valid || ticks < 1)
		return false;
	if (!g_pred || !g_gm || !g_helper)
		return false;

	ResolveOffsets();
	if (!g_off.origin || !g_off.velocity)
		return false;

	if (ticks > kMaxSimTicks)
		ticks = kMaxSimTicks;

	s_sim_anchor    = anchor;
	s_sim_req_ticks = ticks;
	s_sim_provider  = provider;
	s_sim_ready     = 0;
	s_sim_pending   = 1;
	return true;
}

bool Prediction::SimBusy()    { return s_sim_pending != 0; }
bool Prediction::SimReady()   { return s_sim_ready != 0; }
bool Prediction::SimFaulted() { return s_sim_fault != 0; }
int  Prediction::SimCount()   { return s_sim_count; }

void Prediction::TakeSim(Frame* frames, SimState* states) {
	if (!s_sim_ready)
		return;
	if (frames && s_sim_count > 0)
		memcpy(frames, s_sim_frames, sizeof(Frame) * s_sim_count);
	if (states && s_sim_count > 0)
		memcpy(states, s_sim_states, sizeof(SimState) * s_sim_count);
	s_sim_ready = 0;
}

bool Prediction::CaptureStartState(StartState& out) {
	if (!engine || !entitylist || !engine->IsInGame())
		return false;
	void* player = entitylist->GetClientEntity(engine->GetLocalPlayer());
	if (!player)
		return false;

	ResolveOffsets();
	if (!g_off.origin || !g_off.velocity)
		return false;

	out.origin   = NetVars::Get<Vector>(player, g_off.origin);
	out.velocity = NetVars::Get<Vector>(player, g_off.velocity);
	const int flags = g_off.flags ? NetVars::Get<int>(player, g_off.flags) : 0;
	out.ducked  = (flags & FL_DUCKING) != 0;
	out.pitch   = g_off.eye_angles ? NetVars::Get<float>(player, g_off.eye_angles) : 0.f;
	out.yaw     = g_off.eye_angles ? NetVars::Get<float>(player, g_off.eye_angles + 4) : 0.f;
	out.stamina = g_off.stamina ? NetVars::Get<float>(player, g_off.stamina) : 0.f;
	out.valid = true;
	return true;
}

bool Prediction::LastRealMoveOrigin(Vector& out) {
	if (!g_real_move_valid)
		return false;
	out = g_real_move_origin;
	return true;
}

Prediction::Diag Prediction::LastDiag() { return g_diag; }
