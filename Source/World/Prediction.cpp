#include "Prediction.h"

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
	extern float pred_forwardmove;
	extern float pred_sidemove;
	extern bool  pred_jump;
	extern bool  pred_duck;
}

// ---------------------------------------------------------------------------
// Constants pinned by static RE of cstrike/bin/x64/client.dll (see prediction-re
// memory). We run inside the engine's valid prediction context (hooked
// FinishMove), so no player context has to be faked - only save/restore of what
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

	// Output + state.
	Vector g_path[257];
	int    g_path_count = 0;
	bool   g_in_lookahead = false;
	float  g_interval = 0.f;
	int    g_fault_count = 0;
	unsigned long long g_last_fault_rva = 0;
	unsigned long long g_last_fault_access = 0;
	Prediction::Diag g_diag;

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

	template <int Idx, typename... Args>
	void CallV(void* obj, Args... args) {
		using Fn = void(*)(void*, Args...);
		GetVirtualFunction<Fn>(obj, Idx)(obj, args...);
	}

	// Minimal fault net: record where a fault landed (for diagnosis), then recover.
	long FaultFilter(EXCEPTION_POINTERS* ep) {
		const EXCEPTION_RECORD* r = ep->ExceptionRecord;
		const uintptr_t rip = reinterpret_cast<uintptr_t>(r->ExceptionAddress);
		g_last_fault_rva = (g_client_base && rip >= g_client_base) ? (rip - g_client_base) : rip;
		g_last_fault_access = (r->ExceptionCode == EXCEPTION_ACCESS_VIOLATION && r->NumberParameters >= 2)
			? r->ExceptionInformation[1] : 0;
		return EXCEPTION_EXECUTE_HANDLER;
	}

	// Run `ticks` hypothetical ticks from the player's current state, holding the
	// input in `ucmd`. Returns the number of points written to g_path, or <0 on
	// failure. Fully restores the player, globals, and touch list.
	int RunLookaheadSEH(void* player, void* ucmd, int ticks) {
		__try {
			void* gpg = *g_gpg_holder;
			if (!gpg) return -1;
			s_p_frametime = reinterpret_cast<float*>(reinterpret_cast<char*>(gpg) + kFrametimeOff);
			s_p_curtime   = reinterpret_cast<float*>(reinterpret_cast<char*>(gpg) + kCurtimeOff);
			const float interval = *reinterpret_cast<float*>(reinterpret_cast<char*>(gpg) + kIntervalOff);
			g_interval = interval;
			if (interval <= 0.f || interval > 1.f) return -2;

			s_player = player;
			memcpy(s_snapshot, player, kPlayerSize);
			s_save_frametime = *s_p_frametime;
			s_save_curtime   = *s_p_curtime;
			s_p_touch    = reinterpret_cast<int*>(reinterpret_cast<char*>(g_helper) + kTouchCountOff);
			s_save_touch = *s_p_touch;
			s_restore_ready = true;

			*s_p_frametime = interval;   // drive movement at one tick

			// Base our command on the live one (valid vtable + current input).
			alignas(16) unsigned char cmdbuf[sizeof(CUserCmd)];
			memcpy(cmdbuf, ucmd, sizeof(CUserCmd));
			CUserCmd* cmd = reinterpret_cast<CUserCmd*>(cmdbuf);
			if (!WorldDraw::pred_live_input) {
				cmd->forwardmove = WorldDraw::pred_forwardmove;
				cmd->sidemove    = WorldDraw::pred_sidemove;
				cmd->buttons     = (WorldDraw::pred_jump ? IN_JUMP : 0) | (WorldDraw::pred_duck ? IN_DUCK : 0);
			}

			unsigned char movebuf[kMoveDataBufSize];
			for (int i = 0; i < ticks; ++i) {
				memset(movebuf, 0, sizeof(movebuf));
				cmd->command_number = i + 1;
				cmd->tick_count     = i + 1;

				CallV<kSetupMoveIdx>(g_pred, player, cmd, g_helper, movebuf);
				CallV<kProcessMovementIdx>(g_gm, player, movebuf);
				g_path[i] = *reinterpret_cast<Vector*>(movebuf + kMoveDataOriginOff);
				g_orig_finishmove(g_pred, player, cmd, movebuf);   // original: avoid our hook
				*s_p_curtime += interval;
			}

			// Restore everything we disturbed (discard touches our look-ahead queued).
			memcpy(player, s_snapshot, kPlayerSize);
			*s_p_frametime = s_save_frametime;
			*s_p_curtime   = s_save_curtime;
			*s_p_touch     = s_save_touch;
			return ticks;
		}
		__except (FaultFilter(GetExceptionInformation())) {
			if (s_restore_ready) {
				memcpy(s_player, s_snapshot, kPlayerSize);
				if (s_p_frametime) *s_p_frametime = s_save_frametime;
				if (s_p_curtime)   *s_p_curtime   = s_save_curtime;
				if (s_p_touch)     *s_p_touch     = s_save_touch;
			}
			return -100;
		}
	}

	// Hook for IPrediction::FinishMove. Runs the real move, then (once per frame)
	// the look-ahead in the same valid context.
	void OnFinishMove(void* thisptr, void* player, void* ucmd, void* movedata) {
		g_orig_finishmove(thisptr, player, ucmd, movedata);   // real movement, unchanged

		if (!WorldDraw::draw_prediction || g_in_lookahead || !g_pred || !g_gm || !g_helper || !g_gpg_holder)
			return;
		// Only the newest command (skip re-predicted history) -> once per frame.
		if (!GetVirtualFunction<bool(*)(void*)>(thisptr, kIsFirstTimePredictedIdx)(thisptr))
			return;

		int ticks = WorldDraw::pred_ticks;
		if (ticks < 1)   ticks = 1;
		if (ticks > 256) ticks = 256;

		s_restore_ready = false;
		g_in_lookahead = true;
		const int n = RunLookaheadSEH(player, ucmd, ticks);
		g_in_lookahead = false;

		if (n == -100)
			++g_fault_count;
		g_path_count = (n > 0) ? n : 0;

		Prediction::Diag d;
		d.installed = true;
		d.ran = (n > 0);
		d.ticks = (n > 0) ? n : 0;
		d.interval_per_tick = g_interval;
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

Prediction::Diag Prediction::LastDiag() { return g_diag; }
