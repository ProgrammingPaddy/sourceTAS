#include "Prediction.h"
#include "NetVars.h"
#include "BspWorld.h"
#include "../Menu/Breadcrumb.h"

#include <cstdint>
#include <cstdio>
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
	// Raw + leaked: the unique_ptr's atexit destructor restored the vtable
	// into game memory that was already gone at process exit (crash.log
	// 2026-07-24, ??__Fg_pred_hook thunk). Nothing ever unloads this DLL
	// mid-game, so the hook lives exactly as long as the process.
	VMTHook* g_pred_hook = nullptr;
	using FinishMoveFn = void(*)(void*, void*, void*, void*);
	FinishMoveFn g_orig_finishmove = nullptr;

	// Live look-ahead output.
	Vector g_path[257];
	int    g_path_count = 0;

	// Origin of the newest REAL command's movedata (basis diagnostic: lets the
	// menu compare the movement pipeline's origin against the netvar origin).
	Vector g_real_move_origin;
	float  g_real_maxspeed = 0.f;
	bool   g_real_move_valid = false;
	int    g_pred_flags = 0;          // player m_fFlags after the newest
	bool   g_pred_flags_valid = false;// first-time-predicted command
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
		int mins = 0, maxs = 0;
		int gravity = 0, basevel = 0;
		// THE ground truth, literally: SetGroundEntity writes this handle.
		// FL_ONGROUND lags it on the client - measured twice now (the engine
		// jumped from a state whose flag read airborne, and an isolated
		// CategorizePosition set ground on 0 of 72 states the engine itself
		// had recorded as grounded while the handle was the only thing it
		// actually wrote).
		int groundent = 0;
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
		// CCollisionProperty's bounds - the hull the engine actually sweeps.
		g_off.mins       = NetVars::Offset("DT_CSPlayer", "m_vecMins");
		g_off.maxs       = NetVars::Offset("DT_CSPlayer", "m_vecMaxs");
		// Player gravity scale (trigger_gravity writes it; CGameMovement reads
		// it every tick). Networked in DT_BasePlayer; try the leaf class first
		// in case the walk is flat.
		g_off.gravity    = NetVars::Offset("DT_CSPlayer", "m_flGravity");
		if (!g_off.gravity)
			g_off.gravity = NetVars::Offset("DT_BasePlayer", "m_flGravity");
		// Base velocity (trigger_push boosters write it; the movement code
		// applies + decays it natively, so the push physics stay engine code).
		g_off.basevel    = NetVars::Offset("DT_CSPlayer", "m_vecBaseVelocity");
		if (!g_off.basevel)
			g_off.basevel = NetVars::Offset("DT_BasePlayer", "m_vecBaseVelocity");
		g_off.groundent  = NetVars::Offset("DT_CSPlayer", "m_hGroundEntity");
		if (!g_off.groundent)
			g_off.groundent = NetVars::Offset("DT_BasePlayer", "m_hGroundEntity");
		g_off.init = true;
	}

	// Trigger events fired by the last editor sim (whole-player snapshot
	// restore makes the gravity write safe; teleports rewrite origin/velocity
	// mid-sim exactly like the server's Touch would).
	Prediction::TriggerEvent s_trig_events[64];
	int s_trig_event_count = 0;

	// Last STANDING hull read off the live player. Cached because the live
	// value shrinks while ducked and the model simulates standing.
	Vector g_hull_min(-16.f, -16.f, 0.f);
	Vector g_hull_max(16.f, 16.f, 72.f);
	bool   g_hull_measured = false;

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

				// Triggers apply to the live look-ahead too (same rules as the
				// editor sim; no event recording here).
				{
					const int fl2 = g_off.flags
						? *reinterpret_cast<int*>(reinterpret_cast<char*>(player) + g_off.flags) : 0;
					Vector hmin = g_hull_min, hmax = g_hull_max;
					if (fl2 & FL_DUCKING)
						hmax.Z = 54.f;
					BspWorld::TriggerHit th;
					if (BspWorld::CheckTriggers(g_path[i], hmin, hmax, &th)) {
						char* pb2 = reinterpret_cast<char*>(player);
						if (th.grav_touched && g_off.gravity)
							*reinterpret_cast<float*>(pb2 + g_off.gravity) = th.gravity;
						if (th.pushed && g_off.basevel && g_off.flags) {
							int fl3 = *reinterpret_cast<int*>(pb2 + g_off.flags);
							Vector push = th.push_vec;
							if (fl3 & FL_BASEVELOCITY) {
								const Vector& bv = *reinterpret_cast<Vector*>(pb2 + g_off.basevel);
								push.X += bv.X; push.Y += bv.Y; push.Z += bv.Z;
							}
							if (push.Z > 0.f && (fl3 & FL_ONGROUND)) {
								fl3 &= ~FL_ONGROUND;
								reinterpret_cast<Vector*>(pb2 + g_off.origin)->Z += 1.f;
							}
							*reinterpret_cast<Vector*>(pb2 + g_off.basevel) = push;
							*reinterpret_cast<int*>(pb2 + g_off.flags) = fl3 | FL_BASEVELOCITY;
						}
						if (th.teleported) {
							g_path[i] = th.tp_origin;
							*reinterpret_cast<Vector*>(pb2 + g_off.origin) = th.tp_origin;
							*reinterpret_cast<Vector*>(pb2 + g_off.velocity) = Vector(0.f, 0.f, 0.f);
						}
					}
				}
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
	// ---- FUNCTION-LEVEL DIFFERENTIAL FUZZ ---------------------------------
	// m_surfaceFriction on the CLIENT player (binary-decoded 2026-08-15:
	// client.dll @0011adb4 / @0011fb81 - movss [player+0x1868] then mulss by
	// sv_friction's parent value, the same Friction signature that pinned the
	// server's +0x36C). Not a netvar, so it needs the literal offset; it is
	// an explicit fuzz input AND output so the friction rule is measured, not
	// assumed.
	constexpr int kSurfFricOff = 0x1868;
	// Tick 0 is a SETTLE tick: its output is the engine's own fully-derived
	// state (real ground entity, re-derived duck state, real surface
	// friction) and becomes the seed BOTH sides start from. Measured
	// 2026-08-15: writing FL_ONGROUND does NOT ground the player (ground is
	// m_hGroundEntity, a handle - 96% of "grounded" probes were airborne),
	// and the engine re-derives duck state on a third of probes. Asserting
	// an initial state was the harness lying; ticks 1..K are the test.
	constexpr int kFuzzTicks = 5;

	struct FuzzProbeIn {
		float ox, oy, oz, vx, vy, vz, bx, by, bz;
		int   onground, ducked, ducking;
		float ducktime, stamina, gravity, sfric;
		float hullmin_z, hullmax_z, hull_xy;
		float yaw[kFuzzTicks], fmove[kFuzzTicks], smove[kFuzzTicks];
		int   buttons[kFuzzTicks];
	};
	struct FuzzTickOut {
		float ox, oy, oz, vx, vy, vz, bx, by, bz;
		int   flags, ducked, ducking;
		float ducktime, stamina, sfric, maxz;
		int   ok;
	};

	// ONE probe, POD-only so SEH can wrap it: write the arbitrary state, run
	// kFuzzTicks of engine movement, record every output field per tick, then
	// restore the player. A faulting probe is marked and the batch continues.
	int RunOneFuzzProbeSEH(void* player, const FuzzProbeIn* in,
	                       FuzzTickOut* out) {
		__try {
			if (!BeginStateGuard(player))
				return 0;
			char* pb = reinterpret_cast<char*>(player);
			if (g_off.origin)
				*reinterpret_cast<Vector*>(pb + g_off.origin) =
					Vector(in->ox, in->oy, in->oz);
			if (g_off.velocity)
				*reinterpret_cast<Vector*>(pb + g_off.velocity) =
					Vector(in->vx, in->vy, in->vz);
			if (g_off.basevel)
				*reinterpret_cast<Vector*>(pb + g_off.basevel) =
					Vector(in->bx, in->by, in->bz);
			if (g_off.flags) {
				int fl = *reinterpret_cast<int*>(pb + g_off.flags);
				fl = in->onground ? (fl | FL_ONGROUND) : (fl & ~FL_ONGROUND);
				fl = in->ducked ? (fl | FL_DUCKING) : (fl & ~FL_DUCKING);
				if (in->bx != 0.f || in->by != 0.f || in->bz != 0.f)
					fl |= FL_BASEVELOCITY;
				else
					fl &= ~FL_BASEVELOCITY;
				*reinterpret_cast<int*>(pb + g_off.flags) = fl;
			}
			if (g_off.ducked)   *reinterpret_cast<bool*>(pb + g_off.ducked) = in->ducked != 0;
			if (g_off.ducking)  *reinterpret_cast<bool*>(pb + g_off.ducking) = in->ducking != 0;
			if (g_off.ducktime) *reinterpret_cast<float*>(pb + g_off.ducktime) = in->ducktime;
			if (g_off.stamina)  *reinterpret_cast<float*>(pb + g_off.stamina) = in->stamina;
			if (g_off.gravity)  *reinterpret_cast<float*>(pb + g_off.gravity) = in->gravity;
			*reinterpret_cast<float*>(pb + kSurfFricOff) = in->sfric;
			if (g_off.mins)
				*reinterpret_cast<Vector*>(pb + g_off.mins) =
					Vector(-in->hull_xy, -in->hull_xy, in->hullmin_z);
			if (g_off.maxs)
				*reinterpret_cast<Vector*>(pb + g_off.maxs) =
					Vector(in->hull_xy, in->hull_xy, in->hullmax_z);

			alignas(16) unsigned char cmdbuf[sizeof(CUserCmd)];
			CUserCmd* cmd = reinterpret_cast<CUserCmd*>(cmdbuf);
			unsigned char movebuf[kMoveDataBufSize];

			for (int t = 0; t < kFuzzTicks; ++t) {
				memset(cmdbuf, 0, sizeof(cmdbuf));
				memset(movebuf, 0, sizeof(movebuf));
				cmd->command_number = t + 1;
				cmd->tick_count     = t + 1;
				cmd->viewangles     = QAngle(0.f, in->yaw[t], 0.f);
				cmd->forwardmove    = in->fmove[t];
				cmd->sidemove       = in->smove[t];
				cmd->upmove         = 0.f;
				cmd->buttons        = in->buttons[t];

				CallV<kSetupMoveIdx>(g_pred, player, cmd, g_helper, movebuf);
				if (t == 0) {
					// SetupMove copies ABS velocity/origin; seed the movedata
					// directly so the probe's exact state is what moves.
					*reinterpret_cast<Vector*>(movebuf + kMoveDataVelOff) =
						Vector(in->vx, in->vy, in->vz);
					*reinterpret_cast<Vector*>(movebuf + kMoveDataOriginOff) =
						Vector(in->ox, in->oy, in->oz);
				}
				CallV<kProcessMovementIdx>(g_gm, player, movebuf);
				g_orig_finishmove(g_pred, player, cmd, movebuf);
				*s_p_curtime += g_interval;

				FuzzTickOut& o = out[t];
				const Vector& mo = *reinterpret_cast<Vector*>(movebuf + kMoveDataOriginOff);
				const Vector& mv = *reinterpret_cast<Vector*>(movebuf + kMoveDataVelOff);
				o.ox = mo.X; o.oy = mo.Y; o.oz = mo.Z;
				o.vx = mv.X; o.vy = mv.Y; o.vz = mv.Z;
				o.flags   = g_off.flags ? *reinterpret_cast<int*>(pb + g_off.flags) : 0;
				o.ducked  = g_off.ducked ? (*reinterpret_cast<bool*>(pb + g_off.ducked) ? 1 : 0) : 0;
				o.ducking = g_off.ducking ? (*reinterpret_cast<bool*>(pb + g_off.ducking) ? 1 : 0) : 0;
				o.ducktime = g_off.ducktime ? *reinterpret_cast<float*>(pb + g_off.ducktime) : 0.f;
				o.stamina  = g_off.stamina ? *reinterpret_cast<float*>(pb + g_off.stamina) : 0.f;
				o.sfric    = *reinterpret_cast<float*>(pb + kSurfFricOff);
				o.maxz     = g_off.maxs ? reinterpret_cast<Vector*>(pb + g_off.maxs)->Z : -1.f;
				const Vector bvv = g_off.basevel
					? *reinterpret_cast<Vector*>(pb + g_off.basevel) : Vector(0.f, 0.f, 0.f);
				o.bx = bvv.X; o.by = bvv.Y; o.bz = bvv.Z;
				o.ok = 1;
			}
			EndStateGuard();
			return 1;
		}
		__except (FaultFilter(GetExceptionInformation())) {
			if (s_restore_ready)
				EndStateGuard();
			return 0;
		}
	}

	// ---- FUNCPROBE ---------------------------------------------------------
	// CGameMovement member offsets, confirmed by disassembly (client.dll
	// CategorizePosition @0x1174f0): "mov rax,[rcx+8]" -> player, and
	// "mov rax,[rsi+0x10]" -> mv. The latch below re-proves both every run
	// against pointers the live hook already knows, so an update that moves
	// them fails closed instead of answering with garbage.
	constexpr int kGmPlayerOff = 0x08;
	constexpr int kGmMvOff     = 0x10;
	int g_gm_ctx_gate = -1;   // -1 unknown, 1 verified, 0 mismatch

	// Supported call shapes. Every CGameMovement method we pin takes only
	// `this` in RCX; the difference is whether it returns a value.
	enum FuncAbi { kAbiVoidThis = 0, kAbiIntThis = 1 };

	struct FuncPin {
		char name[48];
		unsigned rva;
		int abi;
		// Optional PRELUDE: a function called first, on the same context, to
		// establish state we cannot write directly. CheckJumpButton returns
		// immediately unless the player has a ground ENTITY, and a valid
		// EHANDLE cannot be synthesised for an arbitrary probe - so
		// CategorizePosition (already proven 99.99% in isolation) is called
		// first to derive ground from geometry. The prelude is part of the
		// declared input, not a hidden fixup.
		unsigned prelude_rva;
	};
	struct FuncProbeIn {
		int pin;                     // index into pins
		float ox, oy, oz, vx, vy, vz, bx, by, bz;
		int   onground, ducked, ducking, buttons;
		float ducktime, stamina, sfric, gravity, hullmax_z, yaw, fmove, smove;
	};
	struct FuncProbeOut {
		float ox, oy, oz, vx, vy, vz;
		int   flags, ducked, ducking, ret;
		float ducktime, stamina, sfric, maxz;
		int   groundent;   // raw m_hGroundEntity handle: -1/0xFFFFFFFF = none
		int   ok;
	};

	std::vector<FuncPin>      s_fp_pins;
	std::vector<FuncProbeIn>  s_fp_probes;
	std::vector<FuncProbeOut> s_fp_outs;
	std::string s_fp_out_path;
	volatile int s_fp_pending = 0;
	int s_fp_next = 0;
	int s_fp_ok = 0;

	// One isolated call: install our state into the movedata + player, point
	// the CGameMovement context at them, invoke the pinned function, read
	// every observable back. POD-only so SEH can wrap it.
	int RunOneFuncProbeSEH(void* player, const FuncPin* pin,
	                       const FuncProbeIn* in, FuncProbeOut* out) {
		__try {
			if (!BeginStateGuard(player))
				return 0;
			char* pb = reinterpret_cast<char*>(player);
			char* gm = reinterpret_cast<char*>(g_gm);

			// Player-side inputs.
			if (g_off.origin)
				*reinterpret_cast<Vector*>(pb + g_off.origin) = Vector(in->ox, in->oy, in->oz);
			if (g_off.velocity)
				*reinterpret_cast<Vector*>(pb + g_off.velocity) = Vector(in->vx, in->vy, in->vz);
			if (g_off.basevel)
				*reinterpret_cast<Vector*>(pb + g_off.basevel) = Vector(in->bx, in->by, in->bz);
			if (g_off.flags) {
				int fl = *reinterpret_cast<int*>(pb + g_off.flags);
				fl = in->onground ? (fl | FL_ONGROUND) : (fl & ~FL_ONGROUND);
				fl = in->ducked ? (fl | FL_DUCKING) : (fl & ~FL_DUCKING);
				*reinterpret_cast<int*>(pb + g_off.flags) = fl;
			}
			if (g_off.ducked)   *reinterpret_cast<bool*>(pb + g_off.ducked) = in->ducked != 0;
			if (g_off.ducking)  *reinterpret_cast<bool*>(pb + g_off.ducking) = in->ducking != 0;
			if (g_off.ducktime) *reinterpret_cast<float*>(pb + g_off.ducktime) = in->ducktime;
			if (g_off.stamina)  *reinterpret_cast<float*>(pb + g_off.stamina) = in->stamina;
			if (g_off.gravity)  *reinterpret_cast<float*>(pb + g_off.gravity) = in->gravity;
			*reinterpret_cast<float*>(pb + kSurfFricOff) = in->sfric;
			// GROUND INPUT: always cleared to "none". Writing FL_ONGROUND is
			// a no-op (the flag is a shadow of m_hGroundEntity, measured: an
			// isolated call set the flag on 0 of 72 known-grounded states),
			// and there is no honest way to synthesise a VALID EHANDLE for
			// an arbitrary probe. So every probe starts ungrounded and the
			// function's job is to SET ground from geometry; the differ
			// seeds our model the same way.
			if (g_off.groundent)
				*reinterpret_cast<int*>(pb + g_off.groundent) = -1;
			if (g_off.mins)
				*reinterpret_cast<Vector*>(pb + g_off.mins) = Vector(-16.f, -16.f, 0.f);
			if (g_off.maxs)
				*reinterpret_cast<Vector*>(pb + g_off.maxs) = Vector(16.f, 16.f, in->hullmax_z);

			// Movedata: build a real one through SetupMove, then overwrite
			// the fields under test so the function sees exactly our input.
			alignas(16) unsigned char cmdbuf[sizeof(CUserCmd)];
			CUserCmd* cmd = reinterpret_cast<CUserCmd*>(cmdbuf);
			unsigned char movebuf[kMoveDataBufSize];
			memset(cmdbuf, 0, sizeof(cmdbuf));
			memset(movebuf, 0, sizeof(movebuf));
			cmd->command_number = 1;
			cmd->tick_count     = 1;
			cmd->viewangles     = QAngle(0.f, in->yaw, 0.f);
			cmd->forwardmove    = in->fmove;
			cmd->sidemove       = in->smove;
			cmd->buttons        = in->buttons;
			CallV<kSetupMoveIdx>(g_pred, player, cmd, g_helper, movebuf);
			*reinterpret_cast<Vector*>(movebuf + kMoveDataOriginOff) =
				Vector(in->ox, in->oy, in->oz);
			*reinterpret_cast<Vector*>(movebuf + kMoveDataVelOff) =
				Vector(in->vx, in->vy, in->vz);

			// Point the movement context at our player + movedata, call the
			// isolated function, then put the context back exactly.
			void* save_player = *reinterpret_cast<void**>(gm + kGmPlayerOff);
			void* save_mv     = *reinterpret_cast<void**>(gm + kGmMvOff);
			*reinterpret_cast<void**>(gm + kGmPlayerOff) = player;
			*reinterpret_cast<void**>(gm + kGmMvOff)     = movebuf;

			int ret = 0;
			if (pin->prelude_rva)
				reinterpret_cast<void(*)(void*)>(
					g_client_base + pin->prelude_rva)(g_gm);
			const uintptr_t fn = g_client_base + pin->rva;
			if (pin->abi == kAbiIntThis)
				ret = reinterpret_cast<int(*)(void*)>(fn)(g_gm);
			else
				reinterpret_cast<void(*)(void*)>(fn)(g_gm);

			*reinterpret_cast<void**>(gm + kGmPlayerOff) = save_player;
			*reinterpret_cast<void**>(gm + kGmMvOff)     = save_mv;

			const Vector& mo = *reinterpret_cast<Vector*>(movebuf + kMoveDataOriginOff);
			const Vector& mvv = *reinterpret_cast<Vector*>(movebuf + kMoveDataVelOff);
			out->ox = mo.X; out->oy = mo.Y; out->oz = mo.Z;
			out->vx = mvv.X; out->vy = mvv.Y; out->vz = mvv.Z;
			out->flags    = g_off.flags ? *reinterpret_cast<int*>(pb + g_off.flags) : 0;
			out->ducked   = g_off.ducked ? (*reinterpret_cast<bool*>(pb + g_off.ducked) ? 1 : 0) : 0;
			out->ducking  = g_off.ducking ? (*reinterpret_cast<bool*>(pb + g_off.ducking) ? 1 : 0) : 0;
			out->ducktime = g_off.ducktime ? *reinterpret_cast<float*>(pb + g_off.ducktime) : 0.f;
			out->stamina  = g_off.stamina ? *reinterpret_cast<float*>(pb + g_off.stamina) : 0.f;
			out->sfric    = *reinterpret_cast<float*>(pb + kSurfFricOff);
			out->maxz     = g_off.maxs ? reinterpret_cast<Vector*>(pb + g_off.maxs)->Z : -1.f;
			out->groundent = g_off.groundent
				? *reinterpret_cast<int*>(pb + g_off.groundent) : 0;
			out->ret      = ret;
			out->ok       = 1;
			EndStateGuard();
			return 1;
		}
		__except (FaultFilter(GetExceptionInformation())) {
			if (s_restore_ready)
				EndStateGuard();
			return 0;
		}
	}

	void WriteFuncResults() {
		FILE* fo = nullptr;
		if (fopen_s(&fo, s_fp_out_path.c_str(), "w") != 0 || !fo)
			return;
		fprintf(fo, "# funcprobe v1 ctxgate %d\n", g_gm_ctx_gate);
		fprintf(fo, "id,fn,ox,oy,oz,vx,vy,vz,flags,ducked,ducking,ducktime,"
			"stamina,sfric,maxz,ret,ok,groundent\n");
		for (size_t i = 0; i < s_fp_probes.size(); ++i) {
			const FuncProbeOut& o = s_fp_outs[i];
			const int pi = s_fp_probes[i].pin;
			fprintf(fo, "%d,%s,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%d,%d,%d,"
				"%.9g,%.9g,%.9g,%.9g,%d,%d,%d\n",
				static_cast<int>(i),
				(pi >= 0 && pi < static_cast<int>(s_fp_pins.size()))
					? s_fp_pins[pi].name : "?",
				o.ox, o.oy, o.oz, o.vx, o.vy, o.vz, o.flags, o.ducked,
				o.ducking, o.ducktime, o.stamina, o.sfric, o.maxz,
				o.ret, o.ok, o.groundent);
		}
		fclose(fo);
	}

	// Deferred fuzz job state (filled by RunFuzz, executed in OnFinishMove).
	std::vector<FuzzProbeIn> s_fuzz_probes;
	std::vector<FuzzTickOut> s_fuzz_outs;
	std::string s_fuzz_out_path;
	volatile int s_fuzz_pending = 0;
	int s_fuzz_next = 0;
	int s_fuzz_ok = 0;

	void WriteFuzzResults() {
		FILE* fo = nullptr;
		if (fopen_s(&fo, s_fuzz_out_path.c_str(), "w") != 0 || !fo)
			return;
		fprintf(fo, "id,tick,ox,oy,oz,vx,vy,vz,flags,ducked,ducking,"
			"ducktime,stamina,sfric,maxz,bx,by,bz,ok\n");
		for (size_t i = 0; i < s_fuzz_probes.size(); ++i) {
			for (int t = 0; t < kFuzzTicks; ++t) {
				const FuzzTickOut& o = s_fuzz_outs[i * kFuzzTicks + t];
				fprintf(fo, "%d,%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%d,%d,%d,"
					"%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%d\n",
					static_cast<int>(i), t, o.ox, o.oy, o.oz,
					o.vx, o.vy, o.vz, o.flags, o.ducked, o.ducking,
					o.ducktime, o.stamina, o.sfric, o.maxz,
					o.bx, o.by, o.bz, o.ok);
			}
		}
		fclose(fo);
	}

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

			s_trig_event_count = 0;
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
				if (i == 0) {
					// MEASURED (2026-08-14, slice decks): the netvar
					// velocity write never reaches SetupMove's copy (abs-
					// velocity path) - nonzero-velocity anchors started at
					// v=0. Seed the FIRST tick's movedata directly through
					// the pinned offsets; FinishMove propagates onward.
					*reinterpret_cast<Vector*>(movebuf + kMoveDataVelOff)
						= s_sim_anchor.velocity;
					*reinterpret_cast<Vector*>(movebuf + kMoveDataOriginOff)
						= s_sim_anchor.origin;
				}
				CallV<kProcessMovementIdx>(g_gm, player, movebuf);

				Prediction::SimState st;
				st.origin   = *reinterpret_cast<Vector*>(movebuf + kMoveDataOriginOff);
				st.velocity = *reinterpret_cast<Vector*>(movebuf + kMoveDataVelOff);
				// Direct reads (no layout inference): both CMoveData float
				// candidates around m_flMaxSpeed, raw.
				st.mspd_a = *reinterpret_cast<float*>(movebuf + 0x3C);
				st.mspd_b = *reinterpret_cast<float*>(movebuf + 0x40);

				g_orig_finishmove(g_pred, player, cmd, movebuf);
				*s_p_curtime += g_interval;

				st.flags = g_off.flags ? *reinterpret_cast<int*>(pb + g_off.flags) : 0;
				// The collision hull the engine ACTUALLY carries after this
				// tick (CCollisionProperty m_vecMaxs.z, origin-relative).
				st.hull_top = g_off.maxs
					? reinterpret_cast<Vector*>(pb + g_off.maxs)->Z : -1.f;
				// The engine's own stamina clock after the tick - per-tick
				// observable, so stamina drift is caught at its birth tick.
				st.stamina_ms = g_off.stamina
					? *reinterpret_cast<float*>(pb + g_off.stamina) : -1.f;

				// TRIGGERS: server-side entities the client prediction never
				// runs - fire touched trigger_teleport / trigger_gravity here
				// so the simulated line matches the real run. The whole-player
				// snapshot restore covers every field written.
				{
					Vector hmin = g_hull_min, hmax = g_hull_max;
					if (st.flags & FL_DUCKING)
						hmax.Z = 54.f;   // SDK VEC_DUCK_HULL_MAX height
					BspWorld::TriggerHit th;
					if (BspWorld::CheckTriggers(st.origin, hmin, hmax, &th)) {
						if (th.grav_touched && g_off.gravity) {
							*reinterpret_cast<float*>(pb + g_off.gravity) = th.gravity;
							if (s_trig_event_count < 64) {
								Prediction::TriggerEvent& ev = s_trig_events[s_trig_event_count++];
								ev.tick = i; ev.type = 2;
								ev.to = st.origin; ev.gravity = th.gravity;
							}
						}
						if (th.pushed && g_off.basevel && g_off.flags) {
							// SDK CTriggerPush::Touch for players: accumulate
							// onto an existing base velocity, unground + 1u
							// nudge for upward pushes, set FL_BASEVELOCITY -
							// the engine's own movement applies and decays it.
							int fl2 = *reinterpret_cast<int*>(pb + g_off.flags);
							Vector push = th.push_vec;
							if (fl2 & FL_BASEVELOCITY) {
								const Vector& bv = *reinterpret_cast<Vector*>(pb + g_off.basevel);
								push.X += bv.X; push.Y += bv.Y; push.Z += bv.Z;
							}
							if (push.Z > 0.f && (fl2 & FL_ONGROUND)) {
								fl2 &= ~FL_ONGROUND;
								reinterpret_cast<Vector*>(pb + g_off.origin)->Z += 1.f;
								st.origin.Z += 1.f;
							}
							*reinterpret_cast<Vector*>(pb + g_off.basevel) = push;
							fl2 |= FL_BASEVELOCITY;
							*reinterpret_cast<int*>(pb + g_off.flags) = fl2;
							st.flags = fl2;
							if (s_trig_event_count < 64) {
								Prediction::TriggerEvent& ev = s_trig_events[s_trig_event_count++];
								ev.tick = i; ev.type = 3;
								ev.to = st.origin; ev.gravity = 0.f;
							}
						}
						if (th.teleported) {
							st.origin = th.tp_origin;
							st.velocity = Vector(0.f, 0.f, 0.f);   // SDK zeroes it
							*reinterpret_cast<Vector*>(pb + g_off.origin) = st.origin;
							*reinterpret_cast<Vector*>(pb + g_off.velocity) = st.velocity;
							if (s_trig_event_count < 64) {
								Prediction::TriggerEvent& ev = s_trig_events[s_trig_event_count++];
								ev.tick = i; ev.type = 1;
								ev.to = st.origin; ev.gravity = 0.f;
							}
						}
					}
				}

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
		// Freeze-hunt breadcrumbs (2026-08-05): the frozen sessions' journals
		// show every OTHER instrumented path completing cleanly - this hook
		// runs on the main thread OUTSIDE those paths and fires the first
		// weapon-prediction work right after a click. enter-without-exit
		// after a freeze = the main thread died in here.
		Breadcrumb::Note(Breadcrumb::SlotPred, "pred: enter");
		g_orig_finishmove(thisptr, player, ucmd, movedata);   // real movement, unchanged
		Breadcrumb::Note(Breadcrumb::SlotPred, "pred: orig done");

		if (g_in_hook_work || !g_pred || !g_gm || !g_helper || !g_gpg_holder) {
			Breadcrumb::Note(Breadcrumb::SlotPred, "pred: exit early");
			return;
		}
		// Only the newest command (skip re-predicted history) -> once per frame.
		if (!GetVirtualFunction<bool(*)(void*)>(thisptr, kIsFirstTimePredictedIdx)(thisptr)) {
			Breadcrumb::Note(Breadcrumb::SlotPred, "pred: exit repredict");
			return;
		}

		if (movedata) {
			g_real_move_origin = *reinterpret_cast<Vector*>(reinterpret_cast<char*>(movedata) + kMoveDataOriginOff);
			// Live m_flMaxSpeed (+0x3C, identified by two weapon states:
			// knife 250 / no weapon 260) - the weapon-dependent movement
			// cap, READ from the real command so params never assume it.
			g_real_maxspeed = *reinterpret_cast<float*>(reinterpret_cast<char*>(movedata) + 0x3C);
			g_real_move_valid = true;
		}

		Breadcrumb::Note(Breadcrumb::SlotPred, "pred: resolve");
		ResolveOffsets();

		// Post-move flags of this newest first-time-predicted command: the
		// tick-exact grounded signal. The m_fFlags netvar read at CreateMove
		// time can lag prediction by a tick (the entity is restored to
		// NETWORKED values around each packet), and a one-tick-late autohop
		// press costs a full friction tick on every landing.
		if (player && g_off.flags) {
			g_pred_flags = NetVars::Get<int>(player, g_off.flags);
			g_pred_flags_valid = true;
		}

		// FUNCPROBE CONTEXT LATCH: the real ProcessMovement just ran with
		// this player and this movedata, so if our offsets are right the
		// CGameMovement object must hold exactly those two pointers. This
		// is the zero-cost proof that +0x08/+0x10 survived a game update.
		if (g_gm_ctx_gate < 0 && g_gm && player && movedata) {
			char* g = reinterpret_cast<char*>(g_gm);
			g_gm_ctx_gate =
				(*reinterpret_cast<void**>(g + kGmPlayerOff) == player
				 && *reinterpret_cast<void**>(g + kGmMvOff) == movedata)
				? 1 : 0;
		}

		// FUNCPROBE: one engine function per call, chunked like the fuzz.
		if (s_fp_pending && player) {
			Breadcrumb::Note(Breadcrumb::SlotPred, "pred: funcprobe chunk");
			g_in_hook_work = true;
			constexpr int kFpChunk = 400;
			int did = 0;
			while (s_fp_next < static_cast<int>(s_fp_probes.size())
				&& did < kFpChunk) {
				const FuncProbeIn& q = s_fp_probes[s_fp_next];
				s_restore_ready = false;
				s_fp_ok += RunOneFuncProbeSEH(player, &s_fp_pins[q.pin], &q,
					&s_fp_outs[s_fp_next]);
				s_fp_next++;
				did++;
			}
			g_in_hook_work = false;
			if (s_fp_next >= static_cast<int>(s_fp_probes.size())) {
				WriteFuncResults();
				s_fp_pending = 0;
			}
			return;
		}

		// FUNCTION FUZZ runs HERE, not from the UI thread: ProcessMovement is
		// only valid inside this hook's context (the first attempt drove it
		// from the ImGui button and every probe faulted). Chunked so a huge
		// corpus costs a few frames instead of one long hitch.
		if (s_fuzz_pending && player) {
			Breadcrumb::Note(Breadcrumb::SlotPred, "pred: fuzz chunk");
			g_in_hook_work = true;
			constexpr int kChunk = 400;
			int did = 0;
			while (s_fuzz_next < static_cast<int>(s_fuzz_probes.size())
				&& did < kChunk) {
				FuzzTickOut* o = &s_fuzz_outs[
					static_cast<size_t>(s_fuzz_next) * kFuzzTicks];
				s_restore_ready = false;
				s_fuzz_ok += RunOneFuzzProbeSEH(player,
					&s_fuzz_probes[s_fuzz_next], o);
				s_fuzz_next++;
				did++;
			}
			g_in_hook_work = false;
			if (s_fuzz_next >= static_cast<int>(s_fuzz_probes.size())) {
				WriteFuzzResults();
				s_fuzz_pending = 0;
			}
			Breadcrumb::Note(Breadcrumb::SlotPred, "pred: fuzz chunk done");
			return;
		}

		if (s_sim_pending) {
			Breadcrumb::Note(Breadcrumb::SlotPred, "pred: editor sim");
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
			Breadcrumb::Note(Breadcrumb::SlotPred, "pred: exit sim");
			return;
		}

		if (!WorldDraw::draw_prediction) {
			Breadcrumb::Note(Breadcrumb::SlotPred, "pred: exit");
			return;
		}

		Breadcrumb::Note(Breadcrumb::SlotPred, "pred: lookahead");
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
		Breadcrumb::Note(Breadcrumb::SlotPred, "pred: exit lookahead");
	}
}

int Prediction::TriggerEventCount() {
	return s_trig_event_count;
}

const Prediction::TriggerEvent* Prediction::TriggerEventAt(int i) {
	if (i < 0 || i >= s_trig_event_count)
		return nullptr;
	return &s_trig_events[i];
}

float Prediction::CurTime() {
	if (!g_gpg_holder)
		return -1.f;
	if (s_p_curtime)
		return *s_p_curtime;
	void* gpg = *g_gpg_holder;
	if (!gpg)
		return -1.f;
	return *reinterpret_cast<float*>(reinterpret_cast<char*>(gpg) + kCurtimeOff);
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
		g_pred_hook = new VMTHook(g_pred);
		g_orig_finishmove = g_pred_hook->GetOriginalFunction<FinishMoveFn>(kFinishMoveIdx);
		g_pred_hook->HookFunction(reinterpret_cast<void*>(&OnFinishMove), kFinishMoveIdx);
		d.installed = (g_orig_finishmove != nullptr);
	}
	g_diag = d;
}

void Prediction::GetPath(std::vector<Vector>& out) {
	out.assign(g_path, g_path + g_path_count);
}

// FUNCTION-LEVEL DIFFERENTIAL FUZZ: read arbitrary probe states, run each
// through the real engine movement, write every output field. Must run on
// the game thread with a valid player (called from the FinishMove hook).
int Prediction::RunFuzz(const char* probe_path, const char* out_path) {
	if (!g_pred || !g_gm || !g_helper || !probe_path || !out_path)
		return -1;
	void* player = entitylist->GetClientEntity(engine->GetLocalPlayer());
	if (!player)
		return -1;
	ResolveOffsets();
	if (!g_off.origin || !g_off.velocity)
		return -1;

	std::vector<FuzzProbeIn> probes;
	{
		FILE* f = nullptr;
		if (fopen_s(&f, probe_path, "r") != 0 || !f)
			return -1;
		char line[1024];
		while (fgets(line, sizeof(line), f)) {
			if (line[0] == '#' || line[0] == 'i')   // comment / header
				continue;
			FuzzProbeIn p = {};
			int id = 0;
			const int n = sscanf_s(line,
				"%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d,%d,%f,%f,%f,%f,%f,%f,%f,"
				"%f,%f,%f,%d,%f,%f,%f,%d,%f,%f,%f,%d,%f,%f,%f,%d,%f,%f,%f,%d",
				&id, &p.ox, &p.oy, &p.oz, &p.vx, &p.vy, &p.vz,
				&p.bx, &p.by, &p.bz, &p.onground, &p.ducked, &p.ducking,
				&p.ducktime, &p.stamina, &p.gravity, &p.sfric,
				&p.hullmin_z, &p.hullmax_z, &p.hull_xy,
				&p.yaw[0], &p.fmove[0], &p.smove[0], &p.buttons[0],
				&p.yaw[1], &p.fmove[1], &p.smove[1], &p.buttons[1],
				&p.yaw[2], &p.fmove[2], &p.smove[2], &p.buttons[2],
				&p.yaw[3], &p.fmove[3], &p.smove[3], &p.buttons[3],
				&p.yaw[4], &p.fmove[4], &p.smove[4], &p.buttons[4]);
			if (n == 40)
				probes.push_back(p);
		}
		fclose(f);
	}
	if (probes.empty())
		return -1;

	// QUEUE it: execution happens inside the FinishMove hook, chunked.
	s_fuzz_probes.swap(probes);
	s_fuzz_outs.assign(s_fuzz_probes.size() * kFuzzTicks, FuzzTickOut{});
	s_fuzz_out_path = out_path;
	s_fuzz_next = 0;
	s_fuzz_ok = 0;
	s_fuzz_pending = 1;
	return static_cast<int>(s_fuzz_probes.size());
}

int Prediction::RunFuncProbe(const char* pin_path, const char* probe_path,
                             const char* out_path) {
	if (!g_pred || !g_gm || !g_helper || !pin_path || !probe_path || !out_path)
		return -1;
	void* player = entitylist->GetClientEntity(engine->GetLocalPlayer());
	if (!player)
		return -1;
	ResolveOffsets();
	if (!g_off.origin || !g_off.velocity)
		return -1;
	// FAIL CLOSED: without a verified context latch every call would write
	// through guessed offsets. -1 (never observed) is as fatal as 0.
	if (g_gm_ctx_gate != 1)
		return -2;

	std::vector<FuncPin> pins;
	{
		FILE* f = nullptr;
		if (fopen_s(&f, pin_path, "r") != 0 || !f)
			return -1;
		char line[256];
		while (fgets(line, sizeof(line), f)) {
			if (line[0] == '#' || line[0] == '\n')
				continue;
			FuncPin p = {};
			unsigned rva = 0, prel = 0;
			int abi = 0;
			const int nf = sscanf_s(line, "%47[^,],%x,%d,%x", p.name,
				static_cast<unsigned>(sizeof(p.name)), &rva, &abi, &prel);
			if (nf >= 3) {
				p.rva = rva;
				p.abi = abi;
				p.prelude_rva = (nf >= 4) ? prel : 0u;
				pins.push_back(p);
			}
		}
		fclose(f);
	}
	if (pins.empty())
		return -1;

	std::vector<FuncProbeIn> probes;
	{
		FILE* f = nullptr;
		if (fopen_s(&f, probe_path, "r") != 0 || !f)
			return -1;
		char line[512];
		while (fgets(line, sizeof(line), f)) {
			if (line[0] == '#' || line[0] == 'f')
				continue;
			FuncProbeIn q = {};
			char fname[48] = "";
			if (sscanf_s(line,
				"%47[^,],%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d,%d,%d,"
				"%f,%f,%f,%f,%f,%f,%f,%f",
				fname, static_cast<unsigned>(sizeof(fname)),
				&q.ox, &q.oy, &q.oz, &q.vx, &q.vy, &q.vz,
				&q.bx, &q.by, &q.bz,
				&q.onground, &q.ducked, &q.ducking, &q.buttons,
				&q.ducktime, &q.stamina, &q.sfric, &q.gravity,
				&q.hullmax_z, &q.yaw, &q.fmove, &q.smove) == 22) {
				q.pin = -1;
				for (size_t k = 0; k < pins.size(); ++k)
					if (_stricmp(pins[k].name, fname) == 0) {
						q.pin = static_cast<int>(k);
						break;
					}
				if (q.pin >= 0)
					probes.push_back(q);
			}
		}
		fclose(f);
	}
	if (probes.empty())
		return -1;

	s_fp_pins.swap(pins);
	s_fp_probes.swap(probes);
	s_fp_outs.assign(s_fp_probes.size(), FuncProbeOut{});
	s_fp_out_path = out_path;
	s_fp_next = 0;
	s_fp_ok = 0;
	s_fp_pending = 1;
	return static_cast<int>(s_fp_probes.size());
}

bool Prediction::FuncProbeBusy() { return s_fp_pending != 0; }

int Prediction::FuncProbeProgress(int* total, int* ok) {
	if (total) *total = static_cast<int>(s_fp_probes.size());
	if (ok) *ok = s_fp_ok;
	return s_fp_next;
}

int Prediction::FuncProbeCtxGate() { return g_gm_ctx_gate; }

bool Prediction::FuzzBusy() { return s_fuzz_pending != 0; }

int Prediction::FuzzProgress(int* total, int* ok) {
	if (total) *total = static_cast<int>(s_fuzz_probes.size());
	if (ok) *ok = s_fuzz_ok;
	return s_fuzz_next;
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

bool Prediction::PlayerHull(Vector* mins, Vector* maxs) {
	if (mins) *mins = g_hull_min;
	if (maxs) *maxs = g_hull_max;
	return g_hull_measured;
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

	// The collision hull, straight from the engine's CCollisionProperty -
	// the box it sweeps. Only sampled while STANDING (it shrinks when ducked)
	// because the model simulates a standing player; the values feed the
	// collider expansion instead of hardcoded 16/16/72.
	if (!out.ducked && g_off.mins && g_off.maxs) {
		const Vector mn = NetVars::Get<Vector>(player, g_off.mins);
		const Vector mx = NetVars::Get<Vector>(player, g_off.maxs);
		if (mx.X > mn.X && mx.Y > mn.Y && mx.Z > mn.Z
			&& mx.Z - mn.Z > 8.f && mx.Z - mn.Z < 200.f) {
			g_hull_min = mn;
			g_hull_max = mx;
			g_hull_measured = true;
		}
	}

	// View angles come from the engine: the m_angEyeAngles netvars exist for
	// drawing REMOTE players and are unreliable for the local one (pitch reads
	// ~0), which sent pick rays out level at eye height. Netvars stay as the
	// fallback only if the engine call reads exactly zero.
	QAngle view(0.f, 0.f, 0.f);
	engine->GetViewAngles(view);
	out.pitch = view.X;
	out.yaw   = view.Y;
	if (view.X == 0.f && view.Y == 0.f && g_off.eye_angles) {
		out.pitch = NetVars::Get<float>(player, g_off.eye_angles);
		out.yaw   = NetVars::Get<float>(player, g_off.eye_angles + 4);
	}

	out.stamina = g_off.stamina ? NetVars::Get<float>(player, g_off.stamina) : 0.f;
	out.valid = true;
	return true;
}

bool Prediction::PredictedFlags(int* out) {
	if (!out || !g_pred_flags_valid || !engine || !engine->IsInGame())
		return false;
	*out = g_pred_flags;
	return true;
}

bool Prediction::LiveFlags(int* out) {
	if (!out || !engine || !entitylist || !engine->IsInGame())
		return false;
	void* player = entitylist->GetClientEntity(engine->GetLocalPlayer());
	if (!player)
		return false;
	ResolveOffsets();
	if (!g_off.flags)
		return false;
	*out = NetVars::Get<int>(player, g_off.flags);
	return true;
}

bool Prediction::LastRealMaxSpeed(float* out) {
	if (!g_real_move_valid)
		return false;
	if (out) *out = g_real_maxspeed;
	return true;
}

bool Prediction::LastRealMoveOrigin(Vector& out) {
	if (!g_real_move_valid)
		return false;
	out = g_real_move_origin;
	return true;
}

Prediction::Diag Prediction::LastDiag() { return g_diag; }
