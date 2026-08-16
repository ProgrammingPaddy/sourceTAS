#pragma once

// One full engine movement tick for the solver core (CGameMovement::PlayerMove
// -> FullWalkMove order, MOVETYPE_WALK, no water). The airborne pipeline
// (AirAccelerate -> TryPlayerMove -> CategorizePosition, unclamped-enterfrac
// traces, crease resolution, ground snap) is a verbatim mirror of the DLL's
// month-validated model. Ground walking, jumping, and ducking are new here
// (SDK-shaped) and are arbitrated by tape-replay parity against real runs -
// where SDK recollection and measured behavior disagree, the measurement wins.

#include "SolverWorld.h"

namespace Solver {

	// Source SDK button bits (stable across the era).
	enum {
		IN_ATTACK = 1 << 0,
		IN_JUMP = 1 << 1,
		IN_DUCK = 1 << 2,
		IN_FORWARD = 1 << 3,
		IN_BACK = 1 << 4,
		IN_USE = 1 << 5,
		IN_MOVELEFT = 1 << 9,
		IN_MOVERIGHT = 1 << 10,
	};

	struct MoveParams {
		float dt = 0.015f;               // 66.67 tps tick interval
		float gravity = 800.f;           // sv_gravity
		float accelerate = 10.f;         // sv_accelerate (surf setup value)
		float airaccelerate = 150.f;     // sv_airaccelerate (surf setup value)
		float friction = 4.f;            // sv_friction (CONFIRMED from engine
		                                 // ground-truth fit, 2026-08-13)
		float stopspeed = 75.f;          // sv_stopspeed (CONFIRMED 75 from the
		                                 // fitted drop curve - NOT Source's 100)
		float maxspeed = 250.f;          // weapon run speed (knife 250)
		float maxvelocity = 3500.f;      // sv_maxvelocity, clamped per component
		float stepsize = 18.f;           // step height (StayOnGround reach)
		float air_speed_cap = 30.f;      // CSS GetAirSpeedCap (the "30 cap")
		// Jump impulse: BINARY-DECODED (server.dll x64 @2eddab, 2026-08-15).
		// The movement-optimizations build BAKES sqrt(2*800*57) as a DOUBLE
		// constant - it does NOT track sv_gravity - and applies it in double
		// with a single rounding back to float (see CheckJumpButton).
		double jump_impulse_d = 301.99337741082996;
		float non_jump_velocity = 140.f; // CategorizePosition vz gate
		// SURF AIR FRICTION (fuzz-measured 2026-08-15): CategorizePosition
		// sets m_surfaceFriction = 0.25 when the ground probe finds NO
		// walkable plane while the player is RISING (0 < vz <= 140), and
		// every accelerate scales by it. Engine samples pinned the constant
		// exactly - a ducked rising tick applied 47.8125 u/s of accel =
		// 150 * 85 * 0.015 * 0.25, with the addspeed cap far above. The
		// field is player+0x36C (confirmed: server.dll Friction @0020edac
		// reads it and multiplies by sv_friction).
		float air_friction_up = 0.25f;
		float walkable_z = 0.7f;         // minimum ground plane normal.z
		float duck_speed_frac = 0.34f;   // CSS fully-ducked maxspeed fraction
		float time_to_duck_ms = 400.f;   // ground duck transition time
		float time_to_unduck_ms = 200.f; // SDK TIME_TO_UNDUCK (ducking stays
		                                 // true through the unduck - fuzz-
		                                 // measured via the SET-path jump)
		// In-air duck/unduck origin shift. ENGINE-MEASURED 2026-08-13 from the
		// basictest ground truth: duck tick +8.500 exactly, unduck −8.500
		// exactly - NOT the SDK-theoretical hull delta (72-54=18). The duck
		// HULL height stays 54 for collision (only a ceiling map can test it).
		float duck_air_shift = 8.5f;
		// STAMINA - BINARY-DECODED from the shipped x64 server.dll
		// (2026-08-15, capstone disassembly; constants read out of
		// .text/.rdata, zero fitting):
		//  - CheckJumpButton @2eddfc: the WHOLE post-impulse vz scales by
		//    (1 - stamina*0.00019f) in float, then m_flStamina is stored as
		//    raw bits 0x44a47943 == float(25000.f/19.f) exactly (@2ede47).
		//  - WalkMove @2f0160: vel.xy *= powf(1 - stamina*0.00019f,
		//    frametime*70.0f), after Friction, before accelerate. The pow
		//    exponent scales with TICKRATE (1.05 at 66.67tps) - the engine
		//    adapts to interval changes and so does this mirror.
		//  - ONE shared scale constant (.rdata 0056faf4) serves both sites.
		//  - Drain stays 1000*frametime ms per tick everywhere.
		// History: the affine walk fit (slope 0.00019833, intercept
		// 0.000327) was a chord across the 1.05-power curve, and the
		// impulse-only jump scale 0.00018622 was 0.00019*(296/302); both
		// retired by the disassembly. Landings arm NOTHING on this server
		// (measured -775 u/s landing walked at scale 1.00000).
		float stamina_jump_ms = 25000.f / 19.f;   // == engine bits 0x44a47943
		float stamina_scale_per_ms = 0.00019f;    // shared walk+jump scale
		float stamina_pow_rate = 70.f;            // walk exponent per second
		// sv_enablebunnyhopping 1 (surf setup) disables PreventBunnyJumping's
		// 1.1*maxspeed pre-jump clamp (binary-scanned server behavior).
		bool enablebunnyhopping = true;
		// sv_autobunnyhopping != 0 BYPASSES CheckJumpButton's jump-release
		// gate entirely (client.dll @1f546c reads the cvar before testing
		// oldbuttons & IN_JUMP). Live server cvar, exported like the rest.
		bool autobunnyhopping = false;
		// POST-AIR-UNDUCK TRANSIENT HULL (see PlayerState::hull_state):
		// gates hull_state 2 on air unducks. Default ON (the measured
		// behavior); off = flag-coupled hulls for arbitration.
		bool unduck_hull_defer = true;
	};

	struct PlayerState {
		Vec3 pos;                  // feet origin
		Vec3 vel;
		bool ducked = false;       // FL_DUCKING: the duck FLAG (speed caps,
		                           // eye offset semantics)
		// The COLLISION hull state, decoupled from the flag (battery+oracle
		// fitted 2026-08-14; corrected vs the VDC dimensions after user
		// challenge): 0 = standing (72), 1 = ducked (54), 2 = the POST-AIR-
		// UNDUCK TRANSIENT - the unduck drops the origin 8.5 while the
		// world-space bounds stay, leaving the top at 54+8.5 = 62.5 until
		// grounding re-bases it (engine release bracketed to (56.7, 63.7);
		// the engine's own 72-hull trace HITS where its movement flew).
		unsigned char hull_state = 0;
		// SDK duck state machine (fuzz-corrected 2026-08-15): m_bDucking is
		// true during BOTH transitions - duck AND unduck - and the shared
		// timer COUNTS DOWN from 1000 (drained in ReduceTimers). A ground
		// duck tapped and released keeps ducking=true for TIME_TO_UNDUCK
		// (200ms), which flips the jump to the SET path: fuzz_a's t4 jump
		// measured 289.99 (SET) where the instant-cancel model gave 283.99.
		bool ducking = false;      // transition in progress (either way)
		float duck_timer_ms = 0.f; // SDK m_flDucktime, counts DOWN from 1000
		bool on_ground = false;
		int  ground_brush = -1;    // index into World::brushes (-1 = none)
		// m_surfaceFriction: 1 on ground / falling, 0.25 airborne-rising
		// (see MoveParams::air_friction_up). Scales EVERY accelerate.
		float surface_friction = 1.f;
		int  old_buttons = 0;      // IN_JUMP release gate lives here
		float stamina = 0.f;       // landing-stamina timer, ms (see MoveParams;
		                           // the tape anchor's value seeds it)
		// TRIGGER STATE (total-parity port 2026-08-15, mirrors the DLL's
		// RequestSim application + SDK movement consumption):
		float gravity_scale = 1.f; // trigger_gravity player scale (persists)
		Vec3 basevel;              // trigger_push base velocity
		bool basevel_flag = false; // FL_BASEVELOCITY: decays unless refreshed
		// ENGINE-SHAPE FIELDS (client.dll CheckJumpButton/FinishGravity read
		// them; the current world model can never set them, but the mirrored
		// functions branch on them exactly as the engine does):
		float water_jump_time = 0.f; // m_flWaterJumpTime (+0x1684)
		int   water_level = 0;       // 0 = dry; >= 2 (WL_Waist) refuses jumps
		bool  duck_until_ground = false; // +0x1a5c, third SET-path condition
	};

	// Per-tick observations for tracing and diagnostics.
	struct TickEvents {
		int   contact_brush[4];    // World::brushes indices struck this tick
		int   contact_plane[4];
		float contact_loss[4];     // speed the clip resolution removed (u/s)
		int   ncontacts = 0;
		bool  jumped = false;
		bool  landed = false;      // air -> ground this tick
		bool  left_ground = false; // ground -> air this tick
		bool  duck_changed = false;
		bool  teleported = false;  // trigger_teleport fired this tick
	};

	void MoveTick(PlayerState& s, const World& w, const MoveParams& p,
	              float pitch, float yaw, float fmove, float smove, float umove,
	              int buttons, TickEvents* ev);

	// ---- FUNCPROBE: individually callable mirrors -------------------------
	// The whole-tick comparison can only say the COMPOSITION diverged. These
	// expose single functions so each can be diffed against the engine's own
	// body in isolation (see Docs/FuncProbe.md and solver/func_pins.cfg).
	namespace Fn {
		void CategorizePosition(PlayerState& s, const World& w,
		                        const MoveParams& p);
		// Returns true when the jump fired (engine returns bool).
		bool CheckJumpButton(PlayerState& s, const World& w,
		                     const MoveParams& p);
		// Duck family (pinned via the CCSGameMovement vtable 2026-08-15).
		void Duck(PlayerState& s, const World& w, const MoveParams& p,
		          int buttons);
		bool CanUnduck(const PlayerState& s, const World& w,
		               const MoveParams& p);
	}

} // namespace Solver
