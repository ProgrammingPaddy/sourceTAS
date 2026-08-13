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
		float jump_height = 57.f;        // CSS: vz = sqrt(2*gravity*57) ~ 302
		float non_jump_velocity = 140.f; // CategorizePosition vz gate
		float walkable_z = 0.7f;         // minimum ground plane normal.z
		float duck_speed_frac = 0.34f;   // CSS fully-ducked maxspeed fraction
		float time_to_duck_ms = 400.f;   // ground duck transition time
		// In-air duck/unduck origin shift. ENGINE-MEASURED 2026-08-13 from the
		// basictest ground truth: duck tick +8.500 exactly, unduck −8.500
		// exactly - NOT the SDK-theoretical hull delta (72-54=18). The duck
		// HULL height stays 54 for collision (only a ceiling map can test it).
		float duck_air_shift = 8.5f;
		// JUMP STAMINA (engine-fitted 2026-08-13, real-playback capture with
		// FOUR jumps): the JUMP arms the timer (1317.5 ms ~ classic 25/19 s),
		// it drains dt*1000 per tick everywhere, and the residue acts twice:
		//  - WALK ticks scale v.xy by (1 - stamina*stamina_scale_per_ms)
		//    after friction (fitted 0.00019833 from the t479-landing decay);
		//  - a HOT JUMP scales its impulse by
		//    (1 - stamina*stamina_jump_scale_per_ms), fitted 0.000186 from
		//    taxed jumps at gaps 49/44/43 ticks (predicts all three within
		//    0.05 u/s; zero-tax confirmed at gap 900). The two scales are
		//    fitted independently per regime - do not unify without data.
		// Landings arm NOTHING (jump-free -775 u/s landing: scale 1.00000).
		float stamina_jump_ms = 1317.5f;
		float stamina_scale_per_ms = 0.00019833f;
		float stamina_jump_scale_per_ms = 0.000186f;
		// SDK CheckJumpButton calls FinishGravity() inside itself - an extra
		// half-gravity on the jump tick on top of FullWalkMove's own pair.
		// Kept as a toggle so replay parity data can arbitrate the quirk.
		bool jump_finishgravity = true;
		// sv_enablebunnyhopping 1 (surf setup) disables PreventBunnyJumping's
		// 1.1*maxspeed pre-jump clamp (binary-scanned server behavior).
		bool enablebunnyhopping = true;
	};

	struct PlayerState {
		Vec3 pos;                  // feet origin
		Vec3 vel;
		bool ducked = false;       // FL_DUCKING: hull swapped
		bool ducking = false;      // ground transition in progress
		float duck_elapsed_ms = 0.f;
		bool on_ground = false;
		int  ground_brush = -1;    // index into World::brushes (-1 = none)
		int  old_buttons = 0;      // IN_JUMP release gate lives here
		float stamina = 0.f;       // landing-stamina timer, ms (see MoveParams;
		                           // the tape anchor's value seeds it)
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
	};

	void MoveTick(PlayerState& s, const World& w, const MoveParams& p,
	              float pitch, float yaw, float fmove, float smove, float umove,
	              int buttons, TickEvents* ev);

} // namespace Solver
