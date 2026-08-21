#include "SolverMove.h"

namespace Solver {

	namespace {

		// CGameMovement::ClipVelocity with overbounce 1: out = in - n*dot(in,n),
		// then the engine's one adjust iteration so the result never still
		// points into the plane. (Verbatim from the validated DLL model.)
		void EngineClipVelocity(const Vec3& in, const Vec3& n, Vec3& out) {
			const float backoff = Dot(in, n);
			out = in - Scale(n, backoff);
			const float adjust = Dot(out, n);
			if (adjust < 0.f)
				out = out - Scale(n, adjust);
		}

		// CGameMovement::CheckVelocity - client.dll @0x1182e0, re-coded 1-to-1.
		// Per component: a velocity whose exponent bits are all-ones (NaN/Inf,
		// mask 0x7f800000 @118333) is warned about and ZEROED, the origin gets
		// the same check, then the +/- sv_maxvelocity clamp (function tail).
		void CheckVelocity(PlayerState& s, const MoveParams& p) {
			float* v[3] = { &s.vel.X, &s.vel.Y, &s.vel.Z };
			for (int i = 0; i < 3; ++i) {
				unsigned bits;
				memcpy(&bits, v[i], 4);
				if ((bits & 0x7f800000u) == 0x7f800000u)   // @118333 IS_NAN
					*v[i] = 0.f;                            // @11838a
				// (origin NaN check @118395 - unreachable in this model's
				// arithmetic; the velocity clamp below is the live tail)
				if (*v[i] > p.maxvelocity)
					*v[i] = p.maxvelocity;
				else if (*v[i] < -p.maxvelocity)
					*v[i] = -p.maxvelocity;
			}
		}

		// Move directions from the view yaw with z zeroed and normalized -
		// AirMove and WalkMove both flatten the basis, which is why pitch never
		// enters surf movement. forward = (cos yaw, sin yaw), right = (sin yaw,
		// -cos yaw): facing +x puts right at -y (Source angle convention).
		void WishFromInput(float yaw, float fmove, float smove,
		                   float* wx, float* wy) {
			const float r = Deg2Rad(yaw);
			const float cy = cosf(r), sy = sinf(r);
			*wx = cy * fmove + sy * smove;
			*wy = sy * fmove - cy * smove;
		}

		// TryPlayerMove: up to 4 bumps; partial moves re-base the clip set;
		// first impact = plain clip; multi-plane crease resolution; stop-dead
		// guards. Verbatim mirror of the validated DLL model (which itself
		// mirrors the engine's measured behavior - do NOT "fix" it toward SDK
		// recollection without parity evidence).
		void TryPlayerMove(PlayerState& s, const World& w, const MoveParams& p,
		                   TickEvents* ev) {
			float time_left = p.dt;
			Vec3 planes[5];
			int numplanes = 0;
			Vec3 original_v = s.vel;
			const Vec3 primal_v = s.vel;
			float all_fraction = 0.f;   // SDK: sum of every bump's fraction
			for (int bump = 0; bump < 4; ++bump) {
				if (Len2(s.vel) == 0.f)
					break;
				const Vec3 end = s.pos + Scale(s.vel, time_left);
				TraceResult tr;
				const float frac = w.TraceHull3(s.pos, end, s.hull_state, &tr);
				// SDK TryPlayerMove: "entity is trapped in another solid" -
				// velocity is ZEROED and the move aborts. This is the surf
				// RAMP BUG, and it is deterministic: the engine then repeats
				// it every tick until something frees the hull (measured 18
				// straight ticks at v(0,0,-6) in the d34 tape, and in 32.7%
				// of fuzz probes).
				if (tr.allsolid) {
					s.vel = Vec3();
					if (ev && ev->ncontacts < 4) {
						ev->contact_brush[ev->ncontacts] = tr.brush;
						ev->contact_plane[ev->ncontacts] = tr.plane;
						ev->contact_loss[ev->ncontacts] = Len(primal_v);
						ev->contact_pos[ev->ncontacts] = s.pos;
						ev->contact_vel[ev->ncontacts] = primal_v;
						ev->ncontacts++;
					}
					break;
				}
				all_fraction += frac;
				if (frac > 0.f) {
					// SDK "Player will become stuck!!!" guard: on a FULL move
					// the engine re-tests the destination with an UNSWEPT box
					// and, if that lands in solid, ZEROES VELOCITY and stops.
					// A swept trace can pass while its endpoint is embedded -
					// which is precisely how a clean-looking flight ends in a
					// frozen player (the d34 tape's t193).
					if (frac >= 1.f) {
						const Vec3 dest = s.pos + Scale(end - s.pos, frac);
						TraceResult st;
						const float sf = w.TraceHull3(dest, dest,
							s.hull_state, &st);
						if (st.startsolid || sf != 1.f) {
							s.pos = dest;
							s.vel = Vec3();
							break;
						}
					}
					s.pos = s.pos + Scale(end - s.pos, frac);
					original_v = s.vel;   // engine re-bases the clip set
					numplanes = 0;
				}
				if (frac >= 1.f || tr.brush < 0)
					break;
				time_left -= time_left * frac;
				const Vec3 n = tr.normal;
				if (ev && ev->ncontacts < 4) {
					ev->contact_brush[ev->ncontacts] = tr.brush;
					ev->contact_plane[ev->ncontacts] = tr.plane;
					ev->contact_loss[ev->ncontacts] = 0.f;
					// s.pos = the impact point (the partial move already
					// ran); s.vel = the velocity entering this bump's clip.
					ev->contact_pos[ev->ncontacts] = s.pos;
					ev->contact_vel[ev->ncontacts] = s.vel;
					ev->ncontacts++;
				}
				if (numplanes >= 5) {
					s.vel = Vec3();
					break;
				}
				planes[numplanes++] = n;
				const float sp_before = Len(s.vel);
				// SDK: the single-plane REFLECT path is gated on being
				// AIRBORNE (MOVETYPE_WALK && GetGroundEntity() == NULL). A
				// GROUNDED first impact takes the general path below - which
				// carries the stop-dead guard our version was skipping.
				if (numplanes == 1 && !s.on_ground) {
					EngineClipVelocity(original_v, planes[0], s.vel);
					original_v = s.vel;
				} else {
					int i = 0;
					for (; i < numplanes; ++i) {
						EngineClipVelocity(original_v, planes[i], s.vel);
						int j = 0;
						for (; j < numplanes; ++j)
							if (j != i && Dot(s.vel, planes[j]) < 0.f)
								break;
						if (j == numplanes)
							break;
					}
					if (i == numplanes) {
						if (numplanes != 2) {
							s.vel = Vec3();
							break;
						}
						// Slide along the crease of the two planes.
						Vec3 dir(planes[0].Y * planes[1].Z - planes[0].Z * planes[1].Y,
						         planes[0].Z * planes[1].X - planes[0].X * planes[1].Z,
						         planes[0].X * planes[1].Y - planes[0].Y * planes[1].X);
						const float dl = Len(dir);
						if (dl > 1e-6f) dir = Scale(dir, 1.f / dl);
						s.vel = Scale(dir, Dot(dir, s.vel));
					}
					if (Dot(s.vel, primal_v) <= 0.f) {
						s.vel = Vec3();
						break;
					}
				}
				if (ev && ev->ncontacts > 0)
					ev->contact_loss[ev->ncontacts - 1] = sp_before - Len(s.vel);
			}
			// SDK tail: if nothing moved at all across every bump, the engine
			// zeroes velocity outright.
			if (all_fraction == 0.f)
				s.vel = Vec3();
		}

		// CategorizePosition: the engine's ONLY grounding rule. Falling no
		// faster up than NON_JUMP_VELOCITY, a 2-unit down-trace onto a walkable
		// plane (nz >= 0.7) sets ground AND SNAPS the origin to the trace end.
		void CategorizePosition(PlayerState& s, const World& w, const MoveParams& p,
		                        TickEvents* ev) {
			const bool was_ground = s.on_ground;
			s.on_ground = false;
			s.ground_brush = -1;
			// SDK: every CategorizePosition resets m_surfaceFriction to 1.
			s.surface_friction = 1.f;
			if (s.vel.Z <= p.non_jump_velocity) {
				TraceResult tr;
				const float gf = w.TraceHull3(s.pos, s.pos - Vec3(0.f, 0.f, 2.f),
				                             s.hull_state, &tr);
				bool walk = gf < 1.f && tr.brush >= 0
					&& tr.normal.Z >= p.walkable_z;
				if (!walk) {
					// TracePlayerBBoxForGround (client.dll @0x117696): when
					// the full-box probe finds no walkable plane, the engine
					// re-probes with FOUR QUADRANT boxes - each half the hull
					// in x and y - and grounds if any of them lands on a
					// walkable plane, keeping the original fraction. This is
					// how a hull overlapping a wall or hanging off an edge
					// still stands. Measured: 13 of 40,072 isolated calls
					// where the engine grounded and we did not, every one of
					// them beside a wall or on a rim.
					const Vec3 hmn = s.hull_state == 1 ? w.HullDims().duck_min
						: s.hull_state == 2 ? w.HullDims().unduck_min
						: w.HullDims().stand_min;
					const Vec3 hmx = s.hull_state == 1 ? w.HullDims().duck_max
						: s.hull_state == 2 ? w.HullDims().unduck_max
						: w.HullDims().stand_max;
					const Vec3 quads[4][2] = {
						{ hmn, Vec3(fminf(0.f, hmx.X), fminf(0.f, hmx.Y), hmx.Z) },
						{ Vec3(fmaxf(0.f, hmn.X), fmaxf(0.f, hmn.Y), hmn.Z), hmx },
						{ Vec3(hmn.X, fmaxf(0.f, hmn.Y), hmn.Z),
						  Vec3(fminf(0.f, hmx.X), hmx.Y, hmx.Z) },
						{ Vec3(fmaxf(0.f, hmn.X), hmn.Y, hmn.Z),
						  Vec3(hmx.X, fminf(0.f, hmx.Y), hmx.Z) },
					};
					for (int q = 0; q < 4 && !walk; ++q) {
						TraceResult qt;
						w.TraceHullBox(s.pos, s.pos - Vec3(0.f, 0.f, 2.f),
							quads[q][0], quads[q][1], &qt);
						if (qt.brush >= 0 && qt.normal.Z >= p.walkable_z) {
							walk = true;
							tr = qt;
						}
					}
				}
				if (!walk && s.vel.Z > 0.f) {
					// No walkable plane under a RISING player: the engine's
					// surf air-accel quirk (see MoveParams).
					s.surface_friction = p.air_friction_up;
				}
				if (walk) {
					// NO origin snap: engine ground truth (2026-08-13, t479)
					// sets ground with the origin still 1.575u above the
					// surface; StayOnGround reaches it on the next WALK tick.
					// (The DLL model's in-probe snap conflated end-of-flight
					// analysis with continuing simulation - do not restore it.)
					s.on_ground = true;
					s.ground_brush = tr.brush;
					// SetGroundEntity zeroes vz whenever NEW ground is set
					// (mv->m_vecVelocity.z = 0). Invisible on ordinary ticks
					// (the tick-end grounded zero masks it) but LOAD-BEARING
					// when ground is set mid-tick and left the same tick:
					// spine495 t537 (capture 2026-08-14) - the air unduck's
					// -8.5 origin drop grounds on the ramp via the mid-tick
					// categorize, this zero eats vz +100.177, and the queued
					// jump runs the exact cold chain -6+301.9934-6-6=283.9934.
					s.vel.Z = 0.f;
					// Grounding realigns the collision hull to the duck
					// FLAG (the air-unduck's deferred hull ends here).
					s.hull_state = s.ducked ? 1 : 0;
				}
			}
			if (ev) {
				if (!was_ground && s.on_ground) ev->landed = true;
				if (was_ground && !s.on_ground) ev->left_ground = true;
			}
			// Landings arm NOTHING (measured: the jump-free -775 u/s floor
			// landing walked at scale 1.00000) - stamina is the JUMP's tax.
		}

		void Friction(PlayerState& s, const MoveParams& p) {
			const float speed = Len(s.vel);
			if (speed < 0.1f)
				return;
			const float control = (speed < p.stopspeed) ? p.stopspeed : speed;
			// SDK: friction = sv_friction * m_surfaceFriction (server.dll
			// @0020edac reads player+0x36C and multiplies). 1.0 on ground.
			const float drop = control * p.friction * s.surface_friction * p.dt;
			float newspeed = speed - drop;
			if (newspeed < 0.f) newspeed = 0.f;
			if (newspeed != speed)
				s.vel = Scale(s.vel, newspeed / speed);
		}

		// CGameMovement::FinishGravity - client.dll @0x11a0c0, re-coded 1-to-1.
		void FinishGravity(PlayerState& s, const MoveParams& p) {
			// @11a0d0  if (m_flWaterJumpTime) return;
			if (s.water_jump_time != 0.f)
				return;
			// @11a0e3  ent_gravity = player->m_flGravity, 0 -> 1.0 (@11a0f0)
			float ent_gravity = s.gravity_scale;
			if (ent_gravity == 0.f)
				ent_gravity = 1.f;
			// @11a10b..11a129  vz -= (gravity * ent_gravity) * (frametime * 0.5)
			// The PAIRING is the engine's: float multiply is not associative,
			// so (g*e)*(dt*0.5) is mirrored literally, not rewritten.
			s.vel.Z -= (p.gravity * ent_gravity) * (p.dt * 0.5f);
			// @11a12e  CheckVelocity()
			CheckVelocity(s, p);
		}

		// CCSGameMovement::CheckJumpButton - client.dll @0x1f5330, re-coded
		// 1-to-1 from the x64 disassembly (2026-08-15): same branches, same
		// ORDER, same arithmetic and rounding. Each block cites the
		// instruction it mirrors. Returns the engine's bool (true = jumped).
		bool CheckJumpButton(PlayerState& s, const World& /*w*/, const MoveParams& p,
		                     TickEvents* ev) {
			// @1f5340  if (player->pl.deadflag) { oldbuttons |= IN_JUMP;
			//          return false; }  - no death in this world model.
			// @1f5359  water-jump timer: drain and refuse. (The binary
			//          subtracts RAW gpGlobals->frametime @1f537d - NOT the
			//          SDK's 1000*frametime.)
			if (s.water_jump_time != 0.f) {
				s.water_jump_time -= p.dt;
				if (s.water_jump_time < 0.f)
					s.water_jump_time = 0.f;
				return false;
			}
			// @1f53bd  waterlevel >= WL_Waist(2): swim-shove and refuse.
			//          Unreachable until the world model carries water.
			if (s.water_level >= 2) {
				s.on_ground = false;              // @1f53ce SetGroundEntity(0)
				s.ground_brush = -1;
				s.vel.Z = 100.f;                  // @1f53e9 CONTENTS_WATER
				                                  // (@1f5407: SLIME -> 80)
				return false;
			}
			// @1f5445  the ground gate is the ENTITY - GetGroundEntity() ==
			//          NULL - not FL_ONGROUND (FUNCPROBE control: the flag
			//          was set on 0 of 72 known-grounded isolated calls).
			if (!s.on_ground) {
				s.old_buttons |= IN_JUMP;         // @1f5453
				return false;
			}
			// @1f546c  sv_autobunnyhopping != 0 BYPASSES the release gate.
			if (!p.autobunnyhopping) {
				if (s.old_buttons & IN_JUMP)      // @1f5480
					return false;
			}
			// @1f5486  if (!sv_enablebunnyhopping) PreventBunnyJumping()
			//          (vtable+0x1b8): clamp speed to 1.1*maxspeed.
			if (!p.enablebunnyhopping) {
				const float maxscaled = p.maxspeed * 1.1f;
				const float spd = Len(s.vel);
				if (spd > maxscaled && spd > 0.f)
					s.vel = Scale(s.vel, maxscaled / spd);
			}
			// @1f54c8  SetGroundEntity(NULL) - leaving ground zeroes nothing.
			s.on_ground = false;
			s.ground_brush = -1;
			// @1f54fd  PlayStepSound / @1f5511 jump animation: no physics.
			// @1f5521  flGroundFactor = surfacedata ? jumpFactor(+0x44) : 1.0.
			//          No surfaceprops in the world model; every surface in
			//          the corpus is default (factor 1.0).
			const float ground_factor = 1.f;
			// @1f557e  startz = mv->m_vecVelocity[2]
			const float startz = s.vel.Z;
			// @1f5577/1f5586/1f558f  SET if (m_duckUntilOnGround || m_bDucking
			//          || (m_fFlags & FL_DUCKING)), else ADD. The multiply is
			//          DOUBLE against the baked constant (@1f55a0), the add is
			//          double (@1f55a8), ONE rounding at the store (@1f55ba).
			if (s.duck_until_ground || s.ducking || s.ducked)
				s.vel.Z = static_cast<float>(
					static_cast<double>(ground_factor) * p.jump_impulse_d);
			else
				s.vel.Z = static_cast<float>(
					static_cast<double>(ground_factor) * p.jump_impulse_d
					+ static_cast<double>(startz));
			// @1f55dd  stamina tax: the whole post-impulse vz scales by
			//          (1 - stamina * 0.00019f), in float.
			if (s.stamina > 0.f) {
				const float ratio = 1.f - s.stamina * p.stamina_scale_per_ms;
				s.vel.Z *= ratio;
			}
			// @1f5629  m_flStamina = raw bits 0x44a47943 (== 25000.f/19.f),
			//          armed BEFORE FinishGravity - engine order.
			s.stamina = p.stamina_jump_ms;
			// @1f5633  FinishGravity() - unconditional, inside the jump.
			FinishGravity(s, p);
			// @1f5646  m_outJumpVel.z += vel.z - startz; @1f565d
			//          m_outStepHeight += 0.1f; @1f5675 OnJump(): MoveData
			//          outputs the solver does not consume.
			// @1f5694  oldbuttons |= IN_JUMP; return true.
			s.old_buttons |= IN_JUMP;
			if (ev) {
				ev->jumped = true;
				ev->left_ground = true;
			}
			return true;
		}

		// CGameMovement::StepMove, 1:1 (user directive 2026-08-14: perfect
		// engine representation - the deferred "slides only" walk path owned
		// the tick-537 spine divergence that broke the 495 tape in-game).
		// Semantics, verbatim from the SDK:
		//   1. DOWN attempt: plain TryPlayerMove from the current state.
		//   2. Reset; trace UP by stepsize+DIST_EPSILON (partial ok);
		//      TryPlayerMove; trace DOWN by stepsize+DIST_EPSILON.
		//   3. If the down trace's plane is NOT walkable (a fraction-1 trace
		//      leaves the normal zeroed, which also fails the check - the
		//      engine's own behavior), the DOWN attempt wins outright.
		//   4. Otherwise apply the down trace endpoint and keep whichever
		//      attempt traveled farther in XY; when the STEP wins, the
		//      slide's vertical velocity carries over.
		void StepMove(PlayerState& s, const World& w, const MoveParams& p,
		              TickEvents* ev) {
			const float kDistEpsilon = 0.03125f;   // engine DIST_EPSILON
			const Vec3 pos0 = s.pos;
			const Vec3 vel0 = s.vel;

			// (1) Slide move down.
			TryPlayerMove(s, w, p, ev);
			const Vec3 down_pos = s.pos;
			const Vec3 down_vel = s.vel;

			// (2) Reset, step up (partial application, SDK-style).
			s.pos = pos0;
			s.vel = vel0;
			{
				const Vec3 up_to(pos0.X, pos0.Y,
					pos0.Z + p.stepsize + kDistEpsilon);
				TraceResult tr;
				const float f = w.TraceHull3(s.pos, up_to, s.hull_state, &tr);
				s.pos = s.pos + Scale(up_to - s.pos, f);
			}
			// Slide move up (contacts already recorded by the down attempt;
			// physics identical, bookkeeping single-counted).
			TryPlayerMove(s, w, p, nullptr);
			// Step down.
			const Vec3 down_to(s.pos.X, s.pos.Y,
				s.pos.Z - (p.stepsize + kDistEpsilon));
			TraceResult dn;
			const float fd = w.TraceHull3(s.pos, down_to, s.hull_state, &dn);

			// (3) Landed on a non-walkable plane (or nothing): slide wins.
			if (dn.normal.Z < p.walkable_z) {
				s.pos = down_pos;
				s.vel = down_vel;
				return;
			}
			// (4) Apply the down endpoint; farther XY attempt wins.
			s.pos = s.pos + Scale(down_to - s.pos, fd);
			const float ddown =
				(down_pos.X - pos0.X) * (down_pos.X - pos0.X)
				+ (down_pos.Y - pos0.Y) * (down_pos.Y - pos0.Y);
			const float dup =
				(s.pos.X - pos0.X) * (s.pos.X - pos0.X)
				+ (s.pos.Y - pos0.Y) * (s.pos.Y - pos0.Y);
			if (ddown > dup) {
				s.pos = down_pos;
				s.vel = down_vel;
			} else {
				s.vel.Z = down_vel.Z;   // copy z from the slide move
			}
		}

		void StayOnGround(PlayerState& s, const World& w, const MoveParams& p) {
			Vec3 start = s.pos;
			start.Z += 2.f;
			Vec3 end = s.pos;
			end.Z -= p.stepsize;
			TraceResult up;
			const float fu = w.TraceHull3(s.pos, start, s.hull_state, &up);
			start = s.pos;
			start.Z += 2.f * fu;
			TraceResult dn;
			const float fd = w.TraceHull3(start, end, s.hull_state, &dn);
			if (fd > 0.f && fd < 1.f && dn.brush >= 0
				&& dn.normal.Z >= p.walkable_z) {
				const Vec3 land = start + Scale(end - start, fd);
				if (fabsf(s.pos.Z - land.Z) > 0.015625f)   // half COORD_RESOLUTION
					s.pos = land;
			}
		}

		void WalkMove(PlayerState& s, const World& w, const MoveParams& p,
		              float yaw, float fmove, float smove, bool cap_ducked,
		              TickEvents* ev) {
			// Landing stamina drags WALK ticks only (a bhop tick jumps before
			// reaching here - measured: t432 kept full speed, t480+ decayed).
			// BINARY-MIRRORED (server.dll x64 @2f0160, CCSGameMovement::
			// WalkMove prologue - after Friction, before the base
			// accelerate+move):
			//   ratio = powf(1 - m_flStamina*0.00019f, frametime*70.0f)
			// The frametime exponent (0.015*70 = 1.05 at 66.67tps) is why
			// every affine fit insisted on slope ~0.000198 > 0.00019 plus an
			// intercept: the fitted line was a chord across this power curve.
			if (s.stamina > 0.f) {
				const float base = 1.f - s.stamina * p.stamina_scale_per_ms;
				const float ratio = powf(base, p.dt * p.stamina_pow_rate);
				s.vel.X *= ratio;
				s.vel.Y *= ratio;
			}
			float wx, wy;
			WishFromInput(yaw, fmove, smove, &wx, &wy);
			float wishspeed = sqrtf(wx * wx + wy * wy);
			Vec3 wishdir(0.f, 0.f, 0.f);
			if (wishspeed > 1e-6f)
				wishdir = Vec3(wx / wishspeed, wy / wishspeed, 0.f);
			// ENGINE-MEASURED 2026-08-13 (unseeded-tape ground truth, tick
			// 204): the ducked speed cap applies the moment duck is PRESSED
			// (m_bDucking), not when the transition finishes. REFINED
			// 2026-08-14 (unduck_face battery, t122 fitted to 0.1 u/s): the
			// UNDUCK tick still uses the ducked cap too - the tick's cap is
			// ducked if the duck state was ducked/ducking at ANY point in
			// this tick's duck processing (cap_ducked from MoveTick).
			const float effmax = cap_ducked
				? p.maxspeed * p.duck_speed_frac : p.maxspeed;
			if (wishspeed > effmax)
				wishspeed = effmax;
			// Accelerate.
			const float cur = Dot(s.vel, wishdir);
			const float add = wishspeed - cur;
			if (add > 0.f) {
				float accelspeed = p.accelerate * p.dt * wishspeed
					* s.surface_friction;
				if (accelspeed > add) accelspeed = add;
				s.vel = s.vel + Scale(wishdir, accelspeed);
			}
			s.vel.Z = 0.f;
			if (Len(s.vel) < 1.f) {
				s.vel = Vec3();
				return;
			}
			// Straight attempt first; on a hit, the engine's FULL StepMove
			// (slide vs step-up/slide/step-down, farther XY wins) - the
			// former slide-only path was the last known movement gap.
			// SDK WalkMove: base velocity rides through the move only.
			s.vel = s.vel + s.basevel;
			const Vec3 dest(s.pos.X + s.vel.X * p.dt,
			                s.pos.Y + s.vel.Y * p.dt, s.pos.Z);
			TraceResult tr;
			const float frac = w.TraceHull3(s.pos, dest, s.hull_state, &tr);
			if (frac >= 1.f) {
				s.pos = dest;
			} else {
				StepMove(s, w, p, ev);
			}
			s.vel = s.vel - s.basevel;
			StayOnGround(s, w, p);
			s.vel.Z = 0.f;
		}

		void AirMove(PlayerState& s, const World& w, const MoveParams& p,
		             float yaw, float fmove, float smove, bool cap_ducked,
		             TickEvents* ev) {
			float wx, wy;
			WishFromInput(yaw, fmove, smove, &wx, &wy);
			float wishspeed = sqrtf(wx * wx + wy * wy);
			Vec3 wishdir(0.f, 0.f, 0.f);
			if (wishspeed > 1e-6f)
				wishdir = Vec3(wx / wishspeed, wy / wishspeed, 0.f);
			const float effmax = cap_ducked
				? p.maxspeed * p.duck_speed_frac : p.maxspeed;
			if (wishspeed > effmax)
				wishspeed = effmax;
			// AirAccelerate: the add is capped at 30 (air speed cap) but the
			// accel budget scales with the FULL wishspeed - the asymmetry that
			// makes airaccelerate 150 behave the way surf servers expect.
			const float wishspd = (wishspeed > p.air_speed_cap)
				? p.air_speed_cap : wishspeed;
			const float cur = Dot(s.vel, wishdir);
			const float add = wishspd - cur;
			if (add > 0.f) {
				// accelspeed scales with m_surfaceFriction - 0.25 while
				// airborne and RISING (fuzz-measured; see MoveParams).
				float accelspeed = p.airaccelerate * wishspeed * p.dt
					* s.surface_friction;
				if (accelspeed > add) accelspeed = add;
				s.vel = s.vel + Scale(wishdir, accelspeed);
			}
			// SDK AirMove: base velocity is added AFTER AirAccelerate, rides
			// through TryPlayerMove, and is pulled back out. Adding it before
			// the accel (our old MoveTick did) poisons currentspeed and the
			// whole air-accel budget - fuzz probe 5, dvel 55.6 with the
			// engine and model at identical entry state.
			s.vel = s.vel + s.basevel;
			TryPlayerMove(s, w, p, ev);
			s.vel = s.vel - s.basevel;
		}

		// Duck state machine (CGameMovement::Duck, CS hulls). In the air a duck
		// finishes INSTANTLY with the origin pulled up by (standing - ducked)
		// hull height (the measured ~18u FinishDuck lift); on the ground the
		// hull swaps after TIME_TO_DUCK. Unduck reverses the shift and must fit.
		// ---- THE DUCK FAMILY, in the ENGINE'S OWN DECOMPOSITION ----------
		// One mirror function per pinned engine function (func_pins.cfg:
		// Duck 0x1f5cb0, FinishDuck 0x1f6500, FinishUnDuck 0x1f6840,
		// CanUnduck 0x1f4b20, HandleDuckingSpeedCrop 0x1f6ba0, plus
		// FixPlayerCrouchStuck as FinishDuck's helper). Behavior is the
		// isolated-grade-validated law set, unchanged - this is structure,
		// not semantics.

		// CGameMovement::FixPlayerCrouchStuck (upward): the stuck test is
		// the engine's TestPlayerPosition - a ZERO-LENGTH DUCKED-hull trace
		// with certified startsolid semantics, not the legacy point test
		// (which stopped one 1u ladder step short: ours -1056 vs engine
		// -1055 across a whole residual family). Probe up for a free spot;
		// RESTORE the origin when none is found (the keep-partial-nudge
		// variant ended 18u high).
		void FixPlayerCrouchStuck(PlayerState& s, const World& w) {
			// The stuck test is BRUSHES-ONLY - measured twice: routing it
			// through the leaf-aware raw ray took FinishDuck 2,549 -> 3,243
			// and the corpus down 432. Whatever trace TestPlayerPosition
			// wraps, it does not see leaf contents at these sites.
			auto stuck = [&](const Vec3& at) {
				TraceResult st;
				w.TraceHull3(at, at, 1, &st);
				return st.startsolid;
			};
			if (!stuck(s.pos))
				return;
			const Vec3 save = s.pos;
			for (int i = 0; i < 36; ++i) {
				s.pos.Z += 1.f;
				if (!stuck(s.pos))
					return;
			}
			s.pos = save;
		}

		// CCSGameMovement::FinishDuck (0x1f6500): completion is idempotent
		// on the flag; the +8.5 air re-base applies to a FIRST-TIME duck
		// only (already-ducked and grounded completions hold dz 0 across
		// every isolated class), and FixPlayerCrouchStuck runs
		// UNCONDITIONALLY (the 1u ladder shows on already-ducked and
		// grounded rows too, dz +17..+35).
		void FinishDuck(PlayerState& s, const World& w, const MoveParams& p,
		                TickEvents* ev) {
			const bool already = s.ducked;
			s.ducked = true;
			s.hull_state = 1;          // duck: flag and hull together
			s.ducking = false;
			if (ev && !already) ev->duck_changed = true;
			if (!s.on_ground && !already)
				s.pos.Z += p.duck_air_shift;
			FixPlayerCrouchStuck(s, w);
		}

		// CCSGameMovement::CanUnduck (0x1f4b20), the law from 5,981 isolated
		// calls margin-cross-tabbed (2026-08-15):
		//  - GROUNDED: pos == cand, an UNSWEPT standing test - blocked iff
		//    startsolid, which now includes the leaf-contents rule (the 212
		//    blocked-while-brush-clear rows on the ceiling slab).
		//  - AIR: a SWEPT standing trace pos -> pos-8.5. The brush the box
		//    STARTS inside is invisible for the whole sweep (64+21 rows
		//    free with both endpoints overlapping the same brush), so
		//    blocked iff the sweep ENTERS a started-outside brush
		//    (fraction < 1) - NOT iff the destination overlaps. The earlier
		//    "destination box" and "frac>=1 && !startsolid" variants each
		//    modeled half of this and each broke the other half.
		// DECODED VERBATIM from 0x1f4b20's tail (2026-08-15): ONE raw
		// TraceRay from mv->origin to newOrigin with the STANDING hull
		// (grounded: newOrigin == origin -> UNSWEPT; air: -8.5), verdict
		// @1f4f94: return !trace.startsolid && trace.fraction == 1.0. The
		// RAW-RAY unswept path consults LEAF CONTENTS (the 212 grounded
		// blocked-while-brush-clear rows on the ceiling slab), so the leaf
		// rule applies HERE for the unswept case - TestPlayerPosition
		// consumers (crouch-stuck ladder, stuck-guard) measurably do not
		// inherit it. This is also the surf ramp-ride rule: mid-ride the
		// -8.5 sweep clips the ramp (fraction < 1) and the unduck refuses
		// until the feet clear the surface.
		bool CanUnduck(const PlayerState& s, const World& w,
		               const MoveParams& p) {
			const Vec3 cand = s.on_ground
				? s.pos
				: Vec3(s.pos.X, s.pos.Y, s.pos.Z - p.duck_air_shift);
			// The decoded body calls raw TraceRay - our raw-ray model is
			// TraceHullBox (38,286/38,286 against the oracle at these very
			// sites), so the mirror calls it too. No call-site special
			// cases: the unswept leaf law lives inside the raw ray, where
			// the engine keeps it.
			TraceResult ut;
			const float uf = w.TraceHullBox(s.pos, cand,
				w.HullDims().stand_min, w.HullDims().stand_max, &ut);
			return !ut.startsolid && uf >= 1.f;
		}

		// CCSGameMovement::FinishUnDuck (0x1f6840): derives the same
		// candidate origin CanUnduck tested. Binary-decoded: the AIR shift
		// is UNCONDITIONAL - no was-ducked gate - so a never-completed duck
		// released in the air still re-bases -8.5 (fuzz_b t4, measured).
		void FinishUnDuck(PlayerState& s, const World& /*w*/,
		                  const MoveParams& p, TickEvents* ev) {
			const bool was_ducked = s.ducked;
			s.ducked = false;
			s.ducking = false;
			s.duck_timer_ms = 0.f;
			if (ev) ev->duck_changed = true;
			if (!s.on_ground) {
				s.pos.Z -= p.duck_air_shift;
				// Hull: a REAL unduck leaves the world-space ducked bounds
				// in place until grounding (the measured 62.5 transient); a
				// never-ducked "unduck" had standing bounds all along.
				s.hull_state = was_ducked
					? (p.unduck_hull_defer ? 2 : 0) : 0;
			} else {
				s.hull_state = 0;    // ground: stand now
			}
		}

		// CCSGameMovement::HandleDuckingSpeedCrop (0x1f6ba0): baked 0.34 on
		// fwd/side/up when duck-held OR ducking OR ducked, once per tick
		// (the m_iSpeedCropped bit; a per-tick local here). The CALL SITE
		// inside the engine's tick is still being mapped (the isolated
		// diagnostic split 15,979 vs 4,021 on "Duck itself crops") - this
		// mirror exists for isolation and for wiring once measured.
		void HandleDuckingSpeedCrop(const PlayerState& s, int buttons,
		                            float* fwd, float* side, float* up,
		                            bool* cropped) {
			if (cropped && *cropped)
				return;
			if ((buttons & IN_DUCK) || s.ducking || s.ducked) {
				const float k = 0.34f;
				if (fwd)  *fwd *= k;
				if (side) *side *= k;
				if (up)   *up *= k;
				if (cropped) *cropped = true;
			}
		}

		// CCSGameMovement::CheckParameters' MOVE SCALING (0x1f56b0, decoded:
		// spd2 = f*f + s*s + u*u via mv +0x2c/+0x34/+0x38, GetPlayerMaxSpeed
		// min'd with the surface maxspeed factor, then all three moves scale
		// by maxspeed/sqrt(spd2) when over). Runs BEFORE Duck in the tick,
		// which is why ducked ground speed is 0.34 * 250 = 85: the sqrt cap
		// takes 450 -> 250 first, the crop takes 250 -> 85 second. The rest
		// of CheckParameters (IN_SPEED halving, duck-hold curtime gate,
		// angle work) is NOT yet mirrored and stays listed as open.
		void CheckParametersScaleMoves(const MoveParams& p, float* fmove,
		                               float* smove) {
			const float f = *fmove, s = *smove;
			const float spd2 = f * f + s * s;
			if (spd2 > p.maxspeed * p.maxspeed && spd2 > 0.f) {
				const float ratio = p.maxspeed / sqrtf(spd2);
				*fmove *= ratio;
				*smove *= ratio;
			}
		}

		// CCSGameMovement::Duck (0x1f5cb0): the state machine, calling the
		// functions above exactly where the engine's body calls its own.
		// Laws (all isolated-grade-measured 2026-08-15):
		//  - DOWN-counting shared timer; ducking=true through BOTH
		//    transitions (a tapped ground duck stays "ducking" for
		//    TIME_TO_UNDUCK - flips the next jump to the SET path).
		//  - ASYMMETRIC boundaries: duck completes AT exactly TIME_TO_DUCK
		//    (>=), unduck holds AT exactly TIME_TO_UNDUCK (strict >).
		//  - BLOCKED unduck: timer := 1000 and, on a release EDGE only,
		//    ducking := true.
		// NOT yet modeled: the CS curtime duck-hold gate (gm+0xed8) that
		// can force-clear IN_DUCK - zeroed per probe by the runner; needs
		// its cvar identified before the mirror can be honest.
		void Duck(PlayerState& s, const World& w, const MoveParams& p,
		          int buttons, TickEvents* ev, float* fmove, float* smove) {
			// The 0.34 crop fires INSIDE Duck on the PRE-duck state
			// (isolated grades: HandleDuckingSpeedCrop 5,000/5,000 with the
			// pure pre-state condition; the Duck-row fwd/side outputs match
			// pre-state cropping and mismatch post-state on exactly the
			// state-changing rows).
			if (fmove || smove) {
				bool cropped = false;
				HandleDuckingSpeedCrop(s, buttons, fmove, smove, nullptr,
					&cropped);
			}
			const bool want = (buttons & IN_DUCK) != 0;
			const bool was = (s.old_buttons & IN_DUCK) != 0;
			const bool pressed = want && !was;
			if (want)
				s.old_buttons |= IN_DUCK;
			else
				s.old_buttons &= ~IN_DUCK;

			if (!(s.ducking || s.ducked || want))
				return;

			if (want) {
				if (pressed && !s.ducked) {
					s.duck_timer_ms = 1000.f;   // GAMEMOVEMENT_DUCK_TIME
					s.ducking = true;
				}
				if (s.ducking) {
					const float elapsed = 1000.f - s.duck_timer_ms > 0.f
						? 1000.f - s.duck_timer_ms : 0.f;
					if (elapsed >= p.time_to_duck_ms || !s.on_ground
						|| s.ducked)
						FinishDuck(s, w, p, ev);
				}
			} else if (s.ducking || s.ducked) {
				if (CanUnduck(s, w, p)) {
					// Release while FULLY ducked restarts the shared timer
					// for the unduck transition; a mid-duck release keeps
					// the press timer running (fuzz_a t4).
					if (was && !want && s.ducked) {
						s.duck_timer_ms = 1000.f;
						s.ducking = true;
					}
					const float elapsed = 1000.f - s.duck_timer_ms > 0.f
						? 1000.f - s.duck_timer_ms : 0.f;
					if (elapsed > p.time_to_unduck_ms || !s.on_ground)
						FinishUnDuck(s, w, p, ev);
				} else {
					s.duck_timer_ms = 1000.f;
					if (was && !want)
						s.ducking = true;
				}
			}
		}

		// Transitional alias for the tick path.
		void HandleDuck(PlayerState& s, const World& w, const MoveParams& p,
		                int buttons, TickEvents* ev) {
			Duck(s, w, p, buttons, ev, nullptr, nullptr);
		}

	} // namespace

	// FUNCPROBE mirror: exactly the same code the tick runs, callable alone.
	namespace Fn {
		void WishFromInput(float yaw, float fmove, float smove,
		                   float* wx, float* wy) {
			::Solver::WishFromInput(yaw, fmove, smove, wx, wy);
		}
		void CategorizePosition(PlayerState& s, const World& w,
		                        const MoveParams& p) {
			::Solver::CategorizePosition(s, w, p, nullptr);
		}
		bool CheckJumpButton(PlayerState& s, const World& w,
		                     const MoveParams& p) {
			TickEvents ev;
			// The mirror returns the engine's own bool now.
			return ::Solver::CheckJumpButton(s, w, p, &ev);
		}
		void Duck(PlayerState& s, const World& w, const MoveParams& p,
		          int buttons) {
			::Solver::Duck(s, w, p, buttons, nullptr, nullptr, nullptr);
		}
		bool CanUnduck(const PlayerState& s, const World& w,
		               const MoveParams& p) {
			return ::Solver::CanUnduck(s, w, p);
		}
		void FinishDuck(PlayerState& s, const World& w, const MoveParams& p) {
			::Solver::FinishDuck(s, w, p, nullptr);
		}
		void FinishUnDuck(PlayerState& s, const World& w,
		                  const MoveParams& p) {
			::Solver::FinishUnDuck(s, w, p, nullptr);
		}
		void HandleDuckingSpeedCrop(const PlayerState& s, int buttons,
		                            float* fwd, float* side) {
			bool cropped = false;
			::Solver::HandleDuckingSpeedCrop(s, buttons, fwd, side, nullptr,
				&cropped);
		}
		void ReduceTimers(PlayerState& s, const MoveParams& p) {
			// The tick's drains, exactly as MoveTick runs them (the CS body
			// drains stamina; the base drains the duck timer).
			if (s.stamina > 0.f) {
				s.stamina -= p.dt * 1000.f;
				if (s.stamina < 0.f)
					s.stamina = 0.f;
			}
			if (s.duck_timer_ms > 0.f) {
				s.duck_timer_ms -= p.dt * 1000.f;
				if (s.duck_timer_ms < 0.f)
					s.duck_timer_ms = 0.f;
			}
		}
		void ClipVelocity(const Vec3& in, const Vec3& n, Vec3* out) {
			EngineClipVelocity(in, n, *out);
		}
	}

	// OBSERVABILITY ONLY (session 20 search-architecture audit): a
	// process-wide count of authoritative ticks executed. Touches no
	// physics state and no control flow; the thirteen bitwise-replay
	// suites are the no-drift proof.
	long long g_movetick_count = 0;

	void MoveTick(PlayerState& s, const World& w, const MoveParams& p,
	              float /*pitch*/, float yaw, float fmove, float smove,
	              float /*umove*/, int buttons, TickEvents* ev) {
		g_movetick_count++;
		if (ev)
			*ev = TickEvents();

		// BASE VELOCITY PERSISTS (fuzz-measured 2026-08-15, probe 5): with no
		// input at all the engine displaced the player 49x further in XY than
		// velocity*dt, at IDENTICAL velocity and identical z - the delta over
		// dt being exactly the probe's base velocity, still applied a full
		// tick after it was set. The old per-tick decay was wrong. Only the Z
		// component is consumed (StartGravity integrates it and clears it);
		// XY rides along inside Walk/AirMove (added before the move,
		// subtracted after) until a trigger changes it.

		// ReduceTimers: the stamina clock drains every tick, airborne too;
		// the SDK duck timer counts DOWN alongside it.
		if (s.stamina > 0.f) {
			s.stamina -= p.dt * 1000.f;
			if (s.stamina < 0.f)
				s.stamina = 0.f;
		}
		if (s.duck_timer_ms > 0.f) {
			s.duck_timer_ms -= p.dt * 1000.f;
			if (s.duck_timer_ms < 0.f)
				s.duck_timer_ms = 0.f;
		}

		// NOTE (measured, fuzz_a t117 vs fuzz_b t22): m_surfaceFriction is
		// NOT recomputed at tick entry - the value that governs this tick's
		// accel is the one left by the END-OF-MOVE CategorizePosition of the
		// previous tick (which runs BEFORE FinishGravity), plus any duck-
		// transition recategorize. t116 ended its move at vz 143.5, above
		// NON_JUMP_VELOCITY, so the engine skipped the ground probe and left
		// friction 1.0 - and t117 then accelerated on the FULL addspeed
		// (55.6) instead of the 0.25 budget (47.81). A tick-entry recompute
		// (vz 137.5, below the gate) would have forced 0.25 and broken it.

		// PlayerMove: Duck() runs before the move itself, and the SDK
		// re-categorizes position right after it - so a duck-state origin
		// shift (air-duck +8.5 / unduck -8.5) that lands the origin within
		// ground range makes the WHOLE tick a ground tick (friction, walk
		// accel, stay-on-ground). Capture-fitted 2026-08-13 (solved11 t729):
		// the engine ran the unduck-landing tick as ground movement (ended
		// ON the surface at 256.031 with one tick of friction, v 414->389)
		// while the end-categorize-only model ran it as an air tick.
		const bool was_ducked = s.ducked;
		// ENGINE SHAPE, WIRING DEFERRED (measured 2026-08-15): the engine's
		// mechanism is CheckParameters' sqrt scale (450 -> 250) then Duck's
		// internal 0.34 crop (250 -> 85) - both mirrored as functions and
		// probe-verified - but wiring them here produced a 0.001u wobble on
		// the 292 tape (my scale float-detail is not yet exact: umove term,
		// surface maxspeed, possible double math). The covenant is tape
		// truth, so the tick keeps the validated equivalent-on-saturated-
		// inputs cap until CheckParameters' arithmetic is decoded exactly.
		const bool cap_pre = s.ducked || s.ducking;
		Duck(s, w, p, buttons, ev, nullptr, nullptr);
		if (s.ducked != was_ducked)
			CategorizePosition(s, w, p, nullptr);
		const bool cap_ducked = cap_pre || s.ducked || s.ducking;

		// FullWalkMove. StartGravity (SDK): ent_gravity scale applies, then
		// base velocity's Z integrates ONCE into vz and clears.
		s.vel.Z -= s.gravity_scale * p.gravity * 0.5f * p.dt;   // StartGravity
		s.vel.Z += s.basevel.Z * p.dt;
		s.basevel.Z = 0.f;
		if (buttons & IN_JUMP)
			CheckJumpButton(s, w, p, ev);
		else
			s.old_buttons &= ~IN_JUMP;
		if (s.on_ground) {
			s.vel.Z = 0.f;
			Friction(s, p);
		}
		CheckVelocity(s, p);
		if (s.on_ground)
			WalkMove(s, w, p, yaw, fmove, smove, cap_ducked, ev);
		else
			AirMove(s, w, p, yaw, fmove, smove, cap_ducked, ev);
		CategorizePosition(s, w, p, ev);
		CheckVelocity(s, p);
		s.vel.Z -= s.gravity_scale * p.gravity * 0.5f * p.dt;   // FinishGravity
		if (s.on_ground)
			s.vel.Z = 0.f;
		// sv_maxvelocity clamp AFTER the final gravity: fuzz probe 818 fell
		// at terminal speed and the engine reported exactly -3500 where we
		// reported -3506, i.e. gravity added past the cap.
		CheckVelocity(s, p);

		// TRIGGERS (post-move, mirroring the engine's touch order and the
		// DLL RequestSim application 1:1 - total-parity port 2026-08-15):
		// gravity scale persists; push accumulates onto base velocity with
		// the unground + 1u nudge for upward pushes; teleport sets origin
		// and ALWAYS zeroes velocity.
		if (w.HasTriggers()) {
			World::TriggerHitS th;
			if (w.CheckTriggers(s.pos, s.hull_state, &th)) {
				if (th.grav_touched)
					s.gravity_scale = th.gravity;
				if (th.pushed) {
					Vec3 push = th.push_vec;
					if (s.basevel_flag)
						push = push + s.basevel;
					if (push.Z > 0.f && s.on_ground) {
						s.on_ground = false;
						s.ground_brush = -1;
						s.pos.Z += 1.f;
					}
					s.basevel = push;
					s.basevel_flag = true;
				}
				if (th.teleported) {
					s.pos = th.tp_origin;
					s.vel = Vec3();
					if (ev)
						ev->teleported = true;
				}
			}
		}
	}

} // namespace Solver
