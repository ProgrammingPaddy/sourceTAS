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

		void CheckVelocity(PlayerState& s, const MoveParams& p) {
			auto clampc = [&](float& c) {
				if (c > p.maxvelocity) c = p.maxvelocity;
				else if (c < -p.maxvelocity) c = -p.maxvelocity;
			};
			clampc(s.vel.X);
			clampc(s.vel.Y);
			clampc(s.vel.Z);
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
			for (int bump = 0; bump < 4; ++bump) {
				if (Len2(s.vel) == 0.f)
					break;
				const Vec3 end = s.pos + Scale(s.vel, time_left);
				TraceResult tr;
				const float frac = w.TraceHull3(s.pos, end, s.hull_state, &tr);
				if (frac > 0.f) {
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
					ev->ncontacts++;
				}
				if (numplanes >= 5) {
					s.vel = Vec3();
					break;
				}
				planes[numplanes++] = n;
				const float sp_before = Len(s.vel);
				if (numplanes == 1) {
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
		}

		// CategorizePosition: the engine's ONLY grounding rule. Falling no
		// faster up than NON_JUMP_VELOCITY, a 2-unit down-trace onto a walkable
		// plane (nz >= 0.7) sets ground AND SNAPS the origin to the trace end.
		void CategorizePosition(PlayerState& s, const World& w, const MoveParams& p,
		                        TickEvents* ev) {
			const bool was_ground = s.on_ground;
			s.on_ground = false;
			s.ground_brush = -1;
			if (s.vel.Z <= p.non_jump_velocity) {
				TraceResult tr;
				const float gf = w.TraceHull3(s.pos, s.pos - Vec3(0.f, 0.f, 2.f),
				                             s.hull_state, &tr);
				if (gf < 1.f && tr.brush >= 0 && tr.normal.Z >= p.walkable_z) {
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
			const float drop = control * p.friction * p.dt;   // surface friction 1
			float newspeed = speed - drop;
			if (newspeed < 0.f) newspeed = 0.f;
			if (newspeed != speed)
				s.vel = Scale(s.vel, newspeed / speed);
		}

		void CheckJumpButton(PlayerState& s, const World& /*w*/, const MoveParams& p,
		                     TickEvents* ev) {
			if (!s.on_ground) {
				s.old_buttons |= IN_JUMP;
				return;
			}
			if (s.old_buttons & IN_JUMP)
				return;   // must release jump before jumping again
			// PreventBunnyJumping (server, binary-scanned): clamp velocity to
			// 1.1*maxspeed before the impulse - OFF under sv_enablebunnyhopping.
			if (!p.enablebunnyhopping) {
				const float maxscaled = p.maxspeed * 1.1f;
				const float spd = Len(s.vel);
				if (spd > maxscaled && spd > 0.f)
					s.vel = Scale(s.vel, maxscaled / spd);
			}
			s.on_ground = false;
			s.ground_brush = -1;
			// Engine-measured 2026-08-13 (basictest ground truth): standing
			// jumps ADD sqrt(2g*57)=302 onto the post-StartGravity vz (-6),
			// ducked/ducking jumps SET it - tick-end vz 284 vs 290, both exact.
			// Hot-stamina jumps are taxed (REAL-playback fit, 4 jumps - see
			// MoveParams). The earlier "no tax" conclusion came from pairing
			// the wrong sim export with a rewritten tape file - retracted.
			// FORMULA CORRECTED by 5-jump regression (2026-08-14): the
			// stamina ratio applies to the WHOLE vz after the impulse
			// (StartGravity's -6 included), not to the impulse alone - the
			// regressed effective impulse-only scale 0.00018622 equals
			// 0.00019*(296/302) exactly, unmasking the textbook scale
			// under the whole-vz application. Line-fit residuals < 0.001.
			const float impulse = sqrtf(2.f * p.gravity * p.jump_height);
			if (s.ducked || s.ducking)
				s.vel.Z = impulse;
			else
				s.vel.Z += impulse;
			if (s.stamina > 0.f)
				s.vel.Z *= 1.f - s.stamina * p.stamina_jump_scale_per_ms;
			if (p.jump_finishgravity)
				s.vel.Z -= p.gravity * 0.5f * p.dt;   // SDK's in-jump FinishGravity
				                                      // (confirmed by the -6 -6 tail)
			// The jump ARMS the stamina timer (see MoveParams: jump tax,
			// drained in flight, residue drags the landing walk).
			s.stamina = p.stamina_jump_ms;
			s.old_buttons |= IN_JUMP;
			if (ev) {
				ev->jumped = true;
				ev->left_ground = true;
			}
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
			// Applied after Friction, before Accelerate (fit order A).
			if (s.stamina > 0.f) {
				// Affine drag law (see MoveParams: regressed slope +
				// measured intercept).
				const float ratio = 1.f - (s.stamina * p.stamina_scale_per_ms
					+ p.stamina_walk_offset);
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
				float accelspeed = p.accelerate * p.dt * wishspeed;   // friction 1
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
			const Vec3 dest(s.pos.X + s.vel.X * p.dt,
			                s.pos.Y + s.vel.Y * p.dt, s.pos.Z);
			TraceResult tr;
			const float frac = w.TraceHull3(s.pos, dest, s.hull_state, &tr);
			if (frac >= 1.f) {
				s.pos = dest;
			} else {
				StepMove(s, w, p, ev);
			}
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
				float accelspeed = p.airaccelerate * wishspeed * p.dt;   // friction 1
				if (accelspeed > add) accelspeed = add;
				s.vel = s.vel + Scale(wishdir, accelspeed);
			}
			TryPlayerMove(s, w, p, ev);
		}

		// Duck state machine (CGameMovement::Duck, CS hulls). In the air a duck
		// finishes INSTANTLY with the origin pulled up by (standing - ducked)
		// hull height (the measured ~18u FinishDuck lift); on the ground the
		// hull swaps after TIME_TO_DUCK. Unduck reverses the shift and must fit.
		void HandleDuck(PlayerState& s, const World& w, const MoveParams& p,
		                int buttons, TickEvents* ev) {
			const bool want = (buttons & IN_DUCK) != 0;
			// Engine-measured shift (see MoveParams), NOT the hull delta.
			const float lift = p.duck_air_shift;
			if (want) {
				if (s.ducked)
					return;
				if (!s.ducking) {
					s.ducking = true;
					s.duck_elapsed_ms = 0.f;
				}
				s.duck_elapsed_ms += p.dt * 1000.f;
				if (!s.on_ground || s.duck_elapsed_ms >= p.time_to_duck_ms) {
					// FinishDuck.
					s.ducked = true;
					s.hull_state = 1;       // duck: flag and hull together
					s.ducking = false;
					if (ev) ev->duck_changed = true;
					if (!s.on_ground) {
						s.pos.Z += lift;
						// FixPlayerCrouchStuck: nudge up until the ducked hull
						// fits (rare in open air; bounded).
						for (int i = 0; i < 18 && w.OriginInSolid(s.pos, true); ++i)
							s.pos.Z += 1.f;
					}
				}
			} else {
				if (s.ducking) {
					s.ducking = false;
					s.duck_elapsed_ms = 0.f;
				}
				if (s.ducked) {
					if (!s.on_ground) {
						const Vec3 cand(s.pos.X, s.pos.Y, s.pos.Z - lift);
						if (!w.OriginInSolid(cand, false)) {
							s.pos = cand;
							s.ducked = false;
							// AIR unduck: the flag clears and the origin
							// shifts, but the COLLISION hull stays ducked
							// until grounding (battery+oracle fitted - see
							// PlayerState::hull_ducked).
							s.hull_state = p.unduck_hull_defer
								? 2 : 0;   // transient top 62.5
							if (ev) ev->duck_changed = true;
						}
						// No room: stay ducked (engine behavior).
					} else {
						if (!w.OriginInSolid(s.pos, false)) {
							s.ducked = false;
							s.hull_state = 0;        // ground: stand now
							if (ev) ev->duck_changed = true;
						}
					}
				}
			}
		}

	} // namespace

	void MoveTick(PlayerState& s, const World& w, const MoveParams& p,
	              float /*pitch*/, float yaw, float fmove, float smove,
	              float /*umove*/, int buttons, TickEvents* ev) {
		if (ev)
			*ev = TickEvents();

		// ReduceTimers: the stamina clock drains every tick, airborne too.
		if (s.stamina > 0.f) {
			s.stamina -= p.dt * 1000.f;
			if (s.stamina < 0.f)
				s.stamina = 0.f;
		}

		// PlayerMove: Duck() runs before the move itself, and the SDK
		// re-categorizes position right after it - so a duck-state origin
		// shift (air-duck +8.5 / unduck -8.5) that lands the origin within
		// ground range makes the WHOLE tick a ground tick (friction, walk
		// accel, stay-on-ground). Capture-fitted 2026-08-13 (solved11 t729):
		// the engine ran the unduck-landing tick as ground movement (ended
		// ON the surface at 256.031 with one tick of friction, v 414->389)
		// while the end-categorize-only model ran it as an air tick.
		const bool was_ducked = s.ducked;
		const bool cap_pre = s.ducked || s.ducking;
		HandleDuck(s, w, p, buttons, ev);
		if (s.ducked != was_ducked)
			CategorizePosition(s, w, p, nullptr);
		// The tick's speed cap is ducked if the duck state was ducked or
		// ducking at ANY point in this tick's duck processing (press ticks
		// AND unduck ticks both cap ducked - see WalkMove/battery notes).
		const bool cap_ducked = cap_pre || s.ducked || s.ducking;

		// FullWalkMove.
		s.vel.Z -= p.gravity * 0.5f * p.dt;                     // StartGravity
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
		s.vel.Z -= p.gravity * 0.5f * p.dt;                     // FinishGravity
		if (s.on_ground)
			s.vel.Z = 0.f;

	}

} // namespace Solver
