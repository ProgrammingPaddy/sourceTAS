#pragma once

// THE CANONICAL RIDE EXECUTOR (ExitField stage 1; design of record
// Docs/ExitFieldSpec.md). This is Ride what Air::FlyWishSchedule is to
// flight: the ONE executor that defines what a ride witness IS. It
// consumes a finite per-tick canonical schedule and stops on the first
// operator-boundary outcome. It contains NO controller, NO splines, NO
// ExitDoomed, NO optimizer and NO human data - proposal machinery may
// eventually generate schedules for it, but nothing here defines the
// feasible set except the exact engine.
//
// Canonical control basis (per tick):
//     u_k = (side_k in {-1,0,+1}, cosa_k, duck_k)  [+ optional mag_k]
// side = 0 is the COASTING channel (zero wish). Wish emission routes
// through Air::WishInputs - the ONE place the stored-basis convention
// lives - so ride and flight can never drift apart. mag (optional)
// scales the emitted move magnitude for analog sub-cap inputs; the
// keyboard input model never needs it (all full-key combos exceed the
// air wishspeed cap, making direction the only physical channel).
//
// SEAMS (spec section 2): the entry BoundaryState is the tick-boundary
// state immediately after the accepted board tick (Air::Result::
// end_state). The executor owns ride ticks through the LAST ridden-
// face-contact tick. AIR_EXIT is confirmed by a one-tick PEEK whose
// INPUT is part of the witness (it is the existence proof that a
// continuation in clean air exists) but whose TICK is never booked:
// S+ is the boundary BEFORE the peeked tick, ticks excludes it, and
// the next operator re-simulates that tick under its own control -
// the peek's simulation is classification-only. (Measured by exitfit
// first run: a witness holding only the booked prefix cannot be
// classified standalone - the executor runs out of inputs while the
// last tick still touches the face and honestly reports HORIZON.)
// CONTACT_TRANSFER / GROUND / END own their event tick; S+ is the
// boundary after it.
//
// HORIZON is not an exit: it means the supplied controls ended while
// still riding, contributes no transition, and leaves the domain
// UNRESOLVED (standing law B-C).

#include <vector>

#include "SolverAir.h"
#include "SolverMove.h"
#include "SolverRoute.h"
#include "SolverSteer.h"

namespace Solver {
namespace Ride {

	// First-boundary outcome kinds (spec section 1).
	enum EventKind {
		kAirExit = 0,          // clean separation into air
		kContactTransfer = 1,  // touched anything not the ridden face
		kGround = 2,           // grounded on a walkable
		kEnd = 3,              // entered the end-zone volume
		kHorizon = 4,          // controls ended while still riding
	};

	// The complete boundary payload S_B (spec section 3): the WHOLE
	// PlayerState - never a hand-pruned subset - plus the control seam
	// state (Invariant 9) and the ridden-face operator identity.
	struct BoundaryState {
		PlayerState ps;
		Steer::CtlState ctl;
		int face = -1;         // Route::Graph face index
	};

	struct Result {
		int  kind = kHorizon;
		bool legal = true;     // schedule admissible vs carried ctl
		int  illegal_tick = -1;
		int  ticks = 0;        // ride ticks BOOKED to this operator
		int  face_ticks = 0;   // ticks that touched the ridden face
		// S+ at the canonical tick boundary (see seam rules above),
		// with the carried-through control seam state.
		PlayerState end_state;
		Steer::CtlState end_ctl;
		// Event metadata (never state): AIR_EXIT separation boundary,
		// or the first non-ridden contact for CONTACT_TRANSFER.
		Vec3  exit_pos, exit_vel;
		int   contact_brush = -1;
		int   contact_plane = -1;
		float contact_loss = 0.f;
	};

	// Schedule legality against the CARRIED control state: the dwell
	// law crosses operator seams, so the first reversal of this ride
	// is constrained by the previous operator's last. Coast ticks
	// (side = 0) advance age and change nothing else - identical
	// semantics to the Entrance search's sched_legal.
	inline bool SchedLegal(const Steer::CtlState& ctl0,
	                       const std::vector<signed char>& side,
	                       int min_gap, int* bad_tick = nullptr) {
		signed char cur = ctl0.side;
		int age = ctl0.age;
		for (size_t k = 0; k < side.size(); ++k) {
			const signed char v = side[k];
			if (v != 0 && cur != 0 && v != cur) {
				if (age < min_gap) {
					if (bad_tick)
						*bad_tick = static_cast<int>(k);
					return false;
				}
				age = 1;
			} else {
				age++;
			}
			if (v != 0)
				cur = v;
		}
		return true;
	}

	// One native input tick inverted into the canonical basis THROUGH
	// the authoritative formulas: Fn::WishFromInput derives the wish
	// the engine would build, and the (side, cosa) solution inverts
	// Air::WishInputs' emission (wish realized at wh + pi with
	// wh = h + side*acos(cosa)). `analog` marks a tick whose wish
	// magnitude is physically distinct from full-key input: the engine
	// uses min(|wishvel|, maxspeed) in the accelspeed term, so any
	// |wishvel| >= maxspeed is equivalent and anything below is not.
	// Degenerate directions (wish exactly along/against velocity) make
	// the side label physically irrelevant, so it inherits prev_side
	// for dwell-legality continuity.
	struct CanonTick {
		int   side = 0;
		float cosa = 1.f;
		float mag = 1.f;      // emission multiplier (450 * mag)
		bool  analog = false;
	};

	inline void InvertWishInput(float h, float yaw_deg, float fmove,
	                            float smove, const MoveParams& p,
	                            int prev_side, CanonTick* out) {
		float wx = 0.f, wy = 0.f;
		Fn::WishFromInput(yaw_deg, fmove, smove, &wx, &wy);
		const float wmag = sqrtf(wx * wx + wy * wy);
		if (wmag < 1e-3f) {
			out->side = 0;
			out->cosa = 1.f;
			out->mag = 1.f;
			out->analog = false;
			return;
		}
		const float kPi = 3.14159265f;
		const float phi_w = atan2f(wy, wx);
		const float wh = Steer::WrapPi(phi_w - kPi);
		const float delta = Steer::WrapPi(wh - h);
		if (fabsf(delta) < 1e-4f || fabsf(delta) > kPi - 1e-4f)
			out->side = prev_side != 0 ? prev_side : 1;
		else
			out->side = delta > 0.f ? 1 : -1;
		out->cosa = cosf(fabsf(delta));
		out->analog = wmag < p.maxspeed - 1e-3f;
		out->mag = out->analog ? wmag / 450.f : 1.f;
	}

	// The canonical executor. side/cosa/duck must be equal length; the
	// schedule length IS the horizon M. mag (optional) is a per-tick
	// multiplier on the emitted move magnitude for analog inputs.
	// traj/ctl_traj (optional) receive the state and carried control
	// state at every tick boundary the ride executed (exitfit's
	// comparison hooks). zone_min/max (optional) arm the END outcome.
	inline Result FlyRideSchedule(const BoundaryState& B, const World& w,
	                              const MoveParams& p,
	                              const Route::Graph& g,
	                              const std::vector<signed char>& side,
	                              const std::vector<float>& cosa,
	                              const std::vector<unsigned char>& duck,
	                              const std::vector<float>* mag = nullptr,
	                              std::vector<PlayerState>* traj = nullptr,
	                              std::vector<Steer::CtlState>* ctl_traj
	                                  = nullptr,
	                              const Vec3* zone_min = nullptr,
	                              const Vec3* zone_max = nullptr) {
		Result r;
		r.end_state = B.ps;
		r.end_ctl = B.ctl;
		if (B.face < 0 || B.face >= static_cast<int>(g.faces.size()))
			return r;
		const Route::Face& fc = g.faces[static_cast<size_t>(B.face)];
		const int M = static_cast<int>(side.size());
		const int min_gap = static_cast<int>(
			ceilf((1.f / p.dt) / p.strafe_rate_max));
		// An illegal schedule is NOT a witness: refuse, never run.
		// Refusal is a legality verdict, not a boundary event.
		if (!SchedLegal(B.ctl, side, min_gap, &r.illegal_tick)) {
			r.legal = false;
			return r;
		}
		PlayerState s = B.ps;
		Steer::CtlState ctl = B.ctl;
		PlayerState prev = s;
		Steer::CtlState prev_ctl = ctl;
		for (int k = 0; k < M; ++k) {
			prev = s;
			prev_ctl = ctl;
			// ---- canonical input emission (the ONE mapping)
			const float s2d = Len2D(s.vel);
			const float h = s2d > 1.f
				? atan2f(s.vel.Y, s.vel.X) : 0.f;
			const int sd = static_cast<int>(
				side[static_cast<size_t>(k)]);
			float yaw_deg = h * 57.2957795f;
			float fmove = 0.f, smove = 0.f;
			if (sd != 0 && s2d > 1.f)
				Air::WishInputs(h, sd,
					cosa[static_cast<size_t>(k)], &yaw_deg,
					&fmove, &smove);
			if (mag && k < static_cast<int>(mag->size())) {
				fmove *= (*mag)[static_cast<size_t>(k)];
				smove *= (*mag)[static_cast<size_t>(k)];
			}
			const int btn = duck[static_cast<size_t>(k)]
				? IN_DUCK : 0;
			TickEvents ev;
			MoveTick(s, w, p, 0.f, yaw_deg, fmove, smove, 0.f, btn,
				&ev);
			// ---- control seam bookkeeping (dwell crosses seams)
			if (sd != 0 && ctl.side != 0
				&& static_cast<signed char>(sd) != ctl.side)
				ctl.age = 1;
			else
				ctl.age++;
			if (sd != 0)
				ctl.side = static_cast<signed char>(sd);
			if (traj)
				traj->push_back(s);
			if (ctl_traj)
				ctl_traj->push_back(ctl);
			// ---- classify the tick (priority: END > GROUND >
			// CONTACT_TRANSFER > separation peek > riding)
			bool on_face = false;
			int other = -1;
			for (int c = 0; c < ev.ncontacts; ++c) {
				if (ev.contact_brush[c] == fc.brush
					&& ev.contact_plane[c] == fc.side)
					on_face = true;
				else if (other < 0)
					other = c;
			}
			if (zone_min && zone_max
				&& s.pos.X >= zone_min->X && s.pos.X <= zone_max->X
				&& s.pos.Y >= zone_min->Y && s.pos.Y <= zone_max->Y
				&& s.pos.Z >= zone_min->Z
				&& s.pos.Z <= zone_max->Z) {
				r.kind = kEnd;
				r.ticks = k + 1;
				r.end_state = s;
				r.end_ctl = ctl;
				return r;
			}
			if (s.on_ground) {
				r.kind = kGround;
				r.ticks = k + 1;
				r.end_state = s;
				r.end_ctl = ctl;
				return r;
			}
			if (other >= 0) {
				// Anything that is not the ridden face is an EXPLICIT
				// boundary event (a tap is a transfer, never a hidden
				// continuation). The event tick belongs to the ride;
				// S+ is the next face's S_B.
				r.kind = kContactTransfer;
				r.ticks = k + 1;
				r.end_state = s;
				r.end_ctl = ctl;
				r.contact_brush = ev.contact_brush[other];
				r.contact_plane = ev.contact_plane[other];
				r.contact_loss = ev.contact_loss[other];
				return r;
			}
			if (!on_face) {
				// Clean-air tick: separation was already complete at
				// the PREVIOUS boundary. This tick is a PEEK owned by
				// Air - its input is not consumed.
				r.kind = kAirExit;
				r.ticks = k;
				r.end_state = prev;
				r.end_ctl = prev_ctl;
				r.exit_pos = prev.pos;
				r.exit_vel = prev.vel;
				if (traj)
					traj->pop_back();   // the peek is not ride state
				if (ctl_traj)
					ctl_traj->pop_back();
				return r;
			}
			r.face_ticks++;
		}
		r.kind = kHorizon;
		r.ticks = M;
		r.end_state = s;
		r.end_ctl = ctl;
		return r;
	}

} // namespace Ride
} // namespace Solver
