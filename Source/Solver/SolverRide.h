#pragma once

// THE CANONICAL RIDE EXECUTOR (ExitField stages 1-2; design of record
// Docs/ExitFieldSpec.md). This is to Ride what Air::FlyWishSchedule is
// to flight: the ONE executor that defines what a ride witness IS. It
// consumes a finite per-tick canonical schedule and stops on the first
// operator-boundary outcome. It contains NO controller, NO splines, NO
// ExitDoomed, NO optimizer and NO human data - proposal machinery may
// eventually generate schedules for it, but nothing here defines the
// feasible set except the exact engine.
//
// TWO REPRESENTATIONS, ONE TRUTH (advisor 2026-08-19g):
//   search coordinates  u_k = (side, cosa, duck)[, mag]  - compact,
//     approximate-friendly, the basis candidate generation thinks in;
//   exact witness       MoveInput packets - PRECISELY the arguments
//     MoveTick consumes, frozen at emission time.
// Candidate generation may be approximate; ACCEPTED WITNESS REPLAY IS
// EXACT (FlyRideInputs, bit-for-bit). The native-inversion gate proves
// the compact coordinates cover the input channel (measured worst
// wish-direction error 5.25e-06 rad); the packet witness carries the
// burden of being the executable proof object.
//
// SEAMS (spec section 2, re-amended 2026-08-19g): the entry
// BoundaryState is the tick-boundary state immediately after the
// accepted board tick (Air::Result::end_state). THE RIDE OWNS AND
// BOOKS THE SEPARATION TICK: an exit classified by an unbooked input
// that the next operator is free to replace is not closed under
// composition (it proves only "exists u: next tick separates"). So
// AIR_EXIT books the first clean-air tick, S+ is the boundary AFTER
// it, and Air begins at the following tick. CONTACT_TRANSFER / GROUND
// / END likewise own their event tick. No tick is double-booked and
// every ExitTransition is self-contained.
//
// COASTING (side = 0) is first-class and MUST NOT erase strafe
// history: the last nonzero side is unchanged and the dwell age keeps
// incrementing through coast ticks - otherwise coasting becomes an
// accidental reset of the six-tick reversal law. Fields the executor
// ignores on a coast tick (cosa, mag) are canonicalized by
// CanonSchedule so two physically identical schedules cannot hash as
// different controls.
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

	// THE EXACT WITNESS PACKET: precisely the arguments MoveTick
	// consumes, no assumptions baked (pitch/umove are stored even
	// though canonical emission zeroes them - an exact packet is
	// exact). Serializable POD.
	struct MoveInput {
		float pitch = 0.f;
		float yaw = 0.f;
		float fmove = 0.f;
		float smove = 0.f;
		float umove = 0.f;
		int   buttons = 0;
	};

	struct Result {
		int  kind = kHorizon;
		bool legal = true;     // schedule admissible vs carried ctl
		int  illegal_tick = -1;
		int  ticks = 0;        // ride ticks BOOKED to this operator
		int  face_ticks = 0;   // ticks that touched the ridden face
		// S+ at the canonical tick boundary AFTER the event tick, with
		// the carried-through control seam state.
		PlayerState end_state;
		Steer::CtlState end_ctl;
		// Event metadata (never state): the separation-tick state for
		// AIR_EXIT, or the first non-ridden contact for
		// CONTACT_TRANSFER.
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

	// Canonicalize the fields the executor IGNORES so physically
	// identical schedules compare/hash identically: a coast tick's
	// cosa and mag are meaningless and are pinned to 1. Any future
	// frontier hashing MUST route through this.
	inline void CanonSchedule(const std::vector<signed char>& side,
	                          std::vector<float>* cosa,
	                          std::vector<float>* mag) {
		for (size_t k = 0; k < side.size(); ++k)
			if (side[k] == 0) {
				if (cosa && k < cosa->size())
					(*cosa)[k] = 1.f;
				if (mag && k < mag->size())
					(*mag)[k] = 1.f;
			}
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

	// Shared per-tick event classification + booking so the two
	// executors below CANNOT drift. Returns true when the ride is over
	// and fills the result; `k` is the tick index just executed. Every
	// event books its own tick (ticks = k + 1) - including AIR_EXIT's
	// separation tick, per the seam rule above.
	inline bool ClassifyTick(const PlayerState& s, const TickEvents& ev,
	                         const Route::Face& fc, int k,
	                         const Steer::CtlState& ctl,
	                         const Vec3* zone_min, const Vec3* zone_max,
	                         Result* r) {
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
			&& s.pos.Z >= zone_min->Z && s.pos.Z <= zone_max->Z) {
			r->kind = kEnd;
			r->ticks = k + 1;
			r->end_state = s;
			r->end_ctl = ctl;
			return true;
		}
		if (s.on_ground) {
			r->kind = kGround;
			r->ticks = k + 1;
			r->end_state = s;
			r->end_ctl = ctl;
			return true;
		}
		if (other >= 0) {
			// Anything that is not the ridden face is an EXPLICIT
			// boundary event (a tap is a transfer, never a hidden
			// continuation). The event tick belongs to the ride; S+
			// is the next face's S_B.
			r->kind = kContactTransfer;
			r->ticks = k + 1;
			r->end_state = s;
			r->end_ctl = ctl;
			r->contact_brush = ev.contact_brush[other];
			r->contact_plane = ev.contact_plane[other];
			r->contact_loss = ev.contact_loss[other];
			return true;
		}
		if (!on_face) {
			// THE SEPARATION TICK IS BOOKED (advisor 2026-08-19g): an
			// exit classified by an unbooked input the next operator
			// is free to replace proves only "exists u: next tick
			// separates" and is not closed under composition. The
			// ride owns the tick that actually produced clean air; S+
			// is the boundary after it; Air begins at the next tick.
			r->kind = kAirExit;
			r->ticks = k + 1;
			r->end_state = s;
			r->end_ctl = ctl;
			r->exit_pos = s.pos;
			r->exit_vel = s.vel;
			return true;
		}
		r->face_ticks++;
		return false;
	}

	// Control seam bookkeeping (dwell crosses seams): coast ticks
	// advance age and never touch the last nonzero side.
	inline void CtlAdvance(Steer::CtlState* ctl, int sd) {
		if (sd != 0 && ctl->side != 0
			&& static_cast<signed char>(sd) != ctl->side)
			ctl->age = 1;
		else
			ctl->age++;
		if (sd != 0)
			ctl->side = static_cast<signed char>(sd);
	}

	// The canonical executor over SEARCH COORDINATES. side/cosa/duck
	// must be equal length; the schedule length IS the horizon M. mag
	// (optional) is a per-tick multiplier on the emitted move
	// magnitude for analog inputs. traj/ctl_traj (optional) receive
	// the state and carried control state at every BOOKED tick
	// boundary. inputs (optional) receives the EXACT MoveInput packet
	// of every booked tick - the executable witness. zone_min/max
	// (optional) arm the END outcome.
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
	                              std::vector<MoveInput>* inputs = nullptr,
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
		for (int k = 0; k < M; ++k) {
			// ---- canonical input emission (the ONE mapping)
			const float s2d = Len2D(s.vel);
			const float h = s2d > 1.f
				? atan2f(s.vel.Y, s.vel.X) : 0.f;
			const int sd = static_cast<int>(
				side[static_cast<size_t>(k)]);
			MoveInput mi;
			mi.yaw = h * 57.2957795f;
			if (sd != 0 && s2d > 1.f)
				Air::WishInputs(h, sd,
					cosa[static_cast<size_t>(k)], &mi.yaw,
					&mi.fmove, &mi.smove);
			if (mag && k < static_cast<int>(mag->size())) {
				mi.fmove *= (*mag)[static_cast<size_t>(k)];
				mi.smove *= (*mag)[static_cast<size_t>(k)];
			}
			mi.buttons = duck[static_cast<size_t>(k)] ? IN_DUCK : 0;
			TickEvents ev;
			MoveTick(s, w, p, mi.pitch, mi.yaw, mi.fmove, mi.smove,
				mi.umove, mi.buttons, &ev);
			CtlAdvance(&ctl, sd);
			if (traj)
				traj->push_back(s);
			if (ctl_traj)
				ctl_traj->push_back(ctl);
			if (inputs)
				inputs->push_back(mi);
			if (ClassifyTick(s, ev, fc, k, ctl, zone_min, zone_max,
				&r))
				return r;
		}
		r.kind = kHorizon;
		r.ticks = M;
		r.end_state = s;
		r.end_ctl = ctl;
		return r;
	}

	// THE AUTHORITATIVE WITNESS REPLAY: consumes frozen MoveInput
	// packets directly - no WishInputs, no float re-derivation - so an
	// accepted witness replays BIT-FOR-BIT. The control seam state is
	// tracked by inverting each packet through the authoritative
	// formulas (needed only for end_ctl and legality accounting; it
	// cannot affect the physics, which consume the packets verbatim).
	inline Result FlyRideInputs(const BoundaryState& B, const World& w,
	                            const MoveParams& p,
	                            const Route::Graph& g,
	                            const std::vector<MoveInput>& in,
	                            std::vector<PlayerState>* traj = nullptr,
	                            const Vec3* zone_min = nullptr,
	                            const Vec3* zone_max = nullptr) {
		Result r;
		r.end_state = B.ps;
		r.end_ctl = B.ctl;
		if (B.face < 0 || B.face >= static_cast<int>(g.faces.size()))
			return r;
		const Route::Face& fc = g.faces[static_cast<size_t>(B.face)];
		PlayerState s = B.ps;
		Steer::CtlState ctl = B.ctl;
		int prev_side = static_cast<int>(B.ctl.side);
		for (size_t k = 0; k < in.size(); ++k) {
			const float s2d = Len2D(s.vel);
			const float h = s2d > 1.f
				? atan2f(s.vel.Y, s.vel.X) : 0.f;
			CanonTick ct;
			InvertWishInput(h, in[k].yaw, in[k].fmove, in[k].smove,
				p, prev_side, &ct);
			if (ct.side != 0)
				prev_side = ct.side;
			TickEvents ev;
			MoveTick(s, w, p, in[k].pitch, in[k].yaw, in[k].fmove,
				in[k].smove, in[k].umove, in[k].buttons, &ev);
			CtlAdvance(&ctl, ct.side);
			if (traj)
				traj->push_back(s);
			if (ClassifyTick(s, ev, fc, static_cast<int>(k), ctl,
				zone_min, zone_max, &r))
				return r;
		}
		r.kind = kHorizon;
		r.ticks = static_cast<int>(in.size());
		r.end_state = s;
		r.end_ctl = ctl;
		return r;
	}

	// ---- THE STAGE-3 TRANSITION RECORD (typed continuations; advisor
	// 2026-08-19g). The route assembler dispatches on `kind`:
	//   kAirExit          -> air continuation (feeds Air/EntranceField)
	//   kContactTransfer  -> ALREADY a board contact on another face -
	//                        no Air leg in between; `board_face` is the
	//                        route face resolved from (brush, plane),
	//                        or -1 when the contact is not a graph face
	//                        (still a valid explicit contact event)
	//   kGround           -> terminal until a Ground operator exists
	//   kEnd              -> the finish, terminal success
	//   kHorizon          -> NO transition (MakeTransition refuses)
	// The witness is the exact executable packet sequence - replaying
	// it through FlyRideInputs reproduces the transition bit-for-bit.
	struct ExitTransition {
		int  kind = kHorizon;
		int  dt = 0;                     // exact ride ticks booked
		PlayerState s_plus;              // continuation at the boundary
		Steer::CtlState ctl_plus;        // carried control seam state
		int  board_face = -1;            // kContactTransfer only
		std::vector<MoveInput> witness;  // the executable proof object
		// event metadata
		Vec3  exit_pos, exit_vel;
		int   contact_brush = -1;
		int   contact_plane = -1;
		float contact_loss = 0.f;
	};

	inline bool MakeTransition(const Result& r,
	                           const Route::Graph& g,
	                           const std::vector<MoveInput>& packets,
	                           ExitTransition* out) {
		if (!r.legal || r.kind == kHorizon)
			return false;   // HORIZON is UNRESOLVED, not a transition
		if (static_cast<int>(packets.size()) < r.ticks)
			return false;   // a witness must carry every booked tick
		out->kind = r.kind;
		out->dt = r.ticks;
		out->s_plus = r.end_state;
		out->ctl_plus = r.end_ctl;
		out->witness.assign(packets.begin(),
			packets.begin() + r.ticks);
		out->exit_pos = r.exit_pos;
		out->exit_vel = r.exit_vel;
		out->contact_brush = r.contact_brush;
		out->contact_plane = r.contact_plane;
		out->contact_loss = r.contact_loss;
		out->board_face = -1;
		if (r.kind == kContactTransfer)
			for (size_t i = 0; i < g.faces.size(); ++i)
				if (g.faces[i].brush == r.contact_brush
					&& g.faces[i].side == r.contact_plane) {
					out->board_face = static_cast<int>(i);
					break;
				}
		return true;
	}

} // namespace Ride
} // namespace Solver
