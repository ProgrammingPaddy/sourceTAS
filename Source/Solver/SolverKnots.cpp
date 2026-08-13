#include "SolverKnots.h"

namespace Solver {

	float NormYawDeg(float y) {
		while (y > 180.f) y -= 360.f;
		while (y < -180.f) y += 360.f;
		return y;
	}

	float MaxGainYaw(const Vec3& vel, int side, bool ducked,
	                 const MoveParams& p) {
		// With add budget a and cap L, the max-gain wish sits at
		// acos(max(L-a,0)/speed) off the velocity; at surf settings that is
		// 90 deg - look along velocity, hold the strafe key. Derived from the
		// LIVE params every tick, so a cvar change changes the strafing.
		const float sp = Len2D(vel);
		const float heading = atan2f(vel.Y, vel.X) * (180.f / kPi);
		const float effmax = ducked ? p.maxspeed * p.duck_speed_frac
		                            : p.maxspeed;
		const float wishspeed = fminf(450.f, effmax);
		const float a = p.airaccelerate * wishspeed * p.dt;
		float theta = 90.f;
		if (sp > 1.f) {
			const float cstar = fmaxf(p.air_speed_cap - a, 0.f);
			float c = cstar / sp;
			if (c > 1.f) c = 1.f;
			theta = acosf(c) * (180.f / kPi);
		}
		return NormYawDeg(heading + static_cast<float>(side) * (theta - 90.f));
	}

	void KnotTick(PlayerState& s, float& yaw, const Knot& k, int tick_in_knot,
	              const World& w, const MoveParams& p, TickEvents* ev,
	              TapeFrame* emit) {
		float fmove = 0.f, smove = 0.f;
		int buttons = 0;
		if (k.side != 0) {
			if (s.on_ground) {
				// Prestrafe-style ground knot: W + strafe key, yaw drifting
				// into the turn at the knot's sampled rate.
				fmove = 450.f;
				buttons |= IN_FORWARD;
				smove = (k.side > 0) ? -450.f : 450.f;
				buttons |= (k.side > 0) ? IN_MOVELEFT : IN_MOVERIGHT;
				yaw = NormYawDeg(yaw + k.ground_turn
					* (k.side > 0 ? 1.f : -1.f));
			} else {
				// Air knot: pure strafe key, max-gain controller.
				smove = (k.side > 0) ? -450.f : 450.f;
				buttons |= (k.side > 0) ? IN_MOVELEFT : IN_MOVERIGHT;
				yaw = MaxGainYaw(s.vel, k.side, s.ducked, p);
			}
		}
		if ((k.flags & 1) && tick_in_knot == 0)
			buttons |= IN_JUMP;
		// Duck presses and releases happen AIRBORNE only - the proven duck
		// regimes (air-duck ±8.5, ducked landing, ducked jump, air-unduck)
		// all matched real playback; GROUND duck transitions are engine
		// lifecycle the model doesn't carry (measured divergences).
		bool want_duck = (k.flags & 2) != 0;
		if (s.on_ground) {
			if (s.ducked)
				want_duck = true;    // never release duck on the ground
			else
				want_duck = false;   // never start a duck on the ground
		}
		if (want_duck)
			buttons |= IN_DUCK;
		if (emit) {
			emit->pitch = 0.f;
			emit->yaw = yaw;
			emit->fmove = fmove;
			emit->smove = smove;
			emit->umove = 0.f;
			emit->buttons = buttons;
		}
		MoveTick(s, w, p, 0.f, yaw, fmove, smove, 0.f, buttons, ev);
	}

	Knot SampleKnot(std::mt19937& rng, signed char last_side, short since_flip,
	                int min_knot) {
		Knot kn;
		kn.dur = static_cast<short>(min_knot + rng() % 57);
		const bool can_flip = (last_side == 0) || (since_flip >= min_knot);
		const int r = static_cast<int>(rng() % 100);
		if (last_side == 0)
			kn.side = (r < 45) ? 1 : (r < 90) ? -1 : 0;
		else if (r < 40)
			kn.side = last_side;
		else if (r < 80 && can_flip)
			kn.side = static_cast<signed char>(-last_side);
		else if (r < 90)
			kn.side = 0;
		else
			kn.side = last_side;
		kn.ground_turn = 0.5f + static_cast<float>(rng() % 1000) * 0.003f;
		const unsigned f = rng() % 100;
		if (f < 10) kn.flags |= 1;         // jump press
		else if (f < 16) kn.flags |= 2;    // duck held
		return kn;
	}

	bool FlipsLegal(const std::vector<Knot>& knots, int min_knot,
	                signed char last_side0, short since_flip0) {
		signed char last_side = last_side0;
		int since_flip = since_flip0;
		for (const Knot& k : knots) {
			if (k.side != 0 && last_side != 0 && k.side != last_side) {
				if (since_flip < min_knot)
					return false;
				since_flip = 0;
			}
			if (k.side != 0)
				last_side = k.side;
			since_flip += k.dur;
			if (since_flip > 999)
				since_flip = 999;
		}
		return true;
	}

} // namespace Solver
