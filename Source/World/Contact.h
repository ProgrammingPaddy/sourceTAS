#pragma once

#include <cstrike/Structures/Vector.h>

#include "Prediction.h"

// CONTACT-STATE DETECTION (session 46n): the shared capability under the
// hitmarker suite and the snap-board segment. Given any simmed line (the
// editor run line or the live prediction look-ahead - both are per-tick
// Prediction::SimState arrays), classify each tick's contact and emit an
// event at every AIR -> SURFACE transition ("board event").
//
// Two detectors run together, in priority order:
//   1. TAGGED-FACE CROSSING (exact): the hull's support corner crossing a
//      tagged face's plane within the face polygon - the same math the
//      solver's real-pass measurement uses. Catches even zero-loss kisses
//      on the faces the user cares about, and identifies the face.
//   2. PHYSICS RESIDUAL (general): the per-tick velocity change minus
//      gravity exceeds what air-accelerate could add, so a plane clipped
//      the velocity. The residual direction estimates the plane normal,
//      which is then matched against nearby world-brush faces. A residual
//      with NO matching face is reported face-unknown - boosters and
//      teleports also look like impulses, so callers should only trust
//      face-known events (the editor additionally filters trigger ticks).
//   GROUND onset (FL_ONGROUND newly set after air) is always an event.
namespace Contact {

	struct BoardEvent {
		int    tick = 0;           // the tick DURING which contact happened
		Vector pos;                // feet origin at the end of that tick
		Vector normal;             // face normal (exact if face_known, else
		                           // estimated from the velocity residual)
		int    brush = -1;         // matched face (-1 = unresolved)
		int    plane = -1;
		bool   face_known = false; // brush/plane are a real matched face
		bool   grounded = false;   // FL_ONGROUND landing (vs a surf board)
		float  arrive_speed = 0.f; // 3D speed entering the tick
		float  arrive_speed2d = 0.f;
		float  clip_loss = 0.f;    // u/s the plane clip removed (0 = clean)
	};

	// Scan states[1..count-1] (each tick needs its predecessor) and write up
	// to max_out events. `gravity` u/s^2 and `max_add` (max air-accel gain
	// per tick, u/s) bound what free flight can explain; both are the
	// caller's measured values. min_air = airborne ticks required before a
	// contact counts as a board (default 2 - a slide along a ramp clips
	// every tick and must not spam events). Returns the event count.
	int Analyze(const Prediction::SimState* states, int count,
	            float interval, float gravity, float max_add,
	            BoardEvent* out, int max_out, int min_air = 2);
}
