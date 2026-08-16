#include "SolverLedger.h"

#include <math.h>
#include <stdio.h>

#include "SolverBoard.h"

namespace Solver {
namespace Ledger {

	namespace {

		// Per-tick classification of the replay.
		enum class TickKind { Ground, Air, Ride };

	} // namespace

	bool Build(const World& w, const Route::Graph& g, const MoveParams& p,
	           const Tape& tape, Run* out,
	           int sabotage_t0, int sabotage_t1) {
		auto find_face = [&](int brush, int plane) {
			for (size_t i = 0; i < g.faces.size(); ++i)
				if (g.faces[i].brush == brush && g.faces[i].side == plane)
					return static_cast<int>(i);
			return -1;
		};
		PlayerState s;
		s.pos = tape.start.origin;
		s.vel = tape.start.velocity;
		s.ducked = tape.start.ducked;
		s.hull_state = tape.start.ducked ? 1 : 0;
		s.stamina = tape.start.stamina;
		{
			TraceResult tr;
			const float gf = w.TraceHull(s.pos,
				s.pos - Vec3(0.f, 0.f, 2.f), s.ducked, &tr);
			if (gf < 1.f && tr.brush >= 0
				&& tr.normal.Z >= p.walkable_z) {
				s.pos.Z -= 2.f * gf;
				s.on_ground = true;
				s.ground_brush = tr.brush;
			}
		}
		out->phases.clear();
		out->total_dissipation = 0.f;
		out->total_shortfall = 0.f;
		out->ticks = static_cast<int>(tape.frames.size());

		Phase cur;
		bool have_cur = false;
		int air_gap = 0;   // contact-free ticks inside a ride (< 3 merge)
		auto close_phase = [&](int last_t) {
			if (!have_cur)
				return;
			cur.t1 = last_t;
			if (cur.kind == Kind::Air) {
				cur.gain2 = cur.s2d_out * cur.s2d_out
					- cur.s2d_in * cur.s2d_in;
				cur.max_gain2 = p.air_speed_cap * p.air_speed_cap
					* static_cast<float>(cur.Ticks());
				cur.shortfall = cur.max_gain2 - cur.gain2;
				out->total_shortfall += cur.shortfall;
			}
			if (cur.kind == Kind::Ride) {
				cur.grav_conv = 2.f * p.gravity
					* (cur.z_in - cur.z_out);
				cur.wish_work = cur.v2_out + cur.ride_dot2
					- cur.v2_in - cur.grav_conv;
				out->total_dissipation += cur.ride_dot2;
			}
			out->phases.push_back(cur);
			have_cur = false;
		};
		auto open_phase = [&](Kind k, int t, int face) {
			cur = Phase();
			cur.kind = k;
			cur.t0 = t;
			cur.face = face;
			cur.v2_in = Len2(s.vel);
			cur.s2d_in = Len2D(s.vel);
			cur.z_in = s.pos.Z;
			have_cur = true;
		};

		for (size_t t = 0; t < tape.frames.size(); ++t) {
			const TapeFrame& f = tape.frames[t];
			const bool sab = sabotage_t0 >= 0
				&& static_cast<int>(t) >= sabotage_t0
				&& static_cast<int>(t) < sabotage_t1;
			TickEvents ev;
			// The entering state for boards this tick.
			MoveTick(s, w, p, f.pitch, f.yaw, sab ? 0.f : f.fmove,
				sab ? 0.f : f.smove, f.umove, f.buttons, &ev);
			// Classify the tick.
			TickKind tk;
			int face = -1;
			if (s.on_ground) {
				tk = TickKind::Ground;
			} else if (ev.ncontacts > 0) {
				const int b = ev.contact_brush[0];
				const int pl = ev.contact_plane[0];
				if (b >= 0 && pl >= 0) {
					const Vec3& n = w.brushes[b].n[pl];
					face = (n.Z > 0.f && n.Z < p.walkable_z)
						? find_face(b, pl) : -1;
				}
				tk = TickKind::Ride;
			} else {
				tk = TickKind::Air;
			}
			const int ti = static_cast<int>(t);
			switch (tk) {
			case TickKind::Ground:
				if (!have_cur || cur.kind != Kind::Ground) {
					close_phase(ti - 1);
					open_phase(Kind::Ground, ti, -1);
				}
				break;
			case TickKind::Air:
				if (have_cur && cur.kind == Kind::Ride
					&& air_gap < 2) {
					air_gap++;   // brief separation: still the ride
					break;
				}
				if (!have_cur || cur.kind != Kind::Air) {
					close_phase(ti - 1);
					open_phase(Kind::Air, ti, -1);
				}
				break;
			case TickKind::Ride: {
				const bool same = have_cur && cur.kind == Kind::Ride
					&& cur.face == face;
				if (!same) {
					const bool from_air = have_cur
						&& cur.kind == Kind::Air;
					close_phase(ti - 1);
					open_phase(Kind::Ride, ti, face);
					// The board: first clip arriving from the air.
					if (from_air && ev.ncontacts >= 1 && face >= 0) {
						const Vec3& v1 = ev.contact_vel[0];
						const Route::Face& fc = g.faces[face];
						cur.board_dot = Dot(v1, fc.n);
						cur.board_dot2 = cur.board_dot
							* cur.board_dot;
						const float l2 = Len2(v1);
						cur.board_frac = l2 > 1.f
							? cur.board_dot2 / l2 : 0.f;
						const float md = Board::MinApproachDot(
							Len2D(v1), v1.Z, fc.n);
						cur.board_min2 = md < 1e8f ? md * md : 0.f;
					}
				}
				air_gap = 0;
				for (int c = 0; c < ev.ncontacts; ++c) {
					const int b = ev.contact_brush[c];
					const int pl = ev.contact_plane[c];
					if (b < 0 || pl < 0)
						continue;
					const Vec3& n = w.brushes[b].n[pl];
					const float d = Dot(ev.contact_vel[c], n);
					cur.ride_dot2 += d * d;
					cur.cross_bound += p.gravity * p.dt
						* fabsf(n.Z) * fabsf(d);
				}
				break;
			}
			}
			if (have_cur) {
				cur.v2_out = Len2(s.vel);
				cur.s2d_out = Len2D(s.vel);
				cur.z_out = s.pos.Z;
			}
		}
		close_phase(static_cast<int>(tape.frames.size()) - 1);
		return true;
	}

	void Print(const Run& run, int detail_top_n) {
		printf("ledger: %d phases over %d ticks | dissipation %.0f "
			"(speed-equiv %.1f u/s at 1000) | air shortfall %.0f\n",
			static_cast<int>(run.phases.size()), run.ticks,
			run.total_dissipation,
			run.total_dissipation > 0.f
				? 1000.f - sqrtf(1000.f * 1000.f
					- (run.total_dissipation < 1e6f
						? run.total_dissipation : 1e6f)) : 0.f,
			run.total_shortfall);
		for (size_t i = 0; i < run.phases.size(); ++i) {
			const Phase& ph = run.phases[i];
			const char* kn = ph.kind == Kind::Ground ? "GND "
				: (ph.kind == Kind::Air ? "AIR " : "RIDE");
			if (ph.kind == Kind::Air)
				printf("  [%2d] %s t%4d..%4d (%3d) s2d %6.1f->%6.1f "
					"| gain2 %8.0f / max %7.0f | SHORTFALL %8.0f\n",
					static_cast<int>(i), kn, ph.t0, ph.t1,
					ph.Ticks(), ph.s2d_in, ph.s2d_out, ph.gain2,
					ph.max_gain2, ph.shortfall);
			else if (ph.kind == Kind::Ride)
				printf("  [%2d] %s t%4d..%4d (%3d) face %d s2d "
					"%6.1f->%6.1f | BOARD dot %7.1f loss2 %8.0f "
					"frac %.3f min2 %6.0f | ride dots2 %8.0f grav "
					"%9.0f wish %8.0f (+/-%.0f)\n",
					static_cast<int>(i), kn, ph.t0, ph.t1,
					ph.Ticks(), ph.face, ph.s2d_in, ph.s2d_out,
					ph.board_dot, ph.board_dot2, ph.board_frac,
					ph.board_min2, ph.ride_dot2, ph.grav_conv,
					ph.wish_work, ph.cross_bound);
			else
				printf("  [%2d] %s t%4d..%4d (%3d) s2d %6.1f->%6.1f\n",
					static_cast<int>(i), kn, ph.t0, ph.t1,
					ph.Ticks(), ph.s2d_in, ph.s2d_out);
		}
		// The largest boards, spelled out for hand analysis.
		if (detail_top_n > 0) {
			printf("ledger: largest boards (hand-check: loss2 = dot^2;"
				" regret = dot^2 - min2):\n");
			std::vector<int> idx;
			for (size_t i = 0; i < run.phases.size(); ++i)
				if (run.phases[i].kind == Kind::Ride
					&& run.phases[i].board_dot2 > 0.f)
					idx.push_back(static_cast<int>(i));
			for (size_t a = 0; a < idx.size(); ++a)
				for (size_t b = a + 1; b < idx.size(); ++b)
					if (run.phases[idx[b]].board_dot2
						> run.phases[idx[a]].board_dot2) {
						const int tmp = idx[a];
						idx[a] = idx[b];
						idx[b] = tmp;
					}
			for (size_t a = 0; a < idx.size()
				&& a < static_cast<size_t>(detail_top_n); ++a) {
				const Phase& ph = run.phases[idx[a]];
				printf("  board@t%d face %d: dot %.1f -> loss2 %.0f "
					"(%.1f%% of v1^2) | tangency min2 %.0f -> "
					"REGRET %.0f\n", ph.t0, ph.face, ph.board_dot,
					ph.board_dot2, 100.f * ph.board_frac,
					ph.board_min2, ph.board_dot2 - ph.board_min2);
			}
		}
		fflush(stdout);
	}

} // namespace Ledger
} // namespace Solver
