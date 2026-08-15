#include "SolverSmooth.h"
#include "SolverExplore.h" // ResolveThreadCount
#include "SolverKnots.h"   // NormYawDeg, button conventions

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <random>
#include <thread>
#include <vector>

namespace Solver {

	namespace {

		// Origin-space footprint test (hull-expanded box), same rule the lab
		// and explorer use for zone exit / finish checks.
		bool InsideXY(const Vec3& p, const WorldBrush& b) {
			return p.X >= b.gmin_stand.X && p.X <= b.gmax_stand.X
				&& p.Y >= b.gmin_stand.Y && p.Y <= b.gmax_stand.Y;
		}

		double Clampd(double v, double lo, double hi) {
			return v < lo ? lo : (v > hi ? hi : v);
		}

	} // namespace

	SmoothOpt::SmoothOpt(const World& w, const SmoothConfig& cfg,
	                     const TapeAnchor& anchor)
		: w_(w), cfg_(cfg) {
		if (cfg_.cp_ticks < 14)
			cfg_.cp_ticks = 14;   // flips <= 5/s, structural
		ncp_ = cfg_.max_ticks / cfg_.cp_ticks + 2;

		root_.pos = anchor.origin;
		root_.vel = anchor.velocity;
		root_.ducked = anchor.ducked;
		root_.hull_state = anchor.ducked ? 1 : 0;
		root_.stamina = anchor.stamina;
		root_yaw_ = anchor.yaw;
		// Ground settle exactly like replay: 2u down-probe, no physics tick.
		TraceResult tr;
		const float gf = w_.TraceHull(root_.pos,
			root_.pos - Vec3(0.f, 0.f, 2.f), root_.ducked, &tr);
		if (gf < 1.f && tr.brush >= 0
			&& tr.normal.Z >= cfg_.params.walkable_z) {
			root_.pos.Z -= 2.f * gf;
			root_.on_ground = true;
			root_.ground_brush = tr.brush;
		}

		start_idx_ = w_.IndexOfBrushId(cfg_.start_brush_id);
		end_idx_ = w_.IndexOfBrushId(cfg_.end_brush_id);
		if (end_idx_ >= 0) {
			const WorldBrush& b = w_.brushes[end_idx_];
			end_center_ = Vec3(0.5f * (b.bmin.X + b.bmax.X),
			                   0.5f * (b.bmin.Y + b.bmax.Y), b.bmax.Z);
		}
		if (cfg_.chain_genes && cfg_.goal_face_brush < 0 && end_idx_ >= 0) {
			// END-landing guidance frame: the landing footprint's top
			// rectangle RAISED by land_margin (the probe insight as a
			// controller - home on a gene-chosen point ABOVE the landing,
			// arc over the lip, descend in).
			const WorldBrush& b = w_.brushes[end_idx_];
			face_n_ = Vec3(0.f, 0.f, 1.f);
			face_d_ = b.bmax.Z + cfg_.land_margin;
			face_center_ = Vec3(0.5f * (b.gmin_stand.X + b.gmax_stand.X),
				0.5f * (b.gmin_stand.Y + b.gmax_stand.Y),
				b.bmax.Z + cfg_.land_margin);
			face_t1_ = Vec3(1.f, 0.f, 0.f);
			face_t2_ = Vec3(0.f, 1.f, 0.f);
			face_e1_ = 0.5f * (b.gmax_stand.X - b.gmin_stand.X);
			face_e2_ = 0.5f * (b.gmax_stand.Y - b.gmin_stand.Y);
		}
		if (cfg_.goal_face_brush >= 0
			&& cfg_.goal_face_brush < static_cast<int>(w_.brushes.size())) {
			// Target-face frame: center projected onto the plane + two
			// tangent axes + extents (the 8 AABB corners projected). The
			// guidance genes pick the board point inside this rectangle.
			const WorldBrush& b = w_.brushes[cfg_.goal_face_brush];
			Vec3 c = Vec3(0.5f * (b.bmin.X + b.bmax.X),
			              0.5f * (b.bmin.Y + b.bmax.Y),
			              0.5f * (b.bmin.Z + b.bmax.Z));
			const int pi = cfg_.goal_face_plane;
			if (pi >= 0 && pi < static_cast<int>(b.n.size())) {
				face_n_ = b.n[pi];
				face_d_ = b.d[pi];
				const float off = Dot(face_n_, c) - face_d_;
				c = c - Scale(face_n_, off);
				// Horizontal along-face axis + up-slope axis.
				Vec3 t1(-face_n_.Y, face_n_.X, 0.f);
				const float t1l = Len(t1);
				face_t1_ = t1l > 1e-4f ? Scale(t1, 1.f / t1l)
					: Vec3(1.f, 0.f, 0.f);
				face_t2_ = Vec3(
					face_n_.Y * face_t1_.Z - face_n_.Z * face_t1_.Y,
					face_n_.Z * face_t1_.X - face_n_.X * face_t1_.Z,
					face_n_.X * face_t1_.Y - face_n_.Y * face_t1_.X);
				float e1 = 0.f, e2 = 0.f;
				for (int k = 0; k < 8; ++k) {
					const Vec3 corner(
						(k & 1) ? b.bmax.X : b.bmin.X,
						(k & 2) ? b.bmax.Y : b.bmin.Y,
						(k & 4) ? b.bmax.Z : b.bmin.Z);
					const Vec3 r = corner - c;
					e1 = fmaxf(e1, fabsf(Dot(r, face_t1_)));
					e2 = fmaxf(e2, fabsf(Dot(r, face_t2_)));
				}
				face_e1_ = e1;
				face_e2_ = e2;
			}
			face_center_ = c;
		}
		float mz = 1e9f;
		for (const WorldBrush& b : w_.brushes)
			mz = fminf(mz, b.bmin.Z);
		world_min_z_ = mz - 512.f;
	}

	void SmoothOpt::SetRoot(const PlayerState& s, float yaw) {
		root_ = s;
		root_yaw_ = yaw;
	}

	bool SmoothOpt::SeedFromTape(const Tape& tape) {
		if (tape.frames.empty())
			return false;
		const MoveParams& p = cfg_.params;
		const int n = static_cast<int>(tape.frames.size());
		seed_frames_ = tape.frames;
		duck_overlay_.assign(n, 0);

		// Core replay of the tape, inverting each tick's controls into the
		// u-language (the projection must SPEAK the decoder's dialect or the
		// init lands nowhere near the line - measured: the side-skeleton
		// projection spun the prestrafe in circles).
		std::vector<float> ut(n, 0.f);
		PlayerState s = root_;
		float prev_yaw = root_yaw_;
		int jump_tick = -1;
		for (int t = 0; t < n; ++t) {
			const TapeFrame& f = tape.frames[t];
			duck_overlay_[t] = (f.buttons & IN_DUCK) ? 1 : 0;
			int side = 0;
			if (f.buttons & IN_MOVELEFT) side += 1;
			if (f.buttons & IN_MOVERIGHT) side -= 1;
			const float sp = Len2D(s.vel);
			if (s.on_ground) {
				// Ground decode is yaw ARC at 3 deg/tick per unit u.
				const float dy = NormYawDeg(f.yaw - prev_yaw);
				ut[t] = static_cast<float>(Clampd(dy / 3.0, -1.0, 1.0));
			} else if (side != 0 && sp > 1.f) {
				// Air decode: yaw = heading + side*(phi-90) inverted, then
				// phi -> u through the same add map the decoder uses.
				const float heading = atan2f(s.vel.Y, s.vel.X)
					* (180.f / kPi);
				float phi = 90.f + side * NormYawDeg(f.yaw - heading);
				if (phi < 0.f)
					phi = 0.f;
				float au;
				if (phi >= 90.f)
					au = 1.f + (phi - 90.f) / cfg_.overdrive_deg;
				else
					au = 1.f - sp * cosf(phi * (kPi / 180.f))
						/ p.air_speed_cap;
				au = static_cast<float>(Clampd(au, 0.05, 2.0));
				ut[t] = side * au;
			} else {
				ut[t] = 0.f;
			}
			prev_yaw = f.yaw;
			TickEvents ev;
			MoveTick(s, w_, p, f.pitch, f.yaw, f.fmove, f.smove, f.umove,
				f.buttons, &ev);
			if (jump_tick < 0 && ev.jumped)
				jump_tick = t;
		}

		seed_mean_.assign(static_cast<size_t>(ncp_) + 3, 0.0);
		for (int i = 0; i < ncp_; ++i) {
			const int c = i * cfg_.cp_ticks;
			const int a = c - cfg_.cp_ticks / 2;
			const int b = c + cfg_.cp_ticks / 2;
			double sum = 0.0;
			int cnt = 0;
			for (int t = a; t < b; ++t) {
				if (t < 0 || t >= n)
					continue;
				sum += ut[t];
				cnt++;
			}
			seed_mean_[i] = cnt > 0 ? sum / cnt : 0.0;
		}
		seed_mean_[ncp_] = jump_tick >= 0 ? jump_tick / 10.0 : -1.0;
		// Duck-gene init: the tape's LAST pump (press/release pair) - the
		// landing-area duck is the one the ending BVP needs first.
		int last_on = -1, last_off = -1;
		for (int t = 1; t < n; ++t) {
			if (duck_overlay_[t] && !duck_overlay_[t - 1])
				last_on = t;
			if (!duck_overlay_[t] && duck_overlay_[t - 1]
				&& last_on >= 0)
				last_off = t;
		}
		if (cfg_.seed_duck && last_on >= 0) {
			seed_mean_[ncp_ + 1] = last_on / 10.0;
			seed_mean_[ncp_ + 2] = (last_off > last_on ? last_off : n) / 10.0;
		} else {
			seed_mean_[ncp_ + 1] = -0.5;
			seed_mean_[ncp_ + 2] = -0.5;
		}
		have_seed_mean_ = true;
		return true;
	}

	float SmoothOpt::DistToEnd(const Vec3& p, float vz) const {
		if (end_idx_ < 0)
			return Len(p - end_center_);
		const WorldBrush& b = w_.brushes[end_idx_];
		const float dx = fmaxf(fmaxf(b.gmin_stand.X - p.X, 0.f),
			p.X - b.gmax_stand.X);
		const float dy = fmaxf(fmaxf(b.gmin_stand.Y - p.Y, 0.f),
			p.Y - b.gmax_stand.Y);
		float dz = p.Z - b.bmax.Z;
		if (dz < 0.f) {
			// Below the lip: the gap minus what BALLISTICS can still buy -
			// only upward vz lifts a free flight (total speed credited the
			// climb let 882 u/s under-platform lines read distance 0).
			const float climb = vz > 0.f
				? vz * vz / (2.f * cfg_.params.gravity) : 0.f;
			dz = fmaxf(0.f, -dz - climb);
		}
		return sqrtf(dx * dx + dy * dy + dz * dz);
	}

	float SmoothOpt::SplineEval(const std::vector<double>& x, int t) const {
		// Catmull-Rom through the CPs (CP i sits at tick i*cp_ticks).
		const int seg = t / cfg_.cp_ticks;
		const double fx = static_cast<double>(t - seg * cfg_.cp_ticks)
			/ cfg_.cp_ticks;
		auto cp = [&](int i) {
			if (i < 0) i = 0;
			if (i >= ncp_) i = ncp_ - 1;
			return x[i];
		};
		const double p0 = cp(seg - 1), p1 = cp(seg), p2 = cp(seg + 1),
			p3 = cp(seg + 2);
		const double v = 0.5 * ((2.0 * p1)
			+ (-p0 + p2) * fx
			+ (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * fx * fx
			+ (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * fx * fx * fx);
		return static_cast<float>(v);
	}

	double SmoothOpt::Evaluate(const std::vector<double>& x,
	                           SmoothStats* stats,
	                           std::vector<TapeFrame>* emit) const {
		const MoveParams& p = cfg_.params;
		PlayerState s = root_;
		float yaw = root_yaw_;
		SmoothStats st;

		const int jump_tick = x[ncp_] < 0.0 ? -1
			: static_cast<int>(x[ncp_] * 10.0 + 0.5);
		const int duck_on = x[ncp_ + 1] < 0.0 ? -1
			: static_cast<int>(x[ncp_ + 1] * 10.0 + 0.5);
		const int duck_off = x[ncp_ + 2] < 0.0 ? (1 << 28)
			: static_cast<int>(x[ncp_ + 2] * 10.0 + 0.5);
		// Guidance genes (chain segments): board/landing point + gain +
		// engagement tick (the release decision).
		Vec3 board_target = face_center_;
		float guide_gain = 0.f;
		int guide_from = 0;
		float carve = 0.f;
		if (cfg_.chain_genes && static_cast<int>(x.size()) >= ncp_ + 8)
			carve = static_cast<float>(Clampd(x[ncp_ + 7], -1.0, 1.0));
		if (cfg_.chain_genes
			&& static_cast<int>(x.size()) >= ncp_ + 7) {
			const float tx = static_cast<float>(
				Clampd(x[ncp_ + 3], 0.0, 1.0)) * 2.f - 1.f;
			const float tyu = cfg_.ty_lo + (cfg_.ty_hi - cfg_.ty_lo)
				* static_cast<float>(Clampd(x[ncp_ + 4], 0.0, 1.0));
			const float ty = tyu * 2.f - 1.f;
			guide_gain = static_cast<float>(
				Clampd(x[ncp_ + 5], 0.0, 1.0));
			guide_from = x[ncp_ + 6] < 0.0 ? 0
				: static_cast<int>(x[ncp_ + 6] * 10.0 + 0.5);
			board_target = face_center_
				+ Scale(face_t1_, tx * 0.8f * face_e1_)
				+ Scale(face_t2_, ty * 0.8f * face_e2_);
		}

		signed char last_side = 0;
		short since_flip = 999;
		bool launch_jump = false;
		int touch_at = -1;
		int last_contact = -1;
		// Structural ride hold: the steep face contacted most recently.
		int hold_fresh = -999;
		Vec3 hold_n;
		float e_prev = 0.5f * Len2(s.vel) + p.gravity * s.pos.Z;
		bool zone_exited = start_idx_ < 0;
		const int follow = cfg_.seed_follow
			< static_cast<int>(seed_frames_.size())
			? cfg_.seed_follow : static_cast<int>(seed_frames_.size());
		int zone_jumps = 0;
		int grounded_run = 0;

		for (int t = 0; t < cfg_.max_ticks; ++t) {
			float pitch = 0.f, fmove = 0.f, smove = 0.f, umove = 0.f;
			int buttons = 0;

			if (t < follow) {
				// Prefix-follow: the seed tape's frame, verbatim.
				const TapeFrame& f = seed_frames_[t];
				pitch = f.pitch;
				yaw = f.yaw;
				fmove = f.fmove;
				smove = f.smove;
				umove = f.umove;
				buttons = f.buttons;
				// Keep the flip clock honest across the boundary.
				const signed char ts = f.smove < 0.f ? 1
					: (f.smove > 0.f ? -1 : 0);
				if (ts != 0) {
					if (last_side != 0 && ts != last_side)
						since_flip = 0;
					last_side = ts;
				}
				if (since_flip < 999)
					since_flip++;
			} else {
				float u = SplineEval(x, t);
				u = static_cast<float>(Clampd(u, -2.0, 2.0));

				// CARVE-HOLD: pre-release (or during the settle), while
				// steep-face contact is fresh, the wish direction blends
				// from pure into-face (carve 0) toward along-face travel
				// (|carve| -> 1, sign = which end). Explicit wish, own
				// yaw target; the same 15 deg/tick cap and 14-tick flip
				// law apply. This is the traverse the user described -
				// rides that climb frontally bleed to ~350 u/s (measured).
				bool carved = false;
				if (cfg_.chain_genes && !s.on_ground
					&& t - hold_fresh <= 3
					&& (t < guide_from || st.touched)) {
					Vec3 t1h(-hold_n.Y, hold_n.X, 0.f);
					const float t1l = Len2D(t1h);
					if (t1l > 1e-4f) {
						t1h = Scale(t1h, 1.f / t1l);
						const float ac = fabsf(carve);
						float wx = -hold_n.X * (1.f - ac) + t1h.X * carve;
						float wy = -hold_n.Y * (1.f - ac) + t1h.Y * carve;
						const float wl = sqrtf(wx * wx + wy * wy);
						if (wl > 1e-4f) {
							wx /= wl;
							wy /= wl;
							const float wyaw = atan2f(wy, wx)
								* (180.f / kPi);
							const float v1 = NormYawDeg(wyaw - 90.f);
							const float v2 = NormYawDeg(wyaw + 90.f);
							int side = fabsf(NormYawDeg(v1 - yaw))
								<= fabsf(NormYawDeg(v2 - yaw)) ? 1 : -1;
							if (last_side != 0 && side != last_side
								&& since_flip < 14)
								side = last_side;
							const float target = side > 0 ? v1 : v2;
							float dyw = NormYawDeg(target - yaw);
							if (dyw > 15.f) dyw = 15.f;
							else if (dyw < -15.f) dyw = -15.f;
							yaw = NormYawDeg(yaw + dyw);
							buttons |= (side > 0) ? IN_MOVELEFT
								: IN_MOVERIGHT;
							smove = (side > 0) ? -450.f : 450.f;
							if (last_side != 0 && side != last_side)
								since_flip = 0;
							last_side = static_cast<signed char>(side);
							if (since_flip < 999)
								since_flip++;
							carved = true;
						}
					}
				}
				// CLOSED-LOOP GUIDANCE (chain segments, in flight, after
				// release): predict the ballistic crossing of the target
				// plane, steer the horizontal velocity to null the miss
				// against the gene-chosen point. The guided u passes
				// through the same wish mapping, flip guard, and yaw cap
				// as any u - position control the open-loop spline cannot
				// do.
				if (!carved && cfg_.chain_genes && guide_gain > 0.01f
					&& t >= guide_from && !st.touched && !s.on_ground) {
					const float spg = Len2D(s.vel);
					if (spg > 50.f) {
						const float sd = Dot(face_n_, s.pos) - face_d_;
						const float aq = -0.5f * p.gravity * face_n_.Z;
						const float bq = Dot(face_n_, s.vel);
						float tau = -1.f;
						if (fabsf(aq) < 1e-3f) {
							if (bq < -1.f)
								tau = -sd / bq;
						} else {
							const float disc = bq * bq - 4.f * aq * sd;
							if (disc >= 0.f) {
								const float sq = sqrtf(disc);
								const float r1 = (-bq + sq) / (2.f * aq);
								const float r2 = (-bq - sq) / (2.f * aq);
								tau = 1e9f;
								if (r1 > 0.f && r1 < tau) tau = r1;
								if (r2 > 0.f && r2 < tau) tau = r2;
								if (tau > 1e8f) tau = -1.f;
							}
						}
						if (tau > 0.f) {
							if (tau > 3.f) tau = 3.f;
							// TANGENT LEAD-IN: aim at a point displaced
							// from the board point along the face plane,
							// collapsing as the range closes - the final
							// approach asymptotes the tangent line,
							// nulling the normal component BEFORE contact.
							// (Measured without it: pure pursuit built
							// hard landings by construction - V collapsed
							// link by link, ramp-3 boards at spd 349.)
							// MINIMUM-LEAD rule: a FRONTAL approach has no
							// in-plane component, the lead vanishes, and
							// pursuit reverts to a smash (measured: face
							// segments died on ramp 4 while a through-
							// flight boarded it) - synthesize the lead
							// along the face's horizontal axis toward the
							// board point's side: a CARVE entry.
							const Vec3 toT = board_target - s.pos;
							const float toTl = Len(toT);
							Vec3 inp = toT
								- Scale(face_n_, Dot(face_n_, toT));
							float ipl = Len(inp);
							if (ipl < 0.3f * toTl) {
								const float sgn =
									Dot(toT, face_t1_) >= 0.f ? 1.f : -1.f;
								inp = inp + Scale(face_t1_,
									sgn * 0.5f * toTl);
								ipl = Len(inp);
							}
							Vec3 aim_pt = board_target;
							if (ipl > 1.f)
								aim_pt = board_target - Scale(
									Scale(inp, 1.f / ipl),
									0.5f * toTl);
							Vec3 pred = s.pos + Scale(s.vel, tau);
							pred.Z -= 0.5f * p.gravity * tau * tau;
							const Vec3 miss = aim_pt - pred;
							const Vec3 aimv(
								aim_pt.X - s.pos.X + miss.X,
								aim_pt.Y - s.pos.Y + miss.Y, 0.f);
							if (Len2D(aimv) > 1.f) {
								const float ah = atan2f(aimv.Y, aimv.X)
									* (180.f / kPi);
								const float hh = atan2f(s.vel.Y, s.vel.X)
									* (180.f / kPi);
								const float err = NormYawDeg(ah - hh);
								const float turn = fmaxf(0.5f,
									p.air_speed_cap / spg
										* (180.f / kPi));
								float ua = err / turn;
								if (ua > 1.5f) ua = 1.5f;
								else if (ua < -1.5f) ua = -1.5f;
								u = u * (1.f - guide_gain)
									+ ua * guide_gain;
							}
						}
					}
				}

				// Structural flip guard: a sign change sooner than 14 ticks
				// after the previous one holds the old side (spline
				// overshoot between CPs could otherwise wiggle twice).
				signed char want = u > 0.02f ? 1 : (u < -0.02f ? -1 : 0);
				if (want != 0 && last_side != 0 && want != last_side
					&& since_flip < 14) {
					want = last_side;
					u = static_cast<float>(last_side) * fabsf(u);
				}

				const float sp = Len2D(s.vel);
				const float heading = sp > 1.f
					? atan2f(s.vel.Y, s.vel.X) * (180.f / kPi) : yaw;

				if (carved) {
					// carve-hold already set yaw/keys this tick
				} else if (s.on_ground) {
					// Ground: W-accelerate; u arcs the yaw (prestrafe) with
					// the matching strafe key held.
					fmove = 450.f;
					buttons |= IN_FORWARD;
					if (fabsf(u) >= 0.05f) {
						buttons |= (u > 0.f) ? IN_MOVELEFT : IN_MOVERIGHT;
						smove = (u > 0.f) ? -450.f : 450.f;
						const float arc = static_cast<float>(
							Clampd(u, -1.0, 1.0)) * 3.f;
						yaw = NormYawDeg(yaw + arc);
					}
				} else if (fabsf(u) >= 0.02f) {
					const int side = u > 0.f ? 1 : -1;
					const float au = fminf(fabsf(u), 2.f);
					// Wish angle off the velocity heading. |u|<=1
					// interpolates the ADD linearly: the wish projection
					// eats (1-|u|) of the live cap; |u|>1 over-rotates past
					// 90 (carve). All from LIVE params + state.
					float phi;
					if (au >= 1.f) {
						phi = 90.f + (au - 1.f) * cfg_.overdrive_deg;
					} else if (sp > 1.f) {
						float c = (1.f - au) * p.air_speed_cap / sp;
						if (c > 1.f) c = 1.f;
						phi = acosf(c) * (180.f / kPi);
					} else {
						phi = 90.f;
					}
					// STRUCTURAL yaw-rate cap: the target chases the live
					// heading, which turns noisy at low speed (measured 177
					// deg/tick thrash at speed 19). 15 deg/tick is far above
					// any human usage (their max: 5.1) yet makes snapping
					// unrepresentable - smoothness is the decoder's contract.
					const float target = NormYawDeg(heading
						+ static_cast<float>(side) * (phi - 90.f));
					float dyw = NormYawDeg(target - yaw);
					if (dyw > 15.f) dyw = 15.f;
					else if (dyw < -15.f) dyw = -15.f;
					yaw = NormYawDeg(yaw + dyw);
					buttons |= (side > 0) ? IN_MOVELEFT : IN_MOVERIGHT;
					smove = (side > 0) ? -450.f : 450.f;
				} else {
					// Coast: view eases onto the line (no accel input).
					if (sp > 1.f) {
						float dyw = NormYawDeg(heading - yaw);
						if (dyw > 15.f) dyw = 15.f;
						else if (dyw < -15.f) dyw = -15.f;
						yaw = NormYawDeg(yaw + dyw);
					}
				}

				// Duck genes: held on [duck_on, duck_off). Transitions are
				// AIRBORNE-only (the proven duck regimes) - on the ground
				// the current duck state is held, never toggled.
				bool want_duck = duck_on >= 0
					&& t >= duck_on && t < duck_off;
				if (s.on_ground)
					want_duck = s.ducked;
				if (want_duck)
					buttons |= IN_DUCK;

				// Jump gene, under the ONE universal ban: never a second
				// startzone jump (the prefix may already have spent it).
				if (t == jump_tick
					&& !(zone_jumps >= 1 && !zone_exited))
					buttons |= IN_JUMP;

				if (!carved) {
					if (want != 0) {
						if (last_side != 0 && want != last_side)
							since_flip = 0;
						last_side = want;
					}
					if (since_flip < 999)
						since_flip++;
				}
			}

			if (emit) {
				TapeFrame f;
				f.pitch = pitch;
				f.yaw = yaw;
				f.fmove = fmove;
				f.smove = smove;
				f.umove = umove;
				f.buttons = buttons;
				emit->push_back(f);
			}

			TickEvents ev;
			MoveTick(s, w_, p, pitch, yaw, fmove, smove, umove, buttons, &ev);
			st.sim_ticks++;
			if (ev.jumped && !zone_exited)
				zone_jumps++;
			// Ride-hold bookkeeping: remember the steep face just clipped.
			if (cfg_.chain_genes) {
				for (int c = 0; c < ev.ncontacts; ++c) {
					const WorldBrush& cb = w_.brushes[ev.contact_brush[c]];
					const int cp = ev.contact_plane[c];
					if (cp >= 0 && cp < static_cast<int>(cb.n.size())) {
						const Vec3& cn = cb.n[cp];
						if (cn.Z > 0.05f && cn.Z < p.walkable_z) {
							hold_n = cn;
							hold_fresh = t;
						}
					}
				}
			}

			// Energy ledger (dissipation = every drop of KE + PE).
			const float e_now = 0.5f * Len2(s.vel) + p.gravity * s.pos.Z;
			if (e_now < e_prev)
				st.eloss += e_prev - e_now;
			e_prev = e_now;

			float impact = 0.f;
			for (int c = 0; c < ev.ncontacts; ++c)
				impact += ev.contact_loss[c];
			if (impact > st.max_impact)
				st.max_impact = impact;
			if (ev.landed)
				st.nlandings++;

			// Ending classifier (identical to explorer/optimizer): the class
			// is the LAST surface departure; touching any non-finish face
			// resets a jump taint, the landing on the end brush does not.
			if (ev.left_ground)
				launch_jump = ev.jumped;
			else if (!s.on_ground && ev.ncontacts > 0
				&& w_.brushes[ev.contact_brush[0]].id != cfg_.end_brush_id)
				launch_jump = false;

			if (!zone_exited && start_idx_ >= 0
				&& !InsideXY(s.pos, w_.brushes[start_idx_])) {
				zone_exited = true;
				st.exit_tick = t;
			}

			const float d = cfg_.goal_face_brush >= 0
				? Len(s.pos - face_center_)
				: DistToEnd(s.pos, s.vel.Z);
			if (d < st.dmin) {
				st.dmin = d;
				st.eloss_at_dmin = st.eloss;
				st.vmix_at_dmin = 0.5f * Len2(s.vel)
					+ cfg_.mix_mu * p.gravity * s.pos.Z;
			}

			// Segment mode: success = boarding the target face and RIDING
			// it - the junction is a CONTACT tick at least touch_settle
			// after the first touch. A tap-and-fly (graze, then ballistic)
			// never qualifies: losing the face for a full window cancels
			// the touch (measured failure: energy-optimal "settles" were
			// single grazes flying away, and every child segment died).
			if (cfg_.goal_face_brush >= 0) {
				bool hit = false;
				for (int c = 0; c < ev.ncontacts; ++c)
					if (ev.contact_brush[c] == cfg_.goal_face_brush
						&& (cfg_.goal_face_plane < 0
							|| ev.contact_plane[c] == cfg_.goal_face_plane))
						hit = true;
				if (hit) {
					if (!st.touched) {
						st.touched = true;
						touch_at = t;
					}
					last_contact = t;
					// Finalize only INSIDE the face polygon (with margin):
					// rim grazes ride the plane's edge for ticks without
					// ever being ON the face - that is not a board.
					const Vec3 r = s.pos - face_center_;
					const bool in_rect =
						fabsf(Dot(r, face_t1_)) <= face_e1_ * 0.9f
						&& fabsf(Dot(r, face_t2_)) <= face_e2_ * 0.9f;
					if (in_rect && t >= touch_at + cfg_.touch_settle) {
						st.clean = !launch_jump;
						st.tick = t;
						st.rel = st.exit_tick >= 0 ? t - st.exit_tick : t;
						st.finish_speed = Len2D(s.vel);
						st.vboard = 0.5f * Len2(s.vel)
							+ cfg_.chain_mu * p.gravity * s.pos.Z;
						st.end_state = s;
						st.end_yaw = yaw;
						break;
					}
					if (t - touch_at > 120) {
						st.touched = false;   // stuck grazing; not a board
						touch_at = -1;
					}
				} else if (st.touched
					&& t - last_contact > cfg_.touch_settle) {
					st.touched = false;   // lost the face; a later board
					touch_at = -1;        // may re-arm
				}
			} else if (end_idx_ >= 0 && s.on_ground && s.ground_brush >= 0
				&& w_.brushes[s.ground_brush].id == cfg_.end_brush_id
				&& InsideXY(s.pos, w_.brushes[end_idx_])) {
				st.finished = true;
				st.clean = !launch_jump;
				st.tick = t;
				st.rel = st.exit_tick >= 0 ? t - st.exit_tick : t;
				st.finish_speed = Len2D(s.vel);
				st.end_state = s;
				st.end_yaw = yaw;
				break;
			}
			if (s.pos.Z < world_min_z_) {
				st.touched = false;   // died mid-settle: not a junction
				break;
			}

			// Sim-budget bounds (same class as max_path_ticks, never route
			// rules): a line that walks grounded for a full second after the
			// zone, or never leaves the zone at all, is not going anywhere
			// this rollout - stop paying for it.
			if (s.on_ground)
				grounded_run++;
			else
				grounded_run = 0;
			if (zone_exited) {
				if (grounded_run > 60 && s.ground_brush >= 0
					&& w_.brushes[s.ground_brush].id != cfg_.end_brush_id) {
					st.touched = false;
					break;
				}
			} else if (t > 400) {
				break;
			}
		}
		// Rollout ended inside the settle window (domain edge): no junction.
		if (cfg_.goal_face_brush >= 0 && st.touched && st.tick == 0)
			st.touched = false;

		if (stats)
			*stats = st;

		// Genotype leash: decode clamps to [-2,2]; a tiny quadratic outside
		// keeps CMA from drifting along the flat region.
		double leash = 0.0;
		for (int i = 0; i < ncp_; ++i) {
			const double a = fabs(x[i]);
			if (a > 2.0)
				leash += 0.001 * (a - 2.0) * (a - 2.0);
		}

		if (cfg_.fitness_mode == 1) {
			// Junction fitness (CMA is rank-based - raw energy units are
			// fine): maximize board value minus fitted waste, pay per tick.
			if (st.touched)
				return cfg_.w_tick * st.tick
					- (st.vboard - cfg_.wloss * st.eloss) + leash;
			return 1e9 + st.dmin + leash;
		}
		if (st.finished && st.clean)
			return st.rel + st.eloss * 1e-7 - 100000.0 + leash;
		if (st.finished)
			return st.rel + st.eloss * 1e-7 + 5000.0 + leash;
		// Non-finisher shaping: the energy-aware distance already orders
		// states by landability; the fitted waste penalty orders equal
		// approaches by how much line they burned getting there.
		return 10000.0 + st.dmin / 10.0
			+ cfg_.wloss * st.eloss_at_dmin * 1e-6 + leash;
	}

	bool SmoothOpt::BuildFrames(const std::vector<double>& x,
	                            std::vector<TapeFrame>& out,
	                            SmoothStats* stats) {
		out.clear();
		SmoothStats st;
		Evaluate(x, &st, &out);
		if ((st.finished || st.touched)
			&& static_cast<int>(out.size()) > st.tick + 1)
			out.resize(st.tick + 1);
		if (stats)
			*stats = st;
		return !out.empty();
	}

	// ---- CMA-ES (Hansen's standard formulation, doubles throughout) ----

	namespace {

		// Cyclic Jacobi eigendecomposition of a symmetric matrix. C is n*n
		// row-major; on return B holds eigenvectors (columns), D eigenvalues.
		void JacobiEig(int n, const std::vector<double>& C,
		               std::vector<double>& B, std::vector<double>& D) {
			std::vector<double> A = C;
			B.assign(static_cast<size_t>(n) * n, 0.0);
			for (int i = 0; i < n; ++i)
				B[static_cast<size_t>(i) * n + i] = 1.0;
			for (int sweep = 0; sweep < 12; ++sweep) {
				double off = 0.0;
				for (int i = 0; i < n; ++i)
					for (int j = i + 1; j < n; ++j)
						off += fabs(A[static_cast<size_t>(i) * n + j]);
				if (off < 1e-11)
					break;
				for (int i = 0; i < n; ++i) {
					for (int j = i + 1; j < n; ++j) {
						const double aij = A[static_cast<size_t>(i) * n + j];
						if (fabs(aij) < 1e-13)
							continue;
						const double aii = A[static_cast<size_t>(i) * n + i];
						const double ajj = A[static_cast<size_t>(j) * n + j];
						const double tau = (ajj - aii) / (2.0 * aij);
						const double sgn = tau >= 0.0 ? 1.0 : -1.0;
						const double tt = sgn
							/ (fabs(tau) + sqrt(1.0 + tau * tau));
						const double c = 1.0 / sqrt(1.0 + tt * tt);
						const double sn = tt * c;
						for (int k = 0; k < n; ++k) {
							const double aik = A[static_cast<size_t>(i) * n + k];
							const double ajk = A[static_cast<size_t>(j) * n + k];
							A[static_cast<size_t>(i) * n + k] =
								c * aik - sn * ajk;
							A[static_cast<size_t>(j) * n + k] =
								sn * aik + c * ajk;
						}
						for (int k = 0; k < n; ++k) {
							const double aki = A[static_cast<size_t>(k) * n + i];
							const double akj = A[static_cast<size_t>(k) * n + j];
							A[static_cast<size_t>(k) * n + i] =
								c * aki - sn * akj;
							A[static_cast<size_t>(k) * n + j] =
								sn * aki + c * akj;
						}
						for (int k = 0; k < n; ++k) {
							const double bki = B[static_cast<size_t>(k) * n + i];
							const double bkj = B[static_cast<size_t>(k) * n + j];
							B[static_cast<size_t>(k) * n + i] =
								c * bki - sn * bkj;
							B[static_cast<size_t>(k) * n + j] =
								sn * bki + c * bkj;
						}
					}
				}
			}
			D.assign(n, 0.0);
			for (int i = 0; i < n; ++i) {
				double d = A[static_cast<size_t>(i) * n + i];
				if (d < 1e-14)
					d = 1e-14;
				D[i] = sqrt(d);
			}
		}

		// Parallel population evaluator: persistent workers, generation
		// barrier. Evaluations are pure, so results are deterministic
		// regardless of worker count or scheduling.
		struct EvalPool {
			std::mutex mx;
			std::condition_variable cv_work, cv_done;
			std::vector<std::thread> ths;
			const std::vector<std::vector<double>>* xs = nullptr;
			std::vector<double>* fs = nullptr;
			std::vector<SmoothStats>* sts = nullptr;
			std::atomic<int> next{ 0 };
			int pending = 0;
			unsigned gen = 0;
			bool quit = false;
			const SmoothOpt* opt = nullptr;

			void Start(int nthreads) {
				for (int i = 0; i < nthreads; ++i)
					ths.emplace_back([this]() { Worker(); });
			}
			void Worker() {
				unsigned my_gen = 0;
				for (;;) {
					{
						std::unique_lock<std::mutex> lk(mx);
						cv_work.wait(lk, [&]() {
							return quit || gen != my_gen; });
						if (quit)
							return;
						my_gen = gen;
					}
					for (;;) {
						const int i = next.fetch_add(1);
						if (i >= static_cast<int>(xs->size()))
							break;
						(*fs)[i] = opt->Evaluate((*xs)[i], &(*sts)[i],
							nullptr);
						{
							std::lock_guard<std::mutex> lk(mx);
							if (--pending == 0)
								cv_done.notify_all();
						}
					}
				}
			}
			void Run(const std::vector<std::vector<double>>& X,
			         std::vector<double>& F, std::vector<SmoothStats>& S) {
				{
					std::lock_guard<std::mutex> lk(mx);
					xs = &X;
					fs = &F;
					sts = &S;
					next.store(0);
					pending = static_cast<int>(X.size());
					gen++;
				}
				cv_work.notify_all();
				std::unique_lock<std::mutex> lk(mx);
				cv_done.wait(lk, [&]() { return pending == 0; });
			}
			void Stop() {
				{
					std::lock_guard<std::mutex> lk(mx);
					quit = true;
				}
				cv_work.notify_all();
				for (auto& t : ths)
					t.join();
			}
		};

	} // namespace

	SmoothResult SmoothOpt::Run() {
		const int n = Dims();
		const int lambda = cfg_.pop > 0 ? cfg_.pop
			: (64 > 4 + static_cast<int>(3.0 * log(static_cast<double>(n)))
				? 64 : 4 + static_cast<int>(3.0 * log(static_cast<double>(n))));
		const int mu = lambda / 2;

		std::vector<double> wts(mu);
		double wsum = 0.0;
		for (int i = 0; i < mu; ++i) {
			wts[i] = log(mu + 0.5) - log(i + 1.0);
			wsum += wts[i];
		}
		double w2 = 0.0;
		for (int i = 0; i < mu; ++i) {
			wts[i] /= wsum;
			w2 += wts[i] * wts[i];
		}
		const double mueff = 1.0 / w2;
		const double cs = (mueff + 2.0) / (n + mueff + 5.0);
		const double ds = 1.0 + cs + 2.0 * fmax(0.0,
			sqrt((mueff - 1.0) / (n + 1.0)) - 1.0);
		const double cc = (4.0 + mueff / n) / (n + 4.0 + 2.0 * mueff / n);
		const double c1 = 2.0 / ((n + 1.3) * (n + 1.3) + mueff);
		const double cmu = fmin(1.0 - c1, 2.0 * (mueff - 2.0 + 1.0 / mueff)
			/ ((n + 2.0) * (n + 2.0) + mueff));
		const double chiN = sqrt(static_cast<double>(n))
			* (1.0 - 1.0 / (4.0 * n) + 1.0 / (21.0 * n * n));

		const int nthreads = ResolveThreadCount(cfg_.threads);
		if (!quiet_) {
			printf("smooth: %d dims (%d CPs @ %d ticks + jump), pop %d, "
				"%d workers, %s init, budget %.0fs (deterministic, rng %u)\n",
				n, ncp_, cfg_.cp_ticks, lambda, nthreads,
				have_seed_mean_ ? "SEEDED" : "cold", cfg_.budget_seconds,
				cfg_.rng_seed);
			fflush(stdout);
		}

		SmoothResult res;
		std::mt19937_64 rng(cfg_.rng_seed);
		std::normal_distribution<double> gauss(0.0, 1.0);

		EvalPool pool;
		pool.opt = this;
		pool.Start(nthreads);

		std::vector<double> mean(n, 0.0), pc(n, 0.0), ps(n, 0.0);
		std::vector<double> C(static_cast<size_t>(n) * n, 0.0);
		std::vector<double> B, D;
		double sigma = have_seed_mean_ ? cfg_.sigma_seed : cfg_.sigma0;

		auto reinit = [&](bool first) {
			for (int i = 0; i < n; ++i)
				for (int j = 0; j < n; ++j)
					C[static_cast<size_t>(i) * n + j] = i == j ? 1.0 : 0.0;
			JacobiEig(n, C, B, D);
			std::fill(pc.begin(), pc.end(), 0.0);
			std::fill(ps.begin(), ps.end(), 0.0);
			if (have_seed_mean_) {
				mean = seed_mean_;
				if (static_cast<int>(mean.size()) < n)
					mean.resize(n, 0.5);
				// Escalating restart step: the same basin keeps catching
				// gentle re-inits; each restart doubles the reach (cap 8x).
				const double esc = first ? 1.0
					: fmin(8.0, pow(2.0, 1.0 + res.restarts % 4));
				sigma = cfg_.sigma_seed * esc;
			} else {
				for (int i = 0; i < ncp_; ++i)
					mean[i] = 0.3 * gauss(rng);
				mean[ncp_] = 1.0 + 4.0
					* (static_cast<double>(rng() & 0xFFFF) / 65535.0);
				mean[ncp_ + 1] = -0.5;   // duck genes start disabled
				mean[ncp_ + 2] = -0.5;
				if (n > ncp_ + 3) {
					// Guidance genes: center of the face, mostly guided,
					// homing from ~a quarter second in (release timing is
					// CMA's to move).
					mean[ncp_ + 3] = 0.3 + 0.4
						* (static_cast<double>(rng() & 0xFF) / 255.0);
					mean[ncp_ + 4] = 0.3 + 0.4
						* (static_cast<double>(rng() & 0xFF) / 255.0);
					mean[ncp_ + 5] = 0.7;
					// Release window: rides can need 20-70 ticks before
					// the exit (a low board must climb first).
					mean[ncp_ + 6] = 2.0
						+ 5.0 * (static_cast<double>(rng() & 0xFF) / 255.0);
					if (n > ncp_ + 7)
						mean[ncp_ + 7] = 0.0;   // carve: neutral hold
				}
				sigma = cfg_.sigma0;
			}
			if (!first)
				res.restarts++;
		};
		reinit(true);

		std::vector<std::vector<double>> X(lambda,
			std::vector<double>(n));
		std::vector<std::vector<double>> Z(lambda,
			std::vector<double>(n));
		std::vector<double> F(lambda);
		std::vector<SmoothStats> S(lambda);
		std::vector<int> order(lambda);

		double best_f = 1e30;
		double restart_best = 1e30;
		int restart_stall = 0;
		int gens_since_eig = 0;
		int gen_in_restart = 0;

		const auto t0 = std::chrono::steady_clock::now();
		auto last_beat = t0;
		int gen = 0;

		for (;;) {
			const double el = std::chrono::duration<double>(
				std::chrono::steady_clock::now() - t0).count();
			if (el >= cfg_.budget_seconds)
				break;

			// Sample x = mean + sigma * B * (D o z).
			for (int k = 0; k < lambda; ++k) {
				for (int i = 0; i < n; ++i)
					Z[k][i] = gauss(rng);
				for (int i = 0; i < n; ++i) {
					double y = 0.0;
					for (int j = 0; j < n; ++j)
						y += B[static_cast<size_t>(i) * n + j] * D[j]
							* Z[k][j];
					X[k][i] = mean[i] + sigma * y;
				}
			}
			pool.Run(X, F, S);
			res.evals += lambda;
			for (int k = 0; k < lambda; ++k)
				res.ticks_simulated += S[k].sim_ticks;

			for (int i = 0; i < lambda; ++i)
				order[i] = i;
			std::sort(order.begin(), order.end(),
				[&](int a, int b) { return F[a] < F[b]; });

			const int bi = order[0];
			if (F[bi] < best_f) {
				best_f = F[bi];
				res.best_x = X[bi];
				res.best_stats = S[bi];
				if (S[bi].finished || S[bi].touched) {
					// Announce only CLASS or TICK improvements (the eloss
					// tie-break otherwise spams equal-rel "NEW BEST" lines).
					const bool worth = !res.ok
						|| (S[bi].clean && !res.clean)
						|| (S[bi].clean == res.clean
							&& S[bi].rel < res.best_rel);
					res.ok = true;
					res.clean = S[bi].clean;
					res.best_tick = S[bi].tick;
					res.best_rel = S[bi].rel;
					if (worth && !quiet_) {
						printf("[smooth] gen %5d  NEW BEST  %s %d scored "
							"(%d abs)  eloss %.0fk  impact %.0f\n",
							gen, S[bi].clean ? "[clean]" : "[JUMP ]",
							S[bi].rel, S[bi].tick, S[bi].eloss / 1000.f,
							S[bi].max_impact);
						fflush(stdout);
					}
				}
			}
			if (F[bi] < restart_best - 1e-9) {
				restart_best = F[bi];
				restart_stall = 0;
			} else {
				restart_stall++;
			}

			// Recombination.
			std::vector<double> old_mean = mean;
			std::vector<double> zw(n, 0.0), yw(n, 0.0);
			for (int i = 0; i < n; ++i) {
				double m = 0.0, zz = 0.0;
				for (int k = 0; k < mu; ++k) {
					m += wts[k] * X[order[k]][i];
					zz += wts[k] * Z[order[k]][i];
				}
				mean[i] = m;
				zw[i] = zz;
				yw[i] = (m - old_mean[i]) / sigma;
			}
			// ps = (1-cs) ps + sqrt(cs(2-cs)mueff) * B*zw
			double psn2 = 0.0;
			for (int i = 0; i < n; ++i) {
				double bz = 0.0;
				for (int j = 0; j < n; ++j)
					bz += B[static_cast<size_t>(i) * n + j] * zw[j];
				ps[i] = (1.0 - cs) * ps[i]
					+ sqrt(cs * (2.0 - cs) * mueff) * bz;
				psn2 += ps[i] * ps[i];
			}
			const double psn = sqrt(psn2);
			gen_in_restart++;
			const bool hs = psn
				/ sqrt(1.0 - pow(1.0 - cs, 2.0 * gen_in_restart))
				< (1.4 + 2.0 / (n + 1.0)) * chiN;
			for (int i = 0; i < n; ++i)
				pc[i] = (1.0 - cc) * pc[i] + (hs
					? sqrt(cc * (2.0 - cc) * mueff) * yw[i] : 0.0);
			// Covariance update. When hs failed, the pc shrink is repaid on
			// the DECAY term (Hansen: + c1*(1-hs)*cc*(2-cc)*C).
			const double cdecay = 1.0 - c1 - cmu
				+ (hs ? 0.0 : c1 * cc * (2.0 - cc));
			// Pre-scale the selected steps once: O(mu*n) instead of per-cell.
			std::vector<std::vector<double>> Y(mu, std::vector<double>(n));
			for (int k = 0; k < mu; ++k)
				for (int i = 0; i < n; ++i)
					Y[k][i] = (X[order[k]][i] - old_mean[i]) / sigma;
			for (int i = 0; i < n; ++i) {
				for (int j = i; j < n; ++j) {
					double v = cdecay * C[static_cast<size_t>(i) * n + j]
						+ c1 * pc[i] * pc[j];
					double rk = 0.0;
					for (int k = 0; k < mu; ++k)
						rk += wts[k] * Y[k][i] * Y[k][j];
					v += cmu * rk;
					C[static_cast<size_t>(i) * n + j] = v;
					C[static_cast<size_t>(j) * n + i] = v;
				}
			}
			sigma *= exp((cs / ds) * (psn / chiN - 1.0));
			if (sigma > 5.0)
				sigma = 5.0;

			if (++gens_since_eig >= 20) {
				JacobiEig(n, C, B, D);
				gens_since_eig = 0;
			}
			gen++;
			res.generations = gen;

			double dmax = 0.0;
			for (int i = 0; i < n; ++i)
				dmax = fmax(dmax, D[i]);
			if (restart_stall >= cfg_.stall_gens || sigma * dmax < 1e-4) {
				// Early return only once SUCCESSFUL and explored - a
				// segment that hasn't found its target yet keeps hunting
				// for its whole budget (measured: failure-stalls burned 3
				// restarts in ~1s and handed back 8s budgets unused).
				if (cfg_.max_restarts > 0 && res.ok
					&& res.restarts + 1 >= cfg_.max_restarts)
					break;
				reinit(false);
				restart_best = 1e30;
				restart_stall = 0;
				gens_since_eig = 0;
				gen_in_restart = 0;
			}

			const auto now = std::chrono::steady_clock::now();
			if (!quiet_ && std::chrono::duration<double>(now - last_beat)
				.count() >= 5.0) {
				last_beat = now;
				char bb[128];
				if (res.ok)
					snprintf(bb, sizeof(bb), "%s %d scored",
						res.clean ? "[clean]" : "[JUMP ]", res.best_rel);
				else
					snprintf(bb, sizeof(bb), "no finish yet (dmin %.0f)",
						res.best_stats.dmin);
				printf("[smooth] %5.0fs  gen %6d  %6.2fM evals  best %s  "
					"sigma %.3f  restarts %d\n", el, gen,
					res.evals / 1e6, bb, sigma, res.restarts);
				fflush(stdout);
			}
		}

		pool.Stop();
		res.seconds = std::chrono::duration<double>(
			std::chrono::steady_clock::now() - t0).count();
		return res;
	}

} // namespace Solver
