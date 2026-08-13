#include "SolverOptimize.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <utility>

namespace Solver {

	Optimizer::Optimizer(const World& w, const OptimizeConfig& cfg,
	                     const PlayerState& root, float root_yaw,
	                     const Tape* seed_tape)
		: w_(w), cfg_(cfg), root_(root), root_yaw_(root_yaw), seed_(seed_tape) {}

	bool Optimizer::InsideStartZone(const Vec3& p) const {
		if (cfg_.start_brush_idx < 0)
			return false;
		const WorldBrush& b = w_.brushes[cfg_.start_brush_idx];
		return p.X >= b.gmin_stand.X && p.X <= b.gmax_stand.X
			&& p.Y >= b.gmin_stand.Y && p.Y <= b.gmax_stand.Y
			&& p.Z >= b.bmax.Z - 1.f;
	}

	// Flip state carried into the knot list after `prefix` seed frames - the
	// same bookkeeping the explorer's seeding used (prefix erosion must
	// recompute it or FlipsLegal judges against a stale baseline).
	void Optimizer::SeedFlipState(int prefix, signed char* side,
	                              short* since) const {
		signed char last_side = 0;
		int since_flip = 999;
		if (seed_) {
			for (int t = 0; t < prefix
				&& t < static_cast<int>(seed_->frames.size()); ++t) {
				const TapeFrame& f = seed_->frames[t];
				signed char s = 0;
				if (f.buttons & IN_MOVELEFT) s = 1;
				else if (f.buttons & IN_MOVERIGHT) s = -1;
				if (s != 0 && last_side != 0 && s != last_side)
					since_flip = 0;
				if (s != 0)
					last_side = s;
				if (since_flip < 999)
					since_flip++;
			}
		}
		*side = last_side;
		*since = static_cast<short>(since_flip);
	}

	namespace {
		// Fixed-size "top energy drops" accumulator (threshold-free: the
		// route's own worst ticks define what gets aimed at).
		void NoteLoss(std::vector<std::pair<float, int>>& top, float drop,
		              int knot) {
			if (top.size() < 8) {
				top.emplace_back(drop, knot);
				return;
			}
			size_t mi = 0;
			for (size_t i = 1; i < top.size(); ++i)
				if (top[i].first < top[mi].first)
					mi = i;
			if (drop > top[mi].first)
				top[mi] = std::make_pair(drop, knot);
		}
	}

	Optimizer::Eval Optimizer::Run(const Explorer::FlatGenome& g, int abort_at,
	                               std::vector<TapeFrame>* emit,
	                               long long* ticks,
	                               std::vector<std::pair<float, int>>* loss_top) {
		Eval res;
		PlayerState s = root_;
		float yaw = root_yaw_;
		int tick = 0;
		int zone_jumps = 0;
		// Zone clock: -1 while inside the startzone; scored ticks = tick -
		// exit. Anchors outside any zone start the clock immediately.
		int exit_tick = InsideStartZone(s.pos) ? -1 : 0;
		auto scored = [&]() {
			if (!cfg_.zone_clock)
				return tick;
			return exit_tick >= 0 ? tick - exit_tick : 0;
		};
		float e_prev = 0.5f * Len2(s.vel) + cfg_.params.gravity * s.pos.Z;
		if (loss_top)
			loss_top->clear();
		if (emit)
			emit->clear();

		auto check = [&](const TickEvents& ev) -> int {
			// -1 abort, 0 continue, 1 finished
			if (ev.jumped && InsideStartZone(s.pos)) {
				if (++zone_jumps > cfg_.max_zone_jumps)
					return -1;
			}
			if (cfg_.goal_touch) {
				if (s.on_ground && s.ground_brush >= 0
					&& w_.brushes[s.ground_brush].id == cfg_.end_brush_id)
					return 1;
				for (int c = 0; c < ev.ncontacts; ++c)
					if (w_.brushes[ev.contact_brush[c]].id
						== cfg_.end_brush_id)
						return 1;
			} else if (cfg_.end_brush_idx >= 0 && s.on_ground
				&& s.ground_brush >= 0
				&& w_.brushes[s.ground_brush].id == cfg_.end_brush_id) {
				const WorldBrush& b = w_.brushes[cfg_.end_brush_idx];
				if (s.pos.X >= b.gmin_stand.X && s.pos.X <= b.gmax_stand.X
					&& s.pos.Y >= b.gmin_stand.Y && s.pos.Y <= b.gmax_stand.Y)
					return 1;
			}
			return 0;
		};

		// Seed prefix (fixed tape frames).
		if (g.seed_prefix > 0 && seed_) {
			const int n = g.seed_prefix
				< static_cast<int>(seed_->frames.size())
				? g.seed_prefix : static_cast<int>(seed_->frames.size());
			for (int t = 0; t < n; ++t) {
				const TapeFrame& f = seed_->frames[t];
				TickEvents ev;
				MoveTick(s, w_, cfg_.params, f.pitch, f.yaw, f.fmove, f.smove,
				         f.umove, f.buttons, &ev);
				tick++;
				if (ticks) (*ticks)++;
				if (emit)
					emit->push_back(f);
				if (loss_top) {
					const float e_now = 0.5f * Len2(s.vel)
						+ cfg_.params.gravity * s.pos.Z;
					if (e_now < e_prev)
						NoteLoss(*loss_top, e_prev - e_now, -1);
					e_prev = e_now;
				}
				if (exit_tick < 0 && !InsideStartZone(s.pos))
					exit_tick = tick;
				yaw = f.yaw;
				const int c = check(ev);
				if (c != 0 || scored() >= abort_at) {
					res.finished = (c == 1);
					res.tick = tick;
					res.rel = scored();
					return res;
				}
			}
		}

		// Knots.
		for (size_t ki = 0; ki < g.knots.size(); ++ki) {
			const Knot& k = g.knots[ki];
			for (int t = 0; t < k.dur; ++t) {
				TickEvents ev;
				TapeFrame f;
				KnotTick(s, yaw, k, t, w_, cfg_.params, &ev,
				         emit ? &f : nullptr);
				tick++;
				if (ticks) (*ticks)++;
				if (emit)
					emit->push_back(f);
				if (loss_top) {
					const float e_now = 0.5f * Len2(s.vel)
						+ cfg_.params.gravity * s.pos.Z;
					if (e_now < e_prev)
						NoteLoss(*loss_top, e_prev - e_now,
							static_cast<int>(ki));
					e_prev = e_now;
				}
				if (exit_tick < 0 && !InsideStartZone(s.pos))
					exit_tick = tick;
				const int c = check(ev);
				if (c == 1) {
					res.finished = true;
					res.tick = tick;
					res.rel = scored();
					return res;
				}
				if (c == -1 || scored() >= abort_at) {
					res.tick = tick;
					res.rel = scored();
					return res;
				}
			}
		}
		res.tick = tick;
		res.rel = scored();
		return res;
	}

	bool Optimizer::BuildFrames(const Explorer::FlatGenome& g,
	                            std::vector<TapeFrame>& out, int* finish_tick,
	                            int* finish_rel) {
		long long tk = 0;
		const Eval e = Run(g, cfg_.max_path_ticks, &out, &tk);
		if (!e.finished)
			return false;
		out.resize(e.tick);   // truncate AT the finish
		if (finish_tick)
			*finish_tick = e.tick;
		if (finish_rel)
			*finish_rel = e.rel;
		return true;
	}

	// Instrumented pass over the current best: which knots were active at the
	// route's worst one-tick energy drops. Those (and their approach knots)
	// become mutation targets - the measured "the waste is at board entries"
	// theme turned into aim, not fitness.
	void Optimizer::RefreshAim(const Explorer::FlatGenome& g) {
		aim_.clear();
		if (!cfg_.aim_contacts)
			return;
		std::vector<std::pair<float, int>> top;
		long long dummy = 0;
		Run(g, cfg_.max_path_ticks, nullptr, &dummy, &top);
		for (const auto& lk : top) {
			const int k = lk.second;
			if (k < 0)
				continue;   // seed-prefix tick: erosion reaches it, aim can't
			for (int t = k - 1; t <= k; ++t) {
				if (t < 0)
					continue;
				bool have = false;
				for (int a : aim_)
					if (a == t) { have = true; break; }
				if (!have)
					aim_.push_back(t);
			}
		}
	}

	OptimizeResult Optimizer::Improve(Explorer::FlatGenome& g) {
		OptimizeResult res;
		const auto t0 = std::chrono::steady_clock::now();
		auto elapsed = [&]() {
			return std::chrono::duration<double>(
				std::chrono::steady_clock::now() - t0).count();
		};
		std::mt19937 rng(cfg_.rng_seed);

		const Eval e0 = Run(g, cfg_.max_path_ticks, nullptr,
		                    &res.ticks_simulated);
		if (!e0.finished) {
			res.seconds = elapsed();
			return res;
		}
		res.ok = true;
		// All scores are SCORED ticks (since zone exit under the zone clock;
		// Run reports rel == tick when the clock is off).
		res.initial_tick = e0.rel;
		int best_tick = e0.rel;
		RefreshAim(g);

		// Half of the knot-picking mutations aim at the measured loss knots
		// (when aiming is on and targets exist); the rest stay uniform so
		// nothing is ever unreachable.
		auto pick_knot = [&](int nk) -> int {
			if (!aim_.empty() && static_cast<int>(rng() % 100) < 50) {
				const int i = aim_[rng() % aim_.size()];
				if (i < nk)
					return i;
			}
			return static_cast<int>(rng() % nk);
		};

		while (elapsed() < cfg_.budget_seconds) {
			Explorer::FlatGenome cand = g;
			const int op = static_cast<int>(rng() % 100);
			const int nk = static_cast<int>(cand.knots.size());
			if (op < 27 && nk > 0) {
				// Trim a knot (a knot shrinking to nothing is deleted; flip
				// legality is judged afterward like every mutation).
				const int i = pick_knot(nk);
				cand.knots[i].dur = static_cast<short>(cand.knots[i].dur
					- (1 + static_cast<int>(rng() % 8)));
				if (cand.knots[i].dur < 1)
					cand.knots.erase(cand.knots.begin() + i);
			} else if (op < 35 && nk > 1) {
				// Boundary shift: move 1-2 ticks between adjacent knots - a
				// pure TIMING adjustment (total length preserved, downstream
				// knots unmoved). This is the tangent-boarding micro-move the
				// themes data named: the entry angle is set by when the
				// approach knot hands over, and trim/grow can only express
				// that by displacing the whole rest of the route.
				int i = pick_knot(nk);
				if (i < 1)
					i = 1;
				const int d = 1 + static_cast<int>(rng() % 2);
				Knot& ka = cand.knots[i - 1];
				Knot& kb = cand.knots[i];
				if (rng() & 1) {
					if (ka.dur <= d)
						continue;
					ka.dur = static_cast<short>(ka.dur - d);
					kb.dur = static_cast<short>(kb.dur + d);
				} else {
					if (kb.dur <= d)
						continue;
					kb.dur = static_cast<short>(kb.dur - d);
					ka.dur = static_cast<short>(ka.dur + d);
				}
			} else if (op < 50 && nk > 1) {
				cand.knots.erase(cand.knots.begin() + pick_knot(nk));
			} else if (op < 72) {
				// Resample the suffix with the explorer's own distribution.
				// Aimed: resample FROM a loss knot - re-roll the approach and
				// everything after it.
				int i = static_cast<int>(rng() % (nk + 1));
				if (!aim_.empty() && static_cast<int>(rng() % 100) < 50) {
					const int a = aim_[rng() % aim_.size()];
					if (a <= nk)
						i = a;
				}
				signed char ls = cand.base_side;
				short sf = cand.base_since_flip;
				for (int j = 0; j < i; ++j) {
					const Knot& k = cand.knots[j];
					if (k.side != 0 && ls != 0 && k.side != ls)
						sf = 0;
					if (k.side != 0)
						ls = k.side;
					sf = static_cast<short>(
						sf + k.dur > 999 ? 999 : sf + k.dur);
				}
				cand.knots.resize(i);
				const int add = 1 + static_cast<int>(rng() % 3);
				for (int a = 0; a < add; ++a) {
					Knot kn = SampleKnot(rng, ls, sf, cfg_.min_knot);
					if (kn.side != 0 && ls != 0 && kn.side != ls)
						sf = 0;
					if (kn.side != 0)
						ls = kn.side;
					sf = static_cast<short>(
						sf + kn.dur > 999 ? 999 : sf + kn.dur);
					cand.knots.push_back(kn);
				}
			} else if (op < 85 && nk > 0) {
				// Parameter jitter.
				Knot& k = cand.knots[pick_knot(nk)];
				switch (rng() % 4) {
				case 0:
					k.ground_turn += (static_cast<int>(rng() % 200) - 100)
						* 0.01f;
					if (k.ground_turn < 0.2f) k.ground_turn = 0.2f;
					if (k.ground_turn > 4.f) k.ground_turn = 4.f;
					break;
				case 1: k.flags ^= 2; break;   // duck held
				case 2: k.flags ^= 1; break;   // jump press
				default:
					k.dur = static_cast<short>(k.dur + 1
						+ static_cast<int>(rng() % 4));
					break;
				}
			} else if (op < 93 && nk > 0) {
				Knot& k = cand.knots[pick_knot(nk)];
				k.dur = static_cast<short>(k.dur + 1
					+ static_cast<int>(rng() % 6));
			} else if (cand.seed_prefix > 0) {
				// Erode the seed prefix: branch earlier into the human tape.
				cand.seed_prefix -= 1 + static_cast<int>(rng() % 40);
				if (cand.seed_prefix < 0)
					cand.seed_prefix = 0;
				SeedFlipState(cand.seed_prefix, &cand.base_side,
				              &cand.base_since_flip);
			} else if (nk > 0) {
				Knot& k = cand.knots[static_cast<int>(rng() % nk)];
				if (k.dur > 1)
					k.dur = static_cast<short>(k.dur - 1);
			} else {
				continue;
			}

			if (!FlipsLegal(cand.knots, cfg_.min_knot, cand.base_side,
				cand.base_since_flip))
				continue;
			const Eval e = Run(cand, best_tick, nullptr, &res.ticks_simulated);
			res.evals++;
			if (e.finished && e.rel < best_tick) {
				g = cand;
				best_tick = e.rel;
				res.improvements++;
				RefreshAim(g);   // knot indices shift after every accept
			}
		}
		res.best_tick = best_tick;
		res.seconds = elapsed();
		return res;
	}

} // namespace Solver
