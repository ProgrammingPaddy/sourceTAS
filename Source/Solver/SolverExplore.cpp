#include "SolverExplore.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <random>
#include <unordered_map>

namespace Solver {

	namespace {
		using CellHash = std::unordered_map<unsigned long long, int>;
	}

	#define CMAP() (*static_cast<CellHash*>(cellmap_))

	Explorer::Explorer(const World& w, const ExploreConfig& cfg)
		: w_(w), cfg_(cfg) {
		cellmap_ = new CellHash();
		// flips/sec -> minimum ticks between L/R flips, enforced structurally.
		const float tps = 1.f / cfg_.params.dt;
		min_knot_ = static_cast<int>(tps / cfg_.flips_per_sec) + 1;
		if (min_knot_ < 2) min_knot_ = 2;
		start_idx_ = (cfg_.start_brush_id >= 0)
			? w_.IndexOfBrushId(cfg_.start_brush_id) : -1;
		end_idx_ = (cfg_.end_brush_id >= 0)
			? w_.IndexOfBrushId(cfg_.end_brush_id) : -1;
		if (end_idx_ >= 0) {
			const WorldBrush& b = w_.brushes[end_idx_];
			end_center_ = Vec3((b.bmin.X + b.bmax.X) * 0.5f,
			                   (b.bmin.Y + b.bmax.Y) * 0.5f, b.bmax.Z);
		}
	}

	Explorer::~Explorer() {
		delete static_cast<CellHash*>(cellmap_);
	}

	void Explorer::SetAnchor(const TapeAnchor& a) {
		entries_.clear();
		CMAP().clear();
		for (int b = 0; b < 32; ++b)
			bands_[b].clear();
		for (int b = 0; b < 16; ++b)
			sbands_[b].clear();
		min_dist_seen_ = 1e9f;
		max_speed_seen_ = 0.f;
		min_dist_entry_ = -1;
		air_entries_ = 0;
		near_ring_.clear();
		Entry e;
		e.st.pos = a.origin;
		e.st.vel = a.velocity;
		e.st.ducked = a.ducked;
		e.st.stamina = a.stamina;
		// Establish ground exactly like the replay path does.
		TraceResult tr;
		const float gf = w_.TraceHull(e.st.pos, e.st.pos - Vec3(0.f, 0.f, 2.f),
		                              e.st.ducked, &tr);
		if (gf < 1.f && tr.brush >= 0 && tr.normal.Z >= cfg_.params.walkable_z) {
			e.st.on_ground = true;
			e.st.ground_brush = tr.brush;
		}
		e.yaw = a.yaw;
		e.tick = 0;
		e.parent = -1;
		e.seed_ticks = -1;
		e.ticks_since_flip = 999;
		entries_.push_back(e);
		int band = static_cast<int>(Len(e.st.pos - end_center_) / 256.f);
		if (band > 31) band = 31;
		if (band < 0) band = 0;
		bands_[band].push_back(0);
	}

	void Explorer::RootState(PlayerState* s, float* yaw) const {
		if (entries_.empty())
			return;
		if (s) *s = entries_[0].st;
		if (yaw) *yaw = entries_[0].yaw;
	}

	bool Explorer::ExtractGenome(int entry_index, FlatGenome& out) const {
		out = FlatGenome();
		if (entry_index < 0 || entry_index >= static_cast<int>(entries_.size()))
			return false;
		std::vector<int> chain;
		for (int i = entry_index; i >= 0; i = entries_[i].parent)
			chain.push_back(i);
		std::reverse(chain.begin(), chain.end());
		size_t ci = 0;
		if (chain[0] != 0) {
			const Entry& basee = entries_[chain[0]];
			if (basee.seed_ticks < 0)
				return false;   // orphan chain
			out.seed_prefix = basee.seed_ticks;
			out.base_side = basee.last_side;
			out.base_since_flip = basee.ticks_since_flip;
			ci = 1;
		} else {
			ci = 1;   // root itself carries no knots
		}
		for (; ci < chain.size(); ++ci) {
			const Entry& e = entries_[chain[ci]];
			for (int ki = 0; ki < e.nknots; ++ki)
				out.knots.push_back(e.knots[ki]);
		}
		return true;
	}

	bool Explorer::InsideZoneXY(const Vec3& p, int brush_idx) const {
		if (brush_idx < 0)
			return false;
		const WorldBrush& b = w_.brushes[brush_idx];
		return p.X >= b.gmin_stand.X && p.X <= b.gmax_stand.X
			&& p.Y >= b.gmin_stand.Y && p.Y <= b.gmax_stand.Y;
	}

	// The startzone per the user's definition: the platform footprint extended
	// UP into the sky - "if the starting platform is NOT beneath the player,
	// the time has started". Being UNDER the platform (floor bounces) is not
	// in the zone.
	bool Explorer::InsideStartZone(const Vec3& p) const {
		return start_idx_ >= 0 && InsideZoneXY(p, start_idx_)
			&& p.Z >= w_.brushes[start_idx_].bmax.Z - 1.f;
	}

	bool Explorer::IsFinish(const PlayerState& s) const {
		return end_idx_ >= 0 && s.on_ground && s.ground_brush >= 0
			&& w_.brushes[s.ground_brush].id == cfg_.end_brush_id
			&& InsideZoneXY(s.pos, end_idx_);
	}

	bool Explorer::GoalReached(const PlayerState& s, const TickEvents& ev) const {
		if (!cfg_.goal_touch)
			return IsFinish(s);
		// Segment-experiment goal: ANY contact with the end brush counts.
		if (s.on_ground && s.ground_brush >= 0
			&& w_.brushes[s.ground_brush].id == cfg_.end_brush_id)
			return true;
		for (int c = 0; c < ev.ncontacts; ++c)
			if (w_.brushes[ev.contact_brush[c]].id == cfg_.end_brush_id)
				return true;
		return false;
	}

	unsigned long long Explorer::CellKey(const PlayerState& s, int contact_kind,
	                                     int contact_brush) const {
		// Free-air states get 3x cells and 2x-coarse speed buckets: contact-
		// adjacent precision is what matters (event-boundary lesson);
		// fine-grained void cells were flooding the archive.
		const float inv = 1.f / (contact_kind == 0 ? cfg_.cell_size * 3.f
			: cfg_.cell_size);
		const unsigned long long qx =
			static_cast<unsigned long long>((s.pos.X + 16384.f) * inv) & 0x3FF;
		const unsigned long long qy =
			static_cast<unsigned long long>((s.pos.Y + 16384.f) * inv) & 0x3FF;
		const unsigned long long qz =
			static_cast<unsigned long long>((s.pos.Z + 16384.f) * inv) & 0x3FF;
		unsigned long long sb = static_cast<unsigned long long>(Len2D(s.vel)
			/ (contact_kind == 0 ? cfg_.speed_bucket * 2.f : cfg_.speed_bucket));
		if (sb > 63) sb = 63;
		const unsigned long long kd = s.ducked ? 1ULL : 0ULL;
		const unsigned long long kk = static_cast<unsigned long long>(contact_kind) & 3;
		const unsigned long long kb = static_cast<unsigned long long>(
			contact_brush < 0 ? 0 : contact_brush) & 0xFFF;
		return qx | (qy << 10) | (qz << 20) | (sb << 30) | (kd << 36)
			| (kk << 37) | (kb << 39);
	}

	int Explorer::RecordCell(const PlayerState& s, float yaw, int tick,
	                         int parent, int seed_ticks,
	                         const std::vector<Knot>& knots, int cur_knot,
	                         int ticks_into_knot, signed char last_side,
	                         short since_flip, short zone_jumps, float eloss,
	                         const TickEvents& ev) {
		int kind = 0, brush = -1;
		if (s.on_ground) {
			kind = 1;
			brush = s.ground_brush;
		} else if (ev.ncontacts > 0) {
			kind = 2;
			brush = ev.contact_brush[0];
		}
		if (kind == 0 && air_entries_ >= 1500000)
			return -1;   // air sub-cap: keep room for the contact pipeline
		const unsigned long long key = CellKey(s, kind, brush);
		CellHash& m = CMAP();
		auto it = m.find(key);
		// Hysteresis: replace only a meaningfully earlier arrival (tick-level
		// churn was flooding the archive in the first unseeded runs).
		if (it != m.end() && entries_[it->second].tick <= tick + 4)
			return -1;
		if (entries_.size() >= 4000000)
			return -1;   // global cap; Run() freezes the frontier when hit
		if (kind == 0)
			air_entries_++;
		Entry e;
		e.st = s;
		e.yaw = yaw;
		e.tick = tick;
		e.parent = parent;
		e.seed_ticks = seed_ticks;
		e.last_side = last_side;
		e.ticks_since_flip = since_flip;
		e.zone_jumps = zone_jumps;
		e.eloss = eloss;
		e.cbrush = static_cast<short>(brush);
		e.ckind = static_cast<signed char>(kind);
		if (cur_knot >= 0) {
			e.nknots = static_cast<unsigned char>(cur_knot + 1);
			for (int i = 0; i <= cur_knot && i < 3; ++i)
				e.knots[i] = knots[i];
			e.knots[cur_knot].dur = static_cast<short>(ticks_into_knot);
		}
		const int idx = static_cast<int>(entries_.size());
		entries_.push_back(e);
		if (it != m.end()) it->second = idx; else m[key] = idx;
		const float dist = Len(s.pos - end_center_);
		if (dist < min_dist_seen_) {
			min_dist_seen_ = dist;
			min_dist_entry_ = idx;
		}
		if (dist < min_dist_seen_ + 350.f) {
			if (near_ring_.size() < 512)
				near_ring_.push_back(idx);
			else
				near_ring_[idx % 512] = idx;
		}
		int band = static_cast<int>(dist / 256.f);
		if (band > 31) band = 31;
		// The finish-distance pull walks CONTACT cells only: junk falls that
		// drift near the finish were soaking the band walk (measured: closest
		// entry was a dead vertical fall beside the platform). Air cells stay
		// reachable via the speed/energy/uniform axes.
		if (kind != 0)
			bands_[band].push_back(idx);
		const float sp2 = Len2D(s.vel);
		if (sp2 > max_speed_seen_) max_speed_seen_ = sp2;
		int sband = static_cast<int>(sp2 / 100.f);
		if (sband > 15) sband = 15;
		sbands_[sband].push_back(idx);
		return idx;
	}

	int Explorer::SeedFromTape(const Tape& tape) {
		if (entries_.empty() || !tape.start.valid)
			return 0;
		seed_tape_ = &tape;
		PlayerState s = entries_[0].st;
		signed char last_side = 0;
		short since_flip = 999;
		short zone_jumps = 0;
		float eloss = 0.f;
		float e_prev = 0.5f * Len2(s.vel) + cfg_.params.gravity * s.pos.Z;
		int seeded = 0;
		std::vector<Knot> none;
		for (int t = 0; t < static_cast<int>(tape.frames.size()); ++t) {
			const TapeFrame& f = tape.frames[t];
			TickEvents ev;
			MoveTick(s, w_, cfg_.params, f.pitch, f.yaw, f.fmove, f.smove,
			         f.umove, f.buttons, &ev);
			const float e_now = 0.5f * Len2(s.vel)
				+ cfg_.params.gravity * s.pos.Z;
			if (e_now < e_prev)
				eloss += e_prev - e_now;
			e_prev = e_now;
			signed char side = 0;
			if (f.buttons & IN_MOVELEFT) side = 1;
			else if (f.buttons & IN_MOVERIGHT) side = -1;
			if (side != 0 && last_side != 0 && side != last_side)
				since_flip = 0;
			if (side != 0) last_side = side;
			if (since_flip < 999) since_flip++;
			if (ev.jumped && InsideStartZone(s.pos))
				zone_jumps++;
			RecordCell(s, f.yaw, t + 1, -1, t + 1, none, -1, 0,
			           last_side, since_flip, zone_jumps, eloss, ev);
			seeded++;
			if (GoalReached(s, ev)) {
				seed_finish_tick_ = t + 1;
				break;   // restart points past the finish are useless
			}
		}
		return seeded;
	}

	ExploreResult Explorer::Run() {
		ExploreResult res;
		if (entries_.empty() || end_idx_ < 0)
			return res;
		std::mt19937 rng(cfg_.rng_seed);
		const auto t0 = std::chrono::steady_clock::now();
		auto elapsed = [&]() {
			return std::chrono::duration<double>(
				std::chrono::steady_clock::now() - t0).count();
		};
		double next_print = 1.0;
		std::vector<Knot> knots;
		std::vector<Finisher> finishers;
		bool cap_hit = false;

		while (elapsed() < cfg_.budget_seconds
			&& (cfg_.max_rollouts == 0 || res.rollouts < cfg_.max_rollouts)) {
			// Round-robin selection (validated prior-attempt lesson): rotate
			// four pulls - toward the finish, along the speed pipeline, up
			// the energy ladder, and uniform - so no single score collapses
			// the frontier.
			int base = -1;
			const int mode = static_cast<int>(res.rollouts % 5);
			if (mode == 4 && !near_ring_.empty()) {
				// Breed near-misses: back up 1-2 knot-chain links from a
				// close approach and resample the ending.
				int i = near_ring_[rng() % near_ring_.size()];
				const int back = 1 + static_cast<int>(rng() % 2);
				for (int bsteps = 0; bsteps < back; ++bsteps)
					if (entries_[i].parent >= 0)
						i = entries_[i].parent;
				if (!entries_[i].finished)
					base = i;
			}
			if (base < 0 && mode == 0) {
				for (int pass = 0; pass < 2 && base < 0; ++pass)
					for (int b = 0; b < 32 && base < 0; ++b) {
						if (bands_[b].empty())
							continue;
						if (rng() % 100 < 55) {
							const std::vector<int>& v = bands_[b];
							const int i = v[rng() % v.size()];
							if (!entries_[i].finished)
								base = i;
						}
					}
			} else if (mode == 1) {
				for (int pass = 0; pass < 2 && base < 0; ++pass)
					for (int b = 15; b >= 0 && base < 0; --b) {
						if (sbands_[b].empty())
							continue;
						if (rng() % 100 < 55) {
							const std::vector<int>& v = sbands_[b];
							const int i = v[rng() % v.size()];
							if (!entries_[i].finished)
								base = i;
						}
					}
			} else if (mode == 2 && cfg_.eloss_bias) {
				// SMOOTH FRONTIER (user's routing intuition, measured: fast
				// runs dissipate ~260k vs 538k for the meandering route):
				// among the few contact bands nearest the finish, restart
				// from the LOWEST cumulative-dissipation entry sampled.
				float best_l = 1e30f;
				int nonempty = 0;
				for (int b = 0; b < 32 && nonempty < 3; ++b) {
					if (bands_[b].empty())
						continue;
					nonempty++;
					const std::vector<int>& v = bands_[b];
					for (int c = 0; c < 4; ++c) {
						const int i = v[rng() % v.size()];
						if (!entries_[i].finished
							&& entries_[i].eloss < best_l) {
							best_l = entries_[i].eloss;
							base = i;
						}
					}
				}
			}
			if (base < 0) {
				for (int t = 0; t < 8 && base < 0; ++t) {
					const int i = static_cast<int>(rng() % entries_.size());
					if (!entries_[i].finished)
						base = i;
				}
			}
			if (base < 0)
				continue;

			PlayerState s = entries_[base].st;
			float yaw = entries_[base].yaw;
			int tick = entries_[base].tick;
			signed char last_side = entries_[base].last_side;
			short since_flip = entries_[base].ticks_since_flip;
			short zone_jumps = entries_[base].zone_jumps;
			float eloss = entries_[base].eloss;
			float e_prev = 0.5f * Len2(s.vel) + cfg_.params.gravity * s.pos.Z;
			res.rollouts++;

			knots.clear();
			const int K = 1 + static_cast<int>(rng() % 3);
			bool aborted = false;
			for (int k = 0; k < K && !aborted; ++k) {
				Knot kn = SampleKnot(rng, last_side, since_flip, min_knot_);
				if (kn.side != 0 && last_side != 0 && kn.side != last_side)
					since_flip = 0;
				if (kn.side != 0)
					last_side = kn.side;
				knots.push_back(kn);

				for (int t = 0; t < kn.dur; ++t) {
					TickEvents ev;
					KnotTick(s, yaw, kn, t, w_, cfg_.params, &ev, nullptr);
					tick++;
					res.ticks_simulated++;
					const float e_now = 0.5f * Len2(s.vel)
						+ cfg_.params.gravity * s.pos.Z;
					if (e_now < e_prev)
						eloss += e_prev - e_now;
					e_prev = e_now;
					if (since_flip < 999) since_flip++;
					if (ev.jumped && InsideStartZone(s.pos)) {
						zone_jumps++;
						if (zone_jumps > cfg_.max_zone_jumps) {
							aborted = true;   // the one universal ban
							break;
						}
					}
					if (tick >= cfg_.max_path_ticks) {
						aborted = true;
						break;
					}
					// Event-aligned recording (contact/ground/duck/jump ticks,
					// knot ends, every 4th tick otherwise) - tick-level
					// recording was pure churn.
					const bool eventful = ev.ncontacts > 0 || ev.landed
						|| ev.left_ground || ev.duck_changed || ev.jumped;
					int idx = -1;
					if (eventful || t == kn.dur - 1 || (tick & 3) == 0)
						idx = RecordCell(s, yaw, tick, base, -1, knots,
							k, t + 1, last_side, since_flip, zone_jumps,
							eloss, ev);
					if (!cap_hit && entries_.size() >= 4000000) {
						// FREEZE the frontier, don't stop the run: rollouts
						// keep exploring from the existing archive (finisher
						// entries are still force-added).
						cap_hit = true;
						printf("  entry cap (4M): frontier frozen, exploration "
							"continues\n");
						fflush(stdout);
					}
					if (GoalReached(s, ev)) {
						int fi = idx;
						if (fi < 0) {
							// Cell already better - still keep the finisher
							// entry so its genome is reconstructable.
							Entry e;
							e.st = s;
							e.yaw = yaw;
							e.tick = tick;
							e.parent = base;
							e.seed_ticks = -1;
							e.last_side = last_side;
							e.ticks_since_flip = since_flip;
							e.zone_jumps = zone_jumps;
							e.eloss = eloss;
							e.nknots = static_cast<unsigned char>(k + 1);
							for (int i = 0; i <= k; ++i)
								e.knots[i] = knots[i];
							e.knots[k].dur = static_cast<short>(t + 1);
							fi = static_cast<int>(entries_.size());
							entries_.push_back(e);
						}
						entries_[fi].finished = true;
						Finisher fin;
						fin.tick = tick;
						fin.speed = Len2D(s.vel);
						fin.eloss = eloss;
						fin.pos = s.pos;
						fin.entry = fi;
						finishers.push_back(fin);
						aborted = true;
						break;
					}
				}
			}

			if (elapsed() >= next_print) {
				next_print += 1.0;
				int best = -1;
				for (const Finisher& f : finishers)
					if (best < 0 || f.tick < best)
						best = f.tick;
				printf("  %5.1fs  rollouts %lld  entries %d  cells %d  "
					"finishes %d  best %s  min-dist %.0f  max-spd %.0f\n",
					elapsed(), res.rollouts, static_cast<int>(entries_.size()),
					static_cast<int>(CMAP().size()),
					static_cast<int>(finishers.size()),
					best < 0 ? "-" : std::to_string(best).c_str(),
					min_dist_seen_, max_speed_seen_);
				fflush(stdout);
			}
		}
		if (min_dist_entry_ >= 0) {
			const Entry& me = entries_[min_dist_entry_];
			printf("  closest entry: tick %d  pos (%.0f, %.0f, %.0f)  "
				"vel (%.0f, %.0f, %.0f)  %s%s\n", me.tick,
				me.st.pos.X, me.st.pos.Y, me.st.pos.Z,
				me.st.vel.X, me.st.vel.Y, me.st.vel.Z,
				me.st.on_ground ? "ground" : "air",
				me.st.ducked ? " ducked" : "");
			fflush(stdout);
		}
		// Per-brush contact census: names which pipeline stage is starved.
		{
			struct BStat {
				int n1 = 0, n2 = 0;
				float maxsp1 = 0.f, maxsp2 = 0.f, mind = 1e9f;
			};
			std::vector<BStat> bs(w_.brushes.size());
			for (const Entry& e : entries_) {
				if (e.ckind == 0 || e.cbrush < 0
					|| e.cbrush >= static_cast<short>(bs.size()))
					continue;
				BStat& b = bs[e.cbrush];
				const float sp = Len2D(e.st.vel);
				if (e.ckind == 1) {
					b.n1++;
					if (sp > b.maxsp1) b.maxsp1 = sp;
				} else {
					b.n2++;
					if (sp > b.maxsp2) b.maxsp2 = sp;
				}
				const float d = Len(e.st.pos - end_center_);
				if (d < b.mind) b.mind = d;
			}
			printf("  contact census (brush: ground/touch, gnd-spd/tch-spd, min-dist):\n");
			for (size_t i = 0; i < bs.size(); ++i)
				if (bs[i].n1 + bs[i].n2 > 0)
					printf("    brush %2d: %d/%d  %5.0f/%5.0f  %5.0f\n",
						w_.brushes[i].id, bs[i].n1, bs[i].n2,
						bs[i].maxsp1, bs[i].maxsp2, bs[i].mind);
			fflush(stdout);
		}
		std::sort(finishers.begin(), finishers.end(),
			[](const Finisher& a, const Finisher& b) { return a.tick < b.tick; });
		res.finishers = std::move(finishers);
		res.entries = static_cast<int>(entries_.size());
		res.cells = static_cast<int>(CMAP().size());
		res.seconds = elapsed();
		return res;
	}

	bool Explorer::BuildFrames(int entry_index, std::vector<TapeFrame>& out,
	                           int* finish_tick) {
		out.clear();
		if (entry_index < 0 || entry_index >= static_cast<int>(entries_.size()))
			return false;
		std::vector<int> chain;
		for (int i = entry_index; i >= 0; i = entries_[i].parent)
			chain.push_back(i);
		std::reverse(chain.begin(), chain.end());

		// Base state: the root anchor entry (index 0).
		PlayerState s = entries_[0].st;
		float yaw = entries_[0].yaw;

		// A chain whose base is a SEED entry replays the seed tape prefix.
		size_t ci = 0;
		if (chain[0] != 0) {
			const Entry& basee = entries_[chain[0]];
			if (basee.seed_ticks < 0 || !seed_tape_)
				return false;   // orphan chain - determinism fault
			for (int t = 0; t < basee.seed_ticks; ++t) {
				const TapeFrame& f = seed_tape_->frames[t];
				TickEvents ev;
				MoveTick(s, w_, cfg_.params, f.pitch, f.yaw, f.fmove, f.smove,
				         f.umove, f.buttons, &ev);
				out.push_back(f);
			}
			yaw = basee.yaw;
			ci = 1;
		} else {
			ci = 1;   // skip the root itself (no knots)
		}

		for (; ci < chain.size(); ++ci) {
			const Entry& e = entries_[chain[ci]];
			for (int ki = 0; ki < e.nknots; ++ki) {
				const Knot& k = e.knots[ki];
				for (int t = 0; t < k.dur; ++t) {
					TickEvents ev;
					TapeFrame f;
					KnotTick(s, yaw, k, t, w_, cfg_.params, &ev, &f);
					out.push_back(f);
				}
			}
			// Determinism check: the re-rolled state must land on the stored
			// snapshot (same code path, same params -> bit-equal expected).
			const float d = Len(s.pos - e.st.pos);
			if (d > 0.01f) {
				printf("BuildFrames: chain state mismatch %.4f u at entry %d "
					"(tick %d) - determinism fault\n", d, chain[ci], e.tick);
				return false;
			}
		}
		if (finish_tick)
			*finish_tick = static_cast<int>(out.size());
		if (static_cast<int>(out.size()) != entries_[entry_index].tick) {
			printf("BuildFrames: tick count mismatch (%d vs %d)\n",
				static_cast<int>(out.size()), entries_[entry_index].tick);
			return false;
		}
		return true;
	}

} // namespace Solver
