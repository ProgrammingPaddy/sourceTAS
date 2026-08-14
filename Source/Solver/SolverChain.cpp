#include "SolverChain.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <queue>
#include <vector>

namespace Solver {

	namespace {

		bool InsideXY(const Vec3& p, const WorldBrush& b) {
			return p.X >= b.gmin_stand.X && p.X <= b.gmax_stand.X
				&& p.Y >= b.gmin_stand.Y && p.Y <= b.gmax_stand.Y;
		}

	} // namespace

	ChainSolver::ChainSolver(const World& w, const ChainConfig& cfg,
	                         const TapeAnchor& anchor)
		: w_(w), cfg_(cfg), anchor_(anchor) {
		root_.pos = anchor.origin;
		root_.vel = anchor.velocity;
		root_.ducked = anchor.ducked;
		root_.stamina = anchor.stamina;
		root_yaw_ = anchor.yaw;
		TraceResult tr;
		const float gf = w_.TraceHull(root_.pos,
			root_.pos - Vec3(0.f, 0.f, 2.f), root_.ducked, &tr);
		if (gf < 1.f && tr.brush >= 0
			&& tr.normal.Z >= cfg_.params.walkable_z) {
			root_.pos.Z -= 2.f * gf;
			root_.on_ground = true;
			root_.ground_brush = tr.brush;
		}

		// Surf faces from the collision set: real sides too steep to walk
		// on but facing upward. walkable_z comes from live params.
		for (int bi = 0; bi < static_cast<int>(w_.brushes.size()); ++bi) {
			const WorldBrush& b = w_.brushes[bi];
			for (int pi = 0; pi < b.nsides; ++pi) {
				const Vec3& n = b.n[pi];
				if (n.Z <= 0.05f || n.Z >= cfg_.params.walkable_z)
					continue;
				Face f;
				f.brush = bi;
				f.brush_id = b.id;
				f.plane = pi;
				f.n = n;
				Vec3 c(0.5f * (b.bmin.X + b.bmax.X),
				       0.5f * (b.bmin.Y + b.bmax.Y),
				       0.5f * (b.bmin.Z + b.bmax.Z));
				const float off = Dot(n, c) - b.d[pi];
				f.center = c - Scale(n, off);
				faces_.push_back(f);
			}
		}
	}

	ChainSolver::SegOut ChainSolver::SolveSegment(const PlayerState& root,
	                                              float yaw, int target_face,
	                                              bool first_segment,
	                                              unsigned rng,
	                                              long long* ticks,
	                                              float ty_lo, float ty_hi) {
		SmoothConfig sc;
		sc.params = cfg_.params;
		sc.end_brush_id = cfg_.end_brush_id;
		sc.start_brush_id = first_segment ? cfg_.start_brush_id : -1;
		sc.max_ticks = first_segment ? cfg_.first_seg_ticks : cfg_.seg_ticks;
		sc.cp_ticks = cfg_.cp_ticks;
		sc.rng_seed = rng;
		sc.threads = cfg_.threads;
		sc.wloss = cfg_.wloss;
		sc.mix_mu = cfg_.mix_mu;
		sc.w_tick = cfg_.w_tick;
		sc.max_restarts = 3;      // segments early-stop once explored
		sc.stall_gens = 200;
		sc.chain_genes = true;    // guidance + structural ride hold
		if (target_face >= 0) {
			sc.goal_face_brush = faces_[target_face].brush;
			sc.goal_face_plane = faces_[target_face].plane;
			sc.fitness_mode = 1;
			sc.touch_settle = cfg_.touch_settle;
			// Banded solves split the edge budget (3 bands ~ 1.8x one).
			sc.budget_seconds = cfg_.seg_seconds * 0.6;
			sc.ty_lo = ty_lo;
			sc.ty_hi = ty_hi;
		} else {
			sc.fitness_mode = 0;
			sc.budget_seconds = cfg_.end_seconds > cfg_.seg_seconds
				? cfg_.end_seconds : cfg_.seg_seconds;
		}

		SmoothOpt opt(w_, sc, anchor_);
		opt.SetRoot(root, yaw);
		opt.SetQuiet(true);
		SmoothResult r = opt.Run();
		if (ticks)
			*ticks += r.ticks_simulated;

		SegOut out;
		out.dmin = r.best_stats.dmin;
		if (!r.ok) {
			if (!cfg_.dump_dir.empty() && !r.best_x.empty()) {
				SmoothOpt od(w_, sc, anchor_);
				od.SetRoot(root, yaw);
				od.SetQuiet(true);
				std::vector<TapeFrame> df;
				SmoothStats ds;
				if (od.BuildFrames(r.best_x, df, &ds) && !df.empty()) {
					TapeAnchor sa;
					sa.valid = true;
					sa.origin = root.pos;
					sa.velocity = root.vel;
					sa.ducked = root.ducked;
					sa.stamina = root.stamina;
					sa.yaw = yaw;
					static int dump_n = 0;
					char nm[64];
					_snprintf_s(nm, sizeof(nm), _TRUNCATE,
						"\\dead%03d_to%d.tas", dump_n++,
						target_face >= 0
							? faces_[target_face].brush_id : -1);
					std::string err;
					WriteTas(cfg_.dump_dir + nm, sa, "surf_basictest",
						df, &err);
				}
			}
			return out;
		}
		// END mode accepts only CLEAN finishes as segment success (the
		// user's acceptance rule; jump-ended assemblies are never built).
		if (target_face < 0 && !r.clean)
			return out;
		SmoothStats st;
		std::vector<TapeFrame> frames;
		SmoothOpt opt2(w_, sc, anchor_);
		opt2.SetRoot(root, yaw);
		opt2.SetQuiet(true);
		if (!opt2.BuildFrames(r.best_x, frames, &st))
			return out;
		if (!(st.finished || st.touched))
			return out;
		out.ok = true;
		out.finished = st.finished && st.clean;
		out.V = st.vboard - cfg_.wloss * st.eloss;
		out.speed = st.finish_speed;
		out.ticks = st.tick;
		out.end_state = st.end_state;
		out.end_yaw = st.end_yaw;
		out.frames = frames;
		return out;
	}

	ChainSolver::AsmStats ChainSolver::ReplayAssembly(
		const std::vector<TapeFrame>& frames) const {
		AsmStats a;
		PlayerState s = root_;
		const int start_idx = w_.IndexOfBrushId(cfg_.start_brush_id);
		const int end_idx = w_.IndexOfBrushId(cfg_.end_brush_id);
		bool launch_jump = false;
		for (int t = 0; t < static_cast<int>(frames.size()); ++t) {
			const TapeFrame& f = frames[t];
			TickEvents ev;
			MoveTick(s, w_, cfg_.params, f.pitch, f.yaw, f.fmove, f.smove,
				f.umove, f.buttons, &ev);
			float loss = 0.f;
			for (int c = 0; c < ev.ncontacts; ++c)
				loss += ev.contact_loss[c];
			if (loss > a.max_impact)
				a.max_impact = loss;
			if (ev.left_ground)
				launch_jump = ev.jumped;
			else if (!s.on_ground && ev.ncontacts > 0
				&& w_.brushes[ev.contact_brush[0]].id != cfg_.end_brush_id)
				launch_jump = false;
			if (a.exit < 0 && start_idx >= 0
				&& !InsideXY(s.pos, w_.brushes[start_idx]))
				a.exit = t;
			if (a.finish < 0 && end_idx >= 0 && s.on_ground
				&& s.ground_brush >= 0
				&& w_.brushes[s.ground_brush].id == cfg_.end_brush_id
				&& InsideXY(s.pos, w_.brushes[end_idx])) {
				a.finish = t;
				break;
			}
		}
		a.clean = !launch_jump;
		return a;
	}

	ChainResult ChainSolver::Run() {
		ChainResult res;
		const auto t0 = std::chrono::steady_clock::now();
		auto elapsed = [&]() {
			return std::chrono::duration<double>(
				std::chrono::steady_clock::now() - t0).count();
		};

		printf("chain: %d surf faces:", static_cast<int>(faces_.size()));
		for (size_t i = 0; i < faces_.size(); ++i)
			printf("  F%d=brush %d n(%.2f,%.2f,%.2f)",
				static_cast<int>(i), faces_[i].brush_id,
				faces_[i].n.X, faces_[i].n.Y, faces_[i].n.Z);
		printf("\n");
		printf("chain: budget %.0fs total, %.1fs/segment, beam %d, "
			"depth <= %d, rng %u\n", cfg_.total_seconds, cfg_.seg_seconds,
			cfg_.beam, cfg_.max_depth, cfg_.rng_seed);
		fflush(stdout);
		if (faces_.empty() || cfg_.end_brush_id < 0)
			return res;

		struct Node {
			PlayerState st;
			float yaw = 0.f;
			std::vector<TapeFrame> stream;
			float V = 0.f;
			int depth = 0;
			std::vector<int> skel;
		};
		struct PQE {
			float pri;
			int idx;
			bool operator<(const PQE& o) const { return pri < o.pri; }
		};
		std::vector<Node> nodes;
		std::priority_queue<PQE> pq;
		{
			Node r;
			r.st = root_;
			r.yaw = root_yaw_;
			nodes.push_back(r);
			pq.push({ 1e9f, 0 });   // root first, always
		}
		// Beam ledger: top-V junctions per (depth, face), POSITION-DIVERSE
		// (three copies of the same corner clip starve everything after -
		// arrival diversity is the load-bearing lesson).
		struct BeamEnt { float V; Vec3 pos; };
		std::vector<std::vector<BeamEnt>> beamv(
			static_cast<size_t>(cfg_.max_depth + 1) * faces_.size());

		std::vector<TapeFrame> best_frames;
		std::vector<int> best_skel;
		int best_scored = 1 << 28;
		bool have_best = false;
		unsigned seg_counter = 0;

		auto skel_str = [&](const std::vector<int>& sk) {
			std::string s = "[";
			for (size_t i = 0; i < sk.size(); ++i) {
				if (i) s += ">";
				s += std::to_string(faces_[sk[i]].brush_id);
			}
			s += "]";
			return s;
		};

		while (!pq.empty()
			&& elapsed() < cfg_.total_seconds - cfg_.final_seconds) {
			const Node node = nodes[pq.top().idx];
			pq.pop();
			res.nodes_expanded++;

			std::string line = "[chain] d" + std::to_string(node.depth)
				+ " " + skel_str(node.skel);

			// 1) Try to LAND from here (clean finishes only).
			if (node.depth > 0) {
				SegOut endseg = SolveSegment(node.st, node.yaw, -1, false,
					cfg_.rng_seed + 7919u * ++seg_counter,
					&res.ticks_simulated);
				res.segments_solved++;
				if (endseg.finished) {
					std::vector<TapeFrame> full = node.stream;
					full.insert(full.end(), endseg.frames.begin(),
						endseg.frames.end());
					const AsmStats a = ReplayAssembly(full);
					if (a.finish >= 0 && a.clean) {
						const int scored = a.exit >= 0
							? a.finish - a.exit : a.finish;
						res.finishes++;
						line += " ->END FINISH " + std::to_string(scored)
							+ " scored";
						if (!have_best || scored < best_scored) {
							have_best = true;
							best_scored = scored;
							best_frames = full;
							best_skel = node.skel;
							line += " ** NEW BEST **";
						}
					} else {
						line += " ->END verify-miss";
					}
				} else {
					line += " ->END no (d" + std::to_string(
						static_cast<int>(endseg.dmin)) + ")";
				}
			}

			// 2) Expand to every face (skeleton enumeration; immediate
			// re-board of the same face deferred - logged, not searched).
			if (node.depth < cfg_.max_depth) {
				// CLAUSE-3 ENUMERATION: solve each edge in THREE board-
				// height bands (low/mid/high thirds of the face) - energy
				// is conserved in flight, so a single V pick collapses
				// genuinely different reach options into "earliest board".
				static const float kBands[4] = { 0.f, 0.34f, 0.67f, 1.f };
				for (int fi = 0; fi < static_cast<int>(faces_.size());
					++fi) {
					if (!node.skel.empty() && node.skel.back() == fi)
						continue;
					bool any = false;
					for (int bnd = 0; bnd < 3; ++bnd) {
						if (elapsed() >= cfg_.total_seconds
							- cfg_.final_seconds)
							break;
						SegOut seg = SolveSegment(node.st, node.yaw, fi,
							node.depth == 0,
							cfg_.rng_seed + 7919u * ++seg_counter,
							&res.ticks_simulated,
							kBands[bnd], kBands[bnd + 1]);
						res.segments_solved++;
						if (!seg.ok)
							continue;
						any = true;
						std::vector<BeamEnt>& bv = beamv[
							static_cast<size_t>(node.depth + 1)
								* faces_.size() + fi];
						// Position-diverse beam admission.
						int near = -1;
						for (size_t k = 0; k < bv.size(); ++k)
							if (Len(bv[k].pos - seg.end_state.pos)
								< cfg_.beam_sep)
								{ near = static_cast<int>(k); break; }
						if (near >= 0) {
							if (seg.V <= bv[near].V)
								continue;
							bv[near] = { seg.V, seg.end_state.pos };
						} else if (static_cast<int>(bv.size())
							< cfg_.beam) {
							bv.push_back({ seg.V, seg.end_state.pos });
						} else {
							float mn = bv[0].V;
							size_t mi = 0;
							for (size_t k = 1; k < bv.size(); ++k)
								if (bv[k].V < mn) { mn = bv[k].V; mi = k; }
							if (seg.V <= mn)
								continue;
							bv[mi] = { seg.V, seg.end_state.pos };
						}
						Node child;
						child.st = seg.end_state;
						child.yaw = seg.end_yaw;
						child.stream = node.stream;
						child.stream.insert(child.stream.end(),
							seg.frames.begin(), seg.frames.end());
						child.V = seg.V;
						child.depth = node.depth + 1;
						child.skel = node.skel;
						child.skel.push_back(fi);
						nodes.push_back(child);
						pq.push({ seg.V,
							static_cast<int>(nodes.size()) - 1 });
						char b[96];
						_snprintf_s(b, sizeof(b), _TRUNCATE,
							"  F%d/%c V%.0fk spd%.0f z%.0f", fi,
							"LMH"[bnd], seg.V / 1000.f, seg.speed,
							seg.end_state.pos.Z);
						line += b;
					}
					if (!any)
						line += "  F" + std::to_string(fi) + " dead";
				}
			}
			char tail[96];
			_snprintf_s(tail, sizeof(tail), _TRUNCATE,
				"  | q%d %ds best %s", static_cast<int>(pq.size()),
				static_cast<int>(elapsed()),
				have_best ? std::to_string(best_scored).c_str() : "-");
			line += tail;
			printf("%s\n", line.c_str());
			fflush(stdout);
		}

		if (!have_best) {
			res.seconds = elapsed();
			printf("chain: NO clean assembly found (%d nodes, %d segments)\n",
				res.nodes_expanded, res.segments_solved);
			return res;
		}

		// 3) Global polish: invert the machine stream (near-lossless) and
		// run full-line CMA on it.
		printf("chain: best assembly %s %d scored - global polish %.0fs\n",
			skel_str(best_skel).c_str(), best_scored, cfg_.final_seconds);
		fflush(stdout);
		{
			SmoothConfig gc;
			gc.params = cfg_.params;
			gc.start_brush_id = cfg_.start_brush_id;
			gc.end_brush_id = cfg_.end_brush_id;
			gc.max_ticks = static_cast<int>(best_frames.size()) + 256;
			gc.cp_ticks = cfg_.cp_ticks;
			gc.budget_seconds = cfg_.final_seconds;
			gc.rng_seed = cfg_.rng_seed;
			gc.threads = cfg_.threads;
			gc.wloss = cfg_.wloss;
			gc.mix_mu = cfg_.mix_mu;
			SmoothOpt g(w_, gc, anchor_);
			Tape asmb;
			asmb.start = anchor_;
			asmb.start.valid = true;
			asmb.frames = best_frames;
			g.SeedFromTape(asmb);
			SmoothStats pm;
			g.Evaluate(g.SeedMean(), &pm, nullptr);
			printf("chain: polish projection: %s\n", pm.finished
				? (pm.clean ? "clean finish (inversion holds)"
					: "JUMP finish")
				: "no finish (assembly ships un-polished)");
			fflush(stdout);
			if (pm.finished && pm.clean) {
				SmoothResult pr = g.Run();
				res.ticks_simulated += pr.ticks_simulated;
				if (pr.ok && pr.clean && pr.best_rel < best_scored) {
					std::vector<TapeFrame> pf;
					SmoothStats ps;
					if (g.BuildFrames(pr.best_x, pf, &ps) && ps.finished
						&& ps.clean) {
						const AsmStats a = ReplayAssembly(pf);
						if (a.finish >= 0 && a.clean) {
							const int scored = a.exit >= 0
								? a.finish - a.exit : a.finish;
							if (scored < best_scored) {
								printf("chain: polish IMPROVED %d -> %d "
									"scored\n", best_scored, scored);
								best_scored = scored;
								best_frames = pf;
							}
						}
					}
				}
			}
		}

		const AsmStats fin = ReplayAssembly(best_frames);
		res.ok = fin.finish >= 0;
		res.clean = fin.clean;
		res.scored = fin.exit >= 0 ? fin.finish - fin.exit : fin.finish;
		res.abs_tick = fin.finish;
		res.frames = best_frames;
		if (res.ok && static_cast<int>(res.frames.size()) > fin.finish + 1)
			res.frames.resize(fin.finish + 1);
		res.skeleton = best_skel;
		res.seconds = elapsed();
		return res;
	}

} // namespace Solver
