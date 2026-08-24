# The Capability Checklist — master table

**Interactive companion:** `Docs/CapabilityChecklist.html` — the same
rows searchable, sortable, and cross-linked (terms ↔ capabilities ↔
dependencies), with the expansive detail in click-open drawers.
Update BOTH files every session; this file remains canonical.

**Opened 2026-08-22 (session 25), user directive.** One table, every
capability: its terms, what it feeds, the selected method, the
alternatives, an efficiency figure, and a checklist status. This is
BOTH the visual organization of the whole program AND the working
checklist — update the status column every session. Full method
analysis lives in `Docs/CapabilityMethods.md`; statuses and session
logs in `Docs/CapabilityLibrary.md`.

**Status marks:** `[x]` done + certified · `[~]` partial (what
remains is named) · `[d]` designed, not built · `[ ]` not started ·
`[-]` excluded by stated domain (listed so no gap hides).

**Efficiency column:** measured numbers where they exist (this
week's machine), targets marked "target". The reference costs:
one exact engine tick (MoveTick) = 559 ns; one exact kernel tick
(A4-prod) = 62 ns.

## A — physical capabilities

| ID | Capability | Key terms | Feeds | Primary method | Other methods | Efficiency | Status |
|----|-----------|-----------|-------|----------------|---------------|------------|--------|
| A0 | canonical frame transform | position, heading | every table/family | `CapFrame` — the ONE rotation+translation (bitwise the inline math every solver used); tables canonical, witnesses world-frame | — | ~ns; round trip ≤ 0.0016u at 8000u; rotation-commutation of full rollouts measured 0.0002u worst (no theorem — the world-frame witness rule stands) | `[x]` packaged s33 (capsolve: 40k bitwise, round trip gated) |
| A1 | vertical state z(N), vz(N) | z, vz, gravity halves | arrival windows (B0), A13, A27 | closed form | per-tick iteration (pointless) | ~ns, O(1) | `[x]` |
| A2 | face slice λ(z) | λ, face plane | A13 targets, B1 | plane/polygon algebra | — | ~ns, O(1) | `[x]` |
| A3 | hull contact geometry | hull boxes, support offsets | A14, A26, A27, C7 targeting | `CapHull::PlaneOffset` — Minkowski support of the negated hull, float-identical to the world loader's own expansion (d = d_raw + off(n)) | analytic contact regions per face (edge/bevel regimes, open) | O(1), ~ns/plane | `[x]` packaged s32; capxfer gates it bitwise vs d_stand/d_duck/d_unduck (48/48); fixing C7 v1's 1.8–3.7-tick early-contact systematic was its first consumer |
| A4 | one-tick air law | c = s·cosα_true, s², a, δψ | everything airborne | closed law (TickLaw) + bitwise kernel | full MoveTick (9× slower) | kernel 16.0M ticks/s = 62 ns/tick; law match: worst Δs² 5.0, Δψ 1.7e-5 rad over 1M states | `[x]` capkern 5/5 |
| A5 | dwell automaton | dwell age, min_gap 6 | all schedule legality | counter + comparison | — | ~ns | `[x]` guarded by the lattice-agreement gate |
| A6 | N-tick turn/gain max V* | s², ψ, reversal count | B5/B6, C5, transfer valuation | closed-form ceiling √(s0²+900N) for bounds; band table for full surfaces | parametric family (via A12 machinery, future); shooting oracle | ceiling O(1) ns; band build 48–276 s/v0, then O(1) query; gaps: sampled-action 0, continuous +20…+93 u/s measured | `[x]` as certified interval; `[ ]` heading-aware UB (B22) |
| A7 | dual Ψ* (max turn at speed floor) | ψ, V_min | B5, approach planning | read A6 layers | — | O(bins) µs | `[x]` with A6 |
| A8 | directional reach D* | arc length L(N), projections, **the heading-freedom + turn-bound lemmas** | B2, B13, B22, corridor checks | certified bounds (single query); on-demand sweep (batch); **the TURN-GATED direction-aware UB (s39): D*_φ(N) ≤ Σ_{i≥T_turn} √(s0²+900i)·dt, gated by two certified lemmas — heading is ONE-TICK FREE at s ≤ B = 562.5·sf (full-brake reversal, constructive), and above B one tick turns at most atan(B/(s−B)) (measured 93.3% tight)** | parametric family endpoints (future, exact); local refinement on-demand (no consumer yet) | bounds O(N) ~µs; the direction-aware UB certifies a 3–5% tightening for backward φ at high speed | `[x]` s39 (capreach 8/0): reversal witnesses 4/4 exact (slowest backward 7.50 = 562.5−555, the arithmetic confirming the law); turn bound unbeaten by 200k adversarial one-tick actions; the travel UB unbeaten by 20k schedules × 4 (s0, φ) configs — the LAST A amber resolved |
| A9 | displacement with terminal constraint | endpoint + (ψ_N or V_N) | A22 pairing | `CapP2P::SolveTerminal` — endpoint-ranked family (reversals × brake placement × brake STRENGTH) → heading-feasible kernel rolls → scored coarse climb (both contracts judged per roll) → **the 3×3 finisher: (tail c1, tail c2, brake strength) against (endpoint x, y, heading) — a square Newton** → accept only when BOTH contracts hold, else DECLINE | Pareto layer of A8 sweep (future); richer heading family (second brake run) | ~24 ms/request | `[x]` v1 s34 (capsolve): contracts HARD 19/19 (endpoint ≤ 0.5u, heading ≤ tol, independently re-rolled), engine bitwise 8/8; acceptance 19/40 vs the adversarial generator (random braked schedules' exact terminal states) = the measured family-sufficiency line |
| A10 | minimum air time N_min | N window, feasibility | route timing, B21 | `CapP2P::MinAirTime` — the A1 z-window intersected with the A12 scan, first feasible N wins | table lookup | 3.0 ms mean (window-restricted scan) | `[x]` s33 (capsolve: 24/24 == brute full scan) |
| A11 | fixed-time point solve | reversal times, brake run, exact-hit tail, segment tables | A12 core | **EXACT SEGMENT COMPOSITION** (the all-plus spiral's prefix tables give any reversal schedule's endpoint in O(k), no trig — exhaustive k≤2, dense k=3) + brake-run range control + kernel refinement + exact-hit Newton | analytic inversion (the remaining speed refinement); shooting oracle; reach-table batch | **v4 VERIFIED (capp2p 21/0): machinery 99–100/100; single-shot 0.6–3.3 ms; BATCH MODE (s30): organize 0.1–1.4 ms/start, then 34–135 µs/target with full-solver fallback — the 10–30 µs ambition met at short flights; 150/150 bitwise replays** | `[x]` machinery + batch; s40 (THE LEGO RULE): the exact-hit stage is a **STRATEGY PORTFOLIO** — (a) the proven 2×2 three-start tail, (b) the 3-parameter least-norm tail, (c) the post-brake-sized tail, each intact as its own unit, composed as fallbacks (later strategies run only while unsolved → MONOTONE by construction). **The portfolio beat every individual configuration on every cell: 400: 99/100/100 · 800: 91/100/96 · 1400: 93/98/100 — misses 127 → 23 of 900 (2.6%)**; measured coverage pinned as NINE HARD FLOOR GATES in capp2p + two solver-coverage floors in capxfer (55°/65° both 37/49). Remaining measured amber: multi-brake-phase deep interior (worst 48u at 800/N90) |
| A12 | point-to-point air solver | all of A10/A11 | C7, gap feasibility (B13/B14), A22 pairing | `CapP2P::SolveFreeN` — A11 iterated over N with the certified B13 precull (a necessary condition can never hide a feasible N — gated) | shooting oracle (cross-check); R_N table (batch) | 9.7 ms mean free-N scan; 261 N-candidates culled / 36 targets | `[x]` s33 (capsolve: 36/36 preculled == brute, 10/10 engine bitwise); found+fixed the latent small-N stride hang in the brake probes |
| A13 | point-to-face solve | (T, λ) targets | entrance/transfer (C7 consumes it) | `CapFaceSolve::SolveToFace` — A12 at 1-D λ targets per candidate tick on the HULL-EXPANDED plane + `PredictContact` (the exact clip-law prediction) | generic 2-D targeting (wasteful) | organize ~3.5 ms/start + ~100 µs/target (capxfer's measured chain) | `[x]` s33 — capxfer refactored to CONSUME the package (8/0 = its acceptance); rotated-azimuth scenario gates the general slice geometry (12/12 predictions exact) |
| A14 | first-contact prediction | trace, local brush set | A28, event scans | `CapContact::BuildLocalSet` + `ClipSegment` — the engine's per-brush clip (enter-clamp tie-break, DIST_EPSILON pads, corner release, start-solid law) transcribed VERBATIM over a corridor-gathered set; out-of-corridor queries DECLINE | full-world trace (the engine's own, same answers) | 60–74 ns/query vs 113–166 ns full trace (1–2-brush worlds; the locality win scales with brush count) | `[x]` s34 (capcontact 4/0): 40,000/40,000 queries BITWISE identical to the full trace (fraction bits + brush + plane + startsolid), DECLINE law clean, 25/25 flight first-contacts (tick AND brush) vs engine replays |
| A15 | board clip | v·n, loss, post² | A16/A17, C2 | exported engine clip + closed post-speed law | — | ~ns; law dev ≤ 0.0034 u/s over 33k arrivals | `[x]` |
| A16 | best/worst board | v·n extrema over heading | C2, B8/B9 | closed-form candidates (endpoints, cos extrema, zero crossings) | dense enumeration (baseline only) | ~ns (few trig); verified to 1e-4 over 640 configs | `[x]` |
| A17 | board feasibility arcs | loss ≤ L heading arcs | B8, approach windows | closed-form arccos arcs (≤2 per period) | enumeration | ~ns; 0 mismatches / 24×7200 | `[x]` |
| A18 | one-tick ride law | in-plane gravity, sf state, **the carried gap** | A19–A22 | interior: the proven composition (gravity-half → clamp → accel(sf) → one clip → clamp → gravity-half → clamp); **boundary/hover/multi-plane: `CapRide::MirrorTick` — the exact mixed roller** (airborne preamble under STATEFUL sf + the A24 local move mirror + the exact categorize sf rule) | full MoveTick (same answers) | interior ~A4-kernel cost; mirror ~200 ns/tick | `[x]` COMPLETE s36: interior 1246/1246 bitwise (s24) + the boundary regime EXACT — capgap 3600/3600 mixed ticks bitwise (pos+vel+sf) including 842 hover ticks and 327 re-contacts; the carried gap needs no separate recurrence (it IS n·pos − d_exp of a bitwise position) |
| A19 | ride N-tick turn/gain | slope vs heading, sf carried state, **the contact epsilon** (re-contact needs v·n ≤ −(1/32)/dt ≈ −2.08 u/s — hover band measured) | A22, B10/B11 | `CapRide::BuildRide` on the proven A18 law, engine-anchored starts, stay-on-face guard | plane-table refinements | **56–84 s/ramp builds (28–34M nodes); 30/30 witness replays BITWISE as engine continuations; falsifier gaps: 50° +0.9, 60° +215.6, 70° +116.4 (the conservative guard's boundary sliver)** | `[x]` s36 — the named refinement is DELIVERED: the exact contact-boundary condition exists (`CapRide::MirrorTick` + the generalized contact law, capgap 6/0); the conservative guard stays in the BUILD (rebuilding surfaces with the exact roller = optional tightening; the falsifier gaps remain its measured price) |
| A20 | ride directional reach | in-plane (downslope, cross-slope), contact-epsilon guard + drift margin | A22, B10 | `CapRideReach::BuildRideReach` — position-tracking sweep on the proven ride law; **engine-validated witnesses per direction** (a direction with no clean witness DECLINES its bound) | finer local refinement | **9–19 s/ramp (3.3–6.1M nodes at 64u/10°/50u-s); 23/24 directions validated (upslope@50° declined — the near-walkable near-stall corner); D\*ride(0.72 s): down 702–844u, cross ~470u, up ≈ 0** | `[x]` s36 — same resolution as A19: the boundary regime is exactly simulable (capgap); the surface's conservative guard + drift margin remain its build-time choice, and the declined upslope@50° direction is now REACHABLE BY EXACT SIMULATION when needed |
| A21 | minimum ride time | exit point, N_min | route timing | `CapRideReach::MinRideTime` — first A20 layer answering within the lattice contract; indexed-first, every miss confirmed by the exhaustive scan (result EQUALS brute by construction) | pure brute per-layer scan (same answers, measured 696 µs vs 857 µs — the confirm pass eats the index's edge on miss-heavy scans) | ~0.86 ms/query | `[x]` s33 (capmin: 50/50 == brute; 12/12 minimum-time witnesses engine-validated, 0 fringe) |
| A22 | board→exit solver | entry state × exit point | C6, the two-ramp transfer | **v1: cell-witness lookup in the A20 sweep** (exact schedules out); v2: the A11 exact-hit construction on the plane | DP table per face (baseline/falsifier) | **v2 (s30): O(1) indexed query 0.2–0.3 µs (~10,000× the layer scan), lattice-resolution answers (mean ~30u, worst 55u, all within the 96u contract; the nearest-node full scan at 0.7u mean remains available)** | `[x]` v3 s36 (capgap): **the exact-hit tail on the ride plane** — a 20-tick tail (C- or S-curve shape, speed-correct live-band mapping ca = 0.95·cap/s·x) Newton-driven through the exact mixed roller: 20/20 in-authority targets recovered to mean 0.097u / max 0.242u (the lattice answered ~30u), 8/8 refined schedules engine-BITWISE; targets beyond the tail's authority hull DECLINE to the lattice+N choice |
| A23 | ride edge interception | edge geometry, exit λ | launch selection | `CapRideReach::EdgeIntercept` — the A21 minimum-time query batched along an in-plane edge segment | — | ~0.86 ms/sample (A21's cost) | `[x]` s35 (capmin 8/0): 25/25 samples equal the brute per-layer scan; 8/8 interception witnesses engine-validated, 0 fringe |
| A24 | direct contact transfer | adjacent-face clip chain, crease law | face-to-face routing | **`CapEdge::TryMoveLocal` — the verified TryPlayerMove as a PURE LOCAL FUNCTION** (every trace = the bitwise A14 clip): partial-move rebases, two-plane crease resolution, stop-dead guard, allsolid zeroing, the unswept stuck guard; `EdgeTickFull` wraps the whole crossing tick | the analytical two-plane laws (`ResolveTwoPlanes`/`SequentialTransfer`, kept as the decomposition); full engine roll-through | ~ns/edge chain | `[x]` s35 (capmin 8/0): 36/36 valley-crease crossing ticks BITWISE in position AND velocity — including 17 three-plus-bump chains no two-plane law covers (the census that forced the full mirror) |
| A25 | ground/runoff acceleration | friction, stopspeed, accelerate, stamina power curve | spawn/launch prep | `CapGround::WalkKernelTick` — the grounded MoveTick chain partially evaluated on the flat-floor domain | full MoveTick baseline | **40.8M ticks/s = 10.9× engine; 200k walk ticks + 50k position checks, 0 bitwise mismatches (velocity, feet z, stamina)** | `[x]` capground 6/6 (s27) |
| A26 | edge launch | leave-ground event, launch state | route segments | `CapLaunch::Detector` — the unified leave-ground event detector fed by exact rollouts; captures both boundary states | the route-graph LaunchWitness/ReplayLaunch instruments remain the production launch pipeline | ~ns/tick fed | `[x]` s33 (capmin): threshold bracketed to ONE constant across speeds — (270.7, 272.4] ∋ 256+16, the A3 hull-overhang law measured, quadrant semantics not assumed; captures continue BITWISE; first clean air tick hands to A1 bitwise |
| A27 | END-volume crossing | Minkowski box, z-window | finish timing | `CapEnd::EarliestBoxCrossing` — z-window-pruned exact scan | full per-tick scan (baseline) | **= naive on 10k (schedule, box) pairs; 12.1× fewer horizontal tests** | `[x]` capwindow (s28) |
| A28 | obstacle/corridor traversal | first contact along family | route validation | `CapContact::FirstContactOnPath` — the A12 + A14 composition: kernel-roll the schedule, clip every motion segment against the corridor set; clean / obstructed(tick, brush, fraction) / DECLINE | swept-volume precomputation | 4–5 µs/leg measured | `[x]` s35 (capcontact 5/0): 24/24 solved paths agree with engine replays — 16 clean tick-for-tick, 8 obstructed at the exact tick AND brush; plus the flight gate 14/14 with the stuck-guard predictor armed |
| A29 | duck capability | duck flag/hull/timer, 8.5u shift, 0.34 speed crop | duck-required routes, A3 hulls | the vtable-pinned `Fn::` duck family IS the package (Duck/CanUnduck/FinishDuck/FinishUnDuck/HandleDuckingSpeedCrop/ReduceTimers) + the A3 hull switch; the LAWS gated independently | — | ~ns/tick law | `[x]` s37 (capdebt 7/0): air press/unduck shift = EXACTLY ±8.5000u, one constant across events, instant with hull 1 / transient hull 2; timer drains exactly dt·1000 on steady ticks (162/162); grounded crop terminal speed exactly 85.00 (0.34·250); AND the mixed roller at hull 1 predicts ducked-flight contacts 15/15 vs engine — the roller generalizes across hulls |
| A30 | jump capability | jump impulse (double const), stamina scale, autobhop gate, three gravity half-steps | spawn runs, bhop segments, launches | `CapGround::JumpKernelTick` (jump head + the A4 air chain) | — | **50k jump ticks, 0 bitwise mismatches on all channels + stamina; vz law: stamina 0→283.993, 1315.8→210.839; release gate + autobhop bypass verified** | `[x]` capground (s27); SET path (ducked) = A29 debt |
| A31 | trigger interactions | basevel, gravity_scale, teleport | maps with push/teleport/gravity | `CapTrigger::ApplyHit` — the mirror's post-move application transcribed as ONE pure transform (gravity overwrites the scale; pushes accumulate onto basevel with the unground + 1u nudge; teleports set origin and zero velocity) | — | ~ns/event | `[x]` s37 (capdebt 7/0): all three types bitwise on touch ticks (23/23), AND the carried state flows through the certified kernel as a FULL COMPOSITION — per-tick kernel + re-touch + ApplyHit (pushes accumulate inside the volume) matches the engine bitwise for 15 post-touch ticks, 23/23 flights |
| A32 | water / ladders (exclusion) | — | — | — | — | — | `[-]` excluded: surf maps in scope have neither; revisit only if a target map does |

## B — certified reducers (prune only with proof)

| ID | Reducer | Key terms | Feeds | Primary method | Other methods | Efficiency | Status |
|----|---------|-----------|-------|----------------|---------------|------------|--------|
| B0 | vertical tick window per face | z window | sweep/search culls | closed form (A1+A2) | — | ~ns | `[x]` deployed |
| B1 | face-slice validity | λ interval | target filters | closed form (A2) | — | ~ns | `[x]` |
| B2 | travel bound (replaces the refuted cone) | max-speed integral | reach culls | Σ√(s0²+900i)·dt upper bound | direction-aware version (B22-adjacent, future) | O(N) ~µs, precomputable prefix sums → O(1) | `[x]` certified; forward-tight to 1.0u measured |
| B3 | earliest contact tick | B0 × B2 | search windows | `CapWindow::ContactWindow` (exact vertical recurrence × certified travel bound) | — | ~µs; falsifier: 915 real schedules, 0 beat the window | `[x]` capwindow (s28) |
| B4 | latest useful contact tick | z window closure | search windows | below the band falling ⟹ never returns (exact recurrence) | — | ~µs | `[x]` with B3 |
| B5 | heading-change feasibility | Ψ* | approach culls | A6/A7 lower bounds ADMIT only | heading-aware UB (B22) would enable culls | O(1) | `[~]` admit-only until B22 |
| B6 | terminal-speed feasibility | speed ceiling | gap culls | required V > √(s0²+900N) ⟹ impossible | — | O(1) ~ns | `[x]` first certified air cull |
| B7 | point/region reachability | B0×B2 + A8 | candidate filters | interval tests then A8/A12 | R_N membership (batch) | ~ns then µs | `[~]` |
| B8 | boardability bound | A17 arcs | arrival culls | closed-form arcs | — | ~ns | `[x]` via A17 |
| B9 | minimum unavoidable board loss | min v·n | valuation floors | closed form (A16) | — | ~ns | `[x]` via A16 |
| B10 | ride point/edge reachability | A19/A20 bounds | ride culls | ride analogues of B2/B6 | — | target ~ns | `[ ]` with A19/A20 |
| B11 | minimum ride ticks | A21 | timing bounds | inverse ride bounds | — | target O(1) | `[ ]` |
| B12 | ride energy ceiling U_E | E = s²+vz² | ride culls | certified (session 15) | — | O(1) | `[x]` |
| B13 | minimum departure resource for a gap | A12 inverse | exit valuation | `CapBounds::MinDepartSpeed` — closed-form ceiling + travel-bound bisection (both certified-monotone) | table | **O(1)/~µs; 24 A11-solver attacks from 0.98× the bound, 0 refutations** | `[x]` capwindow (s30) |
| B14 | successor's minimum incoming resource | backward A12/A16/A22 | the speed-compounding explainer | `CapBounds::MinIncomingSpeed` — **exact piecewise-quadratic infimum** (the monotone-bisection draft REFUTED at 20/127, a missing zero-loss root at 22/185 — both falsifier-caught, both fixed) | — | **O(1) closed form; 185 configs × 401 exact-clip headings at 0.98× the bound, 0 violations** | `[x]` capwindow (s30) |
| B15 | guaranteed collision / corridor exclusion | A14 along families | route culls | local-set trace certificates | swept volumes | target ~µs/leg | `[ ]` |
| B16 | conservative successor-face set | B0–B15 | route enumeration | intersection of the above | — | ~µs/face pair | `[ ]` composition |
| B17 | cell optimistic bound | capability extrema per cell | batch search | derive from A6/A8 bounds ONLY (instruments retired from pruning) | — | O(1)/cell | `[~]` proofs-only rule enforced |
| B18 | boundary-label dominance | state equivalence | frontier compression | OPEN THEOREM — prune only if proven | — | — | `[ ]` open |
| B19 | exact duplicate dominance | state identity | dedup | hash compare | — | ~ns; measured near-zero yield on real geometry | `[x]` kept, expectations recorded |
| B20 | competitive horizon | incumbent T* | global search | T*−g cutoff | — | O(1) | `[x]` mechanism ready, unengaged |
| B21 | minimum remaining ticks to END | compose B2/B11 | admissible h | sum of certified leg minima | — | target O(path) | `[ ]` — h_cert stays 0 until real |
| B22 | heading-aware speed ceiling (NEW row) | V*(N, Δψ) UB | closes A6's 180°-at-speed corner; B5 culls | braking-time analysis (the recorded theorem step) | empirical band + margin (never prunes) | target O(1) | `[ ]` next theorem |

## C — composition / selection (consume proofs, never invent physics)

| ID | Function | Key terms | Primary method | Other methods | Efficiency | Status |
|----|----------|-----------|----------------|---------------|------------|--------|
| C0 | canonical transition label | (dt, E, ψ, vz, loss, …) | `CapLabel::Transition` — ONE struct, fields split by role: compatibility (kind, u/v, ψ, vz, sf) vs quality (dt↓, s2↑, loss2↓) | — | ~ns | `[x]` unified s32; emitted by BOTH kernels (C7 capxfer 63 labels, C6 capmat 38,231 labels) |
| C1 | local Pareto comparison | label partial order | `CapLabel::Dominates` — CELL-LOCAL strict dominance (compatibility fields must share a cell at the caller's pitch; no uncertified cross-state comparison) | scalarization (banned as truth) | ~ns/pair | `[x]` s32; property gates over 38k emitted labels: irreflexive, 0 antisymmetry / 0 transitivity violations (capmat) |
| C2–C4 | quality projections (board/ride/air) | label views | pure projections of C0 | — | ~ns | `[ ]` after C0 |
| C5 | successor-requirement matching | B14 vs exit label | interval test | — | ~ns | `[d]` after B14 |
| C6 | sampled board→exit kernel | A22 matrix | **the capmat production loop: per boarding row ONE A20 surface, then the full time-resolved matrix (every horizon × every in-plane cell) through the A22 indexed query** | on-demand + memo | **v1 VERIFIED (capmat 9/0, s32): 715,057 REAL pairs at 270 µs/pair amortized = 10⁶ pairs in 4.5 min (target met, no extrapolation); queries alone 3.9 µs mean; builds dominate (~16 s/row)** | `[x]` v1 s32; soundness ridealongs: indexed-never-beats-exhaustive 1800/1800, engine witnesses 22/24 (2 boundary fringe, counted); cheaper builds = the named lever |
| C7 | sampled exit→board kernel | A12/A13 matrix | **the capxfer composition: A1 vertical timetable → A2+A3 HULL-EXPANDED face slices → B13 O(1) culls → batch A11 → A4 kernel roll predicting the contact EXACTLY (engine clip fraction (d1−1/32)/(d1−d2) + clipped slide) → engine verification** | reach-table per start (measured 4–5 orders slower to first answer); shooting | **v2 VERIFIED (capxfer 8/0, s32): contact tick EXACT 63/63, end position dev ≤ 1e-4 u; A18 law bitwise at all 63 contacts; 61/63 board on the requested tick (2 curved brake paths board early — predicted exactly, counted); organize ~3.5 ms/start, ~100 µs/target fast path** | `[x]` v2 s32; v1's 1.8–3.7-tick systematic was exactly the A3 offset; interior-coverage amber inherited from A11 |
| C8 | composition without re-simulation | memo keys | memoized exact results (repeat-share measured 84–85%) | — | hit ~ns | `[~]` measured; production memo not built |
| C9 | min-plus path composition | tick costs | standard shortest-path over labels | — | ~µs/graph | `[ ]` trivial once C0 exists |
| C10 | route cost = ticks | g | by decree | — | — | `[x]` |
| C11 | search-order heuristics | any | quarantined: order only, never truth | — | — | `[x]` rule standing |
| C12/C13 | refinement priority / value-of-information | bound gaps | largest-gap-first over certified intervals | — | ~ns/candidate | `[ ]` |
| C14 | proof ledger | cull records, version stamps | record per hard prune | — | ~ns/record | `[~]` stamps exist; per-cull records not unified |
| C15 | route reconstruction | witness chains | concatenate exact schedules, replay bitwise | — | ~ms/route | `[x]` mechanism proven (cold bitwise replays) |

## The terms table (laws behind the columns)

| Term | Exact law / source | Consumed by |
|------|--------------------|-------------|
| s² | Δ = 900 − c² per cap-limited tick (law identity: 1M-state certificate) | A6, B6, tables (stratify in s²) |
| c = s·cosα_true | the one control scalar; stored↔true bridge = negation (measured) | A4, A11/A12, A19 |
| a (applied accel) | min(cap − c, 562.5·sf) | A4, braking analysis (B22) |
| δψ | atan2(a·sinα, s + a·cosα); free-turn atan(30/s) | A6/A7, A11 family |
| step, arc length | step = post-update speed × dt; max-gain L(N) deterministic | A8/B2, A11 |
| reversal times | ≥6 apart (dwell); ≤3 at optimum (measured) | A11/A12 parameters |
| z, vz | closed form (±6 halves) | A1, B0, A13, A27 |
| sf (surface friction) | 1 falling / 0.25 rising; budget-only; STALE (this tick uses last tick's value). **AIR-KERNEL DOMAIN LAW (s35): the kernel ctx carries a FIXED sf — on rising ticks a BRAKING wish's budget 562.5·sf diverges from the engine (measured ~250 u/s in one tick); max-gain wishes (add ≤ ~30) never reach the budget. Kernel rolls with braking wishes are valid on falling flights, or must carry sf state** | A18/A19, braking regime, every kernel-roll consumer (A11–A13, A28, C7) |
| v·n, loss, post² | clip laws (dev ≤ 0.0034) | A15–A17, C2 |
| in-plane gravity | constant per face | A18–A22 |
| jump impulse | double-precision constant × stamina scale (decoded) | A30, A25 |
| duck shift / crop | 8.5u air shift; 0.34 speed crop (decoded) | A29 |
| basevel / gravity_scale | trigger-applied state (mirror carries it) | A31 |
| the heading laws (s39) | heading is ONE-TICK FREE at s ≤ B = airaccelerate·maxspeed·dt·sf = 562.5·sf (the full-brake wish a = min(30+s, B) reverses; intermediate wishes sweep all headings); above B: one-tick turn ≤ atan(B/(s−B)) — certified sound, measured 93.3% tight; speed sheds at most B per tick | A8 direction-aware UB, B22, route heading feasibility |
| the contact law (generalized, s36) | a tick contacts iff gap + (v_move·n)·dt ≤ 0 — the measured −1/32 epsilon is its gap = 1/32 special case; the carried gap = n·pos − d_exp of a bitwise-tracked position (no separate recurrence needed); certified 3600/3600 mixed ticks incl. hover + re-contacts (capgap) | A18–A20 boundary regime, A22 v3, exact ride simulation |
| E = s² + vz² | conserved less loss at clips | B12, C0 |
| END box | Minkowski intersection, feet-z window | A27 |
| hull offset off(n) | Minkowski support of the negated hull: −Σ nᵢ·(nᵢ>0 ? minᵢ : maxᵢ); traces run on d + off(n) | A3, C7 targeting, every origin-space plane prediction |
| trace clip fraction | f = (d1 − 1/32)/(d1 − d2); the end sits EXACTLY 1/32 above the expanded plane along n; clipped velocity slides the remaining (1−f)·dt | C7 contact prediction (verified to ≤1e-4 u), A18 boundary work |

## Change log
- 2026-08-23m (session 40): **THE LEGO RULE (user directive) enters
  the binding semantics, and its first application beats every
  tuned configuration.** The rule: multiple approaches per
  algorithm are FIRST-CLASS units; never edit approach A into
  approach B — compose them as ordered fallback portfolios (later
  strategies run only while unsolved; evaluations update only on
  improvement → MONOTONE by construction, coverage = the union);
  "improved X but regressed Y" is never a tradeoff — it signals a
  missing composition; measured coverage is pinned as HARD FLOOR
  GATES that only ever rise. Applied to A11's exact-hit stage: the
  proven 2×2 tail, the 3-parameter least-norm tail, and the
  post-brake-sized tail now stand as three intact strategies in
  fallback order — and the portfolio's coverage EXCEEDED every
  individual configuration on every cell (400: 99/100/100; 800:
  91/100/96; 1400: 93/98/100 — misses 127 → 23 of 900, with two
  cells above all constituents). Enforcement armed: NINE per-cell
  floors in capp2p (now 30/0) and two solver-coverage floors in
  capxfer (both ramps 37/49, now 10/0). Session 38's compromise
  hybrid is retired; its lesson (the s38 change-log tradeoff
  language) stands corrected by this entry. All twelve suites
  green. Rule recorded in the library's binding semantics (#8) and
  the standing memory.
- 2026-08-23l (session 39): **THE LAST A AMBER RESOLVES — A8's
  direction-aware bound family lands as two certified lemmas and a
  turn-gated UB (capreach 8/0).** The HEADING-FREEDOM LEMMA: at
  s ≤ B = 562.5·sf, one tick reaches any heading — the full-brake
  wish a = min(30+s, B) carries the velocity past reversal
  (constructive witnesses 4/4, and the arithmetic confirms the law
  exactly: from s=555 the backward speed is 7.50 = 562.5 − 555).
  The TURN-BOUND LEMMA: above B, one tick turns at most
  atan(B/(s−B)) — unbeaten by 200k adversarial one-tick actions and
  measured 93.3% TIGHT. The TURN-GATED TRAVEL UB gates the blind
  integral by the minimum heading-rotation time on the certified
  minimum-speed path: sound for every φ, a strict 3–5% tightening
  for backward directions at speed, unbeaten by 20k adversarial
  schedules per config (best adversaries reach only 13–39% — the
  honest slack line). The scoping result matters most: direction-
  aware bounds can ONLY improve on the blind UB above the budget —
  below it heading is provably free. The lemmas enter the terms
  table (feeding B22's future heading-aware ceiling); local
  refinement stays documented on-demand (no consumer). ALL TWELVE
  suites green. **The A category now stands complete with no open
  ambers beyond A11's minor multi-brake note.**
- 2026-08-23k (session 38): **THE A11 INTERIOR AMBER FALLS FROM
  14.1% TO 4.9% MISS** — a measured-decision session end to end.
  The transplants from A9: brake STRENGTH as a per-candidate
  continuous lever (stage-1 seeds at 0.9 and 0.6, strength climbs
  in refinement), the BRAKE-FIRST + post-brake-reversal shape class
  (the deep-interior misses' anatomy: hard early brake into the
  slow regime where turning is fast — a class the family never
  enumerated), K widened 6→8, and the **3-parameter least-norm
  rescue tail**: (tail c1, tail c2, brake strength) against the 2-D
  endpoint via Jᵀ(JJᵀ)⁻¹ — as a RESCUE only, after the proven 2×2
  starts fail on a brake candidate. Two negative results kept the
  design honest and are recorded: post-brake tail-sizing (A9's fix)
  REGRESSED here — the overlapping tail acts as a useful hybrid
  shape in this solver's flow — and always-on 3-parameter stepping
  destabilized long-N solves the 2×2 owned (400/N90 100→91) before
  the rescue-only gating restored them. Sufficiency map (was →
  now): 400: 88/99/100 → 96/100/100; 800: 67/97/93 → 88/99/95;
  1400: 48/94/87 → 83/98/97. Downstream: capxfer's 65° transfers
  29 → 35 solved of 49. All eleven suites green (capp2p 21/0
  gates unchanged — the sufficiency lines are the measured
  falsifiers they were designed to be).
- 2026-08-23j (session 37): **A29 AND A31 CLOSE — EVERY A ROW IS NOW
  CLOSED.** The debt rows land on the machinery decoded long ago,
  with the laws finally gated independently (capdebt 7/0, new
  suite). A29: the air press/unduck origin shift measured at
  EXACTLY ±8.5000u — one constant across all events — instant with
  hull 1 on press and the transient hull 2 on unduck; the shared
  timer drains exactly dt·1000 per steady tick (162/162); the
  grounded 0.34 crop's terminal speed is exactly 85.00; and the
  exact mixed roller at HULL 1 predicts ducked-flight first
  contacts 15/15 against the engine — the session-36 roller
  generalizes across hulls. A31: `CapTrigger::ApplyHit` (the
  post-move application as one pure transform) bitwise on all three
  trigger types' touch ticks, and the FULL COMPOSITION — per-tick
  kernel (ctx basevel + gravity_scale) + re-touch + ApplyHit, with
  pushes accumulating while the flight stays inside the volume —
  matches the engine bitwise for 15 post-touch ticks on every
  flight. **The A category stands complete: A0–A31 closed, A32
  excluded by decree; the two remaining named ambers (A8
  direction-aware UB + local refinement, A11 short-flight interior
  coverage) live on rows already certified with measured lines.**
  Battery green: capdebt 7/0, capgap 6/0, capkern 5/5, capboard
  5/5, capground 6/6, capwindow 6/6, capcontact 5/0, capmin 8/0,
  capxfer 8/0.
- 2026-08-23i (session 36): **THE CARRIED-GAP LAW IS EXACT — A18
  completes, A19/A20's named refinement delivered, A22 v3.** The one
  regime only the full engine could walk — the ride boundary: the
  carried 1/32 standoff and its float drift, hover ticks,
  re-contact fractions — closes BY COMPOSITION of certified pieces:
  `CapRide::MirrorTick` = the airborne chain preamble under
  STATEFUL stale sf (the exact categorize rule: reset to 1, probe
  only at vz ≤ non_jump_velocity, air_friction_up on rising
  no-walkable) + the A24 local move mirror. **capgap 6/0: 3600/3600
  mixed contact/hover/re-contact ticks BITWISE (position, velocity,
  AND sf) across two ramps — 842 hover ticks, 327 re-contacts —
  and the GENERALIZED CONTACT LAW verified on every tick: a tick
  contacts iff gap + (v_move·n)·dt ≤ 0** (the measured −1/32
  epsilon is its gap = 1/32 special case; the carried gap needs no
  separate recurrence — it IS n·pos − d_exp of a bitwise position).
  A22 v3: the exact-hit tail on the ride plane — a 20-tick C- or
  S-curve tail with the SPEED-CORRECT live-band mapping (true-cos
  beyond cap/s is dead control authority — measured, the flat
  mapping wasted half its range), Newton through the mixed roller:
  20/20 in-authority targets recovered to mean 0.097u (the lattice
  answered ~30u), 8/8 engine-bitwise, beyond-authority targets
  DECLINE to the lattice+N choice. The tail-authority ellipse was
  measured en route (same-side shapes are near-1D — the S-curve
  spans 2D). Battery green: capgap 6/0 (new), capkern 5/5,
  capboard 5/5, capground 6/6, capwindow 6/6, capcontact 5/0,
  capmin 8/0, capxfer 8/0.
- 2026-08-23h (session 35): **A23, A24, A28 close — and the LOCAL
  MOVE MIRROR arrives.** A24's first design (two analytical transfer
  laws) was REFUTED by its own census: valley-crease crossings
  produced three-plus-bump chains (17 of 36) no two-plane law
  covers. The answer is structural: **`CapEdge::TryMoveLocal` — the
  verified TryPlayerMove transcribed as a PURE LOCAL FUNCTION over
  the bitwise A14 clip** (partial-move rebases, crease resolution,
  stop-dead guard, allsolid zeroing, the unswept stuck guard) —
  36/36 crossing ticks matched BITWISE in position AND velocity
  (capmin 8/0). A28: `CapContact::FirstContactOnPath` (the A12+A14
  composition) — 24/24 solved paths agree with engine replays, 16
  clean tick-for-tick and 8 obstructed at the exact tick and brush
  (capcontact 5/0); en route, the two-ramp world was rebuilt with
  FINITE segments after the first build's backing slab engulfed the
  spawn region (a wrong-world lesson, caught by the zero-clean
  falsifier), and a VALLEY was relearned to be non-convex air — two
  wall brushes, never one. A23: `CapRideReach::EdgeIntercept` —
  25/25 edge samples equal the brute scan, 8/8 witnesses validated.
  **NEW DOMAIN LAW (terms table, sf row): the air kernel's ctx
  carries FIXED surface friction while the engine's is STALE-
  STATEFUL (0.25 on rising ticks) — a BRAKING wish's accel budget
  562.5·sf diverges ~250 u/s in one rising tick (measured), while
  max-gain wishes never reach the budget. Kernel rolls with braking
  wishes are valid on FALLING flights** — the flight gates now
  respect the domain, and the stuck-guard predictor (the d34
  freeze: silent velocity zero at (0,0,−6)) is armed in the A14
  flight gate. Battery green: capcontact 5/0, capmin 8/0, capsolve
  10/0, capxfer 8/0, capkern 5/5, capboard 5/5, capground 6/6,
  capwindow 6/6.
- 2026-08-23g (session 34): **A9 and A14 close — the march continues.**
  A14 (capcontact 4/0, new suite): `CapContact` runs the engine's own
  per-brush clip verbatim over a corridor-gathered local set —
  40,000/40,000 queries BITWISE identical to the full trace across
  one- and two-ramp worlds (fraction bits, brush, plane, startsolid),
  the corridor DECLINE law clean (6,742 refused honestly, every
  answer bitwise), and 25/25 kernel-flight first contacts (tick AND
  brush) matching real engine replays; 60–74 ns/query. A9 (capsolve
  10/0): `CapP2P::SolveTerminal` — endpoint + terminal-heading
  requests solved through an iterated design driven by measured
  falsifier feedback: brake placement AND strength as heading/range
  levers, a scored coarse climb judging both contracts per kernel
  roll (heading-blind climbing measurably walked into far basins;
  heading-only judging measurably stranded 50u out), the recorded
  paired-translation lesson, multi-start exact-hit tails, and **the
  3×3 finisher — (tail c1, tail c2, brake strength) against
  (endpoint x, y, heading), a square Newton** that took acceptance
  2→19 of 40 against the most adversarial generator possible (random
  braked schedules' exact terminal states). Contracts are HARD:
  19/19 accepts re-verified independently (endpoint ≤ 0.5u, heading
  ≤ 5°, mean 0.49°), 8/8 engine-bitwise; the 21 DECLINEs are the
  honest family-sufficiency measure and the named refinement
  (second brake run / A8 Pareto layer). Battery green: capsolve
  10/0, capcontact 4/0, capmin 6/0, capxfer 8/0, capkern 5/5,
  capboard 5/5, capground 6/6, capwindow 6/6.
- 2026-08-23f (session 33): **THE A MARCH OPENS (user directive:
  finish ALL of category A before more composition) — six A rows
  close: A0, A10, A12, A13, A21, A26.** Two new suites: **capsolve
  7/0** (A0 `CapFrame` bitwise vs the inline math 40k/40k, round
  trip ≤ 0.0016u, rotation-commutation of full rollouts MEASURED at
  0.0002u worst — the world-frame witness rule stands on measurement,
  not assumption; A12 `SolveFreeN` 36/36 preculled == brute with
  10/10 engine-bitwise replays; A10 `MinAirTime` 24/24 == brute;
  A13's rotated-azimuth scenario 12/12 exact contact predictions on
  the general slice geometry) and **capmin 6/0** (A21 `MinRideTime`
  50/50 == the brute per-layer scan with 12/12 engine-validated
  minimum-time witnesses; A26 `CapLaunch::Detector` — the launch
  threshold bracketed to ONE constant across approach speeds,
  (270.7, 272.4] ∋ 256+16 = the A3 hull-overhang law confirmed by
  measurement, captures continue bitwise, first clean airborne tick
  matches A1 bitwise). capxfer refactored to CONSUME the A13 package
  (`CapFaceSolve`) and stays 8/0 — one implementation, its gates the
  acceptance tests. BUG FOUND AND FIXED by A12's small-N scan: the
  brake-probe stride N/8 was 0 for N<8 — an infinite loop latent in
  BOTH SolveFixedN and SolveTargetBatch since session 28 (no prior
  caller used N<8); both sites clamped, capp2p/capxfer/capmat green
  after. Remaining open A rows, the march order: A9, A14, A22
  exact-hit tail, A23, A24, A28, A19/A20 carried-gap law, A8
  refinements, A11 interior coverage, A29, A31.
- 2026-08-23e (session 32): **FIVE ROWS CLOSE — A3, C0, C1, C6, C7
  v2.** A3 packaged (`CapHull::PlaneOffset`, float-identical to the
  world loader's expansion, gated bitwise 48/48 in capxfer) and its
  first consumer collapses C7 v1's early-contact systematic: targets
  now sit on the HULL-EXPANDED plane, and the A4 kernel roll plus the
  engine's own clip law — fraction (d1−1/32)/(d1−d2), end exactly
  1/32 above the plane, clipped slide for the remaining time —
  predicts every contact EXACTLY: **capxfer 8/0, contact tick exact
  63/63, end position within 1e-4 u, A18 bitwise at all 63 contacts**
  (2 curved brake paths board early — predicted exactly, counted
  honestly). C6 v1 VERIFIED (**capmat 9/0**): per boarding row one
  A20 surface, then the FULL time-resolved matrix through the A22
  indexed query — **715,057 real pairs at 270 µs/pair amortized =
  10⁶ pairs in 4.5 min** (target met without extrapolation; queries
  alone 3.9 µs; builds ~16 s/row are the named lever), with
  soundness ridealongs (indexed never beats exhaustive 1800/1800;
  engine witnesses 22/24, 2 boundary fringe). C0 unified
  (`CapLabel::Transition`, compatibility vs quality fields) and
  emitted by BOTH kernels (63 + 38,231 labels); C1 cell-local strict
  dominance property-gated over the emitted set (irreflexive, 0
  antisymmetry, 0 transitivity violations). Regression green:
  capkern 5/5, capboard 5/5, capground 6/6, capwindow 6/6.
- 2026-08-23c (session 31): **C7 v1 VERIFIED (capxfer 4/0) — the
  first true composition**: an air exit state answering boarding
  targets on a destination ramp through the full chain (A1 vertical
  timetable → A2 face slices → B13 constant-time culls → batch A11 →
  engine verification). Transfers land within 1.8–3.7 ticks and
  13–24u of prediction (the hull's leading-edge early contact is the
  measured systematic — A3's offset, the v2 anticipation);
  **the A18 ride law held bitwise at all 62 contact ticks on
  arbitrary incoming flights**. Method tested, not assumed: the
  reach-table alternative costs 4–5 orders of magnitude more per
  start to its first answer (measured s29 vs s30 numbers). ALSO: the
  interactive dashboard `Docs/CapabilityChecklist.html` opened — the
  same rows searchable/sortable with cross-linked terms,
  dependencies, and click-open detail drawers (72 rows, 16 terms,
  update alongside this file).
- 2026-08-23b (session 30): **the organize-once/query-constant
  session.** A11 gains BATCH MODE (`CapP2P::P2PBatch`): the family's
  law endpoints hashed spatially once per start (0.1–1.4 ms), then
  **34–135 µs per target** (99–100/100 hits, full-solver fallback as
  the completeness backstop) — inside the original 10–30 µs ambition
  at short flights, 20–60× faster than single-shot; capp2p 21/0.
  A22 gains the O(1) indexed query (`QueryTargetFast`, per-layer
  cell index): **0.2–0.3 µs per query** (~10,000× faster), answers
  at lattice resolution (mean ~30u; the full scan's nearest-node
  0.7u remains available); capexit 9/0. NEW CERTIFIED CULLS: B13
  (min departure resource — closed-form ceiling + travel bisection;
  24 A11-solver attacks from below, 0 refutations) and B14 (min
  incoming resource at a face — the EXACT piecewise-quadratic
  infimum, after the falsifier refuted the monotone-bisection draft
  at 20/127 and then exposed a missing zero-loss-band root at
  22/185: post-board speed is NOT monotone in horizontal speed;
  slow-horizontal fast-vertical arrivals retain more); capwindow
  6/6. Battery green: capkern 5/5, capboard 5/5, capground 6/6,
  capride 7/0.
- 2026-08-23 (session 29): **A20 + A22 v1 verified** (capexit 9/0);
  A18 position law measured (bitwise on 992/1090 steady ticks, worst
  deviation 3.5e-5 u — the carried-gap micro-fraction, folded into
  capboard as a fifth gate); the contact-epsilon discovery written
  into the PERMANENT records (CapabilityLibrary A18 row as a
  standing law + the CapabilityMethods terms catalog). The ride
  reach sweep needed its own pitch (the air pitch hit 4.2M
  cells/layer and aborted — ride speeds fan positions faster);
  witnessed-bound semantics upgraded: each reach direction's bound
  is ENGINE-VALIDATED or explicitly declined (upslope at 50° — the
  near-walkable near-stall corner — declines; 60°/70° validate 8/8
  with zero fringe). A22 v1 answers engine-reachable targets from
  the sweep at sub-3.3u residual. No drift: capboard 5/5 (now with
  the position gate), capride 7/0.
- 2026-08-22/23 (session 28): **A19 built on the proven ride law**
  (capride 7/0): engine-anchored starts (along-downslope boarding —
  head-on entries lose everything to the clip, measured), composed
  surface-friction state, and the discovered **contact-epsilon
  physics**: the rider hovers 1/32 above the plane, so re-contact
  needs v·n ≤ −2.08 u/s — the hover band was producing 10/10 witness
  divergences until guarded, then 30/30 witnesses replay bitwise as
  engine continuations. Ride physics measured: 450 u/s boarding →
  1322 u/s after 0.9 s of 50° downslope; ±30° steering nearly free.
  Falsifier ambers recorded (boundary-surfing trajectories up to
  +215.6 — the conservative guard's sliver, the named refinement).
  **A27 + B3/B4 certified** (capwindow 4/4): the vertical recurrence
  bitwise vs the engine (incl. clamps), window logic = naive scans,
  the contact window never beaten by 915 real schedules, the
  END-crossing predicate = naive with 12.1× fewer tests. Battery all
  green: capkern 5/5, capboard 4/4, capground 6/6, capp2p 12/0,
  capwindow 4/4, capride 7/0, groute 11/11.
- 2026-08-22 (session 27): **A11 VERIFIED** (capp2p 12/0) — the
  exact-segment-composition enumeration (robotics motion-primitive
  concatenation on our spirals; exhaustive k≤2) fixed the machinery
  gate at 99–100/100 everywhere; the stored-vs-true wish-basis sign
  and a stale incremental build were the two failure causes found on
  the way (full-rebuild discipline noted). **A25 + A30
  BITWISE-CERTIFIED** (capground 6/6): the flat-ground walk law and
  the jump law packaged as kernels, 300k parity ticks total with 0
  mismatches, ground kernel 10.9× engine; the settled hull measured
  one trace epsilon (1/32) above the hull-expanded floor plane. No
  drift: capkern 5/5, capboard 4/4, groute 11/11.
- 2026-08-22 (session 26): A11 implemented and iterated v1→v3 under
  its own falsifiers. v1: own-family targets 55–87% hit, general 0%
  — the max-gain family provably lives on a thin arc-length shell.
  v2: +brake-run range control (true wish opposing velocity sheds
  speed, shortening the path) — general targets 0→13–97%. v3:
  paired-translation refinement, top-3 candidates, tail on top-3 —
  machinery 92–98%, general up to 99% (long flights), 29–30% (short
  flights). All physics gates green throughout: every emitted
  schedule dwell-legal, 145/145 solved schedules replay through the
  real engine with bitwise-identical endpoints. The suite stays RED
  on the 99% machinery gate; the measured diagnosis is initializer
  quality (coarse grid + local climbing stalls in neighboring
  basins), and the designed continuum closed-form initializer is the
  named fix — it also carries the 10–30 µs cost target (replaces
  ~1300 grid rollouts with a handful).
- 2026-08-22 (session 25): table opened; gap rows added (A29 duck,
  A30 jump, A31 triggers, A32 exclusions, B22 heading-aware ceiling);
  efficiency figures from this week's measurements; A11/A12 marked
  in-build.
