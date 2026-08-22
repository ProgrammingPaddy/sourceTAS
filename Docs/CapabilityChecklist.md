# The Capability Checklist — master table

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
| A0 | canonical frame transform | position, heading | every table/family | O(1) rotation+translation; tables canonical, witnesses world-frame | — | ~ns | `[~]` implicit in builds; formal float-semantics note done, helper API not extracted |
| A1 | vertical state z(N), vz(N) | z, vz, gravity halves | arrival windows (B0), A13, A27 | closed form | per-tick iteration (pointless) | ~ns, O(1) | `[x]` |
| A2 | face slice λ(z) | λ, face plane | A13 targets, B1 | plane/polygon algebra | — | ~ns, O(1) | `[x]` |
| A3 | hull contact geometry | hull boxes, support offsets | A14, A26, A27 | Minkowski offsets per plane (engine's own d_stand/d_duck) | analytic contact regions per face | ~ns/plane | `[~]` offsets exist in World; face contact-region API not packaged |
| A4 | one-tick air law | c = s·cosα_true, s², a, δψ | everything airborne | closed law (TickLaw) + bitwise kernel | full MoveTick (9× slower) | kernel 16.0M ticks/s = 62 ns/tick; law match: worst Δs² 5.0, Δψ 1.7e-5 rad over 1M states | `[x]` capkern 5/5 |
| A5 | dwell automaton | dwell age, min_gap 6 | all schedule legality | counter + comparison | — | ~ns | `[x]` guarded by the lattice-agreement gate |
| A6 | N-tick turn/gain max V* | s², ψ, reversal count | B5/B6, C5, transfer valuation | closed-form ceiling √(s0²+900N) for bounds; band table for full surfaces | parametric family (via A12 machinery, future); shooting oracle | ceiling O(1) ns; band build 48–276 s/v0, then O(1) query; gaps: sampled-action 0, continuous +20…+93 u/s measured | `[x]` as certified interval; `[ ]` heading-aware UB (B22) |
| A7 | dual Ψ* (max turn at speed floor) | ψ, V_min | B5, approach planning | read A6 layers | — | O(bins) µs | `[x]` with A6 |
| A8 | directional reach D* | arc length L(N), projections | B2, B13, corridor checks | certified bounds (single query); on-demand sweep (batch) | parametric family endpoints (future, exact); finer local refinement | bounds O(N) ~µs; sweep 31.1M nodes / 101 s at pitch (96, 24), N=60 | `[x]` LB+UB at measured pitch; `[ ]` direction-aware UB, local refinement |
| A9 | displacement with terminal constraint | endpoint + (ψ_N or V_N) | A22 pairing | A12 family with terminal constraint added | Pareto layer of A8 sweep | target ~10² µs | `[d]` |
| A10 | minimum air time N_min | N window, feasibility | route timing, B21 | O(1) feasibility scan over the A1 window + A12 solve at first feasible N | table lookup | target <1 ms | `[d]` — build with A12 |
| A11 | fixed-time point solve | reversal times, brake run, exact-hit tail, segment tables | A12 core | **EXACT SEGMENT COMPOSITION** (the all-plus spiral's prefix tables give any reversal schedule's endpoint in O(k), no trig — exhaustive k≤2, dense k=3) + brake-run range control + kernel refinement + exact-hit Newton | analytic inversion (the remaining speed refinement); shooting oracle; reach-table batch | **v4 VERIFIED (capp2p 12/0): machinery 99–100/100 everywhere (worst residual 0.25–2.3u); general targets 29–99% (interior at short flights = the brake family's measured limit); 0.6–3.3 ms/solve; 150/150 bitwise replays** | `[x]` machinery; `[~]` short-flight interior coverage (measured line) |
| A12 | point-to-point air solver | all of A10/A11 | C7, gap feasibility (B13/B14), A22 pairing | A11 iterated over N | shooting oracle (cross-check); R_N table (batch) | inherits A11 + N-scan | `[d]` after A11 goes green |
| A13 | point-to-face solve | (T, λ) targets | entrance/transfer | A12 at 1-D λ targets per candidate tick | generic 2-D targeting (wasteful) | target ~10× A11 per face | `[d]` |
| A14 | first-contact prediction | trace, local brush set | A28, event scans | exact trace vs pruned local brush set | full-world trace (the engine's own) | engine trace ~µs; local-set target ~100 ns | `[~]` engine path exists; local-set pruning not built |
| A15 | board clip | v·n, loss, post² | A16/A17, C2 | exported engine clip + closed post-speed law | — | ~ns; law dev ≤ 0.0034 u/s over 33k arrivals | `[x]` |
| A16 | best/worst board | v·n extrema over heading | C2, B8/B9 | closed-form candidates (endpoints, cos extrema, zero crossings) | dense enumeration (baseline only) | ~ns (few trig); verified to 1e-4 over 640 configs | `[x]` |
| A17 | board feasibility arcs | loss ≤ L heading arcs | B8, approach windows | closed-form arccos arcs (≤2 per period) | enumeration | ~ns; 0 mismatches / 24×7200 | `[x]` |
| A18 | one-tick ride law | in-plane gravity, sf state | A19–A22 | proven composition: gravity-half → clamp → accel(sf) → one clip → clamp → gravity-half → clamp | full MoveTick (edges/multi-plane keep it) | same order as A4 kernel + clip; 1246/1246 bitwise | `[x]` interior; `[ ]` edge/multi-plane ticks stay full-engine |
| A19 | ride N-tick turn/gain | slope vector vs heading | A22, B10/B11 | A6 treatment on the A18 law (one added parameter) | table on the plane | target: ceiling O(1) + build ~minutes | `[ ]` next after A12 |
| A20 | ride directional reach | in-plane (λ, downslope) | A22, B10 | A8 treatment on the plane (2-D positions) | — | target: bounds O(N); sweep ≪ air (2-D) | `[ ]` |
| A21 | minimum ride time | exit point, N_min | route timing | inverse of A19/A20 (scan + solve) | — | target <1 ms | `[ ]` |
| A22 | board→exit solver | entry state × exit point | C6, the two-ramp transfer | ride point-to-point: the A12 construction on the plane with in-plane gravity | DP table per face (baseline/falsifier) | target 10–100 µs/pair | `[d]` — the ride's A12 |
| A23 | ride edge interception | edge geometry, exit λ | launch selection | A20 bounds + A22 solve at edge targets | — | target ~A22 | `[ ]` |
| A24 | direct contact transfer | adjacent-face clip chain | face-to-face routing | exact clip transform at the shared edge | full engine roll-through | ~ns/edge | `[~]` clip exists; adjacency composition not packaged |
| A25 | ground/runoff acceleration | friction, stopspeed, accelerate, stamina power curve | spawn/launch prep | `CapGround::WalkKernelTick` — the grounded MoveTick chain partially evaluated on the flat-floor domain | full MoveTick baseline | **40.8M ticks/s = 10.9× engine; 200k walk ticks + 50k position checks, 0 bitwise mismatches (velocity, feet z, stamina)** | `[x]` capground 6/6 (s27) |
| A26 | edge launch | leave-ground event, launch state | route segments | event detector on exact rollouts | — | ~engine-tick cost per scan tick | `[~]` instruments exist (LaunchWitness/ReplayLaunch); API not unified |
| A27 | END-volume crossing | Minkowski box, z-window | finish timing | per-tick 1-D interval test from A1 window + box | full trace | ~ns/tick | `[~]` box computed; packaged predicate pending |
| A28 | obstacle/corridor traversal | first contact along family | route validation | A14 local-set checks along A12 candidate paths | swept-volume precomputation | target ~µs/leg | `[ ]` last in air chain |
| A29 | duck capability (NEW, gap) | duck flag/hull/timer, 8.5u shift, 0.34 speed crop | duck-required routes, A3 hulls | package the decoded duck state machine as LAW + hull switch | — | ~ns/tick law | `[ ]` currently domain-stamped OUT of all air/ride families — the stamp is the gap record |
| A30 | jump capability | jump impulse (double const), stamina scale, autobhop gate, three gravity half-steps | spawn runs, bhop segments, launches | `CapGround::JumpKernelTick` (jump head + the A4 air chain) | — | **50k jump ticks, 0 bitwise mismatches on all channels + stamina; vz law: stamina 0→283.993, 1315.8→210.839; release gate + autobhop bypass verified** | `[x]` capground (s27); SET path (ducked) = A29 debt |
| A31 | trigger interactions (NEW, gap) | basevel, gravity_scale, teleport | maps with push/teleport/gravity | mirror the trigger application rules as transforms | — | ~ns/event | `[ ]` engine mirror carries state; capability + tests not built |
| A32 | water / ladders (exclusion) | — | — | — | — | — | `[-]` excluded: surf maps in scope have neither; revisit only if a target map does |

## B — certified reducers (prune only with proof)

| ID | Reducer | Key terms | Feeds | Primary method | Other methods | Efficiency | Status |
|----|---------|-----------|-------|----------------|---------------|------------|--------|
| B0 | vertical tick window per face | z window | sweep/search culls | closed form (A1+A2) | — | ~ns | `[x]` deployed |
| B1 | face-slice validity | λ interval | target filters | closed form (A2) | — | ~ns | `[x]` |
| B2 | travel bound (replaces the refuted cone) | max-speed integral | reach culls | Σ√(s0²+900i)·dt upper bound | direction-aware version (B22-adjacent, future) | O(N) ~µs, precomputable prefix sums → O(1) | `[x]` certified; forward-tight to 1.0u measured |
| B3 | earliest contact tick | B0 × B2 | search windows | interval intersection | — | ~ns | `[~]` composition wired ad hoc |
| B4 | latest useful contact tick | z window + geometry | search windows | closed form from A1 + face extent | — | ~ns | `[ ]` |
| B5 | heading-change feasibility | Ψ* | approach culls | A6/A7 lower bounds ADMIT only | heading-aware UB (B22) would enable culls | O(1) | `[~]` admit-only until B22 |
| B6 | terminal-speed feasibility | speed ceiling | gap culls | required V > √(s0²+900N) ⟹ impossible | — | O(1) ~ns | `[x]` first certified air cull |
| B7 | point/region reachability | B0×B2 + A8 | candidate filters | interval tests then A8/A12 | R_N membership (batch) | ~ns then µs | `[~]` |
| B8 | boardability bound | A17 arcs | arrival culls | closed-form arcs | — | ~ns | `[x]` via A17 |
| B9 | minimum unavoidable board loss | min v·n | valuation floors | closed form (A16) | — | ~ns | `[x]` via A16 |
| B10 | ride point/edge reachability | A19/A20 bounds | ride culls | ride analogues of B2/B6 | — | target ~ns | `[ ]` with A19/A20 |
| B11 | minimum ride ticks | A21 | timing bounds | inverse ride bounds | — | target O(1) | `[ ]` |
| B12 | ride energy ceiling U_E | E = s²+vz² | ride culls | certified (session 15) | — | O(1) | `[x]` |
| B13 | minimum departure resource for a gap | A12 inverse | exit valuation | invert the A12 feasibility over v0 (monotone bisection) | table | target ~µs | `[d]` after A12 |
| B14 | successor's minimum incoming resource | backward A12/A16/A22 | the speed-compounding explainer | compose B13 with board loss floors | — | target ~µs | `[d]` |
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
| C0 | canonical transition label | (dt, E, ψ, vz, loss, …) | fixed tuple, exact fields | — | ~ns | `[~]` fields exist across code; single struct not unified |
| C1 | local Pareto comparison | label partial order | dominance over the tuple | scalarization (banned as truth) | ~ns/pair | `[ ]` |
| C2–C4 | quality projections (board/ride/air) | label views | pure projections of C0 | — | ~ns | `[ ]` after C0 |
| C5 | successor-requirement matching | B14 vs exit label | interval test | — | ~ns | `[d]` after B14 |
| C6 | sampled board→exit kernel | A22 matrix | batch A22 over sampled pairs | on-demand + memo | target: 10⁶ pairs in minutes | `[ ]` the end-state test's left half |
| C7 | sampled exit→board kernel | A12/A13 matrix | batch A12 over sampled pairs | on-demand + memo | target: 10⁶ pairs in minutes | `[ ]` right half |
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
| sf (surface friction) | 1 falling / 0.25 rising; budget-only | A18/A19, braking regime |
| v·n, loss, post² | clip laws (dev ≤ 0.0034) | A15–A17, C2 |
| in-plane gravity | constant per face | A18–A22 |
| jump impulse | double-precision constant × stamina scale (decoded) | A30, A25 |
| duck shift / crop | 8.5u air shift; 0.34 speed crop (decoded) | A29 |
| basevel / gravity_scale | trigger-applied state (mirror carries it) | A31 |
| E = s² + vz² | conserved less loss at clips | B12, C0 |
| END box | Minkowski intersection, feet-z window | A27 |

## Change log
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
