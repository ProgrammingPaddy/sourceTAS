# The Capability Library

**Program opened 2026-08-20 (session 22), user + advisor ruling
2026-08-20f. Registry adopted 2026-08-21 (session 23), user + advisor
taxonomy 2026-08-21a.** This is the design of record for the bottom-up
capability program and the SINGLE canonical registry of every atomic
capability. The full-map solver is a requirements document and a
regression harness; the product is this library. G0R is not a
milestone.

## The governing principle

For every movement capability, answer:

> Given an exact initial state and a small number of constraints,
> what are the MINIMUM and MAXIMUM physically achievable outcomes?

Each capability is a unit function over an explicit domain, solved by
exhaustive engine-exact enumeration of the reduced domain, then
(where possible) collapsed to a proven law. Outputs may be scalar
extrema, intervals, or small Pareto relations — but every capability
gets a narrow, map-independent contract. Every proof permanently
removes dimensions from every future search. The functions outlive
any particular solver architecture.

## The three categories

- **A — engine prediction / physical capability.** "What can
  physically happen?" Exact transforms, capability extrema/Pareto
  surfaces, and their inversions.
- **B — search-space reduction / certified impossibility.** "What can
  we prove does not need searching?" Equations, table lookups,
  interval intersections, support-function tests. Nothing is promoted
  here without proof.
- **C — scoring / selection / composition.** "Which physically valid
  alternative is preferable for a stated objective?" Consumes proven
  A/B outputs; NEVER invents physics scores.

## Library-wide semantics (binding)

1. **LB admits, UB culls.** A witnessed lower-bound surface certifies
   POSSIBILITY (a real schedule exists). Only an exact or certified
   upper-bound surface certifies IMPOSSIBILITY. **No B-category cull
   may ever consume an LB.** (Matches the standing rule: only
   certified bounds hard-prune, each with proof records.)
2. **No capability may assume scalar dominance merely because its
   requested output is scalar.** (The Conjecture M lesson: a slower
   state can out-turn-then-accelerate a faster one.) Internal
   representations preserve whatever Pareto dimensions affect future
   attainment until a theorem removes them.
3. **Reference vs production implementations, cross-falsifying
   forever.** `X_Ref` is the slow full-`MoveTick` truth instrument;
   `X_Fast` is the partially-evaluated exact kernel with the SAME
   float semantics (same ops, same order, same compile flags),
   parity-gated bitwise against `X_Ref` over dense fuzz + edge corpora
   (the `wishparity` pattern). Partial evaluation of the exact engine
   under a proven domain is NOT approximation; it is the standing
   performance principle.
4. **Tables canonicalize; witnesses stay world-frame.** Canonical
   transforms (A0) are exact in the reals but rotation is not exact
   in floats, so: capability TABLES live in the canonical frame;
   every WITNESS is stored as an exact control schedule and
   re-verified bitwise in the frame where it will be used (the
   `ReplayEdge` practice — sessions 17–18 edges cold-replay bitwise
   because we store schedules, not transformed states).
5. **Domain stamps.** Every surface states its exclusions explicitly.
   Current standing stamps: standing hull only (0x1C50 size; no
   duck), no jump inside air capabilities, clean air (no water,
   ladders, triggers), dwell-6 control legality. A stamp is a
   documented boundary, not an assumption.
6. **The exact simulator is NEVER approximated** (standing law,
   session 20). Search/representation resolution is adaptive;
   simulator accuracy is not.
7. **No map geometry and no human data in capability derivation** —
   synthetic/unit domains only. Map geometry enters only through
   B/C-layer consumers after the map-independent function is solved.

## The standard deliverables (every capability, every time)

1. explicit domain and minimal input variables (cancelled terms
   proven irrelevant);
2. exact output quantity / Pareto relation;
3. exhaustive dense engine-exact surface;
4. a visual report of the function (HTML, `Docs/reports/`);
5. observed structure: monotonicity, symmetry, branches;
6. proposed analytic law or low-dimensional interpolation;
7. derivation / certified bounds;
8. adversarial falsification against the exact engine;
9. constructive witnesses for the extrema;
10. a stable callable API, map-independent;
11. **reference implementation AND production implementation with a
    standing bitwise parity gate** (`X_Ref` / `X_Fast` / `xparity`).

## The method (experimental mathematics)

A) minimal inputs -> B) brute-force the reduced domain dense ->
C) visualize -> D) propose the theorem -> E) prove from the movement
equations -> F) falsify exhaustively/randomly -> G) freeze as a
primitive and never search that dimension again.

## Implementation types

Every registry row carries a type:

- **T (closed transform):** direct O(1) exact function. No optimizer.
- **S (capability surface):** extrema/Pareto surface — densely
  enumerate, derive, prove, table, invert. The expensive builds.
- **X (composition/query):** consumes already-solved capability
  results; ideally does no simulation at all.

---

# REGISTRY A — engine prediction / physical capability

Statuses: CERT (solved/certified) · PROD (fast kernel + parity gate
live) · REF (truth instrument exists, no fast kernel) · LB (witnessed
lower bound only) · PART (partial assets exist) · TBD.

| ID | Capability | Type | Status | Existing assets | Depends on |
|----|------------|------|--------|-----------------|------------|
| A0 | Canonical-frame transform | T | PART | implicit in clean-air builds (d0/a0, origin starts); formalize float semantics per rule 4 | — |
| A1 | Vertical state z(N), vz(N) | T | CERT | closed form; deployed as the session-20 vertical-window cancellation | — |
| A2 | Face slice z -> [λmin, λmax] | T | CERT | (T, λ) destination cells (session 21) | — |
| A3 | Hull contact geometry | T | PART | standing hull 0x1C50; Minkowski END box computed (session 18) | — |
| A4 | One-tick air turn/gain | T | **CERT + PROD (s24)** | `CapAir::KernelTick`, bitwise-certified by `capkern`: 2.05M+ engine pairs, 0 mismatches on all six channels incl. vz≠0 and signed zeros; vz-decoupling of the horizontal channels certified (1M pairs); 16.0M ticks/s = **8.9× MoveTick** measured | — |
| A5 | Dwell/reversal automaton | T | CERT | min_gap = 6 law; age dimension in every table; the `dwell law matches the lattice` gate guards the param/lattice agreement (s24 review) | — |
| A6 | N-tick air turn/gain V* | S | **LB + closed-form UB (s24)** | Exact-band flat lattice completes ×6 on A4-prod (22–96M nodes, 48–276s); bitwise witness replay green; **certified UB V\* ≤ √(v0²+cap²·N)** from the accel algebra, TIGHT at free heading (379 vs 378.8 at v0 250, 491 vs 491.0 at v0 400); falsifier gaps = measured tightness; sub-cell state collapse measured | A4, A5 |
| A7 | Dual Ψ* | S | LB | `QueryPsiStarX` reads A6 layers; duality green | A6 |
| A8 | Air directional displacement D* | S | **LB + UB v1 (s24)** | R_N sweep on the kernel; pitch ladder measured (11.1M/31.1M/42M+ nodes at hp 128/96/64, N=60); D* witnesses bitwise (position AND velocity); **the s20 cone REFUTED** (61,513 schedule violations; the LB itself beat it by 190u backward) → replaced by the certified max-speed-integral UB | A0, A4, A5 |
| A9 | Displacement with terminal constraint | X | TBD | Pareto layer of R_N | A8 |
| A10 | Minimum air time N_min | X | TBD | membership query over N (A1 bounds the N range) | A8, A1 |
| A11 | Fixed-time air point solver | X | TBD | witness query into R_N at Q | A8 |
| A12 | Air point-to-point solver | X | TBD | A11 over feasible N; `RefSolve` stays demoted to oracle/polisher here | A8, A1 |
| A13 | Air point-to-face solver | X | TBD | A12 over (T, λ) slices — 1-D per tick, never generic XYZ | A8, A2 |
| A14 | First-contact air prediction | T | REF | world-first-contact sweep + `FlyZoneSchedule` (zone-tested-per-tick); prod = trace vs small local brush set. Map geometry enters HERE only | — |
| A15 | Board clip | T | CERT | analytic, overbounce 1: v' = v − (v·n)n; loss = |v·n|; post-speed law verified vs `Fn::ClipVelocity` to **0.0034 u/s worst** over 33k arrivals (s24) | — |
| A16 | Best/worst board capability | S | **SOLVED (s24)** | `CapBoard::BestWorstBoard` closed form (candidates: endpoints, cos extrema, zero crossings); 640 configs vs 7200-step enumeration, 0 disagreements, worst dev 1e-4 | A15 |
| A17 | Board feasibility (inverse) | X | **SOLVED (s24)** | `CapBoard::LossFeasibleArcs` (≤2 arcs per period); 24 configs × 7200 headings, 0 mismatches | A16 |
| A18 | One-tick planar ride law | T | **PROVEN interior (s24)** | `CapBoard::RideGrayTick`: start-gravity → clamp → air-accel(stale sf) → ONE `Fn::ClipVelocity` → clamp → finish-gravity → clamp; **1246/1246 contact ticks bitwise** across 3 ramps × 12 runs; 0 multi-plane, 0 engine-zeroed in the ride domain; velocity is frac-independent on single-plane ticks. **THE CONTACT EPSILON (s28, standing law):** the rider hovers one trace epsilon (1/32 u) off the plane, so a tick RE-CONTACTS only when the inward motion closes that gap within dt — v·n ≤ −(1/32)/dt ≈ −2.083 u/s at 66.67 tps. Inward velocities in (−2.083, 0) are HOVER ticks: legal, airborne, un-clipped (measured separation at v·n = −1.80; the exact-condition guard turned 10/10 witness divergences into 30/30 bitwise). Mixed hover/contact motion is real surf technique and richer than the pure-contact domain | A4, A15 |
| A19 | N-tick ride turn/gain V*_ride | S | TBD — **unblocked by A18** | V* machinery + one slope parameter (in-plane gravity vector vs heading); surface_friction is carried state (0.25 when rising) | A18 |
| A20 | Ride directional displacement D*_ride | S | TBD — unblocked | reach-family machinery on the plane (2-D positions: λ, along-slope) | A18 |
| A21 | Minimum ride time between points | X | TBD | inverse A19/A20 | A19, A20 |
| A22 | Board→exit Pareto solver | S | TBD | THE central sampled-ramp kernel. Regression seeds: exitfrontier/exitfit/efrefine suites; U_E is a standing cross-check | A18–A20 |
| A23 | Ride edge interception | X | TBD | plane/edge geometry + A20 | A20 |
| A24 | Direct contact transfer | T | PART | exact clip/contact transform exists; face adjacency composition later | A15 |
| A25 | Ground/runoff acceleration | S | TBD | spawn/ground primitive incl. jump timing; map-independent | — |
| A26 | Edge launch | T | PART | `LaunchWitness`/`ReplayLaunch` instruments (session 18) | — |
| A27 | END-volume crossing | T | PART | Minkowski END box + zone-tested-per-tick exact event phase | A3 |
| A28 | Obstacle/corridor traversal | X | TBD (last) | from A14 + reach functions | A8, A14 |

## The reach family (A8–A13 collapse — adopted session 23)

The five air-solving rows are ONE object. Because vertical is
deterministic (A1) and the frame canonicalizes (A0), the fixed-time
reachable set

    R_N(v0, side0, age0)  ⊂  (x, y, vx, vy)   at tick N

carries everything: **D\*** is its support function; **V\*/Ψ\*** are
its velocity marginals; **N_min** is a membership scan over N;
**A11/A12/A13** are witness queries into it (A13 through (T, λ)
slices). One build, five capabilities — and one free falsifier: V*
read from R_N must agree with A6's independent build.

Two consumption modes, both exact:

- **On-demand per exact start** (production solve path): one forward
  sweep with the A4-prod kernel per entrance state, memoized on the
  session-19 K key — the measured 84–85% K-duplication says the memo
  hits constantly; this IS the batched-witness-field lever (~6.3–6.7×)
  in its correct form.
- **Canonical sampled tables** (bounds/design path): R_N over sampled
  v0 with interval semantics — LB from witnesses, UB from certified
  envelopes; v0-interpolation is a stated conjecture with a held-out
  falsifier.

Measured basis: the session-21 B@32 sweep held 474k lattice states
over 90 ticks from one start for 2.97M MoveTicks (2.2 s); with the
A4-prod kernel that is sub-second per start.

## The ride reduction (A18–A23 structure hypothesis)

Surf riding is airborne-against-a-plane: air acceleration + gravity,
velocity clipped into the plane each tick. In the ramp's tangent
frame that is the SAME TickLaw structure plus a constant in-plane
gravity component — so A19/A20 are V*/D* with ONE extra parameter
(the in-plane gravity vector relative to heading), positions are 2-D
in-plane, and A22 becomes a compact kernel over (entry λ, entry
state) × (exit edge point). Interior-of-face only; edge/multi-plane
ticks keep the general engine path. The hypothesis is settled by
extracting A18-prod and parity-gating it (deliverable 11), not by
argument. U_E(B, M) (certified, session 15) already upper-bounds ride
energy and stands as the cross-check.

---

# REGISTRY B — search-space reduction / certified impossibility

Reminder (semantics rule 1): **LB admits, UB culls.** Rows below only
hard-prune once fed by exact/UB surfaces, each cull with a proof
record.

| ID | Reducer | Status | Derived from | Existing assets |
|----|---------|--------|--------------|-----------------|
| B0 | Exact vertical tick window per face | CERT | A1 + A2 | deployed: collapsed a dead 48.7M-tick sweep to a 3-tick answer (session 20) |
| B1 | Face-slice validity region | CERT | A2 | (T, λ) machinery |
| B2 | Horizontal reach envelope | **REFUTED as stated → replaced (s24)** | A8 | **the s20 cone's premise "per-tick |Δv| ≤ 30" is FALSE for braking wishes** (add = cap + |v|; the accel budget ~562 u/s binds) — capreach measured 61,513 real-schedule violations and witnessed states 190u beyond it. `FFReachable`/`FFReachAny` must NOT hard-prune. Replacement: the certified max-speed-integral bound (`CapReach::DStarUB`) |
| B3 | Earliest possible contact tick | PART | A1 + A8 | B0 × B2 intersection as deployed |
| B4 | Latest useful contact tick | TBD | A1 + geometry | — |
| B5 | Required heading-change feasibility | LB only | A6/A7 | V* LB can certify FEASIBLE today; heading-aware culling still needs the theorem step |
| B6 | Required terminal-speed feasibility | **CERT cull (s24)** | A6 | the closed-form speed UB is a true certified cull: required V > √(v0²+cap²·N) ⟹ IMPOSSIBLE — the library's first air UB prune |
| B7 | Combined point/region reachability | PART | A1 + A6 + A8 | B0 × B2 today; exact via R_N membership |
| B8 | Boardability bound | TBD | A16/A17 | — |
| B9 | Minimum unavoidable board loss | TBD | A16 | near-analytic: min |v·n| over admissible arrivals |
| B10 | Ride point/edge reachability | TBD | A19 + A20 | — |
| B11 | Minimum ride ticks to exit | TBD | A21 | — |
| B12 | Finite-horizon ride energy ceiling | CERT | — | U_E(B, M), certified session 15 |
| B13 | Minimum departure resource for an air gap | TBD | inverse A8/A12 | — |
| B14 | Minimum incoming resource required by successor | TBD | backward A12/A16/A22 | the MOTIVATING measured fact: session-18 f3 block — 300–500 u/s f2 arrivals cannot bridge a ~300u gap needing ~600+ u/s. B14 is how the library explains speed compounding |
| B15 | Guaranteed collision / corridor exclusion | TBD | A14 + A28 | — |
| B16 | Conservative successor-face set | X of B0–B15 | — | — |
| B17 | Heatmap-cell optimistic bound | INSTRUMENT | capability extrema over a cell | fpot (validated 1.000) / doom cull / exitgate exist as instruments; promotion requires proof records |
| B18 | Boundary-label dominance theorem | OPEN THEOREM | future capability proofs | session-16 lesson: label dominance must be PROVEN, not assumed |
| B19 | Exact duplicate-state dominance | CERT, measured weak | exact identity | 13 prunes / 1940 states on real geometry — kept because it is free, expected ~nothing |
| B20 | Competitive horizon from incumbent | MECHANISM READY | global T* − g | `HorizonFor` + G13 boundary gate; never engaged yet (no incumbent ever) |
| B21 | Minimum remaining ticks to END | TBD | compose lower-time capability bounds | h_cert = 0 stands until this row is real |

---

# REGISTRY C — scoring / selection / composition

C0/C1 carry information; C2–C5 are projections. No row here invents a
physics score, ever ("high energy" / "fast/flat/high" are banned as
admission rules — measured history).

| ID | Capability | Status | Notes / existing assets |
|----|------------|--------|--------------------------|
| C0 | Canonical transition label (dt, E, q, ψ, vz, d, a, …) | PART | edge records in the graph carry most of this; formalize the tuple |
| C1 | Local Pareto comparison | TBD | retain alternatives that trade time/resource/heading; the anti-Conjecture-M rule at composition level |
| C2 | Board-quality projection | TBD | projection only, never admission |
| C3 | Ride-quality projection | TBD | same |
| C4 | Air-transfer-quality projection | TBD | same |
| C5 | Successor-requirement matching | TBD | value an exit against what the NEXT capability requires (the B14 consumer) |
| C6 | Sampled board→exit kernel (matrix of A22) | TBD | the 10^6-pairs end-state table |
| C7 | Sampled exit→board kernel (matrix of A12/A13) | TBD | same |
| C8 | Transition composition without re-simulation | PART (measured) | the K-duplication result: 84–85% repeat-share, ~6.3–6.7× batching ceiling — memoized R_N is C8's correct form |
| C9 | Min-plus path composition | TBD | trivial once C0 labels exist |
| C10 | Route cost = ticks | CERT (by decree) | g = ticks; h_cert = 0 standing |
| C11 | Search-order heuristic | QUARANTINED | deepen priors / service lanes / diversity buckets exist as instruments; "arbitrary useful ordering; never truth" |
| C12 | Refinement priority | TBD | where uncertainty could alter the answer |
| C13 | Bound-gap / value-of-information | TBD | the first-lost diagnostic (session 21) is the seed instrument |
| C14 | Proof ledger | PART | proof records for certified bounds + suite gates; formalize per-cull records |
| C15 | Global path reconstruction | PART | `RouteWitness`/`ReplayRouteWitness` — hierarchical, bitwise cold replays (G0) |

---

# Dependency structure and build order

The core chains:

    Air:    A4 -> A6/A7 -> A8(R_N) -> A10/A11 -> A12/A13
    Board:  A15 -> A16/A17
    Ride:   A18 -> A19/A20 -> A21/A22/A23
    Two-ramp transfer = A22 -> A12/A13 -> A16   (then C compares)

## Phases

- **Phase 0 — the unlock (next session): A4-prod.**
  `CapAir::KernelTick(vx, vy, side, cosα) -> (vx', vy')` extracted
  from the engine's exact air path (same float ops, same order),
  parity gate `airkparity` (~10^7 random states × all 43 actions +
  edge corpus: v≈0, cosα=±1, speeds at caps). Expected order
  30–100× MoveTick (estimate; MoveTick measured 1.35–1.5M/s). Every
  S-type build downstream runs on this kernel.
- **Phase 1 — finish A6/A7 + the A8 reach family.** Flat-lattice
  exact (v, ψ) band on A4-prod (~1.5–4B transitions/v0 ≈ minutes on
  the kernel; the 400k+ cells/layer are arrays, ~0.5–2 GB); falsifier
  margins must go to zero/O(pitch); the turn-then-accelerate theorem
  step. Then R_N (one sweep per canonical start) with D* + the query
  modes A9–A13, cross-falsified against A6.
- **Phase 2 — board + boundary events.** A16/A17 frozen from the clip
  law (near-analytic); A26/A27 formalized from the existing
  launch/END instruments.
- **Phase 3 — ride.** A18-prod extraction + parity gate settles the
  tangent-frame hypothesis; then A19/A20 (V*/D* + slope parameter),
  then A21/A22/A23 with U_E and the exit suites as cross-checks.
- **Phase 4 — the B battery.** Promote B5/B6/B7 to UB-certified culls
  from the exact builds; B13/B14 successor-resource propagation (the
  speed-compounding explainer); per-cull proof records (C14).
- **Phase 5 — the C layer and the end-state test.** C0/C1 labels,
  C6/C7 sampled kernels, C9 min-plus. The end-state acceptance test:
  ~10^3 × 10^3 ramp-point pairs on basictest = 10^6 evaluations —
  B-battery interval tests first, table queries for survivors — with
  the full-map solve reappearing as composition + pathfinding over
  certified tables. Performance north star: real maps < 10 min,
  basictest seconds-to-sub-minute.

---

## A6/A7 — the N-tick air turn/gain maximum V* and its dual Ψ*

**Naming note (session 25):** sessions 22–24 called this function
"capability 1" from a retired numbering. Registry IDs (A6/A7) are the
only names now — in documents, in suite output, and in discussion.

    V*(v0, N, dψ, d0, a0) = max over legal dwell-6 control schedules
    of terminal speed, subject to net velocity-heading change dψ
    after exactly N clean-air ticks.

**Session-22 state (first full A–G loop):**

1. Six v0 surfaces (250–1400 u/s; N ≤ 90; 0.25° bins; ~48M MoveTicks
   / ~28 s each) — every value a witnessed, engine-replayed schedule
   ⟹ certified LOWER BOUNDS. Dual Ψ* reads the same layers; duality
   green ×6.
2. **CONJECTURE M (max-speed dominance per heading cell) REFUTED** by
   the 30k-random-schedule falsifier: beats +34…+242 u/s with
   witnesses and duality green (instrument sound; conjecture wrong).
   Physics: turn rate ∝ 1/v. Origin of semantics rule 2.
3. The corrected v-stratified build is implemented and cost-measured:
   the exact (v, ψ) band holds 400k+ cells/layer; the std::map build
   aborts at the honest ceiling; falsifier beats vs aborted builds
   are empty-layer artifacts, not counted. `capair` prints RED for
   exactly this reason.
4. Observed structure: V* monotone in N, decreasing in |dψ|; optimal
   witnesses use 1–3 reversals with cosα ∈ [0, 0.15] in the turn
   phase — turn-then-accelerate. At N = 90 heading is nearly free at
   any surf speed; short horizons pay steeply.

**Next for A6/A7 (revised session 23):** A4-prod kernel FIRST
(Phase 0), then the flat-lattice exact band on it, falsifier rerun
(margins → 0), then the theorem step (the turn/accelerate branch
family), then the LB surfaces retire into interval semantics.

## Session 24 — the capability build-out (Phase 0 + spine)

1. **A4-prod exists and is bitwise-certified.** `CapAir::KernelTick`
   is partial evaluation of the verified MoveTick airborne chain
   under the stamped fresh-state clean-air domain, with every elision
   citing the engine line it removes. `capkern`: 2.05M+ engine pairs
   (random + edge corpora incl. signed zeros, exact caps, the s2d = 1
   gate, the total-stop branch, vz ≠ 0), **0 bitwise mismatches on
   all six state channels**; vz-decoupling of the horizontal channels
   certified separately (1M pairs). Throughput 16.0M ticks/s vs
   1.79M = **8.9× measured** (the honest number vs the 30–100×
   estimate: the transcendental chain — atan2f + two sincos per
   action — is the irreducible cost; per-node hoisting of atan2f is a
   recorded future ~20–25%).
2. **A6's exact band completes ×6** on the kernel (the
   std::map ceiling is gone): 22.3M–95.5M nodes, 48–276 s/v0, peak
   layer 311k–1.21M. Every kernel-built extremal schedule replays
   through the REAL engine bit for bit (the composition gate).
3. **The closed-form speed law is certified and TIGHT:**
   V*(N) ≤ √(v0² + cap²·N) from the accel algebra
   (per tick, |v'|² − |v|² = a(2·cur + a) ≤ wishspd² ≤ cap²), and
   the measured free-heading surfaces SIT ON it (379 vs 378.8;
   491 vs 491.0). A6 is now a certified interval
   [witnessed LB, closed-form UB] with the falsifier gap as the
   measured tightness. B6 gains the library's first certified air
   cull.
4. **The session-20 reach cone is REFUTED** — the program's second
   falsified "certainty," caught by A8's falsifier
   (61,513 real-schedule violations) and independently by the swept
   LB itself (witnessed backward reach 365u vs the cone's 284u at
   N=60). The flaw: "per-tick |Δv| ≤ 30" fails for braking wishes
   (add = cap + |v|; the budget ~562 u/s binds). Retroactive: B2's
   `FFReachable`/`FFReachAny` are demoted — nothing may hard-prune
   with them. Replacement: the certified max-speed-integral UB.
5. **The board analytic layer is solved** (A16/A17 closed forms
   verified against dense enumeration of the exact clip; the
   post-speed law within 0.0034 u/s of `Fn::ClipVelocity` over 33k
   arrivals).
6. **The one-tick ride law is PROVEN on face interiors** (A18):
   the decomposition start-gravity → clamp → air-accel(stale
   surface_friction) → single clip → clamp → finish-gravity → clamp
   reproduced **1246/1246** engine contact ticks bitwise across
   3 synthetic ramps × 12 runs; 0 multi-plane ticks and 0
   engine-zeroed (ramp-bug) ticks appeared in the ride domain.
   A19/A20 (ride V*/D*) are unblocked with sf as carried state.
7. **The free reach set is much bigger than any map-directed sweep**
   (A8's ladder: 11.1M nodes at hp 128 → 31.1M at 96 →
   >42M aborted at 64, N=60, one v0) — the session-21 pitch was
   validated with the map doing the culling. The reach family
   ships LB + UB v1 at the measured pitch; direction-aware UB
   tightening and local refinement are the open theorem steps.
8. **Falsifier semantics matured** (adversarial review, 3 verdicts):
   suite-failing gates are now PROOF-BACKED only (build completes,
   bitwise witnesses, duality, coast monotonicity — new theorem
   gate, closed-form UB, LB/UB sandwich, UB-never-beaten, dwell/
   lattice agreement); falsifier beats are MEASURED tightness, never
   an underived threshold; the LB-vs-band comparison is per-sign
   (chirality-aware) and seam-wrapped.

**Next (user directive session 33): FINISH ALL OF CATEGORY A before
more composition.** March order: A9 (terminal-constraint
displacement), A14 (local-set first-contact), A22 exact-hit tail on
the ride plane, A23 (ride edge interception), A24 (adjacent-face
clip packaging), A28 (corridor traversal), the A19/A20 carried-gap
exact law, A8 direction-aware refinements, A11 short-flight interior
coverage, then the A29 duck and A31 trigger debt rows. A32 stays
excluded by decree. Composition work (C9, B16, C5, cheaper A20
builds) resumes after A closes.

**Row-list note (session 27):** `Docs/CapabilityChecklist.md` is the
CANONICAL complete row list (it extends the registry with A29–A32 and
B22 from the session-26 gap audit); this document remains the
deep-status and session-log record.

## Change log

- 2026-08-23i (session 36): **the carried-gap law is EXACT — A18
  completes; A19/A20 close; A22 v3** (detail in the checklist
  change log). `CapRide::MirrorTick` (preamble under stateful sf +
  the A24 mirror + the exact categorize sf rule) rolls mixed
  contact/hover/re-contact rides BITWISE (capgap 6/0: 3600/3600
  ticks, 842 hovers, 327 re-contacts); the generalized contact law
  (contact iff gap + (v_move·n)·dt ≤ 0) verified on every tick;
  A22's exact-hit tail recovers in-authority ride-plane targets to
  0.097u mean (lattice ~30u) with 8/8 engine-bitwise. Remaining A:
  A8 refinements, A11 interior, A29, A31.
- 2026-08-23h (session 35): **A23, A24, A28 close** (detail in the
  checklist change log). The headline is structural: A24 =
  `CapEdge::TryMoveLocal`, the verified TryPlayerMove as a pure
  local function over the bitwise A14 clip — its own census refuted
  the two-law draft (three-plus-bump crease chains), and the full
  mirror matched 36/36 crossings bitwise in position and velocity.
  A28 = the A12+A14 composition (24/24 vs engine); A23 = the A21
  query along edge segments (25/25 == brute). NEW sf-term domain
  law: the air kernel's fixed ctx sf vs the engine's
  stale-stateful rising 0.25 — braking wishes diverge on rising
  ticks; kernel rolls valid on falling flights. Remaining A: A22
  tail, A19/A20 carried-gap, A8 refinements, A11 interior, A29,
  A31.
- 2026-08-23g (session 34): **A9 and A14 close** (detail in the
  checklist change log). A14: `CapContact` — the verified trace's
  per-brush clip verbatim on corridor local sets, 40k/40k bitwise,
  DECLINE outside domain, 25/25 flight contacts vs engine. A9:
  `CapP2P::SolveTerminal` — the 3×3 finisher (tail c1/c2 + brake
  strength vs endpoint x/y + heading) closed the last units;
  contracts hard 19/19, acceptance 19/40 on the adversarial
  generator = the measured family-sufficiency line. Remaining A:
  A22 tail, A23, A24, A28, A19/A20 carried-gap, A8 refinements,
  A11 interior, A29, A31.
- 2026-08-23f (session 33): **the A march opens (user directive:
  finish all of category A first) — A0, A10, A12, A13, A21, A26
  close** (detail in the checklist change log). capsolve 7/0 +
  capmin 6/0; capxfer refactored to consume the packaged A13
  (`CapFaceSolve`) and stays 8/0. The A12 small-N scan found and
  fixed a latent infinite loop (brake-probe stride N/8 = 0 for
  N < 8) in both fixed-N solvers. A26's launch-threshold bracket
  (270.7, 272.4] confirms the A3 hull-overhang constant 256+16 by
  pure measurement.
- 2026-08-23e (session 32): **the composition layer becomes real —
  A3, C0, C1, C6, C7 v2 all close** (full detail in the checklist
  change log, the per-session record since session 29). A3 packaged
  as `CapHull::PlaneOffset` (bitwise vs the world loader, 48/48);
  C7 v2 aims at the HULL-EXPANDED plane and predicts every contact
  EXACTLY through the engine's own clip law — capxfer 8/0, contact
  tick exact 63/63, end position ≤ 1e-4 u, A18 bitwise at all 63
  contacts; the trace clip fraction f = (d1 − 1/32)/(d1 − d2) is
  recorded as a terms-table law. C6 v1 verified — capmat 9/0,
  715,057 real pairs at 270 µs/pair amortized (10⁶ in 4.5 min, no
  extrapolation), indexed-never-beats-exhaustive 1800/1800, engine
  witnesses 22/24 with the 2 boundary-fringe declines counted. C0
  `CapLabel::Transition` unified (compatibility vs quality fields)
  and emitted by both kernels; C1 cell-local strict dominance
  property-gated over 38k labels. Sessions 29–31 (A20/A22, the
  constant-time session, C7 v1 + the interactive dashboard) are
  logged in `Docs/CapabilityChecklist.md`'s change log.
- 2026-08-23 (session 28): A19 (ride N-tick turn/gain) built on the
  proven A18 law — capride 7/0, 30/30 witnesses bitwise as engine
  continuations; **THE CONTACT EPSILON discovered and recorded as a
  standing law on the A18 row** (re-contact needs v·n ≤ −(1/32)/dt;
  the hover band explains both the initial witness failures and the
  falsifier's boundary-surfing ambers up to +215.6 u/s — the exact
  contact-boundary condition with carried gap state is the named
  refinement). Boarding must arrive along-downslope (head-on entries
  measured to lose nearly all speed to the clip). A27 + B3/B4
  certified (capwindow 4/4; END predicate = naive with 12.1× fewer
  tests). Ride physics: 450 → 1322 u/s over 0.9 s at 50°.
- 2026-08-22c (session 27): A11 (fixed-tick point-to-point) VERIFIED
  — capp2p 12/0 via exact segment composition (exhaustive k≤2
  reversal enumeration); A25 (flat-ground walk law) and A30 (jump
  law) BITWISE-CERTIFIED — capground 6/6, ground kernel 10.9×
  engine, 300k parity ticks 0 mismatches. No drift: capkern 5/5,
  capboard 4/4, groute 11/11. Remaining A11 ambers recorded
  (short-flight interior coverage; analytic inversion for cost).
- 2026-08-21b (session 24): **the capability build-out opens** —
  A4-prod kernel bitwise-certified (`capkern`, 8.9×); A6's
  exact band completes ×6 with bitwise witnesses + the certified
  closed-form speed UB (tight at free heading); the s20 reach cone
  REFUTED by A8's falsifier and replaced (B2 demoted, B6
  promoted to a certified cull); board A16/A17 solved; **A18 ride law
  proven bitwise on interiors (1246/1246)**; reach ladder measured;
  gates matured to proof-backed-only after a 3-skeptic adversarial
  review. New commands: `capkern`, `capboard`, `capreach`; `capair`
  gains the exact arm + FINE experiment. Regressions green: groute
  11/11, airprops 24/24, airrec hash bit-identical. Report:
  `Docs/reports/session24-capability-buildout.html`.
- 2026-08-21a (session 23): **the registry adopted** — advisor A/B/C
  taxonomy (A0–A28, B0–B21, C0–C15) is the canonical vocabulary;
  every existing asset mapped to its ID; three amendments recorded:
  (1) A8–A13 collapse into the reach family R_N with two exact
  consumption modes (on-demand memoized / canonical tables), (2)
  A4-prod partial-evaluation kernel promoted to Phase 0 as the
  universal unlock, (3) the LB-admits/UB-culls rule + canonical-
  tables/world-frame-witnesses rule + domain stamps made binding
  library semantics. Deliverable 11 (ref vs prod + parity gate)
  adopted. Build order phased 0–5. Organization session — no solver
  code changed. Report:
  `Docs/reports/session23-capability-registry.html`.
- 2026-08-20f (session 22): program opened; registry seeded;
  Capability 1 LB surfaces + dual built and cross-checked
  (`SolverCapability.h`, `capair`); CONJECTURE M REFUTED by the
  adversarial falsifier (the program's first successful
  falsification); corrected v-stratified build implemented and
  cost-measured (aborts at the cell ceiling; flat lattice next).
  Report: `Docs/reports/session22-capability-air-turngain.html`.
