# The Capability Library

**Program opened 2026-08-20 (session 22), user + advisor ruling
2026-08-20f.** This is the design of record for the bottom-up
capability program. The full-map solver is paused as a development
target and retained as a regression consumer; G0R is no longer the
next milestone.

## The governing principle

For every movement capability, answer:

> Given an exact initial state and a small number of constraints,
> what are the MINIMUM and MAXIMUM physically achievable outcomes?

Not "can our optimizer find something good", and not (yet) "how does
this solve the map". Each capability is a unit function

    C(D) = [C_min(D), C_max(D)]     (or its Pareto relation)

over an explicit domain D, solved by exhaustive engine-exact
enumeration of the reduced domain, then (where possible) collapsed to
a proven law. Every proof permanently removes dimensions from every
future search. The functions outlive any particular solver
architecture.

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
10. a stable callable API, map-independent.

## The method (experimental mathematics)

A) minimal inputs -> B) brute-force the reduced domain dense ->
C) visualize -> D) propose the theorem -> E) prove from the movement
equations -> F) falsify exhaustively/randomly -> G) freeze as a
primitive and never search that dimension again.

## Rules

- The exact simulator is NEVER approximated (standing law, session
  20). Search/representation resolution is adaptive; simulator
  accuracy is not.
- No map geometry and no human data in capability derivation -
  synthetic/unit domains only. Map geometry enters only after the
  relevant map-independent functions are solved.
- Representational assumptions (dominance rules, bin pitches,
  representative bases) are stated CONJECTURES and every surface
  ships with an adversarial falsifier attacking them.
- A visual surface report accompanies every capability session.

## The registry

| # | Capability | Status | Inputs | Output | Where |
|---|------------|--------|--------|--------|-------|
| 0a | Vertical state | SOLVED (closed form) | S0, N | z(N), vz(N) | deterministic per tick; used everywhere |
| 0b | One-tick air turn/gain | SOLVED (exact law) | v, cos-alpha | v', d-theta | `Strafe::TickLaw`; wishparity is the standing gate |
| 0c | Board clip | SOLVED (analytic) | v, n | v', loss | overbounce 1; the exitenv premise |
| 0d | Face slice | SOLVED (geometry) | face, z | lambda interval | session 21 (T, lambda) cells |
| 1 | N-tick air turn/gain V* | **IN PROGRESS: LB surfaces built; Conjecture M REFUTED; corrected build cost-measured** | v0, N, d-psi, d0, a0 | max v_N | `CapAir::BuildVStar` (LB) / `BuildVStarStrat` (exact band, needs flat lattice), `capair` |
| 1d | Dual Psi* | same status as #1 | v0, N, V_min | max abs d-psi | `CapAir::QueryPsiStar`; cross-checks #1 |
| 2 | Air directional reach D* | TBD (next) | S, N, phi (+ V_min) | max displacement support function | builds on #1 |
| 3 | Plane interception | TBD | S, plane | T_min/T_max, reachable slice interval per T | #0a + #0d + #2 |
| 4 | Board capability | TBD (mostly #0c) | v_in, vz, n, heading range | E_post min/max, theta_post range, min loss | freeze from existing clip laws |
| 5 | Ride N-tick turn/gain | TBD | B, N, d-theta | max v_exit | the surf-contact analogue of #1 |
| 6 | Ride directional reach | TBD | B, N, phi | max along-ramp displacement | analogue of #2 |
| 7 | Board->exit Pareto | TBD | B_i, X_j | reachable? T_min, E_max, Pareto front | composition of #5/#6 |
| 8 | Exit->board air transfer | TBD | X_i, B_j | valid? T_min, E_max, arrival state | composition of #1/#2/#3/#4 |

## Capability 1 - the N-tick air turn/gain function

    V*(v0, N, d-psi, d0, a0) = max over legal dwell-6 control
    schedules of terminal speed, subject to net velocity-heading
    change d-psi after exactly N clean-air ticks.

**Why first:** it appears inside reachability, air transfer, board
approach, speed-sacrifice decisions, feasibility tests, upper bounds,
and every sampled point-to-point transfer - and it is fully
map-independent.

**Build:** one forward DP per v0 (`CapAir::BuildVStar`); every
transition is an authoritative `MoveTick` in a clean-air world (one
inert brush far below the flight band). State cells =
(heading bin 0.25 deg, carried side, dwell age capped 7), max-speed
representative per cell, full parent chains for witnesses. All
(N <= n_max, d-psi) answers come from the one build; the dual Psi*
reads the same layers.

**Stated conjectures under falsification:**
- CONJECTURE M (dominance): at equal (heading bin, side, dwell) and
  tick, higher speed never has a worse reachable (v, psi) future.
- The 0.25-deg bin pitch and 43-action control sample (coast + 2
  sides x 21 cosa values spanning the full legal wish circle,
  including braking-turns) suffice at the surface's resolution.

**Falsifiers shipped in `capair`:** witness continuous replay
(bitwise engine, real vertical evolution); duality cross-check
V*(N, Psi*(N,V)) >= V on a grid; 30,000 random legal schedules per
v0 that must never beat the surface in their own heading bin (a beat
refutes Conjecture M or the pitch and prints RED).

**Carried findings from session 21 that this capability subsumes:**
the action-sampling/state-lattice co-design law (which cosa samples
suffice is now a property of THIS function's structure, answered
once), and the representative-basis question (what a state cell must
remember is readable off V*'s branch structure).

## Capability 1 — session-22 findings (the first full A-G loop)

1. **Six v0 surfaces built** (250/400/600/800/1100/1400 u/s; N <= 90;
   0.25-deg heading bins; ~48M engine MoveTicks / ~28 s each). Every
   number is a witnessed, engine-replayed schedule => the scalar
   surfaces are certified LOWER BOUNDS on V*.
2. **CONJECTURE M (max-speed dominance per heading cell) is REFUTED**
   — the adversarial falsifier found 74-186 random schedules per v0
   beating the surface by +34 to +220 u/s, with witnesses replaying
   clean and duality green (the instrument is sound; the conjecture
   is what failed). The physics: turn rate scales ~ 1/v, so at equal
   heading a slower state can out-turn-then-accelerate a faster one
   when the remaining task is turn-heavy. Max-speed-per-heading-cell
   is an unsound compression. THIS IS THE METHOD WORKING: stated
   conjecture, adversarial attack, concrete refutation, physical
   cause, corrected representation — in one session.
3. **The corrected representation** (speed joins the lattice: cells =
   (psi bin, v stratum, side, dwell), `BuildVStarStrat`) is
   implemented and its cost MEASURED: the exact reachable (v, psi)
   band holds 400k+ cells per layer and the std::map build hits the
   honest abort ceiling. Falsifier beats against an aborted build are
   empty-layer artifacts and are not counted; the capability gate
   honestly reads "strat build completes: FAIL" until the flat-lattice
   implementation lands.
4. **Observed structure (from the LB surfaces):** V* is monotone in N
   and decreasing in |d-psi| (bin noise aside); optimal witnesses use
   1-3 reversals with cosa concentrated near 0.0-0.15 during the turn
   phase (hard-turn regime) — consistent with turn-then-accelerate;
   straight-line gain matches the known one-tick law compounding.

**Next for capability 1:** flat-lattice (v, psi) band build (arrays,
not maps; the band is ~10^5-10^6 cells x 90 layers x 43 actions ~
1.5-4B MoveTicks ~ 20-45 min/v0 single-thread — or the first
legitimate parallelism target); rerun the falsifier; then the
theorem step (branch family for the turn/accelerate phase boundary).

## Change log

- 2026-08-20f (session 22): program opened; registry seeded;
  Capability 1 LB surfaces + dual built and cross-checked
  (`SolverCapability.h`, `capair`); CONJECTURE M REFUTED by the
  adversarial falsifier (the program's first successful
  falsification); corrected v-stratified build implemented and
  cost-measured (aborts at the cell ceiling; flat lattice next).
  Report: `Docs/reports/session22-capability-air-turngain.html`.
