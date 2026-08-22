# Capability Methods — algorithm selection per atomic function

**Opened 2026-08-22 (session 25), by user directive:** for every atomic
capability, consider EVERY candidate solution method, select the most
efficient one that reflects reality exactly, and organize the results
so the building blocks can be recombined under ANY future architecture.
Nothing in this document assumes the current full-map solver design
(no heatmaps, no entrance fields, no route graphs) — each function is
a standalone solver with an exact contract.

Companion documents: `Docs/CapabilityLibrary.md` holds the registry
(IDs A0–A28 / B0–B21 / C0–C15 — the ONLY capability numbering),
statuses, and session logs. This document holds the method analysis
and the selection rationale.

## 0. Exactness classes (used throughout)

- **CLOSED** — exact closed form in real arithmetic, with a measured
  float deviation bound against the engine.
- **LAW** — exact per-tick recurrence in reduced variables (closed
  form per tick, iterated), bitwise- or ULP-gated against the engine.
- **PARAM-EXACT** — a finite-parameter family of control schedules
  where every candidate is EVALUATED through the exact engine kernel;
  the answer is an exact, replayable schedule. Approximation exists
  only in which candidates are tried, never in physics.
- **CERT-BOUND** — a proven inequality (upper/lower bound) with a
  float-rounding allowance; may hard-prune.
- **EXACT-ENUM** — exhaustive engine-exact enumeration on a stated
  lattice; yields witnessed lower bounds plus measured gaps.
- **MEASURED** — an empirical quantity with no proof attached; may
  inform, never prune.

## 1. The terms catalog

Every quantity that appears in the movement problems, with its exact
law and where it earns its keep. (Constants at surf params: cap =
air_speed_cap = 30; full-stick budget = airaccelerate·wishspeed·dt·sf
= 562.5·sf, sf ∈ {1, 0.25}; ducked-rising budget 47.8; maxspeed 250;
per-component clamp 3500; dt = 0.015; gravity 800.)

| Term | Definition | Exact law / status | Where it matters |
|------|------------|--------------------|------------------|
| N, dt | tick index, tick interval | exact by construction | every capability; time IS the objective (C10) |
| z, vz | vertical state | **CLOSED** (A1): vz halves ±6/tick, z quadratic | arrival-tick windows (B0), plane interception (A13) |
| p = (x, y) | horizontal position | per tick: p′ = p + v′·dt (post-accel velocity) | reach (A8), point-to-point (A12) |
| v, s, ψ | horizontal velocity, speed, heading | state | everything |
| **s²** | squared speed | **THE natural linear variable**: Δ(s²) = cap² − c² per cap-limited tick — affine in the control | tables should stratify in s², not s (uniform growth per tick); energy accounting |
| **c = s·cos α_true** | velocity-aligned component of the wish (α_true = angle wish-from-velocity) | **THE control coordinate** — one scalar per tick fully parameterizes the accel (Strafe::TickLaw) | the entire air family; the true/stored bridge: true cos = −stored cos, true side = −stored side (measured, `wishparity`) |
| a | accel magnitude applied | a = min(cap − c, budget), 0 if c ≥ cap | the only physics the wish controls |
| δψ | heading change per tick | δψ = atan2(a·sinα, s + a·cosα); max-gain value θ(s) = atan(cap/s) | turning tax; curvature |
| Ψmax(s) | max per-tick turn over legal c | 1-D maximization of δψ(c); derivable closed form (one page, open) | path feasibility test in A12 |
| step | per-tick displacement magnitude | step = s′·dt exactly (position uses post-accel velocity) | arc length; endpoint maps |
| **L(N)** | arc length of a max-gain flight | **deterministic**: L = dt·Σ√(s0²+cap²·i) — every max-gain trajectory of N ticks has the SAME length and speed profile | the A8 upper bound (deployed); the A12 reduction |
| dwell age | ticks since last strafe-side reversal | reversal legal iff age ≥ 6 (min_gap law) | the only combinatorial constraint on sign sequences |
| r₁..r_k | reversal times of a control schedule | measured: optimal A6 witnesses use k ≤ 3 | the parameter vector of the A12 solver |
| sf | surface_friction carried state | 1 falling / 0.25 rising (no-walkable rule); scales budget only — **cap-limited ticks are sf-invariant** | ride accel (A18+); budget-bound braking |
| n | face normal | brush plane data | board family |
| v·n | incidence | boarding requires v·n < 0; loss = \|v·n\| | A15–A17 |
| post² | post-board squared speed | s² + vz² − (v·n)² (**CLOSED**, dev ≤ 0.0034 u/s vs engine clip) | exit→board valuation |
| λ | position along a face's fixed-z slice | **CLOSED** geometry (A2) | plane interception (A13), (T, λ) targets |
| g_t | in-plane gravity on a ramp | constant vector per face; enters the PROVEN A18 tick | ride family (A19–A22) |
| E | s² + vz² | derived; conserved under clip only up to loss | exit valuation; U_E ceiling (B12) |
| kernel ticks | count of exact tick evaluations | measured throughput: engine 1.79M/s, A4 kernel 16.0M/s | every cost model in this document |

## 2. Method menu per atomic function

Grouping the registry rows by the method question they pose. Verdicts
name the production method and the certificate method (they need not
be the same algorithm).

### 2.1 Pure transforms — CLOSED, no search (done or near-done)

| ID | Function | Method | Verdict |
|----|----------|--------|---------|
| A1 | vertical state | closed form | **done** |
| A2 | face slice λ(z) | plane/polygon algebra | **done** |
| A4 | one-tick air law | Strafe::TickLaw (LAW) + CapAir::KernelTick (bitwise kernel, 8.9×) | **done**; the dense law-identity gate added this session ties the two together across the full domain |
| A5 | dwell automaton | 6-tick counter | **done** |
| A15 | board clip | Fn::ClipVelocity + closed post-speed law | **done** |
| A16/A17 | best/worst board, feasibility arcs | 1-D trig optimization, closed candidates | **done** (session 24) |
| A18 | one-tick ride law | proven composition of exported engine pieces (1246/1246 bitwise) | **done** on interiors; edge/multi-plane ticks stay on the full engine |

### 2.2 The air family — where method choice decides everything

The candidate methods, once, since they recur:

- **(M1) Dense DP tables** (EXACT-ENUM): forward dynamic programs on a
  state lattice (the current A6 band builds, the A8 sweeps). Cost:
  minutes per start configuration, 10⁷–10⁸ nodes; answers ALL queries
  for that start; witnessed lower bounds with measured gaps; lattice
  and action sampling are the error terms.
- **(M2) Parametric schedule families** (PARAM-EXACT): reduce the
  control space by structure theorems (below), then solve the few
  remaining parameters by root-finding, each candidate evaluated as an
  exact kernel rollout. Cost: microseconds per query. Exact output.
  Requires the structure theorem to state WHICH family suffices — the
  family's optimality is a conjecture until proven, so ship with a
  falsifier against (M1)/(M3).
- **(M3) Guided shooting** (the existing Entrance::RefSolve): general
  inverse solver, ~10³–10⁴ tick-equivalents per query. Correct,
  slow; keeps its role as the cross-check oracle.
- **(M4) Certified bounds** (CERT-BOUND): the closed-form speed
  ceiling √(s0²+cap²·N) (tight at free heading), the max-speed
  integral for displacement, duality, coast monotonicity. Constant
  time. These are the only hard-prune instruments.
- **(M5) Continuum limits**: replace the tick recurrence by its ODE
  (ds²/dn = cap²−c², dψ/dn = δψ) and integrate closed-form. NEVER a
  production answer (the standing law: the simulator is not
  approximated) — but the right INITIAL-GUESS generator for (M2)'s
  root-finds, because the discrete Newton then converges in 2–3
  steps.

**The structural reduction that makes (M2) work — "the path
determines the speed" (user observation, now formalized):**

Per tick, the pair (Δs², δψ) is parameterized by the single scalar c.
Eliminating c: given the current speed and the turn you demand this
tick, the speed change is DETERMINED (feasible iff |δψ| ≤ Ψmax(s)).
Therefore a heading sequence (a path) induces its speed profile
exactly, tick by tick — path and speed are not independent unknowns.
Consequences, each checked against session-24 data:

1. Under the max-gain policy (c = 0 every tick): the speed profile is
   s_n² = s0² + 900n exactly (the certified ceiling — the measured
   surfaces sit on it); the per-tick turn MAGNITUDE is exactly
   θ_n = atan(30/s_n); only the turn SIGN is free, under dwell-6.
   Every max-gain flight of N ticks has the same arc length
   L(N) = dt·Σ s_i. The whole family is parameterized by its
   reversal times alone.
2. "Straight" motion is the minimum-curvature motif: signs
   alternating at the 6-tick dwell floor (net drift ≈ 0). A "turn"
   is a constant-sign run. Measured cross-check: the A8 forward
   reach lower bound at N=60 came within 1.0u of L(60) — the
   alternation cost is real but tiny.
3. Tighter-than-θ(s) turning requires c ≠ 0 and pays speed
   quadratically (Δs² = 900 − c²) while buying rotation ~linearly —
   the A6 witnesses' measured turn phases (stored cosα ∈ [0, 0.15],
   i.e. true c ∈ [−0.15·s, 0], slightly-braking hard turns) are
   exactly this trade at its cheap end.
4. At low speed the trade inverts hard enough that losing speed to
   rotate faster wins (the Conjecture-M refutation, session 22) —
   so families for heading-constrained problems need a braking
   phase parameter, not just reversal times.

### 2.3 The point-to-point air solver (A12, with A10/A11/A13 as modes)
**— the user-named priority, and the largest efficiency win**

Problem: exact start state → target point Q (or face slice (T, λ)):
produce an exact control schedule arriving at Q, minimizing ticks
(or hitting a fixed tick count), with optional terminal-speed/heading
constraints.

Dimensional structure first: vertical is CLOSED (A1), so a 3-D target
fixes the arrival-tick window; each candidate N makes the problem
2-D. A face target is a 1-D λ-interval per tick T (A2). So the core
is: 2-D endpoint, fixed N.

The recommended construction (**PARAM-EXACT, near-constant time**):

- **Family**: max-gain schedules with k reversal times (k = 0..3),
  plus an optional terminal adjustment phase (the last 1–2 ticks
  free in c) to absorb sub-step endpoint error — 2 unknowns, 2
  equations, exact hit.
- **Initial guess**: continuum integrals of the max-gain law give
  endpoint-vs-reversal-times in closed form; invert analytically
  (O(1) arithmetic).
- **Polish**: discrete Newton on the reversal times, each residual
  evaluation ONE exact kernel rollout of N ticks (N=90: ~5.6 µs at
  the measured 16M ticks/s). Expected 2–5 rollouts.
- **Cost**: ~10–30 µs per solve, output an exact replayable
  schedule. Against the measured alternatives: R_N table build is
  ~10⁵× more work per start; guided shooting ~10²–10³× per query.
- **Feasibility pre-check (constant time)**: the certified bounds —
  target beyond the max-speed integral, or required terminal speed
  above √(s0²+900N), is IMPOSSIBLE with a proof (B6/B13). The pair
  (fast parametric solve, certified cull) covers both sides.
- **Modes**: A10 (minimum N) = scan N over the A1 window with the
  O(1) feasibility test, first feasible N solved exactly; A11
  (fixed N) = the core; A13 (face) = solve at (T, λ) targets over
  the 1-D slice.
- **Falsification plan (ships with it)**: (i) endpoints must land
  within stated tolerance, replayed bitwise; (ii) minimality
  cross-checked against the A6/A8 certified bounds and against
  guided shooting on random targets; (iii) any target the family
  cannot hit but the R_N sweep can reach REFUTES the family
  conjecture (k ≤ 3 + terminal phase) and grows the family.

Status: DESIGNED (this document); implementation is the next
session's headline. The family conjecture (which schedule shapes
suffice) is exactly the kind of statement the program's falsifiers
exist to attack.

### 2.4 The tabled surfaces (A6/A7 turn/gain, A8 reach) — role change

With (M2) solving point queries in microseconds, the DP tables'
production role narrows to what tables are actually best at:
- certified-interval SURFACES for bounds and structure discovery
  (they found the closed-form ceiling and the action-sampling
  attribution);
- massive batched queries over one start (the memoized on-demand
  form, keyed by the measured 84–85% repeat-share of entrance
  evaluations);
- falsifier baselines for every (M2) family conjecture.
Their measured error terms stand: action-sampling density is the
binding refinement lever (the 2× lattice experiment moved nothing;
the sampled-action falsifier is clean everywhere).

### 2.5 The ride family (A19–A22) — inherit the air treatment

The A18 proof makes the ride tick the air tick plus one clip against
a fixed plane. In the ramp's tangent frame this is the same
c-parameterized law with a constant in-plane gravity term — so every
air method above transfers: a ride LAW form first (close the algebra,
gate it bitwise like the air law), then certified bounds (U_E is
already certified), then the parametric family for board→exit
(A22: entry state × exit point on a 2-D face — the same
path-determines-speed reduction applies on the plane), with the DP
table as falsifier baseline. Method order: LAW → bounds → (M2)
family → (M1) baseline. Same falsifier discipline.

### 2.6 Ground, launch, END (A25–A27)

A25 ground/runoff: closed friction+accelerate recurrences exist in
the engine mirror; LAW form + small tables; low risk. A26 edge
launch: an event detector on exact rollouts (already instrumented).
A27 END crossing: closed hull-box timing per tick (the Minkowski box
is computed; the per-tick z-window makes it a 1-D interval test).
None of these need search methods; they are transforms plus event
scans.

## 3. The selection matrix (production / certificate, per atomic)

| ID | Function | Production method | Certificate method |
|----|----------|-------------------|--------------------|
| A1/A2/A5 | vertical, slice, dwell | CLOSED | same |
| A4 | one-tick air | LAW (TickLaw) + kernel | bitwise gates (capkern, wishparity, law-identity) |
| A6/A7 | N-tick turn/gain + dual | closed-form ceiling + band table for surfaces | coast monotonicity, duality, ceiling, bitwise witnesses |
| A8 | directional reach | on-demand sweep (batched) / bounds (single query) | max-speed integral UB + witnessed LB |
| A10–A13 | **point-to-point family** | **PARAM-EXACT reversal family + terminal phase (next build)** | B6/B13 culls + A6/A8 bounds + shooting oracle |
| A14 | first contact | exact trace vs local brush set | the trace IS the engine |
| A15–A17 | board layer | CLOSED | enumeration gates (done) |
| A18 | ride tick | proven composition | bitwise harness (done) |
| A19–A22 | ride family | LAW → PARAM-EXACT (planned) | U_E + enumeration baselines |
| A25–A27 | ground/launch/END | CLOSED + event scan | replay gates |

## 4. Build order (updated)

1. **A12/A11 point-to-point solver** (the family + Newton + culls +
   falsifiers) — the named priority.
2. The ride LAW form (A18 algebra closed and gated), then A22
   board→exit as the ride's point-to-point.
3. Heading-aware air upper bound (the braking analysis) — closes
   A6's 180°-at-speed corner and tightens A8 backward.
4. Action-density study for the band tables (the measured binding
   error), only as falsifier baselines require.

## Change log

- 2026-08-22 (session 25): document opened. The c-coordinate
  path-determines-speed reduction formalized from Strafe::TickLaw;
  the point-to-point PARAM-EXACT design selected over tables and
  shooting with measured cost arithmetic; per-atomic method menus
  and the selection matrix recorded; ride family inherits the air
  treatment on the proven A18 tick.
