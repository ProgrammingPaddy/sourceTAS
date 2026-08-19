# AirRec — The Constructive Recoverability Suite (design of record)

Filed 2026-08-19 from the advisor's directive (session 11). This is the
reference for the current build and for every session after context
compaction. Read together with:

- `Docs/OptimalBoardingHandoff.md` — the entrance-operator definition.
- `Docs/FullMapSolverHandoff.md` — the full-map architecture + Stage A.
- `Docs/SolverRebuild.md` §2.11 — adopted rulings.
- `Docs/SolverRebuildChecklist.md` — the session journal.

## 0. Where we are

Stage A has the expected machinery: canonical `(side, cosα)` basis,
exact open-loop replay as truth, complete seam/dwell state, witness
store with migration semantics, heading-conditioned solving, guided
sequential shooting (m=0/1/2 nodes), adaptive node timing,
short-horizon analytical guidance, exact-engine rollout ranking,
within-side varying cosα, outcome-space correction, full-circle heading
exploration, and the synthetic arbitrary-target airsuite.

The architecture question is closed. The open question is:

> **How reliably does this architecture recover boundary conditions
> that we KNOW are actually reachable?**

`airrec` answers that. It is the highest-value next build because it
separates *search failures* from *infeasible arbitrary targets* — the
current airsuite cannot tell them apart (a 0/48 exact-point row is
ambiguous; a 0% cold-recovery row is not).

## 1. The two quarantined fixture layers

### Layer 1 — Air boundary recoverability

Generate a start `S0` and a random *legal* canonical schedule
`U = {(d_k, cosα_k)}` (dwell-6 law, fresh `CtlState`). Replay it in an
obstacle-free environment (high free air over the map; the generator
*verifies* zero contacts and zero groundings) and record the realized
boundary condition `(Q, T, θ, s_T)`. Then hide `U` completely and ask
production to solve

    SolveAirBoundary(S0, Q, T, θ)

cold. This tests the numerical core `(S0, Q, T, θ) → s_A*` — pure
shooting quality, no board response involved. Because free-air vertical
motion is control-independent, the z-residual is 0 by construction and
the position residual is effectively in-plane.

### Layer 2 — Air → Board recoverability

A second class where the hidden legal schedule produces a valid final
target-face strike with no prior collision (single-plane clean-air
semantics). Record `(Q, T, θ, v⁻, E_post)`. Then cold production solves
the ordinary entrance query `(S0, face, Q, T)` — θ is NOT given
(sweeping θ is the production frontier's own job); the oracle θ is used
only to report the recovered strike's heading residual. This tests the
actual operator being certified: Air → Board.

**Construction trick (harness-side only):** free-air flight is
translation-invariant. Fly the hidden schedule from a canonical high
start, take its tick-T displacement Δ, and place the real start at
`q + n̂·ε − Δ` so the true-world replay crosses the face plane at ≈T.
Retry over ε and initial-bearing rotations; a case that cannot be
constructed is printed as UNCONSTRUCTIBLE with the reason — never
silently dropped (no silent caps).

### Quarantine (absolute)

- Oracle schedules, oracle features, and oracle values live in the
  test harness only. Production receives exactly `(S0, Q, T[, θ])`.
- Nothing from airrec is written to the production witness bank, and
  no oracle schedule is ever passed as a seed (`seed_side/seed_cosa`
  stay null; `on_strike` stays null).
- Human tapes appear nowhere in airrec.

## 2. Oracle generation — deliberate control-space stratification

Uniform random per-tick controls produce easy garbage. Stratify hidden
schedules by *generic control complexity* (this samples the canonical
control space; it encodes no trajectory shapes and no map knowledge):

- reversals: 0 / 1 / 2 / 3+ (segments alternate side, each ≥ min_gap)
- dwell style: near-minimum (≈6–9 ticks) vs long dwell
- cosα profile: constant-high; smooth-varying; aggressive evolution;
  deliberate braking sections (cosα < 0)
- horizon: short / medium / long
- start speed: low / medium / high
- vertical state: ascending / near-apex / falling

Deterministic seeded generation (no wall-clock randomness); the matrix
is a stratified cross, not a full product. Report recovery *per
stratum* ("1-reversal recovery 97%" vs "3-reversal aggressive 61%"),
not one aggregate.

**Stored-basis convention (measured 2026-08-19).** The canonical
`(side, cosα)` executor realizes its wish at `wh + π`, so *stored*
`cosα = +1` wishes backward (maximum brake) and the active gain band is
small-negative stored cosα (≈ −cap/s; everything below that band is
inert at that speed — the wish projects above the cap and the engine
adds nothing). The first smoke run proved this constructively: constant
stored 0.95 bled 350→32 u/s. Generator profiles are therefore written
in stored units — gain ≈ −0.02, smooth ∈ [−0.02, 0.5], aggressive
alternates −0.02/0.65, brake-turn 0.7 — so each stratum measures what
its name claims. Feature `brake_ticks` counts stored cosα > 0.3.

**First measured catch (same day): the stage-1 convention bridge.**
`Strafe::TickLaw` is written in TRUE wish-from-velocity units (its
header says so; it implements the SDK addspeed literally), but
`GuidedShootSeq`'s stage-1 closed-form ranking passed *stored* c
straight into `NewSpeed2(c)`/`TurnRad(c,·)` — evaluating every
candidate's speed effect mirrored (c ↔ −c): brakes ranked as inert,
inert as brakes, only c = 0 modeled right. The exact stage-2 rollout
(true engine) could only re-rank a shortlist the mirrored model had
already chosen. Fix: negate at the bridge (`law.NewSpeed2(-c)`), add
speed-derived gain-band candidates (±½·cap/s, −0.95·cap/s — the
vocabulary previously could not express sustained-gain carve arcs at
speed), and a fourth speed-derived knot-resolution rung
(cap/2s; the fixed 0.05 floor was wider than the entire active band at
speed, so long arcs could not be aimed). The first flat recovery curve
(m0 = m1 = m2 = 8/32) was measured BEFORE these fixes — a
vocabulary/model ceiling, not evidence about shooting architecture;
the Level-B verdict reads the post-fix curve.

## 3. Metrics — more than hit/miss

Per hidden case:

- `r_p = |Q_found − Q_oracle|` (Layer 1: state at exactly tick T;
  Layer 2: closest clean strike to the oracle contact)
- `r_θ` terminal-heading residual (distance outside the oracle
  interval)
- Layer 2: `ΔE = E_oracle − E_found` and oracle-energy-recovered %
- spatial recovery ladder at **32 / 16 / 8 / 4 / 2 / 1 u**
- evaluations / engine-tick spend
- failure class (no-strike, wrong-region, wrong-heading, value-short)

Aggregates per (layer, machinery level): cold recovery rate, median and
p95 `r_p`, median and p95 `r_θ`, ladder counts per rung, energy
recovery, spend. This yields a benchmark *curve*, not "42/48 struck".

## 4. Minimum-representation feature correlation

The harness (never production) also classifies each oracle by measured
complexity features: flight ticks, reversal count, minimum dwell, total
realized heading variation Σ|Δh|, max per-tick variation, energy
sacrificed vs ideal gain, braking-tick count. Correlate cold recovery
with those features to learn *which* variable is actually hard (e.g.
"large integrated heading change at high speed", not "speed").

## 5. The recovery curve is the Level-B trigger

Run airrec at machinery levels m=0 → m=1 → m=2 (node count of the
sequential shooter; base seeds/frontier/deepen/scouts run at every
level). Decide from the SHAPE:

- `m0 31% → m1 64% → m2 93%`: sequential shooting scales; Level B
  (defect-based multiple shooting) likely unnecessary.
- `m0 31% → m1 48% → m2 51%` with a persistent known-reachable failing
  class: long-range continuity conditioning is structurally bad —
  build true Level B.

Do NOT start Level B merely because some cases fail.

## 6. Outcome-space correction: full boundary residual, full ξ

The boundary condition is `(Q, T, θ)`. The Gauss–Newton correction must
solve the complete terminal residual

    R(ξ) = [ x_T − x_Q,  y_T − y_Q,  w_θ · d(θ_T, I_θ) ]

over the full active shooting parameter vector — for m=2,
`ξ = (x1, y1, x2, y2)` — with damped least squares

    Δξ = −(JᵀJ + λI)⁻¹ Jᵀ R,   J = ∂R/∂ξ  (no squareness required).

A cheap single-node 2-variable positional pass may remain as the fast
first pass. If node timing later joins ξ, the same formulation holds.
Truth remains exact open-loop replay; GN evaluations are charged.

## 7. Heading intervals: {L_i, U_i, witness, status}

Full-circle interval coverage is search ORDER, not certification. One
shot in a distant interval establishes nothing there — that interval is
**UNRESOLVED**, which is acceptable and must be reported as such. Each
interval I_i carries: witnessed lower bound L_i (best in-radius strike
whose terminal heading lies in I_i), certified upper bound U_i (today:
the global energy ceiling — valid, loose), and status (UNRESOLVED until
it has a witness or a bound proves it irrelevant). Estimates prioritize;
only certified bounds erase. Adaptive interval refinement (split by
normal-dot uncertainty, tangent sign change, competitive upper bound)
comes after airrec.

## 8. Budget allocation: κ is difficulty, not value

The measured budget dilution is real, but conditioning κ alone must not
drive production allocation. The hierarchy:

1. every unresolved domain gets a minimum scout allowance;
2. prioritize regions whose optimistic bound can materially improve
   the answer (competitive gap `U_i − incumbent`, `U_i − L_i`);
3. use κ to estimate how much effort they will need;
4. stop refining once bounds prove irrelevance.

Sketch: `P_i ∝ competitiveGap_i · (1 + β·κ_i) / estimatedCost_i`.
For airrec itself (every test diagnostically equal), flat or
κ-stratified budgets are fine — v1 uses a flat per-case budget.

## 9. Budgets are resolution, not truth

A 300-eval query result never means "the value of the cell". It means
`L = best witness found, U = certified ceiling, status = unresolved`.
A future global solver asks `Query(cell, cheap)` then
`Refine(cell, tighterGap)` when the cell becomes globally important.
All displayed floors are derived queries with explicit semantics —
`L(near16)`, `L(exact4)`, `L(region)` — never a bare number that could
be misread as `H(Q)`.

## 10. AirRec falsifies ceilings; it does not certify them

Every oracle is a constructive lower bound: `H* ≥ E_oracle`. Any
claimed certified ceiling with `U < E_oracle` is **immediately
falsified** (print loudly, fail the run). But `U ≥ E_oracle` across
thousands of tests is *empirical validation only*. Certification
remains a derivation proving `H* ≤ U` over the claimed domain. Keep the
two labels separate forever.

## 11. Nested ceiling attribution (after airrec + scheduler)

Instrument `U0 ≥ U1 ≥ U2 ≥ … ≥ H*`:

- U0: ideal maximum gain
- U1: + fixed T and vertical branch
- U2: + horizontal displacement requirement
- U3: + terminal-heading interval
- U4: + optimistic board response

Report which tightening removes how much of the measured mean gap
(366k at filing time) — that is where the future global admissible
heuristic h(S) gets its strength.

## 12. Separate dashboards from now on

One headline number is no longer enough. Track independently:

- **Search quality**: `E_found / E_oracle` on known-reachable cases
- **Exactness**: `|Q_found − Q|` ladder
- **Boundary quality**: terminal heading / normal-dot residual
- **Bound quality**: `U − E_oracle` and `U − L`
- **Compute**: engine-equivalent evaluations

A stronger search raises L and tightens the sandwich without the
ceiling improving — different achievements, reported separately.

## 13. Standing design laws (post-compaction re-affirmation)

1. General map solver. Human runs and current faces are regression
   fixtures only (deletion invariant: removing every reference tape
   must not change production search decisions).
2. Canonical Air control is per-tick `(side, cosα)` with 6-tick
   reversal state carried across operator seams (Invariant 9).
3. Search machinery may use controllers, shooting, GN, anything;
   accepted truth is only frozen open-loop replay
   (`Air::FlyWishSchedule`).
4. Heatmap value = best actual post-board energy at that physical
   point from that exact state.
5. BestBoard and FastestTangent stay separate; tangent dominance is
   an open theorem.
6. Velocity is physical payload, never independently enumerated
   globally.
7. Clean Air contains no hidden intermediate contacts.
8. The witness bank stores actual transitions; region/exact-point
   floors are derived queries.
9. Human/synthetic oracle witnesses never alter production search
   decisions.
10. Heuristics order search. Only certified bounds hard-eliminate.
11. Unsearched or under-searched regions are UNRESOLVED, not
    unreachable.
12. Fixed evaluation budgets set refinement quality, not mathematical
    truth.
13. EntranceField exposes witnessed lower bounds + certified upper
    bounds + lazy refinement.
14. ExitField comes only when the generic Air operator is trustworthy.
15. Final architecture: EntranceField → ExitField → transfer operators
    → global minimum-tick search → certificate.

ExitField, carve integration, and full-map Phase B remain FROZEN until
the generic Air operator passes Stage A: cold recoverability,
exact-point convergence, frontier coverage, replay truth, honest
bounds.

## 14. v1 implementation map (session 11)

- `Air::FlyWishSchedule` gains a free-flight branch (`t.face < 0`):
  no strike target; any contact or grounding ends the flight with
  `struck_brush`/`grounded` set; `end_state` filled on every exit path.
- `Entrance::RefTune { shoot_m, bnd_theta }` — optional last parameter
  of `RefSolve`. `bnd_theta` (interval `{lo, hi}`) switches RefSolve to
  boundary mode: schedules fixed at length T, flights scored on the
  tick-T residual `r_p + 250·r_θ`, success = `r_p ≤ radius` inside the
  interval, H = terminal horizontal speed (the s_A* quantity). All
  machinery (seeds, scouts, bins, deepen, guided shooting, GN) runs
  unchanged on that score.
- `Entrance::kLadder[6] = {32,16,8,4,2,1}`; `RefResult` gains
  `strike_rmin`, `strike_rmin_th`, `rung_E[6]`, boundary residuals
  `bnd_rp/bnd_rth/bnd_s`, and per-interval coverage
  `iv_L[6]/iv_hits[6]/iv_shots[6]` + `iv_ref` (the reference heading
  the intervals are anchored to).
- Full-ξ GN block after the single-node fast pass (m=2 gate).
- `airsuite` prints per-case `rmin` and the aggregate tolerance
  ladder + median/p95.
- `airrec <map> [--m N] [--budget N]` — generates both layers,
  cold-solves at m=0/1/2 (or the single requested level), prints
  per-case rows, per-stratum aggregates, the ladder, feature
  correlations, falsification checks, and the recovery curve.
