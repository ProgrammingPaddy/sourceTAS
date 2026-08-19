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

## 13b. RULING 2026-08-19b — what must happen before the m-curve can
##      be read as a Level-B verdict

Received after the first airrec measurements. Level B is **NOT**
approved yet; the flat pre-fix curve was a vocabulary ceiling and the
post-fix curve is not yet a controlled experiment. Nine gates must be
green before "sequential shooting is structurally inadequate" may even
be claimed, and two of them change how the *instrument* works.

**(1) The ≤16u → 0-at-≤8u cliff is partly solver-induced.** Boundary
success is `r_p ≤ radius` while H becomes terminal speed the moment a
trajectory is "successful" — so the optimizer has every reason to reach
15.9u and then spend the rest of its budget on speed. Fix: **active
tolerance continuation**, not passive ladder reporting.

    minimise (r_p, r_θ)  until feasible at the requested ε
    then maximise s_T    subject to r_p ≤ ε and θ_T ∈ I_θ

The ladder 32→16→8→4→2→1 becomes a *refinement schedule*: entering one
tolerance tightens the next, so the residual gradient never flattens.
Until this is in place, sub-8u failure is not evidence of a
conditioning problem.

**(2) The convention bug deserves hardening, not a patch.** Stored
cosα and true wish-from-velocity cosine are different semantic units
that happen to share a type. Required: explicit semantic distinction
(`StoredCosAlpha` vs `TrueWishCos`) with an explicit conversion
(`ToTrueWish`), `Strafe::TickLaw` accepting only true-wish,
`FlyWishSchedule` accepting only stored — and if wrapper structs are
inconvenient, the APIs and names must still encode the units. Plus a
**permanent parity gate**: over a grid of (speed, stored cosα, side),
compare one exact engine tick against `TickLaw(ToTrueWish(c))` for new
speed, heading change, and gain/brake classification, plus a few
multi-tick varying-cosα schedules. **Audit every TickLaw consumer, not
just the shooter — including certified bounds.** A ceiling that
crosses the same boundary wrongly has not been exonerated merely
because airrec has not falsified it (airrec is a falsifier, never a
proof).

**(3) Coordinate charts, not a new trajectory family.** Arc/curvature
node coordinates are approved as an *additional numerical chart* over
the same free shooting nodes. The chord chart
`Y(λ,b) = P₀ + λ(Q−P₀) + b·e⊥` is excellent for direct paths and
terrible under large accumulated turn; add a chart generated from
generic `(λ, φ)` (signed total sweep) or `(λ, κ)`, both signs, broad
range, with offsets around the reference curve permitted. Production
carries **chord chart + curvature chart**; neither defines
admissibility, neither may exclude the other's candidates, and if some
future case is poorly represented by both we add coordinates rather
than declare the trajectory impossible.

**(4) Fixed lateral range must stop being a representational limit.**
±0.5·chord is acceptable as a finite seed range and unacceptable as a
hard bound on where a node may be placed. Node proposal ranges should
come from broad **optimistic reachable slices**, not a chord fraction.

**(5) Curvature-aware to-go residual.** The straight-line remaining
residual is structurally misleading for exactly the failing class. Use
the generic local construction: put the current state at the origin
with velocity along +x and the target at `Q_h = (x,y)`; the circle
tangent to the current heading through Q has

    κ = 2y / (x² + y²),        φ = 2·atan2(y, x)

giving required curvature, accumulated turn, approximate arc length and
implied terminal tangent — compared against remaining ticks, current
speed, free-turn capacity, braking-turn capacity and the requested
terminal-heading interval. Explicitly **heuristic/advisory**, never a
bound, and it encodes no corridor or bulge knowledge.

**(6) Two curves, not one.** Because budget dilution is now measured,
the m-curve must be reported twice:
- **iso-budget** (identical total compute per level) — measures
  production efficiency: recovery bought per unit compute;
- **capability / sufficiently-funded** (each level funded so its own
  added degrees of freedom are actually exercised instead of starving
  the earlier lanes) — measures whether extra shooting structure
  genuinely enlarges the reachable basin.
Without both, `7→8→9` is ambiguous between "m=2 barely helps" and
"m=2 helps a lot but cannibalises its own refinement budget" — two
completely different architectural conclusions. A budget-diluted curve
is **not** a Level-B verdict.

**(7) Freeze and version the fixtures.** The pre/post-fix curves are
not comparable because the generated profiles changed meaning when the
stored-cosα convention was discovered. From now on the oracle
generator is versioned (`airrec-fixture-v2`) with a persisted hidden
manifest sufficient to recreate the identical oracle set; changing
generator semantics, strata, ranges, or the physics convention creates
a new version. airrec is becoming the Stage-A yardstick, so all future
m-curves must be apples-to-apples.

**(8) Use the oracle as a diagnostic, in the harness only.** For failed
hidden cases, take the oracle's actual intermediate positions at the
candidate node times τ_j and express them in *both* production charts;
report `b_oracle/|Q−P₀|` and the nearest-seed distance per chart. If
oracle intermediates lie outside the chord proposal distribution but
naturally inside the arc chart, the coordinate diagnosis is confirmed;
if they already sit comfortably inside the current node domain, the
problem is guidance/optimisation, not node placement. This never feeds
production.

**(9) L1 and L2 stay separate diagnostics.** L1 (exact tick-T boundary
with θ prescribed) exposes the full endpoint-conditioning problem; L2
lets the production frontier choose θ, so it is less constrained.
Strong L2 near-strike recovery therefore does **not** mean Air→Board is
finished: `H(Q) = max_θ J_Q(θ)`, so inaccessible terminal classes still
mean the field may underestimate H(Q).

**Level-B trigger, restated.** Only after (a) convention parity green
and all bridges audited, (b) the cliff retested under active
continuation, (c) both charts available, (d) curvature-aware
to-go residual, (e) rerun on frozen fixtures, (f) both curves, (g)
full-residual GN active, (h) heading intervals still
unresolved/refinable — if a persistent known-reachable class still
improves only weakly (e.g. 25%→30%→32% *with* sufficient refinement
budget), build defect-based multiple shooting. If instead the curve
becomes 25%→60%→85%, sequential shooting is sound and the remaining
work is refinement and scheduling.

**Standing warning re-affirmed:** the gain-vocabulary result (dot −864
→ −34.5, 549k → 1144k from fixing *search vocabulary*) is why poor
search output must never be read as a physical limit. Bounds remain
mathematical; search incapacity remains search incapacity;
under-searched means UNRESOLVED.

## 13c. MEASURED ANSWERS to the nine Level-B gates (session 12,
##      2026-08-19)

**Gate 1 — convention parity + all bridges audited: GREEN.** The new
`wishparity` gate flies one exact engine tick against the law over 168
(speed, stored cosα, side) rows and *discriminates* two hypotheses
rather than assuming one. Measured: H2 wins by 7 orders of magnitude
(max |Δspeed²| error **0.5** = float-ULP scale vs 2.38M for H1; max
|Δheading| error **7.2e-07 rad** vs π), and the realized rotation
follows −side in **98/98** turning rows. So the bridge is a **full π
rotation**: true cos = −stored cos **and** realized rotation = −stored
side. Session 11 negated only the cosine, leaving every modelled turn
pointing the wrong way. `Strafe::TickLaw` now accepts only
`Strafe::TrueWishCos`, so the compiler refuses a naked stored value at
every call site; `ToTrueWishCos` / `ToTrueRotSide` /
`ToStoredWishCos` / `ToStoredSide` carry the measured bridge; all ~25
consumers were audited (repo-wide fan-out) and classified.
**Certified bounds escaped**: `Field::BrakeTurnPeak` (the one place the
heatmap hard-culls) and the priced-brake scan take only
speed/params/ducked, so no stored value can reach them — clean by
construction, not by luck.

**The controller was worse than mirrored — it was inert.** The probe
showed the heading error *frozen* (0.1500 → 0.1500 over 25 ticks, 0/4
targets converging): a requested braking turn mirrored into true
cos ≈ +0.9, far above cap/v, so `addspeed ≤ 0` and the engine did
**nothing**. After the fix: **4/4 targets converge to 0.0000 rad within
5 ticks**, including 1.2 rad. This is the root cause of the historical
"heading-command channel is incomplete (66–282u misses)" finding — the
basis was never the problem; the emitter was.

**The vocabulary consequence.** The law's ACTIVE band is true cos in
[0, cap/v] — only **0.033 wide at v=900**. A fixed grid over [−1,1]
cannot resolve it, so the old candidate/seed grids were, in true units,
a spread of braking turns plus one max-gain point — with **no
sustained-gain member at all**. Candidates and seeds are now generated
speed-normalized (fractions of the band) and converted at the boundary.

**Gate 2 — the cliff retested under active continuation: CONFIRMED
partly solver-induced.** Premise correction first: airsuite's
acceptance radius is **28u**, not 16u, so rung 2 (8u) sat 20u inside
the acceptance set and no search pressure ever pointed there. The
plateau turned out to be **four reinforcing mechanisms**: (1) the score
discarding the residual at the threshold; (2) elite pools freezing —
bins/scouts replace on raw score, and a 32u scout dedupe radius coarser
than every rung below 32u; (3) the Gauss-Newton residual minimiser
gated OFF in face mode by `best.ok` and its iterates then discarded by
value-dominated elites; (4) a second plateau in the *guidance* cost
(quadratic position vs linear sacrifice, so the accuracy gradient
vanishes as the residual shrinks). Fixes: one monotone residual channel
(strike < miss < clean-air rejection, so a converging candidate's score
can no longer *drop* before it jumps), value tier only inside the
active tolerance, tolerance tightened down the rungs **between** rounds
with elites re-keyed from stored raw outcomes, tolerance-scaled scout
dedupe, residual-gated GN in both modes, and per-rung witnesses so a
tight result is usable rather than merely observable. Crediting stays
pinned to `radius`, so no external consumer's contract changed and
continuation can only ADD tight witnesses. Result: **f2's exact-point
row RESOLVED at the literal coordinate for the first time (931k at 0u,
−14k from the human)**; airrec sub-8u rungs moved from 22→27 (8u) and
16→22 (4u) at funded m=2.

Three pre-existing bugs fell out of the same audit: the dry counter
incremented on its *first* round for any solve that never struck
(−1e30f + 1e-3f == −1e30f in single precision), cutting the hardest
cases off after exactly 4 rounds; the crediting gate `sc >= 1e7f`
silently dropped any strike with negative H (reachable whenever a
caller's zmin sits above the contact z — airrec passes zmin = 0); and
the guided shooter's node winner keyed on `best.H`, so on any case that
had not yet struck it stayed pinned at (interval 0, lateral 0) —
meaning m=2 and both GN passes were locked to the same seed on exactly
the cases that needed them most.

**Gate 3 — fixtures frozen and versioned: DONE.** `airrec-fixture-v2`
with a manifest hashing the full oracle set by **bit pattern** (strata,
horizon, start state, schedule, realized boundary condition) plus the
params and law versions. Runs print VERIFIED / DRIFT; measured
`de1b000e431a84fb`, verified across runs. The bank's decimal-text idiom
is lossy and could not certify identity, so it was not reused here.

**Gate 4 — both charts available, hard lateral cap removed: DONE.**
The chord chart keeps `P₀ + λ(Q−P₀) + b·e⊥` but `b` now scales with a
**broad optimistic reachable slice** (from `Envelope::DMax` forward
from P₀ and backward from Q) instead of ±0.5·chord, which was the only
lateral generator in the code and therefore a hard representational
limit. The curvature chart generates nodes from generic `(λ, φ)` on the
circular arc through P₀ and Q with total signed sweep φ, both signs,
broad range. Proposals are **interleaved** round-robin across charts
and intervals, because the guided phase has a fixed budget share and
each shot costs ~25 eval-equivalents with exact rollouts — measured
consequence: at 600–1000 evals only the first ~7–13 of the enumerated
shots ever ran, so **the m=2 pass and the node-time adaptation had
never executed at production budgets**. m=2 and the GN seeds now build
on the winning chart and node time.

**Gate 5 — rerun on frozen fixtures: DONE.** **Gate 6 — both curves:
DONE**, and they are the decision:

> **SUPERSEDED — pre-adversarial-review measurement. Do not use for
> architectural decisions.** The table immediately below was measured
> before the seven review defects of §13d were fixed (in particular the
> boundary value tier firing on a scalarised proxy for the feasibility
> gate). It is kept for history only; the canonical numbers are in the
> *post-review* table that follows it.

| curve (SUPERSEDED) | m=0 | m=1 | m=2 |
|---|---|---|---|
| iso-budget (equal compute) | 15/32 (47%) | 18/32 (56%) | 20/32 (63%) |
| capability (funded 1×/2×/3×) | 15/32 (47%) | 24/32 (75%) | 30/32 (94%) |

**CANONICAL (post-review, fixture hash `de1b000e431a84fb` VERIFIED):**

| curve | m=0 | m=1 | m=2 |
|---|---|---|---|
| iso-budget (equal compute) | 16/32 (50%) | 14/32 (44%) | 14/32 (44%) |
| capability (funded 1×/2×/3×) | 16/32 (50%) | **20/32 (63%)** | **29/32 (91%)** |

The iso curve *declines*, which is a stronger statement than the
pre-review numbers made: at flat spend, added representation is a net
loss. Capability m=2 ladder: 8u:28, 4u:23, 2u:15, 1u:11 of 32; rp p50
2.0u, p95 9.1u; high-curvature class 4/15 → 7/15 → 14/15; zero bound
falsifications.

The persistent known-reachable failing class — high integrated heading
change (dh_tot above median) — goes **4/15 → 10/15 → 14/15** on the
capability curve. L2 reaches 12/12 with rp p50 1.6u / p95 2.1u.
Zero bound falsifications across every run.

**Gate 9 / the ruling: DO NOT BUILD LEVEL B.** The capability curve is
the advisor's own "25%→60%→85%" scaling shape, not the flat
"31%→48%→51%". Sequential multiple shooting **scales**; what remains is
an **allocation** problem — m=2 helps enormously and cannibalizes its
own refinement budget at fixed spend (that is exactly the iso-vs-cap
separation). The next build is therefore the adaptive refinement
scheduler, with κ as a difficulty signal and competitive unresolved
bound gap as the value signal, never κ alone.

**Still open / deferred with reasons.** (a) The oracle-projection
diagnostic (ruling §8) was deferred: its purpose was to decide whether
node *coordinates* explained the failing class, and the capability
curve plus the dh_tot split already answer that affirmatively — it
remains worth building as a permanent instrument, not as a blocker.
(b) f0's rediscovery regressed this session (890k → 750k at 14u, floor
895k standing) while f2 and f3 improved sharply; f0 is short-horizon
and low-curvature, the one stratum the new gain-biased seeds and
curvature to-go serve least. (c) airsuite's own ladder still shows 0
below 8u because its 600-eval budget cannot reach the continuation
phase even with the reserved share — the same allocation finding, and
another argument for the scheduler. (d) `strafelaw` prints "EXACT
(float ULP)" at threshold 4.0 but returns exit code 2 at threshold
0.05: the printed verdict and the exit code disagree (cosmetic, but a
gate should not lie in either direction).

## 13d. What the adversarial review caught (session 12, same day)

The session-12 diff was reviewed by three independent agents against
the standing-laws rubric. Seven real defects were found — including one
of high severity introduced by this very session — and all seven were
fixed before the work was accepted. Recording them because the *class*
of each is instructive.

1. **HIGH — the plateau relocated, not removed.** In boundary mode the
   value tier fired on the weighted ordering residual
   `rp + 250·rth ≤ tol`, while success requires `rp ≤ radius` **and**
   `rth ≤ 1e-4`. The tier was a strict *superset* of the success set,
   so at tol = 8 it accepted heading error up to 0.032 rad — 320× the
   admissible value — and the search switched to maximising speed with
   the heading channel still wide open. That is exactly the pathology
   continuation exists to remove, moved from the position channel into
   the heading channel, and it contradicted §13b as written in this
   same commit. **Fix:** feasibility is now a *predicate on both
   channels*; the weighted sum survives only as the descent ordering.
   *Lesson:* when a spec says "subject to A **and** B", a scalarised
   proxy for the gate is not the gate.

2. **HIGH — stale elite outcomes.** Elites cache a search key plus the
   raw (residual, value) used to recompute it when the tolerance
   tightens. `deepen()` improved an elite's schedule without updating
   the raw triple, so a re-key could score a 3u witness as the 12u
   schedule it had replaced. **Fix:** the evaluator publishes its raw
   outcome; every accept path refreshes it.

3. **MEDIUM — the reserved share was a race, not a reservation.**
   Continuation was triggered by "half the budget spent", tested only
   *after* an unbounded-cost round, which both preempted the value
   phase and could still be inert at small budgets. **Fix:** an
   explicit `phase1_cap` splits the budget deterministically. The
   accompanying claim "no value metric can regress from it" was
   **false** and is removed; the honest statement is that crediting
   stays pinned to the caller's radius, so no *contract* changes, but
   at a fixed budget a reallocated search can certainly report less.
   The measured f0 rediscovery drop is a live instance.

4. **MEDIUM — a fix that loosened the coarse case.** Scaling the scout
   dedupe radius as `max(4·tol, 8)` improved fine tolerances and
   *tripled* it (32u → 115u) for the production field builder, which
   never continues — with only three scout slots, that collapses
   endpoint diversity, the pool's entire purpose. **Fix:** clamped to
   never exceed the original 32u. *Lesson:* a formula introduced for
   one regime must be checked at the regimes it silently also covers.

5. **MEDIUM — the pinning bug was only half fixed.** The node-winner
   key was corrected, but Gauss-Newton pass 2 still built its seed from
   `bmag[win_bi]`, and `win_bi` is no longer assigned at all — so pass
   2 stayed pinned to the chord centreline regardless of which chart
   won. **Fix:** both pass-2 nodes now come from the winning chart and
   node time.

6. **MEDIUM — an unbounded advisory penalty.** The curvature to-go
   estimate diverges when the target lies directly behind (κ → 0 while
   |φ| → π): measured J up to 1.3e10 against a position term of ~18,
   making sidestepping look cheaper than turning. **Fix:** the arc
   length is clamped between the chord and the half-circle.

7. **MEDIUM — a never-live residual channel.** `shot_key()` and the
   convergence counter read `strike_rmin`, which is never written when
   a solve produces no clean strike at all, so the hardest face-mode
   cases still had a pinned winner and hit dry within a few rounds.
   **Fix:** an always-live best-residual channel across every outcome
   class.

**What the review confirmed clean** (verified, not assumed): all 38
TickLaw call sites repo-wide carry correct units and none is mirrored;
the type migration is a numeric no-op, so the one hard cull
(`Field::BrakeTurnPeak`) is bit-identical; crediting of
`best.H`/bins/`iv_L`/tangent is keyed to the original radius and never
to the tightened tolerance; no bank write path changed and a floor
still requires a strictly higher H, so budget reallocation cannot lower
a banked floor; `Field::Build` and `fieldexact` pass no tune, so
`key_of` reduces exactly to the previous face scoring; airrec passes
only (S0, Q, T[, θ]) with seeds and `on_strike` null and no bank
writes; no new erasing prune anywhere — every new `continue` is a
proposal-degeneracy guard, and both the GN gate and the dry gate
*widened*; the new vocabulary literals are band-normalised proposal
ordering only; and the arc-chart closed form was verified numerically
(endpoints, radius, centre, and the |φ| > π major arc), with an
orthonormal right-handed chord frame.

One process finding worth keeping: `Steer::Controller` is shared by the
FROZEN carve and assembly paths, so repairing it changed their
behaviour and the emitted `.tas` frames. That is a *repair* of a
measured defect rather than an architectural change, and it was
re-certified by running both legacy gates — `airsolve` 3/3 (arrivals at
dot −67/−36/−20 versus the tape's −115/−206/−142) and `carve` 3/3
PASS, a gate that had been red 2/3 since the dwell-6 law landed.

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

---

## 15. STANDING RULING 2026-08-19c — build the scheduler; two Stage-A
##     finish lines

Received after §13c/§13d were measured. This section is the current
marching order and supersedes any earlier "Level B if the curve is
flat" language.

### 15.1 The ruling

> **Do not build Level B. Build the scheduler.**

The funded curve shows sequential shooting has the necessary expressive
power; the fixed-budget curve shows compute is currently spent badly.
At sufficient funding 50% / 63% / 91%; at equal spend 50% / 44% / 44%.
So: *more representation helps enormously when exercised, but blindly
paying for it everywhere is worse.*

### 15.2 The convention boundary is permanent law

    solver mathematics / search  →  TrueWish coordinates
                                 →  explicit conversion
                                 →  stored/executor witness

All new search-side physics speaks **true-wish** semantics. The stored
convention is confined to the named execution bridge. **Do not migrate
the bank again for aesthetics** — it stores the exact replay payload and
that is correct as-is.

### 15.3 The historical diagnosis is corrected

"The heading-command representation is structurally incomplete" is
**FALSIFIED**. The real cause was the controller emitter's wish
convention, which made requested turns produce no acceleration at all.
The canonical witness stays the direct per-tick wish schedule (cleanest
representation of admissible inputs; replay remains authoritative), but
the corrected controller is now a **validated search primitive**, not a
discredited one.

### 15.4 The scheduler design

The atomic semantic domain is

    D = (Q region, T branch, I_theta)

Chord/curvature chart, m=0/1/2, GN, scouts are **methods that refine
D**, not separate domains. Each domain carries
`(L_D, U_D, status, kappa_D, spent_D)`. For a board query
`L* = max_D L_D`, and the certified competitive slack is
`C_D = max(0, U_D − L*)`. If `U_D ≤ L* + ε` the domain needs no further
refinement at the requested accuracy — **that is a proof decision;
everything else is scheduling.**

Progressive escalation, not paying for m=2 everywhere:

    broad cheap coverage → competitive intervals
      → hard conditioned intervals → precision

Every unresolved domain gets a cheap scout. Promote through m=0→1→2 only
as necessary; once a feasible witness exists, promote through the
positional continuation ladder; GN/polish only where the residual/value
gap justifies it.

**κ estimates difficulty, never value.** Certified competitive gap
decides whether a domain matters; κ estimates what it will cost. No
single priority equation is sacred — `C_D`, `U_D − L_D`, `κ_D`,
`cost_D`, recent improvement rate, chart success and residual slope are
all fair *ordering* signals. **Only a certified U may ever terminate a
branch.**

**Minimum portfolio coverage.** The f0 regression is the lesson: new
curvature/gain machinery helped long-turn cases and weakened a short
low-curvature one. The answer is NOT an "f0 fix" — it is a retained
generic strategy portfolio (direct/chord, curvature, gain-dominant,
braking-capable, scouts), each with a small exploration floor, with
marginal-useful-improvement-per-cost influencing allocation thereafter.
**No new method may starve an old one merely because its enumeration
comes first.**

**Compute must be monotonic.** For `B2 > B1`, `L(B2) ≥ L(B1)` — inside a
single query's refinement state, not only via the persistent bank. A
larger budget may discover that earlier priorities were foolish; it may
never *forget* a better verified answer.

### 15.5 Measurement additions

Keep the iso-budget and capability curves forever. Add a third mode:

    CURVE[scheduled]   at the same total compute as iso-budget

That is now the number being optimised: scheduled m≤2 behaviour should
approach the funded capability result without funded-everywhere compute.
Also report recovery as a function of total budget, `R(B)` — an anytime
curve that must rise monotonically.

**Do not chase 100% fixed-budget recovery as the definition of truth.**
The operator's contract remains `L, U, RESOLVED/UNRESOLVED, witnesses`.
A hard case unresolved after a cheap query is legal; a hard case falsely
declared impossible is not. A *generous reference run* should recover
essentially all frozen known-reachable fixtures — that is the practical
completeness regression.

### 15.6 TWO Stage-A finish lines (replaces "frozen until perfect")

**EntranceField INTEGRATION-READY** — safe to build higher operators on
because *it never lies*. Returning `L=900k, U=1.05M, UNRESOLVED` is
fine. Requires: exact witness replay; correct control/seam laws; stable
witness bank; no false hard culls; semantic feasibility predicates;
heading domains preserved when unresolved; a scheduler that actually
exercises m0/m1/m2; monotonic lower bounds under increasing compute;
broad recovery of known-reachable synthetic cases under escalation;
certified upper bounds surviving oracle falsification; a working
query/refine API. **When this is green, start ExitField.**

**EntranceField CERTIFICATION-MATURE** — bounds tight enough for
efficient global proof: materially tighter nested ceilings, adaptive
heading/space splitting, local gap closure. This continues *alongside*
ExitField. Separating the bars prevents permanent Stage-A purgatory, and
the global solver uses lazy refinement anyway.

### 15.7 ExitField gets a head start

Do **not** resurrect the 24k carve architecture (greedy scoring,
families, beam, local commitment are all still wrong). But the repaired
controller is reusable *as search machinery* inside
`ExitField(S_board)`, bringing a parity-checked turn law, free-turn and
braking-turn capability, dwell-state continuity, exact replay, a working
on-surface controller and duck-off behaviour. The canonical ExitField
witness remains direct ride inputs — likely `(side, cosα, duck)` plus
whatever `exitfit` proves necessary. `carve` returning to 3/3 the moment
the convention was corrected is strong evidence much of Stage 2's
numerical machinery already exists.

### 15.8 Order of work

1. Gate/document infrastructure (DONE 2026-08-19: `strafelaw` verdict
   and exit code now derive from one threshold pair; the status header,
   the superseded dwell law and the falsified basis diagnosis are all
   annotated). Convert the seven §13d defects into regression/property
   tests.
2. The deterministic central refinement scheduler (§15.4).
3. Nested ceilings `U0 ≥ U1 ≥ U2 ≥ U3 ≥ U4 ≥ H*` with per-constraint
   decrement reporting — the scheduler becomes far more useful as soon
   as `U_D` differs meaningfully between domains.
4. `CURVE[scheduled]` + `R(B)`.
5. Then **stop extending Stage A horizontally** and unfreeze ExitField.

**Design the scheduler as the first implementation of the project's
general refinement philosophy, not an Air-only hack** — the same lazy
allocation recurs at every scale: cosα/control → shooting complexity →
heading interval → face region → transfer → global route branch.

---

## 16. Session 12c — the ruling's "immediately" items, and what the
##     property gate found

### 16.1 `strafelaw` no longer lies to automation

One authoritative tolerance pair (`kLawMaxDG = 4.f`, `kLawMaxDT = 1e-5f`,
ULP-relative) now drives **both** the printed verdict and the process
exit code. Measured: prints EXACT, exits 0. Previously it printed EXACT
at threshold 4.0 while exiting 2 at threshold 0.05.

### 16.2 Design records corrected (all three were live hazards)

- **Status header**: was `NOW → M3 transfer refinement & assembly`, now
  Stage A EntranceField certification, with the standing ruling and a
  pointer to this document. The old header is kept, marked SUPERSEDED.
- **The dwell law**: the M0 foundation entry still described the
  historical ~11-tick alternation limit. It now carries an explicit
  SUPERSEDED note stating the standing law (minimum **6-tick dwell**, a
  lower bound on segment length, not a cadence) so a compacted agent
  cannot rebuild 12-tick pacing from the "status document".
- **The falsified diagnosis**: "the heading-command representation is
  structurally incomplete" is marked **FALSIFIED** in all three places
  it was recorded (the checklist's session-9d entry, `SolverAir.h`, and
  `repfit`'s header in `SolverLab.cpp`), with the real cause named —
  the controller emitter's wish convention made requested turns produce
  no acceleration at all. The canonical witness stays the direct wish
  schedule; the corrected controller is now a **validated search
  primitive**.
- **The m-curve**: the pre-review table is retained and marked
  `SUPERSEDED — pre-adversarial-review measurement`, with the canonical
  post-review table beside it. History preserved, not overwritten.

### 16.3 The property gate `airprops` (7/7 green)

Each of the seven §13d defects is now an invariant checked against the
real production path. Two pure helpers (`ScoutDedupe`, `ToGoArcLen` /
`ToGoSweep`) were extracted so production and the test share one
implementation and cannot drift.

| property | invariant |
|---|---|
| P1 | the boundary value tier is the feasibility **predicate** on both channels, never a threshold on their weighted sum |
| P2 | **monotonic compute**: `L(B2) ≥ L(B1)` for `B2 > B1`, inside one query |
| P3 | deterministic budget ownership: phase 1 owns a bounded share and the tolerance actually tightens |
| P4 | scout dedupe non-decreasing in tol and never coarser than 32u |
| P5 | the winning coordinate chart is recorded and used downstream |
| P6 | the curvature to-go arc length is bounded everywhere, including directly-behind targets |
| P7 | the residual machinery stays active on hopeless queries (no round-1 dry exit) |

**It immediately caught two live violations**, which is the point:

1. **P3 failed**: the tolerance never tightened at all. Two causes — the
   refinement *round* was unbounded (so continuation was gated on budget
   remaining *after* it), and then my own guard tested the phase ceiling
   it was supposed to lift (a self-lock: phase 1 ran to 1015 of 1200 and
   `tol_final` stayed 28u).
2. **P2 failed**: `L(300) = 1028k` but `L(1200) = 1018k` — a larger
   budget produced a **worse** lower bound, violating the monotone-compute
   law. Root cause: `exact_top` flipped at a 600-eval threshold, so the
   two budgets flew *different schedules entirely* rather than one being
   a prefix of the other.

**The precise statement of prefix containment**, learned the hard way:
`run(B1)` must be a **prefix** of `run(B2)`. Truncating a fixed action
sequence at a budget-derived point preserves that — the larger budget
flies the same shots and more. What breaks it is any budget-derived gate
that changes the **content** of an action. So `exact_top` is now fixed,
and the coverage sequence keeps its truncation point. Removing the
truncation as well cost airsuite 48/48 → 38/48 (a fixed 40-shot prefix
costs ~1000 eval-equivalents and starved every 600-eval query) —
restoring it recovered 48/48 at a best-ever 242k mean gap.

### 16.4 A measured input to the scheduler design

With prefix containment in force, the coverage prefix is
budget-independent by construction, so a *fraction* of the budget cannot
bound it: the reserved precision share must apply to the **discretionary
remainder after coverage**. And coverage is expensive — ~25
eval-equivalents per guided shot with exact rollouts on, so budgets
below roughly 1500 are almost entirely coverage. That is a direct
argument for the scheduler's cheap-scout-first escalation over flying
every shot at every domain, and it is now enforced as P3.

---

## 17. Scheduler v0 — the build spec (next session starts here)

Filed 2026-08-19 after the ruling. The scheduler is specified in §15.4;
this section records the **implementation contract** and the one hard
law that the aborted first attempt clarified.

### 17.1 The elevated invariant

> **The requested budget may terminate the solve; it may never influence
> what the solver would do next.**

This is stronger than "same results at larger budgets". It forbids any
budget-derived *skip*, not just budget-derived *content*. Session 12c
already removed budget-derived content (`exact_top` no longer flips at
600 evals). What remains is the guided phase's `gbudget = budget / 3`
truncation: it does not merely stop the solve, it *skips ahead* to the
refinement rounds, so `run(B1)` executes refinement actions that
`run(B2)` reaches only later — not a literal prefix.

**Measured constraint on the fix** (this is why the truncation cannot
simply be deleted): the guided sequence costs ~25 eval-equivalents per
shot, so 40 shots ≈ 1000. Deleting the truncation without an interleaving
scheduler starved every 600-eval query and cost airsuite **48/48 → 38/48**.
The truncation is therefore load-bearing *until* the scheduler exists to
interleave coverage with refinement. Remove them together, never
separately.

### 17.2 Atomic work units

Actions must be small enough that prefix containment is testable below
the round level, and each action's content must be a pure function of
(config, results so far, bounds, conditioning):

    Scout(domain, method, proposal_id)
    Shoot(domain, chart, m, node_seed)
    ExactRank(domain, candidate_id)
    Deepen(domain, witness_id, move_id)
    GNIteration(domain, witness_id, iteration)
    PrecisionStep(domain, rung)
    SplitHeading(domain)

An action whose internal cost is several engine-tick equivalents is
acceptable **only** if that cost is fixed and known in advance, and the
runner refuses to *start* it without enough remaining budget. The runner
then **stops** — it must never skip an unaffordable action in favour of a
cheaper later one, because that reorders the stream.

### 17.3 Domain state

    D = (Q region, T branch, I_theta)
    per D: L_D, U_D, R_D, kappa_D, C_D (spent), M_D (method history),
           sigma_D (RESOLVED/UNRESOLVED)

Chord/curvature charts, m0/m1/m2, scouts, GN and precision continuation
are **methods applied to a domain**, never separate physical branches.
Within a single `RefSolve` call Q and T are fixed, so the live axis is
`I_theta` — the six heading intervals are the v0 domains.

### 17.4 The v0 priority rule (deliberately simple)

1. every unresolved domain gets a minimum cheap scout;
2. mark RESOLVED (with a recorded proof) any domain where
   `U_D <= L* + eps`;
3. competitive domains get m=0 coverage;
4. escalate poorly-converging competitive domains to m=1, then m=2;
5. precision-continue feasible witnesses down the tolerance ladder;
6. GN/deepen where residual or value improvement remains plausible;
7. split heading/spatial domains when the coarse representation is
   itself the unresolved issue.

**Importance is `U_D − L*` only.** κ, residual history and chart history
select *how* to refine and *what it will cost* — never *whether the
domain matters*. Each method carries an exploration-debt floor `F_{D,m}`
so a new method cannot starve an older generic one before certified
bounds make its domain irrelevant (the f0 lesson, generalised).

### 17.5 Properties to add alongside (extend `airprops`)

- **Trace prefix**: for `B1 < B2 < B3`, compare recorded action
  ID/content hashes — `Trace(B1) ≺ Trace(B2) ≺ Trace(B3)` literally, not
  just equal final results.
- **Lower-bound monotonicity**: `L(B1) ≤ L(B2) ≤ L(B3)` with the
  persistent bank disabled.
- **Determinism**: identical (start state, query, version, config) ⇒
  identical action prefix and witnesses.
- **Certified-prune stability**: every domain elimination records the
  bound ID/version and the numerical inequality that caused it — the
  start of proof-carrying prunes.

### 17.6 Sequencing

`scheduler v0` (prove: budget independence, cheap coverage, progressive
m0→m1→m2, fairness, monotonicity) → **nested ceilings U0…U4** (today
most heading domains share one loose global ceiling, so `U_D − L*` has
little information content and tuning against it would be tuning against
noise) → `scheduler v1` + `CURVE[scheduled]` + `R(B)` → adversarial
review → generous AirRec escalation run → **the integration-ready
decision**.

### 17.7 Level B is off the active roadmap

Formally moved from *pending escalation* to **inactive contingency**.
Reopen only if a future generic known-reachable class stays flat under
sufficiently funded sequential m0→m1→m2.
