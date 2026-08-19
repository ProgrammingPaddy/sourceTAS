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

| curve | m=0 | m=1 | m=2 |
|---|---|---|---|
| iso-budget (equal compute) | 15/32 (47%) | 18/32 (56%) | 20/32 (63%) |
| capability (funded 1×/2×/3×) | 15/32 (47%) | **24/32 (75%)** | **30/32 (94%)** |

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
