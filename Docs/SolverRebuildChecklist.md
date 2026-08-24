# Solver Rebuild â€” Build Plan & Living Checklist

**This is the status document.** Design and testimony live in
`Docs/SolverRebuild.md`; the narrative log lives in
`Docs/FullMapSolver.md`; this file tracks WHAT IS BUILT and WHAT IS
NEXT, and is updated every working session (change log at the bottom).

Legend: `[x]` done+validated Â· `[~]` in progress Â· `[ ]` not started Â·
`(!)` blocked/depends Â· each milestone ends with its ACCEPTANCE GATE â€”
a measurable pass/fail, never a vibe.

**THE A CATEGORY IS COMPLETE (session 37, 2026-08-23): A0–A31 all
closed, A32 excluded by decree** — the user directive "finish all A
first" is delivered across sessions 33–37. Two named ambers remain
on already-certified rows (A8 direction-aware UB + local
refinement; A11 short-flight interior coverage) — refinements, not
open rows. **NOW ->** those ambers if desired, then the composition
resumption recorded in the library's next-spine (C9 min-plus over
the C0 labels, B16 + C5 on certified B13/B14, cheaper A20 builds).
The change-log tail below is the session journal.

**STATUS 2026-08-23m (session 40): THE LEGO RULE — binding
semantics #8, and its first application beats every tuned
configuration.** User directive: multiple approaches per algorithm
are FIRST-CLASS units; improvements must be MONOTONE. Never edit
approach A into approach B — compose them as ordered fallback
portfolios (later strategies run only while unsolved; evals update
only on improvement → coverage = the union by construction);
"improved X but regressed Y" signals a missing composition, never a
tradeoff; measured coverage pins as HARD FLOOR GATES that only ever
rise. Applied to A11's exact-hit stage (the proven 2×2 tail + the
3-parameter least-norm tail + the post-brake-sized tail as three
intact fallback strategies): the portfolio EXCEEDED every
individual configuration on every cell — 400: 99/100/100; 800:
91/100/96; 1400: 93/98/100; misses 127 → 23 of 900 (2.6%), two
cells above all constituents. capxfer coverage rose to 37/49 on
BOTH ramps. Floors armed: nine per-cell gates in capp2p (30/0) and
two solver floors in capxfer (10/0). The rule is recorded in the
library's binding semantics, the standing memory, and here. All
twelve suites green.

**STATUS 2026-08-23l (session 39): THE LAST A AMBER RESOLVES — THE
A CATEGORY IS COMPLETE WITH NO OPEN AMBERS.** A8's direction-aware
family lands as certified law (capreach 8/0): the HEADING-FREEDOM
LEMMA (one-tick reversal at s ≤ B = 562.5·sf — constructive 4/4,
arithmetic-exact: backward speed from 555 is 7.50 = 562.5 − 555),
the TURN-BOUND LEMMA (atan(B/(s−B)) per tick above the budget —
unbeaten by 200k adversarial one-tick actions, measured 93.3%
TIGHT), and the TURN-GATED TRAVEL UB (the blind integral with its
first T_turn terms removed; sound for every φ; a certified 3–5%
backward tightening; unbeaten by 20k adversarial schedules per
config, adversaries reaching only 13–39% — the honest slack). The
scoping result: direction-awareness can only bite above the accel
budget — below it heading is provably one-tick free. The heading
laws enter the terms table and feed B22. ALL TWELVE suites green.
NOW -> composition resumes: C9 min-plus over the C0 labels, B16 +
C5 on certified B13/B14, cheaper A20 builds.

**STATUS 2026-08-23k (session 38): THE A11 INTERIOR AMBER FALLS —
14.1% → 4.9% MISS.** A measured-decision session: the A9 transplant
(brake STRENGTH as a per-candidate lever; BRAKE-FIRST + post-brake-
reversal shapes — the measured missing class: hard early brake into
the slow regime where turning is fast; K 6→8) plus the
**3-parameter least-norm rescue tail** — (tail c1, tail c2, brake
strength) against the 2-D endpoint via Jᵀ(JJᵀ)⁻¹, firing ONLY after
the proven 2×2 starts fail on a brake candidate. Two negative
results recorded: post-brake tail-sizing (A9's own fix) REGRESSED
in this solver's flow (the overlapping tail is a useful hybrid
shape here) and always-on 3-parameter stepping destabilized long-N
solves (400/N90 100→91) until rescue-only gating restored them.
Sufficiency (was → now): 400: 88/99/100 → 96/100/100; 800:
67/97/93 → 88/99/95; 1400: 48/94/87 → 83/98/97. Downstream:
capxfer 65° transfers 29 → 35 of 49. All ELEVEN suites green.
Remaining ambers: A8 direction-aware UB + local refinement; A11's
smaller multi-brake-phase interior (worst 48u at 800/N90).

**STATUS 2026-08-23j (session 37): A29 AND A31 CLOSE — THE A
CATEGORY STANDS COMPLETE.** capdebt 7/0 (new suite). A29: the duck
laws gated on the vtable-pinned Fn:: family — the air press/unduck
origin shift is EXACTLY ±8.5000u (one constant across all events),
instant with hull 1 on press / transient hull 2 on unduck; the
shared timer drains exactly dt·1000 per steady tick (162/162); the
grounded 0.34 crop's terminal speed is exactly 85.00; the
session-36 mixed roller at HULL 1 predicts ducked-flight contacts
15/15 vs the engine (the roller generalizes across hulls). A31:
`CapTrigger::ApplyHit` bitwise on all three trigger types (23/23
touch ticks), and the FULL COMPOSITION — per-tick kernel with ctx
(basevel, gravity_scale) + re-touch + ApplyHit, pushes accumulating
inside the volume — bitwise for 15 post-touch ticks on every
flight. Battery green: capdebt 7/0, capgap 6/0, capkern 5/5,
capboard 5/5, capground 6/6, capwindow 6/6, capcontact 5/0, capmin
8/0, capxfer 8/0.

**STATUS 2026-08-23i (session 36): THE CARRIED-GAP LAW IS EXACT —
A18 COMPLETES, A19/A20 CLOSE, A22 v3.** The ride boundary — the
carried 1/32 standoff and its drift, hover ticks, re-contact
fractions, the one regime only the full engine could walk — closes
BY COMPOSITION: `CapRide::MirrorTick` = the airborne chain preamble
under STATEFUL stale sf (the exact categorize rule transcribed:
reset to 1, probe only at vz ≤ non_jump_velocity, air_friction_up
on rising no-walkable) + the A24 local move mirror. capgap 6/0
(new suite): **3600/3600 mixed contact/hover/re-contact ticks
BITWISE (position, velocity, sf) — 842 hovers, 327 re-contacts —
and the GENERALIZED CONTACT LAW on every tick: contact iff gap +
(v_move·n)·dt ≤ 0** (the −1/32 epsilon = its gap = 1/32 case; the
carried gap IS n·pos − d_exp of a bitwise position — no separate
recurrence). A22 v3: the exact-hit tail on the ride plane — 20-tick
C-/S-curve tail with the SPEED-CORRECT live-band mapping (true-cos
past cap/s is dead control; the flat mapping measurably wasted half
its range; same-side shapes are measurably near-1D), Newton through
the mixed roller: 20/20 in-authority targets recovered to mean
0.097u / max 0.242u (the lattice answered ~30u), 8/8
engine-BITWISE; beyond-authority DECLINES to the lattice+N choice.
A19/A20 close: the named refinement delivered; the conservative
build guards remain the surfaces' measured price (rebuilds with the
exact roller = optional tightening). Battery green: capgap 6/0,
capkern 5/5, capboard 5/5, capground 6/6, capwindow 6/6, capcontact
5/0, capmin 8/0, capxfer 8/0. Remaining A: A8 direction-aware
refinements, A11 short-flight interior, A29 duck, A31 triggers.

**STATUS 2026-08-23h (session 35): A23, A24, A28 CLOSE — THE LOCAL
MOVE MIRROR.** A24's first design (two analytical transfer laws)
was refuted by its own census — valley-crease crossings produce
three-plus-bump chains (17/36) no two-plane law covers — and the
structural answer is `CapEdge::TryMoveLocal`: the verified
TryPlayerMove transcribed as a PURE LOCAL FUNCTION over the bitwise
A14 clip (rebases, crease resolution, stop-dead guard, allsolid
zeroing, the unswept stuck guard): 36/36 crossing ticks BITWISE in
position AND velocity (capmin 8/0). A28
(`CapContact::FirstContactOnPath`, the A12+A14 composition): 24/24
solved paths agree with engine replays, 16 clean tick-for-tick +
8 obstructed at the exact tick and brush (capcontact 5/0); the
two-ramp world rebuilt with FINITE segments (the backing slab had
engulfed the spawns — caught by the zero-clean falsifier), and a
valley relearned as non-convex air (two wall brushes, never one).
A23 (`CapRideReach::EdgeIntercept`): 25/25 == brute, 8/8 witnesses.
NEW DOMAIN LAW on the sf term: the air kernel's FIXED ctx sf vs the
engine's stale-stateful 0.25-when-rising — braking wishes diverge
~250 u/s in one rising tick (measured); kernel rolls with braking
wishes are valid on FALLING flights. The stuck-guard predictor (the
d34 silent freeze at (0,0,−6)) is armed in the A14 flight gate.
Battery green: capcontact 5/0, capmin 8/0, capsolve 10/0, capxfer
8/0, capkern 5/5, capboard 5/5, capground 6/6, capwindow 6/6.
Remaining A: A22 exact-hit tail, A19/A20 carried-gap, A8
refinements, A11 interior, A29, A31.

**STATUS 2026-08-23g (session 34): A9 AND A14 CLOSE.** A14
(capcontact 4/0, new suite): `CapContact` — the engine's per-brush
clip transcribed VERBATIM over a corridor-gathered local set:
40,000/40,000 queries BITWISE identical to the full trace (fraction
bits + brush + plane + startsolid) across one- and two-ramp worlds;
the corridor DECLINE law refused 6,742 out-of-domain segments while
every answer stayed bitwise; 25/25 kernel-flight first contacts
(tick AND brush) matched real engine replays; 60–74 ns/query. A9
(capsolve 10/0): `CapP2P::SolveTerminal` — (endpoint, arrival tick,
terminal heading) requests through falsifier-driven design: brake
placement AND strength as the heading/range levers, a scored coarse
climb judging BOTH contracts on every kernel roll (heading-blind
climbing measurably walked into far basins; heading-only judging
measurably stranded candidates 50u out), the recorded
paired-translation lesson, multi-start exact-hit tails, and the
3×3 FINISHER — (tail c1, tail c2, brake strength) against
(endpoint x, y, heading), a square Newton — which took acceptance
2 → 19 of 40 against the most adversarial generator possible
(random braked schedules' exact terminal states). Contracts HARD:
19/19 accepts independently re-rolled (endpoint ≤ 0.5u, heading
≤ 5°, mean 0.49°), 8/8 engine-bitwise; the 21 DECLINEs are the
honest family-sufficiency measure (named refinement: second brake
run / A8 Pareto layer). Battery green: capsolve 10/0, capcontact
4/0, capmin 6/0, capxfer 8/0, capkern 5/5, capboard 5/5, capground
6/6, capwindow 6/6.

**STATUS 2026-08-23f (session 33): THE A MARCH OPENS — SIX ROWS
CLOSE (A0, A10, A12, A13, A21, A26).** User directive: finish ALL of
category A before more composition. Two new suites. **capsolve 7/0**:
A0 packaged (`CapFrame`, bitwise the inline rotation 40k/40k, round
trip ≤ 0.0016u, rotation-commutation of full rollouts MEASURED at
0.0002u worst — the world-frame witness rule stands on measurement);
A12 (`CapP2P::SolveFreeN`, A11 over N with the certified B13
precull) gated 36/36 against the same scan WITHOUT the precull, with
10/10 engine-bitwise replays; A10 (`CapP2P::MinAirTime`, the A1
z-window intersected with that scan) 24/24 == brute; A13
(`CapFaceSolve::SolveToFace` + `PredictContact`) — capxfer
REFACTORED to consume the package and stays 8/0 (one implementation,
its gates the acceptance tests), plus a rotated-azimuth scenario
exercising the general slice geometry: 12/12 exact contact
predictions. **capmin 6/0**: A21 (`CapRideReach::MinRideTime`,
indexed-first with exhaustive confirmation of every miss) 50/50 ==
the brute per-layer scan, 12/12 minimum-time witnesses
engine-validated with 0 boundary fringe; A26 (`CapLaunch::Detector`)
— the launch threshold bracketed to ONE constant across approach
speeds, (270.7, 272.4] ∋ 256+16 = the A3 hull-overhang law confirmed
without assuming quadrant geometry; captures continue BITWISE 30
re-anchored ticks; the first clean airborne tick matches the A1
recurrence bitwise. **BUG FOUND AND FIXED by A12's small-N scan**:
the brake-probe stride N/8 was 0 for N < 8 — an infinite loop latent
in BOTH SolveFixedN and SolveTargetBatch since session 28 (no prior
caller used small N); both sites clamped. Regression green after the
fix: capxfer 8/0, capkern 5/5, capboard 5/5, capground 6/6,
capwindow 6/6 (+ capp2p/capmat rerun).

**STATUS 2026-08-23e (session 32): FIVE ROWS CLOSE — EXACT CONTACT
PREDICTION AND THE PRODUCTION MATRIX.** A3 packaged
(`CapHull::PlaneOffset`, float-identical to the world loader's own
hull expansion, gated bitwise 48/48 inside capxfer) and immediately
consumed: **C7 v2 (capxfer 8/0)** aims its boarding targets at the
HULL-EXPANDED plane — v1's 1.8–3.7-tick early-contact systematic
was exactly this missing offset — and the A4 kernel roll predicts
every contact EXACTLY through the engine's own clip law (fraction
(d1 − 1/32)/(d1 − d2), end position exactly 1/32 above the traced
plane, clipped slide for the remaining time): contact tick exact
63/63, end position within 1e-4 u, A18 ride law bitwise at all 63
contacts; 61/63 schedules board on the requested tick and the 2
curved brake paths that board early are predicted exactly and
counted. **C6 v1 (capmat 9/0)**: per boarding row one A20 surface,
then the FULL time-resolved matrix through the A22 indexed query —
715,057 REAL pairs at 270 µs/pair amortized = 10⁶ pairs in 4.5 min
(the C6 target met without extrapolation; queries alone 3.9 µs
mean; A20 builds ~16 s/row are the named amortization lever), with
soundness ridealongs: indexed-never-beats-exhaustive 1800/1800,
engine witnesses 22/24 (2 boundary-fringe declines counted). **C0
unified** (`CapLabel::Transition`, compatibility vs quality fields;
emitted by BOTH kernels — 63 + 38,231 labels) and **C1 certified as
cell-local strict dominance** (property gates over the emitted set:
irreflexive, 0 antisymmetry, 0 transitivity violations; no
uncertified cross-state comparison — that is the honesty boundary).
Two new terms-table laws recorded: the hull offset off(n) and the
trace clip fraction. Regression green: capkern 5/5, capboard 5/5,
capground 6/6, capwindow 6/6, capxfer 8/0, capmat 9/0.

**STATUS 2026-08-23d (session 31): C7 v1 — THE FIRST TRUE
COMPOSITION VERIFIED; THE INTERACTIVE DASHBOARD OPENED.** capxfer
4/0: one air exit state answers boarding targets on a destination
ramp through the full capability chain — the exact vertical
recurrence (A1) picks each target's arrival tick, the face slice
(A2) gives the target line, the certified departure bound (B13)
culls in constant time, batch A11 solves the horizontal problem
(certified vz-decoupling makes the horizontal schedule valid under
the real vertical), and the ENGINE verifies: transfers first-contact
the ramp within 1.8–3.7 ticks and 13–24u of prediction, with the
PROVEN A18 ride law bitwise at all 62 contact ticks on arbitrary
incoming flights. The hull's leading-edge early contact (the A3
Minkowski offset) is the measured systematic; anticipating it is the
v2 refinement, alongside the matrix production harness. Method
choice tested against alternatives with measured numbers (reach
table: 4–5 orders slower to first answer per start). NEW ARTIFACT:
`Docs/CapabilityChecklist.html` — the living interactive dashboard
(searchable, sortable, term↔capability↔dependency cross-links,
click-open detail drawers; 72 rows, 16 terms; DOM-verified). Update
it alongside the .md every session.

**STATUS 2026-08-23c (session 30): ORGANIZE ONCE, QUERY IN CONSTANT
TIME — BATCH SOLVING, INDEXED TRANSFERS, AND TWO NEW CERTIFIED
CULLS.** Per the user's directive (engine truths are organized, then
terms solve in constant time): A11 batch mode (`CapP2P::P2PBatch` —
all family law-endpoints spatially hashed once per start in 0.1–1.4
ms, then each target seeds from its cell neighborhood and polishes:
**34–135 µs per target at 99–100/100 hits**, full-solver fallback as
the completeness backstop; capp2p 21/0). A22 indexed queries
(`CapRideReach::QueryTargetFast`: per-layer in-plane cell index —
**0.2–0.3 µs per query, ~10,000× faster than the layer scan**, at
lattice resolution with the nearest-node full scan still available;
capexit 9/0). B13 CERTIFIED (min departure resource for a gap:
closed-form speed ceiling + travel-bound bisection; the falsifier IS
the A11 solver attacking from 0.98× the bound — 24 attacks, 0
refutations). B14 CERTIFIED the hard way: the monotone-bisection
draft was REFUTED by its own falsifier (20/127 — post-board speed is
NOT monotone in horizontal speed; a slow-horizontal fast-vertical
arrival retains more), the second draft missed the zero-loss band's
feasibility root (22/185) — the final version computes the EXACT
piecewise-quadratic infimum in closed form, O(1), and survives 185
configurations × 401 exact-clip headings; capwindow 6/6. Battery:
capkern 5/5, capboard 5/5, capground 6/6, capride 7/0, capexit 9/0.

**STATUS 2026-08-23b (session 29): A20 + A22 v1 VERIFIED — THE FIRST
BOARD→EXIT ANSWERS EXIST; THE CONTACT EPSILON IS IN THE PERMANENT
RECORDS.** The contact-epsilon law (session 28's discovery) is now
recorded as a standing law on the A18 row of `Docs/CapabilityLibrary.md`
and as a term in `Docs/CapabilityMethods.md` (user directive). The
A18 POSITION law was measured en route (capboard's new fifth gate):
bitwise on 992/1090 steady contact ticks, worst deviation 3.5e-5 u —
the carried-gap micro-fraction (the clip's adjust pass drifts the
1/32 hover offset by microns per tick, occasionally producing a
micro-fraction bump; the exact carried-gap law remains the recorded
refinement). A20 (`CapRideReach::BuildRideReach` + capexit 9/0): the
ride reach sweep at its OWN pitch (64u positions / 10° headings /
50 u/s strata — the air-derived pitch hit 4.2M cells per layer and
aborted honestly), 3.3–6.1M nodes in 9–19 s per ramp, with UPGRADED
witnessed-bound semantics: each direction's reach bound is the best
node whose witness REPLAYS CLEANLY through the engine (velocity
bitwise, position ≤ 0.01u, never leaving the face), and a direction
with no validated witness DECLINES its bound — 23/24 directions
validated (upslope at 50°, the near-walkable near-stall corner,
declines with 16 boundary-fringe rejects; 60°/70° validate 8/8 with
zero fringe). Measured reach at 0.72 s: downslope 702/770/844 u at
50/60/70°, cross-slope ~470 u, upslope ≈ 0 — the anisotropy that
routes must respect. A22 v1 (board→exit): cell-witness lookup in the
A20 sweep answered 286/286 held-out engine-reachable targets
(rejection-sampled contact-keeping schedules) with mean residual
0.7 u, worst 3.3 u against the 96 u lattice promise. No drift:
capboard 5/5, capride 7/0. Next: the A22 v2 exact-hit tail +
indexed queries; the carried-gap exact law; then the composition
layer (C6/C7 sampled kernels).

**STATUS 2026-08-23 (session 28): A19 BUILT ON THE PROVEN RIDE LAW;
A27 + B3/B4 CERTIFIED — THE RIDE FAMILY IS OPEN.** capwindow 4/4:
the exact vertical recurrence proven bitwise against the engine
(including velocity clamps), the z-window early-exit logic equal to
naive scans over 100k cases, the B3/B4 contact window never beaten by
915 real schedules, and the A27 END-box crossing predicate equal to
the naive scan with 12.1× fewer horizontal tests. capride 7/0 (A19,
`CapRide::BuildRide` + the capride suite): surfaces from exact
ENGINE-REACHED starts (along-downslope boarding — the first run
proved head-on entries lose nearly everything to the clip and settle
near-stopped), surface_friction composed as carried state via the
vz-at-categorize rule, and a discovered piece of ride physics: **the
rider hovers one trace epsilon (1/32) off the plane, so a tick only
re-contacts when v·n ≤ −(1/32)/dt ≈ −2.08 u/s** — the hover band
caused 10/10 witness divergences until the stay-on-face guard used
the exact epsilon condition, after which ALL 30 witnesses replay
bitwise as continuations of real engine runs. Measured ride physics:
a 450 u/s boarding reaches 1322 u/s after 0.9 s of 50° downslope;
±30° of steering is nearly free; builds 56–84 s per ramp (28–34M
nodes). Honest ambers: falsifier beats up to +215.6 u/s at 60° from
trajectories surfing the exact contact boundary that the conservative
guard excludes — the named refinement is the exact contact-boundary
condition (carried gap state). Battery: capkern 5/5, capboard 4/4,
capground 6/6, capp2p 12/0, capwindow 4/4, capride 7/0, groute
11/11.

**STATUS 2026-08-22c (session 27): A11 VERIFIED; A25 + A30
BITWISE-CERTIFIED — THREE CHECKLIST ROWS CLOSED IN ONE SESSION.**
A11 (fixed-tick point-to-point): the Dubins/motion-primitive research
transfer produced the EXACT SEGMENT COMPOSITION enumeration (prefix
tables of the all-plus spiral give any reversal schedule's endpoint
in O(k) with no trigonometry → k ≤ 2 exhaustive, k = 3 dense), and
capp2p went 12/0: machinery 99–100/100 at every (v0, N), worst
residual 0.25–2.3u, 0.6–3.3 ms/solve, 150/150 bitwise replays, all
schedules dwell-legal. Two failures found and fixed en route: the
stored-vs-true wish-basis sign (exactly the SolverStrafe.h bridge),
and a stale incremental build that masked the fix (full-rebuild
discipline for header-only kernel changes). Honest ambers: general
interior targets at short flights 29–88% (the brake family's
measured limit) and the 10–30 µs cost target (analytic inversion
still open). A25 (flat-ground walk law) + A30 (jump law):
`CapGround::WalkKernelTick`/`JumpKernelTick` + the capground suite
6/6 — 200k walk ticks + 50k position checks + 50k jump ticks with 0
bitwise mismatches across velocity, position, feet z, and stamina;
ground kernel 40.8M ticks/s = 10.9× engine; the jump vz law measured
(stamina 0 → 283.993, full tax → 210.839); release gate and
autobunnyhop bypass verified; the settled hull measured one trace
epsilon (1/32) above the hull-expanded floor plane. No drift:
capkern 5/5, capboard 4/4, groute 11/11.

**STATUS 2026-08-22b (session 26): THE MASTER CHECKLIST OPENED; A11
BUILT AND ITERATED v1→v3 UNDER ITS OWN FALSIFIERS.** New:
`Docs/CapabilityChecklist.md` — the single master table (every
capability A0–A32 / B0–B22 / C0–C15 with terms, consumers, primary
method, alternative methods, efficiency figure, and checklist
status), with the gap rows the audit found: A29 duck (currently
domain-stamped out everywhere), A30 jump/bunnyhop (laws decoded, not
packaged), A31 triggers, A32 explicit exclusions, B22 heading-aware
speed ceiling. A11 (fixed-tick point-to-point air solve,
`CapP2P::SolveFixedN` + the `capp2p` suite) implemented and iterated
against its own gates: v1 exposed that max-gain reversal schedules
live on a thin arc-length shell (general targets 0% hit); v2's
brake-run range control (stored cosa ≈ +1 = true wish opposing
velocity) opened the interior (0 → 13–97%); v3 (paired-translation
refinement, top-3 candidates through the exact-hit tail) reached
machinery 92–98% and general up to 99% at 0.4–7.0 ms/solve — with
every emitted schedule dwell-legal and 145/145 solved schedules
replaying through the real engine bitwise. The suite is HONESTLY RED
on the ≥99% machinery gate: the measured binding constraint is the
initializer (coarse grid + local climbing stalls in neighboring
basins), and the continuum closed-form initializer from
`Docs/CapabilityMethods.md` 2.3 is the named next build, carrying
both the remaining hit-rate and the 10–30 µs cost target.

**STATUS 2026-08-22 (session 25): METHOD SELECTION PER ATOMIC — THE
POINT-TO-POINT DESIGN, AND THE NAMING DISCIPLINE.** User directives:
be precise always (registry IDs are the ONLY capability names — the
retired "capability 1/2" ordinals are purged from documents and suite
labels); for every atomic function consider every candidate method
and select the best; point-to-point solving is the priority and
should be near-constant-time; do not restrict designs to the current
full-map architecture. New design of record for method selection:
`Docs/CapabilityMethods.md` — exactness classes; the TERMS CATALOG
(every useful quantity with its exact law: s² is the natural linear
variable, c = s·cos(alpha_true) is the one-scalar control, step
length = post-accel speed × dt, arc length of max-gain flight is
deterministic, sf scales only the budget so cap-limited ticks are
sf-invariant); per-atomic method menus (dense DP tables / parametric
exact schedule families / guided shooting / certified bounds /
continuum initial guesses) with production + certificate verdicts;
and the A12 POINT-TO-POINT DESIGN: the path-determines-speed
reduction (per tick, speed change is a function of the turn executed
— formalized from Strafe::TickLaw), the max-gain reversal-time
family (fixed speed profile s_n² = s0² + 900n, fixed turn magnitudes
atan(30/s_n), only reversal times free, k ≤ 3 measured, terminal
2-tick adjustment for exact hits), continuum closed-form initial
guess + discrete Newton with exact kernel rollouts (~10–30 µs per
solve, output an exact replayable schedule), certified feasibility
culls on both sides, and a falsification plan (bounds, shooting
oracle, R_N sweeps). NEW GATE: `A4 law identity (dense)` in capkern —
one MILLION random states × stored-basis actions: the closed-form law
matches the bitwise kernel to worst |Δ(speed²)| = 5.0 (float scale at
s² ~ 10⁷) and worst |Δheading| = 1.7e-5 rad; capkern now 5/5.
Implementation of the A12 solver is the next session's headline.

**STATUS 2026-08-21b (session 24): THE CAPABILITY BUILD-OUT — PHASE 0
DONE, THE SPINE MOVING, TWO FALSE CERTAINTIES KILLED BY THE
FALSIFIERS.** A4-prod (`CapAir::KernelTick`) is bitwise-certified by
`capkern` (2.05M+ engine pairs, 0 mismatches, all six channels,
vz-decoupling certified; 16.0M ticks/s = 8.9× MoveTick measured).
A6's exact (ψ, v-stratum) band now COMPLETES for all six v0
on the kernel (22–96M nodes, 48–276 s; the std::map ceiling is gone)
with bitwise engine witness replay, and gains a CERTIFIED closed-form
UB: V* ≤ sqrt(v0² + cap²·N) from the accel algebra — TIGHT at free
heading (379 vs 378.8; 491 vs 491.0); falsifier beats are the
MEASURED tightness (worst +19.6…+93.4 continuous-action), and the
LB-vs-band collapse count measures sub-cell state loss. A8
(reach family R_N, D* LB/UB) ran its pitch ladder (11.1M/31.1M/42M+
nodes at hp 128/96/64, N=60 — the FREE reach set is far bigger than
any map-directed sweep) — and its falsifier REFUTED THE SESSION-20
REACH CONE (61,513 real-schedule violations; the witnessed LB itself
beat the "certified" bound by 190u backward): the premise "per-tick
|Δv| ≤ 30" fails for braking wishes (add = cap + |v|, budget ~562
u/s). B2 (FFReachable/FFReachAny) is DEMOTED — nothing hard-prunes
with it; the replacement is the certified max-speed-integral UB, and
B6 becomes the library's first certified air cull. Board: A16/A17
SOLVED (closed forms vs dense exact-clip enumeration; post-speed law
within 0.0034 u/s over 33k arrivals). **A18 (one-tick ride law)
PROVEN bitwise on face interiors: 1246/1246 contact ticks across 3
ramps** — gravity-half → clamp → accel(stale sf) → ONE clip → clamp →
gravity-half → clamp; 0 multi-plane, 0 ramp-bug zeroings in the ride
domain; ride V*/D* unblocked. A 3-skeptic adversarial review drove
the gate semantics to proof-backed-only (coast monotonicity theorem
gate, per-sign chirality-aware LB comparison, seam wraps, dwell/
lattice guard, kernel basevel.Z ride fix). Regressions green: groute
11/11, airprops 24/24, airrec hash de1b000e431a84fb bit-identical.
New commands: capkern, capboard, capreach. See
`Docs/CapabilityLibrary.md` session-24 section + the registry
status columns; report `Docs/reports/session24-capability-buildout.html`.

**STATUS 2026-08-21a (session 23): THE CAPABILITY REGISTRY IS
ADOPTED — A/B/C IS THE CANONICAL VOCABULARY; A4-PROD IS PHASE 0.**
User + advisor taxonomy 2026-08-21a: the full atomic-capability
registry (A0–A28 physical, B0–B21 certified reduction, C0–C15
scoring/composition) is adopted verbatim into
`Docs/CapabilityLibrary.md` as the single canonical registry, with
every existing asset mapped to its ID (TickLaw/wishparity = A4,
clip = A15, U_E = B12, vertical window = B0, reach cone = B2-outer,
(T,λ) = A2/A13, dwell-6 = A5, K-dup = C8, HorizonFor = B20,
RouteWitness = C15, …). Three amendments recorded as binding
semantics: (1) A8–A13 collapse into ONE reach family R_N(x,y,vx,vy)
— D* is its support function, V*/Ψ* its marginals, N_min/point/face
solvers its query modes, with two exact consumption modes (on-demand
memoized per exact start = the correct form of the session-19
batching lever; canonical sampled tables with LB/UB intervals); (2)
the partial-evaluation principle (advisor) is Phase 0: extract
A4-prod `KernelTick(vx,vy,side,cosα)` with identical float
semantics, bitwise `airkparity` gate, est. 30–100× MoveTick — every
S-type build downstream runs on it, turning capability 1's exact
band from hours into minutes; (3) LB ADMITS, UB CULLS — no B-row may
hard-prune from a witnessed LB; tables canonicalize, witnesses stay
world-frame; domain stamps (standing hull, no duck/jump, clean air)
on every surface. Deliverable 11 added (ref vs prod implementations
cross-falsifying forever). Build order phased 0–5 (kernel →
capability 1 + reach family → board/boundary → ride → B battery →
C layer + the 10^6-pair end-state test). Organization session — no
solver code changed; suites untouched. Report:
`Docs/reports/session23-capability-registry.html`.

**STATUS 2026-08-20f (session 22): THE CAPABILITY LIBRARY PROGRAM IS
OPEN — BOTTOM-UP MIN-MAX UNIT FUNCTIONS; THE GLOBAL SOLVER IS A
REGRESSION CONSUMER; G0R IS NOT THE NEXT MILESTONE.** User + advisor
ruling 2026-08-20f: too many simultaneous assumptions at the top —
the program pivots to min-maxing every individual capability function
with unit-level proofs that survive any future architecture (exact
primitive -> min/max capability law -> certified unit operator ->
composition later). Design of record: `Docs/CapabilityLibrary.md`
(registry, standard deliverables, the A-G experimental-mathematics
workflow, rules: simulator never approximated, no map geometry or
human data in derivations, conjectures always under adversarial
falsification, a visual HTML report per capability session).
Capability 1 (the map-independent N-tick air turn/gain
V*(v0,N,dpsi,d0,a0) = max terminal speed under dwell-6, plus its dual
Psi*) is built in `SolverCapability.h` + `capair`: one engine-exact
forward DP per v0 (every transition a real MoveTick in a clean-air
world), all (N, dpsi) answers from one build, witnesses by parent
chain, and three falsifiers (continuous witness replay, duality
cross-check, 30k random legal schedules per v0 that must never beat
the surface). Sessions 18-21 (gmap, the resource lane, the search
architecture correction, the compression benchmark) stand as the
measured context; session 21's co-design law and representative-basis
question are now CAPABILITY questions subsumed by V*'s structure.

**STATUS 2026-08-20d (session 20): THE SEARCH ARCHITECTURE CORRECTION
— THE AUDIT IS DELIVERED; PRODUCTION SEARCH IS UNDER RECONSTRUCTION.**
The standing correction (advisor 2026-08-20d, now a top-level section
of `Docs/ExitFieldSpec.md`): the exact simulator is CHEAP (~1.35-1.5M
MoveTicks/sec single-threaded, measured) and must never be
approximated; the expensive thing is the search FORMULATION —
`RefSolve` (the certified INVERSE boundary solver) was promoted into
the production field engine, so the heatmap became thousands of
per-acceptance-ball optimization runs instead of the OUTPUT of one
forward reachable-state sweep. The `forwardfield` audit ran both arms
on the real f0->f1 problem: the cancelled terms work as arithmetic
(the deterministic vertical window collapsed a dead 48.7M-tick sweep
to a 3-tick answer; the certified 30/tick reach cone prunes en
route); the forward representation PASSES the resolution-scaling
criterion (exact work flat while cells grow 66 -> 1105 from one
sweep) and the production inverse path FAILS it (finer coverage =
more solves; the 84-85% dup is that failure at global scale); but v0
forward is ~50x LESS MoveTick-efficient per cell than one RefSolve
ladder on an easy target and loses 27 shared cells by up to 302k E —
the STATE EQUIVALENCE/PARTITION RULE is the named core research
problem (6.2M strikes -> ~1100 cells; 10.2M frontier-cap overflow).
RefSolve is retained as oracle/polisher/witness-constructor, never
the per-cell engine. User targets recorded: real maps < 10 min (~30
exhaustive); basictest toward seconds-to-<1-min. Paused per advisor:
batching, launch probe, service tuning, G0R marches. NEXT: the
compression rule, then rebuild Entrance production on the forward
field, then G0R. New: `g_movetick_count` (observability only;
airrec hash bit-identical), `forwardfield` command. See
`Docs/ExitFieldSpec.md` "THE SEARCH ARCHITECTURE CORRECTION" +
session 20.

**STATUS 2026-08-20c (session 19): G0Q DIRECTIONALLY CONFIRMED; THE
84% K-DUPLICATION CEILING IS THE MEASURED PATH TO THE 5-MINUTE
TARGET.** The resource-preservation service lane (advisor
2026-08-20c: objective min-g + resource max-E(B) representatives per
face/position bucket, least-served fair rotation, SERVICE ONLY - no
blended scores, no thresholds, no pruning; legacy behind --svc18)
ran a clean 3600s-vs-3600s causal A/B with the launch and all
operators frozen. MEASURED: the same 250 u/s launch exposes
materially stronger late-face states when they receive refinement -
bridge-class f2 exits (606 u/s @ z 157, inside f3's window by the
arithmetic) exist ONLY in the resource arm; f1 median board E +24% -
but the lane pays in march speed and neither arm's hour completed
the fast-f2 fine ladders against f3 (G0R still open, f3 never
witnessed). THE BINDING CONSTRAINT IS THROUGHPUT: the entrance
K-duplication measurement (queries grouped by exact (air start,
face, horizon)) shows 84-85% of ALL entrance evals are repeat-share
across BOTH policies - ~5,400-6,000 distinct physical problems
solved 4.2-4.4x over. The ~6x batched-witness-field amortization is
the advisor's post-G0R project, now measured justified, and the only
path to the user's <= 5-min basictest budget. Open: the conditional
generous launch-capability probe. See `Docs/ExitFieldSpec.md`
session 19.

**STATUS 2026-08-20b (session 18): G1M DELIVERED, G0R NOT REACHED -
THE REAL-MAP MEASUREMENT.** The `gmap` command ran the explorer cold
on surf_basictest eight times; the march works autonomously three
faces deep (f0 -> f1 -> f2, 1940 exact states, 1944 edges all
cold-replaying bitwise, 0 topology violations, no tape anywhere), and
every starvation was measured and answered with ordering/coverage
policy (ballistic + spread aims, geometry-only horizons, exit
diversity buckets, entrance-query persistence, within-face position-
bucket deepening). THE MEASUREMENT: entrance refinement = 98-99.5% of
all wall-clock (43.4M evals/hour); same-face hop proliferation
dominates state-space (exact-dup dominance nearly never fires: 13
prunes / 1940 states); and the G0R blocker is SPEED COMPOUNDING -
cheap short-hop transfers are found first, slow arrivals ride slow,
and 300-500 u/s f2 arrivals physically cannot bridge to f3's window
(needs ~600+ u/s at height). groute is 11/11 (G13 horizon-boundary
gate new); DomainAudit stable domain identities + GlobalRouteWitness
+ END-via-air landed in `SolverGlobal.h`. NEXT: advisor ruling on the
speed-compounding dynamic (service/ordering or launch), and entrance
spend is THE perf target - nothing else is measurable. See
`Docs/ExitFieldSpec.md` session 18.

**PERFORMANCE NORTH STAR (user, 2026-08-20): solve surf_basictest in
<= 5 minutes wall (goal: sub-1-minute).** Context: basictest is 4
ramps / under 10s of play; real maps are 30+ ramps and 60+ seconds of
play. Session 18 measured entrance refinement at 98-99.5% of all
wall-clock, so the target lives or dies on entrance throughput -
the advisor-approved post-G0R project is the K-duplication
measurement (instrumented session 19) -> shared/batched witness-field
evaluation IF the measured overlap justifies it. Every service/perf
decision should be checked against this budget.

**RULING 2026-08-19: do NOT build Level B (defect-based multiple
shooting). BUILD THE SCHEDULER.** The funded recovery curve showed
sequential shooting has the expressive power (16/20/29 of 32 at
1x/2x/3x); the equal-compute curve (16/14/14) showed compute is being
spent badly. Next build = the deterministic central refinement
scheduler over domains D = (Q region, T branch, I_theta).

> **PARTLY SUPERSEDED 2026-08-19 (session 12l).** The Level-B half
> of this ruling STANDS. The "next build = the scheduler" half is
> spent: the scheduler was built (v0, v1, ScoutCheap, S1-S10) and
> measured, and it is NOT the production local engine. Legacy
> `RefSolve` keeps that job; the anytime contract moved up to
> `Entrance::RefineStep`. Do not read this paragraph as an
> outstanding task.

**SUPERSEDED HEADERS (kept for history, do not act on):** this document
previously read "NOW -> M3 transfer refinement & assembly". M3-era
assembly, the carve port and full-map Phase B are FROZEN behind Stage A.

---

## M0 â€” Foundations

- [x] 0.1 Design of record + uncompressed expert knowledge base
      (`SolverRebuild.md`, commit c6de8a8)
- [x] 0.2 Strafe law: closed form (`SolverStrafe.h`) + `strafelaw`
      prover â€” 1,120 states vs certified MoveTick, float-ULP exact
      (max 2.0 in speedÂ² of 11.5M; 8e-9 rad) (d4f6402)
- [x] 0.3 Feature extractor v0 + `routegraph` command â€” brush-side
      polygons, plane/area/extents/downhill, candidate edges (d4f6402)
- [x] 0.4 Face coverage on basictest â€” GATE PASSED via the new
      `facecover` command: both certified tapes replayed through the
      exact sim, **483/483 surf contacts map to extracted faces, 0
      missing**. (The spine worry was unfounded: its ridden moments are
      walkable-top GROUND contacts, not surf.) `facecover` stays as the
      standing per-map gate.
- [x] 0.5 Zone anchoring: `Route::AnchorZones` â€” start = brush under
      the anchor origin; end = --end-brush or DERIVED by replaying the
      anchor tape to its finish (zones are plugin-side on real servers,
      so a human trace is the honest zone source â€” the expert demo
      traces serve this role per map). basictest: start idx 6, 3
      candidate first boards; end id 10, 2 feeder faces.
- [x] 0.6 Strafe alternation rate limit: **6 direction changes per
      second** (user, 2026-08-16: "more than 6 strafes per second is
      extremely rare") = min ~11 ticks between alternations at
      66.67tps. `MoveParams::strafe_rate_max`, params-file loadable;
      becomes the yaw-spline knot cap in M1.3.

## M0.7 â€” DLL crash hardening (user-blocking, added 2026-08-16)

- [x] 7.1 Demo-capture freeze ROOT-CAUSED by the breadcrumb journal
      (last stage = update:democap, EndScene thr != CreateMove thr):
      with multicore rendering, ALL UI ran on the render thread, and
      connection transitions (playdemo/disconnect/map) issued there
      deadlock the engine. FIX: game-thread command marshal
      (PushEngineCmd/DrainEngineCmds via the CreateMove hook) +
      DemoCap disconnect-first phase + observer-index clamp (1..64).
- [x] 7.2 Old hard-crash family: the menu/HUD draw ran UNGUARDED for
      its whole life (crash.log "stage=(none)" rows). Now under the
      same hook-level SEH as Update/WorldDraw: fault -> logged +
      retried, not a dead game.
- [x] 7.3 Tab-out crash: the Reset hook rebuilt ImGui device objects
      even when Reset FAILED (D3DERR_DEVICELOST retries while tabbed
      out), corrupting the device. FIX: rebuild only on SUCCEEDED
      reset + skip all drawing while TestCooperativeLevel != D3D_OK.
- [ ] 7.4 Remaining families, evidence pending their next occurrence
      (the journal now names the subsystem): inject-while-in-map,
      map-load-without-server, wrong-map load, recordings-loaded
      freezes. Do NOT guess - read breadcrumb.log after each.

## M1 â€” Transfer primitives & the ledger

- [x] 1.1 Reachability envelope (`SolverEnvelope.h` + `envelope` gate
      command): EXACT discrete ballistic z (half-gravity structure:
      z(n) = z0 + nÂ·dtÂ·vz0 âˆ’ gÂ·dtÂ²Â·nÂ²/2), speed bound s(n)Â² â‰¤ s0Â²+900n,
      distance bound (gain-optimal straight line), ZWindow + CanReach
      edge gate. GATE PASSED: 15/15 tape airborne stretches contained
      (z/speed/dist all 0 violations) + 500/500 random-control
      falsification trials stay inside, 0 escapes. Laws learned: the
      envelope bounds the AIR PHASE ONLY â€” measure at the last airborne
      tick, never after the contact tick (the board clip converts vz
      into horizontal speed = M1.2's business); z sits in a duck-offset
      band {âˆ’8.5, 0, +8.5} RELATIVE TO ENTRY DUCK STATE, not on the
      ballistic point.
- [x] 1.2 Board window per face (`SolverBoard.h` + `boardwin` gate):
      closed-form clip physics â€” dot = v1Â·n sweeps [vzÂ·nz âˆ’ sÂ·h,
      vzÂ·nz + sÂ·h] over aim; speedÂ² loss = dotÂ² EXACTLY; MinApproachDot
      = the tangency law (0 â‡” sÂ·h â‰¥ |vzÂ·nz|, else unavoidable loss);
      AimCone(cap); polygon region test with hull-center slack 43u
      (=|(16,16,36)|, geometric). TickEvents grew contact_pos/
      contact_vel (pure instrumentation); Fn::ClipVelocity exposed.
      GATE PASSED first run: 13/13 tape boards approaching + in-region
      (max edge âˆ’0.4u) + min-law held + clip model EXACT vs the
      mirror's own measured loss (worst 0.0000 u/s); spot check 827
      single-plane strikes across all faces Ã— aim spectrum, 0 cone
      violations, closed-form zero-input clip-tick prediction matches
      MoveTick exactly (StartGravity â†’ clip â†’ FinishGravity decomposition
      confirmed). FINDING: the certified tapes' worst board = |dot|
      432 u/s = 38.5% of speedÂ² lost â€” the OLD solver's board quality
      quantified (these tapes are parity-certified, not optimality-
      certified). The expert cap must come from the demo traces (M1.5
      ledger), NOT from these tapes.
- [x] 1.3 Air-phase primitive (`SolverAir.h/.cpp` + `airsolve` gate):
      target-heading spline flown on the exact engine by a law-derived
      controller â€” per tick it lands the spline heading exactly via the
      certified turn-curve inversion (cosa âˆˆ [0, cap/v]: full-turn-full-
      gain â†’ no-turn-no-gain), and beyond the perp rate engages the
      BRAKING TURN (cosa < 0: budget 562.5 >> cap 30 buys ~32Â°/tick at
      850 u/s for ~114 u/s â€” testimony's "eat the energy in the turn",
      now a controller capability whose cost the optimizer owns via the
      spline slope). Strafe-side flips rate-limited by construction
      (min 12 ticks; blocked flip with small error weaves, large error
      coasts). Search: 5 unseeded initial families (linear/late-turn/
      pure-pursuit/mirrored-tangent/outward-bump) + BASIN HOPPING
      (deterministic jitter restarts â€” plain coordinate descent
      converged at ~200 evals and left the rest of any budget unspent)
      + polish hops around the global best. Spline spans the expected
      flight (aim_tick), not the sim cap â€” late knots must be live
      parameters. GATE PASSED: 12/12 tape transfers reproduced
      unseeded (entry state + landing window only, never tape
      controls), 1.0s wall for the whole suite at 3000 evals/transfer.
      Quality: the primitive beats the tape's board on 10/12 â€”
      e.g. dot âˆ’50 vs tape âˆ’377, âˆ’205 vs âˆ’432, two near-perfect
      tangent arrivals (âˆ’1.8, âˆ’0.6).
- [x] 1.4 Carve primitive (`SolverCarve.h/.cpp` + `carve` gate + the
      shared `SolverSteer.h` controller): on-face rides driven by the
      SAME certified controller as the air phase, with two extra
      dials â€” an EFFORT channel (per-knot duty-cycled coasting: the
      expert's speed control; slow rides are effort choices, not
      heading choices) and the DUCK-OFF exit move (press duck while
      riding: the +8.5 air-duck shift pops the hull off the face,
      keeping the climb velocity â€” how solved12's crest exit works,
      found by `ridedump` on the tape's final tick: b1028 d1). Carve
      ENERGY LAW verified against the engine: vÂ²_exit + Î£dotÂ² = vÂ²_entry
      + 2gÂ·drop (+ wish work â‰¤ 900/tick), with the DERIVED discrete
      cross-term bound Î£ gÂ·dtÂ·nzÂ·|dot| (a clip at fraction f inside a
      half-gravity tick shifts E by gÂ·dtÂ·(2fâˆ’1)Â·nzÂ·dot â€” measured law,
      not a fitted tolerance). Exit spec = full VELOCITY VECTOR (crest
      launches have mostly-vertical velocity; horizontal heading alone
      is ill-conditioned). GATE PASSED: 10/10 tape carves reproduced
      unseeded (worst dv 16.4 u/s, seven rows < 8), manifold 80/80
      zero-input identity + 80/80 strafing law bound, 6.4s wall.
- [x] 1.5 THE LEDGER (`SolverLedger.h/.cpp` + `ledger`/`ledgergate`/
      `ledger-trace`): exact event-sourced phase decomposition
      (GROUND/AIR/RIDE) of any control line â€” air-gain shortfall vs
      the 900/tick law, board lossÂ² + fraction + TANGENCY REGRET
      (dotÂ² âˆ’ minÂ² at the actual arrival), ride clip dissipation,
      gravity conversion, wish work via energy closure (self-auditing
      within the derived cross bound + a float-accumulation allowance
      ~ULP(vÂ²)/op). GATE PASSED: closure 0/9 bad on the 292 tape;
      planted 15-tick coast localized (prior phases bit-identical,
      delta +65.3k = 13.5k direct theft + 40.9k of misaligned-yaw
      braking the ledger also priced). THE INDICTMENT of the old
      line, now in numbers: 329,477 uÂ²/sÂ² dissipated on clips +
      77,634 air shortfall in one 405-tick run; boards at âˆ’206
      (regret 42.6k, tangency 0 was available!) and âˆ’142 (regret
      20.3k, ditto). `ledger-trace` audits the 7 expert demo CSVs
      (energy-model-break events per snapshot pair) â€” CAVEAT: on the
      big maps those events mix real clips with teleports/boosters;
      classification needs map geometry (deferred to M5.1). The
      apples-to-apples expert comparison happens on shared maps.

## M2 â€” Route search (stage 2)

- [x] 2.1-2.3 Route search (`SolverRouteSearch.h/.cpp` + `routesgate`):
      best-first enumeration over feature sequences under THE POTENTIAL
      LEDGER â€” total energy obeys E' = E + 900Â·ticks + 2gÂ·Î”z for
      flights and rides alike, anchored per node (anchors telescope;
      cycles net exactly their wish work, killing the corner-bounce
      energy exploit that broke two earlier attempts). Edges = M1
      closed forms: ballistic z-window Ã— gain-law distance coverage
      from the departure anchor; ride traversal priced arrivalâ†’
      departure anchor at ledger speed; the END edge allows LANDING
      SHORT + RUNNING the remainder (the human line's final leg).
      START = measured prestrafe ceiling (certified-sim circle-strafe
      probe Ã—1.15, no fitted constant) + the single legal jump folded
      into E. Skips are first-class (the search REJECTS startâ†’1/2/3
      as unreachable without the face-0 board â€” correct physics).
      Ranking = best lb per DISTINCT BASE SHAPE (first-occurrence face
      order; cycle variants and multi-taps collapse â€” the pool stage 3
      consumes). GATE PASSED: 60 raw routes â†’ 10 shapes in 0.00s;
      the pool contains the HUMAN shape [0 2 3], solved12's [0 1 2 3]
      (rank 7), the old solver's [0 2], and the speculative one-ride
      [0] (rank 4 â€” no real run validates it; worth probing in M4).
      CORRECTION 2026-08-16: a session mislabeled Run 21.tas as "the
      human line" (one ride, 71k dissipation, pit finish) and briefly
      made it the benchmark. Run 21 is a DEAD TAPE â€” it rides face 0
      into the map FLOOR (z âˆ’1056 = brush 1's top) and never finishes;
      its header says surf_basictest only because it was recorded
      in-game there. All conclusions drawn from it are PURGED. The
      REAL human run is `basictest.tas` (user-confirmed; see M4).

## M3 â€” Transfer refinement & assembly (stage 3)

- [~] 3.1/3.2 Assembler (`SolverAssemble.h/.cpp` + `msolvegate`) â€” IN
      PROGRESS. Built and working: shape iteration from the M2 pool;
      START plan search scored by the RESULTING BOARD via probe air
      solves (launch-speed-toward-the-face was the head-on plunge
      setup: dot âˆ’458 â†’ âˆ’59.8 when fixed); region-mode air targets
      (miss gradient to the nearest face point â€” point pursuit fights
      tangency); graze-through flights (a non-target clip continues
      the flight; ending it killed chains 37u short); TAP TRANSFERS
      (striking the next leg's face IS the transfer â€” the 3-tick
      clean-air exit is impossible between adjoining valley faces);
      the UNIFIED TRANSFER primitive (tap mode: the ride flows through
      exit + flight and is scored by the next strike â€” the design's
      stage-3 unit); zone proxy + ZoneTick + ledger comparison + .tas
      export all wired.
      SESSION 2026-08-16b (tapprobe-driven, every step measured):
      the unified transfer NOW SOLVES the human's own transfers -
      `tapprobe` (new isolation instrument: replay any tape to a
      tick, run the tap/zone solve from that exact state) showed the
      0â†’2 valley transfer unseeded at dot âˆ’162.8 @ 887 u/s (human:
      âˆ’206.4 @ 874 on the same face pair) and the face-3â†’zone ENDING
      in 101 ticks (human: 123 from the same entry). What it took,
      in order (each verified by the probe): (1) tap target = the
      next face's BOARD REGION (aim_region math, not corner points);
      (2) FRONT-SIDE-ONLY miss gradient (behind-plane closeness is
      not approach â€” kills the corridor/under-dive traps); (3)
      graze-through in the flight section (only the tap face ends a
      tap; zone mode grazes everything); (4) ballistic REACH-
      SHORTFALL term at separation (M1.1 closed form: prices
      "separate higher/ascending"); (5) S-CARVE families (dive then
      up â€” the measured human shape: final ride heading +8Â°, exit
      ascending +128 vz off the face edge); (6) DUAL SPLINE DOMAINS
      in SolveCarve (est and est/2; zone estÃ—1.7 â€” the M1.3 dead-
      knot lesson recurs because the ride doubles the speed); (7)
      6 knots for two-phase transfers; (8) 12k carve evals (probe-
      measured: âˆ’395@4k â†’ âˆ’163@12k); (9) ZONE MODE = the last
      transfer ends by entering the end volume, scored by arrival
      tick (the objective itself), no more FlyToZone dependence;
      (10) board ALTERNATIVES (SolveTransfer returns top-K diverse
      hits; the leg composes each with its carve â€” a dot-minimal
      board measurably set up âˆ’400 taps where a harder board set up
      âˆ’110); (11) rolling-context NEXT_COST (exact climb law
      vâˆ’âˆš(vÂ²âˆ’2gÂ·sh) toward the leg-after target).
      In-assembler best so far: [0 2] leg 0 at âˆ’110.3 @ 877.
      REMAINING (the one blocker): chain CONSISTENCY - tap landings
      cluster at the bottom-west of each face where the NEXT
      transfer has no runway; min-over-verts reach can't
      discriminate (the lowest vert is always reachable). NEXT: the
      design's rolling window applied literally - depth-2
      composition (probe the next leg from each candidate landing
      before committing; beam 2-3 per leg). All primitives are
      proven capable; this is selection, not capability.
      All prior gates still pass after every change this session
      (airsolve 12/12, carve 10/10 re-run after each edit).
      GATE (M3): unseeded finisher on basictest in < 2 minutes wall
      clock; its ledger strictly dominates the old solver's best line
      (fewer board losses, less approach regret, fewer ticks).
      ZONES RESOLVED 2026-08-16 (user: "the one with the red texture
      is the end zone" + BSP texture data): START = brush 6 (carries
      CABLE/GREEN, the spawn platform), END = brush 10 (carries
      CABLE/RED, reflectivity 0.511/0.002/0.002 â€” the raised platform
      z 192..256). The assembler's InZone target (brush 10) was
      already correct. Texture identity now loads from the BSP
      (texinfoâ†’texdataâ†’string lumps, per-side `texd` +
      `World::texnames/texreflect`; `mapinfo` prints per-brush
      textures) so zone identification is map data, not lore.
      `tapeinfo` (new) prints every tape's map/frames/anchor â€”
      the provenance check that would have caught the Run 21 mixup.

## M4 â€” Polish & anytime behavior (stage 4)

- [ ] 4.1 CMA-ES residual polish on yaw splines (boundary-locked,
      seeded from 3.1, never random).
- [ ] 4.2 Ledger-directed re-solve: budget flows to the worst transfer.
- [ ] 4.3 Anytime toggles: beam width, window depth, polish rounds;
      2-minute vs 20-minute dial demonstrated.
      GATE (M4): **beat the human tape's zone time on basictest,
      unseeded.** The 2-min setting lands within ~5 ticks of the
      20-min setting.
      THE MISSION IS THE SOLVER, NOT A MAP (user, repeatedly): the
      gate is generic â€” on any map with a human/reference run, the
      unseeded solve beats it. Per-map numbers below are VALIDATION
      INSTANCES only; nothing in the solver may reference them.
      basictest instance (measured 2026-08-16, user-confirmed tape):
      `basictest.tas` = the human run. Zone clock **319 ticks
      (4.785 s)** from start-zone exit (t112) to grounding on brush
      10 (t431). Route shape [0 2 3] (skips face 1). Max 915.9 u/s.
      Its ledger: boards âˆ’115.1 (regret 5.4k), âˆ’206.4 (regret 42.6k,
      tangency 0 available), âˆ’142.4 (regret 20.3k); whole-tape
      dissipation 343.9k + air shortfall 138.7k. The old solver's
      best line (cma_S292) scored 292 ticks â€” already faster than
      the human â€” so this instance's bar is < 292 ticks unseeded,
      with the human's 68k board regret as energy headroom.

## M5 â€” Generalization

- [ ] 5.1 Second linear map, brush-geometry only, end-to-end unseeded.
      GATE: finisher + clean ledger, no code changes specific to the
      map.
- [ ] 5.2 (!) Displacement collision in the world model â€” parity-side
      prerequisite for most of the map population (tracked as parity
      open item; build when a target map demands it).
- [x] 5.3 Expert demo ingestion - CAPTURED (took 4 rounds; each failure
      root-caused from the evidence log: render-thread transitions, menu
      drain gap, UTF-8 BOM in the queue file, obs-netvar unreliability
      in TV demos -> runner found BY NAME via runtime-validated
      GetPlayerInfo, stopdemo instead of mid-demo disconnect). All 7
      traces in solver\demo_traces\*.csv with *_run.csv run segments
      (Tools/demo_run_extract.py: dense+moving signature selector).
      Six tight (run time + 2-3s float); facility_Paddy partial (7.9s,
      target-switch contamination) but facility is fully covered by
      m@'s faster run. IF ever re-capturing: derive per-demo target
      priority from the queue filename. Expert-ledger comparison awaits
      M1.5.
- [ ] 5.4 Staged maps: per-stage chaining with teleport-reset
      boundaries and per-stage prestrafe.
- [ ] 5.5 Solve-time hardening: worst-case < 30 min across the test
      set; profile and push down. Throughput benchmark of the
      leaf-ordered trace under solver load (carried from parity era).

## M6 â€” In-game validation (batched; respect the relaunch fatigue)

- [ ] 6.1 Export candidate lines and validate IN ONE SESSION per batch
      (playback capture, diff 0.000u required).
- [ ] 6.2 Only then: showcase-quality review against the aesthetic
      rate-limit rule.

## Cross-cutting rules

- Every module validates against the certified engine or the tapes
  BEFORE anything builds on it (the strafelaw pattern: prover first).
- No fitted constants in scoring â€” bounds derive from the engine model;
  the ledger measures regret, not vibes.
- No per-map tuning anywhere. If a map needs special handling, the
  design is wrong.
- Jumps: exceptional route edges only; spine-bhop hard-filtered.

---

## Change log

- 2026-08-20d (session 20, THE SEARCH ARCHITECTURE CORRECTION + THE
  forwardfield AUDIT): user withdrew confidence in the search
  architecture ("it isn't fulfilling the promise of deterministic,
  cheap evaluation that can be exhaustively searched"); advisor
  2026-08-20d: the simulator is cheap, the FORMULATION is expensive -
  RefSolve (inverse boundary solver, a capability-test tool) was
  promoted into the production field engine; the heatmap must be the
  OUTPUT of one forward reachable-state sweep, not thousands of
  per-ball solves; pause batching/launch/service/G0R; simulator
  accuracy is never traded for throughput. LANDED: (1)
  `g_movetick_count` in MoveTick - observability only, no-drift
  proven by groute 11/11 + the airrec fixture hash bit-identical;
  (2) the `forwardfield` audit command - deterministic real-problem
  construction (yaw 0/15 roots -> f0 board -> exit selected by the
  composite vertical-window + certified-reach-cone score) and both
  arms on one problem: v0 forward sweep (per-tick action set
  {coast, +-side x 4 cosa}, dwell-6 gating, (xy, heading, speed,
  side, age) binning with 2 exact reps, compact lineage witnesses,
  200k frontier cap with REPORTED overflow) vs the production
  RefSolve ladder (3 aims x 600/1800/3600, strikes via on_strike).
  MEASURED: audit run 1 - the flat/high prior picked a 3-tick-window
  dive; both arms burned everything for zero strikes (priors are not
  reachability; cancelled terms must precede simulation). Run 2 -
  the window rule made the dead sweep instant; window-only selection
  still picked an unreachable exit; the composite score found the
  real transfer (yaw-15 root, 601 u/s). Run 3 (the head-to-head) -
  forward: ~50-58M MoveTicks/sweep, 40-48 s, strikes 1.9-6.2M, cells
  66->1105 across 32->2u with exact work FLAT (PASSES the advisor's
  scaling criterion); RefSolve: 18k evals = 1.14M MoveTicks = 0.77 s,
  12,183 strikes, 331 cells at 8u, ~50x better MT/cell on this easy
  target and beats forward in 27 shared cells by up to 302k E (the
  oracle role works). The named core research problem: the state
  equivalence/partition rule (6.2M strikes -> 1105 cells; 10.2M cap
  overflow). Throughput truth: simulator ~1.35-1.5M MoveTicks/sec
  single-thread (~63 ticks per RefSolve eval); the old
  billions/minute were arithmetic candidates - the funnel's
  arithmetic layer already runs at that scale. User targets: real
  maps <10 min (~30 exhaustive), basictest -> seconds-to-<1-min.
  Milestones: A0 delivered; G0R deferred behind the reconstruction.
  Thirteen suites green. Full tables: `Docs/ExitFieldSpec.md`
  session 20 + the standing SEARCH ARCHITECTURE CORRECTION section.

- 2026-08-20c (session 19, THE RESOURCE-PRESERVATION SERVICE LANE -
  G0Q directionally confirmed; the 84% K-duplication ceiling): advisor
  ruling 2026-08-20c ("route quality enters the SERVICE layer - as an
  independent physical-resource lane, not a new objective or a
  speed-biased admission score"). User performance north star
  recorded: basictest <= 5 min wall (goal sub-1-min); real maps 30+
  ramps / 60+ s. THE CHANGE (service only; objective stays ticks;
  operators/launch/topology/horizons/aims frozen): within the
  face/position-bucket deepen rotation, two lanes per bucket - the
  OBJECTIVE representative (min g) and the RESOURCE representative
  (max exact E(B) via the one certified ExitField::EBoundary) -
  least-served lane next, deterministic tie to objective; the
  conceptual advisory Pareto service frontier in (g, -E); no blending,
  no thresholds, nothing pruned or dominated; legacy session-18
  deepening preserved behind --svc18 as the A/B arm. NEW INSTRUMENTS:
  per-face board-E median/max + g@maxE + per-bucket max-E; per-face
  distinct exit capability (speed, z); lane attribution; first-witness
  snapshots; and the K-DUPLICATION measurement (queries grouped by
  exact (air start state, target face, horizon) across aim/region
  queries). THE CAUSAL A/B (3600s vs 3600s, same binary): resource
  arm - f2 exits max 606 u/s @ z 157 (bridge-class, inside f3's
  window by the arithmetic) vs baseline 510 @ 125; f1 median board E
  238k vs 192k (+24%); f2 max E 268k@g500 vs 254k@g355; cost - f2
  first witnessed 1265s vs 690s (march speed traded for capability);
  f3 NEVER witnessed in either arm; G0R open. THE DECISIVE NUMBER:
  K-dup repeat share 84% vs 85% - POLICY-INVARIANT; both arms
  re-solved ~5,400-6,000 distinct physical problems 4.2-4.4x over
  (38.7M / 47.2M evals). The ~6x batched-witness-field amortization
  is now measured justified as the post-G0R project and the only
  path to the 5-minute budget; the advisor's conditional generous
  launch-capability probe stays open as the second lever. Suites:
  groute 11/11 and the full thirteen green from the session binary.
  Full A/B tables: `Docs/ExitFieldSpec.md` session 19.

- 2026-08-20b (session 18, G1M DELIVERED / G0R NOT REACHED - the
  first real-map global runs): advisor ruling 2026-08-20b ("treat
  this first run primarily as an instrumented global-search
  experiment... do not only report the final route"). PRE-RUN
  ADDITIONS: G13 horizon-boundary gate (groute 11/11 - the
  "< incumbent" convention verified against the real executor at
  T*-1/T*/T*+1: M = dt_e admits the END booking at exactly tick M, no
  off-by-one; ties/slower excluded and refused); DomainAudit stable
  unresolved-domain identities (FNV over node/kind/face/variant,
  statuses unexplored/unresolved/witnessed - no "impossible" -
  outcome reason codes, attempts update never re-identify);
  topology falsification counters (0 violations ever; the
  distance-gated Route prediction measured UNSAFE - thousands of
  realized self-transfers it would have missed); GlobalRouteWitness
  (Launch + TransferEdges + END) with continuous one-PlayerState
  replay; LaunchWitness + ReplayLaunch; END-VIA-AIR (FlyZoneSchedule,
  kEdgeEnd with dt_air, extended ReplayEdge; sound Minkowski-
  intersection END box for brush 10, stated standing-only underside
  gap). THE gmap COMMAND: cold instrumented explorer on basictest -
  real spawn anchor, 24-direction production launch fan, all-faces
  conservative topology, persistent exit (qcache) AND entrance
  (ecache) lazy queries, deterministic two-lane service. EIGHT RUNS,
  EIGHT MEASUREMENTS (each fix ordering/coverage only, physics and
  operators untouched): (1) launch starved - the acceptance ball is a
  REGION query; BallisticAim (coast-arc plane crossing) unlocked 6
  exact f0 roots; (2) min-g service floods same-face hops; (3) TWO
  self-inflicted regressions caught by the ledger - speed-scaled
  horizons cut fast nodes to M 105 where the f0->f1 witness needs
  ~160+ (fix: geometry-only nh at 300 u/s reference), and pure
  breadth never ladders to the finding profiles; (4) f0->f1 restored
  (6 edges, fine, M 192) but the flat/high exit prior is
  ANTI-correlated with f2's south-diving feeders (fix: exit diversity
  buckets + long-axis spread aims + defer-far); (5/6) --diag-exits
  showed the deepest f1 node boards at the far WEST end - min-g
  laddering only ever deepens a face's earliest boards (fix:
  within-face position-bucket deepen rotation, 2:1 deepen:breadth);
  (7) THE MARCH WORKS - f0(346) -> f1(452) -> f2(324) autonomously,
  the east-f1 board's f2 domain carried 62 edges at coarse/medium;
  (8, 3600s) 336 expansions, 1940 states (f2 865), 1944 edges ALL
  cold-replaying bitwise, f3 still zero: SPEED COMPOUNDS ACROSS HOPS
  - cheap short-hop transfers are found first, 300-500 u/s f2
  arrivals fall ~350u across the 300u gap and physically cannot reach
  f3's window (~600+ u/s at height needed); the deficit chains back
  to the 250 u/s run-off launch. G1M HEADLINES: entrance refinement =
  98-99.5% of ALL wall-clock everywhere (43.4M evals/hour; exit/
  probes/bookkeeping < 0.2s each) - nothing else is a measurable perf
  target; same-face hop proliferation dominates state-space and
  exact-dup dominance nearly never fires on real geometry (13 prunes
  / 1940 states); competitive horizons never engaged (no incumbent) -
  the T* feedback loop is unexercised. G0R recorded NOT REACHED,
  plainly; the blocker is a route-quality ordering dynamic for the
  advisor to rule on, not aims/horizons/resolution/topology (each of
  those was found and fixed above). Clean-rebuild board: groute
  11/11, faceleg 6/6, exitlazy 8/8, exitenv 8/8, exitfrontier 9/9,
  exitfit 10/10, efrefine 7/7, airprops 24/24, airsuite 48/48, airrec
  VERIFIED, wishparity, carve M1.4, airsolve M1.3. Full run log:
  `Docs/ExitFieldSpec.md` session 18.

- 2026-08-20 (session 17, G0: THE GLOBAL EXPLORER - 10/10; "the
  rebuilt operators autonomously form real routes"): advisor ruling
  2026-08-20. `Source/Solver/SolverGlobal.h` is the full-map solver
  substrate: Node = exact BoardBoundaryState + exact ticks g, with
  `succ_complete` ALWAYS false in v0 (the law "expansion != successor
  completeness" lives in the type); TransferEdge = the production edge
  contract (ride packets + optional Air schedule + destination +
  provenance, cold-replayed bitwise by ReplayEdge); cost is TICKS ONLY
  with h_order/h_cert separate (h_cert = 0); the one certified
  elimination is exact duplicate-state dominance with PruneProof
  records; OfferFinish is the single incumbent path; HorizonFor derives
  competitive horizons from T* - g - 1 (physical restriction, not
  budget). THE FIXTURE (all engine-generated): two wedges across an AIR
  GAP (crease ping-pongs 1-tick - measured), a 2-launch production
  LaunchFrontier with exact launch ticks, END zone placed by an engine
  probe (+-56 mid-path; +-90 swallowed the board region, +-48
  quarter-point never crossed - measured). THE RUN: 8 nodes, 17 edges,
  9 dominance prunes; the explorer autonomously assembles LAUNCH ->
  B_0(A) -> [exit/air/board] -> B_1(B) -> [ride] -> END at T* = 66.
  GATES (10/10): all edges cold-replay; the route replays continuously
  (57 ticks == sum dt, g(B0)+route == T*); dominance with proofs
  (differing ctl.age NOT merged); succ_complete false everywhere;
  re-refinement discovers new edges (2 -> 4); production incumbent
  (equal/slower refused); horizons <= T* - g - 1; INVERTED expansion
  order changes nothing physical; no tape; cold route rebuild.
  G8'S HONEST CAVEAT (open obligation, recorded so it cannot silently
  become "demonstrated"): this fixture admits exactly ONE finish time -
  every discovered finish and constructed variant ties (measured across
  zone sizes/placements/wish variants/deeper refinement) - so
  faster-replaces-incumbent is verified at the CONTRACT level with a
  labeled synthetic offer; the natural demonstration transfers to the
  first real-map run. G0 deliberately does NOT claim optimality
  (h_cert = 0), successor completeness, or a real-map route. NEXT: the
  basictest full run (real spawn LaunchFrontier, real end zone, G8
  natural replacement), then MEASURE the explosion before building the
  weakest useful certified remaining-time bound (G1). Standing
  prohibitions honored: no ExitDoomed certification, no approximate
  global dominance, no beams/top-K/ML. Clean-rebuild board: thirteen
  suites green (groute 10/10, faceleg 6/6, exitlazy 8/8, exitenv 8/8,
  exitfrontier 9/9, exitfit 10/10, efrefine 7/7, airprops 24/24,
  airsuite 48/48, airrec VERIFIED, wishparity, carve M1.4, airsolve
  M1.3). THE PROJECT IS NOW A PATHFINDING ENGINE SUBSTRATE - the
  full-map solver has begun. Full detail: `Docs/ExitFieldSpec.md`
  session 17.

- 2026-08-20 (session 16, PROOFS REPAIRED + STAGES 5-6: THE FIRST
  REUSABLE FULL-MAP EDGE EXISTS): advisor 2026-08-19j. PROOF REPAIRS
  then promotion: (1) the wish derivation's invalid intermediate
  (dE <= c^2 - p^2, wrong for p < -c) replaced by the clean chain
  p <= c - a => dE <= c^2 - (a-c)^2 <= 900, braking included; (2) the
  FP slack is DERIVED (conservative op inventory under certified
  magnitude clamps: |E| < 2^27, ulp 8; inventory < 332, certified 512
  per tick with headroom - E1's observed 0.5 stays as evidence, never
  justification; E8's mutation detection survives the looser slack);
  (3) the clip premise is analytical (beta(beta-2)(v.n)^2, beta = 1
  exactly in the helper). exitenv 8/8 -> **bound 45580001 CERTIFIED**.
  STAGE 5 (exitlazy 8/8): `ExitField::ExitQuery` with the complete
  fixed domain key (exact B bytes + ctl + face + M + WorldIdent +
  ProvenanceHash); W_known PARTITION-INDEPENDENT and append-only with
  `Rebin` a pure derived view (measured: knowledge byte-identical while
  materialization churns 21 -> 18); honest statuses with NO
  RESOLVED/COMPLETE; provenance mismatch REFUSES; (B, 40) vs (B, 80)
  are different domains (keys split, U_E may grow with M, M immutable
  through refinement); deterministic profile history.
  STAGE 6 (faceleg 6/6): 2 AIR legs B_i -> ExitField -> Air ->
  EntranceField -> B_j and a direct CONTACT_TRANSFER leg, ALL verified
  against one continuous exact-engine replay: composed B_j BITWISE
  equal, dt_leg = dt_exit + dt_air with no +-1 (measured convention:
  Air::Result::tick counts through the strike tick), every ride tick
  except separation touches face i and exactly one board contact on
  the final tick, the dwell law survives both seams (independent
  whole-leg walk), two distinct exits to one face with distinct B_j
  both kept, every edge rebuilds cold, no tape on the path.
  COMPOSITION LESSONS (found by the continuous-replay gate failing
  honestly): a 700 u/s into-face board is a faceplant (exits ~200 u/s,
  nothing composes); down-slope rides exit low/diving and fall below
  every board window - the composable exit class is FAST, FLAT, HIGH
  (the human's 94-tick transfer flew ~1000u nearly level; lateral high
  boards produce it); the entrance flight horizon must come from
  GEOMETRY (fixed n_hint 60 found zero strikes ever; real transfers
  need 100-160 ticks) - the competitive-horizon argument arriving
  early. These preview the GLOBAL layer's job (B_i/aim/horizon
  selection is route planning). Clean-rebuild board: faceleg 6/6,
  exitlazy 8/8, exitenv 8/8, exitfrontier 9/9, exitfit 10/10, efrefine
  7/7, airprops 24/24, airsuite 48/48, airrec VERIFIED, wishparity,
  carve M1.4, airsolve M1.3. THE PROJECT STATE: both operators exist
  as certified, witnessed, composable pieces and a face-to-face edge
  has been constructed, verified bitwise and rebuilt cold. NEXT: the
  global graph/best-first layer over these edges. Full detail:
  `Docs/ExitFieldSpec.md` session 16.

- 2026-08-19 (session 15, THREE CORRECTIONS + U_E(B, M) CERTIFIED):
  advisor 2026-08-19i. CORRECTIONS: (1) partition keys carry SPATIAL
  REGIONS in face-local frames (RegionBits, 128u cells; AIR keyed on
  the ridden face's exit region, TRANSFER on the target face's contact
  region) - two exits from opposite ramp ends are now separately
  requestable subdomains; (2) reopen provenance is VERSIONED
  (kProposalVersion + ProvenanceHash over proposal/engine/control/
  collision identity and params) - a deferred hash is regenerable only
  under the same provenance; (3) F6 strengthened to "the composed event
  IS the earliest post-handoff boundary of an instrumented continuous
  run" (same tick/kind/face, none skipped) and F9 pins deterministic
  simultaneous-event precedence END > GROUND > TRANSFER > AIR with a
  genuine same-tick fixture. exitfrontier 9/9. F8's 7/8 left alone per
  directive.
  STAGE 4 (exitenv 8/8): U_E(B, M) = E(B) + cap^2 M + 2 g gs
  duck_air_shift + eps M is CERTIFIED (bound id 45580001, domain
  D_static-surf: static geometry, constant gs, basevel 0, no triggers,
  wish+duck inputs, booked ticks airborne). PHASE FIRST: E := |v|^2 +
  2 g gs z at the exact s_plus boundary; the leapfrog conserves E
  exactly in real arithmetic (E1: realized drift = one float ulp/tick,
  eps 2.0). THE WISH LAW IS DERIVED IN THE RIDE DOMAIN, NOT COPIED:
  dE = 2a proj + a^2 with a <= cap - proj gives dE <= cap^2 = 900 for
  any legal wish at any wishspeed/friction/speed; E2 measured the
  realized one-tick max at 899.00 of 900 over 576 wish ticks - tight.
  Clips never add energy (E3 worst expansion 0.0000 over 2000). DUCK
  ORIGIN IS ONE NET SHIFT, not per-tick slack: transitions alternate so
  the prefix sum of +-8.5 shifts is at most one +8.5 (E4: duck-tick
  jump +14,486 measured, envelope holds, margin 2191). E5: 1,918 booked
  boundary states, 0 violations. E6: every frontier transition
  contained at its own dt. E7: THE HORIZON LAW - M is part of the QUERY
  DOMAIN, not the refinement profile; R(B,40) subset R(B,80), so U may
  legitimately grow with M; "W grows, U tightens" holds only for fixed
  (B, M); ExtendHorizon is explicit domain expansion, never a profile
  side effect (now standing in ExitFieldSpec). E8: removing the hull
  term turns the suite RED and the measured 899.0 proves a 50-point
  wish-law understatement is detectable - the suite can notice broken
  semantics by construction. Clean-rebuild board: exitenv 8/8,
  exitfrontier 9/9, exitfit 10/10, efrefine 7/7, airprops 24/24,
  airsuite 48/48, airrec VERIFIED, wishparity, carve M1.4, airsolve
  M1.3. NEXT: stage 5 lazy Query/Refine for fixed (B, M) (W_known never
  shrinks vs W_materialized may recompress), then stage 6 - the
  composed transfer with exact tick accounting and packet
  concatenation, plus B_i -> CONTACT_TRANSFER -> B_j with no fake Air
  segment. Full detail: `Docs/ExitFieldSpec.md` session 15.

- 2026-08-19 (session 14, STAGE 3: THE EXIT WITNESS FRONTIER - 8/8):
  advisor ruling 2026-08-19h implemented. The distinction is in the
  DATA MODEL: `ExitField::WitnessFrontier` (SolverExitField.h) is the
  witnessed LOWER approximation W_F(B) of the operator R_F(B) - the
  set-valued analogue of EntranceField's L; the status enums
  (kUnexplored/kActive/kCompressed) have no "dead" or "impossible"
  value BY CONSTRUCTION, and absence from the frontier never means
  unreachable. GROUND partitions carry continuation_unsupported = true
  (exact continuation preserved; the global layer may not prove a route
  irrelevant because its next operator is unbuilt; fail-zone vs
  optional ground contact is map semantics, deliberately undefined
  now). COMPRESSION IS RESOLUTION ONLY: diversity slots (earliest /
  fastest / highest-vz / max-energy; no single local objective) cap at
  2+2*level per partition; displaced or capped members become DEFERRED
  HASHES; reopen provenance is the deterministic proposal enumeration
  itself (MakeProposal(index) is pure) - refinement re-enumerates,
  dedupes by witness hash, and deferred members MATERIALIZE under the
  larger cap. Proposal families (72/132/204 at L0/1/2): constant-side x
  cosa x horizon + duck-hold, single reversal, coast-prefix + duck-off,
  finer cosa + late reversals - all through SchedLegal + the canonical
  executor; no Carve, no controller, no tapes, no ExitDoomed.
  GATES (exitfrontier, 8/8): F1 25 active members replay BITWISE to
  (kind, dt, S+); F2 event typing + GROUND flags; F3 monotone knowledge
  (7 -> 31 known, nothing lost); F4 3 compressed partitions all
  carrying deferred hashes - status says COMPRESSED, nothing says
  dominated; F5 reopening (4 -> 6 actives, a previously deferred member
  materialized); F6 TRANSFER COMPOSITION PARITY on the synthetic valley
  - handoff S+ BITWISE, composed suffix bitwise for every booked tick,
  and the composed run's lawful EARLY STOP (crease ping-pong: the hull
  touches wedge A again while riding B - a real transition) matches a
  real contact in the continuous run; F7 quarantine (ExitDoomed cull
  toggled -> hash-identical frontier); F8 recoverability smoke 7/8
  hidden off-family rides recovered (the miss - a short-duration
  heading bucket - is the smoke test exposing a family hole, reported).
  Gate-level findings: F6's first failure was the GATE's semantics, not
  the seam's (early stop is lawful); compression status must mean "has
  deferred members" however they got there (displacement-deferrals were
  unmarked - fixed, with reversion to kActive when the deferred list
  empties). Clean-rebuild board: exitfrontier 8/8, exitfit 10/10,
  efrefine 7/7, airprops 24/24, airsuite 48/48, airrec VERIFIED,
  wishparity, strafelaw, carve M1.4, airsolve M1.3. NEXT (stage 4): the
  first certified outer-envelope component U_E(B, M) from the RIDE
  ledger laws (not copied from the Air law), certified as "for all Z
  with dt <= M: E(S+) <= U_E"; loose acceptable. Then stage 5 (lazy
  Query/Refine: W grows, U tightens) and stage 6 - the first composed
  transfer B_i -> ExitField -> Air -> EntranceField -> B_j. Full
  detail: `Docs/ExitFieldSpec.md` session 14.

- 2026-08-19 (session 13b, THE FIVE PRE-STAGE-3 CORRECTIONS - exitfit
  10/10): advisor directive 2026-08-19g implemented in full. (1) THE
  AIR_EXIT SEAM IS CLOSED UNDER COMPOSITION: the session-13 peek rule
  is SUPERSEDED - the ride OWNS AND BOOKS the separation tick (dt
  includes the first clean-air tick, S+ is the boundary after it, Air
  begins at the next tick); an exit classified by a replaceable
  unbooked input proved only "exists u: next tick separates". Every
  event kind books its own tick uniformly via the shared ClassifyTick
  helper, so the two executors cannot drift. Measured: X7's carve seam
  difference (t25 vs t26) disappeared exactly. (2) SEARCH COORDINATES
  vs EXACT WITNESS: `Ride::MoveInput` freezes precisely the MoveTick
  arguments; `FlyRideInputs` is the authoritative packet replay. X1b:
  canonical packets AND native packets replay BITWISE, 0 bad over 36
  fixtures - the 0.7u inversion drift lived in re-emission and the
  packet witness removes it. (3) COAST-THROUGH-DWELL gated (X3):
  reversal after 2 coasts on age 1 refused at the right tick, after 3
  accepted; coast-tick cosa variants ride bitwise identically and
  CanonSchedule pins ignored fields. (4) DETERMINISTIC EVENT COVERAGE
  (XE): GROUND x2 natural; END via a zone volume (t6); CONTACT_TRANSFER
  measured impossible in ride scope on available real maps (24/24
  basictest adjacency probes end in AIR; surf_climb extracts no faces;
  surf_speed has 2 without contact) so per the advisor's directive a
  SYNTHETIC two-wedge valley (World::AddTestBrush/FinalizeTestWorld,
  harness-only, additive, exercising the leaf-less grid-fallback trace
  path) delivers it: 20 face ticks then contact with the other wedge at
  t21, board_face resolved. HORIZON refuses MakeTransition. (5) TYPED
  TRANSITIONS: `Ride::ExitTransition` + `MakeTransition` (kind, exact
  dt, s_plus + ctl_plus, board_face, packet witness); the assembler
  dispatches on kind (AIR_EXIT -> Air/Entrance, CONTACT_TRANSFER ->
  already a board contact, GROUND terminal, END finish, HORIZON no
  transition). Honest note: the cosa chart quantizes angle ~sqrt(ulp)
  near degenerate directions (8.3e-05 rad measured at |cosa| ~ 1 vs
  5.25e-06 in-band) - a property of the parameterization, irrelevant to
  packet-exact witnesses; X1 gate is angle-aware (2e-5 / 2e-4).
  Clean-rebuild board: exitfit 10/10, efrefine 7/7, airprops 24/24,
  airsuite 48/48 gap 242k, airrec VERIFIED, wishparity, strafelaw,
  carve M1.4, airsolve M1.3. STAGE 3 IS SAFE TO BEGIN: witness-backed
  event-partitioned frontier, proposals order-only, reopenable
  compression, no approximate dominance, ExitDoomed advisory. Full
  detail: `Docs/ExitFieldSpec.md` session 13b.

- 2026-08-19 (session 13, EXITFIELD STAGES 0-2: the operator is defined,
  the canonical executor exists, and the representation gate is GREEN
  after three real first-run findings). New design of record:
  `Docs/ExitFieldSpec.md` - ExitField is the SET-VALUED exact transition
  relation `R_F(B) = {(dt, e, S+, U_R)}` to the first boundary event
  (AIR_EXIT / CONTACT_TRANSFER / GROUND / END / HORIZON); scalar max
  exit energy is a dashboard projection; HORIZON => UNRESOLVED, never an
  exit; standing laws B-A..B-F filed (ExitDoomed advisory-only and
  banned from the canonical executor; set-valued certified envelope
  `U_E(B, M)`; witness = consumed prefix; reopenable compression, no
  approximate dominance; naming quarantine - `Field::ExitMap` is an
  advisory estimator, classified at the source; human/carve rides are
  regression only).
  STAGE 0 (Entrance continuation API): `RefResult` gained per-interval
  witness lanes; `BoardTransition` (stable lane ids: winner / tangent /
  rung r / interval i) + `QueryState::transitions` merged monotonically
  per lane (a lane once exposed is never lost, its L only rises, replay
  horizon derives from the witness's own length);
  `ReplayBoardTransition` is the one sanctioned way to obtain S_B.
  **Gate R7: 10 lanes, 6 heading intervals, all replay to their
  recorded L at |dL| 0.00** - BestBoard is a projection, the witnesses
  are the payload.
  STAGE 1 (canonical executor): `Source/Solver/SolverRide.h` -
  `Ride::FlyRideSchedule` over per-tick `(side, cosa, duck)` with
  side=0 the coasting channel and optional analog magnitude; emission
  routes through `Air::WishInputs`; `Fn::WishFromInput` exposed as a
  forwarding mirror so the inverter (`Ride::InvertWishInput`) cannot
  drift from the tick; `Ride::SchedLegal` validates the dwell law
  against the CARRIED CtlState and refuses illegal schedules (refusal
  is a legality verdict, not an event). No controller, no splines, no
  ExitDoomed, no optimizer.
  STAGE 2 (`exitfit`, **7/7 GREEN**): X1a inversion identity 176/176;
  X1 control completeness on 42 hidden native rides across 8 strata,
  4 faces, 21 carried-ctl variants, 7 engine-built hull-2 entries, 2
  analog - wish-direction worst **5.25e-06 rad**, zero
  dir/class/duck mismatches, events exact, trajectory envelope worst
  0.72u of 2u; XH horizon-never-an-exit; X2a **126 interior checkpoints
  serialize -> reconstruct -> replay BITWISE equal**; X2b carried-dwell
  seam legality (corrupted-younger age refused at the right tick); X2c
  ducking/duck_timer_ms load-bearing (3/12 diverge) while hull-2 rides
  as standing per the DECLARED model (Hulls::unduck_* == stand_* since
  2026-08-15 - the gate asserts consistency, divergence would mean
  collision-model drift); X7 carve-ride native inputs invert+replay to
  the same exit (t25 vs t26 seam bookkeeping, 533.2 vs 533.6 u/s).
  THREE FIRST-RUN FINDINGS, all resolved by measurement: (1) an
  AIR_EXIT witness must carry the one-tick separation-witness INPUT
  (tick never booked; spec section 2 amended) or it cannot be
  classified standalone; (2) bitwise reproduction of native rides is
  impossible for any single-encoding basis (float ulp x clip
  amplification, 0.72u measured), so X1's claim was sharpened to the
  INPUT CHANNEL - direction to float precision, magnitude class, duck -
  with a 2u trajectory envelope; (3) poking `hull_state` up is not a
  legal transient (the engine normalizes it away, 0/12) - probes must
  use ENGINE-CONSTRUCTED transients, which exposed that hull-2 is
  collision-identical to standing by declared model.
  COVERAGE HOLES RECORDED: no fixture yet ends CONTACT_TRANSFER /
  GROUND / END (blind native generation on open ramps); a
  transfer-bearing fixture is REQUIRED before the stage-6 composition
  milestone. The keyboard input model (binary wish magnitude) is
  adopted explicitly per spec section 4; the analog channel exists and
  is measured but production does not use it.
  Whole board from a clean rebuild: exitfit 7/7, efrefine 7/7 (R7 new),
  airprops 24/24, airsuite 48/48 gap 242k, airrec fixture VERIFIED,
  wishparity PASS, strafelaw float-ULP, carve M1.4, airsolve M1.3.
  NEXT (stage 3): the witness-backed continuation frontier with
  reopenable partition compression + the first CONTACT_TRANSFER
  fixtures; then U_E(B, M) (stage 4), lazy Query/Refine (stage 5),
  face-to-face composition (stage 6).

- 2026-08-19 (session 12l, THE ANYTIME BOUNDARY MOVES UP A LEVEL -
  ENTRANCEFIELD IS INTEGRATION-READY): advisor ruling 2026-08-19d
  implemented. Sessions 12e-12k had fused two responsibilities - LOCAL
  TRAJECTORY OPTIMIZATION (legacy RefSolve is strong at it) and LAZY
  MONOTONE COMPUTE ALLOCATION (what the full solver needs). Nothing
  requires one piece of code to do both, so the anytime contract now
  wraps the strong local engine instead of replacing it.
  BUILT: `Entrance::RefProfile` (immutable versioned RESOLUTION levels,
  not truth levels: coarse 600/4.0u, medium 1800/2.0u, fine 3600/1.0u,
  exhaustive 9000/1.0u, each with a stable id); `Entrance::QueryState`
  (L + its replayable witness, certified U + the bound id that produced
  it, status, evals, ordered profile audit trail);
  `Entrance::RefineStep` - ONE ATOMIC ACTION that runs the next profile
  IN FULL with `scheduler = 0` and merges monotonically
  `L_new = max(L_old, L_run)`, `U_new = min(U_old, U_certified)`;
  `Entrance::MarkIrrelevant` - the ONLY door to PROVED_IRRELEVANT, which
  refuses unless a certified ceiling is actually beaten by an incumbent.
  The key realisation: `RefSolve(3600)` not being an action-prefix of
  `RefSolve(600)` does not matter - they are two DIFFERENT atomic
  actions, and the prefix law applies to the stream of actions, which is
  where the global solver needs it. Also extracted
  `Entrance::UCertifiedFace` (exact ballistic arrival-tick window +
  FinishGravity phase + acceptance-ball potential + interval-vs-broad
  attribution) so RefSolve and the refinement layer cannot drift.
  NEW GATE `efrefine` **6/6 green**: R1 L monotone (830k -> 1022k ->
  1030k -> 1030k, no witness forgotten); R2 U never loosens (1265k,
  bound id 55300001); R3 sandwich holds and TIGHTENS (gap 435k ->
  235k); R4 the final 57-tick witness replays to **1030k vs stored
  1030k, err 0.00**; R5 deterministic for the same action sequence
  (15,000 evals, 4 profiles, identical L/U/witness/profile-ids); R6
  **failure never becomes impossibility** - an unreachable target fails
  all 4 profiles and still returns status UNRESOLVED with a certified U,
  and `MarkIrrelevant` is refused below U and granted above. R1 reaching
  1030k is the point: the wrapper exposes the strong engine's FULL
  strength rather than capping it.
  **ENTRANCEFIELD: INTEGRATION-READY.** The old requirement -
  scheduler-v1 must replace legacy RefSolve - is WITHDRAWN. The bar is a
  deterministic, monotonic lazy refinement API around a strong local
  solver, which `efrefine` certifies. Clean-rebuild board: efrefine 6/6,
  airprops 24/24, airsuite 48/48 gap 242k, airrec fixture
  `de1b000e431a84fb` VERIFIED capability 16/23/29, wishparity PASS,
  strafelaw float-ULP exact, carve M1.4 PASS, airsolve M1.3 PASS,
  humanexact f2 PASS. **EXITFIELD IS UNFROZEN.**
  CLASSIFICATION recorded at the `RefTune::scheduler` declaration itself
  so compaction cannot reinterpret it: `scheduler = 0` legacy RefSolve
  is the PRODUCTION local refinement engine, not deprecated and not
  scheduled for removal; `scheduler = 1/2` is an EXPERIMENTAL
  refinement-scheduler research path and the source of the scheduling
  laws now applied one level up, NOT a production replacement
  requirement. Scheduler-v1's 18/32 is not an unresolved prerequisite -
  it is the measured reason the path was not promoted. AirRec m-curves
  taken in scheduler mode are INVALID as m-curves (scheduler mode never
  consults `shoot_m`, so 17/18/18 is the same machinery three times);
  only the equal-compute 29/32 vs 18/32 comparison is meaningful.
  NEXT: ExitField, representation before optimization - `exitfit` must
  show a canonical ride basis `u_k = (side, cos alpha, duck)` carried
  with the full incoming boundary state exactly replays arbitrary legal
  rides (ordinary/short/long, climb/descend, high/low speed, reversals
  under the six-tick law, top/bottom/side exits, duck-off exits) before
  any ride optimizer is built. Exact replay is truth; `carve` is
  regression evidence; the corrected `Steer::Controller` is proposal
  machinery only. Full detail: `Docs/AirRecSpec.md` section 25.

- 2026-08-19 (session 12k, SCOUTCHEAP - the coverage cost is gone, and a
  SECOND capability gap surfaced): the one authorised focused change is
  built, gated and measured; the integration decision is still NO, for a
  reason unrelated to coverage. `A_COVER` (ScoutCheap) is now a distinct
  action identity from `A_SHOOT`: final target only, CLOSED FORM ONLY, no
  intermediate nodes / no second-stage exact ranking / no
  m1/m2/GN/deepen/precision. `shoot()` gained an explicit rollout-depth
  argument fixed by the ACTION, never the budget (escalation 4, coverage
  `RefTune::cover_top`, default 0). Coverage stays the deterministic
  prefix - one per unresolved domain before any escalation - so the
  prefix law is untouched (S1/S2/S3 green). **New gate S10 (coverage
  conservatism)**: at 40 evals all 6 actions are coverage and nothing
  else runs; at 1400 there are exactly 6, one per domain, costing 12
  evals; zero uncertified eliminations; escalation still reached. A
  coverage action that finds nothing yields UNRESOLVED and may NEVER
  conclude a heading interval is unreachable - only a certified bound
  eliminates a domain, which is precisely what makes it safe to be this
  cheap. Board **24/24** (P1-P7, S1-S10, B1-B7).
  **CORRECTION to session 12j.** That entry put the mandatory prefix at
  "240 of 600 evals (40%)", inferred from the CHARGED price
  `kCostShot = 40` (the runner's affordability reserve), not measured.
  Measured over all 48 AirSuite cases: full-price coverage = 6,680 evals
  = 139/case = **23.8%**; ScoutCheap = 576 evals = **12/case = 2.0%**.
  Real and large, just not 40% - and now **11.6x** smaller.
  CONTROLLED COMPARISON (identical fixture/methods/bounds, new
  `--sched` / `--cover-top` / `--suite-budget` flags): AirSuite @600
  legacy 48/48 (<=16u 40, p95 19.8u) | v1 full-coverage 39/48 (29,
  47.8u) | **v1 ScoutCheap 40/48 (35, 47.8u)**; @1800 48/48 for all
  three, <=16u 45 / 36 / **40**; @3600 all 48/48, <=16u 47 / 48 / **48**
  and ScoutCheap edges legacy on p50 and p95. The 6000 single-query
  differential is unchanged (v1 998k rmin 12.6u), confirming the change
  touched the cheap prefix and nothing else. So ScoutCheap helps
  everywhere and hurts nowhere - the advisor's "helps only modestly"
  branch on AirSuite.
  **THE DECISION IS STILL NO, ON NEW EVIDENCE.** AirRec is the
  constructive known-reachable yardstick. At EQUAL compute (3000
  ev/case, m pinned to 2, frozen fixture `de1b000e431a84fb`): legacy
  recovers **29/32** (<=8u 28, rp p50 1.0u p95 8.8u); v1 ScoutCheap
  **18/32** (17, 4.7u, 33.8u); v1 full-coverage 18/32. Zero bound
  falsifications either way. **AirRec L1 is BOUNDARY mode where
  `n_dom = 1`** - one domain, so there is no coverage prefix worth
  speaking of, no domain spread and no fairness discipline to blame, and
  indeed the two coverage arms are IDENTICAL at 18/32. This is a second,
  independent gap: the scheduler's method ladder does not expose, on a
  single domain, what legacy's fixed interleaved sequence does. Against
  the standing integration criterion - reaches full generic capability
  under escalation - that fails, on the suite built to answer exactly
  that question. Legacy NOT retired, `scheduler` default stays 0,
  EntranceField NOT integration-ready.
  Recorded alongside: **the scheduler path never consults
  `RefTune::shoot_m`** (`tune_m` is read only at the three legacy call
  sites), so AirRec's m-curve cannot be computed for the scheduler arm -
  its 17/18/18 is the same machinery measured three times, not a
  representation curve. The 29-vs-18 comparison avoids that trap by
  pinning m and equalising the budget. PROCESS NOTE: two separate
  capability gaps, found by two different suites, neither visible to the
  single-query capability probe that reported 97% and would have waved
  both through. Generic suites are the gate; one query is an anecdote.
  Regressions unchanged at default config: airsuite 48/48 gap 242k,
  airrec fixture VERIFIED, wishparity PASS, strafelaw float-ULP exact,
  carve 3/3 M1.4 PASS, airsolve 3/3 M1.3 PASS. Full detail:
  `Docs/AirRecSpec.md` section 24.

- 2026-08-19 (session 12j, THE BOUND IS CERTIFIED - AND THE RETIREMENT
  GATE FAILED): B5 and B7 built; both green; `Entrance::UBoardHeading`
  promoted **ADVISORY -> CERTIFIED**; legacy **NOT** retired.
  **B7 found a real unsafety.** It pushes the ceiling's own maximiser
  (all three candidate classes) through `Board::PredictClipTickVel` -
  the helper production owns - and demands agreement on the bounded
  quantity. The mismatch was velocity phase: production stores
  `H = |end_state.vel|^2 + 2 g gs (z - zmin)` measured AFTER
  FinishGravity, while the relaxed set is written in PRE-clip
  velocities. With `G = gs g dt / 2`,
  `|v_end|^2 = s^2 - d^2 + (v_z - G)^2 + 2 G n_z d`; on an approaching
  arrival `d <= 0`, so for `n_z >= 0` the cross term drops and the bound
  keeps its exact closed form with `base = (v_z - G)^2 + pot`. Omitting
  `G` under-counted true energy by `2 G |v_z| - G^2` - the direction
  that makes an upper bound UNSAFE. Writing B5 exposed two more
  nominal-vs-credited mismatches: the ceiling was built at tick `N` only
  though strikes are credited anywhere in `[8, n_cap] + 8` (fixed via
  the EXACT ballistic z-window - no control touches `v_z`), and the
  potential used `q.Z` though any contact within `radius` is credited
  (44.8k of energy at radius 28). All three applied to the broad energy
  ceiling too. CAUSAL PROBE (gravity phase removed from the ceiling
  only): B7 192/192 fail worst 14,502; B2 worst excess 15,166; **B5
  still PASSES** (sharp margin 15k -> 9k) - the constructive oracles
  validated but could not falsify, exactly as the advisor's wording
  requires. B2 only became able to catch it once rerouted through the
  authoritative helper instead of re-deriving energy the ceiling's own
  way. Gate board **23/23**: B5 8 legal oracles, violations 0/0/0 across
  all three certified stages, tightest margin 256k/256k/**15k**; B7 192
  maximisers, candidates 28/127/37, bad theta/dot/energy 0/0/0, worst
  residual **0.31** on ~1.19e6. `d.U` now takes the interval ceiling
  when tighter and prunes name which bound eliminated them. S5 still
  reports **0 prunes and that is correct**: tightest `U_D` 1081k vs
  `L*` 998k-1030k, so nothing is provably irrelevant yet - certification
  made pruning possible, tightness decides whether it fires.
  **THE RETIREMENT GATE FAILED.** With the scheduler authoritative the
  single-query probe still looked fine; `airsuite` @600 evals/case went
  **48/48 -> 39/48** (<=16u 40 -> 31, p95 19.8u -> 47.8u), concentrated
  entirely in the hard class (hz55 15/24, s900 15/24; hz30 and s400
  both 24/24). Budget scaling separates capability from cost: 1800 ->
  48/48, 3600 -> 48/48 with **<=16u 48 and p95 13.7u, better than legacy
  on every axis**. So it is a COVERAGE COST, not a capability deficit:
  mandatory minimum coverage of 6 heading domains costs 6 x kCostShot 40
  = 240 of 600 evals (40%) before anything escalates. That cost is a
  correct consequence of the budget-oblivious law, but a 48/48 -> 39/48
  generic regression at the operational budget is precisely the
  "important generic regression" the acceptance criteria forbid. Legacy
  stays, default stays 0, **EntranceField is NOT integration-ready**.
  The single-query capability probe was never sufficient evidence - the
  suite was. One identified lead, not a licence to reopen scheduler
  quality: the coverage unit is charged at full shot price; a cheaper
  coverage unit shortens the mandatory prefix without touching the
  anytime law. Regressions after the change (legacy default, clean
  build): airprops 23/23, airsuite 48/48 gap 242k unchanged, airrec
  fixture `de1b000e431a84fb` VERIFIED (L1 32/32, L2 12/12), wishparity
  PASS 4/4, strafelaw float-ULP exact, humanexact f2 PASS (945k vs
  945k) / f0 -209k / f3 N/A, carve 3/3 M1.4 PASS, airsolve 3/3 M1.3
  PASS. Also corrected a stale record: the canonical airrec curve in
  AirRecSpec 13c predates the 12b shared-path fixes - measured now and
  deterministic on the same verified fixture it is iso 16/13/**18**,
  capability 16/**23**/29 (was 16/14/14 and 16/20/29); the do-not-build
  Level-B ruling is unchanged and slightly stronger. Full detail:
  `Docs/AirRecSpec.md` section 23.

- 2026-08-19 (sessions 12e-12i, THE SCHEDULER - built, gated, and
  measured; BACKFILLED into this log 2026-08-19 session 12j, which found
  they had been recorded only in `Docs/AirRecSpec.md` 18-22).
  **12e - scheduler v0** (`RefTune::scheduler = 1`): one deterministic
  action stream; `NextAction()` is a pure function of solver state and
  CANNOT see the requested or remaining budget; the Runner has exactly
  one budget interaction - ask what is next, look up its fixed known
  charge, execute if it fits, otherwise STOP, never skip. Domains
  `D = (Q region, T branch, I_theta)` with THREE terminal states
  (UNRESOLVED / PROVED_IRRELEVANT / PARTITIONED - a split is not a
  resolution). Importance = `U_D - L*` only; kappa/stall/residual choose
  HOW to refine, never WHETHER a domain matters. Gates S1-S5 green. S1
  immediately caught a budget leak reasoning had missed: legacy
  `gshots`/`gbudget` made scheduled actions no-ops.
  **12f - action set complete** (deepen + one GN iteration as
  first-class actions, S6). The capability gate was run BEFORE retiring
  legacy, per the advisor's ordering correction, and it FAILED: legacy
  1030k vs scheduled 965k at 6000 evals. Diagnosis: representation
  adequate, scheduler semantics correct, scheduler INFORMATION
  inadequate. S1 caught a second budget leak - `deepen()` consulted
  `budget_cur`.
  **12g - the interval board ceiling** `Entrance::UBoardHeading`, exact
  in closed form: with `a(theta) = h cos(theta - phi)`, `b = n_z v_z`,
  `d = a s + b`, E is convex in s because `|a| <= h <= 1`, so the
  maximum sits at `s = 0`, `s = s_max` or the approach boundary
  `s = -b/a`. `CosRange` is a separate property-tested function because
  a circular-extrema mistake is the one error that makes the bound
  UNSAFE rather than loose. Gates B1-B4/B6 green; wired ADVISORY.
  **The causal experiment: the bound landed and the gap did NOT close**
  (965k unchanged) - the missing-information hypothesis was NOT
  supported. Per standing rule this did not justify building the
  displacement ceiling; treating a scheduling defect with more bound
  mathematics is the error the rule exists to prevent.
  **12h - scheduler v1: allocation was the defect.** Same physics,
  bounds, methods and action set; only the SERVICE DISCIPLINE changed to
  deterministic weighted-fair virtual runtime plus a method lane ladder
  that yields on stall. 965k -> **998k** with a BETTER residual (12.6u
  vs 19.5u) and ~40% fewer actions (166 vs 257). Measured `U_D` spread
  171k-184k, answering the advisor's diagnostic: the bound DOES
  discriminate, so the fault had been policy, not information. New gates
  S7 (no domain monopoly) and S8 (within-domain method fairness).
  **12i - precision as an explicit rung state machine.** A domain
  acquires precision debt only once it OWNS a witness at the ACTIVE
  rung; the step then tightens exactly one rung and yields back to
  ordinary refinement: find feasible -> tighten -> refine -> find
  feasible. That closes both failure modes at once - no dormancy, no
  runaway tightening. **S9 PASS** both halves: 2 steps, exactly the
  kLadder rungs between the 28u acceptance radius and the 8u tolerance
  with no skips, and 0 steps on an unreachable target. **Quality
  unchanged at 998k** - the fix made precision CORRECT, not more
  productive, which rules out precision starvation as the cause of the
  remaining differential. Recorded honestly as a v1 simplification: the
  active tolerance is still solve-wide, not per-domain (the per-domain
  `rung` counter exists and is tracked; the tolerance is shared).
  Advisor ruling closing 12i: stop pursuing the 998k->1030k differential
  and do not implement per-domain tolerances - classify the shared
  tolerance as certification-mature work, not an integration blocker.

- 2026-08-19 (session 12d, SCHEDULER PREP - spec filed, one law
  sharpened, NO code shipped). Advisor elevated the prefix law to
  replay-truth status: THE REQUESTED BUDGET MAY TERMINATE THE SOLVE; IT
  MAY NEVER INFLUENCE WHAT THE SOLVER WOULD DO NEXT. That forbids
  budget-derived SKIPS, not just budget-derived content. Session 12c had
  already removed the content dependence (exact_top no longer flips at
  600 evals); the remaining violation is the guided phase's
  `gbudget = budget/3`, which does not stop the solve but SKIPS AHEAD to
  the refinement rounds, so run(B1) executes actions run(B2) reaches
  later - not a literal prefix. ATTEMPTED and REVERTED: removing the
  truncation alone. MEASURED CONSTRAINT (the reason it must not be
  removed without the scheduler): the guided sequence costs ~25
  eval-equivalents per shot, so 40 shots ~ 1000, and deleting the
  truncation starved every 600-eval query - airsuite fell 48/48 -> 38/48
  earlier the same day. The truncation is load-bearing UNTIL the
  scheduler interleaves coverage with refinement; the two must be
  removed together, never separately. A partial refactor also corrupted
  line endings (2548 doubled CRs) - reverted via git checkout, tree
  restored to the verified 48c0765 state and re-verified: airprops 7/7
  GREEN, airsuite 48/48 gap 242k. FILED: Docs/AirRecSpec.md 17 = the
  scheduler v0 build contract (the elevated invariant; the atomic action
  list Scout/Shoot/ExactRank/Deepen/GNIteration/PrecisionStep/
  SplitHeading with fixed known costs and a runner that STOPS rather
  than skips an unaffordable action; domain state D = (Q region, T
  branch, I_theta) with charts/m-levels/GN/precision as METHODS not
  branches; the seven-step v0 priority rule; importance = U_D - L* ONLY
  with kappa/residual/chart history selecting HOW to refine and what it
  costs; per-method exploration-debt floors F_{D,m} generalising the f0
  lesson; the four scheduler properties - trace-prefix on action content
  hashes, lower-bound monotonicity with the bank disabled, deterministic
  replay, and proof-carrying prune records; and the sequencing rule that
  nested ceilings come BEFORE scheduler tuning because today's domains
  share one loose global ceiling so U_D - L* carries little
  information). LEVEL B formally moved from pending escalation to
  INACTIVE CONTINGENCY. NEXT: build scheduler v0 to that contract.

- 2026-08-19 (session 12c, THE RULING'S "IMMEDIATELY" ITEMS + THE
  PROPERTY GATE. Advisor ruling received: DO NOT BUILD LEVEL B, BUILD
  THE SCHEDULER; full text + the two Stage-A finish lines in
  Docs/AirRecSpec.md 15, this session's work in 16.) DONE: (1)
  `strafelaw` now derives its printed verdict AND its exit code from ONE
  tolerance pair (was EXACT at 4.0 while exiting 2 at 0.05 - a gate that
  lied to automation); (2) all three dangerous doc defects corrected
  WITHOUT losing history - the status header (was "NOW -> M3 transfer
  refinement & assembly", now Stage A, old header kept as SUPERSEDED),
  the M0 entry's historical ~11-tick alternation law (now carries an
  explicit SUPERSEDED note naming dwell6 as a MINIMUM not a cadence),
  and the FALSIFIED "heading-command representation is structurally
  incomplete" diagnosis, marked superseded in all three places it was
  recorded (checklist session 9d, SolverAir.h, repfit's header) with the
  real cause named; the pre-review m-curve table is marked SUPERSEDED
  beside the canonical post-review one; (3) NEW GATE `airprops` - the
  seven adversarial-review defects converted into invariants checked
  against the real production path, with two pure helpers
  (Entrance::ScoutDedupe, ToGoArcLen/ToGoSweep) extracted so test and
  production cannot drift. IT IMMEDIATELY CAUGHT TWO LIVE VIOLATIONS:
  P3 - the tolerance NEVER TIGHTENED (the refinement round was unbounded
  so continuation was gated on budget remaining after it, and then my own
  guard tested the phase ceiling it was meant to lift = a self-lock;
  phase1 ran 1015 of 1200 with tol_final still 28u); P2 - MONOTONE
  COMPUTE VIOLATED, L(300)=1028k vs L(1200)=1018k, root-caused to
  exact_top flipping at a 600-eval threshold so the two budgets flew
  DIFFERENT SCHEDULES rather than one being a prefix of the other.
  PREFIX CONTAINMENT, PRECISELY: run(B1) must be a PREFIX of run(B2);
  truncating a fixed action sequence at a budget-derived point preserves
  that, but any budget-derived gate that changes an action's CONTENT
  does not. exact_top is now fixed; the coverage sequence keeps its
  truncation point (removing it too cost airsuite 48/48 -> 38/48 because
  a fixed 40-shot prefix costs ~1000 eval-equivalents and starved every
  600-eval query; restoring it recovered 48/48 at a best-ever 242k mean
  gap). SCHEDULER INPUT: with prefix containment the coverage prefix is
  budget-INDEPENDENT, so a budget FRACTION cannot bound it - the
  reserved precision share applies to the DISCRETIONARY REMAINDER after
  coverage (now enforced as P3), and coverage costs ~25 evals/shot, so
  budgets under ~1500 are almost all coverage. That is the measured case
  for cheap-scout-first escalation. FINAL BOARD (fixture hash
  de1b000e431a84fb VERIFIED): airprops 7/7 GREEN; wishparity PASS
  (H2, controller 4/4); strafelaw EXACT exit 0; airsolve 3/3 PASS; carve
  3/3 PASS; airsuite 48/48 struck, mean gap 242k (best ever), rmin p50
  13.4u p95 19.8u; CURVE[cap] L1 15/20/28 of 32 with the high-curvature
  class 4/15 -> 8/15 -> 14/15, CURVE[iso] L1 15/12/17 - the funded curve
  still scales and the flat-spend curve still does not, so the ruling
  stands. NEXT: the deterministic central refinement scheduler over
  domains D = (Q region, T branch, I_theta) per AirRecSpec 15.4, then
  nested ceilings, then CURVE[scheduled], then UNFREEZE ExitField (the
  repaired controller is a validated search primitive for it).

- 2026-08-19 (session 12, THE CONVENTION WAS A FULL PI ROTATION - and
  the capability curve SETTLES LEVEL B: DO NOT BUILD IT. Full detail in
  Docs/AirRecSpec.md 13b (the ruling) and 13c (the measured answers to
  all nine gates); this entry is the index.) METHOD: a 5-agent
  read-only audit fan-out first (TickLaw consumers repo-wide, plateau
  scoring paths, representational caps + envelope APIs, airrec harness
  anchors, standing-laws rubric) - it found four reinforcing plateau
  mechanisms, three live pre-existing bugs, a premise correction
  (airsuite's acceptance radius is 28u NOT 16u) and the reason the
  m-curve looked flat (at 600-1000 evals only ~7-13 of 37 enumerated
  guided shots ever ran, so THE M=2 PASS AND NODE-TIME ADAPTATION HAD
  NEVER EXECUTED at production budgets). NEW GATE `wishparity`: one
  exact engine tick vs the closed-form law over 168 (speed, stored
  cosa, side) rows, DISCRIMINATING two hypotheses instead of assuming
  one. VERDICT H2 by seven orders of magnitude (max |dspeed^2| err 0.5
  = float-ULP vs 2.38M; max |dheading| err 7.2e-07 rad vs pi; rotation
  follows -side in 98/98 turning rows): the stored basis realizes its
  wish at wh+pi so true cos = -stored AND realized rotation = -stored
  side. SESSION 11 FIXED ONLY THE COSINE - every modelled turn still
  pointed the wrong way. WORSE, THE CONTROLLER WAS INERT: the probe
  showed the heading error FROZEN (0.1500 -> 0.1500 over 25 ticks, 0/4
  converging) because a braking request mirrored to true cos ~ +0.9,
  far above cap/v, so addspeed <= 0 and the engine did nothing - the
  true root cause of the historical "heading channel incomplete
  (66-282u misses)" verdict. After the fix 4/4 targets converge to
  0.0000 rad within 5 ticks. HARDENING: Strafe::TrueWishCos is now a
  real type and TickLaw accepts ONLY it (the compiler refuses a naked
  stored value); ToTrueWishCos/ToTrueRotSide/ToStoredWishCos/
  ToStoredSide carry the measured bridge; all ~25 consumers audited and
  classified; Steer::Controller's duplicate emitter now converts
  explicitly; certified bounds (Field::BrakeTurnPeak - the one
  hard-cull - and the priced-brake scan) verified convention-CLEAN BY
  CONSTRUCTION (no stored value can reach them). VOCABULARY: the active
  band is true cos in [0, cap/v] = 0.033 wide at v=900, so a fixed grid
  over [-1,1] cannot resolve it - the old candidate AND seed grids were,
  in true units, a spread of braking turns plus one max-gain point with
  NO sustained-gain member; both are now speed-normalized fractions of
  the band, converted at the boundary. PLATEAU (advisor item 2): one
  monotone residual channel (strike < miss < clean-air rejection - the
  old negative band had three incommensurate quantities so a converging
  candidate's score DROPPED before it jumped), value tier only inside
  the ACTIVE tolerance, tolerance tightened down the rungs between
  rounds with elites re-keyed from stored raw outcomes (no re-flying),
  tolerance-scaled scout dedupe (32u was coarser than every rung below
  32u), residual-gated Gauss-Newton in BOTH modes (face mode gated on
  best.ok = off exactly when needed), per-rung witnesses, and a
  RESERVED budget share for the precision phase (measured: at 600 evals
  the value phase never converged so continuation never fired at all).
  Crediting stays pinned to `radius` so no external contract changed.
  THREE PRE-EXISTING BUGS FIXED: dry incremented on its FIRST round for
  any solve that never struck (-1e30f + 1e-3f == -1e30f in single
  precision) cutting the hardest cases off after 4 rounds; the `sc >=
  1e7f` crediting gate silently dropped any strike with negative H
  (airrec passes zmin = 0); the node winner keyed on best.H so
  unstruck cases pinned m=2 and both GN passes to (interval 0, lateral
  0). CHARTS (item 4): chord chart's lateral now scales with a broad
  OPTIMISTIC REACHABLE SLICE (Envelope::DMax forward from P0 and
  backward from Q) instead of +-0.5*chord, which was the only lateral
  generator and therefore a hard representational limit; new CURVATURE
  chart generates nodes from generic (lambda, phi) on the circular arc
  through P0 and Q, both signs, broad range; proposals INTERLEAVED
  round-robin so no chart is starved by truncation; m=2 and GN seeds
  build on the winning chart/node-time. CURVATURE-AWARE TO-GO (item 6):
  kappa = 2y/(x^2+y^2), phi = 2 atan2(y,x) in the local frame gives
  required turn, arc length and implied terminal tangent, compared
  against remaining ticks, free-turn AND braking-turn capacity -
  advisory, never a bound. FIXTURES (item 3): airrec-fixture-v2 +
  manifest hashing the whole oracle set BY BIT PATTERN plus params/law
  versions; prints VERIFIED/DRIFT; measured de1b000e431a84fb, verified
  across runs. TWO CURVES (item 7), FINAL (post-review, fixture hash
  de1b000e431a84fb VERIFIED): CURVE[iso] L1 16/14/14 of 32 vs
  CURVE[cap] funded 1x/2x/3x L1 16/20/29 of 32 (50/63/91%), L2
  11/11/12 of 12; the persistent high-curvature class (dh_tot above
  median) goes 4/15 -> 7/15 -> 14/15 on the capability curve; cap m2
  ladder 8u:28 4u:23 2u:15 1u:11 of 32, rp p50 2.0u p95 9.1u; 0 bound
  falsifications in any run. NOTE the iso curve now DECLINES (16 ->
  14 -> 14): at equal compute the extra machinery is a net loss, which
  makes the dilution finding sharper than the pre-review numbers
  (91% funded vs 44% at flat spend) and the scheduler case airtight. => THE ADVISOR'S OWN
  SCALING SHAPE: sequential shooting is sound, LEVEL B NOT NEEDED; what
  remains is ALLOCATION (m=2 cannibalizes its own refinement budget at
  fixed spend - exactly the iso-vs-cap separation), so the next build
  is the adaptive refinement scheduler (competitive unresolved bound gap
  as the VALUE signal, kappa as DIFFICULTY only, minimum exploration
  everywhere). OTHER MEASURED: airsuite 40/48 -> 48/48 STRUCK (first
  full coverage) and mean sandwich gap 366k -> 261k best-ever (note the
  mean is conditioned on struck cases, so it is not directly comparable
  across coverage changes); humanexact f2 519k -> 947k at 9u = THE FIRST
  EVER PASS ON AN ADMISSIBLE HUMAN ROW (+2k ABOVE the human's 945k;
  during the session an intermediate build also RESOLVED the <=4u row
  at 931k/0u, and after the review fixes the <=4u row is unresolved
  again while near16 passes - both facts reported, neither averaged),
  f3 276k -> 796k (new 805k floor banked earlier in the session); f0
  REGRESSED 890k -> 750k at 14u (floor 895k stands - reported as failed
  rediscovery, never as regression) and is the one short-horizon
  low-curvature stratum the new gain-biased seeds and curvature to-go
  serve least; repfit per-tick rows unchanged (dp 0.0/0.0/0.1u - the
  executor was never touched, only the models that predict it).
  DEFERRED WITH REASONS: the oracle-projection diagnostic (its question
  - do node coordinates explain the failing class - is already answered
  affirmatively by the capability curve and the dh_tot split; still
  worth building as a permanent instrument); airsuite's own sub-8u
  rungs (its 600-eval budget cannot reach the continuation phase - the
  same allocation finding); `strafelaw` prints EXACT at threshold 4.0
  but exits 2 at threshold 0.05 (cosmetic disagreement between verdict
  and exit code). ADVERSARIAL REVIEW (3 agents vs the standing-laws
  rubric) FOUND SEVEN REAL DEFECTS INCLUDING ONE HIGH-SEVERITY ONE
  INTRODUCED THIS SESSION - all fixed before acceptance, all recorded
  in Docs/AirRecSpec.md 13d: (1) the boundary value tier fired on the
  WEIGHTED SUM rather than the feasibility predicate, so the plateau
  was relocated into the heading channel (accepting 320x the
  admissible heading error) - the exact defect continuation exists to
  remove; (2) deepen() improved elites without refreshing their raw
  outcome, so a tolerance re-key could score a 3u witness as the 12u
  schedule it replaced; (3) the reserved share was a race tested after
  an unbounded round (now an explicit phase1_cap), and its
  accompanying claim that no value metric can regress was FALSE and is
  removed; (4) the tolerance-scaled scout dedupe LOOSENED the coarse
  case 32u -> 115u for Field::Build (clamped); (5) GN pass 2 still
  seeded from bmag[win_bi], which is no longer assigned, so it stayed
  pinned to the chord centreline; (6) the curvature to-go diverged
  behind the target (J up to 1.3e10, rewarding sidestepping over
  turning) - arc length now clamped between chord and half-circle;
  (7) shot_key/dry read strike_rmin, never written when no clean
  strike exists, so the hardest cases kept a pinned winner. Review
  also CONFIRMED clean: all 38 TickLaw sites correct, the type
  migration a numeric no-op (so Field::BrakeTurnPeak is bit-identical),
  crediting keyed to the original radius, no bank path changed, no new
  erasing prune, airrec quarantine intact, arc geometry verified
  numerically. Legacy gates re-certified after the controller repair:
  airsolve 3/3 PASS, carve 3/3 PASS. ExitField, carve integration and
  Phase B remain FROZEN.

- 2026-08-19 (session 11, AIRREC BUILT - the constructive
  recoverability suite; the stored-basis convention MEASURED and the
  stage-1 mirror bug caught+fixed; spec doc filed as
  Docs/AirRecSpec.md, the post-compaction design of record with the
  15 standing laws). THE INSTRUMENT (advisor's top priority): hidden
  LEGAL wish schedules create known-reachable problems; production
  solves them cold; oracles are quarantined fixtures (never banked,
  never seeded; production receives only (S0,Q,T[,theta])). TWO
  LAYERS: L1 free-air boundary (S0,Q,T,theta) via new RefTune
  {shoot_m, bnd_theta} boundary mode in RefSolve (fixed-T schedules,
  tick-T residual rp+250*rth, H = terminal horizontal speed = the
  s_A* quantity; FlyWishSchedule gained a free-flight branch t.face
  < 0, end_state now filled on every exit path) - tests the
  numerical shooting core; L2 clean face strikes built by the
  TRANSLATION TRICK (free flight is translation invariant: probe the
  hidden schedule once in open air, translate the start so the
  trajectory crosses the face, demand a clean single-plane strike
  from the true replay; unconstructible cases printed with reasons,
  never dropped) - tests the actual Air->Board operator. Strata:
  reversals {0,1,2,3} x profile {gain, smooth, aggressive, brake->
  gain} x dwell {near-min, long} x T {28,48,76} x s0 {350,700,1000}
  x vz {+180,-60,-380}; 32 L1 + 12 L2 oracles, 100% constructed,
  legality-audited. Metrics per advisor: cold recovery %, tolerance
  ladder 32/16/8/4/2/1u, rp/rth p50/p95, oracle-energy recovery %,
  ev/case, per-stratum splits, dh_tot median-split feature
  correlation, and BOUND FALSIFICATION (U < E_oracle -> loud fail;
  0 falsifications across all 264 solves today - empirical
  validation only, not certification). ALSO BUILT: per-interval
  heading coverage {L_i, hits, shots} + iv_ref in RefResult
  (UNRESOLVED reporting: one-shot far intervals are coverage, not
  value); exactness ladder in RefResult (strike_rmin + rung_E[6]) +
  airsuite ladder line; FULL-xi GAUSS-NEWTON pass 2 (advisor
  correction): xi = (x1,y1,x2,y2) of the m=2 configuration, 3x4
  FD Jacobian, damped LS via 4x4 elimination, complete boundary
  residual (x,y,theta) - the single-node 2-var pass stays as the
  cheap first pass. THE DISCOVERY CHAIN (the suite paying for
  itself on day one): (1) smoke run: constant stored cosa=0.95 bled
  350->32 u/s -> MEASURED: the executor realizes its wish at wh+pi,
  so realized cos(rho) = -stored_c; stored +1 = max brake; the
  active gain band is stored c in ~(-cap/s, +cap/s) (verified
  numerically: stored -0.02 at s350/T48 gained exactly the law-
  predicted +54 u/s); generator profiles rewritten in stored units.
  (2) FIRST SWEEP (pre-fix): recovery curve FLAT - L1 m0 8 = m1 8 =
  m2 8 of 32, machinery levels changing NOTHING; failures
  concentrated in 0-reversal sustained-gain ARCS (0/8; e.g. 627u
  miss on a 76-tick gain flight while recovered flights bled to
  30-58 u/s). Root-caused, not hand-waved: (a) Strafe::TickLaw is
  in TRUE wish-from-velocity units (header + SDK-literal addspeed)
  but GuidedShootSeq stage-1 passed STORED c straight in -> every
  candidate's speed effect modeled MIRRORED (brakes ranked inert,
  inert ranked brake; only c=0 right; stage-2 exact rollouts could
  only re-rank a mirror-chosen shortlist); (b) the candidate grid
  had NO gain-band members (band width ~2cap/s is speed-dependent);
  (c) the deepen knot floor 0.05 is WIDER than the whole gain band
  at speed - long arcs could not be aimed. FIXES (all search-side;
  law/executor untouched): negate at the bridge (law.NewSpeed2(-c),
  TurnRad(-c,.)), speed-derived gain-band candidates (+-0.5cap/s,
  -0.95cap/s), fourth speed-derived knot rung cap/2s. (3) POST-FIX
  SWEEP: L1 m0 7 -> m1 8 -> m2 9 of 32 (mild slope, still low);
  ladder-32u 15->21 at m2, 1u 0->2; L2 10/12 at all levels (two
  constructed strikes at 20-27u rmin now unrecovered - budget
  shifted); THE PERSISTENT KNOWN-REACHABLE FAILING CLASS = long
  sustained-gain arcs (r0 1/8; large total curvature, chord-frame
  nodes + 6-tick lookahead cannot enter them). REGRESSION BOARD:
  floors ALL STAND (f0 895k, f2 950k, f3 759k; bank correct);
  humanexact rediscovery f0 890k@14u (~same), f2 519k@16u - THE
  FIRST production cold near-contact at the f2 disc in this era
  (class floor 950k unmet, but the needle is finally TOUCHED), f3
  276k@16u (down this run; floor stands, reported as failed
  rediscovery); airsuite 40/48 struck (was 42 - two hz30 s900
  vz-100 strikes lost = the dilution class) but MEAN SANDWICH GAP
  366k -> 312k (best ever) with dramatically softer high-speed
  arrivals (f0 hz30 s900 vz-400: dot -863.9 -> -34.5, H 549k ->
  1144k; f3 1163k -> 1429k) - the gain-band vocabulary is what the
  value search was missing; new airsuite ladder exposes the
  16u->8u CLIFF: 28-30/48 within 16u, 0/48 within 8u. VERDICT FOR
  THE ADVISOR (Level-B trigger reads the curve): 7->8->9 is not the
  scaling shape, but two cheap generic conditioners remain untried
  before defect shooting - (a) arc-frame node placement (nodes
  sampled along constant-turn-rate arc families through entry->Q -
  generic geometry; chord-frame laterals cap at 0.5*chord and
  cannot express large-arc bulges), (b) curvature-aware remaining-
  residual reach (currently straight-line DMax). NEXT: advisor
  ruling on arc-frame conditioning vs Level B; then the adaptive
  budget scheduler (P ~ competitiveGap*(1+beta*kappa)/cost, kappa =
  difficulty NOT value, minimum scout allowance everywhere); then
  nested-ceiling gap attribution (U0>=U1>=...>=H*) against the 312k
  mean gap; airrec reruns are the regression gate for all of it.
  shared test `Carve::ExitDoomed` (tap: Envelope::CanReach with true
  2D polygon distance + xy-AABB containment proxy, z from face
  verts, hull-padded; zone: Field::ZoneReach) - benched as row
  Mw-WIRED before wiring: falsekill 0.0% on ALL rides (the looser
  hull-padded bounds fixed M0's 2.2%), deadcatch 91-99%, shrink
  88-94%, 9-17 us. IN-SIM: RideHeadingSpline terminates a candidate
  at any separation (re-tested after each graze) whose state fails
  the test - new outcome DOOMED in results + reports; one-shot
  region-distance miss + reach_short preserve the search gradient
  for dead branches; g_doom_cull=false lets exitbench keep
  collecting true fates. MEASURED EFFECT on the solve: face-3
  arrivals now carry 592-644k (805-835 u/s) vs 18-337k pre-cull;
  1->2 crease grazes halved (-113..-195k vs -570k); endings launch
  from 567-614k (deficit to the ~722k requirement now ~110-155k =
  70-95u of height). Remaining blockers, ledger-visible: the 1->2
  crease graze persists in surviving lines (partially-grazed but
  still-reachable paths pass the cull - correct; they need the
  search to find the clean basin), and the human-style clean 2->3
  (level ride, ascending pop, near-parallel board) is still
  unfound. Gates green (airsolve/carve/boardproof) with the cull
  live. Partial export now the [0 1 2 3] chain at 1088 frames.

- 2026-08-17 (session 5, PROOF + BENCH - the board heatmap proven,
  exit-map models graded on engine truth, exit guidance UNWIRED):
  USER CORRECTIONS DRIVING THIS SESSION: the fpot "ratio 1.000"
  claim was a too-easy test (a nearly-flat score rates everything at
  the top - passing says only "not doomed"); exit maps must be
  graded against BEST RESULTS (engine truth), never against a
  reference run; entry heatmaps preserved; visuals were all-red
  (flat field normalized = wall of red).
  UNWIRED: all exit-map steering removed from the assembler - tap
  legs aim by the ENTRY (board) field exactly as before; the exit
  map steers nothing until it passes exitbench.
  BOARDPROOF (new gate) - the entry heatmap proven on its original
  three claims vs the exact engine: PASS. Bounds perfect: 22/22
  probed cells engine-reached, 0 violations of the gain-law speed
  bound / tangency-law minimum loss / energy bound; 0/15
  unreachable claims falsified (the turn cap's 0.5 brake factor
  held). Effective: 100% hit-rate across hot/warm/cold tiers.
  Inexpensive: 0.05-0.20 ms per full map (0.3-0.6 us/cell).
  Caveat recorded: the 0.21 energy correlation is measured with
  miss-minimizing probes that do not TRY to realize the bound.
  EXITBENCH (new bench) - candidate exit states + real fates
  collected from the assembler's own search (Carve::ExitRec
  collector; ~24k records per ride from the tape's board states),
  models graded blind: M0 feasibility-only, M1 +ballistic-arc world
  clearance, M2 +braking-law turn pricing, M3 both. RESULTS
  (basictest, human-tape entries): the search population is 93-97%
  DEAD (the user's "tons of grounded runs", quantified). M0 alone
  catches 99.2-99.9% of dead exits and 100% of scrape exits at
  0-2.2% false-kill of clean boards, shrinking 93-99.9% of exit
  space at 15-28 us/record - the state-space shrinker exists and
  is nearly free. M1 (arc clearance) added ZERO everywhere - drop
  it, measured no-op. M2 fixed energy prediction (bias +529k ->
  +93k, MAE 529k -> 222k) but ranking within the alive set stays
  weak (corr 0.21) - the exit map's proven value is CULLING, not
  fine ranking. Bin purity 97-100% at 64u/30deg and 128u/45deg -
  coarse cells are pure, so variable-resolution representation is
  valid on this data. NEXT (pending user): diagnose the 2.2%
  false-kills (which bound; push to 0 with law-derived slack),
  then wire M0 as IN-SIM TERMINATION at first separation -
  provably-doomed flights end at birth (removes grounded spam,
  buys wall time for deeper search).
  VISUALS: click-inspect now shows TOTAL ENERGY at the picked
  point (per-point speeds recorded: sims + reference tapes;
  E = v^2 + 2g*z with u/s and z shown); LEFT-drag pans, RIGHT-drag
  rotates (swapped per user).

- 2026-08-17 (session 4, THE EXIT MAP + THE ENERGY LEDGER - the
  user's exit-heatmap proposal built, validated, wired; the final-
  ramp energy question ANSWERED in numbers): NEW MACHINERY:
  `Field::ComputeExit`/`EvalExitSample` (exit = point on face x
  in-plane direction from a post-board state, valued by the BOARD
  MAP IT INDUCES on the next target; in-plane exits per the measured
  duck-off physics - mid-face departures must ascend, descending
  dirs depart at the boundary; ride bookkeeping = 2g + cap^2/tick
  wish + free-turn check), `Field::MapQuality` (functionals over an
  induced map), `Field::ZoneReach` (earliest platform arrival
  bound), `exitgate` (validation vs real tapes: bespoke human
  sample scored in the map's own currency - nearest-bucket matching
  distorts vz by hundreds of u/s and once flagged the human's real
  exit DOOMED; --bmap renders the B marginal-robustness overlay,
  report-only per the user's "careful with B").
  FUNCTIONAL VERDICT (the user's tuning question, settled by their
  own run): FPOT = potential-priced induced max (induced e_eff -
  cap^2*(flight n + ride ticks), the M2 telescoping currency) rates
  the human's exact exits ratio 1.000 on BOTH face rides; unpriced
  max 0.94-0.95 (its argmax is a ride-backward wish-credit
  artifact); hot-AREA functionals 0.09-0.22 (the speed run does NOT
  maximize margin area - area = margin overlay, matching runway
  2.8b). Doom cull: 32%/16%/60% of exits provably dead before
  departure. ZoneReach passes both real endings (human margin 61z,
  old line 9z). Bookkeeping optimism +65..+114 u/s = unpriced ride
  dissipation (admissible; also a measurement of ride quality).
  MapQuality argmax honors the RUNWAY LADDER (viable+free > viable
  > free > any) - without it the induced argmax picked energetic
  descending-bottom landings that strand the next leg.
  WIRED (guidance only, committing calls only - the map per
  depth-2 probe call halved shape depth in the wall budget): tap
  legs get exit_heading + the best exit's induced argmax as
  field_aim; zone legs get the earliest-arrival exit heading.
  Measured effect: leg-0 taps improved outright (0->1 dot -60.6 @
  946 u/s carrying 903k vs the prior -202 @ 953).
  THE LEG ENERGY LEDGER (user: report in energy; show where it
  goes): per-leg print E_in -> E_out (u^2/s^2 with u/s + z) split
  board / ride clips / GRAZES / tap / resid. Graze dot^2 was NEVER
  ACCOUNTED (flight-section clips continue silently) - now
  accumulated (Carve::Result.graze_loss2) and the ledger closes
  (resid ~ +0..70k wish everywhere). THE HEADLINE FINDING: the
  soft "-7.5" taps the prior session celebrated are CREASE-SCRAPES
  - the flight rides the valley crease between faces and is sanded
  down by grazes before arriving: measured 1->2 grazes -570k
  (903k -> 337k) and 2->3 grazes -363k (337k -> -33k relative);
  a variant line grazed -712k. Ride clips run -30..-81k per face.
  Boards/taps themselves are small (-3..-48k). SO: the final-ramp
  energy deficit = crease grazes (dominant, previously invisible,
  actively selected FOR by the 0.6|dot| softness pressure - the
  crease is a |dot| minimizer) + ride clip bleed. NEXT LEVER (probe
  first, search-score changes regressed twice): a graze_loss2
  regularizer in the SEARCH score (arrival-shape family, like
  0.6|dot|, NOT energy currency) and/or crease-avoiding aim
  construction; selection already prices the outcome.
  EXPORTS (user ask): partial chains now export on failed solves,
  clearly named beside the anchor tape - this session produced
  surf_basictest_PARTIAL_shape0-1-2_legs3_0817-0427.tas and
  surf_basictest_PARTIAL_shape0-1-2-3_legs4_0817-0433.tas (the
  full four-face chain, 1167 frames, playable in-game; it will not
  finish - review artifact). RunResult.legs_done + SolveMap
  partial out-param. route_features.csv untracked (accidental
  commit debris). Reports: exit_*.html (exit heat + human/best
  markers), exit_B_*.html (B overlay), msolve_*.html per run.
  Gates green after every change (airsolve 12/12, carve 10/10;
  exitgate human ratios re-verified 1.000 after the ladder).
  ENDING STATE: still open - the true blocker is now measured as
  upstream energy destruction, not ending-search capability; the
  last [0 2 3] variant rode f3 to an ascending crest exit but flew
  from 409 u/s (nothing left after -234k of grazes).

- 2026-08-16: Created. M0.1â€“0.3 done (design doc; strafe law proven
  float-ULP vs certified mirror; extractor v0 with 4 faces + 12
  candidate edges on basictest). NOW = M0.4 face coverage.
- 2026-08-16 (later): M5.3 advanced - demos received and staged;
  file-parse path measured dead (TV demos, zero cmdinfo); in-game
  queue-capture built into the DLL (one click, unattended, snapshot-
  gated rows). Tools/demextract.py kept for header/cmd walking.
  Awaiting one short game session for the traces; not blocking M0.4.
- 2026-08-16 (later): M0.4 GATE PASSED (facecover: 483/483 tape surf
  contacts covered, 0 missing) and M0.5 done (AnchorZones + derived
  end zone from tape finish). NOW = M1.1 envelopes; M0.6 awaits the
  strafe rate-limit number from the user.
- 2026-08-16 (later): M0.6 done (6/s). M0.7 crash hardening: marshal +
  menu-draw SEH + device-lost fixes. Marshal ROUND 2 after the first
  capture attempt: drain #1 (CreateMove) never fires in the MENU, so
  queued playdemos sat stale and fired on the user's next map load;
  drain #2 added in the message pump (main thread, all app states),
  abort/timeout clear the queue. Stale zero-origin python traces
  deleted. Capture still PENDING one click with the new DLL.
- 2026-08-16 (later): M5.3 CAPTURED (all 7 traces, rounds 3-5 fixes:
  DT_CSPlayer-rooted netvars, settle+load-timeout, BOM strip,
  runner-by-name, stopdemo; run segments via dense+moving selector).
  M1.1 GATE PASSED after two fix rounds, both diagnosed from gate
  evidence: (1) first run had 6 "speed violations" up to +57u â€” the
  measurement included the contact tick, where the board clip converts
  vz into horizontal speed; envelope now measured at the last airborne
  tick. (2) 476 falsification escapes at exactly 8.5002 â€” the air-duck
  origin shift; z checks now use the duck band, RELATIVE TO ENTRY DUCK
  STATE {âˆ’8.5, 0, +8.5} (a stretch entering ducked that unducks
  mid-air sits at âˆ’8.5). Final: 15/15 contained, 500/500 falsification
  clean. Battery still 14/15 (unduck_face 1.45u = the documented
  parity-era residual, untouched). NOW = M1.2 board windows.
- 2026-08-16 (later): M1.2 GATE PASSED first run (boardwin: 13/13 tape
  boards contained, clip model + zero-input tick decomposition both
  EXACT vs the engine mirror; 827 synthetic strikes, 0 violations).
  Board physics is now closed-form: lossÂ² = (v1Â·n)Â², min-loss law
  max(0, |vz|Â·nz âˆ’ sÂ·h)Â², aim cone per cap. KEY FINDING for the
  mission: the old solver's tapes contain a 38.5%-of-speedÂ² board â€”
  the "needlessly lost energy on boards" the user diagnosed, now a
  number the ledger can chase. NOW = M1.3 yaw-spline air primitive.
- 2026-08-16 (later): M1.3 GATE PASSED, 12/12 unseeded in 1.0s. Four
  iteration rounds, each from row evidence: (1) spline domain must be
  the expected flight, not the sim cap (dead-knot bug); (2) arrival
  tick needs a hard window (tick_tol) or the search grazes back at
  tick 1; (3) the controller NEEDED the braking turn (cosa < 0) â€” the
  perp-only strafe family cannot soften hard boards; adding it turned
  6 rows at once and IS the flick/eat-energy-in-the-turn mechanic;
  (4) plain coordinate descent converges at ~200 evals regardless of
  budget (identical output at 900 vs 3000) â€” basin hopping with
  deterministic jitter actually spends the budget and closed the last
  2 rows. Primitive beats the tape's board loss on 10/12 transfers.
  NOW = M1.4 carve primitive.
- 2026-08-16 (later): M1.4 GATE PASSED (10/10 carves + 160/160
  manifold). Six iteration rounds, each evidence-driven: (1) the carve
  energy identity needs the DERIVED half-gravity cross-term bound, not
  a tolerance; (2) slow rides demanded the EFFORT channel; (3) crest
  launches demanded the full exit-velocity-vector spec (heading of a
  near-vertical launch is noise); (4) the 1e6 no-exit wall froze two
  rows across three fix rounds â€” a stall NEAR the aim must outscore an
  exit FAR from it (comparable scores restored the gradient and one
  row snapped to dv 2.2); (5) budget was NOT the lever (20k evals =
  same failure); (6) `ridedump` on the last stubborn ride revealed the
  DUCK-OFF exit (duck press on the final tick separates the hull with
  climb velocity intact) â€” added as a searched genome dimension and
  the row closed at dv 16.4. The primitive kit now expresses: tangent
  boards, braking flicks, weaves, coasting, crest launches, duck-offs.
  NOW = M1.5 the ledger.
- 2026-08-16 (later): M1.5 GATE PASSED â€” M1 COMPLETE. Ledger lessons:
  closure tolerance must be law + derived cross bound + float
  allowance (first run failed by 66 on a flat 60); sabotage damage is
  NOT capped by the stolen window â€” misaligned downstream yaws brake
  actively (+40.9k observed) and the ledger correctly prices that;
  localization = priors bit-identical + theft realized in the planted
  phase. The 292 line's indictment: 329k clip dissipation + 78k air
  shortfall; two of three boards had tangency-0 available (63k of
  pure regret). ledger-trace expert audit built but trigger/teleport
  classification deferred to M5.1 (big-map events are contaminated).
  ALL FIVE M1 PRIMITIVES GATED. NOW = M2 route search: analytic edge
  bounds (2.1) from envelope+window closed forms, beam/DP over
  feature sequences (2.2), envelope pruning + start/end planning
  (2.3). Gate: top-10 routes on basictest include the human shape and
  solved12 shape, enumeration < 5s.
- 2026-08-16 (later): M2 GATE PASSED after three bound revisions, each
  caught by edge diagnostics: (1) unpriced ride traversal made
  adjacent-face cycles free; (2) per-face full-drop energy credits let
  corner bounces harvest fake energy every revisit â€” replaced by THE
  POTENTIAL LEDGER (E' = E + 900Â·ticks + 2gÂ·Î”z uniformly, telescoping
  anchors, cycle-proof); (3) vertex-anchored credits inverted on one
  edge (s_ub 10) â€” same ledger fix. Plus: the END edge needed the
  fly-then-RUN leg, and ranking needed base-shape dedup. (A Run 21
  "human line" claim from this entry is RETRACTED â€” see the next
  entry.) NOW = M3: chain SolveTransfer/SolveCarve along the shape
  pool, assemble full runs, export .tas; gate = unseeded finisher
  < 2 min whose ledger dominates the old line.
- 2026-08-16 (session end): M3 assembler ~80% â€” ten evidence-driven
  iterations, findings baked into code and the 3.1 status above. The
  transfer physics of basictest measured from the certified line
  (ridedump): valley-hop transfers separate MID-FACE at +55..+100
  above zmin, cross flat-or-ascending (in-plane heading clamped out
  of the downhill half), and board the next base at z â‰ˆ âˆ’10; Run 21's
  crest exit is a DUCK-OFF; the start jump must be chosen by its
  BOARD, not its launch speed. Two chained boards at âˆ’59.8/âˆ’10.1
  prove the primitives compose. Next concrete steps: (1) make the
  unified tap transfer's guidance walk the ride through the measured
  band before separation (the strike gradient alone lets rides dive
  and ground in the valley); (2) once a full shape chains, FlyToZone
  from the last face finishes the run; (3) resolve the ZONE QUESTION
  with the user (platform vs pit) before M4. Wall per full attempt
  ~25-30s â€” well under the 2-min gate budget.
- 2026-08-16 (corrections, user in the loop): (a) Run 21 RETRACTED as
  any kind of benchmark â€” it is a dead in-game tape that rides face 0
  into the floor; the REAL human run is `basictest.tas` (319 zone
  ticks; ledger in M4 notes). (b) Zones are marked BY TEXTURE (user:
  red = end zone; basictest: green CABLE/GREEN on start brush 6, red
  CABLE/RED on end brush 10). Added texture identity to the world
  loader (texinfoâ†’texdataâ†’string lumps), per-brush textures in
  `mapinfo`, and `tapeinfo` (per-tape map/frames/anchor provenance).
  (c) REFRAME, user's words: "we are not solving a map, we are
  building a solver to solve every map" â€” so zone identification
  becomes a solver capability (detect start/end from texture
  dominance in map data; trigger-entity zones join in M5), and all
  per-map numbers in this doc are validation instances, never inputs.
- 2026-08-17 (session 3d, THE FINISH CAMPAIGN - chains reach human+
  quality end to end; the ending is one leg away): instruments
  added: `stateprobe` (tap/zone solve from an EXPLICIT state - the
  fast loop for mid-chain entries) + full entry-vector TRANSFER
  prints. THE RIDEABILITY LAW (measured root cause): tangent taps
  were landing on the EXITING branch of the tangent cone - touch
  the downhill lip, bounce off; a board the next leg cannot ride is
  a graze, so tap acceptance now requires the landing to stay on
  the polygon through the 3-tick catch window (the codebase's own
  streak constant; rejected touches continue as grazes WITHOUT
  poisoning the struck record - a first version did and produced
  phantom zero-state transfers, fixed + tick>0 hardening).
  SPLIT SCORES (measured resolution of the softness-vs-carry war):
  the SEARCH keeps 0.6|dot|-0.01*carry (it finds diverse rideable
  strikes; two energy-currency attempts regressed the search
  landscape both times), while the assembler's SELECTION among
  alts/depth-2 prices by ENERGY in potential-ledger units
  (-carry^2/1000 + 0.9*tick) - soft-but-slow no longer beats
  fast-and-clean where the route needs the speed.
  RESULT CHAIN ([0 1 2 3], 24k evals budget 170s): 0->1 -202 @ 953,
  1->2 -7.5 @ 920, 2->3 -7.5 @ 778 landing (628,-382) north-mid
  ascending - every transfer at-or-beyond the human's (-115/-206/
  -142), speed preserved, rideable. THE ENDING (face 3 -> zone)
  remains the one unfinished leg: probe-characterized as feasible
  from ~850 entries (the human's 854 crests ~500; our 778 crests
  ~310; the gap IS the entry speed), so the finish route is either
  +70 u/s through the chain or a better-banking ending ride.
  OPEN CRASH (repro catalogued, debug next session):
  `stateprobe <map> 628 -382 -60 73 771 85 3 -1 36000` dies
  0xC0000005 before any output; neighboring states (y +382, y
  -791) run clean; the same entry inside msolvegate does NOT crash.
  Zone-leg evals note: probe identical at 24k vs 72k pre-crash
  states - the ending search is landscape-limited, not budget-
  limited, from sub-850 entries. Gates green all session.
- 2026-08-17 (session 3c, RUNWAY IN CODE + field-guided aims +
  family win-rate data): Field::Sample now carries run_avail /
  run_req / run_viable with a NextCtx (next face centroid or zone
  volume). Laws: available = in-plane ray from the landing toward
  the objective to the face boundary; required = turn arc at the
  free rate + climb priced at the face's max climb rate (hn),
  measured FROM THE DEPARTURE EDGE (the ride travels the runway
  first - measuring from the landing overstated 10x), with the
  ascending-exit allowance (vz up to pv*hn buys hn*L altitude).
  Selection tiers: viable+free-turn > viable > free-turn > any;
  heat overlay dims runway-short samples. VALIDATION state: face-2
  event near-exact (human spot 444/471, field best 25u away and
  viable 444/435 - their real ride was ~450u); face-0/face-3
  required still overstates (flat-exit + straight-ray crudeness) -
  runway stays GUIDANCE + rendering, never a hard cull; iterate
  with expert eyes on the dimmed maps. WIRED: tap and air targets
  aim at the field's runway-aware best sample (fallback to blends);
  guidance only, families still compete. FAMILY WIN-RATE
  instrumentation (Carve::Result.family + per-solve histogram):
  first data - f13 az_obj-straight 14 wins (top), f15 az_tan 7,
  f10 S-late 8, f1 linear 7; ZERO wins in solve context: f0 hold,
  f6 slow-via, f7 climb, f8 duck-off, f11 runway-via-exit, f16
  az_tan-via. PRUNING DEFERRED: gate rows exercise different
  contexts (duck-off wins the solved12 crest row) - collect gate-
  context histograms before cutting. CHAIN STATE: 2->3 improved
  again to -296.5 @ 759 ([0 2 3]); 1->2 at -187.9 @ 931; endings
  still open. Gates green.
- 2026-08-17 (session 3b, RUNWAY - new testimony, recorded in
  SolverRebuild.md 2.8b near-verbatim + priority hierarchy #4):
  runway = usable ramp space after the board point for the action
  the ride must perform. It is the SELECTOR within the field's
  energy-equivalent hot areas (the field says where energy
  survives; runway decides where to land among those). Explains
  two measured facts retroactively: the human ending's ~60-tick
  ride (runway as free-energy bank, ~54k wish work) and their
  face-3 board at the hot area's far edge (space to convert into
  the climb). Formalization hook written into the doc: per field
  sample, runway_available (in-plane extent toward the departure
  region) vs runway_required (law-derived from the action: climb
  distance / turn arc / energy-bank ticks), ratio = the margin
  dial, priced at the potential-ledger exchange rate. BUILD NEXT
  with the field-guided aim wiring. ALSO recorded (user): old
  solver runs (pre-restructure) are negative comparisons only.
- 2026-08-17 (session 3a, THE HOTSPOT FIELD - user's framework
  observation, built + validated): `SolverField.h` computes, from
  any state and target face, the EFFECTIVE ENERGY of every landing
  point under best approach + optimal board - closed forms only
  (M1.1 ballistic roots both branches, DMax reachability, gain-law
  arrival speed, M1.2 tangency residual + tangent heading, turn
  need vs the certified free-turn budget + brake ceiling). LESSON
  BAKED IN: the currency must be TOTAL mechanical energy (kinetic +
  2g*(z - zmin)); kinetic-only wrongly sent every best point to the
  face bottom (fall speed looked free; a high tangent board keeps
  its potential for the ride to convert losslessly). VALIDATION
  (`fieldgate`, tapes as validation only): the human's three boards
  rate 1.00 / 1.00 / 0.99 - face 0's argmax lands within ~20u of
  the human's actual strike. DISCOVERY from the negative control:
  solved12's slams also sit at HOT positions - their failures were
  EXECUTION (arrival attitude), not placement; on this map position
  is broadly forgiving and attitude is the binding constraint. The
  field renders as a HEAT OVERLAY in the reports (AddHeat + energy
  heat toggle; fieldgate writes field_*.html with strike markers).
  TERMINAL TANGENT CONTROLLER v1 (testimony 2.1 as construction):
  built latched (naive engagement chatters - turning lowers closing
  which grows predicted impact time); A/B on the 0->2 probe shows
  the greedy override is NET NEGATIVE (-381 vs -162 free search:
  rate-limited tracking lags the MOVING tangent target while the
  free search plans the whole final arc). Flag `terminal_tangent`
  stays in both primitives, enables OFF everywhere; the next form
  should PLAN the final arc backward from the arrival, not track
  greedily. FAMILY-DILUTION LAW (measured): probe quality follows
  PER-FAMILY budget - the -162 0->2 recovers exactly at 27k evals
  with 18 families (= the old 500/family that found it at 12 fams).
  Family count is not free; NEXT: prune families by win-rate data,
  then re-express budgets. Gates green throughout (airsolve 12/12
  with terminal off by default, carve 10/10).
- 2026-08-16 (session 2h, TRIED AND REVERTED at user direction):
  interpreted the report observation "arrivals below the surfable
  plane dying on the bottom vertical border" as a geometric dead
  band and added a z floor (zmin + slack) to both region gradients.
  WORSE results (2->3 fell to -590 @ 409 from -360 @ 728) and the
  user judged the framing inaccurate. Reverted fully (git revert
  f24b19f). The observation itself STANDS UNEXPLAINED - low
  arrivals dying on the bottom border is a symptom to understand
  from data (probe-from-state instrument), not a constraint to
  impose on the gradient. Do not re-add altitude floors.
- 2026-08-16 (session 2g, user re-reference to testimony 2.1): the
  arrival-parallelism idea IS the original boarding law - "velocity
  tangent to the ramp face normal IN THE APPROACH DIRECTION ...
  closer to parallel with the ramp" - and the AIR primitive already
  implements it (its init families come from the M1.2 tangent-arrival
  headings). The tap transfer never got the same construction; its
  family end-headings were all objective-derived. ADDED: az_tan =
  the tap face's face-parallel direction (psi +/- 90, the M1.2 dot
  line at flat arrival) signed to CONTINUE the approach motion, as
  two families (straight-in, via-objective-side) - the same
  geometric init the air solver uses, no seeding, no prescription.
  Also: START-PLAN CACHE keyed by (first, second) face - the
  repeated identical start searches were eating the wall budget
  before deep shapes ran; with the cache the pool completes and
  [0 1 2 3] reaches all four legs again. Current chain quality:
  0->1 -191..-200 @ 898-943, 1->2 -131.2 @ 919, 2->3 still the weak
  transfer (-606 direct / -713 deep in-chain vs human -142).
  Gates green (airsolve 12/12, carve 10/10).
- 2026-08-16 (session 2f, TRIED AND REVERTED - recorded per the
  standing rule): pure energy-currency tap scoring. Hypothesis (from
  the user's "preserve more energy into the board" + the observation
  that v_post^2 = v_pre^2 - dot^2 makes a separate |dot| charge a
  double-count): score taps by -carry + time at the gain law's rate,
  no dot term. RESULT: measurably worse both ways. 2D carry has a
  plunge-conversion exploit (a hard board converts vz INTO horizontal
  - 2D speed rises while total energy burns); 3D carry still produced
  harder strikes and shorter chains (-433/-779 vs -187/-115).
  CONCLUSION: the 0.6|dot| term is not redundant - it is the arrival-
  shape regularizer that creates good search basins (tangent-arrival
  pressure); carry alone tolerates rushed strikes wherever speed
  survives. Reverted to 0.6|dot| - 0.01*(2D carry). The reverted
  build reproduces the good chains and set a new best 2->3:
  -360.8 @ 728 ([0 2 3]), alongside 0->1->2 at -200/-115.2 @ 918.
  How the human 2->3 boards (the target, measured): SHORT LEVEL ride
  across face 2 (874->854, slight climb, wish-banking), ascending
  pop-off +202 vz, 32-tick flight, arrival nearly PARALLEL to face 3
  (post-board v (-88, 850, -116) - almost pure +y) at dot -142.
  Candidate next lever: an arrival-PARALLELISM preference (the
  in-plane component of arrival velocity vs the face's lateral axis)
  as a family/score experiment, and the ending's ride-prefix seeding.
- 2026-08-16 (session 2e, expert steer round 2): the user read the
  report and called the defect: candidates turn RIGHT into the last
  ramp instead of LEFT ("more space to launch off of = smoother
  landing, better conversion"). Root-caused in two steps: (1) the
  objective-vert tie on face 3 (its centroid y equals the zone rim's
  clamp point, so north/south top corners are EQUIDISTANT) broke
  south by vert order and dragged every aim across the approach
  momentum - a right-turn slam by construction; (2) forcing the
  momentum-ahead vert as THE aim overcorrected (1->2 taps overshot
  the strike plane and grounded on the brush top - the side choice
  is one-dimensional, so the objective offset must slide along the
  face's LATERAL/contour axis only). FINAL FORM: az_aim reverts to
  the neutral centroid; the momentum-ahead + lateral-slid objective
  point becomes az_obj, offered as FAMILY VARIANTS (linear-to-obj,
  via-obj-then-out) competing with all others - carry/tangency
  scoring picks the side per situation instead of a global bias.
  MEASURED after: leg0 0->2 committed -187.8 @ 916 (was -400);
  0->1->2 chain at -200 / -115.2 @ 918 landing at the BOTTOM of
  face 2 (the user's called-for bottom-to-bottom 1->2 transition,
  at speed, whole face left as runway). REMAINING: the ending leg
  only (zone solve still short of the bank-wish-work-then-crest
  line). REPORT: click-to-select now GRAYS all unrelated lines
  (ghost alpha) and keeps the selected line white + its committed
  chain's descendants and the references bright (contexts are chain
  prefixes, so descendants = prefix matches). Gates green.
- 2026-08-16 (session 2d, expert steer applied): TESTIMONY ADDITIONS
  (user): (a) the end platform has NO boarding requirement - the
  finish = make it ON TOP of the brush -> ZoneVolume is now hull-
  expanded xy with z from just under the TOP (side entries below the
  top no longer count; shared by InZone/carve zone mode/tapprobe);
  (b) [0 1 2] can probably zone with enough energy preserved - keep
  it live, perfect the 1->2 bottom-to-bottom transition; (c) boarding
  = choosing how the SPACE LEFT ON THE RAMP serves the next
  objective (face 3 is rotated relative to the others). Encoded:
  carry-speed scoring (tap score now credits sqrt(v^2-2g*shortfall),
  the speed actually kept after the next-leg obligation - the old
  additive climb cost let dead slams look cheaper than fast landings
  that owe a climb); objective-serving pursuit aims (tap families
  aim at centroid blended toward the vert nearest the LEG-AFTER
  target; zones aim at their NEAREST RIM, never the sprawling AABB
  center); RUNWAY families (ride the face's length banking wish
  work, then turn out) + runway-spanning zone spline domains.
  MEASURED: 2->3 improved -728 -> -221/-267 @ 714-734 striking
  higher (z +7 vs -63); 1->2 hit -64 @ 898 in an earlier beam run.
  ENDING PHYSICS PINNED DOWN (closed forms + human tape): the
  platform arrival needs crest state ~(s2d 316, vz +200); from our
  y~-720 entries the direct crest launch tops out ~296 total - the
  human's extra energy is EXACTLY the wish work banked over their
  60-tick northward runway ride (~54k = the gap). The machinery to
  find it now exists (runway families/domains) but the zone solve
  still converges short of it - next lever: zone-solve budget/
  domain sweep or seeding zone families from the depth-2 probe's
  best ride prefix. REPORT V2 (all user asks): pan fixed (right/
  shift-drag, correct camera basis), outcome tooltips, CLICK-TO-
  INSPECT with committed-chain lineage ("fed by: [0 2] from spawn |
  start jump | L0 board f0 -62 | ..."), and msolvegate now ALWAYS
  saves a timestamped report under Output/reports (iteration
  history). Gates green throughout.
- 2026-08-16 (session 2c): THE SEARCH-SPACE RECORDER (user directive:
  "I want to actually see the routes being searched... a density
  state space based on real lines"). New module SolverSearchLog.h/
  .cpp: both spline sims stream every evaluated candidate's REAL
  trajectory (every 3rd tick) into a per-stage sink â€” all best-so-far
  improvements kept + a uniform reservoir of the rest â€” and
  `WriteHtml` emits a SELF-CONTAINED interactive WebGL report: line
  density over the map wireframe + face polygons + the red zone box,
  stage/outcome/score-percentile filters, alpha dial, and a
  chronological scrub that REPLAYS the search eval-by-eval;
  reference tape + solved line as overlays. `--viz <path>` on
  msolvegate and tapprobe. Serve via .claude/launch.json
  (`search-reports`, port 8777) or open the file directly. Proven:
  the full msolvegate report = 1,874,001 evaluated candidates
  sampled to 44,452 lines across 66 stages, 12 MB, loads instantly.
  SOLVER PROGRESS same session (each change probe/report-verified):
  (12) interior-margin region gradient (EdgeDistOut +
  kHullCenterSlack â€” the raw polygon boundary is a MARGINAL strike;
  20-50u edge-skim misses were killing every chain: landings now
  stick); (13) PER-LEG BEAM â€” SolveCarve returns top-K diverse
  successes (mirroring the air alts), every leg composes (board alt
  x carve alt) candidates and prices the top 3 with a DEPTH-2 probe
  of the next leg from their landing; the beam found a 1->2 transfer
  at dot -64 @ 898 u/s (human: -206 there). REMAINING, sharply
  isolated by the beam prints: the 2->3 transfer (all candidates
  strike face 3 low-south at dead speed vs the human's ascending
  north-mid board at -8) and face-2 endings (likely genuinely
  infeasible â€” the human uses face 3 as the elevator). The msolve
  report shows both patterns as density; expert eyes requested.
  Gates green after every step (airsolve 12/12, carve 10/10).
- 2026-08-16 (session 2b): the transfer-capability campaign â€” eleven
  measured steps (list in 3.1) driven by the new `tapprobe` isolation
  instrument (the FUNCPROBE method at transfer granularity: replay a
  tape to a tick, solve from that exact state, dump the winner's
  trajectory). Probe verdicts: 0â†’2 unseeded âˆ’162.8 @ 887 (human âˆ’206
  @ 874); face-3â†’zone ending 101 ticks (human 123). Killed traps, in
  the order the trajectories exposed them: corner-point targets â†’ the
  corridor thread (behind-plane miss counted as progress) â†’ grazes
  ending flights â†’ descending separations (ballistic shortfall term)
  â†’ dead spline knots (dual domains, again) â†’ arrival shaping (6
  knots, 12k evals) â†’ the ending (zone mode: arrival tick IS the
  score) â†’ myopic boards (top-K diverse alts composed with their
  carve). `DetectEndZone` now feeds routegraph/routesgate/msolvegate
  from texture marks with reference-replay fallback. Remaining: chain
  consistency (landings that strand the NEXT leg â€” depth-2 rolling-
  window composition is the next move, all primitives proven).
  Gates re-run green after every step: airsolve 12/12, carve 10/10.

- 2026-08-17 (session 5c): MID-FLIGHT doom re-test (every 8th airborne tick - void lines die when the proof holds, not at the floor) + SNAKE families (swing-wide bulge to az_tan arrival, tap-only after a carve-gate fail re-measured the dilution law; divisor conditional). First run: snake f18 won 4 contexts; clean crease-free 1->2 exists (815k->783k, grazes 0); crease basin persists in other lines; no finisher yet. Gates green.

- 2026-08-17 (session 5d, THE MAP BECOMES THE OBJECTIVE - user's diagnosis): the no-strike flight gradient pulled to the NEAREST face point (a pre-field rule from the corridor-trap era) - its optimum WAS a cold-edge perpendicular slam; every field improvement since only touched inits, never the objective. FIX: (1) heat-density cell schedule (Carve::Target.cells: hot cells 3x, warm 1x, cold/non-viable never; each evaluation's gradient pulls to ITS OWN scheduled cell - search density = map density); (2) cold strikes (below the warm 0.6 ratio, the existing fieldgate threshold) score as failures in TapScore; (3) the board heatmap now renders into EVERY msolvegate report (once per tap face). Gates green.

- 2026-08-17 (session 5e, BLIND-ENTRY FIX - the -545's real cause): legs whose ride must CLIMB before the flight had a blind entry-state field (fmap.best<0 -> no cells/bar/aim -> silent old behavior). Fix: when blind, the map comes from the EXIT MAP's best departure (ride bookkeeping sees the climb) - the exit map's designed role, now wired. RESULT: the -545 slam is gone; [0 1 2 3] boards f3 at -162 @ 804 (591k kept, 0 grazes); chains are clean-board-only under the map-best bar. Ending still open from ~566k. Gates green.

- 2026-08-17 (session 6, THE PATH-PRICED HEATMAP - model worked through with the user, all answers recorded in SolverRebuild.md 2.10): reachability stays physics-true; the efficient frontier enters as VALUE - turn beyond the free budget is priced at the certified braking exchange, CHEAPEST COST PER RADIAN (two exclusion forms and the max-rate price were falsified by boardproof/fieldgate in sequence: engine reaches cells via early low-speed turns and braking; max-rate pricing read the humans f3 board COLD). Final state: boardproof PASS (0 falsifications), human boards HOT 3/3 (1.00/1.00/0.94) under the priced value. Convertibility stays EMERGENT (user call); corners emerge, never named; flights always have a specific intended board. NEXT MAJOR: the dedicated exit-to-cell optimal path solver (turn-hold-turn segments, gain-aware) - the state space the user named.

- 2026-08-17 (session 7, PATH-SOLVER FIDELITY - tried and measured, reverted to best): three trace models for the constructed flight, each pathgate-measured: (1) full-gain trace (COMMITTED, 2a420d9): f0 CONSTRUCTS at -110.0 vs human -115.1 in one eval; f2 no-plan at 96u; f3 trace-exec divergence (cap-rejected in the solver, harmless). (2) no-gain-on-holds (the controller's literal exact-landing semantics): LOST f0 - the real controller weaves/gains through the dense-knot spline. (3) full command-mirror with turn-weave-turn generator: WORSE (f0 miss 191) - command-level mirroring desyncs from the controller's flip state and drift compounds. Padding note: the no-padding spline stretch (n knots over n+8) accidentally compensates full-gain optimism on f0 - keep as committed. CONTROLLER SEMANTICS LEARNED (SolverSteer read): lands each commanded step at the gain that turn requires (full step = full gain, zero step = zero gain), blocked flips are null ticks (weave branch wishes at the no-accel point), coast on blocked+large-error. NEXT (the honest fix, from the data): CLOSED-LOOP construction - fly the plan on the real engine and Newton-correct psi from the measured miss (2-3 engine evals; the engine IS the trace; still ~500x cheaper than search). Then re-run pathgate for 3/3 and let constructed flights carry the solve.

- 2026-08-18 (session 10d, TWO-STAGE SELECTOR + FULL-DOMAIN
  FRONTIER + GN NODE POLISH): advisor's next directive executed:
  (1) within-side cosa PROFILES in the lookahead (linear a->b pairs
  - the dwell law fixes the SIDE for six ticks, never the wish
  angle; braking turns can now evolve the angle inside one side);
  (2) EXACT-ENGINE second-stage ranking - closed forms rank ~23
  candidates, top-4 get 6-tick exact rollouts, sim ticks charged as
  flight-equivalents (budget-gated >= 600); (3) FULL-CIRCLE heading
  coverage - 6 intervals span the whole domain, near ones get full
  treatment, far ones one shot each (estimates ORDER, never erase);
  (4) m=2 generic second node (depart->develop->close, symmetric
  laterals); (5) OUTCOME-SPACE GAUSS-NEWTON on the winning node -
  finite-difference Jacobian of the replay residual w.r.t. node
  position, damped 2x2 least-squares steps (state coordinates, not
  schedule coordinates). MEASURED (airsuite): mean sandwich gap
  401k -> 366k; steep-fall class 12/16 -> 16/16 PERFECT; but
  BUDGET DILUTION shifted failures to the ascending class (+150:
  16/16 -> 12/16) and cost f3 rediscovery this run (floor 759k
  stands, f0 new floor 895k, f2 950k stands) - the machinery
  breadth now exceeds fixed per-case budgets. NEXT (advisor's top
  priority, not yet built): THE CONSTRUCTIVE RECOVERABILITY SUITE
  (airrec) - synthetic hidden legal wish schedules create known-
  reachable (S0,Q,T,theta,E) problems; cold solve must recover;
  oracle simultaneously falsifies any U < E_oracle; oracle
  witnesses live in the TEST FIXTURE, never production bank/seeds;
  tolerance-ladder reporting (32/16/8/4/2/1u); then adaptive
  budget allocation driven by conditioning measurements, and the
  nested-ceiling gap attribution (U0>=U1>=...>=H*). Level B only
  on measured systematic failure after all that.
- 2026-08-18 (session 10c, TERMINAL-CLASS CONDITIONING + bank-as-
  witness-store + airsuite v1): advisor's 15-point directive
  executed in order. DIAGNOSIS SHARPENED: multiple shooting had
  conditioned POSITION, not the TERMINAL-STATE CLASS - the solve is
  (Q, T, theta) jointly. BUILT: (1) GuideTarget carries a terminal-
  heading INTERVAL (cost = squared distance outside it); shots run
  PER INTERVAL over a generic coarse partition (the numerical
  implementation of theta -> s_A*(theta)); (2) intermediate-node
  cost gains the REMAINING-RESIDUAL term (closed-form feasibility
  of the final (Q,theta) from the predicted node state: free-turn
  shortfall + reach shortfall) - a node reached beautifully that
  leaves an unsolvable final problem now scores badly; (3) node-
  TIME adaptation (retry the winning node at 0.30/0.38/0.55/0.65
  of the flight); (4) BANK = WITNESS STORE: entries now carry the
  FULL raw start state (pos/vel/duck/side/age), realized contact,
  and witness-content hash - region/exact floors are DERIVED
  queries; replay-and-reindex migration becomes possible for any
  future key-semantics change (the age-cap orphaning cannot recur
  silently); (5) humanexact headline relabeled L(near16u) - a 13u
  strike is NEAR, never exact; (6) AIRSUITE v1 - the synthetic
  certification matrix (faces x horizon {30,55} x speed {400,900}
  x vz {+150,-100,-400}, region targets, NO TAPE ANYWHERE):
  strike rates + sandwich gaps aggregated per dimension - the
  generality law's teeth. MEASURED: f3 recovered 366k -> 759k
  (matches its floor) under interval conditioning; f0 892k stable;
  f2 cold class STILL open (664k near vs floor 950k standing).
  REMAINING per advisor order: exact-engine short rollouts for
  final action ranking (deferred - cost knob), m=2, Level B only
  if airsuite shows systematic conditioning failure. Stage-A
  success = the GENERIC operator (airsuite can block Stage A even
  with green fixtures).

- 2026-08-18 (session 10b, GENERALITY CORRECTION + multi-node
  shooting v1 + the bank migration lesson): user + advisor scope
  correction RECORDED AS LAW - the human run and surf_basictest are
  REGRESSION FIXTURES, never design targets; production derives
  nothing from tapes (nodes, headings, reversal ticks, sampling
  priors); THE DELETION INVARIANT: removing all reference tapes
  must leave production search decisions unchanged (current code
  complies - only test commands touch tapes); multiple shooting =
  generic adaptive conditioning (node count/positions/times are
  numerical resolution axes like spatial/heading/knot refinement);
  synthetic validation matrix (airsuite: start speed/vz x range x
  height x normal x horizon x reversals x target width) is the
  certification bed, human tapes just extra rows; entrance spec is
  H(Q|S0) = max over legal trajectories - never "beat the human";
  human full runs may seed the INCUMBENT (upper bound) in the map
  solver, never the route. BUILT: (1) GuidedShootSeq - sequential
  multi-node shooting (Level A): free horizontal nodes (neutral
  interpolation + symmetric lateral offsets both signs, geometry-
  derived), continuous simulation, SHORT-HORIZON law-model lookahead
  (6-tick hold rollout + straight-run remainder) replacing per-tick
  greed; m adaptive (m=0 shots then m=1 grid); (2) dwell-age cap at
  the gap (exact state reduction) in boundary measurement; (3) bank
  entries verified/migrated: THE MIGRATION LESSON - the age-cap
  changed state hashes and ORPHANED all floors (gate correctly
  showed "new floor 666k" where "950k stands" belonged); legacy-key
  migration built (replays under both hashes, merges, erases);
  floors RESTORED: f0 892k, f2 950k, f3 759k under canonical keys.
  PERMANENT RULE: key-semantics changes require migration; NEXT
  HARDENING: entries must store their full start state (not only
  its hash) so future migrations never depend on re-deriving
  states. MEASURED: multi-node shooting now REACHES the f2 disc
  (13u strikes, was NONE) but in the hard -550 basin - the 950k
  class remains cold-inaccessible; f0 892k @ 14u unchanged. NEXT:
  ladder measures basin radius in NODE coordinates (a,b) per the
  advisor; adaptive node times/count + Level B defect shooting if
  needed; build airsuite (the general certification matrix); f3
  759k->847k refind; then the humanexact/efield/fieldexact ladder.
- 2026-08-18 (session 10, THE FULL-MAP HANDOFF + the go, first
  guided-shooting run): Docs/FullMapSolverHandoff.md filed = the
  second design of record (transfer-operator composition -> Bellman/
  best-first B&B over exact boundary states -> certification; Â§51
  certify-without-resolving = the tractability mechanism; Â§19 role
  reversal: closed forms become bounds/proposals/impossibility
  proofs, never values). Advisor's go received with rulings: h(S) =
  max of admissible bounds ladder (direct-to-zone day one, relaxed
  reverse map solver as the middle layer - rehabilitates the
  potential ledger); exploratory dedup = DEFERRED CLUSTERS (record,
  reopen before certification), never dominance; bank = exact
  witness records, regions derive; f2 formal floor = 951k-class
  (banked diagnostic; two gates: known-floor + cold accessibility);
  ride basis discovered by exact replay (minimal Markov-complete
  control, duck included if physics demands); proof-carrying prunes
  (bound ID/version/domain/status); STATIC-MAP DOMAIN DECLARED
  (user: maps are time-invariant) so exact-state-earlier-time
  dominance is safe; Stage-A exit = 11 explicit criteria. BUILT
  THIS SESSION: (1) Steer::CtlState + Invariant-9 threading -
  boundary control state (side, dwell age) measured from tapes at
  separation, carried through EFEvent -> Build/RefSolve -> schedule
  admissibility validation (sched_legal at the seam); (2)
  Air::WishInputs factored - the wh+pi mapping lives in ONE place;
  (3) WITNESS BANK v2 - keys = state hash + map hash + params hash
  + em1/dwell6/clip1 versions + operator + event, src-tagged
  (production/diagnostic), diagnostic floors never seeds; (4) the
  GUIDED SHOOTER v1 - per-tick soft-cost feedback proposal
  generator (endpoint residual + terminal-heading blend + gain
  sacrifice; dwell law hard inline; braking turns reachable),
  frozen schedules enter ONLY via open-loop replay, charged to
  budget; (5) scout POOL (3, endpoint-diverse) with full deepening
  each round alongside the terminal-heading bins. FIRST RUN:
  ladder banks the 950k diagnostic floor; humanexact round-trips
  it ("bank: floor 950k stands" = the known-floor gate working
  exactly as ruled); f0 892k @ 14u (exact-point unresolved); f2
  cold production still NONE within 16u - guided single shooting
  with a straight-run endpoint predictor does NOT enter the needle
  (the greedy pull aims straight at Q; the human's line bulges
  wide) -> NEXT per the sanctioned progression: waypoint/MULTIPLE
  SHOOTING (0.3B), not another family; f3 climbs back to 759k @
  12u (847k regression target still to refind). Phase B + carve
  frozen; ExitField waits on Stage A.

- 2026-08-18 (session 9f, THE LADDER VERDICT - f2's basin is a
  NEEDLE): advisor directives built: (1) `ladder` - the controlled
  recovery diagnostic around the known-admissible f2 witness
  (diagnostic only, never a production seed); (2) terminal-heading
  FRONTIER in RefSolve (24 bins, per-bin elites, every flight feeds
  its landing bin; one solve now yields BestBoard AND FastestTangent;
  + a SCOUT lane for pre-strike guidance after the binless loop
  exited at 14 flights); (3) humanexact near(<=16u)/exact(<=4u)
  split; (4) THE WITNESS BANK (<tape>.bank - monotonic floors,
  replayed each run, MONOTONICITY BREAK detection; first floor
  banked); (5) Tier-0 split: reach predicate UNCERTIFIED vs energy
  ceiling re-certified independently on the contested cells (efield
  prints ceiling violations separately). THE LADDER'S DIAGNOSIS
  (f2, E_h 945k): K-ladder 1->17 interp knots MISS (59->19u,
  non-monotone: K=1 strikes 885k while K=17 misses - closeness in
  schedule space is NOT closeness in outcome space); K=per-tick
  REPRODUCES at 949k, 0u; optimizer FROM the exact schedule HOLDS
  AND IMPROVES to 951k (> human 945k - the moves are not
  destructive); perturbation recovery 0/8 at ALL radii incl. 0.05.
  VERDICT: not resolution, not local moves - BASIN ACCESSIBILITY:
  the f2 arrival class is a needle under open-loop cosa schedules
  (per-tick drift compounding), unreachable from seeds or any
  perturbed start. ALSO HONEST: the frontier restructure REGRESSED
  point-targeted solves (f0 898k->798k, f3 lost its 847k row within
  16u; the pre-frontier top-3-by-score deepening was effectively 3
  full-powered scouts; the new scout runs weaker fixed steps and
  the bins idle when few flights strike a 16u disc). The banked f0
  798k floor + the bank machinery would have CAUGHT this regression
  automatically going forward - which is the point. NEXT: give
  point-solves the full old deepening treatment on the scout pool
  (bins remain for field/tangent structure); then the conditioning
  question the needle-basin finding raises - search coordinates in
  which the f2 basin is wide (arrival-anchored profiles) WITHOUT
  reintroducing a heading controller into the definition.

- 2026-08-18 (session 9e, REFSOLVE ON THE WISH BASIS - first human-
  beating number): RefSolve rebuilt on the canonical control space
  per the advisor's two-layer architecture - Layer 0 = per-tick
  (side, cosa) via Air::FlyWishSchedule (no controller in the
  definition), Layer 1 = side-RUNS carrying cosa KNOT LADDERS
  (linear across the run; midpoint-insert refinement toward
  per-tick resolution - a dial, not a family), deterministic seed
  grid (single-run cosa ramps both sides, bulge two-runs, coast) +
  top-3 deepening (knot/length moves, ladder splits, reversal
  insertion) + LCG-seeded restarts until budget or 4 dry cycles.
  Witnesses now stored/replayed AS wish schedules (Rec.wside/wcosa;
  witnesscheck replays via FlyWishSchedule - 118/118 reproduce).
  LADDER RESULTS (basictest): humanexact f0 898k vs 921k (-23k, 7u
  off the literal contact, was NONE) FAIL-mandatory; f2 689k vs
  945k (-256k) FAIL-mandatory - the one deep case left; f3 847k @
  dot -127.5 vs the human's 834k = THE SOLVER EXCEEDS THE HUMAN
  WITH A LEGAL 6-TICK FLIGHT (+12k; their witness needed a 4-tick
  reversal - N/A formally, exactly the advisor's hoped-for
  outcome). efield: f0 77 cells to 908k, f2 9 to 760k (clean-air
  law rejecting 12 contact-assisted strikes), f3 32 cells to 820k
  (was 8/689k). fieldexact 11/17 saturated; f3 fresh references
  beat in-Build stored cells by up to +310k = the gap is now
  BUDGET ALLOCATION (in-Build 220 vs gate 700 flights), not
  representation. repfit hardened per advisor: full terminal-state
  deltas (f0: dp 0.0u, dv (0,0,0), d(n.v) -0.0 - engine-tolerance
  membership) + explicit basis_representable / 6tick_admissible
  flags (f3: YES/NO - the apparent contradiction resolved: the
  executor executes, the SEARCH owns admissibility). OPEN: f2's
  soft-fast arrival class (-256k, the real mystery); in-Build
  budget laddering to absorb gate-level discoveries; Tier-0 reach
  model (clean-air contradictions f2:3 f3:1, cull stays off);
  carve port FROZEN until entrance convergence per advisor.

- 2026-08-18 (session 9d, THE BASIS VERDICT - repfit settles it):
  advisor's representation-completeness unit test built (`repfit`)
  and it OVERTURNED the optimizer-blame hypothesis in three steps:
  (1) heading-CHANNEL test: flying the human's own realized per-tick
  headings through the tracking controller misses their contacts by
  66-282u (the M1.3-era per-tick mirror desync, reconfirmed) - the
  heading-command channel is the wrong basis; (2) NEW
  Air::FlyWishSchedule - the OPEN-LOOP wish basis: per tick,
  (side, cosa) about the current velocity heading = the direct
  admissible input space, no feedback to desync. Inversion subtlety
  caught by measurement: the controller's input mapping realizes its
  wish at wh + pi in WishFromInput coordinates - the tape inversion
  must go through the SAME mapping (one-line fix after grounded/313u
  misses); (3) THE EXACT ROWS THEN REPRODUCE ALL THREE HUMAN FLIGHTS
  PERFECTLY: 0u off at f0/f2/f3, dots -115.1/-203.9/-139.0 vs their
  -115.1/-206.4/-142.4, E within 1%. U_human IS in U_solver -
  representation completeness of the wish basis is PROVEN, including
  the above-free-rate turning (human peaks 2.4-5.3x free rate; rates
  beyond |1| are legal via the braking-turn branch - RefSolve's rate
  clamp widened to +/-3 accordingly). Segment compression at 2-4
  piecewise-constant segments drifts 13-125u = open-loop compounding
  of tiny cosa quantization, NOT a basis defect (finer variance
  thresholds do not split further; drift is positional, the search
  space is what matters). humanexact rows reclassified per the
  ruling: f0/f2 = FAIL (admissible witness not matched), f3 = N/A
  (4-tick reversal, outside the 6-tick law; law KEPT per advisor).
  NEXT (the one sanctioned build): migrate RefSolve onto the wish
  basis (side-run schedules + cosa profiles, seeds + moves + full-
  budget deterministic restarts), re-run
  humanexact/efield/fieldexact; then carve-search hardening under
  dwell 6.

- 2026-08-18 (session 9c, THE RULINGS LAND - dwell 6, clean-air law,
  humanexact, the general reference solver): user+advisor directives
  implemented (full text in SolverRebuild.md 2.11 RULINGS block).
  (1) DWELL LAW 6 TICKS: strafe_rate_max 6->12/sec; every hardcoded
  12 now derives from the law (SolverPath flip gap, Entrance flip
  prune, controller min_gap). Gate ripple: airsolve 3/3 PASS; carve
  gate RED 2/3 - the f3 slow-crest reproduction row (tape n=58 s2d
  315) no longer converges (solver finds n=37 s2d 725, dv 690; the
  carve search families were tuned under 12-tick pacing) - OPEN, fix
  = carve-search hardening, not a rule revert. (2) CLEAN-AIR LAW: a
  field witness must be collision-free before Q (struck_brush < 0);
  contact-assisted strikes counted + rejected (0 observed! - the f2
  "bound-unreachable" cells are CLEAN flights: the earlier graze
  attribution was WRONG; Tier-0's reach model itself is too tight
  there, likely the contact-altitude/tick-band - Tier-0 cull stays
  off). (3) FAMILY DISCIPLINE: 5-knot spline family DELETED;
  RefSolve = one general solve over segment schedules (len >= dwell,
  signed turn-rate fraction; split/append moves add reversals);
  powers the in-Build second layer (source-tagged), fieldexact, and
  humanexact. (4) HUMANEXACT (the exact-point lower-bound gate):
  0/3 covered - f0 no clean strike within 16u of the literal
  contact (E_h 921k); f2 659k vs 945k (gap -286k, our dot -557 vs
  their -206); f3 no strike (E_h 834k). HUMAN DWELL AUDIT: f0/f2
  flights are ADMISSIBLE (1 reversal, 21/25-tick dwell) - those
  misses are OUR incompleteness; f3 uses a 4-TICK reversal, OUTSIDE
  the 6-tick law -> user decision needed (the advisor's predicted
  case). (5) Case D downgraded to CANDIDATE everywhere. DIAGNOSIS
  (one cause across all gates): the segment SPACE is right, the
  OPTIMIZER is weak - greedy coordinate descent with few
  deterministic seeds converges prematurely (humanexact used
  137/1500 budget; fresh fieldexact references land -104k..-244k
  BELOW stored H at f2 = seed-sensitive, untrustworthy as a
  certifier). fieldexact 14/17 sat, worst gap 13k. witnesscheck
  100% (81/81). NEXT (priority): make RefSolve trustworthy - seed
  grid (multiple arrival headings x turn rates x lengths, both
  bulge signs), top-k deepening, deterministic restarts until the
  budget is USED; then re-run the gate ladder; then carve-search
  hardening under dwell 6.

- 2026-08-18 (session 9b, PHASE A BUILT - the witness-backed entrance
  field laboratory; ruling received and recorded: laws become
  EXECUTION-VERIFICATION, admission = exact witness-backed argmax
  incl. lossy winners; the two-hold family = REALIZATION STRATEGY
  ONLY, never the definition of H(Q) - certify only if it saturates
  a broader reference frontier). NEW: Source/Solver/SolverEntrance
  .h/.cpp - per (cell x vertical branch) records {H, theta*, s*, vz,
  v_pre, v_post, cp, dot, loss2, witness, FastestTangent slot} where
  EVERY stored H is an engine-REPLAYED post-board energy (|v_post|^2
  + 2g*(cp.z-zmin)), never a trace value. Realization layers, each
  measured in: (1) two-hold family sweep (psi1 x split x psi2, flip-
  gap law honored) + within-bin refinement; (2) psi2 ENGINE LINE-
  SEARCH per seed (trace is full-gain optimistic - flights landed
  short; 6->34 f0 cells on landing); (3) SPLINE SECOND FAMILY -
  bounded 5-knot searches at unverified high-promise bins + bulge
  inits (cracked corridor f2: 0->7 cells; f3 3->14; f0 human cell
  VERIFIED); (4) engine-side tangent walk with bisection at the
  strike frontier; (5) bound-proposed targets (ballistic closed
  forms propose untraced cells; QUOTA SPLIT 16 trace + 6 bound after
  merged sorting let the loss-free bound scale evict realistic
  targets - measured regression, fixed). Lab commands: `efield`
  (per-event field + human-coverage row + bound sandwich + tangent
  survey with the CASE-D DISCRIMINATOR + --witnesscheck +
  --dwellcost; heat + winning-heading-arrow report) and `fieldexact`
  (family-completeness gate: 5-knot reference search must fail to
  beat production H per sampled cell). MEASURED STATE (basictest,
  human tape events): f0 62 cells verified, human cell H 907k vs
  their 921k (dot -113.3 vs -115.1, rank 0.95, field max 912k) -
  14k residual gap; f0 is CASE D (closed-form min |dot| 91.1 - NO
  tangent exists from that separation; our softest -96.3 is SOFTER
  than the human's -115); f2 7 cells to 600k vs human 945k and f3
  14 cells to 728k vs human 834k - CORRIDOR FAMILY GAP stands (the
  pathgate verdict, now with exact numbers); f2 shows 4 engine-real
  cells the Tier-0 bound calls UNREACHABLE (graze-modified
  ballistics outside the bound's model - Tier-0 CANNOT hard-cull
  until this is resolved); witnesscheck 83/83 replays reproduce H
  (determinism); bound sandwich 0 over-bound everywhere. NEXT:
  corridor realization (edge-waypoint two-segment construction as a
  third family; tangent walk for spline strikes), f3 human-cell
  reach, then fieldexact saturation before any Phase-B integration.

- 2026-08-18 (session 9, THE OPTIMAL-BOARDING HANDOFF + efficiency
  visuals): the user delivered a formal heatmap redesign spec -
  copied to Docs/OptimalBoardingHandoff.md (design of record;
  summary + adoption notes in SolverRebuild.md 2.11). Core: per-point
  ENTRANCE values become witness-backed optima - s_A*(S0,Q,T,theta)
  boundary-value function replaces the separable bound x budget x
  price approximation; H(Q) = max exact post-board energy over
  arrival branches x terminal headings; FastestTangent(Q,T) stored
  separately with tangent_gap; tangent dominance = OPEN THEOREM
  (Case E: faster lossy arrivals CAN beat slower tangent ones -
  supersedes the absolutist reading; measure, don't assume).
  REPORT VISUALS (user directive, shipped this session): efficiency
  coloring - per line the energy ledger E = s^2 + 2g*z is walked
  over kept points; light blue = gained >=88% of the ideal wish rate
  (cap^2/tick x stride), green = no loss, orange->dark red = energy
  destroyed (darker = larger fraction); stage coloring stays as a
  radio toggle; legend in sidebar. CONTACT MARKS: Sink::Contact()
  force-keeps the exact strike point (line now ends AT the wall) +
  yellow 3-axis crosses at every board/tap contact (air kHit + carve
  tap kHit call sites); marks skip selection-dimmed lines. Also
  fixed: the doomed outcome filter checkbox (loop stopped at 6 - the
  8th entry never rendered). Data: stride + marks per traj row,
  cap2 in the blob. Verified live on a fresh msolvegate report
  (26,305 lines: 28 blue / 9,702 green / 16,575 loss-colored, 4,664
  contact marks, 0 lines without speed data, no JS errors). Honest
  instrument reading: only 28 lines sustain >=88% ideal gain - real
  flights weave at ~80% of cap^2/tick (the free-rate curve headroom
  the user deferred). Solve itself unchanged (leg-failure rule
  firing correctly; no finisher; partial exported).

- 2026-08-18 (session 8, ENGINE LINE-SEARCH + the corridor verdict): closed-loop model corrections replaced by an honest psi LINE-SEARCH on the real engine (9 flights max in pathgate, 7 in the solver integration; first on-cell/cap-passing strike wins). Measured: f0 constructs at -110 vs human -115 (1 eval); f2 and f3 RESIST STRUCTURALLY - their true approaches are CORRIDOR-HUGGERS (f2 skims the f0/f1 slope, closest point pinned 155u north/57u high at every psi - diving early grazes and the clip kills the southward push; f3 strikes 264u along-face from the cell). No geometry-blind trace family spans these; psi search converges to the corridor wall. Overnight zombie exe (PID 32932) held the build lock - killed. NEXT: EDGE-WAYPOINT CONSTRUCTION for corridor transfers - plan as two open-air segments via the face bottom-edge crossing (along-face leg to just past the edge, then the short dive to the cell); both segments are trace-friendly and the corridor constraint becomes the waypoint. Gates green throughout.

  **SUPERSEDED 2026-08-19 - THE (1) DIAGNOSIS ABOVE IS FALSIFIED.** The
  heading-command channel is NOT structurally incomplete. The 66-282u
  misses were caused by `Steer::Controller`'s EMITTER using the wrong
  wish convention: it computed a correct TRUE-basis cosa and emitted it
  through the stored-basis mapping without converting, so a requested
  braking turn became true cos ~ +0.9 - far above cap/v - and the tick
  produced NO ACCELERATION AT ALL. Measured by `wishparity`: 0/4 targets
  converged and the heading error stood still (0.1500 -> 0.1500 over 25
  ticks); after the conversion, 4/4 converge to 0.0000 rad within 5
  ticks and the `carve` gate returned to 3/3 PASS on its own. What
  survives unchanged: the canonical witness REMAINS the direct per-tick
  wish schedule replayed open-loop, because that is the cleanest
  representation of actual admissible inputs and replay is the only
  truth. What changes: the corrected controller is now a VALIDATED
  SEARCH PRIMITIVE (and a head start for ExitField), not a discredited
  channel.


      **SUPERSEDED 2026-08-18 - THE STANDING LAW IS DWELL 6.** The
      rate limit above (6 changes/sec, ~11 ticks) is HISTORICAL. The
      current control law is `strafe_rate_max = 12/sec` = a MINIMUM
      6-TICK DWELL between side reversals, and it is a lower bound on
      segment length, not a cadence (segments of 13, 8, 21, 6, 17 ...
      are all legal). Every gap constant derives from it; the dwell
      timer crosses operator seams (Invariant 9). Do not rebuild the
      12-tick pacing from this entry.
