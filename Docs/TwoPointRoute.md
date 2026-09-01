# Two-Point Route: the closed-form optimal-strafe solve

Opened 2026-08-29 (session 47), user directive. Goal: from any starting point
and velocity, compute the optimal pure-A/D strafe path to any known point
with a required ending movement direction, in constant or near-constant time
— no exhaustive search. The capability registry supplies the foundation; the
parity engine confirms every claim (binding semantics #9, verify-then-accept).

## Problem contract (Step 0)

- **Inputs**: start `(p0, v0)` (3D position, 3D velocity), target point `p1`,
  required arrival **movement direction** `theta1` (velocity heading, not the
  view), engine params (gravity `g`, tick `dt`, per-component cap `L`,
  per-tick add `a` — all the measured values, never assumptions).
- **Controls**: one strafe key held per tick (full A or full D, never W), view
  yaw free per tick. Since the wish is perpendicular to the view and the view
  is free, the per-tick control is equivalently the **wish angle `alpha`**
  relative to current velocity; the A/D side only fixes the sign convention
  and carries the dwell rule.
- **Objective**: maximize final energy = final horizontal speed at the pass
  tick, subject to passing through `p1` and arriving with heading `theta1`.
  View snaps cost speed through the SAME per-tick physics as every other
  tick, so a last-tick snap is suboptimal *by the objective itself* — no
  blending, no safeguards, exactly as specified.
- **The metric** (user correction, session 47): work in v² space. The
  per-tick gain budget is a CONSTANT 900 there, so available gain over T
  ticks is exactly 900T and `efficiency = (vT² − v0²) / 900T`. A speed-ratio
  metric has a ~92%-for-doing-nothing floor at high v0 and HID a coasting
  path as "96%" (it was 15.5% by energy). Bonus cancellation: the objective
  is LINEAR in the per-tick gains — Step 4's Hamiltonian gets simpler.
- **Feasibility is binary**: a candidate that misses the pass tolerance OR
  the arrival heading is a MISS, whatever its other numbers. Instruments
  must say "nothing feasible found" rather than decorate a near-miss.
- **Scope pin**: pure airborne flight between the two points (no mid-flight
  board; collision-freedom of the corridor is a separate check against the
  BSP machinery). Boards/ramp contact belong to the entrance-field track.

## RULING (user, session 47): attachments, not scores

The start (point + velocity) and the end (point + movement direction) are
ATTACHMENT POINTS - boundary conditions the equations satisfy identically.
There are no candidates, no tolerances, no near-misses, no scoring: the
solve drives the attachment residuals to float-zero or reports the input
infeasible. Scans/estimates may exist ONLY as falsifiers that check a
finished solve from the outside - never as the solver, never in the UI as
results. "Estimations, guesses, and close-enough thinking are poison."

## The route equations (exact, from the measured kernel)

Gaining regime (full A/D input; the budget clamp sits at c < 30 - 562.5*sf,
unreachable for efficient routes). Per tick, control = wish direction w_k
(unit; the free view makes it free, the A/D side only signs it for dwell):

    (1) cap attachment   v_{k+1} . w_k = 30            (exactly - the engine
                                                        sets the component)
    (2) update           v_{k+1} = v_k + (30 - c_k) w_k,   c_k = v_k . w_k
    (3) energy identity  E_{k+1} = E_k + 900 - c_k^2
                         => E_T = E_0 + 900T - SUM c_k^2 ; loss == SUM c_k^2
    (4) position         p_T - p_0 = dt * SUM_{k=1..T} v_k     (linear)
    (5) sensitivity      dv_{k+1}/dv_k = I - w_k w_k^T         (projector)

Problem: minimize SUM c_k^2 subject to (2), p_T = p1 identically, and
v_T parallel to theta1 identically. (Identical to maximizing E_T by (3);
a final-tick view snap is just a huge c_T^2 - punished by the same term as
every other tick, no special handling, exactly the user's requirement.)

Stationarity (multipliers: lambda in R^2 for position, nu for heading):

    adjoint     pi_k = (I - w_k w_k^T) pi_{k+1} + dt*lambda
    terminal    pi_T = 2 v_T + dt*lambda + nu * rot90(theta1_hat)
    control     (30 - c_k)(pi_{k+1} . w_perp) - (v_k . w_perp)(pi_{k+1} . w) = 0
                (per-tick closed-form tangency for the wish angle)

Three unknown scalars (lambda_x, lambda_y, nu); three residuals (2 position
components, v_T x theta1_hat = 0); Newton drives them to float-zero with
O(T) forward/backward sweeps per iteration. Root-finding on closed-form
equations is not a search: the equations define the answer, the iteration
extracts the root to machine precision.

The ZERO-LOSS FAMILY (c == 0 every tick): per-tick turn magnitude is then
FIXED (only its sign is free), so zero-loss paths form the exact skeleton
of the reachable set - the "constant-time formula for the optimal shape".
Exact position attachment generically needs c != 0 somewhere; the solve
finds the minimum-loss bending. Feasibility = existence of the root.

## The two structural cancellations

1. **Vertical decouples exactly.** A/D wishes are horizontal, so `vz` evolves
   by gravity alone. The arrival tick `T` (and the pass fraction inside it)
   is a tick-quantized quadratic solve — CS:S half-gravity order included —
   independent of everything horizontal. `T` is *fixed by ballistics*, not
   searched. (The solver already lives by this: "the pass tick is computed
   from gravity, never searched.")
2. **Position is linear in the velocity history.** Displacement
   `= dt * sum(v_k)`. A linear terminal constraint means its Pontryagin
   costate is a **constant 2-vector `lambda`**. Add one scalar multiplier
   `nu` for the arrival-heading constraint. The entire optimal control —
   every tick's wish angle — is then determined by just **three scalars**
   `(lambda_x, lambda_y, nu)` through a tick-local stationarity condition.
   The feedback the user named (angle changes speed changes optimal angle)
   is real, but it is *captured*, not fought: it lives in the forward state
   `(speed, heading)` recursion, while the multipliers stay constant.

Honest complexity statement: a true O(1) closed form for the whole path is
not on the table — the per-tick turn/gain coupling makes the state recursion
genuinely sequential. What IS on the table: **O(T) rollouts (tens of flops
per tick, T <= a few hundred) inside a 3-scalar Newton solve (~5-10
iterations)** — single-digit microseconds per full solve. That is the
"near-constant" target, and it is principled, not approximated: the rollout
uses the exact engine tick map, so model == engine by construction.

## Steps

**Step 1 — Ballistic timeframe (exact, closed form).**
`T` and the in-tick pass fraction from the vertical quadratic with the
engine's exact gravity ordering (both roots reported; caller picks). Gate:
zero-mismatch vs the parity sim across a fuzz grid of `(vz, dz)`. Mostly an
extraction/formalization of what the solver already does, landed as its own
registry row with its own falsifier.

**Step 2 — The exact per-tick map, analytic (the foundation).**
Closed form for one tick — with the MEASURED action convention (playground
finding #2, 2026-08-29): the input mapping realizes its wish at `wh + pi`
(SolverAir::WishInputs, repfit-measured), so the lab's `cosa` is against the
REVERSED velocity:

    cur  = -v * cosa           (cosa +1 = full-budget brake, 0 = max gain,
                                negative = forward/no-op at speed)
    add  = clamp(30 - cur, 0, budget),  budget = aa*wishspeed*dt*sf = 562.5*sf
    v'^2 = v^2 + add*(2*cur + add)

Verified against the corpus-checked mirror: worst |dv| 2.6e-4 u/s over a
3280-case sweep. NOTE the budget DOES bind - in the braking region (an
early draft claimed cap-regime-only; the playground falsified that). Max
gain is +900 u^2/tick at cosa = 0, independent of sf at full A/D input.
Remaining deliverables: the heading-turn half of the map, the inverse, and
derivatives - same falsifier pattern.

**Step 3 — Feasibility (two units, portfolio-composed).**
- **3a. Certified upper bound (cheap gate):** max horizontal distance
  coverable in `T` ticks from `s0` = `dt * sum` of the max-gain speed
  recursion, refined by a turn-cost debit from the Step-2 frontier for the
  required total heading change. One O(T) pass, never a false "infeasible"
  (soundness gate: bound >= every simmed rollout on the fuzz corpus).
- **3b. Exact feasibility = the Step-4 solve converging.** 3a prunes, 3b
  decides — the lego composition; 3a alone never rejects what 3b could reach.
- The user's instinct that "max gain probably gets close, but sacrificing
  gain for angle may win" is exactly what `lambda` arbitrates — optimally,
  per tick, with no heuristic.

**Step 4 — The core solve: 3-scalar shooting (the hard part).**
Discrete Pontryagin conditions for the objective + constraints; forward
state `(s_k, theta_k)`, backward costate for `(s, theta)` (the position
costate is the constant `lambda`); per-tick stationarity picks `alpha_k`
via the Step-2 inverse. Outer loop: Newton on `(lambda_x, lambda_y, nu)`
against the 3 residuals (2D pass-point miss, heading miss). Each iteration
is one O(T) forward-backward sweep of pure algebra.
Gates (verify-then-accept):
- *Exactness*: the extremal rollout replayed through the Prediction sim is
  tick-identical (it must be — same tick map).
- *Optimality falsifier*: for small T (<= ~40) run a dense exhaustive
  control search as ground truth; the shooting result must match the true
  optimum within float noise across the corpus. No acceptance without this.
- *Convergence ledger*: iteration counts + failure rate over fuzz; failures
  fall back to the existing solver search (monotone rule: the portfolio is
  never worse than today's solver).

**Step 5 — Dwell constraint + side schedule.**
Extremal `alpha` paths map to sides (sign of the wish offset); flips are
few (0-3 on real routes — this is structurally the existing SolverData
alternating-strafe schedule, so it plugs into the applied-schedule
machinery). Dwell (no A<->D flip within N=6 ticks): re-solve with flip
TIMES as explicit unknowns spaced >= N — a tiny constrained outer problem
around the same inner sweeps. Gates: dwell result <= unconstrained optimum
(sanity), >= the current solver's best on the same cases (monotone floor).

**Step 6 — Integration + instruments.**
Wire as the solver segment's primary engine (closed-form first, search
fallback), auto-verified through the real-pass measurement like every
applied schedule today. One-click falsifier battery; registry rows +
dashboard updated together; gen_capability_code rerun; timing target
recorded (goal: full solve < 50 microseconds — vs milliseconds+ of search).

**Parallelism**: 1 and 2 are independent (start together). 3a needs 2.
4's derivation can be written against 2's interface before 2's gates close.
5 strictly after 4. Registry IDs are assigned against the live checklist at
implementation start — no rows are named here that don't exist yet.

## Instruments (live since 2026-08-29)

- **The playground**: `..\reports\playground.html` (+ `playground_corpus.js`
  beside it; regenerate with `SolverLab pgcorpus` and re-wrap). Tabs:
  Scenario (arbitrary start/target/heading, ballistic T, feasibility bound,
  path candidates rendered with efficiency stats and dwell legality),
  Stress (Law A per-tick gain sweep, Law B ballistic closed form, reach
  sweep heatmap), Verify (the float32 JS mirror measured against the C++
  corpus - singles worst ~5e-6, 120-tick rollouts ~5e-4 u, sf law exact).
- **`SolverLab pgcorpus`**: 600 KernelTick singles + 16 authoritative
  120-tick MoveTick rollouts as JSON - the ground truth the page embeds.
- Serve the reports dir to view (file:// blocks the corpus include):
  `py -m http.server 8123 --directory C:\Users\Connor\Documents\SourceTAS\reports`

## THE MIXING LAW (user redefinition, session 47 - the working model)

The controls are ONLY the two locally speed-optimal strafes: optimal left
(c = 0, wish perpendicular-left) and optimal right. Everything follows:

1. **Speed decouples.** Any mix of the two optimal controls gains exactly
   900/tick: `s_k = sqrt(s0^2 + 900k)` - closed form, independent of the
   path. All feasible mixes tie at the ceiling `sqrt(s0^2 + 900T)`.
2. **One state remains.** Averaged dynamics: `dh/dk = m * delta_k` with mix
   `m = 2*lambda - 1 in [-1,1]` and authority `delta_k = atan(30/s_k)`.
3. **Position costate is constant** (position dynamics are trivial), so
   Pontryagin with the minimal-effort selection (min integral m^2 - the
   unique tie-break) gives the closed-form feedback law
       m(k) = sat( -delta(k)/2 * [ nu + cross(Lam, R(k)) ] ),
   R(k) = target - P(k) the remaining displacement. Three scalars
   (Lam_x, Lam_y, nu) against three attachments (position x2, arrival
   heading). Solved by a damped 3x3 Newton with homotopy on the position
   target (heading-only first - its endpoint is the free endpoint - then
   walk the target in). `sat` is implemented as tanh (same slope at 0) so
   root tracking survives deep bang arcs; bang arcs are EMERGENT.
4. **Realization is quantization**: 1-bit sigma-delta of m into A/D signs
   (every realized tick c = 0, full 900/tick preserved), then the exact
   per-tick Newton (solveRoute) closes the quantization gap to float-zero
   inside the law's basin, and the float32 mirror replay measures it.
5. **The loss branch (tight regime)** - when the law saturates, the optimum
   leaves the two-control hull: overturn `c < 0` buys turn rate
   `psi(c,s) = atan((30-c)sqrt(s^2-c^2)/(s^2+(30-c)c))` at quadratic cost
   c^2 and lowers speed (raising future authority). Same constant position
   costate; one energy costate w (terminal w_T = 0); per-tick condition
       2(1 - w) c* = |p| dpsi/dc,   sigma = -sign(p).
   UNIFIED SMOOTH LAW: `omega = -tanh(delta p / 2) * psi(c*, s)` - its
   small-p limit IS the mixing law (c* -> 0), so slack and tight are one
   law. Four scalars (Lam, nu, w0). Realization: dither the side on the
   tanh fraction; demanded-side ticks overturn at c*. The law rollout is a
   BASIN GENERATOR for the exact Newton, which owns final attachment (in
   the tight regime the KKT system is non-degenerate - exactly where it is
   strong; measured: 4-5 iterations from law basins).

**Measured coverage** (2026-08-29, T=71 -> (520,260) arrive 45deg unless
noted; all OK rows replay 0.000 u through the corpus-verified mirror):

| scenario | branch | vT / ceiling | result (all replay 0.000 u) |
|---|---|---|---|
| v600 ref | zero-loss | 651.0/651.1 (100.00%) | 20 ms |
| bend90 | zero-loss | 651.1/651.1 (100.00%) | 43 ms |
| short T=30 | zero-loss | 622.0/622.1 (99.99%) | 6 ms |
| headingOnly | zero-loss | 651.1/651.1 (100.00%) | 47 ms |
| slow300 near | zero-loss | 392.2/392.3 (100.0%) | ~70 ms |
| nearBound (straight shed) | zero-loss | 651.0/651.1 (99.99%) | 70 ms (fold seeds) |
| v700 | loss | 740.8/744.2 (99.5%) | 331 ms |
| v800 | loss | 818.5/839.0 (97.6%) | 317 ms |
| startHeading200 | loss | 646.5/651.1 (99.3%) | 354 ms |
| hook120 | loss, target continuation | 633.1/651.1 (97.2%) | 1.9 s |
| v1000 | loss, speed continuation | 957.9/1031.5 (92.9%) | 9.9 s |
| v1500 | loss, speed continuation | 1272.0/1521.2 (83.6%) | 25 s |

**2026-08-30 rev (user directives applied)**: (a) the per-tick control map
is now ONE function over the full range c in [-s, 30] - `tickFull`:
add = clamp(30-c, 0, B), dE = add(2c+add), psi = atan2(add rt, s^2+add c);
loss L(c) = 900 - dE is zero at c = 0 and nonzero exactly where the physics
charges. The per-tick optimum is the argmin of (1-w)L - |p| psi over that
one function (theta-grid + golden refine; the function is multi-modal:
shallow overturn, deep overturn, pure brake are its shapes, not solver
branches). Deep-brake controls carry into the exact Newton as locked rows
(dv'/dv = I on the clamp branch). (b) Winding classes: the arrival heading
attaches as h_T = hreq + 2 pi n with UNWRAPPED residuals, n in {-1,0,1}
solved as ordinary members - wrap-around routes included. (c) Fold seeds
(arch amplitude A = sqrt(2 shed/arc), P0 = 8A/(T delta^2), both signs) are
extra deterministic homotopy starts. (d) Continuation completes coverage:
exactWalk (position-target continuation of the exact Newton) and speedWalk
(start-speed continuation from an attachable base, locking budget-clamp
controls as they appear). (e) Playground: sliders (speed 0..3500,
headings -180..180, vz +-3500) with instant re-solve on input.

**Open work**: extreme-regime solve time (v1000+ at ~10-25 s - candidate
lever: analytic Jacobian for the exact Newton instead of FD); locked
clamp controls are held at law values, not re-optimized (bounded
suboptimality, measured in replay); independent falsifier for
no-attachment verdicts; dwell-6 realization (coarser sigma-delta).

**Findings log** (what the instrument caught, newest first):
11. "FIX ALL OF IT" ROUND (2026-08-31): three structural additions, honest
   status: NOT all fixed. (a) Constructive monotonicity - every pass
   stores its speed as the column anchor; higher cells brake to (near) it
   with an offset family. Sweep passes rose 1164 -> 1357. (b) TERMINAL
   WHIP: reversed arrivals attach via a locked brake(b)+spin(w) suffix
   (deep overturn turns 60-110 deg/tick; entry speed is closed-form
   consistent because a zero-loss sub arrives exactly at
   sqrt(v0^2+900(T-n))); members ranked by predicted arrival speed with a
   forward-achievability filter. arrive -135/-170/180 at 600 now attach
   with exact replay, dwell-legal (e.g. -135: vT 209, 6/10f). (c) Repair
   cascade with persisted suite state and a Repair-more button.
   MEASURED LIMIT: the post-sweep survivors are a hard class - repair
   yield ~6 percent at 5-8 s/cell (non-lite, whip gated to seeds), so
   budget does not close them. Final board: 1362 pass / 798 no-attach /
   389 violations / 12 isolated (from 862/1298/578 two rounds back).
   The evidence points at two levers, in order: the analytic Jacobian
   (evalF was 19.2M calls per sweep - FD is the tax that makes thorough
   polish unaffordable at scale) and richer law basins for mid-speed
   backward angles (mix reach 30-60 percent plateau). The turn-debit
   certificate (step 3a) stays pending to classify the truly-infeasible
   remainder of the deep-backward wedge.

10. THE PERIODIC FAILURE PATTERN DIAGNOSED AND CLOSED (2026-08-30, user
   observation: fail-notch walking toward 0 deg as speed rises, near-total
   miss bands at two 50-steps, repeating). Measured causes, in order of
   effect: (a) the exit-speed window after braking can be ~60 u/s wide
   while the lite ladder sampled 4 fixed ratios of sLo - the sliding
   window fell between grid points periodically in v0 (the walking notch),
   and n->n+1 brake-tick transitions moved sLo discontinuously (the miss
   bands). Fix: reach-guided refinement - the remainder law's reach
   profile points at the window; bisect toward its peak (formula-guided,
   no fixed grid dependence). (b) The LM acceptance rejected candidates
   that crossed a regime boundary transiently even while merit fell 45->6
   - but the forward map is the full truthful function, so those states
   are valid; now accepted on strong merit decrease (rows self-correct on
   re-entry). This also recovered hook120 as a distributed-overturn basin
   (96.4 percent of ceiling). (c) brake+arc composition (dump then hook)
   attaches the reversed-arrival family at speed (v2000 arrive -90:
   35.9 percent of ceiling, replay exact). Suite (dwell-6 enforced):
   1164 pass (was 862) / 996 no-attach (was 1298) / 462 violations (was
   578) / 4 isolated; 289 s (up from 151 - the acceptance change lets
   formerly instant-fail cells run: elim 33 percent, jtj 31, evalF 17,
   law.mix 15). Audit now two-tier: orange = fail above a pass in-column
   (the monotone law); yellow = isolated fail with >= 3 passing
   neighbors. VERDICT on the brake prefix (user hunch): the dump itself
   is formula-derived (energy identity + closed-form dE inversion); the
   hack was its fixed sampling grid, now replaced by reach-guided
   refinement. REMAINING red mass: deep-backward arrivals (-180..-135)
   where covering the chord and looping back are mutually exclusive at
   this T - likely GENUINELY infeasible but uncertified: the turn-cost
   debit on the distance bound (step 3a, still pending) is the missing
   certificate that would recolor them gray with proof.

9. PROFILER + DWELL LAYER + EFFICIENCY (2026-08-30, user directives).
   Stage profiler (exclusive buckets, percentages of wall) in the Route
   panel and suite summary. MEASURED single-solve battery: newton.elim
   40.1% / newton.jtj 33.1% / newton.evalF 20.8% / laws 3.2% - 94% of all
   time is the exact Newton's linear algebra, mostly FAILING iterations.
   Fixes from the data: LM inner attempts 10->4; stall bail (merit <0.1%
   over 25 iters); dwell patterns pre-ranked by conform-roll miss (best 2
   polished). Worst battery case 10.5 s -> 0.42 s (25x); all under 0.5 s.
   DWELL RULE now enforced end-to-end: free attach first, then pattern
   legalization (window rebuild / neighbor merge / periodic phases) with
   side barriers in the exact Newton (KKT: at an active side constraint
   the stationarity IS the constraint - locked ticks excluded from the
   tangency gate). Battery 6/8 dwell-legal at 97.9-99.7% of ceiling
   (hook120, v1500 still illegal - flagged). Correctness hole exposed and
   fixed: the cross-product heading residual also vanished ANTI-PARALLEL
   (a locked polish attached arriving backward); replaced by the wrapped
   angle difference with matching analytic terminal adjoint. Flip/dwell
   accounting: brake and no-op ticks (|cosa| > 0.85) are side-neutral.
   Suite rerun WITH dwell-6: 862 pass / 1298 no-attach / 396 bound / 578
   violations in 150.6 s (was 258.6 s at the easier no-dwell bar). Suite
   profile: law.mix 27.7% (ladder scans), elim 27.0%, jtj 24.9%, evalF
   14.7%. Next levers by data: coverage of the no-attach family (failing
   cells ARE the cost), analytic Jacobian, ladder-scan reuse.

8. COVERAGE SUITE ONLINE (2026-08-30, user directive): 71 speeds
   (0..3500 step 50) x 36 arrival headings (step 10 deg) against the
   scenario target, every cell = the lite pipeline, pass requires an
   exact-Newton attach; monotonicity audited per column (a fail above a
   pass is outlined - solver defect by the monotone law, not physics);
   click-through loads any cell into the Scenario tab for the full solve
   with float32 replay. Pipeline rebuilt search-free the same day:
   continuation walks deleted; brake prefix parameterized by target exit
   speed with the closed-form dE inversion (dump feasibility is an
   INTERVAL - too little turns short, too much fails the distance bound -
   bound-pruned arithmetically); the small-overturn root has the closed
   linear form c* = -|p|/(2(1-w)s); background-tab-safe scheduling.
   MEASURED first run: 865 pass / 1295 no-attach (bound-feasible) / 396
   bound-infeasible / 692 monotonicity violations, 258.6 s. Honest reads:
   (a) 4x over the <60 s budget - dominant per-cell cost is the exact
   Newton's FD Jacobian + J^T J build (analytic Jacobian is the named
   lever); (b) the no-attach mass is the deep-backward-arrival family
   (arrive -180..-90 relative to travel) at nearly all speeds - the law
   basins do not cover hooks past the target yet; the violations quantify
   exactly where the solver falls short of the monotone law. UI-click
   solves now cap at ~2 s worst case (was 25 s).
7. THE MIXING LAW IS ONLINE and the old finding-5 scenario closes at
   99.99-100.00% of the energy ceiling in ~60 ms (law 2 ms + polish),
   replay 0.000 u. The loss branch composition attaches v700/v800/
   startHeading200 at 97.6-99.6% of ceiling with the exact Newton
   converging in 4-5 iterations from law basins (it owns the tight regime,
   where KKT is non-degenerate). Structural lessons measured this session:
   hard clamps and bang signs kill FD Newton (tanh-smooth the selection,
   keep exactness in the polish); forward-shot adjoints and target
   homotopy are fragile beyond ~40% saturation - the composition
   (law basin -> exact Newton) is what actually closes.
6. FINDING 5 FALSIFIED (user caught it; measured same day). The coast
   solution was a WRONG-BASIN local stationary point, not the optimum. A
   zero-loss falsifier path (all c = 0: dip right ~10 ticks, arc left,
   hold 45 deg - sign structure only) reaches 24 u from the target with NO
   loss; warm-starting the same Newton from it attaches at float-zero in
   4 iterations, 0 coasts, vT 649.52 vs the coast answer's 623.91
   (replay miss 0.0001 u). Root cause is DEGENERACY, not a bug: the loss
   is sum c^2, and at the true optimum nearly all c = 0, where the cost
   gradient vanishes - so the KKT stationarity system reads 0 = 0 along
   the real decision dimension (the SIGN/turn schedule) and multipliers
   collapse toward zero. A stationarity solver cannot pick the basin; the
   optimum is characterized by FEASIBILITY (geometry), not stationarity,
   except when turn authority saturates. Corollaries, all measured:
   coast NEVER beats zero-loss wiggle while curvature authority remains
   (wiggle sheds chord length AND keeps the 900/tick); every v0 > 600
   trial failed (wrong basin + the in-regime guard forbids the
   budget-clamp braking branch, so speed-tanking routes are
   unrepresentable); turn/hold/turn with the hold glued to the arc end is
   too thin a family - arc PLACEMENT is the third scalar. The corrected
   architecture is the heading-space term solve (below); the T+3 Newton
   survives only as the final polish inside a falsifier-chosen basin.
   (Levenberg-Marquardt on z = [phi_1..phi_T absolute wish angles, lam, nu],
   FD Jacobian, scaled rows) PLUS an active-set outer loop: the interior
   solve walks leading ticks onto the c = 30 boundary when T is generous -
   those are REAL optimal structure (coast arcs: a no-accel tick sheds
   distance at zero energy cost, beating quadratic-loss wiggle). Pinned
   ticks become exact engine no-ops (cosa = -1 replays as add = 0
   identically); pins only grow; re-solve warm until no new pins.
   Reference scenario (v0 = 600 along +x, T = 71, target (520,260),
   arrive 45deg): residuals [1.4e-10 u, 7.3e-11 u], heading 3.1e-15,
   tangency 2.3e-12; 17 coast ticks (13 leading + 4 chatter at the
   transition); KKT-checked (freeing any pinned tick into the gaining
   region never improves the Lagrangian); float32 REPLAY through the
   corpus-verified mirror: pass miss 0.000 u, heading err 0.0000 deg,
   vT 623.91 = the double solve exactly. Bilevel (inner ascent + outer
   multiplier Newton) was structurally fragile and is dead. Failure modes
   are honest: infeasible-by-bound cases are refused BEFORE the solve (the
   B22 bound is a proof); a feasible hook arrival (arrive 120 deg off a
   26 deg displacement) still fails from the cold start - basin/init
   limitation, flagged in-UI, falsifier + basin study pending. Chatter
   coasts (alternating live/dead) are the discrete shadow of a singular
   arc; the dwell layer (step 5) will regularize them.
-1. DRIFT CORRECTION (user ruling): candidate scans with tolerance scoring
   had crept in as if they were the product. Removed from the instrument's
   results entirely; the attachment-points ruling and the route equations
   above are the binding restatement. Scans survive only as external
   falsifiers of a finished solve.
0. The speed-ratio efficiency metric hid coasting (96% shown for a path that
   captured 15.5% of the available energy gain) and no candidate was being
   held to the arrival-heading constraint. Both corrected: energy-fraction
   metric + binary HIT/miss vs position AND heading tolerances, with an
   explicit "nothing feasible" verdict. The coarse 2-phase scan frequently
   CANNOT satisfy both constraints - that gap is exactly what the Step-4
   exact solve exists to close.
1. Constant single-side actions reach only a thin family of endpoint arcs -
   coverage sweeps need at least two phases (near-black heatmap otherwise).
2. The `cosa` action convention is against the REVERSED velocity (wish at
   `wh + pi`); the Step-2 law's sign and the budget-binds claim were wrong
   until measured. See Step 2.
3. CategorizePosition (and so the sf law) runs BEFORE the FinishGravity
   half - a post-tick-vz sf rule disagrees one tick per rise; mid-tick vz
   is correct. Caught by the rollout corpus (119/120 on both rise scripts).
4. The ballistic closed form is exact until |vz| hits the 3500/component
   clamp (annotated in Law B); Step 1 must report the clamp tick.

## Standing rules that bind this work

Engine values measured, never assumed; the model rollout uses the exact
tick map (sim never approximated); every acceptance runs through a
falsifier vs exhaustive/parity ground truth; improvements compose as
portfolios and are monotone against today's solver; reports render outside
the repo.
