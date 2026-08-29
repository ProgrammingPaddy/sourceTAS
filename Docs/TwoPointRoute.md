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

**Findings log** (what the instrument caught, newest first):
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
