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
- **Scope pin**: pure airborne flight between the two points (no mid-flight
  board; collision-freedom of the corridor is a separate check against the
  BSP machinery). Boards/ramp contact belong to the entrance-field track.

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
Closed form for one tick: given speed `s` and wish angle `alpha`:
`c = s*cos(alpha)`, `add = clamp(L - c, 0, a)`, new speed and heading turn
from the vector sum. Deliverables: forward map `(s, alpha) -> (s', dtheta)`,
its **inverse** (`(s, dtheta) -> alpha, s'`), and its derivatives (Newton
needs them). This is B22's measured joint frontier made analytic. Gate:
byte-level differential vs the engine tick over a `(s, alpha)` sweep
(FUNCPROBE-style); the B22 table becomes the cross-check, not the primary.

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

## Standing rules that bind this work

Engine values measured, never assumed; the model rollout uses the exact
tick map (sim never approximated); every acceptance runs through a
falsifier vs exhaustive/parity ground truth; improvements compose as
portfolios and are monotone against today's solver; reports render outside
the repo.
