# Solver Rebuild — Design of Record

Opened 2026-08-15. Mission: **minimum time from start zone to end zone,
beating a strong human line consistently, unseeded, no route knowledge,
on every map** — starting with linear maps (staged = linear chained by
teleport resets). The certified engine model (Docs/
EngineParityReference.md) is the trusted simulation substrate.

The prior solver's diagnosis (user, near-verbatim): fundamentally too
disjointed — billions of options searched, needless energy lost on
boards and transitions, because per-tick choices were never scored in
the context of the segment, ramp, and run. The human line wins on
smooth air, clean boards, and positioning for the *next* feature.

---

## 1. Objective and legality

- Objective: **ticks between start-zone exit and end-zone entry.
  Nothing else.** Finish speed has zero terminal value.
- Start zone: exactly ONE jump per run; no bhop in zone; prestrafe is
  legal and expected. Exit as late as possible is *subordinate* to a
  clean first board — the first board's quality dominates the few extra
  falling ticks.
- No bhopping ramp spines. (Exact legality rule for mid-run jumps:
  OPEN QUESTION #3 below.)
- Most banned-styles are slower anyway; the design filters them by
  routing value, with hard rules only where value doesn't (spine bhop).

## 2. The human model (distilled from expert testimony, 2026-08-15)

**The board.** A good entry lands with velocity tangent to the ramp
plane — minimal normal component in the approach direction — and the
*entire approach* is part of the interaction: the turn into the board
is taken at the best turning rate for strafe gain given height and
landing options. Never fly fast at a face and crank a late, lossy turn.
With excess speed, dissipate the minimum inside the turn and still
board clean. Limit cases: pure vertical fall → look straight down-ramp
(yaw 90° to the face) and let the board turn the fall into the plane;
head-on approach → a ~90°+ arc (more when board velocity is downward).
**Bad boards kill runs.**

**The carve.** Three simultaneous jobs: preserve the boarded energy;
gain the (mostly negligible, strafe-rate-setting) ramp-strafe energy;
convert height to kinetic energy where possible; and above all SET UP
THE FLICK. The exit point on the face is fully determined by current
speed/energy and the next ramp's position.

**The flick (exit).** Yaw chooses the exit angle. Leaving before the
edge = turn into the ramp, then strafe away toward the target. The best
transitions lose nothing or GAIN; when height must be bought, buy the
minimum. On fast runs the flick also minimizes air time to the next
board — subject to boarding high/far enough to repeat the process.

**Height vs speed.** Fast runs repeatedly convert height into speed;
flatter transfers of equal energy win (more direct). Prioritize
horizontal speed anywhere you can get away with it; buy height only
when overshoot or a bad board forces it. Side boards and mid-face
boards exist; skipping a ramp entirely is a real routing decision.

**Lookahead.** A rolling plan over the next 2→3→4 ramps: choose the
current exit so the next board sets up the ramp after it. Ramps chain:
in isolation a ride cares about the previous and next feature only,
but the chain couples the whole run.

**Air between features.** On long straight transfers, weave — strafe
side-to-side for energy gain while holding net heading. Air gain and
heading change share one budget (the strafe-gain curve); spending it
is a real optimization, not a straight line.

**Start.** Prestrafe: maximize ground speed → the single jump →
maximize air-strafe gain → leave the zone late — but sacrifice
late-exit ticks whenever it buys a better first board.

**Skill tells** (= our loss patterns to kill): snappy yaw spikes (no
plan; late compensation), hard boards, routes that walk into bad
positions. Smooth air + clean boards + next-ramp positioning is the
whole skill hierarchy.

## 3. Representation

### 3.1 Feature graph (from BSP, per map, automatic)
Nodes: surfable features — ramp faces (non-walkable planes, nz < 0.7,
grouped into faces/spines with edges and extents), the start zone, end
zone, triggers (teleports/boosts, already modeled). Edges: candidate
transfers A→B (including skips) gated by coarse reachability envelopes
(gravity + air-gain bounds). Staged maps: the graph fragments at
teleport resets into chained linear solves.

### 3.2 The TRANSFER primitive (the unit of value)
One transfer = exit state on A → air phase → board on B → carve on B →
exit state on B. Everything the expert testified lives at this
granularity. Per feature B and entry state, define:
- **Board window**: region on the face × velocity cone (tangent-dominant)
  that boards with clip loss below threshold.
- **Exit manifold**: the set of exit states (position on face edge or
  early-flick point × velocity) reachable from an entry via legal
  carves, each tagged with time cost and energy delta.
Transfers compose by intersecting A's exit manifold (propagated through
the air phase) with B's board window.

### 3.3 Controls parameterization
Per phase, controls collapse to a **yaw profile over ticks** (+ duck
where relevant); fmove/smove follow optimal-strafe sync mechanically.
Air phase: yaw(t) spline with boundary conditions (exit heading → board
heading). Carve: yaw(t) on the face. This replaces free per-tick
mutation with boundary-conditioned, physically-shaped curves — snappy
yaw is unrepresentable except where the optimum truly is a fast arc.

## 4. Scoring: the regret ledger

Every level is scored as **regret against a physics bound**, not raw
fitness — this is the context-awareness fix.

- **Board loss** (exact): |v·n̂| destroyed at contact — already
  instrumented tick-by-tick (TickEvents.contact_loss).
- **Approach loss**: deviation of the turn from the optimal strafe-gain
  turn rate (the gain-per-degree curve at the 30u/s cap) integrated
  over the approach; late-sharp-turn is a large measured regret.
- **Air-gain shortfall**: energy gained vs the strafe-gain bound for
  the ticks and heading-change actually available (weaving counts).
- **Conversion shortfall** on the carve: kinetic gained vs the
  potential available along the chosen path.
- **Time regret**: transfer ticks vs the flat-transfer bound at equal
  energy.
- **Route regret**: DP value difference vs the best known sibling
  route.

A run produces a LEDGER: loss attributed per transfer, per phase. The
search spends compute where the ledger says, not everywhere.

## 5. Search architecture (anytime, four stages)

1. **Graph build** (offline, seconds): features + edges + bounds.
2. **Route search** (fast, wide): beam/DP over feature sequences using
   analytic bounds (energy in → time bound out per edge). Emits
   thousands of ordered candidate routes; start-zone and skip decisions
   live here. Admissible-ish heuristics keep it honest.
3. **Transfer refinement** (the core): per candidate route, solve each
   transfer's trajectory primitive (yaw-profile optimization on the
   exact engine over short horizons: approach turn, board, carve,
   flick), propagating entry states forward; infeasible transfers
   prune the route immediately. Rolling 2-4 feature window exactly like
   the human: each flick optimized against the NEXT board's window and
   the ramp after it.
4. **Assembly + polish**: chain the transfer solutions into full runs
   on the exact engine; CMA-ES persists ONLY as a local polisher over
   the yaw-spline residuals (small, well-conditioned, boundary-locked),
   seeded by stage 3, never random. The ledger routes further polish to
   the worst transfers.

Anytime property: stage 2 yields finishable chains in minutes; quality
scales with budget. Toggles: beam widths, window depth, polish rounds —
the "2-minute vs 20-minute, 5 ticks apart" dial. If a map can't finish
inside 30 minutes, the ROUTING rules are wrong — fix rules, not budget.

## 6. Validation

- Exact-engine replay + in-game tape verification (existing
  instruments; batch into rare sessions — the design loop is offline).
- **Human demos as validation, never seeding**: hour-long session
  demos exist, ~10-20s of valid completion each, marked by the
  completion chat line. Build a .dem extractor that locates completes
  and emits position/angle traces; compare the expert's ledger (their
  board losses, transfer times) against ours on maps we never tuned on.
- The regret ledger itself is the primary development instrument:
  "needless loss" is a number with an address (map, transfer, phase).

## 7. What survives from the current codebase

Survives: the engine model (all of it), playback/diff/battery
instruments, .tas writer, trigger handling, worker-pool infrastructure,
CMA-ES (demoted to polisher). Dies: corridor beams, raw-fitness
archives, contact-anchored random mutation, dissipation-bias heuristics
— all replaced by the graph/transfer/ledger stack.

## 8. Open questions (answers slot here)

1. Mid-run jump legality: is ANY jump after the start-zone jump
   illegal, or only grounded bhop chains (e.g., on spines)? Exact rule
   wanted for the legality filter.
2. Yaw legality: tick-perfect yaw is allowed (TAS), correct? Smoothness
   is an efficiency emergent, not a rule?
3. Demo handoff: paths to the session files + which maps they complete.
4. Strafe-gain curve: derive the exact per-tick gain-vs-turn-rate law
   from the certified engine (analytic, offline) — first math task of
   the era.
