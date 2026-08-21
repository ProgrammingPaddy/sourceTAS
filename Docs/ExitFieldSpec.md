# ExitField — Design of Record (Stage B)

Opened 2026-08-19 (advisor ruling 2026-08-19e, refined 2026-08-19f after
the tree-grounded review). This document is to Stage B what
`Docs/AirRecSpec.md` is to Stage A: every ExitField design decision,
ruling and measured result lives here or is linked from here.
EntranceField is INTEGRATION-READY as of commit 89590d9
(`Docs/AirRecSpec.md` section 25); ExitField is UNFROZEN.

---

## 1. The operator definition (production semantics)

Let `B` be an exact **BoardBoundaryState** obtained by replaying an
EntranceField board witness through the authoritative executor. Let

    U_R = (u_0, ..., u_{M-1})

be a finite canonical per-tick ride schedule. The canonical ride
executor runs the exact engine from `B` until the FIRST of

    AIR_EXIT | CONTACT_TRANSFER | GROUND | END | HORIZON

For a physical boundary event `e != HORIZON`, define the transition

    Z = (dt, e, S+, U_R^consumed)

where `dt` is the exact ride duration in ticks, `S+` the exact
canonical continuation state at a tick boundary, and `U_R^consumed`
the exact consumed input prefix through the event. Then **ExitField
is the set-valued relation**

    R_F(B) = { Z : U_R legal, exact replay from B produces first
               physical boundary event Z }

- `HORIZON` contributes NO transition: it means "the supplied controls
  ended while still riding" and always leaves the domain UNRESOLVED.
  It is an execution/search outcome, never an exit.
- No local score appears in this definition. Scalar max exit energy is
  a DASHBOARD PROJECTION; production preserves the continuation set,
  because only the set-valued relation composes with the next
  EntranceField (the argmax depends on the successor face, which is
  not known locally).
- A collision with anything that is not the ridden face is an explicit
  `CONTACT_TRANSFER` boundary event, never a hidden continuation.
  Whether production later groups geometrically continuous faces into
  a ride surface component is an open topology question; hidden face
  transitions inside a ride witness are forbidden regardless.

## 2. The seams (tick ownership)

One canonical phase, end-of-tick boundaries, no pre-clip quantities:

    A --EntranceField (owns the board tick)--> B
    B --ExitField (owns ride ticks through the last contact tick)--> A+
    A+ --EntranceField--> B'

- `B` = movement state at the tick boundary immediately after the
  accepted board tick. This is exactly `Air::Result::end_state`
  ("the board tick belongs to this flight's last frame"). The mid-tick
  contact point `Q` is event METADATA, not state.
- **AIR_EXIT** (RE-AMENDED 2026-08-19g - the session-13 peek rule is
  SUPERSEDED): **the ride owns and books the separation tick.** An
  exit classified by an unbooked input that the next operator is free
  to replace is not closed under composition - it proves only
  "exists u: next tick separates", and the composed trajectory could
  lawfully diverge from the classification basis. So the first
  clean-air tick is booked to the ride (`dt` includes it), `S+` is the
  boundary AFTER it, the witness carries its input like any other
  booked tick, and Air begins at the following tick. Every
  ExitTransition is self-contained; no tick is double-booked.
  (Measured confirmation: the carve seam bookkeeping difference
  `t25 vs t26` disappeared exactly - X7 now agrees tick-for-tick and
  in exit speed.)
- **CONTACT_TRANSFER / GROUND / END**: the event tick belongs to the
  ride; `S+` = boundary after it (for CONTACT_TRANSFER this is the
  next face's `S_B`, same convention as EntranceField).
- Degenerate case: a board whose next tick is already clean air gives
  `dt = 0`, `S+ = B`. Legal and exact.

## 3. The boundary state S_B

    S_B = (complete PlayerState, Steer::CtlState, ridden-face identity)

- **Complete `PlayerState`, never a hand-pruned subset.** Serialization
  is the whole POD, bit-exact. Fixture ablation may be REPORTED as
  coverage diagnostics; it may NEVER shrink `PlayerState` (a field
  that ablates clean on a static map — `gravity_scale`, `basevel` —
  matters enormously once triggers arrive).
- `Steer::CtlState` (side + dwell age) crosses the seam (Invariant 9).
  It constrains LEGALITY of future inputs, not physics: the executor
  validates schedules against the carried state and refuses illegal
  ones. Dropping it cannot corrupt a replay — it corrupts which
  replays are admissible.
- Ridden-face identity = `Route::Face` index (brush + plane side),
  the operator-semantics context that classifies contacts.
- Diagnostic accumulators (graze totals, logging) are NOT boundary
  state. The test is whether changing a value changes future
  physical/operator behaviour, not whether the harness stores it.
- Duck is not one boolean. `PlayerState` carries five duck-related
  fields (`ducked`, 3-valued `hull_state` including the post-air-
  unduck transient `hull_state == 2`, `ducking`, `duck_timer_ms`,
  `duck_until_ground`); fixtures must exercise the transient and
  in-air transitions specifically.

## 4. The canonical ride control basis

Initial hypothesis (per tick):

    u_k = (side_k in {-1,0,+1}, cosa_k, duck_k in {0,1})
          [+ optional analog magnitude m_k]

- `side = 0` IS the coasting channel (zero wish). `Carve::effort`
  proved coasting is a real independent control capability; its
  duty-cycle spline is a PROPOSAL encoding that may propose canonical
  schedules but never define them. Same for `duck_at`: canonical truth
  carries per-tick duck input.
- Wish emission routes through `Air::WishInputs` — the ONE place the
  stored-basis convention lives. The engine-relevant wish for keyboard
  input is direction-only above the air cap (all full-key combos give
  wishspeed = min(|wishvel|, maxspeed) >> cap), so `(side, cosa)`
  spans the keyboard-legal wish set exactly. Sub-cap (analog partial)
  magnitudes are physically distinct; `exitfit` measures whether the
  basis needs a continuous `m_k` for the full engine-legal set. The
  production input model (the hardware input driver is keyboard-style)
  may adopt the binary restriction only as an explicit, documented
  choice — never silently.
- Legality: dwell-6 on canonical side reversals, validated against the
  CARRIED `CtlState`. Coast ticks advance age and change nothing else.

## 5. Standing laws (Stage B)

- **B-A (uncertified culls do not erase).** `Carve::ExitDoomed` is
  ADVISORY ONLY for all ExitField work: it may rank/order/defer
  reopenable work; it may not terminate canonical execution, erase
  part of `R_F(B)`, declare an exit impossible, or support a global
  hard prune. The canonical executor must not contain it at all. If it
  is ever wanted as a certified cull it gets its own derivation and
  gate suite (the B7 lesson: "optimistic by derivation" is not
  certification).
- **B-B (certified outer envelope, set-valued).** ExitField exposes a
  certified optimistic outer envelope `R_F(B) ⊆ U_F(B)` from the
  start, but it is NOT one scalar U: high energy does not dominate a
  different exit direction. First component: certified ride
  energy/speed ceiling versus tick, horizon-parameterized
  `U_E(B, M)` (a universal finite ride bound is not assumed). Absence
  of a tight bound means UNRESOLVED, not impossible.
- **B-C (finite schedule semantics).** Accepted witnesses store the
  exact consumed prefix through the boundary event, separation tick
  included (section 2, re-amended 2026-08-19g). `HORIZON` =>
  UNRESOLVED, never an exit. Permanent property gate.
- **B-D (reopenable compression, no approximate dominance).** v1
  performs NO approximate hard dominance pruning: exact-state
  dominance essentially never fires on float state, and anything
  discarded for sharing a bucket is compressed/deferred, never
  physically dominated. Frontier diversity is maintained by
  partitioning over future-relevant projections (exit region, tick
  range, heading, speed, vz, side/dwell, hull/duck), all reopenable
  under finer queries. Certified dominance beyond trivial duplicates
  waits for a theorem.
- **B-E (naming).** The pre-existing `Field::ExitMap` /
  `Carve::ExitRec` / `exitgate` / `exitbench` are ANALYTIC/ADVISORY
  estimators and instruments — they order work, they are not the
  transition relation. New canonical objects use non-colliding names:
  `Ride::FlyRideSchedule` (executor), `Ride::BoundaryState`,
  `Ride::Result` (transition record), later `ExitTransition` /
  `ExitQueryState` / `ExitEnvelope`.
- **B-F (quarantine).** Human/carve/tape rides are regression fixtures
  only; synthetic hidden legal rides are the primary completeness
  test. The corrected `Steer::Controller` is proposal machinery only.
  Nothing controller- or human-derived defines whether `exitfit`
  passes or what production proposes.

## 6. EntranceField composition API (stage 0, prerequisite)

`QueryState`'s scalar `L` remains the heatmap/value projection. But
production additionally exposes **multiple distinct replayable
BoardTransition witnesses** — initially drawn from the existing
exactness-rung and heading-interval witness lanes plus winner and
tangent — merged monotonically per lane (a transition once exposed is
never lost; a lane's L only rises). `S_B` is obtained by AUTHORITATIVE
REPLAY of a BoardTransition witness (`Air::FlyWishSchedule` →
`end_state`); any cached state is acceleration, never truth.

    BestBoard is a projection; BoardTransition witnesses are the
    transition payload. A lower-energy board can win the route.

## 7. Stage sequence

    0. Fix EntranceField continuation API (multiple BoardTransitions)   [this session]
    1. Canonical ride executor (Ride::FlyRideSchedule)                  [this session]
    2. exitfit: control + operator-state completeness                   [this session]
    3. Witness-backed ExitField continuation frontier
    4. First certified ExitEnvelope component (U_E(B, M))
    5. Lazy Query/Refine wrapper (mirrors Entrance::RefineStep)
    6. Face-to-face composition  E_{F_j} ∘ R_{F_i}

No ride-quality optimization until 0–2 are green.

## 8. exitfit — what it proves (and what it must not do)

`exitfit` validates the pair `(S_B, U_R)` as sufficient to reproduce
`R_F(B)`. It does not decide which exits are good, contains no
optimizer, no controller, no `ExitDoomed`, no human data in
generation.

Two hypotheses, kept separate:

- **X1 control completeness** — hidden NATIVE legal engine inputs
  (per-tick yaw/fmove/smove/buttons patterns: held key combos, yaw
  ramps, coasts, duck presses; never generated in the canonical
  basis), executed exactly; inverted tick-by-tick into the canonical
  basis THROUGH the same math `WishInputs` defines; frozen; replayed
  by the canonical executor; required to reproduce the trajectory,
  event kind, event tick and continuation state. Preceded by **X1a**,
  the inversion identity property (canonical → emit → invert →
  identical canonical, on a grid).
- **X2 operator-state completeness** — aimed at the SEAM, not at
  `MoveTick` purity (`MoveTick` is pure on `PlayerState`; copy-resume
  of the struct is trivially exact — the Markov risk lives in what is
  NOT in `PlayerState`):
  - **X2a serialize/resume**: checkpoint interior ticks, serialize the
    complete `S_B` to bytes, reconstruct fresh, replay the frozen
    suffix, require bitwise-identical trajectory, event and `S+`.
  - **X2b seam legality**: the carried `CtlState` changes which
    suffixes are ADMISSIBLE — a legal-under-truth suffix reversal is
    accepted with the correct carried dwell and REFUSED with a
    corrupted-younger one.
  - **X2c duck/hull sensitivity**: legal boundary states differing
    only in duck/hull fields (`hull_state == 2` transient, in-flight
    `ducking`/`duck_timer_ms`) must produce diverging rides on at
    least one fixture — proof the fields are load-bearing and that
    the fixture matrix can detect hidden state.
- **XH horizon semantics** — a too-short schedule returns HORIZON and
  is not an exit (B-C's permanent gate).
- **X7 reference regression** — a `Carve::RideHeadingSpline` ride's
  realized native inputs invert and replay through the canonical
  executor to the same exit (regression only, quarantined from
  generation).

Fixture matrix axes: short/long rides, ascend/descend, low/high
entrance speed, gain/brake wish profiles, 0/1/multiple legal
reversals including at the six-tick minimum, varied carried
side/dwell, ducked and unducked entry, duck transitions mid-ride,
ordinary edge separation and duck-off separation, strongly vertical
exits, and the `hull_state == 2` entry transient.

---

## Session log

(append-only, newest last)

### Session 13 (2026-08-19) — stages 0–2 built; the gate found three
### real things on its first run

**Stage 0 — EntranceField continuation API.** `RefResult` gained
per-interval witness lanes (`iv_side/iv_cosa/iv_res/iv_th`, captured at
the strike-binning site alongside `iv_L`); the header gained
`BoardTransition` (stable lane ids: `0x0001` winner, `0x0300` tangent,
`0x0100+r` rung, `0x0200+i` interval) and `QueryState::transitions`;
`RefineStep` merges lanes monotonically (a lane once exposed is never
lost, its L only rises; the replay horizon derives from the WITNESS's
own schedule length, not from whichever lane won the solve);
`ReplayBoardTransition` is the one sanctioned way to obtain `S_B`.
**Gate R7 (efrefine): PASS — 10 lanes, 6 of them heading intervals,
monotone per lane, every lane replays to its recorded L at |dL| 0.00.**

**Stage 1 — the canonical executor.** `Source/Solver/SolverRide.h`:
`Ride::FlyRideSchedule` (header-only, like the other operator layers),
`Ride::BoundaryState`, `Ride::SchedLegal`, `Ride::InvertWishInput`.
Wish emission routes through `Air::WishInputs`; the engine's wish
construction was exposed as `Fn::WishFromInput` (a forwarding mirror,
one authoritative body) so the inverter cannot drift from the tick.
No controller, no splines, no ExitDoomed, no optimizer.

**Stage 2 — exitfit: 7/7 GREEN**, and the first run was RED in three
places, each a genuine finding:

1. **The AIR_EXIT witness needs the peek input.** A witness holding
   only the booked prefix cannot be classified standalone — the
   executor runs out of inputs while the last tick still touches the
   face and honestly reports HORIZON. Resolution (spec section 2
   amended): the peek tick's INPUT is part of the witness — it is the
   existence proof of a clean-air continuation — but its TICK is never
   booked: S+ is the boundary before it, and the next operator
   re-simulates that tick under its own control. The composed
   trajectory may lawfully differ there; the tag is an existence
   statement, the STATE is exact regardless.
2. **Bitwise reproduction of native rides is impossible for any
   single-encoding basis** — two float encodings of the same wish
   differ in the last ulp and clips amplify it (measured: 0.72u over a
   full ride, one fixture of 42). X1's claim was therefore sharpened to
   the INPUT CHANNEL: every tick's canonical emission must realize the
   same physical wish — direction to float precision (measured worst
   5.25e-06 rad over 42 rides), same magnitude class, same duck — with
   the trajectory tracking inside a 2u envelope and the event
   kind/tick exact. That is the representation claim; the envelope is
   the chaos allowance.
3. **Poking `hull_state` up is not a legal transient** — it lacks the
   companion duck fields and the engine normalizes it away (0/12
   divergence). Re-aimed at ENGINE-CONSTRUCTED transients (the factory
   actually unducks in flight), which exposed the real fact:
   `Hulls::unduck_* == stand_*` since 2026-08-15 — the 62.5 transient
   was removed as fit-noise, so hull-2 rides identically to standing
   BY DECLARED MODEL. X2c now asserts consistency with that model
   (divergence would mean collision-model drift) and gates on the
   genuinely load-bearing fields: `ducking`/`duck_timer_ms` diverge on
   3/12 probes (the mid-ride transition applies the origin shift).

The final board:

| gate | result |
|---|---|
| X1a inversion identity | 176 ticks round-trip, 0 mismatches, worst dcosa 1.2e-06 |
| X1 control completeness | 42 rides (2 analog): wish-dir worst **5.25e-06 rad**, 0 mismatches; events exact; envelope worst 0.72u of 2u |
| XH horizon semantics | cut schedule returns HORIZON, never an exit |
| X2a serialized resume | **126 checkpoints bitwise equal** (traj, event, tick, S+) |
| X2b seam legality | carried age 2 accepted, corrupted age 1 refused at the right tick |
| X2c duck/hull | ducking/timer load-bearing (3/12 diverge); hull2 = standing per model (0 diverge, expected 0) |
| X7 carve regression | carve exit t26 vs canonical t25 (seam bookkeeping), speed 533.6 vs 533.2 |

Fixture matrix: 42 accepted of 96 generated (discards reported: 15
short, 7 dwell-illegal), 8 strata, 4 faces, 21 with carried control
state, 7 hull-2 entries, 2 analog.

**Known coverage holes, recorded not hidden:** every accepted fixture
ends AIR_EXIT or HORIZON — `CONTACT_TRANSFER`, `GROUND` and `END` are
classified by code paths no fixture exercises yet (basictest's open
ramps make them hard to hit with blind native generation). Stage 3's
frontier generation will produce transfers naturally; a
transfer-bearing fixture is a requirement BEFORE the composition
milestone, not after. The analog stratum confirms the basis needs the
optional per-tick magnitude only for sub-maxspeed wishes; the keyboard
input model (the production choice, matching the hardware input
driver) never needs it — adopted explicitly per spec section 4.

**Classification comments filed at the source** (laws B-A/B-E):
`Carve::ExitDoomed` is uncertified-advisory for all ExitField work and
must never appear in the canonical executor; `Field::ExitMap` is an
analytic advisory estimator, not the transition relation.

Whole-board regression from a clean rebuild: exitfit 7/7, efrefine 7/7
(R7 new), airprops 24/24, airsuite 48/48 gap 242k, airrec fixture
VERIFIED, wishparity PASS, strafelaw float-ULP, carve M1.4, airsolve
M1.3.

**Next (stage 3): the witness-backed continuation frontier** —
`ExitField(S_B)` returning the set of exact transitions with reopenable
partition compression, no dominance pruning, plus the first
`CONTACT_TRANSFER` fixtures. Then stage 4 (first certified
`U_E(B, M)` envelope component), stage 5 (lazy Query/Refine), stage 6
(face-to-face composition).

### Session 13b (2026-08-19) — the five pre-stage-3 corrections; the
### representation gate is 10/10

The advisor's directive before stage 3, implemented in full.

**1. The AIR_EXIT seam is closed under composition.** The session-13
peek rule (input in the witness, tick never booked) proved only
"exists u: next tick separates" — the next operator was free to replace
that input and lawfully diverge from the classification basis. RE-AMENDED
(spec section 2): **the ride owns and books the separation tick.** `dt`
includes the first clean-air tick, `S+` is the boundary after it, the
witness carries its input like any other booked tick, Air begins at the
following tick. Every event kind now books its own tick uniformly, and
the shared `ClassifyTick` helper means the two executors cannot drift.
Measured confirmation: X7's carve seam difference (`t25 vs t26`)
disappeared exactly — tick-for-tick and exit-speed agreement.

**2. Search coordinates are separated from the exact witness.**
`Ride::MoveInput` freezes PRECISELY the arguments `MoveTick` consumes
(all six, pitch/umove included — an exact packet is exact);
`FlyRideSchedule` records the packet of every booked tick;
`Ride::FlyRideInputs` is the authoritative replay that consumes packets
verbatim. **X1b: canonical packets replay bitwise AND a native ride's
own packets replay the native ride bitwise — 0 bad over 36 fixtures.**
Candidate generation may be approximate; accepted witness replay is
exact — the 0.7u inversion drift lives in re-emission and the packet
witness removes it entirely. The compact basis keeps its role as
coverage-tested search coordinates.

**3. Coast-through-dwell is a gated invariant (X3).** A reversal after
two coasts on carried age 1 (age 5) is REFUSED at the right tick; after
three coasts (age 6) it is ACCEPTED — coasting keeps the last nonzero
side and the age increments through it, in both directions. Two
schedules differing only in coast-tick cosa ride bitwise identically,
and `Ride::CanonSchedule` pins the ignored fields so future frontier
hashing cannot see phantom distinctions.

**4. Event coverage is deterministic (XE).** CONTACT_TRANSFER, GROUND,
END and HORIZON-refusal are all exercised:
- **GROUND x2** arise naturally (rides into the valley floor).
- **END** via a zone volume straddling a known ride path (entered t6).
- **CONTACT_TRANSFER** required real work. Measured: 24/24
  adjacency-driven probes on basictest end in AIR — its ramps never
  touch within ride scope (historical "taps" were air-phase strikes);
  surf_climb extracts no faces and surf_speed_the_movie has 2 with no
  contact either. Per the advisor's synthetic-geometry directive,
  `World::AddTestBrush`/`FinalizeTestWorld` (harness-only, additive,
  running EXACTLY the loader's finalize path and exercising the
  grid-fallback trace that leaf-less maps already use) builds a tiny
  two-wedge valley; the ride boards wedge A, holds a gain wish 20 face
  ticks and contacts wedge B at t21 — **CONTACT_TRANSFER with
  `board_face` resolved to the other route face.** A physics/event
  unit fixture, not a map route.
- **HORIZON refuses MakeTransition** (no transition, UNRESOLVED).

**5. Typed transitions exist.** `Ride::ExitTransition` (kind, exact
`dt`, `s_plus` + carried `ctl_plus`, `board_face` for transfers, the
exact packet witness, event metadata) + `MakeTransition`, which refuses
HORIZON and short witnesses. The route assembler dispatches on kind:
AIR_EXIT feeds Air/EntranceField; CONTACT_TRANSFER is already a board
contact (no Air leg); GROUND terminal until a Ground operator exists;
END is the finish.

**One honest quantization note (X1 gate made angle-aware):** the cosa
chart loses angular resolution as sqrt(ulp) near the degenerate
directions (cos then acos), measured 8.3e-05 rad on brake-stratum
ticks with |cosa| ~ 1 versus 5.25e-06 in the well-conditioned band.
That is a property of the stored parameterization, not the inverter —
and irrelevant to witnesses, which are packet-exact (X1b). The gate is
2e-5 in-band, 2e-4 for |cosa| > 0.999, both reported.

**The board: exitfit 10/10** (X1a, X1, X1b, XH, X2a, X2b, X2c, X3, X7,
XE). Whole-board regression from a clean rebuild: efrefine 7/7,
airprops 24/24, airsuite 48/48 (gap 242k), airrec fixture VERIFIED,
wishparity, strafelaw, carve M1.4, airsolve M1.3.

**Stage 3 is now safe to begin**: witness-backed, event-partitioned
continuation frontier — Carve/controller/ExitMap as proposal/ordering
only, every retained transition from exact canonical replay, reopenable
partition compression, no approximate dominance. Then the first
certified finite-horizon ExitEnvelope component, `ExitDoomed` staying
advisory.

### Session 14 (2026-08-19) — stage 3: the ExitWitnessFrontier, 8/8

Advisor ruling 2026-08-19h implemented. The headline distinction is now
in the data model, not just prose:

    W_F(B)  SUBSET OF  R_F(B)  SUBSET OF  U_F(B)   [stage 4]

**The operator is not the frontier.** `ExitField::WitnessFrontier`
(`Source/Solver/SolverExitField.h`) is the witnessed LOWER
approximation - the set-valued analogue of EntranceField's L. Absence
from it never acquires unreachable semantics; the status enums have no
"dead" or "impossible" value by construction (`kUnexplored / kActive /
kCompressed`).

**GROUND is not terminal.** Partitions of kind GROUND carry
`continuation_unsupported = true` - the exact continuation is
preserved; the global layer must refuse to prove a route irrelevant
because its next operator is unbuilt. (User note, recorded: some
walkable ground contact is always bad - fail zones - some optional;
classifying that is map semantics, deliberately undefined at this
stage.)

**Compression is resolution only.** Diversity slots (earliest exit /
fastest / highest vz / max energy - no single local objective exists)
hold up to `ActiveCap(level) = 2 + 2*level` representatives per
partition; displaced or capped members become DEFERRED HASHES with the
partition marked `kCompressed`. Reopen provenance = the deterministic
proposal enumeration itself (`MakeProposal(index)` is a pure function):
refinement re-enumerates, dedupes by witness hash, and previously
deferred members MATERIALIZE under the larger cap - measured by F5. A
re-encountered known hash is NOT a no-op; that re-encounter is exactly
how regeneration works.

**Proposal families (production, quarantined):** constant-side x cosa
x horizon (+ late duck-hold), single reversal, coast-prefix + duck-off,
finer cosa ladder + late reversals - 72/132/204 proposals at levels
0/1/2, all through `SchedLegal` and the canonical executor, no Carve,
no controller, no tapes, no ExitDoomed.

### The gates (exitfrontier, 8/8)

| gate | result |
|---|---|
| F1 exact frontier truth | 25 active members replay bitwise to (kind, dt, S+) |
| F2 event partition integrity | 0 mismatches; GROUND partitions all flagged unsupported |
| F3 monotone witness knowledge | all L0 hashes survive to L2 (7 -> 31 known); 0 actives lost |
| F4 no false dominance | 3 compressed partitions, every one carries deferred hashes |
| F5 partition reopening | compressed partition: 4 actives at L2 -> 6 at L3, a previously deferred member materialized |
| F6 transfer composition parity | handoff S+ **bitwise**; composed suffix bitwise for every booked tick; early stop matches a real contact in the continuous run |
| F7 proposal quarantine | ExitDoomed cull toggled -> hash-identical frontier |
| F8 recoverability smoke | 7/8 hidden off-family rides recovered (miss: kind 0 dt 19, reported) |

Two gate-level findings worth keeping:

1. **F6's first failure was the gate's, not the seam's.** The composed
   run lawfully STOPS EARLY at its own next boundary - at the crease
   the hull touches wedge A again while riding B, a real ping-pong
   transition. Parity's honest statement: bitwise handoff, bitwise
   state agreement for every tick the composed run booked, and the
   composed event must correspond to a real contact/ground in the
   continuous run at that tick. All three hold.
2. **Compression status must mean "has deferred members", however they
   got there** - the first implementation only marked cap-refusals,
   missing displacement-deferrals, which made F5's precondition
   unfindable. Fixed in `Insert` (and a partition whose deferred list
   empties reverts to `kActive`).

F8's one miss (a short-duration heading bucket) is the smoke test doing
its job - a structural note for the proposal families, not a blocker.

Board from a clean rebuild: exitfrontier 8/8, exitfit 10/10, efrefine
7/7, airprops 24/24, airsuite 48/48, airrec VERIFIED, wishparity,
strafelaw, carve M1.4, airsolve M1.3.

**Next (stage 4):** the first certified outer-envelope component
`U_E(B, M)` - derived from the ride ledger laws and exact phase
semantics, NOT copied from the Air cap^2/tick law; certified as
"for all Z in R_F(B) with dt(Z) <= M: E(S+(Z)) <= U_E(B, M)"; loose is
acceptable. Then stage 5 (lazy Query/Refine mirroring
Entrance::RefineStep: W only grows, U only tightens), and stage 6 -
the first composed transfer `B_i -> ExitField -> Air -> EntranceField
-> B_j`, where the rebuilt pieces become the full-map solver.

---

## The horizon law (standing, advisor 2026-08-19i)

**M is part of the physical query domain, not the refinement profile.**
`R(B, 40) SUBSET OF R(B, 80)`: increasing M enlarges the feasible set,
so `U_E(B, 80)` may legitimately exceed `U_E(B, 40)` - that is domain
expansion, not a monotonicity failure. The set-valued anytime contract

    W grows, U tightens

is asserted ONLY for fixed `(B, M)`. Extending the horizon is an
explicit `ExtendHorizon(M -> M')`, never a `coarse/medium/fine` side
effect - otherwise Stage 5 acquires the exact "U unexpectedly loosened"
ambiguity that took so long to eradicate on EntranceField. Gate E7 pins
`U_E` monotone in M; global branch-and-bound naturally supplies the
competitive horizon (a continuation needing `dt >= T* - g(S)` cannot
improve the incumbent), so no universal maximum ride horizon is ever
required.

Also standing from the same ruling: `W_known` (hashes + provenance,
never shrinks) is distinct from `W_materialized` (active
representatives, may recompress); and F8's 7/8 stays as recorded
evidence until Stage-5 profiles rerun it across resolution - proposal
vocabulary work happens only if the miss survives strong refinement.

### Session 15 (2026-08-19) — the three corrections + U_E(B, M) CERTIFIED

**Corrections (advisor 2026-08-19i), all in before the bound spread:**

1. **Spatial regions in partition keys.** `RegionBits` projects the
   event position into a face-local frame (`FaceFrame`: in-plane u/v),
   128u cells, 8 bits in the key. AIR_EXIT keys carry the exit region
   on the RIDDEN face; CONTACT_TRANSFER keys carry the contact region
   on the TARGET face. Two exits from opposite ends of a ramp now live
   in different partitions the global solver can request separately.
   The 128u cell is a coarse resolution setting; finer splitting is
   refinement's job.
2. **Versioned reopen provenance.** `kProposalVersion` +
   `ProvenanceHash` (proposal version, engine/control/collision model
   versions, cap/dwell/dt params, family sizes) stamped on every
   frontier. A deferred hash is regenerable only under the same
   provenance; anything else is a migration by replay. Same lesson the
   witness bank learned.
3. **F6 strengthened + F9 added.** F6 now requires the composed event
   to be the SAME EARLIEST post-handoff boundary event as an
   instrumented continuous run - same tick, same kind, same contacted
   face, no earlier boundary skipped (measured: kind 1 t1 == kind 1
   t1). F9 pins deterministic simultaneous-event precedence
   END > GROUND > CONTACT_TRANSFER > AIR_EXIT with a genuine
   same-tick fixture (transfer tick + a +-3u END zone entered ON that
   tick: classifies END). exitfrontier is 9/9.

**Stage 4 — the envelope, derived then certified (exitenv 8/8):**

    for all Z in R_F(B), dt(Z) <= M:
        E_boundary(S+(Z)) <= U_E(B, M)
        = E(B) + cap^2 M + 2 g gs duck_air_shift + eps M

- **Phase first:** E := |v|^2 + 2 g gs z at the exact s_plus boundary
  (after FinishGravity). The half-tick leapfrog conserves E EXACTLY in
  real arithmetic (d(vz^2) = -2Gvz + G^2 against d(2g'z) = +2Gvz - G^2)
  - E1 measures realized drift at one float ulp per tick (0.5 at
  |E| ~ 6.4M), eps = 2.0/tick.
- **The wish law is DERIVED in the ride domain, not copied from Air:**
  dE = 2a*proj + a^2 with a <= addspeed = min(wishspd, cap) - proj
  gives dE <= cap^2 - proj^2 <= cap^2 = 900 for ANY legal wish, any
  wishspeed, any surface friction, any speed. E2 measured the realized
  one-tick maximum at **899.00 of 900** across 576 legal wish ticks -
  the law is tight.
- **Clips never add energy** (E3: 2000 random clips through the
  authoritative helper, worst expansion 0.0000).
- **Duck origin is one net shift, not per-tick slack:** transitions
  alternate, so the prefix sum of +-8.5 shifts is at most one +8.5 -
  the certified slack is a single 2 g gs duck_air_shift (~13.6k). E4
  measured the duck-tick jump at +14,486 (shift + that tick's wish
  work) with the envelope holding, worst margin 2191.
- **E5:** 1,918 booked boundary states across all 204 family schedules
  + 40 random-legal schedules: 0 violations. **E6:** every frontier
  transition contained at its own dt. **E7:** the horizon law. **E8:**
  removing the hull term turns the suite RED (1 violation) and the
  measured 899.0 max proves a 50-point understatement of the wish law
  would be detected - the suite is strong enough to notice broken
  semantics (the B5-passed-while-broken lesson, closed by
  construction).

**Domain, scoped honestly (D_static-surf):** static geometry, constant
gravity_scale, basevel == 0, no triggers, wish+duck inputs only, every
booked tick starts airborne. Outside that domain the environment enters
the operator state and a new bound version gets certified.

`U_E(B, M)` enters the certified registry as bound id `45580001`.

Board from a clean rebuild: exitenv 8/8, exitfrontier 9/9, exitfit
10/10, efrefine 7/7, airprops 24/24, airsuite 48/48, airrec VERIFIED,
wishparity, carve M1.4, airsolve M1.3.

**Next (stage 5):** the lazy Query/Refine wrapper for fixed `(B, M)` -
`ExitQueryState = (W, U, partitions, deferred provenance, status,
profile trail)`, refinement profiles varying proposal effort /
partition resolution / active caps, never B or M; `W_known` never
shrinks, `U` only tightens. Then stage 6, the composition:
`B_i -> ExitField -> Air -> EntranceField -> B_j` with exact tick
accounting, control-state carry, packet concatenation, replay of the
composed witness through the uninterrupted engine - and separately
`B_i -> CONTACT_TRANSFER -> B_j` with no fake Air segment.

### Session 16 (2026-08-20) — proofs repaired, 45580001 CERTIFIED,
### stages 5 and 6 built: THE FIRST REUSABLE FULL-MAP EDGE EXISTS

**Proof repairs (advisor 2026-08-19j), then promotion:**

1. **The wish-work derivation is fixed.** The invalid intermediate
   `dE <= c^2 - p^2` (wrong for p < -c, where convex f(a) = 2ap + a^2
   beats a negative c^2 - p^2 at a = 0) is replaced by the clean chain:
   a <= c - p gives p <= c - a, so
   dE = 2ap + a^2 <= 2a(c-a) + a^2 = c^2 - (a-c)^2 <= c^2 <= 900,
   with addspeed <= 0 giving a = 0, dE = 0. Braking included.
2. **The FP slack is DERIVED, not measured.** Conservative forward
   error inventory under the certified engine magnitude clamps
   (per-component |v| <= maxvelocity 3500, BSP |z| <= 16384, |E| < 2^27
   where one ulp is 8): gravity 2 ops, wish ~12, clips <= 4 bumps ~240,
   integrate ~7, E-evaluation twice ~72 - inventory < 332, certified at
   512/tick with headroom. E1's observed 0.5/tick stays as evidence of
   conservatism, never justification. E8's hull-mutation still turns
   RED at eps = 512, so detection survives the looser slack.
3. **The clip premise is analytical**: v' = v - beta (v.n) n gives
   |v'|^2 - |v|^2 = beta(beta-2)(v.n)^2, non-expansive for
   0 <= beta <= 2; the authoritative helper uses overbounce beta = 1
   exactly; creases are repeated beta = 1 projections; allsolid zeroes.
   E3's probes falsify drift; the premise is the proof.

exitenv 8/8 under the repaired proofs: **bound id 45580001 is
CERTIFIED** (domain D_static-surf).

**Stage 5 (exitlazy 8/8):** `ExitField::ExitQuery` - the fixed physical
domain `(exact B bytes, ctl, face, M, WorldIdent, ProvenanceHash)` as
the query key; **W_known partition-independent** (authoritative
append-only transitions + hashes; `Rebin` is a pure derived view);
honest statuses (kQUnexplored / kQUnresolved / kQRefined - deliberately
no RESOLVED: scalar U_E is one certified projection, not completeness);
stable profile ids; provenance-mismatch REFUSAL. Measured: W_known
7 -> 16 -> 31 append-only with zero replay failures; materialization
churns (21 -> 18 actives) while knowledge stays byte-identical; double
rebin byte-identical; U_E constant across profiles; repeat profile
finds 0 and invalidates nothing; mismatched provenance refused;
(B, 40) vs (B, 80) split keys with U_E(B,80) > U_E(B,40) and M
immutable; two fresh queries with the same profile sequence are
identical in every observable.

**Stage 6 (faceleg 6/6): the composition.**

    B_i -> ExitField -> Air -> EntranceField -> B_j     2 AIR legs
    B_i -> CONTACT_TRANSFER -> B_j                      direct leg

- **C1/C2/C4:** composed B_j == one continuous exact-engine run,
  BITWISE; dt_leg = dt_exit + dt_air with no +-1 anywhere (measured
  convention: `Air::Result::tick` already counts through the strike
  tick, so dt_air = ar.tick); every ride tick except the separation
  tick touches face i, exactly one face-j contact and it is the final
  booked tick.
- **C3:** the dwell law survives both seams - B_j's carried (side, age)
  equals an independent whole-leg walk (ride inversion + air schedule).
- **C5:** the direct transfer leg composes bitwise with dt_leg =
  dt_exit and NO Air segment.
- **C6:** two distinct exit witnesses from one B_i reach the same face
  with bitwise-distinct B_j, both kept (12 transfer witnesses
  available on the synthetic valley) - no max-energy collapse.
- **C8:** every edge rebuilds cold from (B_i, ride packets, air
  schedule, face_j) alone. **C7:** no tape anywhere on the path.

**What composition needed, measured not assumed** (all three were
found by the continuous-replay gate failing honestly first):

- A 700 u/s into-face board is a FACEPLANT (65% speed loss, exits
  dt<=4 at ~200 u/s - nothing to compose). Plausible boards ride.
- Down-slope rides exit LOW and DIVING (vz ~ -0.8 speed): they fall
  below every board window. The composable exit class is FAST, FLAT
  and HIGH (the human's 94-tick transfer flew ~1000u nearly level);
  lateral high boards produce it.
- The entrance flight horizon comes from the GEOMETRY
  (distance/speed), not a fixed n_hint: 60 capped the search at ~100
  ticks and found zero strikes ever; the real transfers need 100-160.

These fixture lessons are recorded because they preview the GLOBAL
layer's job: choosing B_i, aim points and horizons is route planning,
and the fixed-horizon lesson is exactly the competitive-time-horizon
argument (T* - g(S)) arriving early.

Board from a clean rebuild: faceleg 6/6, exitlazy 8/8, exitenv 8/8,
exitfrontier 9/9, exitfit 10/10, efrefine 7/7, airprops 24/24,
airsuite 48/48, airrec VERIFIED, wishparity, carve M1.4, airsolve
M1.3.

**The project state: EntranceField and ExitField both exist as
certified, witnessed, composable operators, and a face-to-face edge
has been constructed, verified bitwise against the uninterrupted
engine, and rebuilt cold. The next layer is the global graph/best-first
search over these edges.**

### Session 17 (2026-08-20) — G0: THE GLOBAL EXPLORER, 10/10.
### "The rebuilt operators autonomously form real routes."

Advisor ruling 2026-08-20 implemented: the project is no longer
operator development - `Source/Solver/SolverGlobal.h` is the full-map
solver's substrate.

**The data model carries the laws:**

- `GlobalSearch::Node` = exact `BoardBoundaryState` + exact elapsed
  ticks g. `succ_complete` exists and is ALWAYS false in v0 - nothing
  may set it: a discovered edge is exact and witnessed, the successor
  set is never complete, and absence from an edge list never means
  nonexistence.
- `GlobalSearch::TransferEdge` = the production edge contract: ride
  packets + optional canonical Air schedule + destination boundary
  state + provenance; `ReplayEdge` reproduces it cold, bitwise.
  AIR / CONTACT / END kinds mirror the event typing.
- Cost is TICKS ONLY (`g`, objective min finish tick). h_order and
  h_cert are separate; h_cert = 0 in G0; the ONE certified elimination
  is exact duplicate-state dominance (byte-hash equality, static-world
  theorem), every prune carrying a `PruneProof`.
- `OfferFinish` is the single incumbent code path; `HorizonFor`
  derives competitive local horizons from T* - g - 1 once an incumbent
  exists (a physical restriction, not a budget; before an incumbent,
  the explicit finite domain M0 = 240).

**The G0 fixture** (all engine-generated, no reference data): two surf
wedges separated by an AIR GAP (the crease variant ping-pongs in
1-tick hops - measured), a production `LaunchFrontier` of two drop-in
launches with exact launch tick counts, and an END zone placed by an
engine probe ride on wedge B (+-56 at the mid path; +-90 swallowed the
boarding region and every finish tied at one tick; +-48 at the quarter
point was never crossed - all measured, all recorded).

**The run:** 8 nodes, 17 edges, 9 dominance prunes, and the explorer
autonomously assembles

    LAUNCH -> B_0(wedge A) -> [exit, air, board] -> B_1(wedge B)
           -> [ride] -> END        at T* = 66

**The gates (10/10):** G1 all 17 edges cold-replay; G2+G3+G4 the
incumbent route replays continuously through one PlayerState with 57
ticks == sum(dt) and g(B0) + route == T*; G5 exact-state dominance
with proof records (and differing ctl.age NOT merged); G6 every
expanded node still `succ_complete == false`; G7 re-refining an
expanded node discovers new edges (2 -> 4); G8 the incumbent is
production-established, equal/slower offers refused - with the honest
caveat recorded below; G9 every issued horizon <= T* - g - 1; G10 the
inverted expansion order changes nothing physical (same T*, same
edges, same nodes); G11 no tape anywhere; G12 the incumbent rebuilds
cold edge-by-edge.

**G8's honest caveat (open obligation):** this fixture physically
admits exactly ONE finish time - every discovered finish and every
constructed variant ties (measured across zone sizes, placements,
wish variants, deeper refinement, and dominated-route reconstruction).
So "a later faster witness replaces the incumbent" is verified at the
CONTRACT level (a labeled synthetic faster offer through the one
`OfferFinish` path), not by a naturally discovered witness. The
natural-replacement demonstration transfers to the first real-map run,
where finish diversity exists. Recorded here so it cannot silently
become "demonstrated".

**What G0 deliberately does not claim:** optimality certification
(h_cert = 0), successor completeness anywhere, or a real-map route.
The next measured step is the basictest full run - LaunchFrontier from
the real spawn, the real end zone, and the G8 natural replacement -
and after its explosion is MEASURED, the weakest useful certified
remaining-time bound for G1 optimality proof. Per the standing
prohibition list: no ExitDoomed certification, no approximate global
dominance, no beams, no top-K route shapes, no ML ranking, no
universal ride-horizon proof.

Board from a clean rebuild: groute 10/10, faceleg 6/6, exitlazy 8/8,
exitenv 8/8, exitfrontier 9/9, exitfit 10/10, efrefine 7/7, airprops
24/24, airsuite 48/48, airrec VERIFIED, wishparity, carve M1.4,
airsolve M1.3 - thirteen suites.

**The project state, in the advisor's words: this is a pathfinding
engine substrate. The full-map solver has begun.**

---

### Session 18 (2026-08-20) — G0R attempted, G1M DELIVERED:
### the first real-map global runs, and what they measured

Advisor ruling 2026-08-20b: run the explorer cold on surf_basictest
as an INSTRUMENTED EXPERIMENT — "do not only report the final route;
the search-shape measurements are what decide G1." Eight runs were
made; each measured a specific starvation, each got a specific
ordering/coverage fix (never an admission rule, never operator
research), and the run log below is the deliverable.

**Pre-run additions (all landed; groute is now 11/11):**

1. **G13 HORIZON BOUNDARY** (groute): with the fixture's real finish
   ride (dt_e ticks), T* is set so that finish would land at T*-1 /
   T* / T*+1. Measured: M = dt_e ADMITS it (END books at exactly tick
   dt_e == M — no off-by-one truncation; OfferFinish accepts);
   M = dt_e - 1 excludes the tie and the offer is refused; slower
   likewise. The "< incumbent" convention cannot cut a
   one-tick-faster route.
2. **STABLE UNRESOLVED-DOMAIN IDENTITY** (`GlobalSearch::DomainAudit`)
   — per-node records with FNV identity over (node hash, kind, face,
   variant); kinds exit/entrance/end/launch; statuses unexplored /
   unresolved / witnessed (deliberately NO "impossible"); outcome
   reason codes; attempts update a record, identity never changes.
   This ledger made every diagnosis below a one-glance read.
3. **TOPOLOGY FALSIFICATION**: v0 candidates = ALL faces;
   `Graph::topo_violations` armed on every realized edge (0 across
   all runs). The DIAGNOSTIC counter measuring the distance-gated
   Route prediction registered THOUSANDS of realized transfers the
   gate would have missed (it has no self-edges, and same-face
   re-entry is real and load-bearing) — the gated topology is hereby
   measured UNSAFE as an admission rule.
4. **GlobalRouteWitness** (Launch + TransferEdge[] + END) with
   continuous `ReplayRouteWitness` through ONE PlayerState;
   `LaunchWitness` = exact ground packets + air schedule;
   `ReplayLaunch` cold.
5. **END-VIA-AIR**: `FlyZoneSchedule` (the zone is a position box,
   not a face; zone tested first per tick, mirroring the END > GROUND
   > CONTACT precedence); `kEdgeEnd` with dt_air > 0; `ReplayEdge`
   extended. The sound END set for brush 10 = the Minkowski
   intersection box (XY +16, Z [bmin-54, bmax]): every claimed finish
   is a hull touch in EITHER hull state. Stated completeness gap:
   standing-only underside touches with feet in [bmin-72, bmin-54).
6. Incumbent history (`Graph::history`), finish-offer counter, and
   the full G1M instrument set (per-phase timers, branching/depth/
   horizon distributions, queue high-water, per-face populations).

**The map, production-measured:** 4 surf faces — south-facing f0/f1,
north-facing f2 (an orientation flip), west-facing f3 (a second) —
END platform brush 10 across a ~320u air gap east of f3. Spawn on
brush 6. f0 is the ONLY ballistically reachable first face.

**The run log (every fix is ordering/coverage policy; physics and
operators untouched):**

- **Run 1 — the launch aim.** 24-direction fan -> 9 airborne -> ZERO
  boards from 172,800 entrance evals. The acceptance ball is a REGION
  query; centroid/low-region aims sat 130-190u from where ballistic
  arcs meet the faces. FIX: `BallisticAim` — the pure-coast arc's
  plane crossing as the first aim (geometry from the current state,
  never route knowledge). After: 6 exact roots on f0.
- **Run 2 — the service flood.** min-g service flooded f0's cheap
  self-re-entry hops (133 states / 10 expansions; one node grew 37
  f0->f0 edges) and starved every later face. Entrance = 94% of
  wall-clock. FIX: face-round-robin breadth + coarse-only level-0
  entrance (`lvl0_coarse`).
- **Run 3 — two self-inflicted regressions, caught by the ledger.**
  Pure breadth kept every node at coarse (nobody laddered to the
  profiles that find f1) and the nh formula divided by CURRENT exit
  speed, cutting fast nodes to M 105 where the known f0->f1 witness
  needs ~160+. FIXES: geometry-only horizons at a 300 u/s reference
  speed (a horizon is a physical domain, not a per-state discount);
  interleaved work lanes; entrance-query persistence (`ecache` — the
  entrance mirror of the exit qcache; RefineStep was already
  incremental, the harness had been discarding its state).
- **Run 4 — f0->f1 RESTORED** (6 edges at fine profile, M 192), but
  the cascade stalled: f2 faces NORTH and sits SOUTH of f1, so its
  feeders are f1's south-DIVING exits — exactly the class the
  fast/flat/high ordering prior deprioritizes. One score cannot rank
  feeders for every transfer orientation. FIXES: exit DIVERSITY
  (tried exits round-robin (vz class x heading quadrant) buckets);
  long-axis SPREAD AIMS; DEFER_FAR (no coast crossing => coarse only
  until the node deepens — a deferral, never a ban).
- **Run 5/6 — the board-position lesson.** f1 grew to 504 nodes (one
  entrance domain alone carried 81 edges) but f2 stayed zero; the
  --diag-exits inventory showed the deepest f1 node boards at
  x = -551 (f1's far WEST end) and every south-diving exit passes
  200-380u west of f2 with arcs falling under its polygon. The
  feeder class = EAST-half f1 boards — present in the graph, never
  deepened, because min-g laddering always picks a face's earliest
  (= west) arrivals. FIX: within-face deepen diversity — position
  buckets along the face's long axis, rotated per visit; 2:1
  deepen:breadth interleave.
- **Run 7 — THE MARCH WORKS.** f0(346) -> f1(452) -> f2(324 nodes)
  autonomously in one cold run; the east-f1 board's f2 domain carried
  62 edges at coarse/medium (M 80) — once the right board laddered,
  the transfer was easy. f3: zero.
- **Run 8 (3600s) — the honest wall.** 336 expansions, 1940 states
  (f2 = 865), 1944 edges, every one cold-replaying bitwise, 0 topo
  violations — and f3 still zero. The measured reason: **SPEED
  COMPOUNDS ACROSS HOPS.** The f2 arrivals (300-500 u/s, fed by the
  cheap short-hop transfers found first) physically cannot bridge
  the ~300u gap to f3's window: a measured 319 u/s exit falls ~350u
  in the crossing time; the bridge needs roughly 600+ u/s at exit
  height. The deficit chains back through every hop to the 250 u/s
  run-off launch. This is not an aim, horizon, resolution, or
  topology problem — those were each found and fixed above. It is a
  route-quality dynamic: the explorer reaches faces through the
  cheapest transfers, and cheap arrivals are slow arrivals.

**G1M — the measurement the advisor asked for:**

- COMPUTE: EntranceField refinement is 98-99.5% of ALL wall-clock in
  every configuration (run 8: 43.4M flight evals / 3628s of 3646s).
  Exit queries, zone probes, launch, and global bookkeeping are
  rounding errors (< 0.2s each per run). Any G1/perf work that does
  not attack entrance spend is attacking noise.
- STATE-SPACE: same-face hop proliferation dominates (~90% of nodes
  are ride-hop variants; queue high-water 1883). Exact-duplicate
  dominance almost never fires on real geometry (13 prunes / 1940
  states) — every hop differs in bytes. The advisor's predicted
  "many exact state variants on same face -> state handling policy"
  is the measured reality.
- BRANCHING: median 3, mean 3.8-7.0, max 53 new edges per expansion.
  Depth reached 9. Horizons: every expansion ran at the explicit
  M0 = 240; competitive horizons NEVER engaged (no incumbent) — the
  entire T*-feedback loop is unexercised on the real map.
- The failure-mode ladder the advisor listed (launch coverage /
  topology / entrance coverage / exit coverage / horizon selection /
  branching / refinement / compute) was descended one measured rung
  at a time; the run log above is that descent, and the bottom rung
  is ROUTE-QUALITY ORDERING (speed compounding), not any of the
  layers above it.

**G0R: NOT REACHED.** Recorded plainly. The march mechanism is
validated three faces deep with bitwise replays throughout; the
finish needs fast chains, which needs the service/ordering layer (or
the launch) to value speed the way the objective ultimately will —
an advisor ruling, not a unilateral hack.

**Open instrumentation added for the next run:** per-face arrival
speed distributions (median/max), and --diag-exits now dumps each
face's fastest deepest node.

**What was deliberately NOT done:** no operator changes, no admission
rules, no ExitDoomed, no approximate dominance, no beams/top-K, no
human data anywhere (G11 structural: gmap never opens a tape), and no
chasing G0R past the measurement mandate.

---

### Session 19 (2026-08-20) — the resource-preservation service lane:
### G0Q measured, and the 84% K-duplication ceiling

Advisor ruling 2026-08-20c implemented exactly: global refinement-
service policy only; objective untouched (ticks); no blended scores,
no speed thresholds, no pruning; LaunchFrontier and every operator
frozen. User performance north star recorded: basictest <= 5 min wall
(goal sub-1-min); real maps 30+ ramps.

**The change (SERVICE ONLY):** within the existing face/position-
bucket deepen rotation, TWO lanes per bucket - the OBJECTIVE
representative (min g) and the RESOURCE representative (max exact
board-boundary energy E(B) = |v|^2 + 2 g gs z, via the one certified
`ExitField::EBoundary` formula). Least-served lane goes next,
deterministic tie to objective. Conceptually the advisory Pareto
service frontier in (g, -E). Lower-energy states keep their breadth/
position/diversity service - nothing is dominated, suppressed, or
"impossible". The legacy session-18 speed-parity deepening is
preserved behind `--svc18` as the A/B baseline arm.

**New instrumentation (measure now, build only if measured):**
per-face board-E median/max with g@maxE and per-bucket max-E; per-face
distinct exit capability (speed, z) by witness hash; service-lane
attribution (expansions/evals/level histograms per lane); per-face
first-witness snapshots (iteration/seconds/evals); and the advisor's
K-DUPLICATION measurement - queries grouped by the underlying physical
problem K = (exact air start state, target face, flight horizon)
across differing aim/region queries.

**Run 9 (resource lane, 3600s, launch frozen at the same 250 u/s
run-off):**

- Lane service: breadth 112 exps / 1.9M evals, objective 115 / 21.6M,
  resource 109 / 15.2M; the resource lane's level histogram skews low
  (40,31,19,19) - it keeps pulling FRESH high-E states into their
  first ladders, exactly its job.
- THE PARETO TAIL IS REAL AND SERVICED: g@maxE is late everywhere
  (f0: 621k@g170, f1: 561k@g351, f2: 268k@g500) - later-but-richer
  states now receive fine refinement; f2's max-E concentrates in the
  east buckets (b2 249k, b3 268k), matching the geometry.
- f2 EXIT CAPABILITY now reaches the bridge class: speed med/max
  184/606 u/s, z med/max 27/157. At 606 u/s from z 157 the ~300u gap
  costs ~100u of fall -> arrival inside f3's window. The capability
  the bridge needs EXISTS at the tail of the distribution.
- f3: STILL NEVER WITNESSED. The march is now compute-starved rather
  than misdirected: f1 first witnessed at 118s/0.84M evals, f2 only
  at 1265s/12.0M evals - two thirds of the hour went to reaching and
  strengthening f2; the fast f2 states' fine ladders toward f3 ran
  out of budget. 2173 states, 2177 edges, all cold-replay bitwise,
  0 topo violations, no incumbent.
- **THE K-DUPLICATION MEASUREMENT: 84% of all entrance evals are
  repeat-share.** 38.7M evals across 22,695 aim/region queries hit
  only 5,398 distinct physical problems (avg 4.2 queries/K, max 25).
  The batched-witness-field amortization ceiling is ~6x - the
  measured green light for the post-G0R entrance-throughput project,
  and the difference between the current hour-scale runs and the
  user's 5-minute budget.

**Run 10 (--svc18 frozen baseline, 3600s, identical binary and
instrumentation):** 369 expansions, 47.2M evals, 2150 states
(f2: 987), f3 never. Baseline reached f2 FASTER (690s / 8.3M evals vs
1265s / 12.0M) and made more f2 nodes - pure march speed - but its
capability tail is WEAKER everywhere the bridge cares: f2 exits
510 u/s max @ z 125 (vs 606 @ 157), f2 board E max 254k@g355 (vs
268k@g500), f1 median board E 192k (vs 238k). Its lane report shows
the structural difference plainly: objective 246 expansions / 45.1M
evals, resource 0/0.

**THE A/B (both arms 3600s, same binary, same frozen launch):**

    metric                     svc18 baseline   resource lane
    f2 exit speed max          510 u/s          606 u/s   (+19%)
    f2 exit z max              125              157       (+26%)
    f2 board E max (g@)        254k @ g355      268k @ g500
    f1 board E median          192k             238k      (+24%)
    f2 board E median          110k             118k      (+7%)
    f2 first witnessed         690s / 8.3M ev   1265s / 12.0M ev
    f2 nodes                   987              851
    f3 witnessed               never            never
    K-dup repeat share         85%              84%

**G0Q verdict - DIRECTIONALLY CONFIRMED, NOT COMPLETE:** the same
250 u/s launch does expose materially stronger late-face states when
they receive refinement - the bridge-class f2 exits (600+ u/s at
height, arriving inside f3's window by the ballistic arithmetic)
exist ONLY in the resource arm, and the f1/f2 populations
strengthened, not just the max. But the lane pays in march speed, and
neither arm's hour was enough to run the fast f2 states' fine ladders
against f3. f2 -> f3 remains unwitnessed; G0R remains open.

**The binding constraint is now measured to be THROUGHPUT, not
allocation:** the K-duplication repeat share is POLICY-INVARIANT
(84% vs 85%) - both arms spent the hour re-solving ~5,400-6,000
distinct physical problems 4.2-4.4x over. At the ~6x amortization
ceiling, the resource arm's hour becomes ~10 minutes - which both
fits the fast-f2 fine ladders that f3 needs and is the only measured
path toward the user's <= 5-minute budget (sub-1-minute goal). The
advisor's conditional launch-capability probe remains open as the
second lever if post-batching f2 states still fall short.

**Standing state:** groute 11/11 and the twelve other suites green
from the session binary; gmap runs consistent (all edges cold-replay
bitwise, 0 topo violations, no tape reachable); h_cert = 0; no
incumbent yet, competitive horizons still unexercised.

---

## THE SEARCH ARCHITECTURE CORRECTION (advisor 2026-08-20d — STANDING)

Two different things were gradually conflated and are now formally
separated:

    the DEFINITION of the exact optimum        (strong; unchanged)
    the ALGORITHM used to search for it        (under reconstruction)

**The correction in one line: the exact simulator is CHEAP; the
current search FORMULATION is expensive.** The recoded engine exists
precisely to support enormous exact search rates. Production must
never respond to the throughput problem by approximating the
simulator — search RESOLUTION is adaptive; simulator ACCURACY never
changes.

**The category error being corrected:** `RefSolve` is an INVERSE
boundary solver — built and certified (AirRec era) to answer "given
an arbitrary known-reachable boundary condition (Q, T, theta), can
the machinery recover it cold?" That is a solver CAPABILITY test. It
was then promoted into the ordinary production mechanism for
populating the board field: every acceptance ball, heading interval,
resolution rung, and profile launches its own guided-shooting
optimization run, each flying hundreds-to-thousands of exact
trajectories. One physical (air start, face) becomes many independent
solves (sessions 18-19 measured 84-85% of all entrance evals as
repeat-share across such queries — 4.2-4.4 queries per distinct
physical problem). The heatmap became thousands of search problems;
it was supposed to be the OUTPUT of one search.

**The intended computational shape (restored as the standing
production architecture):**

    billions of cheap deterministic evaluations   (arithmetic: the
        cancelled terms - deterministic vertical, certified reach
        cones, turn/gain laws, board-loss bounds)
      -> millions of surviving candidate evolutions (exact cheap
        state transitions through the REAL simulator)
      -> thousands of retained representative states (adaptive
        binning/compression of the reachable set)
      -> hundreds of exact witnesses deposited into the field

    The heatmap H(Q) is an output of forward reachable-state
    enumeration. Field values remain ACTUAL replayed trajectories -
    the old field's failure mode (combining independently-optimistic
    quantities into a nonexistent trajectory) stays dead. L(Q) comes
    from witnesses; U(Q) is cheap and certified; most of a map may
    keep a wide (L, U) gap forever; only competitive cells earn
    refinement.

**The state variables of the free-air search** (why it is compact):
(x, y, vx, vy, carried side d, dwell age a<=6) plus tick t. z(t) and
vz(t) are DETERMINISTIC per tick (no duck/jump input) — every
frontier state at tick t shares them exactly. The dwell-6 reversal
law is explicit finite structure (runs of held side). The certified
strafe law gives the exact one-tick speed/turn response. Per-tick
horizontal |dv| <= air_speed_cap = 30 exactly (the accel law), so
reach cones are certified.

**RefSolve's retained roles (it is NOT deleted):** falsifying forward
coverage (the recoverability oracle — cells it wins reveal which
state/control dimension the sweep under-resolves); locally polishing
a promising frontier region; constructing better witnesses inside an
identified basin; regression gates. It stops being the per-cell field
engine.

**The core open research problem (named, not yet solved): the state
equivalence/partition rule.** Forward-enumerating every continuous
cosa schedule exactly is infinite; the audit's v0 binning (xy cell x
heading sector x speed bucket x side x dwell, 2 representatives)
already collapses millions of strikes into ~1000 cells but
hemorrhages coverage at a hard frontier cap. The exhaustive search
target is adaptive branch-and-bound over reachable-state cells, each
carrying exact witnessed representatives + a certified optimistic
envelope + honest unresolved status — refined when it could matter,
eliminated only by certified bounds.

---

### Session 20 (2026-08-20) — the search-architecture audit:
### `forwardfield`, the funnel, and the honest head-to-head

User direction: real maps (30+ ramps, 60+ s) must solve in <10 min
(~30 min exhaustive); basictest must trend toward seconds-to-<1-min.
Confidence in the architecture was withdrawn pending an audit.
Advisor: pause batching, launch work, service tuning, and G0R runs;
expose what one Entrance search actually does; the simulator is never
approximated.

**Changes landed this session:**

1. `g_movetick_count` — a process-wide count of authoritative ticks
   inside `MoveTick`. OBSERVABILITY ONLY: no physics state, no
   control flow. No-drift proof: groute 11/11 and the airrec fixture
   hash de1b000e431a84fb VERIFIED bit-identical after the change.
2. `forwardfield` (SolverLab command) — the audit instrument. Builds
   ONE real Entrance problem deterministically (anchor -> yaw 0/15
   run -> entrance to f0 -> board -> exit query -> exit selected by
   the composite window+cone score) and runs both arms on it:
   - ARM 1 `ForwardSweep`: v0 forward reachable-state enumeration.
     Per tick, actions = {coast, hold/flip side x 4 cosa samples},
     flips gated by dwell-6; states bin by (xy cell at r, 48 heading
     sectors, 25 u/s speed buckets, side, capped age), keeping the 2
     fastest exact representatives per bin; compact lineage arena =
     witnesses; every target-face contact deposits
     (Q, E_boundary, lineage) into the field. Board resolutions
     32/16/8/4/2u.
   - THE CANCELLED TERMS operate as arithmetic inside the sweep: the
     DETERMINISTIC VERTICAL WINDOW (all states share z(t), vz(t)
     exactly; once below the face floor with vz < 0 the entire sweep
     stops) and the CERTIFIED REACH CONE (per-tick air |dv| <= 30
     exactly => coast point + 0.225k^2 disc; a state whose cone
     misses the face box at every remaining window tick is eliminated
     — a certified prune).
   - ARM 2: the production inverse path — per-aim (ballistic /
     centroid / low-region) RefSolve ladder exactly as production
     runs it (600/1800/3600 as independent actions), every strike
     collected via on_strike and binned into the same grids.

**Audit run 1 (the selection lesson):** the fixture's fast/flat/high
prior selected a south-diving exit whose vertical window closes in ~3
ticks; the v0 sweep (no window rule yet) burned 48.7-75.5M MoveTicks
/ 28-52 s per resolution marching a dead domain, and RefSolve burned
its full 12,000-eval ladder for zero strikes. Both arms failed
identically — the FIRST measured demonstration that the cancelled
terms must run BEFORE any simulation, and that priors are not
reachability.

**Audit run 2 (window selection only):** the window rule collapsed
the dead sweep to instant (~104k ticks, 60 ms) but window-only
selection picked a westward exit (window 12 ticks, cone never touches
f1) — zero strikes again. The composite window+cone score fixed
selection: score 72, the yaw-15 root, 601 u/s — the REAL f0->f1
transfer state (run 4's witness lineage).

**Audit run 3 (the head-to-head, real problem):**

    FORWARD SWEEP v0 (per resolution):
      r | moveticks | maxfront |   overflow | strikes | cells | bestE
     32 |     57.9M |  200,000 |    115,815 |   1.96M |    66 |  581k
     16 |     63.7M |  200,000 |    924,268 |   3.12M |   162 |  581k
      8 |     59.1M |  200,000 |  3,002,442 |   4.84M |   330 |  581k
      4 |     52.3M |  200,000 |  5,927,828 |   6.23M |   704 |  581k
      2 |     49.4M |  200,000 | 10,216,995 |   5.63M |  1105 |  581k
      (~40-48 s per sweep, single-threaded)

    REFSOLVE production ladder (3 aims x 600/1800/3600):
      18,000 evals | 1.14M MoveTicks | 0.77 s | 12,183 strikes
      cells at 32/16/8/4/2: 64 / 152 / 331 / 691 / 1246

    HEAD-TO-HEAD at r = 8: forward-only 250 cells | shared 80 |
    refsolve-only 251 | RefSolve beats forward by >1k E in 27 shared
    cells (worst gap 302,761).

**The honest verdicts:**

- THE SCALING CRITERION (advisor's pass/fail): the forward
  representation PASSES — exact work stays flat (58M -> 49M ticks)
  while covered cells grow 66 -> 1105 from ONE sweep, rebinned. The
  production inverse path FAILS it — its strikes are fixed per solve;
  finer coverage requires new per-region solves (the 84-85% dup is
  that failure measured at global scale).
- v0 FORWARD DOES NOT YET BEAT REFSOLVE per cell: on this EASY target
  (ballistic aim valid, strikes plentiful) RefSolve is ~50x more
  MoveTick-efficient per covered cell (3.4k vs 179k MT/cell) and
  finds far higher-E arrivals in 27 shared cells. RefSolve is a good
  LOCAL optimizer; the production pathology is the invocation pattern
  (per-region x per-profile x duplication on hard/empty targets),
  not the solver.
- THE STATE-COMPRESSION PROBLEM is now measured, not hypothesized:
  6.2M strikes collapse to ~1100 cells; the 200k frontier cap
  overflowed by up to 10.2M states (reported, never silent). The v0
  bin rule (2 fastest per bin) keeps redundancy and drops coverage
  arbitrarily at the cap. This is the advisor's named core research
  problem for the reconstruction.
- APPLES-TO-APPLES THROUGHPUT: the exact simulator executes ~1.35-1.5M
  MoveTicks/sec single-threaded (one RefSolve "flight eval" ~ 63
  exact ticks on this problem; session-18's "12k evals/sec" is
  therefore ~0.8-2M MoveTicks/sec — the simulator was never the
  bottleneck). The old "billions of options/minute" were ARITHMETIC
  candidates; the funnel's arithmetic layer (window + cone tests)
  already evaluates at that scale inline. CPU: single-threaded
  throughout; parallelism untouched headroom.

**What this session deliberately did NOT do:** no batching build, no
launch probe, no service tuning, no G0R march, no approximate
simulator, no operator/physics changes, no global-layer changes. The
resource lane and all session-19 state stand.

**Milestones:** A0 (the audit) DELIVERED. G0Q stands directionally
passed. G0R deferred behind the search reconstruction. NEXT (advisor
gate): the state equivalence/partition rule for the forward
representation — make the sweep's per-cell efficiency competitive
with RefSolve on easy targets while keeping its one-sweep-many-cells
scaling — then rebuild Entrance production on the forward field with
RefSolve as oracle/polisher, then return to G0R.

Board: thirteen suites green from the session binary (groute 11/11;
airrec fixture hash bit-identical across the MoveTick-counter
change).

---

### Session 21 (2026-08-20) — the compression benchmark: the derived
### lattice works at coarse h, explodes at fine h, and the oracle
### names the next law

Advisor ruling 2026-08-20e implemented: three resolutions separated
(board observation ~2000/1e6 u^2 ~ 22u; internal state cells DERIVED
from output resolution via dv ~ h/tau; local refinement deferred);
world-first-contact sweeps (per-face fields are views); (T, lambda)
destination cells with half-pitch edge bands; multi-representative
state cells with optimistic envelopes and honest compressed counts;
NO frontier cap - an abort ceiling that reports explosion as a
finding; RefSolve as the adversarial oracle with the first-lost-state
diagnostic. Paused throughout: G0R, batching, launch, service,
simulator approximation.

**The rebuilt `forwardfield`:** scheme A = session-20 arbitrary bins
(200k cap, continuity); scheme B = derived lattice, 1 representative
(max speed); scheme C = derived lattice, 4 representatives (max
speed / min heading / max heading / first). All schemes deposit into
the same (face, T, lambda) board; benchmark at h in
{32,16,8,4,2} (A at {32,8,2}).

**THE BENCHMARK (world-first-contact, one S0, the real f0->f1
problem, window+cone score 72, horizon 103, faces prepped
f0..f3 with windows to tick 68-76):**

    sch |  h | moveticks | retained | board | f1cells | wall  | note
      A | 32 |     38.4M |    6.16M |   550 |     521 | 31.8s |
      A |  8 |     34.9M |    5.64M |   879 |     879 | 30.3s |
      A |  2 |     32.8M |    5.30M |  1305 |    1305 | 31.1s |
      B | 32 |      2.97M|     474k |   614 |     471 |  2.2s |
      B | 16 |     22.3M |    3.55M |  1231 |     916 | 18.1s |
      B |  8 |    198.4M |   31.7M  |  2425 |    1763 |211.8s |
      B |  4 |         - |        - |     - |       - |     - | ABORT
      B |  2 |         - |        - |     - |       - |     - | ABORT
      C | 32 |      9.18M|    1.47M |   661 |     489 |  6.1s |
      C | 16 |     64.9M |    10.4M |  1269 |     932 | 47.3s |
      C |  8 |         - |        - |     - |       - |     - | ABORT
      C |  4 |         - |        - |     - |       - |     - | ABORT
      C |  2 |         - |        - |     - |       - |     - | ABORT

**Finding 1 — THE DERIVED LATTICE WORKS AT THE INTENDED DENSITY.**
B@32: 2.97M ticks / 474k retained / 614 cells / 2.2 s - versus
scheme A@32's 38.4M / 6.16M / 550 / 31.8 s. Thirteen-fold
compression AND more coverage, within sight of the advisor's target
scale (tens-to-hundreds of thousands retained). C@32 buys ~8% more
cells for 3x cost via the extra representatives. The sensitivity
rule (position at h; velocity at h/tau) is doing exactly what it was
derived to do at the ~22-32u observation density.

**Finding 2 — THE LATTICE EXPLODES AT FINE h; THE THIRD RESOLUTION
IS NOW MEASURED NECESSARY.** dv = h/tau makes the velocity lattice
finer as h shrinks; the frontier crosses the 1.5M honest abort
ceiling at h <= 8 (scheme C) / h <= 4 (scheme B); B@8 completed only
by brute force (198M ticks, 212 s). The law this measures: fine
OUTPUT resolution cannot be bought with uniformly fine INTERNAL
resolution - h-refinement must be LOCAL to competitive regions (the
third resolution), never global. The abort ceiling did its job:
explosion is a reported finding, not a silent truncation.

**Finding 3 — THE ORACLE NAMES THE NEXT LAW: ACTION SAMPLING AND
STATE LATTICE MUST BE CO-DESIGNED.** The first-lost-state diagnostic
fired at TICK 0 for every valuable missing witness, cone clean:
RefSolve's continuous cosa diverges from all nine sampled actions
within one tick by more than the lattice's dv. The 4-sample cosa set
{0.9995, 0.98, 0.9, 0.75} cannot fill a lattice whose cells are
h/tau ~ 5-8 u/s wide. Either the action set densifies with the
lattice, or cell membership must absorb sub-cell control differences
(rounding into cells rather than exact generation). This is the
first concrete, measured design constraint on the compression rule -
delivered by the oracle exactly as designed, on its first run.

**Also measured:** the (T, lambda) parameterization works (the
destination is 1D per arrival tick; edge cells at half pitch); the
world-first-contact sweep populates multiple faces from one S0
(f0 re-entries + the f1 target from the same frontier - though this
S0's cone confines most output to f1); zero cone-flagged witnesses
(the certified reach cone never wrongly pruned an oracle path).

**The final run (oracle recovery + diagnostics at h=32):**

    oracle recovery (oracle cells at that h / recovered / missed /
    ref-wins>1k):
      A@32: 279/191/88/93    B@32: 279/186/93/102   C@32: 279/189/90/93
      A@8:  561/168/393/87   B@16: 399/261/138/114  C@16: 399/261/138/108
      A@2: 1032/131/901/120  B@8:  561/352/209/122  (finer rows aborted)

    HEAD-TO-HEAD f1 at h=32: fwd-only 300 | shared 189 | ref-only 90
    | ref wins-by->1k 93.

- PER-CELL EFFICIENCY CLOSED FROM 50x TO ~1.5x: B@32 spends 6.3k
  MoveTicks per covered cell vs the oracle ladder's 4.1k - while
  covering 1.7x more cells (471 vs 279) from ONE sweep and retaining
  13x fewer states than the v0 bins. C@32: 18.8k MT/cell for +4%
  cells - the 4-rep basis is not yet paying for itself.
- REFSOLVE'S QUALITY EDGE PERSISTS at every resolution (~90-120
  cells won by >1k E): the representative basis loses outcome
  extremes even where cells match - the second co-design input.
- FIRST-LOST, UNANIMOUS: 12/12 valuable misses lost at TICK 0 with
  the parent cell PRESENT and the cone clean => the four-sample cosa
  action set cannot produce the witness's velocity cell even at
  h=32. THE CO-DESIGN LAW (the session's central finding): the
  action sampling density and the state-lattice pitch are one design
  variable, not two - either actions densify with dv = h/tau, or
  cell membership must absorb sub-cell control differences.

Board: thirteen suites green (no engine/operator changes this
session; the audit instrument only).

**Sequel note (same day): the program pivoted** - see the
CAPABILITY LIBRARY program (user + advisor 2026-08-20f,
`Docs/CapabilityLibrary.md`): bottom-up min-max capability functions
with unit proofs; the global solver becomes a regression consumer;
G0R is no longer the next milestone. The session-21 findings carry
directly: the co-design law and the representative-basis question
become CAPABILITY questions (the N-tick air turn/gain function
subsumes "which cosa samples suffice"), answered once, reused
everywhere.
