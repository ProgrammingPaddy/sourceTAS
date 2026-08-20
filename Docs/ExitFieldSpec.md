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
