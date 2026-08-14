# Full Map Solver — Lab Log

**Read this file FIRST before any solver-era work.** This is the source of truth for the
full-map-solver effort: mission rules, candidate designs, decisions, benchmarks, experiments,
and dead ends. Append entries (dated); never rewrite history. Dead ends stay recorded so they
are never retried without new evidence. Keeping this current is cheaper than re-deriving
anything after a context compaction.

---

## Mission (kickoff 2026-08-12)

Solve a surf map hands-off: from the start platform to the end platform/zone, minimizing
**total ticks**. Feasibility first: *"If we can find tons of routes, culling bad ones is easy.
If we can't find any routes, getting a good one is impossible."*

Hard rules (user-set):
1. **Results define conditions.** No prescriptive value rules; slow or non-finishing behavior
   is culled by selection, not by hand-coded penalties. Prescriptive bans only for
   truly-universal cases (startzone speed-farming is the canonical example).
2. **Strafe direction changes ≤ 5/sec** (no ugly 1-tick strafing). At 66.67 tps that is ≥ 14
   ticks between L/R flips. Enforce STRUCTURALLY in the control encoding, never by
   post-filtering. Revisit only with evidence it blocks real routes.
3. **Wall clock ≤ 10 min per map** — sooner is better. Compute budget is a first-class design
   input, not an afterthought.
4. **Adaptable to every map.** Zero per-map hand configuration beyond declaring start/end.
5. **Engine-verified output only.** A route "exists" when it compiles through
   Prediction::RequestSim and passes the divergence check — solver-model claims don't count.
6. Never break existing features; the solver is additive on top of v1.1 (tag `v1.1`, commit
   `ba9461c`).

---

## Assets already in the tree (reuse, don't rebuild)

Names below are from the segment-solver era (see memory `solver-roadmap-1-1`); verify exact
signatures in code before building on them.

- **Engine-exact sim**: `Prediction::RequestSim` — closed-loop provider through the real
  movement pipeline. Main-thread, frame-hitching → ground truth + final compile, NOT search
  volume. Test-play divergence machinery already exists.
- **Fast offline model (the crown jewel)**: `EngineMoveTick` + `TraceHullWorld` — AirMove /
  friction / jump / gravity plus brush collision replicating the engine's exact behaviors
  (CategorizePosition 2u ground probe + origin snap, unclamped-enterfrac started-touching
  tie-break, exact axial bevels from the brushside lump, Minkowski hull expansion from live
  `CCollisionProperty` mins/maxs). Validated to dpos = 0.000 across months of segment-solver
  traces. Known residual: knife-edge apex cases are decided by real-replay arbitration, never
  by geometric gates (v5.6 lesson — do not re-attempt fragility gates).
- **BSP world**: full .bsp file parse; worldspawn-only collision (model-0 tree walk; entity
  brushes flagged non-collision); triggers parsed (teleport / push / gravity) with SDK-exact
  Touch semantics already wired into both sim loops; face polygons/planes; surf-face tags;
  board targets; per-map `.geo` persistence.
- **Threaded worker pool**: `hardware_concurrency − 2` (cap 12), thread_local per-worker
  state, SEH per job, results under one mutex. Proven on segment searches.
- **Control primitives (provider vocabulary)**: optimal strafe (max-gain), steering L/R rate,
  prestrafe, ride, coast, jump/duck, boundary smoothing, `MaxAirGain` closed form.
- **7 human recordings** (.tas v3 with map names) — validation corpus + search seeds.
- **Known gaps**: colliders were corridor-scoped (cap 256) → need a whole-map spatial index;
  displacements NOT collided (dispinfo skipped at parse); func_-entity solids not collided;
  disconnected stage starts unsupported.

---

## Compute reality (hypotheses — MEASURE in Phase 0, record under Benchmarks)

- 66.67 tps (0.015 s interval — confirm from gpGlobals). A 2-minute route ≈ 8000 ticks.
- Engine sim is main-thread only → verify/compile budget, roughly "a handful of full routes
  per minute". All search volume must live on the fast model.
- Fast model with a whole-map spatial index: target ≥ 0.5–2 M ticks/s/thread. 10 workers ×
  5 min ≈ 1.5–6 B ticks ≈ 200k–750k full-route equivalents — and archive-style search mostly
  simulates SHORT continuations from snapshots, so effective coverage is higher still.

---

## Candidate approaches (written 2026-08-12, BEFORE seeing the user's offline findings)

### A. Knot control parameterization (foundation for everything below)
Controls are not per-tick inputs. A route is an ordered list of **knots**:
`{duration ≥ 14 ticks, steer: L / R / straight, mode: optimal-strafe toward a drifting
heading OR fixed steering rate, buttons (jump; duck deferred)}`. Within a knot, yaw comes
from the existing closed-loop provider math (optimal strafe / steering) — smooth by
construction, so 1-tick jitter is impossible and the ≤5 flips/sec rule is structural (flips
only at knot boundaries). Collapses the search space from 8000 continuous yaw decisions to
~100–400 mostly-inert genes, and the output maps 1:1 onto existing editor segment kinds
(the solved route lands as an editable .tasproj).

### B. Archive-driven exploration — RECOMMENDED route finder (Go-Explore family)
- **Cell** = quantized position (start 64–128 u) × coarse speed bucket (± maybe velocity
  octant). Archive keeps the best entry per cell (earliest tick; speed tiebreak) + a full
  fast-model **state snapshot** + the input tape that reached it.
- **Loop**: sample a cell (weight = frontier-ness: low visit count × progress potential),
  restore its snapshot (O(1) struct copy — deterministic model; THE reason search can't run
  in-engine), roll 1–4 randomized knots (headings biased toward the distance field's
  downhill and toward keeping speed), commit new/improved cells. Fully parallel across
  workers.
- **Automatic progress metric, zero per-map config**: voxelize free space from the BSP, BFS
  a **distance field from the end volume**, flowing through teleport triggers (source →
  destination edges). Used ONLY to bias sampling — never to cull. Any reached cell is kept:
  results define conditions.
- **Finish** = a cell inside the end volume → extract the tape. Keep exploring for MANY
  finishes; distinctness = edit distance over coarse cell/face sequences ("tons of routes").
- **Seeding**: replay the human recordings through the fast model to pre-populate the
  archive along known routes (instant finisher-to-beat on mapped maps). Works from scratch
  without them.
- Startzone farming: if the clock runs from anchor tick 0, zone looping costs ticks and
  self-culls — no rule needed. Only a timer-style "clock starts at zone exit" convention
  needs an explicit prespeed rule. (Open question 1.)

### C. Evolutionary refinement on knots — RECOMMENDED optimizer
Population = diverse finishers from B. **Objective = finish tick, nothing else.** Mutations:
knot heading/duration jitter, split/merge, boundary nudges; crossover = splice two routes at
a shared archive cell; greedy time-trim passes (shorten each knot, keep iff it still
finishes). Optionally CMA-ES over a fixed skeleton's continuous genes between structural
rounds. A mutant that doesn't finish is dead — no shaped fitness, no style scoring.
Re-boarding a ramp survives only if it is actually faster (user's philosophy, self-culling).

### D. Geometric ramp-graph macro planner — optional bias/seed layer, NOT primary
Extract surfable faces (slope band + area + existing tags), edges = reachability validated
by sampled fast-model rollouts (with speed windows), K-shortest-paths → macro routes.
Interpretable and fast on linear maps, but risks prescribing away unmodeled routes (skips,
booster tricks, teleport abuse) and mostly duplicates what B + the distance field achieve
with fewer assumptions. Build only if B demonstrably needs the help.

### E. Kinodynamic RRT / SST — fallback alternative to B
Tree in (pos, vel) with sampled knot controls. Needs a dynamics-aware distance metric
(position-only misleads badly at 800+ u/s). The archive method dominates it in this setting
(deterministic restore, cross-iteration reuse, cell dominance ≈ SST witness sets). Keep as
fallback if B stalls.

### F. Reinforcement learning — rejected for v1
Per-map training busts the 10-minute budget; a general cross-map policy is a GPU research
project with opaque failures; reward shaping fights results-define-conditions. Revisit only
as a learned sampling prior distilled from archive data.

### G. Formal optimal control (collocation / DDP) — rejected
Contact sequence is discrete and the dynamics non-smooth (ClipVelocity, ground snaps) →
mixed-integer intractability. C is the derivative-free practical cousin. Keep only the
phase-sequencing insight.

### Recommended pipeline (per map, inside 10 min)
- **Phase 0** (≤ ~30 s): whole-map collision index (grid/BVH over worldspawn brushes),
  free-space voxel distance field from the end volume (through teleports), throughput +
  fidelity benchmarks, audit of map features we don't collide (displacements etc.).
- **Phase 1** (~3–5 min): archive exploration (B) on the fast model, all workers.
- **Phase 2** (~2–4 min): evolutionary tick-minimization (C) on the top-K diverse finishers.
- **Phase 3** (~30 s): compile top routes through the engine (RequestSim), divergence check,
  LOCAL re-solve around any divergence (the segment-solver's correction pattern), emit
  .tasproj knots→segments so the result is hand-editable in the v1.1 editor.

---

## Risks (log findings against these IDs)

- **R1** Fast-model fidelity compounding over 8k-tick horizons. Mitigation: Phase 3 verify +
  local correction; the ROUTE (which ramps, what order) is robust even when exact ticks
  shift; Phase 0 benchmarks divergence against the human recordings.
- **R2** Collision gaps: displacements, func_-entity solids. Phase 0 must REPORT what a map
  contains that we don't simulate, per map, before any search is trusted.
- **R3** Archive cell resolution: too coarse misses tight lines, too fine wastes memory and
  stalls convergence. Measure before making it adaptive.
- **R4** Maps needing duck tunnels or precise jump chains — scope call for v1.
- **R5** Solver home: in-DLL background workers vs offline harness exe (decision pending).

---

## Open questions (2026-08-12 — ANSWERED 2026-08-13, see "Final contract" below)

1. **Timing + start legality**: does the clock run from the anchor (tick 0) or timer-style
   from start-zone exit? Former ⇒ startzone farming self-culls, no rule needed. Latter ⇒
   need the explicit rule (prespeed cap? no re-entry?). And what is the legal start state —
   the existing anchor captured standing on the platform? Is platform prestrafe/jump
   allowed?
2. **End condition**: user-picked end volume (crosshair-picked box, persisted in .geo), a
   named map trigger, or coordinates?
3. **Testbed maps**: 1–3 maps easy→hard, ideally ≥1 with existing recordings (validation +
   seeding).
4. **Solver home**: default proposal = engine-agnostic solver core compiled BOTH into the
   DLL (background workers, real workflow) AND a console harness exe (tune/batch offline
   without injecting — the torture.cpp pattern scaled up). Veto if DLL-only preferred.
5. **v1 input scope**: jump as a knot gene from day 1 (platform links, bhop sections); duck
   deferred unless a testbed map needs it?

---

## 2026-08-13 — Final contract + prior-attempt handoff review

### Final contract (user answers)

- **Timing / objective**: fitness = **ticks from anchor tick 0** to finish (user's stated
  ideal; makes startzone farming self-culling). Real-surf semantics — timer starts on the
  first tick the start platform is no longer beneath the player (platform footprint extended
  upward; in-zone prestrafe is free) — is a LATER refinement, possibly with an in-TAS
  startzone/endzone creator. v1 uses the player ORIGIN for all zone tests (user's
  simplification).
- **Startzone rule (the one universal ban)**: at most **ONE jump while the origin is over
  the start platform footprint**. Belt-and-braces under tick-0 timing.
- **Finish v1**: **grounded on the end platform's top face** (standing on red). Nearness,
  flyover, or airborne closest-approach is NOT success. (Real-surf endzone = plane above
  the platform crossed ⇒ timer stop; later, with the zone creator.)
- **Testbed**: `surf_glade\surf_basictest.bsp` (facts verified below). User will supply a
  manual recording on it (parity corpus + seed). **Generalization is a requirement** — no
  map-specific logic; diagnostic route families are probes, never solver logic. ML is
  acceptable only if it learns a new map quickly (no slow per-map training).
- **Solver home**: standalone-first, per user ("DLL only would be a problem"). Solver core =
  engine-independent C++ lib built from OUR BspWorld parse + OUR validated fast model,
  driven by a console harness (SolverLab.exe) for search speed and iteration; the DLL gets
  run control + full in-game visualization of solver artifacts + final engine verify.
- **Duck**: deferred (limited use cases; user delegated judgement).
- **Direction-change rate**: a PARAMETER, default 5/s to start (the prior attempt's 4/s was
  explicitly only an example). Min knot duration = ceil(66.67/5) = **14 ticks** ⇒ any
  rolling window satisfies the cap by construction, including across prefix/suffix
  boundaries (the prior attempt's "prefix boundary ignored" debt cannot occur).

### Handoff review (C# lab, Aug 5–12 — "take with a grain of salt", verified selectively)

Credibility frame: every number in the handoff is exact-to-its-own C# oracle, never
validated against the native predictor (its #1 admitted debt — a debt WE structurally don't
have, since our fast model is the one already validated against the engine). Its failure to
find a legal route is evidence about **search organization**, not map feasibility: the
user's manual 1-3-4 run finishes this map legally, and every impressive terminal number in
the handoff was achieved with ILLEGAL per-tick A/D switching (their ControlGrammar validator
existed but was disconnected from every active sampler). **The legal knot space was never
actually searched end-to-end.** Their only finish (15.810 s) farmed 16 start-platform hops
and is rejected + slower than the user's manual run.

**ACCEPT** (their controlled evidence + consistent with our segment-solver experience):
1. Event/contact-boundary organization beats monolithic per-tick beams at equal budget
   (their strongest ablation: hybrid macro portfolio hit all 4 ramps in ~2M transitions;
   monolithic beams hit ZERO ramps).
2. Arrival DIVERSITY at contact boundaries is load-bearing; greedy/narrow handoffs poison
   everything downstream. (This is the archive/Go-Explore thesis restated.)
3. Max-work A/D arcs as the default proposal family; loss-taking controls only event-local
   (loss-everywhere = ~15× transition blowup AND worse results).
4. Finite-face target enumeration + small segment BVP solves was their most productive
   local generator — and it is essentially OUR existing LM/RouteSolution machinery. Reuse
   ours.
5. Targets must carry full state (position + velocity + contact mode); point-only targets
   produce false optimism.
6. Funnel visualization is where diagnosis power comes from; they added it too late. Build
   it first-class here.
7. Legality must be structural in the genome, never a side validator samplers can bypass.

**REJECT / DEMOTE**:
1. Energy as a frontier score or hard prune (two documented false-prune postmortems; high
   energy with wrong direction/contact is worthless). Demote to diagnostics (board loss in
   u²/s² in the funnel viz); optimistic energy route-graph shelved.
2. Sparse intermediate reverse-reachability joins (cost ≫ benefit in their runs; forward
   result got WORSE in one config). Shelve ALL backward machinery unless the explorer shows
   their terminal-stall signature (reaches last surface repeatedly, never finishes).
3. Their C# oracle and its archives/traces as any form of ground truth — not transferable.
   Map facts + organization lessons only.
4. Stage-wise diagnostic route restrictions (1-3-4 etc.) as solver logic — probes only.

**RE-VERIFY with OUR model before reliance**:
- Ramp-4 terminal envelope (~875 u/s uphill tangent at low board, ~500 u/s near lip) —
  C#-derived, re-measure if it starts driving decisions.
- Their C# throughput 4,817,961 ticks/s single-thread = the FLOOR our C++ core should beat.
- Board-loss magnitudes (u²/s²) — recompute with our clip math.

Key compute datum: their ENTIRE largest failed campaign was ~95M transitions / 172 s. At
our worker-pool scale that is seconds. **The bottleneck is search organization, not
throughput** — but throughput discipline still matters for generalizing to real maps.

### Verified map facts (surf_basictest.bsp, parsed independently 2026-08-13)

Handoff Section 3.3 CONFIRMED in full; extras noted. Parser: scratchpad `parse_basictest.py`.

- VBSP v20, 108 planes, 12 brushes, 76 sides, **models=1 ⇒ ALL brushes are world collision**;
  2 entities (worldspawn + info_player_terrorist) ⇒ **no triggers at all** (no teleports/
  boosters/zones on this map).
- Spawn: (-1578.3, -431.025, 449), angles 0 0 0 (on green, facing +x).
- Enclosure: floor z=-1088..-1056, ceiling z=1248..1280, 4 walls — the map bottom is a
  landable fail surface the search will discover; that's fine (dead-end cells).
- **Green start = brush 6**: x=-2176..-1280, y=-2048..1216, top **z=448** (CABLE/GREEN).
- **Red finish = brush 10**: x=1216..2112, y=-2336..928, top **z=256** (CABLE/RED).
- Ramps (all 51.3° — nz=0.624695; one non-axial side each + 1 bevel side):
  - Ramp 1 = brush 7: x=-1152..-704, y=-448..-192, n=(0,-0.780869,0.624695) d=349.829
    (face rises from z≈0 at y=-448 to z≈320 at y=-192).
  - Ramp 2 = brush 8: x=-576..-128, same n, d=309.849 (z≈-64..256).
  - Ramp 3 = brush 9: x=-128..320, y=-768..-512, n=(0,+0.780869,0.624695) d=-439.785
    (opposite-facing, z≈-64 at y=-512 up to z≈256 at y=-768).
  - Ramp 4 = brush 11: x=640..896, y=-832..64, n=(-0.780869,0,0.624695) d=-539.737
    (rises toward +x; lip z≈256 at x=896).
- Terminal geometry: ramp-4 lip (z≈256 @ x=896) is EXACTLY level with the red top (z=256,
  x≥1216) ⇒ a 320 u horizontal gap to clear without net height loss. Explains the prior
  attempt's terminal wall.
- User's known legal manual route: 1-3-4 (skips ramp 2), no prehop.

### Revised design v2 (deltas from candidates A–C above)

1. **Cells gain CONTACT MODE**: key = (quantized position, speed bucket, contact: air |
   surface-brush | ground-brush). Continuation unit = roll knots until the contact event
   changes OR a horizon expires; boundary states archived with full state + tape +
   snapshot. (Marries their event-boundary win to the archive.)
2. **Proposal mix per continuation**: max-gain arc L/R (dominant), straight hold,
   face-target BVP proposals (enumerate points on plausible next faces, solve a legal
   3-phase knot schedule with our LM machinery), rare event-local loss arcs. Proposals only
   bias — results decide survival.
3. Energy = diagnostics only. Distance field stays the exploration bias (bias ≠ cull).
4. Backward reachability shelved (revive only on terminal-stall signature).
5. Startzone 1-jump rule + tick-0 timing; finish = grounded-on-end-platform predicate.
6. All physics inputs in an explicit `MoveParams` (gravity/accel/airaccel/maxspeed/
   tickinterval/hull) — fed from live cvars in-game, from config offline. Testbed server
   settings: sv_airaccelerate 150, sv_accelerate 10 (+ surf setup's sv_enablebunnyhopping 1).

### Visualization (first-class; prior attempts failed here per user)

In-game is the PRIMARY viewer — we have the real world, the overlay budget system, freecam,
and the whole editor:
- **Solver tab**: run control (params incl. changes/sec, budget), live counters (cells,
  boundary states, finishes, best ticks, ticks/s), sortable route list (ticks, max speed,
  route signature) → selecting a route draws it + "Load as segments" lands it as a
  .tasproj-style run in the editor (cursor scrub, test play, divergence — all existing
  tooling applies).
- **World draws** (budgeted/decimated like the BSP wireframe): explored-cell cloud colored
  by earliest-tick/speed; top-K route polylines; **board funnels** — board points rendered
  ON the ramp face in situ, colored by post-board speed / board loss (their 2D scatter, in
  3D); optional distance-field arrows near the cursor.
- **Harness emits artifact files** (routes, archive snapshot, funnels); the DLL loads and
  renders them identically ⇒ offline runs are fully inspectable in-game. This satisfies
  "control and visualize in-game" with a standalone-speed search.

### Phase 0 build list (next implementation work)

1. Extract solver core (`Source/Solver/`, engine-independent): BSP parse + fast model
   behind `MoveParams`; audit and remove live-engine reads; console harness `SolverLab.exe`.
2. Whole-map collision index (uniform grid over brush AABBs) replacing corridor scoping;
   regression: identical traces vs the corridor path on segment-solver cases.
3. Parity benchmark: replay the user's surf_basictest recording through the core; compare
   against in-game engine states; record drift here. Throughput benchmark (target ≥ 5M
   ticks/s/thread — beat the C# floor).
4. Zone semantics + finish predicate + 1-jump rule + knot grammar (min-duration constant
   from the changes/sec parameter).
5. Then Phase 1 explorer per revised design v2.

---

## Phase 1 build log (2026-08-13, same session as the divergence hunt)

### Server-settings adaptation (user directive: nothing hardcoded)
- DLL `ExportServerParams()` (TasEditor.cpp): every Map Solve states export now also
  writes `Documents\sourceTAS\solver\server_params.cfg` from LIVE values — Cvars::GetFloat
  for sv_gravity / sv_airaccelerate / sv_accelerate / sv_friction / sv_stopspeed /
  sv_maxvelocity / sv_enablebunnyhopping, measured tick interval (LastDiag), live hull
  (PlayerHull), strafe-model wishspeed as maxspeed. Every line labeled `live` vs
  `default (not readable)`.
- Harness `SolverParams.{h,cpp}`: all commands auto-load the canonical file (or
  `--params <file>`), precedence defaults < file < CLI flags, applied keys printed loudly.
  Engine-behavior constants (duck shift, stamina, air cap...) are overridable file keys
  too — one visible place, zero recompiles. The max-gain controller derives its optimal
  wish angle from the live params EVERY tick (acos(max(cap−a,0)/speed)), so a cvar change
  changes the strafing itself, not just the sim.

### Explorer (Source/Solver/SolverExplore.{h,cpp} + `solve` command)
As designed in v2: knot genomes {dur ≥ min-knot, side L/R/coast, jump, duck, ground turn}
with flip legality carried ACROSS archive splice points (ticks-since-flip in every entry);
cell archive (pos × speed × contact kind/brush × ducked) with POD snapshot restore and
shared-prefix genome chains; 1-jump startzone rule enforced in rollouts (abort, never
scored); finish = grounded on the end brush (expanded footprint); `.tas` writer so solver
output loads in-game like any recording (WriteTas, STAS v3). Fixed rng seed 1337 =
reproducible runs; `--rng` varies.

### Iteration findings (each fix data-driven from the run logs)
1. Uniform tournament + per-tick recording: 2M entry cap hit at 10 s, 1.2M of the entries
   were tick-level replacement churn, ZERO finishes in 240M ticks. → Event-aligned
   recording (contacts/ground/duck/jump ticks, knot ends, every 4th tick), +4-tick
   replacement hysteresis, POD inline knots (no per-entry heap).
2. Straight-line distance-band pull alone: min-dist stalled at 1097, zero finishes — the
   pull drags restarts to SLOW junk cells near the finish (floor/walls) while the route
   needs speed built far away. → **Round-robin selection** (the prior attempt's validated
   lesson, now ours too): rotate dist-band pull / speed-band pull / uniform. Free-air
   cells get 2× coarser quantization (void bloat).
3. **SEEDED solve (basictest.tas as archive seed): 113 finishes in 9.0 s; best 430 ticks
   vs the seed's 432 — the solver beat the human run on its first working session.**
   Best route verified two ways: BuildFrames determinism check (re-rolled state matches
   archived snapshots) + independent `replay` of the written tape (finish tick 429 by
   replay's 0-indexed convention — counter conventions need unifying, cosmetic). The
   solver's landing is its OWN variant: duck 417 → UNDUCK 421 → lands (1203.4, 58.0) vs
   the human's (1213.7, 63.3). Written to recordings as `basictest_solved.tas` (in-game
   playable; engine verify = test play, expected near-exact after Phase 0 parity).
   Solve-loop throughput ~5.3M ticks/s single-thread (record/copy overhead vs the 15.26M
   raw bench).

### Phase 1.1 — UNSEEDED DISCOVERY ACHIEVED (2026-08-13, five data-driven iterations)

Each iteration was diagnosed from the previous run's printout (min-dist / max-spd /
closest-entry / per-brush census diagnostics added along the way — the census names which
pipeline stage is starved and was the decisive instrument):

1. Energy-band selection axis added to the round-robin (dist/speed/ENERGY/uniform).
   Energy = 0.5|v|²+gz separates live pipeline cells (high+fast) from fallen junk —
   allowed as ONE axis (the prior attempt only proved it fails as a sole score).
   Cap-freeze instead of cap-stop (budget no longer wasted).
2. Air-cell sub-cap (1.5M) + 3× coarser air cells + 2× coarser air speed buckets:
   junk falls were filling the archive before the contact pipeline matured (frontier
   froze at 20 s with min-dist still advancing). Contact cells keep full resolution —
   the event-boundary lesson again.
3. Finish-distance band walk restricted to CONTACT cells only: the measured closest
   entry was a dead vertical fall beside the platform soaking the pull.
4. Startzone z-fix: the zone is the platform footprint extended UP per the user's
   definition; floor bounces under it wrongly burned the jump budget.
5. **Near-miss breeding** (the dam-breaker): census showed the whole pipeline populated
   (ramps at 912–945 u/s, 23k ramp-4 touches, 50 touches on RED's west wall just below
   the lip — arrivals cresting with too much vertical, too little forward). New selection
   mode: back up 1–2 knot links from the closest approaches and resample the release.

**Result: pure discovery finishes surf_basictest with ZERO human input.**
- rng 1337: 404 finishes in 120 s (first at ~60 s), best 1264 ticks; verified by
  independent replay — legal, grounded on red, and **0 startzone jumps** (the solver
  invented a different opening than the human route entirely).
- rng 7: 1 finish in 120 s (~113 s) — works but HIGH VARIANCE across seeds. Headroom
  levers: worker-pool parallelism (many seeds racing/sharing an archive), and the Phase 2
  optimizer both compressing routes (1264 → competitive) and stabilizing discovery.
- Feasibility bar met: seeded = beats the human; unseeded = finds routes from nothing.

### Phase 1.2 — THE FRAGILE-SOLVE INCIDENT (2026-08-13, user-caught, fixed)

**User report:** the shipped unseeded tape (`surf_basictest_solved.tas`, the rng-7
1510-tick route auto-written by the default --out) diverges IN-GAME at ramp 1's spine,
falls to the enclosure floor, and appears to "finish" below the endzone. Suspected
false-positive finish.

**Findings (instrumented, not guessed):**
- Finish heights audited across the default seed's finishers: ALL at z 256.88–257.50,
  genuinely ON TOP of red IN THE CORE. Structurally, IsFinish can only fire on red's top
  plane (the only nz≥0.7 plane of brush 10). So NOT a below-map false positive —
  **core-legal but ENGINE-DIVERGENT**: the in-game playback separated from the core sim
  before the first ramp contact and everything after was garbage.
- New NEAR-EDGE detector (TryPlayerMove impact within 2u of a second plane of the same
  brush = geometry where the engine consults edge bevels the validated model omits):
  human run = 0 edge ticks; EVERY fragile finisher's chain contained edge ticks, first at
  ~tick 99 = the hull grazing the START PLATFORM's corner while walking off the lip (the
  whole fragile family launches by walk-off; the human jumps). The graze seeds the
  engine-vs-core split; the spine hit the user watched is downstream fallout.
- Taint-as-ABORT failed (0 finishes in 150 s): wall-kisses below the lip — the near-miss
  breeding stock — are edge-adjacent by nature; aborting sterilized them.
- Taint-as-INHERITED-FLAG + clean-only shipping still gave 1272/1272 fragile: the
  archive's best-per-cell rule was TAINT-BLIND — tainted first-arrivals permanently
  blocked clean lineage from recording (clean arrival 5+ ticks later = rejected).
- **Fix: class-aware cell replacement** — clean beats tainted regardless of tick;
  tainted never displaces clean; ±4-tick hysteresis within a class. Result: **9 CLEAN
  finishes (edge-ticks 0 audited over the whole chain), best 1887 ticks**, finish z
  256.88, 0 zone jumps, and a novel ending (three legal bhops building 274→344→403 into
  the gap jump onto red). Shipped as the replacement `surf_basictest_solved.tas`.

**Residual risk, stated:** "clean" = no TryPlayerMove edge contacts. The clean route
still LAUNCHES by walking off the lip (grounding simply ceases; the ground PROBE near
the expanded corner is exempt from tainting because apex-top grounding is engine-real
per v5.4). If the user's in-game test of the clean tape diverges at the launch, the next
lever is edge-checking CategorizePosition probes too (or the probe-region subset of
them). IN-GAME TEST PENDING = the actual acceptance gate.

**Rules earned:** (1) a solver claim is only as good as the model's coverage of the
geometry it exercised — audit edge exposure on everything shipped; (2) archive
acceptance rules must respect trust classes or fragile lineage monopolizes the frontier;
(3) auto-writing solver output into the user's recordings needs the same skepticism as
any shipped artifact (the bad tape reached the user because --out defaulted there).

### Phase 1.3 — THE AGREEMENT PROTOCOL (2026-08-13, after the user's second test)

**User's second in-game test:** still deviation; tick-0 ambiguity called out (they tried
the project start AND map spawn for the 1887 tape - neither matched its actual anchor,
which was the map spawn but reached only ramp 3 with the "end" on the floor). Verdict:
need (a) a tick-0 state both sides sign, (b) REAL engine tick capture in-game, (c) no
edge-contact theory accepted without engine data. All built this round:

**Map Solve tab v2 (QOL instruments — removable once proven):**
- **Capture solve anchor**: writes the current player state to
  `solver\solve_anchor.cfg` (+ live server params) — the agreed tick-0.
- **Teleport to anchor** (solve anchor) and **Teleport to '<selected run>' anchor**
  (any recording's own tick-0, for aligned playback of a specific tape).
- **Surf setup** button (same cvars as the Server tab).
- **Arm capture for next playback**: records every replayed tick's ACTUAL engine state
  (netvar basis, same as the divergence verdict; inputs stashed from each replayed cmd so
  rows carry buttons+yaw), auto-exports `solver\playback_states.csv` on playback end.
  Row −1 = the settled pre-frame-0 state → the diff's anchor check DIRECTLY verifies
  tick-0 agreement. The last frame's outcome row is deliberately omitted (post-playback
  physics would pollute it).
- Harness: `solve` anchor resolution = `--anchor file` > `--anchor-tas tape` > captured
  solve_anchor.cfg > seed anchor > map spawn (source always printed); seeding refuses a
  seed whose anchor differs from the solve anchor (>0.5 u) — foreign-anchor seeds are
  garbage.

**Shipped test tape**: clean unseeded route from the USER'S RECORDING ANCHOR (the anchor
already proven in-game by the working seeded tape): rng 11, **1312 ticks, 45 clean
finishes, finish z 256.03 landing at x 1224–1271 (platform interior, not the expanded
corner), 0 zone jumps, 0 edge ticks** → `recordings\surf_basictest_solved.tas`.
(rng 1337/3 from this anchor: 0 clean in 90–180 s — clean-lineage discovery is
rng-sensitive; parallelism remains the structural fix.)

**THE PROTOCOL (user's next session — settles everything with data):**
1. Map Solve → Surf setup.
2. INSTRUMENT VALIDATION (known-good tape): Record tab → select `basictest_solved`
   (seeded 430, works in-game) → Map Solve → Teleport to its anchor → Arm capture →
   play it. Then offline:
   `SolverLab diff <bsp> <basictest_solved.tas> solver\playback_states.csv`
   EXPECTED: agreement (first |dpos|>1 late/never). This proves the capture instrument.
3. THE QUESTION (new clean tape): same steps with `surf_basictest_solved` (1312).
   The diff then names the first divergent tick + everyone can see what geometry lives
   there. Either it tracks (Phase 1 ONLINE) or the divergence tick is the undeniable
   evidence the user asked for — and the fix target.

### Phase 1.4 — THE PROTOCOL RAN: two real bugs found, edge theory dead (2026-08-13)

The user ran the protocol and delivered captures (seeded playback + both tapes' engine
sims). Results, in order:

1. **INSTRUMENT + CORE PROVEN vs REAL PLAYBACK: max |dpos| 0.000 across all 429 ticks**
   of the seeded tape. The capture pipeline, anchor protocol, and core physics are exact
   against actual in-game playback, not just prediction.
2. **Poisoned-params incident**: the auto-loaded server_params.cfg held ALL-DEFAULT
   values (airaccelerate 10, bhop 0, stopspeed 100) — snapshotted while the server
   wasn't surf-configured (and/or unvalidated cvar reads). It silently corrupted the
   first diff pass (fake tick-17 divergence = the stopspeed-100 signature). Quarantined
   as `server_params_SUSPECT.cfg`. FIXES: params are now (re)written at every playback
   capture (the moment that matters), and `hull_stand 62` from that file is flagged as
   an untrusted single read (72 remains the parity-proven default). RULE: never filter
   the harness's `params:` lines out of a diff read-out.
3. **Real divergence #1 — tick 204, flat ground, mid-platform:** the DUCKED SPEED CAP
   applies the moment duck is PRESSED (engine showed pure friction −19.1 = 318·0.06 from
   the press tick), not when the 400 ms transition completes. Fixed: effmax uses
   (ducked || ducking). Verified: divergence moved 204 → 514, max drift 1481 → 130.
4. **Real divergence #2 — tick 514, post-landing:** the −775 u/s jump-free floor landing
   showed stamina scale = 1.00000 EXACTLY → **the stamina model was reframed: the JUMP
   arms the timer (fitted 1317.5 ms ≈ classic 25/19 s), it drains 15 ms/tick everywhere,
   and the RESIDUE at landing drags walk ticks.** Reconciles everything: jump t432 +
   705 ms flight = 611 ms at the t479 landing (the old "landing constant" 612.5), full
   bhop height at stamina 0, and zero slowdown after jump-free falls. A hot-stamina jump
   scales its impulse by the same factor (classic formula, not yet capture-exercised).
   Verified: divergence moved 514 → 917.
5. **Residual regime — tick 917: CS:S GROUND-DUCK LIFECYCLE** (transition/spam
   mechanics; engine walks duck-capped at wishspeed 85 while FL_DUCKING=0). Not modeled
   yet; instead the SOLVER emits duck presses/releases AIRBORNE ONLY (state-dependent
   input rule) — every duck regime in shipped tapes is a proven-parity class (air-duck
   ±8.5, ducked landing, ducked jump, air-unduck: all 0.000-verified). Ground-duck
   modeling = future capture study if crouch-walking ever matters on a surf route.
6. **EDGE-CONTACT THEORY: NOT CONFIRMED — gate removed** (user's skepticism vindicated
   twice). Both measured divergences were duck physics on flat ground, nowhere near
   edges. `taint_edges` now defaults OFF (`--edge-taint` re-enables if capture data ever
   convicts edges); the near-edge detector stays as an AUDIT COLUMN only. The earlier
   "spine hit" observation is explained as downstream fallout of tapes already
   out-of-sync from the duck bug + start ambiguity.
7. **Export discipline (user directive)**: every export now writes a unique
   self-describing name (`sim_<project> (N).csv`, `playback_<runname> (N).csv`) and one
   manifest line in `solver\exports.log` (timestamp | kind | map | source | ticks |
   path). Identification is a lookup, never a guess. Playback captures also rewrite the
   live params alongside.
8. **QOL**: "God (toggle)" button on the tab (fall-damage death breaks playback sync —
   the user's floor-landing report; god is a console toggle so it stays out of Surf
   setup to avoid silent double-click disarm).
9. **Shipped**: new unseeded route from the proven recording anchor under all fixes —
   **1125 ticks (16.86 s)**, finish z 256.03 ducked-landing at 466.9 u/s, all duck
   transitions airborne, 1 legal zone jump, 0 edge contacts →
   `recordings\surf_basictest_solved.tas`. IN-GAME TEST = the gate (god on;
   teleport-to-tape-anchor button; if it deviates, arm capture → manifest-named CSV →
   one diff command names the tick).

### Phase 1.4b — one-click capture flow (2026-08-13, user: "just make the process clean")
The Map Solve tab now owns the whole capture workflow: a **Recording picker** (library
combo with map + tick count), an **Export name field** (auto-fills from the picked run,
user-editable, governs BOTH the playback capture and the sim export — nothing lands as
"untitled" again), and one **"Teleport, play & capture"** button that teleports to the
recording's own anchor (setpos_exact + setang, freecam-aware), plays it via the ephemeral
player with the standard 12-tick settle grace, captures every real tick, and exports
`playback_<name>.csv` + live params + a manifest line on finish. Abort button while
capturing. No tab-hopping, no manual arming, no name guessing.

### Phase 1.5 — SECOND CAPTURE ROUND: jump tax fitted exactly, cruft stripped (2026-08-13)

User captured both tapes via the one-click flow (`playback_seeded_solve` /
`playback_unseeded_solve` — the manifest identified everything; the unseeded route
"failed the last jump" in-game and plummeted).

- Seeded regression vs REAL playback: **0.000 again.**
- **Retraction:** the earlier "engine does NOT tax hot-stamina jumps" conclusion was a
  WRONG-PAIRING artifact — that diff compared against a stale sim export of the OLD
  1312 tape after the tape FILE had been rewritten with the 1125 route under the same
  name. Lesson enforced: shipped tapes now get UNIQUE names (`*_solved2.tas`, ...);
  never rewrite a tape file that captures may still reference.
- **Real-playback jump audit (4 jumps in one capture): the impulse tax EXISTS,** with
  its own scale: `impulse *= 1 − stamina_ms · 0.000186` (walk-drag keeps its
  independently fitted 0.00019833; same 1317.5 ms arm + 15 ms/tick drain). The fit
  predicts all three taxed jumps (gaps 49/44/43 ticks) within 0.05 u/s and the
  zero-floor at gap 900. Constants are per-regime fits — do not unify without data.
- **Result: the failing 1125 tape now matches REAL playback at max |dpos| 0.137 u over
  all 1124 ticks, all flags exact** — the core reproduces even the route's real-world
  failure. The route itself was simply planned under the older, slightly-off tax.
- **Poisoned params, root-caused:** the generic ConVar reads RETURN DEFAULTS on this
  build (server demonstrably ran surf settings) — every capture was re-writing the bad
  file. `ExportServerParams` now writes ONLY trustworthy sources: measured tick
  interval + the editor's strafe model (gravity/airaccelerate/maxspeed/air cap);
  everything else defers to SolverLab's parity-proven defaults. hull_stand=62 read
  discarded with the rest.
- **Edge apparatus fully removed** (user directive): detector, TickEvents fields,
  taint inheritance, class-aware cell replacement, fragile finisher splitting, CLI
  flags, audit columns — all gone. If capture data ever implicates edge geometry, it
  gets rebuilt from evidence, not resurrected.
- **Shipped `surf_basictest_solved2.tas`**: unseeded, corrected-physics plan, 1208
  ticks (18.1 s), finish z 256.03 at 455 u/s, 1 zone jump (300 s run, rng 5; rng
  11/1337/42/7 at 120–150 s found nothing — discovery variance under the tighter
  physics reaffirms worker-pool parallelism as the next structural lever).

### PHASE 1 SIGNED OFF (2026-08-13): the user played `surf_basictest_solved2.tas`
in-game and **it lands on the platform**. Full loop proven end to end: unseeded
discovery from nothing → offline plan on the capture-proven core → real-engine playback
→ legal finish. Era gates cleared: consistent anchor, real-tick capture, physics parity
(0.000 / 0.137u), export discipline, one-click workflow.

## Phase 2 — the optimizer (opened 2026-08-13)

Contract: fitness = finish tick, NOTHING else; legality structural (flip spacing,
airborne-only duck transitions, startzone jump budget enforced in every eval); every
evaluation runs through the capture-proven core; improvements accepted only when the
re-rolled route finishes strictly earlier. Design: finisher chains flatten to genomes
{seed-prefix length, knot list}; operators = knot trims, knot deletion, suffix
resampling, parameter jitter, and seed-prefix erosion (branching earlier into the human
prefix); abort-at-best evaluation keeps evals cheap (~5.8k/s at 7M ticks/s).

### Phase 2 v1 — built + first results (2026-08-13)

**Built:** `SolverKnots.{h,cpp}` (Knot/KnotTick/MaxGainYaw/SampleKnot/FlipsLegal extracted
— one shared control vocabulary for explorer + optimizer, stream-identical to the old
inline code) and `SolverOptimize.{h,cpp}` (FlatGenome hill-climb: knot trims, deletion,
suffix resampling with the explorer's own distribution, parameter jitter, knot growth,
seed-prefix erosion with flip-state recompute; strict tick-improvement acceptance;
abort-at-incumbent evaluation → ~23k evals/s; genomes re-verified by full re-roll and
tapes truncated AT the finish tick). Wired as `solve --optimize-s N` (default 60; top-3
distinct-tick subjects split the budget). Default `--out` names now UNIQUIFY (never
overwrite a shipped tape — the capture-pairing lesson, enforced).

**First results (honest):**
- Seeded subjects: 432 → 430 in a handful of improvements (~930k evals each). The 430
  floor holds: the human seed prefix is a monolith knot operators can't restructure, and
  prefix erosion without a reconnect-repair operator almost never survives.
- Unseeded 1208: → 1207 in 1.18M evals. The route's slack is STRUCTURAL (long floor
  meanders), and single-mutation strict hill-climbing cannot restructure - as the
  original design notes predicted (splice/crossover is the structural operator).
- **Tightening-loop concept validated as machinery** (zero new code): exploring with
  `--max-ticks <incumbent-1>` makes every rollout that exceeds the best route abort -
  all discovery pressure goes to strictly-faster routes by construction. One 180 s
  trial round (rng 6, cap 1206) found nothing - single-threaded rounds are too
  expensive for the variance.

**Verdict: PARALLELISM IS NOW THE GATING WORK.** Discovery variance, tightening rounds,
and archive-splice operators all reduce to "make exploration rounds cheap." Next:
worker-pool exploration (shared or racing archives), then an automated tighten loop
{explore(cap=best-1) → optimize} and archive-shortcut splicing (graft archive-best
prefixes onto finisher suffixes at shared cells).

Shipped for optional in-game testing: `basictest_opt.tas` (seeded, 430) and
`surf_basictest_opt.tas` (unseeded, 1207).

### Phase 2 v2a — DISSIPATION BIAS + SEGMENT LAB (2026-08-13, user-directed)

Committed the era first: `ac781e7` (21 files, +4948).

**The user's energy intuition, measured** (core traces of the three verified runs,
E = 0.5|v|²+gz):
| run | ticks | dissipated | pumped | big-loss ticks | worst tick |
|---|---|---|---|---|---|
| human | 432 | 260k | 168k | 3 | 84k |
| seeded | 430 | 260k | 167k | 3 | 77k |
| unseeded | 1207 | 538k | 491k | 8 | 167k |
Fast runs dissipate ~0.73× E0 total (the user's "slight-to-moderate multiple above
closed") with only the intentional landings as big events; the slow route wastes 2.07×
as much and pumps 3× the strafe work to re-earn it. The actionable signal is
**cumulative dissipation at equal progress** (absolute E already failed the prior
attempt as a sole score — and our own ebands were the weaker form of this).

**Implemented (bias, never fitness):** per-path `eloss` on every archive entry
(inherited; += max(0, E drop) per tick); the round-robin's energy axis replaced by
**SMOOTH FRONTIER** — lowest-eloss entry among the 3 contact bands nearest the finish;
`--no-eloss-bias` control arm; finisher tables report eloss; **`--goal touch`** turns
any brush into a segment goal (the user's "fastest to ramp N" theory lab — segment
experiments are now one-liners with fast rounds and low variance).

**A/B (touch ramp 3, 30 s × rng {1,2,3}):** bias ON = 160k/164k/135k finishes; OFF =
88k/80k/85k — **~1.9× finisher throughput**, best-tick neutral (204-227 vs 205-206).
Full-map rng 11 at 150 s remains dry with the bias: it accelerates, it does not fix
single-thread discovery variance. Parallelism stays the gating work — and now
multiplies a 1.9×-richer finisher stream when it lands.

**User calibration (2026-08-13):** the human run was "far from clean — just decent";
smoother boards would beat it. And **do not lock in energy multiples** (the 0.73×E0
figure is ONE map's measurement, not a law) — stay flexible as new data lands. Both
points proved out immediately in v2b: see the 604-tick eloss inversion below.

### Phase 2 v2b — WORKER-POOL PARALLEL EXPLORER (2026-08-13)

v2a committed first as `59601d6`. The explorer is now a worker pool over ONE shared
archive (20 logical cores on this box; `--threads N`, default auto = cores−1 = 19).

**Design (SolverExplore.{h,cpp} rewrite):**
- Entries preallocated (4M cap + 4096 slack ≈ 0.5 GB buffer, pages commit on touch);
  allocation = atomic counter; **entries immutable once published** (finished flag set
  at creation — the goal check moved BEFORE recording so finisher entries are born
  finished; no post-publish mutation anywhere). Per-entry publish flags (release/acquire)
  make uniform selection safe against half-written slots.
- Cell map sharded 64 ways by mixed key hash — lookup+publish under one short shard
  lock, different map regions land on different shards. Hysteresis/replacement decisions
  inside the shard lock; entry snapshot reads lock-free (immutability).
- Dist/speed bands keep per-band mutexes + atomic size counters for lock-free empty
  checks; near-miss ring became 512 atomic slots (benign races by design). min-dist /
  max-speed = CAS loops; finishers under one mutex.
- Per-worker mt19937 (worker 0 seeds at exactly cfg.rng_seed); round-robin mode from the
  shared rollout counter. Frontier-freeze / air sub-cap semantics preserved (force-adds
  spill into the slack region so finishers are never dropped at cap).
- Monitor thread prints the 1 s heartbeat; census/closest reports unchanged post-join.
- CmdSolve: `--threads`; optimize stage now runs up to `threads` distinct-tick subjects
  CONCURRENTLY, each with the FULL --optimize-s budget (parallelism buys subject breadth,
  not budget splitting).
- Basehook untouched (it never compiled the explorer) — zero risk to in-game features.

**Determinism gate PASSED:** old v2a binary vs new at `--threads 1 --rollouts 200000
--rng 1` (touch-9): 16,076,635 ticks, 432,817 entries, 385,801 cells, 9,376 finishes —
every number identical, census identical, output tapes hash-identical. The refactor is
provably behavior-preserving single-threaded; multi-thread runs are non-reproducible by
design (interleaving), worker streams still seeded deterministically.

**Scaling (touch ramp 3, 30 s, rng 1):**
| threads | ticks/s | rollouts | finishes | best |
|---|---|---|---|---|
| 1 | 7.7M | 2.93M | 182k | 226 |
| 4 | 30.7M | 11.75M | 697k | 202 |
| 10 | 63.1M | 23.6M | 1.31M | 216 |
| 19 | 79.2M | 31.7M | 1.81M | 207 |
~4.0× at 4 workers, 10.3× at 19 (the box is 20 logical; past physical cores the
per-thread gain flattens, as expected). Lock design holds: throughput scales with
finisher throughput, so contention is not eating the archive traffic.

**Full-map dam BROKEN:** rng 11, the seed that was STONE DRY at 150 s single-threaded,
now: first finishes at ~90 s, 2,244 finishes by 150 s (best 2499, a meander — but it
finishes, so the optimizer can eat it). rng 1337 same budget: **4,619 finishes, best
604 ticks — the unseeded record HALVED (was 1207/1208)**, 73.3M ticks/s, 12 distinct-
tick subjects optimized in parallel (~6.2M evals/60 s aggregate vs ~1.4M sequential).
Shipped `surf_basictest_solved3.tas` (604, replay-verified: grounded on brush 10,
finish z 256.03, ZERO startzone jumps — it walks off the platform, saving the jump
budget entirely; finishes ducked at 491 u/s).

**eloss inversion (flexibility point proven):** the 604 winner dissipates MORE (440k)
than the 612 (392k) and 637 (381k) runners-up. Lowest-dissipation ≠ fastest at the
margin — exactly the user's tradeoff warning. Dissipation stays a SELECTION BIAS and
diagnostic; it must never become fitness or a gate.

**Tighten probe (rng 1337, --max-ticks 603, 150 s + optimize):** NO sub-603 finish.
Brush 10 touched 385×, min-dist 465, but never grounded under the cap; the capped
archive is leaner (1.37M entries) because long lines die at 603. Tighten rounds are now
CHEAP (one round ≈ 44 core-minutes) but this seed's next improvement wants better
OPERATORS, not just more capped exploration — see the themes data for where.

**Human-run themes (data-only pass, scratchpad/themes.py over core replay CSVs; tick
classification from raw deltas: ballistic air = dvz exactly −12, ramp/clip air = any
other airborne dvz):**
| run | timeline (phase ticks) | ramp episodes | board entries | mid-run big losses |
|---|---|---|---|---|
| human 431 | g70 j1 b72 r47 b47 r35 b34 r58 b66 g1 | 3 | 9.1°/13.7°/11.1° costing 11k/20k/10k | NONE (only the finish landing, 84k) |
| seeded-opt 429 | identical prefix; optimizer DROPPED the human's landing duck | 3 | same | none |
| unseeded-opt 1206 | 11 episodes; ~700 ticks stuck at x 800-880 | 11 | two SLAMS: 36.5° −166k, 37.2° −78k | 3 slams + 2 ground |
| unseeded 604 | walks off start (0 zone jumps); ends in a 3-hop chain onto red | 7 | 4 hard boards 25-33° costing 47-80k (~257k total vs human ~40k) | 4 board slams |
Other measurements: human strafes at max-gain (median yaw-vs-heading offset 2.3°,
p90 5.6° — validates MaxGainYaw as the control vocabulary); human flip cadence 1.54/s
with MIN gap 20 ticks (never near the 14-tick structural floor → the 5/s budget is not
binding for human-quality lines); human touches each ramp exactly once, zero mid-run
ground contacts, phase durations near-metronomic; duck usage is rare and situational
(human: one pre-landing duck the optimizer proved unnecessary; 604: short tactical
air-ducks).

**The data names the remaining gap:** 604's waste is almost entirely BOARD-ENTRY
harshness (25-33° vs the human's 9-14°). Neither uniform knot-jitter (optimizer ops
pick random knots) nor capped re-exploration targets contact-adjacent timing.
Candidate v3 operators, in order: (1) **contact-anchored mutation** — aim trims/jitter
at the knots around recorded board events; (2) archive-splice (graft archive-best
prefixes onto finisher suffixes at shared cells); (3) board-entry deflection as an
AUDIT column first (eloss may already price it — deflection is the cause, eloss the
effect; segment-lab A/B decides if it earns selection weight). Open intuition
questions for the user: is a hard board ever deliberately correct (speed-kill before a
transfer), and is the 604's end-stretch bhop chain a real technique here or an
artifact the tighten rounds should erase?

### CORNER-RELEASE FIX — the 604 tape's in-game failure (2026-08-13, user-reported)

**User report:** the 604 tape does not finish in-game (exported capture). Diff vs the
real playback: perfect to tick 283, then at 284 the core gains dv (0, +23.74, +18.99)
— exactly 30.4 u/s along brush 9's face normal (0, 0.780869, 0.624695): the core
performed ONE MORE face clip than the engine at the moment the box slid off the
brush's END (origin x 335.94, expanded end-cap at exactly 336).

**Dead hypothesis (tested, refuted):** non-axial edge bevels. Loaded them from the
lump behind `--no-edge-bevels` — THE LUMP HAS NONE for these brushes (ramps carry 6
real sides + 1 axial bevel only; brush 9 raw sides dumped side-by-side). Diff
identical both arms. (The loader change stays — it is lump-correct and loads nothing
here; the prior-attempt "bevels broke proven lines" warning was about synthesized
bevels, still dead.)

**Measured mechanism (scratchpad/corner.py, exact BSP planes + capture rows):** both
models rest at face d0 = +0.0312 (= DIST_EPSILON) identically, tick after tick. At
the 284 move: face enterfrac (eps-padded) = −0.000128, end-cap leavefrac = +0.003226
→ the padded interval says HIT (and the old core hit). But the TRUE un-padded
crossings say: face entry 0.0684 AFTER cap exit 0.0063 → the box exits the brush's
extent before it would actually re-touch the face → the REAL ENGINE reports no hit
and flies free. **DIST_EPSILON pads the reported position, never the hit topology.**
The Q2-family reference code conflates them; this engine does not. Neither our
Solver port nor the DLL's validated TraceHullWorld carried the distinction — no
previously captured route ever slid off a ramp's end mid-surf.

**Fix:** TraceHull now tracks the true interval (tmin_t/tmax_t, no epsilon) alongside
the padded one; a brush hit additionally requires the true interval be non-empty.
`--legacy-corner` keeps the old rule as a control arm. Position math, enterfrac
tie-break (unclamped, apex-validated) all unchanged.

**Validation:**
- 604 tape vs its real capture: first >0.1u divergence 284 → **544**; max 3.542u at
  602; ground/duck flag mismatches: NEVER. The corrected core reproduces the real
  (failing) playback — the tape was invalid plan output from the old physics, same
  failure class as every prior model bug.
- Regression battery, all prior captures: seeded 430 = 0.000u (both captures),
  unseeded 1125 = 0.137u @1123 (identical to proven), solved2 1208 = 0.024u max.
  Replay invariants: human 431, seeded-opt 429, unseeded-opt 1206 — unchanged.
- Bench: 15.26 → 14.2M ticks/s single-thread (~7% for two extra divides per
  crossing plane).

**Open sub-item:** remaining 544+ tail (0.1→3.5u over the last 60 ticks, during the
ducked ramp-11 spine ride, no flag flips; dv direction (0.59, 0.33, 0.73) matches no
single face — partial-fraction resolution differences). Needs its own micro-study;
40× smaller than the fixed bug but 3.5u can still flip a knife-edge ending.

**IN-GAME SIGN-OFF (2026-08-13, user):** `surf_basictest_solved4.tas` (1120) FINISHES
in-game — the first reality-validated unseeded solve on the corrected corner physics.

**Corrected-physics re-solve (rng 1337, 150 s + 60 s optimize):** 2,710 finishes,
explorer best 1136; the parallel optimize stage earned its keep — all 8 subjects
improved, best 1136 → **1120 ticks** (16.8 s). Shipped `surf_basictest_solved4.tas`
(replay-verified: grounded on brush 10 @ 452.9 u/s, ZERO startzone jumps, max speed
750). The 604 exploit class is gone; this route is honest meander-class — long
low-speed middle around ramp 11, one spine ground+jump (t1070, 402 u/s) launching to
red. The unseeded gap vs human-class (430) is now legitimately SEARCH quality, which
is what the v3 operators (tangent-boarding prior, contact-anchored mutation,
archive-splice) are for. Energy note: with corrected physics the finisher cohort
re-aligned with the user's intuition even at this horizon — 1136t @ 444k dissipated
vs 1139t @ 511k (faster = lower loss within-cohort this time).

**User surf calibration recorded this round:** (1) energy: at the current
poor-quality horizon dissipation won't correlate with best tick among solver runs,
but vs human/seeded runs lower needless loss still tracks quality; expect the
correlation to STRENGTHEN as runs approach the efficient horizon. (2) **Tangent
boarding**: landing basically tangent to the ramp plane is faster ~95%+ of the time
and is a PRIMARY efficiency driver (the tangent target shifts with approach angle to
minimize loss) — strong prior for contact-anchored operators; validate weights in
the segment lab regardless. (3) **Spine bhops are generally bad — discourage** (the
604's ending hopped brush 11's spine at z 257.5). Mechanism: bias/operators, not
bans; check whether corrected physics + eloss bias already avoids them before adding
machinery. (4) The human's pre-landing crouch compensated for an imperfect line —
but note the real micro-tech: a faster, flatter ballistic ending can use the
duck's leg-lift (+8.5u) to land EARLIER. The optimizer dropping the crouch on the
seeded run was consistent with (4).

### Phase 2 v3a — CONTACT AIM (measured neutral), BOUNDARY-SHIFT OP, TIGHTEN LOOP (2026-08-13)

Built after the solved4 in-game sign-off, per the user's "keep moving forward."

**Contact-anchored aiming: built, A/B'd, measured NEUTRAL → default OFF (`--aim`).**
Instrumented eval records the knot indices at the route's top-8 one-tick energy
drops; half the knot-picking mutations then target those knots (or the approach knot
before). Segment-lab A/B on DETERMINISTIC subjects (--threads 1 + fixed --rollouts →
both arms optimize the identical genome): touch-9 rng{1,2,3} aim 207/264/219 vs
uniform 205/264/209; touch-11 rng{1,2} exact ties (308/308, 297/297). Verdict: at
~1M evals/subject the acceptance bottleneck is NOT proposal targeting; aiming
redistributes proposals without earning ticks. Apparatus kept behind `--aim` (full-map
genomes are 5× longer — 8 loss events are far more selective there; untested).
Honest caveat: touch-goal segments also pollute the aim set (the goal contact IS the
biggest loss).

**Boundary-shift operator (kept, op slice 27-35 from trim's 35):** moves 1-2 ticks
between ADJACENT knots — total length preserved, downstream knots unmoved. This is
the only operator that can express "hand over from approach to board knot slightly
earlier/later" without displacing the rest of the route — the tangent-boarding
micro-move (user: tangent boarding is the ~95% rule and a primary efficiency
driver). Segment check vs pre-shift reference: 207/264/210 + 307/297 vs 205/264/209 +
308/297 — neutral (±2, stream noise). Segments are a SATURATION regime for the
optimizer (tiny genomes, ~1M evals) — they prove non-harm, not benefit; full-map
rounds are the real test.

**Tighten loop (`--tighten N`), incumbent-SEEDED:** after the primary
explore+optimize, each round re-explores with max_path_ticks = incumbent−1 and seeds
the archive from the incumbent's own frames (in-memory tape, same anchor) — restart
points all along the best route, so the search only finds where to DEVIATE, not the
whole line. SeedFromTape now stops at the cap (restart points at/past it are dead on
arrival). Any finish in a capped round is a strict improvement by construction. The
optimize stage was refactored into BuildBestFrames (shared by primary + rounds; each
round's subjects get round-shifted rng).

**Tighten VALIDATED — record cut 1120 → 965 (−14%).** Run: primary seeded from
solved4 (60 s explore: 631,555 finishes — the seeded line re-walked en masse — but
8 optimize subjects ALL stuck at 1120: mutation alone cannot break the structure) →
round 1 (cap 1119, seeded from the incumbent): ONE finish among 35.6M rollouts, a
969-tick deviation, optimizer → **965** → round 2 (cap 964): dry in 60 s, cleanly
bounding this seed's progress. Contrast with the COLD capped probe (603 cap, 150 s,
zero finishes ever): incumbent seeding is what makes capped rounds land. Shipped
`surf_basictest_solved5.tas` (965, replay-verified: grounded on red @ 379.8 u/s,
0 startzone jumps, finish y +76 — a genuinely different landing spot than the
incumbent's y −700, not a shaved copy). Note the primary's dry variance at 90 s
budgets (the same rng that finished at 150 s found nothing in one 90 s run —
first-finish time on this map sits near 30-90 s wall, budget accordingly).

### START-ENERGY + CLEAN-APPROACH EXPERIMENTS (2026-08-13, user-directed)

User theories to test: (1) a startzone jump to ramp 1 "almost certainly helps"
(free prestrafe+jump energy the walk-off starts leave behind); (2) the endgame
energy deficit comes from board losses + the missing start energy; (3) requested
test: "get to the next ramp cleanly while wasting the least energy." New lab
features: **cleanest-routes table** (finishers re-ranked by eloss, printed beside
the fastest table), **`--out-eloss`** (write the min-dissipation finisher tape),
**`--seed-ticks N`** (seed only a tape's first N ticks — hand the archive an
OPENING without the rest of the line).

**Exp 1 — fastest vs cleanest to ramp-1 touch (30 s, rng 1):** fastest = 100 ticks,
39.0° entry, 695→563 u/s, **89.6k lost**; cleanest = 130 ticks, ~tangent graze,
428 u/s retained, **1.1k lost** (30-tick premium buys back ~90k). BOTH openings
prestrafe + startzone-jump (fast: jump t24 @301; clean: jump t32 @311, airstrafes
to 375 pre-exit at 4.6° off max-gain vs the slammer's 11.2°) — at segment horizons
the solver finds and uses the zone jump on its own. Caveat measured in the data:
for the segment METRIC the slam still wins (563 u/s, 30t earlier) — its true cost
is downstream direction/energy, which touch-goal ticks cannot price. Segment
fitness alone would overfit to slams; dissipation-at-progress is what sees it.

**Exp 2 — opening-only seed (--seed-ticks 120 of basictest.tas: prestrafe, jump,
zone exit, NOTHING after; 150 s + 45 s + tighten 1): RECORD 965 → 630 (−35%).**
Primary produced 1800-tick meanders; the tighten round (cap 1800, incumbent
carries the human opening) exploded — 1,030 finishes, optimized 630. Shipped
`surf_basictest_solved6.tas` (replay-verified: grounded on red @ 423.6 u/s).
**The twist: seed erosion DELETED the human's jump.** The winner prestrafes 87
ticks to 316 u/s (human: 70t/282) and WALKS OFF — dropping onto ramp 1's face sets
up a pump the jump-arc start doesn't get: board @480 → leave @814; ramp 2 GAINS on
the board (829→893); ramp 3 converts to height. Total start energy favors the jump
(282²/2+302²/2 > 316²/2); arrival GEOMETRY beat raw energy — the user's "general
rule with exceptions" caught in the act, and the right verdict process (both
openings competed in one archive; ticks decided). Start + ramps 1-3 are now
clean; the ENTIRE remaining gap to the human 432 is the ramp-4 endgame (~260
ticks of pocket-dancing, two jumps). Honest flags: the 630 ends with a SPINE-TOP
jump onto red (grounds on ramp-4 spine t579, jumps @408) — the user-disliked
pattern is back in the record holder and eloss pricing did not kill it; the zone
jump stays unused. Spine-discouragement mechanism question is now LIVE (bias vs
operator), and endgame structure is the next target.

### ZONE CLOCK + TAIL DIAGNOSIS (2026-08-13, user-directed)

**Zone clock (user directive): the timer starts at STARTZONE EXIT.** Fitness,
cell-replacement hysteresis, abort-at-incumbent, tighten caps, and finisher ranking
all run on ticks-since-exit ("scored"); absolute ticks still bound the simulation
(max_path_ticks backstop; `max_rel_ticks` carries the scored caps). Prestrafe is
FREE — this also structurally removes the tick-greedy cell pressure that suppressed
investment starts (a long prestrafe no longer loses its cells to a quick walk-off).
`--clock anchor` restores the old absolute clock. Entry carries exit_tick; replay
prints the zone-clock score for any tape. Caveat logged: the replay's exit detector
is XY-only while the solver's InsideStartZone is XY+z (identical for normal exits
over the lip; differs only for under-platform paths). solved6's in-game FAILURE
(user: falls just short of red) is consistent with the 544-tail residual (below) —
its landing margin was ~3 u.

**Zone-clock baselines (replay, scored ticks):** human 319 (exit 112, finish 431);
seeded-opt 317; solved5 877 (exit 87); solved6 542 (in-game INVALID). The target
number is now 319.

**Zone-clock solve (rng 1337, 150 s + 45 s + tighten 1, FULLY UNSEEDED): 617
scored ticks** (708 abs; primary 1242 → tighten round found a 620-family deviation
→ optimized 617). Shipped `surf_basictest_solved7.tas`. Structure: 90-tick
prestrafe to 333 u/s (the free clock stretched the opening past the old 87t/316),
double-touch pump off ramp 1 exiting 864, clean ramps 2-3 (peak 872), then STILL
~350 ticks in the ramp-4 pocket (near-stall at 70 u/s) and a spine-top jump @453
to red. Landing margin 11.7 u past the edge (vs solved6's fatal 3.4 u). Scored
ladder now: human 319 / solved7 617 / solved5 877. The pocket + ending remain the
whole gap — consistent with the user's "not enough energy at the last ramp"
diagnosis; the endgame needs the compromise-equation work once the settle residual
is fixed.

**544+ tail DIAGNOSED (existing 604 capture vs current core):** the whole ≤3.5u
tail is ONE 5-tick event at ticks 544-548 — during a progressive settle onto ramp
4's face (ducked, dvz sequence +77/+53/+40/+32/+27/+24 then DIVERGENT: eng +15.5 vs
core +20.7, converging by 549). Before: bit-perfect. After: BOTH models track the
identical ride rate (−7.32/tick exactly) and the offset just propagates
(+0.04-0.06 u/tick, parallel dynamics). Same knife-edge family as the corner bug,
40× smaller: sub-epsilon clip-fraction resolution while pressing onto a plane.
NEXT DATA NEEDED: a fresh capture of a failing endgame tape (solved6 playback
export) for a second instance to fit the rule against — one event is not enough to
fix without guessing.

### TRACE ORACLE — ENGINE-TRUTH COLLISION (2026-08-13, user-directed)

solved7 ALSO failed in-game. User: "We have the actual physics engine right here,
why not take from that instead of guessing?" — correct. Two knife-edge rules fitted
from captures, two new knife-edges found in reality. New methodology: the injected
engine ANSWERS OUR TRACES DIRECTLY.

**Built:**
- DLL (`Include/cstrike/Interfaces/IEngineTrace.h` + Map Solve tab): "Run trace
  oracle" reads `solver\trace_queries.csv`, answers every row with
  IEngineTrace::TraceRay (EngineTraceClient004/003, MASK_PLAYERSOLID,
  TRACE_WORLD_ONLY filter — the same CM path player movement uses vs worldspawn),
  writes `trace_results.csv` + a manifest line. ABI pins (TraceRay vtable index,
  default 5 = 2013 layout; Ray_t with/without m_pWorldAxisTransform) are probed by
  a KNOWN-ANSWER battery from the solve anchor before any batch, and the probe
  call is SEH-guarded — a wrong pin reports instead of crashing the game (the
  IVDebugOverlay probe-then-pin discipline). trace_t read at fixed CBaseTrace
  offsets into an oversized buffer.
- Lab: `replay --trace-log <csv> [--trace-window a b]` logs EVERY TraceHull call
  (query + our answer; the file doubles as the oracle input);
  `tracediff <queries> <results>` compares us vs the engine per trace (exact /
  <1e-4 / <1e-3 / mismatch buckets, worst-12 with tick context).

**Staged: 2,072 queries** (`solver\trace_queries.csv`) = every trace of the full
solved7 tape (1,800) + the 604 settle window ticks 500-604 (272). ONE user click
in-game (Map Solve → Run trace oracle, with surf_basictest loaded) produces the
engine's answers; tracediff then names every trace where our TraceHull disagrees
with the engine — the settle event and solved7's failure become exact per-trace
deltas instead of inferred rules. Fix TraceHull until the battery is clean, then
regression-replay everything.

### CORNER RULE ORACLE-PINNED (2026-08-13) — collision now answers like the engine

**Oracle run 1 (2,072 real-route traces):** 2,030 bit-exact, 41 within 1.3e-4
(per-trace expansion rounding, ~0.0004 u, benign), **ONE hard mismatch**: solved7
t512 (spine climb) — engine HITS ramp 4's west face at frac 0 where our
true-interval rule skipped the brush. Combined with the 604 t284 engine-MISS, no
single interval variant explained both → **oracle sweep battery** (1,483 synthetic
queries around both cases: face-d0 through the epsilon region, trace length,
corner slides; run-1 files backed up as *_run1.csv).

**Sweeps split cleanly:** A-family (leave through a REAL side) behaves
true-interval; B-family (leave through a BEVEL side) behaves plain-padded. Two
independent flips pinned the boundary to one 0.005 u step. **THE RULE (H′): a
brush is skipped iff the TRUE (un-padded) entry time ≥ the earliest REAL-SIDE
leave extended by one DIST_EPSILON ((d0−ε)/(d0−d1)); BEVEL sides never release**
(they exist to smooth corners — releasing through them would punch holes at every
corner). Padded interval still sets the fraction. H′ = **0 disagreements on all
3,555 engine answers** (both batteries), and explains both original cases.

**Implemented in TraceHull** (bevel flag = pid<0, already tagged) + `traceself`
command (run our TraceHull over a stored query file → results format) so any
future TraceHull change re-verifies against stored engine answers offline.
C++ port vs engine: 0 hit/miss disagreements, worst frac delta 3e-4.

**Regression + the payoff:** all prior parity EXACT (seeded 0.000 / solved2 0.024
/ unseeded 0.137 / finishes unchanged) — and **solved7 vs its REAL failing
capture: 0.000 u over the ENTIRE tape.** The failure is fully explained: the old
model missed the t512 clip, the solver planned through it, reality clipped. The
model now sees what the engine sees.

**Settle event EXONERATED as a trace bug:** all 272 settle-window traces (604
t500-604) match the engine <1e-3 — the 3.5u settle residual lives in the MOVEMENT
layer's query pattern (which traces a tick issues), not in trace answers. Only
known residual; confined to a dead route class; own hunt later.

**New pre-ship gate:** every candidate tape's full trace log gets oracle-verified
(one in-game click) BEFORE the tape is handed over for play.

### PIPELINE CLOSED + ROUTE NORMALCY ROUND (2026-08-13, user-directed)

**solved8 FINISHES IN-GAME** — first reality-confirmed finish on engine-verified
collision. Gate: 0 mismatches in its 4,292 traces (4,168 bit-exact). Capture:
0.1u through tick 1471 of 1661 (89%), one movement-layer residual event after
(10.2u peak, 2.4u final — survivable; third settle-class data point, still
confined to trace-issuing patterns, not trace answers). It uses the STARTZONE
JUMP (exit t198) — the first full-map route to take the free energy, now that
the phantom walk-off shortcut died with the corner fix.

**User verdict: finishes, but bizarre — "get the solutions to be normal." Spotted
1-tick yaw snaps.** Measured (per-tick |Δyaw|): human max 5.1°, ZERO snaps >10°;
solved8: 180 snaps >10°, 15 >30°, max 169° — EVERY large snap airborne at
40-110 u/s. Mechanism: air yaw = MaxGainYaw tracks the VELOCITY HEADING
absolutely; at low speed the heading is ill-conditioned (30 u/s per-tick gain
rotates a 40 u/s velocity ~40°), so the controller faithfully chases a flailing
target. The snaps are the SYMPTOM. The disease: low-speed pocket-flailing
survives selection because it is DISSIPATION-CHEAP (a slow meander loses almost
nothing — the eloss axis reads it as clean). Loss-at-progress cannot distinguish
parked from fast.

**Fix (user's compromise equation, clause 1, as a selection axis — no gates, no
rate limits per the user's framing):** ENERGY FRONTIER — the round-robin's idle
mode-3 slot now restarts from the HIGHEST-energy entry (0.5|v|²+gz) among the 3
contact bands nearest the finish; exact mirror of the smooth-frontier, no new
storage, `--no-energy-frontier` control arm. Snaps lose their habitat instead of
being gated. A/B in flight (full pipelines, rng 1337, on/off).

**Energy-frontier A/B VERDICT (full pipelines, rng 1337): full-E form HARMFUL —
2774 scored vs control 804.** Altitude dominated (E = ½v²+gz lets 300u of height
outvote 700 u/s), the axis selected slow HIGH touches, and restart concentration
collapsed archive diversity (522k entries at 29s vs the usual ~1.7M). The prior
attempt's absolute-E dead end, rebuilt as a bias and killed by the same measurement.
DEAD END — do not resurrect E-with-gz selection in any form. Kinetic-only revision
(fastest contact near the finish) measured NEUTRAL on seeded 90s screens → default
OFF (`--energy-frontier` enables for a future full-budget test).

**The control arm set the RECORD: 804 scored (933 abs)** — same config as solved8's
1463, pure seed variance. AND it is the most normal solver route yet: 46 snaps >10°
(vs solved8's 180), max 66° (vs 169°), only 44 low-speed airborne ticks of 933 —
faster routes shed the flail habitat, exactly the user's prediction that efficiency
filters the snaps. Shipped `surf_basictest_solved9.tas`, gate queries staged
(2,444; run-2/3 batteries backed up).

**Experiment-cadence discipline (user: "taking way too long"):** screens run at
60s+30s no-tighten (~90s); SEEDED screens for A/Bs (unseeded 60s primaries are
first-finish-variance noise — both arms dry); full budgets ONLY for record
attempts after a change proves out at screen scale. Screens saturate (804 tie both
arms) — they prove non-harm, not benefit; treat accordingly.

### VALUE-MIX FRONTIER — THE USER'S ENERGY MODEL WINS (2026-08-13)

**User correction:** raw-E failing ≠ energy failing — the directive is a CONTROLLED
tradeoff: "find the best value mix between potential and kinetic energy for the
lowest loss." And: unless something beats the human run without energy, energy
stays in the search. Also: narrate long runs live (monitor went dark), and account
for throughput honestly (a 150 s solve = ~10 BILLION ticks / ~100M rollouts; the
funnel to ~2M archived entries and 10²-10³ finishes is best-per-cell selectivity,
not idle hardware; search efficiency, not tick throughput, is the lever).

**Implemented: V = KE + μ·(g·z) − λ·eloss** as the mode-3 restart axis (`--emix μ λ`;
within a band absolute-z cancels — only the mix matters, nothing map-specific).

**Sweep (150 s explore-only, rng 1337, narrated live):**
| arm | μ | λ | finishes | best scored |
|---|---|---|---|---|
| A | 0.25 | 0.5 | 1,142 | **778** |
| B | 0.50 | 0.5 | 1,126 | 1824 |
| C | 0.00 | 1.0 | 5,107 | 2268 |
| control | off | — | 104 | 1117 |
The genuine mix beats both poles AND the control (10× finisher stream, −339 raw
ticks). μ=0.25/λ=0.5 is now the DEFAULT. The user's energy framework, at the right
ratio, is measurably the best restart signal the search has.

**Record: 778 → 762 scored** via two seeded tighten rounds on the sweep-winner tape
(778→768→762; a same-flags rerun of the primary hit thread-interleaving variance —
1 finish at 79 s — killed it rather than wait; tighten-on-incumbent is the reliable
lever). Shipped `surf_basictest_solved10.tas` (762 scored / 846 abs, walk-off exit
t84, landing (1200,−828)). **Normalcy trend intact: snaps>10° per record: 180 →
46 → 27; max 169° → 66° → 44°** — efficiency filters the flailing, zero gates.
Gate queries staged (2,037; run-4 backup).

Scored ladder: human 319 | solved10 762 | solved9 804 (in-game status pending both).

### HUMAN-CALIBRATED MIX FIT + RECORD 695 (2026-08-13, user-directed)

**The human run as the tuning baseline (user directive), made rigorous:** band
every run's FIRST-ARRIVAL CONTACT state by distance-to-goal (the explorer's own
256u bands), then grid-search (μ, λ) for the mix under which V ranks states the
way reality ranks the routes (9-run quality ladder: human 319 + 762/778/804/1117/
1463/1824/2268/2774; scratchpad/mixfit.py). **FIT: μ=0.90, λ=2.0** — Spearman
0.670 vs 0.581 for the hand-picked 0.25/0.5; the human ranks #1 in 7/9 shared
bands under it. Reading: **V ≈ E_total − 2×waste** — the user's "total energy
matters every time" is the fitted truth; the raw-E failure was the MISSING LOSS
PENALTY, not the PE term (gz alone anti-predicts at −0.28; eloss alone 0.63 =
strongest single feature). NEW DEFAULTS μ=0.9 λ=2.0.

**The gap, quantified per band (human vs 762):** through bands 4-6 (ramp-3→4
stage) the human carries 373-436k KE having spent 70-132k; the 762 arrives with
165-233k having wasted 252-304k — half the energy, triple the waste; and even at
ramps 1-2 the human's boards lose ~5× less. The ramp-4 farming is an
energy-budget deficit that builds from the first board.

**Validation (cold explores too high-variance to referee — measured again):
tighten-screen A/B from the 762 incumbent, cap 761, 2 seeds/mix:** fitted mix =
one dry + one DEEP single-deviation cut (761→731, −31); old mix = shallow-frequent
(864 finishes → −10, plus a 760 from the optimizer). Different search characters
(focused/rare/deep vs diffuse/common/shallow); the record came from the fitted
mix. Chained pass on the 731: the seeded primary's OPTIMIZER cut 731→**695**
(−36; the archive around the line finally gave mutation compressible subjects);
tighten dry below 694. **Shipped `surf_basictest_solved11.tas` (695 scored /
780 abs, 10.425 s). Ladder: human 319 | 695 | 762 | 804.** Snaps 47 (>10°), max
67° — normalcy holding around the 800-class level, not yet improving below it.

### UNDUCK-LANDING MECHANISM FOUND (2026-08-13/14) — settle-class root-caused

solved11 FAILED in-game (user; also: the 780-frame tape vs "695" = zone-clock
display — 695 is the SCORED portion, tape length is absolute). Capture: discrete
branch at t729 (0.1u and 1u the same tick), 18.2u by the end. Gate: 0/1,993
mismatches — collision exonerated a THIRD time. The route at t729: LANDS DUCKED
on the ramp-4 spine with the duck released the same tick, jumps next tick.

**Mechanism (capture-fitted in two steps):** the SDK's PlayerMove RE-CATEGORIZES
POSITION right after Duck() — an unduck origin drop (−8.5) that lands the origin
within ground range makes the WHOLE tick a ground tick: friction (v 414.9→388.9
measured ✓), walk accel, stay-on-ground (ends ON the surface: 256.031 = top +
DIST_EPSILON ✓) — while the end-categorize-only model ran it as an air tick and
kept phantom speed + 1.9u of height. **Fix: CategorizePosition after any
duck-state change** (SDK-shaped, narrow; air duck/unduck far from ground is
untouched). solved11 capture: 18.2u → **1.85u**; ALL prior parity EXACT (0.000 /
0.000 / 0.024 / 0.137 / solved7 0.000 full).

**Remaining 1.85u:** a ~3.6° heading difference during the ground-tick WALK on
the 32u-wide spine — the documented Phase-0 StepMove gap (our WalkMove slides
only) with its first live exhibit. StepMove port queued. Every settle-class event
(604 t544 ducked ride, solved8 t1472, solved11 t729) is duck+near-ground; this
mechanism plausibly covers the family — re-measure after StepMove.

**Systemic note (the user's fundamental point):** the failing tapes keep dying on
1-tick duck stunts at knife-edge spine states — routes the search LOVES because
marginal/phantom gains at fragile states win ticks. With the ground-tick
mechanism modeled, unduck-landings now cost real friction in the model too
(26 u/s/instance), pricing the stunt honestly. The class-level answer is next:
(1) ROBUSTNESS SHIP-GATE — re-eval the winner under small state perturbations,
ship only if it still finishes (kills <2u-margin routes regardless of residual
model gaps); (2) SCORING/FILTERING refit from the ladder (loss functional:
linear vs quadratic vs max-event — "smooth most of the time" = few big events;
plateau-drift acceptance in the optimizer: equal-tick lower-loss moves accepted).

### VALUATION AUDIT (2026-08-14, user-directed) — the generator is the bottleneck

**User boundaries:** NO robustness gate, no arbitrary blockers of any kind — a
landed run counts wherever it lands. Scoring refit only. Free start energy must
rise on its own; smooth boards/turns must rise on their own; the human run is the
calibration reference ("I don't want it to learn the map, but I do want it to
know how to surf"); ML acceptable only for TRAINING the search's valuation —
search stays the product.

**Loss-functional refit (lossfit.py):** LINEAR cumulative loss WINS (Spearman
0.691) over quadratic (0.667) and worst-event (0.607) — "few big events" as a
formula does not beat the plain sum. Joint fit moved the mix: **μ=0.40, λ=6.5**
(waste punished ~3× harder than the previous fit). New defaults.

**New instrument: `solve --audit <tape>`** — replays a reference tape against the
LIVE archive: per-band cell verdicts (add/replace/tie/lose), reference-V
percentile vs 400 band samples, zone-jump census.

**Audit findings (human vs 150 s unseeded archive):**
1. **Valuation: FIXED.** Under μ.4/λ6.5 the human outranks ~400/400 archive
   samples in every band past the launch (archBest −1,380k at band 10 vs human
   +284k). Selection would choose human states everywhere — they just don't exist.
2. **Zone-exit tie inversion: MEASURED then FIXED.** Human 251k-V exit states
   were tie-rejected by 198k first-comers (±4 rel band, first-come ownership).
   Fix: PRE-EXIT cells decide ties by VALUE with a materiality margin from the
   archive's own quantization (one speed-bucket's KE width — no new constants).
   TWO CHURN LESSONS on the way (both measured, both reverted): global V-ties
   ate the whole 4M entry budget in 77 s (every +ε flip allocates; 52k cells,
   search parked); margined-but-global still churned (eloss deltas beat any KE
   margin). Zone-scoped = healthy dynamics (1.87M entries / 1.57M cells).
   (Audit's verdict column still classifies by the old ±4 rule — cosmetic,
   update with the next audit change.)
3. **THE BOTTLENECK: the GENERATOR.** Bands 10→3 are all "add" — the search
   never visits human-class cells at all. From a good frontier state, random
   knot continuations hemorrhage energy within a band or two (every band's
   archBest is deep-negative); the frontier polishes the best of a garbage
   population. Selection cannot fix what sampling never generates. "Knowing how
   to surf" must live in the CONTINUATION SAMPLER.

**Next (the generator program):** micro-lookahead knot sampling — at rollout
knot boundaries, sample K candidate knots, simulate each a short horizon, keep
the best under the fitted V (a player considering options; map-general, pure
search). Optionally refit SampleKnot's distributions (dur/side/turn) from the
human tape's measured knot statistics. ML-as-tool (learned V or learned knot
proposals trained across maps) stays sanctioned if hand-built lookahead
plateaus; per-map ML stays out.

### MICRO-LOOKAHEAD: BUILT, MEASURED, DEFAULT OFF (2026-08-14)

User greenlit with the principle "the solver should always know the most
efficient possible moves available." Built `LookaheadKnot` (SolverKnots): each
knot decision draws C candidates from the legal distribution, simulates each up
to H ticks, continues with the best fitted-V (`--lookahead C H`, honest tick
accounting into the rollout totals; per-tick air yaw inside each sim is already
the max-gain optimum, so this lifts the certainty to the discrete choices).

**Measured:** the designed structural effect LANDED — zone-jump census flipped
from 500:1 walk-off-starved to **3:1 JUMP-DOMINANT** (325,733 vs 112,655); zone
band best-V rose 120k→190k. But END-TO-END the ~2.4× tick tax loses: cold
primary 2726 scored (noise-prone, but poor); decision-grade seeded tighten
screens (from the 695-class incumbent, which re-rolls to ~706 under the duck-fix
physics — its old ending exploited the phantom unduck): lookahead 706 + one dry
arm vs control **665 / 767**. On seeded screens the incumbent already supplies
good states and raw rollout volume wins. **DEFAULT OFF** — apparatus kept for
cheaper variants (smaller C/H; lookahead only at contact-adjacent decisions;
candidate reuse). Results define conditions, including for my own builds.

**Screen byproduct = NEW BEST: 665 scored (750 abs, 9.975 s)** from the control
arm — shipped `surf_basictest_solved12.tas`, gate staged (1,949 queries, run-6
backup). Built entirely under current physics (duck fix in). Snaps 45 (max 67°),
41 low-speed air ticks — normalcy steady at the record class.
**Scored ladder: human 319 | 665 | 695* | 762** (*695 model-stale post-duck-fix).

### FINISH-FLIGHT PROBE + CLEAN-ENDING RULE (2026-08-14, user breakthrough round)

**User conditions:** anything over 500 scored = the same trash family; a jump off
the last ramp's spine into the finish is UNACCEPTABLE (a user-defined acceptance
criterion, not an internal blocker).

**Built: the FINISH-FLIGHT PROBE** — near-goal contact states get a release
check (coast/strafe-L/strafe-R at max-gain yaw, duck held, NO jump, ≤120 ticks):
a landing IS a finisher, the flight appended as one knot in the existing genome
language. Collapses the horizon from "reach red" to "reach any releasable
state". Probe cadence tuned through two measured failure modes: accepts-only
goes silent in mature archives (owned cells reject, no probes); every-contact-
tick collapses throughput 50× (riding pays 360 sim-ticks/tick). Final: always at
decision points (accepts, landings) + 1-in-16 on rejected riding ticks (~2.5×
rollout cost).

**Built: the ENDING CLASSIFIER + ratchets** — four definitional iterations, each
forced by a counterexample tape:
1. rollout-window "was the last left_ground a jump" → optimizer freely mutated a
   clean 674 into a jump 671 → OPTIMIZER RATCHET (clean subjects never accept
   jump endings; tainted may rise to clean, one-way).
2. subject-choice + tighten-accept adopted tainted products → PIPELINE-LEVEL
   ratchet (clean product never traded for tainted, any ticks) + per-subject
   class labels + [CLEAN/JUMP-END] on every shipped line.
3. ground-only departures let an old mid-route jump taint face-riding routes →
   any NON-FINISH face touch resets the ending class.
4. rollouts initialized the classifier fresh → a late-seed rollout "forgot" the
   prefix's spine jump (marked the 495 chassis clean) → classifier state is now
   INHERITED on entries like eloss/zone_jumps. Explorer marks ≡ optimizer evals.

**Results (all rng-1337/41 pipelines):** the probe detonated discovery — 22,473
sub-cap finishes in one tighten round, cascade 847→560→**495 scored** (7.425 s;
under the 500 bar)... which the honest classifier then exposed as JUMP-ENDED
(quiet spine grounding t537). Seeded reruns on that chassis with the final
classifier: **0 genuinely clean endings in 528,921 finishers** — on this chassis
the ramp-4 face rides carry ~330 u/s and the release flight cannot reach red.
THE CLEAN ENDING REQUIRES THE ENERGY CHAIN (human: face exit at 862 u/s). The
user's diagnosis holds at every level of the system.

**State:** nothing shipped this round (no acceptable tape exists yet — reported
honestly). The machinery is now trustworthy: probes find every reachable direct
ending; classification is inheritance-exact; clean beats tainted at every stage.
The open problem is BUILDING the energy chain: mid-route states fast enough that
the probe's release reaches red.

## 2026-08-14 — PARADIGM SHIFT: LINE-SPACE SEARCH (smooth spline + CMA-ES)

User verdict opening the round: *"We need a new paradigm for the search... the
boards are not clean (landing hard on a ramp while pointing tangent to it means
its still a hard landing since the approach wasn't clean). There is still yaw
jumping all over the place. I think the knots are not helping... I don't know
how its even possible to search this many combinations and NOT get an
efficient line when that is basically a driving component of the search."*

**The answer to that question (now measured, not argued):** knots are a
BANG-BANG controller. Every air tick strafes at exactly max-gain, so the turn
rate is physics-dictated (~1719/speed deg/tick) and the ONLY control authority
is when to flip sides. Clean approaches need CONTINUOUS modulation of how hard
you strafe — trading gain for line curvature so the VELOCITY (not the view)
arrives tangent. That family is unrepresentable in knot space: the archive
ranked 100M rollouts of the same jittery class because the generator never
proposed anything else. Valuation picks the best of what is proposed; it
cannot manufacture what is never proposed.

**The new control language** (Source/Solver/SolverSmooth.{h,cpp}, `smooth`
command): u(t), a Catmull-Rom spline over control points every cp_ticks.
u=±1 = max-gain strafe; |u|<1 = partial (wish projection eats (1-|u|) of the
live cap — the per-tick ADD is linear in |u|); |u|>1 = carve past 90°
(overdrive_deg span); sign = strafe key. One jump gene + duck on/off tick
genes (10-tick units). Yaw = heading + side*(phi-90) from LIVE params — and a
STRUCTURAL 15 deg/tick yaw-rate cap in the decoder (human max: 5.1; the cap
only ever binds where low-speed heading noise would thrash). Flip legality
enforced in decode (14-tick spacing) regardless of CP spacing. Search:
standard CMA-ES (full covariance, Jacobi eigen) over ~80-155 dims;
population evaluated on a persistent worker pool; rollouts are pure, so runs
are DETERMINISTIC at fixed rng regardless of thread count. ~30-45M ticks/s,
~60k generations/min at pop 64.

**Objective:** clean finish = scored ticks − 1e5 (eloss tie-break); jump
finish = scored + 5e3 (never competitive — the user's acceptance rule);
non-finisher = energy-aware distance + fitted waste penalty (wloss 6.5).

**Three measured traps, three physics fixes (each found by dumping the best
non-finisher and LOOKING, not guessing):**
1. Distance-to-CENTER read ~900u for lines brushing the platform edge (the
   human FINISHES at "dmin 890") → clamped-box distance to the hull-expanded
   landing footprint.
2. The LOB trap: with positional shaping, the optimizer's best line traded
   all speed for height and apexed 6u below the lip at SPEED 19 (2.9M evals
   of wall kisses). → the vertical term became the ENERGY SHORTFALL height
   max(0, gap − climb). First cut credited total v²/2g and created 882 u/s
   UNDER-platform speeders reading dmin 0 → only max(0,vz)²/2g is ballistic
   truth. The lob then reads its full shortfall, a rising fast kiss reads ~0.
3. Low-speed heading noise wrote 177 deg/tick yaw thrash into tapes (p95
   61.7) → the decoder rate cap. After: p95 3.9 on every output.

**Seeding and the prefix ladder:** control INVERSION (ground u = yaw-rate/arc,
air u = wish-angle map inverted against the core-replayed heading) projects a
tape onto the basis — but an open-loop projection of a 4-board chaotic line
diverges (the basis is closed-loop in HEADING, open-loop in POSITION; drift
compounds at each board). Measured boundary on basictest: projection of the
human tape finishes from prefix ≥335 (post-r4-board-hold), dies ≤330. The
critical windows are the few ticks of board entry/hold — 16-tick CPs cannot
express them; **8-tick CPs can** (still ≥14-tick flip spacing structurally).
So: `--seed-follow N` follows the seed tape verbatim to a checkpoint, the
spline owns the rest, and EROSION re-runs with the previous winner as seed at
ever-earlier N. Each rung re-solves ~one board window.

**RESULTS (the ladder, one evening, 45-90s per rung):**
- rung 335 (cp16): **298 scored clean** — FIRST SUB-HUMAN CLEAN TAPE EVER.
- rung 330 (cp8): 296. rung 320: 293 (impact 23!). rung 310: **292 scored
  (404 abs), landing impact 31 u/s, total clip loss 224, eloss 171k, yaw p95
  3.94, flips 1.65/s** — every quality number AT or BETTER than the human
  (319 / 189 impact / 391 clip / 259k eloss / p95 3.76).
- rung 300 (spline owns the whole r4 board): found clean 305 but not 292 at
  the fixed rng — deeper rungs need rng multi-start arms (deterministic runs
  retrace themselves; budget alone changes nothing).
- **SHIPPED: surf_basictest_cma_S292_A404_CLEAN_0814-0248.tas** (recordings).
  Trace-oracle queries written to solver\trace_queries.csv (1905) — pre-ship
  gate pending the in-game oracle run.

**Output labeling (user-directed):** all solve/smooth outputs now write
`<map>_<tag>_S<scored>_A<abs>_<CLEAN|JUMP>_<MMDD-HHMM>.tas` + a TEST IN GAME
card, and every produced/replayed tape prints the QUALITY SCAN: hard contacts
(clip >20 u/s, the user's definition of a hard landing — "pointing tangent"
doesn't make an approach clean, the clip loss does), air yaw-rate max/p95,
flips/s, ending class.

**Where this leaves the paradigm:** spline+CMA is a proven FINISHER/polisher —
it beat the human on the human's own route the first evening, with cleaner
contacts than the human. What it does NOT yet do: own multiple boards at once
(open-loop position drift; each board is a narrow basin CMA must thread). The
erosion ladder walks boards one at a time; full-line ownership wants either
rng-arm ladders (cheap, automatable tonight) or the board-anchored closed-loop
step: aim conditions at the NEXT face (tangent-arrival) as the generator
between spline segments — map geometry as DATA, not map rules.

## 2026-08-14 (later) — UNSEEDED TWO-LAYER SOLVER (`chain`)

User direction: seeding is not the mission ("the point is to do full map
without relying on existing inputs"); the skeleton idea approved,
"especially if it enables a true exhaustive search"; ML explicitly weighed
and benched (physics is cheap and exact here; one map of data; no-per-map-
training constraint — a learned proposal prior stays available for measured
bottlenecks only). Flip law re-confirmed structural before building: the
14-tick decode guard + 15 deg/tick yaw cap make 1-tick strafing
unrepresentable in ANY smooth/chain output.

**Architecture** (Source/Solver/SolverChain.{h,cpp}, `chain` command):
- LAYER 1 — SKELETONS: surfable faces read from the collision set (brush
  planes with 0 < n.z < walkable_z; live params, no map rules — basictest
  yields exactly the 4 ramp faces). Skeleton = face sequence (depth cap).
  Best-first tree over sequences, priority = junction value; every expanded
  node also attempts the END landing. At this scale the enumeration is
  EXHAUSTIVE within budget — the meaningful version of "exhaustive best
  line" (per-tick exhaustive is ~10^205 on this map; the archive already
  demonstrated dense enumeration of the wrong language).
- LAYER 2 — SEGMENTS: each edge is a smooth-spline CMA solve (segment mode
  in SolverSmooth): target = CONTACT with the next face; fitness = the
  user's compromise equation, w_tick*ticks − (V_board − wloss*eloss),
  V = KE + mu*g*z fitted mix. END edges use the clean-finish fitness (jump
  endings never assemble). Junction handoff = RIDE-ANCHORED: a contact tick
  ≥ touch_settle (16) after first touch, face lost for a full window =
  touch canceled. (Iteration history: first-touch junctions handed off
  corner clips → children starved; survive-only settle was gamed into
  single-graze ballistic "boards" — energy conservation makes tap-and-fly
  V-optimal — → contact-anchored ride requirement, the physical definition
  of a board.)
- Junction BEAM per (depth, face), POSITION-DIVERSE (beam_sep 96u): three
  copies of the same corner clip starve downstream; arrival diversity is
  the handoff's load-bearing lesson.
- ASSEMBLY: segments emit per-tick frames from exactly the parent's end
  state; concatenation replays identically from the anchor (verified by an
  authoritative re-replay before ranking). Best clean assembly gets a
  global smooth-CMA polish (machine streams invert near-losslessly).
- Zone-jump budget structural (single gene + in-zone guard); walker aborts
  (>60 grounded post-exit, >400 in-zone) are sim-budget bounds.

**Campaign log (10 iterations, every fix driven by a DUMPED TRAJECTORY, not
a guess; rng 1337 throughout):**
- v1-v3: tree enumerates correctly; children die. Fixed: first-touch
  junctions (corner clips) -> settle windows; survive-only settle gamed
  into tap-and-fly (energy conservation makes a single graze V-optimal) ->
  contact-anchored ride requirement.
- v4-v5: segments starved (max_restarts early-stop burned 8s budgets in
  ~1s on failure stalls) -> early-stop only after success. Guidance added;
  still dead: cold splines cannot HOLD a ride (mean-zero u slides off the
  face before release) -> structural ride hold + release gene.
- v6 dump: the "best board of ramp 2" was a 982 u/s BOTTOM-RIM graze at
  z=-68 OUTSIDE the face polygon, descending -> honest boards (settle must
  finalize INSIDE the face rect; junction V uses total energy, chain_mu=1,
  since riding makes PE/KE fungible).
- v7: FIRST FULL TOPOLOGY [7>8>9>11] chains unseeded - but links SMASH
  (V370k -> -1958k; ramp-3 boards at spd 349). Pure-pursuit guidance
  builds hard landings by construction (the user's exact words about the
  old solver) -> TANGENT LEAD-IN (aim displaced along the face plane,
  collapsing with range; normal component nulls before contact).
- v8: links carry real energy (ramp-4 board V+307k at spd 933; 1>2>3 at
  ~970 u/s). END attempts reach d49-d89 (energy-aware) - never land.
  Junctions all LOW on faces: with E conserved in flight, a single V pick
  collapses to "earliest cheapest board" - clause 3 (preserve reach) was
  missing -> HEIGHT BANDS (each edge solved in low/mid/high thirds of the
  face; each band a separate junction).
- v9-v10: high boards exist (ramp-1 z 175-180); beams expand diverse
  variants; END budget raised to 45s -> best END attempt **d31** - the
  under-lip regime again, budget shrinks the gap but does not convert.
  The 3>4 link still smashes (V -2256k class; the sharpest turn on the
  map). 73-113 segment solves / ~20B ticks per campaign.

**State:** the unseeded solver BUILDS real multi-ramp chains and is
exhaustive at the skeleton level; no unseeded clean finish yet. The two
measured gaps: (1) END conversion - import the FINISH-FLIGHT PROBE into
END-segment rollouts (release checks from near-goal states; the mechanism
that detonated discovery in the archive era) + z-margin in the landing aim
(aim past the near lip on a descending arc); (2) the 3>4 link - enumerate
exit SIDES (around the +-y face ends) the way heights are banded.

**Endgame round (v11-v12, user "continue"):**
- v11 (raised landing aim, land_margin 40): d31 -> d67; the aim plane was
  NOT the binding constraint. The DUMP told the real story: an END segment
  from [7>8] discovered a 2->4 transfer, boarded ramp 4 at 838 u/s - then
  rode STRAIGHT UP the face (homing pulls toward red = up-slope), bleeding
  487 u/s into the climb, exiting at 351 u/s with red unreachable, wall
  taps at z158 (98 below the lip). The archive era's exact dead end
  (~330 u/s face rides), rediscovered by a different searcher. The human
  carves ALONG the face and exits the side at 862.
- Root cause of dead ramp-4 FACE segments found in the same dump: the
  tangent lead-in DEGENERATES on frontal approaches (in-plane component
  ~0 -> lead vanishes -> pure-pursuit smash) -> MINIMUM-LEAD rule: frontal
  approaches synthesize the lead along the face's horizontal axis toward
  the board point's side (carve entry).
- v12 (carve entries): ramp-4 boards from depth-1 junctions now EXIST at
  908 u/s (previously dead) - but still lossy (V -487k..-1420k) and END
  attempts plateau d135-188. The remaining control gap is the RIDE itself:
  between board and release the only steering is the hold (into face);
  along-face travel direction is uncontrolled, so rides climb and bleed.

**Next mechanism (concrete): CARVE CONTROL** - during the ride hold, blend
a small along-face steering component toward a gene-chosen exit side, so
ramp-4 rides travel the face horizontally and release around the y-end at
speed with the landing homing engaged from a y-offset. Plus duck-pump
usage at the lip (genes exist). User intuition on the ramp-4 ride is
worth asking for before building further.

### Open items
- ~~Worker-pool parallelism~~ SHIPPED v2b. NUMA/affinity untested.
- **Chain endgame**: carve control (along-face ride steering + exit-side
  gene); then long campaigns + rng arms. END aim margin kept (40u).
- **Erosion campaign automation**: ladder loop (rng arms × rungs, carry-best,
  auto-reseed) as a lab mode — walk the prefix to 0 and the whole line is
  machine-owned. Candidate: `smooth --erode` (unbuilt).
- **Board-anchored closed-loop generator** (the structural fix for multi-board
  ownership): tangent-arrival targets on the next face between spline
  segments. BSP faces as data; no map-specific logic.
- **THE energy chain** — REFRAMED by this round: the chain is now solvable
  rung-by-rung (each erosion rung re-carves one segment under the clean-first
  objective). The unseeded full-chain remains open.
- Cold-start smooth (no seed): still dies mid-map (multimodal cliffs) — needs
  the closed-loop step or archive hybridization (archive finisher → invert →
  polish).
- Generator problem (mid-map unreachable-class) — same root as the chain.
- **StepMove port** (spine-edge ~3.6°) — last known movement residual.
- Mix-interleaving; optimize-stage thread underuse; audit verdict-column refresh.
- Archive-splice operator (graft archive-best prefixes onto finisher suffixes at
  shared cells) — next structural operator after v3a.
- Ramp-4 endgame structure (the whole remaining human gap); spine-jump
  discouragement mechanism (bias vs operator — user wants them discouraged).
- User's compromise-equation frame for the endgame (board next ramp with max
  energy / leave previous cleanly / preserve reach to the one after): the design
  brief for chained-segment value assessment — build AFTER the settle fix, tested
  in the segment lab.
- Robustness idea (from the 3u-margin miss): prefer winners that survive small
  state perturbations (verify-with-jitter before shipping) — candidate v4, unbuilt.
- Phase 2 v3 operators (themes-motivated): contact-anchored mutation (target board-
  entry knots), archive-splice, tighten-loop automation ({explore capped at best−1 →
  optimize} rounds — machinery proven, single capped round on rng 1337 found nothing).
- Board-entry deflection as audit column; segment-lab A/B before any selection weight
  (eloss may already price it).
- Segment-lab studies queued for the lab: per-stage fastest times (ramp2/3/4) as
  reference conditions for operator tests.
- Ground-duck lifecycle / duck-flag timing / hull-height data (dormant, non-blocking).
- Ground-duck lifecycle capture study (only if a route ever needs crouch-walking).
- Duck-flag completion timing (mismatch at 230 — flag-only today, no position error).
- Hot-stamina jump impulse tax not yet capture-exercised.
- hull_stand=62 read vs parity-proven 72 (needs a ceiling map or a better netvar read).
- Phase 2 optimizer: trim/splice/mutate CLEAN finishers (1887 and the seeded 430 both
  have obvious fat; optimizer must preserve cleanliness — mutations audited).
- Parallelism (worker pool) for discovery variance + real-map budgets.
- Edge-bevel modeling (load non-axial bevels as a SEPARATE validated trace mode?) would
  convert "avoid edges" into "predict edges" — needs its own engine ground-truth study
  (the v5.4 warning stands: naive inclusion broke proven lines).
- Cell-space audit, finish-tick display conventions, duck hull height / stamina
  fall-speed data, in-game archive viz in the Map Solve tab.

## Decisions log
- 2026-08-12: Era opened. Candidates A–G written pre-findings per user request.
- 2026-08-13: User answered all open questions + supplied the prior-attempt handoff
  (C:\Users\Connor\Documents\Player Controler\solver-feasibility\Source_TAS_Solver_Search_Handoff.md)
  and the testbed map. Final contract + handoff review + revised design v2 recorded above.
  Architecture: standalone-first solver core + in-game control/visualization/verify.
  Map facts independently verified (parse matches handoff Section 3.3 exactly).

## Phase 0 build log (2026-08-13)

**Built and verified this session** (all NEW files; Basehook untouched — zero regression risk):

- `Source/Solver/SolverMath.h` — engine-free Vec3 (X/Y/Z field names match the DLL's Vector).
- `Source/Solver/SolverWorld.{h,cpp}` — BSP collision loader + whole-map uniform grid +
  swept-hull trace. Mirrors the validated model: all non-bevel sides + EXACT axial lump
  bevels (non-axial bevels excluded), worldspawn-only via model-0 walk (v0=56/v1=32 leaf
  strides, divides-check), Minkowski hull expansion, CM_ClipBoxToBrush trace semantics
  (enterfrac from −1, unclamped tie-break, DIST_EPSILON 1/32). NEW vs DLL: **two expanded
  plane sets per brush (standing + ducked hull)** so the mover can crouch mid-flight;
  brush AABBs from the compiler's axial planes (exact, not winding-derived); spawn parsed
  from the entities lump.
- `Source/Solver/SolverMove.{h,cpp}` — full FullWalkMove tick: Duck() → StartGravity →
  CheckJumpButton → Friction → Walk/AirMove → CategorizePosition → FinishGravity.
  TryPlayerMove/CategorizePosition/ClipVelocity are verbatim ports of the DLL-validated
  code (**rule: validated DLL behavior wins over SDK recollection**). NEW (SDK-shaped,
  parity-arbitrated): ground friction/accelerate, WalkMove + StayOnGround, CheckJumpButton
  (vz = √(2g·57)≈302, `jump_finishgravity` toggle for the SDK's in-jump FinishGravity
  half-step, PreventBunnyJumping clamp gated on enablebunnyhopping), CS duck state machine
  (instant air duck, +18u FinishDuck lift = 72−54 hull delta, ground 400 ms transition,
  unduck fit-validated via static hull-overlap test, duck maxspeed ×0.34).
- `Source/Solver/SolverTape.{h,cpp}` — read-only .tas (STAS v1–3) reader, byte-for-byte
  mirror of RecordingStore (which stays the format owner).
- `Source/Solver/SolverLab.cpp` + `SolverLab.vcxproj` — console harness (v145, MaxSpeed,
  default FP — same flags as Basehook so float behavior matches when the DLL embeds the
  core). Commands: `mapinfo`, `replay` (events timeline + CSV dump + param overrides),
  `bench`. Output: `Output\SolverLab\SolverLab.exe`.

**Verification results:**
- `mapinfo surf_basictest.bsp` matches the independent python parse EXACTLY (12 world
  brushes, 108 planes, spawn, all AABBs incl. derived ramp tops 320/256/256/256; ramps
  carry 6 sides + 1 axial bevel).
- **Replay of the user's `basictest.tas` (629 frames): qualitative parity is STRONG.**
  The sim reproduces the whole run shape open-loop: prestrafe to 282 u/s → single jump
  tick 71 → startzone exit tick 109 @ 320.8 → boards ramp 1 tick 146 (leaves 817 u/s) →
  skips ramp 2 → ramp 3 tick 241 (841) → ramp 4 tick 316 (849) → **the user's landing
  crouch appears at tick 417 exactly where they crouched** → but leaves ramp 4 at only
  258 u/s and drops short of the red platform (crosses x=1216 at z≈113 vs top 256).
  Max speed 909.7. FINISH NOT reached.
- **Sensitivity sweep** (no-jump-fg / stopspeed 75 / maxspeed 260 / maxspeed 240): same
  route shape in all variants, none finish; ramp-4 entry point moves by tens of units
  across variants and the 56-tick terminal carve amplifies whatever drift exists at
  entry. stopspeed is irrelevant at prestrafe speeds (as predicted). Conclusion: an
  open-loop 372-tick replay through three boards cannot localize drift — DO NOT blind-fit
  parameters; get per-tick engine ground truth instead.

**Known gaps (core), logged not hidden:** StepMove step-up/down comparison deferred
(WalkMove slides only — flat platforms unaffected); CS:S stamina carried but its jump
factor not modeled (single-jump runs start at 0 where the factor is 1); triggers not yet
in the core (basictest has none); displacements still uncollided (basictest has none);
`old_buttons` starts 0 (a run that begins with jump already held would differ).

**Ground-truth loop built (same day):**
- **Map Solve tab** added to the editor (7th tab, between Server and Rendering; case 5 in
  the tab switch) — the designated home for EVERYTHING solver-related from here on.
  Phase 0 contents: sim status line, "Export sim states CSV" button (unlocks only when
  the sim is READY and not dirty), last-export path display, harness command crib.
- `ExportSimStates()` (TasEditor.cpp, next to DrawMapSolveTab): writes the ENGINE-exact
  per-tick states of the compiled run to `Documents\sourceTAS\solver\<project>_states.csv`.
  Columns `tick,x,y,z,vx,vy,vz,speed2d,ground,ducked,buttons,yaw` (ground/duck from
  SimState.flags FL_ONGROUND/FL_DUCKING; buttons+yaw from the compiled frames for input
  alignment); row 0 is the anchor as tick −1 (ground −1 = unknown). `SolverDir()` helper
  creates the folder. New prefs/persistence: none (session-only display).
- **`SolverLab diff <map.bsp> <run.tas> <states.csv>`**: replays the tape through the
  core and reports the FIRST divergence (thresholds 0.1/1/10 u), first ground/duck flag
  mismatches, max + final |dpos|, and a ±3-tick context dump (engine | core | delta,
  per axis) around the earliest interesting tick. Verifies input alignment first
  (buttons+yaw) and refuses to interpret state deltas past an input mismatch — a project
  whose frames differ from the .tas would otherwise masquerade as physics drift. Takes
  the same param overrides as replay (--maxspeed etc.) → parameter sweeps can be scored
  against ground truth offline. Replay's --csv now emits the identical 12-column format.
- **Identity self-test PASSED**: core replay CSV diffed against the core itself = 629
  ticks, every threshold "never", max |dpos| 0.000. The diff plumbing is sound.

**Next-session workflow (user in-game, one click + one command):**
1. Load the basictest project → Map Solve tab → "Export sim states CSV".
2. `SolverLab.exe diff <surf_basictest.bsp> <basictest.tas> Documents\sourceTAS\solver\basictest_states.csv`
3. Fix the core at the first divergent tick the data names; re-diff until the full run
   tracks. Prime suspects by phase: ticks 0–70 = ground friction/accelerate/maxspeed,
   tick 71 = jump impulse (+ the jump_finishgravity quirk), 72+ = air/board fidelity
   (expected near-exact — DLL-validated math), tick ~417 = duck constants.

## 2026-08-13 (later) — THE DIVERGENCE HUNT: full tick-exact parity achieved

The user exported engine states (545 rows) from the Map Solve tab; `SolverLab diff`
localized each divergence in turn. Every fix below is ENGINE-MEASURED from that ground
truth, applied one at a time, each verified by re-diff. Final result: **545/545 ticks
exact, max |dpos| 0.001 u (CSV rounding), zero flag mismatches** — prestrafe, jump,
boards 1-3-4, air-crouch, landing on red (FINISH tick 431, 6.465 s, 352.9 u/s), bhop off
red, second flight + landing, stamina walk-out to standstill. **The offline core is now
tick-exact against the engine on the whole reference run.**

Fixes, in discovery order (all in Source/Solver/SolverMove.{h,cpp}):
1. **sv_stopspeed = 75, maxspeed = 250 confirmed.** Fitted drop curve matches
   control=max(speed,75)·4·dt at every tick; fitted accel gain is 37.50 flat =
   10·0.015·250. (First guess 100 caused a −1.5/tick deficit; a maxspeed-260 trial
   "fixed" two ticks by two errors cancelling — the per-tick k/a decomposition fit
   exposed it. Sweep-fitting without decomposition is a trap; the fit script is
   scratchpad `fit_ground.py`.)
2. **Jump impulse: ADD onto post-StartGravity vz when standing, SET when ducked/ducking.**
   Jump tick ends at vz 284.0 = (−6+302)−6−6 (standing, t71); ducked bhop ends at 290.0 =
   302−6−6 (t432). Both confirm the SDK's in-CheckJumpButton FinishGravity quirk
   (jump_finishgravity stays true). No PreventBunnyJumping clamp active
   (sv_enablebunnyhopping 1).
3. **In-air duck/unduck origin shift = ±8.500 EXACTLY** (duck t417, unduck t443) — NOT
   the SDK hull delta 18. Kept as measured constant `duck_air_shift`; duck hull height
   for collision left 54 (only a ceiling map can test it — flagged).
4. **CategorizePosition does NOT snap the origin.** t479 lands with ground SET and the
   origin still 1.575 u above the surface; StayOnGround reaches the surface on the next
   WALK tick. The DLL model's in-probe snap was an end-of-flight conflation — removed
   from the core, DO NOT restore. (t431's landing hit the plane inside TryPlayerMove, so
   both stories agreed there.)
5. **Landing stamina (this build's variant).** Landing arms a timer (`stamina_land_ms
   612.5`); every WALK tick scales v.xy by (1 − stam·0.00019833) AFTER friction; the
   timer drains 15 ms/tick including airborne. Fit is exact across the whole 19-tick
   decay (order alternative "scale before friction" ruled out — blows up at low speed).
   Data also shows: **no jump-impulse stamina tax** (t432 bhop full height), and bhop
   ticks bypass the scale entirely (CheckJumpButton leaves the ground branch before
   WalkMove) — which is why chained hops keep speed on this build.
6. Finish predicate uses the hull-EXPANDED footprint (engine grounds on red at origin
   x 1213.69 vs the box's 1216).

**Undetermined (needs targeted data, logged not guessed):**
- Stamina initial value's fall-speed dependence — one landing observed (constant fits;
  CSGO-style landcost·vz also roughly fits). A second landing at a different fall speed
  discriminates.
- Duck hull height (ceiling clearance) — untestable on this map.
- Ground duck transition timing (400 ms path) — this run only air-ducks.

**Dead ends (do NOT resurrect without new evidence):** SDK-classic stamina constants
(1052.6 ms / 0.00019); SDK 18u air-duck shift; in-CategorizePosition origin snap;
blind parameter sweeps as a fitting method.

## Benchmarks (measured only — never assumed)
| date | machine/map | metric | value | notes |
|------|-------------|--------|-------|-------|
| 2026-08-12 | (their machine, C# lab) | C# oracle throughput, 1 thread | 4,817,961 ticks/s | EXTERNAL reference from handoff — our C++ floor to beat |
| 2026-08-13 | user machine, surf_basictest | solver core throughput, 1 thread | **15,258,605–15,260,745 ticks/s** | SolverLab bench, 2M and 10M ticks, full collision + grid; 3.2× the C# floor; ~150M/s at 10 workers |
| 2026-08-13 | user machine, surf_basictest | full-run replay (629 ticks) | instant (<1 ms scale) | replay wall time dominated by process start |
| — | — | engine RequestSim throughput (ticks/s) | TBD | |
| 2026-08-13 | surf_basictest, basictest.tas | **fast-vs-engine divergence, 545 ticks** | **max 0.001 u, all flags exact** | after the divergence-hunt fixes; 0.001 = CSV %.4f/%.3f rounding floor |

## Experiments log
Template: `date | hypothesis | setup | result | verdict (+risk ID)`

## Dead ends (do NOT retry without new evidence)
(none yet — segment-solver era dead ends live in memory `solver-roadmap-1-1`: geometric
fragility gates, SDK-adjacency vtable guesses, board-coupling gen-1.)

## Findings
(none yet)
