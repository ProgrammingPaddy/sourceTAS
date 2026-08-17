# Solver Rebuild — Build Plan & Living Checklist

**This is the status document.** Design and testimony live in
`Docs/SolverRebuild.md`; the narrative log lives in
`Docs/FullMapSolver.md`; this file tracks WHAT IS BUILT and WHAT IS
NEXT, and is updated every working session (change log at the bottom).

Legend: `[x]` done+validated · `[~]` in progress · `[ ]` not started ·
`(!)` blocked/depends · each milestone ends with its ACCEPTANCE GATE —
a measurable pass/fail, never a vibe.

**NOW →** M3 transfer refinement & assembly (IN PROGRESS — see 3.1/3.2
status and the 2026-08-16 changelog tail for exactly where it stands).

---

## M0 — Foundations

- [x] 0.1 Design of record + uncompressed expert knowledge base
      (`SolverRebuild.md`, commit c6de8a8)
- [x] 0.2 Strafe law: closed form (`SolverStrafe.h`) + `strafelaw`
      prover — 1,120 states vs certified MoveTick, float-ULP exact
      (max 2.0 in speed² of 11.5M; 8e-9 rad) (d4f6402)
- [x] 0.3 Feature extractor v0 + `routegraph` command — brush-side
      polygons, plane/area/extents/downhill, candidate edges (d4f6402)
- [x] 0.4 Face coverage on basictest — GATE PASSED via the new
      `facecover` command: both certified tapes replayed through the
      exact sim, **483/483 surf contacts map to extracted faces, 0
      missing**. (The spine worry was unfounded: its ridden moments are
      walkable-top GROUND contacts, not surf.) `facecover` stays as the
      standing per-map gate.
- [x] 0.5 Zone anchoring: `Route::AnchorZones` — start = brush under
      the anchor origin; end = --end-brush or DERIVED by replaying the
      anchor tape to its finish (zones are plugin-side on real servers,
      so a human trace is the honest zone source — the expert demo
      traces serve this role per map). basictest: start idx 6, 3
      candidate first boards; end id 10, 2 feeder faces.
- [x] 0.6 Strafe alternation rate limit: **6 direction changes per
      second** (user, 2026-08-16: "more than 6 strafes per second is
      extremely rare") = min ~11 ticks between alternations at
      66.67tps. `MoveParams::strafe_rate_max`, params-file loadable;
      becomes the yaw-spline knot cap in M1.3.

## M0.7 — DLL crash hardening (user-blocking, added 2026-08-16)

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

## M1 — Transfer primitives & the ledger

- [x] 1.1 Reachability envelope (`SolverEnvelope.h` + `envelope` gate
      command): EXACT discrete ballistic z (half-gravity structure:
      z(n) = z0 + n·dt·vz0 − g·dt²·n²/2), speed bound s(n)² ≤ s0²+900n,
      distance bound (gain-optimal straight line), ZWindow + CanReach
      edge gate. GATE PASSED: 15/15 tape airborne stretches contained
      (z/speed/dist all 0 violations) + 500/500 random-control
      falsification trials stay inside, 0 escapes. Laws learned: the
      envelope bounds the AIR PHASE ONLY — measure at the last airborne
      tick, never after the contact tick (the board clip converts vz
      into horizontal speed = M1.2's business); z sits in a duck-offset
      band {−8.5, 0, +8.5} RELATIVE TO ENTRY DUCK STATE, not on the
      ballistic point.
- [x] 1.2 Board window per face (`SolverBoard.h` + `boardwin` gate):
      closed-form clip physics — dot = v1·n sweeps [vz·nz − s·h,
      vz·nz + s·h] over aim; speed² loss = dot² EXACTLY; MinApproachDot
      = the tangency law (0 ⇔ s·h ≥ |vz·nz|, else unavoidable loss);
      AimCone(cap); polygon region test with hull-center slack 43u
      (=|(16,16,36)|, geometric). TickEvents grew contact_pos/
      contact_vel (pure instrumentation); Fn::ClipVelocity exposed.
      GATE PASSED first run: 13/13 tape boards approaching + in-region
      (max edge −0.4u) + min-law held + clip model EXACT vs the
      mirror's own measured loss (worst 0.0000 u/s); spot check 827
      single-plane strikes across all faces × aim spectrum, 0 cone
      violations, closed-form zero-input clip-tick prediction matches
      MoveTick exactly (StartGravity → clip → FinishGravity decomposition
      confirmed). FINDING: the certified tapes' worst board = |dot|
      432 u/s = 38.5% of speed² lost — the OLD solver's board quality
      quantified (these tapes are parity-certified, not optimality-
      certified). The expert cap must come from the demo traces (M1.5
      ledger), NOT from these tapes.
- [x] 1.3 Air-phase primitive (`SolverAir.h/.cpp` + `airsolve` gate):
      target-heading spline flown on the exact engine by a law-derived
      controller — per tick it lands the spline heading exactly via the
      certified turn-curve inversion (cosa ∈ [0, cap/v]: full-turn-full-
      gain → no-turn-no-gain), and beyond the perp rate engages the
      BRAKING TURN (cosa < 0: budget 562.5 >> cap 30 buys ~32°/tick at
      850 u/s for ~114 u/s — testimony's "eat the energy in the turn",
      now a controller capability whose cost the optimizer owns via the
      spline slope). Strafe-side flips rate-limited by construction
      (min 12 ticks; blocked flip with small error weaves, large error
      coasts). Search: 5 unseeded initial families (linear/late-turn/
      pure-pursuit/mirrored-tangent/outward-bump) + BASIN HOPPING
      (deterministic jitter restarts — plain coordinate descent
      converged at ~200 evals and left the rest of any budget unspent)
      + polish hops around the global best. Spline spans the expected
      flight (aim_tick), not the sim cap — late knots must be live
      parameters. GATE PASSED: 12/12 tape transfers reproduced
      unseeded (entry state + landing window only, never tape
      controls), 1.0s wall for the whole suite at 3000 evals/transfer.
      Quality: the primitive beats the tape's board on 10/12 —
      e.g. dot −50 vs tape −377, −205 vs −432, two near-perfect
      tangent arrivals (−1.8, −0.6).
- [x] 1.4 Carve primitive (`SolverCarve.h/.cpp` + `carve` gate + the
      shared `SolverSteer.h` controller): on-face rides driven by the
      SAME certified controller as the air phase, with two extra
      dials — an EFFORT channel (per-knot duty-cycled coasting: the
      expert's speed control; slow rides are effort choices, not
      heading choices) and the DUCK-OFF exit move (press duck while
      riding: the +8.5 air-duck shift pops the hull off the face,
      keeping the climb velocity — how solved12's crest exit works,
      found by `ridedump` on the tape's final tick: b1028 d1). Carve
      ENERGY LAW verified against the engine: v²_exit + Σdot² = v²_entry
      + 2g·drop (+ wish work ≤ 900/tick), with the DERIVED discrete
      cross-term bound Σ g·dt·nz·|dot| (a clip at fraction f inside a
      half-gravity tick shifts E by g·dt·(2f−1)·nz·dot — measured law,
      not a fitted tolerance). Exit spec = full VELOCITY VECTOR (crest
      launches have mostly-vertical velocity; horizontal heading alone
      is ill-conditioned). GATE PASSED: 10/10 tape carves reproduced
      unseeded (worst dv 16.4 u/s, seven rows < 8), manifold 80/80
      zero-input identity + 80/80 strafing law bound, 6.4s wall.
- [x] 1.5 THE LEDGER (`SolverLedger.h/.cpp` + `ledger`/`ledgergate`/
      `ledger-trace`): exact event-sourced phase decomposition
      (GROUND/AIR/RIDE) of any control line — air-gain shortfall vs
      the 900/tick law, board loss² + fraction + TANGENCY REGRET
      (dot² − min² at the actual arrival), ride clip dissipation,
      gravity conversion, wish work via energy closure (self-auditing
      within the derived cross bound + a float-accumulation allowance
      ~ULP(v²)/op). GATE PASSED: closure 0/9 bad on the 292 tape;
      planted 15-tick coast localized (prior phases bit-identical,
      delta +65.3k = 13.5k direct theft + 40.9k of misaligned-yaw
      braking the ledger also priced). THE INDICTMENT of the old
      line, now in numbers: 329,477 u²/s² dissipated on clips +
      77,634 air shortfall in one 405-tick run; boards at −206
      (regret 42.6k, tangency 0 was available!) and −142 (regret
      20.3k, ditto). `ledger-trace` audits the 7 expert demo CSVs
      (energy-model-break events per snapshot pair) — CAVEAT: on the
      big maps those events mix real clips with teleports/boosters;
      classification needs map geometry (deferred to M5.1). The
      apples-to-apples expert comparison happens on shared maps.

## M2 — Route search (stage 2)

- [x] 2.1-2.3 Route search (`SolverRouteSearch.h/.cpp` + `routesgate`):
      best-first enumeration over feature sequences under THE POTENTIAL
      LEDGER — total energy obeys E' = E + 900·ticks + 2g·Δz for
      flights and rides alike, anchored per node (anchors telescope;
      cycles net exactly their wish work, killing the corner-bounce
      energy exploit that broke two earlier attempts). Edges = M1
      closed forms: ballistic z-window × gain-law distance coverage
      from the departure anchor; ride traversal priced arrival→
      departure anchor at ledger speed; the END edge allows LANDING
      SHORT + RUNNING the remainder (the human line's final leg).
      START = measured prestrafe ceiling (certified-sim circle-strafe
      probe ×1.15, no fitted constant) + the single legal jump folded
      into E. Skips are first-class (the search REJECTS start→1/2/3
      as unreachable without the face-0 board — correct physics).
      Ranking = best lb per DISTINCT BASE SHAPE (first-occurrence face
      order; cycle variants and multi-taps collapse — the pool stage 3
      consumes). GATE PASSED: 60 raw routes → 10 shapes in 0.00s;
      the pool contains the HUMAN shape [0 2 3], solved12's [0 1 2 3]
      (rank 7), the old solver's [0 2], and the speculative one-ride
      [0] (rank 4 — no real run validates it; worth probing in M4).
      CORRECTION 2026-08-16: a session mislabeled Run 21.tas as "the
      human line" (one ride, 71k dissipation, pit finish) and briefly
      made it the benchmark. Run 21 is a DEAD TAPE — it rides face 0
      into the map FLOOR (z −1056 = brush 1's top) and never finishes;
      its header says surf_basictest only because it was recorded
      in-game there. All conclusions drawn from it are PURGED. The
      REAL human run is `basictest.tas` (user-confirmed; see M4).

## M3 — Transfer refinement & assembly (stage 3)

- [~] 3.1/3.2 Assembler (`SolverAssemble.h/.cpp` + `msolvegate`) — IN
      PROGRESS. Built and working: shape iteration from the M2 pool;
      START plan search scored by the RESULTING BOARD via probe air
      solves (launch-speed-toward-the-face was the head-on plunge
      setup: dot −458 → −59.8 when fixed); region-mode air targets
      (miss gradient to the nearest face point — point pursuit fights
      tangency); graze-through flights (a non-target clip continues
      the flight; ending it killed chains 37u short); TAP TRANSFERS
      (striking the next leg's face IS the transfer — the 3-tick
      clean-air exit is impossible between adjoining valley faces);
      the UNIFIED TRANSFER primitive (tap mode: the ride flows through
      exit + flight and is scored by the next strike — the design's
      stage-3 unit); zone proxy + ZoneTick + ledger comparison + .tas
      export all wired.
      SESSION 2026-08-16b (tapprobe-driven, every step measured):
      the unified transfer NOW SOLVES the human's own transfers -
      `tapprobe` (new isolation instrument: replay any tape to a
      tick, run the tap/zone solve from that exact state) showed the
      0→2 valley transfer unseeded at dot −162.8 @ 887 u/s (human:
      −206.4 @ 874 on the same face pair) and the face-3→zone ENDING
      in 101 ticks (human: 123 from the same entry). What it took,
      in order (each verified by the probe): (1) tap target = the
      next face's BOARD REGION (aim_region math, not corner points);
      (2) FRONT-SIDE-ONLY miss gradient (behind-plane closeness is
      not approach — kills the corridor/under-dive traps); (3)
      graze-through in the flight section (only the tap face ends a
      tap; zone mode grazes everything); (4) ballistic REACH-
      SHORTFALL term at separation (M1.1 closed form: prices
      "separate higher/ascending"); (5) S-CARVE families (dive then
      up — the measured human shape: final ride heading +8°, exit
      ascending +128 vz off the face edge); (6) DUAL SPLINE DOMAINS
      in SolveCarve (est and est/2; zone est×1.7 — the M1.3 dead-
      knot lesson recurs because the ride doubles the speed); (7)
      6 knots for two-phase transfers; (8) 12k carve evals (probe-
      measured: −395@4k → −163@12k); (9) ZONE MODE = the last
      transfer ends by entering the end volume, scored by arrival
      tick (the objective itself), no more FlyToZone dependence;
      (10) board ALTERNATIVES (SolveTransfer returns top-K diverse
      hits; the leg composes each with its carve — a dot-minimal
      board measurably set up −400 taps where a harder board set up
      −110); (11) rolling-context NEXT_COST (exact climb law
      v−√(v²−2g·sh) toward the leg-after target).
      In-assembler best so far: [0 2] leg 0 at −110.3 @ 877.
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
      CABLE/RED, reflectivity 0.511/0.002/0.002 — the raised platform
      z 192..256). The assembler's InZone target (brush 10) was
      already correct. Texture identity now loads from the BSP
      (texinfo→texdata→string lumps, per-side `texd` +
      `World::texnames/texreflect`; `mapinfo` prints per-brush
      textures) so zone identification is map data, not lore.
      `tapeinfo` (new) prints every tape's map/frames/anchor —
      the provenance check that would have caught the Run 21 mixup.

## M4 — Polish & anytime behavior (stage 4)

- [ ] 4.1 CMA-ES residual polish on yaw splines (boundary-locked,
      seeded from 3.1, never random).
- [ ] 4.2 Ledger-directed re-solve: budget flows to the worst transfer.
- [ ] 4.3 Anytime toggles: beam width, window depth, polish rounds;
      2-minute vs 20-minute dial demonstrated.
      GATE (M4): **beat the human tape's zone time on basictest,
      unseeded.** The 2-min setting lands within ~5 ticks of the
      20-min setting.
      THE MISSION IS THE SOLVER, NOT A MAP (user, repeatedly): the
      gate is generic — on any map with a human/reference run, the
      unseeded solve beats it. Per-map numbers below are VALIDATION
      INSTANCES only; nothing in the solver may reference them.
      basictest instance (measured 2026-08-16, user-confirmed tape):
      `basictest.tas` = the human run. Zone clock **319 ticks
      (4.785 s)** from start-zone exit (t112) to grounding on brush
      10 (t431). Route shape [0 2 3] (skips face 1). Max 915.9 u/s.
      Its ledger: boards −115.1 (regret 5.4k), −206.4 (regret 42.6k,
      tangency 0 available), −142.4 (regret 20.3k); whole-tape
      dissipation 343.9k + air shortfall 138.7k. The old solver's
      best line (cma_S292) scored 292 ticks — already faster than
      the human — so this instance's bar is < 292 ticks unseeded,
      with the human's 68k board regret as energy headroom.

## M5 — Generalization

- [ ] 5.1 Second linear map, brush-geometry only, end-to-end unseeded.
      GATE: finisher + clean ledger, no code changes specific to the
      map.
- [ ] 5.2 (!) Displacement collision in the world model — parity-side
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

## M6 — In-game validation (batched; respect the relaunch fatigue)

- [ ] 6.1 Export candidate lines and validate IN ONE SESSION per batch
      (playback capture, diff 0.000u required).
- [ ] 6.2 Only then: showcase-quality review against the aesthetic
      rate-limit rule.

## Cross-cutting rules

- Every module validates against the certified engine or the tapes
  BEFORE anything builds on it (the strafelaw pattern: prover first).
- No fitted constants in scoring — bounds derive from the engine model;
  the ledger measures regret, not vibes.
- No per-map tuning anywhere. If a map needs special handling, the
  design is wrong.
- Jumps: exceptional route edges only; spine-bhop hard-filtered.

---

## Change log

- 2026-08-16: Created. M0.1–0.3 done (design doc; strafe law proven
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
  evidence: (1) first run had 6 "speed violations" up to +57u — the
  measurement included the contact tick, where the board clip converts
  vz into horizontal speed; envelope now measured at the last airborne
  tick. (2) 476 falsification escapes at exactly 8.5002 — the air-duck
  origin shift; z checks now use the duck band, RELATIVE TO ENTRY DUCK
  STATE {−8.5, 0, +8.5} (a stretch entering ducked that unducks
  mid-air sits at −8.5). Final: 15/15 contained, 500/500 falsification
  clean. Battery still 14/15 (unduck_face 1.45u = the documented
  parity-era residual, untouched). NOW = M1.2 board windows.
- 2026-08-16 (later): M1.2 GATE PASSED first run (boardwin: 13/13 tape
  boards contained, clip model + zero-input tick decomposition both
  EXACT vs the engine mirror; 827 synthetic strikes, 0 violations).
  Board physics is now closed-form: loss² = (v1·n)², min-loss law
  max(0, |vz|·nz − s·h)², aim cone per cap. KEY FINDING for the
  mission: the old solver's tapes contain a 38.5%-of-speed² board —
  the "needlessly lost energy on boards" the user diagnosed, now a
  number the ledger can chase. NOW = M1.3 yaw-spline air primitive.
- 2026-08-16 (later): M1.3 GATE PASSED, 12/12 unseeded in 1.0s. Four
  iteration rounds, each from row evidence: (1) spline domain must be
  the expected flight, not the sim cap (dead-knot bug); (2) arrival
  tick needs a hard window (tick_tol) or the search grazes back at
  tick 1; (3) the controller NEEDED the braking turn (cosa < 0) — the
  perp-only strafe family cannot soften hard boards; adding it turned
  6 rows at once and IS the flick/eat-energy-in-the-turn mechanic;
  (4) plain coordinate descent converges at ~200 evals regardless of
  budget (identical output at 900 vs 3000) — basin hopping with
  deterministic jitter actually spends the budget and closed the last
  2 rows. Primitive beats the tape's board loss on 10/12 transfers.
  NOW = M1.4 carve primitive.
- 2026-08-16 (later): M1.4 GATE PASSED (10/10 carves + 160/160
  manifold). Six iteration rounds, each evidence-driven: (1) the carve
  energy identity needs the DERIVED half-gravity cross-term bound, not
  a tolerance; (2) slow rides demanded the EFFORT channel; (3) crest
  launches demanded the full exit-velocity-vector spec (heading of a
  near-vertical launch is noise); (4) the 1e6 no-exit wall froze two
  rows across three fix rounds — a stall NEAR the aim must outscore an
  exit FAR from it (comparable scores restored the gradient and one
  row snapped to dv 2.2); (5) budget was NOT the lever (20k evals =
  same failure); (6) `ridedump` on the last stubborn ride revealed the
  DUCK-OFF exit (duck press on the final tick separates the hull with
  climb velocity intact) — added as a searched genome dimension and
  the row closed at dv 16.4. The primitive kit now expresses: tangent
  boards, braking flicks, weaves, coasting, crest launches, duck-offs.
  NOW = M1.5 the ledger.
- 2026-08-16 (later): M1.5 GATE PASSED — M1 COMPLETE. Ledger lessons:
  closure tolerance must be law + derived cross bound + float
  allowance (first run failed by 66 on a flat 60); sabotage damage is
  NOT capped by the stolen window — misaligned downstream yaws brake
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
  corner bounces harvest fake energy every revisit — replaced by THE
  POTENTIAL LEDGER (E' = E + 900·ticks + 2g·Δz uniformly, telescoping
  anchors, cycle-proof); (3) vertex-anchored credits inverted on one
  edge (s_ub 10) — same ledger fix. Plus: the END edge needed the
  fly-then-RUN leg, and ranking needed base-shape dedup. (A Run 21
  "human line" claim from this entry is RETRACTED — see the next
  entry.) NOW = M3: chain SolveTransfer/SolveCarve along the shape
  pool, assemble full runs, export .tas; gate = unseeded finisher
  < 2 min whose ledger dominates the old line.
- 2026-08-16 (session end): M3 assembler ~80% — ten evidence-driven
  iterations, findings baked into code and the 3.1 status above. The
  transfer physics of basictest measured from the certified line
  (ridedump): valley-hop transfers separate MID-FACE at +55..+100
  above zmin, cross flat-or-ascending (in-plane heading clamped out
  of the downhill half), and board the next base at z ≈ −10; Run 21's
  crest exit is a DUCK-OFF; the start jump must be chosen by its
  BOARD, not its launch speed. Two chained boards at −59.8/−10.1
  prove the primitives compose. Next concrete steps: (1) make the
  unified tap transfer's guidance walk the ride through the measured
  band before separation (the strike gradient alone lets rides dive
  and ground in the valley); (2) once a full shape chains, FlyToZone
  from the last face finishes the run; (3) resolve the ZONE QUESTION
  with the user (platform vs pit) before M4. Wall per full attempt
  ~25-30s — well under the 2-min gate budget.
- 2026-08-16 (corrections, user in the loop): (a) Run 21 RETRACTED as
  any kind of benchmark — it is a dead in-game tape that rides face 0
  into the floor; the REAL human run is `basictest.tas` (319 zone
  ticks; ledger in M4 notes). (b) Zones are marked BY TEXTURE (user:
  red = end zone; basictest: green CABLE/GREEN on start brush 6, red
  CABLE/RED on end brush 10). Added texture identity to the world
  loader (texinfo→texdata→string lumps), per-brush textures in
  `mapinfo`, and `tapeinfo` (per-tape map/frames/anchor provenance).
  (c) REFRAME, user's words: "we are not solving a map, we are
  building a solver to solve every map" — so zone identification
  becomes a solver capability (detect start/end from texture
  dominance in map data; trigger-entity zones join in M5), and all
  per-map numbers in this doc are validation instances, never inputs.
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
  trajectory (every 3rd tick) into a per-stage sink — all best-so-far
  improvements kept + a uniform reservoir of the rest — and
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
  kHullCenterSlack — the raw polygon boundary is a MARGINAL strike;
  20-50u edge-skim misses were killing every chain: landings now
  stick); (13) PER-LEG BEAM — SolveCarve returns top-K diverse
  successes (mirroring the air alts), every leg composes (board alt
  x carve alt) candidates and prices the top 3 with a DEPTH-2 probe
  of the next leg from their landing; the beam found a 1->2 transfer
  at dot -64 @ 898 u/s (human: -206 there). REMAINING, sharply
  isolated by the beam prints: the 2->3 transfer (all candidates
  strike face 3 low-south at dead speed vs the human's ascending
  north-mid board at -8) and face-2 endings (likely genuinely
  infeasible — the human uses face 3 as the elevator). The msolve
  report shows both patterns as density; expert eyes requested.
  Gates green after every step (airsolve 12/12, carve 10/10).
- 2026-08-16 (session 2b): the transfer-capability campaign — eleven
  measured steps (list in 3.1) driven by the new `tapprobe` isolation
  instrument (the FUNCPROBE method at transfer granularity: replay a
  tape to a tick, solve from that exact state, dump the winner's
  trajectory). Probe verdicts: 0→2 unseeded −162.8 @ 887 (human −206
  @ 874); face-3→zone ending 101 ticks (human 123). Killed traps, in
  the order the trajectories exposed them: corner-point targets → the
  corridor thread (behind-plane miss counted as progress) → grazes
  ending flights → descending separations (ballistic shortfall term)
  → dead spline knots (dual domains, again) → arrival shaping (6
  knots, 12k evals) → the ending (zone mode: arrival tick IS the
  score) → myopic boards (top-K diverse alts composed with their
  carve). `DetectEndZone` now feeds routegraph/routesgate/msolvegate
  from texture marks with reference-replay fallback. Remaining: chain
  consistency (landings that strand the NEXT leg — depth-2 rolling-
  window composition is the next move, all primitives proven).
  Gates re-run green after every step: airsolve 12/12, carve 10/10.
