# Solver Rebuild â€” Build Plan & Living Checklist

**This is the status document.** Design and testimony live in
`Docs/SolverRebuild.md`; the narrative log lives in
`Docs/FullMapSolver.md`; this file tracks WHAT IS BUILT and WHAT IS
NEXT, and is updated every working session (change log at the bottom).

Legend: `[x]` done+validated Â· `[~]` in progress Â· `[ ]` not started Â·
`(!)` blocked/depends Â· each milestone ends with its ACCEPTANCE GATE â€”
a measurable pass/fail, never a vibe.

**NOW â†’** M3 transfer refinement & assembly (IN PROGRESS â€” see 3.1/3.2
status and the 2026-08-16 changelog tail for exactly where it stands).

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
