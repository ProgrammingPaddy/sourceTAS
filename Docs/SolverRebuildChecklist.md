# Solver Rebuild — Build Plan & Living Checklist

**This is the status document.** Design and testimony live in
`Docs/SolverRebuild.md`; the narrative log lives in
`Docs/FullMapSolver.md`; this file tracks WHAT IS BUILT and WHAT IS
NEXT, and is updated every working session (change log at the bottom).

Legend: `[x]` done+validated · `[~]` in progress · `[ ]` not started ·
`(!)` blocked/depends · each milestone ends with its ACCEPTANCE GATE —
a measurable pass/fail, never a vibe.

**NOW →** M1.5 THE LEDGER.

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
- [ ] 1.5 THE LEDGER: per-transfer, per-phase regret readout (board
      loss, approach loss, air-gain shortfall, conversion shortfall,
      time regret) computable for ANY simulated line.
      GATE: ledger of the 292 tape line matches hand analysis on 3
      transfers; ledger of a deliberately bad line localizes the
      planted losses.

## M2 — Route search (stage 2)

- [ ] 2.1 Analytic edge bounds: energy-in → min-ticks + exit-energy
      bound per transfer edge (from 1.1 + 1.2).
- [ ] 2.2 Beam/DP over feature sequences: skips first-class, start-zone
      plan (prestrafe + single jump + first-board tradeoff), end-zone
      terminal.
- [ ] 2.3 Feasibility pruning via envelopes; jump edges only as
      exceptional moves (legality per SolverRebuild §1).
      GATE (M2): on basictest, unseeded, the top-10 routes include the
      human route shape and the solved12 shape; route enumeration under
      5 seconds.

## M3 — Transfer refinement & assembly (stage 3)

- [ ] 3.1 Rolling-window chained refinement along a route (2→4
      features): each flick optimized against the next board window and
      the ramp after; entry states propagated; infeasible → prune.
- [ ] 3.2 Assembly into a full run on the exact engine; .tas export.
      GATE (M3): unseeded finisher on basictest in < 2 minutes wall
      clock; its ledger strictly dominates the old solver's best line
      (fewer board losses, less approach regret, fewer ticks).

## M4 — Polish & anytime behavior (stage 4)

- [ ] 4.1 CMA-ES residual polish on yaw splines (boundary-locked,
      seeded from 3.1, never random).
- [ ] 4.2 Ledger-directed re-solve: budget flows to the worst transfer.
- [ ] 4.3 Anytime toggles: beam width, window depth, polish rounds;
      2-minute vs 20-minute dial demonstrated.
      GATE (M4): **beat the human tape's zone time on basictest,
      unseeded.** The 2-min setting lands within ~5 ticks of the
      20-min setting.

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
