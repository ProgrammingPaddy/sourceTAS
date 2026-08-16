# Solver Rebuild — Build Plan & Living Checklist

**This is the status document.** Design and testimony live in
`Docs/SolverRebuild.md`; the narrative log lives in
`Docs/FullMapSolver.md`; this file tracks WHAT IS BUILT and WHAT IS
NEXT, and is updated every working session (change log at the bottom).

Legend: `[x]` done+validated · `[~]` in progress · `[ ]` not started ·
`(!)` blocked/depends · each milestone ends with its ACCEPTANCE GATE —
a measurable pass/fail, never a vibe.

**NOW →** M1.1 transfer envelopes (M0.6 rate-limit constant awaits the
user's number).

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

- [ ] 1.1 Reachability envelope: from an exit state (pos, vel), the
      cone of reachable states at time t under gravity + the proven
      gain law (outer bound: perpendicular gain every tick; inner:
      ballistic). Used to gate graph edges and prune routes.
      GATE: envelope CONTAINS every transfer observed in the two tapes
      and excludes states beyond the analytic bound (sim-sampled
      falsification, 10k random control sequences stay inside).
- [ ] 1.2 Board window per face: region × velocity cone with clip loss
      below threshold (tangent-dominant per testimony §2.1), including
      side/mid-face boards.
      GATE: every tape board lands inside its face's window; window
      edge cases spot-checked against the exact engine.
- [ ] 1.3 Air-phase primitive: yaw-spline boundary-value solver (entry
      state → target board window), knots capped by 0.6; solved on the
      exact engine.
      GATE: reproduces each tape transfer's board within small
      tick/loss deltas WITHOUT seeing the tape's controls.
- [ ] 1.4 Carve primitive: on-face entry → exit manifold with time +
      energy tags (conversion along downhill axis, flick setup).
      GATE: reproduces tape carves' exits; manifold matches sim
      sampling on 3 faces.
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
- [~] 5.3 Expert demo ingestion. Demos received (7 cut runs: axiom,
      cement, facility ×2 incl. m@'s faster bonus, fiellu_ksf, huh,
      jumble). File parsing is DEAD (TV-style demos: democmdinfo carries
      no camera - verified all-zero origins across 300k packets); the
      chosen path lets THE ENGINE decode them: DLL "Capture demo traces"
      button plays solver\demo_queue.txt unattended and records the
      spectated target's SNAPSHOT positions (m_flSimulationTime-gated -
      no interpolation smear) to solver\demo_traces\*.csv. Demos staged
      in cstrike\demos\, queue written, DLL built. NEEDS: one game
      session, one click, ~2-3 min unattended.
      Then: the expert-ledger comparison once M1.5 exists.
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
