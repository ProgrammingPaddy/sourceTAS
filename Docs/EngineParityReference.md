# Engine Parity Reference — the complete resume point

Written 2026-08-15 at the close of the parity era (last commit `d09eea4`),
for the day this work has to be reopened. Everything here was MEASURED or
DISASSEMBLED; nothing is recollection. The companion narrative with full
evidence chains is `Docs/FullMapSolver.md` (2026-08-15 entries) and
`Docs/FuncProbe.md` (the original method spec). Binary RVAs are for the
x64 CS:S build present on this machine on 2026-08-15 — a game update
invalidates every RVA (the context latch will fail closed and tell you).

---

## 1. State at close

| surface | score |
|---|---|
| Trace/world vs every engine answer ever collected (75,003 box + 38,286 box + 6,624 sweep) | **EXACT — zero mismatches** |
| CategorizePosition (incl. quadrant ground) | 20,072/20,072 **PERFECT** |
| CheckJumpButton | 20,000/20,000 **PERFECT** |
| HandleDuckingSpeedCrop | 5,000/5,000 **PERFECT** |
| ReduceTimers | 100/100 **PERFECT** |
| FinishUnDuck | 4,996/5,000 |
| Duck / CanUnduck / FinishDuck | 24,665/26,381 · 9,696/11,381 · 8,832/11,381 — every miss at UNREACHABLE torture states (inside solid / outside the sealed map); grounded CanUnduck proven exact by engine self-consistency (3,599 rows, 0 contradictions) |
| Tapes (292 + solved12, full length) | 0.000u / 0.000u |
| Battery | 14/15 — `unduck_face` 1.450u open |
| Whole-tick fuzz corpus (5-tick adversarial windows) | 33,382/40,000 |

## 2. The method (do not regress to anything weaker)

**Per-function isolation (FUNCPROBE).** Call ONE engine function by RVA
with installed state; read every observable back; grade against the
mirror. Whole-tick fuzzing composes ~25 functions and cannot attribute a
mismatch — it produced every wrong turn of the era.

**Fail-closed gates, all mandatory:**
- Context latch: during a real FinishMove, verify gm+0x08 == player and
  gm+0x10 == movedata. Gate value in the results header (`ctxgate 1`).
  Anything else → funcdiff refuses.
- Control probes: states the engine itself recorded as grounded in a
  certified capture, fed in with the observable CLEARED so the function
  must SET it. 0/72 = harness void, no grading. This gate caught: the
  FL_ONGROUND lie, a wrong-map click, and a stale-session serial.
- Preludes are DECLARED in `func_pins.cfg` (4th field = prelude RVA),
  never hidden in the runner.
- Every probe input that cannot be honestly installed is forced to a
  declared constant (ground handle → ungrounded both sides; gm+0xcc4 and
  gm+0xed8 zeroed per probe; mv oldbuttons from the probe row).

**Data over inference, always.** When two candidate laws each fix one
dataset and break another, the ORGANIZATION is wrong, not the constants
(this is how engine-ordered leaf tracing was found). When an engine
verdict contradicts the engine's own measured ray, your decode of the
call site is wrong — go back to bytes.

## 3. Instruments and exact workflows

Build (both targets):
```
vswhere -latest -find MSBuild\**\Bin\MSBuild.exe
MSBuild SolverLab.vcxproj -p:Configuration=Release -p:Platform=x64
MSBuild Basehook.vcxproj  -p:Configuration=Release -p:Platform=x64
```
Inject `Output\Release\Basehook.dll` into cstrike_win64.exe, F8 menu.
In-game buttons (Map Solve tab): **Run FUNCPROBE** (reads
`solver\func_pins.cfg` + `solver\func_probes.csv`, writes
`solver\func_results.csv`, ~400 calls/frame), **Run trace oracle** (reads
`solver\trace_queries.csv`, writes `solver\trace_results.csv`).

Offline commands (`Output\SolverLab\SolverLab.exe`), map =
`surf_basictest.bsp` full path:
```
funcgen <map> --n 20000 --control <playback csv> [--sites func_mismatch.csv]
funcdiff <map> func_probes.csv func_results.csv     # grades all pinned fns,
                                                    # writes func_mismatch.csv
tracegen-quads <map> func_mismatch.csv trace_queries.csv
quaddiff <map> trace_queries.csv trace_results.csv  # world vs raw-ray oracle
tracegen-map <map> <out>                            # full-map face sweep
soliddiff / tracediff                               # sweep flag/frac compares
battery <map>                                       # 15 capture decks
diff <map> <tas> <playback csv>                     # tape replay parity
fuzzdiff <map> fuzz_probes.csv fuzz_results.csv --grav1
```
Which steps need the game: ONLY fresh FUNCPROBE/oracle answers. Grading,
slicing, regeneration, and every A/B of a law against existing captures is
offline. Batch aggressively: one session can answer a 100k-probe deck plus
a 75k-query oracle batch (a few seconds each).

**File formats** (headers are authoritative; parsers accept these):
- probes v2 (23 cols): `fn,ox,oy,oz,vx,vy,vz,bx,by,bz,onground,ducked,
  ducking,buttons,ducktime,stamina,sfric,gravity,hullmaxz,yaw,fmove,
  smove,oldbuttons`
- results v2 (20 cols): `id,fn,ox..vz,flags,ducked,ducking,ducktime,
  stamina,sfric,maxz,ret,ok,fwd,side,groundent` (+ `# funcprobe v2
  ctxgate N` header). `ret` is AL-masked at capture.
- box queries (15 cols): `id,tick,ax,ay,az,bx,by,bz,ducked,mnx,mny,mnz,
  mxx,mxy,mxz` — a==b is a zero-length (unswept) test; 9-col rows are
  legacy hull-by-flag.
- mismatch manifest: `fn,ox,oy,oz,ducked,grounded,candz`.

## 4. Binary reference (x64, this build only)

### client.dll — CCSGameMovement
**True vtable base `0x4781c8`** (ctor `0x1e8b0`; stored into the GM
singleton pointer at `.data 0x68eec0`). The earlier working map was based
0x58 too high — all slot indices derived before 2026-08-15 late-night are
shifted; function identities are body-confirmed and correct.

Pins (all body-confirmed; `func_pins.cfg` carries the evidence inline):
| function | RVA | ABI | prelude |
|---|---|---|---|
| CategorizePosition | 0x1174f0 | void(this) | — |
| CheckJumpButton | 0x1f5330 | bool(this) | 0x1174f0 |
| Duck | 0x1f5cb0 | void(this) | 0x1174f0 |
| CanUnduck | 0x1f4b20 | bool(this) | 0x1174f0 |
| FinishDuck | 0x1f6500 | void(this) | — |
| FinishUnDuck | 0x1f6840 | void(this) | — |
| HandleDuckingSpeedCrop | 0x1f6ba0 | void(this) | — |
| ReduceTimers | 0x1f7600 | void(this) | — |

Identified, not yet pinned/probed: CheckParameters `0x1f56b0` (contains
the CS duck-until-ground head AND the duck-hold curtime gate),
FinishGravity `0x11a0c0`, CheckVelocity `0x1182e0`, GetPlayerMins
`0x11c240` (viewvecs × modelscale, returns out-ptr in RAX),
PlayerRoughLandingEffects `0x11d250` (takes float in xmm1),
WalkMove-stamina-drag body `0x1f7790` (pow drag on mv->vel.xy — the walk
era's first pin), LadderMask `0x17ac20` (returns `0x200400b`),
ProcessMovement-prologue body `0x11d360` (fills the -9999 stuck table,
stores player/mv at +0x08/+0x10, zeroes gm+0xcc4).

Object offsets (all read from bodies):
- GM: `+0x08` player, `+0x10` mv, `+0xcc4` m_iSpeedCropped, `+0xed8`
  duck-hold timestamp, `+0xEE0` cached CCSPlayer.
- player: `+0x440` m_fFlags (FL_ONGROUND=1, FL_DUCKING=2), `+0x8e4`
  m_flModelScale, `+0x1dc` m_flGravity (0→1.0), `+0x127c` punch,
  `+0x1250/1251/1254` m_bDucked/m_bDucking/m_flDucktime, `+0x1560`
  pl.deadflag, `+0x1684` m_flWaterJumpTime, `+0x1f8` waterlevel,
  `+0x1f4` movetype (9=ladder), `+0x1860` surfacedata (+0x40 maxspeed
  factor, +0x44 jump factor), `+0x1868` m_surfaceFriction, `+0x1a2c`
  m_flStamina, `+0x1a5c` m_duckUntilOnGround. m_hGroundEntity via netvar
  walk (DT_CSPlayer→DT_BasePlayer) at runtime.
- CMoveData: `+0x24` buttons, `+0x28` oldbuttons, `+0x2c/0x34/0x38`
  fwd/side/up, `+0x3c` maxspeed (CheckParameters writes the cap here),
  `+0x44` velocity, `+0x68` outStepHeight, `+0x74` outJumpVel.z, `+0x9c`
  m_vecAbsOrigin.
- trace_t: `+0x18` plane.normal, `+0x2c` fraction, `+0x36` allsolid,
  `+0x37` startsolid, `+0x38` fractionleftsolid; m_pEnt tested non-null
  for ground.
- gpGlobals ptr at `[0x5ae280]`: `+0xC` curtime, `+0x10` frametime.
- GetViewVectors: gamerules ptr `[0x607200]`, vtable `+0xf8` → struct:
  `+0x0c..0x14` hull min, `+0x18..0x20` hull max, `+0x24..0x2c` duck
  min, `+0x30..0x38` duck max — ALL scaled by modelscale. (The measured
  8.5 shift with a 54 collision duck hull implies the VIEW duck hull is
  55 — unverified, listed open.)
- cvar objects (int value at +0x58, float at +0x54):
  `[0x68fe88]` sv_autobunnyhopping, `[0x68ff18]` sv_enablebunnyhopping,
  `[0x68edd8]` the duck-hold window cvar (NAME UNKNOWN — identify via
  ConVar-registration xref before mirroring CheckParameters),
  `[0x681258]` a CanUnduck debug-draw toggle (no physics).
- enginetrace client ptr `[0x607a40]`.

### engine.dll
- CEngineTraceClient: singleton `0x4796f0` (via the
  `EngineTraceClient003` registration chain), vtable `0x3a4110`,
  **TraceRay = slot 4 = `0x18e7d0`**. Body structure: world trace →
  startsolid SKIPS the entity loop → finalization rescales
  fraction/fractionleftsolid for entity clips and zeroes
  fractionleftsolid on unswept rays. **There is NO startsolid rewrite.**
  On an entity-free map, world-only and everything filters produce
  identical traces.

## 5. The laws (each measured; see FullMapSolver.md for the evidence)

1. **Ground truth is m_hGroundEntity.** FL_ONGROUND is a server-driven
   shadow (isolated calls set it on 0/72 known-grounded states).
2. **bool returns live in AL only.** Mask `& 0xff` at capture; upper EAX
   is stack garbage (19,761 phantom mismatches once).
3. **Start-solid law:** a brush the sweep starts inside contributes
   flags only (0/1,236 ss-only rows zeroed fraction); an ALLSOLID brush
   zeroes fraction THE MOMENT IT IS PROCESSED (789/789) so earlier
   recordings survive and later ones cannot. Mechanism in code: process
   brushes in ENGINE ORDER, set `best = 0` at the allsolid brush,
   recording stays `enterfrac < fraction`.
4. **Engine order = the leaf walk:** leafbrushes lump 17, per-leaf lump
   order, near-child-first, first-occurrence-only. dleaf_t:
   firstleafbrush@+24 / numleafbrushes@+26 (the FACE fields at +20/22
   parse as plausible garbage — twice a bug source). Node descent uses
   strict `>` both-sides on touch (the literal `>=` prune lost 20 real
   flags).
5. **Unswept raw-ray leaf law:** a zero-length TraceRay reports
   startsolid+allsolid when the RAY START POINT is in a CONTENTS_SOLID
   leaf — point, not box extent (49 ladder rungs). Applies to
   TraceHullBox (the raw-ray model) and to CanUnduck's grounded branch.
   The movement-path zero-length consumers (FixPlayerCrouchStuck ladder,
   TryPlayerMove stuck-guard) are BRUSHES-ONLY — measured twice, a tape
   broke both times it was wired there.
6. **CategorizePosition:** vz <= 140 gate; 2u down-trace; walkable
   nz >= 0.7 with m_pEnt non-null; on no walkable plane,
   TracePlayerBBoxForGround re-probes four half-hull quadrant boxes;
   grounding zeroes vz (SetGroundEntity) and realigns the hull to the
   duck flag; sfric=1 reset at entry, 0.25 when rising with no walkable
   plane. Hull follows m_bDucked, NEVER the collision-bounds netvar.
7. **CheckJumpButton (full decode):** deadflag → waterjump drain (RAW
   frametime) → waterlevel>=2 shove → ground-ENTITY gate →
   sv_autobunnyhopping BYPASSES the release gate → PreventBunnyJumping
   if !sv_enablebunnyhopping → SetGroundEntity(0) → groundfactor
   (surfacedata+0x44, else 1) → SET (duckUntilOnGround||ducking||ducked)
   vs ADD, double multiply vs 301.99337741082996, ONE rounding → stamina
   tax (1 - s*0.00019f) → stamina := bits 0x44a47943 BEFORE
   FinishGravity → FinishGravity: (g*ent)*(dt*0.5) pairing then
   CheckVelocity (NaN exponent mask 0x7f800000 zeroing + ±maxvelocity).
8. **Duck family:** DOWN-counting shared timer; ducking=true through
   both transitions; ASYMMETRIC boundaries (duck completes AT exactly
   400ms `>=`; unduck holds AT exactly 200ms strict `>`); blocked unduck
   sets timer=1000 always and ducking=true on a release EDGE only;
   FinishDuck's +8.5 air shift is first-time-only but
   FixPlayerCrouchStuck (zero-length DUCKED-hull TRACE, brushes-only, 1u
   ladder ×36, restore-if-never-free) runs unconditionally; FinishUnDuck
   air shift −8.5 unconditional on the transition; CanUnduck decoded:
   ONE raw TraceRay origin→newOrigin (grounded: newOrigin==origin;
   air: −0.5×viewvec hull delta), verdict `!startsolid && fraction==1`.
   The 0.34 crop fires INSIDE Duck on the PRE-duck state; the tick
   mechanism is CheckParameters' sqrt scale (450→250) THEN the crop
   (→85).
9. **ReduceTimers:** stamina and ducktime each drain dt*1000, clamp 0.
10. **Player-clip note:** loader keeps contents & 0x201400B; the map's
    shell slabs (brush 0 = ceiling z[1248,1280], brush 1 = floor
    z[−1088,−1056]) fill the void leaves' brush lists — "impossible"
    solids at void positions are these.

## 6. Traps that lied (check these FIRST when something looks wrong)

- FL_ONGROUND (shadow), m_flGravity writes ignored, hullmaxz netvar
  ignored by traces.
- AL-only bools; `id,...` headers starting with 'i' slipping past
  `'#'/'f'` line filters (207 phantom sites); results rows dropped on
  faults misaligning index joins.
- dleaf faces-vs-brushes offsets (+20/22 vs +24/26) — bit us twice.
- Vtable base off-by-N: NEVER trust slot arithmetic without finding the
  ctor's vtable store; identities must come from bodies.
- Wrong-map / fresh-session captures: worldspawn serial changes;
  controls void the batch — trust the gate.
- The oracle's stored-ours columns go stale across builds (tracediff
  compares generation-time answers). soliddiff re-traces; tracediff does
  not.
- Session-to-session variance at embedded sites: 3 CategorizePosition
  rows flipped between identical runs once — suspected live-entity
  proximity; never reproduced. If fresh runs disagree with old runs at
  torture sites, re-run before theorizing.

## 7. Hypotheses tried and REVERTED (do not retry without new data)

CanUnduck candidate at −18 (broke five decks); universal 62.5 transient
hull; any out-of-world solidity rule for SWEPT traces (AABB and
BSP-leaf); startsolid keep-fraction (579); startsolid plane-wipe;
allsolid survivor by index-prefix and by global-best (leaf ORDER is the
truth); `>=` node prune; leaf law inside the movement trace (broke
solved12 at 544u, FinishDuck 496→1,147); box-extent leaf law (49 rungs);
CanUnduck as sweep-with-ss-verdict (21→58) and as destination-box-only
(582-contradiction); tick wiring of scale+crop with the current
approximate scale math (0.001u tape wobble — the covenant is tape truth).

## 8. Open questions, each with its exact next action

1. **The air-CanUnduck anomaly (gates Duck/CanUnduck/FinishDuck
   residuals).** 582 joined rows: verdict=free while the engine's own
   ray at the assumed pos→pos−8.5 standing box says startsolid; blocked
   and free rows share identical ray signatures. TraceRay itself has no
   ss rewrite → the RAY BUILT DIFFERS from the assumption. The decode's
   soft spot: the mask arg at `0x1f4e84-0x1f4e9c` resolves (via the
   corrected vtable) to GetPlayerMins returning a POINTER — nonsense →
   the call-target/arg-role reading is wrong somewhere. NEXT: hand-decode
   raw bytes 0x1f4e60-0x1f4f00 (modrm by hand, no tooling shortcuts), or
   ONE 10-second click: a runner hook logging TraceRay's actual
   (ray.start, ray.delta, mins, maxs, mask) during isolated CanUnduck
   calls. Note: all affected states are unreachable in legal play.
2. **CheckParameters exact float math** (then wire scale+crop into the
   tick and retire the duck maxspeed cap): needs the umove term, the
   surfacedata maxspeed min, IN_SPEED (×0.52 upmove), double-vs-float
   details, and the duck-hold cvar name at `[0x68edd8]` (find via
   ConVar-registration xref: locate the registration call that passes
   that object address; the name string pointer is loaded nearby).
   `unduck_face` (1.450u) likely falls here or in transient-hull
   interaction — crop alone did NOT fix it (measured).
3. **FinishUnDuck's 4 rows** — dump and read individually after (1).
4. **Segment-splitting recursion** (CM_RecursiveHullCheck's true clipped
   descent) — 25 normal-only box rows at one embedded site; also
   hardens long multi-leaf traces for arbitrary maps.
5. **Walk era:** pin via the corrected vtable + `0x1f7790`; extend the
   runner ABI table for arg-taking functions (TryPlayerMove,
   ClipVelocity, Accelerate, TestPlayerPosition — the last also settles
   the unswept split independently). StartGravity has NEVER been
   disassembled (tick gravity association unverified).
6. **Corpus closure 33,382 → 40,000** by per-function exclusion.
7. **World completeness for any map:** displacements (most surf maps),
   water + the already-shaped water branches, ladders, surfaceprops
   (jump/maxspeed/friction factors — currently default 1.0), an
   entity-solid/prop policy, then the full instrument suite on a second
   map as certification.
8. **Housekeeping:** regenerate the map-sweep query/results pair (the
   stored pair predates current generators), solver throughput benchmark
   under leaf-ordered tracing, cvar-export audit.

## 9. Where everything lives

- Code: `Source/Solver/` (SolverMove = mirrors in the engine's own
  decomposition, SolverWorld = trace + BSP, SolverLab = all commands),
  `Source/World/Prediction.cpp` (the in-game runner), `Source/Editor/
  TasEditor.cpp` (oracle + params export).
- Data: `C:\Users\Connor\Documents\sourceTAS\solver\` — func_pins.cfg,
  func_probes/results.csv, func_mismatch.csv, trace_queries/results.csv
  (+ `_mapsweep` backups), fuzz decks, playback captures, server_params.
- Tapes: `...\sourceTAS\recordings\` — the two certified references are
  `surf_basictest_cma_S292_A404_CLEAN_0814-0248.tas` and
  `surf_basictest_solved12.tas` with matching `playback_*.csv`.
- RE scratch scripts (rebuild if the scratchpad is gone — each is <100
  lines and described in FullMapSolver.md): re_disasm.py (RVA-range
  disassembler with pool annotations), vtable_hunt.py, pin_jump.py,
  enginetrace_hunt.py, duck_slice.ps1, canunduck_margin.py.
