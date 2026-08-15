# FUNCPROBE - per-function differential mapping of the engine movement code

Generated 2026-08-15 by a multi-agent research pass (VDC wiki + binary RE +
codebase inventory + adversarial critique). THIS IS THE IMPLEMENTATION
CONTRACT for replacing the whole-tick fuzzer, which cannot attribute a
mismatch to any single function.

{
  "summary": "Inventory every engine movement/world function and design a per-function differential fuzzer",
  "agentCount": 9,
  "logs": [],
  "result": {
    "plan": "# FINAL DELIVERABLE — Per-function differential fuzzer for 1:1 Source movement cloning

**Verdict on the two designs: hybrid, weighted toward B, with A's discipline and a narrowed version of A's direct-call.** The adversarial critique is correct on every load-bearing point I could verify in the code, so this spec adopts its corrections rather than either design as written.

The three decisions that shape everything below:

1. **Isolation through the real `SetupMove → ProcessMovement → FinishMove` is the primary instrument.** It is the only one that can observe call *order* and cross-tick *ownership* — the defect class that produced green-board items 2, 3, and 4 (`Docs\\FullMapSolver.md:1953-1980`). Direct-call cannot discover an order you supply yourself.
2. **Direct-call survives only for the eight confirmed *virtuals*** (hull dims, `TracePlayerBBox`, `PlayerSolidMask`, `CanUnduck`). Virtual functions must have bodies, so inlining cannot silently gut coverage; and each is pinned by *agreement with the already-validated `IEngineTrace` oracle* or by a bit-exact constant, never by a no-op probe. `Friction`, `Accelerate`, `ClipVelocity` are **not** direct-called: `Friction` is inlined (mechanism research: `0x11ADB0`), and their ordering is the thing under test.
3. **Design A's per-function input/output *slot vectors* are deleted.** They are unfalsifiable claims that bake in answers (`Accelerate` reading `surface_friction`; `Friction` taking a settable `on_ground` bool that this project has already *measured* to be unsettable — ground is `m_hGroundEntity`, `Prediction.cpp:296` comment block). The registry survives in a corrected role: a shared **inventory + verdict namespace**, not a call-dispatch/purity table.

Verified against the tree (correcting stale numbers in both designs): `SolverMove.cpp:5` opens `namespace {`, closes at `:578`; `namespace Solver` spans `:3`–`:696`. Design A's `:578` was right; the critique's `:696` refers to the outer namespace.

---

## PART 0 — Definitions used by the inventory table

**Chambers** (seed locations, derived offline from the loaded `.bsp` — no map authoring):

| id | chamber | construction rule |
|---|---|---|
| C0 | void | 64u-radius sweep of hull traces all return frac 1 |
| C1 | flat floor | 0.1u above a face with `n.z ≥ 0.99`, ≥128u headroom |
| C2 | ramp | above a face with `0.7 ≤ n.z < 0.95` |
| C3 | wall | beside a face with `|n.z| < 0.1` |
| C4 | crease | near a convex edge between two faces (the seam class) |
| C5 | ceiling | headroom where the 54 hull fits and the 72 hull does not |
| C6 | embedded | origin inside a brush (allsolid / stuck class) |
| C7 | ledge | walkable face with a 4–18u riser within one tick's reach |

**Strata** (what is silenced; a convar at 0 makes its term an *exact* identity, not an approximation — `drop = control·0·sfric·dt = 0`, `accelspeed = 0·… = 0`, gravity term `= 0`):

| id | stratum | silenced | chamber | left active |
|---|---|---|---|---|
| S0 | `identity` | friction/accel/airaccel/gravity 0, maxvelocity 1e9, zero input | C0 | ReduceTimers + CategorizePosition + integration **only** — the gate |
| S1 | `gravity` | friction/accel/airaccel 0 | C0 | Start/FinishGravity |
| S2 | `maxvel` | all 0 | C0 | CheckVelocity |
| S3 | `friction` | accel/airaccel/gravity 0 | C1 | Friction (+ WalkMove/StayOnGround residue) |
| S4 | `accel` | friction/gravity 0 | C1 | Accelerate, duck speed crop |
| S5 | `airaccel` | gravity 0 | C0 | AirAccelerate, 30-cap knee, `surface_friction` scaling |
| S6 | `integrate` | all 0 | C0 | TryPlayerMove position integration |
| S7 | `clip` | all 0 | C2/C3/C4 | TryPlayerMove clip loop + ClipVelocity + trace |
| S8 | `step` | all 0 | C7 | StepMove + StayOnGround |
| S9 | `jump` | gravity swept | C1 | CheckJumpButton, PreventBunnyJumping, stamina arm |
| S10 | `duck` | all 0 | C0+C5 | Duck machine, hulls, CanUnduck, FixPlayerCrouchStuck |
| S11 | `timers` | all 0, zero input | C0 | ReduceTimers, stamina/ducktime/jumptime decay |
| S12 | `categorize` | all 0 | C1/C2 | CategorizePosition (z-offset × vz sweep across the 140 gate) |
| S13 | `stuck` | all 0 | C6 | CheckStuck / allsolid |
| S14 | `cmdclamp` | all 0 | C0 | CheckParameters magnitude rescale |
| S15 | `composite` | **nothing** | all | full-tick integration — the honesty control arm |

**Call-directly legend:** `VT n` = virtual, in scope · `VT n ✗` = virtual, deliberately out of scope · `RVA ✗` = body exists, calling it is policy-forbidden · `INLINED` = no standalone body · `SERVER` = never executes in our client hook · `n/a` = not an engine function.

---

## PART 1 — INVENTORY (leaf-first: dependencies before composites)

| L# | Function | Class | Inputs | Outputs | Call directly? | Modeled? | How it will be fuzzed |
|---|---|---|---|---|---|---|---|
| 0.1 | `AngleVectors` (yaw basis) | shared math | viewangles | forward/right/up | INLINED | YES — `WishFromInput` `SolverMove.cpp:32` | S4/S5 yaw sweep ±180 incl. ULP-adjacent; wishdir recovered from the velocity delta |
| 0.2 | `VectorNormalize` | shared math | vec | unit vec, len | INLINED | YES (implicit) | S4/S5 near-zero and denormal wishspeed; MXCSR matched offline |
| 0.3 | `CGameMovement::ClipVelocity` | CGameMovement | in vel, plane n, overbounce | out vel, **`blocked` mask** | INLINED | PARTIAL — `EngineClipVelocity:10`, **`blocked` discarded** | S7 single-plane C3/C2 impacts; velocity delta *is* the clip; `blocked` inferred from whether the engine took the floor/step branch |
| 0.4 | `CGameMovement::CheckVelocity` | CGameMovement | vel, `sv_maxvelocity`; SDK also NaN-checks vel **and clamps origin to MAX_COORD** | clamped vel/origin | INLINED | PARTIAL — `:18`; **no NaN zeroing, no origin clamp** | S2: sweep `sv_maxvelocity` × {0, 1e-30, 1e-7, ±3499.9998, ±3600, ±1e30, inf, NaN} × origin at ±16384 |
| 0.5 | `CGameMovement::PlayerSolidMask` | CGameMovement | `brushOnly` | `0x1400B` / `0x201400B` | **VT 12** | NO (BspWorld hardcodes SOLID\\|GRATE\\|PLAYERCLIP\\|LADDER, `BspWorld.cpp:19-22`) | Direct-call pin, both args; bit-exact constants. **Predicted red: CONTENTS_MOVEABLE/WINDOW/MONSTERCLIP missing from our solid set** |
| 0.6 | `CGameMovement::CalcRoll` | CGameMovement | angles, vel, `sv_rollangle/rollspeed` | roll degrees | INLINED | NO | S14: read `pm_angles_z` from the movedata dump; sweep vel × view yaw |
| 0.7 | `CGameMovement::DecayPunchAngle` | CGameMovement | `m_vecPunchAngle`, `m_vecPunchAngleVel`, dt, spring 65 / damp 9 | both decayed, snap <0.001 | INLINED | NO | S11 with punch seeded via `m_Local.m_vecPunchAngle` netvar; read back per tick. Affects `mv->m_vecAngles`, not wishdir — matters for closed-loop recording only |
| 0.8 | `CGameMovement::StartGravity` | CGameMovement | vz, `sv_gravity`, `m_flGravity`, basevel.z, dt | vz −= g·s·0.5·dt; vz += basevel.z·dt; basevel.z = 0 | INLINED | YES — `MoveTick:638-640` | S1: sweep `sv_gravity` {0,1,800,1e5} × `m_flGravity` {0.2,1,2} × vz. **Also the `m_flGravity`-write-rejected probe** |
| 0.9 | `CGameMovement::FinishGravity` | CGameMovement | vz, gravity, dt | vz −= g·s·0.5·dt | INLINED | YES — `:656` | S1; the `CheckVelocity`-after-FinishGravity ordering is re-proved by a terminal-velocity probe (the −3500 vs −3506 case) |
| 0.10 | `CGameMovement::ReduceTimers` | CGameMovement | dt; `m_flDucktime`, `m_flDuckJumpTime`, `m_flJumpTime`, `m_flSwimSoundTime`, `m_flStepSoundTime`, CS `m_flStamina` | each −= dt·1000, floored at 0 | INLINED | PARTIAL — stamina+ducktime open-coded `MoveTick:597-606`; **`m_flDuckJumpTime`/`m_flJumpTime` absent** | S11: zero input, seed every timer across {0, 0.5·dt·1000, exactly dt·1000, 1000}, 4 test ticks; verdict is exact per-timer decay + the ordering claim (drains *before* Duck) |
| 0.11 | `CBasePlayer::UpdateButtonState` | CBasePlayer | `ucmd->buttons`, prev `m_nButtons` | `m_afButtonLast/Pressed/Released` | SERVER (client mirrors) | NO (we track `old_buttons` = `mv->m_nOldButtons`) | S9/S10 button edge patterns; verdict from `mv->m_nOldButtons` in the movedata dump, which is the field the duck/jump gates actually read |
| 0.12 | CS stamina walk drag | CCSGameMovement | `m_flStamina`, dt, scale 0.00019, rate 70 | `vel.xy *= powf(1−s·k, dt·70)` | INLINED | YES — `WalkMove:366-371` | S3+S4 with stamina seeded {0,1,100,1315.79,1400}; the pow-vs-affine question re-decided by data |
| 0.13 | CS stamina jump tax | CCSGameMovement | stamina, post-impulse vz | `vz *= (1 − s·0.00019f)` | INLINED | YES — `CheckJumpButton:254` | S9 stamina sweep × ducked/standing |
| 0.14 | `CGameMovement::GetAirSpeedCap` | CCSGameMovement | — | 30 | VT ✗ (index unproven) | YES — `air_speed_cap` | S5: wishspeed swept across {29.999998, 30, 30.000002} — the knee declares itself |
| 0.15 | `CGameMovement::Accelerate` | CGameMovement | vel, wishdir, wishspeed, `sv_accelerate`, dt, **`m_surfaceFriction` (claim under test)**, maxspeed | vel += wishdir·accelspeed | INLINED | YES — inline in `WalkMove:390-397` | S4: `sv_accelerate` × wishspeed × current-speed-along-wish, on C1. **`surface_friction` is NOT supplied — it is seeded via `+0x1868` and read back; whether it participates is the measurement, not the premise** |
| 0.16 | `CGameMovement::AirAccelerate` | CGameMovement | vel, wishdir, wishspeed, `sv_airaccelerate`, dt, sfric, cap 30 | vel += wishdir·accelspeed | INLINED | YES — inline in `AirMove:435-449` | S5 in C0: the asymmetric budget (`accelspeed ∝ wishspeed`, `add` capped at 30) is separable because the two use different wishspeeds |
| 0.17 | `CGameMovement::Friction` | CGameMovement | speed, `sv_friction`, `sv_stopspeed`, `m_surfaceFriction`, dt | vel scaled by `newspeed/speed` | **INLINED** (`0x11ADB0`) | YES — `:209` | S3 on C1, XY-only velocity on open floor so the position channel is pure `pos += vel·dt`; sweep `sv_friction`/`sv_stopspeed`/speed/sfric. **Confirms the project's 75-not-100 stopspeed** |
| 0.18 | `PreventBunnyJumping` | CCSGameMovement | speed, `sv_maxspeed`, `sv_enablebunnyhopping` | pre-jump 1.1× clamp | INLINED | YES — `CheckJumpButton:233-238` | S9 with the convar written **both** ways and read back; speeds straddling 1.1·maxspeed |
| 1.1 | `GetPlayerMins(bool)` | CGameMovement | ducked, `m_flModelScale`, hull table `[0x607200]` | Vector | **VT 5** | Implicit (`Hulls`, `SolverWorld.h:30`) | Direct pin: `mins(f).z==0`, `mins(t).z==0`, `mins.x==−16`; **+ ambient-invariance** (same args under stand/duck/moving/weapon-swapped settle → bit-identical or the spec is incomplete) |
| 1.2 | `GetPlayerMaxs(bool)` | CGameMovement | same | Vector | **VT 6** | Implicit | Pin: `maxs(f).z==72`, `maxs(t).z==54`, **delta==18** (discriminative — a null fn fails); + ambient-invariance |
| 1.3 | `GetPlayerViewOffset(bool)` | CGameMovement | same | Vector | **VT 7** | NO (eye height irrelevant to collision) | Pinned for record only; feeds nothing |
| 1.4 | `GetPlayerMins()/Maxs()` | CGameMovement | live hull-type virtual, model scale | Vector | **VT 8 / VT 9** | Implicit | Pinned; **the transient-hull question**: sampled every experiment tick post-air-unduck to test our 62.5 `hull_state 2` against the engine's own live answer |
| 1.5 | `TracePlayerBBox` | CGameMovement | start, end, mask, group; VT8/9 hull | `trace_t` | **VT 10** | Mirrored by `World::TraceHull3` | Pinned by **agreement with the already-validated `IEngineTrace`** (`TasEditor.cpp:8385`) on 3 chamber-anchored probes; then **every** experiment emits 8 canonical sweeps answered by *both* oracles → continuous cross-pin |
| 1.6 | vt11 (trace wrapper, likely `TryTouchGround`) | CGameMovement | ≥5 args, unproven | `trace_t` | **VT 11 ✗** | NO | **Not called.** Recorded `UNPINNED` in the manifest; never graded |
| 1.7 | `GetPointContentsCached` / `ResetGetPointContentsCache` | CGameMovement | point, slot, per-tick cache | contents (stale within a tick) | RVA ✗ | NO | Only reachable through water code we do not model → `UNCOVERED`, stated as a gap |
| 1.8 | `CategorizeGroundSurface` | CGameMovement | `pm.surface.surfaceProps`, `IPhysicsSurfaceProps` | `m_pSurfaceData`, `m_surfaceFriction = friction·1.25` clamped to 1 | RVA ✗ | **NO** — we emit only {1.0, 0.25} | S12 on every distinct surface the map offers; `+0x1868` read back. **On a single-material map this is structurally uncoverable** (see Part 3) |
| 1.9 | `CheckStuck` + `CheckInterval` + `GetRandomStuckOffsets` | CGameMovement | origin, hull, 54-entry offset table, per-player counter, entity index, tickcount | teleports origin; **early-returns the whole tick** | RVA ✗ | **NO** — we zero velocity and continue | S13 on C6, recorded as engine truth. Graded **OBSERVE-ONLY** until the persistent counter is located: the displacement is history-dependent, not a pure function of position |
| 1.10 | `CanUnduck` | CGameMovement | origin, standing hull, `TracePlayerBBox`+`PlayerSolidMask` | bool | **VT 50** | Approximated — `w.OriginInSolid(cand,false)` `HandleDuck:537` | Direct pin on C5 (false under the ceiling, true 32u away). **The −18 vs −8.5 test-origin question** (`SolverMove.cpp:532-536`, reverted 2026-08-15) becomes directly decidable |
| 1.11 | `CanUnDuckJump` / `StartUnDuckJump` / `FinishUnDuckJump` | CGameMovement | duck-jump timers, hull | duck-jump path | VT ✗ | NO | S10 with `m_flDuckJumpTime` seeded; `UNCOVERED` until the timer is modeled |
| 1.12 | `FixPlayerCrouchStuck` | CGameMovement | origin, hull | upward probe, restore on failure | INLINED | YES — `HandleDuck:495-507` | S10 on C6/C5: the "restore the original origin when none is found" rule (the 18.0000 phantom offset) is re-proved |
| 1.13 | `SetDuckedEyeOffset` / `UpdateDuckJumpEyeOffset` | CGameMovement | duck fraction, VT5/VT7 | view offset | RVA ✗ | NO | Cosmetic; `OUT-OF-SCOPE` by declaration |
| 2.1 | `CGameMovement::CheckParameters` | CGameMovement | fmove/smove/**upmove**, `m_flClientMaxSpeed`, `m_flMaxSpeed`, surface `maxSpeedFactor`, constraint factor, FL_FROZEN/ONTRAIN/dead, punch | **rescales the 3D command magnitude**; zeroes when frozen/dead; `DecayPunchAngle`; writes `mv->m_vecAngles` | INLINED, **unsilenceable** | **NO — absent from `MoveTick` entirely** | **S14**: sweep (fmove,smove,upmove) over {0,±150,±250,±400,±450,±10000} and read `sm_fmove/sm_smove/sm_umove` (post-`SetupMove`) vs `pm_*`. **This is the highest-confidence predicted red on the board** |
| 2.2 | `CheckJumpButton` | CCSGameMovement | on_ground, `mv->m_nOldButtons`, vel, ducked/ducking, stamina, `m_flGravity`, baked impulse `301.99337741082996` | vz set/added, stamina armed to `25000/19`, ungrounded, FinishGravity | RVA ✗ (mutates live state) | YES — `:223` | S9 on C1: release-gate edges × ducked/ducking × stamina × `sv_gravity` swept (impulse must **not** track it) × bunnyhop convar both ways |
| 2.3 | `TryPlayerMove` | CGameMovement | vel, pos, hull, dt, world | pos, vel, `blocked`, touch list | INLINED | YES — `:45` | **Split**: S6 (C0, integration only — must be exactly `pos += vel·dt`) and S7 (C2/C3/C4, clip loop). Approach angle/offset from the `tracegen-map` query family |
| 2.4 | `StepMove` | CGameMovement | pos, vel, `sv_stepsize`, walkable_z, DIST_EPSILON | pos, vel; farther-XY-wins | INLINED | YES — `:283` | S8 on C7: riser heights {1,4,8,17.9,18,18.1,24} × approach speed/angle. Attribution by which of the two attempts the engine kept |
| 2.5 | `StayOnGround` | CGameMovement | pos, stepsize, 2u up-probe | snapped pos | INLINED | YES — `:335` | S3/S8 on C1/C7; z-offsets straddling the `COORD_RESOLUTION/2` guard |
| 2.6 | `CategorizePosition` + `SetGroundEntity` | CGameMovement | vz vs 140, 2u down-trace, walkable_z, `sv_optimizedmovement` | on_ground, ground entity, **`m_surfaceFriction`**, vz zero on new ground, hull re-base | INLINED, **unsilenceable** | PARTIAL — `:162`; no water/ladder/moving-platform ground, friction ∈ {1, 0.25} only | **S12**: z-offset above surface × vz swept through {139.999985, 140, 140.000015} × rising/falling × walkable/non-walkable plane. **The ownership rule is tested by the PRE/POST forcing cross-product (see §2.9), not assumed** |
| 2.7 | `Duck` / `FinishDuck` / `FinishUnDuck` | CGameMovement | `IN_DUCK` edges, `m_flDucktime`, `m_bDucked/m_bDucking`, on_ground, `CanUnduck` | duck flags, timer, hull, ±8.5 air origin shift | RVA ✗ | YES — `HandleDuck:464` | S10 on C0+C5: full press/release edge lattice × timer seeds {0,200,600,800,1000} × air/ground × blocked-by-ceiling. 4 test ticks, teacher-forced between each |
| 2.8 | `HandleDuckingSpeedCrop` | CGameMovement | duck state, `m_iSpeedCropped` | fmove/smove ×0.34 | INLINED | YES (`cap_ducked`, `WalkMove:385`) | S4 on C1 across the duck lattice; the "ducked at ANY point this tick" rule is measured, not assumed |
| 2.9 | `WalkMove` | CCSGameMovement | vel, wishdir, ground, stamina, basevel | full ground move | INLINED | YES — `:354` | S15 composite on C1/C7; per-stage attribution via the model checkpoint tap (§2.9c) |
| 2.10 | `AirMove` | CGameMovement | vel, wishdir, basevel | full air move | INLINED | YES — `:422` | S5+S7 in C0/C2; the basevel-after-accel ordering (probe 5, dvel 55.6) is re-proved by S15 with basevel seeded |
| 2.11 | `FullWalkMove` | CGameMovement | ground state, water, jump | Start/FinishGravity, jump, friction, Walk/AirMove, categorize order | INLINED | YES — `MoveTick:636-662` | **The order itself is the unit under test**: S1/S3/S9 pairwise-silenced, so which stage owns a delta is decidable |
| 2.12 | `LadderMove` / `FullLadderMove` | CGameMovement | ladder contents, view | ladder movetype | RVA ✗ | **NO** | No experiments emitted → `UNCOVERED`, never green |
| 2.13 | `CheckWater` / `CategorizeWaterLevel` / `WaterMove` / `WaterJump` | CGameMovement | water contents, eye/waist points | water physics, `m_nOldWaterLevel` | RVA ✗ | **NO** | `UNCOVERED` (see Part 3) |
| 2.14 | `FullNoClipMove` / `FullTossMove` / `FullObserverMove` | CGameMovement | movetype | alt movement | RVA ✗ | **NO** | `OUT-OF-SCOPE` by declaration (MOVETYPE_WALK only) |
| 2.15 | `CGameMovement::PlayerMove` | CGameMovement | movetype, dead, `sv_optimizedmovement`, water, ground | the **call-order spine**: CheckParameters → timers → basis → stuck → *conditional* categorize → step sound → duck → ladder → movetype switch | RVA ✗ | Implicit in `MoveTick` | S0 gate + S15. **The `sv_optimizedmovement` opening-categorize skip and the `vz>250` unground are probed explicitly in S12** |
| 2.16 | `CGameMovement::FinishMove` | CGameMovement | mv | writes back to player, `m_nOldButtons` | Hooked (VT 19 on IPrediction) | Implicit | Every experiment already runs the real `g_orig_finishmove`; `post_*` dump is its output |
| 2.17 | `CGameMovement::ProcessMovement` (+CS override) | CGameMovement / CCSGameMovement | player, movedata, `frametime × GetLaggedMovementValue()` | installs `player`/`mv`, PlayerMove, FinishMove | **VT 1** (already used) | n/a | The harness's own call. **`GetLaggedMovementValue() ≠ 1` detected by the S0 ballistic-dt check** |
| 3.1 | `CPlayerMove::SetupMove` | CPlayerMove | ~15 player members | fills CMoveData | VT 18 on IPrediction (already used) | Partially — our seed writes | `sm_*` column group in `state.csv` makes the whole marshalling boundary observable |
| 3.2 | `CPlayerMove::RunCommand` | CPlayerMove | ucmd, tickbase, TICK_INTERVAL, paused | sets `curtime`/`frametime`, drives the whole tick | SERVER | Implicit (`BeginStateGuard` forces `frametime = interval`) | Not fuzzed; `frametime` forcing verified by the S0 ballistic-dt check |
| 3.3 | `CPlayerMove::CheckMovingGround` | CPlayerMove | FL_BASEVELOCITY, conveyor, ground entity vel, dt | basevel fold-in with `(1 + dt/2)` overshoot, flag clear | SERVER | PARTIAL — `MoveTick:586-593` keeps basevel; **no `(1+dt/2)` fold, no conveyor** | Client-side basevel behavior fuzzed in S15 with `m_vecBaseVelocity` seeded; **the server rule is unverifiable here** (Part 3) |
| 3.4 | `RunPreThink` / `RunThink` / `RunPostThink` | CPlayerMove | think schedule | trigger touch, fall damage, step sound tail | SERVER | NO | Unverifiable in this design (Part 3) |
| 3.5 | `CPlayerMove::StartCommand` | CPlayerMove | ucmd | `RandomSeed(cmd->random_seed)` | SERVER | NO | Determinism only; irrelevant to movement math |
| 3.6 | `trigger_push` / `trigger_gravity` / `trigger_teleport` Touch | server entities | volume overlap | basevel + FL_BASEVELOCITY, `m_flGravity`, origin+vel zero | SERVER | YES — `MoveTick:669-693` mirrors `BspWorld::CheckTriggers` | **Not fuzzable by this instrument.** Certified only by full-tape playback capture on a triggered map |

---

## PART 2 — IMPLEMENTATION PLAN

Working name: **FUNCPROBE**. One button: `Run FUNCPROBE (engine query)` in the Map Solve tab.

### 2.0 Files

**New — shared, added to BOTH `Basehook.vcxproj` and `SolverLab.vcxproj`:**
- `Source\\Solver\\FuncSpec.h`, `Source\\Solver\\FuncSpec.cpp` — the registry (POD only; no engine, no solver includes)
- `Source\\Solver\\SolverFn.h` — model leaf declarations

**New — SolverLab only:**
- `Source\\Solver\\SolverFuncPlan.h/.cpp` — chamber discovery + plan generation (`funcgen`)
- `Source\\Solver\\SolverFuncDiff.h/.cpp` — teacher-forced differ + ablation (`funcdiff`)

**New — Basehook only:**
- `Source\\World\\FuncProbe.h/.cpp` — the in-game runner
- `Source\\World\\TraceOracle.h/.cpp` — mechanical extraction of `EngineTraceABI`, `AcquireEngineTrace`, `OracleTrace`, `ValidateOraclePin`, `ProcessTraceOracle` out of `TasEditor.cpp:8329-8488` so `FuncProbe` can use them

**Modified:**
- `Source\\Solver\\SolverMove.cpp` — `namespace {` at `:5` → `namespace Fn {`, close at `:578`; hoist inline leaves; add `MoveTickTraced`
- `Source\\Solver\\SolverMove.h` — declare `TickTrace`, `MoveTickTraced`
- `Source\\Solver\\SolverLab.cpp` — dispatch for `funcgen`/`funcdiff` next to `:3317-3335`; usage text; **the `s_at_fail` fix** (§2.12)
- `Source\\World\\Prediction.h/.cpp` — `EngineCtx` accessor, gm-context latch at `:630`, FuncProbe job branch beside the fuzz branch at `:656`
- `Source\\World\\Cvars.h/.cpp` — `Handle` / `GetByHandle` / `SetByHandle`
- `Source\\Editor\\TasEditor.cpp` — one button near `:8657`
- both `.vcxproj`

### 2.1 Model refactor (mechanical, zero behavior change)

`SolverFn.h` exposes what the anonymous namespace hides, plus the leaves currently inline inside `WalkMove`/`AirMove`/`MoveTick`:

```cpp
namespace Solver { namespace Fn {
  void  ClipVelocity(const Vec3& in, const Vec3& n, float overbounce,
                     Vec3& out, int& blocked);        // blocked mask NOW returned
  void  CheckVelocity(Vec3& vel, float maxvelocity);
  void  WishFromInput(float yaw, float fmove, float smove, float* wx, float* wy);
  void  Accelerate   (Vec3& vel, const Vec3& wishdir, float wishspeed,
                      float accel, float dt, float sfric);
  void  AirAccelerate(Vec3& vel, const Vec3& wishdir, float wishspeed,
                      float airaccel, float dt, float sfric, float cap);
  void  Friction     (Vec3& vel, float sfric, float sv_friction,
                      float stopspeed, float dt);
  void  StartGravity (Vec3& vel, Vec3& basevel, float g, float gscale, float dt);
  void  FinishGravity(Vec3& vel, float g, float gscale, float dt);
  float StaminaWalkScale(float stamina, float k, float rate, float dt);
  float StaminaJumpScale(float stamina, float k);
  void  ReduceTimers (float& stamina, float& ducktime, float dt);
  // level 1+ keep their existing PlayerState/World signatures
  void  TryPlayerMove(PlayerState&, const World&, const MoveParams&, TickEvents*);
  void  CategorizePosition(PlayerState&, const World&, const MoveParams&, TickEvents*);
  void  StepMove(...); void StayOnGround(...); void CheckJumpButton(...);
  void  WalkMove(...); void AirMove(...); void HandleDuck(...);
}}
```

Plus the checkpoint tap that makes attribution sharp without any engine call:

```cpp
struct TickTrace {
  enum Stage { kEntry, kPostTimers, kPostDuck, kPostStartGravity, kPostJump,
               kPostFriction, kPostCheckVel1, kPostAccel, kPostMove,
               kPostCategorize, kPostFinishGravity, kEnd, kStageCount };
  Vec3  pos[kStageCount], vel[kStageCount];
  float sfric[kStageCount];
  unsigned char hull[kStageCount];
  bool  on_ground[kStageCount];
};
void MoveTickTraced(PlayerState&, const World&, const MoveParams&, float pitch,
                    float yaw, float fmove, float smove, float umove,
                    int buttons, TickEvents*, TickTrace* tt /*nullable*/);
```

`MoveTick` becomes a one-line wrapper. **Acceptance gate for this step, before anything else is built:** run `SolverLab battery <map.bsp>`, `SolverLab fuzzdiff …`, and every tape deck before and after; the CSVs must be **byte-identical**. Commit only on identical. A refactor that changes physics silently poisons every later measurement.

### 2.2 FuncSpec registry — corrected role

The registry is the **shared inventory and verdict namespace**, not a purity table:

```cpp
namespace FuncSpec {
  enum Access : unsigned char { kAccVtable, kAccInlined, kAccServerOnly,
                                kAccOutOfScope, kAccUnpinned };
  enum Klass  : unsigned char { kBase, kCS, kPlayerMove, kShared };
  struct Spec {
    const char* name;           // "CategorizePosition"
    int         level;          // 0/1/2/3, table order
    Klass       klass;
    Access      access;
    int         vtable_idx;     // kAccVtable only
    const char* model_symbol;   // "Solver::Fn::CategorizePosition" or nullptr
    const short* strata; int n_strata;   // which strata grade this entry
    bool        unsilenceable;  // graded only by residue/ablation
  };
  int Count(); const Spec& At(int); int IndexOf(const char*);
}
```

No input/output slot vectors. What varies is a property of the **stratum's sweep**, which is a claim about what we vary — falsifiable and honest — not a claim about what the function reads, which is not testable from the outside.

### 2.3 Gates (all fail-closed; every result recorded in the manifest)

Run once at batch start, in this order. **Any failure marks the affected scope and the differ refuses to grade it.**

**G1 — gm-context latch.** Verbatim from the mechanism research, inserted in `OnFinishMove` at `Prediction.cpp:630`:

```cpp
constexpr int kGmPlayerOff = 0x08;   // CGameMovement::player
constexpr int kGmMvOff     = 0x10;   // CGameMovement::mv
int g_gm_ctx_ok = -1;                // -1 unknown / 1 verified / 0 MISMATCH
if (g_gm_ctx_ok < 0 && g_gm && player && movedata) {
    char* g = reinterpret_cast<char*>(g_gm);
    g_gm_ctx_ok = (*reinterpret_cast<void**>(g + kGmPlayerOff) == player
                && *reinterpret_cast<void**>(g + kGmMvOff)     == movedata) ? 1 : 0;
}
```
Surface in `Prediction::Diag` beside `installed`. Build this **first** — it is the zero-cost proof that the offsets survived a game update.

**G2 — convar resolve + write-back.** `Cvars` gains:
```cpp
namespace Cvars {
  struct Handle { const void* obj; int val_off; bool ok; };
  Handle Resolve(const char* name);          // reuses the validated layout
  bool   Get(const Handle&, float* out);
  bool   Set(const Handle&, float v);        // writes m_fValue AND m_nValue=(int)v
}
```
`Set` writes both halves of the validated `float m_fValue` / `int m_nValue` pair (the pair test in `Cvars.cpp` `ValuePairAt` is what proves the layout). Every convar the plan names is resolved, saved, written, **read back**, and restored. A convar that will not resolve or will not read back is recorded `MISSING` / `WRITE_REJECTED`; every stratum depending on it is marked `UNSILENCED` and its experiments grade only as composite.

**G3 — direct-call pin battery.** Per §PART 1 entries 0.5, 1.1–1.5, 1.10. Discriminative answers only — no "nothing happened" probes. `TracePlayerBBox` is pinned by *agreement with the already-validated `IEngineTrace`*, which is independent evidence, not a guessed constant. Failures → `UNPINNED` (never a mismatch).

**G4 — ambient-invariance test** (the real, non-tautological purity check). Every `kAccVtable` entry is called with **identical declared arguments** under **deliberately different ambient state** — post-stand-settle vs post-duck-settle, zero vs high velocity, different origin. Bit-identical output required. A difference means the argument list is incomplete and the entry is downgraded to `AMBIENT-DEPENDENT`.

**G5 — S0 baseline residue.** Void chamber, all convars zeroed, zero input: the tick must reduce to `pos += vel·dt` + `CategorizePosition` + `ReduceTimers` and nothing else, and the model must predict it bit-exactly. Failure = silencing did not work; the entire run is marked untrusted.

**G6 — lagged-dt check.** From the same S0 ballistic tick, recover `dt` from the position delta and compare against `interval_per_tick`. Disagreement means `GetLaggedMovementValue() ≠ 1` and every dt in the run is wrong. Void the batch.

**G7 — determinism repeats + canary.** The first 16 experiments of every stratum are re-run at the batch tail; dumps must be bit-identical. A fixed canary experiment runs every 64 experiments; any drift = residue leakage. **Consequence: no per-chunk guard hoisting.** Full `BeginStateGuard`/`EndStateGuard` per experiment, exactly as `RunOneFuzzProbeSEH` does today — a 7248-byte memcpy is nothing next to `ProcessMovement`, and G7 proves it.

**G8 — MXCSR.** Sample `_mm_getcsr()` at batch start **and** end; record both. `funcdiff` sets the same value via `_mm_setcsr` before replaying and refuses if the two samples differ. This closes the FTZ/DAZ hole that would otherwise report `1e-30f` denormals as physics defects.

**G9 — netvar write-back.** Every field the seed writes is read back immediately. `m_flGravity` in particular (the reason `--fuzz-grav1` exists) is recorded per experiment as `WRITE_OK` / `WRITE_REJECTED`; rejected fields downgrade their dependents rather than producing a fake green.

### 2.4 Probe schema — `funcprobe\\plan.csv`

One file (so one button reads one path). Record types are `$`-prefixed; `#` is a comment. Parsed with `sscanf_s` per line, same style as `Prediction::RunFuzz`.

```
# funcprobe plan v1 map=<stem> seed=<u32> settle=<S> test=<T>
$STRATUM,id,name,n_cvar,name0,val0,name1,val1,...            (up to 8 pairs)
$CHAMBER,id,kind,ax,ay,az,nx,ny,nz,note
$NOISE,scope,off                                             (learned mask, optional)
$LEAF,leaf_id,func_idx,arg_kind,a0,a1,a2,a3,a4,a5
$EXP,id,stratum,chamber,settle,test,
     ox,oy,oz,vx,vy,vz,bx,by,bz,
     ducked,ducking,ducktime,duckjumptime,jumptime,stamina,gravity,sfric,
     hullminz,hullmaxz,hullxy,punchx,punchy,
     yaw0,fmove0,smove0,upmove0,buttons0, ... (repeated `test` times)
```

**An experiment is `S` settle ticks + `T` test ticks — not a free-running trajectory.** Settle ticks carry zero input and exist because writing `FL_ONGROUND` does not ground the player (ground is `m_hGroundEntity`); the engine's own post-settle dump is the seed **both** sides start from. Defaults `S=1, T=4`. `T>1` is safe because the differ teacher-forces between every tick (§2.9), so each test tick is an independent one-tick datapoint — this is what turns 4000 probes × 5 ticks from ~4000 comparisons into ~4×N.

### 2.5 Runner loop

`FuncProbe.h`:
```cpp
namespace FuncProbe {
  int  Run(const char* plan_path, const char* out_dir);  // queues; -1 unavailable
  bool Busy();
  int  Progress(int* phase, int* done, int* total);       // for the UI
}
```

Executed inside `OnFinishMove`, chunked, beside the existing fuzz branch at `Prediction.cpp:656`:

```
if (s_fp_pending && player) {
  g_in_hook_work = true;
  switch (s_fp_phase) {
    case kGate:  RunGatesSEH(player);         break;   // G1..G8, once
    case kPin:   RunPinsSEH(player);          break;   // G3,G4,  once
    case kLeaf:  RunLeafChunkSEH(player);     break;   // $LEAF rows, chunked
    case kExp:   RunExpChunkSEH(player);      break;   // $EXP rows, chunked
    case kSweep: RunMapSweepChunk();          break;   // tracegen-map answers
    case kDone:  WriteAll(); s_fp_pending=0;  break;
  }
  g_in_hook_work = false;
  return;
}
```

One experiment (`RunOneExpSEH`, POD-only so SEH can wrap it, `__except` restoring exactly like `RunOneFuzzProbeSEH:414`):

1. `BeginStateGuard(player)` (forces `frametime = interval`)
2. save + write the stratum's convars; read back each
3. write the seed state through the resolved netvar offsets + `+0x1868`; read back each (G9)
4. `for s in 0..S-1`: build a zero `CUserCmd`, `SetupMove` → `ProcessMovement` → `g_orig_finishmove`, `curtime += interval`
5. **dump** the post-settle state (all four phase groups) + the pre-tick player snapshot
6. `for t in 0..T-1`: build the authored `CUserCmd`; snapshot player; `SetupMove`; dump `sm_*`; `ProcessMovement`; dump `pm_*`; `g_orig_finishmove`; dump `post_*`; delta-dump changed dwords vs the pre-snapshot (player + movedata), noise-masked; `curtime += interval`
7. emit the **8 canonical trace queries** computed arithmetically from the post-settle state and `dt` (categorize-down-2, full-dt sweep, step-up, step-down, stay-on-ground up/down, duck-fit, unduck-fit) and answer each with **both** `TracePlayerBBox` (VT 10) and `IEngineTrace` world-only → continuous cross-pin + the differ's geometry-substitution corpus
8. restore convars; `EndStateGuard()`

Chunk size 256 experiments/frame (each is ~5 `ProcessMovement` calls + 2 memcpy + 2 memcmp of 7248 bytes). 8192 experiments ≈ 32 frames.

### 2.6 Output CSVs — `Documents\\sourceTAS\\solver\\funcprobe\\`

**`manifest.csv`** — `key,value[,detail]`. The fail-closed record:
```
version,1
map,surf_basictest
build,2026-08-15 14:02:11
gm_ctx_ok,1
mxcsr_begin,8064
mxcsr_end,8064
tick_interval,0.015
lagged_dt_check,PASS,measured 0.0150000
maxspeed_live,250
player_size,7248
cvar.sv_gravity,OK,800,writeback_ok
cvar.sv_friction,OK,4,writeback_ok
cvar.sv_staminajumpcost,MISSING,,
pin.PlayerSolidMask,PASS,0x1400B/0x201400B
pin.TracePlayerBBox,PASS,agrees IEngineTrace 3/3 max dfrac 0.000000
pin.CanUnduck,UNPINNED,no C5 chamber on this map
ambient.GetPlayerMaxs_b,PASS
baseline_residue,PASS,dp 0.000000 dv 0.000000
determinism_repeats,PASS,16/16 bit-identical
canary_drift,PASS,0/128
exp_total,8192
exp_ok,8192
exp_faulted,0
```

**`state.csv`** — one wide row per `(exp,tick)`; `tick` is `-S..-1` for settle, `0..T-1` for test:
```
exp,tick,ok,
pre_ox,pre_oy,pre_oz,pre_vx,pre_vy,pre_vz,pre_bx,pre_by,pre_bz,pre_flags,
pre_ducked,pre_ducking,pre_ducktime,pre_duckjump,pre_jumptime,pre_stamina,
pre_sfric,pre_gravity,pre_minsz,pre_maxsz,pre_oldbtn,pre_groundent,pre_fallvel,
sm_fmove,sm_smove,sm_umove,sm_maxspeed,sm_clientmaxspeed,sm_buttons,sm_oldbuttons,
sm_ang_x,sm_ang_y,sm_ang_z,sm_ox,sm_oy,sm_oz,sm_vx,sm_vy,sm_vz,
pm_ox,pm_oy,pm_oz,pm_vx,pm_vy,pm_vz,pm_maxspeed,pm_oldbuttons,
pm_outwishvel_x,pm_outwishvel_y,pm_outjumpvel_z,pm_outstepheight,
post_ox,post_oy,post_oz,post_vx,post_vy,post_vz,post_bx,post_by,post_bz,post_flags,
post_ducked,post_ducking,post_ducktime,post_duckjump,post_jumptime,post_stamina,
post_sfric,post_gravity,post_minsz,post_maxsz,post_oldbtn,post_groundent,post_fallvel
```
The `sm_*` group (movedata after `SetupMove`, before `ProcessMovement`) is what makes `CheckParameters`, the duck speed crop, and `m_flMaxSpeed` selection **directly observable** instead of inferred.

**`delta.csv`** — the offset-independence layer:
```
exp,tick,scope,off,pre_u32,post_u32
```
`scope` 0 = player object, 1 = movedata. Only changed dwords not in the learned noise mask. This is what lets a field be discovered *after* the capture: you record dword `0x1868` because it changed, and offline observe that it takes only `{1.0, 0.25}` and correlates with rising-vs-falling. Pinned offsets move from "baked into the DLL" to "derived from the data, re-derivable after any patch."

**`leaf.csv`** — `leaf_id,func_idx,ok,r0,r1,r2,r3`
**`trace.csv`** — `exp,q,kind,ax,ay,az,bx,by,bz,hull,src,frac,ex,ey,ez,nx,ny,nz,startsolid,allsolid`
(`kind` 0..7 per §2.5 step 7; `src` 0 = VT10, 1 = IEngineTrace)
**`sweep.csv`** — the map-wide `tracegen-map` answers, same columns as today's `trace_results.csv`.

### 2.7 Offline differ — `SolverLab funcdiff <map.bsp> <funcprobe_dir> [--reach-only] [--dump N] [--func <name>]`

**(a) Refuse-to-grade gate.** Read `manifest.csv`. If `gm_ctx_ok != 1`, `baseline_residue != PASS`, `determinism_repeats != PASS`, `lagged_dt_check != PASS`, or `mxcsr_begin != mxcsr_end` → print why and exit 3. No partial credit.

**(b) Teacher-forced per-tick replay.** For each experiment, for each test tick `t`: build `PlayerState` from the **engine's** `post_*` row of tick `t−1` (for `t=0`, the settle row). Apply the stratum's convar values (as read back) into `MoveParams`. Run `MoveTickTraced`. Compare against the engine's `post_*` row of tick `t`. **Every test tick is an independent datapoint** — no free running, no `break` on first divergence, no accumulated error masquerading as a new defect.

**(c) Ablation ladder** on mismatch:

1. **Field forcing, PRE × POST cross-product.** For each forceable field `X ∈ {pos, vel, on_ground, ground_brush, ducked, ducking, duck_timer_ms, hull_state, surface_friction, old_buttons, stamina, gravity_scale, basevel, basevel_flag}`, re-run twice: once with `X` forced from the engine's tick-`t−1` post value (`X:PRE`), once from the engine's tick-`t` post value (`X:POST`). **Report the full repair set, ranked** — never the first repair.
   - repaired by `X:PRE` → the producer of `X` was wrong in an **earlier** tick; ownership is cross-tick
   - repaired by `X:POST` → `X` is wrong **this** tick and its consumer reads the post value → the producer runs **before** the consumer this tick
   - `|repair set| > 1` → verdict `AMBIGUOUS`, printed as a set. Attribution is a set; pretending it is a name is the guessing this instrument exists to kill.
   
   This is the correction the critique demanded: forcing only post-values would have **mis-attributed the surf-friction bug**, which is exactly the ownership rule (`m_surfaceFriction` written by the end-of-move categorize, consumed by the next tick).

2. **Trace substitution.** Re-run with every model trace whose query matches a row in `trace.csv` answered by the engine. Repairs → **geometry defect** (`SolverWorld`). Still fails → **math defect** (`SolverMove`). Queries the canonical set does not contain (TryPlayerMove bump ≥ 2) yield `TRACE-UNRESOLVED`; because `tracegen-map` already certifies geometry map-wide (6624/6624 exact), an unresolved substitution falls back to "geometry certified; treat as math" — stated in the report, never hidden.

3. **Checkpoint tap.** Compare `TickTrace` stage-by-stage against the stratum's analytic expectation. In a silenced stratum only one stage is active, so **the first stage that deviates from identity names the function directly** — no call-order assumption, because the *engine's* order produced the reference and our stages are compared against identity, not against each other.

**(d) Verdicts.** One row per `FuncSpec` entry in `funcdiff_attrib.csv`:
```
func,level,klass,stratum,verdict,n_exp,n_fail,repair_set,
first_fail_exp,first_fail_tick,worst_dp,worst_dv,note
```
`verdict ∈ {PASS, FAIL, AMBIGUOUS, UNCOVERED, UNSILENCED, UNPINNED, AMBIENT-DEPENDENT, OBSERVE-ONLY, OUT-OF-SCOPE}`. **`UNCOVERED` is never `PASS`.** A board is green only when every in-scope entry is `PASS` and the uncovered set is explicitly acknowledged.

**(e) Reachability, kept.** Per the critique: reachability is a *fact* about the solver's domain, interaction classes were *guesswork*. `--reach-only` splits the report by whether the seed state is inside the sealed world with a fitting hull; C6 (embedded) is deliberately unreachable and always reported separately. The eight-way "interaction class" table and the "defect signature" strings are **deleted** — strata replace them, and a stratum is a fact about what was silenced.

**(f) Composition rung.** `funcdiff` green does **not** imply a green tape — teacher forcing deliberately suppresses error accumulation. `SolverLab battery` and the tape decks remain the composition gate and must stay in the release checklist. This is the rung Design A had no answer for; it already exists, so keep it.

### 2.8 The one button

In `DrawMapSolveTab`, replacing the two separate buttons near `TasEditor.cpp:8566` and `:8657`:

```cpp
if (FuncProbe::Busy()) {
    int ph = 0, done = 0, total = 0;
    FuncProbe::Progress(&ph, &done, &total);
    ImGui::TextColored(Theme::Warning, "FUNCPROBE [%s] %d/%d...",
                       kPhaseName[ph], done, total);
} else if (ImGui::Button("Run FUNCPROBE (engine query)", ImVec2(300, 0))) {
    const std::string d = SolverDir() + "\\\\funcprobe";
    const int n = FuncProbe::Run((d + "\\\\plan.csv").c_str(), d.c_str());
    g_status = (n < 0)
        ? "FUNCPROBE: no plan (SolverLab funcgen) or no engine context."
        : FmtStr("FUNCPROBE: %d experiments queued...", n);
}
```

The button does gates → pins → leaves → experiments → map sweep in one press. Offline loop:
```
SolverLab funcgen <map.bsp>          -> funcprobe\\plan.csv (+ trace_queries.csv)
[ ONE in-game button press ]         -> funcprobe\\{manifest,state,delta,leaf,trace,sweep}.csv
SolverLab funcdiff <map.bsp> <dir>   -> funcdiff_attrib.csv + the board
```

### 2.9 Build order (each step has its own acceptance)

| # | Step | Acceptance |
|---|---|---|
| 1 | G1 gm-context latch + `Prediction::Diag` surfacing | `gm_ctx_ok == 1` shown in the menu in-game |
| 2 | Model refactor `SolverFn` + `MoveTickTraced` | **byte-identical** `battery` / `fuzzdiff` / tape-deck CSVs before vs after |
| 3 | `Cvars::Resolve/Get/Set` + write-back readback | manifest lists every planned convar with `writeback_ok` or an honest `MISSING` |
| 4 | `TraceOracle` extraction from `TasEditor.cpp` | `Run trace oracle` still produces a byte-identical `trace_results.csv` |
| 5 | `FuncSpec` registry + `funcgen` chamber discovery | plan generated; chambers it could **not** build are printed, not silently skipped |
| 6 | `FuncProbe` gates G2–G8, no experiments yet | S0 baseline residue `PASS` at 0.000000; G7 repeats bit-identical |
| 7 | `$LEAF` direct-call pins G3/G4 | `PlayerSolidMask` exact constants; `TracePlayerBBox` agrees with `IEngineTrace` |
| 8 | `$EXP` runner + all five output files | 8192 experiments, 0 faults, canary flat |
| 9 | `funcdiff` teacher-forced replay only | reproduces today's known-green results on S15 composite |
| 10 | Ablation ladder + trace substitution + checkpoint tap | re-derives green-board item 4's ownership rule **from the data alone** — the regression test for the whole instrument |

### 2.10 Bugs in the existing code, to fix regardless of this plan

**`SolverLab.cpp:1267` / `:1272` — the "pre" state printed in every fuzz divergence dump is OUR model's tick-`t−1` output, not the engine's.** `s_at_fail = s;` captures our `PlayerState` after our own previous tick. The engine's true pre-state is sitting unused at `res[i*kFuzzTicks + t - 1]`. Every forensic dump read since the fuzz harness landed anchored on a state the engine never occupied. Four-line fix: build a `PlayerState` from `res[i*kFuzzTicks + t - 1]` for the dump — and, better, re-seed `s` from it each tick, which converts `fuzzdiff` to teacher forcing and removes the `break;` on first divergence at `:1352`. Do this before or alongside the new instrument; keep `fuzzgen`/`fuzzdiff` for one release as a control arm, then retire.

---

## PART 3 — WHAT THIS DESIGN CANNOT VERIFY (no hiding)

1. **All server-only code.** We hook client prediction. `CPlayerMove::RunCommand`, `CheckMovingGround` (the `(1 + dt/2)` base-velocity fold and conveyors), `RunPreThink/RunThink/RunPostThink`, fall damage, and **all trigger Touch functions** never execute in our context. Our trigger port (`MoveTick:669-693`) is certified only by full-tape playback capture on a map that has triggers — which `basictest` does not.
2. **Client/server binary divergence.** The jump impulse `301.99337741082996`, the stamina constants, and the `FinishUnDuck` unconditional shift were decoded from **server.dll**; this instrument fuzzes **client.dll**. Any place the two binaries differ is invisible here and shows up only in real playback capture.
3. **Inlined functions have no callable body.** `Friction` is inlined (`0x11ADB0`). No leaf is ever verified by direct call — every leaf verdict is a *stratum* verdict, i.e. "this function's contribution, isolated by silencing everything else, matches." That is weaker than a unit test and stronger than an end-to-end diff, and it is the honest ceiling.
4. **`CategorizePosition`, `CheckParameters`, `ReduceTimers`, `PlayerMove` cannot be silenced.** They run every tick. Their verdicts come from S0/S11/S12/S14 residue plus ablation, so a defect in one can in principle be attributed to another. `|repair set| > 1` → `AMBIGUOUS`, and the report says so rather than picking a name.
5. **Ground cannot be *set*, only *achieved*.** `m_hGroundEntity` is a handle; writing `FL_ONGROUND` does not ground the player (measured: 96% of "grounded" probes were airborne). So "grounded in the void" is unconstructible; `Friction`'s grounded path is only testable on a real floor, where `WalkMove`, `StayOnGround` and `CategorizePosition` also run.
6. **`CategorizeGroundSurface` / surfaceprops-derived friction is structurally uncoverable on a single-material map.** Our model emits only `{1.0, 0.25}`. Real maps have ice (0.1 → 0.125), dirt, metal (0.8 → 1.0 after the ×1.25 and clamp). Reading the true value needs the BSP texinfo→texdata→surfaceprop chain *and* the surfaceprops KV files from the VPK — neither of which `SolverWorld` parses. **Any map with more than one surface material is out of model, and this fuzzer will not tell you.**
7. **`CheckStuck` is history-dependent.** The unstuck offset walk is indexed by entity index and a persistent counter, so the displacement is not a pure function of position. C6 experiments are graded `OBSERVE-ONLY` until that counter is located and included in the state guard. Note also that a stuck tick makes `PlayerMove` **early-return** — no movement, no categorize — where our model zeroes velocity and continues.
8. **Water, ladders, noclip, toss, observer, vehicles, trains, `m_flDuckJumpTime`/`m_flJumpTime` paths** — no experiments are emitted. Reported `UNCOVERED`. A green board with these uncovered means "green over MOVETYPE_WALK, no water, no ladders," and the report must be read that way.
9. **`PlayerSolidMask` content bits we do not model.** `CONTENTS_MOVEABLE`, `CONTENTS_WINDOW`, `CONTENTS_MONSTERCLIP` are in `MASK_PLAYERSOLID` and are absent from `BspWorld.cpp:19-22`. The direct-call pin will *prove the mask value*, but only geometry that actually carries those contents can prove the *consequence*. Nothing on a clean surf map will.
10. **Trace substitution has a hole.** The canonical 8 queries cover bump 1 of `TryPlayerMove` plus the step/ground sweeps. Deeper bumps depend on earlier bumps' results and cannot be pre-computed by the runner. Those experiments report `TRACE-UNRESOLVED`; the geometry/math split falls back on the map-wide sweep's certification rather than a per-experiment answer.
11. **Composition and error accumulation.** Teacher forcing makes every tick independent — deliberately. A green `funcdiff` does not imply a green tape. `battery` and the tape decks remain the only evidence that per-tick agreement composes.
12. **The delta dump observes *changes*, not *reads*.** A field the engine reads but never writes is invisible to it. The ambient-invariance test (G4) partially covers this for the direct-call entries; for everything else, an unread-but-influential field can only be found by a stratum that happens to vary it.
13. **MXCSR is sampled, not controlled.** If another DLL changes it mid-batch we detect it (`mxcsr_begin != mxcsr_end` → refuse) but cannot prevent it.
14. **Convar writes bypass change callbacks.** Writing `m_fValue`/`m_nValue` directly does not update `m_pszString` and does not fire replicated-convar callbacks. Movement code reads `GetFloat()` → `m_fValue`, so the physics is right; anything that reads the *string* or reacts to the *callback* is not. Within a single hook invocation this has no known consumer, but it is an assumption, and the read-back check does not test it.

---

## PART 4 — What each source design contributed

- **From A:** the shared registry compiled into both binaries; the refactor acceptance gate (byte-identical CSVs — the single best line in either document); the gm-context latch and fail-closed posture; probe-then-pin discipline.
- **From B:** isolation through the real `ProcessMovement` as the primary instrument; the full-object delta dump (offsets become derived data, not baked constants); per-field forcing as the attribution engine; convar strata; the baseline residue gate; folding the trace oracle into the same press.
- **From the critique, adopted verbatim:** delete the input/output slot vectors (they encode answers as premises); keep reachability, kill interaction classes; no per-chunk guard hoisting; discriminative pins only; force from **both** PRE and POST and report the **full repair set** (the correction that makes the surf-friction ownership rule discoverable rather than mis-attributed); `kF64` for the baked double; MXCSR matched offline; and the `s_at_fail` fix in the existing differ.
- **Rejected from both:** direct-calling `Friction`/`Accelerate`/`ClipVelocity` at RVAs (inlined and/or order-dependent, with a wrong signature corrupting the stack inside live prediction), and A's `kAccInlined` "isolation by nulling," which assumes the very call order under test.",
    "inventory": {
      "functions": [
        {
          "name": "CPlayerMove::RunCommand",
          "klass": "other",
          "signature": "void CPlayerMove::RunCommand( CBasePlayer *player, CUserCmd *ucmd, IMoveHelper *moveHelper )",
          "inputs": "CUserCmd (viewangles, forwardmove, sidemove, upmove, buttons, impulse, tick_count, weaponselect, mousedx/dy); player->m_nTickBase; TICK_INTERVAL; player->m_bGamePaused; player->GetVehicle()",
          "outputs": "Sets gpGlobals->curtime = m_nTickBase*TICK_INTERVAL and gpGlobals->frametime = TICK_INTERVAL (0 if paused). Drives the whole tick: StartCommand -> UpdateButtonState -> CheckMovingGround -> RunPreThink -> RunThink -> SetupMove -> ProcessMovement -> FinishMove -> ProcessImpacts -> RunPostThink.",
          "purity": "mutates-member-state",
          "callable_directly": "no - it is the tick driver; it writes gpGlobals and requires a live CBasePlayer, CUserCmd and IMoveHelper",
          "notes": "THE entry point of one movement tick. Not documented on VDC at all. gpGlobals->frametime is exactly TICK_INTERVAL here, NOT the render frametime - this is why a TAS tick is a fixed dt."
        },
        {
          "name": "CPlayerMove::StartCommand",
          "klass": "other",
          "signature": "void CPlayerMove::StartCommand( CBasePlayer *player, CUserCmd *cmd )",
          "inputs": "CUserCmd, player",
          "outputs": "player->m_pCurrentCommand = cmd; RandomSeed( cmd->random_seed ); resets prediction error tracking",
          "purity": "mutates-member-state",
          "callable_directly": "no - sets per-command globals",
          "notes": "Seeds the shared RNG so client and server prediction agree. Irrelevant to movement math but relevant to determinism."
        },
        {
          "name": "CBasePlayer::UpdateButtonState",
          "klass": "CBasePlayer",
          "signature": "void CBasePlayer::UpdateButtonState( int nUserCmdButtonMask )",
          "inputs": "ucmd->buttons; m_nButtons (previous)",
          "outputs": "m_afButtonLast, m_nButtons, m_afButtonPressed = (buttons & ~last), m_afButtonReleased = (~buttons & last)",
          "purity": "mutates-member-state",
          "callable_directly": "yes - pure bit arithmetic over two ints, trivially reimplementable",
          "notes": "Distinct from mv->m_nOldButtons, which CGameMovement maintains separately inside FinishMove. The duck/jump edge logic uses mv->m_nOldButtons, not these."
        },
        {
          "name": "CPlayerMove::CheckMovingGround",
          "klass": "other",
          "signature": "void CPlayerMove::CheckMovingGround( CBasePlayer *player, double frametime )",
          "inputs": "player flags FL_ONGROUND / FL_BASEVELOCITY; ground entity flags (FL_CONVEYOR); groundentity->GetGroundVelocityToApply(); player->GetBaseVelocity(); frametime",
          "outputs": "If on a conveyor: SetBaseVelocity(conveyorVel [+ existing basevel]) and AddFlag(FL_BASEVELOCITY). If FL_BASEVELOCITY is NOT set: ApplyAbsVelocityImpulse( (1.0 + frametime*0.5) * GetBaseVelocity() ) then SetBaseVelocity(0). Always ends with RemoveFlag(FL_BASEVELOCITY).",
          "purity": "mutates-member-state",
          "callable_directly": "unknown - logic is simple but it depends on the ground entity's velocity, which needs the entity system",
          "notes": "THIS is the base-velocity mechanism. trigger_push sets basevel + FL_BASEVELOCITY during touch; the NEXT tick's CheckMovingGround sees the flag was set, so it does not fold it in, and clears the flag. If the trigger does not re-touch, the tick after that folds basevel into abs velocity with the (1 + dt/2) overshoot factor and zeroes it. VDC's Velocity page documents the three-velocity model (local/base/abs) but not this function."
        },
        {
          "name": "CPlayerMove::RunPreThink / RunThink / RunPostThink",
          "klass": "other",
          "signature": "void RunPreThink( CBasePlayer * ); void RunThink( CBasePlayer *, double frametime ); void RunPostThink( CBasePlayer * )",
          "inputs": "player think schedule, m_nNextThinkTick",
          "outputs": "Calls player->PreThink() before movement and player->PostThink() after. PostThink is where CBasePlayer::UpdateStepSound tail work, fall damage bookkeeping, and PhysicsTouchTriggers-driven trigger entry/exit resolve.",
          "purity": "mutates-member-state",
          "callable_directly": "no - full entity/think system",
          "notes": "Trigger volumes (push/gravity/teleport) fire from the touch pass around here, not inside CGameMovement."
        },
        {
          "name": "CPlayerMove::SetupMove",
          "klass": "other",
          "signature": "void CPlayerMove::SetupMove( CBasePlayer *player, CUserCmd *ucmd, IMoveHelper *pHelper, CMoveData *move )",
          "inputs": "player->GetAbsVelocity(), GetNetworkOrigin(), pl.v_angle, m_flMaxspeed, GetPlayerMaxSpeed(), m_flConstraintRadius/Width/SpeedFactor; ucmd fields",
          "outputs": "Fills CMoveData: m_vecVelocity, SetAbsOrigin, m_vecOldAngles, m_vecViewAngles, m_vecAngles, m_flForwardMove/m_flSideMove/m_flUpMove, m_nButtons, m_nOldButtons (from player->m_Local.m_nOldButtons), m_nImpulseCommand, m_flClientMaxSpeed, m_flMaxSpeed, m_bGameCodeMovedPlayer, constraint fields",
          "purity": "mutates-member-state",
          "callable_directly": "unknown - it is a pure struct copy but reads ~15 player members",
          "notes": "The player->CMoveData marshalling boundary. In the user's DLL this is the hooked vtable index 18. CMoveData has no VDC page."
        },
        {
          "name": "CGameMovement::ProcessMovement",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::ProcessMovement( CBasePlayer *pPlayer, CMoveData *pMove )",
          "inputs": "pPlayer, pMove, gpGlobals->frametime, pPlayer->GetLaggedMovementValue(), pPlayer->GetPlayerMaxSpeed()",
          "outputs": "Saves frametime; gpGlobals->frametime *= GetLaggedMovementValue(); ResetGetPointContentsCache(); m_iSpeedCropped = SPEED_CROPPED_RESET; sets members player=pPlayer and mv=pMove; mv->m_flMaxSpeed = GetPlayerMaxSpeed(); calls PlayerMove(); calls FinishMove(); restores gpGlobals->frametime.",
          "purity": "mutates-member-state",
          "callable_directly": "no - it installs the `player` and `mv` members every other function reads; that IS the calling convention",
          "notes": "Top of the movement stack proper. Note the frametime scaling by GetLaggedMovementValue() (player_speedmod / slow-motion) happens here and is invisible to every downstream function - they just see a different dt. VDC has no page for this."
        },
        {
          "name": "CCSGameMovement::ProcessMovement",
          "klass": "CCSGameMovement",
          "signature": "void CCSGameMovement::ProcessMovement( CBasePlayer *pBasePlayer, CMoveData *pMove )",
          "inputs": "pBasePlayer",
          "outputs": "m_pCSPlayer = ToCSPlayer( pBasePlayer ); then BaseClass::ProcessMovement",
          "purity": "mutates-member-state",
          "callable_directly": "no",
          "notes": "Only caches the CS player pointer so the CS overrides (stamina, duck fraction, bunnyhop clamp) can reach m_flStamina."
        },
        {
          "name": "CGameMovement::PlayerMove",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::PlayerMove( void )",
          "inputs": "mv, player, player->GetMoveType(), player->pl.deadflag, player->GetFlags() (FL_ONTRAIN), mv->m_vecVelocity.z, sv_optimizedmovement, player->GetWaterLevel(), player->GetGroundEntity()",
          "outputs": "Ordered side effects: (1) CheckParameters(); (2) mv->m_outWishVel.Init(), mv->m_outJumpVel.Init(); (3) MoveHelper()->ResetTouchList(); (4) ReduceTimers(); (5) AngleVectors(mv->m_vecViewAngles, &m_vecForward,&m_vecRight,&m_vecUp); (6) if movetype not in {NOCLIP,NONE,ISOMETRIC,OBSERVER} and alive: CheckInterval(STUCK) then CheckStuck() - early-returns if stuck; (7) CategorizePosition() UNLESS (movetype==WALK && !m_bGameCodeMovedPlayer && sv_optimizedmovement), in which case it only does SetGroundEntity(NULL) when velocity.z > 250; (8) m_nOldWaterLevel = GetWaterLevel(); (9) if airborne, m_Local.m_flFallVelocity = -velocity.z; (10) m_nOnLadder = 0; (11) player->UpdateStepSound(...); (12) UpdateDuckJumpEyeOffset(); (13) Duck(); (14) LadderMove() unless dead/FL_ONTRAIN, demoting MOVETYPE_LADDER back to WALK if it returns false; (15) switch on movetype -> FullWalkMove / FullNoClipMove / FullTossMove / FullLadderMove / FullObserverMove.",
          "purity": "mutates-member-state",
          "callable_directly": "no - requires mv/player installed by ProcessMovement",
          "notes": "THE call-order spine. Two things matter enormously for a TAS: (a) with sv_optimizedmovement 1 (the shipped default) the OPENING CategorizePosition is SKIPPED for MOVETYPE_WALK - ground state entering the tick is whatever last tick's closing CategorizePosition left, and the only opening adjustment is the vz>250 unground; (b) Duck() runs BEFORE any velocity work, so a duck this tick changes the hull the whole move is traced with."
        },
        {
          "name": "CGameMovement::CheckParameters",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::CheckParameters( void )",
          "inputs": "mv->m_flForwardMove/m_flSideMove/m_flUpMove, mv->m_flMaxSpeed, player->m_pSurfaceData->game.maxSpeedFactor, ComputeConstraintSpeedFactor(), player->GetFlags() (FL_FROZEN, FL_ONTRAIN), IsDead(), mv->m_vecAngles, player->m_Local.m_vecPunchAngle, sv_rollangle, sv_rollspeed, g_bMovementOptimizations",
          "outputs": "mv->m_flMaxSpeed *= surface/constraint speed factor; if |(fmove,smove,umove)| > m_flMaxSpeed, scales all three by m_flMaxSpeed/spd; zeroes all three if FL_FROZEN/FL_ONTRAIN/dead; DecayPunchAngle(); mv->m_vecAngles[PITCH/YAW] from view+punch, mv->m_vecAngles[ROLL] = CalcRoll(...)*4; wraps YAW into (-180,180]",
          "purity": "mutates-member-state",
          "callable_directly": "unknown - the math is self-contained but it reads surface data and punch angle",
          "notes": "This is the FIRST clamp on the wish vector, and it is a 3D clamp including upmove. It runs before any duck crop. With g_bMovementOptimizations the sqrt is skipped by comparing squares - numerically identical outcome, but the branch matters if you are matching bit-for-bit."
        },
        {
          "name": "CGameMovement::DecayPunchAngle",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::DecayPunchAngle( void )",
          "inputs": "player->m_Local.m_vecPunchAngle, m_vecPunchAngleVel, gpGlobals->frametime",
          "outputs": "Spring-damps m_vecPunchAngle toward zero (vel += -spring*angle*dt - damping*vel; angle += vel*dt), snapping to zero below threshold",
          "purity": "mutates-member-state",
          "callable_directly": "yes - pure function of two vectors and dt",
          "notes": "Affects mv->m_vecAngles (via CheckParameters) and therefore the forward/right basis. For a TAS with no recoil this is a no-op, but it is NOT a no-op after shooting."
        },
        {
          "name": "CGameMovement::CalcRoll",
          "klass": "CGameMovement",
          "signature": "float CGameMovement::CalcRoll( const QAngle &angles, const Vector &velocity, float rollangle, float rollspeed )",
          "inputs": "view angles, velocity, sv_rollangle, sv_rollspeed",
          "outputs": "Returns a roll angle (sign of velocity dot right vector, ramped)",
          "purity": "pure",
          "callable_directly": "yes - genuinely pure",
          "notes": "sv_rollangle defaults to 0 in the Console Command List, so this returns 0 and roll is cosmetic. Listed for completeness."
        },
        {
          "name": "CGameMovement::ComputeConstraintSpeedFactor",
          "klass": "CGameMovement",
          "signature": "float CGameMovement::ComputeConstraintSpeedFactor( void )",
          "inputs": "mv->m_flConstraintRadius, m_flConstraintWidth, m_flConstraintSpeedFactor, m_vecConstraintCenter, mv->GetAbsOrigin()",
          "outputs": "Returns 1.0 if no constraint, else a 1..speedFactor ramp based on distance from the constraint center",
          "purity": "pure",
          "callable_directly": "yes",
          "notes": "Used for HL2 vortigaunt/strider leashes. Always 1.0 in CS:S surf/bhop."
        },
        {
          "name": "IMoveHelper::ResetTouchList",
          "klass": "other",
          "signature": "void IMoveHelper::ResetTouchList( void )",
          "inputs": "none",
          "outputs": "Clears the per-tick touch list that TryPlayerMove and SetGroundEntity append to",
          "purity": "mutates-member-state",
          "callable_directly": "no - interface into the move helper singleton",
          "notes": "The touch list is drained later by moveHelper->ProcessImpacts(), which is what actually delivers Touch() calls to trigger_push etc."
        },
        {
          "name": "CGameMovement::ReduceTimers",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::ReduceTimers( void )",
          "inputs": "gpGlobals->frametime; player->m_Local.m_flDucktime, m_flDuckJumpTime, m_flJumpTime; player->m_flSwimSoundTime",
          "outputs": "frame_msec = 1000*frametime; decrements each of the four timers by frame_msec, clamping at 0",
          "purity": "mutates-member-state",
          "callable_directly": "yes - four independent clamped subtractions",
          "notes": "All four are MILLISECOND timers counting DOWN. m_flDucktime starts at GAMEMOVEMENT_DUCK_TIME (1000). Duck() converts it to elapsed via (1000 - m_flDucktime). Critically, this runs BEFORE Duck(), so the tick a duck is pressed already has one frame_msec drained."
        },
        {
          "name": "CCSGameMovement::ReduceTimers",
          "klass": "CCSGameMovement",
          "signature": "void CCSGameMovement::ReduceTimers( void )",
          "inputs": "m_pCSPlayer->m_flStamina, gpGlobals->frametime",
          "outputs": "m_flStamina -= 1000*frametime, clamped at 0; then BaseClass::ReduceTimers()",
          "purity": "mutates-member-state",
          "callable_directly": "yes",
          "notes": "Stamina is a millisecond countdown like the duck timers. User's binary-decode confirms drain is 1000*frametime per tick everywhere."
        },
        {
          "name": "AngleVectors",
          "klass": "other",
          "signature": "void AngleVectors( const QAngle &angles, Vector *forward, Vector *right, Vector *up )",
          "inputs": "pitch/yaw/roll in degrees",
          "outputs": "Orthonormal basis. forward = (cp*cy, cp*sy, -sp); right = (-1*sr*sp*cy + -1*cr*-sy, ...); up likewise.",
          "purity": "pure",
          "callable_directly": "yes - the single most reusable function in the whole stack",
          "notes": "Called at least three times per tick: once in PlayerMove into m_vecForward/m_vecRight/m_vecUp, and again independently inside WalkMove and AirMove into LOCAL forward/right. The local ones are the ones that get z-zeroed and renormalized; the members are not. Uses SinCos on degrees*(M_PI/180) in float."
        },
        {
          "name": "CGameMovement::CheckInterval",
          "klass": "CGameMovement",
          "signature": "bool CGameMovement::CheckInterval( IntervalType_t type )",
          "inputs": "gpGlobals->tickcount, player entindex, s_CheckInterval table (STUCK=1, GROUND=1, LADDER=1)",
          "outputs": "Returns true when (tickcount + entindex) % interval == 0",
          "purity": "reads-member-state",
          "callable_directly": "yes",
          "notes": "With interval 1 for STUCK this is always true in practice, so CheckStuck runs every tick. The entindex offset staggers players across ticks when intervals are > 1."
        },
        {
          "name": "CGameMovement::CheckStuck",
          "klass": "CGameMovement",
          "signature": "int CGameMovement::CheckStuck( void )",
          "inputs": "mv->GetAbsOrigin(), player hull (GetPlayerMins/Maxs, which read m_bDucked), COLLISION_GROUP_PLAYER_MOVEMENT, rgv3tStuckTable (54 entries), m_StuckLast, player->IsServer(), gpGlobals->realtime",
          "outputs": "Returns 0 if not stuck (and resets m_StuckLast). If stuck: walks the 54-entry stuck offset table, and on finding a free spot writes mv->SetAbsOrigin(test) and returns 0. If nothing frees it, returns 1 and PlayerMove EARLY-RETURNS - the entire tick is skipped, no gravity, no friction, no move.",
          "purity": "mutates-member-state",
          "callable_directly": "unknown - the table walk is deterministic and reimplementable, but it depends on TestPlayerPosition and on the client/server idx split",
          "notes": "Return 1 is the total-freeze case a TAS must model: velocity is NOT zeroed, position does not change, and the next tick tries again. The stuck table is generated once by CreateStuckTable and is deterministic (not random despite GetRandomStuckOffsets's name)."
        },
        {
          "name": "CGameMovement::CreateStuckTable",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::CreateStuckTable( void )",
          "inputs": "none (guarded by a static bool)",
          "outputs": "Fills static Vector rgv3tStuckTable[54] with a fixed pattern: 0, then +/-x, +/-y, +/-z at 0.25 and 0.5 scale, then the 27-point cube at scale 2 and 4",
          "purity": "mutates-member-state",
          "callable_directly": "yes - it is a constant table generator with no inputs",
          "notes": "Deterministic and identical on client and server. Worth precomputing verbatim in a solver."
        },
        {
          "name": "CGameMovement::GetRandomStuckOffsets / ResetStuckOffset",
          "klass": "CGameMovement",
          "signature": "int GetRandomStuckOffsets( CBasePlayer *pPlayer, Vector &offset ); void ResetStuckOffset( CBasePlayer *pPlayer )",
          "inputs": "player entindex, static rgStuckLast[MAX_CLIENTS][2] counter, rgv3tStuckTable",
          "outputs": "offset = rgv3tStuckTable[ idx ]; increments the per-player counter modulo 54. Reset zeroes the counter.",
          "purity": "mutates-member-state",
          "callable_directly": "yes",
          "notes": "Named 'Random' but is a round-robin index. The [2] second dimension is the client/server split so prediction stays in sync."
        },
        {
          "name": "CGameMovement::TestPlayerPosition",
          "klass": "CGameMovement",
          "signature": "CBaseHandle CGameMovement::TestPlayerPosition( const Vector& pos, int collisionGroup, trace_t& pm )",
          "inputs": "pos, collisionGroup, current hull via GetPlayerMins/GetPlayerMaxs (which read player->m_Local.m_bDucked), PlayerSolidMask()",
          "outputs": "Runs TracePlayerBBox(pos, pos, ...) - a zero-length hull trace. Returns the hit entity handle (INVALID_ENTITY_HANDLE if free), fills pm.",
          "purity": "reads-member-state",
          "callable_directly": "unknown - trivially reimplementable given a trace oracle, but it reads m_bDucked implicitly",
          "notes": "Zero-length hull trace = 'am I embedded right now'. Used by CheckStuck and FixPlayerCrouchStuck. Note startsolid is the meaningful field, not fraction."
        },
        {
          "name": "CGameMovement::CategorizePosition",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::CategorizePosition( void )",
          "inputs": "mv->GetAbsOrigin(), mv->m_vecVelocity.z, player->IsObserver(), player->GetMoveType(), player->GetGroundEntity() and its AbsVelocity.z, GetPlayerMins()/GetPlayerMaxs(), MASK_PLAYERSOLID, COLLISION_GROUP_PLAYER_MOVEMENT, NON_JUMP_VELOCITY (140.0f), walkable normal.z threshold 0.7",
          "outputs": "UNCONDITIONALLY sets player->m_surfaceFriction = 1.0f first. Calls CheckWater(). Returns early for observers. Then: if zvel > 140 (adjusted by ground entity's own vz) OR (moving up on a ladder) -> SetGroundEntity(NULL). Else TryTouchGround(origin, origin-2z, ...); if no entity or plane.normal.z < 0.7, retry with TryTouchGroundInQuadrants; if still no walkable plane -> SetGroundEntity(NULL) and, IF velocity.z > 0 and movetype != NOCLIP, set player->m_surfaceFriction = 0.25f. Otherwise SetGroundEntity(&pm), which calls CategorizeGroundSurface and zeroes velocity.z.",
          "purity": "mutates-member-state",
          "callable_directly": "no - needs the trace oracle and mutates ground entity + friction",
          "notes": "THE most important function for surf/bhop. Three separate things live here: ground detection, the 140 u/s unground gate, and m_surfaceFriction. The 0.25 airborne-rising friction is exactly what the user's repo fuzz-measured (air_friction_up=0.25) and it is set HERE, not in Friction(). Because friction is reset to 1.0 at the top and only re-set at the bottom, the value every Accelerate/AirAccelerate sees is whatever the PREVIOUS CategorizePosition left - which is why the repo's board note says 'friction is owned by the end-of-move categorize'. The 2-unit probe offset (flOffset = 2.0f) is fixed. VDC does not document any of this; sv_standable_normal/sv_walkable_normal (CS:GO-only cvars) expose the 0.7 constant."
        },
        {
          "name": "CGameMovement::TryTouchGround",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::TryTouchGround( const Vector& start, const Vector& end, const Vector& mins, const Vector& maxs, unsigned int fMask, int collisionGroup, trace_t& pm )",
          "inputs": "start, end, explicit mins/maxs (NOT the implicit hull), fMask, collisionGroup",
          "outputs": "Builds Ray_t::Init(start, end, mins, maxs), runs enginetrace->TraceRay with CTraceFilterSimple(player, collisionGroup), fills pm",
          "purity": "pure",
          "callable_directly": "yes - it is a thin trace wrapper; only needs a trace oracle",
          "notes": "Takes mins/maxs explicitly precisely so TryTouchGroundInQuadrants can shrink the box. This is the one ground probe in the whole system."
        },
        {
          "name": "CGameMovement::TryTouchGroundInQuadrants",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::TryTouchGroundInQuadrants( const Vector& start, const Vector& end, unsigned int fMask, int collisionGroup, trace_t& pm )",
          "inputs": "start, end, GetPlayerMins()/GetPlayerMaxs(), incoming pm.fraction and pm.endpos",
          "outputs": "Saves pm.fraction/pm.endpos. Tries four quarter-footprint boxes in turn: (-x,-y) = mins .. (min(0,maxs.x), min(0,maxs.y), maxs.z); then (+x,-y); (-x,+y); (+x,+y). Returns as soon as one finds pm.m_pEnt with plane.normal.z >= 0.7. In ALL cases restores pm.fraction and pm.endpos to the saved values before returning - only the plane/entity survive.",
          "purity": "pure",
          "callable_directly": "yes - needs only a trace oracle",
          "notes": "This is the 'standing on an edge / straddling two slopes' rescue. It is why you can stand with 3/4 of your hull over a void. The fraction/endpos restore is a subtle detail: the caller must not use the quadrant trace's endpos."
        },
        {
          "name": "CGameMovement::SetGroundEntity",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::SetGroundEntity( trace_t *pm )",
          "inputs": "pm (may be NULL), player->GetGroundEntity() (old), player->GetBaseVelocity(), newGround->GetAbsVelocity(), oldGround->GetAbsVelocity()",
          "outputs": "On ground gained: basevel -= newGround->AbsVelocity, basevel.z = newGround->AbsVelocity.z. On ground lost: basevel += oldGround->AbsVelocity, basevel.z = oldGround->AbsVelocity.z. SetBaseVelocity(basevel); player->SetGroundEntity(newGround). If newGround: CategorizeGroundSurface(*pm) (which OVERWRITES m_surfaceFriction from surfaceprops), player->m_flWaterJumpTime = 0, AddToTouched if it did not hit world, and mv->m_vecVelocity.z = 0.0f.",
          "purity": "mutates-member-state",
          "callable_directly": "no - mutates ground entity, base velocity and surface friction",
          "notes": "Landing zeroes vz HERE, inside SetGroundEntity, in addition to FullWalkMove's own explicit zeroing. The moving-platform basevel handoff is the mechanism that lets you keep a train's velocity when you jump off it."
        },
        {
          "name": "CGameMovement::CategorizeGroundSurface",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::CategorizeGroundSurface( trace_t &pm )",
          "inputs": "pm.surface.surfaceProps, IPhysicsSurfaceProps from MoveHelper()->GetSurfaceProps()",
          "outputs": "player->m_surfaceProps = pm.surface.surfaceProps; player->m_pSurfaceData = GetSurfaceData(...); reads physics friction into player->m_surfaceFriction; then m_surfaceFriction *= 1.25f and is clamped to <= 1.0f; player->m_chTextureType = m_pSurfaceData->game.material",
          "purity": "mutates-member-state",
          "callable_directly": "unknown - needs the surfaceprops database, which comes from the material system not the BSP",
          "notes": "The other writer of m_surfaceFriction. For default concrete the vphysics friction is 0.8, so 0.8*1.25 = 1.0 exactly - which is why standard ground reads exactly 1.0. Ice/slick surfaces land below 1.0 and scale friction AND acceleration. Also sets m_pSurfaceData->game.jumpFactor and .maxSpeedFactor, which CheckJumpButton and CheckParameters read."
        },
        {
          "name": "CBasePlayer::UpdateStepSound",
          "klass": "CBasePlayer",
          "signature": "void CBasePlayer::UpdateStepSound( surfacedata_t *psurface, const Vector &vecOrigin, const Vector &vecVelocity )",
          "inputs": "surface data, origin, velocity, m_flStepSoundTime, water level, ducked/walking state",
          "outputs": "Decrements m_flStepSoundTime, plays footsteps, sets m_iStepSide",
          "purity": "mutates-member-state",
          "callable_directly": "no - audio side effects",
          "notes": "Purely cosmetic for movement; included because it sits between CategorizePosition and Duck in PlayerMove's order and can be safely stubbed in a solver."
        },
        {
          "name": "CGameMovement::UpdateDuckJumpEyeOffset",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::UpdateDuckJumpEyeOffset( void )",
          "inputs": "player->m_Local.m_flDuckJumpTime, GAMEMOVEMENT_DUCK_TIME (1000.0f), TIME_TO_UNDUCK (0.2f)",
          "outputs": "If m_flDuckJumpTime != 0: flDuckSeconds = (1000 - m_flDuckJumpTime)/1000; if that exceeds TIME_TO_UNDUCK, zero m_flDuckJumpTime and SetDuckedEyeOffset(0.0f); else SetDuckedEyeOffset( SimpleSpline( 1 - flDuckSeconds/TIME_TO_UNDUCK ) )",
          "purity": "mutates-member-state",
          "callable_directly": "yes - given the timer value; only writes the view offset",
          "notes": "VIEW ONLY - it does not touch the hull or the origin. Safe to skip in a physics-only solver. Note the units bug-shaped detail: it divides by GAMEMOVEMENT_DUCK_TIME (1000) rather than multiplying by 0.001, same result."
        },
        {
          "name": "CGameMovement::Duck",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::Duck( void )",
          "inputs": "mv->m_nButtons, mv->m_nOldButtons (for buttonsChanged/Pressed/Released), player->GetGroundEntity() (bInAir), player->GetFlags()&FL_DUCKING (bInDuck), player->m_Local.m_flJumpTime (bDuckJump), m_flDuckJumpTime (bDuckJumpTime), m_bDucking, m_bDucked, m_bInDuckJump, m_flDucktime, m_bAllowAutoMovement, IsDead(); constants GAMEMOVEMENT_DUCK_TIME=1000, TIME_TO_DUCK=0.4, TIME_TO_UNDUCK=0.2",
          "outputs": "Latches IN_DUCK into mv->m_nOldButtons. Calls HandleDuckingSpeedCrop(). On duck press (not already ducked, not duck-jumping): m_flDucktime = 1000, m_bDucking = true. During duck transition: elapsed = 1000 - m_flDucktime, seconds = elapsed*0.001; if seconds > TIME_TO_DUCK OR bInDuck OR bInAir -> FinishDuck() (note: IN AIR THE DUCK COMPLETES IMMEDIATELY); else SetDuckedEyeOffset(SimpleSpline(seconds/TIME_TO_DUCK)). Duck-jump path: StartUnDuckJump() if not yet ducked, else CanUnDuckJump()/FinishUnDuckJump() and m_flDuckJumpTime = TIME_TO_UNDUCK*1000*(1-trace.fraction) + inverse constant. On release: if bInDuck && !bDuckJump -> m_flDucktime = 1000; else if partially ducked, INVERTS the timer so the unduck takes a proportional slice of TIME_TO_UNDUCK. Then if CanUnduck(): finish immediately when seconds > TIME_TO_UNDUCK or (bInAir && !bDuckJump) -> FinishUnDuck(); else SetDuckedEyeOffset(SimpleSpline(1 - seconds/TIME_TO_UNDUCK)) and m_bDucking = true. If CanUnduck() fails, forces the fully-ducked state (SetDuckedEyeOffset(1), m_flDucktime=1000, m_bDucked=true, m_bDucking=false, AddFlag(FL_DUCKING)).",
          "purity": "mutates-member-state",
          "callable_directly": "no - it is the duck state machine over eight player members plus traces",
          "notes": "Runs before ALL velocity work, so the hull for this tick's traces is decided here. Two TAS-critical facts confirmed by the wiki's Duck Jump Fix page: (1) in air, both duck and unduck complete IMMEDIATELY (one tick), producing a single-frame origin jump - the page calls it 'approximately 32 units in one frame' and that is precisely the behavior the fix removes; (2) m_bDucking stays true through BOTH transitions, matching the user's fuzz finding that a tapped-and-released ground duck keeps ducking=true for TIME_TO_UNDUCK and flips CheckJumpButton to the SET path."
        },
        {
          "name": "CGameMovement::HandleDuckingSpeedCrop",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::HandleDuckingSpeedCrop( void )",
          "inputs": "m_iSpeedCropped (SPEED_CROPPED_DUCK bit), player->GetFlags()&FL_DUCKING, player->GetGroundEntity()",
          "outputs": "If not already cropped AND FL_DUCKING AND on ground: mv->m_flForwardMove/m_flSideMove/m_flUpMove *= frac; sets the SPEED_CROPPED_DUCK bit so it cannot double-apply",
          "purity": "mutates-member-state",
          "callable_directly": "yes - three multiplies plus a latch bit",
          "notes": "frac is 0.33333333f in the base SDK; CS:S uses ~0.34 (the user's repo pins duck_speed_frac = 0.34). Note the ON-GROUND condition: a ducked airborne player is NOT speed-cropped, which is the whole basis of duck-jumping into gaps. m_iSpeedCropped is reset once per usercmd in ProcessMovement."
        },
        {
          "name": "CGameMovement::CanUnduck",
          "klass": "CGameMovement",
          "signature": "bool CGameMovement::CanUnduck( void )",
          "inputs": "mv->GetAbsOrigin(), player->GetGroundEntity(), VEC_DUCK_HULL_MIN/VEC_HULL_MIN/VEC_DUCK_HULL_MAX/VEC_HULL_MAX, player->m_Local.m_bDucked, PlayerSolidMask()",
          "outputs": "Computes the candidate standing origin: on ground, newOrigin += (VEC_DUCK_HULL_MIN - VEC_HULL_MIN) (zero for the default hulls, since both mins.z are 0); in air, newOrigin -= (hullSizeNormal - hullSizeCrouch), i.e. the origin drops so the head stays put. Temporarily clears m_bDucked so the trace uses the STANDING hull, runs TracePlayerBBox(origin -> newOrigin), restores m_bDucked. Returns false if trace.startsolid or fraction != 1.0.",
          "purity": "reads-member-state",
          "callable_directly": "unknown - it temporarily mutates m_bDucked (restored) and needs a trace oracle",
          "notes": "The m_bDucked save/restore is the trick that makes the trace use the standing hull. This is the 'can I stand up here' test - the reason you stay crouched under a vent."
        },
        {
          "name": "CGameMovement::FinishDuck",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::FinishDuck( void )",
          "inputs": "player->GetFlags()&FL_DUCKING (early-out if already set), player->GetGroundEntity(), the four hull constants",
          "outputs": "AddFlag(FL_DUCKING); m_bDucked = true; m_bDucking = false; SetViewOffset(GetPlayerViewOffset(true)); on ground shifts origin by (VEC_DUCK_HULL_MIN - VEC_HULL_MIN); IN AIR shifts origin UP by (hullSizeNormal - hullSizeCrouch) so the head stays in place while the feet rise; FixPlayerCrouchStuck(true); CategorizePosition().",
          "purity": "mutates-member-state",
          "callable_directly": "no - mutates flags, origin, view, and re-runs ground detection",
          "notes": "The in-air branch is the origin jump. SDK-theoretical delta is 72-36=36 for HL2 hulls; for CS:S hulls it is smaller. The user's repo ENGINE-MEASURED the CS:S air duck shift as exactly +8.500 (and unduck exactly -8.500), which does NOT equal the naive hull delta - trust the measurement over the formula. Note it calls CategorizePosition at the end, so ducking can gain or lose ground within a tick."
        },
        {
          "name": "CGameMovement::FinishUnDuck",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::FinishUnDuck( void )",
          "inputs": "mv->GetAbsOrigin(), player->GetGroundEntity(), hull constants",
          "outputs": "Computes newOrigin (on ground: += (VEC_DUCK_HULL_MIN - VEC_HULL_MIN); in air: -= (hullSizeNormal - hullSizeCrouch), i.e. origin DROPS). Then m_bDucked = false; RemoveFlag(FL_DUCKING); m_bDucking = false; m_bInDuckJump = false; SetViewOffset(GetPlayerViewOffset(false)); m_flDucktime = 0; mv->SetAbsOrigin(newOrigin); CategorizePosition().",
          "purity": "mutates-member-state",
          "callable_directly": "no",
          "notes": "Note it does NOT call CanUnduck itself - the caller (Duck) must gate it. The user's repo found a TRANSIENT HULL STATE here: after an air unduck the origin drops 8.5 but the effective world-space bounds do not immediately re-base, leaving the top around 62.5 until grounding re-bases it. That is a measured engine behavior with no SDK counterpart and no wiki documentation."
        },
        {
          "name": "CGameMovement::CanUnDuckJump",
          "klass": "CGameMovement",
          "signature": "bool CGameMovement::CanUnDuckJump( trace_t &trace )",
          "inputs": "mv->GetAbsOrigin(), the hardcoded 36.0f drop, player->m_Local.m_bDucked, PlayerSolidMask()",
          "outputs": "Traces the DUCKED hull from origin down to origin.z - 36.0f. If fraction < 1.0, computes vecEnd.z = origin.z + (-36.0f * fraction), then runs a zero-length trace there with m_bDucked temporarily false (STANDING hull). Returns true if that standing trace is not startsolid, and leaves the DOWN trace in `trace`.",
          "purity": "reads-member-state",
          "callable_directly": "unknown - needs a trace oracle; temporarily mutates m_bDucked",
          "notes": "The 36.0f is a hardcoded magic number tied to the HL2 hull; the wiki's Player Entity page explicitly flags line 4170 'vecEnd.z -= 36.0f; // This will have to change if bounding hull change!' as something you must edit when resizing the player. For CS:S hulls this constant is a latent mismatch."
        },
        {
          "name": "CGameMovement::StartUnDuckJump",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::StartUnDuckJump( void )",
          "inputs": "hull constants, mv->GetAbsOrigin()",
          "outputs": "AddFlag(FL_DUCKING); m_bDucked = true; m_bDucking = false; SetViewOffset(GetPlayerViewOffset(true)); origin += (hullSizeNormal - hullSizeCrouch) (moves UP); FixPlayerCrouchStuck(true); CategorizePosition().",
          "purity": "mutates-member-state",
          "callable_directly": "no",
          "notes": "Called from the duck-jump branch of Duck when jumping while not yet ducked - it snaps the bbox small immediately so you clear the ledge."
        },
        {
          "name": "CGameMovement::FinishUnDuckJump",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::FinishUnDuckJump( trace_t &trace )",
          "inputs": "trace.fraction from CanUnDuckJump, mv->GetAbsOrigin(), hull constants",
          "outputs": "viewDelta = (hullSizeNormal - hullSizeCrouch); flDeltaZ = viewDelta.z; viewDelta.z *= trace.fraction; flDeltaZ -= viewDelta.z. Clears FL_DUCKING, m_bDucked, m_bDucking, m_bInDuckJump; zeroes m_flDucktime, m_flDuckJumpTime, m_flJumpTime. View offset = standing offset with z reduced by flDeltaZ (so the eye lerps). origin -= viewDelta (a PARTIAL drop scaled by trace.fraction). CategorizePosition().",
          "purity": "mutates-member-state",
          "callable_directly": "no",
          "notes": "The fraction scaling is what lets you land partway out of a duck-jump without clipping into the floor - the origin only drops as far as the trace said was free."
        },
        {
          "name": "CGameMovement::SetDuckedEyeOffset",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::SetDuckedEyeOffset( float duckFraction )",
          "inputs": "duckFraction, GetPlayerMins(true/false), GetPlayerViewOffset(true/false), player->GetViewOffset()",
          "outputs": "fMore = duckHullMin.z - standHullMin.z; view.z = (duckViewOffset.z - fMore)*duckFraction + standViewOffset.z*(1-duckFraction); SetViewOffset(view)",
          "purity": "mutates-member-state",
          "callable_directly": "yes - pure lerp, only writes the view offset",
          "notes": "VIEW ONLY. No effect on collision or velocity. Skippable in a physics solver. SimpleSpline (3t^2 - 2t^3) is applied by the CALLER, not here."
        },
        {
          "name": "CGameMovement::FixPlayerCrouchStuck",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::FixPlayerCrouchStuck( bool upward )",
          "inputs": "mv->GetAbsOrigin(), direction flag, current hull, COLLISION_GROUP_PLAYER_MOVEMENT",
          "outputs": "Returns immediately if TestPlayerPosition finds the origin free. Otherwise probes up to 36 unit-steps in the given direction (+1 per iteration for upward, -1 for downward), and on finding a free position writes mv->SetAbsOrigin(test). If all 36 fail, restores the original origin.",
          "purity": "mutates-member-state",
          "callable_directly": "unknown - simple loop, but needs a trace oracle",
          "notes": "Called from FinishDuck and StartUnDuckJump. 36 iterations of 1 unit is exactly the HL2 crouch hull height - another hardcoded-hull dependency."
        },
        {
          "name": "CGameMovement::GetPlayerViewOffset",
          "klass": "CGameMovement",
          "signature": "const Vector& CGameMovement::GetPlayerViewOffset( bool ducked ) const",
          "inputs": "ducked flag; g_DefaultViewVectors (VEC_VIEW, VEC_DUCK_VIEW)",
          "outputs": "Returns (0,0,64) standing or (0,0,28) ducked for the HL2 defaults",
          "purity": "pure",
          "callable_directly": "yes",
          "notes": "Values documented on the wiki's Player Entity page: VEC_VIEW (0,0,64), VEC_DUCK_VIEW (0,0,28), VEC_DEAD_VIEWHEIGHT (0,0,14). Dimensions page corroborates: eye 64 standing, 28 crouched."
        },
        {
          "name": "CGameMovement::GetPlayerMins / GetPlayerMaxs",
          "klass": "CGameMovement",
          "signature": "const Vector& GetPlayerMins( bool ducked ) const; const Vector& GetPlayerMaxs( bool ducked ) const; and the no-arg overloads that read player->m_Local.m_bDucked",
          "inputs": "ducked flag, or player->m_Local.m_bDucked for the no-arg form; g_DefaultViewVectors",
          "outputs": "HL2 defaults: standing (-16,-16,0)..(16,16,72); ducked (-16,-16,0)..(16,16,36). CS:S overrides these (CCSGameMovement::GetPlayerMins/Maxs) to a 62-tall standing hull and a 45-tall ducked hull.",
          "purity": "reads-member-state",
          "callable_directly": "yes - constant lookup, but the no-arg form silently reads m_bDucked",
          "notes": "The no-arg overloads reading m_bDucked are the hidden coupling that CanUnduck/CanUnDuckJump exploit by temporarily flipping the flag. Wiki Player Entity page gives the HL2 constants verbatim; Dimensions (HL2 and CS:S) gives CS:S: 62 standing / 45 crouched collision height, 32 wide/deep, 18 max step. The user's repo measured the CS:S ducked collision height as 54, NOT 45 - the repo comment explicitly says this was corrected against the VDC dimensions after challenge. Trust the measurement."
        },
        {
          "name": "CGameMovement::LadderMove",
          "klass": "CGameMovement",
          "signature": "bool CGameMovement::LadderMove( void )",
          "inputs": "player->GetMoveType(), GameHasLadders(), m_vecForward, LadderDistance(), LadderMask(), OnLadder(trace), mv->m_flForwardMove/m_flSideMove, mv->m_vecViewAngles, ClimbSpeed(), LadderLateralMultiplier(), MAX_CLIMB_SPEED, player->GetGroundEntity()",
          "outputs": "Returns false if noclip or no ladders. Otherwise traces forward LadderDistance() and tests OnLadder(); on success sets player->SetMoveType(MOVETYPE_LADDER), SetMoveCollide(MOVECOLLIDE_DEFAULT), stores the ladder normal, and computes mv->m_vecVelocity directly from the view basis projected onto the ladder plane (no gravity, no friction, no acceleration). Dismount when moving away from the ladder while on ground. Returns true while mounted.",
          "purity": "mutates-member-state",
          "callable_directly": "no - changes movetype and needs traces plus the func_ladder entity",
          "notes": "Ladders BYPASS the entire accelerate/friction/gravity pipeline - velocity is set outright from input. LadderDistance() is 2.0 on ground / 24.0 in air in the base SDK; LadderMask() is MASK_PLAYERSOLID minus CONTENTS_PLAYERCLIP; ClimbSpeed() is MAX_CLIMB_SPEED (200); LadderLateralMultiplier is 1.0 (2.0 when ducked). The wiki has Func ladder and Ladders pages but they are mapper-facing and document none of this."
        },
        {
          "name": "CGameMovement::OnLadder",
          "klass": "CGameMovement",
          "signature": "bool CGameMovement::OnLadder( trace_t &trace )",
          "inputs": "trace.contents, trace.m_pEnt",
          "outputs": "Returns true if trace.contents & CONTENTS_LADDER, or the hit entity is a func_ladder",
          "purity": "pure",
          "callable_directly": "yes",
          "notes": "CONTENTS_LADDER is a BSP contents flag - see BSP flags (Source). This is the one place the movement code reads a contents bit for a non-solid purpose."
        },
        {
          "name": "CGameMovement::FullLadderMove",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::FullLadderMove( void )",
          "inputs": "mv, player, water level",
          "outputs": "CheckWater(); MoveHelper()->ResetTouchList() bookkeeping; then TryPlayerMove() with the ladder velocity; no gravity, no friction",
          "purity": "mutates-member-state",
          "callable_directly": "no",
          "notes": "The MOVETYPE_LADDER branch of PlayerMove's switch."
        },
        {
          "name": "CGameMovement::FullWalkMove",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::FullWalkMove( void )",
          "inputs": "player->GetWaterLevel(), m_flWaterJumpTime, mv->m_nButtons (IN_JUMP), player->GetGroundEntity(), m_nOldWaterLevel",
          "outputs": "EXACT ORDER for the dry-land path: (1) if !CheckWater() then StartGravity() - HALF gravity applied up front; (2) if m_flWaterJumpTime, do WaterJump/TryPlayerMove/CheckWater and RETURN; (3) if waterlevel >= WL_Waist take the water branch (CheckWaterJump, CheckJumpButton or clear IN_JUMP from m_nOldButtons, WaterMove, CategorizePosition, zero vz if grounded); (4) OTHERWISE: if IN_JUMP -> CheckJumpButton(), else mv->m_nOldButtons &= ~IN_JUMP; (5) if on ground: mv->m_vecVelocity.z = 0 then Friction(); (6) CheckVelocity(); (7) if on ground WalkMove() else AirMove(); (8) CategorizePosition(); (9) CheckVelocity(); (10) if !CheckWater() then FinishGravity() - the other HALF; (11) if on ground, mv->m_vecVelocity.z = 0; (12) CheckFalling(). Finally plays swim sounds on water-level transitions.",
          "purity": "mutates-member-state",
          "callable_directly": "no",
          "notes": "The half-gravity split (StartGravity before the move, FinishGravity after) is the single most important integration detail - it is a leapfrog/velocity-Verlet scheme, not Euler. CheckJumpButton runs BEFORE friction, which is why a perfect bhop takes no ground friction. And note CheckJumpButton itself ALSO calls FinishGravity internally, giving the jump tick an extra half-gravity on top of this pair - the user's repo models this as the jump_finishgravity toggle and has it ON."
        },
        {
          "name": "CGameMovement::CheckWater",
          "klass": "CGameMovement",
          "signature": "bool CGameMovement::CheckWater( void )",
          "inputs": "mv->GetAbsOrigin(), GetPlayerMins()/GetPlayerMaxs(), player->GetViewOffset(), MASK_WATER, GetPointContentsCached",
          "outputs": "Sets player water level to WL_NotInWater and water type to CONTENTS_EMPTY, then probes three points: feet (mins.z + 1), hull midpoint, and eye (origin.z + viewoffset.z). Sets WL_Feet / WL_Waist / WL_Eyes accordingly and SetWaterType(cont). Returns true if water level > WL_Feet.",
          "purity": "mutates-member-state",
          "callable_directly": "unknown - needs a point-contents oracle",
          "notes": "Called from PlayerMove indirectly (via CategorizePosition), from FullWalkMove twice, and from CategorizePosition. Note the return value is 'waist or deeper', not 'touching water at all' - that asymmetry is why FullWalkMove still runs gravity when only your feet are wet."
        },
        {
          "name": "CGameMovement::GetPointContentsCached",
          "klass": "CGameMovement",
          "signature": "int CGameMovement::GetPointContentsCached( const Vector &point, int slot )",
          "inputs": "point, slot index (0/1/2 for feet/waist/eyes), m_CachedGetPointContents[slot], sv_optimizedmovement",
          "outputs": "Returns cached contents if the point matches the cached origin for that slot, else calls enginetrace->GetPointContents(point) and caches it",
          "purity": "mutates-member-state",
          "callable_directly": "unknown - trivial wrapper, but the cache is per-tick state",
          "notes": "ResetGetPointContentsCache() at the top of ProcessMovement invalidates all three slots. The cache means CheckWater called three times in one tick only pays for one engine query per slot."
        },
        {
          "name": "CGameMovement::StartGravity",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::StartGravity( void )",
          "inputs": "player->GetGravity() (per-entity multiplier, 0 means 1.0), GetCurrentGravity() (= sv_gravity), gpGlobals->frametime, player->GetBaseVelocity().z",
          "outputs": "mv->m_vecVelocity.z -= ent_gravity * GetCurrentGravity() * 0.5 * frametime; then mv->m_vecVelocity.z += GetBaseVelocity().z * frametime; then zeroes basevel.z via SetBaseVelocity; then CheckVelocity()",
          "purity": "mutates-member-state",
          "callable_directly": "yes - given gravity, entity gravity, dt and basevel.z",
          "notes": "The 0.5 is intentional (the SDK comment says 'yes, this 0.5 looks wrong, but it's not'). ent_gravity is where trigger_gravity lands: CTriggerGravity::GravityTouch calls pOther->SetGravity(scale), and it PERSISTS until another trigger_gravity changes it (per the wiki's Trigger gravity page). Note this function also consumes and CLEARS the z component of base velocity - a vertical trigger_push is applied here, once, as a dt-scaled velocity add."
        },
        {
          "name": "CGameMovement::FinishGravity",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::FinishGravity( void )",
          "inputs": "player->m_flWaterJumpTime (early-out), player->GetGravity(), GetCurrentGravity(), gpGlobals->frametime",
          "outputs": "mv->m_vecVelocity.z -= ent_gravity * GetCurrentGravity() * frametime * 0.5; CheckVelocity()",
          "purity": "mutates-member-state",
          "callable_directly": "yes",
          "notes": "The second half of the leapfrog. Unlike StartGravity it does NOT touch base velocity. Early-returns during a water jump."
        },
        {
          "name": "CGameMovement::AddGravity",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::AddGravity( void )",
          "inputs": "same as StartGravity",
          "outputs": "mv->m_vecVelocity.z -= ent_gravity * GetCurrentGravity() * frametime (FULL step, not half); adds and clears basevel.z; CheckVelocity()",
          "purity": "mutates-member-state",
          "callable_directly": "yes",
          "notes": "NOT used by the MOVETYPE_WALK path - FullTossMove uses it. The SDK comment literally says 'Add gravity incorrectly'. This is the one CGameMovement function the VDC wiki names: the Gravity page lists 'void CGameMovement::AddGravity()' under QPhysics, described as adding sv_gravity to falling players. That single mention is the extent of VDC's CGameMovement coverage."
        },
        {
          "name": "CGameMovement::CheckWaterJump",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::CheckWaterJump( void )",
          "inputs": "m_vecForward, mv->GetAbsOrigin(), mv->m_vecVelocity, player->GetWaterLevel(), WATERJUMP_HEIGHT (8), CONTENTS_SOLID probes",
          "outputs": "If facing a ledge just above the waterline with forward speed, sets player->m_flWaterJumpTime = 2000 and mv->m_vecVelocity.z = 225, and sets FL_WATERJUMP",
          "purity": "mutates-member-state",
          "callable_directly": "unknown - needs point-contents probes",
          "notes": "Only called when water level is exactly WL_Waist."
        },
        {
          "name": "CGameMovement::WaterJump",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::WaterJump( void )",
          "inputs": "player->m_flWaterJumpTime, m_vecWaterJumpVel, gpGlobals->frametime",
          "outputs": "Decrements m_flWaterJumpTime by frametime; forces mv->m_vecVelocity.x/y to the stored waterjump velocity and leaves z alone",
          "purity": "mutates-member-state",
          "callable_directly": "yes",
          "notes": "While m_flWaterJumpTime is nonzero, Friction, AirAccelerate and FinishGravity all early-out - the player is on rails."
        },
        {
          "name": "CGameMovement::WaterMove",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::WaterMove( void )",
          "inputs": "mv->m_vecViewAngles (local AngleVectors, NOT z-zeroed), mv->m_flForwardMove/m_flSideMove/m_flUpMove, mv->m_nButtons (IN_JUMP), mv->m_flClientMaxSpeed, mv->m_flMaxSpeed, sv_friction, sv_accelerate, player->m_surfaceFriction, gpGlobals->frametime, player->GetBaseVelocity(), player->m_Local.m_bAllowAutoMovement, m_flStepSize",
          "outputs": "Builds a full 3D wishvel (z included, unlike WalkMove/AirMove). IN_JUMP adds m_flClientMaxSpeed to wishvel.z; no input at all subtracts 60 (sink); otherwise adds upmove plus an exaggerated forward-look term clamped to [0, clientMaxSpeed]. Clamps to m_flMaxSpeed, then multiplies wishspeed by 0.8. Applies water friction: newspeed = speed - frametime*speed*sv_friction*m_surfaceFriction (a proportional drag, NOT the ground stopspeed model). Then water acceleration with accelspeed = sv_accelerate * wishspeed * frametime * m_surfaceFriction. Adds base velocity, attempts a step-up move, falls back to TryPlayerMove or StepMove, then subtracts base velocity back out.",
          "purity": "mutates-member-state",
          "callable_directly": "no",
          "notes": "Note water uses sv_friction/sv_accelerate, not sv_waterfriction/sv_wateraccelerate, in this code path - those cvars exist (defaults 1 and 10 per the Console Command List) but the SDK 2013 WaterMove reads sv_friction and sv_accelerate. Also note the 0.8 wishspeed scale and the fully-3D wish vector."
        },
        {
          "name": "CGameMovement::CheckJumpButton",
          "klass": "CGameMovement",
          "signature": "bool CGameMovement::CheckJumpButton( void )",
          "inputs": "player->pl.deadflag, m_flWaterJumpTime, player->GetWaterLevel(), GetWaterType(), player->GetGroundEntity(), mv->m_nOldButtons (IN_JUMP), player->m_Local.m_bDucking, GetFlags()&FL_DUCKING, m_flDuckJumpTime, m_pSurfaceData->game.jumpFactor, GetCurrentGravity(), GAMEMOVEMENT_JUMP_HEIGHT (21.0), g_bMovementOptimizations, m_flStamina (CS)",
          "outputs": "Returns false (and latches IN_JUMP into m_nOldButtons) if dead, airborne, or pogo-sticking. Returns false WITHOUT latching if m_bDucking && FL_DUCKING (cannot jump mid-unduck transition) or m_flDuckJumpTime > 0. Water level >= 2 sets vz to 100 (water) or 80 (slime) and returns false. On success: SetGroundEntity(NULL); flMul = sqrt(2 * GetCurrentGravity() * GAMEMOVEMENT_JUMP_HEIGHT), or a BAKED CONSTANT when g_bMovementOptimizations is on (160.0f for HL2's gravity 600, 268.3281572999747f for gravity 800). If ducking or FL_DUCKING: mv->m_vecVelocity.z = flGroundFactor * flMul (SET); otherwise mv->m_vecVelocity.z += flGroundFactor * flMul (ADD). Then FinishGravity(); mv->m_outJumpVel.z += (vz - startz); mv->m_outStepHeight += 0.15f; mv->m_nOldButtons |= IN_JUMP. Returns true.",
          "purity": "mutates-member-state",
          "callable_directly": "no - mutates ground entity, velocity, and the old-buttons latch",
          "notes": "The SET-vs-ADD split on the duck flag is the single highest-leverage branch in the whole file for a TAS. The user's fuzz work confirmed it: a tapped-and-released ground duck keeps m_bDucking true for TIME_TO_UNDUCK, flipping the jump to the SET path (measured 289.99 vs 283.99 for the ADD path). The g_bMovementOptimizations baked constant is decisive: it does NOT track sv_gravity. The user's binary decode of the shipped CS:S x64 server.dll found the constant baked as a DOUBLE, sqrt(2*800*57) = 301.99337741082996, applied in double with a single rounding back to float - so CS:S uses jump height 57, not the SDK's 21, and the value is frozen regardless of sv_gravity. FinishGravity being called HERE is the extra half-gravity on the jump tick."
        },
        {
          "name": "CCSGameMovement::CheckJumpButton",
          "klass": "CCSGameMovement",
          "signature": "bool CCSGameMovement::CheckJumpButton( void )",
          "inputs": "everything the base takes, plus m_pCSPlayer->m_flStamina, STAMINA_MAX (100), STAMINA_COST_JUMP (25), STAMINA_RECOVER_RATE (19), sv_enablebunnyhopping / sv_autobunnyhopping",
          "outputs": "Calls PreventBunnyJumping() first (unless sv_enablebunnyhopping). Applies the base impulse, then scales: flRatio = (STAMINA_MAX - (m_flStamina/1000)*STAMINA_RECOVER_RATE)/STAMINA_MAX, i.e. (1 - m_flStamina*0.00019f), and multiplies the WHOLE post-impulse vz by it. Then sets m_flStamina to the jump cost.",
          "purity": "mutates-member-state",
          "callable_directly": "no",
          "notes": "The user's disassembly (server.dll x64 @2eddfc) pins this exactly: the whole post-impulse vz scales by (1 - stamina*0.00019f) in float, and m_flStamina is then stored as raw bits 0x44a47943 == float(25000.0/19.0) exactly. That is STAMINA_COST_JUMP*1000/STAMINA_RECOVER_RATE = 25*1000/19 - the SDK formula and the binary agree perfectly. Also measured: landings arm NOTHING on this server (a -775 u/s landing walked at scale 1.00000), so STAMINA_COST_FALL is not in play."
        },
        {
          "name": "CCSGameMovement::PreventBunnyJumping",
          "klass": "CCSGameMovement",
          "signature": "void CCSGameMovement::PreventBunnyJumping( void )",
          "inputs": "mv->m_flMaxSpeed, mv->m_vecVelocity, BUNNYJUMP_MAX_SPEED_FACTOR",
          "outputs": "maxscaledspeed = BUNNYJUMP_MAX_SPEED_FACTOR * mv->m_flMaxSpeed; if current speed exceeds it, scales the entire velocity vector by maxscaledspeed/spd",
          "purity": "mutates-member-state",
          "callable_directly": "yes - pure clamp on velocity given maxspeed",
          "notes": "The pre-jump speed cap. Factor is 1.1 in CS:S (1.2 in CS:GO). sv_enablebunnyhopping 1 disables it entirely - which is exactly the surf/bhop server setup, and matches the user's MoveParams::enablebunnyhopping = true. The wiki's Bunnyhop page describes the HL2 variant of this anti-bhop measure and its ABH exploit, and notes sv_autobunnyhopping was added to CS:S on the TF2 branch in Feb 2025."
        },
        {
          "name": "CGameMovement::Friction",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::Friction( void )",
          "inputs": "player->m_flWaterJumpTime (early-out), mv->m_vecVelocity, player->GetGroundEntity(), sv_friction, player->m_surfaceFriction, sv_stopspeed, gpGlobals->frametime",
          "outputs": "speed = |velocity|; returns if speed < 0.1f. drop = 0. If on ground: friction = sv_friction * m_surfaceFriction; control = (speed < sv_stopspeed) ? sv_stopspeed : speed; drop += control * friction * frametime. newspeed = max(0, speed - drop); if newspeed != speed, scales velocity by newspeed/speed. Also updates mv->m_outWishVel.",
          "purity": "mutates-member-state",
          "callable_directly": "yes - pure given velocity, ground flag, the three scalars and dt",
          "notes": "The stopspeed floor is what makes low speeds decay linearly instead of exponentially. sv_friction default 4 (wiki confirms 4 for CS and CS:S, 5.2 for CS:GO); sv_stopspeed default 100 per the Console Command List - BUT the user's engine ground-truth fit CONFIRMED sv_stopspeed = 75 on their target server, not 100. m_surfaceFriction multiplies in here, so the 0.25 airborne value from CategorizePosition would quarter friction - except this branch only runs when grounded, so in practice the 0.25 only ever affects Accelerate/AirAccelerate, not Friction."
        },
        {
          "name": "CGameMovement::CheckVelocity",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::CheckVelocity( void )",
          "inputs": "mv->m_vecVelocity, mv->GetAbsOrigin(), sv_maxvelocity",
          "outputs": "Per COMPONENT (x, y, z independently): NaN check on velocity (zeroed) and origin (zeroed via SetAbsOrigin); then clamps velocity[i] to [-sv_maxvelocity, +sv_maxvelocity]",
          "purity": "mutates-member-state",
          "callable_directly": "yes - three independent scalar clamps",
          "notes": "PER-AXIS, not magnitude - the wiki's Console Command List describes sv_maxvelocity (default 3500) as 'Maximum speed any ballistically moving object is allowed to attain PER AXIS', which is the correct reading. So the true speed cap is 3500*sqrt(3) diagonally. Called from StartGravity, FinishGravity, AddGravity, and twice explicitly in FullWalkMove."
        },
        {
          "name": "CGameMovement::WalkMove",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::WalkMove( void )",
          "inputs": "mv->m_vecViewAngles (LOCAL AngleVectors), player->GetGroundEntity() (saved as oldground), mv->m_flForwardMove/m_flSideMove, mv->m_flMaxSpeed, sv_accelerate, gpGlobals->frametime, player->GetBaseVelocity(), player->GetWaterLevel(), m_flWaterJumpTime, g_bMovementOptimizations",
          "outputs": "Computes local forward/right, ZEROES their z and renormalizes (under g_bMovementOptimizations, only if z != 0). wishvel.xy = forward.xy*fmove + right.xy*smove, wishvel.z = 0. wishdir = normalized, wishspeed = length. Clamps wishvel/wishspeed to mv->m_flMaxSpeed. Sets velocity.z = 0, calls Accelerate(wishdir, wishspeed, sv_accelerate), sets velocity.z = 0 again. ADDS base velocity. If total speed < 1.0, zeroes velocity, subtracts basevel back out, returns. Traces horizontally to dest = origin + velocity.xy*frametime (dest.z unchanged). mv->m_outWishVel += wishdir*wishspeed. If fraction == 1: SetAbsOrigin(endpos), subtract basevel, StayOnGround(), return. If oldground was NULL and not in water: subtract basevel and return (NO stair-stepping when you just landed). If m_flWaterJumpTime: subtract basevel and return. Otherwise StepMove(dest, pm), subtract basevel, StayOnGround().",
          "purity": "mutates-member-state",
          "callable_directly": "no",
          "notes": "Three details a solver must get right: (1) the z-zero-and-renormalize of forward/right happens on LOCAL copies, so m_vecForward is untouched; (2) base velocity is added BEFORE the trace and subtracted after, so conveyor motion moves you but is not accumulated; (3) the oldground==NULL early return is why you cannot step up onto a ledge on the tick you land."
        },
        {
          "name": "CCSGameMovement::WalkMove",
          "klass": "CCSGameMovement",
          "signature": "void CCSGameMovement::WalkMove( void )",
          "inputs": "m_pCSPlayer->m_flStamina, gpGlobals->frametime, plus everything the base takes",
          "outputs": "Applies the stamina scale to velocity.xy after Friction and before Accelerate, then runs the base logic",
          "purity": "mutates-member-state",
          "callable_directly": "no",
          "notes": "The SDK-shaped form is a linear scale by (1 - stamina*0.00019). The user's binary decode (server.dll x64 WalkMove @2f0160) found the shipped build instead does vel.xy *= powf(1 - stamina*0.00019f, frametime*70.0f) - a POWER, with an exponent that scales with tickrate (1.05 at 66.67 tps). The same .rdata constant at 0056faf4 feeds both this and the jump site. An earlier affine fit (slope 0.00019833, intercept 0.000327) was just a chord across that power curve and has been retired. Trust the disassembly."
        },
        {
          "name": "CGameMovement::Accelerate",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::Accelerate( Vector& wishdir, float wishspeed, float accel )",
          "inputs": "CanAccelerate(), mv->m_vecVelocity, wishdir, wishspeed, accel (sv_accelerate), gpGlobals->frametime, player->m_surfaceFriction",
          "outputs": "Returns if !CanAccelerate(). currentspeed = velocity DOT wishdir; addspeed = wishspeed - currentspeed; returns if addspeed <= 0. accelspeed = accel * frametime * wishspeed * m_surfaceFriction; clamped to addspeed. velocity += accelspeed * wishdir (all three components).",
          "purity": "mutates-member-state",
          "callable_directly": "yes - pure given velocity, wishdir, wishspeed, accel, dt and surfaceFriction",
          "notes": "Note the ordering: accel * frametime * wishspeed, so acceleration scales with how fast you WANT to go, not with a fixed rate. m_surfaceFriction multiplies here - this is the second place the 0.25 airborne value bites (though on ground it is normally 1.0). The dot-product projection is what makes counter-strafing work."
        },
        {
          "name": "CGameMovement::CanAccelerate",
          "klass": "CGameMovement",
          "signature": "bool CGameMovement::CanAccelerate( void )",
          "inputs": "player->pl.deadflag, player->GetWaterJumpTime()",
          "outputs": "Returns false if dead or water-jumping, true otherwise",
          "purity": "reads-member-state",
          "callable_directly": "yes",
          "notes": "CCSGameMovement overrides this so observers can move. Always true for a live moving player."
        },
        {
          "name": "CGameMovement::AirMove",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::AirMove( void )",
          "inputs": "mv->m_vecViewAngles (LOCAL AngleVectors), mv->m_flForwardMove/m_flSideMove, mv->m_flMaxSpeed, sv_airaccelerate, player->GetBaseVelocity()",
          "outputs": "Local forward/right with z zeroed and UNCONDITIONALLY renormalized (no g_bMovementOptimizations branch here, unlike WalkMove). wishvel.xy from fmove/smove, wishvel.z = 0. wishdir normalized, wishspeed = length, clamped to mv->m_flMaxSpeed. AirAccelerate(wishdir, wishspeed, sv_airaccelerate). Adds base velocity, TryPlayerMove(), subtracts base velocity back out.",
          "purity": "mutates-member-state",
          "callable_directly": "no",
          "notes": "Note the asymmetry with WalkMove: AirMove always normalizes, WalkMove only when the optimization flag is off. Also AirMove does NOT zero velocity.z around the accelerate call, because gravity lives in z."
        },
        {
          "name": "CGameMovement::AirAccelerate",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::AirAccelerate( Vector& wishdir, float wishspeed, float accel )",
          "inputs": "player->pl.deadflag, player->m_flWaterJumpTime, GetAirSpeedCap(), mv->m_vecVelocity, wishdir, wishspeed, accel (sv_airaccelerate), gpGlobals->frametime, player->m_surfaceFriction",
          "outputs": "Returns if dead or water-jumping. wishspd = min(wishspeed, GetAirSpeedCap()). currentspeed = velocity DOT wishdir. addspeed = wishspd - currentspeed; returns if <= 0. accelspeed = accel * wishspeed * frametime * m_surfaceFriction - NOTE it uses the UNCAPPED wishspeed here, while addspeed used the CAPPED wishspd. Clamps accelspeed to addspeed. velocity += accelspeed*wishdir; mv->m_outWishVel += accelspeed*wishdir.",
          "purity": "mutates-member-state",
          "callable_directly": "yes - pure given velocity, wishdir, wishspeed, accel, air cap, dt and surfaceFriction",
          "notes": "THE air-strafing function. The capped/uncapped wishspeed asymmetry is deliberate and is the entire mechanism of air acceleration: the addspeed budget is capped at 30, but the rate of filling it is not. This is why high sv_airaccelerate lets you fill the 30-unit budget every tick. m_surfaceFriction multiplying accelspeed is where the 0.25 rising-airborne value from CategorizePosition does real damage - the user's engine sample pinned it exactly: a ducked rising tick applied 47.8125 u/s = 150 * 85 * 0.015 * 0.25. sv_airaccelerate default is 10 per the Console Command List; surf servers run 150. VDC does not document this function; the Bunnyhop page only says servers 'change certain cvars ... such as sv_airaccelerate'."
        },
        {
          "name": "CGameMovement::GetAirSpeedCap",
          "klass": "CGameMovement",
          "signature": "virtual float CGameMovement::GetAirSpeedCap( void ) const",
          "inputs": "none in the base class",
          "outputs": "Returns 30.0f",
          "purity": "pure",
          "callable_directly": "yes - a constant",
          "notes": "The famous '30 cap'. CS:S keeps 30 (the user's MoveParams::air_speed_cap = 30). This is the only thing standing between sv_airaccelerate and unbounded acceleration."
        },
        {
          "name": "CGameMovement::TryPlayerMove",
          "klass": "CGameMovement",
          "signature": "int CGameMovement::TryPlayerMove( Vector *pFirstDest = NULL, trace_t *pFirstTrace = NULL )",
          "inputs": "mv->m_vecVelocity, mv->GetAbsOrigin(), gpGlobals->frametime, player->GetMoveType(), player->GetGroundEntity(), player->m_surfaceFriction, sv_bounce, MAX_CLIP_PLANES (5), PlayerSolidMask(), g_bMovementOptimizations, optional pFirstDest/pFirstTrace reuse",
          "outputs": "Returns a `blocked` bitmask (1 = floor hit, 2 = step/wall hit, 4 = trapped in solid). Up to 4 bumps. Per bump: break if speed == 0; end = origin + velocity*time_left; TracePlayerBBox (reusing pFirstTrace if end == *pFirstDest); allFraction += fraction. If allsolid -> zero velocity, return 4. If fraction > 0: an extra ZERO-LENGTH stuck check at pm.endpos - if that comes back startsolid or fraction != 1, zero velocity and break; otherwise SetAbsOrigin(endpos), refresh original_velocity, numplanes = 0. If fraction == 1, break. AddToTouched(pm). Set blocked bits from plane.normal.z (>0.7 floor, ==0 wall). time_left -= time_left*fraction. If numplanes >= 5, zero velocity and break. Push plane. THEN two clipping regimes: (a) if numplanes == 1 AND movetype == WALK AND ground entity is NULL, ClipVelocity with overbounce 1 for floors (normal.z > 0.7) and overbounce 1.0 + sv_bounce*(1 - m_surfaceFriction) for walls; (b) otherwise, for each plane i clip and test against all other planes j - if the clipped velocity moves into no other plane, accept; if all planes fail, and numplanes == 2, slide along the CREASE (dir = cross(planes[0],planes[1]) normalized, velocity = dir * (dir DOT velocity)); if numplanes != 2, zero velocity and break. Finally, if velocity DOT primal_velocity <= 0, zero velocity and break. After the loop: if allFraction == 0, zero velocity. Computes lateral stopping amount and calls PlayerRoughLandingEffects.",
          "purity": "mutates-member-state",
          "callable_directly": "no - it is the collision solver and mutates origin and velocity",
          "notes": "The heart of surf. Four bumps, five planes max, crease sliding on exactly two planes, and the anti-oscillation dot test against primal velocity. The single-plane airborne branch (regime a) is what gives surf ramps their behavior - overbounce 1 means pure projection with no bounce, and sv_bounce defaults to 0 so the wall term collapses to 1.0 as well. The zero-length stuck re-check after a fraction==1 trace is the terrain-precision guard and it can silently kill all velocity. The user's solver mirrors this with unclamped-enterfrac traces and explicit crease resolution."
        },
        {
          "name": "CGameMovement::ClipVelocity",
          "klass": "CGameMovement",
          "signature": "int CGameMovement::ClipVelocity( Vector& in, Vector& normal, Vector& out, float overbounce )",
          "inputs": "in velocity, plane normal, overbounce factor",
          "outputs": "Returns blocked bits (0x01 if normal.z > 0, 0x02 if normal.z == 0). backoff = (in DOT normal) * overbounce; out[i] = in[i] - normal[i]*backoff. THEN a correction pass: adjust = out DOT normal; if adjust < 0, out -= normal*adjust.",
          "purity": "pure",
          "callable_directly": "yes - genuinely pure, three inputs and one output",
          "notes": "The trailing adjust pass is essential and often omitted in reimplementations - it guarantees the result never points into the plane after float error. The user's SolverMove.cpp has this verbatim as EngineClipVelocity including the adjust < 0 branch."
        },
        {
          "name": "CGameMovement::TracePlayerBBox",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::TracePlayerBBox( const Vector& start, const Vector& end, unsigned int fMask, int collisionGroup, trace_t& pm )",
          "inputs": "start, end, fMask, collisionGroup, GetPlayerMins()/GetPlayerMaxs() (implicitly from player->m_Local.m_bDucked), mv->m_nPlayerHandle",
          "outputs": "Ray_t ray; ray.Init(start, end, GetPlayerMins(), GetPlayerMaxs()); enginetrace->TraceRay(ray, fMask, &CTraceFilterSimple(mv->m_nPlayerHandle.Get(), collisionGroup), &pm)",
          "purity": "reads-member-state",
          "callable_directly": "unknown - trivially reimplementable, but the hull comes from m_bDucked implicitly",
          "notes": "EVERY collision query in the movement code funnels through here (plus TryTouchGround for explicit hulls). The implicit hull read is why CanUnduck and CanUnDuckJump have to flip m_bDucked around their calls. VDC has no IEngineTrace or CGameTrace page; the trace_t page is a two-line stub that only says it is a typedef of CGameTrace, and UTIL_TraceLine covers only line traces for hitscan, not hull sweeps."
        },
        {
          "name": "CGameMovement::PlayerSolidMask",
          "klass": "CGameMovement",
          "signature": "virtual unsigned int CGameMovement::PlayerSolidMask( bool brushOnly = false )",
          "inputs": "brushOnly flag",
          "outputs": "Returns MASK_PLAYERSOLID_BRUSHONLY if brushOnly, else MASK_PLAYERSOLID",
          "purity": "pure",
          "callable_directly": "yes",
          "notes": "MASK_PLAYERSOLID = CONTENTS_SOLID|MOVEABLE|PLAYERCLIP|WINDOW|MONSTER|GRATE. CS:S overrides it to add team-based player clip contents. The contents bits are documented on the wiki's BSP flags (Source) page (bspflags.h)."
        },
        {
          "name": "IMoveHelper::AddToTouched",
          "klass": "other",
          "signature": "void IMoveHelper::AddToTouched( const trace_t& tr, const Vector& impactvelocity )",
          "inputs": "trace result, impact velocity",
          "outputs": "Appends the hit entity to the per-tick touch list if not already present",
          "purity": "mutates-member-state",
          "callable_directly": "no",
          "notes": "Called from TryPlayerMove on every blocking hit and from SetGroundEntity when the ground is not the world. The list is drained by moveHelper->ProcessImpacts() after ProcessMovement returns - THAT is when trigger_push/trigger_teleport actually fire."
        },
        {
          "name": "CGameMovement::StepMove",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::StepMove( Vector &vecDestination, trace_t &trace )",
          "inputs": "vecDestination, incoming trace, mv->GetAbsOrigin(), mv->m_vecVelocity, player->m_Local.m_bAllowAutoMovement, player->m_Local.m_flStepSize (sv_stepsize, 18), gpGlobals->frametime",
          "outputs": "Saves origin and velocity. (A) DOWN attempt: TryPlayerMove(&vecEndPos, &trace); records vecDownPos/vecDownVel. (B) Restores origin/velocity; traces UP by m_flStepSize (scaled by TIME_TO_TICKS(frametime) when auto-movement is allowed); if not startsolid/allsolid, SetAbsOrigin(trace.endpos); TryPlayerMove(); then traces DOWN by the same step size. If the landing plane's normal.z < 0.7, reverts entirely to vecDownPos/vecDownVel, adds the step height, and returns. Otherwise SetAbsOrigin(trace.endpos) if free. (C) Compares horizontal distance travelled by the down attempt vs the up attempt; takes whichever went farther, but when taking the UP result it still copies vecDownVel.z into velocity.z. Adds the net z gain to mv->m_outStepHeight.",
          "purity": "mutates-member-state",
          "callable_directly": "no - runs TryPlayerMove twice and mutates origin/velocity",
          "notes": "Two full slide-move attempts per tick when stepping. The 'take the farther one but keep the down attempt's vz' rule is easy to miss. sv_stepsize default 18 (wiki confirms: 'sets the max height that the player will automatically snap up to when moving into a wall'), and the Dimensions page lists 18 as the step height for every stance in HL2, CS:S and GMod, with 16 noted as 'Max step height' in its grid table."
        },
        {
          "name": "CGameMovement::StayOnGround",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::StayOnGround( void )",
          "inputs": "mv->GetAbsOrigin(), player->GetStepSize(), PlayerSolidMask(), COLLISION_GROUP_PLAYER_MOVEMENT, COORD_RESOLUTION",
          "outputs": "start = origin with z += 2; end = origin with z -= stepSize. First traces origin -> start (to find a safe start height), takes trace.endpos as the new start. Then traces start -> end. If fraction > 0 AND fraction < 1 AND !startsolid AND plane.normal.z >= 0.7: if |origin.z - trace.endpos.z| > 0.5*COORD_RESOLUTION, SetAbsOrigin(trace.endpos).",
          "purity": "mutates-member-state",
          "callable_directly": "unknown - needs a trace oracle, otherwise self-contained",
          "notes": "This is what glues you to stairs and ramps as you walk down them instead of launching. Called at the end of WalkMove on both the clean-move and the StepMove paths. The COORD_RESOLUTION guard exists because network quantization makes tiny corrections unrepresentable."
        },
        {
          "name": "CGameMovement::CheckFalling",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::CheckFalling( void )",
          "inputs": "player->GetGroundEntity(), player->m_Local.m_flFallVelocity, PLAYER_FALL_PUNCH_THRESHOLD (350), PLAYER_MAX_SAFE_FALL_SPEED (526.5), player->GetWaterLevel(), IsDead()",
          "outputs": "Early-returns if airborne or m_flFallVelocity <= 0. Otherwise plays landing effects, sets m_Local.m_vecPunchAngle[ROLL] = m_flFallVelocity * 0.013, calls MoveHelper()->PlayerFallingDamage(). ALWAYS ends by setting player->m_Local.m_flFallVelocity = 0.",
          "purity": "mutates-member-state",
          "callable_directly": "no - deals damage and plays effects",
          "notes": "Despite the name it handles LANDING, not falling. m_flFallVelocity was captured at the top of PlayerMove as -velocity.z while airborne. The punch angle it writes feeds back into next tick's CheckParameters via DecayPunchAngle. Fall damage thresholds are tabulated in detail on the wiki's Dimensions page (no damage below 240 units of drop; 100 damage at 726+; falling into water >= 2 units deep negates all fall damage)."
        },
        {
          "name": "CGameMovement::PlayerRoughLandingEffects",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::PlayerRoughLandingEffects( float fvol )",
          "inputs": "fvol, player surface data",
          "outputs": "Plays landing sound / view punch",
          "purity": "mutates-member-state",
          "callable_directly": "no - pure side effects",
          "notes": "Called both from CheckFalling and from the end of TryPlayerMove (for lateral slams, when the 2D speed lost exceeds PLAYER_MAX_SAFE_FALL_SPEED). Cosmetic; safe to stub."
        },
        {
          "name": "CGameMovement::PlaySwimSound",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::PlaySwimSound( void )",
          "inputs": "player",
          "outputs": "MoveHelper()->StartSound(...)",
          "purity": "mutates-member-state",
          "callable_directly": "no",
          "notes": "Cosmetic. Fires on water-level transitions at the end of FullWalkMove."
        },
        {
          "name": "CGameMovement::FullNoClipMove",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::FullNoClipMove( float factor, float maxacceleration )",
          "inputs": "sv_noclipspeed (5), sv_noclipaccelerate (5), mv->m_flForwardMove/m_flSideMove/m_flUpMove, m_vecForward/m_vecRight/m_vecUp, mv->m_flMaxSpeed",
          "outputs": "Builds a full 3D wishvel from the UNPROJECTED basis (z NOT zeroed), scales by factor, either accelerates toward it or sets velocity outright depending on maxacceleration, then SetAbsOrigin(origin + velocity*frametime) with NO trace at all",
          "purity": "mutates-member-state",
          "callable_directly": "yes - no collision involved",
          "notes": "No gravity, no friction, no collision. Note the wiki's sv_friction page warns that negative sv_friction while in noclip accelerates infinitely until the game crashes."
        },
        {
          "name": "CGameMovement::FullTossMove",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::FullTossMove( void )",
          "inputs": "player->GetMoveType() (FLY vs FLYGRAVITY), mv->m_vecVelocity, base velocity, sv_friction",
          "outputs": "CheckWater(); if FLYGRAVITY, AddGravity() (the FULL-step version); adds base velocity; traces the move and ClipVelocity's off the impact plane; applies friction; subtracts base velocity; CheckWater()",
          "purity": "mutates-member-state",
          "callable_directly": "no",
          "notes": "The only caller of AddGravity. Used for MOVETYPE_FLY / MOVETYPE_FLYGRAVITY, per the wiki's MoveType page (4 and 5)."
        },
        {
          "name": "CGameMovement::FullObserverMove",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::FullObserverMove( void )",
          "inputs": "player->GetObserverMode(), sv_specnoclip, spectator hull VEC_OBS_HULL_MIN/MAX",
          "outputs": "Roams or locks to a target; in roaming mode accelerates and moves with optional collision",
          "purity": "mutates-member-state",
          "callable_directly": "no",
          "notes": "VEC_OBS_HULL_MIN/MAX are (-10,-10,-10)..(10,10,10) per the wiki's Player Entity page (a 20x20x20 hull, which that page flags as needing confirmation)."
        },
        {
          "name": "CGameMovement::FinishMove",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::FinishMove( void )",
          "inputs": "mv->m_nButtons",
          "outputs": "mv->m_nOldButtons = mv->m_nButtons",
          "purity": "mutates-member-state",
          "callable_directly": "yes - one assignment",
          "notes": "One line, but it is the edge-detection latch for the ENTIRE next tick: jump release, duck press, duck release all key off m_nOldButtons. Note CheckJumpButton and Duck have already OR'd/AND'd bits into m_nOldButtons before this overwrites it wholesale - so those in-tick latches are transient and only matter within the tick."
        },
        {
          "name": "CPlayerMove::FinishMove",
          "klass": "other",
          "signature": "void CPlayerMove::FinishMove( CBasePlayer *player, CUserCmd *ucmd, CMoveData *move )",
          "inputs": "CMoveData after ProcessMovement",
          "outputs": "player->m_flMaxspeed = move->m_flClientMaxSpeed; player->SetAbsOrigin(move->GetAbsOrigin()); player->SetAbsVelocity(move->m_vecVelocity); SetPreviouslyPredictedOrigin(...); player->m_Local.m_nOldButtons = move->m_nButtons; copies m_outStepHeight, m_outWishVel, m_outJumpVel. Deliberately does NOT copy back the angles.",
          "purity": "mutates-member-state",
          "callable_directly": "unknown - a struct copy back onto the entity",
          "notes": "The CMoveData->player unmarshalling boundary; hooked at vtable index 19 in the user's DLL. The angles are intentionally not written back because the movement code treats its copy as non-authoritative."
        },
        {
          "name": "IMoveHelper::ProcessImpacts",
          "klass": "other",
          "signature": "void IMoveHelper::ProcessImpacts( void )",
          "inputs": "The touch list accumulated by AddToTouched during the tick",
          "outputs": "Delivers Touch()/StartTouch() to every touched entity, in order",
          "purity": "mutates-member-state",
          "callable_directly": "no",
          "notes": "This is where trigger volumes and pushable entities actually get their callbacks. It runs AFTER ProcessMovement and FinishMove, so a trigger touched this tick affects the NEXT tick's movement, not this one."
        },
        {
          "name": "CBaseEntity::PhysicsTouchTriggers",
          "klass": "other",
          "signature": "void CBaseEntity::PhysicsTouchTriggers( const Vector *pPrevAbsOrigin = NULL )",
          "inputs": "entity bounds, previous origin, the trigger spatial partition",
          "outputs": "Calls StartTouch/Touch/EndTouch on trigger volumes whose bounds the entity entered/occupies/left",
          "purity": "mutates-member-state",
          "callable_directly": "no - requires the spatial partition and entity system",
          "notes": "Invoked from SetAbsOrigin and from the post-think pass. This plus ProcessImpacts are the two paths by which trigger_push/gravity/teleport reach the player."
        },
        {
          "name": "CTriggerPush::Touch",
          "klass": "other",
          "signature": "void CTriggerPush::Touch( CBaseEntity *pOther )",
          "inputs": "m_vecPushDir (from the pushdir angles keyvalue), m_flSpeed (speed keyvalue, in units/second), m_flAlternateTicksFix, pOther's movetype and flags",
          "outputs": "For players (MOVETYPE_WALK/STEP): computes vecPush = m_vecPushDir * m_flSpeed, folds in any existing base velocity, then pOther->SetBaseVelocity(vecPush) and pOther->AddFlag(FL_BASEVELOCITY). Entities with a parent are skipped.",
          "purity": "mutates-member-state",
          "callable_directly": "no - entity method",
          "notes": "Confirmed on the wiki's Trigger push page: keyvalues are Push Direction (pushdir, angles), Speed of Push (speed, integer, 'in inches / second'), and 'Scale force for alternate ticks' (alternateticksfix, float). The base velocity it sets is consumed by CheckMovingGround on subsequent ticks and by StartGravity for the z component. The wiki's Velocity page documents the base/local/abs split and warns to use SetBaseVelocity rather than touching local or abs velocity."
        },
        {
          "name": "CTriggerGravity::GravityTouch",
          "klass": "other",
          "signature": "void CTriggerGravity::GravityTouch( CBaseEntity *pOther )",
          "inputs": "the gravity keyvalue (float multiplier)",
          "outputs": "pOther->SetGravity( GetGravity() ) - sets the per-entity gravity multiplier that StartGravity/FinishGravity/AddGravity read as ent_gravity",
          "purity": "mutates-member-state",
          "callable_directly": "no",
          "notes": "Wiki Trigger gravity page is explicit and useful: the change PERSISTS until the player touches another trigger_gravity, even across respawn in multiplayer; 1 is normal, 0 is none; negative and decimal values are accepted; it affects ONLY the player's gravity; and it also changes the fall height before fall damage. That persistence is why a solver must carry gravity_scale as player state, exactly as the user's PlayerState::gravity_scale does."
        },
        {
          "name": "CTriggerTeleport::Touch",
          "klass": "other",
          "signature": "void CTriggerTeleport::Touch( CBaseEntity *pOther )",
          "inputs": "target destination entity, optional landmark entity, pOther's origin/angles/velocity",
          "outputs": "pOther->Teleport(&newOrigin, &newAngles, &newVelocity). Without a landmark, angles are SET to the destination's angles. With a Local Destination Landmark, the entity is offset from the target by its initial offset from the landmark and angles are LEFT ALONE.",
          "purity": "mutates-member-state",
          "callable_directly": "no",
          "notes": "Wiki Trigger teleport page confirms both the plain and landmark-relative behaviors. The landmark form is the one that preserves velocity direction meaningfully and is what surf/bhop map stage transitions use."
        },
        {
          "name": "IEngineTrace::TraceRay",
          "klass": "IEngineTrace",
          "signature": "void IEngineTrace::TraceRay( const Ray_t &ray, unsigned int fMask, ITraceFilter *pTraceFilter, trace_t *pTrace )",
          "inputs": "Ray_t (m_Start, m_Delta, m_StartOffset, m_Extents, m_IsRay, m_IsSwept), contents mask, trace filter, and the world's collision representation (BSP nodes/leaves/brushes plus entity collideables)",
          "outputs": "Fills trace_t: fraction, endpos, startsolid, allsolid, plane (normal + dist), surface (name, surfaceProps, flags), contents, m_pEnt, hitbox/hitgroup, fractionleftsolid",
          "purity": "pure",
          "callable_directly": "yes - it is the engine's trace oracle and is exactly what the user's repo hooked for the trace-oracle work",
          "notes": "VDC has NO IEngineTrace page and no CGameTrace page (404). The trace_t page is a 4-line stub. Everything above about the struct comes from the SDK headers, not the wiki. This is the single external dependency of the whole movement system - given a faithful TraceRay, every CGameMovement function becomes reimplementable."
        },
        {
          "name": "IEngineTrace::GetPointContents",
          "klass": "IEngineTrace",
          "signature": "int IEngineTrace::GetPointContents( const Vector &vecAbsPosition, int contentsMask, IHandleEntity **ppEntity )",
          "inputs": "a world-space point, contents mask",
          "outputs": "Returns the OR of contents flags at that point (CONTENTS_WATER, CONTENTS_SLIME, CONTENTS_LADDER, CONTENTS_SOLID, ...)",
          "purity": "pure",
          "callable_directly": "yes",
          "notes": "Used only via GetPointContentsCached, for water detection. Contents flags are documented on the wiki's BSP flags (Source) page as bspflags.h."
        },
        {
          "name": "IEngineTrace::ClipRayToEntity / SweepCollideable / PointOutsideWorld",
          "klass": "IEngineTrace",
          "signature": "void ClipRayToEntity( const Ray_t&, unsigned int fMask, IHandleEntity*, trace_t* ); void SweepCollideable( ICollideable*, const Vector& start, const Vector& end, const QAngle& angles, unsigned int fMask, ITraceFilter*, trace_t* ); bool PointOutsideWorld( const Vector& )",
          "inputs": "ray/collideable, mask, filter",
          "outputs": "trace_t / bool",
          "purity": "pure",
          "callable_directly": "yes",
          "notes": "Not called by CGameMovement directly but are the tools for building an independent trace oracle against specific brush models (func_ entities). Undocumented on VDC."
        },
        {
          "name": "Ray_t::Init",
          "klass": "IEngineTrace",
          "signature": "void Ray_t::Init( const Vector& start, const Vector& end, const Vector& mins, const Vector& maxs )",
          "inputs": "start, end, hull mins/maxs",
          "outputs": "m_Delta = end - start; m_Extents = (maxs - mins) * 0.5; m_StartOffset = (maxs + mins) * 0.5; m_Start = start + m_StartOffset; m_IsSwept = (delta length != 0); m_IsRay = (extents length < 1e-6)",
          "purity": "pure",
          "callable_directly": "yes",
          "notes": "Critical detail for anyone reimplementing: the ray is RE-CENTERED. Because the player hull is (-16,-16,0)..(16,16,72), m_StartOffset is (0,0,36) and m_Extents is (16,16,36) - the sweep is of a CENTERED box, not a feet-origin box. Getting this wrong offsets every trace by half the hull height."
        },
        {
          "name": "CTraceFilterSimple::ShouldHitEntity",
          "klass": "IEngineTrace",
          "signature": "bool CTraceFilterSimple::ShouldHitEntity( IHandleEntity *pHandleEntity, int contentsMask )",
          "inputs": "the candidate entity, m_pPassEnt, m_collisionGroup, the g_pGameRules->ShouldCollide hook",
          "outputs": "Returns false for the pass entity and its hierarchy, and defers team/collision-group rules to the game rules",
          "purity": "pure",
          "callable_directly": "unknown - needs the entity hierarchy and game rules",
          "notes": "COLLISION_GROUP_PLAYER_MOVEMENT is passed by every movement trace. It is what lets players pass through each other's clip volumes on some servers."
        },
        {
          "name": "BSP lump 1 parse: dplane_t",
          "klass": "BSP-parse",
          "signature": "struct dplane_t { Vector normal; float dist; int type; }",
          "inputs": "LUMP_PLANES bytes",
          "outputs": "20 bytes per plane. normal is a unit vector; dist is the distance from the map origin to the nearest point on the plane. Plane equation Ax + By + Cz = D with (A,B,C) = normal and D = dist. type is the axis identifier.",
          "purity": "pure",
          "callable_directly": "yes",
          "notes": "Verbatim from the wiki's BSP (Source) page, which also notes the lump length must be a multiple of 20 and that each plane divides space into on-plane (F=0), in front (F>0) and behind (F<0). The 0.7 walkable threshold in CategorizePosition is tested against normal.z of the trace's returned plane, which comes from here."
        },
        {
          "name": "BSP lump 18/19 parse: dbrush_t / dbrushside_t",
          "klass": "BSP-parse",
          "signature": "struct dbrush_t { int firstside; int numsides; int contents; }; struct dbrushside_t { unsigned short planenum; short texinfo; short dispinfo; /* BSPVERSION >= 21 */ unsigned char bevel; unsigned char thin; /* else */ short bevel; }",
          "inputs": "LUMP_BRUSHES (12 bytes/entry) and LUMP_BRUSHSIDES (8 bytes/entry)",
          "outputs": "firstside indexes the brushside array; that plus numsides gives all sides of the brush. contents holds the bspflags.h contents flags. planenum indexes the plane array; texinfo and dispinfo reference the texture and displacement lumps; bevel is 1 for bevel planes (axis-aligned padding planes added for hull sweeping); thin marks a side with any edge shorter than step size.",
          "purity": "pure",
          "callable_directly": "yes",
          "notes": "Verbatim from the wiki's BSP (Source) page. Brush array limit is 8192 (MAX_MAP_BRUSHES). The BEVEL PLANES are what make hull-vs-brush sweeping work at all - they are the Minkowski padding. The wiki's Surf page names this directly: rotated instances 'will not be correctly surfable, due to not creating axis-aligned brushside anti-bevels for angled QPhysics geometry'. Note the struct changed at BSP version 21 (L4D2/Portal 2/CS:GO); CS:S is version 19/20, so use the `short bevel` variant."
        },
        {
          "name": "BSP lump 5 parse: dnode_t",
          "klass": "BSP-parse",
          "signature": "struct dnode_t { int planenum; int children[2]; short mins[3]; short maxs[3]; unsigned short firstface; unsigned short numfaces; short area; short padding; }",
          "inputs": "LUMP_NODES bytes",
          "outputs": "32 bytes per node. planenum indexes the plane array. children[] are node indices if positive; if negative the value (-1 - child) is the leaf index. mins/maxs are a rough bounding box for frustum culling. area is the area index if all leaves below share one, else -1.",
          "purity": "pure",
          "callable_directly": "yes",
          "notes": "Verbatim from the wiki's BSP (Source) page. Max 65536 nodes (MAX_MAP_NODES). Note the negative-child encoding is -(leafs + 1) - the page gives the worked example that -100 references leaf 99."
        },
        {
          "name": "BSP lump 10 parse: dleaf_t",
          "klass": "BSP-parse",
          "signature": "struct dleaf_t { int contents; short cluster; short area:9; short flags:7; short mins[3]; short maxs[3]; unsigned short firstleafface; unsigned short numleaffaces; unsigned short firstleafbrush; unsigned short numleafbrushes; short leafWaterDataID; /* lump version 0 only: CompressedLightCube ambientLighting; */ short padding; }",
          "inputs": "LUMP_LEAFS bytes",
          "outputs": "56 or 32 bytes per element depending on lump version. contents is the OR of all brushes in the leaf. area and flags share one 16-bit bitfield. firstleafbrush/numleafbrushes index LUMP_LEAFBRUSHES (lump 17), which in turn indexes the brush array. leafWaterDataID is -1 when not in water.",
          "purity": "pure",
          "callable_directly": "yes",
          "notes": "Verbatim from the wiki's BSP (Source) page, including its note that for lump version 0 (maps of version 19 or lower) the ambientLighting member is present and must be uncommented - which matters for CS:S maps specifically. The leaf -> leafbrush -> brush -> brushside -> plane chain is the full path from a spatial query to the planes you sweep against."
        },
        {
          "name": "BSP displacement parse: ddispinfo_t and displacement collision",
          "klass": "BSP-parse",
          "signature": "struct ddispinfo_t { Vector startPosition; int DispVertStart; int DispTriStart; int power; int minTess; float smoothingAngle; int contents; unsigned short MapFace; int LightmapAlphaStart; int LightmapSamplePositionStart; CDispNeighbor EdgeNeighbors[4]; CDispCornerNeighbors CornerNeighbors[4]; unsigned int AllowedVerts[10]; }",
          "inputs": "LUMP_DISPINFO plus LUMP_DISP_VERTS and LUMP_DISP_TRIS",
          "outputs": "A triangle mesh generated by subdividing the parent brush face; power determines the tessellation (2^power + 1 vertices per side)",
          "purity": "pure",
          "callable_directly": "yes",
          "notes": "The wiki's Displacement page covers displacements as a mapping concept thoroughly but does not document the collision path. For movement this matters because displacement collision is TRIANGLE-based, not brush/plane-based - and the SDK comment inside TryPlayerMove's zero-length re-check names it directly: 'There's a precision issue with terrain tracing that can cause a swept box to successfully trace when the end position is stuck in the triangle.' That extra stuck check exists specifically for displacements."
        }
      ],
      "sources": [
        "https://developer.valvesoftware.com/wiki/Dimensions_(Half-Life_2_and_Counter-Strike:_Source)",
        "https://developer.valvesoftware.com/wiki/Player_Entity",
        "https://developer.valvesoftware.com/wiki/Duck_Jump_Fix",
        "https://developer.valvesoftware.com/wiki/Velocity",
        "https://developer.valvesoftware.com/wiki/Gravity",
        "https://developer.valvesoftware.com/wiki/MoveType",
        "https://developer.valvesoftware.com/wiki/BSP_(Source)",
        "https://developer.valvesoftware.com/wiki/BSP_flags_(Source)",
        "https://developer.valvesoftware.com/wiki/Displacement",
        "https://developer.valvesoftware.com/wiki/Surf",
        "https://developer.valvesoftware.com/wiki/Bunnyhop",
        "https://developer.valvesoftware.com/wiki/Trigger_push",
        "https://developer.valvesoftware.com/wiki/Trigger_gravity",
        "https://developer.valvesoftware.com/wiki/Trigger_teleport",
        "https://developer.valvesoftware.com/wiki/Sv_friction",
        "https://developer.valvesoftware.com/wiki/Sv_gravity",
        "https://developer.valvesoftware.com/wiki/Sv_stepsize",
        "https://developer.valvesoftware.com/wiki/Sv_standable_normal",
        "https://developer.valvesoftware.com/wiki/Console_Command_List/S",
        "https://developer.valvesoftware.com/wiki/Trace_t",
        "https://developer.valvesoftware.com/wiki/UTIL_TraceLine",
        "https://developer.valvesoftware.com/wiki/Func_ladder",
        "https://developer.valvesoftware.com/wiki/Prediction",
        "https://developer.valvesoftware.com/wiki/CBasePlayer",
        "C:\\Users\\Connor\\Documents\\SourceTAS\\sourceTAS\\Source\\Solver\\SolverMove.h (repo mirror: measured constants, stamina disassembly notes, hull_state model)",
        "C:\\Users\\Connor\\Documents\\SourceTAS\\sourceTAS\\Source\\Solver\\SolverMove.cpp (repo mirror: EngineClipVelocity, CheckVelocity, TryPlayerMove bump loop, Friction, StayOnGround)"
      ]
    },
    "binary": {
      "functions": [
        {
          "name": "CGameMovement::~CGameMovement (scalar deleting dtor)",
          "klass": "CGameMovement",
          "signature": "void* __fastcall(CGameMovement* rcx, unsigned char flags dl)",
          "inputs": "this; dl bit0 = delete flag",
          "outputs": "writes this->vfptr = &IGameMovement_vtable (0x438300); optionally frees",
          "purity": "mutates-member-state",
          "callable_directly": "yes - direct RVA call, but do not call (destroys the engine's singleton)",
          "notes": "RVA 0x116a40. vtable idx 0. CCS override 0x1f4a70. confidence HIGH: writes the IGameMovement vtable RVA 0x438300 into this[0], classic MSVC deleting dtor. Non-deleting dtor is 0x116a70."
        },
        {
          "name": "CGameMovement::ProcessMovement",
          "klass": "CGameMovement",
          "signature": "void __fastcall(CGameMovement*, CBasePlayer* pPlayer, CMoveData* pMove)",
          "inputs": "pPlayer (all movement-relevant player state), pMove (buttons, wishmove, angles, velocity, origin), gpGlobals->frametime",
          "outputs": "this->player, this->mv, then full mutation of *pMove and *pPlayer through PlayerMove",
          "purity": "mutates-member-state",
          "callable_directly": "yes - this is the entry point already hooked by the project",
          "notes": "RVA 0x11d360. vtable idx 1 (matches the project's known anchor). CCS override 0x1f7580 (0x73 bytes, sets up then calls into base path). confidence CERTAIN (given anchor + vtable position)."
        },
        {
          "name": "CGameMovement::StartTrackPredictionErrors",
          "klass": "CGameMovement",
          "signature": "void __fastcall(CGameMovement*, CBasePlayer* rdx)",
          "inputs": "pPlayer",
          "outputs": "this->player (this+0x8) = pPlayer",
          "purity": "mutates-member-state",
          "callable_directly": "yes",
          "notes": "RVA 0x4b750 (2 instructions: mov [rcx+8], rdx / ret). vtable idx 2. confidence HIGH: only IGameMovement slot in this position whose body is 'player = pPlayer'. Also proves CGameMovement::player lives at this+0x8."
        },
        {
          "name": "CGameMovement::FinishTrackPredictionErrors",
          "klass": "CGameMovement",
          "signature": "void __fastcall(CGameMovement*, CBasePlayer*)",
          "inputs": "none (compiled out in release)",
          "outputs": "none",
          "purity": "pure",
          "callable_directly": "yes (no-op)",
          "notes": "RVA 0x383f0 - a shared empty 'ret' stub also used at idx 3/25/26/27. vtable idx 3. confidence MEDIUM-HIGH (position in IGameMovement + empty body; PREDICTION_ERROR_CHECK_LEVEL==0)."
        },
        {
          "name": "CGameMovement::DiffPrint",
          "klass": "CGameMovement",
          "signature": "void __fastcall(CGameMovement*, const char* fmt, ...)",
          "inputs": "none effective",
          "outputs": "none",
          "purity": "pure",
          "callable_directly": "yes (no-op)",
          "notes": "RVA 0x119720. vtable idx 4. Body is 'mov [rsp+0x18],r8 ; mov [rsp+0x20],r9 ; ret' - homes varargs then returns. confidence HIGH: the varargs-homing prologue is unique to a variadic empty function."
        },
        {
          "name": "CGameMovement::GetPlayerMins(bool ducked) const",
          "klass": "CGameMovement",
          "signature": "const Vector& __fastcall(CGameMovement*, Vector* sret rdx, bool ducked r8b)",
          "inputs": "ducked flag; player->m_flModelScale (player+0x8e4); global hull table object at [0x607200] (ducked fields +0x24/0x28/0x2c, standing +0xc/0x10/0x14)",
          "outputs": "writes the Vector at rdx; returns rdx",
          "purity": "reads-member-state",
          "callable_directly": "yes",
          "notes": "RVA 0x11c240. vtable idx 5. confidence HIGH: SetDuckedEyeOffset (0x11d8d0) calls vtable+0x28 (=idx5) with ducked=true then ducked=false and subtracts the .z components - exactly GetPlayerMins(true)-GetPlayerMins(false)."
        },
        {
          "name": "CGameMovement::GetPlayerMaxs(bool ducked) const",
          "klass": "CGameMovement",
          "signature": "const Vector& __fastcall(CGameMovement*, Vector* sret, bool ducked)",
          "inputs": "same as GetPlayerMins",
          "outputs": "Vector at sret",
          "purity": "reads-member-state",
          "callable_directly": "yes",
          "notes": "RVA 0x11c0a0. vtable idx 6. confidence MEDIUM-HIGH: identical shape to idx5/idx7; assigned by elimination once idx5=Mins and idx7=ViewOffset were pinned, and by IGameMovement declaration order (Mins, Maxs, ViewOffset)."
        },
        {
          "name": "CGameMovement::GetPlayerViewOffset(bool ducked) const",
          "klass": "CGameMovement",
          "signature": "const Vector& __fastcall(CGameMovement*, Vector* sret, bool ducked)",
          "inputs": "ducked; player->m_flModelScale; global view-offset table",
          "outputs": "Vector at sret",
          "purity": "reads-member-state",
          "callable_directly": "yes",
          "notes": "RVA 0x11c2e0. vtable idx 7. confidence HIGH: SetDuckedEyeOffset calls vtable+0x38 (=idx7) with true/false right after the two GetPlayerMins calls, matching the SDK body verbatim."
        },
        {
          "name": "CGameMovement::GetPlayerMins(void) const",
          "klass": "CGameMovement",
          "signature": "Vector __fastcall(CGameMovement*, Vector* sret rdx)",
          "inputs": "player (this+0x8) hull-type virtual [player_vt+0x790]; player->m_flModelScale",
          "outputs": "Vector at sret",
          "purity": "reads-member-state",
          "callable_directly": "yes",
          "notes": "RVA 0x11c140. vtable idx 8. confidence HIGH: TracePlayerBBox (0x11e520) calls idx8 and idx9 to build Ray_t::Init(start,end,mins,maxs); MSVC right-to-left evaluation makes idx9=Maxs (called first) and idx8=Mins."
        },
        {
          "name": "CGameMovement::GetPlayerMaxs(void) const",
          "klass": "CGameMovement",
          "signature": "Vector __fastcall(CGameMovement*, Vector* sret rdx)",
          "inputs": "player hull type; model scale",
          "outputs": "Vector at sret",
          "purity": "reads-member-state",
          "callable_directly": "yes",
          "notes": "RVA 0x11bfa0. vtable idx 9. confidence HIGH (see idx 8 - it is the first of the pair invoked by TracePlayerBBox)."
        },
        {
          "name": "CGameMovement::TracePlayerBBox",
          "klass": "CGameMovement",
          "signature": "void __fastcall(CGameMovement*, const Vector& start rdx, const Vector& end r8, unsigned int fMask r9d, int collisionGroup [stack], trace_t& pm [stack])",
          "inputs": "start, end, fMask, collisionGroup; GetPlayerMins()/GetPlayerMaxs() (idx8/idx9); mv->m_nPlayerHandle; world collision state",
          "outputs": "fills pm (trace_t)",
          "purity": "reads-member-state",
          "callable_directly": "yes - highest-value entry point for a TAS collision oracle",
          "notes": "RVA 0x11e520. vtable idx 10. confidence HIGH: builds a Ray from idx8/idx9 then traces; called by CanUnduck (idx50) and CanUnDuckJump together with PlayerSolidMask (idx12). References r_visualizetraces."
        },
        {
          "name": "unknown_0011ef50 (trace wrapper, likely TryTouchGround)",
          "klass": "CGameMovement",
          "signature": "void __fastcall(CGameMovement*, ... >=5 args, 5th read from [rsp+0xe0])",
          "inputs": "5+ args incl. a stack arg; r_visualizetraces",
          "outputs": "fills a trace_t",
          "purity": "reads-member-state",
          "callable_directly": "yes but signature unproven",
          "notes": "RVA 0x11ef50. vtable idx 11. CCS override 0x1f7680. confidence LOW on name - only proven facts: >=5 parameters, forwards to 0x76480, references r_visualizetraces, sits between TracePlayerBBox and PlayerSolidMask. Do NOT assume TracePlayerBBox here (that is idx 10)."
        },
        {
          "name": "CGameMovement::PlayerSolidMask",
          "klass": "CGameMovement",
          "signature": "unsigned int __fastcall(CGameMovement*, bool brushOnly dl)",
          "inputs": "brushOnly only",
          "outputs": "returns 0x1400B (MASK_PLAYERSOLID_BRUSHONLY) if brushOnly else 0x201400B (MASK_PLAYERSOLID)",
          "purity": "pure",
          "callable_directly": "yes",
          "notes": "RVA 0x11d350. vtable idx 12. CCS override 0x1f74d0 (returns 0x1400b/0x3400b depending on this+0x8). confidence CERTAIN: body is 'movzx eax,dl; xor eax,1; shl eax,0x19; add eax,0x1400b' which yields exactly the two Source mask constants."
        },
        {
          "name": "CGameMovement::PlayerMove",
          "klass": "CGameMovement",
          "signature": "void __fastcall(CGameMovement*)",
          "inputs": "mv (all fields), player (movetype, flags, health), gpGlobals->frametime, sv_noclipspeed, sv_noclipaccelerate, sv_optimizedmovement",
          "outputs": "dispatches whole move; clears mv->m_outWishVel/m_outJumpVel; mutates mv and player",
          "purity": "mutates-member-state",
          "callable_directly": "yes",
          "notes": "RVA 0x11ced0 (3 chunks, 884 bytes). vtable idx 13. CCS override 0x1f6d00. confidence CERTAIN: first virtual call is [vt+0x150]=idx42 CheckParameters, then [vt+0x158]=idx43 ReduceTimers, [vt+0xe0]=idx28 GetCheckInterval, [vt+0x138]=idx39 CheckStuck, [vt+0x148]=idx41 CategorizePosition. Movetype jump table at RVA 0x11d218 (11 entries)."
        },
        {
          "name": "CGameMovement::CalcRoll",
          "klass": "CGameMovement",
          "signature": "float __fastcall(CGameMovement*, const QAngle& angles rdx, const Vector& velocity r8, float rollangle xmm3, float rollspeed [stack])",
          "inputs": "angles, velocity, rollangle, rollspeed",
          "outputs": "returns roll angle",
          "purity": "pure",
          "callable_directly": "yes",
          "notes": "RVA 0x54720. vtable idx 14 (shared CGM/CCS). confidence MEDIUM-HIGH: calls AngleVectors with two lea'd out-vectors then dots velocity with right; and CheckParameters (idx42) is the function that references sv_rollangle/sv_rollspeed."
        },
        {
          "name": "CGameMovement::DecayPunchAngle",
          "klass": "CGameMovement",
          "signature": "void __fastcall(CGameMovement*)",
          "inputs": "player->m_Local.m_vecPunchAngle, m_vecPunchAngleVel; gpGlobals->frametime",
          "outputs": "mutates player punch angle + velocity",
          "purity": "mutates-member-state",
          "callable_directly": "yes",
          "notes": "RVA 0x119390. vtable idx 15. CCS override 0x1f5ae0. confidence HIGH: loads the f64 0.001 at RVA 0x3f5f58 as the LengthSqr threshold, exactly the SDK body."
        },
        {
          "name": "CGameMovement::CheckWaterJump",
          "klass": "CGameMovement",
          "signature": "void __fastcall(CGameMovement*)",
          "inputs": "mv->m_vecViewAngles, mv->m_vecVelocity, mv->m_vecAbsOrigin, player->m_flWaterJumpTime, world trace",
          "outputs": "sets player->m_flWaterJumpTime, mv->m_vecVelocity[2]",
          "purity": "mutates-member-state",
          "callable_directly": "yes",
          "notes": "RVA 0x1187a0 (7 chunks, 1184 bytes). vtable idx 16. confidence HIGH: contains 24.0f (TRACE_DISTANCE), 8.0f (WATERJUMP_HEIGHT) and the ~0.7 vertical-wall test."
        },
        {
          "name": "CGameMovement::WaterMove",
          "klass": "CGameMovement",
          "signature": "void __fastcall(CGameMovement*)",
          "inputs": "mv->m_vecViewAngles, m_flForwardMove(+0x2c), m_flSideMove(+0x34), m_flUpMove(+0x38), m_flMaxSpeed(+0x3c), m_flClientMaxSpeed(+0x40), m_nButtons(+0x24), player->m_surfaceFriction(+0x1868), sv_accelerate, sv_friction, gpGlobals->frametime",
          "outputs": "mv->m_vecVelocity, mv origin via move",
          "purity": "mutates-member-state",
          "callable_directly": "yes",
          "notes": "RVA 0x11f980 (1422 bytes). vtable idx 17. confidence CERTAIN: decoded the exact SDK shape - wishvel[2] -= 60 when all inputs zero, upward exaggeration '* forward.z * 2' clamped to [0, m_flClientMaxSpeed]. Water friction site at 0x11fb81 reads player+0x1868 (the m_surfaceFriction site named in the project's notes)."
        },
        {
          "name": "CGameMovement::AirAccelerate",
          "klass": "CGameMovement",
          "signature": "void __fastcall(CGameMovement*, Vector& wishdir rdx, float wishspeed xmm2, float accel xmm3)",
          "inputs": "wishdir, wishspeed, accel; mv->m_vecVelocity; player->m_surfaceFriction (player+0x1868); GetAirSpeedCap() (idx20); gpGlobals->frametime",
          "outputs": "mv->m_vecVelocity",
          "purity": "mutates-member-state",
          "callable_directly": "yes - key for strafe math",
          "notes": "RVA 0x116b80 (3 chunks, 369 bytes). vtable idx 18. confidence HIGH: 'mulss xmm8,[rax+0x1868]' at 0x116c32 is the accelspeed *= m_surfaceFriction term; signature (Vector&, float, float) matches; sits directly before AirMove."
        },
        {
          "name": "CGameMovement::AirMove",
          "klass": "CGameMovement",
          "signature": "void __fastcall(CGameMovement*)",
          "inputs": "mv->m_vecViewAngles(+0xc), m_flForwardMove, m_flSideMove, m_flMaxSpeed; sv_airaccelerate; player state",
          "outputs": "calls AirAccelerate then TryPlayerMove; mutates mv->m_vecVelocity and origin",
          "purity": "mutates-member-state",
          "callable_directly": "yes",
          "notes": "RVA 0x116d00 (670 bytes). vtable idx 19. CCS override 0x1f4ab0 (calls base 0x116d00 then 0x1f4fc0). confidence CERTAIN: unconditionally zeroes forward[2] and right[2] and re-normalizes both (WalkMove does this conditionally under g_bMovementOptimizations), matching AirMove exactly."
        },
        {
          "name": "CGameMovement::GetAirSpeedCap",
          "klass": "CGameMovement",
          "signature": "float __fastcall(CGameMovement*)",
          "inputs": "none",
          "outputs": "returns 30.0f",
          "purity": "pure",
          "callable_directly": "yes",
          "notes": "RVA 0x11bf00. vtable idx 20. confidence CERTAIN: whole body is 'movss xmm0,[0x40ec18]=30.0f ; ret'. The 30 air-speed-cap constant lives at RVA 0x40ec18."
        },
        {
          "name": "CGameMovement::CanAccelerate",
          "klass": "CGameMovement",
          "signature": "bool __fastcall(CGameMovement*)",
          "inputs": "player->pl.deadflag (player+0x1560), player->m_flWaterJumpTime (player+0x1684)",
          "outputs": "returns bool",
          "purity": "reads-member-state",
          "callable_directly": "yes",
          "notes": "RVA 0x116fa0. vtable idx 21. CCS override 0x1f4ad0. confidence CERTAIN: 'if (player[0x1560]) return false; if (player[0x1684] != 0) return false; return true' is the SDK body verbatim. Pins player->pl.deadflag=+0x1560 and m_flWaterJumpTime=+0x1684."
        },
        {
          "name": "CGameMovement::Accelerate",
          "klass": "CGameMovement",
          "signature": "void __fastcall(CGameMovement*, Vector& wishdir rdx, float wishspeed xmm2, float accel xmm3)",
          "inputs": "wishdir, wishspeed, accel; mv->m_vecVelocity, mv->m_flMaxSpeed; player->m_surfaceFriction (+0x1868); CanAccelerate(); gpGlobals->frametime",
          "outputs": "mv->m_vecVelocity",
          "purity": "mutates-member-state",
          "callable_directly": "yes",
          "notes": "RVA 0x116aa0 (219 bytes). vtable idx 22. confidence HIGH: 'mulss xmm6,[rax+0x1868]' at 0x116b14 (accelspeed *= m_surfaceFriction) and the (Vector&,float,float) signature; immediately follows CanAccelerate."
        },
        {
          "name": "CGameMovement::WalkMove",
          "klass": "CGameMovement",
          "signature": "void __fastcall(CGameMovement*)",
          "inputs": "mv->m_vecViewAngles, m_flForwardMove, m_flSideMove, m_flMaxSpeed, m_vecVelocity, m_vecAbsOrigin(+0x9c); player ground entity, m_surfaceFriction; sv_accelerate; g_bMovementOptimizations; gpGlobals->frametime",
          "outputs": "mv->m_vecVelocity, mv->m_vecAbsOrigin, mv->m_outWishVel, mv->m_outStepHeight; calls StepMove/StayOnGround",
          "purity": "mutates-member-state",
          "callable_directly": "yes",
          "notes": "RVA 0x11f330 (5 chunks, 1613 bytes). vtable idx 23. CCS override 0x1f7790 (calls base). confidence CERTAIN: cross-pinned against the project's server anchor WalkMove@server 0x2f0140 which is server CCSGameMovement vtable idx 23; references sv_accelerate; calls StayOnGround (0x11de60)."
        },
        {
          "name": "CGameMovement::FullWalkMove",
          "klass": "CGameMovement",
          "signature": "void __fastcall(CGameMovement*)",
          "inputs": "CheckWater(); player ground entity, water level, m_flWaterJumpTime, gravity(+0x1dc), m_vecBaseVelocity(+0x1d0), m_surfaceFriction(+0x1868); mv->m_nButtons; sv_friction, sv_stopspeed, sv_gravity; gpGlobals->frametime",
          "outputs": "full walk-frame mutation of mv and player; Friction() and StartGravity() are INLINED here",
          "purity": "mutates-member-state",
          "callable_directly": "yes",
          "notes": "RVA 0x11ba70 (3 chunks, 1167 bytes). vtable idx 24. confidence CERTAIN: opens with 'if (!CheckWater()) { velocity.z -= gravity*0.5*frametime; ... CheckVelocity(); }' - i.e. StartGravity inlined; references sv_friction+sv_stopspeed (Friction inlined, m_surfaceFriction read at 0x11bd55). Reached from PlayerMove's jump table for MOVETYPE_ISOMETRIC(1) and MOVETYPE_WALK(2) via [vt+0xc0]."
        },
        {
          "name": "unknown_000383f0_idx25 (empty void f(float) hook; likely OnJump)",
          "klass": "CGameMovement",
          "signature": "void __fastcall(CGameMovement*, float xmm1)",
          "inputs": "none in base",
          "outputs": "none in base",
          "purity": "pure",
          "callable_directly": "yes (no-op in base)",
          "notes": "RVA 0x383f0 (shared empty stub). vtable idx 25. CCS override 0x1f6cc0 -> 'mov rcx,[rcx+0xee0]; jmp 0x1fadc0' where 0x1fadc0 forwards a float (xmm1) to a CBaseEntity virtual [+0xbb0]. confidence LOW on name; CERTAIN that it is an empty-in-base void(float) hook that CS:S overrides."
        },
        {
          "name": "unknown_000383f0_idx26 (empty void f(float) hook; likely OnLand)",
          "klass": "CGameMovement",
          "signature": "void __fastcall(CGameMovement*, float xmm1)",
          "inputs": "none in base",
          "outputs": "none in base",
          "purity": "pure",
          "callable_directly": "yes (no-op in base)",
          "notes": "RVA 0x383f0. vtable idx 26. CCS override 0x1f6cf0 -> 'mov rcx,[rcx+0xee0]; jmp 0x1fadf0'. Same reasoning as idx 25. confidence LOW on name."
        },
        {
          "name": "unknown_000383f0_idx27 (empty hook; likely OnTryPlayerMoveCollision)",
          "klass": "CGameMovement",
          "signature": "void __fastcall(CGameMovement*, trace_t& rdx)",
          "inputs": "none",
          "outputs": "none",
          "purity": "pure",
          "callable_directly": "yes (no-op)",
          "notes": "RVA 0x383f0. vtable idx 27. Not overridden by CCS. confidence LOW on name - only proven that it is an empty stub."
        },
        {
          "name": "CGameMovement::GetCheckInterval",
          "klass": "CGameMovement",
          "signature": "int __fastcall(CGameMovement*, IntervalType_t type edx)",
          "inputs": "type enum only (GROUND=0, STUCK=1, LADDER=2)",
          "outputs": "returns an int interval",
          "purity": "pure",
          "callable_directly": "yes",
          "notes": "RVA 0x11bf10. vtable idx 28. confidence HIGH: body is a 3-way compare on edx returning small ints, and PlayerMove calls it via [vt+0xe0] with edx=1 (STUCK) right before CheckStuck."
        },
        {
          "name": "CGameMovement::CheckJumpButton",
          "klass": "CGameMovement",
          "signature": "bool __fastcall(CGameMovement*)",
          "inputs": "mv->m_nButtons/m_nOldButtons, mv->m_vecVelocity; player ground entity, water level, m_flWaterJumpTime, duck state; gpGlobals->frametime",
          "outputs": "sets mv->m_vecVelocity[2] = 268.3281573 (sqrt(2*800*45)), mv->m_outJumpVel, clears ground entity; calls FinishGravity",
          "purity": "mutates-member-state",
          "callable_directly": "yes",
          "notes": "RVA 0x117950 (7 chunks, 696 bytes). vtable idx 29. confidence CERTAIN: references the f32 268.3281573 at RVA 0x438678 from 0x117b28, and cross-pins to the project's server anchor CheckJumpButton@server 0x2eddab which lies inside server CCS vtable idx 29 (0x2edb30)."
        },
        {
          "name": "CCSGameMovement::CheckJumpButton",
          "klass": "CCSGameMovement",
          "signature": "bool __fastcall(CCSGameMovement*)",
          "inputs": "player->m_flStamina, m_flMaxspeed(+0x1618), duck state, ground entity; mv->m_nButtons; sv_enablebunnyhopping/sv_autobunnyhopping; gpGlobals->frametime",
          "outputs": "velocity.z = sqrt(2*800*57) = 301.99337741082996 scaled by stamina; applies 0.00019 stamina jump cost",
          "purity": "mutates-member-state",
          "callable_directly": "yes",
          "notes": "RVA 0x1f5330. CCS vtable idx 29. confidence CERTAIN: references the f64 301.99337741082996 at RVA 0x4783e0 (mulsd at 0x1f55a0 and 0x1f55b2) and the f32 0.00019 stamina constant at RVA 0x4783d8 (mulss at 0x1f55e9). This is THE CS:S jump function."
        },
        {
          "name": "CGameMovement::FullTossMove",
          "klass": "CGameMovement",
          "signature": "void __fastcall(CGameMovement*)",
          "inputs": "CheckWater(); mv->m_flForwardMove/m_flSideMove/m_flUpMove, m_vecViewAngles, m_flMaxSpeed, m_nButtons; player->m_surfaceFriction(+0x1868), move collide, gravity; sv_accelerate; gpGlobals->frametime",
          "outputs": "mv->m_vecVelocity and origin; PerformFlyCollisionResolution is INLINED (backoff = 2.0 - m_surfaceFriction at 0x11b8f3, plus the 0.7 floor test)",
          "purity": "mutates-member-state",
          "callable_directly": "yes",
          "notes": "RVA 0x11b310 (6 chunks, 1884 bytes). vtable idx 30. confidence CERTAIN: PlayerMove's jump table sends MOVETYPE_FLY(4) and MOVETYPE_FLYGRAVITY(5) to 'jmp [vt+0xf0]' = idx 30; body opens 'CheckWater(); if (fmove||smove||upmove) {AngleVectors...Accelerate(sv_accelerate)}' matching the SDK verbatim. Calls PushEntity (0x11d700), ClipVelocity (0x118c50), CheckVelocity (0x1182e0)."
        },
        {
          "name": "CGameMovement::FullLadderMove",
          "klass": "CGameMovement",
          "signature": "void __fastcall(CGameMovement*)",
          "inputs": "CheckWater(); mv->m_nButtons(+0x24)&IN_JUMP; player->m_vecBaseVelocity(+0x1d0)",
          "outputs": "mv->m_nOldButtons, mv->m_vecVelocity(+0x44..0x4c); calls TryPlayerMove",
          "purity": "mutates-member-state",
          "callable_directly": "yes",
          "notes": "RVA 0x11a9b0 (196 bytes). vtable idx 31. confidence CERTAIN: PlayerMove jump table sends MOVETYPE_LADDER(9) to 'jmp [vt+0xf8]' = idx 31; body is the SDK FullLadderMove verbatim - CheckWater, IN_JUMP->CheckJumpButton else clear, add base velocity, TryPlayerMove ([vt+0x100]=idx32), subtract base velocity."
        },
        {
          "name": "CGameMovement::TryPlayerMove",
          "klass": "CGameMovement",
          "signature": "int __fastcall(CGameMovement*, Vector* pFirstDest rdx, trace_t* pFirstTrace r8)",
          "inputs": "mv->m_vecVelocity, mv->m_vecAbsOrigin(+0x9c); player->m_surfaceFriction(+0x1868), movetype, ground entity; sv_bounce; gpGlobals->frametime; world geometry via TracePlayerBBox",
          "outputs": "mv->m_vecAbsOrigin, mv->m_vecVelocity; returns blocked bitmask (1=floor, 2=step/wall)",
          "purity": "mutates-member-state",
          "callable_directly": "yes - core collision/slide routine",
          "notes": "RVA 0x11e660 (single chunk, 2286 bytes). vtable idx 32. confidence CERTAIN: contains the f64 0.7 walkable test (movsd from RVA 0x41ce50) AND the distinctive air-bounce overbounce 'ClipVelocity(..., 1.0 + sv_bounce.GetFloat() * (1 - player->m_surfaceFriction))' - sv_bounce parent load at 0x11ea3e and 'subss xmm0,[rax+0x1868]' at 0x11ea66. Calls ClipVelocity (0x118c50). This is the function your TAS engine must match bit-for-bit."
        },
        {
          "name": "CGameMovement::LadderMove",
          "klass": "CGameMovement",
          "signature": "bool __fastcall(CGameMovement*)",
          "inputs": "mv->m_vecViewAngles, m_flForwardMove/m_flSideMove, m_nButtons, m_vecAbsOrigin; player movetype/ground; sv_ladder_angle, sv_ladder_dampen; ClimbSpeed(), LadderLateralMultiplier(), LadderDistance(), LadderMask(), OnLadder()",
          "outputs": "returns true if on ladder; mutates mv->m_vecVelocity and movetype",
          "purity": "mutates-member-state",
          "callable_directly": "yes",
          "notes": "RVA 0x11c630 (7 chunks, 2050 bytes). vtable idx 33. CCS override 0x1f6c70 (calls base then extra). confidence HIGH: uniquely references sv_ladder_angle and sv_ladder_dampen, and it is the head of the confirmed ladder block idx33-38."
        },
        {
          "name": "CGameMovement::OnLadder",
          "klass": "CGameMovement",
          "signature": "bool __fastcall(CGameMovement*, trace_t& trace rdx)",
          "inputs": "trace.contents (trace+0x30)",
          "outputs": "returns (trace.contents & CONTENTS_LADDER) != 0",
          "purity": "pure",
          "callable_directly": "yes",
          "notes": "RVA 0x11ce40. vtable idx 34. CCS override 0x1f6cd0 = 'if (trace.plane.normal.z == 1.0f) return false; return base::OnLadder(trace)'. confidence CERTAIN: base tests 'dword [rdx+0x30] & 0x20000000' = CONTENTS_LADDER."
        },
        {
          "name": "CGameMovement::LadderDistance",
          "klass": "CGameMovement",
          "signature": "float __fastcall(CGameMovement*)",
          "inputs": "none",
          "outputs": "returns 2.0f",
          "purity": "pure",
          "callable_directly": "yes",
          "notes": "RVA 0x11c620. vtable idx 35. CCS override 0x1f6c30 returns 10.0f when player+0x1f4 (movetype) == 9 (MOVETYPE_LADDER) else 2.0f. confidence CERTAIN: body is 'movss xmm0,[2.0f]; ret'."
        },
        {
          "name": "CGameMovement::LadderMask",
          "klass": "CGameMovement",
          "signature": "unsigned int __fastcall(CGameMovement*)",
          "inputs": "none",
          "outputs": "returns 0x201400B (MASK_PLAYERSOLID)",
          "purity": "pure",
          "callable_directly": "yes",
          "notes": "RVA 0x9a9f0. vtable idx 36. CCS override 0x17ac20 returns 0x200400B = MASK_PLAYERSOLID & ~CONTENTS_PLAYERCLIP. confidence CERTAIN: 0x201400B is exactly SOLID|WINDOW|GRATE|MOVEABLE|PLAYERCLIP|MONSTER."
        },
        {
          "name": "CGameMovement::ClimbSpeed",
          "klass": "CGameMovement",
          "signature": "float __fastcall(CGameMovement*)",
          "inputs": "none in base",
          "outputs": "returns 200.0f (MAX_CLIMB_SPEED)",
          "purity": "pure",
          "callable_directly": "yes",
          "notes": "RVA 0x118c40. vtable idx 37. CCS override 0x1f5ab0: if (player flags & FL_DUCKING bit at [rax+0x24]&4) return 200.0f * 0.34f else 200.0f. confidence CERTAIN - this is where the 0.34 duck factor (RVA 0x47cb5c) lives."
        },
        {
          "name": "CGameMovement::LadderLateralMultiplier",
          "klass": "CGameMovement",
          "signature": "float __fastcall(CGameMovement*)",
          "inputs": "none in base",
          "outputs": "returns 1.0f",
          "purity": "pure",
          "callable_directly": "yes",
          "notes": "RVA 0x4d150. vtable idx 38. CCS override 0x1f6c50: ducked -> 1.0f, else 0.5f. confidence CERTAIN."
        },
        {
          "name": "CGameMovement::CheckStuck",
          "klass": "CGameMovement",
          "signature": "int __fastcall(CGameMovement*)",
          "inputs": "mv->m_vecAbsOrigin (mv+0x9c); the stuck offset table built by CreateStuckTable; player->m_StuckLast (player+0x1204); TestPlayerPosition (idx51); developer convar",
          "outputs": "returns 0 when free, nonzero when stuck; may nudge mv->m_vecAbsOrigin; writes player+0x1204",
          "purity": "mutates-member-state",
          "callable_directly": "yes",
          "notes": "RVA 0x117f60 (6 chunks, 892 bytes). vtable idx 39. confidence CERTAIN: calls CreateStuckTable (0x119090) then TestPlayerPosition via [vt+0x198]=idx51 with r8 = mv+0x9c (origin) and r9d = 8 (COLLISION_GROUP_PLAYER_MOVEMENT); on INVALID handle (-1) it zeroes player+0x1204 and returns 0 - the SDK body verbatim. Also confirms CMoveData::m_vecAbsOrigin = +0x9C."
        },
        {
          "name": "CGameMovement::CheckWater",
          "klass": "CGameMovement",
          "signature": "bool __fastcall(CGameMovement*)",
          "inputs": "mv->m_vecAbsOrigin; GetPlayerMins/Maxs; GetPointContentsCached (0x11c380); player water level/type",
          "outputs": "sets player water level and type, base velocity current; returns true if in water",
          "purity": "mutates-member-state",
          "callable_directly": "yes",
          "notes": "RVA 0x118540 (601 bytes). vtable idx 40. confidence CERTAIN: FullWalkMove (idx24) opens with 'if (!vt[0x140]()) StartGravity()' i.e. 'if (!CheckWater())'; FullTossMove and FullLadderMove both open by calling it too."
        },
        {
          "name": "CGameMovement::CategorizePosition",
          "klass": "CGameMovement",
          "signature": "void __fastcall(CGameMovement*)",
          "inputs": "mv->m_vecVelocity (z vs NON_JUMP_VELOCITY 140.0f), mv->m_vecAbsOrigin; player water level, movetype; TracePlayerBBox / TryTouchGroundInQuadrants (0x11f040); plane.normal.z vs 0.7",
          "outputs": "player->m_surfaceFriction (player+0x1868) = 1.0f at entry, = 0.25f on the rising-airborne path; SetGroundEntity(); player ground state",
          "purity": "mutates-member-state",
          "callable_directly": "yes",
          "notes": "RVA 0x1174f0 (3 chunks, 608 bytes). vtable idx 41. confidence CERTAIN: 'mov dword [rax+0x1868], 0x3f800000' (=1.0f) at 0x117500 and 'mov dword [rax+0x1868], 0x3e800000' (=0.25f) at 0x11771c - the exact surf-air-friction 0.25 rule in your lab notes - plus the 140.0f NON_JUMP_VELOCITY constant. Calls TryTouchGroundInQuadrants (0x11f040)."
        },
        {
          "name": "CGameMovement::CheckParameters",
          "klass": "CGameMovement",
          "signature": "void __fastcall(CGameMovement*)",
          "inputs": "mv->m_flForwardMove/m_flSideMove/m_flUpMove, m_flMaxSpeed, m_vecAngles; player movetype, flags (FL_FROZEN/FL_ONTRAIN), punch angle; IsDead(); sv_rollangle, sv_rollspeed",
          "outputs": "clamps mv wish-move to m_flMaxSpeed, zeroes it when frozen/dead, sets mv->m_vecAngles incl. ROLL; calls DecayPunchAngle",
          "purity": "mutates-member-state",
          "callable_directly": "yes",
          "notes": "RVA 0x117c10 (7 chunks, 844 bytes). vtable idx 42. confidence CERTAIN (three independent proofs): (a) PlayerMove's first virtual call is [vt+0x150]=idx42, matching 'CheckParameters()' as the first statement; (b) it uniquely references sv_rollangle and sv_rollspeed; (c) the CCS override 0x1f56b0 references the string 'BUG: CCSGameMovement::CheckParameters - too many s' at RVA 0x478390 (lea rcx at 0x1f5a54)."
        },
        {
          "name": "CGameMovement::ReduceTimers",
          "klass": "CGameMovement",
          "signature": "void __fastcall(CGameMovement*)",
          "inputs": "gpGlobals->frametime (via [0x5ae280]+0x10) * 1000.0f; player timer fields (m_flDucktime at ~player+0x1254, duck-jump/swim timers)",
          "outputs": "decrements player millisecond timers",
          "purity": "mutates-member-state",
          "callable_directly": "yes",
          "notes": "RVA 0x11d7f0. vtable idx 43. CCS override 0x1f7600. confidence CERTAIN: multiplies frametime by the f32 1000.0 at RVA 0x3f9608 then subtracts from player timer floats; and PlayerMove calls [vt+0x158]=idx43 immediately after CheckParameters, matching source order."
        },
        {
          "name": "CGameMovement::CheckFalling",
          "klass": "CGameMovement",
          "signature": "void __fastcall(CGameMovement*)",
          "inputs": "player ground entity (call 0x179250), IsDead(), player->m_Local.m_flFallVelocity, m_surfaceFriction",
          "outputs": "fall damage/sound signalling; calls PlayerRoughLandingEffects; resets m_flFallVelocity",
          "purity": "mutates-member-state",
          "callable_directly": "yes",
          "notes": "RVA 0x117750 (8 chunks, 511 bytes). vtable idx 44. confidence MEDIUM-HIGH: position (immediately after ReduceTimers, before PlayerRoughLandingEffects) plus the opening 'player->GetGroundEntity()' call; called from the tail of FullWalkMove."
        },
        {
          "name": "CGameMovement::PlayerRoughLandingEffects",
          "klass": "CGameMovement",
          "signature": "void __fastcall(CGameMovement*, float fvol xmm1)",
          "inputs": "fvol; player surface data/material",
          "outputs": "plays landing sound / viewpunch",
          "purity": "mutates-member-state",
          "callable_directly": "yes - safe to stub out in a TAS sim (no movement effect)",
          "notes": "RVA 0x11d250 (5 chunks, 247 bytes). vtable idx 45. confidence HIGH: takes exactly one float in xmm1 and immediately does 'comiss xmm1, 0; jbe ret' = 'if (fvol > 0.0)', matching the SDK signature and first line."
        },
        {
          "name": "CGameMovement::Duck",
          "klass": "CGameMovement",
          "signature": "void __fastcall(CGameMovement*)",
          "inputs": "mv->m_nButtons/m_nOldButtons (IN_DUCK), mv->m_vecVelocity, player->m_Local.m_flDucktime/m_bDucked/m_bDucking/m_bInDuckJump, ground entity; GAMEMOVEMENT_DUCK_TIME 1000.0f; gpGlobals->frametime",
          "outputs": "player duck flags/timers, view offset, mv origin; calls HandleDuckingSpeedCrop(idx47), FinishUnDuck(idx48), FinishDuck(idx49), CanUnduck(idx50)",
          "purity": "mutates-member-state",
          "callable_directly": "yes",
          "notes": "RVA 0x119730 (7 chunks, 1330 bytes). vtable idx 46. CCS override 0x1f5cb0. confidence CERTAIN: cross-pins to the project's server anchor Duck@server ~0x2eea90 which lies inside server CCS idx 46 (0x2ee520); references the 1000.0f duck-time constant; and it is the only function that virtual-calls idx 47, 48, 49 and 50 - exactly the SDK's Duck() body."
        },
        {
          "name": "CGameMovement::HandleDuckingSpeedCrop",
          "klass": "CGameMovement",
          "signature": "void __fastcall(CGameMovement*)",
          "inputs": "this+0xcc4 (m_bSpeedCropped flag, bit0), player flags FL_DUCKING (player+0x440 bit1), mv->m_flForwardMove/m_flSideMove/m_flUpMove",
          "outputs": "scales mv wish-move by 1/3 (0.33333334f) and sets the speed-cropped flag",
          "purity": "mutates-member-state",
          "callable_directly": "yes",
          "notes": "RVA 0x11c4a0 (116 bytes). vtable idx 47. CCS override 0x1f6ba0 which uses 0.34f (RVA 0x47cb54) instead. confidence CERTAIN: base loads the f32 1/3 at RVA 0x438644 (movss at 0x11c4cd); CCS loads 0.34 three times at 0x1f6be4/0x1f6bf6/0x1f6c0c. Also pins CGameMovement::m_iSpeedCropped to this+0xCC4."
        },
        {
          "name": "CGameMovement::FinishUnDuck",
          "klass": "CGameMovement",
          "signature": "void __fastcall(CGameMovement*)",
          "inputs": "mv->m_vecAbsOrigin, player ground entity, hull sizes via GetPlayerMins/Maxs, GetPlayerViewOffset(false) (idx7)",
          "outputs": "clears FL_DUCKING, m_bDucked/m_bDucking, m_flDucktime; shifts mv->m_vecAbsOrigin by the hull delta; sets view offset",
          "purity": "mutates-member-state",
          "callable_directly": "yes",
          "notes": "RVA 0x11a150 (3 chunks, 991 bytes). vtable idx 48. CCS override 0x1f6840. confidence MEDIUM-HIGH: it is one of the three Duck-called slots {48,49,50}; idx50 provably returns bool (CanUnduck) and idx49 provably calls FixPlayerCrouchStuck+CategorizePosition (FinishDuck), leaving FinishUnDuck for idx48 - which also matches SDK declaration order. Calls GetPlayerViewOffset (idx7). NOTE: unlike the stock SDK it does NOT call CategorizePosition here."
        },
        {
          "name": "CGameMovement::FinishDuck",
          "klass": "CGameMovement",
          "signature": "void __fastcall(CGameMovement*)",
          "inputs": "player flags FL_DUCKING, ground entity; hull deltas; GetPlayerViewOffset(true) (idx7)",
          "outputs": "sets FL_DUCKING and m_bDucked, sets view offset, shifts mv origin, calls FixPlayerCrouchStuck (0x11a840) then CategorizePosition (idx41)",
          "purity": "mutates-member-state",
          "callable_directly": "yes",
          "notes": "RVA 0x119c70 (5 chunks, 1098 bytes). vtable idx 49. CCS override 0x1f6500. confidence HIGH: it is the only one of {48,49,50} that calls both 0x11a840 (which itself calls TestPlayerPosition idx51 = FixPlayerCrouchStuck) and CategorizePosition (idx41) - the exact SDK FinishDuck tail."
        },
        {
          "name": "CGameMovement::CanUnduck",
          "klass": "CGameMovement",
          "signature": "bool __fastcall(CGameMovement*)",
          "inputs": "mv->m_vecAbsOrigin, player ground entity, hull sizes, TracePlayerBBox (idx10) with PlayerSolidMask (idx12)",
          "outputs": "returns bool (true if there is room to stand)",
          "purity": "reads-member-state",
          "callable_directly": "yes",
          "notes": "RVA 0x1170d0 (5 chunks, 1049 bytes). vtable idx 50. CCS override 0x1f4b20. confidence HIGH: only member of {48,49,50} with boolean return sequences (mov al,1 / xor eax,eax) and it virtual-calls idx10 (TracePlayerBBox) + idx12 (PlayerSolidMask), matching the SDK body."
        },
        {
          "name": "CGameMovement::TestPlayerPosition",
          "klass": "CGameMovement",
          "signature": "CBaseHandle __fastcall(CGameMovement*, const Vector& pos rdx, int collisionGroup r8d, trace_t& pm r9)",
          "inputs": "pos, collisionGroup; GetPlayerMins/Maxs; PlayerSolidMask; world geometry",
          "outputs": "fills pm; returns entity handle (-1 / INVALID when free)",
          "purity": "reads-member-state",
          "callable_directly": "yes - useful as a stuck oracle",
          "notes": "RVA 0x11e3a0 (371 bytes, shared CGM/CCS). vtable idx 51. confidence CERTAIN: CheckStuck calls [vt+0x198]=idx51 with the origin, collisionGroup 8, and a trace_t, then compares the returned dword to -1."
        },
        {
          "name": "CGameMovement::SetGroundEntity",
          "klass": "CGameMovement",
          "signature": "void __fastcall(CGameMovement*, trace_t* pm rdx)",
          "inputs": "pm (may be NULL), player ground entity, player/ground base velocity, surface props",
          "outputs": "player ground entity, player base velocity, player->m_surfaceFriction (player+0x1868) via INLINED CategorizeGroundSurface (scaled then clamped to 1.0f), mv->m_vecVelocity.z = 0",
          "purity": "mutates-member-state",
          "callable_directly": "yes",
          "notes": "RVA 0x11d9b0 (568 bytes, shared CGM/CCS). vtable idx 52. confidence HIGH: 'lea rcx,[rdx+0x1868]' at 0x11db20 passes &player->m_surfaceFriction as an out-param to GetPhysicsProperties, then scales it, clamps '>1.0f -> 1.0f' and stores 0x3f800000 - the CategorizeGroundSurface body inlined, which only SetGroundEntity calls."
        },
        {
          "name": "CGameMovement::StepMove",
          "klass": "CGameMovement",
          "signature": "void __fastcall(CGameMovement*, Vector& vecDestination rdx, trace_t& trace r8)",
          "inputs": "vecDestination, trace; mv->m_vecAbsOrigin, mv->m_vecVelocity; player step size (18); DIST_EPSILON 0.03125f; plane.normal.z vs 0.7",
          "outputs": "mv->m_vecAbsOrigin, mv->m_vecVelocity, mv->m_outStepHeight",
          "purity": "mutates-member-state",
          "callable_directly": "yes",
          "notes": "RVA 0x11dfe0 (958 bytes, shared CGM/CCS). vtable idx 53. confidence HIGH: opens with 'movss xmm0,[rdx]' (Vector& first arg), contains the 0.03125 DIST_EPSILON constant and the f64 0.7 walkable test; called by WalkMove."
        },
        {
          "name": "CGameMovement::GameHasLadders",
          "klass": "CGameMovement",
          "signature": "bool __fastcall(CGameMovement*)",
          "inputs": "none",
          "outputs": "returns true",
          "purity": "pure",
          "callable_directly": "yes",
          "notes": "RVA 0x3af00. vtable idx 54 (shared CGM/CCS, last slot of CGameMovement). Body is 'mov al,1; ret'. confidence HIGH."
        },
        {
          "name": "CCSGameMovement::PreventBunnyJumping",
          "klass": "CCSGameMovement",
          "signature": "void __fastcall(CCSGameMovement*)",
          "inputs": "player->m_flMaxspeed (player+0x1618), BUNNYJUMP_MAX_SPEED_FACTOR = 1.1f (RVA 0x403028), mv->m_vecVelocity length",
          "outputs": "scales mv->m_vecVelocity down to 1.1 * m_flMaxspeed when it exceeds that",
          "purity": "mutates-member-state",
          "callable_directly": "yes",
          "notes": "RVA 0x1f7500. CCS-ONLY vtable idx 55 (the extra slot CCSGameMovement adds beyond CGameMovement's 55). confidence CERTAIN: 'xmm3 = player[0x1618] * 1.1f; if (xmm3 <= 0) return;' is the SDK PreventBunnyJumping preamble exactly. Directly relevant to bhop TAS parity."
        },
        {
          "name": "CGameMovement::CheckVelocity",
          "klass": "CGameMovement",
          "signature": "void __fastcall(CGameMovement*)",
          "inputs": "mv->m_vecVelocity, mv->m_vecAbsOrigin; sv_maxvelocity",
          "outputs": "NaN-scrubs and clamps each velocity component to +/- sv_maxvelocity",
          "purity": "mutates-member-state",
          "callable_directly": "yes",
          "notes": "RVA 0x1182e0 (600 bytes, NON-VIRTUAL, 4 callers). confidence CERTAIN: it is the only movement helper referencing sv_maxvelocity, and it is the tail call of the inlined StartGravity in FullWalkMove and of FinishGravity (0x11a0c0)."
        },
        {
          "name": "CGameMovement::FinishGravity",
          "klass": "CGameMovement",
          "signature": "void __fastcall(CGameMovement*)",
          "inputs": "player->m_flWaterJumpTime (player+0x1684), player->m_flGravity (player+0x1dc), GetCurrentGravity() (0x15f7a0), gpGlobals->frametime, mv->m_vecVelocity.z (mv+0x4c)",
          "outputs": "mv->m_vecVelocity[2] -= ent_gravity * gravity * frametime * 0.5; then CheckVelocity()",
          "purity": "mutates-member-state",
          "callable_directly": "yes",
          "notes": "RVA 0x11a0c0 (131 bytes, NON-VIRTUAL, 3 callers incl. FullWalkMove and CheckJumpButton). confidence CERTAIN: exact SDK body including the early-out on m_flWaterJumpTime and the ent_gravity==0 -> 1.0 substitution."
        },
        {
          "name": "CGameMovement::StartGravity (INLINED)",
          "klass": "CGameMovement",
          "signature": "inlined into FullWalkMove",
          "inputs": "player->m_flGravity (+0x1dc), GetCurrentGravity() (0x15f7a0), gpGlobals->frametime, player->m_vecBaseVelocity (+0x1d0)",
          "outputs": "mv->m_vecVelocity[2] -= grav*0.5*frametime; += basevel.z*frametime; zeroes basevel.z; CheckVelocity()",
          "purity": "mutates-member-state",
          "callable_directly": "no - no standalone function exists",
          "notes": "Inlined at RVA 0x11ba89-0x11bb5b inside FullWalkMove (0x11ba70). confidence CERTAIN (decoded instruction-by-instruction). There is NO separate StartGravity symbol in client.dll x64."
        },
        {
          "name": "CGameMovement::Friction (INLINED)",
          "klass": "CGameMovement",
          "signature": "inlined into FullWalkMove",
          "inputs": "mv->m_vecVelocity, player->m_surfaceFriction (player+0x1868), sv_friction (object 0x6799c0, parent at +0x38, value at parent+0x54), sv_stopspeed (object 0x6791e0), gpGlobals->frametime",
          "outputs": "scales mv->m_vecVelocity",
          "purity": "mutates-member-state",
          "callable_directly": "no - no standalone function exists",
          "notes": "Inlined in FullWalkMove (0x11ba70); m_surfaceFriction read at 0x11bd55. The two sites named in the project's notes (0x11adb4, 0x11fb81) are the SEPARATE friction-shaped code inside FullNoClipMove and WaterMove, not CGameMovement::Friction. confidence HIGH."
        },
        {
          "name": "CGameMovement::PerformFlyCollisionResolution (INLINED)",
          "klass": "CGameMovement",
          "signature": "inlined into FullTossMove",
          "inputs": "trace_t pm, move vector, player move-collide mode, player->m_surfaceFriction (+0x1868), plane.normal.z vs 0.7",
          "outputs": "mv->m_vecVelocity via ClipVelocity with backoff = 2.0 - m_surfaceFriction",
          "purity": "mutates-member-state",
          "callable_directly": "no",
          "notes": "Inlined in FullTossMove (0x11b310); 'subss xmm0,[r8+0x1868]' at 0x11b8f3 is the '2.0 - m_surfaceFriction' backoff. confidence HIGH."
        },
        {
          "name": "CGameMovement::CategorizeGroundSurface (INLINED)",
          "klass": "CGameMovement",
          "signature": "inlined into SetGroundEntity",
          "inputs": "trace_t.surface.surfaceProps; IPhysicsSurfaceProps",
          "outputs": "player->m_surfaceProps, m_pSurfaceData, m_surfaceFriction (player+0x1868) scaled then clamped to 1.0f, m_chTextureType",
          "purity": "mutates-member-state",
          "callable_directly": "no",
          "notes": "Inlined in SetGroundEntity (0x11d9b0) at 0x11db20-0x11db6a. confidence HIGH."
        },
        {
          "name": "CGameMovement::ClipVelocity",
          "klass": "CGameMovement",
          "signature": "int __fastcall(CGameMovement*, Vector& in, Vector& normal, Vector& out, float overbounce)",
          "inputs": "in, normal, out, overbounce",
          "outputs": "writes out; returns blocked bits (1=floor if normal.z>0.7, 2=wall)",
          "purity": "pure",
          "callable_directly": "yes",
          "notes": "RVA 0x118c50 (404 bytes, NON-VIRTUAL, 2 callers: TryPlayerMove 0x11e660 and FullTossMove 0x11b310). confidence HIGH: exactly the two SDK callers of ClipVelocity, and no other candidate in the region has that caller set."
        },
        {
          "name": "CGameMovement::PushEntity",
          "klass": "CGameMovement",
          "signature": "void __fastcall(CGameMovement*, Vector& push rdx, trace_t* pTrace r8)",
          "inputs": "push delta, mv->m_vecAbsOrigin, TracePlayerBBox",
          "outputs": "mv->m_vecAbsOrigin = trace endpos; fills *pTrace",
          "purity": "mutates-member-state",
          "callable_directly": "yes",
          "notes": "RVA 0x11d700 (226 bytes, NON-VIRTUAL, single caller = FullTossMove). confidence MEDIUM-HIGH: FullTossMove is the only SDK caller of PushEntity, and this is FullTossMove's only unaccounted direct callee."
        },
        {
          "name": "CGameMovement::StayOnGround",
          "klass": "CGameMovement",
          "signature": "void __fastcall(CGameMovement*)",
          "inputs": "mv->m_vecAbsOrigin, player step size, TracePlayerBBox, plane.normal.z >= 0.7",
          "outputs": "may snap mv->m_vecAbsOrigin down to the ground",
          "purity": "mutates-member-state",
          "callable_directly": "yes",
          "notes": "RVA 0x11de60 (383 bytes, NON-VIRTUAL, single caller = WalkMove). confidence HIGH: contains both the '+2' start offset (f32 2.0) and the f64 0.7 slope test, and WalkMove is its only caller - matching the SDK exactly."
        },
        {
          "name": "CGameMovement::TryTouchGroundInQuadrants",
          "klass": "CGameMovement",
          "signature": "void __fastcall(CGameMovement*, ... , trace_t& pm)",
          "inputs": "bump origin, end point, mask, collision group; plane.normal.z vs 0.7",
          "outputs": "fills pm with the best quadrant ground hit",
          "purity": "reads-member-state",
          "callable_directly": "yes",
          "notes": "RVA 0x11f040 (743 bytes, NON-VIRTUAL, single caller = CategorizePosition). confidence MEDIUM-HIGH: CategorizePosition's only direct (non-virtual) callee, carries the f64 0.7 walkable constant - matches the SDK's TryTouchGroundInQuadrants."
        },
        {
          "name": "CGameMovement::FullNoClipMove",
          "klass": "CGameMovement",
          "signature": "void __fastcall(CGameMovement*, float factor xmm1, float maxacceleration xmm2)",
          "inputs": "factor (sv_noclipspeed), maxacceleration (sv_noclipaccelerate), sv_maxspeed, sv_friction, player->m_surfaceFriction (+0x1868), mv wish-move/angles, gpGlobals->frametime",
          "outputs": "mv->m_vecVelocity and mv->m_vecAbsOrigin",
          "purity": "mutates-member-state",
          "callable_directly": "yes",
          "notes": "RVA 0x11aa80 (1070 bytes, NON-VIRTUAL). confidence CERTAIN: PlayerMove's jump table case MOVETYPE_NOCLIP(8) loads [sv_noclipspeed+0x38]->parent->+0x54 into xmm1 and [sv_noclipaccelerate+0x38] into xmm2 then 'jmp 0x11aa80'. Contains the 0.25 (maxspeed/4) friction-control constant; the m_surfaceFriction site at 0x11adb4 lives here."
        },
        {
          "name": "CGameMovement::FullObserverMove",
          "klass": "CGameMovement",
          "signature": "void __fastcall(CGameMovement*)",
          "inputs": "player observer mode/target, sv_specspeed, sv_specaccelerate, sv_specnoclip, sv_friction, sv_maxvelocity, mv wish-move",
          "outputs": "mv->m_vecVelocity, mv->m_vecAbsOrigin",
          "purity": "mutates-member-state",
          "callable_directly": "yes",
          "notes": "RVA 0x11aeb0 (7 chunks, 1106 bytes, NON-VIRTUAL). confidence CERTAIN: PlayerMove jump table case MOVETYPE_OBSERVER(10) tail-jumps here; uniquely references sv_specspeed/sv_specaccelerate/sv_specnoclip."
        },
        {
          "name": "CGameMovement::CreateStuckTable",
          "klass": "CGameMovement",
          "signature": "void __fastcall(void)",
          "inputs": "none (one-shot static init guarded by a bool)",
          "outputs": "fills the global rgv3tStuckTable offset array",
          "purity": "mutates-member-state",
          "callable_directly": "yes (idempotent)",
          "notes": "RVA 0x119090 (4 chunks, 758 bytes). confidence HIGH: called as the very first thing by CheckStuck (0x117f60) and contains the 0.25 / 2.0 / 6.0 offset constants of the SDK table builder."
        },
        {
          "name": "CGameMovement::FixPlayerCrouchStuck",
          "klass": "CGameMovement",
          "signature": "void __fastcall(CGameMovement*, bool moveup)",
          "inputs": "mv->m_vecAbsOrigin; TestPlayerPosition (idx51)",
          "outputs": "nudges mv->m_vecAbsOrigin until unstuck",
          "purity": "mutates-member-state",
          "callable_directly": "yes",
          "notes": "RVA 0x11a840 (3 chunks, 363 bytes, NON-VIRTUAL, 3 callers). confidence HIGH: it is the only helper that virtual-calls idx51 TestPlayerPosition and it is called from FinishDuck (idx49) - the SDK's only FixPlayerCrouchStuck caller pattern."
        },
        {
          "name": "CGameMovement::SetDuckedEyeOffset",
          "klass": "CGameMovement",
          "signature": "void __fastcall(CGameMovement*, float duckFraction xmm1)",
          "inputs": "duckFraction; GetPlayerMins(true/false) (idx5), GetPlayerViewOffset(true/false) (idx7)",
          "outputs": "player view offset interpolated between stand and duck",
          "purity": "mutates-member-state",
          "callable_directly": "yes",
          "notes": "RVA 0x11d8d0 (210 bytes, NON-VIRTUAL, 3 callers). confidence CERTAIN: calls vtable+0x28 (idx5) with ducked=true then ducked=false, subtracts the .z fields, then calls vtable+0x38 (idx7) with true/false - the SDK body verbatim. This is what pinned idx5 and idx7."
        },
        {
          "name": "CGameMovement::IsDead",
          "klass": "CGameMovement",
          "signature": "bool __fastcall(CGameMovement*)",
          "inputs": "player->m_iHealth (player+0xd0), player->IsAlive() (player vtable +0x418)",
          "outputs": "returns (m_iHealth <= 0 && !IsAlive())",
          "purity": "reads-member-state",
          "callable_directly": "yes",
          "notes": "RVA 0x11c5f0 (44 bytes, NON-VIRTUAL, 3 callers). confidence HIGH: exact two-term body; used by CheckParameters and Duck."
        },
        {
          "name": "CGameMovement::GetPointContentsCached",
          "klass": "CGameMovement",
          "signature": "int __fastcall(CGameMovement*, const Vector& point rdx, int slot r8d)",
          "inputs": "point, slot, a global prediction/cache-enable bool at RVA 0x5b4618, cached contents in the this-object",
          "outputs": "returns contents mask; updates the cache slot",
          "purity": "mutates-member-state",
          "callable_directly": "yes",
          "notes": "RVA 0x11c380 (277 bytes, NON-VIRTUAL, single caller = CheckWater idx40). confidence MEDIUM-HIGH: signature (Vector*, int slot) matches ('mov rsi,rdx; movsxd rdi,r8d') and CheckWater is the SDK's caller of GetPointContentsCached."
        },
        {
          "name": "CGameMovement::CanUnDuckJump",
          "klass": "CGameMovement",
          "signature": "bool __fastcall(CGameMovement*, trace_t& trace rdx)",
          "inputs": "mv->m_vecAbsOrigin; TracePlayerBBox (idx10) with PlayerSolidMask (idx12)",
          "outputs": "fills trace; returns bool",
          "purity": "reads-member-state",
          "callable_directly": "yes",
          "notes": "RVA 0x116fc0 (271 bytes, NON-VIRTUAL, single caller = Duck). confidence MEDIUM: returns bool, does a TracePlayerBBox+PlayerSolidMask, and is one of Duck's private helpers - matches CanUnDuckJump. Name not independently proven."
        },
        {
          "name": "CGameMovement::FinishUnDuckJump",
          "klass": "CGameMovement",
          "signature": "void __fastcall(CGameMovement*, trace_t& trace rdx)",
          "inputs": "trace; player duck/jump timers and flags; GetPlayerViewOffset(false) (idx7)",
          "outputs": "clears duck+duck-jump state, sets view offset, sets mv->m_vecAbsOrigin, then CategorizePosition (idx41)",
          "purity": "mutates-member-state",
          "callable_directly": "yes",
          "notes": "RVA 0x11a530 (773 bytes, NON-VIRTUAL, single caller = Duck). confidence MEDIUM: it is the only Duck helper that calls both idx7 (GetPlayerViewOffset) and idx41 (CategorizePosition), which is the distinguishing tail of FinishUnDuckJump in the SDK."
        },
        {
          "name": "unknown_0011dbf0 (Duck helper; UpdateDuckJumpEyeOffset or StartUnDuckJump)",
          "klass": "CGameMovement",
          "signature": "void __fastcall(CGameMovement*, ...)",
          "inputs": "player duck-jump timers; GetPlayerViewOffset (idx7)",
          "outputs": "player view offset / duck-jump state",
          "purity": "mutates-member-state",
          "callable_directly": "yes",
          "notes": "RVA 0x11dbf0 (617 bytes, NON-VIRTUAL, single caller = Duck). confidence LOW on name - proven only that it is a Duck-private helper calling idx7 and no CategorizePosition."
        },
        {
          "name": "unknown_00118df0",
          "klass": "CGameMovement",
          "signature": "unknown",
          "inputs": "unknown",
          "outputs": "unknown",
          "purity": "unknown",
          "callable_directly": "unknown",
          "notes": "RVA 0x118df0 (3 chunks, 671 bytes, 2 callers) - in the gamemovement.cpp code block but not identified. Listed for completeness."
        },
        {
          "name": "unknown_0011c520",
          "klass": "CGameMovement",
          "signature": "unknown",
          "inputs": "unknown",
          "outputs": "unknown",
          "purity": "unknown",
          "callable_directly": "unknown",
          "notes": "RVA 0x11c520 (204 bytes, 1 caller). Unidentified helper in the gamemovement block."
        },
        {
          "name": "CGameMovement vtable (data, not a function)",
          "klass": "other",
          "signature": "const void* vtable[55] @ client.dll RVA 0x438348",
          "inputs": "n/a",
          "outputs": "n/a",
          "purity": "pure",
          "callable_directly": "n/a",
          "notes": "CGameMovement vtable RVA 0x438348, 55 entries. RTTI CompleteObjectLocator at RVA 0x53d948 (pointer stored at 0x438340), TypeDescriptor '.?AVCGameMovement@@' at RVA 0x5df628. IGameMovement vtable at RVA 0x438300 (8 entries: dtor + 7 pure), COL 0x53da18, TypeDescriptor 0x5df650. Server-side equivalents (server.dll x64): CGameMovement vtable 0x4e4548, IGameMovement 0x4e4500, CCSGameMovement 0x56f8e0."
        },
        {
          "name": "CCSGameMovement vtable (data, not a function)",
          "klass": "other",
          "signature": "const void* vtable[56] @ client.dll RVA 0x4781c8",
          "inputs": "n/a",
          "outputs": "n/a",
          "purity": "pure",
          "callable_directly": "n/a",
          "notes": "CCSGameMovement vtable RVA 0x4781c8, 56 entries (adds PreventBunnyJumping at idx 55). COL RVA 0x54b090, TypeDescriptor '.?AVCCSGameMovement@@' at RVA 0x5e34c8. Overridden slots: 0,1,11,12,13,15,19,21,23,25,26,29,33,34,35,36,37,38,42,43,46,47,48,49,50,55. Interface string 'GameMovement001' at RVA 0x478168."
        },
        {
          "name": "LAYOUT NOTES (offsets proven during this pass)",
          "klass": "other",
          "signature": "n/a",
          "inputs": "n/a",
          "outputs": "n/a",
          "purity": "pure",
          "callable_directly": "n/a",
          "notes": "CGameMovement: player = this+0x08, mv = this+0x10, m_iSpeedCropped = this+0xCC4. CCSGameMovement: CS player cached at this+0xEE0. CMoveData: m_nButtons +0x24, m_nOldButtons +0x28, m_flForwardMove +0x2C, m_flSideMove +0x34, m_flUpMove +0x38, m_flMaxSpeed +0x3C, m_flClientMaxSpeed +0x40, m_vecVelocity +0x44 (z at +0x4C), m_vecViewAngles +0x0C, m_vecAbsOrigin +0x9C (matches project notes). CBasePlayer: m_iHealth +0xD0, m_flModelScale +0x8E4, m_vecBaseVelocity +0x1D0, m_flGravity +0x1DC, m_MoveType +0x1F4, m_StuckLast +0x1204, m_flDucktime ~+0x1254, pl.deadflag +0x1560, m_flMaxspeed +0x1618, m_flWaterJumpTime +0x1684, m_surfaceFriction +0x1868. ConVar (x64): object stride 0x90, m_pParent at +0x38, parent m_fValue at +0x54. Helper: GetCurrentGravity() = RVA 0x15f7a0; gpGlobals pointer at RVA 0x5ae280 with frametime at +0x10."
        },
        {
          "name": "CONVAR OBJECT TABLE (gamemovement.cpp block, client.dll)",
          "klass": "other",
          "signature": "n/a",
          "inputs": "n/a",
          "outputs": "n/a",
          "purity": "pure",
          "callable_directly": "n/a",
          "notes": "Verified in declaration order at 0x90 stride: sv_gravity 0x679150, sv_stopspeed 0x6791E0, sv_noclipaccelerate 0x679270, sv_noclipspeed 0x679300, sv_specaccelerate 0x679390, sv_specspeed 0x679420, sv_specnoclip 0x6794B0, sv_maxspeed 0x679540, sv_accelerate 0x6795D0, sv_airaccelerate 0x679660, sv_wateraccelerate 0x6796F0, sv_waterfriction 0x679780, sv_footsteps 0x679810, sv_rollspeed 0x6798A0, sv_rollangle 0x679930, sv_friction 0x6799C0, sv_bounce 0x679A50, sv_maxvelocity 0x679AE0, sv_stepsize 0x679B70, sv_backspeed 0x679C00, sv_waterdist 0x679C90, sv_skyname 0x679D20. Read a value with: parent = *(void**)(obj+0x38); value = *(float*)(parent+0x54). GROUND TRUTH CHECK: PlayerMove's MOVETYPE_NOCLIP case loads exactly [0x679338] (=sv_noclipspeed+0x38) and [0x6792A8] (=sv_noclipaccelerate+0x38)."
        },
        {
          "name": "KEY FLOAT CONSTANT ADDRESSES",
          "klass": "other",
          "signature": "n/a",
          "inputs": "n/a",
          "outputs": "n/a",
          "purity": "pure",
          "callable_directly": "n/a",
          "notes": "client.dll RVAs: f32 30.0 (air speed cap) 0x40EC18; f32 268.3281573 = sqrt(2*800*45) base jump 0x438678; f32 1/3 duck speed crop 0x438644; f64 301.99337741082996 = sqrt(2*800*57) CS:S jump impulse 0x4783E0; f32 0.00019 stamina 0x4783D8; f32 0.34 duck factor 0x47CB54 and 0x47CB5C; f64 0.7 walkable normal 0x41CE50; f64 0.001 punch-decay threshold 0x3F5F58; f32 1000.0 duck time 0x3F9608; f32 0.5 0x3F64B8; f32 3.0 0x3F64D0; f32 1.0 0x3E7FF8; f32 60.0 0x3E7FFC; f32 1.1 bunnyhop factor 0x403028; f32 200.0 MAX_CLIMB_SPEED 0x403060. Note m_surfaceFriction 0.25 is written as an immediate (0x3E800000) not a pool constant, at 0x11771C inside CategorizePosition."
        }
      ],
      "sources": [
        "C:\\Program Files (x86)\\Steam\\steamapps\\common\\Counter-Strike Source\\cstrike\\bin\\x64\\client.dll (primary target; all RVAs above are client.dll unless stated)",
        "C:\\Program Files (x86)\\Steam\\steamapps\\common\\Counter-Strike Source\\cstrike\\bin\\x64\\server.dll (used only to cross-pin vtable indices against the project's existing server anchors WalkMove@0x2f0140=idx23, CheckJumpButton@0x2eddab in idx29, Duck@0x2eea90 in idx46)",
        "MSVC x64 RTTI: TypeDescriptors .?AVCGameMovement@@ @0x5df628, .?AVIGameMovement@@ @0x5df650, .?AVCCSGameMovement@@ @0x5e34c8; CompleteObjectLocators @0x53d948 / 0x53da18 / 0x54b090",
        "PE .pdata RUNTIME_FUNCTION table with UNW_FLAG_CHAININFO resolution (20975 raw entries merged into 12136 functions) - this is what makes function extents reliable despite MSVC hot/cold splitting",
        "String evidence: 'BUG: CCSGameMovement::CheckParameters - too many s' @0x478390, referenced by lea rcx at 0x1f5a54 inside 0x1f56b0; 'GameMovement001' @0x478168",
        "Scratchpad scripts written this session (all under C:\\Users\\Connor\\AppData\\Local\\Temp\\claude\\C--Users-Connor-Documents-SourceTAS-sourceTAS\\f654ec71-3563-47ff-a5ed-3d9fef29e216\\scratchpad): pe.py (shared PE parser), funcs2.py (.pdata chain-merged function table), rtti_find.py, vt_find.py, vt_dump.py, vtprobe.py, srv_vt.py, convars3.py (verified ConVar object map), analyze.py + analysis.pkl (per-function constant/convar/callgraph index), show.py, cg.py, xref.py",
        "Reused from prior work: re_disasm.py (windowed capstone disassembly with rip-relative constant resolution)"
      ]
    },
    "ours": {
      "functions": [
        {
          "name": "EngineClipVelocity",
          "klass": "CGameMovement",
          "signature": "void EngineClipVelocity(const Vec3& in, const Vec3& n, Vec3& out)",
          "inputs": "Source/Solver/SolverMove.cpp:10. Params only: in (velocity), n (plane normal). Reads NO PlayerState/MoveParams/World field. Overbounce is hardcoded 1.0 (not a param).",
          "outputs": "out (Vec3). Writes nothing else.",
          "purity": "pure",
          "callable_directly": "no - it lives in the anonymous namespace opened at SolverMove.cpp:5 and closed at :578; only MoveTick is declared in SolverMove.h",
          "notes": "Mirrors CGameMovement::ClipVelocity with overbounce=1. Derived from the month-validated DLL fast model (comment :7-9 'Verbatim from the validated DLL model'), which is SDK-shaped and measurement-arbitrated. GAP vs SDK: the engine's ClipVelocity RETURNS a `blocked` bitmask (1=floor, 2=step/wall) that TryPlayerMove uses; we discard it. No STOP_EPSILON snap (correct for Source-era SDK, which also lacks it)."
        },
        {
          "name": "CheckVelocity",
          "klass": "CGameMovement",
          "signature": "void CheckVelocity(PlayerState& s, const MoveParams& p)",
          "inputs": "Source/Solver/SolverMove.cpp:18. Reads s.vel.X/.Y/.Z and p.maxvelocity (sv_maxvelocity, default 3500).",
          "outputs": "Writes s.vel.X/.Y/.Z (per-component symmetric clamp to +/-maxvelocity).",
          "purity": "mutates-member-state",
          "callable_directly": "no - anonymous namespace (SolverMove.cpp:5-578)",
          "notes": "Mirrors CGameMovement::CheckVelocity. SDK-recollection for structure; the CALL SITES are measurement-driven - a third call was added after FinishGravity (SolverMove.cpp:659-662) because fuzz probe 818 showed the engine reporting exactly -3500 where we reported -3506. GAPS vs SDK: no IS_NAN check-and-zero on velocity, and no clamp of ORIGIN components to MAX_COORD_INTEGER."
        },
        {
          "name": "WishFromInput",
          "klass": "CGameMovement",
          "signature": "void WishFromInput(float yaw, float fmove, float smove, float* wx, float* wy)",
          "inputs": "Source/Solver/SolverMove.cpp:32. Params only: yaw (degrees), fmove, smove. No state reads.",
          "outputs": "*wx, *wy (the flattened wish velocity XY).",
          "purity": "pure",
          "callable_directly": "no - anonymous namespace",
          "notes": "Mirrors AngleVectors(forward,right) + wishvel = forward*fmove + right*smove with z zeroed and re-normalized, as done inside CGameMovement::WalkMove and ::AirMove. SDK-recollection, validated by every parity deck. Convention pinned in the comment: forward=(cos yaw, sin yaw), right=(sin yaw, -cos yaw). Pitch is deliberately excluded (the engine zeroes z then normalizes, so a yaw-only basis is equivalent) - which is why MoveTick's `pitch` param is unused."
        },
        {
          "name": "TryPlayerMove",
          "klass": "CGameMovement",
          "signature": "void TryPlayerMove(PlayerState& s, const World& w, const MoveParams& p, TickEvents* ev)",
          "inputs": "Source/Solver/SolverMove.cpp:45. Reads s.vel, s.pos, s.hull_state, s.on_ground (gates the single-plane reflect path at :118), p.dt, and the World collision set via w.TraceHull3 (which reads World::brushes[].n/.pid/.d_stand|d_duck|d_unduck/.gmin_*|gmax_*, the uniform grid, and World::true_interval_corner).",
          "outputs": "Writes s.pos, s.vel. Writes ev->contact_brush[], ev->contact_plane[], ev->contact_loss[], ev->ncontacts. Does NOT write s.on_ground.",
          "purity": "mutates-member-state",
          "callable_directly": "no - anonymous namespace",
          "notes": "Mirrors CGameMovement::TryPlayerMove (4 bumps, MAX_CLIP_PLANES=5, crease resolution, all_fraction==0 zero). MEASURED/validated - the header comment (:42-44) explicitly says 'do NOT fix it toward SDK recollection without parity evidence'. Three measured behaviors live here: (a) allsolid -> velocity ZEROED = the surf ramp bug, seen 18 straight ticks in the d34 tape and in 32.7% of fuzz probes (:59-74); (b) the frac>=1 unswept re-test 'Player will become stuck!!!' guard (:77-93, d34 t193); (c) the single-plane reflect path is gated on AIRBORNE (:114-118) so grounded first impacts take the stop-dead guard. GAPS: no `blocked` return, no entity traces (world brushes only), no m_outStepHeight/m_outWishVel bookkeeping."
        },
        {
          "name": "CategorizePosition",
          "klass": "CGameMovement",
          "signature": "void CategorizePosition(PlayerState& s, const World& w, const MoveParams& p, TickEvents* ev)",
          "inputs": "Source/Solver/SolverMove.cpp:162. Reads s.on_ground (prior value, for the landed/left_ground edge), s.vel.Z, s.pos, s.hull_state, s.ducked; p.non_jump_velocity (140), p.walkable_z (0.7), p.air_friction_up (0.25); World via a 2-unit down TraceHull3.",
          "outputs": "Writes s.on_ground, s.ground_brush, s.surface_friction, s.vel.Z (zeroed whenever NEW ground is set, :195), s.hull_state (re-based to the duck FLAG on grounding, :198). Writes ev->landed, ev->left_ground.",
          "purity": "mutates-member-state",
          "callable_directly": "no - anonymous namespace",
          "notes": "Mirrors CGameMovement::CategorizePosition + SetGroundEntity. MEASURED throughout: (a) NO origin snap on grounding - engine ground truth 2026-08-13 t479 set ground with the origin 1.575u above the surface (:180-184, 'do not restore it'); (b) m_surfaceFriction=0.25 when no walkable plane is found while RISING - fuzz-measured 2026-08-15, pinned exactly by a ducked rising tick applying 47.8125 = 150*85*0.015*0.25 (:169-178); (c) the SetGroundEntity vz=0 is load-bearing at spine495 t537. GAPS: no water level, no ladder, no ground-entity tracking (only a brush index), no surfaceprops-derived friction (only 1.0 / 0.25), no standable-entity / moving-platform ground."
        },
        {
          "name": "Friction",
          "klass": "CGameMovement",
          "signature": "void Friction(PlayerState& s, const MoveParams& p)",
          "inputs": "Source/Solver/SolverMove.cpp:209. Reads s.vel (speed), s.surface_friction; p.stopspeed (75), p.friction (4), p.dt.",
          "outputs": "Writes s.vel (uniformly scaled by newspeed/speed).",
          "purity": "mutates-member-state",
          "callable_directly": "no - anonymous namespace",
          "notes": "Mirrors CGameMovement::Friction. MEASURED constants: sv_friction 4 and sv_stopspeed 75 both CONFIRMED from an engine ground-truth curve fit 2026-08-13 (MoveParams comments SolverMove.h:32-35) - note stopspeed 75, NOT Source's stock 100. The `* s.surface_friction` factor is BINARY-DECODED (server.dll Friction @0020edac reads player+0x36C and multiplies by sv_friction). The grounded gate lives at the call site (MoveTick :645-648), not inside. GAP: no water friction branch."
        },
        {
          "name": "CheckJumpButton",
          "klass": "CCSGameMovement",
          "signature": "void CheckJumpButton(PlayerState& s, const World&, const MoveParams& p, TickEvents* ev)",
          "inputs": "Source/Solver/SolverMove.cpp:223. Reads s.on_ground, s.old_buttons (IN_JUMP release gate), s.vel (whole vector for the bunnyhop clamp, vel.Z for the impulse), s.ducked, s.ducking, s.stamina, s.gravity_scale; p.enablebunnyhopping, p.maxspeed, p.jump_impulse_d, p.stamina_scale_per_ms, p.stamina_jump_ms, p.jump_finishgravity, p.gravity, p.dt. World param is UNUSED.",
          "outputs": "Writes s.on_ground=false, s.ground_brush=-1, s.vel (clamped if bunnyhop prevention on), s.vel.Z, s.stamina, s.old_buttons |= IN_JUMP. Writes ev->jumped, ev->left_ground.",
          "purity": "mutates-member-state",
          "callable_directly": "no - anonymous namespace",
          "notes": "Mirrors CCSGameMovement::CheckJumpButton + PreventBunnyJumping. BINARY-DECODED from the shipped x64 server.dll 2026-08-15 (capstone): impulse is a baked DOUBLE 301.99337741082996 at @2eddab that does NOT track sv_gravity, added in double with one rounding to float; ducked/ducking SETS vz instead of adding (@2eddc1 skips the add); the stamina tax scales the WHOLE post-impulse vz by (1 - stamina*0.00019f) at @2eddfc; the arm stores raw bits 0x44a47943 == 25000/19 at @2ede47. `jump_finishgravity` (the SDK's in-jump FinishGravity call) is kept as an arbitration toggle, default true, confirmed by the -6/-6 tail. GAPS: no water jump, no ladder detach, no jump sound / MoveHelper, no m_bInDuckJump."
        },
        {
          "name": "StepMove",
          "klass": "CGameMovement",
          "signature": "void StepMove(PlayerState& s, const World& w, const MoveParams& p, TickEvents* ev)",
          "inputs": "Source/Solver/SolverMove.cpp:283. Reads s.pos, s.vel, s.hull_state; p.stepsize (18), p.walkable_z, p.dt (via TryPlayerMove); World traces. Local const kDistEpsilon = 0.03125f (engine DIST_EPSILON).",
          "outputs": "Writes s.pos, s.vel. Records contacts through the FIRST (down) TryPlayerMove only; the step-up TryPlayerMove is called with ev=nullptr to keep bookkeeping single-counted (:306).",
          "purity": "mutates-member-state",
          "callable_directly": "no - anonymous namespace",
          "notes": "Mirrors CGameMovement::StepMove 1:1. SDK-RECOLLECTION for structure, ARBITRATED by capture: the previous 'slides only' walk path owned the tick-537 spine divergence that broke the 495 tape in-game (user directive 2026-08-14). Semantics documented verbatim at :270-282. Note the non-walkable-plane test at :314 uses dn.normal.Z only, so a fraction-1 down trace (normal left zeroed) also fails - stated as the engine's own behavior. GAPS: no m_bAllowAutoMovement, no m_outStepHeight, no reduced step height while ducked, no water."
        },
        {
          "name": "StayOnGround",
          "klass": "CGameMovement",
          "signature": "void StayOnGround(PlayerState& s, const World& w, const MoveParams& p)",
          "inputs": "Source/Solver/SolverMove.cpp:335. Reads s.pos, s.hull_state; p.stepsize, p.walkable_z; World traces (a +2u up trace, then a -stepsize down trace from the partial up endpoint).",
          "outputs": "Writes s.pos only (snapped to the down-trace landing when |dz| > 0.015625, half COORD_RESOLUTION).",
          "purity": "mutates-member-state",
          "callable_directly": "no - anonymous namespace",
          "notes": "Mirrors CGameMovement::StayOnGround. SDK-RECOLLECTION, validated by the ground decks. DIVERGENCE vs SDK: the SDK guard is `fraction > 0 && fraction < 1 && !startsolid && normal.z >= 0.7`; ours (:346-347) omits the !startsolid term and substitutes `dn.brush >= 0`."
        },
        {
          "name": "WalkMove",
          "klass": "CCSGameMovement",
          "signature": "void WalkMove(PlayerState& s, const World& w, const MoveParams& p, float yaw, float fmove, float smove, bool cap_ducked, TickEvents* ev)",
          "inputs": "Source/Solver/SolverMove.cpp:354. Params yaw, fmove, smove, cap_ducked. Reads s.stamina, s.vel, s.pos, s.hull_state, s.surface_friction, s.basevel; p.stamina_scale_per_ms, p.stamina_pow_rate (70), p.dt, p.maxspeed, p.duck_speed_frac (0.34), p.accelerate (10), p.stepsize/p.walkable_z via StepMove/StayOnGround; World traces.",
          "outputs": "Writes s.vel (stamina drag, accelerate, vel.Z=0 twice, basevel add/subtract), s.pos (direct dest on a clear trace, else via StepMove, then StayOnGround). Writes ev contacts via StepMove.",
          "purity": "mutates-member-state",
          "callable_directly": "no - anonymous namespace",
          "notes": "Mirrors CCSGameMovement::WalkMove with CGameMovement::Accelerate inlined at :389-397. BINARY-DECODED stamina drag (server.dll @2f0160): vel.xy *= powf(1 - stamina*0.00019f, frametime*70.0f), applied AFTER Friction and BEFORE accelerate - this retired the old affine fit (slope 0.00019833 + intercept 0.000327), which was a chord across the power curve. MEASURED duck cap: the ducked wishspeed cap applies from the PRESS tick (2026-08-13 t204) and also on the UNDUCK tick (2026-08-14 unduck_face t122, fitted to 0.1 u/s) - hence cap_ducked is computed by the caller. Accelerate scales by s.surface_friction. basevel rides through the move only (:407/:417). GAPS: no water, no m_outWishVel, no per-tick weapon m_flMaxSpeed refresh."
        },
        {
          "name": "AirMove",
          "klass": "CGameMovement",
          "signature": "void AirMove(PlayerState& s, const World& w, const MoveParams& p, float yaw, float fmove, float smove, bool cap_ducked, TickEvents* ev)",
          "inputs": "Source/Solver/SolverMove.cpp:422. Params yaw, fmove, smove, cap_ducked. Reads s.vel, s.surface_friction, s.basevel, plus s.pos/s.hull_state/s.on_ground via TryPlayerMove; p.maxspeed, p.duck_speed_frac, p.air_speed_cap (30), p.airaccelerate (150), p.dt.",
          "outputs": "Writes s.vel (air accelerate, basevel add/subtract), s.pos via TryPlayerMove, ev contacts.",
          "purity": "mutates-member-state",
          "callable_directly": "no - anonymous namespace",
          "notes": "Mirrors CGameMovement::AirMove + AirAccelerate. The asymmetry is explicit (:435-439): the ADD is capped at GetAirSpeedCap()=30 but the accel BUDGET scales with the FULL wishspeed. accelspeed scales by s.surface_friction (0.25 airborne+rising) - fuzz-measured 2026-08-15. MEASURED ordering fix: basevel is added AFTER AirAccelerate, not before - adding it before poisons currentspeed and the whole budget (fuzz probe 5, dvel 55.6 at identical entry state, :450-454)."
        },
        {
          "name": "HandleDuck",
          "klass": "CGameMovement",
          "signature": "void HandleDuck(PlayerState& s, const World& w, const MoveParams& p, int buttons, TickEvents* ev)",
          "inputs": "Source/Solver/SolverMove.cpp:464. Param buttons (IN_DUCK bit). Reads s.old_buttons (IN_DUCK bit = previous press state), s.ducked, s.ducking, s.duck_timer_ms, s.on_ground, s.pos; p.duck_air_shift (8.5), p.time_to_duck_ms (400), p.time_to_unduck_ms (200), p.unduck_hull_defer; World via w.OriginInSolid.",
          "outputs": "Writes s.old_buttons (IN_DUCK bit), s.ducked, s.ducking, s.duck_timer_ms, s.hull_state (0/1/2), s.pos.Z (+/-8.5 air shift, plus the FixPlayerCrouchStuck probe). Writes ev->duck_changed.",
          "purity": "mutates-member-state",
          "callable_directly": "no - anonymous namespace",
          "notes": "Mirrors CGameMovement::Duck + FinishDuck + FinishUnDuck + CanUnduck + FixPlayerCrouchStuck. Provenance is MIXED and several points explicitly BEAT SDK recollection: (a) air shift is 8.500 exactly, ENGINE-MEASURED, not the 72-54=18 hull delta (SolverMove.h:61-65); (b) m_bDucking stays true through BOTH transitions and the shared timer counts DOWN from 1000 - fuzz-corrected 2026-08-15, engine 289.99 vs the instant-cancel model's 283.99 (:466-470); (c) FinishUnDuck's air origin shift is UNCONDITIONAL - BINARY-DECODED (Duck @2eec76/2eec88 -> FinishUnDuck @2eea20), fuzz_b t4 (:549-553); (d) FixPlayerCrouchStuck restores the ORIGINAL origin when no free spot is found - the old keep-every-nudge loop produced the bogus 18.0000 offset in fuzz probes 3/4/11/13 (:488-494); (e) CanUnduck tests at -8.5, NOT the SDK's -18 - TRIED AND REVERTED 2026-08-15, the SDK value broke five capture decks and dropped the fuzz corpus 98.86% -> 96.61% (:532-536). GAPS: no duck-jump state (m_bInDuckJump / m_flDuckJumpTime), no duck toggle cvar, no view-offset interpolation, no ducking sound."
        },
        {
          "name": "MoveTick",
          "klass": "CGameMovement",
          "signature": "void MoveTick(PlayerState& s, const World& w, const MoveParams& p, float pitch, float yaw, float fmove, float smove, float umove, int buttons, TickEvents* ev)",
          "inputs": "Source/Solver/SolverMove.cpp:580 (declared SolverMove.h:148). Params: yaw, fmove, smove, buttons (pitch and umove are UNUSED - commented out in the definition at :581-582). Reads/uses EVERY PlayerState field: pos, vel, ducked, hull_state, ducking, duck_timer_ms, on_ground, ground_brush, surface_friction, old_buttons, stamina, gravity_scale, basevel, basevel_flag. Reads every MoveParams field. Reads World geometry + World::HasTriggers/CheckTriggers.",
          "outputs": "Writes every PlayerState field and the whole TickEvents record.",
          "purity": "mutates-member-state",
          "callable_directly": "yes - the only public entry point of the mover (SolverMove.h:148)",
          "notes": "Mirrors CGameMovement::ProcessMovement -> PlayerMove -> FullWalkMove for MOVETYPE_WALK, plus ReduceTimers and the server's post-move trigger Touch. Order implemented: ReduceTimers (stamina + duck timer) -> Duck -> conditional re-CategorizePosition on a duck-flag change -> StartGravity (+ basevel.Z integrate and clear) -> CheckJumpButton -> grounded vz=0 + Friction -> CheckVelocity -> Walk/AirMove -> CategorizePosition -> CheckVelocity -> FinishGravity -> grounded vz=0 -> CheckVelocity -> triggers. MEASURED sequencing facts: m_surfaceFriction is NOT recomputed at tick entry - this tick's accel uses the value left by the PREVIOUS tick's end-of-move categorize (fuzz_a t117 vs fuzz_b t22, :608-616); the mid-tick recategorize after a duck-state change makes a duck-shift landing a full GROUND tick (solved11 t729, :618-625); base velocity XY PERSISTS across ticks and only Z is consumed (fuzz probe 5, 49x displacement, :586-593). BUG/INCONSISTENCY: PlayerState::basevel_flag is documented at SolverMove.h:132 as 'FL_BASEVELOCITY: decays unless refreshed', but nothing in the solver core ever clears basevel_flag or decays basevel XY - the only writes are `= true` at :684 and the trigger accumulate at :676-683. GAPS in the pipeline: no CheckParameters (so `umove` never enters the maxspeed norm and view-angle clamping is absent), no CheckWater/WaterMove/water level, no ladder/MOVETYPE_LADDER, no CheckStuck/unstuck offset table, no CheckFalling/fall damage, no MOVETYPE other than WALK, no observer/dead handling."
        },
        {
          "name": "HullExpand",
          "klass": "IEngineTrace",
          "signature": "float HullExpand(const Vec3& n, const Vec3& mn, const Vec3& mx)",
          "inputs": "Source/Solver/SolverWorld.cpp:30. Params only: plane normal n, hull mins mn, hull maxs mx.",
          "outputs": "Returns the distance offset added to the raw plane distance.",
          "purity": "pure",
          "callable_directly": "no - anonymous namespace (SolverWorld.cpp:10-40)",
          "notes": "Mirrors the engine's box-trace plane offset in CM_ClipBoxToBrush (`dist = p->dist - DotProduct(offsets, plane->normal)`), i.e. the Minkowski support of the NEGATED player hull. Applied once at load to produce d_stand/d_duck/d_unduck (SolverWorld.cpp:352-356). SDK/engine-recollection for the formula; validated by 6,624 oracle answers."
        },
        {
          "name": "World::Load",
          "klass": "BSP-parse",
          "signature": "bool World::Load(const std::string& bsp_path, const Hulls& hulls, std::string* err, bool edge_bevels = true)",
          "inputs": "Source/Solver/SolverWorld.cpp:42 (decl SolverWorld.h:93). Params: bsp_path, hulls (stand/duck/unduck min+max), err, edge_bevels. Reads the .bsp file bytes: lumps 0 (entities), 1 (planes, stride 20), 5 (nodes, stride 32), 10 (leafs, stride 56 or 32 by lump version with a length-modulo fallback), 14 (models, stride 48), 17 (leafbrushes, stride 2), 18 (brushes, stride 12), 19 (brushsides, stride 8).",
          "outputs": "Writes hulls_, brushes[] (id, contents, bmin/bmax from EXACT axial planes, n[], d[], d_stand[], d_duck[], d_unduck[], pid[], nsides, gmin_/gmax_ per hull), nodes[], leaf_contents[], tree_n[], tree_d[], headnode, triggers[], trig_brushes[], version, mapRevision, nplanes, nbrushes_total, nmodels, nbrushes_entity, have_spawn, spawn_origin/pitch/yaw, entities_text, and the grid (via BuildGrid: world_min/max, cell_, gmin_/gmax_, nx_/ny_/nz_, cells_).",
          "purity": "mutates-member-state",
          "callable_directly": "yes - public member",
          "notes": "Mirrors the engine's collision-model load (CMod_LoadBrushes / CMod_LoadBrushSides / CollisionBSPData) plus the server's entity spawn. VALIDATED format details (see solver-roadmap-1-1 memory); strides are documented on-disk sizes, never sizeof() (the 30-vs-32-byte leaf bug is called out at :130-131). Clip set = ALL non-bevel sides + the compiler's EXACT AXIAL bevels; only model-0 (worldspawn) brushes collide (:241-244). DOC DEFECT: SolverWorld.h:10-12 and SolverWorld.cpp:290 both state 'non-axial edge bevels are NOT loaded (deliberate)', but :316-332 DOES load them (pid -3) whenever edge_bevels is true - and the default is true (SolverWorld.h:94). The comments are stale; the 604-tape capture at :317-321 is the evidence that convicted the no-edge-bevel arm. GAPS: displacements are never parsed (dispinfo is read at :272 and discarded), no static props, no brush-entity solids, mask is fixed at SOLID|WINDOW|GRATE|PLAYERCLIP (:21)."
        },
        {
          "name": "World::TraceHull3",
          "klass": "IEngineTrace",
          "signature": "float World::TraceHull3(const Vec3& a, const Vec3& b, int hull, TraceResult* out) const",
          "inputs": "Source/Solver/SolverWorld.cpp:648 (decl SolverWorld.h:102). Params: a (start origin), b (end origin), hull (0 stand / 1 duck / 2 post-air-unduck transient), out. Reads brushes[].n, .pid, .d_stand|d_duck|d_unduck, .gmin_*/.gmax_*; the grid (cells_, nx_/ny_/nz_, gmin_, cell_); World::true_interval_corner; and (write side) trace_log, trace_tick.",
          "outputs": "Returns the clamped fraction in [0,1]. Fills out->frac, out->brush (index into World::brushes, NOT the BSP id), out->plane, out->normal, out->startsolid, out->allsolid. Appends a TraceProbeRow to *trace_log when set.",
          "purity": "reads-member-state (const; the only write is the optional trace_log append)",
          "callable_directly": "yes - public const member",
          "notes": "Mirrors IEngineTrace::TraceRay with a player hull, i.e. CM_BoxTrace -> CM_ClipBoxToBrush. ORACLE-MEASURED against the engine's own IEngineTrace answers: 3,555 answers pinned the corner-release rule (:692-701, the un-padded interval decides the hit, DIST_EPSILON pads only the reported position, and BEVEL sides never release); oracle query 6654 (2026-08-15) pinned the enter-plane tie-break as CLAMP-then-compare, so started-touching planes tie at 0 and brushside LUMP ORDER wins - this replaced the earlier 'least-negative' rule that manufactured a climb at ramp 2's base (:715-728). The out-of-world solidity rule was REMOVED after measurement: across 6,624 queries the engine never reported solid where we did not (:658-664). GAPS: world brushes only (no entities, displacements, or props), no contents mask parameter, uniform grid instead of the BSP tree, and the BSP tree that IS parsed is deliberately unused by the trace."
        },
        {
          "name": "World::TraceHull",
          "klass": "IEngineTrace",
          "signature": "float World::TraceHull(const Vec3& a, const Vec3& b, bool ducked, TraceResult* out) const",
          "inputs": "Source/Solver/SolverWorld.cpp:643. Params a, b, ducked, out. No state of its own.",
          "outputs": "Delegates entirely to TraceHull3(a, b, ducked ? 1 : 0, out).",
          "purity": "reads-member-state",
          "callable_directly": "yes - public const member",
          "notes": "Two-state convenience wrapper kept for the older call sites and the trace-log row semantics. Cannot express hull state 2."
        },
        {
          "name": "World::OriginInSolid",
          "klass": "CGameMovement",
          "signature": "bool World::OriginInSolid(const Vec3& o, bool ducked) const",
          "inputs": "Source/Solver/SolverWorld.cpp:831 (decl SolverWorld.h:106). Params: o (feet origin), ducked. Reads brushes[].n and .d_duck|.d_stand, plus the grid.",
          "outputs": "Returns true when the hull at o is strictly inside a collidable brush by more than DIST_EPSILON.",
          "purity": "reads-member-state",
          "callable_directly": "yes - public const member",
          "notes": "Mirrors the engine's TracePlayerBBox startsolid test as used by CGameMovement::CanUnduck and FixPlayerCrouchStuck. The `>= -kDistEpsilon` rule (:846-849) is deliberate so a hull resting ON a face (dist 0) does not read as stuck. GAP: only two hull states - no unduck-transient (hull 2) variant, so HandleDuck's CanUnduck/crouch-stuck probes always test the stand or duck box."
        },
        {
          "name": "World::CheckTriggers",
          "klass": "other",
          "signature": "bool World::CheckTriggers(const Vec3& origin, int hull_state, TriggerHitS* out) const",
          "inputs": "Source/Solver/SolverWorld.cpp:516 (decl SolverWorld.h:156). Params: origin (feet), hull_state (0/1/2), out. Reads triggers[] (spawnflags, ent_origin, teleport/push flags, dest_origin/pitch/yaw, landmark_ok/landmark_origin, push_dir, push_speed, gravity, tb[]), trig_brushes[] (n, d), hulls_.",
          "outputs": "Fills out->teleported, tp_origin, tp_set_angles, tp_pitch/tp_yaw, pushed, push_vec, grav_touched, gravity. Returns whether anything was touched.",
          "purity": "reads-member-state",
          "callable_directly": "yes - public const member",
          "notes": "Mirrors CTriggerTeleport::Touch / CTriggerPush::Touch / CTriggerGravity::GravityTouch plus the SF_TRIGGER_ALLOW_CLIENTS gate (0x01) and SF_TELEPORT_PRESERVE_ANGLES (0x20). Ported 1:1 from the DLL's BspWorld::CheckTriggers (SDK-recollection for the Touch semantics; the containment math is the same support-corner Minkowski test as the trace, using ALL sides + ALL bevels). Applied by MoveTick:669-693. GAPS: trigger enable/disable and Start Disabled, SF_TRIG_PUSH_ONCE, entity filters (filter_activator_*), Touch ORDER (iterates parse order, not entity index), trigger brush-entity `angles` rotation, trigger_multiple/once wait timers, and OnStartTouch/OnEndTouch outputs."
        },
        {
          "name": "World::PointInSolidLeaf",
          "klass": "IEngineTrace",
          "signature": "bool World::PointInSolidLeaf(const Vec3& p) const",
          "inputs": "Source/Solver/SolverWorld.cpp:794 (decl SolverWorld.h:193). Param p. Reads nodes[], tree_n[], tree_d[], leaf_contents[], headnode.",
          "outputs": "Returns (leaf_contents & CONTENTS_SOLID) != 0.",
          "purity": "reads-member-state",
          "callable_directly": "yes - public const member",
          "notes": "Mirrors CM_PointLeafnum's descent exactly (child[0] on d >= 0). Engine-recollection. DEAD IN THE MOVEMENT PATH: TraceHull3:658-664 explicitly rejects any leaf-contents solidity rule as over-reporting, so this is diagnostic only."
        },
        {
          "name": "World::HullInSolidLeaf",
          "klass": "IEngineTrace",
          "signature": "bool World::HullInSolidLeaf(const Vec3& o, int hull) const",
          "inputs": "Source/Solver/SolverWorld.cpp:814 (decl SolverWorld.h:196). Params o, hull. Reads hulls_ and everything PointInSolidLeaf reads.",
          "outputs": "Returns true when any of the 8 hull corners or the origin lands in a solid leaf.",
          "purity": "reads-member-state",
          "callable_directly": "yes - public const member",
          "notes": "APPROXIMATION, not a true mirror: the engine's box-vs-tree query is CM_BoxLeafnums (which descends with the box extents and visits every straddled leaf); sampling 9 points can miss a thin solid the box straddles. Also diagnostic-only for the same reason as PointInSolidLeaf."
        },
        {
          "name": "World::BrushUnder",
          "klass": "other",
          "signature": "int World::BrushUnder(const Vec3& o, bool ducked) const",
          "inputs": "Source/Solver/SolverWorld.cpp:857 (decl SolverWorld.h:109). Params o, ducked. Reads brushes[].id via a 4096u down TraceHull.",
          "outputs": "Returns the BSP brush id beneath o, or -1.",
          "purity": "reads-member-state",
          "callable_directly": "yes - public const member",
          "notes": "Solver convenience, NOT an engine function. Uses the engine-mirroring trace underneath, so its answer inherits trace parity."
        },
        {
          "name": "World::BuildGrid / World::CellRange",
          "klass": "other",
          "signature": "void World::BuildGrid(); void World::CellRange(const Vec3& lo, const Vec3& hi, int* x0, int* x1, int* y0, int* y1, int* z0, int* z1) const",
          "inputs": "Source/Solver/SolverWorld.cpp:585 and :632. BuildGrid reads brushes[].bmin/bmax and .gmin_stand/.gmax_stand; CellRange reads gmin_, cell_, nx_/ny_/nz_.",
          "outputs": "BuildGrid writes world_min/world_max, gmin_/gmax_, cell_, nx_/ny_/nz_, cells_. CellRange writes the six clamped index outputs.",
          "purity": "BuildGrid mutates-member-state; CellRange reads-member-state",
          "callable_directly": "no - private members (SolverWorld.h:227-229)",
          "notes": "NO engine counterpart - the engine uses the BSP tree for broadphase. Ours is a uniform 256u grid over the STANDING expanded AABBs (a conservative superset of the ducked ones), coarsened by doubling if the cell count would exceed 4M. Listed here only because every trace's candidate set comes from it."
        },
        {
          "name": "World::IndexOfBrushId",
          "klass": "other",
          "signature": "int World::IndexOfBrushId(int bsp_id) const",
          "inputs": "Source/Solver/SolverWorld.cpp:863. Param bsp_id; reads brushes[].id.",
          "outputs": "Returns the index into World::brushes, or -1.",
          "purity": "reads-member-state",
          "callable_directly": "yes - public const member",
          "notes": "Bookkeeping helper, not an engine mirror. Linear scan."
        },
        {
          "name": "BspWorld::CheckTriggers",
          "klass": "other",
          "signature": "bool BspWorld::CheckTriggers(const Vector& origin, const Vector& mins, const Vector& maxs, TriggerHit* out)",
          "inputs": "Source/World/BspWorld.cpp:1037 (decl BspWorld.h:131). Params: origin, hull mins/maxs (the CALLER picks 54 vs 72 for the top), out. Reads the file-static apply_triggers, g_trigvols[], g_brushes[].clip_planes/.bevel_planes, g_planes[], g_status.loaded.",
          "outputs": "Fills out->teleported, tp_origin, tp_set_angles, tp_pitch/tp_yaw, grav_touched, gravity, pushed, push_vec.",
          "purity": "reads-member-state (file-static globals)",
          "callable_directly": "yes - declared in BspWorld.h",
          "notes": "The DLL-side TWIN of Solver::World::CheckTriggers - same SDK Touch semantics, same support-corner containment. This is the ORIGINAL that the solver port was copied from (SolverWorld.cpp:367-369 says so). Divergence from the solver twin: this one takes explicit mins/maxs, so the caller in Prediction.cpp (:539-541 and :251-253) hardcodes hmax.Z = 54 when FL_DUCKING, which does NOT reproduce the solver's hull-state-2 (62.5) unduck transient. Same GAP list as the solver twin (no enable/disable, no filters, no angles, no touch ordering)."
        },
        {
          "name": "BspWorld::Pick",
          "klass": "IEngineTrace",
          "signature": "BspWorld::RayHit BspWorld::Pick(const Vector& eye, const Vector& dir, float max_dist)",
          "inputs": "Source/World/BspWorld.cpp:1308 (decl BspWorld.h:34). Params eye, dir, max_dist. Reads g_brushes[].contents/.clip_planes, g_planes[], and the show_solid/show_playerclip/show_ladder toggles.",
          "outputs": "Returns RayHit{hit, brush, plane, contents, dist, point, normal}.",
          "purity": "reads-member-state",
          "callable_directly": "yes - declared in BspWorld.h",
          "notes": "Ray-vs-brush parametric interval clip, shaped like CM_ClipBoxToBrush for a ZERO-EXTENT ray. NOT a movement-parity path (it is the crosshair picker for face tagging / board targets): no DIST_EPSILON backoff, no bevel planes, no hull expansion, no startsolid/allsolid reporting, and it honors the DRAW toggles so what you see is what you pick. Do not use it as a collision oracle."
        },
        {
          "name": "BspWorld::LoadFromFile",
          "klass": "BSP-parse",
          "signature": "bool LoadFromFile(const std::string& path)",
          "inputs": "Source/World/BspWorld.cpp:370. Param path. Reads the .bsp bytes (dheader_t/dplane_t/dbrush_t/dbrushside_t/dnode_t/dmodel_t/dleaf_common_t, version gate 17..21).",
          "outputs": "Writes g_brushes (contents, clip_planes, bevel_planes, faces, world/model tags), g_planes, g_trigvols (via ParseEntities), g_status (brush/face/edge counts, parse_ms, error).",
          "purity": "mutates-member-state",
          "callable_directly": "no - anonymous namespace; reached via the public BspWorld::LoadCurrentMap (BspWorld.cpp:992)",
          "notes": "DLL-side twin of Solver::World::Load, but face-oriented (it winds each side against the others for the wireframe) rather than clip-oriented. Uses sizeof() on packed structs here, unlike the solver loader which deliberately does not (see SolverWorld.cpp:130-131) - the leaf stride is the risky one. Same GAPS: no displacements, no static props."
        },
        {
          "name": "ParseEntities",
          "klass": "BSP-parse",
          "signature": "void ParseEntities(const char* txt, size_t len)",
          "inputs": "Source/World/BspWorld.cpp:259. Params txt, len (the raw entity lump). Keys consumed: classname, targetname, target, landmark, model, origin, angles, spawnflags, gravity, pushdir, speed.",
          "outputs": "Writes g_trigvols[] and the g_trig_teleports / g_trig_gravities / g_trig_pushes counters.",
          "purity": "mutates-member-state",
          "callable_directly": "no - anonymous namespace",
          "notes": "Mirrors the server's entity keyvalue parse + trigger spawn for trigger_teleport / trigger_gravity / trigger_push only. SDK-recollection: 'pushdir' is angles and the world direction is the forward vector (:354-359), duplicated verbatim in the solver port at SolverWorld.cpp:467-474. Triggers with an unresolvable `target` are silently skipped (:350-351). GAP: entity `angles` on the trigger brush model itself are parsed for NAMED entities only and never applied to the volume."
        },
        {
          "name": "BaseWinding / Chop",
          "klass": "BSP-parse",
          "signature": "void BaseWinding(const Vector& n, float d, std::vector<Vector>& out); void Chop(std::vector<Vector>& w, const Vector& n, float d)",
          "inputs": "Source/World/BspWorld.cpp:166 and :190. Plane normal + distance; Chop additionally takes the winding to clip in place.",
          "outputs": "out / w - the face polygon.",
          "purity": "pure (operate only on their arguments)",
          "callable_directly": "no - anonymous namespace",
          "notes": "Mirrors the MAP COMPILER's BaseWindingForPlane / ChopWindingInPlace (vbsp utils), not a runtime engine function. Faces are for drawing and face tagging; collision never consults them (the clip set is the plane set)."
        },
        {
          "name": "RestOnPlane",
          "klass": "other",
          "signature": "bool RestOnPlane(int plane, bool ducked, const Vector& contact_point, Vector& out)",
          "inputs": "Source/World/BspWorld.cpp:862. Params plane id, ducked, contact_point; reads g_planes[]. Hull height is HARDCODED 54/72 here (:867).",
          "outputs": "out = the hull feet origin resting tangent on the plane.",
          "purity": "reads-member-state",
          "callable_directly": "no - anonymous namespace",
          "notes": "Solver-authoring helper (board targets), NOT an engine mirror. Uses the same support-corner geometry as the trace expansion, so it lands the hull where a collision resolve would - but with literal 54/72, independent of Prediction::PlayerHull's measured box."
        },
        {
          "name": "BeginStateGuard / EndStateGuard",
          "klass": "other",
          "signature": "bool BeginStateGuard(void* player); void EndStateGuard()",
          "inputs": "Source/World/Prediction.cpp:174 and :196. Param player. Reads *g_gpg_holder (gpGlobals), gpGlobals+0x10 (frametime), +0x0C (curtime), +0x1C (interval_per_tick, sanity-gated to (0,1]), g_helper+0x18 (touch-list count), and kPlayerSize=0x1C50 bytes of the player object.",
          "outputs": "Writes s_snapshot (whole player), s_save_frametime/curtime/touch, s_p_frametime/curtime/touch, g_interval, s_player, s_restore_ready; OVERWRITES gpGlobals->frametime with interval_per_tick. EndStateGuard restores all four byte-for-byte.",
          "purity": "mutates-member-state (mutates LIVE ENGINE memory)",
          "callable_directly": "no - anonymous namespace",
          "notes": "NOT an engine mirror - this is the save/restore harness that makes hypothetical engine ticks safe. Offsets are pinned by static RE of client.dll (prediction-re memory). Everything downstream of it is engine GROUND TRUTH, not a model."
        },
        {
          "name": "RunLookaheadSEH",
          "klass": "other",
          "signature": "int RunLookaheadSEH(void* player, void* ucmd, int ticks)",
          "inputs": "Source/World/Prediction.cpp:204. Params player, ucmd, ticks. Reads WorldDraw::pred_live_input / pred_forwardmove / pred_sidemove / pred_jump / pred_duck / pred_autobhop, g_off.flags/origin/velocity/gravity/basevel, g_hull_min/g_hull_max, the live CUserCmd.",
          "outputs": "Writes g_path[0..ticks-1]; mutates the live player through the real engine calls and restores it; applies trigger effects to the player object (gravity, basevel + FL_BASEVELOCITY, teleport).",
          "purity": "mutates-member-state",
          "callable_directly": "no - anonymous namespace; reached through the FinishMove hook",
          "notes": "This is the ENGINE ITSELF, not a mirror: it calls IPrediction::SetupMove (vtable 18) -> IGameMovement::ProcessMovement (vtable 1) -> the original IPrediction::FinishMove (vtable 19) per tick. Its only modelled part is the trigger block at :246-278, which duplicates CTriggerPush::Touch / CTriggerTeleport::Touch / CTriggerGravity::GravityTouch (the client never runs server trigger logic). GAP: hmax.Z is hardcoded to 54 when FL_DUCKING (:251-253) - no unduck-transient hull."
        },
        {
          "name": "RunEditorSimSEH",
          "klass": "other",
          "signature": "int RunEditorSimSEH(void* player)",
          "inputs": "Source/World/Prediction.cpp:451. Reads s_sim_anchor (origin, velocity, ducked, stamina), s_sim_req_ticks, s_sim_provider, g_off.* (origin, velocity, flags, ducked, ducking, ducktime, stamina, maxs, gravity, basevel), g_hull_min/max, movedata offsets 0x44 (velocity), 0x9C (origin), 0x3C/0x40 (maxspeed candidates).",
          "outputs": "Writes s_sim_frames[], s_sim_states[] (origin, velocity, flags, hull_top, mspd_a, mspd_b, stamina_ms), s_trig_events[], s_trig_event_count. Mutates the live player and restores it.",
          "purity": "mutates-member-state",
          "callable_directly": "no - anonymous namespace; queued via Prediction::RequestSim (:862)",
          "notes": "Engine ground truth, same three vtable calls. MEASURED harness fact (:500-505): the netvar velocity write never reaches SetupMove's abs-velocity copy, so tick 0's movedata is seeded directly through the pinned offsets. The trigger application block (:538-590) is the modelled part and mirrors CTriggerPush::Touch's accumulate + unground + 1u nudge exactly. This is the function that produces the engine-side rows every parity deck is scored against."
        },
        {
          "name": "RunOneFuzzProbeSEH",
          "klass": "other",
          "signature": "int RunOneFuzzProbeSEH(void* player, const FuzzProbeIn* in, FuzzTickOut* out)",
          "inputs": "Source/World/Prediction.cpp:328. Param `in` carries every field that can affect a tick: origin, velocity, basevel, onground, ducked, ducking, ducktime, stamina, gravity, sfric (written to the literal client offset 0x1868), hullmin_z/hullmax_z/hull_xy, and 5 ticks of yaw/fmove/smove/buttons.",
          "outputs": "out[0..4]: origin, velocity, basevel, flags, ducked, ducking, ducktime, stamina, sfric, maxz, ok.",
          "purity": "mutates-member-state",
          "callable_directly": "no - anonymous namespace; queued via Prediction::RunFuzz (:803), executed in chunks of 400 inside the FinishMove hook",
          "notes": "The differential-fuzz oracle - the instrument that produced the m_surfaceFriction 0.25 rule, the ducking-through-unduck correction, the unconditional air FinishUnDuck shift, and the basevel-persistence finding. MEASURED harness facts: tick 0 is a SETTLE tick because writing FL_ONGROUND does NOT ground the player (ground is the m_hGroundEntity handle - 96% of 'grounded' probes were airborne) and the engine re-derives duck state on a third of probes (:301-307). kSurfFricOff = 0x1868 is BINARY-DECODED (client.dll @0011adb4 / @0011fb81)."
        },
        {
          "name": "OnFinishMove",
          "klass": "other",
          "signature": "void OnFinishMove(void* thisptr, void* player, void* ucmd, void* movedata)",
          "inputs": "Source/World/Prediction.cpp:610. Hook of IPrediction::FinishMove (vtable index 19). Reads g_in_hook_work, g_pred/g_gm/g_helper/g_gpg_holder, IPrediction::IsFirstTimePredicted (vtable 15), movedata+0x9C (origin) and +0x3C (m_flMaxSpeed), s_fuzz_pending, s_sim_pending, WorldDraw::draw_prediction/pred_ticks.",
          "outputs": "Writes g_real_move_origin, g_real_maxspeed, g_real_move_valid, g_pred_flags(+valid), g_path_count, g_diag, and dispatches to the fuzz / editor-sim / lookahead jobs.",
          "purity": "mutates-member-state",
          "callable_directly": "no - installed as a VMT hook by Prediction::Install (:770)",
          "notes": "Runs the REAL movement first (:617) then does one job per frame in the same valid context. All three job types must run here - driving ProcessMovement from the UI thread faults every probe (:652-655). m_flMaxSpeed at movedata+0x3C is IDENTIFIED, not assumed (knife 250 / no weapon 260)."
        },
        {
          "name": "NOT-MODELED: CGameMovement::CheckParameters",
          "klass": "CGameMovement",
          "inputs": "n/a - absent from the solver core",
          "outputs": "n/a",
          "purity": "n/a",
          "callable_directly": "no - does not exist",
          "notes": "The SDK scales fmove/smove/upmove down when sqrt(f^2+s^2+u^2) > maxspeed, clamps view pitch to [-89,89], and sets mv->m_flMaxSpeed. We clamp wishspeed inside Walk/AirMove instead, which is equivalent for the f/s pair but NOT for upmove: MoveTick's `umove` param is discarded entirely (SolverMove.cpp:582), so a nonzero upmove that would have shrunk the horizontal wish in the engine does nothing here. m_flMaxSpeed is a static MoveParams value, not refreshed per tick from the weapon."
        },
        {
          "name": "NOT-MODELED: CGameMovement::CheckWater / WaterMove / CheckWaterJump",
          "klass": "CGameMovement",
          "inputs": "n/a - absent",
          "outputs": "n/a",
          "purity": "n/a",
          "callable_directly": "no - does not exist",
          "notes": "SolverMove.h:4 states the scope outright: 'MOVETYPE_WALK, no water'. Water brushes are also excluded from the clip mask (SolverWorld.cpp:21 kPlayerSolid = SOLID|WINDOW|GRATE|PLAYERCLIP), so a water volume is simply not there. No water level, no buoyancy, no water friction, no waterjump timer."
        },
        {
          "name": "NOT-MODELED: CGameMovement::FullLadderMove / MOVETYPE_LADDER",
          "klass": "CGameMovement",
          "inputs": "n/a - absent",
          "outputs": "n/a",
          "purity": "n/a",
          "callable_directly": "no - does not exist",
          "notes": "CONTENTS_LADDER (0x20000000) is a DRAW category in BspWorld.cpp:22/:1226/:1316 only. The solver's clip mask never includes it and no ladder movement/attach/detach path exists in SolverMove.cpp."
        },
        {
          "name": "NOT-MODELED: CGameMovement::CheckStuck / TestPlayerPosition unstuck table",
          "klass": "CGameMovement",
          "inputs": "n/a - absent",
          "outputs": "n/a",
          "purity": "n/a",
          "callable_directly": "no - does not exist",
          "notes": "The engine runs CheckStuck before FullWalkMove and tries a 54-entry offset table to free an embedded hull. We model only the CONSEQUENCE: TryPlayerMove zeroes velocity on allsolid (SolverMove.cpp:65-74) and the comment asserts the engine 'repeats it every tick until something frees the hull' (measured 18 ticks, d34 tape). That is an empirical claim about cases where CheckStuck failed to free the player - it is not the same as modelling CheckStuck, so any map/geometry where the unstuck table DOES free the hull will diverge."
        },
        {
          "name": "NOT-MODELED: CGameMovement::CheckFalling / m_flFallVelocity / PlayerRoughLandingEffects",
          "klass": "CBasePlayer",
          "inputs": "n/a - absent",
          "outputs": "n/a",
          "purity": "n/a",
          "callable_directly": "no - does not exist",
          "notes": "No fall tracking, no fall damage, no landing slowdown. Measurement supports the omission for movement purposes: SolverMove.cpp:205-206 and :81-82 record that a jump-free -775 u/s floor landing walked at stamina scale 1.00000, i.e. 'Landings arm NOTHING on this server'."
        },
        {
          "name": "NOT-MODELED: surface-property friction (m_surfaceFriction from the struck texture)",
          "klass": "CGameMovement",
          "inputs": "n/a - absent",
          "outputs": "n/a",
          "purity": "n/a",
          "callable_directly": "no - does not exist",
          "notes": "The engine sets m_surfaceFriction in CategorizePosition from the ground surface's physprops friction value. Our CategorizePosition (SolverMove.cpp:166-178) only ever produces 1.0 or the measured 0.25 airborne-rising value. Ice/slick/high-friction surfaces would diverge; surf_basictest has none, which is why the two-value model certified green."
        },
        {
          "name": "NOT-MODELED: ground-entity base velocity (moving platforms, func_conveyor, func_tracktrain)",
          "klass": "CBasePlayer",
          "inputs": "n/a - absent",
          "outputs": "n/a",
          "purity": "n/a",
          "callable_directly": "no - does not exist",
          "notes": "PlayerState::ground_brush is a brush INDEX, not an entity, so a moving standable entity cannot carry the player. Base velocity only ever enters from trigger_push (MoveTick:674-685)."
        },
        {
          "name": "NOT-MODELED: FL_BASEVELOCITY decay",
          "klass": "CBasePlayer",
          "inputs": "n/a - the decay path is absent",
          "outputs": "n/a",
          "purity": "n/a",
          "callable_directly": "no - does not exist",
          "notes": "SolverMove.h:132 documents basevel_flag as 'FL_BASEVELOCITY: decays unless refreshed', but grep over Source/Solver shows the flag is only ever SET (SolverMove.cpp:684) and read (:676); basevel XY is never zeroed or decayed anywhere. Only basevel.Z is consumed, by StartGravity (:639-640). The persistence itself is measured (fuzz probe 5, :586-593) but 'persists forever' and 'decays unless refreshed' are different laws, and only the first is implemented."
        },
        {
          "name": "NOT-MODELED: entity collision (other players, physics props, func_door/func_breakable/brush-entity solids)",
          "klass": "IEngineTrace",
          "inputs": "n/a - absent",
          "outputs": "n/a",
          "purity": "n/a",
          "callable_directly": "no - does not exist",
          "notes": "SolverWorld.cpp:241-244 keeps ONLY model-0 (worldspawn) brushes; every other solid brush is counted into nbrushes_entity and dropped. Documented as correct for triggers/illusionaries (SolverWorld.h:13-14: brush entities are CONTENTS_SOLID on disk but block nothing) - but that reasoning does NOT cover func_ entities that really do collide."
        },
        {
          "name": "NOT-MODELED: displacement surfaces (terrain)",
          "klass": "BSP-parse",
          "inputs": "n/a - absent",
          "outputs": "n/a",
          "purity": "n/a",
          "callable_directly": "no - does not exist",
          "notes": "Neither loader reads the displacement lumps. SolverWorld.cpp:272 reads the brushside `dispinfo` field and never uses it. Docs/FullMapSolver.md:157 flags this as risk R2 ('Collision gaps: displacements, func_-entity solids. Phase 0 must REPORT what a map contains'). Any displacement map is silently un-collidable."
        },
        {
          "name": "NOT-MODELED: CS:S ground-duck lifecycle (crouch-walk / duck spam on the ground)",
          "klass": "CCSGameMovement",
          "inputs": "n/a - partially absent",
          "outputs": "n/a",
          "purity": "n/a",
          "callable_directly": "no - does not exist",
          "notes": "Docs/FullMapSolver.md:531-536: at tick 917 the engine walked duck-capped at wishspeed 85 while FL_DUCKING was 0 - transition/spam mechanics that are NOT modeled. The mitigation is a SOLVER-side input rule (emit duck presses/releases AIRBORNE ONLY), not a model fix. Note the duck decks that ARE certified (air-duck +/-8.5, ducked landing, ducked jump, air-unduck) all sit inside that airborne restriction. HandleDuck's ground branches exist but are the least capture-exercised part of the mover."
        },
        {
          "name": "NOT-MODELED: duck-jump state (m_bInDuckJump / m_flDuckJumpTime)",
          "klass": "CGameMovement",
          "inputs": "n/a - absent",
          "outputs": "n/a",
          "purity": "n/a",
          "callable_directly": "no - does not exist",
          "notes": "PlayerState carries ducked / hull_state / ducking / duck_timer_ms but no duck-jump member. The SDK's Duck() defers the unduck when a jump started the duck; HandleDuck (SolverMove.cpp:464) has no such branch."
        },
        {
          "name": "NOT-MODELED: trigger lifecycle (enable/disable, Start Disabled, SF_TRIG_PUSH_ONCE, filters, touch order, brush-model angles)",
          "klass": "other",
          "inputs": "n/a - absent",
          "outputs": "n/a",
          "purity": "n/a",
          "callable_directly": "no - does not exist",
          "notes": "Both CheckTriggers implementations (SolverWorld.cpp:516, BspWorld.cpp:1037) gate only on SF_TRIGGER_ALLOW_CLIENTS (0x01) and, for teleports, SF_TELEPORT_PRESERVE_ANGLES (0x20). Touch order is entity-lump parse order, not engine entity index; 'first teleport wins' therefore depends on parse order. Only trigger_teleport / trigger_gravity / trigger_push classnames are recognized at all (SolverWorld.cpp:435-438, BspWorld.cpp:320-322)."
        },
        {
          "name": "NOT-MODELED: unduck-transient hull on the DLL trigger path",
          "klass": "other",
          "inputs": "n/a - inconsistency between the two ports",
          "outputs": "n/a",
          "purity": "n/a",
          "callable_directly": "no",
          "notes": "The solver core carries three hull states (72 / 54 / 62.5, Hulls at SolverWorld.h:30-49) and Solver::World::CheckTriggers takes hull_state 0/1/2 (SolverWorld.cpp:524-527). The DLL side hardcodes hmax.Z = 54 on FL_DUCKING (Prediction.cpp:539-541 and :251-253) with no 62.5 case, so trigger touches during a post-air-unduck transient can disagree between the two implementations."
        }
      ],
      "sources": [
        "C:/Users/Connor/Documents/SourceTAS/sourceTAS/Source/Solver/SolverMove.h (152 lines, read in full)",
        "C:/Users/Connor/Documents/SourceTAS/sourceTAS/Source/Solver/SolverMove.cpp (696 lines, read in full)",
        "C:/Users/Connor/Documents/SourceTAS/sourceTAS/Source/Solver/SolverWorld.h (232 lines, read in full)",
        "C:/Users/Connor/Documents/SourceTAS/sourceTAS/Source/Solver/SolverWorld.cpp (870 lines, read in full)",
        "C:/Users/Connor/Documents/SourceTAS/sourceTAS/Source/World/Prediction.h (141 lines, read in full)",
        "C:/Users/Connor/Documents/SourceTAS/sourceTAS/Source/World/Prediction.cpp (987 lines, read in full)",
        "C:/Users/Connor/Documents/SourceTAS/sourceTAS/Source/World/BspWorld.h (164 lines, read in full)",
        "C:/Users/Connor/Documents/SourceTAS/sourceTAS/Source/World/BspWorld.cpp (1631 lines, read at :166-368, :370-384, :855-880, :1030-1114, :1305-1359; function inventory via grep)",
        "C:/Users/Connor/Documents/SourceTAS/sourceTAS/Docs/FullMapSolver.md (gap-admitting sections: :157 risk R2, :520-542 ground-duck lifecycle + edge-contact retraction, :2052-2054 stale known-gaps list)",
        "C:/Users/Connor/.claude/projects/C--Users-Connor-Documents-SourceTAS-sourceTAS/memory/engine-parity-certified.md (instrument inventory + the four mechanics that hid)",
        "grep sweep for gap-admitting comments across Source/Solver and Source/World (TODO/FIXME/not model/water/ladder/assume/missing/fallback)",
        "grep sweep for basevel|basevel_flag|gravity_scale across Source/Solver (established that basevel_flag is never cleared)"
      ]
    },
    "critique": "## Grounding

I read the code both designs cite. Confirmations and corrections before the review:

- `Source\\World\\Prediction.cpp:328` `RunOneFuzzProbeSEH` — confirmed. `FuzzProbeIn` (`:296`) has **no `old_buttons` field** and nothing writes `m_nOldButtons`. Design B's claim (b) is true.
- `Source\\Solver\\SolverLab.cpp` `CmdFuzzDiff` — confirmed `break;` on first divergence inside the tick loop. Design B's claim (a) is true.
- **A currently-live lie neither design names:** the divergence dump prints `s_at_fail` as the "pre" state — that is *our model's* tick-(t−1) output, not the engine's. The engine's true pre-state is sitting unused in `res[i*kFuzzTicks + t - 1]`. Every forensic dump this project has read since the fuzz harness landed anchored on a state the engine never occupied. Fix costs four lines and belongs in whichever design ships.
- `SolverMove.cpp:5` opens `namespace {`, closes `:696` (not `:578` — A's line numbers are stale, which matters only because A presents them as verified).
- Green board items 2, 3, and 4 (`Docs\\FullMapSolver.md:1953-1980`) are **all** ordering/ownership/state-machine defects found by running the real pipeline. Item 4's ownership rule is explicitly *cross-tick*. Hold that thought.

---

# DESIGN A — leaf-first direct-call

### What can silently lie

**1. The purity gate is a tautology as specified.** "The input slot vector is a claim… the purity gate tests that claim empirically." It cannot. Testing "these are ALL the values this function reads" requires varying the values you did *not* declare — i.e. the ones you don't know about. Any gate that only varies declared slots proves declared slots matter and says nothing about the rest. The only honest form is: hold the declared input vector fixed, randomize the entire 0x1C50 player object + gpGlobals + every convar between repeats, demand bit-identical output. A does not say this. Without it, a function that reads `player+0x1868` through `this` returns consistent answers all session and different answers after a weapon switch — the exact "harness silently lies" signature.

**2. `Accelerate`'s slot list contains `surface_friction`.** Declaring it an input **assumes the answer to green-board item 4.** The registry is presented as the executable design document; here it bakes in a resolved question as a premise, and no probe in A can ever falsify it, because A supplies the value. This is not a hypothetical — it's the single most expensive bug this project has found, encoded as a constant in the new instrument.

**3. `Friction`'s slot list contains `{"on_ground", kBool}`.** This project has *already measured* that ground is `m_hGroundEntity`, a handle, and that writing `FL_ONGROUND` does not ground the player — 96% of "grounded" probes were airborne (`Prediction.cpp:296` comment block). A `bool on_ground` input slot is either unsettable or a fabricated entity pointer written into live prediction. A re-commits the documented sin.

**4. MXCSR / denormals.** A captures `Mxcsr()` and never says it *applies* it offline. `SolverLab.exe` runs with FTZ/DAZ off; the game process may not (renderer/audio DLLs set it). The existing corpus **already contains `1e-30f`** (`SolverLab.cpp` `kSpeeds[1]`) and `1e-7f`. Engine returns 0, model returns 1e-30, differ reports "math is wrong." Indistinguishable from a real defect, and armed today.

**5. `double` marshalling.** `Invoke(int, const double*, double*)` with "narrowed to float on the exact bit pattern carried." But `jump_impulse_d = 301.99337741082996` is a **genuine baked double** (`SolverMove.h:44`), decoded from the binary. A slot vocabulary with no `kF64` will round-trip that through a float somewhere and lose the one constant the project spent a disassembly session pinning.

**6. Per-chunk guard hoisting is the most dangerous line in A.** Justification: "every field the function reads is rewritten per probe anyway." That is the input-slot claim *again*, used as a premise to make the harness 250× cheaper. If any function writes an undeclared field — `m_flFallVelocity`, `m_nOldButtons`, the helper touch list, `m_surfaceFriction` — probe N+1 inherits probe N's residue. Symptom: reproducible within a run, non-reproducible when `nprobes` changes or a chunk boundary moves. Reads as flakiness. Gets "fixed" with a tolerance.

**7. Pin batteries are near-vacuous.** `Friction: speed=0 → unchanged; on_ground=false → unchanged`. A null function passes both. `Accelerate: wishspeed=0 → unchanged`. Null passes. `ClipVelocity: in=(0,0,-100), n=(0,0,1) → (0,0,0)` — passes for a large family of wrong callees, and passes for the *right* callee with the wrong calling convention if the first vector arg lands. Known-answer probes whose known answer is "nothing happened" are the least discriminative probes constructible. Only the 45° ramp reflect has real entropy. A is correct that `UNPINNED` must never be reported as a mismatch — but **MISPINNED** is the failure that matters and this battery does not catch it.

**8. `kAccInlined` isolation-by-nulling assumes the call order.** "Call `WalkMove` with fmove=smove=0 and stamina=0, so the delta *is* Friction." That presumes Friction/stamina/accelerate ordering inside WalkMove — and `SolverMove.h:76` documents that order as something that had to be *binary-decoded* (`vel.xy *= powf(...)` **after** Friction, **before** accelerate, @2f0160). If the order were different, the residue is still labelled "Friction," internally consistent, green. This is guessing-dressed-as-classification — A's own opening indictment — relocated one level down and given a nicer name.

### Inputs A cannot set
Ground entity handle. `m_flGravity` (fuzzdiff carries `--fuzz-grav1` *precisely because* the engine may ignore the write — A has no representation for "declared input the engine ignored"; such a slot just measures zero effect at every value and both sides agree, a green that means nothing). Convars, if the callee reads the `ConVar` object rather than an argument — Source `Friction` does. `dt`, if the callee reads `gpGlobals->frametime` rather than a parameter. Hull dims, if `m_vecMins/m_vecMaxs` are re-derived from duck state (the current harness writes them directly and nobody has verified the write survives). Every one of these turns a swept dimension into fake coverage.

### Outputs A cannot observe
Anything written to `this` or `CMoveData` outside the hand-written out-slot list. And because out-lists are hand-written, undeclared side effects are discarded *and then the next ladder rung is fed a hand-built input that lacks them*. **The ladder has no rung that tests composition.** N green functions do not make a green pipeline, and A ships no instrument that can tell the difference.

### Functions A cannot isolate
`Friction` (inlined — **A's flagship level-0 leaf is not directly callable, by A's own text**; coverage collapses at rung 0). `CategorizePosition` (needs real world + ground entity + trace vtable — and note the `needs_world` binding section is where the document is **literally truncated**; the hard part is unwritten). Anything else the optimizer dissolved, which is unknowable in advance. And structurally: **the surf-friction ownership rule is not inside any function.** It lives in the order. No registry entry can hold it.

### How a wrong result looks right
All-green registry, per-function agreement at 1e-7, `SolverLab battery` still failing on real tapes, and no instrument capable of explaining the gap because the instrument is blind to composition by construction. Or: a mispinned `Accelerate` that passes its no-op pin battery, gets certified green against the wrong function, and ships.

### Also wrong
"`--dump-reach` and interaction classes must be **deleted**, not ported." Interaction classes are guesswork — kill them. **Reachability is not guesswork**; it's a fact about the solver's domain, and it's the only thing keeping out-of-world and embedded-start probes from dominating the failure count. A conflates the two and throws away the good one.

### Worth keeping
§2's refactor acceptance — byte-identical `battery`/`fuzzdiff`/tape-deck CSVs before and after — is exactly right. So is §3a's `gm-context-verify` latch and fail-closed posture.

---

# DESIGN B — isolation through ProcessMovement

### What can silently lie

**1. M2's forcing direction is wrong, and wrong on B's own decisive example.** "Force field *i* to the engine-observed **post**-value." The post-value of `m_surfaceFriction` is *not* the value the tick read — that is precisely green-board item 4: it's written by the end-of-move categorize and consumed by the **next** tick. Forcing post-values mis-attributes exactly the defect class B names as its winning argument. B would have gotten the surf-friction bug wrong. Fix: force from tick N−1's post-dump, or force the full {pre, post} cross product and report which one repairs — that difference *is* the ownership signal, and it's free once you're dumping both.

**2. M2's repair sets are not identifiable.** Forcing `surface_friction` OR forcing `vel` can both repair a tick; forcing a downstream field masks an upstream defect. "The set of forcings that repair names the producing function" is false in general. You get a ranked candidate list — better than `"both (dp 1.2 dv 40)"`, but still classification, not proof. B should say so instead of calling it "attribution by ablation."

**3. Convar silencing is load-bearing and least verified.** Direct field writes bypass `SetValue`, therefore bypass the `m_fValue`/`m_nValue`/`m_pszString` sync. A `GetFloat()` readback reads `m_fValue` — **it confirms your own write and proves nothing** if any consumer reads `m_nValue` or re-parses the string. `FCVAR_REPLICATED` server-authoritative resend can revert your write mid-batch, silently, between experiments. This is the same class as the ignored `m_flGravity` write, and `sv_gravity 0` is load-bearing in B's CheckJumpButton stratum.

**4. The baseline residue gate is weaker than advertised.** It runs once, in the void chamber, all knobs zero, no input. A *partial* silencing failure — `sv_friction` took, `sv_accelerate` reverted — passes that gate trivially (no accel happens in the void with no buttons anyway) and then poisons the flat-floor Friction stratum. The gate must be **per-stratum and behavioral**: the Friction stratum ships a companion probe with full wishmove that must produce exactly zero accel, or the stratum is discarded.

**5. M1's pre/post memcmp cannot see intra-tick transients.** Fields written and restored inside the tick are invisible. `m_surfaceFriction` is observable here only because categorize happens to run last — luck, not design. M1 is sold as making ownership observable; it makes *end-state* observable.

**6. M1 will mine confident garbage without a null experiment.** Timers, curtime derivatives, prediction bookkeeping, touch counts, and any RNG state land in the changed set every single experiment, inflate the union mask, and correlate with everything during offline mining. B must run every experiment **twice with identical input** and permanently mask every offset that differs. It doesn't say this.

**7. "Offsets become tier-0" is only true for in-object state.** 0x1C50 is the player. The ground entity, the weapon (maxspeed), the move helper's touch list, and gpGlobals are reached through pointers and are not in the delta. Those stay tier-4 fragile.

**8. `.scap` "outlives the DLL" is true for parsing, false for semantics.** The field dictionary says offset 0x1868 is a float; it never says it's `m_surfaceFriction`. Post-patch, an old and a new capture are both parseable and mutually incomparable — and nothing in B *forces* the offline tool to refuse to merge them. Build identity is **recorded** but not **enforced**. Recording-without-enforcing is how the last generation of harnesses lied.

**9. The 300k-experiment sizing is unearned.** 25 µs/tick for SetupMove→ProcessMovement→FinishMove *plus* a 7248-byte memcmp *plus* two CMoveData dumps *plus* a 7248-byte restore, with real BSP traces on colliding probes, is optimistic by roughly an order of magnitude. The existing `RunFuzz` is already chunked and already instrumentable; nobody measured it. If the real rate is 250 µs you get ~30k, still a large win, but "two orders of magnitude" is the kind of number that gets budgeted against and then missed.

### Inputs B cannot set
Everything A can't, plus one whole category A has: **intra-function inputs**. You cannot ask "what does `AirAccelerate` do at wishspeed 43.7, vel 812, sfric 0.5" — you can only ask for a *player state* that produces it, and for many regimes no such state exists. `surface_friction` is a concrete example: the engine only ever emits `{1.0, 0.25}`, so B can never probe 0.5, and a model that's wrong in `sfric` linearly-but-correct at both endpoints stays green forever.

### Outputs B cannot observe
Intra-tick values (above). Anything outside the player object and CMoveData. **And function identity at all** — B's finest resolution is "the set of state fields whose forcing repairs the tick." `TryPlayerMove`, `StepMove`, and `StayOnGround` all write only pos/vel/on_ground: they are **one indistinguishable blob** to B, forever. B admits Categorize/CheckParameters/ReduceTimers; it does not admit this one, which is worse, because those three are small and these three are where the geometry-adjacent bugs live.

### Functions B cannot isolate
`CategorizePosition`, `CheckParameters`, `ReduceTimers` (admitted). `TryPlayerMove` / `StepMove` / `StayOnGround` (not admitted). `CheckVelocity` vs any other clamp on the same path. And the `FinishGravity`-inside-`CheckJumpButton` quirk, which `SolverMove.h` still carries as a **free toggle** (`jump_finishgravity`) — B measures only composed jump-tick vz, so two wrong things can cancel and the toggle is unfalsifiable.

### How a wrong result looks right
Convar write silently reverted → the stratum's residue quietly absorbs the un-silenced function → offline forcing attributes the miss to whichever field is downstream → you "fix" a *correct* function to compensate → board goes green. That is a compensating error pair, and **this project has already lived through one**: the affine stamina fit (slope 0.00019833, intercept 0.000327) was a chord across the 1.05-power curve (`SolverMove.h:80-95`) and measured green for a month until the disassembly retired it.

### B's strongest argument is correct, and stronger than B states
Three of the four green-board items (duck state machine, unconditional air FinishUnDuck, surf-friction ownership) are ordering/state-machine defects found by running the real pipeline. Zero would have been found by a per-function registry, and item 4 would have been **actively concealed** by one, since A's registry declares `surface_friction` an input to `Accelerate`.

---

# What I would build

**B's spine, with five mandatory corrections, plus exactly two things lifted from A. Not A, and not A-later-as-overlay on B's timeline.**

Why not A: its flagship leaf is inlined by its own admission, so coverage collapses at rung 0; its purity gate can't falsify the claim it exists to test; it encodes the project's most expensive finding as a premise; it has no rung that tests composition, which is where every recent defect lived; and its failure mode is a fabricated argument frame executed inside the live prediction pass while the user is mid-run. B's worst case is a bad row in a CSV.

**Mandatory before any code:**

1. **Force from tick N−1's post-dump** (or the {pre, post} cross product, reporting which repairs — that difference *is* the ownership signal). As written, B mis-attributes the exact class it claims to own.
2. **Per-stratum behavioral silencing gates**, not one void-chamber baseline. Each stratum carries a companion probe that can only pass if *that* stratum's silencings took. Never verify a convar write with `GetFloat()` — that's reading your own write back.
3. **Null experiment / determinism mask.** Every experiment twice, identical input; any differing offset is permanently excluded from mining and from forcing.
4. **Hard-fail, do not warn**, on: mixed build identity between capture and any other capture in the merge set, `gm_ctx_ok != 1`, any failed gate, any failed convar readback. Fail-closed, and refuse to merge, not just annotate.
5. **Measure µs/probe on the existing `RunFuzz` first** and size the plan from that number. Delete "300k / two orders of magnitude" until it's a measurement.

**Lift from A:** (i) the §2 `SolverMove` refactor with its byte-identical-CSV acceptance — free, correct, and it's what makes model-side ablation possible with zero engine calls; (ii) the §3a `gm-context-verify` / `EngineCtx` latch, which B depends on and A specifies properly.

**Reject from A:** the registry as a *contract* (unfalsifiable input-slot claims), the `double` shim, per-chunk guard hoisting, and RVA direct-call for v1 entirely. Revisit direct-call only for the three functions B admits it can't isolate, only under B's cross-check-or-discard rule — and note the win is smaller than it looks, because B's residue already *bounds* those three.

**Reject from A:** deleting `--dump-reach`. Kill the interaction classes; keep reachability.

**Fix regardless of design:** the divergence dump's "pre" line must print the engine's tick-(t−1) output, not the model's. And stop `break`-ing at first divergence — re-seed from the engine's row and keep going, which is the cheapest version of B's independence claim and is available today.

**The acceptance criterion neither design proposes, and the one I'd gate on:** inject a *known* defect into the model at a *known* location — `air_speed_cap` 30 → 30.0001, `stamina_pow_rate` 70 → 69.9, `time_to_unduck_ms` 200 → 201 — and require the harness to name the right function. A harness that cannot localize an intentional bug of known provenance is not measuring anything. Both designs assert attribution power; neither proposes to test it. Given that this project's stated failure history is *harnesses that silently lie*, that is the test that should gate either one.",
    "missing": {
      "functions": [
        {
          "name": "CGameMovement::CheckParameters",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::CheckParameters( void )",
          "inputs": "mv->m_flForwardMove/m_flSideMove/m_flUpMove, mv->m_flClientMaxSpeed, mv->m_flMaxSpeed, player->GetMoveType(), player->GetFlags() (FL_FROZEN, FL_ONTRAIN, FL_ATCONTROLS), player->pl.deadflag, player->m_Local.m_vecPunchAngle, mv->m_vecViewAngles",
          "outputs": "Rescales fmove/smove/umove by ratio = maxspeed/sqrt(f^2+s^2+u^2) when the 3D magnitude exceeds min(m_flClientMaxSpeed, m_flMaxSpeed); zeroes all three when frozen/on-train/dead; calls DecayPunchAngle(); writes mv->m_vecAngles = viewangles + punch with pitch clamped to [-90,90] and roll clamped",
          "purity": "mutates-member-state",
          "callable_directly": "no - reads mv/player members",
          "notes": "MISSING ENTIRELY from OURS (SolverMove.cpp MoveTick never normalizes the command). This is the ONLY place the raw command magnitude is capped, and it caps the 3D magnitude INCLUDING upmove, before any duck crop. A TAS that emits fmove=450/smove=450 (legal on the wire) gets silently rescaled by the engine to 250 total; our solver would accelerate on the uncapped wish. Also the reason a diagonal 400/400 command is not faster than 250 forward."
        },
        {
          "name": "CGameMovement::ReduceTimers",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::ReduceTimers( void )",
          "inputs": "gpGlobals->frametime, player->m_Local.m_flDucktime, m_flDuckJumpTime, m_flJumpTime, m_flSwimSoundTime, player->m_flStepSoundTime, CS: m_pCSPlayer->m_flStamina",
          "outputs": "Decrements each timer by frametime*1000 (m_flStepSoundTime by frametime*1000 too), clamped at 0",
          "purity": "mutates-member-state",
          "callable_directly": "yes if the timers are exposed",
          "notes": "OURS open-codes only the stamina and duck timers in MoveTick. m_flDuckJumpTime and m_flJumpTime are NOT modelled at all, and they gate FinishDuckJump / the duck-jump eye path and the CS duck-jump lockout. The exact ORDER (ReduceTimers runs BEFORE Duck() and before the movetype switch, i.e. the timer this tick's duck logic reads is already drained) is a one-tick offset that will show up in any duck-transition parity deck."
        },
        {
          "name": "CGameMovement::DecayPunchAngle",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::DecayPunchAngle( void )",
          "inputs": "player->m_Local.m_vecPunchAngle, m_vecPunchAngleVel, gpGlobals->frametime, PUNCH_DAMPING (9.0), PUNCH_SPRING_CONSTANT (65.0)",
          "outputs": "Spring-damper integration of m_vecPunchAngle and m_vecPunchAngleVel; zeroes both below the 0.001 threshold",
          "purity": "mutates-member-state",
          "callable_directly": "yes - pure math on two vectors",
          "notes": "Missing. Punch angle feeds mv->m_vecAngles (not m_vecViewAngles), so it does not rotate wishdir - but it DOES mean the angles written back by FinishMove and echoed to the client differ from the commanded angles. Any TAS that reads back engine viewangles to build the next command (closed-loop recording) must subtract punch or it drifts. Also matters for weapon-fire TASes."
        },
        {
          "name": "CGameMovement::CheckStuck + CheckInterval + GetRandomStuckOffsets/ResetStuckOffsets",
          "klass": "CGameMovement",
          "signature": "int CGameMovement::CheckStuck( void ); bool CheckInterval( IntervalType_t type ); void GetRandomStuckOffsets( int nIndex, int server, Vector &offset )",
          "inputs": "player origin/hull, PlayerSolidMask(), the static rgv3tStuckTable[54] offset table, m_StuckLast[MAX_PLAYERS] counters, player entity index, gpGlobals->tickcount, CHECKSTUCK_INTERVAL",
          "outputs": "On a solid start-position: TELEPORTS the player by walking the 54-entry offset table (scaled 0..2 units) until a free spot is found; sets mv->SetAbsOrigin(base+offset); returns nonzero to make PlayerMove EARLY-RETURN (the entire tick's movement is skipped)",
          "purity": "mutates-member-state",
          "callable_directly": "unknown - needs the per-player stuck counters",
          "notes": "Completely missing. Two hard fidelity consequences: (1) a stuck tick produces NO movement at all - PlayerMove returns before CategorizePosition, so ground state and velocity freeze; our TryPlayerMove instead zeroes velocity and continues. (2) the offset walk is indexed by ENTITY INDEX and a persistent per-player counter, so the unstuck displacement is history-dependent, not a pure function of position. Any surf ramp / corner-clip TAS that ever starts a tick solid diverges here."
        },
        {
          "name": "CGameMovement::CategorizeGroundSurface",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::CategorizeGroundSurface( trace_t &pm )",
          "inputs": "pm.surface.surfaceProps, physprops->GetSurfaceData(), surfacedata_t::game.friction, surfacedata_t::game.maxSpeedFactor(some builds)",
          "outputs": "player->m_surfaceProps, player->m_pSurfaceData, player->m_surfaceFriction = m_pSurfaceData->game.friction; then m_surfaceFriction *= 1.25f; if (>1) clamp to 1",
          "purity": "mutates-member-state",
          "callable_directly": "unknown - needs IPhysicsSurfaceProps and the texinfo->texdata->surfaceprop chain",
          "notes": "THE missing source of m_surfaceFriction. OURS only ever produces 1.0 or the measured 0.25 airborne-rising value. Real maps have ice (friction 0.1 -> 0.125), slime, dirt, metal (0.8 -> 1.0 after the 1.25 boost and clamp). Every Friction() and every Accelerate() scales by this, so a single ice brush silently invalidates the whole model. Requires BSP texinfo/texdata/TexdataStringTable parse plus the surfaceprops KV files from the VPK - none of which our world model reads."
        },
        {
          "name": "CGameMovement::PlayerSolidMask / GetCheckInterval / player collision group",
          "klass": "CGameMovement",
          "signature": "unsigned int CGameMovement::PlayerSolidMask( bool brushOnly = false )",
          "inputs": "brushOnly flag; CS override adds CONTENTS_PLAYERCLIP handling and teammate solidity (mp_solid_teammates)",
          "outputs": "MASK_PLAYERSOLID (CONTENTS_SOLID|MOVEABLE|PLAYERCLIP|WINDOW|MONSTER|GRATE) or MASK_PLAYERSOLID_BRUSHONLY",
          "purity": "pure",
          "callable_directly": "yes",
          "notes": "Missing as an explicit entry. Our BspWorld only honours SOLID|GRATE|PLAYERCLIP|LADDER (BspWorld.cpp:19-22). CONTENTS_MOVEABLE (brush entities), CONTENTS_WINDOW and CONTENTS_MONSTERCLIP are in MASK_PLAYERSOLID and are NOT in our solid set - a func_breakable window or a monsterclip volume blocks the real player and passes through ours. Also note StayOnGround/CategorizePosition use MASK_PLAYERSOLID_BRUSHONLY in some paths, so ground and movement can disagree about what is solid."
        },
        {
          "name": "CGameMovement::GetPointContentsCached / ResetGetPointContentsCache",
          "klass": "CGameMovement",
          "signature": "int CGameMovement::GetPointContentsCached( const Vector &point, int slot )",
          "inputs": "point, slot index, m_CachedGetPointContents[MAX_PLAYERS][MAX_PC_CACHE_SLOTS], m_CachedGetPointContentsPoint",
          "outputs": "Cached enginetrace->GetPointContents result; cache is invalidated once per ProcessMovement",
          "purity": "mutates-member-state",
          "callable_directly": "yes",
          "notes": "Missing. This is why water level can be STALE within a tick: CheckWater samples the same slot repeatedly and gets the first tick's answer even after the origin moved. Relevant the moment water is modelled; also a documented source of client/server prediction mismatch."
        },
        {
          "name": "CGameMovement::TracePlayerBBox / TryTouchGround",
          "klass": "CGameMovement",
          "signature": "void TracePlayerBBox( const Vector& start, const Vector& end, unsigned int fMask, int collisionGroup, trace_t& pm ); void TryTouchGround( const Vector& start, const Vector& end, const Vector& mins, const Vector& maxs, unsigned int fMask, int collisionGroup, trace_t& pm )",
          "inputs": "start/end, GetPlayerMins()/GetPlayerMaxs() (hull chosen by the DUCK FLAG, scaled by m_flModelScale), PlayerSolidMask(), COLLISION_GROUP_PLAYER_MOVEMENT, a CTraceFilterSimple wrapping the player",
          "outputs": "trace_t pm (fraction, plane, startsolid, allsolid, m_pEnt, surface)",
          "purity": "reads-member-state",
          "callable_directly": "yes (referenced in BINARY notes at RVA 0x11e520 but not inventoried as an entry)",
          "notes": "The hull used is GetPlayerMins/Maxs(m_bDucked)*m_flModelScale - our model's hull_state trick reproduces the MEASURED behaviour but not the mechanism, and m_flModelScale is entirely absent (any map/plugin that scales the player breaks us). TryTouchGround differs from TracePlayerBBox: it takes EXPLICIT mins/maxs, which CategorizePosition uses for the point-hull retry when the swept hull finds a non-walkable plane."
        },
        {
          "name": "CGameMovement::SetGroundEntity (base-velocity transfer half)",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::SetGroundEntity( trace_t *pm )",
          "inputs": "pm->m_pEnt (new ground), player->GetGroundEntity() (old), newGround->GetAbsVelocity(), player->GetBaseVelocity()",
          "outputs": "On !old && new: baseVelocity -= newGround->GetAbsVelocity(), baseVelocity.z = newGround->GetAbsVelocity().z. On old && !new: baseVelocity += oldGround->GetAbsVelocity(), .z = oldGround velocity z. Then SetBaseVelocity, SetGroundEntity, m_flFallVelocity=0, and mv->m_vecVelocity.z = 0",
          "purity": "mutates-member-state",
          "callable_directly": "no",
          "notes": "OURS models SetGroundEntity as 'set a brush index and zero vz'. The MOVING-PLATFORM VELOCITY TRANSFER is missing: stepping onto a moving func_train/elevator subtracts its velocity into basevel, and jumping OFF adds it back - which is exactly how you carry a platform's momentum into a jump. Any map with elevators, trains, or moving push brushes is unsimulatable without this."
        },
        {
          "name": "CGameMovement::CheckWater / GetWaterLevel / CheckWaterJump / WaterJump / WaterMove",
          "klass": "CGameMovement",
          "signature": "bool CheckWater(void); void WaterMove(void); void CheckWaterJump(void); void WaterJump(void)",
          "inputs": "GetPointContentsCached at origin+mins.z+1 / mid / eye level, CONTENTS_WATER|SLIME, player->GetWaterLevel()/SetWaterLevel, m_flWaterJumpTime, sv_gravity, WATER_FRICTION(1.0), water accel, mv->m_flUpMove, m_flSwimSoundTime",
          "outputs": "WaterLevel 0..3, m_flWaterJumpTime, MOVETYPE_WATERJUMP promotion, wholly different velocity integration (0.8*maxspeed wish, upmove-driven vertical, water friction, no ground friction, sinking at -60 when idle), and the water-exit velocity boost",
          "purity": "mutates-member-state",
          "callable_directly": "no",
          "notes": "Completely absent (SolverMove.h header literally says 'no water'). Water level ALSO changes non-water code: PlayerMove records m_nOldWaterLevel, FullWalkMove skips gravity when WL>WL_Waist, CheckJumpButton refuses/alters the jump in water, and friction is skipped. Any map with a pool, and every surf map with a water pit, is out of scope until this exists. upmove is unused in MoveTick precisely because water/ladder/noclip are missing."
        },
        {
          "name": "CGameMovement::LadderMove / FullLadderMove / OnLadder / GetLadderMove / SetLadder",
          "klass": "CGameMovement",
          "signature": "bool CGameMovement::LadderMove( void ); void CGameMovement::FullLadderMove( void )",
          "inputs": "CONTENTS_LADDER trace along m_vecForward, mv->m_flForwardMove/m_flSideMove/m_flUpMove, viewangles, IN_JUMP (dismount), IN_USE, player movetype (MOVETYPE_LADDER), LadderNormal(), player->GetLadderSurface()",
          "outputs": "Promotes/demotes MOVETYPE_LADDER, replaces the whole velocity with a ladder-basis wish (forward*cos + up*sin projected on the ladder plane), applies the dismount jump impulse, sets m_nOnLadder",
          "purity": "mutates-member-state",
          "callable_directly": "no",
          "notes": "Missing. Our BSP parse already tags CONTENTS_LADDER brushes (BspWorld.cpp:22) and the solver ignores them. CS:S also has func_ladder entities (not just CONTENTS_LADDER brushes) whose bounds come from the ENTITY lump - which we do not read. Ladders are a routing primitive on many maps; also the demotion path in PlayerMove (step 14) silently changes movetype mid-tick."
        },
        {
          "name": "CGameMovement::FullNoClipMove / FullObserverMove / FullTossMove",
          "klass": "CGameMovement",
          "signature": "void FullNoClipMove( float factor, float maxacceleration ); void FullObserverMove( void ); void FullTossMove( void )",
          "inputs": "sv_noclipspeed, sv_noclipaccelerate, sv_specspeed/sv_specaccelerate, mv->m_flUpMove, player observer mode, MOVETYPE_TOSS/FLY/FLYGRAVITY gravity + WATER_FRICTION",
          "outputs": "Whole-velocity replacement (noclip/observer) or ballistic integration with no ground contact (toss)",
          "purity": "mutates-member-state",
          "callable_directly": "no",
          "notes": "Missing. Not needed for a walk-mode TAS run, but REQUIRED for a stage-start / freecam / disconnected-start workflow (a named next feature) and for MOVETYPE_TOSS, which is what the player becomes after some triggers and during certain death/ragdoll transitions. Also FullNoClipMove is the only movetype whose tick does not touch CategorizePosition, so entering/leaving noclip mid-tape must be modelled or ground state is wrong on re-entry."
        },
        {
          "name": "CGameMovement::CanUnDuck / FinishUnDuck / FinishDuck / SetDuckedEyeOffset / FinishDuckJump / StartUnDuckJump",
          "klass": "CGameMovement",
          "signature": "bool CanUnduck( void ); void FinishUnDuck( void ); void FinishDuck( void ); void SetDuckedEyeOffset( float duckFraction )",
          "inputs": "player origin, GetPlayerMins/Maxs(false) standing hull, PlayerSolidMask(), FL_DUCKING, FL_ONGROUND, m_flDucktime, m_bDuckToggled, m_bInDuckJump, m_flDuckJumpTime, GAMEMOVEMENT_DUCK_TIME(1000)",
          "outputs": "CanUnduck runs a HULL TRACE at the would-be standing position (from origin, or origin shifted by the hull delta when airborne) and REFUSES the unduck if it startsolid; FinishDuck/UnDuck set/clear FL_DUCKING, m_bDucked, rebase the origin by the hull delta, and force a CategorizePosition",
          "purity": "mutates-member-state",
          "callable_directly": "no",
          "notes": "OURS collapses all of this into HandleDuck with a measured +/-8.5 air shift. CanUnduck's BLOCKING TRACE is the piece with no analogue: under a low ceiling the engine keeps you ducked (and keeps m_bDucking true, draining the timer forever), while our model unducks unconditionally. Every duck-through-gap and ceiling-clamped bhop route depends on this. The SolverMove.h comment even concedes 'only a ceiling map can test it'."
        },
        {
          "name": "CGameMovement::HandleDuckingSpeedCrop + m_iSpeedCropped (SPEED_CROPPED_DUCK / SPEED_CROPPED_WEAPON)",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::HandleDuckingSpeedCrop( void )",
          "inputs": "m_iSpeedCropped bitmask (reset to SPEED_CROPPED_RESET at the top of ProcessMovement), FL_DUCKING, FL_ONGROUND, mv->m_flForwardMove/m_flSideMove/m_flUpMove",
          "outputs": "If not already cropped and ducking and on ground: multiplies fmove/smove/umove by 0.33333333f and sets the SPEED_CROPPED_DUCK bit",
          "purity": "mutates-member-state",
          "callable_directly": "yes",
          "notes": "Missing as a function, and the semantics differ from OURS. (a) The engine crops the COMMAND (fmove/smove/umove) once per tick, guarded by a latch, not the maxspeed; OURS uses duck_speed_frac 0.34 as a cap. 1/3 = 0.333333 vs 0.34 is a 2% wishspeed error on every ducked ground tick. (b) The latch is shared with the WEAPON crop (scoped/zoomed weapons set SPEED_CROPPED_WEAPON first, which then SUPPRESSES the duck crop) - so a ducked scoped player is not double-cropped. (c) The crop only applies ON GROUND, unlike a maxspeed cap."
        },
        {
          "name": "CGameMovement::FinishMove",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::FinishMove( void )",
          "inputs": "mv->m_nButtons",
          "outputs": "mv->m_nOldButtons = mv->m_nButtons",
          "purity": "mutates-member-state",
          "callable_directly": "yes - one assignment",
          "notes": "Missing. This is the canonical end-of-tick old-buttons latch. OURS mutates s.old_buttons ad hoc inside CheckJumpButton and in the else-branch of MoveTick, which happens to reproduce the IN_JUMP edge but NOT the general contract: every other edge-triggered button (IN_DUCK for m_bDuckToggled, IN_USE, IN_ATTACK) reads mv->m_nOldButtons and would be wrong. Also distinct from CBasePlayer::UpdateButtonState's m_afButtonPressed/Released, which the inventory correctly separates."
        },
        {
          "name": "CPlayerMove::FinishMove (CMoveData -> player writeback)",
          "klass": "other",
          "signature": "void CPlayerMove::FinishMove( CBasePlayer *player, CUserCmd *ucmd, CMoveData *move )",
          "inputs": "CMoveData after ProcessMovement",
          "outputs": "player->m_flMaxspeed = move->m_flClientMaxSpeed; SetNetworkOrigin(move->GetAbsOrigin()); SetAbsVelocity(move->m_vecVelocity); SetLocalViewAngles/pl.v_angle = move->m_vecAngles; player->m_Local.m_nOldButtons = move->m_nOldButtons; m_outStepHeight/m_outWishVel/m_outJumpVel copied to the player for the client's step smoothing",
          "purity": "mutates-member-state",
          "callable_directly": "no",
          "notes": "The other half of the SetupMove marshalling boundary (vtable idx 19 in this project). Missing from the inventory even though SetupMove is listed. Matters because the writeback is where the ANGLES the engine reports back come from (post-punch, post-clamp), which is what a closed-loop recorder samples, and where m_outStepHeight feeds the client's view smoothing (a visual-only field but the thing a human TASer sees)."
        },
        {
          "name": "CGameMovement::CheckFalling / PlayerRoughLandingEffects",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::CheckFalling( void )",
          "inputs": "player->m_Local.m_flFallVelocity, PLAYER_FATAL_FALL_SPEED(1024), PLAYER_MAX_SAFE_FALL_SPEED(580), player ground entity, damage rules",
          "outputs": "Applies fall damage, plays the landing sound/roughness, ZEROES m_flFallVelocity, and in some branches applies the landing view punch",
          "purity": "mutates-member-state",
          "callable_directly": "no - needs the damage system",
          "notes": "Missing. Two live effects for a TAS: (1) fall damage can KILL the run (a route the solver finds may be lethal - the solver has no notion of survivable landing speed), and (2) on CS this is where any landing-side stamina/velocity modifier would be armed. Memory says landings arm nothing on this server (measured), which makes this low-risk for movement but NOT for run validity."
        },
        {
          "name": "CGameMovement::AddToTouched / IMoveHelper::ResetTouchList / CMoveHelperServer::ProcessImpacts",
          "klass": "other",
          "signature": "void CGameMovement::AddToTouched( const trace_t &tr, const Vector &impactvelocity ); void CMoveHelperServer::ProcessImpacts( void )",
          "inputs": "every trace_t from TryPlayerMove/StepMove that hit an entity, the impact velocity, the touchlist accumulated during the tick",
          "outputs": "After movement, replays the touch list: calls CBaseEntity::Touch on each entity with the player, then PhysicsTouchTriggers",
          "purity": "mutates-member-state",
          "callable_directly": "no - full entity system",
          "notes": "THE TRIGGER TIMING MECHANISM, and it is absent from the inventory (RunPreThink's note gestures at it but names no function). OURS calls w.CheckTriggers at the END of MoveTick using a point/hull containment test. The engine instead fires Touch from the MOVE TRACES plus a separate PhysicsTouchTriggers sweep - so an entity you passed THROUGH mid-tick (never occupying it at tick end) still touches, and a trigger you are standing in but did not move within may NOT re-touch. That difference is exactly what decides whether FL_BASEVELOCITY gets refreshed, which is what CheckMovingGround's decay depends on."
        },
        {
          "name": "CBaseTrigger::Touch / CTriggerPush::Touch",
          "klass": "other",
          "signature": "void CTriggerPush::Touch( CBaseEntity *pOther )",
          "inputs": "m_vecPushDir (from angles), m_flSpeed, pOther movetype (PUSH applies to VPHYSICS/WALK/STEP differently), pOther->GetFlags() & FL_ONGROUND, existing base velocity, SF_TRIG_PUSH_ONCE, m_flPushSpeed",
          "outputs": "For MOVETYPE_WALK: if push.z > 0 && FL_ONGROUND -> SetGroundEntity(NULL) and origin.z += 1; then SetBaseVelocity(vecPush [+ existing basevel]) and AddFlag(FL_BASEVELOCITY). For other movetypes: ApplyAbsVelocityImpulse",
          "purity": "mutates-member-state",
          "callable_directly": "no",
          "notes": "OURS reimplements the WALK branch inline in MoveTick (the unground + 1u nudge is there) but the FUNCTION and its gates are not inventoried, and three behaviours are missing: SF_TRIG_PUSH_ONCE (one-shot pushes that then Remove themselves), the m_flSpeed-vs-m_vecPushDir scaling from the entity lump (we hardcode a push_vec), and the fact that Touch fires ONCE PER TOUCH PASS - a player straddling two push triggers accumulates both. Also, because our model never re-touches, basevel_flag never decays and CheckMovingGround's (1 + dt/2) fold-in never happens."
        },
        {
          "name": "CTriggerGravity::StartTouch / EndTouch",
          "klass": "other",
          "signature": "void CTriggerGravity::StartTouch( CBaseEntity *pOther ); void CTriggerGravity::EndTouch( CBaseEntity *pOther )",
          "inputs": "trigger's gravity keyvalue, pOther->SetGravity()",
          "outputs": "StartTouch: pOther->SetGravity( m_flGravityScale ). EndTouch: pOther->SetGravity( 1.0 )",
          "purity": "mutates-member-state",
          "callable_directly": "no",
          "notes": "Missing as a function. OURS persists s.gravity_scale forever once a gravity trigger is touched (MoveTick: 'gravity scale persists'). The engine RESTORES 1.0 on EndTouch. Leaving a low-gravity volume therefore diverges permanently in our model - every subsequent tick integrates the wrong gravity. This is a correctness bug our inventory's own note documents as intentional."
        },
        {
          "name": "CTriggerTeleport::Touch",
          "klass": "other",
          "signature": "void CTriggerTeleport::Touch( CBaseEntity *pOther )",
          "inputs": "m_iLandmark target entity, SF_TELEPORT_PRESERVE_ANGLES, pOther origin/angles/velocity, the destination entity's origin+angles",
          "outputs": "With NO landmark: SetAbsOrigin(dest), SetAbsVelocity(vec3_origin), SnapEyeAngles(dest angles). WITH a landmark: origin and VELOCITY are transformed by the landmark delta (velocity PRESERVED and rotated), angles optionally preserved",
          "purity": "mutates-member-state",
          "callable_directly": "no",
          "notes": "OURS always zeroes velocity on teleport (MoveTick: 'teleport sets origin and ALWAYS zeroes velocity'). The landmark form - which is what surf/bhop map stage teleports and every 'momentum-preserving' teleporter use - preserves and ROTATES velocity. Getting this wrong makes every multi-stage map route invalid after the first teleport. Also SnapEyeAngles overrides the commanded viewangle for the next tick, changing wishdir."
        },
        {
          "name": "CFuncConveyor::GetGroundVelocityToApply / CBaseEntity::GetGroundVelocityToApply",
          "klass": "other",
          "signature": "virtual void CFuncConveyor::GetGroundVelocityToApply( Vector &vecGroundVel )",
          "inputs": "conveyor movedir, m_flConveyorSpeed, entity's m_flSpeed",
          "outputs": "The conveyor surface velocity handed to CheckMovingGround",
          "purity": "reads-member-state",
          "callable_directly": "no",
          "notes": "CheckMovingGround is inventoried but the virtual it calls is not - and that virtual is where the actual number comes from (and it is overridden per entity class). Needed for any map with conveyors or moving walkways, and it is the only other producer of base velocity besides trigger_push."
        },
        {
          "name": "CBaseEntity::PhysicsPushEntity / CPhysicsPushedEntities::SpeculativelyCheckPush (moving brush carrying the player)",
          "klass": "other",
          "signature": "void CPhysicsPushedEntities::MovePushedEntity(...); bool CBaseEntity::PhysicsPushMove( float movetime )",
          "inputs": "the pushing entity's velocity/rotation, the list of entities standing on it (ground-entity backlinks), player hull, blocking traces",
          "outputs": "Directly SETS the player's absolute origin (and rotates it about the pusher's origin for rotating brushes) OUTSIDE of CGameMovement; sets m_bGameCodeMovedPlayer",
          "purity": "mutates-member-state",
          "callable_directly": "no",
          "notes": "Missing entirely, and it is the ONLY player-position mutation that happens outside the movement stack. It is also the thing that sets m_bGameCodeMovedPlayer, which (per the inventory's own PlayerMove note) FORCES the opening CategorizePosition back on even with sv_optimizedmovement 1 - so a tick after a platform push has a structurally different call order. Elevators, func_door platforms, and rotating brushes are unmodellable without it."
        },
        {
          "name": "CBasePlayer::PhysicsSimulate / CBasePlayer::ProcessUsercmds (the command loop above RunCommand)",
          "klass": "CBasePlayer",
          "signature": "void CBasePlayer::PhysicsSimulate( void ); void CBasePlayer::ProcessUsercmds( CUserCmd *cmds, int numcmds, int totalcmds, int dropped_packets, bool paused )",
          "inputs": "m_nTickBase, gpGlobals->tickcount, the command queue, sv_maxusrcmdprocessticks (default 8), sv_maxusrcmdprocessticks_holdaim, sv_clockcorrection_msecs, dropped_packets, paused",
          "outputs": "Decides HOW MANY CUserCmds execute this server tick (0, 1, or many), advances m_nTickBase per command, synthesizes filler commands for dropped packets, and clamps the command backlog",
          "purity": "mutates-member-state",
          "callable_directly": "no",
          "notes": "Missing. The inventory starts at RunCommand and therefore assumes exactly one command per tick. That assumption is what a TAS depends on, but it is only true when the client feeds commands at exactly tickrate with no loss - and sv_maxusrcmdprocessticks lets up to 8 commands run BACK TO BACK inside one server frame (all with frametime = TICK_INTERVAL, so the math per command is unchanged, but the WALL-CLOCK to tick mapping is not 1:1). Any hardware-input tool or laggy replay hits this. It is also where m_nTickBase - the thing RunCommand turns into curtime - is actually advanced."
        },
        {
          "name": "CUserCmd::WriteUserCmd / ReadUserCmd (wire quantization) + CUserCmd::Reset",
          "klass": "other",
          "signature": "void WriteUserCmd( bf_write *buf, CUserCmd *to, CUserCmd *from ); void ReadUserCmd( bf_read *buf, CUserCmd *move, CUserCmd *from )",
          "inputs": "viewangles (3x float32, delta-coded against the previous command), forwardmove/sidemove/upmove (float32), buttons (32 bits), impulse (8), weaponselect (11)/weaponsubtype(6), mousedx/mousedy (SHORT - 16-bit signed), tick_count, random_seed",
          "outputs": "The exact bit representation the server decodes",
          "purity": "pure",
          "callable_directly": "yes",
          "notes": "Missing, and it bounds the TAS's input alphabet. viewangles and the move values survive as full float32 (so arbitrary-precision yaw IS achievable), but mousedx/dy are 16-bit shorts, and the server-side random_seed is what StartCommand feeds RandomSeed(). Worth pinning explicitly rather than assuming: if any field were quantized, the solver's continuous yaw optimization would be searching angles the engine cannot represent."
        },
        {
          "name": "CInput::CreateMove / AdjustAngles / ScaleMovements / ClampAngles",
          "klass": "other",
          "signature": "void CInput::CreateMove( int sequence_number, float input_sample_frametime, bool active ); void CInput::ClampAngles( QAngle& viewangles )",
          "inputs": "cl_forwardspeed(400)/cl_sidespeed(400)/cl_upspeed(320), cl_yawspeed/cl_pitchspeed for keyboard turning, m_pitch/sensitivity, m_flFrameTime, the raw key state, cl_pitchdown(89)/cl_pitchup(89)",
          "outputs": "The fmove/smove/umove magnitudes and the viewangle deltas that end up in CUserCmd; clamps pitch to [-89,+89] and normalizes yaw to [-180,180]",
          "purity": "mutates-member-state",
          "callable_directly": "no - client input system",
          "notes": "Missing. Defines the LEGAL INPUT SPACE for the solver: the yaw must be delivered as an absolute angle in [-180,180] (a solver that emits 361 degrees gets a different wishdir than it planned once the engine normalizes), pitch outside +/-89 is unreachable, and the move magnitudes a real client can produce are quantized to combinations of +/-cl_forwardspeed/cl_sidespeed unless the TAS writes the CUserCmd directly. Relevant because this project injects commands - it must decide which layer it is impersonating."
        },
        {
          "name": "CBasePlayer::SetLocalViewAngles / CCSPlayer::SnapEyeAngles / m_vecViewOffset",
          "klass": "CBasePlayer",
          "signature": "void CBasePlayer::SetLocalViewAngles( const QAngle &viewAngles ); void CCSPlayer::SnapEyeAngles( const QAngle &viewAngles )",
          "inputs": "pl.v_angle, pl.fixangle (FIXANGLE_ABSOLUTE/RELATIVE), m_angEyeAngles",
          "outputs": "Overrides the player's view angles server-side and sets fixangle so the client snaps rather than interpolating",
          "purity": "mutates-member-state",
          "callable_directly": "no",
          "notes": "Missing. Fired by teleports, spawn, and vehicle entry. For a TAS the consequence is concrete: the tick AFTER a snap, the command's viewangles were composed against the OLD angles by the client, so a replayed tape's wishdir is wrong for one tick unless the snap is modelled."
        },
        {
          "name": "CCSPlayer::GetPlayerMaxSpeed / CWeaponCSBase::GetMaxSpeed / m_flVelocityModifier",
          "klass": "CBasePlayer",
          "signature": "float CCSPlayer::GetPlayerMaxSpeed( void ); float CWeaponCSBase::GetMaxSpeed( void )",
          "inputs": "active weapon's CCSWeaponInfo::m_flMaxPlayerSpeed (knife 250, rifles 235/230, AWP 210 / 150 when zoomed, C4 250), m_bIsWalking (+shift = 0.52x), m_bIsDefusing, m_bIsScoped, m_flVelocityModifier (set when SHOT: velocity scaled and recovered over time), FL_DUCKING",
          "outputs": "mv->m_flMaxSpeed and mv->m_flClientMaxSpeed used by CheckParameters and by WalkMove's wishspeed clamp",
          "purity": "reads-member-state",
          "callable_directly": "unknown - needs the weapon system",
          "notes": "The inventory lists m_flMaxspeed as an INPUT everywhere but never names what produces it. OURS hardcodes MoveParams::maxspeed 250 (knife). Weapon switch mid-run, zooming, walk-key, and being shot (m_flVelocityModifier multiplies wishspeed in CCSGameMovement::WalkMove and decays back to 1 over time) all change the cap without any movement-code involvement. A TAS that plans a 250 route and executes it holding a rifle is simply wrong."
        },
        {
          "name": "CPrediction::RunCommand / CPrediction::Update / CBasePlayer::AvoidPhysicsProps",
          "klass": "other",
          "signature": "void CPrediction::RunCommand( C_BasePlayer *player, CUserCmd *ucmd, IMoveHelper *moveHelper )",
          "inputs": "the same CGameMovement instance but with prediction-only entities, cl_predict, the prediction copy tables, m_bGameCodeMovedPlayer, the client's own gpGlobals",
          "outputs": "Client-side predicted origin/velocity, which is what an in-game observer (and any client-side recorder) SEES - not necessarily what the server computed",
          "purity": "mutates-member-state",
          "callable_directly": "no",
          "notes": "Missing, and it is the single biggest source of 'my recording does not match my replay'. The client predicts with a DIFFERENT entity set (no server-only entities, different trigger touch results), so predicted and authoritative positions can differ by units on trigger-heavy geometry, and the server silently corrects. Any parity deck must state which side it measured. Also CBasePlayer::AvoidPhysicsProps directly perturbs the command (it modifies fmove/smove to push you around props) on the CLIENT only."
        },
        {
          "name": "CEngineTrace::TraceRay / CTraceFilterSimple::ShouldHitEntity / CGameRules::ShouldCollide",
          "klass": "IEngineTrace",
          "signature": "void CEngineTrace::TraceRay( const Ray_t &ray, unsigned int fMask, ITraceFilter *pTraceFilter, trace_t *pTrace )",
          "inputs": "Ray_t (start, delta, extents, m_IsSwept/m_IsRay), content mask, the filter's collision group, entity solid flags, g_pGameRules->ShouldCollide(collisionGroup0, collisionGroup1)",
          "outputs": "A trace_t that is the MINIMUM over the world BSP clip and every entity the spatial partition enumerates",
          "purity": "reads-member-state",
          "callable_directly": "yes via the engine interface",
          "notes": "Missing. Our TraceHull3 is world-brushes-only (SolverMove.cpp TryPlayerMove note admits 'no entity traces'). COLLISION_GROUP_PLAYER_MOVEMENT plus the gamerules callback is what makes teammates, prop_physics, and debris non-solid or solid - and CS's mp_solid_teammates flips it mid-round. Also Ray_t::m_IsSwept/m_IsRay pick different code paths (a zero-extent ray uses ClipRayToBSP, a box uses the brush path), which is why the 'point-hull retry' in CategorizePosition behaves differently from the swept hull."
        },
        {
          "name": "CM_ClipBoxToBrush",
          "klass": "IEngineTrace",
          "signature": "static void CM_ClipBoxToBrush( TraceInfo_t *pTraceInfo, const cbrush_t *brush )",
          "inputs": "brush sides (planes + bevels), the box support corner (dist offset = fabs(mins.x*nx)+... for the signed normal), DIST_EPSILON (0.03125), enterfrac init -1, leavefrac init 1, getout/startout flags",
          "outputs": "trace fraction, plane, startsolid, allsolid; enterfrac is CLAMPED to >= 0",
          "purity": "pure",
          "callable_directly": "no - engine internal",
          "notes": "Not inventoried anywhere, yet it is the exact function OURS is mirroring in World::TraceHull3, and it is the origin of the measured surf-ramp allsolid behaviour the solver hardcodes. Three details our mirror must match bit-for-bit: (1) DIST_EPSILON 0.03125 is subtracted from the enter distance (an eighth of a unit of slop that decides whether a ramp entry is a hit), (2) the AXIAL fast path (plane->type < 3) computes the box offset differently from the general dot-product path, giving different float rounding on axis-aligned faces, and (3) the startsolid/allsolid distinction (startout vs getout) is what our 'velocity ZEROED' rule keys off."
        },
        {
          "name": "CM_RecursiveHullCheck / CM_BoxTrace / leaf+brush checkcount dedup",
          "klass": "IEngineTrace",
          "signature": "void CM_RecursiveHullCheck( TraceInfo_t *pTraceInfo, int num, const float p1f, const float p2f, const Vector& p1, const Vector& p2 )",
          "inputs": "BSP nodes/planes (LUMP_NODES, LUMP_PLANES), the box extents (offset per node plane), DIST_EPSILON, leaf contents, the per-trace checkcount stamp on each brush",
          "outputs": "The ordered set of leaves (and therefore brushes) actually tested, each brush tested AT MOST ONCE per trace",
          "purity": "reads-member-state",
          "callable_directly": "no",
          "notes": "Missing, and it is a structural divergence: OURS uses a uniform grid over brush AABBs (BspWorld.cpp) to choose candidates. The engine uses BSP node traversal with a per-node epsilon offset. Two consequences: (1) a brush whose leaf the ray never enters is NEVER tested even if its AABB overlaps - the grid tests it and can produce a hit the engine does not, (2) the node-descent epsilon can terminate a trace early inside thin geometry. Any residual parity failure on complex geometry should be suspected here first."
        },
        {
          "name": "CM_TransformedBoxTrace (rotated/translated brush entities)",
          "klass": "IEngineTrace",
          "signature": "void CM_TransformedBoxTrace( const Ray_t& ray, int headnode, int brushmask, const Vector& origin, QAngle const& angles, trace_t& tr )",
          "inputs": "the entity's model headnode, its origin and angles, the world-space ray",
          "outputs": "Trace performed in the entity's LOCAL space, with the resulting plane normal rotated back to world space",
          "purity": "pure",
          "callable_directly": "no",
          "notes": "Missing. Every func_ brush entity (doors, movelinear, rotating, breakable) collides through this, not through the world tree. Our world model folds only model 0 (BspWorld.cpp:411 collects leaves of the world model) so brush entities are absent both as geometry and as movable geometry. On many surf/bhop maps the boost pads, gates, and stage barriers are brush entities."
        },
        {
          "name": "CDispCollTree::AABBTree_SweepAABBTriangles / AABBTree_Ray (displacement collision)",
          "klass": "IEngineTrace",
          "signature": "bool CDispCollTree::AABBTree_SweepAABBTriangles( CBaseTrace *pTrace, ... )",
          "inputs": "LUMP_DISPINFO(26), LUMP_DISP_VERTS(33), LUMP_DISP_TRIS(48), LUMP_PHYSDISP(28), the displacement power/subdiv, per-triangle tags (DISPTRI_TAG_WALKABLE, SURF_NOPHYSICS), the swept box",
          "outputs": "Triangle-level trace results with plane normals taken from the DISPLACEMENT TRIANGLES, not the parent brush face",
          "purity": "pure",
          "callable_directly": "no",
          "notes": "THE LARGEST WORLD-MODEL GAP. Our BSP parse reads brush sides only and drops dispinfo (BspWorld.cpp's dbrushside_t has a dispinfo field that is parsed but, from the clip_planes/bevel_planes construction, not acted on). A displacement REPLACES its parent brush face for collision: the brush plane is not what you slide on. Any map whose ramps, terrain, or slopes are displacements - extremely common - will produce a physically wrong route with no warning, because the brush-plane approximation still yields plausible-looking traces. Displacement triangles also give per-triangle normals, so surf on a displacement has a continuously varying plane our model cannot express."
        },
        {
          "name": "CStaticPropMgr::TraceRayAgainstStaticProp / static prop vcollide",
          "klass": "IEngineTrace",
          "signature": "void CStaticPropMgr::TraceRayAgainstStaticProp( const Ray_t& ray, int staticProp, trace_t& tr )",
          "inputs": "the sprp game lump (lump 35) prop list, each prop's model, its solid type (SOLID_VPHYSICS / SOLID_BBOX / SOLID_NONE), the model's CPhysCollide from the phy file",
          "outputs": "trace_t against the prop's collision model",
          "purity": "pure",
          "callable_directly": "no",
          "notes": "Missing. prop_static with SOLID_VPHYSICS blocks the player and is enumerated by the spatial partition on every player trace. Our world model has no props at all, so any route that clips a crate, pillar, or rock is invalid. Needs the sprp game lump plus the .phy collision data out of the VPK - a much heavier lift than the brush parse, which is why it should be an explicit KNOWN gap rather than an unnoticed one."
        },
        {
          "name": "CPhysicsCollision::TraceBox (vphysics ledge collide)",
          "klass": "IEngineTrace",
          "signature": "void CPhysicsCollision::TraceBox( const Ray_t &ray, unsigned int contentsMask, IConvexInfo *pConvexInfo, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr )",
          "inputs": "IVP ledge tree from the model's .phy / the map's LUMP_PHYSCOLLIDE(29), the swept box",
          "outputs": "trace_t against convex ledges",
          "purity": "pure",
          "callable_directly": "no",
          "notes": "Missing. This is the second collision representation in the engine (the first being BSP brushes). Solid entities with SOLID_VPHYSICS - most props, some brush entities - use it, and its results do NOT match the brush representation exactly. Also worth noting for completeness: LUMP_PHYSCOLLIDE holds the world's vphysics version of the same brushes, used for prop-vs-world, not player-vs-world."
        },
        {
          "name": "SpatialPartition enumeration order / trace_t entity tie-break",
          "klass": "IEngineTrace",
          "signature": "void ISpatialPartition::EnumerateElementsAlongRay( SpatialPartitionListMask_t, const Ray_t&, bool, IPartitionEnumerator* )",
          "inputs": "the voxel hash occupancy, entity insertion order, the partition list mask (PARTITION_ENGINE_SOLID_EDICTS etc.)",
          "outputs": "The ORDER entities are tested, hence which entity ends up in trace_t::m_pEnt when two surfaces are hit at the same fraction",
          "purity": "reads-member-state",
          "callable_directly": "no",
          "notes": "Missing. Matters for ground selection: when the ground probe hits two coplanar entities/brushes at fraction 0, which one becomes the ground entity decides whether you inherit a moving platform's velocity. Our model stores a single ground_brush index with a grid-iteration order that has no relationship to the engine's."
        },
        {
          "name": "BSP LUMP_DISPINFO(26) / LUMP_DISP_VERTS(33) / LUMP_DISP_TRIS(48) / LUMP_PHYSDISP(28) parse",
          "klass": "BSP-parse",
          "signature": "ddispinfo_t { Vector startPosition; int DispVertStart, DispTriStart; int power, minTess; float smoothingAngle; int contents; unsigned short MapFace; ... }",
          "inputs": "the four displacement lumps plus the parent face (LUMP_FACES) and its four corner verts (LUMP_VERTS/LUMP_SURFEDGES/LUMP_EDGES)",
          "outputs": "A (2^power+1)^2 vertex grid per displacement and its triangle tags, from which the collision triangles are built",
          "purity": "pure",
          "callable_directly": "yes - pure file parse",
          "notes": "Required by the displacement-collision entry above and not present in our parser (BspWorld.cpp:64-105 defines dlump_t/dheader_t/dbrushside_t/dleaf_t only). Note the parent face must be resolved through the surfedge indirection to get the four corners in the right winding, otherwise the whole surface is transposed - a classic silent failure."
        },
        {
          "name": "BSP LUMP_MODELS(14) + LUMP_ENTITIES(0) parse",
          "klass": "BSP-parse",
          "signature": "dmodel_t { Vector mins, maxs, origin; int headnode, firstface, numfaces; }; entity lump = a KeyValues text blob",
          "inputs": "lump 14 model table, lump 0 entity text",
          "outputs": "Per-brush-entity headnode/bounds, and every entity's classname/origin/angles/targetname/spawnflags plus type-specific keys (trigger_push m_flSpeed+pushdir, trigger_gravity gravity, trigger_teleport target, func_ladder bounds, info_player_start, func_door movement)",
          "purity": "pure",
          "callable_directly": "yes",
          "notes": "OURS deliberately parses only model 0 (BspWorld.cpp:411) and hardcodes trigger data into World::TriggerHitS from elsewhere. Consequences: (a) trigger volumes and their PARAMETERS must currently be hand-fed rather than read from the map, (b) brush entities are missing as geometry (see CM_TransformedBoxTrace), (c) the note at BspWorld.cpp:49 - that trigger slabs are CONTENTS_SOLID on disk but block nothing - is only correct BECAUSE brush-entity models are excluded; the moment models > 0 are added, the entity lump is what distinguishes a solid func_brush from a non-solid trigger. The two lumps must be parsed together."
        },
        {
          "name": "BSP contents-mask completeness (CONTENTS_MOVEABLE/WINDOW/MONSTERCLIP/WATER/SLIME) and dbrush_t contents vs texinfo flags",
          "klass": "BSP-parse",
          "signature": "dbrush_t { int firstside, numsides, contents; }; texinfo_t.flags (SURF_*)",
          "inputs": "LUMP_BRUSHES(18) contents field, LUMP_TEXINFO(6) flags, LUMP_TEXDATA(2) + TexdataStringTable/Data for the material name (and thus the surfaceprop)",
          "outputs": "Which brushes are in MASK_PLAYERSOLID; which surface each hit reports for surfaceprops friction",
          "purity": "pure",
          "callable_directly": "yes",
          "notes": "Our parser recognizes 4 contents bits (BspWorld.cpp:19-22). Missing from the solid set: CONTENTS_MOVEABLE (0x4000), CONTENTS_WINDOW (0x2), CONTENTS_MONSTER (0x2000000) - all in MASK_PLAYERSOLID. Missing entirely: CONTENTS_WATER/SLIME (needed for water levels) and the texinfo->texdata->material chain that is the ONLY route to surfaceprops friction. Also SURF_NODRAW/SURF_SKY brushes still collide - visual-flag filtering must not touch the collision set."
        },
        {
          "name": "BSP plane lump type field + the axial fast path",
          "klass": "BSP-parse",
          "signature": "dplane_t { Vector normal; float dist; int type; }",
          "inputs": "LUMP_PLANES(1)",
          "outputs": "normal, dist, and type (0/1/2 = axis-aligned X/Y/Z, 3+ = general)",
          "purity": "pure",
          "callable_directly": "yes",
          "notes": "Missing as a called-out detail. The engine branches on plane->type in both CM_ClipBoxToBrush and CM_RecursiveHullCheck, computing the box offset as a direct component pick rather than a three-term dot product. Same value mathematically, DIFFERENT float rounding. On axis-aligned floors and walls - i.e. most of a map - our general-path dot product can land on the other side of DIST_EPSILON. If a parity deck ever shows sub-0.03 discrepancies only on axial surfaces, this is why."
        },
        {
          "name": "AngleVectors / SinCos / DEG2RAD / VectorNormalize float determinism",
          "klass": "other",
          "signature": "void AngleVectors( const QAngle &angles, Vector *forward, Vector *right, Vector *up ); void SinCos( float radians, float *sine, float *cosine ); float VectorNormalize( Vector& v )",
          "inputs": "angles in degrees, M_PI_F/180.f, the platform sincos implementation, the FPU/SSE control word (rounding mode, FTZ/DAZ), compiler /fp mode",
          "outputs": "forward/right/up basis; normalized vector and its prior length",
          "purity": "pure",
          "callable_directly": "yes",
          "notes": "Missing, and it is the floor on achievable parity. Our WishFromInput derives the basis from yaw alone with its own cos/sin - correct in exact arithmetic, but the engine computes forward/right/up from ALL THREE angles (pitch and roll included) and then zeroes z and renormalizes, so the intermediate rounding differs. Combined with the x87-vs-SSE difference between the old 32-bit build and this x64 port, and with FTZ/DAZ state, the last 1-2 ULP of every wishdir is unpinned. For a 10000-tick tape that compounds. Worth an explicit bit-exactness experiment rather than an assumption."
        },
        {
          "name": "TICK_INTERVAL / gpGlobals->frametime edge cases: host_timescale, host_framerate, sv_alternateticks, pause, and the interval_per_tick setting",
          "klass": "other",
          "signature": "#define TICK_INTERVAL ( gpGlobals->interval_per_tick ); float host_timescale; sv_alternateticks",
          "inputs": "the -tickrate launch parameter / interval_per_tick, host_timescale, host_framerate (fixed-timestep demo mode), sv_alternateticks, the paused flag, player->GetLaggedMovementValue()",
          "outputs": "The dt every movement function sees",
          "purity": "reads-member-state",
          "callable_directly": "yes",
          "notes": "Partly covered (RunCommand and the lagged-movement scaling are inventoried) but three edge cases are not. (1) sv_alternateticks makes the server simulate at HALF rate with DOUBLE frametime - the whole tick becomes a different integration and every measured constant fitted at 0.015 shifts (our stamina pow already scales with dt, but the baked jump impulse and the measured 8.5 duck shift may not). (2) host_timescale scales real time, not frametime, so it changes the command RATE not the per-tick math - which is exactly the confusion that produces bad captures. (3) On pause frametime is 0, and a 0-dt tick still runs the full pipeline (friction with dt 0, CategorizePosition, etc.) rather than being skipped. Our MoveParams::dt is a single float with none of this."
        },
        {
          "name": "CGameMovement::StepMove's second half / m_outStepHeight and the 'DEAD BODY' / step-up-vs-slope arbitration",
          "klass": "CGameMovement",
          "signature": "void CGameMovement::StepMove( Vector &vecDestination, trace_t &trace )",
          "inputs": "the down-trace plane normal, mv->m_outStepHeight, the up/forward/down triple trace, the comparison of downdist vs updist",
          "outputs": "Chooses between the flat-move result and the stepped result by comparing SQUARED XY DISTANCE travelled, writes mv->m_outStepHeight += (final z - original z)",
          "purity": "mutates-member-state",
          "callable_directly": "no",
          "notes": "OURS has StepMove (SolverMove.cpp:283) so the function exists, but two engine details are absent from the inventory description and should be verified: (a) the engine REFUSES the step result when the down-trace plane is not walkable (normal.z < 0.7) and falls back to the flat move - which is what stops you from step-climbing a surf ramp, and (b) m_outStepHeight is what the CLIENT uses to smooth the view; it is not gameplay state but it is the only observable that distinguishes a step from a slope in a recording. If our StepMove picks the stepped result on a steep plane, ramp routes gain free height."
        }
      ],
      "sources": [
        "C:\\Users\\Connor\\Documents\\SourceTAS\\sourceTAS\\Source\\Solver\\SolverMove.h",
        "C:\\Users\\Connor\\Documents\\SourceTAS\\sourceTAS\\Source\\Solver\\SolverMove.cpp",
        "C:\\Users\\Connor\\Documents\\SourceTAS\\sourceTAS\\Source\\Solver\\SolverWorld.h",
        "C:\\Users\\Connor\\Documents\\SourceTAS\\sourceTAS\\Source\\World\\BspWorld.cpp",
        "C:\\Users\\Connor\\Documents\\SourceTAS\\sourceTAS\\Source\\World\\BspWorld.h",
        "Source SDK 2013 game/shared/gamemovement.cpp, game/shared/cstrike/cs_gamemovement.cpp, game/server/player_command.cpp, engine cmodel_bsp/cmodel collision (recollection)"
      ]
    }
  },
  "workflowProgress": [
    {
      "type": "workflow_phase",
      "index": 1,
      "title": "Research"
    },
    {
      "type": "workflow_phase",
      "index": 2,
      "title": "Design"
    },
    {
      "type": "workflow_phase",
      "index": 3,
      "title": "Critique"
    },
    {
      "type": "workflow_phase",
      "index": 4,
      "title": "Synthesize"
    },
    {
      "type": "workflow_agent",
      "index": 1,
      "label": "wiki:movement",
      "phaseIndex": 1,
      "phaseTitle": "Research",
      "agentId": "a3cf1f36f47bc0008",
      "model": "claude-opus-5",
      "state": "done",
      "startedAt": 1786799597929,
      "queuedAt": 1786799591954,
      "attempt": 1,
      "lastToolName": "StructuredOutput",
      "promptPreview": "Research the Valve Developer Community wiki (developer.valvesoftware.com) for Counter-Strike: Source / Source SDK 2013 PLAYER MOVEMENT.

Find and read the relevant pages, e.g.: "Player Movement", "CGameMovement", "Movement variables"/"List of CS:S Cvars", "Dimensions (Half-Life 2 and Counter-Strike: Source)", "Trace", "IEngineTrace", "CMoveData", "CUserCmd", "BSP file format" (brush/brushside/node…",
      "lastProgressAt": 1786800983410,
      "tokens": 142943,
      "toolCalls": 28,
      "durationMs": 1385480,
      "resultPreview": "{"functions":[{"name":"CPlayerMove::RunCommand","klass":"other","signature":"void CPlayerMove::RunCommand( CBasePlayer *player, CUserCmd *ucmd, IMoveHelper *moveHelper )","inputs":"CUserCmd (viewangles, forwardmove, sidemove, upmove, buttons, impulse, tick_count, weaponselect, mousedx/dy); player->m_nTickBase; TICK_INTERVAL; player->m_bGamePaused; player->GetVehicle()","outputs":"Sets gpGlobals->c…"
    },
    {
      "type": "workflow_agent",
      "index": 2,
      "label": "re:vtable",
      "phaseIndex": 1,
      "phaseTitle": "Research",
      "agentId": "aebec0bea09cded9d",
      "model": "claude-opus-5",
      "state": "done",
      "startedAt": 1786799604002,
      "queuedAt": 1786799591954,
      "attempt": 1,
      "lastToolName": "StructuredOutput",
      "promptPreview": "Binary RE task. Target: C:\\Program Files (x86)\\Steam\\steamapps\\common\\Counter-Strike Source\\cstrike\\bin\\x64\\client.dll (x64 CS:S client, contains CGameMovement/CCSGameMovement compiled in).

capstone 5.0.7 and python are installed. Reusable scripts already exist in C:\\Users\\Connor\\AppData\\Local\\Temp\\claude\\C--Users-Connor-Documents-SourceTAS-sourceTAS\\f654ec71-3563-47ff-a5ed-3d9fef29e216\\scratchpa…",
      "lastProgressAt": 1786801521852,
      "tokens": 218001,
      "toolCalls": 56,
      "durationMs": 1917850,
      "resultPreview": "{"functions":[{"name":"CGameMovement::~CGameMovement (scalar deleting dtor)","klass":"CGameMovement","signature":"void* __fastcall(CGameMovement* rcx, unsigned char flags dl)","inputs":"this; dl bit0 = delete flag","outputs":"writes this->vfptr = &IGameMovement_vtable (0x438300); optionally frees","purity":"mutates-member-state","callable_directly":"yes - direct RVA call, but do not call (destroys…"
    },
    {
      "type": "workflow_agent",
      "index": 3,
      "label": "ours:inventory",
      "phaseIndex": 1,
      "phaseTitle": "Research",
      "agentId": "ac9c8eb53e25ca3e2",
      "model": "claude-opus-5",
      "state": "done",
      "startedAt": 1786799596818,
      "queuedAt": 1786799591955,
      "attempt": 1,
      "lastToolName": "StructuredOutput",
      "promptPreview": "Codebase inventory. Repo: C:\\Users\\Connor\\Documents\\SourceTAS\\sourceTAS.

Read Source/Solver/SolverMove.cpp, SolverMove.h, SolverWorld.cpp, SolverWorld.h, and Source/World/Prediction.cpp, Prediction.h, BspWorld.cpp.

List EVERY function in our solver core that is intended to mirror an engine function. For each: our function name + file:line, the engine function it mirrors, its exact inputs (parame…",
      "lastProgressAt": 1786800068503,
      "tokens": 132652,
      "toolCalls": 23,
      "durationMs": 471685,
      "resultPreview": "{"functions":[{"name":"EngineClipVelocity","klass":"CGameMovement","signature":"void EngineClipVelocity(const Vec3& in, const Vec3& n, Vec3& out)","inputs":"Source/Solver/SolverMove.cpp:10. Params only: in (velocity), n (plane normal). Reads NO PlayerState/MoveParams/World field. Overbounce is hardcoded 1.0 (not a param).","outputs":"out (Vec3). Writes nothing else.","purity":"pure","callable_dire…"
    },
    {
      "type": "workflow_agent",
      "index": 4,
      "label": "harness:mechanism",
      "phaseIndex": 1,
      "phaseTitle": "Research",
      "agentId": "a57017cc97da3ae90",
      "model": "claude-opus-5",
      "state": "done",
      "startedAt": 1786799597111,
      "queuedAt": 1786799591955,
      "attempt": 1,
      "lastToolName": "StructuredOutput",
      "promptPreview": "Read C:\\Users\\Connor\\Documents\\SourceTAS\\sourceTAS/Source/World/Prediction.cpp carefully, especially: BeginStateGuard/EndStateGuard, RunOneFuzzProbeSEH, RunEditorSimSEH, OnFinishMove, the g_off netvar offsets, kMoveDataBufSize/kMoveDataOriginOff/kMoveDataVelOff, CallV<Idx>, and how SetupMove/ProcessMovement/FinishMove are invoked.

Then answer, concretely and with code sketches:
1. Given we have g…",
      "lastProgressAt": 1786800364856,
      "tokens": 126787,
      "toolCalls": 53,
      "durationMs": 767745,
      "resultPreview": "{"functions":[{"name":"gm-context-verify","klass":"CGameMovement","signature":"static int VerifyGmContext(void* gm, void* player, void* movedata)  // -1 unknown, 1 ok, 0 mismatch","inputs":"g_gm (GameMovement001 singleton), and the `player` + `movedata` arguments that OnFinishMove already receives. Reads g_gm+0x08 and g_gm+0x10.","outputs":"A one-shot latch s_gm_ctx_ok. 1 = the mv/player member of…"
    },
    {
      "type": "workflow_agent",
      "index": 5,
      "label": "design:leaf-first",
      "phaseIndex": 2,
      "phaseTitle": "Design",
      "agentId": "af55bd17e13d663a1",
      "model": "claude-opus-5",
      "state": "done",
      "startedAt": 1786801525856,
      "queuedAt": 1786801524184,
      "attempt": 1,
      "lastToolName": "Bash",
      "lastToolSummary": "cd "C:/Users/Connor/Documents/SourceTAS/sourceTAS" && sed -…",
      "promptPreview": "You are designing a PER-FUNCTION differential fuzzer for a Source-engine movement clone.

Context - the CURRENT fuzzer is fundamentally flawed: it drives whole ticks through ProcessMovement (a composition of ~25 functions) and compares final state. A mismatch cannot be attributed to any single function, which led to repeated wrong guesses.

Requirements from the project owner, verbatim in spirit:
…",
      "lastProgressAt": 1786801857139,
      "tokens": 96876,
      "toolCalls": 15,
      "durationMs": 331283,
      "resultPreview": "# PER-FUNCTION DIFFERENTIAL FUZZER — ARCHITECTURE + IMPLEMENTATION PLAN

## 0. What is actually wrong today, and the one structural fix

Current path (`C:\\Users\\Connor\\Documents\\SourceTAS\\sourceTAS\\Source\\World\\Prediction.cpp:328` `RunOneFuzzProbeSEH` + `Source\\Solver\\SolverLab.cpp:927` `CmdFuzzGen` / `:1089` `CmdFuzzDiff`) drives `SetupMove -> ProcessMovement -> FinishMove` and compares end-of-ti…"
    },
    {
      "type": "workflow_agent",
      "index": 6,
      "label": "design:isolation",
      "phaseIndex": 2,
      "phaseTitle": "Design",
      "agentId": "a49f38d93b24acbac",
      "model": "claude-opus-5",
      "state": "done",
      "startedAt": 1786801525867,
      "queuedAt": 1786801524185,
      "attempt": 1,
      "lastToolName": "Bash",
      "lastToolSummary": "cd "C:/Users/Connor/Documents/SourceTAS/sourceTAS" && grep …",
      "promptPreview": "Design an alternative architecture for the same problem (per-function differential fuzzing of Source movement code), optimizing for MINIMUM in-game interaction and MAXIMUM offline iteration.

Context: the current whole-tick fuzzer is flawed - it cannot attribute mismatches to a function. Owner wants one in-game button that exhaustively maps every relevant function so they can be cloned 1-to-1.

Re…",
      "lastProgressAt": 1786801871683,
      "tokens": 83168,
      "toolCalls": 15,
      "durationMs": 345815,
      "resultPreview": "# Alternative architecture: per-function differential capture

## Verdict up front

**Isolation-through-ProcessMovement wins, but not for the reason the question implies.** The decisive argument is not that it avoids RVAs (it does). It is that **direct-call cannot test the defect class that has actually cost this project months** — ordering and ownership across function boundaries (the `m_surfaceF…"
    },
    {
      "type": "workflow_agent",
      "index": 7,
      "label": "critique:adversarial",
      "phaseIndex": 3,
      "phaseTitle": "Critique",
      "agentId": "a8869467c653f9615",
      "model": "claude-opus-5",
      "state": "done",
      "startedAt": 1786801874747,
      "queuedAt": 1786801873111,
      "attempt": 1,
      "lastToolName": "Bash",
      "lastToolSummary": "cd "C:/Users/Connor/Documents/SourceTAS/sourceTAS" && grep …",
      "promptPreview": "Adversarially review these two fuzzer designs for a Source-engine movement clone.

DESIGN A (leaf-first direct-call):
# PER-FUNCTION DIFFERENTIAL FUZZER — ARCHITECTURE + IMPLEMENTATION PLAN

## 0. What is actually wrong today, and the one structural fix

Current path (`C:\\Users\\Connor\\Documents\\SourceTAS\\sourceTAS\\Source\\World\\Prediction.cpp:328` `RunOneFuzzProbeSEH` + `Source\\Solver\\SolverLab.cpp…",
      "lastProgressAt": 1786802130289,
      "tokens": 67317,
      "toolCalls": 9,
      "durationMs": 255542,
      "resultPreview": "## Grounding

I read the code both designs cite. Confirmations and corrections before the review:

- `Source\\World\\Prediction.cpp:328` `RunOneFuzzProbeSEH` — confirmed. `FuzzProbeIn` (`:296`) has **no `old_buttons` field** and nothing writes `m_nOldButtons`. Design B's claim (b) is true.
- `Source\\Solver\\SolverLab.cpp` `CmdFuzzDiff` — confirmed `break;` on first divergence inside the tick loop. De…"
    },
    {
      "type": "workflow_agent",
      "index": 8,
      "label": "critique:completeness",
      "phaseIndex": 3,
      "phaseTitle": "Critique",
      "agentId": "a5871abe650b44e9c",
      "model": "claude-opus-5",
      "state": "done",
      "startedAt": 1786801875176,
      "queuedAt": 1786801873111,
      "attempt": 1,
      "lastToolName": "StructuredOutput",
      "promptPreview": "Completeness critic. Here is a claimed inventory of every function relevant to cloning Source movement 1-to-1:

WIKI: {"functions":[{"name":"CPlayerMove::RunCommand","klass":"other","signature":"void CPlayerMove::RunCommand( CBasePlayer *player, CUserCmd *ucmd, IMoveHelper *moveHelper )","inputs":"CUserCmd (viewangles, forwardmove, sidemove, upmove, buttons, impulse, tick_count, weaponselect, mous…",
      "lastProgressAt": 1786802385539,
      "tokens": 65832,
      "toolCalls": 10,
      "durationMs": 510363,
      "resultPreview": "{"functions":[{"name":"CGameMovement::CheckParameters","klass":"CGameMovement","signature":"void CGameMovement::CheckParameters( void )","inputs":"mv->m_flForwardMove/m_flSideMove/m_flUpMove, mv->m_flClientMaxSpeed, mv->m_flMaxSpeed, player->GetMoveType(), player->GetFlags() (FL_FROZEN, FL_ONTRAIN, FL_ATCONTROLS), player->pl.deadflag, player->m_Local.m_vecPunchAngle, mv->m_vecViewAngles","outputs"…"
    },
    {
      "type": "workflow_agent",
      "index": 9,
      "label": "synthesize:spec",
      "phaseIndex": 4,
      "phaseTitle": "Synthesize",
      "agentId": "a98f4055e54452875",
      "model": "claude-opus-5",
      "state": "done",
      "startedAt": 1786802389872,
      "queuedAt": 1786802387021,
      "attempt": 1,
      "lastToolName": "Bash",
      "lastToolSummary": "cd "C:/Users/Connor/Documents/SourceTAS/sourceTAS" && grep …",
      "promptPreview": "Produce the FINAL deliverable: (1) a complete inventory of every function relevant to 1-to-1 Source movement cloning, and (2) a concrete implementation plan for a per-function differential fuzzer driven by ONE in-game button.

Inputs:
WIKI RESEARCH: {"functions":[{"name":"CPlayerMove::RunCommand","klass":"other","signature":"void CPlayerMove::RunCommand( CBasePlayer *player, CUserCmd *ucmd, IMoveH…",
      "lastProgressAt": 1786803137746,
      "tokens": 167719,
      "toolCalls": 21,
      "durationMs": 747874,
      "resultPreview": "# FINAL DELIVERABLE — Per-function differential fuzzer for 1:1 Source movement cloning

**Verdict on the two designs: hybrid, weighted toward B, with A's discipline and a narrowed version of A's direct-call.** The adversarial critique is correct on every load-bearing point I could verify in the code, so this spec adopts its corrections rather than either design as written.

The three decisions tha…"
    }
  ],
  "totalTokens": 1101295,
  "totalToolCalls": 230
}