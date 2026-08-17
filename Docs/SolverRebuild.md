# Solver Rebuild — Design of Record & Surfing Knowledge Base

Opened 2026-08-15, expanded 2026-08-16 with the expert's full testimony.
**This file is deliberately uncompressed.** It exists so that no context
loss can destroy the priorities and intuitive decisions that drive
surfing and routing. When resuming work in a fresh context: read this
WHOLE file before writing solver code. The certified engine model
(Docs/EngineParityReference.md) is the trusted simulation substrate.

Mission: **minimum time from start zone to end zone, beating a strong
human line consistently, unseeded, no route knowledge, on every map** —
linear maps first (staged maps are linear maps chained by teleport
resets that re-baseline energy and force a fresh prestrafe; solvable
per-stage or whole, per-stage likely more efficient).

Why the old solver died (user's diagnosis, near-verbatim): it was
fundamentally too disjointed. The value proposition — low-loss energy
conversion to the finish at high speed — is solid. The execution was
not: despite billions of options, a ton of energy was needlessly lost
on boards and transitions, because the solver was not context-aware
enough to make the right choices — or even to SEARCH IN THE REALM of
the right choices — at any scale: individual tick, segment, ramp, run.
The human line is miles better, and it is only a decent human run.

---

## 1. Rules of the game (complete, from the user)

- **Objective: ticks between start-zone exit and end-zone entry.
  Nothing else.** Finish speed is completely irrelevant as a goal.
- **Start zone:** exactly ONE jump per run is taken in/from the start
  zone. No bhopping inside the zone. Prestrafing to maximize speed
  before leaving off that single jump is legal and expected.
- **Mid-run jumps ARE legal but only in very limited circumstances.**
  Ramp-spine bhops are NEVER allowed. (The precise circumstances are
  case-by-case; the router should treat jumps as exceptional tools, not
  a default move. Most banned styles are slower anyway and should be
  filtered out by better routing valuing them poorly — e.g., slowly
  climbing up a ramp to get on the spine. Hard rule only where value
  won't filter it: spine bhop.)
- **The ONE aesthetic constraint: a rate limit on back-and-forth
  strafes.** Tick-perfect yaw is fine and smoothness should EMERGE, but
  1-tick auto-strafe oscillation is ugly — these TAS runs are meant to
  SHOWCASE surfing on the maps. The rate limit is the only "beauty"
  rule, and it also has practical search value (it bounds control
  oscillation frequency, i.e., yaw-spline knot density).

## 2. EXPERT TESTIMONY — the surfing model (preserve; do not compress)

### 2.1 The board (ramp entry)
For an average ramp (99%+ of cases) a good entry **boards with a clean
approach that lands with minimal energy loss: velocity tangent to the
ramp face normal IN THE APPROACH DIRECTION.**
- Falling vertically onto a ramp with zero horizontal speed: look
  straight DOWN the ramp — yaw at 90° to the face — so the board
  smoothly turns the fall into the ramp plane, converting potential
  energy in the most lossless manner possible.
- Approaching from the side, already aligned, with speed: angle again
  so the board across the ramp normal is as lossless as possible —
  closer to parallel with the ramp.
- Upward boards are somewhat strange but the idea is identical:
  maximize energy through the board. Facing directly at the ramp on
  approach → roughly a 90° turn, often slightly MORE than 90°
  considering the downward board velocity, to board tangent again.
- **The ENTIRE interaction is energy-maximizing, not just the touching
  part.** Take the approach curve at the best turning rate to maximize
  strafe speed gain in the air given height and landing options. NEVER
  fly at a face at high speed and crank a sharp, energy-losing turn in
  the last few moments — that loses a ton of energy before the ramp is
  even touched. With too much speed to turn cleanly, eat that energy
  up IN the turn — spread it through the arc — and still take the
  clean board at the absolute minimum loss.
- **Bad boards kill runs.** Every consideration serves good, smooth,
  clean, energy-preserving boards on every ramp.

### 2.2 The carve (on the ramp)
Three simultaneous jobs:
1. Preserve the energy carried through the board, plus the small
   free-energy gain from strafing into the ramp (Source is not a
   perfect energy system; mostly negligible, but it sets the strafe
   rate somewhat).
2. Maximize kinetic energy — convert height to speed where possible.
3. **Crucially: set up the flick** (the launch to the next ramp).
The exit point on the face is COMPLETELY determined by current speed,
energy, and the position of the next ramp.

### 2.3 The flick (ramp exit)
"Flick" = the general term for exiting the ramp; it does NOT always
involve a lossy jerking motion. The flick launches at the best angle to
get a clean, efficient board on the NEXT ramp; on a fast run it also
minimizes air time to the next ramp while still boarding high enough or
far enough forward to repeat the process. The best transitions lose
nothing — and in fact GAIN energy. When that's impossible, energy is
sacrificed in the exit to gain height — extremely case-specific, no
concrete rule; in general sacrifice the minimum, avoid it if possible.
The exit angle is entirely caused by yaw for that transition decision.
Turning away from the ramp leaves the ramp; flicks that leave before
the edge generally turn INTO the ramp and then strafe away from it,
toward the next target.

### 2.4 Height vs speed
A fast run repetitively converts height into speed. Flatter approaches
preserving the same energy come out FASTER — they are a more direct
path to the next ramp. Provided the next ramp still boards cleanly and
preserves the energy, **fast runs prioritize horizontal speed over
vertical height anywhere they can get away with it.** Exceptions: a
ramp that would be overshot or badly boarded with too much horizontal
speed — vertical height is sometimes better there. Some ramps must be
boarded on the side instead of across the top, or halfway up the face
instead of on an edge. All of that is tied into the approach and exit.

### 2.5 The chain and the plan
Most surf maps are ramp→air→ramp→air→ramp. In isolation a ramp ride
cares only about the ramp before and after it — but because ramps come
in order, they chain in the context of the full run. **Sometimes
skipping a ramp entirely is far easier, more energy-preserving, or
time-saving — ramp selection is a real routing decision.**
The concrete plan while surfing extends over the next 2→3→4 ramps:
where am I on the current ramp, how must I flick to reach the next
ramp SUCH THAT the next ramp sets up the ramp after it. Not thinking
about the final ramp of a minute-long map at the start — but definitely
thinking about the overall route across the next several ramps: fast,
efficient, AND possible.

### 2.6 Air between features
High-level players absolutely strafe-gain during aerial maneuvers: on a
far ramp reachable in basically a straight line, weave — strafe back
and forth quickly — to maximize energy gained in the air while keeping
a relatively straight net heading to the target. Ramps at a different
angle than the exit direction have different exit/board position
considerations based on what energy can be gained in the air and how
it converts. (The weave frequency is bounded by the aesthetic rate
limit in §1.)

### 2.7 The start (prestrafe)
Its own animal; techniques differ slightly from normal surfing. The
shape: get maximum ground speed → jump (the one) → maximize air-strafe
gain from side-to-side strafing → exit the start zone as LATE as
possible before touching the ground — **but a good clean board on the
first ramp that maximizes energy through the rest of the map is MORE
important than a few extra ticks of falling speed-gain in the zone.**
The free energy of the start jump must always be taken.

### 2.8b RUNWAY (added 2026-08-17 from the user; preserve near-verbatim)
Runway = the usable space remaining on the ramp after the board
point, in the direction of the action the ride must perform. It
straddles making hard maps easier and making runs faster.
- A high-level player who simply wants the best chance of beating a
  map very often (context dependent, like most of surf) chooses the
  board that provides the MOST runway for the given ramp to perform
  the action they need. This loosens the margins: more time to build
  speed on the ramp, a smoother transition from the entry angle to
  the exit angle, and more precise planning of the next motion.
- Faster runs sometimes optimize that runway away: it would be
  easier to have more of the ramp, but less runway gives less room
  for error and may sacrifice energy in the moment.
- Runway works IN CONJUNCTION with the energy field (the hotspot
  map): the field says where energy survives; among energy-
  equivalent hot points, RUNWAY IS OFTEN THE DECIDING FACTOR.
  Concrete case read off the validated field (face 3, basictest):
  the hot area is broad, and the human boards at its far edge
  specifically to keep enough runway to convert speed into the
  upward velocity the ending needs — boarding at the other end of
  the hot area would leave no space remaining to do what's needed.
- The ratio matters in both directions: the landing needs the
  optimal ratio of room to make the move to the next target ramp,
  but giving too much runway takes more ticks and can be slower in
  some contexts. In other contexts using MORE runway is faster,
  since it offers the opportunity to build more energy up for free
  (the measured human ending: ~60 extra ride ticks banking ~54k of
  wish work the direct crest launch lacks).
- Once a landing that preserves energy is decided, runway is one of
  the biggest considerations for WHY a high-level player lands
  where they land.

FORMALIZATION HOOK (proposal, to build with the field): per field
sample Q, runway_available = in-plane extent from Q along the
projected ride direction to the face boundary (toward the departure
region serving the next objective); runway_required = space the
KNOWN action needs, derived from laws (climb: dz / face slope;
entry->exit turn: heading change / free-turn rate x speed x dt;
energy target: ticks to bank the deficit at the wish-work rate).
The ratio available/required is the margin dial: consistency mode
maximizes it, speed mode drives it toward the smallest ratio whose
action still completes - EXCEPT where extra runway is net-faster
because banked energy repays the ticks (the potential-ledger
exchange rate 900/tick prices exactly this trade).

### 2.9 EXIT HEATMAPS (added 2026-08-17 from the user; preserve near-verbatim)
High-quality runs are combinations of very good or perfect decisions;
bad decisions DISQUALIFY a line. Some are measurable BEFORE they
resolve: flying off a ramp bottom into the void with no reachable
ramp is a disqualification the moment the exit is taken; slams are
instant failure or major time loss. Use heatmap-style representations
at varying resolutions to represent every combination of high-quality
decisions and choose the fastest start-to-end combination.
- THE THEORY: an optimal ramp EXIT is one that produces a
  higher-quality BOARD heatmap on the NEXT ramp. The tuning question
  is what "optimal board heatmap" means - candidate definitions the
  user named: (a) the average of many board heatmaps from many exits
  scored by heat quality, (b) large hot surface area, (c) higher
  energy-preserving/creating landings. Use the human run for
  data-driven theories.
- Exit heatmaps may also be driven by the ramp TRAVERSAL (smooth /
  energy-gaining moves preferred over snappy flicks or very fast
  dismounts), but the next-board value has more potential than
  general ride rules - dismount thinking is about the next ramp.
- Heatmaps represent huge state spaces approximately for cheap,
  guide higher-resolution heatmaps in selective areas, and LIMIT THE
  SEARCH SPACE.
- Follow-ups (user): BOTH readings intended - (A) score exits by the
  board map each induces (the decision layer) and (B) score landing
  zones by how hot they stay across many exits (marginal robustness)
  - but "don't hurt our entry heatmaps, be careful with B" (B ships
  report-only). The primary exit axis is WHERE: where to exit decides
  when, and where relates directly to the landing on the next ramp
  (and which ramp is targeted, on complex maps).

MEASURED VERDICT (exitgate vs the human run, 2026-08-17 - the
functional question settled by data, not argument): the validated
functional is FPOT = the potential-priced induced maximum (induced
best e_eff minus cap^2 per tick of flight, minus cap^2 per tick of
ride - the M2 telescoping-anchor currency). The human's exact exits
score fpot ratio 1.000 on BOTH face rides (they sit precisely on the
priced field's flat top). Raw induced-max (unpriced) rates them
0.94-0.95 but its argmax is a wish-credit artifact (ride backward
for free energy credit - pricing kills it). Hot-AREA functionals
rate their exits 0.09-0.22: this speed run does NOT maximize margin
area, matching 2.8b (speed mode narrows margins) - area stays a
margin OVERLAY, not the value. Doom culling is real and free: 32% /
16% / 60% of candidate exits per ride are provably dead before
departure. ZoneReach validates both real endings (human margin 61z,
old solver 9z). The ride bookkeeping is optimistic by +65..+114 u/s
(unpriced ride clip dissipation - real rides bleed); admissible by
doctrine, and the overshoot is itself a measurement of ride
execution quality.

### 2.8 Skill tells / the loss patterns to kill
The user can tell a player's skill level immediately from: (1) how
smoothly they move in the air, (2) how cleanly and efficiently they
board, (3) beyond that, how they position for the NEXT ramp. The old
solver's tells, verbatim targets to eliminate:
- snappy random direction changes with hard yaw spikes — they indicate
  NO PLAN to reach the ramp, late compensation;
- hard boards that lose energy and speed — terrible in about every
  case;
- inefficient, non-context-aware routing that walks into bad positions
  that are hard to get out of. **With more energy comes more options** —
  efficient routing compounds.

## 3. Representation

### 3.1 Feature graph (from BSP, automatic, per map)
Nodes: surfable features — ramp faces (non-walkable planes nz < 0.7,
grouped with edges/extents/spines), start zone, end zone, triggers
(teleport/boost — already modeled). Edges: candidate transfers A→B
INCLUDING SKIPS, gated by coarse reachability envelopes (gravity +
air-gain bounds). Staged maps fragment at teleport resets.

### 3.2 The TRANSFER primitive — the unit of value
One transfer = exit state on A → air phase → board on B → carve on B →
exit state on B. All expert criteria live at this granularity:
- **Board window** of a feature: region on the face × velocity cone
  (tangent-dominant, per §2.1) that boards under a clip-loss threshold.
- **Exit manifold**: exit states (edge point or early-flick point ×
  velocity) reachable from an entry via legal carves, tagged with tick
  cost and energy delta.
Transfers compose by propagating A's exit manifold through the air
phase and intersecting with B's board window. Side boards, mid-face
boards, and skips are all just different windows/edges.

### 3.3 Controls parameterization
Per phase, controls collapse to a **yaw profile over ticks** (+ duck
where relevant); fmove/smove follow optimal-strafe sync mechanically.
Air: yaw(t) spline with boundary conditions (exit heading → board
heading), knot density capped by the strafe rate limit (§1). Carve:
yaw(t) on the face. Snappy yaw is UNREPRESENTABLE in this space except
where the optimum truly is a fast arc (e.g., the 90° board turn).

## 4. Scoring: the regret ledger

Every level scored as **regret against a physics bound**, never raw
fitness:
- **Board loss** (exact): |v·n̂| destroyed at contact —
  TickEvents.contact_loss already measures it tick-by-tick.
- **Approach loss**: integrated deviation from the optimal
  strafe-gain-vs-turn-rate curve over the approach arc (the
  "late sharp turn" regret).
- **Air-gain shortfall**: energy gained vs the strafe-gain bound for
  the available ticks and net heading change (weaving counts).
- **Conversion shortfall**: kinetic gained on the carve vs potential
  available along the path taken.
- **Time regret**: transfer ticks vs the flat-transfer bound at equal
  energy (§2.4: flatter equal-energy paths are faster).
- **Route regret**: DP value gap vs best sibling route.
A run yields a LEDGER attributing loss per transfer per phase. Compute
goes where the ledger points. "Needless loss" is a number with an
address.

## 5. Search architecture (anytime, four stages)

1. **Graph build** (seconds): features, edges, bounds.
2. **Route search** (fast, wide): beam/DP over feature sequences using
   analytic bounds (energy in → time bound out). Start-zone plan and
   skip decisions live here. Thousands of ordered candidates.
3. **Transfer refinement** (the core): per route, solve each transfer's
   yaw-profile trajectory on the exact engine over short horizons
   (approach arc / board / carve / flick), propagating entry states
   forward with a rolling 2-4 feature window (§2.5) — each flick
   optimized against the NEXT board window and the ramp after.
   Infeasible transfer → prune route immediately.
4. **Assembly + polish**: chain transfer solutions into full runs on
   the exact engine; CMA-ES survives ONLY as a residual polisher on the
   yaw-splines (small, boundary-locked, seeded, never random). The
   ledger routes extra polish to the worst transfers.

Anytime: finishable chains in minutes from stages 2-3; quality scales
with budget via toggles (beam width, window depth, polish rounds) — the
accepted dial is "2-minute solve within ~5 ticks of the 20-minute
solve". Current budget ceiling 30 minutes; **if a map can't even finish
inside 30 minutes the ROUTING RULES are wrong — fix rules, never throw
budget.** Ultimate goal: thousands-to-millions of good candidates, push
solve time down hard.

## 6. Validation

- OLD SOLVER RUNS (anything before the big restructure) are to be
  IGNORED except as negative comparisons - a way to understand what
  not to do (user, 2026-08-17). The positive references are the
  human tape(s) and the expert demos.
- Exact-engine replay + tape verification with existing instruments
  (battery/diff/playback); batch in-game checks into rare sessions —
  the design loop is offline (the user is DONE with relaunch grind).
- **Human demos: validation only, NEVER seeding.** The user is cutting
  hour-long session demos (top-1% difficulty zones, ~10-20s valid
  completion each, completion marked by a chat line with the time) down
  to useful ranges with a demo editor they are building; files incoming.
  Build a .dem trace extractor when they land; compare the expert's
  ledger (board losses, transfer times) against ours on maps we never
  tuned on.
- The regret ledger is the primary development instrument.

## 7. What survives from the old codebase

Survives: the entire certified engine model, playback/diff/battery
instruments, .tas writer, trigger handling, worker pools, CMA-ES
(demoted to polisher). Dies: corridor beams, raw-fitness archives,
contact-anchored random mutation, dissipation-bias heuristics.

## 8. Priority hierarchy (dominance order, from testimony)

1. Clean tangent boards — bad boards kill runs; first board of the map
   outranks late start-zone exit.
2. Energy through the whole interaction (approach + board + carve +
   flick as one object); spread unavoidable loss through the arc.
3. Horizontal over vertical at equal energy, wherever the next board
   stays clean; buy height minimally and only when forced (overshoot /
   unboardable-fast cases).
4. Route context: flick targets the next board SUCH THAT it sets up the
   one after (rolling 2-4); skips are first-class options; more energy
   = more options, so efficiency compounds. WITHIN energy-equivalent
   board choices, RUNWAY decides (2.8b): enough room on the ramp to
   perform the required action, priced against the ticks it costs and
   the free energy it banks.
5. Time is the only terminal objective; everything above is
   instrumental to it.

## 9. Open items

1. First math task: derive the exact strafe-gain-vs-turn-rate law from
   the certified engine (analytic; underpins approach-loss and air-gain
   bounds and the yaw parameterization).
2. Set the strafe alternation rate limit constant with the user (the
   aesthetic bound; also a search parameter).
3. Demo files: incoming from the user's demo editor; build the
   extractor then.
4. Mid-run jump "limited circumstances": collect concrete cases as maps
   demand them; treat jumps as exceptional route edges, spine-bhop hard-
   filtered.
