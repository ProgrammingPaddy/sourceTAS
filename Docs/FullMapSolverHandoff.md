# Surf Solver Design of Record
## From Witness-Backed Entrance Fields to an Efficient, Certifiable Full-Map Solver

**Purpose:** This document is a self-contained engineering handoff for the next stage of the surf solver. It begins with the latest entrance-field results and immediate guidance, then defines the full-map architecture that should replace the old heuristic per-leg pipeline once the entrance field is trustworthy.

The intended end state is not merely a solver that usually finds good routes. It is a solver whose local transitions are backed by exact engine-replayed witnesses, whose global search operates over meaningful boundary states instead of thousands of raw ticks, and whose pruning can eventually certify that no unresolved route can beat the incumbent finish time.

---

# 0. Immediate guidance from the latest entrance-field results

The latest results materially change the diagnosis of the remaining f2 failure and give a clear next step.

## 0.1 What has now been established

The canonical aerial control space is the direct wish-input basis:

\[
U = \{(d_k,\cos\alpha_k)\}_{k=0}^{N-1},
\]

where:

- \(d_k\) is the left/right side choice at tick \(k\),
- \(\alpha_k\) is the wish-direction angle relative to the current horizontal velocity heading,
- side changes obey the chosen **minimum 6-tick dwell law**,
- the engine is advanced directly through `Air::FlyWishSchedule`,
- no heading-tracking controller is part of the definition of the admissible control space.

The prior heading-command representation was constructively shown to be incomplete: even when handed the human's realized headings, the tracking controller missed the known human contacts by tens to hundreds of units. The direct wish basis, by contrast, can replay the recorded human flights at engine-level positional accuracy when supplied the exact per-tick wish schedule.

That means the low-level physical basis is no longer the suspect. The current problem is **how to search it efficiently**.

The latest f2 diagnostic is particularly important:

- Human reference at f2: approximately **945k** post-board energy.
- Exact per-tick wish reconstruction: approximately **949k**, **0u** contact error.
- Starting from that exact schedule, the local optimizer improves it to approximately **951k**.
- Interpolated compressed profiles with \(K=1\) through \(17\) knots per side run do not reliably reproduce the basin.
- Tiny perturbations of the exact schedule fail to recover the contact: **0/8 recoveries even at radius 0.05**.

This is a classic **ill-conditioned open-loop shooting problem**. Nearby schedules are not necessarily nearby trajectories after 46 ticks because small per-tick changes compound. The good basin is physically real and locally improvable once entered, but it is extremely difficult to enter by blind perturbation in raw schedule coordinates.

That is different from three earlier possibilities:

1. **Incomplete basis** — now disproven for the canonical per-tick wish space.
2. **Destructive local optimizer** — disproven at f2 because the optimizer improves the exact schedule from 949k to 951k.
3. **Ordinary weak seeding** — incomplete diagnosis; the deeper issue is that raw schedule-space distance is a poor metric for endpoint-space closeness.

The result should change the search machinery, **not** the control-space definition.

---

## 0.2 Ruling: search-time closed-loop guidance is allowed and recommended

The agent's proposed resolution is correct:

> Search-time guidance may be feedback-based, target-conditioned, corrective, arrival-anchored, or otherwise designed to make the optimization well-conditioned, provided that the accepted result is always converted into and replayed as the canonical open-loop per-tick wish schedule.

This cleanly separates two concepts.

### Physical/admissibility definition

A valid final aerial witness is:

\[
U^* = \{(d_k,\cos\alpha_k)\}_{k=0}^{N-1}
\]

that, when replayed **open-loop from the exact starting engine state**, produces the claimed contact and post-board state while respecting the 6-tick side-change law.

### Search machinery

A search algorithm may generate that sequence using anything useful:

- a target-tracking controller,
- receding-horizon correction,
- endpoint residual minimization,
- intermediate waypoints,
- backward desired-heading calculations,
- local feedback on current position and velocity,
- learned proposal policies later,
- deterministic or stochastic optimizers.

Those mechanisms are **proposal generators**, not new physics and not new admissible controls.

If a feedback-guided candidate produces a sequence

\[
U_{guided},
\]

then the only value allowed into the heatmap is the value obtained by:

1. recording that exact sequence,
2. resetting to the exact initial state,
3. replaying \(U_{guided}\) open-loop through `FlyWishSchedule`,
4. evaluating the actual contact and post-board state,
5. storing the replayed witness and replayed value.

This preserves the central guarantee:

\[
\boxed{
\text{Every stored heatmap value is attached to a canonical engine-replayed witness.}
}
\]

A search-time controller is therefore harmless as long as **the controller itself is not what is committed or trusted**.

---

## 0.3 Why guided shooting is the right response to the f2 needle

The raw parameter map

\[
U \longrightarrow Q(U)
\]

is poorly conditioned in the f2 corridor. Tiny schedule changes can cause large final-position changes. Therefore Euclidean distance in raw \(\cos\alpha\)-schedule space is not a useful optimization geometry.

Instead, search should use variables that are closer to the desired **outcome geometry**.

Recommended proposal strategy:

### A. Target-conditioned guided shooting

For a chosen side-run topology:

1. Begin from the exact current state.
2. At each tick or short block, compute the remaining endpoint error to \(Q\), desired terminal-heading class, remaining tick budget, and current velocity.
3. Choose a wish direction that reduces the predicted endpoint/heading residual while retaining as much gain as possible.
4. Advance the exact engine.
5. Continue until contact or failure.
6. Record the resulting per-tick wish schedule.
7. Replay it open-loop.
8. Use the canonical replay as the candidate.

The guidance can be parameterized with only a few gains/weights, making the search basin much wider than searching 46 independent \(\cos\alpha_k\) values.

### B. Waypoint-conditioned shooting

Divide a long flight into a few temporal blocks and search over physically meaningful intermediate targets rather than raw controls. For example, over a 46-tick flight:

- tick 0–12: produce a desired lateral displacement / velocity orientation,
- tick 12–26: set up the corridor crossing,
- tick 26–38: shape arrival heading,
- tick 38–46: close the endpoint and board.

Each block is still flown sequentially through the exact engine. The block targets only guide generation of the wish sequence. The final stored object remains one continuous open-loop schedule.

This is not permission to return to the old broken heading controller. The difference is crucial:

- **Old design:** desired heading schedule was treated as the actual control representation, and a separate controller attempted to realize it.
- **New design:** feedback/heading logic may be used internally to *generate actual wish inputs*, but the resulting direct wish inputs are what are replayed and stored.

---

## 0.4 Preserve terminal-state diversity while improving point-targeted search

The new terminal-heading frontier is conceptually correct:

\[
\theta \longmapsto s_A^*(Q,T,\theta).
\]

However, the latest regression also shows that a field-oriented frontier search and a point-targeted solve have different needs.

For a broad face field:

- maintain terminal-heading bins,
- keep per-bin elites,
- refine bins whose achievable speed or board value remains interesting,
- use the same frontier to derive both `BestBoard` and `FastestTangent`.

For an exact point solve such as `humanexact`:

- the target disc is tiny,
- most random flights miss,
- many terminal bins remain empty,
- a small global scout lane can be too weak.

Therefore point-targeted mode should retain a **full-strength scout pool** in addition to the terminal bins. The old top-3 deepening behavior was useful here because it effectively gave several unconstrained high-budget endpoint seekers a chance to enter a narrow basin.

Recommended rule:

> **Bins preserve terminal-state diversity. Scouts maximize probability of entering difficult endpoint basins. Both feed the same canonical witness frontier.**

Do not force one mechanism to do both jobs.

---

## 0.5 Keep the witness bank permanently monotonic

The witness bank is one of the most important pieces of infrastructure now in place.

For every exact key that matters—at minimum including:

- starting engine state identity/hash,
- target face,
- target point or target region,
- arrival branch/tick where relevant,
- 6-tick law version,
- collision-law/version,

store the best verified witness ever found.

The invariant is:

\[
\boxed{
H_{found}^{(n+1)}(Q) \ge H_{found}^{(n)}(Q).
}
\]

Search rewrites may fail to rediscover an old basin; that must never cause the known lower bound to decrease.

The f3 legal 847k human-beating witness and the f0 898k near-contact witness are examples of results that should never silently disappear.

A regression should be reported as:

> Current search failed to rediscover the banked witness.

not as:

> The best known result got worse.

---

## 0.6 Keep the Tier-0 split

The latest experiments correctly separated:

- **reachability predicate** — currently uncertified because clean-air flights exist that it calls unreachable,
- **energy ceiling** — independently re-certified on the contested cells.

This separation must remain explicit.

An uncertified reachability predicate may:

- prioritize,
- seed,
- allocate budget,
- render diagnostics,

but it may **not hard-cull**.

A certified energy upper bound may participate in the sandwich:

\[
\boxed{
H_{found}(Q) \le H_{true}(Q) \le H_{upper}(Q).
}
\]

Never let a failure of one Tier-0 claim silently contaminate the certification status of another.

---

## 0.7 Immediate next entrance-field build order

Before beginning the full-map architecture below, finish the entrance field in this order:

1. **f2 guided-shooting accessibility test** using the direct wish basis.
2. Retain the exact f2 951k witness as a permanent lower bound.
3. Restore strong point-targeted scout deepening alongside the terminal-heading bins.
4. Re-run `humanexact`:
   - f0: admissible, exact-point floor still must be met,
   - f2: admissible, exact-point solver must reach or beat the banked 951k result,
   - f3: human row remains formally N/A under the 6-tick law, but the solver's legal 847k result remains a qualitative success/regression anchor.
5. Re-run `efield`.
6. Re-run `fieldexact` with gate-level reference budgets.
7. Make Build allocate refinement dynamically until its unresolved field regions are close to the gate/reference frontier.
8. Keep Phase-B route integration frozen until the ordinary Build field is trustworthy.
9. Keep carve migration frozen until the entrance machinery has converged; then port the same direct wish/guided-search philosophy to ramp traversal.

The rest of this document describes what happens **after that point**.

---

# 1. End goal: what “perfect full-map solve” means

The target is a solver that finds the fastest legal completion of a surf map under the chosen engine physics and control law.

There are three different meanings of “correct” that must remain separate.

## 1.1 Physics correctness

Every accepted trajectory is replayed by the exact engine predictor using the actual admissible controls.

No closed-form trace, controller prediction, spline estimate, or analytical heatmap value is accepted as reality without a replayed witness.

## 1.2 Search completeness / local optimality

For each local boundary-value problem, the solver either:

- finds the true optimum, or
- maintains a proven upper bound on what remains unresolved.

A witnessed value is a **lower bound** on the optimum. A certified optimistic relaxation is an **upper bound**.

## 1.3 Global optimality

The map solver maintains an incumbent complete run and proves that every unresolved partial route has an optimistic earliest possible finish no better than the incumbent.

If finish time is measured in integer ticks and the best complete run takes \(N\) ticks, then exact tick-optimality is certified when every open branch has:

\[
\boxed{
T_{optimistic} \ge N.
}
\]

At that point there is no need to “search more just in case.” The remaining search space is mathematically unable to win.

---

# 2. The architectural change: from tick search to transfer-function composition

The old system solved a map approximately like this:

\[
\text{route guess}
\rightarrow
\text{board search}
\rightarrow
\text{carve search}
\rightarrow
\text{local selection}
\rightarrow
\text{next leg}.
\]

The replacement should be:

\[
\boxed{
\text{exact local boundary operators}
\rightarrow
\text{small successor frontiers}
\rightarrow
\text{global best-first composition}.
}
\]

The low-level engine still advances tick by tick. The key difference is **where branching occurs**.

The full-map search should not ask:

> What should yaw / wish input be at tick 1732?

It should ask:

> Which exact exit state from this ramp should be composed with which exact entrance state on the next useful face?

The local operator is responsible for the thousands or millions of tick-level possibilities that answer that question.

---

# 3. The state carried between local operators

A boundary state should contain every engine-relevant quantity required to reproduce the future, but those quantities should be **outputs of prior local solves**, not independent dimensions globally enumerated.

A conceptual boundary state is:

\[
S = (F,Q,\mathbf v,t,d,c,m),
\]

where:

- \(F\): current face / surface identity,
- \(Q\): exact boundary/contact position,
- \(\mathbf v\): exact current velocity,
- \(t\): cumulative map tick,
- \(d\): current side/sign state where relevant,
- \(c\): ticks since last side reversal, so the 6-tick dwell law remains enforceable across operator boundaries,
- \(m\): any additional discrete engine mode/state that materially changes future physics.

Examples of \(m\) might include whether the player is grounded, in a catch window, or another exact state flag required by the movement predictor. Do not include speculative fields; include only state that changes future evolution.

## 3.1 Velocity is carried, not globally searched

The intended dimensional reduction is **not**:

> Throw velocity away.

It is:

> Stop independently gridding/searching velocity as a global coordinate.

A local entrance solve produces:

\[
(Q,\mathbf v^+).
\]

That exact \(\mathbf v^+\) is passed to the ramp traversal solve.

A ramp traversal solve produces:

\[
(X,\mathbf v_{exit}).
\]

That exact \(\mathbf v_{exit}\) is passed to the next aerial entrance solve.

Thus velocity remains physically exact while disappearing as a combinatorial global axis.

---

# 4. Local operator A: Air → Board (`EntranceField`)

This is the operator currently being built.

Given an exact airborne start state \(S_A\) and destination face \(F\), compute the best realizable entrance states over the face.

Conceptually:

\[
\boxed{
\mathcal E_F(S_A).
}
\]

For each physical candidate contact point \(Q\), the primary value is:

\[
\boxed{
H(Q\mid S_A)
=
\max_{U\in\mathcal U(Q)} E_{post}(U),
}
\]

where \(\mathcal U(Q)\) is the set of legal direct-wish schedules that start at \(S_A\), remain clean-air until the target contact, and contact the target face at \(Q\).

The record should contain at least:

- contact point \(Q\),
- arrival tick/branch \(T\),
- pre-board velocity \(\mathbf v^-\),
- post-board velocity \(\mathbf v^+\),
- horizontal arrival heading \(\theta\),
- horizontal arrival speed \(s\),
- normal approach \(\mathbf n\cdot\mathbf v^-\),
- exact post-board energy,
- board loss,
- side/dwell state after the witness,
- canonical per-tick wish witness,
- source/proposal method used to discover it,
- lower/upper bound gap if available.

---

# 5. Mathematical reductions inside the entrance operator

These remain important because they reduce the dimension of the local problem before any search occurs.

## 5.1 Canonicalize translation and horizontal yaw

Given:

\[
S_0=(\mathbf p_0,\mathbf v_0),
\]

translate coordinates so:

\[
\mathbf p_0=(0,0,0),
\]

and rotate horizontally so the initial horizontal velocity points along +X:

\[
\mathbf v_{h0}=(s_0,0).
\]

Then:

\[
\mathbf v_0=(s_0,0,v_{z0}).
\]

The target geometry is transformed into the same local frame.

This removes absolute world position and absolute world yaw from the local solve.

## 5.2 Gravity removes vertical velocity as an independent search variable

For a clean-air segment, vertical motion is deterministic once the initial vertical state and elapsed ticks are known.

Continuous notation illustrates the structure:

\[
z(t)=z_0+v_{z0}t-\frac12gt^2,
\]

\[
v_z(t)=v_{z0}-gt.
\]

Production code uses the exact Source tick update, not these continuous approximations.

For a candidate contact point \(Q=(x_Q,y_Q,z_Q)\), solve the exact vertical evolution for valid arrival tick/branch \(T\). Then:

\[
v_z=v_z(T)
\]

is determined.

The aerial solver therefore does **not** independently search \(v_z\).

---

# 6. Board geometry and tangent manifold

Let the target plane have unit normal:

\[
\mathbf n=(n_x,n_y,n_z),
\]

and pre-impact velocity:

\[
\mathbf v^-=(v_x,v_y,v_z).
\]

For ideal tangency:

\[
\mathbf n\cdot\mathbf v^- = 0.
\]

If exact collision semantics require a small inward contact component \(c\), use:

\[
\mathbf n\cdot\mathbf v^- = c.
\]

Once gravity has fixed \(v_z(T)\), this becomes:

\[
\boxed{
n_xv_x+n_yv_y=c-n_zv_z(T).
}
\]

This is a line in horizontal terminal-velocity space.

Therefore a ramp does **not** provide a single ideal board angle. It provides a one-dimensional family of tangent-compatible terminal velocities.

With horizontal speed \(s\) and heading \(\theta\):

\[
v_x=s\cos\theta,\qquad
v_y=s\sin\theta,
\]

so the tangent-required speed is:

\[
\boxed{
s_{tan}(T,\theta)
=
\frac{c-n_zv_z(T)}{n_x\cos\theta+n_y\sin\theta}.
}
\]

Only physically meaningful sign/denominator branches are considered.

---

# 7. Aerial terminal frontier

For a fixed start \(S_0\), contact \(Q\), arrival branch \(T\), and requested terminal heading \(\theta\), define:

\[
\boxed{
s_A^*(S_0,Q,T,\theta)
}
\]

as the maximum horizontal terminal speed obtainable by a legal direct-wish schedule that reaches exactly \(Q\) at \(T\) and ends at heading \(\theta\).

Formally:

\[
s_A^*=
\max_U \|\mathbf v_h(T;U)\|
\]

subject to:

\[
\mathbf p(T;U)=Q,
\]

\[
\arg\mathbf v_h(T;U)=\theta,
\]

plus exact engine dynamics and the 6-tick reversal law.

The terminal pre-board state is:

\[
\mathbf v^-(\theta)=
\left(
 s_A^*(\theta)\cos\theta,
 s_A^*(\theta)\sin\theta,
 v_z(T)
\right).
\]

Then define exact board value:

\[
J_Q(T,\theta)=E_{after}(\mathbf v^-(\theta),F).
\]

The primary point value is:

\[
\boxed{
H(Q)=\max_{T,\theta}J_Q(T,\theta).
}
\]

The winning angle and speed are outputs, not guessed inputs.

---

# 8. Fastest feasible tangent board remains a secondary exact result

For each \(Q,T\), define the aerially reachable terminal set:

\[
\mathcal A(S_0,Q,T),
\]

and the tangent manifold:

\[
\mathcal B(T,F)
=
\{\mathbf v_h:n_xv_x+n_yv_y=c-n_zv_z(T)\}.
\]

Then:

\[
\boxed{
FastestTangent(Q,T)
=
\arg\max_{\mathbf v_h\in\mathcal A\cap\mathcal B}
\| (\mathbf v_h,v_z(T)) \|.
}
\]

In angle form, candidate tangent roots satisfy:

\[
\boxed{
s_A^*(Q,T,\theta)=s_{tan}(T,\theta).
}
\]

or equivalently:

\[
G_Q(T,\theta)=0.
\]

Store:

\[
\Delta_{tan}(Q)=H(Q)-H_{tan}(Q).
\]

Do **not** collapse BestBoard to FastestTangent until tangent dominance is proven for the exact Source control problem. Tangency guarantees minimal normal loss for a given incoming state; it does not by itself prove maximum absolute post-board energy when a faster non-tangent arrival may exist.

---

# 9. Important full-map correction: one scalar BestBoard per point is not always enough

For entrance-field validation, it is correct to ask:

\[
BestBoard(Q)=\arg\max E_{post}.
\]

For a full map, however, immediate post-board energy is not necessarily sufficient to determine the best continuation.

Consider two legal arrivals at the same point \(Q\):

### Arrival A

- post-board energy: 1.00M,
- post-board heading: 10° away from the useful ramp direction,
- requires 28 ramp ticks before a strong exit is available.

### Arrival B

- post-board energy: 0.96M,
- post-board heading: already aligned with the useful ramp direction,
- reaches a strong exit in 15 ticks.

A purely local heatmap would prefer A.

A fastest-map solver may prefer B by a large margin.

Therefore the full-map version of the entrance operator should expose a **small nondominated continuation frontier** at important locations, not only one scalar winner.

Conceptually:

\[
\boxed{
\Pi_E(Q)=\{S_{board}^{(1)},S_{board}^{(2)},\ldots\}.
}
\]

Different retained states may trade:

- arrival tick,
- post-board energy,
- post-board velocity heading,
- side/dwell state,
- vertical state where multiple branches exist.

### Critical caution about “Pareto dominance”

Do not assume that simple componentwise dominance in speed/heading/time is automatically safe. Surf dynamics are nonlinear, and a state with more speed is not automatically future-equivalent to one with less speed if direction and geometry differ.

For **certification mode**, a state should only be hard-pruned when future dominance is actually proven.

A universally safe example is:

> Two states have the exact same engine-relevant state except that one occurs at an earlier cumulative map tick. The later state is dominated because all future physics is identical and it starts later.

Approximate frontiers, clustering, or epsilon-deduplication are useful in exploratory mode, but they are not by themselves proofs of optimality.

---

# 10. Local operator B: Board → Exit (`ExitField`)

This is the next major component to build after the entrance field is trustworthy.

Given an exact post-board state on face \(F\):

\[
S_B=(F,Q_{in},\mathbf v_{in},d,c,\ldots),
\]

solve the legal ramp traversal and produce useful exact exit states.

Define:

\[
\boxed{
\mathcal R_F(S_B).
}
\]

For each exit/dismount point \(X\) or exit boundary region, store one or more nondominated exact states:

\[
X
\rightarrow
\{
\mathbf v_{exit},
\Delta t,
E_{exit},
\theta_{exit},
d_{exit},
c_{exit},
U_{ramp}
\}.
\]

This is the **ramp exit field**.

## 10.1 Why this replaces most of the old carve search

The old core loop spends roughly 80% of solve time in 24k-evaluation carve searches, using many families and a large amount of shaping machinery to discover how to ride from an entry state toward a useful departure.

The new operator directly asks:

> From this exact board state, what exact useful departure states can this ramp produce?

That local search may still be expensive internally, but:

- it is isolated,
- it can be validated independently,
- it can reuse the direct wish-control basis,
- it can use adaptive outcome-conditioned guidance,
- it returns a compact successor frontier,
- it can be cached/warm-started,
- global search no longer repeats the same conceptual carve discovery at multiple lookahead depths.

---

# 11. ExitField objective: do not maximize only exit energy

The same local-greediness problem exists on ramp exits.

At exit point \(X\), a maximum-energy departure may be badly oriented for the next face. A slightly slower departure may have a much shorter next aerial transfer.

Therefore an exit record should retain useful alternatives over at least:

- departure tick,
- departure velocity vector,
- side/dwell state.

Example:

| Candidate | Ramp ticks | Exit speed | Exit heading |
|---|---:|---:|---:|
| A | 15 | 2200 | 20° |
| B | 18 | 2400 | 36° |
| C | 22 | 2520 | 54° |

Depending on the next face, any one of these could produce the fastest total continuation.

Again: retain a compact **continuation frontier**, not thousands of raw carve paths and not one local scalar winner.

---

# 12. ExitField should use the same canonical-control philosophy

The carve/ride operator should ultimately be migrated away from any path representation that can fail to express known engine trajectories.

The same principle that fixed aerial search applies:

1. Define admissibility in terms of actual engine inputs.
2. Search may use higher-level feedback or geometric guidance.
3. Accepted results are compiled/recorded as canonical controls.
4. Reset and replay through the exact engine.
5. Only the replayed result enters the exit field.

The 6-tick side reversal law must remain continuous across the board/ramp/air boundaries. A local operator may not reset control-history legality just because a software module boundary was crossed.

---

# 13. Intermediate contacts must become explicit transitions

The clean-air entrance derivation assumes no collision before the destination board.

If a trajectory does:

\[
S_A \rightarrow C \rightarrow Q,
\]

where \(C\) is another surface contact, then the vertical/gravity-only reduction across the entire interval no longer holds.

Do not hide that contact inside an “air” witness.

Represent it as composition:

\[
\boxed{
Air \rightarrow Contact \rightarrow Air \rightarrow Board.
}
\]

This produces a clean operator graph:

- pure clean-air edges use clean-air bounds,
- contact-assisted routes use explicit contact operators,
- wall taps, ramp grazes, or intentional intermediate contacts become first-class route states if useful,
- certified bounds are never applied outside the domain for which they were proved.

The earlier f2 graze hypothesis turned out to be wrong—those contested flights were clean—but the architectural rule remains important for future maps.

---

# 14. Face-to-face transfer becomes operator composition

Suppose the current board state lies on face \(F_i\) and face \(F_j\) is a plausible next face.

First compute the current-face exit frontier:

\[
\mathcal R_{F_i}(S_i).
\]

For each useful exit state \(E_m\), compute the next-face entrance frontier:

\[
\mathcal E_{F_j}(E_m).
\]

The composed leg operator is:

\[
\boxed{
\mathcal L_{i\rightarrow j}
=
\mathcal E_{F_j}\circ\mathcal R_{F_i}.
}
\]

A returned successor contains:

- current ramp witness,
- clean-air witness,
- board contact,
- exact resulting board state on \(F_j\),
- total tick cost,
- exact cumulative energy ledger.

The old software concept of a “leg” therefore changes from a procedural search routine into a reusable state-transition operator.

---

# 15. The global objective should be finish tick, not a hand-tuned energy/time score

The old assembler ranked survivors with an expression such as:

\[
-\frac{carry^2}{1000}+0.9\,tick.
\]

That was a pragmatic attempt to estimate downstream value with limited lookahead.

Once local transition frontiers and global search exist, the final objective should be simply:

\[
\boxed{
\min T_{finish}.
}
\]

Energy is still crucial, but it is a **physical resource/state variable**, not the final objective.

Energy changes:

- future reachability,
- possible ramp exits,
- flight time,
- achievable terminal headings,
- ability to skip faces.

The solver should not need an arbitrary global exchange rate between “one unit of energy” and “one tick.”

Heuristic scores may still be used to order proposal evaluations. They must never decide the final route when a more exact global value comparison is available.

---

# 16. The global map problem is a Bellman / shortest-path problem over exact boundary states

Let:

\[
V(S)
\]

mean the minimum remaining ticks required to finish from exact boundary state \(S\).

Then:

\[
\boxed{
V(S)=\min_{S'\in\mathcal T(S)}
\left[
\Delta t(S,S')+V(S')
\right],
}
\]

where \(\mathcal T(S)\) is the set of exact useful successor states produced by local operators.

For a terminal state inside the end zone:

\[
V(S_{zone})=0.
\]

This is the clean mathematical replacement for:

- local energy bars,
- top-3 beams,
- depth-2 probing,
- first-tap weighted selection,
- arbitrary energy/time ranking.

---

# 17. Best-first branch-and-bound global search

In practice, the global solver should run as an A*/best-first branch-and-bound search with lazy local operator expansion.

Each partial route has:

- actual ticks used \(g(S)\),
- an admissible optimistic lower bound on remaining ticks \(h(S)\).

Priority:

\[
\boxed{
f(S)=g(S)+h(S).
}
\]

Maintain the best complete route found so far:

\[
T_{incumbent}.
\]

A partial branch may be hard-pruned only if:

\[
\boxed{
g(S)+h(S)\ge T_{incumbent}.
}
\]

assuming \(h\) is genuinely optimistic/admissible.

This is the global equivalent of the local lower/upper-bound sandwich.

---

# 18. Route topology: keep the face graph, remove the top-10 shape commitment

The old pipeline performs best-first enumeration of face sequences, deduplicates to base shapes, and keeps the top 10 before expensive solving.

That is unsafe for a perfect solver because a loose closed-form estimate may rank the globally optimal shape outside the retained set.

The new face graph should instead be a **conservative successor superset**.

An edge:

\[
F_i\rightarrow F_j
\]

means:

> There may exist at least one useful transition from some exit state on \(F_i\) to \(F_j\).

It does **not** mean the transition is already proven realizable from the current state.

False positives are acceptable because local exact operators reject them.

False negatives are dangerous because they can erase the optimum.

Skips remain first-class naturally:

\[
F_1\rightarrow F_4
\]

is simply another possible successor edge.

No fixed “top N shapes” gate should stand between the exact local operators and the global optimum.

---

# 19. What survives from the old closed-form route model

A great deal of the old mathematics remains valuable—just in a different role.

Use closed-form laws for:

- optimistic maximum speed gain,
- ballistic altitude windows,
- minimum possible travel ticks,
- maximum possible reach,
- optimistic collision retention,
- optimistic future energy,
- potential/energy ledgers where independently certified.

But use them primarily as:

\[
\boxed{
\text{bounds, proposal priorities, and impossibility proofs.}
}
\]

Do not let them assign final values to states they have not jointly realized.

The key distinction is:

### Old use

> This cell/shape looks best according to separately composed optimistic quantities, so search it and maybe commit it.

### New use

> Even under optimistic physics, this region/branch cannot beat the incumbent, so it is safe to prune.

The second use is much more valuable for an exact solver.

---

# 20. Local and global lower/upper bounds

The central certification structure should exist at every level.

## 20.1 Local entrance region

For a point or region \(C\) of a destination face:

\[
L_E(C)=\text{best engine witness found in }C,
\]

\[
U_E(C)=\text{certified optimistic maximum possible value in }C.
\]

Then:

\[
L_E(C)\le H_E^*(C)\le U_E(C).
\]

## 20.2 Ramp exit region

Likewise:

\[
L_R(C)\le H_R^*(C)\le U_R(C).
\]

## 20.3 Full route

For a partial global state \(S\):

\[
T_{actual}(S)+T_{remaining}^{optimistic}(S)
\]

is a lower bound on its eventual finish tick.

A complete witness supplies an upper bound on the optimal finish time:

\[
T^*\le T_{incumbent}.
\]

Search closes the gap from both sides.

---

# 21. Adaptive spatial resolution instead of permanent fixed 32u cells

Fixed cells are useful instrumentation and a good initial discretization, but they should not define the ultimate precision of the solver.

Use hierarchical face regions.

For region \(C\), maintain:

- best witnessed lower bound \(L(C)\),
- certified optimistic upper bound \(U(C)\),
- terminal-state/frontier samples,
- witness-bank references.

If the region cannot affect the global incumbent, do not refine it.

If:

\[
U(C)-L(C)
\]

remains large and the region could change the route decision, split it spatially:

\[
C\rightarrow C_1,C_2,C_3,C_4
\]

or along coordinates natural to the planar face.

This allows:

- large simple areas to remain coarse,
- narrow f2-style corridors to receive high spatial resolution,
- tangent-frontier crossings to be refined locally,
- exact contact points to be solved without making the whole map equally expensive.

---

# 22. Adaptive control resolution inside a spatial region

Spatial refinement and control refinement are separate.

For a promising \((Q,T,\theta)\) region:

1. begin with a compact side-run / guided policy parameterization,
2. refine side-run timing if useful,
3. add guidance degrees of freedom,
4. refine \(\cos\alpha\) profiles,
5. move toward per-tick values where the bound gap or endpoint sensitivity requires it.

The f2 result shows why this must be adaptive: a coarse knot schedule can be completely misleading even when the exact per-tick trajectory is excellent.

However, raw per-tick refinement should be focused on regions that have already earned it through:

- good witnesses,
- high optimistic bounds,
- terminal frontier importance,
- unresolved global impact.

---

# 23. Efficient search of needle basins: recommended hierarchy

For difficult clean-air point targets, use the following hierarchy rather than raw random perturbation.

## 23.1 Side-topology enumeration

Because the minimum side dwell is 6 ticks, the number of plausible reversal topologies over a local flight is modest compared with unrestricted binary per-tick switching.

Enumerate or best-first search:

- starting side,
- number of reversals,
- reversal tick locations satisfying the dwell law.

Do not multiply by fictitious choices after the topology is fixed: once reversal locations and starting side are known, the side sign sequence is determined.

## 23.2 Guided policy seeds inside each topology

Generate multiple policies varying:

- endpoint attraction strength,
- desired terminal heading,
- desired normal approach,
- gain-vs-correction tradeoff,
- early/late lateral bulge preference,
- intermediate waypoint locations.

Each policy produces an actual wish schedule.

## 23.3 Canonical open-loop replay

Every generated schedule is replayed from the exact start.

Only replay results feed the frontier.

## 23.4 Outcome-space elite retention

Retain diverse successful/near-successful candidates by:

- terminal heading bin,
- normal-dot bin where useful,
- endpoint residual class,
- arrival tick branch.

Do not let one high-speed but badly oriented basin evict all other arrival classes.

## 23.5 Local polish only after a basin is entered

Once a candidate reaches the correct endpoint basin, increase control resolution and optimize post-board value directly.

This is where the f2 exact schedule already demonstrated that local improvement works.

---

# 24. The next lab after EntranceField: `ExitField`

Do not jump directly from a good entrance heatmap back to full-map solve.

Build a standalone ramp-exit laboratory first.

Given an exact board state from a real/verified witness:

\[
S_B,
\]

produce an exit field over the current ramp.

For each retained exit state record:

- exact exit point,
- exit tick relative to board,
- exact velocity vector,
- horizontal heading,
- energy,
- side/dwell state,
- canonical witness,
- any intervening contact semantics,
- bound gap.

Quality-control it exactly like the entrance field.

If a legal human ride/dismount exists from the same board state, it is a constructive lower bound. The solver must contain or beat it at the corresponding exact exit condition before the exit operator is considered trustworthy.

Suggested commands/gates:

- `exitfit` — representation/replay tests,
- `exitexact` — production search vs broad reference solve,
- `exitwitnesscheck` — replay every stored exit,
- `exitbank` — persistent monotonic witness bank,
- `exitbound` — lower/upper sandwich report.

---

# 25. Build one face-to-face operator before solving a map

Once `EntranceField` and `ExitField` are independently trustworthy, build one composed transfer:

\[
F_0\rightarrow F_1.
\]

Input:

\[
S_{board,0}.
\]

Process:

1. Generate `ExitField(S_board,0)`.
2. For each useful exit state \(E_i\), query/generate `EntranceField(E_i,F_1)`.
3. Compose the witnesses.
4. Retain exact successor board states on \(F_1\).
5. Compare production results against a broad monolithic reference search over the whole transfer.

Define:

\[
\boxed{
Transfer_{0\rightarrow1}(S_0).
}
\]

The acceptance test is not merely that it finds one good route. It should demonstrate that decomposing the motion into exit and entrance operators does not remove better realizable transitions.

Suggested gate:

`legexact` — compare composed operator frontier against a broad direct engine search over the same complete leg.

---

# 26. Then solve three faces with no fixed beam

Before full-map scale, test:

\[
F_0\rightarrow F_1\rightarrow F_2
\]

with a genuine global best-first search.

Do **not** keep only the local top 3.

Instead:

- retain all states not safely pruned by certified bounds or exact equivalence,
- lazily deepen the states whose optimistic global finish remains competitive,
- allow locally second-best entrance/exit states to prove their downstream value.

This experiment should answer an important empirical question:

> How many distinct local continuation states actually survive once downstream value is considered?

That determines how compact the practical frontier can be without relying on arbitrary beam width.

---

# 27. Start phase becomes an initial frontier, not an early commitment

The old start system uses 48 prestrafe candidates, probes the first face, deeply solves the top four, and selects a start based on the first tap it sets up.

The new system should expose:

\[
\boxed{
\mathcal P(S_{spawn})
}
\]

as a nondominated frontier of legal launch states after prestrafe + jump.

Each launch state includes:

- exact position,
- exact velocity,
- launch tick,
- side/dwell state,
- witness.

These launch states become initial nodes of the global solver.

Do not commit to one because it has the best first tap. A slightly weaker first transition can be globally faster.

The old 48 patterns can remain excellent seeds for discovering this launch frontier.

---

# 28. Ending becomes a terminal operator

Define a zone operator:

\[
\boxed{
\mathcal Z(S)=\text{minimum ticks from }S\text{ to enter the end volume}.
}
\]

It may include:

- direct clean-air entry,
- ramp ride then exit,
- lob plus ground run if legal and relevant,
- any other exact route to the terminal volume.

The objective is purely finish tick.

There is no need to force an artificial final board if entering the end zone directly is legal.

---

# 29. Safe dominance and deduplication rules

This is an area where an exact solver must be conservative.

## 29.1 Safe exact-state dominance

If two global states have identical engine-relevant physical state and control-history state, but one occurs earlier:

\[
S_A=(x,v,d,c,\ldots,t_A),
\]

\[
S_B=(x,v,d,c,\ldots,t_B),
\]

with:

\[
t_A<t_B,
\]

then \(S_B\) is safely dominated because every future control sequence available from B is available from the identical earlier state A and finishes earlier by the same amount.

## 29.2 Unsafe intuitive dominance

Do not automatically prune because one state has:

- more energy,
- more speed,
- a more tangent board,
- an apparently better heading.

Those are useful heuristics but are not universal future-dominance theorems.

## 29.3 Approximate deduplication modes

For fast exploratory solving, approximate state clustering may be useful.

Certification mode must either:

- disable approximate merges, or
- accompany them with a certified bound proving that discarded variations cannot improve the incumbent.

---

# 30. Cycles and repeated faces

The old route enumeration used a potential ledger intended to make cycles manageable/cycle-proof.

The new solver should not simply prohibit repeated faces, because a repeated contact or loop may conceivably trade time for speed and enable a faster later skip.

Instead:

- use exact-state dominance to eliminate true repeated-state cycles,
- use the incumbent finish tick as a hard horizon,
- use certified optimistic remaining-time bounds,
- retain any independently proven potential/energy cycle bound that remains valid under the exact operator semantics.

A cycle should be removed because it cannot improve the objective, not merely because the face ID appeared before.

---

# 31. Caching and canonicalization

Local exact operators are expensive, so caching matters.

## 31.1 Exact cache keys

For correctness-critical reuse, cache only when the engine-relevant start state and relevant local geometry are exactly equivalent under a proven symmetry/canonicalization.

Useful exact symmetries include:

- translation removed by local coordinates,
- horizontal yaw rotation when local geometry is rotated identically,
- identical planar geometry/neighborhood under the same collision model.

## 31.2 Approximate reuse as seeds

Nearby but non-identical states may reuse:

- prior witnesses as seeds,
- prior terminal-frontier shapes as priors,
- learned budget allocation,
- policy parameters.

But the resulting candidate must still be replayed from the exact new state.

Approximate cache reuse is therefore a speed optimization, never an authorization to reuse an old value without replay.

---

# 32. Global lazy evaluation

Do not fully solve every possible local operator before the global search starts.

Use lazy expansion.

For a global state \(S\):

1. cheap topology says which faces might matter,
2. cheap certified bounds rank which successor regions could still beat the incumbent,
3. exact entrance/exit operators are refined only on those regions,
4. newly discovered witnesses create exact successor nodes,
5. unresolved local regions remain represented by optimistic bounds,
6. global best-first search chooses where additional local compute has the highest potential to change the final answer.

This is how the solver avoids doing exact heatmaps at maximal resolution on every surface of every map.

---

# 33. Compute allocation should be driven by global value of information

The old system allocates fixed budgets such as 3,000, 8,000, or 24,000 evaluations per component.

The new solver should increasingly allocate compute based on unresolved impact.

A local region deserves more compute if:

- its upper bound can beat the best witnessed continuation,
- it lies on a globally competitive partial route,
- its local \(U-L\) gap is large,
- it contains an unresolved tangent/frontier transition,
- it is a narrow corridor with high endpoint sensitivity.

A local region deserves little or no compute if its optimistic continuation already cannot improve the incumbent map time.

This couples local accuracy to global relevance.

---

# 34. Full-map certification

The eventual exactness criterion should be explicit.

Suppose a complete witnessed run finishes in:

\[
N\text{ ticks}.
\]

Every unresolved global branch \(b\) has a certified optimistic earliest finish:

\[
LB_{finish}(b).
\]

When:

\[
\boxed{
\min_b LB_{finish}(b)\ge N,
}
\]

the run is tick-optimal.

This is stronger than any amount of brute-force confidence.

It means:

> Every route not fully solved has already been proven incapable of finishing earlier.

If the desired objective later includes a tie-break among equal-tick runs—such as highest final speed or smoothness—define that lexicographically and add corresponding bounds. Do not silently blend secondary objectives into the primary tick objective.

---

# 35. Example: why local maximum energy can lose globally

Consider a board point on face F1.

Two exact entrance witnesses are available:

| State | Post-board energy | Direction | Time to useful F1 exit | Next-face flight |
|---|---:|---:|---:|---:|
| A | 1.00M | badly cross-ramp | 28 ticks | 34 ticks |
| B | 0.96M | aligned | 15 ticks | 31 ticks |

Total to the next board:

\[
A: 28+34=62\text{ ticks},
\]

\[
B: 15+31=46\text{ ticks}.
\]

A wins the local heatmap scalar.

B wins the map by 16 ticks.

This is why `BestBoard(Q)` is an excellent local diagnostic/value, but the full-map operator may need to expose multiple continuation states at Q.

---

# 36. Example: f2 needle basin and guided proposal

Known facts from the current development case:

- legal human f2 witness under the 6-tick law,
- human post-board value about 945k,
- canonical per-tick wish replay about 949k at 0u,
- optimizer improves exact schedule to about 951k,
- tiny raw schedule perturbations fail to recover the basin.

A good search should therefore not ask:

> Which random \(\cos\alpha\) perturbation of this 46-tick vector lands closer?

It should ask something like:

> Given 46 ticks, current state, target point Q, and terminal-heading bin θ, what wish direction should be flown now so the predicted remaining trajectory stays on a reachable closing manifold while preserving as much speed as possible?

A guided controller can answer that at search time.

The candidate schedule it generates is then frozen and replayed open-loop.

If it reaches Q at 952k, the field stores the 952k open-loop witness—not the feedback law.

This approach converts an ill-conditioned high-dimensional raw-control search into a lower-dimensional policy/endpoint search while preserving exact canonical admissibility.

---

# 37. Example: f3 human-beating legal alternative

The current f3 case is a useful permanent regression example:

- human reference: about 834k with a 4-tick reversal, outside the chosen 6-tick law,
- solver: about 847k with a fully legal 6-tick schedule.

Formally the human witness is not a lower bound on the constrained optimum because it violates the law.

Qualitatively the result is important because it proves the solver can:

- obey a stricter law,
- find a different trajectory,
- and still outperform the reference.

The 847k witness should remain permanently banked.

---

# 38. Example: FastestTangent versus BestBoard

For a given Q and arrival branch, suppose the fastest feasible tangent board has:

\[
E_{tan}=900k.
\]

A faster but slightly lossy arrival has:

\[
E_{lossy}=915k.
\]

Then:

\[
H(Q)=915k,
\]

\[
H_{tan}(Q)=900k,
\]

\[
\Delta_{tan}=15k.
\]

The map solver must be allowed to use the lossy winner.

If future experiments instead find:

\[
\Delta_{tan}=0
\]

across all reachable states and a proof establishes this structurally, then the local entrance operator may later collapse to the faster tangent-root problem.

Until then, keep both quantities.

---

# 39. How the old solver maps to the new architecture

The previous system was valuable because it exposed what information matters. Much of it survives, but several heuristic decision layers should disappear.

| Old component | New role |
|---|---|
| BSP parse, brushes, planes, textures | **Keep.** Geometry ingestion remains foundational. |
| Start/end zone discovery | **Keep.** These define initial and terminal operators. |
| Surfable face extraction / face graph | **Keep as conservative topology.** Do not use it to hard-rank away routes. |
| Closed-form route enumeration | Use for admissible lower-time bounds / topology ordering, not final route choice. |
| Top-10 base-shape pool | **Delete in certification architecture.** No fixed-N topology cutoff. |
| Potential/energy ledger | Keep where independently certified; use for bounds and diagnostics. |
| 48 prestrafe candidates | Seeds for `LaunchFrontier`; do not early-commit to one. |
| Cheap first-face probe | Cheap proposal/bound stage for lazy initial expansion. |
| Deep first-transfer selection | Replaced by global evaluation of launch states through exact operators. |
| Naive board heatmap | **Replace with witness-backed EntranceField/frontier.** |
| Tangent-heading target + constructed path | Replaced by stored canonical witness. No separate executor reinterpretation. |
| Geometric board fallback families | Replaced by general wish-basis proposal/search machinery. |
| 24k transfer carve search | Replaced by `ExitField` / ramp transfer operator. |
| 19 carve spline families | May survive only as proposal seeds if useful; not as the definition of reachable motion. |
| Arrival gradient / terminal curve guidance | Useful search-time guidance; accepted result must replay as canonical witness. |
| Doom cull | Keep only where the bound is certified for the exact transition domain. |
| Unplanned-touch refusal | Keep as exact transition semantics for a clean-air operator. Intentional contacts become explicit graph transitions. |
| Single-touch law | Replace with exact contact/operator semantics; do not hide contacts inside another primitive. |
| Rideability/catch-window checks | Keep as exact feasibility conditions inside Board→Exit. |
| ≥60% map-best energy admission bar | **Delete as a hard rule.** Exact/global value decides. |
| `-carry²/1000 + 0.9*tick` | Search-order heuristic at most; never the final map objective. |
| Top-3 beam | **Replace with conservative frontier + certified pruning.** |
| Depth-2 probe | Replaced by the actual global value search. |
| Zone-mode carve | Becomes terminal zone operator. |
| Per-eval recorder / reports | **Keep and expand.** Add witness-bank, frontier, bound-gap, and global-search views. |

---

# 40. Old Phase 0: map ingestion — mostly unchanged

Retain:

1. BSP parse → brushes, planes, textures.
2. Zone identification from map data, with replay fallback.
3. Surfable face extraction → polygon, normal, centroid, extents, adjacency/context.
4. Spawn and end-zone anchoring.

Add:

- exact local coordinate frames for faces,
- explicit contact/transition node types,
- conservative face-successor graph,
- geometry signatures useful for local-operator cache reuse,
- spatial hierarchy for adaptive face refinement.

---

# 41. Old Phase 1: route enumeration — replace with lazy global topology search

Do not fully enumerate and then retain the top 10 shapes.

Instead:

1. Create the conservative face/contact topology graph.
2. Initialize the global priority queue with launch-frontier states.
3. Lazily query possible successors only when a state becomes globally competitive.
4. Use closed-form minimum-time/reach bounds as the A*/branch-and-bound heuristic.
5. Allow skips naturally.
6. Keep searching until the incumbent is certified or the chosen exploratory compute budget ends.

The concept of a “shape” can still be useful for reporting and deduplication, but it should not be a hard gate that can discard the optimum.

---

# 42. Old Phase 2: prestrafe/start — convert to `LaunchFrontier`

Instead of choosing a single start based on the first tap:

1. Generate legal prestrafe/jump candidates.
2. Replay every retained launch on the exact engine.
3. Keep useful distinct exact launch states.
4. Bank them.
5. Let the global solver determine which launch produces the fastest completed map.

Cached first-face information can still accelerate evaluation, but it should not prematurely collapse the frontier.

---

# 43. Old Phase 3a/3b: board heatmap + flight — fully replaced

The witness-backed entrance field should replace:

- gain-law arrival-speed assumptions,
- separable tangent residuals,
- turn-price heuristics,
- tangent-heading targets,
- held-heading construction,
- yaw snapping,
- family-specific board search.

The field record already contains the path.

Selecting a board state means selecting its exact witness.

Execution means replaying that witness.

There is no separate “construct a flight to the chosen board” phase anymore.

---

# 44. Old Phase 3c: carve — convert into `ExitField`

The old carve implementation contains many useful discoveries:

- exit-map thinking,
- ballistic-pair guidance,
- runway relevance,
- catch-window legality,
- exact clip/graze accounting,
- local search density based on useful regions.

Preserve those insights, but reorganize them around one definition:

\[
\boxed{
\text{From this exact board state, enumerate/solve the useful exact exit frontier.}
}
\]

Search families and gradients become proposals.

The exit witnesses become the truth.

The global solver consumes exit states, not carve scores.

---

# 45. Old Phase 3d: selection/beam — remove as final decision logic

Delete the hard 60% energy bar from final admissibility unless it is independently proven that no discarded state can be globally optimal.

Delete fixed top-3 beam truncation in certification mode.

Delete depth-2 lookahead as the final selector.

Replace them with:

- exact successor states,
- admissible global lower bounds,
- branch-and-bound,
- lazy local refinement.

The global solver itself is the lookahead.

---

# 46. Instrumentation for the new full-map solver

Retain the current rich report system and add several layers.

## 46.1 Local operator report

For each entrance/exit region show:

- lower-bound witness value,
- certified upper bound,
- gap \(U-L\),
- winning terminal/exit heading arrows,
- tangent result/gap,
- witness source,
- banked-vs-current discovery status,
- spatial/control refinement level.

## 46.2 Global search report

For each open route state show:

- actual ticks used,
- optimistic remaining ticks,
- \(f=g+h\),
- current face/contact node,
- route prefix,
- exact physical state summary,
- why it was expanded or pruned,
- bound source/certification status.

## 46.3 Certificate report

At the end:

- incumbent finish tick,
- best unresolved optimistic finish,
- exact gap,
- number of unresolved states by bound interval,
- local operators still carrying meaningful uncertainty,
- whether the result is exploratory, epsilon-certified, or exact tick-certified.

---

# 47. Recommended command/gate ladder

A concrete development ladder could be:

### Entrance

- `repfit`
- `humanexact`
- `efield`
- `witnesscheck`
- `fieldexact`
- `tangentgap`

### Exit

- `exitfit`
- `exitfield`
- `exitwitnesscheck`
- `exitexact`

### Composed leg

- `legfield`
- `legexact`

### Global

- `mapsolve --explore`
- `mapsolve --exact-local`
- `mapcert`

Each rung should have explicit PASS/FAIL/N/A semantics rather than prose-only interpretation.

---

# 48. Proposed implementation roadmap

## Stage A — Finish EntranceField

Exit criteria:

- f0/f2 admissible human floors reached at exact points,
- f2 needle accessible without injecting the exact human schedule as a production seed,
- witness replay perfect,
- terminal-heading frontier stable,
- Build-vs-reference gaps acceptably small or bounded,
- Tier-0 energy ceiling independently certified,
- reach culling remains disabled until separately repaired,
- no witness-bank regressions.

## Stage B — Build ExitField lab

Use one or two representative ramps, including a corridor/awkward case.

Exit criteria:

- exact witness replay,
- known human/legal exits reproduced or beaten,
- broad reference search agrees with retained exit frontier,
- adaptive control resolution handles difficult rides,
- 6-tick law maintained across board/ride/exit.

## Stage C — Compose one exact leg

Build:

\[
ExitField\rightarrow EntranceField.
\]

Compare against a broad direct search over the same leg.

Exit criterion:

- no meaningful direct-search successor is missing from the composed operator frontier.

## Stage D — Three-face global search

No fixed beam.

Use best-first lazy operator expansion.

Measure:

- frontier growth,
- frequency of locally suboptimal states becoming globally optimal,
- effectiveness of admissible bounds,
- cache reuse,
- compute distribution.

## Stage E — Full-map exploratory solve

Enable conservative face topology and all skip edges.

Allow a wall-clock budget, but report unresolved optimistic gap honestly.

## Stage F — Certification mode

Replace or disable every unsafe approximate pruning rule.

Refine only branches/regions that could still beat the incumbent.

Stop when exact tick-optimality is certified.

---

# 49. A concrete global-search sketch

```text
launch_frontier = BuildLaunchFrontier(spawn)
open = priority queue ordered by g_ticks + optimistic_remaining_ticks
incumbent = NONE

for S in launch_frontier:
    push(open, S)

while open not empty:
    S = pop_best(open)

    if incumbent exists and lower_bound_finish(S) >= incumbent.finish_tick:
        prune S
        continue

    if exact_state_is_dominated(S):
        prune S
        continue

    if can_enter_zone(S):
        Z = SolveZoneOperator(S)
        if Z has witnessed finish and better than incumbent:
            incumbent = Z
        continue

    for next_face in conservative_successors(S.face_or_air_region):
        # Lazy: bounds first, exact refinement only if globally relevant.
        edge_bound = optimistic_transition_bound(S, next_face)

        if incumbent exists and
           S.tick + edge_bound.min_ticks + optimistic_after(next_face) >= incumbent.finish_tick:
            continue

        successors = RefineTransferOperatorAsNeeded(S, next_face, incumbent)

        for S2 in successors.witnessed_states:
            bank(S2)
            push_if_not_safely_dominated(open, S2)

        # unresolved local regions remain represented by bounds;
        # schedule more local refinement if they could change the global result.

return incumbent + certificate_gap(open, unresolved_local_regions)
```

The important point is that an unresolved local operator region does not have to be fully solved immediately. It may exist in the global search as an optimistic bound until global relevance justifies more refinement.

---

# 50. Efficiency expectations

The new architecture attacks the old 80% carve cost in three ways.

## 50.1 Reuse local solutions

The old pipeline repeatedly searches similar ramp/air behavior during alternatives and depth-2 probes.

The operator architecture solves meaningful local transitions once, banks them, and reuses them.

## 50.2 Stop refining globally irrelevant regions

A cold local region with a loose field value may never need an exact witness if its optimistic global finish is already slower than the incumbent.

## 50.3 Replace fixed-depth lookahead with lazy global value search

Compute is spent where it can still change the final route.

This should outperform fixed 24k-per-leg budgets even if some individual local operators occasionally require much deeper search.

---

# 51. What “perfect” does and does not require

A perfect full-map solver does **not** require calculating the exact optimum at maximum resolution for every point on every face.

It requires proving that every unresolved alternative cannot beat the incumbent.

This distinction is the key to tractability.

Example:

- Current complete route: 2,400 ticks.
- An unresolved branch reaches a certain face at tick 1,900.
- Certified physics says even with perfect future gain and zero board loss, the absolute earliest possible finish from there is another 520 ticks.

Then:

\[
1900+520=2420>2400.
\]

That entire branch can be discarded without ever exactly solving its remaining heatmaps.

This is how exactness and efficiency coexist.

---

# 52. Critical invariants for the entire project

These should be treated as design-of-record laws.

## Invariant 1 — Engine replay is truth

No candidate enters a field or route as a final value without exact replay.

## Invariant 2 — Search representation is not physical admissibility

Controllers, splines, waypoint guidance, learned policies, and closed-form traces may generate candidates. They do not define what motions are legal.

## Invariant 3 — Canonical witnesses are direct controls

Aerial witnesses are direct wish schedules. Ramp witnesses should likewise be stored in the lowest-level exact input representation appropriate to the engine.

## Invariant 4 — Witness lower bounds are monotonic

Known verified solutions never disappear.

## Invariant 5 — Hard pruning requires proof

An uncertified heuristic may order work but may not erase a route.

## Invariant 6 — Contacts respect operator boundaries

A clean-air operator cannot silently contain an intermediate contact.

## Invariant 7 — Global objective is explicit

Primary objective: minimum finish tick. Secondary objectives, if any, are lexicographic and separately defined.

## Invariant 8 — Local greed is not global optimality

Do not collapse a continuation frontier solely because one state has the highest immediate energy.

## Invariant 9 — 6-tick reversal law is global

The dwell timer crosses software/operator boundaries. It does not reset at board, carve, or flight transitions.

## Invariant 10 — Certification state is explicit

Every bound and prune should know whether it is certified, empirical, heuristic, or disproven.

---

# 53. Specific guidance to the agent from the current state

The immediate recommendation is:

1. **Accept the proposed search-time feedback resolution.** Use guided/closed-loop logic to condition the f2 search, but always freeze and open-loop replay the produced `(side, cosα)` sequence before accepting it.
2. Treat f2 as the canonical ill-conditioned shooting benchmark. The banked 951k schedule is the lower bound to beat and a diagnostic basin, not a production seed.
3. Build at least one target-conditioned guided shooter whose parameters are lower-dimensional than the raw 46-tick schedule. Compare its basin-of-attraction against raw perturbation search.
4. Keep the terminal-heading frontier for fields and FastestTangent, but restore a full-strength scout pool for tiny exact-point solves.
5. Do not introduce another named physical path family. Any new mechanism is a proposal/guidance method over the same canonical wish space.
6. Keep `humanexact` exact and near rows separate.
7. Preserve all banked witnesses monotonically.
8. Keep Tier-0 reach culling off. Use the independently certified energy ceiling only for the claim it actually proves.
9. Do not port the old carve search yet. Once EntranceField is green enough, build `ExitField` from the same principles instead of merely retuning the old 19-family system.
10. When ExitField is ready, compose one face-to-face transfer and validate decomposition before touching the whole map.
11. Replace top-N shape selection, energy bars, fixed beams, and depth-2 commitment progressively with lazy best-first global search and certified pruning.
12. Design every new local operator to expose both exact witnessed successors and unresolved optimistic bounds so that full-map certification is possible later without another architectural rewrite.

---

# 54. Final target architecture

The desired system is:

\[
\boxed{
\begin{aligned}
MapGeometry
&\rightarrow ConservativeTopology\\
Spawn
&\rightarrow LaunchFrontier\\
AirState
&\rightarrow EntranceFrontier\\
BoardState
&\rightarrow ExitFrontier\\
ExitState
&\rightarrow NextEntranceFrontier\\
IntermediateContact
&\rightarrow ExplicitContactTransition\\
&\vdots\\
&\rightarrow Zone
\end{aligned}
}
\]

with global optimization:

\[
\boxed{
V(S)=\min_{S'}[\Delta t(S,S')+V(S')].
}
\]

Every realized edge has a canonical engine-replayed witness.

Every unresolved edge/region has an optimistic bound if it is to participate in hard pruning.

The final run is simply the concatenation of witnesses chosen by the globally optimal path.

The solver is complete when either:

- exploratory budget expires, in which case it reports the best witnessed run plus the remaining optimality gap, or
- the incumbent is certified against every unresolved branch, in which case it reports an exact tick-optimal solution.

That is the path from the current heatmap work to the actual project goal: **an efficient full-map solver that reasons globally over exact locally solved physics, rather than trying to discover a 60-second run as one giant tick-level trajectory.**


---

# Appendix A — Current measured entrance-field status at handoff time

This appendix is a snapshot of the development state that motivated the guidance at the top. It is not a permanent specification; numbers will change as the search improves. Keep the qualitative lessons and update the measurements.

## A.1 Canonical representation status

### Old heading-command basis

**Rejected as a complete control representation.**

Constructive unit test:

- flying the human's own realized per-tick headings through the tracking controller missed known contacts by approximately 66–282u,
- therefore even “handed the answer,” that control channel could not reproduce the engine trajectory,
- the previously observed mirror/tracking desync is a structural defect for exact optimality claims.

### Direct wish basis

**Accepted as the canonical aerial basis.**

`Air::FlyWishSchedule` uses direct per-tick wish direction represented as:

\[
(side,\cos\alpha)
\]

relative to current horizontal velocity.

Important implementation finding:

- the controller/input mapping realized its wish at `wh + π` in the engine's `WishFromInput` coordinates,
- tape inversion therefore had to use the same mapping,
- before correction, reconstructed flights grounded or missed by roughly 260–330u,
- after correction, per-tick wish reconstruction reproduced the known human flights at the intended contacts.

The executor may fly any supplied schedule; admissibility is imposed by the search. Therefore distinguish:

- `basis_representable`,
- `6tick_admissible`.

The f3 human witness is an example of:

- basis representable: YES,
- 6-tick admissible: NO,

because the recorded human control contains a 4-tick reversal.

---

## A.2 Human reference status

The human tape is **quality control, not a route target**. It is a decent line, not an elite/known-optimal line. Therefore a mature solver should generally outperform it.

When the human witness is legal under the current solver law and starts from the exact same state, it provides a constructive lower bound:

\[
E_{human}\le H_{true}(Q_h).
\]

### f0

Known human value:

\[
E_h\approx921k.
\]

Human reversal behavior is admissible under the 6-tick law (approximately 21-tick dwell).

Recent production search reached approximately:

\[
898k
\]

within about 7u of the literal contact before a later frontier-search regression reduced rediscovery quality.

Status:

- human exact-point floor: **not yet formally cleared**,
- near-point result: useful lower-bound/search regression anchor,
- bank old valid witnesses rather than allowing implementation changes to erase them.

### f2

Known human value:

\[
E_h\approx945k.
\]

Human reversal behavior is admissible under the 6-tick law (approximately 25-tick dwell).

The decisive representation/search ladder found:

- coarse/interpolated K = 1..17 knot fits fail non-monotonically,
- K = per-tick reproduces the exact contact at approximately 949k,
- local optimization started from the exact per-tick schedule improves to approximately **951k**,
- perturbation recovery fails 0/8 even at very small radius.

Status:

- control basis: **contains a human-beating solution**,
- local polish: **can improve that solution**,
- production accessibility: **still unresolved**,
- this is the canonical needle-basin benchmark.

The 951k exact replay should be permanently banked as a known lower bound at that exact state/contact.

### f3

Human value:

\[
E_h\approx834k.
\]

Human witness contains a 4-tick reversal and is therefore outside the current 6-tick law.

A legal solver witness was found at approximately:

\[
847k,
\]

with a softer/better approach than the human in the measured run.

Status:

- formal human lower-bound gate: N/A under 6-tick law,
- qualitative/regression value: extremely high,
- the legal 847k witness must remain permanently banked.

---

## A.3 Current frontier/search structure

The entrance solver currently includes:

- canonical per-tick wish execution,
- adaptive side-run representation,
- cosα knot refinement toward per-tick resolution,
- deterministic seed grids/restarts,
- terminal-heading frontier with approximately 24 bins,
- per-bin elites,
- a separate scout lane/pool for target acquisition,
- one solve feeding both BestBoard and FastestTangent,
- replay-verified witnesses,
- exact/near contact classification,
- persistent witness bank with monotonicity-break detection.

The terminal-heading frontier is conceptually aligned with:

\[
\theta\mapsto s_A^*(Q,T,\theta),
\]

but tiny point-target discs require stronger general scouts than broad face fields do.

---

## A.4 Current replay status

Recent migration runs reported full replay consistency across stored field witnesses (for example 118/118 in one run).

This is an essential but limited guarantee:

> The stored values are true for their stored witnesses.

It does **not** prove:

> The search found the best witness.

Keep those two gates separate forever.

---

## A.5 Current tangent status

`BestBoard` and `FastestTangent` now share the same direct-wish search basis.

Candidate “Case D” labels must remain **candidate** unless tangent infeasibility is established over a sufficiently complete/certified reachable frontier.

Do not interpret an empty tangent slot from an incomplete search as proof that:

\[
\mathcal A\cap\mathcal B=\varnothing.
\]

The long-term tangent question remains:

\[
\text{Can a reachable lossy arrival ever beat the maximum-energy reachable tangent arrival?}
\]

Until proved otherwise:

- compute unrestricted BestBoard,
- compute FastestTangent,
- store `tangent_gap`.

---

## A.6 Current Tier-0 status

Historical Tier-0 contained multiple claims that were once bundled together.

Current status:

### Reachability

**Uncertified / contradicted by clean-air witnesses.**

Clean flights exist in contested f2/f3 cells that Tier-0 marks unreachable.

Therefore it may not hard-cull those cells.

### Energy ceiling

**Independently re-certified on the contested cases according to the current test ladder.**

It may continue to serve as an optimistic ceiling only for the property actually proven.

This separation is a model for all future analytic bounds: certify claims individually.

---

# Appendix B — Historical pre-heatmap full-map solver baseline

This is the system that existed before the witness-backed heatmap redesign. It is included so future work can preserve useful infrastructure while recognizing exactly which heuristic commitments the new architecture is intended to replace.

**Important:** statements below describe the historical implementation and its own measured/certification claims at that time. They should not be automatically inherited as currently certified after control-law and operator changes.

---

## B.1 Phase 0 — Map ingestion (once, seconds)

1. **BSP parse**
   - brushes,
   - planes,
   - per-side textures.

2. **Zone identification from map data**
   - green-dominant texture (`channel > 2× both others`) = start brush,
   - red-dominant texture = end brush,
   - fallback: replay reference tape to its finish,
   - no lore/manual face IDs intended.

3. **Feature extraction → face graph**
   - every surfable plane (normal too steep to walk) becomes a face,
   - store polygon,
   - normal,
   - centroid,
   - extents.

4. **Zone anchoring**
   - spawn state from anchor tape,
   - end volume = end brush top,
   - hull-expanded XY,
   - Z just under the top,
   - no boarding requirement for the end volume.

This phase largely survives.

---

## B.2 Phase 1 — Route enumeration (fast, closed-form)

Historical route enumeration:

5. **Best-first search over face sequences** under a potential ledger.

Historical potential form:

\[
E' = E + 900\cdot ticks + 2g\cdot\Delta z.
\]

The intent was telescoping anchors and cycle-safe comparison.

Candidate edge admission used closed-form reachability:

- ballistic altitude window,
- gain-law distance.

The end edge allowed landing short and running.

Skips were first-class.

6. **Dedup to distinct base shapes** using first-occurrence face order.

Keep the top 10 by lower bound.

Shapes attempted in that order under one approximately **170-second wall budget**.

### New disposition

- Keep conservative topology generation.
- Keep certified optimistic bounds where still valid.
- Remove the hard top-10 shape cutoff for exact/certification mode.
- Use shapes as reporting/grouping concepts, not global-optimum admission gates.

---

## B.3 Phase 2 — Per-shape start

7. **48 prestrafe candidates**

Historical grid:

- 3 launch offsets,
- 4 spin rates,
- 2 directions,
- 2 hold lengths.

Each candidate:

- prestrafe circle,
- one legal jump,
- exact-engine simulation.

8. Each candidate received a **cheap first-face probe**:

- approximately 800 evaluations,
- ranked by board quality.

9. Top 4 received a **deep probe**:

- air ≈ 3000 evaluations,
- carve ≈ 8000 evaluations.

The historical start was chosen by the energy of the first tap it set up, not merely its immediate state.

Result cached per first face across shapes.

### New disposition

Convert this to a `LaunchFrontier`. Keep these candidates as seeds; do not commit globally based on the first transfer.

---

## B.4 Phase 3 — Per-leg core loop

A historical leg was:

\[
\text{flight onto face }k
\rightarrow
\text{ride face }k
\rightarrow
\text{transfer toward }k+1\text{ or zone}.
\]

---

## B.5 Historical 3a — Board heatmap

Per 32u cell, the old field computed closed-form/relaxed quantities including:

- exact ballistic arrival times, both roots,
- gain-law arrival speed,
- tangency-law minimum unavoidable loss,
- tangent heading,
- turn need versus free-turn budget,
- path price for turn demand beyond the free budget,
- braking exchange charged at cheapest modeled cost per radian.

Historical value:

> mechanical energy after the estimated best board (kinetic minus unavoidable loss plus height above face bottom) minus path price.

Runway per cell compared:

- available space toward next objective,
- law-required space.

Historical selection ladder:

1. viable + free-turn,
2. viable,
3. free-turn,
4. any.

### Failure mode exposed by the new work

The field combined optimistic quantities **separably**. It did not prove one trajectory jointly achieved:

- endpoint,
- speed,
- terminal heading,
- turn history.

This produced hot cells describing states no real path could deliver.

### New disposition

Replace with witness-backed `EntranceField`.

---

## B.6 Historical 3b — Board flight

Unless previous tap already boarded the face:

### Constructed-first path

The ladder's best cell supplied:

- target position,
- tangent heading,
- arrival tick.

Historical path solver:

- turn-hold-turn profile,
- held-heading scan across arrival-tick band.

If trace landed within ~150u:

- engine line-search flew up to 7 real held-heading variants,
- first strike under energy cap returned immediately.

Open geometry could therefore produce boards in only 1–7 real evaluations.

A measured easy face-0 example produced approximately dot −110 vs human −115 in one evaluation.

### Search fallback

- 5 geometric init families,
- tangent-arrival seeds,
- basin hopping,
- ~3000 evaluations,
- front-side region gradient,
- graze-through,
- returned top 3 diverse board alternatives.

### New disposition

The selected entrance state already carries the witness. There is no separate path construction phase.

---

## B.7 Historical 3c — Transfer carve

This was the dominant cost center:

- approximately **24,000 evaluations per board alternative**,
- 6 knots,
- dual spline domains.

### Historical cell schedule

- hot cells (≥85% of map best): scheduled 3×,
- warm cells (≥60%): 1×,
- cold/non-viable: never.

Search density was therefore driven by map heat density.

If raw entry could not ballistically see the tap face because a climb came first, the exit map's best departure computed the map instead (“blind-entry fix”).

### Historical ballistic-pair guidance

Every exit-map sample's crossing computed:

- exact landing,
- dot,
- kept energy.

The best bar-passing pair set:

- aim,
- exit heading,
- true arrival tangent.

### Historical 19 spline families

The search included families such as:

- holds,
- pursuits,
- S-carves,
- runway rides,
- duck-off,
- objective-side,
- tangent-arrival,
- snakes to arrival heading,
- mirror branch,
- and other family variants within the total 19.

Each family had its own budget.

Each evaluation was pulled toward its scheduled cell by an **arrival gradient** combining:

- distance to the cell,
- turn-distance to the cell's tangent heading at the free rate.

A head-on line could therefore carry ~1200 units of modeled unabsorbed turn debt.

### Historical laws inside each candidate

#### Doom cull

Checked:

- every separation,
- after every graze,
- every 8th airborne tick.

If target was considered provably unreachable under the historical hull-padded reach bounds (or zone platform-reach bound), candidate terminated as `DOOMED` while preserving gradient information.

Historical bench claim: 0.0% false kills under that test set.

Given later Tier-0 reach failures in the redesigned field, do not generalize this historical certification without re-testing the exact bound/domain.

#### Terminal curve plan

At each separation:

- 17-heading fan of free-rate curves,
- arithmetically traced to first in-polygon crossings,
- best plan engaged only if arrival retained ≥60% of arrival energy,
- osculating executor advanced heading at free rate then held,
- intended to make late crank unrepresentable.

#### Single-touch law

A graze during engaged arc killed the candidate.

#### Unplanned-touch refusal

Tap contact without an engaged arc was never accepted as a board.

#### Rideability law

Landing had to remain on polygon through a 3-tick catch window.

#### Full contact accounting

Graze/clip accounting was used so the energy ledger closed.

### Historical carve search score

Used for basin shaping, not intended as final decision:

- strike loss at full value,
- minus a small carry term,
- slam should rank below developing arc,
- zone mode scored by arrival tick only.

### New disposition

Convert this entire problem into `ExitField` / Board→Exit transfer search. Preserve useful guidance and exact legality checks as proposal/verification machinery, not as an assortment of path-family definitions.

---

## B.8 Historical 3d — Selection and beam

Historical admission:

- candidate tap had to retain ≥60% of map-best energy,
- otherwise it was “not a board.”

Historical survivor ranking:

\[
-\frac{carry^2}{1000}+0.9\cdot tick.
\]

Top 3 board×carve composites each received a depth-2 probe:

- next leg solved from their landing,
- approximately two-thirds normal budget.

Best whole transfer committed.

Per-leg energy ledger printed:

\[
E_{in}\rightarrow E_{out}
\mid board
\mid clips
\mid grazes
\mid tap
\mid resid.
\]

### New disposition

- remove fixed 60% admission as a hard global law,
- remove the final weighted carry/time score,
- remove fixed top-3 beam in certification mode,
- remove depth-2 probe as final selector,
- let exact global best-first search provide the lookahead.

---

## B.9 Historical 3e — Ending

Last leg used zone-mode carve:

- ride through exit and flight,
- end by entering end volume,
- score purely by arrival tick,
- crest-decelerated domain estimate,
- lob-plus-run fallback.

Failed shape kept deepest partial and exported a playable `.tas`.

### New disposition

Preserve partial export, but model finishing as an exact terminal zone operator evaluated by the global search.

---

## B.10 Phase 4 — Solve-wide instrumentation

Historical instrumentation already included:

- every evaluated trajectory streamed to recorder,
- reservoir per stage,
- per-point speeds for energy readout,
- board heatmaps rendered once per tap face,
- family win-rate histogram,
- timestamped report auto-save,
- partial `.tas` beside anchor tape.

This infrastructure should remain and be extended rather than replaced.

Add:

- canonical witness IDs,
- persistent witness-bank state,
- search-source tags,
- lower/upper bounds per region,
- terminal-state frontier visualization,
- global branch-and-bound queue/certificate visualization.

---

## B.11 Historical time distribution

Measured rough solve cost before redesign:

- ~5% start (cached),
- ~10% board flights,
- ~80% transfer carves,
- ~5% endings/overhead.

This is why `ExitField` is expected to be the largest architectural performance win after `EntranceField`: it attacks the component consuming most compute while replacing repeated family search with reusable exact boundary transitions.

---

# Appendix C — Questions the agent should answer as the new architecture is built

These are not blockers to beginning the work; they are the empirical/theoretical questions that determine how aggressively the solver can simplify later.

1. **How wide can guided-shooting basins be made on f2 without injecting the known schedule?**
2. **What terminal-heading/normal-dot resolution is sufficient before adaptive refinement?**
3. **Can FastestTangent ever lose to BestBoard under exact wish control, and by how much?**
4. **How many entrance continuation states at one Q actually survive one-ramp downstream evaluation?**
5. **How many exit continuation states at one X actually survive one-leg downstream evaluation?**
6. **Can safe dominance beyond identical-state-earlier-time be proven?**
7. **Which old potential/reach bounds remain certified after direct-wish migration?**
8. **How often do intentional intermediate contacts matter enough to deserve explicit graph nodes?**
9. **How much local operator reuse is available after translation/yaw canonicalization?**
10. **What fraction of full-map compute is ultimately spent closing certificate gaps versus discovering the incumbent?**
11. **Does the globally optimal route ever use a locally non-max-energy entrance?** This should be expected and explicitly measured.
12. **Does the globally optimal route ever use a locally non-max-energy exit?** Same reason.
13. **How frequently do face skips change after exact local operators replace the closed-form route ranking?**
14. **Can the final integer-tick optimum be certified while leaving large low-value portions of face fields unresolved?** This is expected to be the main path to tractability.

