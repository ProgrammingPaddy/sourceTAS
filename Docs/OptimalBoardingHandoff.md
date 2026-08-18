# Surf Aerial-to-Ramp Optimal Boarding Solver

**Detailed engineering handoff: exact per-point entrance heatmaps,
aerial path optimization, and fastest feasible tangent boards**

Purpose: replace naive geometric heatmaps and global tick-by-tick route
search with locally exact boundary-value transfer functions.

| **Central guarantee —** For a fixed start state S0, a heatmap point Q is valued by the best board that actually exists from that start after the aerial path to Q has itself been optimized. Terminal speed and terminal angle are not guessed independently. |
|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|

| **Secondary stored result —** For every Q, also compute FastestTangent(Q): the highest-energy / fastest terminal state that is both aerially reachable and tangent-valid. Do not assume yet that this tangent result always dominates every lossy board. |
|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|

# 1. Executive summary

**The problem.** For a known airborne starting position and velocity,
find the best possible way to contact a specific point on a destination
surf ramp. The aerial path changes the speed obtained at impact; that
speed changes the tangent-board angle; changing the terminal angle
changes the required aerial path and therefore changes the achievable
speed. Solving speed, angle, and path sequentially creates an apparent
circular dependency.

**The resolution.** Do not preselect an ideal board angle. For each
candidate board point Q and arrival branch/time T, define an aerial
boundary-value function that returns the maximum terminal speed
achievable for each requested terminal heading theta. Compose that
output directly with the exact board/collision response. The winning
terminal angle, speed, and witness path are outputs of a single
constrained optimization.

| H(Q \| S0) = max over legal aerial paths from S0 that contact Q of \[post-board energy\] |
|------------------------------------------------------------------------------------------|

In addition, compute the fastest feasible tangent board as the
highest-energy member of the intersection between the aerially reachable
terminal-state set and the tangent-board manifold:

| FastestTangent(Q,T) = argmax \|\|v^-\|\| subject to v^- in A(S0,Q,T) ∩ B(T,n) |
|-------------------------------------------------------------------------------|

The global route solver can then reason primarily in terms of entry and
exit points on faces. Tickwise controls remain inside local transfer
functions and are retained only as witness trajectories.

# 2. Known values at the start of an aerial transition

Before solving a destination board, the following are known or fixed by
the engine and map geometry.

| **Known quantity**      | **Symbol / representation**                           | **Role**                                                                                              |
|-------------------------|-------------------------------------------------------|-------------------------------------------------------------------------------------------------------|
| Starting position       | p0 = (x0, y0, z0)                                     | Exact world-space point at the beginning of the airborne segment.                                     |
| Starting velocity       | v0 = (vx0, vy0, vz0)                                  | Exact velocity inherited from the prior ramp/exit.                                                    |
| Horizontal start speed  | s0 = sqrt(vx0^2 + vy0^2)                              | Magnitude of the initial horizontal velocity.                                                         |
| Vertical start velocity | vz0                                                   | Vertical component at the beginning of the flight.                                                    |
| Tick interval           | Δt                                                    | For 66 tick, nominally 1/66 s; use the exact simulation timing used by the predictor.                 |
| Gravity                 | g                                                     | The exact Source gravity behavior used by the movement predictor.                                     |
| Air-movement constants  | airaccelerate, wish-speed behavior, etc.              | Must come from the exact physics implementation.                                                      |
| Input restrictions      | L/R controls, reversal-count and/or dwell constraints | Defines the admissible aerial control space.                                                          |
| Destination face        | plane normal n, plane constant d, polygon bounds      | The exact target brush/plane geometry.                                                                |
| Collision model         | exact clip/contact behavior                           | Used to evaluate energy retained after contact; single-face tangency is the analytically simple case. |

| **Implementation rule —** The exact Source movement predictor is the source of truth. Closed-form equations in this document are used to expose dimensional reductions and constraints; the production solver should evaluate final trajectories and collisions with the exact discrete engine model. |
|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|

# 3. Canonicalize coordinates before solving

Two variables can be removed immediately because free-air movement and
planar collision physics are invariant to horizontal translation and
world-yaw rotation.

1.  Translate coordinates so the starting position becomes the origin:
    p0 -\> (0,0,0).

2.  Rotate the horizontal coordinate frame so the initial horizontal
    velocity points along +X.

| v_h0 -\> (s0, 0) |
|------------------|

| v0 -\> (s0, 0, vz0) |
|---------------------|

Transform the destination ramp geometry into the same local frame.
Absolute world position and absolute world yaw have now disappeared from
the local aerial solve. The remaining start-dependent physical
quantities are primarily horizontal speed s0 and vertical velocity vz0,
plus the destination geometry relative to the canonical frame.

# 4. Exact meaning of one entrance-heatmap point

Choose one physical point Q on the destination ramp face:

| Q = (xQ, yQ, zQ) |
|------------------|

The value stored at Q must answer this exact question:

| **Heatmap-point question —** Starting from the known state S0, what is the maximum post-board energy obtainable by any legal aerial trajectory that contacts the target face at exactly Q? |
|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|

The heatmap point does not start with a chosen board angle, a chosen
impact speed, or a chosen yaw history. Those are solved. A valid result
should store at least:

- maximum post-board energy E+\*

- winning pre-impact velocity v-\*

- winning horizontal terminal heading theta\*

- winning impact time/tick branch T\*

- winning aerial witness inputs U\*

- fastest feasible tangent result, if one exists

# 5. Gravity is a dimensional reducer, not an extra search axis

During a free-air segment, the horizontal strafe controls do not
independently control vertical motion. Once z0, vz0, gravity, and
elapsed time are fixed, vertical position and vertical velocity are
deterministic.

Continuous equations illustrate the structure:

| z(t) = z0 + vz0 t - (1/2) g t^2 |
|---------------------------------|

| vz(t) = vz0 - g t |
|-------------------|

In production, use the exact Source tickwise update and collision timing
instead. The conceptual consequence is unchanged: selecting a physical
candidate point Q with height zQ constrains the possible arrival
time(s).

| solve z(T) = zQ |
|-----------------|

Possible results are: no valid arrival branch, one branch, or two
branches (for example ascending and descending). For every valid branch
T:

| vzQ = vz(T) |
|-------------|

Therefore vertical impact velocity is not an independent optimization
variable. It is implied by the board point and arrival branch.

## 5.1 Alternative face parameterization: time plus one face coordinate

For a planar destination face n·p = d, choosing impact time T determines
z(T). Intersecting the face with the horizontal plane z = z(T) gives a
line:

| nx x + ny y = d - nz z(T) |
|---------------------------|

A candidate contact can therefore be parameterized as Q(T, lambda),
where lambda is one coordinate along that line. This is useful when
generating a face heatmap from time slices: gravity fixes height and
vertical velocity; only a one-dimensional position remains on each
slice.

# 6. Board physics: the ramp supplies a response function, not one fixed angle

Let the target plane have unit normal:

| n = (nx, ny, nz) |
|------------------|

Let the complete pre-impact velocity be:

| v^- = (vx, vy, vz) |
|--------------------|

For an ideal tangent board, the incoming normal component is zero:

| n · v^- = 0 |
|-------------|

If the exact collision implementation requires a small inward normal
component c to ensure a valid collision/contact, generalize the
condition to:

| n · v^- = c |
|-------------|

After gravity has fixed vz at the candidate arrival branch, the tangent
condition becomes a linear constraint in horizontal terminal-velocity
space:

| nx vx + ny vy = c - nz vz(T) |
|------------------------------|

This is a line in (vx, vy) space. Therefore the ramp does not provide
one ideal angle. It provides a one-dimensional family of terminal
horizontal velocities that are tangent-compatible.

![Figure 1](surf_optimal_boarding_solver_handoff_assets/figure_1.png)

*Figure 1. Schematic terminal-velocity space. The aerial solver provides
the reachable terminal set/frontier A; the ramp provides the
tangent-compatible line B. Their intersection contains tangent boards
that actually exist from the current start.*

## 6.1 Speed and tangent angle are coupled

Write horizontal terminal velocity as speed s and heading theta:

| vx = s cos(theta), vy = s sin(theta) |
|--------------------------------------|

The tangent condition becomes:

| s \[ nx cos(theta) + ny sin(theta) \] + nz vz(T) = c |
|------------------------------------------------------|

Thus changing s changes the theta required for tangency. This is exactly
the coupling that makes a naive “precompute the ideal angle, then path
to it” design incorrect.

## 6.2 Closed-form tangent-speed function

For a fixed terminal heading theta and fixed vz(T), the horizontal speed
required for a tangent board is:

| s_tan(T,theta) = \[c - nz vz(T)\] / \[nx cos(theta) + ny sin(theta)\] |
|-----------------------------------------------------------------------|

Only branches with a physically valid denominator/sign and positive
speed are meaningful. Equivalent local-normal notation can be used if
desired. This function is start-independent except through vz(T): it is
a cheap geometric/collision requirement supplied by the ramp.

# 7. The aerial boundary-value function that removes the circularity

For a fixed starting state S0, candidate point Q, arrival branch/time T,
and requested horizontal terminal heading theta, define:

| s_A\*(S0,Q,T,theta) = maximum horizontal speed achievable at Q,T while ending at heading theta |
|------------------------------------------------------------------------------------------------|

Formally:

| s_A\* = max over legal control histories U of \|\|v_h(T)\|\| |
|--------------------------------------------------------------|

subject to:

- p(T; U) = Q

- arg(v_h(T; U)) = theta

- the exact Source movement equations

- the permitted L/R reversal count and/or dwell-time constraints

- any other hard feasibility constraints used by the TAS

The aerial solver should also return the witness controls U\*(Q,T,theta)
that achieve the optimum. The path complexity is therefore hidden inside
one function:

| theta -\> s_A\*(Q,T,theta) + witness path |
|-------------------------------------------|

This resolves the apparent speed/angle/path feedback loop. Speed is no
longer guessed. For each terminal angle, the aerial solver tells us the
greatest speed that is consistent with both the endpoint and that
terminal angle.

# 8. Primary per-point solve: maximize actual post-board energy

For a requested terminal heading theta, the complete pre-impact velocity
is assembled from the aerial solver and gravity:

| v^-(theta) = ( s_A\*(theta) cos(theta), s_A\*(theta) sin(theta), vz(T) ) |
|--------------------------------------------------------------------------|

Pass this exact state into the board/collision response. Define the
exact board-value function:

| J_Q(T,theta) = E_after( v^-(theta), target_face ) |
|---------------------------------------------------|

Then the primary heatmap value is:

| H(Q \| S0) = max over valid arrival branches T and terminal headings theta of J_Q(T,theta) |
|--------------------------------------------------------------------------------------------|

The winning angle and time are outputs:

| (T_Q\*, theta_Q\*) = argmax J_Q(T,theta) |
|------------------------------------------|

and the winning speed/path are then:

| s_Q\* = s_A\*(S0,Q,T_Q\*,theta_Q\*) |
|-------------------------------------|

| U_Q\* = U\*(S0,Q,T_Q\*,theta_Q\*) |
|-----------------------------------|

| **Why this is better than the naive heatmap —** The naive design can rate an “ideal board” using a speed/angle pair that is not jointly compatible with the aerial path required to reach Q. The corrected design evaluates only terminal states that the aerial solver can actually produce from S0, and it maximizes the exact composed air-path + collision result at each Q. |
|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|

## 8.1 Simple single-plane energy equation

If the relevant collision is a simple unit-normal projection, the
illustrative update is:

| v^+ = v^- - (n · v^-) n |
|-------------------------|

| \|\|v^+\|\|^2 = \|\|v^-\|\|^2 - (n · v^-)^2 |
|---------------------------------------------|

This makes normal velocity the directly lost component. However, the
production value function should call the exact collision predictor,
especially for edge/crease contacts, overbounce behavior, multiple
planes, and any subtleties of actual surf contact.

# 9. Secondary solve: fastest feasible tangent board

For each Q and arrival branch T, also compute the fastest tangent board
that is actually reachable from S0. This is useful both as a result and
as a possible future simplification of the primary heatmap if tangent
dominance can be proven.

Define the aerially reachable terminal set/frontier:

| A(S0,Q,T) = set of nondominated terminal horizontal velocities reachable at Q,T |
|---------------------------------------------------------------------------------|

Define the tangent-compatible manifold:

| B(T,n) = { v_h : nx vx + ny vy = c - nz vz(T) } |
|-------------------------------------------------|

Then:

| TangentCandidates(Q,T) = A(S0,Q,T) ∩ B(T,n) |
|---------------------------------------------|

| FastestTangent(Q,T) = argmax over TangentCandidates of total impact energy (or \|\|v^-\|\|) |
|---------------------------------------------------------------------------------------------|

Geometrically, this is the furthest/highest-energy point where the
tangent line still lies inside or intersects the aerial capability
boundary.

## 9.1 Equivalent one-dimensional root formulation

If the aerial solver is exposed as s_A\*(theta), compare it directly
with the tangent-required speed s_tan(theta). A tangent board at theta
is feasible if the required tangent speed belongs to the set of speeds
attainable at that angle. Under the common useful condition that lower
speeds can be produced continuously below the maximum, feasibility is
approximately:

| s_tan(T,theta) \<= s_A\*(S0,Q,T,theta) |
|----------------------------------------|

The fastest feasible tangent commonly lies at the boundary where the two
curves meet:

| s_A\*(S0,Q,T,theta\*) = s_tan(T,theta\*) |
|------------------------------------------|

Equivalently define:

| G_Q(T,theta) = s_A\*(S0,Q,T,theta) - s_tan(T,theta) |
|-----------------------------------------------------|

and solve for the outer/highest-energy root G_Q = 0. This is a cheap
one-dimensional root or bracketed-search problem once the aerial speed
function is available.

![Figure 2](surf_optimal_boarding_solver_handoff_assets/figure_2.png)

*Figure 2. Illustrative angle-space view. The fastest feasible tangent
appears where the tangent-required speed reaches the boundary of what
the aerial path can deliver.*

## 9.2 Important unresolved theorem: does fastest tangent always win?

Do not silently replace the primary heatmap with FastestTangent(Q) until
this is proven or exhaustively falsified under the exact Source model:

| Question: Can any reachable non-tangent terminal state produce greater post-board energy than the maximum-energy reachable tangent state at the same Q? |
|---------------------------------------------------------------------------------------------------------------------------------------------------------|

Tangency maximizes retention for a fixed incoming speed vector
magnitude, but it does not automatically maximize absolute energy
remaining when a non-tangent path can arrive faster. Therefore the safe
architecture is:

3.  Compute BestBoard(Q) by maximizing exact post-board energy across
    all reachable terminal states.

4.  Compute FastestTangent(Q) separately as the best reachable tangent
    state.

5.  Compare them and collect counterexamples or evidence.

6.  Only collapse BestBoard(Q) to FastestTangent(Q) if a mathematical
    proof or exhaustive model-specific argument establishes dominance.

# 10. What is searched, what is solved, and what is merely evaluated

| **Quantity**                   | **Role**                                                                                      | **Interpretation**                                                                                                          |
|--------------------------------|-----------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------|
| Candidate face point Q         | Outer face sampling / continuous optimization                                                 | Defines the heatmap location.                                                                                               |
| Arrival branch/time T          | Solved from exact vertical motion for Qz, or enumerated over a very small discrete branch set | Gravity removes vz as an independent variable.                                                                              |
| Vertical impact velocity vz(T) | Solved deterministically                                                                      | Never an independent aerial search coordinate.                                                                              |
| Terminal heading theta         | One-dimensional outer optimization/root variable per Q,T                                      | Connects endpoint steering to board geometry.                                                                               |
| Aerial terminal speed s_A\*    | Optimization output                                                                           | Maximum speed achievable while satisfying Q,T,theta.                                                                        |
| L/R reversal topology          | Discrete local search                                                                         | Initial side, reversal count, reversal tick locations; plus any continuous steering parameters not analytically eliminated. |
| Board response                 | Cheap deterministic evaluation                                                                | Exact collision function maps terminal state to post-board state/energy.                                                    |
| Fastest tangent                | Intersection/root solve                                                                       | Find outer/highest-energy A ∩ B member.                                                                                     |
| Heatmap value H(Q)             | Final maximum                                                                                 | Best post-board energy among actual reachable arrivals.                                                                     |
| Witness inputs                 | Stored output                                                                                 | Allows route solver to recover exact tick inputs after selecting Q.                                                         |

# 11. L/R direction changes: per-tick switching versus a minimum dwell

Two constraints must not be conflated.

## 11.1 Maximum number of reversals

Example: at most six L/R direction changes per one-second segment. For N
ticks, a path can be represented by a starting side plus reversal
boundaries t1...tr with r \<= 6. Once the reversal boundaries are known,
the L/R sign sequence is fixed.

| 0 \< t1 \< t2 \< ... \< tr \< N, r \<= 6 |
|------------------------------------------|

At 66 ticks, raw reversal-timing combinations are small enough for the
existing solver throughput to enumerate directly.

## 11.2 Minimum dwell between reversals

A rule such as “after reversing, remain in that direction for at least
six ticks” is different. It is a lower bound, not a fixed six-tick
cadence:

| t\_(i+1) - t_i \>= 6 |
|----------------------|

A valid schedule might use segment lengths 13, 8, 21, 6, 17, etc. There
is no requirement to reverse exactly every six ticks.

If the aerial search is performed tick-by-tick as a Markov process, this
dwell restriction adds a small control-history state such as
ticks-since-last-reversal. If reversal schedules are enumerated
directly, it is simply an inequality constraint on switch locations. It
reduces the number of legal schedules even though it makes the formal
per-tick state slightly less memoryless.

## 11.3 Why unrestricted per-tick switching is useful as an upper bound

Solving the per-tick-switching version first gives the physical/control
upper envelope A_unrestricted. The dwell-constrained frontier A_D must
be a subset:

| A_D(Q,T) ⊆ A_unrestricted(Q,T) |
|--------------------------------|

This is useful diagnostically: it quantifies exactly how much the
human-like or imposed reversal-rate constraint costs and may reveal
structure before the dwell restriction is applied.

# 12. What the entrance heatmap stores

The visual heatmap should be understood as a compact view over a richer
per-point record.

| **Field**         | **Visualization / storage** | **Meaning**                                                                          |
|-------------------|-----------------------------|--------------------------------------------------------------------------------------|
| H(Q)              | Primary color/value         | Maximum post-board energy actually achievable from S0 at Q.                          |
| theta\*(Q)        | Arrow direction             | Winning terminal horizontal velocity heading.                                        |
| s\*(Q)            | Arrow magnitude / metadata  | Winning horizontal impact speed.                                                     |
| vz(Q)             | Metadata                    | Gravity-determined vertical impact velocity for the winning branch.                  |
| T\*(Q)            | Branch/timing metadata      | Winning impact tick/time branch.                                                     |
| U\*(Q)            | Witness                     | Exact locally optimal input sequence.                                                |
| FastestTangent(Q) | Secondary overlay/metadata  | Best reachable tangent state at Q, with its own speed, heading, timing, and witness. |
| tangent_gap(Q)    | Diagnostic                  | Difference between BestBoard and FastestTangent values; zero if tangent result wins. |

![Figure 3](surf_optimal_boarding_solver_handoff_assets/figure_3.png)

*Figure 3. Schematic entrance heatmap. The scalar field is the best
post-board value H(Q); the arrow field stores the terminal heading of
the winning actual aerial path. A separate tangent overlay can show
FastestTangent(Q).*

# 13. Worked numerical example: one candidate board point

The following uses continuous gravity and an illustrative aerial-speed
function only to demonstrate how the pieces compose. Production results
must come from the exact Source predictor.

## 13.1 Known starting state

| p0 = (0, 0, 500) |
|------------------|

| v0 = (1400, 0, 100) u/s |
|-------------------------|

| g = 800 u/s^2 |
|---------------|

Candidate point on the destination face:

| Q = (1000, 700, 300) |
|----------------------|

## 13.2 Gravity solves the arrival state vertically

Solve:

| 300 = 500 + 100 T - 400 T^2 |
|-----------------------------|

The positive arrival time is approximately:

| T = 0.843 s |
|-------------|

Vertical impact velocity is then:

| vz(T) = 100 - 800(0.843) ≈ -574.5 u/s |
|---------------------------------------|

At this Q and branch, T and vz are now fixed. The aerial solver cannot
choose a different vz without choosing a different collision
time/vertical branch.

## 13.3 Ramp tangent requirement

Use a 45-degree illustrative plane normal:

| n = (1/sqrt(2), 0, 1/sqrt(2)) |
|-------------------------------|

For exact tangency:

| (vx + vz) / sqrt(2) = 0 |
|-------------------------|

Since vz ≈ -574.5:

| vx = 574.5 u/s |
|----------------|

Every horizontal terminal velocity of the form (574.5, vy) is tangent.
Therefore there is a family of tangent speeds/angles rather than one
angle.

## 13.4 Aerial solver returns speed as a function of requested terminal angle

Suppose the exact aerial boundary-value solver were to return the
following illustrative local values for reaching the same Q at the same
T:

| **theta** | **s_A\*(Q,T,theta) u/s** |
|-----------|--------------------------|
| 64°       | 1446                     |
| 66°       | 1476                     |
| 67°       | 1486.5                   |
| 67.3°     | 1489.1                   |
| 68°       | 1494                     |
| 69°       | 1498.5                   |
| 70°       | 1500                     |
| 71°       | 1498.5                   |

For each row, the returned speed is already the maximum obtainable while
satisfying the endpoint and ending-angle constraints. The corresponding
witness path may use perfect-gain strafing where possible and the
minimum necessary sacrifice where it is not.

## 13.5 Self-consistent tangent root

Tangency requires:

| s_A\*(theta) cos(theta) = 574.5 |
|---------------------------------|

At about 67.3 degrees, the illustrative aerial solver gives 1489.1 u/s,
and:

| 1489.1 cos(67.3°) ≈ 574.6 u/s |
|-------------------------------|

So the path-produced speed and the tangent-required angle agree. This is
a self-consistent tangent board. Its total pre-impact speed is
approximately:

| sqrt(1489.1^2 + 574.5^2) ≈ 1596.0 u/s |
|---------------------------------------|

The key point is that neither the 1489.1 u/s speed nor the 67.3-degree
terminal angle was assumed first. The angle was queried, the aerial
solver returned the maximum compatible speed, and the tangent equation
selected the self-consistent root.

# 14. Worked fastest-feasible-tangent example

Use a slightly simplified illustrative case to show the boundary
interpretation clearly. Assume the destination/vertical state makes the
tangent condition:

| vx = 600 u/s |
|--------------|

Therefore the tangent-required horizontal speed at heading theta is:

| s_tan(theta) = 600 / cos(theta) |
|---------------------------------|

Assume the aerial boundary-value solver returns an illustrative
maximum-speed envelope:

| s_A\*(theta) = 1600 - 0.8 (theta - 70)^2 \[theta in degrees, local illustrative fit\] |
|---------------------------------------------------------------------------------------|

At 65 degrees:

| s_tan(65°) ≈ 1420 u/s, s_A\*(65°) ≈ 1580 u/s |
|----------------------------------------------|

Tangency is feasible because the path can produce at least the required
tangent speed. The solver could deliberately give up some available
speed and arrive tangent at about 1420 u/s.

At 70 degrees:

| s_tan(70°) ≈ 1754 u/s, s_A\*(70°) = 1600 u/s |
|----------------------------------------------|

Tangency is impossible at that angle because the tangent requirement
asks for more horizontal speed than the aerial path can deliver.

The boundary lies near 68 degrees, where:

| s_A\*(theta\*) ≈ s_tan(theta\*) ≈ 1597 u/s |
|--------------------------------------------|

That intersection is the fastest feasible tangent for this branch. Below
it, tangent boards exist but require leaving some available aerial speed
unused. Beyond it, the tangent requirement is physically unreachable.

# 15. Cases the implementation must handle

## Case A - Maximum-speed aerial path is already tangent

For some Q,T,theta, the maximum-speed aerial solution lies exactly on
the tangent manifold:

| s_A\*(theta) = s_tan(theta) |
|-----------------------------|

This is ideal: no speed is deliberately sacrificed to satisfy tangency,
and the maximum-gain path is also a tangent board. Store it as both a
tangent candidate and a normal BestBoard candidate.

## Case B - Tangency is feasible, but only below the aerial maximum speed at that angle

Suppose:

| s_tan(theta) \< s_A\*(theta) |
|------------------------------|

The aerial solver can reach Q at theta faster than the speed that would
make that same theta tangent. If the set of attainable speeds at theta
is continuous downward, a slower tangent arrival exists. This is a valid
tangent board, but it may or may not maximize absolute post-board energy
because the faster non-tangent state can retain enough of its extra
speed to win.

## Case C - Fastest feasible tangent sits on the capability boundary

Moving along the tangent family, the required tangent speed increases
until it reaches the aerial maximum:

| s_tan(theta\*) = s_A\*(theta\*) |
|---------------------------------|

This outer/highest-energy intersection is FastestTangent(Q,T). It is the
exact limit at which the maximum-speed aerial path can still be tangent.

## Case D - No tangent board exists at the candidate point/branch

It is possible for the tangent manifold to have no intersection with the
speeds/angles actually reachable while satisfying Q and T. Then:

| A(S0,Q,T) ∩ B(T,n) = empty |
|----------------------------|

FastestTangent(Q,T) is NONE, but the primary BestBoard(Q,T) still exists
if Q is otherwise reachable. Evaluate all relevant reachable terminal
states through the exact collision response and choose the best lossy
board.

## Case E - A lossy faster arrival beats a slower tangent arrival

This is why tangent dominance remains an open theorem. For a fixed
terminal angle, a faster non-tangent arrival has additional energy
before impact. The collision removes only the normal component. In a
simple projection model, the extra speed can exceed the collision loss,
so absolute post-board energy can be higher than that of a slower
tangent state.

Therefore “tangent = zero collision loss” does not by itself prove
“tangent = maximum absolute energy after the board.” The relevant
comparison is specifically the fastest feasible tangent versus the best
reachable lossy state across all angles.

# 16. Recommended per-point algorithm

For one destination face and one known starting state S0:

7.  Canonicalize the start: translate p0 to the origin and rotate the
    horizontal frame so v_h0 points +X.

8.  Select or sample a candidate face point Q. Alternatively enumerate
    impact time T and one face-line coordinate lambda to generate
    Q(T,lambda).

9.  Use the exact vertical predictor to determine every valid arrival
    branch/tick T for Qz and derive vz(T).

10. For each branch, expose the aerial boundary-value function
    s_A\*(S0,Q,T,theta). Internally solve legal L/R control histories
    and retain the witness path for each useful terminal state.

11. Evaluate the exact board response J_Q(T,theta) for those reachable
    terminal states.

12. Choose BestBoard(Q,T) = argmax_theta J_Q(T,theta).

13. Construct the tangent constraint B(T,n), or equivalently
    s_tan(T,theta).

14. Find every feasible tangent intersection/root and choose
    FastestTangent(Q,T) as the highest-energy one.

15. Across vertical branches, choose the best primary board and the best
    tangent result.

16. Store H(Q), winning terminal state, winning witness path,
    FastestTangent(Q), and the difference between them.

17. Repeat over the face. Prune or adaptively refine spatial regions
    using the value field and smoothness/branch structure rather than
    uniformly oversampling forever.

## 16.1 Pseudocode

```text
canonicalize(S0, face)

for Q in candidate_face_points(face):
    primary_best = NONE
    tangent_best = NONE

    for T in vertical_arrival_branches(S0.z, S0.vz, Q.z):
        vz = exact_vertical_velocity(S0.vz, T)

        # Aerial boundary-value solve
        # Returns useful terminal states / envelope, each with a witness path.
        A = solve_aerial_terminal_frontier(
            S0, Q, T, reversal_constraints)

        for terminal_state in A:
            E_after = exact_board_response(
                terminal_state.v_h, vz, face)
            primary_best = max_value(
                primary_best, E_after, terminal_state)

        # Tangent result
        B = tangent_terminal_manifold(
            face.normal, vz, collision_requirement_c)
        tangent_candidates = intersect(A, B)
        if tangent_candidates:
            branch_tangent = argmax_impact_energy(
                tangent_candidates, vz)
            tangent_best = max_value(
                tangent_best, branch_tangent)

    H[Q] = primary_best.post_board_energy
    BestBoard[Q] = primary_best
    FastestTangent[Q] = tangent_best
    TangentGap[Q] = (
        primary_best.value - tangent_best.value
        if tangent_best else INF)

return H, BestBoard, FastestTangent, witnesses
```

# 17. Efficiency strategy

The objective is not merely to make one heatmap correct; it is to make
repeated exact route solving cheap enough that the global problem
becomes equation/function composition plus light brute force.

## 17.1 Do not search variables that are deterministic outputs

- Absolute world position: remove by translation.

- Absolute world yaw: remove by horizontal rotation.

- Vertical position/velocity: derive from exact gravity and arrival
  branch.

- Terminal speed: make it the objective output of the aerial
  boundary-value solve at fixed Q,T,theta.

- Board angle: do not preselect it; find the terminal angle that
  maximizes the composed board value, or solve the tangent root when
  computing FastestTangent.

- Raw input history at the global level: store it as a witness returned
  by the local transfer function.

## 17.2 Prefer frontiers over filled state volumes

For fixed S0,Q,T, many aerial trajectories are dominated. The useful
object is the nondominated terminal frontier: states for which no other
legal arrival is at least as suitable while carrying more relevant
energy. If this frontier empirically collapses to a smooth 1D curve or a
few discrete sheets, the board solve becomes a curve/line intersection
plus a scalar maximization rather than a high-dimensional search.

## 17.3 Cache local transfer functions

Straight planar faces share the same local board-response equation for a
given normal and vertical state. World translation does not matter, and
horizontal rotation can be canonicalized. This supports aggressive
caching or fitting of reusable local transfer functions. Ramp traversal
and dismounts can be treated similarly: entry boundary state -\>
nondominated exit frontier.

# 18. How this feeds the full route solver

The long-term architecture is to replace a global O(number of ticks)
search with composition of locally solved boundary-value operators.

| Air(S_exit, Q_next_entry) -\> Board(Q_next_entry) -\> Ramp(Q_entry, Q_exit) -\> Air(...) |
|------------------------------------------------------------------------------------------|

At the global level, a route can be represented largely by meaningful
face-boundary points and a small branch identifier where necessary:

| Q1_entry -\> Q1_exit -\> Q2_entry -\> Q2_exit -\> ... |
|-------------------------------------------------------|

Each local operator returns the optimal transition value and an exact
witness trajectory. The global optimizer therefore chooses boundary
points and composes value functions instead of directly choosing
thousands of per-tick yaw values.

# 19. Validation / proof obligations before relying on further reductions

| **Test**                                  | **Method**                                                                                                              | **Reason**                                                                      |
|-------------------------------------------|-------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------|
| Exact per-point heatmap correctness       | For sampled Q, compare BestBoard(Q) against exhaustive/very dense brute-force local searches using the exact predictor. | Must pass. This is the core design guarantee.                                   |
| Reachable-speed continuity at fixed theta | Test whether speeds below s_A\*(theta) can generally be attained continuously while preserving Q,T,theta.               | Determines whether s_tan \<= s_A\* is sufficient for tangent feasibility.       |
| Fastest-tangent dominance                 | Search aggressively for any Q where a lossy state has greater post-board energy than FastestTangent(Q).                 | If no counterexample plus proof, primary heatmap may collapse to tangent solve. |
| Aerial frontier dimensionality            | Measure whether nondominated terminal states form one smooth curve, several sheets, or a larger region.                 | Determines the most efficient representation.                                   |
| Dwell penalty                             | Compare unrestricted and minimum-dwell frontiers over representative states.                                            | Quantifies cost of reversal restrictions.                                       |
| Single-plane vs edge/crease contacts      | Separate and validate multi-plane cases.                                                                                | Avoid applying the analytic plane model where Source uses multi-plane clipping. |
| Heatmap spatial smoothness                | Measure gradients/discontinuities across Q.                                                                             | Enables adaptive sampling instead of uniform dense grids.                       |

# 20. Compact mathematical specification

Given known start S0 and candidate face point Q:

| T ∈ ArrivalBranches(S0.z, S0.vz, Q.z) |
|---------------------------------------|

| vz = VzExact(S0.vz, T) |
|------------------------|

| s_A\*(Q,T,theta) = max_U \|\|v_h(T)\|\| subject to p(T)=Q, arg(v_h(T))=theta, legal dynamics |
|----------------------------------------------------------------------------------------------|

| v^-(Q,T,theta) = (s_A\* cos(theta), s_A\* sin(theta), vz) |
|-----------------------------------------------------------|

| J_Q(T,theta) = ExactPostBoardEnergy(v^-, face) |
|------------------------------------------------|

| H(Q \| S0) = max\_{T,theta} J_Q(T,theta) |
|------------------------------------------|

| (T\*,theta\*) = argmax\_{T,theta} J_Q(T,theta) |
|------------------------------------------------|

Fastest tangent:

| B(T,n) = {v_h : nx vx + ny vy = c - nz vz(T)} |
|-----------------------------------------------|

| FastestTangent(Q) = max-energy member across T of \[ A(S0,Q,T) ∩ B(T,n) \] |
|----------------------------------------------------------------------------|

Equivalent angle-root form where applicable:

| s_tan(T,theta) = \[c - nz vz(T)\] / \[nx cos(theta) + ny sin(theta)\] |
|-----------------------------------------------------------------------|

| find outer/highest-energy theta such that s_A\*(Q,T,theta) = s_tan(T,theta) |
|-----------------------------------------------------------------------------|

| **Final architecture rule —** BestBoard(Q) is the authoritative heatmap value. FastestTangent(Q) is an additional exact constrained result. If later proven that FastestTangent(Q) always equals or dominates BestBoard(Q) under the exact Source physics and admissible aerial controls, the general board optimization can be safely collapsed to the tangent-boundary solve. |
|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|

# 21. Deliverable definition for the coding agent

A successful implementation should expose an API conceptually equivalent
to:

- buildEntranceField(start_state, target_face, control_constraints) -\>
  per-Q BestBoard and FastestTangent records

- solveAerialBoundary(start_state, Q, T, terminal_heading, constraints)
  -\> maximum terminal speed + witness inputs

- evaluateBoard(preimpact_state, face) -\> exact post-board state/energy

- solveFastestTangent(start_state, Q, T, face, constraints) -\>
  highest-energy reachable tangent state + witness

- queryEntranceField(Q) -\> exact best reachable board at Q from the
  field's start state

- eventually: solveRampTransfer(entry_boundary_state, target_exit_set)
  -\> nondominated exit frontier + witnesses

The implementation should treat the exact movement/collision predictor
as authoritative and use the analytic reductions only to eliminate
unnecessary state and organize the search. The desired result is an
efficiently searchable space in which every heatmap point represents the
true local optimum from the specified starting state, not a heuristic
approximation of board quality.
