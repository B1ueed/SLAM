# 2D Particle Filter SLAM: A Research Log

## Overview

This repository documents every iteration of a 2D SLAM simulation written from scratch in Python. The project started from particle filter intuition built in ECE368 and grew into a full autonomous exploration system with frontier-based navigation, A\* path planning, and multi-stage stuck recovery. The document is structured as a research log: every change is recorded alongside what motivated it, what the observed behaviour was, and what it led to next. The goal is to capture every design decision, including the ones that were later reverted.

---

## Background and Motivation

After completing ECE368's rover lab, which involved implementing the forward-backward and Viterbi algorithms for a discrete HMM grid, the natural question was what it would take to generalise those ideas to continuous space with a real sensor model. The connection is direct:

| ECE368 Rover Lab | This Project |
|---|---|
| Discrete grid states | Continuous $(x, y, \theta)$ pose |
| Known transition matrix $A$ | Noisy differential drive odometry |
| Known emission matrix $B$ | Noisy 2D lidar |
| Forward algorithm: $p(s_t \mid o_{1:t})$ | Particle filter: $p(x_t \mid z_{1:t})$ |
| Map is given | Map must be built simultaneously |

The rover lab handed you the map. SLAM removes that assumption: you must localise within a map you are simultaneously building.

The secondary motivation was a robotics co-op application where SLAM was explicitly listed as a development target. Having a working simulation to discuss concretely was the goal.

---

## Iteration Log

---

### v1: Basic Particle Filter with Hardcoded Waypoints

A 10x8 metre rectangular room with three interior wall segments, a robot following a hardcoded waypoint list, and a particle filter estimating pose from noisy lidar scans.

The particle filter loop:

```
predict()   apply odometry + Gaussian noise to all N particles
update()    weight each particle by p(z | x) against the known walls
resample()  systematic resampling
```

The occupancy grid is updated from the best-weight particle's estimated pose each step.

**Key parameters:**

```python
N_PARTICLES      = 200
N_BEAMS          = 24
MOTION_NOISE_XY  = 0.05   # metres std per step
MOTION_NOISE_TH  = 0.02   # radians std per step
SENSOR_NOISE_STD = 0.15   # metres std
L_OCC            = +0.85
L_FREE           = -0.40
GRID_RES         = 0.10   # metres per cell
```

The asymmetry between `L_OCC` and `L_FREE` is intentional. Each beam hit provides stronger evidence of a wall than each free-space pass provides of emptiness, so walls accumulate confidence faster than they erode. This mirrors the asymmetric update in the log-odds Bayes rule.

**What worked:** The particle cloud converged reliably. Localisation error stayed under 0.2 m in open corridor sections. The occupancy grid reproduced wall geometry within roughly 50 timesteps.

**Bug found:** The robot drives through the final wall. The last waypoint is back at the origin, and the proportional controller computes a straight-line heading there regardless of obstacles. This exposed the core architectural gap: SLAM tells you where you are and what the world looks like, but a separate navigation layer is needed to plan a path that avoids walls. These are two distinct problems.

**Also noted:** Particle weights are computed by ray-casting against the known ground-truth wall geometry. This means v1 is really localisation in a known map, not true SLAM. Every particle shares the same map. Fixing this is the entire motivation for v2.

> 📷 *[v1_particle_cloud.png: particle cloud converging around the true robot pose at around step 150]*  
> 📷 *[v1_wall_passthrough.png: robot driving through the final wall at the end of the run]*

---

### v2: Per-Particle Maps (True SLAM)

Each particle now carries its own log-odds occupancy grid, initialised to all zeros (pure unknown). Particle weights are computed by ray-casting against the particle's personal map rather than against known walls. This is the first version that is actually doing SLAM.

**How per-particle weighting works:** A particle whose pose hypothesis is wrong has a map full of unknown cells in the directions the real scan is measuring. Its ray-casts into unknown space return max range. The real scan returns, say, 1.5 metres. The weight is proportional to `exp(-sum((z_actual - z_hat)^2) / 2*sigma^2)`, so the mismatch drives the weight toward zero and the particle dies in resampling.

**The 4-particle diagnostic panel:** Rather than rendering just the best particle, a 2x2 panel shows four particles selected by weight rank: the best, two mid-weight particles, and the worst. Each panel shows that particle's personal map and its expected scan drawn as yellow lines. The border colour and thickness encode status: green and thick for THRIVING, orange for surviving, dark orange for fading, red for DEAD. This panel makes the predict/weight/resample cycle visually interpretable.

**Performance problem:** N=200 independent Python ray-cast loops are too slow for real-time Matplotlib rendering. The GIL prevents threading from helping for CPU-bound Python work.

**Solutions considered:**
- `multiprocessing.Pool` with `spawn` context: each process gets its own interpreter and GIL, so ray-casts run in true parallel. Pickle overhead is the cost. Viable at N=20.
- Vectorised NumPy: reformulate ray-casts as array operations at the C level.
- Numba JIT with `@njit(parallel=True)`: compiles to native threaded code. Fastest after first-call overhead, but adds a dependency.

**Chosen approach:** `multiprocessing.Pool` with spawn context for weight computation. Each particle's map is passed as a flat 1D array to minimise pickle cost. N is reduced to 20 for this version. The occupancy grid update stays single-threaded since the Bresenham loop is pure Python and would need Numba to beat pickle overhead.

**Final localisation error:** approximately 12 cm, higher than v1's 2 cm. The penalty is working with a self-built uncertain map rather than ground truth, especially early in the run when walls are not yet confirmed.

**Still broken:** hardcoded waypoints, no autonomy.

> 📷 *[v2_4particle_panel.png: four particles at different weight ranks showing their personal maps and expected scans]*

---

### v3: Dijkstra Frontier Exploration (First Autonomous Version)

Hardcoded waypoints require knowing the map in advance. A robot doing real SLAM does not have that luxury. The standard solution is frontier-based exploration.

A frontier is a free cell adjacent to at least one unknown cell. Frontiers are the boundary between what the robot has observed and what it has not. Driving toward a frontier is a simple and effective exploration strategy that requires no prior map knowledge.

**Why Dijkstra and not A\*:** When selecting the best frontier, there is no known goal yet. You need distances from the robot to all frontier cells simultaneously. Dijkstra's single-source shortest path tree gives this in one pass. A\* would need a separate run for each frontier candidate. Once the best frontier is chosen, A\* is the right tool because there is now a specific goal; that split is implemented in v4a.

**What was added:**

1. Frontier detection: scan the grid for free cells adjacent to unknown cells, then BFS-cluster adjacent frontier cells into contiguous groups.
2. Dijkstra from the robot's current cell across all non-occupied, non-inflated cells.
3. Frontier scoring: `score = cluster_size / path_distance`. Naive, but correctly prefers large nearby frontiers.
4. Path extraction via the Dijkstra predecessor map.
5. Path following: step along the extracted cell sequence.

**What worked:** The robot genuinely explores without any map knowledge. Exploration terminates when no frontier cells remain, which is the correct completion condition.

**New bugs introduced:** The robot commits to a path and follows it even as new walls appear in the map (wall clipping). The scoring does not account for what is behind the frontier (information gain). There are no triggers to force a replan when the situation changes mid-path.

> 📷 *[v3_frontiers.png: frontier clusters highlighted on the occupancy grid]*  
> 📷 *[v3_dijkstra_tree.png: Dijkstra distance map from the robot's current cell]*

---

### v4a: A\*, Live Centroids, Information Gain, Recency Penalty, Medial Axis

This version introduces the full planning architecture that the later versions are built on. Every component has a specific motivation tied to a bug observed in v3.

**Architecture at this point:**

```
SENSING:   Particle filter (150 particles) + occupancy grid
FRONTIER:  BFS-cluster; centroid recomputed live every step from current grid
SCORING:   Dijkstra distances + info_gain^1.5 / (path_dist+1) * recency * locality
PLANNING:  A* with medial axis edge weights, pruned to sparse waypoints (angle > 0.3 rad)
REPLAN:    centroid drift > MAX_CENTROID_DRIFT, cluster dissolves, waypoint blocked, stuck
```

**Live centroid recomputation:** The v3 approach locked onto a centroid at planning time and drove toward it indefinitely. As the robot approaches a frontier, cells get marked free or occupied and the centroid shifts. The robot was chasing a stale position. The fix is to recompute the centroid from the current grid on every step. A\* only reruns when the centroid has drifted more than `MAX_CENTROID_DRIFT = 5` cells from the tracked position.

**Information gain scoring:**

```python
info_gain = unknown cells within sensor range of frontier centroid
score = info_gain^1.5 / (path_dist + 1)
```

The 1.5 exponent strongly suppresses small frontiers even when nearby. A frontier opening into a large unexplored region beats a tiny frontier that would reveal only a few cells.

**Recency penalty instead of blacklist:** Blacklisting frontiers where A\* failed permanently excluded frontiers that were only temporarily unreachable (for example, not yet mapped enough to plan through). The recency penalty multiplies the score by 0.5 on each failure. The frontier stays in the candidate pool but scores lower, and can win again if everything else is worse.

**Sparse waypoint pruning:** Raw A\* output on a 0.25 m grid produces 40 to 100 cells per path, most of which lie on straight corridor runs between turns. Feeding raw cells to the proportional controller causes micro-stepping instability: the heading correction term fires on tiny positional offsets and the robot oscillates. The fix is to keep only cells where the heading changes by more than `WAYPOINT_ANGLE_TOL = 0.3` radians. Straight runs collapse to just start and end.

**Medial axis edge weights:** A\* edge costs are set to `1 + MEDIAL_WEIGHT / wall_dist^2`, routing paths through corridor centres and away from walls. This required a BFS wall-distance pass at every replan.

**Pocket fallback:** When three or fewer frontiers remain, the medial axis penalty is dropped so the robot can squeeze into tight unexplored corners.

> 📷 *[v4a_info_gain.png: frontier clusters coloured by information gain score]*  
> 📷 *[v4a_sparse_waypoints.png: raw A\* path vs pruned sparse waypoints]*

---

### v4b: Medial Axis Removed

The medial axis edge weights required computing a full BFS wall-distance map at every replan. Testing showed that the sparse waypoint pruning from v4a already kept paths from hugging walls naturally. The added complexity was not justified by the improvement.

Removed: `compute_wall_distance_map`, `MEDIAL_WEIGHT`, the medial axis term in A\* edge costs, and the `use_medial` flag. A\* now uses uniform edge costs. The pocket fallback is also removed since it only existed to disable the medial axis.

---

### v4c: Cluster Certainty, Squared Locality, Cluster Merging, Direction Heuristic

Four scoring improvements added in one pass.

**Cluster certainty:**

```python
def cluster_certainty(cells, grid):
    return min(mean([abs(grid.log_odds[r, c]) for r, c in cells]) / L_MAX, 1.0)
```

Cells near `log_odds = 0` are barely observed boundaries. Cells with high `|log_odds|` are well-confirmed edges. Rewarding certainty prevents the robot from repeatedly targeting the same ambiguous boundary that keeps shifting.

**Squared locality:** The locality term `exp(-lambda * d)` was squared to `exp(-lambda * d)^2`. This makes the exponential decay much steeper and strongly suppresses distant frontiers until nearby ones are consumed.

**Cluster merging:** Frontier clusters whose centroids are within `MERGE_RADIUS = 5` cells of each other are merged into one larger cluster before scoring. This prevents the robot from oscillating between two adjacent small clusters that represent the same unexplored region.

**Directional heuristic:** Frontiers in the robot's current travel direction receive a score multiplier of `1 + DIRECTION_WEIGHT * max(0, cos(angle_between_heading_and_frontier))`. This suppresses the oscillation between two equal-scoring frontiers on opposite sides of a corridor.

**Updated scoring priority:** locality squared and certainty dominate, cluster size is secondary, recency is a tiebreaker.

---

### v5a: Asymmetric Cell Lock Thresholds

The OccupancyGrid class gained an `update_counts` array and asymmetric lock thresholds. Once a cell has accumulated enough observations and its log-odds crosses the threshold, further updates in the opposite direction are ignored.

```python
LOCK_THRESH_OCC       = 3.0   # occupied cells lock at this log-odds value
LOCK_THRESH_FREE      = 5.0   # free cells require more evidence before locking
LOCK_MIN_UPDATES_OCC  = 5     # minimum observations before occupied lock applies
LOCK_MIN_UPDATES_FREE = 12    # minimum observations before free lock applies
```

The asymmetry is deliberate. Occupied cells should lock quickly: a well-confirmed wall should not be eroded by a few stray free-space readings from a drifted pose estimate later in the run. Free cells need more evidence before locking, because a navigable cell that gets accidentally marked occupied is much more dangerous than a wall cell that gets softened slightly.

**Max-range beam fix:** Previously, beams at max range skipped the free-space marking along the ray as well as the hit marking at the endpoint. This was wrong: the robot's path to max range was clear, and those cells should be marked free. The fix runs free-space marking always and skips only the endpoint occupied update when `d >= MAX_RANGE * 0.99`.

---

### v5b: Robot Speed Halved, Waypoint Timeout Fixed

**Speed reduction:** `ROBOT_SPEED` was halved from 0.12 to 0.06 m/step. At 0.12 m/step, a wall section was seen from only 2 to 3 angles before the robot passed it. At 0.06 m/step it gets roughly 6 observations, which is enough to push the log-odds well past the lock threshold and produce a stable map.

**Timeout counting fix:** The stuck timeout counter was incrementing on every step, including pure heading correction steps where `dx == dy == 0`. A robot doing a sharp turn in place could time out at a waypoint it was actively approaching. The fix: `steps_on_waypoint` only increments when `dx != 0 or dy != 0`. `MAX_STEPS_PER_WAYPOINT` was doubled from 60 to 120 to keep the effective timeout distance (speed * max_steps) unchanged.

---

### v5c: LOS Waypoint Pruning, Pre-Move Cell Check, Path Validation

**LOS waypoint pruning:** The angle-tolerance pruning from v4a kept cells where the heading changed by more than 0.3 radians. This had a subtle bug: the proportional controller drives between waypoints in continuous world space using a straight line, but the straight line between two pruned waypoints passes through different grid cells than the original A\* path visited. If any of those intermediate cells were blocked, the controller would clip through a wall that A\* had avoided.

The fix is line-of-sight pruning using Bresenham's algorithm. Starting from the first cell, the pruner greedily advances as far as it can while a Bresenham line from the anchor to the lookahead is entirely clear of the inflation mask. The only cells the controller traverses are cells the Bresenham check approved. The disagreement between planner and validator is eliminated.

`_bresenham` is extracted to module scope so it can be called without class method overhead, since it sits inside the innermost loop over all beams times all particles.

**Pre-move cell check:** Before each movement step, the very next grid cell is checked against the current inflation mask. If it is blocked, a replan is triggered before the robot physically moves. This catches cases where a newly mapped wall has just appeared on the planned path but the path validation has not yet had a chance to run.

**Continuous path validation:** Every step, the remaining waypoints and the Bresenham segments between consecutive waypoints are checked against a freshly rebuilt inflation mask. If any future waypoint or segment is blocked, a replan triggers immediately. This eliminates the wall-clipping behaviour from v3.

---

### v5d: Momentum EMA, Replan Hysteresis

**Momentum EMA:** The directional heuristic from v4c used the robot's instantaneous heading. A single sharp turn would flip the bonus to the wrong frontier, causing oscillation between two equal-scoring options on opposite sides of a corridor. The fix is an exponential moving average of recent displacement:

```python
MOMENTUM_ALPHA  = 0.25   # EMA decay
MOMENTUM_WEIGHT = 0.35   # maximum fractional bonus

momentum_dx = MOMENTUM_ALPHA * nx + (1 - MOMENTUM_ALPHA) * momentum_dx
momentum_dy = MOMENTUM_ALPHA * ny + (1 - MOMENTUM_ALPHA) * momentum_dy
```

The bonus persists for roughly 4 steps after a direction change (the EMA decay), which is long enough to prevent single-step oscillation.

**Replan hysteresis:** Cluster-dissolution replans were firing immediately after a new plan was made, because the centroid was still settling as the occupancy grid updated around the frontier. The fix suppresses centroid-drift replans for the first `MIN_JOURNEY_STEPS = 20` translating steps after any replan. After 20 steps the centroid has stabilised and drift is a reliable signal.

---

### v5e: Observation Spin Mechanic

When the robot arrived at a frontier it immediately computed the next target and drove toward it, often without having adequately scanned the newly revealed area. Adding a 360-degree spin on arrival updates the occupancy grid from all angles before replanning, giving the next frontier selection better information.

**Two triggers:**

Arrival trigger: the robot is within `2 * ARRIVE_DIST` of the final waypoint and more than `SPIN_INFO_GAIN_THRESHOLD = 30` unknown cells are visible.

Density trigger: more than `SPIN_FRONTIER_DENSITY_THRESHOLD = 3` frontier clusters exist within sensor range, meaning the robot is surrounded by unexplored space and should observe before committing to one direction.

**Spin state machine:** When `spinning = True`, each step executes one rotation of `SPIN_STEP_RAD = 0.30` radians, runs the lidar, particle filter, and grid update, then checks exit conditions. Early exit fires if information gain drops below `SPIN_EARLY_EXIT_GAIN = 10` after at least `SPIN_MIN_STEPS_BEFORE_EXIT = 6` steps. A hard cap of `SPIN_MAX_STEPS = 25` prevents runaway spins. `SPIN_COOLDOWN_STEPS = 80` prevents re-spinning at the same location.

---

### v5f: Spin Parameter Tuning

The spin thresholds from v5e turned out to be too aggressive. In the 12x12 maze, the density trigger (`3` nearby clusters) fired at nearly every junction. The robot was spinning before entering rooms and seeing through doorways into the adjacent space, collapsing those frontiers before physically entering. This caused premature room resolution: the robot believed a room was explored before it had actually driven through it.

Parameters raised:

```python
SPIN_INFO_GAIN_THRESHOLD        = 30  ->  60   # arrival spin needs 2x more unknown cells
SPIN_FRONTIER_DENSITY_THRESHOLD = 3   ->  6    # density spin needs more nearby clusters
SPIN_MAX_STEPS                  = 25  ->  15   # shorter arc per spin
SPIN_COOLDOWN_STEPS             = 80  ->  200  # longer quiet period between spins
```

---

### v6a: Frontier Priority Queue, Correct Termination, Recency Zero-Threshold

**Priority queue:** The previous approach scored all frontiers, sorted them, and tried the best one. If A\* failed, the list was rebuilt from scratch. The replacement uses a `heapq` keyed by `-score`. At planning time, candidates are popped until one yields a valid A\* path. Termination fires when the heap is exhausted without producing a valid plan, which is strictly the correct condition: no reachable frontier exists.

```python
pq = []
for cluster in frontier_clusters:
    heapq.heappush(pq, (-score, cr, cc, cluster))

while pq:
    neg_sc, cr, cc, cluster = heapq.heappop(pq)
    path = astar(robot_cell, (cr, cc), inflation_mask)
    if path:
        commit_to_path(path)
        break
else:
    exploration_done = True
```

**Recency zero-threshold:** The recency decay multiplied scores by 0.5 on each failure but never reached zero. A truly unreachable frontier would accumulate decays but remain in the candidate pool indefinitely, eventually winning when everything else was also scored low. The fix skips any frontier whose recency has fallen below `RECENCY_ZERO_THRESHOLD = 0.03`, which corresponds to roughly 5 consecutive A\* failures.

**Randomised nudge directions:** The escape-angle nudge tried candidate directions in a fixed order, so the robot behaved deterministically when stuck, wedging itself into the same configuration every time. Shuffling the candidate list before iterating breaks the symmetry.

---

### v7a: Thick Walls, Wall Stamp, 12x12 Maze

**The collinear ray miss bug:** Zero-thickness walls are line segments. The `ray_segment_intersect` function finds the crossing point of two infinite lines. For parallel lines, the crossing is at infinity and the function returns max range. A ray travelling exactly parallel to a wall would miss it entirely, leaving that wall undetected in the occupancy grid.

**Fix:** Each centreline wall is represented as two parallel ray-cast segments offset by `+WALL_HALF_WIDTH = 0.125` m and `-WALL_HALF_WIDTH` in the perpendicular direction. A ray parallel to the original wall now intersects one of the two offset copies.

```python
def _make_thick_walls(centrelines):
    result = []
    for x1, y1, x2, y2 in centrelines:
        dx, dy = x2-x1, y2-y1
        L = hypot(dx, dy)
        nx, ny = -dy/L, dx/L
        for sign in (+1, -1):
            result.append((x1+sign*nx*WALL_HALF_WIDTH, y1+sign*ny*WALL_HALF_WIDTH,
                           x2+sign*nx*WALL_HALF_WIDTH, y2+sign*ny*WALL_HALF_WIDTH))
    return result
```

**Wall stamp:** Before the robot moves, every cell inside a thick wall is pre-marked at log-odds `+4.0`. This gives the Bayesian updater a strong prior that prevents wall cells from being eroded by early free-space readings from a drifted pose estimate.

**12x12 maze:** The world was expanded from 10x8 m to 12x12 m with four horizontal and three vertical internal walls, multiple passage gaps, and narrow corridors. This exercises the exploration algorithm more aggressively than the original open room.

**Gap clearance analysis:** Wall thickness plus inflation consumed passage width. Each side of a gap loses `WALL_HALF_WIDTH + INFLATION_RADIUS = 0.125 + 0.25 = 0.375` m. Both sides together consume 0.75 m. All passage gaps in the new map are at least 1.5 m centre-to-centre, leaving a usable opening of roughly 0.75 m after inflation, which is three grid cells and sufficient for the robot.

> 📷 *[v7a_thick_wall_comparison.png: occupancy grid before and after the thick wall model showing the collinear miss artefact eliminated]*  
> 📷 *[v7a_maze.png: the 12x12 maze with passage gaps labelled]*

---

### v7b: Gap Widening and Progressive Stuck Recovery

**Gap widening:** Testing in the 12x12 maze showed that several passages that satisfied the 1.5 m minimum on paper were still causing navigation failures because the inflation mask was occasionally one cell wider than expected due to floating-point rounding in the grid coordinate conversion. All gaps were widened to a comfortable 1.5 to 2.0 m in the WALLS definition to eliminate these edge cases.

**BFS escape:** The old stuck recovery used random-angle nudge teleportation. The problems were that the nudge direction was always the current robot heading (not toward any free cell), the nudge was an instantaneous position change that could place the robot inside a wall, and the failure counter declared exploration complete after 8 consecutive nudge failures even when frontiers still existed.

The replacement does a BFS outward from the robot's current grid cell, ignoring the inflation mask, to find the nearest free cell at least `STUCK_BFS_RADIUS = 3` cells away. A\* without inflation then plans a physical path to that cell and the robot drives there. This allows the robot to back out of dead-ends that the inflated mask had made unreachable.

**Wall-follow mode:** When BFS escape itself fails repeatedly, the controller computes a distance-weighted average of nearby occupied cell normals to get a tangent direction, then adds a tangential velocity component. The robot steers along the wall contour until it finds open space. This handles convex corners where the BFS escape cell is blocked from multiple directions.

**Progressive recovery stages:**

1. Replan with normal inflation
2. BFS escape: physical drive to the nearest reachable free cell
3. Wall-follow: tangential steering around the nearest obstacle
4. Replan with zero inflation (pocket mode)
5. Exploration complete: only if the frontier PQ is genuinely empty

> 📷 *[v7b_bfs_escape.png: BFS tree from stuck position showing the escape route]*

---

## Feature Provenance

| Feature | Version |
|---|---|
| Basic particle filter + occupancy grid | v1 |
| Per-particle maps, true SLAM | v2 |
| Frontier detection + BFS clustering | v3 |
| Dijkstra for frontier scoring | v3 |
| A\* for path planning | v4a |
| Live centroid recomputation | v4a |
| Information gain scoring | v4a |
| Recency penalty | v4a |
| Sparse waypoint pruning (angle) | v4a |
| Medial axis edge weights | v4a |
| Medial axis removed | v4b |
| Cluster certainty metric | v4c |
| Squared locality | v4c |
| Directional heuristic | v4c |
| Cluster merging | v4c |
| Asymmetric cell lock thresholds | v5a |
| update_counts array | v5a |
| Max-range beam free-space fix | v5a |
| Robot speed halved | v5b |
| Waypoint timeout counting fixed | v5b |
| LOS waypoint pruning | v5c |
| Bresenham extracted to module scope | v5c |
| Pre-move cell check | v5c |
| Continuous path validation | v5c |
| Momentum EMA | v5d |
| Replan hysteresis | v5d |
| Observation spin mechanic | v5e |
| Spin parameter tuning | v5f |
| Frontier priority queue | v6a |
| PQ-based correct termination | v6a |
| Recency zero-threshold | v6a |
| Randomised nudge directions | v6a |
| Two-segment thick wall model | v7a |
| Wall stamp at startup | v7a |
| 12x12 maze world | v7a |
| Gap width analysis and redesign | v7b |
| BFS escape stuck recovery | v7b |
| Wall-follow mode | v7b |
| Progressive recovery stages | v7b |

---

## Open Problems

**Frontier thrashing near map completion:** When two or three small isolated unknown pockets remain, the robot cycles through frontier candidates rapidly without committing, burning steps at each replan. A straightforward fix would be to relax `MIN_FRONTIER_SIZE` toward the end of exploration, or to add a pocket-scan mode that drives directly to the nearest unknown cell rather than treating it as a frontier cluster to score and plan toward.

**Frontier resolution false negatives:** BFS clusters below `MIN_FRONTIER_SIZE = 2` cells are never targeted. Isolated unknown pockets below this threshold leave small map gaps even after exploration complete is declared.

**Bresenham/A\* cell disagreement (residual):** The LOS pruning in v5c largely resolved this, but a residual case exists. The proportional controller moves in continuous world-space coordinates. The path validator checks discrete grid cells. Submillimetre floating-point offsets can cause the controller's actual trajectory to cross a grid cell that the Bresenham check did not include, resulting in a false clear signal.

---

## Theoretical Connections to ECE368

| This project | ECE368 |
|---|---|
| Particle filter predict step | Markov chain process model $p(s_t \mid s_{t-1}, u_t)$ |
| Particle filter update step | Bayes rule: $w \propto p(z \mid x)$ |
| Systematic resampling | Importance sampling from a weighted empirical distribution |
| Occupancy grid per-cell update | Bernoulli log-odds Bayes update |
| Dijkstra for frontier scoring | Single-source shortest path on a probabilistic free-space graph |
| Information gain scoring | Expected entropy reduction $H(M) - H(M \mid z_{\text{new}})$ |
| Adaptive locality lambda | Posterior-conditioned scoring: fewer remaining hypotheses, flatter prior |

