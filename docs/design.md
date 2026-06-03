# Design Notes

This document describes the supported planner in `src/` and `include/`. It is not a description of the older `LatticeDstarPathplanning/` prototype.

## Supported Architecture

The maintained code path is intentionally small:

1. `src/main.cpp` parses CLI arguments and chooses a scenario or benchmark mode.
2. `plan(...)` in `src/planner.cpp` dispatches to one of three search configurations:
   - `lattpath`: heading-aware lattice search with five primitives
   - `astar`: the same pose state representation with a smaller primitive set and a heuristic
   - `dijkstra`: the same pose state representation and primitive set as `astar`, but with no heuristic
3. The planner emits a `PlanResult` or `BenchmarkResult`.
4. JSON writers serialize search traces, recovered paths, and summary metrics for the visualizer and benchmark tools.

The core design choice is that this repo searches over a reusable motion graph, not over raw grid adjacency.

## Why `(x, y, heading)` Instead Of Only `(x, y)`

For a car-like agent, position alone is not enough to describe the next feasible motion. Two vehicles at the same grid cell but facing different directions do not have the same legal successors.

Using `(x, y, heading)` solves three problems:

- It makes turning history explicit. A state already knows which way the vehicle is facing.
- It allows the planner to distinguish between “reached the cell” and “reached the cell in a useful orientation.”
- It lets the search graph encode motion primitives that depend on current heading.

If the state were only `(x, y)`, turns would have to be faked as ordinary grid moves, and the planner could not represent orientation-dependent feasibility without carrying extra hidden state somewhere else.

## Motion Primitives

The maintained planner uses a fixed set of reusable moves:

- `forward`
- `long_forward`
- `cruise_forward`
- `left_arc`
- `right_arc`

These are discrete macro-actions, not continuous trajectories. Each primitive expands to a short sequence of traversed grid cells and a final pose.

Why they matter:

- They encode feasible short-horizon vehicle behavior directly into the search graph.
- They reduce the number of decisions needed to traverse open corridors.
- They let the planner trade off path shape and search effort by choosing between short and long moves.

The baselines intentionally use a smaller primitive set. That is useful for comparison, but it means the comparison is about both search strategy and motion vocabulary.

## What Makes LattPath Different From Generic A*

`A*` is a graph-search method. It says how to search a graph efficiently given edge costs and a heuristic.

`LattPath` in this repo is primarily a graph design choice:

- the nodes are heading-aware poses
- the edges are reusable motion primitives
- the `lattpath` configuration includes longer forward primitives than the baselines

So the important distinction is not “LattPath uses search and A* does not.” The important distinction is that the repo’s `lattpath` mode searches a richer state lattice than the stepwise baselines.

## How To Interpret The Benchmark Comparison

The dense benchmark does **not** prove that “A* is bad.”

What it actually compares on the committed suite is:

- `lattpath`: A*-style search over a lattice with `forward`, `long_forward`, `cruise_forward`, `left_arc`, and `right_arc`
- `astar`: A*-style search over the same pose state space but with only `forward`, `left_arc`, and `right_arc`
- `dijkstra`: the same reduced primitive set as `astar`, with the heuristic removed

That means:

- `astar` vs `dijkstra` mostly isolates the effect of the heuristic
- `lattpath` vs `astar` mixes two effects: the heuristic is present in both, but the motion model is different

When `lattpath` expands fewer states, a large part of that improvement comes from richer macro-primitives that cover useful distance in a single expansion. That is a valid result, but it is not a universal statement about all possible A* implementations.

## Cost Model

The current primitive costs are hard-coded:

- `forward`: `1.0`
- `long_forward`: `1.75`
- `cruise_forward`: `3.25`
- `left_arc`: `2.2`
- `right_arc`: `2.2`

This cost model is simple and intentionally hand-tuned:

- longer forward moves are cheaper than chaining the equivalent number of `forward` moves
- turning costs more than a single forward step
- left and right turns are symmetric

These costs are not learned, not vehicle-calibrated, and not tied to a physical unit such as seconds or meters. They are planning costs used to shape the search.

## Heuristic

The heuristic used by `lattpath` and `astar` is:

- Euclidean distance in grid cells
- plus a small heading-mismatch penalty

This is a pragmatic ranking function, not a formal optimality guarantee. Because diagonal and macro moves can cover more Euclidean distance than their nominal step cost suggests, the heuristic should be treated as informative rather than strictly admissible.

In practice, that means the implementation is aimed at fast, plausible search behavior on the bundled scenarios, not at proving globally optimal cost with respect to the primitive cost model.

## Simulation Assumptions

The city and Manhattan demos add another layer of assumptions on top of the single-agent planner:

- road geometry is rasterized onto a grid
- vehicle orientation is quantized to eight headings
- coordination is modeled in discrete cell-time steps
- traffic controls and waits are simplified overlays, not full signal-phase simulation
- road-speed metadata is coarse and used as scenario-level guidance, not as continuous vehicle dynamics

This is enough to compare coordination behavior on a repeatable problem instance, but it is still a stylized simulation.

## Where The Model Breaks Down

The planner and simulator are intentionally lightweight, so there are clear limits:

- Grid discretization loses lane geometry, curvature fidelity, and clearance detail.
- Eight discrete headings are coarse for tight maneuvers.
- Primitive validity only checks raster occupancy, not swept volume, steering limits, or dynamic stability.
- Costs are shaped for search behavior, not calibrated against real vehicle time or energy.
- The heuristic is not guaranteed admissible under the current move-cost choices.
- The multi-agent demo uses discrete reservations and simplified reaction delays, not a full traffic or autonomy stack.

The right way to read the repo is as a compact, inspectable state-lattice planning project with reproducible demos and benchmarks, not as a production driving system.
