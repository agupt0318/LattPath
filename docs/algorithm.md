# Algorithm

This document describes the current implementation in `src/planner.cpp`.

## State Representation

Each search state is a pose:

```text
(x, y, heading)
```

where:

- `x` and `y` are integer grid coordinates
- `heading` is one of eight discrete orientations

The heading index maps to these direction vectors:

```text
0: ( 1,  0)
1: ( 1,  1)
2: ( 0,  1)
3: (-1,  1)
4: (-1,  0)
5: (-1, -1)
6: ( 0, -1)
7: ( 1, -1)
```

Headings are normalized modulo `8` before they are used as state indices.

For a grid of width `W`, height `H`, and `Theta = 8` headings, the state space size is:

```text
V = W * H * Theta
```

## Search Variants

All three modes use the same overall search loop and the same pose state indexing:

- `lattpath`: five primitives plus heuristic
- `astar`: three primitives plus heuristic
- `dijkstra`: the same three primitives as `astar`, but heuristic `= 0`

So the main differences are primitive vocabulary and whether a heuristic contributes to priority.

## Neighbor Expansion

The planner expands a state by attempting each primitive in the configured primitive set.

### LattPath Primitive Set

- `forward`
- `long_forward`
- `cruise_forward`
- `left_arc`
- `right_arc`

### Baseline Primitive Set

- `forward`
- `left_arc`
- `right_arc`

Each primitive deterministically produces:

- a final pose
- a list of traversed grid cells
- an additive edge cost

The current primitive semantics are:

- `forward`: advance one cell along the current heading
- `long_forward`: advance two cells along the current heading
- `cruise_forward`: advance four cells along the current heading
- `left_arc`: advance one cell, rotate heading left by one discrete step, then advance one more cell
- `right_arc`: advance one cell, rotate heading right by one discrete step, then advance one more cell

## Primitive Validity Checks

A primitive is considered valid only if every traversed cell:

- stays within grid bounds
- is not occupied by an obstacle

Validation is performed incrementally. The implementation does not “jump” directly to the final pose and then check the endpoint. It checks each intermediate cell in order.

That matters for the longer primitives:

- `long_forward` fails if either of its two cells is blocked
- `cruise_forward` fails if any of its four cells is blocked
- arc primitives fail if either of their two constituent moves is blocked

## Search Data Structures

The search loop maintains:

- `g_scores`: best known cost-to-come for every indexed pose state
- `parents`: predecessor state index for path reconstruction
- `parent_primitive_ids`: which primitive reached that state
- `open_set`: a priority queue ordered by `f = g + h`

The planner uses lazy duplicate handling. If a state is popped from the priority queue with a `g` value worse than the current best recorded `g_score`, that queue entry is skipped.

## Goal Condition

A state is accepted as a goal when:

- `x` matches the goal `x`
- `y` matches the goal `y`
- heading distance from the goal is at most one discrete heading step

So the implementation allows a small orientation tolerance at the goal rather than requiring exact heading equality.

## Path Reconstruction

Once a goal state is popped, the planner reconstructs the state chain by following `parents` backward from the goal index to the start index, then reversing that sequence.

The resulting `PathResult` contains:

- `states`: the pose chain
- `primitives`: the primitive name used on each transition
- `cells`: the full traversed cell trace, reconstructed from the primitive sequence
- `cost`: the `g_score` of the goal state

`states.size()` is therefore expected to equal:

```text
primitives.size() + 1
```

`cells.size()` is usually larger because a single primitive can traverse multiple cells.

## Heuristic

The heuristic used by `lattpath` and `astar` is:

```text
h(current, goal) =
    EuclideanDistance((x, y), (goal_x, goal_y))
    + 0.35 * heading_distance(current_heading, goal_heading)
```

`dijkstra` sets `h = 0`.

### Is It Admissible?

Not in the strict textbook sense for the current cost model.

Reasons:

- diagonal motion can cover `sqrt(2)` Euclidean distance for a `forward` cost of `1.0`
- macro-primitives can move several cells for sublinear cost relative to repeated `forward` moves
- the heading term is a hand-tuned penalty, not a proven lower bound on reorientation cost

So the heuristic is better described as an informed ranking function than as a guaranteed admissible lower bound. On the bundled scenarios it works well as a speed-up, but it is not a proof of optimality.

## Complexity

Let:

- `W` = grid width
- `H` = grid height
- `Theta` = heading count (`8` here)
- `P` = primitive count (`5` for `lattpath`, `3` for the baselines)
- `S` = maximum per-primitive traversal length in cells (`4` here)

Then:

- number of states: `V = W * H * Theta`
- number of candidate edges: `E = O(V * P)`
- primitive validation work per attempted edge: `O(S)`

Using a binary-heap priority queue, the worst-case search cost is:

```text
O((V + E) log V)
```

With the primitive-validation factor made explicit:

```text
O(V * P * S * log V)
```

for this implementation.

Memory usage is dominated by:

- `g_scores`
- `parents`
- `parent_primitive_ids`
- queued frontier entries

So the core bookkeeping is `O(V)`, with additional queue overhead.

## Expanded States vs Final Path Length

These are different quantities and should not be compared directly.

- `expanded_states` counts how many frontier nodes were actually popped and processed.
- `path.states.size()` counts only the recovered solution chain.
- `path.cells.size()` counts the raster cells traversed by that recovered chain.

Because `lattpath` includes macro-primitives, it can sometimes:

- expand fewer states
- produce a short `path.states` sequence
- still traverse many `path.cells`

That is why benchmark summaries in this repo report both search effort and recovered path cost instead of only path length in cells.
