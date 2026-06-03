# Benchmark Methodology

This document describes the dense benchmark that produces [`artifacts/dense_suite_benchmark.json`](../artifacts/dense_suite_benchmark.json).

## Scope

The committed dense suite contains three built-in scenarios:

- `warehouse`
- `switchbacks`
- `dense_city`

Each scenario is run with:

- `lattpath`
- `astar`
- `dijkstra`

The benchmark output records both the final plan for the last run and timing aggregates across repeated runs.

## Reproducing The Dense Benchmark

Build the project first:

```bash
cmake -S . -B build
cmake --build build
```

Then run:

```bash
./build/lattpath --benchmark-dense-suite --benchmark-iterations 250 --benchmark-output artifacts/dense_suite_benchmark.json
```

That is the command that generates the committed benchmark JSON path used by the README.

## What To Record Alongside A Benchmark Run

If you publish or compare numbers, record at least:

- Git commit SHA
- benchmark command line and iteration count
- OS and kernel version
- CPU model and core count
- compiler name and version
- CMake version
- build type and major compile flags

Practical commands on Linux:

```bash
git rev-parse HEAD
uname -a
lscpu
c++ --version
cmake --version
```

If the machine is a laptop or shared development machine, it is also worth noting whether the system was under load. The absolute runtime numbers here are small enough that background activity can matter.

## Reported Metrics

Each algorithm entry records:

- `expanded_states`: number of popped and processed search states
- `path_cost`: cost of the recovered plan under the planner’s primitive cost model
- `runtime_ms`: wall-clock runtime of the final benchmark iteration
- `timing.mean_runtime_ms`
- `timing.min_runtime_ms`
- `timing.max_runtime_ms`
- the recovered path and expansion trace for inspection

`expanded_states` and `path_cost` describe the search result. `mean_runtime_ms` describes repeated execution on one machine. Those are related, but they are not interchangeable.

## Why LattPath Can Expand Fewer States

On this suite, `lattpath` has access to `long_forward` and `cruise_forward`, while `astar` and `dijkstra` do not.

That means one `lattpath` expansion can cover several cells of useful progress that the baselines must realize through multiple shorter expansions. Fewer expanded states here does not mean the implementation discovered a universal theorem about search algorithms. It means the chosen lattice contains richer macro-actions on these scenarios.

## Committed Dense-Suite Numbers

The README reports the aggregate means below, computed from the committed benchmark JSON over the three dense scenarios.

| Algorithm | Mean runtime (ms) | Mean expanded states |
| --- | ---: | ---: |
| LattPath | 0.982 | 118 |
| A* | 1.481 | 376 |
| Dijkstra | 16.567 | 5,133.667 |

Per scenario, the committed artifact contains:

| Scenario | Algorithm | Path cost | Expanded states | Mean runtime (ms) |
| --- | --- | ---: | ---: | ---: |
| warehouse | LattPath | 24.750 | 118 | 0.747 |
| warehouse | A* | 27.000 | 141 | 0.528 |
| warehouse | Dijkstra | 27.000 | 893 | 2.731 |
| switchbacks | LattPath | 31.550 | 112 | 0.659 |
| switchbacks | A* | 36.600 | 182 | 0.649 |
| switchbacks | Dijkstra | 36.600 | 445 | 1.289 |
| dense_city | LattPath | 73.800 | 124 | 1.541 |
| dense_city | A* | 79.800 | 805 | 3.265 |
| dense_city | Dijkstra | 79.800 | 14,063 | 45.681 |

## Valid Conclusions

On the committed benchmark suite, it is reasonable to say:

- `lattpath` expands fewer states than the bundled `astar` baseline on all three dense scenarios
- `lattpath` and the bundled baselines are solving different motion models because the primitive sets differ
- removing the heuristic and using `dijkstra` increases search effort substantially on this suite
- absolute runtime differences are modest on the smaller scenarios and much larger on `dense_city`

## Invalid Conclusions

This benchmark does **not** justify saying:

- “A* is bad”
- “state lattices are always faster”
- “the planner is universally optimal”
- “runtime scales exactly in proportion to expanded state count”
- “these results predict production driving performance”

The benchmark is a controlled comparison on the committed suite, not a universal claim about all maps, heuristics, or vehicle models.

## Notes On Interpretation

Two details are easy to miss:

- `astar` and `dijkstra` use the same primitive set here, so their matching path costs on the committed suite are informative, but not a formal guarantee for all future scenarios because the current heuristic is not strictly admissible.
- `lattpath` can have both lower search effort and lower path cost on this suite because its primitive vocabulary is richer, not because the underlying search loop is fundamentally different from all A* variants.
