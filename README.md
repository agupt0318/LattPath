# LattPath

Portable state-lattice path planning demo with a built-in visualizer, a dense-environment benchmark, and generated walkthrough media.

![LattPath demo](assets/lattpath_demo.gif)

[MP4 version](assets/lattpath_demo.mp4) · [Static SVG](assets/lattpath_demo.svg)

## What this repo is now

`LattPath` now ships as a small, runnable path-planning project instead of a non-portable Visual Studio snapshot:

- C++17 state-lattice planner over `(x, y, heading)` states
- Heading-aware motion primitives: `forward`, `long_forward`, `cruise_forward`, `left_arc`, and `right_arc`
- Dense benchmark suite comparing LattPath against A* and Dijkstra
- Built-in demo scenarios with structured JSON output
- Browser visualizer at [`visualizer/index.html`](visualizer/index.html)
- SVG, GIF, and MP4 rendering pipeline for README-ready media

The original prototype files are still present under `LatticeDstarPathplanning/` as legacy reference material, but the supported entrypoint for the repo is the new planner in `src/`.

## Quick start

Build the planner:

```bash
cmake -S . -B build
cmake --build build
```

List the bundled scenarios:

```bash
./build/lattpath --list-scenarios
```

List the bundled algorithms:

```bash
./build/lattpath --list-algorithms
```

Generate a sample plan:

```bash
./build/lattpath --scenario downtown --output artifacts/downtown_plan.json
```

That writes a JSON file containing:

- the grid dimensions
- the obstacle cells
- every expanded search state
- the recovered path states and traversed cells
- summary stats such as search cost and runtime

## Visualize a plan

Open `visualizer/index.html` in a browser.

The page includes a bundled downtown demo and also lets you load any planner JSON file with the file picker.

You can also regenerate the static and animated media from the command line:

```bash
python3 tools/visualize_plan.py artifacts/downtown_plan.json --still-output assets/lattpath_demo.svg --video-output assets/lattpath_demo.gif
python3 tools/visualize_plan.py artifacts/downtown_plan.json --video-output assets/lattpath_demo.mp4
```

`ffmpeg` is required for video output.

## Dense benchmark

![Dense benchmark](assets/lattpath_dense_benchmark.gif)

[MP4 version](assets/lattpath_dense_benchmark.mp4) · [Benchmark JSON](artifacts/dense_suite_benchmark.json)

The benchmark video above uses the bundled dense suite:

- `warehouse`
- `switchbacks`
- `dense_city`

Averaged over `250` runs per scenario from the committed benchmark JSON:

- `LattPath`: `0.96 ms` mean runtime, `118` mean expanded states
- `A*`: `1.46 ms` mean runtime, `376` mean expanded states
- `Dijkstra`: `16.42 ms` mean runtime, `5,134` mean expanded states

On this dense suite, `LattPath` comes out about `1.5x` faster than `A*` and `17.1x` faster than `Dijkstra`.

This comparison uses the same start and goal pairs and the same heading-aware state space. The difference is that `LattPath` can use longer macro motion primitives like `long_forward` and `cruise_forward`, while the `A*` and `Dijkstra` baselines are limited to stepwise primitives.

Regenerate the benchmark JSON and benchmark video with:

```bash
./build/lattpath --benchmark-dense-suite --benchmark-iterations 250 --benchmark-output artifacts/dense_suite_benchmark.json
python3 tools/render_benchmark_video.py artifacts/dense_suite_benchmark.json --scenario dense_city --video-output assets/lattpath_dense_benchmark.gif
python3 tools/render_benchmark_video.py artifacts/dense_suite_benchmark.json --scenario dense_city --video-output assets/lattpath_dense_benchmark.mp4
```

## Repo layout

- `src/` and `include/`: portable planner implementation and CLI
- `visualizer/`: standalone HTML visualizer with a bundled sample plan
- `tools/visualize_plan.py`: SVG and video renderer
- `tools/render_benchmark_video.py`: dense-suite comparison video renderer
- `artifacts/`: sample planner outputs committed to the repo
- `assets/`: generated media used by this README
- `LatticeDstarPathplanning/`: legacy prototype snapshot

## Validation

The planner is covered by the CMake smoke tests:

```bash
ctest --test-dir build --output-on-failure
```

The committed sample outputs were generated from:

- `downtown`
- `warehouse`
- `switchbacks`
