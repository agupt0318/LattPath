# LattPath

Portable state-lattice path planning demo with a built-in visualizer and generated walkthrough media.

![LattPath demo](assets/lattpath_demo.gif)

[MP4 version](assets/lattpath_demo.mp4) · [Static SVG](assets/lattpath_demo.svg)

## What this repo is now

`LattPath` now ships as a small, runnable path-planning project instead of a non-portable Visual Studio snapshot:

- C++17 state-lattice planner over `(x, y, heading)` states
- Heading-aware motion primitives: `forward`, `long_forward`, `left_arc`, and `right_arc`
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

## Repo layout

- `src/` and `include/`: portable planner implementation and CLI
- `visualizer/`: standalone HTML visualizer with a bundled sample plan
- `tools/visualize_plan.py`: SVG and video renderer
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
