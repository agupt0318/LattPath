#!/usr/bin/env bash

set -euo pipefail

repo_root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$repo_root"

python3 tools/build_manhattan_osm.py \
  --data-dir data \
  --artifacts-dir artifacts

./build/lattpath \
  --scenario-file artifacts/manhattan_osm_grid.txt \
  --algorithm lattpath \
  --output artifacts/manhattan_osm_lattpath_plan.json

./build/lattpath \
  --scenario-file artifacts/manhattan_osm_grid.txt \
  --algorithm astar \
  --output artifacts/manhattan_osm_astar_plan.json

python3 tools/visualize_plan.py \
  artifacts/manhattan_osm_lattpath_plan.json \
  --still-output assets/manhattan_osm_lattpath.svg \
  --video-output assets/manhattan_osm_lattpath.gif

python3 tools/visualize_plan.py \
  artifacts/manhattan_osm_lattpath_plan.json \
  --video-output assets/manhattan_osm_lattpath.mp4

python3 tools/simulate_manhattan_agents.py \
  --scenario-file artifacts/manhattan_midtown_osm_grid.txt \
  --agents-file artifacts/manhattan_midtown_agents.json \
  --output-dir artifacts

python3 tools/render_manhattan_race.py \
  artifacts/manhattan_independent_astar_simulation.json \
  artifacts/manhattan_cooperative_lattpath_simulation.json \
  --video-output assets/manhattan_coordination_race.gif

python3 tools/render_manhattan_race.py \
  artifacts/manhattan_independent_astar_simulation.json \
  artifacts/manhattan_cooperative_lattpath_simulation.json \
  --video-output assets/manhattan_coordination_race.mp4
