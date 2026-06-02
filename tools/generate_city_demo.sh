#!/usr/bin/env bash

set -euo pipefail

repo_root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$repo_root"

if [[ -n "${LATTPATH_PYTHONPATH:-}" ]]; then
  export PYTHONPATH="${LATTPATH_PYTHONPATH}${PYTHONPATH:+:$PYTHONPATH}"
fi

city_config="${LATTPATH_CITY_CONFIG:-configs/cities/manhattan.json}"
city_pbf="${LATTPATH_CITY_PBF:-}"

route_scenario="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8"))["route"]["scenario_name"])' "$city_config")"
district_scenario="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8"))["district"]["scenario_name"])' "$city_config")"
city_id="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8"))["city_id"])' "$city_config")"

build_args=(
  --city-config "$city_config"
  --data-dir data
  --artifacts-dir artifacts
)

if [[ -n "$city_pbf" ]]; then
  build_args+=(--pbf-file "$city_pbf")
fi

python3 tools/build_city_osm.py "${build_args[@]}"

./build/lattpath \
  --scenario-file "artifacts/${route_scenario}_grid.txt" \
  --algorithm lattpath \
  --output "artifacts/${route_scenario}_lattpath_plan.json"

./build/lattpath \
  --scenario-file "artifacts/${route_scenario}_grid.txt" \
  --algorithm astar \
  --output "artifacts/${route_scenario}_astar_plan.json"

python3 tools/visualize_plan.py \
  "artifacts/${route_scenario}_lattpath_plan.json" \
  --still-output "assets/${route_scenario}_lattpath.svg" \
  --video-output "assets/${route_scenario}_lattpath.gif"

python3 tools/visualize_plan.py \
  "artifacts/${route_scenario}_lattpath_plan.json" \
  --video-output "assets/${route_scenario}_lattpath.mp4"

python3 tools/simulate_city_agents.py \
  --scenario-file "artifacts/${district_scenario}_grid.txt" \
  --agents-file "artifacts/${district_scenario}_agents.json" \
  --network-file "artifacts/${district_scenario}_network.json" \
  --output-prefix "$city_id" \
  --output-dir artifacts

python3 tools/render_city_race.py \
  "artifacts/${city_id}_independent_astar_simulation.json" \
  "artifacts/${city_id}_cooperative_lattpath_simulation.json" \
  --video-output "assets/${city_id}_coordination_race.gif"

python3 tools/render_city_race.py \
  "artifacts/${city_id}_independent_astar_simulation.json" \
  "artifacts/${city_id}_cooperative_lattpath_simulation.json" \
  --video-output "assets/${city_id}_coordination_race.mp4"
