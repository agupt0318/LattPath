#!/usr/bin/env bash

set -euo pipefail

repo_root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$repo_root"

python_bin="${LATTPATH_PYTHON_BIN:-python3}"

city_config="${LATTPATH_CITY_CONFIG:-configs/cities/manhattan.json}"
city_pbf="${LATTPATH_CITY_PBF:-}"
city_backend="${LATTPATH_CITY_BACKEND:-custom}"
output_tag="${LATTPATH_OUTPUT_TAG:-$city_backend}"
pythonpath_value="${LATTPATH_PYTHONPATH:-}"

if [[ "$city_backend" == "custom" && -n "${LATTPATH_CUSTOM_PYTHONPATH:-}" ]]; then
  pythonpath_value="${LATTPATH_CUSTOM_PYTHONPATH}"
fi

if [[ "$city_backend" == "osmnx" && -n "${LATTPATH_OSMNX_PYTHONPATH:-}" ]]; then
  pythonpath_value="${LATTPATH_OSMNX_PYTHONPATH}"
fi

if [[ -n "$pythonpath_value" ]]; then
  export PYTHONPATH="${pythonpath_value}${PYTHONPATH:+:$PYTHONPATH}"
fi

route_scenario="$("$python_bin" -c 'import json,sys; base=json.load(open(sys.argv[1], encoding="utf-8"))["route"]["scenario_name"]; tag=sys.argv[2]; print(f"{base}_{tag}" if tag else base)' "$city_config" "$output_tag")"
district_scenario="$("$python_bin" -c 'import json,sys; base=json.load(open(sys.argv[1], encoding="utf-8"))["district"]["scenario_name"]; tag=sys.argv[2]; print(f"{base}_{tag}" if tag else base)' "$city_config" "$output_tag")"
city_id="$("$python_bin" -c 'import json,sys; base=json.load(open(sys.argv[1], encoding="utf-8"))["city_id"]; tag=sys.argv[2]; print(f"{base}_{tag}" if tag else base)' "$city_config" "$output_tag")"

build_args=(
  --city-config "$city_config"
  --backend "$city_backend"
  --output-tag "$output_tag"
  --data-dir data
  --artifacts-dir artifacts
)

if [[ -n "$city_pbf" && "$city_backend" == "custom" ]]; then
  build_args+=(--pbf-file "$city_pbf")
fi

"$python_bin" tools/build_city_osm.py "${build_args[@]}"

./build/lattpath \
  --scenario-file "artifacts/${route_scenario}_grid.txt" \
  --algorithm lattpath \
  --output "artifacts/${route_scenario}_lattpath_plan.json"

./build/lattpath \
  --scenario-file "artifacts/${route_scenario}_grid.txt" \
  --algorithm astar \
  --output "artifacts/${route_scenario}_astar_plan.json"

"$python_bin" tools/visualize_plan.py \
  "artifacts/${route_scenario}_lattpath_plan.json" \
  --still-output "assets/${route_scenario}_lattpath.svg" \
  --video-output "assets/${route_scenario}_lattpath.gif"

"$python_bin" tools/visualize_plan.py \
  "artifacts/${route_scenario}_lattpath_plan.json" \
  --video-output "assets/${route_scenario}_lattpath.mp4"

"$python_bin" tools/simulate_city_agents.py \
  --scenario-file "artifacts/${district_scenario}_grid.txt" \
  --agents-file "artifacts/${district_scenario}_agents.json" \
  --network-file "artifacts/${district_scenario}_network.json" \
  --output-prefix "$city_id" \
  --output-dir artifacts

"$python_bin" tools/render_city_race.py \
  "artifacts/${city_id}_independent_astar_simulation.json" \
  "artifacts/${city_id}_cooperative_lattpath_simulation.json" \
  --video-output "assets/${city_id}_coordination_race.gif"

"$python_bin" tools/render_city_race.py \
  "artifacts/${city_id}_independent_astar_simulation.json" \
  "artifacts/${city_id}_cooperative_lattpath_simulation.json" \
  --video-output "assets/${city_id}_coordination_race.mp4"
