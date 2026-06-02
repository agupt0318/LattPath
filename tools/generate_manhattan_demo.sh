#!/usr/bin/env bash

set -euo pipefail

repo_root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$repo_root"

export LATTPATH_CITY_CONFIG="${LATTPATH_CITY_CONFIG:-configs/cities/manhattan.json}"
export LATTPATH_CITY_PBF="${LATTPATH_CITY_PBF:-${LATTPATH_MANHATTAN_PBF:-}}"

tools/generate_city_demo.sh
