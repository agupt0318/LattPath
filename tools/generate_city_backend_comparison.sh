#!/usr/bin/env bash

set -euo pipefail

repo_root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$repo_root"

city_config="${LATTPATH_CITY_CONFIG:-configs/cities/manhattan.json}"
python_bin="${LATTPATH_PYTHON_BIN:-python3}"

LATTPATH_CITY_CONFIG="$city_config" \
LATTPATH_CITY_BACKEND="custom" \
LATTPATH_OUTPUT_TAG="custom" \
LATTPATH_PYTHON_BIN="$python_bin" \
tools/generate_city_demo.sh

LATTPATH_CITY_CONFIG="$city_config" \
LATTPATH_CITY_BACKEND="osmnx" \
LATTPATH_OUTPUT_TAG="osmnx" \
LATTPATH_PYTHON_BIN="$python_bin" \
tools/generate_city_demo.sh
