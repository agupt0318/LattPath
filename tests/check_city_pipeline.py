#!/usr/bin/env python3

import json
import subprocess
import sys
import tempfile
from pathlib import Path


def assert_true(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def main() -> None:
    repo_root = Path(__file__).resolve().parents[1]
    network_path = repo_root / "artifacts" / "manhattan_osm_custom_network.json"
    plan_path = repo_root / "artifacts" / "manhattan_osm_custom_lattpath_plan.json"

    network = load_json(network_path)
    plan = load_json(plan_path)

    assert_true(network["schema_version"] >= 4, "city network should expose dual-grid schema metadata")
    assert_true("display_grid" in network, "city network should include display grid metadata")
    assert_true(network["display_grid"]["width"] > network["width"], "display grid should be denser than the planning grid")
    assert_true(network["display_grid"]["height"] > network["height"], "display grid should be taller than the planning grid")
    assert_true(len(network.get("polylines", [])) > 1000, "city network should retain projected centerline polylines")

    assert_true(plan["stats"]["success"], "committed Manhattan custom route should remain solvable")
    assert_true(plan["grid"]["width"] == network["width"], "plan and planning network width should match")
    assert_true(plan["grid"]["height"] == network["height"], "plan and planning network height should match")

    with tempfile.TemporaryDirectory(prefix="lattpath_city_render_") as tmp_dir:
        output_path = Path(tmp_dir) / "render.svg"
        subprocess.run(
            [
                sys.executable,
                str(repo_root / "tools" / "visualize_plan.py"),
                str(plan_path),
                "--network",
                str(network_path),
                "--still-output",
                str(output_path),
            ],
            check=True,
            cwd=repo_root,
        )
        svg = output_path.read_text(encoding="utf-8")
        assert_true("<polyline" in svg, "rendered SVG should contain road/path polylines")
        assert_true("LattPath demo: manhattan_osm_custom" in svg, "rendered SVG should contain the scenario label")


if __name__ == "__main__":
    main()
