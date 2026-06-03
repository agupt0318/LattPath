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


def assert_no_cell_conflicts(simulation: dict) -> None:
    for tick, frame in enumerate(simulation["history"]):
        occupied = set()
        for entry in frame:
            cell = (entry["x"], entry["y"])
            assert_true(cell not in occupied, f"cell conflict at tick {tick}: {cell}")
            occupied.add(cell)


def assert_no_edge_swaps(simulation: dict) -> None:
    for previous, current in zip(simulation["history"], simulation["history"][1:]):
        previous_by_id = {entry["id"]: (entry["x"], entry["y"]) for entry in previous}
        current_by_id = {entry["id"]: (entry["x"], entry["y"]) for entry in current}
        ids = sorted(set(previous_by_id) & set(current_by_id))
        for index, left_id in enumerate(ids):
            for right_id in ids[index + 1 :]:
                assert_true(
                    not (
                        previous_by_id[left_id] == current_by_id[right_id]
                        and previous_by_id[right_id] == current_by_id[left_id]
                    ),
                    f"edge swap between {left_id} and {right_id}",
                )


def main() -> None:
    repo_root = Path(__file__).resolve().parents[1]
    generator = repo_root / "tools" / "generate_multi_agent_demos.py"

    with tempfile.TemporaryDirectory(prefix="lattpath_multi_agent_") as tmp_dir:
        output_dir = Path(tmp_dir)
        subprocess.run([sys.executable, str(generator), "--output-dir", str(output_dir)], check=True, cwd=repo_root)

        for demo_name in ("intersection_swap", "long_crossing"):
            independent = load_json(output_dir / f"{demo_name}_independent_astar_simulation.json")
            cooperative = load_json(output_dir / f"{demo_name}_cooperative_lattpath_simulation.json")

            assert_true(independent["mode"] == "independent_astar", f"{demo_name} independent mode mismatch")
            assert_true(cooperative["mode"] == "cooperative_lattpath", f"{demo_name} cooperative mode mismatch")

            assert_true(independent["lattice"]["state_representation"] == "(x, y, heading)", f"{demo_name} independent lattice metadata missing")
            assert_true(cooperative["lattice"]["state_representation"] == "(x, y, heading, t)", f"{demo_name} cooperative lattice metadata missing")
            assert_true(cooperative["lattice"]["wait_primitive_enabled"], f"{demo_name} should advertise wait primitive support")
            assert_true("wait" in cooperative["lattice"]["primitive_set"], f"{demo_name} should list wait in primitive set")
            assert_true(cooperative["lattice"]["reservation_model"] == "cell_and_edge", f"{demo_name} reservation model mismatch")

            assert_true(len(cooperative["agents"]) == 2, f"{demo_name} should keep both agents")
            assert_true(all("plan" in agent for agent in cooperative["agents"]), f"{demo_name} cooperative plan metadata missing")
            assert_true(all("path" in agent["plan"] for agent in cooperative["agents"]), f"{demo_name} cooperative path metadata missing")
            assert_true(all(agent["plan"]["expanded_states"] > 0 for agent in cooperative["agents"]), f"{demo_name} cooperative expanded states missing")
            assert_true(any(agent["plan"]["uses_wait_primitive"] for agent in cooperative["agents"]), f"{demo_name} should force at least one temporal wait")

            assert_no_cell_conflicts(cooperative)
            assert_no_edge_swaps(cooperative)

            assert_true(cooperative["conflicts"] == 0, f"{demo_name} cooperative conflicts should stay at zero")
            assert_true(cooperative["completed_agents"] == len(cooperative["agents"]), f"{demo_name} cooperative run should finish")
            assert_true(independent["conflicts"] > 0, f"{demo_name} independent run should expose at least one conflict")
            assert_true(
                independent["completed_agents"] < len(independent["agents"]),
                f"{demo_name} independent run should remain visibly unresolved",
            )


if __name__ == "__main__":
    main()
