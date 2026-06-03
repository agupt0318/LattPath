#!/usr/bin/env python3

import argparse
import subprocess
import sys
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate deterministic multi-agent demo artifacts.")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("artifacts"),
        help="Directory for generated simulation JSON files.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    repo_root = Path(__file__).resolve().parents[1]
    simulator = repo_root / "tools" / "simulate_city_agents.py"
    configs = repo_root / "configs" / "multi_agent"

    demos = [
        ("intersection_swap", configs / "intersection_swap.grid", configs / "intersection_swap_agents.json"),
        ("long_crossing", configs / "long_crossing.grid", configs / "long_crossing_agents.json"),
    ]

    args.output_dir.mkdir(parents=True, exist_ok=True)

    for name, scenario_file, agents_file in demos:
        subprocess.run(
            [
                sys.executable,
                str(simulator),
                "--scenario-file",
                str(scenario_file),
                "--agents-file",
                str(agents_file),
                "--output-dir",
                str(args.output_dir),
                "--output-prefix",
                name,
            ],
            check=True,
            cwd=repo_root,
        )
        print(f"Generated {name} demos in {args.output_dir}")


if __name__ == "__main__":
    main()
