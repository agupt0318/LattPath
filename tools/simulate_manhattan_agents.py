#!/usr/bin/env python3

import argparse
import heapq
import json
import math
from collections import defaultdict
from pathlib import Path

HEADING_VECTORS = (
    (1, 0),
    (1, 1),
    (0, 1),
    (-1, 1),
    (-1, 0),
    (-1, -1),
    (0, -1),
    (1, -1),
)

LATTPATH_PRIMITIVES = (
    {"name": "forward", "kind": "forward", "cost": 1.0},
    {"name": "long_forward", "kind": "forward", "steps": 2, "cost": 1.75},
    {"name": "cruise_forward", "kind": "forward", "steps": 4, "cost": 3.25},
    {"name": "left_arc", "kind": "left_arc", "cost": 2.2},
    {"name": "right_arc", "kind": "right_arc", "cost": 2.2},
)

ASTAR_PRIMITIVES = (
    {"name": "forward", "kind": "forward", "cost": 1.0},
    {"name": "left_arc", "kind": "left_arc", "cost": 2.2},
    {"name": "right_arc", "kind": "right_arc", "cost": 2.2},
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Simulate cooperative LattPath vs independent A* on a city street raster.")
    parser.add_argument("--scenario-file", type=Path, required=True, help="Scenario grid file produced from Manhattan OSM.")
    parser.add_argument("--agents-file", type=Path, required=True, help="Agent spawn/goal file.")
    parser.add_argument("--network-file", type=Path, help="Optional per-cell heading metadata produced by the OSM builder.")
    parser.add_argument("--output-prefix", help="Optional prefix for generated simulation JSON filenames.")
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts"), help="Directory for simulation outputs.")
    return parser.parse_args()


def normalize_heading(heading: int) -> int:
    return heading % len(HEADING_VECTORS)


def heading_distance(left: int, right: int) -> int:
    raw = abs(normalize_heading(left) - normalize_heading(right))
    return min(raw, len(HEADING_VECTORS) - raw)


def parse_scenario(path: Path) -> dict:
    lines = [line.rstrip("\n") for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    scenario = {"name": path.stem}
    grid_start = lines.index("grid")

    for line in lines[:grid_start]:
        if line.startswith("name "):
            scenario["name"] = line[5:]
        elif line.startswith("width "):
            scenario["width"] = int(line[6:])
        elif line.startswith("height "):
            scenario["height"] = int(line[7:])
        elif line.startswith("start "):
            x, y, heading = map(int, line[6:].split())
            scenario["start"] = {"x": x, "y": y, "heading": heading}
        elif line.startswith("goal "):
            x, y, heading = map(int, line[5:].split())
            scenario["goal"] = {"x": x, "y": y, "heading": heading}

    grid_lines = lines[grid_start + 1 :]
    scenario["grid"] = list(reversed(grid_lines))
    scenario["free"] = {
        (x, y)
        for y, row in enumerate(scenario["grid"])
        for x, cell in enumerate(row)
        if cell == "."
    }
    return scenario


def parse_network(path: Path) -> dict:
    payload = json.loads(path.read_text(encoding="utf-8"))
    allowed = {
        (entry["x"], entry["y"]): set(entry["allowed_headings"])
        for entry in payload.get("cells", [])
    }
    dominant = {
        (entry["x"], entry["y"]): entry["dominant_heading"]
        for entry in payload.get("cells", [])
    }
    return {
        "scenario": payload.get("scenario"),
        "allowed": allowed,
        "dominant": dominant,
    }


def serialize_scenario(scenario: dict) -> dict:
    obstacles = [
        {"x": x, "y": y}
        for y, row in enumerate(scenario["grid"])
        for x, cell in enumerate(row)
        if cell == "#"
    ]
    return {
        "name": scenario["name"],
        "width": scenario["width"],
        "height": scenario["height"],
        "start": scenario["start"],
        "goal": scenario["goal"],
        "obstacles": obstacles,
    }


def completed_agent_count(planned_agents: list, horizon: int) -> int:
    return sum(1 for agent in planned_agents if agent["done_tick"] is not None and agent["done_tick"] <= horizon)


def heading_allowed(network: dict | None, x: int, y: int, heading: int) -> bool:
    if not network:
        return True
    allowed = network["allowed"].get((x, y))
    if not allowed:
        return True
    return normalize_heading(heading) in allowed


def dominant_cell_heading(network: dict | None, x: int, y: int, fallback: int) -> int:
    if not network:
        return fallback
    return network["dominant"].get((x, y), fallback)


def apply_primitive(scenario: dict, pose: tuple, primitive: dict):
    x, y, heading = pose
    traversed = []
    end_heading = heading

    def advance(step_heading: int) -> bool:
        nonlocal x, y
        dx, dy = HEADING_VECTORS[normalize_heading(step_heading)]
        next_x = x + dx
        next_y = y + dy
        if next_x < 0 or next_x >= scenario["width"] or next_y < 0 or next_y >= scenario["height"]:
            return False
        if (next_x, next_y) not in scenario["free"]:
            return False
        if not heading_allowed(scenario.get("network"), next_x, next_y, step_heading):
            return False
        x, y = next_x, next_y
        traversed.append((x, y))
        return True

    kind = primitive["kind"]
    if kind == "forward":
        for _ in range(primitive.get("steps", 1)):
            if not advance(heading):
                return None
        end_heading = heading
    elif kind == "left_arc":
        if not advance(heading):
            return None
        end_heading = normalize_heading(heading + 1)
        if not advance(end_heading):
            return None
    elif kind == "right_arc":
        if not advance(heading):
            return None
        end_heading = normalize_heading(heading - 1)
        if not advance(end_heading):
            return None
    else:
        return None

    return {
        "end": (x, y, end_heading),
        "cells": traversed,
        "name": primitive["name"],
        "cost": primitive["cost"],
    }


def reconstruct_spatial_path(goal_state: tuple, parents: dict, start_state: tuple) -> dict:
    chain = []
    current = goal_state
    while current != start_state:
        parent, step = parents[current]
        chain.append((current, step))
        current = parent
    chain.reverse()

    states = [{"x": start_state[0], "y": start_state[1], "heading": start_state[2]}]
    cells = [{"x": start_state[0], "y": start_state[1]}]
    primitives = []
    timeline = [{"x": start_state[0], "y": start_state[1], "heading": start_state[2]}]

    current_pose = start_state
    for next_state, step in chain:
        primitives.append(step["name"])
        for cell in step["cells"]:
            cells.append({"x": cell[0], "y": cell[1]})
            timeline.append({"x": cell[0], "y": cell[1], "heading": current_pose[2]})
        current_pose = next_state
        timeline[-1]["heading"] = next_state[2]
        states.append({"x": next_state[0], "y": next_state[1], "heading": next_state[2]})

    return {
        "states": states,
        "cells": cells,
        "primitives": primitives,
        "timeline": timeline,
    }


def plan_spatial(scenario: dict, start: dict, goal: dict, primitives: tuple) -> dict:
    start_state = (start["x"], start["y"], start["heading"])

    frontier = []
    heapq.heappush(frontier, (0.0, 0.0, start_state))
    g_cost = {start_state: 0.0}
    parents = {}
    expanded = []

    while frontier:
        f_score, current_cost, state = heapq.heappop(frontier)
        if current_cost > g_cost[state] + 1e-9:
            continue

        expanded.append({"x": state[0], "y": state[1], "heading": state[2], "g": current_cost, "f": f_score})
        if state[0] == goal["x"] and state[1] == goal["y"] and heading_distance(state[2], goal["heading"]) <= 1:
            path = reconstruct_spatial_path(state, parents, start_state)
            return {
                "success": True,
                "expanded": expanded,
                "path": path,
                "cost": current_cost,
            }

        for primitive in primitives:
            step = apply_primitive(scenario, state, primitive)
            if step is None:
                continue
            neighbor = step["end"]
            next_cost = current_cost + step["cost"]
            if next_cost + 1e-9 >= g_cost.get(neighbor, float("inf")):
                continue
            g_cost[neighbor] = next_cost
            parents[neighbor] = (state, step)
            heuristic = math.hypot(goal["x"] - neighbor[0], goal["y"] - neighbor[1])
            heapq.heappush(frontier, (next_cost + heuristic, next_cost, neighbor))

    return {"success": False}


def build_preferred_lane(leader_timeline: list, scenario: dict, side: str) -> dict:
    preferences = {}
    last_heading = leader_timeline[0]["heading"]
    for time_index in range(1, len(leader_timeline)):
        previous = leader_timeline[time_index - 1]
        current = leader_timeline[time_index]
        dx = current["x"] - previous["x"]
        dy = current["y"] - previous["y"]

        if dx == 0 and dy == 0:
            heading = last_heading
        else:
            heading = next(
                index for index, vector in enumerate(HEADING_VECTORS)
                if vector == (max(-1, min(1, dx)), max(-1, min(1, dy)))
            )
            last_heading = heading

        forward_x, forward_y = HEADING_VECTORS[heading]
        if side == "left":
            offset_x, offset_y = -forward_y, forward_x
        else:
            offset_x, offset_y = forward_y, -forward_x

        candidate = (current["x"] + offset_x, current["y"] + offset_y)
        if candidate in scenario["free"]:
            preferences[time_index] = candidate

    return preferences


def reservation_conflict(cell_reservations: dict, edge_reservations: dict, current_cell: tuple, traversed: list, start_time: int) -> bool:
    previous = current_cell
    for step_index, cell in enumerate(traversed, start=1):
        tick = start_time + step_index
        if cell in cell_reservations.get(tick, set()):
            return True
        if (cell, previous) in edge_reservations.get(tick, set()):
            return True
        previous = cell
    return False


def reconstruct_temporal_path(goal_state: tuple, parents: dict, start_state: tuple) -> dict:
    chain = []
    current = goal_state
    while current != start_state:
        parent, step = parents[current]
        chain.append((current, step))
        current = parent
    chain.reverse()

    states = [{"x": start_state[0], "y": start_state[1], "heading": start_state[2]}]
    cells = [{"x": start_state[0], "y": start_state[1]}]
    primitives = []
    timeline = [{"x": start_state[0], "y": start_state[1], "heading": start_state[2], "t": 0}]

    for next_state, step in chain:
        primitives.append(step["name"])
        current_heading = timeline[-1]["heading"]
        for cell in step["cells"]:
            timeline.append({"x": cell[0], "y": cell[1], "heading": current_heading, "t": len(timeline)})
            cells.append({"x": cell[0], "y": cell[1]})
        timeline[-1]["heading"] = next_state[2]
        states.append({"x": next_state[0], "y": next_state[1], "heading": next_state[2]})

    return {
        "states": states,
        "cells": cells,
        "primitives": primitives,
        "timeline": timeline,
    }


def plan_temporal(
    scenario: dict,
    start: dict,
    goal: dict,
    primitives: tuple,
    cell_reservations: dict,
    edge_reservations: dict,
    preferred_cells: dict = None,
    max_time: int = 180,
) -> dict:
    preferred_cells = preferred_cells or {}
    start_state = (start["x"], start["y"], start["heading"], 0)

    frontier = []
    heapq.heappush(frontier, (0.0, 0.0, start_state))
    g_cost = {start_state: 0.0}
    parents = {}
    expanded = []

    while frontier:
        f_score, current_cost, state = heapq.heappop(frontier)
        if current_cost > g_cost[state] + 1e-9:
            continue

        x, y, heading, current_time = state
        expanded.append({"x": x, "y": y, "heading": heading, "g": current_cost, "f": f_score, "t": current_time})
        if x == goal["x"] and y == goal["y"] and heading_distance(heading, goal["heading"]) <= 1:
            path = reconstruct_temporal_path(state, parents, start_state)
            return {"success": True, "expanded": expanded, "path": path, "cost": current_cost}

        if current_time >= max_time:
            continue

        # Waiting is part of the communication-aware planner so agents can yield.
        wait_cell = (x, y)
        if wait_cell not in cell_reservations.get(current_time + 1, set()):
            wait_state = (x, y, heading, current_time + 1)
            wait_cost = current_cost + 1.0
            if wait_cost + 1e-9 < g_cost.get(wait_state, float("inf")):
                g_cost[wait_state] = wait_cost
                parents[wait_state] = (state, {"name": "wait", "cells": [(x, y)]})
                heuristic = math.hypot(goal["x"] - x, goal["y"] - y)
                heapq.heappush(frontier, (wait_cost + heuristic, wait_cost, wait_state))

        for primitive in primitives:
            step = apply_primitive(scenario, (x, y, heading), primitive)
            if step is None:
                continue

            duration = len(step["cells"])
            next_time = current_time + duration
            if next_time > max_time:
                continue
            if reservation_conflict(cell_reservations, edge_reservations, (x, y), step["cells"], current_time):
                continue

            neighbor = (step["end"][0], step["end"][1], step["end"][2], next_time)
            preference_penalty = 0.0
            preferred = preferred_cells.get(next_time)
            if preferred is not None and preferred != (step["end"][0], step["end"][1]):
                preference_penalty = 0.2

            next_cost = current_cost + step["cost"] + preference_penalty
            if next_cost + 1e-9 >= g_cost.get(neighbor, float("inf")):
                continue
            g_cost[neighbor] = next_cost
            parents[neighbor] = (state, step)
            heuristic = math.hypot(goal["x"] - step["end"][0], goal["y"] - step["end"][1])
            heapq.heappush(frontier, (next_cost + heuristic, next_cost, neighbor))

    return {"success": False}


def reserve_timeline(path: dict, cell_reservations: dict, edge_reservations: dict, hold_until: int = None) -> None:
    timeline = path["timeline"]
    start = timeline[0]
    cell_reservations[0].add((start["x"], start["y"]))
    for index in range(1, len(timeline)):
        current = timeline[index]
        previous = timeline[index - 1]
        tick = current["t"]
        cell_reservations[tick].add((current["x"], current["y"]))
        edge_reservations[tick].add(((previous["x"], previous["y"]), (current["x"], current["y"])))

    if hold_until is None:
        return

    final = timeline[-1]
    for tick in range(final["t"] + 1, hold_until + 1):
        cell_reservations[tick].add((final["x"], final["y"]))


def count_wait_steps(timeline: list) -> int:
    return sum(
        1
        for previous, current in zip(timeline, timeline[1:])
        if previous["x"] == current["x"] and previous["y"] == current["y"]
    )


def path_overlap_score(left: dict, right: dict) -> int:
    left_cells = {(cell["x"], cell["y"]) for cell in left["path"]["cells"]}
    right_cells = {(cell["x"], cell["y"]) for cell in right["path"]["cells"]}
    exact = len(left_cells & right_cells)
    adjacent = 0
    for x, y in left_cells:
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                if (x + dx, y + dy) in right_cells:
                    adjacent += 1
    return exact * 4 + adjacent


def choose_pair_side(leader: dict, follower: dict) -> str:
    leader_start = leader["timeline"][0]
    follower_start = follower["start"]
    if len(leader["timeline"]) > 1:
        next_state = leader["timeline"][1]
        dx = next_state["x"] - leader_start["x"]
        dy = next_state["y"] - leader_start["y"]
    else:
        heading = leader_start["heading"]
        dx, dy = HEADING_VECTORS[heading]

    relative_x = follower_start["x"] - leader_start["x"]
    relative_y = follower_start["y"] - leader_start["y"]
    cross = dx * relative_y - dy * relative_x
    return "left" if cross > 0 else "right"


def build_communication_pairs(scenario: dict, agents: list) -> tuple:
    nominal_plans = {}
    for agent in agents:
        plan = plan_spatial(scenario, agent["start"], agent["goal"], LATTPATH_PRIMITIVES)
        if not plan["success"]:
            raise RuntimeError(f"Nominal LattPath failed for {agent['id']}")
        nominal_plans[agent["id"]] = plan

    candidates = []
    for index, left in enumerate(agents):
        for right in agents[index + 1 :]:
            score = path_overlap_score(nominal_plans[left["id"]], nominal_plans[right["id"]])
            if score > 0:
                candidates.append((score, left["id"], right["id"]))
    candidates.sort(reverse=True)

    by_id = {agent["id"]: agent for agent in agents}
    used = set()
    ordered_groups = []
    formation_pairs = []

    for score, left_id, right_id in candidates:
        if left_id in used or right_id in used:
            continue
        left_plan = nominal_plans[left_id]
        right_plan = nominal_plans[right_id]
        leader_id, follower_id = (
            (left_id, right_id)
            if len(left_plan["path"]["cells"]) >= len(right_plan["path"]["cells"])
            else (right_id, left_id)
        )
        leader = by_id[leader_id]
        follower = by_id[follower_id]
        side = choose_pair_side(
            {"timeline": nominal_plans[leader_id]["path"]["timeline"]},
            follower,
        )
        ordered_groups.append((leader, follower, side))
        formation_pairs.append({
            "leader": leader_id,
            "follower": follower_id,
            "side": side,
            "overlap_score": score,
        })
        used.add(left_id)
        used.add(right_id)

    for agent in agents:
        if agent["id"] not in used:
            ordered_groups.append((agent, None, None))

    return ordered_groups, formation_pairs


def simulate_independent_astar(scenario: dict, agents: list) -> dict:
    planned_agents = []
    for agent in agents:
        plan = plan_spatial(scenario, agent["start"], agent["goal"], ASTAR_PRIMITIVES)
        if not plan["success"]:
            raise RuntimeError(f"Independent A* failed for {agent['id']}")
        planned_agents.append({
            "id": agent["id"],
            "start": agent["start"],
            "goal": agent["goal"],
            "timeline": plan["path"]["timeline"],
            "cursor": 0,
            "done_tick": None,
        })

    ticks = 0
    conflict_count = 0
    wait_events = 0
    history = []
    max_ticks = max(len(agent["timeline"]) for agent in planned_agents) * 4

    while ticks < max_ticks:
        state_snapshot = []
        proposals = {}
        edge_claims = {}
        completed = 0

        for agent in planned_agents:
            timeline = agent["timeline"]
            current_index = agent["cursor"]
            current = timeline[current_index]

            if current_index >= len(timeline) - 1:
                completed += 1
                state_snapshot.append({"id": agent["id"], "x": current["x"], "y": current["y"], "heading": current["heading"]})
                continue

            next_state = timeline[current_index + 1]
            proposals.setdefault((next_state["x"], next_state["y"]), []).append(agent["id"])
            edge_claims[agent["id"]] = ((current["x"], current["y"]), (next_state["x"], next_state["y"]))
            state_snapshot.append({"id": agent["id"], "x": current["x"], "y": current["y"], "heading": current["heading"]})

        history.append(state_snapshot)
        if completed == len(planned_agents):
            break

        blocked_agents = set()
        for claimants in proposals.values():
            if len(claimants) > 1:
                blocked_agents.update(claimants)

        edge_items = list(edge_claims.items())
        for index, (left_id, left_edge) in enumerate(edge_items):
            for right_id, right_edge in edge_items[index + 1 :]:
                if left_edge[0] == right_edge[1] and left_edge[1] == right_edge[0]:
                    blocked_agents.add(left_id)
                    blocked_agents.add(right_id)

        for agent in planned_agents:
            if agent["id"] in blocked_agents:
                conflict_count += 1
                wait_events += 1
                continue
            if agent["cursor"] < len(agent["timeline"]) - 1:
                agent["cursor"] += 1
                if agent["cursor"] == len(agent["timeline"]) - 1 and agent["done_tick"] is None:
                    agent["done_tick"] = ticks + 1

        ticks += 1

    return {
        "mode": "independent_astar",
        "ticks": ticks,
        "conflicts": conflict_count,
        "wait_events": wait_events,
        "completed_agents": completed_agent_count(planned_agents, ticks),
        "history": history,
        "agents": [
            {
                "id": agent["id"],
                "start": agent["start"],
                "goal": agent["goal"],
                "timeline": agent["timeline"],
                "done_tick": agent["done_tick"] if agent["done_tick"] is not None else ticks,
            }
            for agent in planned_agents
        ],
    }


def simulate_cooperative_lattpath(scenario: dict, agents: list) -> dict:
    cell_reservations = defaultdict(set)
    edge_reservations = defaultdict(set)
    planned_agents = []
    ordered_groups, formation_pairs = build_communication_pairs(scenario, agents)
    nominal_lengths = []
    for group in ordered_groups:
        leader = group[0]
        nominal = plan_spatial(scenario, leader["start"], leader["goal"], LATTPATH_PRIMITIVES)
        if nominal["success"]:
            nominal_lengths.append(len(nominal["path"]["timeline"]))
        follower = group[1]
        if follower is not None:
            nominal = plan_spatial(scenario, follower["start"], follower["goal"], LATTPATH_PRIMITIVES)
            if nominal["success"]:
                nominal_lengths.append(len(nominal["path"]["timeline"]))
    planning_horizon = max(nominal_lengths or [60]) * 3

    for leader, follower, side in ordered_groups:
        leader_plan = plan_temporal(
            scenario,
            leader["start"],
            leader["goal"],
            LATTPATH_PRIMITIVES,
            cell_reservations,
            edge_reservations,
            max_time=planning_horizon,
        )
        if not leader_plan["success"]:
            raise RuntimeError(f"Cooperative LattPath failed for {leader['id']}")

        reserve_timeline(leader_plan["path"], cell_reservations, edge_reservations, hold_until=planning_horizon)
        planned_agents.append({
            "id": leader["id"],
            "start": leader["start"],
            "goal": leader["goal"],
            "timeline": leader_plan["path"]["timeline"],
            "done_tick": leader_plan["path"]["timeline"][-1]["t"],
        })

        if follower is None:
            continue

        preferred = build_preferred_lane(leader_plan["path"]["timeline"], scenario, side)
        follower_plan = plan_temporal(
            scenario,
            follower["start"],
            follower["goal"],
            LATTPATH_PRIMITIVES,
            cell_reservations,
            edge_reservations,
            preferred_cells=preferred,
            max_time=planning_horizon,
        )
        if not follower_plan["success"]:
            raise RuntimeError(f"Cooperative LattPath failed for {follower['id']}")

        reserve_timeline(follower_plan["path"], cell_reservations, edge_reservations, hold_until=planning_horizon)
        planned_agents.append({
            "id": follower["id"],
            "start": follower["start"],
            "goal": follower["goal"],
            "timeline": follower_plan["path"]["timeline"],
            "done_tick": follower_plan["path"]["timeline"][-1]["t"],
        })

    max_tick = max(agent["done_tick"] for agent in planned_agents)
    history = []
    for tick in range(max_tick + 1):
        frame = []
        for agent in planned_agents:
            timeline = agent["timeline"]
            state = timeline[min(tick, len(timeline) - 1)]
            frame.append({"id": agent["id"], "x": state["x"], "y": state["y"], "heading": state["heading"]})
        history.append(frame)

    return {
        "mode": "cooperative_lattpath",
        "ticks": max_tick,
        "conflicts": 0,
        "wait_events": sum(count_wait_steps(agent["timeline"]) for agent in planned_agents),
        "completed_agents": completed_agent_count(planned_agents, planning_horizon),
        "formation_pairs": formation_pairs,
        "history": history,
        "agents": planned_agents,
    }


def main() -> None:
    args = parse_args()
    scenario = parse_scenario(args.scenario_file)
    if args.network_file is not None:
        scenario["network"] = parse_network(args.network_file)
    agents = json.loads(args.agents_file.read_text(encoding="utf-8"))["agents"]
    output_prefix = args.output_prefix or scenario["name"]

    args.output_dir.mkdir(parents=True, exist_ok=True)

    independent = simulate_independent_astar(scenario, agents)
    independent["scenario"] = scenario["name"]
    independent["grid"] = serialize_scenario(scenario)
    (args.output_dir / f"{output_prefix}_independent_astar_simulation.json").write_text(
        json.dumps(independent, indent=2) + "\n",
        encoding="utf-8",
    )

    cooperative = simulate_cooperative_lattpath(scenario, agents)
    cooperative["scenario"] = scenario["name"]
    cooperative["grid"] = serialize_scenario(scenario)
    (args.output_dir / f"{output_prefix}_cooperative_lattpath_simulation.json").write_text(
        json.dumps(cooperative, indent=2) + "\n",
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
