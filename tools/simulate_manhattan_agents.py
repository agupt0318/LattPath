#!/usr/bin/env python3

import argparse
import heapq
import json
import math
from collections import defaultdict
from pathlib import Path
from typing import Optional

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
    parser.add_argument("--controls-file", type=Path, help="Optional traffic-control and behavior overlay produced by the city builder.")
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
    free_flow = {
        (entry["x"], entry["y"]): float(entry.get("free_flow_kph", 24.0))
        for entry in payload.get("cells", [])
    }
    highways = {
        (entry["x"], entry["y"]): entry.get("dominant_highway", "residential")
        for entry in payload.get("cells", [])
    }
    max_free_flow_kph = max(free_flow.values(), default=24.0)
    return {
        "schema_version": payload.get("schema_version", 1),
        "city_id": payload.get("city_id"),
        "display_name": payload.get("display_name"),
        "scenario": payload.get("scenario"),
        "meters_per_cell_x": float(payload.get("meters_per_cell_x", 1.0)),
        "meters_per_cell_y": float(payload.get("meters_per_cell_y", 1.0)),
        "allowed": allowed,
        "dominant": dominant,
        "free_flow_kph": free_flow,
        "dominant_highway": highways,
        "max_free_flow_kph": max_free_flow_kph,
    }


def parse_controls(path: Path) -> dict:
    payload = json.loads(path.read_text(encoding="utf-8"))
    traffic_lights = {
        (entry["x"], entry["y"]): {
            "id": entry.get("id", f"traffic_light_{entry['x']}_{entry['y']}"),
            "cycle_ticks": int(entry.get("cycle_ticks", 8)),
            "green_ticks": int(entry.get("green_ticks", 4)),
            "offset_ticks": int(entry.get("offset_ticks", 0)),
        }
        for entry in payload.get("traffic_lights", [])
    }
    stop_signs = {
        (entry["x"], entry["y"]): {
            "id": entry.get("id", f"stop_sign_{entry['x']}_{entry['y']}"),
            "hold_ticks": int(entry.get("hold_ticks", 1)),
        }
        for entry in payload.get("stop_signs", [])
    }
    intersections = {
        (entry["x"], entry["y"])
        for entry in payload.get("intersection_cells", [])
    }
    return {
        "scenario": payload.get("scenario"),
        "tick_seconds": float(payload.get("tick_seconds", 1.0)),
        "vehicle_model": payload.get("vehicle_model", {}),
        "sensor_model": payload.get("sensor_model", {}),
        "human_driver_model": payload.get("human_driver_model", {}),
        "traffic_lights": traffic_lights,
        "stop_signs": stop_signs,
        "intersection_cells": intersections,
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
        "controls": summarize_controls(scenario),
    }


def primitive_names(primitives: tuple) -> list:
    return [primitive["name"] for primitive in primitives]


def summarize_plan(plan: dict, temporal: bool) -> dict:
    path = plan["path"]
    return {
        "state_representation": "(x, y, heading, t)" if temporal else "(x, y, heading)",
        "expanded_states": len(plan["expanded"]),
        "cost": plan["cost"],
        "uses_wait_primitive": "wait" in path["primitives"],
        "path": path,
        "expanded": plan["expanded"],
    }


def summarize_reservations(cell_reservations: dict, edge_reservations: dict) -> dict:
    return {
        "reserved_ticks": len(cell_reservations),
        "reserved_cell_entries": sum(len(cells) for cells in cell_reservations.values()),
        "reserved_edge_entries": sum(len(edges) for edges in edge_reservations.values()),
    }


def summarize_controls(scenario: dict) -> dict:
    controls = scenario.get("controls", {})
    return {
        "tick_seconds": float(controls.get("tick_seconds", 1.0)),
        "traffic_lights": [
            {"x": x, "y": y, **spec}
            for (x, y), spec in sorted(controls.get("traffic_lights", {}).items())
        ],
        "stop_signs": [
            {"x": x, "y": y, **spec}
            for (x, y), spec in sorted(controls.get("stop_signs", {}).items())
        ],
        "intersection_count": len(controls.get("intersection_cells", set())),
        "vehicle_model": controls.get("vehicle_model", {}),
        "sensor_model": controls.get("sensor_model", {}),
        "human_driver_model": controls.get("human_driver_model", {}),
    }


def completed_agent_count(planned_agents: list, horizon: int) -> int:
    return sum(1 for agent in planned_agents if agent["done_tick"] is not None and agent["done_tick"] <= horizon)


def heading_allowed(network: Optional[dict], x: int, y: int, heading: int) -> bool:
    if not network:
        return True
    allowed = network["allowed"].get((x, y))
    if not allowed:
        return True
    return normalize_heading(heading) in allowed


def desired_heading_between_points(start: tuple, goal: tuple) -> int:
    dx = goal[0] - start[0]
    dy = goal[1] - start[1]
    if dx == 0 and dy == 0:
        return 0
    unit = (max(-1, min(1, dx)), max(-1, min(1, dy)))
    return next(index for index, vector in enumerate(HEADING_VECTORS) if vector == unit)


def select_local_heading(scenario: dict, x: int, y: int, target: tuple, fallback: int) -> int:
    network = scenario.get("network")
    if not network:
        return fallback
    allowed = network["allowed"].get((x, y))
    if not allowed:
        return fallback
    desired = desired_heading_between_points((x, y), target)
    return min(allowed, key=lambda heading: heading_distance(heading, desired))


def step_distance_meters(network: Optional[dict], heading: int) -> float:
    if not network:
        return 1.0
    dx, dy = HEADING_VECTORS[normalize_heading(heading)]
    return math.hypot(
        abs(dx) * network.get("meters_per_cell_x", 1.0),
        abs(dy) * network.get("meters_per_cell_y", 1.0),
    )


def cell_travel_seconds(network: Optional[dict], x: int, y: int, heading: int) -> float:
    if not network:
        return 1.0
    speed_kph = network["free_flow_kph"].get((x, y), 24.0)
    meters_per_second = max(speed_kph * 1000.0 / 3600.0, 0.1)
    return step_distance_meters(network, heading) / meters_per_second


def controls_for_scenario(scenario: dict) -> dict:
    return scenario.get("controls", {})


def tick_seconds(scenario: dict) -> float:
    return max(controls_for_scenario(scenario).get("tick_seconds", 1.0), 0.1)


def vehicle_model(scenario: dict) -> dict:
    return controls_for_scenario(scenario).get("vehicle_model", {})


def sensor_model(scenario: dict) -> dict:
    return controls_for_scenario(scenario).get("sensor_model", {})


def human_driver_model(scenario: dict) -> dict:
    return controls_for_scenario(scenario).get("human_driver_model", {})


def traffic_light_at(scenario: dict, cell: tuple) -> Optional[dict]:
    return controls_for_scenario(scenario).get("traffic_lights", {}).get(cell)


def stop_sign_at(scenario: dict, cell: tuple) -> Optional[dict]:
    return controls_for_scenario(scenario).get("stop_signs", {}).get(cell)


def is_intersection_cell(scenario: dict, cell: tuple) -> bool:
    return cell in controls_for_scenario(scenario).get("intersection_cells", set())


def primitive_distance_meters(scenario: dict, start_heading: int, end_heading: int, traversed: list) -> float:
    if not traversed:
        return 0.0
    network = scenario.get("network")
    heading = start_heading
    total = 0.0
    for index, cell in enumerate(traversed):
        if index == len(traversed) - 1:
            heading = end_heading
        total += step_distance_meters(network, heading)
    return total


def primitive_travel_ticks(scenario: dict, start_heading: int, end_heading: int, traversed: list) -> int:
    if not traversed:
        return 1

    model = vehicle_model(scenario)
    base_seconds = primitive_travel_cost(scenario, start_heading, end_heading, traversed)
    max_speed_mps = float(model.get("max_speed_mps", 11.0))
    acceleration_mps2 = max(float(model.get("acceleration_mps2", 2.5)), 0.1)
    turn_speed_scale = min(max(float(model.get("turn_speed_scale", 0.65)), 0.2), 1.0)
    distance_meters = primitive_distance_meters(scenario, start_heading, end_heading, traversed)

    if start_heading != end_heading:
        max_speed_mps *= turn_speed_scale
        acceleration_mps2 *= turn_speed_scale

    time_to_max = max_speed_mps / acceleration_mps2
    accel_distance = 0.5 * acceleration_mps2 * time_to_max * time_to_max

    if distance_meters <= 0.0:
        envelope_seconds = base_seconds
    elif (2.0 * accel_distance) >= distance_meters:
        envelope_seconds = 2.0 * math.sqrt(distance_meters / acceleration_mps2)
    else:
        cruise_distance = distance_meters - (2.0 * accel_distance)
        envelope_seconds = (2.0 * time_to_max) + (cruise_distance / max_speed_mps)

    total_seconds = max(base_seconds, envelope_seconds)
    return max(1, int(math.ceil(total_seconds / tick_seconds(scenario))))


def signal_wait_ticks(light: dict, arrival_tick: int) -> int:
    cycle_ticks = max(int(light.get("cycle_ticks", 8)), 1)
    green_ticks = max(0, min(int(light.get("green_ticks", cycle_ticks // 2)), cycle_ticks))
    offset_ticks = int(light.get("offset_ticks", 0))
    phase = (arrival_tick + offset_ticks) % cycle_ticks
    if phase < green_ticks:
        return 0
    return cycle_ticks - phase


def sensitive_cells_for_step(scenario: dict, traversed: list) -> int:
    count = 0
    for cell in traversed:
        if traffic_light_at(scenario, cell) or stop_sign_at(scenario, cell) or is_intersection_cell(scenario, cell):
            count += 1
    return count


def sensor_risk_penalty(scenario: dict, traversed: list, automated: bool) -> float:
    model = sensor_model(scenario)
    if not model or not traversed:
        return 0.0

    risk_weight = float(model.get("risk_weight", 0.35))
    communication_discount = float(model.get("communication_discount", 0.5 if automated else 1.0))
    control_buffer = max(int(model.get("control_buffer_cells", 1)), 0)
    if not automated:
        communication_discount = 1.0

    sensitive_cells = set()
    controls = controls_for_scenario(scenario)
    sensitive_cells.update(controls.get("traffic_lights", {}).keys())
    sensitive_cells.update(controls.get("stop_signs", {}).keys())
    sensitive_cells.update(controls.get("intersection_cells", set()))
    if not sensitive_cells:
        return 0.0

    total = 0.0
    for x, y in traversed:
        hit = False
        for dx in range(-control_buffer, control_buffer + 1):
            for dy in range(-control_buffer, control_buffer + 1):
                if (x + dx, y + dy) in sensitive_cells:
                    hit = True
                    break
            if hit:
                break
        if hit:
            total += 1.0

    return risk_weight * communication_discount * total


def fixed_preparation_ticks(scenario: dict, traversed: list, automated: bool) -> tuple:
    if not traversed:
        return 0, {"stop_sign_waits": 0, "sensor_caution_waits": 0, "human_reaction_waits": 0}

    delays = {
        "stop_sign_waits": 0,
        "sensor_caution_waits": 0,
        "human_reaction_waits": 0,
    }

    first_control_index = None
    for index, cell in enumerate(traversed):
        if traffic_light_at(scenario, cell) or stop_sign_at(scenario, cell):
            first_control_index = index
            break

    if first_control_index is not None:
        control_cell = traversed[first_control_index]
        stop = stop_sign_at(scenario, control_cell)
        if stop is not None:
            delays["stop_sign_waits"] += int(stop.get("hold_ticks", 1))

    if sensitive_cells_for_step(scenario, traversed):
        model = sensor_model(scenario)
        caution_ticks = int(model.get("caution_ticks", 0 if automated else 1))
        delays["sensor_caution_waits"] += caution_ticks

    if not automated:
        human_model = human_driver_model(scenario)
        delays["human_reaction_waits"] += int(human_model.get("reaction_delay_ticks", 1))
        if sensitive_cells_for_step(scenario, traversed):
            delays["human_reaction_waits"] += int(human_model.get("intersection_scan_ticks", 1))

    total_delay = sum(delays.values())
    return total_delay, delays


def traffic_light_wait_ticks(scenario: dict, current_time: int, traversed: list) -> tuple:
    if not traversed:
        return 0, {"traffic_light_waits": 0}

    for index, cell in enumerate(traversed):
        light = traffic_light_at(scenario, cell)
        if light is None:
            continue
        arrival_tick = current_time + index + 1
        wait_ticks = signal_wait_ticks(light, arrival_tick)
        return wait_ticks, {"traffic_light_waits": wait_ticks}

    return 0, {"traffic_light_waits": 0}


def primitive_travel_cost(scenario: dict, start_heading: int, end_heading: int, traversed: list) -> float:
    network = scenario.get("network")
    if not traversed:
        return 0.0

    total = 0.0
    heading = start_heading
    for index, cell in enumerate(traversed):
        if index == len(traversed) - 1:
            heading = end_heading
        total += cell_travel_seconds(network, cell[0], cell[1], heading)

    if start_heading != end_heading:
        total += 1.5
    return total


def remaining_travel_time_heuristic(scenario: dict, pose: tuple, goal: dict) -> float:
    network = scenario.get("network")
    dx = goal["x"] - pose[0]
    dy = goal["y"] - pose[1]

    if network:
        meters = math.hypot(
            dx * network.get("meters_per_cell_x", 1.0),
            dy * network.get("meters_per_cell_y", 1.0),
        )
        max_speed_mps = max(network.get("max_free_flow_kph", 24.0) * 1000.0 / 3600.0, 0.1)
        return (meters / max_speed_mps) + (0.25 * heading_distance(pose[2], goal["heading"]))

    return math.hypot(dx, dy) + (0.25 * heading_distance(pose[2], goal["heading"]))


def remaining_temporal_heuristic(pose: tuple, goal: dict) -> float:
    return math.hypot(goal["x"] - pose[0], goal["y"] - pose[1]) + (0.25 * heading_distance(pose[2], goal["heading"]))


def normalize_agent_headings(scenario: dict, agents: list) -> list:
    normalized = []
    for agent in agents:
        start = dict(agent["start"])
        goal = dict(agent["goal"])
        desired = desired_heading_between_points((start["x"], start["y"]), (goal["x"], goal["y"]))
        step_dx, step_dy = HEADING_VECTORS[desired]
        start["heading"] = select_local_heading(scenario, start["x"], start["y"], (goal["x"], goal["y"]), start["heading"])
        goal["heading"] = select_local_heading(
            scenario,
            goal["x"],
            goal["y"],
            (goal["x"] + step_dx, goal["y"] + step_dy),
            goal["heading"],
        )
        normalized.append({
            "id": agent["id"],
            "start": start,
            "goal": goal,
        })
    return normalized


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
        "cost": primitive_travel_cost(scenario, heading, end_heading, traversed),
        "duration_ticks": primitive_travel_ticks(scenario, heading, end_heading, traversed),
        "sensor_risk_cost": sensor_risk_penalty(scenario, traversed, automated=True),
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
            next_cost = current_cost + step["cost"] + step.get("sensor_risk_cost", 0.0)
            if next_cost + 1e-9 >= g_cost.get(neighbor, float("inf")):
                continue
            g_cost[neighbor] = next_cost
            parents[neighbor] = (state, step)
            heuristic = remaining_travel_time_heuristic(scenario, neighbor, goal)
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


def expand_step_positions(current_cell: tuple, traversed: list, movement_ticks: int) -> list:
    if not traversed:
        return [current_cell] * max(movement_ticks, 1)

    checkpoints = [
        max(1, int(math.ceil(((index + 1) * movement_ticks) / len(traversed))))
        for index in range(len(traversed))
    ]
    positions = []
    last_cell = current_cell
    cursor = 0
    for tick in range(1, movement_ticks + 1):
        while cursor < len(checkpoints) and tick >= checkpoints[cursor]:
            last_cell = traversed[cursor]
            cursor += 1
        positions.append(last_cell)
    return positions


def reservation_conflict(cell_reservations: dict, edge_reservations: dict, current_cell: tuple, timed_cells: list, start_time: int) -> bool:
    previous = current_cell
    for step_index, cell in enumerate(timed_cells, start=1):
        tick = start_time + step_index
        if cell in cell_reservations.get(tick, set()):
            return True
        if cell != previous and (cell, previous) in edge_reservations.get(tick, set()):
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
        for index, cell in enumerate(step.get("timed_cells", step["cells"])):
            timeline.append({"x": cell[0], "y": cell[1], "heading": current_heading, "t": len(timeline)})
            if index == len(step.get("timed_cells", step["cells"])) - 1 or not cells or cells[-1] != {"x": cell[0], "y": cell[1]}:
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
                parents[wait_state] = (state, {"name": "wait", "cells": [(x, y)], "timed_cells": [(x, y)]})
                heuristic = remaining_temporal_heuristic((x, y, heading), goal)
                heapq.heappush(frontier, (wait_cost + heuristic, wait_cost, wait_state))

        for primitive in primitives:
            step = apply_primitive(scenario, (x, y, heading), primitive)
            if step is None:
                continue

            prep_wait_ticks, prep_breakdown = fixed_preparation_ticks(scenario, step["cells"], automated=True)
            signal_wait, signal_breakdown = traffic_light_wait_ticks(scenario, current_time, step["cells"])
            pre_wait_ticks = prep_wait_ticks + signal_wait
            wait_breakdown = dict(prep_breakdown)
            wait_breakdown.update(signal_breakdown)
            planner_wait_cap = int(sensor_model(scenario).get("planner_wait_cap_ticks", 1))
            bounded_wait_ticks = min(pre_wait_ticks, planner_wait_cap)
            movement_ticks = step.get("duration_ticks", len(step["cells"]))
            timed_cells = ([(x, y)] * bounded_wait_ticks) + expand_step_positions((x, y), step["cells"], movement_ticks)
            duration = len(timed_cells)
            next_time = current_time + duration
            if next_time > max_time:
                continue
            if reservation_conflict(cell_reservations, edge_reservations, (x, y), timed_cells, current_time):
                continue

            neighbor = (step["end"][0], step["end"][1], step["end"][2], next_time)
            preference_penalty = 0.0
            preferred = preferred_cells.get(next_time)
            if preferred is not None and preferred != (step["end"][0], step["end"][1]):
                preference_penalty = 0.2

            next_cost = current_cost + step["cost"] + step.get("sensor_risk_cost", 0.0) + pre_wait_ticks + movement_ticks + preference_penalty
            if next_cost + 1e-9 >= g_cost.get(neighbor, float("inf")):
                continue
            g_cost[neighbor] = next_cost
            enriched_step = dict(step)
            enriched_step["wait_ticks"] = pre_wait_ticks
            enriched_step["bounded_wait_ticks"] = bounded_wait_ticks
            enriched_step["wait_breakdown"] = wait_breakdown
            enriched_step["timed_cells"] = timed_cells
            parents[neighbor] = (state, enriched_step)
            heuristic = remaining_temporal_heuristic(step["end"], goal)
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
        if (previous["x"], previous["y"]) != (current["x"], current["y"]):
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


def schedule_spatial_timeline(
    scenario: dict,
    base_timeline: list,
    cell_reservations: dict,
    edge_reservations: dict,
    automated: bool,
    max_tick: int,
) -> list:
    scheduled = [{
        "x": base_timeline[0]["x"],
        "y": base_timeline[0]["y"],
        "heading": base_timeline[0]["heading"],
        "t": 0,
    }]

    for next_state in base_timeline[1:]:
        prep_ticks, _ = fixed_preparation_ticks(scenario, [(next_state["x"], next_state["y"])], automated=automated)
        for _ in range(prep_ticks):
            current = scheduled[-1]
            if current["t"] >= max_tick:
                raise RuntimeError("Exceeded scheduling horizon while applying controls and reservations.")
            scheduled.append({
                "x": current["x"],
                "y": current["y"],
                "heading": current["heading"],
                "t": current["t"] + 1,
            })

        while True:
            current = scheduled[-1]
            current_cell = (current["x"], current["y"])
            target_cell = (next_state["x"], next_state["y"])
            if current["t"] >= max_tick:
                raise RuntimeError("Exceeded scheduling horizon while applying controls and reservations.")

            delay_ticks, _ = traffic_light_wait_ticks(scenario, current["t"], [target_cell])
            blocked = (
                target_cell in cell_reservations.get(current["t"] + 1, set())
                or (target_cell, current_cell) in edge_reservations.get(current["t"] + 1, set())
            )

            if delay_ticks > 0 or blocked:
                scheduled.append({
                    "x": current["x"],
                    "y": current["y"],
                    "heading": current["heading"],
                    "t": current["t"] + 1,
                })
                continue

            scheduled.append({
                "x": next_state["x"],
                "y": next_state["y"],
                "heading": next_state["heading"],
                "t": current["t"] + 1,
            })
            break

    return scheduled


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


def choose_priority_winner(scenario: dict, target_cell: tuple, claimants: list) -> Optional[str]:
    if stop_sign_at(scenario, target_cell) or traffic_light_at(scenario, target_cell) or is_intersection_cell(scenario, target_cell):
        return sorted(claimants)[0]
    return None


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
            "pending_waits": 0,
            "prepared_target": None,
            "plan": summarize_plan(plan, temporal=False),
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
        paused_agents = set()

        for agent in planned_agents:
            timeline = agent["timeline"]
            current_index = agent["cursor"]
            current = timeline[current_index]

            if current_index >= len(timeline) - 1:
                completed += 1
                state_snapshot.append({"id": agent["id"], "x": current["x"], "y": current["y"], "heading": current["heading"]})
                continue

            if agent["pending_waits"] > 0:
                agent["pending_waits"] -= 1
                paused_agents.add(agent["id"])
                wait_events += 1
                state_snapshot.append({"id": agent["id"], "x": current["x"], "y": current["y"], "heading": current["heading"]})
                continue

            next_state = timeline[current_index + 1]
            target_cell = (next_state["x"], next_state["y"])
            if agent["prepared_target"] != target_cell:
                prep_ticks, _ = fixed_preparation_ticks(scenario, [target_cell], automated=False)
                if prep_ticks > 0:
                    agent["pending_waits"] = max(prep_ticks - 1, 0)
                    agent["prepared_target"] = target_cell
                    paused_agents.add(agent["id"])
                    wait_events += 1
                    state_snapshot.append({"id": agent["id"], "x": current["x"], "y": current["y"], "heading": current["heading"]})
                    continue

            signal_wait, _ = traffic_light_wait_ticks(scenario, ticks, [target_cell])
            if signal_wait > 0:
                agent["pending_waits"] = max(signal_wait - 1, 0)
                paused_agents.add(agent["id"])
                wait_events += 1
                state_snapshot.append({"id": agent["id"], "x": current["x"], "y": current["y"], "heading": current["heading"]})
                continue

            proposals.setdefault(target_cell, []).append(agent["id"])
            edge_claims[agent["id"]] = ((current["x"], current["y"]), target_cell)
            state_snapshot.append({"id": agent["id"], "x": current["x"], "y": current["y"], "heading": current["heading"]})

        history.append(state_snapshot)
        if completed == len(planned_agents):
            break

        blocked_agents = set()
        for target_cell, claimants in proposals.items():
            if len(claimants) > 1:
                winner = choose_priority_winner(scenario, target_cell, claimants)
                if winner is None:
                    blocked_agents.update(claimants)
                else:
                    blocked_agents.update(agent_id for agent_id in claimants if agent_id != winner)

        edge_items = list(edge_claims.items())
        for index, (left_id, left_edge) in enumerate(edge_items):
            for right_id, right_edge in edge_items[index + 1 :]:
                if left_edge[0] == right_edge[1] and left_edge[1] == right_edge[0]:
                    blocked_agents.add(left_id)
                    blocked_agents.add(right_id)

        for agent in planned_agents:
            if agent["id"] in paused_agents:
                continue
            if agent["id"] in blocked_agents:
                conflict_count += 1
                wait_events += 1
                continue
            if agent["cursor"] < len(agent["timeline"]) - 1:
                agent["cursor"] += 1
                agent["prepared_target"] = None
                if agent["cursor"] == len(agent["timeline"]) - 1 and agent["done_tick"] is None:
                    agent["done_tick"] = ticks + 1

        ticks += 1

    return {
        "mode": "independent_astar",
        "lattice": {
            "planner": "spatial_lattice",
            "state_representation": "(x, y, heading)",
            "primitive_set": primitive_names(ASTAR_PRIMITIVES),
            "wait_primitive_enabled": False,
            "reservation_model": "none",
            "expanded_states_total": sum(agent["plan"]["expanded_states"] for agent in planned_agents),
        },
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
                "plan": agent["plan"],
                "role": "independent",
            }
            for agent in planned_agents
        ],
    }


def simulate_cooperative_lattpath(scenario: dict, agents: list) -> dict:
    cell_reservations = defaultdict(set)
    edge_reservations = defaultdict(set)
    planned_agents = []
    if scenario.get("network") or scenario.get("controls"):
        ordered_groups, formation_pairs = build_communication_pairs(scenario, agents)
    else:
        ordered_groups = [(agent, None, None) for agent in agents]
        formation_pairs = []
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
    control_count = len(controls_for_scenario(scenario).get("traffic_lights", {})) + len(controls_for_scenario(scenario).get("stop_signs", {}))
    planning_horizon = (max(nominal_lengths or [60]) * 20) + (control_count * 20)

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
            "plan": summarize_plan(leader_plan, temporal=True),
            "role": "solo" if follower is None else "leader",
        })

        if follower is None:
            continue

        preferred_cells = build_preferred_lane(leader_plan["path"]["timeline"], scenario, side)
        follower_plan = plan_temporal(
            scenario,
            follower["start"],
            follower["goal"],
            LATTPATH_PRIMITIVES,
            cell_reservations,
            edge_reservations,
            preferred_cells=preferred_cells,
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
            "plan": summarize_plan(follower_plan, temporal=True),
            "role": "follower",
            "preferred_side": side,
            "preferred_cells": len(preferred_cells),
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
        "lattice": {
            "planner": "temporal_lattice",
            "state_representation": "(x, y, heading, t)",
            "primitive_set": primitive_names(LATTPATH_PRIMITIVES) + ["wait"],
            "wait_primitive_enabled": True,
            "reservation_model": "cell_and_edge",
            "planning_horizon": planning_horizon,
            "expanded_states_total": sum(agent["plan"]["expanded_states"] for agent in planned_agents),
            "agents_with_wait": sum(1 for agent in planned_agents if agent["plan"]["uses_wait_primitive"]),
            "reservations": summarize_reservations(cell_reservations, edge_reservations),
        },
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
    if args.controls_file is not None:
        scenario["controls"] = parse_controls(args.controls_file)
    agents = json.loads(args.agents_file.read_text(encoding="utf-8"))["agents"]
    agents = normalize_agent_headings(scenario, agents)
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
