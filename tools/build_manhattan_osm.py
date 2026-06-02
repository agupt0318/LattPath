#!/usr/bin/env python3

import argparse
from collections import Counter, defaultdict
import json
import math
import re
import ssl
import time
import urllib.error
import urllib.parse
import urllib.request
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Optional

DEFAULT_CITY_CONFIG = Path("configs/cities/manhattan.json")

EXCLUDED_HIGHWAYS = {
    "bridleway",
    "construction",
    "corridor",
    "cycleway",
    "footway",
    "path",
    "pedestrian",
    "proposed",
    "steps",
    "track",
}

ROAD_WIDTH = {
    "motorway": 3,
    "motorway_link": 2,
    "trunk": 3,
    "trunk_link": 2,
    "primary": 3,
    "primary_link": 2,
    "secondary": 2,
    "secondary_link": 2,
    "tertiary": 2,
    "tertiary_link": 2,
    "residential": 1,
    "service": 1,
    "unclassified": 1,
    "living_street": 1,
}

DEFAULT_SPEED_KPH = {
    "motorway": 88.5,
    "motorway_link": 64.0,
    "trunk": 72.0,
    "trunk_link": 56.0,
    "primary": 48.0,
    "primary_link": 40.0,
    "secondary": 40.0,
    "secondary_link": 32.0,
    "tertiary": 32.0,
    "tertiary_link": 28.0,
    "residential": 24.0,
    "service": 16.0,
    "unclassified": 24.0,
    "living_street": 12.0,
}

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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Download and rasterize a city OpenStreetMap driving network.")
    parser.add_argument(
        "--city-config",
        type=Path,
        default=DEFAULT_CITY_CONFIG,
        help="City configuration JSON describing the region, route, and district agents.",
    )
    parser.add_argument("--data-dir", type=Path, default=Path("data"), help="Directory for cached OSM tiles.")
    parser.add_argument("--artifacts-dir", type=Path, default=Path("artifacts"), help="Directory for generated outputs.")
    parser.add_argument("--pbf-file", type=Path, help="Optional local .osm.pbf extract to use instead of live tile downloads.")
    return parser.parse_args()


def load_city_config(path: Path) -> dict:
    config = json.loads(path.read_text(encoding="utf-8"))

    required_top_level = ("city_id", "display_name", "full_bbox", "full_grid_width", "route", "district")
    for key in required_top_level:
        if key not in config:
            raise ValueError(f"City config {path} is missing required key: {key}")

    route = config["route"]
    district = config["district"]
    for key in ("scenario_name", "start", "goal"):
        if key not in route:
            raise ValueError(f"City config {path} route is missing required key: {key}")
    for key in ("name", "scenario_name", "bbox", "agents"):
        if key not in district:
            raise ValueError(f"City config {path} district is missing required key: {key}")

    return config


def meters_per_degree_lon(latitude_degrees: float) -> float:
    latitude_radians = math.radians(latitude_degrees)
    return 111320.0 * math.cos(latitude_radians)


def estimate_grid_resolution_meters(bbox: dict, width: int, height: int) -> dict:
    mid_lat = (bbox["south"] + bbox["north"]) / 2.0
    width_m = (bbox["east"] - bbox["west"]) * meters_per_degree_lon(mid_lat)
    height_m = (bbox["north"] - bbox["south"]) * 111132.0
    return {
        "meters_per_cell_x": width_m / max(width - 1, 1),
        "meters_per_cell_y": height_m / max(height - 1, 1),
    }


def parse_maxspeed_kph(raw_value: object) -> Optional[float]:
    if raw_value is None:
        return None
    if isinstance(raw_value, list):
        candidates = [parse_maxspeed_kph(value) for value in raw_value]
        candidates = [value for value in candidates if value is not None]
        if candidates:
            return sum(candidates) / len(candidates)
        return None

    text = str(raw_value).strip().lower()
    if not text:
        return None

    pieces = re.split(r"[;|,]", text)
    values = []
    for piece in pieces:
        match = re.search(r"(\d+(?:\.\d+)?)", piece)
        if not match:
            continue
        value = float(match.group(1))
        if "mph" in piece:
            value *= 1.60934
        values.append(value)

    if values:
        return sum(values) / len(values)
    return None


def infer_speed_kph(highway: str, maxspeed: object = None) -> float:
    parsed_maxspeed = parse_maxspeed_kph(maxspeed)
    if parsed_maxspeed is not None:
        return parsed_maxspeed
    return DEFAULT_SPEED_KPH.get(highway, DEFAULT_SPEED_KPH["residential"])


def bbox_to_query(bbox: dict) -> str:
    return f"{bbox['west']:.6f},{bbox['south']:.6f},{bbox['east']:.6f},{bbox['north']:.6f}"


def osm_url(bbox: dict) -> str:
    return f"https://api.openstreetmap.org/api/0.6/map?bbox={bbox_to_query(bbox)}"


def split_bbox(bbox: dict) -> list:
    lon_span = bbox["east"] - bbox["west"]
    lat_span = bbox["north"] - bbox["south"]

    if lat_span >= lon_span:
        middle = (bbox["south"] + bbox["north"]) / 2.0
        return [
            {"west": bbox["west"], "south": bbox["south"], "east": bbox["east"], "north": middle},
            {"west": bbox["west"], "south": middle, "east": bbox["east"], "north": bbox["north"]},
        ]

    middle = (bbox["west"] + bbox["east"]) / 2.0
    return [
        {"west": bbox["west"], "south": bbox["south"], "east": middle, "north": bbox["north"]},
        {"west": middle, "south": bbox["south"], "east": bbox["east"], "north": bbox["north"]},
    ]


def tile_bbox(bbox: dict, max_lon_span: float = 0.03, max_lat_span: float = 0.03) -> list:
    lon_spans = max(1, math.ceil((bbox["east"] - bbox["west"]) / max_lon_span))
    lat_spans = max(1, math.ceil((bbox["north"] - bbox["south"]) / max_lat_span))
    tiles = []

    for row in range(lat_spans):
        south = bbox["south"] + ((bbox["north"] - bbox["south"]) * row / lat_spans)
        north = bbox["south"] + ((bbox["north"] - bbox["south"]) * (row + 1) / lat_spans)
        for column in range(lon_spans):
            west = bbox["west"] + ((bbox["east"] - bbox["west"]) * column / lon_spans)
            east = bbox["west"] + ((bbox["east"] - bbox["west"]) * (column + 1) / lon_spans)
            tiles.append((row, column, {"west": west, "south": south, "east": east, "north": north}))

    return tiles


def download_bbox_recursive(bbox: dict, target_dir: Path, prefix: str, depth: int = 0) -> list:
    target_dir.mkdir(parents=True, exist_ok=True)
    output_path = target_dir / f"{prefix}.osm"
    if output_path.exists():
        return [output_path]

    request = urllib.request.Request(
        osm_url(bbox),
        headers={
            "User-Agent": "LattPath-City-Demo/1.0 (educational simulation)",
            "Accept": "application/xml,text/xml;q=0.9,*/*;q=0.8",
        },
    )

    try:
        # Public OSM extracts are read-only inputs; using an unverified context avoids
        # local certificate chain issues on this machine without changing the data itself.
        with urllib.request.urlopen(request, timeout=120, context=ssl._create_unverified_context()) as response:
            output_path.write_bytes(response.read())
        time.sleep(0.3)
        return [output_path]
    except urllib.error.HTTPError as error:
        body = error.read().decode("utf-8", errors="replace")
        if error.code == 509:
            match = re.search(r"(\d+)\s+seconds", body)
            wait_seconds = int(match.group(1)) if match else 10
            time.sleep(wait_seconds + 1)
            return download_bbox_recursive(bbox, target_dir, prefix, depth)
        if error.code == 400 and "too many nodes" in body.lower() and depth < 12:
            tile_paths = []
            for child_index, child_bbox in enumerate(split_bbox(bbox)):
                tile_paths.extend(download_bbox_recursive(child_bbox, target_dir, f"{prefix}_{child_index}", depth + 1))
            return tile_paths
        raise RuntimeError(f"Failed to download bbox {bbox_to_query(bbox)}: HTTP {error.code} {body}") from error


def load_osm(tile_paths: list) -> tuple:
    nodes = {}
    ways = {}

    for tile_path in tile_paths:
        root = ET.parse(tile_path).getroot()

        for element in root:
            if element.tag == "node":
                nodes[element.attrib["id"]] = {
                    "lat": float(element.attrib["lat"]),
                    "lon": float(element.attrib["lon"]),
                }
            elif element.tag == "way":
                node_refs = [child.attrib["ref"] for child in element if child.tag == "nd"]
                tags = {child.attrib["k"]: child.attrib["v"] for child in element if child.tag == "tag"}
                if not node_refs or "highway" not in tags or tags["highway"] in EXCLUDED_HIGHWAYS:
                    continue

                existing = ways.get(element.attrib["id"])
                if existing is None or len(node_refs) > len(existing["nodes"]):
                    ways[element.attrib["id"]] = {"nodes": node_refs, "tags": tags}

    return nodes, ways


def way_is_oneway(tags: dict) -> tuple:
    value = str(tags.get("oneway", "")).strip().lower()
    if value in {"yes", "true", "1"}:
        return True, False
    if value == "-1":
        return True, True
    if tags.get("junction", "").lower() == "roundabout":
        return True, False
    return False, False


def extract_polylines_from_osm(nodes: dict, ways: dict) -> list:
    polylines = []

    for way in ways.values():
        coords = []
        for node_ref in way["nodes"]:
            node = nodes.get(node_ref)
            if node is not None:
                coords.append((node["lon"], node["lat"]))

        if len(coords) < 2:
            continue

        is_oneway, reverse = way_is_oneway(way["tags"])
        if reverse:
            coords = list(reversed(coords))

        highway = way["tags"].get("highway", "residential")

        polylines.append({
            "coords": coords,
            "highway": highway,
            "bidirectional": not is_oneway,
            "speed_kph": infer_speed_kph(highway, way["tags"].get("maxspeed")),
        })

    return polylines


def load_driving_polylines_from_pbf(pbf_path: Path, bbox: dict) -> tuple:
    try:
        import shapely  # noqa: F401
        from pyrosm import OSM
    except ImportError as error:
        raise RuntimeError(
            "PBF support requires pyrosm and shapely to be importable. "
            "Set PYTHONPATH to the environment where those packages are installed."
        ) from error

    osm = OSM(
        str(pbf_path),
        bounding_box=[bbox["west"], bbox["south"], bbox["east"], bbox["north"]],
    )
    edges = osm.get_network(network_type="driving")

    polylines = []
    for _, edge in edges.iterrows():
        highway = edge.get("highway")
        if isinstance(highway, list):
            highway = highway[0] if highway else "residential"
        geometry = edge.geometry
        if geometry is None:
            continue

        if geometry.geom_type == "LineString":
            geometries = [geometry]
        elif geometry.geom_type == "MultiLineString":
            geometries = list(geometry.geoms)
        else:
            continue

        for line in geometries:
            coords = [(lon, lat) for lon, lat in line.coords]
            if len(coords) >= 2:
                polylines.append({
                    "coords": coords,
                    "highway": highway or "residential",
                    "bidirectional": False,
                    "speed_kph": infer_speed_kph(highway or "residential", edge.get("maxspeed")),
                })

    return polylines, len(edges)


def infer_height(width: int, bbox: dict) -> int:
    lat_span = bbox["north"] - bbox["south"]
    lon_span = bbox["east"] - bbox["west"]
    mid_lat = math.radians((bbox["south"] + bbox["north"]) / 2.0)
    aspect = lat_span / max(lon_span * math.cos(mid_lat), 1e-9)
    return max(32, int(round(width * aspect)))


def project_point(lat: float, lon: float, bbox: dict, width: int, height: int) -> tuple:
    x = (lon - bbox["west"]) / (bbox["east"] - bbox["west"])
    y = (lat - bbox["south"]) / (bbox["north"] - bbox["south"])
    grid_x = max(0, min(width - 1, int(round(x * (width - 1)))))
    grid_y = max(0, min(height - 1, int(round(y * (height - 1)))))
    return grid_x, grid_y


def bresenham(start: tuple, end: tuple) -> list:
    x0, y0 = start
    x1, y1 = end
    cells = []
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    error = dx + dy

    while True:
        cells.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        twice_error = 2 * error
        if twice_error >= dy:
            error += dy
            x0 += sx
        if twice_error <= dx:
            error += dx
            y0 += sy

    return cells


def draw_disk(grid: list, cell_x: int, cell_y: int, radius: int) -> None:
    height = len(grid)
    width = len(grid[0])
    for offset_y in range(-radius, radius + 1):
        for offset_x in range(-radius, radius + 1):
            if (offset_x * offset_x) + (offset_y * offset_y) > radius * radius:
                continue
            x = cell_x + offset_x
            y = cell_y + offset_y
            if 0 <= x < width and 0 <= y < height:
                grid[y][x] = "."


def heading_from_segment(start: tuple, end: tuple) -> Optional[int]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    if dx == 0 and dy == 0:
        return None

    unit = (max(-1, min(1, dx)), max(-1, min(1, dy)))
    for index, vector in enumerate(HEADING_VECTORS):
        if vector == unit:
            return index
    return None


def new_cell_stats() -> dict:
    return {"speed_sum": 0.0, "samples": 0, "highways": Counter()}


def paint_heading_disk(
    grid: list,
    heading_counts: dict,
    cell_stats: dict,
    cell_x: int,
    cell_y: int,
    radius: int,
    headings: list,
    speed_kph: float,
    highway: str,
) -> None:
    height = len(grid)
    width = len(grid[0])
    for offset_y in range(-radius, radius + 1):
        for offset_x in range(-radius, radius + 1):
            if (offset_x * offset_x) + (offset_y * offset_y) > radius * radius:
                continue
            x = cell_x + offset_x
            y = cell_y + offset_y
            if 0 <= x < width and 0 <= y < height:
                grid[y][x] = "."
                for heading in headings:
                    heading_counts[(x, y)][heading] += 1
                stats = cell_stats[(x, y)]
                stats["speed_sum"] += speed_kph
                stats["samples"] += 1
                stats["highways"][highway] += 1


def rasterize_polylines(polylines: list, bbox: dict, width: int) -> tuple:
    height = infer_height(width, bbox)
    grid = [["#" for _ in range(width)] for _ in range(height)]
    heading_counts = defaultdict(Counter)
    cell_stats = defaultdict(new_cell_stats)

    for polyline in polylines:
        projected = [
            project_point(lat, lon, bbox, width, height)
            for lon, lat in polyline["coords"]
            if bbox["west"] <= lon <= bbox["east"] and bbox["south"] <= lat <= bbox["north"]
        ]

        if len(projected) < 2:
            continue

        radius = ROAD_WIDTH.get(polyline["highway"], 1)
        for start, end in zip(projected, projected[1:]):
            heading = heading_from_segment(start, end)
            if heading is None:
                continue
            headings = [heading]
            if polyline.get("bidirectional", False):
                headings.append((heading + 4) % len(HEADING_VECTORS))
            for cell_x, cell_y in bresenham(start, end):
                paint_heading_disk(
                    grid,
                    heading_counts,
                    cell_stats,
                    cell_x,
                    cell_y,
                    radius,
                    headings,
                    polyline.get("speed_kph", DEFAULT_SPEED_KPH["residential"]),
                    polyline.get("highway", "residential"),
                )

    return grid, height, heading_counts, cell_stats


def rasterize_bbox(nodes: dict, ways: dict, bbox: dict, width: int) -> tuple:
    return rasterize_polylines(extract_polylines_from_osm(nodes, ways), bbox, width)


def nearest_free_cell(grid: list, target_x: int, target_y: int) -> tuple:
    height = len(grid)
    width = len(grid[0])
    if grid[target_y][target_x] == ".":
        return target_x, target_y

    max_radius = max(width, height)
    for radius in range(1, max_radius):
        for offset_y in range(-radius, radius + 1):
            for offset_x in range(-radius, radius + 1):
                if abs(offset_x) != radius and abs(offset_y) != radius:
                    continue
                x = target_x + offset_x
                y = target_y + offset_y
                if 0 <= x < width and 0 <= y < height and grid[y][x] == ".":
                    return x, y

    raise RuntimeError("Could not find a free road cell in the rasterized grid.")


def crop_grid(grid: list, full_bbox: dict, crop_bbox: dict) -> tuple:
    full_height = len(grid)
    full_width = len(grid[0])
    left, bottom = project_point(crop_bbox["south"], crop_bbox["west"], full_bbox, full_width, full_height)
    right, top = project_point(crop_bbox["north"], crop_bbox["east"], full_bbox, full_width, full_height)

    min_x = max(0, min(left, right))
    max_x = min(full_width - 1, max(left, right))
    min_y = max(0, min(bottom, top))
    max_y = min(full_height - 1, max(bottom, top))

    cropped = [row[min_x : max_x + 1] for row in grid[min_y : max_y + 1]]
    return cropped, {"min_x": min_x, "min_y": min_y, "max_x": max_x, "max_y": max_y}


def point_to_grid(point: dict, bbox: dict, grid: list) -> tuple:
    width = len(grid[0])
    height = len(grid)
    x, y = project_point(point["lat"], point["lon"], bbox, width, height)
    return nearest_free_cell(grid, x, y)


def dominant_heading(heading_counts: dict, cell: tuple, fallback: int = 0) -> int:
    counts = heading_counts.get(cell)
    if not counts:
        return fallback
    return counts.most_common(1)[0][0]


def heading_distance(left: int, right: int) -> int:
    raw = abs(left - right)
    return min(raw, len(HEADING_VECTORS) - raw)


def desired_heading(start: tuple, goal: tuple) -> int:
    dx = goal[0] - start[0]
    dy = goal[1] - start[1]
    if dx == 0 and dy == 0:
        return 0
    unit = (max(-1, min(1, dx)), max(-1, min(1, dy)))
    for index, vector in enumerate(HEADING_VECTORS):
        if vector == unit:
            return index
    return 0


def choose_agent_heading(heading_counts: dict, cell: tuple, target: tuple, fallback: int = 0) -> int:
    counts = heading_counts.get(cell)
    if not counts:
        return fallback
    desired = desired_heading(cell, target)
    return min(counts.keys(), key=lambda heading: heading_distance(heading, desired))


def crop_heading_counts(heading_counts: dict, crop: dict) -> dict:
    cropped = defaultdict(Counter)
    for (x, y), counts in heading_counts.items():
        if crop["min_x"] <= x <= crop["max_x"] and crop["min_y"] <= y <= crop["max_y"]:
            cropped[(x - crop["min_x"], y - crop["min_y"])].update(counts)
    return cropped


def crop_cell_stats(cell_stats: dict, crop: dict) -> dict:
    cropped = defaultdict(new_cell_stats)
    for (x, y), stats in cell_stats.items():
        if crop["min_x"] <= x <= crop["max_x"] and crop["min_y"] <= y <= crop["max_y"]:
            target = cropped[(x - crop["min_x"], y - crop["min_y"])]
            target["speed_sum"] += stats["speed_sum"]
            target["samples"] += stats["samples"]
            target["highways"].update(stats["highways"])
    return cropped


def serialize_network_cells(heading_counts: dict, cell_stats: dict) -> list:
    cells = []
    for (x, y), counts in sorted(heading_counts.items(), key=lambda item: (item[0][1], item[0][0])):
        allowed = [heading for heading, _ in counts.most_common()]
        stats = cell_stats.get((x, y), new_cell_stats())
        samples = max(int(stats["samples"]), 1)
        dominant_highway = "residential"
        if stats["highways"]:
            dominant_highway = stats["highways"].most_common(1)[0][0]
        cells.append({
            "x": x,
            "y": y,
            "allowed_headings": allowed,
            "dominant_heading": allowed[0],
            "sample_count": int(sum(counts.values())),
            "free_flow_kph": round(stats["speed_sum"] / samples, 3),
            "dominant_highway": dominant_highway,
        })
    return cells


def write_scenario(path: Path, name: str, grid: list, start: tuple, goal: tuple) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    width = len(grid[0])
    height = len(grid)
    lines = [
        f"name {name}",
        f"width {width}",
        f"height {height}",
        f"start {start[0]} {start[1]} 0",
        f"goal {goal[0]} {goal[1]} 0",
        "grid",
    ]
    for row in reversed(grid):
        lines.append("".join(row))
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_agents(path: Path, agents: list) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({"agents": agents}, indent=2) + "\n", encoding="utf-8")


def write_network(
    path: Path,
    scenario_name: str,
    city: dict,
    bbox: dict,
    width: int,
    height: int,
    heading_counts: dict,
    cell_stats: dict,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    resolution = estimate_grid_resolution_meters(bbox, width, height)
    payload = {
        "schema_version": 2,
        "city_id": city["city_id"],
        "display_name": city["display_name"],
        "scenario": scenario_name,
        "bbox": bbox,
        "width": width,
        "height": height,
        **resolution,
        "cells": serialize_network_cells(heading_counts, cell_stats),
    }
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def main() -> None:
    args = parse_args()
    city = load_city_config(args.city_config)
    city_id = city["city_id"]
    full_bbox = city["full_bbox"]
    full_grid_width = int(city["full_grid_width"])
    route = city["route"]
    district = city["district"]
    district_bbox = district["bbox"]

    metadata = {
        "city_id": city_id,
        "display_name": city["display_name"],
        "city_config": str(args.city_config),
        "full_bbox": full_bbox,
        "district_bbox": district_bbox,
        "route_scenario": route["scenario_name"],
        "district_scenario": district["scenario_name"],
    }

    if args.pbf_file is not None:
        print(f"Loading {city['display_name']} driving network from {args.pbf_file}", flush=True)
        polylines, edge_count = load_driving_polylines_from_pbf(args.pbf_file, full_bbox)
        full_grid, _, full_heading_counts, full_cell_stats = rasterize_polylines(polylines, full_bbox, full_grid_width)
        metadata.update({
            "source": "pbf",
            "pbf_file": str(args.pbf_file),
            "edge_count": edge_count,
            "polyline_count": len(polylines),
        })
    else:
        tile_dir = args.data_dir / "osm_tiles"
        tile_paths = []
        for row, column, bbox in tile_bbox(full_bbox):
            print(f"Downloading OSM tile r{row} c{column}: {bbox_to_query(bbox)}", flush=True)
            tile_paths.extend(download_bbox_recursive(bbox, tile_dir, f"{city_id}_r{row}_c{column}"))

        nodes, ways = load_osm(tile_paths)
        full_grid, _, full_heading_counts, full_cell_stats = rasterize_bbox(nodes, ways, full_bbox, full_grid_width)
        metadata.update({
            "source": "live_tiles",
            "tile_count": len(tile_paths),
            "node_count": len(nodes),
            "way_count": len(ways),
        })

    full_start = point_to_grid(route["start"], full_bbox, full_grid)
    full_goal = point_to_grid(route["goal"], full_bbox, full_grid)
    write_scenario(
        args.artifacts_dir / f"{route['scenario_name']}_grid.txt",
        route["scenario_name"],
        full_grid,
        full_start,
        full_goal,
    )
    write_network(
        args.artifacts_dir / f"{route['scenario_name']}_network.json",
        route["scenario_name"],
        city,
        full_bbox,
        len(full_grid[0]),
        len(full_grid),
        full_heading_counts,
        full_cell_stats,
    )

    district_grid, crop_info = crop_grid(full_grid, full_bbox, district_bbox)
    district_heading_counts = crop_heading_counts(full_heading_counts, crop_info)
    district_cell_stats = crop_cell_stats(full_cell_stats, crop_info)
    district_start = point_to_grid(district["agents"][0]["start"], district_bbox, district_grid)
    district_goal = point_to_grid(district["agents"][0]["goal"], district_bbox, district_grid)
    write_scenario(
        args.artifacts_dir / f"{district['scenario_name']}_grid.txt",
        district["scenario_name"],
        district_grid,
        district_start,
        district_goal,
    )
    write_network(
        args.artifacts_dir / f"{district['scenario_name']}_network.json",
        district["scenario_name"],
        city,
        district_bbox,
        len(district_grid[0]),
        len(district_grid),
        district_heading_counts,
        district_cell_stats,
    )

    agents = []
    for entry in district["agents"]:
        start = point_to_grid(entry["start"], district_bbox, district_grid)
        goal = point_to_grid(entry["goal"], district_bbox, district_grid)
        start_heading = choose_agent_heading(district_heading_counts, start, goal, fallback=0)
        goal_dx, goal_dy = HEADING_VECTORS[start_heading]
        goal_heading = choose_agent_heading(
            district_heading_counts,
            goal,
            (goal[0] + goal_dx, goal[1] + goal_dy),
            fallback=start_heading,
        )
        agents.append(
            {
                "id": entry["id"],
                "start": {"x": start[0], "y": start[1], "heading": start_heading},
                "goal": {"x": goal[0], "y": goal[1], "heading": goal_heading},
            }
        )

    write_agents(args.artifacts_dir / f"{district['scenario_name']}_agents.json", agents)
    metadata["crop"] = crop_info
    (args.artifacts_dir / f"{route['scenario_name']}_metadata.json").write_text(
        json.dumps(metadata, indent=2) + "\n",
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
