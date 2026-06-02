#!/usr/bin/env python3

import argparse
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

FULL_BBOX = {
    "west": -74.0300,
    "south": 40.6990,
    "east": -73.9000,
    "north": 40.8820,
}

MIDTOWN_BBOX = {
    "west": -74.0120,
    "south": 40.7380,
    "east": -73.9680,
    "north": 40.7730,
}

FULL_GRID_WIDTH = 180
MIDTOWN_GRID_WIDTH = 120

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

FULL_ROUTE_POINTS = {
    "start": {"lat": 40.7046, "lon": -74.0150},
    "goal": {"lat": 40.8738, "lon": -73.9264},
}

MIDTOWN_AGENT_POINTS = [
    {
        "id": "agent_1",
        "start": {"lat": 40.7395, "lon": -74.0085},
        "goal": {"lat": 40.7700, "lon": -73.9745},
    },
    {
        "id": "agent_2",
        "start": {"lat": 40.7395, "lon": -73.9735},
        "goal": {"lat": 40.7700, "lon": -74.0045},
    },
    {
        "id": "agent_3",
        "start": {"lat": 40.7465, "lon": -74.0090},
        "goal": {"lat": 40.7670, "lon": -73.9810},
    },
    {
        "id": "agent_4",
        "start": {"lat": 40.7440, "lon": -73.9750},
        "goal": {"lat": 40.7645, "lon": -74.0010},
    },
    {
        "id": "agent_5",
        "start": {"lat": 40.7410, "lon": -74.0040},
        "goal": {"lat": 40.7595, "lon": -73.9725},
    },
    {
        "id": "agent_6",
        "start": {"lat": 40.7420, "lon": -73.9785},
        "goal": {"lat": 40.7615, "lon": -73.9995},
    },
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Download and rasterize Manhattan OpenStreetMap roads.")
    parser.add_argument("--data-dir", type=Path, default=Path("data"), help="Directory for cached OSM tiles.")
    parser.add_argument("--artifacts-dir", type=Path, default=Path("artifacts"), help="Directory for generated outputs.")
    return parser.parse_args()


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
            "User-Agent": "LattPath-Manhattan-Demo/1.0 (educational simulation)",
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


def rasterize_bbox(nodes: dict, ways: dict, bbox: dict, width: int) -> tuple:
    height = infer_height(width, bbox)
    grid = [["#" for _ in range(width)] for _ in range(height)]

    for way in ways.values():
        points = []
        for node_ref in way["nodes"]:
            node = nodes.get(node_ref)
            if node is None:
                continue
            if not (bbox["west"] <= node["lon"] <= bbox["east"] and bbox["south"] <= node["lat"] <= bbox["north"]):
                continue
            points.append(project_point(node["lat"], node["lon"], bbox, width, height))

        if len(points) < 2:
            continue

        radius = ROAD_WIDTH.get(way["tags"].get("highway", ""), 1)
        for start, end in zip(points, points[1:]):
            for cell_x, cell_y in bresenham(start, end):
                draw_disk(grid, cell_x, cell_y, radius)

    return grid, height


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


def main() -> None:
    args = parse_args()

    tile_dir = args.data_dir / "osm_tiles"
    tile_paths = []
    for row, column, bbox in tile_bbox(FULL_BBOX):
        print(f"Downloading OSM tile r{row} c{column}: {bbox_to_query(bbox)}", flush=True)
        tile_paths.extend(download_bbox_recursive(bbox, tile_dir, f"manhattan_r{row}_c{column}"))

    nodes, ways = load_osm(tile_paths)

    full_grid, _ = rasterize_bbox(nodes, ways, FULL_BBOX, FULL_GRID_WIDTH)
    full_start = point_to_grid(FULL_ROUTE_POINTS["start"], FULL_BBOX, full_grid)
    full_goal = point_to_grid(FULL_ROUTE_POINTS["goal"], FULL_BBOX, full_grid)
    write_scenario(args.artifacts_dir / "manhattan_osm_grid.txt", "manhattan_osm", full_grid, full_start, full_goal)

    midtown_grid, crop_info = crop_grid(full_grid, FULL_BBOX, MIDTOWN_BBOX)
    midtown_start = point_to_grid({"lat": 40.7395, "lon": -74.0085}, MIDTOWN_BBOX, midtown_grid)
    midtown_goal = point_to_grid({"lat": 40.7700, "lon": -73.9745}, MIDTOWN_BBOX, midtown_grid)
    write_scenario(args.artifacts_dir / "manhattan_midtown_osm_grid.txt", "manhattan_midtown_osm", midtown_grid, midtown_start, midtown_goal)

    agents = []
    for entry in MIDTOWN_AGENT_POINTS:
        start = point_to_grid(entry["start"], MIDTOWN_BBOX, midtown_grid)
        goal = point_to_grid(entry["goal"], MIDTOWN_BBOX, midtown_grid)
        agents.append(
            {
                "id": entry["id"],
                "start": {"x": start[0], "y": start[1], "heading": 0},
                "goal": {"x": goal[0], "y": goal[1], "heading": 0},
            }
        )

    write_agents(args.artifacts_dir / "manhattan_midtown_agents.json", agents)
    metadata = {
        "full_bbox": FULL_BBOX,
        "midtown_bbox": MIDTOWN_BBOX,
        "tile_count": len(tile_paths),
        "node_count": len(nodes),
        "way_count": len(ways),
        "crop": crop_info,
    }
    (args.artifacts_dir / "manhattan_osm_metadata.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
