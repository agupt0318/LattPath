#!/usr/bin/env python3

import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from build_manhattan_osm import (
    DEFAULT_SPEED_KPH,
    HEADING_VECTORS,
    choose_agent_heading,
    crop_cell_stats,
    crop_grid,
    crop_heading_counts,
    download_bbox_recursive,
    load_city_config,
    load_driving_polylines_from_pbf,
    load_osm,
    point_to_grid,
    rasterize_bbox,
    rasterize_polylines,
    tile_bbox,
    write_agents,
    write_network,
    write_scenario,
)

DEFAULT_CITY_CONFIG = Path("configs/cities/manhattan.json")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Build a city driving-network raster using either a custom OSM parser or OSMnx.")
    parser.add_argument(
        "--city-config",
        type=Path,
        default=DEFAULT_CITY_CONFIG,
        help="City configuration JSON describing the region, route, and district agents.",
    )
    parser.add_argument("--backend", choices=("custom", "osmnx"), default="custom", help="Network ingestion backend.")
    parser.add_argument("--output-tag", help="Optional suffix appended to scenario names and output prefixes.")
    parser.add_argument("--data-dir", type=Path, default=Path("data"), help="Directory for cached OSM tiles.")
    parser.add_argument("--artifacts-dir", type=Path, default=Path("artifacts"), help="Directory for generated outputs.")
    parser.add_argument("--pbf-file", type=Path, help="Optional local .osm.pbf extract for the custom backend.")
    return parser.parse_args()


def apply_output_tag(name: str, output_tag: str | None) -> str:
    if not output_tag:
        return name
    return f"{name}_{output_tag}"


def build_city_names(city: dict, output_tag: str | None) -> dict:
    route_name = apply_output_tag(city["route"]["scenario_name"], output_tag)
    district_name = apply_output_tag(city["district"]["scenario_name"], output_tag)
    city_prefix = apply_output_tag(city["city_id"], output_tag)
    return {
        "route_scenario": route_name,
        "district_scenario": district_name,
        "city_prefix": city_prefix,
    }


def load_custom_network(city: dict, args: argparse.Namespace) -> tuple:
    full_bbox = city["full_bbox"]
    full_grid_width = int(city["full_grid_width"])
    metadata = {}

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
            tile_paths.extend(download_bbox_recursive(bbox, tile_dir, f"{city['city_id']}_r{row}_c{column}"))

        nodes, ways = load_osm(tile_paths)
        full_grid, _, full_heading_counts, full_cell_stats = rasterize_bbox(nodes, ways, full_bbox, full_grid_width)
        metadata.update({
            "source": "live_tiles",
            "tile_count": len(tile_paths),
            "node_count": len(nodes),
            "way_count": len(ways),
        })

    return full_grid, full_heading_counts, full_cell_stats, metadata


def ensure_osmnx_modules() -> tuple:
    try:
        import networkx as nx
        import osmnx as ox
    except ImportError as error:
        raise RuntimeError(
            "The osmnx backend requires osmnx and networkx to be importable. "
            "Set PYTHONPATH to the environment where those packages are installed."
        ) from error

    return ox, nx


def write_merged_osm_xml(nodes: dict, ways: dict, output_path: Path) -> None:
    root = ET.Element("osm", attrib={"version": "0.6", "generator": "LattPath"})

    for node_id, node in sorted(nodes.items(), key=lambda item: int(item[0])):
        ET.SubElement(
            root,
            "node",
            attrib={
                "id": str(node_id),
                "lat": str(node["lat"]),
                "lon": str(node["lon"]),
            },
        )

    for way_id, way in sorted(ways.items(), key=lambda item: int(item[0])):
        way_element = ET.SubElement(root, "way", attrib={"id": str(way_id)})
        for node_ref in way["nodes"]:
            ET.SubElement(way_element, "nd", attrib={"ref": str(node_ref)})
        for key, value in way["tags"].items():
            ET.SubElement(way_element, "tag", attrib={"k": str(key), "v": str(value)})

    output_path.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(output_path, encoding="utf-8", xml_declaration=True)


def load_osmnx_tile_graphs(city: dict, args: argparse.Namespace):
    ox, _ = ensure_osmnx_modules()

    useful_tags = set(ox.settings.useful_tags_way)
    useful_tags.update({"highway", "maxspeed", "lanes", "name", "oneway"})
    ox.settings.useful_tags_way = list(useful_tags)

    tile_dir = args.data_dir / "osm_tiles"
    cached_paths = sorted(tile_dir.glob(f"{city['city_id']}*.osm"))
    tile_paths = list(cached_paths)

    if not tile_paths:
        for row, column, bbox in tile_bbox(city["full_bbox"]):
            tile_paths.extend(download_bbox_recursive(bbox, tile_dir, f"{city['city_id']}_r{row}_c{column}"))

    nodes, ways = load_osm(tile_paths)
    merged_path = tile_dir / f"{city['city_id']}_osmnx_merged.osm"
    write_merged_osm_xml(nodes, ways, merged_path)
    graph = ox.graph.graph_from_xml(merged_path, bidirectional=False, simplify=False, retain_all=True)

    graph = ox.routing.add_edge_speeds(graph, hwy_speeds=DEFAULT_SPEED_KPH, fallback=DEFAULT_SPEED_KPH["residential"])
    graph = ox.routing.add_edge_travel_times(graph)
    return graph, tile_paths, merged_path


def graph_to_polylines(graph) -> list:
    polylines = []

    for origin, destination, _, data in graph.edges(keys=True, data=True):
        start = graph.nodes[origin]
        end = graph.nodes[destination]
        geometry = data.get("geometry")
        if geometry is not None:
            coords = [(lon, lat) for lon, lat in geometry.coords]
        else:
            coords = [(start["x"], start["y"]), (end["x"], end["y"])]

        highway = data.get("highway", "residential")
        if isinstance(highway, list):
            highway = highway[0] if highway else "residential"

        speed_kph = float(data.get("speed_kph", DEFAULT_SPEED_KPH.get(highway, DEFAULT_SPEED_KPH["residential"])))
        polylines.append({
            "coords": coords,
            "highway": highway,
            "bidirectional": False,
            "speed_kph": speed_kph,
        })

    return polylines


def load_osmnx_network(city: dict, args: argparse.Namespace) -> tuple:
    if args.pbf_file is not None:
        print("Ignoring --pbf-file for the osmnx backend; using cached/downloaded XML tiles instead.", flush=True)

    graph, tile_paths, merged_path = load_osmnx_tile_graphs(city, args)
    full_grid, _, full_heading_counts, full_cell_stats = rasterize_polylines(
        graph_to_polylines(graph),
        city["full_bbox"],
        int(city["full_grid_width"]),
    )
    metadata = {
        "source": "osmnx_xml_tiles",
        "tile_count": len(tile_paths),
        "merged_xml_file": str(merged_path),
        "node_count": len(graph.nodes),
        "edge_count": len(graph.edges),
        "polyline_count": len(graph.edges),
    }
    return full_grid, full_heading_counts, full_cell_stats, metadata


def write_city_outputs(city: dict, names: dict, backend: str, args: argparse.Namespace, full_grid: list, full_heading_counts: dict, full_cell_stats: dict, source_metadata: dict) -> None:
    full_bbox = city["full_bbox"]
    district_bbox = city["district"]["bbox"]
    route = city["route"]
    district = city["district"]

    metadata = {
        "city_id": city["city_id"],
        "display_name": city["display_name"],
        "city_config": str(args.city_config),
        "backend": backend,
        "output_tag": args.output_tag,
        "full_bbox": full_bbox,
        "district_bbox": district_bbox,
        "route_scenario": names["route_scenario"],
        "district_scenario": names["district_scenario"],
        **source_metadata,
    }

    full_start = point_to_grid(route["start"], full_bbox, full_grid)
    full_goal = point_to_grid(route["goal"], full_bbox, full_grid)
    write_scenario(
        args.artifacts_dir / f"{names['route_scenario']}_grid.txt",
        names["route_scenario"],
        full_grid,
        full_start,
        full_goal,
    )
    write_network(
        args.artifacts_dir / f"{names['route_scenario']}_network.json",
        names["route_scenario"],
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
        args.artifacts_dir / f"{names['district_scenario']}_grid.txt",
        names["district_scenario"],
        district_grid,
        district_start,
        district_goal,
    )
    write_network(
        args.artifacts_dir / f"{names['district_scenario']}_network.json",
        names["district_scenario"],
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
        agents.append({
            "id": entry["id"],
            "start": {"x": start[0], "y": start[1], "heading": start_heading},
            "goal": {"x": goal[0], "y": goal[1], "heading": goal_heading},
        })

    write_agents(args.artifacts_dir / f"{names['district_scenario']}_agents.json", agents)
    metadata["crop"] = crop_info
    (args.artifacts_dir / f"{names['route_scenario']}_metadata.json").write_text(
        json.dumps(metadata, indent=2) + "\n",
        encoding="utf-8",
    )


def main() -> None:
    args = parse_args()
    city = load_city_config(args.city_config)
    names = build_city_names(city, args.output_tag)

    if args.backend == "custom":
        full_grid, full_heading_counts, full_cell_stats, source_metadata = load_custom_network(city, args)
    else:
        full_grid, full_heading_counts, full_cell_stats, source_metadata = load_osmnx_network(city, args)

    write_city_outputs(city, names, args.backend, args, full_grid, full_heading_counts, full_cell_stats, source_metadata)


if __name__ == "__main__":
    main()
