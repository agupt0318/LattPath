#!/usr/bin/env python3

import argparse
import json
import shutil
import subprocess
import tempfile
from pathlib import Path

from visualize_plan import blend

PALETTE = {
    "background": (252, 251, 247),
    "panel": (255, 253, 246),
    "panel_border": (214, 202, 175),
    "grid": (233, 224, 204),
    "obstacle": (31, 41, 55),
    "expanded_start": (191, 219, 254),
    "expanded_end": (37, 99, 235),
    "start": (15, 118, 110),
    "goal": (185, 28, 28),
    "ink": (31, 41, 55),
    "muted": (107, 114, 128),
    "accent": (234, 88, 12),
    "accent_b": (8, 145, 178),
    "accent_c": (127, 29, 29),
    "success": (22, 163, 74),
}

FONT = {
    " ": ["00000", "00000", "00000", "00000", "00000", "00000", "00000"],
    "*": ["00000", "01010", "00100", "11111", "00100", "01010", "00000"],
    ".": ["00000", "00000", "00000", "00000", "00000", "01100", "01100"],
    "-": ["00000", "00000", "00000", "11111", "00000", "00000", "00000"],
    "/": ["00001", "00010", "00100", "01000", "10000", "00000", "00000"],
    ":": ["00000", "01100", "01100", "00000", "01100", "01100", "00000"],
    "0": ["01110", "10001", "10011", "10101", "11001", "10001", "01110"],
    "1": ["00100", "01100", "00100", "00100", "00100", "00100", "01110"],
    "2": ["01110", "10001", "00001", "00010", "00100", "01000", "11111"],
    "3": ["11110", "00001", "00001", "01110", "00001", "00001", "11110"],
    "4": ["00010", "00110", "01010", "10010", "11111", "00010", "00010"],
    "5": ["11111", "10000", "10000", "11110", "00001", "00001", "11110"],
    "6": ["01110", "10000", "10000", "11110", "10001", "10001", "01110"],
    "7": ["11111", "00001", "00010", "00100", "01000", "01000", "01000"],
    "8": ["01110", "10001", "10001", "01110", "10001", "10001", "01110"],
    "9": ["01110", "10001", "10001", "01111", "00001", "00001", "01110"],
    "A": ["01110", "10001", "10001", "11111", "10001", "10001", "10001"],
    "B": ["11110", "10001", "10001", "11110", "10001", "10001", "11110"],
    "C": ["01110", "10001", "10000", "10000", "10000", "10001", "01110"],
    "D": ["11110", "10001", "10001", "10001", "10001", "10001", "11110"],
    "E": ["11111", "10000", "10000", "11110", "10000", "10000", "11111"],
    "F": ["11111", "10000", "10000", "11110", "10000", "10000", "10000"],
    "G": ["01110", "10001", "10000", "10111", "10001", "10001", "01111"],
    "H": ["10001", "10001", "10001", "11111", "10001", "10001", "10001"],
    "I": ["11111", "00100", "00100", "00100", "00100", "00100", "11111"],
    "J": ["00111", "00010", "00010", "00010", "00010", "10010", "01100"],
    "K": ["10001", "10010", "10100", "11000", "10100", "10010", "10001"],
    "L": ["10000", "10000", "10000", "10000", "10000", "10000", "11111"],
    "M": ["10001", "11011", "10101", "10101", "10001", "10001", "10001"],
    "N": ["10001", "11001", "10101", "10011", "10001", "10001", "10001"],
    "O": ["01110", "10001", "10001", "10001", "10001", "10001", "01110"],
    "P": ["11110", "10001", "10001", "11110", "10000", "10000", "10000"],
    "R": ["11110", "10001", "10001", "11110", "10100", "10010", "10001"],
    "S": ["01111", "10000", "10000", "01110", "00001", "00001", "11110"],
    "T": ["11111", "00100", "00100", "00100", "00100", "00100", "00100"],
    "U": ["10001", "10001", "10001", "10001", "10001", "10001", "01110"],
    "V": ["10001", "10001", "10001", "10001", "10001", "01010", "00100"],
    "X": ["10001", "10001", "01010", "00100", "01010", "10001", "10001"],
    "Y": ["10001", "10001", "01010", "00100", "00100", "00100", "00100"],
}

ALGORITHM_STYLE = {
    "lattpath": {"title": "LATTPATH", "path": PALETTE["accent"]},
    "astar": {"title": "A*", "path": PALETTE["accent_b"]},
    "dijkstra": {"title": "DIJKSTRA", "path": PALETTE["accent_c"]},
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


class Canvas:
    def __init__(self, width: int, height: int, background: tuple):
        self.width = width
        self.height = height
        self.pixels = bytearray(background * (width * height))

    def set_pixel(self, x: int, y: int, color: tuple) -> None:
        if x < 0 or x >= self.width or y < 0 or y >= self.height:
            return
        offset = (y * self.width + x) * 3
        self.pixels[offset : offset + 3] = bytes(color)

    def fill_rect(self, x0: int, y0: int, x1: int, y1: int, color: tuple) -> None:
        left = max(0, min(x0, x1))
        right = min(self.width - 1, max(x0, x1))
        top = max(0, min(y0, y1))
        bottom = min(self.height - 1, max(y0, y1))
        for y in range(top, bottom + 1):
            start = (y * self.width + left) * 3
            end = (y * self.width + right + 1) * 3
            self.pixels[start:end] = bytes(color) * (right - left + 1)

    def draw_line(self, x0: int, y0: int, x1: int, y1: int, color: tuple, thickness: int = 1) -> None:
        delta_x = x1 - x0
        delta_y = y1 - y0
        steps = max(abs(delta_x), abs(delta_y), 1)
        radius = max(0, thickness // 2)
        for step in range(steps + 1):
            x = int(round(x0 + (delta_x * step / steps)))
            y = int(round(y0 + (delta_y * step / steps)))
            self.fill_rect(x - radius, y - radius, x + radius, y + radius, color)

    def draw_circle(self, center_x: int, center_y: int, radius: int, color: tuple) -> None:
        radius_sq = radius * radius
        for y in range(center_y - radius, center_y + radius + 1):
            for x in range(center_x - radius, center_x + radius + 1):
                if ((x - center_x) * (x - center_x)) + ((y - center_y) * (y - center_y)) <= radius_sq:
                    self.set_pixel(x, y, color)

    def draw_text(self, x: int, y: int, text: str, color: tuple, scale: int = 2) -> None:
        cursor_x = x
        for character in text.upper():
            glyph = FONT.get(character, FONT[" "])
            for row, pattern in enumerate(glyph):
                for column, bit in enumerate(pattern):
                    if bit == "1":
                        self.fill_rect(
                            cursor_x + column * scale,
                            y + row * scale,
                            cursor_x + ((column + 1) * scale) - 1,
                            y + ((row + 1) * scale) - 1,
                            color,
                        )
            cursor_x += (len(glyph[0]) + 1) * scale

    def write_ppm(self, path: Path) -> None:
        with path.open("wb") as handle:
            handle.write(f"P6\n{self.width} {self.height}\n255\n".encode("ascii"))
            handle.write(self.pixels)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Render a benchmark race video for the LattPath dense suite.")
    parser.add_argument("benchmark", type=Path, help="Path to dense suite benchmark JSON.")
    parser.add_argument("--scenario", default="dense_city", help="Scenario to animate from the benchmark JSON.")
    parser.add_argument("--video-output", type=Path, required=True, help="Animated output path (.gif or .mp4).")
    parser.add_argument("--fps", type=int, default=8, help="Frames per second for the output.")
    return parser.parse_args()


def load_benchmark(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def compute_summary(benchmark: dict) -> dict:
    summary = {}
    for scenario in benchmark["scenarios"]:
        for entry in scenario["algorithms"]:
            bucket = summary.setdefault(entry["algorithm"], {"runtime": 0.0, "expanded": 0.0, "count": 0})
            bucket["runtime"] += entry["timing"]["mean_runtime_ms"]
            bucket["expanded"] += entry["stats"]["expanded_states"]
            bucket["count"] += 1

    for value in summary.values():
        value["runtime"] /= value["count"]
        value["expanded"] /= value["count"]

    return summary


def panel_layout(panel_x: int, panel_y: int, panel_width: int, panel_height: int, plan: dict) -> dict:
    cell = max(4, int(min((panel_width - 26) / plan["grid"]["width"], (panel_height - 72) / plan["grid"]["height"])))
    grid_width = plan["grid"]["width"] * cell
    grid_height = plan["grid"]["height"] * cell
    left = panel_x + (panel_width - grid_width) // 2
    top = panel_y + 52 + (panel_height - 64 - grid_height) // 2
    return {
        "cell": cell,
        "left": left,
        "top": top,
        "grid_width": grid_width,
        "grid_height": grid_height,
        "height": plan["grid"]["height"],
    }


def grid_top_left(layout: dict, x: int, y: int) -> tuple:
    px = layout["left"] + (x * layout["cell"])
    py = layout["top"] + ((layout["height"] - 1 - y) * layout["cell"])
    return px, py


def grid_center(layout: dict, x: int, y: int) -> tuple:
    px, py = grid_top_left(layout, x, y)
    half = layout["cell"] // 2
    return px + half, py + half


def draw_panel(
    canvas: Canvas,
    panel_x: int,
    panel_y: int,
    panel_width: int,
    panel_height: int,
    entry: dict,
    summary: dict,
    expanded_count: int,
    path_count: int,
    state_index: int,
) -> None:
    style = ALGORITHM_STYLE[entry["algorithm"]]
    runtime_ms = summary[entry["algorithm"]]["runtime"]

    canvas.fill_rect(panel_x, panel_y, panel_x + panel_width, panel_y + panel_height, PALETTE["panel"])
    canvas.draw_line(panel_x, panel_y, panel_x + panel_width, panel_y, PALETTE["panel_border"], thickness=2)
    canvas.draw_line(panel_x, panel_y, panel_x, panel_y + panel_height, PALETTE["panel_border"], thickness=2)
    canvas.draw_line(panel_x + panel_width, panel_y, panel_x + panel_width, panel_y + panel_height, PALETTE["panel_border"], thickness=2)
    canvas.draw_line(panel_x, panel_y + panel_height, panel_x + panel_width, panel_y + panel_height, PALETTE["panel_border"], thickness=2)

    canvas.draw_text(panel_x + 14, panel_y + 12, style["title"], PALETTE["ink"], scale=3)
    canvas.draw_text(panel_x + 14, panel_y + 34, f"AVG {runtime_ms:.2f}MS", PALETTE["muted"], scale=2)

    layout = panel_layout(panel_x, panel_y, panel_width, panel_height, entry)
    grid_left = layout["left"]
    grid_top = layout["top"]
    grid_right = grid_left + layout["grid_width"]
    grid_bottom = grid_top + layout["grid_height"]

    for column in range(entry["grid"]["width"] + 1):
        x = grid_left + (column * layout["cell"])
        canvas.draw_line(x, grid_top, x, grid_bottom, PALETTE["grid"], thickness=1)
    for row in range(entry["grid"]["height"] + 1):
        y = grid_top + (row * layout["cell"])
        canvas.draw_line(grid_left, y, grid_right, y, PALETTE["grid"], thickness=1)

    inset = 1 if layout["cell"] <= 5 else 2
    for obstacle in entry["obstacles"]:
        x0, y0 = grid_top_left(layout, obstacle["x"], obstacle["y"])
        canvas.fill_rect(
            x0 + inset,
            y0 + inset,
            x0 + layout["cell"] - inset,
            y0 + layout["cell"] - inset,
            PALETTE["obstacle"],
        )

    total_expanded = max(len(entry["expanded"]) - 1, 1)
    radius = max(1, layout["cell"] // 6)
    for index, expanded in enumerate(entry["expanded"][:expanded_count]):
        ratio = index / total_expanded
        color = blend(PALETTE["expanded_start"], PALETTE["expanded_end"], ratio)
        cx, cy = grid_center(layout, expanded["x"], expanded["y"])
        canvas.draw_circle(cx, cy, radius, color)

    visible_cells = entry["path"]["cells"][:path_count]
    if len(visible_cells) > 1:
        for previous, current in zip(visible_cells, visible_cells[1:]):
            start_x, start_y = grid_center(layout, previous["x"], previous["y"])
            end_x, end_y = grid_center(layout, current["x"], current["y"])
            canvas.draw_line(
                start_x,
                start_y,
                end_x,
                end_y,
                style["path"],
                thickness=max(2, layout["cell"] // 3),
            )

    if entry["path"]["states"]:
        state = entry["path"]["states"][max(0, min(state_index, len(entry["path"]["states"]) - 1))]
        center_x, center_y = grid_center(layout, state["x"], state["y"])
        delta_x, delta_y = HEADING_VECTORS[state["heading"]]
        tip_x = int(round(center_x + (delta_x * layout["cell"] * 0.55)))
        tip_y = int(round(center_y - (delta_y * layout["cell"] * 0.55)))
        canvas.draw_circle(center_x, center_y, max(2, layout["cell"] // 4), style["path"])
        canvas.draw_line(center_x, center_y, tip_x, tip_y, style["path"], thickness=max(2, layout["cell"] // 5))

    start_x, start_y = grid_center(layout, entry["start"]["x"], entry["start"]["y"])
    goal_x, goal_y = grid_center(layout, entry["goal"]["x"], entry["goal"]["y"])
    canvas.draw_circle(start_x, start_y, max(2, layout["cell"] // 3), PALETTE["start"])
    canvas.draw_circle(goal_x, goal_y, max(2, layout["cell"] // 3), PALETTE["goal"])


def draw_summary(canvas: Canvas, summary: dict, benchmark_name: str) -> None:
    canvas.draw_text(40, 24, "DENSE BENCHMARK RACE", PALETTE["ink"], scale=4)
    canvas.draw_text(40, 56, "SAME MAP FAMILY SAME START AND GOAL", PALETTE["muted"], scale=2)
    canvas.draw_text(40, 76, benchmark_name.replace("_", " "), PALETTE["muted"], scale=2)

    fastest = min(summary.items(), key=lambda item: item[1]["runtime"])[0]
    speedup_vs_astar = summary["astar"]["runtime"] / summary["lattpath"]["runtime"]
    speedup_vs_dijkstra = summary["dijkstra"]["runtime"] / summary["lattpath"]["runtime"]

    cards_y = 642
    card_width = 380
    for index, algorithm in enumerate(("lattpath", "astar", "dijkstra")):
        card_x = 40 + (index * 420)
        canvas.fill_rect(card_x, cards_y, card_x + card_width, cards_y + 108, PALETTE["panel"])
        canvas.draw_line(card_x, cards_y, card_x + card_width, cards_y, PALETTE["panel_border"], thickness=2)
        canvas.draw_line(card_x, cards_y, card_x, cards_y + 108, PALETTE["panel_border"], thickness=2)
        canvas.draw_line(card_x + card_width, cards_y, card_x + card_width, cards_y + 108, PALETTE["panel_border"], thickness=2)
        canvas.draw_line(card_x, cards_y + 108, card_x + card_width, cards_y + 108, PALETTE["panel_border"], thickness=2)
        canvas.draw_text(card_x + 16, cards_y + 14, ALGORITHM_STYLE[algorithm]["title"], ALGORITHM_STYLE[algorithm]["path"], scale=3)
        canvas.draw_text(card_x + 16, cards_y + 42, f"AVG {summary[algorithm]['runtime']:.2f}MS", PALETTE["ink"], scale=2)
        canvas.draw_text(card_x + 16, cards_y + 62, f"EXPANDED {summary[algorithm]['expanded']:.0f}", PALETTE["muted"], scale=2)

        if algorithm == fastest:
            canvas.draw_text(card_x + 16, cards_y + 84, "FASTEST IN SUITE", PALETTE["success"], scale=2)

    canvas.draw_text(74, 780, f"LATTPATH {speedup_vs_astar:.1f}X FASTER THAN A*", PALETTE["success"], scale=3)
    canvas.draw_text(654, 780, f"LATTPATH {speedup_vs_dijkstra:.1f}X FASTER THAN DIJKSTRA", PALETTE["success"], scale=3)


def render_frame(frame: int, total_frames: int, scenario_entries: dict, summary: dict, benchmark_name: str) -> Canvas:
    canvas = Canvas(1320, 840, PALETTE["background"])
    draw_summary(canvas, summary, benchmark_name)

    race_frames = 40
    completion_frames = 24
    hold_frames = total_frames - race_frames - completion_frames

    fastest_runtime = min(summary[algorithm]["runtime"] for algorithm in summary)
    race_elapsed_ms = fastest_runtime * 2.2 * min(frame, race_frames - 1) / max(race_frames - 1, 1)

    if frame < race_frames:
        completion_progress = 0.0
    elif frame < race_frames + completion_frames:
        completion_progress = (frame - race_frames + 1) / completion_frames
    else:
        completion_progress = 1.0

    panel_width = 396
    panel_height = 500
    panel_y = 116
    panel_positions = [40, 462, 884]

    for panel_index, algorithm in enumerate(("lattpath", "astar", "dijkstra")):
        entry = scenario_entries[algorithm]
        runtime_ms = summary[algorithm]["runtime"]
        race_progress = min(1.0, race_elapsed_ms / max(runtime_ms, 1e-6))
        final_progress = race_progress + ((1.0 - race_progress) * completion_progress)

        path_progress = max(0.0, min(1.0, (final_progress - 0.65) / 0.35))
        expanded_count = max(1, int(round(len(entry["expanded"]) * final_progress)))
        path_count = max(1, int(round(len(entry["path"]["cells"]) * path_progress)))
        state_index = int(round((len(entry["path"]["states"]) - 1) * path_progress)) if entry["path"]["states"] else 0

        if frame >= race_frames + completion_frames + hold_frames:
            expanded_count = len(entry["expanded"])
            path_count = len(entry["path"]["cells"])
            state_index = len(entry["path"]["states"]) - 1

        draw_panel(
            canvas,
            panel_positions[panel_index],
            panel_y,
            panel_width,
            panel_height,
            entry,
            summary,
            expanded_count,
            path_count,
            state_index,
        )

    return canvas


def build_video(benchmark: dict, scenario_name: str, output_path: Path, fps: int) -> None:
    if shutil.which("ffmpeg") is None:
        raise RuntimeError("ffmpeg is required for benchmark video output.")

    selected_scenario = None
    for scenario in benchmark["scenarios"]:
        if scenario["scenario"] == scenario_name:
            selected_scenario = scenario
            break

    if selected_scenario is None:
        raise ValueError(f"Scenario {scenario_name} not found in benchmark JSON.")

    summary = compute_summary(benchmark)
    scenario_entries = {entry["algorithm"]: entry for entry in selected_scenario["algorithms"]}

    race_frames = 40
    completion_frames = 24
    hold_frames = 14
    total_frames = race_frames + completion_frames + hold_frames

    with tempfile.TemporaryDirectory() as tmp_dir:
        frame_dir = Path(tmp_dir)
        for frame in range(total_frames):
            canvas = render_frame(frame, total_frames, scenario_entries, summary, benchmark["benchmark"])
            canvas.write_ppm(frame_dir / f"frame_{frame:04d}.ppm")

        output_path.parent.mkdir(parents=True, exist_ok=True)
        input_pattern = str(frame_dir / "frame_%04d.ppm")
        common = ["ffmpeg", "-y", "-framerate", str(fps), "-start_number", "0", "-i", input_pattern]

        if output_path.suffix.lower() == ".mp4":
            command = common + ["-c:v", "libx264", "-pix_fmt", "yuv420p", str(output_path)]
        elif output_path.suffix.lower() == ".gif":
            palette_path = frame_dir / "palette.png"
            subprocess.run(
                common + ["-vf", "palettegen=stats_mode=single", "-frames:v", "1", "-update", "1", str(palette_path)],
                check=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            command = [
                "ffmpeg",
                "-y",
                "-framerate",
                str(fps),
                "-start_number",
                "0",
                "-i",
                input_pattern,
                "-i",
                str(palette_path),
                "-lavfi",
                "paletteuse=dither=bayer:bayer_scale=3",
                str(output_path),
            ]
        else:
            raise ValueError("Video output must end in .gif or .mp4")

        subprocess.run(command, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main() -> None:
    args = parse_args()
    benchmark = load_benchmark(args.benchmark)
    build_video(benchmark, args.scenario, args.video_output, args.fps)


if __name__ == "__main__":
    main()
