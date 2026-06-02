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
    "ink": (31, 41, 55),
    "muted": (107, 114, 128),
    "success": (22, 163, 74),
    "warning": (194, 65, 12),
    "link": (180, 83, 9),
}

AGENT_COLORS = (
    (14, 116, 144),
    (5, 150, 105),
    (217, 119, 6),
    (220, 38, 38),
    (109, 40, 217),
    (30, 64, 175),
)

FONT = {
    " ": ["00000", "00000", "00000", "00000", "00000", "00000", "00000"],
    "*": ["00000", "01010", "00100", "11111", "00100", "01010", "00000"],
    "-": ["00000", "00000", "00000", "11111", "00000", "00000", "00000"],
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
    "Y": ["10001", "10001", "01010", "00100", "00100", "00100", "00100"],
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
    parser = argparse.ArgumentParser(description="Render a Manhattan OSM coordination race between A* and LattPath.")
    parser.add_argument("independent", type=Path, help="Independent A* simulation JSON.")
    parser.add_argument("cooperative", type=Path, help="Cooperative LattPath simulation JSON.")
    parser.add_argument("--video-output", type=Path, required=True, help="Animated output path (.gif or .mp4).")
    parser.add_argument("--fps", type=int, default=8, help="Frames per second for the output.")
    return parser.parse_args()


def load_simulation(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def panel_layout(panel_x: int, panel_y: int, panel_width: int, panel_height: int, grid: dict) -> dict:
    cell = max(3, int(min((panel_width - 40) / grid["width"], (panel_height - 130) / grid["height"])))
    grid_width = grid["width"] * cell
    grid_height = grid["height"] * cell
    left = panel_x + (panel_width - grid_width) // 2
    top = panel_y + 92 + (panel_height - 118 - grid_height) // 2
    return {
        "cell": cell,
        "left": left,
        "top": top,
        "grid_width": grid_width,
        "grid_height": grid_height,
        "height": grid["height"],
    }


def grid_top_left(layout: dict, x: int, y: int) -> tuple:
    px = layout["left"] + (x * layout["cell"])
    py = layout["top"] + ((layout["height"] - 1 - y) * layout["cell"])
    return px, py


def grid_center(layout: dict, x: int, y: int) -> tuple:
    px, py = grid_top_left(layout, x, y)
    half = layout["cell"] // 2
    return px + half, py + half


def color_for_agent(index: int) -> tuple:
    return AGENT_COLORS[index % len(AGENT_COLORS)]


def build_agent_lookup(simulation: dict) -> dict:
    return {agent["id"]: agent for agent in simulation["agents"]}


def state_by_agent(frame: list) -> dict:
    return {entry["id"]: entry for entry in frame}


def draw_grid(canvas: Canvas, layout: dict, grid: dict) -> None:
    grid_left = layout["left"]
    grid_top = layout["top"]
    grid_right = grid_left + layout["grid_width"]
    grid_bottom = grid_top + layout["grid_height"]

    for column in range(grid["width"] + 1):
        x = grid_left + (column * layout["cell"])
        canvas.draw_line(x, grid_top, x, grid_bottom, PALETTE["grid"], thickness=1)
    for row in range(grid["height"] + 1):
        y = grid_top + (row * layout["cell"])
        canvas.draw_line(grid_left, y, grid_right, y, PALETTE["grid"], thickness=1)

    inset = 1 if layout["cell"] <= 5 else 2
    for obstacle in grid["obstacles"]:
        x0, y0 = grid_top_left(layout, obstacle["x"], obstacle["y"])
        canvas.fill_rect(
            x0 + inset,
            y0 + inset,
            x0 + layout["cell"] - inset,
            y0 + layout["cell"] - inset,
            PALETTE["obstacle"],
        )


def draw_goal_markers(canvas: Canvas, layout: dict, agents: list) -> None:
    for index, agent in enumerate(agents):
        color = color_for_agent(index)
        start_x, start_y = grid_center(layout, agent["start"]["x"], agent["start"]["y"])
        goal_x, goal_y = grid_center(layout, agent["goal"]["x"], agent["goal"]["y"])
        canvas.draw_circle(start_x, start_y, max(1, layout["cell"] // 4), blend(color, PALETTE["background"], 0.35))
        canvas.draw_line(goal_x - 3, goal_y - 3, goal_x + 3, goal_y + 3, color, thickness=2)
        canvas.draw_line(goal_x - 3, goal_y + 3, goal_x + 3, goal_y - 3, color, thickness=2)


def draw_agent_trails(canvas: Canvas, layout: dict, agents: list, tick: int) -> None:
    for index, agent in enumerate(agents):
        color = color_for_agent(index)
        trail_color = blend(color, PALETTE["background"], 0.45)
        timeline = agent["timeline"][: min(tick + 1, len(agent["timeline"]))]
        if len(timeline) < 2:
            continue
        for previous, current in zip(timeline, timeline[1:]):
            start_x, start_y = grid_center(layout, previous["x"], previous["y"])
            end_x, end_y = grid_center(layout, current["x"], current["y"])
            canvas.draw_line(
                start_x,
                start_y,
                end_x,
                end_y,
                trail_color,
                thickness=max(2, layout["cell"] // 3),
            )


def draw_pair_links(canvas: Canvas, layout: dict, frame_lookup: dict, formation_pairs: list) -> None:
    for pair in formation_pairs:
        leader = frame_lookup.get(pair["leader"])
        follower = frame_lookup.get(pair["follower"])
        if leader is None or follower is None:
            continue
        start_x, start_y = grid_center(layout, leader["x"], leader["y"])
        end_x, end_y = grid_center(layout, follower["x"], follower["y"])
        canvas.draw_line(start_x, start_y, end_x, end_y, PALETTE["link"], thickness=2)


def draw_agents(canvas: Canvas, layout: dict, frame_lookup: dict, agents: list) -> None:
    for index, agent in enumerate(agents):
        state = frame_lookup.get(agent["id"])
        if state is None:
            continue
        color = color_for_agent(index)
        center_x, center_y = grid_center(layout, state["x"], state["y"])
        delta_x, delta_y = HEADING_VECTORS[state["heading"]]
        tip_x = int(round(center_x + (delta_x * layout["cell"] * 0.58)))
        tip_y = int(round(center_y - (delta_y * layout["cell"] * 0.58)))
        canvas.draw_circle(center_x, center_y, max(2, layout["cell"] // 3), color)
        canvas.draw_line(center_x, center_y, tip_x, tip_y, color, thickness=max(2, layout["cell"] // 5))


def draw_metrics(canvas: Canvas, panel_x: int, panel_y: int, simulation: dict, final_tick: int, status_color: tuple) -> None:
    canvas.draw_text(panel_x + 16, panel_y + 14, simulation["title"], PALETTE["ink"], scale=3)
    canvas.draw_text(panel_x + 16, panel_y + 38, simulation["subtitle"], PALETTE["muted"], scale=2)
    canvas.draw_text(panel_x + 16, panel_y + 62, f"COMPLETE {simulation['completed_agents']}/{len(simulation['agents'])}", status_color, scale=2)
    canvas.draw_text(panel_x + 260, panel_y + 62, f"TICKS {final_tick}", PALETTE["ink"], scale=2)
    canvas.draw_text(panel_x + 16, panel_y + 80, f"WAITS {simulation['wait_events']}", PALETTE["warning"], scale=2)
    canvas.draw_text(panel_x + 180, panel_y + 80, f"CONFLICTS {simulation['conflicts']}", PALETTE["warning"], scale=2)


def draw_panel(canvas: Canvas, panel_x: int, panel_y: int, panel_width: int, panel_height: int, simulation: dict, tick: int) -> None:
    canvas.fill_rect(panel_x, panel_y, panel_x + panel_width, panel_y + panel_height, PALETTE["panel"])
    canvas.draw_line(panel_x, panel_y, panel_x + panel_width, panel_y, PALETTE["panel_border"], thickness=2)
    canvas.draw_line(panel_x, panel_y, panel_x, panel_y + panel_height, PALETTE["panel_border"], thickness=2)
    canvas.draw_line(panel_x + panel_width, panel_y, panel_x + panel_width, panel_y + panel_height, PALETTE["panel_border"], thickness=2)
    canvas.draw_line(panel_x, panel_y + panel_height, panel_x + panel_width, panel_y + panel_height, PALETTE["panel_border"], thickness=2)

    final_tick = min(tick, simulation["ticks"])
    status_color = PALETTE["success"] if simulation["completed_agents"] == len(simulation["agents"]) else PALETTE["warning"]
    draw_metrics(canvas, panel_x, panel_y, simulation, final_tick, status_color)

    layout = panel_layout(panel_x, panel_y, panel_width, panel_height, simulation["grid"])
    draw_grid(canvas, layout, simulation["grid"])
    draw_goal_markers(canvas, layout, simulation["agents"])
    draw_agent_trails(canvas, layout, simulation["agents"], final_tick)

    frame = simulation["history"][min(final_tick, len(simulation["history"]) - 1)]
    frame_lookup = state_by_agent(frame)
    if simulation["mode"] == "cooperative_lattpath":
        draw_pair_links(canvas, layout, frame_lookup, simulation.get("formation_pairs", []))
    draw_agents(canvas, layout, frame_lookup, simulation["agents"])


def draw_summary(canvas: Canvas, independent: dict, cooperative: dict) -> None:
    canvas.draw_text(46, 28, "MANHATTAN OSM MIDTOWN", PALETTE["ink"], scale=4)
    canvas.draw_text(46, 60, "INDEPENDENT A* AGAINST COOPERATIVE LATTPATH", PALETTE["muted"], scale=2)

    completed_delta = cooperative["completed_agents"] - independent["completed_agents"]
    wait_delta = independent["wait_events"] - cooperative["wait_events"]
    tick_delta = independent["ticks"] - cooperative["ticks"]

    canvas.draw_text(46, 86, f"LATTPATH AGENTS FINISH {completed_delta} MORE ROUTES", PALETTE["success"], scale=2)
    canvas.draw_text(472, 86, f"AND SAVE {max(tick_delta, 0)} TICKS", PALETTE["success"], scale=2)
    canvas.draw_text(790, 86, f"WITH {max(wait_delta, 0)} FEWER WAITS", PALETTE["success"], scale=2)


def prepare_simulation(simulation: dict, title: str, subtitle: str) -> dict:
    simulation["title"] = title
    simulation["subtitle"] = subtitle
    return simulation


def render_frame(frame: int, independent: dict, cooperative: dict, hold_frames: int) -> Canvas:
    canvas = Canvas(1460, 860, PALETTE["background"])
    draw_summary(canvas, independent, cooperative)

    panel_y = 120
    panel_width = 662
    panel_height = 680
    current_tick = min(frame, max(independent["ticks"], cooperative["ticks"]))

    draw_panel(canvas, 46, panel_y, panel_width, panel_height, independent, current_tick)
    draw_panel(canvas, 752, panel_y, panel_width, panel_height, cooperative, current_tick)

    if frame >= max(independent["ticks"], cooperative["ticks"]) + hold_frames // 2:
        canvas.draw_text(820, 774, "RESERVATIONS AND LANE PAIRS REDUCE BLOCKING", PALETTE["success"], scale=2)

    return canvas


def build_video(independent: dict, cooperative: dict, output_path: Path, fps: int) -> None:
    if shutil.which("ffmpeg") is None:
        raise RuntimeError("ffmpeg is required for Manhattan race video output.")

    independent = prepare_simulation(independent, "INDEPENDENT A*", "NO SHARED INTENT")
    cooperative = prepare_simulation(cooperative, "COOPERATIVE LATTPATH", "RESERVATIONS AND PAIR LANES")

    max_ticks = max(independent["ticks"], cooperative["ticks"])
    hold_frames = 18
    total_frames = max_ticks + hold_frames + 1

    with tempfile.TemporaryDirectory() as tmp_dir:
        frame_dir = Path(tmp_dir)
        for frame in range(total_frames):
            canvas = render_frame(frame, independent, cooperative, hold_frames)
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
    independent = load_simulation(args.independent)
    cooperative = load_simulation(args.cooperative)
    build_video(independent, cooperative, args.video_output, args.fps)


if __name__ == "__main__":
    main()
