#!/usr/bin/env python3

import argparse
import json
import shutil
import subprocess
import tempfile
from pathlib import Path

PALETTE = {
    "background": (252, 251, 247),
    "grid": (231, 220, 196),
    "obstacle": (31, 41, 55),
    "start": (15, 118, 110),
    "goal": (185, 28, 28),
    "expanded_a": (191, 219, 254),
    "expanded_b": (30, 64, 175),
    "path": (234, 88, 12),
    "vehicle": (180, 83, 9),
    "text": (75, 85, 99),
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
    parser = argparse.ArgumentParser(description="Render planner output into an SVG, GIF, or MP4.")
    parser.add_argument("plan", type=Path, help="Path to a planner JSON output file.")
    parser.add_argument("--video-output", type=Path, help="Optional animated output (.gif or .mp4).")
    parser.add_argument("--still-output", type=Path, help="Optional static SVG output.")
    parser.add_argument("--fps", type=int, default=8, help="Frames per second for video output.")
    return parser.parse_args()


def load_plan(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def blend(start: tuple, end: tuple, ratio: float) -> tuple:
    ratio = max(0.0, min(1.0, ratio))
    return tuple(int(start[index] + ((end[index] - start[index]) * ratio)) for index in range(3))


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
            row_start = (y * self.width + left) * 3
            row_end = (y * self.width + right + 1) * 3
            self.pixels[row_start:row_end] = bytes(color) * (right - left + 1)

    def draw_line(self, x0: int, y0: int, x1: int, y1: int, color: tuple, thickness: int = 1) -> None:
        delta_x = x1 - x0
        delta_y = y1 - y0
        steps = max(abs(delta_x), abs(delta_y), 1)
        for step in range(steps + 1):
            x = int(round(x0 + (delta_x * step / steps)))
            y = int(round(y0 + (delta_y * step / steps)))
            radius = max(0, thickness // 2)
            self.fill_rect(x - radius, y - radius, x + radius, y + radius, color)

    def draw_circle(self, center_x: int, center_y: int, radius: int, color: tuple) -> None:
        radius_sq = radius * radius
        for y in range(center_y - radius, center_y + radius + 1):
            for x in range(center_x - radius, center_x + radius + 1):
                if ((x - center_x) * (x - center_x)) + ((y - center_y) * (y - center_y)) <= radius_sq:
                    self.set_pixel(x, y, color)

    def write_ppm(self, path: Path) -> None:
        with path.open("wb") as handle:
            handle.write(f"P6\n{self.width} {self.height}\n255\n".encode("ascii"))
            handle.write(self.pixels)


def layout(plan: dict) -> dict:
    max_canvas_width = 1280
    max_canvas_height = 820
    width = plan["grid"]["width"]
    height = plan["grid"]["height"]
    cell = max(2, min(30, int(min(max_canvas_width / max(width, 1), max_canvas_height / max(height, 1)))))
    margin = 28
    return {
        "cell": cell,
        "margin": margin,
        "width": width,
        "height": height,
        "image_width": (width * cell) + (margin * 2),
        "image_height": (height * cell) + (margin * 2),
    }


def grid_top_left(config: dict, x: int, y: int) -> tuple:
    px = config["margin"] + (x * config["cell"])
    py = config["margin"] + ((config["height"] - 1 - y) * config["cell"])
    return px, py


def grid_center(config: dict, x: int, y: int) -> tuple:
    px, py = grid_top_left(config, x, y)
    half = config["cell"] // 2
    return px + half, py + half


def draw_base(canvas: Canvas, plan: dict, config: dict) -> None:
    left = config["margin"]
    top = config["margin"]
    right = left + (config["width"] * config["cell"])
    bottom = top + (config["height"] * config["cell"])

    for column in range(config["width"] + 1):
        x = left + (column * config["cell"])
        canvas.draw_line(x, top, x, bottom, PALETTE["grid"], thickness=1)
    for row in range(config["height"] + 1):
        y = top + (row * config["cell"])
        canvas.draw_line(left, y, right, y, PALETTE["grid"], thickness=1)

    inset = 3
    for obstacle in plan["obstacles"]:
        x0, y0 = grid_top_left(config, obstacle["x"], obstacle["y"])
        canvas.fill_rect(
            x0 + inset,
            y0 + inset,
            x0 + config["cell"] - inset,
            y0 + config["cell"] - inset,
            PALETTE["obstacle"],
        )

    start = plan["start"]
    goal = plan["goal"]
    start_x, start_y = grid_center(config, start["x"], start["y"])
    goal_x, goal_y = grid_center(config, goal["x"], goal["y"])
    canvas.draw_circle(start_x, start_y, max(2, config["cell"] // 4), PALETTE["start"])
    canvas.draw_circle(goal_x, goal_y, max(2, config["cell"] // 4), PALETTE["goal"])


def draw_expanded(canvas: Canvas, plan: dict, config: dict, count: int) -> None:
    total = max(len(plan["expanded"]) - 1, 1)
    radius = max(2, config["cell"] // 8)
    for index, expanded in enumerate(plan["expanded"][:count]):
        ratio = index / total
        color = blend(PALETTE["expanded_a"], PALETTE["expanded_b"], ratio)
        center_x, center_y = grid_center(config, expanded["x"], expanded["y"])
        canvas.draw_circle(center_x, center_y, radius, color)


def draw_path(canvas: Canvas, plan: dict, config: dict, count: int) -> None:
    if count <= 1:
        return
    visible = plan["path"]["cells"][:count]
    for previous, current in zip(visible, visible[1:]):
        start_x, start_y = grid_center(config, previous["x"], previous["y"])
        end_x, end_y = grid_center(config, current["x"], current["y"])
        canvas.draw_line(start_x, start_y, end_x, end_y, PALETTE["path"], thickness=max(3, config["cell"] // 6))


def draw_vehicle(canvas: Canvas, state: dict, config: dict) -> None:
    center_x, center_y = grid_center(config, state["x"], state["y"])
    delta_x, delta_y = HEADING_VECTORS[state["heading"]]
    arrow_scale = config["cell"] * 0.32
    tip_x = int(round(center_x + (delta_x * arrow_scale)))
    tip_y = int(round(center_y - (delta_y * arrow_scale)))
    canvas.draw_circle(center_x, center_y, max(2, config["cell"] // 6), PALETTE["vehicle"])
    canvas.draw_line(center_x, center_y, tip_x, tip_y, PALETTE["vehicle"], thickness=max(2, config["cell"] // 10))
    wing = max(4, config["cell"] // 8)
    canvas.draw_line(tip_x, tip_y, tip_x - wing, tip_y + wing, PALETTE["vehicle"], thickness=2)
    canvas.draw_line(tip_x, tip_y, tip_x + wing, tip_y + wing, PALETTE["vehicle"], thickness=2)


def write_svg(plan: dict, output_path: Path) -> None:
    config = layout(plan)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    width = config["image_width"]
    height = config["image_height"]
    cell = config["cell"]
    margin = config["margin"]
    grid_width = config["width"] * cell
    grid_height = config["height"] * cell

    path_points = []
    for cell_entry in plan["path"]["cells"]:
        center_x, center_y = grid_center(config, cell_entry["x"], cell_entry["y"])
        path_points.append(f"{center_x},{center_y}")

    obstacle_rects = []
    for obstacle in plan["obstacles"]:
        x0, y0 = grid_top_left(config, obstacle["x"], obstacle["y"])
        obstacle_rects.append(
            f'<rect x="{x0 + 3}" y="{y0 + 3}" width="{cell - 6}" height="{cell - 6}" rx="4" fill="#1f2937" />'
        )

    expanded_circles = []
    total = max(len(plan["expanded"]) - 1, 1)
    for index, expanded in enumerate(plan["expanded"]):
        ratio = index / total
        color = "#{:02x}{:02x}{:02x}".format(*blend(PALETTE["expanded_a"], PALETTE["expanded_b"], ratio))
        center_x, center_y = grid_center(config, expanded["x"], expanded["y"])
        expanded_circles.append(
            f'<circle cx="{center_x}" cy="{center_y}" r="{max(2, cell // 8)}" fill="{color}" opacity="0.55" />'
        )

    heading_markers = []
    stride = max(1, len(plan["path"]["states"]) // 10)
    for state in plan["path"]["states"][::stride]:
        center_x, center_y = grid_center(config, state["x"], state["y"])
        delta_x, delta_y = HEADING_VECTORS[state["heading"]]
        tip_x = center_x + int(round(delta_x * cell * 0.28))
        tip_y = center_y - int(round(delta_y * cell * 0.28))
        heading_markers.append(
            f'<line x1="{center_x}" y1="{center_y}" x2="{tip_x}" y2="{tip_y}" '
            f'stroke="#b45309" stroke-width="3" stroke-linecap="round" />'
        )

    svg = f"""<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
  <rect width="100%" height="100%" fill="#fcfbf7" />
  <rect x="{margin}" y="{margin}" width="{grid_width}" height="{grid_height}" fill="#fffdf6" stroke="#d1c5ae" />
  <g stroke="#e7dcc4" stroke-width="1">
    {"".join(f'<line x1="{margin + i * cell}" y1="{margin}" x2="{margin + i * cell}" y2="{margin + grid_height}" />' for i in range(config["width"] + 1))}
    {"".join(f'<line x1="{margin}" y1="{margin + i * cell}" x2="{margin + grid_width}" y2="{margin + i * cell}" />' for i in range(config["height"] + 1))}
  </g>
  <g>{"".join(obstacle_rects)}</g>
  <g>{"".join(expanded_circles)}</g>
  <polyline fill="none" stroke="#ea580c" stroke-width="6" stroke-linecap="round" stroke-linejoin="round" points="{' '.join(path_points)}" />
  <g>{"".join(heading_markers)}</g>
  <circle cx="{grid_center(config, plan["start"]["x"], plan["start"]["y"])[0]}" cy="{grid_center(config, plan["start"]["x"], plan["start"]["y"])[1]}" r="{max(2, cell // 4)}" fill="#0f766e" />
  <circle cx="{grid_center(config, plan["goal"]["x"], plan["goal"]["y"])[0]}" cy="{grid_center(config, plan["goal"]["x"], plan["goal"]["y"])[1]}" r="{max(2, cell // 4)}" fill="#b91c1c" />
  <text x="{margin}" y="18" fill="#4b5563" font-family="Arial, sans-serif" font-size="16">LattPath demo: {plan["scenario"]}</text>
  <text x="{margin}" y="{height - 8}" fill="#4b5563" font-family="Arial, sans-serif" font-size="14">
    expanded={plan["stats"]["expanded_states"]} path_cost={plan["stats"]["path_cost"]:.1f} runtime_ms={plan["stats"]["runtime_ms"]:.2f}
  </text>
</svg>
"""
    output_path.write_text(svg, encoding="utf-8")


def render_frame(plan: dict, config: dict, expanded_count: int, path_count: int, state_index: int) -> Canvas:
    canvas = Canvas(config["image_width"], config["image_height"], PALETTE["background"])
    draw_base(canvas, plan, config)
    draw_expanded(canvas, plan, config, expanded_count)
    draw_path(canvas, plan, config, path_count)
    if plan["path"]["states"]:
        state = plan["path"]["states"][max(0, min(state_index, len(plan["path"]["states"]) - 1))]
        draw_vehicle(canvas, state, config)
    return canvas


def build_video(plan: dict, output_path: Path, fps: int) -> None:
    if shutil.which("ffmpeg") is None:
        raise RuntimeError("ffmpeg is required for video output but was not found on PATH.")

    config = layout(plan)
    expanded_total = len(plan["expanded"])
    path_total = len(plan["path"]["cells"])
    state_total = len(plan["path"]["states"])
    expansion_frames = 45
    path_frames = max(path_total, 1)
    settle_frames = 8
    total_frames = expansion_frames + path_frames + settle_frames

    with tempfile.TemporaryDirectory() as tmp_dir:
        frame_dir = Path(tmp_dir)
        for frame in range(total_frames):
            expansion_ratio = min(1.0, frame / max(expansion_frames - 1, 1))
            expanded_count = int(round(expanded_total * expansion_ratio))
            if frame < expansion_frames:
                path_ratio = 0.0
            else:
                path_ratio = min(1.0, (frame - expansion_frames + 1) / max(path_frames, 1))
            path_count = max(1, int(round(path_total * path_ratio))) if path_total else 0
            state_index = int(round((state_total - 1) * path_ratio)) if state_total else 0

            canvas = render_frame(plan, config, expanded_count, path_count, state_index)
            canvas.write_ppm(frame_dir / f"frame_{frame:04d}.ppm")

        output_path.parent.mkdir(parents=True, exist_ok=True)
        input_pattern = str(frame_dir / "frame_%04d.ppm")
        common_prefix = ["ffmpeg", "-y", "-framerate", str(fps), "-start_number", "0", "-i", input_pattern]

        if output_path.suffix.lower() == ".mp4":
            command = common_prefix + [
                "-c:v",
                "libx264",
                "-pix_fmt",
                "yuv420p",
                str(output_path),
            ]
        elif output_path.suffix.lower() == ".gif":
            palette_path = frame_dir / "palette.png"
            subprocess.run(
                common_prefix
                + [
                    "-vf",
                    "palettegen=stats_mode=single",
                    "-frames:v",
                    "1",
                    "-update",
                    "1",
                    str(palette_path),
                ],
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
            raise ValueError("Video output must use a .gif or .mp4 extension.")

        subprocess.run(command, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main() -> None:
    args = parse_args()
    plan = load_plan(args.plan)

    if not plan["stats"]["success"]:
        raise SystemExit("Planner output indicates failure; refusing to render failed search results.")

    if not args.video_output and not args.still_output:
        raise SystemExit("Provide --video-output, --still-output, or both.")

    if args.still_output:
        write_svg(plan, args.still_output)
    if args.video_output:
        build_video(plan, args.video_output, args.fps)


if __name__ == "__main__":
    main()
