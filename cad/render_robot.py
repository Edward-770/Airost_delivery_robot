"""Render the CAD tessellation into a report-ready robot preview."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont

ROOT = Path(__file__).resolve().parents[1]
CANVAS = "#eef2f6"
PANEL = "#f9fbfd"
INK = "#12243b"
MUTED = "#526479"
ACCENT = "#006e73"
EDGE = "#d8e2ea"
FONT_DIR = Path("C:/Windows/Fonts")


def font(size, bold=False):
    path = FONT_DIR / ("segoeuib.ttf" if bold else "segoeui.ttf")
    return ImageFont.truetype(str(path), size)


def unit(vector):
    vector = np.array(vector, dtype=float)
    return vector / np.linalg.norm(vector)


def hex_rgb(value):
    value = value.removeprefix("#")
    return np.array(tuple(int(value[i:i + 2], 16) for i in (0, 2, 4)))


def load_mesh(path):
    parts = json.loads(Path(path).read_text(encoding="utf-8"))
    for part in parts:
        part["vertices"] = np.asarray(part["vertices"], dtype=float)
        part["faces"] = np.asarray(part["faces"], dtype=int)
    return [part for part in parts if len(part["faces"])]


def camera(parts, size, direction, up):
    n = unit(direction)
    right = unit(np.cross(up, n))
    up = np.cross(n, right)
    basis = np.column_stack((right, -up, n))
    cloud = np.concatenate([p["vertices"] for p in parts]) @ basis
    lo, hi = cloud[:, :2].min(axis=0), cloud[:, :2].max(axis=0)
    scale = min(np.array(size) * .88 / np.maximum(hi - lo, 1))
    offset = np.array(size) / 2 - (lo + hi) / 2 * scale

    def project(points):
        points = np.atleast_2d(points) @ basis
        points[:, :2] = points[:, :2] * scale + offset
        return points

    return project, n


def raster_triangle(frame, zbuffer, triangle, color):
    a, b, c = triangle
    w, h = frame.shape[1], frame.shape[0]
    x0, y0 = np.maximum(np.floor(triangle[:, :2].min(axis=0)).astype(int), 0)
    x1, y1 = np.minimum(np.ceil(triangle[:, :2].max(axis=0)).astype(int), (w - 1, h - 1))
    if x0 > x1 or y0 > y1:
        return
    ax, ay = b[:2] - a[:2]
    bx, by = c[:2] - a[:2]
    det = ax * by - ay * bx
    if abs(det) < 1e-9:
        return
    xs = np.arange(x0, x1 + 1)[None, :] + .5 - a[0]
    ys = np.arange(y0, y1 + 1)[:, None] + .5 - a[1]
    t = (xs * by - ys * bx) / det
    u = (ys * ax - xs * ay) / det
    z = a[2] + t * (b[2] - a[2]) + u * (c[2] - a[2])
    buffer = zbuffer[y0:y1 + 1, x0:x1 + 1]
    paint = (t >= -1e-7) & (u >= -1e-7) & (t + u <= 1.0000001) & (z > buffer)
    buffer[paint] = z[paint]
    frame[y0:y1 + 1, x0:x1 + 1][paint] = color


def render(parts, size, direction, up=(0, 0, 1), shadow=False):
    antialias = 1.35
    full_size = tuple(round(v * antialias) for v in size)
    project, eye = camera(parts, full_size, direction, up)
    base = Image.new("RGB", full_size, PANEL)
    if shadow:
        shadow_map = Image.new("L", full_size, 0)
        ring = [[90 * np.cos(t), -8 + 100 * np.sin(t), -.5] for t in np.linspace(0, 2 * np.pi, 80)]
        xy = [tuple(pt) for pt in project(ring)[:, :2]]
        ImageDraw.Draw(shadow_map).polygon(xy, fill=46)
        shadow_map = shadow_map.filter(ImageFilter.GaussianBlur(radius=14 * antialias))
        shade = Image.new("RGB", full_size, "#576a7d")
        base = Image.composite(shade, base, shadow_map)
    frame = np.asarray(base).copy()
    zbuffer = np.full(frame.shape[:2], -1e12, dtype=np.float32)
    light = unit((-0.25, .3, 1))
    for part in parts:
        vertices = part["vertices"]
        faces = part["faces"]
        raw = vertices[faces]
        xyz = project(vertices)[faces]
        normals = np.cross(raw[:, 1] - raw[:, 0], raw[:, 2] - raw[:, 0])
        normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-10)
        facing = normals @ eye
        normals[facing < 0] *= -1
        intensity = (.58 + .26 * (normals @ light).clip(0) + .16 * np.abs(facing))
        rgb = hex_rgb(part["color"])
        colors = (rgb * intensity[:, None] + 12 * intensity[:, None]).clip(0, 255).astype(np.uint8)
        for tri, color in zip(xyz, colors):
            raster_triangle(frame, zbuffer, tri, color)
    result = Image.fromarray(frame).resize(size, Image.Resampling.LANCZOS)

    def final_project(points):
        return project(points)[:, :2] / antialias

    return result, final_project


def text_width(d, text, typeface):
    return d.textlength(text, font=typeface)


def tag(d, xy, text):
    x, y = xy
    f = font(16, True)
    w = text_width(d, text, f) + 25
    d.rounded_rectangle((x, y, x + w, y + 34), radius=17, fill="#e4f1f2")
    d.text((x + 12, y + 16), text, font=f, fill=ACCENT, anchor="lm")


def view_head(d, x, y, number, title):
    d.text((x, y), number, font=font(16, True), fill=ACCENT, anchor="lt")
    d.text((x + 40, y - 2), title, font=font(23, True), fill=INK, anchor="lt")


def badge(d, centre, label):
    x, y = centre
    r = 15
    d.ellipse((x - r, y - r, x + r, y + r), fill=INK, outline="white", width=2)
    d.text((x, y - 1), label, font=font(16, True), fill="white", anchor="mm")


def marker(d, projected, anchor, offset, label):
    x, y = projected
    bx, by = anchor
    px, py = x + offset[0], y + offset[1]
    d.line(((px, py), (bx, by)), fill="white", width=5)
    d.line(((px, py), (bx, by)), fill="#516983", width=2)
    d.ellipse((bx - 3, by - 3, bx + 3, by + 3), fill="white", outline="#516983")
    badge(d, (px, py), label)


def mark_world(d, project, origin, world, delta, label):
    target = project(world)[0] + np.array(origin)
    marker(d, target, target, delta, label)


def label_key(d, x, y, n, label):
    badge(d, (x + 14, y + 14), n)
    d.text((x + 37, y + 14), label, font=font(19, True), fill=INK, anchor="lm")


def build_preview(parts, target):
    full = Image.new("RGB", (1710, 1050), CANVAS)
    d = ImageDraw.Draw(full)
    d.rectangle((0, 0, 1710, 7), fill=ACCENT)
    d.text((54, 42), "AIROST   /   WEEK 3", font=font(18, True), fill=ACCENT)
    d.text((52, 70), "Dual-mode delivery robot", font=font(56, True), fill=INK)
    d.text((55, 145), "Rev B CAD & hardware layout   •   Cascaded encoder PID, ToF, RGB & Bluetooth",
           font=font(23), fill=MUTED)
    tag(d, (1430, 65), "CONCEPT  /  REV B")

    d.rounded_rectangle((48, 208, 1094, 875), radius=20, fill=PANEL, outline=EDGE, width=2)
    d.rounded_rectangle((1114, 208, 1662, 530), radius=20, fill=PANEL, outline=EDGE, width=2)
    d.rounded_rectangle((1114, 549, 1662, 875), radius=20, fill=PANEL, outline=EDGE, width=2)

    main_pos = (62, 269)
    image, main = render(parts, (1018, 544), (.77, 1.2, .82), shadow=True)
    full.paste(image, main_pos)
    view_head(d, 76, 228, "01", "Assembly (Open Deck)")
    label_key(d, 76, 822, "1", "NUCLEO-F756ZG")
    label_key(d, 310, 822, "2", "Servo pickup")
    label_key(d, 557, 822, "3", "Drive wheels")
    label_key(d, 785, 822, "4", "Distance sensors")

    top_pos = (1135, 250)
    open_parts = [p for p in parts if p.get("group") not in {"tray"}]
    image, _ = render(open_parts, (503, 240), (0, -.12, 1), (-1, 0, 0))
    full.paste(image, top_pos)
    view_head(d, 1139, 228, "02", "Top deck & universal slots")
    d.text((1140, 507), "Nucleo, TB6612FNG, ZS-040 BT & under-deck battery", font=font(17, True),
           fill=INK, anchor="lm")

    low_pos = (1135, 594)
    image, low = render(parts, (503, 227), (.40, 1.35, -.77))
    full.paste(image, low_pos)
    view_head(d, 1139, 572, "03", "Front underside")
    label_key(d, 1140, 832, "5", "IR line array")
    label_key(d, 1390, 832, "6", "Tucked colour sensor")

    # Anchor to the matching CAD group or named part rather than guessing the layout.
    def anchor(groups, names, default, coordinate="centre"):
        grouped = [p for p in parts if p.get("group", "").lower() in groups]
        named = [p for p in parts if any(key in p["name"].lower() for key in names)]
        matches = named or grouped
        if not matches:
            return default
        verts = np.concatenate([p["vertices"] for p in matches])
        lo, hi = verts.min(0), verts.max(0)
        centre = (lo + hi) / 2
        if coordinate == "top": centre[2] = hi[2]
        if coordinate == "front": centre[1] = hi[1] - 3
        if coordinate == "bottom": centre[2] = lo[2]
        return centre

    payload = anchor({"electronics"}, ("nucleo-f756zg pcb", "stm32f756zg"), [-14, -8, 62], "top")
    pickup = anchor({"pickup"}, ("claw servo",), [0, 105, 44], "top")
    wheels = [p for p in parts if "tire" in p["name"].lower() or "tyre" in p["name"].lower()
              or "wheel" in p["name"].lower()]
    if wheels:
        front_wheel = max(wheels, key=lambda p: p["vertices"][:, 0].max())["vertices"]
        wh = (front_wheel.min(0) + front_wheel.max(0)) / 2
        wh[0] = front_wheel[:, 0].max()
    else:
        wh = [88, 18, 33]
    tof = anchor({"tof"}, ("right tof optical face", "right vl53l1x"), [67, 58, 67])
    line = anchor({"line"}, ("line sensor pcb", "line_array", "ir line array", "ir optical pair", "qtr"),
                  [0, 74, 5], "bottom")
    color = anchor({"rgb", "colour"}, ("rgb sensor pcb", "colour_sensor", "color_sensor", "tcs34725"),
                   [90, 68, 6], "bottom")

    mark_world(d, main, main_pos, payload, (-73, -53), "1")
    mark_world(d, main, main_pos, pickup, (62, -48), "2")
    mark_world(d, main, main_pos, wh, (-59, 29), "3")
    mark_world(d, main, main_pos, tof, (-65, -45), "4")
    mark_world(d, low, low_pos, line, (-39, 32), "5")
    mark_world(d, low, low_pos, color, (-27, 34), "6")

    d.line((48, 912, 1662, 912), fill=EDGE, width=2)
    d.text((53, 947), "170 × 134 × 4 mm", font=font(31, True), fill=INK)
    d.text((54, 993), "Base plate   /   L × W × T", font=font(18), fill=MUTED)
    d.text((572, 947), "Differential drive", font=font(31, True), fill=INK)
    d.text((574, 993), "Two encoder wheels and rear caster", font=font(18), fill=MUTED)
    d.text((1124, 947), "Front and side ranging", font=font(31, True), fill=INK)
    d.text((1125, 993), "Three distance sensor mounting positions", font=font(18), fill=MUTED)
    Path(target).parent.mkdir(parents=True, exist_ok=True)
    full.save(target, optimize=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mesh", type=Path, default=ROOT / "work" / "robot_mesh.json")
    parser.add_argument("--output", type=Path, default=ROOT / "outputs" / "AIROST_Robot_Preview.png")
    args = parser.parse_args()
    build_preview(load_mesh(args.mesh), args.output)


if __name__ == "__main__":
    main()
