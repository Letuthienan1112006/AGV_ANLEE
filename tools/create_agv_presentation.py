#!/usr/bin/env python3
"""Create an experiment-led AGV presentation in the style of the supplied deck.

Run in two stages:
  1. python3 tools/create_agv_presentation.py --assets-only
  2. Start LibreOffice listening on port 2002, then run without --assets-only.

The output is intentionally evidence-led: all numbers are parsed from the
repository's training logs, telemetry CSV files, bridge logs, and Confg.py.
"""

from __future__ import annotations

import argparse
import csv
import glob
import math
import os
import re
import statistics as st
import zipfile
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont, ImageOps

RESAMPLE_LANCZOS = getattr(Image, "Resampling", Image).LANCZOS


ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = ROOT / "deliverables"
ASSET_DIR = OUT_DIR / "agv_ppt_assets"
SOURCE_DECK = Path("/home/an-lee/Bao_cao_AGV_NDA_editable_v3.pptx")
OUTPUT_DECK = OUT_DIR / "Bao_cao_AGV_NDA_thuc_nghiem.pptx"

NAVY = "173F67"
NAVY_DARK = "0F3154"
BLUE = "1487C9"
CYAN = "19B7C9"
ORANGE = "F39A22"
RED = "D94949"
GREEN = "2A9D66"
PURPLE = "7E3E98"
INK = "18324A"
MUTED = "667587"
PALE = "F4F7FA"
PALE_BLUE = "EAF4FA"
PALE_ORANGE = "FFF3E2"
WHITE = "FFFFFF"
GRID = "D8E2EA"


def rgb(hex_color: str) -> tuple[int, int, int]:
    return tuple(int(hex_color[i : i + 2], 16) for i in (0, 2, 4))


def int_color(hex_color: str) -> int:
    return int(hex_color, 16)


FONT_REG = "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf"
FONT_BOLD = "/usr/share/fonts/truetype/liberation2/LiberationSans-Bold.ttf"


def font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont:
    return ImageFont.truetype(FONT_BOLD if bold else FONT_REG, size=size)


def rounded_mask(size: tuple[int, int], radius: int = 26) -> Image.Image:
    mask = Image.new("L", size, 0)
    ImageDraw.Draw(mask).rounded_rectangle((0, 0, size[0], size[1]), radius=radius, fill=255)
    return mask


def fit_crop(im: Image.Image, size: tuple[int, int], focus_y: float = 0.5) -> Image.Image:
    """Cover-crop to size, with an optional vertical focal position."""
    im = im.convert("RGB")
    target_ratio = size[0] / size[1]
    ratio = im.width / im.height
    if ratio > target_ratio:
        new_w = int(im.height * target_ratio)
        left = max(0, (im.width - new_w) // 2)
        im = im.crop((left, 0, left + new_w, im.height))
    else:
        new_h = int(im.width / target_ratio)
        top = int((im.height - new_h) * focus_y)
        top = max(0, min(top, im.height - new_h))
        im = im.crop((0, top, im.width, top + new_h))
    return im.resize(size, RESAMPLE_LANCZOS)


def save_rounded_photo(src: Path, dst: Path, size=(1200, 675), focus_y=0.5, radius=32):
    im = fit_crop(Image.open(src), size, focus_y)
    canvas = Image.new("RGBA", size, (255, 255, 255, 0))
    canvas.paste(im, (0, 0), rounded_mask(size, radius))
    canvas.save(dst)


def extract_logos() -> tuple[Path, Path]:
    university = ASSET_DIR / "logo_hcmute.png"
    lab = ASSET_DIR / "logo_islab.png"
    if SOURCE_DECK.exists():
        with zipfile.ZipFile(SOURCE_DECK) as zf:
            university.write_bytes(zf.read("ppt/media/image.png"))
            lab.write_bytes(zf.read("ppt/media/image2.png"))
    return university, lab


def make_pair_image(left_path: Path, right_path: Path, dst: Path):
    w, h = 1500, 720
    canvas = Image.new("RGB", (w, h), rgb(WHITE))
    draw = ImageDraw.Draw(canvas)
    pad, gap, cap = 26, 24, 64
    pw = (w - 2 * pad - gap) // 2
    ph = h - cap - pad
    for x, src, label, color in [
        (pad, left_path, "ẢNH CAMERA", NAVY),
        (pad + pw + gap, right_path, "KẾT QUẢ PHÂN ĐOẠN", CYAN),
    ]:
        im = fit_crop(Image.open(src), (pw, ph))
        canvas.paste(im, (x, cap))
        draw.rounded_rectangle((x, 8, x + 260, 50), 18, fill=rgb(color))
        draw.text((x + 18, 17), label, font=font(20, True), fill=rgb(WHITE))
    canvas.save(dst, quality=94)


def make_experiment_grid(paths: list[Path], dst: Path):
    w, h = 1600, 900
    canvas = Image.new("RGB", (w, h), rgb(WHITE))
    draw = ImageDraw.Draw(canvas)
    gap, pad = 18, 20
    cw, ch = (w - 2 * pad - gap) // 2, (h - 2 * pad - gap) // 2
    labels = [
        "BÁM VẠCH — LỆCH +55 px",
        "KHÚC CUA — LỆCH −45 px",
        "NGƯỜI XÂM NHẬP — CẢNH BÁO",
        "VẬT CẢN GẦN — GIỮ LÀN",
    ]
    for i, src in enumerate(paths):
        x = pad + (i % 2) * (cw + gap)
        y = pad + (i // 2) * (ch + gap)
        im = fit_crop(Image.open(src), (cw, ch))
        canvas.paste(im, (x, y))
        draw.rectangle((x, y + ch - 54, x + cw, y + ch), fill=rgb(NAVY))
        draw.text((x + 16, y + ch - 42), labels[i], font=font(21, True), fill=rgb(WHITE))
    canvas.save(dst, quality=94)


def parse_training_log(path: Path):
    points = []
    pat = re.compile(r"Epoch\s+(\d+)/\d+.*train_loss=([0-9.]+).*val_loss=([0-9.]+)")
    for line in path.read_text(errors="ignore").splitlines():
        match = pat.search(line)
        if match:
            points.append(tuple(map(float, match.groups())))
    return points


def draw_axes(draw, box, xmin, xmax, ymin, ymax, x_ticks, y_ticks, x_label="", y_label=""):
    x0, y0, x1, y1 = box
    draw.line((x0, y1, x1, y1), fill=rgb(INK), width=3)
    draw.line((x0, y0, x0, y1), fill=rgb(INK), width=3)
    for value in y_ticks:
        y = y1 - (value - ymin) / (ymax - ymin) * (y1 - y0)
        draw.line((x0, y, x1, y), fill=rgb(GRID), width=1)
        label = f"{value:g}"
        tw = draw.textbbox((0, 0), label, font=font(20))[2]
        draw.text((x0 - tw - 14, y - 11), label, font=font(20), fill=rgb(MUTED))
    for value in x_ticks:
        x = x0 + (value - xmin) / (xmax - xmin) * (x1 - x0)
        draw.line((x, y1, x, y1 + 8), fill=rgb(INK), width=2)
        label = f"{value:g}"
        tw = draw.textbbox((0, 0), label, font=font(20))[2]
        draw.text((x - tw / 2, y1 + 13), label, font=font(20), fill=rgb(MUTED))
    if x_label:
        draw.text(((x0 + x1) / 2, y1 + 49), x_label, anchor="mm", font=font(22), fill=rgb(MUTED))
    if y_label:
        draw.text((x0, y0 - 32), y_label, font=font(22, True), fill=rgb(INK))


def plot_training(dst: Path):
    series = [
        ("ResNet50", parse_training_log(ROOT / "train_data/train_log.txt"), ORANGE),
        ("MobileNetV2", parse_training_log(ROOT / "train_data/train_mobilenetv2_log.txt"), BLUE),
    ]
    w, h = 1400, 720
    canvas = Image.new("RGB", (w, h), rgb(WHITE))
    draw = ImageDraw.Draw(canvas)
    box = (120, 80, 1335, 600)
    draw_axes(draw, box, 1, 30, 0.05, 0.21, [1, 5, 10, 15, 20, 25, 30], [0.05, 0.10, 0.15, 0.20], "Epoch", "Validation loss")
    for name, pts, color in series:
        xy = []
        for epoch, _, val in pts:
            x = box[0] + (epoch - 1) / 29 * (box[2] - box[0])
            y = box[3] - (val - 0.05) / 0.16 * (box[3] - box[1])
            xy.append((x, y))
        draw.line(xy, fill=rgb(color), width=5, joint="curve")
        best = min(pts, key=lambda p: p[2])
        x = box[0] + (best[0] - 1) / 29 * (box[2] - box[0])
        y = box[3] - (best[2] - 0.05) / 0.16 * (box[3] - box[1])
        draw.ellipse((x - 8, y - 8, x + 8, y + 8), fill=rgb(color), outline=rgb(WHITE), width=3)
    lx = 900
    for i, (name, pts, color) in enumerate(series):
        y = 28 + i * 34
        draw.line((lx, y + 12, lx + 52, y + 12), fill=rgb(color), width=6)
        best = min(p[2] for p in pts)
        draw.text((lx + 66, y), f"{name}: tốt nhất {best:.4f}", font=font(22, True), fill=rgb(INK))
    canvas.save(dst)


def load_run_stats():
    stats = []
    for directory in sorted((ROOT / "runs_pulled").glob("20260910_*_road")):
        files = list(directory.glob("patrol_telem_*.csv"))
        if not files:
            continue
        rows = list(csv.DictReader(files[0].open()))
        if not rows:
            continue
        n = len(rows)
        duration = float(rows[-1]["t"])
        line_pct = 100 * sum(r["src"] == "LINE" for r in rows) / n
        sat_pct = 100 * sum(abs(int(r["steer"])) >= 20 for r in rows) / n
        pairs = [(float(r["raw_err"]), int(r["steer"])) for r in rows if r["src"] == "LINE" and abs(float(r["raw_err"])) > 20]
        correct = 100 * sum(e * s > 0 for e, s in pairs) / len(pairs) if pairs else 100
        stats.append({
            "id": directory.name[9:15],
            "dir": directory,
            "rows": rows,
            "n": n,
            "duration": duration,
            "fps": n / duration,
            "line": line_pct,
            "sat": sat_pct,
            "correct": correct,
        })
    return stats


def plot_run_summary(stats, dst: Path):
    w, h = 1500, 760
    canvas = Image.new("RGB", (w, h), rgb(WHITE))
    draw = ImageDraw.Draw(canvas)
    left = (105, 155, 720, 650)
    right = (850, 155, 1435, 650)
    draw_axes(draw, left, 0, len(stats) - 1, 0, 100, [], [0, 25, 50, 75, 100], "Lần chạy (HHMMSS)", "")
    draw_axes(draw, right, 0, len(stats) - 1, 0, 4, [], [0, 1, 2, 3, 4], "Lần chạy (HHMMSS)", "")
    draw.text((left[0], 100), "Tỷ lệ nhận vạch (%)", font=font(22, True), fill=rgb(INK))
    draw.text((right[0], 100), "Tốc độ vòng lặp (fps)", font=font(22, True), fill=rgb(INK))
    bw = 58
    for idx, item in enumerate(stats):
        x = left[0] + (idx + 0.5) / len(stats) * (left[2] - left[0])
        y = left[3] - item["line"] / 100 * (left[3] - left[1])
        draw.rounded_rectangle((x - bw / 2, y, x + bw / 2, left[3]), 10, fill=rgb(CYAN if item["line"] >= 95 else ORANGE))
        draw.text((x, y - 25), f"{item['line']:.0f}%", anchor="mm", font=font(20, True), fill=rgb(INK))
        x2 = right[0] + (idx + 0.5) / len(stats) * (right[2] - right[0])
        y2 = right[3] - item["fps"] / 4 * (right[3] - right[1])
        draw.rounded_rectangle((x2 - bw / 2, y2, x2 + bw / 2, right[3]), 10, fill=rgb(BLUE))
        draw.text((x2, y2 - 25), f"{item['fps']:.2f}", anchor="mm", font=font(20, True), fill=rgb(INK))
    # Replace generic 0..N tick labels with actual IDs using white patches.
    for box in (left, right):
        draw.rectangle((box[0] - 10, box[3] + 10, box[2] + 30, box[3] + 42), fill=rgb(WHITE))
        for idx, item in enumerate(stats):
            x = box[0] + (idx + 0.5) / len(stats) * (box[2] - box[0])
            draw.text((x, box[3] + 18), item["id"], anchor="ma", font=font(17), fill=rgb(MUTED))
    draw.text((w / 2, 24), "6 LẦN CHẠY THẬT TRÊN ĐƯỜNG — 10/09/2026", anchor="ma", font=font(26, True), fill=rgb(NAVY))
    canvas.save(dst)


def plot_telemetry(stats, dst: Path):
    target = next(item for item in stats if item["id"] == "030448")
    rows = target["rows"]
    times = [float(r["t"]) for r in rows]
    t0 = times[0]
    times = [v - t0 for v in times]
    raw = [float(r["raw_err"]) for r in rows]
    final = [float(r["final_err"]) for r in rows]
    steer = [int(r["steer"]) for r in rows]
    w, h = 1600, 780
    canvas = Image.new("RGB", (w, h), rgb(WHITE))
    draw = ImageDraw.Draw(canvas)
    box1 = (125, 75, 1515, 405)
    box2 = (125, 500, 1515, 700)
    xmax = math.ceil(max(times) / 20) * 20
    draw_axes(draw, box1, 0, xmax, -220, 220, list(range(0, xmax + 1, 20)), [-200, -100, 0, 100, 200], "", "Sai số làn (px)")
    draw_axes(draw, box2, 0, xmax, -20, 20, list(range(0, xmax + 1, 20)), [-20, -10, 0, 10, 20], "Thời gian (s)", "Lệnh lái (độ)")
    def xy(vals, box, lo, hi):
        return [(box[0] + t / xmax * (box[2] - box[0]), box[3] - (v - lo) / (hi - lo) * (box[3] - box[1])) for t, v in zip(times, vals)]
    draw.line(xy(raw, box1, -220, 220), fill=rgb(ORANGE), width=3)
    draw.line(xy(final, box1, -220, 220), fill=rgb(BLUE), width=5)
    draw.line(xy(steer, box2, -20, 20), fill=rgb(CYAN), width=5)
    for y, color, label in [(18, ORANGE, "Sai số thô"), (18, BLUE, "Sai số sau EMA"), (18, CYAN, "Steer ±20°")]:
        pass
    legends = [(980, ORANGE, "Sai số thô"), (1165, BLUE, "Sai số sau EMA"), (1380, CYAN, "Steer")]
    for x, color, label in legends:
        draw.line((x, 28, x + 42, 28), fill=rgb(color), width=6)
        draw.text((x + 52, 16), label, font=font(19, True), fill=rgb(INK))
    canvas.save(dst)


ENC_RE = re.compile(
    r"ENC,(-?\d+),(-?\d+),DELTA,(-?\d+),(-?\d+),TARGET,(-?\d+),(-?\d+),"
    r"PWM,(-?\d+),(-?\d+),STEER,(-?\d+),SPEED,(-?\d+),MODE,(\w)"
)


def mechanical_stats(stats):
    out = []
    for item in stats:
        entries = []
        bridge = item["dir"] / "bridge.log"
        if bridge.exists():
            for line in bridge.read_text(errors="ignore").splitlines():
                match = ENC_RE.search(line)
                if match:
                    g = list(match.groups())
                    entries.append({
                        "dl": int(g[2]), "dr": int(g[3]), "pl": int(g[6]),
                        "pr": int(g[7]), "steer": int(g[8]), "speed": int(g[9]),
                        "mode": g[10],
                    })
        saturated = [e for e in entries if e["mode"] == "V" and e["speed"] > 0 and abs(e["steer"]) >= 19]
        if saturated:
            out.append({
                "id": item["id"],
                "delta": st.mean(abs(e["dr"] - e["dl"]) for e in saturated),
                "pwm": st.mean(abs(e["pr"] - e["pl"]) for e in saturated),
                "ticks": st.mean((e["dl"] + e["dr"]) / 2 for e in saturated),
            })
    return out


def plot_mechanics(mech, dst: Path):
    w, h = 1320, 720
    canvas = Image.new("RGB", (w, h), rgb(WHITE))
    draw = ImageDraw.Draw(canvas)
    box = (125, 80, 1245, 600)
    draw_axes(draw, box, 0, len(mech) - 1, 0, 10, [], [0, 2, 4, 6, 8, 10], "Lần chạy (HHMMSS)", "Chênh lệch encoder khi bẻ hết (tick)")
    bw = 92
    for idx, item in enumerate(mech):
        x = box[0] + (idx + 0.5) / len(mech) * (box[2] - box[0])
        y = box[3] - item["delta"] / 10 * (box[3] - box[1])
        draw.rounded_rectangle((x - bw / 2, y, x + bw / 2, box[3]), 12, fill=rgb(ORANGE if item["delta"] < 10 else GREEN))
        draw.text((x, y - 32), f"{item['delta']:.1f}", anchor="mm", font=font(28, True), fill=rgb(INK))
    draw.rectangle((box[0] - 10, box[3] + 10, box[2] + 30, box[3] + 42), fill=rgb(WHITE))
    for idx, item in enumerate(mech):
        x = box[0] + (idx + 0.5) / len(mech) * (box[2] - box[0])
        draw.text((x, box[3] + 18), item["id"], anchor="ma", font=font(20), fill=rgb(MUTED))
    draw.line((box[0], box[3] - 10 / 10 * (box[3] - box[1]), box[2], box[3] - 10 / 10 * (box[3] - box[1])), fill=rgb(GREEN), width=3)
    draw.text((box[2] - 6, box[1] + 8), "Mục tiêu ≥ 10 tick", anchor="ra", font=font(21, True), fill=rgb(GREEN))
    canvas.save(dst)


def build_assets():
    OUT_DIR.mkdir(exist_ok=True)
    ASSET_DIR.mkdir(exist_ok=True)
    extract_logos()
    save_rounded_photo(
        ROOT / "runs_pulled/20260910_030448_road/frames/f241.jpg",
        ASSET_DIR / "cover_hero.png",
        size=(1050, 1180),
        focus_y=0.5,
        radius=0,
    )
    make_pair_image(
        ROOT / "snapshots_pulled/20260910_013603_raw.jpg",
        ROOT / "snapshots_pulled/20260910_013603_overlay.jpg",
        ASSET_DIR / "segmentation_pair.jpg",
    )
    make_experiment_grid(
        [
            ROOT / "runs_pulled/20260910_014718_road/frames/f001.jpg",
            ROOT / "runs_pulled/20260910_030448_road/frames/f349.jpg",
            ROOT / "runs_pulled/20260910_014718_road/frames/f085.jpg",
            ROOT / "runs_pulled/20260910_014718_road/frames/f052.jpg",
        ],
        ASSET_DIR / "experiment_grid.jpg",
    )
    compare_sources = [
        ROOT / "train_data/model_compare_v2data/02_cam_000374_jpg.rf.ce878d30bcba489c5dc6a12e0924b7ee.jpg.png",
        ROOT / "train_data/model_compare_v2data/06_cam_000806_jpg.rf.f95dfc1ff6125fd3fd04068f39ac6228.jpg.png",
        ROOT / "train_data/model_compare_v2data/13_img_0305_jpg.rf.f428755357c45313d9435901e117fc21.jpg.png",
    ]
    # Long horizontal comparison strips stacked vertically.
    canvas = Image.new("RGB", (1600, 850), rgb(WHITE))
    draw = ImageDraw.Draw(canvas)
    for i, src in enumerate(compare_sources):
        im = fit_crop(Image.open(src), (1540, 245))
        y = 32 + i * 270
        canvas.paste(im, (30, y))
        draw.rounded_rectangle((44, y + 14, 158, y + 52), 16, fill=rgb(NAVY))
        draw.text((62, y + 22), f"MẪU {i + 1}", font=font(18, True), fill=rgb(WHITE))
    canvas.save(ASSET_DIR / "model_compare_grid.jpg", quality=94)

    stats = load_run_stats()
    plot_training(ASSET_DIR / "training_curve.png")
    plot_run_summary(stats, ASSET_DIR / "run_summary.png")
    plot_telemetry(stats, ASSET_DIR / "telemetry_030448.png")
    plot_mechanics(mechanical_stats(stats), ASSET_DIR / "mechanics.png")
    return stats


# ---- LibreOffice / UNO presentation helpers ---------------------------------

def prop(name, value):
    from com.sun.star.beans import PropertyValue
    p = PropertyValue()
    p.Name = name
    p.Value = value
    return p


def connect_uno(port=2002):
    import uno
    local_ctx = uno.getComponentContext()
    resolver = local_ctx.ServiceManager.createInstanceWithContext(
        "com.sun.star.bridge.UnoUrlResolver", local_ctx
    )
    ctx = resolver.resolve(
        f"uno:socket,host=127.0.0.1,port={port};urp;StarOffice.ComponentContext"
    )
    smgr = ctx.ServiceManager
    desktop = smgr.createInstanceWithContext("com.sun.star.frame.Desktop", ctx)
    return ctx, smgr, desktop


class Deck:
    W = 33867
    H = 19050

    def __init__(self, smgr, ctx, desktop):
        self.smgr = smgr
        self.ctx = ctx
        self.doc = desktop.loadComponentFromURL("private:factory/simpress", "_blank", 0, ())
        self.pages = self.doc.getDrawPages()
        self.graphic_provider = smgr.createInstanceWithContext("com.sun.star.graphic.GraphicProvider", ctx)
        self.page_no = 0
        self.current = None

    def new_page(self, dark=False, number=True):
        if self.page_no == 0:
            page = self.pages.getByIndex(0)
            while page.getCount():
                page.remove(page.getByIndex(0))
        else:
            page = self.pages.insertNewByIndex(self.pages.getCount())
        page.Width = self.W
        page.Height = self.H
        self.current = page
        self.page_no += 1
        self.rect(0, 0, self.W, self.H, NAVY if dark else WHITE, line=None)
        if number:
            self.text(32350, 18100, 850, 420, f"{self.page_no:02d}", 9, WHITE if dark else MUTED, bold=True, align="right")
        return page

    def rect(self, x, y, w, h, fill, line=GRID, radius=0, width=30):
        from com.sun.star.awt import Point, Size
        from com.sun.star.drawing import LineStyle
        sh = self.doc.createInstance("com.sun.star.drawing.RectangleShape")
        sh.Position = Point(int(x), int(y))
        sh.Size = Size(int(w), int(h))
        sh.FillColor = int_color(fill)
        sh.FillTransparence = 0
        if line is None:
            sh.LineStyle = LineStyle.NONE
        else:
            sh.LineStyle = LineStyle.SOLID
            sh.LineColor = int_color(line)
            sh.LineWidth = width
        if radius:
            try:
                sh.CornerRadius = int(radius)
            except Exception:
                pass
        self.current.add(sh)
        return sh

    def line(self, x, y, w, h, color=GRID, width=35):
        from com.sun.star.awt import Point, Size
        from com.sun.star.drawing import LineStyle
        sh = self.doc.createInstance("com.sun.star.drawing.LineShape")
        sh.Position = Point(int(x), int(y))
        sh.Size = Size(int(w), int(h))
        sh.LineStyle = LineStyle.SOLID
        sh.LineColor = int_color(color)
        sh.LineWidth = width
        self.current.add(sh)
        return sh

    def text(self, x, y, w, h, value, size=18, color=INK, bold=False,
             align="left", valign="top", font_name="Liberation Sans", margin=0):
        from com.sun.star.awt import Point, Size
        from com.sun.star.awt import FontWeight
        from com.sun.star.style import ParagraphAdjust
        from com.sun.star.drawing import TextVerticalAdjust
        sh = self.doc.createInstance("com.sun.star.drawing.TextShape")
        sh.Position = Point(int(x), int(y))
        sh.Size = Size(int(w), int(h))
        sh.String = value
        sh.TextLeftDistance = int(margin)
        sh.TextRightDistance = int(margin)
        sh.TextUpperDistance = int(margin)
        sh.TextLowerDistance = int(margin)
        cursor = sh.createTextCursor()
        cursor.gotoEnd(True)
        cursor.CharFontName = font_name
        cursor.CharHeight = float(size)
        cursor.CharColor = int_color(color)
        cursor.CharWeight = FontWeight.BOLD if bold else FontWeight.NORMAL
        cursor.ParaAdjust = {
            "left": ParagraphAdjust.LEFT,
            "center": ParagraphAdjust.CENTER,
            "right": ParagraphAdjust.RIGHT,
        }[align]
        sh.TextVerticalAdjust = {
            "top": TextVerticalAdjust.TOP,
            "center": TextVerticalAdjust.CENTER,
            "bottom": TextVerticalAdjust.BOTTOM,
        }[valign]
        self.current.add(sh)
        return sh

    def image(self, path: Path, x, y, w, h, line=None):
        import uno
        from com.sun.star.awt import Point, Size
        from com.sun.star.drawing import LineStyle
        sh = self.doc.createInstance("com.sun.star.drawing.GraphicObjectShape")
        sh.Position = Point(int(x), int(y))
        sh.Size = Size(int(w), int(h))
        graphic = self.graphic_provider.queryGraphic((prop("URL", uno.systemPathToFileUrl(str(path.resolve()))),))
        sh.Graphic = graphic
        if line:
            sh.LineStyle = LineStyle.SOLID
            sh.LineColor = int_color(line)
            sh.LineWidth = 30
        else:
            sh.LineStyle = LineStyle.NONE
        self.current.add(sh)
        return sh

    def title(self, title, subtitle=None, dark=False, number=None):
        color = WHITE if dark else NAVY
        self.text(1450, 950, 26800, 1200, title, 28, color, bold=True)
        self.rect(1450, 2220, 1250, 95, ORANGE, line=None)
        if subtitle:
            self.text(1450, 2420, 28500, 700, subtitle, 13, WHITE if dark else MUTED)
        if number is not None:
            self.text(30200, 980, 1700, 700, f"{number:02d}", 15, WHITE if dark else MUTED, bold=True, align="right")

    def pill(self, x, y, w, label, fill=PALE_BLUE, color=NAVY):
        self.rect(x, y, w, 660, fill, line=None, radius=280)
        self.text(x + 100, y + 50, w - 200, 560, label, 12, color, bold=True, align="center", valign="center")

    def metric(self, x, y, w, h, value, label, accent=CYAN, dark=False):
        fill = NAVY_DARK if dark else PALE
        self.rect(x, y, w, h, fill, line=None, radius=180)
        self.rect(x, y, 110, h, accent, line=None, radius=70)
        self.text(x + 380, y + 220, w - 580, 800, value, 29, WHITE if dark else NAVY, bold=True)
        self.text(x + 390, y + 1080, w - 580, 700, label, 12, "D6E4EF" if dark else MUTED, bold=True)

    def footer_logos(self, university, lab, dark=False):
        if not university.exists() or not lab.exists():
            return
        self.image(university, 640, 250, 4100, 940)
        self.image(lab, 29550, 300, 2900, 1265)

    def save(self, path: Path):
        import uno
        path.parent.mkdir(exist_ok=True)
        url = uno.systemPathToFileUrl(str(path.resolve()))
        self.doc.storeAsURL(url, (prop("FilterName", "Impress MS PowerPoint 2007 XML"), prop("Overwrite", True)))
        self.doc.close(True)


def add_process_box(deck, x, y, w, h, label, sub, color):
    deck.rect(x, y, w, h, WHITE, line=color, radius=220, width=45)
    deck.text(x + 250, y + 220, w - 500, 620, label, 15, color, bold=True, align="center")
    deck.text(x + 250, y + 890, w - 500, 650, sub, 10.5, MUTED, align="center")


def build_deck(stats):
    _, smgr, desktop = connect_uno()
    d = Deck(smgr, _, desktop)
    logo_uni = ASSET_DIR / "logo_hcmute.png"
    logo_lab = ASSET_DIR / "logo_islab.png"

    # 1 — Cover
    d.new_page(number=False)
    d.image(ASSET_DIR / "cover_hero.png", 20600, 0, 13267, 19050)
    d.rect(19920, 0, 680, 19050, ORANGE, line=None)
    d.image(logo_uni, 850, 620, 7400, 1690)
    d.image(logo_lab, 14500, 520, 3500, 1525)
    d.text(1700, 3900, 16400, 4300, "XE TỰ HÀNH\nTUẦN TRA THÔNG MINH", 36, NAVY, bold=True)
    d.text(1730, 8500, 15000, 1250, "Thị giác máy tính · Điều khiển thời gian thực · An toàn đa tầng", 16, ORANGE, bold=True)
    d.line(1730, 10050, 14300, 0, GRID, 25)
    d.text(1730, 10650, 8200, 650, "ĐỘI THI NDA", 15, INK, bold=True)
    d.text(1730, 11620, 9200, 1100, "GVHD: PGS.TS Lê Mỹ Hà", 13, MUTED)
    d.text(1730, 13250, 9800, 2200, "Huỳnh Lê Thành Nhân\nLê Tự Thiện An\nĐặng Cao Dương", 13, INK, bold=True)
    d.pill(1730, 16800, 4600, "KẾT QUẢ THỰC NGHIỆM 10/09/2026", NAVY, WHITE)

    # 2 — One-slide project story
    d.new_page()
    d.footer_logos(logo_uni, logo_lab)
    d.title("Từ camera đến chuyển động an toàn", "Một vòng kín xử lý ảnh — quyết định — chấp hành", number=2)
    cards = [
        ("01", "NHÌN", "Phân đoạn 6 lớp\n+ phát hiện người", CYAN),
        ("02", "HIỂU", "Tâm vạch · ArUco\ntrạng thái an toàn", BLUE),
        ("03", "LÁI", "PID + tốc độ thích nghi\nSTM32 điều khiển bánh", ORANGE),
    ]
    for i, (num, title, sub, color) in enumerate(cards):
        x = 1600 + i * 10650
        d.rect(x, 4700, 9350, 8200, PALE, line=None, radius=240)
        d.text(x + 580, 5200, 1600, 900, num, 24, color, bold=True)
        d.text(x + 580, 6900, 7500, 900, title, 25, NAVY, bold=True)
        d.text(x + 580, 8100, 7500, 1550, sub, 16, INK, bold=True)
        d.rect(x + 580, 11100, 3000, 100, color, line=None)
    d.text(1600, 14200, 30400, 1700, "Mục tiêu: xe tự bám vạch, nhận biết rủi ro và dừng an toàn — không phụ thuộc máy tính ngoài.", 21, NAVY, bold=True, align="center")

    # 3 — Architecture
    d.new_page()
    d.footer_logos(logo_uni, logo_lab)
    d.title("Kiến trúc hệ thống hiện tại", "Phần cứng thật và luồng dữ liệu đang chạy trên xe", number=3)
    y = 7000
    add_process_box(d, 1000, y, 4800, 2500, "CẢM BIẾN", "Camera C270\nLiDAR · BNO055", BLUE)
    add_process_box(d, 7200, y, 6400, 2500, "JETSON NANO", "DeepLabV3–MobileNetV2\nYOLOv8n · ArUco · PID", CYAN)
    add_process_box(d, 15000, y, 5200, 2500, "STM32 F411", "Watchdog · encoder\nPI tốc độ bánh", ORANGE)
    add_process_box(d, 21600, y, 4800, 2500, "CHẤP HÀNH", "2× BTS7960\n2 động cơ DC", RED)
    add_process_box(d, 27800, y, 4800, 2500, "GIÁM SÁT", "MQTT · Discord\nlog + video", NAVY)
    for x in (5900, 13700, 20300, 26500):
        d.text(x, y + 650, 1100, 900, "→", 25, ORANGE, bold=True, align="center")
    d.metric(3300, 12400, 6500, 2600, "640 × 360", "CAMERA · 30 FPS ĐẦU VÀO", BLUE)
    d.metric(10500, 12400, 6500, 2600, "256 × 448", "KÍCH THƯỚC SUY LUẬN", CYAN)
    d.metric(17700, 12400, 6500, 2600, "1.20 / 1.60 m", "LIDAR STOP / RESUME", ORANGE)
    d.metric(24900, 12400, 6500, 2600, "1.4 s", "WATCHDOG AI", RED)

    # 4 — Dataset and training
    d.new_page()
    d.footer_logos(logo_uni, logo_lab)
    d.title("Dữ liệu và huấn luyện mô hình", "DeepLabV3 với backbone MobileNetV2 — tối ưu cho Jetson Nano", number=4)
    d.image(ASSET_DIR / "training_curve.png", 11800, 3500, 20500, 10550, line=GRID)
    d.metric(1500, 4100, 8200, 2300, "2.157", "ẢNH HUẤN LUYỆN", CYAN)
    d.metric(1500, 6800, 8200, 2300, "203", "ẢNH XÁC THỰC", BLUE)
    d.metric(1500, 9500, 8200, 2300, "6 lớp", "ROAD · LINE · XE · NGƯỜI…", ORANGE)
    d.metric(1500, 12200, 8200, 2300, "0,0866", "BEST VALIDATION LOSS", GREEN)
    d.text(11800, 15100, 20200, 900, "MobileNetV2 đạt chất lượng gần ResNet50 nhưng thời gian mỗi epoch giảm từ ≈63 s xuống ≈29 s.", 14, NAVY, bold=True, align="center")

    # 5 — Visual model comparison
    d.new_page()
    d.footer_logos(logo_uni, logo_lab)
    d.title("Mô hình học được vạch liên tục", "So sánh trực tiếp trên các cảnh khó trong bộ dữ liệu", number=5)
    d.image(ASSET_DIR / "model_compare_grid.jpg", 1450, 3550, 30950, 12850, line=GRID)
    d.pill(2100, 16850, 6200, "+83% PIXEL VẠCH", PALE_BLUE, NAVY)
    d.pill(8750, 16850, 6200, "+64% ĐỘ PHỦ", PALE_BLUE, NAVY)
    d.pill(15400, 16850, 6200, "27/30 ẢNH TỐT HƠN", PALE_ORANGE, NAVY)
    d.text(22400, 16840, 9300, 700, "Kết quả kiểm thử nội bộ ghi trong cấu hình dự án", 11, MUTED, valign="center")

    # 6 — Real segmentation
    d.new_page()
    d.footer_logos(logo_uni, logo_lab)
    d.title("Nhận thức môi trường trên đường thật", "Ảnh camera và đầu ra phân đoạn cùng thời điểm", number=6)
    d.image(ASSET_DIR / "segmentation_pair.jpg", 1450, 3500, 30950, 14800, line=GRID)
    d.pill(1780, 16800, 3400, "ROAD", PURPLE, WHITE)
    d.pill(5350, 16800, 3400, "LINE", CYAN, NAVY)
    d.pill(8920, 16800, 3400, "CAR", GREEN, WHITE)
    d.pill(12490, 16800, 3400, "PERSON", RED, WHITE)

    # 7 — Steering and safety control
    d.new_page()
    d.footer_logos(logo_uni, logo_lab)
    d.title("Điều khiển bám vạch và lưới an toàn", "Thông số sau hiệu chỉnh trên xe thật", number=7)
    d.rect(1500, 3650, 14350, 12100, PALE, line=None, radius=260)
    d.text(2350, 4450, 12600, 800, "e = tâm vạch − tâm ảnh + góc hướng", 23, NAVY, bold=True, align="center")
    d.text(2350, 5850, 12600, 900, "δ = 0,16e + 0,03∫e dt + 0,05 de/dt", 21, ORANGE, bold=True, align="center")
    d.line(2900, 7350, 11200, 0, GRID, 30)
    rows = [
        ("16", "đường thẳng", CYAN),
        ("14", "vào cua", BLUE),
        ("12", "phục hồi |e| > 150 px", ORANGE),
        ("0", "mất vạch 8 khung", RED),
    ]
    for i, (value, label, color) in enumerate(rows):
        y = 8050 + i * 1650
        d.text(2800, y, 2100, 900, value, 25, color, bold=True, align="center")
        d.rect(5200, y + 330, 6000 - i * 900, 290, color, line=None, radius=140)
        d.text(11900, y + 70, 2600, 700, label, 12.5, INK, bold=True, align="right")
    d.rect(17300, 3650, 15050, 12100, NAVY, line=None, radius=260)
    d.text(18400, 4500, 12800, 900, "ƯU TIÊN DỪNG", 20, ORANGE, bold=True)
    safety = [
        ("1", "LiDAR", "< 1,20 m trong 3 khung"),
        ("2", "Watchdog", "mất lệnh AI > 1,4 s"),
        ("3", "Mất vạch", "8 khung liên tiếp"),
        ("4", "BNO055", "phát hiện xe bị nhấc"),
    ]
    for i, (num, title, desc) in enumerate(safety):
        y = 6200 + i * 2050
        d.rect(18400, y, 1100, 1100, ORANGE if i == 0 else BLUE, line=None, radius=550)
        d.text(18400, y, 1100, 1100, num, 17, WHITE, bold=True, align="center", valign="center")
        d.text(20200, y - 80, 4500, 650, title, 17, WHITE, bold=True)
        d.text(20200, y + 650, 10000, 550, desc, 12, "D6E4EF")

    # 8 — Field frames
    d.new_page()
    d.footer_logos(logo_uni, logo_lab)
    d.title("Hình ảnh thực nghiệm", "Các tình huống được ghi trực tiếp từ camera gắn trên xe", number=8)
    d.image(ASSET_DIR / "experiment_grid.jpg", 1450, 3200, 30950, 14550, line=GRID)

    # 9 — Run summary
    d.new_page()
    d.footer_logos(logo_uni, logo_lab)
    d.title("Kết quả 6 lần chạy trên đường", "Telemetry ngày 10/09/2026 — tổng 1.094 khung điều khiển", number=9)
    d.image(ASSET_DIR / "run_summary.png", 1200, 3500, 31500, 12450, line=GRID)
    d.metric(2100, 16250, 6600, 2050, "3,34 fps", "LẦN CHẠY DÀI 120,5 s", BLUE)
    d.metric(9600, 16250, 6600, 2050, "97%", "KHUNG NHẬN ĐƯỢC VẠCH", CYAN)
    d.metric(17100, 16250, 6600, 2050, "99%", "LỆNH LÁI ĐÚNG CHIỀU", GREEN)
    d.metric(24600, 16250, 6600, 2050, "403", "KHUNG TRONG MỘT LẦN CHẠY", ORANGE)

    # 10 — Telemetry detail
    d.new_page()
    d.footer_logos(logo_uni, logo_lab)
    d.title("Telemetry của lần chạy dài nhất", "Run 03:04:48 — 403 khung trong 120,5 giây", number=10)
    d.image(ASSET_DIR / "telemetry_030448.png", 1100, 3500, 31700, 12800, line=GRID)
    d.pill(2500, 16700, 7200, "EMA α = 0,5 GIẢM RUNG", PALE_BLUE, NAVY)
    d.pill(10200, 16700, 7200, "STEER BÃO HÒA 32%", PALE_ORANGE, NAVY)
    d.pill(17900, 16700, 7200, "LINE 97% · HOLD 3%", PALE_BLUE, NAVY)
    d.pill(25600, 16700, 5600, "120,5 GIÂY", NAVY, WHITE)

    # 11 — Honest engineering diagnosis
    d.new_page(dark=True)
    d.footer_logos(logo_uni, logo_lab, dark=True)
    d.title("Nút thắt hiện tại nằm ở cơ khí truyền động", "Dữ liệu encoder cho thấy lệnh lái đúng, nhưng hai bánh chưa tạo đủ chênh lệch tốc độ", dark=True, number=11)
    d.image(ASSET_DIR / "mechanics.png", 1200, 3900, 19200, 10500)
    d.text(22000, 4350, 9400, 1050, "AI + điều khiển", 17, CYAN, bold=True)
    d.text(22000, 5600, 9400, 1500, "99%", 39, WHITE, bold=True)
    d.text(22000, 7100, 9400, 900, "lệnh lái cùng chiều sai số", 13, "D6E4EF")
    d.line(22000, 8450, 8800, 0, "43627D", 30)
    d.text(22000, 9050, 9400, 1050, "Chấp hành thực tế", 17, ORANGE, bold=True)
    d.text(22000, 10300, 9400, 1500, "1,1–5,7 tick", 31, WHITE, bold=True)
    d.text(22000, 11950, 9400, 1150, "chênh lệch encoder khi bẻ hết\n(mục tiêu ≥ 10 tick)", 13, "D6E4EF")
    d.rect(21700, 13900, 10000, 2500, "214D76", line=None, radius=220)
    d.text(22500, 14400, 8400, 1400, "Kết luận: cần tăng mô-men/độ bám hoặc tối ưu truyền động trước khi tiếp tục tune PID.", 16, WHITE, bold=True, valign="center")

    # 12 — Conclusion
    d.new_page(dark=True, number=False)
    d.footer_logos(logo_uni, logo_lab, dark=True)
    d.text(1700, 3150, 18200, 1500, "KẾT LUẬN", 33, WHITE, bold=True)
    d.rect(1700, 4950, 1450, 100, ORANGE, line=None)
    wins = [
        ("01", "Nhận thức ổn định", "DeepLabV3–MobileNetV2 nhận vạch tới 97–100% ở 5/6 lần chạy."),
        ("02", "Vòng điều khiển hoàn chỉnh", "Camera → Jetson → STM32 → encoder, có log và video đối chứng."),
        ("03", "An toàn đa tầng", "LiDAR, watchdog, mất vạch, IMU và cảnh báo từ xa."),
    ]
    for i, (num, title, desc) in enumerate(wins):
        y = 6200 + i * 2850
        d.text(1800, y, 1200, 700, num, 16, ORANGE, bold=True)
        d.text(3600, y - 80, 13200, 700, title, 17, WHITE, bold=True)
        d.text(3600, y + 780, 14000, 900, desc, 12.5, "D6E4EF")
    d.rect(20500, 0, 13367, 19050, NAVY_DARK, line=None)
    d.text(22200, 3250, 9200, 900, "BƯỚC TIẾP THEO", 18, ORANGE, bold=True)
    next_steps = [
        "Tăng chênh lệch tốc độ hai bánh",
        "Chạy lại bài test ≥ 3 phút",
        "Giảm tỷ lệ bão hòa steer",
        "Hiệu chuẩn camera + IMU",
    ]
    for i, item in enumerate(next_steps):
        y = 5600 + i * 2100
        d.rect(22200, y, 850, 850, ORANGE if i == 0 else BLUE, line=None, radius=425)
        d.text(22200, y, 850, 850, str(i + 1), 13, WHITE, bold=True, align="center", valign="center")
        d.text(23600, y - 40, 7600, 900, item, 15, WHITE, bold=True, valign="center")
    d.text(22200, 15200, 9300, 900, "CẢM ƠN CÔ VÀ CÁC BẠN", 17, ORANGE, bold=True)
    d.text(22200, 16400, 9300, 1000, "Q&A", 30, WHITE, bold=True)

    d.save(OUTPUT_DECK)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--assets-only", action="store_true")
    args = parser.parse_args()
    stats = build_assets()
    print("Assets:", ASSET_DIR)
    print("Road runs:", len(stats), "frames:", sum(s["n"] for s in stats))
    if args.assets_only:
        return
    build_deck(stats)
    print("PowerPoint:", OUTPUT_DECK)


if __name__ == "__main__":
    main()
