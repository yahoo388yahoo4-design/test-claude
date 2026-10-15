#!/usr/bin/env python3
"""Printable ID tags for the ESP32 nodes, found automatically while you scan the room.

Each tag is a 70 mm square: a textured border (gives ARCore / ARKit image tracking enough features to
find the tag live in the apps) around a 40 mm ArUco 4x4 marker whose id is the node id (found offline in
recorded video by esp_locate.py). Stick tag N on, or flat against, ESP node N.

  python esp_tags.py                     # tags/esp_1..6.png, tags/esp_tags_A4.pdf (+ .png) and esp_tags.json
  python esp_tags.py --install           # also copy the tag images into the Android and iOS apps

Print the PDF at 100 % ("actual size", no fit-to-page) and check the 50 mm bar with a ruler.
"""
from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, str(Path(__file__).resolve().parent))
from esp_common import ARUCO_DICT, ARUCO_SIZE_M, HERE, NODE_COUNT, REPO, TAG_SIZE_M  # noqa: E402

PX = 800                                   # tag image side (pixels)
ARUCO_PX = int(round(PX * ARUCO_SIZE_M / TAG_SIZE_M))   # 457
ANDROID_ASSETS = REPO / "android/app/src/main/assets/esp_tags"
IOS_TAGS = REPO / "ios/R2SCapture/Esp/Tags"
PALETTE = [(20, 20, 20), (200, 30, 40), (20, 90, 200), (240, 170, 0), (10, 140, 70), (120, 40, 160), (250, 250, 250)]


def font(size: int):
    for name in ("DejaVuSans-Bold.ttf", "Arial Bold.ttf", "arialbd.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            pass
    return ImageFont.load_default(size=size)


def aruco_bits(node_id: int) -> np.ndarray:
    """(6, 6) uint8 0/1 marker cells incl. the black border, for DICT_4X4_50 id = node_id."""
    import cv2
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, ARUCO_DICT))
    img = cv2.aruco.generateImageMarker(d, node_id, 6, borderBits=1)
    return (img > 127).astype(np.uint8)


def make_tag(node_id: int) -> Image.Image:
    rng = np.random.RandomState(9300 + node_id)   # seeds checked with ARCore arcoreimg: score 100 for all six
    im = Image.new("RGB", (PX, PX), (235, 232, 222))
    dr = ImageDraw.Draw(im)
    # Feature-rich random texture: polygons, discs and bars in high-contrast colours.
    for _ in range(420):
        c = PALETTE[rng.randint(len(PALETTE))]
        kind = rng.randint(3)
        x, y = rng.randint(0, PX, 2)
        s = rng.randint(10, 56)
        if kind == 0:
            pts = [(x + rng.randint(-s, s), y + rng.randint(-s, s)) for _ in range(rng.randint(3, 6))]
            dr.polygon(pts, fill=c)
        elif kind == 1:
            dr.ellipse([x - s // 2, y - s // 2, x + s // 2, y + s // 2], fill=c)
        else:
            dr.line([(x, y), (x + rng.randint(-2 * s, 2 * s), y + rng.randint(-2 * s, 2 * s))], fill=c,
                    width=int(rng.randint(4, 14)))
    # Node number in the four corners (also helps a person tell tags apart).
    f = font(96)
    for (cx, cy) in ((70, 70), (PX - 70, 70), (70, PX - 70), (PX - 70, PX - 70)):
        dr.rectangle([cx - 52, cy - 58, cx + 52, cy + 58], fill=(255, 255, 255))
        dr.text((cx, cy), str(node_id), fill=(0, 0, 0), font=f, anchor="mm")
    # White quiet zone + ArUco marker in the centre.
    q = int(ARUCO_PX / 12)                         # half a marker cell
    a0 = (PX - ARUCO_PX) // 2
    dr.rectangle([a0 - q, a0 - q, a0 + ARUCO_PX + q - 1, a0 + ARUCO_PX + q - 1], fill=(255, 255, 255))
    bits = aruco_bits(node_id)
    cell = ARUCO_PX / 6
    for r in range(6):
        for c in range(6):
            if bits[r, c] == 0:
                dr.rectangle([a0 + round(c * cell), a0 + round(r * cell),
                              a0 + round((c + 1) * cell) - 1, a0 + round((r + 1) * cell) - 1], fill=(0, 0, 0))
    dr.rectangle([0, 0, PX - 1, PX - 1], outline=(0, 0, 0), width=3)
    return im


def manifest() -> dict:
    return {"version": 1, "tag_size_m": TAG_SIZE_M, "aruco_size_m": round(ARUCO_PX / PX * TAG_SIZE_M, 6), "aruco_dict": ARUCO_DICT,
            "image_px": PX, "aruco_px": ARUCO_PX,
            "tags": [{"node_id": i, "file": f"esp_{i}.png"} for i in range(1, NODE_COUNT + 1)]}


def sheet(tags: list[Image.Image], dpi: int = 300) -> Image.Image:
    """A4 page, tags at exact physical size, 2 columns x 3 rows, with a 50 mm check bar."""
    mm = dpi / 25.4
    W, H = int(210 * mm), int(297 * mm)
    page = Image.new("RGB", (W, H), "white")
    dr = ImageDraw.Draw(page)
    side = int(round(TAG_SIZE_M * 1000 * mm))
    gx, gy = int(25 * mm), int(18 * mm)
    x0 = (W - 2 * side - gx) // 2
    y0 = int(32 * mm)
    f = font(int(4.2 * mm))
    dr.text((x0, int(10 * mm)), "R2S ESP32 node tags - print at 100 % (actual size). Tag N goes on node N.",
            fill="black", font=f)
    for k, t in enumerate(tags):
        r, c = divmod(k, 2)
        x, y = x0 + c * (side + gx), y0 + r * (side + gy)
        page.paste(t.resize((side, side), Image.NEAREST), (x, y))
        dr.text((x, y + side + int(2 * mm)), f"ESP node {k + 1}  (ArUco id {k + 1}, {TAG_SIZE_M * 1000:.0f} mm)",
                fill="black", font=f)
    by = y0 + 3 * (side + gy) + int(2 * mm)
    dr.rectangle([x0, by, x0 + int(50 * mm), by + int(3 * mm)], fill="black")
    dr.text((x0 + int(53 * mm), by - int(1 * mm)), "<- must measure 50 mm", fill="black", font=f)
    return page


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", type=Path, default=HERE / "tags")
    ap.add_argument("--install", action="store_true", help="copy tag images + esp_tags.json into the apps")
    a = ap.parse_args(argv)
    a.out.mkdir(parents=True, exist_ok=True)
    tags = [make_tag(i) for i in range(1, NODE_COUNT + 1)]
    for i, t in enumerate(tags, 1):
        t.save(a.out / f"esp_{i}.png", optimize=True)
    (a.out / "esp_tags.json").write_text(json.dumps(manifest(), indent=1))
    pg = sheet(tags)
    pg.save(a.out / "esp_tags_A4.pdf", resolution=300)
    pg.save(a.out / "esp_tags_A4.png", dpi=(300, 300), optimize=True)
    print(f"wrote {NODE_COUNT} tags + esp_tags_A4.pdf to {a.out}")
    if a.install:
        for dst in (ANDROID_ASSETS, IOS_TAGS):
            dst.mkdir(parents=True, exist_ok=True)
            for i in range(1, NODE_COUNT + 1):
                shutil.copy2(a.out / f"esp_{i}.png", dst / f"esp_{i}.png")
            shutil.copy2(a.out / "esp_tags.json", dst / "esp_tags.json")
            print(f"installed into {dst.relative_to(REPO)}")


if __name__ == "__main__":
    main()
