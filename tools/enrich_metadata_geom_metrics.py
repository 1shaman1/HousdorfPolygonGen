#!/usr/bin/env python3
"""Enrich metadata CSV with geometry-derived metrics for polygon pairs."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Iterable

DELIMITER = ";"
EPS = 1e-12


def read_polygon(path: Path) -> list[tuple[float, float]]:
    lines = [ln.strip() for ln in path.read_text(encoding="utf-8").splitlines() if ln.strip()]
    if not lines:
        return []
    try:
        n = int(lines[0])
    except ValueError:
        n = len(lines)
        start = 0
    else:
        start = 1
    pts: list[tuple[float, float]] = []
    for ln in lines[start : start + n]:
        parts = ln.split()
        if len(parts) < 2:
            continue
        pts.append((float(parts[0]), float(parts[1])))
    return pts


def signed_area(poly: list[tuple[float, float]]) -> float:
    s = 0.0
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        s += x1 * y2 - x2 * y1
    return 0.5 * s


def area_abs(poly: list[tuple[float, float]]) -> float:
    return abs(signed_area(poly))


def perimeter(poly: list[tuple[float, float]]) -> float:
    p = 0.0
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        p += math.hypot(x2 - x1, y2 - y1)
    return p


def point_segment_dist(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
    abx = bx - ax
    aby = by - ay
    ab2 = abx * abx + aby * aby
    if ab2 < EPS:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * abx + (py - ay) * aby) / ab2
    t = 0.0 if t < 0.0 else 1.0 if t > 1.0 else t
    qx = ax + t * abx
    qy = ay + t * aby
    return math.hypot(px - qx, py - qy)


def max_concavity_depth(nonconvex: list[tuple[float, float]], hull: list[tuple[float, float]]) -> float:
    if len(nonconvex) < 3 or len(hull) < 3:
        return 0.0
    depth = 0.0
    for px, py in nonconvex:
        best = float("inf")
        for i in range(len(hull)):
            ax, ay = hull[i]
            bx, by = hull[(i + 1) % len(hull)]
            d = point_segment_dist(px, py, ax, ay, bx, by)
            if d < best:
                best = d
        if best > depth:
            depth = best
    return depth


def oriented_bbox_aspect(points: list[tuple[float, float]]) -> float:
    if len(points) < 2:
        return 1.0
    best_area = float("inf")
    best_aspect = 1.0
    n = len(points)
    for i in range(n):
        x1, y1 = points[i]
        x2, y2 = points[(i + 1) % n]
        ex = x2 - x1
        ey = y2 - y1
        length = math.hypot(ex, ey)
        if length < EPS:
            continue
        ux, uy = ex / length, ey / length
        vx, vy = -uy, ux
        uvals = [(px * ux + py * uy) for px, py in points]
        vvals = [(px * vx + py * vy) for px, py in points]
        width = max(uvals) - min(uvals)
        height = max(vvals) - min(vvals)
        if width < EPS or height < EPS:
            continue
        area = width * height
        if area < best_area:
            best_area = area
            best_aspect = max(width, height) / min(width, height)
    return best_aspect


def reflex_deficits(poly: list[tuple[float, float]]) -> list[float]:
    if len(poly) < 3:
        return []
    signed = signed_area(poly)
    pts = list(poly) if signed >= 0 else list(reversed(poly))
    deficits: list[float] = []
    n = len(pts)
    for i in range(n):
        ax, ay = pts[(i - 1) % n]
        bx, by = pts[i]
        cx, cy = pts[(i + 1) % n]
        ux, uy = ax - bx, ay - by
        vx, vy = cx - bx, cy - by
        lu = math.hypot(ux, uy)
        lv = math.hypot(vx, vy)
        if lu < EPS or lv < EPS:
            continue
        cross = ux * vy - uy * vx
        if cross >= 0:
            continue
        dot = max(-1.0, min(1.0, (ux * vx + uy * vy) / (lu * lv)))
        angle = math.acos(dot)
        deficits.append(math.pi - angle)
    return deficits


def entropy(values: Iterable[float]) -> float:
    vals = [v for v in values if v > EPS]
    total = sum(vals)
    if total < EPS:
        return 0.0
    ent = 0.0
    for v in vals:
        p = v / total
        ent -= p * math.log(p)
    return ent


def fmt_float(value: float) -> str:
    return f"{value:.8f}" if math.isfinite(value) else ""


def run(input_csv: Path, output_csv: Path) -> None:
    with input_csv.open("r", newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f, delimiter=DELIMITER))
        fieldnames = list(rows[0].keys()) if rows else []

    extra_fields = [
        "p_over_sqrt_a",
        "a_hull_over_a",
        "convexity_deficit_rel",
        "max_concavity_depth_over_sqrt_a",
        "min_neck_width_over_sqrt_a_proxy",
        "aspect_ratio_obb",
        "angular_mass_calc",
        "angular_entropy",
    ]
    for name in extra_fields:
        if name not in fieldnames:
            fieldnames.append(name)

    base_dir = input_csv.parent
    for row in rows:
        nonconvex_rel = row.get("polygon_nonconvex_rel", "").strip()
        hull_rel = row.get("polygon_convex_rel", "").strip()
        if not nonconvex_rel or not hull_rel:
            continue
        nonconvex_path = (base_dir / nonconvex_rel).resolve()
        hull_path = (base_dir / hull_rel).resolve()
        if not nonconvex_path.exists() or not hull_path.exists():
            continue

        poly = read_polygon(nonconvex_path)
        hull = read_polygon(hull_path)
        if len(poly) < 3 or len(hull) < 3:
            continue

        a = area_abs(poly)
        a_hull = area_abs(hull)
        sqrt_a = math.sqrt(a) if a > EPS else float("nan")

        p_over_sqrt_a = perimeter(poly) / sqrt_a if math.isfinite(sqrt_a) and sqrt_a > EPS else float("nan")
        a_hull_over_a = a_hull / a if a > EPS else float("nan")
        convexity_deficit_rel = a_hull_over_a - 1.0 if math.isfinite(a_hull_over_a) else float("nan")
        max_depth = max_concavity_depth(poly, hull)
        max_depth_over_sqrt_a = max_depth / sqrt_a if math.isfinite(sqrt_a) and sqrt_a > EPS else float("nan")

        # Proxy for "neck width": convert existing mouth width normalization to sqrt(A) basis.
        pocket_width_rel_actual = row.get("pocket_width_rel_actual", "").strip()
        if pocket_width_rel_actual:
            neck_width_over_sqrt_a_proxy = float(pocket_width_rel_actual) * math.sqrt(a_hull_over_a)
        else:
            neck_width_over_sqrt_a_proxy = float("nan")

        aspect = oriented_bbox_aspect(poly)
        deficits = reflex_deficits(poly)
        mass = sum(deficits)
        ent = entropy(deficits)

        row["p_over_sqrt_a"] = fmt_float(p_over_sqrt_a)
        row["a_hull_over_a"] = fmt_float(a_hull_over_a)
        row["convexity_deficit_rel"] = fmt_float(convexity_deficit_rel)
        row["max_concavity_depth_over_sqrt_a"] = fmt_float(max_depth_over_sqrt_a)
        row["min_neck_width_over_sqrt_a_proxy"] = fmt_float(neck_width_over_sqrt_a_proxy)
        row["aspect_ratio_obb"] = fmt_float(aspect)
        row["angular_mass_calc"] = fmt_float(mass)
        row["angular_entropy"] = fmt_float(ent)

    with output_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, delimiter=DELIMITER)
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True, help="Path to source metadata.csv")
    parser.add_argument("--output", type=Path, required=True, help="Path to output enriched csv")
    args = parser.parse_args()
    run(args.input.resolve(), args.output.resolve())
    print(f"Saved enriched metadata: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
