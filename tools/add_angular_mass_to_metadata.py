#!/usr/bin/env python3
"""Compute angular_mass from non-convex polygon files and append to metadata.csv."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

DELIMITER = ";"


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


def angular_mass(poly: list[tuple[float, float]]) -> float:
    if len(poly) < 3:
        return 0.0
    if signed_area(poly) < 0:
        poly = list(reversed(poly))

    mass = 0.0
    n = len(poly)
    for i in range(n):
        ax, ay = poly[(i - 1) % n]
        bx, by = poly[i]
        cx, cy = poly[(i + 1) % n]
        ux, uy = ax - bx, ay - by
        vx, vy = cx - bx, cy - by
        lu = math.hypot(ux, uy)
        lv = math.hypot(vx, vy)
        if lu < 1e-12 or lv < 1e-12:
            continue
        cross = ux * vy - uy * vx
        if cross >= 0:
            continue
        dot = max(-1.0, min(1.0, (ux * vx + uy * vy) / (lu * lv)))
        angle = math.acos(dot)
        mass += math.pi - angle
    return mass


def process_run(run_dir: Path) -> int:
    metadata_path = run_dir / "metadata.csv"
    with metadata_path.open("r", newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f, delimiter=DELIMITER))
        fieldnames = list(rows[0].keys()) if rows else []

    if "angular_mass" not in fieldnames:
        insert_idx = fieldnames.index("alpha_lebedev_full") + 1 if "alpha_lebedev_full" in fieldnames else len(fieldnames)
        fieldnames.insert(insert_idx, "angular_mass")

    updated = 0
    for row in rows:
        case_id = row.get("case_id", "")
        if not case_id:
            continue
        if row.get("angular_mass", "").strip():
            continue
        poly_path = run_dir / case_id / f"{case_id}_polygon_nonconvex.txt"
        if not poly_path.exists():
            continue
        mass = angular_mass(read_polygon(poly_path))
        row["angular_mass"] = f"{mass:.8f}"
        updated += 1

    with metadata_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, delimiter=DELIMITER)
        writer.writeheader()
        writer.writerows(rows)
    return updated


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--run-dir", type=Path, required=True)
    args = ap.parse_args()
    changed = process_run(args.run_dir.resolve())
    print(f"Updated angular_mass for {changed} rows in {args.run_dir / 'metadata.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
