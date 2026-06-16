#!/usr/bin/env python3
"""Enrich metadata with convex-hull gap profile and layer-deficit metrics."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

DELIMITER = ";"
EPS = 1e-12


def parse_float(value: str | None) -> float | None:
    if value is None:
        return None
    s = str(value).strip()
    if not s:
        return None
    try:
        return float(s)
    except ValueError:
        return None


def fmt_float(value: float | None) -> str:
    if value is None or not math.isfinite(value):
        return ""
    return f"{value:.8f}"


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
    t = min(1.0, max(0.0, t))
    qx = ax + t * abx
    qy = ay + t * aby
    return math.hypot(px - qx, py - qy)


def distance_to_hull_boundary(px: float, py: float, hull: list[tuple[float, float]]) -> float:
    best = float("inf")
    n = len(hull)
    for i in range(n):
        ax, ay = hull[i]
        bx, by = hull[(i + 1) % n]
        d = point_segment_dist(px, py, ax, ay, bx, by)
        if d < best:
            best = d
    return best


def sample_boundary_by_arclength(poly: list[tuple[float, float]], n_samples: int) -> list[tuple[float, float]]:
    n = len(poly)
    if n < 2:
        return []
    edge_lengths: list[float] = []
    total = 0.0
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        ll = math.hypot(x2 - x1, y2 - y1)
        edge_lengths.append(ll)
        total += ll
    if total < EPS:
        return []

    samples: list[tuple[float, float]] = []
    step = total / n_samples
    edge_idx = 0
    edge_start = 0.0
    edge_end = edge_lengths[0]
    for k in range(n_samples):
        s = (k + 0.5) * step
        while s > edge_end and edge_idx < n - 1:
            edge_idx += 1
            edge_start = edge_end
            edge_end += edge_lengths[edge_idx]
        x1, y1 = poly[edge_idx]
        x2, y2 = poly[(edge_idx + 1) % n]
        ll = edge_lengths[edge_idx]
        if ll < EPS:
            samples.append((x1, y1))
            continue
        t = (s - edge_start) / ll
        samples.append((x1 + t * (x2 - x1), y1 + t * (y2 - y1)))
    return samples


def compute_profile_metrics(
    poly: list[tuple[float, float]],
    hull: list[tuple[float, float]],
    *,
    n_samples: int,
) -> dict[str, float]:
    a = area_abs(poly)
    l = perimeter(poly)
    if a < EPS or l < EPS:
        return {}
    sqrt_a = math.sqrt(a)
    if sqrt_a < EPS:
        return {}

    samples = sample_boundary_by_arclength(poly, n_samples=n_samples)
    if not samples:
        return {}

    gaps = [distance_to_hull_boundary(px, py, hull) for px, py in samples]
    g_rel = [g / sqrt_a for g in gaps]
    m1 = sum(g_rel) / len(g_rel)
    m2 = sum(v * v for v in g_rel) / len(g_rel)
    m3 = sum(v * v * v for v in g_rel) / len(g_rel)

    # Layer-integral deficit proxy:
    # D_tau = (1/A) * ∫_boundary min(d(s), tau*sqrt(A)) ds
    # Integral is approximated by arc-length uniform samples.
    def layer_deficit(tau: float) -> float:
        cutoff = tau * sqrt_a
        mean_capped = sum(min(g, cutoff) for g in gaps) / len(gaps)
        return (l * mean_capped) / a

    return {
        "gap_profile_m1_rel": m1,
        "gap_profile_m2_rel": m2,
        "gap_profile_m3_rel": m3,
        "gap_profile_max_rel": max(g_rel),
        "layer_deficit_tau_005_rel": layer_deficit(0.05),
        "layer_deficit_tau_010_rel": layer_deficit(0.10),
        "layer_deficit_tau_020_rel": layer_deficit(0.20),
    }


def run(input_csv: Path, output_csv: Path, n_samples: int) -> None:
    with input_csv.open("r", encoding="utf-8", newline="") as f:
        rows = list(csv.DictReader(f, delimiter=DELIMITER))
        if not rows:
            raise SystemExit("CSV has no rows")
        fieldnames = list(rows[0].keys())

    new_fields = [
        "gap_profile_m1_rel",
        "gap_profile_m2_rel",
        "gap_profile_m3_rel",
        "gap_profile_max_rel",
        "layer_deficit_tau_005_rel",
        "layer_deficit_tau_010_rel",
        "layer_deficit_tau_020_rel",
    ]
    for col in new_fields:
        if col not in fieldnames:
            fieldnames.append(col)

    base_dir = input_csv.parent
    for row in rows:
        nonconvex_rel = row.get("polygon_nonconvex_rel", "").strip()
        hull_rel = row.get("polygon_convex_rel", "").strip()
        if not nonconvex_rel or not hull_rel:
            continue
        poly_path = (base_dir / nonconvex_rel).resolve()
        hull_path = (base_dir / hull_rel).resolve()
        if not poly_path.exists() or not hull_path.exists():
            continue

        poly = read_polygon(poly_path)
        hull = read_polygon(hull_path)
        if len(poly) < 3 or len(hull) < 3:
            continue

        metrics = compute_profile_metrics(poly, hull, n_samples=n_samples)
        for col in new_fields:
            row[col] = fmt_float(parse_float(str(metrics.get(col))) if col in metrics else None)

    with output_csv.open("w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames, delimiter=DELIMITER)
        w.writeheader()
        w.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--samples",
        type=int,
        default=256,
        help="Boundary arc-length sample count for gap profile (default: 256)",
    )
    args = parser.parse_args()
    run(args.input.resolve(), args.output.resolve(), n_samples=max(64, int(args.samples)))
    print(f"Saved CSV with profile metrics: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
