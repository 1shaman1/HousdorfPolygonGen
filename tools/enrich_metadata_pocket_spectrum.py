#!/usr/bin/env python3
"""Enrich metadata with pocket-spectrum features (exact and approximate)."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

from shapely.geometry import MultiPolygon, Polygon
from shapely.validation import make_valid

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


def fmt_float(value: float | None) -> str:
    if value is None or not math.isfinite(value):
        return ""
    return f"{value:.8f}"


def safe_polygon(coords: list[tuple[float, float]]) -> Polygon | None:
    if len(coords) < 3:
        return None
    poly = Polygon(coords)
    if poly.is_empty:
        return None
    if not poly.is_valid:
        poly = make_valid(poly)
    if poly.is_empty:
        return None
    if isinstance(poly, MultiPolygon):
        poly = max(poly.geoms, key=lambda g: g.area)
    if not isinstance(poly, Polygon):
        return None
    return poly


def shapely_polygon_components(geom) -> list[Polygon]:
    if geom.is_empty:
        return []
    if isinstance(geom, Polygon):
        return [geom]
    if isinstance(geom, MultiPolygon):
        return [g for g in geom.geoms if g.area > EPS]
    if hasattr(geom, "geoms"):
        out: list[Polygon] = []
        for g in geom.geoms:
            out.extend(shapely_polygon_components(g))
        return out
    return []


def entropy(vals: list[float]) -> float:
    positive = [v for v in vals if v > EPS]
    total = sum(positive)
    if total < EPS:
        return 0.0
    h = 0.0
    for v in positive:
        p = v / total
        h -= p * math.log(p)
    return h


def gini(vals: list[float]) -> float:
    positive = sorted(v for v in vals if v > EPS)
    n = len(positive)
    if n == 0:
        return 0.0
    total = sum(positive)
    if total < EPS:
        return 0.0
    weighted = sum((i + 1) * v for i, v in enumerate(positive))
    return (2.0 * weighted) / (n * total) - (n + 1) / n


def nearest_point_on_hull(x: float, y: float, hull: list[tuple[float, float]]) -> tuple[float, float]:
    best_d2 = float("inf")
    best = hull[0]
    px, py = x, y
    n = len(hull)
    for i in range(n):
        ax, ay = hull[i]
        bx, by = hull[(i + 1) % n]
        abx = bx - ax
        aby = by - ay
        ab2 = abx * abx + aby * aby
        if ab2 < EPS:
            qx, qy = ax, ay
        else:
            t = ((px - ax) * abx + (py - ay) * aby) / ab2
            t = max(0.0, min(1.0, t))
            qx = ax + t * abx
            qy = ay + t * aby
        d2 = (px - qx) ** 2 + (py - qy) ** 2
        if d2 < best_d2:
            best_d2 = d2
            best = (qx, qy)
    return best


def sample_boundary(poly: list[tuple[float, float]], samples: int) -> tuple[list[tuple[float, float]], float]:
    n = len(poly)
    if n < 2:
        return [], 0.0
    lengths: list[float] = []
    total = 0.0
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        ll = math.hypot(x2 - x1, y2 - y1)
        lengths.append(ll)
        total += ll
    if total < EPS:
        return [], 0.0

    out: list[tuple[float, float]] = []
    step = total / samples
    edge_i = 0
    edge_start = 0.0
    edge_end = lengths[0]
    for k in range(samples):
        s = (k + 0.5) * step
        while s > edge_end and edge_i < n - 1:
            edge_i += 1
            edge_start = edge_end
            edge_end += lengths[edge_i]
        x1, y1 = poly[edge_i]
        x2, y2 = poly[(edge_i + 1) % n]
        ll = lengths[edge_i]
        if ll < EPS:
            out.append((x1, y1))
        else:
            t = (s - edge_start) / ll
            out.append((x1 + t * (x2 - x1), y1 + t * (y2 - y1)))
    return out, total


def circular_runs(mask: list[bool]) -> list[tuple[int, int]]:
    n = len(mask)
    if n == 0 or not any(mask):
        return []
    runs: list[tuple[int, int]] = []
    i = 0
    while i < n:
        if not mask[i]:
            i += 1
            continue
        start = i
        while i + 1 < n and mask[i + 1]:
            i += 1
        end = i
        runs.append((start, end))
        i += 1
    if len(runs) > 1 and mask[0] and mask[-1]:
        _, e0 = runs[0]
        s1, _ = runs[-1]
        runs = [(s1, e0 + n)] + runs[1:-1]
    return runs


def aggregate_spectrum(
    *,
    areas: list[float],
    depths: list[float],
    mouths: list[float],
    a_poly: float,
    sqrt_a: float,
    prefix: str,
) -> dict[str, float]:
    out: dict[str, float] = {
        f"{prefix}_count": float(len(areas)),
        f"{prefix}_total_area_rel": 0.0,
        f"{prefix}_depth_max_rel": 0.0,
        f"{prefix}_depth_mean_rel": 0.0,
        f"{prefix}_mouth_max_rel": 0.0,
        f"{prefix}_mouth_mean_rel": 0.0,
        f"{prefix}_entropy_area": 0.0,
        f"{prefix}_gini_area": 0.0,
        f"{prefix}_top1_share": 0.0,
    }
    if not areas:
        return out

    total_area = sum(areas)
    out[f"{prefix}_total_area_rel"] = total_area / a_poly if a_poly > EPS else 0.0
    out[f"{prefix}_depth_max_rel"] = max(depths) / sqrt_a if sqrt_a > EPS else 0.0
    out[f"{prefix}_depth_mean_rel"] = (
        sum(d * a for d, a in zip(depths, areas)) / total_area / sqrt_a if total_area > EPS and sqrt_a > EPS else 0.0
    )
    out[f"{prefix}_mouth_max_rel"] = max(mouths) / sqrt_a if mouths and sqrt_a > EPS else 0.0
    out[f"{prefix}_mouth_mean_rel"] = (
        sum(m * a for m, a in zip(mouths, areas)) / total_area / sqrt_a if total_area > EPS and sqrt_a > EPS else 0.0
    )
    out[f"{prefix}_entropy_area"] = entropy(areas)
    out[f"{prefix}_gini_area"] = gini(areas)
    out[f"{prefix}_top1_share"] = max(areas) / total_area if total_area > EPS else 0.0
    return out


def exact_spectrum(
    hull_poly: Polygon,
    poly: Polygon,
    *,
    sample_perimeter_points: int,
    a_poly: float,
    sqrt_a: float,
) -> dict[str, float]:
    gap = hull_poly.difference(poly)
    components = shapely_polygon_components(gap)

    hull_boundary = hull_poly.boundary
    areas: list[float] = []
    depths: list[float] = []
    mouths: list[float] = []

    for comp in components:
        if comp.area < EPS:
            continue
        areas.append(comp.area)
        mouth_len = comp.boundary.intersection(hull_boundary).length
        mouths.append(float(mouth_len))

        ext = comp.exterior
        step = max(8, sample_perimeter_points)
        max_depth = 0.0
        for i in range(step):
            pt = ext.interpolate(i / step, normalized=True)
            d = pt.distance(hull_boundary)
            if d > max_depth:
                max_depth = d
        depths.append(max_depth)

    return aggregate_spectrum(
        areas=areas,
        depths=depths,
        mouths=mouths,
        a_poly=a_poly,
        sqrt_a=sqrt_a,
        prefix="pocket_exact",
    )


def approx_spectrum(
    poly_pts: list[tuple[float, float]],
    hull_pts: list[tuple[float, float]],
    *,
    samples: int,
    eps_rel: float,
    a_poly: float,
    sqrt_a: float,
) -> dict[str, float]:
    boundary_samples, perimeter_total = sample_boundary(poly_pts, samples)
    if not boundary_samples or perimeter_total < EPS or sqrt_a < EPS:
        return aggregate_spectrum(
            areas=[],
            depths=[],
            mouths=[],
            a_poly=a_poly,
            sqrt_a=sqrt_a,
            prefix="pocket_approx",
        )

    ds = perimeter_total / len(boundary_samples)
    gaps = []
    for x, y in boundary_samples:
        qx, qy = nearest_point_on_hull(x, y, hull_pts)
        gaps.append(math.hypot(x - qx, y - qy))

    threshold = eps_rel * sqrt_a
    mask = [d > threshold for d in gaps]
    runs = circular_runs(mask)

    areas: list[float] = []
    depths: list[float] = []
    mouths: list[float] = []
    n = len(boundary_samples)

    for start, end in runs:
        idxs = [i % n for i in range(start, end + 1)]
        seg_gaps = [gaps[i] for i in idxs]
        if not seg_gaps:
            continue
        area_proxy = sum(seg_gaps) * ds
        areas.append(area_proxy)
        depths.append(max(seg_gaps))

        x1, y1 = boundary_samples[idxs[0]]
        x2, y2 = boundary_samples[idxs[-1]]
        q1x, q1y = nearest_point_on_hull(x1, y1, hull_pts)
        q2x, q2y = nearest_point_on_hull(x2, y2, hull_pts)
        mouths.append(math.hypot(q2x - q1x, q2y - q1y))

    return aggregate_spectrum(
        areas=areas,
        depths=depths,
        mouths=mouths,
        a_poly=a_poly,
        sqrt_a=sqrt_a,
        prefix="pocket_approx",
    )


def run(input_csv: Path, output_csv: Path, *, samples: int, eps_rel: float) -> None:
    with input_csv.open("r", encoding="utf-8", newline="") as f:
        rows = list(csv.DictReader(f, delimiter=DELIMITER))
        if not rows:
            raise SystemExit("CSV has no rows")
        fieldnames = list(rows[0].keys())

    new_cols = [
        "pocket_exact_count",
        "pocket_exact_total_area_rel",
        "pocket_exact_depth_max_rel",
        "pocket_exact_depth_mean_rel",
        "pocket_exact_mouth_max_rel",
        "pocket_exact_mouth_mean_rel",
        "pocket_exact_entropy_area",
        "pocket_exact_gini_area",
        "pocket_exact_top1_share",
        "pocket_approx_count",
        "pocket_approx_total_area_rel",
        "pocket_approx_depth_max_rel",
        "pocket_approx_depth_mean_rel",
        "pocket_approx_mouth_max_rel",
        "pocket_approx_mouth_mean_rel",
        "pocket_approx_entropy_area",
        "pocket_approx_gini_area",
        "pocket_approx_top1_share",
    ]
    for col in new_cols:
        if col not in fieldnames:
            fieldnames.append(col)

    base_dir = input_csv.parent
    for row in rows:
        poly_rel = row.get("polygon_nonconvex_rel", "").strip()
        hull_rel = row.get("polygon_convex_rel", "").strip()
        if not poly_rel or not hull_rel:
            continue

        poly_path = (base_dir / poly_rel).resolve()
        hull_path = (base_dir / hull_rel).resolve()
        if not poly_path.exists() or not hull_path.exists():
            continue

        poly_pts = read_polygon(poly_path)
        hull_pts = read_polygon(hull_path)
        poly_shape = safe_polygon(poly_pts)
        hull_shape = safe_polygon(hull_pts)
        if poly_shape is None or hull_shape is None:
            continue

        a_poly = poly_shape.area
        sqrt_a = math.sqrt(a_poly) if a_poly > EPS else 0.0
        if a_poly < EPS or sqrt_a < EPS:
            continue

        exact = exact_spectrum(
            hull_shape,
            poly_shape,
            sample_perimeter_points=max(32, samples // 4),
            a_poly=a_poly,
            sqrt_a=sqrt_a,
        )
        approx = approx_spectrum(
            poly_pts,
            hull_pts,
            samples=samples,
            eps_rel=eps_rel,
            a_poly=a_poly,
            sqrt_a=sqrt_a,
        )
        merged = {**exact, **approx}
        for col in new_cols:
            row[col] = fmt_float(merged.get(col))

    with output_csv.open("w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames, delimiter=DELIMITER)
        w.writeheader()
        w.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--samples", type=int, default=512, help="Boundary samples for approx pockets")
    parser.add_argument("--eps-rel", type=float, default=0.01, help="Relative threshold for d(s) > eps*sqrt(A)")
    args = parser.parse_args()
    run(
        args.input.resolve(),
        args.output.resolve(),
        samples=max(128, int(args.samples)),
        eps_rel=max(0.0, float(args.eps_rel)),
    )
    print(f"Saved CSV with pocket spectrum: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
