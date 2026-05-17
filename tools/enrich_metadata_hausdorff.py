#!/usr/bin/env python3
"""Дополняет metadata.csv: grid+Shor и УЛЛ (shift, optimal Hausdorff)."""

from __future__ import annotations

import argparse
import csv
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

HOSUDORF_ROOT = Path(__file__).resolve().parents[2]
GRID_SRC = HOSUDORF_ROOT / "grid_plus_shor" / "src"
ULL_SRC = HOSUDORF_ROOT / "multi_step_method" / "src"

for p in (GRID_SRC, ULL_SRC):
    s = str(p)
    if s not in sys.path:
        sys.path.insert(0, s)

from scipy.spatial import cKDTree  # noqa: E402

from geometry import Point as GridPoint  # noqa: E402
from hausdorff_grid_search import find_optimal_translation_grid  # noqa: E402
from polygon_hausdorff_fast import default_workers, to_array  # noqa: E402
from polygon_rasterization import rasterize_polygon  # noqa: E402
from q0_init import initQ0  # noqa: E402
from shor_optimize import shor_optimize  # noqa: E402

from geometry import Point as UllPoint  # noqa: E402  # type: ignore[no-redef]
from polygon_fast import PolygonPair, default_workers as ull_default_workers  # noqa: E402
from solver import solve  # noqa: E402

HAUSDORFF_COLUMNS = (
    "grid_hausdorff",
    "grid_shift_x",
    "grid_shift_y",
    "ull_hausdorff",
    "ull_shift_x",
    "ull_shift_y",
    "ull_vs_grid_rel_err",
)


@dataclass
class HausdorffResult:
    grid_hausdorff: float
    grid_shift_x: float
    grid_shift_y: float
    ull_hausdorff: float
    ull_shift_x: float
    ull_shift_y: float
    ull_vs_grid_rel_err: float


def load_polygon(path: Path) -> list[GridPoint]:
    lines = path.read_text(encoding="utf-8").strip().splitlines()
    n = int(lines[0].strip())
    points: list[GridPoint] = []
    for line in lines[1 : n + 1]:
        parts = line.split()
        if len(parts) < 2:
            continue
        points.append(GridPoint(float(parts[0]), float(parts[1])))
    if len(points) != n:
        raise ValueError(f"{path}: ожидалось {n} вершин, прочитано {len(points)}")
    return points


def to_ull_points(poly: list[GridPoint]) -> list[UllPoint]:
    return [UllPoint(p.x, p.y) for p in poly]


def compute_grid_shor(
    convex: list[GridPoint],
    nonconvex: list[GridPoint],
    *,
    raster_steps: int,
    grid_steps: int,
    inner_workers: int | None,
) -> tuple[float, float, float]:
    a_r = rasterize_polygon(convex, raster_steps, workers=inner_workers)
    b_r = rasterize_polygon(nonconvex, raster_steps, workers=inner_workers)
    q0 = initQ0(convex, nonconvex)
    a_tree = cKDTree(to_array(a_r))
    b_tree = cKDTree(to_array(b_r))
    _best_x, _grid_val, _ = find_optimal_translation_grid(
        a_r, b_r, a_tree, b_tree, q0, grid_steps
    )
    refine_x, refine_dist = shor_optimize(a_r, b_r, a_tree, b_tree, _best_x)
    return refine_dist, refine_x.x, refine_x.y


def compute_ull(convex: list[GridPoint], nonconvex: list[GridPoint]) -> tuple[float, float, float]:
    a = to_ull_points(convex)
    b = to_ull_points(nonconvex)
    pair = PolygonPair.from_polygons(a, b)
    dist, shift, _status = solve(a, b, pair=pair)
    return dist, shift.x, shift.y


def process_case(
    run_dir: Path,
    case_id: str,
    *,
    raster_steps: int,
    grid_steps: int,
    inner_workers: int | None,
) -> Optional[HausdorffResult]:
    case_dir = run_dir / case_id
    convex_path = case_dir / f"{case_id}_polygon_convex.txt"
    nonconvex_path = case_dir / f"{case_id}_polygon_nonconvex.txt"
    if not convex_path.is_file() or not nonconvex_path.is_file():
        return None

    convex = load_polygon(convex_path)
    nonconvex = load_polygon(nonconvex_path)

    grid_h, grid_x, grid_y = compute_grid_shor(
        convex,
        nonconvex,
        raster_steps=raster_steps,
        grid_steps=grid_steps,
        inner_workers=inner_workers,
    )
    ull_h, ull_x, ull_y = compute_ull(convex, nonconvex)

    rel_err = 0.0
    if abs(grid_h) > 1e-15:
        rel_err = abs(ull_h - grid_h) / abs(grid_h)

    return HausdorffResult(
        grid_hausdorff=grid_h,
        grid_shift_x=grid_x,
        grid_shift_y=grid_y,
        ull_hausdorff=ull_h,
        ull_shift_x=ull_x,
        ull_shift_y=ull_y,
        ull_vs_grid_rel_err=rel_err,
    )


def _process_case_worker(
    args: tuple[str, str, int, int, int | None],
) -> tuple[str, Optional[HausdorffResult]]:
    run_dir_s, case_id, raster_steps, grid_steps, inner_workers = args
    if not case_id:
        return "", None
    return case_id, process_case(
        Path(run_dir_s),
        case_id,
        raster_steps=raster_steps,
        grid_steps=grid_steps,
        inner_workers=inner_workers,
    )


def format_row_values(res: HausdorffResult) -> dict[str, str]:
    return {
        "grid_hausdorff": f"{res.grid_hausdorff:.8f}",
        "grid_shift_x": f"{res.grid_shift_x:.8f}",
        "grid_shift_y": f"{res.grid_shift_y:.8f}",
        "ull_hausdorff": f"{res.ull_hausdorff:.8f}",
        "ull_shift_x": f"{res.ull_shift_x:.8f}",
        "ull_shift_y": f"{res.ull_shift_y:.8f}",
        "ull_vs_grid_rel_err": f"{res.ull_vs_grid_rel_err:.8f}",
    }


def enrich_metadata(
    run_dir: Path,
    *,
    raster_steps: int,
    grid_steps: int,
    workers: int | None,
) -> int:
    meta_path = run_dir / "metadata.csv"
    if not meta_path.is_file():
        print(f"metadata.csv not found: {meta_path}", file=sys.stderr)
        return 1

    with meta_path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f, delimiter=";")
        if not reader.fieldnames:
            print("metadata.csv is empty", file=sys.stderr)
            return 1
        fieldnames = list(reader.fieldnames)
        rows = list(reader)

    for col in HAUSDORFF_COLUMNS:
        if col not in fieldnames:
            fieldnames.append(col)

    case_ids = [r.get("case_id", "").strip() for r in rows if r.get("case_id")]
    n_workers = workers if workers is not None else max(ull_default_workers(), default_workers())
    n_workers = max(1, n_workers)

    case_parallel = n_workers > 1 and len(case_ids) > 1
    inner_workers = 1 if case_parallel else workers
    run_dir_s = str(run_dir.resolve())
    task_args = [
        (run_dir_s, cid, raster_steps, grid_steps, inner_workers) for cid in case_ids
    ]

    results: dict[str, HausdorffResult] = {}
    if not case_parallel:
        for args in task_args:
            key, res = _process_case_worker(args)
            if res is not None:
                results[key] = res
    else:
        chunksize = max(1, len(task_args) // (n_workers * 4))
        with ProcessPoolExecutor(max_workers=n_workers) as pool:
            for key, res in pool.map(
                _process_case_worker,
                task_args,
                chunksize=chunksize,
            ):
                if res is not None:
                    results[key] = res

    updated = 0
    for row in rows:
        cid = row.get("case_id", "").strip()
        res = results.get(cid)
        if res is None:
            continue
        row.update(format_row_values(res))
        updated += 1

    with meta_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, delimiter=";", lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)

    print(f"Updated {updated}/{len(rows)} rows in {meta_path}")
    return 0 if updated > 0 else 2


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--run-dir", type=Path, required=True)
    ap.add_argument("--raster", type=int, default=50)
    ap.add_argument("--grid", type=int, default=20)
    ap.add_argument("--workers", type=int, default=None)
    args = ap.parse_args()
    return enrich_metadata(
        args.run_dir.resolve(),
        raster_steps=args.raster,
        grid_steps=args.grid,
        workers=args.workers,
    )


if __name__ == "__main__":
    raise SystemExit(main())
