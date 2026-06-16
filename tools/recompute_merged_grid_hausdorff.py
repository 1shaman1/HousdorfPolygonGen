#!/usr/bin/env python3
"""Recompute only grid_hausdorff column in merged metadata CSV."""

from __future__ import annotations

import argparse
import csv
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass
from pathlib import Path

from enrich_metadata_hausdorff import compute_grid_shor, load_polygon


@dataclass
class Job:
    row_idx: int
    convex_path: str
    nonconvex_path: str


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--metadata", type=Path, required=True)
    ap.add_argument("--raster", type=int, default=80)
    ap.add_argument("--grid", type=int, default=30)
    ap.add_argument("--workers", type=int, default=8)
    return ap.parse_args()


def _worker(args: tuple[Job, int, int]) -> tuple[int, float]:
    job, raster_steps, grid_steps = args
    convex = load_polygon(Path(job.convex_path))
    nonconvex = load_polygon(Path(job.nonconvex_path))
    dist, _x, _y = compute_grid_shor(
        convex,
        nonconvex,
        raster_steps=raster_steps,
        grid_steps=grid_steps,
        inner_workers=1,
    )
    return job.row_idx, dist


def main() -> int:
    args = parse_args()
    meta_path = args.metadata.resolve()
    if not meta_path.is_file():
        raise FileNotFoundError(f"metadata not found: {meta_path}")

    with meta_path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f, delimiter=";")
        fieldnames = list(reader.fieldnames or [])
        rows = list(reader)

    if "grid_hausdorff" not in fieldnames:
        raise ValueError("grid_hausdorff column not found")

    base = meta_path.parent
    jobs: list[Job] = []
    for i, row in enumerate(rows):
        convex_rel = (row.get("polygon_convex_rel") or "").strip()
        nonconvex_rel = (row.get("polygon_nonconvex_rel") or "").strip()
        if not convex_rel or not nonconvex_rel:
            continue
        convex_path = (base / convex_rel).resolve()
        nonconvex_path = (base / nonconvex_rel).resolve()
        if not convex_path.is_file() or not nonconvex_path.is_file():
            continue
        jobs.append(
            Job(
                row_idx=i,
                convex_path=str(convex_path),
                nonconvex_path=str(nonconvex_path),
            )
        )

    total = len(jobs)
    if total == 0:
        print("No valid polygon path pairs found; nothing to update.")
        return 2

    task_args = [(j, args.raster, args.grid) for j in jobs]
    updated = 0
    with ProcessPoolExecutor(max_workers=max(1, args.workers)) as pool:
        for row_idx, dist in pool.map(_worker, task_args, chunksize=2):
            rows[row_idx]["grid_hausdorff"] = f"{dist:.8f}"
            updated += 1
            if updated % 100 == 0:
                print(f"progress {updated}/{total}", flush=True)

    with meta_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, delimiter=";")
        writer.writeheader()
        writer.writerows(rows)

    print(f"done updated={updated} total_rows={len(rows)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
