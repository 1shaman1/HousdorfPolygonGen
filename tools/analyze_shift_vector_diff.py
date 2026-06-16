#!/usr/bin/env python3
"""Analyze shift-vector differences for rows where ULL beats grid+shor."""

from __future__ import annotations

import argparse
import csv
import math
import statistics
from pathlib import Path

DELIMITER = ";"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--metadata",
        type=Path,
        required=True,
        help="Path to metadata CSV (semicolon-separated).",
    )
    parser.add_argument(
        "--export-csv",
        type=Path,
        default=None,
        help=(
            "Optional CSV export path for rows where ULL is better. "
            "Exports only vector-shift deltas."
        ),
    )
    return parser.parse_args()


def parse_float(row: dict[str, str], key: str) -> float | None:
    raw = (row.get(key) or "").strip()
    if not raw:
        return None
    try:
        return float(raw)
    except ValueError:
        return None


def percentile(sorted_values: list[float], q: float) -> float:
    if not sorted_values:
        return float("nan")
    if len(sorted_values) == 1:
        return sorted_values[0]
    pos = (len(sorted_values) - 1) * q
    left = int(math.floor(pos))
    right = int(math.ceil(pos))
    if left == right:
        return sorted_values[left]
    weight = pos - left
    return sorted_values[left] * (1.0 - weight) + sorted_values[right] * weight


def format_stats(name: str, values: list[float]) -> list[str]:
    vals = sorted(values)
    mean = statistics.fmean(vals)
    median = statistics.median(vals)
    stdev = statistics.pstdev(vals) if len(vals) > 1 else 0.0
    p05 = percentile(vals, 0.05)
    p95 = percentile(vals, 0.95)
    return [
        f"{name}:",
        f"  count={len(vals)}",
        f"  mean={mean:.6f}",
        f"  median={median:.6f}",
        f"  std={stdev:.6f}",
        f"  min={vals[0]:.6f}",
        f"  p05={p05:.6f}",
        f"  p95={p95:.6f}",
        f"  max={vals[-1]:.6f}",
    ]


def export_deltas(path: Path, rows: list[dict[str, str | float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = ("merged_case_id", "delta_shift_x", "delta_shift_y", "delta_shift_norm")
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, delimiter=DELIMITER, lineterminator="\n")
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {
                    "merged_case_id": row["merged_case_id"],
                    "delta_shift_x": f"{row['delta_shift_x']:.8f}",
                    "delta_shift_y": f"{row['delta_shift_y']:.8f}",
                    "delta_shift_norm": f"{row['delta_shift_norm']:.8f}",
                }
            )


def main() -> int:
    args = parse_args()
    metadata_path = args.metadata.expanduser().resolve()
    if not metadata_path.is_file():
        print(f"metadata file not found: {metadata_path}")
        return 2

    total_rows = 0
    usable_rows = 0
    better_rows = 0
    dx_values: list[float] = []
    dy_values: list[float] = []
    norm_values: list[float] = []
    export_rows: list[dict[str, str | float]] = []

    with metadata_path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f, delimiter=DELIMITER)
        for row in reader:
            total_rows += 1
            grid_h = parse_float(row, "grid_hausdorff")
            ull_h = parse_float(row, "ull_hausdorff")
            grid_x = parse_float(row, "grid_shift_x")
            grid_y = parse_float(row, "grid_shift_y")
            ull_x = parse_float(row, "ull_shift_x")
            ull_y = parse_float(row, "ull_shift_y")
            if None in (grid_h, ull_h, grid_x, grid_y, ull_x, ull_y):
                continue
            usable_rows += 1
            if ull_h >= grid_h:
                continue

            dx = ull_x - grid_x
            dy = ull_y - grid_y
            dn = math.hypot(dx, dy)
            dx_values.append(dx)
            dy_values.append(dy)
            norm_values.append(dn)
            export_rows.append(
                {
                    "merged_case_id": (row.get("merged_case_id") or row.get("case_id") or "").strip(),
                    "delta_shift_x": dx,
                    "delta_shift_y": dy,
                    "delta_shift_norm": dn,
                }
            )
            better_rows += 1

    print(f"metadata: {metadata_path}")
    print(f"rows_total={total_rows}")
    print(f"rows_with_all_needed_fields={usable_rows}")
    print(f"rows_where_ull_better_than_grid={better_rows}")

    if better_rows == 0:
        print("No rows where ull_hausdorff < grid_hausdorff.")
        return 0

    print()
    print("Delta shift vector definition:")
    print("  d = (ull_shift_x - grid_shift_x, ull_shift_y - grid_shift_y)")
    print("  |d| = sqrt(dx^2 + dy^2)")
    print()
    for line in format_stats("dx", dx_values):
        print(line)
    print()
    for line in format_stats("dy", dy_values):
        print(line)
    print()
    for line in format_stats("|d|", norm_values):
        print(line)

    if args.export_csv is not None:
        export_path = args.export_csv.expanduser().resolve()
        export_deltas(export_path, export_rows)
        print()
        print(f"Exported delta vectors: {export_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
