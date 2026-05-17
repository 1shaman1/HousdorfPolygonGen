#!/usr/bin/env python3
"""Join metadata.csv with benchmark CSV and plot metric vs relative ULL error."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt


def read_csv(path: Path, delimiter: str = ";") -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f, delimiter=delimiter))


def rel_err_from_meta(row: dict[str, str]) -> float | None:
    if row.get("ull_vs_grid_rel_err"):
        return float(row["ull_vs_grid_rel_err"])
    grid_h = row.get("grid_hausdorff", "")
    ull_h = row.get("ull_hausdorff", "")
    if grid_h and ull_h:
        f_ref = float(grid_h)
        f_m = float(ull_h)
        if abs(f_ref) < 1e-15:
            return 0.0
        return abs(f_m - f_ref) / abs(f_ref)
    return None


def rel_err(row: dict[str, str]) -> float:
    meta_err = rel_err_from_meta(row)
    if meta_err is not None:
        return meta_err
    for key in ("rel_err", "relative_error"):
        if key in row and row[key]:
            return float(row[key])
    f_ref = float(row.get("f_ref", row.get("f_grid", 0)))
    f_m = float(row.get("f_method", row.get("f_ull", 0)))
    if abs(f_ref) < 1e-15:
        return 0.0
    return abs(f_m - f_ref) / abs(f_ref)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--run-dir", type=Path, required=True, help="run_YYYYMMDD_HHMMSS folder")
    ap.add_argument("--benchmark-csv", type=Path, required=True)
    ap.add_argument(
        "--metric",
        default="depth_rel_actual",
        help="column from metadata (e.g. depth_rel_actual, alpha_lebedev_full)",
    )
    ap.add_argument("--out", type=Path, default=None, help="PNG output path")
    args = ap.parse_args()

    meta_path = args.run_dir / "metadata.csv"
    meta = {r["case_id"]: r for r in read_csv(meta_path)}
    bench = read_csv(args.benchmark_csv)

    xs: list[float] = []
    ys: list[float] = []
    for row in bench:
        cid = row.get("case_id", "")
        if cid not in meta:
            continue
        if args.metric not in meta[cid]:
            continue
        xs.append(float(meta[cid][args.metric]))
        ys.append(rel_err(row))

    if not xs:
        raise SystemExit("No joined rows — check case_id and metric column names")

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.scatter(xs, ys, alpha=0.65, s=24)
    ax.set_xlabel(args.metric)
    ax.set_ylabel("|f_ULL - f_ref| / f_ref")
    ax.set_title(f"ULL error vs {args.metric}")
    ax.grid(True, alpha=0.3)

    out = args.out or (args.run_dir / f"ull_vs_{args.metric}.png")
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    print(f"Wrote {out} ({len(xs)} points)")


if __name__ == "__main__":
    main()
