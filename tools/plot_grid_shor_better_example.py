#!/usr/bin/env python3
"""Build one illustrative plot where grid+shor beats ULL."""

from __future__ import annotations

import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def load_polygon(path: Path) -> np.ndarray:
    lines = path.read_text(encoding="utf-8").strip().splitlines()
    n = int(lines[0].strip())
    points = np.array(
        [[float(a), float(b)] for a, b, *_ in (line.split() for line in lines[1 : n + 1])],
        dtype=float,
    )
    return np.vstack([points, points[:1]])


def main() -> int:
    repo_root = Path(__file__).resolve().parents[2]
    meta_path = repo_root / "HousdorfPolygonGen" / "merged" / "metadata_3runs_fixed_shor.csv"
    out_path = (
        repo_root
        / "HousdorfPolygonGen"
        / "merged"
        / "plots_best_hausdorff_fixed_shor_requested_metrics"
        / "grid_shor_better_example_random_s42_n8.png"
    )

    with meta_path.open(encoding="utf-8", newline="") as f:
        rows = list(csv.DictReader(f, delimiter=";"))

    row = next(r for r in rows if r["case_id"] == "random_s42_n8")
    if float(row["grid_hausdorff"]) >= float(row["ull_hausdorff"]):
        raise RuntimeError("Chosen case does not satisfy grid+shor < ull")

    case_dir = (meta_path.parent / row["source_run_rel"] / row["case_id"]).resolve()
    poly_nonconvex = load_polygon(case_dir / f"{row['case_id']}_polygon_nonconvex.txt")
    poly_convex = load_polygon(case_dir / f"{row['case_id']}_polygon_convex.txt")

    tx = float(row["grid_shift_x"])
    ty = float(row["grid_shift_y"])
    poly_shifted = poly_nonconvex.copy()
    poly_shifted[:, 0] += tx
    poly_shifted[:, 1] += ty

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.fill(poly_nonconvex[:, 0], poly_nonconvex[:, 1], color="#8ecae6", alpha=0.30)
    ax.plot(
        poly_nonconvex[:, 0],
        poly_nonconvex[:, 1],
        color="#1f77b4",
        lw=2.0,
        label="P",
    )
    ax.plot(
        poly_convex[:, 0],
        poly_convex[:, 1],
        color="#d62728",
        lw=2.0,
        ls="-",
        label="P0 (conv)",
    )
    ax.plot(
        poly_shifted[:, 0],
        poly_shifted[:, 1],
        color="#2ca02c",
        lw=2.0,
        ls="--",
        label="Optimal position P",
    )

    cx = float(poly_nonconvex[:-1, 0].mean())
    cy = float(poly_nonconvex[:-1, 1].mean())
    ax.arrow(
        cx,
        cy,
        tx,
        ty,
        width=2.5,
        head_width=25,
        head_length=30,
        length_includes_head=True,
        color="#2ca02c",
        alpha=0.75,
        label="Shift vector t",
    )
    ax.text(
        cx + tx * 0.55,
        cy + ty * 0.55,
        f"t = ({tx:.2f}, {ty:.2f})",
        color="#2ca02c",
        fontsize=10,
        bbox={"boxstyle": "round,pad=0.2", "fc": "white", "ec": "#2ca02c", "alpha": 0.8},
    )

    ax.set_title(
        f"Кейс {row['case_id']}: grid+shor={float(row['grid_hausdorff']):.2f} < "
        f"ull={float(row['ull_hausdorff']):.2f}",
        fontsize=11,
    )
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=180)
    plt.close(fig)

    print(out_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
