#!/usr/bin/env python3
"""Plot scatter and histogram for shift-vector deltas."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt

DELIMITER = ";"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input-csv",
        type=Path,
        required=True,
        help="CSV with delta_shift_x, delta_shift_y, delta_shift_norm (delimiter ';').",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        required=True,
        help="Output directory for PNG files.",
    )
    return parser.parse_args()


def read_values(path: Path) -> tuple[list[float], list[float], list[float]]:
    dx_values: list[float] = []
    dy_values: list[float] = []
    norm_values: list[float] = []
    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f, delimiter=DELIMITER)
        for row in reader:
            dx = (row.get("delta_shift_x") or "").strip()
            dy = (row.get("delta_shift_y") or "").strip()
            dn = (row.get("delta_shift_norm") or "").strip()
            if not dx or not dy or not dn:
                continue
            try:
                dx_values.append(float(dx))
                dy_values.append(float(dy))
                norm_values.append(float(dn))
            except ValueError:
                continue
    return dx_values, dy_values, norm_values


def plot_scatter(dx_values: list[float], dy_values: list[float], out_path: Path) -> None:
    fig, ax = plt.subplots(figsize=(7, 6))
    ax.scatter(dx_values, dy_values, s=20, alpha=0.65)
    ax.axhline(0.0, color="gray", linewidth=1.0, alpha=0.8)
    ax.axvline(0.0, color="gray", linewidth=1.0, alpha=0.8)
    ax.set_title("Shift Delta Scatter (ULL better than grid+shor)")
    ax.set_xlabel("delta_shift_x")
    ax.set_ylabel("delta_shift_y")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_hist(norm_values: list[float], out_path: Path) -> None:
    fig, ax = plt.subplots(figsize=(7, 5))
    ax.hist(norm_values, bins=30, alpha=0.85)
    ax.set_title("Histogram of |delta_shift| (ULL better than grid+shor)")
    ax.set_xlabel("delta_shift_norm")
    ax.set_ylabel("count")
    ax.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def main() -> int:
    args = parse_args()
    input_csv = args.input_csv.expanduser().resolve()
    out_dir = args.out_dir.expanduser().resolve()
    if not input_csv.is_file():
        print(f"input CSV not found: {input_csv}")
        return 2

    dx_values, dy_values, norm_values = read_values(input_csv)
    if not dx_values:
        print("No valid delta rows found in input CSV.")
        return 3

    out_dir.mkdir(parents=True, exist_ok=True)
    scatter_path = out_dir / "shift_delta_scatter.png"
    hist_path = out_dir / "shift_delta_norm_hist.png"

    plot_scatter(dx_values, dy_values, scatter_path)
    plot_hist(norm_values, hist_path)

    print(f"Wrote {scatter_path}")
    print(f"Wrote {hist_path}")
    print(f"points={len(dx_values)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
