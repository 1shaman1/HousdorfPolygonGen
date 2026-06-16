#!/usr/bin/env python3
"""Add pseudo coefficient k from H = k * area_ratio^power."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

DELIMITER = ";"


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


def run(input_csv: Path, output_csv: Path, power: float, column_name: str) -> None:
    with input_csv.open("r", encoding="utf-8", newline="") as f:
        rows = list(csv.DictReader(f, delimiter=DELIMITER))
        if not rows:
            raise SystemExit("CSV has no rows")
        fieldnames = list(rows[0].keys())

    if column_name not in fieldnames:
        fieldnames.append(column_name)

    for row in rows:
        area_ratio = parse_float(row.get("area_ratio_actual"))
        grid_h = parse_float(row.get("grid_hausdorff"))
        ull_h = parse_float(row.get("ull_hausdorff"))

        if (
            area_ratio is None
            or area_ratio <= 0.0
            or grid_h is None
            or ull_h is None
        ):
            row[column_name] = ""
            continue

        best_h = min(grid_h, ull_h)
        denom = area_ratio**power
        if denom <= 0.0:
            row[column_name] = ""
            continue

        k_value = best_h / denom
        row[column_name] = fmt_float(k_value)

    with output_csv.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, delimiter=DELIMITER)
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--power",
        type=float,
        required=True,
        help="Power in denominator: k = H / (area_ratio^power)",
    )
    parser.add_argument(
        "--column-name",
        type=str,
        required=True,
        help="Output column name for pseudo coefficient k",
    )
    args = parser.parse_args()
    run(
        args.input.resolve(),
        args.output.resolve(),
        power=args.power,
        column_name=args.column_name.strip(),
    )
    print(f"Saved CSV with pseudo coefficient: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
