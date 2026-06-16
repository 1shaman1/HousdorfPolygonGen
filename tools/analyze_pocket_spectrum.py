#!/usr/bin/env python3
"""Analyze pocket-spectrum metrics against best Hausdorff distance."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import numpy as np

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


def rank_average(values: list[float]) -> list[float]:
    order = sorted(range(len(values)), key=lambda i: values[i])
    ranks = [0.0] * len(values)
    i = 0
    while i < len(order):
        j = i
        while j + 1 < len(order) and values[order[j + 1]] == values[order[i]]:
            j += 1
        avg_rank = 0.5 * (i + j) + 1.0
        for k in range(i, j + 1):
            ranks[order[k]] = avg_rank
        i = j + 1
    return ranks


def pearson(x: list[float], y: list[float]) -> float:
    if len(x) < 2 or len(y) < 2:
        return float("nan")
    mx = sum(x) / len(x)
    my = sum(y) / len(y)
    num = sum((a - mx) * (b - my) for a, b in zip(x, y))
    denx = sum((a - mx) ** 2 for a in x)
    deny = sum((b - my) ** 2 for b in y)
    den = math.sqrt(denx * deny)
    if den <= 0.0:
        return float("nan")
    return num / den


def spearman(x: list[float], y: list[float]) -> float:
    if len(x) < 2:
        return float("nan")
    return pearson(rank_average(x), rank_average(y))


def simple_regression_r2(x: list[float], y: list[float]) -> float:
    r = pearson(x, y)
    if not math.isfinite(r):
        return float("nan")
    return r * r


def multi_linear_r2(y: np.ndarray, x_cols: list[np.ndarray]) -> float:
    if y.size < 3:
        return float("nan")
    X = np.column_stack([np.ones(y.size), *x_cols])
    beta, *_ = np.linalg.lstsq(X, y, rcond=None)
    y_hat = X @ beta
    ss_res = float(np.sum((y - y_hat) ** 2))
    ss_tot = float(np.sum((y - y.mean()) ** 2))
    if ss_tot <= 0.0:
        return float("nan")
    return 1.0 - ss_res / ss_tot


def collect(rows: list[dict[str, str]], metric: str) -> tuple[list[float], list[float]]:
    x: list[float] = []
    y: list[float] = []
    for row in rows:
        xv = parse_float(row.get(metric))
        gh = parse_float(row.get("grid_hausdorff"))
        uh = parse_float(row.get("ull_hausdorff"))
        if xv is None or gh is None or uh is None:
            continue
        x.append(xv)
        y.append(min(gh, uh))
    return x, y


def run(input_csv: Path, output_txt: Path) -> None:
    with input_csv.open("r", encoding="utf-8", newline="") as f:
        rows = list(csv.DictReader(f, delimiter=DELIMITER))
    if not rows:
        raise SystemExit("CSV has no rows")

    exact_metrics = [
        "pocket_exact_total_area_rel",
        "pocket_exact_depth_max_rel",
        "pocket_exact_depth_mean_rel",
        "pocket_exact_mouth_mean_rel",
        "pocket_exact_entropy_area",
        "pocket_exact_gini_area",
        "pocket_exact_top1_share",
    ]
    approx_metrics = [
        "pocket_approx_total_area_rel",
        "pocket_approx_depth_max_rel",
        "pocket_approx_depth_mean_rel",
        "pocket_approx_mouth_mean_rel",
        "pocket_approx_entropy_area",
        "pocket_approx_gini_area",
        "pocket_approx_top1_share",
    ]

    lines: list[str] = []
    lines.append("Pocket spectrum analysis (best Hausdorff = min(grid, ull))")
    lines.append(f"rows={len(rows)}")
    lines.append("")

    lines.append("Per-metric univariate statistics:")
    lines.append("metric;pearson;spearman;r2_linear;n")

    all_stats: list[tuple[str, float]] = []
    for metric in exact_metrics + approx_metrics:
        x, y = collect(rows, metric)
        p = pearson(x, y)
        s = spearman(x, y)
        r2 = simple_regression_r2(x, y)
        lines.append(f"{metric};{p:.6f};{s:.6f};{r2:.6f};{len(x)}")
        all_stats.append((metric, abs(p)))

    lines.append("")
    lines.append("Top-5 metrics by |pearson|:")
    top5 = sorted(all_stats, key=lambda t: t[1], reverse=True)[:5]
    for name, score in top5:
        lines.append(f"{name};{score:.6f}")

    # Group-level multivariate linear fit.
    y_vals: list[float] = []
    exact_cols = [[] for _ in exact_metrics]
    approx_cols = [[] for _ in approx_metrics]
    for row in rows:
        gh = parse_float(row.get("grid_hausdorff"))
        uh = parse_float(row.get("ull_hausdorff"))
        if gh is None or uh is None:
            continue
        ex = [parse_float(row.get(m)) for m in exact_metrics]
        ap = [parse_float(row.get(m)) for m in approx_metrics]
        if any(v is None for v in ex + ap):
            continue
        y_vals.append(min(gh, uh))
        for i, v in enumerate(ex):
            exact_cols[i].append(v if v is not None else float("nan"))
        for i, v in enumerate(ap):
            approx_cols[i].append(v if v is not None else float("nan"))

    y_arr = np.asarray(y_vals, dtype=float)
    exact_arrs = [np.asarray(col, dtype=float) for col in exact_cols]
    approx_arrs = [np.asarray(col, dtype=float) for col in approx_cols]
    r2_exact = multi_linear_r2(y_arr, exact_arrs)
    r2_approx = multi_linear_r2(y_arr, approx_arrs)
    r2_both = multi_linear_r2(y_arr, exact_arrs + approx_arrs)

    lines.append("")
    lines.append("Group linear models:")
    lines.append(f"R2_exact={r2_exact:.6f}")
    lines.append(f"R2_approx={r2_approx:.6f}")
    lines.append(f"R2_exact_plus_approx={r2_both:.6f}")

    # Exact vs approx paired similarity.
    lines.append("")
    lines.append("Exact vs approx paired metric similarity (pearson):")
    pairs = [
        ("total_area_rel", "pocket_exact_total_area_rel", "pocket_approx_total_area_rel"),
        ("depth_max_rel", "pocket_exact_depth_max_rel", "pocket_approx_depth_max_rel"),
        ("depth_mean_rel", "pocket_exact_depth_mean_rel", "pocket_approx_depth_mean_rel"),
        ("mouth_mean_rel", "pocket_exact_mouth_mean_rel", "pocket_approx_mouth_mean_rel"),
        ("entropy_area", "pocket_exact_entropy_area", "pocket_approx_entropy_area"),
        ("gini_area", "pocket_exact_gini_area", "pocket_approx_gini_area"),
        ("top1_share", "pocket_exact_top1_share", "pocket_approx_top1_share"),
    ]
    for short, ex_m, ap_m in pairs:
        x: list[float] = []
        y: list[float] = []
        for row in rows:
            exv = parse_float(row.get(ex_m))
            apv = parse_float(row.get(ap_m))
            if exv is None or apv is None:
                continue
            x.append(exv)
            y.append(apv)
        sim = pearson(x, y)
        lines.append(f"{short};{sim:.6f};n={len(x)}")

    output_txt.parent.mkdir(parents=True, exist_ok=True)
    output_txt.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(output_txt)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    run(args.input.resolve(), args.output.resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
