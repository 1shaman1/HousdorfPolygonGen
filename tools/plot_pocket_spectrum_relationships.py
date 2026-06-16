#!/usr/bin/env python3
"""Build summary plots for pocket-spectrum relationships."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt
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
        avg = 0.5 * (i + j) + 1.0
        for k in range(i, j + 1):
            ranks[order[k]] = avg
        i = j + 1
    return ranks


def pearson(x: list[float], y: list[float]) -> float:
    if len(x) < 2:
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
    return pearson(rank_average(x), rank_average(y))


def collect_xy(rows: list[dict[str, str]], metric: str) -> tuple[list[float], list[float]]:
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


def calc_stats(rows: list[dict[str, str]], metrics: list[str]) -> list[tuple[str, float, float]]:
    out: list[tuple[str, float, float]] = []
    for m in metrics:
        x, y = collect_xy(rows, m)
        out.append((m, pearson(x, y), spearman(x, y)))
    return out


def plot_corr_bars(stats: list[tuple[str, float, float]], out_path: Path) -> None:
    ordered = sorted(stats, key=lambda t: abs(t[1]), reverse=True)
    labels = [x[0] for x in ordered]
    pvals = [x[1] for x in ordered]
    svals = [x[2] for x in ordered]

    idx = np.arange(len(labels))
    w = 0.38
    fig, ax = plt.subplots(figsize=(14, 6))
    ax.bar(idx - w / 2, pvals, width=w, label="Pearson", color="#2563eb")
    ax.bar(idx + w / 2, svals, width=w, label="Spearman", color="#dc2626")
    ax.axhline(0.0, color="#374151", linewidth=1.0, alpha=0.7)
    ax.set_xticks(idx)
    ax.set_xticklabels(labels, rotation=30, ha="right")
    ax.set_ylabel("Correlation with best_hausdorff")
    ax.set_title("Pocket metrics: Pearson vs Spearman")
    ax.legend()
    ax.grid(True, axis="y", alpha=0.25)
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=170, bbox_inches="tight")
    plt.close(fig)


def plot_exact_approx_similarity(rows: list[dict[str, str]], out_path: Path) -> None:
    pairs = [
        ("total_area_rel", "pocket_exact_total_area_rel", "pocket_approx_total_area_rel"),
        ("depth_max_rel", "pocket_exact_depth_max_rel", "pocket_approx_depth_max_rel"),
        ("depth_mean_rel", "pocket_exact_depth_mean_rel", "pocket_approx_depth_mean_rel"),
        ("mouth_mean_rel", "pocket_exact_mouth_mean_rel", "pocket_approx_mouth_mean_rel"),
        ("entropy_area", "pocket_exact_entropy_area", "pocket_approx_entropy_area"),
        ("gini_area", "pocket_exact_gini_area", "pocket_approx_gini_area"),
        ("top1_share", "pocket_exact_top1_share", "pocket_approx_top1_share"),
    ]

    labels: list[str] = []
    sims: list[float] = []
    for short, ex_col, ap_col in pairs:
        x: list[float] = []
        y: list[float] = []
        for row in rows:
            exv = parse_float(row.get(ex_col))
            apv = parse_float(row.get(ap_col))
            if exv is None or apv is None:
                continue
            x.append(exv)
            y.append(apv)
        labels.append(short)
        sims.append(pearson(x, y))

    fig, ax = plt.subplots(figsize=(10, 5))
    colors = ["#059669" if v >= 0.9 else "#0ea5e9" if v >= 0.7 else "#f59e0b" for v in sims]
    bars = ax.bar(labels, sims, color=colors)
    ax.set_ylim(0.0, 1.02)
    ax.set_ylabel("Pearson(exact, approx)")
    ax.set_title("Exact vs Approx feature similarity")
    ax.grid(True, axis="y", alpha=0.25)
    for b, v in zip(bars, sims):
        ax.text(b.get_x() + b.get_width() / 2, v + 0.015, f"{v:.3f}", ha="center", va="bottom", fontsize=9)
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=170, bbox_inches="tight")
    plt.close(fig)


def r2_for_group(rows: list[dict[str, str]], cols: list[str]) -> float:
    x_rows: list[list[float]] = []
    y_vals: list[float] = []
    for row in rows:
        gh = parse_float(row.get("grid_hausdorff"))
        uh = parse_float(row.get("ull_hausdorff"))
        if gh is None or uh is None:
            continue
        vals = [parse_float(row.get(c)) for c in cols]
        if any(v is None for v in vals):
            continue
        x_rows.append([float(v) for v in vals if v is not None])
        y_vals.append(min(gh, uh))

    if len(y_vals) < 3:
        return float("nan")
    X = np.asarray(x_rows, dtype=float)
    y = np.asarray(y_vals, dtype=float)
    x_design = np.column_stack([np.ones(X.shape[0]), X])
    beta, *_ = np.linalg.lstsq(x_design, y, rcond=None)
    y_hat = x_design @ beta
    ss_res = float(np.sum((y - y_hat) ** 2))
    ss_tot = float(np.sum((y - y.mean()) ** 2))
    if ss_tot <= 0.0:
        return float("nan")
    return 1.0 - ss_res / ss_tot


def plot_group_r2(rows: list[dict[str, str]], out_path: Path) -> None:
    exact_cols = [
        "pocket_exact_total_area_rel",
        "pocket_exact_depth_max_rel",
        "pocket_exact_depth_mean_rel",
        "pocket_exact_mouth_mean_rel",
        "pocket_exact_entropy_area",
        "pocket_exact_gini_area",
        "pocket_exact_top1_share",
    ]
    approx_cols = [
        "pocket_approx_total_area_rel",
        "pocket_approx_depth_max_rel",
        "pocket_approx_depth_mean_rel",
        "pocket_approx_mouth_mean_rel",
        "pocket_approx_entropy_area",
        "pocket_approx_gini_area",
        "pocket_approx_top1_share",
    ]
    vals = [
        r2_for_group(rows, exact_cols),
        r2_for_group(rows, approx_cols),
        r2_for_group(rows, exact_cols + approx_cols),
    ]
    labels = ["Exact", "Approx", "Exact+Approx"]

    fig, ax = plt.subplots(figsize=(7, 4.5))
    bars = ax.bar(labels, vals, color=["#2563eb", "#f59e0b", "#059669"])
    ax.set_ylim(0.0, 1.0)
    ax.set_ylabel("R²")
    ax.set_title("Group linear models for best_hausdorff")
    ax.grid(True, axis="y", alpha=0.25)
    for b, v in zip(bars, vals):
        ax.text(b.get_x() + b.get_width() / 2, v + 0.02, f"{v:.3f}", ha="center", va="bottom", fontsize=10)
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=170, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--metadata", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    args = parser.parse_args()

    with args.metadata.open("r", encoding="utf-8", newline="") as f:
        rows = list(csv.DictReader(f, delimiter=DELIMITER))
    if not rows:
        raise SystemExit("CSV has no rows")

    metrics = [
        "pocket_exact_total_area_rel",
        "pocket_exact_depth_max_rel",
        "pocket_exact_depth_mean_rel",
        "pocket_exact_mouth_mean_rel",
        "pocket_exact_entropy_area",
        "pocket_exact_gini_area",
        "pocket_exact_top1_share",
        "pocket_approx_total_area_rel",
        "pocket_approx_depth_max_rel",
        "pocket_approx_depth_mean_rel",
        "pocket_approx_mouth_mean_rel",
        "pocket_approx_entropy_area",
        "pocket_approx_gini_area",
        "pocket_approx_top1_share",
    ]
    stats = calc_stats(rows, metrics)
    out_dir = args.out_dir.resolve()
    plot_corr_bars(stats, out_dir / "pocket_corr_pearson_spearman.png")
    plot_exact_approx_similarity(rows, out_dir / "pocket_exact_vs_approx_similarity.png")
    plot_group_r2(rows, out_dir / "pocket_group_r2_models.png")

    print(f"Saved summary plots to: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
