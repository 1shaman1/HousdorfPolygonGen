#!/usr/bin/env python3
"""Analyze requested metrics vs best Hausdorff distance for fixed_shor metadata."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.stats import pearsonr, spearmanr

DELIMITER = ";"

# requested_name -> source column in metadata
METRIC_MAP = {
    "depth_rel": "depth_rel_actual",
    "bridge_width_rel": "bridge_width_rel_actual",
    "pocket_width_rel": "pocket_width_rel_actual",
    "area_ratio": "area_ratio_actual",
    "alpha_lebedev": "alpha_lebedev_full",
    "angular_mass": "angular_mass",
    "angular_entropy": "angular_entropy",
    "gap_profile_m1_rel": "gap_profile_m1_rel",
    "gap_profile_m2_rel": "gap_profile_m2_rel",
    "gap_profile_m3_rel": "gap_profile_m3_rel",
    "gap_profile_max_rel": "gap_profile_max_rel",
    "layer_deficit_tau_005_rel": "layer_deficit_tau_005_rel",
    "layer_deficit_tau_010_rel": "layer_deficit_tau_010_rel",
    "layer_deficit_tau_020_rel": "layer_deficit_tau_020_rel",
}

INTEGRAL_METRICS = [
    "angular_mass",
    "angular_entropy",
    "gap_profile_m1_rel",
    "gap_profile_m2_rel",
    "gap_profile_m3_rel",
    "gap_profile_max_rel",
    "layer_deficit_tau_005_rel",
    "layer_deficit_tau_010_rel",
    "layer_deficit_tau_020_rel",
]

CONTOUR_METRICS = INTEGRAL_METRICS + ["alpha_lebedev"]

REQUESTED_HEATMAP_PAIRS = [
    ("area_ratio", "gap_profile_max_rel"),
    ("area_ratio", "layer_deficit_tau_005_rel"),
    ("area_ratio", "angular_mass"),
    ("gap_profile_max_rel", "layer_deficit_tau_020_rel"),
    ("area_ratio", "gap_profile_m1_rel"),
    ("area_ratio", "gap_profile_m2_rel"),
    ("area_ratio", "gap_profile_m3_rel"),
    ("depth_rel", "pocket_width_rel"),
    ("depth_rel", "bridge_width_rel"),
]


def ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def build_alias_frame(df: pd.DataFrame) -> pd.DataFrame:
    out = df.copy()
    for alias, src in METRIC_MAP.items():
        out[alias] = pd.to_numeric(out[src], errors="coerce")
    out["grid_hausdorff"] = pd.to_numeric(out["grid_hausdorff"], errors="coerce")
    out["ull_hausdorff"] = pd.to_numeric(out["ull_hausdorff"], errors="coerce")
    out["best_hausdorff"] = np.minimum(out["grid_hausdorff"], out["ull_hausdorff"])
    return out


def plot_scatter(df: pd.DataFrame, metric: str, out_path: Path) -> None:
    data = df[[metric, "best_hausdorff"]].dropna()
    if data.empty:
        return
    x = data[metric].to_numpy()
    y = data["best_hausdorff"].to_numpy()

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.scatter(x, y, s=8, alpha=0.35, color="#1d4ed8", edgecolors="none")
    if len(data) >= 3:
        coef = np.polyfit(x, y, deg=1)
        xx = np.linspace(float(np.min(x)), float(np.max(x)), 200)
        yy = coef[0] * xx + coef[1]
        ax.plot(xx, yy, color="#dc2626", linewidth=2.0, label="linear trend")
        ax.legend(loc="best")
    ax.set_xlabel(metric)
    ax.set_ylabel("best_hausdorff = min(grid, ull)")
    ax.set_title(f"best_hausdorff vs {metric}")
    ax.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(out_path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def plot_contour(df: pd.DataFrame, metric_y: str, out_path: Path, bins: int = 32) -> None:
    data = df[["area_ratio", metric_y, "best_hausdorff"]].dropna()
    if len(data) < 50:
        return
    x = data["area_ratio"].to_numpy()
    y = data[metric_y].to_numpy()
    z = data["best_hausdorff"].to_numpy()

    x_edges = np.linspace(float(np.min(x)), float(np.max(x)), bins + 1)
    y_edges = np.linspace(float(np.min(y)), float(np.max(y)), bins + 1)
    x_idx = np.clip(np.digitize(x, x_edges) - 1, 0, bins - 1)
    y_idx = np.clip(np.digitize(y, y_edges) - 1, 0, bins - 1)

    sums = np.zeros((bins, bins), dtype=float)
    counts = np.zeros((bins, bins), dtype=float)
    for xi, yi, zi in zip(x_idx, y_idx, z):
        sums[yi, xi] += zi
        counts[yi, xi] += 1.0

    with np.errstate(invalid="ignore"):
        mean_z = sums / counts
    mean_z[counts < 5] = np.nan

    xc = 0.5 * (x_edges[:-1] + x_edges[1:])
    yc = 0.5 * (y_edges[:-1] + y_edges[1:])
    X, Y = np.meshgrid(xc, yc)

    fig, ax = plt.subplots(figsize=(8, 5.5))
    contour = ax.contourf(X, Y, mean_z, levels=12, cmap="viridis")
    ax.contour(X, Y, mean_z, levels=12, colors="white", linewidths=0.4, alpha=0.6)
    cbar = fig.colorbar(contour, ax=ax)
    cbar.set_label("mean best_hausdorff")
    ax.set_xlabel("area_ratio")
    ax.set_ylabel(metric_y)
    ax.set_title(f"Contour: best_hausdorff(area_ratio, {metric_y})")
    ax.grid(alpha=0.15)
    fig.tight_layout()
    fig.savefig(out_path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def binned_mean(
    x: np.ndarray,
    y: np.ndarray,
    z: np.ndarray,
    bins: int,
    min_points_per_bin: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    x_min = float(np.min(x))
    x_max = float(np.max(x))
    y_min = float(np.min(y))
    y_max = float(np.max(y))
    if x_min == x_max or y_min == y_max:
        return np.array([]), np.array([]), np.array([[]]), np.array([[]])

    x_edges = np.linspace(x_min, x_max, bins + 1)
    y_edges = np.linspace(y_min, y_max, bins + 1)
    x_idx = np.clip(np.digitize(x, x_edges) - 1, 0, bins - 1)
    y_idx = np.clip(np.digitize(y, y_edges) - 1, 0, bins - 1)

    sums = np.zeros((bins, bins), dtype=float)
    counts = np.zeros((bins, bins), dtype=float)
    for xi, yi, zi in zip(x_idx, y_idx, z):
        sums[yi, xi] += zi
        counts[yi, xi] += 1.0

    with np.errstate(invalid="ignore"):
        mean_z = sums / counts
    mean_z[counts < min_points_per_bin] = np.nan
    return x_edges, y_edges, mean_z, counts


def plot_heatmap_pair(
    df: pd.DataFrame,
    metric_x: str,
    metric_y: str,
    out_path: Path,
    bins: int = 32,
    min_points_per_bin: int = 5,
) -> dict[str, object]:
    data = df[[metric_x, metric_y, "best_hausdorff"]].dropna()
    report: dict[str, object] = {
        "metric_x": metric_x,
        "metric_y": metric_y,
        "n": int(len(data)),
        "occupied_bins": 0,
        "plotted_bins": 0,
        "status": "skipped",
        "path": str(out_path),
    }
    if len(data) < 50:
        report["status"] = "too_few_points"
        return report

    x = data[metric_x].to_numpy()
    y = data[metric_y].to_numpy()
    z = data["best_hausdorff"].to_numpy()
    x_edges, y_edges, mean_z, counts = binned_mean(x, y, z, bins, min_points_per_bin)
    if x_edges.size == 0 or y_edges.size == 0:
        report["status"] = "constant_axis"
        return report

    occupied_bins = int(np.sum(counts > 0))
    plotted_bins = int(np.sum(np.isfinite(mean_z)))
    report["occupied_bins"] = occupied_bins
    report["plotted_bins"] = plotted_bins
    if plotted_bins == 0:
        report["status"] = "no_bins_after_filter"
        return report

    fig, ax = plt.subplots(figsize=(8, 5.5))
    mesh = ax.pcolormesh(x_edges, y_edges, mean_z, shading="auto", cmap="viridis")
    ax.scatter(x, y, s=5, c="white", alpha=0.12, edgecolors="none")
    cbar = fig.colorbar(mesh, ax=ax)
    cbar.set_label("mean best_hausdorff")
    ax.set_xlabel(metric_x)
    ax.set_ylabel(metric_y)
    ax.set_title(f"Heatmap: best_hausdorff({metric_x}, {metric_y})")
    ax.grid(alpha=0.15)
    fig.tight_layout()
    fig.savefig(out_path, dpi=180, bbox_inches="tight")
    plt.close(fig)

    report["status"] = "saved"
    return report


def plot_z_contour_over_hausdorff(
    df: pd.DataFrame,
    metric_x: str,
    metric_z: str,
    out_path: Path,
    bins: int = 32,
    min_points_per_bin: int = 5,
) -> dict[str, object]:
    data = df[[metric_x, metric_z, "best_hausdorff"]].dropna()
    report: dict[str, object] = {
        "metric_x": metric_x,
        "metric_y": "best_hausdorff",
        "metric_z": metric_z,
        "n": int(len(data)),
        "occupied_bins": 0,
        "plotted_bins": 0,
        "status": "skipped",
        "path": str(out_path),
    }
    if len(data) < 50:
        report["status"] = "too_few_points"
        return report

    x = data[metric_x].to_numpy()
    y = data["best_hausdorff"].to_numpy()
    z = data[metric_z].to_numpy()
    x_edges, y_edges, mean_z, counts = binned_mean(x, y, z, bins, min_points_per_bin)
    if x_edges.size == 0 or y_edges.size == 0:
        report["status"] = "constant_axis"
        return report

    occupied_bins = int(np.sum(counts > 0))
    plotted_bins = int(np.sum(np.isfinite(mean_z)))
    report["occupied_bins"] = occupied_bins
    report["plotted_bins"] = plotted_bins
    if plotted_bins == 0:
        report["status"] = "no_bins_after_filter"
        return report

    xc = 0.5 * (x_edges[:-1] + x_edges[1:])
    yc = 0.5 * (y_edges[:-1] + y_edges[1:])
    X, Y = np.meshgrid(xc, yc)
    z_masked = np.ma.masked_invalid(mean_z)

    fig, ax = plt.subplots(figsize=(8, 5.5))
    filled = ax.contourf(X, Y, z_masked, levels=12, cmap="plasma")
    if plotted_bins >= 4 and float(np.nanmin(mean_z)) < float(np.nanmax(mean_z)):
        lines = ax.contour(X, Y, z_masked, levels=12, colors="white", linewidths=0.45, alpha=0.75)
        ax.clabel(lines, inline=True, fontsize=7, fmt="%.3g")
    ax.scatter(x, y, s=5, c="black", alpha=0.10, edgecolors="none")
    cbar = fig.colorbar(filled, ax=ax)
    cbar.set_label(f"mean {metric_z}")
    ax.set_xlabel(metric_x)
    ax.set_ylabel("best_hausdorff = min(grid, ull)")
    ax.set_title(f"Contour: {metric_z}({metric_x}, best_hausdorff)")
    ax.grid(alpha=0.15)
    fig.tight_layout()
    fig.savefig(out_path, dpi=180, bbox_inches="tight")
    plt.close(fig)

    report["status"] = "saved"
    return report


def calc_correlations(df: pd.DataFrame) -> pd.DataFrame:
    rows = []
    for metric in METRIC_MAP:
        sub = df[[metric, "best_hausdorff"]].dropna()
        if len(sub) < 3:
            rows.append(
                {
                    "metric": metric,
                    "n": int(len(sub)),
                    "pearson_r": np.nan,
                    "pearson_pvalue": np.nan,
                    "spearman_rho": np.nan,
                    "spearman_pvalue": np.nan,
                }
            )
            continue
        pr, pp = pearsonr(sub[metric], sub["best_hausdorff"])
        sr, sp = spearmanr(sub[metric], sub["best_hausdorff"])
        rows.append(
            {
                "metric": metric,
                "n": int(len(sub)),
                "pearson_r": float(pr),
                "pearson_pvalue": float(pp),
                "spearman_rho": float(sr),
                "spearman_pvalue": float(sp),
            }
        )
    out = pd.DataFrame(rows)
    out["abs_pearson"] = out["pearson_r"].abs()
    out["abs_spearman"] = out["spearman_rho"].abs()
    return out.sort_values(["abs_pearson", "abs_spearman"], ascending=False)


def write_presence_report(df: pd.DataFrame, out_path: Path) -> None:
    lines = []
    lines.append("Requested metrics presence/completeness check")
    lines.append(f"rows_total={len(df)}")
    lines.append("")
    for alias, src in METRIC_MAP.items():
        present = src in df.columns
        non_null = int(df[src].notna().sum()) if present else 0
        lines.append(f"{alias}: source={src}; present={present}; non_null={non_null}")
    out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_heatmap_report(reports: list[dict[str, object]], out_path: Path) -> None:
    pd.DataFrame(reports).to_csv(out_path, sep=DELIMITER, index=False)


def write_z_contour_report(reports: list[dict[str, object]], out_path: Path) -> None:
    pd.DataFrame(reports).to_csv(out_path, sep=DELIMITER, index=False)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True, help="Input metadata CSV with all metrics")
    parser.add_argument("--out-dir", type=Path, required=True, help="Output directory for plots and reports")
    parser.add_argument("--heatmap-bins", type=int, default=32, help="Number of bins per axis for requested heatmaps")
    parser.add_argument(
        "--heatmap-min-points",
        type=int,
        default=5,
        help="Minimum points per bin for requested heatmaps",
    )
    args = parser.parse_args()

    out_dir = args.out_dir.resolve()
    ensure_dir(out_dir)
    ensure_dir(out_dir / "scatter")
    ensure_dir(out_dir / "contour_integral_vs_area_ratio")
    ensure_dir(out_dir / "heatmap_requested_metric_pairs")
    ensure_dir(out_dir / "contour_requested_z_over_hausdorff")

    df_src = pd.read_csv(args.input, sep=DELIMITER)
    write_presence_report(df_src, out_dir / "requested_metrics_presence.txt")
    df = build_alias_frame(df_src)
    df.to_csv(out_dir / "metadata_3runs_fixed_shor_all_metrics_with_aliases.csv", sep=DELIMITER, index=False)

    corr = calc_correlations(df)
    corr.to_csv(out_dir / "correlations_pearson_spearman.csv", sep=DELIMITER, index=False)

    for metric in METRIC_MAP:
        plot_scatter(df, metric, out_dir / "scatter" / f"best_hausdorff__{metric}.png")

    for metric in CONTOUR_METRICS:
        plot_contour(
            df,
            metric,
            out_dir / "contour_integral_vs_area_ratio" / f"best_hausdorff_contour__area_ratio__{metric}.png",
        )

    heatmap_reports = []
    for metric_x, metric_y in REQUESTED_HEATMAP_PAIRS:
        heatmap_reports.append(
            plot_heatmap_pair(
                df,
                metric_x,
                metric_y,
                out_dir / "heatmap_requested_metric_pairs" / f"best_hausdorff_heatmap__{metric_x}__{metric_y}.png",
                bins=args.heatmap_bins,
                min_points_per_bin=args.heatmap_min_points,
            )
        )
    write_heatmap_report(heatmap_reports, out_dir / "requested_heatmap_pairs_report.csv")

    z_contour_reports = []
    for metric_x, metric_z in REQUESTED_HEATMAP_PAIRS:
        z_contour_reports.append(
            plot_z_contour_over_hausdorff(
                df,
                metric_x,
                metric_z,
                out_dir
                / "contour_requested_z_over_hausdorff"
                / f"{metric_z}_contour__x_{metric_x}__y_best_hausdorff.png",
                bins=args.heatmap_bins,
                min_points_per_bin=args.heatmap_min_points,
            )
        )
    write_z_contour_report(z_contour_reports, out_dir / "requested_z_contour_over_hausdorff_report.csv")

    print(f"Saved analysis outputs to: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
