#!/usr/bin/env python3
"""PNG/SVG превью пар P0/P из run_dir (каждый N-й кейс из metadata.csv)."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def load_polygon(path: Path) -> np.ndarray:
    lines = path.read_text(encoding="utf-8").strip().splitlines()
    n = int(lines[0].strip())
    pts = []
    for line in lines[1 : n + 1]:
        parts = line.split()
        if len(parts) >= 2:
            pts.append((float(parts[0]), float(parts[1])))
    if len(pts) != n:
        raise ValueError(f"{path}: expected {n} vertices, got {len(pts)}")
    arr = np.asarray(pts, dtype=float)
    return np.vstack([arr, arr[:1]])


def shift_poly(poly: np.ndarray, dx: float, dy: float) -> np.ndarray:
    out = poly.copy()
    out[:, 0] += dx
    out[:, 1] += dy
    return out


def plot_pair(
    case_id: str,
    p0: np.ndarray,
    p: np.ndarray,
    out_path: Path,
    *,
    grid_shift: tuple[float, float] | None = None,
    ull_shift: tuple[float, float] | None = None,
    grid_h: float | None = None,
    ull_h: float | None = None,
    dpi: int = 120,
) -> None:
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.fill(p[:, 0], p[:, 1], color="lightblue", alpha=0.35, label="P")
    ax.plot(p0[:, 0], p0[:, 1], color="crimson", lw=2, label="P₀ (conv)")
    ax.plot(p[:, 0], p[:, 1], color="steelblue", lw=1.2)

    if grid_shift is not None:
        pg = shift_poly(p, *grid_shift)
        ax.plot(pg[:, 0], pg[:, 1], color="forestgreen", ls="--", lw=1.2, label="P + grid shift")
    if ull_shift is not None:
        pu = shift_poly(p, *ull_shift)
        ax.plot(pu[:, 0], pu[:, 1], color="darkorange", ls=":", lw=1.4, label="P + ULL shift")

    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    title = case_id
    if grid_h is not None:
        title += f"\ngrid H={grid_h:.2f}"
    if ull_h is not None:
        title += f", ull H={ull_h:.2f}"
    ax.set_title(title, fontsize=10)
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=dpi, bbox_inches="tight")
    plt.close(fig)


def save_svg(case_id: str, p0: np.ndarray, p: np.ndarray, out_path: Path) -> None:
    """Тот же стиль, что savePreviewSvg в polygon_io.cpp."""
    p_open = p[:-1]
    p0_open = p0[:-1]
    pts_p = " ".join(f"{x:.6f},{y:.6f}" for x, y in p_open)
    pts_p0 = " ".join(f"{x:.6f},{y:.6f}" for x, y in p0_open)
    body = (
        "<svg xmlns='http://www.w3.org/2000/svg' "
        "width='1200' height='1200' viewBox='-50 -50 1100 1100'>\n"
        f"<polygon points='{pts_p}' fill='lightblue' fill-opacity='0.3' stroke='blue'/>\n"
        f"<polygon points='{pts_p0}' fill='none' stroke='red' stroke-width='2'/>\n"
        "</svg>\n"
    )
    out_path.write_text(body, encoding="utf-8")


def _float(row: dict, key: str) -> float | None:
    v = row.get(key, "").strip()
    if not v:
        return None
    return float(v)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--run-dir", type=Path, required=True)
    ap.add_argument("--every", type=int, default=10, help="каждый N-й кейс (1 = все)")
    ap.add_argument("--out-dir", type=Path, default=None)
    ap.add_argument("--format", choices=("png", "svg", "both"), default="png")
    ap.add_argument("--show-shifts", action="store_true", default=True)
    ap.add_argument("--no-shifts", action="store_true")
    args = ap.parse_args()

    run_dir = args.run_dir.resolve()
    meta_path = run_dir / "metadata.csv"
    if not meta_path.is_file():
        print(f"not found: {meta_path}", file=__import__("sys").stderr)
        return 1

    out_dir = (args.out_dir or (run_dir / f"previews_every_{args.every}")).resolve()
    show_shifts = args.show_shifts and not args.no_shifts

    with meta_path.open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f, delimiter=";"))

    every = max(1, args.every)
    selected = rows[::every]
    n_ok = 0
    for row in selected:
        cid = row.get("case_id", "").strip()
        if not cid:
            continue
        case_dir = run_dir / cid
        p0_path = case_dir / f"{cid}_polygon_convex.txt"
        p_path = case_dir / f"{cid}_polygon_nonconvex.txt"
        if not p0_path.is_file() or not p_path.is_file():
            print(f"skip missing files: {cid}", file=__import__("sys").stderr)
            continue
        p0 = load_polygon(p0_path)
        p = load_polygon(p_path)

        grid_shift = ull_shift = None
        if show_shifts:
            gx = _float(row, "grid_shift_x")
            gy = _float(row, "grid_shift_y")
            if gx is not None and gy is not None:
                grid_shift = (gx, gy)
            ux = _float(row, "ull_shift_x")
            uy = _float(row, "ull_shift_y")
            if ux is not None and uy is not None:
                ull_shift = (ux, uy)

        if args.format in ("png", "both"):
            plot_pair(
                cid,
                p0,
                p,
                out_dir / f"{cid}.png",
                grid_shift=grid_shift,
                ull_shift=ull_shift,
                grid_h=_float(row, "grid_hausdorff"),
                ull_h=_float(row, "ull_hausdorff"),
            )
        if args.format in ("svg", "both"):
            save_svg(cid, p0, p, out_dir / f"{cid}.svg")

        n_ok += 1

    print(f"Wrote {n_ok} previews to {out_dir} (every {every}, from {len(rows)} cases)")
    return 0 if n_ok > 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
