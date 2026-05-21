#!/usr/bin/env python3
"""Временный прогон enrich на подмножестве кейсов (junction + замер wall-time)."""

from __future__ import annotations

import argparse
import csv
import shutil
import subprocess
import sys
import time
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent


def prepare_bench_dir(source: Path, bench: Path, limit: int) -> int:
    meta_src = source / "metadata.csv"
    if not meta_src.is_file():
        raise FileNotFoundError(meta_src)

    if bench.exists():
        shutil.rmtree(bench)
    bench.mkdir(parents=True)

    with meta_src.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f, delimiter=";")
        fieldnames = reader.fieldnames
        if not fieldnames:
            raise ValueError("empty metadata")
        rows = []
        for row in reader:
            rows.append(row)
            if len(rows) >= limit:
                break

    meta_dst = bench / "metadata.csv"
    with meta_dst.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, delimiter=";", lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)

    linked = 0
    for row in rows:
        cid = row.get("case_id", "").strip()
        if not cid:
            continue
        src_case = source / cid
        if not src_case.is_dir():
            print(f"skip missing case dir: {cid}", file=sys.stderr)
            continue
        dst_case = bench / cid
        dst_case.mkdir(exist_ok=True)
        for name in src_case.iterdir():
            target = dst_case / name.name
            if target.exists():
                continue
            target.symlink_to(name.resolve(), target_is_directory=name.is_dir())
        linked += 1

    return linked


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", type=Path, required=True)
    ap.add_argument("--bench-dir", type=Path, required=True)
    ap.add_argument("--limit", type=int, default=300)
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--raster", type=int, default=80)
    ap.add_argument("--grid", type=int, default=30)
    args = ap.parse_args()

    source = args.source.resolve()
    bench = args.bench_dir.resolve()
    n = prepare_bench_dir(source, bench, args.limit)
    print(f"Bench dir: {bench} ({n} cases linked)")

    enrich_script = SCRIPT_DIR / "enrich_metadata_hausdorff.py"
    cmd = [
        sys.executable,
        str(enrich_script),
        "--run-dir",
        str(bench),
        "--raster",
        str(args.raster),
        "--grid",
        str(args.grid),
        "--workers",
        str(args.workers),
    ]
    print("Command:", " ".join(cmd))
    t0 = time.perf_counter()
    proc = subprocess.run(cmd, check=False)
    elapsed = time.perf_counter() - t0
    rc = proc.returncode
    print(f"Wall time: {elapsed:.2f} s ({elapsed / max(n, 1):.3f} s/case)")
    print(f"Exit code: {rc}")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
