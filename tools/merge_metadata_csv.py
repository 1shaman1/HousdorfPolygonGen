#!/usr/bin/env python3
"""Merge multiple metadata.csv files and attach relative polygon paths."""

from __future__ import annotations

import argparse
import csv
import os
import sys
from pathlib import Path

DELIMITER = ";"
REQUIRED_COLUMNS = ("case_id",)
ADDED_COLUMNS = (
    "source_metadata_rel",
    "source_run_rel",
    "merged_case_id",
    "polygon_convex_rel",
    "polygon_nonconvex_rel",
)


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--metadata",
        type=Path,
        nargs="+",
        required=True,
        help="Paths to source metadata.csv files.",
    )
    ap.add_argument(
        "--output",
        type=Path,
        required=True,
        help="Path to merged metadata CSV.",
    )
    ap.add_argument(
        "--strict-schema",
        action="store_true",
        help="Fail when source schemas are different (without it union schema is used).",
    )
    ap.add_argument(
        "--allow-duplicate-merged-case-id",
        action="store_true",
        help="Allow duplicate merged_case_id values (not recommended).",
    )
    return ap.parse_args()


def normalize(path: Path) -> Path:
    return path.expanduser().resolve()


def relpath_or_abs(path: Path, base: Path) -> str:
    try:
        rel = os.path.relpath(path.resolve(), start=base.resolve())
        return Path(rel).as_posix()
    except ValueError:
        return path.resolve().as_posix()


def ensure_required_columns(fieldnames: list[str], metadata_path: Path) -> None:
    missing = [c for c in REQUIRED_COLUMNS if c not in fieldnames]
    if missing:
        joined = ", ".join(missing)
        raise ValueError(f"{metadata_path}: missing required column(s): {joined}")


def ensure_schema(
    source_fieldnames: list[str],
    metadata_path: Path,
    merged_fieldnames: list[str],
    *,
    strict_schema: bool,
) -> None:
    if strict_schema and merged_fieldnames and source_fieldnames != merged_fieldnames:
        raise ValueError(
            f"{metadata_path}: schema differs from first source while --strict-schema is enabled"
        )
    for name in source_fieldnames:
        if name not in merged_fieldnames:
            merged_fieldnames.append(name)


def build_augmented_row(
    row: dict[str, str],
    *,
    case_id: str,
    run_dir: Path,
    metadata_path: Path,
    output_dir: Path,
) -> dict[str, str]:
    merged_case_id = f"{run_dir.name}/{case_id}"
    case_dir = run_dir / case_id
    convex_file = case_dir / f"{case_id}_polygon_convex.txt"
    nonconvex_file = case_dir / f"{case_id}_polygon_nonconvex.txt"

    row["source_metadata_rel"] = relpath_or_abs(metadata_path, output_dir)
    row["source_run_rel"] = relpath_or_abs(run_dir, output_dir)
    row["merged_case_id"] = merged_case_id
    row["polygon_convex_rel"] = relpath_or_abs(convex_file, output_dir)
    row["polygon_nonconvex_rel"] = relpath_or_abs(nonconvex_file, output_dir)
    return row


def ensure_unique_case_id(
    merged_case_id: str,
    seen_merged_case_ids: set[str],
    *,
    allow_duplicate_merged_case_id: bool,
) -> None:
    if not allow_duplicate_merged_case_id and merged_case_id in seen_merged_case_ids:
        raise ValueError(
            f"Duplicate merged_case_id '{merged_case_id}'. "
            "Use --allow-duplicate-merged-case-id if this is expected."
        )
    seen_merged_case_ids.add(merged_case_id)


def merge(
    metadata_paths: list[Path],
    output_path: Path,
    *,
    strict_schema: bool,
    allow_duplicate_merged_case_id: bool,
) -> tuple[int, int]:
    output_dir = output_path.parent.resolve()
    rows_out: list[dict[str, str]] = []
    merged_fieldnames: list[str] = []
    seen_merged_case_ids: set[str] = set()
    source_schemas: list[list[str]] = []

    for metadata_path in metadata_paths:
        with metadata_path.open("r", newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f, delimiter=DELIMITER)
            source_fieldnames = list(reader.fieldnames or [])
            if not source_fieldnames:
                continue
            ensure_required_columns(source_fieldnames, metadata_path)
            source_schemas.append(source_fieldnames)
            ensure_schema(
                source_fieldnames,
                metadata_path,
                merged_fieldnames,
                strict_schema=strict_schema,
            )

            run_dir = metadata_path.parent
            for row in reader:
                case_id = (row.get("case_id") or "").strip()
                if not case_id:
                    continue

                merged_case_id = f"{run_dir.name}/{case_id}"
                ensure_unique_case_id(
                    merged_case_id,
                    seen_merged_case_ids,
                    allow_duplicate_merged_case_id=allow_duplicate_merged_case_id,
                )
                rows_out.append(
                    build_augmented_row(
                        row,
                        case_id=case_id,
                        run_dir=run_dir,
                        metadata_path=metadata_path,
                        output_dir=output_dir,
                    )
                )

    if not rows_out:
        return 0, 0

    for extra_name in ADDED_COLUMNS:
        if extra_name not in merged_fieldnames:
            merged_fieldnames.append(extra_name)

    with output_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=merged_fieldnames,
            delimiter=DELIMITER,
            lineterminator="\n",
            extrasaction="ignore",
        )
        writer.writeheader()
        for row in rows_out:
            writer.writerow({name: row.get(name, "") for name in merged_fieldnames})

    return len(source_schemas), len(rows_out)


def main() -> int:
    args = parse_args()
    output_path = normalize(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    metadata_paths = [normalize(p) for p in args.metadata]
    missing = [p for p in metadata_paths if not p.is_file()]
    if missing:
        for p in missing:
            print(f"metadata file not found: {p}", file=sys.stderr)
        return 2

    try:
        source_count, row_count = merge(
            metadata_paths,
            output_path,
            strict_schema=args.strict_schema,
            allow_duplicate_merged_case_id=args.allow_duplicate_merged_case_id,
        )
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    if row_count == 0:
        print("No rows merged (empty inputs or no valid case_id rows).", file=sys.stderr)
        return 3

    print(
        f"Merged {row_count} rows from {source_count} file(s) into {output_path}",
        file=sys.stdout,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
