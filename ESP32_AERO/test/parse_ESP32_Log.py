"""Parse and inspect ESP32 Aero pipe-delimited telemetry logs.

Supports the current ``aero_log_v2.txt`` schema, its rotated
``aero_log_v2.old.txt`` companion, and legacy ``aero_log.txt`` files.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Iterable

import pandas as pd


V2_COLUMNS = [
    "Time", "Airspeed", "AirDensity", "Temperature", "Pressure",
    "Altitude", "AltitudeChange", "GroundSpeed", "ClimbRate_mps",
    "ForwardAccel_mps2", "AccelX_g", "AccelY_g", "AccelZ_g",
]
LEGACY_COLUMNS = [
    "Time", "Airspeed", "AirDensity", "Temperature", "Pressure",
    "Altitude", "AltitudeChange", "GroundSpeed",
]


def default_log_path() -> Path:
    base = Path(__file__).resolve().parent
    for name in ("aero_log_v2.txt", "aero_log.txt"):
        candidate = base / name
        if candidate.exists():
            return candidate
    return base / "aero_log_v2.txt"


def related_log_paths(path: Path, include_rotated: bool = True) -> list[Path]:
    """Return logs in chronological order (rotated first, active second)."""
    paths: list[Path] = []
    if include_rotated and path.name == "aero_log_v2.txt":
        rotated = path.with_name("aero_log_v2.old.txt")
        if rotated.exists():
            paths.append(rotated)
    paths.append(path)
    return paths


def _clean_fields(line: str) -> list[str]:
    line = line.strip()
    if line.endswith("*"):
        line = line[:-1]
    fields = [field.strip() for field in line.split("|")]
    while fields and fields[-1] == "":
        fields.pop()
    return fields


def parse_aero_logs(paths: Iterable[Path]) -> tuple[pd.DataFrame, list[str]]:
    records: list[dict[str, str]] = []
    warnings: list[str] = []

    for path in paths:
        if not path.exists():
            warnings.append(f"Missing file: {path}")
            continue

        columns: list[str] | None = None
        with path.open("r", encoding="utf-8", errors="replace") as handle:
            for line_number, raw_line in enumerate(handle, start=1):
                if not raw_line.strip():
                    continue
                fields = _clean_fields(raw_line)
                if fields and fields[0] == "Time":
                    columns = fields
                    continue

                if columns is None:
                    if len(fields) == len(V2_COLUMNS):
                        columns = V2_COLUMNS
                    elif len(fields) == len(LEGACY_COLUMNS):
                        columns = LEGACY_COLUMNS
                    else:
                        warnings.append(
                            f"{path.name}:{line_number}: cannot infer schema "
                            f"from {len(fields)} fields"
                        )
                        continue

                if len(fields) != len(columns):
                    warnings.append(
                        f"{path.name}:{line_number}: expected {len(columns)} "
                        f"fields, found {len(fields)}; record skipped"
                    )
                    continue

                records.append(dict(zip(columns, fields)))

    frame = pd.DataFrame.from_records(records)
    if not frame.empty:
        for column in frame.columns:
            if column != "Time":
                frame[column] = pd.to_numeric(frame[column], errors="coerce")
    return frame, warnings


def parse_aero_log(file_path: str | Path, include_rotated: bool = True) -> pd.DataFrame:
    path = Path(file_path).expanduser().resolve()
    frame, warnings = parse_aero_logs(related_log_paths(path, include_rotated))
    for warning in warnings:
        print(f"Warning: {warning}", file=__import__("sys").stderr)
    return frame


def print_records(frame: pd.DataFrame) -> None:
    if frame.empty:
        print("No valid telemetry records found.")
        return
    padding = max(len(str(column)) for column in frame.columns) + 1
    for _, row in frame.iterrows():
        print("********")
        for column in frame.columns:
            value = row[column]
            rendered = "NA" if pd.isna(value) else str(value)
            print(f"{column:<{padding}}: {rendered}")
    print("********")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", nargs="?", type=Path, default=default_log_path())
    parser.add_argument(
        "--active-only", action="store_true",
        help="Do not prepend aero_log_v2.old.txt when parsing a v2 log.",
    )
    parser.add_argument(
        "--summary-only", action="store_true",
        help="Print schema and record counts instead of every record.",
    )
    args = parser.parse_args()

    paths = related_log_paths(args.log.expanduser().resolve(), not args.active_only)
    frame, warnings = parse_aero_logs(paths)
    print("Parsing:", ", ".join(str(path) for path in paths))
    print(f"Valid records: {len(frame)} | Malformed/missing: {len(warnings)}")
    print("Columns:", ", ".join(frame.columns) if len(frame.columns) else "none")
    for warning in warnings:
        print(f"Warning: {warning}")
    if not args.summary_only:
        print_records(frame)
    return 0 if not frame.empty else 1


if __name__ == "__main__":
    raise SystemExit(main())
