"""Plot numeric fields from ESP32 Aero v2 or legacy telemetry logs."""

from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path

import pandas as pd

from parse_ESP32_Log import default_log_path, parse_aero_logs, related_log_paths


def time_to_seconds(value: object) -> float:
    text = str(value).strip()
    if "-" in text and " " in text:
        try:
            return datetime.fromisoformat(text).timestamp()
        except ValueError:
            return float("nan")
    try:
        hours, minutes, seconds, hundredths = map(int, text.split(":"))
        return hours * 3600 + minutes * 60 + seconds + hundredths / 100.0
    except (TypeError, ValueError):
        return float("nan")


def elapsed_seconds(times: pd.Series) -> pd.Series:
    raw = times.map(time_to_seconds)
    result: list[float] = []
    day_offset = 0.0
    previous = float("nan")
    for value in raw:
        if pd.isna(value):
            result.append(float("nan"))
            continue
        adjusted = value + day_offset
        if not pd.isna(previous) and adjusted < previous:
            day_offset += 24.0 * 3600.0
            adjusted = value + day_offset
        result.append(adjusted)
        previous = adjusted
    series = pd.Series(result, index=times.index)
    first = series.dropna()
    return series - (first.iloc[0] if not first.empty else 0.0)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", nargs="?", type=Path, default=default_log_path())
    parser.add_argument("--active-only", action="store_true")
    parser.add_argument(
        "--columns", nargs="+",
        help="Specific numeric columns to plot; defaults to all numeric fields.",
    )
    parser.add_argument("--output", type=Path, default=Path("ESP32_log_plot.png"))
    parser.add_argument("--show", action="store_true", help="Also open an interactive window.")
    args = parser.parse_args()

    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("Error: matplotlib is required (python3 -m pip install matplotlib).")
        return 2

    paths = related_log_paths(args.log.expanduser().resolve(), not args.active_only)
    frame, warnings = parse_aero_logs(paths)
    for warning in warnings:
        print(f"Warning: {warning}")
    if frame.empty:
        print("Error: no valid telemetry records found.")
        return 1

    frame["Elapsed_s"] = elapsed_seconds(frame["Time"])
    available = [
        column for column in frame.columns
        if column not in ("Time", "Elapsed_s") and pd.api.types.is_numeric_dtype(frame[column])
    ]
    columns = args.columns or available
    missing = [column for column in columns if column not in available]
    if missing:
        print("Error: unavailable numeric columns: " + ", ".join(missing))
        print("Available: " + ", ".join(available))
        return 2

    fig, axes = plt.subplots(len(columns), 1, figsize=(12, max(4, 2.2 * len(columns))), sharex=True)
    if len(columns) == 1:
        axes = [axes]
    for axis, column in zip(axes, columns):
        axis.plot(frame["Elapsed_s"], frame[column], linewidth=1.0)
        axis.set_ylabel(column)
        axis.grid(True, alpha=0.3)
    axes[-1].set_xlabel("Elapsed time (s)")
    fig.suptitle("ESP32 Aero Telemetry")
    fig.tight_layout()
    fig.savefig(args.output, dpi=150)
    print(f"Plotted {len(frame)} records to {args.output.resolve()}")
    if args.show:
        plt.show()
    plt.close(fig)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
