#!/usr/bin/env python3
"""Analyze STM32 PID/PI motor logs and plot step responses.

Usage:
    python pid_log_analyzer.py                      # file dialog
    python pid_log_analyzer.py logs/test.csv
    python pid_log_analyzer.py logs/*.csv --save-dir graphs
    python pid_log_analyzer.py logs/test.csv --tolerance 5 --deadline-ms 2000 --hold-ms 500

Requires: pandas, numpy, matplotlib. tkinter is optional for the file dialog.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

REQUIRED = ("time_ms", "target_rpm", "measured_rpm")
OPTIONAL = ("error", "pwm", "control_output", "pwm_command")


@dataclass
class Analysis:
    name: str
    passed: bool
    pass_time_ms: float | None
    hold_start_ms: float | None
    first_entry_ms: float | None
    rise_90_ms: float | None
    settle_ms: float | None
    peak_rpm: float
    overshoot_pct: float | None
    steady_rpm: float
    steady_bias_rpm: float
    steady_mae_rpm: float
    mean_step_ms: float
    max_step_ms: float
    target_rpm: float
    lower_rpm: float
    upper_rpm: float
    n_samples: int


def choose_files() -> list[Path]:
    try:
        import tkinter as tk
        from tkinter import filedialog

        root = tk.Tk()
        root.withdraw()
        root.attributes('-topmost', True)
        names = filedialog.askopenfilenames(
            title="Select STM32 PID CSV logs",
            filetypes=[("CSV logs", "*.csv"), ("All files", "*.*")],
        )
        root.destroy()
        return [Path(p) for p in names]
    except (ImportError, RuntimeError, Exception) as exc:
        raise RuntimeError("Cannot open file dialog. Supply CSV paths on the command line.") from exc


def load_log(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path, comment="#", skip_blank_lines=True)
    df.columns = df.columns.str.strip()
    missing = sorted(set(REQUIRED).difference(df.columns))
    if missing:
        raise ValueError(f"Missing columns: {', '.join(missing)}")
    for col in (*REQUIRED, *OPTIONAL):
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors="coerce")
    df = df.dropna(subset=list(REQUIRED)).sort_values("time_ms").reset_index(drop=True)
    if len(df) < 2:
        raise ValueError("Need at least two valid data rows")
    if df.time_ms.duplicated().any():
        raise ValueError("Duplicate time_ms values found")
    if (np.diff(df.time_ms.to_numpy(dtype=float)) <= 0).any():
        raise ValueError("time_ms must strictly increase")
    if not np.isfinite(df[list(REQUIRED)].to_numpy(dtype=float)).all():
        raise ValueError("Non-finite required values")
    return df


def first_crossing_time(t: np.ndarray, y: np.ndarray, level: float, direction: int) -> float | None:
    """First crossing, linearly interpolated, relative to logged sample times."""
    qualifying = (y >= level) if direction > 0 else (y <= level)
    if qualifying[0]:
        return float(t[0])
    for i in range(1, len(y)):
        if qualifying[i] and not qualifying[i - 1]:
            dy = y[i] - y[i - 1]
            frac = (level - y[i - 1]) / dy if dy else 0.0
            return float(t[i - 1] + frac * (t[i] - t[i - 1]))
    return None


def analyze(df: pd.DataFrame, name: str, tolerance: float, deadline_ms: float,
            hold_ms: float, steady_tail_s: float, require_stays_settled: bool) -> Analysis:
    t = df.time_ms.to_numpy(dtype=float)
    y = df.measured_rpm.to_numpy(dtype=float)
    target = df.target_rpm.to_numpy(dtype=float)
    if not np.allclose(target, target[0], atol=1e-6, rtol=1e-6):
        raise ValueError("This version analyzes constant setpoints only; target_rpm changes during the log")
    sp = float(target[0])
    if sp == 0:
        raise ValueError("Target must be nonzero; relative percentage tolerance is undefined at 0 RPM")

    band = abs(sp) * tolerance / 100.0
    lo, hi = sp - band, sp + band
    inside = (y >= lo) & (y <= hi)
    entry = float(t[np.flatnonzero(inside)[0]]) if inside.any() else None

    # A hold starts at an observed in-band sample. It passes only when a later
    # observed in-band sample is >= hold_ms after that start, with no out-of-band
    # sample in between. This avoids assuming interpolation between samples.
    hold_start = None
    pass_time = None
    start_idx = None
    for i in range(len(t)):
        if not inside[i]:
            start_idx = None
            continue
        if start_idx is None:
            start_idx = i
        if t[i] - t[start_idx] >= hold_ms and t[i] <= deadline_ms:
            hold_start, pass_time = float(t[start_idx]), float(t[i])
            break
    passed = pass_time is not None

    # Conventional settling time (must stay inside until the end of the log).
    last_bad = np.flatnonzero(~inside)
    if not len(last_bad):
        settle = float(t[0])
    elif last_bad[-1] == len(t) - 1:
        settle = None
    else:
        settle = float(t[last_bad[-1] + 1])
    if require_stays_settled and (settle is None or settle > deadline_ms):
        passed = False

    initial = float(y[0])
    moving_up = sp > 0
    rise90 = first_crossing_time(t, y, 0.9 * sp, 1 if moving_up else -1)
    peak = float(np.max(y) if moving_up else np.min(y))
    overshoot = max(0.0, (peak - sp) / abs(sp) * 100) if moving_up else max(0.0, (sp - peak) / abs(sp) * 100)

    cutoff = t[-1] - steady_tail_s * 1000.0
    tail = y[t >= cutoff]
    steady = float(np.mean(tail))
    bias = steady - sp
    mae = float(np.mean(np.abs(tail - sp)))
    steps = np.diff(t)
    return Analysis(name, passed, pass_time, hold_start, entry, rise90, settle, peak,
                    overshoot, steady, bias, mae, float(steps.mean()),
                    float(steps.max()), sp, lo, hi, len(t))


def plot_log(df: pd.DataFrame, info: Analysis, deadline_ms: float,
             save_dir: Path | None, show: bool, filter_window: int = 1) -> None:
    t = df.time_ms.to_numpy(dtype=float) / 1000
    has_command = next((c for c in ("pwm", "control_output", "pwm_command") if c in df.columns), None)
    fig, ax = plt.subplots(2 if has_command else 1, 1, figsize=(11, 7 if has_command else 5.5),
                           sharex=True, constrained_layout=True)
    if has_command:
        ax_rpm, ax_cmd = ax
    else:
        ax_rpm, ax_cmd = ax, None
    ax_rpm.plot(t, df.target_rpm, linestyle="--", label="Target RPM", linewidth=1.5)
    ax_rpm.plot(t, df.measured_rpm, label="Measured RPM (raw)", linewidth=1.3)
    if filter_window > 1:
        y_filtered = df.measured_rpm.rolling(filter_window, min_periods=1).mean()
        ax_rpm.plot(t, y_filtered, label=f"Causal mean ({filter_window} samples; diagnostic)", linewidth=1.6)
    ax_rpm.axhspan(info.lower_rpm, info.upper_rpm, alpha=0.15, label="Tolerance band")
    ax_rpm.axvline(deadline_ms / 1000, linestyle=":", label="Deadline")
    if info.passed and info.hold_start_ms is not None and info.pass_time_ms is not None:
        ax_rpm.axvspan(info.hold_start_ms / 1000, info.pass_time_ms / 1000,
                       alpha=0.16, label="Passing hold interval")
    ax_rpm.set_ylabel("Speed (RPM)")
    ax_rpm.grid(True, alpha=0.3)
    ax_rpm.legend(loc="best")
    result = "PASS" if info.passed else "FAIL"
    ax_rpm.set_title(f"{info.name} | {result} | target {info.target_rpm:g} RPM")
    if ax_cmd is not None:
        ax_cmd.plot(t, df[has_command], label=has_command)
        ax_cmd.set_ylabel("Control command (raw units)")
        ax_cmd.grid(True, alpha=0.3)
        ax_cmd.legend(loc="best")
    (ax_cmd if ax_cmd is not None else ax_rpm).set_xlabel("Time (s)")
    if save_dir is not None:
        save_dir.mkdir(parents=True, exist_ok=True)
        safe_name = Path(info.name).name
        if safe_name.lower().endswith(".csv"):
            safe_name = safe_name[:-4]
        out = save_dir / f"{safe_name}_analysis.png"
        fig.savefig(out, dpi=160)
        print(f"  Graph saved: {out}")
    if not show:
        plt.close(fig)


def fmt(value: float | None, suffix: str = "") -> str:
    return "N/A" if value is None else f"{value:.2f}{suffix}"


def main() -> int:
    parser = argparse.ArgumentParser(description="Plot STM32 motor PID CSV logs with automatic pass/fail")
    parser.add_argument("csv", nargs="*", type=Path, help="CSV files (omit to choose through file dialog)")
    parser.add_argument("--tolerance", type=float, default=5, help="RPM tolerance percentage (default 5)")
    parser.add_argument("--deadline-ms", type=float, default=2000, help="Deadline to complete hold, milliseconds")
    parser.add_argument("--hold-ms", type=float, default=500, help="Continuous in-band hold required, milliseconds")
    parser.add_argument("--steady-tail-s", type=float, default=3, help="Tail duration for steady-state stats")
    parser.add_argument("--strict-settling", action="store_true", help="Also require staying within band through end of log")
    parser.add_argument("--save-dir", type=Path, default=None, help="Save PNG graphs and metrics CSV here")
    parser.add_argument("--no-show", action="store_true", help="Do not open plot windows")
    args = parser.parse_args()

    if not (0 < args.tolerance < 100 and args.deadline_ms > 0 and args.hold_ms > 0 and args.steady_tail_s > 0):
        parser.error("tolerance must be 0–100; deadline, hold, and steady-tail must be positive")
    if args.hold_ms > args.deadline_ms:
        parser.error("hold duration cannot exceed the deadline")
    files = args.csv or choose_files()
    if not files:
        print("No CSV files selected.")
        return 1
    results = []
    for file in files:
        try:
            df = load_log(file)
            info = analyze(df, file.name, args.tolerance, args.deadline_ms,
                           args.hold_ms, args.steady_tail_s, args.strict_settling)
            results.append(info)
            print(f"\n{file.name} : {'PASS' if info.passed else 'FAIL'}")
            print(f"  Target / band: {info.target_rpm:g} RPM  [{info.lower_rpm:.2f}, {info.upper_rpm:.2f}] RPM")
            print(f"  First entered band: {fmt(info.first_entry_ms, ' ms')}")
            print(f"  Hold start / completed: {fmt(info.hold_start_ms, ' ms')} / {fmt(info.pass_time_ms, ' ms')}")
            print(f"  90% rise crossing: {fmt(info.rise_90_ms, ' ms')}")
            print(f"  Conventional settling: {fmt(info.settle_ms, ' ms')}")
            print(f"  Peak / overshoot: {info.peak_rpm:.2f} RPM / {fmt(info.overshoot_pct, '%')}")
            print(f"  Tail mean / bias / MAE: {info.steady_rpm:.2f} / {info.steady_bias_rpm:+.2f} / {info.steady_mae_rpm:.2f} RPM")
            print(f"  Sample step mean/max: {info.mean_step_ms:.2f} / {info.max_step_ms:.2f} ms")
            plot_log(df, info, args.deadline_ms, args.save_dir, not args.no_show)
        except (OSError, ValueError, pd.errors.ParserError) as exc:
            print(f"ERROR reading {file}: {exc}", file=sys.stderr)
    if args.save_dir is not None and results:
        args.save_dir.mkdir(parents=True, exist_ok=True)
        result_file = args.save_dir / "pid_analysis_summary.csv"
        pd.DataFrame([vars(i) for i in results]).to_csv(result_file, index=False)
        print(f"\nSummary saved: {result_file}")
    if results and not args.no_show:
        plt.show()
    return 0 if results else 1


if __name__ == "__main__":
    raise SystemExit(main())
