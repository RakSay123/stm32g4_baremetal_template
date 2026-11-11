#!/usr/bin/env python3
"""STM32G474 PID tuner: manual trials and monitored, time-bounded Optuna search.

Firmware protocol: FF ON/OFF, PID kp ki kd, RUN rpm duration_ms, STOP,
OK PID / OK RUN / BEGIN / CSV HEADER / END DONE.

AUTO 20     -> up to 20 exploratory trials, within the default 10-minute limit
AUTO UNTIL  -> search for up to --auto-minutes (default 10), regardless of PASS
AUTO 5M     -> search for up to 5 minutes
AUTO 10M    -> search for up to 10 minutes
AUTO 0.5H   -> search for up to 30 minutes

AUTOMATION IS NOT A HARDWARE SAFETY SYSTEM. Supervise all motor operation. The
STM32 must independently stop on its experiment deadline; have a power cutoff.
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
import serial
from pid_log_analyzer import analyze, load_log, plot_log

HEADER = "time_ms,target_rpm,measured_rpm,error,pwm"
TX_CHARACTER_DELAY_S = 0.005  # G474 polling RX cannot reliably accept bursts yet
RESULT_COLUMNS = (
    "timestamp", "tag", "kp", "ki", "kd", "rpm", "duration_ms", "score", "accepted",
    "hold_pass", "pass_time_ms", "first_entry_ms", "first_hold_ms", "rise_90_ms",
    "filtered_rise_90_ms", "settle_ms", "filtered_settle_ms", "overshoot_pct",
    "early_iae", "early_mae_rpm", "iae", "steady_mae_rpm", "tail_std_rpm",
    "tail_peak_to_peak_rpm", "pwm_tail_std",
    "best_window_mae_rpm", "best_window_start_ms", "best_window_end_ms",
    "best_window_max_error_rpm", "best_window_score", "samples", "csv",
)


def read_line(port, deadline: float) -> str:
    """Reassemble a complete LF-terminated line across serial read timeouts."""
    chunks = []
    size = 0
    while time.monotonic() < deadline:
        raw = port.readline()
        if not raw:
            continue
        chunks.append(raw)
        size += len(raw)
        if size > 512:
            raise RuntimeError("Oversized UART line (lost framing or telemetry corruption)")
        if raw.endswith(b"\n"):
            return b"".join(chunks).decode("utf-8", errors="replace").strip()
    raise TimeoutError("STM32 response timed out (incomplete or missing line)")


def write_paced(port, command: str, delay: float = TX_CHARACTER_DELAY_S) -> None:
    for byte in (command + "\r").encode("ascii"):
        port.write(bytes((byte,)))
        time.sleep(delay)
    port.flush()


def send_command(port, command: str, expected: str, timeout: float = 4.0) -> str:
    write_paced(port, command)
    deadline = time.monotonic() + timeout
    while True:
        line = read_line(port, deadline)
        if not line or line == command:
            continue
        if "ERR UART_RX" in line:
            raise RuntimeError("ERR UART_RX (STM32 receive overrun)")
        if line.startswith("ERR"):
            raise RuntimeError(line)
        if line.startswith(expected):
            return line
        if line.startswith(command) and expected in line:
            return line[line.index(expected):]
        if line.startswith("READY") or line.startswith("Type HELP"):
            continue
        print("STM32:", line)


def emergency_stop(port) -> None:
    """Best effort only; cannot guarantee stopping on a broken serial link."""
    try:
        write_paced(port, "STOP")
    except (OSError, serial.SerialException):
        pass


def ensure_cooldown(last_end: float, seconds: float) -> None:
    remaining = seconds - (time.monotonic() - last_end)
    if remaining > 0:
        time.sleep(remaining)


def set_gains(port, gains: tuple[float, float, float]) -> None:
    # Use EXACT same quantized gains in Optuna results and MCU command.
    send_command(port, f"PID {gains[0]:.4f} {gains[1]:.4f} {gains[2]:.4f}", "OK PID")


def first_90_ms(t_ms: np.ndarray, speed: np.ndarray, rpm: float) -> float | None:
    """First interpolated 90% target crossing, assuming a start from rest."""
    level = rpm * 0.9
    if speed[0] >= level:
        return float(t_ms[0])
    for i in range(1, len(speed)):
        if speed[i] >= level and speed[i - 1] < level:
            dv = speed[i] - speed[i - 1]
            frac = (level - speed[i - 1]) / dv if dv else 0.0
            return float(t_ms[i - 1] + frac * (t_ms[i] - t_ms[i - 1]))
    return None


def first_hold_completion(t_ms: np.ndarray, inside: np.ndarray, hold_ms: float) -> float | None:
    """Earliest completed uninterrupted raw-sample hold, without deadline cutoff."""
    start = None
    for idx, ok in enumerate(inside):
        if not ok:
            start = None
        elif start is None:
            start = idx
        elif t_ms[idx] - t_ms[start] >= hold_ms:
            return float(t_ms[idx])
    return None


def settling_time(t_ms: np.ndarray, inside: np.ndarray) -> float | None:
    bad = np.flatnonzero(~inside)
    if not len(bad):
        return float(t_ms[0])
    if bad[-1] >= len(t_ms) - 1:
        return None
    return float(t_ms[bad[-1] + 1])


def best_500ms_window(t_ms: np.ndarray, speed: np.ndarray,
                      target_rpm: float, hold_ms: float,
                      deadline_ms: float, tolerance_rpm: float) -> dict | None:
    """Rank *raw* near-500ms windows ending by deadline, accuracy first.

    For every possible sampled start, use the earliest sampled endpoint at
    least hold_ms later. At 20ms telemetry intervals this gives 500ms windows;
    small timestamp jitter can make the sampled interval slightly longer.

    H = mean(|RPM - target|) / tolerance_rpm
        + 0.10 * window_end_ms / deadline_ms

    Earlier windows win ties or near-ties; time is a deliberately small bonus.
    This score does NOT decide PASS: analyzer's raw uninterrupted in-band
    hold still decides PASS.
    """
    best = None
    if len(t_ms) < 2:
        return None

    for start in range(len(t_ms) - 1):
        nominal_end = t_ms[start] + hold_ms
        if nominal_end > deadline_ms:
            break

        end = int(np.searchsorted(t_ms, nominal_end, side="left"))
        if end >= len(t_ms) or t_ms[end] > deadline_ms:
            continue

        errors = np.abs(speed[start:end + 1] - target_rpm)
        mean_error = float(np.mean(errors))
        end_ms = float(t_ms[end])
        h = mean_error / max(1e-9, tolerance_rpm) + 0.10 * end_ms / deadline_ms

        if best is None or h < best["score"]:
            best = {
                "score": float(h),
                "mae_rpm": mean_error,
                "start_ms": float(t_ms[start]),
                "end_ms": end_ms,
                "max_abs_error_rpm": float(np.max(errors)),
            }

    return best


def calculate_metrics(df, info, args) -> dict:
    t_ms = df.time_ms.to_numpy(dtype=float)
    y = df.measured_rpm.to_numpy(dtype=float)
    rpm = float(df.target_rpm.iloc[0])
    pwm = df.pwm.to_numpy(dtype=float)
    band = abs(rpm) * args.tolerance / 100.0
    inside = np.abs(y - rpm) <= band

    # Causal rolling-mean filter for DIAGNOSTICS only, not for acceptance.
    filtered = df.measured_rpm.rolling(args.filter_window, min_periods=1).mean().to_numpy(dtype=float)
    filtered_inside = np.abs(filtered - rpm) <= band
    filtered_settle = settling_time(t_ms, filtered_inside)

    early_cutoff = min(t_ms[-1], float(args.early_window_ms))
    # Include the first sample after starting. Do not invent a t=0 measurement.
    early_mask = t_ms <= early_cutoff
    et = t_ms[early_mask] / 1000.0
    ey = y[early_mask]
    early_iae = float(np.trapezoid(np.abs(ey - rpm), et)) if len(et) >= 2 else float("inf")
    early_elapsed_s = max(0.001, float(et[-1] - et[0])) if len(et) else 0.001
    early_mae = early_iae / early_elapsed_s
    iae = float(np.trapezoid(np.abs(y - rpm), t_ms / 1000.0))

    cutoff = t_ms[-1] - args.steady_tail_s * 1000.0
    tail = y[t_ms >= cutoff]
    tail_pwm = pwm[t_ms >= cutoff]
    tail_std = float(np.std(tail))
    tail_p2p = float(np.ptp(tail))
    pwm_std = float(np.std(tail_pwm))
    raw_rise = first_90_ms(t_ms, y, rpm)
    filtered_rise = first_90_ms(t_ms, filtered, rpm)
    hold_any = first_hold_completion(t_ms, inside, args.hold_ms)
    best_window = best_500ms_window(t_ms, y, rpm, args.hold_ms,
                                    args.deadline_ms, band)

    # Optimization objective: explicitly prioritize startup speed and arrival.
    # Conventional settling time is deliberately NOT in this score: one late
    # quantized sample otherwise makes almost every candidate score ~10 sec.
    window_s = args.early_window_ms / 1000.0
    early_norm = early_iae / max(1.0, abs(rpm) * window_s)
    rise_norm = (filtered_rise if filtered_rise is not None else args.auto_duration_ms + 1000) / args.deadline_ms
    # New H: accuracy in the best early 500ms window, with a slight time preference.
    hold_norm = best_window["score"] if best_window is not None else 20.0
    tail_norm = info.steady_mae_rpm / max(1.0, band)

    # Within allowed overshoot there's little incentive to under-drive the motor.
    overshoot = float(info.overshoot_pct or 0.0)
    overshoot_excess = max(0.0, overshoot - args.max_overshoot)
    overshoot_penalty = 0.4 * (overshoot_excess / 5.0) ** 2
    score = (0.50 * early_norm + 0.25 * rise_norm + 0.15 * hold_norm
             + 0.10 * tail_norm + overshoot_penalty)

    # IMPORTANT: acceptance always uses unfiltered/raw analyzer hold logic.
    accepted = bool(info.passed and info.hold_start_ms is not None
                    and info.hold_start_ms < args.arrival_ms
                    and overshoot <= args.max_overshoot
                    and info.steady_mae_rpm <= args.max_tail_mae
                    and tail_std <= args.max_tail_std)
    return {
        "score": float(score), "accepted": accepted,
        "first_hold_ms": hold_any, "rise_90_ms": raw_rise,
        "filtered_rise_90_ms": filtered_rise, "filtered_settle_ms": filtered_settle,
        "early_iae": early_iae, "early_mae_rpm": early_mae,
        "iae": iae, "tail_std_rpm": tail_std,
        "tail_peak_to_peak_rpm": tail_p2p, "pwm_tail_std": pwm_std,
        "best_window": best_window, "best_window_score": hold_norm,
    }


def save_result(rows, gains, rpm, duration_ms, args, tag, make_graph=True) -> dict:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    name = f"pid_{stamp}_kp{gains[0]:g}_ki{gains[1]:g}_kd{gains[2]:g}_{tag}"
    args.logs.mkdir(parents=True, exist_ok=True)
    csv_path = args.logs / f"{name}.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(HEADER.split(","))
        writer.writerows(rows)
    df = load_log(csv_path)
    if "pwm" not in df:
        raise ValueError("Telemetry requires pwm column")
    info = analyze(df, name, args.tolerance, args.deadline_ms, args.hold_ms,
                   args.steady_tail_s, args.strict_settling)
    metrics = calculate_metrics(df, info, args)
    if make_graph and not args.no_graphs:
        plot_log(df, info, args.deadline_ms, args.graphs, show=False,
                 filter_window=args.filter_window)
    record = dict(
        timestamp=stamp, tag=tag, kp=gains[0], ki=gains[1], kd=gains[2],
        rpm=rpm, duration_ms=duration_ms,
        score=round(metrics["score"], 6), accepted=metrics["accepted"],
        hold_pass=info.passed, pass_time_ms=info.pass_time_ms,
        first_entry_ms=info.first_entry_ms, first_hold_ms=metrics["first_hold_ms"],
        rise_90_ms=metrics["rise_90_ms"],
        filtered_rise_90_ms=metrics["filtered_rise_90_ms"],
        settle_ms=info.settle_ms,
        filtered_settle_ms=metrics["filtered_settle_ms"],
        overshoot_pct=info.overshoot_pct,
        early_iae=metrics["early_iae"], early_mae_rpm=metrics["early_mae_rpm"],
        iae=metrics["iae"], steady_mae_rpm=info.steady_mae_rpm,
        tail_std_rpm=metrics["tail_std_rpm"],
        tail_peak_to_peak_rpm=metrics["tail_peak_to_peak_rpm"],
        pwm_tail_std=metrics["pwm_tail_std"],
        best_window_mae_rpm=(metrics["best_window"] or {}).get("mae_rpm"),
        best_window_start_ms=(metrics["best_window"] or {}).get("start_ms"),
        best_window_end_ms=(metrics["best_window"] or {}).get("end_ms"),
        best_window_max_error_rpm=(metrics["best_window"] or {}).get("max_abs_error_rpm"),
        best_window_score=metrics["best_window_score"],
        samples=len(rows), csv=str(csv_path),
    )
    args.results.parent.mkdir(parents=True, exist_ok=True)
    exists = args.results.exists() and args.results.stat().st_size > 0
    with args.results.open("a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=RESULT_COLUMNS)
        if not exists:
            writer.writeheader()
        writer.writerow(record)
    return record


def fmt_ms(value) -> str:
    return "N/A" if value is None else f"{value:.0f} ms"


def verify_feedforward_mode(port, args) -> None:
    """Affirm the mode before EVERY run; fail closed if MCU rebooted/old firmware."""
    target = args.feedforward.upper()
    acknowledgement = send_command(port, f"FF {target}", f"OK FF {target}")
    if acknowledgement != f"OK FF {target}":
        raise RuntimeError(f"Unexpected FF acknowledgement: {acknowledgement!r}")


def run_test(port, gains, rpm, duration_ms, args, tag, make_graph=True) -> dict:
    """Stream and check telemetry; request STOP on communication/protocol fault."""
    rows = []
    begun = False
    header_seen = False
    last_t = -1.0
    try:
        verify_feedforward_mode(port, args)
        send_command(port, f"RUN {rpm:g} {duration_ms}", "OK RUN")
        deadline = time.monotonic() + duration_ms / 1000.0 + args.timeout_margin
        while True:
            # Detect a stalled CSV stream well before the total experiment timeout.
            line = read_line(port, min(deadline, time.monotonic() + args.stream_stall_s))
            if not line:
                continue
            if "ERR UART_RX" in line or line.startswith("ERR"):
                raise RuntimeError(f"STM32 reported: {line}")
            if line == "BEGIN":
                begun = True
                continue
            if not begun:
                continue
            if line == HEADER:
                header_seen = True
                continue
            if line.startswith("END "):
                end_reason = line[4:].strip()
                break
            if not header_seen:
                continue
            parts = line.split(",")
            if len(parts) != 5:
                raise RuntimeError(f"Malformed telemetry: {line!r}")
            try:
                t, target, measured, error, pwm = (float(v) for v in parts)
            except ValueError as exc:
                raise RuntimeError(f"Non-numeric telemetry: {line!r}") from exc
            if not all(math.isfinite(v) for v in (t, target, measured, error, pwm)):
                raise RuntimeError("Nonfinite telemetry")
            if t <= last_t or t > duration_ms + 250:
                raise RuntimeError("Bad telemetry timestamps")
            last_t = t
            if abs(target - rpm) > 0.2:
                raise RuntimeError("Unexpected target RPM")
            if measured < -args.max_reverse_rpm or measured > rpm * (1 + args.abort_overshoot / 100):
                raise RuntimeError(f"Safety abort: measured RPM {measured:.1f}")
            if pwm < -1 or pwm > args.max_pwm + 1:
                raise RuntimeError(f"Safety abort: PWM command {pwm:.1f}")
            rows.append(parts)
        if end_reason != "DONE":
            raise RuntimeError(f"Experiment ended: {end_reason}")
        if len(rows) < max(2, int(duration_ms / 20 * args.min_sample_fraction)):
            raise RuntimeError(f"Too few telemetry rows: {len(rows)}")
        result = save_result(rows, gains, rpm, duration_ms, args, tag, make_graph)
        print(f"  {'PASS' if result['accepted'] else 'FAIL'} | score {result['score']:.3f} | "
              f"first90(filtered) {fmt_ms(result['filtered_rise_90_ms'])} | "
              f"hold {fmt_ms(result['first_hold_ms'])} | "
              f"best500 MAE {result['best_window_mae_rpm']:.2f} RPM "
              f"(ends {fmt_ms(result['best_window_end_ms'])}) | "
              f"overshoot {result['overshoot_pct']:.1f}% | "
              f"tail MAE {result['steady_mae_rpm']:.2f} RPM")
        return result
    except BaseException:
        emergency_stop(port)
        raise


def study_signature(args) -> tuple[str, dict]:
    """Separate incompatible score/bounds/acceptance setups into different studies."""
    settings = {
        "version": 5, "feedforward": args.feedforward,
        "lut_id": args.lut_id, "rpm": args.auto_rpm, "duration_ms": args.auto_duration_ms,
        "kp": [args.kp_min, args.kp_max], "ki": [args.ki_min, args.ki_max],
        "kd": [args.kd_min, args.kd_max], "tolerance": args.tolerance,
        "deadline_ms": args.deadline_ms, "arrival_ms": args.arrival_ms,
        "hold_ms": args.hold_ms,
        "max_overshoot": args.max_overshoot, "max_tail_mae": args.max_tail_mae,
        "max_tail_std": args.max_tail_std,
        "steady_tail_s": args.steady_tail_s, "strict_settling": args.strict_settling,
        "early_window_ms": args.early_window_ms, "filter_window": args.filter_window,
    }
    digest = hashlib.sha256(json.dumps(settings, sort_keys=True).encode()).hexdigest()[:12]
    return f"stm32_pid_ff_{digest}", settings


def create_study(args):
    import optuna
    optuna.logging.set_verbosity(optuna.logging.WARNING)
    args.study_db.parent.mkdir(parents=True, exist_ok=True)
    # SQLite persists the entire Optuna trial history across Python sessions.
    url = "sqlite:///" + args.study_db.as_posix()
    name, settings = study_signature(args)
    study = optuna.create_study(study_name=name, storage=url,
                                load_if_exists=True, direction="minimize",
                                sampler=optuna.samplers.TPESampler(seed=args.seed))
    if not study.user_attrs.get("configuration"):
        study.set_user_attr("configuration", settings)
    return study


def make_candidate(trial, args) -> tuple[float, float, float]:
    return (round(trial.suggest_float("kp", args.kp_min, args.kp_max), 4),
            round(trial.suggest_float("ki", args.ki_min, args.ki_max), 4),
            round(trial.suggest_float("kd", args.kd_min, args.kd_max), 4))


def print_ranked(records):
    if not records:
        return
    print("\nBest candidates this session:")
    for rec in sorted(records, key=lambda r: r["score"])[:3]:
        print(f"  Kp={rec['kp']:.4f} Ki={rec['ki']:.4f} Kd={rec['kd']:.4f} "
              f"score={rec['score']:.3f}, single_pass={rec['accepted']}")


def auto_batch(port, limit_trials, minutes, args, last_end):
    try:
        import optuna
    except ImportError:
        print("Install Optuna first: python -m pip install optuna")
        return last_end

    print(f"\nAUTO: FF {args.feedforward.upper()}, {args.auto_rpm:g} RPM, {args.auto_duration_ms} ms/test, "
          f"up to {minutes:g} minutes / {limit_trials} exploratory trials")
    print(f"Bounds: Kp {args.kp_min:g}..{args.kp_max:g}, "
          f"Ki {args.ki_min:g}..{args.ki_max:g}, Kd {args.kd_min:g}..{args.kd_max:g}")
    print("Stop condition: time or trial limit only (PASS does not stop optimization).")
    print(f"PASS: within ±{args.tolerance:g}% before {args.arrival_ms:g} ms, "
          f"hold {args.hold_ms:g} ms uninterrupted, complete by {args.deadline_ms:g} ms; "
          "overshoot/tail limits still apply.")
    print("Safety: secure wheel; verify forward motion AND encoder sign; "
          "independent motor cutoff; supervise; firmware RUN deadline is required.")
    print("Software checks use UART telemetry and are NOT a substitute for MCU watchdog, current/thermal protection.")
    print(f"Operator check-in every {args.checkin_minutes:g} min; motor stays stopped while waiting.")
    if input("Type YES to start monitored automated search: ").strip() != "YES":
        print("Cancelled.")
        return last_end

    study = create_study(args)
    print(f"Optuna persistent study: {study.study_name} | existing trials: {len(study.trials)}")
    records = []
    start = time.monotonic()
    end_at = start + minutes * 60.0
    checkin_at = start + args.checkin_minutes * 60.0
    runs_since_rest = 0
    count = 0
    stop_reason = "trial limit reached"
    passing_candidates = 0

    def have_time_for_one(last: float) -> bool:
        # Reserve cooldown, one complete test, and a small communication margin.
        needed = max(0.0, args.cooldown_s - (time.monotonic() - last))
        needed += args.auto_duration_ms / 1000.0 + args.tx_budget_s
        return time.monotonic() + needed <= end_at

    def pause_if_due(last: float, run_count: int) -> tuple[float, int]:
        nonlocal checkin_at
        if args.rest_every > 0 and run_count >= args.rest_every:
            print(f"Scheduled motor rest: {args.rest_seconds:g} s")
            emergency_stop(port)
            time.sleep(args.rest_seconds)
            run_count = 0
        if time.monotonic() >= checkin_at:
            emergency_stop(port)
            print("\nOperator check-in: inspect motor/driver heat, supply, wiring and fixture.")
            answer = input("Motor confirmed stopped and safe? Type YES to continue: ").strip()
            if answer != "YES":
                raise RuntimeError("Operator ended session at check-in")
            checkin_at = time.monotonic() + args.checkin_minutes * 60.0
        return last, run_count

    try:
        while count < limit_trials:
            if not have_time_for_one(last_end):
                stop_reason = "time budget reached"
                break
            last_end, runs_since_rest = pause_if_due(last_end, runs_since_rest)
            if not have_time_for_one(last_end):
                stop_reason = "time budget reached"
                break
            trial = study.ask()
            gains = make_candidate(trial, args)
            count += 1
            print(f"\nTrial {count}/{limit_trials} | {int((time.monotonic()-start)/60)} min elapsed | "
                  f"Kp={gains[0]:.4f}, Ki={gains[1]:.4f}, Kd={gains[2]:.4f}")
            ensure_cooldown(last_end, args.cooldown_s)
            set_gains(port, gains)
            try:
                rec = run_test(port, gains, args.auto_rpm, args.auto_duration_ms,
                               args, f"auto_{trial.number:05d}",
                               make_graph=(count % args.graph_every == 0 or count == 1))
            finally:
                last_end = time.monotonic()
                runs_since_rest += 1
            # Save the optimization result immediately: interruptions won't lose progress.
            study.tell(trial, rec["score"])
            records.append(rec)
            print(f"  Study trials completed: {len(study.trials)}")

            if rec["accepted"]:
                passing_candidates += 1
                print(f"  PASS recorded ({passing_candidates} passing candidates this session); "
                      "continuing Optuna search.")
        print_ranked(records)
        print(f"\nAUTO ended: {stop_reason}. Explored {count} candidates in {(time.monotonic()-start)/60:.1f} min")
        print(f"Passing candidates this session: {passing_candidates}/{count}.")
        print("PASS is reported but never terminates AUTO; best gains are ranked by score.")
        try:
            historical_best = study.best_trial
            print(f"Historical best (this configuration): Kp={historical_best.params['kp']:.4f}, "
                  f"Ki={historical_best.params['ki']:.4f}, Kd={historical_best.params['kd']:.4f}, "
                  f"score={historical_best.value:.3f}")
        except ValueError:
            pass
        print(f"History: {args.study_db} | results: {args.results} | graphs: {args.graphs}")
    except (KeyboardInterrupt, RuntimeError, TimeoutError, ValueError,
            OSError, serial.SerialException) as exc:
        print(f"\nAUTO ABORTED: {exc}")
        print("Inspect hardware and confirm motor has stopped before continuing.")
    finally:
        emergency_stop(port)
    return time.monotonic()


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", required=True)
    p.add_argument("--feedforward", choices=("on", "off"), default="on",
                   help="Explicit STM32 FF mode; verified before EVERY RUN")
    p.add_argument("--lut-id", default="ascending_38pt_20261110",
                   help="Change when motor LUT, motor, load, or supply calibration changes")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--logs", type=Path, default=Path("logs/pid_tuning_ff"))
    p.add_argument("--graphs", type=Path, default=Path("analysis/graphs_ff"))
    # v3 adds new early-window columns; keep v2 results CSV untouched.
    p.add_argument("--results", type=Path, default=Path("analysis/pid_tuning_results_ff.csv"))
    p.add_argument("--study-db", type=Path, default=Path("analysis/pid_optuna_ff.sqlite3"))
    p.add_argument("--tolerance", type=float, default=2.5)
    p.add_argument("--deadline-ms", type=float, default=1500)
    p.add_argument("--arrival-ms", type=float, default=500,
                   help="PASS requires the uninterrupted in-band hold to start before this time")
    p.add_argument("--hold-ms", type=float, default=1000)
    p.add_argument("--steady-tail-s", type=float, default=1)
    p.add_argument("--strict-settling", action="store_true")
    p.add_argument("--filter-window", type=int, default=5,
                   help="Causal rolling-mean window for scoring/plotting only; raw data decides PASS")
    p.add_argument("--early-window-ms", type=int, default=1000)
    p.add_argument("--timeout-margin", type=float, default=8)
    p.add_argument("--stream-stall-s", type=float, default=3)
    p.add_argument("--cooldown-s", type=float, default=20)
    p.add_argument("--auto-minutes", type=float, default=10,
                   help="Default wall-clock limit for AUTO UNTIL and AUTO N, in minutes")
    p.add_argument("--max-trials", type=int, default=1000)
    p.add_argument("--auto-rpm", type=float, default=180)
    p.add_argument("--auto-duration-ms", type=int, default=4000)
    p.add_argument("--checkin-minutes", type=float, default=5)
    p.add_argument("--rest-every", type=int, default=10)
    p.add_argument("--rest-seconds", type=float, default=60)
    p.add_argument("--tx-budget-s", type=float, default=4.0,
                   help="Time reserve per trial for commands/firmware responses")
    p.add_argument("--graph-every", type=int, default=1)
    p.add_argument("--kp-min", type=float, default=0.2)
    p.add_argument("--kp-max", type=float, default=4)
    p.add_argument("--ki-min", type=float, default=0)
    p.add_argument("--ki-max", type=float, default=12)
    p.add_argument("--kd-min", type=float, default=0)
    p.add_argument("--kd-max", type=float, default=0.12)
    p.add_argument("--max-overshoot", type=float, default=10)
    p.add_argument("--abort-overshoot", type=float, default=35)
    p.add_argument("--max-reverse-rpm", type=float, default=15)
    p.add_argument("--max-pwm", type=float, default=999)
    p.add_argument("--max-tail-mae", type=float, default=4.5)
    p.add_argument("--max-tail-std", type=float, default=4.5,
                   help="Maximum raw steady-state RPM standard deviation for PASS")
    p.add_argument("--min-sample-fraction", type=float, default=0.85)
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--no-graphs", action="store_true")
    args = p.parse_args(argv)
    if args.feedforward == "off":
        # Keep baseline studies and graphs distinct from FF experiments.
        if args.logs == Path("logs/pid_tuning_ff"):
            args.logs = Path("logs/pid_tuning_baseline")
        if args.graphs == Path("analysis/graphs_ff"):
            args.graphs = Path("analysis/graphs_baseline")
        if args.results == Path("analysis/pid_tuning_results_ff.csv"):
            args.results = Path("analysis/pid_tuning_results_baseline.csv")
        if args.study_db == Path("analysis/pid_optuna_ff.sqlite3"):
            args.study_db = Path("analysis/pid_optuna_baseline.sqlite3")
    if not (0 < args.tolerance < 100 and 0 < args.arrival_ms
            and args.arrival_ms + args.hold_ms <= args.deadline_ms
            and 0 < args.hold_ms <= args.deadline_ms
            and 30 <= args.auto_rpm <= 250 and 1000 <= args.auto_duration_ms <= 15000
            and 0 < args.early_window_ms <= args.auto_duration_ms
            and 0 <= args.kp_min < args.kp_max <= 20
            and 0 <= args.ki_min < args.ki_max <= 40
            and 0 <= args.kd_min < args.kd_max <= 0.5
            and args.abort_overshoot > args.max_overshoot
            and args.max_tail_mae > 0 and args.max_tail_std > 0
            and args.cooldown_s >= 1 and args.stream_stall_s > 0
            and args.checkin_minutes > 0 and args.rest_seconds >= 0
            and args.rest_every >= 0 and 1 <= args.filter_window <= 100
            and 1 <= args.graph_every <= 1000
            and 0 < args.auto_minutes <= 30 and 1 <= args.max_trials <= 10000
            and 0 < args.min_sample_fraction <= 1):
        p.error("Invalid configuration. AUTO limit is 30 minutes; RUN limit is 15000 ms.")
    return args


def main(argv=None) -> int:
    args = parse_args(argv)
    gains = None
    last_end = -1e12
    with serial.Serial(args.port, args.baud, timeout=0.12, write_timeout=2) as port:
        port.reset_input_buffer()
        verify_feedforward_mode(port, args)
        print(f"Confirmed FF {args.feedforward.upper()} on STM32")
        print(f"Connected {args.port}. PID kp ki kd | RUN rpm ms | AUTO 20 | "
              "AUTO UNTIL | AUTO 10M | BEST | STATUS | HELP | STOP | QUIT")
        while True:
            try:
                tokens = input("PID> ").strip().split()
                if not tokens:
                    continue
                cmd = tokens[0].upper()
                if cmd in ("QUIT", "EXIT"):
                    emergency_stop(port)
                    break
                if cmd == "AUTO" and len(tokens) == 2:
                    arg = tokens[1].upper()
                    if arg == "UNTIL":
                        limit, minutes = args.max_trials, args.auto_minutes
                    elif arg.endswith("M"):
                        limit, minutes = args.max_trials, float(arg[:-1])
                    elif arg.endswith("H"):
                        limit, minutes = args.max_trials, 60.0 * float(arg[:-1])
                    else:
                        limit, minutes = int(arg), args.auto_minutes
                    if not math.isfinite(minutes) or not 0 < minutes <= 30:
                        raise ValueError("AUTO duration must be >0 and <=30 minutes")
                    if not 1 <= limit <= args.max_trials:
                        raise ValueError(f"AUTO trial count must be 1..{args.max_trials}")
                    last_end = auto_batch(port, limit, minutes, args, last_end)
                elif cmd == "PID" and len(tokens) == 4:
                    proposed = tuple(round(float(v), 4) for v in tokens[1:])
                    if any(not math.isfinite(x) or x < 0 for x in proposed):
                        print("Invalid gains")
                        continue
                    if (proposed[0] > 20 or proposed[1] > 40 or proposed[2] > 0.5):
                        print("Outside firmware gain bounds")
                        continue
                    set_gains(port, proposed)
                    gains = proposed
                    print("Gains updated")
                elif cmd == "RUN" and len(tokens) == 3:
                    if gains is None:
                        print("Use PID kp ki kd first")
                        continue
                    rpm, duration = float(tokens[1]), int(tokens[2])
                    if not 30 <= rpm <= 250 or not 1000 <= duration <= 15000:
                        print("Outside firmware RUN bounds")
                        continue
                    ensure_cooldown(last_end, args.cooldown_s)
                    try:
                        run_test(port, gains, rpm, duration, args, "manual")
                    finally:
                        last_end = time.monotonic()
                elif cmd == "BEST" and len(tokens) == 1:
                    try:
                        st = create_study(args)
                        best = st.best_trial
                        print(f"BEST Optuna score={best.value:.3f}: "
                              f"Kp={best.params['kp']:.4f} Ki={best.params['ki']:.4f} "
                              f"Kd={best.params['kd']:.4f} | Trials={len(st.trials)}")
                        print("Note: BEST ranks the objective, not necessarily a confirmed PASS.")
                    except ValueError:
                        print("No completed optimization trials yet.")
                elif cmd in ("STATUS", "HELP", "STOP") and len(tokens) == 1:
                    expected = {"STATUS": "STATUS", "HELP": "COMMANDS:", "STOP": "OK STOP"}[cmd]
                    print("STM32:", send_command(port, cmd, expected))
                    quiet = time.monotonic() + 0.35
                    while time.monotonic() < quiet:
                        raw = port.readline()
                        if raw:
                            print("STM32:", raw.decode(errors="replace").strip())
                            quiet = time.monotonic() + 0.15
                else:
                    print("PID kp ki kd | RUN rpm ms | AUTO 20 | AUTO UNTIL | AUTO 10M | BEST | STATUS | HELP | STOP | QUIT")
            except (ValueError, RuntimeError, TimeoutError, OSError, serial.SerialException) as exc:
                emergency_stop(port)
                print("ERROR:", exc)
            except KeyboardInterrupt:
                emergency_stop(port)
                print("\nSTOP requested. Inspect motor before resuming.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
