#!/usr/bin/env python3
r"""STM32G474 fixed-gain feedforward + PID reliability sweep (no Optuna).

Place beside pid_log_analyzer.py in analysis/ and run from the project root:
  python .\analysis\pid_rpm_reliability_sweep.py --port COM11 --dry-run
  python .\analysis\pid_rpm_reliability_sweep.py --port COM11

Firmware: FF ON, PID <kp> <ki> <kd>, RUN <rpm> <duration_ms>;
          OK RUN, BEGIN, CSV, END DONE. Each run has an MCU-enforced deadline.
Supervise motor, verify driver wiring and temperature, and maintain an independent
power cutoff. UART STOP is best effort and not a hardware safety system.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import time
from datetime import datetime
from pathlib import Path

HEADER = "time_ms,target_rpm,measured_rpm,error,pwm"
DEFAULT_SPEEDS = (50, 75, 100, 125, 150, 175, 200, 225, 250, 275, 300, 325, 350, 375, 400)
FIELDS = ("rpm", "repeat", "status", "pass", "first_entry_ms", "hold_start_ms",
          "pass_time_ms", "rise90_ms", "settle_ms", "peak_rpm", "overshoot_pct",
          "tail_mae_rpm", "tail_mean_rpm", "tail_std_rpm", "pwm_peak",
          "pwm_tail_mean", "samples", "csv", "error")
TX_DELAY_S = 0.005  # Polling STM32 UART RX needs character pacing.


def write_command(port, command: str) -> None:
    for byte in (command + "\r").encode("ascii"):
        port.write(bytes([byte]))
        time.sleep(TX_DELAY_S)
    port.flush()


def read_line(port, deadline: float) -> str:
    chunks: list[bytes] = []
    size = 0
    while time.monotonic() < deadline:
        raw = port.readline()
        if not raw:
            continue
        chunks.append(raw)
        size += len(raw)
        if size > 512:
            raise RuntimeError("Oversized UART line")
        if raw.endswith(b"\n"):
            return b"".join(chunks).decode("utf-8", errors="replace").strip()
    raise TimeoutError("UART timed out waiting for a complete line")


def command_ack(port, command: str, expected: str) -> str:
    write_command(port, command)
    deadline = time.monotonic() + 4
    while True:
        line = read_line(port, deadline)
        if line.startswith("ERR") or "ERR UART_RX" in line:
            raise RuntimeError(f"STM32 rejected {command}: {line}")
        if line.startswith(expected):
            return line
        if line.startswith(command) and expected in line:
            return line[line.index(expected):]
        # Ignore READY and residual acknowledgements; never ignore errors.


def stop_best_effort(port) -> None:
    try:
        if port.is_open:
            write_command(port, "STOP")
    except Exception:
        pass


def collect_run(port, rpm: int, duration_ms: int, args) -> list[list[float]]:
    command_ack(port, f"RUN {rpm} {duration_ms}", "OK RUN")
    deadline = time.monotonic() + duration_ms / 1000 + 8
    began = header_seen = False
    rows: list[list[float]] = []
    last_t = -1
    while True:
        line = read_line(port, min(deadline, time.monotonic() + 3))
        if line.startswith("ERR") or "ERR UART_RX" in line:
            raise RuntimeError(f"STM32 telemetry error: {line}")
        if line == "BEGIN":
            began = True
            continue
        if line == HEADER:
            header_seen = True
            continue
        if line.startswith("END "):
            if line != "END DONE":
                raise RuntimeError(f"Aborted by STM32: {line}")
            break
        if not (began and header_seen):
            continue
        fields = line.split(",")
        if len(fields) != 5:
            raise RuntimeError(f"Malformed UART telemetry: {line!r}")
        try:
            vals = [float(x) for x in fields]
        except ValueError as exc:
            raise RuntimeError(f"Non-numeric UART telemetry: {line!r}") from exc
        if not all(math.isfinite(x) for x in vals):
            raise RuntimeError("Non-finite UART telemetry")
        t, target, actual, error, pwm = vals
        if t <= last_t or t > duration_ms + 250:
            raise RuntimeError(f"Unexpected timestamp: {t}")
        if abs(target - rpm) > 0.25 or not -1 <= pwm <= 1000:
            raise RuntimeError(f"Unexpected target/PWM: {line!r}")
        # Abort dangerous excursions; ordinary off-target trials are FAILs, not errors.
        if actual < -15 or actual > max(rpm * 1.35, rpm + 30):
            raise RuntimeError(f"Safety abort: actual RPM {actual:.2f} at target {rpm}")
        if t >= 600 and actual < max(3.0, rpm * 0.08) and pwm >= 900:
            raise RuntimeError("Safety abort: near-stalled motor at high PWM")
        rows.append(vals)
        last_t = t
    if not (began and header_seen):
        raise RuntimeError("Missing BEGIN/CSV header")
    if len(rows) < int(duration_ms / 20 * 0.85):
        raise RuntimeError(f"Incomplete telemetry: {len(rows)} rows")
    return rows


def metrics_from_csv(path: Path, rpm: int, args) -> dict:
    # Reuse the project's exact existing raw-sample PASS/hold definition.
    from pid_log_analyzer import analyze, load_log
    df = load_log(path)
    info = analyze(df, path.stem, args.tolerance, args.deadline_ms,
                   args.hold_ms, args.steady_tail_s, False)
    tail = df[df.time_ms >= (df.time_ms.iloc[-1] - args.steady_tail_s * 1000)]
    first_entry = info.first_entry_ms
    # A passing hold must START by 500ms, be uninterrupted, and finish by 1500ms.
    passed = bool(info.passed and info.hold_start_ms is not None
                  and info.hold_start_ms < args.arrival_ms
                  and (info.overshoot_pct or 0) <= args.max_overshoot
                  and info.steady_mae_rpm <= args.max_tail_mae
                  and float(tail.measured_rpm.std(ddof=0)) <= args.max_tail_std)
    return dict(status="COMPLETE", pass_=passed, first_entry_ms=first_entry,
                hold_start_ms=info.hold_start_ms, pass_time_ms=info.pass_time_ms,
                rise90_ms=info.rise_90_ms, settle_ms=info.settle_ms,
                peak_rpm=info.peak_rpm, overshoot_pct=info.overshoot_pct,
                tail_mae_rpm=info.steady_mae_rpm,
                tail_mean_rpm=float(tail.measured_rpm.mean()),
                tail_std_rpm=float(tail.measured_rpm.std(ddof=0)),
                pwm_peak=float(df.pwm.max()),
                pwm_tail_mean=float(tail.pwm.mean()), samples=len(df))


def save_summary(directory: Path, records: list[dict]) -> None:
    with (directory / "summary.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(records)


def create_graphs(directory: Path, records: list[dict], args) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from pid_log_analyzer import load_log
    graphs = directory / "graphs"
    graphs.mkdir(exist_ok=True)
    good = [r for r in records if r["status"] == "COMPLETE"]
    if not good:
        return

    # Per-trial response plot with identical tolerance/arrival/hold graphics.
    for rec in good:
        path = directory / rec["csv"]
        df = load_log(path)
        fig, (ax, ax2) = plt.subplots(2, 1, sharex=True, figsize=(10, 6),
                                     gridspec_kw={"height_ratios": [2, 1]}, constrained_layout=True)
        x = df.time_ms / 1000
        rpm = rec["rpm"]
        band = rpm * args.tolerance / 100
        ax.plot(x, df.measured_rpm, lw=1.1, label="Raw encoder RPM")
        ax.plot(x, df.measured_rpm.rolling(5, min_periods=1).mean(),
                lw=1.4, label="5-sample mean (display only)")
        ax.axhline(rpm, ls="--", color="black", alpha=.65, label="Target")
        ax.axhspan(rpm-band, rpm+band, alpha=.12, label="±2.5%")
        ax.axvline(args.arrival_ms / 1000, ls=":", color="gray", label="Arrival deadline")
        ax.axvline(args.deadline_ms / 1000, ls=":", color="black", alpha=.5, label="Hold deadline")
        if rec["pass"] and rec["hold_start_ms"] is not None:
            ax.axvspan(rec["hold_start_ms"] / 1000, rec["pass_time_ms"] / 1000,
                       alpha=.14, color="green", label="Passing hold")
        ax.set(ylabel="Speed (RPM)", title=f'{rpm} RPM | repeat {rec["repeat"]} | '
               + ("PASS" if rec["pass"] else "FAIL"))
        ax.legend(fontsize=8, loc="best")
        ax.grid(alpha=.2)
        ax2.plot(x, df.pwm, lw=1)
        ax2.set(xlabel="Time (s)", ylabel="PWM command", ylim=(0, 1010))
        ax2.grid(alpha=.2)
        fig.savefig(graphs / f'rpm_{rpm:03d}_repeat_{rec["repeat"]}.png', dpi=145)
        plt.close(fig)

    # Multi-run normalized speed trace: same scale for all speeds.
    fig, ax = plt.subplots(figsize=(11, 6))
    for rec in good:
        df = load_log(directory / rec["csv"])
        ax.plot(df.time_ms / 1000, df.measured_rpm / rec["rpm"] * 100,
                label=f'{rec["rpm"]} RPM r{rec["repeat"]}', alpha=.65, lw=1)
    ax.axhline(100, color="black", ls="--", lw=1)
    ax.axhspan(100-args.tolerance, 100+args.tolerance, color="green", alpha=.10)
    ax.axvline(args.arrival_ms / 1000, color="gray", ls=":")
    ax.set(xlabel="Time (s)", ylabel="Measured / target (%)",
           title="All step responses normalized by requested RPM")
    ax.set_ylim(bottom=0)
    ax.legend(fontsize=7, ncol=3)
    ax.grid(alpha=.2)
    fig.tight_layout()
    fig.savefig(graphs / "all_normalized_step_responses.png", dpi=155)
    plt.close(fig)

    # Summary metrics, with individual runs and per-speed means.
    for key, title, ylabel in (
        ("rise90_ms", "90% rise time vs target RPM", "Rise time (ms)"),
        ("tail_mae_rpm", "Steady-state mean absolute error vs target RPM", "MAE (RPM)"),
        ("overshoot_pct", "Peak overshoot vs target RPM", "Overshoot (%)"),
        ("pwm_tail_mean", "Mean steady PWM vs target RPM", "PWM units"),
    ):
        fig, ax = plt.subplots(figsize=(9, 5))
        speeds = sorted({r["rpm"] for r in good})
        means = []
        for rpm in speeds:
            vals = [float(r[key]) for r in good if r["rpm"] == rpm and r[key] is not None]
            means.append(statistics.mean(vals) if vals else math.nan)
            if vals:
                ax.scatter([rpm] * len(vals), vals, color="tab:blue", alpha=.5, s=28)
        ax.plot(speeds, means, "o-", color="tab:orange", label="Mean of completed repeats")
        ax.set(xlabel="Target RPM", ylabel=ylabel, title=title)
        ax.grid(alpha=.25)
        ax.legend()
        fig.tight_layout()
        fig.savefig(graphs / f'{key}_vs_rpm.png', dpi=155)
        plt.close(fig)

    speeds = sorted({r["rpm"] for r in good})
    rates = [sum(bool(r["pass"]) for r in good if r["rpm"] == rpm) /
             sum(r["rpm"] == rpm for r in good) * 100 for rpm in speeds]
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.bar([str(s) for s in speeds], rates, color="tab:blue")
    ax.set(xlabel="Target RPM", ylabel="PASS rate among completed trials (%)",
           title="Strict pass rate: ±2.5%, arrival <500 ms, 1000 ms hold", ylim=(0, 105))
    ax.grid(axis="y", alpha=.2)
    fig.tight_layout()
    fig.savefig(graphs / "pass_rate_by_rpm.png", dpi=155)
    plt.close(fig)


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default="COM11")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--speeds", type=int, nargs="+", default=list(DEFAULT_SPEEDS))
    p.add_argument("--repeats", type=int, default=3)
    p.add_argument("--duration-ms", type=int, default=4000)
    p.add_argument("--kp", type=float, default=2.1541)
    p.add_argument("--ki", type=float, default=7.1090)
    p.add_argument("--kd", type=float, default=0.0056)
    p.add_argument("--cooldown-s", type=float, default=20)
    p.add_argument("--rest-every", type=int, default=10)
    p.add_argument("--rest-seconds", type=float, default=60)
    p.add_argument("--checkin-minutes", type=float, default=5)
    p.add_argument("--tolerance", type=float, default=2.5)
    p.add_argument("--arrival-ms", type=float, default=500)
    p.add_argument("--hold-ms", type=float, default=1000)
    p.add_argument("--deadline-ms", type=float, default=1500)
    p.add_argument("--steady-tail-s", type=float, default=1)
    p.add_argument("--max-overshoot", type=float, default=10)
    p.add_argument("--max-tail-mae", type=float, default=4.5)
    p.add_argument("--max-tail-std", type=float, default=4.5)
    p.add_argument("--output", type=Path, default=Path("analysis/pid_rpm_reliability"))
    p.add_argument("--dry-run", action="store_true")
    p.add_argument("--extended-range-confirmed", action="store_true",
                   help="Confirm reflashed STM32 RUN accepts targets to 400 RPM and motor/driver can safely operate there")
    args = p.parse_args(argv)
    if (not args.speeds or len(args.speeds) != len(set(args.speeds))
        or any(not 30 <= s <= 400 for s in args.speeds)
        or not 1 <= args.repeats <= 10 or not 1000 <= args.duration_ms <= 15000
        or not (0 < args.arrival_ms < args.deadline_ms <= args.duration_ms)
        or args.arrival_ms + args.hold_ms > args.deadline_ms
        or not 0 < args.steady_tail_s < args.duration_ms / 1000
        or args.cooldown_s < 1 or args.checkin_minutes <= 0
        or args.rest_every < 0 or args.rest_seconds < 0
        or any(not math.isfinite(v) or v < 0 for v in (args.kp, args.ki, args.kd))
        or args.kp > 20 or args.ki > 40 or args.kd > .5):
        p.error("Invalid speed, PID, timing, or safety configuration")
    return args


def main(argv=None) -> int:
    args = parse_args(argv)
    schedule = [(rpm, repeat) for rpm in args.speeds for repeat in range(1, args.repeats + 1)]
    print(f"Gains FIXED: Kp={args.kp:.4f} Ki={args.ki:.4f} Kd={args.kd:.4f} | FF ON")
    print(f"Schedule: {len(schedule)} tests at {args.duration_ms / 1000:g}s each; "
          f"speeds {args.speeds} RPM, {args.repeats} repeat(s) each")
    print(f"PASS: ±{args.tolerance:g}% by <{args.arrival_ms:g}ms, raw 1000ms hold ending "
          f"by {args.deadline_ms:g}ms; <= {args.max_overshoot:g}% overshoot; "
          f"tail MAE/std <= {args.max_tail_mae:g}/{args.max_tail_std:g} RPM")
    print("IMPORTANT: recent TB6612FNG reverse wiring/overheating could have damaged driver. "
          "Check driver temperature, wiring, and battery BEFORE starting.")
    print("Tests run with a secure elevated wheel, live supervision, and independent motor cutoff.")
    if args.dry_run:
        for i, (rpm, rep) in enumerate(schedule, 1):
            print(f"  {i:02d}: RPM {rpm:3d} | repeat {rep}")
        return 0
    extended = [s for s in args.speeds if s > 250]
    if extended and not args.extended_range_confirmed:
        print(f"REFUSING TO RUN: targets {extended} RPM exceed the previously confirmed "
              "STM32 RUN range (30..250 RPM). Rebuild and flash firmware with "
              "APP_MAX_TARGET_RPM=400, verify safe hardware operation over the range, "
              "then rerun with --extended-range-confirmed. For the current firmware, "
              "use --speeds 50 75 100 125 150 175 200 225 250.")
        return 2
    if input("Verify motor driver safe, wheel secure, and cutoff available. Type YES: ").strip() != "YES":
        print("Cancelled")
        return 1
    import serial
    session = args.output / datetime.now().strftime("%Y%m%d_%H%M%S")
    session.mkdir(parents=True, exist_ok=False)
    (session / "csv").mkdir()
    (session / "graphs").mkdir()
    config = {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()}
    (session / "config.json").write_text(json.dumps(config, indent=2), encoding="utf-8")
    print(f"Results folder: {session}")
    records = []
    last_end = -1e12
    next_check = time.monotonic() + args.checkin_minutes * 60
    completed = 0
    try:
        with serial.Serial(args.port, args.baud, timeout=.12, write_timeout=2) as port:
            port.reset_input_buffer()
            command_ack(port, "FF ON", "OK FF ON")
            command_ack(port, f"PID {args.kp:.4f} {args.ki:.4f} {args.kd:.4f}", "OK PID")
            for index, (rpm, rep) in enumerate(schedule, 1):
                if args.rest_every and completed and completed % args.rest_every == 0:
                    stop_best_effort(port)
                    print(f"Motor rest {args.rest_seconds:g}s")
                    time.sleep(args.rest_seconds)
                if time.monotonic() >= next_check:
                    stop_best_effort(port)
                    if input("Inspect TB6612FNG temperature and supply; motor STOPPED? Type YES: ").strip() != "YES":
                        print("Operator ended sweep")
                        break
                    next_check = time.monotonic() + args.checkin_minutes * 60
                remaining = args.cooldown_s - (time.monotonic() - last_end)
                if remaining > 0:
                    time.sleep(remaining)
                print(f"[{index}/{len(schedule)}] target {rpm} RPM, repeat {rep}", flush=True)
                try:
                    command_ack(port, "FF ON", "OK FF ON")
                    command_ack(port, f"PID {args.kp:.4f} {args.ki:.4f} {args.kd:.4f}", "OK PID")
                    data = collect_run(port, rpm, args.duration_ms, args)
                finally:
                    last_end = time.monotonic()
                    stop_best_effort(port)
                filename = f"csv/rpm_{rpm:03d}_repeat_{rep}.csv"
                with (session / filename).open("w", newline="", encoding="utf-8") as f:
                    writer = csv.writer(f)
                    writer.writerow(HEADER.split(","))
                    writer.writerows(data)
                result = metrics_from_csv(session / filename, rpm, args)
                rec = {"rpm": rpm, "repeat": rep, "status": result["status"],
                       "pass": result["pass_"], **{k: v for k, v in result.items()
                                               if k not in ("status", "pass_")},
                       "csv": filename, "error": ""}
                records.append(rec)
                completed += 1
                save_summary(session, records)
                print(f'  {"PASS" if rec["pass"] else "FAIL"} | rise90={rec["rise90_ms"]} ms | '
                      f'overshoot={rec["overshoot_pct"]:.2f}% | tail MAE={rec["tail_mae_rpm"]:.2f} RPM')
    except (KeyboardInterrupt, TimeoutError, RuntimeError, OSError, serial.SerialException) as exc:
        print(f"SWEEP STOPPED: {exc}")
        print("Confirm motor is physically stopped. Do not restart until hardware is safe.")
    finally:
        if records:
            try:
                create_graphs(session, records, args)
            except Exception as exc:
                print(f"Graph generation error (CSV files are preserved): {exc}")
        print(f"Saved {len(records)}/{len(schedule)} completed tests to {session}")
        if records:
            n_pass = sum(bool(r["pass"]) for r in records)
            print(f"Strict pass rate on completed tests: {n_pass}/{len(records)} "
                  f"({100*n_pass/len(records):.1f}%). Inspect by-speed pass rates in graphs/.")
    return 0 if len(records) == len(schedule) else 1


if __name__ == "__main__":
    raise SystemExit(main())
