#!/usr/bin/env python3
"""Resumable, supervised, open-loop STM32 motor characterization.

Firmware protocol:
    PWM <0..999> <duration_ms> => OK PWM, BEGIN, CSV header, data, END DONE
    STOP                     => best-effort stop; firmware MUST time out independently

Examples:
    python analysis/motor_ff_characterize.py --dry-run --step 25
    python analysis/motor_ff_characterize.py --port COM11 --min-pwm 150 --max-pwm 550 --step 25
    python analysis/motor_ff_characterize.py --port COM11 --resume analysis/ff_characterization/20261110_123456
    python analysis/motor_ff_characterize.py --report-only analysis/ff_characterization/20261110_123456

Disconnects terminate the sweep; never silently reconnect/restart a physical motor.
Only completed, valid runs are recorded, and report/LUT generation is attempted
whether the sweep completes or is interrupted.
"""
import argparse
import csv
import json
import math
import statistics
import time
from datetime import datetime
from pathlib import Path

import serial

HEADER = 'time_ms,target_rpm,measured_rpm,error,pwm'
RUN_FIELDS = ('direction', 'pwm', 'repeat', 'file', 'mean_rpm', 'median_rpm',
              'std_rpm', 'min_rpm', 'max_rpm', 'rise90_ms', 'tail_samples')


def send(port, command):
    for byte in (command + '\r').encode('ascii'):
        port.write(bytes([byte]))
        time.sleep(0.005)
    port.flush()


def stop(port):
    try:
        if port.is_open:
            send(port, 'STOP')
    except (OSError, serial.SerialException):
        pass  # UART STOP is not a guaranteed motor stop.


def receive_line(port, deadline):
    while time.monotonic() < deadline:
        data = port.readline()
        if data:
            line = data.decode('utf-8', errors='replace').strip()
            if line:
                return line
    raise TimeoutError('UART telemetry stalled')


def run_pwm(port, pwm, duration_ms, timeout_extra=5):
    send(port, f'PWM {pwm} {duration_ms}')
    deadline = time.monotonic() + duration_ms / 1000 + timeout_extra
    rows = []
    seen_ok = seen_begin = seen_header = False
    last_time = -1
    while True:
        line = receive_line(port, min(deadline, time.monotonic() + 3))
        if line.startswith('ERR'):
            raise RuntimeError(f'Firmware error: {line}')
        if line.startswith('OK PWM'):
            seen_ok = True
            continue
        if line == 'BEGIN':
            seen_begin = True
            continue
        if line == HEADER:
            seen_header = True
            continue
        if line.startswith('END '):
            if line != 'END DONE':
                raise RuntimeError(f'Unexpected end marker: {line}')
            break
        if not seen_header:
            continue
        fields = line.split(',')
        if len(fields) != 5:
            raise RuntimeError(f'Malformed telemetry: {line!r}')
        try:
            values = [float(x) for x in fields]
        except ValueError as exc:
            raise RuntimeError(f'Invalid numerical telemetry: {line!r}') from exc
        if not all(math.isfinite(x) for x in values):
            raise RuntimeError('Nonfinite telemetry')
        t, target, rpm, error, received_pwm = values
        if t <= last_time or t > duration_ms + 250:
            raise RuntimeError(f'Invalid timestamp: {t}')
        if abs(received_pwm - pwm) > 1.1:
            raise RuntimeError(f'PWM mismatch: {received_pwm} vs {pwm}')
        if rpm < -15:
            raise RuntimeError(f'Unexpected reverse RPM: {rpm}')
        last_time = t
        rows.append(values)
    if not (seen_ok and seen_begin and seen_header):
        raise RuntimeError('Incomplete firmware handshake')
    if len(rows) < max(10, int(duration_ms / 20 * 0.70)):
        raise RuntimeError(f'Insufficient samples: {len(rows)}')
    return rows


def summarize(rows, settle_ms, duration_ms):
    tail = [r for r in rows if settle_ms <= r[0] <= duration_ms]
    if len(tail) < 10:
        raise RuntimeError('Not enough steady-state samples')
    rpms = [r[2] for r in tail]
    initial = rows[0][2]
    final = statistics.mean(rpms)
    threshold = initial + 0.9 * (final - initial)
    rise = (next((r[0] - rows[0][0] for r in rows if r[2] >= threshold), None)
            if final > initial + 5 else None)
    return {
        'mean_rpm': round(final, 4),
        'median_rpm': round(statistics.median(rpms), 4),
        'std_rpm': round(statistics.stdev(rpms), 4) if len(rpms) > 1 else 0.0,
        'min_rpm': round(min(rpms), 4),
        'max_rpm': round(max(rpms), 4),
        'rise90_ms': rise,
        'tail_samples': len(tail),
    }


def monotonic_lut(points):
    """Take monotonically increasing PWM->RPM data and invert to RPM->PWM.

    This is only a provisional LUT. Do not extrapolate beyond measured range.
    """
    kept = []
    for p in sorted(points, key=lambda item: item['pwm']):
        rpm = float(p['mean_rpm'])
        if not math.isfinite(rpm) or rpm <= 0:
            continue
        if kept and rpm <= kept[-1]['rpm']:  # FIX: entries use 'rpm', not 'mean_rpm'.
            continue
        kept.append({'rpm': round(rpm, 2), 'pwm': int(p['pwm'])})
    return kept


def write_header(path, points):
    with path.open('w', encoding='utf-8') as f:
        f.write('/* PROVISIONAL open-loop feedforward LUT; validate before use. */\n')
        f.write('#pragma once\n#include <stdint.h>\n\n')
        f.write('#include "motor_feedforward/motor_feedforward.h"\n\n')
        f.write(f'static const MotorFFPoint motor_ff_lut[{len(points)}] = {{\n')
        for point in points:
            f.write(f'    {{ {point["rpm"]:.2f}f, {point["pwm"]} }},\n')
        f.write('};\n')
        f.write(f'static const uint32_t motor_ff_lut_count = {len(points)}U;\n')


def write_summary(out, records):
    path = out / 'summary.csv'
    with path.open('w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=RUN_FIELDS)
        writer.writeheader()
        writer.writerows(records)


def read_summary(out):
    path = out / 'summary.csv'
    if not path.exists():
        return []
    records = []
    with path.open(newline='', encoding='utf-8') as f:
        for row in csv.DictReader(f):
            row['pwm'] = int(row['pwm'])
            row['repeat'] = int(row['repeat'])
            for key in ('mean_rpm','median_rpm','std_rpm','min_rpm','max_rpm'):
                row[key] = float(row[key])
            row['tail_samples'] = int(row['tail_samples'])
            row['rise90_ms'] = float(row['rise90_ms']) if row['rise90_ms'] else None
            records.append(row)
    return records


def plot(out, records):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('matplotlib unavailable; skipping plot')
        return
    fig, ax = plt.subplots(figsize=(8,5))
    for direction, marker in (('up','o'),('down','s')):
        pts = sorted((r for r in records if r['direction']==direction), key=lambda r:r['pwm'])
        if pts:
            ax.errorbar([r['pwm'] for r in pts], [r['mean_rpm'] for r in pts],
                        yerr=[r['std_rpm'] for r in pts], fmt=marker+'-',
                        capsize=2, label=direction)
    ax.set(xlabel='PWM (0-999)', ylabel='Steady-state RPM',
           title='Motor open-loop characterization (completed measurements only)')
    ax.grid(alpha=.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out/'pwm_rpm.png', dpi=160)
    plt.close(fig)


def build_reports(out, records):
    if not records:
        print('No completed runs to summarize.')
        return
    write_summary(out, records)
    plot(out, records)
    grouped = {}
    for r in records:
        if r['direction'] == 'up':
            grouped.setdefault(r['pwm'], []).append(r['mean_rpm'])
    points = [{'pwm':pwm,'mean_rpm':statistics.mean(vals)}
              for pwm,vals in sorted(grouped.items())]
    lut = monotonic_lut(points)
    with (out/'feedforward_lut.json').open('w', encoding='utf-8') as f:
        json.dump({'note':'PROVISIONAL; ascending-sweep points only. Do not extrapolate, verify voltage/load and direction.',
                   'points': lut}, f, indent=2)
    if lut:
        write_header(out/'motor_ff_lut.h', lut)
    print(f'Saved {len(records)} completed measurements to: {out}')
    print(f'Generated {len(lut)} provisional feedforward LUT points.')
    if lut:
        print(f'Measured LUT range: {lut[0]["rpm"]:.2f}–{lut[-1]["rpm"]:.2f} RPM')


def make_schedule(args):
    # The CLI accepts 1000 as 100% duty, but firmware uses 0..999.
    # Store/stream the actual hardware PWM so telemetry and resume match.
    levels = [min(x, 999) for x in range(args.min_pwm, args.max_pwm + 1, args.step)]
    endpoint = min(args.max_pwm, 999)
    if levels[-1] != endpoint:
        levels.append(endpoint)
    levels = list(dict.fromkeys(levels))
    return [(direction, pwm, repeat)
            for repeat in range(1, args.repeats + 1)
            for direction, seq in (('up', levels), ('down', levels[::-1]))
            for pwm in seq]


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--port', default='COM11')
    p.add_argument('--baud', type=int, default=115200)
    p.add_argument('--min-pwm', type=int, default=150)
    p.add_argument('--max-pwm', type=int, default=550)
    p.add_argument('--step', type=int, default=50)
    p.add_argument('--duration-ms', type=int, default=4000)
    p.add_argument('--settle-ms', type=int, default=2500)
    p.add_argument('--cooldown-s', type=float, default=7.5)
    p.add_argument('--repeats', type=int, default=2)
    p.add_argument('--check-every', type=int, default=10000)
    p.add_argument('--output', type=Path, default=Path('analysis/ff_characterization'))
    p.add_argument('--resume', type=Path, help='Resume session from its directory')
    p.add_argument('--report-only', type=Path, help='Regenerate reports without connecting to motor')
    p.add_argument('--dry-run', action='store_true')
    args = p.parse_args()
    if not (0 <= args.min_pwm <= args.max_pwm <= 1000 and args.step > 0 and args.repeats >= 1
            and 1000 <= args.duration_ms <= 15000 and 0 <= args.settle_ms <= args.duration_ms - 500
            and args.cooldown_s >= 0 and args.check_every >= 1):
        p.error('Invalid sweep parameters')
    if args.report_only:
        if not args.report_only.is_dir():
            p.error('Report directory does not exist')
        build_reports(args.report_only, read_summary(args.report_only))
        return
    if args.resume:
        out = args.resume
        config_path = out/'session_config.json'
        if config_path.exists():
            settings = json.loads(config_path.read_text(encoding='utf-8'))
            for key in ('min_pwm','max_pwm','step','duration_ms','settle_ms','repeats'):
                setattr(args, key, settings[key])
            print(f'Resuming session settings: {settings}')
        else:
            print('Legacy session (no session_config.json). Supply the ORIGINAL --min-pwm, --max-pwm, --step, --duration-ms, --settle-ms, and --repeats arguments.')
        records = read_summary(out)
    else:
        out = args.output / datetime.now().strftime('%Y%m%d_%H%M%S')
        records = []
    schedule = make_schedule(args)
    if args.max_pwm == 1000:
        print('NOTE: PWM 1000 means 100% requested duty; sending 999 (firmware maximum).')
    if max(pwm for _, pwm, _ in schedule) >= 900:
        print('CAUTION: high open-loop PWM may overheat or over-speed the motor. '               'Do not run the full range without verifying motor supply ratings and safe operating limits.')
    done = {(r['direction'], r['pwm'], r['repeat']) for r in records}
    remaining = [(direction, pwm, repeat) for direction,pwm,repeat in schedule
                 if (direction,pwm,repeat) not in done]
    print(f'Plan: {len(schedule)} runs, {len(done)} completed, {len(remaining)} remaining; '
          f'PWM {args.min_pwm}..{args.max_pwm}; {args.duration_ms/1000:g}s/test; '
          f'{args.cooldown_s:g}s cooldown.')
    print('Open-loop motor testing requires supervision, safe voltage/current, secured wheel, '
          'and an STM32-enforced timeout. UART STOP is not a hardware safety cutoff.')
    if args.dry_run:
        for i,(direction,pwm,repeat) in enumerate(remaining,1):
            print(f'{i:3d}: {direction:4s} PWM={pwm:3d} repeat={repeat}')
        return
    if not remaining:
        build_reports(out, records)
        return
    if input('Verify safe test setup and independent shutdown. Type YES: ').strip() != 'YES':
        print('Cancelled')
        return
    out.mkdir(parents=True, exist_ok=True)
    config = {key:getattr(args,key) for key in
              ('min_pwm','max_pwm','step','duration_ms','settle_ms','repeats')}
    (out/'session_config.json').write_text(json.dumps(config,indent=2),encoding='utf-8')
    # Print this before opening COM so it is available if the UART disconnects.
    print(f'Session directory: {out}')
    try:
        with serial.Serial(args.port,args.baud,timeout=.15,write_timeout=2) as port:
            port.reset_input_buffer()
            for i,(direction,pwm,repeat) in enumerate(remaining,1):
                if i > 1 and (i-1) % args.check_every == 0:
                    stop(port)
                    if input('Inspect motor/driver temperature and wiring. Type YES to continue: ').strip() != 'YES':
                        print('Stopped by operator.')
                        break
                print(f'[{i}/{len(remaining)}] {direction} PWM={pwm} repeat={repeat}',flush=True)
                try:
                    rows = run_pwm(port,pwm,args.duration_ms)
                finally:
                    stop(port)
                filename = f'{direction}_pwm{pwm}_r{repeat}.csv'
                with (out/filename).open('w',newline='',encoding='utf-8') as f:
                    w=csv.writer(f)
                    w.writerow(HEADER.split(','))
                    w.writerows(rows)
                rec = dict(direction=direction,pwm=pwm,repeat=repeat,file=filename,
                           **summarize(rows,args.settle_ms,args.duration_ms))
                records.append(rec)
                # Save after EACH successful experiment.
                write_summary(out,records)
                print(f'  mean {rec["mean_rpm"]:.2f} RPM | std {rec["std_rpm"]:.2f} RPM '
                      f'| rise90 {rec["rise90_ms"]} ms')
                time.sleep(args.cooldown_s)
    except (KeyboardInterrupt, TimeoutError, RuntimeError, OSError, serial.SerialException) as exc:
        print(f'\nSweep interrupted safely at Python level: {exc}')
        print('IMPORTANT: Confirm motor physically stopped. A failed UART STOP is NOT reliable.')
        print('Check COM port, STM32 reset, USB cable and other serial monitors before resuming.')
    finally:
        # No serial reconnect and no automatic motor restart after communication failures.
        build_reports(out,records)
        incomplete = len({(r['direction'],r['pwm'],r['repeat']) for r in records}) < len(schedule)
        if incomplete:
            print(f'To resume after verifying hardware: python analysis/motor_ff_characterize.py '
                  f'--port {args.port} --resume "{out}"')


if __name__ == '__main__':
    main()
