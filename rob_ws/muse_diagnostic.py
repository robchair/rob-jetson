#!/usr/bin/env python3
"""
MUSE Diagnostic — run this BEFORE eeg_cmd_node.py to find real thresholds.
Prerequisites:
  muselsl stream --address 00:55:DA:B8:34:01 --acc

Usage:
  python3 muse_diagnostic.py          # ACC + EEG if available
  python3 muse_diagnostic.py --acc    # ACC only
  python3 muse_diagnostic.py --eeg    # EEG only
"""
import argparse
import time
import numpy as np
from collections import deque
from pylsl import StreamInlet, resolve_streams

RESET  = "\033[0m"
RED    = "\033[31m"
GREEN  = "\033[32m"
YELLOW = "\033[33m"
CYAN   = "\033[36m"
BOLD   = "\033[1m"

def find_streams(want_acc=True, want_eeg=True):
    print("Resolving LSL streams (up to 5s)...")
    streams = resolve_streams(wait_time=5.0)
    types = [s.type() for s in streams]
    print(f"  Found: {types if types else 'NONE'}\n")
    acc_inlet = eeg_inlet = None
    for s in streams:
        if want_acc and s.type() == 'ACC' and acc_inlet is None:
            acc_inlet = StreamInlet(s)
            print(f"{GREEN}✓ ACC stream connected{RESET}  (channels: {s.channel_count()}, rate: {s.nominal_srate()} Hz)")
        if want_eeg and s.type() == 'EEG' and eeg_inlet is None:
            eeg_inlet = StreamInlet(s)
            print(f"{GREEN}✓ EEG stream connected{RESET}  (channels: {s.channel_count()}, rate: {s.nominal_srate()} Hz)")
    if want_acc and acc_inlet is None:
        print(f"{RED}✗ No ACC stream. Did you run: muselsl stream --acc --address ...?{RESET}")
    if want_eeg and eeg_inlet is None:
        print(f"{YELLOW}⚠ No EEG stream found (run muselsl stream without --acc to get both){RESET}")
    return acc_inlet, eeg_inlet

# ── Phase 1: calibrate neutral ─────────────────────────────────────────────

def calibrate(acc_inlet, duration=3.0):
    print(f"\n{BOLD}=== CALIBRATION: hold head still, looking straight ahead ==={RESET}")
    print(f"  Collecting {duration}s of neutral ACC data...")
    samples = []
    t0 = time.time()
    while time.time() - t0 < duration:
        s, _ = acc_inlet.pull_sample(timeout=0.1)
        if s:
            samples.append(s)
    if not samples:
        print(f"{RED}No ACC samples received!{RESET}")
        return None, None
    data = np.array(samples)
    x_base = np.mean(data[:, 0])
    y_base = np.mean(data[:, 1])
    z_base = np.mean(data[:, 2])
    x_std  = np.std(data[:, 0])
    y_std  = np.std(data[:, 1])
    print(f"\n  {CYAN}Baseline  X:{x_base:+.4f}  Y:{y_base:+.4f}  Z:{z_base:+.4f}{RESET}")
    print(f"  Noise std X:{x_std:.4f}  Y:{y_std:.4f}")
    print(f"  Suggested x_threshold (2× noise): {2*x_std:.4f}")
    print(f"  Collected {len(samples)} samples")
    return np.array([x_base, y_base, z_base]), data

# ── Phase 2: live ACC tilt display ─────────────────────────────────────────

def watch_acc(acc_inlet, baseline, duration=20.0):
    print(f"\n{BOLD}=== LIVE ACC: tilt head in all directions for {int(duration)}s ==={RESET}")
    print("  Columns: raw_X  raw_Y  raw_Z | delta_X delta_Y")
    print("  Tilt forward=+Y  back=-Y  left=+X  right=-X\n")
    x_base, y_base, z_base = baseline
    x_min = x_max = 0.0
    y_min = y_max = 0.0
    t0 = time.time()
    while time.time() - t0 < duration:
        s, _ = acc_inlet.pull_sample(timeout=0.05)
        if s is None:
            continue
        dx = s[0] - x_base
        dy = s[1] - y_base
        x_min = min(x_min, dx); x_max = max(x_max, dx)
        y_min = min(y_min, dy); y_max = max(y_max, dy)

        bar_x = int(abs(dx) / 0.05)  # 1 char per 0.05g
        bar_y = int(abs(dy) / 0.05)
        dir_x = ("←" if dx < 0 else "→") if abs(dx) > 0.02 else "·"
        dir_y = ("↑" if dy > 0 else "↓") if abs(dy) > 0.02 else "·"
        color = GREEN if (abs(dx) < 0.3 and abs(dy) < 0.4) else YELLOW

        print(
            f"\r  raw [{s[0]:+.3f} {s[1]:+.3f} {s[2]:+.3f}]"
            f"  Δ X:{color}{dx:+.3f}{dir_x}{'█'*bar_x}{RESET}"
            f"  Y:{color}{dy:+.3f}{dir_y}{'█'*bar_y}{RESET}   ",
            end="", flush=True
        )
    print(f"\n\n  {CYAN}ACC range observed:{RESET}")
    print(f"    ΔX: {x_min:+.3f} … {x_max:+.3f}  → suggested x_threshold: {max(abs(x_min),abs(x_max))*0.3:.3f}")
    print(f"    ΔY: {y_min:+.3f} … {y_max:+.3f}  → forward trigger at ~{y_max*0.3:.3f}")

# ── Phase 3: EEG clench threshold finder ───────────────────────────────────

def watch_eeg(eeg_inlet, duration=30.0):
    print(f"\n{BOLD}=== EEG CLENCH TEST: relax, then jaw-clench hard 3× over {int(duration)}s ==={RESET}")
    print("  Watching peak absolute µV across all 4 channels.\n")
    buf = deque(maxlen=10)
    quiet_peaks = []
    clench_peaks = []
    t0 = time.time()
    phase = "quiet"
    phase_switch = 10.0  # first 10s quiet, then watch for clenches

    while time.time() - t0 < duration:
        s, _ = eeg_inlet.pull_sample(timeout=0.05)
        if s is None:
            continue
        peak = np.max(np.abs(np.array(s)))
        buf.append(peak)
        elapsed = time.time() - t0

        if elapsed < phase_switch:
            quiet_peaks.append(peak)
            label = f"{CYAN}QUIET{RESET}"
        else:
            clench_peaks.append(peak)
            label = f"{YELLOW}WATCH{RESET}"

        smooth = np.mean(buf)
        bar = int(min(smooth, 1000) / 20)
        color = RED if smooth > 400 else (YELLOW if smooth > 150 else GREEN)
        print(
            f"\r  [{label}] t:{elapsed:5.1f}s  peak:{color}{peak:6.1f}µV{RESET}"
            f"  smooth:{smooth:6.1f}  {'█'*bar}{'░'*(50-bar)}   ",
            end="", flush=True
        )

    print(f"\n\n  {CYAN}EEG summary:{RESET}")
    if quiet_peaks:
        print(f"    Quiet   — median:{np.median(quiet_peaks):.1f}  p95:{np.percentile(quiet_peaks,95):.1f}  max:{max(quiet_peaks):.1f} µV")
    if clench_peaks:
        print(f"    Clench  — median:{np.median(clench_peaks):.1f}  p95:{np.percentile(clench_peaks,95):.1f}  max:{max(clench_peaks):.1f} µV")
    if quiet_peaks and clench_peaks:
        quiet95  = np.percentile(quiet_peaks, 95)
        clench5  = np.percentile(clench_peaks, 5)
        midpoint = (quiet95 + clench5) / 2
        print(f"\n    {GREEN}Suggested clench_threshold: {midpoint:.0f} µV{RESET}")
        print(f"    (midpoint between quiet p95={quiet95:.0f} and clench p5={clench5:.0f})")

# ── Main ───────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--acc', action='store_true', help='ACC only')
    ap.add_argument('--eeg', action='store_true', help='EEG only')
    args = ap.parse_args()
    want_acc = not args.eeg
    want_eeg = not args.acc

    acc_inlet, eeg_inlet = find_streams(want_acc, want_eeg)

    if want_acc and acc_inlet:
        baseline, _ = calibrate(acc_inlet)
        if baseline is not None:
            input(f"\n  {BOLD}Press ENTER to start live ACC tilt test (20s)...{RESET}")
            watch_acc(acc_inlet, baseline, duration=20.0)

    if want_eeg and eeg_inlet:
        input(f"\n  {BOLD}Press ENTER to start EEG clench test (30s)...{RESET}")
        watch_eeg(eeg_inlet, duration=30.0)

    print(f"\n{BOLD}=== Done. Copy suggested thresholds into eeg_cmd_node.py ==={RESET}\n")

if __name__ == '__main__':
    main()
