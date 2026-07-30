#!/usr/bin/env python3
"""Terminal FSR monitor for the PIUFSR master.

Reads the master's 20 Hz calibration stream and draws the pad layout with
per-sensor colouring. Thresholds are read back from the slaves at startup
(`t` command) so the display reflects what the firmware is actually using;
pass a threshold on the command line to override all twenty.
"""
import argparse
import sys
import time

import serial

NUM_PANELS = 5
FSRS_PER_PANEL = 4
NUM_SENSORS = NUM_PANELS * FSRS_PER_PANEL

LABELS = ["B.LFT", "T.LFT", "CENTER", "T.RGT", "B.RGT"]
DEFAULT_BAUD = 115200
THRESHOLD_TIMEOUT = 3.0


def clear():
    print("\033[H\033[J", end="")


def colored(val, thr):
    if val >= thr:
        return f"\033[31m{val:3d}\033[0m"
    if val >= thr // 2:
        return f"\033[33m{val:3d}\033[0m"
    return f"{val:3d}"


def panel_lines(vals, pi, thrs):
    """Return [label_line, lr_line, ud_line] for a panel."""
    b = pi * FSRS_PER_PANEL
    label = f"P{pi} {LABELS[pi]}"
    lr = (f" L[{colored(vals[b], thrs[b])}]"
          f" R[{colored(vals[b + 1], thrs[b + 1])}] ")
    ud = (f" U[{colored(vals[b + 2], thrs[b + 2])}]"
          f" D[{colored(vals[b + 3], thrs[b + 3])}] ")
    return label, lr, ud


def draw(values, thrs):
    clear()
    # Row 1: TL (P1) centered
    la, lr, ud = panel_lines(values, 1, thrs)
    print(f"{la:^48s}")
    print(f"{lr:^48s}")
    print(f"{ud:^48s}")
    print()
    # Row 2: BL (P0), C (P2), TR (P3)
    for pi in [0, 2, 3]:
        la, _, _ = panel_lines(values, pi, thrs)
        print(f"{la:^14s}  ", end="")
    print()
    for pi in [0, 2, 3]:
        _, lr, _ = panel_lines(values, pi, thrs)
        print(f"{lr:^14s}  ", end="")
    print()
    for pi in [0, 2, 3]:
        _, _, ud = panel_lines(values, pi, thrs)
        print(f"{ud:^14s}  ", end="")
    print("\n")
    # Row 3: BR (P4) centered
    la, lr, ud = panel_lines(values, 4, thrs)
    print(f"{la:^48s}")
    print(f"{lr:^48s}")
    print(f"{ud:^48s}")
    print()
    lo, hi = min(thrs), max(thrs)
    span = f"{lo}" if lo == hi else f"{lo}-{hi}"
    print(f"Thresholds: {span}   Ctrl+C to exit")
    print("Master console: 'o' = zero offsets, 's <i> <v>' = set threshold")


def read_thresholds(ser):
    """Ask the master for the live thresholds. None if it never answers.

    The reply is `t` followed by NUM_SENSORS values; the master emits zeros
    for a panel that did not respond, so the count is always the same.
    """
    ser.write(b"t\n")
    ser.flush()
    deadline = time.time() + THRESHOLD_TIMEOUT
    while time.time() < deadline:
        line = ser.readline().decode("utf-8", errors="replace").strip()
        if line.startswith(">"):
            line = line[1:].strip()  # strip the master's glued-on prompt
        if not line.startswith("t "):
            continue
        parts = line.split()
        if len(parts) != NUM_SENSORS + 1:
            continue
        try:
            return [int(p) for p in parts[1:]]
        except ValueError:
            continue
    return None


def main():
    parser = argparse.ArgumentParser(description="PIUFSR terminal monitor")
    parser.add_argument("port", help="Serial port of the Pro Micro master")
    parser.add_argument("baud", nargs="?", type=int, default=DEFAULT_BAUD)
    parser.add_argument("threshold", nargs="?", type=int, default=None,
                        help="Override every threshold instead of reading them")
    args = parser.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.5)
    except serial.SerialException as e:
        print(f"Could not open {args.port}: {e}", file=sys.stderr)
        sys.exit(1)
    streaming = False
    try:
        time.sleep(2)
        ser.reset_input_buffer()

        if args.threshold is not None:
            thrs = [args.threshold] * NUM_SENSORS
        else:
            thrs = read_thresholds(ser)
            if thrs is None:
                print("Could not read thresholds from the master; using 50.",
                      file=sys.stderr)
                print("(Is another program holding the port open?)",
                      file=sys.stderr)
                time.sleep(2)
                thrs = [50] * NUM_SENSORS
            else:
                # An offline panel reports 0, which would colour every reading
                # red. Fall back to a sane value for those sensors.
                thrs = [t if t > 0 else 50 for t in thrs]

        ser.reset_input_buffer()
        ser.write(b"c\n")
        streaming = True
        time.sleep(0.2)
        ser.reset_input_buffer()

        while True:
            line = ser.readline().decode("utf-8", errors="replace").strip()
            if not line.startswith("c "):
                continue
            parts = line.split()
            if len(parts) != NUM_SENSORS + 1:
                continue
            try:
                values = [int(p) for p in parts[1:]]
            except ValueError:
                continue
            draw(values, thrs)
    except KeyboardInterrupt:
        pass
    finally:
        if streaming:
            # 'c' toggles; a bare newline (what this used to send) is ignored
            # by the master, which left the stream running for the next tool.
            try:
                ser.write(b"c\n")
                ser.flush()
            except serial.SerialException:
                pass
        ser.close()


if __name__ == "__main__":
    main()
