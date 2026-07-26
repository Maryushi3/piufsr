#!/usr/bin/env python3
import serial
import sys
import time

LABELS = ["B.LFT", "T.LFT", "CENTER", "T.RGT", "B.RGT"]
INDICES = [0, 1, 2, 3, 4]  # BL, TL, C, TR, BR (sensors 0-3, 4-7, ...)

def clear():
    print("\033[H\033[J", end="")

def colored(val, thr):
    if val >= thr:
        return f"\033[31m{val:3d}\033[0m"
    if val >= thr // 2:
        return f"\033[33m{val:3d}\033[0m"
    return f"{val:3d}"

def panel_lines(vals, pi, thr):
    """Return [label_line, lr_line, ud_line] for a panel."""
    b = pi * 4
    label = f"P{pi} {LABELS[pi]}"
    lr = f" L[{colored(vals[b], thr)}] R[{colored(vals[b+1], thr)}] "
    ud = f" U[{colored(vals[b+2], thr)}] D[{colored(vals[b+3], thr)}] "
    return label, lr, ud

def draw(values, thr):
    clear()
    # Row 1: TL (P1) centered
    la, lr, ud = panel_lines(values, 1, thr)
    print(f"{la:^48s}")
    print(f"{lr:^48s}")
    print(f"{ud:^48s}")
    print()
    # Row 2: BL (P0), C (P2), TR (P3)
    for pi in [0, 2, 3]:
        la, _, _ = panel_lines(values, pi, thr)
        print(f"{la:^14s}  ", end="")
    print()
    for pi in [0, 2, 3]:
        _, lr, _ = panel_lines(values, pi, thr)
        print(f"{lr:^14s}  ", end="")
    print()
    for pi in [0, 2, 3]:
        _, _, ud = panel_lines(values, pi, thr)
        print(f"{ud:^14s}  ", end="")
    print("\n")
    # Row 3: BR (P4) centered
    la, lr, ud = panel_lines(values, 4, thr)
    print(f"{la:^48s}")
    print(f"{lr:^48s}")
    print(f"{ud:^48s}")
    print()
    print(f"Threshold: {thr}   Ctrl+C to exit")
    print("Serial: 'o' = zero offsets, '<s> <v>' = set threshold")

def main():
    if len(sys.argv) < 2:
        print("usage: %s <serial_port> [baud] [threshold]" % sys.argv[0])
        sys.exit(1)

    port = sys.argv[1]
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    thr = int(sys.argv[3]) if len(sys.argv) > 3 else 50

    ser = serial.Serial(port, baud, timeout=0.5)
    time.sleep(2)
    ser.reset_input_buffer()
    ser.write(b"c\n")
    time.sleep(0.2)
    ser.reset_input_buffer()

    try:
        while True:
            line = ser.readline().decode("utf-8", errors="replace").strip()
            if not line.startswith("c "):
                continue
            parts = line.split()
            if len(parts) != 21:
                continue
            draw([int(p) for p in parts[1:]], thr)
    except KeyboardInterrupt:
        pass
    finally:
        ser.write(b"\n")
        ser.close()

if __name__ == "__main__":
    main()
