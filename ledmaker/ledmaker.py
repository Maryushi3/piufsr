#!/usr/bin/env python3
"""Interactive LED pattern maker for PIUFSR panels.

Walks through one panel pixel at a time over the Pro Micro master's USB
serial port:
  1. asks for a panel ID (0-4) and a save slot (0-3),
  2. clears the whole panel pixel by pixel, waiting for the master to
     confirm each set-pixel command (with retries),
  3. lights each pixel one at a time and asks whether to keep it on,
  4. saves the resulting bitmap to the panel's EEPROM slot.

Protocol notes:
  Every command is answered with "Panel <p> OK" or "Panel <p> FAIL".
  The master also prints a "> " prompt after each reply, which glues onto
  the next line read — replies are therefore matched by line suffix.
  Calibration stream lines ("c ...") and all other output are ignored.

Usage:
    python3 ledmaker.py <serial_port> [--baud 115200] [--delay 0.05] [--retries 3]
"""
import argparse
import sys
import time

import serial

NUM_PANELS = 5
PANEL_SIZE = 16
NUM_LEDS = PANEL_SIZE * PANEL_SIZE
SLOTS = 4

DEFAULT_BAUD = 115200
DEFAULT_DELAY = 0.05   # seconds to wait after each command before reading
REPLY_TIMEOUT = 1.0    # max seconds to wait for one reply
MAX_RETRIES = 3

NOREPLY_DIAG = """\
ERROR: the master does not reply to commands, so pixel confirmations
cannot be honored. Check, in order:
  1. The Pro Micro is flashed with the CURRENT master.ino — older
     masters never reply to 'x' commands. Reflash it and retry.
     (The slaves do NOT need reflashing.)
  2. No other program has the port open (serial monitor, cal.py,
     cal_web; on Linux also ModemManager / brltty can steal the port).
  3. You passed the correct serial port."""

FAIL_DIAG = """\
ERROR: the master replies, but the slave does not confirm (FAIL).
Check the panel's wiring, slave power, and that the slave's I2C_ADDR
matches the panel ID you chose (use the master's 'i' bus scan)."""


def read_reply(ser):
    """Wait for one command reply from the master.

    Returns True for OK, False for FAIL, None on timeout.
    """
    deadline = time.time() + REPLY_TIMEOUT
    while time.time() < deadline:
        line = ser.readline().decode("utf-8", errors="replace").strip()
        if line.startswith(">"):
            line = line[1:].strip()  # strip the master's glued-on prompt
        if line.endswith("OK"):
            return True
        if line.endswith("FAIL"):
            return False
        # Anything else (stream lines, banner, help) is not a reply.
    return None


def send_command(ser, cmd, delay, retries=MAX_RETRIES):
    """Send a command and require an OK reply, retrying on FAIL/timeout.

    Returns "ok", "fail" (master answers but reports failure), or
    "noreply" (master never answers at all).
    """
    status = "noreply"
    for attempt in range(1, retries + 1):
        ser.write((cmd + "\n").encode())
        ser.flush()
        time.sleep(delay)
        resp = read_reply(ser)
        if resp is True:
            return "ok"
        if resp is False:
            status = "fail"
        print(f"  ! '{cmd}' attempt {attempt}: {'timeout' if resp is None else 'FAIL'}")
        time.sleep(0.1)
    return status


def set_pixel(ser, panel, x, y, on, delay, retries):
    return send_command(ser, f"x {panel} {x} {y} {1 if on else 0}", delay, retries)


def prompt_int(prompt, default, low, high):
    """Ask for an integer in [low, high]; default on empty input."""
    value = input(prompt).strip()
    if value == "":
        return default
    try:
        value = int(value)
        if low <= value <= high:
            return value
    except ValueError:
        pass
    print(f"Invalid input, using default {default}")
    return default


def prompt_bool(prompt, default):
    """Ask for y/n; default on empty input."""
    value = input(prompt).strip().lower()
    if value == "":
        return default
    return value.startswith("y")


def draw_summary(bitmap):
    """Print a 16x16 ASCII view of the chosen pattern."""
    print("Pattern summary:")
    for y in range(PANEL_SIZE):
        print("".join("#" if bitmap[y][x] else "." for x in range(PANEL_SIZE)))
    print(f"Total on: {sum(sum(row) for row in bitmap)} / {NUM_LEDS}")


def preflight(ser, panel, delay, retries):
    """Verify the master confirms commands before doing any real work.

    Uses 'x <panel> 0 0 0' as the probe — the first clear command anyway,
    so it has no side effects beyond the planned clear.
    """
    print("Checking that the master confirms commands...")
    status = set_pixel(ser, panel, 0, 0, False, delay, retries)
    if status == "ok":
        print("Master link OK.")
        return True
    print(NOREPLY_DIAG if status == "noreply" else FAIL_DIAG)
    return False


def clear_panel(ser, panel, delay, retries):
    """Set every pixel off, requiring an OK for each one."""
    print("Clearing panel (256 pixels, this takes ~20 s)...")
    for y in range(PANEL_SIZE):
        print(f"  row {y + 1}/{PANEL_SIZE}")
        for x in range(PANEL_SIZE):
            if set_pixel(ser, panel, x, y, False, delay, retries) != "ok":
                print(f"  ABORT: panel did not confirm clear of pixel ({x},{y})")
                return False
    return True


def main():
    parser = argparse.ArgumentParser(description="Interactive LED pattern maker")
    parser.add_argument("port", help="Serial port of the Pro Micro master")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Serial baud rate")
    parser.add_argument("--delay", type=float, default=DEFAULT_DELAY,
                        help=f"Seconds to wait after each command (default {DEFAULT_DELAY})")
    parser.add_argument("--retries", type=int, default=MAX_RETRIES,
                        help=f"Attempts per pixel before giving up (default {MAX_RETRIES})")
    args = parser.parse_args()

    panel = prompt_int("Panel ID (0-4) [0]: ", 0, 0, NUM_PANELS - 1)
    slot = prompt_int("Save to slot (0-3) [0]: ", 0, 0, SLOTS - 1)

    ser = serial.Serial(args.port, args.baud, timeout=REPLY_TIMEOUT)
    time.sleep(2)  # let the master settle after opening the port
    ser.reset_input_buffer()

    try:
        if not preflight(ser, panel, args.delay, args.retries):
            sys.exit(1)

        if not clear_panel(ser, panel, args.delay, args.retries):
            sys.exit(1)

        print(f"Configuring panel {panel}. 'y' = on, anything else = off.")
        bitmap = [[False for _ in range(PANEL_SIZE)] for _ in range(PANEL_SIZE)]
        for y in range(PANEL_SIZE):
            for x in range(PANEL_SIZE):
                # Light the pixel so the user can see which one is asked about.
                if set_pixel(ser, panel, x, y, True, args.delay, args.retries) != "ok":
                    print(f"  WARNING: could not light preview pixel ({x},{y})")

                on = prompt_bool(f"Pixel ({x:2},{y:2}) on? [y/N]: ", False)

                # Apply the user's final choice (idempotent when it is 'on').
                if set_pixel(ser, panel, x, y, on, args.delay, args.retries) != "ok":
                    print(f"  WARNING: panel did not confirm pixel ({x},{y}) = "
                          f"{'on' if on else 'off'} — panel state may differ")
                bitmap[y][x] = on

        print(f"Saving pattern to panel {panel} slot {slot}...")
        if send_command(ser, f"w {panel} {slot}", args.delay, args.retries) == "ok":
            print("Saved.")
        else:
            print("SAVE FAILED — panel did not confirm. Pattern is lost, rerun the tool.")
            sys.exit(1)

        draw_summary(bitmap)
    except KeyboardInterrupt:
        print("\nAborted by user — nothing was saved.")
        sys.exit(1)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
