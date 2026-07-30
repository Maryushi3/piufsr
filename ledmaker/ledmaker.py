#!/usr/bin/env python3
"""LED pattern tool for PIUFSR panels.

Three ways to get a pattern onto a panel, over the Pro Micro master's USB
serial port:

  --load FILE   upload a pattern file in one shot (4 I2C frames, no prompting)
  (default)     interactive: light each pixel and answer y/N, 256 times
  --identify    light landmark pixels to discover a panel's matrix layout

Coordinates are LOGICAL: (x, y) as seen facing the panel, (0,0) = top-left,
y downward. Each panel's LED matrix may be wired/mounted differently, so a
per-panel layout table (PANEL_LAYOUTS below) translates logical positions to
the linear pixel index the slave expects.

Pattern file format: 16 lines of 16 characters. '#', 'X', 'x' or '1' mean on;
anything else means off. Lines starting with ';' are comments.

Protocol notes:
  Every command is answered with "Panel <p> OK" or "Panel <p> FAIL".
  The master also prints a "> " prompt after each reply, which glues onto
  the next line read — replies are therefore matched by line suffix.
  Calibration stream lines ("c ...") and all other output are ignored.

Usage:
    python3 ledmaker.py <port> [--panel N] [--slot N]
    python3 ledmaker.py <port> --load pattern.txt [--panel N] [--slot N]
    python3 ledmaker.py <port> --panel N --identify
"""
import argparse
import sys
import time

import serial

NUM_PANELS = 5
PANEL_SIZE = 16
NUM_LEDS = PANEL_SIZE * PANEL_SIZE
SLOTS = 4
BITMAP_BYTES = NUM_LEDS // 8

DEFAULT_BAUD = 115200
DEFAULT_DELAY = 0.05   # seconds to wait after each command before reading
REPLY_TIMEOUT = 1.0    # max seconds to wait for one reply
MAX_RETRIES = 3

ON_CHARS = "#Xx1"

# ---------------------------------------------------------------------------
# Panel LED layouts
# ---------------------------------------------------------------------------
# The wire protocol addresses pixels linearly: index = y*16 + x. Where each
# index physically lands depends on how that panel's matrix is wired and
# mounted — it differs per panel. A layout maps a logical position (x, y as
# seen facing the panel, (0,0) = top-left, y downward) to the linear index
# the slave expects.

def _row_major(x, y):
    return y * PANEL_SIZE + x


def _row_snake(x, y):
    return y * PANEL_SIZE + (x if y % 2 == 0 else PANEL_SIZE - 1 - x)


def _col_major(x, y):
    return x * PANEL_SIZE + y


def _col_snake(x, y):
    return x * PANEL_SIZE + (y if x % 2 == 0 else PANEL_SIZE - 1 - y)


_BASE_LAYOUTS = {
    "row-major": _row_major,
    "row-snake": _row_snake,
    "col-major": _col_major,
    "col-snake": _col_snake,
}


# Mounting transforms: rot-N = panel rotated N degrees clockwise relative to
# the viewer; flip-h = mirrored left-right.
def _rot90(f):  return lambda x, y: f(y, PANEL_SIZE - 1 - x)
def _rot180(f): return lambda x, y: f(PANEL_SIZE - 1 - x, PANEL_SIZE - 1 - y)
def _rot270(f): return lambda x, y: f(PANEL_SIZE - 1 - y, x)
def _fliph(f):  return lambda x, y: f(PANEL_SIZE - 1 - x, y)


def _build_layouts():
    transforms = [
        ("", lambda f: f),
        (" rot-90", _rot90),
        (" rot-180", _rot180),
        (" rot-270", _rot270),
        (" flip-h", _fliph),
        (" flip-h rot-90", lambda f: _rot90(_fliph(f))),
        (" flip-h rot-180", lambda f: _rot180(_fliph(f))),
        (" flip-h rot-270", lambda f: _rot270(_fliph(f))),
    ]
    return {base + t: tfn(fn)
            for base, fn in _BASE_LAYOUTS.items()
            for t, tfn in transforms}


LAYOUTS = _build_layouts()

# Layout name per panel (list index = panel ID). Run `./ledmaker.py PORT
# --panel N --identify` to discover a panel's layout, then update this table.
PANEL_LAYOUTS = [
    "row-snake",   # panel 0: snake starting top-left (user-reported)
    "row-major",   # panel 1
    "row-major",   # panel 2
    "row-major",   # panel 3
    "row-major",   # panel 4
]


def build_to_index(layout_fn):
    """Table mapping logical (x, y) -> linear pixel index."""
    return [[layout_fn(x, y) for x in range(PANEL_SIZE)]
            for y in range(PANEL_SIZE)]


def invert_table(to_index):
    """Table mapping linear pixel index -> logical (x, y)."""
    to_xy = [None] * NUM_LEDS
    for y in range(PANEL_SIZE):
        for x in range(PANEL_SIZE):
            to_xy[to_index[y][x]] = (x, y)
    return to_xy


NOREPLY_DIAG = """\
ERROR: the master does not reply to commands, so pixel confirmations
cannot be honored. Check, in order:
  1. No other program has the port open right now — CLOSE the serial
     monitor first (also cal.py / cal_web; on Linux, ModemManager or
     brltty can steal the port). Two readers split the incoming data.
  2. The Pro Micro is flashed with the CURRENT master.ino — older
     masters never reply to 'x' commands.
  3. You passed the correct serial port.
Note: this script asserts DTR/RTS at open, which the 32U4 needs before
it transmits anything. If the run above said "no data received at all"
despite all of the above, please share the printed output."""

FAIL_DIAG = """\
ERROR: the master replies, but the slave does not confirm (FAIL).
Check the panel's wiring, slave power, and that the slave's I2C_ADDR
matches the panel ID you chose (use the master's 'i' bus scan)."""


def read_reply(ser):
    """Wait for one command reply from the master.

    Returns (True, ignored) for OK, (False, ignored) for FAIL, or
    (None, ignored) on timeout. `ignored` holds the non-reply lines seen
    while waiting, for diagnostics.
    """
    ignored = []
    deadline = time.time() + REPLY_TIMEOUT
    while time.time() < deadline:
        raw = ser.readline()
        if not raw:
            time.sleep(0.01)
            continue
        line = raw.decode("utf-8", errors="replace").strip()
        if line.startswith(">"):
            line = line[1:].strip()  # strip the master's glued-on prompt
        if line.endswith("OK"):
            return True, ignored
        if line.endswith("FAIL"):
            return False, ignored
        # Anything else (stream lines, banner, help) is not a reply.
        ignored.append(line)
    return None, ignored


def send_command(ser, cmd, delay, retries=MAX_RETRIES):
    """Send a command and require an OK reply, retrying on FAIL/timeout.

    Returns "ok", "fail" (master answers but reports failure), or
    "noreply" (master never answers at all).
    """
    status = "noreply"
    for attempt in range(1, max(1, retries) + 1):
        ser.write((cmd + "\n").encode())
        ser.flush()
        time.sleep(delay)
        resp, ignored = read_reply(ser)
        if resp is True:
            return "ok"
        if resp is False:
            status = "fail"
        print(f"  ! '{cmd}' attempt {attempt}: {'timeout' if resp is None else 'FAIL'}")
        if attempt == max(1, retries):
            if ignored:
                print(f"  (received but not a reply: {ignored[:5]})")
            elif resp is None:
                print("  (no data received from the master at all)")
        time.sleep(0.1)
    return status


def set_index(ser, panel, idx, on, delay, retries):
    """Set a pixel by raw linear index (no layout mapping)."""
    return send_command(
        ser, f"x {panel} {idx % PANEL_SIZE} {idx // PANEL_SIZE} {1 if on else 0}",
        delay, retries)


def set_pixel(ser, panel, to_index, x, y, on, delay, retries):
    """Set the pixel at logical (x, y), translated by the panel's layout."""
    return set_index(ser, panel, to_index[y][x], on, delay, retries)


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


def prompt_choice(prompt, choices):
    """Ask for one of `choices`; empty input returns None (skip)."""
    value = input(prompt).strip().lower()
    if value == "":
        return None
    if value in choices:
        return value
    print(f"  (not one of {'/'.join(choices)} — skipped)")
    return None


IDENTIFY_LANDMARKS = (0, 1, 15, 16, 255)


def run_identify(ser, panel, delay, retries):
    """Light landmark indices and deduce the panel's layout from answers.

    Returns a list of matching layout names ([] if none, None on abort).
    """
    print("\nIdentify mode — face the physical panel upright.")
    print("Landmark pixels will light one at a time; note WHERE each appears.\n")
    input("Press Enter to begin...")
    for idx in IDENTIFY_LANDMARKS:
        if set_index(ser, panel, idx, True, delay, retries) != "ok":
            print("  ABORT: panel did not confirm a command.")
            return None
        input(f"  index {idx:3} is lit — note its position, then Enter...")
        set_index(ser, panel, idx, False, delay, retries)

    corners = {"tl": (0, 0), "tr": (PANEL_SIZE - 1, 0),
               "bl": (0, PANEL_SIZE - 1), "br": (PANEL_SIZE - 1, PANEL_SIZE - 1)}
    directions = {"right": (1, 0), "left": (-1, 0), "down": (0, 1), "up": (0, -1)}

    print("\nAnswer from your notes (Enter = don't know):")
    c0 = prompt_choice("  index 0 was at which corner? [tl/tr/bl/br]: ", corners)
    d1 = prompt_choice("  index 1 was which way from index 0? [right/left/down/up]: ",
                       directions)
    adj = prompt_choice("  were index 15 and 16 next to each other? [y/n]: ", ("y", "n"))
    c255 = prompt_choice("  index 255 was at which corner? [tl/tr/bl/br]: ", corners)

    matches = []
    for name, fn in LAYOUTS.items():
        to_xy = invert_table(build_to_index(fn))
        if c0 and to_xy[0] != corners[c0]:
            continue
        if d1:
            x0, y0 = to_xy[0]
            x1, y1 = to_xy[1]
            if (x1 - x0, y1 - y0) != directions[d1]:
                continue
        if adj is not None:
            x15, y15 = to_xy[15]
            x16, y16 = to_xy[16]
            adjacent = abs(x15 - x16) + abs(y15 - y16) == 1
            if adjacent != (adj == "y"):
                continue
        if c255 and to_xy[NUM_LEDS - 1] != corners[c255]:
            continue
        matches.append(name)
    return matches


def draw_summary(bitmap):
    """Print a 16x16 ASCII view of the chosen pattern."""
    print("Pattern summary:")
    for y in range(PANEL_SIZE):
        print("".join("#" if bitmap[y][x] else "." for x in range(PANEL_SIZE)))
    print(f"Total on: {sum(sum(row) for row in bitmap)} / {NUM_LEDS}")


def read_pattern_file(path):
    """Parse a 16x16 pattern file into a logical bitmap. Raises ValueError."""
    rows = []
    with open(path, encoding="utf-8") as fh:
        for lineno, raw in enumerate(fh, 1):
            line = raw.rstrip("\n").rstrip("\r")
            if line.startswith(";") or line.strip() == "":
                continue
            if len(line) < PANEL_SIZE:
                raise ValueError(
                    f"{path}:{lineno}: row has {len(line)} chars, need {PANEL_SIZE}")
            rows.append([c in ON_CHARS for c in line[:PANEL_SIZE]])
    if len(rows) != PANEL_SIZE:
        raise ValueError(f"{path}: found {len(rows)} rows, need exactly {PANEL_SIZE}")
    return rows


def write_pattern_file(path, bitmap):
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(";  PIUFSR 16x16 pattern ('#' = on)\n")
        for y in range(PANEL_SIZE):
            fh.write("".join("#" if bitmap[y][x] else "." for x in range(PANEL_SIZE)))
            fh.write("\n")


def bitmap_to_hex(bitmap, to_index):
    """Logical bitmap -> 64 hex chars in the panel's linear index space."""
    data = bytearray(BITMAP_BYTES)
    for y in range(PANEL_SIZE):
        for x in range(PANEL_SIZE):
            if bitmap[y][x]:
                idx = to_index[y][x]
                data[idx >> 3] |= 1 << (idx & 7)
    return data.hex()


def preflight(ser, panel, delay, retries):
    """Verify the master confirms commands before doing any real work.

    Uses 'x <panel> 0 0 0' as the probe — the first clear command anyway,
    so it has no side effects beyond the planned clear.
    """
    print("Checking that the master confirms commands...")
    status = set_index(ser, panel, 0, False, delay, retries)
    if status == "ok":
        print("Master link OK.")
        return True
    print(NOREPLY_DIAG if status == "noreply" else FAIL_DIAG)
    return False


def clear_panel(ser, panel, delay, retries):
    """Blank the panel's live pattern with a single command."""
    print("Clearing panel...")
    if send_command(ser, f"z {panel}", delay, retries) != "ok":
        print("  ABORT: panel did not confirm the clear.")
        return False
    return True


def save_and_activate(ser, panel, slot, delay, retries):
    """Save the live pattern to `slot` and make that slot the active one.

    Selecting the slot matters: `w` only stores the bitmap, so without the
    follow-up the panel would keep showing (and reload at power-up) whichever
    slot was active before.
    """
    print(f"Saving pattern to panel {panel} slot {slot}...")
    if send_command(ser, f"w {panel} {slot}", delay, retries) != "ok":
        print("SAVE FAILED — panel did not confirm. Pattern is lost, rerun the tool.")
        return False
    if send_command(ser, f"p {panel} {slot}", delay, retries) != "ok":
        print(f"WARNING: saved to slot {slot}, but the panel did not confirm "
              f"selecting it. Send 'p {panel} {slot}' from the master console.")
        return True
    print("Saved and selected.")
    return True


def run_load(ser, panel, slot, bitmap, to_index, delay, retries):
    """Upload a whole bitmap, then select the slot so it becomes visible."""
    hexdata = bitmap_to_hex(bitmap, to_index)
    print(f"Uploading pattern to panel {panel} slot {slot}...")
    if send_command(ser, f"u {panel} {slot} {hexdata}", delay, retries) != "ok":
        print("UPLOAD FAILED — panel did not confirm.")
        return False
    if send_command(ser, f"p {panel} {slot}", delay, retries) != "ok":
        print(f"WARNING: uploaded to slot {slot}, but selecting it was not "
              f"confirmed. Send 'p {panel} {slot}' from the master console.")
        return True
    print("Uploaded and selected.")
    return True


def run_interactive(ser, panel, slot, to_index, layout_name, out_path,
                    delay, retries):
    if not clear_panel(ser, panel, delay, retries):
        return False

    print(f"Configuring panel {panel} (layout '{layout_name}').")
    print("'y' = on, anything else = off.")
    bitmap = [[False for _ in range(PANEL_SIZE)] for _ in range(PANEL_SIZE)]
    for y in range(PANEL_SIZE):
        for x in range(PANEL_SIZE):
            # Light the pixel so the user can see which one is asked about.
            if set_pixel(ser, panel, to_index, x, y, True,
                         delay, retries) != "ok":
                print(f"  WARNING: could not light preview pixel ({x},{y})")

            on = prompt_bool(f"Pixel ({x:2},{y:2}) on? [y/N]: ", False)

            # Apply the user's final choice (idempotent when it is 'on').
            if set_pixel(ser, panel, to_index, x, y, on,
                         delay, retries) != "ok":
                print(f"  WARNING: panel did not confirm pixel ({x},{y}) = "
                      f"{'on' if on else 'off'} — panel state may differ")
            bitmap[y][x] = on

    # Written before the save round-trip, so an hour of clicking survives even
    # if the panel stops answering at the last step. A failure here must never
    # skip the save that follows.
    if out_path:
        try:
            write_pattern_file(out_path, bitmap)
            print(f"Pattern written to {out_path}")
        except OSError as e:
            print(f"WARNING: could not write {out_path}: {e}")
            print("Pattern (copy this if the save below fails):")
            draw_summary(bitmap)

    ok = save_and_activate(ser, panel, slot, delay, retries)
    draw_summary(bitmap)
    return ok


def main():
    parser = argparse.ArgumentParser(description="PIUFSR LED pattern tool")
    parser.add_argument("port", help="Serial port of the Pro Micro master")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Serial baud rate")
    parser.add_argument("--delay", type=float, default=DEFAULT_DELAY,
                        help=f"Seconds to wait after each command (default {DEFAULT_DELAY})")
    parser.add_argument("--retries", type=int, default=MAX_RETRIES,
                        help=f"Attempts per command before giving up (default {MAX_RETRIES})")
    parser.add_argument("--panel", type=int, choices=range(NUM_PANELS),
                        help="Panel ID (asked interactively when omitted)")
    parser.add_argument("--slot", type=int, choices=range(SLOTS),
                        help="EEPROM slot (asked interactively when omitted)")
    parser.add_argument("--load", metavar="FILE",
                        help="Upload a 16x16 pattern file instead of prompting")
    parser.add_argument("--out", metavar="FILE",
                        help="Also write the interactively drawn pattern here")
    parser.add_argument("--identify", action="store_true",
                        help="Discover the panel's LED layout instead of editing a pattern")
    args = parser.parse_args()

    if args.identify and args.load:
        parser.error("--identify and --load are mutually exclusive")
    if args.out and (args.load or args.identify):
        parser.error("--out only applies to an interactive session")
    if args.out:
        # Checked now, not after 256 prompts: --out exists to protect that work.
        try:
            with open(args.out, "a", encoding="utf-8"):
                pass
        except OSError as e:
            parser.error(f"--out {args.out} is not writable: {e}")

    panel = args.panel
    if panel is None:
        panel = prompt_int("Panel ID (0-4) [0]: ", 0, 0, NUM_PANELS - 1)
    layout_name = PANEL_LAYOUTS[panel]
    if layout_name not in LAYOUTS:
        print(f"ERROR: unknown layout '{layout_name}' for panel {panel}.")
        print("Fix PANEL_LAYOUTS at the top of this file (names are in LAYOUTS).")
        sys.exit(1)
    to_index = build_to_index(LAYOUTS[layout_name])

    bitmap = None
    if args.load:
        # Parsed before the port is opened, so a bad file fails instantly
        # instead of after touching the hardware.
        try:
            bitmap = read_pattern_file(args.load)
        except (OSError, ValueError) as e:
            print(f"ERROR: {e}")
            sys.exit(1)

    slot = 0
    if not args.identify:
        if args.slot is not None:
            slot = args.slot
        else:
            slot = prompt_int("Save to slot (0-3) [0]: ", 0, 0, SLOTS - 1)

    # The 32U4 (Pro Micro) only transmits USB serial data once the host has
    # asserted DTR/RTS (its CDC "line state"); incoming commands work either
    # way. Some pyserial/platform combos don't assert these by default, which
    # silently swallows every master reply while commands still get through.
    # Setting dtr/rts before open() applies them atomically at open.
    ser = serial.Serial()
    ser.port = args.port
    ser.baudrate = args.baud
    ser.timeout = REPLY_TIMEOUT
    ser.dtr = True
    ser.rts = True
    ser.open()
    time.sleep(2)  # let the master settle after opening the port
    ser.reset_input_buffer()

    try:
        if not preflight(ser, panel, args.delay, args.retries):
            sys.exit(1)

        if args.identify:
            matches = run_identify(ser, panel, args.delay, args.retries)
            if matches is None:
                sys.exit(1)
            if not matches:
                print("\nNo known layout matches those observations.")
                print("Note exactly where indices 0, 1, 15, 16, 255 lit and")
                print("ask for a matching layout to be added to LAYOUTS.")
                sys.exit(1)
            print(f"\nMatching layout(s): {', '.join(matches)}")
            print("Set this near the top of ledmaker.py:")
            print(f'    PANEL_LAYOUTS[{panel}] = "{matches[0]}"')
            sys.exit(0)

        if bitmap is not None:
            draw_summary(bitmap)
            ok = run_load(ser, panel, slot, bitmap, to_index,
                          args.delay, args.retries)
        else:
            ok = run_interactive(ser, panel, slot, to_index, layout_name,
                                 args.out, args.delay, args.retries)
        if not ok:
            sys.exit(1)
    except KeyboardInterrupt:
        print("\nAborted by user — nothing was saved.")
        sys.exit(1)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
