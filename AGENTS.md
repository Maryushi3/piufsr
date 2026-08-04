# AGENTS.md — PIUFSR

## The one rule that matters

**`slave/slave.ino` is FINAL.** All five slaves have been flashed with this
exact code; any change (even a comment) makes the file differ from what runs
on the hardware. Do not modify it on your own initiative. If a slave-side
change ever seems necessary, stop and ask the user explicitly — they can
override this rule if there is a real need.

The master (`master/master.ino`) and the PC tools are easy to reflash/rerun
and may be modified normally.

## Project shape

- `master/master.ino` — Pro Micro (ATmega32U4): I2C master, USB HID gamepad
  (HID-Project lib), serial console (115200). Full docs: `docs.md`.
- `slave/slave.ino` — Pro Mini (ATmega328P): per-panel FSR + LED firmware.
  Slaves toggle their own LEDs from their FSR state; the master's game loop is
  read-only on the I2C bus.
- `ledmaker/ledmaker.py` — LED pattern tool (interactive / `--load` /
  `--fill all|center4` / `--identify`).
- `cal_web/app.py` — browser calibration UI (press + release thresholds).
- `docs.md` — full hardware/protocol documentation. Keep it in sync when
  changing master behavior or the serial/I2C protocols.

## Verifying firmware changes

No local Arduino install is required; use an isolated toolchain:

```sh
cd <temp-dir>
./arduino-cli core install arduino:avr
# libs FastLED + HID-Project must be fetched manually into ./libraries
./arduino-cli compile --libraries libraries \
  --fqbn arduino:avr:pro:cpu=16MHzatmega328 <repo>/slave   # Pro Mini 5V/16MHz
./arduino-cli compile --libraries libraries \
  --fqbn arduino:avr:leonardo <repo>/master                # Pro Micro
```

Expected sizes (ballpark): slave ~8.0 kB flash / ~1.2 kB RAM; master ~14 kB
flash / ~0.45 kB RAM. Always compile before committing firmware changes.

## Protocol contract (do not break)

- Master serial replies `Panel <p> OK` / `Panel <p> FAIL` + `> ` prompt;
  ledmaker.py matches these by line suffix.
- I2C slave commands `0x00`–`0x10`; Wire slave buffer is 32 bytes and
  **cannot** be enlarged from a sketch — all frames must stay ≤ 32 bytes.
- Gamepad reports are sent on state change only (a write blocks ~250 ms
  when the host isn't polling; the master times writes and backs off).
