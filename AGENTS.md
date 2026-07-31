# AGENTS.md — PIUFSR

## The one rule that matters

**`slave/slave.ino` is FINAL. Never modify it.** All five slaves have been
flashed with this exact code for the last time; any change (even a comment)
makes the file differ from what runs on the hardware. If a slave-side change
ever seems necessary, stop and confirm with the user explicitly.

The master (`master/master.ino`) and the PC tools are easy to reflash/rerun
and may be modified normally.

## Project shape

- `master/master.ino` — Pro Micro (ATmega32U4): I2C master, USB HID gamepad
  (HID-Project lib), serial console (115200). Full docs: `docs.md`.
- `slave/slave.ino` — Pro Mini (ATmega328P): per-panel FSR + LED firmware.
  **Frozen, see above.**
- `ledmaker/ledmaker.py` — LED pattern tool (interactive / `--load` /
  `--fill all|center4` / `--identify`).
- `calibrate/cal.py` — terminal FSR monitor. `cal_web/app.py` — browser UI.
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
- I2C slave commands `0x00`–`0x0D`; Wire slave buffer is 32 bytes and
  **cannot** be enlarged from a sketch — all frames must stay ≤ 32 bytes.
- Gamepad reports are sent on state change only (a write blocks ~250 ms
  when the host isn't polling; the master times writes and backs off).
