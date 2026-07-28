# PIU FSR Dance Pad

DIY Pump It Up dance pad: 5 FSR panels with LED matrices, reporting as a USB gamepad.

```
┌──────────────────┐     ┌────────┐
│  PC (USB HID)    │◄────│ Pro    │──I2C──► Pro Mini ×5
│  gamepad/serial  │     │ Micro  │        (0x10-0x14)
└──────────────────┘     │ Master │
                         └────────┘
```

Each panel: 4 FSRs + a 16×16 WS2812B matrix, driven by a Pro Mini slave.
The Pro Micro master polls all slaves over I2C (~1.4 kHz), reports button
state as a USB gamepad, and forwards serial commands for calibration and LED
editing.

## Repo layout

| Path | What |
|------|------|
| `master/` | Pro Micro firmware (USB gamepad + I2C master + serial console) |
| `slave/`  | Pro Mini firmware (FSR read + LED render, one per panel) |
| `ledmaker/` | Interactive tool for drawing per-panel LED patterns |

## Quick start

1. **Flash each slave**: set `I2C_ADDR` (0x10–0x14) in `slave/slave.ino`, upload. Each slave blinks its panel ID on D13 at power-up.
2. **Flash the master**: install the **HID-Project** library (NicoHood), upload `master/master.ino`.
3. **Calibrate**: open the serial console (115200), run `o` (zero), tune with `s <i> <v>`, then `s` (save). Send `h` for all commands.
4. **Play**: any program that opens the gamepad (game, `evtest`, joy.cpl) starts input delivery.
5. **LED patterns**: `ledmaker/setup.sh`, then `./ledmaker.py <port>` — draw patterns pixel by pixel; `--identify` discovers each panel's matrix layout.

Full wiring, protocol, and tuning details: **[docs.md](docs.md)**
