# PIU FSR Dance Pad — Arduino Firmware

## System Overview

```
┌──────────────────┐     ┌────────┐
│  PC (USB HID)    │◄────│ Pro    │──I2C──► Pro Mini ×5
│  Serial (115200) │     │ Micro  │        (0x10-0x14)
│  └─ gamepad      │     │ Master │
│  └─ calibration  │     └────────┘
└──────────────────┘
```

5 Pro Mini slaves each drive one PIU panel. Each panel has:
- 4 FSRs (left, right, up, down sides)
- 16×16 WS2812B LED matrix (256 LEDs, white at brightness 3/255)

A Pro Micro master polls all 5 slaves over I2C at ~1.4 kHz, reports button state as a USB gamepad, and forwards serial commands for calibration/LED ediding.

---

## Hardware Wiring

### I2C Bus (Master ↔ Slaves)
- SDA → A4 (on both Pro Micro and Pro Mini)
- SCL → A5
- 5.1kΩ pull-ups on SDA and SCL to 5V
- 400 kHz clock
- 2 ms timeout per transaction

### FSRs (per slave/panel)
Each FSR forms a voltage divider:
```
5V ── FSR ── A0 ── 5.1kΩ ── GND
```
A0 = Left, A1 = Right, A2 = Up, A3 = Down.

Reading: `analogRead(pin) >> 2` (10-bit → 8-bit).

### LEDs (per slave/panel)
- Data pin: 6
- Type: WS2812B, GRB order
- 256 LEDs (16×16)
- Brightness: 3/255

---

## Panel Mapping (PIU Layout)

```
      ┌──────┐
      │ TL   │  ← I2C 0x11 (P1)
      └──────┘
┌──────┐┌──────┐┌──────┐
│ BL   ││ CENT ││ TR   │  ← 0x10, 0x12, 0x13 (P0, P2, P3)
└──────┘└──────┘└──────┘
      ┌──────┐
      │ BR   │  ← I2C 0x14 (P4)
      └──────┘
```

Panel positions (numpad 17593):
| Div | I2C Addr | Label  | Sensors |
|-----|----------|--------|---------|
| div2| 0x11     | T.LFT  |  4-7    |
| div4| 0x13     | T.RGT  | 12-15   |
| div3| 0x12     | CENTER |  8-11   |
| div1| 0x10     | B.LFT  |  0-3    |
| div5| 0x14     | B.RGT  | 16-19   |

Each panel's FSRs: `[L, R, U, D]` at indices `panel*4 + 0..3`.

---

## Master (`master/master.ino`)

Board: Arduino Pro Micro (ATmega32U4)

### I2C — `twiRead` / `twiWrite`
Custom TWI implementation with 2 ms timeout. If a slave doesn't respond, the transaction is skipped (the panel is treated as inactive).

### Poll Loop
```
pollAllPanels()      →  5-byte read per slave
handleLEDTransitions()  →  send 0x01/0x00 on state change
updateGamepad()         →  pressButton/releaseButton
Joystick.sendState()    →  USB HID report
processSerial()         →  handle commands from PC
```

Polling runs at ~1.4 kHz (5-byte reads each panel).

### Gamepad
5 buttons (one per panel). Button is pressed when `(buf[0] & 0x0F) != 0` (any FSR above threshold).

### Calibration Streaming
When enabled (`c` command), sends at 20 Hz:
```
c <v0> <v1> ... <v19>
```

### Serial Command Protocol

| Input | Action |
|-------|--------|
| `o` | Zero offsets (command `0x05` to all slaves) |
| `v` | Print current compensated values |
| `t` | Print thresholds (reads each slave's 9-byte response) |
| `s` | Save calibration to EEPROM (command `0x07` to all slaves) |
| `s <idx> <val>` | Set threshold for sensor `idx` (0-19) to `val` (0-255) |
| `c` | Toggle streaming mode |
| `u <panel> <slot> <64 hex chars>` | Upload 32-byte bitmap pattern to `panel` (0-4) `slot` (0-3) |
| `x <panel> <x> <y> <0/1>` | Toggle pixel at (x,y) on/off |
| `w <panel> <slot>` | Save current LEDs to EEPROM slot |

---

## Slave (`slave/slave.ino`)

Board: Pro Mini (ATmega328P, 5V/16MHz).

**Must set `I2C_ADDR` per panel before flashing.**

### I2C Response (9 bytes)
```
[0]  fsrActive       bitmask: bit 0=Left,1=Right,2=Up,3=Down
[1..4]  compensated[0..3]  compensated FSR values (raw - offset)
[5..8]  thresholds[0..3]   current threshold values
```

### I2C Commands Received

| Cmd | Args | Action |
|-----|------|--------|
| `0x00` | — | Turn LEDs off |
| `0x01` | — | Turn LEDs on (render patternBuffer) |
| `0x02` | brightness | Set global brightness |
| `0x03` | slot + 32 bytes | Upload bitmap to EEPROM slot |
| `0x04` | slot | Select active pattern slot |
| `0x05` | — | Zero offsets (capture current raw as offsets) |
| `0x06` | idx, value | Set threshold[idx] |
| `0x07` | — | Save calibration to EEPROM |
| `0x08` | — | Load calibration from EEPROM |
| `0x09` | x, y, on | Set pixel (updates leds[] and patternBuffer) |
| `0x0A` | slot | Save current patternBuffer to EEPROM slot |

### FSR Processing (in `loop()`)
```
raw = analogRead(pin) >> 2
compensated = (raw >= offset) ? (raw - offset) : 0
fsrActive bit = (compensated >= threshold)
```

### LED Rendering
- 60 FPS via `millis()` timer
- If `panelOn == true`: calls `ledsFromBitmap()`, then `FastLED.show()`
- If `panelOn == false`: calls `FastLED.clear()`, then `FastLED.show()`

### EEPROM Layout

| Range | Content |
|-------|---------|
| `0..3` | Offsets[0..3] |
| `4..7` | Thresholds[0..3] |
| `8..39` | Pattern slot 0 (32 bytes bitmap) |
| `40..71` | Pattern slot 1 |
| `72..103` | Pattern slot 2 |
| `104..135` | Pattern slot 3 |
| `136` | Active slot index |

---

## Setup / Build

### 1. Flash each slave
1. Set `I2C_ADDR` to 0x10..0x14
2. Wire FSRs to A0-A3 (L/R/U/D)
3. Wire WS2812B data to pin 6
4. Upload `slave/slave.ino`
5. Physical panel → I2C address mapping is up to you

### 2. Flash master
1. Upload `master/master.ino` to Pro Micro
2. Connect SDA/SCL to all slaves (pull-ups on bus)

### 3. First-time calibration
```
o          →  zero offsets (with feet resting on pad)
t          →  verify thresholds
s <i> <v>  →  adjust individual FSR thresholds
s          →  save to EEPROM
c          →  toggle streaming (for visualizer)
```

### 4. LED pattern upload
```
u 0 0 ffffffff...  →  upload 64 hex chars to panel 0 slot 0
w 0 0              →  save to EEPROM
```
