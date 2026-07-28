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

| Signal | Color | Pro Micro (Master) | Pro Mini (Slave) |
|--------|-------|--------------------|------------------|
| SDA    | Green | D2 (pin 5)        | A4 (pin 27)      |
| SCL    | Blue  | D3 (pin 6)        | A5 (pin 28)      |
| 5V     | Red   | VCC                | VCC              |
| GND    | Black | GND                | GND              |

- Pull-ups on SDA and SCL to 5V (one set per bus, at the master — not per slave).
  5.1k is fine for a single short leg; for the full 5-leg bus use 1.5k-2.2k
  (see capacitance note below)
- 400 kHz clock
- 12 ms timeout per transaction — longer than a slave's worst-case interrupt
  blackout (one 256-LED `FastLED.show()` ≈ 8 ms), so a poll that collides with
  an LED update waits out the clock-stretch instead of aborting mid-transaction
- All 5 slaves connected in parallel on the same 4 wires

With 5 slaves on one bus (30-60 cm star legs), the risks are mostly electrical:
- Twist each of SDA and SCL with its own GND wire — never SDA+SCL together
  (that maximizes their mutual coupling). In a parallel bundle, order the
  wires SDA-GND-SCL-VCC so GND sits between the two signals. Twisting cuts
  crosstalk/glitch pickup ~10x at the cost of extra capacitance (handled by
  stronger pull-ups).
- Pull-ups: bus capacitance is the sum of all legs (~60-100 pF/m wire +
  ~10 pF/slave), so a 5-leg star can reach 200-300 pF. At 300 pF, 5.1k gives
  ~1.3 us rise times — far too slow for 400 kHz (0.3 us max). Use one
  1.5k-2.2k set at the master (don't go below ~1.5k at 5V).
- Clock vs report rate (one 5-byte read per panel per loop):
  - 400 kHz → ~1 ms loop → ~1000 Hz reports (gold standard)
  - 200 kHz → ~1.7 ms loop → ~600 Hz reports (big rise-time margin; set
    `kI2CClock = 200000L` if 400 kHz shows `offline`/`online` flaps)
  - 100 kHz → ~3 ms loop → ~300 Hz reports (below the 500 Hz target; last resort)
- Power 5V/GND as a star from the supply; add >=470 uF at each panel's LED
  power input; don't route LED strip current through the Pro Mini's VCC pin —
  the strip's PWM current pulses must not share the I2C ground return. Use
  thick or doubled GND/VCC wire in each leg.
- Optional: ~100 ohm series resistors on SDA/SCL at the master damp ESD and
  edge ringing. Keep the I2C bundle away from LED data lines and FSR analog
  wires (cross at 90° if unavoidable).
- Every slave must have a unique `I2C_ADDR` — run `i` (bus scan) after wiring.
  Two slaves sharing an address both ACK and both drive the bus, producing
  corrupt frames (caught by the master's sanity check) and offline errors

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
- Identity LED: D13 (built-in), blinks panel ID on power-up

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
Custom TWI implementation with 12 ms timeout. Every panel is polled every loop — after 5 consecutive failures a panel is reported `offline`, but polling continues and it is reported `online` again as soon as it responds.

If a panel stays unreachable for ~1 s, the master runs a bit-banged bus
recovery: up to 9 SCL clocks (a slave interrupted mid-byte keeps waiting for
the missing clocks while holding SDA low — the clocks let it finish and
release the bus), then a STOP condition and a TWI re-init.

Read frames are sanity-checked: the status byte only uses the low nibble, so
a frame with high bits set is corrupt and counts as a poll failure.

When several panels light up at the same instant, their LED updates can each
clock-stretch a poll by ~8 ms. The 12 ms timeout absorbs this; worst case is
~1 extra frame of gamepad latency for that instant.

### Poll Loop
```
pollAllPanels()      →  5-byte read per slave
handleLEDTransitions()  →  send 0x01/0x00 on state change
updateGamepad()         →  Gamepad.press/release
Gamepad.write()         →  USB HID report
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
| `x <panel> <x> <y> <0/1>` | Set pixel at (x,y) on/off — `panel` (0-4), `x`/`y` (0-15) |
| `w <panel> <slot>` | Save current LEDs to EEPROM slot — `panel` (0-4) |
| `i <panel>` | Trigger identify blink on panel (blinks 1-5 on D13 LED) |
| `i` | Scan the bus: report which panel addresses ACK |

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
| `0x0B` | — | Identify: blink LED on D13 (1-5 times) |

### FSR Processing (in `loop()`)
```
raw = analogRead(pin) >> 2
compensated = (raw >= offset) ? (raw - offset) : 0
Schmitt trigger: bit set when compensated >= threshold,
                 cleared when compensated < threshold * 3/4
```
Once any FSR goes active, `fsrActive` is held for `RELEASE_HOLD_MS` (40 ms)
after the last hit, so values wobbling near the threshold during a hold don't
flap the panel state (LED flicker / gamepad press-release jitter). Presses
stay instant — only the release is held off.

### LED Rendering
- Event-driven: `FastLED.show()` runs only when the visuals change (panel on/off, pattern/pixel upload, slot select, brightness). WS2812B latches its data, so no periodic refresh is needed.
- This matters for I2C reliability: a 256-LED `show()` blocks interrupts for ~8 ms on the ATmega328P, so a fixed-rate refresh would keep the slave from answering the master ~half of the time.
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
1. Install **HID-Project** library by **NicoHood** (Library Manager)
2. Upload `master/master.ino` to Pro Micro
3. Connect SDA/SCL to all slaves (pull-ups on bus)

### 3. First-time calibration
```
o          →  zero offsets (with feet resting on pad)
t          →  verify thresholds
s <i> <v>  →  adjust individual FSR thresholds
s          →  save to EEPROM
c          →  toggle streaming (for visualizer)
```

On power-up each slave blinks its panel ID on D13 (1 blink = panel 0, 5 blinks = panel 4) so you can verify the address mapping. Use `i <p>` at any time to re-identify a panel.

### 4. LED pattern upload

#### Manual hex upload
```
u 0 0 ffffffff...  →  upload 64 hex chars to panel 0 slot 0
w 0 0              →  save to EEPROM
```

#### Interactive LED maker (`ledmaker/ledmaker.py`)
```
cd ledmaker
./setup.sh                  # create venv + install pyserial
source venv/bin/activate
./ledmaker.py /dev/ttyACM0  # use the Pro Micro's port
```

The script asks for a panel (0-4, default 0) and a save slot (0-3, default 0).
It clears every pixel, then lights each pixel one at a time and asks `y/N` to
turn it on. After the last pixel it saves the bitmap to the chosen panel/slot.
The master replies `OK`/`FAIL` after each pixel so the script can retry on I2C
errors; a small default delay is inserted between commands to avoid overrunning
the panel while it updates the WS2812 matrix.
