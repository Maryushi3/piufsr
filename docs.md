# PIU FSR Dance Pad — Full Documentation

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

A Pro Micro master polls all 5 slaves over I2C at ~1.4 kHz, reports button state as a USB gamepad, and forwards serial commands for calibration/LED editing.

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
  (see capacitance note below). The AVRs' internal pull-ups (~20-50k) are
  enabled by `Wire.begin()` on every node and are far too weak to matter; the
  external set at the master does the work.
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
- Every slave must have a unique address — run `i` (bus scan) after wiring.
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
- Brightness: 3/255 at boot, adjustable at runtime (`b` command)
- Identity LED: D13 (built-in), blinks panel ID on power-up

### Optional address jumpers (per slave)
`slave.ino` normally takes its address from the compile-time `I2C_ADDR`, so
each panel needs its own build. Setting `ADDR_FROM_JUMPERS` to 1 instead reads
the panel index from three pins (D4/D5/D7 by default), each internally pulled
up and grounded by a jumper to mean 1 bit:

| D7 | D5 | D4 | Panel | Address |
|----|----|----|-------|---------|
| GND | GND | GND | 0 | 0x10 |
| GND | GND | open | 1 | 0x11 |
| GND | open | GND | 2 | 0x12 |
| GND | open | open | 3 | 0x13 |
| open | GND | GND | 4 | 0x14 |
| open | open | open | — | falls back to `I2C_ADDR` |

One binary then flashes to every panel. Off by default so the wiring above is
unchanged unless you opt in.

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
Custom TWI implementation with 12 ms timeout. Every panel is polled every loop — after 5 consecutive failures a panel is reported `offline` (and its cached readings are zeroed so `v`/streaming don't show phantom pressure), but polling continues and it is reported `online` again as soon as it responds.

Timeouts are compared with a signed difference (`(int32_t)(micros() - deadline) >= 0`) so a deadline that wraps past 2^32 doesn't make every transaction fail instantly for ~12 ms once every ~71 minutes.

`twiStop()` waits for the hardware to clear `TWSTO` before returning. That
matters twice over: rewriting `TWCR` early can truncate the STOP and leave the
bus unreleased, and `twiSdaStuck()` would otherwise sample SDA while the
master's own STOP is still holding it low.

If a panel stays unreachable **and SDA is being held low**, the master runs a
bit-banged bus recovery at most once a second: up to 9 SCL clocks (a slave
interrupted mid-byte keeps waiting for the missing clocks while holding SDA
low — the clocks let it finish and release the bus), then a STOP condition and
a TWI re-init. A genuinely absent slave NACKs cleanly and needs no recovery,
so the SDA check keeps the routine from toggling a bus the other panels are
using.

Read frames are sanity-checked: the status byte only uses the low nibble, so
a frame with high bits set is corrupt and counts as a poll failure.

When several panels light up at the same instant, their LED updates can each
clock-stretch a poll by ~8 ms. The 12 ms timeout absorbs this; worst case is
~1 extra frame of gamepad latency for that instant.

### Poll Loop
```
pollAllPanels()         →  5-byte read per slave
handleLEDTransitions()  →  send 0x01/0x00 on state change
updateGamepad()         →  Gamepad.press/release on state change
Gamepad.write()         →  USB HID report, only when state changed
processSerial()         →  handle at most ONE command from the PC
```

Polling runs at ~1.4 kHz (5-byte reads each panel).

`processSerial()` deliberately handles **one line per iteration**. A single
command can cost several I2C transactions, so draining a whole burst (a
calibration UI clicking through slider presets) would starve `pollAllPanels()`
and could outlast the 2 s watchdog, whose only reset is at the top of `loop()`.
A line longer than the 127-char buffer is reported and discarded rather than
executed truncated.

The trade-off: at ~1 command per millisecond, sustained bulk input can outrun
the AVR's 64-byte serial receive buffer and lose characters mid-line. Don't
paste multi-line scripts into a serial monitor. Tools should send one command
and wait for its `Panel <p> OK` reply (or at least the `> ` prompt) before
sending the next — `ledmaker.py` and `cal_web` both do.

The watchdog is armed *after* the boot banner: a host that has the CDC port
open but is not draining it makes `Serial` writes block, and ~700 bytes of
banner could otherwise outlast the 2 s timeout before `loop()` first runs.

### Gamepad
5 buttons (one per panel). Button is pressed when `(buf[0] & 0x0F) != 0` (any FSR above threshold).

**HID reports are only written when the button state changes.** Hosts only
poll a gamepad's USB interrupt endpoint while some program holds the device
open (a game, a tester like `evtest`/joy.cpl, etc.). When nothing polls, a
`Gamepad.write()` can block up to ~250 ms (the USB stack's send timeout), so
writing on every loop iteration would slow the whole master to a few Hz until
a program opens the gamepad. Sending on change keeps the loop at full speed
regardless. Note this also means the mat appears unresponsive on the PC until
*something* has opened the gamepad device — that is host behavior, not a
fault.

Because HID-Project's `write()` returns `void`, a stalled write is detected
by **timing** it: anything ≥ 50 ms hit the ~250 ms timeout, so reporting
backs off for 1 s (state keeps accumulating) and then probes once. While you
only watch the serial monitor, presses therefore cost at most one hiccup per
second instead of one per crossing; with a game attached, writes take
microseconds and the backoff never engages.

A panel going offline forces its button released, so a dead slave cannot leave
a button stuck down. The matching LED-off write is skipped while the panel is
offline and re-attempted once it answers again, so a wedged slave doesn't cost
a 12 ms timeout on every loop iteration.

### Calibration Streaming
When enabled (`c` command), sends at 20 Hz:
```
c <v0> <v1> ... <v19>
```

### Report rate and latency

The nominal rate budget is fixed by constants; the actual rate sags under
specific hardware conditions (all absorbed without data loss):

| Stage | Budget | Source |
|-------|--------|--------|
| Slave FSR sampling | ~1 kHz | 4 × `analogRead` ≈ 440 µs + loop |
| Master panel poll | ~1 kHz | 5 × 5-byte reads @ 400 kHz ≈ 1 ms/loop (`kI2CClock`) |
| HID send pacing | ≤ 1 kHz | `kSendIntervalUs = 1000`, on-change only |
| USB IN polling | 1 kHz | endpoint `bInterval = 1` while a program holds the gamepad open |

What makes it vary: a slave's `FastLED.show()` blacks out its interrupts for
~8 ms, so a poll that lands on a press/release LED update is clock-stretched
(the 12 ms timeout absorbs it); heavy bus capacitance slows edges; serial
prints can block briefly if the host stops draining the port. Press→event
latency is ~2-5 ms typically; release adds the intentional 40 ms
`RELEASE_HOLD_MS` (anti-flap). The game itself samples at its own frame rate
on top of all of this.

**Do not trust browser gamepad testers for rate numbers.** They sample the
Web Gamepad API at `requestAnimationFrame` cadence (display Hz) and coalesce
reports between samples; with the on-change-only reporting in this firmware,
an idle mat correctly sends zero reports, which such sites misread as a low
rate. Measure instead:

1. **At the source** — serial console, `r` before and after playing a song:
   prints loops/sec and the worst single-loop time. ~1000 Hz / worst <12 ms
   means the master is keeping up under real gameplay.
2. **At the kernel** — `evtest /dev/input/eventX` (Linux) prints every event
   with its URB-completion timestamp. Rapid taps should show single-digit-ms,
   evenly spaced event times with no coalesced gaps.
3. **On the wire** — `usbmon` (e.g. Wireshark's USBPcap/usbmon capture) shows
   actual URB completions at 1 ms cadence while any app holds the device open.

### Serial Command Protocol

| Input | Action |
|-------|--------|
| `o` | Zero offsets (command `0x05` to all slaves) |
| `v` | Print cached compensated values |
| `t` | Print thresholds — always 20 values, `0` for an unreachable panel |
| `s` | Save calibration to EEPROM (command `0x07` to all slaves) |
| `s <idx> <val>` | Set threshold for sensor `idx` (0-19) to `val` (0-255) |
| `a <val>` | Set every threshold to `val` — one write per panel (`0x0C`) |
| `c` | Toggle streaming mode |
| `u <panel> <slot> <64 hex chars>` | Upload 32-byte bitmap to `panel` (0-4) `slot` (0-3) |
| `x <panel> <x> <y> <0/1>` | Set pixel at (x,y) on/off — `panel` (0-4), `x`/`y` (0-15) |
| `z <panel>` | Clear the panel's live pattern (`0x0D`) |
| `w <panel> <slot>` | Save live pattern to EEPROM slot |
| `p <panel> <slot>` | Select the active slot — makes it visible and load at boot (`0x04`) |
| `b <panel> <val>` | Set panel brightness 0-255 (`0x02`) |
| `i <panel>` | Trigger identify blink on panel (blinks 1-5 on D13 LED) |
| `i` | Scan the bus: report which panel addresses ACK |
| `r` | Print loop-rate stats since the last `r` (or boot), then reset the window |
| `h` / `?` | Help |

Every panel-directed command is answered with `Panel <p> OK` or
`Panel <p> FAIL`, followed by the `> ` interactive prompt. Malformed or
out-of-range arguments print a usage line instead of being silently coerced
to 0 (a bare `x` used to clear pixel 0,0 of panel 0).

`w` stores a bitmap; it does **not** change which slot the panel displays or
reloads at power-up. Follow it with `p <panel> <slot>` — that is what
`ledmaker.py` does.

---

## Slave (`slave/slave.ino`)

Board: Pro Mini (ATmega328P, 5V/16MHz).

> **FROZEN FIRMWARE:** all slaves have been flashed for the last time. Do not
> modify `slave/slave.ino` — the file must stay byte-identical to what runs
> on the hardware. This document describes the frozen behavior.

**Set `I2C_ADDR` per panel before flashing** (or enable `ADDR_FROM_JUMPERS`).

### Wire buffer limit
Do not try to enlarge Wire's buffers with `#define TWI_BUFFER_LENGTH` /
`BUFFER_LENGTH` in the sketch. `Wire.cpp` and `twi.c` are compiled as a
separate core library, so a macro defined in the sketch never reaches them —
the slave RX buffer stays 32 bytes regardless. Every command is therefore
<= 32 bytes and pattern uploads are chunked (`0x03`).

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
| `0x03` | slot, offset, 8 bytes | Upload one 8-byte chunk of a bitmap; the chunk at offset 24 commits all 32 bytes to the slot |
| `0x04` | slot | Select active pattern slot (persisted, loaded at boot) |
| `0x05` | — | Zero offsets (capture current raw as offsets) |
| `0x06` | idx, value | Set threshold[idx] |
| `0x07` | — | Save calibration to EEPROM |
| `0x08` | — | Load calibration from EEPROM |
| `0x09` | x, y, on | Set pixel (updates leds[] and patternBuffer) |
| `0x0A` | slot | Save current patternBuffer to EEPROM slot |
| `0x0B` | — | Identify: blink LED on D13 (1-5 times) |
| `0x0C` | value | Set all four thresholds to `value` |
| `0x0D` | — | Clear the live pattern |

Argument bytes are consumed before validation, so an out-of-range argument
can never leave payload bytes in the buffer to be misread as the next
command. An unrecognised command byte discards the rest of the frame for the
same reason. Thresholds are clamped to a minimum of 1: at 0 the Schmitt
trigger's release point would also be 0, so the FSR would read as permanently
pressed.

### Deferred operations
Anything slow (EEPROM writes, `analogRead`, `FastLED` calls) is deferred out
of ISR context into `loop()` via a **bitmask** of pending operations, each with
its own argument. A bitmask rather than a single slot, so two commands
arriving back-to-back cannot overwrite each other's work.

`0x0A` saves `patternBuffer`, which set-pixel and upload keep current. It must
not be rebuilt from `leds[]`: `leds[]` is zeroed whenever the master turns the
panel off (a foot anywhere on that panel), so a rebuild would silently save an
empty pattern over the one being edited.

Both bitmap EEPROM writes snapshot the volatile buffer to a plain local with
interrupts off first. A 32-byte write takes up to ~106 ms, and a set-pixel ISR
landing mid-write would otherwise produce a half-old/half-new saved pattern.

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

### Identify blink
Non-blocking: driven from `loop()` off `millis()`. A blocking blink stalled FSR
reads for up to 1.9 s on panel 4, which made the panel dead as an input for two
seconds whenever it was identified. Re-triggering mid-pulse forces the LED low
first, so no pulse is lost from the count. The power-up blink in `setup()` still
runs to completion before the watchdog is armed.

### TWI stall recovery
The slave records I2C activity. If the master has ever talked to it and then
goes quiet for 3 s, `Wire` is torn down and re-initialised (guarded by
`WIRE_HAS_END`). The watchdog cannot cover this case on its own: a wedged TWI
state machine doesn't stop `loop()`, which keeps calling `wdt_reset()`.

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
| `137` | Calibration magic (`0xA5`) |

The magic byte marks "calibration has been written". Freshness used to be
inferred from `offsets[0] == 0xFF`, which would factory-reset a pad whose first
offset legitimately read 255. Writes use `EEPROM.update`, so unchanged bytes
cost no wear and no 3.3 ms erase cycle.

---

## Setup / Build

### 1. Flash each slave
1. Set `I2C_ADDR` to 0x10..0x14 (or enable `ADDR_FROM_JUMPERS`)
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
o          →  zero offsets (with feet off the pad)
t          →  verify thresholds (20 values; 0 means that panel is unreachable)
s <i> <v>  →  adjust an individual FSR threshold
a <v>      →  set every threshold at once
s          →  save to EEPROM
c          →  toggle streaming (for a visualizer)
```

On power-up each slave blinks its panel ID on D13 (1 blink = panel 0, 5 blinks = panel 4) so you can verify the address mapping. Use `i <p>` at any time to re-identify a panel.

### 3a. Browser calibration UI (`cal_web/app.py`)
```
cd cal_web
./setup.sh
venv/bin/python app.py /dev/ttyACM0      # then open http://localhost:8765
venv/bin/python app.py --demo            # no hardware, synthetic readings
```
Thresholds shown in the UI are read back from the slaves at startup (`t`), so
the entry fields reflect what the firmware is actually using rather than a
hardcoded default. Type a value (0–255) in a field and press Enter/blur to set
that sensor; the "All" field or a preset button sets every sensor at once.
"Reload thr" re-reads the fields.

The server binds `127.0.0.1` by default. `/cmd` has no authentication and can
zero and permanently save calibration to every slave's EEPROM, so `--host
0.0.0.0` hands that ability to anyone who can reach the port. Only do it on a
network you trust.

### 4. LED pattern upload

#### Manual hex upload
```
u 0 0 ffffffff...  →  upload 64 hex chars to panel 0 slot 0
p 0 0              →  make slot 0 the active slot
```
The master splits the 32-byte bitmap into four 11-byte I2C frames because
Wire's slave buffer is 32 bytes.

#### Pattern files (`ledmaker/ledmaker.py`)
```
cd ledmaker
./setup.sh
venv/bin/python ledmaker.py /dev/ttyACM0 --load arrow.txt --panel 0 --slot 0
```

Two built-in presets skip the file (useful as panel tests):
```
venv/bin/python ledmaker.py /dev/ttyACM0 --fill all     --panel 0 --slot 0
venv/bin/python ledmaker.py /dev/ttyACM0 --fill center4 --panel 0 --slot 0
```
`all` lights every LED (dead-pixel test); `center4` lights only the 4 center
LEDs in logical coordinates (a quick orientation/layout check). Both upload
to the slot and select it, exactly like `--load`.
A pattern file is 16 lines of 16 characters; `#`, `X`, `x` or `1` mean on,
anything else means off, and lines starting with `;` are comments:
```
;  up arrow
.......##.......
......####......
.....######.....
....########....
...##########...
..############..
.......##.......
.......##.......
.......##.......
.......##.......
.......##.......
.......##.......
.......##.......
.......##.......
................
................
```
The file is parsed before the port is opened, so a malformed file fails
instantly. Upload is 2 commands total (`u` then `p`).

#### Interactive drawing
```
venv/bin/python ledmaker.py /dev/ttyACM0 --panel 0 --slot 0 --out arrow.txt
```
Clears the panel with a single `z` command, then lights each pixel one at a
time and asks `y/N`. `--out` writes the result to a pattern file *before* the
save round-trip, so an hour of clicking survives even if the panel stops
answering at the last step — re-apply it later with `--load`. The path is
checked for writability at startup, before any prompting, and applies only to
an interactive session (there is nothing to capture when uploading a file).

Every command must be confirmed by the master's `Panel <p> OK` reply before the
script moves on (with retries on `FAIL` or timeout); the master's `> ` prompt
fragments and any streaming output are ignored while waiting for a reply.

##### Panel LED layouts

Pixels on the wire are linear indices (`index = y*16 + x`), but each panel's
matrix may be wired/mounted differently — so the preview coordinates only
match the physical panel if ledmaker knows that panel's layout. Layouts are
configured in `PANEL_LAYOUTS` at the top of `ledmaker.py` (one name per
panel, chosen from 32 combinations of row/column × snake/straight ×
rotation/mirror).

To discover a panel's layout, run:

```
venv/bin/python ledmaker.py /dev/ttyACM0 --panel 0 --identify
```

It lights landmark pixels (index 0, 1, 15, 16, 255) one at a time, asks where
each appeared (corner, direction, adjacency), and prints the exact
`PANEL_LAYOUTS` line to paste into the script. Wrong layout = wrong preview and
a mirrored/rotated `--load`, but the panel itself still renders whatever it was
sent (patterns are stored in the panel's own index space).
