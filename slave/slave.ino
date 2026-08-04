// NOTE: do NOT try to enlarge Wire's buffers with
//   #define TWI_BUFFER_LENGTH / BUFFER_LENGTH
// here. Wire.cpp and twi.c are compiled as a separate core library, so a
// macro defined in this sketch never reaches them: the slave RX buffer stays
// 32 bytes no matter what. Every command below is therefore <= 32 bytes, and
// pattern uploads are chunked (command 0x03).
#include <Wire.h>
#include <FastLED.h>
#include <EEPROM.h>
#include <avr/wdt.h>

// Default I2C address (0x10 + panel index). Must be unique per panel.
#define I2C_ADDR        0x10

// Optional jumper addressing: set to 1 to derive the address from three pins
// so one binary can be flashed to every panel. Each pin is pulled up
// internally; a jumper to GND pulls that bit low, and the 3-bit value is the
// panel index (panel 0 = all three grounded). All three left open reads as
// 0b111, which means "no jumpers fitted" and falls back to I2C_ADDR above.
#define ADDR_FROM_JUMPERS 0
#define ADDR_PIN_BIT0   4
#define ADDR_PIN_BIT1   5
#define ADDR_PIN_BIT2   7

#define ID_LED_PIN      13
#define NUM_FSRS        4
#define NUM_LEDS        256
#define PANEL_W         16
#define LED_DATA_PIN    6
#define LED_TYPE        WS2812B
#define COLOR_ORDER     GRB
#define LED_BRIGHTNESS  3
#define NUM_SLOTS       4
#define BITMAP_BYTES    32
#define UPLOAD_CHUNK    8

// EEPROM layout is fixed: pattern slots live at 8..135 and must never move.
// Release thresholds are new — they go in the previously-unused bytes after
// the calibration magic, so existing patterns/calibration are left untouched.
#define EEP_OFF(n)      (n)
#define EEP_THR(n)      ((n) + 4)
#define EEP_PAT(slot)   (8 + (slot) * BITMAP_BYTES)
#define EEP_ACTIVE_SLOT (8 + NUM_SLOTS * BITMAP_BYTES)
#define EEP_CAL_MAGIC   (EEP_ACTIVE_SLOT + 1)
#define EEP_REL(n)      (138 + (n))
#define EEP_REL_MAGIC   (142)

static const uint8_t kFsrPins[NUM_FSRS] = {A0, A1, A2, A3};
static const uint8_t kCalMagic = 0xA5;
static const uint8_t kDefaultThreshold = 125;
// The Schmitt trigger clears an FSR bit below this value, independently of the
// press threshold. Anchoring it near the offset floor (rather than scaling
// with the press threshold) makes release track the FSR's real mechanical
// relaxation, so lights and the gamepad go off as soon as the foot leaves.
static const uint8_t kDefaultReleaseThreshold = 20;
// A press threshold of 0 would set the bit whenever compensated > 0, leaving
// no room for a release edge. 1 is the lowest usable value.
static const uint8_t kMinThreshold = 1;
static const uint16_t kIdentityOnMs = 120;
static const uint16_t kIdentityGapMs = 200;
// If the master has ever talked to us and then goes quiet for this long, the
// TWI slave state machine is assumed wedged. loop() keeps running (and keeps
// petting the watchdog) in that state, so the watchdog can never recover it —
// re-init Wire explicitly instead.
static const uint16_t kI2cStallMs = 3000;

static uint8_t i2cAddress = I2C_ADDR;

static volatile uint8_t compensated[NUM_FSRS];
static volatile uint8_t offsets[NUM_FSRS];
static volatile uint8_t thresholds[NUM_FSRS];
static volatile uint8_t releaseThrs[NUM_FSRS];
static volatile uint8_t fsrActive;

static CRGB leds[NUM_LEDS];
static volatile bool panelOn;
static volatile bool showPending;
// While set, the LED display is pinned by the master (pattern editing) and
// ignores the FSR state. Any foot press clears it, handing control back to
// gameplay; the master can also clear it explicitly via command 0x0E.
static volatile bool ledManualHold;
static volatile bool identifyPending;
// Written from the I2C receive ISR (set-pixel / upload into the active slot),
// read by loop() — must be volatile.
static volatile uint8_t patternBuffer[BITMAP_BYTES];

static uint8_t lastActiveSlot;

// One bit per deferred operation, so two commands arriving back-to-back can
// no longer overwrite each other. Each op keeps its own argument.
#define OP_STORE_UPLOAD   0x01  // write uploadBuffer to uploadSlot
#define OP_SAVE_CAL       0x02
#define OP_SAVE_PATTERN   0x04  // write patternBuffer to saveSlot
#define OP_ZERO_OFFSETS   0x08
#define OP_SET_BRIGHTNESS 0x10
#define OP_SELECT_SLOT    0x20
#define OP_LOAD_CAL       0x40

static volatile uint8_t pendingOps;
static volatile uint8_t uploadBuffer[BITMAP_BYTES];
static volatile uint8_t uploadSlot;
static volatile uint8_t saveSlot;
static volatile uint8_t selectSlot;
static volatile uint8_t pendingBrightness;

static volatile bool i2cActivity;
static bool i2cEverSeen;
static unsigned long lastI2cMs;

static uint8_t identityBlinksLeft;
static bool identityLedOn;
static bool identityActive;
static unsigned long identityNextMs;

static uint8_t panelIndex() {
  uint8_t idx = i2cAddress - 0x10;
  return (idx < 5) ? idx : 0;
}

static uint8_t resolveAddress() {
#if ADDR_FROM_JUMPERS
  pinMode(ADDR_PIN_BIT0, INPUT_PULLUP);
  pinMode(ADDR_PIN_BIT1, INPUT_PULLUP);
  pinMode(ADDR_PIN_BIT2, INPUT_PULLUP);
  delayMicroseconds(50);  // let the pull-ups settle
  uint8_t v = (digitalRead(ADDR_PIN_BIT2) << 2) |
              (digitalRead(ADDR_PIN_BIT1) << 1) |
              digitalRead(ADDR_PIN_BIT0);
  if (v < 5) return 0x10 + v;  // 5..7 = invalid / no jumpers fitted
#endif
  return I2C_ADDR;
}

static uint8_t clampThreshold(uint8_t v) {
  return (v < kMinThreshold) ? kMinThreshold : v;
}

// The release edge of the Schmitt trigger. Stored independently of the press
// threshold, but clamped to keep a hysteresis gap (and to never go below 1,
// otherwise the bit would never clear).
static uint8_t effectiveRelease(int i) {
  uint8_t thr = thresholds[i];
  uint8_t rel = releaseThrs[i];
  if (thr <= 1) return 1;
  if (rel >= thr) rel = thr - 1;
  if (rel < 1) rel = 1;
  return rel;
}

static void saveCalibration() {
  for (int i = 0; i < NUM_FSRS; i++) {
    EEPROM.update(EEP_OFF(i), offsets[i]);
    EEPROM.update(EEP_THR(i), thresholds[i]);
    EEPROM.update(EEP_REL(i), releaseThrs[i]);
  }
  EEPROM.update(EEP_CAL_MAGIC, kCalMagic);
  EEPROM.update(EEP_REL_MAGIC, kCalMagic);
}

static void loadCalibration() {
  // A magic byte marks "calibration has been written", so a legitimate stored
  // value of 0xFF can no longer be mistaken for a blank chip.
  if (EEPROM.read(EEP_CAL_MAGIC) != kCalMagic) {
    for (int i = 0; i < NUM_FSRS; i++) {
      offsets[i] = 0;
      thresholds[i] = kDefaultThreshold;
      releaseThrs[i] = kDefaultReleaseThreshold;
    }
    saveCalibration();
  } else {
    for (int i = 0; i < NUM_FSRS; i++) {
      offsets[i] = EEPROM.read(EEP_OFF(i));
      thresholds[i] = clampThreshold(EEPROM.read(EEP_THR(i)));
    }
    // Release thresholds postdate the calibration magic: a slave flashed with
    // older firmware has blank (0xFF) bytes here and no REL magic. Give those
    // the default without disturbing the stored patterns or offsets. Writes
    // use EEPROM.update, so unchanged bytes cost nothing.
    if (EEPROM.read(EEP_REL_MAGIC) != kCalMagic) {
      for (int i = 0; i < NUM_FSRS; i++) releaseThrs[i] = kDefaultReleaseThreshold;
      saveCalibration();
    } else {
      for (int i = 0; i < NUM_FSRS; i++) releaseThrs[i] = EEPROM.read(EEP_REL(i));
    }
  }
}

// Copy a volatile bitmap out to a plain local with interrupts off, so a
// set-pixel ISR landing mid-copy cannot produce a half-old/half-new snapshot.
// Worth it because the EEPROM write that follows takes ~3.3 ms per changed
// byte — up to ~106 ms of exposure for a 32-byte bitmap.
static void snapshotBitmap(volatile uint8_t* src, uint8_t* dst) {
  cli();
  for (int i = 0; i < BITMAP_BYTES; i++) dst[i] = src[i];
  sei();
}

static void restoreBitmap(const uint8_t* src, volatile uint8_t* dst) {
  cli();
  for (int i = 0; i < BITMAP_BYTES; i++) dst[i] = src[i];
  sei();
}

static void loadPattern(uint8_t slot) {
  uint8_t buf[BITMAP_BYTES];
  for (int i = 0; i < BITMAP_BYTES; i++) buf[i] = EEPROM.read(EEP_PAT(slot) + i);
  restoreBitmap(buf, patternBuffer);
}

static void savePatternBytes(uint8_t slot, const uint8_t* src) {
  for (int i = 0; i < BITMAP_BYTES; i++) {
    EEPROM.update(EEP_PAT(slot) + i, src[i]);
  }
}

// Only the bitmap is touched: renderIfDirty() rebuilds all 256 leds[] entries
// from it, so there is no need to walk the LED array from ISR context.
static void clearPattern() {
  for (int i = 0; i < BITMAP_BYTES; i++) patternBuffer[i] = 0;
}

/*===========================================================================*/
/* Identity blink (non-blocking)                                             */
/*===========================================================================*/

static void startIdentity() {
  identityBlinksLeft = panelIndex() + 1;
  // Forced low in case a previous blink is still mid-pulse: otherwise the
  // first service call would drive an already-high LED high again, costing one
  // visible pulse — on a feature whose whole job is being countable.
  digitalWrite(ID_LED_PIN, LOW);
  identityLedOn = false;
  identityActive = true;
  identityNextMs = millis();
}

// Drives the blink from loop() instead of blocking in delay(). A blocking
// blink stalled FSR reads for up to 1.9 s, which made the panel dead as an
// input for two seconds whenever it was identified.
static void serviceIdentity() {
  if (!identityActive) return;
  unsigned long now = millis();
  if ((long)(now - identityNextMs) < 0) return;
  if (identityLedOn) {
    digitalWrite(ID_LED_PIN, LOW);
    identityLedOn = false;
    if (--identityBlinksLeft == 0) {
      identityActive = false;
      return;
    }
    identityNextMs = now + kIdentityGapMs;
  } else {
    digitalWrite(ID_LED_PIN, HIGH);
    identityLedOn = true;
    identityNextMs = now + kIdentityOnMs;
  }
}

static void ledsFromBitmap() {
  for (int i = 0; i < NUM_LEDS; i++) {
    uint8_t b = patternBuffer[i >> 3];
    uint8_t m = 1 << (i & 7);
    leds[i] = (b & m) ? CRGB::White : CRGB(0, 0, 0);
  }
}

/*===========================================================================*/
/* I2C handlers (ISR context — defer anything slow via pendingOps)           */
/*===========================================================================*/

static void requestHandler() {
  uint8_t out[13];
  out[0] = fsrActive;
  for (int i = 0; i < NUM_FSRS; i++) out[1 + i] = compensated[i];
  for (int i = 0; i < NUM_FSRS; i++) out[5 + i] = thresholds[i];
  for (int i = 0; i < NUM_FSRS; i++) out[9 + i] = releaseThrs[i];
  Wire.write(out, 13);
  i2cActivity = true;
}

static void receiveHandler(int numBytes) {
  (void)numBytes;
  i2cActivity = true;
  while (Wire.available() > 0) {
    uint8_t cmd = Wire.read();
    switch (cmd) {
      case 0x00:
        ledManualHold = true;
        panelOn = false;
        showPending = true;
        break;
      case 0x01:
        ledManualHold = true;
        panelOn = true;
        showPending = true;
        break;
      case 0x02:
        if (Wire.available() > 0) {
          pendingBrightness = Wire.read();
          pendingOps |= OP_SET_BRIGHTNESS;
        }
        break;
      case 0x03:
        // [slot][offset][UPLOAD_CHUNK bytes]. Chunked because Wire's slave RX
        // buffer is 32 bytes and a whole 32-byte bitmap would not fit in one
        // frame. Every argument byte is consumed before validation, so a bad
        // slot/offset cannot leave payload bytes in the buffer to be
        // misparsed as the next command.
        if (Wire.available() >= 2 + UPLOAD_CHUNK) {
          uint8_t slot = Wire.read();
          uint8_t off = Wire.read();
          uint8_t chunk[UPLOAD_CHUNK];
          for (uint8_t i = 0; i < UPLOAD_CHUNK; i++) chunk[i] = Wire.read();
          if (slot < NUM_SLOTS && (off % UPLOAD_CHUNK) == 0 &&
              off <= BITMAP_BYTES - UPLOAD_CHUNK) {
            uploadSlot = slot;
            for (uint8_t i = 0; i < UPLOAD_CHUNK; i++) {
              uploadBuffer[off + i] = chunk[i];
            }
            if (off + UPLOAD_CHUNK >= BITMAP_BYTES) {
              pendingOps |= OP_STORE_UPLOAD;
            }
          }
        }
        break;
      case 0x04:
        if (Wire.available() > 0) {
          uint8_t slot = Wire.read();
          if (slot < NUM_SLOTS) {
            selectSlot = slot;
            pendingOps |= OP_SELECT_SLOT;
          }
        }
        break;
      case 0x05:
        pendingOps |= OP_ZERO_OFFSETS;
        break;
      case 0x06:
        if (Wire.available() >= 2) {
          uint8_t idx = Wire.read();
          uint8_t val = Wire.read();  // consumed even when idx is invalid
          if (idx < NUM_FSRS) thresholds[idx] = clampThreshold(val);
        }
        break;
      case 0x07:
        pendingOps |= OP_SAVE_CAL;
        break;
      case 0x08:
        // Deferred: loadCalibration() can write EEPROM, which would block
        // this ISR (and hold the bus) for tens of milliseconds.
        pendingOps |= OP_LOAD_CAL;
        break;
      case 0x09:
        if (Wire.available() >= 3) {
          uint8_t x = Wire.read();
          uint8_t y = Wire.read();
          uint8_t on = Wire.read();
          if (x < PANEL_W && y < PANEL_W) {
            int idx = y * PANEL_W + x;
            if (on) {
              leds[idx] = CRGB::White;
              patternBuffer[idx >> 3] |= 1 << (idx & 7);
            } else {
              leds[idx] = CRGB(0, 0, 0);
              patternBuffer[idx >> 3] &= ~(1 << (idx & 7));
            }
            panelOn = true;
            ledManualHold = true;
            showPending = true;
          }
        }
        break;
      case 0x0A:
        if (Wire.available() > 0) {
          uint8_t slot = Wire.read();
          if (slot < NUM_SLOTS) {
            saveSlot = slot;
            pendingOps |= OP_SAVE_PATTERN;
          }
        }
        break;
      case 0x0B:
        identifyPending = true;
        break;
      case 0x0C:
        if (Wire.available() > 0) {
          uint8_t val = Wire.read();
          for (int i = 0; i < NUM_FSRS; i++) thresholds[i] = clampThreshold(val);
        }
        break;
      case 0x0D:
        clearPattern();
        ledManualHold = true;
        panelOn = true;
        showPending = true;
        break;
      case 0x0E:
        // Re-enter FSR-driven LED mode after a pattern-editing session.
        ledManualHold = false;
        break;
      case 0x0F:
        if (Wire.available() >= 2) {
          uint8_t idx = Wire.read();
          uint8_t val = Wire.read();  // consumed even when idx is invalid
          if (idx < NUM_FSRS) releaseThrs[idx] = val;
        }
        break;
      case 0x10:
        if (Wire.available() > 0) {
          uint8_t val = Wire.read();
          for (int i = 0; i < NUM_FSRS; i++) releaseThrs[i] = val;
        }
        break;
      default:
        // Unknown command: the rest of the frame cannot be interpreted
        // safely, so drop it rather than parsing payload bytes as commands.
        while (Wire.available() > 0) Wire.read();
        break;
    }
  }
}

static void initPatterns() {
  uint8_t s = EEPROM.read(EEP_ACTIVE_SLOT);
  if (s >= NUM_SLOTS) {
    for (int slot = 0; slot < NUM_SLOTS; slot++) {
      for (int i = 0; i < BITMAP_BYTES; i++) {
        EEPROM.update(EEP_PAT(slot) + i, 0);
      }
    }
    for (int i = 0; i < BITMAP_BYTES; i++) patternBuffer[i] = 0;
    EEPROM.update(EEP_ACTIVE_SLOT, 0);
    lastActiveSlot = 0;
  } else {
    loadPattern(s);
    lastActiveSlot = s;
  }
}

static void wireStart() {
  Wire.begin(i2cAddress);
  Wire.onRequest(requestHandler);
  Wire.onReceive(receiveHandler);
}

void setup() {
  i2cAddress = resolveAddress();
  // Pull-ups live at the master only (see docs.md); Wire.begin() already
  // enables the AVR's internal ones, so nothing extra is done here.
  wireStart();

  FastLED.addLeds<LED_TYPE, LED_DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(LED_BRIGHTNESS);
  FastLED.clear();
  FastLED.show();

  loadCalibration();
  initPatterns();
  panelOn = false;
  ledManualHold = false;

  pinMode(ID_LED_PIN, OUTPUT);
  startIdentity();
  while (identityActive) serviceIdentity();  // blocking is fine before wdt_enable

  lastI2cMs = millis();
  wdt_enable(WDTO_4S);
}

static void readFsrs() {
  for (int i = 0; i < NUM_FSRS; i++) {
    uint8_t raw = analogRead(kFsrPins[i]) >> 2;
    uint8_t off = offsets[i];
    compensated[i] = (raw >= off) ? (raw - off) : 0;
  }

  // Schmitt trigger per FSR: set at >= press threshold, clear below the
  // release threshold. Anchoring the release edge near the offset floor (not
  // scaled off the press threshold) makes release track the FSR's actual
  // mechanical relaxation — no fixed hold delay is needed, so the state
  // clears as soon as the foot leaves.
  uint8_t active = fsrActive;
  for (int i = 0; i < NUM_FSRS; i++) {
    uint8_t thr = thresholds[i];
    uint8_t rel = effectiveRelease(i);
    if (compensated[i] >= thr) {
      active |= 1 << i;
    } else if (compensated[i] < rel) {
      active &= ~(1 << i);
    }
  }
  fsrActive = active;

  // LED auto-toggle: while no panel command has pinned the display, the LED
  // follows the FSR state 1:1 (same bit the master forwards as the gamepad
  // button). The first foot press after a pattern-editing session clears
  // ledManualHold, handing the display back to gameplay.
  if (active != 0) ledManualHold = false;
  if (!ledManualHold) {
    bool want = (fsrActive != 0);
    if (want != panelOn) {
      panelOn = want;
      showPending = true;
    }
  }
}

static void processPendingOps() {
  cli();
  uint8_t ops = pendingOps;
  pendingOps = 0;
  uint8_t uSlot = uploadSlot;
  uint8_t sSlot = saveSlot;
  uint8_t nSlot = selectSlot;
  uint8_t bright = pendingBrightness;
  sei();
  if (!ops) return;

  if (ops & OP_STORE_UPLOAD) {
    uint8_t snap[BITMAP_BYTES];
    snapshotBitmap(uploadBuffer, snap);
    savePatternBytes(uSlot, snap);
    if (uSlot == lastActiveSlot) {
      restoreBitmap(snap, patternBuffer);
      ledManualHold = true;
      showPending = true;
    }
  }
  if (ops & OP_SAVE_CAL) {
    saveCalibration();
  }
  if (ops & OP_SAVE_PATTERN) {
    // Saves patternBuffer, which set-pixel and upload keep current. It must
    // NOT be rebuilt from leds[]: leds[] is zeroed whenever the master turns
    // the panel off (a foot on the pad), so a rebuild would silently save an
    // empty pattern over the one being edited.
    uint8_t snap[BITMAP_BYTES];
    snapshotBitmap(patternBuffer, snap);
    savePatternBytes(sSlot, snap);
  }
  if (ops & OP_ZERO_OFFSETS) {
    for (int i = 0; i < NUM_FSRS; i++) {
      offsets[i] = analogRead(kFsrPins[i]) >> 2;
    }
  }
  if (ops & OP_SET_BRIGHTNESS) {
    FastLED.setBrightness(bright);
    showPending = true;
  }
  if (ops & OP_SELECT_SLOT) {
    lastActiveSlot = nSlot;
    EEPROM.update(EEP_ACTIVE_SLOT, nSlot);
    loadPattern(nSlot);
    ledManualHold = true;
    showPending = true;
  }
  if (ops & OP_LOAD_CAL) {
    loadCalibration();
  }
}

static void serviceIdentityRequest() {
  cli();
  bool req = identifyPending;
  identifyPending = false;
  sei();
  if (req) startIdentity();
}

static void renderIfDirty() {
  // WS2812B latches its data, so only push a frame when the visuals actually
  // changed. A 256-LED FastLED.show() blocks interrupts for ~8 ms, which
  // stalls this slave's I2C responses — refreshing at a fixed rate would
  // keep the bus unavailable ~half of the time.
  cli();
  bool dirty = showPending;
  showPending = false;
  sei();
  if (!dirty) return;
  if (panelOn) {
    ledsFromBitmap();
  } else {
    FastLED.clear();
  }
  FastLED.show();
}

static void serviceI2cStall(unsigned long now) {
  if (i2cActivity) {
    i2cActivity = false;
    i2cEverSeen = true;
    lastI2cMs = now;
  } else if (i2cEverSeen && (unsigned long)(now - lastI2cMs) >= kI2cStallMs) {
#if defined(WIRE_HAS_END)
    Wire.end();   // guarded: end() is absent from very old AVR cores
#endif
    wireStart();
    lastI2cMs = now;
  }
}

void loop() {
  wdt_reset();

  readFsrs();
  processPendingOps();
  serviceIdentityRequest();
  serviceIdentity();
  renderIfDirty();
  serviceI2cStall(millis());
}
