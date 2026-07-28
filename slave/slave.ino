#define TWI_BUFFER_LENGTH 64
#define BUFFER_LENGTH 64
#include <Wire.h>
#include <FastLED.h>
#include <EEPROM.h>
#include <avr/wdt.h>

#define I2C_ADDR        0x10
#define ID_LED_PIN      13
#define NUM_FSRS        4
#define NUM_LEDS        256
#define LED_DATA_PIN    6
#define LED_TYPE        WS2812B
#define COLOR_ORDER     GRB
#define LED_BRIGHTNESS  3
#define NUM_SLOTS       4
#define BITMAP_BYTES    32
#define RELEASE_HOLD_MS 40

#define EEP_OFF(n)      (n)
#define EEP_THR(n)      ((n) + 4)
#define EEP_PAT(slot)   (8 + (slot) * BITMAP_BYTES)
#define EEP_ACTIVE_SLOT (8 + NUM_SLOTS * BITMAP_BYTES)

static const uint8_t kFsrPins[NUM_FSRS] = {A0, A1, A2, A3};

static volatile uint8_t compensated[NUM_FSRS];
static volatile uint8_t offsets[NUM_FSRS];
static volatile uint8_t thresholds[NUM_FSRS];
static volatile uint8_t fsrActive;
static bool panelActive;

static CRGB leds[NUM_LEDS];
static volatile bool panelOn;
static volatile bool showPending;
static volatile bool identifyPending;
static uint8_t patternBuffer[BITMAP_BYTES];
static unsigned long lastActiveMs;

static uint8_t lastActiveSlot;
static volatile uint8_t pendingOp;
static volatile uint8_t pendingSlot;
static volatile uint8_t pendingPattern[BITMAP_BYTES];
static volatile uint8_t pendingBrightness;

static void loadCalibration() {
  if (EEPROM.read(EEP_OFF(0)) == 0xFF) {
    for (int i = 0; i < NUM_FSRS; i++) {
      offsets[i] = 0;
      thresholds[i] = 125;
      EEPROM.write(EEP_OFF(i), 0);
      EEPROM.write(EEP_THR(i), 125);
    }
  } else {
    for (int i = 0; i < NUM_FSRS; i++) {
      offsets[i] = EEPROM.read(EEP_OFF(i));
      thresholds[i] = EEPROM.read(EEP_THR(i));
    }
  }
}

static void saveCalibration() {
  for (int i = 0; i < NUM_FSRS; i++) {
    EEPROM.write(EEP_OFF(i), offsets[i]);
    EEPROM.write(EEP_THR(i), thresholds[i]);
  }
}

static void loadPattern(uint8_t slot) {
  for (int i = 0; i < BITMAP_BYTES; i++) {
    patternBuffer[i] = EEPROM.read(EEP_PAT(slot) + i);
  }
}

static void savePattern(uint8_t slot) {
  for (int i = 0; i < BITMAP_BYTES; i++) {
    EEPROM.write(EEP_PAT(slot) + i, patternBuffer[i]);
  }
}

static void blinkIdentity() {
  uint8_t n = (I2C_ADDR & 0x0F);
  n++;
  pinMode(ID_LED_PIN, OUTPUT);
  for (uint8_t i = 0; i < n; i++) {
    digitalWrite(ID_LED_PIN, HIGH);
    delay(120);
    digitalWrite(ID_LED_PIN, LOW);
    if (i < n - 1) delay(200);
  }
  delay(500);
}

static void ledsFromBitmap() {
  for (int i = 0; i < NUM_LEDS; i++) {
    uint8_t b = patternBuffer[i >> 3];
    uint8_t m = 1 << (i & 7);
    leds[i] = (b & m) ? CRGB::White : CRGB(0, 0, 0);
  }
}

static void bitmapFromLeds() {
  for (int i = 0; i < BITMAP_BYTES; i++) {
    patternBuffer[i] = 0;
  }
  for (int i = 0; i < NUM_LEDS; i++) {
    if (leds[i].r != 0 || leds[i].g != 0 || leds[i].b != 0) {
      patternBuffer[i >> 3] |= 1 << (i & 7);
    }
  }
}

static void requestHandler() {
  uint8_t out[9];
  out[0] = fsrActive;
  for (int i = 0; i < 4; i++) out[1 + i] = compensated[i];
  for (int i = 0; i < 4; i++) out[5 + i] = thresholds[i];
  Wire.write(out, 9);
}

static void receiveHandler(int numBytes) {
  while (Wire.available() > 0) {
    uint8_t cmd = Wire.read();
    switch (cmd) {
      case 0x00:
        panelOn = false;
        showPending = true;
        break;
      case 0x01:
        panelOn = true;
        showPending = true;
        break;
      case 0x02:
        if (Wire.available() > 0) {
          pendingBrightness = Wire.read();
          pendingOp = 5;
        }
        break;
      case 0x03:
        if (Wire.available() >= BITMAP_BYTES + 1) {
          pendingSlot = Wire.read();
          for (int i = 0; i < BITMAP_BYTES; i++) {
            uint8_t d = Wire.read();
            pendingPattern[i] = d;
            if (pendingSlot == lastActiveSlot) {
              patternBuffer[i] = d;
            }
          }
          if (pendingSlot == lastActiveSlot) {
            showPending = true;
          }
          pendingOp = 1;
        }
        break;
      case 0x04:
        if (Wire.available() > 0) {
          pendingSlot = Wire.read();
          if (pendingSlot < NUM_SLOTS) {
            pendingOp = 6;
          }
        }
        break;
      case 0x05:
        pendingOp = 4;
        break;
      case 0x06:
        if (Wire.available() >= 2) {
          uint8_t idx = Wire.read();
          if (idx < NUM_FSRS) thresholds[idx] = Wire.read();
        }
        break;
      case 0x07:
        pendingOp = 2;
        break;
      case 0x08:
        loadCalibration();
        break;
      case 0x09:
        if (Wire.available() >= 3) {
          uint8_t x = Wire.read();
          uint8_t y = Wire.read();
          uint8_t on = Wire.read();
          int idx = y * 16 + x;
          if (idx < NUM_LEDS) {
            if (on) {
              leds[idx] = CRGB::White;
              patternBuffer[idx >> 3] |= 1 << (idx & 7);
            } else {
              leds[idx] = CRGB(0, 0, 0);
              patternBuffer[idx >> 3] &= ~(1 << (idx & 7));
            }
            panelOn = true;
            showPending = true;
          }
        }
        break;
      case 0x0A:
        if (Wire.available() > 0) {
          pendingSlot = Wire.read();
          if (pendingSlot < NUM_SLOTS) {
            bitmapFromLeds();
            pendingOp = 3;
          }
        }
        break;
      case 0x0B:
        identifyPending = true;
        break;
    }
  }
}

static void initPatterns() {
  uint8_t s = EEPROM.read(EEP_ACTIVE_SLOT);
  if (s >= NUM_SLOTS) {
    for (int i = 0; i < BITMAP_BYTES; i++) {
      patternBuffer[i] = 0;
      EEPROM.write(EEP_PAT(0) + i, 0);
    }
    for (int slot = 1; slot < NUM_SLOTS; slot++) {
      for (int i = 0; i < BITMAP_BYTES; i++) {
        EEPROM.write(EEP_PAT(slot) + i, 0);
      }
    }
    EEPROM.write(EEP_ACTIVE_SLOT, 0);
    lastActiveSlot = 0;
  } else {
    loadPattern(s);
    lastActiveSlot = s;
  }
}

void setup() {
  Wire.begin(I2C_ADDR);
  PORTC |= _BV(PC4) | _BV(PC5);
  Wire.onRequest(requestHandler);
  Wire.onReceive(receiveHandler);

  FastLED.addLeds<LED_TYPE, LED_DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(LED_BRIGHTNESS);
  FastLED.clear();
  FastLED.show();

  loadCalibration();
  initPatterns();
  panelOn = false;
  blinkIdentity();
  wdt_enable(WDTO_4S);
}

void loop() {
  wdt_reset();
  for (int i = 0; i < NUM_FSRS; i++) {
    uint8_t raw = analogRead(kFsrPins[i]) >> 2;
    uint8_t off = offsets[i];
    compensated[i] = (raw >= off) ? (raw - off) : 0;
  }

  // Schmitt trigger per FSR: set at >= threshold, clear below 3/4 of it,
  // so a value wobbling right at the threshold doesn't flap the bit.
  uint8_t active = fsrActive;
  for (int i = 0; i < NUM_FSRS; i++) {
    uint8_t thr = thresholds[i];
    if (compensated[i] >= thr) {
      active |= 1 << i;
    } else if (compensated[i] < thr - (thr >> 2)) {
      active &= ~(1 << i);
    }
  }

  // Keep reporting the panel as active for a short time after the last hit.
  // Without this, FSR values hovering near the threshold during a hold make
  // the panel state flap, which the master forwards as LED on/off spam
  // (visible flicker) and gamepad press/release jitter. Presses stay
  // instant — only the release is held off.
  unsigned long now = millis();
  if (active != 0) {
    fsrActive = active;
    lastActiveMs = now;
  } else if ((unsigned long)(now - lastActiveMs) >= RELEASE_HOLD_MS) {
    fsrActive = 0;
  }
  panelActive = (fsrActive != 0);

  // WS2812B latches its data, so only push a frame when the visuals actually
  // changed. A 256-LED FastLED.show() blocks interrupts for ~8 ms, which
  // stalls this slave's I2C responses — refreshing at a fixed rate would
  // keep the bus unavailable ~half of the time.
  if (showPending) {
    showPending = false;
    if (panelOn) {
      ledsFromBitmap();
    } else {
      FastLED.clear();
    }
    FastLED.show();
  }

  if (identifyPending) {
    identifyPending = false;
    blinkIdentity();
  }

  cli();
  uint8_t op = pendingOp;
  if (op) pendingOp = 0;
  sei();
  if (op) {
    switch (op) {
      case 1:
        for (int i = 0; i < BITMAP_BYTES; i++) {
          EEPROM.write(EEP_PAT(pendingSlot) + i, pendingPattern[i]);
        }
        break;
      case 2:
        saveCalibration();
        break;
      case 3:
        savePattern(pendingSlot);
        break;
      case 4:
        for (int i = 0; i < NUM_FSRS; i++) {
          uint8_t raw = analogRead(kFsrPins[i]) >> 2;
          offsets[i] = raw;
        }
        break;
      case 5:
        FastLED.setBrightness(pendingBrightness);
        showPending = true;
        break;
      case 6:
        lastActiveSlot = pendingSlot;
        EEPROM.write(EEP_ACTIVE_SLOT, pendingSlot);
        loadPattern(pendingSlot);
        showPending = true;
        break;
    }
  }
}
