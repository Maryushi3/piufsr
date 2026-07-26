#define TWI_BUFFER_LENGTH 64
#define BUFFER_LENGTH 64
#include <Wire.h>
#include <FastLED.h>
#include <EEPROM.h>

#define I2C_ADDR        0x10
#define NUM_FSRS        4
#define NUM_LEDS        256
#define LED_DATA_PIN    6
#define LED_TYPE        WS2812B
#define COLOR_ORDER     GRB
#define LED_BRIGHTNESS  3
#define LED_FPS         60
#define NUM_SLOTS       4
#define BITMAP_BYTES    32

#define EEP_OFF(n)      (n)
#define EEP_THR(n)      ((n) + 4)
#define EEP_PAT(slot)   (8 + (slot) * BITMAP_BYTES)
#define EEP_ACTIVE_SLOT (8 + NUM_SLOTS * BITMAP_BYTES)

static const uint8_t kFsrPins[NUM_FSRS] = {A0, A1, A2, A3};

static uint8_t compensated[NUM_FSRS];
static uint8_t offsets[NUM_FSRS];
static uint8_t thresholds[NUM_FSRS];
static uint8_t fsrActive;
static bool panelActive;

static CRGB leds[NUM_LEDS];
static volatile bool panelOn;
static volatile bool showPending;
static unsigned long lastLEDUpdate;
static uint8_t patternBuffer[BITMAP_BYTES];

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
  memcpy(out + 1, compensated, 4);
  memcpy(out + 5, thresholds, 4);
  Wire.write(out, 9);
}

static void receiveHandler(int numBytes) {
  while (Wire.available() > 0) {
    uint8_t cmd = Wire.read();
    switch (cmd) {
      case 0x00:
        panelOn = false;
        break;
      case 0x01:
        panelOn = true;
        ledsFromBitmap();
        showPending = true;
        break;
      case 0x02:
        if (Wire.available() > 0) FastLED.setBrightness(Wire.read());
        break;
      case 0x03:
        if (Wire.available() >= BITMAP_BYTES + 1) {
          uint8_t slot = Wire.read();
          for (int i = 0; i < BITMAP_BYTES; i++) {
            uint8_t d = Wire.read();
            EEPROM.write(EEP_PAT(slot) + i, d);
            if (slot == EEPROM.read(EEP_ACTIVE_SLOT)) {
              patternBuffer[i] = d;
            }
          }
        }
        break;
      case 0x04:
        if (Wire.available() > 0) {
          uint8_t slot = Wire.read();
          if (slot < NUM_SLOTS) {
            EEPROM.write(EEP_ACTIVE_SLOT, slot);
            loadPattern(slot);
          }
        }
        break;
      case 0x05:
        for (int i = 0; i < NUM_FSRS; i++) {
          uint8_t raw = analogRead(kFsrPins[i]) >> 2;
          offsets[i] = raw;
        }
        break;
      case 0x06:
        if (Wire.available() >= 2) {
          uint8_t idx = Wire.read();
          if (idx < NUM_FSRS) thresholds[idx] = Wire.read();
        }
        break;
      case 0x07:
        saveCalibration();
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
          uint8_t slot = Wire.read();
          if (slot < NUM_SLOTS) {
            bitmapFromLeds();
            savePattern(slot);
          }
        }
        break;
    }
  }
}

static void initPatterns() {
  uint8_t s = EEPROM.read(EEP_ACTIVE_SLOT);
  if (s >= NUM_SLOTS) {
    for (int i = 0; i < BITMAP_BYTES; i++) {
      patternBuffer[i] = 0xFF;
      EEPROM.write(EEP_PAT(0) + i, 0xFF);
    }
    for (int slot = 1; slot < NUM_SLOTS; slot++) {
      for (int i = 0; i < BITMAP_BYTES; i++) {
        EEPROM.write(EEP_PAT(slot) + i, 0);
      }
    }
    EEPROM.write(EEP_ACTIVE_SLOT, 0);
  } else {
    loadPattern(s);
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
  lastLEDUpdate = 0;
}

void loop() {
  for (int i = 0; i < NUM_FSRS; i++) {
    uint8_t raw = analogRead(kFsrPins[i]) >> 2;
    compensated[i] = (raw >= offsets[i]) ? (raw - offsets[i]) : 0;
  }

  fsrActive = 0;
  bool any = false;
  for (int i = 0; i < NUM_FSRS; i++) {
    if (compensated[i] >= thresholds[i]) {
      fsrActive |= 1 << i;
      any = true;
    }
  }
  panelActive = any;

  unsigned long now = millis();
  if (showPending || now - lastLEDUpdate >= 1000 / LED_FPS) {
    lastLEDUpdate = now;
    showPending = false;
    if (panelOn) {
      ledsFromBitmap();
    } else {
      FastLED.clear();
    }
    FastLED.show();
  }
}
