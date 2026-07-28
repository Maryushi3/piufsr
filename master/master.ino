#include "HID-Project.h"
#include <avr/wdt.h>

#define NUM_PANELS      5
#define FSRS_PER_PANEL  4
#define NUM_SENSORS     (NUM_PANELS * FSRS_PER_PANEL)

static const uint8_t kPanelAddresses[NUM_PANELS] = {0x10, 0x11, 0x12, 0x13, 0x14};
static const int32_t kI2CClock = 400000L;
static const int32_t kSerialBaud = 115200;
static const int32_t kSendIntervalUs = 1000;
// Must exceed the slave's worst-case interrupt blackout: one 256-LED
// FastLED.show() is ~8 ms. With a shorter timeout the master would abort
// mid-stretch, which can wedge the slave's TWI state machine.
static const uint32_t kI2CTimeoutUs = 12000;
static const uint8_t kMaxPanelFails = 5;

static bool panelActive[NUM_PANELS];
static bool prevPanelActive[NUM_PANELS];
static uint8_t calValues[NUM_SENSORS];
static uint8_t panelFailCount[NUM_PANELS];
static unsigned long lastBusRecoverMs;
static unsigned long lastSendUs;
static unsigned long lastIterationUs;
static bool calibrating;
static unsigned long calPrintUs;

/*===========================================================================*/
/* TWI master with timeout                                                  */
/*===========================================================================*/

#define TWI_READ  1
#define TWI_WRITE 0

static void twiInit() {
  TWBR = (F_CPU / kI2CClock - 16) / 2;
  TWSR = 0;
  PORTD |= _BV(PD0) | _BV(PD1);
  TWCR = _BV(TWEN);
}

static bool twiWait(uint32_t* deadline) {
  while (!(TWCR & _BV(TWINT))) {
    if (micros() > *deadline) {
      TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN);
      uint32_t t = micros() + 100;
      while (!(TWCR & _BV(TWSTO)) && micros() < t);
      TWCR = _BV(TWEN);
      return false;
    }
  }
  return true;
}

static bool twiStart(uint32_t* deadline) {
  TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);
  return twiWait(deadline);
}

static void twiStop() {
  TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN);
}

// Bit-banged bus recovery. A slave interrupted mid-byte can be left waiting
// for the missing bit-clocks while holding SDA low, wedging the whole bus.
// Clocking SCL up to 9 times lets it finish the byte and release SDA,
// followed by a STOP and a TWI re-init. (32U4: PD0 = SCL, PD1 = SDA.)
static void twiBusRecover() {
  TWCR = 0;  // TWI off, pins become GPIO
  // Release both lines (input + pull-up; external 5.1k pull-ups do the work)
  DDRD &= ~(_BV(PD0) | _BV(PD1));
  PORTD |= _BV(PD0) | _BV(PD1);
  delayMicroseconds(10);
  for (uint8_t i = 0; i < 9 && !(PIND & _BV(PD1)); i++) {  // while SDA held low
    PORTD &= ~_BV(PD0);
    DDRD |= _BV(PD0);                                     // SCL low
    delayMicroseconds(10);
    DDRD &= ~_BV(PD0);
    PORTD |= _BV(PD0);                                    // SCL released
    delayMicroseconds(10);
  }
  // STOP condition: SDA low, then SCL high, then SDA high
  PORTD &= ~_BV(PD1);
  DDRD |= _BV(PD1);                                       // SDA low
  delayMicroseconds(10);
  DDRD &= ~_BV(PD0);
  PORTD |= _BV(PD0);                                      // SCL released
  delayMicroseconds(10);
  DDRD &= ~_BV(PD1);
  PORTD |= _BV(PD1);                                      // SDA released
  delayMicroseconds(10);
  twiInit();
}

static bool twiSendAddr(uint8_t addr, bool read, uint32_t* deadline) {
  TWDR = (addr << 1) | (read ? 1 : 0);
  TWCR = _BV(TWINT) | _BV(TWEN);
  if (!twiWait(deadline)) return false;
  uint8_t status = TWSR & 0xF8;
  return read ? (status == 0x40) : (status == 0x18);
}

static bool twiWriteByte(uint8_t data, uint32_t* deadline) {
  TWDR = data;
  TWCR = _BV(TWINT) | _BV(TWEN);
  if (!twiWait(deadline)) return false;
  return (TWSR & 0xF8) == 0x28;
}

static bool twiReadByte(uint8_t* data, bool sendAck, uint32_t* deadline) {
  if (sendAck) {
    TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWEA);
  } else {
    TWCR = _BV(TWINT) | _BV(TWEN);
  }
  if (!twiWait(deadline)) return false;
  *data = TWDR;
  return true;
}

static bool twiRead(uint8_t addr, uint8_t* buf, uint8_t len) {
  uint32_t deadline = micros() + kI2CTimeoutUs;
  if (!twiStart(&deadline)) return false;
  if (!twiSendAddr(addr, TWI_READ, &deadline)) { twiStop(); return false; }
  for (uint8_t i = 0; i < len; i++) {
    if (!twiReadByte(&buf[i], i < len - 1, &deadline)) { twiStop(); return false; }
  }
  twiStop();
  return true;
}

static bool twiWrite(uint8_t addr, uint8_t* data, uint8_t len) {
  uint32_t deadline = micros() + kI2CTimeoutUs;
  if (!twiStart(&deadline)) return false;
  if (!twiSendAddr(addr, TWI_WRITE, &deadline)) { twiStop(); return false; }
  for (uint8_t i = 0; i < len; i++) {
    if (!twiWriteByte(data[i], &deadline)) { twiStop(); return false; }
  }
  twiStop();
  return true;
}

// Probe whether a slave ACKs its address (no data transferred).
static bool twiProbe(uint8_t addr) {
  uint32_t deadline = micros() + kI2CTimeoutUs;
  if (!twiStart(&deadline)) return false;
  bool ok = twiSendAddr(addr, TWI_WRITE, &deadline);
  twiStop();
  return ok;
}

/*===========================================================================*/

// Last state actually sent to the host. Reports are only written when the
// state changes, because Gamepad.write() blocks up to ~250 ms whenever the
// host is not polling the HID interrupt endpoint — and hosts only poll a
// gamepad while some program has it open. Writing unconditionally every loop
// would slow the whole master to a few Hz until a game/tester opens the
// device.
static bool gamepadReported[NUM_PANELS];
static bool gamepadDirty;

static void updateGamepad() {
  for (int p = 0; p < NUM_PANELS; p++) {
    if (panelActive[p] != gamepadReported[p]) {
      gamepadReported[p] = panelActive[p];
      if (panelActive[p]) {
        Gamepad.press(p + 1);
      } else {
        Gamepad.release(p + 1);
      }
      gamepadDirty = true;
    }
  }
}

static void handleLEDTransitions() {
  for (int p = 0; p < NUM_PANELS; p++) {
    if (panelActive[p] != prevPanelActive[p]) {
      uint8_t cmd = panelActive[p] ? 0x01 : 0x00;
      if (twiWrite(kPanelAddresses[p], &cmd, 1)) {
        prevPanelActive[p] = panelActive[p];
      }
    }
  }
}

static void pollAllPanels() {
  for (int p = 0; p < NUM_PANELS; p++) {
    // Never stop polling a panel: a genuinely absent slave fails fast (NACK),
    // and a busy slave (e.g. mid LED update) recovers within a few polls.
    uint8_t buf[5];
    // The status byte only uses the low nibble — a set high bit means a
    // corrupt frame (e.g. two slaves sharing an address); count it as a
    // failure rather than consuming garbage.
    bool ok = twiRead(kPanelAddresses[p], buf, 5) && !(buf[0] & 0xF0);
    if (ok) {
      if (panelFailCount[p] >= kMaxPanelFails) {
        Serial.print(F("Panel "));
        Serial.print(p);
        Serial.println(F(" online"));
      }
      panelFailCount[p] = 0;
      panelActive[p] = (buf[0] & 0x0F) != 0;
      int base = p * FSRS_PER_PANEL;
      for (int i = 0; i < FSRS_PER_PANEL; i++) {
        calValues[base + i] = buf[i + 1];
      }
    } else if (panelFailCount[p] < kMaxPanelFails) {
      panelFailCount[p]++;
      if (panelFailCount[p] >= kMaxPanelFails) {
        Serial.print(F("Panel "));
        Serial.print(p);
        Serial.println(F(" offline"));
      }
    } else if (millis() - lastBusRecoverMs >= 1000) {
      // Panel unreachable for a sustained time: a slave may be wedged
      // mid-transaction and holding the bus. Try to release it.
      twiBusRecover();
      lastBusRecoverMs = millis();
    }
  }
}

static void printPrompt();
static void printHelp();
static void handleIdentify(char* buf);
static void handleScan();

void setup() {
  Serial.begin(kSerialBaud);
  twiInit();
  for (int p = 0; p < NUM_PANELS; p++) {
    panelActive[p] = prevPanelActive[p] = gamepadReported[p] = false;
  }
  Gamepad.begin();
  gamepadDirty = true;  // send the initial all-released state once
  lastSendUs = 0;
  lastIterationUs = 0;
  calibrating = false;
  calPrintUs = 0;
  for (int p = 0; p < NUM_PANELS; p++) panelFailCount[p] = 0;
  lastBusRecoverMs = 0;
  wdt_enable(WDTO_2S);
  Serial.println(F("PIUFSR Master ready."));
  printHelp();
  printPrompt();
}

void loop() {
  unsigned long now = micros();
  wdt_reset();

  pollAllPanels();
  handleLEDTransitions();

  if (now - lastSendUs + lastIterationUs >= kSendIntervalUs) {
    updateGamepad();
    if (gamepadDirty) {
      Gamepad.write();
      gamepadDirty = false;
    }
    lastSendUs = now;
  }

  if (calibrating && now - calPrintUs >= 50000) {
    Serial.print('c');
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(' ');
      Serial.print(calValues[i]);
    }
    Serial.println();
    calPrintUs = now;
  }

  processSerial();

  lastIterationUs = micros() - now;
}

/*===========================================================================*/
/* Serial command processing                                                */
/*===========================================================================*/

static uint8_t hexNibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return 0;
}

static void printPrompt() {
  Serial.print(F("> "));
}

static void printHelp() {
  Serial.println(F("  o          Zero offsets (feet off)"));
  Serial.println(F("  v          Print compensated values"));
  Serial.println(F("  t          Print thresholds"));
  Serial.println(F("  s          Save calibration to EEPROM"));
  Serial.println(F("  s <i> <v>  Set threshold sensor i (0-19) to v (0-255)"));
  Serial.println(F("  c          Toggle streaming 20Hz"));
  Serial.println(F("  u <p> <s> <64hex>  Upload 32B pattern to panel p slot s"));
  Serial.println(F("  x <p> <x> <y> <0/1>  Set pixel on panel p"));
  Serial.println(F("  w <p> <s>  Save pattern to EEPROM slot s"));
  Serial.println(F("  i <p>      Identify panel p (blink LED)"));
  Serial.println(F("  i          Scan bus for panels"));
  Serial.println(F("  h / ?      This help"));
}

static void processSerial() {
  static char line[128];
  static size_t pos = 0;
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (pos == 0) continue;
      line[pos] = '\0';
      pos = 0;
      char* buf = line;

      switch (buf[0]) {
        case 'o': case 'O': handleZeroOffsets(); break;
        case 'v': case 'V': printValues(); break;
        case 't': case 'T': printThresholds(); break;
        case 's': case 'S':
          if (buf[1] == ' ' || buf[1] == '\t' || (buf[1] >= '0' && buf[1] <= '9')) {
            handleSetThreshold(buf + 1);
          } else {
            handleSaveCal();
          }
          break;
        case 'c': case 'C':
          calibrating = !calibrating;
          calPrintUs = micros();
          Serial.print(F("Streaming "));
          Serial.println(calibrating ? F("ON") : F("OFF"));
          break;
        case 'u': case 'U': handleUploadPattern(buf); break;
        case 'x': case 'X': handleSetPixel(buf); break;
        case 'w': case 'W': handleWritePattern(buf); break;
        case 'i': case 'I':
          if (buf[1] == '\0') {
            handleScan();
          } else {
            handleIdentify(buf);
          }
          break;
        case 'h': case 'H': case '?': printHelp(); break;
        default:
          Serial.println(F("Unknown. Type h for help."));
          break;
      }
      printPrompt();
    } else if (pos < sizeof(line) - 1) {
      line[pos++] = c;
    }
  }
}

static void handleZeroOffsets() {
  uint8_t cmd = 0x05;
  for (int p = 0; p < NUM_PANELS; p++) {
    bool ok = twiWrite(kPanelAddresses[p], &cmd, 1);
    Serial.print(F("Panel "));
    Serial.print(p);
    Serial.println(ok ? F(" OK") : F(" FAIL"));
  }
}

static void printValues() {
  Serial.print('v');
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(' ');
    Serial.print(calValues[i]);
  }
  Serial.println();
}

static void printThresholds() {
  Serial.print('t');
  for (int p = 0; p < NUM_PANELS; p++) {
    uint8_t buf[9];
    if (twiRead(kPanelAddresses[p], buf, 9)) {
      for (int i = 0; i < FSRS_PER_PANEL; i++) {
        Serial.print(' ');
        Serial.print(buf[i + 5]);
      }
    }
  }
  Serial.println();
}

static void handleSaveCal() {
  uint8_t cmd = 0x07;
  for (int p = 0; p < NUM_PANELS; p++) {
    bool ok = twiWrite(kPanelAddresses[p], &cmd, 1);
    Serial.print(F("Panel "));
    Serial.print(p);
    Serial.println(ok ? F(" OK") : F(" FAIL"));
  }
}

static void handleSetThreshold(char* buf) {
  char* end = nullptr;
  long idx = strtol(buf, &end, 10);
  if (end == buf || idx < 0 || idx >= NUM_SENSORS) return;
  long val = strtol(end, nullptr, 10);
  if (val < 0 || val > 255) return;
  int panel = idx / FSRS_PER_PANEL;
  int fsr = idx % FSRS_PER_PANEL;
  uint8_t cmd[] = {0x06, (uint8_t)fsr, (uint8_t)val};
  bool ok = twiWrite(kPanelAddresses[panel], cmd, 3);
  Serial.print(F("Panel "));
  Serial.print(panel);
  Serial.println(ok ? F(" OK") : F(" FAIL"));
  printThresholds();
}

static void handleScan() {
  for (int p = 0; p < NUM_PANELS; p++) {
    Serial.print(F("0x"));
    Serial.print(kPanelAddresses[p], HEX);
    Serial.print(F(" (panel "));
    Serial.print(p);
    Serial.print(F("): "));
    Serial.println(twiProbe(kPanelAddresses[p]) ? F("OK") : F("--"));
  }
}

static void handleIdentify(char* buf) {
  char* end = nullptr;
  long panel = strtol(buf + 1, &end, 10);
  if (panel < 0 || panel >= NUM_PANELS) return;
  uint8_t cmd = 0x0B;
  bool ok = twiWrite(kPanelAddresses[panel], &cmd, 1);
  Serial.print(F("Panel "));
  Serial.print(panel);
  Serial.println(ok ? F(" blink OK") : F(" FAIL"));
}

static void handleUploadPattern(char* buf) {
  char* end = nullptr;
  long panel = strtol(buf + 1, &end, 10);
  long slot = strtol(end, &end, 10);
  if (panel < 0 || panel >= NUM_PANELS || slot < 0 || slot >= 4) return;
  while (*end == ' ' || *end == '\t') end++;
  char* hex = end;
  uint8_t data[34];
  data[0] = 0x03;
  data[1] = (uint8_t)slot;
  for (int i = 0; i < 32; i++) {
    if (hex[i * 2] == '\0' || hex[i * 2 + 1] == '\0') return;
    data[i + 2] = (hexNibble(hex[i * 2]) << 4) | hexNibble(hex[i * 2 + 1]);
  }
  bool ok = twiWrite(kPanelAddresses[panel], data, 34);
  Serial.print(F("Panel "));
  Serial.print(panel);
  Serial.println(ok ? F(" OK") : F(" FAIL"));
}

static void handleSetPixel(char* buf) {
  char* end = nullptr;
  long panel = strtol(buf + 1, &end, 10);
  long x = strtol(end, &end, 10);
  long y = strtol(end, &end, 10);
  long on = strtol(end, nullptr, 10);
  if (panel < 0 || panel >= NUM_PANELS || x < 0 || x > 15 || y < 0 || y > 15) return;
  uint8_t cmd[] = {0x09, (uint8_t)x, (uint8_t)y, on ? (uint8_t)1 : (uint8_t)0};
  bool ok = twiWrite(kPanelAddresses[panel], cmd, 4);
  Serial.print(F("Panel "));
  Serial.print(panel);
  Serial.println(ok ? F(" OK") : F(" FAIL"));
}

static void handleWritePattern(char* buf) {
  char* end = nullptr;
  long panel = strtol(buf + 1, &end, 10);
  long slot = strtol(end, nullptr, 10);
  if (panel < 0 || panel >= NUM_PANELS || slot < 0 || slot >= 4) return;
  uint8_t cmd[] = {0x0A, (uint8_t)slot};
  bool ok = twiWrite(kPanelAddresses[panel], cmd, 2);
  Serial.print(F("Panel "));
  Serial.print(panel);
  Serial.println(ok ? F(" OK") : F(" FAIL"));
}
