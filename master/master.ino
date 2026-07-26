#include <Joystick.h>

#define NUM_PANELS      5
#define FSRS_PER_PANEL  4
#define NUM_SENSORS     (NUM_PANELS * FSRS_PER_PANEL)

static const uint8_t kPanelAddresses[NUM_PANELS] = {0x10, 0x11, 0x12, 0x13, 0x14};
static const int32_t kI2CClock = 400000L;
static const int32_t kSerialBaud = 115200;
static const int32_t kSendIntervalUs = 1000;
static const uint32_t kI2CTimeoutUs = 2000;

static bool panelActive[NUM_PANELS];
static bool prevPanelActive[NUM_PANELS];
static uint8_t calValues[NUM_SENSORS];
static unsigned long lastSendUs;
static int loopTimeUs;
static bool calibrating;
static unsigned long calPrintUs;

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_GAMEPAD, 5, 0,
                   false, false, false, false, false, false,
                   false, false, false, false, false);

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

/*===========================================================================*/

static void sendLEDCommand(int panel, bool on) {
  uint8_t cmd = on ? 0x01 : 0x00;
  twiWrite(kPanelAddresses[panel], &cmd, 1);
}

static void updateGamepad() {
  for (int p = 0; p < NUM_PANELS; p++) {
    if (panelActive[p]) {
      Joystick.pressButton(p + 1);
    } else {
      Joystick.releaseButton(p + 1);
    }
  }
}

static void handleLEDTransitions() {
  for (int p = 0; p < NUM_PANELS; p++) {
    if (panelActive[p] != prevPanelActive[p]) {
      sendLEDCommand(p, panelActive[p]);
      prevPanelActive[p] = panelActive[p];
    }
  }
}

static void pollAllPanels() {
  for (int p = 0; p < NUM_PANELS; p++) {
    uint8_t buf[5];
    if (twiRead(kPanelAddresses[p], buf, 5)) {
      panelActive[p] = (buf[0] & 0x0F) != 0;
      int base = p * FSRS_PER_PANEL;
      for (int i = 0; i < FSRS_PER_PANEL; i++) {
        calValues[base + i] = buf[i + 1];
      }
    }
  }
}

void setup() {
  Serial.begin(kSerialBaud);
  twiInit();
  for (int p = 0; p < NUM_PANELS; p++) panelActive[p] = prevPanelActive[p] = false;
  Joystick.begin(false);
  lastSendUs = 0;
  loopTimeUs = -1;
  calibrating = false;
  calPrintUs = 0;
}

void loop() {
  unsigned long now = micros();

  pollAllPanels();
  handleLEDTransitions();

  if (loopTimeUs == -1 || now - lastSendUs + loopTimeUs >= kSendIntervalUs) {
    updateGamepad();
    Joystick.sendState();
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

  if (loopTimeUs == -1) {
    loopTimeUs = micros() - now;
  }
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

static void processSerial() {
  while (Serial.available() > 0) {
    static char buf[128];
    size_t len = Serial.readBytesUntil('\n', buf, sizeof(buf) - 1);
    buf[len] = '\0';
    if (len == 0) return;

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
      case 'c': case 'C': calibrating = !calibrating; calPrintUs = micros(); break;
      case 'u': case 'U': handleUploadPattern(buf); break;
      case 'x': case 'X': handleSetPixel(buf); break;
      case 'w': case 'W': handleWritePattern(buf); break;
      default:  handleSetThreshold(buf); break;
    }
  }
}

static void handleZeroOffsets() {
  uint8_t cmd = 0x05;
  for (int p = 0; p < NUM_PANELS; p++) {
    twiWrite(kPanelAddresses[p], &cmd, 1);
  }
  Serial.println(F("Offsets updated on all panels."));
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
    twiWrite(kPanelAddresses[p], &cmd, 1);
  }
  Serial.println(F("Calibration saved."));
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
  twiWrite(kPanelAddresses[panel], cmd, 3);
  printThresholds();
}

static void handleUploadPattern(char* buf) {
  char* end = nullptr;
  long panel = strtol(buf + 1, &end, 10);
  long slot = strtol(end, &end, 10);
  if (panel < 0 || panel >= NUM_PANELS || slot < 0 || slot >= 4) return;
  char* hex = end;
  uint8_t data[34];
  data[0] = 0x03;
  data[1] = (uint8_t)slot;
  for (int i = 0; i < 32; i++) {
    if (hex[i * 2] == '\0' || hex[i * 2 + 1] == '\0') return;
    data[i + 2] = (hexNibble(hex[i * 2]) << 4) | hexNibble(hex[i * 2 + 1]);
  }
  if (twiWrite(kPanelAddresses[panel], data, 34)) {
    Serial.print(F("Uploaded to panel "));
    Serial.print(panel);
    Serial.print(F(" slot "));
    Serial.println(slot);
  } else {
    Serial.println(F("Upload failed"));
  }
}

static void handleSetPixel(char* buf) {
  char* end = nullptr;
  long panel = strtol(buf + 1, &end, 10);
  long x = strtol(end, &end, 10);
  long y = strtol(end, &end, 10);
  long on = strtol(end, nullptr, 10);
  if (panel < 0 || panel >= NUM_PANELS || x < 0 || x > 15 || y < 0 || y > 15) return;
  uint8_t cmd[] = {0x09, (uint8_t)x, (uint8_t)y, on ? (uint8_t)1 : (uint8_t)0};
  twiWrite(kPanelAddresses[panel], cmd, 4);
}

static void handleWritePattern(char* buf) {
  char* end = nullptr;
  long panel = strtol(buf + 1, &end, 10);
  long slot = strtol(end, nullptr, 10);
  if (panel < 0 || panel >= NUM_PANELS || slot < 0 || slot >= 4) return;
  uint8_t cmd[] = {0x0A, (uint8_t)slot};
  twiWrite(kPanelAddresses[panel], cmd, 2);
  Serial.print(F("Pattern saved to panel "));
  Serial.print(panel);
  Serial.print(F(" slot "));
  Serial.println(slot);
}
