#include "HID-Project.h"
#include <avr/wdt.h>

#define NUM_PANELS      5
#define FSRS_PER_PANEL  4
#define NUM_SENSORS     (NUM_PANELS * FSRS_PER_PANEL)
#define NUM_SLOTS       4
#define BITMAP_BYTES    32
#define UPLOAD_CHUNK    8

static const uint8_t kPanelAddresses[NUM_PANELS] = {0x10, 0x11, 0x12, 0x13, 0x14};
static const int32_t kI2CClock = 400000L;
static const int32_t kSerialBaud = 115200;
static const int32_t kSendIntervalUs = 1000;
// Must exceed the slave's worst-case interrupt blackout: one 256-LED
// FastLED.show() is ~8 ms. With a shorter timeout the master would abort
// mid-stretch, which can wedge the slave's TWI state machine.
static const uint32_t kI2CTimeoutUs = 12000;
static const uint8_t kMaxPanelFails = 5;
static const uint16_t kBusRecoverIntervalMs = 1000;

static bool panelActive[NUM_PANELS];
static uint8_t calValues[NUM_SENSORS];
static uint8_t panelFailCount[NUM_PANELS];
static unsigned long lastBusRecoverMs;
static unsigned long lastSendUs;
static unsigned long lastIterationUs;
static bool calibrating;
static unsigned long calPrintUs;

// Loop-rate statistics for the 'r' command: lets you verify the real poll
// rate (and worst-case loop time) under gameplay conditions.
static unsigned long statLoops;
static uint32_t statMaxLoopUs;
static unsigned long statStartMs;

/*===========================================================================*/
/* Forward declarations                                                      */
/*===========================================================================*/
/* Declared explicitly rather than relying on the Arduino IDE's automatic
 * prototype insertion, so the sketch also builds with a plain avr-gcc or a
 * toolchain that does not run that preprocessing step. */

static void twiInit();
static bool twiExpired(uint32_t deadline);
static bool twiWait(uint32_t* deadline);
static bool twiStart(uint32_t* deadline);
static void twiStop();
static bool twiSdaStuck();
static void twiBusRecover();
static bool twiSendAddr(uint8_t addr, bool read, uint32_t* deadline);
static bool twiWriteByte(uint8_t data, uint32_t* deadline);
static bool twiReadByte(uint8_t* data, bool sendAck, uint32_t* deadline);
static bool twiRead(uint8_t addr, uint8_t* buf, uint8_t len);
static bool twiWrite(uint8_t addr, uint8_t* data, uint8_t len);
static bool twiProbe(uint8_t addr);

static void updateGamepad();
static void pollAllPanels();

static int8_t hexNibble(char c);
static bool parseInt(char** p, long* out);
static char* skipBlanks(char* p);
static void printPrompt();
static void printHelp();
static void processSerial();
static void dispatchCommand(char* buf);
static void reportPanel(long panel, bool ok);
static void handleRateStats();
static void handleZeroOffsets();
static void printValues();
static void printThresholds();
static void printReleaseThresholds();
static void handleSaveCal();
static void handleSetThreshold(char* args);
static void handleSetAllThresholds(char* args);
static void handleSetReleaseThreshold(char* args);
static void handleSetAllReleaseThresholds(char* args);
static void handleLiveMode();
static void handleScan();
static void handleIdentify(char* args);
static void handleUploadPattern(char* args);
static void handleSetPixel(char* args);
static void handleClearPattern(char* args);
static void handleWritePattern(char* args);
static void handleSelectSlot(char* args);
static void handleBrightness(char* args);

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

// Overflow-safe deadline test. A plain `micros() > deadline` fires immediately
// for the ~12 ms window in which the deadline has wrapped past 2^32 but
// micros() has not, which made every transaction fail once every ~71 minutes.
static bool twiExpired(uint32_t deadline) {
  return (int32_t)(micros() - deadline) >= 0;
}

static bool twiWait(uint32_t* deadline) {
  while (!(TWCR & _BV(TWINT))) {
    if (twiExpired(*deadline)) {
      twiStop();          // waits for the STOP to actually be generated
      TWCR = _BV(TWEN);   // safe now: TWSTO has already cleared
      return false;
    }
  }
  return true;
}

static bool twiStart(uint32_t* deadline) {
  TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);
  if (!twiWait(deadline)) return false;
  uint8_t status = TWSR & 0xF8;
  return status == 0x08 || status == 0x10;  // START / repeated START
}

// TWSTO is cleared by hardware once the STOP has actually been generated.
// Waiting for it matters twice over: rewriting TWCR early can truncate the
// STOP and leave the bus unreleased, and twiSdaStuck() would otherwise sample
// SDA while this very STOP is still holding it low — reading a transient low
// as a wedged slave and firing a pointless bus recovery. At 400 kHz the wait
// is ~2.5 us.
static void twiStop() {
  TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN);
  uint32_t deadline = micros() + 100;
  while ((TWCR & _BV(TWSTO)) && !twiExpired(deadline));
}

// True when some device is holding SDA low while the bus should be idle.
static bool twiSdaStuck() {
  return !(PIND & _BV(PD1));
}

// Bit-banged bus recovery. A slave interrupted mid-byte can be left waiting
// for the missing bit-clocks while holding SDA low, wedging the whole bus.
// Clocking SCL up to 9 times lets it finish the byte and release SDA,
// followed by a STOP and a TWI re-init. (32U4: PD0 = SCL, PD1 = SDA.)
static void twiBusRecover() {
  TWCR = 0;  // TWI off, pins become GPIO
  // Release both lines (input + pull-up; the external 1.5k-2.2k bus pull-ups
  // at the master do the actual work)
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

// When no program on the PC holds the gamepad open, the HID interrupt
// endpoint is never drained and Gamepad.write() blocks for the USB stack's
// ~250 ms send timeout (e.g. every press while you only watch the serial
// monitor). Detect a stalled write by timing it, then back reporting off for
// kHidRetryMs — the state keeps accumulating in gamepadDirty and a single
// probe after the backoff re-arms reporting as soon as something opens the
// device. While a game is attached, writes take microseconds and this never
// triggers.
static unsigned long hidRetryAtMs;
static const uint16_t kHidRetryMs = 1000;
static const uint32_t kHidStallUs = 50000;  // 50 ms >> any legit (<1 ms) write

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
      // Status prints are gated on `if (Serial)`: when no host program holds
      // the CDC port open, the TX bank is never drained and a print can
      // block up to the USB stack's ~250 ms send timeout, stalling the loop.
      if (panelFailCount[p] >= kMaxPanelFails) {
        if (Serial) {
          Serial.print(F("Panel "));
          Serial.print(p);
          Serial.println(F(" online"));
        }
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
        // Drop the last known readings so a dead panel does not keep showing
        // phantom pressure in `v` output and the calibration stream.
        int base = p * FSRS_PER_PANEL;
        for (int i = 0; i < FSRS_PER_PANEL; i++) calValues[base + i] = 0;
        panelActive[p] = false;
        if (Serial) {
          Serial.print(F("Panel "));
          Serial.print(p);
          Serial.println(F(" offline"));
        }
      }
    } else if (twiSdaStuck() &&
               (unsigned long)(millis() - lastBusRecoverMs) >= kBusRecoverIntervalMs) {
      // Only recover when SDA is actually being held low. An absent slave
      // NACKs cleanly and needs no recovery — running the bit-bang routine
      // for it would pointlessly toggle a bus the other panels are using.
      twiBusRecover();
      lastBusRecoverMs = millis();
    }
  }
}

void setup() {
  Serial.begin(kSerialBaud);
  twiInit();
  for (int p = 0; p < NUM_PANELS; p++) {
    panelActive[p] = gamepadReported[p] = false;
    panelFailCount[p] = 0;
  }
  Gamepad.begin();
  gamepadDirty = true;  // send the initial all-released state once
  hidRetryAtMs = 0;
  lastSendUs = 0;
  lastIterationUs = 0;
  calibrating = false;
  calPrintUs = 0;
  lastBusRecoverMs = 0;
  statLoops = 0;
  statMaxLoopUs = 0;
  statStartMs = millis();
  Serial.println(F("PIUFSR Master ready."));
  printHelp();
  printPrompt();
  // Armed only after the banner. A host that has the CDC port open but is not
  // draining it makes Serial writes block, and ~700 bytes of banner could
  // otherwise outlast the 2 s watchdog before loop() ever gets to reset it.
  wdt_enable(WDTO_2S);
}

void loop() {
  unsigned long now = micros();
  wdt_reset();

  // Reads only: the slaves toggle their own LEDs from their FSR state, so
  // gameplay needs no writes to the panels at all.
  pollAllPanels();

  if (now - lastSendUs + lastIterationUs >= (unsigned long)kSendIntervalUs) {
    updateGamepad();
    if (gamepadDirty && (long)(millis() - hidRetryAtMs) >= 0) {
      unsigned long wStart = micros();
      Gamepad.write();
      if (micros() - wStart >= kHidStallUs) {
        hidRetryAtMs = millis() + kHidRetryMs;  // endpoint undrained; back off
      } else {
        gamepadDirty = false;
      }
    }
    lastSendUs = now;
  }

  if (calibrating && now - calPrintUs >= 50000) {
    if (Serial) {  // see pollAllPanels: an undrained port costs ~250 ms/print
      Serial.print('c');
      for (int i = 0; i < NUM_SENSORS; i++) {
        Serial.print(' ');
        Serial.print(calValues[i]);
      }
      Serial.println();
    }
    calPrintUs = now;
  }

  processSerial();

  lastIterationUs = micros() - now;
  statLoops++;
  if (lastIterationUs > statMaxLoopUs) statMaxLoopUs = lastIterationUs;
}

/*===========================================================================*/
/* Serial command processing                                                */
/*===========================================================================*/

// Returns -1 for a non-hex character so malformed input is rejected instead
// of silently becoming zero nibbles.
static int8_t hexNibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

static char* skipBlanks(char* p) {
  while (*p == ' ' || *p == '\t') p++;
  return p;
}

// Parse one decimal integer, advancing *p past it. False when no digits are
// present, so a missing or malformed argument is reported instead of silently
// defaulting to 0 (which used to make a bare "x" clear pixel 0,0 of panel 0).
static bool parseInt(char** p, long* out) {
  char* s = skipBlanks(*p);
  char* end = nullptr;
  long v = strtol(s, &end, 10);
  if (end == s) return false;
  *p = end;
  *out = v;
  return true;
}

static void printPrompt() {
  Serial.print(F("> "));
}

static void printHelp() {
  Serial.println(F("  o          Zero offsets (feet off)"));
  Serial.println(F("  v          Print compensated values"));
  Serial.println(F("  t          Print thresholds (20 values, 0 = offline)"));
  Serial.println(F("  s          Save calibration to EEPROM"));
  Serial.println(F("  s <i> <v>  Set threshold sensor i (0-19) to v (0-255)"));
  Serial.println(F("  a <v>      Set every threshold to v (one write per panel)"));
  Serial.println(F("  e <i> <v>  Set release threshold sensor i (0-19) to v (0-255)"));
  Serial.println(F("  y <v>      Set every release threshold to v (one write per panel)"));
  Serial.println(F("  q          Print release thresholds (20 values, 0 = offline)"));
  Serial.println(F("  l          Live LED mode on all panels (FSR-driven, after editing)"));
  Serial.println(F("  c          Toggle streaming 20Hz"));
  Serial.println(F("  u <p> <s> <64hex>  Upload 32B pattern to panel p slot s"));
  Serial.println(F("  x <p> <x> <y> <0/1>  Set pixel on panel p"));
  Serial.println(F("  z <p>      Clear panel p's live pattern"));
  Serial.println(F("  w <p> <s>  Save live pattern to EEPROM slot s"));
  Serial.println(F("  p <p> <s>  Select active slot s on panel p"));
  Serial.println(F("  b <p> <v>  Set panel p brightness to v (0-255)"));
  Serial.println(F("  i <p>      Identify panel p (blink LED)"));
  Serial.println(F("  i          Scan bus for panels"));
  Serial.println(F("  r          Print loop rate stats (resets the window)"));
  Serial.println(F("  h / ?      This help"));
}

// Reads at most one complete line per call. A single command can cost several
// I2C transactions, so draining a whole burst here (the calibration UI sends
// one command per slider preset) would starve pollAllPanels() and could run
// long enough to trip the 2 s watchdog, whose only reset is at the top of
// loop().
static void processSerial() {
  static char line[128];
  static size_t pos = 0;
  static bool overflow = false;

  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (overflow) {
        pos = 0;
        overflow = false;
        Serial.println(F("Line too long, ignored."));
        printPrompt();
        return;
      }
      if (pos == 0) continue;
      line[pos] = '\0';
      pos = 0;
      dispatchCommand(line);
      printPrompt();
      return;
    }
    if (pos < sizeof(line) - 1) {
      line[pos++] = c;
    } else {
      overflow = true;  // never execute a truncated command
    }
  }
}

static void dispatchCommand(char* buf) {
  char* args = skipBlanks(buf + 1);
  switch (buf[0]) {
    case 'o': case 'O': handleZeroOffsets(); break;
    case 'v': case 'V': printValues(); break;
    case 't': case 'T': printThresholds(); break;
    case 's': case 'S':
      // Bare "s" (or "s" plus blanks) saves; "s <i> <v>" sets a threshold.
      if (*args == '\0') {
        handleSaveCal();
      } else {
        handleSetThreshold(args);
      }
      break;
    case 'a': case 'A': handleSetAllThresholds(args); break;
    case 'e': case 'E': handleSetReleaseThreshold(args); break;
    case 'y': case 'Y': handleSetAllReleaseThresholds(args); break;
    case 'q': case 'Q': printReleaseThresholds(); break;
    case 'l': case 'L': handleLiveMode(); break;
    case 'c': case 'C':
      calibrating = !calibrating;
      calPrintUs = micros();
      Serial.print(F("Streaming "));
      Serial.println(calibrating ? F("ON") : F("OFF"));
      break;
    case 'u': case 'U': handleUploadPattern(args); break;
    case 'x': case 'X': handleSetPixel(args); break;
    case 'z': case 'Z': handleClearPattern(args); break;
    case 'w': case 'W': handleWritePattern(args); break;
    case 'p': case 'P': handleSelectSlot(args); break;
    case 'b': case 'B': handleBrightness(args); break;
    case 'i': case 'I':
      if (*args == '\0') {
        handleScan();
      } else {
        handleIdentify(args);
      }
      break;
    case 'r': case 'R': handleRateStats(); break;
    case 'h': case 'H': case '?': printHelp(); break;
    default:
      Serial.println(F("Unknown. Type h for help."));
      break;
  }
}

// Every pattern/pixel command answers in this exact shape; ledmaker.py matches
// replies by the trailing OK/FAIL.
static void reportPanel(long panel, bool ok) {
  Serial.print(F("Panel "));
  Serial.print(panel);
  Serial.println(ok ? F(" OK") : F(" FAIL"));
}

// Prints the loop rate over the window since the last 'r' (or boot), then
// resets the counters. Use it to sanity-check the real sensor poll rate:
// ~1000 Hz is the target; the worst-loop figure shows whether anything
// (I2C timeouts, serial bursts) is stalling the loop.
static void handleRateStats() {
  unsigned long elapsedMs = millis() - statStartMs;
  unsigned long loops = statLoops;
  uint32_t maxUs = statMaxLoopUs;
  Serial.print(loops);
  Serial.print(F(" loops in "));
  Serial.print(elapsedMs / 1000.0, 1);
  Serial.print(F(" s = "));
  if (elapsedMs > 0) {
    Serial.print(loops / (elapsedMs / 1000.0), 1);
  } else {
    Serial.print('?');
  }
  Serial.print(F(" Hz, worst loop "));
  Serial.print(maxUs);
  Serial.println(F(" us"));
  statLoops = 0;
  statMaxLoopUs = 0;
  statStartMs = millis();
}

static void handleZeroOffsets() {
  uint8_t cmd = 0x05;
  for (int p = 0; p < NUM_PANELS; p++) {
    reportPanel(p, twiWrite(kPanelAddresses[p], &cmd, 1));
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

// Always emits exactly NUM_SENSORS values so clients can index into the reply
// positionally; an unreachable panel contributes four zeros. Reads 13 bytes
// per slave (status + compensated + press + release thresholds) and only
// prints the press thresholds; `q` prints the release half.
static void printThresholds() {
  Serial.print('t');
  for (int p = 0; p < NUM_PANELS; p++) {
    uint8_t buf[13];
    bool ok = twiRead(kPanelAddresses[p], buf, 13);
    for (int i = 0; i < FSRS_PER_PANEL; i++) {
      Serial.print(' ');
      Serial.print(ok ? buf[i + 5] : 0);
    }
  }
  Serial.println();
}

static void printReleaseThresholds() {
  Serial.print('q');
  for (int p = 0; p < NUM_PANELS; p++) {
    uint8_t buf[13];
    bool ok = twiRead(kPanelAddresses[p], buf, 13);
    for (int i = 0; i < FSRS_PER_PANEL; i++) {
      Serial.print(' ');
      Serial.print(ok ? buf[i + 9] : 0);
    }
  }
  Serial.println();
}

static void handleSaveCal() {
  uint8_t cmd = 0x07;
  for (int p = 0; p < NUM_PANELS; p++) {
    reportPanel(p, twiWrite(kPanelAddresses[p], &cmd, 1));
  }
}

static void handleSetThreshold(char* args) {
  long idx, val;
  if (!parseInt(&args, &idx) || !parseInt(&args, &val)) {
    Serial.println(F("Usage: s <sensor 0-19> <value 0-255>"));
    return;
  }
  if (idx < 0 || idx >= NUM_SENSORS || val < 0 || val > 255) {
    Serial.println(F("Range: sensor 0-19, value 0-255"));
    return;
  }
  int panel = idx / FSRS_PER_PANEL;
  int fsr = idx % FSRS_PER_PANEL;
  uint8_t cmd[] = {0x06, (uint8_t)fsr, (uint8_t)val};
  // Deliberately no threshold read-back here: it would add five I2C
  // transactions to every slider move. Send `t` when you want the values.
  reportPanel(panel, twiWrite(kPanelAddresses[panel], cmd, 3));
}

// One write per panel instead of twenty single-sensor writes, so a "set
// everything to N" UI action costs 5 transactions rather than 20 commands.
static void handleSetAllThresholds(char* args) {
  long val;
  if (!parseInt(&args, &val)) {
    Serial.println(F("Usage: a <value 0-255>"));
    return;
  }
  if (val < 0 || val > 255) {
    Serial.println(F("Range: value 0-255"));
    return;
  }
  uint8_t cmd[] = {0x0C, (uint8_t)val};
  for (int p = 0; p < NUM_PANELS; p++) {
    reportPanel(p, twiWrite(kPanelAddresses[p], cmd, 2));
  }
}

static void handleSetReleaseThreshold(char* args) {
  long idx, val;
  if (!parseInt(&args, &idx) || !parseInt(&args, &val)) {
    Serial.println(F("Usage: e <sensor 0-19> <value 0-255>"));
    return;
  }
  if (idx < 0 || idx >= NUM_SENSORS || val < 0 || val > 255) {
    Serial.println(F("Range: sensor 0-19, value 0-255"));
    return;
  }
  int panel = idx / FSRS_PER_PANEL;
  int fsr = idx % FSRS_PER_PANEL;
  uint8_t cmd[] = {0x0F, (uint8_t)fsr, (uint8_t)val};
  reportPanel(panel, twiWrite(kPanelAddresses[panel], cmd, 3));
}

static void handleSetAllReleaseThresholds(char* args) {
  long val;
  if (!parseInt(&args, &val)) {
    Serial.println(F("Usage: y <value 0-255>"));
    return;
  }
  if (val < 0 || val > 255) {
    Serial.println(F("Range: value 0-255"));
    return;
  }
  uint8_t cmd[] = {0x10, (uint8_t)val};
  for (int p = 0; p < NUM_PANELS; p++) {
    reportPanel(p, twiWrite(kPanelAddresses[p], cmd, 2));
  }
}

// Return every panel to FSR-driven LED mode after a manual/editing preview.
// Normally unnecessary: the first foot press already hands the LED back.
static void handleLiveMode() {
  uint8_t cmd = 0x0E;
  for (int p = 0; p < NUM_PANELS; p++) {
    reportPanel(p, twiWrite(kPanelAddresses[p], &cmd, 1));
  }
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

static void handleIdentify(char* args) {
  long panel;
  if (!parseInt(&args, &panel) || panel < 0 || panel >= NUM_PANELS) {
    Serial.println(F("Usage: i <panel 0-4>   (bare 'i' scans the bus)"));
    return;
  }
  uint8_t cmd = 0x0B;
  reportPanel(panel, twiWrite(kPanelAddresses[panel], &cmd, 1));
}

static void handleUploadPattern(char* args) {
  long panel, slot;
  if (!parseInt(&args, &panel) || !parseInt(&args, &slot)) {
    Serial.println(F("Usage: u <panel 0-4> <slot 0-3> <64 hex chars>"));
    return;
  }
  if (panel < 0 || panel >= NUM_PANELS || slot < 0 || slot >= NUM_SLOTS) {
    Serial.println(F("Range: panel 0-4, slot 0-3"));
    return;
  }
  char* hex = skipBlanks(args);
  uint8_t bytes[BITMAP_BYTES];
  for (int i = 0; i < BITMAP_BYTES; i++) {
    int8_t hi = hexNibble(hex[i * 2]);
    int8_t lo = (hi < 0) ? -1 : hexNibble(hex[i * 2 + 1]);
    if (hi < 0 || lo < 0) {
      Serial.println(F("Need exactly 64 hex chars."));
      return;
    }
    bytes[i] = (uint8_t)((hi << 4) | lo);
  }
  if (*skipBlanks(hex + BITMAP_BYTES * 2) != '\0') {
    Serial.println(F("Need exactly 64 hex chars."));
    return;
  }
  // Wire's slave receive buffer is 32 bytes and cannot be enlarged from the
  // sketch, so the bitmap goes out as four frames of
  // [0x03][slot][offset][8 data bytes] = 11 bytes each.
  bool ok = true;
  for (uint8_t off = 0; off < BITMAP_BYTES && ok; off += UPLOAD_CHUNK) {
    uint8_t frame[3 + UPLOAD_CHUNK];
    frame[0] = 0x03;
    frame[1] = (uint8_t)slot;
    frame[2] = off;
    for (uint8_t i = 0; i < UPLOAD_CHUNK; i++) frame[3 + i] = bytes[off + i];
    ok = twiWrite(kPanelAddresses[panel], frame, sizeof(frame));
  }
  reportPanel(panel, ok);
}

static void handleSetPixel(char* args) {
  long panel, x, y, on;
  if (!parseInt(&args, &panel) || !parseInt(&args, &x) ||
      !parseInt(&args, &y) || !parseInt(&args, &on)) {
    Serial.println(F("Usage: x <panel 0-4> <x 0-15> <y 0-15> <0|1>"));
    return;
  }
  if (panel < 0 || panel >= NUM_PANELS || x < 0 || x > 15 || y < 0 || y > 15) {
    Serial.println(F("Range: panel 0-4, x 0-15, y 0-15"));
    return;
  }
  uint8_t cmd[] = {0x09, (uint8_t)x, (uint8_t)y, on ? (uint8_t)1 : (uint8_t)0};
  reportPanel(panel, twiWrite(kPanelAddresses[panel], cmd, 4));
}

// One command instead of 256 set-pixel calls, which is what clearing a panel
// used to cost.
static void handleClearPattern(char* args) {
  long panel;
  if (!parseInt(&args, &panel) || panel < 0 || panel >= NUM_PANELS) {
    Serial.println(F("Usage: z <panel 0-4>"));
    return;
  }
  uint8_t cmd = 0x0D;
  reportPanel(panel, twiWrite(kPanelAddresses[panel], &cmd, 1));
}

static void handleWritePattern(char* args) {
  long panel, slot;
  if (!parseInt(&args, &panel) || !parseInt(&args, &slot)) {
    Serial.println(F("Usage: w <panel 0-4> <slot 0-3>"));
    return;
  }
  if (panel < 0 || panel >= NUM_PANELS || slot < 0 || slot >= NUM_SLOTS) {
    Serial.println(F("Range: panel 0-4, slot 0-3"));
    return;
  }
  uint8_t cmd[] = {0x0A, (uint8_t)slot};
  reportPanel(panel, twiWrite(kPanelAddresses[panel], cmd, 2));
}

// Without this, slots 1-3 were write-only: `w` could store a pattern there but
// nothing could make it the slot loaded at power-up.
static void handleSelectSlot(char* args) {
  long panel, slot;
  if (!parseInt(&args, &panel) || !parseInt(&args, &slot)) {
    Serial.println(F("Usage: p <panel 0-4> <slot 0-3>"));
    return;
  }
  if (panel < 0 || panel >= NUM_PANELS || slot < 0 || slot >= NUM_SLOTS) {
    Serial.println(F("Range: panel 0-4, slot 0-3"));
    return;
  }
  uint8_t cmd[] = {0x04, (uint8_t)slot};
  reportPanel(panel, twiWrite(kPanelAddresses[panel], cmd, 2));
}

static void handleBrightness(char* args) {
  long panel, val;
  if (!parseInt(&args, &panel) || !parseInt(&args, &val)) {
    Serial.println(F("Usage: b <panel 0-4> <value 0-255>"));
    return;
  }
  if (panel < 0 || panel >= NUM_PANELS || val < 0 || val > 255) {
    Serial.println(F("Range: panel 0-4, value 0-255"));
    return;
  }
  uint8_t cmd[] = {0x02, (uint8_t)val};
  reportPanel(panel, twiWrite(kPanelAddresses[panel], cmd, 2));
}
