/*
  ESP32 + 4" TFT — UltraGauge-style OBD-II Display
  Vehicle: 2007 Lexus GX470 via ELM327 Bluetooth Classic adapter
  Display: Hosyond 4.0" 320x480, ST7796S, resistive touch (XPT2046), TFT_eSPI

  ============================ WHAT THIS IS ============================
  Same OBD engine, PIN, touch calibration and swipe navigation as the
  multi-screen sketch, but the gauge pages are laid out like an UltraGauge:
    - Pages with 4, 6 or 8 gauges (plus a single-gauge full-screen page)
    - Dark background, only the text is lit
    - Small label on top of each zone, big value, unit underneath
    - Page indicator dots along the bottom, with a connection dot and a
      "heartbeat" dot that blinks each time a full polling loop completes
    - Per-gauge alarms: the value flashes red past its limit
    - Tap anywhere on a gauge page = next page (like UltraGauge's single
      key press). Swipe left/right also works.
    - Only the PIDs shown on the current page are polled, so fewer gauges
      on screen = faster updates

  Pages: ENGINE (6) / MILEAGE (4) / VITALS (8) / TRANS (1 big + bar) /
         TRIMS (6: fuel trims + front/upstream O2 on both banks) /
         EXTRA (5: timing advance, module voltage, baro pressure,
         runtime, distance since codes cleared) / DIAG

  Removed gauges: Ambient Temp, Fuel Level and Fuel Rate were removed after
  a capability diagnostic (see Serial output history) proved this vehicle's
  ECU doesn't expose PIDs 0x46, 0x2F or 0x5E over standard OBD-II at all --
  not a code bug, the data just isn't offered. O2 B2S1 was initially
  removed too (wrong PID guessed, 0x26) but added back correctly as PID
  0x28 once the capability data revealed the real bit-to-PID mapping (see
  the comment above G_O2_B2S1 in pollGauge()).

  ============================ SETUP ============================
  - Save inside a folder with the same name as this file:
      esp32_gx470_ultragauge/esp32_gx470_ultragauge.ino
  - Libraries: TFT_eSPI (Library Manager). BluetoothSerial and Preferences
    come with the ESP32 board package.
  - TFT_eSPI User_Setup.h: ST7796_DRIVER, your display pins, and TOUCH_CS.
  - Fonts: uses TFT_eSPI's built-in fonts only (no extra font library
    needed) — blockier than smooth fonts but compiles with zero extra
    dependencies.
  - Touch calibration runs automatically on the first boot and is saved.
    Hold the top-left corner for ~2 s to recalibrate later.
  - Screen orientation: set SCREEN_ROTATION below (0/2 = portrait, 1/3 =
    the two opposite landscape orientations) to match how you're mounting
    the display/USB-C port. Changing it forces one automatic touch
    recalibration on the next boot.
  - Units: set USE_IMPERIAL below (true = mph / F / MPG, false = km/h / C / km/L).

  ---------------- Find YOUR ELM327 adapter's MAC address and PIN ----------------
  elmAddress and elmPin below are placeholders -- every adapter has its own
  unique MAC, and this sketch connects by MAC (not by name), so it will not
  find your adapter until you fill these in:
    1. Flash bt_scanner.ino (a separate, small diagnostic sketch included
       alongside this one) to your ESP32 and open its Serial Monitor. It
       scans for nearby Bluetooth Classic devices and prints each one's
       name and MAC address.
    2. Find your ELM327/OBD adapter in that list (commonly named "OBDII",
       "OBDLink", "Vgate", "VLinker", etc.) and copy its MAC address.
    3. Paste that MAC into elmAddress below, in the same
       { 0x.., 0x.., 0x.., 0x.., 0x.., 0x.. } format.
    4. Set elmPin to whatever PIN your adapter's manual/listing specifies.
       "1234" and "0000" are the two most common defaults; some (like
       OBDLink) use "6789". If the wrong PIN is set, pairing will fail.

  ============================ NOTES ============================
  - Transmission temps use Toyota's enhanced Mode 21 PID D9 (header 7E0).
    Not verified on a GX470: sanity-check against coolant temp.
  - Instant MPG is computed from MAF and speed (gasoline, 14.7:1).
  - Alarm limits live in the gaugeDefs table below.
*/

#include <SPI.h>
#include <TFT_eSPI.h>
#include "BluetoothSerial.h"
#include <Preferences.h>
#include <vector>
#include <math.h>
#include <ctype.h>

// Low-level ESP-IDF Bluetooth GAP API — used below for detailed connection
// logging (and to clear any stale bond at boot). Not strictly required for
// the current Vgate adapter, which connects fine via the standard
// SerialBT.connect(), but kept in place since it's useful diagnostic info
// if a future adapter needs it again.
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_spp_api.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Pick a Partition Scheme that includes Bluetooth.
#endif

// NOTE: this sketch intentionally uses only TFT_eSPI's built-in fonts
// (via setTextSize), not the Adafruit-GFX "free fonts" (setFreeFont).
// The free fonts require Adafruit-GFX-Library's Fonts/ headers to be on
// your include path, which varies by install and isn't guaranteed —
// built-in fonts have zero extra dependency and always compile.

TFT_eSPI tft = TFT_eSPI();
BluetoothSerial SerialBT;
Preferences prefs;

// ---------------- User settings ----------------
// elmDeviceName is unused by this sketch (it connects by MAC, not by name)
// -- kept only as a label for whatever you happen to see in bt_scanner.ino.
const char* elmDeviceName = "OBDII"; // cosmetic only; see elmAddress below
const char* elmPin        = "1234";  // CHANGE ME if your adapter's PIN differs
                                      // (see "Find YOUR ELM327..." in the
                                      // header comment above)
const bool  USE_IMPERIAL  = true;

// Landscape rotation. TFT_eSPI's 0/2 are portrait, 1/3 are the two opposite
// landscape orientations (180 degrees apart). Was 1 (USB-C on the right as
// viewed); 3 flips it 180 degrees so USB-C ends up on the left, to match how
// this is being mounted. Touch calibration is tied to rotation, so changing
// this forces a one-time recalibration on the next boot (see loadCalData()).
const int SCREEN_ROTATION = 3;
const unsigned long AUTO_CYCLE_MS = 0;   // e.g. 8000 = auto-rotate gauge pages; 0 = off

// Connecting by MAC address skips the Bluetooth discovery/inquiry scan that
// name-based connect() does. CHANGE ME: this is a placeholder -- find your
// own adapter's MAC with bt_scanner.ino (see "Find YOUR ELM327..." in the
// header comment above). The sketch will not find your adapter until you do.
uint8_t elmAddress[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// ---------------- Types (must come before the first function) ----------------
enum GaugeId {
  G_RPM = 0, G_SPEED, G_COOLANT, G_THROTTLE, G_TRANS_PAN, G_TRANS_TC,
  G_BATT, G_IAT, G_LOAD, G_MAF, G_MPG,
  // Fuel trims (bank 1 / bank 2) and front (upstream) wideband O2 sensors
  // on both banks.
  G_STFT1, G_LTFT1, G_STFT2, G_LTFT2, G_O2_B1S1, G_O2_B2S1,
  // Misc standard PIDs. Ambient temp (0x46), fuel level (0x2F) and fuel
  // rate (0x5E) were removed for the same reason -- confirmed unsupported
  // by this vehicle's ECU, not a code bug.
  G_TIMING, G_MODULE_V, G_BARO, G_RUNTIME,
  G_DIST_CLR,
  G_COUNT
};

struct GaugeDef {
  const char* label;
  const char* unitMetric;
  const char* unitImperial;
  float alarmHigh;
  float alarmLow;
  float barMin;
  float barMax;
};

enum FontRole { F_LABEL, F_TITLE, F_VAL_8, F_VAL_6, F_VAL_4, F_VAL_1 };

struct PageDef {
  const char* name;
  uint8_t count;
  GaugeId gauges[8];
  bool isDiag;
};

struct BtnRect { int x, y, w, h; };

// ---------------- Gauge and page tables ----------------
#define NO_HI  9999.0f
#define NO_LO -9999.0f

// Values are stored internally in metric (C, km/h, g/s, MPG). Alarm limits
// and bar ranges below are in those internal units.
const GaugeDef gaugeDefs[G_COUNT] = {
  { "RPM",      "rpm",  "rpm", 5500.0f, NO_LO,  0.0f, 6000.0f },
  { "SPEED",    "km/h", "mph", NO_HI,   NO_LO,  0.0f,  200.0f },
  { "COOLANT",  "C",    "F",   110.0f,  NO_LO, 40.0f,  130.0f },
  { "THROTTLE", "%",    "%",   NO_HI,   NO_LO,  0.0f,  100.0f },
  { "TRANS",    "C",    "F",   110.0f,  NO_LO, 20.0f,  130.0f },
  { "TRANS TC", "C",    "F",   130.0f,  NO_LO, 20.0f,  150.0f },
  { "BATTERY",  "V",    "V",   15.5f,   11.5f, 10.0f,   16.0f },
  { "INTAKE",   "C",    "F",   NO_HI,   NO_LO,  0.0f,   80.0f },
  { "LOAD",     "%",    "%",   NO_HI,   NO_LO,  0.0f,  100.0f },
  { "MAF",      "g/s",  "g/s", NO_HI,   NO_LO,  0.0f,  100.0f },
  { "MPG",      "km/L", "MPG", NO_HI,   NO_LO,  0.0f,   40.0f },
  // Fuel trims: +/-25% is a commonly used "something's wrong" threshold
  { "STFT1",    "%",    "%",    25.0f,  -25.0f, -30.0f,  30.0f },
  { "LTFT1",    "%",    "%",    25.0f,  -25.0f, -30.0f,  30.0f },
  { "STFT2",    "%",    "%",    25.0f,  -25.0f, -30.0f,  30.0f },
  { "LTFT2",    "%",    "%",    25.0f,  -25.0f, -30.0f,  30.0f },
  // Wideband A/F equivalence ratio, not voltage (see pollGauge()): 1.00 is
  // stoichiometric, so flag anything drifting too far from it either way.
  { "O2 B1S1",  "eqr",  "eqr",  1.20f,  0.80f,  0.7f,   1.3f },
  { "O2 B2S1",  "eqr",  "eqr",  1.20f,  0.80f,  0.7f,   1.3f },
  { "TIMING",   "deg",  "deg",  NO_HI,  NO_LO, -10.0f,  40.0f },
  { "MODULE V", "V",    "V",    15.5f,  11.5f,  10.0f,  16.0f },
  { "BARO",     "kPa",  "kPa",  NO_HI,  NO_LO,  80.0f, 105.0f },
  { "RUNTIME",  "min",  "min",  NO_HI,  NO_LO,   0.0f,  60.0f },
  { "DIST CLR", "km",   "mi",   NO_HI,  NO_LO,   0.0f, 500.0f },
};

#define PAGE_COUNT 7
const PageDef pages[PAGE_COUNT] = {
  { "ENGINE",  6, { G_RPM, G_SPEED, G_COOLANT, G_THROTTLE, G_TRANS_PAN, G_TRANS_TC }, false },
  { "MILEAGE", 4, { G_MPG, G_SPEED, G_MAF, G_LOAD }, false },
  { "VITALS",  8, { G_RPM, G_SPEED, G_COOLANT, G_TRANS_PAN, G_BATT, G_IAT, G_LOAD, G_THROTTLE }, false },
  { "TRANS",   1, { G_TRANS_PAN }, false },
  { "TRIMS",   6, { G_STFT1, G_LTFT1, G_STFT2, G_LTFT2, G_O2_B1S1, G_O2_B2S1 }, false },
  { "EXTRA",   5, { G_TIMING, G_MODULE_V, G_BARO, G_RUNTIME, G_DIST_CLR }, false },
  { "DIAG",    0, { }, true },
};

// ---------------- Look (RGB565 colors) ----------------
#define COL_VALUE      TFT_WHITE
#define COL_LABEL      0x65BF     // soft light blue
#define COL_UNIT       0x8410     // mid gray
#define COL_DIM        0x4208     // dark gray, used for "---"
#define COL_DIV        0x2104     // very dark gray divider lines
#define COL_ALARM      TFT_RED
#define COL_ALARM_DIM  0x7800

#define IND_H     24   // bottom indicator strip height
#define TOP_PAD   30   // room for the zone label
#define BOT_PAD   26   // room for the unit

// ---------------- Live state ----------------
float   gaugeVal[G_COUNT];
bool    gaugeOk[G_COUNT];
uint8_t failCount[G_COUNT];

bool connected = false;
bool isCan = true;
int  commFail = 0;
unsigned long lastConnectTry = 0;

int currentPage = 0;
unsigned long lastPageChange = 0;

String   zoneStr[8];
uint16_t zoneColor[8];

bool needed[G_COUNT];
int  pollList[G_COUNT];
int  pollCount = 0;
int  pollIdx = 0;

bool heartbeat = false;
String   statusMsg = "";
uint16_t statusColor = TFT_YELLOW;

// Diagnostics state
BtnRect readBtn  = { 20, 14, 200, 52 };
BtnRect clearBtn = { 260, 14, 200, 52 };
std::vector<String> dtcCodes;
bool   dtcRead = false;
String diagMsg = "";
unsigned long clearArmedUntil = 0;

// Touch state
uint16_t calData[5];
bool wasTouched = false;
int touchStartX = -1, touchStartY = -1, lastTouchX = -1, lastTouchY = -1;
unsigned long touchStartTime = 0;
bool recalTriggered = false;
const int SWIPE_MIN_PX = 60;
const unsigned long SWIPE_MAX_MS = 600;
const unsigned long RECAL_HOLD_MS = 2000;

// =========================================================================
//                              FONT HELPERS
// =========================================================================
// Font 1 (the old GLCD 5x7 font) blown up with setTextSize() is what made
// everything look chunky/pixelated — each tiny pixel just gets stretched
// into a big square. Font 2 (16px) and Font 4 (26px) are TFT_eSPI's own
// built-in larger fonts, drawn at their real design size instead of being
// stretched, so edges stay clean. These are compiled into TFT_eSPI itself
// (not the Adafruit-GFX "free fonts" that caused the earlier missing-header
// error) and are enabled by default in virtually every TFT_eSPI setup.
void useFont(FontRole r) {
  switch (r) {
    case F_LABEL: tft.setTextFont(2); tft.setTextSize(1); break;  // ~16px
    case F_TITLE: tft.setTextFont(2); tft.setTextSize(1); break;  // ~16px
    case F_VAL_8: tft.setTextFont(4); tft.setTextSize(1); break;  // ~26px
    case F_VAL_6: tft.setTextFont(4); tft.setTextSize(1); break;  // ~26px
    case F_VAL_4: tft.setTextFont(4); tft.setTextSize(2); break;  // ~52px
    case F_VAL_1: tft.setTextFont(4); tft.setTextSize(3); break;  // ~78px
  }
}

FontRole valueFontFor(int count) {
  switch (count) {
    case 1:  return F_VAL_1;
    case 4:  return F_VAL_4;
    case 8:  return F_VAL_8;
    default: return F_VAL_6;
  }
}

// =========================================================================
//                          TOUCH CALIBRATION
// =========================================================================
void saveCalData() {
  prefs.begin("touchCal", false);
  prefs.putBytes("calData", calData, sizeof(calData));
  prefs.putInt("rot", SCREEN_ROTATION);   // remember which rotation this cal is for
  prefs.end();
}

bool loadCalData() {
  prefs.begin("touchCal", true);
  bool haveIt = prefs.isKey("calData") &&
                prefs.getBytesLength("calData") == sizeof(calData) &&
                prefs.getInt("rot", -1) == SCREEN_ROTATION;   // stale cal from a
                                                               // different rotation
                                                               // forces a redo
  if (haveIt) prefs.getBytes("calData", calData, sizeof(calData));
  prefs.end();
  return haveIt;
}

void runTouchCalibration() {
  tft.fillScreen(TFT_BLACK);
  useFont(F_TITLE);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("Touch each corner", tft.width() / 2, tft.height() / 2);
  delay(1200);

  tft.calibrateTouch(calData, TFT_WHITE, TFT_RED, 15);
  saveCalData();
  tft.setTouch(calData);

  // Wait for the finger to lift so it doesn't retrigger anything
  uint16_t tx, ty;
  unsigned long t0 = millis();
  while (tft.getTouch(&tx, &ty) && millis() - t0 < 3000) delay(20);
}

void loadOrCalibrateTouch() {
  if (loadCalData()) tft.setTouch(calData);
  else runTouchCalibration();
}

// =========================================================================
//                       ELM327 COMMUNICATION
// =========================================================================
String sendCommandT(const String &cmd, unsigned long timeoutMs) {
  while (SerialBT.available()) SerialBT.read();

  SerialBT.print(cmd);
  SerialBT.print("\r");

  String response = "";
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    if (SerialBT.available()) {
      char c = SerialBT.read();
      if (c == '>') {            // ELM327 prompt = response finished
        commFail = 0;
        return response;
      }
      response += c;
    } else {
      delay(1);
    }
  }
  commFail++;                    // never saw the prompt: count as a failure
  return response;
}

String sendCommand(const String &cmd) {
  return sendCommandT(cmd, 1000);
}

int hexByte(const String &hex, int pos) {
  return (int)strtol(hex.substring(pos, pos + 2).c_str(), NULL, 16);
}

// Turns a raw ELM327 reply into one string of hex digits. Drops status
// lines (SEARCHING..., NO DATA, etc.), multi-frame "0:"/"1:" prefixes and
// the multi-frame length header line.
String cleanHex(const String &raw) {
  String out = "";
  int start = 0;
  int len = raw.length();
  while (start <= len) {
    int end = raw.indexOf('\r', start);
    if (end < 0) end = len;
    String line = raw.substring(start, end);
    start = end + 1;

    line.replace("\n", "");
    line.replace(" ", "");
    line.trim();
    line.toUpperCase();
    if (line.length() == 0) continue;

    bool hadIndex = false;
    int colon = line.indexOf(':');
    if (colon >= 0) { line = line.substring(colon + 1); hadIndex = true; }
    if (!hadIndex && line.length() == 3) continue;   // multi-frame length header

    bool allHex = true;
    for (unsigned int i = 0; i < line.length(); i++) {
      if (!isxdigit((unsigned char)line[i])) { allHex = false; break; }
    }
    if (!allHex) continue;
    out += line;
  }
  return out;
}

void initELM327() {
  sendCommandT("ATZ", 2000);
  sendCommand("ATE0");
  sendCommand("ATL0");
  sendCommand("ATS0");
  sendCommand("ATH0");
  sendCommand("ATSP0");
  sendCommandT("0100", 8000);          // forces protocol auto-detection now

  // Protocol number: 6-9, A-C are CAN. Unknown (0) is treated as CAN (GX470 is CAN).
  String p = sendCommand("ATDPN");
  p.replace("\r", ""); p.replace("\n", ""); p.replace(" ", ""); p.trim(); p.toUpperCase();
  char c = p.length() ? p.charAt(p.length() - 1) : '6';
  isCan = !(c >= '1' && c <= '5');

  // NOTE: the engine-ECU-only header (ATSH7E0) used to be set here globally,
  // but that locks EVERY query for the rest of the session to one module.
  // Some standard PIDs (fuel level, ambient temp) are owned by a different
  // module on this vehicle (body ECU / combination meter, not the engine
  // ECU), so they went silent once this became permanent. The header is now
  // only switched to 7E0 right before the Mode 21 transmission query, in
  // pollTrans() below, and switched back afterward so everything else uses
  // the normal default broadcast addressing (whichever module owns a PID
  // answers, same as any generic OBD scan tool).
  commFail = 0;

  printCapabilityDiagnostics();
}

// =========================================================================
//              ONE-TIME CAPABILITY DIAGNOSTIC (Serial Monitor only)
// =========================================================================
// Rather than keep guessing PID numbers for gauges that come back blank,
// this asks the ELM327/ECU directly: (1) which standard PIDs it actually
// supports, so we can tell "genuinely unsupported" apart from "wrong PID
// number", and (2) which O2 sensor positions are physically equipped and
// in what slot order, so the wideband Sensor1-8 numbering (PIDs 0x24-0x2B)
// can be mapped to actual Bank/Sensor positions correctly instead of
// guessed. Runs once, prints to Serial only, no effect on the gauges.
bool capabilityDiagPrinted = false;

void printPidSupportBlock(int basePid) {
  char cmd[8];
  snprintf(cmd, sizeof(cmd), "01%02X", basePid);
  String hex = cleanHex(sendCommandT(String(cmd), 3000));
  char key[8];
  snprintf(key, sizeof(key), "41%02X", basePid);
  int idx = hex.indexOf(key);
  if (idx < 0) {
    Serial.printf("  PIDs 0x%02X-0x%02X: no response (block not supported)\n",
                  basePid + 1, basePid + 0x20);
    return;
  }
  int p = idx + 4;
  if ((int)hex.length() < p + 8) {
    Serial.printf("  PIDs 0x%02X-0x%02X: short/garbled response\n", basePid + 1, basePid + 0x20);
    return;
  }
  Serial.printf("  Supported in 0x%02X-0x%02X: ", basePid + 1, basePid + 0x20);
  for (int byteIdx = 0; byteIdx < 4; byteIdx++) {
    int b = hexByte(hex, p + byteIdx * 2);
    for (int bit = 7; bit >= 0; bit--) {
      if (b & (1 << bit)) {
        int pidNum = basePid + (byteIdx * 8) + (7 - bit) + 1;
        Serial.printf("0x%02X ", pidNum);
      }
    }
  }
  Serial.println();
}

void printO2LocationBlock(int pid, const char* label) {
  int A = 0, B = 0;
  if (!queryStd(pid, A, B)) {
    Serial.printf("  PID 0x%02X (%s): no response\n", pid, label);
    return;
  }
  Serial.printf("  PID 0x%02X (%s): raw byte 0x%02X, bits 7->0: ", pid, label, A);
  for (int bit = 7; bit >= 0; bit--) Serial.printf("%d", (A >> bit) & 1);
  Serial.println();
}

void printCapabilityDiagnostics() {
  if (capabilityDiagPrinted) return;
  capabilityDiagPrinted = true;

  Serial.println("=================================");
  Serial.println("CAPABILITY DIAGNOSTIC (one-time, this ECU/vehicle only)");
  Serial.println("Which standard PIDs this vehicle actually supports:");
  printPidSupportBlock(0x00);   // PIDs 0x01-0x20
  printPidSupportBlock(0x20);   // PIDs 0x21-0x40 (covers 0x2F fuel level, 0x33 baro)
  printPidSupportBlock(0x40);   // PIDs 0x41-0x60 (covers 0x42 module V, 0x46 ambient, 0x5E fuel rate)

  Serial.println("O2 sensor positions actually equipped (for mapping wideband PIDs 0x24-0x2B):");
  printO2LocationBlock(0x13, "O2 sensors present, 2-bank/legacy layout");
  printO2LocationBlock(0x1D, "O2 sensors present, 4-bank/wideband layout");
  Serial.println("=================================");
}

bool queryStd(int pid, int &A, int &B) {
  char cmd[8];
  snprintf(cmd, sizeof(cmd), "01%02X", pid);
  String hex = cleanHex(sendCommand(String(cmd)));

  char key[8];
  snprintf(key, sizeof(key), "41%02X", pid);
  int idx = hex.indexOf(key);
  if (idx < 0) return false;

  int p = idx + 4;
  if ((int)hex.length() < p + 2) return false;
  A = hexByte(hex, p);
  B = ((int)hex.length() >= p + 4) ? hexByte(hex, p + 2) : 0;
  return true;
}

bool queryBattery(float &v) {
  String r = sendCommand("ATRV");
  r.replace("\r", ""); r.replace("\n", ""); r.replace("V", ""); r.replace("v", ""); r.trim();
  if (r.length() == 0) return false;
  float f = r.toFloat();
  if (f < 5.0f || f > 20.0f) return false;
  v = f;
  return true;
}

void noteResult(GaugeId g, bool ok) {
  if (ok) {
    failCount[g] = 0;
    gaugeOk[g] = true;
  } else {
    if (failCount[g] < 100) failCount[g]++;
    if (failCount[g] >= 3) gaugeOk[g] = false;   // tolerate a couple of dropped replies
  }
}

void pollTrans() {
  // This Mode 21 PID only answers when addressed straight to the engine
  // ECU. Switch there just for this one query, then switch back to the
  // default broadcast header so every other PID reaches whichever module
  // actually owns it.
  sendCommand("ATSH7E0");
  String hex = cleanHex(sendCommand("21D9"));
  sendCommand("ATSH7DF");
  bool ok = false;
  int idx = hex.indexOf("61D9");
  if (idx >= 0) {
    int d = idx + 4;
    if ((int)hex.length() >= d + 16) {
      int E = hexByte(hex, d + 8),  F = hexByte(hex, d + 10);
      int G = hexByte(hex, d + 12), H = hexByte(hex, d + 14);
      gaugeVal[G_TRANS_PAN] = ((E * 256 + F) / 256) - 40;
      gaugeVal[G_TRANS_TC]  = ((G * 256 + H) / 256) - 40;
      ok = true;
    }
  }
  noteResult(G_TRANS_PAN, ok);
  noteResult(G_TRANS_TC, ok);
}

void pollGauge(GaugeId g) {
  int A = 0, B = 0;
  bool ok = false;
  float v = 0;
  switch (g) {
    case G_RPM:       ok = queryStd(0x0C, A, B); v = ((A * 256) + B) / 4.0f;  break;
    case G_SPEED:     ok = queryStd(0x0D, A, B); v = A;                       break;
    case G_COOLANT:   ok = queryStd(0x05, A, B); v = A - 40;                  break;
    case G_THROTTLE:  ok = queryStd(0x11, A, B); v = A * 100.0f / 255.0f;     break;
    case G_IAT:       ok = queryStd(0x0F, A, B); v = A - 40;                  break;
    case G_LOAD:      ok = queryStd(0x04, A, B); v = A * 100.0f / 255.0f;     break;
    case G_MAF:       ok = queryStd(0x10, A, B); v = ((A * 256) + B) / 100.0f; break;
    case G_BATT:      ok = queryBattery(v);                                   break;
    // Fuel trims: raw byte -100..+99.2%, formula per SAE J1979
    case G_STFT1:     ok = queryStd(0x06, A, B); v = (A - 128) * 100.0f / 128.0f; break;
    case G_LTFT1:     ok = queryStd(0x07, A, B); v = (A - 128) * 100.0f / 128.0f; break;
    case G_STFT2:     ok = queryStd(0x08, A, B); v = (A - 128) * 100.0f / 128.0f; break;
    case G_LTFT2:     ok = queryStd(0x09, A, B); v = (A - 128) * 100.0f / 128.0f; break;
    // The legacy narrow-band O2 voltage PIDs (0x14/0x18) come back empty on
    // this engine -- like most Toyota/Lexus engines from this era, the
    // 1GR-FE uses wideband air-fuel ratio sensors upstream, which report as
    // an equivalence ratio (1.00 = stoichiometric, <1 = rich, >1 = lean)
    // instead of a raw voltage. A,B here are the first two of that PID's 4
    // response bytes; formula: ratio = ((A*256)+B) * 2 / 65536, range 0..2.
    //
    // Which wideband PID (0x24-0x2B) maps to which physical sensor isn't
    // bank/sensor order -- it's PID = 0x24 + (bit index in the sensor
    // location bitmap, PID 0x13). This ECU's PID 0x13 result (0x33 =
    // 00110011) has bits 0, 1, 4 and 5 set: bit0=Bank1Sensor1 -> PID 0x24,
    // bit1=Bank1Sensor2 -> 0x25, bit4=Bank2Sensor1 -> PID 0x28,
    // bit5=Bank2Sensor2 -> 0x29. Only the two upstream sensors (0x24, 0x28)
    // are in this ECU's supported-PID list -- the downstream ones (0x25,
    // 0x29) aren't exposed as wideband readings, which is normal (catalyst
    // monitoring uses the fuel-trim/switching data instead).
    case G_O2_B1S1:   ok = queryStd(0x24, A, B); v = ((A * 256) + B) * 2.0f / 65536.0f; break;
    case G_O2_B2S1:   ok = queryStd(0x28, A, B); v = ((A * 256) + B) * 2.0f / 65536.0f; break;
    case G_TIMING:    ok = queryStd(0x0E, A, B); v = (A / 2.0f) - 64.0f;       break;
    case G_MODULE_V:  ok = queryStd(0x42, A, B); v = ((A * 256) + B) / 1000.0f; break;
    case G_BARO:      ok = queryStd(0x33, A, B); v = A;                       break;
    case G_RUNTIME:   ok = queryStd(0x1F, A, B); v = ((A * 256) + B) / 60.0f;  break; // seconds -> minutes
    case G_DIST_CLR:  ok = queryStd(0x31, A, B); v = (A * 256) + B;           break; // km
    case G_TRANS_PAN: pollTrans(); return;
    default:          return;
  }
  if (ok) gaugeVal[g] = v;
  noteResult(g, ok);
}

// Builds the list of PIDs the current page actually needs.
void buildPollList() {
  for (int i = 0; i < G_COUNT; i++) needed[i] = false;
  const PageDef &p = pages[currentPage];
  if (!p.isDiag) {
    for (int i = 0; i < p.count; i++) needed[p.gauges[i]] = true;
  }
  if (needed[G_MPG])      { needed[G_MAF] = true; needed[G_SPEED] = true; }
  if (needed[G_TRANS_TC]) { needed[G_TRANS_PAN] = true; }

  pollCount = 0;
  for (int g = 0; g < G_COUNT; g++) {
    if (needed[g] && g != G_TRANS_TC && g != G_MPG) pollList[pollCount++] = g;
  }
  pollIdx = 0;
}

// Instant MPG from MAF + speed (gasoline, 14.7:1 air/fuel ratio):
//   fuel mass flow (g/s)  = MAF / 14.7
//   fuel mass flow (g/hr) = that * 3600
//   gasoline density      = 6.17 lb/gal = 6.17 * 453.592 = 2798.66 g/gal
//   gallons/hr (GPH)      = MAF * 3600 / (14.7 * 2798.66) = MAF * 0.0875
//   MPG                   = mph / GPH = (1/0.0875) * mph/MAF = 11.43 * mph/MAF
// The old constant here (710.7) was about 62x too large, which is why this
// always either showed 0.0 (stationary, below the 0.5 mph cutoff below) or
// pegged at the 99.9 display cap (any real MAF reading while moving blew
// the result way past it).
const float MPG_CONST = 11.43f;

void updateDerived() {
  if (!needed[G_MPG]) return;
  if (gaugeOk[G_SPEED] && gaugeOk[G_MAF]) {
    float mph = gaugeVal[G_SPEED] * 0.621371f;
    float maf = gaugeVal[G_MAF];
    float mpg = 0;
    if (maf > 0.5f && mph > 0.5f) mpg = MPG_CONST * mph / maf;
    if (mpg > 99.9f) mpg = 99.9f;
    gaugeVal[G_MPG] = mpg;
    gaugeOk[G_MPG] = true;
  } else {
    gaugeOk[G_MPG] = false;
  }
}

// =========================================================================
//                          GAUGE FORMATTING
// =========================================================================
String formatGauge(GaugeId g) {
  if (!gaugeOk[g]) return String("---");
  float v = gaugeVal[g];
  switch (g) {
    case G_SPEED:
    case G_DIST_CLR:                     // same km->mi conversion as speed
      if (USE_IMPERIAL) v *= 0.621371f;
      return String((int)lroundf(v));
    case G_COOLANT:
    case G_TRANS_PAN:
    case G_TRANS_TC:
    case G_IAT:
      if (USE_IMPERIAL) v = v * 1.8f + 32.0f;
      return String((int)lroundf(v));
    case G_MAF:
    case G_BATT:
    case G_MODULE_V:
      return String(v, 1);
    case G_STFT1:
    case G_LTFT1:
    case G_STFT2:
    case G_LTFT2:
      return String(v, 1);
    case G_O2_B1S1:
    case G_O2_B2S1:
      return String(v, 2);
    case G_MPG:
      if (!USE_IMPERIAL) v *= 0.425144f;
      return String(v, 1);
    default:
      return String((int)lroundf(v));
  }
}

bool isAlarm(GaugeId g) {
  if (!gaugeOk[g]) return false;
  float v = gaugeVal[g];
  return v > gaugeDefs[g].alarmHigh || v < gaugeDefs[g].alarmLow;
}

const char* unitFor(GaugeId g) {
  return USE_IMPERIAL ? gaugeDefs[g].unitImperial : gaugeDefs[g].unitMetric;
}

// =========================================================================
//                       INDICATOR STRIP (bottom)
// =========================================================================
void drawHeartbeat() {
  int y = tft.height() - IND_H / 2;
  tft.fillCircle(tft.width() - 14, y, 4, heartbeat ? TFT_WHITE : COL_DIV);
}

void drawStatusText() {
  int y0 = tft.height() - IND_H;
  tft.fillRect(22, y0 + 1, 170, IND_H - 1, TFT_BLACK);
  if (statusMsg.length() == 0) return;
  useFont(F_LABEL);
  tft.setTextDatum(ML_DATUM);
  tft.setTextColor(statusColor, TFT_BLACK);
  tft.drawString(statusMsg, 24, y0 + IND_H / 2);
}

void showStatus(const char* msg, uint16_t color) {
  statusMsg = msg;
  statusColor = color;
  drawStatusText();
}

void drawIndicator() {
  int W = tft.width();
  int y0 = tft.height() - IND_H;
  int cy = y0 + IND_H / 2;

  tft.fillRect(0, y0, W, IND_H, TFT_BLACK);
  tft.drawFastHLine(0, y0, W, COL_DIV);

  tft.fillCircle(12, cy, 5, connected ? TFT_GREEN : TFT_RED);

  int spacing = 18;
  int startX = W / 2 - ((PAGE_COUNT - 1) * spacing) / 2;
  for (int i = 0; i < PAGE_COUNT; i++) {
    bool cur = (i == currentPage);
    tft.fillCircle(startX + i * spacing, cy, cur ? 5 : 3, cur ? TFT_WHITE : TFT_DARKGREY);
  }

  drawHeartbeat();
  drawStatusText();
}

// =========================================================================
//                           GAUGE PAGES
// =========================================================================
void layoutFor(int count, int &cols, int &rows) {
  switch (count) {
    case 1:  cols = 1; rows = 1; break;
    case 4:  cols = 2; rows = 2; break;
    case 8:  cols = 4; rows = 2; break;
    default: cols = 3; rows = 2; break;   // 6
  }
}

void zoneRect(int i, int &x, int &y, int &w, int &h) {
  int cols, rows;
  layoutFor(pages[currentPage].count, cols, rows);
  int areaH = tft.height() - IND_H;
  w = tft.width() / cols;
  h = areaH / rows;
  x = (i % cols) * w;
  y = (i / cols) * h;
}

void drawGaugePageStatic() {
  const PageDef &p = pages[currentPage];
  int cols, rows;
  layoutFor(p.count, cols, rows);
  int areaH = tft.height() - IND_H;

  tft.fillScreen(TFT_BLACK);

  for (int c = 1; c < cols; c++)
    tft.drawFastVLine(c * (tft.width() / cols), 6, areaH - 12, COL_DIV);
  for (int r = 1; r < rows; r++)
    tft.drawFastHLine(6, r * (areaH / rows), tft.width() - 12, COL_DIV);

  for (int i = 0; i < p.count; i++) {
    int x, y, w, h;
    zoneRect(i, x, y, w, h);
    GaugeId g = p.gauges[i];

    useFont(F_LABEL);
    tft.setTextDatum(TC_DATUM);
    tft.setTextColor(COL_LABEL, TFT_BLACK);
    tft.drawString(gaugeDefs[g].label, x + w / 2, y + 6);

    tft.setTextDatum(BC_DATUM);
    tft.setTextColor(COL_UNIT, TFT_BLACK);
    tft.drawString(unitFor(g), x + w / 2, y + h - 6);

    zoneStr[i] = "~";      // force the first value draw
    zoneColor[i] = 0;
  }
}

void drawBar(int x, int y, int w, int h, GaugeId g, uint16_t col) {
  int bx = x + 24;
  int bw = w - 48;
  int bh = 18;
  int by = y + h - BOT_PAD - 34;

  tft.drawRect(bx, by, bw, bh, COL_UNIT);
  tft.fillRect(bx + 1, by + 1, bw - 2, bh - 2, TFT_BLACK);
  if (!gaugeOk[g]) return;

  const GaugeDef &d = gaugeDefs[g];
  float f = (gaugeVal[g] - d.barMin) / (d.barMax - d.barMin);
  if (f < 0) f = 0;
  if (f > 1) f = 1;
  int fw = (int)((bw - 2) * f);
  if (fw > 0) tft.fillRect(bx + 1, by + 1, fw, bh - 2, col);
}

void drawZoneValue(int i, const String &s, uint16_t col) {
  const PageDef &p = pages[currentPage];
  int x, y, w, h;
  zoneRect(i, x, y, w, h);

  int vy = y + TOP_PAD;
  int vh = h - TOP_PAD - BOT_PAD;
  if (p.count == 1) vh -= 50;         // leave room for the bar

  tft.fillRect(x + 2, vy, w - 4, vh, TFT_BLACK);
  useFont(valueFontFor(p.count));
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(col, TFT_BLACK);
  tft.drawString(s, x + w / 2, vy + vh / 2);

  if (p.count == 1) drawBar(x, y, w, h, p.gauges[i], col);
}

void refreshGaugePage() {
  const PageDef &p = pages[currentPage];
  if (p.isDiag) return;

  bool blinkOn = ((millis() / 400) % 2) == 0;
  for (int i = 0; i < p.count; i++) {
    GaugeId g = p.gauges[i];
    String s = formatGauge(g);

    uint16_t col = COL_VALUE;
    if (!gaugeOk[g]) col = COL_DIM;
    else if (isAlarm(g)) col = blinkOn ? COL_ALARM : COL_ALARM_DIM;

    if (s != zoneStr[i] || col != zoneColor[i]) {
      drawZoneValue(i, s, col);
      zoneStr[i] = s;
      zoneColor[i] = col;
    }
  }
}

// =========================================================================
//                        DIAGNOSTICS PAGE (DTCs)
// =========================================================================
bool pointInRect(int x, int y, const BtnRect &r) {
  return x >= r.x && x <= r.x + r.w && y >= r.y && y <= r.y + r.h;
}

void drawButton(const BtnRect &r, const char* label, uint16_t color) {
  tft.fillRect(r.x, r.y, r.w, r.h, TFT_BLACK);
  tft.drawRoundRect(r.x, r.y, r.w, r.h, 8, color);
  useFont(F_TITLE);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(color, TFT_BLACK);
  tft.drawString(label, r.x + r.w / 2, r.y + r.h / 2);
}

void drawClearButton() {
  if (clearArmedUntil != 0 && millis() < clearArmedUntil)
    drawButton(clearBtn, "Confirm?", TFT_RED);
  else
    drawButton(clearBtn, "Clear Codes", TFT_ORANGE);
}

void drawDtcList() {
  int y0 = readBtn.y + readBtn.h + 16;
  int y1 = tft.height() - IND_H - 4;
  tft.fillRect(0, y0, tft.width(), y1 - y0, TFT_BLACK);

  useFont(F_TITLE);
  tft.setTextDatum(TL_DATUM);

  if (diagMsg.length() > 0) {
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.drawString(diagMsg, 20, y0);
    return;
  }
  if (!dtcRead) {
    tft.setTextColor(COL_UNIT, TFT_BLACK);
    tft.drawString("Tap Read Codes", 20, y0);
    return;
  }
  if (dtcCodes.empty()) {
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.drawString("No trouble codes", 20, y0);
    return;
  }

  int lineH = 30;
  int maxLines = (y1 - y0) / lineH;
  int shown = min((int)dtcCodes.size(), maxLines);
  tft.setTextColor(TFT_RED, TFT_BLACK);
  for (int i = 0; i < shown; i++) tft.drawString(dtcCodes[i], 20, y0 + i * lineH);
  if ((int)dtcCodes.size() > shown) {
    tft.setTextColor(COL_UNIT, TFT_BLACK);
    tft.drawString("+" + String((int)dtcCodes.size() - shown) + " more", 20, y0 + shown * lineH);
  }
}

void drawDiagPage() {
  int W = tft.width();
  readBtn.x = 20;
  readBtn.w = (W - 60) / 2;
  clearBtn.x = 40 + readBtn.w;
  clearBtn.w = readBtn.w;

  tft.fillScreen(TFT_BLACK);
  drawButton(readBtn, "Read Codes", TFT_CYAN);
  drawClearButton();
  drawDtcList();
}

String decodeDTC(int A, int B) {
  const char* letters = "PCBU";
  char letter = letters[(A >> 6) & 0x03];
  int d1 = (A >> 4) & 0x03;
  int d2 = A & 0x0F;
  int d3 = (B >> 4) & 0x0F;
  int d4 = B & 0x0F;
  char buf[8];
  snprintf(buf, sizeof(buf), "%c%d%X%X%X", letter, d1, d2, d3, d4);
  return String(buf);
}

// On CAN, the byte right after 43 is the number of codes. Older
// (non-CAN) protocols have no count byte.
std::vector<String> parseDTCs(const String &raw) {
  std::vector<String> codes;
  String hex = cleanHex(raw);
  int i = hex.indexOf("43");
  if (i < 0) return codes;
  hex = hex.substring(i);

  int pos = 2;
  int maxPairs = 20;
  if (isCan) {
    if (hex.length() < 4) return codes;
    maxPairs = hexByte(hex, 2);
    pos = 4;
  }
  for (int n = 0; n < maxPairs && pos + 4 <= (int)hex.length(); n++, pos += 4) {
    int A = hexByte(hex, pos);
    int B = hexByte(hex, pos + 2);
    if (A == 0 && B == 0) continue;
    codes.push_back(decodeDTC(A, B));
  }
  return codes;
}

void readDTCs() {
  if (!connected) { diagMsg = "Not connected"; drawDtcList(); return; }
  diagMsg = "Reading...";
  drawDtcList();
  String r = sendCommandT("03", 2500);
  dtcCodes = parseDTCs(r);
  dtcRead = true;
  diagMsg = "";
  drawDtcList();
}

void clearDTCs() {
  if (!connected) { diagMsg = "Not connected"; drawDtcList(); return; }
  diagMsg = "Clearing...";
  drawDtcList();
  sendCommandT("04", 2500);
  dtcCodes.clear();
  dtcRead = true;
  diagMsg = "";
  drawDtcList();
}

void handleDiagTap(int x, int y) {
  if (pointInRect(x, y, readBtn)) {
    clearArmedUntil = 0;
    drawClearButton();
    readDTCs();
  } else if (pointInRect(x, y, clearBtn)) {
    if (clearArmedUntil != 0 && millis() < clearArmedUntil) {
      clearArmedUntil = 0;
      drawClearButton();
      clearDTCs();
    } else {
      clearArmedUntil = millis() + 4000;   // clearing also resets readiness monitors: confirm first
      drawClearButton();
    }
  }
}

// =========================================================================
//                      PAGE SWITCHING AND TOUCH
// =========================================================================
void goToPage(int pg) {
  currentPage = (pg + PAGE_COUNT) % PAGE_COUNT;
  lastPageChange = millis();
  buildPollList();

  if (pages[currentPage].isDiag) drawDiagPage();
  else drawGaugePageStatic();

  drawIndicator();
  if (!pages[currentPage].isDiag) refreshGaugePage();
}

void nextPage() { goToPage(currentPage + 1); }
void prevPage() { goToPage(currentPage - 1); }

void onTap(int x, int y) {
  if (pages[currentPage].isDiag) {
    // Only a tap that actually lands on Read/Clear should trigger those
    // buttons — anywhere else on the DIAG page behaves like every other
    // page and advances forward, so a plain tap can never leave you
    // stuck here the way it could before (swipe was always an out, but
    // a tap used to do nothing at all off the buttons).
    if (pointInRect(x, y, readBtn) || pointInRect(x, y, clearBtn)) {
      handleDiagTap(x, y);
    } else {
      nextPage();
    }
  } else {
    nextPage();                          // like UltraGauge's single key press
  }
}

void handleTouch() {
  uint16_t tx, ty;
  bool touched = tft.getTouch(&tx, &ty);

  if (touched) {
    if (!wasTouched) {
      touchStartX = tx;
      touchStartY = ty;
      touchStartTime = millis();
      recalTriggered = false;
    }
    lastTouchX = tx;
    lastTouchY = ty;
    wasTouched = true;

    if (!recalTriggered && touchStartX < 40 && touchStartY < 40 &&
        millis() - touchStartTime > RECAL_HOLD_MS) {
      recalTriggered = true;
      runTouchCalibration();
      goToPage(currentPage);
      wasTouched = false;
    }
  } else if (wasTouched) {
    wasTouched = false;
    int dx = lastTouchX - touchStartX;
    int dy = lastTouchY - touchStartY;
    unsigned long dt = millis() - touchStartTime;

    if (abs(dx) > SWIPE_MIN_PX && abs(dx) > abs(dy) && dt < SWIPE_MAX_MS) {
      if (dx < 0) nextPage(); else prevPage();
    } else if (abs(dx) < 20 && abs(dy) < 20 && dt < 700) {
      onTap(touchStartX, touchStartY);
    }
  }
}

// =========================================================================
//                    LOW-LEVEL BLUETOOTH PAIRING (GAP)
// =========================================================================
// Registered after SerialBT.begin(). Replaces BluetoothSerial's own internal
// GAP callback — fine here since we never call SerialBT.discover() during
// normal operation (only the separate bt_scanner.ino sketch does that).
void btGapCallback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
  Serial.printf("GAP: event fired, id=%d\n", (int)event); // logs EVERY GAP event, handled or not
  switch (event) {
    case ESP_BT_GAP_PIN_REQ_EVT: {
      Serial.println("GAP: PIN requested by remote — replying with fixed PIN");
      esp_bt_pin_code_t pinCode;
      int len = strlen(elmPin);
      memcpy(pinCode, elmPin, len);
      esp_bt_gap_pin_reply(param->pin_req.bda, true, len, pinCode);
      break;
    }
    case ESP_BT_GAP_CFM_REQ_EVT:
      // Some devices use Secure Simple Pairing "just works" confirmation
      // instead of a legacy PIN. Auto-accept it.
      Serial.println("GAP: SSP confirmation requested — auto-accepting");
      esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
      break;
    case ESP_BT_GAP_AUTH_CMPL_EVT:
      if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
        Serial.println("GAP: pairing/authentication SUCCEEDED");
      } else {
        Serial.printf("GAP: pairing/authentication FAILED, status=%d\n", param->auth_cmpl.stat);
      }
      break;
    case ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT:
      Serial.printf("GAP: ACL link CONNECTED, status=%d (0=SUCCESS)\n",
                     (int)param->acl_conn_cmpl_stat.stat);
      break;
    case ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT:
      Serial.printf("GAP: ACL link DISCONNECTED, reason=%d\n",
                     (int)param->acl_disconn_cmpl_stat.reason);
      break;
    default:
      break;
  }
}

void setupPairing() {
  SerialBT.setPin(elmPin, strlen(elmPin)); // library-level PIN, for the standard connect() path
  esp_bt_pin_code_t pinCode;
  int len = strlen(elmPin);
  memcpy(pinCode, elmPin, len);
  esp_bt_gap_set_pin(ESP_BT_PIN_TYPE_FIXED, len, pinCode);
  esp_bt_gap_register_callback(btGapCallback);

  // Forget any existing bond for this MAC before connecting, so every boot
  // starts from a clean slate rather than silently trying to reuse a stale
  // link key.
  esp_err_t rm = esp_bt_gap_remove_bond_device(elmAddress);
  Serial.printf("Removed existing bond for ELM327 (if any): result=%d (0=ESP_OK)\n", (int)rm);
}

// =========================================================================
//                       CONNECTION MANAGEMENT
// =========================================================================
// The original unbranded "OBDII" clone adapter never accepted a connection
// no matter what was tried here (name vs MAC connect, SerialBT.setPin(),
// a custom low-level GAP callback, raw esp_spp_connect() with various
// security flags) — it always aborted the link immediately with no pairing
// negotiation, which turned out to be a limitation of that specific clone's
// Bluetooth stack rather than anything on the ESP32 side. Switching to a
// Vgate adapter (real CSR Bluetooth chipset) let the plain, standard
// SerialBT.connect(address) succeed right away, so that's what's used here
// now — no raw esp_spp_connect() workaround needed. The GAP callback above
// is still registered so we keep the detailed connection logging either way.
bool connectRaw() {
  Serial.println("connectRaw(): calling SerialBT.connect(address) (standard)...");
  bool ok = SerialBT.connect(elmAddress);
  Serial.printf("connectRaw(): SerialBT.connect() returned %s\n", ok ? "true" : "false");

  if (ok && SerialBT.connected()) {
    Serial.println("connectRaw(): SerialBT.connected() is true");
    return true;
  }

  unsigned long start = millis();
  while (millis() - start < 4000) {
    if (SerialBT.connected()) {
      Serial.println("connectRaw(): SerialBT.connected() became true");
      return true;
    }
    delay(200);
  }
  Serial.println("connectRaw(): connect failed / not connected");
  return false;
}

void tryConnect() {
  lastConnectTry = millis();
  showStatus("Connecting...", TFT_YELLOW);
  drawIndicator();

  const int MAX_ATTEMPTS = 4;
  for (int attempt = 1; attempt <= MAX_ATTEMPTS; attempt++) {
    Serial.printf("Connect attempt %d/%d...\n", attempt, MAX_ATTEMPTS);
    connected = connectRaw();
    Serial.println(connected ? "ELM327 connected" : "ELM327 connect failed");
    if (connected) break;
    if (attempt < MAX_ATTEMPTS) delay(1500);
  }

  if (connected) {
    initELM327();
    showStatus("", TFT_GREEN);
  } else {
    showStatus("No ELM327", TFT_RED);
  }
  drawIndicator();
}

void markDisconnected() {
  SerialBT.disconnect();
  connected = false;
  for (int i = 0; i < G_COUNT; i++) gaugeOk[i] = false;
  showStatus("Link lost", TFT_RED);
  drawIndicator();
  refreshGaugePage();
  lastConnectTry = millis();
}

// =========================================================================
//                              SETUP / LOOP
// =========================================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("=================================");
  Serial.println("BOOT MARKER: sketch build v7 (Vgate adapter, standard connect)");
  Serial.println("=================================");

  tft.init();
  tft.setRotation(SCREEN_ROTATION);   // landscape, 480 x 320, USB-C on the left
  tft.fillScreen(TFT_BLACK);

  loadOrCalibrateTouch();             // 4-corner prompt only on the very first boot

  goToPage(0);

  SerialBT.begin("ESP32_OBD", true);
  delay(1000);   // let SerialBT.begin()'s internal (partly async) setup finish
                 // before we register our own GAP callback over it

  setupPairing();
  Serial.println("GAP callback registered, pin type set to FIXED");
  tryConnect();
}

void loop() {
  handleTouch();

  if (!connected) {
    // connect() blocks for several seconds, so retry only occasionally
    if (millis() - lastConnectTry > 20000) tryConnect();
    delay(10);
    return;
  }

  if (commFail >= 6) {
    markDisconnected();
    return;
  }

  if (pages[currentPage].isDiag) {
    if (clearArmedUntil != 0 && millis() >= clearArmedUntil) {
      clearArmedUntil = 0;
      drawClearButton();
    }
    delay(10);
    return;
  }

  if (pollCount > 0) {
    GaugeId g = (GaugeId)pollList[pollIdx];
    pollIdx = (pollIdx + 1) % pollCount;
    pollGauge(g);
    if (pollIdx == 0) {                 // one full pass through the list
      heartbeat = !heartbeat;
      drawHeartbeat();
    }
  }

  updateDerived();
  refreshGaugePage();

  if (AUTO_CYCLE_MS > 0 && millis() - lastPageChange > AUTO_CYCLE_MS) {
    int n = (currentPage + 1) % PAGE_COUNT;
    if (pages[n].isDiag) n = 0;
    goToPage(n);
  }
}
