/*****************************************************************************
 * main.cpp — "Don't Get Lost" direction-pointer UI
 *
 * A round-watch style compass screen for the Waveshare ESP32-S3 Touch LCD
 * 1.28" (GC9A01, 240x240) that points a big arrow toward a locked target
 * bearing, using the BNO08x SH2_ROTATION_VECTOR report — the sensor-fused
 * (accelerometer + gyroscope + magnetometer) orientation, referenced to
 * magnetic north. The gyro gives fast response, the magnetometer pins yaw
 * to north (BNO08x datasheet §2.2.4).
 *
 * Controls (CST816S touch):
 *   - Tap center of the dial ....... lock the current heading as the target
 *   - Tap START / STOP pill ........ start / stop a walk (time + est. dist)
 *
 * Distance comes from the BN-880 GPS (u-blox M8N, UART1 @ 9600) when it has
 * a fix, and falls back to the BNO08x step counter estimate when it doesn't.
 * Wiring: BN-880 TX -> GPIO17, BN-880 RX -> GPIO18, VCC -> 3V3, GND -> GND.
 *
 * Build & flash:   pio run -t upload
 *****************************************************************************/
#include <Arduino.h>
#include "LCD_Test.h"
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <TinyGPSPlus.h>
#include <math.h>
#include "wmm.h"
#include "route_store.h"
#include "route_ble.h"
#include "route_nav.h"

/* ------------------------------------------------------------------ */
/*  Configuration                                                      */
/* ------------------------------------------------------------------ */
static const int   BNO_SDA            = 15;
static const int   BNO_SCL            = 16;
static const uint8_t BNO_ADDR         = 0x4B;

// BN-880 GPS on UART1 (module TX -> GPS_RX, module RX -> GPS_TX)
static const int   GPS_RX             = 17;
static const int   GPS_TX             = 18;
static const uint32_t GPS_BAUD       = 9600;
// below this, successive fixes are treated as jitter, not movement
static const float GPS_MIN_STEP_M     = 2.0f;

// Mounting / calibration tweaks:
// extra rotation between the BNO08x X/Y axes and the screen "up" direction
static const float HEADING_OFFSET_DEG = 0.0f;
// magnetic -> true north correction. Default is the SF Bay Area (WMM2025 at
// 2026.6: SF 12.85E, Oakland 12.83E, San Jose 12.67E); once the GPS has a
// fix and date, it is replaced by the WMM value for the actual position.
static const float MAG_DECLINATION_DEFAULT = 12.8f;
// recompute declination after moving this far from where it was computed
static const float DECL_RECOMPUTE_M       = 10000.0f;

static const float STRIDE_M           = 0.72f;  // avg step length, meters
static const uint32_t FRAME_MS        = 50;     // ~20 fps while moving
static const float SMOOTH_ALPHA       = 0.30f;  // heading low-pass strength

/* Power saving */
static const uint32_t CPU_MHZ         = 80;     // min for BLE; APB (SPI/UART) unaffected
static const uint32_t FRAME_IDLE_MS   = 500;    // refresh rate when nothing moves
static const float    ACTIVE_HDG_DEG  = 0.8f;   // heading delta that counts as motion
static const uint8_t  BL_ACTIVE       = 90;     // backlight, recent interaction
static const uint8_t  BL_DIM          = 12;     // backlight, idle
static const uint32_t BL_DIM_AFTER_MS = 20000;

/* ------------------------------------------------------------------ */
/*  Colors (RGB565)                                                    */
/* ------------------------------------------------------------------ */
#define COL_BG        BLACK
#define COL_TICK      0x7BEF   // mid gray
#define COL_TICK_MAJ  WHITE
#define COL_CARDINAL  WHITE
#define COL_INTERCARD GRAY
#define COL_NORTH     RED
#define COL_TEXT      WHITE
#define COL_DIM       GRAY
#define COL_GOOD      0x07E0   // green: on course
#define COL_WARN      0xFD20   // orange: adjust
#define COL_BAD       0xF800   // red: wrong way
#define COL_TARGET    0x07FF   // cyan: target marker
#define COL_BTN_GO    0x03E0   // dark green pill
#define COL_BTN_STOP  0xB000   // dark red pill

/* ------------------------------------------------------------------ */
/*  Globals                                                            */
/* ------------------------------------------------------------------ */
UWORD *BlackImage = NULL;                 // framebuffer (referenced by LCD_1in28.cpp)
CST816S touch(6, 7, 13, 5);               // sda, scl, rst, irq
Adafruit_BNO08x bno08x;
TinyGPSPlus gps;

enum AppState { ST_IDLE, ST_TRACKING };
static AppState  state          = ST_IDLE;
static bool      imuOk          = false;

static float     headingDisp    = 0.0f;   // smoothed heading, deg [0,360)
static float     smoothSin      = 0.0f;
static float     smoothCos      = 1.0f;
static uint8_t   headingAcc     = 0;      // BNO08x status 0(bad)..3(good)
static bool      haveHeading    = false;

static float     targetBearing  = 0.0f;   // deg [0,360)
static bool      targetSet      = false;

static uint32_t  startMs        = 0;
static uint32_t  finalElapsedMs = 0;
static uint32_t  trackedSteps   = 0;
static uint16_t  lastStepCount  = 0;
static bool      haveStepBase   = false;

static uint32_t  lastFrameMs    = 0;
static uint32_t  lastTapMs      = 0;
static uint32_t  lockFlashUntil = 0;      // "TARGET SET" flash timer
static uint32_t  lastInteractMs = 0;      // backlight / frame-rate boost
static float     lastDrawnHdg   = -999.0f;
static bool      menuDirty      = false;  // menu screen redraws on demand
static bool      gpsCfgDone     = false;

static bool      gpsFix         = false;  // valid & recent position
static uint32_t  gpsLastDataMs  = 0;      // last time NMEA bytes arrived
static double    gpsLastLat     = 0.0;    // anchor of the current segment
static double    gpsLastLng     = 0.0;
static bool      gpsHaveAnchor  = false;
static float     gpsDistanceM   = 0.0f;   // accumulated over the walk
static bool      gpsDistUsed    = false;  // walk has >= 1 GPS segment

static float     magDeclinationDeg = MAG_DECLINATION_DEFAULT;
static double    declLat        = 0.0;    // where declination was computed
static double    declLng        = 0.0;
static bool      declFromGps    = false;

static double    gpsCurLat      = 0.0;    // most recent valid fix
static double    gpsCurLng      = 0.0;

/* Route mode / menu */
enum UiScreen { SCR_MAIN, SCR_MENU };
static UiScreen  uiScreen       = SCR_MAIN;
static bool      routeMode      = false;  // arrow follows route checkpoints
static int       activeSlot     = -1;
static RouteMeta menuMeta[ROUTE_SLOTS];
static RoutePoint *routeBuf     = NULL;
static char      toastMsg[24]   = "";
static uint32_t  toastUntil     = 0;

/* Screen geometry */
static const int CX = 120, CY = 120;
/* START/STOP pill hit box */
static const int BTN_X0 = 74, BTN_X1 = 166, BTN_Y0 = 192, BTN_Y1 = 220;

/* ------------------------------------------------------------------ */
/*  Small math / drawing helpers                                       */
/* ------------------------------------------------------------------ */
static inline float wrap360(float d) {
  d = fmodf(d, 360.0f);
  return d < 0 ? d + 360.0f : d;
}
static inline float wrap180(float d) {
  d = wrap360(d);
  return d > 180.0f ? d - 360.0f : d;
}
/* Polar -> screen. bearing 0 = up (12 o'clock), clockwise positive. */
static inline int polarX(float bearingDeg, float r) {
  return (int)lroundf(CX + r * sinf(bearingDeg * DEG_TO_RAD));
}
static inline int polarY(float bearingDeg, float r) {
  return (int)lroundf(CY - r * cosf(bearingDeg * DEG_TO_RAD));
}

static void hLine(int x0, int x1, int y, UWORD color) {
  if (y < 0 || y > 239) return;
  if (x0 > x1) { int t = x0; x0 = x1; x1 = t; }
  if (x1 < 0 || x0 > 239) return;
  if (x0 < 0) x0 = 0;
  if (x1 > 239) x1 = 239;
  Paint_DrawLine(x0, y, x1, y, color, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
}

/* Scanline-filled triangle (Paint lib only has outline primitives). */
static void fillTriangle(int x0, int y0, int x1, int y1, int x2, int y2, UWORD color) {
  if (y0 > y1) { int t; t = x0; x0 = x1; x1 = t; t = y0; y0 = y1; y1 = t; }
  if (y1 > y2) { int t; t = x1; x1 = x2; x2 = t; t = y1; y1 = y2; y2 = t; }
  if (y0 > y1) { int t; t = x0; x0 = x1; x1 = t; t = y0; y0 = y1; y1 = t; }
  if (y2 == y0) { hLine(min(x0, min(x1, x2)), max(x0, max(x1, x2)), y0, color); return; }
  for (int y = y0; y <= y2; y++) {
    float xa = x0 + (float)(x2 - x0) * (y - y0) / (float)(y2 - y0);
    float xb;
    if (y < y1)
      xb = (y1 == y0) ? x1 : x0 + (float)(x1 - x0) * (y - y0) / (float)(y1 - y0);
    else
      xb = (y2 == y1) ? x1 : x1 + (float)(x2 - x1) * (y - y1) / (float)(y2 - y1);
    hLine((int)xa, (int)xb, y, color);
  }
}

static void clampedLine(int x0, int y0, int x1, int y1, UWORD color, DOT_PIXEL w) {
  if (x0 < 0) x0 = 0; if (x0 > 239) x0 = 239;
  if (x1 < 0) x1 = 0; if (x1 > 239) x1 = 239;
  if (y0 < 0) y0 = 0; if (y0 > 239) y0 = 239;
  if (y1 < 0) y1 = 0; if (y1 > 239) y1 = 239;
  Paint_DrawLine(x0, y0, x1, y1, color, w, LINE_STYLE_SOLID);
}

static void drawStringCentered(int cx, int y, const char *s, sFONT *font, UWORD fg, UWORD bg) {
  int x = cx - (int)(strlen(s) * font->Width) / 2;
  if (x < 0) x = 0;
  Paint_DrawString_EN(x, y, s, font, fg, bg);
}

/* Filled pill (rounded-rect button). */
static void drawPill(int x0, int y0, int x1, int y1, UWORD fill) {
  int r = (y1 - y0) / 2;
  int cy = (y0 + y1) / 2;
  Paint_DrawRectangle(x0 + r, y0, x1 - r, y1, fill, DOT_PIXEL_1X1, DRAW_FILL_FULL);
  Paint_DrawCircle(x0 + r, cy, r, fill, DOT_PIXEL_1X1, DRAW_FILL_FULL);
  Paint_DrawCircle(x1 - r, cy, r, fill, DOT_PIXEL_1X1, DRAW_FILL_FULL);
}

static const char *CARDINAL16[16] = {
  "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
  "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"
};
static const char *cardinalName(float deg) {
  return CARDINAL16[(int)((wrap360(deg) + 11.25f) / 22.5f) % 16];
}

/* ------------------------------------------------------------------ */
/*  BNO08x                                                             */
/* ------------------------------------------------------------------ */
static void imuEnableReports() {
  // Fused orientation (gyro + accel + magnetometer), 20 Hz — plenty for a
  // walking compass, and cheaper than 50 Hz for the sensor, I2C, and CPU
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR, 50000))
    Serial.println("Failed to enable rotation vector");
  // Step counter for distance estimate until GPS exists, 2 Hz
  if (!bno08x.enableReport(SH2_STEP_COUNTER, 500000))
    Serial.println("Failed to enable step counter");
}

static void imuInit() {
  Wire1.begin(BNO_SDA, BNO_SCL);
  Wire1.setClock(100000);
  imuOk = bno08x.begin_I2C(BNO_ADDR, &Wire1);
  if (imuOk) {
    imuEnableReports();
    Serial.println("BNO08x ready");
  } else {
    Serial.println("BNO08x not detected!");
  }
}

/* Drain pending sensor events, update heading / steps. */
static void imuService() {
  if (!imuOk) return;
  if (bno08x.wasReset()) {
    Serial.println("BNO08x reset - re-enabling reports");
    imuEnableReports();
  }
  sh2_SensorValue_t v;
  while (bno08x.getSensorEvent(&v)) {
    if (v.sensorId == SH2_ROTATION_VECTOR) {
      float qr = v.un.rotationVector.real;
      float qi = v.un.rotationVector.i;
      float qj = v.un.rotationVector.j;
      float qk = v.un.rotationVector.k;
      // Yaw about the up axis (ENU frame, CCW positive); compass heading is
      // clockwise from north, hence the sign flip.
      float yaw = atan2f(2.0f * (qr * qk + qi * qj),
                         1.0f - 2.0f * (qj * qj + qk * qk));
      float heading = wrap360(-yaw * RAD_TO_DEG
                              + HEADING_OFFSET_DEG + magDeclinationDeg);
      headingAcc = v.status & 0x03;
      // Low-pass on the unit vector so 359° -> 1° doesn't spin the long way
      float hr = heading * DEG_TO_RAD;
      smoothSin += SMOOTH_ALPHA * (sinf(hr) - smoothSin);
      smoothCos += SMOOTH_ALPHA * (cosf(hr) - smoothCos);
      headingDisp = wrap360(atan2f(smoothSin, smoothCos) * RAD_TO_DEG);
      haveHeading = true;
    } else if (v.sensorId == SH2_STEP_COUNTER) {
      uint16_t s = v.un.stepCounter.steps;
      if (!haveStepBase) { lastStepCount = s; haveStepBase = true; }
      uint16_t delta = (uint16_t)(s - lastStepCount);  // uint16 wrap-safe
      lastStepCount = s;
      if (state == ST_TRACKING) trackedSteps += delta;
    }
  }
}

/* ------------------------------------------------------------------ */
/*  GPS (BN-880)                                                       */
/* ------------------------------------------------------------------ */
/* Send a UBX frame (u-blox binary protocol) with checksum. */
static void ubxSend(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len) {
  uint8_t ckA = 0, ckB = 0;
  auto put = [&](uint8_t b) { Serial1.write(b); ckA += b; ckB += ckA; };
  Serial1.write(0xB5);
  Serial1.write(0x62);
  put(cls); put(id); put(len & 0xFF); put(len >> 8);
  for (uint16_t i = 0; i < len; i++) put(payload[i]);
  Serial1.write(ckA);
  Serial1.write(ckB);
}

/* Cut GPS power draw: drop the NMEA sentences we never parse (GSV is the
 * bulk of the traffic) and put the u-blox M8 into its "aggressive 1 Hz"
 * power-save mode (duty-cycled tracking, roughly halves receiver current). */
static void gpsConfigurePower() {
  Serial1.print("$PUBX,40,GLL,0,0,0,0,0,0*5C\r\n");
  Serial1.print("$PUBX,40,GSA,0,0,0,0,0,0*4E\r\n");
  Serial1.print("$PUBX,40,GSV,0,0,0,0,0,0*59\r\n");
  Serial1.print("$PUBX,40,VTG,0,0,0,0,0,0*5E\r\n");
  const uint8_t pms[8] = {0, 3, 0, 0, 0, 0, 0, 0};  // UBX-CFG-PMS: aggressive 1 Hz
  ubxSend(0x06, 0x86, pms, 8);
  Serial.println("GPS power-save configured");
}

/* Decimal year from the GPS date, e.g. 2026.63 (leap-day error is
 * negligible for declination). */
static float gpsDecimalYear() {
  static const uint16_t daysBefore[12] =
      {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
  int doy = daysBefore[gps.date.month() - 1] + gps.date.day();
  return gps.date.year() + (doy - 1) / 365.0f;
}

/* WMM declination for the current fix; refresh after moving far enough
 * for it to matter. */
static void updateDeclination(double lat, double lng) {
  if (!gps.date.isValid() || gps.date.year() < 2025) return;
  if (declFromGps &&
      TinyGPSPlus::distanceBetween(lat, lng, declLat, declLng) < DECL_RECOMPUTE_M)
    return;
  float altKm = gps.altitude.isValid() ? gps.altitude.meters() / 1000.0f : 0.0f;
  magDeclinationDeg = wmmDeclination(lat, lng, altKm, gpsDecimalYear());
  declLat = lat;
  declLng = lng;
  declFromGps = true;
  Serial.printf("Declination %.2f deg (WMM, %.4f %.4f)\n",
                magDeclinationDeg, lat, lng);
}

/* Feed NMEA to the parser, keep fix state and walk distance current. */
static void gpsService() {
  // one-shot config, delayed so the module is done booting
  if (!gpsCfgDone && millis() > 3000) {
    gpsConfigurePower();
    gpsCfgDone = true;
  }
  while (Serial1.available()) {
    gps.encode(Serial1.read());
    gpsLastDataMs = millis();
  }

  gpsFix = gps.location.isValid() && gps.location.age() < 3000;

  if (gps.location.isUpdated() && gps.location.isValid()) {
    double lat = gps.location.lat();
    double lng = gps.location.lng();
    gpsCurLat = lat;
    gpsCurLng = lng;
    updateDeclination(lat, lng);
    if (routeMode) navUpdate(lat, lng);
    if (state == ST_TRACKING && gpsHaveAnchor) {
      float d = (float)TinyGPSPlus::distanceBetween(gpsLastLat, gpsLastLng, lat, lng);
      if (d >= GPS_MIN_STEP_M) {
        gpsDistanceM += d;
        gpsDistUsed = true;
        gpsLastLat = lat;
        gpsLastLng = lng;
      }
    } else {
      // Idle: keep the anchor on the current position so a new walk
      // starts measuring from wherever START is pressed.
      gpsLastLat = lat;
      gpsLastLng = lng;
      gpsHaveAnchor = true;
    }
  }
}

/* ------------------------------------------------------------------ */
/*  Routes: BLE service pump + selection                               */
/* ------------------------------------------------------------------ */
static void showToast(const char *msg) {
  strncpy(toastMsg, msg, sizeof(toastMsg) - 1);
  toastMsg[sizeof(toastMsg) - 1] = 0;
  toastUntil = millis() + 2000;
}

/* Backlight: full while interacting or showing a toast, dim when idle. */
static void setBacklight(uint8_t v) {
  static uint8_t cur = 255;
  if (v != cur) {
    DEV_SET_PWM(v);
    cur = v;
  }
}

static bool blIsDim = false;   // touches on a dim screen only wake it

static void backlightService() {
  uint32_t now = millis();
  bool bright = (now - lastInteractMs < BL_DIM_AFTER_MS) || now < toastUntil;
  setBacklight(bright ? BL_ACTIVE : BL_DIM);
  blIsDim = !bright;
}

/* Rebuild the JSON served by the BLE INFO characteristic. */
static void refreshBleInfo() {
  static char js[320];
  int off = 0;
  off += sprintf(js + off, "[");
  for (uint8_t s = 0; s < ROUTE_SLOTS; s++) {
    RouteMeta m;
    routeLoadMeta(s, m);
    off += sprintf(js + off, "%s{\"slot\":%u,\"name\":\"%s\",\"points\":%u,\"m\":%d}",
                   s ? "," : "", s, m.count ? m.name : "", m.count, (int)m.lengthM);
  }
  sprintf(js + off, "]");
  routeBleSetInfo(js);
}

static bool selectRoute(uint8_t slot) {
  RouteMeta m;
  if (!routeBuf || !routeLoadPoints(slot, routeBuf, ROUTE_MAX_POINTS, m)) return false;
  navSetRoute(m, routeBuf);
  routeMode = true;
  activeSlot = slot;
  showToast(m.name[0] ? m.name : "ROUTE LOADED");
  Serial.printf("Route '%s' active: %u pts, %.0f m\n", m.name, m.count, m.lengthM);
  return true;
}

static void openMenu();   // defined with the UI below

/* Handle storage work queued by the BLE task, then ack the client. */
static void bleService() {
  RouteBleOp op = routeBleTakeOp();
  if (op == RBLE_NONE) return;

  uint8_t slot = routeBleOpSlot();
  if (op == RBLE_SAVE) {
    char name[ROUTE_NAME_LEN + 1];
    strncpy(name, routeBleOpName(), sizeof(name) - 1);
    name[sizeof(name) - 1] = 0;
    for (char *p = name; *p; p++)          // keep the INFO JSON valid
      if (*p == '"' || *p == '\\') *p = '_';
    bool ok = routeSave(slot, name, routeBleOpPoints(), routeBleOpCount());
    refreshBleInfo();
    routeBleAck(0x02, ok ? 0 : 1);
    showToast(ok ? "ROUTE RECEIVED" : "SAVE FAILED");
    if (ok && activeSlot == (int)slot) selectRoute(slot);  // live reload
    if (uiScreen == SCR_MENU) openMenu();                  // refresh the list
  } else if (op == RBLE_DELETE) {
    bool ok = routeDelete(slot);
    refreshBleInfo();
    routeBleAck(0x03, ok ? 0 : 1);
    if (ok && activeSlot == (int)slot) {
      navClear();
      routeMode = false;
      activeSlot = -1;
      showToast("ROUTE DELETED");
    }
    if (uiScreen == SCR_MENU) openMenu();                  // refresh the list
  }
}

/* ------------------------------------------------------------------ */
/*  UI                                                                 */
/* ------------------------------------------------------------------ */
static void drawCompassRing() {
  // Bezel
  Paint_DrawCircle(CX, CY, 118, COL_TICK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);

  // Rotating tick ring: bearing-up = heading
  for (int b = 0; b < 360; b += 15) {
    float a = wrap360((float)b - headingDisp);
    bool major = (b % 45) == 0;
    float r0 = major ? 105.0f : 110.0f;
    UWORD col = (b == 0) ? COL_NORTH : (major ? COL_TICK_MAJ : COL_TICK);
    clampedLine(polarX(a, r0), polarY(a, r0),
                polarX(a, 116), polarY(a, 116),
                col, major ? DOT_PIXEL_2X2 : DOT_PIXEL_1X1);
  }

  // Cardinal letters (positions rotate with heading, glyphs stay upright)
  for (int i = 0; i < 8; i++) {
    float bearing = i * 45.0f;
    float a = wrap360(bearing - headingDisp);
    bool main4 = (i % 2) == 0;
    sFONT *f = main4 ? &Font16 : &Font12;
    const char *name = CARDINAL16[i * 2];
    int x = polarX(a, 91) - (int)(strlen(name) * f->Width) / 2;
    int y = polarY(a, 91) - f->Height / 2;
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    UWORD col = (i == 0) ? COL_NORTH : (main4 ? COL_CARDINAL : COL_INTERCARD);
    Paint_DrawString_EN(x, y, name, f, col, COL_BG);
  }

  // Fixed lubber marker at 12 o'clock: "this is where you're heading"
  fillTriangle(112, 2, 128, 2, 120, 14, WHITE);

  // Target marker on the ring
  if (targetSet) {
    float a = wrap360(targetBearing - headingDisp);
    fillTriangle(polarX(a, 100), polarY(a, 100),
                 polarX(a - 6, 114), polarY(a - 6, 114),
                 polarX(a + 6, 114), polarY(a + 6, 114),
                 COL_TARGET);
  }
}

static void drawArrow() {
  float rel;          // where the arrow points, relative to screen-up
  UWORD fill;
  if (!imuOk || !haveHeading) {
    rel = 0.0f;
    fill = COL_DIM;
  } else if (routeMode) {
    if (navDone()) {
      rel = 0.0f;
      fill = COL_GOOD;
    } else if (!gpsFix) {
      rel = 0.0f;
      fill = COL_DIM;
    } else {
      rel = wrap360(navBearingDeg(gpsCurLat, gpsCurLng) - headingDisp);
      fill = navOffRoute() ? COL_WARN : COL_GOOD;
    }
  } else if (!targetSet) {
    // No target yet: act as a plain compass needle pointing north
    rel = wrap360(-headingDisp);
    fill = COL_DIM;
  } else {
    float dev = wrap180(targetBearing - headingDisp);
    rel = wrap360(dev);
    float ad = fabsf(dev);
    fill = (ad <= 12.0f) ? COL_GOOD : (ad <= 60.0f) ? COL_WARN : COL_BAD;
  }

  int tipX  = polarX(rel, 58),          tipY  = polarY(rel, 58);
  int lX    = polarX(rel + 142, 34),    lY    = polarY(rel + 142, 34);
  int rX    = polarX(rel - 142, 34),    rY    = polarY(rel - 142, 34);
  int tailX = polarX(rel + 180, 12),    tailY = polarY(rel + 180, 12);

  fillTriangle(tipX, tipY, lX, lY, tailX, tailY, fill);
  fillTriangle(tipX, tipY, tailX, tailY, rX, rY, fill);
  // Crisp outline
  clampedLine(tipX, tipY, lX, lY, WHITE, DOT_PIXEL_1X1);
  clampedLine(lX, lY, tailX, tailY, WHITE, DOT_PIXEL_1X1);
  clampedLine(tailX, tailY, rX, rY, WHITE, DOT_PIXEL_1X1);
  clampedLine(rX, rY, tipX, tipY, WHITE, DOT_PIXEL_1X1);
}

/* Walk timer as hours:minutes, e.g. "0:07", "1:23". */
static void formatTime(uint32_t ms, char *out) {
  uint32_t m = ms / 60000;
  sprintf(out, "%lu:%02lu", m / 60, m % 60);
}

static void formatDistance(float meters, char *out) {
  if (meters >= 1000.0f) sprintf(out, "%.2fkm", meters / 1000.0f);
  else                   sprintf(out, "%dm", (int)meters);
}

static void drawReadouts() {
  char buf[20];

  // Heading readout, top center
  if (imuOk && haveHeading) {
    sprintf(buf, "%03d", (int)headingDisp);
    drawStringCentered(CX - 8, 24, buf, &Font24, COL_TEXT, COL_BG);
    Paint_DrawCircle(CX + 26, 28, 3, COL_TEXT, DOT_PIXEL_1X1, DRAW_FILL_EMPTY); // ° sign
    drawStringCentered(CX, 52, cardinalName(headingDisp), &Font12, COL_DIM, COL_BG);
  } else {
    drawStringCentered(CX, 30, "NO IMU", &Font16, COL_BAD, COL_BG);
  }

  // Sensor accuracy dot (BNO08x status 0..3), top, left of the heading
  UWORD accCol = (headingAcc >= 3) ? COL_GOOD : (headingAcc == 2) ? COL_WARN : COL_BAD;
  Paint_DrawCircle(CX - 44, 34, 4, imuOk ? accCol : COL_BAD, DOT_PIXEL_1X1, DRAW_FILL_FULL);

  // GPS dot, right of the heading: green = fix, orange = NMEA but no
  // fix yet (sky search), red = no data at all (check wiring)
  bool gpsAlive = (millis() - gpsLastDataMs) < 3000 && gpsLastDataMs != 0;
  UWORD gpsCol = gpsFix ? COL_GOOD : gpsAlive ? COL_WARN : COL_BAD;
  Paint_DrawCircle(CX + 44, 34, 4, gpsCol, DOT_PIXEL_1X1, DRAW_FILL_FULL);
  sprintf(buf, "%d", (int)gps.satellites.value());
  Paint_DrawString_EN(CX + 52, 30, buf, &Font8, COL_DIM, COL_BG);

  // Status line under the arrow
  if (millis() < toastUntil) {
    drawStringCentered(CX, 152, toastMsg, &Font12, COL_TARGET, COL_BG);
  } else if (routeMode) {
    if (navDone()) {
      drawStringCentered(CX, 152, "ROUTE DONE", &Font12, COL_GOOD, COL_BG);
    } else if (!gpsFix) {
      drawStringCentered(CX, 152, "ROUTE: NO GPS", &Font12, COL_WARN, COL_BG);
    } else {
      char db[12];
      formatDistance(navDistToTargetM(gpsCurLat, gpsCurLng), db);
      sprintf(buf, "%s CP %u/%u %s", navOffRoute() ? "OFF!" : "",
              navTargetIdx() + 1, navCount(), db);
      drawStringCentered(CX, 152, buf, &Font12,
                         navOffRoute() ? COL_WARN : COL_TARGET, COL_BG);
    }
  } else if (millis() < lockFlashUntil) {
    drawStringCentered(CX, 152, "TARGET SET", &Font12, COL_TARGET, COL_BG);
  } else if (targetSet) {
    sprintf(buf, "TGT %03d %s", (int)targetBearing, cardinalName(targetBearing));
    drawStringCentered(CX, 152, buf, &Font12, COL_TARGET, COL_BG);
  } else {
    // alternate the idle hint so the menu gesture is discoverable
    drawStringCentered(CX, 152,
                       ((millis() / 3000) & 1) ? "HOLD FOR ROUTES" : "TAP DIAL TO SET",
                       &Font12, COL_DIM, COL_BG);
  }

  // Info row: elapsed time | estimated distance (steps until GPS)
  uint32_t elapsed = (state == ST_TRACKING) ? (millis() - startMs) : finalElapsedMs;
  drawStringCentered(72, 166, "TIME", &Font8, COL_DIM, COL_BG);
  formatTime(elapsed, buf);
  drawStringCentered(72, 175, buf, &Font16, COL_TEXT, COL_BG);

  // Route mode: distance left on the route. Otherwise distance walked:
  // GPS when the walk has coverage, step estimate if not.
  if (routeMode && gpsFix && !navDone()) {
    drawStringCentered(168, 166, "LEFT", &Font8, COL_DIM, COL_BG);
    formatDistance(navRemainingM(gpsCurLat, gpsCurLng), buf);
  } else {
    drawStringCentered(168, 166, gpsDistUsed ? "DIST GPS" : "DIST EST", &Font8, COL_DIM, COL_BG);
    formatDistance(gpsDistUsed ? gpsDistanceM : trackedSteps * STRIDE_M, buf);
  }
  drawStringCentered(168, 175, buf, &Font16, COL_TEXT, COL_BG);

  // BLE link dot, bottom of the dial
  if (routeBleConnected())
    Paint_DrawCircle(CX, 228, 3, BLUE, DOT_PIXEL_1X1, DRAW_FILL_FULL);

  // START / STOP button
  bool tracking = (state == ST_TRACKING);
  drawPill(BTN_X0, BTN_Y0, BTN_X1, BTN_Y1, tracking ? COL_BTN_STOP : COL_BTN_GO);
  drawStringCentered((BTN_X0 + BTN_X1) / 2, BTN_Y0 + 7,
                     tracking ? "STOP" : "START", &Font16, WHITE,
                     tracking ? COL_BTN_STOP : COL_BTN_GO);
}

/* Route menu: 4 slot rows + compass-only row. Opened by long press. */
static const int MENU_ROW_Y0 = 44, MENU_ROW_H = 34;
static const int MENU_COMPASS_Y0 = 184, MENU_COMPASS_Y1 = 214;

static void openMenu() {
  for (uint8_t s = 0; s < ROUTE_SLOTS; s++) routeLoadMeta(s, menuMeta[s]);
  uiScreen = SCR_MENU;
  menuDirty = true;
}

static void drawMenu() {
  char buf[28];
  Paint_Clear(COL_BG);
  drawStringCentered(CX, 16, "ROUTES", &Font16, COL_TEXT, COL_BG);

  for (int s = 0; s < ROUTE_SLOTS; s++) {
    int y = MENU_ROW_Y0 + s * MENU_ROW_H;
    UWORD frame = (s == activeSlot && routeMode) ? COL_TARGET : COL_TICK;
    Paint_DrawRectangle(28, y, 212, y + MENU_ROW_H - 6, frame,
                        DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    if (menuMeta[s].count) {
      sprintf(buf, "%.14s", menuMeta[s].name[0] ? menuMeta[s].name : "(no name)");
      Paint_DrawString_EN(36, y + 4, buf, &Font12, COL_TEXT, COL_BG);
      if (menuMeta[s].lengthM >= 1000)
        sprintf(buf, "%.1fkm", menuMeta[s].lengthM / 1000.0f);
      else
        sprintf(buf, "%dm", (int)menuMeta[s].lengthM);
      Paint_DrawString_EN(148, y + 16, buf, &Font12, COL_DIM, COL_BG);
    } else {
      Paint_DrawString_EN(36, y + 8, "- empty -", &Font12, COL_DIM, COL_BG);
    }
  }

  UWORD cc = routeMode ? COL_TICK : COL_TARGET;
  Paint_DrawRectangle(60, MENU_COMPASS_Y0, 180, MENU_COMPASS_Y1, cc,
                      DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
  drawStringCentered(CX, MENU_COMPASS_Y0 + 10, "COMPASS", &Font12,
                     routeMode ? COL_TEXT : COL_TARGET, COL_BG);
}

static void menuTap(int x, int y) {
  if (y >= MENU_COMPASS_Y0 - 6 && y <= MENU_COMPASS_Y1 + 6) {
    navClear();
    routeMode = false;
    activeSlot = -1;
    uiScreen = SCR_MAIN;
    showToast("COMPASS MODE");
    return;
  }
  if (x < 20 || x > 220) return;
  int s = (y - MENU_ROW_Y0) / MENU_ROW_H;
  if (s < 0 || s >= ROUTE_SLOTS || y < MENU_ROW_Y0) return;
  if (!menuMeta[s].count) { showToast("EMPTY SLOT"); return; }
  if (selectRoute(s)) uiScreen = SCR_MAIN;
  else showToast("LOAD FAILED");
}

static void drawFrame() {
  if (uiScreen == SCR_MENU) {
    drawMenu();
  } else {
    Paint_Clear(COL_BG);
    drawCompassRing();
    drawArrow();
    drawReadouts();
  }
  LCD_1IN28_Display(BlackImage);
}

/* ------------------------------------------------------------------ */
/*  Touch                                                              */
/* ------------------------------------------------------------------ */
/* The CST816S's own LONG_PRESS gesture needs a ~2 s perfectly-still hold
 * and frequently never fires, so tap-vs-hold is decided here instead:
 * a press that lasts HOLD_MS without moving opens the menu, anything
 * shorter is a tap dispatched on release. Release is detected from the
 * chip's finger-count register, so a missed "up" event can't wedge the
 * state machine. Chip swipe gestures (which are reliable) also open it. */
static const uint32_t HOLD_MS = 500;
static bool     pressActive = false;
static bool     pressMoved = false;      // drifted >30 px: not a tap/hold
static bool     pressConsumed = false;   // action already fired this press
static uint32_t pressStartMs = 0, pressPollMs = 0;
static int      pressX = 0, pressY = 0;

static int touchFingerCount() {
  Wire.beginTransmission(CST816S_ADDRESS);
  Wire.write(0x02);
  if (Wire.endTransmission(true)) return -1;
  if (Wire.requestFrom(CST816S_ADDRESS, 1) != 1) return -1;
  return Wire.read();
}

static void toggleMenu() {
  if (uiScreen == SCR_MAIN) openMenu();
  else uiScreen = SCR_MAIN;
}

static void dispatchTap(int x, int y) {
  uint32_t now = millis();

  if (uiScreen == SCR_MENU) {
    menuTap(x, y);
    menuDirty = true;
    return;
  }

  // START / STOP pill (with a little slop around it)
  if (x >= BTN_X0 - 10 && x <= BTN_X1 + 10 && y >= BTN_Y0 - 10 && y <= 239) {
    lastTapMs = now;
    if (state == ST_IDLE) {
      state = ST_TRACKING;
      startMs = now;
      finalElapsedMs = 0;
      trackedSteps = 0;
      gpsDistanceM = 0.0f;
      gpsDistUsed = false;
      Serial.println("Walk started");
    } else {
      state = ST_IDLE;
      finalElapsedMs = now - startMs;
      Serial.println("Walk stopped");
    }
    return;
  }

  // Center of the dial: lock current heading as the target bearing
  // (compass mode only; in route mode the route drives the arrow)
  int dx = x - CX, dy = y - CY;
  if (routeMode) {
    if (dx * dx + dy * dy < 60 * 60) showToast("HOLD FOR MENU");
    return;
  }
  if (dx * dx + dy * dy < 60 * 60 && imuOk && haveHeading) {
    lastTapMs = now;
    targetBearing = headingDisp;
    targetSet = true;
    lockFlashUntil = now + 1200;
    Serial.printf("Target locked: %d deg\n", (int)targetBearing);
  }
}

static void handleTouch() {
  uint32_t now = millis();

  if (touch.available()) {
    // A touch on a dimmed screen only wakes it — swallow the whole press
    // so waking never locks a target or hits START by accident.
    bool wakeOnly = blIsDim && !pressActive;
    lastInteractMs = now;              // wake backlight / boost frame rate
    uint8_t g = touch.data.gestureID;
    uint8_t ev = touch.data.event;     // 0 down, 1 up, 2 contact
    int x = touch.data.x, y = touch.data.y;

    // Chip swipes are dependable (unlike its long press): menu shortcut.
    // The chip repeats the gesture id in every report while the finger
    // stays down, so fire at most once per press or the menu toggles
    // over and over (visible as flashing).
    if (g == SWIPE_UP || g == SWIPE_DOWN || g == LONG_PRESS) {
      bool fire = !wakeOnly && !(pressActive && pressConsumed) &&
                  now - lastTapMs >= 350;
      if (fire) {
        lastTapMs = now + 200;
        toggleMenu();
      }
      if (ev == 1) {                   // gesture ended with this report
        pressActive = false;
      } else {                         // finger still down: swallow repeats
        if (!pressActive) pressStartMs = now;
        pressActive = true;
      }
      pressConsumed = true;
      pressMoved = true;               // never also a tap/hold
      return;
    }

    if (ev == 1) {                     // release
      if (pressActive && !pressMoved && !pressConsumed &&
          now - pressStartMs < HOLD_MS && now - lastTapMs >= 350) {
        lastTapMs = now;
        dispatchTap(pressX, pressY);
      } else if (!pressActive && !wakeOnly && now - lastTapMs >= 350) {
        lastTapMs = now;               // chip condensed the press to one event
        dispatchTap(x, y);
      }
      pressActive = false;
      pressMoved = false;
      pressConsumed = false;
      return;
    }

    // press start / finger still down
    if (!pressActive) {
      pressActive = true;
      pressMoved = false;
      pressConsumed = wakeOnly;        // wake press: never tap, never hold
      pressStartMs = now;
      pressX = x;
      pressY = y;
    } else if (abs(x - pressX) > 30 || abs(y - pressY) > 30) {
      pressMoved = true;               // a drag, not a tap or hold
    }
    return;
  }

  // No new event: watch a live press for hold-expiry or a missed release
  if (!pressActive || now - pressPollMs < 60) return;
  pressPollMs = now;
  int fingers = touchFingerCount();
  if (fingers == 0) {                  // released without an "up" event
    if (!pressMoved && !pressConsumed && now - pressStartMs < HOLD_MS &&
        now - lastTapMs >= 350) {
      lastTapMs = now;
      dispatchTap(pressX, pressY);
    }
    pressActive = false;
    pressMoved = false;
    pressConsumed = false;
  } else if (fingers > 0 && !pressMoved && !pressConsumed &&
             now - pressStartMs >= HOLD_MS) {
    pressConsumed = true;              // one action per press
    lastTapMs = now + 200;
    toggleMenu();
  }
}

/* ------------------------------------------------------------------ */
/*  Arduino entry points                                               */
/* ------------------------------------------------------------------ */
void setup() {
  setCpuFrequencyMhz(CPU_MHZ);   // 80 MHz: ~half the core power of 240 MHz
  Serial.begin(115200);
  Serial1.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  delay(300);

  touch.begin();

  if (psramInit()) Serial.println("PSRAM OK");
  else             Serial.println("PSRAM not available!");

  UDOUBLE imageSize = (UDOUBLE)LCD_1IN28_HEIGHT * LCD_1IN28_WIDTH * 2;
  BlackImage = (UWORD *)ps_malloc(imageSize);
  if (BlackImage == NULL) {
    Serial.println("Framebuffer allocation failed");
    while (true) delay(1000);
  }

  if (DEV_Module_Init() != 0) Serial.println("GPIO init failed");
  LCD_1IN28_Init(HORIZONTAL);
  DEV_SET_PWM(90);   // backlight

  Paint_NewImage((UBYTE *)BlackImage, LCD_1IN28.WIDTH, LCD_1IN28.HEIGHT, 0, BLACK);
  Paint_SetScale(65);
  Paint_SetRotate(ROTATE_0);

  // Splash while the IMU comes up
  Paint_Clear(COL_BG);
  drawStringCentered(CX, 96, "DON'T GET LOST", &Font16, COL_TEXT, COL_BG);
  drawStringCentered(CX, 126, "finding north...", &Font12, COL_DIM, COL_BG);
  LCD_1IN28_Display(BlackImage);

  imuInit();

  routeBuf = (RoutePoint *)ps_malloc(ROUTE_MAX_POINTS * sizeof(RoutePoint));
  routeStoreInit();
  routeBleInit();
  refreshBleInfo();
}

void loop() {
  imuService();
  gpsService();
  bleService();
  handleTouch();
  backlightService();

  uint32_t now = millis();
  if (uiScreen == SCR_MENU) {
    // menu is static: redraw only when its content changed
    if (menuDirty && now - lastFrameMs >= FRAME_MS) {
      menuDirty = false;
      lastFrameMs = now;
      drawFrame();
    }
  } else {
    // full rate while something moves on screen, 2 Hz when the watch is
    // still — each frame is a 115 KB SPI transfer, so idle frames are the
    // single biggest CPU/bus cost
    bool uiActive = (state == ST_TRACKING) || now < toastUntil ||
                    (now - lastInteractMs < 3000) ||
                    fabsf(wrap180(headingDisp - lastDrawnHdg)) > ACTIVE_HDG_DEG;
    if (now - lastFrameMs >= (uiActive ? FRAME_MS : FRAME_IDLE_MS)) {
      lastFrameMs = now;
      lastDrawnHdg = headingDisp;
      drawFrame();
    }
  }

  delay(2);   // let the idle task run (also required for the task watchdog)
}
