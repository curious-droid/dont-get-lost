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
static const uint32_t FRAME_MS        = 50;     // ~20 fps
static const float SMOOTH_ALPHA       = 0.30f;  // heading low-pass strength

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
  // Fused orientation (gyro + accel + magnetometer), 50 Hz
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR, 20000))
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
  while (Serial1.available()) {
    gps.encode(Serial1.read());
    gpsLastDataMs = millis();
  }

  gpsFix = gps.location.isValid() && gps.location.age() < 3000;

  if (gps.location.isUpdated() && gps.location.isValid()) {
    double lat = gps.location.lat();
    double lng = gps.location.lng();
    updateDeclination(lat, lng);
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

static void formatTime(uint32_t ms, char *out) {
  uint32_t s = ms / 1000;
  if (s >= 3600) sprintf(out, "%lu:%02lu:%02lu", s / 3600, (s / 60) % 60, s % 60);
  else           sprintf(out, "%02lu:%02lu", s / 60, s % 60);
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

  // Target line under the arrow
  if (millis() < lockFlashUntil) {
    drawStringCentered(CX, 152, "TARGET SET", &Font12, COL_TARGET, COL_BG);
  } else if (targetSet) {
    sprintf(buf, "TGT %03d %s", (int)targetBearing, cardinalName(targetBearing));
    drawStringCentered(CX, 152, buf, &Font12, COL_TARGET, COL_BG);
  } else {
    drawStringCentered(CX, 152, "TAP DIAL TO SET", &Font12, COL_DIM, COL_BG);
  }

  // Info row: elapsed time | estimated distance (steps until GPS)
  uint32_t elapsed = (state == ST_TRACKING) ? (millis() - startMs) : finalElapsedMs;
  drawStringCentered(72, 166, "TIME", &Font8, COL_DIM, COL_BG);
  formatTime(elapsed, buf);
  drawStringCentered(72, 175, buf, &Font16, COL_TEXT, COL_BG);

  // GPS distance when the walk has GPS coverage, step estimate otherwise
  drawStringCentered(168, 166, gpsDistUsed ? "DIST GPS" : "DIST EST", &Font8, COL_DIM, COL_BG);
  formatDistance(gpsDistUsed ? gpsDistanceM : trackedSteps * STRIDE_M, buf);
  drawStringCentered(168, 175, buf, &Font16, COL_TEXT, COL_BG);

  // START / STOP button
  bool tracking = (state == ST_TRACKING);
  drawPill(BTN_X0, BTN_Y0, BTN_X1, BTN_Y1, tracking ? COL_BTN_STOP : COL_BTN_GO);
  drawStringCentered((BTN_X0 + BTN_X1) / 2, BTN_Y0 + 7,
                     tracking ? "STOP" : "START", &Font16, WHITE,
                     tracking ? COL_BTN_STOP : COL_BTN_GO);
}

static void drawFrame() {
  Paint_Clear(COL_BG);
  drawCompassRing();
  drawArrow();
  drawReadouts();
  LCD_1IN28_Display(BlackImage);
}

/* ------------------------------------------------------------------ */
/*  Touch                                                              */
/* ------------------------------------------------------------------ */
static void handleTouch() {
  if (!touch.available()) return;
  uint32_t now = millis();
  if (now - lastTapMs < 350) return;   // debounce
  int x = touch.data.x;
  int y = touch.data.y;

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
  int dx = x - CX, dy = y - CY;
  if (dx * dx + dy * dy < 60 * 60 && imuOk && haveHeading) {
    lastTapMs = now;
    targetBearing = headingDisp;
    targetSet = true;
    lockFlashUntil = now + 1200;
    Serial.printf("Target locked: %d deg\n", (int)targetBearing);
  }
}

/* ------------------------------------------------------------------ */
/*  Arduino entry points                                               */
/* ------------------------------------------------------------------ */
void setup() {
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
}

void loop() {
  imuService();
  gpsService();
  handleTouch();

  uint32_t now = millis();
  if (now - lastFrameMs >= FRAME_MS) {
    lastFrameMs = now;
    drawFrame();
  }
}
