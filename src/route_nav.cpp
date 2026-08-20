#include "route_nav.h"
#include <TinyGPSPlus.h>

/* Tuning */
static const float CAPTURE_M     = 25.0f;   // checkpoint reached inside this
static const float OFFROUTE_M    = 60.0f;   // farther than this from any
                                            // window checkpoint = off route
static const float LOOKAHEAD_MIN = 120.0f;  // window bounds, meters of
static const float LOOKAHEAD_MAX = 500.0f;  // route ahead of progress
static const int   SKIP_FIXES    = 3;       // consecutive fixes to shortcut

static RoutePoint pts[ROUTE_MAX_POINTS];
static float cum[ROUTE_MAX_POINTS];         // arc length to each point
static RouteMeta meta;
static bool active = false;

static uint16_t target = 0;                 // next checkpoint to reach
static bool done = false;
static bool offRoute = false;
static float lookaheadM = LOOKAHEAD_MAX;
static int skipCand = -1;
static int skipStreak = 0;

static inline double plat(uint16_t i) { return pts[i].lat * 1e-7; }
static inline double plng(uint16_t i) { return pts[i].lng * 1e-7; }

static float distToIdx(double lat, double lng, uint16_t i) {
  return (float)TinyGPSPlus::distanceBetween(lat, lng, plat(i), plng(i));
}

void navSetRoute(const RouteMeta &m, const RoutePoint *p) {
  meta = m;
  memcpy(pts, p, (size_t)m.count * sizeof(RoutePoint));
  cum[0] = 0;
  for (uint16_t i = 1; i < m.count; i++)
    cum[i] = cum[i - 1] + routePointDistM(pts[i - 1], pts[i]);
  lookaheadM = constrain(0.25f * cum[m.count - 1], LOOKAHEAD_MIN, LOOKAHEAD_MAX);
  target = 0;
  done = false;
  offRoute = false;
  skipCand = -1;
  skipStreak = 0;
  active = true;
}

void navClear() { active = false; done = false; }
bool navActive() { return active; }
bool navDone() { return done; }
bool navOffRoute() { return active && !done && offRoute; }
uint16_t navTargetIdx() { return target; }
uint16_t navCount() { return meta.count; }
const char *navName() { return meta.name; }

/* Mark checkpoint `idx` reached and aim at the next one; swallow any
 * immediately-following checkpoints we are already inside. */
static void reach(uint16_t idx, double lat, double lng) {
  target = idx + 1;
  while (target < meta.count && distToIdx(lat, lng, target) < CAPTURE_M)
    target++;
  skipCand = -1;
  skipStreak = 0;
  if (target >= meta.count) done = true;
}

void navUpdate(double lat, double lng) {
  if (!active || done) return;

  float dTarget = distToIdx(lat, lng, target);
  if (dTarget < CAPTURE_M) {
    reach(target, lat, lng);
    offRoute = false;
    return;
  }

  /* Shortcut detection: scan the window ahead of the target for a
   * checkpoint we are inside. Window is measured in route arc length from
   * current progress, so a second lap of a loop stays out of reach. */
  float base = (target > 0) ? cum[target - 1] : 0;
  int best = -1;
  float bestD = CAPTURE_M, minD = dTarget;
  for (uint16_t j = target + 1;
       j < meta.count && cum[j] - base <= lookaheadM; j++) {
    float d = distToIdx(lat, lng, j);
    if (d < minD) minD = d;
    if (d < bestD) { bestD = d; best = j; }
  }

  if (best >= 0) {
    if (best == skipCand) skipStreak++;
    else { skipCand = best; skipStreak = 1; }
    if (skipStreak >= SKIP_FIXES) {
      Serial.printf("Route: shortcut, skipping to checkpoint %d\n", best);
      reach((uint16_t)best, lat, lng);
      offRoute = false;
      return;
    }
  } else {
    skipCand = -1;
    skipStreak = 0;
  }

  offRoute = minD > OFFROUTE_M;
}

float navBearingDeg(double lat, double lng) {
  return (float)TinyGPSPlus::courseTo(lat, lng, plat(target), plng(target));
}

float navDistToTargetM(double lat, double lng) {
  return distToIdx(lat, lng, target);
}

float navRemainingM(double lat, double lng) {
  if (!active) return 0;
  if (done) return 0;
  return (cum[meta.count - 1] - cum[target]) + distToIdx(lat, lng, target);
}
