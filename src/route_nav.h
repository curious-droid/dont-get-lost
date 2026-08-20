/*****************************************************************************
 * route_nav.h — route follower: monotonic progress + bounded lookahead
 *
 * The route is a sequence of checkpoints. Progress only moves forward, and
 * the GPS position is only ever matched against checkpoints within a
 * bounded window ahead of current progress. That makes following robust on
 * self-intersecting routes and multi-lap loops (a later lap's identical
 * coordinates are outside the window, so no accidental jump), while still
 * allowing genuine shortcuts up to the window length: skipping ahead
 * requires being inside the capture radius of a specific later checkpoint
 * for several consecutive fixes.
 *
 * Target index 0 means "go to the start of the route".
 *****************************************************************************/
#ifndef _ROUTE_NAV_H_
#define _ROUTE_NAV_H_

#include "route_store.h"

void navSetRoute(const RouteMeta &meta, const RoutePoint *pts);
void navClear();
bool navActive();
bool navDone();
bool navOffRoute();

/* Call on every (valid, updated) GPS fix. */
void navUpdate(double latDeg, double lngDeg);

/* Bearing (deg, true) and distance (m) from a position to the current
 * target checkpoint. Only valid while navActive() && !navDone(). */
float navBearingDeg(double latDeg, double lngDeg);
float navDistToTargetM(double latDeg, double lngDeg);

uint16_t navTargetIdx();   // current target checkpoint
uint16_t navCount();       // total checkpoints
float navRemainingM(double latDeg, double lngDeg);
const char *navName();

#endif
