/*****************************************************************************
 * route_store.h — persistent route storage (LittleFS)
 *
 * Up to ROUTE_SLOTS routes, each a named polyline of lat/lng checkpoints
 * (int32, degrees * 1e7 — ~1 cm resolution). Routes survive reboot.
 *****************************************************************************/
#ifndef _ROUTE_STORE_H_
#define _ROUTE_STORE_H_

#include <Arduino.h>

#define ROUTE_SLOTS      4
#define ROUTE_MAX_POINTS 600
#define ROUTE_NAME_LEN   16

struct RoutePoint {
  int32_t lat;   // degrees * 1e7
  int32_t lng;
};

struct RouteMeta {
  char     name[ROUTE_NAME_LEN + 1];
  uint16_t count;     // 0 = slot empty
  float    lengthM;
};

bool routeStoreInit();
bool routeSave(uint8_t slot, const char *name, const RoutePoint *pts, uint16_t count);
bool routeLoadMeta(uint8_t slot, RouteMeta &meta);          // header only
bool routeLoadPoints(uint8_t slot, RoutePoint *buf, uint16_t maxCount, RouteMeta &meta);
bool routeDelete(uint8_t slot);

/* Haversine distance between two stored points, meters. */
float routePointDistM(const RoutePoint &a, const RoutePoint &b);

#endif
