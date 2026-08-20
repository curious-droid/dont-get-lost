#include "route_store.h"
#include <LittleFS.h>
#include <math.h>

/* File layout (little-endian):
 *   u32 magic 'DGL1' | u8 nameLen | name bytes | u16 count | f32 lengthM |
 *   count * (i32 lat, i32 lng)
 */
static const uint32_t MAGIC = 0x314C4744;  // "DGL1"

static void slotPath(uint8_t slot, char *out) {
  sprintf(out, "/route%u.bin", slot);
}

float routePointDistM(const RoutePoint &a, const RoutePoint &b) {
  double la1 = a.lat * 1e-7 * DEG_TO_RAD, lo1 = a.lng * 1e-7 * DEG_TO_RAD;
  double la2 = b.lat * 1e-7 * DEG_TO_RAD, lo2 = b.lng * 1e-7 * DEG_TO_RAD;
  double sdla = sin((la2 - la1) / 2), sdlo = sin((lo2 - lo1) / 2);
  double h = sdla * sdla + cos(la1) * cos(la2) * sdlo * sdlo;
  return (float)(2.0 * 6371000.0 * asin(sqrt(h)));
}

bool routeStoreInit() {
  if (!LittleFS.begin(true)) {   // format on first boot
    Serial.println("LittleFS mount failed");
    return false;
  }
  return true;
}

bool routeSave(uint8_t slot, const char *name, const RoutePoint *pts, uint16_t count) {
  if (slot >= ROUTE_SLOTS || count < 2 || count > ROUTE_MAX_POINTS) return false;

  float len = 0;
  for (uint16_t i = 1; i < count; i++) len += routePointDistM(pts[i - 1], pts[i]);

  char path[20];
  slotPath(slot, path);
  File f = LittleFS.open(path, "w");
  if (!f) return false;

  uint8_t nameLen = strnlen(name, ROUTE_NAME_LEN);
  bool ok = f.write((uint8_t *)&MAGIC, 4) == 4
         && f.write(&nameLen, 1) == 1
         && f.write((const uint8_t *)name, nameLen) == nameLen
         && f.write((uint8_t *)&count, 2) == 2
         && f.write((uint8_t *)&len, 4) == 4
         && f.write((const uint8_t *)pts, (size_t)count * sizeof(RoutePoint))
              == (size_t)count * sizeof(RoutePoint);
  f.close();
  if (!ok) LittleFS.remove(path);
  return ok;
}

static bool readHeader(File &f, RouteMeta &meta) {
  uint32_t magic = 0;
  uint8_t nameLen = 0;
  if (f.read((uint8_t *)&magic, 4) != 4 || magic != MAGIC) return false;
  if (f.read(&nameLen, 1) != 1 || nameLen > ROUTE_NAME_LEN) return false;
  memset(meta.name, 0, sizeof(meta.name));
  if (f.read((uint8_t *)meta.name, nameLen) != nameLen) return false;
  if (f.read((uint8_t *)&meta.count, 2) != 2) return false;
  if (f.read((uint8_t *)&meta.lengthM, 4) != 4) return false;
  return meta.count >= 2 && meta.count <= ROUTE_MAX_POINTS;
}

bool routeLoadMeta(uint8_t slot, RouteMeta &meta) {
  meta.count = 0;
  meta.name[0] = 0;
  meta.lengthM = 0;
  if (slot >= ROUTE_SLOTS) return false;
  char path[20];
  slotPath(slot, path);
  if (!LittleFS.exists(path)) return false;  // empty slot: avoid VFS error spam
  File f = LittleFS.open(path, "r");
  if (!f) return false;
  bool ok = readHeader(f, meta);
  f.close();
  if (!ok) meta.count = 0;
  return ok;
}

bool routeLoadPoints(uint8_t slot, RoutePoint *buf, uint16_t maxCount, RouteMeta &meta) {
  if (slot >= ROUTE_SLOTS) return false;
  char path[20];
  slotPath(slot, path);
  if (!LittleFS.exists(path)) return false;
  File f = LittleFS.open(path, "r");
  if (!f) return false;
  bool ok = readHeader(f, meta) && meta.count <= maxCount
         && f.read((uint8_t *)buf, (size_t)meta.count * sizeof(RoutePoint))
              == (size_t)meta.count * sizeof(RoutePoint);
  f.close();
  if (!ok) meta.count = 0;
  return ok;
}

bool routeDelete(uint8_t slot) {
  if (slot >= ROUTE_SLOTS) return false;
  char path[20];
  slotPath(slot, path);
  if (!LittleFS.exists(path)) return true;   // already gone
  return LittleFS.remove(path);
}
