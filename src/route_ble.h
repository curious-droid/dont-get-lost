/*****************************************************************************
 * route_ble.h — BLE GATT service for route upload/management
 *
 * Custom service, designed to be driven by a Web Bluetooth page:
 *   CTRL (write + notify) — commands, acked by a [opcode, status] notify:
 *       0x01 BEGIN  [op, slot, count_lo, count_hi, nameLen, name...]
 *       0x02 END    [op]                 -> saves the uploaded route
 *       0x03 DELETE [op, slot]
 *   DATA (write / write-no-rsp) — raw route bytes between BEGIN and END:
 *       count * (i32 lat*1e7, i32 lng*1e7), little-endian, any chunking
 *   INFO (read) — JSON array of stored routes
 *
 * Threading: NimBLE callbacks only fill buffers and set a pending op; the
 * main loop must poll routeBleTakeOp() and do the filesystem work, then
 * call routeBleAck() so the web page sees the result.
 *****************************************************************************/
#ifndef _ROUTE_BLE_H_
#define _ROUTE_BLE_H_

#include "route_store.h"

enum RouteBleOp : uint8_t {
  RBLE_NONE   = 0,
  RBLE_SAVE   = 1,   // upload finished, ready to persist
  RBLE_DELETE = 2,
};

void routeBleInit();

/* Returns the pending operation (and clears it). For RBLE_SAVE the upload
 * data is valid until the next BEGIN arrives. */
RouteBleOp routeBleTakeOp();
uint8_t routeBleOpSlot();
const char *routeBleOpName();
uint16_t routeBleOpCount();
const RoutePoint *routeBleOpPoints();

/* Notify the client of a command result (0 = ok). */
void routeBleAck(uint8_t opcode, uint8_t status);

/* Refresh the JSON served by the INFO characteristic. */
void routeBleSetInfo(const char *json);

bool routeBleConnected();

#endif
