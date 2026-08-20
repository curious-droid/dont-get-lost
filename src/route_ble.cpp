#include "route_ble.h"
#include <NimBLEDevice.h>

static const char *SVC_UUID  = "f3641400-00b0-4240-ba50-05ca45bf8abc";
static const char *CTRL_UUID = "f3641401-00b0-4240-ba50-05ca45bf8abc";
static const char *DATA_UUID = "f3641402-00b0-4240-ba50-05ca45bf8abc";
static const char *INFO_UUID = "f3641403-00b0-4240-ba50-05ca45bf8abc";

static NimBLECharacteristic *ctrlChr = nullptr;
static NimBLECharacteristic *infoChr = nullptr;

static volatile bool connected = false;

/* Upload in progress (filled on the NimBLE task) */
static RoutePoint *rxPts = nullptr;
static volatile bool rxActive = false;
static volatile uint32_t rxBytes = 0;
static uint16_t rxCount = 0;
static uint8_t rxSlot = 0;
static char rxName[ROUTE_NAME_LEN + 1];

/* Handoff to the main loop */
static volatile RouteBleOp pendingOp = RBLE_NONE;
static volatile uint8_t pendingSlot = 0;

class SrvCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *, NimBLEConnInfo &) override {
    connected = true;
    Serial.println("BLE connected");
  }
  void onDisconnect(NimBLEServer *, NimBLEConnInfo &, int) override {
    connected = false;
    rxActive = false;
    Serial.println("BLE disconnected");
    NimBLEDevice::startAdvertising();
  }
};

class CtrlCB : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *c, NimBLEConnInfo &) override {
    NimBLEAttValue v = c->getValue();
    const uint8_t *d = v.data();
    size_t n = v.size();
    if (n < 1) return;

    switch (d[0]) {
      case 0x01: {  // BEGIN: slot, count16, nameLen, name
        if (n < 5) { routeBleAck(0x01, 2); return; }
        uint8_t slot = d[1];
        uint16_t count = d[2] | (d[3] << 8);
        uint8_t nameLen = d[4];
        if (slot >= ROUTE_SLOTS || count < 2 || count > ROUTE_MAX_POINTS ||
            nameLen > ROUTE_NAME_LEN || n < (size_t)5 + nameLen) {
          routeBleAck(0x01, 2);
          return;
        }
        rxSlot = slot;
        rxCount = count;
        memset(rxName, 0, sizeof(rxName));
        memcpy(rxName, d + 5, nameLen);
        rxBytes = 0;
        rxActive = true;
        routeBleAck(0x01, 0);
        break;
      }
      case 0x02: {  // END
        if (!rxActive || rxBytes != (uint32_t)rxCount * sizeof(RoutePoint)) {
          rxActive = false;
          routeBleAck(0x02, 3);
          return;
        }
        rxActive = false;
        pendingSlot = rxSlot;
        pendingOp = RBLE_SAVE;   // main loop saves + acks
        break;
      }
      case 0x03: {  // DELETE
        if (n < 2 || d[1] >= ROUTE_SLOTS) { routeBleAck(0x03, 2); return; }
        pendingSlot = d[1];
        pendingOp = RBLE_DELETE;  // main loop deletes + acks
        break;
      }
    }
  }
};

class DataCB : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *c, NimBLEConnInfo &) override {
    if (!rxActive) return;
    NimBLEAttValue v = c->getValue();
    uint32_t cap = (uint32_t)rxCount * sizeof(RoutePoint);
    if (rxBytes + v.size() > cap) { rxActive = false; return; }
    memcpy((uint8_t *)rxPts + rxBytes, v.data(), v.size());
    rxBytes += v.size();
  }
};

void routeBleInit() {
  rxPts = (RoutePoint *)ps_malloc(ROUTE_MAX_POINTS * sizeof(RoutePoint));
  if (!rxPts) {
    Serial.println("BLE rx buffer alloc failed");
    return;
  }

  NimBLEDevice::init("DontGetLost");
  NimBLEDevice::setMTU(517);

  NimBLEServer *srv = NimBLEDevice::createServer();
  srv->setCallbacks(new SrvCB());

  NimBLEService *svc = srv->createService(SVC_UUID);
  ctrlChr = svc->createCharacteristic(
      CTRL_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
  ctrlChr->setCallbacks(new CtrlCB());
  NimBLECharacteristic *dataChr = svc->createCharacteristic(
      DATA_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  dataChr->setCallbacks(new DataCB());
  infoChr = svc->createCharacteristic(INFO_UUID, NIMBLE_PROPERTY::READ);
  infoChr->setValue("[]");
  svc->start();

  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(SVC_UUID);
  adv->setName("DontGetLost");
  // the 128-bit service UUID fills the 31-byte advertisement, so the name
  // must ride in the scan response or scanners never see it
  adv->enableScanResponse(true);
  // relaxed advertising interval (200-400 ms) — a fraction of the radio
  // power of the default fast advertising, still discovered in ~1 s
  adv->setMinInterval(320);   // units of 0.625 ms
  adv->setMaxInterval(640);
  adv->start();
  Serial.println("BLE advertising as DontGetLost");
}

RouteBleOp routeBleTakeOp() {
  RouteBleOp op = pendingOp;
  pendingOp = RBLE_NONE;
  return op;
}
uint8_t routeBleOpSlot() { return pendingSlot; }
const char *routeBleOpName() { return rxName; }
uint16_t routeBleOpCount() { return rxCount; }
const RoutePoint *routeBleOpPoints() { return rxPts; }

void routeBleAck(uint8_t opcode, uint8_t status) {
  if (!ctrlChr) return;
  uint8_t msg[2] = {opcode, status};
  ctrlChr->setValue(msg, 2);
  ctrlChr->notify();
}

void routeBleSetInfo(const char *json) {
  if (infoChr) infoChr->setValue(json);
}

bool routeBleConnected() { return connected; }
