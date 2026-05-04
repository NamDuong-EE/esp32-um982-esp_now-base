#include "LoRaWan_APP.h"

uint8_t deviceState = DEVICE_STATE_INIT;
uint32_t txDutyCycleTime = 0;

MockLoRaWAN LoRaWAN;

void MockLoRaWAN::generateDeveuiByChipID() { generateDeveuiByChipIDCalled = true; }

void MockLoRaWAN::init(DeviceClass_t cls, LoRaMacRegion_t region) {
  initCalled = true;
  lastClass = cls;
  lastRegion = region;
}

void MockLoRaWAN::setDefaultDR(uint8_t dr) {
  setDefaultDRCalled = true;
  lastDefaultDR = dr;
}

void MockLoRaWAN::join() { joinCalled = true; }

void MockLoRaWAN::cycle(uint32_t dutyCycle) {
  cycleCalled = true;
  lastCycleDuty = dutyCycle;
}

void MockLoRaWAN::sleep(DeviceClass_t cls) {
  sleepCalled = true;
  lastClass = cls;
}

void MockLoRaWAN::resetMockState() {
  generateDeveuiByChipIDCalled = false;
  initCalled = false;
  setDefaultDRCalled = false;
  joinCalled = false;
  cycleCalled = false;
  sleepCalled = false;
  lastClass = CLASS_A;
  lastRegion = LORAMAC_REGION_AS923;
  lastDefaultDR = 0;
  lastCycleDuty = 0;
}
