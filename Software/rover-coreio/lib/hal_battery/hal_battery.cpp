#include "hal_battery.h"

float HalBatteryGetVoltage(RoboClaw &rc, uint8_t address) {
  uint16_t raw = rc.ReadMainBatteryVoltage(address);
  return raw/10.0f;
}

