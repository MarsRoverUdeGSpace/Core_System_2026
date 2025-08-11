#pragma once
#include <RoboClaw.h>
#include <stdint.h>

float HalBatteryGetVoltage(RoboClaw &rc, uint8_t address);

