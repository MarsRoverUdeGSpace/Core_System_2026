#pragma once
#include "config.h"
#include "freertos/task.h"

typedef enum {
  TELEOPERATED   = 0,
  AUTONOMOUS     = 1,
  ARUCO_DETECTED = 2
} OperationMode;

void HalLedInit();
void HalLedHandleMode(OperationMode mode);

