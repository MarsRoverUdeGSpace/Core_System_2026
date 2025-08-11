#include <Arduino.h>
#include "config.h"
#include "eStop.h"
#include "hal_led.h"
#include "hal_motor.h"
#include "ros_interface.h"
#include "tasks.h"

void setup() {
  HalLedInit();
  EmergencyStopInit();
  HalMotorInit(Serial1);
  RosIfInit();

  motor_queue = xQueueCreate(1, sizeof(MotorCommand_t));
  xTaskCreatePinnedToCore(rosTask,   "ros_io", 4096, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(motorTask, "motor",  4096, nullptr, 1, nullptr, 1);
}

void loop() {
  // all work is in tasks
}

