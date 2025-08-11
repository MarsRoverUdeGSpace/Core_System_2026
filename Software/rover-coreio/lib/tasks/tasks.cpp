#include "tasks.h"
#include "config.h"         // for roboclaw1, roboclaw2, motor_queue, MTRx_ADDR
#include "eStop.h"
#include "ros_interface.h"  // for RosIfGetNH(), RosIfGetBattPub()
#include "hal_battery.h"    // for HalBatteryGetVoltage()
#include "hal_motor.h"      // for MotorCommand_t, HalMotorApply()
#include <std_msgs/Float32.h>

void rosTask(void* ) {
  auto &nh  = RosIfGetNH();
  auto &batt_pub = RosIfGetBattPub();
  auto &curr_pub = RosIfGetCurrPub();
  std_msgs::Float32 battMsg;
  std_msgs::Float32 currMsg;
  uint32_t last = 0;
  int16_t  c1, c2;

  for (;;) {
    nh.spinOnce();
    uint32_t now = millis();
    if (now - last < 1000) continue;
    last = now;
    
    battMsg.data = HalBatteryGetVoltage(roboclaw, MTR1_ADDR);
    batt_pub.publish(&battMsg);
    battMsg.data = HalBatteryGetVoltage(roboclaw, MTR3_ADDR);

    if (roboclaw.ReadCurrents(MTR1_ADDR, c1, c2)) {
      currMsg.data = c1 * 0.1f;
      curr_pub.publish(&currMsg);
      currMsg.data = c2 * 0.2f;
      curr_pub.publish(&currMsg);
    }
  }
}

void motorTask(void* ) {
  MotorCommand_t cmd;
  for (;;) {
    if(EmergencyStopActivated()) {
      if (xQueueReceive(motor_queue, &cmd, portMAX_DELAY) == pdPASS) {
        HalMotorApply(&cmd);
      }
    }
    else {
        cmd.m1_speed = 0.0f;
        cmd.m2_speed = 0.0f;
        cmd.m3_speed = 0.0f;
        cmd.m4_speed = 0.0f;
        HalMotorApply(&cmd);
    }
  }
}

