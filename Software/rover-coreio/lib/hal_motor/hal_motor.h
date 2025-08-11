#pragma once
#include <RoboClaw.h>
#include <math.h>
#include <sensor_msgs/Joy.h>
#include "freertos/queue.h"
#include "config.h"

typedef struct {
  int8_t m1_speed, m2_speed, m3_speed, m4_speed;
} MotorCommand_t;

void HalMotorInit(HardwareSerial &s);
void HalMotorApply(const MotorCommand_t *cmd);
MotorCommand_t HalMotorMakeCommand(const sensor_msgs::Joy &joy);
void HalMotorQueueCommand(const MotorCommand_t *cmd);
