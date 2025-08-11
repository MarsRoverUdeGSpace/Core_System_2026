#include "hal_motor.h"

static inline float db(float v, float t=0.05f) {
  return (fabsf(v) < t) ? 0.0f : v;
}

void HalMotorInit(HardwareSerial &s) {
  s.begin(RBW_BAUD,  SERIAL_8N1, 40, 39);
  roboclaw.begin(RBW_BAUD);
}

MotorCommand_t HalMotorMakeCommand(const sensor_msgs::Joy &joy) {
  // choose speed/turn scales based on buttons
  float thr = 0.0f, trn = 0.0f;
  if      (joy.buttons[1]) { thr = 0.1f;  trn = 0.2f;  } // B
  else if (joy.buttons[2]) { thr = 0.2f;  trn = 0.3f;  } // X
  else if (joy.buttons[3]) { thr = 0.3f;  trn = 0.4f;  } // Y
  else if (joy.buttons[0]) { thr = 0.5f;  trn = 0.7f;  } // A
  else if (joy.buttons[5]) { thr = 0.7f;  trn = 0.85f; } // RB
  else if (joy.buttons[4]) { thr = 0.99f; trn = 0.99f; } // LB

  // raw normalized inputs
  float t =  db(joy.axes[1]) * thr;
  float r = db(joy.axes[0]) * trn;

  int16_t rawL = (uint16_t)roundf((t + r) * 127.0f);
  int16_t rawRgt = (uint16_t)roundf((t - r) * 127.0f);
  
  rawL  = constrain(rawL,  -127,  127);
  rawRgt = constrain(rawRgt, -127, 127);

  // pack into int8 range
  MotorCommand_t cmd;
  cmd.m1_speed = (int8_t)rawL;
  cmd.m2_speed = (int8_t)rawRgt;
  cmd.m3_speed = cmd.m1_speed;
  cmd.m4_speed = cmd.m2_speed;
  return cmd;
}

void HalMotorApply(const MotorCommand_t *cmd) {
  // M1
  if      (cmd->m1_speed > 0) roboclaw.ForwardM1 (MTR1_ADDR, cmd->m1_speed);
  else if (cmd->m1_speed < 0) roboclaw.BackwardM1(MTR1_ADDR, -cmd->m1_speed);
  else                         roboclaw.ForwardM1 (MTR1_ADDR, 0);
  // M2
  if      (cmd->m2_speed > 0) roboclaw.ForwardM2 (MTR2_ADDR, cmd->m2_speed);
  else if (cmd->m2_speed < 0) roboclaw.BackwardM2(MTR2_ADDR, -cmd->m2_speed);
  else                         roboclaw.ForwardM2 (MTR2_ADDR, 0);
  // M3
  if      (cmd->m3_speed > 0) roboclaw.ForwardM1 (MTR3_ADDR, cmd->m3_speed);
  else if (cmd->m3_speed < 0) roboclaw.BackwardM1(MTR3_ADDR, -cmd->m3_speed);
  else                         roboclaw.ForwardM1 (MTR3_ADDR, 0);
  // M4
  if      (cmd->m4_speed > 0) roboclaw.ForwardM2 (MTR4_ADDR, cmd->m4_speed);
  else if (cmd->m4_speed < 0) roboclaw.BackwardM2(MTR4_ADDR, -cmd->m4_speed);
  else                         roboclaw.ForwardM2 (MTR4_ADDR, 0);
}

void HalMotorQueueCommand(const MotorCommand_t *cmd) {
  xQueueOverwrite(motor_queue, cmd);
}

