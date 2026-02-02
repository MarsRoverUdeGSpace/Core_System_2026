#include "motors.h"

static float clamp_unit(float value)
{
  if (value > 1.0F)
  {
    return 1.0F;
  }
  if (value < -1.0F)
  {
    return -1.0F;
  }
  return value;
}

static float apply_deadband(float value)
{
  static constexpr float deadband = 0.03F;
  if ((value > -deadband) && (value < deadband))
  {
    return 0.0F;
  }
  return value;
}

static uint8_t to_pwm(float value)
{
  float magnitude = value;
  if (magnitude < 0.0F)
  {
    magnitude = -magnitude;
  }
  if (magnitude > 1.0F)
  {
    magnitude = 1.0F;
  }
  float scaled = (magnitude * 255.0F) + 0.5F;
  if (scaled > 255.0F)
  {
    scaled = 255.0F;
  }
  return static_cast<uint8_t>(scaled);
}

/**
 * @brief It receives linear (m/s) and angular (rad/s) velocity and moves motors
 */
void Hal_Motor_SetTwist(float linear_x, float angular_z)
{
  const float left_cmd  = apply_deadband(clamp_unit(linear_x - angular_z));
  const float right_cmd = apply_deadband(clamp_unit(linear_x + angular_z));

  const uint8_t pwm_left  = to_pwm(left_cmd);
  const uint8_t pwm_right = to_pwm(right_cmd);

  const bool left_forward  = (left_cmd >= 0.0F);
  const bool right_forward = (right_cmd >= 0.0F);

  // M1 
  if (left_forward) roboclaw.ForwardM1(ADDR_RB1, pwm_left);
  else              roboclaw.BackwardM1(ADDR_RB1, pwm_left);

  // M2
  if (right_forward) roboclaw.ForwardM2(ADDR_RB1, pwm_right);
  else               roboclaw.BackwardM2(ADDR_RB1, pwm_right);

  // M3 
  if (left_forward) roboclaw.ForwardM1(ADDR_RB2, pwm_left);
  else              roboclaw.BackwardM1(ADDR_RB2, pwm_left);

  // M4
  if (right_forward) roboclaw.ForwardM2(ADDR_RB2, pwm_right);
  else               roboclaw.BackwardM2(ADDR_RB2, pwm_right);
}
