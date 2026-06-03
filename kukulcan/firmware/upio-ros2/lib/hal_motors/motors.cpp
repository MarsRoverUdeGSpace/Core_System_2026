#include "motors.h"

static bool g_motor_output_valid = false;
static bool g_last_left_forward = true;
static bool g_last_right_forward = true;
static uint8_t g_last_pwm_left = 0U;
static uint8_t g_last_pwm_right = 0U;

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

static void set_motor_twist(float linear_x, float angular_z, bool force_write)
{
  if (xRoboClawMutex == NULL)
  {
    return;
  }
  if (xSemaphoreTake(xRoboClawMutex, pdMS_TO_TICKS(3U)) != pdTRUE)
  {
    return;
  }

  const float left_cmd  = apply_deadband(clamp_unit(linear_x - angular_z));
  const float right_cmd = apply_deadband(clamp_unit(linear_x + angular_z));

  const uint8_t pwm_left  = to_pwm(left_cmd);
  const uint8_t pwm_right = to_pwm(right_cmd);

  const bool left_forward  = (left_cmd >= 0.0F);
  const bool right_forward = (right_cmd >= 0.0F);

  if (g_motor_output_valid &&
      (force_write == false) &&
      (g_last_left_forward == left_forward) &&
      (g_last_right_forward == right_forward) &&
      (g_last_pwm_left == pwm_left) &&
      (g_last_pwm_right == pwm_right))
  {
    (void)xSemaphoreGive(xRoboClawMutex);
    return;
  }

  /* RoboClaw 0x80 controls the full left side. */
  const bool left_m1_ok = left_forward ?
      roboclaw.ForwardM1(ADDR_RB1, pwm_left) :
      roboclaw.BackwardM1(ADDR_RB1, pwm_left);

  const bool left_m2_ok = left_forward ?
      roboclaw.ForwardM2(ADDR_RB1, pwm_left) :
      roboclaw.BackwardM2(ADDR_RB1, pwm_left);

  /* RoboClaw 0x81 controls the full right side. */
  const bool right_m1_ok = right_forward ?
      roboclaw.ForwardM1(ADDR_RB2, pwm_right) :
      roboclaw.BackwardM1(ADDR_RB2, pwm_right);

  const bool right_m2_ok = right_forward ?
      roboclaw.ForwardM2(ADDR_RB2, pwm_right) :
      roboclaw.BackwardM2(ADDR_RB2, pwm_right);

  if (left_m1_ok && left_m2_ok && right_m1_ok && right_m2_ok)
  {
    g_last_left_forward = left_forward;
    g_last_right_forward = right_forward;
    g_last_pwm_left = pwm_left;
    g_last_pwm_right = pwm_right;
    g_motor_output_valid = true;
  }

  (void)xSemaphoreGive(xRoboClawMutex);
}

/**
 * @brief It receives linear (m/s) and angular (rad/s) velocity and moves motors
 */
void Hal_Motor_SetTwist(float linear_x, float angular_z)
{
  set_motor_twist(linear_x, angular_z, false);
}

void Hal_Motor_Stop(void)
{
  set_motor_twist(0.0F, 0.0F, true);
}
