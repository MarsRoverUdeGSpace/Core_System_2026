#include "odom.h"

#include "enc.h"

#include <math.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

static constexpr float ODOM_TICKS_PER_REV = 5277.55F;
static constexpr float ODOM_WHEEL_RADIUS_M = 0.165F;
static constexpr float ODOM_WHEEL_SEPARATION_M = 1.14F;
static constexpr float ODOM_LEFT_TICKS_SIGN = 1.0F;
static constexpr float ODOM_RIGHT_TICKS_SIGN = -1.0F;
static constexpr double ODOM_TWO_PI = 6.28318530717958647692;

static odom_data_t g_odom_cache;
static SemaphoreHandle_t g_odom_mutex = NULL;
static uint32_t g_odom_seq = 0U;

static bool g_odom_have_baseline = false;
static int32_t g_last_left_ticks = 0;
static int32_t g_last_right_ticks = 0;
static TickType_t g_last_tick = 0U;
static uint32_t g_last_enc_seq = 0U;

static double g_odom_x_m = 0.0;
static double g_odom_y_m = 0.0;
static double g_odom_yaw_rad = 0.0;

static double odom_NormalizeYaw(const double yaw_rad)
{
  return atan2(sin(yaw_rad), cos(yaw_rad));
}

void App_Odom_Init(void)
{
  if (g_odom_mutex == NULL)
  {
    g_odom_mutex = xSemaphoreCreateMutex();
  }

  g_odom_cache.x_m = 0.0;
  g_odom_cache.y_m = 0.0;
  g_odom_cache.yaw_rad = 0.0;
  g_odom_cache.linear_x_mps = 0.0F;
  g_odom_cache.angular_z_radps = 0.0F;
  g_odom_cache.seq = 0U;
  g_odom_cache.valid = false;

  g_odom_seq = 0U;
  g_odom_have_baseline = false;
  g_last_left_ticks = 0;
  g_last_right_ticks = 0;
  g_last_tick = 0U;
  g_last_enc_seq = 0U;
  g_odom_x_m = 0.0;
  g_odom_y_m = 0.0;
  g_odom_yaw_rad = 0.0;
}

void App_Odom_Update(void)
{
  enc_data_t enc;
  if (!Hal_Enc_GetLatest(&enc) || (enc.valid == false))
  {
    return;
  }
  if (enc.seq == g_last_enc_seq)
  {
    return;
  }
  g_last_enc_seq = enc.seq;

  const TickType_t now = xTaskGetTickCount();
  const int32_t left_ticks = static_cast<int32_t>(ODOM_LEFT_TICKS_SIGN * enc.left_ticks);
  const int32_t right_ticks = static_cast<int32_t>(ODOM_RIGHT_TICKS_SIGN * enc.right_ticks);

  if (g_odom_have_baseline == false)
  {
    g_last_left_ticks = left_ticks;
    g_last_right_ticks = right_ticks;
    g_last_tick = now;
    g_odom_have_baseline = true;
    return;
  }

  const TickType_t dt_ticks = now - g_last_tick;
  if (dt_ticks == 0U)
  {
    return;
  }

  const double dt_s = static_cast<double>(dt_ticks) / static_cast<double>(configTICK_RATE_HZ);
  if (dt_s <= 0.0)
  {
    return;
  }

  const int32_t delta_left_ticks = left_ticks - g_last_left_ticks;
  const int32_t delta_right_ticks = right_ticks - g_last_right_ticks;
  const double meters_per_tick =
      (ODOM_TWO_PI * static_cast<double>(ODOM_WHEEL_RADIUS_M)) /
      static_cast<double>(ODOM_TICKS_PER_REV);

  const double dl_m = static_cast<double>(delta_left_ticks) * meters_per_tick;
  const double dr_m = static_cast<double>(delta_right_ticks) * meters_per_tick;
  const double dc_m = 0.5 * (dl_m + dr_m);
  const double dtheta_rad = (dr_m - dl_m) / static_cast<double>(ODOM_WHEEL_SEPARATION_M);

  g_odom_x_m += dc_m * cos(g_odom_yaw_rad + (0.5 * dtheta_rad));
  g_odom_y_m += dc_m * sin(g_odom_yaw_rad + (0.5 * dtheta_rad));
  g_odom_yaw_rad = odom_NormalizeYaw(g_odom_yaw_rad + dtheta_rad);

  odom_data_t local;
  local.x_m = g_odom_x_m;
  local.y_m = g_odom_y_m;
  local.yaw_rad = g_odom_yaw_rad;
  local.linear_x_mps = static_cast<float>(dc_m / dt_s);
  local.angular_z_radps = static_cast<float>(dtheta_rad / dt_s);
  local.seq = ++g_odom_seq;
  local.valid = true;

  if (g_odom_mutex != NULL)
  {
    if (xSemaphoreTake(g_odom_mutex, pdMS_TO_TICKS(1U)) == pdTRUE)
    {
      g_odom_cache = local;
      (void)xSemaphoreGive(g_odom_mutex);
    }
  }

  g_last_left_ticks = left_ticks;
  g_last_right_ticks = right_ticks;
  g_last_tick = now;
}

bool App_Odom_GetLatest(odom_data_t * out)
{
  if ((out == NULL) || (g_odom_mutex == NULL))
  {
    return false;
  }

  bool ok = false;
  if (xSemaphoreTake(g_odom_mutex, pdMS_TO_TICKS(1U)) == pdTRUE)
  {
    if (g_odom_cache.valid)
    {
      *out = g_odom_cache;
      ok = true;
    }
    (void)xSemaphoreGive(g_odom_mutex);
  }
  return ok;
}
