#include "imu.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <utility/imumaths.h>

static float dps_to_rad(float value_dps)
{
  static constexpr float deg_to_rad = 0.01745329252F;
  return value_dps * deg_to_rad;
}

static const TickType_t imu_task_period_ticks = pdMS_TO_TICKS(20U);

static imu_data_t g_imu_cache;
static SemaphoreHandle_t g_imu_mutex = NULL;
static TaskHandle_t g_imu_task = NULL;
static uint32_t g_imu_seq = 0U;

static void imu_Task(void * pvParameters)
{
  (void)pvParameters;
  TickType_t last_wake = xTaskGetTickCount();

  for (;;)
  {
    imu_data_t local;
    local.valid = false;

    if (Hal_Imu_Read(&local.qw, &local.qx, &local.qy, &local.qz,
                     &local.gx, &local.gy, &local.gz,
                     &local.ax, &local.ay, &local.az))
    {
      local.valid = true;
    }
    local.seq = ++g_imu_seq;

    if (g_imu_mutex != NULL)
    {
      if (xSemaphoreTake(g_imu_mutex, pdMS_TO_TICKS(1U)) == pdTRUE)
      {
        g_imu_cache = local;
        (void)xSemaphoreGive(g_imu_mutex);
      }
    }

    vTaskDelayUntil(&last_wake, imu_task_period_ticks);
  }
}

void Hal_Imu_Init(void)
{
  if (g_imu_mutex == NULL)
  {
    g_imu_mutex = xSemaphoreCreateMutex();
  }
  g_imu_cache.valid = false;
  g_imu_cache.seq = 0U;
  g_imu_seq = 0U;
}

void Hal_Imu_StartTask(void)
{
  if (g_imu_task != NULL)
  {
    return;
  }

  (void)xTaskCreatePinnedToCore(
      imu_Task,
      "HalImu",
      4096U,
      NULL,
      2U,
      &g_imu_task,
      1);
}

bool Hal_Imu_GetLatest(imu_data_t * out)
{
  if ((out == NULL) || (g_imu_mutex == NULL))
  {
    return false;
  }

  bool ok = false;
  if (xSemaphoreTake(g_imu_mutex, pdMS_TO_TICKS(1U)) == pdTRUE)
  {
    if (g_imu_cache.valid)
    {
      *out = g_imu_cache;
      ok = true;
    }
    (void)xSemaphoreGive(g_imu_mutex);
  }
  return ok;
}

/**
 * @brief Read IMU data (quaternion, gyro, linear accel).
 */
bool Hal_Imu_Read(float * quat_w, float * quat_x, float * quat_y, float * quat_z,
                  float * gyro_x, float * gyro_y, float * gyro_z,
                  float * accel_x, float * accel_y, float * accel_z)
{
  if ((quat_w == NULL) || (quat_x == NULL) || (quat_y == NULL) || (quat_z == NULL) ||
      (gyro_x == NULL) || (gyro_y == NULL) || (gyro_z == NULL) ||
      (accel_x == NULL) || (accel_y == NULL) || (accel_z == NULL))
  {
    return false;
  }
  if (bno055_ready == false)
  {
    return false;
  }

  const imu::Quaternion quat = bno055.getQuat();
  const imu::Vector<3> gyro_dps = bno055.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  const imu::Vector<3> accel = bno055.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  *quat_w = static_cast<float>(quat.w());
  *quat_x = static_cast<float>(quat.x());
  *quat_y = static_cast<float>(quat.y());
  *quat_z = static_cast<float>(quat.z());

  *gyro_x = dps_to_rad(static_cast<float>(gyro_dps.x()));
  *gyro_y = dps_to_rad(static_cast<float>(gyro_dps.y()));
  *gyro_z = dps_to_rad(static_cast<float>(gyro_dps.z()));

  *accel_x = static_cast<float>(accel.x());
  *accel_y = static_cast<float>(accel.y());
  *accel_z = static_cast<float>(accel.z());

  return true;
}
