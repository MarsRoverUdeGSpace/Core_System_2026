#include "sensors.h"
#include "imu.h"
#include "alt.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

/* Deterministic sampling periods. */
static const TickType_t sensor_task_period_ticks = pdMS_TO_TICKS(20U);
static const TickType_t bme_period_ticks = pdMS_TO_TICKS(1000U);

typedef struct
{
  imu_data_t imu;
  bme_data_t bme;
} sensors_data_t;

/* Latest-sample cache protected by a mutex. */
static sensors_data_t g_sensors_data;
static SemaphoreHandle_t g_sensors_mutex = NULL;
static TaskHandle_t g_sensors_task = NULL;
static uint32_t g_imu_seq = 0U;
static uint32_t g_bme_seq = 0U;

static void sensors_Task(void * pvParameters)
{
  (void)pvParameters;

  TickType_t last_bme_tick = xTaskGetTickCount();
  TickType_t last_wake = xTaskGetTickCount();

  for (;;)
  {
    imu_data_t imu_local;
    imu_local.valid = false;

    /* IMU read: always attempt at 50 Hz. */
    if (Hal_Imu_Read(&imu_local.qw, &imu_local.qx, &imu_local.qy, &imu_local.qz,
                     &imu_local.gx, &imu_local.gy, &imu_local.gz,
                     &imu_local.ax, &imu_local.ay, &imu_local.az))
    {
      imu_local.valid = true;
    }
    imu_local.seq = ++g_imu_seq;

    bme_data_t bme_local;
    bme_local.valid = false;

    const TickType_t now = xTaskGetTickCount();
    if ((now - last_bme_tick) >= bme_period_ticks)
    {
      last_bme_tick = now;
      /* BME read: throttle to 1 Hz. */
      if (Hal_Alt_Read(&bme_local.temp_c, &bme_local.press_pa, &bme_local.hum_pct))
      {
        bme_local.valid = true;
        bme_local.seq = ++g_bme_seq;
      }
    }

    /* Commit latest samples atomically. */
    if (g_sensors_mutex != NULL)
    {
      if (xSemaphoreTake(g_sensors_mutex, pdMS_TO_TICKS(1U)) == pdTRUE)
      {
        g_sensors_data.imu = imu_local;
        if (bme_local.valid)
        {
          g_sensors_data.bme = bme_local;
        }
        else if ((now - last_bme_tick) == 0U)
        {
          /* Mark BME invalid only when a 1 Hz cycle has just executed. */
          g_sensors_data.bme.valid = false;
        }
        (void)xSemaphoreGive(g_sensors_mutex);
      }
    }

    vTaskDelayUntil(&last_wake, sensor_task_period_ticks);
  }
}

void Sensors_Init(void)
{
  if (g_sensors_mutex == NULL)
  {
    g_sensors_mutex = xSemaphoreCreateMutex();
  }
  /* Initialize cache to a known state. */
  g_sensors_data.imu.valid = false;
  g_sensors_data.bme.valid = false;
  g_sensors_data.imu.seq = 0U;
  g_sensors_data.bme.seq = 0U;
  g_imu_seq = 0U;
  g_bme_seq = 0U;
}

void Sensors_StartTask(void)
{
  if (g_sensors_task != NULL)
  {
    return;
  }

  /* Task pinned to core 1 to isolate I2C from micro-ROS transport. */
  (void)xTaskCreatePinnedToCore(
      sensors_Task,
      "AppSensors",
      4096U,
      NULL,
      2U,
      &g_sensors_task,
      1);
}

bool Sensors_GetImu(imu_data_t * out)
{
  if ((out == NULL) || (g_sensors_mutex == NULL))
  {
    return false;
  }

  bool ok = false;
  if (xSemaphoreTake(g_sensors_mutex, pdMS_TO_TICKS(1U)) == pdTRUE)
  {
    if (g_sensors_data.imu.valid)
    {
      *out = g_sensors_data.imu;
      ok = true;
    }
    (void)xSemaphoreGive(g_sensors_mutex);
  }
  return ok;
}

bool Sensors_GetBme(bme_data_t * out)
{
  if ((out == NULL) || (g_sensors_mutex == NULL))
  {
    return false;
  }

  bool ok = false;
  if (xSemaphoreTake(g_sensors_mutex, pdMS_TO_TICKS(1U)) == pdTRUE)
  {
    if (g_sensors_data.bme.valid)
    {
      *out = g_sensors_data.bme;
      ok = true;
    }
    (void)xSemaphoreGive(g_sensors_mutex);
  }
  return ok;
}
