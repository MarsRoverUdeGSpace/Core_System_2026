#include "alt.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

static const TickType_t bme_task_period_ticks = pdMS_TO_TICKS(1000U);

static bme_data_t g_bme_cache;
static SemaphoreHandle_t g_bme_mutex = NULL;
static TaskHandle_t g_bme_task = NULL;
static uint32_t g_bme_seq = 0U;

static void bme_Task(void * pvParameters)
{
  (void)pvParameters;
  TickType_t last_wake = xTaskGetTickCount();

  for (;;)
  {
    bme_data_t local;
    local.valid = false;

    if (Hal_Alt_Read(&local.temp_c, &local.press_pa, &local.hum_pct))
    {
      local.valid = true;
      local.seq = ++g_bme_seq;
    }

    if (g_bme_mutex != NULL)
    {
      if (xSemaphoreTake(g_bme_mutex, pdMS_TO_TICKS(1U)) == pdTRUE)
      {
        if (local.valid)
        {
          g_bme_cache = local;
        }
        else
        {
          g_bme_cache.valid = false;
        }
        (void)xSemaphoreGive(g_bme_mutex);
      }
    }

    vTaskDelayUntil(&last_wake, bme_task_period_ticks);
  }
}

void Hal_Alt_Init(void)
{
  if (g_bme_mutex == NULL)
  {
    g_bme_mutex = xSemaphoreCreateMutex();
  }
  g_bme_cache.valid = false;
  g_bme_cache.seq = 0U;
  g_bme_seq = 0U;
}

void Hal_Alt_StartTask(void)
{
  if (g_bme_task != NULL)
  {
    return;
  }

  (void)xTaskCreatePinnedToCore(
      bme_Task,
      "HalBme",
      4096U,
      NULL,
      2U,
      &g_bme_task,
      1);
}

bool Hal_Alt_GetLatest(bme_data_t * out)
{
  if ((out == NULL) || (g_bme_mutex == NULL))
  {
    return false;
  }

  bool ok = false;
  if (xSemaphoreTake(g_bme_mutex, pdMS_TO_TICKS(1U)) == pdTRUE)
  {
    if (g_bme_cache.valid)
    {
      *out = g_bme_cache;
      ok = true;
    }
    (void)xSemaphoreGive(g_bme_mutex);
  }
  return ok;
}

/**
 * @brief Read BME280 data (temperature, pressure, humidity).
 */
bool Hal_Alt_Read(float * temp_c, float * press_pa, float * hum_pct)
{
  if ((temp_c == NULL) || (press_pa == NULL) || (hum_pct == NULL))
  {
    return false;
  }
  if (bme280_ready == false)
  {
    return false;
  }

  const float temp = bme280.readTemperature();
  const float press = bme280.readPressure();
  const float hum = bme280.readHumidity();

  *temp_c = temp;
  *press_pa = press;
  *hum_pct = hum;

  return true;
}
