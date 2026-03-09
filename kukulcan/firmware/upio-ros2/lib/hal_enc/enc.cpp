#include "enc.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

static const TickType_t enc_task_period_ticks = pdMS_TO_TICKS(20U);

static enc_data_t g_enc_cache;
static SemaphoreHandle_t g_enc_mutex = NULL;
static TaskHandle_t g_enc_task = NULL;
static uint32_t g_enc_seq = 0U;

static void enc_Task(void * pvParameters)
{
  (void)pvParameters;
  TickType_t last_wake = xTaskGetTickCount();

  for (;;)
  {
    enc_data_t local;
    local.valid = false;

    if (Hal_Enc_Read(&local.left_ticks,
                     &local.right_ticks,
                     &local.left_qpps,
                     &local.right_qpps,
                     &local.left_status,
                     &local.right_status))
    {
      local.valid = true;
    }
    local.seq = ++g_enc_seq;

    if (g_enc_mutex != NULL)
    {
      if (xSemaphoreTake(g_enc_mutex, pdMS_TO_TICKS(1U)) == pdTRUE)
      {
        g_enc_cache = local;
        (void)xSemaphoreGive(g_enc_mutex);
      }
    }

    vTaskDelayUntil(&last_wake, enc_task_period_ticks);
  }
}

void Hal_Enc_Init(void)
{
  if (g_enc_mutex == NULL)
  {
    g_enc_mutex = xSemaphoreCreateMutex();
  }
  g_enc_cache.valid = false;
  g_enc_cache.seq = 0U;
  g_enc_seq = 0U;
}

void Hal_Enc_StartTask(void)
{
  if (g_enc_task != NULL)
  {
    return;
  }

  (void)xTaskCreatePinnedToCore(
      enc_Task,
      "HalEnc",
      4096U,
      NULL,
      2U,
      &g_enc_task,
      1);
}

bool Hal_Enc_GetLatest(enc_data_t * out)
{
  if ((out == NULL) || (g_enc_mutex == NULL))
  {
    return false;
  }

  bool ok = false;
  if (xSemaphoreTake(g_enc_mutex, pdMS_TO_TICKS(1U)) == pdTRUE)
  {
    if (g_enc_cache.valid)
    {
      *out = g_enc_cache;
      ok = true;
    }
    (void)xSemaphoreGive(g_enc_mutex);
  }
  return ok;
}

bool Hal_Enc_Read(int32_t * left_ticks,
                  int32_t * right_ticks,
                  int32_t * left_qpps,
                  int32_t * right_qpps,
                  uint8_t * left_status,
                  uint8_t * right_status)
{
  if ((left_ticks == NULL) || (right_ticks == NULL) ||
      (left_qpps == NULL) || (right_qpps == NULL) ||
      (left_status == NULL) || (right_status == NULL))
  {
    return false;
  }
  if (xRoboClawMutex == NULL)
  {
    return false;
  }
  if (xSemaphoreTake(xRoboClawMutex, pdMS_TO_TICKS(3U)) != pdTRUE)
  {
    return false;
  }

  bool left_valid = false;
  bool right_valid = false;
  bool left_speed_valid = false;
  bool right_speed_valid = false;

  const int32_t left_ticks_local =
      static_cast<int32_t>(roboclaw.ReadEncM1(ADDR_RB1, left_status, &left_valid));
  const int32_t right_ticks_local =
      static_cast<int32_t>(roboclaw.ReadEncM1(ADDR_RB2, right_status, &right_valid));
  const int32_t left_qpps_local =
      static_cast<int32_t>(roboclaw.ReadSpeedM1(ADDR_RB1, NULL, &left_speed_valid));
  const int32_t right_qpps_local =
      static_cast<int32_t>(roboclaw.ReadSpeedM1(ADDR_RB2, NULL, &right_speed_valid));

  (void)xSemaphoreGive(xRoboClawMutex);

  if ((!left_valid) || (!right_valid) || (!left_speed_valid) || (!right_speed_valid))
  {
    return false;
  }

  *left_ticks = left_ticks_local;
  *right_ticks = right_ticks_local;
  *left_qpps = left_qpps_local;
  *right_qpps = right_qpps_local;
  return true;
}
