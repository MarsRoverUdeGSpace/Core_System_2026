/**
 * @file app.cpp
 * @brief Application layer implementation.
 */

#include "app.h"
#include "rte.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Task periods */
static const TickType_t app_task_period_ticks   = pdMS_TO_TICKS(10U);
static const TickType_t other_task_period_ticks = pdMS_TO_TICKS(1000U);

/**
 * @brief Initialize application layer (micro-ROS runtime).
 */
void app_Init(void)
{
  rte_Init();
}

/**
 * @brief Execute one application step (delegate to RTE).
 */
void app_Run(void)
{
  rte_Run();
}

/**
 * @brief Task entry for micro-ROS application (pinned to one core).
 *
 * @param pvParameters Unused parameter.
 */
static void app_TaskMicroRos(void * pvParameters)
{
  (void)pvParameters;

  for (;;)
  {
    app_Run();
    vTaskDelay(app_task_period_ticks);
  }
}

/**
 * @brief Placeholder task entry for second core.
 *
 * @param pvParameters Unused parameter.
 */
static void app_TaskOther(void * pvParameters)
{
  (void)pvParameters;

  for (;;)
  {
    /* Placeholder for future functionality. */
    vTaskDelay(other_task_period_ticks);
  }
}

/**
 * @brief Create application tasks pinned to each core.
 */
void app_StartTasks(void)
{
  (void)xTaskCreatePinnedToCore(
      app_TaskMicroRos,
      "AppMicroRos",
      4096U,
      NULL,
      2U,
      NULL,
      0);   /* Core 0 */

  (void)xTaskCreatePinnedToCore(
      app_TaskOther,
      "AppOther",
      4096U,
      NULL,
      1U,
      NULL,
      1);   /* Core 1 */
}

