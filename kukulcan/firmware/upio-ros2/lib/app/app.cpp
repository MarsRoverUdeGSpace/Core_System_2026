/**
 * @file app.cpp
 * @brief Application layer implementation.
 */

#include "app.h"
#include "rte.h"
#include "config.h"

/* Task periods for application tasks. */
static const TickType_t app_task_period_ticks   = pdMS_TO_TICKS(10U);
static const TickType_t motortask_period_ticks  = pdMS_TO_TICKS(15U);

/**
 * @brief Initialize application layer (Micro-ROS runtime).
 */
void app_Init(void)
{
  rte_Init();
}

/**
 * @brief Execute one application step (delegate to the RTE).
 */
void app_Run(void)
{
  rte_Run();
}

/**
 * @brief Task entry for micro-ROS processing (pinned to one core).
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
 * @brief Motor control task entry for the second core.
 *
 * @param pvParameters Unused parameter.
 */
static void app_TaskMotorControl(void * pvParameters)
{
  (void)pvParameters;
  
  for (;;)
  {
    cmd_velQueueMsg_t receivedMsg;
    /* Non-blocking receive keeps the motor task period stable. */
    if (xcmd_velQueue != NULL &&
        xQueueReceive(xcmd_velQueue, &receivedMsg, 0) == pdTRUE)
    {
      // Apply the received velocity command to motors.
    }
    vTaskDelay(motortask_period_ticks);
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
      app_TaskMotorControl,
      "AppMotorControl",
      4096U,
      NULL,
      1U,
      NULL,
      1);   /* Core 1 */
}
