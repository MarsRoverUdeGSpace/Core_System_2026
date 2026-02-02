/**
 * @file app.cpp
 * @brief Application layer implementation.
 */

#include "app.h"
#include "rte.h"
#include "config.h"
#include "motors.h"

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

  const TickType_t cmd_vel_timeout_ticks = pdMS_TO_TICKS(250U);
  TickType_t last_cmd_tick = xTaskGetTickCount();
  bool       have_cmd = false;

  for (;;)
  {
    cmd_velQueueMsg_t receivedMsg;
    /* Non-blocking receive keeps the motor task period stable. */
    if (xcmd_velQueue != NULL &&
        xQueueReceive(xcmd_velQueue, &receivedMsg, 0) == pdTRUE)
    {
      have_cmd = true;
      last_cmd_tick = xTaskGetTickCount();
      Hal_Motor_SetTwist(receivedMsg.linear_x, receivedMsg.angular_z);
    }
    else
    {
      const TickType_t now = xTaskGetTickCount();
      if ((have_cmd == false) || ((now - last_cmd_tick) > cmd_vel_timeout_ticks))
      {
        Hal_Motor_SetTwist(0.0F, 0.0F);
      }
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
