/**
 * @file app.cpp
 * @brief Application layer implementation.
 */

#include "app.h"
#include "rte.h"
#include "config.h"
#include "motors.h"
#include "sensors.h"

/* Task periods for deterministic scheduling. */
static const TickType_t app_task_period_ticks   = pdMS_TO_TICKS(5U);
static const TickType_t motortask_period_ticks  = pdMS_TO_TICKS(10U);
static const TickType_t pub_task_period_ticks   = pdMS_TO_TICKS(1U);
static const TickType_t imu_pub_period_ticks    = pdMS_TO_TICKS(20U);
static const TickType_t bme_pub_period_ticks    = pdMS_TO_TICKS(1000U);

/**
 * @brief Initialize application layer (Micro-ROS runtime).
 */
void app_Init(void)
{
  /* Order matters: transport/config must be ready before sensors/publishers. */
  rte_Init();
  Sensors_Init();
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
  TickType_t last_wake = xTaskGetTickCount();

  for (;;)
  {
    /* Spin micro-ROS executor only; no I2C or publishing here. */
    rte_SpinOnce();
    vTaskDelayUntil(&last_wake, app_task_period_ticks);
  }
}

static void app_TaskPublisher(void * pvParameters)
{
  (void)pvParameters;

  TickType_t last_wake = xTaskGetTickCount();
  TickType_t imu_wake = last_wake;
  TickType_t bme_wake = last_wake;

  for (;;)
  {
    const TickType_t now = xTaskGetTickCount();

    /* IMU publisher: fixed 50 Hz. */
    if ((now - imu_wake) >= imu_pub_period_ticks)
    {
      imu_wake += imu_pub_period_ticks;
      rte_PublishImu();
    }

    /* BME + debug publishers: fixed 1 Hz. */
    if ((now - bme_wake) >= bme_pub_period_ticks)
    {
      bme_wake += bme_pub_period_ticks;
      rte_PublishBme();
      rte_PublishDebug();
    }

    vTaskDelayUntil(&last_wake, pub_task_period_ticks);
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
  TickType_t last_wake = xTaskGetTickCount();

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
      /* Timeout guard: stop motors if command stream stalls. */
      const TickType_t now = xTaskGetTickCount();
      if ((have_cmd == false) || ((now - last_cmd_tick) > cmd_vel_timeout_ticks))
      {
        Hal_Motor_SetTwist(0.0F, 0.0F);
      }
    }
    vTaskDelayUntil(&last_wake, motortask_period_ticks);
  }
  
}

/**
 * @brief Create application tasks pinned to each core.
 */
void app_StartTasks(void)
{
  /* Sensor task must be running before publishers. */
  Sensors_StartTask();

  (void)xTaskCreatePinnedToCore(
      app_TaskMicroRos,
      "AppMicroRos",
      4096U,
      NULL,
      4U,
      NULL,
      0);   /* Core 0 */

  /* Publisher task runs on the sensor core to minimize cross-core contention. */
  (void)xTaskCreatePinnedToCore(
      app_TaskPublisher,
      "AppPublisher",
      4096U,
      NULL,
      3U,
      NULL,
      1);   /* Core 1 */

  /* Motor task runs on the sensor core at lower priority. */
  (void)xTaskCreatePinnedToCore(
      app_TaskMotorControl,
      "AppMotorControl",
      4096U,
      NULL,
      2U,
      NULL,
      1);   /* Core 1 */
}
