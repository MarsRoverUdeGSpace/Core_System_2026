/**
 * @file app.cpp
 * @brief Application layer implementation.
 */

#include "app.h"
#include "rte.h"
#include "config.h"
#include "motors.h"
#include "enc.h"
#include "imu.h"
#include "alt.h"
#include "gnss.h"
#include "led_status.h"
#include "odom.h"

/* Task periods for deterministic scheduling. */
static const TickType_t app_task_period_ticks   = pdMS_TO_TICKS(5U);
static const TickType_t motortask_period_ticks  = pdMS_TO_TICKS(10U);
static const TickType_t pub_task_period_ticks   = pdMS_TO_TICKS(5U);
static const TickType_t imu_pub_period_ticks    = pdMS_TO_TICKS(40U);
static const TickType_t mag_pub_period_ticks    = pdMS_TO_TICKS(200U);
static const TickType_t enc_pub_period_ticks    = pdMS_TO_TICKS(100U);
static const TickType_t odom_pub_period_ticks   = pdMS_TO_TICKS(100U);
static const TickType_t gnss_pub_period_ticks   = pdMS_TO_TICKS(200U);
static const TickType_t bme_pub_period_ticks    = pdMS_TO_TICKS(1000U);

/**
 * @brief Initialize application layer (Micro-ROS runtime).
 */
void app_Init(void)
{
  /* Order matters: transport/config must be ready before sensors/publishers. */
  rte_Init();
  App_Odom_Init();
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
  TickType_t mag_wake = last_wake;
  TickType_t enc_wake = last_wake;
  TickType_t odom_wake = last_wake;
  TickType_t gnss_wake = last_wake;
  TickType_t bme_wake = last_wake;

  for (;;)
  {
    const TickType_t now = xTaskGetTickCount();

    /* IMU publisher: fixed 25 Hz for localization without excess traffic. */
    if ((now - imu_wake) >= imu_pub_period_ticks)
    {
      imu_wake = now;
      rte_PublishImu();
    }

    /* Magnetometer publisher: fixed 5 Hz, separate from sensor_msgs/Imu. */
    if ((now - mag_wake) >= mag_pub_period_ticks)
    {
      mag_wake = now;
      rte_PublishMag();
    }

    if ((now - enc_wake) >= enc_pub_period_ticks)
    {
      enc_wake = now;
      rte_PublishEncoders();
    }

    if ((now - odom_wake) >= odom_pub_period_ticks)
    {
      odom_wake = now;
      App_Odom_Update();
      rte_PublishOdom();
    }

    if ((now - gnss_wake) >= gnss_pub_period_ticks)
    {
      gnss_wake = now;
      rte_PublishGnss();
    }

    /* BME publisher: fixed 1 Hz, matching the sensor update period. */
    if ((now - bme_wake) >= bme_pub_period_ticks)
    {
      bme_wake = now;
      rte_PublishBme();
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
  bool       timeout_stop_sent = false;
  bool       last_cmd_zero = true;
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
      timeout_stop_sent = false;
      if ((receivedMsg.linear_x == 0.0F) && (receivedMsg.angular_z == 0.0F))
      {
        if (last_cmd_zero == false)
        {
          Hal_Motor_Stop();
        }
        else
        {
          Hal_Motor_SetTwist(0.0F, 0.0F);
        }
        last_cmd_zero = true;
      }
      else
      {
        Hal_Motor_SetTwist(receivedMsg.linear_x, receivedMsg.angular_z);
        last_cmd_zero = false;
      }
    }
    else
    {
      /* Timeout guard: stop motors if command stream stalls. */
      const TickType_t now = xTaskGetTickCount();
      if (((have_cmd == false) || ((now - last_cmd_tick) > cmd_vel_timeout_ticks)) &&
          (timeout_stop_sent == false))
      {
        Hal_Motor_Stop();
        timeout_stop_sent = true;
        last_cmd_zero = true;
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
  /* Sensor tasks must be running before publishers. */
  Hal_LedStatus_StartTask();
  Hal_Imu_StartTask();
  Hal_Alt_StartTask();
  Hal_Enc_StartTask();
  Hal_Gnss_StartTask();

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
      1U,
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
