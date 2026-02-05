/**
 * @file rte.cpp
 * @brief Micro-ROS runtime environment implementation.
 */
#include "config.h"
#include "rte.h"
#include "imu.h"
#include "alt.h"

#include <stdio.h>
#include <string.h>
#include <freertos/semphr.h>

#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/temperature.h>
#include <sensor_msgs/msg/fluid_pressure.h>
#include <sensor_msgs/msg/relative_humidity.h>
#include <sensor_msgs/msg/imu.h>


static constexpr const char * RTE_NODE           = "kukulcan";
static constexpr const char * RTE_TOPIC_TEMP     = "sensors/bme280/temperature";
static constexpr const char * RTE_TOPIC_PRESS    = "sensors/bme280/pressure";
static constexpr const char * RTE_TOPIC_HUM      = "sensors/bme280/humidity";
static constexpr const char * RTE_TOPIC_IMU      = "imu/data";
static constexpr const char * RTE_TOPIC_SUB      = "cmd_vel";

/* Transport spin period is controlled by the app task (see app.cpp). */
static constexpr uint32_t     RTE_SPIN_PERIOD_MS = 10U;
static constexpr TickType_t   RTE_BME_PUB_PERIOD_TICKS = pdMS_TO_TICKS(1000U);

/* RTE state for node, executor, and entities. */
static MicroRosState rte_state;

/* Publisher state for BME280. */
static rcl_publisher_t rte_pub_temp;
static rcl_publisher_t rte_pub_press;
static rcl_publisher_t rte_pub_hum;
static rcl_publisher_t rte_pub_imu;
static rcl_timer_t     rte_timer;

static sensor_msgs__msg__Temperature      rte_temp_msg;
static sensor_msgs__msg__FluidPressure    rte_press_msg;
static sensor_msgs__msg__RelativeHumidity rte_hum_msg;
static sensor_msgs__msg__Imu              rte_imu_msg;
static char                               rte_imu_frame_id[16];

/* Subscriber state for cmd_vel. */
static geometry_msgs__msg__Twist rte_sub_msg;
/* Guard all rcl/rmw calls; micro-ROS stack is not thread-safe. */
static SemaphoreHandle_t g_rcl_mutex = NULL;
static volatile bool g_ros_ready = false;
static volatile uint32_t g_spin_count = 0U;
static uint32_t g_imu_pub_ok = 0U;
static uint32_t g_imu_pub_miss = 0U;
static uint32_t g_bme_pub_ok = 0U;
static uint32_t g_bme_pub_miss = 0U;
static TickType_t g_err_led_until = 0U;

static void rte_ErrorLed_Update(void)
{
  const TickType_t now = xTaskGetTickCount();
  if ((g_err_led_until != 0U) && (now >= g_err_led_until))
  {
    digitalWrite(LED_RED_PIN, LOW);
    g_err_led_until = 0U;
  }
}

static void rte_ErrorLed_Pulse(void)
{
  digitalWrite(LED_RED_PIN, HIGH);
  g_err_led_until = xTaskGetTickCount() + pdMS_TO_TICKS(100U);
}

static void rte_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  (void)last_call_time;
  if (timer == NULL)
  {
    return;
  }
}

/**
 * @brief Subscriber callback for the "cmd_vel" topic.
 *
 * @param msgin Incoming Twist message.
 */
static void rte_sub_callback(const void * msgin)
{
  const geometry_msgs__msg__Twist * msg =
  static_cast<const geometry_msgs__msg__Twist *>(msgin);

  if (xcmd_velQueue == NULL)
  {
    return;
  }

  cmd_velQueueMsg_t queue_msg;
  queue_msg.linear_x = msg->linear.x;
  queue_msg.angular_z = msg->angular.z;

  (void)xQueueOverwrite(xcmd_velQueue, &queue_msg);
}

/**
 * @brief Initialize transport, allocator, node, entities, and executor.
 */
void rte_Init(void)
{
  config_init();
  if (RTE_USE_USB_CDC)
  {
    set_microros_serial_transports(Serial);
  }
  else
  {
    set_microros_serial_transports(rte_serial);
  }
  /* Allow transport to settle before creating ROS entities. */
  delay(2000U);

  /* Create mutex for all rcl/rmw calls before any ROS entities. */
  if (g_rcl_mutex == NULL)
  {
    g_rcl_mutex = xSemaphoreCreateMutex();
    if (g_rcl_mutex == NULL)
    {
      error_loop();
    }
  }

  rte_state.allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(
      &rte_state.support,
      0,
      NULL,
      &rte_state.allocator));

  RCCHECK(rclc_node_init_default(
      &rte_state.node,
      RTE_NODE,
      "",
      &rte_state.support));
  
  /* --- PUBLISHERS --- */

  RCCHECK(rclc_publisher_init_default(
      &rte_pub_temp,
      &rte_state.node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature),
      RTE_TOPIC_TEMP));

  RCCHECK(rclc_publisher_init_default(
      &rte_pub_press,
      &rte_state.node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, FluidPressure),
      RTE_TOPIC_PRESS));

  RCCHECK(rclc_publisher_init_default(
      &rte_pub_hum,
      &rte_state.node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, RelativeHumidity),
      RTE_TOPIC_HUM));

  RCCHECK(rclc_publisher_init_best_effort(
      &rte_pub_imu,
      &rte_state.node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      RTE_TOPIC_IMU));

  rte_imu_msg.header.frame_id.data = rte_imu_frame_id;
  rte_imu_msg.header.frame_id.size = 0U;
  rte_imu_msg.header.frame_id.capacity = sizeof(rte_imu_frame_id);
  (void)snprintf(rte_imu_frame_id, sizeof(rte_imu_frame_id), "imu_link");
  rte_imu_msg.header.frame_id.size = strlen(rte_imu_frame_id);

   /* ------SUBSCRIBERS-------- */

  // Subscriber: cmd_vel (Twist).
  RCCHECK(rclc_subscription_init_best_effort(
      &rte_state.subs,
      &rte_state.node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      RTE_TOPIC_SUB));

   /* ------- EXECUTOR----------- */

  /* Two handles: cmd_vel subscription + dummy timer. */
  RCCHECK(rclc_executor_init(
      &rte_state.executor,
      &rte_state.support.context,
      2U,
      &rte_state.allocator));

  RCCHECK(rclc_executor_add_subscription(
      &rte_state.executor,
      &rte_state.subs,
      &rte_sub_msg,
      &rte_sub_callback,
      ON_NEW_DATA));

  RCCHECK(rclc_timer_init_default(
      &rte_timer,
      &rte_state.support,
      RCL_MS_TO_NS(1000U),
      rte_timer_callback));

  RCCHECK(rclc_executor_add_timer(
      &rte_state.executor,
      &rte_timer));

  g_ros_ready = true;

}


void rte_SpinOnce(void)
{
  if (g_ros_ready == false)
  {
    return;
  }

  if ((g_rcl_mutex != NULL) &&
      (xSemaphoreTake(g_rcl_mutex, pdMS_TO_TICKS(2U)) == pdTRUE))
  {
    /* Transport-only spin: no callbacks should block here. */
    RCSOFTCHECK(rclc_executor_spin_some(
        &rte_state.executor,
        RCL_MS_TO_NS(2U)));
    (void)xSemaphoreGive(g_rcl_mutex);
  }

  g_spin_count++;
  if ((g_spin_count % 200U) == 0U)
  {
    digitalWrite(LED_WHITE_PIN, !digitalRead(LED_WHITE_PIN));
  }
}

void rte_PublishImu(void)
{
  rte_ErrorLed_Update();
  static uint32_t last_seq = 0U;
  imu_data_t imu;
  if (!Hal_Imu_GetLatest(&imu))
  {
    g_imu_pub_miss++;
    rte_ErrorLed_Pulse();
    return;
  }
  if (imu.valid == false)
  {
    g_imu_pub_miss++;
    rte_ErrorLed_Pulse();
    return;
  }
  if (imu.seq == last_seq)
  {
    g_imu_pub_miss++;
    rte_ErrorLed_Pulse();
    return;
  }
  last_seq = imu.seq;

  rte_imu_msg.orientation.w = imu.qw;
  rte_imu_msg.orientation.x = imu.qx;
  rte_imu_msg.orientation.y = imu.qy;
  rte_imu_msg.orientation.z = imu.qz;

  rte_imu_msg.angular_velocity.x = imu.gx;
  rte_imu_msg.angular_velocity.y = imu.gy;
  rte_imu_msg.angular_velocity.z = imu.gz;

  rte_imu_msg.linear_acceleration.x = imu.ax;
  rte_imu_msg.linear_acceleration.y = imu.ay;
  rte_imu_msg.linear_acceleration.z = imu.az;

  for (size_t i = 0U; i < 9U; ++i)
  {
    rte_imu_msg.orientation_covariance[i] = -1.0F;
    rte_imu_msg.angular_velocity_covariance[i] = -1.0F;
    rte_imu_msg.linear_acceleration_covariance[i] = -1.0F;
  }

  if ((g_rcl_mutex != NULL) &&
      (xSemaphoreTake(g_rcl_mutex, pdMS_TO_TICKS(2U)) == pdTRUE))
  {
    /* Publish guarded by rcl mutex. */
    if (rcl_publish(&rte_pub_imu, &rte_imu_msg, NULL) == RCL_RET_OK)
    {
      g_imu_pub_ok++;
    }
    else
    {
      g_imu_pub_miss++;
      rte_ErrorLed_Pulse();
    }
    (void)xSemaphoreGive(g_rcl_mutex);
  }
  else
  {
    g_imu_pub_miss++;
    rte_ErrorLed_Pulse();
  }
}

void rte_PublishBme(void)
{
  rte_ErrorLed_Update();
  static uint32_t last_seq = 0U;
  bme_data_t bme;
  if (!Hal_Alt_GetLatest(&bme))
  {
    g_bme_pub_miss++;
    rte_ErrorLed_Pulse();
    return;
  }
  if (bme.valid == false)
  {
    g_bme_pub_miss++;
    rte_ErrorLed_Pulse();
    return;
  }
  if (bme.seq == last_seq)
  {
    g_bme_pub_miss++;
    rte_ErrorLed_Pulse();
    return;
  }
  last_seq = bme.seq;

  rte_temp_msg.temperature = bme.temp_c;
  rte_temp_msg.variance = 0.0F;

  rte_press_msg.fluid_pressure = bme.press_pa;
  rte_press_msg.variance = 0.0F;

  rte_hum_msg.relative_humidity = bme.hum_pct / 100.0F;
  rte_hum_msg.variance = 0.0F;

  if ((g_rcl_mutex != NULL) &&
      (xSemaphoreTake(g_rcl_mutex, pdMS_TO_TICKS(2U)) == pdTRUE))
  {
    /* Publish guarded by rcl mutex. */
    if (rcl_publish(&rte_pub_temp, &rte_temp_msg, NULL) == RCL_RET_OK)
    {
      g_bme_pub_ok++;
    }
    else
    {
      g_bme_pub_miss++;
      rte_ErrorLed_Pulse();
    }
    (void)rcl_publish(&rte_pub_press, &rte_press_msg, NULL);
    (void)rcl_publish(&rte_pub_hum, &rte_hum_msg, NULL);
    (void)xSemaphoreGive(g_rcl_mutex);
  }
  else
  {
    g_bme_pub_miss++;
    rte_ErrorLed_Pulse();
  }
}
