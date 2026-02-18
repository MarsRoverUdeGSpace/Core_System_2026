/**
 * @file rte.cpp
 * @brief Micro-ROS runtime environment implementation.
 */
#include "config.h"
#include "rte.h"
#include "imu.h"
#include "alt.h"
#include "motors.h"
#include "led_status.h"

#include <stdio.h>
#include <string.h>
#include <freertos/semphr.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/u_int8.h>
#include <sensor_msgs/msg/temperature.h>
#include <sensor_msgs/msg/fluid_pressure.h>
#include <sensor_msgs/msg/relative_humidity.h>
#include <sensor_msgs/msg/imu.h>

static constexpr const char * RTE_NODE       = "kukulcan";
static constexpr const char * RTE_TOPIC_TEMP = "sensors/bme280/temperature";
static constexpr const char * RTE_TOPIC_PRESS = "sensors/bme280/pressure";
static constexpr const char * RTE_TOPIC_HUM  = "sensors/bme280/humidity";
static constexpr const char * RTE_TOPIC_IMU  = "imu/data";
static constexpr const char * RTE_TOPIC_SUB  = "cmd_vel";
static constexpr const char * RTE_TOPIC_MODE = "mode";

static constexpr TickType_t RTE_PING_PERIOD_TICKS = pdMS_TO_TICKS(100U);
static constexpr uint8_t    RTE_PING_FAIL_LIMIT = 3U;

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
static rcl_subscription_t rte_mode_sub;
static std_msgs__msg__UInt8 rte_mode_msg;

/* Guard all rcl/rmw calls; micro-ROS stack is not thread-safe. */
static SemaphoreHandle_t g_rcl_mutex = NULL;

static volatile bool     g_ros_ready = false;
static volatile uint32_t g_spin_count = 0U;

static uint32_t g_imu_pub_ok = 0U;
static uint32_t g_imu_pub_miss = 0U;
static uint32_t g_bme_pub_ok = 0U;
static uint32_t g_bme_pub_miss = 0U;

static TickType_t g_err_led_until = 0U;
static TickType_t g_last_ping_tick = 0U;
static uint8_t g_ping_fail_count = 0U;
static bool g_link_down = false;
static TickType_t g_link_blink_tick = 0U;

static bool rte_MicroRos_Init(void);
static void rte_MicroRos_Deinit(void);

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

static void rte_LinkLed_Update(void)
{
  if (g_link_down == false)
  {
    return;
  }

  const TickType_t now = xTaskGetTickCount();
  if ((now - g_link_blink_tick) >= pdMS_TO_TICKS(100U))
  {
    g_link_blink_tick = now;
    digitalWrite(LED_RED_PIN, !digitalRead(LED_RED_PIN));
  }
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
 * @brief Subscriber callback for status mode control.
 *
 * Mode mapping:
 * 0 teleop, 1 autonomous, 2 goal reached, 3 safe fault stop, 4 fabulous, 5 teleop arm.
 */
static void rte_mode_callback(const void * msgin)
{
  const std_msgs__msg__UInt8 * msg =
      static_cast<const std_msgs__msg__UInt8 *>(msgin);
  if (msg == NULL)
  {
    return;
  }

  switch (msg->data)
  {
    case 0U:
      Hal_LedStatus_SetMode(LED_STATUS_MODE_TELEOP);
      break;
    case 1U:
      Hal_LedStatus_SetMode(LED_STATUS_MODE_AUTONOMOUS);
      break;
    case 2U:
      Hal_LedStatus_SetMode(LED_STATUS_MODE_GOAL_REACHED);
      break;
    case 3U:
      Hal_LedStatus_SetMode(LED_STATUS_MODE_SAFE_FAULT_STOP);
      break;
    case 4U:
      Hal_LedStatus_SetMode(LED_STATUS_MODE_FABULOUS);
      break;
    case 5U:
      Hal_LedStatus_SetMode(LED_STATUS_MODE_TELEOP_ARM);
      break;
    default:
      Hal_LedStatus_SetMode(LED_STATUS_MODE_FABULOUS);
      break;
  }
}

/**
 * @brief Initialize transport and micro-ROS runtime.
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

  (void)rte_MicroRos_Init();
}

static bool rte_MicroRos_Init(void)
{
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

  /* ------ SUBSCRIBERS ------ */

  RCCHECK(rclc_subscription_init_best_effort(
      &rte_state.subs,
      &rte_state.node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      RTE_TOPIC_SUB));

  RCCHECK(rclc_subscription_init_best_effort(
      &rte_mode_sub,
      &rte_state.node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
      RTE_TOPIC_MODE));

  /* ------- EXECUTOR ------- */

  /* Three handles: cmd_vel subscription + mode subscription + 1 Hz timer. */
  RCCHECK(rclc_executor_init(
      &rte_state.executor,
      &rte_state.support.context,
      3U,
      &rte_state.allocator));

  RCCHECK(rclc_executor_add_subscription(
      &rte_state.executor,
      &rte_state.subs,
      &rte_sub_msg,
      &rte_sub_callback,
      ON_NEW_DATA));

  RCCHECK(rclc_executor_add_subscription(
      &rte_state.executor,
      &rte_mode_sub,
      &rte_mode_msg,
      &rte_mode_callback,
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
  return true;
}

static void rte_MicroRos_Deinit(void)
{
  if (g_ros_ready == false)
  {
    return;
  }
  g_ros_ready = false;

  (void)rclc_executor_fini(&rte_state.executor);
  (void)rcl_timer_fini(&rte_timer);
  (void)rcl_subscription_fini(&rte_state.subs, &rte_state.node);
  (void)rcl_subscription_fini(&rte_mode_sub, &rte_state.node);
  (void)rcl_publisher_fini(&rte_pub_temp, &rte_state.node);
  (void)rcl_publisher_fini(&rte_pub_press, &rte_state.node);
  (void)rcl_publisher_fini(&rte_pub_hum, &rte_state.node);
  (void)rcl_publisher_fini(&rte_pub_imu, &rte_state.node);
  (void)rcl_node_fini(&rte_state.node);
  (void)rclc_support_fini(&rte_state.support);
}

void rte_SpinOnce(void)
{
  const TickType_t now = xTaskGetTickCount();
  if ((now - g_last_ping_tick) >= RTE_PING_PERIOD_TICKS)
  {
    g_last_ping_tick = now;
    if (rmw_uros_ping_agent(20, 1) == RMW_RET_OK)
    {
      g_ping_fail_count = 0U;
      if (g_link_down)
      {
        g_link_down = false;
        digitalWrite(LED_RED_PIN, LOW);
      }
      if (g_ros_ready == false)
      {
        (void)rte_MicroRos_Init();
      }
    }
    else
    {
      if (g_ping_fail_count < RTE_PING_FAIL_LIMIT)
      {
        g_ping_fail_count++;
      }
      if ((g_ping_fail_count >= RTE_PING_FAIL_LIMIT) && (g_ros_ready == true))
      {
        rte_MicroRos_Deinit();
        Hal_Motor_SetTwist(0.0F, 0.0F);
        g_link_down = true;
        g_link_blink_tick = now;
        digitalWrite(LED_WHITE_PIN, LOW);
      }
    }
  }

  rte_LinkLed_Update();

  if (g_ros_ready == false)
  {
    return;
  }

  if ((g_rcl_mutex != NULL) &&
      (xSemaphoreTake(g_rcl_mutex, pdMS_TO_TICKS(2U)) == pdTRUE))
  {
    /* Transport-only spin: no callbacks should block here. */
    (void)rclc_executor_spin_some(
        &rte_state.executor,
        RCL_MS_TO_NS(2U));
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
  if (g_ros_ready == false)
  {
    return;
  }
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
  if (g_ros_ready == false)
  {
    return;
  }
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
