/**
 * @file rte.cpp
 * @brief Micro-ROS runtime environment implementation.
 */
#include "config.h"
#include "rte.h"
#include "imu.h"
#include "alt.h"
#include "enc.h"
#include "gnss.h"
#include "motors.h"
#include "led_status.h"
#include "odom.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <esp_timer.h>
#include <freertos/semphr.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microros/timing.h>
#include <rmw_microros/time_sync.h>

#include <builtin_interfaces/msg/time.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <std_msgs/msg/u_int8.h>
#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/temperature.h>
#include <sensor_msgs/msg/fluid_pressure.h>
#include <sensor_msgs/msg/relative_humidity.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <sensor_msgs/msg/nav_sat_status.h>

static constexpr const char * RTE_NODE       = "kukulcan";
static constexpr const char * RTE_TOPIC_TEMP = "sensors/bme280/temperature";
static constexpr const char * RTE_TOPIC_PRESS = "sensors/bme280/pressure";
static constexpr const char * RTE_TOPIC_HUM  = "sensors/bme280/humidity";
static constexpr const char * RTE_TOPIC_IMU  = "sensors/bno055/imu/data";
static constexpr const char * RTE_TOPIC_MAG  = "sensors/bno055/mag";
static constexpr const char * RTE_TOPIC_GNSS = "sensors/gnss/fix";
static constexpr const char * RTE_TOPIC_ODOM = "odom";
static constexpr const char * RTE_TOPIC_ENC_LEFT_TICKS  = "sensors/roboclaw/encoders/left_m1/ticks";
static constexpr const char * RTE_TOPIC_ENC_RIGHT_TICKS = "sensors/roboclaw/encoders/right_m2/ticks";
static constexpr const char * RTE_TOPIC_ENC_LEFT_QPPS   = "sensors/roboclaw/encoders/left_m1/qpps";
static constexpr const char * RTE_TOPIC_ENC_RIGHT_QPPS  = "sensors/roboclaw/encoders/right_m2/qpps";
static constexpr const char * RTE_TOPIC_SUB  = "cmd_vel";
static constexpr const char * RTE_TOPIC_MODE = "mode";

static constexpr TickType_t RTE_PING_PERIOD_TICKS = pdMS_TO_TICKS(1000U);
static constexpr TickType_t RTE_TIME_SYNC_PERIOD_TICKS = pdMS_TO_TICKS(1000U);
static constexpr uint8_t    RTE_PING_FAIL_LIMIT = 3U;
static constexpr int        RTE_RELIABLE_PUBLISH_TIMEOUT_MS = 5;
static constexpr float      RTE_IMU_ORIENTATION_VARIANCE = 0.05F;
static constexpr float      RTE_IMU_ANGULAR_VELOCITY_VARIANCE = 0.02F;
static constexpr float      RTE_IMU_LINEAR_ACCELERATION_VARIANCE = 0.10F;
static constexpr float      RTE_GNSS_HDOP_UERE_M = 5.0F;
static constexpr float      RTE_GNSS_VERTICAL_SIGMA_SCALE = 2.0F;
static constexpr double     RTE_ODOM_POSE_XY_VARIANCE = 0.02;
static constexpr double     RTE_ODOM_POSE_Z_VARIANCE = 1.0e6;
static constexpr double     RTE_ODOM_POSE_ROLL_PITCH_VARIANCE = 1.0e6;
static constexpr double     RTE_ODOM_POSE_YAW_VARIANCE = 0.05;
static constexpr double     RTE_ODOM_TWIST_X_VARIANCE = 0.05;
static constexpr double     RTE_ODOM_TWIST_Y_VARIANCE = 0.05;
static constexpr double     RTE_ODOM_TWIST_Z_VARIANCE = 1.0e6;
static constexpr double     RTE_ODOM_TWIST_ROLL_PITCH_VARIANCE = 1.0e6;
static constexpr double     RTE_ODOM_TWIST_YAW_VARIANCE = 0.08;

/* RTE state for node, executor, and entities. */
static MicroRosState rte_state;

/* Publisher state for BME280. */
static rcl_publisher_t rte_pub_temp;
static rcl_publisher_t rte_pub_press;
static rcl_publisher_t rte_pub_hum;
static rcl_publisher_t rte_pub_imu;
static rcl_publisher_t rte_pub_mag;
static rcl_publisher_t rte_pub_gnss;
static rcl_publisher_t rte_pub_odom;
static rcl_publisher_t rte_pub_enc_left_ticks;
static rcl_publisher_t rte_pub_enc_right_ticks;
static rcl_publisher_t rte_pub_enc_left_qpps;
static rcl_publisher_t rte_pub_enc_right_qpps;
static rcl_timer_t     rte_timer;

static sensor_msgs__msg__Temperature      rte_temp_msg;
static sensor_msgs__msg__FluidPressure    rte_press_msg;
static sensor_msgs__msg__RelativeHumidity rte_hum_msg;
static sensor_msgs__msg__Imu              rte_imu_msg;
static sensor_msgs__msg__MagneticField    rte_mag_msg;
static sensor_msgs__msg__NavSatFix        rte_gnss_msg;
static nav_msgs__msg__Odometry            rte_odom_msg;
static std_msgs__msg__Int32               rte_enc_left_ticks_msg;
static std_msgs__msg__Int32               rte_enc_right_ticks_msg;
static std_msgs__msg__Int32               rte_enc_left_qpps_msg;
static std_msgs__msg__Int32               rte_enc_right_qpps_msg;
static char                               rte_imu_frame_id[16];
static char                               rte_mag_frame_id[16];
static char                               rte_gnss_frame_id[16];
static char                               rte_odom_frame_id[16];
static char                               rte_odom_child_frame_id[20];

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
static uint32_t g_mag_pub_ok = 0U;
static uint32_t g_mag_pub_miss = 0U;
static uint32_t g_bme_pub_ok = 0U;
static uint32_t g_bme_pub_miss = 0U;
static uint32_t g_gnss_pub_ok = 0U;
static uint32_t g_gnss_pub_miss = 0U;
static uint32_t g_odom_pub_ok = 0U;
static uint32_t g_odom_pub_miss = 0U;
static uint32_t g_enc_pub_ok = 0U;
static uint32_t g_enc_pub_miss = 0U;

static TickType_t g_err_led_until = 0U;
static TickType_t g_last_ping_tick = 0U;
static TickType_t g_last_time_sync_tick = 0U;
static uint8_t g_ping_fail_count = 0U;
static bool g_link_down = false;
static TickType_t g_link_blink_tick = 0U;

/* Stamp cache bridges synchronized ROS epoch time to monotonic MCU time. */
static portMUX_TYPE g_time_cache_mux = portMUX_INITIALIZER_UNLOCKED;
static bool g_time_synced = false;
static int64_t g_epoch_anchor_ns = 0;
static int64_t g_monotonic_anchor_us = 0;

static bool rte_MicroRos_Init(void);
static void rte_MicroRos_Deinit(void);

static void rte_BoundReliablePublishWait(rcl_publisher_t * publisher)
{
  rmw_publisher_t * rmw_publisher = rcl_publisher_get_rmw_handle(publisher);
  if ((rmw_publisher == NULL) ||
      (rmw_uros_set_publisher_session_timeout(
          rmw_publisher,
          RTE_RELIABLE_PUBLISH_TIMEOUT_MS) != RMW_RET_OK))
  {
    error_loop();
  }
}

static void rte_TimeCache_Clear(void)
{
  portENTER_CRITICAL(&g_time_cache_mux);
  g_time_synced = false;
  g_epoch_anchor_ns = 0;
  g_monotonic_anchor_us = 0;
  portEXIT_CRITICAL(&g_time_cache_mux);
}

static bool rte_TimeCache_IsSynced(void)
{
  bool synced;
  portENTER_CRITICAL(&g_time_cache_mux);
  synced = g_time_synced;
  portEXIT_CRITICAL(&g_time_cache_mux);
  return synced;
}

static bool rte_TimeSync_Update(void)
{
  if (g_rcl_mutex == NULL)
  {
    return false;
  }

  bool synced = false;
  int64_t epoch_ns = 0;
  int64_t monotonic_us = 0;
  if (xSemaphoreTake(g_rcl_mutex, pdMS_TO_TICKS(20U)) == pdTRUE)
  {
    synced = (rmw_uros_sync_session(100) == RMW_RET_OK) &&
             rmw_uros_epoch_synchronized();
    if (synced)
    {
      epoch_ns = rmw_uros_epoch_nanos();
      monotonic_us = esp_timer_get_time();
      synced = (epoch_ns > 0);
    }
    (void)xSemaphoreGive(g_rcl_mutex);
  }

  if (synced)
  {
    portENTER_CRITICAL(&g_time_cache_mux);
    g_epoch_anchor_ns = epoch_ns;
    g_monotonic_anchor_us = monotonic_us;
    g_time_synced = true;
    portEXIT_CRITICAL(&g_time_cache_mux);
  }
  else
  {
    rte_TimeCache_Clear();
  }
  return synced;
}

static void rte_SetDiagonal3(double covariance[9], const double x, const double y, const double z)
{
  for (size_t i = 0U; i < 9U; ++i)
  {
    covariance[i] = 0.0;
  }

  covariance[0] = x;
  covariance[4] = y;
  covariance[8] = z;
}

static void rte_SetDiagonal6(double covariance[36],
                             const double x,
                             const double y,
                             const double z,
                             const double roll,
                             const double pitch,
                             const double yaw)
{
  for (size_t i = 0U; i < 36U; ++i)
  {
    covariance[i] = 0.0;
  }

  covariance[0] = x;
  covariance[7] = y;
  covariance[14] = z;
  covariance[21] = roll;
  covariance[28] = pitch;
  covariance[35] = yaw;
}

static void rte_SetYawQuaternion(const double yaw_rad, geometry_msgs__msg__Quaternion * quat)
{
  if (quat == NULL)
  {
    return;
  }

  quat->x = 0.0;
  quat->y = 0.0;
  quat->z = sin(yaw_rad * 0.5);
  quat->w = cos(yaw_rad * 0.5);
}

static bool rte_FillStamp(builtin_interfaces__msg__Time * stamp)
{
  if (stamp == NULL)
  {
    return false;
  }

  bool synced;
  int64_t epoch_anchor_ns;
  int64_t monotonic_anchor_us;
  portENTER_CRITICAL(&g_time_cache_mux);
  synced = g_time_synced;
  epoch_anchor_ns = g_epoch_anchor_ns;
  monotonic_anchor_us = g_monotonic_anchor_us;
  portEXIT_CRITICAL(&g_time_cache_mux);

  if (!synced)
  {
    return false;
  }

  const int64_t monotonic_us = esp_timer_get_time();
  const int64_t epoch_ns =
      epoch_anchor_ns + ((monotonic_us - monotonic_anchor_us) * 1000LL);
  if (epoch_ns <= 0)
  {
    return false;
  }

  stamp->sec = static_cast<int32_t>(epoch_ns / 1000000000LL);
  stamp->nanosec = static_cast<uint32_t>(epoch_ns % 1000000000LL);
  return true;
}

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
  rte_BoundReliablePublishWait(&rte_pub_temp);

  RCCHECK(rclc_publisher_init_default(
      &rte_pub_press,
      &rte_state.node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, FluidPressure),
      RTE_TOPIC_PRESS));
  rte_BoundReliablePublishWait(&rte_pub_press);

  RCCHECK(rclc_publisher_init_default(
      &rte_pub_hum,
      &rte_state.node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, RelativeHumidity),
      RTE_TOPIC_HUM));
  rte_BoundReliablePublishWait(&rte_pub_hum);

  RCCHECK(rclc_publisher_init_best_effort(
      &rte_pub_imu,
      &rte_state.node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      RTE_TOPIC_IMU));

  RCCHECK(rclc_publisher_init_best_effort(
      &rte_pub_mag,
      &rte_state.node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
      RTE_TOPIC_MAG));

  RCCHECK(rclc_publisher_init_best_effort(
      &rte_pub_gnss,
      &rte_state.node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix),
      RTE_TOPIC_GNSS));

  /*
   * Odometry exceeds the 512-byte best-effort XRCE MTU once both covariance
   * matrices are serialized. Use the reliable output stream for fragmentation.
   */
  RCCHECK(rclc_publisher_init_default(
      &rte_pub_odom,
      &rte_state.node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
      RTE_TOPIC_ODOM));
  rte_BoundReliablePublishWait(&rte_pub_odom);

  RCCHECK(rclc_publisher_init_best_effort(
      &rte_pub_enc_left_ticks,
      &rte_state.node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      RTE_TOPIC_ENC_LEFT_TICKS));

  RCCHECK(rclc_publisher_init_best_effort(
      &rte_pub_enc_right_ticks,
      &rte_state.node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      RTE_TOPIC_ENC_RIGHT_TICKS));

  RCCHECK(rclc_publisher_init_best_effort(
      &rte_pub_enc_left_qpps,
      &rte_state.node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      RTE_TOPIC_ENC_LEFT_QPPS));

  RCCHECK(rclc_publisher_init_best_effort(
      &rte_pub_enc_right_qpps,
      &rte_state.node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      RTE_TOPIC_ENC_RIGHT_QPPS));

  rte_imu_msg.header.frame_id.data = rte_imu_frame_id;
  rte_imu_msg.header.frame_id.size = 0U;
  rte_imu_msg.header.frame_id.capacity = sizeof(rte_imu_frame_id);
  (void)snprintf(rte_imu_frame_id, sizeof(rte_imu_frame_id), "imu_link");
  rte_imu_msg.header.frame_id.size = strlen(rte_imu_frame_id);

  rte_mag_msg.header.frame_id.data = rte_mag_frame_id;
  rte_mag_msg.header.frame_id.size = 0U;
  rte_mag_msg.header.frame_id.capacity = sizeof(rte_mag_frame_id);
  (void)snprintf(rte_mag_frame_id, sizeof(rte_mag_frame_id), "imu_link");
  rte_mag_msg.header.frame_id.size = strlen(rte_mag_frame_id);

  rte_gnss_msg.header.frame_id.data = rte_gnss_frame_id;
  rte_gnss_msg.header.frame_id.size = 0U;
  rte_gnss_msg.header.frame_id.capacity = sizeof(rte_gnss_frame_id);
  (void)snprintf(rte_gnss_frame_id, sizeof(rte_gnss_frame_id), "gnss_link");
  rte_gnss_msg.header.frame_id.size = strlen(rte_gnss_frame_id);

  rte_odom_msg.header.frame_id.data = rte_odom_frame_id;
  rte_odom_msg.header.frame_id.size = 0U;
  rte_odom_msg.header.frame_id.capacity = sizeof(rte_odom_frame_id);
  (void)snprintf(rte_odom_frame_id, sizeof(rte_odom_frame_id), "odom");
  rte_odom_msg.header.frame_id.size = strlen(rte_odom_frame_id);

  rte_odom_msg.child_frame_id.data = rte_odom_child_frame_id;
  rte_odom_msg.child_frame_id.size = 0U;
  rte_odom_msg.child_frame_id.capacity = sizeof(rte_odom_child_frame_id);
  (void)snprintf(rte_odom_child_frame_id, sizeof(rte_odom_child_frame_id), "base_footprint");
  rte_odom_msg.child_frame_id.size = strlen(rte_odom_child_frame_id);

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
  g_last_time_sync_tick = xTaskGetTickCount();
  (void)rte_TimeSync_Update();
  return true;
}

static void rte_MicroRos_Deinit(void)
{
  if (g_ros_ready == false)
  {
    return;
  }
  g_ros_ready = false;
  rte_TimeCache_Clear();

  (void)rclc_executor_fini(&rte_state.executor);
  (void)rcl_timer_fini(&rte_timer);
  (void)rcl_subscription_fini(&rte_state.subs, &rte_state.node);
  (void)rcl_subscription_fini(&rte_mode_sub, &rte_state.node);
  (void)rcl_publisher_fini(&rte_pub_temp, &rte_state.node);
  (void)rcl_publisher_fini(&rte_pub_press, &rte_state.node);
  (void)rcl_publisher_fini(&rte_pub_hum, &rte_state.node);
  (void)rcl_publisher_fini(&rte_pub_imu, &rte_state.node);
  (void)rcl_publisher_fini(&rte_pub_mag, &rte_state.node);
  (void)rcl_publisher_fini(&rte_pub_gnss, &rte_state.node);
  (void)rcl_publisher_fini(&rte_pub_odom, &rte_state.node);
  (void)rcl_publisher_fini(&rte_pub_enc_left_ticks, &rte_state.node);
  (void)rcl_publisher_fini(&rte_pub_enc_right_ticks, &rte_state.node);
  (void)rcl_publisher_fini(&rte_pub_enc_left_qpps, &rte_state.node);
  (void)rcl_publisher_fini(&rte_pub_enc_right_qpps, &rte_state.node);
  (void)rcl_node_fini(&rte_state.node);
  (void)rclc_support_fini(&rte_state.support);
}

void rte_SpinOnce(void)
{
  const TickType_t now = xTaskGetTickCount();
  if ((now - g_last_ping_tick) >= RTE_PING_PERIOD_TICKS)
  {
    g_last_ping_tick = now;
    bool ping_attempted = false;
    bool ping_ok = false;
    bool reconnect = false;
    if ((g_rcl_mutex != NULL) &&
        (xSemaphoreTake(g_rcl_mutex, 0U) == pdTRUE))
    {
      ping_attempted = true;
      ping_ok = (rmw_uros_ping_agent(20, 1) == RMW_RET_OK);
      if (ping_ok)
      {
        g_ping_fail_count = 0U;
        if (g_link_down)
        {
          g_link_down = false;
          digitalWrite(LED_RED_PIN, LOW);
        }
        reconnect = (g_ros_ready == false);
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
          Hal_Motor_Stop();
          g_link_down = true;
          g_link_blink_tick = now;
          digitalWrite(LED_WHITE_PIN, LOW);
        }
      }
      (void)xSemaphoreGive(g_rcl_mutex);
    }

    if (ping_attempted && ping_ok && reconnect)
    {
      (void)rte_MicroRos_Init();
    }
  }

  rte_LinkLed_Update();

  if (g_ros_ready == false)
  {
    return;
  }

  if (((now - g_last_time_sync_tick) >= RTE_TIME_SYNC_PERIOD_TICKS) &&
      (rte_TimeCache_IsSynced() == false))
  {
    g_last_time_sync_tick = now;
    (void)rte_TimeSync_Update();
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

  if (!rte_FillStamp(&rte_imu_msg.header.stamp))
  {
    g_imu_pub_miss++;
    return;
  }

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

  rte_SetDiagonal3(
      rte_imu_msg.orientation_covariance,
      RTE_IMU_ORIENTATION_VARIANCE,
      RTE_IMU_ORIENTATION_VARIANCE,
      RTE_IMU_ORIENTATION_VARIANCE);
  rte_SetDiagonal3(
      rte_imu_msg.angular_velocity_covariance,
      RTE_IMU_ANGULAR_VELOCITY_VARIANCE,
      RTE_IMU_ANGULAR_VELOCITY_VARIANCE,
      RTE_IMU_ANGULAR_VELOCITY_VARIANCE);
  rte_SetDiagonal3(
      rte_imu_msg.linear_acceleration_covariance,
      RTE_IMU_LINEAR_ACCELERATION_VARIANCE,
      RTE_IMU_LINEAR_ACCELERATION_VARIANCE,
      RTE_IMU_LINEAR_ACCELERATION_VARIANCE);

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

void rte_PublishEncoders(void)
{
  if (g_ros_ready == false)
  {
    return;
  }
  rte_ErrorLed_Update();
  static uint32_t last_seq = 0U;

  enc_data_t enc;
  if (!Hal_Enc_GetLatest(&enc))
  {
    g_enc_pub_miss++;
    rte_ErrorLed_Pulse();
    return;
  }
  if (enc.valid == false)
  {
    g_enc_pub_miss++;
    rte_ErrorLed_Pulse();
    return;
  }
  if (enc.seq == last_seq)
  {
    g_enc_pub_miss++;
    rte_ErrorLed_Pulse();
    return;
  }
  last_seq = enc.seq;

  rte_enc_left_ticks_msg.data = enc.left_ticks;
  rte_enc_right_ticks_msg.data = enc.right_ticks;
  if ((g_rcl_mutex != NULL) &&
      (xSemaphoreTake(g_rcl_mutex, pdMS_TO_TICKS(2U)) == pdTRUE))
  {
    const bool left_ticks_ok =
        (rcl_publish(&rte_pub_enc_left_ticks, &rte_enc_left_ticks_msg, NULL) == RCL_RET_OK);
    const bool right_ticks_ok =
        (rcl_publish(&rte_pub_enc_right_ticks, &rte_enc_right_ticks_msg, NULL) == RCL_RET_OK);
    bool left_qpps_ok = true;
    bool right_qpps_ok = true;
    if (enc.qpps_valid)
    {
      rte_enc_left_qpps_msg.data = enc.left_qpps;
      rte_enc_right_qpps_msg.data = enc.right_qpps;
      left_qpps_ok =
          (rcl_publish(&rte_pub_enc_left_qpps, &rte_enc_left_qpps_msg, NULL) == RCL_RET_OK);
      right_qpps_ok =
          (rcl_publish(&rte_pub_enc_right_qpps, &rte_enc_right_qpps_msg, NULL) == RCL_RET_OK);
    }

    if (left_ticks_ok && right_ticks_ok && left_qpps_ok && right_qpps_ok)
    {
      g_enc_pub_ok++;
    }
    else
    {
      g_enc_pub_miss++;
      rte_ErrorLed_Pulse();
    }
    (void)xSemaphoreGive(g_rcl_mutex);
  }
  else
  {
    g_enc_pub_miss++;
    rte_ErrorLed_Pulse();
  }
}

void rte_PublishMag(void)
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
    g_mag_pub_miss++;
    rte_ErrorLed_Pulse();
    return;
  }
  if (imu.valid == false)
  {
    g_mag_pub_miss++;
    rte_ErrorLed_Pulse();
    return;
  }
  if (imu.seq == last_seq)
  {
    g_mag_pub_miss++;
    rte_ErrorLed_Pulse();
    return;
  }
  last_seq = imu.seq;

  if (!rte_FillStamp(&rte_mag_msg.header.stamp))
  {
    g_mag_pub_miss++;
    return;
  }

  rte_mag_msg.magnetic_field.x = imu.mx;
  rte_mag_msg.magnetic_field.y = imu.my;
  rte_mag_msg.magnetic_field.z = imu.mz;

  for (size_t i = 0U; i < 9U; ++i)
  {
    rte_mag_msg.magnetic_field_covariance[i] = 0.0;
  }

  if ((g_rcl_mutex != NULL) &&
      (xSemaphoreTake(g_rcl_mutex, pdMS_TO_TICKS(2U)) == pdTRUE))
  {
    if (rcl_publish(&rte_pub_mag, &rte_mag_msg, NULL) == RCL_RET_OK)
    {
      g_mag_pub_ok++;
    }
    else
    {
      g_mag_pub_miss++;
      rte_ErrorLed_Pulse();
    }
    (void)xSemaphoreGive(g_rcl_mutex);
  }
  else
  {
    g_mag_pub_miss++;
    rte_ErrorLed_Pulse();
  }
}

void rte_PublishOdom(void)
{
  if (g_ros_ready == false)
  {
    return;
  }
  rte_ErrorLed_Update();
  static uint32_t last_seq = 0U;

  odom_data_t odom;
  if (!App_Odom_GetLatest(&odom))
  {
    g_odom_pub_miss++;
    return;
  }
  if (odom.valid == false)
  {
    g_odom_pub_miss++;
    return;
  }
  if (odom.seq == last_seq)
  {
    g_odom_pub_miss++;
    return;
  }
  last_seq = odom.seq;

  if (!rte_FillStamp(&rte_odom_msg.header.stamp))
  {
    g_odom_pub_miss++;
    return;
  }

  rte_odom_msg.pose.pose.position.x = odom.x_m;
  rte_odom_msg.pose.pose.position.y = odom.y_m;
  rte_odom_msg.pose.pose.position.z = 0.0;
  rte_SetYawQuaternion(odom.yaw_rad, &rte_odom_msg.pose.pose.orientation);

  rte_odom_msg.twist.twist.linear.x = odom.linear_x_mps;
  rte_odom_msg.twist.twist.linear.y = 0.0;
  rte_odom_msg.twist.twist.linear.z = 0.0;
  rte_odom_msg.twist.twist.angular.x = 0.0;
  rte_odom_msg.twist.twist.angular.y = 0.0;
  rte_odom_msg.twist.twist.angular.z = odom.angular_z_radps;

  rte_SetDiagonal6(
      rte_odom_msg.pose.covariance,
      RTE_ODOM_POSE_XY_VARIANCE,
      RTE_ODOM_POSE_XY_VARIANCE,
      RTE_ODOM_POSE_Z_VARIANCE,
      RTE_ODOM_POSE_ROLL_PITCH_VARIANCE,
      RTE_ODOM_POSE_ROLL_PITCH_VARIANCE,
      RTE_ODOM_POSE_YAW_VARIANCE);
  rte_SetDiagonal6(
      rte_odom_msg.twist.covariance,
      RTE_ODOM_TWIST_X_VARIANCE,
      RTE_ODOM_TWIST_Y_VARIANCE,
      RTE_ODOM_TWIST_Z_VARIANCE,
      RTE_ODOM_TWIST_ROLL_PITCH_VARIANCE,
      RTE_ODOM_TWIST_ROLL_PITCH_VARIANCE,
      RTE_ODOM_TWIST_YAW_VARIANCE);

  if ((g_rcl_mutex != NULL) &&
      (xSemaphoreTake(g_rcl_mutex, pdMS_TO_TICKS(2U)) == pdTRUE))
  {
    if (rcl_publish(&rte_pub_odom, &rte_odom_msg, NULL) == RCL_RET_OK)
    {
      g_odom_pub_ok++;
    }
    else
    {
      g_odom_pub_miss++;
      rte_ErrorLed_Pulse();
    }
    (void)xSemaphoreGive(g_rcl_mutex);
  }
  else
  {
    g_odom_pub_miss++;
    rte_ErrorLed_Pulse();
  }
}

void rte_PublishGnss(void)
{
  if (g_ros_ready == false)
  {
    return;
  }
  rte_ErrorLed_Update();

  gnss_data_t gnss;
  if (!Hal_Gnss_GetLatest(&gnss))
  {
    g_gnss_pub_miss++;
    rte_ErrorLed_Pulse();
    return;
  }
  if (gnss.valid == false)
  {
    g_gnss_pub_miss++;
    return;
  }

  if (!rte_FillStamp(&rte_gnss_msg.header.stamp))
  {
    g_gnss_pub_miss++;
    return;
  }

  if (gnss.has_fix)
  {
    rte_gnss_msg.status.status = sensor_msgs__msg__NavSatStatus__STATUS_FIX;
    rte_gnss_msg.latitude = gnss.latitude_deg;
    rte_gnss_msg.longitude = gnss.longitude_deg;
    rte_gnss_msg.altitude = gnss.altitude_m;
  }
  else
  {
    rte_gnss_msg.status.status = sensor_msgs__msg__NavSatStatus__STATUS_NO_FIX;
    rte_gnss_msg.latitude = 0.0;
    rte_gnss_msg.longitude = 0.0;
    rte_gnss_msg.altitude = 0.0;
  }

  rte_gnss_msg.status.service = sensor_msgs__msg__NavSatStatus__SERVICE_GPS;
  if (gnss.has_fix && (gnss.hdop > 0.0F) && isfinite(gnss.hdop))
  {
    const double sigma_xy_m = static_cast<double>(gnss.hdop * RTE_GNSS_HDOP_UERE_M);
    const double sigma_z_m = sigma_xy_m * static_cast<double>(RTE_GNSS_VERTICAL_SIGMA_SCALE);
    rte_SetDiagonal3(
        rte_gnss_msg.position_covariance,
        sigma_xy_m * sigma_xy_m,
        sigma_xy_m * sigma_xy_m,
        sigma_z_m * sigma_z_m);
    rte_gnss_msg.position_covariance_type =
        sensor_msgs__msg__NavSatFix__COVARIANCE_TYPE_DIAGONAL_KNOWN;
  }
  else
  {
    for (size_t i = 0U; i < 9U; ++i)
    {
      rte_gnss_msg.position_covariance[i] = 0.0;
    }
    rte_gnss_msg.position_covariance_type =
        sensor_msgs__msg__NavSatFix__COVARIANCE_TYPE_UNKNOWN;
  }

  if ((g_rcl_mutex != NULL) &&
      (xSemaphoreTake(g_rcl_mutex, pdMS_TO_TICKS(2U)) == pdTRUE))
  {
    if (rcl_publish(&rte_pub_gnss, &rte_gnss_msg, NULL) == RCL_RET_OK)
    {
      g_gnss_pub_ok++;
    }
    else
    {
      g_gnss_pub_miss++;
      rte_ErrorLed_Pulse();
    }
    (void)xSemaphoreGive(g_rcl_mutex);
  }
  else
  {
    g_gnss_pub_miss++;
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
