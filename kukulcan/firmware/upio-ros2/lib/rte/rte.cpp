/**
 * @file rte.cpp
 * @brief Micro-ROS runtime environment implementation.
 */
#include "config.h"
#include "rte.h"
#include "alt.h"

#include <stdio.h>

#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/temperature.h>
#include <sensor_msgs/msg/fluid_pressure.h>
#include <sensor_msgs/msg/relative_humidity.h>


static constexpr const char * RTE_NODE           = "kukulcan";
static constexpr const char * RTE_TOPIC_TEMP     = "sensors/bme280/temperature";
static constexpr const char * RTE_TOPIC_PRESS    = "sensors/bme280/pressure";
static constexpr const char * RTE_TOPIC_HUM      = "sensors/bme280/humidity";
static constexpr const char * RTE_TOPIC_SUB      = "cmd_vel";

static constexpr uint32_t     RTE_SPIN_PERIOD_MS = 10U;
static constexpr TickType_t   RTE_BME_PUB_PERIOD_TICKS = pdMS_TO_TICKS(1000U);

/* RTE state for node, executor, and entities. */
static MicroRosState rte_state;

/* Publisher state for BME280. */
static rcl_publisher_t rte_pub_temp;
static rcl_publisher_t rte_pub_press;
static rcl_publisher_t rte_pub_hum;

static sensor_msgs__msg__Temperature      rte_temp_msg;
static sensor_msgs__msg__FluidPressure    rte_press_msg;
static sensor_msgs__msg__RelativeHumidity rte_hum_msg;

/* Subscriber state for cmd_vel. */
static geometry_msgs__msg__Twist rte_sub_msg;

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

   /* ------SUBSCRIBERS-------- */

  // Subscriber: cmd_vel (Twist).
  RCCHECK(rclc_subscription_init_default(
      &rte_state.subs,
      &rte_state.node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      RTE_TOPIC_SUB));

   /* ------- EXECUTOR----------- */

  RCCHECK(rclc_executor_init(
      &rte_state.executor,
      &rte_state.support.context,
      1U,
      &rte_state.allocator));

  RCCHECK(rclc_executor_add_subscription(
      &rte_state.executor,
      &rte_state.subs,
      &rte_sub_msg,
      &rte_sub_callback,
      ON_NEW_DATA));

}


/**
 * @brief Publish Topics and spin the executor.
 */
void rte_Run(void)
{
  static TickType_t last_bme_pub_tick = 0U;
  const TickType_t now_tick = xTaskGetTickCount();

  if ((now_tick - last_bme_pub_tick) >= RTE_BME_PUB_PERIOD_TICKS)
  {
    float temp_c = 0.0F;
    float press_pa = 0.0F;
    float hum_pct = 0.0F;

    last_bme_pub_tick = now_tick;

    if (Hal_Alt_Read(&temp_c, &press_pa, &hum_pct))
    {
      rte_temp_msg.temperature = temp_c;
      rte_temp_msg.variance = 0.0F;

      rte_press_msg.fluid_pressure = press_pa;
      rte_press_msg.variance = 0.0F;

      rte_hum_msg.relative_humidity = hum_pct / 100.0F;
      rte_hum_msg.variance = 0.0F;

      RCSOFTCHECK(rcl_publish(
          &rte_pub_temp,
          &rte_temp_msg,
          NULL));

      RCSOFTCHECK(rcl_publish(
          &rte_pub_press,
          &rte_press_msg,
          NULL));

      RCSOFTCHECK(rcl_publish(
          &rte_pub_hum,
          &rte_hum_msg,
          NULL));
    }
  }

  RCSOFTCHECK(rclc_executor_spin_some(
      &rte_state.executor,
      RCL_MS_TO_NS(RTE_SPIN_PERIOD_MS)));
}
