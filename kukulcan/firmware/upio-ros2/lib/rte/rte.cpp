/**
 * @file rte.cpp
 * @brief Micro-ROS runtime environment implementation.
 */
#include "config.h"
#include "rte.h"

#include <stdio.h>
#include <string.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/string.h>


static constexpr const char * RTE_NODE           = "kukulcan";
static constexpr const char * RTE_TOPIC_PUB      = "app_hello";
static constexpr const char * RTE_TOPIC_SUB      = "cmd_vel";

static constexpr uint32_t     RTE_SPIN_PERIOD_MS = 10U;

/* RTE state for node, executor, and entities. */
static MicroRosState rte_state;

/* Publisher state for app_hello. */
static std_msgs__msg__String rte_pub_msg;
static char                  rte_pub_buffer[32];
static uint32_t              rte_counter = 0U;

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

  /* Publisher: "app_hello" */
  RCCHECK(rclc_publisher_init_default(
      &rte_state.publis,
      &rte_state.node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      RTE_TOPIC_PUB));

  /* Bind message buffer to the outgoing String message. */
  rte_pub_msg.data.data     = rte_pub_buffer;
  rte_pub_msg.data.size     = 0U;
  rte_pub_msg.data.capacity = sizeof(rte_pub_buffer);

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
 * @brief Publish "Hello N" and spin the executor.
 */
void rte_Run(void)
{
  int    written;
  size_t len;

  written = snprintf(
      rte_pub_buffer,
      sizeof(rte_pub_buffer),
      "Hello %lu",
      static_cast<unsigned long>(rte_counter));

  if (written > 0)
  {
    len = static_cast<size_t>(written);
    if (len >= sizeof(rte_pub_buffer))
    {
      len = sizeof(rte_pub_buffer) - 1U;
      rte_pub_buffer[len] = '\0';
    }
    rte_pub_msg.data.size = len;
  }

  RCCHECK(rcl_publish(
      &rte_state.publis,
      &rte_pub_msg,
      NULL));

  rte_counter++;

  RCCHECK(rclc_executor_spin_some(
      &rte_state.executor,
      RCL_MS_TO_NS(RTE_SPIN_PERIOD_MS)));
}
