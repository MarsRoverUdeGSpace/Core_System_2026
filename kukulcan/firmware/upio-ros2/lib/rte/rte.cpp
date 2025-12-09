/**
 * @file rte.cpp
 * @brief micro-ROS runtime environment implementation.
 */

#include "rte.h"
#include "config.h"

#include <std_msgs/msg/string.h>
#include <stdio.h>
#include <string.h>

static constexpr const char * RTE_NODE           = "kukulcan";
static constexpr const char * RTE_TOPIC_PUB      = "app_hello";
static constexpr const char * RTE_TOPIC_SUB      = "app_in";
static constexpr uint32_t     RTE_SPIN_PERIOD_MS = 10U;

/* Internal state */
static MicroRosState rte_state;

/* Publisher state */
static std_msgs__msg__String rte_pub_msg;
static char                  rte_pub_buffer[32];
static uint32_t              rte_counter = 0U;

/* Subscriber state */
static std_msgs__msg__String rte_sub_msg;
static char                  rte_sub_buffer[64];

/**
 * @brief Subscriber callback for "app_in" topic.
 *
 * @param msgin Pointer to received message.
 */
static void rte_sub_callback(const void * msgin)
{
  const std_msgs__msg__String * msg =
      static_cast<const std_msgs__msg__String *>(msgin);

  Serial.print("[RTE] app_in: ");
  Serial.write(msg->data.data, msg->data.size);
  Serial.println();
}

/**
 * @brief Initialize serial transport, allocator, support, node, entities, and executor.
 */
void rte_Init(void)
{
  config_init();
  set_microros_serial_transports(rte_serial);
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

  /* Publisher: "app_hello" */
  RCCHECK(rclc_publisher_init_default(
      &rte_state.publis,
      &rte_state.node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      RTE_TOPIC_PUB));

  rte_pub_msg.data.data     = rte_pub_buffer;
  rte_pub_msg.data.size     = 0U;
  rte_pub_msg.data.capacity = sizeof(rte_pub_buffer);

  /* Subscriber: "app_in" */
  RCCHECK(rclc_subscription_init_default(
      &rte_state.subs,
      &rte_state.node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      RTE_TOPIC_SUB));

  rte_sub_msg.data.data     = rte_sub_buffer;
  rte_sub_msg.data.size     = 0U;
  rte_sub_msg.data.capacity = sizeof(rte_sub_buffer);

  /* Executor with one handle (subscriber) */
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
 * @brief Publish "Hello N" and spin executor.
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

