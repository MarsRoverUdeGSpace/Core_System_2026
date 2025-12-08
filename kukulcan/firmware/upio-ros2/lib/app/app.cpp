/**
 * @file App.cpp
 * @brief Application layer implementation.
 */

#include "app.h"
#include "rte.h"

#include <std_msgs/msg/string.h>
#include <stdio.h>
#include <string.h>

/* Publisher state */
static std_msgs__msg__String app_pub_msg;
static char                  app_pub_buffer[32];
static uint32_t              app_counter = 0U;

/* Subscriber state */
static std_msgs__msg__String app_sub_msg;
static char                  app_sub_buffer[64];

/**
 * @brief Subscriber callback for "app_in" topic.
 *
 * @param msgin Pointer to received message.
 */
static void app_sub_callback(const void * msgin)
{
  const std_msgs__msg__String * msg =
      static_cast<const std_msgs__msg__String *>(msgin);

  Serial.print("[App] app_in: ");
  Serial.write(msg->data.data, msg->data.size);
  Serial.println();
}

/**
 * @brief Initialize application publisher and subscriber.
 */
void app_Init(void)
{
  MicroRosState * state;

  rte_Init();
  state = rte_GetState();

  /* Publisher: "app_hello" */
  RCCHECK(rclc_publisher_init_default(
      &state->publis,
      &state->node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "app_hello"));

  app_pub_msg.data.data     = app_pub_buffer;
  app_pub_msg.data.size     = 0U;
  app_pub_msg.data.capacity = sizeof(app_pub_buffer);

  /* Subscriber: "app_in" */
  RCCHECK(rclc_subscription_init_default(
      &state->subs,
      &state->node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "app_in"));

  app_sub_msg.data.data     = app_sub_buffer;
  app_sub_msg.data.size     = 0U;
  app_sub_msg.data.capacity = sizeof(app_sub_buffer);

  RCCHECK(rclc_executor_add_subscription(
      &state->executor,
      &state->subs,
      &app_sub_msg,
      &app_sub_callback,
      ON_NEW_DATA));
}

/**
 * @brief Publish app_hello message and service rte.
 */
void app_Run(void)
{
  MicroRosState * state;
  int             written;
  size_t          len;

  state = rte_GetState();

  written = snprintf(
      app_pub_buffer,
      sizeof(app_pub_buffer),
      "Hello %lu",
      static_cast<unsigned long>(app_counter));

  if (written > 0)
  {
    len = static_cast<size_t>(written);
    if (len >= sizeof(app_pub_buffer))
    {
      len = sizeof(app_pub_buffer) - 1U;
      app_pub_buffer[len] = '\0';
    }
    app_pub_msg.data.size = len;
  }

  RCCHECK(rcl_publish(
      &state->publis,
      &app_pub_msg,
      NULL));

  app_counter++;

  rte_Run();
}


