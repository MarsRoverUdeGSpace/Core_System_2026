#pragma once

/**
 * @file rte.h
 * @brief Micro-ROS runtime environment interface.
 */

#include <Arduino.h>
#include "config.h"
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

/**
 * @brief Check micro-ROS return code and trap on error.
 *
 * @note This macro hides control flow.
 */
static inline void error_loop(void)
{
  for (;;)
  {
    digitalWrite(LED_RED_PIN, !digitalRead(LED_RED_PIN));
    delay(100U);
  }
}

#define RCCHECK(fn)                \
  do {                             \
    rcl_ret_t temp_rc = (fn);      \
    if (temp_rc != RCL_RET_OK) {   \
      error_loop();               \
    }                             \
  } while (0)

#define RCSOFTCHECK(fn)            \
  do {                             \
    rcl_ret_t temp_rc = (fn);      \
    if (temp_rc != RCL_RET_OK) {   \
    }                             \
  } while (0)

typedef struct MicroRosStateTag
{
  rcl_publisher_t    publis;
  rcl_subscription_t subs;
  rclc_executor_t    executor;
  rclc_support_t     support;
  rcl_allocator_t    allocator;
  rcl_node_t         node;
} MicroRosState;

/**
 * @brief Initialize the Micro-ROS runtime environment.
 */
void rte_Init(void);

/**
 * @brief Execute one Micro-ROS step (publish + spin).
 */
void rte_Run(void);
