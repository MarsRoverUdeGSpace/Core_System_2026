#pragma once

/**
 * @file rte.h
 * @brief Micro-ROS runtime environment interface.
 */

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

/**
 * @brief Check micro-ROS return code and trap on error.
 *
 * @note This macro hides control flow.
 */
#define RCCHECK(fn)                \
  do {                             \
    rcl_ret_t rc_ = (fn);          \
    if (rc_ != RCL_RET_OK) {       \
      for (;;) {                   \
        delay(100U);               \
      }                            \
    }                              \
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
