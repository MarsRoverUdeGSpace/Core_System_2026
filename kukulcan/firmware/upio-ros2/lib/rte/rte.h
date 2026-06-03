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
 * @brief Fatal error handler: blink red LED N times, then reboot (ESP32).
 *
 * @note This function never returns.
 */
static inline void error_loop(void)
{
  /* Fatal error: white LED off, red LED blink N times, then reboot. */
  digitalWrite(LED_WHITE_PIN, LOW);

  for (uint32_t i = 0U; i < 10U; ++i)
  {
    /* Toggle red LED to indicate fatal condition. */
    digitalWrite(LED_RED_PIN, !digitalRead(LED_RED_PIN));
    delay(100U);
  }

#if defined(ARDUINO_ARCH_ESP32)
  /* Soft reboot (ESP32 Arduino core). */
  ESP.restart();
#endif

  /* Should never reach here; trap as a fallback. */
  for (;;)
  {
    /* Intentional infinite loop */
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
      /* Intentionally ignored */  \
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
 * @brief Spin the micro-ROS executor once.
 */
void rte_SpinOnce(void);

/**
 * @brief Publish latest IMU sample from cache.
 */
void rte_PublishImu(void);

/**
 * @brief Publish latest magnetometer sample from the IMU cache.
 */
void rte_PublishMag(void);

void rte_PublishEncoders(void);

/**
 * @brief Publish latest app-layer wheel odometry estimate.
 */
void rte_PublishOdom(void);

/**
 * @brief Publish latest GNSS fix sample from cache.
 */
void rte_PublishGnss(void);

/**
 * @brief Publish latest BME sample from cache.
 */
void rte_PublishBme(void);
