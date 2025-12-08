/**
 * @file rte.cpp
 * @brief micro-ROS runtime environment implementation.
 */

#include "rte.h"

/* Internal state (non-const global by design for RTE) */
static MicroRosState rte_state;

/**
 * @brief Return pointer to RTE state.
 */
MicroRosState * rte_GetState(void)
{
  return &rte_state;
}

/**
 * @brief Initialize serial transport, allocator, support, node, and executor.
 */
void rte_Init(void)
{
  Serial.begin(921600);
  set_microros_serial_transports(Serial);
  delay(2000U);

  rte_state.allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(
      &rte_state.support,
      0,
      NULL,
      &rte_state.allocator));

  RCCHECK(rclc_node_init_default(
      &rte_state.node,
      "kukulcan",
      "",
      &rte_state.support));

  RCCHECK(rclc_executor_init(
      &rte_state.executor,
      &rte_state.support.context,
      1U,
      &rte_state.allocator));
}

/**
 * @brief Spin executor for a short period to process entities.
 */
void rte_Run(void)
{
  RCCHECK(rclc_executor_spin_some(
      &rte_state.executor,
      RCL_MS_TO_NS(10U)));
}


