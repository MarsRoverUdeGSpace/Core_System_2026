#include "RTE.h"

MicroRosState rte_state;

/** 
  * @brief  Initializes the micro-ROS Runtime Environment (Transport, Node, Entities).
  * @note   Configures Serial at 921600 baud, creates a Reliable QoS publisher,
  *         and attaches the subscriber callback to the executor.If any step fails,
  *         it triggers the error loop via RCCHECK.
  */

void RTE_Init() 
{
  Serial.begin(921600);set_microros_serial_transports(Serial);delay(2000);
  rte_state.allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&rte_state.support, 0, NULL, &rte_state.allocator));
  RCCHECK(rclc_node_init_default(&rte_state.node, "kukulcan", "", &rte_state.support));

  RCCHECK(rclc_executor_init(&rte_state.executor, &rte_state.support.context, 1, &rte_state.allocator));
}

void RTE_Run() 
{
  RCCHECK(rclc_executor_spin_some(&rte_state.executor, RCL_MS_TO_NS(10)));
}

