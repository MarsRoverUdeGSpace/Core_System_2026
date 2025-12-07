#include "RTE.h"

MicroRosState rte_state;

/**
  * @brief  It receives commands from the Joystick and forwards them to the ESP32 (Bridge).
  * @param  msgin  Generic pointer to incoming message (Twist).
  * @note   It runs automatically when data arrives at the subscriber and 
  *         copies the data to the 'rte_state' structure before sending.
  */ 

static void twist_callback(const void * msgin) 
{
  const geometry_msgs__msg__Twist * twist_in = (const geometry_msgs__msg__Twist *)msgin;
  
  rte_state.twist_msg.linear.x = twist_in->linear.x;
  rte_state.twist_msg.angular.z = twist_in->angular.z;
  
  RCCHECK(rcl_publish(&rte_state.publis, &rte_state.twist_msg, NULL));
}

/** 
  * @brief  Initializes the micro-ROS Runtime Environment (Transport, Node, Entities).
  * @note   Configures Serial at 115200 baud, creates a Reliable QoS publisher,
  *         and attaches the subscriber callback to the executor.If any step fails,
  *         it triggers the error loop via RCCHECK.
  */

void RTE_Init() 
{
  Serial.begin(115200);set_microros_serial_transports(Serial);delay(2000);
  rte_state.allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&rte_state.support, 0, NULL, &rte_state.allocator));
  RCCHECK(rclc_node_init_default(&rte_state.node, "Kukulcan", "", &rte_state.support));

  RCCHECK(rclc_subscription_init_default
    (
    &rte_state.subs,
    &rte_state.node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel_from_joy"
    ));
  
  RCCHECK(rclc_publisher_init
    (
    &rte_state.publis,
    &rte_state.node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/turtle1/cmd_vel",
    &rmw_qos_profile_services_default 
    ));

  RCCHECK(rclc_executor_init(&rte_state.executor, &rte_state.support.context, 1, &rte_state.allocator));
  RCCHECK(rclc_executor_add_subscription(&rte_state.executor,&rte_state.subs,&rte_state.twist_msg,&twist_callback,ON_NEW_DATA));
}

void RTE_Run() 
{
  RCCHECK(rclc_executor_spin_some(&rte_state.executor, RCL_MS_TO_NS(10)));
}

