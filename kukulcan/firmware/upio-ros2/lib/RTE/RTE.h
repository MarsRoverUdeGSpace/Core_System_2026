#pragma once

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#define RCCHECK(fn) {rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

typedef struct {
    rcl_publisher_t             publis;
    rcl_subscription_t          subs;
    rclc_executor_t             executor;
    rclc_support_t              support;
    rcl_allocator_t             allocator;
    rcl_node_t                  node;
} MicroRosState;

void RTE_Init();
void RTE_Run();

