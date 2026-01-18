#pragma once

/**
 * @file config.h
 * @brief Hardware configuration for Kukulcan upio-ros2.
 */
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <HardwareSerial.h>

/* UART instance index used for Micro-ROS transport. */
/* Set true to use USB CDC (Serial) for testing/debugging. */
static constexpr bool     RTE_USE_USB_CDC   = true;
static constexpr uint32_t RTE_USB_BAUD      = 921600UL;
static constexpr uint8_t  RTE_UART_INSTANCE = 2;
static constexpr uint32_t RTE_UART_BAUD      = 921600UL;
static constexpr uint32_t RTE_UART_CONFIG   = SERIAL_8N1;
static constexpr uint8_t  RTE_UART_TX_PIN = 4;
static constexpr uint8_t  RTE_UART_RX_PIN = 5;
/* Queue depth for cmd_vel messages handed to the app layer. */
static constexpr UBaseType_t RTE_CMD_VEL_QUEUE_DEPTH = 10U;

/* Global serial object for RTE transport (defined in config.cpp). */
extern HardwareSerial rte_serial;

/**
 * @brief Structure to hold velocity commands in the queue.
 */
typedef struct {
    float linear_x;
    float angular_z;
} cmd_velQueueMsg_t;

/* Queue handle for cmd_vel messages (created in config_init). */
extern QueueHandle_t xcmd_velQueue;

/**
 * @brief Initialize UART used by the RTE.
 */
void config_init(void);
