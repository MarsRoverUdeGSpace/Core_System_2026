#pragma once

/**
 * @file config.h
 * @brief Hardware configuration for Kukulcan upio-ros2.
 */

#include <Arduino.h>
#include <HardwareSerial.h>

/* UART instance index used for micro-ROS transport. */
static constexpr uint8_t  RTE_UART_INSTANCE = 2;
static constexpr uint32_t RTE_UART_BAUD      = 921600UL;
static constexpr uint32_t RTE_UART_CONFIG   = SERIAL_8N1;
static constexpr uint8_t  RTE_UART_TX_PIN = 4;
static constexpr uint8_t  RTE_UART_RX_PIN = 5;

/* Global serial object for RTE transport (defined in config.cpp). */
extern HardwareSerial rte_serial;

/**
 * @brief Initialize UART used by RTE.
 */
void config_init(void);

