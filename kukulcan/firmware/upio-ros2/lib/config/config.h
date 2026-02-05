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
#include <RoboClaw.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

/* UART instance index used for Micro-ROS transport. */
/* Set true to use USB CDC (Serial) for testing/debugging. */
static constexpr bool     RTE_USE_USB_CDC   = true;
static constexpr uint32_t RTE_USB_BAUD      = 921600UL;
static constexpr uint8_t  RTE_UART_INSTANCE = 2;
static constexpr uint32_t RTE_UART_BAUD     = 921600UL;
static constexpr uint32_t RTE_UART_CONFIG   = SERIAL_8N1;
static constexpr uint8_t  RTE_UART_TX_PIN = 4;
static constexpr uint8_t  RTE_UART_RX_PIN = 5;

/* Queue depth for cmd_vel messages handed to the app layer. */
static constexpr UBaseType_t RTE_CMD_VEL_QUEUE_DEPTH = 1U;

/* UART instance index used for Roboclaw. */
static constexpr uint32_t RBW_UART_BAUD = 460800UL;
static constexpr uint8_t  ADDR_RB1 = 0x80U;
static constexpr uint8_t  ADDR_RB2 = 0x81U;
static constexpr uint8_t  RBW_UART_RX_PIN = 36;
static constexpr uint8_t  RBW_UART_TX_PIN = 35;
static constexpr uint8_t  RBW_UART_INSTANCE = 1;
static constexpr uint32_t RBW_UART_CONFIG = SERIAL_8N1;

/* Status LEDs (diagnostics). */
static constexpr uint8_t LED_WHITE_PIN = 33U;
static constexpr uint8_t LED_RED_PIN   = 34U;

/* I2C bus (shared across sensors). */
static constexpr uint8_t I2C_SDA_PIN = 10U;
static constexpr uint8_t I2C_SCL_PIN = 11U;
static constexpr uint32_t I2C_CLOCK_HZ = 400000UL;
static constexpr uint8_t BME280_I2C_ADDR = 0x77U;
static constexpr uint8_t BNO055_I2C_ADDR = 0x28U;
static constexpr int32_t BNO055_SENSOR_ID = 55;

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

/* Roboclaw Hardware Configuration */
extern HardwareSerial motorSerial;
extern RoboClaw roboclaw;

/* I2C bus and BME280 driver instances. */
extern TwoWire &i2c_bus;
extern Adafruit_BME280 bme280;
extern bool bme280_ready;
extern Adafruit_BNO055 bno055;
extern bool bno055_ready;

/**
 * @brief Initialize UART used by the RTE.
 */
void config_init(void);
