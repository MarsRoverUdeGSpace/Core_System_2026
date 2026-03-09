#pragma once

/**
 * @file config.h
 * @brief Hardware configuration for Kukulcan upio-ros2.
 */

#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <RoboClaw.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_Sensor.h>

/* ----------------------------- RTE transport (micro-ROS) ----------------------------- */

static constexpr bool     RTE_USE_USB_CDC   = true;        /* true: USB CDC (Serial), false: UART */
static constexpr uint32_t RTE_USB_BAUD      = 921600UL;

static constexpr uint8_t  RTE_UART_INSTANCE = 2U;
static constexpr uint32_t RTE_UART_BAUD     = 921600UL;
static constexpr uint32_t RTE_UART_CONFIG   = SERIAL_8N1;
static constexpr uint8_t  RTE_UART_TX_PIN   = 9U;
static constexpr uint8_t  RTE_UART_RX_PIN   = 8U;

/* ----------------------------- Motor bus (RoboClaw) --------------------------------- */

static constexpr uint8_t  RBW_UART_INSTANCE = 1U;
static constexpr uint32_t RBW_UART_BAUD     = 460800UL;
static constexpr uint32_t RBW_UART_CONFIG   = SERIAL_8N1;
static constexpr uint8_t  RBW_UART_RX_PIN   = 36U;
static constexpr uint8_t  RBW_UART_TX_PIN   = 35U;

static constexpr uint8_t  ADDR_RB1          = 0x80U;
static constexpr uint8_t  ADDR_RB2          = 0x81U;

/* ----------------------------- I2C bus (shared) ------------------------------------- */

static constexpr uint8_t  I2C_SDA_PIN       = 10U;
static constexpr uint8_t  I2C_SCL_PIN       = 11U;
static constexpr uint32_t I2C_CLOCK_HZ      = 400000UL;

static constexpr uint8_t  BME280_I2C_ADDR   = 0x77U;
static constexpr uint8_t  BNO055_I2C_ADDR   = 0x28U;
static constexpr int32_t  BNO055_SENSOR_ID  = 55;

/* ----------------------------- Status LEDs ------------------------------------------ */

static constexpr uint8_t LED_WHITE_PIN      = 33U;
static constexpr uint8_t LED_RED_PIN        = 34U;

/* ----------------------------- NeoPixel status strip -------------------------------- */

static constexpr int16_t kNeoPixelPin = 37;
static constexpr uint16_t kNeoPixelCount = 90U;
static constexpr neoPixelType kNeoPixelType =
    static_cast<neoPixelType>(NEO_GRB + NEO_KHZ800);

/* ----------------------------- cmd_vel queue ---------------------------------------- */

typedef struct
{
  float linear_x;
  float angular_z;
} cmd_velQueueMsg_t;

static constexpr UBaseType_t RTE_CMD_VEL_QUEUE_DEPTH = 1U;

/* ----------------------------- Extern objects (defined in config.cpp) ---------------- */

extern QueueHandle_t xcmd_velQueue;
extern SemaphoreHandle_t xRoboClawMutex;

extern HardwareSerial rte_serial;

extern HardwareSerial motorSerial;
extern RoboClaw roboclaw;

extern TwoWire &i2c_bus;

extern Adafruit_BME280 bme280;
extern bool bme280_ready;

extern Adafruit_BNO055 bno055;
extern bool bno055_ready;

extern Adafruit_NeoPixel status_strip;

/* ----------------------------- Public API ------------------------------------------- */

void config_init(void);
