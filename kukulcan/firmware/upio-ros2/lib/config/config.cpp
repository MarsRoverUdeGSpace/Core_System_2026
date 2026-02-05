/**
 * @file config.cpp
 * @brief Hardware configuration implementation.
 */

#include "config.h"

/* Queue used to pass cmd_vel messages to the app layer. */
QueueHandle_t xcmd_velQueue = NULL;

/* Define the global serial object for RTE. */
HardwareSerial rte_serial(RTE_UART_INSTANCE);

HardwareSerial motorSerial(RBW_UART_INSTANCE);
RoboClaw roboclaw(&motorSerial, 10000);
TwoWire &i2c_bus = Wire;
Adafruit_BME280 bme280;
bool bme280_ready = false;
Adafruit_BNO055 bno055(BNO055_SENSOR_ID, BNO055_I2C_ADDR, &i2c_bus);
bool bno055_ready = false;

static bool i2c_device_present(uint8_t addr)
{
  i2c_bus.beginTransmission(addr);
  const uint8_t err = i2c_bus.endTransmission(true);
  return (err == 0U);
}

/**
 * @brief Initialize UART used by the RTE.
 */
void config_init(void)
{
  if (RTE_USE_USB_CDC)
  {
    Serial.begin(RTE_USB_BAUD);
  }
  else
  {
    rte_serial.begin(
        RTE_UART_BAUD,
        RTE_UART_CONFIG,
        RTE_UART_RX_PIN,
        RTE_UART_TX_PIN);
  }

  /* Diagnostics LEDs default OFF. */
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_WHITE_PIN, OUTPUT);
  digitalWrite(LED_RED_PIN, LOW);
  digitalWrite(LED_WHITE_PIN, LOW);

  /* Create cmd_vel queue for RTE-to-app handoff. */
  xcmd_velQueue = xQueueCreate(RTE_CMD_VEL_QUEUE_DEPTH, sizeof(cmd_velQueueMsg_t));

  /* Motor UART. */
  motorSerial.begin(RBW_UART_BAUD, RBW_UART_CONFIG, RBW_UART_RX_PIN, RBW_UART_TX_PIN);
  roboclaw.begin(RBW_UART_BAUD);

  /* Shared I2C bus and sensor discovery. */
  i2c_bus.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  i2c_bus.setClock(I2C_CLOCK_HZ);
  bme280_ready = false;
  bno055_ready = false;

  if (i2c_device_present(BNO055_I2C_ADDR))
  {
    bno055_ready = bno055.begin(OPERATION_MODE_NDOF);
  }

  if (i2c_device_present(BME280_I2C_ADDR))
  {
    bme280_ready = bme280.begin(BME280_I2C_ADDR, &i2c_bus);
    if (bme280_ready)
    {
      bme280.setSampling(
          Adafruit_BME280::MODE_NORMAL,
          Adafruit_BME280::SAMPLING_X2,
          Adafruit_BME280::SAMPLING_X16,
          Adafruit_BME280::SAMPLING_X1,
          Adafruit_BME280::FILTER_X16,
          Adafruit_BME280::STANDBY_MS_500);
    }
  }
}
