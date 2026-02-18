/**
 * @file config.cpp
 * @brief Hardware configuration implementation.
 */

#include "config.h"
#include "imu.h"
#include "alt.h"

/* ----------------------------- Globals (defined here, declared extern in config.h) ----------------------------- */

/* Queue used to pass cmd_vel messages to the app layer. */
QueueHandle_t xcmd_velQueue = NULL;

/* RTE serial (UART transport when not using USB CDC). */
HardwareSerial rte_serial(RTE_UART_INSTANCE);

/* Motor UART + RoboClaw driver. */
HardwareSerial motorSerial(RBW_UART_INSTANCE);
RoboClaw roboclaw(&motorSerial, 10000U);

/* Shared I2C bus and sensors. */
TwoWire &i2c_bus = Wire;

Adafruit_BME280 bme280;
bool bme280_ready = false;

Adafruit_BNO055 bno055(BNO055_SENSOR_ID, BNO055_I2C_ADDR, &i2c_bus);
bool bno055_ready = false;

Adafruit_NeoPixel status_strip(kNeoPixelCount, kNeoPixelPin, kNeoPixelType);

/* ----------------------------- Local helpers ----------------------------- */

/**
 * @brief Probe an I2C address (7-bit) by issuing a START and STOP and checking for ACK.
 * @param addr I2C 7-bit address.
 * @return true if device ACKs, false otherwise.
 */
static bool i2c_device_present(const uint8_t addr)
{
  i2c_bus.beginTransmission(addr);
  const uint8_t err = i2c_bus.endTransmission(true);
  return (err == 0U);
}

/**
 * @brief Initialize the RTE transport (USB CDC or UART).
 */
static void rte_transport_init(void)
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
}

/**
 * @brief Initialize diagnostic LEDs and set them to OFF.
 */
static void leds_init_default_off(void)
{
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_WHITE_PIN, OUTPUT);

  digitalWrite(LED_RED_PIN, LOW);
  digitalWrite(LED_WHITE_PIN, LOW);
}

/**
 * @brief Create the cmd_vel queue used for RTE-to-app handoff.
 */
static void cmd_vel_queue_init(void)
{
  xcmd_velQueue = xQueueCreate(RTE_CMD_VEL_QUEUE_DEPTH, sizeof(cmd_velQueueMsg_t));
  /* If you require fail-fast behavior, handle NULL here. */
}

/**
 * @brief Initialize the motor UART and RoboClaw driver.
 */
static void motor_bus_init(void)
{
  motorSerial.begin(RBW_UART_BAUD, RBW_UART_CONFIG, RBW_UART_RX_PIN, RBW_UART_TX_PIN);
  roboclaw.begin(RBW_UART_BAUD);
}

/**
 * @brief Initialize shared I2C bus and bring up sensors if present.
 */
static void sensors_i2c_init(void)
{
  i2c_bus.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  i2c_bus.setClock(I2C_CLOCK_HZ);

  bme280_ready = false;
  bno055_ready = false;

  /* BNO055: only attempt begin() if the address ACKs. */
  if (i2c_device_present(BNO055_I2C_ADDR))
  {
    bno055_ready = bno055.begin(OPERATION_MODE_NDOF);
  }

  /* BME280: only attempt begin() if the address ACKs. */
  if (i2c_device_present(BME280_I2C_ADDR))
  {
    bme280_ready = bme280.begin(BME280_I2C_ADDR, &i2c_bus);

    if (bme280_ready)
    {
      bme280.setSampling(
          Adafruit_BME280::MODE_NORMAL,
          Adafruit_BME280::SAMPLING_X2,     /* Temperature */
          Adafruit_BME280::SAMPLING_X16,    /* Pressure */
          Adafruit_BME280::SAMPLING_X1,     /* Humidity */
          Adafruit_BME280::FILTER_X16,
          Adafruit_BME280::STANDBY_MS_500);
    }
  }
}

/* ----------------------------- Public API ----------------------------- */

void config_init(void)
{
  rte_transport_init();
  leds_init_default_off();
  cmd_vel_queue_init();
  motor_bus_init();
  sensors_i2c_init();
  Hal_Imu_Init();
  Hal_Alt_Init();
}
