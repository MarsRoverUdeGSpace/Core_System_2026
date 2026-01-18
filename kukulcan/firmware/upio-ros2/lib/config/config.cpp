/**
 * @file config.cpp
 * @brief Hardware configuration implementation.
 */

#include "config.h"

/* Queue used to pass cmd_vel messages to the app layer. */
QueueHandle_t xcmd_velQueue = NULL;

/* Define the global serial object for RTE. */
HardwareSerial rte_serial(RTE_UART_INSTANCE);

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

  /* Create cmd_vel queue for RTE-to-app handoff. */
  xcmd_velQueue = xQueueCreate(RTE_CMD_VEL_QUEUE_DEPTH, sizeof(cmd_velQueueMsg_t));
}
