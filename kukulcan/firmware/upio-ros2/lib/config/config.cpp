/**
 * @file config.cpp
 * @brief Hardware configuration implementation.
 */

#include "config.h"

/* Define the global serial object for RTE. */
HardwareSerial rte_serial(RTE_UART_INSTANCE);

/**
 * @brief Initialize UART used by RTE.
 */
void config_init(void)
{
  rte_serial.begin(
      RTE_UART_BAUD,
      RTE_UART_CONFIG,
      RTE_UART_RX_PIN,
      RTE_UART_TX_PIN);
}

