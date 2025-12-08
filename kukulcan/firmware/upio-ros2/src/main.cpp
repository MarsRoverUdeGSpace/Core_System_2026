/**
 * @file main.cpp
 * @brief Arduino entry point for upio-ros2 firmware.
 */

#include <Arduino.h>
#include "app.h"

/**
 * @brief Initialize application layer.
 */
void setup() { app_Init(); }

/**
 * @brief Periodic application execution.
 */
void loop() { app_Run(); }

