/**
 * @file main.cpp
 * @brief Arduino entry point for upio-ros2 firmware.
 */

#include "app.h"

/**
 * @brief Initialize application layer and start tasks.
 */
void setup(void)
{
  app_Init();
  app_StartTasks();
}

/**
 * @brief Idle loop; work is handled by FreeRTOS tasks.
 */
void loop(void){}
