#pragma once

/**
 * @file led_status.h
 * @brief NeoPixel status strip HAL.
 */

#include <stdint.h>

typedef enum
{
  LED_STATUS_MODE_TELEOP = 0,
  LED_STATUS_MODE_AUTONOMOUS = 1,
  LED_STATUS_MODE_GOAL_REACHED = 2,
  LED_STATUS_MODE_SAFE_FAULT_STOP = 3,
  LED_STATUS_MODE_FABULOUS = 4,
  LED_STATUS_MODE_TELEOP_ARM = 5
} led_status_mode_t;

void Hal_LedStatus_Init(void);
void Hal_LedStatus_StartTask(void);
void Hal_LedStatus_SetMode(led_status_mode_t mode);
led_status_mode_t Hal_LedStatus_GetMode(void);
