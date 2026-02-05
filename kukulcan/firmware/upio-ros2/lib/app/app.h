#pragma once

/**
 * @file app.h
 * @brief Application layer interface.
 */

/**
 * @brief Initialize application layer.
 */
void app_Init(void);

/**
 * @brief Start application FreeRTOS tasks.
 */
void app_StartTasks(void);
