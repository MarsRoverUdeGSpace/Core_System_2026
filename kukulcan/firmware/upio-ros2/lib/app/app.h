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

/**
 * @brief Execute one application step.
 *
 * @note Intended to be called from the Micro-ROS task.
 */
void app_Run(void);
