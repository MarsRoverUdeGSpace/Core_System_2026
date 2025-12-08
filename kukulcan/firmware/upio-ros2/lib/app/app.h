#pragma once

/**
 * @file app.h
 * @brief Application layer interface.
 */

/**
 * @brief Initialize application entities (publishers, subscribers).
 */
void app_Init(void);

/**
 * @brief Execute one application step.
 */
void app_Run(void);


