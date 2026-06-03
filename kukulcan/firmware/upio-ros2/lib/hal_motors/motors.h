#pragma once
#include "config.h"

/**
 * @brief Convierte velocidad lineal y angular a movimiento de motores.
 * * @param linear_x Velocidad hacia adelante (m/s o normalizada -1.0 a 1.0)
 * @param angular_z Velocidad de giro (rad/s o normalizada -1.0 a 1.0)
 */
void Hal_Motor_SetTwist(float linear_x, float angular_z);

/**
 * @brief Force a zero command to every motor channel.
 */
void Hal_Motor_Stop(void);
