#pragma once
#include "config.h"

/**
 * @brief Read BME280 data (temperature, pressure, humidity).
 *
 * @param[out] temp_c Temperature in degrees C.
 * @param[out] press_pa Pressure in Pa.
 * @param[out] hum_pct Relative humidity in %.
 * @return true on success, false on invalid arguments.
 */
bool Hal_Alt_Read(float * temp_c, float * press_pa, float * hum_pct);
