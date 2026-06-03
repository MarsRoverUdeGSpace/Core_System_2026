#pragma once

/**
 * @file odom.h
 * @brief Application-layer wheel odometry integration from cached encoder ticks.
 */

#include "config.h"

/**
 * @brief Latest differential-drive odometry estimate.
 */
typedef struct
{
  double x_m;
  double y_m;
  double yaw_rad;
  float linear_x_mps;
  float angular_z_radps;
  uint32_t seq;
  bool valid;
} odom_data_t;

/**
 * @brief Initialize application odometry state and cache.
 */
void App_Odom_Init(void);

/**
 * @brief Consume latest encoder sample and update odometry when both wheels advance.
 */
void App_Odom_Update(void);

/**
 * @brief Copy latest odometry estimate from cache.
 *
 * @param[out] out Destination odometry sample.
 * @return true when a valid odometry sample was copied.
 */
bool App_Odom_GetLatest(odom_data_t * out);
