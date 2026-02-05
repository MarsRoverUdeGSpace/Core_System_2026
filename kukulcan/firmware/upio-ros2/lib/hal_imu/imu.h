#pragma once
#include "config.h"

typedef struct
{
  float qw;
  float qx;
  float qy;
  float qz;
  float gx;
  float gy;
  float gz;
  float ax;
  float ay;
  float az;
  uint32_t seq;
  bool  valid;
} imu_data_t;

void Hal_Imu_Init(void);
void Hal_Imu_StartTask(void);
bool Hal_Imu_GetLatest(imu_data_t * out);

/**
 * @brief Read IMU data (quaternion, gyro, linear accel).
 *
 * @param[out] quat_w Quaternion w.
 * @param[out] quat_x Quaternion x.
 * @param[out] quat_y Quaternion y.
 * @param[out] quat_z Quaternion z.
 * @param[out] gyro_x Angular velocity x (rad/s).
 * @param[out] gyro_y Angular velocity y (rad/s).
 * @param[out] gyro_z Angular velocity z (rad/s).
 * @param[out] accel_x Linear acceleration x (m/s^2).
 * @param[out] accel_y Linear acceleration y (m/s^2).
 * @param[out] accel_z Linear acceleration z (m/s^2).
 * @return true on success, false on invalid arguments or IMU not ready.
 */
bool Hal_Imu_Read(float * quat_w, float * quat_x, float * quat_y, float * quat_z,
                  float * gyro_x, float * gyro_y, float * gyro_z,
                  float * accel_x, float * accel_y, float * accel_z);
