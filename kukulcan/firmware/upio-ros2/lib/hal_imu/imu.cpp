#include "imu.h"

#include <utility/imumaths.h>

static float dps_to_rad(float value_dps)
{
  static constexpr float deg_to_rad = 0.01745329252F;
  return value_dps * deg_to_rad;
}

/**
 * @brief Read IMU data (quaternion, gyro, linear accel).
 */
bool Hal_Imu_Read(float * quat_w, float * quat_x, float * quat_y, float * quat_z,
                  float * gyro_x, float * gyro_y, float * gyro_z,
                  float * accel_x, float * accel_y, float * accel_z)
{
  if ((quat_w == NULL) || (quat_x == NULL) || (quat_y == NULL) || (quat_z == NULL) ||
      (gyro_x == NULL) || (gyro_y == NULL) || (gyro_z == NULL) ||
      (accel_x == NULL) || (accel_y == NULL) || (accel_z == NULL))
  {
    return false;
  }
  if (bno055_ready == false)
  {
    return false;
  }

  const imu::Quaternion quat = bno055.getQuat();
  const imu::Vector<3> gyro_dps = bno055.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  const imu::Vector<3> accel = bno055.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  *quat_w = static_cast<float>(quat.w());
  *quat_x = static_cast<float>(quat.x());
  *quat_y = static_cast<float>(quat.y());
  *quat_z = static_cast<float>(quat.z());

  *gyro_x = dps_to_rad(static_cast<float>(gyro_dps.x()));
  *gyro_y = dps_to_rad(static_cast<float>(gyro_dps.y()));
  *gyro_z = dps_to_rad(static_cast<float>(gyro_dps.z()));

  *accel_x = static_cast<float>(accel.x());
  *accel_y = static_cast<float>(accel.y());
  *accel_z = static_cast<float>(accel.z());

  return true;
}
