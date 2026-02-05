#pragma once
#include <Arduino.h>

/* Latest IMU sample (NDOF). */
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

/* Latest BME280 sample. */
typedef struct
{
  float temp_c;
  float press_pa;
  float hum_pct;
  uint32_t seq;
  bool  valid;
} bme_data_t;

/* Initialize sensor cache and synchronization primitives. */
void Sensors_Init(void);
/* Start the I2C sampling task (50 Hz IMU, 1 Hz BME). */
void Sensors_StartTask(void);
/* Retrieve most recent IMU sample (non-blocking). */
bool Sensors_GetImu(imu_data_t * out);
/* Retrieve most recent BME sample (non-blocking). */
bool Sensors_GetBme(bme_data_t * out);
