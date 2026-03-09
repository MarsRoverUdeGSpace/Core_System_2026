#pragma once

#include "config.h"

/**
 * @file enc.h
 * @brief RoboClaw encoder HAL for raw M1 encoder access on both controllers.
 */

typedef struct
{
  int32_t left_ticks;
  int32_t right_ticks;
  int32_t left_qpps;
  int32_t right_qpps;
  uint8_t left_status;
  uint8_t right_status;
  uint32_t seq;
  bool valid;
} enc_data_t;

void Hal_Enc_Init(void);
void Hal_Enc_StartTask(void);
bool Hal_Enc_GetLatest(enc_data_t * out);

/**
 * @brief Read raw encoder counters and speeds from RoboClaw M1 on both controllers.
 *
 * Mapping:
 * - left  = RoboClaw 0x80 M1
 * - right = RoboClaw 0x81 M1
 *
 * @param[out] left_ticks Raw encoder count for left M1.
 * @param[out] right_ticks Raw encoder count for right M1.
 * @param[out] left_qpps Raw encoder speed for left M1 in quadrature pulses per second.
 * @param[out] right_qpps Raw encoder speed for right M1 in quadrature pulses per second.
 * @param[out] left_status RoboClaw status byte for left encoder read.
 * @param[out] right_status RoboClaw status byte for right encoder read.
 * @return true on success, false on invalid arguments or bus/read failure.
 */
bool Hal_Enc_Read(int32_t * left_ticks,
                  int32_t * right_ticks,
                  int32_t * left_qpps,
                  int32_t * right_qpps,
                  uint8_t * left_status,
                  uint8_t * right_status);
