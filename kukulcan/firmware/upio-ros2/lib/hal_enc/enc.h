#pragma once

#include "config.h"

/**
 * @file enc.h
 * @brief RoboClaw encoder HAL for raw left-M1 and right-M2 encoder access.
 */

typedef struct
{
  int32_t left_ticks;
  int32_t right_ticks;
  int32_t left_qpps;
  int32_t right_qpps;
  uint8_t left_status;
  uint8_t right_status;
  bool qpps_valid;
  uint32_t seq;
  bool valid;
} enc_data_t;

void Hal_Enc_Init(void);
void Hal_Enc_StartTask(void);
bool Hal_Enc_GetLatest(enc_data_t * out);

/**
 * @brief Read raw encoder counters and optional speeds from the configured RoboClaw channels.
 *
 * Mapping:
 * - left  = RoboClaw 0x80 M1
 * - right = RoboClaw 0x81 M2
 *
 * @param[out] left_ticks Raw encoder count for left M1.
 * @param[out] right_ticks Raw encoder count for right M2.
 * @param[out] left_qpps Raw encoder speed for left M1 in quadrature pulses per second.
 * @param[out] right_qpps Raw encoder speed for right M2 in quadrature pulses per second.
 * @param[out] left_status RoboClaw status byte for left encoder read.
 * @param[out] right_status RoboClaw status byte for right encoder read.
 * @param[in] read_qpps Whether to perform optional wheel-speed reads.
 * @param[out] qpps_valid True when optional wheel-speed reads succeeded.
 * @return true when both encoder counters were read successfully.
 */
bool Hal_Enc_Read(int32_t * left_ticks,
                  int32_t * right_ticks,
                  int32_t * left_qpps,
                  int32_t * right_qpps,
                  uint8_t * left_status,
                  uint8_t * right_status,
                  bool read_qpps,
                  bool * qpps_valid);
