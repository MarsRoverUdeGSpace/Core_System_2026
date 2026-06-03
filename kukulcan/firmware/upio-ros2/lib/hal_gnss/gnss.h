#pragma once

#include "config.h"

/**
 * @file gnss.h
 * @brief GNSS HAL for the u-blox NEO-M8N receiver over I2C/DDC.
 */

typedef struct
{
  double latitude_deg;
  double longitude_deg;
  float altitude_m;
  uint32_t satellites;
  float hdop;
  uint32_t chars_processed;
  uint32_t passed_checksum;
  uint32_t failed_checksum;
  uint32_t seq;
  bool valid;
  bool has_fix;
} gnss_data_t;

void Hal_Gnss_Init(void);
void Hal_Gnss_StartTask(void);
bool Hal_Gnss_GetLatest(gnss_data_t * out);
