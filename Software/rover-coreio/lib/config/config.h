#pragma once
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <RoboClaw.h>
#include "freertos/queue.h"

// ── pin & bus definitions ─────────────────────────────────────
#define ESTOP_PIN   9
#define LED_PIN     15
#define N_LEDS      27
#define RBW_BAUD    38400
#define MTR1_ADDR   0x80U
#define MTR2_ADDR   0x80U
#define MTR3_ADDR   0x81U
#define MTR4_ADDR   0x81U

extern Adafruit_NeoPixel strip;
extern RoboClaw roboclaw;
extern QueueHandle_t motor_queue;

