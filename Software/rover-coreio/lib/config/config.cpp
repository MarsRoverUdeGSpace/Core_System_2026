#include "config.h"

HardwareSerial &RBW_SERIAL  = Serial1;

Adafruit_NeoPixel strip(N_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
RoboClaw roboclaw(&RBW_SERIAL,  10000);

QueueHandle_t motor_queue = nullptr;

