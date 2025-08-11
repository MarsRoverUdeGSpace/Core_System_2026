#include "eStop.h"
#include "config.h"

// volatile flag updated in ISR
static volatile bool estop_flag = true;

// ISR fires on any pin change; update flag based on pin state
static void IRAM_ATTR estop_isr() {
  // LOW when pressed (button closes to GND)
  estop_flag = (digitalRead(ESTOP_PIN));
}

void EmergencyStopInit() {
  // Configure pull‑up so pin is HIGH when open, LOW when closed
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  // Trigger on both press and release
  attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), estop_isr, CHANGE);
}

bool EmergencyStopActivated() {
  return estop_flag;
}

