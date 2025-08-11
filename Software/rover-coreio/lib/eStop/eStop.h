#pragma once
#include <Arduino.h>

// Initialize E‑Stop input + interrupt
void EmergencyStopInit();

// Returns true if the button is currently pressed
bool EmergencyStopActivated();
