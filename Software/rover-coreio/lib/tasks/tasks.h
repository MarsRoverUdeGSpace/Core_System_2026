#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

void rosTask(void *pvParameters);
void motorTask(void *pvParameters);

