#include "hal_led.h"

static Adafruit_NeoPixel *s = nullptr;

void HalLedInit() {
  s = &strip;
  s->begin();
  s->fill(s->Color(255, 255, 255), 0, N_LEDS);
  s->show();
}

void HalLedHandleMode(OperationMode mode) {
  if (mode == ARUCO_DETECTED) {
    for (int i = 0; i < 3; i++) {
      s->fill(s->Color(0,255,0), 0, N_LEDS); s->show();
      vTaskDelay(pdMS_TO_TICKS(200));
      s->fill(0, 0, N_LEDS);               s->show();
      vTaskDelay(pdMS_TO_TICKS(200));
    }
    s->fill(s->Color(255,0,0), 0, N_LEDS);
    s->show();
    return;
  }
  uint32_t c = (mode==AUTONOMOUS)   ? s->Color(255,0,0)
                : (mode==TELEOPERATED) ? s->Color(0,0,255)
                : 0;
  s->fill(c, 0, N_LEDS);
  s->show();
}

