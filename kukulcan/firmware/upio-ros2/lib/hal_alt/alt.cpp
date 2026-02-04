#include "alt.h"

/**
 * @brief Read BME280 data (temperature, pressure, humidity).
 */
bool Hal_Alt_Read(float * temp_c, float * press_pa, float * hum_pct)
{
  if ((temp_c == NULL) || (press_pa == NULL) || (hum_pct == NULL))
  {
    return false;
  }
  if (bme280_ready == false)
  {
    return false;
  }

  const float temp = bme280.readTemperature();
  const float press = bme280.readPressure();
  const float hum = bme280.readHumidity();

  *temp_c = temp;
  *press_pa = press;
  *hum_pct = hum;

  return true;
}
