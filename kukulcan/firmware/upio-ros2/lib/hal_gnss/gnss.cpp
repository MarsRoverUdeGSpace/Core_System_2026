#include "gnss.h"

#include <TinyGPSPlus.h>

/*
 * Poll fast enough to consume the NMEA stream without monopolizing shared I2C.
 * At 50 ms, 64-byte reads can consume up to 1280 bytes/s.
 */
static const TickType_t gnss_task_period_ticks = pdMS_TO_TICKS(50U);
static constexpr uint8_t GNSS_READ_CHUNK_BYTES = 64U;
static constexpr uint32_t GNSS_FIX_MAX_AGE_MS = 2000UL;

static TinyGPSPlus g_gnss_parser;
static gnss_data_t g_gnss_cache;
static SemaphoreHandle_t g_gnss_mutex = NULL;
static TaskHandle_t g_gnss_task = NULL;
static uint32_t g_gnss_seq = 0U;
static bool g_gnss_present = false;

static bool gnss_i2c_device_present(void)
{
  if ((xI2cMutex == NULL) ||
      (xSemaphoreTake(xI2cMutex, pdMS_TO_TICKS(10U)) != pdTRUE))
  {
    return false;
  }

  i2c_bus.beginTransmission(NEO_M8N_I2C_ADDR);
  const uint8_t err = i2c_bus.endTransmission(true);
  (void)xSemaphoreGive(xI2cMutex);
  return (err == 0U);
}

static void gnss_UpdateCache(void)
{
  gnss_data_t local;
  local.latitude_deg = 0.0;
  local.longitude_deg = 0.0;
  local.altitude_m = 0.0F;
  local.satellites = 0U;
  local.hdop = 0.0F;
  local.chars_processed = g_gnss_parser.charsProcessed();
  local.passed_checksum = g_gnss_parser.passedChecksum();
  local.failed_checksum = g_gnss_parser.failedChecksum();
  local.seq = ++g_gnss_seq;
  local.valid = g_gnss_present && (local.chars_processed > 0U);
  local.has_fix = false;

  if (g_gnss_parser.location.isValid() &&
      (g_gnss_parser.location.age() <= GNSS_FIX_MAX_AGE_MS))
  {
    local.has_fix = true;
    local.latitude_deg = g_gnss_parser.location.lat();
    local.longitude_deg = g_gnss_parser.location.lng();
  }

  if (g_gnss_parser.altitude.isValid())
  {
    local.altitude_m = static_cast<float>(g_gnss_parser.altitude.meters());
  }

  if (g_gnss_parser.satellites.isValid())
  {
    local.satellites = g_gnss_parser.satellites.value();
  }

  if (g_gnss_parser.hdop.isValid())
  {
    local.hdop = static_cast<float>(g_gnss_parser.hdop.hdop());
  }

  if (g_gnss_mutex != NULL)
  {
    if (xSemaphoreTake(g_gnss_mutex, pdMS_TO_TICKS(1U)) == pdTRUE)
    {
      g_gnss_cache = local;
      (void)xSemaphoreGive(g_gnss_mutex);
    }
  }
}

static void gnss_Task(void * pvParameters)
{
  (void)pvParameters;
  TickType_t last_wake = xTaskGetTickCount();

  for (;;)
  {
    if (g_gnss_present == false)
    {
      g_gnss_present = gnss_i2c_device_present();
    }

    if (g_gnss_present)
    {
      if ((xI2cMutex != NULL) &&
          (xSemaphoreTake(xI2cMutex, pdMS_TO_TICKS(10U)) == pdTRUE))
      {
        const uint8_t bytes_requested =
            i2c_bus.requestFrom(NEO_M8N_I2C_ADDR, GNSS_READ_CHUNK_BYTES);
        (void)bytes_requested;

        while (i2c_bus.available() > 0)
        {
          const uint8_t c = i2c_bus.read();
          if (c != 0xFFU)
          {
            (void)g_gnss_parser.encode(static_cast<char>(c));
          }
        }
        (void)xSemaphoreGive(xI2cMutex);
      }
    }

    gnss_UpdateCache();
    vTaskDelayUntil(&last_wake, gnss_task_period_ticks);
  }
}

void Hal_Gnss_Init(void)
{
  if (g_gnss_mutex == NULL)
  {
    g_gnss_mutex = xSemaphoreCreateMutex();
  }

  g_gnss_cache.latitude_deg = 0.0;
  g_gnss_cache.longitude_deg = 0.0;
  g_gnss_cache.altitude_m = 0.0F;
  g_gnss_cache.satellites = 0U;
  g_gnss_cache.hdop = 0.0F;
  g_gnss_cache.chars_processed = 0U;
  g_gnss_cache.passed_checksum = 0U;
  g_gnss_cache.failed_checksum = 0U;
  g_gnss_cache.seq = 0U;
  g_gnss_cache.valid = false;
  g_gnss_cache.has_fix = false;
  g_gnss_seq = 0U;
  g_gnss_present = gnss_i2c_device_present();
}

void Hal_Gnss_StartTask(void)
{
  if (g_gnss_task != NULL)
  {
    return;
  }

  (void)xTaskCreatePinnedToCore(
      gnss_Task,
      "HalGnss",
      4096U,
      NULL,
      2U,
      &g_gnss_task,
      1);
}

bool Hal_Gnss_GetLatest(gnss_data_t * out)
{
  if ((out == NULL) || (g_gnss_mutex == NULL))
  {
    return false;
  }

  bool ok = false;
  if (xSemaphoreTake(g_gnss_mutex, pdMS_TO_TICKS(1U)) == pdTRUE)
  {
    *out = g_gnss_cache;
    ok = true;
    (void)xSemaphoreGive(g_gnss_mutex);
  }
  return ok;
}
