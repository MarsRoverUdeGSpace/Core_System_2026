#include "led_status.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "config.h"

static const TickType_t kLedTaskPeriodTicks = pdMS_TO_TICKS(35U);
static const TickType_t kBlinkPeriodTicks = pdMS_TO_TICKS(400U);
static const TickType_t kPowerSettleTicks = pdMS_TO_TICKS(100U);
static const uint8_t kStripBrightness = 70U;

static volatile led_status_mode_t g_mode = LED_STATUS_MODE_FABULOUS;
static uint8_t g_fabulous_phase = 0U;
static TickType_t g_last_blink_toggle = 0U;
static bool g_blink_on = false;
static TaskHandle_t g_led_task_handle = NULL;

static uint32_t color_wheel(uint8_t pos)
{
  uint8_t r = 0U;
  uint8_t g = 0U;
  uint8_t b = 0U;

  pos = static_cast<uint8_t>(255U - pos);

  if (pos < 85U)
  {
    r = static_cast<uint8_t>(255U - (pos * 3U));
    b = static_cast<uint8_t>(pos * 3U);
  }
  else if (pos < 170U)
  {
    pos = static_cast<uint8_t>(pos - 85U);
    g = static_cast<uint8_t>(pos * 3U);
    b = static_cast<uint8_t>(255U - (pos * 3U));
  }
  else
  {
    pos = static_cast<uint8_t>(pos - 170U);
    r = static_cast<uint8_t>(pos * 3U);
    g = static_cast<uint8_t>(255U - (pos * 3U));
  }

  return status_strip.Color(r, g, b);
}

static uint8_t pseudo_sine_u8(uint8_t phase)
{
  const uint8_t tri = (phase & 0x80U)
                          ? static_cast<uint8_t>(255U - ((phase & 0x7FU) << 1U))
                          : static_cast<uint8_t>((phase & 0x7FU) << 1U);

  const uint16_t t = static_cast<uint16_t>(tri);
  return static_cast<uint8_t>((t * t) / 255U);
}

static uint8_t scale_u8(uint8_t value, uint8_t scale)
{
  return static_cast<uint8_t>((static_cast<uint16_t>(value) * scale + 127U) / 255U);
}

static uint8_t apply_floor(uint8_t intensity, uint8_t min_intensity)
{
  const uint16_t span = static_cast<uint16_t>(255U - min_intensity);
  return static_cast<uint8_t>(
      min_intensity + ((static_cast<uint16_t>(intensity) * span + 127U) / 255U));
}

static void fill_solid(uint8_t r, uint8_t g, uint8_t b)
{
  status_strip.fill(status_strip.Color(r, g, b), 0U, kNeoPixelCount);
  status_strip.show();
}

static void render_blink(uint8_t r, uint8_t g, uint8_t b)
{
  const TickType_t now = xTaskGetTickCount();
  if ((now - g_last_blink_toggle) >= kBlinkPeriodTicks)
  {
    g_last_blink_toggle = now;
    g_blink_on = !g_blink_on;
  }

  if (g_blink_on)
  {
    fill_solid(r, g, b);
  }
  else
  {
    fill_solid(0U, 0U, 0U);
  }
}

static void render_fabulous(void)
{
  static constexpr uint8_t kHueStep = 3U;
  static constexpr uint8_t kWaveStep = 5U;
  static constexpr uint8_t kWaveSpeed = 1U;
  static constexpr uint8_t kMinIntensity = 35U;

  for (uint16_t i = 0U; i < kNeoPixelCount; ++i)
  {
    const uint8_t hue = static_cast<uint8_t>((i * kHueStep) + g_fabulous_phase);
    const uint8_t wave_phase = static_cast<uint8_t>((i * kWaveStep) - (g_fabulous_phase * 2U));
    uint8_t intensity = pseudo_sine_u8(wave_phase);
    intensity = apply_floor(intensity, kMinIntensity);

    const uint32_t c = color_wheel(hue);
    const uint8_t r = static_cast<uint8_t>(c >> 16);
    const uint8_t g = static_cast<uint8_t>(c >> 8);
    const uint8_t b = static_cast<uint8_t>(c);

    status_strip.setPixelColor(
        i,
        scale_u8(r, intensity),
        scale_u8(g, intensity),
        scale_u8(b, intensity));
  }

  status_strip.show();
  g_fabulous_phase = static_cast<uint8_t>(g_fabulous_phase + kWaveSpeed);
}

static void render_teleop_arm(void)
{
  static constexpr uint8_t kHueStep = 3U;
  static constexpr uint8_t kWaveStep = 5U;
  static constexpr uint8_t kWaveSpeed = 1U;
  static constexpr uint8_t kMinIntensity = 35U;

  static constexpr uint8_t kMintR = 60U;
  static constexpr uint8_t kMintG = 220U;
  static constexpr uint8_t kMintB = 200U;

  static constexpr uint8_t kPurpleR = 70U;
  static constexpr uint8_t kPurpleG = 20U;
  static constexpr uint8_t kPurpleB = 130U;

  for (uint16_t i = 0U; i < kNeoPixelCount; ++i)
  {
    const uint8_t hue = static_cast<uint8_t>((i * kHueStep) + g_fabulous_phase);
    const uint8_t wave_phase = static_cast<uint8_t>((i * kWaveStep) - (g_fabulous_phase * 2U));

    uint8_t mix = pseudo_sine_u8(hue);
    uint8_t intensity = pseudo_sine_u8(wave_phase);
    intensity = apply_floor(intensity, kMinIntensity);

    const uint8_t inv_mix = static_cast<uint8_t>(255U - mix);
    const uint8_t r = static_cast<uint8_t>(
        ((static_cast<uint16_t>(kMintR) * mix) + (static_cast<uint16_t>(kPurpleR) * inv_mix)) /
        255U);
    const uint8_t g = static_cast<uint8_t>(
        ((static_cast<uint16_t>(kMintG) * mix) + (static_cast<uint16_t>(kPurpleG) * inv_mix)) /
        255U);
    const uint8_t b = static_cast<uint8_t>(
        ((static_cast<uint16_t>(kMintB) * mix) + (static_cast<uint16_t>(kPurpleB) * inv_mix)) /
        255U);

    status_strip.setPixelColor(
        i,
        scale_u8(r, intensity),
        scale_u8(g, intensity),
        scale_u8(b, intensity));
  }

  status_strip.show();
  g_fabulous_phase = static_cast<uint8_t>(g_fabulous_phase + kWaveSpeed);
}

void Hal_LedStatus_Init(void)
{
  vTaskDelay(kPowerSettleTicks);
  status_strip.begin();
  status_strip.clear();
  status_strip.setBrightness(kStripBrightness);
  status_strip.show();

  g_last_blink_toggle = xTaskGetTickCount();
  g_blink_on = false;
  g_fabulous_phase = 0U;
}

void Hal_LedStatus_SetMode(led_status_mode_t mode)
{
  g_mode = mode;
}

led_status_mode_t Hal_LedStatus_GetMode(void)
{
  return g_mode;
}

static void Hal_LedStatus_Task(void * pvParameters)
{
  (void)pvParameters;

  Hal_LedStatus_Init();
  TickType_t last_wake = xTaskGetTickCount();

  /* Render one frame immediately after init. */
  render_fabulous();

  for (;;)
  {
    const led_status_mode_t mode = g_mode;
    switch (mode)
    {
      case LED_STATUS_MODE_TELEOP:
        fill_solid(0U, 0U, 255U);  /* blue */
        break;
      case LED_STATUS_MODE_AUTONOMOUS:
        fill_solid(255U, 0U, 0U);  /* red */
        break;
      case LED_STATUS_MODE_GOAL_REACHED:
        render_blink(0U, 255U, 0U);  /* green blink */
        break;
      case LED_STATUS_MODE_SAFE_FAULT_STOP:
        render_blink(255U, 120U, 0U);  /* amber blink */
        break;
      case LED_STATUS_MODE_FABULOUS:
        render_fabulous();
        break;
      case LED_STATUS_MODE_TELEOP_ARM:
        render_teleop_arm();
        break;
      default:
        fill_solid(0U, 0U, 0U);
        break;
    }

    vTaskDelayUntil(&last_wake, kLedTaskPeriodTicks);
  }
}

void Hal_LedStatus_StartTask(void)
{
  if (g_led_task_handle != NULL)
  {
    return;
  }

  if (xTaskCreatePinnedToCore(
      Hal_LedStatus_Task,
      "HalLedStatus",
      4096U,
      NULL,
      2U,
      &g_led_task_handle,
      0) != pdPASS)
  {
    g_led_task_handle = NULL;
  }
}
