/**
  ******************************************************************************
  * File Name          : ws2812.c
  * Description        : Stable SK6812/WS2812 PWM-DMA output.
  ******************************************************************************
  */

#include "ws2812.h"

#include <string.h>

#include "mysys.h"
#include "tim.h"

#define WS2812_TRANSFER_TIMEOUT_MS 5U

static uint32_t color_storage[PIXEL_MAX];
uint32_t *color_buf = color_storage;
uint8_t rled[PIXEL_MAX];
uint8_t gled[PIXEL_MAX];
uint8_t bled[PIXEL_MAX];

static frame_buf frame;
static volatile uint8_t transfer_busy;
static volatile uint8_t frame_dirty;
static uint32_t transfer_started_ms;

volatile uint32_t ws2812_transfer_count;
volatile uint32_t ws2812_recovery_count;
volatile uint32_t ws2812_start_error_count;

static uint8_t scaled_channel(uint8_t channel)
{
  const uint8_t brightness = brightness_index <= 100U ? brightness_index : 100U;
  return (uint8_t)(((uint16_t)channel * brightness + 50U) / 100U);
}

static void refresh_scaled_pixel(uint8_t num)
{
  const uint32_t color = color_storage[num];
  rled[num] = scaled_channel((uint8_t)(color >> 16));
  gled[num] = scaled_channel((uint8_t)(color >> 8));
  bled[num] = scaled_channel((uint8_t)color);
}

static void build_frame(void)
{
  for (uint8_t i = 0U; i < PIXEL_MAX; ++i) {
    for (uint8_t bit = 0U; bit < 8U; ++bit) {
      const uint8_t mask = (uint8_t)(0x80U >> bit);
      frame.data[24U * i + bit] = (gled[i] & mask) ? BIT_1 : BIT_0;
      frame.data[24U * i + bit + 8U] = (rled[i] & mask) ? BIT_1 : BIT_0;
      frame.data[24U * i + bit + 16U] = (bled[i] & mask) ? BIT_1 : BIT_0;
    }
  }
}

void sk6812_init(uint8_t num)
{
  (void)num;
  memset(color_storage, 0, sizeof(color_storage));
  memset(rled, 0, sizeof(rled));
  memset(gled, 0, sizeof(gled));
  memset(bled, 0, sizeof(bled));
  memset(&frame, 0, sizeof(frame));
  transfer_busy = 0U;
  frame_dirty = 1U;
  ws2812_transfer_count = 0U;
  ws2812_recovery_count = 0U;
  ws2812_start_error_count = 0U;
}

void neopixel_set_color(uint8_t num, uint32_t color)
{
  if (num >= PIXEL_MAX) {
    return;
  }

  color &= 0x00FFFFFFU;
  if (color_storage[num] != color) {
    color_storage[num] = color;
    refresh_scaled_pixel(num);
    frame_dirty = 1U;
  }
}

void ws2812_service(void)
{
  const uint32_t now_ms = HAL_GetTick();
  if (transfer_busy) {
    if ((now_ms - transfer_started_ms) <= WS2812_TRANSFER_TIMEOUT_MS) {
      return;
    }

    /* A two-pixel frame is shorter than 0.1 ms. Recover only if a completion
       callback was lost; never abort a healthy frame merely to refresh it. */
    (void)HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
    transfer_busy = 0U;
    frame_dirty = 1U;
    ws2812_recovery_count++;
  }

  if (!frame_dirty) {
    return;
  }

  build_frame();
  transfer_busy = 1U;
  frame_dirty = 0U;
  transfer_started_ms = now_ms;

  const HAL_StatusTypeDef status = HAL_TIM_PWM_Start_DMA(
      &htim3,
      TIM_CHANNEL_2,
      (const uint32_t *)&frame,
      (uint16_t)(3U + 24U * PIXEL_MAX + 1U));
  if (status == HAL_OK) {
    ws2812_transfer_count++;
  } else {
    transfer_busy = 0U;
    frame_dirty = 1U;
    ws2812_start_error_count++;
  }
}

void ws2812_show(void)
{
  for (uint8_t i = 0U; i < PIXEL_MAX; ++i) {
    refresh_scaled_pixel(i);
  }
  frame_dirty = 1U;
  ws2812_service();
}

void ws2812_on_pwm_complete(TIM_HandleTypeDef *htim)
{
  if (htim == NULL || htim->Instance != TIM3 ||
      htim->Channel != HAL_TIM_ACTIVE_CHANNEL_2) {
    return;
  }

  __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC2);
  (void)HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_2);
  transfer_busy = 0U;
}
