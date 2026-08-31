#include "local_rgb.h"

#include "main.h"
#include "local_ui.h"
#include "mysys.h"
#include "ws2812.h"

#define LOCAL_RGB_UPDATE_PERIOD_MS 20U
#define LOCAL_RGB_SAVE_HOLD_MS 700U
#define LOCAL_RGB_BREATH_PERIOD_MS 3200U
#define LOCAL_RGB_FAULT_PERIOD_MS 1200U

#define LOCAL_RGB_COLOR_PAUSED 0x00000CU
#define LOCAL_RGB_COLOR_MENU 0x001218U
#define LOCAL_RGB_COLOR_EDIT 0x181000U
#define LOCAL_RGB_COLOR_SAVING 0x001C04U
#define LOCAL_RGB_COLOR_FAULT 0x280000U

static LocalRgbState current_state;
static uint32_t current_color;
static uint32_t state_started_ms;
static uint32_t next_update_ms;
static volatile uint32_t save_hold_until_ms;

static uint8_t time_reached(uint32_t now_ms, uint32_t deadline_ms)
{
  return (int32_t)(now_ms - deadline_ms) >= 0;
}

static uint32_t pack_scaled_color(uint8_t red,
                                  uint8_t green,
                                  uint8_t blue,
                                  uint8_t level)
{
  const uint32_t r = ((uint16_t)red * level + 127U) / 255U;
  const uint32_t g = ((uint16_t)green * level + 127U) / 255U;
  const uint32_t b = ((uint16_t)blue * level + 127U) / 255U;
  return (r << 16) | (g << 8) | b;
}

static uint32_t running_breath_color(uint32_t elapsed_ms)
{
  const uint32_t phase = elapsed_ms % LOCAL_RGB_BREATH_PERIOD_MS;
  const uint32_t half_period = LOCAL_RGB_BREATH_PERIOD_MS / 2U;
  const uint32_t triangle = phase <= half_period
                                ? phase * 255U / half_period
                                : (LOCAL_RGB_BREATH_PERIOD_MS - phase) *
                                      255U / half_period;
  /* Integer smoothstep removes the sharp turn of a raw triangle wave. */
  const uint32_t smooth = triangle * triangle * (765U - 2U * triangle) /
                          65025U;
  const uint8_t level = (uint8_t)(72U + smooth * 183U / 255U);
  return pack_scaled_color(20U, 2U, 32U, level);
}

static LocalRgbState select_state(uint32_t now_ms)
{
  const uint8_t save_pending = MysysLocalSavePending();
  if (error_code != ERR_NONE || sys_status == SYS_ERROR) {
    return LOCAL_RGB_STATE_FAULT;
  }
  if (save_pending || !time_reached(now_ms, save_hold_until_ms)) {
    return LOCAL_RGB_STATE_SAVING;
  }
  if (LocalUiIsEditing()) {
    return LOCAL_RGB_STATE_EDIT;
  }
  if (LocalUiIsMenuActive()) {
    return LOCAL_RGB_STATE_MENU;
  }
  if (!motor_output || sys_status == SYS_STANDBY) {
    return LOCAL_RGB_STATE_PAUSED;
  }
  return LOCAL_RGB_STATE_RUNNING;
}

static uint32_t state_color(LocalRgbState state, uint32_t elapsed_ms)
{
  switch (state) {
  case LOCAL_RGB_STATE_RUNNING:
    return running_breath_color(elapsed_ms);
  case LOCAL_RGB_STATE_PAUSED:
    return LOCAL_RGB_COLOR_PAUSED;
  case LOCAL_RGB_STATE_MENU:
    return LOCAL_RGB_COLOR_MENU;
  case LOCAL_RGB_STATE_EDIT:
    return LOCAL_RGB_COLOR_EDIT;
  case LOCAL_RGB_STATE_SAVING:
    return LOCAL_RGB_COLOR_SAVING;
  case LOCAL_RGB_STATE_FAULT: {
    const uint32_t phase = elapsed_ms % LOCAL_RGB_FAULT_PERIOD_MS;
    const uint8_t on = phase < 140U || (phase >= 260U && phase < 400U);
    return on ? LOCAL_RGB_COLOR_FAULT : 0U;
  }
  default:
    return 0U;
  }
}

static void apply_color(uint32_t color)
{
  if (color == current_color) {
    return;
  }
  current_color = color;
  for (uint8_t pixel = 0U; pixel < PIXEL_MAX; ++pixel) {
    neopixel_set_color(pixel, color);
  }
  ws2812_show();
}

void LocalRgbInitialize(void)
{
  current_state = (LocalRgbState)0xFFU;
  current_color = 0xFFFFFFFFU;
  state_started_ms = HAL_GetTick();
  next_update_ms = 0U;
  save_hold_until_ms = 0U;
}

void LocalRgbTask(void)
{
  const uint32_t now_ms = HAL_GetTick();
  ws2812_service();

  if (next_update_ms != 0U && !time_reached(now_ms, next_update_ms)) {
    return;
  }
  next_update_ms = now_ms + LOCAL_RGB_UPDATE_PERIOD_MS;

  const LocalRgbState selected = select_state(now_ms);
  if (selected != current_state) {
    current_state = selected;
    state_started_ms = now_ms;
  }
  apply_color(state_color(current_state, now_ms - state_started_ms));
}

void LocalRgbNotifySave(void)
{
  save_hold_until_ms = HAL_GetTick() + LOCAL_RGB_SAVE_HOLD_MS;
}

LocalRgbState LocalRgbGetState(void)
{
  return current_state;
}

uint32_t LocalRgbGetColor(void)
{
  return current_color;
}
