/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include "button.h"

uint8_t button_enable_flag = 1;
uint8_t button_init_flag = 0;

button_t my_button;

static uint8_t button_raw_state = 1U;
static uint8_t button_stable_state = 1U;
static uint8_t button_long_reported;
static uint8_t button_click_pending;
static uint32_t button_raw_changed_ms;
static uint32_t button_pressed_ms;
static uint32_t button_first_click_ms;

static void button_begin_press(uint32_t now_ms)
{
  my_button.is_pressed = 1U;
  my_button.button_delay = now_ms;
  button_pressed_ms = now_ms;
  button_long_reported = 0U;
}

static void button_end_press(uint32_t now_ms)
{
  const uint32_t held_ms = now_ms - button_pressed_ms;

  my_button.is_pressed = 0U;
  if (held_ms >= BUTTON_LONG_LONG_PRESS_MS) {
    my_button.was_longlongpress = 1U;
  }

  if (button_long_reported || held_ms < BUTTON_MIN_CLICK_MS) {
    return;
  }

  if (button_click_pending &&
      (now_ms - button_first_click_ms) <= BUTTON_DOUBLE_CLICK_MS) {
    button_click_pending = 0U;
    my_button.was_click = 0U;
    my_button.was_double_click = 1U;
  }
  else {
    button_click_pending = 1U;
    button_first_click_ms = now_ms;
  }
}

void button_update(void)
{
  if (!button_enable_flag) {
    return;
  }

  const uint32_t now_ms = HAL_GetTick();
  const uint8_t sampled_state =
      HAL_GPIO_ReadPin(SYS_SW_GPIO_Port, SYS_SW_Pin) != GPIO_PIN_RESET;

  if (!button_init_flag) {
    button_raw_state = sampled_state;
    button_stable_state = sampled_state;
    button_raw_changed_ms = now_ms;
    my_button.button_status = sampled_state;
    my_button.button_delay = now_ms;
    button_init_flag = 1U;
    if (!sampled_state) {
      button_begin_press(now_ms);
    }
  }

  if (sampled_state != button_raw_state) {
    button_raw_state = sampled_state;
    button_raw_changed_ms = now_ms;
  }

  if (button_raw_state != button_stable_state &&
      (now_ms - button_raw_changed_ms) >= BUTTON_DEBOUNCE_MS) {
    button_stable_state = button_raw_state;
    my_button.button_status = button_stable_state;
    if (!button_stable_state) {
      button_begin_press(now_ms);
    }
    else {
      button_end_press(now_ms);
    }
  }

  if (!button_stable_state && my_button.is_pressed) {
    const uint32_t held_ms = now_ms - button_pressed_ms;
    if (!button_long_reported && held_ms >= BUTTON_LONG_PRESS_MS) {
      button_long_reported = 1U;
      button_click_pending = 0U;
      my_button.was_longpress = 1U;
    }
    if (held_ms >= BUTTON_LONG_LONG_PRESS_MS) {
      my_button.is_longlongpressed = 1U;
    }
  }

  if (button_click_pending && button_stable_state &&
      (now_ms - button_first_click_ms) > BUTTON_DOUBLE_CLICK_MS) {
    button_click_pending = 0U;
    my_button.was_click = 1U;
  }
}

void button_cancel_events(void)
{
  button_click_pending = 0U;
  if (my_button.is_pressed) {
    button_long_reported = 1U;
  }
  my_button.was_click = 0U;
  my_button.was_double_click = 0U;
  my_button.was_longpress = 0U;
  my_button.was_longlongpress = 0U;
  my_button.is_longlongpressed = 0U;
}
