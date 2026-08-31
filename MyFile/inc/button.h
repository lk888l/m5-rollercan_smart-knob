/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef __BUTTON_H__
#define __BUTTON_H__

#include "main.h"

#define BUTTON_DEBOUNCE_MS 20U
#define BUTTON_MIN_CLICK_MS 30U
#define BUTTON_DOUBLE_CLICK_MS 350U
#define BUTTON_LONG_PRESS_MS 1200U
#define BUTTON_LONG_LONG_PRESS_MS 5000U

typedef struct
{
    uint8_t button_status;
    unsigned long button_delay;
    uint8_t is_pressed;
    uint8_t is_longlongpressed;
    uint8_t was_click;
    uint8_t was_double_click;
    uint8_t was_longpress;
    uint8_t was_longlongpress;
} button_t;

extern button_t my_button;

void button_update(void);

#endif
