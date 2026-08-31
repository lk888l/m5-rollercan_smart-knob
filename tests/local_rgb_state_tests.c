#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "local_rgb.h"
#include "mysys.h"

uint8_t motor_mode = 4U;
uint8_t motor_output = 1U;
uint8_t sys_status = 1U;
uint8_t error_code;
uint8_t over_vol_flag;
float mechanical_rad;
float motor_rps;
float ph_crrent_lpf;

static uint32_t fake_tick_ms;
static uint8_t menu_active;
static uint8_t editing;
static uint8_t save_pending;
static uint32_t pixel_color[2];
static uint32_t show_count;

uint32_t HAL_GetTick(void)
{
    return fake_tick_ms;
}

uint32_t micros(void)
{
    return fake_tick_ms * 1000U;
}

uint8_t MysysLocalSavePending(void)
{
    return save_pending;
}

uint8_t LocalUiIsMenuActive(void)
{
    return menu_active;
}

uint8_t LocalUiIsEditing(void)
{
    return editing;
}

void neopixel_set_color(uint8_t num, uint32_t color)
{
    assert(num < 2U);
    pixel_color[num] = color;
}

void ws2812_show(void)
{
    show_count++;
}

void ws2812_service(void)
{
}

static void advance(uint32_t milliseconds)
{
    fake_tick_ms += milliseconds;
    LocalRgbTask();
}

int main(void)
{
    LocalRgbInitialize();
    LocalRgbTask();
    assert(LocalRgbGetState() == LOCAL_RGB_STATE_RUNNING);
    assert(pixel_color[0] == pixel_color[1]);
    assert(LocalRgbGetColor() != 0U);
    const uint32_t initial_shows = show_count;

    /* Calls before the 20 ms effect deadline must not retransmit. */
    advance(10U);
    assert(show_count == initial_shows);

    motor_output = 0U;
    sys_status = SYS_STANDBY;
    advance(10U);
    assert(LocalRgbGetState() == LOCAL_RGB_STATE_PAUSED);
    assert(LocalRgbGetColor() == 0x00000CU);
    const uint32_t paused_shows = show_count;
    for (uint32_t i = 0U; i < 20U; ++i) {
        advance(20U);
    }
    assert(show_count == paused_shows);

    menu_active = 1U;
    sys_status = SYS_RUNNING;
    advance(20U);
    assert(LocalRgbGetState() == LOCAL_RGB_STATE_MENU);
    assert(LocalRgbGetColor() == 0x001218U);

    editing = 1U;
    advance(20U);
    assert(LocalRgbGetState() == LOCAL_RGB_STATE_EDIT);
    assert(LocalRgbGetColor() == 0x181000U);

    save_pending = 1U;
    LocalRgbNotifySave();
    editing = 0U;
    menu_active = 0U;
    advance(20U);
    assert(LocalRgbGetState() == LOCAL_RGB_STATE_SAVING);
    assert(LocalRgbGetColor() == 0x001C04U);
    save_pending = 0U;
    advance(600U);
    assert(LocalRgbGetState() == LOCAL_RGB_STATE_SAVING);
    advance(120U);
    assert(LocalRgbGetState() == LOCAL_RGB_STATE_PAUSED);

    error_code = ERR_STALLED;
    advance(20U);
    assert(LocalRgbGetState() == LOCAL_RGB_STATE_FAULT);
    assert(LocalRgbGetColor() == 0x280000U);
    advance(160U);
    assert(LocalRgbGetColor() == 0U);
    advance(120U);
    assert(LocalRgbGetColor() == 0x280000U);

    puts("local RGB state machine passed");
    return 0;
}
