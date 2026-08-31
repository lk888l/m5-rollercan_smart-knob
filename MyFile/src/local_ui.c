#include "local_ui.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "app_rtos.h"
#include "button.h"
#include "mysys.h"
#include "oled_u8g2.h"
#include "smart_knob.h"
#include "smart_knob_modes.h"

#define LOCAL_UI_FRAME_PERIOD_MS 50U
#define LOCAL_UI_ROOT_ITEM_COUNT 5
#define LOCAL_UI_FORCE_MIN 25
#define LOCAL_UI_FORCE_MAX 125
#define LOCAL_UI_FORCE_STEP 5
#define LOCAL_UI_LIMIT_MIN_UNITS 10
#define LOCAL_UI_LIMIT_MAX_UNITS 45
#define LOCAL_UI_LIMIT_STEP_UNITS 5
#define LOCAL_UI_WIDTH_MIN_DEG 1
#define LOCAL_UI_WIDTH_MAX_DEG 60

typedef enum {
    LOCAL_UI_DASHBOARD = 0,
    LOCAL_UI_ROOT_MENU,
    LOCAL_UI_EDIT_MODE,
    LOCAL_UI_EDIT_FORCE,
    LOCAL_UI_EDIT_WIDTH,
    LOCAL_UI_EDIT_LIMIT,
} LocalUiScreen;

typedef struct {
    uint8_t mode;
    uint8_t force_percent;
    uint8_t current_limit_10ma;
    uint8_t step_width_deg;
} LocalUiProfile;

static const char *const root_items[LOCAL_UI_ROOT_ITEM_COUNT] = {
    "MODE", "FORCE", "STEP", "LIMIT", "SAVE"
};

static const char *const mode_names[SMART_KNOB_MODE_COUNT] = {
    "CUSTOM", "FREE", "BOUND", "MULTI", "ON/OFF", "CENTER",
    "FINE", "F-DET", "COARSE+", "COARSE-", "MAGNET", "C-DET"
};

static const char *const dashboard_mode_names[SMART_KNOB_MODE_COUNT] = {
    "CUS", "FREE", "BND", "MULT", "ON", "CTR",
    "FINE", "FDET", "C+", "C-", "MAG", "CDET"
};

static LocalUiScreen screen = LOCAL_UI_DASHBOARD;
static LocalUiProfile profile;
static LocalUiProfile edit_backup;
static int8_t root_cursor;
static int32_t last_navigation_position;
static uint8_t navigation_position_valid;
static uint32_t next_frame_ms;

static int32_t clamp_i32(int32_t value, int32_t lower, int32_t upper)
{
    if (value < lower) {
        return lower;
    }
    if (value > upper) {
        return upper;
    }
    return value;
}

static int32_t wrap_i32(int32_t value, int32_t count)
{
    while (value < 0) {
        value += count;
    }
    while (value >= count) {
        value -= count;
    }
    return value;
}

static uint32_t pack_profile(const LocalUiProfile *value)
{
    return (uint32_t)value->mode |
           ((uint32_t)value->force_percent << 8) |
           ((uint32_t)value->current_limit_10ma << 16) |
           ((uint32_t)value->step_width_deg << 24);
}

static LocalUiProfile unpack_profile(uint32_t packed)
{
    LocalUiProfile value;
    value.mode = (uint8_t)packed;
    value.force_percent = (uint8_t)(packed >> 8);
    value.current_limit_10ma = (uint8_t)(packed >> 16);
    value.step_width_deg = (uint8_t)(packed >> 24);
    return value;
}

static void draw_centered(const char *text, uint8_t baseline)
{
    const uint16_t width = u8g2_GetStrWidth(&u8g2, text);
    const uint8_t x = width < 64U ? (uint8_t)((64U - width) / 2U) : 0U;
    u8g2_DrawStr(&u8g2, x, baseline, text);
}

static void draw_dashboard(const SmartKnobRuntimeState *state)
{
    const int16_t center_x = 32;
    const int16_t center_y = 24;
    const int16_t radius = 22;
    /* Bottom-left -> top -> bottom-right, like a signed speedometer. */
    const float start_angle = 2.35619449f;
    const float sweep_angle = 4.71238898f;
    char text[20];

    u8g2_ClearBuffer(&u8g2);
    u8g2_SetDrawColor(&u8g2, 1);
    u8g2_DrawCircle(&u8g2, center_x, center_y, radius, U8G2_DRAW_ALL);

    for (uint8_t tick = 0U; tick < 9U; ++tick) {
        const float angle = start_angle + sweep_angle * (float)tick / 8.0f;
        const int16_t x_outer = center_x + (int16_t)lroundf(cosf(angle) * 20.0f);
        const int16_t y_outer = center_y + (int16_t)lroundf(sinf(angle) * 20.0f);
        const int16_t x_inner = center_x + (int16_t)lroundf(cosf(angle) * 17.0f);
        const int16_t y_inner = center_y + (int16_t)lroundf(sinf(angle) * 17.0f);
        u8g2_DrawLine(&u8g2, x_inner, y_inner, x_outer, y_outer);
    }

    float velocity = state->shaft_velocity_rad_s;
    if (velocity < -24.0f) {
        velocity = -24.0f;
    } else if (velocity > 24.0f) {
        velocity = 24.0f;
    }
    const float needle_angle = start_angle +
        sweep_angle * (velocity + 24.0f) / 48.0f;
    const int16_t needle_x = center_x +
        (int16_t)lroundf(cosf(needle_angle) * 15.0f);
    const int16_t needle_y = center_y +
        (int16_t)lroundf(sinf(needle_angle) * 15.0f);
    u8g2_DrawLine(&u8g2, center_x, center_y, needle_x, needle_y);
    u8g2_DrawLine(&u8g2, center_x + 1, center_y, needle_x, needle_y);
    u8g2_DrawDisc(&u8g2, center_x, center_y, 2U, U8G2_DRAW_ALL);

    const uint8_t mode = state->active_mode < SMART_KNOB_MODE_COUNT
                             ? state->active_mode
                             : profile.mode;
    u8g2_SetFont(&u8g2, u8g2_font_4x6_tr);
    uint16_t text_width = u8g2_GetStrWidth(&u8g2, dashboard_mode_names[mode]);
    uint8_t text_x = text_width < 64U ? (uint8_t)((64U - text_width) / 2U) : 0U;
    u8g2_SetDrawColor(&u8g2, 0);
    u8g2_DrawBox(&u8g2, text_x > 1U ? text_x - 1U : 0U, 3U,
                  (uint8_t)text_width + 2U, 7U);
    u8g2_SetDrawColor(&u8g2, 1);
    u8g2_DrawStr(&u8g2, text_x, 9U, dashboard_mode_names[mode]);

    if (state->current_position >= -999 && state->current_position <= 999) {
        u8g2_SetFont(&u8g2, u8g2_font_6x13_tr);
    } else {
        u8g2_SetFont(&u8g2, u8g2_font_4x6_tr);
    }
    snprintf(text, sizeof(text), "%ld", (long)state->current_position);
    text_width = u8g2_GetStrWidth(&u8g2, text);
    text_x = text_width < 64U ? (uint8_t)((64U - text_width) / 2U) : 0U;
    u8g2_SetDrawColor(&u8g2, 0);
    u8g2_DrawBox(&u8g2, text_x > 2U ? text_x - 2U : 0U, 27U,
                  (uint8_t)text_width + 4U, 12U);
    u8g2_SetDrawColor(&u8g2, 1);
    u8g2_DrawStr(&u8g2, text_x, 38U, text);

    if (error_code != ERR_NONE) {
        const char *fault = over_vol_flag ? "OVER VOLT" :
                            (err_stalled_flag ? "STALLED" : "FAULT");
        u8g2_SetDrawColor(&u8g2, 1);
        u8g2_DrawBox(&u8g2, 7U, 16U, 50U, 14U);
        u8g2_SetDrawColor(&u8g2, 0);
        u8g2_SetFont(&u8g2, u8g2_font_5x8_tr);
        draw_centered(fault, 26U);
    } else if (!motor_output) {
        u8g2_SetDrawColor(&u8g2, 1);
        u8g2_DrawBox(&u8g2, 11U, 16U, 42U, 14U);
        u8g2_SetDrawColor(&u8g2, 0);
        u8g2_SetFont(&u8g2, u8g2_font_5x8_tr);
        draw_centered("PAUSED", 26U);
    }
    u8g2_SetDrawColor(&u8g2, 1);
    u8g2_SendBuffer(&u8g2);
}

static void draw_three_row_list(const char *title,
                                const char *previous,
                                const char *selected,
                                const char *next)
{
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetDrawColor(&u8g2, 1);
    u8g2_DrawBox(&u8g2, 0U, 0U, 64U, 10U);
    u8g2_SetDrawColor(&u8g2, 0);
    u8g2_SetFont(&u8g2, u8g2_font_5x8_tr);
    draw_centered(title, 8U);

    u8g2_SetDrawColor(&u8g2, 1);
    u8g2_SetFont(&u8g2, u8g2_font_5x8_tr);
    draw_centered(previous, 19U);
    u8g2_DrawBox(&u8g2, 0U, 21U, 64U, 11U);
    u8g2_SetDrawColor(&u8g2, 0);
    draw_centered(selected, 30U);
    u8g2_SetDrawColor(&u8g2, 1);
    draw_centered(next, 43U);
    u8g2_SendBuffer(&u8g2);
}

static void draw_root_menu(void)
{
    const int32_t previous = wrap_i32(root_cursor - 1, LOCAL_UI_ROOT_ITEM_COUNT);
    const int32_t next = wrap_i32(root_cursor + 1, LOCAL_UI_ROOT_ITEM_COUNT);
    draw_three_row_list("LOCAL MENU",
                        root_items[previous],
                        root_items[root_cursor],
                        root_items[next]);
}

static void draw_mode_editor(void)
{
    const int32_t previous = wrap_i32((int32_t)profile.mode - 1,
                                      SMART_KNOB_MODE_COUNT);
    const int32_t next = wrap_i32((int32_t)profile.mode + 1,
                                  SMART_KNOB_MODE_COUNT);
    draw_three_row_list("MODE",
                        mode_names[previous],
                        mode_names[profile.mode],
                        mode_names[next]);
}

static void draw_numeric_editor(const char *title, const char *value)
{
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetDrawColor(&u8g2, 1);
    u8g2_DrawBox(&u8g2, 0U, 0U, 64U, 10U);
    u8g2_SetDrawColor(&u8g2, 0);
    u8g2_SetFont(&u8g2, u8g2_font_5x8_tr);
    draw_centered(title, 8U);
    u8g2_SetDrawColor(&u8g2, 1);
    u8g2_SetFont(&u8g2, u8g2_font_6x13_tr);
    draw_centered(value, 31U);
    u8g2_SetFont(&u8g2, u8g2_font_4x6_tr);
    draw_centered("TURN  DBL OK", 45U);
    u8g2_SendBuffer(&u8g2);
}

static void draw_current_screen(const SmartKnobRuntimeState *state)
{
    char value[20];
    switch (screen) {
    case LOCAL_UI_DASHBOARD:
        draw_dashboard(state);
        break;
    case LOCAL_UI_ROOT_MENU:
        draw_root_menu();
        break;
    case LOCAL_UI_EDIT_MODE:
        draw_mode_editor();
        break;
    case LOCAL_UI_EDIT_FORCE:
        snprintf(value, sizeof(value), "%u%%", profile.force_percent);
        draw_numeric_editor("FORCE", value);
        break;
    case LOCAL_UI_EDIT_WIDTH:
        snprintf(value, sizeof(value), "%u deg", profile.step_width_deg);
        draw_numeric_editor("STEP", value);
        break;
    case LOCAL_UI_EDIT_LIMIT:
        snprintf(value, sizeof(value), "%u mA",
                 (unsigned int)profile.current_limit_10ma * 10U);
        draw_numeric_editor("LIMIT", value);
        break;
    default:
        screen = LOCAL_UI_DASHBOARD;
        break;
    }
}

static void apply_navigation_delta(int32_t delta)
{
    delta = clamp_i32(delta, -12, 12);
    switch (screen) {
    case LOCAL_UI_ROOT_MENU:
        root_cursor = (int8_t)wrap_i32((int32_t)root_cursor + delta,
                                       LOCAL_UI_ROOT_ITEM_COUNT);
        break;
    case LOCAL_UI_EDIT_MODE:
        profile.mode = (uint8_t)wrap_i32((int32_t)profile.mode + delta,
                                         SMART_KNOB_MODE_COUNT);
        profile.step_width_deg = smart_knob_mode_default_width_deg(profile.mode);
        break;
    case LOCAL_UI_EDIT_FORCE:
        profile.force_percent = (uint8_t)clamp_i32(
            (int32_t)profile.force_percent + delta * LOCAL_UI_FORCE_STEP,
            LOCAL_UI_FORCE_MIN,
            LOCAL_UI_FORCE_MAX);
        break;
    case LOCAL_UI_EDIT_WIDTH:
        profile.step_width_deg = (uint8_t)clamp_i32(
            (int32_t)profile.step_width_deg + delta,
            LOCAL_UI_WIDTH_MIN_DEG,
            LOCAL_UI_WIDTH_MAX_DEG);
        break;
    case LOCAL_UI_EDIT_LIMIT:
        profile.current_limit_10ma = (uint8_t)clamp_i32(
            (int32_t)profile.current_limit_10ma +
                delta * LOCAL_UI_LIMIT_STEP_UNITS,
            LOCAL_UI_LIMIT_MIN_UNITS,
            LOCAL_UI_LIMIT_MAX_UNITS);
        break;
    default:
        break;
    }
    next_frame_ms = 0U;
}

static void update_navigation_position(const SmartKnobRuntimeState *state)
{
    if (screen == LOCAL_UI_DASHBOARD ||
        state->active_mode != SMART_KNOB_NAVIGATION_MODE) {
        navigation_position_valid = 0U;
        return;
    }
    if (!navigation_position_valid) {
        last_navigation_position = state->current_position;
        navigation_position_valid = 1U;
        return;
    }

    const int32_t delta = state->current_position - last_navigation_position;
    if (delta != 0) {
        last_navigation_position = state->current_position;
        apply_navigation_delta(delta);
    }
}

static void request_save_and_exit(void)
{
    if (App_PostControlCommand(APP_CONTROL_COMMAND_LOCAL_MENU_EXIT,
                               (int32_t)pack_profile(&profile))) {
        screen = LOCAL_UI_DASHBOARD;
        navigation_position_valid = 0U;
        next_frame_ms = 0U;
    }
}

static void handle_long_press(void)
{
    if (screen == LOCAL_UI_DASHBOARD) {
        if (error_code == ERR_NONE &&
            App_PostControlCommand(APP_CONTROL_COMMAND_LOCAL_MENU_ENTER, 0)) {
            profile = unpack_profile(MysysLocalProfilePacked());
            root_cursor = 0;
            screen = LOCAL_UI_ROOT_MENU;
            navigation_position_valid = 0U;
            next_frame_ms = 0U;
        }
    } else if (screen == LOCAL_UI_ROOT_MENU) {
        request_save_and_exit();
    } else {
        profile = edit_backup;
        screen = LOCAL_UI_ROOT_MENU;
        next_frame_ms = 0U;
    }
}

static void handle_double_click(void)
{
    if (screen == LOCAL_UI_DASHBOARD) {
        (void)App_PostControlCommand(APP_CONTROL_COMMAND_LOCAL_TOGGLE_OUTPUT, 0);
        next_frame_ms = 0U;
        return;
    }
    if (screen != LOCAL_UI_ROOT_MENU) {
        screen = LOCAL_UI_ROOT_MENU;
        next_frame_ms = 0U;
        return;
    }

    edit_backup = profile;
    switch (root_cursor) {
    case 0: screen = LOCAL_UI_EDIT_MODE; break;
    case 1: screen = LOCAL_UI_EDIT_FORCE; break;
    case 2: screen = LOCAL_UI_EDIT_WIDTH; break;
    case 3: screen = LOCAL_UI_EDIT_LIMIT; break;
    case 4: request_save_and_exit(); return;
    default: root_cursor = 0; break;
    }
    next_frame_ms = 0U;
}

void LocalUiInitialize(void)
{
    profile = unpack_profile(MysysLocalProfilePacked());
    screen = LOCAL_UI_DASHBOARD;
    navigation_position_valid = 0U;
    next_frame_ms = 0U;
}

void LocalUiTask(void)
{
    SmartKnobRuntimeState state = {0};
    (void)smart_knob_get_runtime_state(&state);

    button_update();
    update_navigation_position(&state);

    const uint8_t long_press = my_button.was_longpress;
    const uint8_t double_click = my_button.was_double_click;
    my_button.was_longpress = 0U;
    my_button.was_double_click = 0U;
    my_button.was_click = 0U;
    my_button.is_longlongpressed = 0U;
    my_button.was_longlongpress = 0U;

    if (long_press) {
        handle_long_press();
    } else if (double_click) {
        handle_double_click();
    }

    const uint32_t now_ms = HAL_GetTick();
    if (next_frame_ms == 0U || (int32_t)(now_ms - next_frame_ms) >= 0) {
        draw_current_screen(&state);
        next_frame_ms = now_ms + LOCAL_UI_FRAME_PERIOD_MS;
    }
}

uint8_t LocalUiIsMenuActive(void)
{
    return screen != LOCAL_UI_DASHBOARD;
}

uint8_t LocalUiIsEditing(void)
{
    return screen == LOCAL_UI_EDIT_MODE ||
           screen == LOCAL_UI_EDIT_FORCE ||
           screen == LOCAL_UI_EDIT_WIDTH ||
           screen == LOCAL_UI_EDIT_LIMIT;
}
