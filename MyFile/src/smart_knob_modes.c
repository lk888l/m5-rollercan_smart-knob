#include "smart_knob_modes.h"

#include <string.h>

#define DEG_TO_RAD(deg) ((deg) * PI / 180.0f)
#define DEFAULT_CURRENT_LIMIT_A 0.45f
#define DEFAULT_MAX_CURRENT_PERMILLE 700U

#define MODE_CONFIG(name, pos, min_pos, max_pos, width_deg, detent, endstop, snap, bias, hue, p, d, scale, friction, click) \
    {                                                                                                                   \
        .config = {                                                                                                     \
            .position = (pos),                                                                                          \
            .sub_position_unit = 0.0f,                                                                                  \
            .position_nonce = 0U,                                                                                       \
            .min_position = (min_pos),                                                                                  \
            .max_position = (max_pos),                                                                                  \
            .position_width_radians = DEG_TO_RAD(width_deg),                                                            \
            .detent_strength_unit = (detent),                                                                           \
            .endstop_strength_unit = (endstop),                                                                         \
            .snap_point = (snap),                                                                                       \
            .text = name,                                                                                               \
            .snap_point_bias = (bias),                                                                                  \
            .led_hue = (hue),                                                                                           \
        },                                                                                                              \
        .tuning = {                                                                                                     \
            .p_gain = (p),                                                                                              \
            .d_gain = (d),                                                                                              \
            .current_scale_a = (scale),                                                                                 \
            .current_limit_a = DEFAULT_CURRENT_LIMIT_A,                                                                 \
            .max_current_permille = DEFAULT_MAX_CURRENT_PERMILLE,                                                       \
            .friction_current_a = (friction),                                                                           \
            .click_current_a = (click),                                                                                 \
        },                                                                                                              \
    }

static const SmartKnobModeConfig default_modes[SMART_KNOB_MODE_COUNT] = {
    [SMART_KNOB_MODE_CUSTOM] =
        MODE_CONFIG("Custom", 0, 0, -1, 10.0f, 0.0f, 1.0f, 0.55f, 0.0f,
                    120, 0.0f, 0.0f, 0.0875f, 0.0f, 0.0f),
    [SMART_KNOB_MODE_UNBOUNDED_SMOOTH] =
        MODE_CONFIG("Unbounded smooth", 0, 0, -1, 10.0f, 0.0f, 1.0f, 0.75f, 0.0f,
                    200, 0.0f, 0.0f, 0.0375f, 0.02f, 0.0f),
    [SMART_KNOB_MODE_BOUNDED_SMOOTH] =
        MODE_CONFIG("Bounded 0-10", 0, 0, 10, 10.0f, 0.0f, 1.0f, 1.1f, 0.0f,
                    0, 0.0f, 0.0f, 0.0625f, 0.0f, 0.0f),
    [SMART_KNOB_MODE_MULTI_REV_SMOOTH] =
        MODE_CONFIG("Multi-rev smooth", 0, 0, 72, 10.0f, 0.0f, 5.0f, 0.75f, 0.0f,
                    73, 0.0f, 0.0f, 0.0375f, 0.0f, 0.0f),
    [SMART_KNOB_MODE_ON_OFF_STRONG] =
        MODE_CONFIG("On/off strong", 0, 0, 1, 60.0f, 1.0f, 1.0f, 0.55f, 0.0f,
                    157, 38.0f, 0.55f, 0.1f, 0.0f, 0.0f),
    [SMART_KNOB_MODE_RETURN_TO_CENTER] =
        MODE_CONFIG("Return to center", 0, 0, 0, 60.0f, 0.01f, 0.6f, 1.1f, 0.0f,
                    45, 40.0f, 0.1f, 0.2f, 0.0075f, 0.0f),
    [SMART_KNOB_MODE_FINE_SMOOTH] =
        MODE_CONFIG("Fine smooth", 127, 0, 255, 1.0f, 0.0f, 1.0f, 1.1f, 0.0f,
                    219, 0.0f, 0.1f, 0.075f, 0.0f, 0.0f),
    [SMART_KNOB_MODE_FINE_DETENTS] =
        MODE_CONFIG("Fine detents", 127, 0, 255, 1.0f, 1.0f, 1.0f, 0.9f, 0.0f,
                    25, 0.0f, 0.1f, 0.0625f, 0.0f, 0.1f),
    [SMART_KNOB_MODE_COARSE_STRONG] =
        MODE_CONFIG("Coarse strong", 0, 0, 31, 8.225806452f, 2.5f, 1.0f, 0.75f, 0.0f,
                    200, 28.0f, 0.16f, 0.2f, 0.0f, 0.0f),
    [SMART_KNOB_MODE_COARSE_WEAK] =
        MODE_CONFIG("Coarse weak", 0, 0, 31, 8.225806452f, 0.2f, 1.0f, 0.9f, 0.0f,
                    0, 5.0f, 0.16f, 0.2f, 0.0f, 0.35f),
    [SMART_KNOB_MODE_MAGNETIC] = {
        .config = {
            .position = 0,
            .min_position = 0,
            .max_position = 31,
            .position_width_radians = DEG_TO_RAD(7.0f),
            .detent_strength_unit = 2.5f,
            .endstop_strength_unit = 1.0f,
            .snap_point = 0.7f,
            .text = "Magnetic detents",
            .detent_positions_count = 4U,
            .detent_positions = {2, 10, 21, 22},
            .led_hue = 73,
        },
        .tuning = {
            .p_gain = 40.0f,
            .d_gain = 0.2f,
            .current_scale_a = 0.2f,
            .current_limit_a = DEFAULT_CURRENT_LIMIT_A,
            .max_current_permille = DEFAULT_MAX_CURRENT_PERMILLE,
        },
    },
    [SMART_KNOB_MODE_RETURN_TO_CENTER_DETENTS] =
        MODE_CONFIG("Center detents", 0, -6, 6, 60.0f, 1.0f, 1.0f, 0.55f, 0.4f,
                    157, 10.0f, 0.1f, 0.2f, 0.0f, 0.0f),
};

static SmartKnobModeConfig modes[SMART_KNOB_MODE_COUNT];
static bool initialized;

void smart_knob_modes_initialize(void)
{
    if (initialized) {
        return;
    }
    memcpy(modes, default_modes, sizeof(modes));
    initialized = true;
}

uint8_t smart_knob_modes_count(void)
{
    return (uint8_t)SMART_KNOB_MODE_COUNT;
}

const SmartKnobModeConfig *smart_knob_mode_get(uint8_t mode_index)
{
    smart_knob_modes_initialize();
    if (mode_index >= SMART_KNOB_MODE_COUNT) {
        return NULL;
    }
    return &modes[mode_index];
}

SmartKnobModeConfig *smart_knob_mode_get_mutable(uint8_t mode_index)
{
    return (SmartKnobModeConfig *)smart_knob_mode_get(mode_index);
}
