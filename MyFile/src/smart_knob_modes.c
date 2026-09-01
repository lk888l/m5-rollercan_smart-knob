#include "smart_knob_modes.h"

#include <limits.h>
#include <math.h>
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
        MODE_CONFIG("On/off strong", 0, 0, 1, 60.0f, 10.0f, 1.0f, 0.55f, 0.0f,
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
        MODE_CONFIG("Coarse strong", 0, 0, 31, 10.0f, 8.0f, 1.0f, 0.75f, 0.0f,
                    200, 28.0f, 0.16f, 0.2f, 0.0f, 0.0f),
    [SMART_KNOB_MODE_COARSE_WEAK] =
        MODE_CONFIG("Coarse weak", 0, 0, 31, 10.0f, 0.2f, 1.0f, 0.9f, 0.0f,
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

static int32_t scaled_to_i32(float value)
{
    const double scaled = (double)value * 1000.0;
    if (!isfinite(value)) {
        return 0;
    }
    if (scaled > INT32_MAX) {
        return INT32_MAX;
    }
    if (scaled < INT32_MIN) {
        return INT32_MIN;
    }
    return (int32_t)lround(scaled);
}

static int32_t width_millidegrees(const SmartKnobModeConfig *mode)
{
    return scaled_to_i32(mode->config.position_width_radians * 180.0f / PI);
}

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

static int32_t round_to_step(int32_t value, int32_t step)
{
    if (value >= 0) {
        return ((value + step / 2) / step) * step;
    }
    return ((value - step / 2) / step) * step;
}

static void export_mode_values(
    const SmartKnobModeConfig *mode,
    int32_t values[SMART_KNOB_SNAPSHOT_MODE_VALUE_COUNT])
{
    values[0] = width_millidegrees(mode);
    values[1] = scaled_to_i32(mode->tuning.p_gain);
    values[2] = scaled_to_i32(mode->tuning.d_gain);
    values[3] = scaled_to_i32(mode->tuning.current_scale_a);
    values[4] = scaled_to_i32(mode->tuning.current_limit_a);
    values[5] = mode->tuning.max_current_permille;
    values[6] = scaled_to_i32(mode->tuning.friction_current_a);
    values[7] = scaled_to_i32(mode->tuning.click_current_a);
}

static void import_mode_values(
    SmartKnobModeConfig *mode,
    const int32_t values[SMART_KNOB_SNAPSHOT_MODE_VALUE_COUNT])
{
    const int32_t width = clamp_i32(values[0], 1, 360000);
    mode->config.position_width_radians = DEG_TO_RAD((float)width / 1000.0f);
    mode->tuning.p_gain = (float)(values[1] < 0 ? 0 : values[1]) / 1000.0f;
    mode->tuning.d_gain = (float)(values[2] < 0 ? 0 : values[2]) / 1000.0f;
    mode->tuning.current_scale_a =
        (float)(values[3] < 0 ? 0 : values[3]) / 1000.0f;
    mode->tuning.current_limit_a =
        (float)clamp_i32(values[4], 0, 1200) / 1000.0f;
    mode->tuning.max_current_permille =
        (uint16_t)clamp_i32(values[5], 0, 1000);
    mode->tuning.friction_current_a =
        (float)(values[6] < 0 ? 0 : values[6]) / 1000.0f;
    mode->tuning.click_current_a =
        (float)(values[7] < 0 ? 0 : values[7]) / 1000.0f;
}

void smart_knob_modes_initialize(void)
{
    if (initialized) {
        return;
    }
    memcpy(modes, default_modes, sizeof(modes));
    initialized = true;
}

void smart_knob_modes_reset_defaults(void)
{
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

const SmartKnobModeConfig *smart_knob_mode_get_default(uint8_t mode_index)
{
    if (mode_index >= SMART_KNOB_MODE_COUNT) {
        return NULL;
    }
    return &default_modes[mode_index];
}

SmartKnobModeConfig *smart_knob_mode_get_mutable(uint8_t mode_index)
{
    return (SmartKnobModeConfig *)smart_knob_mode_get(mode_index);
}

bool smart_knob_mode_apply_local_profile(uint8_t mode_index,
                                         uint8_t force_percent,
                                         uint16_t current_limit_ma,
                                         uint8_t step_width_deg)
{
    smart_knob_modes_initialize();
    if (mode_index >= SMART_KNOB_MODE_COUNT) {
        return false;
    }

    if (force_percent < 25U) {
        force_percent = 25U;
    } else if (force_percent > 125U) {
        force_percent = 125U;
    }
    if (current_limit_ma < 100U) {
        current_limit_ma = 100U;
    } else if (current_limit_ma > 450U) {
        current_limit_ma = 450U;
    }
    if (step_width_deg < 1U) {
        step_width_deg = 1U;
    } else if (step_width_deg > 60U) {
        step_width_deg = 60U;
    }

    modes[mode_index] = default_modes[mode_index];
    modes[mode_index].tuning.current_scale_a *=
        (float)force_percent / 100.0f;
    modes[mode_index].tuning.current_limit_a =
        (float)current_limit_ma / 1000.0f;
    modes[mode_index].config.position_width_radians =
        DEG_TO_RAD((float)step_width_deg);
    return true;
}

bool smart_knob_mode_apply_local_edit(const LocalProfileEdit *edit)
{
    if (edit == NULL || edit->mode >= SMART_KNOB_MODE_COUNT) {
        return false;
    }

    smart_knob_modes_initialize();
    SmartKnobModeConfig *mode = &modes[edit->mode];
    const SmartKnobModeConfig *defaults = &default_modes[edit->mode];

    if ((edit->dirty_mask & LOCAL_PROFILE_DIRTY_FORCE) != 0U) {
        const uint8_t force = (uint8_t)clamp_i32(edit->force_percent, 25, 125);
        mode->tuning.current_scale_a =
            defaults->tuning.current_scale_a * (float)force / 100.0f;
    }
    if ((edit->dirty_mask & LOCAL_PROFILE_DIRTY_LIMIT) != 0U) {
        const uint16_t current_ma = (uint16_t)clamp_i32(
            (int32_t)edit->current_limit_10ma * 10, 100, 450);
        mode->tuning.current_limit_a = (float)current_ma / 1000.0f;
    }
    if ((edit->dirty_mask & LOCAL_PROFILE_DIRTY_WIDTH) != 0U) {
        const uint8_t width = (uint8_t)clamp_i32(edit->step_width_deg, 1, 60);
        mode->config.position_width_radians = DEG_TO_RAD((float)width);
    }

    return smart_knob_select_mode(edit->mode);
}

bool smart_knob_snapshot_export(SmartKnobPersistentSnapshot *snapshot)
{
    if (snapshot == NULL) {
        return false;
    }
    smart_knob_modes_initialize();
    memset(snapshot, 0, sizeof(*snapshot));
    const uint8_t active = smart_knob_active_mode();
    snapshot->selected_mode = active < SMART_KNOB_MODE_COUNT
                                  ? active
                                  : (uint8_t)SMART_KNOB_DEFAULT_MODE;

    for (uint8_t index = 0U; index < SMART_KNOB_MODE_COUNT; ++index) {
        export_mode_values(&modes[index], snapshot->mode_values[index]);
    }

    const SmartKnobModeConfig *custom = &modes[SMART_KNOB_MODE_CUSTOM];
    snapshot->custom_values[0] = custom->config.position;
    snapshot->custom_values[1] = custom->config.min_position;
    snapshot->custom_values[2] = custom->config.max_position;
    snapshot->custom_values[3] = width_millidegrees(custom);
    snapshot->custom_values[4] =
        scaled_to_i32(custom->config.detent_strength_unit);
    snapshot->custom_values[5] =
        scaled_to_i32(custom->config.endstop_strength_unit);
    snapshot->custom_values[6] = scaled_to_i32(custom->config.snap_point);
    snapshot->custom_values[7] = scaled_to_i32(custom->config.snap_point_bias);
    snapshot->custom_values[8] = scaled_to_i32(custom->tuning.click_current_a);
    snapshot->custom_values[9] =
        scaled_to_i32(custom->tuning.friction_current_a);
    snapshot->custom_values[10] =
        scaled_to_i32(custom->tuning.current_scale_a);
    snapshot->custom_values[11] = scaled_to_i32(custom->tuning.p_gain);
    snapshot->custom_values[12] = scaled_to_i32(custom->tuning.d_gain);
    snapshot->custom_values[13] = custom->config.led_hue;
    return true;
}

bool smart_knob_snapshot_import(const SmartKnobPersistentSnapshot *snapshot)
{
    if (snapshot == NULL || snapshot->selected_mode >= SMART_KNOB_MODE_COUNT) {
        return false;
    }

    smart_knob_modes_reset_defaults();
    SmartKnobModeConfig *custom = &modes[SMART_KNOB_MODE_CUSTOM];
    custom->config.position = snapshot->custom_values[0];
    custom->config.min_position = snapshot->custom_values[1];
    custom->config.max_position = snapshot->custom_values[2];
    custom->config.position_width_radians = DEG_TO_RAD(
        (float)clamp_i32(snapshot->custom_values[3], 1, 360000) / 1000.0f);
    custom->config.detent_strength_unit =
        (float)(snapshot->custom_values[4] < 0 ? 0 : snapshot->custom_values[4]) /
        1000.0f;
    custom->config.endstop_strength_unit =
        (float)(snapshot->custom_values[5] < 0 ? 0 : snapshot->custom_values[5]) /
        1000.0f;
    custom->config.snap_point =
        (float)clamp_i32(snapshot->custom_values[6], 500, 1500) / 1000.0f;
    custom->config.snap_point_bias =
        (float)clamp_i32(snapshot->custom_values[7], -1000, 1000) / 1000.0f;
    custom->tuning.click_current_a =
        (float)(snapshot->custom_values[8] < 0 ? 0 : snapshot->custom_values[8]) /
        1000.0f;
    custom->tuning.friction_current_a =
        (float)(snapshot->custom_values[9] < 0 ? 0 : snapshot->custom_values[9]) /
        1000.0f;
    custom->tuning.current_scale_a =
        (float)(snapshot->custom_values[10] < 0 ? 0 : snapshot->custom_values[10]) /
        1000.0f;
    custom->tuning.p_gain =
        (float)(snapshot->custom_values[11] < 0 ? 0 : snapshot->custom_values[11]) /
        1000.0f;
    custom->tuning.d_gain =
        (float)(snapshot->custom_values[12] < 0 ? 0 : snapshot->custom_values[12]) /
        1000.0f;
    custom->config.led_hue = clamp_i32(snapshot->custom_values[13], 0, 255);

    for (uint8_t index = 0U; index < SMART_KNOB_MODE_COUNT; ++index) {
        import_mode_values(&modes[index], snapshot->mode_values[index]);
    }
    return smart_knob_select_mode(snapshot->selected_mode);
}

bool smart_knob_mode_local_projection(uint8_t mode_index,
                                      LocalProfileEdit *profile,
                                      uint8_t *unrepresentable_mask)
{
    if (profile == NULL || mode_index >= SMART_KNOB_MODE_COUNT) {
        return false;
    }
    smart_knob_modes_initialize();
    const SmartKnobModeConfig *mode = &modes[mode_index];
    const SmartKnobModeConfig *defaults = &default_modes[mode_index];
    uint8_t mask = 0U;

    const float default_scale_a = defaults->tuning.current_scale_a;
    const float actual_scale_a = mode->tuning.current_scale_a;
    int32_t force = 100;
    if (default_scale_a > 0.0f && isfinite(actual_scale_a)) {
        const float ratio = actual_scale_a * 100.0f / default_scale_a;
        if (ratio <= 25.0f) {
            force = 25;
        } else if (ratio >= 125.0f) {
            force = 125;
        } else {
            force = round_to_step((int32_t)lroundf(ratio), 5);
        }
    }
    force = clamp_i32(force, 25, 125);
    const float represented_scale_a =
        default_scale_a * (float)force / 100.0f;
    if (!isfinite(actual_scale_a) ||
        fabsf(represented_scale_a - actual_scale_a) > 0.00001f) {
        mask |= SMART_KNOB_LOCAL_UNREPRESENTABLE_FORCE;
    }

    const int32_t actual_limit_ma = scaled_to_i32(mode->tuning.current_limit_a);
    const int32_t limit_ma =
        clamp_i32(round_to_step(actual_limit_ma, 50), 100, 450);
    if (limit_ma != actual_limit_ma) {
        mask |= SMART_KNOB_LOCAL_UNREPRESENTABLE_LIMIT;
    }

    const int32_t actual_width = width_millidegrees(mode);
    const int32_t width_deg =
        clamp_i32(round_to_step(actual_width, 1000) / 1000, 1, 60);
    if (width_deg * 1000 != actual_width) {
        mask |= SMART_KNOB_LOCAL_UNREPRESENTABLE_WIDTH;
    }

    int32_t current_values[SMART_KNOB_SNAPSHOT_MODE_VALUE_COUNT];
    int32_t default_values[SMART_KNOB_SNAPSHOT_MODE_VALUE_COUNT];
    export_mode_values(mode, current_values);
    export_mode_values(defaults, default_values);
    if (current_values[1] != default_values[1] ||
        current_values[2] != default_values[2] ||
        current_values[5] != default_values[5] ||
        current_values[6] != default_values[6] ||
        current_values[7] != default_values[7]) {
        mask |= SMART_KNOB_LOCAL_UNREPRESENTABLE_ADVANCED;
    }

    if (mode_index == SMART_KNOB_MODE_CUSTOM &&
        (mode->config.position != defaults->config.position ||
         mode->config.min_position != defaults->config.min_position ||
         mode->config.max_position != defaults->config.max_position ||
         scaled_to_i32(mode->config.detent_strength_unit) !=
             scaled_to_i32(defaults->config.detent_strength_unit) ||
         scaled_to_i32(mode->config.endstop_strength_unit) !=
             scaled_to_i32(defaults->config.endstop_strength_unit) ||
         scaled_to_i32(mode->config.snap_point) !=
             scaled_to_i32(defaults->config.snap_point) ||
         scaled_to_i32(mode->config.snap_point_bias) !=
             scaled_to_i32(defaults->config.snap_point_bias) ||
         mode->config.led_hue != defaults->config.led_hue)) {
        mask |= SMART_KNOB_LOCAL_UNREPRESENTABLE_ADVANCED;
    }

    profile->mode = mode_index;
    profile->force_percent = (uint8_t)force;
    profile->current_limit_10ma = (uint8_t)(limit_ma / 10);
    profile->step_width_deg = (uint8_t)width_deg;
    profile->dirty_mask = 0U;
    if (unrepresentable_mask != NULL) {
        *unrepresentable_mask = mask;
    }
    return true;
}

uint8_t smart_knob_mode_default_width_deg(uint8_t mode_index)
{
    if (mode_index >= SMART_KNOB_MODE_COUNT) {
        mode_index = (uint8_t)SMART_KNOB_DEFAULT_MODE;
    }
    float width_deg = default_modes[mode_index].config.position_width_radians *
                      180.0f / PI;
    if (width_deg < 1.0f) {
        width_deg = 1.0f;
    } else if (width_deg > 60.0f) {
        width_deg = 60.0f;
    }
    return (uint8_t)(width_deg + 0.5f);
}
