/*
 * Firmware-owned SmartKnob controller.
 *
 * Detent and endstop behavior is derived from scottbez1/smartknob.  The final
 * actuator is current rather than voltage/torque, so tuning is expressed in
 * amperes and clamped to the RollerCAN 1.2 A current limit.
 */
#include "smart_knob.h"

#include <math.h>
#include <string.h>

#include "motordriver.h"
#include "mysys.h"
#include "smart_knob_modes.h"

#define SMART_KNOB_DEAD_ZONE_DETENT_PERCENT 0.2f
#define SMART_KNOB_DEAD_ZONE_RAD DEG_TO_RAD(1.0f)
#define SMART_KNOB_IDLE_VELOCITY_RAD_S 0.05f
#define SMART_KNOB_IDLE_DELAY_MS 500U
#define SMART_KNOB_IDLE_MAX_ANGLE_RAD DEG_TO_RAD(5.0f)
#define SMART_KNOB_POSITION_FILTER_ALPHA_BASE 0.15f
#define SMART_KNOB_IDLE_VELOCITY_ALPHA_BASE 0.001f
#define SMART_KNOB_IDLE_CORRECTION_ALPHA_BASE 0.0005f
#define SMART_KNOB_LEGACY_RATE_HZ (56000.0f / 11.0f)
#define SMART_KNOB_SETTLE_TIME_MS 300U
#define SMART_KNOB_MAX_SAMPLE_SPEED_RAD_S 80.0f
#define SMART_KNOB_MIN_SAMPLE_STEP_RAD DEG_TO_RAD(3.0f)
#define SMART_KNOB_SAMPLE_RECOVERY_COUNT 2U
#define SMART_KNOB_SPEED_DERATE_START_RAD_S 20.0f
#define SMART_KNOB_SPEED_DERATE_STOP_RAD_S 45.0f
#define SMART_KNOB_MAX_HAPTIC_SPEED_RAD_S 60.0f
#define SMART_KNOB_HAPTIC_RESUME_SPEED_RAD_S 40.0f
#define SMART_KNOB_PID_LIMIT 10.0f
#define SMART_KNOB_HARD_CURRENT_LIMIT_A 1.2f
#define SMART_KNOB_CURRENT_DEADBAND_A 0.09f
#define SMART_KNOB_CLICK_PHASE_MS 2U
#define SMART_KNOB_CLICK_TOTAL_MS 4U
#define SMART_KNOB_TELEMETRY_DEFAULT_RATE_HZ 50U
#define SMART_KNOB_TELEMETRY_MAX_RATE_HZ 100U
#define SMART_KNOB_TELEMETRY_DEFAULT_ENABLED 1U
#define SMART_KNOB_SCALE 1000.0f
#define DEG_TO_RAD(deg) ((deg) * PI / 180.0f)
#define RAD_TO_DEG(rad) ((rad) * 180.0f / PI)

int32_t current_position;
float latest_sub_position_unit;

static uint8_t active_mode = (uint8_t)SMART_KNOB_DEFAULT_MODE;
static bool controller_initialized;
static float current_detent_center;
static float filtered_position_rad;
static float last_raw_position_rad;
static uint32_t last_sample_us;
static uint32_t enable_after_ms;
static bool sample_valid;
static bool sample_recovery_pending;
static uint8_t sample_recovery_count;
static bool high_speed_active;
static float idle_velocity_ewma;
static uint32_t idle_start_ms;
static float position_filter_alpha = SMART_KNOB_POSITION_FILTER_ALPHA_BASE;
static float idle_velocity_alpha = SMART_KNOB_IDLE_VELOCITY_ALPHA_BASE;
static float idle_correction_alpha = SMART_KNOB_IDLE_CORRECTION_ALPHA_BASE;
static bool out_of_bounds;
static float last_command_current_ma;

static int32_t previous_click_position;
static uint32_t click_started_ms;
static float click_direction = 1.0f;
static bool click_running;

static volatile uint32_t runtime_state_version;
static SmartKnobRuntimeState runtime_state;

static volatile uint8_t telemetry_enable = SMART_KNOB_TELEMETRY_DEFAULT_ENABLED;
static volatile uint8_t telemetry_rate_hz = SMART_KNOB_TELEMETRY_DEFAULT_RATE_HZ;
static volatile uint8_t telemetry_host_id;
static uint32_t telemetry_last_ms;
static bool telemetry_started;
static uint8_t telemetry_sequence;
static uint8_t telemetry_pending_type;
static SmartKnobRuntimeState telemetry_latched_state;

static float clampf(float value, float lower, float upper)
{
    if (value < lower) {
        return lower;
    }
    if (value > upper) {
        return upper;
    }
    return value;
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

static int16_t saturate_i16(float value)
{
    if (value > 32767.0f) {
        return INT16_MAX;
    }
    if (value < -32768.0f) {
        return INT16_MIN;
    }
    return (int16_t)lroundf(value);
}

static int32_t position_count(const PB_SmartKnobConfig *config)
{
    int64_t count = (int64_t)config->max_position - config->min_position + 1;
    if (count <= 0 || count > INT32_MAX) {
        return 0;
    }
    return (int32_t)count;
}

static void sanitize_mode(SmartKnobModeConfig *mode)
{
    PB_SmartKnobConfig *config = &mode->config;
    SmartKnobTuning *tuning = &mode->tuning;

    if (!isfinite(config->position_width_radians) || config->position_width_radians < 0.001f) {
        config->position_width_radians = DEG_TO_RAD(1.0f);
    }
    config->detent_strength_unit = fmaxf(0.0f, config->detent_strength_unit);
    config->endstop_strength_unit = fmaxf(0.0f, config->endstop_strength_unit);
    config->snap_point = clampf(config->snap_point, 0.5f, 1.5f);
    config->snap_point_bias = clampf(config->snap_point_bias, -1.0f, 1.0f);
    if (config->detent_positions_count > SMART_KNOB_MAX_DETENT_POSITIONS) {
        config->detent_positions_count = SMART_KNOB_MAX_DETENT_POSITIONS;
    }
    if (config->min_position <= config->max_position) {
        config->position = clamp_i32(config->position,
                                    config->min_position,
                                    config->max_position);
    }

    tuning->p_gain = fmaxf(0.0f, tuning->p_gain);
    tuning->d_gain = fmaxf(0.0f, tuning->d_gain);
    tuning->current_scale_a = fmaxf(0.0f, tuning->current_scale_a);
    tuning->current_limit_a = clampf(tuning->current_limit_a,
                                     0.0f,
                                     SMART_KNOB_HARD_CURRENT_LIMIT_A);
    if (tuning->max_current_permille > 1000U) {
        tuning->max_current_permille = 1000U;
    }
    tuning->friction_current_a = fmaxf(0.0f, tuning->friction_current_a);
    tuning->click_current_a = fmaxf(0.0f, tuning->click_current_a);
}

static SmartKnobModeConfig *active_mode_mutable(void)
{
    return smart_knob_mode_get_mutable(active_mode);
}

static void reset_click(void)
{
    previous_click_position = current_position;
    click_started_ms = 0U;
    click_direction = 1.0f;
    click_running = false;
}

static void reanchor_active_mode(bool reset_position)
{
    SmartKnobModeConfig *mode = active_mode_mutable();
    if (mode == NULL) {
        return;
    }
    sanitize_mode(mode);
    if (reset_position) {
        current_position = mode->config.position;
    }
    if (mode->config.min_position <= mode->config.max_position) {
        current_position = clamp_i32(current_position,
                                     mode->config.min_position,
                                     mode->config.max_position);
    }
    latest_sub_position_unit = mode->config.sub_position_unit;
    current_detent_center = filtered_position_rad +
                            latest_sub_position_unit * mode->config.position_width_radians;
    idle_velocity_ewma = 0.0f;
    idle_start_ms = 0U;
    out_of_bounds = false;
    reset_click();
}

static float stable_position(void)
{
    const uint32_t now_us = micros();
    float sample_time_s = (now_us - last_sample_us) * 1e-6f;
    const float raw_position_rad = mechanical_rad;
    const float raw_delta_rad = raw_position_rad - last_raw_position_rad;

    if (sample_time_s <= 0.0f || sample_time_s > 0.020f) {
        sample_time_s = 0.001f;
    }
    const float max_delta_rad = fmaxf(SMART_KNOB_MIN_SAMPLE_STEP_RAD,
                                      SMART_KNOB_MAX_SAMPLE_SPEED_RAD_S * sample_time_s);
    last_sample_us = now_us;
    /* Always advance the raw baseline. Otherwise one rejected high-speed or
       encoder-glitch sample makes every later sample compare against a stale
       position and permanently disables haptics. */
    last_raw_position_rad = raw_position_rad;
    if (fabsf(raw_delta_rad) <= max_delta_rad) {
        if (sample_recovery_pending) {
            if (++sample_recovery_count >= SMART_KNOB_SAMPLE_RECOVERY_COUNT) {
                /* Drop filter history accumulated before the discontinuity,
                   but preserve the detent/return center itself. */
                filtered_position_rad = raw_position_rad;
                sample_recovery_pending = false;
                sample_recovery_count = 0U;
                sample_valid = true;
            } else {
                sample_valid = false;
            }
        } else {
            filtered_position_rad += position_filter_alpha *
                                     (raw_position_rad - filtered_position_rad);
            sample_valid = true;
        }
    } else {
        sample_recovery_pending = true;
        sample_recovery_count = 0U;
        sample_valid = false;
    }
    return filtered_position_rad;
}

static void update_high_speed_state(float velocity_rad_s)
{
    const float speed_rad_s = fabsf(velocity_rad_s);
    if (high_speed_active) {
        if (speed_rad_s <= SMART_KNOB_HAPTIC_RESUME_SPEED_RAD_S) {
            high_speed_active = false;
        }
    } else if (speed_rad_s >= SMART_KNOB_MAX_HAPTIC_SPEED_RAD_S) {
        high_speed_active = true;
    }
}

static float derate_accelerating_current(float requested_current_a,
                                         float velocity_rad_s)
{
    const float speed_rad_s = fabsf(velocity_rad_s);
    if (requested_current_a * velocity_rad_s <= 0.0f ||
        speed_rad_s <= SMART_KNOB_SPEED_DERATE_START_RAD_S) {
        return requested_current_a;
    }

    const float scale = clampf(
        (SMART_KNOB_SPEED_DERATE_STOP_RAD_S - speed_rad_s) /
            (SMART_KNOB_SPEED_DERATE_STOP_RAD_S -
             SMART_KNOB_SPEED_DERATE_START_RAD_S),
        0.0f,
        1.0f);
    return requested_current_a * scale;
}

static bool has_detent_at(const PB_SmartKnobConfig *config, int32_t position)
{
    if (config->detent_positions_count == 0U) {
        return true;
    }
    for (pb_size_t index = 0U; index < config->detent_positions_count; ++index) {
        if (config->detent_positions[index] == position) {
            return true;
        }
    }
    return false;
}

static float friction_current(float velocity_rad_s, float compensation_a)
{
    if (fabsf(velocity_rad_s) <= SMART_KNOB_IDLE_VELOCITY_RAD_S) {
        return 0.0f;
    }
    const float taper = atanf(fabsf(velocity_rad_s) /
                              (SMART_KNOB_IDLE_VELOCITY_RAD_S * 10.0f)) /
                        (PI * 0.5f);
    return compensation_a * copysignf(taper, velocity_rad_s);
}

static float click_current(const SmartKnobTuning *tuning, bool click_allowed, uint32_t now_ms)
{
    if (current_position != previous_click_position) {
        previous_click_position = current_position;
        if (click_allowed && tuning->click_current_a > 0.0f) {
            click_direction = -click_direction;
            click_started_ms = now_ms;
            click_running = true;
        }
    }
    if (!click_allowed || !click_running) {
        return 0.0f;
    }

    const uint32_t elapsed_ms = now_ms - click_started_ms;
    if (elapsed_ms >= SMART_KNOB_CLICK_TOTAL_MS) {
        click_running = false;
        return 0.0f;
    }
    return (elapsed_ms < SMART_KNOB_CLICK_PHASE_MS ? click_direction : -click_direction) *
           tuning->click_current_a;
}

static void publish_runtime_state(float velocity_rad_s)
{
    uint8_t flags = 0U;
    if (motor_mode == MODE_DIAL) {
        flags |= (1U << 0);
    }
    if (MotorDriverIsOutputEnabled()) {
        flags |= (1U << 1);
    }
    if (out_of_bounds) {
        flags |= (1U << 2);
    }
    if (sample_valid) {
        flags |= (1U << 3);
    }
    if (telemetry_enable != 0U) {
        flags |= (1U << 4);
    }
    if (high_speed_active) {
        flags |= (1U << 5);
    }
    if (error_code != 0U) {
        flags |= (1U << 6);
    }
    if (over_vol_flag != 0U) {
        flags |= (1U << 7);
    }

    ++runtime_state_version;
    __DMB();
    runtime_state.active_mode = active_mode;
    runtime_state.flags = flags;
    runtime_state.current_position = current_position;
    runtime_state.sub_position_unit = latest_sub_position_unit;
    runtime_state.shaft_angle_rad = filtered_position_rad;
    runtime_state.shaft_velocity_rad_s = velocity_rad_s;
    runtime_state.commanded_current_ma = last_command_current_ma;
    runtime_state.measured_current_ma = ph_crrent_lpf;
    __DMB();
    ++runtime_state_version;
}

void init_smart_knob(void)
{
    smart_knob_modes_initialize();
    if (active_mode >= smart_knob_modes_count()) {
        active_mode = (uint8_t)SMART_KNOB_DEFAULT_MODE;
    }

    filtered_position_rad = mechanical_rad;
    last_raw_position_rad = mechanical_rad;
    last_sample_us = micros();
    enable_after_ms = HAL_GetTick() + SMART_KNOB_SETTLE_TIME_MS;
    sample_valid = true;
    sample_recovery_pending = false;
    sample_recovery_count = 0U;
    high_speed_active =
        fabsf(motor_rps) > SMART_KNOB_HAPTIC_RESUME_SPEED_RAD_S;
    last_command_current_ma = 0.0f;

    reanchor_active_mode(!controller_initialized);
    controller_initialized = true;
    publish_runtime_state(0.0f);
}

void smart_knob_set_update_rate(float update_rate_hz)
{
    if (update_rate_hz <= 0.0f) {
        return;
    }
    const float period_ratio = SMART_KNOB_LEGACY_RATE_HZ / update_rate_hz;
    position_filter_alpha = 1.0f -
        powf(1.0f - SMART_KNOB_POSITION_FILTER_ALPHA_BASE, period_ratio);
    idle_velocity_alpha = 1.0f -
        powf(1.0f - SMART_KNOB_IDLE_VELOCITY_ALPHA_BASE, period_ratio);
    idle_correction_alpha = 1.0f -
        powf(1.0f - SMART_KNOB_IDLE_CORRECTION_ALPHA_BASE, period_ratio);
}

void handle_smart_knob(void)
{
    SmartKnobModeConfig *mode = active_mode_mutable();
    if (mode == NULL) {
        MotorDriverSetCurrentReal(0.0f);
        return;
    }
    sanitize_mode(mode);

    const uint32_t now_ms = HAL_GetTick();
    const float knob_position_rad = stable_position();
    const float velocity_rad_s = motor_rps;
    update_high_speed_state(velocity_rad_s);
    if (!sample_valid || (int32_t)(now_ms - enable_after_ms) < 0) {
        last_command_current_ma = 0.0f;
        MotorDriverSetCurrentReal(0.0f);
        publish_runtime_state(velocity_rad_s);
        return;
    }

    PB_SmartKnobConfig *config = &mode->config;
    SmartKnobTuning *tuning = &mode->tuning;
    const int32_t num_positions = position_count(config);

    if (num_positions != 1) {
        idle_velocity_ewma = fabsf(velocity_rad_s) * idle_velocity_alpha +
                             idle_velocity_ewma * (1.0f - idle_velocity_alpha);
        if (idle_velocity_ewma > SMART_KNOB_IDLE_VELOCITY_RAD_S) {
            idle_start_ms = 0U;
        } else if (idle_start_ms == 0U) {
            idle_start_ms = now_ms;
        }
        if (idle_start_ms != 0U && (now_ms - idle_start_ms) > SMART_KNOB_IDLE_DELAY_MS &&
            fabsf(knob_position_rad - current_detent_center) < SMART_KNOB_IDLE_MAX_ANGLE_RAD) {
            current_detent_center = knob_position_rad * idle_correction_alpha +
                                    current_detent_center * (1.0f - idle_correction_alpha);
        }
    }

    float angle_to_center = knob_position_rad - current_detent_center;
    const float width = config->position_width_radians;
    const float snap_radians = width * config->snap_point;
    const float bias_radians = width * config->snap_point_bias;

    /* At 1 kHz one transition is typical; the bounded loop also recovers
       cleanly if a fast turn spans several very fine detents in one sample. */
    for (uint8_t transition = 0U; transition < 8U; ++transition) {
        const float snap_decrease = snap_radians +
            (current_position <= 0 ? bias_radians : -bias_radians);
        const float snap_increase = -snap_radians +
            (current_position >= 0 ? -bias_radians : bias_radians);
        if (angle_to_center > snap_decrease &&
            (num_positions <= 0 || current_position > config->min_position)) {
            current_detent_center += width;
            angle_to_center -= width;
            --current_position;
        } else if (angle_to_center < snap_increase &&
                   (num_positions <= 0 || current_position < config->max_position)) {
            current_detent_center -= width;
            angle_to_center += width;
            ++current_position;
        } else {
            break;
        }
    }

    latest_sub_position_unit = -angle_to_center / width;
    const float dead_zone_adjustment = clampf(
        angle_to_center,
        fmaxf(-width * SMART_KNOB_DEAD_ZONE_DETENT_PERCENT,
              -SMART_KNOB_DEAD_ZONE_RAD),
        fminf(width * SMART_KNOB_DEAD_ZONE_DETENT_PERCENT,
              SMART_KNOB_DEAD_ZONE_RAD));
    out_of_bounds = num_positions > 0 &&
        ((angle_to_center > 0.0f && current_position == config->min_position) ||
         (angle_to_center < 0.0f && current_position == config->max_position));

    float input = -angle_to_center + dead_zone_adjustment;
    if (!out_of_bounds && !has_detent_at(config, current_position)) {
        input = 0.0f;
    }
    const float p_gain = out_of_bounds ? config->endstop_strength_unit * 4.0f
                                       : tuning->p_gain;
    const float pid = clampf(p_gain * input - tuning->d_gain * velocity_rad_s,
                             -SMART_KNOB_PID_LIMIT,
                             SMART_KNOB_PID_LIMIT);
    float requested_current_a = tuning->current_scale_a * pid;
    requested_current_a += friction_current(velocity_rad_s,
                                            tuning->friction_current_a);
    requested_current_a += click_current(tuning,
                                         !out_of_bounds &&
                                             config->detent_positions_count == 0U,
                                         now_ms);

    const float safety_limit_a = SMART_KNOB_HARD_CURRENT_LIMIT_A *
                                 tuning->max_current_permille / 1000.0f;
    const float current_limit_a = fminf(tuning->current_limit_a,
                                        safety_limit_a);
    if (high_speed_active) {
        requested_current_a = 0.0f;
    } else {
        requested_current_a = clampf(requested_current_a,
                                     -current_limit_a,
                                     current_limit_a);
        if (num_positions == 1) {
            requested_current_a = derate_accelerating_current(requested_current_a,
                                                              velocity_rad_s);
        }
    }
    if (fabsf(requested_current_a) <= SMART_KNOB_CURRENT_DEADBAND_A) {
        requested_current_a = 0.0f;
    }

    last_command_current_ma = requested_current_a * 1000.0f;
    MotorDriverSetCurrentReal(last_command_current_ma);
    publish_runtime_state(velocity_rad_s);
}

bool smart_knob_select_mode(uint8_t mode_index)
{
    if (smart_knob_mode_get(mode_index) == NULL) {
        return false;
    }
    active_mode = mode_index;
    if (!controller_initialized) {
        init_smart_knob();
    } else {
        reanchor_active_mode(true);
        publish_runtime_state(motor_rps);
    }
    return true;
}

uint8_t smart_knob_active_mode(void)
{
    return active_mode;
}

const SmartKnobModeConfig *smart_knob_active_config(void)
{
    return smart_knob_mode_get(active_mode);
}

static int32_t scaled_to_i32(float value)
{
    if (!isfinite(value)) {
        return 0;
    }
    const double scaled = (double)value * SMART_KNOB_SCALE;
    if (scaled > INT32_MAX) {
        return INT32_MAX;
    }
    if (scaled < INT32_MIN) {
        return INT32_MIN;
    }
    return (int32_t)lround(scaled);
}

bool smart_knob_read_parameter(uint16_t index, int32_t *value)
{
    const SmartKnobModeConfig *mode = smart_knob_active_config();
    const SmartKnobModeConfig *custom = smart_knob_mode_get(SMART_KNOB_MODE_CUSTOM);
    SmartKnobRuntimeState state;
    if (value == NULL || mode == NULL || custom == NULL) {
        return false;
    }

    switch (index) {
    case SMART_KNOB_FUNC_MODE: *value = active_mode; break;
    case SMART_KNOB_FUNC_TELEMETRY_ENABLE: *value = telemetry_enable; break;
    case SMART_KNOB_FUNC_TELEMETRY_RATE_HZ: *value = telemetry_rate_hz; break;
    case SMART_KNOB_FUNC_TELEMETRY_HOST_ID: *value = telemetry_host_id; break;
    case SMART_KNOB_FUNC_MODE_COUNT: *value = smart_knob_modes_count(); break;
    case SMART_KNOB_FUNC_PROTOCOL_VERSION: *value = SMART_KNOB_TELEMETRY_PROTOCOL_VERSION; break;
    case SMART_KNOB_FUNC_P_GAIN: *value = scaled_to_i32(mode->tuning.p_gain); break;
    case SMART_KNOB_FUNC_D_GAIN: *value = scaled_to_i32(mode->tuning.d_gain); break;
    case SMART_KNOB_FUNC_CURRENT_SCALE: *value = scaled_to_i32(mode->tuning.current_scale_a); break;
    case SMART_KNOB_FUNC_CURRENT_LIMIT: *value = scaled_to_i32(mode->tuning.current_limit_a); break;
    case SMART_KNOB_FUNC_MAX_CURRENT_PERMILLE: *value = mode->tuning.max_current_permille; break;
    case SMART_KNOB_FUNC_FRICTION_CURRENT: *value = scaled_to_i32(mode->tuning.friction_current_a); break;
    case SMART_KNOB_FUNC_CLICK_CURRENT: *value = scaled_to_i32(mode->tuning.click_current_a); break;
    case SMART_KNOB_FUNC_CUSTOM_POSITION: *value = custom->config.position; break;
    case SMART_KNOB_FUNC_CUSTOM_MIN_POSITION: *value = custom->config.min_position; break;
    case SMART_KNOB_FUNC_CUSTOM_MAX_POSITION: *value = custom->config.max_position; break;
    case SMART_KNOB_FUNC_CUSTOM_WIDTH_DEG:
        *value = scaled_to_i32(RAD_TO_DEG(custom->config.position_width_radians)); break;
    case SMART_KNOB_FUNC_CUSTOM_DETENT_STRENGTH:
        *value = scaled_to_i32(custom->config.detent_strength_unit); break;
    case SMART_KNOB_FUNC_CUSTOM_ENDSTOP_STRENGTH:
        *value = scaled_to_i32(custom->config.endstop_strength_unit); break;
    case SMART_KNOB_FUNC_CUSTOM_SNAP_POINT: *value = scaled_to_i32(custom->config.snap_point); break;
    case SMART_KNOB_FUNC_CUSTOM_SNAP_BIAS: *value = scaled_to_i32(custom->config.snap_point_bias); break;
    case SMART_KNOB_FUNC_CUSTOM_CLICK_CURRENT: *value = scaled_to_i32(custom->tuning.click_current_a); break;
    case SMART_KNOB_FUNC_CUSTOM_FRICTION_CURRENT: *value = scaled_to_i32(custom->tuning.friction_current_a); break;
    case SMART_KNOB_FUNC_CUSTOM_CURRENT_SCALE: *value = scaled_to_i32(custom->tuning.current_scale_a); break;
    case SMART_KNOB_FUNC_CUSTOM_P_GAIN: *value = scaled_to_i32(custom->tuning.p_gain); break;
    case SMART_KNOB_FUNC_CUSTOM_D_GAIN: *value = scaled_to_i32(custom->tuning.d_gain); break;
    case SMART_KNOB_FUNC_CUSTOM_LED_HUE: *value = custom->config.led_hue; break;
    case SMART_KNOB_FUNC_STATE_POSITION:
        *value = current_position; break;
    case SMART_KNOB_FUNC_STATE_SUB_POSITION:
        *value = (int32_t)lroundf(latest_sub_position_unit * 1000000.0f); break;
    case SMART_KNOB_FUNC_STATE_COMMAND_CURRENT:
        if (!smart_knob_get_runtime_state(&state)) return false;
        *value = (int32_t)lroundf(state.commanded_current_ma * 100.0f); break;
    case SMART_KNOB_FUNC_STATE_MEASURED_CURRENT:
        if (!smart_knob_get_runtime_state(&state)) return false;
        *value = (int32_t)lroundf(state.measured_current_ma * 100.0f); break;
    default:
        return false;
    }
    return true;
}

static void update_custom_runtime_after_write(uint16_t index,
                                              float previous_width,
                                              int32_t previous_position)
{
    if (active_mode != SMART_KNOB_MODE_CUSTOM || !controller_initialized) {
        return;
    }
    SmartKnobModeConfig *custom = smart_knob_mode_get_mutable(SMART_KNOB_MODE_CUSTOM);
    if (index == SMART_KNOB_FUNC_CUSTOM_POSITION &&
        custom->config.position != previous_position) {
        current_position = custom->config.position;
        current_detent_center = filtered_position_rad +
            custom->config.sub_position_unit * custom->config.position_width_radians;
        reset_click();
    } else if (index == SMART_KNOB_FUNC_CUSTOM_WIDTH_DEG &&
               custom->config.position_width_radians != previous_width) {
        current_detent_center = filtered_position_rad +
            latest_sub_position_unit * custom->config.position_width_radians;
    }
    if (custom->config.min_position <= custom->config.max_position) {
        current_position = clamp_i32(current_position,
                                     custom->config.min_position,
                                     custom->config.max_position);
    }
}

bool smart_knob_write_parameter(uint16_t index, int32_t value, uint8_t host_id)
{
    SmartKnobModeConfig *mode = active_mode_mutable();
    SmartKnobModeConfig *custom = smart_knob_mode_get_mutable(SMART_KNOB_MODE_CUSTOM);
    if (mode == NULL || custom == NULL) {
        return false;
    }
    const float scaled = value / SMART_KNOB_SCALE;
    const float previous_width = custom->config.position_width_radians;
    const int32_t previous_position = custom->config.position;

    switch (index) {
    case SMART_KNOB_FUNC_MODE:
        return value >= 0 && value <= UINT8_MAX && smart_knob_select_mode((uint8_t)value);
    case SMART_KNOB_FUNC_TELEMETRY_ENABLE:
        telemetry_host_id = host_id;
        telemetry_enable = value != 0;
        telemetry_pending_type = 0U;
        telemetry_started = false;
        return true;
    case SMART_KNOB_FUNC_TELEMETRY_RATE_HZ:
        telemetry_host_id = host_id;
        telemetry_rate_hz = (uint8_t)clamp_i32(value, 1, SMART_KNOB_TELEMETRY_MAX_RATE_HZ);
        telemetry_last_ms = 0U;
        telemetry_started = false;
        return true;
    case SMART_KNOB_FUNC_TELEMETRY_HOST_ID:
        telemetry_host_id = (uint8_t)clamp_i32(value, 0, UINT8_MAX);
        return true;
    case SMART_KNOB_FUNC_P_GAIN: mode->tuning.p_gain = scaled; break;
    case SMART_KNOB_FUNC_D_GAIN: mode->tuning.d_gain = scaled; break;
    case SMART_KNOB_FUNC_CURRENT_SCALE: mode->tuning.current_scale_a = scaled; break;
    case SMART_KNOB_FUNC_CURRENT_LIMIT: mode->tuning.current_limit_a = scaled; break;
    case SMART_KNOB_FUNC_MAX_CURRENT_PERMILLE:
        mode->tuning.max_current_permille = (uint16_t)clamp_i32(value, 0, 1000); break;
    case SMART_KNOB_FUNC_FRICTION_CURRENT: mode->tuning.friction_current_a = scaled; break;
    case SMART_KNOB_FUNC_CLICK_CURRENT: mode->tuning.click_current_a = scaled; break;
    case SMART_KNOB_FUNC_CUSTOM_POSITION: custom->config.position = value; break;
    case SMART_KNOB_FUNC_CUSTOM_MIN_POSITION: custom->config.min_position = value; break;
    case SMART_KNOB_FUNC_CUSTOM_MAX_POSITION: custom->config.max_position = value; break;
    case SMART_KNOB_FUNC_CUSTOM_WIDTH_DEG: custom->config.position_width_radians = DEG_TO_RAD(scaled); break;
    case SMART_KNOB_FUNC_CUSTOM_DETENT_STRENGTH:
        custom->config.detent_strength_unit = scaled;
        custom->tuning.p_gain = fmaxf(0.0f, scaled) * 4.0f;
        break;
    case SMART_KNOB_FUNC_CUSTOM_ENDSTOP_STRENGTH: custom->config.endstop_strength_unit = scaled; break;
    case SMART_KNOB_FUNC_CUSTOM_SNAP_POINT: custom->config.snap_point = scaled; break;
    case SMART_KNOB_FUNC_CUSTOM_SNAP_BIAS: custom->config.snap_point_bias = scaled; break;
    case SMART_KNOB_FUNC_CUSTOM_CLICK_CURRENT: custom->tuning.click_current_a = scaled; break;
    case SMART_KNOB_FUNC_CUSTOM_FRICTION_CURRENT: custom->tuning.friction_current_a = scaled; break;
    case SMART_KNOB_FUNC_CUSTOM_CURRENT_SCALE: custom->tuning.current_scale_a = scaled; break;
    case SMART_KNOB_FUNC_CUSTOM_P_GAIN: custom->tuning.p_gain = scaled; break;
    case SMART_KNOB_FUNC_CUSTOM_D_GAIN: custom->tuning.d_gain = scaled; break;
    case SMART_KNOB_FUNC_CUSTOM_LED_HUE: custom->config.led_hue = clamp_i32(value, 0, 255); break;
    default:
        return false;
    }

    sanitize_mode(mode);
    sanitize_mode(custom);
    update_custom_runtime_after_write(index, previous_width, previous_position);
    publish_runtime_state(motor_rps);
    return true;
}

bool smart_knob_get_runtime_state(SmartKnobRuntimeState *state)
{
    if (state == NULL) {
        return false;
    }
    for (uint8_t attempt = 0U; attempt < 4U; ++attempt) {
        const uint32_t before = runtime_state_version;
        if ((before & 1U) != 0U) {
            continue;
        }
        __DMB();
        *state = runtime_state;
        __DMB();
        if (before == runtime_state_version) {
            return true;
        }
    }
    return false;
}

bool smart_knob_telemetry_enabled(void)
{
    return telemetry_enable != 0U;
}

bool smart_knob_build_telemetry(uint32_t now_ms, SmartKnobTelemetryFrame *frame)
{
    if (frame == NULL || telemetry_enable == 0U) {
        telemetry_pending_type = 0U;
        return false;
    }

    if (telemetry_pending_type == 0U) {
        const uint32_t period_ms = 1000U / telemetry_rate_hz;
        if (telemetry_started && (now_ms - telemetry_last_ms) < period_ms) {
            return false;
        }
        if (!smart_knob_get_runtime_state(&telemetry_latched_state)) {
            return false;
        }
        telemetry_last_ms = now_ms;
        telemetry_started = true;
        ++telemetry_sequence;
        telemetry_pending_type = SMART_KNOB_TELEMETRY_STATE_TYPE;
    }

    memset(frame, 0, sizeof(*frame));
    frame->sequence = telemetry_sequence;
    frame->type = telemetry_pending_type;
    frame->destination_id = telemetry_host_id;

    if (telemetry_pending_type == SMART_KNOB_TELEMETRY_STATE_TYPE) {
        const int16_t sub_position = saturate_i16(
            telemetry_latched_state.sub_position_unit * 10000.0f);
        frame->data[0] = telemetry_latched_state.active_mode;
        frame->data[1] = telemetry_latched_state.flags;
        memcpy(&frame->data[2], &telemetry_latched_state.current_position, 4U);
        memcpy(&frame->data[6], &sub_position, 2U);
        telemetry_pending_type = SMART_KNOB_TELEMETRY_MOTION_TYPE;
    } else {
        const int32_t angle_cdeg = (int32_t)lroundf(
            RAD_TO_DEG(telemetry_latched_state.shaft_angle_rad) * 100.0f);
        const int16_t command_ma = saturate_i16(
            telemetry_latched_state.commanded_current_ma);
        const int16_t measured_ma = saturate_i16(
            telemetry_latched_state.measured_current_ma);
        memcpy(&frame->data[0], &angle_cdeg, 4U);
        memcpy(&frame->data[4], &command_ma, 2U);
        memcpy(&frame->data[6], &measured_ma, 2U);
        telemetry_pending_type = 0U;
    }
    return true;
}
