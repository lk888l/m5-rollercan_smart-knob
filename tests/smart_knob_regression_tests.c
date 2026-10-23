#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "mysys.h"
#include "smart_knob.h"
#include "smart_knob_modes.h"

uint8_t motor_mode = MODE_DIAL;
uint8_t error_code;
uint8_t over_vol_flag;
float mechanical_rad;
float motor_rps;
float motor_rps_fast;
float ph_crrent_lpf;

static uint32_t fake_time_us;
static float commanded_current_ma;
static uint8_t output_enabled = 1U;
static bool zero_current_tracking;
static uint32_t actuator_write_count;

uint32_t HAL_GetTick(void)
{
    return fake_time_us / 1000U;
}

uint32_t micros(void)
{
    return fake_time_us;
}

void MotorDriverSetCurrentReal(float phase_current_ma)
{
    ++actuator_write_count;
    commanded_current_ma = fabsf(phase_current_ma) <= 60.0f
                               ? 0.0f
                               : phase_current_ma;
}

void MotorDriverSetCurrentRealContinuous(float phase_current_ma)
{
    ++actuator_write_count;
    commanded_current_ma = phase_current_ma;
    zero_current_tracking = false;
}

void MotorDriverSetTorqueCurrent(float phase_current_ma)
{
    ++actuator_write_count;
    commanded_current_ma = phase_current_ma;
    zero_current_tracking = true;
}

uint8_t MotorDriverIsOutputEnabled(void)
{
    return output_enabled;
}

static void failf(const char *test_name,
                  const char *message,
                  float actual,
                  float expected)
{
    fprintf(stderr,
            "FAIL %s: %s (actual=%.6f expected=%.6f)\n",
            test_name,
            message,
            (double)actual,
            (double)expected);
    exit(EXIT_FAILURE);
}

static void require_true(const char *test_name, bool condition, const char *message)
{
    if (!condition) {
        fprintf(stderr, "FAIL %s: %s\n", test_name, message);
        exit(EXIT_FAILURE);
    }
}

static void advance_and_handle(float position_rad,
                               float velocity_rad_s,
                               uint32_t elapsed_us)
{
    mechanical_rad = position_rad;
    motor_rps = velocity_rad_s;
    motor_rps_fast = velocity_rad_s;
    fake_time_us += elapsed_us;
    handle_smart_knob();
}

static void reset_controller(uint8_t mode_index)
{
    SmartKnobModeConfig *mode = smart_knob_mode_get_mutable(mode_index);
    require_true("reset_controller", mode != NULL, "requested mode must exist");

    /* Exercise a normal detent, not either bounded endstop. The coarse preset
       boots at its minimum position, where endstop_strength replaces p_gain
       and would hide the reported quarter-detent discontinuity. */
    if (mode_index == SMART_KNOB_MODE_COARSE_STRONG) {
        mode->config.position = 15;
    }

    fake_time_us = 0U;
    commanded_current_ma = NAN;
    mechanical_rad = 0.0f;
    motor_rps = 0.0f;
    motor_rps_fast = 0.0f;
    ph_crrent_lpf = 0.0f;
    error_code = 0U;
    over_vol_flag = 0U;
    motor_mode = MODE_DIAL;
    output_enabled = 1U;

    require_true("reset_controller",
                 smart_knob_select_mode(mode_index),
                 "requested mode must exist");
    init_smart_knob();
    smart_knob_set_update_rate(1000.0f);

    /* Pass the controller's 300 ms startup inhibit while staying exactly at
       the newly anchored center. */
    advance_and_handle(0.0f, 0.0f, 301000U);
    advance_and_handle(0.0f, 0.0f, 1000U);
}

static SmartKnobRuntimeState runtime_state(const char *test_name)
{
    SmartKnobRuntimeState state;
    require_true(test_name,
                 smart_knob_get_runtime_state(&state),
                 "runtime state snapshot must be readable");
    return state;
}

static void test_coarse_strong_current_is_continuous_near_quarter_detent(void)
{
    static const char test_name[] =
        "coarse_strong_current_is_continuous_near_quarter_detent";
    reset_controller(SMART_KNOB_MODE_COARSE_STRONG);

    const SmartKnobModeConfig *mode = smart_knob_active_config();
    require_true(test_name, mode != NULL, "active mode must be available");
    const float width = mode->config.position_width_radians;

    float previous_current_ma = commanded_current_ma;
    float maximum_step_ma = 0.0f;
    float step_from_zero_ma = 0.0f;
    float step_from_zero_fraction = 0.0f;
    float quarter_detent_current_ma = 0.0f;
    bool saw_exact_zero = false;

    /* Move slowly through the reported 0.23-detent region. With a continuous
       control law, adjacent 0.001-width samples differ by only a few mA. A
       hard output deadband instead exposes a roughly 0 -> 90 mA edge. */
    for (uint32_t sample = 80U; sample <= 300U; ++sample) {
        const float fraction = (float)sample / 1000.0f;
        advance_and_handle(width * fraction, 0.0f, 1000U);

        const float step_ma = fabsf(commanded_current_ma - previous_current_ma);
        saw_exact_zero = saw_exact_zero || commanded_current_ma == 0.0f;
        if (step_ma > maximum_step_ma) {
            maximum_step_ma = step_ma;
        }
        if (fabsf(previous_current_ma) <= 1.0f &&
            fabsf(commanded_current_ma) >= fabsf(step_from_zero_ma)) {
            step_from_zero_ma = commanded_current_ma;
            step_from_zero_fraction = fraction;
        }
        if (sample == 250U) {
            quarter_detent_current_ma = commanded_current_ma;
        }
        previous_current_ma = commanded_current_ma;
    }

    require_true(test_name,
                 saw_exact_zero,
                 "sweep must include the zero-current side of the transition");
    require_true(test_name,
                 fabsf(commanded_current_ma) > 100.0f,
                 "sweep must reach a nonzero restoring-current region");
    if (fabsf(step_from_zero_ma) >= 50.0f) {
        fprintf(stderr,
                "FAIL %s: zero-current edge jumped to %.3f mA at %.3f detent widths\n",
                test_name,
                (double)step_from_zero_ma,
                (double)step_from_zero_fraction);
        exit(EXIT_FAILURE);
    }
    if (maximum_step_ma > 20.0f) {
        failf(test_name,
              "adjacent commanded-current samples must remain continuous (mA)",
              maximum_step_ma,
              20.0f);
    }
    require_true(test_name,
                 fabsf(quarter_detent_current_ma) > 130.0f &&
                     fabsf(quarter_detent_current_ma) < 170.0f,
                 "quarter-detent restoring current must be continuous and non-zero");
}

static void test_bounded_coarse_idle_does_not_move_detent_center(void)
{
    static const char test_name[] =
        "bounded_coarse_idle_does_not_move_detent_center";
    reset_controller(SMART_KNOB_MODE_COARSE_STRONG);

    const SmartKnobModeConfig *mode = smart_knob_active_config();
    require_true(test_name, mode != NULL, "active mode must be available");
    const float held_position_rad =
        mode->config.position_width_radians * 0.30f;

    /* Let the position filter settle without reaching the 500 ms idle-delay
       threshold, then hold the shaft still well beyond that threshold. */
    for (uint32_t sample = 0U; sample < 40U; ++sample) {
        advance_and_handle(held_position_rad, 0.0f, 1000U);
    }
    const SmartKnobRuntimeState before = runtime_state(test_name);

    if (fabsf(before.commanded_current_ma) <= 100.0f) {
        failf(test_name,
              "precondition: the held offset must have restoring current (mA)",
              before.commanded_current_ma,
              100.0f);
    }
    require_true(test_name,
                 fabsf(before.sub_position_unit) > 0.25f,
                 "precondition: the held offset must remain near 0.30 detent widths");

    for (uint32_t sample = 0U; sample < 900U; ++sample) {
        advance_and_handle(held_position_rad, 0.0f, 1000U);
    }
    const SmartKnobRuntimeState after = runtime_state(test_name);

    if (fabsf(after.sub_position_unit - before.sub_position_unit) > 0.02f) {
        failf(test_name,
              "bounded-mode detent center drifted toward the held shaft position",
              after.sub_position_unit,
              before.sub_position_unit);
    }
    require_true(test_name,
                 fabsf(after.commanded_current_ma) >=
                     0.80f * fabsf(before.commanded_current_ma),
                 "restoring current must not decay toward zero while held still");
}

static void test_exact_center_still_commands_exact_zero(void)
{
    static const char test_name[] =
        "exact_center_still_commands_exact_zero";
    reset_controller(SMART_KNOB_MODE_COARSE_STRONG);

    for (uint32_t sample = 0U; sample < 600U; ++sample) {
        /* Include stationary encoder quantization noise. This must not turn
           the current loop back on through the D term. */
        const float tiny_velocity_rad_s =
            (sample & 1U) != 0U ? 0.002f : -0.002f;
        advance_and_handle(0.0f, tiny_velocity_rad_s, 1000U);
        if (commanded_current_ma != 0.0f) {
            failf(test_name,
                  "exact detent center must command exactly zero mA",
                  commanded_current_ma,
                  0.0f);
        }
    }

    const SmartKnobRuntimeState state = runtime_state(test_name);
    if (state.commanded_current_ma != 0.0f) {
        failf(test_name,
              "telemetry must report exact zero at detent center",
              state.commanded_current_ma,
              0.0f);
    }
}

static void test_coarse_damping_uses_recent_motion(void)
{
    const char *name = "coarse_damping_uses_recent_motion";
    smart_knob_modes_reset_defaults();
    reset_controller(SMART_KNOB_MODE_COARSE_STRONG);

    /* After release/rebound the 2 Hz legacy estimate can retain the opposite
       sign for tens of milliseconds. D must oppose the recent encoder motion. */
    motor_rps = 10.0f;
    motor_rps_fast = -2.0f;
    fake_time_us += 1000U;
    handle_smart_knob();
    require_true(name, commanded_current_ma > 0.0f,
        "coarse damping must brake a reversal even while the legacy speed remains positive");
    const SmartKnobRuntimeState state = runtime_state(name);
    require_true(name, fabsf(state.shaft_velocity_rad_s + 2.0f) < 0.001f,
        "telemetry must expose the velocity actually used by the haptic controller");

    motor_rps_fast = 0.0f;
    fake_time_us += 1000U;
    handle_smart_knob();
    require_true(name, commanded_current_ma == 0.0f,
        "stopping at the center must not leave a torque from the old speed estimate");

    motor_rps = 0.0f;
    motor_rps_fast = 185.0f;
    fake_time_us += 1000U;
    handle_smart_knob();
    require_true(name, commanded_current_ma == 0.0f &&
        (runtime_state(name).flags & (1U << 5)) != 0U,
        "fast flick protection must react before the legacy velocity catches up");
}

static void test_coarse_high_speed_retains_haptics(void)
{
    const char *name = "coarse_high_speed_retains_haptics";
    const uint8_t modes[] = {SMART_KNOB_MODE_COARSE_STRONG, SMART_KNOB_MODE_COARSE_WEAK};
    const float speeds[] = {70.0f, 110.0f, 130.0f, 170.0f};
    for (unsigned mode = 0U; mode < 2U; ++mode) {
        for (unsigned direction = 0U; direction < 2U; ++direction) {
            for (unsigned speed = 0U; speed < sizeof(speeds) / sizeof(speeds[0]); ++speed) {
                smart_knob_modes_reset_defaults();
                reset_controller(modes[mode]);
                smart_knob_mode_get_mutable(modes[mode])->config.position = 15;
                require_true(name, smart_knob_select_mode(modes[mode]), "coarse mode selected");
                const float velocity = (direction == 0U ? 1.0f : -1.0f) * speeds[speed];
                unsigned torque_samples = 0U;
                /* Integrate actual position so the encoder sample envelope is
                   exercised along with the filtered-speed protection. */
                for (unsigned sample = 0U; sample < 30U; ++sample) {
                    advance_and_handle(mechanical_rad + velocity * 0.001f, velocity, 1000U);
                    const SmartKnobRuntimeState state = runtime_state(name);
                    require_true(name, (state.flags & (1U << 3)) != 0U,
                        "fast continuous motion below the new limit must remain a valid sample");
                    require_true(name, (state.flags & (1U << 5)) == 0U,
                        "speeds from 70 through 170 rad/s must retain haptics in both directions");
                    require_true(name, isfinite(commanded_current_ma) &&
                        fabsf(commanded_current_ma) <= 450.01f,
                        "extended-speed haptics must retain the preset current ceiling");
                    if (fabsf(commanded_current_ma) > 1.0f) ++torque_samples;
                }
                require_true(name, torque_samples >= 20U,
                    "valid fast motion must produce haptic output, not just clear the speed flag");
            }
        }
    }
}

static void test_coarse_high_speed_hysteresis_and_sample_recovery(void)
{
    const char *name = "coarse_high_speed_hysteresis_and_sample_recovery";
    for (unsigned direction = 0U; direction < 2U; ++direction) {
        const float sign = direction == 0U ? 1.0f : -1.0f;
        smart_knob_modes_reset_defaults();
        reset_controller(SMART_KNOB_MODE_COARSE_STRONG);
        const float speeds[] = {179.0f, 180.0f, 150.0f, 121.0f, 120.0f, 150.0f};
        const bool inhibited[] = {false, true, true, true, false, false};
        for (unsigned sample = 0U; sample < sizeof(speeds) / sizeof(speeds[0]); ++sample) {
            const float velocity = sign * speeds[sample];
            advance_and_handle(mechanical_rad + velocity * 0.001f, velocity, 1000U);
            const SmartKnobRuntimeState state = runtime_state(name);
            require_true(name, (state.flags & (1U << 3)) != 0U,
                "protection and resume speeds must fit the valid encoder sample envelope");
            require_true(name, ((state.flags & (1U << 5)) != 0U) == inhibited[sample],
                "speed protection must enter at 180 and resume at 120 rad/s without chatter");
            if (inhibited[sample]) require_true(name, commanded_current_ma == 0.0f,
                "the high-speed lockout must command zero current");
        }

        /* Keep the filtered speed low while injecting a raw discontinuity.
           Sample validity must reject it independently of speed lockout. */
        advance_and_handle(mechanical_rad + sign * 0.239f, sign * 50.0f, 1000U);
        require_true(name, (runtime_state(name).flags & (1U << 3)) != 0U,
            "a 239 rad/s raw step must fit the expanded sample envelope");
        advance_and_handle(mechanical_rad + sign * 0.250f, sign * 50.0f, 1000U);
        require_true(name, (runtime_state(name).flags & (1U << 3)) == 0U &&
            commanded_current_ma == 0.0f,
            "a raw step above 240 rad/s must still reject the sample and clear output");
        advance_and_handle(mechanical_rad + sign * 0.050f, sign * 50.0f, 1000U);
        require_true(name, (runtime_state(name).flags & (1U << 3)) == 0U &&
            commanded_current_ma == 0.0f,
            "the first valid sample after a discontinuity must remain inhibited");
        advance_and_handle(mechanical_rad + sign * 0.050f, sign * 50.0f, 1000U);
        require_true(name, (runtime_state(name).flags & (1U << 3)) != 0U &&
            (runtime_state(name).flags & (1U << 5)) == 0U &&
            fabsf(commanded_current_ma) > 1.0f,
            "two good samples must restore haptics without leaving a stale position baseline");
    }
}

static void test_fine_high_speed_tracks_detent_center(void)
{
    const char *name = "fine_high_speed_tracks_detent_center";
    const uint8_t modes[] = {SMART_KNOB_MODE_FINE_SMOOTH, SMART_KNOB_MODE_FINE_DETENTS};
    const float speeds[] = {130.0f, 170.0f};
    for (unsigned mode = 0U; mode < sizeof(modes) / sizeof(modes[0]); ++mode) {
        for (unsigned direction = 0U; direction < 2U; ++direction) {
            for (unsigned speed = 0U; speed < sizeof(speeds) / sizeof(speeds[0]); ++speed) {
                smart_knob_modes_reset_defaults();
                reset_controller(modes[mode]);
                const PB_SmartKnobConfig *config = &smart_knob_active_config()->config;
                const float velocity = (direction == 0U ? 1.0f : -1.0f) * speeds[speed];
                /* At 170 rad/s, a 1 degree preset crosses nearly ten detents
                   per millisecond. Stay away from the bounded endpoints so
                   leftover displacement cannot be mistaken for an endstop. */
                for (unsigned sample = 0U; sample < 12U; ++sample) {
                    advance_and_handle(mechanical_rad + velocity * 0.001f, velocity, 1000U);
                    const SmartKnobRuntimeState state = runtime_state(name);
                    require_true(name, (state.flags & (1U << 3)) != 0U &&
                        (state.flags & (1U << 5)) == 0U,
                        "fine presets must retain valid haptics below the extended speed limit");
                    require_true(name, current_position > config->min_position &&
                        current_position < config->max_position,
                        "fine high-speed tracking must be tested away from either endpoint");
                    if (fabsf(state.sub_position_unit) > config->snap_point + 0.001f) {
                        failf(name,
                            "fine detent center must catch up within each control iteration",
                            fabsf(state.sub_position_unit), config->snap_point);
                    }
                    require_true(name, isfinite(commanded_current_ma) &&
                        fabsf(commanded_current_ma) <=
                            smart_knob_active_config()->tuning.current_limit_a * 1000.0f + 0.01f,
                        "fine high-speed tracking must retain its current ceiling");
                }
            }
        }
    }
}

static bool simulate_coarse_flick(uint8_t mode_index, int32_t initial_position,
                                 float hand_speed, float coulomb_current_a)
{
    const char *name = "coarse_flick_release_settles";
    const float dt = 0.001f;
    const float encoder_step = 2.0f * PI / 16384.0f;
    const float display_angle_step = PI / 1800.0f;
    const float slow_alpha = dt / (dt + 1.0f / (2.0f * PI * 2.0f));
    const float fast_alpha = dt / (dt + 0.008f);

    smart_knob_modes_reset_defaults();
    reset_controller(mode_index);
    smart_knob_mode_get_mutable(mode_index)->config.position = initial_position;
    require_true(name, smart_knob_select_mode(mode_index), "coarse preset must exist");
    float angle = 0.0f;
    float velocity = 0.0f;
    float previous_encoder_angle = 0.0f;
    float slow_velocity = 0.0f;
    float fast_velocity = 0.0f;
    float late_min = INFINITY;
    float late_max = -INFINITY;
    float late_peak_speed = 0.0f;

    for (unsigned sample = 0U; sample < 1500U; ++sample) {
        if (sample < 30U) {
            velocity = hand_speed;
            angle += velocity * dt;
        } else {
            /* Inertia and viscous loss use docs/frictionless-mode.md.
               Coulomb loss also includes a lower-friction boundary case;
               this is a stability model, not a substitute for hardware.
               Substeps keep the plant integration stable at the 1 kHz
               controller rate. */
            for (unsigned substep = 0U; substep < 20U; ++substep) {
                const float loss_a = coulomb_current_a * tanhf(velocity / 0.2f) +
                                     0.000304f * velocity;
                const float acceleration =
                    (commanded_current_ma * 0.001f - loss_a) / 0.000135f;
                velocity += acceleration * dt / 20.0f;
                angle += velocity * dt / 20.0f;
            }
        }
        const float encoder_angle = roundf(angle / encoder_step) * encoder_step;
        const float raw_velocity = (encoder_angle - previous_encoder_angle) / dt;
        previous_encoder_angle = encoder_angle;
        slow_velocity += slow_alpha * (raw_velocity - slow_velocity);
        fast_velocity += fast_alpha * (raw_velocity - fast_velocity);
        mechanical_rad = roundf(angle / display_angle_step) * display_angle_step;
        motor_rps = slow_velocity;
        motor_rps_fast = fast_velocity;
        fake_time_us += 1000U;
        handle_smart_knob();
        require_true(name, isfinite(commanded_current_ma) &&
            fabsf(commanded_current_ma) <= 450.01f,
            "release must always respect the preset current ceiling");
        if (sample >= 1000U) {
            late_min = fminf(late_min, angle);
            late_max = fmaxf(late_max, angle);
            late_peak_speed = fmaxf(late_peak_speed, fabsf(velocity));
        }
    }
    printf("coarse release mode=%u start=%ld speed=%.0f friction_ma=%.0f late_span_deg=%.4f late_peak_rad_s=%.4f\n",
        (unsigned)mode_index, (long)initial_position, (double)hand_speed,
        (double)(1000.0f * coulomb_current_a),
        (double)((late_max - late_min) * 180.0f / PI),
        (double)late_peak_speed);
    require_true(name, current_position >= 0 && current_position <= 31,
        "flicking through an endpoint must retain the bounded preset position range");
    return late_max - late_min < PI / 180.0f && late_peak_speed < 0.5f;
}

static void test_coarse_flick_release_settles(void)
{
    const uint8_t modes[] = {SMART_KNOB_MODE_COARSE_STRONG, SMART_KNOB_MODE_COARSE_WEAK};
    const int32_t initial_positions[] = {15, 0, 31};
    const float friction_currents[] = {0.005f, 0.022f};
    bool all_settled = true;
    for (unsigned mode = 0U; mode < 2U; ++mode) {
        for (unsigned start = 0U; start < 3U; ++start) {
            for (unsigned friction = 0U; friction < 2U; ++friction) {
                for (unsigned direction = 0U; direction < 2U; ++direction) {
                    const bool settled = simulate_coarse_flick(modes[mode],
                        initial_positions[start], direction == 0U ? 30.0f : -30.0f,
                        friction_currents[friction]);
                    all_settled = all_settled && settled;
                }
            }
        }
    }
    require_true("coarse_flick_release_settles", all_settled,
        "both coarse presets must settle after release, including reversals and both endpoints");
}

static void test_coarse_fast_flick_release_settles(void)
{
    const uint8_t modes[] = {SMART_KNOB_MODE_COARSE_STRONG, SMART_KNOB_MODE_COARSE_WEAK};
    const int32_t initial_positions[] = {15, 0, 31};
    const float speeds[] = {70.0f, 110.0f, 130.0f, 170.0f};
    const float friction_currents[] = {0.005f, 0.022f};
    bool all_settled = true;
    for (unsigned mode = 0U; mode < 2U; ++mode) {
        for (unsigned start = 0U; start < 3U; ++start) {
            for (unsigned speed = 0U; speed < sizeof(speeds) / sizeof(speeds[0]); ++speed) {
                for (unsigned friction = 0U; friction < 2U; ++friction) {
                    for (unsigned direction = 0U; direction < 2U; ++direction) {
                        const bool settled = simulate_coarse_flick(modes[mode],
                            initial_positions[start], (direction == 0U ? 1.0f : -1.0f) * speeds[speed],
                            friction_currents[friction]);
                        all_settled = all_settled && settled;
                    }
                }
            }
        }
    }
    require_true("coarse_fast_flick_release_settles", all_settled,
        "both coarse presets must settle after releases through 170 rad/s at center and endpoints");
}

static void test_local_profile_applies_and_resets_preset_tuning(void)
{
    static const char test_name[] = "local_profile_applies_and_resets_preset_tuning";
    require_true(test_name,
                 smart_knob_mode_apply_local_profile(
                     SMART_KNOB_MODE_COARSE_STRONG, 50U, 250U, 12U),
                 "local profile must accept a valid preset");

    const SmartKnobModeConfig *mode =
        smart_knob_mode_get(SMART_KNOB_MODE_COARSE_STRONG);
    require_true(test_name, mode != NULL, "profiled mode must exist");
    if (fabsf(mode->tuning.current_scale_a - 0.1f) > 0.0001f) {
        failf(test_name, "force must scale the preset current gain",
              mode->tuning.current_scale_a, 0.1f);
    }
    if (fabsf(mode->tuning.current_limit_a - 0.25f) > 0.0001f) {
        failf(test_name, "current limit must use milliamps",
              mode->tuning.current_limit_a, 0.25f);
    }
    const float width_deg = mode->config.position_width_radians * 180.0f / PI;
    if (fabsf(width_deg - 12.0f) > 0.01f) {
        failf(test_name, "menu step width must be applied in degrees",
              width_deg, 12.0f);
    }

    require_true(test_name,
                 smart_knob_mode_apply_local_profile(
                     SMART_KNOB_MODE_COARSE_STRONG, 100U, 450U, 8U),
                 "second profile application must succeed");
    mode = smart_knob_mode_get(SMART_KNOB_MODE_COARSE_STRONG);
    if (fabsf(mode->tuning.current_scale_a - 0.2f) > 0.0001f) {
        failf(test_name, "reapplying a profile must start from preset defaults",
              mode->tuning.current_scale_a, 0.2f);
    }
    require_true(test_name,
                 smart_knob_mode_default_width_deg(
                     SMART_KNOB_MODE_COARSE_STRONG) == 10U,
                 "coarse preset default width must match the host table");
}

static void test_navigation_mode_is_private_and_reversible(void)
{
    static const char test_name[] = "navigation_mode_is_private_and_reversible";
    reset_controller(SMART_KNOB_MODE_COARSE_STRONG);
    smart_knob_enter_navigation_mode();

    SmartKnobRuntimeState state = runtime_state(test_name);
    require_true(test_name,
                 state.active_mode == SMART_KNOB_NAVIGATION_MODE,
                 "menu must expose the private navigation mode at runtime");
    require_true(test_name,
                 smart_knob_active_config() != NULL,
                 "navigation mode must provide a valid haptic config");
    require_true(test_name,
                 smart_knob_modes_count() == SMART_KNOB_MODE_COUNT,
                 "navigation mode must not be added to the user preset list");

    require_true(test_name,
                 smart_knob_select_mode(SMART_KNOB_MODE_COARSE_STRONG),
                 "leaving the menu must restore a user preset");
    state = runtime_state(test_name);
    require_true(test_name,
                 state.active_mode == SMART_KNOB_MODE_COARSE_STRONG,
                 "restored preset must replace navigation mode");
}

static void move_for(float velocity, uint32_t milliseconds)
{
    for (uint32_t i = 0; i < milliseconds; ++i) {
        advance_and_handle(mechanical_rad + velocity * 0.001f, velocity, 1000U);
    }
}

static void test_frictionless_motion_and_stop(void)
{
    const char *name = "frictionless_motion_and_stop";
    smart_knob_modes_reset_defaults();
    reset_controller(SMART_KNOB_MODE_FRICTIONLESS);
    /* The 1 A/s specification applies to the mean loss compensation. The
       angle-dependent component has a separate, tested entry envelope. */
    smart_knob_set_test_cogging_scale(0.0f);
    require_true(name, smart_knob_modes_count() == 13U &&
        smart_knob_active_config()->config.min_position > smart_knob_active_config()->config.max_position,
        "new preset is appended and has no endstops");
    for (unsigned i = 0; i < 1000U; ++i) {
        advance_and_handle((i & 1U) ? 0.00038f : 0.0f, (i & 1U) ? 0.08f : -0.08f, 1000U);
        require_true(name, commanded_current_ma == 0.0f && zero_current_tracking,
            "stationary encoder noise must retain zero torque tracking without requesting assist");
    }
    float previous = 0.0f;
    for (unsigned i = 0; i < 200U; ++i) {
        move_for(10.0f, 1U);
        require_true(name, commanded_current_ma >= 0.0f &&
            commanded_current_ma - previous <= 1.01f,
            "assist must follow motion with a bounded attack rate");
        previous = commanded_current_ma;
    }
    require_true(name, commanded_current_ma > 10.0f && zero_current_tracking,
        "sustained motion gets loss compensation and active current tracking");
    move_for(-10.0f, 1U);
    require_true(name, commanded_current_ma == 0.0f,
        "a reversal must immediately discard the old assist");
    move_for(-10.0f, 100U);
    require_true(name, commanded_current_ma < -10.0f,
        "intentional reverse motion must get reverse compensation");
    motor_rps = -10.0f; /* Legacy 2 Hz filter still reports the previous spin. */
    motor_rps_fast = 0.0f;
    fake_time_us += 1000U;
    handle_smart_knob();
    require_true(name, commanded_current_ma == 0.0f && zero_current_tracking,
        "a stopped shaft must not be driven by the lagging legacy speed");
    move_for(3.0f, 25U);
    require_true(name, commanded_current_ma == 0.0f,
        "a brief cogging rebound after stopping must receive no assist");
}

static void test_frictionless_limits_and_recovery(void)
{
    const char *name = "frictionless_limits_and_recovery";
    smart_knob_modes_reset_defaults();
    reset_controller(SMART_KNOB_MODE_FRICTIONLESS);
    smart_knob_set_test_cogging_scale(0.0f);
    SmartKnobModeConfig *mode = smart_knob_mode_get_mutable(SMART_KNOB_MODE_FRICTIONLESS);
    require_true(name, fabsf(mode->tuning.current_limit_a - 0.8f) < 0.0001f &&
        mode->tuning.max_current_permille == 700U,
        "default mode and safety ratio must allow the full 800 mA output range");
    LocalProfileEdit projection;
    uint8_t mask = 0U;
    require_true(name, smart_knob_mode_local_projection(SMART_KNOB_MODE_FRICTIONLESS,
        &projection, &mask) && projection.current_limit_10ma == 80U &&
        (mask & SMART_KNOB_LOCAL_UNREPRESENTABLE_LIMIT) == 0U,
        "OLED projection must represent 800 mA without truncating the saved limit");
    projection.dirty_mask = LOCAL_PROFILE_DIRTY_LIMIT;
    projection.current_limit_10ma = 90U;
    require_true(name, smart_knob_mode_apply_local_edit(&projection) &&
        fabsf(mode->tuning.current_limit_a - 0.8f) < 0.0001f,
        "OLED limit edits must clamp to 800 mA for frictionless");
    require_true(name, smart_knob_mode_local_current_limit_max_ma(SMART_KNOB_MODE_COARSE_STRONG) == 450U,
        "other local modes must keep their existing 450 mA range");
    mode->tuning.current_scale_a = 1000.0f;
    move_for(10.0f, 1100U);
    require_true(name, fabsf(commanded_current_ma - 800.0f) <= 0.1f,
        "800 mA must be reachable with the preset safety ratio");
    mode->tuning.current_limit_a = 1.2f;
    mode->tuning.max_current_permille = 1000U;
    move_for(10.0f, 1100U);
    require_true(name, fabsf(commanded_current_ma - 800.0f) <= 0.1f,
        "even extreme host tuning must respect the frictionless 800 mA ceiling");
    move_for(-10.0f, 1100U);
    require_true(name, fabsf(commanded_current_ma + 800.0f) <= 0.1f,
        "the reverse output limit must also be exactly 800 mA");
    mode->tuning.max_current_permille = 10U;
    move_for(10.0f, 100U);
    require_true(name, commanded_current_ma <= 12.01f,
        "the existing current safety ratio must still apply");
    mode->tuning.current_limit_a = 0.005f;
    move_for(10.0f, 10U);
    require_true(name, commanded_current_ma <= 5.01f, "mode current limit must still apply");
    move_for(46.0f, 100U);
    require_true(name, fabsf(commanded_current_ma - 5.0f) < 0.01f && zero_current_tracking,
        "frictionless assist must not derate above the old 45 rad/s threshold");
    move_for(65.0f, 50U);
    require_true(name, fabsf(commanded_current_ma - 5.0f) < 0.01f,
        "frictionless assist must bypass the legacy 60 rad/s inhibit");
    move_for(10.0f, 100U);
    require_true(name, commanded_current_ma > 0.0f, "assist must recover after speed returns to range");
    advance_and_handle(mechanical_rad + 2.0f, 10.0f, 1000U);
    require_true(name, commanded_current_ma == 0.0f, "encoder discontinuities must inhibit assist");
    move_for(10.0f, 100U);
    require_true(name, commanded_current_ma > 0.0f, "valid samples must recover after an encoder glitch");
    advance_and_handle(mechanical_rad, NAN, 1000U);
    require_true(name, commanded_current_ma == 0.0f, "non-finite speed must never reach the actuator");
    move_for(10.0f, 100U);
    error_code = 1U;
    move_for(10.0f, 1U);
    require_true(name, commanded_current_ma == 0.0f, "faults must remove assist");
    error_code = 0U;
    output_enabled = 0U;
    move_for(10.0f, 100U);
    require_true(name, commanded_current_ma == 0.0f, "disabled output must discard assist state");
    output_enabled = 1U;
    advance_and_handle(mechanical_rad, 10.0f, 50000U);
    require_true(name, commanded_current_ma == 0.0f, "stale control steps must not continue assist");
    mode->tuning.current_scale_a = 0.0f;
    move_for(10.0f, 100U);
    require_true(name, commanded_current_ma == 0.0f && zero_current_tracking,
        "zero compensation at speed must still cancel back EMF through the PI loop");
    (void)smart_knob_select_mode(SMART_KNOB_MODE_CUSTOM);
    move_for(10.0f, 10U);
    require_true(name, commanded_current_ma == 0.0f && !zero_current_tracking,
        "leaving frictionless must restore the legacy zero-output policy");
}

static void test_frictionless_unlimited_speed(void)
{
    const char *name = "frictionless_unlimited_speed";
    smart_knob_modes_reset_defaults();
    reset_controller(SMART_KNOB_MODE_FRICTIONLESS);
    smart_knob_set_test_cogging_scale(0.0f);
    SmartKnobModeConfig *mode = smart_knob_mode_get_mutable(SMART_KNOB_MODE_FRICTIONLESS);
    require_true(name, fabsf(mode->tuning.current_scale_a - 0.987f) < 0.0001f &&
        fabsf(mode->tuning.friction_current_a - 0.022f) < 0.0001f &&
        fabsf(mode->tuning.viscous_friction_a_per_rad_s - 0.000304f) < 1e-7f &&
        mode->tuning.p_gain == 8.0f && fabsf(mode->tuning.d_gain - 0.010f) < 1e-7f,
        "the calibrated loss and virtual inertia preset must be available through normal mode configuration");
    /* Isolate the unrestricted loss law; acceleration and observer behavior
       are covered by the inertia cases at physically useful speeds. */
    mode->tuning.p_gain = 0.0f;
    const float speeds[] = {10.0f, 20.0f, 30.0f, 45.0f, 60.0f, 80.0f, 120.0f,
                            250.0f, 1000.0f, 2000.0f, 3000.0f, -120.0f, -250.0f, -3000.0f};
    for (unsigned i = 0U; i < sizeof(speeds) / sizeof(speeds[0]); ++i) {
        const float velocity = speeds[i];
        move_for(velocity, 1100U);
        const float expected_ma = copysignf(fminf(800.0f,
            987.0f * (0.022f + 0.000304f * fabsf(velocity))), velocity);
        require_true(name, fabsf(commanded_current_ma - expected_ma) < 0.1f,
            "both directions must receive the full requested current regardless of speed");
        const SmartKnobRuntimeState state = runtime_state(name);
        require_true(name, (state.flags & (1U << 3)) != 0U && (state.flags & (1U << 5)) == 0U,
            "continuous high-speed samples must remain valid without a speed inhibit flag");
    }
    /* A large mismatch remains a sensor discontinuity even during fast spin. */
    advance_and_handle(mechanical_rad + 2.0f, -120.0f, 1000U);
    require_true(name, commanded_current_ma == 0.0f,
        "removing speed caps must not remove the encoder discontinuity check");
    move_for(-120.0f, 300U);
    require_true(name, commanded_current_ma < -50.0f,
        "valid high-speed samples must recover after a discontinuity");
    over_vol_flag = 1U;
    move_for(-120.0f, 1U);
    require_true(name, commanded_current_ma == 0.0f, "overvoltage must still stop assist at high speed");
    over_vol_flag = 0U;
    (void)smart_knob_select_mode(SMART_KNOB_MODE_UNBOUNDED_SMOOTH);
    move_for(185.0f, 10U);
    require_true(name, commanded_current_ma == 0.0f &&
        (runtime_state(name).flags & (1U << 5)) != 0U,
        "legacy modes must retain their high-speed inhibit after leaving frictionless");
    (void)smart_knob_select_mode(SMART_KNOB_MODE_FRICTIONLESS);
    require_true(name, (runtime_state(name).flags & (1U << 5)) == 0U,
        "entering frictionless must immediately clear a legacy speed latch");
    move_for(120.0f, 300U);
    require_true(name, commanded_current_ma > 50.0f,
        "switching from an inhibited legacy mode must restore high-speed frictionless assist");
}

/* These fixtures set their own tuning so control invariants do not depend on
   the calibration values eventually selected for the product preset. */
static SmartKnobModeConfig *reset_frictionless_fixture(float start_angle, float ripple_gain)
{
    smart_knob_modes_reset_defaults();
    reset_controller(SMART_KNOB_MODE_FRICTIONLESS);
    smart_knob_set_test_cogging_scale(ripple_gain);
    SmartKnobModeConfig *mode = smart_knob_mode_get_mutable(SMART_KNOB_MODE_FRICTIONLESS);
    mode->tuning.p_gain = 0.0f;
    mode->tuning.d_gain = 0.0f;
    mode->tuning.current_scale_a = 1.0f;
    mode->tuning.friction_current_a = 0.040f;
    mode->tuning.viscous_friction_a_per_rad_s = 0.0f;
    mode->tuning.current_limit_a = 0.8f;
    mode->tuning.max_current_permille = 1000U;
    mechanical_rad = start_angle;
    init_smart_knob();
    advance_and_handle(start_angle, 0.0f, 301000U);
    advance_and_handle(start_angle, 0.0f, 1000U);
    return mode;
}

static float settled_mean_loss(float velocity, float running_friction_a, float viscous_a_per_rad_s)
{
    SmartKnobModeConfig *mode = reset_frictionless_fixture(0.0f, 0.0f);
    mode->tuning.friction_current_a = running_friction_a;
    mode->tuning.viscous_friction_a_per_rad_s = viscous_a_per_rad_s;
    /* Establish intentional motion first, then test the already-armed taper. */
    move_for(copysignf(6.0f, velocity), 100U);
    move_for(velocity, 200U);
    return commanded_current_ma;
}

static void test_frictionless_low_speed_loss_properties(void)
{
    const char *name = "frictionless_low_speed_loss_properties";
    const float speeds[] = {0.2001f, 0.25f, 0.35f, 0.45f, 0.55f,
                            0.599f, 0.600f, 0.601f, 1.0f, 2.0f,
                            3.0f, 4.0f, 6.0f, 8.0f, 12.0f};
    float forward[sizeof(speeds) / sizeof(speeds[0])];
    float previous_taper = 0.0f;
    for (unsigned i = 0U; i < sizeof(speeds) / sizeof(speeds[0]); ++i) {
        forward[i] = settled_mean_loss(speeds[i], 0.040f, 0.0f);
        const float reverse = settled_mean_loss(-speeds[i], 0.040f, 0.0f);
        require_true(name, forward[i] >= 0.0f && forward[i] < 80.0f &&
            fabsf(forward[i] + reverse) < 0.01f,
            "the low-speed shoulder and taper must remain bounded and directionally symmetric");
        if (i > 0U && i <= 5U) require_true(name, forward[i] > forward[i - 1U],
            "armed low-speed assistance must rise toward full compensation");
        if (i <= 7U) {
            /* Separate the gate from the declining running-loss shoulder.
               Their product can peak just before the gate reaches one, even
               though the gate itself remains smooth and monotone. */
            const float viscous_only = settled_mean_loss(speeds[i], 0.0f, 0.000275f);
            const float taper = viscous_only / (0.275f * speeds[i]);
            require_true(name, taper >= previous_taper - 1e-5f && taper <= 1.00001f,
                "the stop-to-full taper itself must rise monotonically without overshoot");
            if (i >= 6U) require_true(name, fabsf(taper - 1.0f) < 1e-5f,
                "the assist gate must be fully open at and above the full-speed threshold");
            previous_taper = taper;
        }
        if (i > 7U) require_true(name, forward[i] <= forward[i - 1U] + 0.01f,
            "extra low-speed loss must fade as speed rises instead of creating a high-speed boost");
    }
    require_true(name, forward[0] < 0.01f,
        "the armed taper must approach zero continuously at the stop threshold");
    require_true(name, forward[6] >= 40.0f && forward[10] > forward[14] + 0.01f,
        "full compensation must be available at 0.6 rad/s and the slow-running shoulder must be present");
    require_true(name, fabsf(forward[6] - forward[5]) < 0.05f &&
        fabsf(forward[7] - forward[6]) < 0.05f,
        "crossing the full-compensation threshold must not create a current discontinuity");
    require_true(name, fabsf(settled_mean_loss(3.0f, 0.020f, 0.0f) * 2.0f - forward[10]) < 0.01f,
        "the complete low-speed loss must remain proportional to running friction");
    const float normal_speeds[] = {12.0f, 20.0f, 60.0f, 120.0f};
    for (unsigned i = 0U; i < sizeof(normal_speeds) / sizeof(normal_speeds[0]); ++i) {
        const float speed = normal_speeds[i];
        const float expected = 40.0f + 0.275f * speed;
        require_true(name, fabsf(settled_mean_loss(speed, 0.040f, 0.000275f) - expected) < 0.03f &&
            fabsf(settled_mean_loss(-speed, 0.040f, 0.000275f) + expected) < 0.03f,
            "medium and high speeds must retain the measured approximately linear loss model");
    }

    SmartKnobModeConfig *mode = reset_frictionless_fixture(0.0f, 1.0f);
    mode->tuning.friction_current_a = 0.0f;
    const float zero_loss_speeds[] = {0.0f, 0.1f, 0.59f, 6.0f, 0.3f, 120.0f, -6.0f, -0.3f};
    for (unsigned i = 0U; i < sizeof(zero_loss_speeds) / sizeof(zero_loss_speeds[0]); ++i) {
        move_for(zero_loss_speeds[i], 200U);
        require_true(name, commanded_current_ma == 0.0f,
            "zero running and viscous loss with inertia off must produce no shoulder or spatial request");
    }
    reset_frictionless_fixture(0.0f, 1.0f);
    move_for(0.0f, 200U);
    move_for(0.59f, 200U);
    require_true(name, commanded_current_ma == 0.0f && zero_current_tracking,
        "the stronger slow-running compensation must not arm at rest or below the start threshold");
    move_for(0.6f, 100U);
    require_true(name, commanded_current_ma > 0.0f,
        "confirmed motion at the start threshold must receive compensation");
    move_for(0.0f, 1U);
    require_true(name, commanded_current_ma == 0.0f && zero_current_tracking,
        "stopping must still clear all low-speed compensation immediately");
}

static void observe_for(float velocity, uint32_t milliseconds)
{
    for (uint32_t i = 0U; i < milliseconds; ++i) {
        mechanical_rad += velocity * 0.001f;
        motor_rps = velocity;
        motor_rps_fast = velocity;
        fake_time_us += 1000U;
        smart_knob_observe_motion();
    }
}

static void test_frictionless_observer_does_not_write_actuator(void)
{
    const char *name = "frictionless_observer_does_not_write_actuator";
    reset_frictionless_fixture(0.0f, 1.0f);
    MotorDriverSetCurrentRealContinuous(37.0f); /* The diagnostic owns this command. */
    const uint32_t writes_before = actuator_write_count;
    observe_for(6.0f, 200U);
    require_true(name, runtime_state(name).commanded_current_ma > 20.0f,
        "observation must update and arm the normal haptic controller");
    require_true(name, actuator_write_count == writes_before &&
        commanded_current_ma == 37.0f && !zero_current_tracking,
        "normal observation must preserve the diagnostic's sole actuator command");

    error_code = 1U;
    observe_for(6.0f, 1U);
    require_true(name, runtime_state(name).commanded_current_ma == 0.0f,
        "observed faults must reset the simulated haptic output");
    error_code = 0U;
    output_enabled = 0U;
    observe_for(6.0f, 1U);
    output_enabled = 1U;
    mechanical_rad += 2.0f;
    observe_for(6.0f, 1U);
    require_true(name, (runtime_state(name).flags & (1U << 3)) == 0U,
        "observation must still reject encoder discontinuities");
    init_smart_knob();
    observe_for(6.0f, 1U); /* Startup inhibit is also an output-free path. */
    require_true(name, actuator_write_count == writes_before && commanded_current_ma == 37.0f,
        "fault, disabled, invalid-sample and startup observation must never write the actuator");
}

static void test_frictionless_observed_handover_is_continuous(void)
{
    const char *name = "frictionless_observed_handover_is_continuous";
    float continuous[12];
    reset_frictionless_fixture(0.0f, 1.0f);
    move_for(6.0f, 500U);
    for (unsigned i = 0U; i < 12U; ++i) {
        move_for(6.0f, 1U);
        continuous[i] = commanded_current_ma;
    }

    reset_frictionless_fixture(0.0f, 1.0f);
    MotorDriverSetCurrentRealContinuous(61.0f);
    const uint32_t writes_before = actuator_write_count;
    observe_for(6.0f, 500U);
    require_true(name, actuator_write_count == writes_before,
        "the diagnostic pulse must remain the sole actuator owner throughout observation");
    for (unsigned i = 0U; i < 12U; ++i) {
        move_for(6.0f, 1U);
        require_true(name, fabsf(commanded_current_ma - continuous[i]) < 0.01f &&
            commanded_current_ma > 20.0f && zero_current_tracking,
            "the first release sample and later samples must match continuous control without settling or rearming");
    }
}

static float settled_frictionless_output(float angle, float velocity, float ripple_gain)
{
    reset_frictionless_fixture(angle - velocity * 0.12f, ripple_gain);
    move_for(velocity, 120U);
    return commanded_current_ma;
}

static void test_frictionless_spatial_ripple(void)
{
    const char *name = "frictionless_spatial_ripple";
    const float pitch = 2.0f * PI;
    float spatial_sum = 0.0f, spatial_min = 0.0f, spatial_max = 0.0f;
    /* Uniform full-turn integration resolves all three identified mechanical
       harmonics (7, 12 and 14), whose common period is one revolution. */
    for (unsigned i = 0U; i < 128U; ++i) {
        const float angle = pitch * (float)i / 128.0f;
        const float forward_base = settled_frictionless_output(angle, 6.0f, 0.0f);
        const float reverse_base = settled_frictionless_output(angle, -6.0f, 0.0f);
        const float spatial = settled_frictionless_output(angle, 6.0f, 1.0f) - forward_base;
        const float reverse_spatial = settled_frictionless_output(angle, -6.0f, 1.0f) - reverse_base;
        const float next_period = settled_frictionless_output(angle + pitch, 6.0f, 1.0f) - forward_base;
        const float fast_base = settled_frictionless_output(angle, 120.0f, 0.0f);
        const float fast_spatial = settled_frictionless_output(angle, 120.0f, 1.0f) - fast_base;
        require_true(name, forward_base >= 40.0f &&
            fabsf(forward_base + reverse_base) < 0.01f,
            "angle gain zero must isolate a directionally symmetric running-loss request");
        require_true(name, fabsf(spatial - reverse_spatial) < 0.05f,
            "the spatial correction at the same angle must keep its sign when rotation reverses");
        require_true(name, fabsf(spatial - next_period) < 0.05f,
            "the combined mechanical angle map must repeat every full revolution");
        require_true(name, fabsf(spatial - fast_spatial) < 0.1f,
            "angle compensation must have no mean-slew-induced phase lag at higher speed");
        require_true(name, fabsf(spatial) < 32.3f,
            "the combined map must remain within the sum of its three harmonic amplitudes");
        spatial_sum += spatial;
        spatial_min = fminf(spatial_min, spatial);
        spatial_max = fmaxf(spatial_max, spatial);
    }
    require_true(name, spatial_min < -25.0f && spatial_max > 25.0f,
        "the measured combined map must retain both restoring-current polarities");
    require_true(name, fabsf(spatial_sum / 128.0f) < 0.01f,
        "full-revolution angle integration must reveal no directional DC torque bias");
}

static float settled_scaled_frictionless_output(float angle, float velocity, float scale, float ripple_gain)
{
    SmartKnobModeConfig *mode = reset_frictionless_fixture(angle - velocity * 1.2f, ripple_gain);
    mode->tuning.friction_current_a = 0.020f;
    mode->tuning.current_scale_a = scale;
    move_for(velocity, 1199U);
    /* Remove accumulated float integration error so the large-gain comparison
       samples exactly the same encoder angle. This is still a valid step. */
    advance_and_handle(angle, velocity, 1000U);
    return commanded_current_ma;
}

static void test_frictionless_ripple_envelope_limits_and_stop(void)
{
    const char *name = "frictionless_ripple_envelope_limits_and_stop";
    float mean_only[120], established_spatial[120];
    for (unsigned i = 0U; i < 120U; ++i) {
        const float angle = 0.010f * (float)(i + 1U);
        const float mean = settled_frictionless_output(angle, 10.0f, 0.0f);
        established_spatial[i] = settled_frictionless_output(angle, 10.0f, 1.0f) - mean;
    }
    reset_frictionless_fixture(0.0f, 0.0f);
    for (unsigned i = 0U; i < 120U; ++i) {
        move_for(10.0f, 1U);
        mean_only[i] = commanded_current_ma;
    }
    reset_frictionless_fixture(0.0f, 1.0f);
    float max_established_ripple = 0.0f;
    for (unsigned i = 0U; i < 120U; ++i) {
        move_for(10.0f, 1U);
        const float ripple = fabsf(commanded_current_ma - mean_only[i]);
        if (i < 5U) require_true(name, ripple == 0.0f, "angle compensation must wait for confirmed motion");
        const float entry_fraction = fminf(1.0f, i >= 5U ? (float)(i - 4U) / 50.0f : 0.0f);
        require_true(name, ripple <= 32.3f * entry_fraction + 0.01f,
            "entry must ramp the combined angle-map envelope over 50 ms");
        if (i >= 55U) {
            require_true(name, fabsf((commanded_current_ma - mean_only[i]) - established_spatial[i]) < 0.05f,
                "after entry the complete map must match established motion at the same angle");
            max_established_ripple = fmaxf(max_established_ripple, ripple);
        }
    }
    require_true(name, max_established_ripple > 20.0f,
        "the full spatial correction must be available after the 50 ms entry ramp");
    move_for(0.0f, 1U);
    require_true(name, commanded_current_ma == 0.0f && zero_current_tracking,
        "stopping must remove both the mean and spatial current exactly");
    move_for(3.0f, 25U);
    require_true(name, commanded_current_ma == 0.0f,
        "the angle term must not bypass the stop/rebound rearm delay");
    move_for(6.0f, 120U);
    error_code = 1U;
    move_for(6.0f, 1U);
    require_true(name, commanded_current_ma == 0.0f, "faults must also clear the spatial component");
    error_code = 0U;
    move_for(6.0f, 120U);
    over_vol_flag = 1U;
    move_for(6.0f, 1U);
    require_true(name, commanded_current_ma == 0.0f, "overvoltage must clear the spatial component");

    SmartKnobModeConfig *mode = reset_frictionless_fixture(0.0f, 1.0f);
    mode->tuning.current_scale_a = 1000.0f;
    mode->tuning.current_limit_a = 1.2f;
    bool reached_positive_cap = false, reached_negative_cap = false;
    for (unsigned i = 0U; i < 2400U; ++i) {
        const float velocity = i < 1200U ? 10.0f : -10.0f;
        move_for(velocity, 1U);
        require_true(name, fabsf(commanded_current_ma) <= 800.01f,
            "mean plus spatial compensation must obey the independent 800 mA ceiling");
        reached_positive_cap |= commanded_current_ma > 799.9f;
        reached_negative_cap |= commanded_current_ma < -799.9f;
    }
    require_true(name, reached_positive_cap && reached_negative_cap,
        "both signs of the combined output must reach but not exceed the hard cap");
    mode->tuning.current_limit_a = 0.3f;
    for (unsigned i = 0U; i < 100U; ++i) {
        move_for(10.0f, 1U);
        require_true(name, fabsf(commanded_current_ma) <= 300.01f,
            "the mode current limit must clamp the combined spatial and mean request");
    }
    mode->tuning.current_limit_a = 1.2f;
    mode->tuning.max_current_permille = 10U;
    for (unsigned i = 0U; i < 100U; ++i) {
        move_for(-10.0f, 1U);
        require_true(name, fabsf(commanded_current_ma) <= 12.01f,
            "the global safety ratio must also clamp the combined request during reversal");
    }
    bool opposed_forward_motion = false, opposed_reverse_motion = false;
    for (unsigned direction = 0U; direction < 2U; ++direction) {
        const float velocity = direction == 0U ? 6.0f : -6.0f;
        for (unsigned i = 0U; i < 64U; ++i) {
            const float angle = 2.0f * PI * (float)i / 64.0f;
            const float calibrated = settled_scaled_frictionless_output(angle, velocity, 1.0f, 1.0f);
            const float mean = settled_scaled_frictionless_output(angle, velocity, 1.0f, 0.0f);
            const float saturated = settled_scaled_frictionless_output(angle, velocity, 1000.0f, 1.0f);
            /* Preserve the shape relative to the complete running loss,
               including any speed-dependent shoulder, when its mean clips. */
            const float expected = fmaxf(-800.0f, fminf(800.0f, calibrated * 800.0f / fabsf(mean)));
            require_true(name, fabsf(saturated - expected) < 0.1f,
                "mean saturation must scale the spatial calibration proportionally before the final clamp");
            if (calibrated * velocity < 0.0f) {
                if (direction == 0U) opposed_forward_motion = true;
                else opposed_reverse_motion = true;
            }
        }
    }
    require_true(name, opposed_forward_motion && opposed_reverse_motion,
        "a spatial restoring torque larger than mean friction may correctly oppose motion at some angles");
}

static SmartKnobModeConfig *reset_inertia_fixture(float multiplier, float filter_seconds)
{
    SmartKnobModeConfig *mode = reset_frictionless_fixture(0.0f, 0.0f);
    mode->tuning.friction_current_a = 0.0f;
    mode->tuning.p_gain = multiplier;
    mode->tuning.d_gain = filter_seconds;
    return mode;
}

static void test_frictionless_inertia_follows_acceleration(void)
{
    const char *name = "frictionless_inertia_follows_acceleration";
    SmartKnobModeConfig *mode = reset_inertia_fixture(2.0f, 0.020f);
    for (unsigned i = 0U; i < 100U; ++i) {
        move_for(6.0f, 1U);
        require_true(name, commanded_current_ma == 0.0f,
            "arming into an established speed must not create an inertia kick");
    }
    for (unsigned i = 1U; i <= 200U; ++i) move_for(6.0f + 0.020f * (float)i, 1U);
    require_true(name, commanded_current_ma < -5.0f && commanded_current_ma > -5.5f,
        "two extra rotor inertias must oppose a sustained 20 rad/s squared acceleration by about 5.4 mA");
    move_for(10.0f, 500U);
    require_true(name, fabsf(commanded_current_ma) < 0.01f,
        "the newly selected constant speed must have no inertia drag or return-to-speed request");
    for (unsigned i = 1U; i <= 300U; ++i) move_for(10.0f - 0.020f * (float)i, 1U);
    require_true(name, commanded_current_ma > 5.0f && commanded_current_ma < 5.5f,
        "deceleration must receive forward inertia current instead of velocity damping");
    move_for(0.3f, 500U);
    require_true(name, fabsf(commanded_current_ma) < 0.01f && zero_current_tracking,
        "an already armed shaft held at a new continuous slow speed must not be locked to its old speed");

    move_for(-6.0f, 100U);
    for (unsigned i = 1U; i <= 200U; ++i) move_for(-6.0f - 0.020f * (float)i, 1U);
    require_true(name, commanded_current_ma > 5.0f && commanded_current_ma < 5.5f,
        "reverse acceleration must receive the opposite inertia polarity");
    move_for(-10.0f, 500U);
    for (unsigned i = 1U; i <= 200U; ++i) move_for(-10.0f + 0.020f * (float)i, 1U);
    require_true(name, commanded_current_ma < -5.0f && commanded_current_ma > -5.5f,
        "reverse deceleration must assist the existing negative rotation");
    mode->tuning.p_gain = 0.0f;
    move_for(-5.0f, 1U);
    require_true(name, commanded_current_ma == 0.0f, "gain zero must disable inertia immediately");
    mode->tuning.p_gain = 2.0f;
    move_for(-5.0f, 1U);
    require_true(name, commanded_current_ma == 0.0f,
        "enabling inertia while moving must initialize from the current speed");
}

static void test_frictionless_inertia_stop_and_recovery(void)
{
    const char *name = "frictionless_inertia_stop_and_recovery";
    reset_inertia_fixture(4.0f, 0.020f);
    move_for(6.0f, 100U);
    move_for(10.0f, 1U);
    require_true(name, commanded_current_ma < -1.0f, "fixture must contain an acceleration history");
    move_for(0.0f, 1U);
    require_true(name, commanded_current_ma == 0.0f && zero_current_tracking,
        "stopping must erase inertia torque while retaining normal zero Iq tracking");
    for (unsigned i = 0U; i < 100U; ++i) {
        move_for(-6.0f, 1U);
        require_true(name, commanded_current_ma == 0.0f,
            "rebound, deliberate reversal and rearming must not replay the old acceleration");
    }
    /* Check each reset while the velocity memory contains a real transient. */
    for (unsigned reset = 0U; reset < 4U; ++reset) {
        move_for(-6.0f, 100U);
        move_for(-10.0f, 1U);
        require_true(name, commanded_current_ma > 1.0f, "reset fixture must have nonzero inertia");
        if (reset == 0U) error_code = 1U;
        if (reset == 1U) output_enabled = 0U;
        if (reset == 2U) over_vol_flag = 1U;
        if (reset == 3U) advance_and_handle(mechanical_rad, -10.0f, 50000U);
        else move_for(-10.0f, 1U);
        require_true(name, commanded_current_ma == 0.0f && !zero_current_tracking,
            "fault, disabled output, overvoltage and stale samples must take the zero-output fault path");
        error_code = 0U;
        output_enabled = 1U;
        over_vol_flag = 0U;
        for (unsigned i = 0U; i < 100U; ++i) {
            move_for(-10.0f, 1U);
            require_true(name, commanded_current_ma == 0.0f,
                "recovery at a constant speed must start with fresh inertia history");
        }
    }
}

static float inertia_step_response(float multiplier, float filter_seconds)
{
    reset_inertia_fixture(multiplier, filter_seconds);
    move_for(6.0f, 100U);
    move_for(6.1f, 1U);
    return commanded_current_ma;
}

static void test_frictionless_inertia_parameter_and_current_limits(void)
{
    const char *name = "frictionless_inertia_parameter_and_current_limits";
    const float default_filter = inertia_step_response(2.0f, 0.0f);
    require_true(name, fabsf(default_filter - inertia_step_response(2.0f, 0.020f)) < 0.001f,
        "zero D must select the default 20 ms inertia filter");
    require_true(name, fabsf(inertia_step_response(80.0f, 0.020f) -
                             inertia_step_response(8.0f, 0.020f)) < 0.001f,
        "host input must not exceed eight extra inertias");
    require_true(name, fabsf(inertia_step_response(2.0f, 0.001f) -
                             inertia_step_response(2.0f, 0.005f)) < 0.001f &&
                       fabsf(inertia_step_response(2.0f, 1.0f) -
                             inertia_step_response(2.0f, 0.200f)) < 0.001f,
        "the inertia filter must clamp to the supported 5 to 200 ms range");
    require_true(name, inertia_step_response(INFINITY, 0.020f) == 0.0f,
        "non-finite inertia gain must never reach the actuator");
    require_true(name, fabsf(inertia_step_response(2.0f, INFINITY) - default_filter) < 0.001f,
        "non-finite filter time must use the finite default");

    SmartKnobModeConfig *mode = reset_inertia_fixture(8.0f, 0.005f);
    move_for(6.0f, 100U);
    move_for(120.0f, 1U);
    require_true(name, fabsf(commanded_current_ma + 40.0f) < 0.01f,
        "the inertia component must independently clamp to minus 40 mA");
    move_for(120.0f, 200U);
    move_for(6.0f, 1U);
    require_true(name, fabsf(commanded_current_ma - 40.0f) < 0.01f,
        "the inertia component must independently clamp to plus 40 mA");
    mode->tuning.current_limit_a = 0.005f;
    move_for(120.0f, 1U);
    require_true(name, fabsf(commanded_current_ma + 5.0f) < 0.01f,
        "the mode limit must also clamp inertia torque");
    mode->tuning.current_limit_a = 1.2f;
    mode->tuning.max_current_permille = 10U;
    move_for(120.0f, 200U);
    move_for(6.0f, 1U);
    require_true(name, fabsf(commanded_current_ma - 12.0f) < 0.01f,
        "the safety ratio must also clamp inertia torque");
    mode->tuning.max_current_permille = 1000U;
    mode->tuning.friction_current_a = 0.8f;
    smart_knob_set_test_cogging_scale(1.0f);
    for (unsigned direction = 0U; direction < 2U; ++direction) {
        const float sign = direction == 0U ? 1.0f : -1.0f;
        move_for(sign * 120.0f, 1200U);
        move_for(sign * 6.0f, 1U);
        require_true(name, fabsf(commanded_current_ma - sign * 800.0f) < 0.01f,
            "mean, angle correction and inertia together must respect the 800 mA hard cap in both directions");
    }
}

static void test_frictionless_inertia_observer_continuity(void)
{
    const char *name = "frictionless_inertia_observer_continuity";
    float reference[20];
    for (unsigned observed = 0U; observed < 2U; ++observed) {
        reset_inertia_fixture(4.0f, 0.020f);
        MotorDriverSetCurrentRealContinuous(61.0f);
        const uint32_t writes_before = actuator_write_count;
        for (unsigned i = 0U; i < 300U; ++i) {
            const float velocity = 3.0f + 0.010f * (float)i;
            if (observed != 0U) observe_for(velocity, 1U);
            else move_for(velocity, 1U);
        }
        if (observed != 0U) require_true(name, actuator_write_count == writes_before &&
            commanded_current_ma == 61.0f,
            "prewarming the inertia observer must leave diagnostic actuator ownership intact");
        for (unsigned i = 0U; i < 20U; ++i) {
            move_for(6.0f + 0.010f * (float)i, 1U);
            if (observed == 0U) reference[i] = commanded_current_ma;
            else require_true(name, fabsf(commanded_current_ma - reference[i]) < 0.001f &&
                commanded_current_ma < -5.0f && zero_current_tracking,
                "release during acceleration must preserve the prewarmed inertia state from its first sample");
        }
    }
}

int main(int argc, char **argv)
{
    if (argc != 2) {
        fputs("usage: smart_knob_regression_tests <case>\n", stderr);
        return EXIT_FAILURE;
    }

    if (strcmp(argv[1], "coarse-current-continuity") == 0) {
        test_coarse_strong_current_is_continuous_near_quarter_detent();
    } else if (strcmp(argv[1], "bounded-center-hold") == 0) {
        test_bounded_coarse_idle_does_not_move_detent_center();
    } else if (strcmp(argv[1], "exact-center-zero") == 0) {
        test_exact_center_still_commands_exact_zero();
    } else if (strcmp(argv[1], "coarse-damping-motion") == 0) {
        test_coarse_damping_uses_recent_motion();
    } else if (strcmp(argv[1], "coarse-flick-release") == 0) {
        test_coarse_flick_release_settles();
    } else if (strcmp(argv[1], "coarse-high-speed") == 0) {
        test_coarse_high_speed_retains_haptics();
        test_coarse_high_speed_hysteresis_and_sample_recovery();
    } else if (strcmp(argv[1], "coarse-fast-flick-release") == 0) {
        test_coarse_fast_flick_release_settles();
    } else if (strcmp(argv[1], "fine-high-speed") == 0) {
        test_fine_high_speed_tracks_detent_center();
    } else if (strcmp(argv[1], "local-profile") == 0) {
        test_local_profile_applies_and_resets_preset_tuning();
    } else if (strcmp(argv[1], "navigation-mode") == 0) {
        test_navigation_mode_is_private_and_reversible();
    } else if (strcmp(argv[1], "frictionless-motion") == 0) {
        test_frictionless_motion_and_stop();
        test_frictionless_low_speed_loss_properties();
    } else if (strcmp(argv[1], "frictionless-limits") == 0) {
        test_frictionless_limits_and_recovery();
    } else if (strcmp(argv[1], "frictionless-unlimited") == 0) {
        test_frictionless_unlimited_speed();
    } else if (strcmp(argv[1], "frictionless-observer") == 0) {
        test_frictionless_observer_does_not_write_actuator();
    } else if (strcmp(argv[1], "frictionless-handover") == 0) {
        test_frictionless_observed_handover_is_continuous();
    } else if (strcmp(argv[1], "frictionless-spatial") == 0) {
        test_frictionless_spatial_ripple();
    } else if (strcmp(argv[1], "frictionless-ripple-limits") == 0) {
        test_frictionless_ripple_envelope_limits_and_stop();
    } else if (strcmp(argv[1], "frictionless-inertia-motion") == 0) {
        test_frictionless_inertia_follows_acceleration();
    } else if (strcmp(argv[1], "frictionless-inertia-reset") == 0) {
        test_frictionless_inertia_stop_and_recovery();
    } else if (strcmp(argv[1], "frictionless-inertia-limits") == 0) {
        test_frictionless_inertia_parameter_and_current_limits();
    } else if (strcmp(argv[1], "frictionless-inertia-observer") == 0) {
        test_frictionless_inertia_observer_continuity();
    } else {
        fprintf(stderr, "unknown test case: %s\n", argv[1]);
        return EXIT_FAILURE;
    }

    printf("PASS %s\n", argv[1]);
    return EXIT_SUCCESS;
}
