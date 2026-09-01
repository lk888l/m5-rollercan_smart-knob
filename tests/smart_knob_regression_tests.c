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
float ph_crrent_lpf;

static uint32_t fake_time_us;
static float commanded_current_ma;
static uint8_t output_enabled = 1U;

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
    commanded_current_ma = fabsf(phase_current_ma) <= 60.0f
                               ? 0.0f
                               : phase_current_ma;
}

void MotorDriverSetCurrentRealContinuous(float phase_current_ma)
{
    commanded_current_ma = phase_current_ma;
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
    } else if (strcmp(argv[1], "local-profile") == 0) {
        test_local_profile_applies_and_resets_preset_tuning();
    } else if (strcmp(argv[1], "navigation-mode") == 0) {
        test_navigation_mode_is_private_and_reversible();
    } else {
        fprintf(stderr, "unknown test case: %s\n", argv[1]);
        return EXIT_FAILURE;
    }

    printf("PASS %s\n", argv[1]);
    return EXIT_SUCCESS;
}
