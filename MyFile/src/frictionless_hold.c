#include "frictionless_hold.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static float clampf(float value, float low, float high)
{
    return fmaxf(low, fminf(value, high));
}

FrictionlessHoldConfig frictionless_hold_default_config(void)
{
    return (FrictionlessHoldConfig){
        .kp_a_per_rad_s = 0.016f,
        .ki_a_per_rad = 0.024f,
        .extra_current_limit_a = 0.024f,
        .start_velocity_rad_s = 0.6f,
        .stop_velocity_rad_s = 0.2f,
        .acceleration_tau_s = 0.020f,
        .gesture_acceleration_rad_s2 = 80.0f,
        .gesture_delta_rad_s = 0.8f,
        .gesture_confirm_s = 0.012f,
        .capture_quiet_acceleration_rad_s2 = 20.0f,
        .capture_quiet_s = 0.030f,
        .capture_max_s = 0.300f,
        .gesture_arm_error_rad_s = 0.7f,
        .gesture_arm_s = 0.120f,
        .restart_delay_s = 0.040f,
        .startup_acquire_min_s = 0.200f,
        .startup_acquire_max_s = 1.200f,
        .startup_quiet_acceleration_rad_s2 = 3.0f,
        .startup_quiet_s = 0.080f,
        .startup_trend_tau_s = 0.080f,
        .clutch_error_rad_s = 0.8f,
        .clutch_current_a = 0.006f,
        .clutch_confirm_s = 0.100f,
        .clutch_fast_current_a = 0.016f,
        .clutch_fast_confirm_s = 0.020f,
        .clutch_current_tau_s = 0.040f,
    };
}

void frictionless_hold_reset(FrictionlessHoldState *state)
{
    if (state != NULL) {
        memset(state, 0, sizeof(*state));
    }
}

static bool config_valid(const FrictionlessHoldConfig *config)
{
    if (config == NULL) {
        return false;
    }
#define VALID_NONNEGATIVE(field) (isfinite(config->field) && config->field >= 0.0f)
#define VALID_POSITIVE(field) (isfinite(config->field) && config->field > 0.0f)
    const bool valid =
        VALID_NONNEGATIVE(kp_a_per_rad_s) &&
        VALID_NONNEGATIVE(ki_a_per_rad) &&
        VALID_NONNEGATIVE(extra_current_limit_a) &&
        VALID_POSITIVE(start_velocity_rad_s) &&
        VALID_NONNEGATIVE(stop_velocity_rad_s) &&
        config->start_velocity_rad_s > config->stop_velocity_rad_s &&
        VALID_POSITIVE(acceleration_tau_s) &&
        VALID_POSITIVE(gesture_acceleration_rad_s2) &&
        VALID_POSITIVE(gesture_delta_rad_s) &&
        VALID_POSITIVE(gesture_confirm_s) &&
        VALID_NONNEGATIVE(capture_quiet_acceleration_rad_s2) &&
        VALID_POSITIVE(capture_quiet_s) && VALID_POSITIVE(capture_max_s) &&
        config->capture_max_s >= config->capture_quiet_s &&
        VALID_POSITIVE(gesture_arm_error_rad_s) && VALID_POSITIVE(gesture_arm_s) &&
        VALID_NONNEGATIVE(restart_delay_s) &&
        VALID_NONNEGATIVE(startup_acquire_min_s) &&
        VALID_POSITIVE(startup_acquire_max_s) &&
        config->startup_acquire_max_s > config->startup_acquire_min_s &&
        VALID_POSITIVE(startup_quiet_acceleration_rad_s2) &&
        VALID_POSITIVE(startup_quiet_s) && VALID_POSITIVE(startup_trend_tau_s) &&
        VALID_POSITIVE(clutch_error_rad_s) && VALID_POSITIVE(clutch_current_a) &&
        VALID_POSITIVE(clutch_confirm_s) && VALID_POSITIVE(clutch_fast_current_a) &&
        VALID_POSITIVE(clutch_fast_confirm_s) && VALID_POSITIVE(clutch_current_tau_s);
#undef VALID_NONNEGATIVE
#undef VALID_POSITIVE
    return valid;
}

static void clear_goal(FrictionlessHoldState *state)
{
    state->reference_velocity_rad_s = 0.0f;
    state->integral_current_a = 0.0f;
    state->correction_current_a = 0.0f;
    state->output_current_a = 0.0f;
    state->phase_elapsed_s = 0.0f;
    state->quiet_elapsed_s = 0.0f;
    state->gesture_elapsed_s = 0.0f;
    state->gesture_start_velocity_rad_s = 0.0f;
    state->gesture_arm_elapsed_s = 0.0f;
    state->gesture_direction = 0;
    state->gesture_ready = false;
    state->settled_integral_current_a = 0.0f;
    state->filtered_correction_current_a = 0.0f;
    state->clutch_elapsed_s = 0.0f;
    state->yield_deadline_s = 0.0f;
    state->yield_bias_current_a = 0.0f;
}

static void capture(FrictionlessHoldState *state, float velocity)
{
    clear_goal(state);
    state->phase = FRICTIONLESS_HOLD_TRACK;
    /* Hold the captured finite speed without a separate speed ceiling.
       Current budgets and stop/fault resets still apply. */
    state->reference_velocity_rad_s = velocity;
    state->direction = velocity > 0.0f ? 1 : -1;
}

static bool gesture_confirmed(FrictionlessHoldState *state,
                              const FrictionlessHoldConfig *config,
                              float velocity, float dt)
{
    const float error = state->reference_velocity_rad_s - velocity;
    if (!state->gesture_ready) {
        /* Do not interpret startup/model settling as a new hand gesture. */
        if (fabsf(state->reference_velocity_rad_s - state->capture_velocity_rad_s) <=
            config->gesture_arm_error_rad_s) {
            state->gesture_arm_elapsed_s += dt;
            if (state->gesture_arm_elapsed_s >= config->gesture_arm_s) {
                state->gesture_ready = true;
                state->settled_integral_current_a = state->integral_current_a;
            }
        } else {
            state->gesture_arm_elapsed_s = 0.0f;
        }
        return false;
    }

    const float acceleration = state->acceleration_rad_s2;
    const int8_t direction = acceleration > 0.0f ? 1 : -1;
    /* Require sustained acceleration AWAY from the frozen reference plus a
       sizable same-direction velocity change within that episode. Periodic
       cogging and slow temperature/model drift must not move the reference. */
    if (fabsf(acceleration) < config->gesture_acceleration_rad_s2 ||
        acceleration * error >= 0.0f) {
        state->gesture_elapsed_s = 0.0f;
        state->gesture_direction = 0;
        return false;
    }
    if (direction != state->gesture_direction) {
        state->gesture_direction = direction;
        state->gesture_start_velocity_rad_s = velocity;
        state->gesture_elapsed_s = 0.0f;
    }
    state->gesture_elapsed_s += dt;
    return state->gesture_elapsed_s >= config->gesture_confirm_s &&
        (velocity - state->gesture_start_velocity_rad_s) * (float)direction >=
            config->gesture_delta_rad_s;
}

static bool clutch_confirmed(FrictionlessHoldState *state,
                             const FrictionlessHoldConfig *config,
                             float nominal, float dt)
{
    if (!state->gesture_ready) {
        state->clutch_elapsed_s = 0.0f;
        return false;
    }
    const float error = state->reference_velocity_rad_s - state->filtered_velocity_rad_s;
    /* Compare with the learned loss bias, not with zero current: ordinary
       bearing friction may already need a substantial steady correction. */
    const float correction = state->output_current_a - nominal;
    state->filtered_correction_current_a += dt / (config->clutch_current_tau_s + dt) *
        (correction - state->filtered_correction_current_a);
    const float additional = state->filtered_correction_current_a - state->settled_integral_current_a;
    if (fabsf(error) >= config->clutch_error_rad_s &&
        fabsf(additional) >= config->clutch_current_a && additional * error > 0.0f) {
        state->clutch_elapsed_s += dt;
    } else {
        state->clutch_elapsed_s = 0.0f;
        if (fabsf(error) < config->clutch_error_rad_s * 0.25f) {
            state->settled_integral_current_a += dt / (0.5f + dt) *
                (state->integral_current_a - state->settled_integral_current_a);
        }
    }
    return state->clutch_elapsed_s >= config->clutch_confirm_s ||
        (fabsf(additional) >= config->clutch_fast_current_a &&
         state->clutch_elapsed_s >= config->clutch_fast_confirm_s &&
         state->acceleration_rad_s2 * error < 0.0f);
}

float frictionless_hold_step(FrictionlessHoldState *state,
                            const FrictionlessHoldConfig *config,
                            float velocity_rad_s, float feedforward_a,
                            float current_limit_a, float dt_s, bool enabled)
{
    if (state == NULL) {
        return 0.0f;
    }
    if (!config_valid(config) || !isfinite(velocity_rad_s) ||
        !isfinite(feedforward_a) || !isfinite(current_limit_a) ||
        !isfinite(dt_s) || dt_s <= 0.0f || dt_s > 0.020f ||
        current_limit_a <= 0.0f) {
        frictionless_hold_reset(state);
        return 0.0f;
    }

    if (!state->velocity_initialized) {
        state->filtered_velocity_rad_s = velocity_rad_s;
        state->capture_velocity_rad_s = velocity_rad_s;
        state->velocity_initialized = true;
    }
    state->acceleration_rad_s2 =
        (velocity_rad_s - state->filtered_velocity_rad_s) /
            (config->acceleration_tau_s + dt_s);
    state->filtered_velocity_rad_s += dt_s * state->acceleration_rad_s2;
    const float capture_acceleration =
        (state->filtered_velocity_rad_s - state->capture_velocity_rad_s) /
            (config->startup_trend_tau_s + dt_s);
    state->capture_velocity_rad_s += dt_s * capture_acceleration;
    state->capture_acceleration_rad_s2 += dt_s / (config->startup_trend_tau_s + dt_s) *
        (capture_acceleration - state->capture_acceleration_rad_s2);

    if (!enabled) {
        clear_goal(state);
        state->phase = FRICTIONLESS_HOLD_OBSERVE;
        state->direction = 0;
        state->restart_remaining_s = 0.0f;
        state->control_active = false;
        return 0.0f;
    }

    const float speed = fabsf(velocity_rad_s);
    const int8_t direction = velocity_rad_s > 0.0f ? 1 : -1;
    state->restart_remaining_s = fmaxf(0.0f, state->restart_remaining_s - dt_s);
    if (speed <= config->stop_velocity_rad_s) {
        if (state->direction != 0) {
            state->restart_remaining_s = config->restart_delay_s;
        }
        clear_goal(state);
        state->phase = FRICTIONLESS_HOLD_REST;
        state->direction = 0;
        state->control_active = true;
        /* A subsequent flick starts a fresh acceleration estimate. */
        state->filtered_velocity_rad_s = velocity_rad_s;
        state->acceleration_rad_s2 = 0.0f;
        state->capture_velocity_rad_s = velocity_rad_s;
        state->capture_acceleration_rad_s2 = 0.0f;
        return 0.0f;
    }

    if (state->direction != 0 && state->direction != direction) {
        clear_goal(state);
        state->phase = FRICTIONLESS_HOLD_REST;
        state->direction = 0;
        state->restart_remaining_s = config->restart_delay_s;
        state->filtered_velocity_rad_s = velocity_rad_s;
        state->acceleration_rad_s2 = 0.0f;
        state->capture_velocity_rad_s = velocity_rad_s;
        state->capture_acceleration_rad_s2 = 0.0f;
    }
    if (!state->control_active || state->phase == FRICTIONLESS_HOLD_REST ||
        state->phase == FRICTIONLESS_HOLD_OBSERVE) {
        const bool from_rest = state->phase != FRICTIONLESS_HOLD_OBSERVE;
        state->control_active = true;
        if (speed < config->start_velocity_rad_s || state->restart_remaining_s > 0.0f) {
            return 0.0f;
        }
        capture(state, from_rest ? velocity_rad_s : state->capture_velocity_rad_s);
        if (from_rest) {
            /* One bounded launch window accommodates gentle hand acceleration.
               Its deadline cannot be extended by feedforward-induced motion. */
            state->phase = FRICTIONLESS_HOLD_ACQUIRE;
        }
    }

    const float nominal = (float)direction * clampf(
        (float)direction * feedforward_a, 0.0f, current_limit_a);
    const float extra = fminf(config->extra_current_limit_a, current_limit_a);
    const float lower = direction > 0 ? -extra : -fminf(-nominal + extra, current_limit_a);
    const float upper = direction > 0 ? fminf(nominal + extra, current_limit_a) : extra;

    if (state->phase == FRICTIONLESS_HOLD_ACQUIRE) {
        state->phase_elapsed_s += dt_s;
        if (state->phase_elapsed_s >= config->startup_acquire_min_s &&
            fabsf(state->capture_acceleration_rad_s2) <=
                config->startup_quiet_acceleration_rad_s2) {
            state->quiet_elapsed_s += dt_s;
        } else {
            state->quiet_elapsed_s = 0.0f;
        }
        if (state->quiet_elapsed_s >= config->startup_quiet_s ||
            state->phase_elapsed_s >= config->startup_acquire_max_s) {
            capture(state, state->capture_velocity_rad_s);
        } else {
            state->output_current_a = nominal;
            return nominal;
        }
    }

    if (state->phase == FRICTIONLESS_HOLD_TRACK) {
        state->phase_elapsed_s += dt_s;
    }
    if (state->phase == FRICTIONLESS_HOLD_TRACK) {
        const bool fast_gesture = gesture_confirmed(state, config, velocity_rad_s, dt_s);
        const bool slow_gesture = clutch_confirmed(state, config, nominal, dt_s);
        if (fast_gesture || slow_gesture) {
            const float learned_bias = state->settled_integral_current_a;
            clear_goal(state);
            state->phase = FRICTIONLESS_HOLD_YIELD;
            state->yield_bias_current_a = learned_bias;
            state->yield_deadline_s = fast_gesture
                ? config->capture_max_s : config->startup_acquire_max_s;
        }
    }

    if (state->phase == FRICTIONLESS_HOLD_YIELD) {
        state->phase_elapsed_s += dt_s;
        if (fabsf(state->capture_acceleration_rad_s2) <=
            config->startup_quiet_acceleration_rad_s2) {
            state->quiet_elapsed_s += dt_s;
        } else {
            state->quiet_elapsed_s = 0.0f;
        }
        if (state->quiet_elapsed_s >= config->capture_quiet_s ||
            state->phase_elapsed_s >= state->yield_deadline_s) {
            /* A short velocity average avoids choosing the trough/crest of
               the residual cogging waveform as the new release target. */
            const float learned_bias = state->yield_bias_current_a;
            capture(state, state->capture_velocity_rad_s);
            state->integral_current_a = clampf(learned_bias, lower - nominal, upper - nominal);
            state->settled_integral_current_a = state->integral_current_a;
            state->filtered_correction_current_a = state->integral_current_a;
        } else {
            state->output_current_a = clampf(nominal + state->yield_bias_current_a, lower, upper);
            state->correction_current_a = state->output_current_a - nominal;
            return state->output_current_a;
        }
    }

    const float error = state->reference_velocity_rad_s - velocity_rad_s;
    const float proportional = config->kp_a_per_rad_s * error;
    /* The correction can cancel ALL nominal loss current, even when an old
       saved setting requests excessive feedforward. A symmetric correction
       cap would leave a runaway above that modeling error. */
    state->integral_current_a = clampf(state->integral_current_a,
                                      lower - nominal, upper - nominal);
    const float unconstrained = nominal + proportional + state->integral_current_a;
    const float delta = config->ki_a_per_rad * error * dt_s;
    if ((unconstrained < upper || delta < 0.0f) &&
        (unconstrained > lower || delta > 0.0f)) {
        state->integral_current_a = clampf(state->integral_current_a + delta,
                                          lower - nominal, upper - nominal);
    }
    state->output_current_a = clampf(nominal + proportional + state->integral_current_a,
                                    lower, upper);
    state->correction_current_a = state->output_current_a - nominal;
    return state->output_current_a;
}
