#include "motion_estimator.h"

#include <limits.h>
#include <stddef.h>

#define MOTION_ENCODER_COUNTS 16384
#define MOTION_TAU_RAD 6.2831853071795864769f
#define MOTION_VELOCITY_TAU_S 0.008f
#define MOTION_MAX_INTERVAL_US 20000U

void motion_estimator_reset(MotionEstimator *state, uint16_t count, uint32_t now_us)
{
    if (state == NULL || count >= MOTION_ENCODER_COUNTS) {
        return;
    }
    *state = (MotionEstimator){
        .sample_us = now_us,
        .previous_count = count,
        .initialized = true,
    };
}

bool motion_estimator_update(MotionEstimator *state, uint16_t count, uint32_t now_us)
{
    if (state == NULL || count >= MOTION_ENCODER_COUNTS) {
        return false;
    }
    if (!state->initialized) {
        motion_estimator_reset(state, count, now_us);
        return true;
    }

    const uint32_t elapsed_us = now_us - state->sample_us;
    if (elapsed_us == 0U) {
        return false;
    }

    /* Like the reference project's AngleTracker, keep integer turns and an
       absolute sample instead of integrating float position increments. */
    int32_t delta = (int32_t)count - state->previous_count;
    int32_t turns = state->turns;
    if (delta > MOTION_ENCODER_COUNTS / 2) {
        if (turns == INT32_MIN) {
            return false;
        }
        --turns;
        delta -= MOTION_ENCODER_COUNTS;
    } else if (delta < -MOTION_ENCODER_COUNTS / 2) {
        if (turns == INT32_MAX) {
            return false;
        }
        ++turns;
        delta += MOTION_ENCODER_COUNTS;
    }

    if (elapsed_us > MOTION_MAX_INTERVAL_US) {
        /* Rejoin at the nearest absolute angle after a scheduling pause. A
           single-turn encoder cannot recover whole turns missed in that gap. */
        state->velocity_rad_s = 0.0f;
    } else {
        const float dt = (float)elapsed_us * 1e-6f;
        const float velocity = (float)delta * (MOTION_TAU_RAD / MOTION_ENCODER_COUNTS) / dt;
        state->velocity_rad_s += dt / (MOTION_VELOCITY_TAU_S + dt) *
                                 (velocity - state->velocity_rad_s);
    }
    state->turns = turns;
    state->previous_count = count;
    state->sample_us = now_us;
    return true;
}

float motion_estimator_position_rad(const MotionEstimator *state)
{
    if (state == NULL || !state->initialized) {
        return 0.0f;
    }
    return (float)state->turns * MOTION_TAU_RAD +
           (float)state->previous_count * (MOTION_TAU_RAD / MOTION_ENCODER_COUNTS);
}

float motion_estimator_velocity_rad_s(const MotionEstimator *state)
{
    return state == NULL ? 0.0f : state->velocity_rad_s;
}

int32_t motion_estimator_turns(const MotionEstimator *state)
{
    return state == NULL ? 0 : state->turns;
}
