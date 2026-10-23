#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "motion_estimator.h"

#define TAU_RAD 6.2831853071795864769f
#define COUNT_RAD (TAU_RAD / 16384.0f)

static void require(bool condition, const char *message)
{
    if (!condition) {
        fprintf(stderr, "FAIL motion_estimator: %s\n", message);
        exit(EXIT_FAILURE);
    }
}

static void near(float actual, float expected, float tolerance, const char *message)
{
    if (!isfinite(actual) || fabsf(actual - expected) > tolerance) {
        fprintf(stderr, "FAIL motion_estimator: %s (actual=%.9g expected=%.9g)\n",
                message, (double)actual, (double)expected);
        exit(EXIT_FAILURE);
    }
}

static void initial_position_and_reset(void)
{
    MotionEstimator state = {0};
    require(motion_estimator_update(&state, 12288U, 1000U), "first sample accepted");
    near(motion_estimator_position_rad(&state), 0.75f * TAU_RAD, 1e-6f,
         "initial sample above half turn keeps its absolute angle");
    require(motion_estimator_turns(&state) == 0, "first sample does not invent a turn");
    near(motion_estimator_velocity_rad_s(&state), 0.0f, 0.0f, "initial velocity is zero");
    require(motion_estimator_update(&state, 12500U, 2000U), "moving sample accepted");
    motion_estimator_reset(&state, 4096U, 3000U);
    near(motion_estimator_position_rad(&state), 0.25f * TAU_RAD, 1e-6f,
         "calibration reset establishes the new absolute angle");
    near(motion_estimator_velocity_rad_s(&state), 0.0f, 0.0f,
         "calibration reset clears speed history");
    require(motion_estimator_turns(&state) == 0, "calibration reset clears turns");
}

static void bidirectional_wrap(void)
{
    MotionEstimator state = {0};
    motion_estimator_reset(&state, 16383U, 0U);
    const float start = motion_estimator_position_rad(&state);
    require(motion_estimator_update(&state, 0U, 1000U), "positive wrap accepted");
    require(motion_estimator_turns(&state) == 1, "positive wrap adds one turn");
    near(motion_estimator_position_rad(&state) - start, COUNT_RAD, 6e-7f,
         "16383 to zero is one count, not zero");
    near(motion_estimator_velocity_rad_s(&state), COUNT_RAD / 0.009f, 1e-6f,
         "positive wrap contributes one velocity count");
    require(motion_estimator_update(&state, 16383U, 2000U), "negative wrap accepted");
    require(motion_estimator_turns(&state) == 0, "negative wrap subtracts one turn");
    near(motion_estimator_position_rad(&state), start, 1e-6f, "round trip restores position");

    motion_estimator_reset(&state, 0U, 0U);
    require(motion_estimator_update(&state, 16383U, 1000U), "negative initial wrap accepted");
    require(motion_estimator_turns(&state) == -1, "negative wrap can cross below zero");
    near(motion_estimator_position_rad(&state), -COUNT_RAD, 6e-7f, "negative count position");
}

static void actual_elapsed_time(void)
{
    MotionEstimator state = {0};
    motion_estimator_reset(&state, 1000U, 1000U);
    require(motion_estimator_update(&state, 1020U, 3000U), "two millisecond sample accepted");
    const float first_velocity = 20.0f * COUNT_RAD / 0.010f;
    near(motion_estimator_velocity_rad_s(&state), first_velocity, 1e-6f,
         "speed and filter use actual two millisecond interval");
    require(!motion_estimator_update(&state, 1030U, 3000U), "duplicate time rejected");
    near(motion_estimator_position_rad(&state), 1020.0f * COUNT_RAD, 1e-6f,
         "duplicate time preserves position baseline");
    near(motion_estimator_velocity_rad_s(&state), first_velocity, 1e-6f,
         "duplicate time preserves velocity history");
    require(motion_estimator_update(&state, 1040U, 4000U), "next timed sample accepted");
    near(motion_estimator_velocity_rad_s(&state),
         first_velocity + (20.0f * COUNT_RAD / 0.001f - first_velocity) / 9.0f, 1e-6f,
         "next timed sample includes displacement from skipped duplicate time");
}

static void timestamp_wrap_and_pause(void)
{
    MotionEstimator state = {0};
    motion_estimator_reset(&state, 1000U, UINT32_MAX - 499U);
    require(motion_estimator_update(&state, 1010U, 500U), "wrapped timestamp accepted");
    near(motion_estimator_velocity_rad_s(&state), 10.0f * COUNT_RAD / 0.009f, 1e-6f,
         "unsigned timestamp difference survives micros wrap");
    require(motion_estimator_update(&state, 2000U, 1000500U), "long pause resynchronizes");
    near(motion_estimator_position_rad(&state), 2000.0f * COUNT_RAD, 1e-6f,
         "long pause retains latest absolute position");
    near(motion_estimator_velocity_rad_s(&state), 0.0f, 0.0f,
         "long pause discards stale velocity without a spike");
    require(motion_estimator_update(&state, 2010U, 1001500U), "motion resumes after pause");
    near(motion_estimator_velocity_rad_s(&state), 10.0f * COUNT_RAD / 0.009f, 1e-6f,
         "post-pause speed uses the fresh baseline");
}

static void invalid_count_preserves_history(void)
{
    MotionEstimator state = {0};
    motion_estimator_reset(&state, 1000U, 1000U);
    require(!motion_estimator_update(&state, 16384U, 1500U), "out-of-range count rejected");
    require(motion_estimator_update(&state, 1020U, 2000U), "valid count recovers");
    near(motion_estimator_velocity_rad_s(&state), 20.0f * COUNT_RAD / 0.009f, 1e-6f,
         "invalid count does not consume time or displacement");
}

int main(void)
{
    initial_position_and_reset();
    bidirectional_wrap();
    actual_elapsed_time();
    timestamp_wrap_and_pause();
    invalid_count_preserves_history();
    puts("PASS motion_estimator");
    return EXIT_SUCCESS;
}
