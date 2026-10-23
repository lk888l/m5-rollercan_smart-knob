#ifndef MOTION_ESTIMATOR_H
#define MOTION_ESTIMATOR_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t sample_us;
    int32_t turns;
    uint16_t previous_count;
    float velocity_rad_s;
    bool initialized;
} MotionEstimator;

/* Input is the direction/offset-corrected 14-bit mechanical encoder count. */
void motion_estimator_reset(MotionEstimator *state, uint16_t count, uint32_t now_us);
bool motion_estimator_update(MotionEstimator *state, uint16_t count, uint32_t now_us);
float motion_estimator_position_rad(const MotionEstimator *state);
float motion_estimator_velocity_rad_s(const MotionEstimator *state);
int32_t motion_estimator_turns(const MotionEstimator *state);

#ifdef __cplusplus
}
#endif

#endif
