#ifndef FRICTIONLESS_CALIBRATION_H
#define FRICTIONLESS_CALIBRATION_H

#include <math.h>

/* Current-equivalent spatial disturbance measured on the module attached to
 * ST-Link 01290141524300143638414B, encoder offset 7774 (2026-09-08).
 * Bidirectional dense captures, common 8 ms filtering and 10 ms intervals;
 * per-capture means removed before fitting harmonics 7, 12 and 14 at lag 0.
 * Shape held-out RMS: 5.755 mA; see dense-centered-shape.json.
 * These are shaft-angle terms, with the same current sign in both directions.
 * Re-identify this calibration after replacing/recalibrating the motor. */
static inline float frictionless_angle_current_a(float shaft_angle_rad)
{
    const float angle = fmodf(shaft_angle_rad, 6.28318530718f);
    return 0.0022761929f * sinf(7.0f * angle) + 0.0049384429f * cosf(7.0f * angle)
         - 0.0049728291f * sinf(12.0f * angle) - 0.0146048373f * cosf(12.0f * angle)
         + 0.0000291448f * sinf(14.0f * angle) + 0.0113851361f * cosf(14.0f * angle);
}

#endif
