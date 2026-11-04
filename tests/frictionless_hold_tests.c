#include "frictionless_hold.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

static void require(bool condition, const char *message)
{
    if (!condition) {
        fprintf(stderr, "FAIL frictionless_hold: %s\n", message);
        exit(EXIT_FAILURE);
    }
}

static void near(float actual, float expected, float tolerance, const char *message)
{
    if (!isfinite(actual) || fabsf(actual - expected) > tolerance) {
        fprintf(stderr, "FAIL frictionless_hold: %s (actual=%g expected=%g)\n",
                message, (double)actual, (double)expected);
        exit(EXIT_FAILURE);
    }
}

typedef struct {
    FrictionlessHoldState controller;
    FrictionlessHoldConfig config;
    float velocity;
    float measured_velocity;
    float angle;
    float inertia_a_s2_rad;
    float loss_a;
    float viscous_a_s_rad;
    float feedforward_scale;
    float output_limit_a;
} Plant;

static Plant plant(float velocity, float feedforward_scale)
{
    Plant p = {
        .config = frictionless_hold_default_config(),
        .velocity = velocity,
        .measured_velocity = velocity,
        .inertia_a_s2_rad = 0.000135f,
        .loss_a = 0.022f,
        .viscous_a_s_rad = 0.000304f,
        .feedforward_scale = feedforward_scale,
        .output_limit_a = 0.080f,
    };
    if (velocity != 0.0f) {
        frictionless_hold_step(&p.controller, &p.config, velocity, 0.024f,
                              p.output_limit_a, 0.001f, false);
    }
    return p;
}

static float plant_step(Plant *p, float external_current_a, float ripple_a)
{
    const float dt = 0.001f;
    const float nominal = copysignf(p->feedforward_scale *
        (0.022f + 0.000304f * fabsf(p->measured_velocity)), p->measured_velocity);
    const float output = frictionless_hold_step(&p->controller, &p->config,
        p->measured_velocity, nominal, p->output_limit_a, dt, true);
    const float loss = copysignf(p->loss_a, p->velocity) + p->viscous_a_s_rad * p->velocity;
    float net = output + external_current_a - loss + ripple_a * sinf(12.0f * p->angle);
    const float next = p->velocity + net / p->inertia_a_s2_rad * dt;
    /* Static friction holds a crossing when applied torque cannot break away. */
    if (next * p->velocity <= 0.0f && fabsf(output + external_current_a) <= p->loss_a) {
        p->velocity = 0.0f;
    } else {
        p->velocity = next;
    }
    p->angle += p->velocity * dt;
    p->measured_velocity += dt / (0.008f + dt) * (p->velocity - p->measured_velocity);
    require(isfinite(output) && fabsf(output) <= p->output_limit_a + 1e-7f,
            "plant output respects effective current budget");
    return output;
}

static void fixed_reference_cancels_model_error(void)
{
    const float speeds[] = {3.0f, -3.0f, 6.0f, -6.0f, 20.0f, -20.0f};
    const float scales[] = {0.8f, 1.0f, 1.2f};
    for (unsigned int v = 0; v < sizeof(speeds) / sizeof(speeds[0]); ++v) {
        for (unsigned int f = 0; f < sizeof(scales) / sizeof(scales[0]); ++f) {
            Plant p = plant(speeds[v], scales[f]);
            float max_speed = fabsf(p.velocity);
            for (int i = 0; i < 20000; ++i) {
                /* Viscous losses drift with temperature, independently of FF. */
                p.viscous_a_s_rad = 0.00024f + 0.00012f * (float)i / 20000.0f;
                plant_step(&p, 0.0f, 0.0008f);
                max_speed = fmaxf(max_speed, fabsf(p.velocity));
                near(p.controller.reference_velocity_rad_s, speeds[v], 0.0001f,
                     "loss error/ripple must never rebase the release reference");
            }
            near(p.velocity, speeds[v], 0.25f, "closed loop maintains release speed for 20 seconds");
            require(max_speed < fabsf(speeds[v]) + 1.5f, "loss error does not cause runaway");
        }
    }
}

static void exaggerated_feedforward_can_be_fully_cancelled(void)
{
    for (int direction = -1; direction <= 1; direction += 2) {
        Plant p = plant(6.0f * (float)direction, 3.0f);
        for (int i = 0; i < 12000; ++i) {
            plant_step(&p, 0.0f, 0.0f);
            near(p.controller.reference_velocity_rad_s, 6.0f * (float)direction, 0.0001f,
                 "large startup error cannot repeatedly reacquire a faster reference");
        }
        near(p.velocity, 6.0f * (float)direction, 0.05f,
             "feedback cancels more than the extra-current limit when FF is excessive");
        require(fabsf(p.controller.correction_current_a) > 0.040f,
                "negative correction can cancel the complete nominal feedforward");
    }
}

static void slow_drift_and_short_impulses_do_not_rebase(void)
{
    FrictionlessHoldState state = {0};
    const FrictionlessHoldConfig config = frictionless_hold_default_config();
    for (int i = 0; i < 500; ++i) {
        frictionless_hold_step(&state, &config, 6.0f, 0.024f, 0.08f, 0.001f, true);
    }
    require(state.gesture_ready, "steady running arms the deliberate gesture detector");
    for (int i = 0; i < 10000; ++i) {
        float velocity = 6.0f + 0.00005f * (float)i;
        if (i == 2000 || i == 7000) {
            velocity += 2.0f;
        }
        frictionless_hold_step(&state, &config, velocity, 0.026f, 0.08f, 0.001f, true);
        near(state.reference_velocity_rad_s, 6.0f, 0.0f,
             "slow drift or an isolated noisy sample cannot change the frozen target");
    }
}

static void periodic_speed_ripple_still_arms_gesture_detection(void)
{
    FrictionlessHoldState state = {0};
    const FrictionlessHoldConfig config = frictionless_hold_default_config();
    frictionless_hold_step(&state, &config, 6.0f, 0.024f, 0.08f, 0.001f, false);
    frictionless_hold_step(&state, &config, 6.0f, 0.024f, 0.08f, 0.001f, true);
    for (int i = 0; i < 1000; ++i) {
        const float velocity = 6.0f + 0.8f * sinf(72.0f * (float)i * 0.001f);
        frictionless_hold_step(&state, &config, velocity, 0.024f, 0.08f, 0.001f, true);
        near(state.reference_velocity_rad_s, 6.0f, 0.0f,
             "periodic speed ripple does not move the release reference");
    }
    require(state.gesture_ready,
            "periodic cogging acceleration must not permanently disable subsequent hand gestures");
    frictionless_hold_reset(&state);
    frictionless_hold_step(&state, &config, 3.0f, 0.024f, 0.08f, 0.001f, false);
    for (int i = 0; i < 500; ++i) {
        frictionless_hold_step(&state, &config, 3.0f, 0.024f, 0.08f, 0.001f, true);
    }
    require(state.gesture_ready, "low-speed ripple test starts with intervention armed");
    for (int i = 0; i < 10000; ++i) {
        const float velocity = 3.0f + 1.5f * sinf(36.0f * (float)i * 0.001f);
        frictionless_hold_step(&state, &config, velocity, 0.024f, 0.08f, 0.001f, true);
        near(state.reference_velocity_rad_s, 3.0f, 0.0f,
             "large low-speed periodic ripple is not a repeated hand adjustment");
    }
}

static void hand_accelerates_and_slows_then_releases(void)
{
    for (int direction = -1; direction <= 1; direction += 2) {
        Plant p = plant(12.0f * (float)direction, 1.0f);
        for (int i = 0; i < 700; ++i) {
            plant_step(&p, 0.0f, 0.0f);
        }
        for (int i = 0; i < 50; ++i) {
            plant_step(&p, 0.045f * (float)direction, 0.0f);
        }
        for (int i = 0; i < 1200; ++i) {
            plant_step(&p, 0.0f, 0.0f);
        }
        const float accelerated = p.controller.reference_velocity_rad_s;
        require(fabsf(accelerated) > 14.0f, "a deliberate accelerating flick captures a new speed");
        near(p.velocity, accelerated, 0.15f, "release after acceleration holds the new speed");
        for (int i = 0; i < 50; ++i) {
            plant_step(&p, -0.040f * (float)direction, 0.0f);
        }
        for (int i = 0; i < 1800; ++i) {
            plant_step(&p, 0.0f, 0.0f);
        }
        require(fabsf(p.controller.reference_velocity_rad_s) < fabsf(accelerated) - 5.0f,
                "a deliberate slowing flick captures a slower speed");
        near(p.velocity, p.controller.reference_velocity_rad_s, 0.15f,
             "release after slowing does not restore the earlier speed");
    }
}

static void stop_reverse_and_observation_handover(void)
{
    FrictionlessHoldState state = {0};
    const FrictionlessHoldConfig config = frictionless_hold_default_config();
    for (int i = 0; i < 1000; ++i) {
        near(frictionless_hold_step(&state, &config, 0.05f, 0.03f, 0.08f, 0.001f, true),
             0.0f, 0.0f, "stationary noise has zero mean output");
    }
    frictionless_hold_step(&state, &config, 6.0f, 0.024f, 0.08f, 0.001f, true);
    near(frictionless_hold_step(&state, &config, 0.0f, 0.024f, 0.08f, 0.001f, true),
         0.0f, 0.0f, "a hand stop clears all assistance immediately");
    near(state.integral_current_a, 0.0f, 0.0f, "stop clears learned integral");
    near(state.reference_velocity_rad_s, 0.0f, 0.0f, "stop clears old release target");
    for (int i = 0; i < 30; ++i) {
        near(frictionless_hold_step(&state, &config, -3.0f, -0.024f, 0.08f, 0.001f, true),
             0.0f, 0.0f, "rebound/reversal has an output-free restart interval");
    }
    for (int i = 0; i < 30; ++i) {
        frictionless_hold_step(&state, &config, -3.0f, -0.024f, 0.08f, 0.001f, true);
    }
    near(state.reference_velocity_rad_s, -3.0f, 0.0f, "intentional reverse captures the new direction");
    for (int i = 0; i < 1000; ++i) {
        near(frictionless_hold_step(&state, &config, 15.0f, 0.03f, 0.08f, 0.001f, false),
             0.0f, 0.0f, "observation never drives the motor");
    }
    near(state.integral_current_a, 0.0f, 0.0f, "observation does not wind up the controller");
    frictionless_hold_step(&state, &config, 15.0f, 0.03f, 0.08f, 0.001f, true);
    near(state.reference_velocity_rad_s, 15.0f, 0.001f, "handover uses actual release speed, not old state");
    near(state.output_current_a, 0.03f, 2e-5f, "handover immediately supplies the known loss current");
}

static void real_launch_from_rest(void)
{
    const int durations[] = {100, 200, 300, 300, 1000};
    const float targets[] = {6.0f, 12.0f, 6.0f, 3.0f, 6.0f};
    for (int direction = -1; direction <= 1; direction += 2) {
        for (unsigned int d = 0; d < sizeof(durations) / sizeof(durations[0]); ++d) {
            Plant p = plant(0.0f, 1.0f);
            for (int i = 0; i < 300; ++i) {
                plant_step(&p, 0.0f, 0.0f);
            }
            const float target_speed = targets[d];
            const float hand_acceleration = target_speed / ((float)durations[d] * 0.001f);
            for (int i = 0; i < durations[d]; ++i) {
                const float hand_velocity = hand_acceleration * (float)(i + 1) * 0.001f *
                    (float)direction;
                const float hand_current = 0.025f * (hand_velocity - p.velocity) +
                    p.inertia_a_s2_rad * hand_acceleration * (float)direction;
                plant_step(&p, hand_current, 0.0f);
            }
            const float released = p.velocity;
            require(fabsf(released) > target_speed - 0.8f,
                    "a launch from rest produces the hand-selected motion");
            for (int i = 0; i < 1800; ++i) {
                plant_step(&p, 0.0f, 0.0f);
            }
            near(p.velocity, released, 1.0f,
                 "a 100-1000ms hand launch holds the released speed, not the start threshold");
            require(p.controller.phase == FRICTIONLESS_HOLD_TRACK,
                    "the one-off launch acquisition ends, not indefinitely available");
            const float target = p.controller.reference_velocity_rad_s;
            p.loss_a -= 0.001f;
            for (int i = 0; i < 10000; ++i) {
                plant_step(&p, 0.0f, 0.0f);
                near(p.controller.reference_velocity_rad_s, target, 0.0f,
                     "post-launch sustained bias must not repeatedly raise the release goal");
            }

            /* A hand physically stops the shaft; the measured velocity has
               the same 8ms lag as the real encoder estimator. */
            for (int i = 0; i < 150; ++i) {
                p.velocity = 0.0f;
                p.measured_velocity += (0.0f - p.measured_velocity) / 9.0f;
                frictionless_hold_step(&p.controller, &p.config, p.measured_velocity,
                    0.024f * (float)direction, p.output_limit_a, 0.001f, true);
            }
            near(p.controller.output_current_a, 0.0f, 0.0f, "holding the knob still drops all assistance");
            near(p.controller.reference_velocity_rad_s, 0.0f, 0.0f, "hand stop removes the last release goal");
            for (int i = 0; i < 150; ++i) {
                plant_step(&p, -0.032f * (float)direction, 0.0f);
            }
            for (int i = 0; i < 1500; ++i) {
                plant_step(&p, 0.0f, 0.0f);
            }
            require(p.controller.reference_velocity_rad_s * (float)direction < -6.0f,
                    "after a hand stop, a reverse launch establishes a reverse release goal");
            near(p.velocity, p.controller.reference_velocity_rad_s, 0.15f,
                 "reverse launch settles at its independently captured goal");
        }
    }
}

static void slow_hand_adjustments_yield_and_keep_new_speed(void)
{
    const float changes[] = {-6.0f, 6.0f};
    for (int direction = -1; direction <= 1; direction += 2) {
        for (unsigned int change = 0; change < sizeof(changes) / sizeof(changes[0]); ++change) {
            Plant p = plant(10.0f * (float)direction, 1.0f);
            for (int i = 0; i < 1000; ++i) {
                plant_step(&p, 0.0f, 0.0f);
            }
            const float hand_acceleration = changes[change] / 0.6f * (float)direction;
            bool yielded = false;
            for (int i = 0; i < 600; ++i) {
                const float hand_velocity = 10.0f * (float)direction +
                    hand_acceleration * (float)(i + 1) * 0.001f;
                const float hand_current = 0.025f * (hand_velocity - p.velocity) +
                    p.inertia_a_s2_rad * hand_acceleration;
                plant_step(&p, hand_current, 0.0f);
                yielded |= p.controller.phase == FRICTIONLESS_HOLD_YIELD;
            }
            const float released = p.velocity;
            for (int i = 0; i < 2200; ++i) {
                plant_step(&p, 0.0f, 0.0f);
            }
            require(yielded, "ordinary 10rad/s2 hand adjustment releases the old speed goal");
            near(p.velocity, released, 1.0f,
                 "slow hand adjustment keeps its new release speed instead of restoring the old goal");
            require(fabsf(p.controller.reference_velocity_rad_s - 10.0f * (float)direction) > 3.0f,
                    "slow adjustment makes a substantive new speed selection");
        }
    }
}

static void high_speed_reference_and_bounded_acquisition(void)
{
    const float speeds[] = {80.0f, 120.0f};
    for (int direction = -1; direction <= 1; direction += 2) {
        for (unsigned int speed = 0; speed < sizeof(speeds) / sizeof(speeds[0]); ++speed) {
            const float released = speeds[speed] * (float)direction;
            Plant p = plant(released, 1.0f);
            p.loss_a += 0.002f;
            for (int i = 0; i < 20000; ++i) {
                plant_step(&p, 0.0f, 0.0f);
                near(p.controller.reference_velocity_rad_s, released, 0.0f,
                     "high-speed release is captured without a speed ceiling and remains frozen");
            }
            near(p.velocity, released, 0.1f,
                 "both rotation directions maintain high release speeds despite loss-model error");
            require(p.controller.gesture_ready,
                    "high-speed hold can settle and rearm hand intervention");
        }
    }
    FrictionlessHoldState state = {0};
    const FrictionlessHoldConfig config = frictionless_hold_default_config();
    for (int i = 0; i < 1205; ++i) {
        /* An unmodelled constant acceleration must not extend acquisition. */
        const float velocity = 0.7f + 0.004f * (float)i;
        frictionless_hold_step(&state, &config, velocity, 0.03f, 0.08f, 0.001f, true);
    }
    require(state.phase == FRICTIONLESS_HOLD_TRACK,
            "the first acquisition has an absolute deadline even without a quiet interval");
    Plant p = plant(0.0f, 2.0f);
    for (int i = 0; i < 50; ++i) {
        plant_step(&p, 0.030f, 0.0f);
    }
    bool captured = false;
    float frozen = 0.0f;
    for (int i = 0; i < 12000; ++i) {
        plant_step(&p, 0.0f, 0.0f);
        if (!captured && p.controller.phase == FRICTIONLESS_HOLD_TRACK) {
            captured = true;
            frozen = p.controller.reference_velocity_rad_s;
        }
        if (captured) {
            near(p.controller.reference_velocity_rad_s, frozen, 0.0f,
                 "large real feedforward error cannot repeatedly reacquire targets after launch");
        }
    }
    require(captured, "biased launch still leaves acquisition and establishes a frozen target");
}

static void saturation_invalid_samples_and_reset(void)
{
    FrictionlessHoldState state = {0};
    const FrictionlessHoldConfig config = frictionless_hold_default_config();
    frictionless_hold_step(&state, &config, 10.0f, 0.03f, 0.04f, 0.001f, false);
    frictionless_hold_step(&state, &config, 10.0f, 0.03f, 0.04f, 0.001f, true);
    for (int i = 0; i < 5000; ++i) {
        float output = frictionless_hold_step(&state, &config, 3.0f, 0.03f, 0.04f, 0.001f, true);
        require(output <= 0.04f, "effective current cap is applied inside PI");
        near(state.integral_current_a, 0.0f, 0.0f, "upper saturation does not wind up integration");
    }
    frictionless_hold_step(&state, &config, 10.0f, 0.03f, 0.04f, 0.001f, true);
    near(state.output_current_a, 0.03f, 1e-7f, "saturation releases without stored excess drive");
    near(frictionless_hold_step(&state, &config, NAN, 0.03f, 0.04f, 0.001f, true),
         0.0f, 0.0f, "invalid speed yields zero output");
    require(state.phase == FRICTIONLESS_HOLD_REST && !state.velocity_initialized,
            "invalid sample fully resets controller state");
    frictionless_hold_step(&state, &config, 10.0f, 0.03f, 0.04f, 0.001f, true);
    near(frictionless_hold_step(&state, &config, 10.0f, 0.03f, 0.04f, 0.1f, true),
         0.0f, 0.0f, "scheduler gap invalidates stored speed and integral");
    frictionless_hold_step(&state, &config, 10.0f, 0.03f, 0.04f, 0.001f, true);
    near(frictionless_hold_step(&state, &config, 10.0f, 0.03f, 0.0f, 0.001f, true),
         0.0f, 0.0f, "zero current budget clears the control state");
    frictionless_hold_reset(&state);
    require(state.phase == FRICTIONLESS_HOLD_REST && state.direction == 0,
            "explicit reset removes every control and detector memory");
}

int main(void)
{
    fixed_reference_cancels_model_error();
    exaggerated_feedforward_can_be_fully_cancelled();
    slow_drift_and_short_impulses_do_not_rebase();
    periodic_speed_ripple_still_arms_gesture_detection();
    hand_accelerates_and_slows_then_releases();
    stop_reverse_and_observation_handover();
    real_launch_from_rest();
    slow_hand_adjustments_yield_and_keep_new_speed();
    high_speed_reference_and_bounded_acquisition();
    saturation_invalid_samples_and_reset();
    puts("frictionless_hold tests passed (closed-loop model and state transitions)");
    return EXIT_SUCCESS;
}
