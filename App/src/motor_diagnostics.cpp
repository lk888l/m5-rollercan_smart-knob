/* Optional SWD mailbox: all actuator writes happen in ControlTask, never in
   a debugger-injected function call. Excluded from normal firmware builds. */
#include "motor_diagnostics.h"

#include <cmath>
#include <cstddef>
#include <cstdint>

extern "C" {
#include "mysys.h"
#include "motordriver.h"
#include "smart_knob.h"
#include "smart_knob_modes.h"
extern float ph_current_rt;
extern float uq;
}

struct MotorDebugSample {
    uint32_t elapsed_ms;
    float angle_rad;
    float velocity_rad_s;
    float slow_velocity_rad_s;
    float command_ma;
    float measured_ma;
    float uq_v;
    float mean_command_ma;
    float mean_velocity_rad_s;
    float mean_filtered_command_ma;
    float mean_filtered_sin12;
    float mean_filtered_cos12;
};

struct MotorDebugDenseSample {
    uint32_t elapsed_ms;
    float angle_rad;
    float velocity_rad_s;
    float filtered_command_ma;
};

union MotorDebugSamples {
    MotorDebugSample normal[128];
    MotorDebugDenseSample dense[384];
};

struct MotorDebugMailbox {
    uint32_t magic;
    uint32_t request;       // Commit last: 1=start, 2=abort, 3=release, 4=save preset, 5=save preset limits.
    uint32_t status;        // 0=idle, 1=running, 2=done, 3=aborted, 4=rejected, 5=saving.
    uint32_t mode;          // UINT32_MAX=zero-output coast, otherwise preset.
    uint32_t pulse_ms;
    uint32_t duration_ms;   // Includes a 350 ms settling phase.
    float pulse_ma;
    uint32_t count;
    uint32_t period_ms;
    float friction_ma;      // Negative: use preset defaults; temporary override.
    float viscous_microamps;
    float pulse_velocity_rad_s; // Nonzero: velocity PI during the bounded pulse.
    float compensation_scale;   // Negative: preset; temporary override.
    float cogging_scale;
    uint32_t dense;             // 1: final 384 ms at 1 kHz; shares the normal buffer.
    float inertia_gain;         // Negative: preset; extra rotor inertia multiples.
    float inertia_tau_ms;       // Negative: preset; velocity-memory filter time.
    MotorDebugSamples samples;
};
static_assert(offsetof(MotorDebugMailbox, dense) == 56U &&
              offsetof(MotorDebugMailbox, samples) == 68U &&
              sizeof(MotorDebugSample) == 48U &&
              sizeof(MotorDebugDenseSample) == 16U &&
              sizeof(MotorDebugSamples) == 6144U &&
              sizeof(MotorDebugMailbox) == 6212U,
              "Update the diagnostic ABI and host script when the layout changes");

extern "C" {
volatile MotorDebugMailbox motor_debug = {0x4B4E4406U}; // Versioned diagnostic ABI.
}

namespace {
bool owns_output;
bool saving;
uint32_t started_ms;
uint32_t next_sample_ms;
uint32_t mode;
uint32_t pulse_ms;
uint32_t duration_ms;
float pulse_ma;
float pulse_velocity;
float velocity_integral_ma;
float command_sum_ma;
float velocity_sum_rad_s;
uint32_t window_count;
float filtered_command_ma;
float filtered_sin12;
float filtered_cos12;
float filtered_command_sum;
float filtered_sin_sum;
float filtered_cos_sum;

void Stop(uint32_t status)
{
    MotorDriverSetCurrentRealContinuous(0.0f);
    MotorDriverSetMode(MDRV_MODE_OFF);
    __DMB();
    motor_debug.status = status;
}
}

extern "C" bool MotorDiagnosticsStep(void)
{
    const uint32_t now = HAL_GetTick();
    const uint32_t request = motor_debug.request;
    if (request != 0U) {
        __DMB();
        motor_debug.request = 0U;
        if ((request == 4U || request == 5U) && !owns_output && !saving &&
            motor_debug.mode < smart_knob_modes_count() &&
            !MysysHostConnected() && error_code == 0U && over_vol_flag == 0U) {
            const uint8_t selected = static_cast<uint8_t>(motor_debug.mode);
            /* Saving is explicit and uses the same transaction as OLED SAVE.
               Request 5 updates limits while retaining the user's feel tuning. */
            SmartKnobModeConfig *selected_config = smart_knob_mode_get_mutable(selected);
            const SmartKnobModeConfig *defaults = smart_knob_mode_get_default(selected);
            if (request == 4U) {
                *selected_config = *defaults;
            } else {
                selected_config->tuning.current_limit_a = defaults->tuning.current_limit_a;
                selected_config->tuning.max_current_permille = defaults->tuning.max_current_permille;
            }
            LocalProfileEdit edit{};
            (void)smart_knob_mode_local_projection(selected, &edit, nullptr);
            edit.dirty_mask = LOCAL_PROFILE_DIRTY_MODE;
            MysysLocalMenuEnter();
            if (smart_knob_active_mode() != SMART_KNOB_NAVIGATION_MODE) {
                motor_debug.status = 4U;
                return false;
            }
            MysysLocalMenuExit(&edit);
            motor_debug.count = 0U;
            motor_debug.status = 5U;
            saving = true;
            return false;
        }
        if (request == 3U) {
            Stop(0U);
            owns_output = false;
            init_smart_knob();
            return false;
        }
        owns_output = true;
        if (request != 1U) {
            Stop(3U);
            return true;
        }
        mode = motor_debug.mode;
        pulse_ms = motor_debug.pulse_ms;
        duration_ms = motor_debug.duration_ms;
        pulse_ma = motor_debug.pulse_ma;
        pulse_velocity = motor_debug.pulse_velocity_rad_s;
        velocity_integral_ma = 0.0f;
        if (!std::isfinite(pulse_ma) || std::fabs(pulse_ma) > 250.0f ||
            !std::isfinite(pulse_velocity) || std::fabs(pulse_velocity) > 60.0f ||
            motor_debug.dense > 1U || (motor_debug.dense != 0U && pulse_ms == 0U) ||
            pulse_ms > 5000U || duration_ms < 400U || duration_ms > 30000U ||
            pulse_ms + 350U >= duration_ms || motor_mode != MODE_DIAL ||
            error_code != 0U || over_vol_flag != 0U ||
            (mode < UINT32_MAX - 2U && mode >= smart_knob_modes_count())) {
            Stop(4U);
            return true;
        }
        if (mode < UINT32_MAX - 2U) {
            (void)smart_knob_select_mode(static_cast<uint8_t>(mode));
            if (mode == SMART_KNOB_MODE_FRICTIONLESS) {
                smart_knob_set_test_cogging_scale(motor_debug.cogging_scale);
                SmartKnobModeConfig *config = smart_knob_mode_get_mutable(static_cast<uint8_t>(mode));
                *config = *smart_knob_mode_get_default(static_cast<uint8_t>(mode));
                if (std::isfinite(motor_debug.inertia_gain) && motor_debug.inertia_gain >= 0.0f) {
                    config->tuning.p_gain = std::fmin(motor_debug.inertia_gain, 8.0f);
                }
                if (std::isfinite(motor_debug.inertia_tau_ms) && motor_debug.inertia_tau_ms >= 0.0f) {
                    config->tuning.d_gain = std::fmax(5.0f, std::fmin(motor_debug.inertia_tau_ms, 200.0f)) * 0.001f;
                }
                if (std::isfinite(motor_debug.compensation_scale) && motor_debug.compensation_scale >= 0.0f) {
                    config->tuning.current_scale_a = std::fmin(motor_debug.compensation_scale, 10.0f);
                }
                if (std::isfinite(motor_debug.friction_ma) && motor_debug.friction_ma >= 0.0f) {
                    config->tuning.friction_current_a = std::fmin(motor_debug.friction_ma, 80.0f) / 1000.0f;
                }
                if (std::isfinite(motor_debug.viscous_microamps) && motor_debug.viscous_microamps >= 0.0f) {
                    config->tuning.viscous_friction_a_per_rad_s = std::fmin(motor_debug.viscous_microamps, 10000.0f) / 1000000.0f;
                }
            }
        }
        init_smart_knob();
        MotorDriverSetCurrentRealContinuous(0.0f);
        MotorDriverSetMode(MDRV_MODE_RUN);
        started_ms = now;
        next_sample_ms = motor_debug.dense != 0U ? duration_ms - 384U : 0U;
        command_sum_ma = 0.0f;
        velocity_sum_rad_s = 0.0f;
        window_count = 0U;
        filtered_command_ma = 0.0f;
        filtered_sin12 = 0.0f;
        filtered_cos12 = 0.0f;
        filtered_command_sum = 0.0f;
        filtered_sin_sum = 0.0f;
        filtered_cos_sum = 0.0f;
        motor_debug.count = 0U;
        motor_debug.period_ms = motor_debug.dense != 0U ? 1U : (duration_ms + 127U) / 128U;
        motor_debug.status = 1U;
    }
    if (!owns_output) {
        if (saving && !MysysLocalSavePending()) {
            motor_debug.status = MysysLocalSaveError() ? 4U : 2U;
            saving = false;
        }
        return false;
    }
    if (motor_debug.status != 1U) {
        return true;
    }
    const uint32_t elapsed = now - started_ms;
    if (error_code != 0U || over_vol_flag != 0U ||
        /* Only this timed diagnostic stops at 200 rad/s. The production
           frictionless controller has no speed-based derating or cutoff. */
        !std::isfinite(motor_rps_fast) || std::fabs(motor_rps_fast) > 200.0f) {
        Stop(3U);
        return true;
    }
    if (elapsed >= duration_ms) {
        Stop(2U);
        return true;
    }
    float command_ma = 0.0f;
    if (mode < smart_knob_modes_count() && elapsed < 350U + pulse_ms) {
        /* Track detent transitions during the simulated hand motion, so
           release starts from the same state as a real manual flick. */
        smart_knob_observe_motion();
    }
    if (elapsed >= 350U && elapsed < 350U + pulse_ms) {
        command_ma = pulse_ma;
        if (pulse_velocity != 0.0f) {
            const float ramp = std::fmin(static_cast<float>(elapsed - 350U) / 200.0f, 1.0f);
            const float error = pulse_velocity * ramp - motor_rps_fast;
            velocity_integral_ma = std::fmax(-100.0f, std::fmin(100.0f,
                velocity_integral_ma + 40.0f * error * 0.001f));
            command_ma = std::fmax(-150.0f, std::fmin(150.0f, 24.0f * error + velocity_integral_ma));
        }
        MotorDriverSetCurrentRealContinuous(command_ma);
    } else if (elapsed >= 350U + pulse_ms && mode == UINT32_MAX - 1U) {
        MotorDriverSetTorqueCurrent(0.0f);
    } else if (elapsed >= 350U + pulse_ms && mode == UINT32_MAX - 2U) {
        if (MotorDriverIsOutputEnabled()) MotorDriverSetMode(MDRV_MODE_OFF);
        MotorDriverSetCurrentRealContinuous(0.0f);
    } else if (elapsed >= 350U + pulse_ms && mode != UINT32_MAX) {
        handle_smart_knob();
        SmartKnobRuntimeState state{};
        if (smart_knob_get_runtime_state(&state)) {
            command_ma = state.commanded_current_ma;
        }
    } else {
        MotorDriverSetCurrentRealContinuous(0.0f);
    }
    command_sum_ma += command_ma;
    velocity_sum_rad_s += motor_rps_fast;
    /* Filter every term of the identification equation with the same 8 ms
       pole as velocity. This preserves the torque/acceleration phase relation
       when separating inertia from angle-dependent disturbance. */
    constexpr float filter_alpha = 1.0f / 9.0f;
    filtered_command_ma += filter_alpha * (command_ma - filtered_command_ma);
    filtered_sin12 += filter_alpha * (std::sin(12.0f * mechanical_rad) - filtered_sin12);
    filtered_cos12 += filter_alpha * (std::cos(12.0f * mechanical_rad) - filtered_cos12);
    filtered_command_sum += filtered_command_ma;
    filtered_sin_sum += filtered_sin12;
    filtered_cos_sum += filtered_cos12;
    ++window_count;
    if (motor_debug.dense != 0U && elapsed >= next_sample_ms && motor_debug.count < 384U) {
        /* The shared buffer holds only the final 384 ms of the bounded test.
           Sampling is independent of PI/release phase; the analysis selects
           intervals wholly inside PI and uses the matched 8 ms signals. */
        volatile MotorDebugDenseSample &sample = motor_debug.samples.dense[motor_debug.count];
        sample.elapsed_ms = elapsed;
        sample.angle_rad = mechanical_rad;
        sample.velocity_rad_s = motor_rps_fast;
        sample.filtered_command_ma = filtered_command_ma;
        __DMB();
        ++motor_debug.count;
        next_sample_ms = elapsed + 1U;
    } else if (motor_debug.dense == 0U && elapsed >= next_sample_ms && motor_debug.count < 128U) {
        volatile MotorDebugSample &sample = motor_debug.samples.normal[motor_debug.count];
        sample.elapsed_ms = elapsed;
        sample.angle_rad = mechanical_rad;
        sample.velocity_rad_s = motor_rps_fast;
        sample.slow_velocity_rad_s = motor_rps;
        sample.command_ma = command_ma;
        sample.measured_ma = ph_current_rt;
        sample.uq_v = uq;
        sample.mean_command_ma = command_sum_ma / static_cast<float>(window_count);
        sample.mean_velocity_rad_s = velocity_sum_rad_s / static_cast<float>(window_count);
        sample.mean_filtered_command_ma = filtered_command_sum / static_cast<float>(window_count);
        sample.mean_filtered_sin12 = filtered_sin_sum / static_cast<float>(window_count);
        sample.mean_filtered_cos12 = filtered_cos_sum / static_cast<float>(window_count);
        filtered_command_sum = 0.0f;
        filtered_sin_sum = 0.0f;
        filtered_cos_sum = 0.0f;
        command_sum_ma = 0.0f;
        velocity_sum_rad_s = 0.0f;
        window_count = 0U;
        __DMB();
        ++motor_debug.count;
        next_sample_ms = elapsed + motor_debug.period_ms;
    }
    return true;
}
